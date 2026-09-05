// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple SMC (System Management Controller) MFD driver
 *
 * Copyright The Asahi Linux Contributors
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/math.h>
#include <linux/mfd/core.h>
#include <linux/mfd/macsmc.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/unaligned.h>

#define SMC_ENDPOINT			0x20

/* We don't actually know the true size here but this seem reasonable */
#define SMC_SHMEM_SIZE			0x1000
#define SMC_MAX_SIZE			255

#define SMC_MSG_READ_KEY		0x10
#define SMC_MSG_WRITE_KEY		0x11
#define SMC_MSG_GET_KEY_BY_INDEX	0x12
#define SMC_MSG_GET_KEY_INFO		0x13
#define SMC_MSG_INITIALIZE		0x17
#define SMC_MSG_NOTIFICATION		0x18
#define SMC_MSG_RW_KEY			0x20

#define SMC_DATA			GENMASK_ULL(63, 32)
#define SMC_WSIZE			GENMASK_ULL(31, 24)
#define SMC_SIZE			GENMASK_ULL(23, 16)
#define SMC_ID				GENMASK_ULL(15, 12)
#define SMC_MSG				GENMASK_ULL(7, 0)
#define SMC_RESULT			SMC_MSG

#define SMC_TIMEOUT_MS		500

static const struct mfd_cell apple_smc_devs[] = {
	MFD_CELL_NAME("macsmc-input"),
	MFD_CELL_NAME("macsmc-power"),
	MFD_CELL_OF("macsmc-gpio", NULL, NULL, 0, 0, "apple,smc-gpio"),
	MFD_CELL_OF("macsmc-low-gpio", NULL, NULL, 0, 0, "apple,smc-low-gpio"),
	MFD_CELL_OF("macsmc-hwmon", NULL, NULL, 0, 0, "apple,smc-hwmon"),
	MFD_CELL_OF("macsmc-reboot", NULL, NULL, 0, 0, "apple,smc-reboot"),
	MFD_CELL_OF("macsmc-rtc", NULL, NULL, 0, 0, "apple,smc-rtc"),
};

/* J700 diagnostic gate: expose only the established gPxx GPIO bank. */
static const struct mfd_cell apple_smc_gpio_devs[] = {
	MFD_CELL_OF("macsmc-gpio", NULL, NULL, 0, 0, "apple,smc-gpio"),
};

static int apple_smc_cmd_locked(struct apple_smc *smc, u64 cmd, u64 arg,
				  u64 size, u64 wsize, u32 *ret_data)
{
	u8 result;
	int ret;
	u64 msg;

	lockdep_assert_held(&smc->mutex);

	if (smc->boot_stage != APPLE_SMC_INITIALIZED)
		return -EIO;
	if (smc->atomic_mode)
		return -EIO;

	reinit_completion(&smc->cmd_done);

	smc->msg_id = (smc->msg_id + 1) & 0xf;
	msg = (FIELD_PREP(SMC_MSG, cmd) |
	       FIELD_PREP(SMC_SIZE, size) |
	       FIELD_PREP(SMC_WSIZE, wsize) |
	       FIELD_PREP(SMC_ID, smc->msg_id) |
	       FIELD_PREP(SMC_DATA, arg));

	ret = apple_rtkit_send_message(smc->rtk, SMC_ENDPOINT, msg, NULL, false);
	if (ret) {
		dev_err(smc->dev, "Failed to send command\n");
		return ret;
	}

	if (wait_for_completion_timeout(&smc->cmd_done, msecs_to_jiffies(SMC_TIMEOUT_MS)) <= 0) {
		dev_err(smc->dev, "Command timed out (%llx)", msg);
		return -ETIMEDOUT;
	}

	if (FIELD_GET(SMC_ID, smc->cmd_ret) != smc->msg_id) {
		dev_err(smc->dev, "Command sequence mismatch (expected %d, got %d)\n",
			smc->msg_id, (unsigned int)FIELD_GET(SMC_ID, smc->cmd_ret));
		return -EIO;
	}

	result = FIELD_GET(SMC_RESULT, smc->cmd_ret);
	if (result)
		return -EIO;

	if (ret_data)
		*ret_data = FIELD_GET(SMC_DATA, smc->cmd_ret);

	return FIELD_GET(SMC_SIZE, smc->cmd_ret);
}

static int apple_smc_cmd(struct apple_smc *smc, u64 cmd, u64 arg,
			 u64 size, u64 wsize, u32 *ret_data)
{
	guard(mutex)(&smc->mutex);

	return apple_smc_cmd_locked(smc, cmd, arg, size, wsize, ret_data);
}

static int apple_smc_rw_locked(struct apple_smc *smc, smc_key key,
				const void *wbuf, size_t wsize,
				void *rbuf, size_t rsize)
{
	u64 smc_size, smc_wsize;
	u32 rdata;
	int ret;
	u64 cmd;

	lockdep_assert_held(&smc->mutex);

	if (rsize > SMC_MAX_SIZE)
		return -EINVAL;
	if (wsize > SMC_MAX_SIZE)
		return -EINVAL;

	if (rsize && wsize) {
		cmd = SMC_MSG_RW_KEY;
		memcpy_toio(smc->shmem.iomem, wbuf, wsize);
		smc_size = rsize;
		smc_wsize = wsize;
	} else if (wsize && !rsize) {
		cmd = SMC_MSG_WRITE_KEY;
		memcpy_toio(smc->shmem.iomem, wbuf, wsize);
		/*
		 * Setting size to the length we want to write and wsize to 0
		 * looks silly but that's how the SMC protocol works ¯\_(ツ)_/¯
		 */
		smc_size = wsize;
		smc_wsize = 0;
	} else if (!wsize && rsize) {
		cmd = SMC_MSG_READ_KEY;
		smc_size = rsize;
		smc_wsize = 0;
	} else {
		return -EINVAL;
	}

	ret = apple_smc_cmd_locked(smc, cmd, key, smc_size, smc_wsize, &rdata);
	if (ret < 0)
		return ret;

	if (rsize) {
		/*
		 * Small data <= 4 bytes is returned as part of the reply
		 * message which is sent over the mailbox FIFO. Everything
		 * bigger has to be copied from SRAM which is mapped as
		 * Device memory.
		 */
		if (rsize <= 4)
			memcpy(rbuf, &rdata, rsize);
		else
			memcpy_fromio(rbuf, smc->shmem.iomem, rsize);
	}

	return ret;
}

int apple_smc_read(struct apple_smc *smc, smc_key key, void *buf, size_t size)
{
	guard(mutex)(&smc->mutex);

	return apple_smc_rw_locked(smc, key, NULL, 0, buf, size);
}
EXPORT_SYMBOL(apple_smc_read);

int apple_smc_write(struct apple_smc *smc, smc_key key, const void *buf, size_t size)
{
	guard(mutex)(&smc->mutex);

	return apple_smc_rw_locked(smc, key, buf, size, NULL, 0);
}
EXPORT_SYMBOL(apple_smc_write);

int apple_smc_rw(struct apple_smc *smc, smc_key key, const void *wbuf, size_t wsize,
		 void *rbuf, size_t rsize)
{
	guard(mutex)(&smc->mutex);

	return apple_smc_rw_locked(smc, key, wbuf, wsize, rbuf, rsize);
}
EXPORT_SYMBOL(apple_smc_rw);

int apple_smc_get_key_by_index(struct apple_smc *smc, int index, smc_key *key)
{
	int ret;

	ret = apple_smc_cmd(smc, SMC_MSG_GET_KEY_BY_INDEX, index, 0, 0, key);

	*key = swab32(*key);
	return ret;
}
EXPORT_SYMBOL(apple_smc_get_key_by_index);

int apple_smc_get_key_info(struct apple_smc *smc, smc_key key, struct apple_smc_key_info *info)
{
	u8 key_info[6];
	int ret;

	ret = apple_smc_cmd(smc, SMC_MSG_GET_KEY_INFO, key, 0, 0, NULL);
	if (ret >= 0 && info) {
		memcpy_fromio(key_info, smc->shmem.iomem, sizeof(key_info));
		info->size = key_info[0];
		info->type_code = get_unaligned_be32(&key_info[1]);
		info->flags = key_info[5];
	}
	return ret;
}
EXPORT_SYMBOL(apple_smc_get_key_info);

int apple_smc_enter_atomic(struct apple_smc *smc)
{
	guard(mutex)(&smc->mutex);

	/*
	 * Disable notifications since this is called before shutdown and no
	 * notification handler will be able to handle the notification
	 * using atomic operations only. Also ignore any failure here
	 * because we're about to shut down or reboot anyway.
	 * We can't use apple_smc_write_flag here since that would try to lock
	 * smc->mutex again.
	 */
	const u8 flag = 0;

	apple_smc_rw_locked(smc, SMC_KEY(NTAP), &flag, sizeof(flag), NULL, 0);

	smc->atomic_mode = true;

	return 0;
}
EXPORT_SYMBOL(apple_smc_enter_atomic);

int apple_smc_write_atomic(struct apple_smc *smc, smc_key key, const void *buf, size_t size)
{
	guard(spinlock_irqsave)(&smc->lock);
	u8 result;
	int ret;
	u64 msg;

	if (size > SMC_MAX_SIZE || size == 0)
		return -EINVAL;

	if (smc->boot_stage != APPLE_SMC_INITIALIZED)
		return -EIO;
	if (!smc->atomic_mode)
		return -EIO;

	memcpy_toio(smc->shmem.iomem, buf, size);
	smc->msg_id = (smc->msg_id + 1) & 0xf;
	msg = (FIELD_PREP(SMC_MSG, SMC_MSG_WRITE_KEY) |
	       FIELD_PREP(SMC_SIZE, size) |
	       FIELD_PREP(SMC_ID, smc->msg_id) |
	       FIELD_PREP(SMC_DATA, key));
	smc->atomic_pending = true;

	ret = apple_rtkit_send_message(smc->rtk, SMC_ENDPOINT, msg, NULL, true);
	if (ret < 0) {
		dev_err(smc->dev, "Failed to send command (%d)\n", ret);
		return ret;
	}

	while (smc->atomic_pending) {
		ret = apple_rtkit_poll(smc->rtk);
		if (ret < 0) {
			dev_err(smc->dev, "RTKit poll failed (%llx)", msg);
			return ret;
		}
		udelay(100);
	}

	if (FIELD_GET(SMC_ID, smc->cmd_ret) != smc->msg_id) {
		dev_err(smc->dev, "Command sequence mismatch (expected %d, got %d)\n",
			smc->msg_id, (unsigned int)FIELD_GET(SMC_ID, smc->cmd_ret));
		return -EIO;
	}

	result = FIELD_GET(SMC_RESULT, smc->cmd_ret);
	if (result)
		return -EIO;

	return FIELD_GET(SMC_SIZE, smc->cmd_ret);
}
EXPORT_SYMBOL(apple_smc_write_atomic);

static void apple_smc_rtkit_crashed(void *cookie, const void *bfr, size_t bfr_len)
{
	struct apple_smc *smc = cookie;

	smc->boot_stage = APPLE_SMC_ERROR_CRASHED;
	dev_err(smc->dev, "SMC crashed! Your system will reboot in a few seconds...\n");
}

static int apple_smc_rtkit_shmem_setup(void *cookie, struct apple_rtkit_shmem *bfr)
{
	struct apple_smc *smc = cookie;
	size_t bfr_end;

	if (!bfr->iova) {
		dev_err(smc->dev, "RTKit wants a RAM buffer\n");
		return -EIO;
	}

	if (check_add_overflow(bfr->iova, bfr->size - 1, &bfr_end))
		return -EFAULT;

	if (bfr->iova < smc->sram->start || bfr->iova > smc->sram->end ||
	    bfr_end > smc->sram->end) {
		dev_err(smc->dev, "RTKit buffer request outside SRAM region: [0x%llx, 0x%llx]\n",
			(unsigned long long)bfr->iova,
			(unsigned long long)bfr_end);
		return -EFAULT;
	}

	bfr->iomem = smc->sram_base + (bfr->iova - smc->sram->start);
	bfr->is_mapped = true;

	return 0;
}

static bool apple_smc_rtkit_recv_early(void *cookie, u8 endpoint, u64 message)
{
	struct apple_smc *smc = cookie;

	if (endpoint != SMC_ENDPOINT) {
		dev_warn(smc->dev, "Received message for unknown endpoint 0x%x\n", endpoint);
		return false;
	}

	if (smc->boot_stage == APPLE_SMC_BOOTING) {
		int ret;

		smc->shmem.iova = message;
		smc->shmem.size = SMC_SHMEM_SIZE;
		ret = apple_smc_rtkit_shmem_setup(smc, &smc->shmem);
		if (ret < 0) {
			smc->boot_stage = APPLE_SMC_ERROR_NO_SHMEM;
			dev_err(smc->dev, "Failed to initialize shared memory (%d)\n", ret);
		} else {
			smc->boot_stage = APPLE_SMC_INITIALIZED;
		}
		complete(&smc->init_done);
	} else if (FIELD_GET(SMC_MSG, message) == SMC_MSG_NOTIFICATION) {
		/* Handle these in the RTKit worker thread */
		return false;
	} else {
		smc->cmd_ret = message;
		if (smc->atomic_pending)
			smc->atomic_pending = false;
		else
			complete(&smc->cmd_done);
	}

	return true;
}

static void apple_smc_rtkit_recv(void *cookie, u8 endpoint, u64 message)
{
	struct apple_smc *smc = cookie;

	if (endpoint != SMC_ENDPOINT) {
		dev_warn(smc->dev, "Received message for unknown endpoint 0x%x\n", endpoint);
		return;
	}

	if (FIELD_GET(SMC_MSG, message) != SMC_MSG_NOTIFICATION) {
		dev_warn(smc->dev, "Received unknown message from worker: 0x%llx\n", message);
		return;
	}

	blocking_notifier_call_chain(&smc->event_handlers, FIELD_GET(SMC_DATA, message), NULL);
}

static const struct apple_rtkit_ops apple_smc_rtkit_ops = {
	.crashed = apple_smc_rtkit_crashed,
	.recv_message = apple_smc_rtkit_recv,
	.recv_message_early = apple_smc_rtkit_recv_early,
	.shmem_setup = apple_smc_rtkit_shmem_setup,
};

static void apple_smc_rtkit_shutdown(void *data)
{
	struct apple_smc *smc = data;

	/* Shut down SMC firmware, if it's not completely wedged */
	if (apple_rtkit_is_running(smc->rtk))
		apple_rtkit_quiesce(smc->rtk);
}

static void apple_smc_disable_notifications(void *data)
{
	struct apple_smc *smc = data;

	apple_smc_write_flag(smc, SMC_KEY(NTAP), false);
}

static int apple_smc_j700_log_function_key(struct apple_smc *smc,
					    smc_key key, const char *name)
{
	struct apple_smc_key_info info;
	int ret;

	ret = apple_smc_get_key_info(smc, key, &info);
	if (ret < 0)
		return dev_err_probe(smc->dev, ret,
				     "Failed to read J700 %s key metadata\n", name);

	dev_info(smc->dev,
		 "J700_SMC_FUNCTION_KEY: name=%s key=%08x size=%u type=%08x flags=%02x readable=%u writable=%u function=%u\n",
		 name, key, info.size, info.type_code, info.flags,
		 !!(info.flags & APPLE_SMC_READABLE),
		 !!(info.flags & APPLE_SMC_WRITABLE),
		 !!(info.flags & APPLE_SMC_FUNCTION));

	return 0;
}

/* Existing macsmc-power read keys, excluding identity and charge controls. */
static int apple_smc_j700_log_power(struct apple_smc *smc)
{
	static const char * const keys[] = {
		"B0AV", "B0AC", "B0AP", "BUIC", "B0AT", "B0CT", "ACPW",
	};
	struct apple_smc_key_info info;
	u8 value[8];
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(keys); i++) {
		ret = apple_smc_get_key_info(smc, _SMC_KEY(keys[i]), &info);
		if (ret < 0)
			return ret;
		if (!(info.flags & APPLE_SMC_READABLE) ||
		    (info.flags & APPLE_SMC_FUNCTION) ||
		    !info.size || info.size > sizeof(value))
			return -EINVAL;

		ret = apple_smc_read(smc, _SMC_KEY(keys[i]), value, info.size);
		if (ret < 0)
			return ret;
		if (ret != info.size)
			return -EIO;
		dev_info(smc->dev,
			 "J700_SMC_POWER_KEY: key=%s type=%08x size=%u raw=%*ph\n",
			 keys[i], info.type_code, info.size, info.size, value);
	}
	return 0;
}

static int apple_smc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_smc *smc;
	bool j700_gpio_only;
	bool j700_read_only;
	__be32 count;
	int ret;

	j700_read_only = of_property_read_bool(dev->of_node,
					       "apple,j700-read-only-diagnostics");
	j700_gpio_only = of_property_read_bool(dev->of_node,
					      "apple,j700-gpio-only");
	if (j700_read_only && j700_gpio_only)
		return dev_err_probe(dev, -EINVAL,
				     "J700 SMC diagnostic modes are mutually exclusive\n");

	smc = devm_kzalloc(dev, sizeof(*smc), GFP_KERNEL);
	if (!smc)
		return -ENOMEM;

	mutex_init(&smc->mutex);
	smc->dev = &pdev->dev;
	smc->sram_base = devm_platform_get_and_ioremap_resource(pdev, 1, &smc->sram);
	if (IS_ERR(smc->sram_base))
		return dev_err_probe(dev, PTR_ERR(smc->sram_base), "Failed to map SRAM region");

	smc->rtk = devm_apple_rtkit_init(dev, smc, NULL, 0, &apple_smc_rtkit_ops);
	if (IS_ERR(smc->rtk))
		return dev_err_probe(dev, PTR_ERR(smc->rtk), "Failed to initialize RTKit");

	smc->boot_stage = APPLE_SMC_BOOTING;
	ret = apple_rtkit_wake(smc->rtk);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to wake up SMC");

	ret = devm_add_action_or_reset(dev, apple_smc_rtkit_shutdown, smc);
	if (ret)
		return ret;

	ret = apple_rtkit_start_ep(smc->rtk, SMC_ENDPOINT);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to start SMC endpoint");

	init_completion(&smc->init_done);
	init_completion(&smc->cmd_done);

	ret = apple_rtkit_send_message(smc->rtk, SMC_ENDPOINT,
				       FIELD_PREP(SMC_MSG, SMC_MSG_INITIALIZE), NULL, false);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to send init message");

	if (wait_for_completion_timeout(&smc->init_done, msecs_to_jiffies(SMC_TIMEOUT_MS)) == 0) {
		dev_err(dev, "Timed out initializing SMC");
		return -ETIMEDOUT;
	}

	if (smc->boot_stage != APPLE_SMC_INITIALIZED) {
		dev_err(dev, "SMC failed to boot successfully, boot stage=%d\n", smc->boot_stage);
		return -EIO;
	}

	dev_set_drvdata(&pdev->dev, smc);
	BLOCKING_INIT_NOTIFIER_HEAD(&smc->event_handlers);

	ret = apple_smc_read(smc, SMC_KEY(#KEY), &count, sizeof(count));
	if (ret >= 0 && ret != sizeof(count))
		ret = -EINVAL;
	if (ret < 0)
		return dev_err_probe(smc->dev, ret, "Failed to get key count");
	smc->key_count = be32_to_cpu(count);

	/*
	 * J700's shipping multi-touch contract names two SMC functions, but the
	 * first hardware gate must not execute them.  Probe only their key
	 * metadata and deliberately skip NTAP plus every MFD child, since both
	 * paths can issue unrelated writes.  This remains opt-in and has no effect
	 * on the normal macsmc path or any existing platform DT.
	 */
	if (j700_read_only || j700_gpio_only) {
		ret = apple_smc_j700_log_function_key(smc, SMC_KEY(pcIO),
						      "pcIO");
		if (ret)
			return ret;
		ret = apple_smc_j700_log_function_key(smc, SMC_KEY(pmIP),
						      "pmIP");
		if (ret)
			return ret;

		if (j700_read_only) {
			ret = apple_smc_j700_log_power(smc);
			if (ret)
				return dev_err_probe(dev, ret,
						     "J700 SMC power-key probe failed\n");
			dev_info(smc->dev,
				 "J700_SMC_READ_ONLY_READY: keys=%u key_writes=0 notifications=0 children=0\n",
				 smc->key_count);
			return 0;
		}

		/*
		 * A J700 pKW8 function with a 0x80000 mask was proven on the
		 * machine through existing gP08.  The AFE function's 0x10000 mask
		 * therefore maps to gP01.  Register only that established GPIO
		 * provider and leave notifications plus unrelated children off.
		 */
		ret = devm_mfd_add_devices(smc->dev, PLATFORM_DEVID_NONE,
					   apple_smc_gpio_devs,
					   ARRAY_SIZE(apple_smc_gpio_devs),
					   NULL, 0, NULL);
		if (ret)
			return dev_err_probe(smc->dev, ret,
					     "Failed to register J700 SMC GPIO provider");

		dev_info(smc->dev,
			 "J700_SMC_GPIO_ONLY_READY: keys=%u notifications=0 children=macsmc-gpio\n",
			 smc->key_count);
		return 0;
	}

	/* Enable notifications */
	apple_smc_write_flag(smc, SMC_KEY(NTAP), true);
	ret = devm_add_action_or_reset(dev, apple_smc_disable_notifications, smc);
	if (ret)
		return ret;

	ret = devm_mfd_add_devices(smc->dev, PLATFORM_DEVID_NONE,
				   apple_smc_devs, ARRAY_SIZE(apple_smc_devs),
				   NULL, 0, NULL);
	if (ret)
		return dev_err_probe(smc->dev, ret, "Failed to register sub-devices");


	return 0;
}

static const struct of_device_id apple_smc_of_match[] = {
	{ .compatible = "apple,t8103-smc" },
	{ .compatible = "apple,smc" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_smc_of_match);

static struct platform_driver apple_smc_driver = {
	.driver = {
		.name = "macsmc",
		.of_match_table = apple_smc_of_match,
	},
	.probe = apple_smc_probe,
};
module_platform_driver(apple_smc_driver);

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_AUTHOR("Sven Peter <sven@kernel.org>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple SMC driver");
