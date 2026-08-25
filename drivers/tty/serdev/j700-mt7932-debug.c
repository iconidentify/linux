// SPDX-License-Identifier: GPL-2.0
/* Receive-only J700 MT7932 firmware debug-UART capture. */

#include <linux/completion.h>
#include <linux/device.h>
#include <linux/j700-mt7932-debug.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define J700_MT7932_DEBUG_BAUD		1843200
#define J700_MT7932_DEBUG_BYTES		8192
#define J700_MT7932_DEBUG_DUMP_DELAY_MS	2000

struct j700_mt7932_debug {
	struct serdev_device *serdev;
	struct delayed_work dump_work;
	spinlock_t lock; /* Protects capture counters and bytes. */
	size_t length;
	size_t dropped;
	u8 bytes[J700_MT7932_DEBUG_BYTES];
};

static DECLARE_COMPLETION(j700_mt7932_debug_ready);

bool j700_mt7932_debug_uart_wait_ready(unsigned long timeout)
{
	return wait_for_completion_timeout(&j700_mt7932_debug_ready, timeout);
}
EXPORT_SYMBOL_GPL(j700_mt7932_debug_uart_wait_ready);

static size_t j700_mt7932_debug_receive(struct serdev_device *serdev,
					const u8 *buf, size_t count)
{
	struct j700_mt7932_debug *debug = serdev_device_get_drvdata(serdev);
	size_t available, copied;

	spin_lock(&debug->lock);
	available = sizeof(debug->bytes) - debug->length;
	copied = min(count, available);
	memcpy(debug->bytes + debug->length, buf, copied);
	debug->length += copied;
	debug->dropped += count - copied;
	spin_unlock(&debug->lock);

	return count;
}

static const struct serdev_device_ops j700_mt7932_debug_ops = {
	.receive_buf = j700_mt7932_debug_receive,
};

static void j700_mt7932_debug_dump(struct work_struct *work)
{
	struct j700_mt7932_debug *debug =
		container_of(to_delayed_work(work), struct j700_mt7932_debug,
			     dump_work);
	size_t offset, length, dropped;

	spin_lock(&debug->lock);
	length = debug->length;
	dropped = debug->dropped;
	spin_unlock(&debug->lock);

	dev_info(&debug->serdev->dev,
		 "J700_MT7932_DEBUG_UART_CAPTURE: bytes=%zu dropped=%zu baud=%u receive_only=1\n",
		 length, dropped, J700_MT7932_DEBUG_BAUD);
	for (offset = 0; offset < length; offset += 64) {
		size_t chunk = min_t(size_t, 64, length - offset);

		dev_info(&debug->serdev->dev,
			 "J700_MT7932_DEBUG_UART_DATA: offset=0x%04zx bytes=%zu hex=%*phN\n",
			 offset, chunk, (int)chunk, debug->bytes + offset);
	}
}

static int j700_mt7932_debug_probe(struct serdev_device *serdev)
{
	struct j700_mt7932_debug *debug;
	unsigned int baud;
	int ret;

	debug = devm_kzalloc(&serdev->dev, sizeof(*debug), GFP_KERNEL);
	if (!debug)
		return -ENOMEM;

	debug->serdev = serdev;
	spin_lock_init(&debug->lock);
	INIT_DELAYED_WORK(&debug->dump_work, j700_mt7932_debug_dump);
	serdev_device_set_drvdata(serdev, debug);
	serdev_device_set_client_ops(serdev, &j700_mt7932_debug_ops);

	ret = serdev_device_open(serdev);
	if (ret)
		return dev_err_probe(&serdev->dev, ret,
				     "failed to open firmware debug UART\n");

	serdev_device_set_flow_control(serdev, false);
	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret)
		goto close;
	baud = serdev_device_set_baudrate(serdev, J700_MT7932_DEBUG_BAUD);
	if (!baud) {
		ret = -EINVAL;
		goto close;
	}

	dev_info(&serdev->dev,
		 "J700_MT7932_DEBUG_UART_READY: requested_baud=%u actual_baud=%u receive_only=1 buffer=%u\n",
		 J700_MT7932_DEBUG_BAUD, baud, J700_MT7932_DEBUG_BYTES);
	complete_all(&j700_mt7932_debug_ready);
	schedule_delayed_work(&debug->dump_work,
			      msecs_to_jiffies(J700_MT7932_DEBUG_DUMP_DELAY_MS));

	return 0;

close:
	serdev_device_close(serdev);
	return ret;
}

static void j700_mt7932_debug_remove(struct serdev_device *serdev)
{
	struct j700_mt7932_debug *debug = serdev_device_get_drvdata(serdev);

	cancel_delayed_work_sync(&debug->dump_work);
	serdev_device_close(serdev);
}

static const struct of_device_id j700_mt7932_debug_of_match[] = {
	{ .compatible = "aurora,j700-mt7932-debug-uart" },
	{ }
};
MODULE_DEVICE_TABLE(of, j700_mt7932_debug_of_match);

static struct serdev_device_driver j700_mt7932_debug_driver = {
	.probe = j700_mt7932_debug_probe,
	.remove = j700_mt7932_debug_remove,
	.driver = {
		.name = "j700-mt7932-debug-uart",
		.of_match_table = j700_mt7932_debug_of_match,
	},
};

static int __init j700_mt7932_debug_init(void)
{
	return serdev_device_driver_register(&j700_mt7932_debug_driver);
}
subsys_initcall(j700_mt7932_debug_init);

MODULE_DESCRIPTION("Receive-only J700 MT7932 firmware debug UART capture");
MODULE_LICENSE("GPL");
