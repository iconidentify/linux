// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2021 The Asahi Linux Contributors
 *
 * PA Semi PWRficient SMBus host driver for Apple SoCs
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "i2c-pasemi-core.h"

struct pasemi_platform_i2c_data {
	struct pasemi_smbus smbus;
	struct clk *clk_ref;
	struct gpio_desc *hub_reset_gpio;
	struct delayed_work usb_diag_work;
};

static int pasemi_platform_i2c_read_byte_data(struct pasemi_smbus *smbus,
					      u16 address, u8 reg)
{
	union i2c_smbus_data value;
	int ret;

	ret = i2c_smbus_xfer(&smbus->adapter, address, 0, I2C_SMBUS_READ,
			     reg, I2C_SMBUS_BYTE_DATA, &value);
	if (ret)
		return ret;

	return value.byte;
}

static int pasemi_platform_i2c_write_byte_data(struct pasemi_smbus *smbus,
					       u16 address, u8 reg, u8 byte)
{
	union i2c_smbus_data value = { .byte = byte };

	return i2c_smbus_xfer(&smbus->adapter, address, 0, I2C_SMBUS_WRITE,
			      reg, I2C_SMBUS_BYTE_DATA, &value);
}

struct j700_usb_i2c_tunable {
	u8 reg;
	u8 mask;
	u8 value;
};

/* /arm-io/i2c2/usb-repeater-i2c:tunable from the live J700 ADT. */
static const struct j700_usb_i2c_tunable j700_ticd2e22_tunable[] = {
	{ 0x00, 0x0f, 0x00 },
	{ 0x01, 0x0f, 0x00 },
	{ 0x02, 0x07, 0x00 },
	{ 0x03, 0xf0, 0x10 },
	{ 0x82, 0x20, 0x20 },
};

/*
 * The custom TICD2E22 exposes two near-identical low register banks plus the
 * bank containing the ADT's 0x82 tunable.  These are the only non-zero control
 * ranges in the reset dump; the calibration/identity block at 0xc0 is stable
 * silicon data and is deliberately left alone.
 */
static const u8 j700_ticd2e22_snapshot_regs[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x80, 0x81, 0x82,
};

static void pasemi_platform_i2c_snapshot_j700_repeater(
	struct pasemi_smbus *smbus, const char *phase)
{
	char values[3 * ARRAY_SIZE(j700_ticd2e22_snapshot_regs) + 1];
	size_t offset = 0;
	unsigned int errors = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(j700_ticd2e22_snapshot_regs); i++) {
		int value = pasemi_platform_i2c_read_byte_data(
			smbus, 0x21, j700_ticd2e22_snapshot_regs[i]);

		if (value < 0)
			errors++;
		offset += scnprintf(values + offset, sizeof(values) - offset,
				    "%s%02x", i ? " " : "",
				    value < 0 ? 0xff : value);
	}

	dev_info(smbus->dev,
		 "J700_USB_I2C_REPEATER_SNAPSHOT: phase=%s regs=00-0f/40-4f/80-82 values=%s errors=%u\n",
		 phase, values, errors);
}

static int pasemi_platform_i2c_apply_j700_repeater(struct pasemi_smbus *smbus)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(j700_ticd2e22_tunable); i++) {
		const struct j700_usb_i2c_tunable *t =
			&j700_ticd2e22_tunable[i];
		int old_value, ret;
		u8 value;

		old_value = pasemi_platform_i2c_read_byte_data(smbus, 0x21,
							    t->reg);
		if (old_value < 0)
			return old_value;

		value = (old_value & ~t->mask) | (t->value & t->mask);
		ret = pasemi_platform_i2c_write_byte_data(smbus, 0x21,
							 t->reg, value);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * J700's shipping AppleTICD2E22::open() reads register 0x82 and clears bit 5
 * when usb-repeater-options is absent (option 0).  The live ADT has no options
 * property, while its tunable deliberately sets bit 5 before open().  Mirror
 * that ordered tune-then-open transaction without changing the other port bit.
 */
static int pasemi_platform_i2c_open_j700_repeater(struct pasemi_smbus *smbus)
{
	int old_value, readback, ret;
	u8 value;

	old_value = pasemi_platform_i2c_read_byte_data(smbus, 0x21, 0x82);
	if (old_value < 0)
		return old_value;

	value = old_value & ~BIT(5);
	ret = pasemi_platform_i2c_write_byte_data(smbus, 0x21, 0x82, value);
	if (ret)
		return ret;

	readback = pasemi_platform_i2c_read_byte_data(smbus, 0x21, 0x82);
	if (readback < 0)
		return readback;
	if (readback != value)
		return -EIO;

	dev_info(smbus->dev,
		 "J700_USB2_REPEATER_OPEN_PASS: option=0 reg82=%02x->%02x readback=%02x\n",
		 old_value, value, readback);

	return 0;
}

static void pasemi_platform_i2c_usb_diag(struct work_struct *work)
{
	struct pasemi_platform_i2c_data *data =
		container_of(to_delayed_work(work),
			     struct pasemi_platform_i2c_data, usb_diag_work);
	struct pasemi_smbus *smbus = &data->smbus;
	bool apply_repeater, open_ok = false, tune_ok = false;
	int repeater0, repeater1, ret;

	pasemi_platform_i2c_snapshot_j700_repeater(smbus, "pre-tune");

	apply_repeater = device_property_read_bool(
		smbus->dev, "apple,j700-usb-apply-repeater-tunable");
	if (apply_repeater) {
		ret = pasemi_platform_i2c_apply_j700_repeater(smbus);
		if (ret)
			dev_err(smbus->dev,
				"J700_USB_I2C_REPEATER_FAIL: address=21 err=%d\n",
				ret);
		else
			dev_info(smbus->dev,
				 "J700_USB_I2C_REPEATER_PASS: address=21 entries=%zu\n",
				 ARRAY_SIZE(j700_ticd2e22_tunable));
		tune_ok = !ret;
	}
	pasemi_platform_i2c_snapshot_j700_repeater(smbus, "post-tune");

	/* The J700 ADT transaction is incomplete until option-0 open follows it. */
	if (apply_repeater) {
		ret = pasemi_platform_i2c_open_j700_repeater(smbus);
		if (ret)
			dev_err(smbus->dev,
				"J700_USB2_REPEATER_OPEN_FAIL: option=0 address=21 reg=82 err=%d\n",
				ret);
		open_ok = !ret;
		pasemi_platform_i2c_snapshot_j700_repeater(smbus,
							 "post-open");
	}

	/* Do not touch VL122 while its USB core owns the otherwise NACKing plane. */
	repeater0 = pasemi_platform_i2c_read_byte_data(smbus, 0x21, 0);
	repeater1 = pasemi_platform_i2c_read_byte_data(smbus, 0x21, 1);
	if (tune_ok && open_ok && repeater0 >= 0 && repeater1 >= 0)
		dev_info(smbus->dev,
			 "J700_USB2_REPEATER_ONLY_PASS: tunable=1 open=1 reg00=%02x reg01=%02x\n",
			 repeater0, repeater1);
	else
		dev_warn(smbus->dev,
			 "J700_USB2_REPEATER_ONLY_FAIL: tunable=%u open=%u ret=%d,%d\n",
			 tune_ok, open_ok, repeater0, repeater1);
}

static int
pasemi_platform_i2c_calc_clk_div(struct pasemi_platform_i2c_data *data,
				 u32 frequency)
{
	unsigned long clk_rate = clk_get_rate(data->clk_ref);

	if (!clk_rate)
		return -EINVAL;

	data->smbus.clk_div = DIV_ROUND_UP(clk_rate, 16 * frequency);
	if (data->smbus.clk_div < 4)
		return dev_err_probe(data->smbus.dev, -EINVAL,
				     "Bus frequency %d is too fast.\n",
				     frequency);
	if (data->smbus.clk_div > 0xff)
		return dev_err_probe(data->smbus.dev, -EINVAL,
				     "Bus frequency %d is too slow.\n",
				     frequency);

	return 0;
}

static int pasemi_platform_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pasemi_platform_i2c_data *data;
	struct pasemi_smbus *smbus;
	u32 frequency;
	int error;
	int irq_num;

	data = devm_kzalloc(dev, sizeof(struct pasemi_platform_i2c_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	smbus = &data->smbus;
	smbus->dev = dev;
	if (device_property_read_bool(dev,
				      "apple,j700-usb-read-only-diagnostics")) {
		data->hub_reset_gpio = devm_gpiod_get_optional(
			dev, "hub-reset", GPIOD_OUT_LOW);
		if (IS_ERR(data->hub_reset_gpio))
			return dev_err_probe(dev, PTR_ERR(data->hub_reset_gpio),
					     "Failed to acquire VL122 reset GPIO\n");
		if (data->hub_reset_gpio)
			dev_info(dev,
				 "J700_USB2_VL122_ALWAYS_ON: reset=deasserted owner=i2c2\n");
	}

	smbus->ioaddr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(smbus->ioaddr))
		return PTR_ERR(smbus->ioaddr);

	if (of_property_read_u32(dev->of_node, "clock-frequency", &frequency))
		frequency = I2C_MAX_STANDARD_MODE_FREQ;

	data->clk_ref = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(data->clk_ref))
		return PTR_ERR(data->clk_ref);

	error = pasemi_platform_i2c_calc_clk_div(data, frequency);
	if (error)
		return error;

	smbus->adapter.dev.of_node = pdev->dev.of_node;
	error = pasemi_i2c_common_probe(smbus);
	if (error)
		return error;

	irq_num = platform_get_irq(pdev, 0);
	error = devm_request_irq(smbus->dev, irq_num, pasemi_irq_handler, 0, "pasemi_apple_i2c", (void *)smbus);

	if (!error)
		smbus->use_irq = 1;
	platform_set_drvdata(pdev, data);

	if (device_property_read_bool(dev,
				      "apple,j700-usb-read-only-diagnostics")) {
		INIT_DELAYED_WORK(&data->usb_diag_work,
				  pasemi_platform_i2c_usb_diag);
		schedule_delayed_work(&data->usb_diag_work, msecs_to_jiffies(3500));
	}

	return 0;
}

static void pasemi_platform_i2c_remove(struct platform_device *pdev)
{
	struct pasemi_platform_i2c_data *data = platform_get_drvdata(pdev);

	if (device_property_read_bool(&pdev->dev,
				      "apple,j700-usb-read-only-diagnostics"))
		cancel_delayed_work_sync(&data->usb_diag_work);
}

static const struct of_device_id pasemi_platform_i2c_of_match[] = {
	{ .compatible = "apple,t8103-i2c" },
	{ .compatible = "apple,i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, pasemi_platform_i2c_of_match);

static struct platform_driver pasemi_platform_i2c_driver = {
	.driver	= {
		.name			= "i2c-apple",
		.of_match_table		= pasemi_platform_i2c_of_match,
	},
	.probe	= pasemi_platform_i2c_probe,
	.remove = pasemi_platform_i2c_remove,
};
module_platform_driver(pasemi_platform_i2c_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple/PASemi SMBus platform driver");
