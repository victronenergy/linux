/*
 * Copyright (C) 2017 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irqflags.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/property.h>

struct lt3593_bl_data {
	struct gpio_desc	*gpio;
};

static int lt3593_bl_update(struct backlight_device *bl)
{
	struct lt3593_bl_data *lt = bl_get_data(bl);
	int brightness = backlight_get_brightness(bl);
	unsigned long flags;

	if (brightness == 0) {
		gpiod_set_value(lt->gpio, 0);
		return 0;
	}

	/*
	 * We must disable IRQs since interrupting the signalling for
	 * more than 128 us will result in an incorrect setting.
	 */

	local_irq_save(flags);

	while (brightness++ < 32) {
		gpiod_set_value(lt->gpio, 0);
		ndelay(250);
		gpiod_set_value(lt->gpio, 1);
		ndelay(250);
	}

	local_irq_restore(flags);

	return 0;
}

static const struct backlight_ops lt3593_backlight_ops = {
	.update_status	= lt3593_bl_update,
};

static int lt3593_bl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct lt3593_bl_data *lt;
	u32 brightness = 31;
	int gflags;

	lt = devm_kzalloc(dev, sizeof(*lt), GFP_KERNEL);
	if (!lt)
		return -ENOMEM;

	device_property_read_u32(dev, "default-brightness-level", &brightness);
	gflags = brightness ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW;

	lt->gpio = devm_gpiod_get(dev, NULL, gflags);
	if (IS_ERR(lt->gpio))
		return PTR_ERR(lt->gpio);

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 31;
	props.brightness = brightness;
	props.power = brightness ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;

	bl = devm_backlight_device_register(dev, dev_name(dev), dev, lt,
					    &lt3593_backlight_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);

	return 0;
}

static int lt3593_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_disable(bl);

	return 0;
}

static struct of_device_id lt3593_bl_of_match[] = {
	{ .compatible = "lltc,lt3593-backlight" },
	{ }
};
MODULE_DEVICE_TABLE(of, lt3593_bl_of_match);

static struct platform_driver lt3593_bl_driver = {
	.driver		= {
		.name		= "lt3593-backlight",
		.of_match_table	= lt3593_bl_of_match,
	},
	.probe		= lt3593_bl_probe,
	.remove		= lt3593_bl_remove,
};
module_platform_driver(lt3593_bl_driver);

MODULE_DESCRIPTION("LT3593 Backlight Driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lt3593-backlight");
