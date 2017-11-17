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

struct lt3593_bl_data {
	struct gpio_desc	*gpio;
	struct gpio_desc	*en_gpio;
};

static void lt3593_bl_config(struct lt3593_bl_data *lt, int brightness)
{
	unsigned long flags;
	int i;

	if (brightness == 0) {
		gpiod_set_value(lt->gpio, 0);
		return;
	}

	/*
	 * We must disable IRQs since interrupting the signalling for
	 * more than 100 us will result in an incorrect setting.
	 */

	local_irq_save(flags);

	for (i = brightness; i < 32; i++) {
		gpiod_set_value(lt->gpio, 0);
		ndelay(250);
		gpiod_set_value(lt->gpio, 1);
		ndelay(250);
	}

	local_irq_restore(flags);
}

static int lt3593_bl_update(struct backlight_device *bl)
{
	struct lt3593_bl_data *lt = bl_get_data(bl);
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	lt3593_bl_config(lt, brightness);

	return 0;
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= lt3593_bl_update,
};

static int lt3593_bl_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct backlight_properties props;
	struct backlight_device *bl;
	struct lt3593_bl_data *lt;
	u32 brightness = 31;
	int power = FB_BLANK_POWERDOWN;

	lt = devm_kzalloc(&pdev->dev, sizeof(*bl), GFP_KERNEL);
	if (!lt)
		return -ENOMEM;

	lt->en_gpio = devm_gpiod_get_optional(&pdev->dev, "enable",
					      GPIOD_OUT_HIGH);
	if (IS_ERR(lt->en_gpio))
		return PTR_ERR(lt->en_gpio);

	lt->gpio = devm_gpiod_get(&pdev->dev, NULL, GPIOD_ASIS);
	if (IS_ERR(lt->gpio))
		return PTR_ERR(lt->gpio);

	if (gpiod_get_direction(lt->gpio) == GPIOF_DIR_OUT) {
		if (gpiod_get_value(lt->gpio) == 1)
			power = FB_BLANK_UNBLANK;
	}

	if (node) {
		if (of_property_read_bool(node, "default-on"))
			power = FB_BLANK_UNBLANK;

		of_property_read_u32(node, "default-brightness-level",
				     &brightness);
	}

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 31;

	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, lt,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	bl->props.brightness = brightness;
	bl->props.power = power;

	gpiod_direction_output(lt->gpio, power == FB_BLANK_UNBLANK);
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);

	return 0;
}

static int lt3593_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct lt3593_bl_data *lt = bl_get_data(bl);

	backlight_device_unregister(bl);
	lt3593_bl_config(lt, 0);

	return 0;
}

static struct of_device_id lt3593_bl_of_match[] = {
	{ .compatible = "lltc,lt3593" },
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
