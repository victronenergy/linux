// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2009,2018 Daniel Mack <daniel@zonque.org>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/slab.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>

#define LED_LT3593_NAME "lt3593"

struct lt3593_led_data {
	struct led_classdev cdev;
	struct gpio_desc *gpiod;
};

static int lt3593_led_set(struct led_classdev *led_cdev,
			   enum led_brightness value)
{
	struct lt3593_led_data *led_dat =
		container_of(led_cdev, struct lt3593_led_data, cdev);
	unsigned long flags;

	/*
	 * The LT3593 resets its internal current level register to the maximum
	 * level on the first falling edge on the control pin. Each following
	 * falling edge decreases the current level by 625uA. Up to 32 pulses
	 * can be sent, so the maximum power reduction is 20mA.
	 * After a timeout of 128us, the value is taken from the register and
	 * applied is to the output driver.
	 */

	if (value == 0) {
		gpiod_set_value_cansleep(led_dat->gpiod, 0);
		return 0;
	}

	/*
	 * We must disable IRQs since interrupting the signalling for
	 * more than 128 us will result in an incorrect setting.
	 */

	local_irq_save(flags);

	while (value++ < 32) {
		gpiod_set_value(led_dat->gpiod, 0);
		ndelay(250);
		gpiod_set_value(led_dat->gpiod, 1);
		ndelay(250);
	}

	local_irq_restore(flags);

	return 0;
}

static int lt3593_led_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lt3593_led_data *led_data;
	struct fwnode_handle *child;
	enum led_default_state state;
	int ret;
	struct led_init_data init_data = {};

	led_data = devm_kzalloc(dev, sizeof(*led_data), GFP_KERNEL);
	if (!led_data)
		return -ENOMEM;

	if (device_get_child_node_count(dev) != 1) {
		dev_err(dev, "Device must have exactly one LED sub-node.");
		return -EINVAL;
	}

	child = device_get_next_child_node(dev, NULL);

	state = led_init_default_state_get(child);

	led_data->cdev.brightness_set_blocking = lt3593_led_set;
	led_data->cdev.max_brightness = 31;
	led_data->cdev.brightness = state ? 31 : 0;

	init_data.fwnode = child;
	init_data.devicename = LED_LT3593_NAME;
	init_data.default_label = ":";

	fwnode_handle_put(child);

	led_data->gpiod = devm_gpiod_get(dev, "lltc,ctrl",
				state ? GPIOD_OUT_HIGH : GPIOD_OUT_LOW);
	if (IS_ERR(led_data->gpiod))
		return PTR_ERR(led_data->gpiod);

	ret = devm_led_classdev_register_ext(dev, &led_data->cdev, &init_data);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, led_data);

	return 0;
}

static const struct of_device_id of_lt3593_leds_match[] = {
	{ .compatible = "lltc,lt3593", },
	{},
};
MODULE_DEVICE_TABLE(of, of_lt3593_leds_match);

static struct platform_driver lt3593_led_driver = {
	.probe		= lt3593_led_probe,
	.driver		= {
		.name	= "leds-lt3593",
		.of_match_table = of_lt3593_leds_match,
	},
};

module_platform_driver(lt3593_led_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("LED driver for LT3593 controllers");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:leds-lt3593");
