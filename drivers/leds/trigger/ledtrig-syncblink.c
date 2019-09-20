/*
 * Synchronous blinking LED trigger
 *
 * Copyright (C) 2019 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/timer.h>

#define DELAY_MS 250

static int sync_trig_activate(struct led_classdev *led_cdev);
static void sync_trig_deactivate(struct led_classdev *led_cdev);

static atomic_t led_count;

static struct led_trigger trig_fast = {
	.name     = "blink-fast",
	.activate = sync_trig_activate,
	.deactivate = sync_trig_deactivate,
};

static struct led_trigger trig_slow = {
	.name     = "blink-slow",
	.activate = sync_trig_activate,
	.deactivate = sync_trig_deactivate,
};

static void trig_timer_tick(struct timer_list *t)
{
	static unsigned int count;

	led_trigger_event(&trig_fast, count & 1 ? LED_OFF : LED_FULL);

	if (!(count & 1))
		led_trigger_event(&trig_slow, count & 2 ? LED_OFF : LED_FULL);

	count++;

	if (atomic_read(&led_count))
		mod_timer(t, jiffies + msecs_to_jiffies(DELAY_MS));
}

static DEFINE_TIMER(trig_timer, trig_timer_tick);

static int sync_trig_activate(struct led_classdev *led_cdev)
{
	if (atomic_inc_return(&led_count) == 1)
		mod_timer(&trig_timer, jiffies);

	return 0;
}

static void sync_trig_deactivate(struct led_classdev *led_cdev)
{
	if (!atomic_dec_return(&led_count))
		del_timer(&trig_timer);
}

static int __init ledtrig_sync_init(void)
{
	led_trigger_register(&trig_fast);
	led_trigger_register(&trig_slow);

	return 0;
}
module_init(ledtrig_sync_init);

static void __exit ledtrig_sync_exit(void)
{
	led_trigger_unregister(&trig_fast);
	led_trigger_unregister(&trig_slow);
}
module_exit(ledtrig_sync_exit);

MODULE_DESCRIPTION("Synchronous blinking LED trigger");
MODULE_AUTHOR("Mans Rullgard");
MODULE_LICENSE("GPL v2");
