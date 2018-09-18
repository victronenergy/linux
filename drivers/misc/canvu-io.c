/*
 * Copyright (C) 2018 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irq_sim.h>
#include <linux/of_gpio.h>
#include <linux/serdev.h>
#include <linux/wait.h>

#define MAX_GPIO_RETRY 10

struct canvu_io {
	size_t bufsize;
	u8 *outbuf;
	u8 *inbuf;
	int inpos;

	struct serdev_device *sdev;
	struct delayed_work dwork;
	struct mutex lock;
	bool ready;

	struct gpio_chip gpio;
	int nr_out;
	int nr_in;
	u8 *pins;
	int gpio_update;
	wait_queue_head_t gpio_wait;
	struct irq_domain *irqdom;
};

static inline struct canvu_io *gpio_to_canvu_io(struct gpio_chip *g)
{
	return container_of(g, struct canvu_io, gpio);
}

static inline struct canvu_io *work_to_canvu_io(struct work_struct *w)
{
	return container_of(w, struct canvu_io, dwork.work);
}

static int hex2u8(const u8 *buf)
{
	int hi = hex_to_bin(buf[0]);
	int lo = hex_to_bin(buf[1]);

	return hi << 4 | lo;
}

static int hex2u16(const u8 *buf)
{
	int hi = hex2u8(&buf[0]);
	int lo = hex2u8(&buf[2]);

	return hi << 8 | lo;
}

static int canvu_io_checksum(u8 *buf, size_t len, u8 **end)
{
	u8 *p = buf;
	int sum = 0;

	do {
		sum += *p;
	} while (*p++ != '*' && p - buf < len);

	if (end)
		*end = p;

	return sum & 0xff;
}

static void canvu_io_irq(struct canvu_io *cio, int pin, int val)
{
	int irq = irq_find_mapping(cio->irqdom, pin - cio->nr_out);
	int edge = val ? IRQ_TYPE_EDGE_RISING : IRQ_TYPE_EDGE_FALLING;
	int type;

	if (!irq)
		return;

	type = irq_get_trigger_type(irq);

	if (type & edge)
		irq_set_irqchip_state(irq, IRQCHIP_STATE_PENDING, true);
}

static int canvu_io_handle_input(struct canvu_io *cio)
{
	const u8 *p = cio->inbuf + 2;
	const u8 *end = cio->inbuf + cio->inpos;
	int pin = cio->nr_out;
	int evt;
	int val;
	int old;

	while (p < end - 6 && *p != '*' && pin < cio->gpio.ngpio) {
		if (*p++ != ',')
			return -EINVAL;

		evt = hex2u16(p);
		if (evt < 0)
			return -EINVAL;

		p += 4;

		if (*p++ != '-')
			return -EINVAL;

		val = hex_to_bin(*p++);
		if ((unsigned)val > 1)
			return -EINVAL;

		old = cio->pins[pin];
		cio->pins[pin] = val;

		if (val != old)
			canvu_io_irq(cio, pin, val);

		pin++;
	}

	return 0;
}

static int canvu_io_handle_output(struct canvu_io *cio)
{
	const u8 *p = cio->inbuf + 2;
	const u8 *end = cio->inbuf + cio->inpos;
	bool need_update = false;
	bool wake = false;
	int err = 0;
	int pin;
	int val;
	int pv;

	mutex_lock(&cio->lock);

	for (pin = -1; p < end - 2 && *p != '*' && pin < cio->nr_out; pin++) {
		if (*p++ != ',') {
			err = -EINVAL;
			goto out;
		}

		val = hex2u8(p);
		if (val < 0) {
			err = -EINVAL;
			goto out;
		}

		p += 2;

		if (pin < 0)
			continue;

		val = !!val;
		pv = cio->pins[pin];

		if ((pv ^ val) == 2) {
			cio->pins[pin] = val;
			wake = true;
		} else if ((pv & 1) != val) {
			need_update = true;
		}
	}

	if (!need_update)
		cio->gpio_update = 0;

	if (wake)
		wake_up_all(&cio->gpio_wait);

out:
	mutex_unlock(&cio->lock);

	return err;
}

static int canvu_io_handle_version(struct canvu_io *cio)
{
	if (cio->inpos < 9)
		return -EINVAL;

	cio->inbuf[cio->inpos - 3] = 0;

	dev_info(&cio->sdev->dev, "version: %s\n", cio->inbuf + 3);

	if (!cio->ready) {
		cio->ready = true;
		wake_up(&cio->gpio_wait);
	}

	return 0;
}

static int canvu_io_handle_message(struct canvu_io *cio)
{
	u8 *end;
	int csum;

	if (cio->inpos < 5)
		return -EINVAL;

	if (cio->inbuf[0] != '#')
		return -EINVAL;

	csum = canvu_io_checksum(cio->inbuf, cio->inpos, &end);
	if (csum != hex2u8(end))
		return -EINVAL;

	switch (cio->inbuf[1]) {
	case 'O':
		return canvu_io_handle_output(cio);

	case 'S':
		return canvu_io_handle_input(cio);

	case 'V':
		return canvu_io_handle_version(cio);
	}

	return 0;
}

static size_t canvu_io_add_data(struct canvu_io *cio, const u8 *buf,
				size_t len)
{
	const u8 *bufend = buf + len;
	const u8 *start = buf;
	const u8 *end;
	bool nl;

	if (cio->inpos == 0) {
		start = memchr(buf, '#', len);
		if (!start)
			return len;

		len = bufend - start;
	}

	end = memchr(start, '\n', len);

	if (end) {
		len = end++ - start;
		nl = true;
	} else {
		end = start + len;
		nl = false;
	}

	if (cio->inpos + len > cio->bufsize) {
		cio->inpos = 0;
		return end - buf;
	}

	memcpy(cio->inbuf + cio->inpos, start, len);
	cio->inpos += len;

	if (nl) {
		canvu_io_handle_message(cio);
		cio->inpos = 0;
	}

	return end - buf;
}

static int canvu_io_recv(struct serdev_device *sdev, const u8 *buf, size_t len)
{
	struct canvu_io *cio = serdev_device_get_drvdata(sdev);
	const unsigned char *end = buf + len;

	while (buf < end)
		buf += canvu_io_add_data(cio, buf, end - buf);

	return len;
}

static const struct serdev_device_ops canvu_io_ops = {
	.receive_buf	= canvu_io_recv,
	.write_wakeup	= serdev_device_write_wakeup,
};

static int canvu_io_send(struct canvu_io *cio, size_t len)
{
	struct serdev_device *sdev = cio->sdev;
	u8 *buf = cio->outbuf;
	u8 *end;
	int csum;
	int ret;

	csum = canvu_io_checksum(buf, len, &end);
	end += snprintf(end, cio->bufsize - len, "%02X\n", csum);

	ret = serdev_device_write(sdev, buf, end - buf, msecs_to_jiffies(10));
	if (ret < 0) {
		dev_err(&sdev->dev, "write error: %d\n", ret);
		return ret;
	}

	return 0;
}

static int canvu_io_set_outputs(struct canvu_io *cio)
{
	u8 *p = cio->outbuf;
	u8 *end = cio->outbuf + cio->bufsize;
	int i;

	p += snprintf(p, end - p, "#o,00");

	for (i = 0; i < cio->nr_out; i++)
		p += snprintf(p, end - p, ",%02X", 255 * (cio->pins[i] & 1));

	p += snprintf(p, end - p, "*");

	canvu_io_send(cio, p - cio->outbuf);

	if (--cio->gpio_update)
		schedule_delayed_work(&cio->dwork, msecs_to_jiffies(40));

	return 0;
}

static int canvu_io_set_output(struct canvu_io *cio, unsigned pin, int val)
{
	int tmo = msecs_to_jiffies(100);

	mutex_lock(&cio->lock);

	cio->pins[pin] = val | 2;
	cio->gpio_update = MAX_GPIO_RETRY;
	canvu_io_set_outputs(cio);

	mutex_unlock(&cio->lock);

	tmo = wait_event_timeout(cio->gpio_wait, !(cio->pins[pin] & 2), tmo);

	if (!tmo) {
		dev_warn(&cio->sdev->dev, "long delay setting output\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int canvu_io_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct canvu_io *cio = gpio_to_canvu_io(chip);

	if (offset < cio->nr_out)
		return 0;

	return 1;
}

static int canvu_io_gpio_direction_input(struct gpio_chip *chip,
					 unsigned offset)
{
	struct canvu_io *cio = gpio_to_canvu_io(chip);

	if (offset < cio->nr_out)
		return -EINVAL;

	return 0;
}

static int canvu_io_gpio_direction_output(struct gpio_chip *chip,
					  unsigned offset, int val)
{
	struct canvu_io *cio = gpio_to_canvu_io(chip);

	if (offset >= cio->nr_out)
		return -EINVAL;

	if (cio->pins[offset] == val)
		return 0;

	canvu_io_set_output(cio, offset, val);

	return 0;
}

static int canvu_io_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct canvu_io *cio = gpio_to_canvu_io(chip);

	return cio->pins[offset] & 1;
}

static void canvu_io_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct canvu_io *cio = gpio_to_canvu_io(chip);

	if (offset >= cio->nr_out)
		return;

	if (cio->pins[offset] == val)
		return;

	canvu_io_set_output(cio, offset, val);
}

static int canvu_io_gpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct canvu_io *cio = gpio_to_canvu_io(chip);

	if (offset < cio->nr_out)
		return -EINVAL;

	return irq_create_mapping(cio->irqdom, offset - cio->nr_out);
}

static int canvu_io_gpio_init(struct device *dev, int nr_out, int nr_in)
{
	struct canvu_io *cio = dev_get_drvdata(dev);
	int npins = nr_out + nr_in;

	cio->nr_out = nr_out;
	cio->nr_in = nr_in;
	init_waitqueue_head(&cio->gpio_wait);

	cio->pins = devm_kzalloc(dev, npins, GFP_KERNEL);
	if (!cio->pins)
		return -ENOMEM;

	cio->gpio.label = dev_name(dev);
	cio->gpio.parent = dev;
	cio->gpio.base = -1;
	cio->gpio.ngpio = npins;
	cio->gpio.can_sleep = true;
	cio->gpio.of_node = dev->of_node;

	cio->gpio.get_direction = canvu_io_gpio_get_direction;
	cio->gpio.direction_input = canvu_io_gpio_direction_input;
	cio->gpio.direction_output = canvu_io_gpio_direction_output;
	cio->gpio.get = canvu_io_gpio_get;
	cio->gpio.set = canvu_io_gpio_set;
	cio->gpio.to_irq = canvu_io_gpio_to_irq;

	cio->irqdom = devm_irq_domain_create_sim(dev, dev->fwnode, nr_in);
	if (IS_ERR(cio->irqdom))
		return PTR_ERR(cio->irqdom);

	return 0;
}

static void canvu_io_work(struct work_struct *w)
{
	struct canvu_io *cio = work_to_canvu_io(w);

	mutex_lock(&cio->lock);

	if (cio->gpio_update)
		canvu_io_set_outputs(cio);

	mutex_unlock(&cio->lock);
}

static int canvu_io_probe(struct serdev_device *sdev)
{
	struct device *dev = &sdev->dev;
	struct canvu_io *cio;
	u32 nr_out;
	u32 nr_in;
	u32 rate;
	int err;
	int i;

	cio = devm_kzalloc(dev, sizeof(*cio), GFP_KERNEL);
	if (!cio)
		return -ENOMEM;

	if (of_property_read_u32(dev->of_node, "current-speed", &rate)) {
		dev_err(dev, "current-speed not provided\n");
		return -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, "nr-dig-out", &nr_out))
		return -EINVAL;

	if (of_property_read_u32(dev->of_node, "nr-dig-in", &nr_in))
		return -EINVAL;

	cio->bufsize = SZ_1K;

	cio->inbuf = devm_kzalloc(dev, cio->bufsize, GFP_KERNEL);
	if (!cio->inbuf)
		return -ENOMEM;

	cio->outbuf = devm_kzalloc(dev, cio->bufsize, GFP_KERNEL);
	if (!cio->outbuf)
		return -ENOMEM;

	serdev_device_set_drvdata(sdev, cio);
	serdev_device_set_client_ops(sdev, &canvu_io_ops);

	cio->sdev = sdev;
	INIT_DELAYED_WORK(&cio->dwork, canvu_io_work);
	mutex_init(&cio->lock);

	err = canvu_io_gpio_init(dev, nr_out, nr_in);
	if (err)
		return err;

	err = serdev_device_open(sdev);
	if (err)
		return err;

	serdev_device_set_baudrate(sdev, rate);
	serdev_device_set_flow_control(sdev, false);

	strcpy(cio->outbuf, "#v*");

	for (i = 0; i < 3; i++) {
		int tmo = msecs_to_jiffies(40);
		canvu_io_send(cio, 3);
		if (wait_event_timeout(cio->gpio_wait, cio->ready, tmo))
			break;
	}

	if (!cio->ready) {
		err = -ENODEV;
		goto err_out;
	}

	err = gpiochip_add(&cio->gpio);
	if (err)
		goto err_out;

	cio->gpio_update = MAX_GPIO_RETRY;
	schedule_delayed_work(&cio->dwork, 0);

	return 0;

err_out:
	serdev_device_close(sdev);

	return err;
}

static void canvu_io_remove(struct serdev_device *sdev)
{
	struct canvu_io *cio = serdev_device_get_drvdata(sdev);

	gpiochip_remove(&cio->gpio);
	serdev_device_close(sdev);
	cancel_delayed_work_sync(&cio->dwork);
}

static const struct of_device_id canvu_io_dt_ids[] = {
	{ .compatible = "cantronik,canvu-io" },
	{ }
};
MODULE_DEVICE_TABLE(of, canvu_io_dt_ids);

static struct serdev_device_driver canvu_io_driver = {
	.driver	= {
		.name		= "canvu-io",
		.of_match_table	= canvu_io_dt_ids,
	},
	.probe	= canvu_io_probe,
	.remove	= canvu_io_remove,
};
module_serdev_device_driver(canvu_io_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
