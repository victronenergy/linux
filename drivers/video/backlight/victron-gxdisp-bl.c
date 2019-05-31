/*
 * Copyright (C) 2019 Mans Rullgard <mans@mansr.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/ctype.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>

/*
 * +------+------+----+----------------------------------+
 * | addr | size |    | function                         |
 * +------+------+----+----------------------------------+
 * |  00  |  2   | R  | preamble                         |
 * |  02  |  1   | R  | size                             |
 * |  03  | 10   | R  | serial                           |
 * |  0d  |  1   | R  | sw_rev                           |
 * |  0e  |  2   | R  | adc_value                        |
 * |  10  |  9   | RW | adc_map                          |
 * |  19  |  1   | R  | adc_8bit_value                   |
 * |  1a  |  1   | RW | pwm_value                        |
 * |  1b  |  2   | R  | temp                             |
 * |  1d  |  1   | RW | 0: output_enable                 |
 * |      |      |    | 1: use_adc_map                   |
 * |      |      |    | 2: disable_signal_detection      |
 * |  1e  |  1   | RW | adc_shift                        |
 * |  1f  |  1   | RW | iir_coeff                        |
 * |  20  |  1   | RW | pwm_hysteresis                   |
 * +------+------+----+----------------------------------+
 */

#define GXBL_PREAMBLE		0x00
#define GXBL_SIZE		0x02
#define GXBL_SERIAL		0x03
#define GXBL_SWREV		0x0d
#define GXBL_ADC_VAL		0x0e
#define GXBL_ADC_MAP		0x10
#define GXBL_ADC_VAL8		0x19
#define GXBL_PWM_VAL		0x1a
#define GXBL_TEMP		0x1b
#define GXBL_CTRL		0x1d
#define GXBL_ADC_SHIFT		0x1e
#define GXBL_IIR_COEFF		0x1f
#define GXBL_PWM_HYST		0x20

#define GXBL_CTRL_EN		BIT(0)
#define GXBL_CTRL_AUTO		BIT(1)
#define GXBL_CTRL_FORCE_EN	BIT(2)

#define GXBL_ADC_MAP_SIZE	9

static const u8 gxbl_brightness_levels[] = {
	0, 2, 4, 8, 13, 20, 29, 40, 54, 72, 92, 116, 145, 177, 214, 255,
};

struct gxbl {
	struct i2c_client *i2c;
	struct backlight_device *bl;
	const u8 *levels;
	bool auto_brightness;
};

static int gxbl_read(struct gxbl *gxbl, u8 addr, void *buf, size_t len)
{
	int ret = i2c_smbus_read_i2c_block_data(gxbl->i2c, addr, len, buf);

	if (ret < 0)
		return ret;

	if (ret < len)
		return -EIO;

	return 0;
}

static int gxbl_write(struct gxbl *gxbl, u8 addr, void *buf, size_t len)
{
	return i2c_smbus_write_i2c_block_data(gxbl->i2c, addr, len, buf);
}

static int gxbl_read8(struct gxbl *gxbl, int addr)
{
	return i2c_smbus_read_byte_data(gxbl->i2c, addr);
}

static int gxbl_read16(struct gxbl *gxbl, int addr)
{
	return i2c_smbus_read_word_data(gxbl->i2c, addr);
}

static int gxbl_write8(struct gxbl *gxbl, int addr, int val)
{
	return i2c_smbus_write_byte_data(gxbl->i2c, addr, val);
}

static int gxbl_update(struct backlight_device *bl)
{
	struct gxbl *gxbl = bl_get_data(bl);
	int brightness = gxbl->levels[bl->props.brightness];
	int power = 1;
	int ctrl = 0;

	if (bl->props.power != FB_BLANK_UNBLANK)
		power = 0;

	if (bl->props.state & (BL_CORE_FBBLANK | BL_CORE_SUSPENDED))
		power = 0;

	if (power)
		ctrl |= GXBL_CTRL_EN;

	if (gxbl->auto_brightness)
		ctrl |= GXBL_CTRL_AUTO;

	gxbl_write8(gxbl, GXBL_CTRL, ctrl);

	if (!gxbl->auto_brightness)
		gxbl_write8(gxbl, GXBL_PWM_VAL, brightness);

	return 0;
}

static struct backlight_ops gxbl_ops = {
	.update_status	= gxbl_update,
};

static ssize_t gxbl_show8(struct device *dev, char *buf, int addr)
{
	struct gxbl *gxbl = bl_get_data(to_backlight_device(dev));
	int val;

	val = gxbl_read8(gxbl, addr);
	if (val < 0)
		return val;

	return sprintf(buf, "%d\n", val);
}

static ssize_t gxbl_store8(struct device *dev, const char *buf, size_t count,
			   int addr, int min, int max)
{
	struct gxbl *gxbl = bl_get_data(to_backlight_device(dev));
	unsigned long val;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	if (val < min || val > max)
		return -EINVAL;

	err = gxbl_write8(gxbl, addr, val);
	if (err)
		return err;

	return count;
}

static ssize_t gxbl_show16(struct device *dev, char *buf, int addr,
			   int scale, int offset)
{
	struct gxbl *gxbl = bl_get_data(to_backlight_device(dev));
	int val;

	val = gxbl_read16(gxbl, addr);
	if (val < 0)
		return val;

	return sprintf(buf, "%d\n", val * scale + offset);
}

static ssize_t adc_value_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return gxbl_show16(dev, buf, GXBL_ADC_VAL, 1, 0);
}
static DEVICE_ATTR_RO(adc_value);

static ssize_t pwm_value_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return gxbl_show8(dev, buf, GXBL_PWM_VAL);
}
static DEVICE_ATTR_RO(pwm_value);

static ssize_t temp_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	return gxbl_show16(dev, buf, GXBL_TEMP, 1000, -273150);
}
static DEVICE_ATTR_RO(temp);

static ssize_t auto_brightness_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct gxbl *gxbl = bl_get_data(to_backlight_device(dev));
	int ctrl;

	ctrl = gxbl_read8(gxbl, GXBL_CTRL);
	if (ctrl < 0)
		return ctrl;

	return sprintf(buf, "%d\n", !!(ctrl & GXBL_CTRL_AUTO));
}

static ssize_t auto_brightness_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gxbl *gxbl = bl_get_data(bl);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	if (val > 1)
		return -EINVAL;

	gxbl->auto_brightness = val;
	gxbl_update(bl);

	return count;
}
static DEVICE_ATTR_RW(auto_brightness);

static ssize_t adc_map_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct gxbl *gxbl = bl_get_data(to_backlight_device(dev));
	u8 adc_map[GXBL_ADC_MAP_SIZE];
	char *p = buf;
	int err;
	int i;

	err = gxbl_read(gxbl, GXBL_ADC_MAP, adc_map, sizeof(adc_map));
	if (err)
		return err;

	for (i = 0; i < GXBL_ADC_MAP_SIZE; i++)
		p += sprintf(p, "%d ", adc_map[i]);

	p[-1] = '\n';

	return p - buf;
}

static ssize_t adc_map_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct backlight_device *bl = to_backlight_device(dev);
	struct gxbl *gxbl = bl_get_data(bl);
	u8 val[GXBL_ADC_MAP_SIZE];
	const char *p = buf;
	char *e;
	int err;
	int i;

	for (i = 0; i < GXBL_ADC_MAP_SIZE; i++) {
		val[i] = simple_strtoul(p, &e, 0);
		if (!isspace(*e))
			break;
		p = ++e;
	}

	if (i < GXBL_ADC_MAP_SIZE || *e)
		return -EINVAL;

	err = gxbl_write(gxbl, GXBL_ADC_MAP, val, GXBL_ADC_MAP_SIZE);
	if (err)
		return err;

	return count;
}
static DEVICE_ATTR_RW(adc_map);

static ssize_t adc_shift_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return gxbl_show8(dev, buf, GXBL_ADC_SHIFT);
}

static ssize_t adc_shift_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	return gxbl_store8(dev, buf, count, GXBL_ADC_SHIFT, 0, 15);
}
static DEVICE_ATTR_RW(adc_shift);

static ssize_t iir_coeff_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return gxbl_show8(dev, buf, GXBL_IIR_COEFF);
}

static ssize_t iir_coeff_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	return gxbl_store8(dev, buf, count, GXBL_IIR_COEFF, 0, 255);
}
static DEVICE_ATTR_RW(iir_coeff);

static ssize_t pwm_hyst_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	return gxbl_show8(dev, buf, GXBL_PWM_HYST);
}

static ssize_t pwm_hyst_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	return gxbl_store8(dev, buf, count, GXBL_PWM_HYST, 0, 255);
}
static DEVICE_ATTR_RW(pwm_hyst);

static struct attribute *gxbl_attrs[] = {
	&dev_attr_adc_map.attr,
	&dev_attr_adc_shift.attr,
	&dev_attr_adc_value.attr,
	&dev_attr_auto_brightness.attr,
	&dev_attr_iir_coeff.attr,
	&dev_attr_pwm_hyst.attr,
	&dev_attr_pwm_value.attr,
	&dev_attr_temp.attr,
	NULL,
};

static const struct attribute_group gxbl_attr_group = {
	.attrs = gxbl_attrs,
};

static int gxbl_probe(struct i2c_client *i2c)
{
	struct device_node *np = i2c->dev.of_node;
	struct gxbl *gxbl;
	struct backlight_device *bl;
	char name[32];
	bool power;
	u32 brightness;
	u32 max_br;
	int ctrl;
	int rev;
	int err;

	gxbl = devm_kzalloc(&i2c->dev, sizeof(*gxbl), GFP_KERNEL);
	if (!gxbl)
		return -ENOMEM;

	snprintf(name, sizeof(name), "%s-%s", "gxdisp", dev_name(&i2c->dev));

	bl = devm_backlight_device_register(&i2c->dev, name, &i2c->dev, gxbl,
					    &gxbl_ops, NULL);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	gxbl->i2c = i2c;
	gxbl->bl = bl;

	i2c_set_clientdata(i2c, gxbl);

	rev = gxbl_read8(gxbl, GXBL_SWREV);
	if (rev < 0)
		return rev;

	ctrl = gxbl_read8(gxbl, GXBL_CTRL);
	if (ctrl < 0)
		return ctrl;

	brightness = gxbl_read8(gxbl, GXBL_PWM_VAL);
	gxbl->levels = gxbl_brightness_levels;
	max_br = ARRAY_SIZE(gxbl_brightness_levels) - 1;

	power = of_property_read_bool(np, "default-on");
	gxbl->auto_brightness = of_property_read_bool(np, "auto-brightness");

	if (!of_property_read_u32(np, "default-brightness", &brightness))
		gxbl->auto_brightness = false;

	bl->props.brightness = min(brightness, max_br);
	bl->props.max_brightness = max_br;
	bl->props.power = power ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;

	err = sysfs_create_group(&bl->dev.kobj, &gxbl_attr_group);
	if (err)
		return err;

	gxbl_update(bl);

	dev_info(&bl->dev, "GX Display backlight rev %d\n", rev);

	return 0;
}

static int gxbl_remove(struct i2c_client *i2c)
{
	struct gxbl *gxbl = i2c_get_clientdata(i2c);

	sysfs_remove_group(&gxbl->bl->dev.kobj, &gxbl_attr_group);

	return 0;
}

static const struct of_device_id gxbl_dt_ids[] = {
	{ .compatible = "victronenergy,gxdisp-backlight" },
	{ }
};
MODULE_DEVICE_TABLE(of, gxbl_dt_ids);

static struct i2c_driver gxbl_driver = {
	.probe_new	= gxbl_probe,
	.remove		= gxbl_remove,
	.driver		= {
		.name		= "gxdisp_bl",
		.of_match_table	= gxbl_dt_ids,
	},
};
module_i2c_driver(gxbl_driver);

MODULE_DESCRIPTION("Victron GX Display backlight driver");
MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_LICENSE("GPL");
