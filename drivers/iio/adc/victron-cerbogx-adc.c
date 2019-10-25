// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Mans Rullgard <mans@mansr.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

#define MIN_FW_REV	4
#define MAX_FW_REV	5

#define NUM_CHANNELS	9
#define MAX_AGE_MS	500

struct cerbo_adc_config {
	u8	id[2];
	__le16	size;
	u8	serial[16];
	__le16	fw_rev;
	__le16	hw_rev;
} __packed;

struct cerbo_adc_data {
	__le16	count;
	__le16	channels[2][9];
} __packed;

struct cerbo_adc {
	struct i2c_client *i2c;
	int values[NUM_CHANNELS];
	unsigned long time;
	struct cerbo_adc_data data;
	int fwversion;
};

#define CERBO_ADC_CHANNEL(i)  {					\
	.type			= IIO_VOLTAGE,			\
	.indexed		= 1,				\
	.channel		= i,				\
	.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW),	\
}

static const struct iio_chan_spec cerbo_adc_channels[] = {
	CERBO_ADC_CHANNEL(0),
	CERBO_ADC_CHANNEL(1),
	CERBO_ADC_CHANNEL(2),
	CERBO_ADC_CHANNEL(3),
	CERBO_ADC_CHANNEL(4),
	CERBO_ADC_CHANNEL(5),
	CERBO_ADC_CHANNEL(6),
	CERBO_ADC_CHANNEL(7),
	CERBO_ADC_CHANNEL(8),
};

static int cerbo_adc_valid(struct cerbo_adc *cadc)
{
	return time_before(jiffies, cadc->time + msecs_to_jiffies(MAX_AGE_MS));
}

static int cerbo_adc_update(struct cerbo_adc *cadc)
{
	struct i2c_client *client = cadc->i2c;
	u8 data_addr = sizeof(struct cerbo_adc_config);
	struct i2c_msg msg[2] = {
		{
			.addr	= client->addr,
			.len	= 1,
			.buf	= &data_addr,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= sizeof(cadc->data),
			.buf	= (u8 *)&cadc->data,
		}
	};
	int buf;
	int ch;
	int i;

	if (i2c_transfer(client->adapter, msg, 2) != 2)
		return -EIO;

	buf = le16_to_cpu(cadc->data.count) & 1;
	ch = 0;

	for (i = 0; i < NUM_CHANNELS; i++)
		cadc->values[ch++] = le16_to_cpu(cadc->data.channels[buf][i]);

	cadc->time = jiffies;

	return 0;
}

static int cerbo_adc_read_raw(struct iio_dev *iio,
			      const struct iio_chan_spec *chan,
			      int *val, int *val2, long m)
{
	struct cerbo_adc *cadc = iio_priv(iio);
	int err;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		err = iio_device_claim_direct_mode(iio);
		if (err)
			return err;

		if (!cerbo_adc_valid(cadc))
			err = cerbo_adc_update(cadc);
		iio_device_release_direct_mode(iio);
		if (err)
			return err;

		*val = cadc->values[chan->channel];

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static const struct iio_info cerbo_adc_info = {
	.read_raw = &cerbo_adc_read_raw,
};

static int cerbo_adc_validate(struct i2c_client *client, int *fwversion)
{
	struct device *dev = &client->dev;
	struct cerbo_adc_config cfg;
	struct gpio_desc *rst;
	int fw_rev;
	int hw_rev;
	int ret;

	rst = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (rst) {
		msleep(10);
		gpiod_set_value(rst, 1);
		msleep(250);
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0, sizeof(cfg), (u8*)&cfg);

	if (ret < 0) {
		dev_err(dev, "i2c read error\n");
		return ret;
	}

	if (ret < sizeof(cfg)) {
		dev_err(dev, "short i2c read\n");
		return -EIO;
	}

	if (cfg.id[0] != 'V' && cfg.id[1] != 'E') {
		dev_err(dev, "bad magic\n");
		return -ENODEV;
	}

	if (le16_to_cpu(cfg.size) <
	    sizeof(struct cerbo_adc_config) + sizeof(struct cerbo_adc_data) -
	    offsetof(struct cerbo_adc_config, serial)) {
		dev_err(dev, "bad data size\n");
		return -ENODEV;
	}

	fw_rev = le16_to_cpu(cfg.fw_rev);
	hw_rev = le16_to_cpu(cfg.hw_rev);

	if (fw_rev < MIN_FW_REV || fw_rev > MAX_FW_REV) {
		dev_err(dev, "unsupported firmware version %d\n", fw_rev);
		return -ENODEV;
	}

	dev_info(dev, "Cerbo GX ADC hw %d, fw %d\n", hw_rev, fw_rev);

	*fwversion = fw_rev;

	return 0;
}

static int cerbo_adc_probe(struct i2c_client *client)
{
	struct cerbo_adc *cadc;
	struct iio_dev *iio;
	int fwversion;
	int err;

	err = cerbo_adc_validate(client, &fwversion);
	if (err) {
		dev_err(&client->dev, "probe failed: %d\n", err);
		return err;
	}

	iio = devm_iio_device_alloc(&client->dev, sizeof(*cadc));
	if (!iio)
		return -ENOMEM;

	i2c_set_clientdata(client, iio);

	cadc = iio_priv(iio);
	cadc->i2c = client;
	cadc->fwversion = fwversion;

	iio->dev.parent = &client->dev;
	iio->dev.of_node = client->dev.of_node;
	iio->name = client->name;
	iio->info = &cerbo_adc_info;
	iio->modes = INDIO_DIRECT_MODE;
	iio->channels = cerbo_adc_channels;
	iio->num_channels = ARRAY_SIZE(cerbo_adc_channels);

	cerbo_adc_update(cadc);

	err = devm_iio_device_register(&client->dev, iio);
	if (err)
		return err;

	return 0;
}

static ssize_t fwversion_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct cerbo_adc *cadc = iio_priv(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%d\n", cadc->fwversion);
}

static DEVICE_ATTR_RO(fwversion);

static struct attribute *cerbo_adc_attrs[] = {
	&dev_attr_fwversion.attr,
	NULL,
};

ATTRIBUTE_GROUPS(cerbo_adc);

static const struct of_device_id cerbo_adc_dt_ids[] = {
	{ .compatible = "victronenergy,cerbo-gx-adc" },
	{}
};
MODULE_DEVICE_TABLE(of, cerbo_adc_dt_ids);

static struct i2c_driver cerbo_adc_driver = {
	.probe		= cerbo_adc_probe,
	.driver = {
		.name		= "cerbo-gx-adc",
		.of_match_table	= cerbo_adc_dt_ids,
		.dev_groups	= cerbo_adc_groups,
	},
};
module_i2c_driver(cerbo_adc_driver);

MODULE_AUTHOR("Mans Rullgard <mans@mansr.com>");
MODULE_DESCRIPTION("Victron Energy Cerbo GX ADC");
MODULE_LICENSE("GPL v2");
