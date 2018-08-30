/* ADC driver for sunxi platforms' (A10, A13 and A31) GPADC
 *
 * Copyright (c) 2016 Quentin Schulz <quentin.schulz@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 *
 * The Allwinner SoCs all have an ADC that can also act as a touchscreen
 * controller and a thermal sensor.
 * The thermal sensor works only when the ADC acts as a touchscreen controller
 * and is configured to throw an interrupt every fixed periods of time (let say
 * every X seconds).
 * One would be tempted to disable the IP on the hardware side rather than
 * disabling interrupts to save some power but that resets the internal clock of
 * the IP, resulting in having to wait X seconds every time we want to read the
 * value of the thermal sensor.
 * This is also the reason of using autosuspend in pm_runtime. If there was no
 * autosuspend, the thermal sensor would need X seconds after every
 * pm_runtime_get_sync to get a value from the ADC. The autosuspend allows the
 * thermal sensor to be requested again in a certain time span before it gets
 * shutdown for not being used.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>
#include <linux/iio/adc/sun4i-gpadc.h>

static unsigned int sun4i_gpadc_chan_select(unsigned int chan)
{
	return SUN4I_GPADC_CTRL1_ADC_CHAN_SELECT(chan);
}

static unsigned int sun6i_gpadc_chan_select(unsigned int chan)
{
	return SUN6I_GPADC_CTRL1_ADC_CHAN_SELECT(chan);
}

struct sun4i_gpadc_iio;

struct gpadc_data {
	int		temp_offset;
	int		temp_scale;
	unsigned int	tp_mode_en;
	unsigned int	tp_adc_select;
	unsigned int	(*adc_chan_select)(unsigned int chan);
	unsigned int	adc_chan_mask;
	bool		adc_channel;
	irqreturn_t	(*ths_irq_thread)(int irq, void *dev_id);
	int		(*ths_suspend)(struct sun4i_gpadc_iio *info);
	int		(*ths_resume)(struct sun4i_gpadc_iio *info);
	bool		support_irq;
	bool		has_bus_clk;
	bool		has_bus_rst;
	bool		has_mod_clk;
	u32		temp_data_base;
	int		sensor_count;
	bool		supports_nvmem;
	u32		ths_irq_clear;
};

static irqreturn_t sun4i_gpadc_data_irq_handler(int irq, void *dev_id);

static int sun4i_ths_resume(struct sun4i_gpadc_iio *info);
static int sun4i_ths_suspend(struct sun4i_gpadc_iio *info);

static int sun8i_h3_ths_resume(struct sun4i_gpadc_iio *info);
static int sun8i_h3_ths_suspend(struct sun4i_gpadc_iio *info);
static irqreturn_t sunx8i_h3_irq_thread(int irq, void *data);

static const struct gpadc_data sun4i_gpadc_data = {
	.temp_offset = -1932,
	.temp_scale = 133,
	.tp_mode_en = SUN4I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN4I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun4i_gpadc_chan_select,
	.adc_chan_mask = SUN4I_GPADC_CTRL1_ADC_CHAN_MASK,
	.adc_channel = true,
	.ths_irq_thread = sun4i_gpadc_data_irq_handler,
	.support_irq = true,
	.temp_data_base = SUN4I_GPADC_TEMP_DATA,
	.ths_resume = sun4i_ths_resume,
	.ths_suspend = sun4i_ths_suspend,
	.sensor_count = 1,
};

static const struct gpadc_data sun5i_gpadc_data = {
	.temp_offset = -1447,
	.temp_scale = 100,
	.tp_mode_en = SUN4I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN4I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun4i_gpadc_chan_select,
	.adc_chan_mask = SUN4I_GPADC_CTRL1_ADC_CHAN_MASK,
	.adc_channel = true,
	.ths_irq_thread = sun4i_gpadc_data_irq_handler,
	.support_irq = true,
	.temp_data_base = SUN4I_GPADC_TEMP_DATA,
	.ths_resume = sun4i_ths_resume,
	.ths_suspend = sun4i_ths_suspend,
	.sensor_count = 1,
};

static const struct gpadc_data sun6i_gpadc_data = {
	.temp_offset = -1623,
	.temp_scale = 167,
	.tp_mode_en = SUN6I_GPADC_CTRL1_TP_MODE_EN,
	.tp_adc_select = SUN6I_GPADC_CTRL1_TP_ADC_SELECT,
	.adc_chan_select = &sun6i_gpadc_chan_select,
	.adc_chan_mask = SUN6I_GPADC_CTRL1_ADC_CHAN_MASK,
	.adc_channel = true,
	.ths_irq_thread = sun4i_gpadc_data_irq_handler,
	.support_irq = true,
	.temp_data_base = SUN4I_GPADC_TEMP_DATA,
	.ths_resume = sun4i_ths_resume,
	.ths_suspend = sun4i_ths_suspend,
	.sensor_count = 1,
};

static const struct gpadc_data sun8i_a33_gpadc_data = {
	.temp_offset = -1662,
	.temp_scale = 162,
	.tp_mode_en = SUN8I_A33_GPADC_CTRL1_CHOP_TEMP_EN,
	.temp_data_base = SUN4I_GPADC_TEMP_DATA,
	.ths_resume = sun4i_ths_resume,
	.ths_suspend = sun4i_ths_suspend,
	.sensor_count = 1,
};

static const struct gpadc_data sun8i_h3_ths_data = {
	.temp_offset = -1791,
	.temp_scale = -121,
	.temp_data_base = SUN8I_H3_THS_TDATA0,
	.ths_irq_thread = sunx8i_h3_irq_thread,
	.support_irq = true,
	.has_bus_clk = true,
	.has_bus_rst = true,
	.has_mod_clk = true,
	.sensor_count = 1,
	.supports_nvmem = true,
	.ths_resume = sun8i_h3_ths_resume,
	.ths_suspend = sun8i_h3_ths_suspend,
	.ths_irq_clear = SUN8I_H3_THS_INTS_TDATA_IRQ_0,
};

struct sun4i_sensor_tzd {
	struct sun4i_gpadc_iio		*info;
	struct thermal_zone_device	*tzd;
	unsigned int			sensor_id;
};

struct sun4i_gpadc_iio {
	struct iio_dev			*indio_dev;
	struct completion		completion;
	int				temp_data;
	u32				adc_data;
	unsigned int			irq_data_type;
	struct regmap			*regmap;
	unsigned int			irq;
	const struct gpadc_data		*data;
	/* prevents concurrent reads of temperature and ADC */
	struct mutex			mutex;
	struct sun4i_sensor_tzd		tzds[MAX_SENSOR_COUNT];
	struct device			*sensor_device;
	struct clk			*bus_clk;
	struct clk			*mod_clk;
	struct reset_control		*reset;
	u32				calibration_data[2];
};

static const struct iio_chan_spec sun4i_gpadc_channels[] = {
	SUN4I_GPADC_ADC_CHANNEL(0, "adc_chan0"),
	SUN4I_GPADC_ADC_CHANNEL(1, "adc_chan1"),
	SUN4I_GPADC_ADC_CHANNEL(2, "adc_chan2"),
	SUN4I_GPADC_ADC_CHANNEL(3, "adc_chan3"),
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.datasheet_name = "temp_adc",
	},
};

static const struct regmap_config sun4i_gpadc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.fast_io = true,
};

static int sun4i_gpadc_irq_init(struct sun4i_gpadc_iio *info)
{
	u32 reg;

	if (info->irq_data_type == SUN4I_GPADC_IRQ_FIFO_DATA)
		reg = SUN4I_GPADC_INT_FIFOC_TEMP_IRQ_EN;
	else
		reg = SUN4I_GPADC_INT_FIFOC_TEMP_IRQ_EN;

	regmap_write(info->regmap, SUN4I_GPADC_INT_FIFOC, reg);

	return 0;
}

static int sun4i_prepare_for_irq(struct iio_dev *indio_dev, int channel,
				 unsigned int irq)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);
	int ret;
	u32 reg;

	pm_runtime_get_sync(indio_dev->dev.parent);

	reinit_completion(&info->completion);

	ret = regmap_write(info->regmap, SUN4I_GPADC_INT_FIFOC,
			   SUN4I_GPADC_INT_FIFOC_TP_FIFO_TRIG_LEVEL(1) |
			   SUN4I_GPADC_INT_FIFOC_TP_FIFO_FLUSH);
	if (ret)
		return ret;

	ret = regmap_read(info->regmap, SUN4I_GPADC_CTRL1, &reg);
	if (ret)
		return ret;

	if (irq == SUN4I_GPADC_IRQ_FIFO_DATA) {
		ret = regmap_write(info->regmap, SUN4I_GPADC_CTRL1,
				   info->data->tp_mode_en |
				   info->data->tp_adc_select |
				   info->data->adc_chan_select(channel));
		/*
		 * When the IP changes channel, it needs a bit of time to get
		 * correct values.
		 */
		if ((reg & info->data->adc_chan_mask) !=
			 info->data->adc_chan_select(channel))
			mdelay(10);

	} else {
		/*
		 * The temperature sensor returns valid data only when the ADC
		 * operates in touchscreen mode.
		 */
		ret = regmap_write(info->regmap, SUN4I_GPADC_CTRL1,
				   info->data->tp_mode_en);
	}
	if (info->data->support_irq)
		sun4i_gpadc_irq_init(info);

	if (ret)
		return ret;

	/*
	 * When the IP changes mode between ADC or touchscreen, it
	 * needs a bit of time to get correct values.
	 */
	if ((reg & info->data->tp_adc_select) != info->data->tp_adc_select)
		mdelay(100);

	return 0;
}

static int sun4i_gpadc_read(struct iio_dev *indio_dev, int channel, int *val,
			    unsigned int irq)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);
	int ret;

	mutex_lock(&info->mutex);

	info->irq_data_type = irq;
	ret = sun4i_prepare_for_irq(indio_dev, channel, irq);
	if (ret)
		goto err;

	enable_irq(info->irq);

	/*
	 * The temperature sensor throws an interruption periodically (currently
	 * set at periods of ~0.6s in sun4i_gpadc_runtime_resume). A 1s delay
	 * makes sure an interruption occurs in normal conditions. If it doesn't
	 * occur, then there is a timeout.
	 */
	if (!wait_for_completion_timeout(&info->completion,
					 msecs_to_jiffies(1000))) {
		ret = -ETIMEDOUT;
		goto err;
	}

	if (irq == SUN4I_GPADC_IRQ_FIFO_DATA)
		*val = info->adc_data;
	else
		*val = info->temp_data;

	ret = 0;
	pm_runtime_mark_last_busy(indio_dev->dev.parent);

err:
	pm_runtime_put_autosuspend(indio_dev->dev.parent);
	disable_irq(info->irq);
	mutex_unlock(&info->mutex);

	return ret;
}

static int sun4i_gpadc_adc_read(struct iio_dev *indio_dev, int channel,
				int *val)
{
	return sun4i_gpadc_read(indio_dev, channel, val,
			SUN4I_GPADC_IRQ_FIFO_DATA);
}

static int sun4i_gpadc_temp_read(struct iio_dev *indio_dev, int *val,
				int sensor)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);

	if (info->data->adc_channel)
		return sun4i_gpadc_read(indio_dev, 0, val,
				SUN4I_GPADC_IRQ_TEMP_DATA);

	pm_runtime_get_sync(indio_dev->dev.parent);

	regmap_read(info->regmap, info->data->temp_data_base + 0x4 * sensor,
			val);

	pm_runtime_mark_last_busy(indio_dev->dev.parent);
	pm_runtime_put_autosuspend(indio_dev->dev.parent);

	return 0;
}

static int sun4i_gpadc_temp_offset(struct iio_dev *indio_dev, int *val)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);

	*val = info->data->temp_offset;

	return 0;
}

static int sun4i_gpadc_temp_scale(struct iio_dev *indio_dev, int *val)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);

	*val = info->data->temp_scale;

	return 0;
}

static int sun4i_gpadc_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan, int *val,
				int *val2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		ret = sun4i_gpadc_temp_offset(indio_dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_VOLTAGE)
			ret = sun4i_gpadc_adc_read(indio_dev, chan->channel,
						   val);
		else
			ret = sun4i_gpadc_temp_read(indio_dev, val, 0);

		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_VOLTAGE) {
			/* 3000mV / 4096 * raw */
			*val = 0;
			*val2 = 732421875;
			return IIO_VAL_INT_PLUS_NANO;
		}

		ret = sun4i_gpadc_temp_scale(indio_dev, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static const struct iio_info sun4i_gpadc_iio_info = {
	.read_raw = sun4i_gpadc_read_raw,
};

static irqreturn_t sun4i_gpadc_data_irq_handler(int irq, void *dev_id)
{
	struct sun4i_gpadc_iio *info = dev_id;

	if (info->irq_data_type == SUN4I_GPADC_IRQ_FIFO_DATA) {
		/* read fifo data */
		if (!regmap_read(info->regmap, SUN4I_GPADC_DATA,
					&info->adc_data))
			complete(&info->completion);
	} else {
		/* read temp data */
		if (!regmap_read(info->regmap, SUN4I_GPADC_TEMP_DATA,
					&info->temp_data))
			complete(&info->completion);
	}
	return IRQ_HANDLED;
}

static irqreturn_t sunx8i_h3_irq_thread(int irq, void *data)
{
	struct sun4i_gpadc_iio *info = data;
	int i;

	regmap_write(info->regmap, SUN8I_H3_THS_STAT,
			info->data->ths_irq_clear);

	for (i = 0; i < info->data->sensor_count; i++)
		thermal_zone_device_update(info->tzds[i].tzd,
						THERMAL_EVENT_TEMP_SAMPLE);

	return IRQ_HANDLED;
}

static int sun8i_h3_calibrate(struct sun4i_gpadc_iio *info)
{
//	regmap_write(info->regmap, SUNXI_THS_CDATA_0_1,
//			info->calibration_data[0]);
//	regmap_write(info->regmap, SUNXI_THS_CDATA_2_3,
//			info->calibration_data[1]);

	return 0;
}

static int sun4i_gpadc_runtime_suspend(struct device *dev)
{
	struct sun4i_gpadc_iio *info = iio_priv(dev_get_drvdata(dev));
	return info->data->ths_suspend(info);
}

static int sun4i_ths_suspend(struct sun4i_gpadc_iio *info)
{

	/* Disable the ADC on IP */
	regmap_write(info->regmap, SUN4I_GPADC_CTRL1, 0);
	/* Disable temperature sensor on IP */
	regmap_write(info->regmap, SUN4I_GPADC_TPR, 0);
	/* Disable irq*/
	regmap_write(info->regmap, SUN4I_GPADC_INT_FIFOC, 0);

	return 0;
}

static int sun8i_h3_ths_suspend(struct sun4i_gpadc_iio *info)
{
	/* Disable ths interrupt */
	regmap_write(info->regmap, SUN8I_H3_THS_INTC, 0x0);
	/* Disable temperature sensor */
	regmap_write(info->regmap, SUN8I_H3_THS_CTRL2, 0x0);

	return 0;
}

static int sun4i_gpadc_runtime_resume(struct device *dev)
{
	struct sun4i_gpadc_iio *info = iio_priv(dev_get_drvdata(dev));
	return info->data->ths_resume(info);
}

static int sun4i_ths_resume(struct sun4i_gpadc_iio *info)
{
	/* clkin = 6MHz */
	regmap_write(info->regmap, SUN4I_GPADC_CTRL0,
		     SUN4I_GPADC_CTRL0_ADC_CLK_DIVIDER(2) |
		     SUN4I_GPADC_CTRL0_FS_DIV(7) |
		     SUN4I_GPADC_CTRL0_T_ACQ(63));
	regmap_write(info->regmap, SUN4I_GPADC_CTRL1, info->data->tp_mode_en);
	regmap_write(info->regmap, SUN4I_GPADC_CTRL3,
		     SUN4I_GPADC_CTRL3_FILTER_EN |
		     SUN4I_GPADC_CTRL3_FILTER_TYPE(1));
	/* period = SUN4I_GPADC_TPR_TEMP_PERIOD * 256 * 16 / clkin; ~0.6s */
	regmap_write(info->regmap, SUN4I_GPADC_TPR,
		     SUN4I_GPADC_TPR_TEMP_ENABLE |
		     SUN4I_GPADC_TPR_TEMP_PERIOD(800));


	return 0;
}

static int sun8i_h3_ths_resume(struct sun4i_gpadc_iio *info)
{
	u32 value;

	sun8i_h3_calibrate(info);

	regmap_write(info->regmap, SUN8I_H3_THS_CTRL0,
			SUN4I_GPADC_CTRL0_T_ACQ(0xff));

	regmap_write(info->regmap, SUN8I_H3_THS_CTRL2,
			SUN8I_H3_THS_ACQ1(0x3f));

	regmap_write(info->regmap, SUN8I_H3_THS_STAT,
			SUN8I_H3_THS_INTS_TDATA_IRQ_0);

	regmap_write(info->regmap, SUN8I_H3_THS_FILTER,
			SUN4I_GPADC_CTRL3_FILTER_EN |
			SUN4I_GPADC_CTRL3_FILTER_TYPE(0x2));

	regmap_write(info->regmap, SUN8I_H3_THS_INTC,
			SUN8I_H3_THS_INTC_TDATA_IRQ_EN0 |
			SUN8I_H3_THS_TEMP_PERIOD(0x55));

	regmap_read(info->regmap, SUN8I_H3_THS_CTRL2, &value);

	regmap_write(info->regmap, SUN8I_H3_THS_CTRL2,
			SUN8I_H3_THS_TEMP_SENSE_EN0 | value);

	return 0;
}

static int sun4i_gpadc_get_temp(void *data, int *temp)
{
	struct sun4i_sensor_tzd *tzd = data;
	struct sun4i_gpadc_iio *info = tzd->info;
	int val, scale, offset;

	if (sun4i_gpadc_temp_read(info->indio_dev, &val, tzd->sensor_id))
		return -ETIMEDOUT;

	sun4i_gpadc_temp_scale(info->indio_dev, &scale);
	sun4i_gpadc_temp_offset(info->indio_dev, &offset);

	*temp = (val + offset) * scale;

	return 0;
}

static const struct thermal_zone_of_device_ops sun4i_ts_tz_ops = {
	.get_temp = &sun4i_gpadc_get_temp,
};

static const struct dev_pm_ops sun4i_gpadc_pm_ops = {
	.runtime_suspend = &sun4i_gpadc_runtime_suspend,
	.runtime_resume = &sun4i_gpadc_runtime_resume,
};

static const struct of_device_id sun4i_gpadc_of_id[] = {
	{
		.compatible = "allwinner,sun8i-a33-ths",
		.data = &sun8i_a33_gpadc_data,
	},
	{
		.compatible = "allwinner,sun4i-a10-gpadc",
		.data = &sun4i_gpadc_data
	},
	{
		.compatible = "allwinner,sun5i-a13-gpadc",
		.data = &sun5i_gpadc_data
	},
	{
		.compatible = "allwinner,sun6i-a31-gpadc",
		.data = &sun6i_gpadc_data
	},
	{
		.compatible = "allwinner,sun8i-h3-ths",
		.data = &sun8i_h3_ths_data,
	},
	{ /* sentinel */ }
};

static int sun4i_gpadc_probe_dt(struct platform_device *pdev,
				struct iio_dev *indio_dev)
{
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);
	struct resource *mem;
	void __iomem *base;
	int ret;
	struct nvmem_cell *cell;
	ssize_t cell_size;
	u32 *cell_data;

	info->data = of_device_get_match_data(&pdev->dev);
	if (!info->data)
		return -ENODEV;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(base))
		return PTR_ERR(base);

	if (info->data->supports_nvmem) {

		cell = nvmem_cell_get(&pdev->dev, "calibration");
		if (IS_ERR(cell)) {
			if (PTR_ERR(cell) == -EPROBE_DEFER)
				return PTR_ERR(cell);
		} else {
			cell_data = (u32 *)nvmem_cell_read(cell, &cell_size);
			if (cell_size != 8)
				dev_err(&pdev->dev,
					"Calibration data has wrong size\n");
			else {
				info->calibration_data[0] = cell_data[0];
				info->calibration_data[1] = cell_data[1];
			}
		}
	}

	if (info->data->has_bus_clk)
		info->regmap = devm_regmap_init_mmio_clk(&pdev->dev, "bus",
				base, &sun4i_gpadc_regmap_config);
	else
		info->regmap = devm_regmap_init_mmio(&pdev->dev, base,
				&sun4i_gpadc_regmap_config);

	if (IS_ERR(info->regmap)) {
		ret = PTR_ERR(info->regmap);
		dev_err(&pdev->dev, "failed to init regmap: %d\n", ret);
		return ret;
	}

	if (info->data->support_irq) {

		/* ths interrupt */
		info->irq = platform_get_irq(pdev, 0);

		ret = devm_request_threaded_irq(&pdev->dev, info->irq,
				NULL, info->data->ths_irq_thread,
				IRQF_ONESHOT, dev_name(&pdev->dev), info);

		if (info->data->adc_channel)
			disable_irq(info->irq);

		if (ret) {
			dev_err(&pdev->dev, "failed to add ths irq: %d\n", ret);
			return ret;
		}
	}

	if (info->data->has_bus_rst) {
		info->reset = devm_reset_control_get(&pdev->dev, NULL);
		if (IS_ERR(info->reset)) {
			ret = PTR_ERR(info->reset);
			return ret;
		}

		ret = reset_control_deassert(info->reset);
		if (ret)
			return ret;
	}

	if (info->data->has_bus_clk) {
		info->bus_clk = devm_clk_get(&pdev->dev, "bus");
		if (IS_ERR(info->bus_clk)) {
			ret = PTR_ERR(info->bus_clk);
			goto assert_reset;
		}

		ret = clk_prepare_enable(info->bus_clk);
		if (ret)
			goto assert_reset;
	}

	if (info->data->has_mod_clk) {
		info->mod_clk = devm_clk_get(&pdev->dev, "mod");
		if (IS_ERR(info->mod_clk)) {
			ret = PTR_ERR(info->mod_clk);
			goto disable_bus_clk;
		}

		/* Running at 4MHz */
		ret = clk_set_rate(info->mod_clk, 4000000);
		if (ret)
			goto disable_bus_clk;

		ret = clk_prepare_enable(info->mod_clk);
		if (ret)
			goto disable_bus_clk;
	}

	info->sensor_device = &pdev->dev;

	return 0;

disable_bus_clk:
	clk_disable_unprepare(info->bus_clk);

assert_reset:
	reset_control_assert(info->reset);

	return ret;
}

static int sun4i_gpadc_probe(struct platform_device *pdev)
{
	struct sun4i_gpadc_iio *info;
	struct iio_dev *indio_dev;
	int ret, i;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);
	platform_set_drvdata(pdev, indio_dev);

	mutex_init(&info->mutex);
	info->indio_dev = indio_dev;
	init_completion(&info->completion);
	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->dev.of_node = pdev->dev.of_node;
	indio_dev->info = &sun4i_gpadc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (&info->data->adc_channel) {
		indio_dev->num_channels = ARRAY_SIZE(sun4i_gpadc_channels);
		indio_dev->channels = sun4i_gpadc_channels;
	}

	ret = sun4i_gpadc_probe_dt(pdev, indio_dev);

	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 SUN4I_GPADC_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	for (i = 0; i < info->data->sensor_count; i++) {
		info->tzds[i].info = info;
		info->tzds[i].sensor_id = i;

		info->tzds[i].tzd = thermal_zone_of_sensor_register(
				info->sensor_device,
				i, &info->tzds[i], &sun4i_ts_tz_ops);
		/*
		 * Do not fail driver probing when failing to register in
		 * thermal because no thermal DT node is found.
		 */
		if (IS_ERR(info->tzds[i].tzd) && \
				PTR_ERR(info->tzds[i].tzd) != -ENODEV) {
			dev_err(&pdev->dev,
				"could not register thermal sensor: %ld\n",
				PTR_ERR(info->tzds[i].tzd));
			return PTR_ERR(info->tzds[i].tzd);
		}
	}

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register the device\n");
		goto err_map;
	}

	return 0;

err_map:
	if (!info->data->support_irq)
		iio_map_array_unregister(indio_dev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int sun4i_gpadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct sun4i_gpadc_iio *info = iio_priv(indio_dev);
	int i;

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	for (i = 0; i < info->data->sensor_count; i++)
		thermal_zone_of_sensor_unregister(info->sensor_device,
						  info->tzds[i].tzd);

	if (!info->data->support_irq)
		iio_map_array_unregister(indio_dev);

	clk_disable_unprepare(info->mod_clk);

	clk_disable_unprepare(info->bus_clk);

	reset_control_assert(info->reset);

	return 0;
}

static struct platform_driver sun4i_gpadc_driver = {
	.driver = {
		.name = "sun4i-gpadc-iio",
		.of_match_table = sun4i_gpadc_of_id,
		.pm = &sun4i_gpadc_pm_ops,
	},
	.probe = sun4i_gpadc_probe,
	.remove = sun4i_gpadc_remove,
};
MODULE_DEVICE_TABLE(of, sun4i_gpadc_of_id);

module_platform_driver(sun4i_gpadc_driver);

MODULE_DESCRIPTION("ADC driver for sunxi platforms");
MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_LICENSE("GPL v2");
