#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define MAX_POINTS		10
#define POLL_PERIOD		20

/* Touchscreen commands */
#define REG_TOUCHDATA		0x10
#define REG_PANEL_INFO		0x20
#define REG_FIRMWARE_VERSION	0x40
#define REG_PROTOCOL_VERSION	0x42
#define REG_CALIBRATE		0xcc

struct point_v1 {
	__le16 x;
	__le16 y;
} __packed;

struct touchdata_v1 {
	u8 status;
	struct point_v1 points[];
} __packed;

struct point_v2 {
	u8 status;
	__be16 x;
	__be16 y;
} __packed;

struct touchdata_v2 {
	u8 npoints;
	struct point_v2 points[];
} __packed;

struct point_v3 {
	__be16 x;
	__be16 y;
	u8 pressure;
} __packed;

struct touchdata_v3 {
	u8 status;
	struct point_v3 points[];
} __packed;

struct panel_info {
	__le16 xmax;
	__le16 ymax;
	u8 xchannel_num;
	u8 ychannel_num;

	/* protocol version 2/3 only */
	u8 max_points;
	u8 touchkey_channel_num;
	u8 touchkey_low;
	u8 touchkey_high;	/* always 0xff */
} __packed;

struct firmware_version {
	u8 fver[4];		/* internal firmware version */
	u8 cver[4];		/* customer firmware version */
} __packed;

struct protocol_version {
	u8 major;
	u8 minor;
	u8 release;
} __packed;

struct ili210x;

struct protocol_info {
	unsigned int version_size;
	unsigned int info_size;
	size_t data_size;
	size_t point_size;
	int (*read_state)(struct ili210x *);
};

struct ili210x {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work dwork;
	struct touchscreen_properties prop;
	int slots[MAX_POINTS];
	struct input_mt_pos pos[MAX_POINTS];

	const struct protocol_info *proto;
	unsigned int max_points;
	size_t data_size;
	void *data_buf;
};

static int ili210x_read_reg(struct i2c_client *client, u8 reg, void *buf,
			    size_t len)
{
	struct i2c_msg msg[2] = {
		{
			.addr	= client->addr,
			.flags	= I2C_M_STOP,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	if (i2c_transfer(client->adapter, msg, 2) != 2) {
		dev_err(&client->dev, "i2c transfer failed\n");
		return -EIO;
	}

	return 0;
}

static int ili210x_read_state_v1(struct ili210x *priv)
{
	struct i2c_client *client = priv->client;
	struct touchdata_v1 *touchdata = priv->data_buf;
	int i;
	unsigned int x, y;
	int np = 0;
	int error;

	error = ili210x_read_reg(client, REG_TOUCHDATA, touchdata,
				 priv->data_size);
	if (error)
		return error;

	for (i = 0; i < priv->max_points; i++) {
		struct point_v1 *p = &touchdata->points[i];

		if (!(touchdata->status & (1 << i)))
			continue;

		x = le16_to_cpu(p->x);
		y = le16_to_cpu(p->y);

		touchscreen_set_mt_pos(&priv->pos[np++], &priv->prop, x, y);
	}

	return np;
}

static int ili210x_read_state_v2(struct ili210x *priv)
{
	struct i2c_client *client = priv->client;
	struct touchdata_v2 *td = priv->data_buf;
	int np = 0;
	int error;
	int i;

	error = ili210x_read_reg(client, REG_TOUCHDATA, &td->npoints, 1);
	if (error)
		return error;

	if (!td->npoints)
		return 0;

	if (td->npoints > priv->max_points) {
		dev_warn(&client->dev, "too many points: %d\n", td->npoints);
		td->npoints = priv->max_points;
	}

	error = ili210x_read_reg(client, REG_TOUCHDATA + 1, &td->points,
				 td->npoints * sizeof(td->points[0]));
	if (error)
		return error;

	for (i = 0; i < td->npoints; i++) {
		struct point_v2 *p = &td->points[i];
		unsigned int x, y;

		if ((p->status & 0xc0) != 0x80)
			continue;

		x = be16_to_cpu(p->x);
		y = be16_to_cpu(p->y);

		touchscreen_set_mt_pos(&priv->pos[np++], &priv->prop, x, y);
	}

	return np;
}

static int ili210x_read_state_v3(struct ili210x *priv)
{
	struct i2c_client *client = priv->client;
	struct touchdata_v3 *td = priv->data_buf;
	int np = 0;
	int error;
	int i;

	error = ili210x_read_reg(client, REG_TOUCHDATA, td, priv->data_size);
	if (error)
		return error;

	if (!td->status)
		return 0;

	for (i = 0; i < priv->max_points; i++) {
		struct point_v3 *p = &td->points[i];
		unsigned int x, y;

		x = be16_to_cpu(p->x);
		y = be16_to_cpu(p->y);

		if ((x & 0xc000) != 0x8000)
			continue;

		touchscreen_set_mt_pos(&priv->pos[np++], &priv->prop,
				       x & 0x3fff, y);
	}

	return np;
}

static void ili210x_work(struct work_struct *work)
{
	struct ili210x *priv = container_of(work, struct ili210x,
					    dwork.work);
	int np;
	int i;

	np = priv->proto->read_state(priv);
	if (np < 0) {
		dev_err(&priv->client->dev, "Error reading touch data: %d\n",
			np);
		return;
	}

	input_mt_assign_slots(priv->input, priv->slots, priv->pos, np, 0);

	for (i = 0; i < np; i++) {
		input_mt_slot(priv->input, priv->slots[i]);
		input_mt_report_slot_state(priv->input, MT_TOOL_FINGER, true);
		input_report_abs(priv->input, ABS_MT_POSITION_X,
				 priv->pos[i].x);
		input_report_abs(priv->input, ABS_MT_POSITION_Y,
				 priv->pos[i].y);
	}

	input_mt_sync_frame(priv->input);
	input_sync(priv->input);

	if (np > 0)
		schedule_delayed_work(&priv->dwork,
				      msecs_to_jiffies(POLL_PERIOD));
}

static irqreturn_t ili210x_irq(int irq, void *irq_data)
{
	struct ili210x *priv = irq_data;

	schedule_delayed_work(&priv->dwork, 0);

	return IRQ_HANDLED;
}

static ssize_t ili210x_calibrate(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ili210x *priv = i2c_get_clientdata(client);
	unsigned long calibrate;
	int rc;
	u8 cmd = REG_CALIBRATE;

	if (kstrtoul(buf, 10, &calibrate))
		return -EINVAL;

	if (calibrate > 1)
		return -EINVAL;

	if (calibrate) {
		rc = i2c_master_send(priv->client, &cmd, sizeof(cmd));
		if (rc != sizeof(cmd))
			return -EIO;
	}

	return count;
}
static DEVICE_ATTR(calibrate, S_IWUSR, NULL, ili210x_calibrate);

static struct attribute *ili210x_attributes[] = {
	&dev_attr_calibrate.attr,
	NULL,
};

static const struct attribute_group ili210x_attr_group = {
	.attrs = ili210x_attributes,
};

static const struct protocol_info ili210x_proto_v1 = {
	.version_size	= 3,
	.info_size	= 6,
	.data_size	= sizeof(struct touchdata_v1),
	.point_size	= sizeof(struct point_v1),
	.read_state	= ili210x_read_state_v1,
};

static const struct protocol_info ili210x_proto_v2 = {
	.version_size	= 8,
	.info_size	= 10,
	.data_size	= sizeof(struct touchdata_v2),
	.point_size	= sizeof(struct point_v2),
	.read_state	= ili210x_read_state_v2,
};

static const struct protocol_info ili210x_proto_v3 = {
	.version_size	= 8,
	.info_size	= 10,
	.data_size	= sizeof(struct touchdata_v3),
	.point_size	= sizeof(struct point_v3),
	.read_state	= ili210x_read_state_v3,
};

static int ili210x_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct gpio_desc *reset;
	struct ili210x *priv;
	struct input_dev *input;
	struct protocol_version pver;
	struct panel_info panel = { };
	struct firmware_version firmware= { };
	int xmax, ymax;
	int error;

	dev_dbg(dev, "Probing for ILI210X I2C Touschreen driver");

	if (client->irq <= 0) {
		dev_err(dev, "No IRQ!\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (reset) {
		msleep(10);
		gpiod_set_value(reset, 1);
		msleep(300);
	}

	error = ili210x_read_reg(client, REG_PROTOCOL_VERSION, &pver,
				 sizeof(pver));
	if (error) {
		dev_err(dev, "error reading protocol version: %d\n", error);
		return error;
	}

	dev_info(dev, "protocol version %d.%d.%d\n",
		 pver.major, pver.minor, pver.release);

	switch (pver.major) {
	case 0:
	case 1:
		priv->proto = &ili210x_proto_v1;
		priv->max_points = 2;
		break;

	case 2:
		priv->proto = &ili210x_proto_v2;
		break;

	case 3:
		priv->proto = &ili210x_proto_v3;
		break;

	default:
		dev_err(dev, "unknown protocol version\n");
		return -EINVAL;
	}

	/* Get firmware version */
	error = ili210x_read_reg(client, REG_FIRMWARE_VERSION,
				 &firmware, priv->proto->version_size);
	if (error) {
		dev_err(dev, "Failed to get firmware version, err: %d\n",
			error);
		return error;
	}

	if (pver.major < 2) {
		dev_info(dev, "firmware version %d.%d.%d\n",
			 firmware.fver[0], firmware.fver[1], firmware.fver[2]);
	} else {
		dev_info(dev, "firmware version %d.%d.%d.%d\n",
			 firmware.fver[0], firmware.fver[1],
			 firmware.fver[2], firmware.fver[3]);
		dev_info(dev, "customer version %d.%d.%d.%d\n",
			 firmware.cver[0], firmware.cver[1],
			 firmware.cver[2], firmware.cver[3]);
	}

	/* get panel info */
	error = ili210x_read_reg(client, REG_PANEL_INFO, &panel,
				 priv->proto->info_size);
	if (error) {
		dev_err(dev, "Failed to get panel information, err: %d\n",
			error);
		return error;
	}

	xmax = le16_to_cpu(panel.xmax);
	ymax = le16_to_cpu(panel.ymax);

	if (priv->proto->info_size > 5)
		priv->max_points = panel.max_points;

	if (!priv->max_points) {
		dev_err(dev, "max points is 0\n");
		return -EINVAL;
	}

	if (priv->max_points > MAX_POINTS) {
		dev_warn(dev, "max points %d, limiting to %d\n",
			 priv->max_points, MAX_POINTS);
		priv->max_points = MAX_POINTS;
	} else {
		dev_info(dev, "max points %d\n", priv->max_points);
	}

	priv->data_size = priv->proto->data_size +
		priv->max_points * priv->proto->point_size;
	priv->data_buf = devm_kzalloc(dev, priv->data_size, GFP_KERNEL);
	if (!priv->data_buf)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	priv->client = client;
	priv->input = input;
	INIT_DELAYED_WORK(&priv->dwork, ili210x_work);

	/* Setup input device */
	input->name = "ILI210x Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	/* Single touch */
	input_set_abs_params(input, ABS_X, 0, xmax, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, ymax, 0, 0);

	/* Multi touch */
	input_mt_init_slots(input, priv->max_points,
		INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED | INPUT_MT_TRACK);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, xmax, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ymax, 0, 0);

	touchscreen_parse_properties(input, true, &priv->prop);

	i2c_set_clientdata(client, priv);

	error = devm_request_irq(dev, client->irq, ili210x_irq, 0,
				 client->name, priv);
	if (error) {
		dev_err(dev, "Unable to request touchscreen IRQ, err: %d\n",
			error);
		return error;
	}

	error = sysfs_create_group(&dev->kobj, &ili210x_attr_group);
	if (error) {
		dev_err(dev, "Unable to create sysfs attributes, err: %d\n",
			error);
		return error;
	}

	error = input_register_device(priv->input);
	if (error) {
		dev_err(dev, "Cannot register input device, err: %d\n", error);
		goto err_remove_sysfs;
	}

	device_init_wakeup(dev, 1);

	return 0;

err_remove_sysfs:
	sysfs_remove_group(&dev->kobj, &ili210x_attr_group);
	return error;
}

static int ili210x_i2c_remove(struct i2c_client *client)
{
	struct ili210x *priv = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &ili210x_attr_group);
	devm_free_irq(&client->dev, client->irq, priv);
	cancel_delayed_work_sync(&priv->dwork);

	return 0;
}

static int __maybe_unused ili210x_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused ili210x_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ili210x_i2c_pm,
			 ili210x_i2c_suspend, ili210x_i2c_resume);

static const struct i2c_device_id ili210x_i2c_id[] = {
	{ "ili210x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ili210x_i2c_id);

static struct i2c_driver ili210x_ts_driver = {
	.driver = {
		.name = "ili210x_i2c",
		.pm = &ili210x_i2c_pm,
	},
	.id_table = ili210x_i2c_id,
	.probe = ili210x_i2c_probe,
	.remove = ili210x_i2c_remove,
};

module_i2c_driver(ili210x_ts_driver);

MODULE_AUTHOR("Olivier Sobrie <olivier@sobrie.be>");
MODULE_DESCRIPTION("ILI210X I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
