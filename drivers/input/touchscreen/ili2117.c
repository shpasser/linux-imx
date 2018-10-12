#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

#define MAX_TOUCHES			10
#define DEFAULT_POLL_PERIOD	20
#define X_MAX 2047
#define Y_MAX 2047


struct finger {
    u8 y_hi : 4;
    u8 x_hi : 4;
    u8 x_lo;
    u8 y_lo;
    u8 c_sum;
} __packed;

struct touchdata {
    u8 packet_id;
    struct finger finger[MAX_TOUCHES];
    u8 key : 4;
    u8 p_sensor : 4;
    u8 checksum;
} __packed;

struct ili2117 {
	struct i2c_client *client;
	struct input_dev *input;
	unsigned int poll_period;
	struct delayed_work dwork;
};

static int ili2117_read(struct i2c_client *client, void *buf, size_t len)
{
	struct i2c_msg msg[1] = {
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	if (i2c_transfer(client->adapter, msg, 1) != 1) {
		dev_err(&client->dev, "i2c transfer failed\n");
		return -EIO;
	}

	return 0;
}

static void ili2117_report_events(struct input_dev *input,
				  const struct touchdata *touchdata)
{
	int i;
	bool touch;
	unsigned int x, y;
	const struct finger *finger;

	for (i = 0; i < MAX_TOUCHES; i++) {
		input_mt_slot(input, i);

		finger = &touchdata->finger[i];

		touch = (touchdata->packet_id == 0x5a) && 
				(touchdata->checksum != 0xff) && 
				(finger->c_sum != 0xff);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, touch);
		if (touch) {
			x = finger->x_lo | (finger->x_hi << 8);
			y = finger->y_lo | (finger->y_hi << 8);

			input_report_abs(input, ABS_MT_POSITION_X, x);
			input_report_abs(input, ABS_MT_POSITION_Y, y);
		}
	}

	input_mt_report_pointer_emulation(input, false);
	input_sync(input);
}

static void ili2117_work(struct work_struct *work)
{
	struct ili2117 *priv = container_of(work, struct ili2117, dwork.work);
	struct i2c_client *client = priv->client;
	struct touchdata touchdata;
	int error;

	error = ili2117_read(client, &touchdata, sizeof(touchdata)); 
	if (error) {
		dev_err(&client->dev, "Unable to get touchdata, err = %d\n", error);
		return;
	}

	ili2117_report_events(priv->input, &touchdata);

	if (touchdata.packet_id == 0x5a) {
		schedule_delayed_work(&priv->dwork, msecs_to_jiffies(priv->poll_period));
	}
}

static irqreturn_t ili2117_irq(int irq, void *irq_data)
{
	struct ili2117 *priv = irq_data;

	schedule_delayed_work(&priv->dwork, 0);

	return IRQ_HANDLED;
}

static int ili2117_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ili2117 *priv;
	struct input_dev *input;
	int error;

	dev_dbg(dev, "Probing for ILI2117 I2C Touschreen driver");

	if (client->irq <= 0) {
		dev_err(dev, "No IRQ!\n");
		return -EINVAL;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	input = input_allocate_device();
	if (!priv || !input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	priv->client = client;
	priv->input = input;
	priv->poll_period = DEFAULT_POLL_PERIOD;
	INIT_DELAYED_WORK(&priv->dwork, ili2117_work);

	/* Setup input device */
	input->name = "ILI2117 Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;

	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);

	/* Single touch */
	input_set_abs_params(input, ABS_X, 0, X_MAX, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, Y_MAX, 0, 0);

	/* Multi touch */
	input_mt_init_slots(input, MAX_TOUCHES, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, Y_MAX, 0, 0);

	input_set_drvdata(input, priv);
	i2c_set_clientdata(client, priv);

	error = request_irq(client->irq, ili2117_irq, 0, client->name, priv);
	if (error) {
		dev_err(dev, "Unable to request touchscreen IRQ, err: %d\n",
			error);
		goto err_free_mem;
	}

	error = input_register_device(priv->input);
	if (error) {
		dev_err(dev, "Cannot register input device, err: %d\n", error);
		goto err_free_irq;
	}

	device_init_wakeup(&client->dev, 1);

	dev_dbg(dev, "ILI2117 initialized (IRQ: %d)", client->irq);

	return 0;

err_free_irq:
	free_irq(client->irq, priv);
err_free_mem:
	input_free_device(input);
	kfree(priv);
	return error;
}

static int ili2117_i2c_remove(struct i2c_client *client)
{
	struct ili2117 *priv = i2c_get_clientdata(client);

	free_irq(priv->client->irq, priv);
	cancel_delayed_work_sync(&priv->dwork);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

static int __maybe_unused ili2117_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int __maybe_unused ili2117_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ili2117_i2c_pm,
			 ili2117_i2c_suspend, ili2117_i2c_resume);

static const struct i2c_device_id ili2117_i2c_id[] = {
	{ "ili2117", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ili2117_i2c_id);

static struct i2c_driver ili2117_ts_driver = {
	.driver = {
		.name = "ili2117_i2c",
		.pm = &ili2117_i2c_pm,
	},
	.id_table = ili2117_i2c_id,
	.probe = ili2117_i2c_probe,
	.remove = ili2117_i2c_remove,
};

module_i2c_driver(ili2117_ts_driver);

MODULE_AUTHOR("Ron Shpasser <shpasser@gmail.com>");
MODULE_DESCRIPTION("ILI2117 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
