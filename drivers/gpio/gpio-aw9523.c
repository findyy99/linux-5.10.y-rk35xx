// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for aw9523 I2C GPIO expanders
 *
 * Copyright (c) 2023 Rockchip Electronics Co. Ltd.
 */
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>  /* For msleep() */

/* AW9523 registers */
#define REG_INPUT_P0        0x00    /* Input port 0 */
#define REG_INPUT_P1        0x01    /* Input port 1 */
#define REG_OUTPUT_P0       0x02    /* Output port 0 */
#define REG_OUTPUT_P1       0x03    /* Output port 1 */
#define REG_CONFIG_P0       0x04    /* Configuration port 0 (0:output, 1:input) */
#define REG_CONFIG_P1       0x05    /* Configuration port 1 (0:output, 1:input) */
#define REG_INT_P0          0x06    /* Interrupt port 0 */
#define REG_INT_P1          0x07    /* Interrupt port 1 */
#define REG_ID              0x10    /* Chip ID register */
#define REG_CTL             0x11    /* Control register */
#define REG_LED_MODE_P0     0x12    /* LED mode for port 0 */
#define REG_LED_MODE_P1     0x13    /* LED mode for port 1 */

/* Control register bits */
#define CTL_SOFT_RESET      0x10    /* Soft reset bit */
#define CTL_GCC_DRV         0x20    /* Global current control */

static const struct i2c_device_id aw9523_id[] = {
	{ "aw9523", 16 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw9523_id);

#ifdef CONFIG_OF
static const struct of_device_id aw9523_of_table[] = {
	{ .compatible = "awinic,aw9523" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw9523_of_table);
#endif

struct aw9523 {
	struct gpio_chip	chip;
	struct irq_chip		irqchip;
	struct i2c_client	*client;
	struct mutex		lock;		/* protect registers access */
	unsigned int		out;		/* software latch */
	unsigned int		direction;	/* gpio direction (1:input, 0:output) */
	unsigned int		status;		/* current status */
	unsigned int		irq_enabled;	/* enabled irqs */
	unsigned int		irq_wake;	/* wake enabled irqs */
	
	struct device		*dev;
	int			shdn_en;	/* shutdown control gpio */
	int			always_on;	/* always on flag */

	int (*write)(struct i2c_client *client, u8 reg, u8 data);
	int (*read)(struct i2c_client *client, u8 reg);
};

static int aw9523_i2c_write_le8(struct i2c_client *client, u8 reg, u8 data)
{
	int ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev, "i2c write failed: %d\n", ret);
	return ret;
}

static int aw9523_i2c_read_le8(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "i2c read failed: %d\n", ret);
	return ret;
}

static int aw9523_get(struct gpio_chip *chip, unsigned int offset)
{
	struct aw9523 *gpio = gpiochip_get_data(chip);
	int value;

	mutex_lock(&gpio->lock);

	if (offset < 8) {
		value = gpio->read(gpio->client, REG_INPUT_P0);
		mutex_unlock(&gpio->lock);
		value = (value < 0) ? value : !!(value & (1 << offset));
	} else {
		value = gpio->read(gpio->client, REG_INPUT_P1);
		mutex_unlock(&gpio->lock);
		value = (value < 0) ? value : !!(value & (1 << (offset - 8)));
	}

	return value;
}

static int aw9523_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct aw9523 *gpio = gpiochip_get_data(chip);

	return (gpio->direction & (1 << offset)) ? 
		GPIO_LINE_DIRECTION_IN : GPIO_LINE_DIRECTION_OUT;
}

static int aw9523_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct aw9523 *gpio = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&gpio->lock);

	/* Set direction bit (1 = input) */
	gpio->direction |= (1 << offset);

	if (offset < 8)
		ret = gpio->write(gpio->client, REG_CONFIG_P0, 
				 (gpio->direction & 0xFF));
	else
		ret = gpio->write(gpio->client, REG_CONFIG_P1, 
				 ((gpio->direction >> 8) & 0xFF));

	mutex_unlock(&gpio->lock);

	dev_dbg(gpio->dev, "direction input: pin %d, direction 0x%04x\n", 
		offset, gpio->direction);

	return ret < 0 ? ret : 0;
}

static int aw9523_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct aw9523 *gpio = gpiochip_get_data(chip);
	int ret;

	/* Set value first */
	chip->set(chip, offset, value);

	mutex_lock(&gpio->lock);

	/* Clear direction bit (0 = output) */
	gpio->direction &= ~(1 << offset);

	if (offset < 8)
		ret = gpio->write(gpio->client, REG_CONFIG_P0, 
				 (gpio->direction & 0xFF));
	else
		ret = gpio->write(gpio->client, REG_CONFIG_P1, 
				 ((gpio->direction >> 8) & 0xFF));

	mutex_unlock(&gpio->lock);

	dev_dbg(gpio->dev, "direction output: pin %d, value %d, direction 0x%04x\n", 
		offset, value, gpio->direction);

	return ret < 0 ? ret : 0;
}

static void aw9523_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct aw9523 *gpio = gpiochip_get_data(chip);
	unsigned int bit = 1 << (offset & 0x7);
	u8 reg_val;

	mutex_lock(&gpio->lock);

	if (offset < 8) {
		reg_val = gpio->out & 0xFF;
		if (value)
			reg_val |= bit;
		else
			reg_val &= ~bit;
		gpio->write(gpio->client, REG_OUTPUT_P0, reg_val);
		gpio->out = (gpio->out & 0xFF00) | reg_val;
	} else {
		reg_val = (gpio->out >> 8) & 0xFF;
		if (value)
			reg_val |= bit;
		else
			reg_val &= ~bit;
		gpio->write(gpio->client, REG_OUTPUT_P1, reg_val);
		gpio->out = (gpio->out & 0x00FF) | (reg_val << 8);
	}

	mutex_unlock(&gpio->lock);

	dev_dbg(gpio->dev, "set value: pin %d, value %d, out 0x%04x\n", 
		offset, value, gpio->out);
}

/*-------------------------------------------------------------------------*/

static irqreturn_t aw9523_irq(int irq, void *data)
{
	struct aw9523 *gpio = data;
	unsigned long change, i;
	unsigned int status = 0;
	int value, nirq;

	/* Read input status */
	value = gpio->read(gpio->client, REG_INPUT_P0);
	if (value >= 0)
		status |= value;

	value = gpio->read(gpio->client, REG_INPUT_P1);
	if (value >= 0)
		status |= (value << 8);

	mutex_lock(&gpio->lock);
	change = (gpio->status ^ status) & gpio->irq_enabled;
	gpio->status = status;
	mutex_unlock(&gpio->lock);

	dev_dbg(gpio->dev, "status:%04lx change:%04lx\n", (unsigned long)status, change);

	for_each_set_bit(i, &change, gpio->chip.ngpio) {
		nirq = irq_find_mapping(gpio->chip.irq.domain, i);
		if (nirq) {
			handle_nested_irq(nirq);
		}
	}

	return IRQ_HANDLED;
}

/* IRQ chip operations */
static void aw9523_noop(struct irq_data *data) { }

static int aw9523_irq_set_wake(struct irq_data *data, unsigned int on)
{
	struct aw9523 *gpio = irq_data_get_irq_chip_data(data);
	unsigned int bit = 1 << data->hwirq;

	if (on)
		gpio->irq_wake |= bit;
	else
		gpio->irq_wake &= ~bit;

	return 0;
}

static void aw9523_irq_enable(struct irq_data *data)
{
	struct aw9523 *gpio = irq_data_get_irq_chip_data(data);

	gpio->irq_enabled |= (1 << data->hwirq);
	dev_dbg(gpio->dev, "irq:%d %d enable\n", data->irq, (int)data->hwirq);
}

static void aw9523_irq_disable(struct irq_data *data)
{
	struct aw9523 *gpio = irq_data_get_irq_chip_data(data);

	gpio->irq_enabled &= ~(1 << data->hwirq);
	dev_dbg(gpio->dev, "irq:%d %d disable\n", data->irq, (int)data->hwirq);
}

static void aw9523_irq_bus_lock(struct irq_data *data)
{
	struct aw9523 *gpio = irq_data_get_irq_chip_data(data);

	mutex_lock(&gpio->lock);
}

static void aw9523_irq_bus_sync_unlock(struct irq_data *data)
{
	struct aw9523 *gpio = irq_data_get_irq_chip_data(data);
	u8 int_p0, int_p1;

	/* Update interrupt mask registers */
	int_p0 = ~(gpio->irq_enabled & 0xFF);
	int_p1 = ~((gpio->irq_enabled >> 8) & 0xFF);

	gpio->write(gpio->client, REG_INT_P0, int_p0);
	gpio->write(gpio->client, REG_INT_P1, int_p1);

	mutex_unlock(&gpio->lock);
}

static void aw9523_init_chip(struct aw9523 *gpio)
{
	/* Reset the chip */
	gpio->write(gpio->client, REG_CTL, CTL_SOFT_RESET);
	msleep(10);

	/* Set all pins to GPIO mode */
	gpio->write(gpio->client, REG_LED_MODE_P0, 0xFF);
	gpio->write(gpio->client, REG_LED_MODE_P1, 0xFF);

	/* Default all pins to input */
	gpio->direction = 0xFFFF;
	gpio->write(gpio->client, REG_CONFIG_P0, 0xFF);
	gpio->write(gpio->client, REG_CONFIG_P1, 0xFF);

	/* Clear interrupt mask (enable all interrupts) */
	gpio->irq_enabled = 0xFFFF;
	gpio->write(gpio->client, REG_INT_P0, 0x00);
	gpio->write(gpio->client, REG_INT_P1, 0x00);

	/* Read initial input state */
	gpio->status = 0;
	gpio->status |= gpio->read(gpio->client, REG_INPUT_P0);
	gpio->status |= (gpio->read(gpio->client, REG_INPUT_P1) << 8);
}

static int aw9523_parse_dt(struct aw9523 *gpio, struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	int ret;
	u32 gpio_base = 0;
	u32 gpio_nums = 0;

	/* Get GPIO base number */
	ret = of_property_read_u32(np, "gpio_base", &gpio_base);
	if (ret < 0 || gpio_base == 0) {
		dev_info(gpio->dev, "gpio_base is not set, using default\n");
	} else {
		gpio->chip.base = gpio_base;
	}

	/* Get GPIO count */
	ret = of_property_read_u32(np, "gpio_nums", &gpio_nums);
	if (ret < 0 || gpio_nums == 0) {
		dev_info(gpio->dev, "gpio_nums is not set, using default 16\n");
	} else {
		gpio->chip.ngpio = gpio_nums;
	}

	/* Check for shutdown enable GPIO */
	ret = of_get_named_gpio(np, "shdn_en", 0);
	if (ret < 0) {
		dev_info(gpio->dev, "of get shdn_en failed\n");
		gpio->shdn_en = -1;
	} else {
		gpio->shdn_en = ret;
		ret = devm_gpio_request_one(gpio->dev, gpio->shdn_en,
					    GPIOF_OUT_INIT_HIGH, "aw9523_SHDN_EN");
		if (ret) {
			dev_err(gpio->dev, "devm_gpio_request_one shdn_en failed\n");
			return ret;
		}
	}

	/* Check for always_on property */
	if (of_find_property(np, "always_on", NULL)) {
		dev_info(gpio->dev, "of get always_on success\n");
		gpio->always_on = 1;
	} else {
		dev_info(gpio->dev, "of get always_on failed\n");
		gpio->always_on = 0;
	}

	return 0;
}

static int aw9523_check_dev_id(struct i2c_client *client)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, REG_ID);
	if (ret < 0) {
		dev_err(&client->dev, "fail to read dev id(%d)\n", ret);
		return ret;
	}

	dev_info(&client->dev, "dev id : 0x%02x\n", ret);

	/* AW9523 should return ID 0x23 */
	if (ret != 0x23) {
		dev_err(&client->dev, "check device id fail, expected 0x23, got 0x%02x\n", ret);
		return -ENODEV;
	}

	return 0;
}

/*-------------------------------------------------------------------------*/

static int aw9523_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct aw9523 *gpio;
	int status;

	dev_info(&client->dev, "===aw9523 probe===\n");

	/* Allocate, initialize, and register this gpio_chip. */
	gpio = devm_kzalloc(&client->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->dev = &client->dev;
	gpio->client = client;

	status = aw9523_parse_dt(gpio, client);
	if (status < 0)
		return status;

	mutex_init(&gpio->lock);

	gpio->chip.base = -1; /* Dynamic allocation */
	gpio->chip.can_sleep = true;
	gpio->chip.parent = &client->dev;
	gpio->chip.owner = THIS_MODULE;
	gpio->chip.get = aw9523_get;
	gpio->chip.set = aw9523_set;
	gpio->chip.get_direction = aw9523_get_direction;
	gpio->chip.direction_input = aw9523_direction_input;
	gpio->chip.direction_output = aw9523_direction_output;
	gpio->chip.ngpio = id->driver_data;
	gpio->chip.label = client->name;

	gpio->write = aw9523_i2c_write_le8;
	gpio->read = aw9523_i2c_read_le8;

	i2c_set_clientdata(client, gpio);

	status = aw9523_check_dev_id(client);
	if (status < 0) {
		dev_err(&client->dev, "check device id fail(%d)\n", status);
		goto fail;
	}

	aw9523_init_chip(gpio);

	/* Enable irqchip if we have an interrupt */
	if (client->irq) {
		struct gpio_irq_chip *girq;

		gpio->irqchip.name = "aw9523";
		gpio->irqchip.irq_enable = aw9523_irq_enable;
		gpio->irqchip.irq_disable = aw9523_irq_disable;
		gpio->irqchip.irq_ack = aw9523_noop;
		gpio->irqchip.irq_mask = aw9523_noop;
		gpio->irqchip.irq_unmask = aw9523_noop;
		gpio->irqchip.irq_set_wake = aw9523_irq_set_wake;
		gpio->irqchip.irq_bus_lock = aw9523_irq_bus_lock;
		gpio->irqchip.irq_bus_sync_unlock = aw9523_irq_bus_sync_unlock;

		dev_info(&client->dev, "irq: %d\n", client->irq);

		status = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, aw9523_irq, IRQF_ONESHOT |
					IRQF_TRIGGER_FALLING | IRQF_SHARED,
					dev_name(&client->dev), gpio);
		if (status) {
			dev_err(&client->dev, "failed to request irq %d\n", client->irq);
			goto fail;
		}

		girq = &gpio->chip.irq;
		girq->chip = &gpio->irqchip;
		/* This will let us handle the parent IRQ in the driver */
		girq->parent_handler = NULL;
		girq->num_parents = 0;
		girq->parents = NULL;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_level_irq;
		girq->threaded = true;
	}

	status = devm_gpiochip_add_data(&client->dev, &gpio->chip, gpio);
	if (status < 0)
		goto fail;

	dev_info(&client->dev, "probed\n");

	return 0;

fail:
	dev_err(&client->dev, "probe error %d for '%s'\n", status,
		client->name);

	return status;
}

static int aw9523_pm_suspend(struct device *dev)
{
	struct aw9523 *gpio = dev_get_drvdata(dev);

	dev_info(gpio->dev, "===aw9523 pm suspend===begin===\n");

	if (gpio->always_on) {
		/* Configure interrupt for wake */
		if (gpio->irq_wake) {
			gpio->write(gpio->client, REG_INT_P0, ~(gpio->irq_wake & 0xFF));
			gpio->write(gpio->client, REG_INT_P1, ~((gpio->irq_wake >> 8) & 0xFF));
			
			dev_info(gpio->dev, "===aw9523 irq_enabled_wake: 0x%x===\n", gpio->irq_wake);
			
			/* Read status to clear any pending interrupts */
			gpio->read(gpio->client, REG_INPUT_P0);
			gpio->read(gpio->client, REG_INPUT_P1);
			
			if (gpio->irq_wake && gpio->client->irq) {
				irq_set_irq_wake(gpio->client->irq, 1);
			}
		}
	}

	dev_info(gpio->dev, "===aw9523 pm suspend===end===\n");
	return 0;
}

static int aw9523_pm_resume(struct device *dev)
{
	struct aw9523 *gpio = dev_get_drvdata(dev);

	dev_info(gpio->dev, "===aw9523 pm resume===begin===\n");

	if (gpio->irq_wake && gpio->client->irq) {
		irq_set_irq_wake(gpio->client->irq, 0);
	}

	/* Restore interrupt mask */
	gpio->write(gpio->client, REG_INT_P0, ~(gpio->irq_enabled & 0xFF));
	gpio->write(gpio->client, REG_INT_P1, ~((gpio->irq_enabled >> 8) & 0xFF));

	/* Read status to clear any pending interrupts */
	gpio->read(gpio->client, REG_INPUT_P0);
	gpio->read(gpio->client, REG_INPUT_P1);

	dev_info(gpio->dev, "===aw9523 pm resume interrupt: 0x%x===\n", gpio->irq_enabled);

	if (!gpio->always_on) {
		/* Restore chip configuration */
		gpio->write(gpio->client, REG_CONFIG_P0, gpio->direction & 0xFF);
		gpio->write(gpio->client, REG_CONFIG_P1, (gpio->direction >> 8) & 0xFF);
		
		gpio->write(gpio->client, REG_OUTPUT_P0, gpio->out & 0xFF);
		gpio->write(gpio->client, REG_OUTPUT_P1, (gpio->out >> 8) & 0xFF);
		
		gpio->write(gpio->client, REG_CTL, 0x10);
		gpio->write(gpio->client, REG_LED_MODE_P0, 0xFF);
		gpio->write(gpio->client, REG_LED_MODE_P1, 0xFF);
	}

	dev_info(gpio->dev, "===aw9523 pm resume===end===\n");
	return 0;
}

static const struct dev_pm_ops aw9523_pm_ops = {
	.suspend = aw9523_pm_suspend,
	.resume = aw9523_pm_resume,
};

static struct i2c_driver aw9523_driver = {
	.driver = {
		.name	= "aw9523",
		.pm = &aw9523_pm_ops,
		.of_match_table = of_match_ptr(aw9523_of_table),
	},
	.probe	= aw9523_probe,
	.id_table = aw9523_id,
};

static int __init aw9523_init(void)
{
	return i2c_add_driver(&aw9523_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(aw9523_init);

static void __exit aw9523_exit(void)
{
	i2c_del_driver(&aw9523_driver);
}
module_exit(aw9523_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rockchip");
MODULE_DESCRIPTION("AW9523 I2C GPIO expander driver"); 
