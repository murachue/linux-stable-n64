/*
 * Driver for the EverDrive-64 i2c adapter
 *
 * Copyright (c) 2018 Murachue <murachue+github@gmail.com>
 *
 * Derived from:
 *  i2c-dln2.c
 *  Copyright (c) 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/mfd/n64pi.h>

#define ED64_I2C_MAX_XFER_SIZE		256

struct ed64_i2c {
	struct device *dev;
	struct n64pi *pi;
	struct i2c_adapter adapter;
};

static void ed64_i2c_write(struct n64pi *pi, uint32_t clk_dat)
{
	n64pi_ed64_regwrite(pi, clk_dat, 0x30);
}

static unsigned int ed64_i2c_read(struct n64pi *pi)
{
	return n64pi_ed64_regread(pi, 0x30);
}

static void i2c_start(struct n64pi *pi)
{
	ed64_i2c_write(pi, 5); /* clk|dat */
	ed64_i2c_write(pi, 4); /* clk|~dat (start condition: dat changes while clk is raised) */
	ed64_i2c_write(pi, 0); /* ~clk(|~dat) */
}

static int i2c_writebyte(struct n64pi *pi, uint32_t byte)
{
	int i, r;
	for (i = 0; i < 8; i++) {
		uint32_t v = (byte >> (7 - i)) & 1;
		ed64_i2c_write(pi, v); /* prepare dat while ~clk */
		ed64_i2c_write(pi, 4 | v); /* raise clk with dat: acq it, device! */
	}

	ed64_i2c_write(pi, 1); /* ~clk|dat */
	ed64_i2c_write(pi, 5); /* clk|dat: raise clk: urge device sends ack (dat=1 required: pull-up open-drain) */
	r = ed64_i2c_read(pi) & 1; /* sense dat */
	ed64_i2c_write(pi, 1); /* ~clk|dat */

	return r;
}

static uint32_t i2c_readbyte(struct n64pi *pi, int nack)
{
	int i;
	uint32_t r = 0;
	for (i = 0; i < 8; i++) {
		ed64_i2c_write(pi, 1); /* lower clk: urge device changes dat */
		ed64_i2c_write(pi, 5); /* raise clk: dat is stable... hopefully. */
		r = (r << 1) | (ed64_i2c_read(pi) & 1);
	}

	nack = !!nack; /* 0 or 1 */
	ed64_i2c_write(pi, nack); /* ~clk|[nack] */
	ed64_i2c_write(pi, 4 | nack); /* clk|<nack>: raise clk: send [n]ack */
	ed64_i2c_write(pi, nack); /* ~clk|[nack] */

	return r;
}

static void i2c_stop(struct n64pi *pi)
{
	ed64_i2c_write(pi, 4); /* clk|~dat */
	ed64_i2c_write(pi, 5); /* clk|dat (stop condition: dat changes while clk is raised) */
}

static int ed64_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct ed64_i2c *ed64 = i2c_get_adapdata(adapter);
	struct n64pi * const pi = ed64->pi;
	struct i2c_msg *pmsg;
	int i, ret = num;

	n64pi_begin(pi);

	n64pi_ed64_enable(pi);

	/* raise the magic flag (or i2c read 0xFF only); must be close before return (or cart-read freezes) */
	n64pi_ed64_regwrite(pi, n64pi_ed64_regread(pi, 0x00) | 0x0060, 0x00);

	for (i = 0; i < num; i++) {
		int j;

		pmsg = &msgs[i];

		i2c_start(pi);

		/* write address and i/o(isread) */
		if (i2c_writebyte(pi, (pmsg->addr << 1) | !!(pmsg->flags & I2C_M_RD)) != 0) {
			/* no device ack? maybe no device at that address. */
			ret = -ENODEV;
			break;
		}

		/* i/o data */
		if (pmsg->flags & I2C_M_RD) {
			for (j = 0; j < pmsg->len; j++) {
				pmsg->buf[j] = i2c_readbyte(pi, j == (pmsg->len - 1));
			}
			/* pmsg->len = ret; */
		} else {
			for (j = 0; j < pmsg->len; j++) {
				if(i2c_writebyte(pi, pmsg->buf[j]) != 0) {
					/* got nack from device */
					ret = -EPROTO;
					break;
				}
			}
		}

		i2c_stop(pi);

#ifdef DEBUG /* defined by CONFIG_I2C_DEBUG_BUS */
		{
			char buf[256];
			int k;
			for (k = 0; k < pmsg->len; k++) {
				sprintf(buf + (k * 2), "%02X", pmsg->buf[k]);
			}
			pr_debug("ed64i2c: %02X %s %s-> %d/%d\n", pmsg->addr, (pmsg->flags & I2C_M_RD) ? "read" : "write", buf, j, ret);
		}
#endif

		if (ret < 0) {
			break;
		}
	}

	/* lower the magic flag */
	n64pi_ed64_regwrite(pi, n64pi_ed64_regread(pi, 0x00) & ~0x0060, 0x00);

	n64pi_ed64_disable(ed64->pi);

	n64pi_end(ed64->pi);

	return ret;
}

static u32 ed64_i2c_func(struct i2c_adapter *a)
{
	/* note: rtc-ds1307 requires I2C_FUNC_SMBUS_BYTE_DATA and I2C_FUNC_SMBUS_I2C_BLOCK... uh what? */
	return I2C_FUNC_I2C |
		I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_BLOCK_PROC_CALL |
		I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm ed64_i2c_algorithm = {
	.master_xfer = ed64_i2c_xfer,
	.functionality = ed64_i2c_func,
};

static struct i2c_adapter_quirks ed64_i2c_quirks = {
	.max_read_len = ED64_I2C_MAX_XFER_SIZE,
	.max_write_len = ED64_I2C_MAX_XFER_SIZE,
};

static int ed64_i2c_probe(struct platform_device *pdev)
{
	int ret;
	struct ed64_i2c *ed64;
	struct device *dev = &pdev->dev;

	ed64 = devm_kzalloc(dev, sizeof(*ed64), GFP_KERNEL);
	if (!ed64)
		return -ENOMEM;

	ed64->dev = dev;
	ed64->pi = dev_get_drvdata(pdev->dev.parent); /* TODO dev.parent->parent is n64pi?? ipaq-micro-leds */

	/* setup i2c adapter description */
	ed64->adapter.owner = THIS_MODULE;
	ed64->adapter.class = I2C_CLASS_HWMON;
	ed64->adapter.algo = &ed64_i2c_algorithm;
	ed64->adapter.quirks = &ed64_i2c_quirks;
	ed64->adapter.dev.parent = dev;
	ed64->adapter.dev.of_node = dev->of_node;
	i2c_set_adapdata(&ed64->adapter, ed64); /* for i2c ops */
	strlcpy(ed64->adapter.name, "ed64-i2c", sizeof(ed64->adapter.name));

	platform_set_drvdata(pdev, ed64); /* for remove */

	/* attach to i2c layer */
	ret = i2c_add_adapter(&ed64->adapter);
	if (ret < 0) {
		dev_err(dev, "failed to add I2C adapter: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ed64_i2c_remove(struct platform_device *pdev)
{
	struct ed64_i2c *ed64 = platform_get_drvdata(pdev);

	i2c_del_adapter(&ed64->adapter);

	return 0;
}

static struct platform_driver ed64_i2c_driver = {
	.probe		= ed64_i2c_probe,
	.remove		= ed64_i2c_remove,
	.driver = {
		.name	= "n64pi-ed64i2c",
	},
};

module_platform_driver(ed64_i2c_driver);

MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
MODULE_DESCRIPTION("EverDrive-64 I2C master interface driver");
MODULE_LICENSE("GPL v2");
//MODULE_ALIAS("platform:ed64-i2c");
