/*
 * Driver for Nintendo 64 Controller Bros. joysticks for Linux/n64
 * with supporting SI(Serial Interface).
 * Copyright (c) 2018 Murachue <murachue+github@gmail.com>
 *
 * TODO: consider non-ControllerBros. connection (Voice Recognizer (Pikachu), Randnet Keyboard, ...)
 * TODO: use POLLDEV or not...?
 * TODO: support Rumble Pak (force feedback)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
/*
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
*/
#include <linux/init.h> /* __init __exit */
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/dma-mapping.h>

#define N64SI_POLL_DELAY (HZ * 20 / 1000) // 20ms

#define REG_DRAMADDR 0x00
#define REG_SI2RAM 0x04
#define REG_RAM2SI 0x10
#define REG_STATUS 0x18

struct n64si { /* represents the driver status of the SI device */
	struct device *dev; /* is a corresponding platform device */
	void __iomem *regbase; /* is a base virtual address of SI registers (ie. 0xA4800000) */
	struct mutex mutex; /* holds mutex for this state container */
	int use_count; /* holds how many controllers are opened (0 to 4, for start/stop polling) */
	int ongoing; /* holds boolean means n64si is DMAing or not */
	struct input_dev *inputdev[4]; /* literally */
	struct timer_list polltimer; /* literally too. */
};

static const char *n64si_phys[4] = { "n64si/input0", "n64si/input1", "n64si/input2", "n64si/input3" };

static const int buttons_from_msb[16] = {
	BTN_A, BTN_B, BTN_Z, BTN_START,
	BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT, /* DPads */
	-1, -1, BTN_TL, BTN_TR,
	BTN_FORWARD, BTN_BACK, BTN_LEFT, BTN_RIGHT, /* Cs: up, down, left, right. */
};

static void n64si_poll(unsigned long private)
{
	struct n64si *si = (struct n64si *)private;
	uint32_t buf[64/4];
	dma_addr_t busaddr;

	/* prepare the SI commands */
	buf[0]  = 0xff010401; /* pad, cmd01:rx04: read buttons */
	buf[2]  = 0xff010401;
	buf[4]  = 0xff010401;
	buf[6]  = 0xff010401;
	buf[8]  = 0xfe000000; /* lenfe: end of command */
	buf[15] = 0x00000001; /* last 01: send commands to controllers! */

	/* send the commands */
	busaddr = dma_map_single(si->dev, &buf, 64, DMA_TO_DEVICE);
	if(dma_mapping_error(si->dev, busaddr)) {
		dev_err(si->dev, "%s: Can't map DMA buffer: %p+40; skipping.\n", "send", &buf);
		goto out;
	}
	__raw_writel(busaddr, si->regbase + REG_DRAMADDR);
	__raw_writel(0x1FC007C0, si->regbase + REG_RAM2SI); /* the value have mean only on low 12bits(0x7C0), but some emulator such as Project64 crashes on other value. */
	while(__raw_readl(si->regbase + REG_STATUS) & 3) /*nothing*/ ;
	dma_unmap_single(si->dev, busaddr, 64, DMA_TO_DEVICE);

	/* receive the results */
	busaddr = dma_map_single(si->dev, &buf, 64, DMA_FROM_DEVICE);
	if(dma_mapping_error(si->dev, busaddr)) {
		dev_err(si->dev, "%s: Can't map DMA buffer: %p+40; skipping.\n", "recv", &buf);
		goto out;
	}
	__raw_writel(busaddr, si->regbase + REG_DRAMADDR);
	__raw_writel(0x1FC007C0, si->regbase + REG_SI2RAM); /* the value have mean only on low 12bits(0x7C0), but some emulator such as Project64 crashes on other value. */
	while(__raw_readl(si->regbase + REG_STATUS) & 3) /*nothing*/ ;
	dma_unmap_single(si->dev, busaddr, 64, DMA_FROM_DEVICE);

	/* finally clears interrupt flag to avoid spurious interrupt. (this is blocking driver... sadly.) */
	__raw_writel(0, si->regbase + REG_STATUS);

	/* inspect and report! */
	{
		int i, j;

		for (i = 0; i < 4; i++) {
			if (buf[i * 2] & 0x0000C000) { /* [15]not_present [14]bad_command */
				continue;
			}

			{
				uint32_t stat = buf[i * 2 + 1];
				struct input_dev *idev = si->inputdev[i];
				uint32_t mask = 0x80000000;

				for(j = 0; j < 16; j++) {
					input_report_key(idev, buttons_from_msb[j], !!(stat & mask));
					mask >>= 1;
				}

				input_report_abs(idev, ABS_X, (int8_t)((stat >> 8) & 0xFF));
				input_report_abs(idev, ABS_Y, (int8_t)((stat >> 0) & 0xFF));

				input_sync(idev);
			}
		}
	}

	/* reschedule */
	mod_timer(&si->polltimer, jiffies + N64SI_POLL_DELAY);
out: /* moving out label to after mod_timer... avoid continuous error. TODO move above? */
	;
}

static irqreturn_t n64si_interrupt(int irq, void *_si)
{
	struct n64si *si = _si;
	/* many messages... acking just after SI DMA is not enough...? don't spit message. */
	/*
	uint32_t status = __raw_readl(si->regbase + REG_STATUS);

	dev_err(si->dev, "spurious interrupt (status=%08X)\n", status);
	*/
	__raw_writel(0, si->regbase + REG_STATUS); /* ack */

	return IRQ_HANDLED;
}

static int n64si_open(struct input_dev *dev)
{
	struct n64si *si = input_get_drvdata(dev);
	int err;

	err = mutex_lock_interruptible(&si->mutex);
	if (err)
		return err;

	if (!si->use_count) {
		mod_timer(&si->polltimer, jiffies + N64SI_POLL_DELAY);
	}

	si->use_count++;

	mutex_unlock(&si->mutex);

	return err;
}

static void n64si_close(struct input_dev *dev)
{
	struct n64si *si = input_get_drvdata(dev);

	mutex_lock(&si->mutex);
	if (!--si->use_count) {
		del_timer_sync(&si->polltimer);
	}
	mutex_unlock(&si->mutex);
}

/* note: probe can be __init because of module_p_d"_probe". */
static int __init n64si_probe(struct platform_device *pdev)
{
	struct n64si *si;
	struct resource *res;
	int irq;
	int ret;

	if(dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(24)) != 0) {
		//dev_err(dev, "No suitable DMA available\n");
		// in MIPS dma-default, dma_map_ops->dma_set_mask does not defined, that cause failure.
		// and generic, dma_map_ops->map_page does not care dev, means mask is ignored.
		// so, ignore this error!!
		dev_warn(&pdev->dev, "No suitable DMA available... don't care this :)\n");
		//return -EINVAL;
	}

	si = devm_kzalloc(&pdev->dev, sizeof(*si), GFP_KERNEL);
	if (!si)
		return -ENOMEM;

	si->dev = &pdev->dev; // TODO is required?

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	si->regbase = devm_ioremap_resource(&pdev->dev, res); /* TODO devm_ioremap_resource uses devm_ioremap (not nocache), but fortunately MIPS becomes nocache. consider other than MIPS? */
	if (IS_ERR(si->regbase))
		return PTR_ERR(si->regbase);

	/* ack SI irq (for avoid a spurious irq before first request) */
	__raw_writel(0, si->regbase + REG_STATUS);

	irq = platform_get_irq(pdev, 0);
	if (!irq)
		return -EINVAL;
	ret = devm_request_irq(&pdev->dev, irq, n64si_interrupt, 0/*noshare*/, "n64si", si);
	if (ret) {
		dev_err(&pdev->dev, "unable to grab IRQ\n");
		return ret;
	}

	mutex_init(&si->mutex);
	si->use_count = 0;
	si->ongoing = 0;
	init_timer(&si->polltimer);
	si->polltimer.function = n64si_poll;
	si->polltimer.data = (unsigned long)si;
	/* note: start timer will be done when first open. */

	/* register inputs */
	{
		int i, j;
		int err;

		for (i = 0; i < 4; i++) {
			struct input_dev *idev;
			idev = devm_input_allocate_device(&pdev->dev);
			if (!idev) {
				err = -ENOMEM;
				break;
			}

			si->inputdev[i] = idev;
			idev->dev.parent = &pdev->dev;
			input_set_drvdata(idev, si);

			idev->name = "Nintendo 64 Controller Bros. joystick";
			idev->phys = n64si_phys[i];
			idev->id.bustype = BUS_N64SI; /* BUS_HOST? as maplecontrol */
			idev->id.vendor = 0x0001; /*???*/
			idev->id.product = 0x0001; /*???*/
			idev->id.version = 0x0100; /*???*/

			idev->open = n64si_open;
			idev->close = n64si_close;

			idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS); /* NOTE: EV_KEY and EV_ABS are same BIT_WORD(=0) */
			idev->absbit[0] = BIT_MASK(ABS_X) | BIT_MASK(ABS_Y); /* NOTE: ABS_X and ABS_Y are same BIT_WORD(=0). They are automatically set by input_set_abs_params. */
			for(j = 0; j < sizeof(buttons_from_msb)/sizeof(*buttons_from_msb); j++) {
				if (buttons_from_msb[j] == -1) {
					continue;
				}
				__set_bit(buttons_from_msb[j], idev->keybit);
			}
			for (j = 0; j < 2; j++) {
				input_set_abs_params(idev, ABS_X + j, -127, 127, 0, 12); /* NOTE: ABS_X and ABS_Y are continuous value */
			}

			err = input_register_device(idev);
			if (err) {
				break;
			}
		}

		if (i < 4) {
			while(0 <= --i) {
				input_unregister_device(si->inputdev[i]);
			}
			return err;
		}
	}

	platform_set_drvdata(pdev, si);

	return 0;
}

/* TODO: __exit can be set? (non-hotpluggable, called only when this module is unloading) */
static int /*__exit*/ n64si_remove(struct platform_device *pdev)
{
	struct n64si *si = platform_get_drvdata(pdev);
	int i;

	for(i = 0; i < 4; i++) {
		input_unregister_device(si->inputdev[i]);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id n64si_of_match[] = {
	{ .compatible = "nintendo,si", },
	{},
};
MODULE_DEVICE_TABLE(of, n64si_of_match);
#endif

static struct platform_driver n64si_driver = {
	.probe = n64si_probe,
	.remove = n64si_remove,
	.driver = {
		.name = "n64si",
		.of_match_table = of_match_ptr(n64si_of_match),
	},
};
/* note: using module_p_d_probe because this n64si can be module and SI is not hot-pluggable
 * (instead of module_p_d(hotplug) or builtin_p_d*(non-modular).)
 */
module_platform_driver_probe(n64si_driver, n64si_probe);

MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo 64 Controller Bros.");
MODULE_LICENSE("GPL");
