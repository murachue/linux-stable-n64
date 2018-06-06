/*
 * Nintendo 64 PI(Peripheral Interface) support
 * Copyright (c) 2018 Murachue <murachue+github@gmail.com>
 *
 * This driver provides a storage and optionally a virtual tty functionality,
 * mainly for arbitrating their accesses.
 * Based on ipaq-micro.c.
 *
 * TODO support 1FD00000-7FFFffff (Cartridge domain 1 address 3)?
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/string.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/mfd/n64pi.h>

#define REG_DRAMADDR 0x00
#define REG_CARTADDR 0x04
#define REG_CART2DRAM 0x08
#define REG_DRAM2CART 0x0C
#define REG_STATUS 0x10

static void do_one_request_locked(struct n64pi *pi)
{
	struct n64pi_request *req;

	if (pi->curreq) {
		/* another request is ongoing. do nothing here now. */
		return;
	}

next:
	if (list_empty(&pi->queue)) {
		/* nothing enqueued...?? */
		return;
	}

	/* dequeue a request into curreq */
	req = pi->curreq = list_entry(pi->queue.next, struct n64pi_request, node);
	list_del_init(&req->node);

	/* reset PI */
	__raw_writel(0x00000003, pi->regbase + REG_STATUS);

	switch (req->type) {
	case N64PI_RTY_C2R_WORD:
	case N64PI_RTY_R2C_WORD:
		if (req->cart_address < 0x05000000U || 0x1FD00000U <= req->cart_address) {
			dev_err(pi->dev, "%s Word cart address out of range: %08X; skipping.\n", req->type == N64PI_RTY_C2R_WORD ? "Cart2RAM" : "RAM2Cart", req->cart_address);
			pi->curreq = NULL;
			goto next;
		}

		switch (req->type) {
		case N64PI_RTY_C2R_WORD:
			req->value = __raw_readl(pi->membase + req->cart_address - 0x05000000U); /* TODO offset is hard coding!! */
			break;
		case N64PI_RTY_R2C_WORD:
			__raw_writel(req->value, pi->membase + req->cart_address - 0x05000000U); /* TODO offset is hard coding!! */
			break;
		default: BUG(); /* FIXME compiler! */
		}

		req->status = __raw_readl(pi->regbase + REG_STATUS);
		req->on_complete(req);
		pi->curreq = NULL; /* forget after calling on_complete to avoid nested request in on_complete. */
		goto next;
	case N64PI_RTY_C2R_DMA:
	case N64PI_RTY_R2C_DMA:
		if (req->cart_address < 0x05000000U || 0x1FD00000U <= req->cart_address) {
			dev_err(pi->dev, "%s DMA cart address out of range: %08X(+%08X); skipping.\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->cart_address, req->length);
			pi->curreq = NULL;
			goto next;
		}
		{
			uint32_t end = req->cart_address + req->length - 1;
			if (end < 0x05000000U || 0x1FD00000U <= end || end < req->cart_address) {
				dev_err(pi->dev, "%s DMA length out of range: %08X+%08X; skipping.\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->cart_address, req->length);
				pi->curreq = NULL;
				goto next;
			}
		}
		if ((req->length & 7) != 0) {
			dev_err(pi->dev, "%s DMA length is not aligned: (%08X+)%08X; sub driver program error? but continuing!\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->cart_address, req->length);
		}

		pi->curbusaddr = dma_map_single(pi->dev, req->ram_vaddress, req->length, req->type == N64PI_RTY_C2R_DMA ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
		if(dma_mapping_error(pi->dev, pi->curbusaddr)) {
			dev_err(pi->dev, "%s DMA can't map DMA buffer: %p+%08X; skipping.\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->ram_vaddress, req->length);
			pi->curreq = NULL;
			goto next;
		}

		__raw_writel(req->cart_address, pi->regbase + REG_CARTADDR);
		__raw_writel(pi->curbusaddr, pi->regbase + REG_DRAMADDR); // TODO not vaddr but devaddr(paddr)
		mb();
		switch (req->type) {
		case N64PI_RTY_C2R_DMA:
			__raw_writel(req->length - 1, pi->regbase + REG_CART2DRAM);
			break;
		case N64PI_RTY_R2C_DMA:
			__raw_writel(req->length - 1, pi->regbase + REG_DRAM2CART);
			break;
		default: BUG(); /* FIXME compiler! */
		}
		mb();
		{
			uint32_t status = __raw_readl(pi->regbase + REG_STATUS);
			if (status & 4) {
				/* reset PI to forcibly stop DMA */
				__raw_writel(0x00000003, pi->regbase + REG_STATUS);

				dev_err(pi->dev, "%s DMA error: (%08X+)%08X status=%08X; skipping!\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->cart_address, req->length, status);
				pi->curreq = NULL;
				goto next;
			}
		}

		/* interrupt tells this DMA is done. we are done at now. */

		return;
	default:
		dev_err(pi->dev, "Unknown request type: %d; skipping.\n", req->type);
		pi->curreq = NULL;
		goto next;
	}
}

int n64pi_request_async(struct n64pi *pi, struct n64pi_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);

	list_add_tail(&req->node, &pi->queue);
	do_one_request_locked(pi); /* to kick enqueued request if nothing is run */

	spin_unlock_irqrestore(&pi->lock, flags);

	return 0;
}
EXPORT_SYMBOL(n64pi_request_async);

static void n64pi_complete_completion(struct n64pi_request *req)
{
	complete((struct completion *)req->cookie);
	/* don't kfree req completion waiter does it. */
}

int n64pi_request_sync(struct n64pi *pi, struct n64pi_request *req)
{
	struct completion *pcomp;
	int ret;

	pcomp = kmalloc(sizeof(struct completion), GFP_NOIO);
	if (!pcomp) {
		return -ENOMEM;
	}
	init_completion(pcomp);
	/* this is synchronous request, caller must NOT set on_complete. */
	req->on_complete = n64pi_complete_completion;
	req->cookie = pcomp;
	ret = n64pi_request_async(pi, req);
	if (ret) {
		kfree(pcomp);
		return ret;
	}
	wait_for_completion(pcomp);

	kfree(pcomp);
	if (!ret && (req->status & 3) != 0) {
		ret = -EIO;
	}

	return ret;
}
EXPORT_SYMBOL(n64pi_request_sync);

static irqreturn_t n64pi_isr(int irq, void *dev_id)
{
	struct n64pi *pi = dev_id;
	uint32_t status;
	struct n64pi_request *req;

	/* TODO uh... spin_lock_irqsave/spin_unlock_irqrestore is bad? thinking it just for paranoid. */
	spin_lock(&pi->lock);

	status = __raw_readl(pi->regbase + REG_STATUS);

	req = pi->curreq;
	if (req == NULL) {
		dev_err(pi->dev, "Spurious interrupt: status=%08X; resetting.\n", status);
		/* reset PI */
		__raw_writel(0x00000003, pi->regbase + REG_STATUS);
	} else {
		dma_unmap_single(pi->dev, pi->curbusaddr, req->length, req->type == N64PI_RTY_C2R_DMA ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

		req->status = status;
		req->on_complete(req);
		pi->curreq = NULL; /* forget after calling on_complete to avoid nested request in on_complete. */
		do_one_request_locked(pi); /* kick next request if already enqueued next, or on_complete does it. */
	}

	spin_unlock(&pi->lock);

	return IRQ_HANDLED;
}

static const struct mfd_cell n64pi_cells[] = {
	{ .name = "n64pi-cart", .of_compatible = "nintendo,pi-cart", },
	{ .name = "n64pi-ed64tty", .of_compatible = "nintendo,pi-ed64tty" }, // TODO OF? CONFIG?
	// TODO 64drive tty support
};

static int __init n64pi_probe(struct platform_device *pdev)
{
	struct n64pi *pi;
	struct resource *res;
	int ret;
	int irq;

	if(dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(24)) != 0) {
		//dev_warn(dev, "n64cart: No suitable DMA available\n");
		// in MIPS dma-default, dma_map_ops->dma_set_mask does not defined, that cause failure.
		// and generic, dma_map_ops->map_page does not care dev, means mask is ignored.
		// so, ignore this error!!
		dev_err(&pdev->dev, "No suitable DMA available... don't care this :)\n");
		//goto out_dmaset;
	}

	pi = devm_kzalloc(&pdev->dev, sizeof(*pi), GFP_KERNEL);
	if (!pi)
		return -ENOMEM;

	pi->dev = &pdev->dev; // TODO is required?

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0); // 0th is regmem
	if (!res)
		return -EINVAL;
	pi->regbase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pi->regbase))
		return PTR_ERR(pi->regbase);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1); // 1st is memmem
	if (!res)
		return -EINVAL;
	pi->membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pi->membase))
		return PTR_ERR(pi->membase);

	// reset the PI (for avoid a spurious irq on next irq request)
	__raw_writel(0x00000003, pi->membase + REG_STATUS);

	irq = platform_get_irq(pdev, 0);
	if (!irq)
		return -EINVAL;
	ret = devm_request_irq(&pdev->dev, irq, n64pi_isr, 0/*noshare*/, "n64pi", pi);
	if (ret) {
		dev_err(&pdev->dev, "unable to grab IRQ\n");
		return ret;
	}

	spin_lock_init(&pi->lock);
	INIT_LIST_HEAD(&pi->queue);
	pi->curreq = NULL;
	platform_set_drvdata(pdev, pi);

	/* do this after device.driver_data is populated, cuz children referefence it as dev.parent.driver_data. */
	ret = mfd_add_devices(&pdev->dev, pdev->id, n64pi_cells, ARRAY_SIZE(n64pi_cells), NULL, 0, NULL);
	if (ret) {
		dev_err(&pdev->dev, "error adding MFD cells\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id n64pi_of_match[] = {
	{ .compatible = "nintendo,pi", },
	{},
};
MODULE_DEVICE_TABLE(of, n64pi_of_match);
#endif

static struct platform_driver n64pi_driver = {
	.driver   = {
		.name	= "n64pi",
		// TODO is builtin_platform_driver_probe compatible with CONFIG_OF?
		.of_match_table = of_match_ptr(n64pi_of_match),
	},
};
builtin_platform_driver_probe(n64pi_driver, n64pi_probe);

MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo 64 Peripheral Interface");
MODULE_LICENSE("GPL");
