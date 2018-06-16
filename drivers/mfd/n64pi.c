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

#define DEBUG_REQLOG

#define REG_DRAMADDR 0x00
#define REG_CARTADDR 0x04
#define REG_CART2DRAM 0x08
#define REG_DRAM2CART 0x0C
#define REG_STATUS 0x10

struct n64pi { /* represents the driver status of the PI device */
	struct device *dev; /* is a corresponding platform device */
	void __iomem *regbase; /* is a base virtual address of PI DMA registers (ie. 0xA4600000) */
	void __iomem *membase; /* is a base virtual address of CPU word access to PI including SRAM/64DD/ROM area (ie. 0xA5000000) */
	spinlock_t lock; /* is a lock for this state container */
	struct list_head queue; /* is a request queue */
	struct n64pi_request *curreq; /* is a current processing request */
	int ed64_enabled; /* holds current ED64 regs enabled count */

	/* TODO move to n64pi_request? but it is bloating a 32bit word more... */
	dma_addr_t curbusaddr; /* holds mapped device address for DMA (valid only while DMAing for curreq) */
};

#ifdef DEBUG_REQLOG
unsigned n64pi_log[4*1024];
unsigned n64pi_logi = 0;

static void n64pi_log_put(unsigned v)
{
	n64pi_log[n64pi_logi] = v;
	n64pi_logi = (n64pi_logi + 1) % (sizeof(n64pi_log)/sizeof(*n64pi_log));
}

static void n64pi_log_req(struct n64pi_request *req)
{
	n64pi_log_put(jiffies);
	switch(req->type) {
	case N64PI_RTY_C2R_WORD:
	case N64PI_RTY_R2C_WORD:
		n64pi_log_put(req->type << 28);
		n64pi_log_put(req->cart_address);
		n64pi_log_put(req->value);
		break;
	case N64PI_RTY_C2R_DMA:
	case N64PI_RTY_R2C_DMA:
		n64pi_log_put((req->type << 28) | req->length);
		n64pi_log_put(req->cart_address);
		n64pi_log_put((unsigned)req->ram_vaddress);
		n64pi_log_put(((unsigned*)req->ram_vaddress)[0]);
		n64pi_log_put(((unsigned*)req->ram_vaddress)[1]);
		n64pi_log_put(((unsigned*)req->ram_vaddress)[2]);
		n64pi_log_put(((unsigned*)req->ram_vaddress)[3]);
		break;
	case N64PI_RTY_RESET:
	case N64PI_RTY_GET_STATUS:
	case N64PI_RTY_ED64_ENABLE:
	case N64PI_RTY_ED64_DISABLE:
		n64pi_log_put(req->type << 28);
		n64pi_log_put(0);
		n64pi_log_put(0);
		break;
	}
}
#endif

static void ed64_dummyread(void __iomem *membase)
{
	__raw_readl(membase + 0x00);
}

/*
static unsigned int ed64_regread(void __iomem *membase, unsigned int regoff)
{
	ed64_dummyread(membase); // dummy read required!!
	return __raw_readl(membase + regoff);
}
*/

static void ed64_regwrite(void __iomem *membase, unsigned int value, unsigned int regoff)
{
	ed64_dummyread(membase); // dummy read required!!
	__raw_writel(value, membase + regoff);
}

static void ed64_enable(void __iomem *membase)
{
	ed64_regwrite(membase, 0x1234, 0x20);
}

static void ed64_disable(void __iomem *membase)
{
	ed64_regwrite(membase, 0, 0x20);
}

static void do_one_request(struct n64pi *pi)
{
	unsigned long flags;
	struct n64pi_request *req;

	spin_lock_irqsave(&pi->lock, flags);

	if (pi->curreq) {
		/* another request is ongoing. do nothing here now. */
		spin_unlock_irqrestore(&pi->lock, flags);
		return;
	}

next:
	if (list_empty(&pi->queue)) {
		/* nothing enqueued */
		spin_unlock_irqrestore(&pi->lock, flags);
		return;
	}

	/* dequeue a request into (cur)req */
	req = pi->curreq = list_first_entry(&pi->queue, struct n64pi_request, node);
	list_del_init(&req->node);

#ifdef DEBUG_REQLOG
	n64pi_log_req(req);
#endif

	spin_unlock_irqrestore(&pi->lock, flags);

	/* PI must be steady (or other driver manipulates PI!?) */
	/* but if current request is RESET, it is expected. (so callee decided issue a RESET request.) */
	if ((__raw_readl(pi->regbase + REG_STATUS) & 7) && (req->type != N64PI_RTY_RESET)) {
		/* unexpected busy or has an error, reset PI to abort it (and clear error bit) */
		dev_err(pi->dev, "Unexpected device busy or has error (cmd=%d); resetting.\n", req->type);
		__raw_writel(0x00000003, pi->regbase + REG_STATUS);
	}

	switch (req->type) {
	case N64PI_RTY_C2R_WORD:
	case N64PI_RTY_R2C_WORD:
		if (req->cart_address < 0x05000000U || 0x1FD00000U <= req->cart_address) {
			dev_err(pi->dev, "%s Word cart address out of range: %08X; skipping.\n", req->type == N64PI_RTY_C2R_WORD ? "Cart2RAM" : "RAM2Cart", req->cart_address);
			req->on_error(req); /* note: I give you req... free that if you want! */

			spin_lock_irqsave(&pi->lock, flags);
			pi->curreq = NULL;
			goto next;
		}

		/* do that request now. */

		switch (req->type) {
		case N64PI_RTY_C2R_WORD:
			req->value = __raw_readl(pi->membase + req->cart_address - 0x05000000U); /* TODO hard coding offset!! */
			break;
		case N64PI_RTY_R2C_WORD:
			__raw_writel(req->value, pi->membase + req->cart_address - 0x05000000U); /* TODO hard coding offset!! */
			break;
		default: BUG(); /* FIXME compiler! */
		}

		req->status = __raw_readl(pi->regbase + REG_STATUS);
		req->on_complete(req); /* note: I give you req... free that if you want! */

		spin_lock_irqsave(&pi->lock, flags);
		pi->curreq = NULL; /* forget after calling on_complete to avoid nested request in on_complete. */
		goto next;
	// TODO only for ED64-carts, check it and call on_error.
	case N64PI_RTY_ED64_ENABLE:
	case N64PI_RTY_ED64_DISABLE:
		switch (req->type) {
		case N64PI_RTY_ED64_ENABLE:
			if (pi->ed64_enabled == 0) {
				ed64_enable(pi->membase);
			}
			// TODO INT_MAX check?
			pi->ed64_enabled++;
			break;
		case N64PI_RTY_ED64_DISABLE:
			if (pi->ed64_enabled == 1) {
				ed64_disable(pi->membase);
			}
			pi->ed64_enabled--;
			if (pi->ed64_enabled < 0) {
				dev_err(pi->dev, "too many ED64 disable request; reset count to 0\n");
				pi->ed64_enabled = 0;
			}
			break;
		default: BUG(); /* FIXME compiler! */
		}

		req->status = __raw_readl(pi->regbase + REG_STATUS);
		req->on_complete(req); /* note: I give you req... free that if you want! */

		spin_lock_irqsave(&pi->lock, flags);
		pi->curreq = NULL; /* forget after calling on_complete to avoid nested request in on_complete. */
		goto next;
	case N64PI_RTY_C2R_DMA:
	case N64PI_RTY_R2C_DMA:
		if (req->cart_address < 0x05000000U || 0x1FD00000U <= req->cart_address) {
			dev_err(pi->dev, "%s DMA cart address out of range: %08X(+%08X); skipping.\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->cart_address, req->length);
			req->on_error(req); /* note: I give you req... free that if you want! */

			spin_lock_irqsave(&pi->lock, flags);
			pi->curreq = NULL;
			goto next;
		}
		{
			uint32_t end = req->cart_address + req->length - 1;
			if (end < 0x05000000U || 0x1FD00000U <= end || end < req->cart_address) {
				dev_err(pi->dev, "%s DMA length out of range: %08X+%08X; skipping.\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->cart_address, req->length);
				req->on_error(req); /* note: I give you req... free that if you want! */

				spin_lock_irqsave(&pi->lock, flags);
				pi->curreq = NULL;
				goto next;
			}
		}
		if ((req->length & 7) != 0) {
			dev_err(pi->dev, "%s DMA length is not aligned: (%08X+)%08X; sub driver program error? skipping.\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->cart_address, req->length);
			req->on_error(req); /* note: I give you req... free that if you want! */

			spin_lock_irqsave(&pi->lock, flags);
			pi->curreq = NULL;
			goto next;
		}

		/* start DMA here, complete on interrupt. */

		pi->curbusaddr = dma_map_single(pi->dev, req->ram_vaddress, req->length, req->type == N64PI_RTY_C2R_DMA ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
		if(dma_mapping_error(pi->dev, pi->curbusaddr)) {
			dev_err(pi->dev, "%s DMA can't map DMA buffer: %p+%08X; skipping.\n", req->type == N64PI_RTY_C2R_DMA ? "Cart2RAM" : "RAM2Cart", req->ram_vaddress, req->length);
			req->on_error(req); /* note: I give you req... free that if you want! */

			spin_lock_irqsave(&pi->lock, flags);
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
				req->on_error(req); /* note: I give you req... free that if you want! */

				spin_lock_irqsave(&pi->lock, flags);
				pi->curreq = NULL;
				goto next;
			}
		}

		/* interrupt tells this DMA is done. we are done at now. */

		return;
	case N64PI_RTY_RESET:
		/* do that request now. */

		/* reset PI */
		__raw_writel(0x00000003, pi->regbase + REG_STATUS);

		req->status = __raw_readl(pi->regbase + REG_STATUS);
		req->on_complete(req); /* note: I give you req... free that if you want! */

		spin_lock_irqsave(&pi->lock, flags);
		pi->curreq = NULL; /* forget after calling on_complete to avoid nested request in on_complete. */
		goto next;
	case N64PI_RTY_GET_STATUS:
		/* do that request now. */

		req->status = __raw_readl(pi->regbase + REG_STATUS);
		req->on_complete(req); /* note: I give you req... free that if you want! */

		spin_lock_irqsave(&pi->lock, flags);
		pi->curreq = NULL; /* forget after calling on_complete to avoid nested request in on_complete. */
		goto next;
	default:
		dev_err(pi->dev, "Unknown request type: %d; skipping.\n", req->type);

		spin_lock_irqsave(&pi->lock, flags);
		pi->curreq = NULL;
		goto next;
	}
}

int n64pi_request_async(struct n64pi *pi, struct n64pi_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);
	list_add_tail(&req->node, &pi->queue);
	spin_unlock_irqrestore(&pi->lock, flags);

	do_one_request(pi); /* to kick enqueued request if nothing is run */

	return 0;
}
EXPORT_SYMBOL_GPL(n64pi_request_async);

/* TODO consider removing this; left as proof of concept */
int n64pi_many_request_async(struct n64pi *pi, struct list_head *reqlist)
{
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);
	list_splice_tail_init(reqlist, &pi->queue);
	spin_unlock_irqrestore(&pi->lock, flags);

	do_one_request(pi); /* to kick enqueued request if nothing is run */

	return 0;
}
EXPORT_SYMBOL_GPL(n64pi_many_request_async);

int n64pi_wedge_request_async(struct n64pi *pi, struct n64pi_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);
	list_add(&req->node, &pi->queue);
	spin_unlock_irqrestore(&pi->lock, flags);

	/* don't do_one_request here, as this function is called in on_complete that ensures following request processing. */
	/* though calling do_one_request here is safe. (it does nothing because curreq should be not NULL.) */

	return 0;
}
EXPORT_SYMBOL_GPL(n64pi_wedge_request_async);

static void n64pi_complete_completion(struct n64pi_request *req)
{
	complete((struct completion *)req->cookie);
	/* don't kfree req completion waiter does it. */
}

/* TODO this may not be required, consider removing (with above n64pi_complete_completion) */
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
EXPORT_SYMBOL_GPL(n64pi_request_sync);

void
n64pi_free_request(struct n64pi_request *req)
{
	kfree(req);
}
EXPORT_SYMBOL_GPL(n64pi_free_request);

static irqreturn_t n64pi_isr(int irq, void *dev_id)
{
	struct n64pi *pi = dev_id;
	uint32_t status;
	struct n64pi_request *req;

	/* get current PI status before do anything */
	status = __raw_readl(pi->regbase + REG_STATUS);

	/* ack this interrupt (before potential next request in on_complete, not to ack next request... cause infinite wait!) */
	__raw_writel(2, pi->regbase + REG_STATUS); // [1]=clear_intr

	/* TODO uh... spin_lock_irqsave/spin_unlock_irqrestore is bad? thinking it just for paranoid. */
	/* TODO atomic op? but curreq and queue have tight relation. */
	spin_lock(&pi->lock);
	req = pi->curreq;
	spin_unlock(&pi->lock);

	if (req == NULL) {
		dev_err(pi->dev, "Spurious interrupt: status=%08X; resetting.\n", status);

		/* reset PI */
		__raw_writel(0x00000003, pi->regbase + REG_STATUS);
	} else {
		/* valid current request found; interrupt means current request's DMA has done */

		/* unmap its buffer */
		dma_unmap_single(pi->dev, pi->curbusaddr, req->length, req->type == N64PI_RTY_C2R_DMA ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

#ifdef DEBUG_REQLOG
		n64pi_log_put(((unsigned*)(req->cart_address + 0xB0000000))[0]);
		n64pi_log_put(((unsigned*)(req->cart_address + 0xB0000000))[1]);
		n64pi_log_put(((unsigned*)(req->cart_address + 0xB0000000))[2]);
		n64pi_log_put(((unsigned*)(req->cart_address + 0xB0000000))[3]);
#endif

		/* completes ran request */
		req->status = status;
		req->on_complete(req); /* note: I give you req... free that if you want! */

		spin_lock(&pi->lock);
		pi->curreq = NULL; /* forget after calling on_complete to avoid nested request in on_complete. */
		spin_unlock(&pi->lock);

		do_one_request(pi); /* kick next request if already enqueued next, or on_complete does it. */
	}

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
	pi->regbase = devm_ioremap_resource(&pdev->dev, res); /* TODO devm_ioremap_resource uses devm_ioremap (not nocache), but MIPS becomes nocache. consider other than MIPS? */
	if (IS_ERR(pi->regbase))
		return PTR_ERR(pi->regbase);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1); // 1st is memmem
	if (!res)
		return -EINVAL;
	pi->membase = devm_ioremap_resource(&pdev->dev, res); /* TODO devm_ioremap_resource uses devm_ioremap (not nocache), but MIPS becomes nocache. consider other than MIPS? */
	if (IS_ERR(pi->membase))
		return PTR_ERR(pi->membase);

	// reset the PI (for avoid a spurious irq on next irq request)
	__raw_writel(0x00000003, pi->regbase + REG_STATUS);

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
