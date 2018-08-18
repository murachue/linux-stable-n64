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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/mfd/n64pi.h>

//#define DEBUG_REQLOG

#define REG_DRAMADDR 0x00
#define REG_CARTADDR 0x04
#define REG_DRAM2CART 0x08
#define REG_CART2DRAM 0x0C
#define REG_STATUS 0x10

struct n64pi { /* represents the driver status of the PI device */
	struct device *dev; /* is a corresponding platform device */
	void __iomem *regbase; /* is a base virtual address of PI DMA registers (ie. 0xA4600000) */
	void __iomem *membase; /* is a base virtual address of CPU word access to PI including SRAM/64DD/ROM area (ie. 0xA5000000) */
	spinlock_t lock; /* is a lock for this state container */
	int ed64_enabled; /* holds current ED64 regs enabled count */

	/* info about current ongoing DMA */
	const char *nonblock_op; /* points to op (const)string that is currently running, or NULL if DMA is not running */
	dma_addr_t curbusaddr; /* holds mapped device address for DMA (valid only while nonblock DMAing) */
	uint32_t length; /* holds mapped DMA buffer length */
	int is_write; /* holds DMA direction */
	n64pi_on_interrupt_t on_interrupt; /* points to current nonblock request's callback, NULL if no nonblock request is on going. */
	void *on_interrupt_cookie; /* holds cookie for on_interrupt callback */

	/* info about queued operations; fixed-array for less memory usage, and its ok because limited sub-drivers, and it should queue only one per one driver. */
	struct n64pi_opqueue {
		void *ram_vaddr;
		uint32_t cart_addr;
		uint32_t length;
		n64pi_before_dma_t before_dma;
		n64pi_on_interrupt_t on_interrupt;
		void *cookie;
		int is_write;
	} opqueue[4];
	uint32_t opqueue_len;
};

#ifdef DEBUG_REQLOG
unsigned n64pi_log[4*1024];
unsigned n64pi_logi = 0;

static void
n64pi_log_put(unsigned v) {
	n64pi_log[n64pi_logi % (sizeof(n64pi_log)/sizeof(*n64pi_log))] = v;
	n64pi_logi++;
}
#endif

static void
ed64_dummyread(void __iomem *membase) {
	__raw_readl(membase + 0x00);
}

/*
static unsigned int
ed64_regread(void __iomem *membase, unsigned int regoff)
{
	ed64_dummyread(membase); // dummy read required!!
	return __raw_readl(membase + regoff);
}
*/

static void
ed64_regwrite(void __iomem *membase, unsigned int value, unsigned int regoff) {
	ed64_dummyread(membase); // dummy read required!!
	__raw_writel(value, membase + regoff);
}

static void
ed64_enable(void __iomem *membase) {
	ed64_regwrite(membase, 0x1234, 0x20);
}

static void
ed64_disable(void __iomem *membase) {
	ed64_regwrite(membase, 0, 0x20);
}

static int
n64pi_is_busy(struct n64pi *pi) {
	return __raw_readl(pi->regbase + REG_STATUS) & 3;
}

static void
n64pi_block_until_pi_completion(struct n64pi *pi) {
	while (n64pi_is_busy(pi)) /* nothing */ ;
}

static uint32_t
n64pi_ensure_noerror(struct n64pi *pi, const char *op) {
	uint32_t status = __raw_readl(pi->regbase + REG_STATUS);

	if (status & 4) {
		/* unexpected error, reset PI to clear error flag */
		dev_err(pi->dev, "%s: Unexpected error (status=%x); resetting.\n", op, status);
		__raw_writel(0x00000003, pi->regbase + REG_STATUS);
	}

	return status;
}

static int
n64pi_validate_dma(struct n64pi *pi, const char *op, uint32_t cart_addr, uint32_t length) {
	if (cart_addr < 0x05000000U || 0x1FD00000U <= cart_addr) {
		dev_err(pi->dev, "%s: DMA cart address out of range: %08X(+%08X); skipping.\n", op, cart_addr, length);
		return N64PI_ERROR_ADDRESSING;
	}
	{
		uint32_t end = cart_addr + length - 1;
		if (end < 0x05000000U || 0x1FD00000U <= end || end < cart_addr) {
			dev_err(pi->dev, "%s: DMA length out of range: %08X+%08X; skipping.\n", op, cart_addr, length);
			return N64PI_ERROR_ADDRESSING;
		}
	}
	if (length == 0 || (length & 7) != 0) {
		dev_err(pi->dev, "%s: DMA length is invalid: (%08X+)%08X; sub driver program error? skipping.\n", op, cart_addr, length);
		return N64PI_ERROR_ADDRESSING;
	}

	return N64PI_ERROR_SUCCESS;
}

static int
n64pi_validate_word(struct n64pi *pi, const char *op, uint32_t cart_addr) {
	if (cart_addr < 0x05000000U || 0x1FD00000U <= cart_addr) {
		dev_err(pi->dev, "%s: Word cart address out of range: %08X; skipping.\n", op, cart_addr);
		return N64PI_ERROR_ADDRESSING;
	}

	return N64PI_ERROR_SUCCESS;
}

static int
n64pi_dma(struct n64pi *pi, const char *op, void *ram_vaddr, uint32_t cart_addr, uint32_t length, dma_addr_t *pcurbusaddr, int is_write) {
	int error;

	if ((error = n64pi_validate_dma(pi, op, cart_addr, length)) != N64PI_ERROR_SUCCESS) {
		return error;
	}

	/* getting buffer ready for DMA */
	*pcurbusaddr = dma_map_single(pi->dev, ram_vaddr, length, is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if(dma_mapping_error(pi->dev, *pcurbusaddr)) {
		dev_err(pi->dev, "%s: Can't map DMA buffer: %p+%08X; skipping.\n", op, ram_vaddr, length);
		return N64PI_ERROR_DMAMAP;
	}

	/* fire DMA */
	__raw_writel(cart_addr, pi->regbase + REG_CARTADDR);
	__raw_writel(*pcurbusaddr, pi->regbase + REG_DRAMADDR);
	mb();
	__raw_writel(length - 1, pi->regbase + (is_write ? REG_DRAM2CART : REG_CART2DRAM));
	mb();

	/* test immediate error */
	{
		uint32_t status = __raw_readl(pi->regbase + REG_STATUS);
		if (status & 4) {
			/* error... reset PI to forcibly stop DMA */
			dev_err(pi->dev, "%s: PI DMA error: (%08X+)%08X status=%08X; reset and skipping!\n", op, cart_addr, length, status);
			__raw_writel(0x00000003, pi->regbase + REG_STATUS);

			dma_unmap_single(pi->dev, *pcurbusaddr, length, is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

			return N64PI_ERROR_PIDMA;
		}
	}

	return N64PI_ERROR_SUCCESS;
}

static int
n64pi_get_intr_pending(struct n64pi *pi) {
	/* FIXME ouch! touching non-PI region without ioremap!! */
	/*       but no such bit in PI registers... */
	return __raw_readl((void*)CKSEG1ADDR(0x04300008)) & (1 << 4);
}

static int
n64pi_cleanup_dma(struct n64pi *pi, const char *op, dma_addr_t curbusaddr, uint32_t length, int is_write) {
	dma_unmap_single(pi->dev, curbusaddr, length, is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	/* test DMA error TODO is this required? (error bit for request after DMA, not this DMA itself) */
	{
		uint32_t status = __raw_readl(pi->regbase + REG_STATUS);
		/* Make PI steady */
		if (status & 4) {
			/* unexpected error, reset PI to clear error flag */
			dev_err(pi->dev, "%s: Unexpected error after DMA (status=%x); resetting.\n", op, status);
			__raw_writel(0x00000003, pi->regbase + REG_STATUS);
			return N64PI_ERROR_PIDMA;
		}
	}

	return N64PI_ERROR_SUCCESS;
}

static int
n64pi_blocking_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length, int is_write) {
	int error;
	unsigned long flags;
	uint32_t status;
	int intr_pending;
	dma_addr_t curbusaddr;
	n64pi_on_interrupt_t on_interrupt;
	const char *op = is_write ? "b_w_d" : "b_r_d";

	spin_lock_irqsave(&pi->lock, flags);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(((is_write ? 2 : 1) << 28) | length);
	n64pi_log_put(cart_addr);
	n64pi_log_put((unsigned)ram_vaddr);
#endif

	/* Wait until PI is busy ("blocking") */
	n64pi_block_until_pi_completion(pi);

	/* remember status for potentially invoking nonblock on_interrupt */
	intr_pending = n64pi_get_intr_pending(pi);
	/* Remember and make PI steady */
	status = n64pi_ensure_noerror(pi, op);

	if ((error = n64pi_dma(pi, op, ram_vaddr, cart_addr, length, &curbusaddr, is_write)) != N64PI_ERROR_SUCCESS) {
		spin_unlock_irqrestore(&pi->lock, flags);
		return error;
	}

	/* wait for DMA completion ("blocking") */
	n64pi_block_until_pi_completion(pi);

	if ((error = n64pi_cleanup_dma(pi, op, curbusaddr, length, is_write)) != N64PI_ERROR_SUCCESS) {
		spin_unlock_irqrestore(&pi->lock, flags);
		return error;
	}

	/* drop interrupt to avoid spurious interrupt */
	if (n64pi_get_intr_pending(pi)) {
		__raw_writel(0x00000002, pi->regbase + REG_STATUS);
	}

	/* slurp on_interrupt to avoid double-call on just after spin_unlock. */
	on_interrupt = pi->on_interrupt;
	pi->on_interrupt = NULL;

	spin_unlock_irqrestore(&pi->lock, flags);

	/* emulate interrupt here (if it was). NOTE on_interrupt may be call n64pi_* that spinlocks, so pi->lock must be unlocked. */
	if (on_interrupt && intr_pending) {
		on_interrupt(pi, pi->on_interrupt_cookie, status);
	}

	return N64PI_ERROR_SUCCESS;
}

static int n64pi_enqueue_op(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length, n64pi_before_dma_t before_dma, n64pi_on_interrupt_t on_interrupt, void *cookie, int is_write) {
	const char *op = is_write ? "n_w_d" : "n_r_d";

	if (sizeof(pi->opqueue)/sizeof(pi->opqueue[0]) <= pi->opqueue_len) {
		/* queue full */
		dev_err(pi->dev, "%s: opqueue full; skipping! %p/%08X+%08X)\n", op, ram_vaddr, cart_addr, length);
		return N64PI_ERROR_NOMEM;
	}

	/* enqueue */
	{
		struct n64pi_opqueue * const opq = &pi->opqueue[pi->opqueue_len];
		opq->ram_vaddr = ram_vaddr;
		opq->cart_addr = cart_addr;
		opq->length = length;
		opq->before_dma = before_dma;
		opq->on_interrupt = on_interrupt;
		opq->cookie = cookie;
		opq->is_write = is_write;
	}
	pi->opqueue_len++;

	/* caller wants this */
	return N64PI_ERROR_BUSY;
}

static int
n64pi_do_nonblock_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length, n64pi_before_dma_t before_dma, n64pi_on_interrupt_t on_interrupt, void *cookie, int is_write) {
	int error;
	const char *op = is_write ? "n_w_d" : "n_r_d";

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(((is_write ? 104 : 103) << 28) | length);
	n64pi_log_put(cart_addr);
	n64pi_log_put((unsigned)ram_vaddr);
#endif

	/* Make PI steady */
	(void)n64pi_ensure_noerror(pi, op);

	before_dma(pi, cookie);

	if ((error = n64pi_dma(pi, op, ram_vaddr, cart_addr, length, &pi->curbusaddr, is_write)) != N64PI_ERROR_SUCCESS) {
		return error;
	}

	pi->nonblock_op = op;
	pi->length = length;
	pi->is_write = is_write;
	pi->on_interrupt = on_interrupt;
	pi->on_interrupt_cookie = cookie;

	return N64PI_ERROR_SUCCESS;
}

static int
n64pi_dequeue_and_do_nonblock_dma(struct n64pi *pi) {
	if (pi->opqueue_len <= 0) {
		/* queue is already empty; nothing to do. */
		return N64PI_ERROR_SUCCESS;
	}

	/* note: touching opqueue[--len] directly is safe here, because pi is locked and this is tailcall(ref to opqueue is only here before calling n64pi_do_nonblock_dma). */
	{
		struct n64pi_opqueue * const opq = &pi->opqueue[--pi->opqueue_len]; // dequeue and reference (that be freed at returning from here)
		return n64pi_do_nonblock_dma(pi, opq->ram_vaddr, opq->cart_addr, opq->length, opq->before_dma, opq->on_interrupt, opq->cookie, opq->is_write);
	}
}

static int
n64pi_nonblock_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length, n64pi_before_dma_t before_dma, n64pi_on_interrupt_t on_interrupt, void *cookie, int is_write) {
	int error;
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(((is_write ? 4 : 3) << 28) | length);
	n64pi_log_put(cart_addr);
	n64pi_log_put((unsigned)ram_vaddr);
#endif

	if (n64pi_is_busy(pi) || pi->nonblock_op) {
		/* PI DMA is busy, or just finished but not callbacked yet... enqueue this op. */
		error = n64pi_enqueue_op(pi, ram_vaddr, cart_addr, length, before_dma, on_interrupt, cookie, is_write);
		spin_unlock_irqrestore(&pi->lock, flags);
		return error;
	}

	/* PI is idle, do it now (without enqueueing) */
	error = n64pi_do_nonblock_dma(pi, ram_vaddr, cart_addr, length, before_dma, on_interrupt, cookie, is_write);

	spin_unlock_irqrestore(&pi->lock, flags);

	return error;
}

int
n64pi_blocking_read_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length) {
	return n64pi_blocking_dma(pi, ram_vaddr, cart_addr, length, 0);
}
EXPORT_SYMBOL_GPL(n64pi_blocking_read_dma);

int
n64pi_nonblock_read_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length, n64pi_before_dma_t before_dma, n64pi_on_interrupt_t on_interrupt, void *cookie) {
	return n64pi_nonblock_dma(pi, ram_vaddr, cart_addr, length, before_dma, on_interrupt, cookie, 0);
}
EXPORT_SYMBOL_GPL(n64pi_nonblock_read_dma);

int
n64pi_blocking_write_dma(struct n64pi *pi, uint32_t cart_addr, void *ram_vaddr, uint32_t length) {
	return n64pi_blocking_dma(pi, ram_vaddr, cart_addr, length, 1);
}
EXPORT_SYMBOL_GPL(n64pi_blocking_write_dma);

int
n64pi_nonblock_write_dma(struct n64pi *pi, uint32_t cart_addr, void *ram_vaddr, uint32_t length, n64pi_before_dma_t before_dma, n64pi_on_interrupt_t on_interrupt, void *cookie) {
	return n64pi_nonblock_dma(pi, ram_vaddr, cart_addr, length, before_dma, on_interrupt, cookie, 1);
}
EXPORT_SYMBOL_GPL(n64pi_nonblock_write_dma);

/* TODO how about addressing error? but I don't want to store value into memory... introduce "value_on_error"? */
uint32_t
n64pi_blocking_read_word(struct n64pi *pi, uint32_t cart_addr) {
	int error;
	unsigned long flags;
	uint32_t value;

	spin_lock_irqsave(&pi->lock, flags);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(5 << 28);
	n64pi_log_put(cart_addr);
#endif

	/* Wait until PI is busy ("blocking") */
	n64pi_block_until_pi_completion(pi);

	/* Make PI steady */
	(void)n64pi_ensure_noerror(pi, "b_r_w");

	if ((error = n64pi_validate_word(pi, "b_r_w", cart_addr)) != N64PI_ERROR_SUCCESS) {
		spin_unlock_irqrestore(&pi->lock, flags);
		return error; /* TODO invalid! */
	}

	value = __raw_readl(pi->membase + cart_addr - 0x05000000U); /* TODO hard coding membase offset!! */

#ifdef DEBUG_REQLOG
	n64pi_log_put(value);
#endif

	spin_unlock_irqrestore(&pi->lock, flags);

	return value;
}
EXPORT_SYMBOL_GPL(n64pi_blocking_read_word);

int
n64pi_blocking_write_word(struct n64pi *pi, uint32_t cart_addr, uint32_t value) {
	int error;
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(6 << 28);
	n64pi_log_put(cart_addr);
	n64pi_log_put(value);
#endif

	/* Wait until PI is busy ("blocking") */
	n64pi_block_until_pi_completion(pi);

	/* Make PI steady */
	(void)n64pi_ensure_noerror(pi, "b_w_w");

	if ((error = n64pi_validate_word(pi, "b_w_w", cart_addr)) != N64PI_ERROR_SUCCESS) {
		spin_unlock_irqrestore(&pi->lock, flags);
		return error; /* TODO invalid! */
	}

	__raw_writel(value, pi->membase + cart_addr - 0x05000000U); /* TODO hard coding membase offset!! */

	spin_unlock_irqrestore(&pi->lock, flags);

	return error;
}
EXPORT_SYMBOL_GPL(n64pi_blocking_write_word);

void
n64pi_reset(struct n64pi *pi) {
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(7 << 28);
	n64pi_log_put(0);
	n64pi_log_put(0);
#endif

	__raw_writel(0x00000003, pi->regbase + REG_STATUS);

	spin_unlock_irqrestore(&pi->lock, flags);
}
EXPORT_SYMBOL_GPL(n64pi_blocking_reset);

uint32_t
n64pi_get_status(struct n64pi *pi) {
	unsigned long flags;
	uint32_t status;

	spin_lock_irqsave(&pi->lock, flags);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(8 << 28);
	n64pi_log_put(0);
#endif

	status = __raw_readl(pi->regbase + REG_STATUS);

#ifdef DEBUG_REQLOG
	n64pi_log_put(status);
#endif

	spin_unlock_irqrestore(&pi->lock, flags);

	return status;
}
EXPORT_SYMBOL_GPL(n64pi_blocking_get_status);

int
n64pi_blocking_ed64_enable(struct n64pi *pi) {
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(9 << 28);
	n64pi_log_put(0);
	n64pi_log_put(pi->ed64_enabled);
#endif

	if (pi->ed64_enabled == 0) {
		ed64_enable(pi->membase - 0x05000000U + 0x08040000); /* TODO hard coding membase offset!! */
		n64pi_block_until_pi_completion(pi); // ED64 regs seems slow, wait for completion.
	}
	// TODO INT_MAX check?
	pi->ed64_enabled++;

	spin_unlock_irqrestore(&pi->lock, flags);

	return N64PI_ERROR_SUCCESS;
}
EXPORT_SYMBOL_GPL(n64pi_blocking_ed64_enable);

int
n64pi_blocking_ed64_disable(struct n64pi *pi) {
	unsigned long flags;

	spin_lock_irqsave(&pi->lock, flags);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(10 << 28);
	n64pi_log_put(0);
	n64pi_log_put(pi->ed64_enabled);
#endif

	if (pi->ed64_enabled == 1) {
		ed64_disable(pi->membase - 0x05000000U + 0x08040000); /* TODO hard coding membase offset!! */
		n64pi_block_until_pi_completion(pi); // ED64 regs seems slow, wait for completion.
	}
	pi->ed64_enabled--;
	if (pi->ed64_enabled < 0) {
		dev_err(pi->dev, "too many ED64 disable request; reset count to 0\n");
		pi->ed64_enabled = 0;
	}

	spin_unlock_irqrestore(&pi->lock, flags);

	return N64PI_ERROR_SUCCESS;
}
EXPORT_SYMBOL_GPL(n64pi_blocking_ed64_disable);

static irqreturn_t n64pi_isr(int irq, void *dev_id)
{
	struct n64pi *pi = dev_id;
	uint32_t status;
	const char *curop;
	dma_addr_t curbusaddr;
	uint32_t length;
	int is_write;
	n64pi_on_interrupt_t on_interrupt;
	void *cookie;

	/* spin_lock pi->lock to use pi->regbase here (and for later memorize). */
	/* TODO may not be required? we are in interrupt context that is intr-masked, no one disturb us... */
	/* TODO uh... spin_lock_irqsave/spin_unlock_irqrestore is bad? thinking it just for paranoid. */
	spin_lock(&pi->lock);

	/* get current PI status before do anything */
	status = __raw_readl(pi->regbase + REG_STATUS);

	/* ack this interrupt (before potential next request in on_complete, not to ack next request... cause infinite wait!) */
	__raw_writel(2, pi->regbase + REG_STATUS); // [1]=clear_intr

	/* memorize in lock means atomic */
	curop = pi->nonblock_op;
	curbusaddr = pi->curbusaddr;
	length = pi->length;
	is_write = pi->is_write;
	on_interrupt = pi->on_interrupt;
	cookie = pi->on_interrupt_cookie;

	pi->nonblock_op = NULL;

	spin_unlock(&pi->lock);

	if (curop == NULL) {
		dev_err(pi->dev, "Spurious interrupt: status=%x; resetting.\n", status);

		/* reset PI */
		__raw_writel(0x00000003, pi->regbase + REG_STATUS);
	} else {
		/* valid current request found; interrupt means current request's DMA has done */
		int error;

		if ((error = n64pi_cleanup_dma(pi, curop, curbusaddr, length, is_write)) != N64PI_ERROR_SUCCESS) {
			/* uh... can't do anything here. */
		}

		/* call the on_interrupt if specified. NOTE pi->lock must be unlocked or it will deadlock (if another request is issued in on_interrupt)! */
		if (on_interrupt) {
			on_interrupt(pi, cookie, status);
		}
	}

	/* optionally run enqueued op */
	n64pi_dequeue_and_do_nonblock_dma(pi);

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
	pi->nonblock_op = NULL;
	pi->ed64_enabled = 0;
	pi->opqueue_len = 0;
	platform_set_drvdata(pdev, pi);

	/* do this after device.driver_data is populated, cuz children reference it as dev.parent.driver_data. */
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
