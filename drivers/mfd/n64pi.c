/*
 * Nintendo 64 PI(Peripheral Interface) support
 * Copyright (c) 2018 Murachue <murachue+github@gmail.com>
 *
 * This driver provides a storage and optionally a virtual tty functionality,
 * mainly for arbitrating their accesses.
 * Based on ipaq-micro.c.
 *
 * TODO auto-probe ED64/64drive and enable its function only if it is probed.
 *      requires moving ED64/64drive specific function to n64pi. (ie. from n64cart/ed64tty)
 * TODO nonblocking DMA support (introduce queue? how about before_dma and on_interrupt(after_dma)?)
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
//#define DEBUG_BEGINEND

//#define ED64_KRIKZZSTYLE

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
	int ongoing; /* holds boolean means n64pi is locked or not (this have meaning only on UniProcessor that spoils spinlock) */
	unsigned long flags; /* holds irq flags */
	int ed64_enabled; /* holds current ED64 regs enabled count */
#ifdef DEBUG_BEGINEND
	void *begin_func;
	unsigned long begin_jiffies;
	void *end_func;
	unsigned long end_jiffies;
#endif
};

#ifdef DEBUG_REQLOG
static unsigned n64pi_log[4*1024];
static unsigned n64pi_logi = 0;

static void
n64pi_log_put(unsigned v) {
	n64pi_log[n64pi_logi % (sizeof(n64pi_log)/sizeof(*n64pi_log))] = v;
	n64pi_logi++;
}
#endif

static int
is_busy(struct n64pi *pi) {
	return __raw_readl(pi->regbase + REG_STATUS) & 3;
}

static void
block_until_pi_completion(struct n64pi *pi) {
	while (is_busy(pi)) /* nothing */ ;
}

static uint32_t
ensure_noerror(struct n64pi *pi, const char *op) {
	uint32_t status = __raw_readl(pi->regbase + REG_STATUS);

	if (status & 4) {
		/* unexpected error, reset PI to clear error flag */
		dev_err(pi->dev, "%s: Unexpected error (status=%x); resetting.\n", op, status);
		__raw_writel(0x00000003, pi->regbase + REG_STATUS);
	}

	return status;
}

static int
validate_dma(struct n64pi *pi, const char *op, uint32_t cart_addr, uint32_t length) {
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
validate_word(struct n64pi *pi, const char *op, uint32_t cart_addr) {
	if (cart_addr < 0x05000000U || 0x1FD00000U <= cart_addr) {
		dev_err(pi->dev, "%s: Word cart address out of range: %08X; skipping.\n", op, cart_addr);
		return N64PI_ERROR_ADDRESSING;
	}

	return N64PI_ERROR_SUCCESS;
}

static int
dma(struct n64pi *pi, const char *op, void *ram_vaddr, uint32_t cart_addr, uint32_t length, dma_addr_t *pcurbusaddr, int is_write) {
	int error;

	if ((error = validate_dma(pi, op, cart_addr, length)) != N64PI_ERROR_SUCCESS) {
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

#if 0 // keep for support nonblocking...
static int
get_intr_pending(struct n64pi *pi) {
	/* FIXME ouch! touching non-PI region without ioremap!! */
	/*       but no such bit in PI registers... */
	return __raw_readl((void*)CKSEG1ADDR(0x04300008)) & (1 << 4);
}
#endif

static int
cleanup_dma(struct n64pi *pi, const char *op, dma_addr_t curbusaddr, uint32_t length, int is_write) {
	dma_unmap_single(pi->dev, curbusaddr, length, is_write ? DMA_TO_DEVICE : DMA_FROM_DEVICE);

	/* test DMA error TODO is this required? (error bit is for last request, not this DMA itself) */
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

static void
ensure_beginned(struct n64pi *pi) {
	if (pi->ongoing) {
		return;
	}

	panic("n64pi sub device program error: something requested but not beginned");
}

static int
blocking_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length, int is_write) {
	int error;
	dma_addr_t curbusaddr;
	const char *op = is_write ? "w_d" : "r_d";

	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(((is_write ? 2 : 1) << 28) | length);
	n64pi_log_put(cart_addr);
	n64pi_log_put((unsigned)ram_vaddr);
#endif

	/* Wait until PI is busy ("blocking") */
	block_until_pi_completion(pi);

	/* Remember and make PI steady */
	(void)ensure_noerror(pi, op);

	/* Do DMA! */
	if ((error = dma(pi, op, ram_vaddr, cart_addr, length, &curbusaddr, is_write)) != N64PI_ERROR_SUCCESS) {
		return error;
	}

	/* Wait for DMA completion ("blocking") */
	block_until_pi_completion(pi);

	/* Cleanup the DMA */
	if ((error = cleanup_dma(pi, op, curbusaddr, length, is_write)) != N64PI_ERROR_SUCCESS) {
		return error;
	}

	/* Drop interrupt pending flag to avoid spurious interrupt */
	__raw_writel(0x00000002, pi->regbase + REG_STATUS);

	return N64PI_ERROR_SUCCESS;
}

void
n64pi_begin(struct n64pi *pi) {
	spin_lock_irqsave(&pi->lock, pi->flags);

	if (pi->ongoing) {
		panic("n64pi sub driver program error: recursive begin (or forgot to n64pi_end?)");
	}

	pi->ongoing = 1;
#ifdef DEBUG_BEGINEND
	pi->begin_func = *(void**)((void*)&pi - sizeof(void*));
	pi->begin_jiffies = jiffies;
#endif
}
EXPORT_SYMBOL_GPL(n64pi_begin);

int
n64pi_trybegin(struct n64pi *pi) {
	unsigned long flags; /* don't destroy pi->flags on cannot-get-lock. */

	if (!spin_trylock_irqsave(&pi->lock, flags)) {
		return N64PI_ERROR_BUSY;
	}

	if (pi->ongoing) {
		/* this path should not be happened... */
		spin_unlock_irqrestore(&pi->lock, flags);
		return N64PI_ERROR_BUSY;
	}

	pi->flags = flags; /* got lock, safe to overwrite. */

	pi->ongoing = 1;
#ifdef DEBUG_BEGINEND
	pi->begin_func = *(void**)((void*)&pi - sizeof(void*));
	pi->begin_jiffies = jiffies;
#endif

	return N64PI_ERROR_SUCCESS;
}
EXPORT_SYMBOL_GPL(n64pi_begin);

void
n64pi_end(struct n64pi *pi) {
	ensure_beginned(pi);

	/* TODO check pi->ed64_enabled == 0 */

#ifdef DEBUG_BEGINEND
	pi->end_func = *(void**)((void*)&pi - sizeof(void*));
	pi->end_jiffies = jiffies;
#endif
	pi->ongoing = 0;

	spin_unlock_irqrestore(&pi->lock, pi->flags);
}
EXPORT_SYMBOL_GPL(n64pi_end);

int
n64pi_read_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length) {
	return blocking_dma(pi, ram_vaddr, cart_addr, length, 0);
}
EXPORT_SYMBOL_GPL(n64pi_read_dma);

int
n64pi_write_dma(struct n64pi *pi, uint32_t cart_addr, void *ram_vaddr, uint32_t length) {
	return blocking_dma(pi, ram_vaddr, cart_addr, length, 1);
}
EXPORT_SYMBOL_GPL(n64pi_write_dma);

/* TODO how about addressing error? but I don't want to store value into memory... introduce "value_on_error"? */
uint32_t
n64pi_read_word(struct n64pi *pi, uint32_t cart_addr) {
	int error;
	uint32_t value;

	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(5 << 28);
	n64pi_log_put(cart_addr);
#endif

	/* Wait until PI is busy ("blocking") */
	block_until_pi_completion(pi);

	/* Make PI steady */
	(void)ensure_noerror(pi, "r_w");

	if ((error = validate_word(pi, "r_w", cart_addr)) != N64PI_ERROR_SUCCESS) {
		return error; /* TODO invalid! */
	}

	value = __raw_readl(pi->membase + cart_addr - 0x05000000U); /* TODO hard coding membase offset!! */

#ifdef DEBUG_REQLOG
	n64pi_log_put(value);
#endif

	return value;
}
EXPORT_SYMBOL_GPL(n64pi_read_word);

int
n64pi_write_word(struct n64pi *pi, uint32_t cart_addr, uint32_t value) {
	int error;

	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(6 << 28);
	n64pi_log_put(cart_addr);
	n64pi_log_put(value);
#endif

	/* Wait until PI is busy ("blocking") */
	block_until_pi_completion(pi);

	/* Make PI steady */
	(void)ensure_noerror(pi, "w_w");

	if ((error = validate_word(pi, "w_w", cart_addr)) != N64PI_ERROR_SUCCESS) {
		return error;
	}

	__raw_writel(value, pi->membase + cart_addr - 0x05000000U); /* TODO hard coding membase offset!! */

	return error;
}
EXPORT_SYMBOL_GPL(n64pi_write_word);

/* TODO how about addressing error? but I don't want to store value into memory... introduce "value_on_error"? */
uint32_t
n64pi_read_word_unsafefast(struct n64pi *pi, uint32_t cart_addr) {
	uint32_t value;

	/* note: logging cause hard time failure! (sdmmc) */
#if 0
	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(11 << 28);
	n64pi_log_put(cart_addr);
#endif
#endif

	/* don't wait, don't reset the error, don't validate. that's unsafe but fast. */

	value = __raw_readl(pi->membase + cart_addr - 0x05000000U); /* TODO hard coding membase offset!! */

#if 0
#ifdef DEBUG_REQLOG
	n64pi_log_put(value);
#endif
#endif

	return value;
}
EXPORT_SYMBOL_GPL(n64pi_read_word_unsafefast);

void
n64pi_write_word_unsafefast(struct n64pi *pi, uint32_t cart_addr, uint32_t value) {
	/* note: logging cause hard time failure! (sdmmc) */
#if 0
	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(12 << 28);
	n64pi_log_put(cart_addr);
	n64pi_log_put(value);
#endif
#endif

	/* don't wait, don't reset the error, don't validate. that's unsafe but fast. */

	__raw_writel(value, pi->membase + cart_addr - 0x05000000U); /* TODO hard coding membase offset!! */
}
EXPORT_SYMBOL_GPL(n64pi_write_word_unsafefast);

void
n64pi_reset(struct n64pi *pi) {
	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(7 << 28);
	n64pi_log_put(0);
	n64pi_log_put(0);
#endif

	__raw_writel(0x00000003, pi->regbase + REG_STATUS);
}
EXPORT_SYMBOL_GPL(n64pi_reset);

uint32_t
n64pi_get_status(struct n64pi *pi) {
	uint32_t status;

	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(8 << 28);
	n64pi_log_put(0);
#endif

	status = __raw_readl(pi->regbase + REG_STATUS);

#ifdef DEBUG_REQLOG
	n64pi_log_put(status);
#endif

	return status;
}
EXPORT_SYMBOL_GPL(n64pi_get_status);

unsigned int
n64pi_ed64_regread_unsafefast(struct n64pi *pi, unsigned int regoff) {
	void __iomem *membase = pi->membase - 0x05000000U + 0x08040000; /* TODO hard coding membase offset!! */
#ifdef ED64_KRIKZZSTYLE
	__raw_readl(membase + 0x00); // dummy read required??
#endif
	return __raw_readl(membase + regoff);
}

void
n64pi_ed64_regwrite_unsafefast(struct n64pi *pi, unsigned int value, unsigned int regoff) {
	void __iomem *membase = pi->membase - 0x05000000U + 0x08040000; /* TODO hard coding membase offset!! */
#ifdef ED64_KRIKZZSTYLE
	__raw_readl(membase + 0x00); // dummy read required?? (only if previous is write, to wait piwrite?)
	__raw_writel(value, membase + regoff);
	/* XXX next pi r/w must be wait iobusy!! */
#else
	__raw_writel(value, membase + regoff);
	while(__raw_readl(pi->regbase + REG_STATUS) & 3) /*nothing*/ ;
#endif
}

static void n64pi_ed64_verify(struct n64pi *pi, unsigned int regoff) {
	ensure_beginned(pi);

	if (pi->ed64_enabled == 0) {
		panic("n64pi sub driver program error: ed64 r/w but not enabled");
	}

	if ((regoff & 3) != 0) {
		panic("n64pi sub driver program error: unaligned ed64 regoff");
	}

	if (0x54 < regoff) {
		panic("n64pi sub driver program error: ed64 regoff out of range");
	}
}

unsigned int
n64pi_ed64_regread(struct n64pi *pi, unsigned int regoff) {
	n64pi_ed64_verify(pi, regoff);

	return n64pi_ed64_regread_unsafefast(pi, regoff);
}

int
n64pi_ed64_regwrite(struct n64pi *pi, unsigned int value, unsigned int regoff) {
	n64pi_ed64_verify(pi, regoff);

	n64pi_ed64_regwrite_unsafefast(pi, value, regoff);

	return 0; /* TODO return error if verify failed */
}

int
n64pi_ed64_enable(struct n64pi *pi) {
	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(9 << 28);
	n64pi_log_put(0);
	n64pi_log_put(pi->ed64_enabled);
#endif

	// TODO INT_MAX check?
	pi->ed64_enabled++;
	if (pi->ed64_enabled == 1) {
		n64pi_ed64_regwrite_unsafefast(pi, 0x1234, 0x20);
		block_until_pi_completion(pi); // ED64 regs seems slow, wait for completion.
	}

	return N64PI_ERROR_SUCCESS;
}
EXPORT_SYMBOL_GPL(n64pi_ed64_enable);

int
n64pi_ed64_disable(struct n64pi *pi) {
	ensure_beginned(pi);

#ifdef DEBUG_REQLOG
	n64pi_log_put(jiffies);
	n64pi_log_put(10 << 28);
	n64pi_log_put(0);
	n64pi_log_put(pi->ed64_enabled);
#endif

	if (pi->ed64_enabled == 1) {
		n64pi_ed64_regwrite_unsafefast(pi, 0, 0x20);
		block_until_pi_completion(pi); // ED64 regs seems slow, wait for completion.
	}
	pi->ed64_enabled--;
	if (pi->ed64_enabled < 0) {
		dev_err(pi->dev, "n64pi sub driver program error: too many ED64 disable request... reset count to 0\n");
		pi->ed64_enabled = 0;
	}

	return N64PI_ERROR_SUCCESS;
}
EXPORT_SYMBOL_GPL(n64pi_ed64_disable);

static irqreturn_t n64pi_isr(int irq, void *dev_id)
{
	struct n64pi *pi = dev_id;
	uint32_t status;

	/* spin_lock pi->lock to use pi->regbase here (and for later memorize). */
	/* TODO may not be required? we are in interrupt context that is intr-masked, no one disturb us... */
	/* TODO uh... spin_lock_irqsave/spin_unlock_irqrestore is bad? thinking it just for paranoid. */
	spin_lock(&pi->lock);

	/* get current PI status before do anything */
	status = __raw_readl(pi->regbase + REG_STATUS);

	/* ack this interrupt (before potential next request in on_complete, not to ack next request... cause infinite wait!) */
	__raw_writel(2, pi->regbase + REG_STATUS); // [1]=clear_intr

	spin_unlock(&pi->lock);

	/* Currently n64pi is blocking op only; interrupt must be never happened. */
	dev_err(pi->dev, "Spurious interrupt: status=%x; resetting.\n", status);

	/* reset PI */
	__raw_writel(0x00000003, pi->regbase + REG_STATUS);

	return IRQ_HANDLED;
}

static const struct mfd_cell n64pi_cells[] = {
	{ .name = "n64pi-cart", .of_compatible = "nintendo,pi-cart", },
	{ .name = "n64pi-ed64tty", .of_compatible = "nintendo,pi-ed64tty" }, // TODO OF? CONFIG?
	{ .name = "n64pi-ed64mmc", .of_compatible = "nintendo,pi-ed64mmc" }, // TODO OF? CONFIG?
	{ .name = "n64pi-ed64i2c", .of_compatible = "nintendo,pi-ed64i2c" }, // TODO OF? CONFIG?
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
	pi->ongoing = 0;
	pi->ed64_enabled = 0;
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
