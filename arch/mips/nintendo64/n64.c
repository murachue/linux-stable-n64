/*
 * Nintendo 64 arch-dependent code
 * Copyright (c) 2016 Murachue <murachue+github@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/io.h> //set_io_port_base
#ifdef CONFIG_USE_OF
#include <linux/of_address.h> //of_address_to_resource
#include <linux/of_fdt.h> //of_scan_flat_dt, __dtb_start
#include <linux/of_irq.h> //of_irq_init
#else
#include <linux/platform_device.h> //platform_device_register
#endif
#include <linux/sizes.h> //SZ_1M
#include <asm/addrspace.h> //KSEG1
#include <asm/bootinfo.h> //detect_memory_region
#include <asm/irq_cpu.h> //mips_cpu_irq_of_init
#include <asm/prom.h> //__dt_setup_arch
#include <asm/time.h> //mips_hpt_frequency

// called from arch/mips/kernel/proc.c:show_cpuinfo, for /proc/cpuinfo
const char *get_system_type(void) {
	// TODO: refer dts:model?
	return "Nintendo 64";
}

static void __iomem *mi_membase;

static void mi_irq_unmask(struct irq_data *d)
{
	__raw_writel(BIT(d->hwirq * 2 + 1), mi_membase + 0x0C); // MI_INTR_MASK_REG, set
}

static void mi_irq_mask(struct irq_data *d)
{
	__raw_writel(BIT(d->hwirq * 2 + 0), mi_membase + 0x0C); // MI_INTR_MASK_REG, clear
}

static struct irq_chip mi_irq_chip = {
	.name		= "n64_mi",
	.irq_unmask	= mi_irq_unmask,
	.irq_mask	= mi_irq_mask,
	.irq_mask_ack	= mi_irq_mask, /* maybe fastpath. if null, irq_mask(mandatory) and irq_ack(optional) are used */
};

static int mi_intc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &mi_irq_chip, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops irq_domain_ops = {
#ifdef CONFIG_USE_OF
	.xlate = irq_domain_xlate_onecell,
	.map = mi_intc_map,
#else
	.map = mi_intc_map,
#endif
};

/* TODO this assumes level-trigger, because this routine does one IRQ at a time.
 * If Nintendo64 MI or MIPS intr does not, enclose with a (possibly while?) loop.
 * On that case, take care of infinite-loop case. (on some driver does not ack intr.) */
static void mi_irq_handler(struct irq_desc *desc)
{
	/* note: at least on Project64, MI_INTR_REG does not masked with MI_INTR_MASK_REG, but we only care unmasked. */
	u32 pending = __raw_readl(mi_membase + 0x08); // MI_INTR_REG
	pending &= __raw_readl(mi_membase + 0x0C); // MI_INTR_MASK_REG

	if (pending) {
		struct irq_domain *domain = irq_desc_get_handler_data(desc);
		generic_handle_irq(irq_find_mapping(domain, __ffs(pending)));
	} else {
		spurious_interrupt();
	}
}

static int __init nintendo_mi_init(struct device_node *of_node, struct resource *pres, int irq) {
	struct irq_domain *domain;

	if (request_mem_region(pres->start, resource_size(pres), pres->name) < 0) {
		pr_err("n64: Failed to request intc memory");
		goto out_reqmemreg;
	}

	mi_membase = ioremap_nocache(pres->start, resource_size(pres));
	if (!mi_membase) {
		pr_err("n64: Failed to remap intc memory");
		goto out_ioremap;
	}

	/* disable all interrupts */
	__raw_writel(0x555, mi_membase + 0x0C); // MI_INTR_MASK_REG

	domain = irq_domain_add_legacy(of_node, 6/*count*/, 8/*base*/, 0/*hwbase*/, &irq_domain_ops, NULL);
	if (!domain) {
		pr_err("n64: Failed to add irqdomain MI intrs");
		goto out_irqdom;
	}

	irq_set_chained_handler_and_data(irq, mi_irq_handler, domain);

	/* tell the kernel which irq is used for performance monitoring */
	//rt_perfcount_irq = irq_create_mapping(domain, 9);

	return 0;
out_irqdom:
	iounmap(mi_membase);
out_ioremap:
	release_mem_region(pres->start, resource_size(pres));
out_reqmemreg:
	return -ENOMEM;
}

#ifdef CONFIG_USE_OF
static int __init nintendo_mi_of_init(struct device_node *of_node, struct device_node *parent) {
	struct resource res;
	int irq;

	irq = irq_of_parse_and_map(of_node, 0);
	if (!irq)
		panic("Failed to get INTC IRQ");

	if (of_address_to_resource(of_node, 0, &res))
		panic("Failed to get intc memory range");

	return nintendo_mi_init(of_node, &res, irq);
}

static struct of_device_id __initdata of_irq_ids[] = {
	{ .compatible = "mti,cpu-interrupt-controller", .data = mips_cpu_irq_of_init },
	{ .compatible = "nintendo,mi", .data = nintendo_mi_of_init },
	{},
};
#endif

#ifndef CONFIG_USE_OF
/*
static struct irqaction cpu_ip2_cascade_action = {
	.handler	= no_action,
	.name		= "cascade_ip2",
	.flags		= IRQF_NO_THREAD,
};
*/
#endif

// called from arch/mips/kernel/irq.c:init_IRQ
void __init arch_init_irq(void)
{
#ifdef CONFIG_USE_OF
	// we can call irqchip_init() here, but N64 does not need to be such general...
	of_irq_init(of_irq_ids);
#else
	mips_cpu_irq_init();
	{
		struct resource res = {
			.name = "irq",
			.flags = IORESOURCE_MEM,
			.start = 0x04300000,
			.end   = 0x0430000F,
		};
		nintendo_mi_init(NULL, &res, 2);
	}
#endif
}

// called from arch/mips/kernel/setup.c:setup_arch
void __init prom_init(void)
{
/*
	int argc;
	char **argv;

	prom_soc_init(&soc_info);

	pr_info("SoC Type: %s\n", get_system_type());

	prom_init_cmdline(argc, argv);
*/
}

#ifdef CONFIG_USE_OF
// called from arch/mips/kernel/setup.c:arch_mem_init
void __init device_tree_init(void)
{
	unflatten_and_copy_device_tree();
}

static int memory_dtb;

static int __init early_init_dt_find_memory(unsigned long node,
		const char *uname, int depth, void *data)
{
	if (depth == 1 && !strcmp(uname, "memory@0"))
		memory_dtb = 1;

	return 0;
}
#endif

// called from arch/mips/kernel/setup.c:setup_arch
void __init plat_mem_setup(void)
{
	set_io_port_base(KSEG1);

#ifdef CONFIG_USE_OF
	/*
	 * Load the builtin devicetree. This causes the chosen node to be
	 * parsed resulting in our memory appearing
	 * This is NOT the appended dtb!
	 */
	__dt_setup_arch(__dtb_start);

	/* TODO see phys 0x0318 to get mem bytes for CICs expect CIC-6105 */

	of_scan_flat_dt(early_init_dt_find_memory, NULL/*unused by callback*/);
	if (memory_dtb)
		of_scan_flat_dt(early_init_dt_scan_memory, NULL/*unused by callback...by spec?*/);
	else
#endif
	{
		/* n64load passes memory size by $a0, stored as fw_arg0 */
		add_memory_region(/*base*/0, /*size*/fw_arg0, BOOT_MEM_RAM);
	}
}

// called from arch/mips/kernel/time.c:time_init
void __init plat_time_init(void)
{
//	struct clk *clk;

//	ralink_of_remap();

//	ralink_clk_init();
//	clk = clk_get_sys("cpu", NULL);
//	if (IS_ERR(clk))
//		panic("unable to get CPU clock, err=%ld", PTR_ERR(clk));
//	pr_info("CPU Clock: %ldMHz\n", clk_get_rate(clk) / 1000000);
//	mips_hpt_frequency = clk_get_rate(clk) / 2;

	/* timer frequency is 1/2 clock rate (93.75MHz/2) */
	mips_hpt_frequency = 93750000/2;

//	clk_put(clk);
	clocksource_probe();
}

// called from arch/mips/mm/init.c:free_initmem
void __init prom_free_prom_memory(void)
{
}

int __init n64_register_devices(void)
{
	{
		/* TODO should assign "name" field to IORES_MEMs for easy identifying in the driver? */
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04000000, .end = 0x04001fff }, /* DMEM, IMEM */
			{ .flags = IORESOURCE_MEM, .start = 0x04040000, .end = 0x0404001f }, /* status regs */
			{ .flags = IORESOURCE_MEM, .start = 0x04080000, .end = 0x04080007 }, /* RSP PC, BIST? */
			{ .flags = IORESOURCE_IRQ, .start = 8+0 /* mi:0 */ },
		};
		static struct platform_device dev = {
			.name          = "n64sp",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
	{
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04100000, .end = 0x0410001f },
			{ .flags = IORESOURCE_IRQ, .start = 8+5 /* mi:5 */ },
		};
		static struct platform_device dev = {
			.name          = "n64dp",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
	{
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04200000, .end = 0x0420000f },
		};
		static struct platform_device dev = {
			.name          = "n64dpspan",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
#if 0
	{
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04300000, .end = 0x0430000f },
			{ .flags = IORESOURCE_IRQ, .start = 2 /* MIPS IRQ 2 */ },
		};
		static struct platform_device dev = {
			.name          = "n64mi",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
#endif
	{
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04400000, .end = 0x04400037 },
			{ .flags = IORESOURCE_IRQ, .start = 8+3 /* mi:3 */ },
		};
		static struct platform_device dev = {
			.name          = "n64vi",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
	{
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04500000, .end = 0x04500017 },
			{ .flags = IORESOURCE_IRQ, .start = 8+2 /* mi:2 */ },
		};
		static struct platform_device dev = {
			.name          = "n64ai",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
	{
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04600000, .end = 0x04600033 },
			{ .flags = IORESOURCE_IRQ, .start = 8+4 /* mi:4 */ },
		};
		static struct platform_device dev = {
			.name          = "n64pi",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
	{
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04700000, .end = 0x0470001f },
		};
		static struct platform_device dev = {
			.name          = "n64ri",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
	{
		static struct resource reses[] = {
			{ .flags = IORESOURCE_MEM, .start = 0x04800000, .end = 0x0480001b },
			{ .flags = IORESOURCE_IRQ, .start = 8+1 /* mi:1 */ },
		};
		static struct platform_device dev = {
			.name          = "n64si",
			.resource      = reses,
			.num_resources = ARRAY_SIZE(reses),
			/*.dev = {
				.platform_data = 0,
			},*/
		};
		platform_device_register(&dev);
	}
	return 0;
}
/* TODO umm... someone prepares arch-depends-device-init function? */
arch_initcall(n64_register_devices);

#ifdef CONFIG_EARLY_PRINTK
//#define EARLY_UART_BASE		0x007F0000
//static __iomem void *uart_membase = (__iomem void *) KSEG1ADDR(EARLY_UART_BASE);
void prom_putchar(unsigned char ch)
{
#if 0
	__raw_writeb(ch, uart_membase);
	uart_membase++;
#elif 0
	__raw_writel(ch, (__iomem void *)KSEG1ADDR(0x04400038));
#elif 1
	unsigned long flags;
	local_irq_save(flags);

	__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
	__raw_writel(0x1234, (__iomem void *)KSEG1ADDR(0x08040020));

	// wait for ED64dma, busyloop...
	for(;;) {
		__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
		if((__raw_readl((__iomem void *)KSEG1ADDR(0x08040004)) & 1) == 0) {
			break;
		}
	}

	// write buffer
	// NOTE a bit lower address for avoid clash with ed64 serial
	__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
	__raw_writel(0x01000000 | ((unsigned)ch << 16), (__iomem void *)KSEG1ADDR(0x13FFF000));
#if 0
	// wipe rest... should be done first time only
	{
		static int wiped = 0;
		if(!wiped) {
			__iomem unsigned long *p;
			for(p = (__iomem unsigned long *)KSEG1ADDR(0x13FFf004); p < (__iomem unsigned long *)KSEG1ADDR(0x13FFf800); p++) {
				__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
				__raw_writel(0, p);
			}
			wiped = 1;
		}
	}
#endif

	__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
	__raw_writel(0x03FFf000/0x800, (__iomem void *)KSEG1ADDR(0x0804000C));
	__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
	__raw_writel(0x0200/0x200-1, (__iomem void *)KSEG1ADDR(0x08040008));
	__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
	__raw_writel(4, (__iomem void *)KSEG1ADDR(0x08040014));

	// wait for ED64dma, busyloop...
	for(;;) {
		__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
		if((__raw_readl((__iomem void *)KSEG1ADDR(0x08040004)) & 1) == 0) {
			break;
		}
	}

	__raw_readl((__iomem void *)KSEG1ADDR(0x08040000));
	__raw_writel(0, (__iomem void *)KSEG1ADDR(0x08040020));

	local_irq_restore(flags);
#endif
}
#endif

#ifdef CONFIG_CPU_HAS_WB
#include <linux/init.h>
#include <asm/bootinfo.h>
#include <asm/wbflush.h>
#include <asm/barrier.h>

static void wbflush_mips(void)
{
	__fast_iob();
}

void (*__wbflush) (void);

void __init wbflush_setup(void)
{
	__wbflush = wbflush_mips;
}

#include <linux/module.h>
EXPORT_SYMBOL(__wbflush);
#endif
