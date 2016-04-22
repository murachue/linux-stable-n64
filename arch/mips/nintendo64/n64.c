/*
arch/mips/built-in.o: In function `show_cpuinfo':
proc.c:(.text+0xc6d0): undefined reference to `get_system_type'
arch/mips/built-in.o: In function `init_IRQ':
(.init.text+0x2e4): undefined reference to `arch_init_irq'
arch/mips/built-in.o: In function `setup_arch':
(.init.text+0x898): undefined reference to `prom_init'
arch/mips/built-in.o: In function `setup_arch':
(.init.text+0x8bc): undefined reference to `plat_mem_setup'
arch/mips/built-in.o: In function `time_init':
(.init.text+0xf2c): undefined reference to `plat_time_init'
arch/mips/built-in.o: In function `free_initmem':
(.ref.text+0x28): undefined reference to `prom_free_prom_memory'
*/

#include <linux/io.h> //set_io_port_base
#include <linux/of_address.h> //of_address_to_resource
#include <linux/of_fdt.h> //of_scan_flat_dt, __dtb_start
#include <linux/of_irq.h> //of_irq_init
#include <linux/sizes.h> //SZ_1M
#include <asm/addrspace.h> //KSEG1
#include <asm/bootinfo.h> //detect_memory_region
#include <asm/irq_cpu.h> //mips_cpu_irq_of_init
#include <asm/prom.h> //__dt_setup_arch
#include <asm/time.h> //mips_hpt_frequency

// called from arch/mips/kernel/proc.c
const char *get_system_type(void) {
	// TODO: refer dts:model?
	return "Nintendo 64";
}

static void __iomem *rt_intc_membase;

static void ralink_intc_irq_unmask(struct irq_data *d)
{
	__raw_writel(BIT(d->hwirq * 2 + 1), rt_intc_membase + 0x0C); // MI_INTR_MASK_REG
}

static void ralink_intc_irq_mask(struct irq_data *d)
{
	__raw_writel(BIT(d->hwirq * 2 + 0), rt_intc_membase + 0x0C); // MI_INTR_MASK_REG
}

static struct irq_chip ralink_intc_irq_chip = {
	.name		= "N64_MI",
	.irq_unmask	= ralink_intc_irq_unmask,
	.irq_mask	= ralink_intc_irq_mask,
	.irq_mask_ack	= ralink_intc_irq_mask, /* TODO mmm, each device has clear intr ... */
};

static int intc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	irq_set_chip_and_handler(irq, &ralink_intc_irq_chip, handle_level_irq);

	return 0;
}

static const struct irq_domain_ops irq_domain_ops = {
	.xlate = irq_domain_xlate_onecell,
	.map = intc_map,
};

static void ralink_intc_irq_handler(struct irq_desc *desc)
{
	u32 pending = __raw_readl(rt_intc_membase + 0x08); // MI_INTR_REG

	if (pending) {
		struct irq_domain *domain = irq_desc_get_handler_data(desc);
		generic_handle_irq(irq_find_mapping(domain, __ffs(pending)));
	} else {
		spurious_interrupt();
	}
}

static int __init nintendo_mi_of_init(struct device_node *of_node, struct device_node *parent) {
#if 0
	struct irq_domain *domain;

	/* Mask interrupts. */
	*0 = 0xffff00ff;

	domain = irq_domain_add_legacy(of_node, 8, MIPS_CPU_IRQ_BASE, 0, &mips_cpu_intc_irq_domain_ops, NULL);
	if (!domain)
		panic("Failed to add irqdomain for Nintendo 64 MI");
#else
	struct resource res;
	struct irq_domain *domain;
	int irq;

	irq = irq_of_parse_and_map(of_node, 0);
	if (!irq)
		panic("Failed to get INTC IRQ");

	if (of_address_to_resource(of_node, 0, &res))
		panic("Failed to get intc memory range");

	if (request_mem_region(res.start, resource_size(&res),
				res.name) < 0)
		pr_err("Failed to request intc memory");

	rt_intc_membase = ioremap_nocache(res.start,
					resource_size(&res));
	if (!rt_intc_membase)
		panic("Failed to remap intc memory");

	/* disable all interrupts */
	__raw_writel(0x555, rt_intc_membase + 0x0C); // MI_INTR_MASK_REG

	domain = irq_domain_add_legacy(of_node, 6/*count*/, 8/*base*/, 0, &irq_domain_ops, NULL);
	if (!domain)
		panic("Failed to add irqdomain MI intrs");

	irq_set_chained_handler_and_data(irq, ralink_intc_irq_handler, domain);

	/* tell the kernel which irq is used for performance monitoring */
	//rt_perfcount_irq = irq_create_mapping(domain, 9);

	return 0;
#endif
}

static struct of_device_id __initdata of_irq_ids[] = {
	{ .compatible = "mti,cpu-interrupt-controller", .data = mips_cpu_irq_of_init },
	{ .compatible = "nintendo,mi", .data = nintendo_mi_of_init },
	{},
};

void __init arch_init_irq(void)
{
	// we can call irqchip_init() here, but N64 does not need to be such general...
	of_irq_init(of_irq_ids);
}

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

// called from arch/mips/kernel/setup.c
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

void __init plat_mem_setup(void)
{
	set_io_port_base(KSEG1);

	/*
	 * Load the builtin devicetree. This causes the chosen node to be
	 * parsed resulting in our memory appearing
	 * This is NOT the appended dtb!
	 */
	__dt_setup_arch(__dtb_start);

	of_scan_flat_dt(early_init_dt_find_memory, NULL/*unused by callback*/);
	if (memory_dtb)
		of_scan_flat_dt(early_init_dt_scan_memory, NULL/*unused by callback...by spec?*/);
//	else if (soc_info.mem_size)
//		add_memory_region(soc_info.mem_base, soc_info.mem_size * SZ_1M,
//				BOOT_MEM_RAM);
	else
		detect_memory_region(/*mem_base*/0,
				/*size_min*/4 * SZ_1M,
				/*size_max*/8 * SZ_1M);
}

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

void __init prom_free_prom_memory(void)
{
}

#ifdef CONFIG_EARLY_PRINTK
//#define EARLY_UART_BASE		0x007F0000
//static __iomem void *uart_membase = (__iomem void *) KSEG1ADDR(EARLY_UART_BASE);
void prom_putchar(unsigned char ch)
{
	/*
	__raw_writeb(ch, uart_membase);
	uart_membase++;
	/*/
	__raw_writel(ch, (__iomem void *)KSEG1ADDR(0x04400038));
	//*/
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
