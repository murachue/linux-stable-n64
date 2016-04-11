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
#include <linux/of_fdt.h> //of_scan_flat_dt, __dtb_start
#include <linux/of_irq.h> //of_irq_init
#include <linux/sizes.h> //SZ_1M
#include <asm/addrspace.h> //KSEG1
#include <asm/bootinfo.h> //detect_memory_region
#include <asm/irq_cpu.h> //mips_cpu_irq_of_init
#include <asm/prom.h> //__dt_setup_arch

const char *get_system_type(void) {
	// TODO: refer dts:model?
	return "Nintendo 64";
}

static struct of_device_id __initdata of_irq_ids[] = {
	{ .compatible = "mti,cpu-interrupt-controller", .data = mips_cpu_irq_of_init },
//	{ .compatible = "ralink,rt2880-intc", .data = intc_of_init },
	{},
};

void __init arch_init_irq(void)
{
	of_irq_init(of_irq_ids);
}

void __init prom_init(void)
{
	//int argc;
	//char **argv;

	//prom_soc_init(&soc_info);

	//pr_info("SoC Type: %s\n", get_system_type());

	//prom_init_cmdline(argc, argv);
}

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
	 */
	__dt_setup_arch(__dtb_start);

	of_scan_flat_dt(early_init_dt_find_memory, NULL);
	if (memory_dtb)
		of_scan_flat_dt(early_init_dt_scan_memory, NULL);
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
//	clk_put(clk);
//	clocksource_probe();
}

void __init prom_free_prom_memory(void)
{
}

#ifdef CONFIG_EARLY_PRINTK
#define EARLY_UART_BASE		0x007F0000
static __iomem void *uart_membase = (__iomem void *) KSEG1ADDR(EARLY_UART_BASE);
void prom_putchar(unsigned char ch)
{
	/*
	__raw_writeb(ch, uart_membase);
	uart_membase++;
	/*/
	__raw_writeb(ch, KSEG1ADDR(0x04400038));
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
