/*
** ed64.c:
**	pseudo serial driver for EverDrive-64 v3 on Nintendo 64.
**
**	(c) Copyright 2018 Murachue <murachue+github@gmail.com>
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** Derived from mux.c: PA-RISC MUX serial driver.
**
*/

/* TODO OpenFirmware DeviceTree support? */
#ifdef CONFIG_USE_OF
#error ed64 does not support device tree yet
#endif

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/console.h>
#include <linux/delay.h> /* for udelay */
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/irq.h>

#if defined(CONFIG_SERIAL_ED64_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#include <linux/sysrq.h>
#define SUPPORT_SYSRQ
#endif

#include <linux/serial_core.h>

#define ED64_POLL_DELAY (30 * HZ / 1000) // 30ms... NOTE HZ<34 is dangerous

static struct uart_port port;
static int enabled;

static struct uart_driver ed64_driver = {
	.owner = THIS_MODULE,
	.driver_name = "ED64 serial",
	.dev_name = "ttyE",
	.major = ED64_MAJOR,//TTY_MAJOR,
	.minor = 0,//64,
	.nr = 1,
};

static struct timer_list ed64_timer;

static void ed64_dummyread(const struct uart_port *port)
{
	__raw_readl(port->membase + 0x00); // dummy read required!!
}

static unsigned int ed64_regread(const struct uart_port *port, unsigned int regoff)
{
	ed64_dummyread(port); // dummy read required!!
	return __raw_readl(port->membase + regoff);
}

static void ed64_regwrite(unsigned int value, const struct uart_port *port, unsigned int regoff)
{
	ed64_dummyread(port); // dummy read required!!
	__raw_writel(value, port->membase + regoff);
}

static void ed64_enable(const struct uart_port *port)
{
	ed64_regwrite(0x1234, port, 0x20);
}

static void ed64_disable(const struct uart_port *port)
{
	ed64_regwrite(0, port, 0x20);
}

/**
 * ed64_tx_empty - Check if the transmitter fifo is empty.
 * @port: Ptr to the uart_port.
 *
 * This function test if the transmitter fifo for the port
 * described by 'port' is empty.  If it is empty, this function
 * should return TIOCSER_TEMT, otherwise return 0.
 */
static unsigned int ed64_tx_empty(struct uart_port *port)
{
	unsigned int status;
	ed64_enable(port);
	status = ed64_regread(port, 0x04);
	ed64_disable(port);
	// bit2: TXE#; if TXE# is high, because FIFO is filled, report "NOT empty", otherwise report "empty".
	return (status & 4) ? 0 : TIOCSER_TEMT;
}

/**
 * ed64_set_mctrl - Set the current state of the modem control inputs.
 * @ports: Ptr to the uart_port.
 * @mctrl: Modem control bits.
 *
 * The ED64 does not support CTS, DCD or DSR so this function
 * is ignored.
 */
static void ed64_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

/**
 * ed64_get_mctrl - Returns the current state of modem control inputs.
 * @port: Ptr to the uart_port.
 *
 * The ED64 does not support CTS, DCD or DSR so these lines are
 * treated as permanently active.
 */
static unsigned int ed64_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

/**
 * ed64_stop_tx - Stop transmitting characters.
 * @port: Ptr to the uart_port.
 *
 * The ED64 does not support this function.
 */
static void ed64_stop_tx(struct uart_port *port)
{
}

/**
 * ed64_start_tx - Start transmitting characters.
 * @port: Ptr to the uart_port.
 *
 * The ED64 does not support this function.
 */
static void ed64_start_tx(struct uart_port *port)
{
}

/**
 * ed64_stop_rx - Stop receiving characters.
 * @port: Ptr to the uart_port.
 *
 * The ED64 does not support this function.
 */
static void ed64_stop_rx(struct uart_port *port)
{
}

/**
 * ed64_break_ctl - Control the transmitssion of a break signal.
 * @port: Ptr to the uart_port.
 * @break_state: Raise/Lower the break signal.
 *
 * The ED64 does not support this function.
 */
static void ed64_break_ctl(struct uart_port *port, int break_state)
{
}

/**
 * ed64_write - Write chars to the ed64 fifo.
 * @port: Ptr to the uart_port.
 *
 * This function writes all the data from the uart buffer to
 * the ed64 fifo.
 */
static void ed64_write(struct uart_port *port)
{
	int count;
	struct circ_buf *xmit = &port->state->xmit;
	char *pbuf = (char*)((uintptr_t)port->private_data | 0xA0000000); // uncached

	// TODO support software flow control?
	/*
	if(port->x_char) { // XON/XOFF
		UART_PUT_CHAR(port, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}
	*/

	if(uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		ed64_stop_tx(port);
		return;
	}

	// TODO avoid reentrant with ed64_read??

	count = port->fifosize;
	*pbuf++ = count & 0xFF;
	do {
		*pbuf++ = xmit->buf[xmit->tail];
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if(uart_circ_empty(xmit))
			break;
	} while(--count > 0);

	{
		unsigned long flags;
		local_irq_save(flags); // disable int to avoid clashing with n64cart... XXX

		// TODO PI arbitrator... do not directly access to MMIO here...

		// wait for PI dma done
		while(__raw_readl((__iomem void *)0xA4600010) & 1) { udelay(1); }
		// transfer dram2cart
		__raw_writel((unsigned)port->private_data, (__iomem void *)0xA4600000); // dramaddr
		__raw_writel(0xB0000000 + 0x04000000 - 0x0800, (__iomem void *)0xA4600004); // cartaddr
		__raw_writel(0x0200, (__iomem void *)0xA4600008); // dram2cart
		// wait for PI dma done
		while(__raw_readl((__iomem void *)0xA4600010) & 1) { udelay(1); }
		// wait ED64 dma
		while(ed64_regread(port, 0x04) & 1) { udelay(1); }
		// transfer cart2usb
		// TODO txe# should be tested
		ed64_regwrite(0, port, 0x08); // dmalen in 512bytes - 1
		ed64_regwrite((0x04000000-0x0800)/0x800, port, 0x0c); // dmaaddr in 2048bytes
		ed64_regwrite(4, port, 0x14); // dmacfg = 4(ram2fifo)
		// wait ED64 dma
		while(ed64_regread(port, 0x04) & 1) { udelay(1); }

		// TODO spurious interrupt on n64cart?

		local_irq_restore(flags);
	}

	if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		ed64_stop_tx(port);
}

/**
 * ed64_read - Read chars from the ed64 fifo.
 * @port: Ptr to the uart_port.
 *
 * This reads all available data from the ed64's fifo and pushes
 * the data to the tty layer.
 */
static void ed64_read(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	int data;
	__u32 start_count = port->icount.rx;

	// TODO avoid reentrant with ed64_write??

	while(1) {
		unsigned int count;
		const unsigned char *pbuf = (const unsigned char*)((uintptr_t)port->private_data | 0xA0000000); // uncached

		// if rxf# is high (=no rx), break.
		if(ed64_regread(port, 0x04) & 8)
			break;

		{
			unsigned long flags;
			local_irq_save(flags); // disable int to avoid clashing with n64cart... XXX

			// TODO PI arbitrator... do not directly access to MMIO here...
			// TODO should 512-bytes block read, and timeout handling.

			// wait ED64 dma
			while(ed64_regread(port, 0x04) & 1) { udelay(1); }
			// transfer usb2cart
			// TODO txe# should be tested
			ed64_regwrite(0, port, 0x08); // dmalen in 512bytes - 1
			ed64_regwrite((0x04000000-0x0800)/0x800, port, 0x0c); // dmaaddr in 2048bytes
			ed64_regwrite(3, port, 0x14); // dmacfg = 3(fifo2ram)
			// wait ED64 dma
			while(ed64_regread(port, 0x04) & 1) { udelay(1); }
			// wait for PI dma done
			while(__raw_readl((__iomem void *)0xA4600010) & 1) { udelay(1); }
			// transfer cart2dram
			__raw_writel((unsigned)port->private_data, (__iomem void *)0xA4600000); // dramaddr
			__raw_writel(0xB0000000 + 0x04000000 - 0x0800, (__iomem void *)0xA4600004); // cartaddr
			__raw_writel(0x0200, (__iomem void *)0xA460000C); // cart2dram
			// wait for PI dma done
			while(__raw_readl((__iomem void *)0xA4600010) & 1) { udelay(1); }

			// TODO spurious interrupt on n64cart?

			local_irq_restore(flags);
		}

		count = *pbuf++;
		if(port->fifosize < count) {
			count = port->fifosize;
		}

		while(count--) {
			data = *pbuf++;
			port->icount.rx++;

			// TODO break support
			/*
			 if (ed64_BREAK(data)) {
			 port->icount.brk++;
			 if(uart_handle_break(port))
			 continue;
			 }
			*/

			if (uart_handle_sysrq_char(port, data & 0xffu))
				continue;

			tty_insert_flip_char(tport, data & 0xFF, TTY_NORMAL);
		}
	}

	if (start_count != port->icount.rx)
		tty_flip_buffer_push(tport);
}

/**
 * ed64_startup - Initialize the port.
 * @port: Ptr to the uart_port.
 *
 * Grab any resources needed for this port and start the
 * ed64 timer.
 */
static int ed64_startup(struct uart_port *port)
{
	enabled = 1;
	return 0;
}

/**
 * ed64_shutdown - Disable the port.
 * @port: Ptr to the uart_port.
 *
 * Release any resources needed for the port.
 */
static void ed64_shutdown(struct uart_port *port)
{
	enabled = 0;
}

/**
 * ed64_set_termios - Chane port parameters.
 * @port: Ptr to the uart_port.
 * @termios: new termios settings.
 * @old: old termios settings.
 *
 * The Serial ed64 does not support this function.
 */
static void
ed64_set_termios(struct uart_port *port, struct ktermios *termios,
	        struct ktermios *old)
{
}

/**
 * ed64_type - Describe the port.
 * @port: Ptr to the uart_port.
 *
 * Return a pointer to a string constant describing the
 * specified port.
 */
static const char *ed64_type(struct uart_port *port)
{
	return "ed64";
}

/**
 * ed64_release_port - Release memory and IO regions.
 * @port: Ptr to the uart_port.
 * 
 * Release any memory and IO region resources currently in use by
 * the port.
 */
static void ed64_release_port(struct uart_port *port)
{
}

/**
 * ed64_request_port - Request memory and IO regions.
 * @port: Ptr to the uart_port.
 *
 * Request any memory and IO region resources required by the port.
 * If any fail, no resources should be registered when this function
 * returns, and it should return -EBUSY on failure.
 */
static int ed64_request_port(struct uart_port *port)
{
	return 0;
}

/**
 * ed64_config_port - Perform port autoconfiguration.
 * @port: Ptr to the uart_port.
 * @type: Bitmask of required configurations.
 *
 * Perform any autoconfiguration steps for the port.  This function is
 * called if the UPF_BOOT_AUTOCONF flag is specified for the port.
 * [Note: This is required for now because of a bug in the Serial core.
 *  rmk has already submitted a patch to linus, should be available for
 *  2.5.47.]
 */
static void ed64_config_port(struct uart_port *port, int type)
{
	port->type = PORT_MUX;
}

/**
 * ed64_verify_port - Verify the port information.
 * @port: Ptr to the uart_port.
 * @ser: Ptr to the serial information.
 *
 * Verify the new serial port information contained within serinfo is
 * suitable for this port type.
 */
static int ed64_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if(port->membase == NULL)
		return -EINVAL;

	return 0;
}

/**
 * ed64_drv_poll - ed64 poll function.
 * @unused: Unused variable
 *
 * This function periodically polls the Serial ed64 to check for new data.
 */
static void ed64_poll(unsigned long unused)
{
	if(enabled) {
		ed64_enable(&port);
		ed64_read(&port);
		ed64_write(&port);
		ed64_disable(&port);
	}

	// schedule next polling
	mod_timer(&ed64_timer, jiffies + ED64_POLL_DELAY);
}

#ifdef CONFIG_SERIAL_ED64_CONSOLE
static void ed64_console_write(struct console *co, const char *_s, unsigned count)
{
	// TODO avoid reentrant with ed64_read/ed64_write??

	const unsigned char *s = _s;
	unsigned long flags;
	local_irq_save(flags); // disable int to avoid clashing with n64cart... XXX

	// TODO PI arbitrator... do not directly access to MMIO here...

	// wait for PI dma done
	while(__raw_readl((__iomem void *)0xA4600010) & 1) { udelay(1); }

	ed64_enable(&port);

	// Nintendo64 PI requires 32bit access.
	// supposed big-endian.
	while(0 < count) {
		unsigned cnt = count < 255 ? count : 255; // min(count, 255)

		unsigned buf = cnt << 0;
		unsigned i = 1; // 1 byte (=cnt) already buffered
		while(i < 256) {
			buf <<= 8;
			if(0 < cnt) {
				buf |= *s++;
				count--;
				cnt--;
			}
			i++;
			if(i % 4 == 0) {
				ed64_dummyread(&port); // dummy read required!!
				__raw_writel(buf, (__iomem unsigned *)(0xB4000000 - 0x0800 + i - 4));
				buf = 0;
				if(cnt == 0) {
					break;
				}
			}
		}

		// wait ED64 dma
		while(ed64_regread(&port, 0x04) & 1) { udelay(1); }
		// transfer cart2usb
		// TODO txe# should be tested
		ed64_regwrite(0, &port, 0x08); // dmalen in 512bytes - 1
		ed64_regwrite((0x04000000-0x0800)/0x800, &port, 0x0c); // dmaaddr in 2048bytes
		ed64_regwrite(4, &port, 0x14); // dmacfg = 4(ram2fifo)
		// wait ED64 dma
		while(ed64_regread(&port, 0x04) & 1) { udelay(1); }
	}

	ed64_disable(&port);

	// TODO spurious interrupt on n64cart?

	local_irq_restore(flags);
}

static int ed64_console_setup(struct console *co, char *options)
{
	return 0;
}

static struct console ed64_console = {
	.name =		"ttyE",
	.write =	ed64_console_write,
	.device =	uart_console_device,
	.setup =	ed64_console_setup,
	.flags =	CON_ENABLED | CON_PRINTBUFFER,
	.index =	0,
	.data =		&ed64_driver,
};

#define ED64_CONSOLE	&ed64_console
#else
#define ED64_CONSOLE	NULL
#endif

static struct uart_ops ed64_pops = {
	.tx_empty =		ed64_tx_empty,
	.set_mctrl =		ed64_set_mctrl,
	.get_mctrl =		ed64_get_mctrl,
	.stop_tx =		ed64_stop_tx,
	.start_tx =		ed64_start_tx,
	.stop_rx =		ed64_stop_rx,
	.break_ctl =		ed64_break_ctl,
	.startup =		ed64_startup,
	.shutdown =		ed64_shutdown,
	.set_termios =		ed64_set_termios,
	.type =			ed64_type,
	.release_port =		ed64_release_port,
	.request_port =		ed64_request_port,
	.config_port =		ed64_config_port,
	.verify_port =		ed64_verify_port,
};

/**
 * ed64_init - Serial ED64 initialization procedure.
 *
 * Register the Serial ED64 driver.
 */
static int __init ed64_init(void)
{
	int status;

	printk(KERN_INFO "Serial: ED64 pseudo serial driver revision 1\n");

	//dev_set_drvdata(&dev->dev, (void *)(long)port_count);
	if(request_mem_region(0x08040000, 0x40, "ed64") == NULL) {
		printk(KERN_ERR "Serial ED64: Unable to request ED64 regs mem region.\n");
		return 1;
	}

	/* register the driver */
	ed64_driver.cons = ED64_CONSOLE;

	status = uart_register_driver(&ed64_driver);
	if(status) {
		printk(KERN_ERR "Serial ED64: Unable to register driver.\n");
		return 1;
	}

	/* register a port in driver */
	port.iobase = 0; // no I/O port
	port.mapbase = 0x08040000;
	port.membase = ioremap_nocache(port.mapbase, 0x50); // 0x30 but something follows
	port.iotype = UPIO_MEM32BE;
	port.type = PORT_MUX;
	port.irq = 0; // no IRQ
	port.uartclk = 0; // ? mux is also 0
	port.fifosize = 255;
	port.ops = &ed64_pops;
	port.flags = UPF_BOOT_AUTOCONF;
	port.line = 0; // port 0
	port.private_data = kmalloc(256+8, GFP_KERNEL); // TODO 8byte align required

	/* The port->timeout needs to match what is present in
	 * uart_wait_until_sent in serial_core.c.  Otherwise
	 * the time spent in msleep_interruptable will be very
	 * long, causing the appearance of a console hang.
	 */
	port.timeout = HZ / 50; // 20ms
	spin_lock_init(&port.lock);

	status = uart_add_one_port(&ed64_driver, &port);
	BUG_ON(status);

	/* Start the ED64 polling timer */
	init_timer(&ed64_timer);
	ed64_timer.function = ed64_poll;
	mod_timer(&ed64_timer, jiffies + ED64_POLL_DELAY);

#ifdef CONFIG_SERIAL_ED64_CONSOLE
	register_console(&ed64_console);
#endif

	return 0;
}

/**
 * ed64_exit - Serial ed64 cleanup procedure.
 *
 * Unregister the Serial ed64 driver from the tty layer.
 */
static void __exit ed64_exit(void)
{
	//int port_count = (long)dev_get_drvdata(&dev->dev);

	/* Delete the ED64 polling timer. */
	del_timer_sync(&ed64_timer);

#ifdef CONFIG_SERIAL_ED64_CONSOLE
	unregister_console(&ed64_console);
#endif

	/* Release the resources associated with the port on the device. */
	uart_remove_one_port(&ed64_driver, &port);
	if(port.private_data)
		kfree(port.private_data);
	if(port.membase)
		iounmap(port.membase);

	uart_unregister_driver(&ed64_driver);
	release_mem_region(0x08040000, 0x40);
}

module_init(ed64_init);
module_exit(ed64_exit);

MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
MODULE_DESCRIPTION("Everdrive-64 v3 pseudo serial driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(ED64_MAJOR);
