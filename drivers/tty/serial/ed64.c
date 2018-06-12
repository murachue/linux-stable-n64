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

//#define DEBUG

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

/* SERIAL_ED64_CONSOLE is required, console is write only but specified tty on console= becomes special, accepts BREAK as SysRQ. */
#if defined(CONFIG_SERIAL_ED64_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#include <linux/sysrq.h>
#define SUPPORT_SYSRQ /* enables uart_handle_sysrq_char in serial_core.h */
#endif

#include <linux/serial_core.h>

#include <linux/mfd/n64pi.h>

#define ED64_POLL_DELAY (HZ * 100 / 1000) // 100ms

enum ed64_poll_state {
	POLL_NONE, // just tty rx/tx request
	POLL_READING, // waiting for finishing ED64 USB-to-ROM DMA (rx)
	POLL_WRITEDMA, // waiting for finishing RAM-to-Cart PI DMA (tx)
	POLL_PIDMA, // waiting for finishing Cart/RAM PI DMA
};

struct ed64_private {
	struct n64pi *pi; // n64pi arbitrator bound to the parent mfd device
	enum ed64_poll_state poll_state; // poll (frequentry) for what
	uint32_t cartbase; // ROM area that is used for rx/tx (2048 bytes align required by ED64 DMA!)
	uint32_t status; // last polled status
	unsigned char __attribute((aligned(8))) rxbuf[256]; // 255-bytes receive buffer, NOTE: must be 8 bytes aligned (required by PI DMA)
};

static struct uart_port port;
static int enabled;

static struct uart_driver ed64_uart = {
	.owner = THIS_MODULE,
	.driver_name = "ED64 serial",
	.dev_name = "ttyE",
	.major = ED64_MAJOR,//TTY_MAJOR,
	.minor = 0,//64,
	.nr = 1,
};

static struct timer_list ed64_timer;

/* TODO merge with n64cart */
static int ed64_dummyread(struct list_head *list)
{
	struct n64pi_request *req;

	req = n64pi_alloc_request(GFP_KERNEL);
	if (!req) {
		pr_err("%s: could not allocate n64pi_request\n", __func__);
		return 0;
	}
	req->type = N64PI_RTY_C2R_WORD;
	req->cart_address = 0x08040000 + 0x00; // dummy read REG_CFG required!!
	req->on_complete = n64pi_free_request;
	req->on_error = n64pi_free_request;
	//req->cookie = NULL;
	list_add_tail(&req->node, list);
	return 1;
}

static unsigned int ed64_regread(unsigned int regoff, void (*on_complete)(struct n64pi_request *req), void *cookie, struct list_head *list)
{
	struct n64pi_request *req;

	if (!ed64_dummyread(list)) { // dummy read required!!
		return 0;
	}

	req = n64pi_alloc_request(GFP_KERNEL);
	if (!req) {
		pr_err("%s: could not allocate n64pi_request\n", __func__);

		{
			struct n64pi_request *pireq_dummyread;
			pireq_dummyread = list_last_entry(list, struct n64pi_request, node);
			list_del(&pireq_dummyread->node);
			kfree(pireq_dummyread);
		}

		return 0;
	}
	req->type = N64PI_RTY_C2R_WORD;
	req->cart_address = 0x08040000 + regoff;
	req->on_complete = on_complete;
	req->on_error = n64pi_free_request;
	req->cookie = cookie;
	list_add_tail(&req->node, list);
	return 1;
}

static int ed64_regwrite(uint32_t value, unsigned int regoff, struct list_head *list)
{
	struct n64pi_request *req;

	if (!ed64_dummyread(list)) { // dummy read required!!
		return 0;
	}

	req = n64pi_alloc_request(GFP_KERNEL);
	if (!req) {
		pr_err("%s: could not allocate n64pi_request\n", __func__);

		{
			struct n64pi_request *pireq_dummyread;
			pireq_dummyread = list_last_entry(list, struct n64pi_request, node);
			list_del(&pireq_dummyread->node);
			kfree(pireq_dummyread);
		}

		return 0;
	}
	req->type = N64PI_RTY_R2C_WORD;
	req->cart_address = 0x08040000 + regoff;
	req->value = value;
	req->on_complete = n64pi_free_request;
	req->on_error = n64pi_free_request;
	//req->cookie = NULL;
	list_add_tail(&req->node, list);
	return 1;
}

static int ed64_enable(struct list_head *list)
{
	return ed64_regwrite(0x1234, 0x20, list);
}

static int ed64_disable(struct list_head *list)
{
	return ed64_regwrite(0, 0x20, list);
}

/**
 * ed64_tx_empty - Check if the transmitter fifo is empty.
 * @port: Ptr to the uart_port.
 *
 * This function test if the transmitter fifo for the port
 * described by 'port' is empty.  If it is empty, this function
 * should return TIOCSER_TEMT, otherwise return 0.
 */
static unsigned int ed64_tx_empty(struct uart_port *_port)
{
	struct list_head *reqs;
	if (!ed64_enable(reqs)) {
		return 0; /* TODO what should return on error? */
	}
	if (!ed64_regread(0x04, FUNCTION, reqs)) {
		/* TODO free reqs */
		return 0; /* TODO what should return on error? */
	}
	if (!ed64_disable(reqs)) {
		/* TODO free reqs */
		return 0; /* TODO what should return on error? */
	}
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

static void ed64_on_status_complete(struct n64pi_request *req);

/**
 * ed64_write - Write chars to the ed64 fifo.
 * @port: Ptr to the uart_port.
 *
 * This function writes all the data from the uart buffer to
 * the ed64 fifo.
 * Do not use PI DMA for spurious interrupt on n64cart driver.
 */
static void ed64_write(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;

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

	{
		unsigned long flags;
		local_irq_save(flags); // disable int to avoid clashing with n64cart... XXX

		// TODO PI arbitrator... do not directly access to MMIO here...

		// wait for PI dma done (maybe conflict with CPU R/W)
		while(__raw_readl((__iomem void *)0xA4600010) & 3) { udelay(1); } // XXX argh!! touching non-requested iomem!!

		ed64_enable(port);

		// wait ED64 dma
		while(ed64_regread(port, 0x04) & 1) { udelay(1); }

		// write into cart
		{
			unsigned int _count = uart_circ_chars_pending(xmit);
			unsigned int count = (_count < port->fifosize) ? _count : port->fifosize; // min(count, port->fifosize)
			unsigned int buf = count & 0xFF; // pre-filled by length field
			int i = 1; // 1 byte already filled
			while(i < 1 + port->fifosize) {
				buf <<= 8;
				if(!uart_circ_empty(xmit)) {
					buf |= (unsigned char)xmit->buf[xmit->tail];
					xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
					port->icount.tx++;
				}
				i++;
				if(i % 4 == 0) {
					ed64_dummyread(port); // dummy read required!!
					__raw_writel(buf, (__iomem unsigned *)(0xB4000000 - 0x0800 + i - 4)); // XXX argh!! touching non-requested iomem!!
					buf = 0;
					if(uart_circ_empty(xmit)) {
						break;
					}
				}
			}
		}

		// transfer cart2usb
		// TODO txe# should be tested
		ed64_regwrite(0, port, 0x08); // dmalen in 512bytes - 1
		ed64_regwrite((0x04000000-0x0800)/0x800, port, 0x0c); // dmaaddr in 2048bytes
		ed64_regwrite(4, port, 0x14); // dmacfg = 4(ram2fifo)

		// wait ED64 dma
		while(ed64_regread(port, 0x04) & 1) { udelay(1); }

		ed64_disable(port);

		local_irq_restore(flags);
	}

	if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		ed64_stop_tx(port);
}

static void ed64_on_readdma_complete(struct n64pi_request *req)
{
	struct uart_port *port = (struct uart_port *)req->cookie;
	struct ed64_private *ed64 = (struct ed64_private *)port->private_data;
	uint32_t pistatus = req->status;

	kfree(req);

	// first, make poll for nothing anymore. no retry on failure. XXX do retry
	ed64->poll_state = POLL_NONE;

	if ((pistatus & 7) != 0) {
		pr_err("%s: read ED64 rx buffer failed status=%x\n", __func__, pistatus);
		// will be retried by next timer
		return;
	}

	{
		struct tty_port *ttyport = &port->state->port;
		int data;
		__u32 start_count = port->icount.rx;
		const unsigned char *pbuf = ed64->rxbuf;
		unsigned int count;

#ifdef DEBUG
		pr_info("ed64: got rx: %02x%02x %02x%02x %02x%02x %02x%02x\n"
						,rbuf[0]
						,rbuf[1]
						,rbuf[2]
						,rbuf[3]
						,rbuf[4]
						,rbuf[5]
						,rbuf[6]
						,rbuf[7]
					 );
#endif

		count = *pbuf++;
		/*
			 if(port->fifosize < count) {
			 count = port->fifosize;
			 }
			 */

		if(count == 0) {
#ifdef DEBUG
			pr_info("ed64: got break\n");
#endif
			port->icount.brk++;
			uart_handle_break(port); // note: return 1 if sysrq prefix
		} else {
#ifdef DEBUG
			pr_info("ed64: got %d chars\n", count);
#endif
			do {
				data = *pbuf++;
				port->icount.rx++;

				if (uart_handle_sysrq_char(port, data & 0xffu))
					continue;

				tty_insert_flip_char(ttyport, data & 0xFF, TTY_NORMAL);
			} while(--count);
		}

		if (start_count != port->icount.rx)
			tty_flip_buffer_push(ttyport);
	}
}

static void ed64_on_readpoll_complete(struct n64pi_request *req)
{
	struct uart_port *port = (struct uart_port *)req->cookie;
	struct ed64_private *ed64 = (struct ed64_private *)port->private_data;
	uint32_t pistatus = req->status;
	uint32_t ed64status = req->value;

	kfree(req);

	if ((pistatus & 7) != 0) {
		pr_err("%s: read ED64 status failed status=%x\n", __func__, pistatus);
		// will be retried by next timer
		return;
	}

	ed64->status = ed64status;

	if (ed64status & 1) {
		// ED64 DMA is still running, poll that later (at timer handler).
		return;
	}

	// ED64 DMA is done.
	// Get ED64 ROM rx buffer

	// we are trying to DMA from cart to ram...
	ed64->poll_state = POLL_PIDMA;

	// note: no need to ed64_enable for cart read
	{
		struct n64pi_request *pireq;

		pireq = n64pi_alloc_request(GFP_KERNEL);
		if (!pireq) {
			pr_err("%s: can't allocate n64pi_request for read rx\n", __func__, req->status);
			// oops... give up rx. recover at better.
			ed64->poll_state = POLL_NONE;
			return;
		}

		pireq->type = N64PI_RTY_C2R_DMA;
		pireq->cart_address = ed64->cartbase;
		pireq->ram_vaddress = ed64->rxbuf;
		pireq->length = 256;
		pireq->on_complete = ed64_on_readdma_complete;
		pireq->on_error = n64pi_free_request; // TODO poll_state to POLL_NONE is required
		pireq->cookie = port;
		n64pi_request_async(pi, pireq);
	}
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
	// TODO avoid reentrant with ed64_write??
	// TODO should timeout handling.

	struct ed64_private *ed64 = (struct ed64_private *)port->private_data;
	struct list_head reqs;
	struct n64pi_request *pireq;

	// we are trying to USB-to-Cart DMA, be "polling for read"
	ed64->poll_state = POLL_READING;

	// let's rock.
	INIT_LIST_HEAD(&reqs);
	if (!ed64_enable(&reqs)) {
		return 0;
	}
	if (!ed64_regwrite(512/512 - 1, 0x08, &reqs)) { // dmalen in 512bytes - 1
		return 0;
	}
	if (!ed64_regwrite(port->cartbase / 2048, 0x0c, &reqs)) { // dmaaddr in 2048bytes
		return 0;
	}
	if (!ed64_regwrite(3, 0x14, &reqs)) { // dmacfg = 3(fifo2ram)
		return 0;
	}
	if (!ed64_regread(0x04, ed64_on_readpoll_complete, port, &reqs)) {
		return 0;
	}
	if (!ed64_disable(&reqs)) {
		return 0;
	}
	n64pi_many_request_async(pi, &reqs);
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

static void ed64_on_status_complete(struct n64pi_request *req)
{
	struct uart_port *port = (struct uart_port *)req->cookie;
	struct ed64_private *ed64 = (struct ed64_private *)port->private_data;
	uint32_t pistatus = req->status;
	uint32_t ed64status = req->value;

	kfree(req);

	if ((pistatus & 7) != 0) {
		pr_err("%s: read ED64 status failed status=%x\n", __func__, pistatus);
		return;
	}

	ed64->status = ed64status;

	// if DMABUSY is low (no ED64 DMA is running) and rxf# is low (=have rx), do read.
	if(!(ed64status & 9)) {
		ed64_read(&port);
	} else {
		// calling ed64_read cause ED64 (USB) DMA... put tx in "else" block.

		// if DMABUSY is low (no ED64 DMA is running) and txe# is low (=can tx), do write.
		// almost call will do nothing because port have no enqueued chars.
		if(!(ed64status & 5)) {
			ed64_write(&port);
		}
	}
}

static void ed64_status(struct uart_port *port)
{
	struct ed64_private *ed64 = (struct ed64_private *)port->private_data;
	struct list_head reqs;
	struct n64pi_request *pireq;

	INIT_LIST_HEAD(&reqs);
	if (!ed64_enable(&reqs)) {
		return 0;
	}
	if (!ed64_regread(0x04, ed64_on_status_complete, port, &reqs)) {
		return 0;
	}
	if (!ed64_disable(&reqs)) {
		return 0;
	}

	n64pi_many_request_async(pi, &reqs);
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
		ed64_status(&port);
	}

	// schedule next polling
	mod_timer(&ed64_timer, jiffies + ED64_POLL_DELAY);
}

#ifdef CONFIG_CONSOLE_POLL
/* TODO poll_init called on early, not just before starting poll... can console_poll live with ordinal tty!? */
static int ed64_poll_init(struct uart_port *port)
{
	unsigned char *rbuf = ((struct ed64_private *)port->private_data)->rxbuf;

	rbuf[0] = 0;

	return 0;
}

// reduced ed64_console_write
static void ed64_poll_put_char(struct uart_port *port, unsigned char ch)
{
	unsigned char sa[1] = {ch};
	unsigned char *s = sa;

	// poll console is special, do not use n64pi arbitrator, using raw mmio.

	// wait for PI dma done (maybe conflict with CPU R/W)
	while(__raw_readl((__iomem void *)0xA4600010) & 3) { udelay(1); } // XXX argh!! touching non-requested iomem!!

	ed64_enable(port);

	// Nintendo64 PI requires 32bit access.
	// supposed big-endian.
	{
		unsigned cnt = 1;

		unsigned buf = cnt << 0;
		unsigned i = 1; // 1 byte (=cnt) already buffered
		while(i < 256) {
			buf <<= 8;
			if(0 < cnt) {
				buf |= *s++;
				cnt--;
			}
			i++;
			if(i % 4 == 0) {
				ed64_dummyread(port); // dummy read required!!
				__raw_writel(buf, (__iomem unsigned *)(0xB4000000 - 0x0800 + i - 4)); // XXX argh!! touching non-requested iomem!!
				buf = 0;
				if(cnt == 0) {
					break;
				}
			}
		}

		// wait ED64 dma
		while(ed64_regread(port, 0x04) & 1) { udelay(1); }
		// transfer cart2usb
		// TODO txe# should be tested
		ed64_regwrite(0, port, 0x08); // dmalen in 512bytes - 1
		ed64_regwrite((0x04000000-0x0800)/0x800, port, 0x0c); // dmaaddr in 2048bytes
		ed64_regwrite(4, port, 0x14); // dmacfg = 4(ram2fifo)
		// wait ED64 dma
		while(ed64_regread(port, 0x04) & 1) { udelay(1); }
	}

	ed64_disable(port);
}

static int poll_get_char_from_buf(unsigned char *rbuf)
{
	unsigned int i, c;
	unsigned char r;

	if(rbuf[0] == 0) {
		return NO_POLL_CHAR;
	}

	// pick a char from buffer (before pull tail up)
	r	= rbuf[1];

	// pull tail up
	c = (unsigned int)rbuf[0];
	for(i = 1; i < c; i++) {
		rbuf[i] = rbuf[i + 1];
	}

	// decr nchars
	rbuf[0]--;

	return (int)(unsigned int)r;
}

static int ed64_poll_get_char(struct uart_port *port)
{
	int data;
	// TODO can I trust private_data on panic?
	unsigned char *rbuf = ((struct ed64_private *)port->private_data)->rxbuf;

	// try get from buf
	if((data = poll_get_char_from_buf(rbuf)) != NO_POLL_CHAR) {
		return data;
	}

	// buf is empty. try to get new chars.
	{
		// poll console is special, do not use n64pi arbitrator, using raw mmio.
		// TODO should timeout handling.

		// wait for PI dma done (maybe conflict with CPU R/W)
		while(__raw_readl((__iomem void *)0xA4600010) & 3) { udelay(1); } // XXX argh!! touching non-requested iomem!!

		ed64_enable(port);

		// if rxf# is high (=no rx), bye.
		if(ed64_regread(port, 0x04) & 8) {
			ed64_disable(port);
			return NO_POLL_CHAR;
		}

		// wait ED64 dma
		while(ed64_regread(port, 0x04) & 1) { udelay(1); }

		// transfer usb2cart
		// TODO timeout should be handled
		ed64_regwrite(0, port, 0x08); // dmalen in 512bytes - 1
		ed64_regwrite((0x04000000-0x0800)/0x800, port, 0x0c); // dmaaddr in 2048bytes
		ed64_regwrite(3, port, 0x14); // dmacfg = 3(fifo2ram)

		// wait ED64 dma
		while(ed64_regread(port, 0x04) & 1) { udelay(1); }

		// here, ed64_disable can be called... but later.

		// transfer cart2dram
		// Must be transferred to memory... to process rx data in interruptible context,
		// that don't be bothered with other PI access (or tty processing).
		// NOTE can't use PI DMA... spurious interrupt on n64cart.
		{
			unsigned int i;
			for(i = 0; i < 256; i += 4) {
				*((unsigned int *)(rbuf + i)) = __raw_readl((__iomem void *)(0xB4000000 - 0x0800 + i)); // XXX argh!! touching non-requested iomem!!
			}
		}

		ed64_disable(port);
	}

#ifdef DEBUG
	pr_info("ed64: got poll rx: %02x%02x %02x%02x %02x%02x %02x%02x\n"
			,rbuf[0]
			,rbuf[1]
			,rbuf[2]
			,rbuf[3]
			,rbuf[4]
			,rbuf[5]
			,rbuf[6]
			,rbuf[7]
			);
#endif

	return poll_get_char_from_buf(rbuf);
}
#endif

#ifdef CONFIG_SERIAL_ED64_CONSOLE
static void ed64_console_write(struct console *co, const char *_s, unsigned count)
{
	// TODO avoid reentrant with ed64_read/ed64_write??

	const unsigned char *s = _s;
	unsigned long flags;
	local_irq_save(flags); // disable int to avoid clashing with n64cart... XXX

	// TODO PI arbitrator... do not directly access to MMIO here...

	// wait for PI dma done (maybe conflict with CPU R/W)
	while(__raw_readl((__iomem void *)0xA4600010) & 3) { udelay(1); } // XXX argh!! touching non-requested iomem!!

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
				__raw_writel(buf, (__iomem unsigned *)(0xB4000000 - 0x0800 + i - 4)); // XXX argh!! touching non-requested iomem!!
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
		ed64_regwrite(512/512 - 1, &port, 0x08); // dmalen in 512bytes - 1
		ed64_regwrite((0x04000000-0x0800)/0x800, &port, 0x0c); // dmaaddr in 2048bytes TODO configurable but there are no "*port"
		ed64_regwrite(4, &port, 0x14); // dmacfg = 4(ram2fifo)
		// wait ED64 dma
		while(ed64_regread(&port, 0x04) & 1) { udelay(1); }
	}

	ed64_disable(&port);

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
	.flags =	CON_ENABLED /*| CON_PRINTBUFFER*/, /* No PRINTBUFFER, don't get dmesg twice on boot. (earlyprintk does it first) */
	.index =	0,
	.data =		&ed64_uart, /* used by uart_console_device */
};

#define ED64_CONSOLE	&ed64_console
#else
#define ED64_CONSOLE	NULL
#endif

static struct uart_ops ed64_ops = {
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
#ifdef CONFIG_CONSOLE_POLL
	.poll_init = ed64_poll_init,
	.poll_put_char = ed64_poll_put_char,
	.poll_get_char = ed64_poll_get_char,
#endif
};

/**
 * ed64_probe - Serial ED64 initialization procedure.
 *
 * Register the Serial ED64 driver.
 */
static int ed64_probe(struct platform_device *pdev)
{
	int status;
	struct ed64_private *ed64;

	//dev_set_drvdata(&dev->dev, (void *)(long)port_count);
	if(request_mem_region(0x08040000, 0x40, "ed64") == NULL) {
		printk(KERN_ERR "Serial ED64: Unable to request ED64 regs mem region.\n");
		return 1;
	}

	/* register the driver */
	ed64_uart.cons = ED64_CONSOLE;

	status = uart_register_driver(&ed64_uart);
	if(status) {
		printk(KERN_ERR "Serial ED64: Unable to register driver.\n");
		return 1;
	}

	/* initialize driver-internal data */
	ed64 = kmalloc(sizeof(struct ed64_private), GFP_KERNEL);
	ed64->pi = dev_get_drvdata(pdev->dev.parent); /* TODO dev.parent->parent is n64pi?? ipaq-micro-leds */
	ed64->poll_state = POLL_NONE;
	ed64->cartbase = (0x04000000-0x0800); // TODO configurable or IORESOURCE_MEM/IO?
	ed64->status = 0;
	ed64->rxbuf[0] = 0;

	/* register a port in driver */
	port.iobase = 0; // no I/O port
	port.mapbase = 0; // MMIO is handled by n64pi
	port.membase = 0; // ditto
	port.iotype = UPIO_MEM32BE;
	port.type = PORT_ED64;
	port.irq = 0; // no IRQ (polling...)
	port.uartclk = 0; // ? mux is also 0
	port.fifosize = 255;
	port.ops = &ed64_ops;
	port.flags = UPF_BOOT_AUTOCONF;
	port.line = 0; // port 0
	port.private_data = ed64;

	/* The port->timeout needs to match what is present in
	 * uart_wait_until_sent in serial_core.c.  Otherwise
	 * the time spent in msleep_interruptable will be very
	 * long, causing the appearance of a console hang.
	 */
	port.timeout = HZ / 50; // 20ms
	spin_lock_init(&port.lock);

	status = uart_add_one_port(&ed64_uart, &port);
	BUG_ON(status);

	/* Start the ED64 polling timer */
	init_timer(&ed64_timer);
	ed64_timer.function = ed64_poll;
	mod_timer(&ed64_timer, jiffies + ED64_POLL_DELAY);

#ifdef CONFIG_SERIAL_ED64_CONSOLE
	/* do this after ed64_uart.tty_driver has initialized (console.device() = uart_console_device uses that) */
	register_console(&ed64_console);
#endif

	return 0;
}

/**
 * ed64_remove - Serial ed64 cleanup procedure.
 *
 * Unregister the Serial ed64 driver from the tty layer.
 */
static int ed64_remove(struct platform_device *pdev)
{
	//int port_count = (long)dev_get_drvdata(&dev->dev);

	/* Delete the ED64 polling timer. */
	del_timer_sync(&ed64_timer);

#ifdef CONFIG_SERIAL_ED64_CONSOLE
	unregister_console(&ed64_console);
#endif

	/* Release the resources associated with the port on the device. */
	uart_remove_one_port(&ed64_uart, &port);
	if(port.private_data)
		kfree(port.private_data);

	uart_unregister_driver(&ed64_uart);
}

static struct platform_driver ed64_driver = {
	.probe  = ed64_probe,
	.remove = ed64_remove,
	.driver = {
	    .name = "n64pi-ed64tty",
	},
};

static int __init ed64_init(void)
{
	int error;

	printk(KERN_INFO "Serial: ED64 pseudo serial driver revision 1\n");

	error = platform_driver_register(&ed64_driver);
	if (error) {
		pr_err("%s:%u: could not register platform driver\n", __func__, __LINE__);
		return error;
	}

	return 0;
}

static void __exit ed64_exit(void)
{
	platform_driver_unregister(&ed64_driver);
}

module_init(ed64_init);
module_exit(ed64_exit);

MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
MODULE_DESCRIPTION("Everdrive-64 v3 pseudo serial driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_CHARDEV_MAJOR(ED64_MAJOR);
