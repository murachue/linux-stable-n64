//#define DEBUG

/* large parts from drivers/block/hd.c */
/* TODO rename "hd" */

/* TODO OpenFirmware DeviceTree support? */
#ifdef CONFIG_USE_OF
#error n64cart does not support device tree yet
#endif

#include <linux/blkdev.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/hdreg.h> // struct hd_geometry
#include <linux/interrupt.h>
#include <asm/addrspace.h> // CPHYSADDR

#define DEVICE_NAME "n64cart"
/* TODO use probe.arg0 platform_device->resource IORESOURCE_MEM (platform_get_resource IORESOURCE_MEM) */
#define PI_PHYSBASE 0x04600000
#define PI_SIZE 0x34
/* TODO use probe.arg0 platform_device->resource IORESOURCE_IRQ (platform_get_irq for with setting trigger) */
#define PI_IRQ (8+4) /* PI intr at MI_INTR_REG[4] */

/* redundant. should not be required? */
static struct platform_driver n64cart_driver = {
	.driver = {
		.name = "n64pi",
	},
};

static DEFINE_SPINLOCK(hd_lock);
static int n64cart_major;
static struct request_queue *hd_queue;
static struct request *hd_req;
static void __iomem *membase;
static void (*do_hd)(void) = NULL;
static void *debug_head512 = NULL;

#define SET_HANDLER(x) \
	do { do_hd = (x); } while(0)

/* FIXME copied from drivers/tty/serial/ed64.c, then modified */
static void ed64_dummyread(void)
{
	__raw_readl((void*)(0xA8040000 + 0x00)); // dummy read required!!
}

/*
static unsigned int ed64_regread(unsigned int regoff)
{
	ed64_dummyread(); // dummy read required!!
	return __raw_readl((void*)(0xA8040000 + regoff));
}
*/

static void ed64_regwrite(unsigned int value, unsigned int regoff)
{
	ed64_dummyread(); // dummy read required!!
	__raw_writel(value, (void*)(0xA8040000 + regoff));
}

static void ed64_enable(void)
{
	ed64_regwrite(0x1234, 0x20);
}

static void ed64_disable(void)
{
	ed64_regwrite(0, 0x20);
}

static bool hd_end_request(int err, unsigned int bytes)
{
	/* note: I am called from hd_interrupt, it locks queue. __blk_end_request requires that. */
	if (__blk_end_request(hd_req, err, bytes))
		return true;
	hd_req = NULL;
	return false;
}

static bool hd_end_request_entire(int err)
{
	return hd_end_request(err, blk_rq_bytes(hd_req));
}

static void hd_request (void);

static void unexpected_hd_intr(void)
{
	pr_warn("%s:%d: spurious interrupt\n", __func__, __LINE__);
}

static void read_intr(void)
{
	u32 status;

	/* check PI DMA error and retry if error. */
	status = __raw_readl(membase + 0x10);

#ifdef DEBUG
	// note: debug print AFTER reading status is important for ed64 console!
	pr_info("%s: read_done: req=%p, disk=%ld+%Xh/%Xh, buffer=%p\n",
	       hd_req->rq_disk->disk_name, hd_req, blk_rq_pos(hd_req),
	       blk_rq_cur_bytes(hd_req), blk_rq_bytes(hd_req), bio_data(hd_req->bio));
#endif

	if((status & 7) != 0) {
		pr_err("%s: PI read error status=%u for %ld+%Xh; retrying\n", hd_req->rq_disk->disk_name, status, blk_rq_pos(hd_req), blk_rq_cur_bytes(hd_req));
		__raw_writel(3, membase + 0x10); // PI_STATUS = RESET|CLEARINTR
	} else {
//pr_info("%s: read: %ld+%Xh=>%08x\n", hd_req->rq_disk->disk_name, blk_rq_pos(hd_req), blk_rq_cur_bytes(hd_req), crc32_be(0, (void*)((unsigned int)bio_data(hd_req->bio) | 0xA0000000), blk_rq_cur_bytes(hd_req)));
//{int s=512/*blk_rq_cur_bytes(hd_req)*/;unsigned char *p=(void*)((unsigned int)bio_data(hd_req->bio) | 0xA0000000),q[64*2+1],*r=q;while(s--){sprintf(r,"%02X",*p++);r+=2;if(s%64==0){pr_info("%s\n",q);r=q;}}}
//{struct request *req=hd_req;void *b=kmalloc(512, GFP_KERNEL | GFP_NOIO);int s=512;unsigned char *p=b,q[64*2+1],*r=q;memcpy(b, (void*)((unsigned int)bio_data(req->bio) | 0xA0000000), 512);while(s--){sprintf(r,"%02X",*p++);r+=2;if(s%64==0){pr_info("%s\n",q);r=q;}};kfree(b);}
if(0){
	int size = blk_rq_cur_bytes(hd_req);
	int *buf = kmalloc(size, GFP_KERNEL | GFP_NOIO);
	void *bbuf = (void*)((unsigned int)bio_data(hd_req->bio) | 0xA0000000);
	//__asm("mtc0 %0,$19; mtc0 %1, $18" : : "r"(0), "r"((((unsigned int)bbuf + 0x10) & 0x1FFFfff8) | 1)); // Write watch to bio buf + 0x10 (="/dev/root")
	__flush_cache_all();
	memcpy(buf, bbuf, size);

	if(0){
		int s = 512;
		char *p = bbuf, q[64*2+1], *r = q;
		while(s--){
			sprintf(r, "%02X", *p++);
			r += 2;
			if(s % 64 == 0) {
				pr_info("%s\n",q);
				r = q;
			}
		}
	}
	if(memcmp(buf, bbuf, size)) {
		pr_err("n64cart: memcmp failed\n");
		if(0){
			int s = 512;
			char *p = bbuf, q[64*2+1], *r = q;
			while(s--){
				sprintf(r, "%02X", *p++);
				r += 2;
				if(s % 64 == 0) {
					pr_info("%s\n",q);
					r = q;
				}
			}
		}
		panic("n64cart bio has broken");
	}
	kfree(buf);
}

		// verify: QUICK DIRTY HACK.
		// seeing cached area.
		if(debug_head512 == NULL) {
			debug_head512 = kmalloc(blk_rq_cur_bytes(hd_req), GFP_KERNEL | GFP_NOIO);
			memcpy(debug_head512, bio_data(hd_req->bio), blk_rq_cur_bytes(hd_req));
		} else {
			int r;
			{
				const unsigned char *p = debug_head512, *q = bio_data(hd_req->bio);
				int len = blk_rq_cur_bytes(hd_req);
				const int atonce = 32;
				char hexp[atonce*2+1], hexq[atonce*2+1];
				while(0 < len) {
					r = memcmp(p, q, atonce);
					if(r) {
						int i;
						for(i = 0; i < atonce; i++) {
							sprintf(hexp + i * 2, "%02X", p[i]);
							sprintf(hexq + i * 2, "%02X", q[i]);
						}
						pr_err("E+%03xh: %s %s\n", (unsigned)p - (unsigned)debug_head512, hexp, hexq);
						//break;
					}
					p += atonce;
					q += atonce;
					len -= atonce;
				}
			}

			if(r /*memcmp(debug_head512, bio_data(hd_req->bio), blk_rq_cur_bytes(hd_req)) != 0*/) {
				pr_err("%s: debug %p <=> read %ld+%Xh verify failed; retrying\n", hd_req->rq_disk->disk_name, debug_head512, blk_rq_pos(hd_req), blk_rq_cur_bytes(hd_req));
				memcpy(debug_head512, bio_data(hd_req->bio), blk_rq_cur_bytes(hd_req));
			} else {
				kfree(debug_head512);
				debug_head512 = NULL;

				/* acking issued blocks for kernel */
				/* note: I am called from hd_interrupt, it locks queue. __blk_end_request requires that. */
				if (!__blk_end_request(hd_req, 0, blk_rq_cur_bytes(hd_req))) {
					/* completed whole request. make see next req. */
					hd_req = NULL;
				}
			}
		}
	}

	/* do next bio or enqueued request */
	hd_request();
}

static void write_ed64_intr(void)
{
	u32 status;

	/* check PI DMA error and retry if error. */
	status = __raw_readl(membase + 0x10);

#ifdef DEBUG
	// note: debug print AFTER reading status is important for ed64 console!
	pr_info("%s: write_ed64_done: req=%p, disk=%ld+%Xh/%Xh, buffer=%p\n",
	       hd_req->rq_disk->disk_name, hd_req, blk_rq_pos(hd_req),
	       blk_rq_cur_bytes(hd_req), blk_rq_bytes(hd_req), bio_data(hd_req->bio));
#endif

	if((status & 7) != 0) {
		pr_err("%s: PI write error status=%u for %ld+%Xh; retrying\n", hd_req->rq_disk->disk_name, status, blk_rq_pos(hd_req), blk_rq_cur_bytes(hd_req));
		__raw_writel(3, membase + 0x10); // PI_STATUS = RESET|CLEARINTR
	} else {
		/* XXX there are no way to verify? read again?? blocking??? */

		/* TODO reconsider conflict with ed64tty. */
		ed64_disable();

		/* acking issued blocks for kernel */
		/* note: I am called from hd_interrupt, it locks queue. __blk_end_request requires that. */
		if (!__blk_end_request(hd_req, 0, blk_rq_cur_bytes(hd_req))) {
			/* completed whole request. make see next req. */
			hd_req = NULL;
		}
	}

	/* do next bio or enqueued request */
	hd_request();
}

/*
 * The driver enables interrupts as much as possible.  In order to do this,
 * (a) the device-interrupt is disabled before entering hd_request(),
 * and (b) the timeout-interrupt is disabled before the sti().
 *
 * Interrupts are still masked (by default) whenever we are exchanging
 * data/cmds with a drive, because some drives seem to have very poor
 * tolerance for latency during I/O. The IDE driver has support to unmask
 * interrupts for non-broken hardware, so use that driver if required.
 */
static void hd_request(void)
{
	sector_t block;
	unsigned int nsect, ncurbytes;
	void *pcurbuf;
	struct request *req;

	/* do_hd != NULL means waiting read done interrupt, ex. block-queue issued too fast. */
	if (do_hd)
		return;

repeat:
	/* hd_req != NULL means we are processing a request partially. */
	if (!hd_req) {
		hd_req = blk_fetch_request(hd_queue);
		if (!hd_req) {
			/* no more jobs. */
			do_hd = NULL;
			return;
		}
	}
	req = hd_req;

	block = blk_rq_pos(req);
	nsect = blk_rq_sectors(req);
	if (block >= get_capacity(req->rq_disk) || ((block+nsect) > get_capacity(req->rq_disk))) {
		pr_err("%s: bad access: block=%ld, count=%d\n", req->rq_disk->disk_name, block, nsect);
		hd_end_request_entire(-EIO);
		goto repeat;
	}

	ncurbytes = blk_rq_cur_bytes(req);
	pcurbuf = bio_data(req->bio);

#ifdef DEBUG
	pr_info("%s: %s req: req=%p, block=%ld, size=%Xh/%Xh, buffer=%p\n",
		req->rq_disk->disk_name,
		rq_data_dir(req) == READ ? "read" : "write",
		req, block, ncurbytes, nsect * 512, pcurbuf);
#endif
	if (req->cmd_type == REQ_TYPE_FS) {
		switch (rq_data_dir(req)) {
		case READ:
			/*
			for(;;){
				u32 status = __raw_readl(membase + 0x10);
				if((status & 7) != 0) {
					pr_err("%s: PI busy status=%u\n", req->rq_disk->disk_name, status);
					__raw_writel(3, membase + 0x10); // PI_STATUS = RESET|CLEARINTR
				} else {
					break;
				}
			}
			*/
			SET_HANDLER(read_intr);
			if(((unsigned int)pcurbuf & 7) != 0) {
				pr_err("%s: buffer not aligned 8bytes: %p\n", req->rq_disk->disk_name, pcurbuf);
			}
			{
				unsigned long flags;
				local_irq_save(flags);
				__raw_writel(3, membase + 0x10); // PI_STATUS = RESET|CLEARINTR
				dma_cache_inv((unsigned long)pcurbuf, ncurbytes); // or blast_inv_dcache_{range,page}?
				/* TODO is CPHYSADDR correct?? no other virt2phys like func? */
				__raw_writel(CPHYSADDR(pcurbuf), membase + 0x00); // PI_DRAM_ADDR
				__raw_writel(0x10000000 + block * 512, membase + 0x04); // PI_CART_ADDR
				barrier();
				__raw_writel(ncurbytes - 1, membase + 0x0C); // PI_WR_LEN
				local_irq_restore(flags);
			}
			break;
		case WRITE:
// TODO CONFIG_N64CART_ED64WRITE
#if 0
			hd_end_request_entire(-EROFS);
#else
			SET_HANDLER(write_ed64_intr);
			if(((unsigned int)pcurbuf & 7) != 0) {
				pr_err("%s: buffer not aligned 8bytes: %p\n", req->rq_disk->disk_name, pcurbuf);
			}
			{
				unsigned long flags;
				local_irq_save(flags);
				ed64_enable();
				ed64_dummyread(); // dummyread for r2c DMA
				__raw_writel(3, membase + 0x10); // PI_STATUS = RESET|CLEARINTR
				dma_cache_wback_inv((unsigned long)pcurbuf, ncurbytes); // or blast_dcache_{range,page}?
				/* TODO is CPHYSADDR correct?? no other virt2phys like func? */
				__raw_writel(CPHYSADDR(pcurbuf), membase + 0x00); // PI_DRAM_ADDR
				__raw_writel(0x10000000 + block * 512, membase + 0x04); // PI_CART_ADDR
				barrier();
				__raw_writel(ncurbytes - 1, membase + 0x08); // PI_RD_LEN
				local_irq_restore(flags);
			}
#endif
			break;
		default:
			pr_err("unknown hd-command\n");
			hd_end_request_entire(-EIO);
			break;
		}
	}
}

/* called from queue */
static void do_hd_request(struct request_queue *q)
{
	hd_request();
}

/*
 * Releasing a block device means we sync() it, so that it can safely
 * be forgotten about...
 */

static irqreturn_t hd_interrupt(int irq, void *dev_id)
{
	void (*handler)(void) = do_hd;

	spin_lock(hd_queue->queue_lock);

	/*
	 * whatever (even unexpected), acking for PI.
	 * MUST DO THIS BEFORE EXECUTING HANDLER, because handler may issue another request.
	 * if do after that, this acks wrong (next) request, cause infinite wait for next request.
	 */
	__raw_writel(2, membase + 0x10); // PI_STATUS, [1]=clear_intr

	do_hd = NULL;
	if (!handler)
		handler = unexpected_hd_intr;
	handler();

	spin_unlock(hd_queue->queue_lock);

	return IRQ_HANDLED;
}

static int hd_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	/* bdev->bd_disk->private_data; */

	/* TODO fill values that something looks correct? */
	geo->cylinders = 4;
	geo->heads = 32;
	geo->sectors = 64;
	return 0;
}

static const struct block_device_operations hd_fops = {
	.owner = THIS_MODULE,
	.getgeo = hd_getgeo,
};

static int __init n64cart_init(void)
{
	int error;
	struct gendisk *disk;

	error = register_blkdev(0, DEVICE_NAME);
	if (error <= 0) {
		pr_err("%s:%u: register_blkdev failed %d\n", __func__, __LINE__, error);
		goto out_regblk;
	}
	n64cart_major = error;

	pr_info("%s:%u: registered block device major %d\n", __func__, __LINE__, n64cart_major);

	hd_queue = blk_init_queue(do_hd_request, &hd_lock);
	if (!hd_queue) {
		pr_err("%s:%u: could not initialize queue\n", __func__, __LINE__);
		error = -ENOMEM;
		goto out_queue;
	}

	blk_queue_max_hw_sectors(hd_queue, 15); /* FIXME how sectors are good? */
	blk_queue_logical_block_size(hd_queue, 512);

	disk = alloc_disk(1); /* max 1 minors(parts) */
	disk->major = n64cart_major;
	disk->first_minor = 0;
	sprintf(disk->disk_name, "cart%c", '0');
	set_capacity(disk, 48*1024*1024/512); /* FIXME be configurable */
	disk->fops = &hd_fops;
	disk->queue = hd_queue;
	disk->private_data = NULL;
	add_disk(disk); /* no error on add_disk, that is void... */

	if (!request_mem_region(PI_PHYSBASE, PI_SIZE/*bytes*/, DEVICE_NAME)) {
		pr_err("%s:%u: IOMEM 0x%x busy\n", __func__, __LINE__, PI_PHYSBASE);
		error = -ENOMEM;
		goto out_reqiomem;
	}

	membase = ioremap_nocache(PI_PHYSBASE, PI_SIZE/*bytes*/);
	if (!membase) {
		pr_err("%s:%u: could not get nocache area for IOMEM 0x%x\n", __func__, __LINE__, PI_PHYSBASE);
		error = -ENOMEM;
		goto out_ioremap;
	}

	/* ack before requesting IRQ, to avoid spurious intr at start. */
	/* (it may be 1 because PI used by boot-loader.) */
	__raw_writel(2, membase + 0x10); // PI_STATUS, [1]=clear_intr

	error = request_irq(PI_IRQ, hd_interrupt, 0, DEVICE_NAME, NULL);
	if (error) {
		pr_err("%s:%u: unable to get IRQ%d for the Nintendo 64 cartridge driver\n", __func__, __LINE__, PI_IRQ);
		goto out_reqirq;
	}

	error = platform_driver_register(&n64cart_driver);
	if (error) {
		pr_err("%s:%u: could not register platform driver\n", __func__, __LINE__);
		goto out_drvreg;
	}

	return 0;
out_drvreg:
	free_irq(PI_IRQ, NULL);
out_reqirq:
	iounmap(membase);
out_ioremap:
	release_mem_region(PI_PHYSBASE, PI_SIZE);
out_reqiomem:
	put_disk(disk);
	blk_cleanup_queue(hd_queue);
out_queue:
	unregister_blkdev(n64cart_major, DEVICE_NAME);
out_regblk:
	return error;
}

static void __exit n64cart_exit(void)
{
	iounmap(membase);
	release_mem_region(PI_PHYSBASE, PI_SIZE);
	free_irq(PI_IRQ, NULL);
	//put_disk(disk);
	blk_cleanup_queue(hd_queue);
	unregister_blkdev(n64cart_major, DEVICE_NAME);
}

module_init(n64cart_init);
module_exit(n64cart_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nintedo 64 Cartridge Driver");
MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
