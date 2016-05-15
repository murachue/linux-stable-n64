
/* large parts from drivers/block/hd.c */
/* TODO rename "hd" */

/* TODO OpenFirmware DeviceTree support? */
#ifdef CONFIG_USE_OF
#error n64cart does not support device tree yet
#endif

#include <linux/blkdev.h>
#include <linux/module.h>
#include <linux/io.h>

#define DEVICE_NAME "n64cart"
#define PI_PHYSBASE 0x04600000
#define PI_SIZE 0x34
#define PI_IRQ (8+5) /* PI intr at MI_INTR_REG[4] */

/* redundant. should not be required? */
static struct platform_driver n64cart_driver = {
	.driver = {
		.name = "n64cart",
	},
};

static DEFINE_SPINLOCK(hd_lock);
static int n64cart_major;
static struct request_queue *hd_queue;
static struct request *hd_req;
static void __iomem *membase;
static void (*do_hd)(void) = NULL;

#define SET_HANDLER(x) \
	do { do_hd = (x) } while(0)

static bool hd_end_request(int err, unsigned int bytes)
{
	/* note: I am called from hd_interrupt, it locks queue. __blk_end_request requires that. */
	if (__blk_end_request(hd_req, err, bytes))
		return true;
	hd_req = NULL;
	return false;
}

static bool hd_end_request_cur(int err)
{
	return hd_end_request(err, blk_rq_cur_bytes(hd_req));
}

static void hd_request (void);

static void unexpected_hd_intr(void)
{
	printk(KERN_WARN "%s:%d: spurious interrupt\n", __func__, __LINE__);
}

static void read_intr(void)
{
#ifdef DEBUG
	printk("%s: read: pos=%ld, nsect=%u, buffer=%p\n",
	       hd_req->rq_disk->disk_name, blk_rq_pos(hd_req),
	       blk_rq_sectors(hd_req), bio_data(hd_req->bio));
#endif
	/* acking for PI */
	__raw_writel((u32 __iomem *)membase + 4, 2); // PI_STATUS

	/* acking all blocks for kernel */
	/* note: I am called from hd_interrupt, it locks queue. __blk_end_request requires that. */
	if (__blk_end_request(hd_req, 0, blk_rq_cur_bytes(hd_req))) {
		printk(KERN_WARN "%s:%d: %s read remains?? pos=%ld nsect=%u\n", __func__, __LINE__, hd_req->rq_disk->disk_name, blk_rq_pos(hd_req), blk_rq_sectors(hd_req));
		SET_HANDLER(&read_intr);
		return;
	}
	hd_req = NULL;

	/* do next (potentially) enqueued request */
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
	unsigned int block, nsect;
	struct request *req;

	if (do_hd)
		return;

repeat:
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
		printk("%s: bad access: block=%d, count=%d\n", req->rq_disk->disk_name, block, nsect);
		hd_end_request_cur(-EIO);
		goto repeat;
	}

	if (disk->special_op) {
		if (do_special_op(disk, req))
			goto repeat;
		return;
	}
#ifdef DEBUG
	printk("%s: %sing: block=%d, sectors=%d, buffer=%p\n",
		req->rq_disk->disk_name,
		req_data_dir(req) == READ ? "read" : "writ",
		block, nsect, bio_data(req->bio));
#endif
	if (req->cmd_type == REQ_TYPE_FS) {
		switch (rq_data_dir(req)) {
		case READ:
			{
				u32 status = __raw_readl((u32 __iomem *)membase + 0);
				if((status & 7) != 0) {
					printk("%s: PI busy status=%u\n", req->rq_disk->disk_name, status);
				}
			}
			SET_HANDLER(read_intr);
			__raw_writel((u32 __iomem *)membase + 0, virt2phys(bio_data(req->bio))); // PI_DRAM_ADDR
			__raw_writel((u32 __iomem *)membase + 1, 0x10000000 + block * 512); // PI_CART_ADDR
			__raw_writel((u32 __iomem *)membase + 3, nsect * 512 - 1); // PI_WR_LEN
			if (reset)
				goto repeat;
			break;
		case WRITE:
			hd_end_request_cur(-EROFS);
			break;
		default:
			printk("unknown hd-command\n");
			hd_end_request_cur(-EIO);
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
	.getgeo =	hd_getgeo,
};

static int __init n64cart_init(void)
{
	int error;

	error = register_blkdev(0, DEVICE_NAME);
	if (error <= 0) {
		printk(KERN_ERR "%s:%u: register_blkdev failed %d\n", __func__, __LINE__, error);
		goto out_regblk;
	}
	n64cart_major = error;

	pr_info("%s:%u: registered block device major %d\n", __func__, __LINE__, n64cart_major);

	hd_queue = blk_init_queue(do_hd_request, &hd_lock);
	if (!hd_queue) {
		printk(KERN_ERR "%s:%u: could not initialize queue\n", __func__, __LINE__);
		error = -ENOMEM;
		goto out_queue;
	}

	blk_queue_max_hw_sectors(hd_queue, 15); /* FIXME how sectors are good? */
	blk_queue_logical_block_size(hd_queue, 512);

	struct gendisk *disk = alloc_disk(64); /* max 64 minors(parts) */
	disk->major = n64cart_major;
	disk->first_minor = 0;
	sprintf(disk->disk_name, "cart%c", '0'+drive);
	set_capacity(disk, 4*1024*1024/512); /* FIXME be configurable */
	disk->fops = &hd_fops;
	disk->queue = hd_queue;
	disk->private_data = NULL;
	add_disk(disk); /* no error on add_disk, that is void... */

	error = request_irq(PI_IRQ, hd_interrupt, 0, DEVICE_NAME, NULL);
	if (error) {
		printk(KERN_ERR "%s:%u: unable to get IRQ%d for the Nintendo 64 cartridge driver\n", __func__, __LINE__, PI_IRQ);
		goto out_reqirq;
	}

	if (!request_mem_region(PI_PHYSBASE, PI_SIZE/*bytes*/, DEVICE_NAME)) {
		printk(KERN_ERR "%s:%u: IOMEM 0x%x busy\n", __func__, __LINE__, PI_PHYSBASE);
		error = -ENOMEM;
		goto out_reqiomem;
	}

	membase = ioremap_nocache(PI_PHYSBASE, PI_SIZE/*bytes*/);
	if (!membase) {
		printk(KERN_ERR "%s:%u: could not get nocache area for IOMEM 0x%x\n", __func__, __LINE__, PI_PHYSBASE);
		error = -ENOMEM;
		goto out_ioremap;
	}

	error = platform_driver_register(&n64cart_driver);
	if (error) {
		printk(KERN_ERR "%s:%u: could not register platform driver\n", __func__, __LINE__);
		goto out_drvreg;
	}

	return 0;
out_drvreg:
	iounmap(membase);
out_ioremap:
	release_mem_region(PI_PHYSBASE, PI_SIZE);
out_reqiomem:
	free_irq(PI_IRQ, NULL);
out_reqirq:
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
	put_disk(disk);
	blk_cleanup_queue(hd_queue);
	unregister_blkdev(n64cart_major, DEVICE_NAME);
}

module_init(n64cart_init);
module_exit(n64cart_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nintedo 64 Cartridge Driver");
MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
