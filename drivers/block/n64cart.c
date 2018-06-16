//#define DEBUG
//#define N64CART_VERIFY_READ // NOTE: VERIFY_READ is already defined...

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
#include <linux/gfp.h>
#include <linux/mfd/n64pi.h>

#define DEVICE_NAME "n64cart"

static DEFINE_SPINLOCK(hd_lock);
static int n64cart_major;
static struct request_queue *hd_queue;
static struct request *hd_req;
static void (*do_hd)(struct n64pi_request *pireq) = NULL;
#ifdef N64CART_VERIFY_READ
static void *debug_head512 = NULL;
#endif

#define SET_HANDLER(x) \
	do { do_hd = (x); } while(0)

/* TODO merge with ed64tty */
static int ed64_dummyread(struct list_head *list)
{
	struct n64pi_request *req;

	req = n64pi_alloc_request(GFP_NOIO);
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

/*
static unsigned int ed64_regread(unsigned int regoff)
{
	ed64_dummyread(); // dummy read required!!
	return __raw_readl((void*)(0xA8040000 + regoff));
}
*/

static int ed64_regwrite(uint32_t value, unsigned int regoff, struct list_head *list)
{
	struct n64pi_request *req;

	if (!ed64_dummyread(list)) { // dummy read required!!
		return 0;
	}

	req = n64pi_alloc_request(GFP_NOIO);
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

static void unexpected_hd_intr(struct n64pi_request *pireq)
{
	pr_warn("%s:%d: spurious interrupt pireqtype=%d\n", __func__, __LINE__, pireq->type);
}

static void read_intr(struct n64pi_request *pireq)
{
	u32 status;
	struct n64pi *pi;

	/* check PI DMA error and retry if error. */
	status = pireq->status;

	pi = dev_get_drvdata(((struct platform_device *)hd_req->rq_disk->private_data)->dev.parent); /* TODO dev.parent->parent is n64pi?? ipaq-micro-leds */

#ifdef DEBUG
	// note: debug print AFTER reading status is important for ed64 console!
	pr_info("%s: read_done: req=%p, disk=%ld+%Xh/%Xh, buffer=%p\n",
	       hd_req->rq_disk->disk_name, hd_req, blk_rq_pos(hd_req),
	       blk_rq_cur_bytes(hd_req), blk_rq_bytes(hd_req), bio_data(hd_req->bio));
#endif

	if((status & 7) != 0) {
		pr_err("%s: PI read error status=%u for %ld+%Xh; retrying\n", hd_req->rq_disk->disk_name, status, blk_rq_pos(hd_req), blk_rq_cur_bytes(hd_req));
		{
			struct n64pi_request *pireq;
			if (!(pireq = n64pi_alloc_request(GFP_NOIO))) {
				pr_err("%s: can't alloc n64pi_request for resetting\n", hd_req->rq_disk->disk_name);
			} else {
				pireq->type = N64PI_RTY_RESET;
				pireq->on_complete = n64pi_free_request;
				pireq->on_error = n64pi_free_request;
				n64pi_request_async(pi, pireq);
			}
		}
	} else {
//pr_info("%s: read: %ld+%Xh=>%08x\n", hd_req->rq_disk->disk_name, blk_rq_pos(hd_req), blk_rq_cur_bytes(hd_req), crc32_be(0, (void*)((unsigned int)bio_data(hd_req->bio) | 0xA0000000), blk_rq_cur_bytes(hd_req)));
//{int s=512/*blk_rq_cur_bytes(hd_req)*/;unsigned char *p=(void*)((unsigned int)bio_data(hd_req->bio) | 0xA0000000),q[64*2+1],*r=q;while(s--){sprintf(r,"%02X",*p++);r+=2;if(s%64==0){pr_info("%s\n",q);r=q;}}}
//{struct request *req=hd_req;void *b=kmalloc(512, GFP_KERNEL | GFP_NOIO);int s=512;unsigned char *p=b,q[64*2+1],*r=q;memcpy(b, (void*)((unsigned int)bio_data(req->bio) | 0xA0000000), 512);while(s--){sprintf(r,"%02X",*p++);r+=2;if(s%64==0){pr_info("%s\n",q);r=q;}};kfree(b);}
#if 0
{
	int size = blk_rq_cur_bytes(hd_req);
	int *buf = kmalloc(size, GFP_NOIO);
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
#endif

#ifdef N64CART_VERIFY_READ
		// verify: QUICK DIRTY HACK.
		// seeing cached area.
		if(debug_head512 == NULL) {
			debug_head512 = kmalloc(blk_rq_cur_bytes(hd_req), GFP_NOIO);
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
#endif

				/* acking issued blocks for kernel */
				/* note: I am called from hd_interrupt, it locks queue. __blk_end_request requires that. */
				if (!__blk_end_request(hd_req, 0, blk_rq_cur_bytes(hd_req))) {
					/* completed whole request. make see next req. */
					hd_req = NULL;
				}
#ifdef N64CART_VERIFY_READ
			}
		}
#endif
	}

	/* do next bio or enqueued request */
	hd_request();
}

static void write_ed64_intr(struct n64pi_request *pireq)
{
	u32 status;
	struct n64pi *pi;

	/* check PI DMA error and retry if error. */
	status = pireq->status;

	pi = dev_get_drvdata(((struct platform_device *)hd_req->rq_disk->private_data)->dev.parent); /* TODO dev.parent->parent is n64pi?? ipaq-micro-leds */

#ifdef DEBUG
	// note: debug print AFTER reading status is important for ed64 console!
	pr_info("%s: write_ed64_done: req=%p, disk=%ld+%Xh/%Xh, buffer=%p\n",
	       hd_req->rq_disk->disk_name, hd_req, blk_rq_pos(hd_req),
	       blk_rq_cur_bytes(hd_req), blk_rq_bytes(hd_req), bio_data(hd_req->bio));
#endif

	if((status & 7) != 0) {
		pr_err("%s: PI write error status=%u for %ld+%Xh; retrying\n", hd_req->rq_disk->disk_name, status, blk_rq_pos(hd_req), blk_rq_cur_bytes(hd_req));
		{
			struct n64pi_request *pireq;
			if (!(pireq = n64pi_alloc_request(GFP_NOIO))) {
				pr_err("%s: can't alloc n64pi_request for resetting\n", hd_req->rq_disk->disk_name);
			} else {
				pireq->type = N64PI_RTY_RESET;
				pireq->on_complete = n64pi_free_request;
				pireq->on_error = n64pi_free_request;
				n64pi_request_async(pi, pireq);
			}
		}
	} else {
		/* XXX there are no way to verify? read again?? blocking??? */

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

static void cart_on_complete(struct n64pi_request *pireq)
{
	void (*handler)(struct n64pi_request *pireq) = do_hd;

	spin_lock(hd_queue->queue_lock);

	do_hd = NULL;
	if (!handler)
		handler = unexpected_hd_intr;
	handler(pireq);

	spin_unlock(hd_queue->queue_lock);
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
	struct n64pi *pi;

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
		hd_req = NULL;
		hd_end_request_entire(-EIO);
		goto repeat;
	}

	pi = dev_get_drvdata(((struct platform_device *)req->rq_disk->private_data)->dev.parent); /* TODO dev.parent->parent is n64pi?? ipaq-micro-leds */
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
			SET_HANDLER(read_intr);
			if(((unsigned int)pcurbuf & 7) != 0) {
				pr_err("%s: buffer not aligned 8bytes: %p\n", req->rq_disk->disk_name, pcurbuf);
			}
			{
				struct n64pi_request *pireq;

				pireq = kzalloc(sizeof(*pireq), GFP_NOIO);
				if (!pireq) {
					pr_err("%s: could not allocate n64pi_request\n", req->rq_disk->disk_name);
					hd_req = NULL;
					hd_end_request_entire(-ENOMEM);
					break;
				}

				pireq->type = N64PI_RTY_C2R_DMA;
				pireq->cart_address = 0x10000000 + block * 512;
				pireq->ram_vaddress = pcurbuf;
				pireq->length = ncurbytes;
				pireq->on_complete = cart_on_complete;
				pireq->on_error = n64pi_free_request;
				//pireq->cookie = NULL;
				n64pi_request_async(pi, pireq);
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
				struct list_head reqs;
				struct n64pi_request *pireq;

				INIT_LIST_HEAD(&reqs);

				if (!ed64_enable(&reqs)) {
					hd_req = NULL;
					hd_end_request_entire(-ENOMEM);
					break;
				}

				if (!ed64_dummyread(&reqs)) { // dummyread for r2c DMA
					// TODO free reqs
					hd_req = NULL;
					hd_end_request_entire(-ENOMEM);
					break;
				}

				pireq = n64pi_alloc_request(GFP_NOIO);
				if (!pireq) {
					pr_err("%s: could not allocate n64pi_request (%d)\n", req->rq_disk->disk_name, __LINE__);
					// TODO free reqs
					hd_req = NULL;
					hd_end_request_entire(-ENOMEM);
					break;
				}
				pireq->type = N64PI_RTY_RESET;
				pireq->on_complete = n64pi_free_request;
				pireq->on_error = n64pi_free_request;
				//pireq->cookie = NULL;
				list_add_tail(&pireq->node, &reqs);

				pireq = n64pi_alloc_request(GFP_NOIO);
				if (!pireq) {
					pr_err("%s: could not allocate n64pi_request (%d)\n", req->rq_disk->disk_name, __LINE__);
					// TODO free reqs
					hd_req = NULL;
					hd_end_request_entire(-ENOMEM);
					break;
				}
				pireq->type = N64PI_RTY_R2C_DMA;
				pireq->cart_address = 0x10000000 + block * 512;
				pireq->ram_vaddress = pcurbuf;
				pireq->length = ncurbytes;
				pireq->on_complete = cart_on_complete;
				pireq->on_error = n64pi_free_request;
				//pireq->cookie = NULL;
				list_add_tail(&pireq->node, &reqs);

				/* enqueue disabling now, or tty may be enqueue while DMAing, wedge as result, potentially cause clashing ED64 reg state. */
				if (!ed64_disable(&reqs)) {
					// TODO free reqs
					hd_req = NULL;
					hd_end_request_entire(-ENOMEM);
					break;
				}

				n64pi_many_request_async(pi, &reqs);
			}
#endif
			break;
		default:
			pr_err("unknown hd-command\n");
			hd_req = NULL;
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

static int n64cart_probe(struct platform_device *pdev)
{
	int error;
	struct gendisk *disk;

	/* TODO this driver does not support two or more devices. detect that and spit an error. */

	error = register_blkdev(0, DEVICE_NAME);
	if (error <= 0) {
		dev_err(&pdev->dev, "register_blkdev failed %d\n", error);
		goto out_regblk;
	}
	n64cart_major = error;

	dev_info(&pdev->dev, "registered block device major %d\n", n64cart_major);

	hd_queue = blk_init_queue(do_hd_request, &hd_lock);
	if (!hd_queue) {
		dev_err(&pdev->dev, "could not initialize queue\n");
		error = -ENOMEM;
		goto out_queue;
	}

	blk_queue_max_hw_sectors(hd_queue, 15); /* FIXME how sectors are good? */
	blk_queue_logical_block_size(hd_queue, 512);

	disk = alloc_disk(16); /* max 16 minors(parts) */
	disk->major = n64cart_major;
	disk->first_minor = 0;
	sprintf(disk->disk_name, "cart%c", '0');
	set_capacity(disk, 48*1024*1024/512); /* FIXME be configurable */
	disk->fops = &hd_fops;
	disk->queue = hd_queue;
	disk->private_data = pdev;
	add_disk(disk); /* no error on add_disk, that is void... */

	return 0;

/* n64pi mfd does this, never fail to here.
out_reqiomem:
	put_disk(disk);
	blk_cleanup_queue(hd_queue);
*/
out_queue:
	unregister_blkdev(n64cart_major, DEVICE_NAME);
out_regblk:
	return error;
}

static int n64cart_remove(struct platform_device *pdev)
{
	//put_disk(disk);
	blk_cleanup_queue(hd_queue);
	unregister_blkdev(n64cart_major, DEVICE_NAME);
	return 0;
}

static struct platform_driver n64cart_driver = {
	.probe  = n64cart_probe,
	.remove = n64cart_remove,
	.driver = {
	    .name = "n64pi-cart",
	},
};

static int __init n64cart_init(void)
{
	int error;

	error = platform_driver_register(&n64cart_driver);
	if (error) {
		pr_err("%s:%u: could not register platform driver\n", __func__, __LINE__);
		return error;
	}

	return 0;
}

static void __exit n64cart_exit(void)
{
	platform_driver_unregister(&n64cart_driver);
}

module_init(n64cart_init);
module_exit(n64cart_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nintedo 64 Cartridge Driver");
MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
