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

static bool hd_end_request(int err, unsigned int bytes)
{
	/* note: I am called from blk-core, it locks queue. __blk_end_request requires that. */
	if (__blk_end_request(hd_req, err, bytes))
		return true;
	hd_req = NULL;
	return false;
}

static bool hd_end_request_entire(int err)
{
	int ret;
	ret = hd_end_request(err, blk_rq_bytes(hd_req));
	hd_req = NULL; /* it will be already done by hd_end_request... but ensure this. */
	return ret;
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
	int ret;

	for(;;) {
		/* hd_req != NULL means we are processing a request partially. */
		if (!hd_req) {
			hd_req = blk_fetch_request(hd_queue);
			if (!hd_req) {
				/* no more jobs. */
				return;
			}

			/* verify this request is valid */
			req = hd_req; /* cache */

			block = blk_rq_pos(req);
			nsect = blk_rq_sectors(req);
			if (block >= get_capacity(req->rq_disk) || ((block+nsect) > get_capacity(req->rq_disk))) {
				pr_err("%s: bad access: block=%ld, count=%d\n", req->rq_disk->disk_name, block, nsect);
				hd_end_request_entire(-EIO);
				continue;
			}
		}

		req = hd_req;

		pi = dev_get_drvdata(((struct platform_device *)req->rq_disk->private_data)->dev.parent); /* TODO dev.parent->parent is n64pi?? cf. ipaq-micro-leds */
		ncurbytes = blk_rq_cur_bytes(req);
		pcurbuf = bio_data(req->bio);

#ifdef DEBUG
		pr_info("%s: %s req: req=%p, block=%ld, size=%Xh/%Xh, buffer=%p\n",
			req->rq_disk->disk_name,
			rq_data_dir(req) == READ ? "read" : "write",
			req, block, ncurbytes, nsect * 512, pcurbuf);
#endif
		if (req->cmd_type != REQ_TYPE_FS) {
			/* TODO should hd_end_request_entire?? */
			continue;
		}

		switch (rq_data_dir(req)) {
		case READ:
			if(((unsigned int)pcurbuf & 7) != 0) {
				pr_err("%s: buffer not aligned 8bytes: %p\n", req->rq_disk->disk_name, pcurbuf);
			}
			{
#ifdef N64CART_VERIFY_READ
				void *verify_buffer = NULL;
#endif

				n64pi_begin(pi);

#ifdef N64CART_VERIFY_READ
				for(;;) { /* loop until re-read content matches to pre-read. */
#endif
					while ((ret = n64pi_read_dma(pi, pcurbuf, 0x10000000 + block * 512, ncurbytes)) != N64PI_ERROR_SUCCESS) {
						pr_err("%s: PI read error (%d) for %ld+%Xh; retrying\n", req->rq_disk->disk_name, ret, blk_rq_pos(req), blk_rq_cur_bytes(req));
					}

#ifdef DEBUG
					pr_info("%s: read_done: req=%p, disk=%ld+%Xh/%Xh, buffer=%p\n",
					        req->rq_disk->disk_name, req, blk_rq_pos(req), blk_rq_cur_bytes(req), blk_rq_bytes(req), bio_data(req->bio));
#endif

#ifdef N64CART_VERIFY_READ
					// verify: QUICK DIRTY HACK.
					if(verify_buffer == NULL) {
						/* this is first read: copy read data to verify_buffer and re-read. */
						verify_buffer = kmalloc(blk_rq_cur_bytes(req), GFP_NOIO);
						memcpy(verify_buffer, bio_data(req->bio), blk_rq_cur_bytes(req));
						continue; /* to re-read same area. */
					}

					/* this is second read: verify with first-read content. */
					/* following block does "memcmp(verify_buffer, bio_data(req->bio), blk_rq_cur_bytes(req))" with showing diff */
					{
						int r = 0;
						{
							const unsigned char *p = verify_buffer, *q = bio_data(req->bio);
							int len = blk_rq_cur_bytes(req);
							const int atonce = 32;
							char hexp[atonce*2+1], hexq[atonce*2+1];
							while(0 < len) {
								int rr = memcmp(p, q, atonce);
								if(rr) {
									int i;
									for(i = 0; i < atonce; i++) {
										sprintf(hexp + i * 2, "%02X", p[i]);
										sprintf(hexq + i * 2, "%02X", q[i]);
									}
									pr_err("E+%03xh: %s %s\n", (unsigned)p - (unsigned)verify_buffer, hexp, hexq);
									r = 1; //break;
								}
								p += atonce;
								q += atonce;
								len -= atonce;
							}
						}

						if(r) {
							pr_err("%s: debug %p <=> read %ld+%Xh verify failed; retrying\n", req->rq_disk->disk_name, verify_buffer, blk_rq_pos(req), blk_rq_cur_bytes(req));
							// remember again
							memcpy(verify_buffer, bio_data(req->bio), blk_rq_cur_bytes(req));
							continue; /* to re-read same area. */
						}
					}

					/* verify ok, forget first-read. */
					kfree(verify_buffer);
					verify_buffer = NULL;
#endif

					/* acking issued blocks for kernel */
					/* note: I am called from blk-core, it locks queue. __blk_end_request requires that. */
					hd_end_request(0, blk_rq_cur_bytes(req));
#ifdef N64CART_VERIFY_READ
				} /* end of "for(;;)" for verify */
#endif

				n64pi_end(pi);
			}
			break;
		case WRITE:
// TODO CONFIG_N64CART_ED64WRITE
#if 0
			hd_end_request_entire(-EROFS);
#else
			// TODO test ED64 DMA before write?

			if(((unsigned int)pcurbuf & 7) != 0) {
				pr_err("%s: buffer not aligned 8bytes: %p\n", req->rq_disk->disk_name, pcurbuf);
			}
			{
				n64pi_begin(pi);

				if (n64pi_ed64_enable(pi) != N64PI_ERROR_SUCCESS) {
					pr_err("%s: could not enable ED64 registers\n", req->rq_disk->disk_name);
					hd_end_request_entire(-ENOMEM);
					n64pi_end(pi);
					break;
				}

				while ((ret = n64pi_write_dma(pi, 0x10000000 + block * 512, pcurbuf, ncurbytes)) != N64PI_ERROR_SUCCESS) {
					pr_err("%s: PI write error (%d) for %ld+%Xh; retrying\n", req->rq_disk->disk_name, ret, blk_rq_pos(req), blk_rq_cur_bytes(req));
				}

#ifdef DEBUG
				// note: debug print AFTER reading status is important for ed64 console!
				pr_info("%s: write_ed64_done: req=%p, disk=%ld+%Xh/%Xh, buffer=%p\n",
				       hd_req->rq_disk->disk_name, hd_req, blk_rq_pos(hd_req),
				       blk_rq_cur_bytes(hd_req), blk_rq_bytes(hd_req), bio_data(hd_req->bio));
#endif

				if (n64pi_ed64_disable(pi) != N64PI_ERROR_SUCCESS) {
					pr_err("%s: could not disable ED64 registers\n", req->rq_disk->disk_name);
					hd_end_request_entire(-ENOMEM);
					n64pi_end(pi);
					break;
				}

				/* XXX there are no way to verify? read again?? blocking??? */

				/* acking issued blocks for kernel */
				/* note: I am called from blk-core, it locks queue. __blk_end_request requires that. */
				hd_end_request(0, blk_rq_cur_bytes(req));

				n64pi_end(pi);
			}
#endif
			break;
		default:
			pr_err("unknown hd-command\n");
			hd_end_request_entire(-EIO);
			break;
		} /* switch(rq_data_dir(req)) */
	} /* "for(;;)" for processing queue loop */
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
