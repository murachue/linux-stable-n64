/*
 * linux/drivers/video/n64vi.c -- Nintendo 64 VI frame buffer device
 * (based on q40fb.c)
 *
 * Copyright (C) 2016 Murachue <murachue+github@gmail.com>
 *
 * note: This driver does not support RDP acceleration yet.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/uaccess.h>
#include <asm/setup.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <asm/pgtable.h>

static struct fb_fix_screeninfo n64vifb_fix = {
	.id          = "N64VIFB",
	.smem_len    = 640*480*2,
	.type        = FB_TYPE_PACKED_PIXELS,
	.visual      = FB_VISUAL_TRUECOLOR,
	.line_length = 640*2,
	.accel       = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo n64vifb_var = {
	.xres           = 640,
	.yres           = 480,
	.xres_virtual   = 640,
	.yres_virtual   = 480,
	.bits_per_pixel = 16,
	.red            = {6, 5, 0},
	.green          = {11, 5, 0},
	.blue           = {1, 5, 0},
	.activate       = FB_ACTIVATE_NOW,
	/* umm... why pixel dimension and physical dimension are merged? */
	/* we assume display is 280x210mm, 13.7in 4:3 */
	.width          = 280,
	.height         = 210,
	.vmode          = FB_VMODE_NONINTERLACED,
};

static int n64vifb_setcolreg(unsigned regno, unsigned red, unsigned green,
                           unsigned blue, unsigned transp,
                           struct fb_info *info)
{
	/*
	 *  Set a single color register. The values supplied have a 16 bit
	 *  magnitude.
	 *  Return != 0 for invalid regno.
	 */

	if (regno > 255)
		return 1;
	red >>= 11;
	green >>= 11;
	blue >>= 11;

	if (regno < 16) {
		((u32 *)info->pseudo_palette)[regno] =
			((red & 31) << 6) |
			((green & 31) << 11) |
			((blue & 31) << 1);
	}
	return 0;
}

static struct fb_ops n64vifb_ops = {
	.owner        = THIS_MODULE,
	.fb_setcolreg = n64vifb_setcolreg,
	/* Nintendo 64's frame buffer is on system RAM(RDRAM), we use sys_* instead of cfb_*. */
	.fb_fillrect  = sys_fillrect,
	.fb_copyarea  = sys_copyarea,
	.fb_imageblit = sys_imageblit,
};

static int n64vifb_init_device(struct device_node *dp)
{
	int r;
	u32 regs[2];
	u32 *reg;
	void *fb_start_vm, *fb_start_nocache;
	u32 fb_start_pm; /* N64 bus is 32bit, u32 instead of uintptr_t is ok. */
	struct fb_info *info;

	r = of_property_read_u32_array(dp, "reg", regs, 2);
	if (r) {
		printk(KERN_ERR "n64vifb: could not get reg u32 array. errno=%d\n", r);
		return r;
	}
	/* TODO ah... it works on regs[0]<512M(that is true in N64), if >=512M then...? if 64bit then...? */
	reg = ioremap_nocache(regs[0], regs[1]);
	if (!reg) {
		printk(KERN_ERR "n64vifb: could not get nocache area for reg\n");
		return -ENOMEM;
	}
	/* TODO should save "reg" here */

	// dma_alloc_writecombine...? dma_alloc_coherent...? __get_free_pages(GFP_KERNEL|__GFP_ZERO,get_order(vmem_size))???
	// We need physically contiguous memory, so can't use vmalloc family.
	fb_start_vm = kzalloc(n64vifb_fix.smem_len, GFP_KERNEL);
	if (!fb_start_vm) {
		printk(KERN_ERR "n64vifb: could not allocate frame buffer memory\n");
		kfree(fb_start_vm);
		return -ENOMEM;
	}

	fb_start_pm = virt_to_phys(fb_start_vm);

	/* TODO this too. (that is true on N64 too.) if >=512M then...? if 64bit then...? */
	/* TODO is it OK to ioremap_nocache for kalloc'ed area?? */
	fb_start_nocache = ioremap_nocache(fb_start_pm, n64vifb_fix.smem_len);
	if (!fb_start_nocache) {
		printk(KERN_ERR "n64vifb: could not get nocache area for frame buffer memory\n");
		kfree(fb_start_vm);
		return -ENOMEM;
	}

	n64vifb_fix.smem_start = (unsigned long)fb_start_nocache;

	info = framebuffer_alloc(sizeof(u32) * 16/* for 16-colors pseudo palette */, NULL);
	if (!info) {
		printk(KERN_ERR "n64vifb: could not allocate framebuffer info\n");
		kfree(fb_start_vm);
		return -ENOMEM;
	}

	/* initialize VI as 640x480 NTSC. from libdragon. */
	/* TODO this should use writel_be? */
	reg[0]  = 0x0001015e; // VI_CONTROL_REG: 16bit, gamma, gamma_dither, divot, aa_resamp_fetch_if_needed. 0x10000??
	reg[1]  = (u32)n64vifb_fix.smem_start & 0x007Fffff; // VI_DRAM_ADDR (phys-addr)
	reg[2]  = 0x00000280; // VI_WIDTH_REG (0d640=0x280)
	//reg[3]  = 0x00000200; // VI_INTR_REG
	//reg[4]  = 0x00000000; // VI_CURRENT_REG // writing clears VI intr.
	reg[5]  = 0x03e52239; // VI_BURST(TIMING?)_REG
	reg[6]  = 0x0000020c; // VI_V_SYNC_REG (0d524=0x20c)
	reg[7]  = 0x00000c15; // VI_H_SYNC_REG
	reg[8]  = 0x0c150c15; // VI_LEAP_REG(H_SYNC_LEAP?)
	reg[9]  = 0x006c02ec; // VI_H_START_REG 108..748
	reg[10] = 0x002301fd; // VI_V_START_REG  35..509
	reg[11] = 0x000e0204; // VI_V_BURST_REG  14..516
	reg[12] = 0x00000400; // VI_X_SCALE_REG subpx_off=0.0(2.10f) scaleup=1/1.0(2.10f)
	reg[13] = 0x02000800; // VI_Y_SCALE_REG subpx_off=0.5(2.10f) scaleup=1/2.0(2.10f)

	info->var = n64vifb_var; /* copy whole struct */
	info->fix = n64vifb_fix;
	info->fbops = &n64vifb_ops;
	info->flags = FBINFO_DEFAULT;
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->screen_base = (char *) n64vifb_fix.smem_start;

	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		printk(KERN_ERR "n64vifb: could not allocate color map\n");
		kfree(fb_start_vm);
		framebuffer_release(info);
		return -ENOMEM;
	}

	if (register_framebuffer(info) < 0) {
		printk(KERN_ERR "n64vifb: unable to register N64VI frame buffer\n");
		fb_dealloc_cmap(&info->cmap);
		kfree(fb_start_vm);
		framebuffer_release(info);
		return -EINVAL;
	}

	fb_info(info, "Nintendo 64 VI frame buffer %dx%dx%d at 0x%p.\n"
		,n64vifb_var.xres_virtual
		,n64vifb_var.yres_virtual
		,n64vifb_var.bits_per_pixel
		,(void*)n64vifb_fix.smem_start /* TODO this is unsigned long, could be casted to void-ptr?? */
		);
	return 0;
}

#if 0
static int n64vifb_probe(struct platform_device *dev)
{
	struct fb_info *info;

#ifndef CONFIG_NINTENDO64
	return -ENXIO;
#endif

	return 0;
}

static struct platform_driver n64vifb_driver = {
	.probe  = n64vifb_probe,
	.remove = 0, /* TODO impl */
	.driver = {
	    .name = "n64vifb",
	},
};

static struct platform_device n64vifb_device = {
	.name = "n64vifb",
};
#endif

int __init n64vifb_init(void)
{
	int ret = 0;
	struct device_node *dp;

	/* fb_get_options returns 1 if specified name is explicitly "off", or something bad (non-"offb" while ofonly) */
	if (fb_get_options("n64vifb", NULL))
		return -ENODEV;

	/* skip platform driver/device registering... */
#if 0
	ret = platform_driver_register(&n64vifb_driver);
	if (ret) {
		return ret;
	}

	ret = platform_device_register(&n64vifb_device);
	if (ret) {
		/* oops, registering device failed. unregister driver too. */
		platform_driver_unregister(&n64vifb_driver);
		return ret;
	}
#endif

	for (dp = NULL; (dp = of_find_compatible_node(dp, NULL, "nintendo,vi"));) {
		ret = n64vifb_init_device(dp);
		if (ret) {
			return ret;
		}
	}

	return ret;
}

module_init(n64vifb_init);
//module_exit(n64vifb_exit); /* TODO impl */
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nintendo 64 Video Frame Buffer Driver");
MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
