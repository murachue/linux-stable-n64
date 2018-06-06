/*
 * Header file for the Nintendo 64 PI MFD driver and its users
 * Copyright (c) 2018 Murachue <murachue+github@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#ifndef _MFD_N64PI_H_
#define _MFD_N64PI_H_

#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>

enum n64pi_request_type { /* request types */
	N64PI_RTY_C2R_WORD, /* 32bit cart to ram */
	N64PI_RTY_R2C_WORD, /* 32bit ram to cart */
	N64PI_RTY_C2R_DMA, /* 64bit*n cart to ram */
	N64PI_RTY_R2C_DMA, /* 64bit*n ram to cart */
};

struct n64pi_request { /* represents a PI command request */
	struct list_head node; /* is connected to the queue if request gets queued */

	enum n64pi_request_type type; /* means what this request want to do */
	uint32_t cart_address; /* where (value|data at ram_vaddress) will be (read|written) */
	uint32_t value; /* holds the value will be written/that was read (valid only if WORD request) */
		/* TODO union with ram_vaddress? providing "value" for convenience. (no need to allocate a 32bit somewhere else.) */
	void *ram_vaddress; /* points to the data will be DMA'd (valid only if DMA request), in virtual address. */
	uint32_t length; /* to be transferred (valid only if DMA request) */

	void (*on_complete)(struct n64pi_request *req); /* is called on completed this request, you must kfree or reuse req. (NOTE: this is called in interrupt context) */
	void *cookie; /* is a free field for on_complete. use for any you want. */

	uint32_t status; /* holds the status on just after complete of this request */
};

struct n64pi { /* represents the driver status of the PI device */
	struct device *dev; /* is a corresponding platform device */
	void __iomem *regbase; /* is a base virtual address of PI DMA registers (ie. 0xA4600000) */
	void __iomem *membase; /* is a base virtual address of CPU word access to PI including SRAM/64DD/ROM area (ie. 0xA5000000) */
	spinlock_t lock; /* is a lock for this state container */
	struct list_head queue; /* is a request queue */
	struct n64pi_request *curreq; /* is a current processing request */

	/* TODO move to n64pi_request? but it is bloating a 32bit word more... */
	dma_addr_t curbusaddr; /* holds mapped device address for DMA (valid only while DMAing for curreq) */
};

extern int
n64pi_request_async(struct n64pi *pi, struct n64pi_request *req);

extern int
n64pi_request_sync(struct n64pi *pi, struct n64pi_request *req);

#endif /* _MFD_N64PI_H_ */
