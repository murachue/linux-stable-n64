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
#include <linux/gfp.h>

enum n64pi_request_type { /* request types */
	N64PI_RTY_C2R_WORD, /* 32bit cart to ram */
	N64PI_RTY_R2C_WORD, /* 32bit ram to cart */
	N64PI_RTY_C2R_DMA, /* 64bit*n cart to ram */
	N64PI_RTY_R2C_DMA, /* 64bit*n ram to cart */
	N64PI_RTY_RESET, /* reset the PI */
	N64PI_RTY_GET_STATUS, /* get the PI status (no write on PI registers) */
	N64PI_RTY_ED64_ENABLE, /* enables ED64 regs */
	N64PI_RTY_ED64_DISABLE, /* disables ED64 regs if this is a last ena-dis pair */
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
	void (*on_error)(struct n64pi_request *req); /* is called on callee program error while processing this request, you must kfree or reuse req. (NOTE: this including ENOMEM, you should to mind that case.) */
	void *cookie; /* is a free field for on_complete. use for any you want. */

	uint32_t status; /* holds the status on just after complete of this request. TODO consider passing this by on_complete argument to allow register passing. */
};

struct n64pi; /* private struct of driver */

extern int
n64pi_request_async(struct n64pi *pi, struct n64pi_request *req);

extern int
n64pi_many_request_async(struct n64pi *pi, struct list_head *reqlist);

extern int
n64pi_wedge_request_async(struct n64pi *pi, struct n64pi_request *req);

extern int
n64pi_request_sync(struct n64pi *pi, struct n64pi_request *req);

extern void
n64pi_free_request(struct n64pi_request *req); /* provides convenience for on_error. */

static inline struct n64pi_request *
n64pi_alloc_request(gfp_t flags)
{
	struct n64pi_request *req;

	req = kzalloc(sizeof(*req), flags);
	if (!req) {
		return NULL;
	}

	INIT_LIST_HEAD(&req->node);

	return req;
}

/* TODO
extern struct n64pi_request *
n64pi_getreq_c2rw(uint32_t cart_address, void *ram_vaddress, void (*on_complete)(struct n64pi_request *req), void (*on_error)(struct n64pi_request *req), void *cookie);
extern struct n64pi_request *
n64pi_getreq_r2cw(uint32_t cart_address, void *ram_vaddress, uint32_t value, void (*on_complete)(struct n64pi_request *req), void (*on_error)(struct n64pi_request *req), void *cookie);
extern struct n64pi_request *
n64pi_getreq_c2rd(uint32_t cart_address, void *ram_vaddress, uint32_t length, void (*on_complete)(struct n64pi_request *req), void (*on_error)(struct n64pi_request *req), void *cookie);
extern struct n64pi_request *
n64pi_getreq_r2cd(uint32_t cart_address, void *ram_vaddress, uint32_t length, void (*on_complete)(struct n64pi_request *req), void (*on_error)(struct n64pi_request *req), void *cookie);
*/

#endif /* _MFD_N64PI_H_ */
