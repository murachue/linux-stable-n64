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

struct n64pi; /* private struct of driver */

typedef void (*n64pi_before_dma_t)(struct n64pi *pi, void *cookie);
typedef void (*n64pi_on_interrupt_t)(struct n64pi *pi, void *cookie, uint32_t status);

#define N64PI_ERROR_SUCCESS    0
#define N64PI_ERROR_ADDRESSING -1
#define N64PI_ERROR_DMAMAP     -2
#define N64PI_ERROR_PIDMA      -3
#define N64PI_ERROR_BUSY       -4
#define N64PI_ERROR_NOMEM      -5

extern int
n64pi_blocking_read_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length);

extern int
n64pi_nonblock_read_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length, n64pi_before_dma_t before_dma, n64pi_on_interrupt_t on_interrupt, void *cookie);

extern int
n64pi_blocking_write_dma(struct n64pi *pi, uint32_t cart_addr, void *ram_vaddr, uint32_t length);

extern int
n64pi_nonblock_write_dma(struct n64pi *pi, uint32_t cart_addr, void *ram_vaddr, uint32_t length, n64pi_before_dma_t before_dma, n64pi_on_interrupt_t on_interrupt, void *cookie);

/* TODO how about addressing error? but I don't want to store value into memory... introduce "value_on_error"? */
extern uint32_t
n64pi_blocking_read_word(struct n64pi *pi, uint32_t cart_addr);

extern int
n64pi_blocking_write_word(struct n64pi *pi, uint32_t cart_addr, uint32_t value);

extern void
n64pi_reset(struct n64pi *pi);

extern uint32_t
n64pi_get_status(struct n64pi *pi);

extern int
n64pi_blocking_ed64_enable(struct n64pi *pi);

extern int
n64pi_blocking_ed64_disable(struct n64pi *pi);

#endif /* _MFD_N64PI_H_ */
