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

#define N64PI_ERROR_SUCCESS    0
#define N64PI_ERROR_BUSY       -1 /* n64pi_trybegin failed */
#define N64PI_ERROR_ADDRESSING -2 /* cart_addr is out of range */
#define N64PI_ERROR_DMAMAP     -3 /* kernel could not map DMA */
#define N64PI_ERROR_PIDMA      -4 /* PI reported an error */
#define N64PI_ERROR_NXIO       -5 /* not supported by cart (ex. ED64 ops on 64drive) */

/* will panic if recursively beginned */
extern void
n64pi_begin(struct n64pi *pi);

/* does not panic, for tty (that could called in other n64pi users) */
extern int
n64pi_trybegin(struct n64pi *pi);

/* will panic if not beginned */
extern void
n64pi_end(struct n64pi *pi);

/* will panic if not beginned */
extern int
n64pi_read_dma(struct n64pi *pi, void *ram_vaddr, uint32_t cart_addr, uint32_t length);

/* will panic if not beginned */
extern int
n64pi_write_dma(struct n64pi *pi, uint32_t cart_addr, void *ram_vaddr, uint32_t length);

/* will panic if not beginned */
/* TODO how about addressing error? but I don't want to store value into memory... introduce "value_on_error"? */
extern uint32_t
n64pi_read_word(struct n64pi *pi, uint32_t cart_addr);

/* TODO following unsafe-fast functions should be partially normal... wait-reseterror should be in caller.
 *      requires rewriting all mfd sub drivers... */

/* intended for timing-severe usecase (ex. ED64 sdmmc cmd->dat response reading) */
extern void
n64pi_write_word_unsafefast(struct n64pi *pi, uint32_t cart_addr, uint32_t value);

/* intended for timing-severe usecase (ex. ED64 sdmmc cmd->dat response reading) */
extern uint32_t
n64pi_read_word_unsafefast(struct n64pi *pi, uint32_t cart_addr);

/* will panic if not beginned */
extern int
n64pi_write_word(struct n64pi *pi, uint32_t cart_addr, uint32_t value);

/* will panic if not beginned */
extern void
n64pi_reset(struct n64pi *pi);

/* will panic if not beginned */
extern uint32_t
n64pi_get_status(struct n64pi *pi);

/* will panic if not beginned, or if not ed64_enable'd */
extern unsigned int
n64pi_ed64_regread(struct n64pi *pi, unsigned int regoff);

/* will panic if not beginned, or if not ed64_enable'd */
extern int
n64pi_ed64_regwrite(struct n64pi *pi, unsigned int value, unsigned int regoff);

/* intended for timing-severe usecase (ex. ED64 sdmmc cmd->dat response reading) */
extern unsigned int
n64pi_ed64_regread_unsafefast(struct n64pi *pi, unsigned int regoff);

/* intended for timing-severe usecase (ex. ED64 sdmmc cmd->dat response reading) */
extern void
n64pi_ed64_regwrite_unsafefast(struct n64pi *pi, unsigned int value, unsigned int regoff);

/* will panic if not beginned */
extern int
n64pi_ed64_enable(struct n64pi *pi);

/* will panic if not beginned */
extern int
n64pi_ed64_disable(struct n64pi *pi);

#endif /* _MFD_N64PI_H_ */
