/*
 *  ed64-sdmmc.c - driver for EverDrive-64 SD/MMC slot
 *                 (derived from sdricoh_cs.c)
 *
 *  Copyright (C) 2018 Murachue <murachue+github@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/* TODO OpenFirmware DeviceTree support? */
#ifdef CONFIG_USE_OF
#error n64cart does not support device tree yet
#endif

/*
#define DEBUG
#define VERBOSE_DEBUG
*/
#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/module.h>
//#include <linux/ioport.h>
//#include <linux/scatterlist.h>
#include <linux/platform_device.h>

#include <linux/io.h>

#include <linux/mmc/host.h>

#include <linux/mfd/n64pi.h>

/* registers & fields */
#define ED64_REG_CFG     0x00
#define ED64_REG_STATUS  0x04
#define ED64_REG_DMALEN  0x08
#define ED64_REG_DMAADDR 0x0C
#define ED64_REG_DMACFG  0x14
#define ED64_REG_SPI     0x18
#define ED64_REG_SPICFG  0x1C

#define ED64_DMACFG_SD2RAM 1
#define ED64_DMACFG_RAM2SD 2

#define ED64_STATUS_DMABUSY (1 << 0)
#define ED64_STATUS_DMATOUT (1 << 1)
#define ED64_STATUS_TXE     (1 << 2)
#define ED64_STATUS_RXF     (1 << 3)
#define ED64_STATUS_SPI     (1 << 4)

#define ED64_SPICFG_SPD0  (1 << 0)
#define ED64_SPICFG_SPD1  (1 << 1)
#define ED64_SPICFG_SS    (1 << 2) /* raw slave_select, in SD/MMC this should be "set" (|0x04 = deasserted-SS). */
#define ED64_SPICFG_RD    (1 << 3) /* 0=write 1=read */
#define ED64_SPICFG_DAT   (1 << 4) /* 0=cmd 1=dat */
#define ED64_SPICFG_PROBE (1 << 5) /* cmd1=1bit_left_shift_latch cmd0=8bit_latch dat1=4lines-1bit-data dat0=4lines-8bit-data */

#define ED64_SPICFG_SPEED_MASK (ED64_SPICFG_SPD1 | ED64_SPICFG_SPD0)
#define ED64_SPICFG_SPEED_INIT ED64_SPICFG_SPD1
#define ED64_SPICFG_SPEED_25   ED64_SPICFG_SPD0
#define ED64_SPICFG_SPEED_50   0

/* timeouts */
#define CMD_TIMEOUT       2048
#define TRANSFER_TIMEOUT  65535

/* mmc privdata */
struct ed64mmc_host {
	struct device *dev;
	struct n64pi *pi; // n64pi arbitrator bound to the parent mfd device
	uint32_t rombase; // ROM area that is used for ED64 DMA read/write (2048 bytes align required by ED64 DMA!)
	uint32_t spicfg; // holds last ED64_REG_SPICFG value (for modify-write)
	uint32_t datwidth; // holds DAT width, 0=1bit or 1=4bit.
};

/* TODO merge with n64cart? */
// FIXME bad n64pi_read_word interface... no error can be returned.
static void ed64_dummyread(struct n64pi *pi) {
	(void)n64pi_read_word(pi, 0x08040000 + 0x00);
}

static uint32_t ed64_regread(struct n64pi *pi, unsigned int regoff) {
	// FIXME bad n64pi_read_word interface... no error can be returned.
	ed64_dummyread(pi); // dummy read required!!

	// FIXME bad n64pi_read_word interface... no error can be returned.
	return n64pi_read_word(pi, 0x08040000 + regoff);
}

static int ed64_regwrite(struct n64pi *pi, uint32_t value, unsigned int regoff) {
	int ret;

	// FIXME bad n64pi_read_word interface... no error can be returned.
	ed64_dummyread(pi); // dummy read required!!

	if ((ret = n64pi_write_word(pi, 0x08040000 + regoff, value)) != N64PI_ERROR_SUCCESS) {
		pr_err("%s: ed64_regwrite failed for %08x=%08x (%d)\n", __func__, regoff, value, ret);
		return ret;
	}

	return N64PI_ERROR_SUCCESS;
}

/* TODO use linux/crc-ccitt.h:crc_ccitt_byte/crc_ccitt but this is little-endian crc, and for single line... */
static void crc16_dat4_update(int (*crcs)[4], int byte) {
	int i, d;

	/* ED64 full 4bit DAT is: {DAT3 DAT2 DAT1 DAT0}[0] {DAT3 DAT2 DAT1 DAT0}[1] */
	for (i = 0; i < 2; i++) {
		for (d = 3; d >= 0; d--) {
			int crc = (*crcs)[d];
			crc ^= (byte >> 7) & 1;
			crc <<= 1;
			if (crc & 0x10000) {
				crc ^= 0x11021;
			}
			(*crcs)[d] = crc;
			byte <<= 1;
		}
	}
}

/* TODO ED64 SPI DMA support */
static int ed64mmc_block_read(struct ed64mmc_host *host, u8 *buf, int len)
{
	struct n64pi *pi = host->pi;
	int crcs[4] = {0};
	int readcrcs[4] = {0};
	int i, d;

	/* 4bit data read */
	ed64_regwrite(pi, ED64_SPICFG_PROBE | ED64_SPICFG_DAT | ED64_SPICFG_RD | host->spicfg, ED64_REG_SPICFG);

	/* seek for start bit */
	for (i = TRANSFER_TIMEOUT; i; i--) {
		ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock, with previously received as 0x?F */
		if ((ed64_regread(pi, ED64_REG_SPI) & 0xF1) == 0xF0) {
			break;
		}
	}
	if (i == 0) {
		return -ETIMEDOUT;
	}

	/* 8bit data read */
	ed64_regwrite(pi, ED64_SPICFG_DAT | ED64_SPICFG_RD | host->spicfg, ED64_REG_SPICFG);

	/* read data */
	for (i = 0; i < len; i++) {
		u32 byte;

		ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock */
		byte = ed64_regread(pi, ED64_REG_SPI);
		*buf++ = byte;
		crc16_dat4_update(&crcs, byte);
	}

	/* read DAT-individual crc16s (from ED64_REG_SPI perspective, seen as interleaved) */
	for (i = 0; i < 8; i++) {
		u32 byte;
		int j;

		ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock */
		byte = ed64_regread(pi, ED64_REG_SPI);
		for (j = 0; j < 2; j++) {
			for (d = 3; d >= 0; d--) {
				readcrcs[d] <<= 1;
				readcrcs[d] |= (byte >> 7) & 1;
				byte <<= 1;
			}
		}
	}

	/* verify crcs */
	for (d = 0; d < 4; d++) {
		if (crcs[d] != readcrcs[d]) {
			return -EIO;
		}
	}

	return 0;
}

static int ed64mmc_block_write(struct ed64mmc_host *host, const u8 *buf, int len)
{
	struct n64pi *pi = host->pi;
	int crcs[4] = {0};
	int i, j, d, resp;

	/* 8bit data write */
	ed64_regwrite(pi, ED64_SPICFG_DAT | host->spicfg, ED64_REG_SPICFG);

	/* stable then put start bit on 4 DATs */
	ed64_regwrite(pi, 0xFF, ED64_REG_SPI);
	ed64_regwrite(pi, 0xF0, ED64_REG_SPI);

	/* write data with crc16 calc */
	for (i = 0; i < len; i++) {
		u32 byte = *buf++;
		ed64_regwrite(pi, byte, ED64_REG_SPI);
		crc16_dat4_update(&crcs, byte);
	}

	/* write DAT-individual crc16 */
	for (i = 0; i < 8; i++) {
		u32 byte = 0;
		for (j = 0; j < 2; j++) {
			for (d = 3; d >= 0; d++) {
				byte <<= 1;
				byte |= (crcs[d] >> 15) & 1;
				crcs[d] <<= 1;
			}
		}
		ed64_regwrite(pi, byte, ED64_REG_SPI);
	}

	/* send stop bit(s) */
	/* 4bit data write */
	ed64_regwrite(pi, ED64_SPICFG_PROBE | ED64_SPICFG_DAT | host->spicfg, ED64_REG_SPICFG);
	ed64_regwrite(pi, 0xFF, ED64_REG_SPI);

	/* wait for write completion at card */
	/* 4bit data read */
	ed64_regwrite(pi, ED64_SPICFG_PROBE | ED64_SPICFG_DAT | ED64_SPICFG_RD | host->spicfg, ED64_REG_SPICFG);

	/* seek for DAT0 response start bit...?? TODO where is this spec? I could find only SPI control token... but this is SD mode. */
	for (i = TRANSFER_TIMEOUT; i; i--) {
		ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock, with previously received as 0x?F */
		if ((ed64_regread(pi, ED64_REG_SPI) & 1) == 0) {
			break;
		}
	}
	if (i == 0) {
		return -ETIMEDOUT;
	}

	/* receive DAT0 response...?? */
	resp = 0;
	for (i = 0; i < 3; i++) {
		resp <<= 1;
		ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock, with previously received as 0x?F */
		resp |= ed64_regread(pi, ED64_REG_SPI) & 1;
	}

	if (resp == 0x05) {
		/* wrong sent CRC... */
		return -EIO; /* TODO what should be returned? */
	} else if (resp == 0x06) { /* TODO really?? */
		/* write error... */
		return -EIO;
	} else if (resp != 0x02) {
		/* something unexpected... */
		return -EILSEQ; /* TODO is there EUNKNOWNERROR? */
	}

	/* wait for end of busy...?? */
	/* 8bit data read */
	ed64_regwrite(pi, ED64_SPICFG_DAT | ED64_SPICFG_RD | host->spicfg, ED64_REG_SPICFG);
	ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock */
	for (i = TRANSFER_TIMEOUT * 2; i; i--) { /* TODO twice is just ED64 does... have some mean? */
		ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock, with previously received as 0x?F */
		if ((ed64_regread(pi, ED64_REG_SPI) & 0xFF) == 0xFF) {
			break;
		}
	}
	if (i == 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

/* already 1bit left shifted crc7. TODO use linux/crc7.h:crc7_be_byte (this is 1ls too); dont forget selects CRC7 */
static int crc7_update(int crc, int byte) {
	int i;

	crc ^= byte;
	for (i = 0; i < 8; i++) {
		crc <<= 1;
		if (crc & 0x100) {
			crc ^= 0x112;
		}
	}

	return crc;
}

static void ed64mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ed64mmc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = cmd->data;
	struct device *dev = host->dev;
	struct n64pi *pi = host->pi;

	dev_dbg(dev, "=============================\n");
	dev_dbg(dev, "ed64mmc_request opcode=%i\n", cmd->opcode);

	n64pi_begin(pi);

	n64pi_ed64_enable(pi);

	/* 8bit command write */
	ed64_regwrite(pi, host->spicfg, ED64_REG_SPICFG);

	{
		u8 opbyte = 0x40 | cmd->opcode;
		u32 arg = cmd->arg;
		int i, crc;

		ed64_regwrite(pi, opbyte, ED64_REG_SPI);
		crc = crc7_update(0, opbyte);

		for (i = 0; i < 4; i++) {
			int abyte = (arg >> 24) & 0xFF;
			ed64_regwrite(pi, abyte, ED64_REG_SPI);
			crc = crc7_update(crc, abyte);
			arg <<= 8;
		}

		ed64_regwrite(pi, crc | 1, ED64_REG_SPI);
	}

	/* read response buffer */
	if (cmd->flags & MMC_RSP_PRESENT) {
		/* read response */
		unsigned int bits = 0xFF; /* fill with all 1s is important */
		int try;

		/* 1bit command read */
		ed64_regwrite(pi, ED64_SPICFG_PROBE | ED64_SPICFG_RD | host->spicfg, ED64_REG_SPICFG);

		/* seek for start-bit and transmission-bit */
		for (try = CMD_TIMEOUT; try; try--) {
			ed64_regwrite(pi, bits, ED64_REG_SPI);
			bits = ed64_regread(pi, ED64_REG_SPI); /* data magically shifts in from lsb */
			if ((bits & 0xC0) == 0) {
				/* found start-bit and transmission-bit! */
				break;
			}
		}

		if (try == 0) {
			cmd->error = -ETIMEDOUT;
		} else if (bits != cmd->opcode) {
			cmd->error = -EILSEQ;
		}

		/* receive response and crc if not timed out (but incl. invalid opcode echo) */
		if (try != 0) {
			int rwords = (cmd->flags & MMC_RSP_136) ? 4 : 1;
			int i, j, crc = (cmd->flags & MMC_RSP_136) ? 0 : crc7_update(0, bits);

			/* 8bit command read */
			ed64_regwrite(pi, ED64_SPICFG_RD | host->spicfg, ED64_REG_SPICFG);

			/* read response word(s) */
			for (i = 0; i < rwords; i++) {
				uint32_t word = 0;
				for (j = 0; j < 4; j++) {
					int rbyte;
					ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock */
					rbyte	= ed64_regread(pi, ED64_REG_SPI) & 0xFF;
					word = (word << 8) | rbyte;
					crc = crc7_update(crc, rbyte);
				}
				cmd->resp[i] = word;
			}

			/* read crc and test it if required */
			{
				int trail;

				ed64_regwrite(pi, 0xFF, ED64_REG_SPI); /* write something to clock */
				trail	= ed64_regread(pi, ED64_REG_SPI) & 0xFF;

				if ((cmd->flags & MMC_RSP_CRC) && (trail != (crc | 1))) {
					/* crc error */
					cmd->error = -EIO;
				}
			}
		}
	}

	/* transfer data */
	if (data && cmd->error == 0) {
		int i;

		dev_dbg(dev, "transfer: blksz %i blocks %i sg_len %i sg length %i\n",
		        data->blksz, data->blocks, data->sg_len, data->sg->length);

		for (i = 0; i < data->blocks; i++) {
			size_t len = data->blksz;
			u8 *buf;
			struct page *page;
			int result;
			page = sg_page(data->sg);

			buf = kmap(page) + data->sg->offset + (len * i);
			if (data->flags & MMC_DATA_READ) {
				result = ed64mmc_block_read(host, buf, len);
			} else {
				result = ed64mmc_block_write(host, buf, len);
			}
			kunmap(page);
			flush_dcache_page(page);
			if (result) {
				dev_err(dev, "ed64mmc_request: cmd %i block transfer failed\n", cmd->opcode);
				cmd->error = result;
				break;
			} else
				data->bytes_xfered += len;
		}
	}

	n64pi_ed64_disable(pi);

	n64pi_end(pi);

	mmc_request_done(mmc, mrq);
	dev_dbg(dev, "=============================\n");
}

static void ed64mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ed64mmc_host *host = mmc_priv(mmc);
	dev_dbg(host->dev, "set_ios\n");

	if (ios->power_mode != MMC_POWER_ON) {
		/* in UNDEFINED, OFF, and UP state are ignored. */
		return;
	}

	/* TODO will MMC_BUS_WIDTH_8 come? I don't say 8bit-support at caps. */
	if (ios->bus_width == MMC_BUS_WIDTH_4) {
		host->datwidth = 1;
	} else {
		host->datwidth = 0;
	}

	if (ios->clock < 25000000) {
		host->spicfg &= ~ED64_SPICFG_SPEED_MASK;
		host->spicfg |= ED64_SPICFG_SPEED_INIT;
	} else if (ios->clock < 50000000) {
		host->spicfg &= ~ED64_SPICFG_SPEED_MASK;
		host->spicfg |= ED64_SPICFG_SPEED_25;
	} else {
		host->spicfg &= ~ED64_SPICFG_SPEED_MASK;
		host->spicfg |= ED64_SPICFG_SPEED_50;
	}

	/* note: both MMC_CS_* and ED64_SPICFG_SS are raw (negated) value. */
	if (ios->chip_select == MMC_CS_HIGH) {
		host->spicfg |= ED64_SPICFG_SS;
	} else if (ios->chip_select == MMC_CS_LOW) {
		host->spicfg &= ~ED64_SPICFG_SS;
	} /* note: DONTCARE literally does not care. */

	ed64_regwrite(host->pi, host->spicfg, ED64_REG_SPICFG);
}

static struct mmc_host_ops ed64mmc_ops = {
	.request = ed64mmc_request,
	.set_ios = ed64mmc_set_ios,
};

/* initialize the control and register it to the mmc framework */
static int ed64mmc_probe(struct platform_device *pdev)
{
	int result = 0;
	struct mmc_host *mmc = NULL;
	struct ed64mmc_host *host = NULL;
	struct device * const dev = &pdev->dev;

	dev_info(dev, "EverDrive-64 SD/MMC host driver\n");

	/* allocate mmc and put into privdata */
	mmc = mmc_alloc_host(sizeof(struct ed64mmc_host), &pdev->dev); /* note: mmc_alloc_host's dev seems read-only, but it is not const, so prepare to be modified. (not using &*dev.) */
	if (!mmc) {
		dev_err(dev, "mmc_alloc_host failed\n");
		result = -ENOMEM;
		goto err;
	}
	dev_set_drvdata(&pdev->dev, mmc);
	host = mmc_priv(mmc);

	host->dev = dev;
	host->pi = dev_get_drvdata(pdev->dev.parent); /* TODO dev.parent->parent is n64pi?? ipaq-micro-leds */
	host->rombase = (0x04000000-0x1000); // note: just lower ed64tty. TODO configurable or IORESOURCE_MEM/IO?
	host->spicfg = ED64_SPICFG_SS | ED64_SPICFG_SPEED_INIT;
	host->datwidth = 0;

	mmc->ops = &ed64mmc_ops;

	/* FIXME: how speed it is? */
	mmc->f_min = 400000; /* TODO how speed of SPEED_INIT is? mmc core use "100000~400000" */
	mmc->f_max = 25000000; /* TODO "50" is defined but not used. be conservative. */
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34; /* TODO I don't know about voltage on EverDrive... */
	mmc->caps |= MMC_CAP_4_BIT_DATA; /* TODO I don't know much about mmc subsystem... stay left as sdricoh. */

	mmc->max_seg_size = 4 * 512;
	mmc->max_blk_size = 512;

	result = mmc_add_host(mmc);

	if (result == 0) {
		dev_dbg(dev, "mmc host registered\n");
		return 0;
	}

err:
	if (mmc)
		mmc_free_host(mmc);

	return result;
}

static int ed64mmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = dev_get_drvdata(&pdev->dev);

	dev_dbg(&pdev->dev, "remove\n");

	/* remove mmc host */
	if (mmc) {
		/*struct ed64mmc_host *host = mmc_priv(mmc);*/
		mmc_remove_host(mmc);
		mmc_free_host(mmc);
	}

	return 0;
}

static struct platform_driver ed64mmc_driver = {
	.probe = ed64mmc_probe,
	.remove = ed64mmc_remove,
	.driver = {
		.name = "n64pi-ed64mmc",
	},
};
module_platform_driver(ed64mmc_driver);

MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
MODULE_DESCRIPTION("EverDrive-64 Secure Digital Interface driver");
MODULE_LICENSE("GPL");
