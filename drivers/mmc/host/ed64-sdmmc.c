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
#error ed64-sdmmc does not support device tree yet
#endif

/* enable dev_dbg/pr_debug? */
//#define DEBUG

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

#define ED64_SPICFG_SPD0 (1 << 0)
#define ED64_SPICFG_SPD1 (1 << 1)
#define ED64_SPICFG_SS   (1 << 2) /* raw slave_select, in SD/MMC this should be "set" (|0x04 = deasserted-SS). */
#define ED64_SPICFG_RD   (1 << 3) /* 0=write 1=read */
#define ED64_SPICFG_DAT  (1 << 4) /* 0=cmd 1=dat */
#define ED64_SPICFG_1BIT (1 << 5) /* cmd: 1=1bit_left_shift_latch=1bit 0=8bit_latch=8bits dat: 1=4lines-1bit-data=4bits 0=4lines-2bit-data=8bits */

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
	uint32_t lastopcode; // holds last opcode (mainly for testing CMD55(prefix of ACMD))
};

static void ed64_spiwait(struct n64pi *pi) {
	while (n64pi_ed64_regread_unsafefast(pi, ED64_REG_STATUS) & ED64_STATUS_DMABUSY) /*nothing*/ ; /* note: Yes, check DMABUSY bit. */
}

static void ed64_spiwrite(struct n64pi *pi, uint32_t value) {
	n64pi_ed64_regwrite_unsafefast(pi, value, ED64_REG_SPI);

	ed64_spiwait(pi); /* wait above write */
}
/* note: ED64_SPICFG_RD must be set in spicfg */
static uint32_t ed64_spiread(struct n64pi *pi, uint32_t value) {
	ed64_spiwrite(pi, value);

	return n64pi_ed64_regread_unsafefast(pi, ED64_REG_SPI);
}

static void ed64_spicfg(struct ed64mmc_host *host, uint32_t cfg, int cmdwithdat) {
	struct n64pi *pi = host->pi;
	int spicfg = cfg | host->spicfg;

	/* ED64 cannot latch both CMD and DAT*. To avoid losing beginning of DAT*, its preceding CMD must be sent with faster clock. */
	/* TODO I think this highly depends on SD card... */
	if (cmdwithdat && ((spicfg & ED64_SPICFG_SPEED_MASK) == ED64_SPICFG_SPEED_INIT)) {
		spicfg &= ~ED64_SPICFG_SPEED_MASK;
		spicfg |= ED64_SPICFG_SPEED_25;
	}

	n64pi_ed64_regwrite_unsafefast(pi, spicfg, ED64_REG_SPICFG);
}

/* TODO use linux/crc-ccitt.h:crc_ccitt_byte/crc_ccitt but it is little-endian crc, and for single line... */
static void crc16_2dat4_update(int (*crcs)[4], int byte) {
	int i, d;

	/* ED64 full 8bit DAT is (from MSbit): {DAT3 DAT2 DAT1 DAT0}[0] {DAT3 DAT2 DAT1 DAT0}[1] */
	for (i = 0; i < 2; i++) {
		for (d = 3; d >= 0; d--) {
			int crc = (*crcs)[d];
			crc ^= ((byte >> 7) & 1) << 15;
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
	int onebit = (host->datwidth == 0);
	int crcs[4] = {0};
	int readcrcs[4] = {0};
	int i, d;
int starttime;

	/* 4lines-1bit data read */
	ed64_spicfg(host, ED64_SPICFG_1BIT | ED64_SPICFG_DAT | ED64_SPICFG_RD, 0);

	/* seek for start bit */
	for (i = TRANSFER_TIMEOUT; i; i--) {
		/* if DAT0(0-3 as spec...) is low: it's start-bit! */
		if ((ed64_spiread(pi, 0xFF/* previously received as 0x?F */) & 1) == 0) {
			break;
		}
	}
	if (i == 0) {
		return -ETIMEDOUT;
	}
starttime=TRANSFER_TIMEOUT-i;

	/* 4lines-2bit data read */
	ed64_spicfg(host, ED64_SPICFG_DAT | ED64_SPICFG_RD, 0);

	/* read data */
	{
		int cnt = len * (onebit ? 4 : 1); /* in 1bit mode, required 4 times. */
		unsigned int bbuf = 0;
		u8 *p = buf; /* FIXME DEBUG keep buf to dump on panic */

		for (i = 0; i < cnt; i++) {
			u32 byte;

			byte = ed64_spiread(pi, 0xFF/* something to clock */);
			if (onebit) {
				bbuf <<= 2;
				bbuf |= (((byte >> 4) & 1) << 1) | (byte & 1);
				if (i % 4 == 3) {
					*p++ = bbuf; /* write low 8bit */
				}
			} else {
				*p++ = byte;
			}
			crc16_2dat4_update(&crcs, byte);
		}
	}

	/* read DAT-individual crc16s (from ED64_REG_SPI perspective, seen as interleaved) */
	for (i = 0; i < 8; i++) {
		u32 byte;
		int j;

		byte = ed64_spiread(pi, 0xFF/* something to clock */);
		for (j = 0; j < 2; j++) {
			for (d = 3; d >= 0; d--) {
				readcrcs[d] <<= 1;
				readcrcs[d] |= (byte >> 7) & 1;
				byte <<= 1;
			}
		}
	}

	/* TODO read end bit? */

	/* verify crcs */
	for (d = 0; d < (onebit ? 1 : 4); d++) {
		if (crcs[d] == readcrcs[d]) {
			continue;
		}

		/* pr_debug */
		if (onebit) {
			panic("ed64mmcbr: crc1 mismatch e=%04x a=%04x @%p st=%d\n", readcrcs[0], crcs[0], buf, starttime);
		} else {
			panic("ed64mmcbr: crc4 mismatch e=%04x.%04x.%04x.%04x a=%04x.%04x.%04x.%04x\n",
							 readcrcs[0], readcrcs[1], readcrcs[2], readcrcs[3],
							 crcs[0], crcs[1], crcs[2], crcs[3]
							);
		}
		return -EIO;
	}

	return 0;
}

static int ed64mmc_block_write(struct ed64mmc_host *host, const u8 *buf, int len)
{
	struct n64pi *pi = host->pi;
	int crcs[4] = {0};
	int i, j, d, resp;

	/* 8bit data write */
	ed64_spicfg(host, ED64_SPICFG_DAT, 0);

	/* stable then put start bit on 4 DATs */
	ed64_spiwrite(pi, 0xFF);
	ed64_spiwrite(pi, 0xF0);

	/* TODO support 1-bit width */
	/* write data with crc16 calc */
	for (i = 0; i < len; i++) {
		u32 byte = *buf++;
		ed64_spiwrite(pi, byte);
		crc16_2dat4_update(&crcs, byte);
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
		ed64_spiwrite(pi, byte);
	}

	/* send stop bit(s) */
	/* 4lines-1bit data write */
	ed64_spicfg(host, ED64_SPICFG_1BIT | ED64_SPICFG_DAT, 0);
	ed64_spiwrite(pi, 0xFF);

	/* wait for write completion at card */
	/* 4lines-1bit data read */
	ed64_spicfg(host, ED64_SPICFG_1BIT | ED64_SPICFG_DAT | ED64_SPICFG_RD, 0);

	/* seek for DAT0 response start bit...?? TODO where is this spec? I could find only SPI control token... but this is SD mode. */
	for (i = TRANSFER_TIMEOUT; i; i--) {
		/* DAT0 is low: complete...??? */
		if ((ed64_spiread(pi, 0xFF/* previously received as 0x?F */) & 1) == 0) {
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
		resp |= ed64_spiread(pi, 0xFF/* previously received as 0x?F */) & 1;
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
	ed64_spicfg(host, ED64_SPICFG_DAT | ED64_SPICFG_RD, 0);
	ed64_spiwrite(pi, 0xFF); /* write something to clock...?? */
	for (i = TRANSFER_TIMEOUT * 2; i; i--) { /* TODO twice is just ED64 does... have some mean? */
		if ((ed64_spiread(pi, 0xFF/* previously received as 0x?F */) & 0xFF) == 0xFF) {
			break;
		}
	}
	if (i == 0) {
		return -ETIMEDOUT;
	}

	return 0;
}

/* already 1bit left shifted crc7. TODO use linux/crc7.h:crc7_be_byte (this is 1ls too); dont forget selects CRC7 */
/* NOTE only lower 8bit(or more concretely, [7:1]bit) is valid, and other are undefined. you must care [(63|31):8] when use. */
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

static void ed64mmc_send_command(struct ed64mmc_host *host, struct mmc_command *cmd, int cmdwithdat)
{
	struct n64pi *pi = host->pi;

	/* 8bit command write */
	ed64_spicfg(host, 0, cmdwithdat);

	{
		u8 opbyte = 0x40 | cmd->opcode;
		u32 arg = cmd->arg;
		int i, crc;

		ed64_spiwrite(pi, 0xFF); /* dummy clock */

		ed64_spiwrite(pi, opbyte);
		crc = crc7_update(0, opbyte);

		for (i = 0; i < 4; i++) {
			int abyte = (arg >> 24) & 0xFF;
			ed64_spiwrite(pi, abyte);
			crc = crc7_update(crc, abyte);
			arg <<= 8;
		}

//		dev_dbg(dev, "crc=%02X\n", crc | 1);
		ed64_spiwrite(pi, crc | 1);
	}
}

/* returns a byte received, or minus value if finding start bit timed out */
static int ed64mmc_recv_cmd_start(struct ed64mmc_host *host, int cmdwithdat)
{
	struct n64pi *pi = host->pi;
	int try, bits = 0xFF; /* fill with all 1s is important */

	/* 1bit command read */
	ed64_spicfg(host, ED64_SPICFG_1BIT | ED64_SPICFG_RD, cmdwithdat);

	/* seek for start-bit and transmission-bit */
	for (try = CMD_TIMEOUT; try; try--) {
		//			u32 obits = bits;
		bits = ed64_spiread(pi, bits/* previously received */); /* data magically shifts in from lsb */
		if ((bits & 0xC0) == 0) {
			/* found start-bit and transmission-bit! */
			//				dev_dbg(dev, "start-bit found: %08X->%08X\n", obits, bits);
			/* convert into byte (strip out ED64 status in high halfword) to comparable with cmd->opcode */
			return bits & 0xFF;
		}
	}

	return -1;
}

/* for non-data commands. */
static void ed64mmc_recv_cmd(struct ed64mmc_host *host, struct mmc_command *cmd)
{
	struct n64pi *pi = host->pi;
	int bits;

	bits = ed64mmc_recv_cmd_start(host, 0);

	if (bits < 0) {
		if (cmd->opcode != 0) {
			cmd->error = -ETIMEDOUT;
		}
	} else if ((cmd->flags & MMC_RSP_OPCODE) && (bits != cmd->opcode)) {
		pr_debug("mis opcode echo: exp=%02X act=%02X\n", cmd->opcode, bits);
		cmd->error = -EILSEQ;
	}

	pr_debug("expect response or cmd0; bits=%d first_err=%d\n", bits, cmd->error);

	/* don't receive following bytes if timed out (not including invalid opcode echo) */
	if (bits < 0) {
		return;
	}

	{
		int rwords = (cmd->flags & MMC_RSP_136) ? 4 : 1;
		int i, j, crc = 0, ncrc = (cmd->flags & MMC_RSP_136) ? 0 : crc7_update(0, bits);

		/* 8bit command read */
		ed64_spicfg(host, ED64_SPICFG_RD, 0);

		/* read response word(s) */
		for (i = 0; i < rwords; i++) {
			uint32_t word = 0;
			for (j = 0; j < 4; j++) {
				u32 rbyte;
				rbyte	= ed64_spiread(pi, 0xFF/* something to clock */) & 0xFF;
				word = (word << 8) | rbyte;
				crc = ncrc;
				ncrc = crc7_update(ncrc, rbyte);
			}
			cmd->resp[i] = word;
		}

		pr_debug(" response0=%08X\n", cmd->resp[0]);

		/* read crc and test it if required */
		{
			int trail;

			trail	= ed64_spiread(pi, 0xFF/* something to clock */) & 0xFF;

			/* note: R3 runs this but useless. */
			if (cmd->flags & MMC_RSP_136) {
				trail = cmd->resp[3] & 0xFF;
				/* note: crc must be kept as previous of ncrc. */
			} else {
				/* advance crc (including last response byte) */
				crc = ncrc;
			}

			if ((cmd->flags & MMC_RSP_CRC) && (trail != (crc | 1))) {
				/* crc error */
				cmd->error = -EIO;
				pr_debug(" found crc error: expect=%02X actual=%02X\n", crc | 1, trail);
			}
		}
	}
}

static void ed64mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ed64mmc_host *host = mmc_priv(mmc);
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = cmd->data;
	struct device *dev = host->dev;
	struct n64pi *pi = host->pi;

	pr_debug("=============================\n");
	pr_debug("ed64mmc_request opcode=%i arg=%08X spicfg=%02X data=%p\n", cmd->opcode, cmd->arg, host->spicfg, data);

	n64pi_begin(pi);

	n64pi_ed64_enable(pi);

	/* some assertions before issueing cmd (to keep hard-time) */
	if (data && cmd->opcode == 0) {
		panic("ed64sdmmc: data with opcode==0??");
	}
	if (data && !(cmd->flags & MMC_RSP_OPCODE)) {
		panic("ed64sdmmc: data without opcode echo??");
	}
	if (data && (cmd->flags & MMC_RSP_136)) {
		panic("ed64sdmmc: data with 136bit response??");
	}
	if (data && !(cmd->flags & MMC_RSP_CRC)) {
		panic("ed64sdmmc: data without response crc??");
	}

	/* SetBlockCount if specified (TODO mrq->sbc->error handling) */
	if (mrq->sbc) {
		pr_debug("ed64mmc_request sbc opcode=%i arg=%08X spicfg=%02X\n", mrq->sbc->opcode, mrq->sbc->arg, host->spicfg);
		ed64mmc_send_command(host, mrq->sbc, 0);
		ed64mmc_recv_cmd(host, mrq->sbc);
	}

	/* send command */
	ed64mmc_send_command(host, cmd, 0);

	if (data) { /* read response buffer and transfer data (fast-path) for reading DAT (hard-time required) */
		int bits;
		int i;

		/* read first byte of response (to wait for command completion if ACMD13) */

		bits = ed64mmc_recv_cmd_start(host, 1);

		//dev_dbg(dev, "w/data first cmd response byte; bits=%d\n", bits);

		if (bits < 0) {
			cmd->error = -ETIMEDOUT;
		} else {
			uint8_t cmdresp[5];

			/* note: testing bits with cmd->opcode or check crc later for keep hard-time */

			if (cmd->opcode == 13 && host->lastopcode == 55) {
				/* ACMD13: ED64 is too slow to receive DAT after ACMD13 response "0d000009205b" completes: QUICK DIRTY HACK: EMULATION!! */
				cmdresp[0] = 0x00;
				cmdresp[1] = 0x00;
				cmdresp[2] = 0x09;
				cmdresp[3] = 0x20;
				cmdresp[4] = 0x5B;
			} else {
				/* receive response and crc (but not verify now) */
				/* TODO which is faster: read 5 bytes, or read into word and a crc byte? */

				/* 8bit command read */
				ed64_spicfg(host, ED64_SPICFG_RD, 1);

				/* read response bytes (incl. crc) */
				for (i = 0; i < 5; i++) {
					cmdresp[i] = ed64_spiread(pi, 0xFF/* something to clock */) & 0xFF;
				}
			}

			//dev_dbg(dev, " response0=%08X\n", cmd->resp[0]);

			/* transfer data */
			/*
			dev_dbg(dev, "transfer: blksz %i blocks %i sg_len %i sg length %i\n",
			        data->blksz, data->blocks, data->sg_len, data->sg->length);
			*/

//if(0x100<data->blksz){extern void stub(void);stub();}
			for (i = 0; i < data->blocks; i++) {
				size_t len = data->blksz;
				u8 *buf;
				struct page *page;
				int result;
				page = sg_page(data->sg);

				/* TODO: it will stride pages, but this maps only first page... in N64, memory is only 8MiB, fits in kseg0, miraclously works. */
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
				} else {
					pr_debug("transfer ok: %xh bytes\n", len);
					data->bytes_xfered += len;
				}
			}

			/* verify cmd now */

			if (bits != cmd->opcode) {
				pr_debug("mis opcode echo: exp=%02X act=%02X\n", cmd->opcode, bits);
				cmd->error = -EILSEQ;
			}

			{
				int crc = crc7_update(0, bits);
				for (i = 0; i < 4; i++) {
					crc = crc7_update(crc, cmdresp[i]);
				}
				if (cmdresp[4] != (crc | 1)) {
					/* crc error */
					cmd->error = -EIO;
					pr_debug(" found crc error: expect=%02X actual=%02X\n", crc | 1, cmdresp[4]);
				}
			}

			/* writeback cmd response */
			cmd->resp[0] = ((struct{int w;}__attribute__((packed))*)cmdresp)->w;
		}
	} else if ((cmd->flags & MMC_RSP_PRESENT) || (cmd->opcode == 0)) { /* read response buffer, or just clock when opcode==0 */
		/* read response */
		ed64mmc_recv_cmd(host, cmd);
	} else {
		pr_debug("expect no response\n");
	}

	/* stop command if specified (TODO mrq->stop->error handling) */
	if (mrq->stop) {
		pr_debug("ed64mmc_request stop opcode=%i arg=%08X spicfg=%02X\n", mrq->stop->opcode, mrq->stop->arg, host->spicfg);
		ed64mmc_send_command(host, mrq->stop, 0);
		ed64mmc_recv_cmd(host, mrq->stop);
	}

	host->lastopcode = cmd->opcode; /* for ACMD13's CMD55 detection */

	n64pi_ed64_disable(pi);

	n64pi_end(pi);

	mmc_request_done(mmc, mrq);
	pr_debug("=============================\n");
}

static void ed64mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ed64mmc_host *host = mmc_priv(mmc);
	dev_dbg(host->dev, "set_ios pm=%d bw=%d clk=%d cs=%d\n", ios->power_mode, ios->bus_width, ios->clock, ios->chip_select);

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

	{
		struct n64pi *pi = host->pi;
		n64pi_begin(pi);
		n64pi_ed64_enable(pi);
		ed64_spicfg(host, 0, 0);
		n64pi_ed64_disable(pi);
		n64pi_end(pi);
	}
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
	mmc->caps |= MMC_CAP_4_BIT_DATA | MMC_CAP_NEEDS_POLL; /* TODO I don't know much about mmc subsystem... stay left as sdricoh. */

	/*
	 * XXX: adopt mmc/core/host.c:mmc_alloc_host default value...
	 *      setting following value will cause oops->panic in account_*_time...
	 *      invoked by destructing their memory by ed64mmc_block_read! (though I did write only in right area!?)
	 *      indicator: mrq->cmd->data->sg->offset = 0x80 (if correct, it is 0)
	mmc->max_seg_size = 4 * 512;
	mmc->max_blk_size = 512;
	*/

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
