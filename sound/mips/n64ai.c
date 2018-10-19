/*
 *   Sound driver for Nintendo 64 RCP AI (Audio Interface).
 *
 *   Copyright 2018 Murachue <murachue+github@gmail.com>
 *
 *   Derived from: sgio2audio.c
 *    Copyright 2003 Vivien Chappelier <vivien.chappelier@linux-mips.org>
 *    Copyright 2008 Thomas Bogendoerfer <tsbogend@alpha.franken.de>
 *    Mxier part taken from mace_audio.c:
 *    Copyright 2007 Thorben Jändling <tj.trevelyan@gmail.com>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#define DEBUG

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>

struct snd_n64ai {
	spinlock_t lock;
	void __iomem *regbase;
	struct snd_card *card;
	struct device *dev;

	struct snd_pcm_substream *substream; /* points to the pcm playback substream. */
	ssize_t pos; /* bytes offset currently playing (when playing) or to-be-played (when stopped) */

	/* runtime (embedded to chip because of only one instance: a playback) */
	dma_addr_t last_dma_addr;
	void *last_dma_vptr;
	ssize_t last_dma_nbytes;

	int playing;
	int stopping;
};

static struct snd_pcm_hardware snd_n64ai_pcm_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
	         SNDRV_PCM_INFO_MMAP_VALID |
	         SNDRV_PCM_INFO_INTERLEAVED |
	         SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats =          SNDRV_PCM_FMTBIT_S16_BE,
	.rates =            SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000_48000,
	.rate_min =         8000,
	.rate_max =         48000,
	.channels_min =     2,
	.channels_max =     2,
	.buffer_bytes_max = 131072*4, /* 32768 frames */
	.period_bytes_min = 8, /* period bytes must be multiply of 8 */
	.period_bytes_max = 65536*4,
	.periods_min =      1, /* uh too short? */
	.periods_max =      1024,
};

static int snd_n64ai_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw = snd_n64ai_pcm_hw;

	pr_debug("n64ai: playback_open\n");

	return 0;
}

static int snd_n64ai_playback_close(struct snd_pcm_substream *substream)
{
	/* nothing to do... */

	pr_debug("n64ai: playback_close\n");

	return 0;
}

static int snd_n64ai_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	pr_debug("n64ai: pcm_hw_params\n");
	/* it will reuse preallocated dma buffer at snd_n64ai_probe, or allocate here if it is too short. */
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int snd_n64ai_pcm_hw_free(struct snd_pcm_substream *substream)
{
	pr_debug("n64ai: pcm_hw_free\n");
	/* it will NOT free preallocated, or it DOES free if allocated at snd_n64ai_pcm_hw_params. */
	return snd_pcm_lib_free_pages(substream);
}

static int snd_n64ai_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_n64ai *chip = snd_pcm_substream_chip(substream); /* or just substream->private_data ? */
	unsigned long flags;

	spin_lock_irqsave(&chip->lock, flags);

	pr_debug("n64ai: pcm_prepare\n");

	/* Setup the status.  */
	chip->substream = substream;
	chip->pos = 0;
	chip->last_dma_nbytes = 0;
	chip->playing = 0;
	chip->stopping = 0;

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		/* TODO support non-NTSC */
		/* TODO should be round-off to nearest (4/5) before minus1? */
		__raw_writel(48681812 / runtime->rate - 1, chip->regbase + 0x10);
		__raw_writel(16 - 1, chip->regbase + 0x14); /* note: should be min(48M/rate/66, 16) - 1, but its >368KHz rate... */
		__raw_writel(1, chip->regbase + 0x08); /* only first required? starting DMA without write addr/len cause nothing. */
		break;
	default:
		pr_warn("snd-n64ai: unknown stream type %d\n", substream->stream);
		break;
	}

	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int snd_n64ai_may_unmap_dma(struct snd_n64ai *chip)
{
	ssize_t size;

	/* whether dma mapped is indicated by chip->last_dma_nbytes != 0 */
	size = chip->last_dma_nbytes;
	if (size == 0) {
		/* no dma area is mapped. do nothing. */
		pr_debug("n64ai: unmap_dma nothing\n");
		return 0;
	}

	pr_debug("n64ai: unmap_dma %p->%x+%x\n", chip->last_dma_vptr, chip->last_dma_addr, size);

	/* unmap it and mark no-dma-mapped. */
	dma_unmap_single(chip->dev, chip->last_dma_addr, size, DMA_TO_DEVICE);
	chip->last_dma_nbytes = 0;

	return 1;
}

static int snd_n64ai_dma_at(struct snd_n64ai *chip, ssize_t pos, snd_pcm_uframes_t nframes)
{
	uint32_t status;
	void *vptr;
	ssize_t size;
	dma_addr_t addr;

	/* unmap old if did */
	snd_n64ai_may_unmap_dma(chip);

	status = __raw_readl(chip->regbase + 0x0C);

	/* abort if already AI buffer is full */
	if (status & 0x80000000) {
		pr_debug("n64ai: dma_at full (%08x)\n", status);
		return -1;
	}

	/* substream and its runtime must be exists */
	if (!chip->substream || !chip->substream->runtime) {
		return -2;
	}

	{
		struct snd_pcm_runtime *runtime = chip->substream->runtime;

		/* map next (to be) */
		vptr = runtime->dma_buffer_p->area + pos;
		size = frames_to_bytes(runtime, nframes);
		addr = dma_map_single(chip->dev, vptr, size, DMA_TO_DEVICE);
		if (dma_mapping_error(chip->dev, addr)) {
			return -3;
		}

		pr_debug("n64ai: dma_at %p->%x+%x (%08x)\n", vptr, addr, size, status);

		/* issue TODO memory-barrier? align test? */
		__raw_writel(addr, chip->regbase + 0x00);
		__raw_writel(size, chip->regbase + 0x04); /* note: not minus 1. */

		__raw_writel(0, chip->regbase + 0x04); /* DEBUG: enqueue zero to trying suppress very early IRQ... */

		/* update dma infos */
		chip->last_dma_vptr = vptr;
		chip->last_dma_nbytes = size;
		chip->last_dma_addr = addr;
	}

	return 0;
}

static int snd_n64ai_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_n64ai *chip = snd_pcm_substream_chip(substream); /* or just substream->private_data ? */
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&chip->lock, flags);

	pr_debug("n64ai: pcm_trigger %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* start the PCM engine */
		if (!chip->playing) {
			int r = snd_n64ai_dma_at(chip, chip->pos, substream->runtime->period_size);
			if (r) {
				pr_err("n64ai: pcm_trigger: dma_at failed err=%d\n", r);
			} else {
				chip->playing = 1;
				chip->stopping = 0;
			}
			ret = r ? -EIO : 0;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* stop the PCM engine */
		//__raw_writel(0, chip->regbase + 0x04); /* set length=0 to make the period currently playing is last. */
		//snd_n64ai_may_unmap_dma(chip);
		chip->playing = 0;
		chip->stopping = 1;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	spin_unlock_irqrestore(&chip->lock, flags);

	return ret;
}

static snd_pcm_uframes_t snd_n64ai_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_n64ai *chip = snd_pcm_substream_chip(substream); /* or just substream->private_data ? */

	/* current hardware pointer is not available on Nintendo 64 AI... software fallback. */
	return bytes_to_frames(substream->runtime, chip->pos);
}

static struct snd_pcm_ops snd_n64ai_playback_ops = {
	.open =        snd_n64ai_playback_open,
	.close =       snd_n64ai_playback_close,
	.ioctl =       snd_pcm_lib_ioctl,
	.hw_params =   snd_n64ai_pcm_hw_params,
	.hw_free =     snd_n64ai_pcm_hw_free,
	.prepare =     snd_n64ai_pcm_prepare,
	.trigger =     snd_n64ai_pcm_trigger,
	.pointer =     snd_n64ai_pcm_pointer,
};

static void snd_n64ai_advance_pos(struct snd_n64ai *chip, struct snd_pcm_runtime *runtime)
{
	ssize_t buffer_nbytes = frames_to_bytes(runtime, runtime->buffer_size);
	chip->pos = (chip->pos + chip->last_dma_nbytes) % buffer_nbytes;
}

static irqreturn_t snd_n64ai_isr(int irq, void *dev_id)
{
	struct snd_n64ai *chip = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&chip->lock, flags);
	{
		struct snd_pcm_substream *substream = chip->substream;
		uint32_t status;

		/* clear interrupt */
		__raw_writel(0, chip->regbase + 0x0C);
		status = __raw_readl(chip->regbase + 0x0C);

		if (chip->stopping) {
			/* this is the last period. just ack. */
			if (status & 0x40000000) {
				if (chip->stopping < 5) {
					pr_debug("n64ai: isr waits %08X\n", status);
					__raw_writel(0, chip->regbase + 0x04); /* enqueue 0 byte to make 1) next interrupt 2) gracefully stop the DMA */
					chip->stopping++;
				} else {
					int i = 0;
					pr_debug("n64ai: isr waits %08X", status);
					while (status & 0x40000000) {
						status = __raw_readl(chip->regbase + 0x0C);
						i++;
					}
					pr_debug("required %d loops\n", i);
				}
			} else {
				pr_debug("n64ai: isr stops %08X", status);
				__raw_writel(0, chip->regbase + 0x08); /* DMA should be empty. stop the DMA. */
				pr_debug("->%08X\n", __raw_readl(chip->regbase + 0x0C));
				chip->stopping = 0;
			}
			if (substream && substream->runtime) {
				/* speaker-test invokes here with substream->runtime == NULL?? */
				snd_n64ai_advance_pos(chip, substream->runtime); /* this must be before snd_n64ai_may_unmap_dma. */
			}
			snd_n64ai_may_unmap_dma(chip);
		} else if (!chip->playing || !substream || !substream->runtime || (chip->last_dma_nbytes == 0)) {
			pr_err("n64ai: spurious interrupt %08X\n", status);
		} else {
			struct snd_pcm_runtime *runtime = substream->runtime;
			int r;
			pr_debug("n64ai: isr playing %08X\n", status);
			snd_n64ai_advance_pos(chip, runtime); /* this must be before snd_pcm_period_elapsed. */
			snd_pcm_period_elapsed(substream);
			if (chip->playing) { /* snd_pcm_period_elapsed may stop substream...!! verify playing now here */
				if ((r = snd_n64ai_dma_at(chip, chip->pos, runtime->period_size)) != 0) {
					pr_err("n64ai: isr: dma_at failed err=%d\n", r);
				}
			}
		}
	}
	spin_unlock_irqrestore(&chip->lock, flags);

	return IRQ_HANDLED;
}

static int snd_n64ai_probe(struct platform_device *pdev)
{
	struct resource *memres;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_n64ai *chip;
	int irq;
	int err;

	/* create the card */
	err = snd_card_new(&pdev->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1, THIS_MODULE, sizeof(struct snd_n64ai), &card);
	if (err < 0) {
		return err;
	}

	/* initialize the device structure */
	chip = card->private_data;

	spin_lock_init(&chip->lock);
	chip->card = card;
	chip->dev = &pdev->dev;
	chip->substream = NULL;

	/* allocate resources for the card (the PCM but 1card:1pcm so it can be called "the card") */
	memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->regbase = devm_ioremap_resource(&pdev->dev, memres);
	if (IS_ERR(chip->regbase)) {
		snd_card_free(card);
		return PTR_ERR(chip->regbase);
	}

	irq = platform_get_irq(pdev, 0);
	err = devm_request_irq(&pdev->dev, irq, snd_n64ai_isr, 0/*noshare*/, "n64ai", chip);
	if (err < 0) {
		snd_card_free(card);
		return err;
	}

	/* create the new pcm of the card */
	err = snd_pcm_new(card, "Nintendo 64 AI Audio", 0, 1, 0, &pcm);
	if (err < 0) {
		snd_card_free(card);
		return err;
	}

	pcm->private_data = chip; /* note: can be reached by pcm->card->private_data, this is shortcut. also for substreams. */
	strcpy(pcm->name, "N64AI PCM");
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_n64ai_playback_ops);
	snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_CONTINUOUS, snd_dma_continuous_data(GFP_KERNEL), snd_n64ai_pcm_hw.buffer_bytes_max, snd_n64ai_pcm_hw.buffer_bytes_max); /* TODO align 8?? */

	/* setup the card description after resources are acquired. */
	strcpy(card->driver, "n64ai driver");
	strcpy(card->shortname, "n64ai");
	sprintf(card->longname, "Nintendo 64 AI regbase %p irq %i", chip->regbase, irq);

	/* register the card!! */
	err = snd_card_register(card);
	if (err < 0) {
		snd_card_free(card);
		return err;
	}

	/* remember the card for remove. */
	platform_set_drvdata(pdev, card);

	return 0;
}

static int snd_n64ai_remove(struct platform_device *pdev)
{
	struct snd_card *card = platform_get_drvdata(pdev);

	snd_card_free(card);

	return 0;
}

static struct platform_driver n64ai_driver = {
	.probe	= snd_n64ai_probe,
	.remove	= snd_n64ai_remove,
	.driver = {
		.name	= "n64ai",
	}
};

module_platform_driver(n64ai_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Nintendo 64 Audio Interface Driver");
MODULE_AUTHOR("Murachue <murachue+github@gmail.com>");
