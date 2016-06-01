#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>

static struct snd_pcm_hardware snd_n64ai_pcm_hw = {
	.info = (SNDRV_PCM_INFO_MMAP |
	         SNDRV_PCM_INFO_MMAP_VALID |
	         SNDRV_PCM_INFO_INTERLEAVED |
	         SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats =          SNDRV_PCM_FMTBIT_S16_BE,
	.rates =            SNDRV_PCM_RATE_8000_48000, // TODO really max 48k?
	.rate_min =         8000,
	.rate_max =         48000,
	.channels_min =     2,
	.channels_max =     2,
	.buffer_bytes_max = 65536,
	.period_bytes_min = 32768,
	.period_bytes_max = 65536,
	.periods_min =      1,
	.periods_max =      1024,
};

static int snd_n64ai_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->hw = snd_n64ai_pcm_hw;
	runtime->private_data = NULL;

	return 0;
}

static int snd_n64ai_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	runtime->private_data = NULL;

	return 0;
}

n64ai_n64ai_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_alloc_vmalloc_buffer(substream, params_buffer_bytes(hw_params));
}

static int snd_n64ai_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int snd_n64ai_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;

	spin_lock_irqsave(&chip->channel[ch].lock, flags);

	/* Setup the pseudo-dma transfer pointers.  */
	chip->channel[ch].pos = 0;
	chip->channel[ch].size = 0;
	chip->channel[ch].substream = substream;

	/* set AD1843 format */
	/* hardware format is always S16_LE */
	switch (substream->stream) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		ad1843_setup_dac(&chip->ad1843,
				 ch - 1,
				 runtime->rate,
				 SNDRV_PCM_FORMAT_S16_LE,
				 runtime->channels);
		break;
	default:
		pr_warn("snd-n64ai: unknown stream type %d\n", substream->stream);
		break;
	}
	spin_unlock_irqrestore(&chip->channel[ch].lock, flags);
	return 0;
}

static int snd_n64ai_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* start the PCM engine */
		snd_sgio2audio_dma_start(substream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* stop the PCM engine */
		snd_sgio2audio_dma_stop(substream);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t snd_n64ai_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_sgio2audio *chip = snd_pcm_substream_chip(substream);
	struct snd_sgio2audio_chan *chan = substream->runtime->private_data;

	/* get the current hardware pointer */
	return bytes_to_frames(substream->runtime,
			       chip->channel[chan->idx].pos);
}

static struct snd_pcm_ops snd_n64ai_playback_ops = {
	.open =        snd_n64ai_playback_open,
	.close =       snd_n64ai_pcm_close,
	.ioctl =       snd_pcm_lib_ioctl,
	.hw_params =   snd_n64ai_pcm_hw_params,
	.hw_free =     snd_n64ai_pcm_hw_free,
	.prepare =     snd_n64ai_pcm_prepare,
	.trigger =     snd_n64ai_pcm_trigger,
	.pointer =     snd_n64ai_pcm_pointer,
	.page =        snd_pcm_lib_get_vmalloc_page,
	.mmap =        snd_pcm_lib_mmap_vmalloc,
};

static int snd_n64ai_probe(struct platform_device *pdev)
{
	struct resource *memres;
	struct snd_card *card;
	struct snd_pcm *pcm;
	int irq;
	int err;

	memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	request_mem_region(memres->start, resource_size(memres), "n64ai-mmio");

	irq = platform_get_irq(pdev, 0);
	request_irq(irq, handler, IRQF_TRIGGER_HIGH, "n64ai-irq", NULL/*cookie. card?*/);

	err = snd_card_new(&pdev->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1, THIS_MODULE, 0, &card);
	if (err < 0)
		return err;

	err = snd_pcm_new(card, "Nintendo 64 AI Audio", 0, 1, 0, &pcm);
	if (err < 0)
		return err;

	strcpy(pcm->name, "AI PCM");

	/* set operators */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &snd_n64ai_playback_ops);

	strcpy(card->driver, "n64ai driver");
	strcpy(card->shortname, "n64AI");
	sprintf(card->longname, "%s irq %i", card->shortname, irq);

	err = snd_card_register(card);
	if (err < 0) {
		snd_card_free(card);
		return err;
	}

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
