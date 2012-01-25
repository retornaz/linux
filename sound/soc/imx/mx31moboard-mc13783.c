/*
 * mx31moboard-mc13783.c  --  SoC audio for mx31moboard boards
 *
 * Copyright 2012 Philippe Retornaz, <philippe.retornaz@epfl.ch>
 *
 * Heavly based on phycore-mc13783:
 * Copyright 2009 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>

#include "../codecs/mc13783.h"
#include "imx-ssi.h"

static struct snd_soc_card imx_moboard;

#define FMT_SSI (SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF | \
		SND_SOC_DAIFMT_CBM_CFM)

static int imx_moboard_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai, FMT_SSI);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, FMT_SSI);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0xfffffffc, 0xfffffffc, 4, 16);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, MC13783_CLK_CLIA, 26000000, 0);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0x0, 0xfffffffc, 2, 16);
	if (ret)
		return ret;

	return 0;
}

static int imx_moboard_hifi_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static struct snd_soc_ops imx_moboard_hifi_ops = {
	.hw_params = imx_moboard_hifi_hw_params,
	.hw_free = imx_moboard_hifi_hw_free,
};

static int imx_moboard_probe(struct snd_soc_card *dev)
{
	return 0;
}

static int imx_moboard_remove(struct snd_soc_card *dev)
{
	return 0;
}

static struct snd_soc_dai_link imx_moboard_dai_mc13783[] = {
	{
		.name = "MC13783",
		.stream_name	 = "Sound",
		.codec_dai_name	 = "mc13783-hifi",
		.codec_name	 = "mc13783-codec",
		.cpu_dai_name	 = "imx-ssi.0",
		.platform_name	 = "imx-pcm-audio.0",
		.ops		 = &imx_moboard_hifi_ops,
		.symmetric_rates = 1,
	},
};

static struct snd_soc_card imx_moboard = {
	.name		= "mx31moboard",
	.probe		= imx_moboard_probe,
	.remove		= imx_moboard_remove,
	.dai_link	= imx_moboard_dai_mc13783,
	.num_links	= ARRAY_SIZE(imx_moboard_dai_mc13783),
};

static struct platform_device *imx_moboard_snd_device;

static int __init imx_moboard_init(void)
{
	int ret;

	if (!machine_is_mx31moboard())
		/* return happy. We might run on a totally different machine */
		return 0;

	imx_moboard_snd_device = platform_device_alloc("soc-audio", -1);
	if (!imx_moboard_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_moboard_snd_device, &imx_moboard);
	ret = platform_device_add(imx_moboard_snd_device);

	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		platform_device_put(imx_moboard_snd_device);
	}

	return ret;
}

static void __exit imx_moboard_exit(void)
{
	platform_device_unregister(imx_moboard_snd_device);
}

late_initcall(imx_moboard_init);
module_exit(imx_moboard_exit);

MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_AUTHOR("Philippe Retornaz <philippe.retornaz@epfl.ch");
MODULE_DESCRIPTION("mx31moboard ALSA SoC driver");
MODULE_LICENSE("GPL");
