/*
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 * Copyright 2009 Sascha Hauer, s.hauer@pengutronix.de
 *
 * Initial development of this code was funded by
 * Phytec Messtechnik GmbH, http://www.phytec.de
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mfd/mc13xxx.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/soc-dapm.h>

#include "mc13783.h"

#define MC13783_AUDIO_RX0	36
#define MC13783_AUDIO_RX1	37
#define MC13783_AUDIO_TX	38
#define MC13783_SSI_NETWORK	39
#define MC13783_AUDIO_CODEC	40
#define MC13783_AUDIO_DAC	41

#define AUDIO_RX0_ALSPEN		(1 << 5)
#define AUDIO_RX0_ALSPSEL		(1 << 7)
#define AUDIO_RX0_ADDCDC		(1 << 21)
#define AUDIO_RX0_ADDSTDC		(1 << 22)
#define AUDIO_RX0_ADDRXIN		(1 << 23)

#define AUDIO_RX1_PGARXEN		(1 << 0);
#define AUDIO_RX1_PGASTEN		(1 << 5)
#define AUDIO_RX1_ARXINEN		(1 << 10)

#define AUDIO_TX_AMC1REN		(1 << 5)
#define AUDIO_TX_AMC1LEN		(1 << 7)
#define AUDIO_TX_AMC2EN			(1 << 9)
#define AUDIO_TX_ATXINEN		(1 << 11)
#define AUDIO_TX_RXINREC		(1 << 13)

#define SSI_NETWORK_CDCTXRXSLOT(x)	(((x) & 0x3) << 2)
#define SSI_NETWORK_CDCTXSECSLOT(x)	(((x) & 0x3) << 4)
#define SSI_NETWORK_CDCRXSECSLOT(x)	(((x) & 0x3) << 6)
#define SSI_NETWORK_CDCRXSECGAIN(x)	(((x) & 0x3) << 8)
#define SSI_NETWORK_CDCSUMGAIN(x)	(1 << 10)
#define SSI_NETWORK_CDCFSDLY(x)		(1 << 11)
#define SSI_NETWORK_DAC_SLOTS_8		(1 << 12)
#define SSI_NETWORK_DAC_SLOTS_4		(2 << 12)
#define SSI_NETWORK_DAC_SLOTS_2		(3 << 12)
#define SSI_NETWORK_DAC_SLOT_MASK	(3 << 12)
#define SSI_NETWORK_DAC_RXSLOT_0_1	(0 << 14)
#define SSI_NETWORK_DAC_RXSLOT_2_3	(1 << 14)
#define SSI_NETWORK_DAC_RXSLOT_4_5	(2 << 14)
#define SSI_NETWORK_DAC_RXSLOT_6_7	(3 << 14)
#define SSI_NETWORK_DAC_RXSLOT_MASK	(3 << 14)
#define SSI_NETWORK_STDCRXSECSLOT(x)	(((x) & 0x3) << 16)
#define SSI_NETWORK_STDCRXSECGAIN(x)	(((x) & 0x3) << 18)
#define SSI_NETWORK_STDCSUMGAIN		(1 << 20)

/*
 * MC13783_AUDIO_CODEC and MC13783_AUDIO_DAC mostly share the same
 * register layout
 */
#define AUDIO_SSI_SEL			(1 << 0)
#define AUDIO_CLK_SEL			(1 << 1)
#define AUDIO_CSM			(1 << 2)
#define AUDIO_BCL_INV			(1 << 3)
#define AUDIO_CFS_INV			(1 << 4)
#define AUDIO_CFS(x)			(((x) & 0x3) << 5)
#define AUDIO_CLK(x)			(((x) & 0x7) << 7)
#define AUDIO_C_EN			(1 << 11)
#define AUDIO_C_CLK_EN			(1 << 12)
#define AUDIO_C_RESET			(1 << 15)

#define AUDIO_CODEC_CDCFS8K16K		(1 << 10)
#define AUDIO_DAC_CFS_DLY_B		(1 << 10)

struct mc13783_priv {
	struct snd_soc_codec codec;
	struct mc13xxx *mc13xxx;

	u32 reg_cache[42];

	int mc13783_asp_val;
	int mc13783_alsp_val;
	enum mc13783_ssi_port adc_ssi_port;
	enum mc13783_ssi_port dac_ssi_port;
	int mc13783_ahsout_val;
};

static unsigned int mc13783_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	struct mc13783_priv *priv = snd_soc_codec_get_drvdata(codec);

	return priv->reg_cache[reg];
}

static int mc13783_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	struct mc13783_priv *priv = snd_soc_codec_get_drvdata(codec);
	int ret;

	mc13xxx_lock(priv->mc13xxx);
	priv->reg_cache[reg] = value;

	ret = mc13xxx_reg_write(priv->mc13xxx, reg, value);

	mc13xxx_unlock(priv->mc13xxx);

	return ret;
}

/* Mapping between sample rates and register value */
static unsigned int mc13783_rates[] = {
	8000, 11025, 12000, 16000,
	22050, 24000, 32000, 44100,
	48000, 64000, 96000
};

static int mc13783_pcm_hw_params_dac(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	unsigned int rate = params_rate(params);
	int i;

	for (i = 0; i < ARRAY_SIZE(mc13783_rates); i++) {
		if (rate == mc13783_rates[i]) {
			snd_soc_update_bits(codec, MC13783_AUDIO_DAC,
					0xf << 17, i << 17);
			return 0;
		}
	}

	return -EINVAL;
}

static int mc13783_pcm_hw_params_codec(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	unsigned int rate = params_rate(params);
	unsigned int val;

	switch (rate) {
	case 8000:
		val = 0;
		break;
	case 16000:
		val = AUDIO_CODEC_CDCFS8K16K;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, MC13783_AUDIO_CODEC, AUDIO_CODEC_CDCFS8K16K,
			val);

	return 0;
}

static int mc13783_pcm_hw_params_sync(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return mc13783_pcm_hw_params_dac(substream, params, dai);
	else
		return mc13783_pcm_hw_params_codec(substream, params, dai);
}

static int mc13783_set_fmt(struct snd_soc_dai *dai, unsigned int fmt, unsigned int reg)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int val;

	val = mc13783_read(codec, reg);

	val &= ~AUDIO_CFS(3);
	val &= ~AUDIO_BCL_INV;
	val &= ~AUDIO_CFS_INV;
	val &= ~AUDIO_CSM;
	val &= ~AUDIO_C_CLK_EN;

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		val |= AUDIO_CFS(2);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		val |= AUDIO_CFS(1);
		break;
	default:
		return -EINVAL;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		val |= AUDIO_BCL_INV;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		val |= AUDIO_BCL_INV;
		val |= AUDIO_CFS_INV;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		val |= AUDIO_CFS_INV;
		break;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		val |= AUDIO_C_CLK_EN;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		val |= AUDIO_CSM;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		return -EINVAL;
	}

	val |= AUDIO_C_RESET;

	mc13783_write(codec, reg, val);

	return 0;
}

static int mc13783_set_fmt_async(struct snd_soc_dai *dai, unsigned int fmt)
{
	if (dai->id == MC13783_ID_STEREO_DAC)
		return mc13783_set_fmt(dai, fmt, MC13783_AUDIO_DAC);
	else
		return mc13783_set_fmt(dai, fmt, MC13783_AUDIO_CODEC);
}

static int mc13783_set_fmt_sync(struct snd_soc_dai *dai, unsigned int fmt)
{
	int ret;

	ret = mc13783_set_fmt(dai, fmt, MC13783_AUDIO_DAC);
	if (ret)
		return ret;

	/*
	 * In synchronous mode force the voice codec into slave mode
	 * so that the clock / framesync from the stereo DAC is used
	 */
	fmt &= ~SND_SOC_DAIFMT_MASTER_MASK;
	fmt |= SND_SOC_DAIFMT_CBS_CFS;
	ret = mc13783_set_fmt(dai, fmt, MC13783_AUDIO_CODEC);

	return ret;
}

static int mc13783_sysclk[] = {
	13000000,
	15360000,
	16800000,
	-1,
	26000000,
	-1, /* 12000000, invalid for voice codec */
	-1, /* 3686400, invalid for voice codec */
	33600000,
};

static int mc13783_set_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir,
				  unsigned int reg)
{
	struct snd_soc_codec *codec = dai->codec;
	int clk;
	unsigned int val;

	val = mc13783_read(codec, reg);

	val &= ~AUDIO_CLK(0x7);
	val &= ~AUDIO_CLK_SEL;

	for (clk = 0; clk < ARRAY_SIZE(mc13783_sysclk); clk++) {
		if (mc13783_sysclk[clk] < 0)
			continue;
		if (mc13783_sysclk[clk] == freq)
			break;
	}

	if (clk == ARRAY_SIZE(mc13783_sysclk))
		return -EINVAL;

	if (clk_id == MC13783_CLK_CLIB)
		val |= AUDIO_CLK_SEL;

	val |= AUDIO_CLK(clk);

	mc13783_write(codec, reg, val);

	return 0;
}

static int mc13783_set_sysclk_dac(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	return mc13783_set_sysclk(dai, clk_id, freq, dir, MC13783_AUDIO_DAC);
}

static int mc13783_set_sysclk_codec(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	return mc13783_set_sysclk(dai, clk_id, freq, dir, MC13783_AUDIO_CODEC);
}

static int mc13783_set_sysclk_sync(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	int ret;

	ret = mc13783_set_sysclk(dai, clk_id, freq, dir, MC13783_AUDIO_DAC);
	if (ret)
		return ret;

	return mc13783_set_sysclk(dai, clk_id, freq, dir, MC13783_AUDIO_CODEC);
}

static int mc13783_set_tdm_slot_dac(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots,
	int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int val;

	val = mc13783_read(codec, MC13783_SSI_NETWORK);

	val &= ~SSI_NETWORK_DAC_SLOT_MASK;
	val &= ~SSI_NETWORK_DAC_RXSLOT_MASK;

	switch (slots) {
	case 2:
		val |= SSI_NETWORK_DAC_SLOTS_2;
		break;
	case 4:
		val |= SSI_NETWORK_DAC_SLOTS_4;
		break;
	case 8:
		val |= SSI_NETWORK_DAC_SLOTS_8;
		break;
	default:
		return -EINVAL;
	}

	switch (rx_mask) {
	case 0xfffffffc:
		val |= SSI_NETWORK_DAC_RXSLOT_0_1;
		break;
	case 0xfffffff3:
		val |= SSI_NETWORK_DAC_RXSLOT_2_3;
		break;
	case 0xffffffcf:
		val |= SSI_NETWORK_DAC_RXSLOT_4_5;
		break;
	case 0xffffff3f:
		val |= SSI_NETWORK_DAC_RXSLOT_6_7;
		break;
	default:
		return -EINVAL;
	};

	mc13783_write(codec, MC13783_SSI_NETWORK, val);

	return 0;
}

static int mc13783_set_tdm_slot_codec(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots,
	int slot_width)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int val;

	if (slots != 4)
		return -EINVAL;

	if (tx_mask != 0xfffffffc)
		return -EINVAL;

	val = mc13783_read(codec, MC13783_SSI_NETWORK);

	val &= ~(0x3f << 0);
	val |= (0x00 << 2);	/* primary timeslot RX/TX(?) is 0 */
	val |= (0x01 << 4);	/* secondary timeslot TX is 1 */

	mc13783_write(codec, MC13783_SSI_NETWORK, val);

	return 0;
}

static int mc13783_set_tdm_slot_sync(struct snd_soc_dai *dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots,
	int slot_width)
{
	int ret;

	ret = mc13783_set_tdm_slot_dac(dai, tx_mask, rx_mask, slots,
			slot_width);
	if (ret)
		return ret;

	ret = mc13783_set_tdm_slot_codec(dai, tx_mask, rx_mask, slots,
			slot_width);

	return ret;
}

static void mc13783_startup(struct snd_soc_dai *dai, int reg)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int val;

	val = mc13783_read(codec, reg);
	val &= ~AUDIO_C_RESET;
	val |= AUDIO_C_EN;
	mc13783_write(codec, reg, val);
}

static int mc13783_startup_dac(struct snd_pcm_substream *stream,
		struct snd_soc_dai *dai)
{
	mc13783_startup(dai, MC13783_AUDIO_DAC);
	return 0;
}

static int mc13783_startup_codec(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	mc13783_startup(dai, MC13783_AUDIO_CODEC);
	return 0;
}

static int mc13783_startup_sync(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		mc13783_startup(dai, MC13783_AUDIO_DAC);
	else
		mc13783_startup(dai, MC13783_AUDIO_CODEC);
	return 0;
}

static void mc13783_shutdown(struct snd_soc_dai *dai, int reg)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int val;

	val = mc13783_read(codec, reg);
	mc13783_write(codec, reg, val & ~AUDIO_C_EN);
}

static void mc13783_shutdown_dac(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	mc13783_shutdown(dai, MC13783_AUDIO_DAC);
}

static void mc13783_shutdown_codec(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	mc13783_shutdown(dai, MC13783_AUDIO_CODEC);
}

static void mc13783_shutdown_sync(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		mc13783_shutdown(dai, MC13783_AUDIO_DAC);
	else
		mc13783_shutdown(dai, MC13783_AUDIO_CODEC);
}

static int mc13783_get_alsp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mc13783_priv *priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = priv->mc13783_alsp_val;

	return 0;
}

static int mc13783_put_alsp(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	struct mc13783_priv *priv = snd_soc_codec_get_drvdata(codec);
	unsigned int val;

	priv->mc13783_alsp_val = ucontrol->value.integer.value[0];

	val = mc13783_read(codec, MC13783_AUDIO_RX0);

	val &= ~(AUDIO_RX0_ALSPEN | AUDIO_RX0_ALSPSEL);

	if (priv->mc13783_alsp_val)
		val |= AUDIO_RX0_ALSPEN;

	if (priv->mc13783_alsp_val == 2)
		val |= AUDIO_RX0_ALSPSEL;

	mc13783_write(codec, MC13783_AUDIO_RX0, val);

	return 0;
}

static int mc13783_ahsout_i_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mc13783_priv *priv = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = priv->mc13783_ahsout_val;

	return 0;
}

static int mc13783_ahsout_i_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mc13783_priv *priv = snd_soc_codec_get_drvdata(codec);
	unsigned int reg;

	priv->mc13783_ahsout_val = ucontrol->value.integer.value[0];

	reg = mc13783_read(codec, MC13783_AUDIO_RX0);

	reg &= ~((1 << 13) | (1 << 14));

	if (priv->mc13783_ahsout_val == 1)
		reg |= 1 << 13;
	else if (priv->mc13783_ahsout_val == 2)
		reg |= 1 << 14;


	mc13783_write(codec, MC13783_AUDIO_RX0, reg);

	return 0;
}

static int mc13783_pcm_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	int val;

	val = mc13783_read(codec, MC13783_AUDIO_RX0);
	ucontrol->value.enumerated.item[0] = (val >> 22) & 1;

	return 0;
}

static int mc13783_pcm_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	unsigned int r36, r37;

	r36 = mc13783_read(codec, MC13783_AUDIO_RX0);
	r37 = mc13783_read(codec, MC13783_AUDIO_RX1);

	r36 &= ~AUDIO_RX0_ADDSTDC;
	r37 &= ~AUDIO_RX1_PGASTEN;

	if (ucontrol->value.enumerated.item[0]) {
		r36 |= AUDIO_RX0_ADDSTDC;
		r37 |= AUDIO_RX1_PGASTEN;
	}

	mc13783_write(codec, MC13783_AUDIO_RX0, r36);
	mc13783_write(codec, MC13783_AUDIO_RX1, r37);

	return 0;
}

static int mc13783_linein_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	int val;

	val = mc13783_read(codec, MC13783_AUDIO_RX0);
	ucontrol->value.enumerated.item[0] = (val >> 23) & 1;

	return 0;
}

static int mc13783_linein_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	unsigned int r36, r37;

	r36 = mc13783_read(codec, MC13783_AUDIO_RX0);
	r37 = mc13783_read(codec, MC13783_AUDIO_RX1);

	r36 &= ~AUDIO_RX0_ADDRXIN;
	r37 &= ~AUDIO_RX1_ARXINEN;

	if (ucontrol->value.enumerated.item[0]) {
		r36 |= AUDIO_RX0_ADDRXIN;
		r37 |= AUDIO_RX1_ARXINEN;
	}

	mc13783_write(codec, MC13783_AUDIO_RX0, r36);
	mc13783_write(codec, MC13783_AUDIO_RX1, r37);

	return 0;
}

static int mc13783_voice_get(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	int val;

	val = mc13783_read(codec, MC13783_AUDIO_RX0);
	ucontrol->value.enumerated.item[0] = (val >> 21) & 1;

	return 0;
}

static int mc13783_voice_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	unsigned int r36, r37;

	r36 = mc13783_read(codec, MC13783_AUDIO_RX0);
	r37 = mc13783_read(codec, MC13783_AUDIO_RX1);

	r36 &= ~AUDIO_RX0_ADDCDC;
	r37 &= ~AUDIO_RX1_PGARXEN;

	if (ucontrol->value.enumerated.item[0]) {
		r36 |= AUDIO_RX0_ADDCDC;
		r37 |= AUDIO_RX1_PGARXEN;
	}

	mc13783_write(codec, MC13783_AUDIO_RX0, r36);
	mc13783_write(codec, MC13783_AUDIO_RX1, r37);

	return 0;
}

static int mc13783_capure_cache;

static int mc13783_get_capture(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.enumerated.item[0] = mc13783_capure_cache;
	return 0;
}

static int mc13783_put_capture(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec =  snd_kcontrol_chip(kcontrol);
	unsigned int r38, change;

	r38 = mc13783_read(codec, MC13783_AUDIO_TX);

	change = (mc13783_capure_cache != ucontrol->value.enumerated.item[0]);
	mc13783_capure_cache = ucontrol->value.enumerated.item[0];
	r38 &= ~(AUDIO_TX_AMC1REN | AUDIO_TX_AMC2EN | AUDIO_TX_ATXINEN |
			AUDIO_TX_RXINREC | AUDIO_TX_AMC1LEN);

	switch (mc13783_capure_cache) {
	case 0:
		break;
	case 1:
		r38 |= AUDIO_TX_RXINREC;
		break;
	case 2:
		r38 |= AUDIO_TX_AMC1REN | AUDIO_TX_AMC1LEN;
		break;
	case 3:
		r38 |= AUDIO_TX_AMC1REN;
		break;
	case 4:
		r38 |= AUDIO_TX_AMC2EN;
		break;
	case 5:
		r38 |= AUDIO_TX_AMC1LEN | AUDIO_TX_AMC2EN;
		break;
	case 6:
		r38 |= AUDIO_TX_ATXINEN;
		break;
	case 7:
		r38 |= AUDIO_TX_AMC1LEN | AUDIO_TX_ATXINEN;
		break;
	case 8:
		r38 |= AUDIO_TX_AMC1LEN | AUDIO_TX_RXINREC;
		break;
	case 9:
		r38 |= AUDIO_TX_AMC1LEN;
		break;
	default:
		break;
	}

	mc13783_write(codec, MC13783_AUDIO_TX, r38);

	return change;
}

static const char *mc13783_asp[] = {"Off", "Codec", "Right"};
static const char *mc13783_alsp[] = {"Off", "Codec", "Right"};

static const char *mc13783_ahs[] = {"Codec", "Mixer"};

static const char *mc13783_ahsout[] = {"Off", "Auto", "On"};

static const char *mc13783_arxout[] = {"Codec", "Mixer"};

static const char *mc13783_capture[] = {"off/off", "rxinl/rxinr",
	"mc1lin/mc1rin", "off/mc1rin", "off/mc2in", "mc1lin/mc2in",
	"off/txin", "mc1lin/txin", "mc1lin/rxinr", "mc1lin/off"};

static const char *mc13783_3d_mixer[] =	{"Stereo", "Phase Mix",
	"Mono", "Mono Mix"};

static const struct soc_enum mc13783_enum_asp =
	SOC_ENUM_SINGLE(MC13783_AUDIO_RX0, 3, ARRAY_SIZE(mc13783_asp), mc13783_asp);

static const struct soc_enum mc13783_enum_alsp =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mc13783_alsp), mc13783_alsp);

static const struct soc_enum mc13783_enum_ahs =
	SOC_ENUM_SINGLE(MC13783_AUDIO_RX0, 11, ARRAY_SIZE(mc13783_ahs),
			mc13783_ahs);

static const struct soc_enum mc13783_enum_ahsout =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mc13783_ahsout), mc13783_ahsout);

static const struct soc_enum mc13783_enum_arxout =
	SOC_ENUM_SINGLE(MC13783_AUDIO_RX0, 17, ARRAY_SIZE(mc13783_arxout),
			mc13783_arxout);

static const struct soc_enum mc13783_enum_capture =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(mc13783_capture), mc13783_capture);

static const struct soc_enum mc13783_enum_3d_mixer =
	SOC_ENUM_SINGLE(MC13783_AUDIO_RX1, 16, ARRAY_SIZE(mc13783_3d_mixer),
			mc13783_3d_mixer);

static struct snd_kcontrol_new mc13783_control_list[] = {
	/* Output Routing */
	SOC_ENUM("Asp Source", mc13783_enum_asp),
	SOC_ENUM_EXT("Alsp Source", mc13783_enum_alsp, mc13783_get_alsp,
			mc13783_put_alsp),
	SOC_ENUM("Ahs Source", mc13783_enum_ahs),
	SOC_SINGLE("Ahsr enable", MC13783_AUDIO_RX0, 9, 1, 0),
	SOC_SINGLE("Ahsl enable", MC13783_AUDIO_RX0, 10, 1, 0),
	SOC_ENUM_EXT("Ahs enable", mc13783_enum_ahsout, mc13783_ahsout_i_get,
			mc13783_ahsout_i_put),
	SOC_ENUM("Arxout Source", mc13783_enum_arxout),
	SOC_SINGLE("ArxoutR enable", MC13783_AUDIO_RX0, 16, 1, 0),
	SOC_SINGLE("ArxoutL enable", MC13783_AUDIO_RX0, 15, 1, 0),
	SOC_SINGLE_EXT("PCM Playback Switch", 0, 0, 1, 0, mc13783_pcm_get,
			mc13783_pcm_put),
	SOC_SINGLE("PCM Playback Volume", MC13783_AUDIO_RX1, 6, 15, 0),
	SOC_SINGLE_EXT("Line in Switch", 0, 0, 1, 0, mc13783_linein_get,
			mc13783_linein_put),
	SOC_SINGLE("Line in Volume", MC13783_AUDIO_RX1, 12, 15, 0),
	SOC_ENUM_EXT("Capture Source", mc13783_enum_capture, mc13783_get_capture,
			mc13783_put_capture),
	SOC_DOUBLE("PCM Capture Volume", MC13783_AUDIO_TX, 19, 14, 31, 0),
	SOC_ENUM("3D Control", mc13783_enum_3d_mixer),
	SOC_SINGLE("MC1 Bias enable", 38, 0, 1, 0),
	SOC_SINGLE("Codec Bypass enable", MC13783_AUDIO_CODEC, 16, 1, 0),
	SOC_SINGLE_EXT("Voice Codec Switch", 0, 0, 1, 0, mc13783_voice_get,
			mc13783_voice_put),
	SOC_SINGLE("Voice Codec Volume", MC13783_AUDIO_RX1, 1, 15, 0),
};

static int mc13783_probe(struct snd_soc_codec *codec)
{
	struct mc13783_priv *priv = snd_soc_codec_get_drvdata(codec);
	int i, ret = 0, val;

	/* these are the reset values */
	priv->reg_cache[MC13783_AUDIO_RX0]  = 0x001000;
	priv->reg_cache[MC13783_AUDIO_RX1]  = 0x00d35A;
	priv->reg_cache[MC13783_AUDIO_TX]    = 0x420000;
	priv->reg_cache[MC13783_SSI_NETWORK] = 0x013060;
	priv->reg_cache[MC13783_AUDIO_CODEC] = 0x180027;
	priv->reg_cache[MC13783_AUDIO_DAC]   = 0x0e0004;

	/* VAUDIOON -> supply audio part, BIAS enable */
	priv->reg_cache[MC13783_AUDIO_RX0] |= 0x3;

	for (i = 36; i < 42; i++)
		mc13783_write(codec, i, priv->reg_cache[i]);

	snd_soc_add_controls(codec, mc13783_control_list,
			     ARRAY_SIZE(mc13783_control_list));

	val = mc13783_read(codec, MC13783_AUDIO_CODEC);
	if (priv->adc_ssi_port == MC13783_SSI1_PORT)
		val &= ~AUDIO_SSI_SEL;
	else
		val |= AUDIO_SSI_SEL;
	mc13783_write(codec, MC13783_AUDIO_CODEC, val);

	val = mc13783_read(codec, MC13783_AUDIO_DAC);
	if (priv->dac_ssi_port == MC13783_SSI1_PORT)
		val &= ~AUDIO_SSI_SEL;
	else
		val |= AUDIO_SSI_SEL;
	mc13783_write(codec, MC13783_AUDIO_DAC, val);

	return ret;
}

static int mc13783_remove(struct snd_soc_codec *codec)
{
	unsigned int val;

	val = mc13783_read(codec, MC13783_AUDIO_RX0);

	/* VAUDIOON -> switch off audio part, BIAS disable */
	val &= ~0x3;

	mc13783_write(codec, MC13783_AUDIO_RX0, val);

	return 0;
}

#define MC13783_RATES_RECORD (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000)

#define MC13783_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops mc13783_ops_dac = {
	.hw_params	= mc13783_pcm_hw_params_dac,
	.set_fmt	= mc13783_set_fmt_async,
	.set_sysclk	= mc13783_set_sysclk_dac,
	.set_tdm_slot	= mc13783_set_tdm_slot_dac,
	.prepare	= mc13783_startup_dac,
	.shutdown	= mc13783_shutdown_dac,
};

static struct snd_soc_dai_ops mc13783_ops_codec = {
	.hw_params	= mc13783_pcm_hw_params_codec,
	.set_fmt	= mc13783_set_fmt_async,
	.set_sysclk	= mc13783_set_sysclk_codec,
	.set_tdm_slot	= mc13783_set_tdm_slot_codec,
	.prepare	= mc13783_startup_codec,
	.shutdown	= mc13783_shutdown_codec,
};

/* 
 * The mc13783 has two SSI ports, both of them can be routed either
 * to the voice codec or the stereo DAC. When two different SSI ports
 * are used for the voice codec and the stereo DAC we can do different
 * formats and sysclock settings for playback and capture
 * (mc13783-hifi-playback and mc13783-hifi-capture). Using the same port
 * forces us to use symmetric rates (mc13783-hifi).
 */
static struct snd_soc_dai_driver mc13783_dai_async[] = {
	{
		.name = "mc13783-hifi-playback",
		.id = MC13783_ID_STEREO_DAC,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = MC13783_FORMATS,
		},
		.ops = &mc13783_ops_dac,
	}, {
		.name = "mc13783-hifi-capture",
		.id = MC13783_ID_STEREO_CODEC,
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC13783_RATES_RECORD,
			.formats = MC13783_FORMATS,
		},
		.ops = &mc13783_ops_codec,
	},
};

static struct snd_soc_dai_ops mc13783_ops_sync = {
	.hw_params	= mc13783_pcm_hw_params_sync,
	.set_fmt	= mc13783_set_fmt_sync,
	.set_sysclk	= mc13783_set_sysclk_sync,
	.set_tdm_slot	= mc13783_set_tdm_slot_sync,
	.prepare	= mc13783_startup_sync,
	.shutdown	= mc13783_shutdown_sync,
};

static struct snd_soc_dai_driver mc13783_dai_sync[] = {
	{
		.name = "mc13783-hifi",
		.id = MC13783_ID_SYNC,
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = MC13783_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = MC13783_RATES_RECORD,
			.formats = MC13783_FORMATS,
		},
		.ops = &mc13783_ops_sync,
		.symmetric_rates = 1,
	}
};

static struct snd_soc_codec_driver soc_codec_dev_mc13783 = {
	.probe		= mc13783_probe,
	.remove		= mc13783_remove,
	.read		= mc13783_read,
	.write		= mc13783_write,
};

static int mc13783_codec_probe(struct platform_device *pdev)
{
	struct mc13xxx *mc13xxx;
	struct mc13783_priv *priv;
	struct mc13xxx_codec_platform_data *pdata = pdev->dev.platform_data;
	int ret;

	mc13xxx = dev_get_drvdata(pdev->dev.parent);

	priv = kzalloc(sizeof(struct mc13783_priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, priv);
	priv->mc13xxx = mc13xxx;
	if (pdata) {
		priv->adc_ssi_port = pdata->adc_ssi_port;
		priv->dac_ssi_port = pdata->dac_ssi_port;
	} else {
		priv->adc_ssi_port = MC13783_SSI1_PORT;
		priv->dac_ssi_port = MC13783_SSI2_PORT;
	}

	if (priv->adc_ssi_port == priv->dac_ssi_port)
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_mc13783,
					mc13783_dai_sync, ARRAY_SIZE(mc13783_dai_sync));
	else
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_mc13783,
					mc13783_dai_async, ARRAY_SIZE(mc13783_dai_async));

	if (ret)
		goto err_register_codec;

	return 0;

err_register_codec:
	dev_err(&pdev->dev, "register codec failed with %d\n", ret);
	kfree(priv);

	return ret;
}

static int mc13783_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static struct platform_driver mc13783_codec_driver = {
	.driver = {
		   .name = "mc13783-codec",
		   .owner = THIS_MODULE,
		   },
	.probe = mc13783_codec_probe,
	.remove = __devexit_p(mc13783_codec_remove),
};

static __init int mc13783_init(void)
{
	return platform_driver_register(&mc13783_codec_driver);
}

static __exit void mc13783_exit(void)
{
	platform_driver_unregister(&mc13783_codec_driver);
}

module_init(mc13783_init);
module_exit(mc13783_exit);

MODULE_DESCRIPTION("ASoC MC13783 driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>");
MODULE_LICENSE("GPL");
