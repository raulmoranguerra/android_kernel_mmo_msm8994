/*
 * tas2552.c  --  ASoC driver for the TI TAS2552 smart amplifier
 *
 * Copyright (C) 2014 Xiaomi Corporation
 * Copyright (C) 2018 XiaoMi, Inc.
 *
 * Author: Nannan Wang <wangnannan@xiaomi.com>
 *
 * Adapted for the Microsoft Lumia 950 (RM-1104), which fits a TAS2553 on
 * I2C-2 fed from Quaternary MI2S. The TAS2553 is register compatible.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include "tas2552.h"

#define TAS2552_PLL_CLK_48000		24576000
#define TAS2552_PLL_CLK_44100		22579200

/*
 * The PGA is specified in 1 dB steps starting at -7 dB, so register value
 * N corresponds to (N - 7) dB.
 *
 * The chip's own default is 0x16 (15 dB). At that gain the boost converter's
 * current peaks sag a ten year old cell far enough for the PMIC to
 * power-cycle the device mid-playback, with nothing logged - reproducible on
 * a Lumia 950 by driving the loudspeaker at full volume. 11 dB is loud
 * enough and has proven stable; boards with a healthy battery can raise it
 * via the "ti,pga-gain" property.
 *
 * The brownout happened with the chip's Battery Tracking AGC switched off
 * (LIM_EN, CONFIG2 bit 2, cleared by the 0xE3 written at probe). The AGC is
 * the hardware feature TI provides for exactly this case (SLAS978B 7.3.10,
 * 9.3): once VBAT drops below the inflection point (0x0B) the allowed peak
 * output voltage falls with the slope in 0x0C, cutting the boost input
 * current before the pack sags to the PMIC undervoltage trip. It is enabled
 * with "ti,limiter-enable" and parametrised with the "ti,battery-guard-*"
 * and "ti,limiter-*" properties; without them the register defaults stand
 * and behaviour is unchanged.
 */
#define TAS2552_PGA_GAIN_DEFAULT	0x12

struct tas2552_priv {
	struct snd_soc_codec *codec;
	unsigned int sysclk;
	int enable_gpio;
	u8 pga_gain;

	/* Battery Tracking AGC ("battery guard" + limiter), see tas2552.h */
	bool limiter_enable;
	int bg_inflection;	/* register 0x0B code, -1 = leave default */
	int bg_slope;		/* register 0x0C code, -1 = leave default */
	int lim_attack;		/* 0x0E ATTACK_TIME[2:0], -1 = leave default */
	int lim_release;	/* 0x0F REL_TIME[3:0], -1 = leave default */
};

/* register 0x19 code -> millivolts, 0x50 = 2500 mV, 17.33 mV per LSB */
static int tas2552_vbat_code_to_mv(unsigned int code)
{
	return TAS2552_VBAT_DATA_MV_MIN +
	       (int)(code - TAS2552_VBAT_DATA_CODE_MIN) *
	       TAS2552_BG_STEP_UV / 1000;
}

static int tas2552_read_vbat(struct tas2552_priv *tas2552, unsigned int *code)
{
	unsigned int val;

	if (!tas2552->codec)
		return -ENODEV;

	val = snd_soc_read(tas2552->codec, TAS2552_REG_VBAT_DATA);
	if (val > 0xFF)
		return -EIO;

	*code = val;
	return 0;
}

/*
 * /sys/bus/i2c/devices/<bus>-0040/vbat_mv: what the amplifier's own battery
 * monitor sees, so the guard can be checked against the fuel gauge. Reads 0
 * while the device is in software shutdown (code below 0x50 is reserved and
 * the datasheet says VBAT data is only valid while the AGC is running).
 */
static ssize_t tas2552_vbat_mv_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct tas2552_priv *tas2552 = dev_get_drvdata(dev);
	unsigned int code;
	int ret;

	ret = tas2552_read_vbat(tas2552, &code);
	if (ret)
		return ret;

	if (code < TAS2552_VBAT_DATA_CODE_MIN)
		return scnprintf(buf, PAGE_SIZE, "0\n");

	return scnprintf(buf, PAGE_SIZE, "%d\n", tas2552_vbat_code_to_mv(code));
}
static DEVICE_ATTR(vbat_mv, S_IRUGO, tas2552_vbat_mv_show, NULL);

static ssize_t tas2552_vbat_raw_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct tas2552_priv *tas2552 = dev_get_drvdata(dev);
	unsigned int code;
	int ret;

	ret = tas2552_read_vbat(tas2552, &code);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", code);
}
static DEVICE_ATTR(vbat_raw, S_IRUGO, tas2552_vbat_raw_show, NULL);

static struct attribute *tas2552_attrs[] = {
	&dev_attr_vbat_mv.attr,
	&dev_attr_vbat_raw.attr,
	NULL,
};

static const struct attribute_group tas2552_attr_group = {
	.attrs = tas2552_attrs,
};

static int tas2552_set_pll_clk(struct snd_soc_codec *codec,
				unsigned int sample_rate)
{
	struct tas2552_priv *tas2552 = snd_soc_codec_get_drvdata(codec);
	unsigned int j = 0, d = 0, p;
	u64 jd;
	unsigned int target_clk;
	unsigned int value;

	target_clk = (sample_rate == 48000) ?
			TAS2552_PLL_CLK_48000 : TAS2552_PLL_CLK_44100;

	if (tas2552->sysclk == target_clk) {
		value = 1 << TAS2552_PLLCTRL2_BYPASS_POS;
		snd_soc_update_bits(codec, TAS2552_REG_PLLCTRL2,
			TAS2552_PLLCTRL2_BYPASS_MSK, value);
		return 0;
	}

	/* PLL_CLK = 0.5 * PLL_CLKIN * J.D / 2^P, see the TAS2552 datasheet */
	for (p = 0; p <= 1; p++) {
		jd = ((u64)target_clk << (p + 1)) * 10000;
		do_div(jd, tas2552->sysclk);
		d = do_div(jd, 10000);
		j = jd;

		if (j < 4 || j > 96 || d > 9999)
			continue;

		if (d == 0) {
			if ((tas2552->sysclk / (1 << p)) >= 512000 &&
			    (tas2552->sysclk / (1 << p)) <= 12288000)
				break;
		} else {
			if ((tas2552->sysclk / (1 << p)) >= 1100000 &&
			    (tas2552->sysclk / (1 << p)) <= 9200000)
				break;
		}
	}

	if (p > 1) {
		dev_err(codec->dev,
			"no PLL solution for sysclk %u, sample rate %u\n",
			tas2552->sysclk, sample_rate);
		return -EINVAL;
	}

	dev_dbg(codec->dev, "PLL J=%u P=%u D=%u\n", j, p, d);

	snd_soc_update_bits(codec, TAS2552_REG_PLLCTRL2,
		TAS2552_PLLCTRL2_BYPASS_MSK, 0);

	value = ((p << TAS2552_PLLCTRL1_P_POS) & TAS2552_PLLCTRL1_P_MSK) |
		(j & TAS2552_PLLCTRL1_J_MSK);
	snd_soc_update_bits(codec, TAS2552_REG_PLLCTRL1,
		TAS2552_PLLCTRL1_J_MSK | TAS2552_PLLCTRL1_P_MSK, value);

	snd_soc_update_bits(codec, TAS2552_REG_PLLCTRL3,
		TAS2552_PLLCTRL3_D_BIT7_0_MSK, d & 0xFF);
	snd_soc_update_bits(codec, TAS2552_REG_PLLCTRL2,
		TAS2552_PLLCTRL2_D_BIT13_8_MSK, (d >> 8) & 0x3F);

	return 0;
}

static int tas2552_codec_probe(struct snd_soc_codec *codec)
{
	struct tas2552_priv *tas2552 = dev_get_drvdata(codec->dev);
	int ret;

	snd_soc_codec_set_drvdata(codec, tas2552);

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret < 0) {
		dev_err(codec->dev, "failed to set cache I/O: %d\n", ret);
		return ret;
	}

	snd_soc_write(codec, TAS2552_REG_CONFIG1, 0x02);
	/*
	 * 0xE3: CLASSD_EN, BOOST_EN, APT_EN, IVSENSE_EN and the bit 0 that the
	 * initialisation sequence clears at bias-on. PLL_EN is raised at
	 * bias-on. LIM_EN (bit 2) only when the DT asks for the battery guard.
	 */
	snd_soc_write(codec, TAS2552_REG_CONFIG2, 0xE3 |
		      (tas2552->limiter_enable ? TAS2552_CONFIG2_LIM_EN : 0));
	/*
	 * 0x5D selects 44.1/48 kHz word clock and, importantly, leaves the
	 * analog input select bit clear so the I2S data reaches the speaker.
	 */
	snd_soc_write(codec, TAS2552_REG_CONFIG3, 0x5D);
	snd_soc_write(codec, TAS2552_REG_OUTPUT_DATA, 0xC8);
	snd_soc_write(codec, TAS2552_REG_PGA_GAIN, tas2552->pga_gain);
	snd_soc_write(codec, TAS2552_REG_BOOST_AUTO_PASS_THROUGH_CTRL, 0x0F);

	/*
	 * Battery Tracking AGC parameters (SLAS978B 7.5.13 - 7.5.17). These
	 * belong to the "configure device register" part of the init sequence
	 * (8.3); the 0x0D / 0x0E[5] / 0x02[0] / 0x01[1] tail is written in
	 * tas2552_set_bias_level() when the stream starts.
	 */
	if (tas2552->bg_inflection >= 0)
		snd_soc_write(codec, TAS2552_REG_BATTERY_GUARD_INFLECTION_PT,
			      tas2552->bg_inflection);
	if (tas2552->bg_slope >= 0)
		snd_soc_write(codec, TAS2552_REG_BATTERY_GUARD_SLOPE_CTRL,
			      tas2552->bg_slope);
	if (tas2552->lim_attack >= 0)
		snd_soc_update_bits(codec, TAS2552_REG_LIMITER_AR_HT,
				    TAS2552_LIMITER_ATTACK_TIME_MSK,
				    tas2552->lim_attack);
	if (tas2552->lim_release >= 0)
		snd_soc_update_bits(codec, TAS2552_REG_LIMITER_RELEASE_RATE,
				    TAS2552_LIMITER_RELEASE_TIME_MSK,
				    tas2552->lim_release);

	tas2552->codec = codec;

	ret = snd_soc_read(codec, TAS2552_REG_VERSION_NUMBER);
	dev_info(codec->dev,
		 "silicon version 0x%x, battery guard %s (0x0B=0x%02x 0x0C=0x%02x 0x0E=0x%02x 0x0F=0x%02x)\n",
		 ret & TAS2552_VERSION_SILICON_VER_MSK,
		 tas2552->limiter_enable ? "on" : "off",
		 snd_soc_read(codec, TAS2552_REG_BATTERY_GUARD_INFLECTION_PT),
		 snd_soc_read(codec, TAS2552_REG_BATTERY_GUARD_SLOPE_CTRL),
		 snd_soc_read(codec, TAS2552_REG_LIMITER_AR_HT),
		 snd_soc_read(codec, TAS2552_REG_LIMITER_RELEASE_RATE));

	return 0;
}

static int tas2552_codec_remove(struct snd_soc_codec *codec)
{
	struct tas2552_priv *tas2552 = snd_soc_codec_get_drvdata(codec);

	tas2552->codec = NULL;

	if (gpio_is_valid(tas2552->enable_gpio))
		gpio_set_value(tas2552->enable_gpio, 0);

	return 0;
}

/* Read-only status registers must bypass the ASoC register cache. */
static int tas2552_volatile_register(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	switch (reg) {
	case TAS2552_REG_DEVICE_STATUS:
	case TAS2552_REG_VERSION_NUMBER:
	case TAS2552_REG_VBOOST_DATA:
	case TAS2552_REG_VBAT_DATA:
		return 1;
	default:
		return 0;
	}
}

static int tas2552_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	dev_dbg(codec->dev, "bias level %d\n", level);

	switch (level) {
	case SND_SOC_BIAS_ON:
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG1,
			TAS2552_CONFIG1_MUTE_MSK, TAS2552_CONFIG1_MUTE);
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG3,
			TAS2552_CONFIG3_SOURCE_SELECT_MSK,
			TAS2552_CONFIG3_SOURCE_SELECT_NONE);
		/* TAS2553 initialisation value (SLAS978B 8.3); TAS2552 is 0xC0 */
		snd_soc_write(codec, TAS2552_REG_LIMITER_LEVEL_CTRL,
			TAS2553_LIMITER_LEVEL_CTRL_INIT_EN);
		snd_soc_update_bits(codec, TAS2552_REG_LIMITER_AR_HT,
			TAS2552_LIMITER_AR_HT_INIT_MSK,
			TAS2552_LIMITER_AR_HT_INIT_EN);
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG2,
			TAS2552_CONFIG2_INIT_MSK,
			TAS2552_CONFIG2_INIT_EN);
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG2,
			TAS2552_CONFIG2_PLL_EN_MSK,
			TAS2552_CONFIG2_PLL_EN_ENABLE);
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG1,
			TAS2552_CONFIG1_SWS_MSK, 0);
		break;
	default:
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG2,
			TAS2552_CONFIG2_PLL_EN_MSK, 0);
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG1,
			TAS2552_CONFIG1_SWS_MSK, TAS2552_CONFIG1_SWS);
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG2,
			TAS2552_CONFIG2_INIT_MSK,
			TAS2552_CONFIG2_INIT_DEFAULT);
		snd_soc_update_bits(codec, TAS2552_REG_LIMITER_AR_HT,
			TAS2552_LIMITER_AR_HT_INIT_MSK,
			TAS2552_LIMITER_AR_HT_INIT_DEFAULT);
		snd_soc_write(codec, TAS2552_REG_LIMITER_LEVEL_CTRL,
			TAS2552_LIMITER_LEVEL_CTRL_INIT_DEFAULT);
		break;
	}

	codec->dapm.bias_level = level;
	return 0;
}

static const char * const tas2552_src_sel_text[] = {
	"None", "Left", "Right", "Mono",
};

static const SOC_ENUM_SINGLE_DECL(
	tas2552_src_sel_enum, TAS2552_REG_CONFIG3,
	TAS2552_CONFIG3_SOURCE_SELECT_POS, tas2552_src_sel_text);

static const DECLARE_TLV_DB_SCALE(tas2552_vol_tlv, -700, 100, 0);

static const struct snd_kcontrol_new tas2552_controls[] = {
	SOC_ENUM("TAS2552 Input Channel Mux", tas2552_src_sel_enum),
	SOC_SINGLE_TLV("TAS2552 Volume", TAS2552_REG_PGA_GAIN,
		TAS2552_PGA_GAIN_POS, TAS2552_PGA_GAIN_MAX,
		0, tas2552_vol_tlv),
	SOC_SINGLE("TAS2552 Mute", TAS2552_REG_CONFIG1,
		TAS2552_CONFIG1_MUTE_POS, TAS2552_CONFIG1_MUTE_MAX, 0),
};

static const struct snd_soc_dapm_widget tas2552_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", NULL),
};

static const struct snd_soc_dapm_route tas2552_routes[] = {
	{ "Int Spk", NULL, "Playback" },
};

static const u8 tas2552_reg_defaults[TAS2552_REG_MAX] = {
	[TAS2552_REG_DEVICE_STATUS]			= 0x00,
	[TAS2552_REG_CONFIG1]				= 0x22,
	[TAS2552_REG_CONFIG2]				= 0xFF,
	[TAS2552_REG_CONFIG3]				= 0x80,
	[TAS2552_REG_DOUT_TRISTATE_MODE]		= 0x00,
	[TAS2552_REG_I2SCTRL1]				= 0x00,
	[TAS2552_REG_I2SCTRL2]				= 0x00,
	[TAS2552_REG_OUTPUT_DATA]			= 0xC0,
	[TAS2552_REG_PLLCTRL1]				= 0x10,
	[TAS2552_REG_PLLCTRL2]				= 0x00,
	[TAS2552_REG_PLLCTRL3]				= 0x00,
	[TAS2552_REG_BATTERY_GUARD_INFLECTION_PT]	= 0x8F,
	[TAS2552_REG_BATTERY_GUARD_SLOPE_CTRL]		= 0x80,
	[TAS2552_REG_LIMITER_LEVEL_CTRL]		= 0xBE,
	[TAS2552_REG_LIMITER_AR_HT]			= 0x08,
	[TAS2552_REG_LIMITER_RELEASE_RATE]		= 0x05,
	[TAS2552_REG_LIMITER_INTEGRATION_COUNT_CTRL]	= 0x00,
	[TAS2552_REG_PDM_CONFIG]			= 0x01,
	[TAS2552_REG_PGA_GAIN]				= 0x00,
	[TAS2552_REG_CLASS_D_EDGE_RATE_CTRL]		= 0x40,
	[TAS2552_REG_BOOST_AUTO_PASS_THROUGH_CTRL]	= 0x00,
	[TAS2552_REG_RESERVED]				= 0x00,
	[TAS2552_REG_VERSION_NUMBER]			= 0x00,
	[TAS2552_REG_INTERRUPT_MASK]			= 0x00,
	[TAS2552_REG_VBOOST_DATA]			= 0x00,
	[TAS2552_REG_VBAT_DATA]				= 0x00,
};

static const struct snd_soc_codec_driver tas2552_codec_drv = {
	.probe = tas2552_codec_probe,
	.remove = tas2552_codec_remove,
	.controls = tas2552_controls,
	.num_controls = ARRAY_SIZE(tas2552_controls),
	.dapm_widgets = tas2552_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas2552_dapm_widgets),
	.dapm_routes = tas2552_routes,
	.num_dapm_routes = ARRAY_SIZE(tas2552_routes),
	.reg_cache_size = ARRAY_SIZE(tas2552_reg_defaults),
	.reg_word_size = sizeof(tas2552_reg_defaults[0]),
	.reg_cache_default = tas2552_reg_defaults,
	.volatile_register = tas2552_volatile_register,
	.set_bias_level = tas2552_set_bias_level,
	.idle_bias_off = 1,
};

#define TAS2552_FORMATS		(SNDRV_PCM_FMTBIT_S16_LE |\
				SNDRV_PCM_FMTBIT_S24_LE |\
				SNDRV_PCM_FMTBIT_S32_LE)

#define TAS2552_RATES		(SNDRV_PCM_RATE_44100 |\
				SNDRV_PCM_RATE_48000)

static int tas2552_set_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tas2552_priv *tas2552 = snd_soc_codec_get_drvdata(codec);
	unsigned int value;

	dev_dbg(codec->dev, "sysclk source %d, freq %u\n", clk_id, freq);

	switch (clk_id) {
	case TAS2552_SCLK_S_MCLK:
		value = TAS2552_CONFIG1_PLL_SRC_MCLK;
		break;
	case TAS2552_SCLK_S_BCLK:
		value = TAS2552_CONFIG1_PLL_SRC_BCLK;
		break;
	case TAS2552_SCLK_S_IVCLKIN:
		value = TAS2552_CONFIG1_PLL_SRC_IVCLKIN;
		break;
	case TAS2552_SCLK_S_INTERNAL_1P8:
		value = TAS2552_CONFIG1_PLL_SRC_INTERNAL_1P8;
		break;
	default:
		dev_err(codec->dev, "unknown clock source %d\n", clk_id);
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_REG_CONFIG1,
		TAS2552_CONFIG1_PLL_SRC_MSK, value);

	tas2552->sysclk = freq;
	return 0;
}

static int tas2552_set_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned int value;

	dev_dbg(codec->dev, "dai format 0x%x\n", fmt);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		value = TAS2552_I2SCTRL1_PCM_DATAFMT_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		value = TAS2552_I2SCTRL1_PCM_DATAFMT_RJF;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		value = TAS2552_I2SCTRL1_PCM_DATAFMT_LJF;
		break;
	default:
		dev_err(codec->dev, "invalid interface format\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_REG_I2SCTRL1,
		TAS2552_I2SCTRL1_PCM_DATAFMT_MSK, value);

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF) {
		dev_err(codec->dev, "invalid clock inversion\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		value = 0;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		value = TAS2552_I2SCTRL1_PCM_BCLKDIR_OUTPUT |
			TAS2552_I2SCTRL1_PCM_WCLKDIR_OUTPUT;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		value = TAS2552_I2SCTRL1_PCM_WCLKDIR_OUTPUT;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		value = TAS2552_I2SCTRL1_PCM_BCLKDIR_OUTPUT;
		break;
	default:
		dev_err(codec->dev, "invalid master/slave setting\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_REG_I2SCTRL1,
		TAS2552_I2SCTRL1_PCM_BCLKDIR_MSK |
		TAS2552_I2SCTRL1_PCM_WCLKDIR_MSK, value);

	return 0;
}

static int tas2552_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "mute %d\n", mute);

	snd_soc_update_bits(codec, TAS2552_REG_CONFIG1,
			TAS2552_CONFIG1_MUTE_MSK,
			mute ? TAS2552_CONFIG1_MUTE : 0);

	if (!mute) {
		/* let the boost converter settle before unmuting the mixer */
		usleep_range(10000, 11000);
		snd_soc_update_bits(codec, TAS2552_REG_CONFIG3,
				TAS2552_CONFIG3_SOURCE_SELECT_MSK,
				TAS2552_CONFIG3_SOURCE_SELECT_MONO);
	}

	return 0;
}

static int tas2552_hw_params(struct snd_pcm_substream *substream,
			struct snd_pcm_hw_params *params,
			struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	unsigned int value;

	dev_dbg(codec->dev, "format %d, rate %d\n",
		params_format(params), params_rate(params));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		value = TAS2552_I2SCTRL1_PCM_FMT_16 |
			TAS2552_I2SCTRL1_PCM_BCLK_32;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		value = TAS2552_I2SCTRL1_PCM_FMT_24 |
			TAS2552_I2SCTRL1_PCM_BCLK_64;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		value = TAS2552_I2SCTRL1_PCM_FMT_32 |
			TAS2552_I2SCTRL1_PCM_BCLK_64;
		break;
	default:
		dev_err(codec->dev, "unsupported sample format\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_REG_I2SCTRL1,
		TAS2552_I2SCTRL1_PCM_FMT_MSK | TAS2552_I2SCTRL1_PCM_BCLK_MSK,
		value);

	switch (params_rate(params)) {
	case 48000:
	case 44100:
		value = TAS2552_CONFIG3_WCLK_44100_48000;
		break;
	default:
		dev_err(codec->dev, "unsupported sample rate\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TAS2552_REG_CONFIG3,
			TAS2552_CONFIG3_WCLK_MSK, value);

	return tas2552_set_pll_clk(codec, params_rate(params));
}

static int tas2552_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	snd_soc_dapm_enable_pin(&dai->codec->dapm, "Int Spk");
	return 0;
}

static void tas2552_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	snd_soc_dapm_disable_pin(&dai->codec->dapm, "Int Spk");
}

static const struct snd_soc_dai_ops tas2552_dai_ops = {
	.startup = tas2552_startup,
	.shutdown = tas2552_shutdown,
	.set_sysclk = tas2552_set_sysclk,
	.set_fmt = tas2552_set_fmt,
	.digital_mute = tas2552_digital_mute,
	.hw_params = tas2552_hw_params,
};

static struct snd_soc_dai_driver tas2552_dai = {
	.name = "tas2552-dai",
	.ops = &tas2552_dai_ops,
	.playback = {
		.stream_name = "Playback",
		.formats = TAS2552_FORMATS,
		.rates = TAS2552_RATES,
		.channels_min = 2,
		.channels_max = 2,
	},
	.symmetric_rates = 1,
};

static int tas2552_parse_dt(struct device *dev, struct tas2552_priv *tas2552)
{
	u32 gain, val;
	int ret;

	tas2552->enable_gpio = of_get_named_gpio(dev->of_node,
						 "ti,enable-gpio", 0);
	if (tas2552->enable_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!gpio_is_valid(tas2552->enable_gpio)) {
		dev_err(dev, "ti,enable-gpio missing or invalid (%d)\n",
			tas2552->enable_gpio);
		return -EINVAL;
	}

	tas2552->pga_gain = TAS2552_PGA_GAIN_DEFAULT;
	ret = of_property_read_u32(dev->of_node, "ti,pga-gain", &gain);
	if (!ret) {
		if (gain > TAS2552_PGA_GAIN_MAX) {
			dev_err(dev, "ti,pga-gain %u out of range\n", gain);
			return -EINVAL;
		}
		tas2552->pga_gain = gain;
	}

	tas2552->limiter_enable = of_property_read_bool(dev->of_node,
							"ti,limiter-enable");
	tas2552->bg_inflection = -1;
	tas2552->bg_slope = -1;
	tas2552->lim_attack = -1;
	tas2552->lim_release = -1;

	/* 0x0B INFLECTION: 0x6D = 3000 mV, 17.33 mV per step, 0xFE = 5500 mV */
	ret = of_property_read_u32(dev->of_node,
				   "ti,battery-guard-inflection-mv", &val);
	if (!ret) {
		if (val < TAS2552_BG_INFLECTION_MV_MIN ||
		    val > TAS2552_BG_INFLECTION_MV_MAX) {
			dev_err(dev, "ti,battery-guard-inflection-mv %u out of range (3000-5500)\n",
				val);
			return -EINVAL;
		}
		tas2552->bg_inflection = TAS2552_BG_INFLECTION_CODE_MIN +
			DIV_ROUND_CLOSEST((val - TAS2552_BG_INFLECTION_MV_MIN) *
					  1000, TAS2552_BG_STEP_UV);
		if (tas2552->bg_inflection > TAS2552_BG_INFLECTION_CODE_MAX)
			tas2552->bg_inflection = TAS2552_BG_INFLECTION_CODE_MAX;
	}

	/* 0x0C SLOPE: 0x00 = 1.2 V/V, 37.3 mV/V per step, 0xFF = 10.75 V/V */
	ret = of_property_read_u32(dev->of_node,
				   "ti,battery-guard-slope-mv-per-v", &val);
	if (!ret) {
		if (val < TAS2552_BG_SLOPE_MVV_MIN ||
		    val > TAS2552_BG_SLOPE_MVV_MAX) {
			dev_err(dev, "ti,battery-guard-slope-mv-per-v %u out of range (1200-10750)\n",
				val);
			return -EINVAL;
		}
		tas2552->bg_slope = DIV_ROUND_CLOSEST(
			(val - TAS2552_BG_SLOPE_MVV_MIN) * 1000,
			TAS2552_BG_SLOPE_STEP_UVV);
		if (tas2552->bg_slope > 0xFF)
			tas2552->bg_slope = 0xFF;
	}

	/* 0x0E ATTACK_TIME[2:0]: 20 us/dB + 350 us/dB per step, max 2470 */
	ret = of_property_read_u32(dev->of_node,
				   "ti,limiter-attack-us-per-db", &val);
	if (!ret) {
		if (val < TAS2552_LIMITER_ATTACK_US_MIN ||
		    val > TAS2552_LIMITER_ATTACK_US_MIN +
			  TAS2552_LIMITER_ATTACK_CODE_MAX *
			  TAS2552_LIMITER_ATTACK_US_STEP) {
			dev_err(dev, "ti,limiter-attack-us-per-db %u out of range (20-2470)\n",
				val);
			return -EINVAL;
		}
		tas2552->lim_attack = DIV_ROUND_CLOSEST(
			val - TAS2552_LIMITER_ATTACK_US_MIN,
			TAS2552_LIMITER_ATTACK_US_STEP);
	}

	/* 0x0F REL_TIME[3:0]: 50 ms/dB + 105 ms/dB per step, max 1625 */
	ret = of_property_read_u32(dev->of_node,
				   "ti,limiter-release-ms-per-db", &val);
	if (!ret) {
		if (val < TAS2552_LIMITER_RELEASE_MS_MIN ||
		    val > TAS2552_LIMITER_RELEASE_MS_MIN +
			  TAS2552_LIMITER_RELEASE_CODE_MAX *
			  TAS2552_LIMITER_RELEASE_MS_STEP) {
			dev_err(dev, "ti,limiter-release-ms-per-db %u out of range (50-1625)\n",
				val);
			return -EINVAL;
		}
		tas2552->lim_release = DIV_ROUND_CLOSEST(
			val - TAS2552_LIMITER_RELEASE_MS_MIN,
			TAS2552_LIMITER_RELEASE_MS_STEP);
	}

	return 0;
}

static int tas2552_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct tas2552_priv *tas2552;
	int ret;

	if (!client->dev.of_node) {
		dev_err(&client->dev, "no device tree node\n");
		return -EINVAL;
	}

	tas2552 = devm_kzalloc(&client->dev, sizeof(*tas2552), GFP_KERNEL);
	if (!tas2552)
		return -ENOMEM;

	ret = tas2552_parse_dt(&client->dev, tas2552);
	if (ret)
		return ret;

	ret = devm_gpio_request_one(&client->dev, tas2552->enable_gpio,
				    GPIOF_OUT_INIT_HIGH, "tas2552-enable");
	if (ret) {
		dev_err(&client->dev, "failed to request enable gpio: %d\n",
			ret);
		return ret;
	}

	/* the chip needs its supplies to come up before it answers on I2C */
	usleep_range(1000, 2000);

	i2c_set_clientdata(client, tas2552);

	ret = snd_soc_register_codec(&client->dev, &tas2552_codec_drv,
				     &tas2552_dai, 1);
	if (ret) {
		dev_err(&client->dev, "failed to register codec: %d\n", ret);
		gpio_set_value(tas2552->enable_gpio, 0);
		return ret;
	}

	ret = sysfs_create_group(&client->dev.kobj, &tas2552_attr_group);
	if (ret)
		dev_warn(&client->dev, "no vbat sysfs files: %d\n", ret);

	dev_info(&client->dev, "TAS2552 registered, PGA gain %d dB, battery guard %s\n",
		 tas2552->pga_gain - 7,
		 tas2552->limiter_enable ? "enabled" : "disabled");
	return 0;
}

static int tas2552_i2c_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &tas2552_attr_group);
	snd_soc_unregister_codec(&client->dev);
	return 0;
}

static const struct of_device_id tas2552_of_match[] = {
	{ .compatible = "ti,tas2552" },
	{ },
};
MODULE_DEVICE_TABLE(of, tas2552_of_match);

static const struct i2c_device_id tas2552_i2c_id[] = {
	{ "tas2552", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, tas2552_i2c_id);

static struct i2c_driver tas2552_i2c_driver = {
	.driver = {
		.name = "tas2552",
		.owner = THIS_MODULE,
		.of_match_table = tas2552_of_match,
	},
	.probe = tas2552_i2c_probe,
	.remove = tas2552_i2c_remove,
	.id_table = tas2552_i2c_id,
};

module_i2c_driver(tas2552_i2c_driver);

MODULE_AUTHOR("Nannan Wang <wangnannan@xiaomi.com>");
MODULE_DESCRIPTION("TI TAS2552 smart amplifier driver");
MODULE_LICENSE("GPL");
