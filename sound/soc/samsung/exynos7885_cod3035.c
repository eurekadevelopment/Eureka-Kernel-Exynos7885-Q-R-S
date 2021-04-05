/*
 *  Driver for COD3035 CODECs on Exynos7885
 *
 *  Copyright 2013 Wolfson Microelectronics
 *  Copyright 2016 Cirrus Logic
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/debugfs.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/mfd/madera/core.h>
#include <linux/extcon/extcon-madera.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <soc/samsung/exynos-pmu.h>
#include <sound/samsung/abox.h>
#include <sound/cod3035x.h>
#if defined(CONFIG_SND_SOC_DBMDX)
#include <sound/dbmdx-export.h>
#endif
#include "../codecs/madera.h"

#include "jack_cod3035_sysfs_cb.h"

#define EXYNOS_PMU_PMU_DEBUG_OFFSET		(0x0A00)
#define SAMSUNG_CODEC_DAI_OFFSET			(13)

#ifdef CONFIG_SOC_EXYNOS8895
/* Used for debugging and test automation */
static u32 voice_trigger_count;
#endif

enum {
	MB_NONE,
	MB_INT_BIAS1,
	MB_INT_BIAS2,
	MB_EXT_GPIO,
	MB_EXT_LDO,
	MB_MAX,
};

/* codec internal input */
enum {
	INT_MIC1,
	INT_MIC2,
	INT_MIC3,
	INT_LINEIN,
	INT_INPUT_MAX,
};

struct universal7885_mic_bias {
	int mode[INT_INPUT_MAX];
	int gpio[INT_INPUT_MAX];
};

struct universal7885_mic_bias_count {
	atomic_t use_count[MB_MAX];
};

struct clk_conf {
	int id;
	int source;
	int rate;
};

struct exynos7885_drvdata {
	struct device *dev;

	struct clk_conf fll1_refclk;
	struct clk_conf sysclk;
	struct clk_conf dspclk;

	struct notifier_block nb;

	struct snd_soc_codec *codec;
	int aifrate;
	struct universal7885_mic_bias mic_bias;
	struct universal7885_mic_bias_count mic_bias_count;
	bool use_external_jd;
	struct regulator *vdd;
};

static struct exynos7885_drvdata exynos7885_drvdata;

static const struct snd_soc_ops rdma_ops = {
};

static const struct snd_soc_ops wdma_ops = {
};

static const struct snd_soc_ops uaif_ops = {
};

static int exynos7885_set_bias_level(struct snd_soc_card *card,
				  struct snd_soc_dapm_context *dapm,
				  enum snd_soc_bias_level level)
{

	return 0;
}

static int exynos7885_set_bias_level_post(struct snd_soc_card *card,
				       struct snd_soc_dapm_context *dapm,
				       enum snd_soc_bias_level level)
{

	return 0;
}

static void universal7885_ext_gpio_bias_ev(struct snd_soc_card *card,
				int event, int gpio)
{
	dev_dbg(card->dev, "%s Called: %d, ext mic bias gpio %s\n", __func__,
			event, gpio_is_valid(gpio) ?
			"valid" : "invalid");

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (gpio_is_valid(gpio))
			gpio_set_value(gpio, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		if (gpio_is_valid(gpio))
			gpio_set_value(gpio, 0);
		break;
	}
}

static void universal7885_ext_ldo_bias_ev(struct snd_soc_card *card,
                int event)
{
	struct exynos7885_drvdata *drvdata = card->drvdata;
	int ret;
    dev_dbg(card->dev, "%s Called\n", __func__);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = regulator_enable(drvdata->vdd);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = regulator_disable(drvdata->vdd);
		break;
	}
}

static int universal7885_int_bias1_ev(struct snd_soc_card *card,
				int event)
{
	struct exynos7885_drvdata *drvdata = card->drvdata;

	dev_info(card->dev, "%s called\n", __func__);
	return cod3035x_mic_bias_ev(drvdata->codec, COD3035X_MICBIAS1, event);
}

static int universal7885_int_bias2_ev(struct snd_soc_card *card,
				int event)
{
	struct exynos7885_drvdata *drvdata = card->drvdata;

	dev_info(card->dev, "%s called\n", __func__);
	return cod3035x_mic_bias_ev(drvdata->codec, COD3035X_MICBIAS2, event);
}

static int universal7885_configure_mic_bias(struct snd_soc_card *card,
		int index, int event)
{
	struct exynos7885_drvdata *drvdata = card->drvdata;
	int process_event = 0;
	int mode_index = drvdata->mic_bias.mode[index];

	pr_err("[3035]%s called: index: %d, event: %d\n", __func__, index, event);
	/* validate the bias mode */
	if ((mode_index < MB_INT_BIAS1) || (mode_index > MB_EXT_LDO))
		return 0;

	/* decrement the bias mode index to match use count buffer
	 * because use count buffer size is 0-2, and the mode is 1-3
	 * so decrement, and this veriable should be used only for indexing
	 * use_count.
	 */
	mode_index--;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		atomic_inc(&drvdata->mic_bias_count.use_count[mode_index]);
		if (atomic_read(&drvdata->mic_bias_count.use_count[mode_index]) == 1)
			process_event = 1;
		break;

	case SND_SOC_DAPM_POST_PMD:
		atomic_dec(&drvdata->mic_bias_count.use_count[mode_index]);
		if (atomic_read(&drvdata->mic_bias_count.use_count[mode_index]) == 0)
			process_event = 1;
		break;

	default:
		break;
	}

	if (!process_event)
		return 0;

	switch(drvdata->mic_bias.mode[index]) {
	case MB_INT_BIAS1:
		universal7885_int_bias1_ev(card, event);
		break;
	case MB_INT_BIAS2:
		universal7885_int_bias2_ev(card, event);
		break;
	case MB_EXT_GPIO:
		universal7885_ext_gpio_bias_ev(card, event,
				drvdata->mic_bias.gpio[index]);
		break;
	case MB_EXT_LDO:
		universal7885_ext_ldo_bias_ev(card, event);
	default:
		break;
	};

	return 0;
}

static int universal7885_mic1_bias(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	return universal7885_configure_mic_bias(w->dapm->card, INT_MIC1, event);
}

static int universal7885_mic2_bias(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	return universal7885_configure_mic_bias(w->dapm->card, INT_MIC2, event);
}

static int universal7885_mic3_bias(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	return universal7885_configure_mic_bias(w->dapm->card, INT_MIC3, event);
}

static int universal7885_linein_bias(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	return universal7885_configure_mic_bias(w->dapm->card, INT_LINEIN, event);
}


static int universal7885_request_ext_mic_bias_en_gpio(struct snd_soc_card *card)
{
	struct exynos7885_drvdata *drvdata = card->drvdata;
	struct device *dev = card->dev;
	int ret;
	int gpio;
	int i;
	char gpio_name[32];

	dev_dbg(dev, "%s called\n", __func__);

	for (i = 0; i < INT_INPUT_MAX; i++) {
		gpio = drvdata->mic_bias.gpio[i];

		/* This is optional GPIO, don't report if not present */
		if (!gpio_is_valid(gpio))
			continue;
		sprintf(gpio_name, "ext_mic_bias-%d", i);

		ret = devm_gpio_request_one(dev, gpio,
				GPIOF_OUT_INIT_LOW, gpio_name);

		if (ret < 0) {
			dev_err(dev, "Ext-MIC bias GPIO request failed\n");
			continue;
		}
		gpio_direction_output(gpio, 0);
	}

	return 0;
}

static int universal7885_init_soundcard(struct snd_soc_card *card)
{
	struct exynos7885_drvdata *drvdata = card->drvdata;
	int ret;
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *aif0_dai;
	struct snd_soc_codec *codec;

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[SAMSUNG_CODEC_DAI_OFFSET].name);
	aif0_dai = rtd->codec_dai;
	codec = aif0_dai->codec;

	drvdata->codec = codec;

	ret = universal7885_request_ext_mic_bias_en_gpio(card);
	if (ret)
		dev_warn(codec->dev, "Failed to get ext mic bias gpios :%d\n",
								ret);
	return 0;
}

#ifdef CONFIG_DEBUG_FS
#ifdef CONFIG_SOC_EXYNOS8895
static void exynos7885_init_debugfs(struct snd_soc_card *card)
{
	struct dentry *root;

	if (!card->debugfs_card_root) {
		dev_warn(card->dev, "No card debugfs root\n");
		return;
	}

	root = debugfs_create_dir("test-automation", card->debugfs_card_root);
	if (!root) {
		dev_warn(card->dev, "Failed to create debugfs dir\n");
		return;
	}

	debugfs_create_u32("voice_trigger_count", S_IRUGO, root,
			   &voice_trigger_count);
}
#endif
#else
static void arndale_init_debugfs(struct snd_soc_card *card)
{
}
#endif

const struct snd_soc_dapm_route universal7885_dapm_routes[] = {
	{"DMIC1_PGA", NULL, "MIC1 Bias"},
	{"MIC1_PGA", NULL, "MIC1 Bias"},
	{"MIC1 Bias", NULL, "IN1L"},

	{"DMIC2_PGA", NULL, "MIC2 Bias"},
	{"MIC2_PGA", NULL, "MIC2 Bias"},
	{"MIC2 Bias", NULL, "IN2L"},

	{"MIC3_PGA", NULL, "MIC3 Bias"},
	{"MIC3 Bias", NULL, "IN3L"},

	{"LINEIN_PGA", NULL, "LINEIN Bias"},
	{"LINEIN Bias", NULL, "IN4L" },
};

static int exynos7885_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *aif1_dai;
	struct snd_soc_codec *cpu;

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link[0].name);
	aif1_dai = rtd->cpu_dai;
	cpu = aif1_dai->codec;


	snd_soc_dapm_ignore_suspend(&card->dapm, "VOUTPUT");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VOUTPUTCALL");
	snd_soc_dapm_ignore_suspend(&card->dapm, "VINPUTCALL");
	snd_soc_dapm_ignore_suspend(&card->dapm, "MIC1 Bias");
	snd_soc_dapm_ignore_suspend(&card->dapm, "MIC2 Bias");
	snd_soc_dapm_ignore_suspend(&card->dapm, "MIC3 Bias");
	snd_soc_dapm_ignore_suspend(&card->dapm, "LINEIN Bias");
	snd_soc_dapm_ignore_suspend(&card->dapm, "SPK");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX RDMA0 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX RDMA1 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX RDMA2 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX RDMA3 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX RDMA4 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX RDMA5 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX RDMA6 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX RDMA7 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX WDMA0 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX WDMA1 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX WDMA2 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX WDMA3 Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(cpu), "ABOX WDMA4 Capture");
	snd_soc_dapm_sync(snd_soc_codec_get_dapm(cpu));

#if defined(CONFIG_SND_SOC_DBMDX)
	dbmdx_remote_add_codec_controls(cpu);
#endif

	register_cod3035_jack_cb(cpu);

	return 0;
}

static struct snd_soc_dai_link exynos7885_dai[] = {
	{
		.name = "RDMA0",
		.stream_name = "RDMA0",
		.cpu_dai_name = "RDMA0",
		.platform_name = "14a51000.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA1",
		.stream_name = "RDMA1",
		.cpu_dai_name = "RDMA1",
		.platform_name = "14a51100.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA2",
		.stream_name = "RDMA2",
		.cpu_dai_name = "RDMA2",
		.platform_name = "14a51200.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA3",
		.stream_name = "RDMA3",
		.cpu_dai_name = "RDMA3",
		.platform_name = "14a51300.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA4",
		.stream_name = "RDMA4",
		.cpu_dai_name = "RDMA4",
		.platform_name = "14a51400.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA5",
		.stream_name = "RDMA5",
		.cpu_dai_name = "RDMA5",
		.platform_name = "14a51500.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA6",
		.stream_name = "RDMA6",
		.cpu_dai_name = "RDMA6",
		.platform_name = "14a51600.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "RDMA7",
		.stream_name = "RDMA7",
		.cpu_dai_name = "RDMA7",
		.platform_name = "14a51700.abox_rdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &rdma_ops,
		.dpcm_playback = 1,
	},
	{
		.name = "WDMA0",
		.stream_name = "WDMA0",
		.cpu_dai_name = "WDMA0",
		.platform_name = "14a52000.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA1",
		.stream_name = "WDMA1",
		.cpu_dai_name = "WDMA1",
		.platform_name = "14a52100.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA2",
		.stream_name = "WDMA2",
		.cpu_dai_name = "WDMA2",
		.platform_name = "14a52200.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA3",
		.stream_name = "WDMA3",
		.cpu_dai_name = "WDMA3",
		.platform_name = "14a52300.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "WDMA4",
		.stream_name = "WDMA4",
		.cpu_dai_name = "WDMA4",
		.platform_name = "14a52400.abox_wdma",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dynamic = 1,
		.ignore_suspend = 1,
		.trigger = {SND_SOC_DPCM_TRIGGER_POST_PRE, SND_SOC_DPCM_TRIGGER_PRE_POST},
		.ops = &wdma_ops,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF0",
		.stream_name = "UAIF0",
		.cpu_dai_name = "UAIF0",
		.platform_name = "snd-soc-dummy",
		.codec_dai_name = "cod3035x-aif",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF1",
		.stream_name = "UAIF1",
		.cpu_dai_name = "UAIF1",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF2",
		.stream_name = "UAIF2",
		.cpu_dai_name = "UAIF2",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "UAIF3",
		.stream_name = "UAIF3",
		.cpu_dai_name = "UAIF3",
		.platform_name = "snd-soc-dummy",
#if !defined(CONFIG_SND_SOC_TFA9872) && !defined(CONFIG_SND_SOC_TFA9896) && !defined(CONFIG_SND_SOC_SMA1301) && !defined(CONFIG_SND_SOC_TAS2562)
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
#endif
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = abox_hw_params_fixup_helper,
		.ops = &uaif_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "ABOX Internal",
		.stream_name = "ABOX Internal",
		.cpu_dai_name = "Internal",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dai_fmt = SND_SOC_DAIFMT_PDM | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		.name = "SPEEDY",
		.stream_name = "SPEEDY",
		.cpu_dai_name = "SPEEDY",
		.platform_name = "snd-soc-dummy",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM,
		.no_pcm = 1,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.ops = &uaif_ops,
		.dpcm_capture = 1,
	},
};

static const char * const vts_output_texts[] = {
        "None",
        "DMIC",
};

static const struct soc_enum vts_output_enum =
        SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(vts_output_texts),
                        vts_output_texts);

static const struct snd_kcontrol_new vts_output_mux[] = {
        SOC_DAPM_ENUM("VTS Virtual Output Mux", vts_output_enum),
};

static const struct snd_kcontrol_new exynos7885_controls[] = {
	SOC_DAPM_PIN_SWITCH("DMIC"),
};

static struct snd_soc_dapm_widget exynos7885_widgets[] = {
	SND_SOC_DAPM_OUTPUT("VOUTPUT"),
	SND_SOC_DAPM_OUTPUT("VOUTPUTCALL"),
	SND_SOC_DAPM_OUTPUT("SPK"),
	SND_SOC_DAPM_INPUT("VINPUTCALL"),
	SND_SOC_DAPM_INPUT("VINPUTFM"),
	SND_SOC_DAPM_MIC("MIC1 Bias", universal7885_mic1_bias),
	SND_SOC_DAPM_MIC("MIC2 Bias", universal7885_mic2_bias),
	SND_SOC_DAPM_MIC("MIC3 Bias", universal7885_mic3_bias),
	SND_SOC_DAPM_MIC("LINEIN Bias", universal7885_linein_bias),
	// SND_SOC_DAPM_SPK("SPK"),
	// SND_SOC_DAPM_INPUT("VI"),
};

static struct snd_soc_codec_conf codec_conf[] = {
	{
		.name_prefix = "ABOX",
	},
#ifdef CONFIG_SOC_EXYNOS8895
	{
		.name_prefix = "VTS",
	},
#endif
};

static struct snd_soc_aux_dev aux_dev[] = {
#ifdef CONFIG_SOC_EXYNOS8895
	{
		.name = "EFFECT",
	},
#endif
};

static struct snd_soc_card exynos7885 = {
	.name = "Exynos7885-COD3035",
	.owner = THIS_MODULE,
	.dai_link = exynos7885_dai,
	.num_links = ARRAY_SIZE(exynos7885_dai),

	.late_probe = exynos7885_late_probe,

	.controls = exynos7885_controls,
	.num_controls = ARRAY_SIZE(exynos7885_controls),
	.dapm_widgets = exynos7885_widgets,
	.num_dapm_widgets = ARRAY_SIZE(exynos7885_widgets),
	.dapm_routes = universal7885_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(universal7885_dapm_routes),

	.set_bias_level = exynos7885_set_bias_level,
	.set_bias_level_post = exynos7885_set_bias_level_post,

	.drvdata = (void *)&exynos7885_drvdata,

	.codec_conf = codec_conf,
	.num_configs = ARRAY_SIZE(codec_conf),

	.aux_dev = aux_dev,
	.num_aux_devs = ARRAY_SIZE(aux_dev),
};

static int exynos7885_read_dai(struct device_node *np, const char * const prop,
			      struct device_node **dai, const char **name)
{
	int ret = 0;

	np = of_get_child_by_name(np, prop);
	if (!np)
		return -ENOENT;

	*dai = of_parse_phandle(np, "sound-dai", 0);
	if (!*dai) {
		ret = -ENODEV;
		goto out;
	}

	if (*name == NULL) {
		/* Ignoring the return as we don't register DAIs to the platform */
		ret = snd_soc_of_get_dai_name(np, name);
		if (ret && !*name)
			return ret;
	}

out:
	of_node_put(np);

	return ret;
}

#ifdef CONFIG_SOC_EXYNOS8895
static struct clk *xclkout;

static void control_xclkout(bool on)
{
	if (on) {
		clk_enable(xclkout);
	} else {
		clk_disable(xclkout);
	}
}
#endif

static void universal7885_mic_bias_parse_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct exynos7885_drvdata *drvdata = card->drvdata;
	int ret;
	int i;
	int gpio, gpio_cnt;

	dev_info(card->dev, "%s\n", __func__);

	for (i = 0; i < INT_INPUT_MAX; i++) {
		drvdata->mic_bias.mode[i] = MB_NONE;
		drvdata->mic_bias.gpio[i] = -EINVAL;
	}

	for ( i = 0; i < MB_MAX; i++)
		atomic_set(&drvdata->mic_bias_count.use_count[i], 0);

	ret = of_property_read_u32_array(np, "mic-bias-mode",
			drvdata->mic_bias.mode, INT_INPUT_MAX);
	if (ret) {
		dev_err(&pdev->dev, "Could not read `mic-bias-mode`\n");
		return;
	}

	for (i = 0, gpio_cnt = 0; i < INT_INPUT_MAX; i++) {
		if (drvdata->mic_bias.mode[i] == MB_EXT_GPIO) {
			gpio = of_get_named_gpio(np, "mic-bias-gpios",
					gpio_cnt++);
			if (gpio_is_valid(gpio))
				drvdata->mic_bias.gpio[i] = gpio;
			else
				dev_err(&pdev->dev, "Invalid mic-bias gpio\n");
		}
	}

	dev_info(&pdev->dev, "BIAS: MIC1(%d), MIC2(%d), MIC3(%d), LINEIN(%d)\n",
			drvdata->mic_bias.mode[INT_MIC1], drvdata->mic_bias.mode[INT_MIC2],
			drvdata->mic_bias.mode[INT_MIC3], drvdata->mic_bias.mode[INT_LINEIN]);
}

static int exynos7885_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &exynos7885;
	struct exynos7885_drvdata *drvdata = card->drvdata;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *dai;
	int nlink = 0, n;
	int ret;

	card->dev = &pdev->dev;
	drvdata->dev = card->dev;
	dev_info(card->dev, "%s\n", __func__);

#ifdef CONFIG_SOC_EXYNOS8895
	xclkout = devm_clk_get(&pdev->dev, "xclkout");
	if (IS_ERR(xclkout)) {
		dev_err(&pdev->dev, "xclkout get failed\n");
		return PTR_ERR(xclkout);
	}
	clk_prepare(xclkout);
#endif

	for_each_child_of_node(np, dai) {
		ret = exynos7885_read_dai(dai, "cpu",
					 &exynos7885_dai[nlink].cpu_of_node,
					 &exynos7885_dai[nlink].cpu_dai_name);
		if (ret) {
			dev_warn(card->dev,
				"Failed to parse cpu DAI for %s: %d\n",
				dai->name, ret);
		}

		if (!exynos7885_dai[nlink].platform_name) {
			exynos7885_dai[nlink].platform_of_node = exynos7885_dai[nlink].cpu_of_node;
		}

		if (!exynos7885_dai[nlink].codec_name) {
			ret = exynos7885_read_dai(dai, "codec",
						 &exynos7885_dai[nlink].codec_of_node,
						 &exynos7885_dai[nlink].codec_dai_name);
			if (ret) {
				dev_warn(card->dev,
					"Failed to parse codec DAI for %s: %d\n",
					dai->name, ret);
			}
		}

		if (++nlink == card->num_links)
			break;
	}

	if (!nlink) {
		dev_warn(card->dev, "No DAIs specified\n");
	}

	if (of_property_read_bool(np, "samsung,routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "samsung,routing");
		if (ret)
			return ret;
	}

	for (n = 0; n < ARRAY_SIZE(codec_conf); n++) {
		codec_conf[n].of_node = of_parse_phandle(np, "samsung,codec", n);

		if (!codec_conf[n].of_node) {
			dev_err(&pdev->dev,
				"Property 'samsung,codec' missing\n");
			return -EINVAL;
		}
	}
#ifdef CONFIG_SOC_EXYNOS8895
	for (n = 0; n < ARRAY_SIZE(aux_dev); n++) {
		aux_dev[n].codec_of_node = of_parse_phandle(np, "samsung,aux", n);

		if (!aux_dev[n].codec_of_node) {
			dev_err(&pdev->dev,
				"Property 'samsung,aux' missing\n");
			return -EINVAL;
		}
	}
#endif

#ifdef CONFIG_SOC_EXYNOS8895
	control_xclkout(true);
#endif
	ret = devm_snd_soc_register_card(card->dev, card);
	if (ret)
	{
		dev_err(card->dev, "snd_soc_register_card() failed:%d\n", ret);
	} else {
		universal7885_mic_bias_parse_dt(pdev);
		universal7885_init_soundcard(card);
	}

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id exynos7885_of_match[] = {
	{ .compatible = "samsung,exynos7885-COD3035", },
	{},
};
MODULE_DEVICE_TABLE(of, exynos7885_of_match);
#endif /* CONFIG_OF */

static struct platform_driver exynos7885_audio_driver = {
	.driver		= {
		.name	= "exynos7885-COD3035",
		.owner	= THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(exynos7885_of_match),
	},

	.probe		= exynos7885_audio_probe,
};

module_platform_driver(exynos7885_audio_driver);

MODULE_DESCRIPTION("ALSA SoC Exynos7885 COD3035 Driver");
MODULE_AUTHOR("Charles Keepax <ckeepax@opensource.wolfsonmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos7885-COD3035");

