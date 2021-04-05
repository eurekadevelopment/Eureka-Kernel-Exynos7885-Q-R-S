/*
 * Copyright (c) 2014 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author: Jason Seong <jason.seong@samsung.com>
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/completion.h>

#include <linux/mfd/samsung/s2mpu07-private.h>
#include <sound/cod3034x.h>
#include "cod3034x.h"

#define COD3034X_SAMPLE_RATE_48KHZ	48000
#define COD3034X_SAMPLE_RATE_192KHZ	192000

#define COD3034X_RESTORE_OTP_COUNT	5
#define COD3034X_RESTORE_REG_COUNT	16
#define COD3034X_OTP_R_OFFSET		0x0

#define COD3034X_MAX_IRQ_CHK_BITS	7
#define COD3034X_START_IRQ_CHK_BIT	2
#define COD3034X_MJ_DET_INVALID		(-1)

#ifdef CONFIG_SND_SOC_SAMSUNG_VERBOSE_DEBUG
#ifdef dev_dbg
#undef dev_dbg
#endif
#ifdef dev_info
#undef dev_info
#endif
#if 1 /* if: print option */
#define dev_dbg dev_err
#define dev_info dev_err
#else /* else: print option */
static void no_dev_dbg(void *v, char *s, ...)
{
}
#define dev_dbg no_dev_dbg
#define dev_info no_dev_dbg
#endif /* endif: print option */
#endif


/* Forward Declarations */
static void cod3034x_save_otp_registers(struct snd_soc_codec *codec);
static void cod3034x_restore_otp_registers(struct snd_soc_codec *codec);
static void cod3034x_reset_io_selector_bits(struct snd_soc_codec *codec);
static void cod3034x_configure_mic_bias(struct snd_soc_codec *codec);
static int cod3034x_disable(struct device *dev);
static int cod3034x_enable(struct device *dev);

static inline void cod3034x_usleep(unsigned int u_sec)
{
	usleep_range(u_sec, u_sec + 10);
}

/**
 * Helper functions to read ADC value for button detection
 */

#define COD3034X_ADC_SAMPLE_SIZE	5

static void cod3034x_adc_start(struct cod3034x_priv *cod3034x)
{
	cod3034x->jack_adc = iio_channel_get_all(cod3034x->dev);
}

static void cod3034x_adc_stop(struct cod3034x_priv *cod3034x)
{
	iio_channel_release(cod3034x->jack_adc);
}

static int cod3034x_adc_get_value(struct cod3034x_priv *cod3034x)
{
	int adc_data = -1;
	int adc_max = 0;
	int adc_min = 0xFFFF;
	int adc_total = 0;
	int adc_retry_cnt = 0;
	int i;
	struct iio_channel *jack_adc = cod3034x->jack_adc;

	for (i = 0; i < COD3034X_ADC_SAMPLE_SIZE; i++) {
		iio_read_channel_raw(&jack_adc[0], &adc_data);
		/* if adc_data is negative, ignore */
		while (adc_data < 0) {
			adc_retry_cnt++;
			if (adc_retry_cnt > 10)
				return adc_data;
			iio_read_channel_raw(&jack_adc[0], &adc_data);
		}

		/* Update min/max values */
		if (adc_data > adc_max)
			adc_max = adc_data;
		if (adc_data < adc_min)
			adc_min = adc_data;

		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (COD3034X_ADC_SAMPLE_SIZE - 2);
}

/**
 * Return value:
 * true: if the register value cannot be cached, hence we have to read from the
 * hardware directly
 * false: if the register value can be read from cache
 */
static bool cod3034x_volatile_register(struct device *dev, unsigned int reg)
{
	/**
	 * For all the registers for which we want to restore the value during
	 * regcache_sync operation, we need to return true here. For registers
	 * whose value need not be cached and restored should return false here.
	 *
	 * For the time being, let us cache the value of all registers other
	 * than the IRQ pending and IRQ status registers.
	 */
	switch (reg) {
	case COD3034X_01_IRQ1PEND ... COD3034X_04_IRQ4PEND:
	case COD3034X_09_STATUS1 ... COD3034X_0D_STATUS5:
	case COD3034X_60_IRQ_SENSOR ... COD3034X_63_OFFSET_DA:
	case COD3034X_80_PDB_ACC1 ... COD3034X_8B_CTR_BDNC3:
		return true;
	default:
		return false;
	}
}

/**
 * Return value:
 * true: if the register value can be read
 * flase: if the register cannot be read
 */
static bool cod3034x_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case 0x00 ... 0xff:
		return true;
	default:
		return false;
	}
}

static bool cod3034x_writeable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	/* Reg-0x09 to Reg-0x0B are read-only status registers */
	case COD3034X_01_IRQ1PEND ... COD3034X_08_IRQ4M:
	case COD3034X_10_PD_REF ... COD3034X_1D_SV_DA:
	case COD3034X_20_VOL_AD1 ... COD3034X_28_DSM_ADS:
	case COD3034X_30_VOL_HPL ... COD3034X_3C_KARAOKE2:
	case COD3034X_40_DIGITAL_POWER ... COD3034X_4F_DMIC4:
	case COD3034X_50_DAC1 ... COD3034X_5F_CRO3:
		/* Reg-0x61 is reserved, Reg-0x62 is read-only */
	case COD3034X_61_OFFSET_AD1 ... COD3034X_63_OFFSET_DA:
	case COD3034X_70_CLK1_COD ... COD3034X_7B_MAN_GN2:
	case COD3034X_80_PDB_ACC1 ... COD3034X_8B_CTR_BDNC3:
	case COD3034X_90_CTR_DLY5 ... COD3034X_9F_IMP_CNT8:
	case COD3034X_D0_CTRL_IREF1 ... COD3034X_DE_CTRL_SPKS2:
	case 0xF0 ... 0xFF:

		return true;
	default:
		return false;
	}
}

const struct regmap_config cod3034x_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	/* "speedy" string should be described in the name field
	 *  this will be used for the speedy inteface,
	 *  when read/write operations are used in the regmap driver.
	 * APM functions will be called instead of the I2C
	 * refer to the "drivers/base/regmap/regmap-i2c.c
	 */
	.name = "speedy, COD3034X",
	.max_register = COD3034X_MAX_REGISTER,
	.readable_reg = cod3034x_readable_register,
	.writeable_reg = cod3034x_writeable_register,
	.volatile_reg = cod3034x_volatile_register,

	.use_single_rw = true,
	.cache_type = REGCACHE_RBTREE,
};

/**
 * TLV_DB_SCALE_ITEM
 *
 * (TLV: Threshold Limit Value)
 *
 * For various properties, the dB values don't change linearly with respect to
 * the digital value of related bit-field. At most, they are quasi-linear,
 * that means they are linear for various ranges of digital values. Following
 * table define such ranges of various properties.
 *
 * TLV_DB_RANGE_HEAD(num)
 * num defines the number of linear ranges of dB values.
 *
 * s0, e0, TLV_DB_SCALE_ITEM(min, step, mute),
 * s0: digital start value of this range (inclusive)
 * e0: digital end valeu of this range (inclusive)
 * min: dB value corresponding to s0
 * step: the delta of dB value in this range
 * mute: ?
 *
 * Example:
 *	TLV_DB_RANGE_HEAD(3),
 *	0, 1, TLV_DB_SCALE_ITEM(-2000, 2000, 0),
 *	2, 4, TLV_DB_SCALE_ITEM(1000, 1000, 0),
 *	5, 6, TLV_DB_SCALE_ITEM(3800, 8000, 0),
 *
 * The above code has 3 linear ranges with following digital-dB mapping.
 * (0...6) -> (-2000dB, 0dB, 1000dB, 2000dB, 3000dB, 3800dB, 4600dB),
 *
 * DECLARE_TLV_DB_SCALE
 *
 * This macro is used in case where there is a linear mapping between
 * the digital value and dB value.
 *
 * DECLARE_TLV_DB_SCALE(name, min, step, mute)
 *
 * name: name of this dB scale
 * min: minimum dB value corresponding to digital 0
 * step: the delta of dB value
 * mute: ?
 *
 * NOTE: The information is mostly for user-space consumption, to be viewed
 * alongwith amixer.
 */

/**
 * cod3034x_ctvol_bst_tlv
 *
 * (0...2) -> (0dB, 1200dB, 2000dB)
 */
static const unsigned int cod3034x_ctvol_bst_tlv[] = {
	TLV_DB_RANGE_HEAD(2),
	0, 1, TLV_DB_SCALE_ITEM(0, 1200, 0),
	2, 2, TLV_DB_SCALE_ITEM(2000, 0, 0),
};

/**
 * cod3034x_ctvol_bst_pga_tlv
 *
 * Range: -16.5dB to +30dB, step 1.5dB
 *
 */
static const DECLARE_TLV_DB_SCALE(cod3034x_ctvol_bst_pga_tlv, -1650, 150, 0);

/**
 * cod3034x_ctvol_hp_tlv
 *
 * Range: -57dB to +6dB, step 1dB
 *
 * CTVOL_HPL, reg(0x30), shift(0), width(6), invert(1), max(63)
 * CTVOL_HPR, reg(0x31), shift(0), width(6), invert(1), max(63)
 */
static const DECLARE_TLV_DB_SCALE(cod3034x_ctvol_hp_tlv, -5700, 100, 0);

/**
 * cod3019_ctvol_ep_tlv
 *
 * Range: -6dB to +12dB, step 1dB
 *
 * CTVOL_EP, reg(0x32), shift(0), width(5), invert(0), max(18)
 */
static const DECLARE_TLV_DB_SCALE(cod3034x_ctvol_ep_tlv, -600, 100, 0);

/**
 * cod3034x_ctvol_spk_pga_tlv
 *
 * Range: -18dB to +6dB, step 1dB
 *
 * CTVOL_SPK_PGA, reg(0x39), shift(0), width(5), invert(1), max(24)
 */
static const DECLARE_TLV_DB_SCALE(cod3034x_ctvol_spk_pga_tlv, -1800, 100, 0);

/**
 * cod3034x_dvol_adc_tlv
 *
 * Map as per data-sheet:
 * (0x00 to 0x86) -> (+12dB to -55dB, step 0.5dB)
 * (0x87 to 0x91) -> (-56dB to -66dB, step 1dB)
 * (0x92 to 0x94) -> (-68dB to -72dB, step 2dB)
 * (0x95 to 0x96) -> (-78dB to -84dB, step 6dB)
 *
 * When the map is in descending order, we need to set the invert bit
 * and arrange the map in ascending order. The offsets are calculated as
 * (max - offset).
 *
 * offset_in_table = max - offset_actual;
 *
 * DVOL_ADL, reg(0x43), shift(0), width(8), invert(1), max(0x96)
 * DVOL_ADR, reg(0x44), shift(0), width(8), invert(1), max(0x96)
 * DVOL_DAL, reg(0x51), shift(0), width(8), invert(1), max(0x96)
 * DVOL_DAR, reg(0x52), shift(0), width(8), invert(1), max(0x96)
 */
static const unsigned int cod3034x_dvol_tlv[] = {
	TLV_DB_RANGE_HEAD(4),
	0x00, 0x01, TLV_DB_SCALE_ITEM(-8400, 600, 0),
	0x02, 0x04, TLV_DB_SCALE_ITEM(-7200, 200, 0),
	0x05, 0x09, TLV_DB_SCALE_ITEM(-6600, 100, 0),
	0x10, 0x96, TLV_DB_SCALE_ITEM(-5500, 50, 0),
};

/**
 * cod3034x_dnc_min_gain_tlv
 *
 * Range: -6dB to 0dB, step 1dB
 *
 * DNC_MINGAIN , reg(0x55), shift(5), width(3)
 */
static const unsigned int cod3034x_dnc_min_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0x00, 0x06, TLV_DB_SCALE_ITEM(-600, 0, 0),
};

/**
 * cod3034x_dnc_max_gain_tlv
 *
 * Range: 0dB to 24dB, step 1dB
 *
 * DNC_MAXGAIN , reg(0x55), shift(0), width(5)
 */
static const unsigned int cod3034x_dnc_max_gain_tlv[] = {
	TLV_DB_RANGE_HEAD(1),
	0x06, 0x1e, TLV_DB_SCALE_ITEM(0, 2400, 0),
};

/**
 * cod3034x_dnc_lvl_tlv
 *
 * Range: -10.5dB to 0dB, step 1.5dB
 *
 * DNCLVL_R/L, reg(0x55), shift(0/4), width(3), invert(0), max(7)
 */
static const DECLARE_TLV_DB_SCALE(cod3034x_dnc_lvl_tlv, -1050, 0, 0);

/**
 * mono_mix_mode
 *
 * Selecting the Mode of Mono Mixer (inside DAC block)
 */
static const char *cod3034x_mono_mix_mode_text[] = {
	"Disable", "R", "L", "LR-Invert",
	"(L+R)/2", "L+R"
};

static const struct soc_enum cod3034x_mono_mix_mode_enum =
SOC_ENUM_SINGLE(COD3034X_50_DAC1, DAC1_MONOMIX_SHIFT,
		ARRAY_SIZE(cod3034x_mono_mix_mode_text),
		cod3034x_mono_mix_mode_text);


static int dac_soft_mute_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	       return 0;
}


static int dac_soft_mute_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	int value = ucontrol->value.integer.value[0];

	if (!value)
		/* enable soft mute */
		snd_soc_update_bits(codec, COD3034X_50_DAC1,
				DAC1_SOFT_MUTE_MASK, DAC1_SOFT_MUTE_MASK);
	else
		/* diable soft mute */
		snd_soc_update_bits(codec, COD3034X_50_DAC1,
				DAC1_SOFT_MUTE_MASK, 0x0);

	dev_info(codec->dev, "%s: soft mute : %s\n", __func__,
			(!value) ? "on":"off");
	return 0;
}

/**
 * struct snd_kcontrol_new cod3034x_snd_control
 *
 * Every distinct bit-fields within the CODEC SFR range may be considered
 * as a control elements. Such control elements are defined here.
 *
 * Depending on the access mode of these registers, different macros are
 * used to define these control elements.
 *
 * SOC_ENUM: 1-to-1 mapping between bit-field value and provided text
 * SOC_SINGLE: Single register, value is a number
 * SOC_SINGLE_TLV: Single register, value corresponds to a TLV scale
 * SOC_SINGLE_TLV_EXT: Above + custom get/set operation for this value
 * SOC_SINGLE_RANGE_TLV: Register value is an offset from minimum value
 * SOC_DOUBLE: Two bit-fields are updated in a single register
 * SOC_DOUBLE_R: Two bit-fields in 2 different registers are updated
 */

/**
 * All the data goes into cod3034x_snd_controls.
 * All path inter-connections goes into cod3034x_dapm_routes
 */
static const struct snd_kcontrol_new cod3034x_snd_controls[] = {
	SOC_SINGLE_TLV("MIC1 Boost Volume", COD3034X_20_VOL_AD1,
			VOLAD1_CTVOL_BST1_SHIFT,
			(BIT(VOLAD1_CTVOL_BST1_WIDTH) - 1), 0,
			cod3034x_ctvol_bst_tlv),

	SOC_SINGLE_TLV("MIC1 Volume", COD3034X_20_VOL_AD1,
			VOLAD1_CTVOL_BST_PGA1_SHIFT,
			(BIT(VOLAD1_CTVOL_BST_PGA1_WIDTH) - 1), 0,
			cod3034x_ctvol_bst_pga_tlv),

	SOC_SINGLE_TLV("MIC2 Boost Volume", COD3034X_21_VOL_AD2,
			VOLAD2_CTVOL_BST2_SHIFT,
			(BIT(VOLAD2_CTVOL_BST2_WIDTH) - 1), 0,
			cod3034x_ctvol_bst_tlv),

	SOC_SINGLE_TLV("MIC2 Volume", COD3034X_21_VOL_AD2,
			VOLAD2_CTVOL_BST_PGA2_SHIFT,
			(BIT(VOLAD2_CTVOL_BST_PGA2_WIDTH) - 1), 0,
			cod3034x_ctvol_bst_pga_tlv),

	SOC_SINGLE_TLV("MIC3 Boost Volume", COD3034X_22_VOL_AD3,
			VOLAD3_CTVOL_BST3_SHIFT,
			(BIT(VOLAD3_CTVOL_BST3_WIDTH) - 1), 0,
			cod3034x_ctvol_bst_tlv),

	SOC_SINGLE_TLV("MIC3 Volume", COD3034X_22_VOL_AD3,
			VOLAD3_CTVOL_BST_PGA3_SHIFT,
			(BIT(VOLAD3_CTVOL_BST_PGA3_WIDTH) - 1), 0,
			cod3034x_ctvol_bst_pga_tlv),

	SOC_DOUBLE_R_TLV("Headphone Volume", COD3034X_30_VOL_HPL,
			COD3034X_31_VOL_HPR, VOLHP_CTVOL_HP_SHIFT,
			(BIT(VOLHP_CTVOL_HP_WIDTH) - 1), 1,
			cod3034x_ctvol_hp_tlv),

	SOC_SINGLE_TLV("Earphone Volume", COD3034X_32_VOL_EP_SPK,
			CTVOL_EP_SHIFT,
			(BIT(CTVOL_EP_WIDTH) - 1), 0,
			cod3034x_ctvol_ep_tlv),

	SOC_SINGLE_TLV("Speaker Volume", COD3034X_32_VOL_EP_SPK,
			CTVOL_SPK_PGA_SHIFT,
			(BIT(CTVOL_SPK_PGA_WIDTH) - 8), 1,
			cod3034x_ctvol_spk_pga_tlv),

	SOC_SINGLE_TLV("ADC Left Gain", COD3034X_47_AVOLL1,
			AD_DA_DVOL_SHIFT,
			AD_DA_DVOL_MAXNUM, 1, cod3034x_dvol_tlv),

	SOC_SINGLE_TLV("ADC Right Gain", COD3034X_48_AVOLR1,
			AD_DA_DVOL_SHIFT,
			AD_DA_DVOL_MAXNUM, 1, cod3034x_dvol_tlv),

	SOC_DOUBLE_R_TLV("DAC Gain", COD3034X_51_DVOLL,
			COD3034X_52_DVOLR, AD_DA_DVOL_SHIFT,
			AD_DA_DVOL_MAXNUM, 1, cod3034x_dvol_tlv),

	SOC_ENUM("MonoMix Mode", cod3034x_mono_mix_mode_enum),

	SOC_SINGLE_EXT("DAC Soft Mute", SND_SOC_NOPM, 0, 100, 0,
			dac_soft_mute_get, dac_soft_mute_put),
};

static int dac_ev(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol,
		int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* DAC digital power On */
		snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER,
				PDB_DACDIG_MASK, PDB_DACDIG_MASK);

		/* DAC digital Reset On/Off */
		snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER,
				RSTB_DAT_DA_MASK, RSTB_DAT_DA_MASK);
		snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER,
				RSTB_DAT_AD_MASK, RSTB_DAT_AD_MASK);

		break;

	case SND_SOC_DAPM_PRE_PMD:
		/* DAC digital Reset Off */
		snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER,
				RSTB_DAT_DA_MASK, 0x0);

		/* DAC digital power Off */
		snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER,
				PDB_DACDIG_MASK, 0x0);
		break;

	default:
		break;
	}

	return 0;
}

static void cod3034x_adc_digital_mute(struct snd_soc_codec *codec, bool on)
{
	if (on)
		snd_soc_update_bits(codec, COD3034X_46_ADC1,
				ADC1_MUTE_AD_EN_MASK, ADC1_MUTE_AD_EN_MASK);
	else
		snd_soc_update_bits(codec, COD3034X_46_ADC1,
				ADC1_MUTE_AD_EN_MASK, 0);
}

static int cod3034x_reference_power_on(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s called\n", __func__);


	/* addr 0x10 <- 0x20 VMID ON */
	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			PDB_VMID_MASK, PDB_VMID_MASK);

	/* addr 0x19 <- 0x30 VMID Fast Charging ON */
	snd_soc_update_bits(codec, COD3034X_19_CTRL_REF,
			CTMF_VMID_MASK, CTMF_VMID_5K_OM);

	/* addr 0x11 <- 0x30 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			EN_DSMR_PREQ_MASK|EN_DSML_PREQ_MASK,
			EN_DSMR_PREQ_MASK|EN_DSML_PREQ_MASK);

	/* old : addr 0x13 <- 0x07 --> modified 0xE7 */
	/* addr 0x13 <- 0xA7 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			0xA7, 0xA7);
	/* snd_soc_write(codec, COD3034X_13_PD_AD3, 0x07); */

	/* addr 0x10 <- 0x30 IGEN ON */
	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			PDB_IGEN_MASK|PDB_VMID_MASK,
			PDB_IGEN_MASK|PDB_VMID_MASK);


	return 0;
}

static int cod3034x_capture_init(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s called\n", __func__);

	/* addr 0x40 : |0x80, Recording Digital  Power on */
	snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER,
			PDB_ADCDIG_MASK, PDB_ADCDIG_MASK);

	/* enable ADC digital mute before configuring ADC */
	cod3034x_adc_digital_mute(codec, true);

	cod3034x_reference_power_on(codec);

	/* cod3034x_adc_digital_mute(codec, false); */

	/* for print complete */
	cod3034x_usleep(10);

	return 0;
}

static void cod3034x_capture_deinit_manual_mode(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			PDB_IGEN_MASK, 0);

	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			PDB_VMID_MASK, 0);
}

static int cod3034x_capture_deinit(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s called\n", __func__);

	cod3034x_capture_deinit_manual_mode(codec);

	/* disable ADC digital mute after configuring ADC */
	cod3034x_adc_digital_mute(codec, false);
	return 0;
}

static int adc_ev(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol,
		int event)
{
	int dac_on;
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	dev_dbg(codec->dev, "%s called, event = %d\n", __func__, event);
	dac_on = snd_soc_read(codec, COD3034X_40_DIGITAL_POWER);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		break;

	case SND_SOC_DAPM_POST_PMU:
		/* disable ADC digital mute after configuring ADC */
		cod3034x_adc_digital_mute(codec, false);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		/* disable ADC digital mute before configuring ADC */
		cod3034x_adc_digital_mute(codec, true);
		break;

	default:
		break;
	}

	return 0;
}

int cod3034x_mic_bias_ev(struct snd_soc_codec *codec, int mic_bias, int event)
{
	int is_other_mic_on, mask;

	dev_dbg(codec->dev, "%s Called, Mic bias = %d, Event = %d\n",
				__func__, mic_bias, event);

	is_other_mic_on = snd_soc_read(codec, COD3034X_10_PD_REF);

	if (mic_bias == COD3034X_MICBIAS1) {
		is_other_mic_on &= PDB_MCB2_MASK;
		mask = is_other_mic_on ? PDB_MCB1_MASK :
			PDB_MCB1_MASK | PDB_MCB_LDO_CODEC_MASK;
	} else if (mic_bias == COD3034X_MICBIAS2) {
		is_other_mic_on &= PDB_MCB1_MASK;
		mask = is_other_mic_on ? PDB_MCB2_MASK :
			PDB_MCB2_MASK | PDB_MCB_LDO_CODEC_MASK;
	} else {
		dev_err(codec->dev, "%s Called , Invalid MIC ID\n",
							__func__);
		return -1;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, COD3034X_10_PD_REF, mask, mask);
		if (mic_bias == COD3034X_MICBIAS2)
			snd_soc_update_bits(codec, COD3034X_19_CTRL_REF,
					CTRM_MCB2_MASK, CTRM_MCB2_MASK);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, COD3034X_10_PD_REF, mask, 0x00);

		if (mic_bias == COD3034X_MICBIAS2)
			snd_soc_update_bits(codec, COD3034X_19_CTRL_REF,
							CTRM_MCB2_MASK, 0);
		break;
	}

	return 0;
}

/**
  * Mute mic if it is active
  *
  * Returns -1 if error, else 0
  */
static int cod3034x_mute_mic(struct snd_soc_codec *codec, bool on)
{

	dev_dbg(codec->dev, "%s called, %s\n", __func__,
			on ? "Mute" : "Unmute");

	if (on)
		cod3034x_adc_digital_mute(codec, true);
	else
		cod3034x_adc_digital_mute(codec, false);

	return 0;
}

/* process the button events based on the need */
void cod3034x_process_button_ev(struct snd_soc_codec *codec, int code, int on)
{
	bool key_press = on ? true : false;

	cod3034x_mute_mic(codec, key_press);
}

static int cod3034_power_on_mic1(struct snd_soc_codec *codec)
{
	int ret_value;

	dev_dbg(codec->dev, "%s called\n", __func__);

	/* addr 0x10 <- 0x3C from 0x38 since mic2 uses mic bias1 */
	/* refer to the katmai schematic */
	snd_soc_write(codec, COD3034X_10_PD_REF, 0x3D);
#if 0
	/* in case it needs to be replaced with the update function. */
	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			PDB_IGEN_AD_MASK|PDB_MCB_LDO_CODEC_MASK|PDB_MCB1_MASK,
			PDB_IGEN_AD_MASK|PDB_MCB_LDO_CODEC_MASK|PDB_MCB1_MASK);
#endif

	/* addr 0x12 <- 0x24 */
	snd_soc_update_bits(codec, COD3034X_12_PD_AD2,
			PDB_MIC_BST1_MASK|PDB_MIC_PGA1_MASK,
			PDB_MIC_BST1_MASK|PDB_MIC_PGA1_MASK);

	/* Mixer On 0x11 & 0x13 */
	/* addr 0x11 <- |0xC0 // 0x30 ->0xF0 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_MIXR_MASK|PDB_MIXL_MASK,
			PDB_MIXR_MASK|PDB_MIXL_MASK);

	/* addr 0x13 <- |0x40 // 0xA7 -> 0xE7 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_MIXC_MASK, PDB_MIXC_MASK);

	/* DSM On 0x11 & 0x13 */
	/* addr 0x11 <- |0x0C // 0xF0 ->0xFC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_DSML_MASK|PDB_DSMR_MASK,
			PDB_DSML_MASK|PDB_DSMR_MASK);

	/* addr 0x13 <- |0x10 // 0xE7 -> 0xF7 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_DSMC_MASK, PDB_DSMC_MASK);

	ret_value = snd_soc_read(codec, COD3034X_13_PD_AD3);
	if (ret_value == 0x87)
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x83);
	else /* ret_value == 0xF7 or else */
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0xF3);

	/* Mixer path selection */
	/* addr 0x25 <- 0x0C */
	snd_soc_update_bits(codec, COD3034X_25_MIX_AD1, 0x30, 0x30);
	snd_soc_write(codec, COD3034X_25_MIX_AD1, 0x30);
	snd_soc_update_bits(codec, COD3034X_26_MIX_ADC, 0x80, 0x80);
	msleep(140);

	ret_value = snd_soc_read(codec, COD3034X_13_PD_AD3);
	if (ret_value == 0x83)
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x07);
	else /* ret_value == 0xF3 or else */
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x77);

	/* VREFP_AD Pre Charging OFF, 0x11 & 0x13 */
	/* addr 0x11 <- ~0x30 // 0xFC ->0xCC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			EN_DSML_PREQ_MASK|EN_DSMR_PREQ_MASK, 0x0);

	/* addr 0x13 <- ~0x20 // 0x77 -> 0x57 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			EN_DSMC_PREQ_MASK, 0x0);

	/* addr 0x19 <- |0x40 // 0x30 -> 0x70, VMID Fast Charging OFF */
	snd_soc_update_bits(codec, COD3034X_19_CTRL_REF,
			CTMF_VMID_MASK, CTMF_VMID_50K_OM);

	/* DSM RESET OFF, 0x11 & 0x13 */
	/* addr 0x11 <- |0x30 // 0xCC -> 0xCF */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			RESETB_DSMR_MASK|RESETB_DSML_MASK,
			RESETB_DSMR_MASK|RESETB_DSML_MASK);

	/* addr 0x13 <- |0x08 // 0x57 -> 0x5F */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			RESETB_DSMC_MASK, RESETB_DSMC_MASK);

	/* addr 0x80 <- 0x10 */
	snd_soc_update_bits(codec, COD3034X_80_PDB_ACC1,
			DET_PDB_MCB_LDO_MASK, DET_PDB_MCB_LDO_MASK);

#if 0 /* For Katmai SMDK */
	/* addr 0x12 <- 0x24 */
	snd_soc_update_bits(codec, COD3034X_12_PD_AD2,
			PDB_MIC_BST1_MASK|PDB_MIC_PGA1_MASK,
			PDB_MIC_BST1_MASK|PDB_MIC_PGA1_MASK);

	/* addr 0x25 <- 0x30 */
	snd_soc_update_bits(codec, COD3034X_25_MIX_AD1, 0x30, 0x30);

	snd_soc_write(codec, COD3034X_4A_DMIC2, 0x74);
	snd_soc_write(codec, COD3034X_49_DMIC1, 0x45);

	msleep(140);
#endif
	/* dump_register(codec); */

	return 0;
}

static int cod3034_power_off_mic1(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s called\n", __func__);

	/* ADC Mute OFF, addr: 0x46(0x42 in SFR), value: 0x0C -> 0x0D */
	snd_soc_update_bits(codec, COD3034X_46_ADC1,
			ADC1_MUTE_AD_EN_MASK, ADC1_MUTE_AD_EN_MASK);

	/* DSM RESET On */
	/* addr: 0x11, value: 0xCF -> 0xCC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			RESETB_DSMR_MASK|RESETB_DSML_MASK, 0);

	/* addr: 0x13, value: 0x5F -> 0x57 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			RESETB_DSMC_MASK, 0);

	/* MIXER Path Off */
	/* addr: 0x25, value: 0x30 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_25_MIX_AD1,
			EN_MIX_MIC1L_MASK | EN_MIX_MIC1R_MASK, 0x0);

	/* addr: 0x26, value: 0x80 -> 0x00 */
	snd_soc_write(codec, COD3034X_26_MIX_ADC, 0x0);

	/* DSM OFF */
	/* addr: 0x11, value: 0xCC -> 0xC0 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_DSMR_MASK | PDB_DSML_MASK, 0);

	/* addr: 0x13, value: 0x57 -> 0x47 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_DSMC_MASK, 0);

	/* MIXER OFF */
	/* addr: 0x11, value: 0xC0 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_MIXR_MASK | PDB_MIXL_MASK, 0);

	/* addr: 0x13, value: 0x47 -> 0x07 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_MIXC_MASK, 0);

	/* MIC1 OFF, addr: 0x12, value: 0x24 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_12_PD_AD2,
			PDB_MIC_PGA1_MASK | PDB_MIC_BST1_MASK, 0);

	/* IGEN_AD OFF, addr: 0x10, value: 0x38 -> 0x30 */
	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			0x0D, 0);

	/* In case if mic is a digital mic */
	/* snd_soc_write(codec, COD3034X_49_DMIC1, 0x00); */

	/* dump_register(codec); */

	return 0;
}
static int cod3034_power_on_mic2(struct snd_soc_codec *codec)
{
	int ret_value;

	dev_dbg(codec->dev, "%s called\n", __func__);

	/* addr 0x10 <- 0x3C from 0x38 since mic2 uses mic bias1 */
	/* refer to the katmai schematic */
	snd_soc_write(codec, COD3034X_10_PD_REF, 0x3D);

	/* addr 0x12 <- 0x24 */
	snd_soc_update_bits(codec, COD3034X_12_PD_AD2,
			PDB_MIC_BST2_MASK|PDB_MIC_PGA2_MASK,
			PDB_MIC_BST2_MASK|PDB_MIC_PGA2_MASK);

	/* Mixer On 0x11 & 0x13 */
	/* addr 0x11 <- |0xC0 // 0x30 ->0xF0 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_MIXR_MASK|PDB_MIXL_MASK,
			PDB_MIXR_MASK|PDB_MIXL_MASK);

	/* addr 0x13 <- |0x40 // 0xA7 -> 0xE7 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_MIXC_MASK, PDB_MIXC_MASK);

	/* DSM On 0x11 & 0x13 */
	/* addr 0x11 <- |0x0C // 0xF0 ->0xFC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_DSML_MASK|PDB_DSMR_MASK,
			PDB_DSML_MASK|PDB_DSMR_MASK);

	/* addr 0x13 <- |0x10 // 0xE7 -> 0xF7 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_DSMC_MASK, PDB_DSMC_MASK);

	ret_value = snd_soc_read(codec, COD3034X_13_PD_AD3);
	if (ret_value == 0x87)
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x85);
	else /* ret_value == 0xF7 or else */
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0xF5);

	/* Mixer path selection */
	/* addr 0x25 <- 0x0C */
	snd_soc_update_bits(codec, COD3034X_25_MIX_AD1, 0xC0, 0xC0);
	snd_soc_write(codec, COD3034X_25_MIX_AD1, 0xC0);
	snd_soc_update_bits(codec, COD3034X_26_MIX_ADC, 0x40, 0x40);
	msleep(140);

	ret_value = snd_soc_read(codec, COD3034X_13_PD_AD3);
	if (ret_value == 0x83)
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x07);
	else /* ret_value == 0xF3 or else */
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x77);

	/* VREFP_AD Pre Charging OFF, 0x11 & 0x13 */
	/* addr 0x11 <- ~0x30 // 0xFC ->0xCC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			EN_DSML_PREQ_MASK|EN_DSMR_PREQ_MASK, 0x0);

	/* addr 0x13 <- ~0x20 // 0x77 -> 0x57 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			EN_DSMC_PREQ_MASK, 0x0);

	/* addr 0x19 <- |0x40 // 0x30 -> 0x70, VMID Fast Charging OFF */
	snd_soc_update_bits(codec, COD3034X_19_CTRL_REF,
			CTMF_VMID_MASK, CTMF_VMID_50K_OM);

	/* DSM RESET OFF, 0x11 & 0x13 */
	/* addr 0x11 <- |0x30 // 0xCC -> 0xCF */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			RESETB_DSMR_MASK|RESETB_DSML_MASK,
			RESETB_DSMR_MASK|RESETB_DSML_MASK);

	/* addr 0x13 <- |0x08 // 0x57 -> 0x5F */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			RESETB_DSMC_MASK, RESETB_DSMC_MASK);

	/* addr 0x80 <- 0x10 */
	snd_soc_update_bits(codec, COD3034X_80_PDB_ACC1,
			DET_PDB_MCB_LDO_MASK, DET_PDB_MCB_LDO_MASK);


#if 0
	/* addr 0x10 <- 0x3C from 0x38 since mic2 uses mic bias1 */
	/* refer to the katmai schematic */
	/*
	* snd_soc_update_bits(codec, COD3034X_10_PD_REF,
	*		PDB_MCB1_MASK, PDB_MCB1_MASK);
	*/
	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			PDB_IGEN_AD_MASK|PDB_MCB_LDO_CODEC_MASK|PDB_MCB1_MASK,
			PDB_IGEN_AD_MASK|PDB_MCB_LDO_CODEC_MASK|PDB_MCB1_MASK);

	/* addr 0x12 <- 0x12 */
	snd_soc_update_bits(codec, COD3034X_12_PD_AD2,
			PDB_MIC_BST2_MASK|PDB_MIC_PGA2_MASK,
			PDB_MIC_BST2_MASK|PDB_MIC_PGA2_MASK);

	/* addr 0x25 <- 0x0C */
	snd_soc_update_bits(codec, COD3034X_25_MIX_AD1, 0x0C, 0x0C);

	msleep(140);
#endif

	/* dump_register(codec); */

	return 0;
}



static int cod3034_power_off_mic2(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s called\n", __func__);

	/* ADC Mute OFF, addr: 0x46(0x42 in SFR), value: 0x0C -> 0x0D */
	snd_soc_update_bits(codec, COD3034X_46_ADC1,
			ADC1_MUTE_AD_EN_MASK, ADC1_MUTE_AD_EN_MASK);

	/* DSM RESET On */
	/* addr: 0x11, value: 0xCF -> 0xCC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			RESETB_DSML_MASK|RESETB_DSMR_MASK, 0);

	/* addr: 0x13, value: 0x5F -> 0x57 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			RESETB_DSMC_MASK, 0);

	/* MIXER Path Off */
	/* addr: 0x25, value: 0xC0 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_25_MIX_AD1,
			EN_MIX_MIC2L_MASK | EN_MIX_MIC2R_MASK, 0x0);

	/* addr: 0x26, value: 0x40 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_26_MIX_ADC, 0x40, 0x0);

	/* DSM OFF */
	/* addr: 0x11, value: 0xCC -> 0xC0 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_DSMR_MASK | PDB_DSML_MASK, 0);

	/* addr: 0x13, value: 0x57 -> 0x47 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_DSMC_MASK, 0);

	/* MIXER OFF */
	/* addr: 0x11, value: 0xC0 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_MIXR_MASK | PDB_MIXL_MASK, 0);

	/* addr: 0x13, value: 0x47 -> 0x07 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_MIXC_MASK, 0);

	/* MIC1 OFF, addr: 0x12, value: 0x12 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_12_PD_AD2,
			PDB_MIC_PGA2_MASK | PDB_MIC_BST2_MASK, 0);

	/* IGEN_AD OFF, addr: 0x10, value: 0x3D -> 0x30 */
	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			0x0D, 0);

	return 0;
}

static int cod3034_power_on_mic3(struct snd_soc_codec *codec)
{
	int ret_value;

	dev_dbg(codec->dev, "%s called\n", __func__);

	/* addr 0x10 <- 0x3C from 0x38 since mic2 uses mic bias1 */
	/* refer to the katmai schematic */
	snd_soc_write(codec, COD3034X_10_PD_REF, 0x3D);

	/* addr 0x12 <- 0x24 */
	snd_soc_update_bits(codec, COD3034X_12_PD_AD2,
			PDB_MIC_BST3_MASK|PDB_MIC_PGA3_MASK,
			PDB_MIC_BST3_MASK|PDB_MIC_PGA3_MASK);

	/* Mixer On 0x11 & 0x13 */
	/* addr 0x11 <- |0xC0 // 0x30 ->0xF0 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_MIXR_MASK|PDB_MIXL_MASK,
			PDB_MIXR_MASK|PDB_MIXL_MASK);

	/* addr 0x13 <- |0x40 // 0xA7 -> 0xE7 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_MIXC_MASK, PDB_MIXC_MASK);

	/* DSM On 0x11 & 0x13 */
	/* addr 0x11 <- |0x0C // 0xF0 ->0xFC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_DSML_MASK|PDB_DSMR_MASK,
			PDB_DSML_MASK|PDB_DSMR_MASK);

	/* addr 0x13 <- |0x10 // 0xE7 -> 0xF7 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_DSMC_MASK, PDB_DSMC_MASK);

	ret_value = snd_soc_read(codec, COD3034X_13_PD_AD3);
	if (ret_value == 0x87)
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x86);
	else /* ret_value == 0xF7 or else */
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0xF6);

	/* Mixer path selection */
	/* addr 0x25 <- 0x0C */
	snd_soc_update_bits(codec, COD3034X_25_MIX_AD1, 0x03, 0x03);
	snd_soc_write(codec, COD3034X_25_MIX_AD1, 0x03);
	snd_soc_update_bits(codec, COD3034X_26_MIX_ADC, 0x20, 0x20);
	msleep(140);

	ret_value = snd_soc_read(codec, COD3034X_13_PD_AD3);
	if (ret_value == 0x83)
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x07);
	else /* ret_value == 0xF3 or else */
		snd_soc_write(codec, COD3034X_13_PD_AD3, 0x77);

	/* VREFP_AD Pre Charging OFF, 0x11 & 0x13 */
	/* addr 0x11 <- ~0x30 // 0xFC ->0xCC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			EN_DSML_PREQ_MASK|EN_DSMR_PREQ_MASK, 0x0);

	/* addr 0x13 <- ~0x20 // 0x77 -> 0x57 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			EN_DSMC_PREQ_MASK, 0x0);

	/* addr 0x19 <- |0x40 // 0x30 -> 0x70, VMID Fast Charging OFF */
	snd_soc_update_bits(codec, COD3034X_19_CTRL_REF,
			CTMF_VMID_MASK, CTMF_VMID_50K_OM);

	/* DSM RESET OFF, 0x11 & 0x13 */
	/* addr 0x11 <- |0x30 // 0xCC -> 0xCF */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			RESETB_DSMR_MASK|RESETB_DSML_MASK,
			RESETB_DSMR_MASK|RESETB_DSML_MASK);

	/* addr 0x13 <- |0x08 // 0x57 -> 0x5F */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			RESETB_DSMC_MASK, RESETB_DSMC_MASK);

	/* addr 0x80 <- 0x10 */
	snd_soc_update_bits(codec, COD3034X_80_PDB_ACC1,
			DET_PDB_MCB_LDO_MASK, DET_PDB_MCB_LDO_MASK);


	return 0;
}

static int cod3034_power_off_mic3(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s called\n", __func__);

	/* ADC Mute OFF, addr: 0x46(0x42 in SFR), value: 0x0C -> 0x0D */
	snd_soc_update_bits(codec, COD3034X_46_ADC1,
			ADC1_MUTE_AD_EN_MASK, ADC1_MUTE_AD_EN_MASK);

	/* DSM RESET On */
	/* addr: 0x11, value: 0xCF -> 0xCC */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			RESETB_DSML_MASK|RESETB_DSMR_MASK, 0);

	/* addr: 0x13, value: 0x5F -> 0x57 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			RESETB_DSMC_MASK, 0);

	/* MIXER Path Off */
	/* addr: 0x25, value: 0x03 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_25_MIX_AD1,
			EN_MIX_MIC3L_MASK | EN_MIX_MIC3R_MASK, 0x0);

	/* addr: 0x26, value: 0x20 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_26_MIX_ADC, 0x20, 0x0);

	/* DSM OFF */
	/* addr: 0x11, value: 0xCC -> 0xC0 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_DSMR_MASK | PDB_DSML_MASK, 0);

	/* addr: 0x13, value: 0x57 -> 0x47 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_DSMC_MASK, 0);

	/* MIXER OFF */
	/* addr: 0x11, value: 0xC0 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_11_PD_AD1,
			PDB_MIXR_MASK | PDB_MIXL_MASK, 0);

	/* addr: 0x13, value: 0x47 -> 0x07 */
	snd_soc_update_bits(codec, COD3034X_13_PD_AD3,
			PDB_MIXC_MASK, 0);

	/* MIC1 OFF, addr: 0x12, value: 0x09 -> 0x00 */
	snd_soc_update_bits(codec, COD3034X_12_PD_AD2,
			PDB_MIC_PGA3_MASK | PDB_MIC_BST3_MASK, 0);

	/* IGEN_AD OFF, addr: 0x10, value: 0x3D -> 0x30 */
	snd_soc_update_bits(codec, COD3034X_10_PD_REF,
			0x0D, 0);

	return 0;
}

static int vmid_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	dev_dbg(codec->dev, "%s called\n", __func__);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod3034x_capture_init(codec);
		break;

	case SND_SOC_DAPM_POST_PMU:
		break;

	case SND_SOC_DAPM_PRE_PMD:
		cod3034x_capture_deinit(codec);
		break;

	default:
		break;
	}

	return 0;
}

/**
 * SFR revision 1.14 recommends following settings.
 * If playback is only HP mode, write f/w OTP value at 0xD4 and 0xD5 and enable
 * DNC.
 * If playback is not only HP mode, write all zero data value at 0xD4 and 0xD5
 * with DNC disabled.
 */
static void cod3034x_update_playback_otp(struct snd_soc_codec *codec)
{
	int hp_on, spk_on, ep_on;
	int chop_val;
	int offset;
	int offset_ep;
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	chop_val = snd_soc_read(codec, COD3034X_77_CHOP_DA);
	hp_on = chop_val & EN_HP_CHOP_MASK;
	spk_on = chop_val & EN_SPK_PGA_CHOP_MASK;
	ep_on = chop_val & EN_EP_CHOP_MASK;

	if (!hp_on && !spk_on && !ep_on) {
		dev_warn(codec->dev, "None of the output paths selected.\n");
		return;
	}

	if (hp_on && !spk_on && !ep_on) {
		/* We are in HP only mode */
		/* Updating OTP register 0xD4 */
		offset = COD3034X_D4_OFFSET_DAL - COD3034X_OTP_REG_WRITE_START;
		snd_soc_write(codec, COD3034X_D4_OFFSET_DAL,
				cod3034x->otp_reg[offset]);

		/* Updating OTP register 0xD5 */
		offset = COD3034X_D5_OFFSET_DAR - COD3034X_OTP_REG_WRITE_START;
		snd_soc_write(codec, COD3034X_D5_OFFSET_DAR,
				cod3034x->otp_reg[offset]);

	} else if (!hp_on && !spk_on && ep_on) {
		/* We are in EP only mode */
		offset_ep = snd_soc_read(codec, COD3034X_D7_CTRL_EP);
		snd_soc_write(codec, COD3034X_D4_OFFSET_DAL,
				offset_ep);

	} else if (hp_on && spk_on && !ep_on) {
		/* We are in HP & SPK  mode */
		/* Updating OTP register 0xD4 */
		offset = COD3034X_D4_OFFSET_DAL - COD3034X_OTP_REG_WRITE_START;
		snd_soc_write(codec, COD3034X_D4_OFFSET_DAL,
				cod3034x->otp_reg[offset]);

		/* Updating OTP register 0xD5 */
		offset = COD3034X_D5_OFFSET_DAR - COD3034X_OTP_REG_WRITE_START;
		snd_soc_write(codec, COD3034X_D5_OFFSET_DAR,
				cod3034x->otp_reg[offset]);
	} else {
		/* This is not-only HP mode */
		snd_soc_write(codec, COD3034X_D4_OFFSET_DAL, 0x0);
		snd_soc_write(codec, COD3034X_D5_OFFSET_DAR, 0x0);
	}

	if (hp_on && !spk_on && !ep_on) {
		/* ToDo : insert hp specific only register settting */

	} else {
		/* This is not-only HP mode */
		/* Disable DNC */
	}

}

static int cod3034x_hp_playback_init(struct snd_soc_codec *codec)
{
	dev_dbg(codec->dev, "%s called\n", __func__);

	/* 0x45: |0x10, DAC Input Data Selection */
	snd_soc_update_bits(codec, COD3034X_45_IF1_FORMAT5, 0x10, 0x10);

	/* Preset reghister for AVC */
#if 0
	snd_soc_write(codec, 0xF2, 0x03);
	snd_soc_write(codec, 0xF3, 0x1F);
	snd_soc_write(codec, 0xF5, 0x1A);
	snd_soc_write(codec, 0xF6, 0xC7);
#endif

	/* set HP volume Level */
	snd_soc_write(codec, COD3034X_30_VOL_HPL, 0x1A);
	snd_soc_write(codec, COD3034X_31_VOL_HPR, 0x1A);

	/* DVOLL & DVOLR */
	snd_soc_write(codec, COD3034X_51_DVOLL, 0x60);
	snd_soc_write(codec, COD3034X_52_DVOLR, 0x60);

	/* digital power reset after format setting. */
	snd_soc_write(codec, COD3034X_5C_AVC9, 0x01);
	/* snd_soc_write(codec, COD3034X_40_DIGITAL_POWER, 0x79); */
	snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER, 0x79, 0x79);


	/* Update OTP configuration */
	cod3034x_update_playback_otp(codec);
	cod3034x_usleep(100);

	return 0;
}

static int spkdrv_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int hp_on, spk_on, ep_on;
	int chop_val;
	unsigned int spk_gain;
	int offset;
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	chop_val = snd_soc_read(codec, COD3034X_77_CHOP_DA);
	hp_on = chop_val & EN_HP_CHOP_MASK;
	spk_on = chop_val & EN_SPK_PGA_CHOP_MASK;
	ep_on = chop_val & EN_EP_CHOP_MASK;

	if (!spk_on) {
		dev_dbg(codec->dev, "%s called but speaker not enabled\n",
				__func__);
		return 0;
	}
	dev_dbg(codec->dev, "%s called event=%d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Update OTP configuration */
		cod3034x_update_playback_otp(codec);

		/* CP Freq setting scenario rev 0.1*/
		snd_soc_write(codec, COD3034X_DD_CTRL_SPKS1, 0x82);

		snd_soc_update_bits(codec, COD3034X_38_MIX_DA2,
				EN_SPK_MIX_DCTL_MASK | EN_SPK_MIX_DCTR_MASK, 0);

		spk_gain = snd_soc_read(codec, COD3034X_32_VOL_EP_SPK);

		snd_soc_update_bits(codec, COD3034X_32_VOL_EP_SPK,
				CTVOL_SPK_PGA_MASK, 0);

		snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
				PW_AUTO_DA_MASK | APW_SPK_MASK,
				PW_AUTO_DA_MASK | APW_SPK_MASK);

		snd_soc_update_bits(codec, COD3034X_38_MIX_DA2,
				EN_SPK_MIX_DCTL_MASK | EN_SPK_MIX_DCTR_MASK,
				EN_SPK_MIX_DCTL_MASK | EN_SPK_MIX_DCTR_MASK);

		msleep(135);

		snd_soc_update_bits(codec, COD3034X_32_VOL_EP_SPK,
				CTVOL_SPK_PGA_MASK, spk_gain);

		break;

	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, COD3034X_32_VOL_EP_SPK,
				CTVOL_SPK_PGA_MASK, 0x6);

		if (hp_on || ep_on) {
			snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
					APW_SPK_MASK, 0);
		} else {
			snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
					PW_AUTO_DA_MASK | APW_SPK_MASK, 0);
		}
		cod3034x_usleep(200);

		snd_soc_update_bits(codec, COD3034X_38_MIX_DA2,
				EN_SPK_MIX_DCTL_MASK | EN_SPK_MIX_DCTR_MASK, 0);

		cod3034x_usleep(100);
		/* Check HP is ON */
		if (hp_on) {
			/* We are in HP only mode */
			/* Updating OTP register 0xD4 */
			offset = COD3034X_D4_OFFSET_DAL
				- COD3034X_OTP_REG_WRITE_START;
			snd_soc_write(codec, COD3034X_D4_OFFSET_DAL,
					cod3034x->otp_reg[offset]);

			/* Updating OTP register 0xD5 */
			offset = COD3034X_D5_OFFSET_DAR
				- COD3034X_OTP_REG_WRITE_START;
			snd_soc_write(codec, COD3034X_D5_OFFSET_DAR,
					cod3034x->otp_reg[offset]);

			snd_soc_write(codec, COD3034X_30_VOL_HPL,
					cod3034x->vol_hpl);
			snd_soc_write(codec, COD3034X_31_VOL_HPR,
					cod3034x->vol_hpr);
		}
		break;
	default:
		break;
	}

	return 0;
}

static int hpdrv_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int hp_on, spk_on, ep_on;
	int chop_val;
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	chop_val = snd_soc_read(codec, COD3034X_77_CHOP_DA);
	hp_on = chop_val & EN_HP_CHOP_MASK;
	spk_on = chop_val & EN_SPK_PGA_CHOP_MASK;
	ep_on = chop_val & EN_EP_CHOP_MASK;

	if (!hp_on) {
		dev_dbg(codec->dev, "%s called but headphone not enabled\n",
				__func__);
		return 0;
	}

	dev_dbg(codec->dev, "%s called, event = %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod3034x->vol_hpl = snd_soc_read(codec, COD3034X_30_VOL_HPL);
		cod3034x->vol_hpr = snd_soc_read(codec, COD3034X_31_VOL_HPR);
		cod3034x_hp_playback_init(codec);
		break;

	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, COD3034X_37_MIX_DA1,
				EN_HP_MIXL_DCTL_MASK | EN_HP_MIXR_DCTR_MASK,
				EN_HP_MIXL_DCTL_MASK | EN_HP_MIXR_DCTR_MASK);

		snd_soc_update_bits(codec, COD3034X_DC_CTRL_EPS,
				0x5F, 0x5F);

		snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
				APW_HP_MASK, APW_HP_MASK);

		msleep(135);

		if (!spk_on && !ep_on) {
			/* Only HP is on, enable DNC and set analog HP
			 * volume
			 */

			snd_soc_write(codec, COD3034X_30_VOL_HPL,
					cod3034x->vol_hpl);
			snd_soc_write(codec, COD3034X_31_VOL_HPR,
					cod3034x->vol_hpr);
		} else {
			/* Either SPK or EP is on, disable DNC and set given
			 * analog HP volume
			 */
			snd_soc_write(codec, COD3034X_30_VOL_HPL,
					cod3034x->vol_hpl);
			snd_soc_write(codec, COD3034X_31_VOL_HPR,
					cod3034x->vol_hpr);
		}

		break;

	case SND_SOC_DAPM_PRE_PMD:
		/* 0x54 <-- 0x05: ~0x02, Function Enable Register for AVC */
		/*
		 * snd_soc_update_bits(codec, COD3034X_54_AVC1,
		 *		AVC_MU_EN_MASK, 0x0);
		 */
		snd_soc_write(codec, 0x54, 0x05);

		/* 0x18 <-- 0x00: ~0x02, HP Path Auto Power Off */
		if (spk_on || ep_on) {
			snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
					APW_HP_MASK, 0);
		} else {
			snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
					PW_AUTO_DA_MASK | APW_HP_MASK, 0);
		}
		snd_soc_update_bits(codec, COD3034X_37_MIX_DA1,
				EN_HP_MIXL_DCTL_MASK | EN_HP_MIXR_DCTR_MASK, 0);
		cod3034x_usleep(2000);

		snd_soc_write(codec, 0x54, 0x07);
		/* 0x54 <-- 0x07: |0x02, Function Enable Register for AVC */
		/*
		 * snd_soc_update_bits(codec, COD3034X_54_AVC1,
		 *		AVC_MU_EN_MASK, AVC_MU_EN_MASK);
		 */
		break;

	default:
		break;
	}

	return 0;
}

static int epdrv_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	int hp_on, spk_on, ep_on;
	int chop_val;
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	chop_val = snd_soc_read(codec, COD3034X_77_CHOP_DA);
	hp_on = chop_val & EN_HP_CHOP_MASK;
	spk_on = chop_val & EN_SPK_PGA_CHOP_MASK;
	ep_on = chop_val & EN_EP_CHOP_MASK;

	if (!ep_on) {
		dev_dbg(codec->dev, "%s called but ear-piece not enabled\n",
				__func__);
		return 0;
	}
	dev_dbg(codec->dev, "%s called, event = %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		/* Update OTP configuration */
		cod3034x_update_playback_otp(codec);

		snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
				APW_EP_MASK | PW_AUTO_DA_MASK,
				APW_EP_MASK | PW_AUTO_DA_MASK);

		snd_soc_update_bits(codec, COD3034X_38_MIX_DA2,
				EN_EP_MIX_DCTL_MASK, EN_EP_MIX_DCTL_MASK);

		msleep(136);

		break;

	case SND_SOC_DAPM_PRE_PMD:
		if (spk_on || hp_on) {
			snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
					APW_EP_MASK, 0x0);
		} else {
			snd_soc_update_bits(codec, COD3034X_18_PWAUTO_DA,
					PW_AUTO_DA_MASK | APW_EP_MASK, 0x0);
		}
		cod3034x_usleep(100);

		snd_soc_update_bits(codec, COD3034X_38_MIX_DA2,
				EN_EP_MIX_DCTL_MASK, 0x0);

		cod3034x_usleep(100);

		break;

	default:
		break;
	}

	return 0;
}

static int mic2_pga_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod3034_power_on_mic2(codec);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		cod3034_power_off_mic2(codec);
		break;
	default:
		break;
	}

	return 0;
}

static int mic1_pga_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod3034_power_on_mic1(codec);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		cod3034_power_off_mic1(codec);
		break;

	default:
		break;
	}

	return 0;
}

static int mic3_pga_ev(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		cod3034_power_on_mic3(codec);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		cod3034_power_off_mic3(codec);
		break;

	default:
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new adcl_mix[] = {
	SOC_DAPM_SINGLE("MIC1L Switch", COD3034X_25_MIX_AD1,
			EN_MIX_MIC1L_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("MIC2L Switch", COD3034X_25_MIX_AD1,
			EN_MIX_MIC2L_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("MIC3L Switch", COD3034X_25_MIX_AD1,
			EN_MIX_MIC3L_SHIFT, 1, 0),
};

static const struct snd_kcontrol_new adcr_mix[] = {
	SOC_DAPM_SINGLE("MIC1R Switch", COD3034X_25_MIX_AD1,
			EN_MIX_MIC1R_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("MIC2R Switch", COD3034X_25_MIX_AD1,
			EN_MIX_MIC2R_SHIFT, 1, 0),
	SOC_DAPM_SINGLE("MIC3R Switch", COD3034X_25_MIX_AD1,
			EN_MIX_MIC3R_SHIFT, 1, 0),
};

static const struct snd_kcontrol_new spk_on[] = {
	SOC_DAPM_SINGLE("SPK On", COD3034X_77_CHOP_DA,
			EN_SPK_PGA_CHOP_SHIFT, 1, 0),
};

static const struct snd_kcontrol_new hp_on[] = {
	SOC_DAPM_SINGLE("HP On", COD3034X_77_CHOP_DA, EN_HP_CHOP_SHIFT, 1, 0),
};

static const struct snd_kcontrol_new ep_on[] = {
	SOC_DAPM_SINGLE("EP On", COD3034X_77_CHOP_DA, EN_EP_CHOP_SHIFT, 1, 0),
};

static const struct snd_kcontrol_new mic1_on[] = {
	SOC_DAPM_SINGLE("MIC1 On", COD3034X_76_CHOP_AD,
			EN_MIC_CHOP_SHIFT, 1, 0),
};

static const struct snd_kcontrol_new mic2_on[] = {
	SOC_DAPM_SINGLE("MIC2 On", COD3034X_76_CHOP_AD,
			EN_MIC_CHOP_SHIFT, 1, 0),
};

static const struct snd_kcontrol_new mic3_on[] = {
	SOC_DAPM_SINGLE("MIC3 On", COD3034X_76_CHOP_AD,
			EN_MIC_CHOP_SHIFT, 1, 0),
};

static const char * const cod3034x_fm_texts[] = {
	"None",
	"FM On",
};

static SOC_ENUM_SINGLE_VIRT_DECL(cod3034x_fm_enum, cod3034x_fm_texts);

static const struct snd_kcontrol_new cod3034x_fm_mux[] = {
	SOC_DAPM_ENUM("FM Link", cod3034x_fm_enum),
};

static const struct snd_soc_dapm_widget cod3034x_dapm_widgets[] = {
	SND_SOC_DAPM_SWITCH("SPK", SND_SOC_NOPM, 0, 0, spk_on),
	SND_SOC_DAPM_SWITCH("HP", SND_SOC_NOPM, 0, 0, hp_on),
	SND_SOC_DAPM_SWITCH("EP", SND_SOC_NOPM, 0, 0, ep_on),
	SND_SOC_DAPM_SWITCH("MIC1", SND_SOC_NOPM, 0, 0, mic1_on),
	SND_SOC_DAPM_SWITCH("MIC2", SND_SOC_NOPM, 0, 0, mic2_on),
	SND_SOC_DAPM_SWITCH("MIC3", SND_SOC_NOPM, 0, 0, mic3_on),

	SND_SOC_DAPM_SUPPLY("VMID", SND_SOC_NOPM, 0, 0, vmid_ev,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUT_DRV_E("SPKDRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			spkdrv_ev, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("EPDRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			epdrv_ev, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("HPDRV", SND_SOC_NOPM, 0, 0, NULL, 0,
			hpdrv_ev, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MUX("FM Link", SND_SOC_NOPM, 0, 0,
			cod3034x_fm_mux),
	SND_SOC_DAPM_PGA_E("MIC1_PGA", SND_SOC_NOPM, 0, 0,
			NULL, 0, mic1_pga_ev,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("MIC2_PGA", SND_SOC_NOPM, 0, 0,
			NULL, 0, mic2_pga_ev,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("MIC3_PGA", SND_SOC_NOPM, 0, 0,
			NULL, 0, mic3_pga_ev,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_MIXER("ADCL Mixer", SND_SOC_NOPM, 0, 0, adcl_mix,
			ARRAY_SIZE(adcl_mix)),
	SND_SOC_DAPM_MIXER("ADCR Mixer", SND_SOC_NOPM, 0, 0, adcr_mix,
			ARRAY_SIZE(adcr_mix)),

	SND_SOC_DAPM_DAC_E("DAC", "AIF Playback", SND_SOC_NOPM, 0, 0,
			dac_ev, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("DAC", "AIF2 Playback", SND_SOC_NOPM, 0, 0,
			dac_ev, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_ADC_E("ADC", "AIF Capture", SND_SOC_NOPM, 0, 0,
			adc_ev, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_ADC_E("ADC", "AIF2 Capture", SND_SOC_NOPM, 0, 0,
			adc_ev, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUTPUT("SPKOUTLN"),
	SND_SOC_DAPM_OUTPUT("HPOUTLN"),
	SND_SOC_DAPM_OUTPUT("EPOUTN"),
	SND_SOC_DAPM_OUTPUT("AIF4OUT"),

	SND_SOC_DAPM_INPUT("IN1L"),
	SND_SOC_DAPM_INPUT("IN2L"),
	SND_SOC_DAPM_INPUT("IN3L"),
	SND_SOC_DAPM_INPUT("IN4L"),

	SND_SOC_DAPM_INPUT("AIF4IN"),
};

static const struct snd_soc_dapm_route cod3034x_dapm_routes[] = {
	/* Sink, Control, Source */
	{"SPKDRV", NULL, "DAC"},
	{"SPK", "SPK On", "SPKDRV"},
	{"SPKOUTLN", NULL, "SPK"},

	{"EPDRV", NULL, "DAC"},
	{"EP", "EP On", "EPDRV"},
	{"EPOUTN", NULL, "EP"},

	{"HPDRV", NULL, "DAC"},
	{"HP", "HP On", "HPDRV"},
	{"HPOUTLN", NULL, "HP"},

	{"DAC", NULL, "AIF Playback"},
	{"DAC", NULL, "AIF2 Playback"},

	{"MIC1_PGA", NULL, "IN1L"},
	{"MIC1_PGA", NULL, "VMID"},
	{"MIC1", "MIC1 On", "MIC1_PGA"},

	{"ADCL Mixer", "MIC1L Switch", "MIC1"},
	{"ADCR Mixer", "MIC1R Switch", "MIC1"},

	{"MIC2_PGA", NULL, "IN2L"},
	{"MIC2_PGA", NULL, "VMID"},
	{"MIC2", "MIC2 On", "MIC2_PGA"},

	{"ADCL Mixer", "MIC2L Switch", "MIC2"},
	{"ADCR Mixer", "MIC2R Switch", "MIC2"},

	{"MIC3_PGA", NULL, "IN3L"},
	{"MIC3_PGA", NULL, "VMID"},
	{"MIC3", "MIC3 On", "MIC3_PGA"},

	{"ADCL Mixer", "MIC3L Switch", "MIC3"},
	{"ADCR Mixer", "MIC3R Switch", "MIC3"},

	{"ADC", NULL, "ADCL Mixer"},
	{"ADC", NULL, "ADCR Mixer"},

	{"AIF Capture", NULL, "ADC"},
	{"AIF2 Capture", NULL, "ADC"},

	{"FM Link", "FM On", "ADC"},
	{"DAC", NULL, "FM Link"},
};

static int cod3034x_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "%s called. fmt: %d\n", __func__, fmt);

	/* for in case the format can come diffrently */
	if (fmt == 0x4001)
		dev_dbg(codec->dev, "DAI format: 0x%x\n", fmt);
	else
		dev_dbg(codec->dev, "DAI format: 0x%x\n", fmt);

	/* I2S Mode */
	snd_soc_write(codec, COD3034X_41_IF1_FORMAT1, 0x00);

	/* length set to 16 bits */
	snd_soc_write(codec, COD3034X_42_IF1_FORMAT2, 0x10);

	/* BCLK : 32fs */
	snd_soc_write(codec, COD3034X_43_IF1_FORMAT3, 0x20);

	/* digital power reset after format setting. */
	snd_soc_write(codec, COD3034X_40_DIGITAL_POWER, 0x69);
	snd_soc_write(codec, COD3034X_40_DIGITAL_POWER, 0x79);
#if 0
	snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER, 0x69, 0x69);
	snd_soc_update_bits(codec, COD3034X_40_DIGITAL_POWER, 0x79, 0x79);
#endif


	return 0;
}

static int cod3034x_i2s_set_fmt(struct snd_soc_codec *codec)
{
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s called.\n", __func__);
	/* for in case the format can come diffrently */
	if (cod3034x->aifrate < 192000)
		dev_dbg(codec->dev, "DAI format: 0x%x\n", cod3034x->aifrate);
	else
		dev_dbg(codec->dev, "DAI format: 0x%x\n", cod3034x->aifrate);

	/* I2S Mode */
	snd_soc_write(codec, COD3034X_41_IF1_FORMAT1, 0x00);

	/* length set to 16 bits */
	snd_soc_write(codec, COD3034X_42_IF1_FORMAT2, 0x10);

	/* BCLK : 32fs */
	snd_soc_write(codec, COD3034X_43_IF1_FORMAT3, 0x20);

	/* digital power reset after format setting. */
	snd_soc_write(codec, COD3034X_40_DIGITAL_POWER, 0x69);
	snd_soc_write(codec, COD3034X_40_DIGITAL_POWER, 0x79);

	return 0;
}

static int cod3034x_dai_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "(%s) %s completed\n",
			substream->stream ? "C" : "P", __func__);


	return 0;
}

static void cod3034x_dai_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	dev_dbg(codec->dev, "(%s) %s completed\n",
			substream->stream ? "C" : "P", __func__);
}

static int cod3034x_dai_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);
	unsigned int cur_aifrate;
	unsigned char ctrl_hps, hp_current_val = CTMI_HP_4_UA;

	dev_dbg(codec->dev, "%s called\n", __func__);

	/* 192 KHz support */
	cur_aifrate = params_rate(params);
	dev_dbg(codec->dev, "%s : aifrate - 0x%x\n", __func__, cur_aifrate);
	if (cod3034x->aifrate != cur_aifrate) {
		/*
		 * If HP is already on, then change the 'current' setting based
		 * on samplerate
		 */
		if (APW_HP_MASK == (snd_soc_read(codec, COD3034X_18_PWAUTO_DA)
					& APW_HP_MASK)) {
			ctrl_hps = snd_soc_read(codec, COD3034X_DB_CTRL_HPS);
			ctrl_hps &= ~CTMI_HP_A_MASK;
			ctrl_hps |= hp_current_val << CTMI_HP_A_SHIFT;
			snd_soc_write(codec, COD3034X_DB_CTRL_HPS, ctrl_hps);
		}

		cod3034x->aifrate = cur_aifrate;
	}

	/*
	 * Codec supports only 24bits per sample, Mixer performs the required
	 * conversion to 24 bits. BFS is fixed at 64fs for mixer<->codec
	 * interface.
	 */


	return 0;
}


static void cod3034x_jack_det_work(struct work_struct *work)
{
	struct cod3034x_priv *cod3034x =
		container_of(work, struct cod3034x_priv, jack_det_work);
	struct cod3034x_jack_det *jackdet = &cod3034x->jack_det;
	struct snd_soc_codec *codec = cod3034x->codec;
	int adc;

	dev_dbg(cod3034x->dev, "%s(%d) jackdet: %d\n",
			__func__, __LINE__, jackdet->jack_det);
	mutex_lock(&cod3034x->jackdet_lock);

	if (jackdet->jack_det == true) {
		/* set delay for read correct adc value */
		msleep(cod3034x->mic_det_delay);

		/* read adc for mic detect */
		adc = cod3034x_adc_get_value(cod3034x);
		dev_dbg(cod3034x->dev, "%s mic det adc  %d\n", __func__, adc);

		if (adc > cod3034x->mic_adc_range)
			jackdet->mic_det = true;
		else
			jackdet->mic_det = false;

		dev_dbg(cod3034x->dev, "%s Mic det: %d\n",
				__func__, jackdet->mic_det);
		jackdet->adc_val = adc;
	} else {
		dev_dbg(cod3034x->dev, "%s JACK OUT\n", __func__);
		/* jack/mic out */
		jackdet->mic_det = false;
		jackdet->adc_val = -EINVAL;
	}

	/* Send the jack detect event to the audio framework */
	if (jackdet->jack_det && jackdet->mic_det)
		switch_set_state(&cod3034x->sdev, 1);	/* 4 Pole */
	else if (jackdet->jack_det)
		switch_set_state(&cod3034x->sdev, 2);	/* 3 Pole */
	else
		switch_set_state(&cod3034x->sdev, 0);


	if (jackdet->jack_det && jackdet->mic_det) {
		/* 4 Pole Jack-in */
		snd_soc_write(codec, 0x92, 0x30);
		/* button threshold */
		snd_soc_write(codec, COD3034X_88_CTR_IMP3, 0x10);
		dev_dbg(cod3034x->dev, "%s 4 Pole Jack-In\n", __func__);
	} else if (jackdet->jack_det) {
		/* 3 Pole Jack-in */
		snd_soc_write(codec, 0x92, 0x20);
		/* button threshold */
		snd_soc_write(codec, COD3034X_88_CTR_IMP3, 0x10);
		dev_dbg(cod3034x->dev, "%s 3 Pole Jack-In\n", __func__);
	} else {
		/* Jack-out */
		snd_soc_write(codec, 0x92, 0x00);
		dev_dbg(cod3034x->dev, "%s JACK OUT\n", __func__);
	}

	dev_dbg(cod3034x->codec->dev, "Jack %s, Mic %s\n",
			jackdet->jack_det ? "inserted" : "removed",
			jackdet->mic_det ? "inserted" : "removed");

	mutex_unlock(&cod3034x->jackdet_lock);
}

#define ADC_TRACE_NUM		5
#define ADC_TRACE_NUM2		2
#define ADC_READ_DELAY_US	500
#define ADC_READ_DELAY_MS	1
#define ADC_DEVI_THRESHOLD	18000

#define BUTTON_PRESS 1
#define BUTTON_RELEASE 0

static int get_adc_avg(int *adc_values)
{
	int i;
	int adc_sum = 0;

	for (i = 0; i < ADC_TRACE_NUM; i++)
		adc_sum += adc_values[i];

	adc_sum = adc_sum / ADC_TRACE_NUM;
	return adc_sum;
}

static int get_adc_devi(int avg, int *adc_values)
{
	int i;
	int devi = 0, diff;

	for (i = 0; i < ADC_TRACE_NUM; i++) {
		diff = adc_values[i]-avg;
		devi += (diff*diff);
	}
	return devi;
}

static void cod3034x_buttons_work(struct work_struct *work)
{
	struct cod3034x_priv *cod3034x =
		container_of(work, struct cod3034x_priv, buttons_work);
	struct cod3034x_jack_det *jd = &cod3034x->jack_det;
	struct jack_buttons_zone *btn_zones = cod3034x->jack_buttons_zones;
	int num_buttons_zones = ARRAY_SIZE(cod3034x->jack_buttons_zones);
	int adc_values[ADC_TRACE_NUM];
	int current_button_state;
	int adc;
	int i, avg, devi;
	int adc_final_values[ADC_TRACE_NUM2];
	int j;
	int adc_final = 0;
	int adc_max = 0;

	if (!jd->jack_det) {
		dev_err(cod3034x->dev, "Skip button events for jack_out\n");
		return;
	}

	if (!jd->mic_det) {
		dev_err(cod3034x->dev, "Skip button events for 3-pole jack\n");
		return;
	}

	/* set delay for read correct adc value */
	mdelay(10);

	for (j = 0; j < ADC_TRACE_NUM2; j++) {
		/* read GPADC for button */
		for (i = 0; i < ADC_TRACE_NUM; i++) {
			adc = cod3034x_adc_get_value(cod3034x);
			adc_values[i] = adc;
			udelay(ADC_READ_DELAY_US);
		}

		/*
		 * check avg/devi value is proper
		 * if not read adc after 5 ms
		 */
		avg = get_adc_avg(adc_values);
		devi = get_adc_devi(avg, adc_values);
		dev_dbg(cod3034x->dev,
				":button adc avg: %d, devi: %d\n", avg, devi);

		if (devi > ADC_DEVI_THRESHOLD) {
			queue_work(cod3034x->buttons_wq,
					&cod3034x->buttons_work);

			for (i = 0; i < ADC_TRACE_NUM;) {
				dev_err(cod3034x->dev,
						":retry button_work :  %d %d %d %d %d\n",
						adc_values[i + 0],
						adc_values[i + 1],
						adc_values[i + 2],
						adc_values[i + 3],
						adc_values[i + 4]);

				i += 5;
			}
			return;
		}
		adc_final_values[j] = avg;

		if (avg > adc_max)
			adc_max = avg;
		mdelay(ADC_READ_DELAY_MS);
	}
	adc_final = adc_max;

	/* check button press/release */
	if (adc_final > cod3034x->btn_release_value) {
		dev_dbg(cod3034x->dev,
				"Button Released! adc_fanal: %d, btn_value: %d\n",
				adc_final,
				cod3034x->btn_release_value);

		current_button_state = BUTTON_RELEASE;
	} else {
		dev_dbg(cod3034x->dev,
				"Button Pressed! adc_final: %d, btn_value: %d\n",
				adc_final,
				cod3034x->btn_release_value);

		current_button_state = BUTTON_PRESS;
	}

	if (jd->privious_button_state == current_button_state) {
		dev_dbg(cod3034x->dev, "status are same\n");
		return;
	}

	jd->privious_button_state = current_button_state;

	adc = adc_final;
	jd->adc_val = adc_final;

	for (i = 0; i < 4; i++)
		dev_dbg(cod3034x->dev,
				"[DEBUG]: adc(%d), buttons: code(%d), low(%d), high(%d)\n",
				adc,
				cod3034x->jack_buttons_zones[i].code,
				cod3034x->jack_buttons_zones[i].adc_low,
				cod3034x->jack_buttons_zones[i].adc_high);

	/* determine which button press or release */
	if (current_button_state == BUTTON_PRESS) {
		for (i = 0; i < num_buttons_zones; i++)
			if (adc >= btn_zones[i].adc_low &&
					adc <= btn_zones[i].adc_high) {
				jd->button_code = btn_zones[i].code;
				input_report_key(cod3034x->input,
						jd->button_code,
						1);

				input_sync(cod3034x->input);
				jd->button_det = true;
				cod3034x_process_button_ev(cod3034x->codec,
						jd->button_code,
						1);

				dev_dbg(cod3034x->dev, ":key %d is pressed, adc %d\n",
						btn_zones[i].code, adc);
				return;
			}

		dev_dbg(cod3034x->dev, ":key skipped. ADC %d\n", adc);
	} else {
		jd->button_det = false;
		input_report_key(cod3034x->input, jd->button_code, 0);
		input_sync(cod3034x->input);
		cod3034x_process_button_ev(cod3034x->codec,
				jd->button_code,
				0);

		dev_dbg(cod3034x->dev, ":key %d released\n", jd->button_code);
	}

	return;
}


int cod3034x_jack_mic_register(struct snd_soc_codec *codec)
{
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);
	int i, ret;

	cod3034x->sdev.name = "h2w";

	ret = switch_dev_register(&cod3034x->sdev);
	if (ret < 0)
		dev_err(codec->dev, "Switch registration failed\n");

	cod3034x->input = devm_input_allocate_device(codec->dev);
	if (!cod3034x->input) {
		dev_err(codec->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	/* Not handling Headset events for now.Headset event handling
	 * registered as Input device, causing some conflict with Keyboard Input
	 * device.So, temporarily not handling Headset event, it will be enabled
	 * after proper fix.
	 */
	cod3034x->input->name = "Codec3034 Headset Events";
	cod3034x->input->phys = dev_name(codec->dev);
	cod3034x->input->id.bustype = BUS_I2C;

	cod3034x->input->evbit[0] = BIT_MASK(EV_KEY);
	for (i = 0; i < 4; i++)
		set_bit(cod3034x->jack_buttons_zones[i].code, cod3034x->input->keybit);
	cod3034x->input->dev.parent = codec->dev;
	input_set_drvdata(cod3034x->input, codec);

	ret = input_register_device(cod3034x->input);
	if (ret != 0) {
		cod3034x->input = NULL;
		dev_err(codec->dev, "Failed to register 3034 input device\n");
	}

#ifdef CONFIG_PM
	pm_runtime_get_sync(codec->dev);
#else
	cod3034x_enable(codec->dev);
#endif

#ifdef CONFIG_PM
	pm_runtime_put_sync(codec->dev);
#else
	cod3034x_disable(codec->dev);
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(cod3034x_jack_mic_register);

static const struct snd_soc_dai_ops cod3034x_dai_ops = {
	.set_fmt = cod3034x_dai_set_fmt,
	.startup = cod3034x_dai_startup,
	.shutdown = cod3034x_dai_shutdown,
	.hw_params = cod3034x_dai_hw_params,
};

#define COD3034X_RATES		SNDRV_PCM_RATE_8000_192000

#define COD3034X_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE |		\
		SNDRV_PCM_FMTBIT_S20_3LE |	\
		SNDRV_PCM_FMTBIT_S24_LE |	\
		SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver cod3034x_dai[] = {
	{
		.name = "cod3034x-aif",
		.id = 1,
		.playback = {
			.stream_name = "AIF Playback",
			.channels_min = 1,
			.channels_max = 8,
			.rates = COD3034X_RATES,
			.formats = COD3034X_FORMATS,
		},
		.capture = {
			.stream_name = "AIF Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = COD3034X_RATES,
			.formats = COD3034X_FORMATS,
		},
		.ops = &cod3034x_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "cod3034x-aif2",
		.id = 2,
		.playback = {
			.stream_name = "AIF2 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = COD3034X_RATES,
			.formats = COD3034X_FORMATS,
		},
		.capture = {
			.stream_name = "AIF2 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = COD3034X_RATES,
			.formats = COD3034X_FORMATS,
		},
		.ops = &cod3034x_dai_ops,
		.symmetric_rates = 1,
	},
};

static int cod3034x_regulators_enable(struct snd_soc_codec *codec)
{
	int ret;
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	ret = regulator_enable(cod3034x->vdd);

	return ret;
}

static void cod3034x_regulators_disable(struct snd_soc_codec *codec)
{
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	regulator_disable(cod3034x->vdd);
}

static void cod3034x_save_otp_registers(struct snd_soc_codec *codec)
{
	int i;
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s called\n", __func__);
	for (i = 0; i < COD3034X_OTP_MAX_REG; i++) {
		cod3034x->otp_reg[i] = (unsigned char) snd_soc_read(codec,
				(COD3034X_D0_CTRL_IREF1 + i));
	}
}

static void cod3034x_restore_otp_registers(struct snd_soc_codec *codec)
{
	int i;
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "%s called\n", __func__);
	for (i = 0; i < COD3034X_OTP_MAX_REG; i++) {
		snd_soc_write(codec, (COD3034X_D0_CTRL_IREF1 + i),
				cod3034x->otp_reg[i]);
	}
}

static void cod3034x_reset_io_selector_bits(struct snd_soc_codec *codec)
{
	/* Reset output selector bits */
	snd_soc_update_bits(codec, COD3034X_77_CHOP_DA,
			EN_HP_CHOP_MASK | EN_EP_CHOP_MASK |
			EN_SPK_PGA_CHOP_MASK, 0x0);
}

/*
 * Configure the mic1 and mic2 bias voltages with default value or the value
 * received from the device tree.
 * Also configure the internal LDO voltage.
 */
static void cod3034x_configure_mic_bias(struct snd_soc_codec *codec)
{
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	/* Configure Mic1 Bias Voltage */
	snd_soc_update_bits(codec, COD3034X_19_CTRL_REF,
			CTRV_MCB1_MASK,
			(cod3034x->mic_bias1_voltage << CTRV_MCB1_SHIFT));

	/* Configure Mic2 Bias Voltage */
	snd_soc_update_bits(codec, COD3034X_82_CTR_MCB,
			CTRV_MCB2_MASK,
			(cod3034x->mic_bias2_voltage << CTRV_MCB2_SHIFT));

	/* Configure Mic Bias LDO Voltage */
	snd_soc_update_bits(codec, COD3034X_82_CTR_MCB,
			CTRV_MCB_LDO_MASK,
			(cod3034x->mic_bias_ldo_voltage << CTRV_MCB_LDO_SHIFT));
}

/**
 * cod3034x_codec_initialize : To be called if f/w update fails
 *
 * In case the firmware is not present or corrupt, we should still be able to
 * run the codec with decent parameters. This values are updated as per the
 * latest stable firmware.
 *
 * The values provided in this function are hard-coded register values, and we
 * need not update these values as per bit-fields.
 */
static void cod3034x_codec_initialize(void *context)
{
	struct snd_soc_codec *codec = (struct snd_soc_codec *)context;
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);
	unsigned int detb_period = CTMF_DETB_PERIOD_2048;
	unsigned int jd_mask = EN_PDB_JD_MASK;

	dev_dbg(codec->dev, "%s called, setting defaults\n", __func__);

	if (cod3034x->use_external_jd) {
		detb_period = CTMF_DETB_PERIOD_8;
		jd_mask = 0;
	} else if (cod3034x->use_btn_adc_mode)
		detb_period = CTMF_DETB_PERIOD_8;

#ifdef CONFIG_PM
	pm_runtime_get_sync(codec->dev);
#else
	cod3034x_enable(codec->dev);
#endif

	/* Default value, enabling HPF and setting freq at 100Hz */
	snd_soc_write(codec, COD3034X_46_ADC1, 0x0c);

	snd_soc_update_bits(codec, COD3034X_D0_CTRL_IREF1,
			CTMI_VCM_MASK | CTMI_MIX_MASK,
			(CTMI_VCM_4U << CTMI_VCM_SHIFT) | CTMI_MIX_2U);

	snd_soc_update_bits(codec, COD3034X_D1_CTRL_IREF2,
			CTMI_INT1_MASK, CTMI_INT1_4U);

	snd_soc_update_bits(codec, COD3034X_D2_CTRL_IREF3,
			CTMI_MIC2_MASK | CTMI_MIC1_MASK,
			(CTMI_MIC2_2U << CTMI_MIC2_SHIFT) | CTMI_MIC1_2U);

	snd_soc_update_bits(codec, COD3034X_D3_CTRL_IREF4,
			CTMI_MIC_BUFF_MASK | CTMI_MIC3_MASK,
			(CTMI_MIC_BUFF_2U << CTMI_MIC_BUFF_SHIFT) | CTMI_MIC3_2U);

	snd_soc_write(codec, COD3034X_47_AVOLL1, 0x18);
	snd_soc_write(codec, COD3034X_48_AVOLR1, 0x18);
	/* Boost 20 dB, Gain 0 dB for MIC1/MIC2/MIC3 */
	snd_soc_write(codec, COD3034X_20_VOL_AD1, 0x54);
	snd_soc_write(codec, COD3034X_21_VOL_AD2, 0x54);
	snd_soc_write(codec, COD3034X_22_VOL_AD3, 0x54);
	snd_soc_write(codec, COD3034X_22_VOL_AD3, 0x54);
	snd_soc_write(codec, COD3034X_36_CTRL_SPK, 0x00);

	/* Jack detect after reboot */
	snd_soc_write(codec, COD3034X_80_PDB_ACC1, 0x02);
	udelay(1000);
	snd_soc_write(codec, COD3034X_80_PDB_ACC1, 0x03);

	/* default HP Volume setting */
	snd_soc_write(codec, COD3034X_51_DVOLL, 0x60);
	snd_soc_write(codec, COD3034X_52_DVOLR, 0x60);

	/* Reset input/output selector bits */
	/* cod3034x_reset_io_selector_bits(codec); */

	/* Configure mic bias voltage */
	cod3034x_configure_mic_bias(codec);

	/* All boot time hardware access is done. Put the device to sleep. */
#ifdef CONFIG_PM
	pm_runtime_put_sync(codec->dev);
#else
	cod3034x_disable(codec->dev);
#endif
}

/**
 * cod3034x_post_fw_update_success: To be called after f/w update
 *
 * The firmware may be enabling some of the path and power registers which are
 * used during path enablement. We need to keep the values of these registers
 * consistent so that the functionality of the codec driver doesn't change
 * because of the firmware.
 */

static void cod3034x_regmap_sync(struct device *dev)
{
	struct cod3034x_priv *cod3034x = dev_get_drvdata(dev);
	unsigned char reg[COD3034X_MAX_REGISTER] = {0,};
	int i;

	/* Read from Cache */
	for (i = 0; i < COD3034X_REGCACHE_SYNC_END_REG; i++)
		if (cod3034x_readable_register(dev, i) &&
				(!cod3034x_volatile_register(dev, i)))
			reg[i] = (unsigned char)
				snd_soc_read(cod3034x->codec, i);

	snd_soc_write(cod3034x->codec, COD3034X_40_DIGITAL_POWER,
			reg[COD3034X_40_DIGITAL_POWER]);

	regcache_sync(cod3034x->regmap);
}

static void cod3034x_reg_restore(struct snd_soc_codec *codec)
{
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	snd_soc_update_bits(codec, COD3034X_80_PDB_ACC1,
			PDB_JD_CLK_EN_MASK, PDB_JD_CLK_EN_MASK);

	/* Give 15ms delay before storing the otp values */
	usleep_range(15000, 15000 + 1000);

	/*
	 * The OTP values are the boot-time values. For registers D0-DE, we need
	 * to save these register values during boot time. After system reset,
	 * these values are lost and we need to restore them using saved values.
	 */
	if (!cod3034x->is_probe_done) {
		cod3034x_regmap_sync(codec->dev);
		cod3034x_reset_io_selector_bits(codec);
	} else {
		cod3034x_regmap_sync(codec->dev);
		cod3034x_restore_otp_registers(cod3034x->codec);
	}
}

static void cod3034x_i2c_parse_dt(struct cod3034x_priv *cod3034x)
{
	/* todo .. Need to add DT parsing for 3034 */
	struct device *dev = cod3034x->dev;
	struct device_node *np = dev->of_node;
	unsigned int bias_v_conf;
	int mic_range, mic_delay, btn_rel_val;
	struct of_phandle_args args;
	int i = 0;
	int ret;

	cod3034x->int_gpio = of_get_gpio(np, 0);

	if (cod3034x->int_gpio < 0)
		dev_err(dev, "(*)Error in getting Codec-3034 Interrupt gpio\n");

	/* Default Bias Voltages */
	cod3034x->mic_bias1_voltage = MIC_BIAS1_VO_3_0V;
	cod3034x->mic_bias2_voltage = MIC_BIAS2_VO_3_0V;
	cod3034x->mic_bias_ldo_voltage = MIC_BIAS_LDO_VO_3_3V;

	ret = of_property_read_u32(dev->of_node,
			"mic-bias1-voltage", &bias_v_conf);
	if ((!ret) && ((bias_v_conf >= MIC_BIAS1_VO_2_8V) &&
				(bias_v_conf <= MIC_BIAS1_VO_3_0V)))
		cod3034x->mic_bias1_voltage = bias_v_conf;
	else
		dev_dbg(dev, "Property 'mic-bias1-voltage' %s",
				ret ? "not found" : "invalid value (use:1-3)");

	ret = of_property_read_u32(dev->of_node,
			"mic-bias2-voltage", &bias_v_conf);
	if ((!ret) && ((bias_v_conf >= MIC_BIAS2_VO_2_8V) &&
				(bias_v_conf <= MIC_BIAS2_VO_3_0V)))
		cod3034x->mic_bias2_voltage = bias_v_conf;
	else
		dev_dbg(dev, "Property 'mic-bias2-voltage' %s",
				ret ? "not found" : "invalid value (use:1-3)");

	ret = of_property_read_u32(dev->of_node, "mic-adc-range", &mic_range);
	if (!ret)
		cod3034x->mic_adc_range = mic_range;
	else
		cod3034x->mic_adc_range = 1120;

	ret = of_property_read_u32(dev->of_node, "mic-det-delay", &mic_delay);
	if (!ret)
		cod3034x->mic_det_delay = mic_delay;
	else
		cod3034x->mic_det_delay = 150;

	ret = of_property_read_u32(dev->of_node,
			"btn-release-value", &btn_rel_val);
	if (!ret)
		cod3034x->btn_release_value = btn_rel_val;
	else
		cod3034x->btn_release_value = 1100;

	ret = of_property_read_u32(dev->of_node,
			"mic-bias-ldo-voltage", &bias_v_conf);
	if ((!ret) && ((bias_v_conf >= MIC_BIAS_LDO_VO_2_8V) &&
				(bias_v_conf <= MIC_BIAS_LDO_VO_3_3V)))
		cod3034x->mic_bias_ldo_voltage = bias_v_conf;
	else
		dev_dbg(dev, "Property 'mic-bias-ldo-voltage' %s",
				ret ? "not found" : "invalid value (use:1-3)");

	dev_dbg(dev, "Bias voltage values: bias1 = %d, bias2= %d, ldo = %d\n",
			cod3034x->mic_bias1_voltage,
			cod3034x->mic_bias2_voltage,
			cod3034x->mic_bias_ldo_voltage);

	if (of_find_property(dev->of_node,
				"update-firmware", NULL))
		cod3034x->update_fw = true;
	else
		cod3034x->update_fw = false;

	if (of_find_property(dev->of_node, "use-btn-adc-mode", NULL) != NULL)
		cod3034x->use_btn_adc_mode = true;

	dev_err(dev, "Using %s for button detection\n",
			cod3034x->use_btn_adc_mode ? "GPADC" : "internal h/w");

	if (cod3034x->use_btn_adc_mode) {
		/* Parsing but-zones, a maximum of 4 buttons are supported */
		for (i = 0; i < 4; i++) {
			if (of_parse_phandle_with_args(dev->of_node,
						"but-zones-list",
						"#list-but-cells", i, &args))
				break;

			cod3034x->jack_buttons_zones[i].code = args.args[0];
			cod3034x->jack_buttons_zones[i].adc_low = args.args[1];
			cod3034x->jack_buttons_zones[i].adc_high = args.args[2];
		}
		/* initialize button status */
		cod3034x->jack_det.privious_button_state = BUTTON_RELEASE;

		for (i = 0; i < 4; i++)
			dev_err(dev, "[DEBUG]: buttons: code(%d), low(%d), high(%d)\n",
					cod3034x->jack_buttons_zones[i].code,
					cod3034x->jack_buttons_zones[i].adc_low,
					cod3034x->jack_buttons_zones[i].adc_high);
	}
}

struct codec_notifier_struct {
	struct cod3034x_priv *cod3034x;
};
static struct codec_notifier_struct codec_notifier_t;

static int cod3034x_notifier_handler(struct notifier_block *nb,
		unsigned long insert,
		void *data)
{
	struct codec_notifier_struct *codec_notifier_data = data;
	struct cod3034x_priv *cod3034x = codec_notifier_data->cod3034x;
	struct cod3034x_jack_det *jd = &cod3034x->jack_det;
	unsigned int  stat1, pend1, pend2, pend3, pend4;
	int jackdet = COD3034X_MJ_DET_INVALID;
	bool det_status_change = false;

	mutex_lock(&cod3034x->key_lock);

	pend1 = cod3034x->irq_val[0];
	pend2 = cod3034x->irq_val[1];
	pend3 = cod3034x->irq_val[2];
	pend4 = cod3034x->irq_val[3];
	stat1 = cod3034x->irq_val[4];

	dev_dbg(cod3034x->dev,
			"[DEBUG] %s , line %d 01: %02x, 02:%02x, 03:%02x, 04:%02x\n",
			__func__, __LINE__, pend1, pend2, pend3, pend4);
	/*
	 * Sequence for Jack/Mic detection
	 *
	 * (JACK bit 0, MIC bit 1)
	 *
	 * 1. Check bits in IRQ2PEND and IRQ3PEND.
	 * 2. If either of them is 1, then the STATUS1 register tells current
	 * status of Jack/Mic. Connected if bit value is 1, removed otherwise.
	 */
	if ((pend1 & IRQ1_JACK_DET_R) || (pend2 & IRQ2_JACK_DET_F)) {
		det_status_change = true;
		jackdet = stat1 & BIT(STATUS1_JACK_DET_SHIFT);
		jd->jack_det = jackdet ? true : false;
	}

	if (det_status_change) {
		cancel_work_sync(&cod3034x->jack_det_work);
		queue_work(cod3034x->jack_det_wq, &cod3034x->jack_det_work);

		mutex_unlock(&cod3034x->key_lock);
		goto out;
	}

	if (cod3034x->use_btn_adc_mode) {
		/* start button work */
		queue_work(cod3034x->buttons_wq, &cod3034x->buttons_work);
	} else {
		pr_err("[DEBUG] %s , line %d\n", __func__, __LINE__);
		/* need to implement button detection */
	}

	mutex_unlock(&cod3034x->key_lock);

out:

	return IRQ_HANDLED;
}
static BLOCKING_NOTIFIER_HEAD(cod3034x_notifier);

int cod3034x_register_notifier(struct notifier_block *n,
		struct cod3034x_priv *cod3034x)
{
	int ret;

	codec_notifier_t.cod3034x = cod3034x;
	ret = blocking_notifier_chain_register(&cod3034x_notifier, n);
	if (ret < 0)
		pr_err("[DEBUG] %s(%d)\n", __func__, __LINE__);
	return ret;
}


/* Notifier registration.
 * MFD driver get interrupts from PMIC and MFD filters the interrups.
 * if the interrup belongs to codec,
 * then it notify the interrupt to the codec.
 * The notifier is the way to communicate btw them
 *
 * The notifier contains
 * >> the irq1, irq2, irq3, irq4 and status registers information of codec.
 */
void cod3034x_call_notifier(int irq1, int irq2, int irq3, int irq4, int status1)
{
	struct cod3034x_priv *cod3034x = codec_notifier_t.cod3034x;

	dev_dbg(cod3034x->dev,
			"[DEBUG] %s(%d)  0x1: %02x 0x2: %02x 0x3: %02x 0x4: %02x\n",
			__func__, __LINE__, irq1, irq2, irq3, irq4);

	cod3034x->irq_val[0] = irq1;
	cod3034x->irq_val[1] = irq2;
	cod3034x->irq_val[2] = irq3;
	cod3034x->irq_val[3] = irq4;
	cod3034x->irq_val[4] = status1;

	blocking_notifier_call_chain(&cod3034x_notifier, 0, &codec_notifier_t);
}
EXPORT_SYMBOL(cod3034x_call_notifier);
struct notifier_block codec_notifier;

static int cod3034x_codec_probe(struct snd_soc_codec *codec)
{
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "3034x CODEC_PROBE: (*) %s\n", __func__);
	cod3034x->codec = codec;

	cod3034x->vdd = devm_regulator_get(codec->dev, "vdd_ldo27");
	if (IS_ERR(cod3034x->vdd)) {
		dev_warn(codec->dev, "failed to get regulator vdd\n");
		return PTR_ERR(cod3034x->vdd);
	}

	/* reset */
	snd_soc_write(codec, COD3034X_05_IRQ1M, 0xff);
	snd_soc_write(codec, COD3034X_06_IRQ2M, 0xff);
	snd_soc_write(codec, COD3034X_07_IRQ3M, 0xff);
	snd_soc_write(codec, COD3034X_08_IRQ4M, 0xff);

	snd_soc_write(codec, COD3034X_81_PDB_ACC2, 0x10);
	msleep(100);
	snd_soc_write(codec, COD3034X_81_PDB_ACC2, 0x00);


#ifdef CONFIG_PM
	pm_runtime_get_sync(codec->dev);
#else
	cod3034x_enable(codec->dev);
#endif

	cod3034x->is_probe_done = true;

	/* Initialize work queue for button handling */
	INIT_WORK(&cod3034x->buttons_work, cod3034x_buttons_work);

	cod3034x->buttons_wq = create_singlethread_workqueue("buttons_wq");
	if (cod3034x->buttons_wq == NULL) {
		dev_err(codec->dev, "Failed to create buttons_wq\n");
		return -ENOMEM;
	}

	INIT_WORK(&cod3034x->jack_det_work, cod3034x_jack_det_work);

	cod3034x->jack_det_wq = create_singlethread_workqueue("jack_det_wq");
	if (cod3034x->jack_det_wq == NULL) {
		dev_err(codec->dev, "Failed to create jack_det_wq\n");
		return -ENOMEM;
	}

	cod3034x_adc_start(cod3034x);

	cod3034x->aifrate = COD3034X_SAMPLE_RATE_48KHZ;

	cod3034x_i2c_parse_dt(cod3034x);

	cod3034x->jack_det.adc_val = -EINVAL;

	mutex_init(&cod3034x->jackdet_lock);
	mutex_init(&cod3034x->key_lock);

	/*
	 * interrupt pin should be shared with pmic.
	 * so codec driver use notifier because of knowing
	 * the interrupt status from mfd.
	 */
	codec_notifier.notifier_call = cod3034x_notifier_handler,
		cod3034x_register_notifier(&codec_notifier, cod3034x);

	set_codec_notifier_flag();

	cod3034x_usleep(10);

	cod3034x_codec_initialize(codec);
	cod3034x_save_otp_registers(codec);

	snd_soc_write(codec, COD3034X_05_IRQ1M, 0x0);
	snd_soc_write(codec, COD3034X_06_IRQ2M, 0x0);
	snd_soc_write(codec, COD3034X_07_IRQ3M, 0x0);
	snd_soc_write(codec, COD3034X_08_IRQ4M, 0x0);
	snd_soc_write(codec, COD3034X_80_PDB_ACC1, 0x00);
	msleep(100);
	snd_soc_write(codec, COD3034X_80_PDB_ACC1, 0x03);

	/* it should be modify to move machine driver */
	cod3034x_jack_mic_register(codec);

	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "SPKOUTLN");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "HPOUTLN");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "EPOUTN");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN1L");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN2L");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN3L");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "IN4L");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF Capture");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF2 Playback");
	snd_soc_dapm_ignore_suspend(snd_soc_codec_get_dapm(codec), "AIF2 Capture");
	snd_soc_dapm_sync(snd_soc_codec_get_dapm(codec));


#ifdef CONFIG_PM
	pm_runtime_put_sync(codec->dev);
#else
	cod3034x_disable(codec->dev);
#endif
	return 0;
}

static int cod3034x_codec_remove(struct snd_soc_codec *codec)
{
	struct cod3034x_priv *cod3034x = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "(*) %s called\n", __func__);
	cancel_delayed_work_sync(&cod3034x->key_work);
	if (cod3034x->int_gpio) {
		free_irq(gpio_to_irq(cod3034x->int_gpio), cod3034x);
		gpio_free(cod3034x->int_gpio);
	}

	cod3034x_regulators_disable(codec);
	destroy_workqueue(cod3034x->buttons_wq);
	destroy_workqueue(cod3034x->jack_det_wq);
	cod3034x_adc_stop(cod3034x);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_cod3034x = {
	.probe = cod3034x_codec_probe,
	.remove = cod3034x_codec_remove,
	.controls = cod3034x_snd_controls,
	.num_controls = ARRAY_SIZE(cod3034x_snd_controls),
	.dapm_widgets = cod3034x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cod3034x_dapm_widgets),
	.dapm_routes = cod3034x_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(cod3034x_dapm_routes),
	.ignore_pmdown_time = true,
	.idle_bias_off = true,
};

static int cod3034x_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct cod3034x_priv *cod3034x;
	struct pinctrl *pinctrl;
	int ret;

	cod3034x = kzalloc(sizeof(struct cod3034x_priv), GFP_KERNEL);
	if (cod3034x == NULL)
		return -ENOMEM;
	cod3034x->dev = &i2c->dev;
	cod3034x->i2c_addr = i2c->addr;
	cod3034x->use_external_jd = false;
	cod3034x->is_probe_done = false;
	cod3034x->use_btn_adc_mode = false;

	cod3034x->regmap = devm_regmap_init_i2c(i2c, &cod3034x_regmap);
	if (IS_ERR(cod3034x->regmap)) {
		dev_err(&i2c->dev, "Failed to allocate regmap: %li\n",
				PTR_ERR(cod3034x->regmap));
		ret = -ENOMEM;
		goto err;
	}

	regcache_mark_dirty(cod3034x->regmap);

	pinctrl = devm_pinctrl_get(&i2c->dev);
	if (IS_ERR(pinctrl)) {
		dev_warn(&i2c->dev, "did not get pins for codec: %li\n",
				PTR_ERR(pinctrl));
	} else {
		cod3034x->pinctrl = pinctrl;
		dev_err(&i2c->dev, "cod3034x_i2c_probe pinctrl\n");
	}

	i2c_set_clientdata(i2c, cod3034x);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_cod3034x,
			cod3034x_dai, ARRAY_SIZE(cod3034x_dai));
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}
#ifdef CONFIG_PM
	pm_runtime_enable(cod3034x->dev);
#endif

	return ret;

err:
	kfree(cod3034x);
	return ret;
}

static int cod3034x_i2c_remove(struct i2c_client *i2c)
{
	struct cod3034x_priv *cod3034x = dev_get_drvdata(&i2c->dev);

	snd_soc_unregister_codec(&i2c->dev);
	kfree(cod3034x);
	return 0;
}

static int cod3034x_enable(struct device *dev)
{
	struct cod3034x_priv *cod3034x = dev_get_drvdata(dev);

	dev_dbg(dev, "(*) %s\n", __func__);
	cod3034x_regulators_enable(cod3034x->codec);

	/* Disable cache_only feature and sync the cache with h/w */
	cod3034x_reg_restore(cod3034x->codec);
	cod3034x_i2s_set_fmt(cod3034x->codec);

	return 0;
}

static int cod3034x_disable(struct device *dev)
{
	struct cod3034x_priv *cod3034x = dev_get_drvdata(dev);

	dev_dbg(dev, "(*) %s\n", __func__);
	cod3034x_regulators_disable(cod3034x->codec);

	return 0;
}

static int cod3034x_sys_suspend(struct device *dev)
{
#ifndef CONFIG_PM
	cod3034x_disable(dev);
#endif

	return 0;
}

static int cod3034x_sys_resume(struct device *dev)
{
#ifndef CONFIG_PM
	struct cod3034x_priv *cod3034x = dev_get_drvdata(dev);

	if (!cod3034x->is_suspend) {
		dev_dbg(dev, "(*)Codec-3034 not resuming, cp functioning\n");
		return 0;
	}
	dev_dbg(dev, "(*) %s\n", __func__);
	cod3034x_enable(dev);
#endif

	return 0;
}

#ifdef CONFIG_PM
static int cod3034x_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "(*) %s\n", __func__);
	cod3034x_enable(dev);

	return 0;
}

static int cod3034x_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "(*) %s\n", __func__);
	cod3034x_disable(dev);

	return 0;
}
#endif

static const struct dev_pm_ops cod3034x_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(
			cod3034x_sys_suspend,
			cod3034x_sys_resume
			)
#ifdef CONFIG_PM
		SET_RUNTIME_PM_OPS(
				cod3034x_runtime_suspend,
				cod3034x_runtime_resume,
				NULL
				)
#endif
};

static const struct i2c_device_id cod3034x_i2c_id[] = {
	{ "cod3034x", 3034 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cod3034x_i2c_id);

const struct of_device_id cod3034x_of_match[] = {
	{ .compatible = "codec,cod3034x", },
	{},
};

static struct i2c_driver cod3034x_i2c_driver = {
	.driver = {
		.name = "cod3034x",
		.owner = THIS_MODULE,
		.pm = &cod3034x_pm,
		.of_match_table = of_match_ptr(cod3034x_of_match),
	},
	.probe = cod3034x_i2c_probe,
	.remove = cod3034x_i2c_remove,
	.id_table = cod3034x_i2c_id,
};

module_i2c_driver(cod3034x_i2c_driver);

MODULE_DESCRIPTION("ASoC COD3034X driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:COD3034X-codec");
