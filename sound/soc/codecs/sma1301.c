/* sma1301.c -- sma1301 ALSA SoC Audio driver
 *
 * r012, 2019.07.16	- initial version  sma1301
 *
 * Copyright 2018 Silicon Mitus Corporation / Iron Device Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <asm/div64.h>

#include "sma1301.h"

#define CHECK_PERIOD_TIME 1 /* sec per HZ */

#define PLL_MATCH(_input_clk_name, _output_clk_name, _input_clk,\
		_post_n, _n, _f1, _f2, _f3_p_cp)\
{\
	.input_clk_name		= _input_clk_name,\
	.output_clk_name	= _output_clk_name,\
	.input_clk		= _input_clk,\
	.post_n			= _post_n,\
	.n			= _n,\
	.f1			= _f1,\
	.f2			= _f2,\
	.f3_p_cp		= _f3_p_cp,\
}

enum sma1301_type {
	SMA1301,
};

/* PLL clock setting Table */
struct sma1301_pll_match {
	char *input_clk_name;
	char *output_clk_name;
	unsigned int input_clk;
	unsigned int post_n;
	unsigned int n;
	unsigned int f1;
	unsigned int f2;
	unsigned int f3_p_cp;
};

struct sma1301_priv {
	enum sma1301_type devtype;
	struct attribute_group *attr_grp;
	struct kobject *kobj;
	struct regmap *regmap;
	struct sma1301_pll_match *pll_matches;
	int num_of_pll_matches;
	unsigned int mclk_in;
	unsigned int sys_clk_id;
	unsigned int init_vol;
	unsigned int cur_vol;
	unsigned int bst_vol_lvl_status;
	unsigned int flt_vdd_gain_status;
	bool amp_power_status;
	bool force_amp_power_down;
	bool stereo_two_chip;
	bool impossible_bst_ctrl;
	bool src_bypass;
	bool ot1_clear_flag;
	const uint32_t *eq_reg_array;
	uint32_t eq_reg_array_len;
	struct mutex lock;
	struct delayed_work check_fault_work;
	long check_fault_period;
	long check_fault_status;
	unsigned int format;
	struct device *dev;
	unsigned int rev_num;
	unsigned int last_over_temp;
	unsigned int last_ocp_val;
};

static struct sma1301_pll_match sma1301_pll_matches[] = {
/* in_clk_name, out_clk_name, input_clk post_n, n, f1, f2, f3_p_cp */
PLL_MATCH("1.536MHz",  "24.576MHz", 1536000,  0x07, 0xE0, 0x00, 0x00, 0x03),
PLL_MATCH("3.072MHz",  "24.576MHz", 3072000,  0x07, 0x70, 0x00, 0x00, 0x03),
PLL_MATCH("6.144MHz",  "24.576MHz", 6144000,  0x07, 0x70, 0x00, 0x00, 0x07),
PLL_MATCH("12.288MHz", "24.576MHz", 12288000, 0x07, 0x70, 0x00, 0x00, 0x0B),
PLL_MATCH("19.2MHz",   "24.343MHz", 19200000, 0x07, 0x47, 0x00, 0x00, 0x0A),
PLL_MATCH("24.576MHz", "24.576MHz", 24576000, 0x07, 0x70, 0x00, 0x00, 0x0F),
};

static struct sma1301_pll_match sma1301_pll_matches_shift[] = {
/* in_clk_name, out_clk_name, input_clk post_n, n, f1, f2, f3_p_cp */
PLL_MATCH("1.536MHz",  "24.576MHz", 1536000,  0x06, 0xC0, 0x00, 0x00, 0x03),
PLL_MATCH("3.072MHz",  "24.576MHz", 3072000,  0x06, 0x60, 0x00, 0x00, 0x03),
PLL_MATCH("6.144MHz",  "24.576MHz", 6144000,  0x06, 0x60, 0x00, 0x00, 0x07),
PLL_MATCH("12.288MHz", "24.576MHz", 12288000, 0x06, 0x60, 0x00, 0x00, 0x0B),
PLL_MATCH("19.2MHz",   "24.4MHz",   19200000, 0x06, 0x3D, 0x00, 0x00, 0x0A),
PLL_MATCH("24.576MHz", "24.576MHz", 24576000, 0x06, 0x60, 0x00, 0x00, 0x0F),
};
static int sma1301_startup(struct snd_soc_codec *);
static int sma1301_shutdown(struct snd_soc_codec *);

/* Initial register value - {register, value} 2018.01.17
 * EQ Band : 1 to 5 / 0x40 to 0x8A (15EA register for each EQ Band)
 * Currently all EQ Bands are flat frequency response
 */
static const struct reg_default sma1301_reg_def[] = {
	{ 0x00, 0x80 }, /* 0x00 SystemCTRL  */
	{ 0x01, 0x00 }, /* 0x01 InputCTRL1  */
	{ 0x02, 0x00 }, /* 0x02 InputCTRL2  */
	{ 0x03, 0x01 }, /* 0x03 InputCTRL3  */
	{ 0x04, 0x17 }, /* 0x04 InputCTRL4  */
	{ 0x09, 0x00 }, /* 0x09 OutputCTRL  */
	{ 0x0A, 0x58 }, /* 0x0A SPK_VOL  */
	{ 0x0E, 0xAF }, /* 0x0E MUTE_VOL_CTRL  */
	{ 0x10, 0x00 }, /* 0x10 SystemCTRL1  */
	{ 0x11, 0x00 }, /* 0x11 SystemCTRL2  */
	{ 0x12, 0x00 }, /* 0x12 SystemCTRL3  */
	{ 0x14, 0x60 }, /* 0x14 Modulator  */
	{ 0x15, 0x01 }, /* 0x15 BassSpk1  */
	{ 0x16, 0x0F }, /* 0x16 BassSpk2  */
	{ 0x17, 0x0F }, /* 0x17 BassSpk3  */
	{ 0x18, 0x0F }, /* 0x18 BassSpk4  */
	{ 0x19, 0x00 }, /* 0x19 BassSpk5  */
	{ 0x1A, 0x00 }, /* 0x1A BassSpk6  */
	{ 0x1B, 0x00 }, /* 0x1B BassSpk7  */
	{ 0x23, 0x19 }, /* 0x23 CompLim1  */
	{ 0x24, 0x00 }, /* 0x24 CompLim2  */
	{ 0x25, 0x00 }, /* 0x25 CompLim3  */
	{ 0x26, 0x04 }, /* 0x26 CompLim4  */
	{ 0x2B, 0x00 }, /* 0x2B EqMode  */
	{ 0x2C, 0x0C }, /* 0x2C EqGraphic1  */
	{ 0x2D, 0x0C }, /* 0x2D EqGraphic2  */
	{ 0x2E, 0x0C }, /* 0x2E EqGraphic3  */
	{ 0x2F, 0x0C }, /* 0x2F EqGraphic4  */
	{ 0x30, 0x0C }, /* 0x30 EqGraphic5  */
	{ 0x33, 0x00 }, /* 0x33 SDM_CTRL  */
	{ 0x36, 0x92 }, /* 0x36 Protection  */
	{ 0x37, 0x3F }, /* 0x37 SlopeCTRL  */
	{ 0x3B, 0x00 }, /* 0x3B Test1  */
	{ 0x3C, 0x00 }, /* 0x3C Test2  */
	{ 0x3D, 0x00 }, /* 0x3D Test3  */
	{ 0x3E, 0x03 }, /* 0x3E ATest1  */
	{ 0x3F, 0x00 }, /* 0x3F ATest2  */
	{ 0x40, 0x00 }, /* 0x40 EQCTRL1 */
	{ 0x41, 0x00 }, /* 0x41 EQCTRL2  */
	{ 0x42, 0x00 }, /* 0x42 EQCTRL3  */
	{ 0x43, 0x00 }, /* 0x43 EQCTRL4  */
	{ 0x44, 0x00 }, /* 0x44 EQCTRL5  */
	{ 0x45, 0x00 }, /* 0x45 EQCTRL6  */
	{ 0x46, 0x20 }, /* 0x46 EQCTRL7  */
	{ 0x47, 0x00 }, /* 0x47 EQCTRL8  */
	{ 0x48, 0x00 }, /* 0x48 EQCTRL9  */
	{ 0x49, 0x00 }, /* 0x49 EQCTRL10  */
	{ 0x4A, 0x00 }, /* 0x4A EQCTRL11  */
	{ 0x4B, 0x00 }, /* 0x4B EQCTRL12  */
	{ 0x4C, 0x00 }, /* 0x4C EQCTRL13  */
	{ 0x4D, 0x00 }, /* 0x4D EQCTRL14  */
	{ 0x4E, 0x00 }, /* 0x4E EQCTRL15  */
	{ 0x4F, 0x00 }, /* 0x4F EQCTRL16 : EQ BAND2 */
	{ 0x50, 0x00 }, /* 0x50 EQCTRL17  */
	{ 0x51, 0x00 }, /* 0x51 EQCTRL18  */
	{ 0x52, 0x00 }, /* 0x52 EQCTRL19  */
	{ 0x53, 0x00 }, /* 0x53 EQCTRL20  */
	{ 0x54, 0x00 }, /* 0x54 EQCTRL21  */
	{ 0x55, 0x20 }, /* 0x55 EQCTRL22  */
	{ 0x56, 0x00 }, /* 0x56 EQCTRL23  */
	{ 0x57, 0x00 }, /* 0x57 EQCTRL24  */
	{ 0x58, 0x00 }, /* 0x58 EQCTRL25  */
	{ 0x59, 0x00 }, /* 0x59 EQCTRL26  */
	{ 0x5A, 0x00 }, /* 0x5A EQCTRL27  */
	{ 0x5B, 0x00 }, /* 0x5B EQCTRL28  */
	{ 0x5C, 0x00 }, /* 0x5C EQCTRL29  */
	{ 0x5D, 0x00 }, /* 0x5D EQCTRL30  */
	{ 0x5E, 0x00 }, /* 0x5E EQCTRL31 : EQ BAND3 */
	{ 0x5F, 0x00 }, /* 0x5F EQCTRL32  */
	{ 0x60, 0x00 }, /* 0x60 EQCTRL33  */
	{ 0x61, 0x00 }, /* 0x61 EQCTRL34  */
	{ 0x62, 0x00 }, /* 0x62 EQCTRL35  */
	{ 0x63, 0x00 }, /* 0x63 EQCTRL36  */
	{ 0x64, 0x20 }, /* 0x64 EQCTRL37  */
	{ 0x65, 0x00 }, /* 0x65 EQCTRL38  */
	{ 0x66, 0x00 }, /* 0x66 EQCTRL39  */
	{ 0x67, 0x00 }, /* 0x67 EQCTRL40  */
	{ 0x68, 0x00 }, /* 0x68 EQCTRL41  */
	{ 0x69, 0x00 }, /* 0x69 EQCTRL42  */
	{ 0x6A, 0x00 }, /* 0x6A EQCTRL43  */
	{ 0x6B, 0x00 }, /* 0x6B EQCTRL44  */
	{ 0x6C, 0x00 }, /* 0x6C EQCTRL45  */
	{ 0x6D, 0x00 }, /* 0x6D EQCTRL46 : EQ BAND4 */
	{ 0x6E, 0x00 }, /* 0x6E EQCTRL47  */
	{ 0x6F, 0x00 }, /* 0x6F EQCTRL48  */
	{ 0x70, 0x00 }, /* 0x70 EQCTRL49  */
	{ 0x71, 0x00 }, /* 0x71 EQCTRL50  */
	{ 0x72, 0x00 }, /* 0x72 EQCTRL51  */
	{ 0x73, 0x20 }, /* 0x73 EQCTRL52  */
	{ 0x74, 0x00 }, /* 0x74 EQCTRL53  */
	{ 0x75, 0x00 }, /* 0x75 EQCTRL54  */
	{ 0x76, 0x00 }, /* 0x76 EQCTRL55  */
	{ 0x77, 0x00 }, /* 0x77 EQCTRL56  */
	{ 0x78, 0x00 }, /* 0x78 EQCTRL57  */
	{ 0x79, 0x00 }, /* 0x79 EQCTRL58  */
	{ 0x7A, 0x00 }, /* 0x7A EQCTRL59  */
	{ 0x7B, 0x00 }, /* 0x7B EQCTRL60  */
	{ 0x7C, 0x00 }, /* 0x7C EQCTRL61 : EQ BAND5 */
	{ 0x7D, 0x00 }, /* 0x7D EQCTRL62  */
	{ 0x7E, 0x00 }, /* 0x7E EQCTRL63  */
	{ 0x7F, 0x00 }, /* 0x7F EQCTRL64  */
	{ 0x80, 0x00 }, /* 0x80 EQCTRL65  */
	{ 0x81, 0x00 }, /* 0x81 EQCTRL66  */
	{ 0x82, 0x20 }, /* 0x82 EQCTRL67  */
	{ 0x83, 0x00 }, /* 0x83 EQCTRL68  */
	{ 0x84, 0x00 }, /* 0x84 EQCTRL69  */
	{ 0x85, 0x00 }, /* 0x85 EQCTRL70  */
	{ 0x86, 0x00 }, /* 0x86 EQCTRL71  */
	{ 0x87, 0x00 }, /* 0x87 EQCTRL72  */
	{ 0x88, 0x00 }, /* 0x88 EQCTRL73  */
	{ 0x89, 0x00 }, /* 0x89 EQCTRL74  */
	{ 0x8A, 0x00 }, /* 0x8A EQCTRL75  */
	{ 0x8B, 0x08 }, /* 0x8B PLL_POST_N  */
	{ 0x8C, 0x20 }, /* 0x8C PLL_N  */
	{ 0x8D, 0x00 }, /* 0x8D PLL_F1  */
	{ 0x8E, 0x00 }, /* 0x8E PLL_F2  */
	{ 0x8F, 0x02 }, /* 0x8F PLL_F3,P,CP  */
	{ 0x91, 0x42 }, /* 0x91 ClassG Control  */
	{ 0x92, 0x80 }, /* 0x92 FDPEC Control  */
	{ 0x94, 0x33 }, /* 0x94 Boost Control1  */
	{ 0x95, 0x39 }, /* 0x95 Boost Control2  */
	{ 0x96, 0x42 }, /* 0x96 Boost Control3  */
	{ 0x97, 0x8A }, /* 0x97 Boost Control4  */
	{ 0xA2, 0x68 }, /* 0xA2 TOP_MAN1  */
	{ 0xA3, 0x28 }, /* 0xA3 TOP_MAN2  */
	{ 0xA4, 0x40 }, /* 0xA4 TOP_MAN3  */
	{ 0xFA, 0xC0 }, /* 0xFA Status1  */
	{ 0xFB, 0x06 }, /* 0xFB Status2  */
	{ 0xFD, 0x00 }, /* 0xFD Status3  */
	{ 0xFF, 0x0A }, /* 0xFF Device Index  */
};

static bool sma1301_readable_register(struct device *dev, unsigned int reg)
{
	if (reg > SMA1301_FF_DEVICE_INDEX)
		return false;

	switch (reg) {
	case SMA1301_00_SYSTEM_CTRL ... SMA1301_04_INPUT1_CTRL4:
	case SMA1301_09_OUTPUT_CTRL ... SMA1301_0A_SPK_VOL:
	case SMA1301_0E_MUTE_VOL_CTRL:
	case SMA1301_10_SYSTEM_CTRL1 ... SMA1301_12_SYSTEM_CTRL3:
	case SMA1301_14_MODULATOR ... SMA1301_1B_BASS_SPK7:
	case SMA1301_23_COMP_LIM1 ... SMA1301_26_COMP_LIM4:
	case SMA1301_2B_EQ_MODE ... SMA1301_30_EQ_GRAPHIC5:
	case SMA1301_33_SDM_CTRL:
	case SMA1301_36_PROTECTION ... SMA1301_37_SLOPE_CTRL:
	case SMA1301_3B_TEST1 ... SMA1301_8F_PLL_F3:
	case SMA1301_91_CLASS_G_CTRL ... SMA1301_92_FDPEC_CTRL:
	case SMA1301_94_BOOST_CTRL1 ... SMA1301_97_BOOST_CTRL4:
	case SMA1301_A2_TOP_MAN1 ... SMA1301_A4_TOP_MAN3:
	case SMA1301_FA_STATUS1 ... SMA1301_FB_STATUS2:
	case SMA1301_FD_STATUS3:
	case SMA1301_FF_DEVICE_INDEX:
		return true;
	default:
		return false;
	}
}

static bool sma1301_writeable_register(struct device *dev, unsigned int reg)
{
	if (reg > SMA1301_FF_DEVICE_INDEX)
		return false;

	switch (reg) {
	case SMA1301_00_SYSTEM_CTRL ... SMA1301_04_INPUT1_CTRL4:
	case SMA1301_09_OUTPUT_CTRL ... SMA1301_0A_SPK_VOL:
	case SMA1301_0E_MUTE_VOL_CTRL:
	case SMA1301_10_SYSTEM_CTRL1 ... SMA1301_12_SYSTEM_CTRL3:
	case SMA1301_14_MODULATOR ... SMA1301_1B_BASS_SPK7:
	case SMA1301_23_COMP_LIM1 ... SMA1301_26_COMP_LIM4:
	case SMA1301_2B_EQ_MODE ... SMA1301_30_EQ_GRAPHIC5:
	case SMA1301_33_SDM_CTRL:
	case SMA1301_36_PROTECTION ... SMA1301_37_SLOPE_CTRL:
	case SMA1301_3B_TEST1 ... SMA1301_8F_PLL_F3:
	case SMA1301_91_CLASS_G_CTRL ... SMA1301_92_FDPEC_CTRL:
	case SMA1301_94_BOOST_CTRL1 ... SMA1301_97_BOOST_CTRL4:
	case SMA1301_A2_TOP_MAN1 ... SMA1301_A4_TOP_MAN3:
		return true;
	default:
		return false;
	}
}

static bool sma1301_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SMA1301_FA_STATUS1 ... SMA1301_FB_STATUS2:
	case SMA1301_FD_STATUS3:
	case SMA1301_FF_DEVICE_INDEX:
		return true;
	default:
		return false;
	}
}

/* DB scale conversion of speaker volume */
static const DECLARE_TLV_DB_SCALE(sma1301_spk_tlv, -6000, 50, 0);

/* common bytes ext functions */
static int bytes_ext_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int reg)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	unsigned int i, reg_val;
	u8 *val;

	val = (u8 *)ucontrol->value.bytes.data;
	for (i = 0; i < params->max; i++) {
		regmap_read(sma1301->regmap, reg + i, &reg_val);
		if (sizeof(reg_val) > 2)
			reg_val = cpu_to_le32(reg_val);
		else
			reg_val = cpu_to_le16(reg_val);
		memcpy(val + i, &reg_val, sizeof(u8));
	}

	return 0;
}

static int bytes_ext_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol, int reg)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	struct soc_bytes_ext *params = (void *)kcontrol->private_value;
	void *data;
	u8 *val;
	int i, ret;

	data = kmemdup(ucontrol->value.bytes.data,
			params->max, GFP_KERNEL | GFP_DMA);
	if (!data)
		return -ENOMEM;

	val = (u8 *)data;
	for (i = 0; i < params->max; i++) {
		ret = regmap_write(sma1301->regmap, reg + i, *(val + i));
		if (ret) {
			dev_err(codec->dev,
				"configuration fail, register: %x ret: %d\n",
				reg + i, ret);
			kfree(data);
			return ret;
		}
	}
	kfree(data);

	return 0;
}

static int power_up_down_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = sma1301->amp_power_status;

	return 0;
}

static int power_up_down_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if (sel && !(sma1301->force_amp_power_down))
		sma1301_startup(codec);
	else
		sma1301_shutdown(codec);

	return 0;
}

static int power_down_control_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = sma1301->force_amp_power_down;

	return 0;
}

static int power_down_control_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	sma1301->force_amp_power_down = ucontrol->value.integer.value[0];

	if (sma1301->force_amp_power_down) {
		dev_info(codec->dev, "%s\n", "Force AMP Power Down");
		sma1301_shutdown(codec);
	}

	return 0;
}

/* Clock System Set */
static const char * const sma1301_clk_system_text[] = {
	"Reserved", "Reserved", "Reserved", "Ext_19.2MHz",
	"Ext_24.576MHz", "Reserved", "Reserved", "Reserved"};

static const struct soc_enum sma1301_clk_system_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_clk_system_text),
		sma1301_clk_system_text);

static int sma1301_clk_system_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_00_SYSTEM_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0xE0) >> 5);

	return 0;
}

static int sma1301_clk_system_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap, SMA1301_00_SYSTEM_CTRL,
			0xE0, (sel << 5));

	return 0;
}

/* InputCTRL1 Set */
static const char * const sma1301_input_format_text[] = {
	"I2S", "LJ", "Reserved", "Reserved",
	"RJ_16", "RJ_18", "RJ_20", "RJ_24"};

static const struct soc_enum sma1301_input_format_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_input_format_text),
		sma1301_input_format_text);

static int sma1301_input_format_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_01_INPUT1_CTRL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0x70) >> 4);

	return 0;
}

static int sma1301_input_format_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_01_INPUT1_CTRL1, 0x70, (sel << 4));

	return 0;
}

/* InputCTRL2 Set */
static const char * const sma1301_in_audio_mode_text[] = {
	"I2S mode", "PCM short", "PCM long", "Reserved"};

static const struct soc_enum sma1301_in_audio_mode_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_in_audio_mode_text),
		sma1301_in_audio_mode_text);

static int sma1301_in_audio_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_02_INPUT1_CTRL2, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma1301_in_audio_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_02_INPUT1_CTRL2, 0xC0, (sel << 6));

	return 0;
}

/* InputCTRL3 Set */
static const char * const sma1301_pcm_n_slot_text[] = {
	"Slot_1", "Slot_2", "Slot_3", "Slot_4", "Slot_5", "Slot_6",
	"Slot_7", "Slot_8", "Slot_9", "Slot_10", "Slot_11", "Slot_12",
	"Slot_13", "Slot_14", "Slot_15", "Slot_16"};

static const struct soc_enum sma1301_pcm_n_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_pcm_n_slot_text),
		sma1301_pcm_n_slot_text);

static int sma1301_pcm_n_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_03_INPUT1_CTRL3, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma1301_pcm_n_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_03_INPUT1_CTRL3, 0x0F, sel);

	return 0;
}

/* InputCTRL4 Set */
static const char * const sma1301_pcm1_slot_text[] = {
	"Slot_1", "Slot_2", "Slot_3", "Slot_4", "Slot_5", "Slot_6",
	"Slot_7", "Slot_8", "Slot_9", "Slot_10", "Slot_11", "Slot_12",
	"Slot_13", "Slot_14", "Slot_15", "Slot_16"};

static const struct soc_enum sma1301_pcm1_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_pcm1_slot_text),
		sma1301_pcm1_slot_text);

static int sma1301_pcm1_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_04_INPUT1_CTRL4, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma1301_pcm1_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_04_INPUT1_CTRL4, 0xF0, (sel << 4));

	return 0;
}

/* InputCTRL4 Set */
static const char * const sma1301_pcm2_slot_text[] = {
	"Slot_1", "Slot_2", "Slot_3", "Slot_4", "Slot_5", "Slot_6",
	"Slot_7", "Slot_8", "Slot_9", "Slot_10", "Slot_11", "Slot_12",
	"Slot_13", "Slot_14", "Slot_15", "Slot_16"};

static const struct soc_enum sma1301_pcm2_slot_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_pcm2_slot_text),
		sma1301_pcm2_slot_text);

static int sma1301_pcm2_slot_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_04_INPUT1_CTRL4, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma1301_pcm2_slot_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_04_INPUT1_CTRL4, 0x0F, sel);

	return 0;
}

/* Input & output port config */
static const char * const sma1301_port_config_text[] = {
	"IN_Port", "Reserved", "OUT_Port", "Reserved"};

static const struct soc_enum sma1301_port_config_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_port_config_text),
			sma1301_port_config_text);

static int sma1301_port_config_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_09_OUTPUT_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0x60) >> 5);

	return 0;
}

static int sma1301_port_config_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_09_OUTPUT_CTRL, 0x60, (sel << 5));

	return 0;
}

/* Output format select */
static const char * const sma1301_port_out_format_text[] = {
	"I2S_32", "I2S_64", "PCM_Short_128fs", "Reserved"};

static const struct soc_enum sma1301_port_out_format_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_port_out_format_text),
	sma1301_port_out_format_text);

static int sma1301_port_out_format_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_09_OUTPUT_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0x18) >> 3);

	return 0;
}

static int sma1301_port_out_format_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_09_OUTPUT_CTRL, 0x18, (sel << 3));

	return 0;
}

/* Output source */
static const char * const sma1301_port_out_sel_text[] = {
	"Disable", "Format_C", "Mixer_out", "After_DSP",
	"Reserved", "Reserved", "Reserved", "Reserved"};

static const struct soc_enum sma1301_port_out_sel_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_port_out_sel_text),
	sma1301_port_out_sel_text);

static int sma1301_port_out_sel_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_09_OUTPUT_CTRL, &val);
	ucontrol->value.integer.value[0] = (val & 0x07);

	return 0;
}

static int sma1301_port_out_sel_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_09_OUTPUT_CTRL, 0x07, sel);

	return 0;
}

/* Volume slope */
static const char * const sma1301_vol_slope_text[] = {
	"Off", "Slow(about 1sec)", "Medium(about 0.5sec)",
	"Fast(about 0.1sec)"};

static const struct soc_enum sma1301_vol_slope_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_vol_slope_text),
	sma1301_vol_slope_text);

static int sma1301_vol_slope_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_0E_MUTE_VOL_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma1301_vol_slope_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_0E_MUTE_VOL_CTRL, 0xC0, (sel << 6));

	return 0;
}

/* Mute slope */
static const char * const sma1301_mute_slope_text[] = {
	"Off", "Slow(about 200ms)", "Medium(about 50ms)",
	"Fast(about 10ms)"};

static const struct soc_enum sma1301_mute_slope_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_mute_slope_text),
	sma1301_mute_slope_text);

static int sma1301_mute_slope_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_0E_MUTE_VOL_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0x30) >> 4);

	return 0;
}

static int sma1301_mute_slope_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_0E_MUTE_VOL_CTRL, 0x30, (sel << 4));

	return 0;
}

/* Speaker mode */
static const char * const sma1301_spkmode_text[] = {
	"Off", "Mono", "Reserved", "Reserved",
	"Stereo", "Reserved", "Reserved", "Reserved"};

static const struct soc_enum sma1301_spkmode_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_spkmode_text),
	sma1301_spkmode_text);

static int sma1301_spkmode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_10_SYSTEM_CTRL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0x1C) >> 2);

	return 0;
}

static int sma1301_spkmode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_10_SYSTEM_CTRL1, 0x1C, (sel << 2));

	if (sel == (SPK_MONO >> 2))
		sma1301->stereo_two_chip = false;
	else if (sel == (SPK_STEREO >> 2))
		sma1301->stereo_two_chip = true;

	return 0;
}

/* SystemCTRL3 Set */
static const char * const sma1301_input_gain_text[] = {
	"Gain_0dB", "Gain_M6dB", "Gain_M12dB", "Gain_MInf"};

static const struct soc_enum sma1301_input_gain_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_input_gain_text),
		sma1301_input_gain_text);

static int sma1301_input_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_12_SYSTEM_CTRL3, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma1301_input_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_12_SYSTEM_CTRL3, 0xC0, (sel << 6));

	return 0;
}

static const char * const sma1301_input_r_gain_text[] = {
	"Gain_0dB", "Gain_M6dB", "Gain_M12dB", "Gain_MInf"};

static const struct soc_enum sma1301_input_r_gain_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_input_r_gain_text),
		sma1301_input_r_gain_text);

static int sma1301_input_r_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_12_SYSTEM_CTRL3, &val);
	ucontrol->value.integer.value[0] = ((val & 0x30) >> 4);

	return 0;
}

static int sma1301_input_r_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_12_SYSTEM_CTRL3, 0x30, (sel << 4));

	return 0;
}

/* Modulator Set */
static const char * const sma1301_spk_hysfb_text[] = {
	"f_625kHz", "f_414kHz", "f_297kHz", "f_226kHz"};

static const struct soc_enum sma1301_spk_hysfb_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_spk_hysfb_text), sma1301_spk_hysfb_text);

static int sma1301_spk_hysfb_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_14_MODULATOR, &val);
	ucontrol->value.integer.value[0] = ((val & 0xC0) >> 6);

	return 0;
}

static int sma1301_spk_hysfb_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_14_MODULATOR, 0xC0, (sel << 6));

	return 0;
}

static int spk_bdelay_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_14_MODULATOR);
}

static int spk_bdelay_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_14_MODULATOR);
}

/* bass boost speaker coeff */
static int bass_spk_coeff_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_15_BASS_SPK1);
}

static int bass_spk_coeff_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_15_BASS_SPK1);
}

/* DRC speaker coeff */
static int comp_lim_spk_coeff_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_23_COMP_LIM1);
}

static int comp_lim_spk_coeff_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_23_COMP_LIM1);
}

/* EQ Mode SELECT */
static const char * const sma1301_eq_mode_text[] = {
	"User Defined", "Classic ", "Rock_Pop", "Jazz",
	"RnB", "Dance", "Speech", "Parametric"};

static const struct soc_enum sma1301_eq_mode_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_eq_mode_text), sma1301_eq_mode_text);

static int sma1301_eq_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_2B_EQ_MODE, &val);
	ucontrol->value.integer.value[0] = (val & 0x07);

	return 0;
}

static int sma1301_eq_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_2B_EQ_MODE, 0x07, sel);

	return 0;
}

/* EqGraphic 1~5 */
static int eqgraphic_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_2C_EQ_GRAPHIC1);
}

static int eqgraphic_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_2C_EQ_GRAPHIC1);
}

/* PWM LR Delay Set */
static const char * const sma1301_lr_delay_text[] = {
	"Delay_00", "Delay_01", "Delay_10", "Delay_11"};

static const struct soc_enum sma1301_lr_delay_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_lr_delay_text), sma1301_lr_delay_text);

static int sma1301_lr_delay_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_36_PROTECTION, &val);
	ucontrol->value.integer.value[0] = ((val & 0x60) >> 5);

	return 0;
}

static int sma1301_lr_delay_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_36_PROTECTION, 0x60, (sel << 5));

	return 0;
}

/* OTP MODE Set */
static const char * const sma1301_otp_mode_text[] = {
	"Disable", "I_L1_S_L2", "R_L1_S_L2", "S_L1_S_L2"};

static const struct soc_enum sma1301_otp_mode_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_otp_mode_text), sma1301_otp_mode_text);

static int sma1301_otp_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_36_PROTECTION, &val);
	ucontrol->value.integer.value[0] = (val & 0x03);

	return 0;
}

static int sma1301_otp_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap, SMA1301_36_PROTECTION, 0x03, sel);

	return 0;
}

/* Slope CTRL */
static int slope_ctrl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_37_SLOPE_CTRL);
}

static int slope_ctrl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_37_SLOPE_CTRL);
}

/* Test 1~3, ATEST 1~2 */
static int test_mode_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_3B_TEST1);
}

static int test_mode_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_3B_TEST1);
}

/* PEQ Band1 */
static int eq_ctrl_band1_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_40_EQ_CTRL1);
}

static int eq_ctrl_band1_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_40_EQ_CTRL1);
}

/* PEQ Band2 */
static int eq_ctrl_band2_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_4F_EQ_CTRL16);
}

static int eq_ctrl_band2_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_4F_EQ_CTRL16);
}

/* PEQ Band3 */
static int eq_ctrl_band3_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_5E_EQ_CTRL31);
}

static int eq_ctrl_band3_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_5E_EQ_CTRL31);
}

/* PEQ Band4 */
static int eq_ctrl_band4_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_6D_EQ_CTRL46);
}

static int eq_ctrl_band4_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_6D_EQ_CTRL46);
}

/* PEQ Band5 */
static int eq_ctrl_band5_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_7C_EQ_CTRL61);
}

static int eq_ctrl_band5_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_7C_EQ_CTRL61);
}

/* PLL setting */
static int pll_setting_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_get(kcontrol, ucontrol, SMA1301_8B_PLL_POST_N);
}

static int pll_setting_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	return bytes_ext_put(kcontrol, ucontrol, SMA1301_8B_PLL_POST_N);
}

/* ClassG control Set */
static const char * const sma1301_attack_lvl_text[] = {
	"BST_ON", "LVL_0.0625FS", "LVL_0.125FS", "LVL_0.1875FS",
	"LVL_0.25FS", "LVL_0.3125FS", "LVL_0.375FS", "LVL_0.4375FS",
	"LVL_0.5FS", "LVL_0.5625FS", "LVL_0.625FS", "LVL_0.6875FS",
	"LVL_0.75FS", "LVL_0.8125FS", "LVL_0.875FS", "BST_OFF"};

static const struct soc_enum sma1301_attack_lvl_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_attack_lvl_text),
		sma1301_attack_lvl_text);

static int sma1301_attack_lvl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_91_CLASS_G_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static const char * const sma1301_release_time_text[] = {
	"Time_0ms", "Time_20ms", "Time_40ms", "Time_60ms",
	"Time_80ms", "Time_100ms", "Time_120ms", "Time_140ms",
	"Time_160ms", "Time_180ms", "Time_200ms", "Time_220ms",
	"Time_240ms", "Time_260ms", "Time_280ms", "Time_300ms"};

static const struct soc_enum sma1301_release_time_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_release_time_text),
		sma1301_release_time_text);

static int sma1301_release_time_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_91_CLASS_G_CTRL, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma1301_release_time_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_91_CLASS_G_CTRL, 0x0F, sel);

	return 0;
}

static int sma1301_attack_lvl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_91_CLASS_G_CTRL, 0xF0, (sel << 4));

	return 0;
}

/* FDPEC control Set */
static const char * const sma1301_flt_vdd_gain_text[] = {
	"VDD_2.4V", "VDD_2.45V", "VDD_2.5V", "VDD_2.55V",
	"VDD_2.6V", "VDD_2.65V", "VDD_2.7V", "VDD_2.75V",
	"VDD_2.8V", "VDD_2.85V", "VDD_2.9V", "VDD_2.95V",
	"VDD_3.0V", "VDD_3.05V", "VDD_3.1V", "VDD_3.15V"};

static const struct soc_enum sma1301_flt_vdd_gain_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_flt_vdd_gain_text),
		sma1301_flt_vdd_gain_text);

static int sma1301_flt_vdd_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_92_FDPEC_CTRL, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma1301_flt_vdd_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	sma1301->flt_vdd_gain_status = sel;

	regmap_update_bits(sma1301->regmap,
		SMA1301_92_FDPEC_CTRL, 0xF0, (sel << 4));

	return 0;
}

/* Boost control1 Set */
static const char * const sma1301_trm_osc_text[] = {
	"f_1.4MHz", "f_1.6MHz", "f_1.8MHz", "f_2.0MHz",
	"f_2.2MHz", "f_2.4MHz", "f_2.6MHz", "f_2.8MHz"};

static const struct soc_enum sma1301_trm_osc_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_osc_text), sma1301_trm_osc_text);

static int sma1301_trm_osc_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_94_BOOST_CTRL1, &val);
	ucontrol->value.integer.value[0] = ((val & 0x70) >> 4);

	return 0;
}

static int sma1301_trm_osc_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_94_BOOST_CTRL1, 0x70, (sel << 4));

	return 0;
}

static const char * const sma1301_trm_rmp_text[] = {
	"RMP_0.49A/us", "RMP_0.98A/us", "RMP_1.47A/us", "RMP_1.96A/us",
	"RMP_2.45A/us", "RMP_2.94A/us", "RMP_3.43A/us", "RMP_3.92A/us"};

static const struct soc_enum sma1301_trm_rmp_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_rmp_text), sma1301_trm_rmp_text);

static int sma1301_trm_rmp_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_94_BOOST_CTRL1, &val);
	ucontrol->value.integer.value[0] = (val & 0x07);

	return 0;
}

static int sma1301_trm_rmp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_94_BOOST_CTRL1, 0x07, sel);

	return 0;
}

/* Boost control2 Set */
static const char * const sma1301_trm_ocl_text[] = {
	"I_1.93A", "I_2.08A", "I_2.21A", "I_2.37A",
	"I_2.51A", "I_2.67A", "I_2.81A", "I_2.97A"};

static const struct soc_enum sma1301_trm_ocl_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_ocl_text), sma1301_trm_ocl_text);

static int sma1301_trm_ocl_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_95_BOOST_CTRL2, &val);
	ucontrol->value.integer.value[0] = ((val & 0x70) >> 4);

	return 0;
}

static int sma1301_trm_ocl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_95_BOOST_CTRL2, 0x70, (sel << 4));

	return 0;
}

/* Boost control2 Set */
static const char * const sma1301_trm_comp_text[] = {
	"PI_50pF_1.5Mohm", "PI_50pF_1.0Mohm",
	"PI_50pF_0.75Mohm", "PI_50pF_0.25Mohm",
	"PI_80pF_1.5Mohm", "PI_80pF_1.0Mohm",
	"PI_80pF_0.75Mohm", "PI_80pF_0.25Mohm",
	"Type2_50pF_1.5Mohm", "Type2_50pF_1.0Mohm",
	"Type2_50pF_0.75Mohm", "Type2_50pF_0.25Mohm",
	"Type2_80pF_1.5Mohm", "Type2_80pF_1.0Mohm",
	"Type2_80pF_0.75Mohm", "Type2_80pF_0.25Mohm"};

static const struct soc_enum sma1301_trm_comp_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_comp_text), sma1301_trm_comp_text);

static int sma1301_trm_comp_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_95_BOOST_CTRL2, &val);
	ucontrol->value.integer.value[0] = (val & 0x0F);

	return 0;
}

static int sma1301_trm_comp_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if (sma1301->rev_num <= REV_NUM_ES0) {
		regmap_update_bits(sma1301->regmap,
			SMA1301_95_BOOST_CTRL2, 0x0F, sel);
	} else {
		dev_info(codec->dev,
		"Trimming of loop compensation does not change on ES1 and above\n");
	}

	return 0;
}

static const char * const sma1301_trm_comp_i_text[] = {
	"IG_15pF", "IG_30pF", "IG_65pF", "IG_80pF"};

static const struct soc_enum sma1301_trm_comp_i_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_comp_i_text),
			sma1301_trm_comp_i_text);

static int sma1301_trm_comp_i_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_95_BOOST_CTRL2, &val);
	ucontrol->value.integer.value[0] = ((val & 0x0C) >> 2);

	return 0;
}

static int sma1301_trm_comp_i_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if (sma1301->rev_num > REV_NUM_ES0) {
		regmap_update_bits(sma1301->regmap,
			SMA1301_95_BOOST_CTRL2, 0x0C, (sel << 2));
	} else {
		dev_info(codec->dev,
		"Trimming of loop compensation I gain does not change on ES0\n");
	}

	return 0;
}

static const char * const sma1301_trm_comp_p_text[] = {
	"PG_1.5Mohm", "PG_1.0Mohm", "PG_0.75Mohm", "PG_0.25Mohm"};

static const struct soc_enum sma1301_trm_comp_p_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_comp_p_text),
			sma1301_trm_comp_p_text);

static int sma1301_trm_comp_p_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_95_BOOST_CTRL2, &val);
	ucontrol->value.integer.value[0] = (val & 0x03);

	return 0;
}

static int sma1301_trm_comp_p_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if (sma1301->rev_num > REV_NUM_ES0) {
		regmap_update_bits(sma1301->regmap,
			SMA1301_95_BOOST_CTRL2, 0x03, sel);
	} else {
		dev_info(codec->dev,
		"Trimming of loop compensation P gain does not change on ES0\n");
	}

	return 0;
}

/* Boost control3 Set */
static const char * const sma1301_trm_dt_text[] = {
	"Time_11.65ns", "Time_9ns", "Time_6ns", "Time_5ns",
	"Time_2.4ns", "Time_2.25ns", "Time_2.1ns", "Time_2.0ns",
	"Time_0.85ns", "Time_0.85ns", "Time_0.85ns", "Time_0.85ns",
	"Time_0.75ns", "Time_0.75ns", "Time_0.75ns", "Time_0.75ns"};

static const struct soc_enum sma1301_trm_dt_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_dt_text), sma1301_trm_dt_text);

static int sma1301_trm_dt_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_96_BOOST_CTRL3, &val);
	ucontrol->value.integer.value[0] = ((val & 0xF0) >> 4);

	return 0;
}

static int sma1301_trm_dt_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_96_BOOST_CTRL3, 0xF0, (sel << 4));

	return 0;
}

static const char * const sma1301_trm_slw_text[] = {
	"Time_8.5ns", "Time_5.0ns", "Time_3.5ns", "Time_2.5ns"};

static const struct soc_enum sma1301_trm_slw_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_slw_text), sma1301_trm_slw_text);

static int sma1301_trm_slw_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_96_BOOST_CTRL3, &val);
	ucontrol->value.integer.value[0] = (val & 0x03);

	return 0;
}

static int sma1301_trm_slw_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_96_BOOST_CTRL3, 0x03, sel);

	return 0;
}

/* Boost control4 Set */
static const char * const sma1301_trm_vref_text[] = {
	"REF_1.24V", "REF_1.23V", "REF_1.22V", "REF_1.21V",
	"REF_1.20V", "REF_1.19V", "REF_1.18V", "REF_1.17V"};

static const struct soc_enum sma1301_trm_vref_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_vref_text), sma1301_trm_vref_text);

static int sma1301_trm_vref_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_97_BOOST_CTRL4, &val);
	ucontrol->value.integer.value[0] = ((val & 0xE0) >> 5);

	return 0;
}

static int sma1301_trm_vref_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_97_BOOST_CTRL4, 0xE0, (sel << 5));

	return 0;
}

/* Boost control4 Set */
static const char * const sma1301_trm_vbst_text[] = {
	"BST_5.5V", "BST_5.6V", "BST_5.7V", "BST_5.8V",
	"BST_5.9V", "BST_6.0V", "BST_6.1V", "BST_6.2V"};

static const struct soc_enum sma1301_trm_vbst_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_vbst_text), sma1301_trm_vbst_text);

static int sma1301_trm_vbst_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_97_BOOST_CTRL4, &val);
	ucontrol->value.integer.value[0] = ((val & 0x1C) >> 2);

	return 0;
}

static int sma1301_trm_vbst_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	if (sma1301->impossible_bst_ctrl)
		dev_info(codec->dev,
		"Trimming of boost voltage does not change on 'impossible-bst-ctrl' property\n");
	else {
		sma1301->bst_vol_lvl_status = sel;

		regmap_update_bits(sma1301->regmap,
			SMA1301_97_BOOST_CTRL4, 0x1C, (sel << 2));
	}

	return 0;
}

static const char * const sma1301_trm_tmin_text[] = {
	"Time_56ns", "Time_59ns", "Time_62ns", "Time_65ns"};

static const struct soc_enum sma1301_trm_tmin_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_trm_tmin_text), sma1301_trm_tmin_text);

static int sma1301_trm_tmin_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_97_BOOST_CTRL4, &val);
	ucontrol->value.integer.value[0] = (val & 0x03);

	return 0;
}

static int sma1301_trm_tmin_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_97_BOOST_CTRL4, 0x03, sel);

	return 0;
}

/* TOP_MAN3 Set */
static const char * const sma1301_o_format_text[] = {
	"RJ", "LJ", "I2S", "SPI",
	"PCM short", "PCM long", "Reserved", "Reserved"};

static const struct soc_enum sma1301_o_format_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_o_format_text), sma1301_o_format_text);

static int sma1301_o_format_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_A4_TOP_MAN3, &val);
	ucontrol->value.integer.value[0] = ((val & 0xE0) >> 5);

	return 0;
}

static int sma1301_o_format_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_A4_TOP_MAN3, 0xE0, (sel << 5));

	return 0;
}

static const char * const sma1301_sck_rate_text[] = {
	"fs_64", "fs_64", "fs_32", "fs_32"};

static const struct soc_enum sma1301_sck_rate_enum =
SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sma1301_sck_rate_text), sma1301_sck_rate_text);

static int sma1301_sck_rate_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int val;

	regmap_read(sma1301->regmap, SMA1301_A4_TOP_MAN3, &val);
	ucontrol->value.integer.value[0] = ((val & 0x18) >> 3);

	return 0;
}

static int sma1301_sck_rate_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int sel = (int)ucontrol->value.integer.value[0];

	regmap_update_bits(sma1301->regmap,
		SMA1301_A4_TOP_MAN3, 0x18, (sel << 3));

	return 0;
}

static const struct snd_kcontrol_new sma1301_snd_controls[] = {

SOC_SINGLE_EXT("Power Up(1:Up_0:Down)", SND_SOC_NOPM, 0, 1, 0,
	power_up_down_control_get, power_up_down_control_put),
SOC_SINGLE_EXT("Force AMP Power Down", SND_SOC_NOPM, 0, 1, 0,
	power_down_control_get, power_down_control_put),
SOC_ENUM_EXT("External Clock System", sma1301_clk_system_enum,
	sma1301_clk_system_get, sma1301_clk_system_put),

SOC_SINGLE("I2S/PCM Clock mode(1:M_2:S)",
		SMA1301_01_INPUT1_CTRL1, 7, 1, 0),
SOC_ENUM_EXT("I2S input format(I2S_LJ_RJ)", sma1301_input_format_enum,
	sma1301_input_format_get, sma1301_input_format_put),
SOC_SINGLE("First-channel pol I2S(1:H_0:L)",
		SMA1301_01_INPUT1_CTRL1, 3, 1, 0),
SOC_SINGLE("Data written SCK edge(1:R_0:F)",
		SMA1301_01_INPUT1_CTRL1, 2, 1, 0),

SOC_ENUM_EXT("Input audio mode", sma1301_in_audio_mode_enum,
	sma1301_in_audio_mode_get, sma1301_in_audio_mode_put),
SOC_SINGLE("Data inversion(1:R_0:L)",
		SMA1301_02_INPUT1_CTRL2, 5, 1, 0),
SOC_SINGLE("Decoding select(1:A_0:u)",
		SMA1301_02_INPUT1_CTRL2, 4, 1, 0),
SOC_SINGLE("Companding PCM data(1:C_2:L)",
		SMA1301_02_INPUT1_CTRL2, 3, 1, 0),
SOC_SINGLE("PCM sample freq(1:16kHz_0:8kHz)",
		SMA1301_02_INPUT1_CTRL2, 2, 1, 0),
SOC_SINGLE("PCM stero/mono sel(1:S_0:M)",
		SMA1301_02_INPUT1_CTRL2, 1, 1, 0),
SOC_SINGLE("PCM data length(1:16_0:8)",
		SMA1301_02_INPUT1_CTRL2, 0, 1, 0),

SOC_SINGLE("SR converted bypass(1:bypass_0:normal)",
		SMA1301_03_INPUT1_CTRL3, 4, 1, 0),
SOC_ENUM_EXT("Number of slots per sampling period(PCM)",
		sma1301_pcm_n_slot_enum, sma1301_pcm_n_slot_get,
		sma1301_pcm_n_slot_put),

SOC_ENUM_EXT("Position of the first sample at 8,16kHz",
		sma1301_pcm1_slot_enum, sma1301_pcm1_slot_get,
		sma1301_pcm1_slot_put),
SOC_ENUM_EXT("Position of the second sample at 16kHz",
		sma1301_pcm2_slot_enum, sma1301_pcm2_slot_get,
		sma1301_pcm2_slot_put),

SOC_ENUM_EXT("Port In/Out port config", sma1301_port_config_enum,
	sma1301_port_config_get, sma1301_port_config_put),
SOC_ENUM_EXT("Port Output Format", sma1301_port_out_format_enum,
	sma1301_port_out_format_get, sma1301_port_out_format_put),
SOC_ENUM_EXT("Port Output Source", sma1301_port_out_sel_enum,
	sma1301_port_out_sel_get, sma1301_port_out_sel_put),

SOC_SINGLE_TLV("Speaker Volume", SMA1301_0A_SPK_VOL,
		0, 167, 1, sma1301_spk_tlv),

SOC_ENUM_EXT("Volume slope", sma1301_vol_slope_enum,
	sma1301_vol_slope_get, sma1301_vol_slope_put),
SOC_ENUM_EXT("Mute slope", sma1301_mute_slope_enum,
	sma1301_mute_slope_get, sma1301_mute_slope_put),
SOC_SINGLE("Speaker Mute Switch(1:muted_0:un)",
		SMA1301_0E_MUTE_VOL_CTRL, 0, 1, 0),

SOC_ENUM_EXT("Speaker Mode", sma1301_spkmode_enum,
	sma1301_spkmode_get, sma1301_spkmode_put),

SOC_SINGLE("Speaker EQ(1:en_0:dis)", SMA1301_11_SYSTEM_CTRL2, 7, 1, 0),
SOC_SINGLE("Speaker Bass(1:en_0:dis)", SMA1301_11_SYSTEM_CTRL2, 6, 1, 0),
SOC_SINGLE("Speaker Comp/Limiter(1:en_0:dis)",
		SMA1301_11_SYSTEM_CTRL2, 5, 1, 0),
SOC_SINGLE("LR_DATA_SW(1:swap_0:normal)", SMA1301_11_SYSTEM_CTRL2, 4, 1, 0),
SOC_SINGLE("Mono Mix(1:en_0:dis)", SMA1301_11_SYSTEM_CTRL2, 0, 1, 0),

SOC_ENUM_EXT("Input gain", sma1301_input_gain_enum,
	sma1301_input_gain_get, sma1301_input_gain_put),
SOC_ENUM_EXT("Input gain for right channel", sma1301_input_r_gain_enum,
	sma1301_input_r_gain_get, sma1301_input_r_gain_put),

SOC_ENUM_EXT("Speaker HYSFB", sma1301_spk_hysfb_enum,
	sma1301_spk_hysfb_get, sma1301_spk_hysfb_put),
SND_SOC_BYTES_EXT("Speaker BDELAY", 1, spk_bdelay_get, spk_bdelay_put),

SND_SOC_BYTES_EXT("Bass Boost SPK Coeff", 7,
	bass_spk_coeff_get, bass_spk_coeff_put),
SND_SOC_BYTES_EXT("DRC SPK Coeff", 4,
	comp_lim_spk_coeff_get, comp_lim_spk_coeff_put),

SOC_ENUM_EXT("EQ Mode", sma1301_eq_mode_enum,
		sma1301_eq_mode_get, sma1301_eq_mode_put),

SOC_SINGLE("EQ Band1 Bypass Switch", SMA1301_2C_EQ_GRAPHIC1, 5, 1, 0),
SOC_SINGLE("EQ Band2 Bypass Switch", SMA1301_2D_EQ_GRAPHIC2, 5, 1, 0),
SOC_SINGLE("EQ Band3 Bypass Switch", SMA1301_2E_EQ_GRAPHIC3, 5, 1, 0),
SOC_SINGLE("EQ Band4 Bypass Switch", SMA1301_2F_EQ_GRAPHIC4, 5, 1, 0),
SOC_SINGLE("EQ Band5 Bypass Switch", SMA1301_30_EQ_GRAPHIC5, 5, 1, 0),
SND_SOC_BYTES_EXT("5 band equalizer(EqGraphic 1_5)",
		5, eqgraphic_get, eqgraphic_put),

SOC_SINGLE("SPK modulator sync(1:1/8_0:1/4)",
		SMA1301_33_SDM_CTRL, 2, 1, 0),

SOC_SINGLE("Edge displacement(1:dis_0:en)",
		SMA1301_36_PROTECTION, 7, 1, 0),
SOC_ENUM_EXT("PWM LR delay", sma1301_lr_delay_enum,
		sma1301_lr_delay_get, sma1301_lr_delay_put),
SOC_SINGLE("SRC random jitter(1:dis_0:add)",
		SMA1301_36_PROTECTION, 4, 1, 0),
SOC_SINGLE("OCP spk output state(1:dis_0:en)",
		SMA1301_36_PROTECTION, 3, 1, 0),
SOC_SINGLE("OCP mode(1:SD_0:AR)",
		SMA1301_36_PROTECTION, 2, 1, 0),
SOC_ENUM_EXT("OTP MODE", sma1301_otp_mode_enum,
		sma1301_otp_mode_get, sma1301_otp_mode_put),

SND_SOC_BYTES_EXT("SlopeCTRL", 1, slope_ctrl_get, slope_ctrl_put),

SND_SOC_BYTES_EXT("Test mode(Test 1~3_ATEST 1~2)",
		5, test_mode_get, test_mode_put),

SND_SOC_BYTES_EXT("EQ Ctrl Band1", 15,
		eq_ctrl_band1_get, eq_ctrl_band1_put),
SND_SOC_BYTES_EXT("EQ Ctrl Band2", 15,
		eq_ctrl_band2_get, eq_ctrl_band2_put),
SND_SOC_BYTES_EXT("EQ Ctrl Band3", 15,
		eq_ctrl_band3_get, eq_ctrl_band3_put),
SND_SOC_BYTES_EXT("EQ Ctrl Band4", 15,
		eq_ctrl_band4_get, eq_ctrl_band4_put),
SND_SOC_BYTES_EXT("EQ Ctrl Band5", 15,
		eq_ctrl_band5_get, eq_ctrl_band5_put),

SND_SOC_BYTES_EXT("PLL Setting", 5, pll_setting_get, pll_setting_put),

SOC_SINGLE("PLL enable(1:en_0:dis)",
		SMA1301_8B_PLL_POST_N, 5, 1, 0),

SOC_ENUM_EXT("Attack level control", sma1301_attack_lvl_enum,
	sma1301_attack_lvl_get, sma1301_attack_lvl_put),
SOC_ENUM_EXT("Release time control", sma1301_release_time_enum,
	sma1301_release_time_get, sma1301_release_time_put),

SOC_ENUM_EXT("Filtered VDD gain control", sma1301_flt_vdd_gain_enum,
	sma1301_flt_vdd_gain_get, sma1301_flt_vdd_gain_put),

SOC_SINGLE("Fast charge(1:dis_0:en)",
		SMA1301_92_FDPEC_CTRL, 2, 1, 0),

SOC_ENUM_EXT("Trimming of switching frequency", sma1301_trm_osc_enum,
	sma1301_trm_osc_get, sma1301_trm_osc_put),
SOC_ENUM_EXT("Trimming of ramp compensation", sma1301_trm_rmp_enum,
	sma1301_trm_rmp_get, sma1301_trm_rmp_put),
SOC_ENUM_EXT("Trimming of over current limit", sma1301_trm_ocl_enum,
	sma1301_trm_ocl_get, sma1301_trm_ocl_put),
SOC_ENUM_EXT("Trimming of loop compensation", sma1301_trm_comp_enum,
	sma1301_trm_comp_get, sma1301_trm_comp_put),
SOC_ENUM_EXT("Trimming of loop comp I gain", sma1301_trm_comp_i_enum,
	sma1301_trm_comp_i_get, sma1301_trm_comp_i_put),
SOC_ENUM_EXT("Trimming of loop comp P gain", sma1301_trm_comp_p_enum,
	sma1301_trm_comp_p_get, sma1301_trm_comp_p_put),
SOC_ENUM_EXT("Trimming of driver deadtime", sma1301_trm_dt_enum,
	sma1301_trm_dt_get, sma1301_trm_dt_put),
SOC_ENUM_EXT("Trimming of switching slew", sma1301_trm_slw_enum,
	sma1301_trm_slw_get, sma1301_trm_slw_put),
SOC_ENUM_EXT("Trimming of reference voltage", sma1301_trm_vref_enum,
	sma1301_trm_vref_get, sma1301_trm_vref_put),
SOC_ENUM_EXT("Trimming of boost voltage", sma1301_trm_vbst_enum,
	sma1301_trm_vbst_get, sma1301_trm_vbst_put),
SOC_ENUM_EXT("Trimming of minimum on-time", sma1301_trm_tmin_enum,
	sma1301_trm_tmin_get, sma1301_trm_tmin_put),

SOC_SINGLE("PLL Lock Skip Mode(1:dis_0:en)",
		SMA1301_A2_TOP_MAN1, 7, 1, 0),
SOC_SINGLE("PLL Power Down(1:PD_0:oper)",
		SMA1301_A2_TOP_MAN1, 6, 1, 0),
SOC_SINGLE("MCLK Sel(1:Ext Clk_0:PLL Clk)",
		SMA1301_A2_TOP_MAN1, 5, 1, 0),
SOC_SINGLE("PLL Ref Clk1(1:Int OSC_0:Ext Clk)",
		SMA1301_A2_TOP_MAN1, 4, 1, 0),
SOC_SINGLE("PLL Ref Clk2(1:SCK_0:PLL_REF_Clk1)",
		SMA1301_A2_TOP_MAN1, 3, 1, 0),
SOC_SINGLE("DAC Down Conver(1:C_0:N)",
		SMA1301_A2_TOP_MAN1, 2, 1, 0),
SOC_SINGLE("SDO Pad Output Ctrl(1:L_0:H)",
		SMA1301_A2_TOP_MAN1, 1, 1, 0),
SOC_SINGLE("Master Mode Enable(1:M_0:S)",
		SMA1301_A2_TOP_MAN1, 0, 1, 0),

SOC_SINGLE("Monitoring at SDO(1:OSC_0:PLL)",
		SMA1301_A3_TOP_MAN2, 7, 1, 0),
SOC_SINGLE("Test clk out en(1:Clk out_0:N)",
		SMA1301_A3_TOP_MAN2, 6, 1, 0),
SOC_SINGLE("PLL SDM PD(1:off_0:on)",
		SMA1301_A3_TOP_MAN2, 5, 1, 0),
SOC_SINGLE("Clk monitor time(1:4u_0:2u)",
		SMA1301_A3_TOP_MAN2, 4, 1, 0),
SOC_SINGLE("SDO output(1:H_0:N)",
		SMA1301_A3_TOP_MAN2, 3, 1, 0),
SOC_SINGLE("SDO output only(1:O_0:N)",
		SMA1301_A3_TOP_MAN2, 2, 1, 0),
SOC_SINGLE("Clk Monitor(1:Not_0:Mon)",
		SMA1301_A3_TOP_MAN2, 1, 1, 0),
SOC_SINGLE("OSC PD(1:PD_0:N)",
		SMA1301_A3_TOP_MAN2, 0, 1, 0),

SOC_ENUM_EXT("Top Manager Output Format", sma1301_o_format_enum,
	sma1301_o_format_get, sma1301_o_format_put),
SOC_ENUM_EXT("Top Manager SCK rate", sma1301_sck_rate_enum,
	sma1301_sck_rate_get, sma1301_sck_rate_put),
SOC_SINGLE("Top Manager LRCK Pol(1:H_0:L)",
		SMA1301_A4_TOP_MAN3, 0, 1, 0),
};

static int sma1301_startup(struct snd_soc_codec *codec)
{
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	if (sma1301->amp_power_status) {
		dev_info(codec->dev, "%s : %s\n",
			__func__, "Already AMP Power on");
		return 0;
	}

	dev_info(codec->dev, "%s : TRM_%s, FLT_%s\n",
	__func__, sma1301_trm_vbst_text[sma1301->bst_vol_lvl_status],
	sma1301_flt_vdd_gain_text[sma1301->flt_vdd_gain_status]);

	regmap_update_bits(sma1301->regmap, SMA1301_00_SYSTEM_CTRL,
			POWER_MASK, POWER_ON);

	if (sma1301->stereo_two_chip == true) {
		/* SPK Mode (Stereo) */
		regmap_update_bits(sma1301->regmap, SMA1301_10_SYSTEM_CTRL1,
				SPK_MODE_MASK, SPK_STEREO);
	} else {
		/* SPK Mode (Mono) */
		regmap_update_bits(sma1301->regmap, SMA1301_10_SYSTEM_CTRL1,
				SPK_MODE_MASK, SPK_MONO);
	}

	regmap_update_bits(sma1301->regmap, SMA1301_0E_MUTE_VOL_CTRL,
			SPK_MUTE_MASK, SPK_UNMUTE);

	sma1301->amp_power_status = true;

	return 0;
}

static int sma1301_shutdown(struct snd_soc_codec *codec)
{
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	if (!(sma1301->amp_power_status)) {
		dev_info(codec->dev, "%s : %s\n",
			__func__, "Already AMP Shutdown");
		return 0;
	}

	dev_info(codec->dev, "%s\n", __func__);

	regmap_update_bits(sma1301->regmap, SMA1301_0E_MUTE_VOL_CTRL,
			SPK_MUTE_MASK, SPK_MUTE);

	/* To improve the Boost OCP issue,
	 * time should be available for the Boost release time(40ms)
	 * and Mute slope time(15ms)
	 */
	msleep(55);

	regmap_update_bits(sma1301->regmap, SMA1301_10_SYSTEM_CTRL1,
			SPK_MODE_MASK, SPK_OFF);

	regmap_update_bits(sma1301->regmap, SMA1301_00_SYSTEM_CTRL,
			POWER_MASK, POWER_OFF);

	sma1301->amp_power_status = false;

	return 0;
}

static int sma1301_clk_supply_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_info(codec->dev, "%s : PRE_PMU\n", __func__);
	break;

	case SND_SOC_DAPM_POST_PMD:
		dev_info(codec->dev, "%s : POST_PMD\n", __func__);
	break;
	}

	return 0;
}

static int sma1301_dac_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_info(codec->dev, "%s : PRE_PMU\n", __func__);

		if (sma1301->force_amp_power_down == false)
			sma1301_startup(codec);

		break;

	case SND_SOC_DAPM_POST_PMU:
		dev_info(codec->dev, "%s : POST_PMU\n", __func__);

		break;

	case SND_SOC_DAPM_PRE_PMD:
		dev_info(codec->dev, "%s : PRE_PMD\n", __func__);

		sma1301_shutdown(codec);

		break;

	case SND_SOC_DAPM_POST_PMD:
		dev_info(codec->dev, "%s : POST_PMD\n", __func__);

		/* PLL Control disable */
		if (sma1301->sys_clk_id == SMA1301_PLL_CLKIN_MCLK
			|| sma1301->sys_clk_id == SMA1301_PLL_CLKIN_BCLK)
			regmap_update_bits(sma1301->regmap,
				SMA1301_8B_PLL_POST_N,
					PLL_EN_MASK, PLL_EN_DISABLE);

		break;
	}

	return 0;
}

static int sma1301_dac_feedback_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		dev_info(codec->dev, "%s : DAC feedback ON\n", __func__);
		regmap_update_bits(sma1301->regmap,
			SMA1301_09_OUTPUT_CTRL,
				PORT_CONFIG_MASK|PORT_OUT_SEL_MASK,
				OUTPUT_PORT_ENABLE|SPEAKER_PATH);
		/* even if Capture stream on, Mixer should turn on
		 * SDO output(1:High-Z,0:Normal output)
		 */
		regmap_update_bits(sma1301->regmap,
			SMA1301_A3_TOP_MAN2, SDO_OUTPUT_MASK,
				NORMAL_OUT);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		dev_info(codec->dev, "%s : DAC feedback OFF\n", __func__);
		regmap_update_bits(sma1301->regmap,
			SMA1301_09_OUTPUT_CTRL, PORT_OUT_SEL_MASK,
				OUT_SEL_DISABLE);
		regmap_update_bits(sma1301->regmap,
			SMA1301_A3_TOP_MAN2, SDO_OUTPUT_MASK,
				HIGH_Z_OUT);
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget sma1301_dapm_widgets[] = {
SND_SOC_DAPM_SUPPLY("CLK_SUPPLY", SND_SOC_NOPM, 0, 0, sma1301_clk_supply_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_DAC_E("DAC", "Playback", SND_SOC_NOPM, 0, 0, sma1301_dac_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_ADC_E("DAC_FEEDBACK", "Capture", SND_SOC_NOPM, 0, 0,
				sma1301_dac_feedback_event,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_OUTPUT("SPK"),
SND_SOC_DAPM_INPUT("SDO"),
};

static const struct snd_soc_dapm_route sma1301_audio_map[] = {
/* sink, control, source */
{"DAC", NULL, "CLK_SUPPLY"},
{"SPK", NULL, "DAC"},
{"DAC_FEEDBACK", NULL, "SDO"},
};

static int sma1301_setup_pll(struct snd_soc_codec *codec,
		struct snd_pcm_hw_params *params)
{
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	int i = 0;
	int calc_to_bclk = params_rate(params) * params_physical_width(params)
					* params_channels(params);

	dev_info(codec->dev, "%s : rate = %d : bit size = %d : channel = %d\n",
		__func__, params_rate(params), params_physical_width(params),
			params_channels(params));

	if (sma1301->sys_clk_id == SMA1301_PLL_CLKIN_MCLK) {
		/* PLL operation, PLL Clock, External Clock,
		 * PLL reference PLL_REF_CLK1 clock
		 */
		regmap_update_bits(sma1301->regmap, SMA1301_A2_TOP_MAN1,
		PLL_PD_MASK|MCLK_SEL_MASK|PLL_REF_CLK1_MASK|PLL_REF_CLK2_MASK,
		PLL_OPERATION|PLL_CLK|REF_EXTERNAL_CLK|PLL_REF_CLK1);

		for (i = 0; i < sma1301->num_of_pll_matches; i++) {
			if (sma1301->pll_matches[i].input_clk ==
					sma1301->mclk_in)
				break;
		}
	} else if (sma1301->sys_clk_id == SMA1301_PLL_CLKIN_BCLK) {
		/* PLL operation, PLL Clock, External Clock,
		 * PLL reference SCK clock
		 */
		regmap_update_bits(sma1301->regmap, SMA1301_A2_TOP_MAN1,
		PLL_PD_MASK|MCLK_SEL_MASK|PLL_REF_CLK1_MASK|PLL_REF_CLK2_MASK,
		PLL_OPERATION|PLL_CLK|REF_EXTERNAL_CLK|PLL_SCK);

		for (i = 0; i < sma1301->num_of_pll_matches; i++) {
			if (sma1301->pll_matches[i].input_clk ==
					calc_to_bclk)
				break;
			}
	}

	regmap_write(sma1301->regmap, SMA1301_8B_PLL_POST_N,
			sma1301->pll_matches[i].post_n);
	regmap_write(sma1301->regmap, SMA1301_8C_PLL_N,
			sma1301->pll_matches[i].n);
	regmap_write(sma1301->regmap, SMA1301_8D_PLL_F1,
			sma1301->pll_matches[i].f1);
	regmap_write(sma1301->regmap, SMA1301_8E_PLL_F2,
			sma1301->pll_matches[i].f2);
	regmap_write(sma1301->regmap, SMA1301_8F_PLL_F3,
			sma1301->pll_matches[i].f3_p_cp);

	/* PLL Control enable */
	regmap_update_bits(sma1301->regmap, SMA1301_8B_PLL_POST_N,
			PLL_EN_MASK, PLL_EN_ENABLE);

	return 0;
}

static int sma1301_dai_hw_params_amp(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	unsigned int input_format = 0;

	dev_info(codec->dev, "%s : rate = %d : bit size = %d\n", __func__,
		params_rate(params), params_width(params));

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		/* The sigma delta modulation setting for
		 * using the fractional divider in the PLL clock
		 * if (params_format(params) == SNDRV_PCM_FORMAT_S24_LE ||
		 *		params_rate(params) == 44100) {
		 *	regmap_update_bits(sma1301->regmap,
		 *	SMA1301_A3_TOP_MAN2, PLL_SDM_PD_MASK, SDM_ON);
		 * } else {
		 *	regmap_update_bits(sma1301->regmap,
		 *	SMA1301_A3_TOP_MAN2, PLL_SDM_PD_MASK, SDM_OFF);
		 * }
		 */
		/* PLL clock setting according to sample rate and bit */
		if (sma1301->force_amp_power_down == false &&
			(sma1301->sys_clk_id == SMA1301_PLL_CLKIN_MCLK
			|| sma1301->sys_clk_id == SMA1301_PLL_CLKIN_BCLK)) {

			regmap_update_bits(sma1301->regmap,
				SMA1301_03_INPUT1_CTRL3,
				BP_SRC_MASK, BP_SRC_NORMAL);

			sma1301_shutdown(codec);
			sma1301_setup_pll(codec, params);
			sma1301_startup(codec);
		}

		switch (params_rate(params)) {
		case 8000:
		case 12000:
		case 16000:
		case 24000:
		case 32000:
		case 44100:
		case 48000:
		case 96000:
		regmap_update_bits(sma1301->regmap, SMA1301_A2_TOP_MAN1,
				DAC_DN_CONV_MASK, DAC_DN_CONV_DISABLE);
		regmap_update_bits(sma1301->regmap, SMA1301_01_INPUT1_CTRL1,
				LEFTPOL_MASK, LOW_FIRST_CH);
		break;

		case 192000:
		regmap_update_bits(sma1301->regmap, SMA1301_A2_TOP_MAN1,
				DAC_DN_CONV_MASK, DAC_DN_CONV_ENABLE);
		regmap_update_bits(sma1301->regmap, SMA1301_01_INPUT1_CTRL1,
				LEFTPOL_MASK, HIGH_FIRST_CH);
		break;

		default:
			dev_err(codec->dev, "%s not support rate : %d\n",
				__func__, params_rate(params));

		return -EINVAL;
		}
	/* substream->stream is SNDRV_PCM_STREAM_CAPTURE */
	} else {

		switch (params_format(params)) {

		case SNDRV_PCM_FORMAT_S16_LE:
			dev_info(codec->dev,
				"%s set format SNDRV_PCM_FORMAT_S16_LE\n",
				__func__);
			break;

		case SNDRV_PCM_FORMAT_S24_LE:
			dev_info(codec->dev,
				"%s set format SNDRV_PCM_FORMAT_S24_LE\n",
				__func__);
			break;

		default:
			dev_err(codec->dev,
				"%s not support data bit : %d\n", __func__,
						params_format(params));
			return -EINVAL;
		}
	}

	switch (params_width(params)) {
	case 16:
		switch (sma1301->format) {
		case SND_SOC_DAIFMT_I2S:
			input_format |= STANDARD_I2S;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			input_format |= LJ;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			input_format |= RJ_16BIT;
			break;
		}
		break;
	case 24:
		switch (sma1301->format) {
		case SND_SOC_DAIFMT_I2S:
			input_format |= STANDARD_I2S;
			break;
		case SND_SOC_DAIFMT_LEFT_J:
			input_format |= LJ;
			break;
		case SND_SOC_DAIFMT_RIGHT_J:
			input_format |= RJ_24BIT;
			break;
		}
		break;

	default:
		dev_err(codec->dev,
			"%s not support data bit : %d\n", __func__,
					params_format(params));
		return -EINVAL;
	}

	regmap_update_bits(sma1301->regmap, SMA1301_01_INPUT1_CTRL1,
				I2S_MODE_MASK, input_format);

	return 0;
}

static int sma1301_dai_set_sysclk_amp(struct snd_soc_dai *dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s\n", __func__);

	/* Requested clock frequency is already setup */
	if (freq == sma1301->mclk_in)
		return 0;

	switch (clk_id) {
	case SMA1301_EXTERNAL_CLOCK_19_2:
		regmap_update_bits(sma1301->regmap, SMA1301_00_SYSTEM_CTRL,
				CLKSYSTEM_MASK, EXT_19_2);
		break;

	case SMA1301_EXTERNAL_CLOCK_24_576:
		regmap_update_bits(sma1301->regmap, SMA1301_00_SYSTEM_CTRL,
				CLKSYSTEM_MASK, EXT_24_576);
		break;
	case SMA1301_PLL_CLKIN_MCLK:
		if (freq < 1536000 || freq > 24576000) {
			/* out of range PLL_CLKIN, fall back to use BCLK */
			dev_warn(codec->dev, "Out of range PLL_CLKIN: %u\n",
				freq);
			clk_id = SMA1301_PLL_CLKIN_BCLK;
			freq = 0;
		}
	case SMA1301_PLL_CLKIN_BCLK:
		break;
	default:
		dev_err(codec->dev, "Invalid clk id: %d\n", clk_id);
		return -EINVAL;
	}
	sma1301->sys_clk_id = clk_id;
	sma1301->mclk_in = freq;
	return 0;
}

static int sma1301_dai_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	if (!(sma1301->amp_power_status)) {
		dev_info(codec->dev, "%s : %s\n",
			__func__, "Already AMP Shutdown");
		return 0;
	}

	if (mute) {
		dev_info(codec->dev, "%s : %s\n", __func__, "MUTE");

		regmap_update_bits(sma1301->regmap, SMA1301_0E_MUTE_VOL_CTRL,
					SPK_MUTE_MASK, SPK_MUTE);
	} else {
		dev_info(codec->dev, "%s : %s\n", __func__, "UNMUTE");

		regmap_update_bits(sma1301->regmap, SMA1301_0E_MUTE_VOL_CTRL,
					SPK_MUTE_MASK, SPK_UNMUTE);
	}

	return 0;
}

static int sma1301_dai_set_fmt_amp(struct snd_soc_dai *codec_dai,
					unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {

	case SND_SOC_DAIFMT_CBS_CFS:
		dev_info(codec->dev, "%s : %s\n", __func__, "I2S slave mode");
		/* I2S/PCM clock mode - slave mode */
		regmap_update_bits(sma1301->regmap, SMA1301_01_INPUT1_CTRL1,
					MASTER_SLAVE_MASK, SLAVE_MODE);
		regmap_update_bits(sma1301->regmap, SMA1301_A2_TOP_MAN1,
					MAS_IO_MASK, MAS_IO_SLAVE);
		break;

	case SND_SOC_DAIFMT_CBM_CFM:
		dev_info(codec->dev, "%s : %s\n", __func__, "I2S master mode");
		/* I2S/PCM clock mode - master mode */
		regmap_update_bits(sma1301->regmap, SMA1301_01_INPUT1_CTRL1,
					MASTER_SLAVE_MASK, MASTER_MODE);
		regmap_update_bits(sma1301->regmap, SMA1301_A2_TOP_MAN1,
					MAS_IO_MASK, MAS_IO_MASTER);
		break;

	default:
		dev_err(codec->dev, "Unsupported MASTER/SLAVE : 0x%x\n", fmt);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {

	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		sma1301->format = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
		break;

	default:
		dev_err(codec->dev, "Unsupported I2S FORMAT : 0x%x\n", fmt);
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops sma1301_dai_ops_amp = {
	.set_sysclk = sma1301_dai_set_sysclk_amp,
	.set_fmt = sma1301_dai_set_fmt_amp,
	.hw_params = sma1301_dai_hw_params_amp,
	.digital_mute = sma1301_dai_digital_mute,
};

#define SMA1301_RATES SNDRV_PCM_RATE_8000_192000
#define SMA1301_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver sma1301_dai[] = {
{
	.name = "sma1301-amplifier",
	.id = 0,
	.playback = {
	.stream_name = "Playback",
	.channels_min = 1,
	.channels_max = 2,
	.rates = SMA1301_RATES,
	.formats = SMA1301_FORMATS,
	},
	.capture = {
	.stream_name = "Capture",
	.channels_min = 1,
	.channels_max = 2,
	.rates = SMA1301_RATES,
	.formats = SMA1301_FORMATS,
	},
	.ops = &sma1301_dai_ops_amp,
}
};

static int sma1301_set_bias_level(struct snd_soc_codec *codec,
				enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:

		dev_info(codec->dev, "%s\n", "SND_SOC_BIAS_ON");
		sma1301_startup(codec);

		break;

	case SND_SOC_BIAS_PREPARE:

		dev_info(codec->dev, "%s\n", "SND_SOC_BIAS_PREPARE");

		break;

	case SND_SOC_BIAS_STANDBY:

		dev_info(codec->dev, "%s\n", "SND_SOC_BIAS_STANDBY");

		break;

	case SND_SOC_BIAS_OFF:

		dev_info(codec->dev, "%s\n", "SND_SOC_BIAS_OFF");
		sma1301_shutdown(codec);

		break;
	}

	/* Don't use codec->dapm.bias_level,
	 * use snd_soc_component_get_dapm() if it is needed
	 */

	return 0;
}

static void sma1301_check_fault_worker(struct work_struct *work)
{
	struct sma1301_priv *sma1301 =
		container_of(work, struct sma1301_priv, check_fault_work.work);
	int ret;
	unsigned int over_temp, ocp_val;

	mutex_lock(&sma1301->lock);

	ret = regmap_read(sma1301->regmap,
			SMA1301_0A_SPK_VOL, &sma1301->cur_vol);
	if (ret != 0) {
		dev_err(sma1301->dev,
			"failed to read SMA1301_0A_SPK_VOL : %d\n", ret);
		mutex_unlock(&sma1301->lock);
		return;
	}

	ret = regmap_read(sma1301->regmap, SMA1301_FA_STATUS1, &over_temp);
	if (ret != 0) {
		dev_err(sma1301->dev,
			"failed to read SMA1301_FA_STATUS1 : %d\n", ret);
		mutex_unlock(&sma1301->lock);
		return;
	}

	ret = regmap_read(sma1301->regmap, SMA1301_FB_STATUS2, &ocp_val);
	if (ret != 0) {
		dev_err(sma1301->dev,
			"failed to read SMA1301_FB_STATUS2 : %d\n", ret);
		mutex_unlock(&sma1301->lock);
		return;
	}
	/* Protected from setting larger than initial volume(0dB) */
	if (sma1301->cur_vol < sma1301->init_vol) {
		sma1301->cur_vol = sma1301->init_vol;
		regmap_write(sma1301->regmap,
			SMA1301_0A_SPK_VOL, sma1301->cur_vol);
	}

	if (~over_temp & OT1_OK_STATUS) {
		dev_crit(sma1301->dev,
			"%s : OT1(Over Temperature Level 1)\n", __func__);
		/* Volume control (Current Volume - 3dB) */
		regmap_write(sma1301->regmap,
			SMA1301_0A_SPK_VOL, sma1301->cur_vol + 6);
		sma1301->ot1_clear_flag = true;
	} else if (sma1301->ot1_clear_flag == true) {
		regmap_write(sma1301->regmap,
			SMA1301_0A_SPK_VOL, sma1301->cur_vol);
		sma1301->ot1_clear_flag = false;
	}
	if (~over_temp & OT2_OK_STATUS) {
		dev_crit(sma1301->dev,
			"%s : OT2(Over Temperature Level 2)\n", __func__);
	}
	if (ocp_val & OCP_SPK_STATUS) {
		dev_crit(sma1301->dev,
			"%s : OCP_SPK(Over Current Protect SPK)\n", __func__);
	}
	if (ocp_val & OCP_BST_STATUS) {
		dev_crit(sma1301->dev,
			"%s : OCP_BST(Over Current Protect Boost)\n", __func__);
	}
	if ((ocp_val & CLK_FAULT_STATUS) && (sma1301->amp_power_status)) {
		dev_crit(sma1301->dev,
			"%s : CLK_FAULT(Clock Fault Monitoring)\n", __func__);
	}

	if ((over_temp != sma1301->last_over_temp) ||
		(ocp_val != sma1301->last_ocp_val)) {

		dev_crit(sma1301->dev, "Please check AMP status");
		dev_info(sma1301->dev, "STATUS1=0x%02X : STATUS2=0x%02X\n",
				over_temp, ocp_val);
		sma1301->last_over_temp = over_temp;
		sma1301->last_ocp_val = ocp_val;
	}

	if (sma1301->check_fault_status) {
		if (sma1301->check_fault_period > 0)
			queue_delayed_work(system_freezable_wq,
				&sma1301->check_fault_work,
					sma1301->check_fault_period * HZ);
		else
			queue_delayed_work(system_freezable_wq,
				&sma1301->check_fault_work,
					CHECK_PERIOD_TIME * HZ);
	}
	mutex_unlock(&sma1301->lock);
}

#ifdef CONFIG_PM
static int sma1301_suspend(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "%s\n", __func__);

	return 0;
}

static int sma1301_resume(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "%s\n", __func__);

	return 0;
}
#else
#define sma1301_suspend NULL
#define sma1301_resume NULL
#endif

static int sma1301_reset(struct snd_soc_codec *codec)
{
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);
	struct reg_default *reg_val;
	int cnt, ret;
	int eq_len = sma1301->eq_reg_array_len / sizeof(uint32_t);
	unsigned int status;

	dev_info(codec->dev, "%s\n", __func__);

	ret = regmap_read(sma1301->regmap, SMA1301_FA_STATUS1, &status);

	if (ret != 0)
		dev_err(sma1301->dev,
			"failed to read SMA1301_FA_STATUS1 : %d\n", ret);
	else
		sma1301->rev_num = status & REV_NUM_STATUS;

	if (sma1301->rev_num == REV_NUM_ES0)
		dev_info(codec->dev, "SMA1301 chip revision ID - ES0\n");
	else if (sma1301->rev_num == REV_NUM_ES1)
		dev_info(codec->dev, "SMA1301 chip revision ID - ES1\n");

	/* External clock 24.576MHz */
	regmap_write(sma1301->regmap, SMA1301_00_SYSTEM_CTRL,	0x80);
	/* Volume control (0dB/0x32) */
	regmap_write(sma1301->regmap, SMA1301_0A_SPK_VOL, sma1301->init_vol);
	/* VOL_SLOPE - Fast, MUTE_SLOPE - Fast */
	regmap_write(sma1301->regmap, SMA1301_0E_MUTE_VOL_CTRL,	0xFF);
	/* Mono for one chip solution */
	regmap_write(sma1301->regmap, SMA1301_10_SYSTEM_CTRL1,	0x04);

	if (sma1301->stereo_two_chip == true) {
		/* MONO MIX Off */
		regmap_update_bits(sma1301->regmap,
		SMA1301_11_SYSTEM_CTRL2, MONOMIX_MASK, MONOMIX_OFF);
	} else {
		/* MONO MIX ON */
		regmap_update_bits(sma1301->regmap,
		SMA1301_11_SYSTEM_CTRL2, MONOMIX_MASK, MONOMIX_ON);
	}

	/* Delay control between OUTA and OUTB
	 * with main clock duty cycle
	 */
	regmap_write(sma1301->regmap, SMA1301_14_MODULATOR,	0x12);
	/* EQ band1, band2, band3, band4, band5 Bypass */
	regmap_update_bits(sma1301->regmap, SMA1301_2C_EQ_GRAPHIC1,
			EQ_BAND1_BYPASS_MASK, EQ_BAND1_BYPASS);
	regmap_update_bits(sma1301->regmap, SMA1301_2D_EQ_GRAPHIC2,
			EQ_BAND2_BYPASS_MASK, EQ_BAND2_BYPASS);
	regmap_update_bits(sma1301->regmap, SMA1301_2E_EQ_GRAPHIC3,
			EQ_BAND3_BYPASS_MASK, EQ_BAND3_BYPASS);
	regmap_update_bits(sma1301->regmap, SMA1301_2F_EQ_GRAPHIC4,
			EQ_BAND4_BYPASS_MASK, EQ_BAND4_BYPASS);
	regmap_update_bits(sma1301->regmap, SMA1301_30_EQ_GRAPHIC5,
			EQ_BAND5_BYPASS_MASK, EQ_BAND5_BYPASS);
	/* Enable test registers */
	regmap_write(sma1301->regmap, SMA1301_3B_TEST1, 0x5A);
	/* Stereo idle noise improvement : Improved idle noise by
	 * asynchronizing the PWM waveform of Left and Right
	 */
	regmap_update_bits(sma1301->regmap, SMA1301_3C_TEST2,
			SDM_SYNC_DIS_MASK, SDM_SYNC_DISABLE);
	/* Low power mode operation, SPK output frequency - 470kHz,
	 * Thermal adjust - 140C, 100C
	 */
	regmap_update_bits(sma1301->regmap, SMA1301_3F_ATEST2,
			SPK_OUT_FREQ_MASK, SPK_OUT_FREQ_470K);
	regmap_update_bits(sma1301->regmap, SMA1301_3F_ATEST2,
			LOW_POWER_MODE_MASK, LOW_POWER_MODE_ENABLE);
	regmap_update_bits(sma1301->regmap, SMA1301_3F_ATEST2,
			THERMAL_ADJUST_MASK, THERMAL_140_100);

	/* Attack level - 0.25FS,
	 * Release time - 40ms(48kHz) & 20ms(96kHz)
	 */
	regmap_write(sma1301->regmap, SMA1301_91_CLASS_G_CTRL,	0x42);
	/* Filtered VDD gain control 2.8V */
	regmap_write(sma1301->regmap, SMA1301_92_FDPEC_CTRL,	0x80);
	sma1301->flt_vdd_gain_status = (FLT_VDD_GAIN_2P80 >> 4);
	/* Trimming of switching frequency - 1.4MHz,
	 * Trimming of ramp compensation - 2.94A/us
	 */
	regmap_write(sma1301->regmap, SMA1301_94_BOOST_CTRL1,	0x05);
	if (sma1301->rev_num <= REV_NUM_ES0) {
		sma1301->pll_matches = sma1301_pll_matches;
		sma1301->num_of_pll_matches =
			ARRAY_SIZE(sma1301_pll_matches);
		/* Apply tuning values of PWM slope control and
		 * PWM dead time control
		 * SPK_SLOPE - 2'b11, SPK_DEAD_TIME - 4'b1000
		 */
		regmap_write(sma1301->regmap, SMA1301_37_SLOPE_CTRL, 0x38);
		/* Trimming of over current limit - 2.21A,
		 * Trimming of loop compensation
		 * - PI compensation, 50pF I-gain, 1.0Mohm P-gain
		 */
		regmap_write(sma1301->regmap, SMA1301_95_BOOST_CTRL2, 0x21);
		/* Trimming of reference voltage - 1.19V,
		 * boost voltage - 5.6V, minimum on-time - 62ns
		 */
		regmap_write(sma1301->regmap, SMA1301_97_BOOST_CTRL4,	0xA6);
	} else {
		sma1301->pll_matches = sma1301_pll_matches_shift;
		sma1301->num_of_pll_matches =
			ARRAY_SIZE(sma1301_pll_matches_shift);
		/* Apply tuning values of PWM slope control and
		 * PWM dead time control
		 * SPK_SLOPE - 2'b00, SPK_DEAD_TIME - 4'b0000
		 */
		regmap_write(sma1301->regmap, SMA1301_37_SLOPE_CTRL, 0x00);
		/* Enable fast off drive speaker */
		regmap_update_bits(sma1301->regmap, SMA1301_3F_ATEST2,
					FAST_OFF_DRIVE_SPK_MASK,
					FAST_OFF_DRIVE_SPK_ENABLE);
		/* Trimming of over current limit - 2.21A,
		 * Trimming of loop compensation
		 * - PI compensation, 65pF I-gain, 1.0Mohm P-gain
		 */
		regmap_write(sma1301->regmap, SMA1301_95_BOOST_CTRL2, 0x29);
		/* Trimming of reference voltage - 1.2V,
		 * boost voltage - 5.6V, minimum on-time - 62ns
		 */
		regmap_write(sma1301->regmap, SMA1301_97_BOOST_CTRL4,	0x86);
	}
	/* Trimming of driver deadtime - 0.75ns, switching slew - 2.5ns */
	regmap_write(sma1301->regmap, SMA1301_96_BOOST_CTRL3,	0xF3);

	sma1301->bst_vol_lvl_status = (TRM_VBST_5P6 >> 2);

	if (sma1301->src_bypass == true) {
		dev_info(codec->dev, "If do not use SRC, mono mix does not work property\n");
		regmap_update_bits(sma1301->regmap, SMA1301_03_INPUT1_CTRL3,
			BP_SRC_MASK, BP_SRC_BYPASS);
	} else {
		regmap_update_bits(sma1301->regmap, SMA1301_03_INPUT1_CTRL3,
			BP_SRC_MASK, BP_SRC_NORMAL);
	}

	if (sma1301->sys_clk_id == SMA1301_EXTERNAL_CLOCK_19_2
		|| sma1301->sys_clk_id == SMA1301_PLL_CLKIN_MCLK) {
		regmap_update_bits(sma1301->regmap, SMA1301_00_SYSTEM_CTRL,
			CLKSYSTEM_MASK, EXT_19_2);

		regmap_update_bits(sma1301->regmap, SMA1301_03_INPUT1_CTRL3,
			BP_SRC_MASK, BP_SRC_NORMAL);
	}

	dev_info(codec->dev,
		"%s init_vol is 0x%x\n", __func__, sma1301->init_vol);
	/* EQ register value writing
	 * if register value is available from DT
	 */
	if (sma1301->eq_reg_array != NULL) {
		for (cnt = 0; cnt < eq_len; cnt += 2) {
			reg_val = (struct reg_default *)
				&sma1301->eq_reg_array[cnt];
			dev_dbg(codec->dev, "%s reg_write [0x%02x, 0x%02x]",
					__func__, be32_to_cpu(reg_val->reg),
						be32_to_cpu(reg_val->def));
			regmap_write(sma1301->regmap, be32_to_cpu(reg_val->reg),
					be32_to_cpu(reg_val->def));
		}
	}

	if (sma1301->check_fault_status) {
		if (sma1301->check_fault_period > 0)
			queue_delayed_work(system_freezable_wq,
				&sma1301->check_fault_work,
					sma1301->check_fault_period * HZ);
		else
			queue_delayed_work(system_freezable_wq,
				&sma1301->check_fault_work,
					CHECK_PERIOD_TIME * HZ);
	}

	return 0;
}

static int sma1301_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	char *dapm_widget_str = NULL;
	int prefix_len = 0, str_max = 30;

	dev_info(codec->dev, "%s\n", __func__);

	if (codec->component.name_prefix != NULL) {
		dev_info(codec->dev, "%s : component name prefix - %s\n",
			__func__, codec->component.name_prefix);

		prefix_len = strlen(codec->component.name_prefix);
		dapm_widget_str = kzalloc(prefix_len + str_max, GFP_KERNEL);

		if (!dapm_widget_str)
			return -ENOMEM;

		strcpy(dapm_widget_str, codec->component.name_prefix);
		strcat(dapm_widget_str, " Playback");

		snd_soc_dapm_ignore_suspend(dapm, dapm_widget_str);

		memset(dapm_widget_str + prefix_len, 0, str_max);

		strcpy(dapm_widget_str, codec->component.name_prefix);
		strcat(dapm_widget_str, " SPK");

		snd_soc_dapm_ignore_suspend(dapm, dapm_widget_str);
	} else {
		snd_soc_dapm_ignore_suspend(dapm, "Playback");
		snd_soc_dapm_ignore_suspend(dapm, "SPK");
	}

	snd_soc_dapm_sync(dapm);

	if (dapm_widget_str != NULL)
		kfree(dapm_widget_str);

	sma1301_reset(codec);

	return 0;
}

static int sma1301_remove(struct snd_soc_codec *codec)
{
	struct sma1301_priv *sma1301 = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s\n", __func__);

	sma1301_set_bias_level(codec, SND_SOC_BIAS_OFF);
	devm_kfree(sma1301->dev, sma1301);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_sma1301 = {
	.probe = sma1301_probe,
	.remove = sma1301_remove,
	.suspend = sma1301_suspend,
	.resume = sma1301_resume,
	.controls = sma1301_snd_controls,
	.num_controls = ARRAY_SIZE(sma1301_snd_controls),
	.dapm_widgets = sma1301_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sma1301_dapm_widgets),
	.dapm_routes = sma1301_audio_map,
	.num_dapm_routes = ARRAY_SIZE(sma1301_audio_map),
	.idle_bias_off = true,
};

const struct regmap_config sma_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = SMA1301_FF_DEVICE_INDEX,
	.readable_reg = sma1301_readable_register,
	.writeable_reg = sma1301_writeable_register,
	.volatile_reg = sma1301_volatile_register,

	.cache_type = REGCACHE_NONE,
	.reg_defaults = sma1301_reg_def,
	.num_reg_defaults = ARRAY_SIZE(sma1301_reg_def),
};

static ssize_t sma1301_check_fault_period_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma1301_priv *sma1301 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma1301->check_fault_period);

	return (ssize_t)rc;
}

static ssize_t sma1301_check_fault_period_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma1301_priv *sma1301 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma1301->check_fault_period);

	if (ret)
		return -EINVAL;

	return (ssize_t)count;
}

static DEVICE_ATTR(check_fault_period, 0644,
	sma1301_check_fault_period_show, sma1301_check_fault_period_store);

static ssize_t sma1301_check_fault_status_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct sma1301_priv *sma1301 = dev_get_drvdata(dev);
	int rc;

	rc = (int)snprintf(buf, PAGE_SIZE,
			"%ld\n", sma1301->check_fault_status);

	return (ssize_t)rc;
}

static ssize_t sma1301_check_fault_status_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct sma1301_priv *sma1301 = dev_get_drvdata(dev);
	int ret;

	ret = kstrtol(buf, 10, &sma1301->check_fault_status);

	if (ret)
		return -EINVAL;

	if (sma1301->check_fault_status) {
		if (sma1301->check_fault_period > 0)
			queue_delayed_work(system_freezable_wq,
				&sma1301->check_fault_work,
					sma1301->check_fault_period * HZ);
		else
			queue_delayed_work(system_freezable_wq,
				&sma1301->check_fault_work,
					CHECK_PERIOD_TIME * HZ);
	}

	return (ssize_t)count;
}

static DEVICE_ATTR(check_fault_status, 0644,
	sma1301_check_fault_status_show, sma1301_check_fault_status_store);

static struct attribute *sma1301_attr[] = {
	&dev_attr_check_fault_period.attr,
	&dev_attr_check_fault_status.attr,
	NULL,
};

static struct attribute_group sma1301_attr_group = {
	.attrs = sma1301_attr,
};

static int sma1301_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct sma1301_priv *sma1301;
	struct device_node *np = client->dev.of_node;
	int ret;
	u32 value, value_clk;
	unsigned int version_status;

	dev_info(&client->dev, "%s is here. Driver version REV012\n", __func__);

	sma1301 = devm_kzalloc(&client->dev, sizeof(struct sma1301_priv),
							GFP_KERNEL);

	if (!sma1301)
		return -ENOMEM;

	sma1301->regmap = devm_regmap_init_i2c(client, &sma_i2c_regmap);
	if (IS_ERR(sma1301->regmap)) {
		ret = PTR_ERR(sma1301->regmap);
		dev_err(&client->dev,
			"Failed to allocate register map: %d\n", ret);
		return ret;
	}

	if (np) {
		if (!of_property_read_u32(np, "init-vol", &value)) {
			dev_info(&client->dev,
				"init_vol is 0x%x from DT\n", value);
			sma1301->init_vol = value;
		} else {
			dev_info(&client->dev,
				"init_vol is set with 0x32(0dB)\n");
			sma1301->init_vol = 0x32;
		}
		if (of_property_read_bool(np, "stereo-two-chip")) {
			dev_info(&client->dev, "Stereo for two chip solution\n");
				sma1301->stereo_two_chip = true;
		} else {
			dev_info(&client->dev, "Mono for one chip solution\n");
				sma1301->stereo_two_chip = false;
		}
		if (of_property_read_bool(np, "impossible-bst-ctrl")) {
			dev_info(&client->dev, "Boost control setting is not possible\n");
				sma1301->impossible_bst_ctrl = true;
		} else {
			dev_info(&client->dev, "Boost control setting is possible\n");
				sma1301->impossible_bst_ctrl = false;
		}
		if (!of_property_read_u32(np, "sys-clk-id", &value)) {
			switch (value) {
			case SMA1301_EXTERNAL_CLOCK_19_2:
				dev_info(&client->dev, "Use the external 19.2MHz clock\n");
				break;
			case SMA1301_EXTERNAL_CLOCK_24_576:
				dev_info(&client->dev, "Use the external 24.576MHz clock\n");
				break;
			case SMA1301_PLL_CLKIN_MCLK:
				if (!of_property_read_u32(np,
					"mclk-freq", &value_clk))
					sma1301->mclk_in = value_clk;
				else
					sma1301->mclk_in = 19200000;

				dev_info(&client->dev,
				"Take an external %dHz clock and covert it to an internal PLL for use\n",
					sma1301->mclk_in);
				break;
			case SMA1301_PLL_CLKIN_BCLK:
				dev_info(&client->dev,
				"Take an BCLK(SCK) and covert it to an internal PLL for use\n");
				break;
			default:
				dev_err(&client->dev,
					"Invalid sys-clk-id: %d\n", value);
				return -EINVAL;
			}
			sma1301->sys_clk_id = value;
		} else {
			dev_info(&client->dev, "Use the internal PLL clock by default\n");
			sma1301->sys_clk_id = SMA1301_PLL_CLKIN_BCLK;
		}
		if (of_property_read_bool(np, "SRC-bypass")) {
			dev_info(&client->dev,
					"Do not set the sample rate converter\n");
				sma1301->src_bypass = true;
		} else {
			dev_info(&client->dev, "Set the sample rate converter\n");
				sma1301->src_bypass = false;
		}

		sma1301->eq_reg_array = of_get_property(np, "registers-of-eq",
			&sma1301->eq_reg_array_len);
		if (sma1301->eq_reg_array == NULL)
			dev_info(&client->dev,
				"There is no EQ registers from DT\n");
	} else {
		dev_err(&client->dev,
			"device node initialization error\n");
		/* SMA1301 REV 008 TEST */
		/* return -ENODEV; */
	}

	ret = regmap_read(sma1301->regmap,
		SMA1301_FF_DEVICE_INDEX, &version_status);

	if ((ret != 0) || (version_status != DEVICE_INDEX)) {
		dev_err(&client->dev, "device initialization error (%d 0x%02X)",
				ret, version_status);
		/* SMA1301 REV 008 TEST for -5 IO ERROR */
		/* return -ENODEV; */
	}
	dev_info(&client->dev, "chip version 0x%02X\n", version_status);

	/* set initial value as normal AMP IC status */
	sma1301->last_over_temp = 0xC0;
	sma1301->last_ocp_val = 0x06;
	sma1301->cur_vol = sma1301->init_vol;
	sma1301->ot1_clear_flag = false;

	INIT_DELAYED_WORK(&sma1301->check_fault_work,
		sma1301_check_fault_worker);

	mutex_init(&sma1301->lock);
	sma1301->check_fault_period = CHECK_PERIOD_TIME;

	sma1301->devtype = id->driver_data;
	sma1301->dev = &client->dev;
	sma1301->kobj = &client->dev.kobj;

	i2c_set_clientdata(client, sma1301);

	sma1301->force_amp_power_down = false;
	sma1301->amp_power_status = false;
	sma1301->check_fault_status = true;

	ret = snd_soc_register_codec(&client->dev,
		&soc_codec_dev_sma1301, sma1301_dai, ARRAY_SIZE(sma1301_dai));

	/* Create sma1301 sysfs attributes */
	sma1301->attr_grp = &sma1301_attr_group;
	ret = sysfs_create_group(sma1301->kobj, sma1301->attr_grp);

	if (ret) {
		dev_err(&client->dev,
			"failed to create attribute group [%d]\n", ret);
		sma1301->attr_grp = NULL;
	}

	return ret;
}

static int sma1301_i2c_remove(struct i2c_client *client)
{
	struct sma1301_priv *sma1301 =
		(struct sma1301_priv *) i2c_get_clientdata(client);

	dev_info(&client->dev, "%s\n", __func__);

	if (sma1301)
		devm_kfree(&client->dev, sma1301);

	snd_soc_unregister_codec(&client->dev);

	return 0;
}

static const struct i2c_device_id sma1301_i2c_id[] = {
	{"sma1301", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, sma1301_i2c_id);

static const struct of_device_id sma1301_of_match[] = {
	{ .compatible = "siliconmitus,sma1301", },
	{ }
};
MODULE_DEVICE_TABLE(of, sma1301_of_match);

static struct i2c_driver sma1301_i2c_driver = {
	.driver = {
		.name = "sma1301",
		.of_match_table = sma1301_of_match,
	},
	.probe = sma1301_i2c_probe,
	.remove = sma1301_i2c_remove,
	.id_table = sma1301_i2c_id,
};

static int __init sma1301_init(void)
{
	int ret;

	ret = i2c_add_driver(&sma1301_i2c_driver);

	if (ret)
		pr_err("Failed to register sma1301 I2C driver: %d\n", ret);

	return ret;
}

static void __exit sma1301_exit(void)
{
	i2c_del_driver(&sma1301_i2c_driver);
}

module_init(sma1301_init);
module_exit(sma1301_exit);

MODULE_DESCRIPTION("ALSA SoC SMA1301 driver");
MODULE_AUTHOR("Brian Pyun, <moosung.pyun@siliconmitus.com>");
MODULE_AUTHOR("GH Park, <gyuhwa.park@irondevice.com>");
MODULE_LICENSE("GPL v2");
