/*
 * mdnie_control.c - mDNIe register sequence intercept and control
 *
 * @Author	: Andrei F. <https://github.com/AndreiLux>
 * @Date	: February 2013 - May 2015
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/device.h>

#include "mdnie.h"

#define DEBUG 0

#ifdef CONFIG_EXYNOS_DECON_MDNIE_LITE
#define MDNIE_LITE
struct mdnie_seq_info *cmds;
#endif

struct mdnie_info *g_mdnie; 

static int reg_hook = 0;
#ifndef MDNIE_LITE
static bool sequence_hook = 0;
#endif

enum mdnie_registers {
	CS_HG_RY	= 0x50,	/*CS hg ry*/
	CS_HG_GC	= 0x51,	/*CS hg gc*/
	CS_HG_BM	= 0x52,	/*CS hg bm*/
	CS_WEIGHT_GRTH	= 0x53,	/*CS weight grayTH*/
	
	SCR_S_RR_YR	= 0x9a,	/*ASCR skin_Rr skin_Yr*/
	SCR_S_RG_YG	= 0x9b,	/*ASCR skin_Rg skin_Yg*/
	SCR_S_RB_YB	= 0x9c,	/*ASCR skin_Rb skin_Yb*/
	
	SCR_S_MR_WR	= 0x9d,	/*ASCR skin_Mr skin_Wr*/
	SCR_S_MG_WG	= 0x9e,	/*ASCR skin_Mg skin_Wg*/
	SCR_S_MB_WB	= 0x9f,	/*ASCR skin_Mb skin_Wb*/

	SCR_RR_CR	= 0xa1,	/*SCR RrCr*/
	SCR_RG_CG	= 0xa2,	/*SCR RgCg*/
	SCR_RB_CB	= 0xa3,	/*SCR RbCb*/

	SCR_GR_MR	= 0xa4,	/*SCR GrMr*/
	SCR_GG_MG	= 0xa5,	/*SCR GgMg*/
	SCR_GB_MB	= 0xa6,	/*SCR GbMb*/

	SCR_BR_YR	= 0xa7,	/*SCR BrYr*/
	SCR_BG_YG	= 0xa8,	/*SCR BgYg*/
	SCR_BB_YB	= 0xa9,	/*SCR BbYb*/

	SCR_KR_WR	= 0xaa,	/*SCR KrWr*/
	SCR_KG_WG	= 0xab,	/*SCR KgWg*/
	SCR_KB_WB	= 0xac,	/*SCR KbWb*/

	CC_CHSEL_STR	= 0x5f,	/*CC chsel strength*/
	CC_0		= 0x60,	/*CC lut r   0*/
	CC_1		= 0x61,	/*CC lut r  16 144*/
	CC_2		= 0x62,	/*CC lut r  32 160*/
	CC_3		= 0x63,	/*CC lut r  48 176*/
	CC_4		= 0x64,	/*CC lut r  64 192*/
	CC_5		= 0x65,	/*CC lut r  80 208*/
	CC_6		= 0x66,	/*CC lut r  96 224*/
	CC_7		= 0x67,	/*CC lut r 112 240*/
	CC_8		= 0x68	/*CC lut r 128 255*/
};

#ifndef MDNIE_LITE
static unsigned short master_sequence[] = { 
	0x06,0x0006,0x07,0x0000,0x08,0x0006,0x09,0x0006,0x0a,0x0006,
	0x0b,0x0007,0x50,0x1010,0x51,0x1010,0x52,0x1010,0x53,0x0014,
	0x5f,0x0080,0x60,0x0000,0x61,0x22b1,0x62,0x38be,0x63,0x4cc9,
	0x64,0x63d5,0x65,0x7ae0,0x67,0x97f5,0x90,0x1000,0x91,0x6a9a,
	0x92,0x251a,0x93,0x162a,0x94,0x0000,0x95,0x375a,0x96,0x4ec5,
	0x97,0x5d17,0x98,0x30c3,0x9a,0xffff,0x9b,0x00ff,0x9c,0x0000,
	0x9d,0xffff,0x9e,0x00ff,0x9f,0xffff,0xa1,0xd994,0xa2,0x20f0,
	0xa3,0x2ced,0xa4,0x8ae4,0xa5,0xe72f,0xa6,0x50e3,0xa7,0x38f6,
	0xa8,0x1dee,0xa9,0xdd59,0xaa,0xff00,0xab,0xf700,0xac,0xef00,
	0xff,0x0000,END_SEQ,0x0000,
};
#endif

static ssize_t show_mdnie_property(struct device *dev,
				    struct device_attribute *attr, char *buf);

static ssize_t store_mdnie_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count);

#define _effect(name_, reg_, mask_, shift_, absolute_, regval_)\
{ 									\
	.attribute = {							\
			.attr = {					\
				  .name = name_,			\
				  .mode = 0664,				\
				},					\
			.show = show_mdnie_property,			\
			.store = store_mdnie_property,			\
		     },							\
	.reg 	= reg_ ,						\
	.mask 	= mask_ ,						\
	.shift 	= shift_ ,						\
	.value 	= 0 ,							\
	.abs	= absolute_ ,						\
	.regval = regval_						\
}

struct mdnie_effect {
	const struct device_attribute	attribute;
	u8				reg;
	unsigned short			mask;
	u8				shift;
	int				value; 
	bool				abs;
	mdnie_t				regval;
};

#ifdef MDNIE_LITE
#define r(x,y) x
#else
#define r(x,y) y
#endif

static struct mdnie_effect mdnie_controls[] = {
#ifdef MDNIE_LITE
	_effect("lce_on gain"		, 1		, 0x00ff, 0	,1, 152	),
	_effect("lce_color_gain"	, 2		, 0x00ff, 0	,1, 36	),
	_effect("lce_scene"		, 3		, 0x00ff, 0	,1, 16	),
	_effect("lce_min_diff"		, 4		, 0x00ff, 0	,1, 20	),
	_effect("lce_illum_gain"	, 5		, 0x00ff, 0	,1, 179	),
	_effect("lce_ref_offset"	, 6		, 0x00ff, 0	,1, 1	),
	_effect("lce_ref_offset2"	, 7		, 0x00ff, 0	,1, 14	),
	_effect("lce_ref_gain"		, 8		, 0x00ff, 0	,1, 1	),
	_effect("lce_ref_gain2"		, 9		, 0x00ff, 0	,1, 0	),
	_effect("lce_block_size"	, 10		, 0x00ff, 0	,1, 102	),
	_effect("lce_bright_th"		, 11		, 0x00ff, 0	,1, 250	),
	_effect("lce_bin_size_ratio"	, 12		, 0x00ff, 0	,1, 45	),
	_effect("lce_dark_th"		, 13		, 0x00ff, 0	,1, 3	),
#endif

	/* Chroma saturation */
	_effect("cs_weight"		, r(52,CS_WEIGHT_GRTH), 0xff00, 8,1, r(1,36)),
#ifndef MDNIE_LITE
	_effect("cs_gray_threshold"	, CS_WEIGHT_GRTH, 0x00ff, 0	,1, 4	),

	_effect("cs_red"		, CS_HG_RY	, 0xff00, 8	,1, 6	),
	_effect("cs_green"		, CS_HG_GC	, 0xff00, 8	,1, 8	),
	_effect("cs_blue"		, CS_HG_BM	, 0xff00, 8	,1, 0	),

	_effect("cs_yellow"		, CS_HG_RY	, 0x00ff, 0	,1, 8	),
	_effect("cs_cyan"		, CS_HG_GC	, 0x00ff, 0	,1, 8	),
	_effect("cs_magenta"		, CS_HG_BM	, 0x00ff, 0	,1, 8	),
#endif

	/* Colour channel modifiers
	 * scr_x_y:
	 *	x = Channel
	 *	y = Channel component modifier
	 */
	
#ifdef MDNIE_LITE
	_effect("ascr_lin_mask"		, 102		, 0x00ff, 0	,1, 0x40),
	_effect("ascr_skin_cb"		, 103		, 0x00ff, 0	,1, 0x67),
	_effect("ascr_skin_cr"		, 104		, 0x00ff, 0	,1, 0xa9),
	
	_effect("ascr_dist_up"		, 105		, 0x00ff, 0	,1, 0x17),
	_effect("ascr_dist_down"	, 106		, 0x00ff, 0	,1, 0x29),
	_effect("ascr_dist_right"	, 107		, 0x00ff, 0	,1, 0x19),
	_effect("ascr_dist_left"	, 108		, 0x00ff, 0	,1, 0x27),
	
	_effect("ascr_div_up1"		, 109		, 0x00ff, 0	,1, 0x00),
	_effect("ascr_div_up2"		, 110		, 0x00ff, 0	,1, 0x59),
	_effect("ascr_div_up3"		, 111		, 0x00ff, 0	,1, 0x0b),
	
	_effect("ascr_div_down1"	, 112		, 0x00ff, 0	,1, 0x00),
	_effect("ascr_div_down2"	, 113		, 0x00ff, 0	,1, 0x31),
	_effect("ascr_div_down3"	, 114		, 0x00ff, 0	,1, 0xf4),
	
	_effect("ascr_div_right1"	, 115		, 0x00ff, 0	,1, 0x00),
	_effect("ascr_div_right2"	, 116		, 0x00ff, 0	,1, 0x51),
	_effect("ascr_div_right3"	, 117		, 0x00ff, 0	,1, 0xec),
	
	_effect("ascr_div_left1"	, 118		, 0x00ff, 0	,1, 0x00),
	_effect("ascr_div_left2"	, 119		, 0x00ff, 0	,1, 0x34),
	_effect("ascr_div_left3"	, 120		, 0x00ff, 0	,1, 0x83),
#endif
	
	/* ASCR Skin */
	
	_effect("skin_red_red"		, r(121, SCR_S_RR_YR	), 0xff00, 8	,1, 255	),
	_effect("skin_red_green"	, r(122, SCR_S_RG_YG	), 0xff00, 8	,1, 80	),
	_effect("skin_red_blue"		, r(123, SCR_S_RB_YB	), 0xff00, 8	,1, 96	),

	_effect("skin_yellow_red"	, r(124, SCR_S_RR_YR	), 0x00ff, 0	,1, 255	),
	_effect("skin_yellow_green"	, r(125, SCR_S_RG_YG	), 0x00ff, 0	,1, 255	),
	_effect("skin_yellow_blue"	, r(126, SCR_S_RB_YB	), 0x00ff, 0	,1, 0	),
	
	_effect("skin_magenta_red"	, r(127, SCR_S_MR_WR	), 0xff00, 8	,1, 255	),
	_effect("skin_magenta_green"	, r(128, SCR_S_MG_WG	), 0xff00, 8	,1, 0	),
	_effect("skin_magenta_blue"	, r(129, SCR_S_MB_WB	), 0xff00, 8	,1, 255	),

	_effect("skin_white_red"	, r(130, SCR_S_MR_WR	), 0x00ff, 0	,1, 255	),
	_effect("skin_white_green"	, r(131, SCR_S_MG_WG	), 0x00ff, 0	,1, 255	),
	_effect("skin_white_blue"	, r(132, SCR_S_MB_WB	), 0x00ff, 0	,1, 255	),

	/* ASCR Wide */

	_effect("wide_red_red"		, r(134, SCR_RR_CR	), 0xff00, 8	,1, 210	),
	_effect("wide_red_green"	, r(136, SCR_RG_CG	), 0xff00, 8	,1, 27	),
	_effect("wide_red_blue"		, r(138, SCR_RB_CB	), 0xff00, 8	,1, 42	),

	_effect("wide_cyan_red"		, r(133, SCR_RR_CR	), 0x00ff, 0	,1, 148	),
	_effect("wide_cyan_green"	, r(135, SCR_RG_CG	), 0x00ff, 0	,1, 240	),
	_effect("wide_cyan_blue"	, r(137, SCR_RB_CB	), 0x00ff, 0	,1, 237	),
	
	_effect("wide_green_red"	, r(140, SCR_GR_MR	), 0xff00, 8	,1, 138	),
	_effect("wide_green_green"	, r(142, SCR_GG_MG	), 0xff00, 8	,1, 231	),
	_effect("wide_green_blue"	, r(144, SCR_GB_MB	), 0xff00, 8	,1, 76	),

	_effect("wide_magenta_red"	, r(139, SCR_GR_MR	), 0x00ff, 0	,1, 210	),
	_effect("wide_magenta_green"	, r(141, SCR_GG_MG	), 0x00ff, 0	,1, 57	),
	_effect("wide_magenta_blue"	, r(143, SCR_GB_MB	), 0x00ff, 0	,1, 222	),
	
	_effect("wide_blue_red"		, r(146, SCR_BR_YR	), 0xff00, 8	,1, 38	),
	_effect("wide_blue_green"	, r(148, SCR_BG_YG	), 0xff00, 8	,1, 53	),
	_effect("wide_blue_blue"	, r(150, SCR_BB_YB	), 0xff00, 8	,1, 230	),

	_effect("wide_yellow_red"	, r(145, SCR_BR_YR	), 0x00ff, 0	,1, 248	),
	_effect("wide_yellow_green"	, r(147, SCR_BG_YG	), 0x00ff, 0	,1, 238	),
	_effect("wide_yellow_blue"	, r(149, SCR_BB_YB	), 0x00ff, 0	,1, 89	),
	
	_effect("wide_black_red"	, r(152, SCR_KR_WR	), 0x00ff, 0	,1, 0	),
	_effect("wide_black_green"	, r(154, SCR_KG_WG	), 0x00ff, 0	,1, 0	),
	_effect("wide_black_blue"	, r(156, SCR_KB_WB	), 0x00ff, 0	,1, 0	),

	_effect("wide_white_red"	, r(151, SCR_KR_WR	), 0xff00, 8	,1, 255	),
	_effect("wide_white_green"	, r(153, SCR_KG_WG	), 0xff00, 8	,1, 247	),
	_effect("wide_white_blue"	, r(155, SCR_KB_WB	), 0xff00, 8	,1, 247	),

	/* Greyscale gamma curve */
	
#ifndef MDNIE_LITE
	_effect("cc_channel_strength"	, CC_CHSEL_STR	, 0xffff, 0	,1, 128	),
	
	_effect("cc_0"			, CC_0		, 0x00ff, 0	,1, 0	),
	_effect("cc_16"			, CC_1		, 0xff00, 8	,1, 16	),
	_effect("cc_32"			, CC_2		, 0xff00, 8	,1, 32	),
	_effect("cc_48"			, CC_3		, 0xff00, 8	,1, 48	),
	_effect("cc_64"			, CC_4		, 0xff00, 8	,1, 64	),
	_effect("cc_80"			, CC_5		, 0xff00, 8	,1, 80	),
	_effect("cc_96"			, CC_6		, 0xff00, 8	,1, 96	),
	_effect("cc_112"		, CC_7		, 0xff00, 8	,1, 112	),
	_effect("cc_128"		, CC_8		, 0xff00, 8	,1, 128	),
	_effect("cc_144"		, CC_1		, 0x00ff, 0	,1, 144	),
	_effect("cc_160"		, CC_2		, 0x00ff, 0	,1, 160	),
	_effect("cc_176"		, CC_3		, 0x00ff, 0	,1, 176	),
	_effect("cc_192"		, CC_4		, 0x00ff, 0	,1, 192	),
	_effect("cc_208"		, CC_5		, 0x00ff, 0	,1, 208	),
	_effect("cc_224"		, CC_6		, 0x00ff, 0	,1, 224	),
	_effect("cc_240"		, CC_7		, 0x00ff, 0	,1, 240	),
	_effect("cc_255"		, CC_8		, 0x00ff, 0	,1, 255	),
#else
	_effect("cc_1b"			, 54		, 0x00ff, 0	,1, 0	),
	_effect("cc_1a"			, 55		, 0x00ff, 0	,1, 32	),
	_effect("cc_2b"			, 56		, 0x00ff, 0	,1, 0	),
	_effect("cc_2a"			, 57		, 0x00ff, 0	,1, 32	),
	_effect("cc_3b"			, 58		, 0x00ff, 0	,1, 0	),
	_effect("cc_3a"			, 59		, 0x00ff, 0	,1, 32	),
	_effect("cc_4b"			, 60		, 0x00ff, 0	,1, 0	),
	_effect("cc_4a"			, 61		, 0x00ff, 0	,1, 32	),
	_effect("cc_5b"			, 62		, 0x00ff, 0	,1, 2	),
	_effect("cc_5a"			, 63		, 0x00ff, 0	,1, 27	),
	_effect("cc_6b"			, 65		, 0x00ff, 0	,1, 27	),
	_effect("cc_6a"			, 66		, 0x00ff, 0	,1, 2	),
	_effect("cc_7b"			, 67		, 0x00ff, 0	,1, 27	),
	_effect("cc_7a"			, 68		, 0x00ff, 0	,1, 2	),
	_effect("cc_8b"			, 69		, 0x00ff, 0	,1, 27	),
	_effect("cc_8a"			, 70		, 0x00ff, 0	,1, 9	),
	
	_effect("cc_9b"			, 71		, 0x00ff, 0	,1, 166	),
	_effect("cc_9a"			, 72		, 0x00ff, 0	,1, 9	),
	_effect("cc_10b"		, 73		, 0x00ff, 0	,1, 166	),
	_effect("cc_10a"		, 74		, 0x00ff, 0	,1, 6	),
	_effect("cc_11b"		, 75		, 0x00ff, 0	,1, 166	),
	_effect("cc_11a"		, 76		, 0x00ff, 0	,1, 0	),
	_effect("cc_12b"		, 77		, 0x00ff, 0	,1, 166	),
	_effect("cc_12a"		, 78		, 0x00ff, 0	,1, 0	),
	_effect("cc_13b"		, 79		, 0x00ff, 0	,1, 32	),
	_effect("cc_13a"		, 80		, 0x00ff, 0	,1, 0	),
	_effect("cc_14b"		, 81		, 0x00ff, 0	,1, 32	),
	_effect("cc_14a"		, 82		, 0x00ff, 0	,1, 0	),
	_effect("cc_15b"		, 83		, 0x00ff, 0	,1, 32	),
	_effect("cc_15a"		, 84		, 0x00ff, 0	,1, 0	),
	_effect("cc_16b"		, 85		, 0x00ff, 0	,1, 32	),
	_effect("cc_16a"		, 86		, 0x00ff, 0	,1, 0	),
	
	_effect("cc_17b"		, 87		, 0x00ff, 0	,1, 32	),
	_effect("cc_17a"		, 88		, 0x00ff, 0	,1, 0	),
	_effect("cc_18b"		, 89		, 0x00ff, 0	,1, 32	),
	_effect("cc_18a"		, 90		, 0x00ff, 0	,1, 0	),
	_effect("cc_19b"		, 91		, 0x00ff, 0	,1, 32	),
	_effect("cc_19a"		, 92		, 0x00ff, 0	,1, 0	),
	_effect("cc_20b"		, 93		, 0x00ff, 0	,1, 32	),
	_effect("cc_20a"		, 94		, 0x00ff, 0	,1, 0	),
	_effect("cc_21b"		, 95		, 0x00ff, 0	,1, 32	),
	_effect("cc_21a"		, 96		, 0x00ff, 0	,1, 0	),
	_effect("cc_22b"		, 97		, 0x00ff, 0	,1, 32	),
	_effect("cc_22a"		, 98		, 0x00ff, 0	,1, 0	),
	_effect("cc_23b"		, 99		, 0x00ff, 0	,1, 32	),
	_effect("cc_23a"		, 100		, 0x00ff, 0	,1, 0	),
	_effect("cc_24b"		, 101		, 0x00ff, 0	,1, 255	),
	_effect("cc_24a"		, 102		, 0x00ff, 0	,1, 64	),
#endif
};

static int is_switch(unsigned int reg)
{
	switch(reg) {
		default:
			return false;
	}
}

static int is_hook_scenario(int scenario)
{
	return !!(reg_hook & (1 << scenario));
	/*
	switch (scenario) {
		case UI_MODE:
		case VIDEO_NORMAL_MODE:
		case GALLERY_MODE:
			return false;
		default:
			return true;
	}
	*/
}

static int effect_switch_hook(struct mdnie_effect *effect, mdnie_t regval)
{
	return effect->value ? !regval : regval;
}

static int secondary_hook(struct mdnie_effect *effect, int val)
{
	if (effect->abs)
		val = effect->regval;
	else
		val += effect->value;

	return val;
}

mdnie_t mdnie_reg_hook(unsigned short reg, mdnie_t value)
{
	struct mdnie_effect *effect = (struct mdnie_effect*)&mdnie_controls;
	int i;
	int tmp, original;
	mdnie_t regval;
	
	printk("mdnie: hook on: 0x%2X (%3d) val: 0x%2X (%3d)\n", reg, reg, value, value);

	original = value;

	if (!is_hook_scenario(g_mdnie->scenario))
		return value;

	for (i = 0; i < ARRAY_SIZE(mdnie_controls); i++) {
	    if (effect->reg == reg) {
#ifndef MDNIE_LITE
		if (sequence_hook)
			tmp = regval = effect->regval;
		else
			tmp = regval = (value & effect->mask) >> effect->shift;
#else
		tmp = regval = value;
#endif

		if (reg_hook) {
			if (is_switch(reg))
				tmp = effect_switch_hook(effect, regval);
			else
				tmp = secondary_hook(effect, tmp);

#ifndef MDNIE_LITE
			if (tmp > (effect->mask >> effect->shift))
				tmp = (effect->mask >> effect->shift);
#else
			if (tmp > (1 << (sizeof(mdnie_t) * 8)))
				tmp = 1 << (sizeof(mdnie_t) * 8);
#endif

			if (tmp < 0)
				tmp = 0;

			regval = (mdnie_t)tmp;
		}

#ifndef MDNIE_LITE
		value &= ~effect->mask;
		value |= regval << effect->shift;
#else
		value = regval;
#endif
		
#if DEBUG
		printk("mdnie: hook on: 0x%X val: 0x%2X -> 0x%2X effect: %3d -> %3d : %s \n",
			reg, original, value, original, value, effect->attribute.attr.name);
#endif
	    }
	    ++effect;
	}
	
	return value;
}

#ifndef MDNIE_LITE
mdnie_t *mdnie_sequence_hook(mdnie_t *seq)
{
	if(!sequence_hook || !is_hook_scenario(g_mdnie->scenario))
		return seq;

	return (mdnie_t *)&master_sequence;
}
#endif

static inline void mdnie_refresh(void)
{
	mdnie_update(g_mdnie);
}

/**** Sysfs ****/

static ssize_t show_mdnie_property(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mdnie_effect *effect = (struct mdnie_effect*)(attr);


	if (effect->abs) 
		return sprintf(buf, "%d\n", effect->regval); 
	else 
		return sprintf(buf, "%d\n", effect->value);
};

static ssize_t store_mdnie_property(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct mdnie_effect *effect = (struct mdnie_effect*)(attr);
	int val;
	
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	if (is_switch(effect->reg)) {
		effect->value = val;
	} else if(!effect->abs) {
#ifndef MDNIE_LITE
		if (val > (effect->mask >> effect->shift))
			val = (effect->mask >> effect->shift);

		if (val < -(effect->mask >> effect->shift))
			val = -(effect->mask >> effect->shift);
#else
		if (val > (1 << (sizeof(mdnie_t) * 8)))
			val = 1 << (sizeof(mdnie_t) * 8);
		
		if (val < -(1 << (sizeof(mdnie_t) * 8)))
			val = -(1 << (sizeof(mdnie_t) * 8));
#endif

		effect->value = val;
	} else { 
		effect->regval = val;
	}

	mdnie_refresh();

	return count;
};

#define MAIN_CONTROL(_name, _var, _callback) \
static ssize_t show_##_name(struct device *dev,					\
				    struct device_attribute *attr, char *buf)	\
{										\
	return sprintf(buf, "%d", _var);					\
};										\
static ssize_t store_##_name(struct device *dev,				\
				     struct device_attribute *attr,		\
				     const char *buf, size_t count)		\
{										\
	int val;								\
										\
	if(sscanf(buf, "%d", &val) != 1)					\
		return -EINVAL;							\
										\
	_var = val;								\
										\
	_callback();								\
										\
	return count;								\
};

MAIN_CONTROL(reg_hook, reg_hook, mdnie_refresh);
DEVICE_ATTR(reg_intercept, 0664, show_reg_hook, store_reg_hook);

#ifndef MDNIE_LITE
MAIN_CONTROL(sequence_intercept, sequence_hook, mdnie_refresh);
DEVICE_ATTR(sequence_intercept, 0664, show_sequence_intercept, store_sequence_intercept);
#endif

void init_mdnie_control(struct mdnie_info *mdnie) 
{
	int i, ret;
	struct mdnie_table *table;
	struct kobject *subdir;
	
#ifdef MDNIE_LITE
	cmds = kzalloc(sizeof(struct mdnie_seq_info) * table->seq[i].len, GFP_KERNEL);
	if (IS_ERR_OR_NULL(mdnie->dev)) {
		pr_err("failed to create control tables\n");
		return;
	}
	
	for (i = 0; table->seq[i].len; i++) {
		cmds[i].cmd = kzalloc(sizeof(mdnie_t) * sizeof(char), GFP_KERNEL);
		if (IS_ERR_OR_NULL(mdnie->dev)) {
			pr_err("failed to create sequences\n");
			return;
		}
	}
#endif

	subdir = kobject_create_and_add("controls", &mdnie->dev->kobj);

	for(i = 0; i < ARRAY_SIZE(mdnie_controls); i++) {
		ret = sysfs_create_file(subdir, &mdnie_controls[i].attribute.attr);
	}

	ret = sysfs_create_file(&mdnie->dev->kobj, &dev_attr_reg_intercept.attr);
#ifndef MDNIE_LITE
	ret = sysfs_create_file(&mdnie->dev->kobj, &dev_attr_sequence_intercept.attr);
#endif
	
	g_mdnie = mdnie;
}
