/*
 * sma1301.h -- sma1301 ALSA SoC Audio driver
 *
 * r011, 2019.05.29
 *
 * Copyright 2018 Silicon Mitus Corporation / Iron Device Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SMA1301_H
#define _SMA1301_H

/* I2C Device Address - 7bit */
#define  SMA1301_I2C_ADDR_LOW		0x1e
#define  SMA1301_I2C_ADDR_HIGH		0x3e

#define  SMA1301_EXTERNAL_CLOCK_19_2	0x00
#define  SMA1301_EXTERNAL_CLOCK_24_576	0x01
#define  SMA1301_PLL_CLKIN_MCLK			0x02
#define  SMA1301_PLL_CLKIN_BCLK			0x03

/*
 * SMA1301 Register Definition
 */

/* SMA1301 Register Addresses */
#define  SMA1301_00_SYSTEM_CTRL		0x00
#define  SMA1301_01_INPUT1_CTRL1	0x01
#define  SMA1301_02_INPUT1_CTRL2	0x02
#define  SMA1301_03_INPUT1_CTRL3	0x03
#define  SMA1301_04_INPUT1_CTRL4	0x04
/* 0x05 ~ 0x08 : Reserved */
#define  SMA1301_09_OUTPUT_CTRL		0x09
#define  SMA1301_0A_SPK_VOL			0x0a
/* 0x0B ~ 0x0D : Reserved */
#define  SMA1301_0E_MUTE_VOL_CTRL	0x0e
/* 0x0F : Reserved */
#define  SMA1301_10_SYSTEM_CTRL1	0x10
#define  SMA1301_11_SYSTEM_CTRL2	0x11
#define  SMA1301_12_SYSTEM_CTRL3	0x12
/* 0x13 : Reserved */
#define  SMA1301_14_MODULATOR		0x14
#define  SMA1301_15_BASS_SPK1		0x15
#define  SMA1301_16_BASS_SPK2		0x16
#define  SMA1301_17_BASS_SPK3		0x17
#define  SMA1301_18_BASS_SPK4		0x18
#define  SMA1301_19_BASS_SPK5		0x19
#define  SMA1301_1A_BASS_SPK6		0x1a
#define  SMA1301_1B_BASS_SPK7		0x1b
/* 0x1C ~ 0x22 : Reserved */
#define  SMA1301_23_COMP_LIM1		0x23
#define  SMA1301_24_COMP_LIM2		0x24
#define  SMA1301_25_COMP_LIM3		0x25
#define  SMA1301_26_COMP_LIM4		0x26
/* 0x27 ~ 0x2A : Reserved */
#define  SMA1301_2B_EQ_MODE			0x2b
#define  SMA1301_2C_EQ_GRAPHIC1		0x2c
#define  SMA1301_2D_EQ_GRAPHIC2		0x2d
#define  SMA1301_2E_EQ_GRAPHIC3		0x2e
#define  SMA1301_2F_EQ_GRAPHIC4		0x2f
#define  SMA1301_30_EQ_GRAPHIC5		0x30
/* 0x31 ~ 0x32 : Reserved */
#define  SMA1301_33_SDM_CTRL		0x33
/* 0x34 ~ 0x35 : Reserved */
#define  SMA1301_36_PROTECTION		0x36
#define  SMA1301_37_SLOPE_CTRL		0x37
/* 0x38 ~ 0x3A : Reserved */
#define  SMA1301_3B_TEST1			0x3b
#define  SMA1301_3C_TEST2			0x3c
#define  SMA1301_3D_TEST3			0x3d
#define  SMA1301_3E_ATEST1			0x3e
#define  SMA1301_3F_ATEST2			0x3f
/****** Parametric EQ  ***************/
/* Band1 */
#define	 SMA1301_40_EQ_CTRL1		0x40
#define	 SMA1301_41_EQ_CTRL2		0x41
#define	 SMA1301_42_EQ_CTRL3		0x42
#define	 SMA1301_43_EQ_CTRL4		0x43
#define	 SMA1301_44_EQ_CTRL5		0x44
#define	 SMA1301_45_EQ_CTRL6		0x45
#define	 SMA1301_46_EQ_CTRL7		0x46
#define	 SMA1301_47_EQ_CTRL8		0x47
#define	 SMA1301_48_EQ_CTRL9		0x48
#define	 SMA1301_49_EQ_CTRL10		0x49
#define	 SMA1301_4A_EQ_CTRL11		0x4a
#define	 SMA1301_4B_EQ_CTRL12		0x4b
#define	 SMA1301_4C_EQ_CTRL13		0x4c
#define	 SMA1301_4D_EQ_CTRL14		0x4d
#define	 SMA1301_4E_EQ_CTRL15		0x4e
/* Band2 */
#define	 SMA1301_4F_EQ_CTRL16		0x4f
#define	 SMA1301_50_EQ_CTRL17		0x50
#define	 SMA1301_51_EQ_CTRL18		0x51
#define	 SMA1301_52_EQ_CTRL19		0x52
#define	 SMA1301_53_EQ_CTRL20		0x53
#define	 SMA1301_54_EQ_CTRL21		0x54
#define	 SMA1301_55_EQ_CTRL22		0x55
#define	 SMA1301_56_EQ_CTRL23		0x56
#define	 SMA1301_57_EQ_CTRL24		0x57
#define	 SMA1301_58_EQ_CTRL25		0x58
#define	 SMA1301_59_EQ_CTRL26		0x59
#define	 SMA1301_5A_EQ_CTRL27		0x5a
#define	 SMA1301_5B_EQ_CTRL28		0x5b
#define	 SMA1301_5C_EQ_CTRL29		0x5c
#define	 SMA1301_5D_EQ_CTRL30		0x5d
/* Band3 */
#define	 SMA1301_5E_EQ_CTRL31		0x5e
#define	 SMA1301_5F_EQ_CTRL32		0x5f
#define	 SMA1301_60_EQ_CTRL33		0x60
#define	 SMA1301_61_EQ_CTRL34		0x61
#define	 SMA1301_62_EQ_CTRL35		0x62
#define	 SMA1301_63_EQ_CTRL36		0x63
#define	 SMA1301_64_EQ_CTRL37		0x64
#define	 SMA1301_65_EQ_CTRL38		0x65
#define	 SMA1301_66_EQ_CTRL39		0x66
#define	 SMA1301_67_EQ_CTRL40		0x67
#define	 SMA1301_68_EQ_CTRL41		0x68
#define	 SMA1301_69_EQ_CTRL42		0x69
#define	 SMA1301_6A_EQ_CTRL43		0x6a
#define	 SMA1301_6B_EQ_CTRL44		0x6b
#define	 SMA1301_6C_EQ_CTRL45		0x6c
/* Band4 */
#define	 SMA1301_6D_EQ_CTRL46		0x6d
#define	 SMA1301_6E_EQ_CTRL47		0x6e
#define	 SMA1301_6F_EQ_CTRL48		0x6f
#define	 SMA1301_70_EQ_CTRL49		0x70
#define	 SMA1301_71_EQ_CTRL50		0x71
#define	 SMA1301_72_EQ_CTRL51		0x72
#define	 SMA1301_73_EQ_CTRL52		0x73
#define	 SMA1301_74_EQ_CTRL53		0x74
#define	 SMA1301_75_EQ_CTRL54		0x75
#define	 SMA1301_76_EQ_CTRL55		0x76
#define	 SMA1301_77_EQ_CTRL56		0x77
#define	 SMA1301_78_EQ_CTRL57		0x78
#define	 SMA1301_79_EQ_CTRL58		0x79
#define	 SMA1301_7A_EQ_CTRL59		0x7a
#define	 SMA1301_7B_EQ_CTRL60		0x7b
/* Band5 */
#define	 SMA1301_7C_EQ_CTRL61		0x7c
#define	 SMA1301_7D_EQ_CTRL62		0x7d
#define	 SMA1301_7E_EQ_CTRL63		0x7e
#define	 SMA1301_7F_EQ_CTRL64		0x7f
#define	 SMA1301_80_EQ_CTRL65		0x80
#define	 SMA1301_81_EQ_CTRL66		0x81
#define	 SMA1301_82_EQ_CTRL67		0x82
#define	 SMA1301_83_EQ_CTRL68		0x83
#define	 SMA1301_84_EQ_CTRL69		0x84
#define	 SMA1301_85_EQ_CTRL70		0x85
#define	 SMA1301_86_EQ_CTRL71		0x86
#define	 SMA1301_87_EQ_CTRL72		0x87
#define	 SMA1301_88_EQ_CTRL73		0x88
#define	 SMA1301_89_EQ_CTRL74		0x89
#define	 SMA1301_8A_EQ_CTRL75		0x8a

#define	 SMA1301_8B_PLL_POST_N		0x8b
#define	 SMA1301_8C_PLL_N			0x8c
#define	 SMA1301_8D_PLL_F1			0x8d
#define	 SMA1301_8E_PLL_F2			0x8e
#define	 SMA1301_8F_PLL_F3			0x8f
/* 0x90 : Reserved */
#define  SMA1301_91_CLASS_G_CTRL	0x91
#define  SMA1301_92_FDPEC_CTRL		0x92
/* 0x93 : Reserved */
#define  SMA1301_94_BOOST_CTRL1		0x94
#define  SMA1301_95_BOOST_CTRL2		0x95
#define  SMA1301_96_BOOST_CTRL3		0x96
#define  SMA1301_97_BOOST_CTRL4		0x97
/* 0x98 ~ 0xA1 : Reserved */
#define	 SMA1301_A2_TOP_MAN1		0xa2
#define	 SMA1301_A3_TOP_MAN2		0xa3
#define	 SMA1301_A4_TOP_MAN3		0xa4
/* 0xA5 ~ 0xF9 : Reserved */
#define	 SMA1301_FA_STATUS1			0xfa
#define	 SMA1301_FB_STATUS2			0xfb
/* 0xFC : Reserved */
#define	 SMA1301_FD_STATUS3			0xfd
#define	 SMA1301_FF_DEVICE_INDEX	0xff

/* SMA1301 Registers Bit Fields */

/* SYSTEM_CTRL : 0x00 */
#define POWER_MASK (1<<0)
#define POWER_ON (1<<0)
#define POWER_OFF (0<<0)

#define CLKSYSTEM_MASK	(7<<5)
#define EXT_19_2	(3<<5)
#define EXT_24_576	(4<<5)

/* INTPUT CTRL1 : 0x01 */
#define MASTER_SLAVE_MASK (1<<7)
#define SLAVE_MODE	(0<<7)
#define MASTER_MODE	(1<<7)

#define I2S_MODE_MASK	(7<<4)
#define STANDARD_I2S	(0<<4)
#define LJ		(1<<4)
#define RJ_16BIT	(4<<4)
#define RJ_18BIT	(5<<4)
#define RJ_20BIT	(6<<4)
#define RJ_24BIT	(7<<4)

#define LEFTPOL_MASK	(1<<3)
#define LOW_FIRST_CH	(0<<3)
#define HIGH_FIRST_CH	(1<<3)

#define SCK_RISING_FALLING_MASK	(1<<2)
#define FALLING_EDGE		(0<<2)
#define RISING_EDGE		(1<<2)

/* INTPUT CTRL2 : 0x02 */
#define IMODE_MASK (3<<6)
#define I2S	(0<<6)
#define PCM_SHORT (1<<6)
#define PCM_LONG (2<<6)

#define RIGHT_FIRST_MASK (1<<5)
#define LEFT_NORMAL (0<<5)
#define RIGHT_INVERTED (1<<5)

#define PCM_ALAW_MASK (1<<4)
#define PCM_U_DECODING (0<<4)
#define PCM_A_DECODING (1<<4)

#define PCM_COMP_MASK (1<<3)
#define PCM_LINEAR (0<<3)
#define PCM_COMPANDING (1<<3)

#define INPUTSEL_MASK (1<<2)
#define PCM_8KHZ (0<<2)
#define PCM_16KHZ (1<<2)

#define PCM_STEREO_MASK (1<<1)
#define PCM_MONO (0<<1)
#define PCM_STEREO (1<<1)

#define PCM_DL_MASK (1<<0)
#define PCM_8BIT (0<<0)
#define PCM_16BIT (1<<0)

/* INTPUT CTRL3 : 0x03 */
#define BP_SRC_MASK (1<<4)
#define BP_SRC_NORMAL (0<<4)
#define BP_SRC_BYPASS (1<<4)

#define PCM_N_SLOT_MASK (15<<0)
#define PCM_N_SLOT1 (0<<0)
#define PCM_N_SLOT2 (1<<0)
#define PCM_N_SLOT3 (2<<0)
#define PCM_N_SLOT4 (3<<0)
#define PCM_N_SLOT5 (4<<0)
#define PCM_N_SLOT6 (5<<0)
#define PCM_N_SLOT7 (6<<0)
#define PCM_N_SLOT8 (7<<0)
#define PCM_N_SLOT9 (8<<0)
#define PCM_N_SLOT10 (9<<0)
#define PCM_N_SLOT11 (10<<0)
#define PCM_N_SLOT12 (11<<0)
#define PCM_N_SLOT13 (12<<0)
#define PCM_N_SLOT14 (13<<0)
#define PCM_N_SLOT15 (14<<0)
#define PCM_N_SLOT16 (15<<0)

/* INTPUT CTRL4 : 0x04 */
#define PCM1_SLOT_MASK (15<<4)
#define PCM1_SLOT1 (0<<4)
#define PCM1_SLOT2 (1<<4)
#define PCM1_SLOT3 (2<<4)
#define PCM1_SLOT4 (3<<4)
#define PCM1_SLOT5 (4<<4)
#define PCM1_SLOT6 (5<<4)
#define PCM1_SLOT7 (6<<4)
#define PCM1_SLOT8 (7<<4)
#define PCM1_SLOT9 (8<<4)
#define PCM1_SLOT10 (9<<4)
#define PCM1_SLOT11 (10<<4)
#define PCM1_SLOT12 (11<<4)
#define PCM1_SLOT13 (12<<4)
#define PCM1_SLOT14 (13<<4)
#define PCM1_SLOT15 (14<<4)
#define PCM1_SLOT16 (15<<4)

#define PCM2_SLOT_MASK (15<<0)
#define PCM2_SLOT1 (0<<0)
#define PCM2_SLOT2 (1<<0)
#define PCM2_SLOT3 (2<<0)
#define PCM2_SLOT4 (3<<0)
#define PCM2_SLOT5 (4<<0)
#define PCM2_SLOT6 (5<<0)
#define PCM2_SLOT7 (6<<0)
#define PCM2_SLOT8 (7<<0)
#define PCM2_SLOT9 (8<<0)
#define PCM2_SLOT10 (9<<0)
#define PCM2_SLOT11 (10<<0)
#define PCM2_SLOT12 (11<<0)
#define PCM2_SLOT13 (12<<0)
#define PCM2_SLOT14 (13<<0)
#define PCM2_SLOT15 (14<<0)
#define PCM2_SLOT16 (15<<0)

/* OUTPUT CTRL : 0x09 */
#define PORT_CONFIG_MASK (3<<5)
#define INPUT_PORT_ONLY (0<<5)
#define OUTPUT_PORT_ENABLE (2<<5)

#define PORT_OUT_FORMAT_MASK (3<<3)
#define I2S_32SCK (0<<3)
#define I2S_64SCK (1<<3)
#define PCM_SHORT_128FS (2<<3)

#define PORT_OUT_SEL_MASK (7<<0)
#define OUT_SEL_DISABLE (0<<0)
#define FORMAT_CONVERTER (1<<0)
#define MIXER_OUTPUT (2<<0)
#define SPEAKER_PATH (3<<0)

/* MUTE_VOL_CTRL : 0x0E */
#define VOL_SLOPE_MASK (3<<6)
#define VOL_SLOPE_OFF (0<<6)
#define VOL_SLOPE_SLOW (1<<6)
#define VOL_SLOPE_MID (2<<6)
#define VOL_SLOPE_FAST (3<<6)

#define MUTE_SLOPE_MASK (3<<4)
#define MUTE_SLOPE_OFF (0<<4)
#define MUTE_SLOPE_SLOW (1<<4)
#define MUTE_SLOPE_MID (2<<4)
#define MUTE_SLOPE_FAST (3<<4)

#define SPK_MUTE_MASK (1<<0)
#define SPK_MUTE (1<<0)
#define SPK_UNMUTE (0<<0)

/* SYSTEM_CTRL1 :0x10 */
#define SPK_MODE_MASK (7<<2)
#define SPK_OFF (0<<2)
#define SPK_MONO (1<<2)
#define SPK_STEREO (4<<2)

/* SYSTEM_CTRL2 : 0x11 */
#define SPK_EQ_MASK (1<<7)
#define SPK_EQ_BYP (0<<7)
#define SPK_EQ_EN (1<<7)
#define SPK_BS_MASK (1<<6)
#define SPK_BS_BYP (0<<6)
#define SPK_BS_EN (1<<6)
#define SPK_LIM_MASK (1<<5)
#define SPK_LIM_BYP (0<<5)
#define SPK_LIM_EN (1<<5)

#define LR_DATA_SW_MASK (1<<4)
#define LR_DATA_SW_NORMAL (0<<4)
#define LR_DATA_SW_SWAP (1<<4)

#define MONOMIX_MASK (1<<0)
#define MONOMIX_OFF (0<<0)
#define MONOMIX_ON (1<<0)

/* SYSTEM_CTRL3 : 0x12 */
#define INPUT_MASK (3<<6)
#define INPUT_0_DB (0<<6)
#define INPUT_M6_DB (1<<6)
#define INPUT_M12_DB (2<<6)
#define INPUT_INFI_DB (3<<6)
#define INPUT_R_MASK (3<<4)
#define INPUT_R_0_DB (0<<4)
#define INPUT_R_M6_DB (1<<4)
#define INPUT_R_M12_DB (2<<4)
#define INPUT_R_INFI_DB (3<<4)

/* Modulator : 0x14 */
#define SPK_HYSFB_MASK (3<<6)
#define HYSFB_625K (0<<6)
#define HYSFB_414K (1<<6)
#define HYSFB_297K (2<<6)
#define HYSFB_226K (3<<6)
#define SPK_BDELAY_MASK (63<<0)

/* EQ_MODE : 0x2B */
#define EQ_MODE_MASK (7<<0)
#define USER_DEFINED (0<<0)
#define CLASSIC (1<<1)
#define ROCK_POP (2<<0)
#define JAZZ (3<<0)
#define RNB (4<<0)
#define DANCE (5<<0)
#define SPEECH (6<<0)
#define PARAMETRIC (7<<0)

/* Graphic EQ_1 : 0x2C */
#define EQ_BAND1_BYPASS_MASK (1<<5)
#define EQ_BAND1_OPERATION (0<<5)
#define EQ_BAND1_BYPASS (1<<5)

/* Graphic EQ_2 : 0x2D */
#define EQ_BAND2_BYPASS_MASK (1<<5)
#define EQ_BAND2_OPERATION (0<<5)
#define EQ_BAND2_BYPASS (1<<5)

/* Graphic EQ_3 : 0x2E */
#define EQ_BAND3_BYPASS_MASK (1<<5)
#define EQ_BAND3_OPERATION (0<<5)
#define EQ_BAND3_BYPASS (1<<5)

/* Graphic EQ_4 : 0x2F */
#define EQ_BAND4_BYPASS_MASK (1<<5)
#define EQ_BAND4_OPERATION (0<<5)
#define EQ_BAND4_BYPASS (1<<5)

/* Graphic EQ_5 : 0x30 */
#define EQ_BAND5_BYPASS_MASK (1<<5)
#define EQ_BAND5_OPERATION (0<<5)
#define EQ_BAND5_BYPASS (1<<5)

/* SDM CONTROL : 0x33 */
#define SDM_Q_SEL_MASK (1<<2)
#define QUART_SEL_1_DIV_4 (0<<2)
#define QUART_SEL_1_DIV_8 (1<<2)

/* PROTECTION : 0x36 */
#define EDGE_DIS_MASK (1<<7)
#define EDGE_DIS_ENABLE (0<<7)
#define EDGE_DIS_DISABLE (1<<7)

#define JITTER_DIS_MASK (1<<4)
#define SRC_JITTER_ADD (0<<4)
#define SRC_JITTER_DISABLE (1<<4)

#define SPK_OCP_DIS_MASK (1<<3)
#define SPK_OCP_ENABLE (0<<3)
#define SPK_OCP_DISABLE (1<<3)

#define OCP_MODE_MASK (1<<2)
#define AUTO_RECOVER (0<<2)
#define SHUT_DOWN_PERMANENT (1<<2)

#define OTP_MODE_MASK (3<<0)
#define OTP_MODE_DISABLE (0<<0)
#define IG_THR1_SHUT_THR2 (1<<0)
#define REC_THR1_SHUT_THR2 (2<<0)
#define SHUT_THR1_SHUT_THR2 (3<<0)

/* TEST2 : 0x3C */
#define SPK_HSDM_BP_MASK (1<<4)
#define SPK_HSDM_ENABLE (0<<4)
#define SPK_HSDM_BYPASS (1<<4)

#define SDM_SYNC_DIS_MASK (1<<5)
#define SDM_SYNC_NORMAL (0<<5)
#define SDM_SYNC_DISABLE (1<<5)

/* ATEST2 : 0x3F */
#define SPK_OUT_FREQ_MASK (1<<2)
#define SPK_OUT_FREQ_370K (0<<2)
#define SPK_OUT_FREQ_470K (1<<2)

#define LOW_POWER_MODE_MASK (1<<3)
#define LOW_POWER_MODE_DISABLE (0<<3)
#define LOW_POWER_MODE_ENABLE (1<<3)

#define THERMAL_ADJUST_MASK (3<<5)
#define THERMAL_150_110 (0<<5)
#define THERMAL_160_120 (1<<5)
#define THERMAL_140_100 (2<<5)

#define FAST_OFF_DRIVE_SPK_MASK (1<<0)
#define FAST_OFF_DRIVE_SPK_DISABLE (0<<0)
#define FAST_OFF_DRIVE_SPK_ENABLE (1<<0)

/* PLL_LDO_CTRL,PLL_POST_N : 0x8B */
#define PLL_EN_MASK (1<<5)
#define PLL_EN_DISABLE (0<<5)
#define PLL_EN_ENABLE (1<<5)

/* FDPEC CONTROL : 0x92 */
#define FLT_VDD_GAIN_MASK (15<<4)
#define FLT_VDD_GAIN_2P40 (0<<4)
#define FLT_VDD_GAIN_2P45 (1<<4)
#define FLT_VDD_GAIN_2P50 (2<<4)
#define FLT_VDD_GAIN_2P55 (3<<4)
#define FLT_VDD_GAIN_2P60 (4<<4)
#define FLT_VDD_GAIN_2P65 (5<<4)
#define FLT_VDD_GAIN_2P70 (6<<4)
#define FLT_VDD_GAIN_2P75 (7<<4)
#define FLT_VDD_GAIN_2P80 (8<<4)
#define FLT_VDD_GAIN_2P85 (9<<4)
#define FLT_VDD_GAIN_2P90 (10<<4)
#define FLT_VDD_GAIN_2P95 (11<<4)
#define FLT_VDD_GAIN_3P00 (12<<4)
#define FLT_VDD_GAIN_3P05 (13<<4)
#define FLT_VDD_GAIN_3P10 (14<<4)
#define FLT_VDD_GAIN_3P15 (15<<4)

#define DIS_FCHG_MASK (1<<2)
#define EN_FAST_CHARGE (0<<2)
#define DIS_FAST_CHARGE (1<<2)

/* BOOST_CONTROL4 : 0x92 */
#define TRM_VBST_MASK (7<<2)
#define TRM_VBST_5P5 (0<<2)
#define TRM_VBST_5P6 (1<<2)
#define TRM_VBST_5P7 (2<<2)
#define TRM_VBST_5P8 (3<<2)
#define TRM_VBST_5P9 (4<<2)
#define TRM_VBST_6P0 (5<<2)
#define TRM_VBST_6P1 (6<<2)
#define TRM_VBST_6P2 (7<<2)

/* TOP_MAN1 : 0xA2 */
#define PLL_LOCK_SKIP_MASK (1<<7)
#define PLL_LOCK_ENABLE (0<<7)
#define PLL_LOCK_DISABLE (1<<7)

#define PLL_PD_MASK (1<<6)
#define PLL_OPERATION (0<<6)
#define PLL_PD (1<<6)

#define MCLK_SEL_MASK (1<<5)
#define PLL_CLK (0<<5)
#define EXTERNAL_CLK (1<<5)

#define PLL_REF_CLK1_MASK (1<<4)
#define REF_EXTERNAL_CLK (0<<4)
#define REF_INTERNAL_OSC (1<<4)

#define PLL_REF_CLK2_MASK (1<<3)
#define PLL_REF_CLK1 (0<<3)
#define PLL_SCK (1<<3)

#define DAC_DN_CONV_MASK (1<<2)
#define DAC_DN_CONV_DISABLE (0<<2)
#define DAC_DN_CONV_ENABLE (1<<2)

#define SDO_IO_MASK (1<<1)
#define HIGH_Z_LRCK_H (0<<1)
#define HIGH_Z_LRCK_L (1<<1)

#define MAS_IO_MASK (1<<0)
#define MAS_IO_SLAVE (0<<0)
#define MAS_IO_MASTER (1<<0)

/* TOP_MAN2 : 0xA3 */
#define MON_OSC_PLL_MASK (1<<7)
#define PLL_SDO (0<<7)
#define PLL_OSC (1<<7)

#define TEST_CLKO_EN_MASK (1<<6)
#define NORMAL_SDO (0<<6)
#define CLK_OUT_SDO (1<<6)

#define PLL_SDM_PD_MASK (1<<5)
#define SDM_ON (0<<5)
#define SDM_OFF (1<<5)

#define CLK_MON_TRIM_MASK (1<<4)
#define TIME_2_USEC (0<<4)
#define TIME_4_USEC (1<<4)

#define SDO_OUTPUT_MASK (1<<3)
#define NORMAL_OUT (0<<3)
#define HIGH_Z_OUT (1<<3)

#define DIS_SDO_IO_MASK (1<<2)
#define SDO_NORMAL (0<<2)
#define SDO_OUT_ONLY (1<<2)

#define CLOCK_MON_MASK (1<<1)
#define CLOCK_MON (0<<1)
#define CLOCK_NOT_MON (1<<1)

#define OSC_PD_MASK (1<<0)
#define NORMAL_OPERATION_OSC (0<<0)
#define POWER_DOWN_OSC (1<<0)

/* TOP_MAN3 0xA4 */
#define O_FORMAT_MASK (7<<5)
#define O_FMT_RJ (0<<5)
#define O_FMT_LJ (1<<5)
#define O_FMT_I2S (2<<5)
#define O_FMT_SPI (3<<5)
#define O_FMT_PCM_SHORT (4<<5)
#define O_FMT_PCM_LONG (5<<5)

#define SCK_RATE_MASK (1<<3)
#define SCK_64FS (0<<3)
#define SCK_32FS (2<<3)

#define LRCK_POL_MASK (1<<0)
#define L_VALID (0<<0)
#define R_VALID (1<<0)

/* STATUS1 : 0xFA */
#define OT1_OK_STATUS (1<<7)
#define OT2_OK_STATUS (1<<6)
#define REV_NUM_STATUS (7<<3)
#define REV_NUM_ES0 (4<<3)
#define REV_NUM_ES1 (5<<3)

/* STATUS2 : 0xFB */
#define OCP_SPK_STATUS (1<<5)
#define OCP_BST_STATUS (1<<4)
#define CLK_FAULT_STATUS (1<<0)

/* VERSION : 0xFF */
#define DEVICE_INDEX (10<<0)

#endif
