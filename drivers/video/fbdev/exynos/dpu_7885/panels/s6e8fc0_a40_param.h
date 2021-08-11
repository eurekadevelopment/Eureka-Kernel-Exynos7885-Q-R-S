#ifndef __S6E8FC0_PARAM_H__
#define __S6E8FC0_PARAM_H__

#include <linux/types.h>
#include <linux/kernel.h>

#define EXTEND_BRIGHTNESS	365
#define UI_MAX_BRIGHTNESS	255
#define UI_DEFAULT_BRIGHTNESS	128

#define NORMAL_TEMPERATURE	25	/* 25 degrees Celsius */

#define ACL_CMD_CNT				((u16)ARRAY_SIZE(SEQ_ACL_OFF))
#define OPR_CMD_CNT				((u16)ARRAY_SIZE(SEQ_ACL_OPR_OFF))
#define HBM_CMD_CNT				((u16)ARRAY_SIZE(SEQ_HBM_OFF))
#define ELVSS_CMD_CNT				((u16)ARRAY_SIZE(SEQ_ELVSS_SET))

#define LDI_REG_BRIGHTNESS			0x51
#define LDI_REG_ID				0x04
#define LDI_REG_COORDINATE			0xA1
#define LDI_REG_DATE				LDI_REG_COORDINATE
#define LDI_REG_MANUFACTURE_INFO		LDI_REG_COORDINATE
#define LDI_REG_CHIP_ID				0xD6

/* len is read length */
#define LDI_LEN_ID				3
#define LDI_LEN_COORDINATE			4
#define LDI_LEN_DATE				7
#define LDI_LEN_MANUFACTURE_INFO		20
#define LDI_LEN_CHIP_ID				5
#define LDI_LEN_ELVSS				(ELVSS_CMD_CNT - 1)

/* offset is position including addr, not only para */
#define LDI_OFFSET_HBM		1
#define LDI_OFFSET_ELVSS_1	4		/* BFh 4th para: ELVSS */
#define LDI_OFFSET_ELVSS_2	1		/* BFh 1th para: TSET */

#define LDI_GPARA_COORDINATE			0	/* A1h 1st Para: x, y */
#define LDI_GPARA_DATE				4	/* A1h 5th Para: [D7:D4]: Year */
#define LDI_GPARA_MANUFACTURE_INFO		11	/* A1h 12th Para: [D7:D4]:Site */

struct bit_info {
	unsigned int reg;
	unsigned int len;
	char **print;
	unsigned int expect;
	unsigned int offset;
	unsigned int g_para;
	unsigned int invert;
	unsigned int mask;
	unsigned int result;
};

enum {
	LDI_BIT_ENUM_05,	LDI_BIT_ENUM_RDNUMED = LDI_BIT_ENUM_05,
	LDI_BIT_ENUM_0A,	LDI_BIT_ENUM_RDDPM = LDI_BIT_ENUM_0A,
	LDI_BIT_ENUM_0E,	LDI_BIT_ENUM_RDDSM = LDI_BIT_ENUM_0E,
	LDI_BIT_ENUM_0F,	LDI_BIT_ENUM_RDDSDR = LDI_BIT_ENUM_0F,
	LDI_BIT_ENUM_EE,	LDI_BIT_ENUM_ESDERR = LDI_BIT_ENUM_EE,
	LDI_BIT_ENUM_MAX
};

static char *LDI_BIT_DESC_05[BITS_PER_BYTE] = {
	[0 ... 6] = "number of corrupted packets",
	[7] = "overflow on number of corrupted packets",
};

static char *LDI_BIT_DESC_0A[BITS_PER_BYTE] = {
	[2] = "Display is Off",
	[7] = "Booster has a fault",
};

static char *LDI_BIT_DESC_0E[BITS_PER_BYTE] = {
	[0] = "Error on DSI",
};

static char *LDI_BIT_DESC_0F[BITS_PER_BYTE] = {
	[7] = "Register Loading Detection",
};

static char *LDI_BIT_DESC_EE[BITS_PER_BYTE] = {
	[2] = "VLIN3 error",
	[3] = "ELVDD error",
	[6] = "VLIN1 error",
};

static struct bit_info ldi_bit_info_list[LDI_BIT_ENUM_MAX] = {
	[LDI_BIT_ENUM_05] = {0x05, 1, LDI_BIT_DESC_05, 0x00, },
	[LDI_BIT_ENUM_0A] = {0x0A, 1, LDI_BIT_DESC_0A, 0x9F, .invert = (BIT(2) | BIT(7)), },
	[LDI_BIT_ENUM_0E] = {0x0E, 1, LDI_BIT_DESC_0E, 0x00, },
	[LDI_BIT_ENUM_0F] = {0x0F, 1, LDI_BIT_DESC_0F, 0x80, .invert = (BIT(7)), },
	[LDI_BIT_ENUM_EE] = {0xEE, 1, LDI_BIT_DESC_EE, 0x00, .offset = 1, },
};

#if defined(CONFIG_DISPLAY_USE_INFO)
#define LDI_LEN_RDNUMED		1		/* DPUI_KEY_PNDSIE: Read Number of the Errors on DSI */
#define LDI_PNDSIE_MASK		(GENMASK(7, 0))

/*
 * ESD_ERROR[2] = VLIN3 error is occurred by ESD.
 * ESD_ERROR[3] = ELVDD error is occurred by ESD.
 * ESD_ERROR[6] = VLIN1 error is occurred by ESD
 */
#define LDI_LEN_ESDERR		1		/* DPUI_KEY_PNELVDE, DPUI_KEY_PNVLI1E, DPUI_KEY_PNVLO3E, DPUI_KEY_PNESDE */
#define LDI_PNELVDE_MASK	(BIT(3))	/* ELVDD error */
#define LDI_PNVLI1E_MASK	(BIT(6))	/* VLIN1 error */
#define LDI_PNVLO3E_MASK	(BIT(2))	/* VLIN3 error */
#define LDI_PNESDE_MASK		(BIT(2) | BIT(3) | BIT(6))

#define LDI_LEN_RDDSDR		1		/* DPUI_KEY_PNSDRE: Read Display Self-Diagnostic Result */
#define LDI_PNSDRE_MASK		(BIT(7))	/* D7: REG_DET: Register Loading Detection */
#endif

struct lcd_seq_info {
	unsigned char	*cmd;
	unsigned int	len;
	unsigned int	sleep;
};

static unsigned char SEQ_SLEEP_OUT[] = {
	0x11
};

static unsigned char SEQ_SLEEP_IN[] = {
	0x10
};

static unsigned char SEQ_DISPLAY_ON[] = {
	0x29
};

static unsigned char SEQ_DISPLAY_OFF[] = {
	0x28
};

static unsigned char SEQ_TEST_KEY_ON_F0[] = {
	0xF0,
	0x5A, 0x5A
};

static unsigned char SEQ_TEST_KEY_OFF_F0[] = {
	0xF0,
	0xA5, 0xA5
};

static unsigned char SEQ_TEST_KEY_ON_FC[] = {
	0xFC,
	0x5A, 0x5A
};

static unsigned char SEQ_TEST_KEY_OFF_FC[] = {
	0xFC,
	0xA5, 0xA5
};

static unsigned char SEQ_ERR_FG_SET[] = {
	0xED,
	0x00, 0x4C, 0x40
};

/* Table 3*/
static unsigned char SEQ_GLB_PARAM_2[] = {
	0xB0,
	0x02, 0xF2
};

static unsigned char SEQ_VFP[] = {
	0xF2,
	0x18
};

static unsigned char SEQ_GLB_PARAM_3[] = {
	0xB0,
	0x07, 0xF6
};

static unsigned char SEQ_LTPS1[] = {
	0xF6,
	0x87, 0x2D
};

static unsigned char SEQ_GLB_PARAM_4[] = {
	0xB0,
	0x17, 0xBE
};

static unsigned char SEQ_LTPS2[] = {
	0xBE,
	0x5C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x22, 0x31, 0x52, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x0B, 0x7D
};

static unsigned char SEQ_LTPS_UPDATE[] = {
	0xF7,
	0x03
};

/* Table 4*/
static unsigned char SEQ_EDGE_DIMMING[] = {
	0xB2,
	0x0A, 0x04, 0xD8, 0xD8, 0xBA, 0xBA /* 5th~6th 0xBA, 0xBA - 50% */
};

/* Table 5*/
static unsigned char SEQ_GLB_PARAM_5[] = {
	0xB0,
	0x04, 0xBF
};

static unsigned char SEQ_DIM_SPEED[] = {
	0xBF,
	0x04 /* 4 Frame */
};

static unsigned char SEQ_ELVSS_SET[] = {
	0xBF,
	0x19,		/* 1st para TSET */
	0x0D, 0x80,
	0xC4,		/* 4th para ELVSS */
	0x04,		/* 5th para 4 frame dim speed */
};

static unsigned char SEQ_HBM_ON[] = {
	0x53,
	0xE8,
};

static unsigned char SEQ_HBM_OFF[] = {
	0x53,
	0x28,
};

static unsigned char SEQ_ACL_OPR_OFF[] = {
	0xC1,
	0x41,		/* C1h 1st Para: 0x41 = 16 Frame Avg at ACL Off */
	0x26, 0x68, 0x15, 0x55, 0x55, 0x55, 0x0F, 0x1B, 0x18,
	0x47, 0x02,
	0x61, 0x28,	/* C1h 13th ~ 14th Para: 0x61,0x28 = ACL 15% */
	0x4A,
	0x42, 0x64,	/* C1h 16th ~ 17th Para: 0x42, 0x64 = ACL start step 60% */
	0x20		/* 18th para Dimming 32 frame */
};

static unsigned char SEQ_ACL_OPR_08P[] = {
	0xC1,
	0x51,		/* C1h 1st Para: 0x51 = 32 Frame Avg at ACL On */
	0x26, 0x68, 0x15, 0x55, 0x55, 0x55, 0x0F, 0x1B, 0x18,
	0x47, 0x02,
	0x60, 0x98,	/* C1h 13th ~ 14th Para: 0x60, 0x98 = ACL 8% */
	0x4A,
	0x41, 0xFC,	/* C1h 16th ~ 17th Para: 0x41,0xFC = ACL start step 50% */
	0x20		/* 18th para Dimming 32 frame */
};

static unsigned char SEQ_ACL_OPR_15P[] = {
	0xC1,
	0x51,		/* C1h 1st Para: 0x51 = 32 Frame Avg at ACL On */
	0x26, 0x68, 0x15, 0x55, 0x55, 0x55, 0x0F, 0x1B, 0x18,
	0x47, 0x02,
	0x61, 0x28,	/* C1h 13th ~ 14th Para: 0x61,0x28 = ACL 15% */
	0x4A,
	0x41, 0xFC,	/* C1h 16th ~ 17th Para: 0x41,0xFC = ACL start step 50% */
	0x20		/* 18th para Dimming 32 frame */
};

static unsigned char SEQ_ACL_OFF[] = {
	0x55,
	0x00	/* 0x00 : ACL OFF */
};

static unsigned char SEQ_ACL_ON[] = {
	0x55,
	0x02	/* 0x02 : ACL ON */
};

enum {
	ACL_STATUS_OFF,
	ACL_STATUS_ON,
	ACL_STATUS_MAX
};

enum {
	OPR_STATUS_OFF,
	OPR_STATUS_08P,
	OPR_STATUS_15P,
	OPR_STATUS_MAX
};

enum {
	TEMP_ABOVE_MINUS_00_DEGREE,	/* T > 0 */
	TEMP_ABOVE_MINUS_15_DEGREE,	/* -15 < T <= 0 */
	TEMP_BELOW_MINUS_15_DEGREE,	/* T <= -15 */
	TEMP_MAX
};

enum {
	HBM_STATUS_OFF,
	HBM_STATUS_ON,
	HBM_STATUS_MAX
};

static unsigned char *HBM_TABLE[HBM_STATUS_MAX] = {SEQ_HBM_OFF, SEQ_HBM_ON};
static unsigned char *ACL_TABLE[ACL_STATUS_MAX] = {SEQ_ACL_OFF, SEQ_ACL_ON};
static unsigned char *OPR_TABLE[OPR_STATUS_MAX] = {SEQ_ACL_OPR_OFF, SEQ_ACL_OPR_08P, SEQ_ACL_OPR_15P};

/* platform brightness <-> acl opr and percent */
static unsigned int brightness_opr_table[ACL_STATUS_MAX][EXTEND_BRIGHTNESS + 1] = {
	{
		[0 ... EXTEND_BRIGHTNESS]			= OPR_STATUS_OFF,
	}, {
		[0 ... UI_MAX_BRIGHTNESS]			= OPR_STATUS_15P,
		[UI_MAX_BRIGHTNESS + 1 ... EXTEND_BRIGHTNESS]	= OPR_STATUS_08P
	}
};

/* platform brightness <-> gamma level */
static unsigned int brightness_table[EXTEND_BRIGHTNESS + 1] = {
	2,
	4, 7, 12, 14, 17, 19, 24, 27, 29, 32,
	37, 39, 42, 44, 48, 52, 56, 59, 64, 66,
	69, 74, 76, 81, 84, 89, 91, 96, 99, 103,
	106, 108, 113, 116, 121, 123, 128, 131, 136, 138,
	143, 146, 148, 153, 155, 158, 163, 165, 170, 173,
	178, 180, 185, 188, 193, 195, 198, 202, 205, 210,
	212, 215, 220, 222, 227, 230, 235, 237, 242, 245,
	250, 252, 254, 259, 262, 267, 269, 274, 277, 282,
	284, 287, 292, 294, 299, 301, 304, 309, 311, 316,
	319, 324, 326, 331, 334, 339, 341, 344, 349, 351,
	356, 358, 361, 366, 368, 373, 376, 381, 383, 388,
	391, 393, 398, 400, 405, 408, 413, 415, 420, 423,
	428, 430, 433, 438, 440, 445, 448, 450, 455, 462,
	465, 469, 474, 479, 484, 489, 494, 496, 501, 506,
	510, 515, 520, 525, 527, 532, 537, 542, 547, 552,
	554, 561, 566, 568, 573, 578, 583, 588, 593, 595,
	600, 605, 610, 614, 619, 624, 626, 631, 636, 641,
	646, 651, 656, 660, 665, 668, 672, 677, 682, 687,
	692, 697, 699, 704, 709, 714, 718, 723, 726, 730,
	735, 740, 745, 750, 755, 759, 764, 767, 772, 776,
	781, 786, 791, 796, 798, 803, 808, 813, 817, 822,
	827, 830, 837, 839, 844, 849, 854, 859, 863, 866,
	871, 876, 880, 885, 890, 895, 897, 902, 907, 912,
	917, 921, 926, 929, 936, 938, 943, 948, 953, 958,
	963, 967, 970, 975, 979, 984, 989, 994, 996, 1001,
	1006, 1011, 1016, 1021, 1023, 4, 9, 11, 16, 20,
	22, 27, 31, 34, 38, 43, 45, 49, 54, 54,
	61, 63, 67, 72, 74, 79, 83, 85, 90, 92,
	97, 101, 103, 108, 110, 114, 119, 123, 126, 130,
	135, 137, 141, 146, 148, 153, 155, 159, 164, 166,
	171, 175, 177, 182, 184, 189, 193, 195, 200, 202,
	206, 211, 213, 218, 222, 224, 229, 233, 236, 240,
	245, 247, 251, 256, 258, 263, 267, 269, 274, 276,
	281, 285, 287, 292, 294, 299, 303, 305, 310, 312,
	316, 321, 325, 328, 332, 337, 339, 343, 348, 350,
	355, 359, 361, 366, 368, 373, 377, 379, 384, 386,
	391, 395, 397, 402, 404
};

static unsigned int eureka_brightness_table[EXTEND_BRIGHTNESS + 1] = {
	2,
	4, 8, 12, 15, 18, 21, 24, 27, 30, 33, /* 1: 5 */
	36, 39, 42, 45, 48, 53, 56, 60, 63, 67,
	70, 74, 77, 81, 84, 88, 91, 95, 98, 102,
	105, 109, 112, 116, 120, 123, 127, 130, 134, 137,
	141, 144, 148, 151, 155, 158, 162, 165, 169, 172,
	176, 179, 183, 186, 190, 193, 197, 200, 204, 207,
	211, 214, 218, 221, 225, 228, 232, 235, 239, 242,
	246, 249, 253, 257, 260, 264, 267, 271, 274, 278,
	281, 285, 288, 292, 295, 299, 302, 306, 309, 313,
	316, 320, 323, 327, 330, 334, 337, 341, 344, 348,
	351, 355, 358, 362, 365, 369, 372, 376, 379, 383,
	387, 390, 394, 397, 401, 404, 408, 411, 415, 418,
	422, 425, 429, 432, 436, 439, 443, 445, 451, 456, /* 128: 445 */
	460, 465, 470, 474, 479, 483, 488, 492, 497, 500,
	506, 510, 515, 520, 524, 529, 533, 538, 542, 547,
	551, 556, 561, 565, 570, 574, 579, 583, 588, 592,
	597, 601, 606, 611, 615, 620, 624, 629, 633, 638,
	642, 647, 652, 656, 661, 665, 670, 674, 679, 683,
	688, 693, 697, 702, 706, 711, 715, 720, 724, 729,
	733, 738, 743, 747, 752, 756, 761, 765, 770, 774,
	779, 784, 788, 793, 797, 802, 806, 811, 815, 820,
	825, 829, 834, 838, 843, 847, 852, 856, 861, 865,
	870, 875, 879, 884, 888, 893, 897, 902, 906, 911,
	916, 920, 925, 929, 934, 938, 943, 947, 952, 956,
	961, 966, 970, 975, 979, 984, 988, 993, 997, 1002,
	1007, 1011, 1016, 1020, 1023, 5, 9, 12, 16, 20, /* 255: 1023 */
	23, 27, 31, 34, 38, 42, 45, 49, 53, 55,
	60, 64, 67, 71, 75, 78, 82, 86, 89, 93,
	97, 100, 104, 108, 111, 115, 119, 123, 126, 130,
	134, 137, 141, 145, 148, 152, 156, 159, 163, 167,
	170, 174, 178, 181, 185, 189, 192, 196, 200, 203,
	207, 211, 214, 218, 222, 225, 229, 233, 236, 240,
	244, 247, 251, 255, 258, 262, 266, 269, 273, 277,
	280, 284, 288, 291, 295, 299, 302, 306, 310, 313,
	317, 321, 325, 328, 332, 336, 339, 343, 347, 350,
	354, 358, 361, 365, 375, 400, 415, 430, 444, 454,
	464, 474, 484, 494, 504,
};

static u8 elvss_table[EXTEND_BRIGHTNESS + 1] = {
	[0 ... 255] = 0xC4,
	[256 ... 268] = 0xCE,
	[269 ... 281] = 0xCD,
	[282 ... 295] = 0xCB,
	[296 ... 309] = 0xCA,
	[310 ... 323] = 0xC9,
	[324 ... 336] = 0xC8,
	[337 ... 350] = 0xC6,
	[351 ... EXTEND_BRIGHTNESS - 1] = 0xC5,
	[EXTEND_BRIGHTNESS] = 0xC4,
};
#endif /* __S6E8FC0_PARAM_H__ */
