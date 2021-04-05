#ifndef __S6E8FC0_PARAM_H__
#define __S6E8FC0_PARAM_H__

#include <linux/types.h>
#include <linux/kernel.h>

#define EXTEND_BRIGHTNESS	355
#define UI_MAX_BRIGHTNESS	255
#define UI_DEFAULT_BRIGHTNESS	128

#define NORMAL_TEMPERATURE	25	/* 25 degrees Celsius */

#define ACL_CMD_CNT			((u16)ARRAY_SIZE(SEQ_ACL_OFF))
#define OPR_CMD_CNT			((u16)ARRAY_SIZE(SEQ_ACL_OPR_OFF))
#define HBM_CMD_CNT				((u16)ARRAY_SIZE(SEQ_HBM_OFF))
#define ELVSS_CMD_CNT				((u16)ARRAY_SIZE(SEQ_ELVSS_SET))

#define LDI_REG_BRIGHTNESS			0x51
#define LDI_REG_ID				0x04
#define LDI_REG_COORDINATE			0xA1
#define LDI_REG_DATE				LDI_REG_COORDINATE
#define LDI_REG_MANUFACTURE_INFO		0xA8	/* A1->A8 because A1 does not support usual GPara */
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

static unsigned char SEQ_ELVSS_SET[] = {
	0xBF,
	0x19,	/* 1st para TSET */
	0x0D, 0x80,
	0xD0,	/* 4th para ELVSS */
	0x04	/* 5th para 4 frame dim speed */
};

static unsigned char SEQ_HBM_ON[] = {
	0x53,
	0xE8,
};

static unsigned char SEQ_HBM_OFF[] = {
	0x53,
	0x28,
};

static unsigned char SEQ_EDGE_DIM[] = {
	0xB2,
	0x0A, 0x0C, 0xD8, 0xD8, 0xBA, 0xBA	/* 5th~6th 0xBA, 0xBA - 50% */
};

static unsigned char SEQ_ACL_OPR_OFF[] = {
	0xC1,
	0x41,	/* 16 Frame Avg. at ACL Off */
	0x11, 0x12, 0x15, 0x55, 0x55, 0x55, 0x0F, 0x1B, 0x18,
	0x47, 0x02,
	0x61, 0x28,	/* 13th~14th para ACL 15% */
	0x4A,
	0x41, 0xFC,	/* 16th~17th para Start step 50% */
	0x00
};

static unsigned char SEQ_ACL_OPR_08P[] = {
	0xC1,
	0x41,	/* 16 Frame Avg. at ACL Off */
	0x11, 0x12, 0x15, 0x55, 0x55, 0x55, 0x0F, 0x1B, 0x18,
	0x47, 0x02,
	0x60, 0x98,	/* 13th~14th para ACL 8% */
	0x4A,
	0x42, 0x64,	/* 16th~17th para Start step 60% */
	0x00
};

static unsigned char SEQ_ACL_OPR_15P[] = {
	0xC1,
	0x51,	/* 32 Frame Avg. at ACL On */
	0x11, 0x12, 0x15, 0x55, 0x55, 0x55, 0x0F, 0x1B, 0x18,
	0x47, 0x02,
	0x61, 0x28,	/* 13th~14th para ACL 15% */
	0x4A,
	0x41, 0xFC,	/* 16th~17th para Start step 50% */
	0x00
};

static unsigned char SEQ_ACL_OFF[] = {
	0x55,
	0x00
};

static unsigned char SEQ_ACL_ON[] = {
	0x55,
	0x02
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
		[0 ... UI_MAX_BRIGHTNESS]					= OPR_STATUS_15P,
		[UI_MAX_BRIGHTNESS + 1 ... EXTEND_BRIGHTNESS]			= OPR_STATUS_08P
	}
};

/* platform brightness <-> gamma level */
static unsigned int brightness_table[EXTEND_BRIGHTNESS + 1] = {
	15,
	18, 21, 24, 27, 29, 32, 35, 38, 41, 44,
	47, 50, 53, 56, 61, 64, 67, 73, 76, 82,
	84, 87, 93, 96, 102, 105, 110, 113, 119, 122,
	125, 131, 134, 139, 142, 148, 151, 157, 160, 163,
	168, 171, 177, 180, 186, 189, 194, 197, 200, 206,
	209, 215, 218, 223, 226, 232, 235, 238, 244, 246,
	252, 255, 261, 264, 270, 273, 275, 281, 284, 290,
	293, 299, 301, 307, 310, 313, 319, 322, 327, 330,
	336, 339, 345, 348, 351, 356, 359, 365, 368, 371,
	377, 382, 385, 388, 394, 397, 403, 406, 408, 414,
	420, 423, 426, 432, 435, 440, 443, 449, 452, 458,
	461, 463, 469, 472, 478, 481, 487, 489, 495, 498,
	504, 507, 510, 516, 518, 524, 527, 530, 536, 538,
	544, 547, 549, 555, 558, 563, 566, 569, 575, 577,
	583, 586, 588, 594, 597, 602, 605, 611, 614, 616,
	622, 625, 627, 633, 636, 639, 644, 647, 653, 655,
	658, 664, 666, 672, 675, 678, 683, 686, 692, 694,
	700, 703, 705, 711, 714, 719, 722, 725, 728, 733,
	736, 742, 744, 747, 753, 756, 761, 764, 770, 772,
	775, 781, 783, 789, 792, 795, 800, 803, 809, 811,
	814, 820, 822, 825, 831, 834, 836, 842, 845, 850,
	853, 859, 861, 864, 870, 873, 878, 881, 884, 889,
	892, 898, 900, 903, 909, 912, 917, 920, 923, 928,
	931, 934, 939, 942, 948, 951, 953, 959, 962, 967,
	970, 973, 978, 981, 987, 990, 992, 998, 1001, 1006,
	1009, 1012, 1017, 1020, 1023, 1, 2, 3, 4, 5,
	5, 6, 7, 8, 9, 10, 11, 11, 12, 13,
	14, 15, 16, 17, 18, 18, 19, 20, 21, 22,
	23, 24, 25, 25, 26, 26, 28, 29, 29, 31,
	31, 31, 33, 34, 34, 36, 37, 37, 37, 38,
	39, 41, 42, 42, 44, 44, 44, 45, 47, 47,
	49, 49, 50, 50, 51, 52, 53, 55, 55, 57,
	57, 57, 58, 60, 60, 61, 62, 63, 63, 65,
	65, 66, 67, 68, 69, 70, 70, 71, 72, 73,
	74, 75, 76, 77, 77, 78, 79, 80, 81, 82,
	83, 83, 84, 85, 86,
};

static u8 elvss_table[EXTEND_BRIGHTNESS + 1] = {
	[0 ... 255] = 0xD0,
	[256 ... 259] = 0xDD,
	[260 ... 264] = 0xDC,
	[265 ... 270] = 0xDB,
	[271 ... 276] = 0xDA,
	[277 ... 284] = 0xD9,
	[285 ... 295] = 0xD8,
	[296 ... 304] = 0xD7,
	[305 ... 310] = 0xD6,
	[311 ... 317] = 0xD5,
	[318 ... 325] = 0xD4,
	[326 ... 331] = 0xD3,
	[332 ... 337] = 0xD2,
	[338 ... 345] = 0xD1,
	[346 ... EXTEND_BRIGHTNESS] = 0xD0,
};
#endif /* __S6E8FC0_PARAM_H__ */
