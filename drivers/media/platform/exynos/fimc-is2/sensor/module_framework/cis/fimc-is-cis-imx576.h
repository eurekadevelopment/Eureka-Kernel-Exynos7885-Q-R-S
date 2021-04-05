/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_CIS_IMX576_H
#define FIMC_IS_CIS_IMX576_H

#include "fimc-is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_IMX576_MAX_WIDTH          (5664 + 0)
#define SENSOR_IMX576_MAX_HEIGHT         (4248 + 0)

#define SENSOR_IMX576_25M_MAX_WIDTH          (5760 + 0)
#define SENSOR_IMX576_25M_MAX_HEIGHT         (4312 + 0)

#define SENSOR_IMX576_FINE_INTEGRATION_TIME_MIN                (0x510)
#define SENSOR_IMX576_FINE_INTEGRATION_TIME_MAX                (0x510)
#define SENSOR_IMX576_COARSE_INTEGRATION_TIME_MIN              (0x04)
#define SENSOR_IMX576_COARSE_INTEGRATION_TIME_MAX_MARGIN       (0x3D)

#define USE_GROUP_PARAM_HOLD                      (1)

#define SENSOR_IMX576_COARSE_INTEG_TIME_ADDR      (0x0202)
#define SENSOR_IMX576_ANALOG_GAIN_ADDR            (0x0204)
#define SENSOR_IMX576_DIG_GAIN_ADDR               (0x020E)
#define SENSOR_IMX576_SOHT_DIG_GAIN_ADDR          (0x0218)

#define SENSOR_IMX576_MIN_ANALOG_GAIN_SET_VALUE   (0)
#define SENSOR_IMX576_MAX_ANALOG_GAIN_SET_VALUE   (960)
#define SENSOR_IMX576_MIN_DIGITAL_GAIN_SET_VALUE  (0x0100)
#define SENSOR_IMX576_MAX_DIGITAL_GAIN_SET_VALUE  (0x0FFF)

#define SENSOR_IMX576_ABS_GAIN_GR_SET_ADDR        (0x0B8E)
#define SENSOR_IMX576_ABS_GAIN_R_SET_ADDR         (0x0B90)
#define SENSOR_IMX576_ABS_GAIN_B_SET_ADDR         (0x0B92)
#define SENSOR_IMX576_ABS_GAIN_GB_SET_ADDR        (0x0B94)

/* Related EEPROM CAL */
#define SENSOR_IMX576_QUAD_SENS_CAL_BASE_REAR     (0x1E40)
#define SENSOR_IMX576_25M_QUAD_SENS_CAL_BASE_REAR (0x1110)
#define SENSOR_IMX576_QUAD_SENS_CAL_BASE_FRONT    (0x3810)
#define SENSOR_IMX576_QUAD_SENS_CAL_SIZE          (1056)
#define SENSOR_IMX576_QUAD_SENS_REG_ADDR          (0x8000)

#define SENSOR_IMX576_DPC_CAL_BASE_REAR           (0x1630)
#define SENSOR_IMX576_25M_DPC_CAL_BASE_REAR       (0x08F0)
#define SENSOR_IMX576_DPC_CAL_BASE_FRONT          (0x3000)
#define SENSOR_IMX576_DPC_CAL_SIZE                (2048)

#define SENSOR_IMX576_DPC_DUMP_NAME               "/data/media/0/IMX576_DPC_DUMP.bin"
#define SENSOR_IMX576_QSC_DUMP_NAME               "/data/media/0/IMX576_QSC_DUMP.bin"
#define SENSOR_IMX576_CAL_DEBUG                   (0)


/*
 * [Mode Information]
 *	- Global Setting -
 *
 *	- 2X2 BINNING -
 *	[0] REG_A: Single Still Preview (4:3)   : 2832x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[1] REG_B: Single Still Preview (16:9)  : 2832x1592@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[2] REG_C: Single Still Preview (18.5:9): 2832x1376@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[3] REG_D: Single Still Preview (1:1)   : 2124x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- QBC_HDR -
 *	[4] REG_E: Single Still 3HDR (4:3)      : 2832x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[5] REG_F: Single Still 3HDR (16:9)     : 2832x1592@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[6] REG_G: Single Still 3HDR (18.5:9)   : 2832x1376@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[7] REG_H: Single Still 3HDR (1:1)      : 2124x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- QBC_REMOSAIC -
 *	[8] REG_I: Single Still Capture (4:3)   : 5664X4248@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[9] REG_J: Single Still Capture (16:9)  : 5664X3184@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[10]REG_K: Single Still Capture (18.5:9): 5664X2752@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[11]REG_L: Single Still Capture (1:1)   : 4248X4248@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- Super Slow Motion (SSM) -
 *	[12]REG_M: Super Slow Motion (16:9)     : 1872x1052@240,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[13]REG_N: Super Slow Motion (16:9)     : 1920x1080@120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[14]REG_O: Super Slow Motion (16:9)     : 1280x720 @240,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[15]REG_U: Super Slow Motion (16:9)     : 1280x720 @120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054 
 *
 *	- FAST AE -
 *	[16]REG_R: Single Preview Fast(4:3)     : 2832x2124@117,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[17]REG_S: Single Preview Fast(4:3)     : 2832x2124@ 60,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[18]REG_T: Single Preview Fast(16:9)    : 2832x1592@120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 */

enum sensor_imx576_mode_enum {
	/* 2x2 Binnig */
	SENSOR_IMX576_2832X2124_2X2BIN_30FPS = 0,
	SENSOR_IMX576_2832X1592_2X2BIN_30FPS,
	SENSOR_IMX576_2832X1376_2X2BIN_30FPS,
	SENSOR_IMX576_2124X2124_2X2BIN_30FPS,

	/* QBC-HDR */
	IMX576_MODE_QBCHDR_START = 4,
	SENSOR_IMX576_2832X2124_QBCHDR_30FPS = IMX576_MODE_QBCHDR_START,
	SENSOR_IMX576_2832X1592_QBCHDR_30FPS,
	SENSOR_IMX576_2832X1376_QBCHDR_30FPS,
	SENSOR_IMX576_2124X2124_QBCHDR_30FPS,
	IMX576_MODE_QBCHDR_END = SENSOR_IMX576_2124X2124_QBCHDR_30FPS,

	/* QBC-REMOSAIC */
	IMX576_MODE_REMOSAIC_START = 8,
	SENSOR_IMX576_5664X4248_QBCREMOSAIC_30FPS = IMX576_MODE_REMOSAIC_START,
	SENSOR_IMX576_5664X3184_QBCREMOSAIC_30FPS,
	SENSOR_IMX576_5664X2752_QBCREMOSAIC_30FPS,
	SENSOR_IMX576_4248X4248_QBCREMOSAIC_30FPS,
	IMX576_MODE_REMOSAIC_END = SENSOR_IMX576_4248X4248_QBCREMOSAIC_30FPS,

	/* Super Slow Motion */
	SENSOR_IMX576_1870X1052_SSM_240FPS = 12,
	SENSOR_IMX576_1920X1080_SSM_120FPS,
	SENSOR_IMX576_1280X720_SSM_240FPS,
	SENSOR_IMX576_1280X720_SSM_120FPS,

	/* FAST AE */
	SENSOR_IMX576_2832x2124_117FPS = 16,
	SENSOR_IMX576_2832X2124_60FPS,
	SENSOR_IMX576_2832X1592_120FPS,
};

/*
 * [Mode Information] 25M
 *	- Global Setting -
 *
 *	- 2X2 BINNING -
 *	[0] REG_Z : Single Still Preview (4:3)_Ver2    : 2880x2156@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[1] REG_AA: Single Still Preview (16:9)_Ver2   : 2880x1620@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[2] REG_AB: Single Still Preview (18.5:9)_Ver2 : 2880x1400@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[3] REG_AC: Single Still Preview (1:1)_Ver2    : 2156x2156@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[4] REG_AI: Single Still Preview (19.5:9)_Ver2 : 2880x1332@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- QBC_HDR -
 *	[5]REG_AK: Single Still 3HDR (4:3)_Ver2       : 2880x2156@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[6]REG_AL: Single Still 3HDR (16:9)_Ver2      : 2880x1620@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[7]REG_AM: Single Still 3HDR (18.5:9)_Ver2    : 2880x1400@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[8]REG_AO: Single Still 3HDR (1:1)_Ver2       : 2156x2156@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[9]REG_AN: Single Still 3HDR (19.5:9)_Ver2    : 2880x1332@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- QBC_REMOSAIC -
 *	[10] REG_P : Full                               : 5760x4312@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[11] REG_AD: Single Still Capture (16:9)_Ver2   : 5760x3240@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[12] REG_AE: Single Still Capture (18.5:9)_Ver2 : 5760x2800@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[13] REG_AF: Single Still Capture (1:1)_Ver2    : 4312x4312@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[14] REG_AJ: Single Still Capture (19.5:9)_Ver2 : 5760x2664@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- Super Slow Motion (SSM) -
 *	[15]REG_M : Super Slow Motion (16:9)           : 1872x1052@240,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[16]REG_N : Super Slow Motion (16:9)           : 1920x1080@120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[17]REG_O : Super Slow Motion (16:9)           : 1280x720 @240,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[18]REG_U : Super Slow Motion (16:9)           : 1280x720 @120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- FAST AE -
 *	[19]REG_R : Single Preview Fast(4:3)           : 2832x2124@117,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[20]REG_S : Single Preview Fast(4:3)           : 2832x2124@ 60,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[21]REG_T : Single Preview Fast(16:9)          : 2832x1592@120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[22]REG_AG: Single Preview Fast(4:3)_Ver2      : 2880x2156@114,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 */

enum sensor_imx576_25M_mode_enum {
	/* 2x2 Binnig */
	SENSOR_IMX576_25M_2X2BIN_2880X2156_30FPS = 0,
	SENSOR_IMX576_25M_2X2BIN_2880X1620_30FPS,
	SENSOR_IMX576_25M_2X2BIN_2880X1400_30FPS,
	SENSOR_IMX576_25M_2X2BIN_2156X2156_30FPS,
	SENSOR_IMX576_25M_2X2BIN_2880X1332_30FPS,

	/* QBC-HDR */
	IMX576_25M_MODE_QBCHDR_START = 5,
	SENSOR_IMX576_25M_QBCHDR_2880X2156_30FPS = IMX576_25M_MODE_QBCHDR_START,
	SENSOR_IMX576_25M_QBCHDR_2880X1620_30FPS,
	SENSOR_IMX576_25M_QBCHDR_2880X1400_30FPS,
	SENSOR_IMX576_25M_QBCHDR_2156X2156_30FPS,
	SENSOR_IMX576_25M_QBCHDR_2880X1332_30FPS,
	IMX576_25M_MODE_QBCHDR_END = SENSOR_IMX576_25M_QBCHDR_2880X1332_30FPS,

	/* QBC-REMOSAIC */
	IMX576_25M_MODE_REMOSAIC_START = 10,
	SENSOR_IMX576_25M_QBCREMOSAIC_5760X4312_30FPS = IMX576_25M_MODE_REMOSAIC_START,
	SENSOR_IMX576_25M_QBCREMOSAIC_5760X3240_30FPS,
	SENSOR_IMX576_25M_QBCREMOSAIC_5760X2800_30FPS,
	SENSOR_IMX576_25M_QBCREMOSAIC_4312X4312_30FPS,
	SENSOR_IMX576_25M_QBCREMOSAIC_5760X2664_30FPS,
	IMX576_25M_MODE_REMOSAIC_END = SENSOR_IMX576_25M_QBCREMOSAIC_5760X2664_30FPS,

	/* Super Slow Motion */
	SENSOR_IMX576_25M_SSM_1872X1052_240FPS = 15,
	SENSOR_IMX576_25M_SSM_1920X1080_120FPS,
	SENSOR_IMX576_25M_SSM_1280X720_240FPS,
	SENSOR_IMX576_25M_SSM_1280X720_120FPS,

	/* FAST AE */
	SENSOR_IMX576_25M_2832X2124_117FPS = 19,
	SENSOR_IMX576_25M_2832X2124_60FPS,
	SENSOR_IMX576_25M_2832X1592_120FPS,
	SENSOR_IMX576_25M_2880X2156_114FPS,
};

/* IS_REMOSAIC(u32 mode): check if mode is remosaic */
#define IS_REMOSAIC(mode) ({                                            \
        bool ret = 0;                                                   \
        typecheck(u32, mode);		                                \
        if(is_enable_25M) {                                             \
                ret = (mode >= IMX576_25M_MODE_REMOSAIC_START) &&       \
                (mode <= IMX576_25M_MODE_REMOSAIC_END);                 \
        } else {                                                        \
                ret = (mode >= IMX576_MODE_REMOSAIC_START) &&           \
                (mode <= IMX576_MODE_REMOSAIC_END);                     \
	}                                                               \
        ret;                                                            \
})

/* IS_3DHDR(struct fimc_is_cis *cis, u32 mode): check if mode is 3dhdr */
#define IS_3DHDR(mode) ({                                               \
        bool ret = 0;                                                   \
        typecheck(u32, mode);                                           \
        if(is_enable_25M) {                                             \
                ret = (mode >= IMX576_25M_MODE_QBCHDR_START) &&         \
                (mode <= IMX576_25M_MODE_QBCHDR_END);                   \
        } else {                                                        \
	        ret = (mode >= IMX576_MODE_QBCHDR_START) &&             \
                (mode <= IMX576_MODE_QBCHDR_END);                       \
	}                                                               \
        ret;                                                            \
})

#endif
