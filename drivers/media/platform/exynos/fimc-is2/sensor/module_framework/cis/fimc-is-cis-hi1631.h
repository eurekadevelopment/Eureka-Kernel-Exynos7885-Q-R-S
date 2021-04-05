/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_CIS_HI1631_H
#define FIMC_IS_CIS_HI1631_H

#include "fimc-is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_HI1631_MAX_WIDTH                   (4656 + 0)
#define SENSOR_HI1631_MAX_HEIGHT                  (3496 + 0)

#define USE_GROUP_PARAM_HOLD                      (0)
#define TOTAL_NUM_OF_MIPI_LANES                   (4)

/* TODO: Check below values are valid */
#define SENSOR_HI1631_FINE_INTEGRATION_TIME_MIN           (0x0) //Not supported
#define SENSOR_HI1631_FINE_INTEGRATION_TIME_MAX           (0x0) //Not supported
#define SENSOR_HI1631_COARSE_INTEGRATION_TIME_MIN         (0x04)
#define SENSOR_HI1631_COARSE_INTEGRATION_TIME_MAX_MARGIN  (0x04)

#define SENSOR_HI1631_FRAME_LENGTH_LINE_ADDR      (0x020E)
#define SENSOR_HI1631_LINE_LENGTH_PCK_ADDR        (0x0206)
#define SENSOR_HI1631_GROUP_PARAM_HOLD_ADDR       (0x0208)
#define SENSOR_HI1631_COARSE_INTEG_TIME_ADDR      (0x020A)
#define SENSOR_HI1631_ANALOG_GAIN_ADDR            (0x0213) //8bit
#define SENSOR_HI1631_DIG_GAIN_ADDR               (0x0214) //Gr:0x214 ,Gb:0x216, R:0x218, B:0x21A
#define SENSOR_HI1631_MODEL_ID_ADDR               (0x0716)
#define SENSOR_HI1631_STREAM_ONOFF_ADDR           (0x0812)
#define SENSOR_HI1631_ISP_ENABLE_ADDR             (0x0B04) //pdaf, mipi, fmt, Hscaler, D gain, DPC, LSC
#define SENSOR_HI1631_MIPI_TX_OP_MODE_ADDR        (0x1002)
#define SENSOR_HI1631_ISP_FRAME_CNT_ADDR          (0x1056)
#define SENSOR_HI1631_PLL_ENABLE_ADDR             (0x0737)

#define SENSOR_HI1631_MIN_ANALOG_GAIN_SET_VALUE   (0)      //x1.0
#define SENSOR_HI1631_MAX_ANALOG_GAIN_SET_VALUE   (0xF0)   //x16.0
#define SENSOR_HI1631_MIN_DIGITAL_GAIN_SET_VALUE  (0x0200) //x1.0
#define SENSOR_HI1631_MAX_DIGITAL_GAIN_SET_VALUE  (0x1FFB) //x15.99

//WB gain : Not Supported ??

struct setfile_info {
	const u32 *file;
	u32 file_size;
};

#define SET_SETFILE_INFO(s) {   \
	.file = s,                  \
	.file_size = ARRAY_SIZE(s), \
}

#endif


