/*
 * Copyright (C) 2018 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef __LINUX_VEML3328_H
#define __LINUX_VEML3328_H

#define	VEML3328_ADDR            0x20 >> 1

/* veml3328 command code */
#define	VEML3328_CONF            0x00
#define VEML3328_R_DATA          0x05
#define VEML3328_G_DATA          0x06
#define VEML3328_B_DATA          0x07
#define VEML3328_C_DATA	         0x04
#define VEML3328_IR_DATA         0x08

#define VEML3328_DEVICE_ID_REG   0x0C
#define VEML3328_DEVICE_ID_VAL   0x28

/* veml3328 CONF command code */
#define VEML3328_CONF_SD         0x8001
#define VEML3328_CONF_SD_MASK    0x7FFE

#define VEML3328_CONF_AF         (1 << 3)
#define VEML3328_CONF_TRIG       (1 << 2)
#define VEML3328_CONF_IT_50MS    (0 << 4)
#define VEML3328_CONF_IT_100MS   (1 << 4)
#define VEML3328_CONF_IT_200MS   (2 << 4)
#define VEML3328_CONF_IT_400MS   (3 << 4)

#define VEML3328_CONF_GAIN1_X1   ((0 << 4) << 8)
#define VEML3328_CONF_GAIN1_X2   ((1 << 4) << 8)
#define VEML3328_CONF_GAIN1_X4   ((2 << 4) << 8)

#define VEML3328_CONF_GAIN2_HALF ((3 << 2) << 8)
#define VEML3328_CONF_GAIN2_X1   ((0 << 2) << 8)
#define VEML3328_CONF_GAIN2_X2   ((1 << 2) << 8)
#define VEML3328_CONF_GAIN2_X4   ((2 << 2) << 8)

#define VEML3328_AUTO_GAIN_HALF  (VEML3328_CONF_GAIN1_X1 | VEML3328_CONF_GAIN2_HALF)
#define VEML3328_CONF_GAIN_2     (VEML3328_CONF_GAIN1_X2 | VEML3328_CONF_GAIN2_X1)
#define VEML3328_CONF_GAIN_8     (VEML3328_CONF_GAIN1_X4 | VEML3328_CONF_GAIN2_X2)

#define VEML3328_CONF_GAIN_MASK  0xC3FF

#define LS_PWR_ON                (1 << 0)

#define PREDEFINE_FILE_PATH      "/efs/FactoryApp/predefine"
#define HH_VERSION_FILE_PATH     "/efs/FactoryApp/hh_version"
#define WINDOW_TYPE_FILE_PATH    "/sys/class/lcd/panel/window_type"

#define PATH_LEN                 50
#define FILE_BUF_LEN             110
#define ID_INDEX_NUMS            2
#define RETRY_MAX                3
#define VERSION_FILE_NAME_LEN    20

enum {
	D_FACTOR,
	R_COEF,
	G_COEF,
	B_COEF,
	C_COEF,
	CT_COEF,
	CT_OFFSET,
	THD_HIGH,
	THD_LOW,
	IRIS_PROX_THD,
	SUM_CRC,
	EFS_SAVE_NUMS,
};

enum {
	ID_UTYPE,
	ID_BLACK,
	ID_WHITE,
	ID_GOLD,
	ID_SILVER,
	ID_GREEN,
	ID_BLUE,
	ID_PINKGOLD,
	ID_MAX,
};

struct light_coef_predefine_item {
	int version;
	int color_id;
	int coef[7];	/* r_coef,g_coef,b_coef,c_coef,dgf,cct_coef,cct_offset */
};

#endif
