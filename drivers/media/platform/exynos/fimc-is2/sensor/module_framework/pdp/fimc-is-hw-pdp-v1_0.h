/*
 * Samsung Exynos SoC series FIMC-IS2 driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_HW_PDP_V1_0_H
#define FIMC_IS_HW_PDP_V1_0_H

#include "fimc-is-hw-api-common.h"

/* the total count of pdp v1.0's regs */
enum fimc_is_hw_pdp_reg_name {
	PDP_R_PDP_CORE_ENABLE,
	PDP_R_PDP_SWRESET0,
	PDP_R_PDP_SWRESET1,
	PDP_R_PDP_CLK_GATING_DIS,
	PDP_R_PDP_INT,
	PDP_R_PDP_INT_MASK,
	PDP_R_SENSOR_TYPE,
	PDP_R_LBCTRL_INPUT_SIZE,
	PDP_REG_CNT
};

static struct fimc_is_reg pdp_regs[PDP_REG_CNT] = {
	{0x0900, "PDP_CORE_ENABLE"},
	{0x0904, "PDP_SWRESET0"},
	{0x0908, "PDP_SWRESET1"},
	{0x090C, "PDP_CLK_GATING_DIS"},
	{0x0910, "PDP_INT"},
	{0x0914, "PDP_INT_MASK"},
	{0x0918, "SENSOR_TYPE"},
	{0x091C, "LBCTRL_INPUT_SIZE"},
};

/* the total count of the fields of pdp v1.0's reg */
enum fimc_is_hw_pdp_reg_field {
	PDP_F_PDP_READSHADOWREG,
	PDP_F_PDP_CORE_ENABLE,
	PDP_F_PAF_STAT_MEM_INI,
	PDP_F_PAF_SWRESET,
	PDP_F_XTC_SWRESET,
	PDP_F_IP_PROCESSING,
	PDP_F_CLK_GATING_LBCTRL_DIS,
	PDP_F_CLK_GATING_LRSUM_DIS,
	PDP_F_CLK_GATING_YEXT_DIS,
	PDP_F_CLK_GATING_XTC_DIS,
	PDP_F_CLK_GATING_PAF_DIS,
	PDP_F_CLK_GATING_APB_DIS,
	PDP_F_PAF_STAT_INT_2,
	PDP_F_PAF_STAT_INT_1,
	PDP_F_PAF_STAT_INT_0,
	PDP_F_FRAME_END,
	PDP_F_FRAME_START,
	PDP_F_PAF_STAT_INT_2_MASK,
	PDP_F_PAF_STAT_INT_1_MASK,
	PDP_F_PAF_STAT_INT_0_MASK,
	PDP_F_FRAME_END_MASK,
	PDP_F_FRAME_START_MASK,
	PDP_F_SENSOR_TYPE,
	PDP_F_IMG_HEIGHT,
	PDP_F_IMG_WIDTH,
	PDP_REG_FIELD_CNT
};

static struct fimc_is_field pdp_fields[PDP_REG_FIELD_CNT] = {
	/* field_name, start_bit, bit_width, type, reset */
	{"PDP_READSHADOWREG",		16,  1, RW, 0},
	{"PDP_CORE_ENABLE",		 0,  1, RW, 0},
	{"PAF_STAT_MEM_INI",		16,  1, RW, 0},
	{"PAF_SWRESET",			 0,  1, RW, 0},
	{"XTC_SWRESET",			 0,  1, RW, 0},
	{"IP_PROCESSING",		16,  1, RW, 0},
	{"CLK_GATING_LBCTRL_DIS",	 5,  1, RW, 0},
	{"CLK_GATING_LRSUM_DIS",	 4,  1, RW, 0},
	{"GATING_YEXT_DIS",		 3,  1, RW, 0},
	{"GATING_XTC_DIS",		 2,  1, RW, 0},
	{"GATING_PAF_DIS",		 1,  1, RW, 0},
	{"GATING_APB_DIS",		 0,  1, RW, 0},
	{"PAF_STAT_INT_2",		 4,  1, RW, 0},
	{"PAF_STAT_INT_1",		 3,  1, RW, 0},
	{"PAF_STAT_INT_0",		 2,  1, RW, 0},
	{"FRAME_END",			 1,  1, RW, 0},
	{"FRAME_START",			 0,  1, RW, 0},
	{"PAF_STAT_INT_2_MASK",		 4,  1, RW, 1},
	{"PAF_STAT_INT_1_MASK",		 3,  1, RW, 1},
	{"PAF_STAT_INT_0_MASK",		 2,  1, RW, 1},
	{"FRAME_END_MASK",		 1,  1, RW, 1},
	{"FRAME_START_MASK",		 0,  1, RW, 1},
	{"SENSOR_TYPE",			 0,  2, RW, 0},
	{"IMG_HEIGHT",			16, 14, RW, 0},
	{"IMG_WIDTH",			 0, 14, RW, 0},
};
#endif
