/*
 * Samsung Exynos SoC series FIMC-IS2 driver
 *
 * exynos fimc-is2 hw csi control functions
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "fimc-is-hw-api-common.h"
#include "fimc-is-hw-pdp-v1_0.h"
#include "fimc-is-config.h"

int pdp_hw_enable(u32 __iomem *base_reg, u32 pd_mode)
{
	fimc_is_hw_set_field(base_reg, &pdp_regs[PDP_R_SENSOR_TYPE],
			&pdp_fields[PDP_F_SENSOR_TYPE], pd_mode);

	return 0;
}

int pdp_hw_dump(u32 __iomem *base_reg)
{
	info("PDP SFR DUMP (v1.0)\n");
	fimc_is_hw_dump_regs(base_reg, pdp_regs, PDP_REG_CNT);

	return 0;
}
