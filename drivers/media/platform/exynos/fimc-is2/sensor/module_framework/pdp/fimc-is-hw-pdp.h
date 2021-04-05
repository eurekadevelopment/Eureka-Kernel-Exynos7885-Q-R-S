/*
 * Samsung Exynos SoC series FIMC-IS2 driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_HW_PDP_H
#define FIMC_IS_HW_PDP_H

int pdp_hw_enable(u32 __iomem *base_reg, u32 pd_mode);
int pdp_hw_dump(u32 __iomem *base_reg);

#endif
