/*
* Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is vender functions
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_VENDOR_CONFIG_H
#define FIMC_IS_VENDOR_CONFIG_H

/* #define ENABLE_REMOSAIC_CAPTURE_WITH_ROTATION */
#define ENABLE_MCSC_TDNR_VENDOR_SIZE

#if defined(ENABLE_MCSC_TDNR_VENDOR_SIZE)
#define MAX_MCSC_DNR_WIDTH		(4608)
#define MAX_MCSC_DNR_HEIGHT		(3456)
#endif

#endif
