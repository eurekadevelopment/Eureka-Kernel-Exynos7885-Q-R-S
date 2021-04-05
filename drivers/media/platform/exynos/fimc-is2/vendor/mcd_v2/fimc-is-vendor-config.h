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

#if defined(CONFIG_FIMC_IS_DDK_DATA_LOAD) || defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
#define USE_TZ_CONTROLLED_MEM_ATTRIBUTE
#endif

#if defined(CONFIG_CAMERA_WXS_V00)
#include "fimc-is-vendor-config_wxs_v00.h"
#elif defined(CONFIG_CAMERA_ATS_V03)
#include "fimc-is-vendor-config_ats_v03.h"
#elif defined(CONFIG_CAMERA_ATS_V04)
#include "fimc-is-vendor-config_ats_v04.h"
#elif defined(CONFIG_CAMERA_AAS_V30)
#include "fimc-is-vendor-config_aas_v30.h"
#elif defined(CONFIG_CAMERA_AAS_V40)
#include "fimc-is-vendor-config_aas_v40.h"
#elif defined(CONFIG_CAMERA_AAS_V20)
#include "fimc-is-vendor-config_aas_v20.h"
#elif defined(CONFIG_CAMERA_AAS_V30C)
#include "fimc-is-vendor-config_aas_v30c.h"
#elif defined(CONFIG_CAMERA_AAS_V30S)
#include "fimc-is-vendor-config_aas_v30s.h"
#elif defined(CONFIG_CAMERA_AAS_V10)
#include "fimc-is-vendor-config_aas_v10.h"
#elif defined(CONFIG_CAMERA_AAS_V30J)
#include "fimc-is-vendor-config_aas_v30j.h"
#elif defined(CONFIG_CAMERA_AAS_V20E)
#include "fimc-is-vendor-config_aas_v20e.h"
#elif defined(CONFIG_CAMERA_AAS_V10E)
#include "fimc-is-vendor-config_aas_v10e.h"
#elif defined(CONFIG_CAMERA_AAS_V10EKX)
#include "fimc-is-vendor-config_aas_v10ekx.h"
#elif defined(CONFIG_CAMERA_XXS_V04S)
#include "fimc-is-vendor-config_xxs_v04s.h"
#elif defined(CONFIG_CAMERA_AAS_V07J)
#include "fimc-is-vendor-config_aas_v07j.h"
#elif defined(CONFIG_CAMERA_MMS_V20)
#include "fimc-is-vendor-config_mms_v20.h"
#elif defined(CONFIG_CAMERA_MMS_V10S)
#include "fimc-is-vendor-config_mms_v10s.h"
#else
#include "fimc-is-vendor-config_common.h"
#endif

#endif
