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

#if defined(CONFIG_CAMERA_GVIEW2)
#include "fimc-is-vendor-config_gview2.h"
#elif defined(CONFIG_CAMERA_JACKPOT)
#include "fimc-is-vendor-config_jackpot.h"
#elif defined(CONFIG_CAMERA_JACKPOT_JPN)
#include "fimc-is-vendor-config_jackpot_jpn.h"
#elif defined(CONFIG_CAMERA_J7TOPE)
#include "fimc-is-vendor-config_j7tope.h"
#elif defined(CONFIG_CAMERA_J3TOPE)
#include "fimc-is-vendor-config_j3tope.h"
#elif defined(CONFIG_CAMERA_J7DUO)
#include "fimc-is-vendor-config_j7duo.h"
#elif defined(CONFIG_CAMERA_A6E)
#include "fimc-is-vendor-config_a6e.h"
#elif defined(CONFIG_CAMERA_A7Y18)
#include "fimc-is-vendor-config_a7y18.h"
#elif defined(CONFIG_CAMERA_J2CORE)
#include "fimc-is-vendor-config_j2corelte.h"
#elif defined(CONFIG_CAMERA_M20)
#include "fimc-is-vendor-config_m20.h"
#elif defined(CONFIG_CAMERA_M30)
#include "fimc-is-vendor-config_m30.h"
#else
#include "fimc-is-vendor-config_jackpot.h" /* Default */
#endif

#endif
