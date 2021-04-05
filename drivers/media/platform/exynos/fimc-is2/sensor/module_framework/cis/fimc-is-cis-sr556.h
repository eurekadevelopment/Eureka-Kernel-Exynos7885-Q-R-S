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

#ifndef FIMC_IS_CIS_SR556_H
#define FIMC_IS_CIS_SR556_H

#include "fimc-is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_SR556_MAX_WIDTH		(2576 + 16)
#define SENSOR_SR556_MAX_HEIGHT		(1932 + 12)

#define SENSOR_SR556_FINE_INTEGRATION_TIME_MIN                0x0
#define SENSOR_SR556_FINE_INTEGRATION_TIME_MAX                0x0
#define SENSOR_SR556_COARSE_INTEGRATION_TIME_MIN              0x06
#define SENSOR_SR556_COARSE_INTEGRATION_TIME_MAX_MARGIN       0x06

#define USE_GROUP_PARAM_HOLD	(0)

#endif

