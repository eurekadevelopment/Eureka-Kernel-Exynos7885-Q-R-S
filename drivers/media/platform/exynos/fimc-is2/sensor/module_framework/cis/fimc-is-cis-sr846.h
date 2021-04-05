/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_CIS_SR846_H
#define FIMC_IS_CIS_SR846_H

#include "fimc-is-cis.h"

#define EXT_CLK_Mhz (26)

#define SENSOR_SR846_MAX_WIDTH		(3264 + 0)
#define SENSOR_SR846_MAX_HEIGHT		(2448 + 0)

/* TODO: Check below values are valid */
#define SENSOR_SR846_FINE_INTEGRATION_TIME_MIN                0x0
#define SENSOR_SR846_FINE_INTEGRATION_TIME_MAX                0x0
#define SENSOR_SR846_COARSE_INTEGRATION_TIME_MIN              0x06
#define SENSOR_SR846_COARSE_INTEGRATION_TIME_MAX_MARGIN       0x06

#define USE_GROUP_PARAM_HOLD	(0)

#endif

