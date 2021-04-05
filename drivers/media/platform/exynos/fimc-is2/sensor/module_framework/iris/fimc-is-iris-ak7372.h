/*
 * Samsung Exynos SoC series Actuator driver
 *
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "fimc-is-core.h"

#ifndef FIMC_IS_DEVICE_AK7372_H
#define FIMC_IS_DEVICE_AK7372_H
int sensor_ak7372_iris_high_temp(struct fimc_is_core *core);
int sensor_ak7372_iris_low_temp(struct fimc_is_core *core);
int sensor_ak7372_iris_init_temp(struct fimc_is_core *core);
#endif
