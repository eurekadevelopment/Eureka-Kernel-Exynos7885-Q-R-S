/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_DEVICE_IMX576_H
#define FIMC_IS_DEVICE_IMX576_H

#define SENSOR_MODULE_NAME           "FIMC-IS-MODULE-IMX576"

enum sensor_module_imx576_position {
	SENSOR_MODULE_IMX576_REAR  = 0,
	SENSOR_MODULE_IMX576_FRONT = 1,
};

#define IMX576_PDAF_MAXWIDTH         0        /* MAX witdh size */
#define IMX576_PDAF_MAXHEIGHT        0        /* MAX height size */
#define IMX576_PDAF_ELEMENT          0        /* V4L2_PIX_FMT_SBGGR16 */

#define IMX576_MIPI_ELEMENT          1        /* V4L2_PIX_FMT_SBGGR16 */

enum sensor_module_imx576_actuator {
	SENSOR_MODULE_IMX576_WITHOUT_ACTUATOR = 0,
	SENSOR_MODULE_IMX576_WITH_ACTUATOR = 1,
};

#endif
