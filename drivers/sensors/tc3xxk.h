/* tc300k.h -- Linux driver for coreriver chip as grip
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Author: Junkyeong Kim <jk0430.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#ifndef __LINUX_TC3XXK_H
#define __LINUX_TC3XXK_H

#define VENDOR_NAME	"CORERIVER"
#define MODULE_NAME	"grip_sensor"
#define MODEL_NAME	"TC3XXK"

struct tc3xxk_platform_data {
	int gpio_int;
	int gpio_sda;
	int gpio_scl;
	int ldo_en;
	int i2c_gpio;
	int (*power) (void *, bool on);
	int (*power_isp) (bool on);
	u32 irq_gpio_flags;
	u32 sda_gpio_flags;
	u32 scl_gpio_flags;
	const char *regulator_ic;
	bool boot_on_ldo;
	int bringup;

	const char *fw_name;
	u32 use_bitmap;
};

#endif /* __LINUX_TC3XXK_H */
