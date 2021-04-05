/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SCORE_SYSFS_H_
#define SCORE_SYSFS_H_

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include "score-config.h"

#ifdef ENABLE_SYSFS_SYSTEM
struct score_sysfs_system {
	unsigned int en_dvfs;
	unsigned int en_clk_gate;
};

ssize_t show_sysfs_system_dvfs_en(struct device *dev,
		struct device_attribute *attr,
		char *buf);
ssize_t store_sysfs_system_dvfs_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

ssize_t show_sysfs_system_clk_gate_en(struct device *dev,
		struct device_attribute *attr,
		char *buf);
ssize_t store_sysfs_system_clk_gate_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

extern struct score_sysfs_system sysfs_system;
#endif
#ifdef ENABLE_SYSFS_STATE
struct score_sysfs_state {
	bool		is_en;
	unsigned int	frame_duration;
	unsigned int	long_time;
	unsigned int	short_time;
	unsigned int	long_v_rank;
	unsigned int	short_v_rank;
	unsigned int	long_r_rank;
	unsigned int	short_r_rank;
};

ssize_t show_sysfs_state_val(struct device *dev,
		struct device_attribute *attr,
		char *buf);
ssize_t store_sysfs_state_val(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

ssize_t show_sysfs_state_en(struct device *dev,
		struct device_attribute *attr,
		char *buf);
ssize_t store_sysfs_state_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);

extern struct score_sysfs_state sysfs_state;
#endif
#endif
