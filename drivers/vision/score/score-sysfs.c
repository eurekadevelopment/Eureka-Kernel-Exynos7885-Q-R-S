/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm_qos.h>
#include <linux/gpio.h>
#include "score-sysfs.h"

#ifdef ENABLE_SYSFS_SYSTEM
ssize_t show_sysfs_system_dvfs_en(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", sysfs_system.en_dvfs);
}

ssize_t store_sysfs_system_dvfs_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef ENABLE_DVFS
	struct score_device *device =
		(struct score_device *)platform_get_drvdata(to_platform_device(dev));
	struct score_system *system;
	int i;

	BUG_ON(!core);

	system = &device->system;

	switch (buf[0]) {
		case '0':
			sysfs_system.en_dvfs = false;
			/* update dvfs lever to max */
			mutex_lock(&system->dvfs_ctrl.lock);
			for (i = 0; i < FIMC_IS_STREAM_COUNT; i++) {
				if (test_bit(SCORE_DEVICE_OPEN, &device.state))
					score_set_dvfs(system, SCORE_DVFS_MAX);
			}
			score_dvfs_init(system);
			system->dvfs_ctrl.static_ctrl->current_id = SCORE_DVFS_MAX;
			mutex_unlock(&system->dvfs_ctrl.lock);
			break;
		case '1':
			/* It can not re-define static scenario */
			sysfs_debug.en_dvfs = true;
			break;
		default:
			pr_debug("%s: %c\n", __func__, buf[0]);
			break;
	}
#endif
	return count;
}

ssize_t show_sysfs_system_clk_gate_en(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", sysfs_system.en_clk_gate);
}

ssize_t store_sysfs_system_clk_gate_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef ENABLE_CLOCK_GATE
	switch (buf[0]) {
		case '0':
			sysfs_system.en_clk_gate = false;
			sysfs_system.clk_gate_mode = CLOCK_GATE_MODE_HOST;
			break;
		case '1':
			sysfs_system.en_clk_gate = true;
			sysfs_system.clk_gate_mode = CLOCK_GATE_MODE_HOST;
			break;
		default:
			pr_debug("%s: %c\n", __func__, buf[0]);
			break;
	}
#endif
	return count;
}

#endif /* ENABLE_SYSFS_SYSTEM */
#ifdef ENABLE_SYSFS_STATE
ssize_t show_sysfs_state_val(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "du(%d) l/s(%d %d) lv/sv(%d %d) lr/sr(%d %d)\n",
			sysfs_state.frame_duration,
			sysfs_state.long_time,
			sysfs_state.short_time,
			sysfs_state.long_v_rank,
			sysfs_state.short_v_rank,
			sysfs_state.long_r_rank,
			sysfs_state.short_r_rank);
}

ssize_t store_sysfs_state_val(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret_count;
	int input_val[7];

	ret_count = sscanf(buf, "%d %d %d %d %d %d %d", &input_val[0], &input_val[1],
			&input_val[2], &input_val[3],
			&input_val[4], &input_val[5], &input_val[6]);
	if (ret_count != 7) {
		probe_err("%s: count should be 7 but %d \n", __func__, ret_count);
		return -EINVAL;
	}

	sysfs_state.frame_duration = input_val[0];
	sysfs_state.long_time = input_val[1];
	sysfs_state.short_time = input_val[2];
	sysfs_state.long_v_rank = input_val[3];
	sysfs_state.short_v_rank = input_val[4];
	sysfs_state.long_r_rank = input_val[5];
	sysfs_state.short_v_rank = input_val[6];

	return count;
}

ssize_t show_sysfs_state_en(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (sysfs_state.is_en)
		return snprintf(buf, PAGE_SIZE, "%s\n", "enabled");
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "disabled");
}

ssize_t store_sysfs_state_en(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	if (buf[0] == '1')
		sysfs_state.is_en = true;
	else
		sysfs_state.is_en = false;

	return count;
}
#endif /* ENABLE_SYSFS_STATE */
