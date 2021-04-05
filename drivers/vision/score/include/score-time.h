/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>

enum score_time_measure_point {
	SCORE_TMP_QUEUE,
	SCORE_TMP_REQUEST,
	SCORE_TMP_RESOURCE,
	SCORE_TMP_PROCESS,
	SCORE_TMP_DONE,
	SCORE_TMP_COUNT
};

struct score_time {
	struct timeval		time;
};

void score_get_timestamp(struct score_time *time);

#define SCORE_TIME_IN_US(v)	((v).time.tv_sec * 1000000 + (v).time.tv_usec)