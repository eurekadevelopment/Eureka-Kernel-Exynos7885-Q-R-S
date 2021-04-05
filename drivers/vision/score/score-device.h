/* * Samsung Exynos SoC series SCORE driver *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef SCORE_DEVICE_H_
#define SCORE_DEVICE_H_

#include "score-system.h"
#include "score-vertex.h"
#include "score-vertexmgr.h"
#include "score-sysfs.h"

#include "score-fw-common.h"
#include "score-fw-queue.h"

enum score_device_state {
	SCORE_DEVICE_STATE_OPEN,
	SCORE_DEVICE_STATE_START
};

struct score_device {
	struct device			*dev;
	struct platform_device		*pdev;
	unsigned long			state;

	struct score_vertexmgr		vertexmgr;
	struct score_vertex		vertex;
	struct score_system		system;
	struct score_fw_dev		fw_dev;
	unsigned int			fw_index;
};

int score_device_open(struct score_device *device);
int score_device_close(struct score_device *device);
int score_device_start(struct score_device *device);
int score_device_stop(struct score_device *device);
int score_device_fw_ld(struct score_device *device, int index);
void score_device_msg_print(void);

#endif
