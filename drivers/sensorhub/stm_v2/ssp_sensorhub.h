/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#ifndef __SSP_SENSORHUB_H__
#define __SSP_SENSORHUB_H__

#include <linux/completion.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include "ssp.h"

#define BIG_DATA_SIZE                   256
#define PRINT_TRUNCATE                  6

#define RESET_REASON_KERNEL_RESET            0x01
#define RESET_REASON_MCU_CRASHED             0x02
#define RESET_REASON_SYSFS_REQUEST           0x03

#ifndef SENSOR_TYPE_SCONTEXT
/* 'LIST_SIZE' should be be rounded-up to a power of 2 */
#define LIST_SIZE                       4
#define MAX_DATA_COPY_TRY               2
#define WAKE_LOCK_TIMEOUT               (0.3*HZ)
#define COMPLETION_TIMEOUT              (2*HZ)
#define DATA                            REL_RX
#define BIG_DATA                        REL_RY /*unused*/
#define NOTICE                          REL_RZ

struct sensorhub_event {
	char *library_data;
	int library_length;
};

struct ssp_sensorhub_data {
	struct ssp_data *ssp_data;
	struct input_dev *sensorhub_input_dev;
	struct task_struct *sensorhub_task;
	struct miscdevice sensorhub_device;
	struct wake_lock sensorhub_wake_lock;
	struct completion read_done;
	struct sensorhub_event events[LIST_SIZE];
	struct kfifo fifo;
	int event_number;
	wait_queue_head_t sensorhub_wq;
	spinlock_t sensorhub_lock;
};

void ssp_sensorhub_report_notice(struct ssp_data *ssp_data, char notice);
int ssp_sensorhub_handle_data(struct ssp_data *ssp_data, char *dataframe,
                              int start, int end);
int ssp_sensorhub_initialize(struct ssp_data *ssp_data);
void ssp_sensorhub_remove(struct ssp_data *ssp_data);
#endif

void ssp_sensorhub_log(const char *func_name,
                       const char *data, int length);
int ssp_scontext_initialize(struct ssp_data *ssp_data);
void ssp_scontext_remove(struct ssp_data *ssp_data);

#endif
