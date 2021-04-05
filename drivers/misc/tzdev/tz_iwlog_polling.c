/*
 * Copyright (C) 2013-2016 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/workqueue.h>

#include "tz_iwlog.h"
#include "tz_iwlog_polling.h"

static void tzio_log_bh(struct work_struct *work);

static DECLARE_DELAYED_WORK(tzio_log_work, tzio_log_bh);

static void tzio_log_bh(struct work_struct *work)
{
	tz_iwlog_read_buffers();
	schedule_delayed_work(&tzio_log_work,
			CONFIG_TZLOG_POLLING_PERIOD * HZ / MSEC_PER_SEC);
}

void tz_iwlog_schedule_delayed_work(void)
{
	schedule_delayed_work(&tzio_log_work,
			CONFIG_TZLOG_POLLING_PERIOD * HZ / MSEC_PER_SEC);
}

void tz_iwlog_cancel_delayed_work(void)
{
	cancel_delayed_work_sync(&tzio_log_work);
}
