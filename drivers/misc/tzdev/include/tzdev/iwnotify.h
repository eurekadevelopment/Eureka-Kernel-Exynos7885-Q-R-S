/*
 * Copyright (C) 2012-2017 Samsung Electronics, Inc.
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

#ifndef __TZDEV_IWNOTIFY_H__
#define __TZDEV_IWNOTIFY_H__

#include <linux/notifier.h>

enum {
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_1,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_2,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_3,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_4,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_5,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_6,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_7,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_8,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_9,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_10,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_11,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_12,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_13,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_14,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_15,
	TZ_IWNOTIFY_OEM_NOTIFICATION_FLAG_16,
	TZ_IWNOTIFY_EVENT_CNT
};

void tz_iwnotify_call_chains(unsigned int event_mask);
int tz_iwnotify_chain_register(unsigned int event, struct notifier_block *nb);
int tz_iwnotify_chain_unregister(unsigned int event, struct notifier_block *nb);
void tz_iwnotify_initialize(void);

#endif /* __TZDEV_IWNOTIFY_H__ */
