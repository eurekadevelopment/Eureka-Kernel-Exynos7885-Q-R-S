/*
 * Copyright (C) 2015 Samsung Electronics, Inc.
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

#ifndef __TZ_HOTPLUG_H__
#define __TZ_HOTPLUG_H__

#include <linux/compiler.h>

#ifdef CONFIG_TZDEV_HOTPLUG
int tzdev_init_hotplug(void);
void tzdev_exit_hotplug(void);
void tzdev_update_nwd_cpu_mask(unsigned long new_mask);
void tzdev_notify_swd_cpu_mask_update(void);

#else
static inline int tzdev_init_hotplug(void)
{
	return 0;
}
static inline void tzdev_exit_hotplug(void)
{
}
static inline void tzdev_update_nwd_cpu_mask(unsigned long new_mask)
{
	(void) new_mask;
}
static inline void tzdev_notify_swd_cpu_mask_update(void)
{
}
#endif /* CONFIG_TZDEV_HOTPLUG */

#endif /* __TZ_HOTPLUG_H__ */
