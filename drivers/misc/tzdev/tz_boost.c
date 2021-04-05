/*
 * Copyright (C) 2016 Samsung Electronics, Inc.
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

#include <linux/mutex.h>
#include <linux/pm_qos.h>

#include "tz_boost.h"
#include "tz_hotplug.h"

static unsigned int tz_boost_is_active;
static struct pm_qos_request tz_boost_qos;
static unsigned int cpu_boost_mask;
static DEFINE_MUTEX(tz_boost_lock);

void tz_boost_set_boost_mask(unsigned int big_cpus_mask)
{
	cpu_boost_mask = big_cpus_mask;
}

void tz_boost_enable(void)
{
	tzdev_update_nwd_cpu_mask(cpu_boost_mask);

	mutex_lock(&tz_boost_lock);

	if (tz_boost_is_active) {
		mutex_unlock(&tz_boost_lock);
		return;
	}

	pm_qos_add_request(&tz_boost_qos, TZ_BOOST_CPU_FREQ_MIN,
				TZ_BOOST_CPU_FREQ_MAX_DEFAULT_VALUE);

	tz_boost_is_active = 1;
	mutex_unlock(&tz_boost_lock);
}

void tz_boost_disable(void)
{
	tzdev_update_nwd_cpu_mask(0);

	mutex_lock(&tz_boost_lock);

	if (!tz_boost_is_active) {
		mutex_unlock(&tz_boost_lock);
		return;
	}

	tz_boost_is_active = 0;

	pm_qos_remove_request(&tz_boost_qos);

	mutex_unlock(&tz_boost_lock);
}
