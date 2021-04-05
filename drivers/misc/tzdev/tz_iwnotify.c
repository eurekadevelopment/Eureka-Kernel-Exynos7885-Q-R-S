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

#include <linux/notifier.h>
#include <linux/spinlock.h>

#include <tzdev/iwnotify.h>

static struct blocking_notifier_head tz_iwnotify_nh[TZ_IWNOTIFY_EVENT_CNT];

void tz_iwnotify_call_chains(unsigned int event_mask)
{
	unsigned int i;

	for (i = 0; i < TZ_IWNOTIFY_EVENT_CNT; ++i)
		if (event_mask & (1 << i))
			blocking_notifier_call_chain(&tz_iwnotify_nh[i], 0, NULL);
}

int tz_iwnotify_chain_register(unsigned int event, struct notifier_block *nb)
{
	if (event >= TZ_IWNOTIFY_EVENT_CNT)
		return -EINVAL;

	return blocking_notifier_chain_register(&tz_iwnotify_nh[event], nb);
}

int tz_iwnotify_chain_unregister(unsigned int event, struct notifier_block *nb)
{
	if (event >= TZ_IWNOTIFY_EVENT_CNT)
		return -EINVAL;

	return blocking_notifier_chain_unregister(&tz_iwnotify_nh[event], nb);
}

void tz_iwnotify_initialize(void)
{
	unsigned int i;

	for (i = 0; i < TZ_IWNOTIFY_EVENT_CNT; i++)
		BLOCKING_INIT_NOTIFIER_HEAD(&tz_iwnotify_nh[i]);
}
