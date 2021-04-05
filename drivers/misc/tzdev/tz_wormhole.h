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

#ifndef __TZ_WORMHOLE_H__
#define __TZ_WORMHOLE_H__

#include <linux/poll.h>
#include <linux/types.h>

#define TZ_WORMHOLE_IOC_MAGIC		'w'

#define TZ_WORMHOLE_GET_CRED		_IOR(TZ_WORMHOLE_IOC_MAGIC, 1, int)

#ifdef CONFIG_TZ_WORMHOLE
int tz_wormhole_tzdev_accept(void);
unsigned int tz_wormhole_tzdev_poll(struct file *filp, poll_table *wait);
void tz_wormhole_close_connection(void);
#else /*  CONFIG_TZ_WORMHOLE */
static inline int tz_wormhole_tzdev_accept(void)
{
	return -ENOSYS;
}

static inline unsigned int tz_wormhole_tzdev_poll(struct file *filp, poll_table *wait)
{
	(void) filp;
	(void) wait;

	return -ENOSYS;
}

static inline void tz_wormhole_close_connection(void)
{
	return;
}

#endif /*   CONFIG_TZ_WORMHOLE */

#endif /*!__TZ_WORMHOLE_H__*/
