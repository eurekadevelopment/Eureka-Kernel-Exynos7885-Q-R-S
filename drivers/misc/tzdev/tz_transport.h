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

#ifndef __TZ_TRANSPORT_H__
#define __TZ_TRANSPORT_H__

#include <linux/types.h>

#define TZ_TRANSPORT_IOC_MAGIC		'c'
#define TZ_TRANSPORT_GET_BUF_SIZE	_IOW(TZ_TRANSPORT_IOC_MAGIC, 0, __u32)

#endif /*!__TZ_TRANSPORT_H__*/
