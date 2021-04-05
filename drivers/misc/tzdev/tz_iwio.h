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

#ifndef __TZ_IWIO_H__
#define __TZ_IWIO_H__

enum {
	TZ_IWIO_CONNECT_LOG,
	TZ_IWIO_CONNECT_SERVICE,
	TZ_IWIO_CONNECT_TRANSPORT,
	TZ_IWIO_CONNECT_PROFILER,
	TZ_IWIO_CONNECT_PANIC_DUMP,
	TZ_IWIO_CONNECT_CNT
};

#define TZ_IWIO_AUX_BUF_SIZE PAGE_SIZE

struct tz_iwio_aux_channel {
	char buffer[TZ_IWIO_AUX_BUF_SIZE];
} __packed;

struct tz_iwio_aux_channel *tz_iwio_get_aux_channel(void);
void tz_iwio_put_aux_channel(void);
int tz_iwio_alloc_aux_channel(int cpu);
void *tz_iwio_alloc_iw_channel(unsigned int mode, unsigned int num_pages);

#endif /* __TZ_IWIO_H__ */
