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

#ifndef __TZDEV_KERNEL_API_H__
#define __TZDEV_KERNEL_API_H__

#include <linux/mm.h>
#include <linux/types.h>

#define TZ_UUID_LEN	16

struct tz_uuid {
	uint32_t time_low;
	uint16_t time_mid;
	uint16_t time_hi_and_version;
	uint8_t clock_seq_and_node[8];
};

int tzdev_kapi_open(const struct tz_uuid *uuid);
int tzdev_kapi_close(int client_id);
int tzdev_kapi_send(int client_id, const void *data, size_t size);
int tzdev_kapi_recv(int client_id, void *buf, size_t size);
int tzdev_kapi_mem_register(void *ptr, unsigned long size, unsigned int write);
int tzdev_kapi_mem_pages_register(struct page **pages, unsigned int nr_pages, unsigned int write);
int tzdev_kapi_mem_release(unsigned int id);
int tzdev_kapi_mem_grant(int client_id, int mem_id);
int tzdev_kapi_mem_revoke(int client_id, int mem_id);

#endif /* __TZDEV_KERNEL_API_H__ */
