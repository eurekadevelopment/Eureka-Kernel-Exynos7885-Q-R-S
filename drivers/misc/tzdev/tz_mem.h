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

#ifndef __TZ_MEM_H__
#define __TZ_MEM_H__

#include <linux/mm.h>
#include <linux/pid.h>

#include "tz_common.h"

struct tzdev_mem_reg {
	struct pid *pid;
	unsigned long nr_pages;
	struct page **pages;
};

int tzdev_mem_init(void);
void tzdev_mem_fini(void);

int tzdev_mem_register_user(struct tzio_mem_register *s);
int tzdev_mem_release_user(unsigned int id);
void tzdev_mem_release_all_user(void);
int tzdev_is_mem_exist(unsigned int id, unsigned int *is_user);

int tzdev_mem_register(void *ptr, unsigned long size, unsigned int write);
int tzdev_mem_pages_register(struct page **pages, unsigned int nr_pages, unsigned int write);
int tzdev_mem_release(unsigned int id);

#endif /* __TZ_MEM_H__ */
