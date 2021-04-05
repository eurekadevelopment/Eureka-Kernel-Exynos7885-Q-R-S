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

#ifndef __TZ_SHMEM_VALIDATOR_H__
#define __TZ_SHMEM_VALIDATOR_H__

#include <linux/pid.h>
#include <linux/types.h>

#define TZ_SHMEM_VALIDATOR_IOC_MAGIC		'c'
#define TZ_SHMEM_VALIDATOR_REGISTER_MEMORY	_IOW(TZ_SHMEM_VALIDATOR_IOC_MAGIC, 0, struct tz_shmem_desc)
#define TZ_SHMEM_VALIDATOR_ALLOW_ANY		_IO(TZ_SHMEM_VALIDATOR_IOC_MAGIC, 1)

struct tz_shmem_desc {
	uint64_t ptr;
	uint64_t size;
};

#ifdef CONFIG_TZ_SHMEM_VALIDATOR
int tz_shmem_validator_approve(pid_t tgid, unsigned long start,
		unsigned long size);
#else
static inline int tz_shmem_validator_approve(pid_t tgid,
		unsigned long start, unsigned long size)
{
	(void) tgid;
	(void) start;
	(void) size;

	return 0;
}
#endif

#endif /* !__TZ_SHMEM_VALIDATOR_H__ */
