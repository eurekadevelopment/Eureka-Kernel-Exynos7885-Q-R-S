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
#include <linux/errno.h>

#include "tzdev.h"

/* TZDEV platform specific functions */
int __weak tzdev_platform_register(void)
{
	return 0;
}

void __weak tzdev_platform_unregister(void)
{
	return;
}

int __weak tzdev_platform_open(void)
{
	return 0;
}

int __weak tzdev_platform_close(void)
{
	return 0;
}

int __weak tzdev_platform_ioctl(unsigned int cmd, unsigned long arg)
{
	(void)cmd;
	(void)arg;

	return -ENOTTY;
}

int __weak tzdev_platform_smc_call(struct tzio_smc_data *data)
{
	register unsigned long _r0 __asm__(REGISTERS_NAME "0") = data->args[0] | TZDEV_SMC_MAGIC;
	register unsigned long _r1 __asm__(REGISTERS_NAME "1") = data->args[1];
	register unsigned long _r2 __asm__(REGISTERS_NAME "2") = data->args[2];
	register unsigned long _r3 __asm__(REGISTERS_NAME "3") = data->args[3];

	__asm__ __volatile__(ARCH_EXTENSION SMC(0) : "+r"(_r0), "+r" (_r1), "+r" (_r2), "+r" (_r3) : : "memory");

	data->args[0] = _r0;
	data->args[1] = _r1;
	data->args[2] = _r2;
	data->args[3] = _r3;

	return 0;
}
