/*
 *sec_debug_test.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <soc/samsung/exynos-pmu.h>

typedef void (*force_modify_func)(char *arg);

static const unsigned long tima_rodata = 0xAA55AA55;
static void test_MODIFY_KERNEL(char *arg);

enum {
	FORCE_WRITE_RO = 0,			/* WRITE RODATA */
	NR_FORCE_MODIFY,
};

struct force_modify_item {
	char errname[SZ_32];
	force_modify_func errfunc;
};

struct force_modify {
	struct force_modify_item item[NR_FORCE_MODIFY];
};

struct force_modify force_modify_vector = {
	.item = {
		{"modifykern",	&test_MODIFY_KERNEL},
	}
};

static void test_MODIFY_KERNEL(char *arg)
{
	unsigned long *ptr = (unsigned long *)&tima_rodata;

	pr_info("%s()\n", __func__);

	set_memory_rw((unsigned long)ptr, 1);
	*ptr ^= 0xabcd1234;
	set_memory_ro((unsigned long)ptr, 1);
}

int tima_debug_modify_kernel(const char *val, struct kernel_param *kp)
{
	int i;
	char *temp;
	char *ptr;

	for (i = 0; i < NR_FORCE_MODIFY; i++)
		if (!strncmp(val, force_modify_vector.item[i].errname,
			     strlen(force_modify_vector.item[i].errname))) {
			temp = (char *)val;
			ptr = strsep(&temp, " ");	/* ignore the first token */
			ptr = strsep(&temp, " ");	/* take the second token */
			force_modify_vector.item[i].errfunc(ptr);
	}
	return 0;
}
EXPORT_SYMBOL(tima_debug_modify_kernel);
