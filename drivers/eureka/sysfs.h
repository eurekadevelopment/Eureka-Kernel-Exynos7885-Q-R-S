/*
* Copyright (c) 2022 Eureka Team.
*      https://github.com/eurekadevelopment
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*/

#ifndef EUREKA_SYSFS_H
#define EUREKA_SYSFS_H

#include <linux/device.h>
#include <linux/types.h>

#ifdef CONFIG_EUREKA_SYSFS
extern struct device *eureka_device_create(void *drvdata, const char *fmt);
extern struct device *eureka_device_find(const char *name);
extern void eureka_device_destroy(dev_t devt);
#else
static inline struct device *eureka_device_create(void *drvdata, const char *fmt)
{
	pr_err("No rule to make eureka sysfs\n");
	return NULL;
}
static inline void eureka_device_destroy(dev_t devt)
{
	return;
}
#endif

#endif /* EUREKA_SYSFS_H */
