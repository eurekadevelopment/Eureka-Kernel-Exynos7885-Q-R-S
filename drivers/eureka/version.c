/*
* Copyright (c) 2022 Eureka Team.
*      https://github.com/eurekadevelopment
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*/

#include "sysfs.h"
#include "version.h"
#include <linux/kernel.h>
#include <linux/init.h>

static ssize_t show_compiler(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", LINUX_COMPILER);
}
static ssize_t show_release(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", UTS_RELEASE);
}
static ssize_t show_user(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", LINUX_COMPILE_BY);
}
static ssize_t show_host(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", LINUX_COMPILE_HOST);
}
static ssize_t show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", UTS_VERSION);
}
static ssize_t show_eureka_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", KERNEL_VERSION);
}
static ssize_t show_sched(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", KERNEL_HMP ? "HMP" : "EMS" );
}

static DEVICE_ATTR(compiler, 0440, show_compiler, NULL);
static DEVICE_ATTR(release, 0440, show_release, NULL);
static DEVICE_ATTR(user, 0440, show_user, NULL);
static DEVICE_ATTR(host, 0440, show_host, NULL);
static DEVICE_ATTR(version, 0440, show_version, NULL);
static DEVICE_ATTR(eureka_version, 0440, show_eureka_version, NULL);
static DEVICE_ATTR(sched, 0440, show_sched, NULL);

static int __init eureka_sysfs_version_init(void)
{
	struct device *dev;

	dev = eureka_device_create(NULL, "version");
	WARN_ON(!dev);
	if (IS_ERR(dev))
		pr_err("%s: Failed to create device\n", __func__);

	if (device_create_file(dev, &dev_attr_compiler) < 0)
		pr_err("%s: Failed to create device file\n", __func__);

	if (device_create_file(dev, &dev_attr_release) < 0)
		pr_err("%s: Failed to create device file\n", __func__);
	if (device_create_file(dev, &dev_attr_user) < 0)
		pr_err("%s: Failed to create device file\n", __func__);
	if (device_create_file(dev, &dev_attr_host) < 0)
		pr_err("%s: Failed to create device file\n", __func__);
	if (device_create_file(dev, &dev_attr_version) < 0)
		pr_err("%s: Failed to create device file\n", __func__);
	if (device_create_file(dev, &dev_attr_eureka_version) < 0)
		pr_err("%s: Failed to create device file\n", __func__);
	if (device_create_file(dev, &dev_attr_sched) < 0)
		pr_err("%s: Failed to create device file\n", __func__);

	return 0;
}
late_initcall(eureka_sysfs_version_init);

