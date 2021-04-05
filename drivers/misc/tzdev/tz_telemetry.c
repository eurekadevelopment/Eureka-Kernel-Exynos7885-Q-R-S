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

#include <linux/atomic.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <tzdev/tzdev.h>

#include "tz_cdev.h"
#include "tz_telemetry.h"
#include "tzdev.h"

MODULE_AUTHOR("Pavel Bogachev <p.bogachev@partner.samsung.com>");
MODULE_DESCRIPTION("Trustzone telemetry driver");
MODULE_LICENSE("GPL");

#define TZ_TELEMETRY_DEVICE_NAME "tz_telemetry"

static long tz_telemetry_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	switch (cmd) {
	case TZ_TELEMETRY_CONTROL: {
		struct tzio_telemetry_ctrl __user *argp
				= (struct tzio_telemetry_ctrl __user *) arg;
		struct tzio_telemetry_ctrl ctrl;

		if (copy_from_user(&ctrl, argp,
				sizeof(struct tzio_telemetry_ctrl)))
			return -EFAULT;

		return tzdev_smc_telemetry_control(ctrl.mode, ctrl.type, ctrl.arg);
	}

	default:
		return -ENOTTY;
	}
}

static int tz_telemetry_open(struct inode *inode, struct file *filp)
{
	if (!tzdev_is_up())
		return -EPERM;

	return 0;
}

static const struct file_operations tz_telemetry_fops = {
	.owner = THIS_MODULE,
	.open = tz_telemetry_open,
	.unlocked_ioctl = tz_telemetry_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tz_telemetry_unlocked_ioctl,
#endif /* CONFIG_COMPAT */
};

static struct tz_cdev tz_telemetry_cdev = {
	.name = TZ_TELEMETRY_DEVICE_NAME,
	.fops = &tz_telemetry_fops,
	.owner = THIS_MODULE,
};

static int __init tz_telemetry_init(void)
{
	int rc;

	rc = tz_cdev_register(&tz_telemetry_cdev);

	return rc;
}

static void __exit tz_telemetry_exit(void)
{
	tz_cdev_unregister(&tz_telemetry_cdev);
}

module_init(tz_telemetry_init);
module_exit(tz_telemetry_exit);
