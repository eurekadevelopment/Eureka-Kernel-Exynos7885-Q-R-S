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

#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include "tzirs.h"
#include "tz_cdev.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Oleksii Mosolab");

static int tzirs_fd_open;
static DEFINE_MUTEX(tzirs_lock);

static int tzirs_open(struct inode *n, struct file *f)
{
	int ret = 0;

	mutex_lock(&tzirs_lock);
	if (tzirs_fd_open) {
		ret = -EBUSY;
		goto out;
	}
	tzirs_fd_open++;
	DBG("open\n");

out:
	mutex_unlock(&tzirs_lock);
	return ret;
}

static inline int tzirs_release(struct inode *inode, struct file *file)
{
	mutex_lock(&tzirs_lock);
	if (tzirs_fd_open)
		tzirs_fd_open--;
	mutex_unlock(&tzirs_lock);
	DBG("release\n");
	return 0;
}

static long tzirs_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned long p1, p2, p3;

	struct irs_ctx __user *ioargp = (struct irs_ctx __user *) arg;
	struct irs_ctx ctx = {0};

	if (_IOC_TYPE(cmd) != IOC_MAGIC) {
		ERR("INVALID CMD = %d\n", cmd);
		return -ENOTTY;
	}
	switch (cmd) {
	case IOCTL_IRS_CMD:
		DBG("IOCT_IRS_CMD\n");
		/* get flag id */
		ret = copy_from_user(&ctx, ioargp, sizeof(struct irs_ctx));
		if (ret != 0) {
			ERR("IRS_CMD copy_from_user failed, ret = 0x%08x\n", ret);
			return -EFAULT;
		}

		p1 = ctx.id;
		p2 = ctx.value;
		p3 = ctx.func_cmd;

		DBG("IRS_CMD before: id = 0x%lx, value = 0x%lx, cmd = 0x%lx\n", (unsigned long)p1, (unsigned long)p2, (unsigned long)p3);

		ret = tzirs_smc(&p1, &p2, &p3);

		DBG("IRS_CMD after: id = 0x%lx, value = 0x%lx, cmd = 0x%lx\n", (unsigned long)p1, (unsigned long)p2, (unsigned long)p3);

		if (ret) {
			ERR("Unable to send IRS_CMD : id = 0x%lx, ret = %d\n", (unsigned long)p1, ret);
			return -EFAULT;
		}

		ctx.id = p1;
		ctx.value = p2;
		ctx.func_cmd = p3;

		ret = copy_to_user(ioargp, &ctx, sizeof(struct irs_ctx));
		if (ret != 0) {
			ERR("IRS_CMD copy_to_user failed, ret = 0x%08x\n", ret);
			return -EFAULT;
		}
		break;

	default:
		ERR("UNKNOWN CMD, cmd = 0x%08x\n", cmd);
		return -ENOTTY;
	}

	return 0;
}

static const struct file_operations tzirs_fops = {
	.owner = THIS_MODULE,
	.open = tzirs_open,
	.unlocked_ioctl = tzirs_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tzirs_unlocked_ioctl,
#endif /* CONFIG_COMPAT */
	.release = tzirs_release,
};

static struct tz_cdev tzirs_cdev = {
	.name = TZIRS_NAME,
	.fops = &tzirs_fops,
	.owner = THIS_MODULE,
};

static int __init tzirs_init(void)
{
	int rc;

	rc = tz_cdev_register(&tzirs_cdev);
	if (rc) {
		ERR("Unable to register TZIRS driver, rc = 0x%08x\n", rc);
		return rc;
	}

	DBG("INSTALLED\n");
	return 0;
}

static void __exit tzirs_exit(void)
{
	tz_cdev_unregister(&tzirs_cdev);
	DBG("REMOVED\n");
}

module_init(tzirs_init);
module_exit(tzirs_exit);
