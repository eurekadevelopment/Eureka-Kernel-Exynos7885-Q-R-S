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

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pid.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "tzlog.h"
#include "tz_cdev.h"
#include "tz_shmem_validator.h"

MODULE_AUTHOR("Pavel Bogachev <p.bogachev@partner.samsung.com>");
MODULE_DESCRIPTION("Trustzone shared memory registration requests validator");
MODULE_LICENSE("GPL");

#define TZ_SHMEM_VALIDATOR_DEVICE_NAME "tz_shmem_validator"

struct task_reg_entry {
	struct list_head link;
	pid_t tgid;
	unsigned int allow_any;
	struct list_head shmem_list;
	atomic_t ref_count;
};

struct shmem_reg_entry {
	struct list_head link;
	unsigned long start;
	unsigned long size;
};

static DEFINE_MUTEX(tz_shmem_validator_mutex);
static LIST_HEAD(tz_shmem_validator_task_reg_list);

int tz_shmem_validator_approve(pid_t tgid, unsigned long start, unsigned long size)
{
	struct task_reg_entry *task_reg_entry;
	struct shmem_reg_entry *shmem_entry;
	int ret = -EPERM;

	mutex_lock(&tz_shmem_validator_mutex);

	list_for_each_entry(task_reg_entry, &tz_shmem_validator_task_reg_list, link) {
		if (task_reg_entry->tgid != tgid)
			continue;

		if (task_reg_entry->allow_any) {
			ret = 0;
			break;
		}

		list_for_each_entry(shmem_entry, &task_reg_entry->shmem_list, link) {
			if (shmem_entry->start != start || shmem_entry->size != size)
				continue;

			list_del(&shmem_entry->link);
			kfree(shmem_entry);
			ret = 0;
			break;
		}

		break;
	}

	mutex_unlock(&tz_shmem_validator_mutex);

	return ret;
}

static struct task_reg_entry *tz_shmem_validator_alloc_task_reg_entry(void)
{
	struct task_reg_entry *entry;

	entry = kmalloc(sizeof(struct task_reg_entry), GFP_KERNEL);
	if (!entry)
		return NULL;

	memset(entry, 0, sizeof(struct task_reg_entry));

	INIT_LIST_HEAD(&entry->link);
	INIT_LIST_HEAD(&entry->shmem_list);
	atomic_set(&entry->ref_count, 1);

	return entry;
}

static struct shmem_reg_entry *tz_shmem_validator_alloc_shmem_entry(void)
{
	struct shmem_reg_entry *entry;

	entry = kmalloc(sizeof(struct shmem_reg_entry), GFP_KERNEL);
	if (!entry)
		return NULL;

	memset(entry, 0, sizeof(struct shmem_reg_entry));

	INIT_LIST_HEAD(&entry->link);

	return entry;
}

static int tz_shmem_validator_register_task(void)
{
	struct task_reg_entry *entry;
	int ret = 0;
	pid_t tgid = current->tgid;

	mutex_lock(&tz_shmem_validator_mutex);

	list_for_each_entry(entry, &tz_shmem_validator_task_reg_list, link) {
		if (entry->tgid == tgid) {
			atomic_inc(&entry->ref_count);
			goto out;
		}
	}

	entry = tz_shmem_validator_alloc_task_reg_entry();
	if (!entry) {
		ret = -ENOMEM;
		goto out;
	}

	entry->tgid = tgid;

	list_add(&entry->link, &tz_shmem_validator_task_reg_list);

out:
	mutex_unlock(&tz_shmem_validator_mutex);

	return ret;
}

static int tz_shmem_validator_register_memory(
		struct tz_shmem_desc *shmem_desc_user)
{
	int ret = 0;
	pid_t tgid = current->tgid;
	struct shmem_reg_entry *shmem_entry;
	struct task_reg_entry *task_reg_entry;
	struct tz_shmem_desc shmem_desc;
	unsigned int found = 0;

	if (copy_from_user(&shmem_desc, shmem_desc_user, sizeof(struct tz_shmem_desc)))
		return -EFAULT;

	mutex_lock(&tz_shmem_validator_mutex);

	list_for_each_entry(task_reg_entry, &tz_shmem_validator_task_reg_list, link) {
		if (tgid == task_reg_entry->tgid) {
			found = 1;
			break;
		}
	}

	if (!found) {
		ret = -ENOENT;
		goto out;
	}

	shmem_entry = tz_shmem_validator_alloc_shmem_entry();
	if (!shmem_entry) {
		ret = -ENOMEM;
		goto out;
	}

	shmem_entry->start = shmem_desc.ptr;
	shmem_entry->size = shmem_desc.size;

	list_add(&shmem_entry->link, &task_reg_entry->shmem_list);

out:
	mutex_unlock(&tz_shmem_validator_mutex);

	return ret;
}

static int tz_shmem_validator_allow_any(void)
{
	int ret = -ENOENT;
	pid_t tgid = current->tgid;
	struct task_reg_entry *task_reg_entry;

	mutex_lock(&tz_shmem_validator_mutex);

	list_for_each_entry(task_reg_entry, &tz_shmem_validator_task_reg_list, link) {
		if (tgid == task_reg_entry->tgid) {
			task_reg_entry->allow_any = 1;
			ret = 0;
			break;
		}
	}

	mutex_unlock(&tz_shmem_validator_mutex);

	return ret;
}

static long tz_shmem_validator_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	switch (cmd) {
	case TZ_SHMEM_VALIDATOR_REGISTER_MEMORY:
		return tz_shmem_validator_register_memory((struct tz_shmem_desc *) arg);

	case TZ_SHMEM_VALIDATOR_ALLOW_ANY:
		return tz_shmem_validator_allow_any();

	default:
		break;
	}

	return -ENOTTY;
}

static void __tz_shmem_validator_clean_task_data(struct task_reg_entry *task_reg_entry)
{
	struct shmem_reg_entry *shmem_entry, *tmp;

	list_for_each_entry_safe(shmem_entry, tmp, &task_reg_entry->shmem_list, link)
		kfree(shmem_entry);

	INIT_LIST_HEAD(&task_reg_entry->shmem_list);
}

static int tz_shmem_validator_open(struct inode *inode, struct file *filp)
{
	int ret;

	ret = tz_shmem_validator_register_task();
	if (!ret)
		filp->private_data = (void *) (unsigned long) current->tgid;

	return ret;
}

static int tz_shmem_validator_release(struct inode *inode, struct file *filp)
{
	pid_t tgid;
	struct task_reg_entry *entry, *tmp;

	if (!filp->private_data)
		return 0;

	tgid = (unsigned long) filp->private_data;

	mutex_lock(&tz_shmem_validator_mutex);

	/* When tz_shmem_validator file is closed by some task, we release all data that
	 * this task has registered. */
	list_for_each_entry_safe(entry, tmp, &tz_shmem_validator_task_reg_list, link) {
		if (entry->tgid == tgid) {
			if (atomic_dec_and_test(&entry->ref_count)) {
				__tz_shmem_validator_clean_task_data(entry);
				list_del(&entry->link);
				kfree(entry);
			}
		}
	}

	mutex_unlock(&tz_shmem_validator_mutex);

	return 0;
}

static const struct file_operations tz_shmem_validator_fops = {
	.owner = THIS_MODULE,
	.open = tz_shmem_validator_open,
	.release = tz_shmem_validator_release,
	.unlocked_ioctl = tz_shmem_validator_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tz_shmem_validator_unlocked_ioctl,
#endif /* CONFIG_COMPAT */
};

static struct tz_cdev tz_shmem_validator_cdev = {
	.name = TZ_SHMEM_VALIDATOR_DEVICE_NAME,
	.fops = &tz_shmem_validator_fops,
	.owner = THIS_MODULE,
};

static int __init tz_shmem_validator_init(void)
{
	return tz_cdev_register(&tz_shmem_validator_cdev);
}

static void __exit tz_shmem_validator_exit(void)
{
	tz_cdev_unregister(&tz_shmem_validator_cdev);
}

module_init(tz_shmem_validator_init);
module_exit(tz_shmem_validator_exit);
