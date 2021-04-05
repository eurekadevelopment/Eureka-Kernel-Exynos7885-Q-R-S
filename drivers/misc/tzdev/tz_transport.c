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
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <tzdev/tzdev.h>

#include "tzdev.h"
#include "tz_iwcbuf.h"
#include "tz_cdev.h"
#include "tz_iwcbuf.h"
#include "tz_iwio.h"
#include "tz_transport.h"

MODULE_AUTHOR("Pavel Bogachev <p.bogachev@partner.samsung.com>");
MODULE_DESCRIPTION("Trustzone transport driver");
MODULE_LICENSE("GPL");

#define TZ_TRANSPORT_DEVICE_NAME "tz_transport"

#define TZDEV_TRANSPORT_BUF_SIZE	(CONFIG_TZ_TRANSPORT_PG_CNT * PAGE_SIZE - sizeof(struct tz_iwcbuf))

static DEFINE_MUTEX(tz_transport_init_mutex);
static DEFINE_SPINLOCK(tz_transport_slock);
static DEFINE_PER_CPU(struct tz_iwcbuf *, tz_transport_buffer);

enum tz_transport_state {
	NOT_INITIALIZED,
	INITIALIZED,
	PANICKED,
};

static enum tz_transport_state tz_transport_state = NOT_INITIALIZED;

static long tz_transport_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	switch (cmd) {
	case TZ_TRANSPORT_GET_BUF_SIZE: {
		__u32 __user *argp = (__u32 *) arg;
		__u32 size;

		size = TZDEV_TRANSPORT_BUF_SIZE;

		return put_user(size, argp);
	}

	default:
		return -ENOTTY;
	}
}

static ssize_t tz_transport_read_buffer(struct tz_iwcbuf *buf,
		char __user *buffer, size_t size)
{
	unsigned int count, write_count, avail;
	size_t total = 0;

	write_count = buf->write_count;

	if (write_count == buf->read_count)
		return 0;

	if (write_count < buf->read_count)
		avail = TZDEV_TRANSPORT_BUF_SIZE - buf->read_count + write_count;
	else
		avail = write_count - buf->read_count;

	if (size < avail)
		return 0;

	if (write_count < buf->read_count) {
		count = TZDEV_TRANSPORT_BUF_SIZE - buf->read_count;

		if (copy_to_user(buffer, buf->buffer + buf->read_count, count))
			return -EFAULT;

		buf->read_count = 0;
		buffer += count;
		total += count;
	}

	count = write_count - buf->read_count;
	if (copy_to_user(buffer, buf->buffer + buf->read_count, count))
		return -EFAULT;

	buf->read_count += count;
	total += count;

	return total;
}

static ssize_t tz_transport_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct tz_iwcbuf *iwc_buf;
	unsigned int i;
	ssize_t position = 0;
	ssize_t ret;

	(void) file;
	(void) ppos;

	if (count == 0)
		return 0;

	spin_lock(&tz_transport_slock);

	for (i = 0; i < NR_SW_CPU_IDS; ++i) {
		iwc_buf = per_cpu(tz_transport_buffer, i);
		if (!iwc_buf)
			continue;

		ret = tz_transport_read_buffer(iwc_buf, buf + position,
				count - position);
		if (ret < 0) {
			spin_unlock(&tz_transport_slock);
			return ret;
		}

		position += ret;
	}

	spin_unlock(&tz_transport_slock);

	return position;
}

static int tz_transport_alloc_buffers(void)
{
	unsigned int i;

	for (i = 0; i < NR_SW_CPU_IDS; ++i) {
		struct tz_iwcbuf *buffer = tz_iwio_alloc_iw_channel(
				TZ_IWIO_CONNECT_TRANSPORT,
				CONFIG_TZ_TRANSPORT_PG_CNT);
		if (!buffer)
			return -ENOMEM;

		per_cpu(tz_transport_buffer, i) = buffer;
	}

	return 0;
}

static int tz_transport_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	if (!tzdev_is_up())
		return -EPERM;

	mutex_lock(&tz_transport_init_mutex);
	if (tz_transport_state == PANICKED) {
		ret = -ESHUTDOWN;
		goto out;
	}

	if (tz_transport_state == INITIALIZED)
		goto out;

	ret = tz_transport_alloc_buffers();
	if (ret)
		tz_transport_state = PANICKED;
	else
		tz_transport_state = INITIALIZED;

out:
	mutex_unlock(&tz_transport_init_mutex);

	return ret;
}

static const struct file_operations tz_transport_fops = {
	.owner = THIS_MODULE,
	.open = tz_transport_open,
	.read = tz_transport_read,
	.unlocked_ioctl = tz_transport_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tz_transport_unlocked_ioctl,
#endif /* CONFIG_COMPAT */
};

static struct tz_cdev tz_transport_cdev = {
	.name = TZ_TRANSPORT_DEVICE_NAME,
	.fops = &tz_transport_fops,
	.owner = THIS_MODULE,
};

static int __init tz_transport_init(void)
{
	int rc;

	rc = tz_cdev_register(&tz_transport_cdev);

	return rc;
}

static void __exit tz_transport_exit(void)
{
	tz_cdev_unregister(&tz_transport_cdev);
}

module_init(tz_transport_init);
module_exit(tz_transport_exit);
