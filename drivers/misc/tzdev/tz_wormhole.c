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

#include <linux/anon_inodes.h>
#include <linux/atomic.h>
#include <linux/cred.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "lib/circ_buf.h"
#include "lib/circ_buf_packet.h"

#include <tzdev/tzdev.h>

#include "tz_cdev.h"
#include "tz_wormhole.h"
#include "tzdev.h"

MODULE_AUTHOR("Artem Kuzin <a.kuzin@samsung.com>");
MODULE_DESCRIPTION("Socket-like device for messages tunneling between system and vendor Android parts");
MODULE_LICENSE("GPL");

#define TZ_WORMHOLE_DEVICE_NAME "tz_wormhole"
#define TZ_WORMHOLE_DATA_BUFFER_SIZE	1024

enum {
	TZ_WORMHOLE_BUFFER_NEW,
	TZ_WORMHOLE_BUFFER_CONNECTED,
	TZ_WORMHOLE_BUFFER_CLOSED,
};

struct tz_wormhole_cred {
	uint32_t pid;
	uint32_t uid;
	uint32_t gid;
};

struct tz_wormhole_desc {
	struct circ_buf *buffer[2];
	unsigned int state[2];

	struct tz_wormhole_cred cred;
	atomic_t ref_counter;
	wait_queue_head_t wq;
};

struct tz_wormhole {
	struct circ_buf_desc write_buffer;
	struct circ_buf_desc read_buffer;
	unsigned int *state;
	unsigned int *peer_state;

	struct mutex mutex;
	struct tz_wormhole_desc *desc;
};

static struct tz_wormhole_desc *public_desc;
static DEFINE_SPINLOCK(publish_lock);

static DEFINE_MUTEX(connection_mutex);

static DECLARE_WAIT_QUEUE_HEAD(tz_wormhole_accept_wq);

static struct tz_wormhole_desc *tz_wormhole_desc_create(void)
{
	struct tz_wormhole_desc *hole_desc;

	hole_desc = kzalloc(sizeof(struct tz_wormhole_desc), GFP_KERNEL);
	if (!hole_desc)
		return NULL;

	hole_desc->buffer[0] = circ_buf_alloc(TZ_WORMHOLE_DATA_BUFFER_SIZE);
	if (!hole_desc->buffer[0])
		goto free_desc;

	hole_desc->buffer[1] = circ_buf_alloc(TZ_WORMHOLE_DATA_BUFFER_SIZE);
	if (!hole_desc->buffer[0])
		goto free_buffer;


	hole_desc->state[0] = TZ_WORMHOLE_BUFFER_NEW;
	hole_desc->state[1] = TZ_WORMHOLE_BUFFER_NEW;

	hole_desc->cred.pid = current->tgid;
	hole_desc->cred.uid = __kuid_val(current_uid());
	hole_desc->cred.gid = __kgid_val(current_gid());

	init_waitqueue_head(&hole_desc->wq);
	atomic_set(&hole_desc->ref_counter, 1);

	return hole_desc;

free_buffer:
	circ_buf_free(hole_desc->buffer[0]);
free_desc:
	kfree(hole_desc);

	return NULL;
}

static void tz_wormhole_desc_destroy(struct tz_wormhole_desc *desc)
{
	circ_buf_free(desc->buffer[0]);
	circ_buf_free(desc->buffer[1]);
	kfree(desc);
}

static void tz_wormhole_desc_get(struct tz_wormhole_desc *desc)
{
	atomic_inc(&desc->ref_counter);
	tzdev_print(2, "Desc %p ref counter %d\n", desc, atomic_read(&desc->ref_counter));
}

static void tz_wormhole_desc_put(struct tz_wormhole_desc *desc)
{
	int ref_counter = atomic_sub_return(1, &desc->ref_counter);

	if (!ref_counter) {
		tzdev_print(2, "Destroy  %p\n", desc);
		tz_wormhole_desc_destroy(desc);
	} else {
		tzdev_print(2, "Desc %p ref counter %d\n", desc, ref_counter);
	}
}

static void tz_wormhole_publish_desc(struct tz_wormhole_desc *desc)
{
	tz_wormhole_desc_get(desc);

	spin_lock(&publish_lock);

	public_desc = desc;

	spin_unlock(&publish_lock);
}

static void tz_wormhole_unpublish_desc(struct tz_wormhole_desc *desc)
{
	spin_lock(&publish_lock);

	public_desc = NULL;

	spin_unlock(&publish_lock);

	tz_wormhole_desc_put(desc);
}

static struct tz_wormhole_desc *tz_wormhole_get_public_desc(void)
{
	struct tz_wormhole_desc *desc;

	spin_lock(&publish_lock);

	if (public_desc)
		tz_wormhole_desc_get(public_desc);

	desc = public_desc;

	spin_unlock(&publish_lock);

	return desc;
}

static int tz_wormhole_connection_done(struct tz_wormhole_desc *hole_desc)
{
	int ret;

	smp_rmb();

	if (!tzdev_is_opened())
		return -ECONNREFUSED;

	switch (hole_desc->state[1]) {
	case TZ_WORMHOLE_BUFFER_NEW:
		ret = -EAGAIN;
		break;
	case TZ_WORMHOLE_BUFFER_CONNECTED:
	case TZ_WORMHOLE_BUFFER_CLOSED:
		ret = 0;
		break;
	default:
		BUG();
	}

	return ret;
}

static int tz_wormhole_connect(struct tz_wormhole_desc *hole_desc)
{
	int ret, res;

	mutex_lock(&connection_mutex);
	tz_wormhole_publish_desc(hole_desc);

	wake_up(&tz_wormhole_accept_wq);
	res = wait_event_interruptible(hole_desc->wq,
			(ret = tz_wormhole_connection_done(hole_desc)) != -EAGAIN);
	if (res)
		ret = res;

	tz_wormhole_unpublish_desc(hole_desc);
	mutex_unlock(&connection_mutex);

	return ret;
}

static int tz_wormhole_open(struct inode *inode, struct file *filp)
{
	int ret;
	struct tz_wormhole *hole;
	struct tz_wormhole_desc *hole_desc;

	hole = kzalloc(sizeof(struct tz_wormhole), GFP_KERNEL);
	if (!hole) {
		tzdev_print(0, "Failed to allocate new hole\n");
		return -ENOMEM;
	}

	hole_desc = tz_wormhole_desc_create();
	if (!hole_desc) {
		ret = -ENOMEM;
		tzdev_print(0, "Failed to create hole descriptor\n");
		goto release_hole;
	}

	ret = tz_wormhole_connect(hole_desc);
	if (ret) {
		tzdev_print(0, "Failed to connect to server hole, %d\n", ret);
		goto release_hole_desc;
	}

	circ_buf_connect(&hole->write_buffer, hole_desc->buffer[0], TZ_WORMHOLE_DATA_BUFFER_SIZE);
	circ_buf_connect(&hole->read_buffer, hole_desc->buffer[1], TZ_WORMHOLE_DATA_BUFFER_SIZE);
	mutex_init(&hole->mutex);

	hole->state = &hole_desc->state[0];
	hole->peer_state = &hole_desc->state[1];
	hole->desc = hole_desc;

	*hole->state = TZ_WORMHOLE_BUFFER_CONNECTED;
	smp_wmb();

	tzdev_print(2, "Created new hole %p\n", hole);
	filp->private_data = hole;

	return 0;

release_hole_desc:
	hole_desc->state[0] = TZ_WORMHOLE_BUFFER_CLOSED;
	smp_wmb();

	wake_up(&hole_desc->wq);

	tz_wormhole_desc_put(hole_desc);
release_hole:
	kfree(hole);

	return ret;
}

static ssize_t __tz_wormhole_write(struct tz_wormhole *hole, const char __user *buffer, size_t size)
{
	int ret;

	mutex_lock(&hole->mutex);

	if (*hole->peer_state == TZ_WORMHOLE_BUFFER_CLOSED) {
		ret = -ECONNRESET;
		goto unlock;
	}

	if (size && size > TZ_WORMHOLE_DATA_BUFFER_SIZE) {
		ret = -EMSGSIZE;
		goto unlock;
	}

	smp_rmb();

	ret = circ_buf_write(&hole->write_buffer,
			(char *)buffer, size, CIRC_BUF_MODE_USER);

unlock:
	mutex_unlock(&hole->mutex);

	return ret;
}

static ssize_t tz_wormhole_write(struct file *filp, const char __user *buf, size_t size, loff_t *ppos)
{
	int ret, res = 0;
	struct tz_wormhole *hole = filp->private_data;
	struct tz_wormhole_desc *hole_desc = hole->desc;

	tzdev_print(2, "Going to write %zd bytes, hole desc %p\n", size, hole_desc);

	if (filp->f_flags & O_NONBLOCK) {
		ret = __tz_wormhole_write(hole, buf, size);
	} else {
		res = wait_event_interruptible(hole_desc->wq, (ret = __tz_wormhole_write(hole, buf, size)) != -EAGAIN);
		if (res)
			ret = res;
	}

	if (ret > 0) {
		tzdev_print(2, "Written %d bytes, hole desc %p\n", ret, hole_desc);
		wake_up(&hole_desc->wq);
	} else {
		tzdev_print(2, "Failed to write %zd bytes with error %d, hole desc %p\n", size, ret, hole_desc);
	}

	return ret;
}

static ssize_t __tz_wormhole_read(struct tz_wormhole *hole, char __user *buffer, size_t size)
{
	int ret;

	mutex_lock(&hole->mutex);

	smp_rmb();

	if (size && size > TZ_WORMHOLE_DATA_BUFFER_SIZE) {
		ret = -EINVAL;
		goto unlock;
	}

	ret = circ_buf_read(&hole->read_buffer,
			(char *)buffer, size, CIRC_BUF_MODE_USER);

	if (ret == -EAGAIN) {
		if (*hole->peer_state == TZ_WORMHOLE_BUFFER_CLOSED) {
			if (circ_buf_is_empty(&hole->write_buffer) && circ_buf_is_empty(&hole->read_buffer))
				ret = 0;
			else if (!circ_buf_is_empty(&hole->write_buffer) && circ_buf_is_empty(&hole->read_buffer))
				ret = -ECONNRESET;
		}
	}

unlock:
	mutex_unlock(&hole->mutex);

	return ret;
}

static ssize_t tz_wormhole_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	int ret, res;
	struct tz_wormhole *hole = filp->private_data;
	struct tz_wormhole_desc *hole_desc = hole->desc;

	tzdev_print(2, "Going to read %zd bytes, hole desc %p\n", size, hole_desc);

	if (filp->f_flags & O_NONBLOCK) {
		ret = __tz_wormhole_read(hole, buf, size);
	} else {
		res = wait_event_interruptible(hole_desc->wq, (ret = __tz_wormhole_read(hole, buf, size)) != -EAGAIN);
		if (res)
			ret = res;
	}

	if (ret > 0) {
		tzdev_print(2, "Read %d bytes, hole desc %p\n", ret, hole_desc);
		wake_up(&hole_desc->wq);
	} else {
		tzdev_print(2, "Failed to read %zd bytes with error %d, hole desc %p\n", size, ret, hole_desc);
	}

	return ret;
}

static unsigned int tz_wormhole_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct tz_wormhole *hole = filp->private_data;

	poll_wait(filp, &hole->desc->wq, wait);

	switch (*hole->state) {
	case TZ_WORMHOLE_BUFFER_NEW:
		break;
	case TZ_WORMHOLE_BUFFER_CONNECTED:
		if (!circ_buf_is_empty(&hole->read_buffer))
			mask |= POLLIN | POLLRDNORM;

		switch (*hole->peer_state) {
		case TZ_WORMHOLE_BUFFER_NEW:
			break;
		case TZ_WORMHOLE_BUFFER_CONNECTED:
			if (!circ_buf_is_full(&hole->write_buffer))
				mask |= POLLOUT | POLLWRNORM;
			break;
		case TZ_WORMHOLE_BUFFER_CLOSED:
			mask |= POLLHUP;
			break;
		}
		break;
	case TZ_WORMHOLE_BUFFER_CLOSED:
		BUG();
	}

	return mask;
}

static int tz_wormhole_release(struct inode *inode, struct file *filp)
{
	struct tz_wormhole *hole = filp->private_data;
	struct tz_wormhole_desc *hole_desc = hole->desc;

	*hole->state = TZ_WORMHOLE_BUFFER_CLOSED;
	smp_wmb();

	wake_up(&hole_desc->wq);

	tz_wormhole_desc_put(hole_desc);
	kfree(hole);

	return 0;
}

static long tz_wormhole_unlocked_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	switch (cmd) {
	case TZ_WORMHOLE_GET_CRED: {
		struct tz_wormhole_cred __user *user_cred;
		struct tz_wormhole *hole = (filp->private_data);

		user_cred = (struct tz_wormhole_cred __user *)arg;

		if (copy_to_user(user_cred, &hole->desc->cred, sizeof(struct tz_wormhole_cred)))
			return -EFAULT;

		return 0;
	}
	default:
		return -ENOTTY;
	}
}

static const struct file_operations tz_wormhole_fops = {
	.owner = THIS_MODULE,
	.open = tz_wormhole_open,
	.unlocked_ioctl = tz_wormhole_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tz_wormhole_unlocked_ioctl,
#endif /* CONFIG_COMPAT */
	.write = tz_wormhole_write,
	.read = tz_wormhole_read,
	.poll = tz_wormhole_poll,
	.release = tz_wormhole_release,
};

static struct tz_cdev tz_wormhole_cdev = {
	.name = TZ_WORMHOLE_DEVICE_NAME,
	.fops = &tz_wormhole_fops,
	.owner = THIS_MODULE,
};

static int __init tz_wormhole_init(void)
{
	int rc;

	rc = tz_cdev_register(&tz_wormhole_cdev);

	return rc;
}

static void __exit tz_wormhole_exit(void)
{
	tz_cdev_unregister(&tz_wormhole_cdev);
}

static struct tz_wormhole_desc *tz_wormhole_get_accepted_desc(void)
{
	struct tz_wormhole_desc *accepted_hole_desc, *hole_desc;

	hole_desc = tz_wormhole_get_public_desc();
	if (!hole_desc)
		return NULL;

	smp_rmb();

	switch (hole_desc->state[0]) {
	case TZ_WORMHOLE_BUFFER_NEW:
		accepted_hole_desc = hole_desc;
		break;
	case TZ_WORMHOLE_BUFFER_CLOSED:
		accepted_hole_desc = NULL;
		tz_wormhole_desc_put(hole_desc);
		break;
	case TZ_WORMHOLE_BUFFER_CONNECTED:
	default:
		BUG();
	}

	return accepted_hole_desc;
}

int tz_wormhole_tzdev_accept(void)
{
	int ret;
	struct tz_wormhole *hole;
	struct tz_wormhole_desc *hole_desc;

	hole_desc = tz_wormhole_get_accepted_desc();
	if (!hole_desc) {
		tzdev_print(0, "Failed to get hole descriptor to accept\n");
		return -ENOENT;
	}

	hole = kzalloc(sizeof(struct tz_wormhole), GFP_KERNEL);
	if (!hole) {
		ret = -ENOMEM;
		tzdev_print(0, "Failed to allocate peer hole\n");
		goto notify_peer;
	}

	circ_buf_connect(&hole->write_buffer, hole_desc->buffer[1], TZ_WORMHOLE_DATA_BUFFER_SIZE);
	circ_buf_connect(&hole->read_buffer, hole_desc->buffer[0], TZ_WORMHOLE_DATA_BUFFER_SIZE);
	mutex_init(&hole->mutex);

	hole->state = &hole_desc->state[1];
	hole->peer_state = &hole_desc->state[0];
	hole->desc = hole_desc;

	ret = anon_inode_getfd("[tz-wormhole]", &tz_wormhole_fops, hole, O_CLOEXEC | O_RDWR);
	if (ret < 0) {
		tzdev_print(0, "Failed to create anon fd for peer hole %d\n", ret);
		goto free_hole;
	}

	*hole->state = TZ_WORMHOLE_BUFFER_CONNECTED;
	smp_wmb();

	tzdev_print(2, "Created new peer hole %p\n", hole);
	wake_up(&hole_desc->wq);

	return ret;

free_hole:
	kfree(hole);
notify_peer:
	hole_desc->state[1] = TZ_WORMHOLE_BUFFER_CLOSED;
	smp_wmb();

	wake_up(&hole_desc->wq);

	tz_wormhole_desc_put(hole_desc);

	return ret;
}

void tz_wormhole_close_connection(void)
{
	struct tz_wormhole_desc *hole_desc;

	hole_desc = tz_wormhole_get_accepted_desc();
	if (!hole_desc) {
		tzdev_print(0, "Failed to get hole descriptor to close\n");
		return;
	}

	hole_desc->state[1] = TZ_WORMHOLE_BUFFER_CLOSED;
	smp_wmb();
	tzdev_print(2, "Connection closed due to tzdev is going down\n");

	wake_up(&hole_desc->wq);

	tz_wormhole_desc_put(hole_desc);
}

unsigned int tz_wormhole_tzdev_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &tz_wormhole_accept_wq, wait);

	spin_lock(&publish_lock);

	if (public_desc && public_desc->state[1] == TZ_WORMHOLE_BUFFER_NEW)
		mask |= POLLIN | POLLRDNORM;

	spin_unlock(&publish_lock);

	return mask;
}

module_init(tz_wormhole_init);
module_exit(tz_wormhole_exit);
