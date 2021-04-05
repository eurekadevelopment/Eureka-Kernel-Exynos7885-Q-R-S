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

#include <linux/completion.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/printk.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <tzdev/kernel_api.h>

#include "tzdev.h"
#include "tzprofiler.h"
#include "tz_cdev.h"
#include "tz_iwcbuf.h"
#include "tz_iwio.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eugene Mandrenko <i.mandrenko@samsung.com>");
MODULE_DESCRIPTION("Trustzone profiler driver");

enum {
	TZPROFILER_LIST1_NEED_CLEAN = 0,
	TZPROFILER_LIST2_NEED_CLEAN
};

static DECLARE_COMPLETION(tzprofiler_pool_completion);

LIST_HEAD(profiler_buf_list1);
LIST_HEAD(profiler_buf_list2);
static int current_full_pool;

static atomic_t sk_is_active_count = ATOMIC_INIT(0);
static atomic_t tzprofiler_data_is_being_written = ATOMIC_INIT(0);
static atomic_t tzprofiler_last_passing = ATOMIC_INIT(0);

static DECLARE_WAIT_QUEUE_HEAD(sk_wait);
static DECLARE_WAIT_QUEUE_HEAD(tzprofiler_data_writing);
static DEFINE_MUTEX(tzprofiler_mutex);

void tzprofiler_wait_for_bufs(void)
{
	int ret;

	if (atomic_read(&tzprofiler_data_is_being_written) == 0)
		return;
	//if (!in_atomic()) { //TODO: temp disable (SystemSW SCM error in kernel/samsung)
		ret = wait_event_interruptible(tzprofiler_data_writing,
				(atomic_read(&tzprofiler_data_is_being_written) == 0) || (atomic_read(&tzprofiler_last_passing) == 0));
		if (ret < 0) {
			tzdev_print(0, "%s:wait_event_interruptible_timeout tzprofiler_data_writing\n", __func__);
			atomic_set(&tzprofiler_data_is_being_written, 0);
		}
	//}
}

void tzprofiler_enter_sw(void)
{
	atomic_inc(&sk_is_active_count);
	atomic_set(&tzprofiler_last_passing, 2);
	wake_up(&sk_wait);
}

void tzprofiler_exit_sw(void)
{
	atomic_set(&tzprofiler_last_passing, 2);
	wake_up(&sk_wait);
	atomic_dec(&sk_is_active_count);
}

static int tzprofiler_init_buf_pool(unsigned int bufs_cnt, unsigned int buf_pg_cnt)
{
	struct profiler_buf_entry *profiler_buf;
	unsigned int i;

	for (i = 0; i < bufs_cnt; ++i) {
		profiler_buf = kmalloc(sizeof(struct profiler_buf_entry), GFP_KERNEL);
		if (profiler_buf == NULL)
			return -ENOMEM;
		profiler_buf->tzio_buf = tz_iwio_alloc_iw_channel(TZ_IWIO_CONNECT_PROFILER, buf_pg_cnt);
		if (profiler_buf->tzio_buf == NULL) {
			kfree(profiler_buf);

			return -ENXIO;
		}
		list_add(&profiler_buf->list, &profiler_buf_list1);
	}

	complete_all(&tzprofiler_pool_completion);

	return 0;
}

static int tzprofiler_set_depth(unsigned int depth)
{
	int ret;

	ret = tzdev_smc_profiler_control(TZDEV_PROFILER_CMD_SET_DEPTH, depth);
	if (ret == 0) {
		tzdev_print(0, "Set depth successfully\n");
	} else if (ret == -ENOSYS) {
		ret = 0;
		tzdev_print(0, " SWd is built without profilerr\n");
	} else {
		tzdev_print(0, "failed: depth setup error. ret = %d\n", ret);
	}

	return ret;
}

static int tzprofiler_start(void)
{
	int ret;

	ret = tzdev_smc_profiler_control(TZDEV_PROFILER_CMD_START, 0);
	if (ret == 0) {
		tzdev_print(0, "Profiler has started successfully\n");
	} else if (ret == -ENOSYS) {
		ret = 0;
		tzdev_print(0, "SWd is built without profiler\n");
	} else {
		tzdev_print(0, "failed: Profiler has not started. ret = %d\n", ret);
	}

	return ret;
}

static int tzprofiler_stop(void)
{
	int ret;

	ret = tzdev_smc_profiler_control(TZDEV_PROFILER_CMD_STOP, 0);
	if (ret == 0) {
		tzdev_print(0, "Profiler has stopped successfully\n");
	} else if (ret == -ENOSYS) {
		ret = 0;
		tzdev_print(0, "SWd is built without profiler\n");
	} else {
		tzdev_print(0, "failed: Profiler has not stopped. ret = %d\n", ret);
	}

	return ret;
}

static int tzprofiler_set_uuid(struct tz_uuid *uuid)
{
	int ret = 0;
	struct tz_iwio_aux_channel *ch;

	ch = tz_iwio_get_aux_channel();
	memcpy(ch->buffer, uuid, sizeof(struct tz_uuid));
	ret = tzdev_smc_profiler_control(TZDEV_PROFILER_CMD_SET_UUID, 0);
	tz_iwio_put_aux_channel();

	if (ret == 0) {
		tzdev_print(0, "Set uuid successfully\n");
	} else if (ret == -ENOSYS) {
		ret = 0;
		tzdev_print(0, "SWd is built without profiler\n");
	} else {
		tzdev_print(0, "failed: uuid setup error. ret = %d\n", ret);
	}

	return ret;
}

static int tzprofiler_set_start_addr(uint64_t *addr)
{
	int ret;
	struct tz_iwio_aux_channel *ch;

	ch = tz_iwio_get_aux_channel();
	memcpy(ch->buffer, addr, sizeof(uint64_t));
	ret = tzdev_smc_profiler_control(TZDEV_PROFILER_CMD_SET_START_ADDR, 0);
	tz_iwio_put_aux_channel();

	if (ret == 0) {
		tzdev_print(0, "Set start addr successfully\n");
	} else if (ret == -ENOSYS) {
		ret = 0;
		tzdev_print(0, "SWd is built without profiler\n");
	} else {
		tzdev_print(0, "failed: start addr setup error. ret = %d\n", ret);
	}

	return ret;
}

static int tzprofiler_set_stop_addr(uint64_t *addr)
{
	int ret;
	struct tz_iwio_aux_channel *ch;

	ch = tz_iwio_get_aux_channel();
	memcpy(ch->buffer, addr, sizeof(uint64_t));
	ret = tzdev_smc_profiler_control(TZDEV_PROFILER_CMD_SET_STOP_ADDR, 0);
	tz_iwio_put_aux_channel();

	if (ret == 0) {
		tzdev_print(0, "Set stop addr successfully\n");
	} else if (ret == -ENOSYS) {
		ret = 0;
		tzdev_print(0, "SWd is built without profiler\n");
	} else {
		tzdev_print(0, "failed: stop addr setup error. ret = %d\n", ret);
	}

	return ret;
}

static int tzprofiler_set_steps_number(uint32_t steps)
{
	int ret;
	struct tz_iwio_aux_channel *ch;

	ch = tz_iwio_get_aux_channel();
	memcpy(ch->buffer, &steps, sizeof(uint32_t));
	ret = tzdev_smc_profiler_control(TZDEV_PROFILER_CMD_SET_STEPS_NUMBER, 0);
	tz_iwio_put_aux_channel();

	if (ret == 0) {
		tzdev_print(0, "Set number of steps successfully\n");
	} else if (ret == -ENOSYS) {
		ret = 0;
		tzdev_print(0, "SWd is built without profiler\n");
	} else {
		tzdev_print(0, "failed: number of steps setup error. ret = %d\n", ret);
	}

	return ret;
}

int tzprofiler_initialize(void)
{
	int ret;

	ret = tzprofiler_init_buf_pool(CONFIG_TZPROFILER_BUFS_CNT, CONFIG_TZPROFILER_BUF_PG_CNT);
	if (ret) {
		if (ret != -ENXIO) {
			tzdev_print(0, "tzprofiler_init_buf_pool failed: %d\n", ret);
			return ret;
		} else {
			return 0;
		}
	}

	ret = tzprofiler_start();
	if (ret)
		tzdev_print(0, "tzprofiler_start failed: %d\n", ret);

	return ret;
}

static int tzprofiler_add_buffers(unsigned int number)
{
	int ret;

	ret = tzprofiler_stop();
	if (ret) {
		tzdev_print(0, "tzprofiler_stop failed: %d\n", ret);
		return ret;
	}

	ret = tzprofiler_init_buf_pool(number, CONFIG_TZPROFILER_BUF_PG_CNT);
	if (ret) {
		tzdev_print(0, "tzprofiler_init_buf_pool failed: %d\n", ret);
		return ret;
	}

	ret = tzprofiler_start();
	if (ret)
		tzdev_print(0, "tzprofiler_start failed: %d\n", ret);

	return ret;
}

static ssize_t tzprofiler_write_buf(unsigned char *s_buf, ssize_t quantity,
		unsigned char *d_buf, ssize_t *saved_count, size_t count)
{
	ssize_t real_saved;

	if ((*saved_count + quantity) <= count)
		real_saved = quantity;
	else
		real_saved = count - *saved_count;

	if (copy_to_user(&d_buf[*saved_count], s_buf, real_saved))
		tzdev_print(0, "%s:can't copy to user\n", __func__);

	*saved_count += real_saved;

	return real_saved;
}

static ssize_t __read(char __user *buf, size_t count, struct list_head *head,
		struct list_head *cleaned_head)
{
	struct tz_iwcbuf *tzio_buf;
	struct profiler_buf_entry *profiler_buf, *tmp;
	ssize_t bytes, quantity, write_count, saved_count = 0;

	list_for_each_entry_safe(profiler_buf, tmp, head, list) {
		tzio_buf = profiler_buf->tzio_buf;
		if (tzio_buf->read_count == tzio_buf->write_count) {
			list_del(&profiler_buf->list);
			list_add(&profiler_buf->list, cleaned_head);
			continue;
		}

		write_count = tzio_buf->write_count;
		if (write_count < tzio_buf->read_count) {
			quantity = TZDEV_PROFILER_BUF_SIZE - tzio_buf->read_count;
			bytes = tzprofiler_write_buf(tzio_buf->buffer + tzio_buf->read_count,
				quantity, buf, &saved_count, count);
			if (bytes < quantity) {
				tzio_buf->read_count += bytes;
				atomic_set(&tzprofiler_data_is_being_written, 1);
				return saved_count;
			}
			tzio_buf->read_count = 0;
		}
		quantity = write_count - tzio_buf->read_count;
		bytes = tzprofiler_write_buf(tzio_buf->buffer + tzio_buf->read_count, quantity,
						buf, &saved_count, count);
		if (bytes < quantity) {
			tzio_buf->read_count += bytes;
			atomic_set(&tzprofiler_data_is_being_written, 1);
			return saved_count;
		}
		tzio_buf->read_count += quantity;
		list_del(&profiler_buf->list);
		list_add(&profiler_buf->list, cleaned_head);
	}
	return saved_count;
}

static ssize_t tzprofiler_read(struct file *filp, char __user *buf, size_t count,
		loff_t *f_pos)
{
	struct list_head *current_head, *cleaned_head;
	size_t saved_count;
	int ret;

	atomic_set(&tzprofiler_data_is_being_written, 0);

	while (1) {
		if ((atomic_read(&sk_is_active_count) == 0) && (atomic_read(&tzprofiler_last_passing) == 0)) {
			ret = wait_event_interruptible_timeout(sk_wait,
					atomic_read(&sk_is_active_count) != 0, msecs_to_jiffies(5000));
			if (ret < 0)
				return (ssize_t)-EINTR;
		}

		ret = wait_for_completion_interruptible(&tzprofiler_pool_completion);
		if (ret < 0)
			return (ssize_t)-EINTR;

		if (current_full_pool == TZPROFILER_LIST1_NEED_CLEAN) {
			current_head = &profiler_buf_list1;
			cleaned_head = &profiler_buf_list2;
		} else {
			current_head = &profiler_buf_list2;
			cleaned_head = &profiler_buf_list1;
		}
		saved_count = __read(buf, count, current_head, cleaned_head);
		if (saved_count) {
			atomic_set(&tzprofiler_data_is_being_written, 1);
			return saved_count;
		}

		wake_up(&tzprofiler_data_writing);
		if (atomic_read(&tzprofiler_last_passing) > 0)
			atomic_dec(&tzprofiler_last_passing);

		if (current_full_pool == TZPROFILER_LIST1_NEED_CLEAN)
			current_full_pool = TZPROFILER_LIST2_NEED_CLEAN;
		else
			current_full_pool = TZPROFILER_LIST1_NEED_CLEAN;
	}

	return 0;
}

static int tzprofiler_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static long tzprofiler_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	uint64_t addr;
	struct tz_uuid uuid;

	switch (cmd) {
	case TZPROFILER_INCREASE_POOL:
		ret = tzprofiler_add_buffers(arg);
		break;
	case TZPROFILER_SET_DEPTH:
		ret = tzprofiler_set_depth(arg);
		break;
	case TZPROFILER_START:
		ret = tzprofiler_start();
		break;
	case TZPROFILER_STOP:
		ret = tzprofiler_stop();
		break;
	case TZPROFILER_SET_UUID:
		if (copy_from_user(&uuid, (void *)arg, sizeof(struct tz_uuid)))
			return -EFAULT;

		ret = tzprofiler_set_uuid(&uuid);
		break;
	case TZPROFILER_SET_START_ADDR:
		if (copy_from_user(&addr, (void *)arg, sizeof(uint64_t)))
			return -EFAULT;

		ret = tzprofiler_set_start_addr(&addr);
		break;
	case TZPROFILER_SET_STOP_ADDR:
		if (copy_from_user(&addr, (void *)arg, sizeof(uint64_t)))
			return -EFAULT;

		ret = tzprofiler_set_stop_addr(&addr);
		break;
	case TZPROFILER_SET_STEPS_NUMBER:
		ret = tzprofiler_set_steps_number(arg);
		break;
	default:
		ret = -ENOTTY;
		tzdev_print(0, "Unexpected command %d\n", cmd);
		break;
	}

	return ret;
}

static int tzprofiler_release(struct inode *inode, struct file *filp)
{
	atomic_set(&tzprofiler_data_is_being_written, 0);
	wake_up(&tzprofiler_data_writing);

	return 0;
}

static const struct file_operations tzprofiler_fops = {
	.owner = THIS_MODULE,
	.read = tzprofiler_read,
	.open = tzprofiler_open,
	.unlocked_ioctl = tzprofiler_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tzprofiler_unlocked_ioctl,
#endif /* CONFIG_COMPAT */
	.release = tzprofiler_release,
};

static struct tz_cdev tzprofiler_cdev = {
	.name = TZPROFILER_NAME,
	.fops = &tzprofiler_fops,
	.owner = THIS_MODULE,
};

static int __init tzprofiler_init(void)
{
	int rc;

	rc = tz_cdev_register(&tzprofiler_cdev);
	if (rc)
		return rc;

	current_full_pool = TZPROFILER_LIST1_NEED_CLEAN;

	init_waitqueue_head(&sk_wait);
	init_waitqueue_head(&tzprofiler_data_writing);

	return 0;
}

static void __exit tzprofiler_exit(void)
{
	tz_cdev_unregister(&tzprofiler_cdev);
}

module_init(tzprofiler_init);
module_exit(tzprofiler_exit);
