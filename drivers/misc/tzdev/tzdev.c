/*
 * Copyright (C) 2012-2019 Samsung Electronics, Inc.
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
#include <linux/buffer_head.h>
#include <linux/completion.h>
#include <linux/cpu.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mmzone.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pid.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <asm/segment.h>
#include <asm/uaccess.h>

#include "sysdep.h"
#include "tzdev.h"
#include "tz_boost.h"
#include "tz_cdev.h"
#include "tz_cma.h"
#include "tz_iw_boot_log.h"
#include "tz_iwio.h"
#include "tz_iwlog.h"
#include "tz_kernel_api_internal.h"
#include "tzlog.h"
#include "tz_mem.h"
#include "tz_panic_dump.h"
#include "tz_platform.h"
#include "tzprofiler.h"
#include "tz_wormhole.h"

MODULE_AUTHOR("Jaemin Ryu <jm77.ryu@samsung.com>");
MODULE_AUTHOR("Vasily Leonenko <v.leonenko@samsung.com>");
MODULE_AUTHOR("Alex Matveev <alex.matveev@samsung.com>");
MODULE_DESCRIPTION("TZDEV driver");
MODULE_LICENSE("GPL");

int tzdev_verbosity;

module_param(tzdev_verbosity, int, 0644);
MODULE_PARM_DESC(tzdev_verbosity, "0: normal, 1: verbose, 2: debug");

enum tzdev_swd_state {
	TZDEV_SWD_DOWN,
	TZDEV_SWD_UP,
	TZDEV_SWD_DEAD
};

static int tzdev_fd_open;
static DEFINE_MUTEX(tzdev_fd_mutex);

static atomic_t tzdev_swd_state = ATOMIC_INIT(TZDEV_SWD_DOWN);

DECLARE_COMPLETION(tzdev_iwi_event_done);

static struct tzio_sysconf tz_sysconf;

static DEFINE_SPINLOCK(tzdev_rsp_lock);
static unsigned int tzdev_rsp_event_mask;

#define TZDEV_RSP_UFIFO_DEPTH		1024
#define TZDEV_RSP_KFIFO_DEPTH		16

static DEFINE_KFIFO(tzdev_rsp_ufifo, unsigned int, TZDEV_RSP_UFIFO_DEPTH);
static DEFINE_KFIFO(tzdev_rsp_kfifo, unsigned int, TZDEV_RSP_KFIFO_DEPTH);

static struct tzio_service_channel *tzdev_service_channel;
/* We need this due to currently Tzdaemon use SWd cpu mask to
 * wake up worker threads. Will be completely removed after
 * moving Tzdaemon worker threads to TZDEV */
static unsigned long tzdaemon_cpu_mask;

int __tzdev_smc_cmd(struct tzio_smc_data *data,
		unsigned int swd_ctx_present)
{
	int ret;

#ifdef TZIO_DEBUG
	printk(KERN_ERR "tzdev_smc_cmd: args[0]=0x%lx, args[1]=0x%lx, args[2]=0x%lx, args[3]=0x%lx\n",
			data->args[0], data->args[1], data->args[2], data->args[3]);
#endif
	tzprofiler_enter_sw();
	tz_iwlog_schedule_delayed_work();

	ret = tzdev_platform_smc_call(data);

	tz_iwlog_cancel_delayed_work();

	if (swd_ctx_present) {
		tzdev_notify_swd_cpu_mask_update();
		tz_iwnotify_call_chains(data->iwnotify_oem_flags);
		data->iwnotify_oem_flags = 0;
	}

	tzprofiler_wait_for_bufs();
	tzprofiler_exit_sw();
	tz_iwlog_read_buffers();

	return ret;
}

/*  TZDEV interface functions */
unsigned int tzdev_is_up(void)
{
	return atomic_read(&tzdev_swd_state) == TZDEV_SWD_UP;
}

static int tzdev_alloc_aux_channels(void)
{
	int ret;
	unsigned int i;

	for (i = 0; i < NR_SW_CPU_IDS; ++i) {
		ret = tz_iwio_alloc_aux_channel(i);
		if (ret)
			return ret;
	}

	return 0;
}

static int tzdev_alloc_service_channel(void)
{
	tzdev_service_channel = tz_iwio_alloc_iw_channel(TZ_IWIO_CONNECT_SERVICE, 1);
	if (!tzdev_service_channel)
		return -ENOMEM;

	return 0;
}

unsigned long tzdev_get_swd_cpu_mask(void)
{
	if (!tzdev_service_channel)
		return 0;

	return tzdev_service_channel->mask;
}

static void tzdev_get_nwd_sysconf(struct tzio_sysconf *s)
{
	s->flags = 0;
#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG)
	s->flags |= SYSCONF_CRYPTO_CLOCK_MANAGEMENT;
#endif
}

static int tzdev_get_sysconf(struct tzio_sysconf *s)
{
	struct tzio_sysconf nwd_sysconf;
	struct tz_iwio_aux_channel *ch;
	int ret = 0;

	/* Get sysconf from SWd */
	ch = tz_iwio_get_aux_channel();
	ret = tzdev_smc_get_swd_sysconf();
	tz_iwio_put_aux_channel();

	if (ret) {
		tzdev_print(0, "tzdev_smc_get_swd_sysconf() failed with %d\n", ret);
		return ret;
	}
	memcpy(s, ch->buffer, sizeof(struct tzio_sysconf));

	tzdev_get_nwd_sysconf(&nwd_sysconf);

	/* Merge NWd and SWd sysconf structures */
	s->flags |= nwd_sysconf.flags;

	tz_boost_set_boost_mask(s->big_cpus_mask);

	return ret;
}

static irqreturn_t tzdev_event_handler(int irq, void *ptr)
{
	complete(&tzdev_iwi_event_done);

	return IRQ_HANDLED;
}

#if CONFIG_TZDEV_IWI_PANIC != 0
static void dump_kernel_panic_bh(struct work_struct *work)
{
	atomic_set(&tzdev_swd_state, TZDEV_SWD_DEAD);
	tz_iw_boot_log_read();
	tz_iwlog_read_buffers();
}

static DECLARE_WORK(dump_kernel_panic, dump_kernel_panic_bh);

static irqreturn_t tzdev_panic_handler(int irq, void *ptr)
{
	schedule_work(&dump_kernel_panic);
	return IRQ_HANDLED;
}
#endif

void tzdev_resolve_iwis_id(unsigned int *iwi_event, unsigned int *iwi_panic)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "samsung,blowfish");
	if (!node) {
		*iwi_event = CONFIG_TZDEV_IWI_EVENT;
#if CONFIG_TZDEV_IWI_PANIC != 0
		*iwi_panic = CONFIG_TZDEV_IWI_PANIC;
#endif
		tzdev_print(0, "of_find_compatible_node() failed\n");
		return;
	}

	*iwi_event = irq_of_parse_and_map(node, 0);
	if (!*iwi_event) {
		*iwi_event = CONFIG_TZDEV_IWI_EVENT;
		tzdev_print(0, "Use IWI event IRQ number from config %d\n",
			CONFIG_TZDEV_IWI_EVENT);
	}

#if CONFIG_TZDEV_IWI_PANIC != 0
	*iwi_panic = irq_of_parse_and_map(node, 1);
	if (!*iwi_panic) {
		*iwi_panic = CONFIG_TZDEV_IWI_PANIC;
		tzdev_print(0, "Use IWI panic IRQ number from config %d\n",
			CONFIG_TZDEV_IWI_PANIC);
	}
#endif
	of_node_put(node);
}

static void tzdev_register_iwis(void)
{
	int ret;
	int iwi_event;
	int iwi_panic;

	tzdev_resolve_iwis_id(&iwi_event, &iwi_panic);

	ret = request_irq(iwi_event, tzdev_event_handler, 0, "tzdev_iwi_event", NULL);
	if (ret)
		tzdev_print(0, "TZDEV_IWI_EVENT registration failed: %d\n", ret);

#if CONFIG_TZDEV_IWI_PANIC != 0
	ret = request_irq(iwi_panic, tzdev_panic_handler, 0, "tzdev_iwi_panic", NULL);
	if (ret)
		tzdev_print(0, "TZDEV_IWI_PANIC registration failed: %d\n", ret);
#endif
}

static int tzdev_run_init_sequence(void)
{
	int ret = 0;

	if (atomic_read(&tzdev_swd_state) == TZDEV_SWD_DOWN) {
		/* check kernel and driver version compatibility with TEEGRIS */
		ret = tzdev_smc_check_version();
		if (ret == -ENOSYS || ret == -EINVAL) {
			/* version is not compatibile. Not critical, continue ... */
			ret = 0;
		} else if (ret) {
			tzdev_print(0, "tzdev_smc_check_version() failed\n");
			goto out;
		}

		if (tzdev_cma_mem_register()) {
			tzdev_print(0, "tzdev_cma_mem_register() failed\n");
			ret = -ESHUTDOWN;
			goto out;
		}

		if (tzdev_alloc_aux_channels()) {
			tzdev_print(0, "tzdev_alloc_AUX_Channels() failed\n");
			ret = -ESHUTDOWN;
			goto out;
		}

		if (tzdev_alloc_service_channel()) {
			tzdev_print(0, "tzdev_alloc_service_channel() failed\n");
			ret = -ESHUTDOWN;
			goto out;
		}

		if (tzdev_mem_init()) {
			tzdev_print(0, "tzdev_mem_init() failed\n");
			ret = -ESHUTDOWN;
			goto out;
		}

		if (tz_iwlog_initialize()) {
			tzdev_print(0, "tz_iwlog_initialize() failed\n");
			ret = -ESHUTDOWN;
			goto out;
		}

		if (tzprofiler_initialize()) {
			tzdev_print(0, "tzprofiler_initialize() failed\n");
			ret = -ESHUTDOWN;
			goto out;
		}

		if (tz_panic_dump_alloc_buffer()) {
			tzdev_print(0, "tz_panic_dump_alloc_buffer() failed\n");
			ret = -ESHUTDOWN;
			goto out;
		}

		if (tzdev_get_sysconf(&tz_sysconf)) {
			tzdev_print(0, "tzdev_get_sysconf() failed\n");
			ret = -ESHUTDOWN;
			goto out;
		}

		tzdev_register_iwis();
		tz_iwnotify_initialize();
		tzdev_kapi_init();

		if (atomic_cmpxchg(&tzdev_swd_state, TZDEV_SWD_DOWN, TZDEV_SWD_UP)) {
			ret = -ESHUTDOWN;
			goto out;
		}
	}
out:
	if (ret == -ESHUTDOWN) {
		atomic_set(&tzdev_swd_state, TZDEV_SWD_DEAD);
		tz_iw_boot_log_read();
		tz_iwlog_read_buffers();
	}
	return ret;
}

static int tzdev_get_access_info(struct tzio_access_info *s)
{
	struct task_struct *task;
	struct mm_struct *mm;
	struct file *exe_file;

	rcu_read_lock();

	task = find_task_by_vpid(s->pid);
	if (!task) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(task);
	rcu_read_unlock();

	s->gid = task->tgid;

	mm = get_task_mm(task);
	put_task_struct(task);
	if (!mm)
		return -ESRCH;

	exe_file = get_mm_exe_file(mm);
	mmput(mm);
	if (!exe_file)
		return -ESRCH;

	strncpy(s->ca_name, exe_file->f_path.dentry->d_name.name, CA_ID_LEN);
	if (s->ca_name[CA_ID_LEN - 1])
		return -ENAMETOOLONG;

	fput(exe_file);

	return 0;
}

static struct tzio_smc_data tzdev_process_reply(struct tzio_smc_data d, unsigned int is_user)
{
	unsigned int pipe = 0;
	unsigned int is_pipe_user = 1;
	unsigned int notify_user = 0;
	unsigned int num_written, num_read;

	if (!tzdev_is_mem_exist(d.pipe, &is_pipe_user))
		tzdev_print(2, "Pipe 0x%x not found in TZDEV shmem idr\n", d.args[0]);

	pipe = d.args[0] & TZDEV_PIPE_TARGET_DEAD_MASK;
	d.args[0] &= ~TZDEV_TARGET_DEAD_MASK;

	spin_lock(&tzdev_rsp_lock);
	tzdev_rsp_event_mask |= d.event_mask;
	/* Here we need to notify user space about pending
	 * event or reply in case current request was sent by kernel */
	if (!is_user) {
		/* Has pending flags */
		if (tzdev_rsp_event_mask)
			notify_user = 1;
		/* CPU mask is outdated */
		if (tzdaemon_cpu_mask != tzdev_get_swd_cpu_mask())
			notify_user = 1;
		/* Got user space pipe but user space reply fifo is empty,
		 * in this case user space can sleep and we need to notify
		 * it */
		if (is_pipe_user && kfifo_is_empty(&tzdev_rsp_ufifo))
			notify_user = 1;
	}
	/* Got zero pipe, no meaningful reply from
	 * secure kernel, try to read reply from fifo */
	if (!pipe)
		goto read_reply;

	num_written = is_pipe_user ?
		sysdep_kfifo_put(&tzdev_rsp_ufifo, pipe) :
		sysdep_kfifo_put(&tzdev_rsp_kfifo, pipe);
	if (!num_written) {
		tzdev_print(0, "Putting response to %s queue failed\n",
				is_pipe_user ? "user" : "kernel");
		/* User FIFO overflow is handled by dropping the message. */
		BUG_ON(!is_pipe_user);
	}

read_reply:
	memset(&d, 0, sizeof(d));

	if (is_user) {
		num_read = kfifo_get(&tzdev_rsp_ufifo, &pipe);
		/* Check if fifo with user space replies
		 * contains more data, if yes we need to notify
		 * user space about it */
		if (num_read && !kfifo_is_empty(&tzdev_rsp_ufifo))
			notify_user = 1;

		d.args[0] = num_read ? pipe : 0;
		/* Update user space notification flags */
		d.event_mask |= tzdev_rsp_event_mask;
		tzdev_rsp_event_mask = 0;
	} else {
		num_read = kfifo_get(&tzdev_rsp_kfifo, &pipe);
		d.args[0] = num_read ? pipe : 0;
	}
	spin_unlock(&tzdev_rsp_lock);

	if (notify_user)
		complete(&tzdev_iwi_event_done);

	return d;
}

static struct tzio_smc_data tzdev_send_command_user(unsigned int tid, unsigned int shm_id)
{
	struct tzio_smc_data d;

	d = tzdev_smc_command(tid, shm_id);

	return tzdev_process_reply(d, 1);
}

struct tzio_smc_data tzdev_send_command(unsigned int tid, unsigned int shm_id)
{
	struct tzio_smc_data d;

	d = tzdev_smc_command(tid, shm_id);

	return tzdev_process_reply(d, 0);
}

static struct tzio_smc_data tzdev_get_event_user(void)
{
	struct tzio_smc_data d;

	d = tzdev_smc_get_event();

	return tzdev_process_reply(d, 1);
}

struct tzio_smc_data tzdev_get_event(void)
{
	struct tzio_smc_data d;

	d = tzdev_smc_get_event();

	return tzdev_process_reply(d, 0);
}

static struct tzio_smc_data tzdev_update_ree_time(void)
{
	struct timespec ts;
	struct tzio_smc_data d;

	getnstimeofday(&ts);
	d = tzdev_smc_update_ree_time(ts.tv_sec, ts.tv_nsec);

	return tzdev_process_reply(d, 1);
}

static int tzdev_restart_swd_userspace(void)
{
#define TZDEV_SWD_USERSPACE_RESTART_TIMEOUT 10

	int ret;
	struct timespec ts, te;

	/* FIXME Ugly, but works.
	 * Will be corrected in the course of SMC handler work */
	getrawmonotonic(&ts);
	do {
		ret = tzdev_smc_swd_restart_userspace();
		getrawmonotonic(&te);
		if (te.tv_sec - ts.tv_sec > TZDEV_SWD_USERSPACE_RESTART_TIMEOUT) {
			atomic_set(&tzdev_swd_state, TZDEV_SWD_DEAD);
			tzdev_print(0, "Timeout on starting SWd userspace.\n");
			return -ETIME;
		}
	} while (ret != 1);

	/* We need to release all iwshmems here to avoid
	 * situation when we have alive SWd applications that
	 * are able to write to iwshmems and released iwshmems on
	 * the Linux kernel side, otherwise
	 * we can have random memory corruptions */
	tzdev_mem_release_all_user();

	return 0;
}

static int tzdev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	if (atomic_read(&tzdev_swd_state) == TZDEV_SWD_DEAD)
		return -ESHUTDOWN;

	mutex_lock(&tzdev_fd_mutex);

	if (tzdev_fd_open != 0) {
		ret = -EBUSY;
		goto out;
	}

	tzdev_platform_open();

#if !defined(CONFIG_TZDEV_EARLY_SWD_INIT)
	ret = tzdev_run_init_sequence();
	if (ret)
		goto out;

	ret = tzdev_restart_swd_userspace();
	if (ret)
		goto out;
#endif /* CONFIG_TZDEV_EARLY_SWD_INIT */

	tzdev_fd_open++;

out:
	mutex_unlock(&tzdev_fd_mutex);
	return ret;
}

static int tzdev_release(struct inode *inode, struct file *filp)
{
#if defined(CONFIG_TZDEV_NWD_PANIC_ON_CLOSE)
	panic("tzdev invalid close\n");
#endif /* CONFIG_TZDEV_NWD_PANIC_ON_CLOSE */

	mutex_lock(&tzdev_fd_mutex);
	tz_boost_disable();

	tzdev_fd_open--;
	BUG_ON(tzdev_fd_open);

	mutex_unlock(&tzdev_fd_mutex);

	tzdev_platform_close();

#if defined(CONFIG_TZDEV_EARLY_SWD_INIT)
	tzdev_restart_swd_userspace();
#endif /* CONFIG_TZDEV_EARLY_SWD_INIT */

	tz_wormhole_close_connection();

	return 0;
}

int tzdev_is_opened(void)
{
	return tzdev_fd_open;
}

static long tzdev_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	if (atomic_read(&tzdev_swd_state) == TZDEV_SWD_DEAD)
		return -ESHUTDOWN;

	switch (cmd) {
	case TZIO_SMC: {
		struct tzio_smc_data __user *argp = (struct tzio_smc_data __user *)arg;
		struct tzio_smc_data s;

		if (copy_from_user(&s, argp, sizeof(struct tzio_smc_data)))
			return -EFAULT;

		s = tzdev_send_command_user(s.args[0], s.args[1]);

		if (copy_to_user(argp, &s, sizeof(struct tzio_smc_data)))
			return -EFAULT;

		return 0;
	}
	case TZIO_GET_ACCESS_INFO: {
		int ret = 0;
		struct tzio_access_info __user *argp = (struct tzio_access_info __user *)arg;
		struct tzio_access_info s;

		if (copy_from_user(&s, argp, sizeof(struct tzio_access_info)))
			return -EFAULT;

		ret = tzdev_get_access_info(&s);
		if (ret)
			return ret;
		if (copy_to_user(argp, &s, sizeof(struct tzio_access_info)))
			return -EFAULT;

		return 0;
	}
	case TZIO_GET_SYSCONF: {
		struct tzio_sysconf __user *argp = (struct tzio_sysconf __user *)arg;

		if (copy_to_user(argp, &tz_sysconf, sizeof(struct tzio_sysconf)))
			return -EFAULT;

		return 0;
	}
	case TZIO_MEM_REGISTER: {
		int ret = 0;
		struct tzio_mem_register __user *argp = (struct tzio_mem_register __user *)arg;
		struct tzio_mem_register s;

		if (copy_from_user(&s, argp, sizeof(struct tzio_mem_register)))
			return -EFAULT;

		ret = tzdev_mem_register_user(&s);
		if (ret)
			return ret;

		if (copy_to_user(argp, &s, sizeof(struct tzio_mem_register)))
			return -EFAULT;

		return 0;
	}
	case TZIO_MEM_RELEASE: {
		return tzdev_mem_release_user(arg);
	}
	case TZIO_WAIT_EVT: {
		return wait_for_completion_interruptible(&tzdev_iwi_event_done);
	}
	case TZIO_GET_PIPE: {
		struct tzio_smc_data __user *argp = (struct tzio_smc_data __user *)arg;
		struct tzio_smc_data s;

		s = tzdev_get_event_user();

		if (copy_to_user(argp, &s, sizeof(struct tzio_smc_data)))
			return -EFAULT;

		return 0;
	}
	case TZIO_UPDATE_REE_TIME: {
		struct tzio_smc_data __user *argp = (struct tzio_smc_data __user *)arg;
		struct tzio_smc_data s;

		if (copy_from_user(&s, argp, sizeof(struct tzio_smc_data)))
			return -EFAULT;

		s = tzdev_update_ree_time();

		if (copy_to_user(argp, &s, sizeof(struct tzio_smc_data)))
			return -EFAULT;

		return 0;
	}
	case TZIO_BOOST: {
		tz_boost_enable();
		return 0;
	}
	case TZIO_RELAX: {
		tz_boost_disable();
		return 0;
	}
	case TZIO_GET_CPU_MASK: {
		tzdaemon_cpu_mask = tzdev_get_swd_cpu_mask();
		return tzdaemon_cpu_mask;
	}
	case TZIO_ACCEPT_NEW_HOLE:
		return tz_wormhole_tzdev_accept();
	default:
		return tzdev_platform_ioctl(cmd, arg);
	}
}

static void tzdev_shutdown(void)
{
	if (atomic_read(&tzdev_swd_state) != TZDEV_SWD_DOWN)
		tzdev_smc_shutdown();
}

static const struct file_operations tzdev_fops = {
	.owner = THIS_MODULE,
	.open = tzdev_open,
	.release = tzdev_release,
	.unlocked_ioctl = tzdev_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = tzdev_unlocked_ioctl,
#endif /* CONFIG_COMPAT */
	.poll = tz_wormhole_tzdev_poll,
};

static struct tz_cdev tzdev_cdev = {
	.name = "tzdev",
	.fops = &tzdev_fops,
	.owner = THIS_MODULE,
};

static struct syscore_ops tzdev_syscore_ops = {
	.shutdown = tzdev_shutdown
};

static int __init init_tzdev(void)
{
	int rc;

	rc = tz_cdev_register(&tzdev_cdev);
	if (rc)
		return rc;

	rc = tzdev_platform_register();
	if (rc) {
		tzdev_print(0, "tzdev_platform_register() failed with error=%d\n", rc);
		goto platform_driver_registration_failed;
	}

	rc = tzdev_init_hotplug();
	if (rc) {
		tzdev_print(0, "tzdev_init_hotplug() failed with error=%d\n", rc);
		goto hotplug_initialization_failed;
	}

	tzdev_cma_mem_init(tzdev_cdev.device);

#if defined(CONFIG_TZDEV_EARLY_SWD_INIT)
	rc = tzdev_run_init_sequence();
	if (rc) {
		tzdev_print(0, "tzdev_run_init_sequence() failed with error=%d\n", rc);
		goto tzdev_initialization_failed;
	}

	rc = tzdev_restart_swd_userspace();
	if (rc) {
		tzdev_print(0, "tzdev_restart_swd_userspace() failed with error=%d\n", rc);
		goto tzdev_swd_restart_failed;
	}
#endif /* CONFIG_TZDEV_EARLY_SWD_INIT */

	register_syscore_ops(&tzdev_syscore_ops);

	return rc;

#if defined(CONFIG_TZDEV_EARLY_SWD_INIT)
tzdev_swd_restart_failed:
tzdev_initialization_failed:
	tzdev_cma_mem_release(tzdev_cdev.device);
#endif /* CONFIG_TZDEV_EARLY_SWD_INIT */
hotplug_initialization_failed:
	tzdev_platform_unregister();
platform_driver_registration_failed:
	tz_cdev_unregister(&tzdev_cdev);

	return rc;
}

static void __exit exit_tzdev(void)
{
	tzdev_platform_unregister();

	tz_cdev_unregister(&tzdev_cdev);

	tzdev_mem_fini();

	tzdev_cma_mem_release(tzdev_cdev.device);

	unregister_syscore_ops(&tzdev_syscore_ops);

	tzdev_shutdown();

	tzdev_exit_hotplug();
}

module_init(init_tzdev);
module_exit(exit_tzdev);
