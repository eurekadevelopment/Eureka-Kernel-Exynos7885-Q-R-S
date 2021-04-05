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

#include <linux/device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>

#if defined(CONFIG_ARCH_MSM) && defined(CONFIG_MSM_SCM)
#if defined(CONFIG_ARCH_APQ8084)
#include <soc/qcom/scm.h>
#elif defined(CONFIG_ARCH_MSM8939) || defined(CONFIG_ARCH_MSM8996)
#include <asm/cacheflush.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/qseecomi.h>
#elif defined(CONFIG_ARCH_MSM8974)
#include <mach/scm.h>
#endif
#endif

#include "tzdev.h"
#include "tzlog.h"
#include "tz_iwlog.h"
#include "tz_platform.h"

/* Define a tzdev device structure for use with dev_debug() etc */
static struct device_driver tzdev_drv = {
	.name = "tzdev"
};

static struct device tzd = {
	.driver = &tzdev_drv
};

static struct device *tzdev_dev = &tzd;

#define QSEE_CE_CLK_100MHZ	100000000

#define QSEE_CLK_ON             0x1
#define QSEE_CLK_OFF            0x0

/* svc_id and cmd_id to call QSEE 3-rd party smc handler */
#define TZ_SVC_EXECUTIVE_EXT		250
#define TZ_CMD_ID_EXEC_SMC_EXT		1

#define SCM_EBUSY			-55
#define SCM_V2_EBUSY			-12

#define SCM_TZM_FNID(s, c) (((((s) & 0xFF) << 8) | ((c) & 0xFF)) | 0x33000000)

#define TZ_EXECUTIVE_EXT_ID_PARAM_ID \
		TZ_SYSCALL_CREATE_PARAM_ID_4( \
		TZ_SYSCALL_PARAM_TYPE_BUF_RW, TZ_SYSCALL_PARAM_TYPE_VAL,\
		TZ_SYSCALL_PARAM_TYPE_BUF_RW, TZ_SYSCALL_PARAM_TYPE_VAL)

struct tzdev_msm_msg {
	uint32_t p0;
	uint32_t p1;
	uint32_t p2;
	uint32_t p3;
#if defined(CONFIG_QCOM_SCM_ARMV8)
	__s64 tv_sec;
	__s32 tv_nsec;
#endif
	uint32_t crypto_clk;
};

struct tzdev_msm_ret_msg {
	uint32_t p0;
	uint32_t p1;
	uint32_t p2;
	uint32_t p3;
#if defined(CONFIG_QCOM_SCM_ARMV8)
	uint32_t timer_remains_ms;
#endif /* CONFIG_QCOM_SCM_ARMV8 */
};

#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT)
static struct clk *tzdev_core_src;
static struct clk *tzdev_core_clk;
static struct clk *tzdev_iface_clk;
static struct clk *tzdev_bus_clk;
static DEFINE_MUTEX(tzdev_qc_clk_mutex);
#endif /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT */

#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG)
static unsigned long tzdev_qc_clk = QSEE_CLK_OFF;
#else /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG */
static unsigned long tzdev_qc_clk = QSEE_CLK_ON;
#endif /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG */

#if defined(CONFIG_QCOM_SCM_ARMV8)
#define BF_SMC_SUCCESS		0
#define BF_SMC_INTERRUPTED	1
#define BF_SMC_KERNEL_PANIC	2

DEFINE_MUTEX(tzdev_smc_lock);

struct hrtimer tzdev_get_event_timer;
extern struct completion tzdev_iwi_event_done;

static enum hrtimer_restart tzdev_get_event_timer_handler(struct hrtimer *timer)
{
	complete(&tzdev_iwi_event_done);

	return HRTIMER_NORESTART;
}

int tzdev_platform_smc_call(struct tzio_smc_data *data)
{
	struct tzdev_msm_msg msm_msg = {
		data->args[0], data->args[1], data->args[2], data->args[3],
		0, 0, tzdev_qc_clk
	};
	struct tzdev_msm_ret_msg ret_msg = {0, 0, 0, 0, 0};
	struct scm_desc desc = {0};
	struct timespec ts;
	void *scm_buf;
	int rv;

	scm_buf = kzalloc(PAGE_ALIGN(sizeof(msm_msg)), GFP_KERNEL);
	if (!scm_buf)
		return -ENOMEM;

	getnstimeofday(&ts);
	msm_msg.tv_sec = ts.tv_sec;
	msm_msg.tv_nsec = ts.tv_nsec;
	memcpy(scm_buf, &msm_msg, sizeof(msm_msg));
	dmac_flush_range(scm_buf, (unsigned char *)scm_buf + sizeof(msm_msg));

	desc.arginfo = TZ_EXECUTIVE_EXT_ID_PARAM_ID;
	desc.args[0] = virt_to_phys(scm_buf);
	desc.args[1] = sizeof(msm_msg);
	desc.args[2] = virt_to_phys(scm_buf);
	desc.args[3] = sizeof(ret_msg);

	mutex_lock(&tzdev_smc_lock);
	hrtimer_cancel(&tzdev_get_event_timer);

#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT) && !defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG)
	tzdev_qc_pm_clock_enable();
#endif
	do {
		rv = scm_call2(SCM_TZM_FNID(TZ_SVC_EXECUTIVE_EXT,
					TZ_CMD_ID_EXEC_SMC_EXT), &desc);
		if (rv) {
			kfree(scm_buf);
			printk(KERN_ERR "scm_call() failed: %d\n", rv);
			if (rv == SCM_V2_EBUSY)
				rv = -EBUSY;

#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT) && !defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG)
			tzdev_qc_pm_clock_disable();
#endif
			mutex_unlock(&tzdev_smc_lock);
			return rv;
		}
	} while (desc.ret[0] == BF_SMC_INTERRUPTED);

#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT) && !defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG)
	tzdev_qc_pm_clock_disable();
#endif

	if (desc.ret[0] == BF_SMC_KERNEL_PANIC) {
		static unsigned int panic_msg_printed;

		tz_iwlog_read_buffers();

		if (!panic_msg_printed) {
			tzdev_print(0, "Secure kernel panicked\n");
			panic_msg_printed = 1;
		}

#if defined(CONFIG_TZDEV_SWD_PANIC_IS_CRITICAL)
		panic("Secure kernel panicked\n");
#else
		mutex_unlock(&tzdev_smc_lock);
		kfree(scm_buf);
		return -EIO;
#endif
	}

	dmac_flush_range(scm_buf, (unsigned char *)scm_buf + sizeof(ret_msg));
	memcpy(&ret_msg, scm_buf, sizeof(ret_msg));

	if (ret_msg.timer_remains_ms) {
		unsigned long secs;
		unsigned long nsecs;
		secs = ret_msg.timer_remains_ms / MSEC_PER_SEC;
		nsecs = (ret_msg.timer_remains_ms % MSEC_PER_SEC) * NSEC_PER_MSEC;

		hrtimer_start(&tzdev_get_event_timer,
				ktime_set(secs, nsecs), HRTIMER_MODE_REL);
	}

	mutex_unlock(&tzdev_smc_lock);

	kfree(scm_buf);

	data->args[0] = ret_msg.p0;
	data->args[1] = ret_msg.p1;
	data->args[2] = ret_msg.p2;
	data->args[3] = ret_msg.p3;

	return 0;
}

#else /* CONFIG_QCOM_SCM_ARMV8 */
int tzdev_platform_smc_call(struct tzio_smc_data *data)
{
	struct tzdev_msm_msg msm_msg = {
		data->args[0], data->args[1], data->args[2], data->args[3],
		tzdev_qc_clk
	};
	struct tzdev_msm_ret_msg ret = {0, 0, 0, 0};
	int rv;

	rv = scm_call(TZ_SVC_EXECUTIVE_EXT, TZ_CMD_ID_EXEC_SMC_EXT,
		      &msm_msg, sizeof(msm_msg),
		      &ret, sizeof(ret));

	if (rv) {
		printk(KERN_ERR "scm_call() failed: %d\n", rv);
		if (rv == SCM_EBUSY)
			rv = -EBUSY;
		return rv;
	}

	data->args[0] = ret_msg.p0;
	data->args[1] = ret_msg.p1;
	data->args[2] = ret_msg.p2;
	data->args[3] = ret_msg.p3;

	return 0;
}
#endif /* CONFIG_QCOM_SCM_ARMV8 */


#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT)
static int tzdev_qc_pm_clock_initialize(void)
{
	int ret = 0;
	int freq_val = 0;
#if defined(CONFIG_ARCH_MSM8939)
	const char *prop_name = "qcom,freq-val";
#else /* CONFIG_ARCH_MSM8939 */
	const char *prop_name = "qcom,ce-opp-freq";
#endif /* CONFIG_ARCH_MSM8939 */

	tzdev_core_src = clk_get(tzdev_dev, "core_clk_src");
	if (IS_ERR(tzdev_core_src)) {
		tzdev_print(0, "no tzdev_core_src, ret = %d", ret);
		ret = PTR_ERR(tzdev_core_src);
		goto error;
	}
	if (of_property_read_u32(tzdev_dev->of_node, prop_name, &freq_val)) {
		freq_val = QSEE_CE_CLK_100MHZ;
		tzdev_print(0, "Unable to get frequency value from \"%s\" property. " \
			       "Set default: %d\n", prop_name, freq_val);
	}
	ret = clk_set_rate(tzdev_core_src, freq_val);
	if (ret) {
		tzdev_print(0, "clk_set_rate failed, ret = %d", ret);
		ret = -EIO;
		goto put_core_src_clk;
	}

	tzdev_core_clk = clk_get(tzdev_dev, "core_clk");
	if (IS_ERR(tzdev_core_clk)) {
		tzdev_print(0, "no tzdev_core_clk");
		ret = PTR_ERR(tzdev_core_clk);
		goto clear_core_clk;
	}
	tzdev_iface_clk = clk_get(tzdev_dev, "iface_clk");
	if (IS_ERR(tzdev_iface_clk)) {
		tzdev_print(0, "no tzdev_iface_clk");
		ret = PTR_ERR(tzdev_iface_clk);
		goto put_core_clk;
	}
	tzdev_bus_clk = clk_get(tzdev_dev, "bus_clk");
	if (IS_ERR(tzdev_bus_clk)) {
		tzdev_print(0, "no tzdev_bus_clk");
		ret = PTR_ERR(tzdev_bus_clk);
		goto put_iface_clk;
	}

	tzdev_print(0, "Got QC HW crypto clks\n");
	return ret;

put_iface_clk:
	clk_put(tzdev_iface_clk);
	tzdev_bus_clk = NULL;

put_core_clk:
	clk_put(tzdev_core_clk);
	tzdev_iface_clk = NULL;

clear_core_clk:
	tzdev_core_clk = NULL;

put_core_src_clk:
	clk_put(tzdev_core_src);

error:
	tzdev_core_src = NULL;

	return ret;
}

static void tzdev_qc_pm_clock_finalize(void)
{
	clk_put(tzdev_bus_clk);
	clk_put(tzdev_iface_clk);
	clk_put(tzdev_core_clk);
	clk_put(tzdev_core_src);
}
#endif /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT */

#if defined(CONFIG_TZDEV_MSM_CRYPTO_WORKAROUND)
#include <linux/msm_ion.h>
#include "../qseecom_kernel.h"

static void msm_crypto_workaround(void)
{
	enum qseecom_client_handle_type {
		QSEECOM_CLIENT_APP = 1,
		QSEECOM_LISTENER_SERVICE,
		QSEECOM_SECURE_SERVICE,
		QSEECOM_GENERIC,
		QSEECOM_UNAVAILABLE_CLIENT_APP,
	};
	struct qseecom_client_handle {
		u32 app_id;
		u8 *sb_virt;
		s32 sb_phys;
		uint32_t user_virt_sb_base;
		size_t sb_length;
		struct ion_handle *ihandle;		/* Retrieve phy addr */
	};
	struct qseecom_listener_handle {
		u32 id;
	};
	struct qseecom_dev_handle {
		enum qseecom_client_handle_type type;
		union {
			struct qseecom_client_handle client;
			struct qseecom_listener_handle listener;
		};
		bool released;
		int  abort;
		wait_queue_head_t abort_wq;
		atomic_t ioctl_count;
		bool perf_enabled;
		bool fast_load_enabled;
	};

	/* Bogus handles */
	struct qseecom_dev_handle dev_handle;
	struct qseecom_handle handle = {
		.dev = &dev_handle
	};
	int ret;

	ret = qseecom_set_bandwidth(&handle, 1);
	if (ret)
		tzdev_print(0, "qseecom_set_bandwidth failed ret = %d\n", ret);
}
#endif /* CONFIG_TZDEV_MSM_CRYPTO_WORKAROUND */

static int tzdev_qc_probe(struct platform_device *pdev)
{
	tzdev_dev->of_node = pdev->dev.of_node;

#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT)
	tzdev_qc_pm_clock_initialize();
#endif /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT */

#if defined(CONFIG_TZDEV_MSM_CRYPTO_WORKAROUND)
	/* Force crypto engine clocks on */
	msm_crypto_workaround();
#endif /* CONFIG_TZDEV_MSM_CRYPTO_WORKAROUND */

#if defined(CONFIG_QCOM_SCM_ARMV8)
	hrtimer_init(&tzdev_get_event_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tzdev_get_event_timer.function = tzdev_get_event_timer_handler;
#endif /* CONFIG_QCOM_SCM_ARMV8 */

	return 0;
}

static int tzdev_qc_remove(struct platform_device *pdev)
{
#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT)
	tzdev_qc_pm_clock_finalize();
#endif /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT */

#if defined(CONFIG_QCOM_SCM_ARMV8)
	hrtimer_cancel(&tzdev_get_event_timer);
#endif /* CONFIG_QCOM_SCM_ARMV8 */
	return 0;
}

static int tzdev_qc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int tzdev_qc_resume(struct platform_device *pdev)
{
	return 0;
}

struct of_device_id tzdev_qc_match[] = {
	{
		.compatible = "qcom,tzd",
	},
	{}
};

struct platform_driver tzdev_qc_plat_driver = {
	.probe = tzdev_qc_probe,
	.remove = tzdev_qc_remove,
	.suspend = tzdev_qc_suspend,
	.resume = tzdev_qc_resume,

	.driver = {
		.name = "tzdev",
		.owner = THIS_MODULE,
		.of_match_table = tzdev_qc_match,
	},
};

#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG) || defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT)
static void tzdev_qc_pm_clock_enable(void)
{
	mutex_lock(&tzdev_qc_clk_mutex);
	if (tzdev_qc_clk != QSEE_CLK_OFF)
		goto out;

	if (clk_prepare_enable(tzdev_core_clk)) {
		tzdev_print(0, "no core clk\n");
		goto out;
	} else if (clk_prepare_enable(tzdev_iface_clk)) {
		tzdev_print(0, "no iface clk\n");
		goto unprepare_core_clk;
	} else if (clk_prepare_enable(tzdev_bus_clk)) {
		tzdev_print(0, "no bus clk\n");
		goto unprepare_iface_clk;
	}
	tzdev_qc_clk = QSEE_CLK_ON;
	goto out;

unprepare_iface_clk:
	clk_disable_unprepare(tzdev_iface_clk);
unprepare_core_clk:
	clk_disable_unprepare(tzdev_core_clk);
out:
	mutex_unlock(&tzdev_qc_clk_mutex);
}

static void tzdev_qc_pm_clock_disable(void)
{
	mutex_lock(&tzdev_qc_clk_mutex);

	if (tzdev_qc_clk != QSEE_CLK_ON)
		goto out;

	clk_disable_unprepare(tzdev_iface_clk);
	clk_disable_unprepare(tzdev_core_clk);
	clk_disable_unprepare(tzdev_bus_clk);

	tzdev_qc_clk = QSEE_CLK_OFF;
out:
	mutex_unlock(&tzdev_qc_clk_mutex);
}
#endif /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG  || CONFIG_TZDEV_QC_CRYPTO_CLOCKS_MANAGEMENT */

int tzdev_platform_register(void)
{
	return platform_driver_register(&tzdev_qc_plat_driver);
}

void tzdev_platform_unregister(void)
{
	platform_driver_unregister(&tzdev_qc_plat_driver);
}

int tzdev_platform_open(void)
{
	return 0;
}

int tzdev_platform_close(void)
{
#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG)
	if (tzdev_qc_clk == QSEE_CLK_ON) {
		tzdev_qc_pm_clock_disable();
		tzdev_qc_clk = QSEE_CLK_OFF;
	}
#endif
	return 0;
}

int tzdev_platform_ioctl(unsigned int cmd, unsigned long arg)
{
#if defined(CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG)
	switch (cmd) {
	case TZIO_SET_QC_CLK: {
		if (arg == QSEE_CLK_ON)
			tzdev_qc_pm_clock_enable();
		else if (arg == QSEE_CLK_OFF)
			tzdev_qc_pm_clock_disable();
		return 0;
	}
	default:
		return -ENOTTY;
	}
#else /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG */
	(void)cmd;
	(void)arg;

	return -ENOTTY;
#endif /* CONFIG_TZDEV_QC_CRYPTO_CLOCKS_USR_MNG */
}
