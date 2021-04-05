/*
 *  Copyright (C) 2018, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#endif

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include <linux/clk.h>
#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "gf_common.h"
#include "fingerprint.h"

#define GF_DEV_NAME "goodix_fp"
#define GF_DEV_MAJOR 0	/* assigned */
#define GF_CLASS_NAME "goodix_fp"
#define GF_NETLINK_ROUTE 25
#define MAX_NL_MSG_LEN 16
#define WAKELOCK_HOLD_TIME 500 /* in ms */

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned int bufsiz = (50 * 1024);
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "maximum data bytes for SPI message");

#ifdef CONFIG_OF
static const struct of_device_id gfspi_of_match[] = {
	{ .compatible = "goodix,fingerprint", },
	{},
};
MODULE_DEVICE_TABLE(of, gfspi_of_match);
#endif

extern int fingerprint_register(struct device *dev, void *drvdata,
	struct device_attribute *attributes[], char *name);
extern void fingerprint_unregister(struct device *dev,
	struct device_attribute *attributes[]);

static struct gf_device *g_data;

#ifdef ENABLE_SENSORS_FPRINT_SECURE
#if !defined(CONFIG_SENSORS_ET5XX) && !defined(CONFIG_SENSORS_VFS8XXX)
int fpsensor_goto_suspend = 0;
#endif
#endif

#if defined(ENABLE_SENSORS_FPRINT_SECURE)
int fps_resume_set(void){
	int ret =0;

	if (fpsensor_goto_suspend) {
		fpsensor_goto_suspend = 0;
#if defined(CONFIG_TZDEV)
		if (!g_data->ldo_onoff) {
			ret = exynos_smc(FP_CSMC_HANDLER_ID, FP_HANDLER_MAIN, FP_SET_POWEROFF, 0);
			pr_info("gfspi %s: FP_SET_POWEROFF ret = %d\n", __func__, ret);
		} else {
			ret = exynos_smc(FP_CSMC_HANDLER_ID, FP_HANDLER_MAIN, FP_SET_POWERON_INACTIVE, 0);
			pr_info("gfspi %s: FP_SET_POWERON_INACTIVE ret = %d\n", __func__, ret);
		}
#else
		ret = exynos_smc(MC_FC_FP_PM_RESUME, 0, 0, 0);
		pr_info("gfspi %s : smc ret = %d\n", __func__, ret);
#endif
	}
	return ret;
}
#endif

static ssize_t gfspi_bfs_values_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct gf_device *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "\"FP_SPICLK\":\"%d\"\n",
			data->spi->max_speed_hz);
}

static ssize_t gfspi_type_check_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct gf_device *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", data->sensortype);
}

static ssize_t gfspi_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", "GOODIX");
}

static ssize_t gfspi_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gf_device *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", data->chipid);
}

static ssize_t gfspi_adm_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", DETECT_ADM);
}

static ssize_t gfspi_intcnt_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct gf_device *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", data->interrupt_count);
}

static ssize_t gfspi_intcnt_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct gf_device *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "c")) {
		data->interrupt_count = 0;
		pr_info("initialization is done\n");
	}
	return size;
}

static ssize_t gfspi_resetcnt_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct gf_device *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", data->reset_count);
}

static ssize_t gfspi_resetcnt_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct gf_device *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "c")) {
		data->reset_count = 0;
		pr_info("initialization is done\n");
	}
	return size;
}

static DEVICE_ATTR(bfs_values, 0444, gfspi_bfs_values_show, NULL);
static DEVICE_ATTR(type_check, 0444, gfspi_type_check_show, NULL);
static DEVICE_ATTR(vendor, 0444,	gfspi_vendor_show, NULL);
static DEVICE_ATTR(name, 0444, gfspi_name_show, NULL);
static DEVICE_ATTR(adm, 0444, gfspi_adm_show, NULL);
static DEVICE_ATTR(intcnt, 0664, gfspi_intcnt_show, gfspi_intcnt_store);
static DEVICE_ATTR(resetcnt, 0664, gfspi_resetcnt_show, gfspi_resetcnt_store);

static struct device_attribute *fp_attrs[] = {
	&dev_attr_bfs_values,
	&dev_attr_type_check,
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_adm,
	&dev_attr_intcnt,
	&dev_attr_resetcnt,
	NULL,
};

static void gfspi_enable_irq(struct gf_device *gf_dev)
{
	if (gf_dev->irq_enabled == 1) {
		pr_err("%s, irq already enabled\n", __func__);
	} else {
		enable_irq(gf_dev->irq);
		enable_irq_wake(gf_dev->irq);
		gf_dev->irq_enabled = 1;
		pr_debug("%s enable interrupt!\n", __func__);
	}
}

static void gfspi_disable_irq(struct gf_device *gf_dev)
{
	if (gf_dev->irq_enabled == 0) {
		pr_err("%s, irq already disabled\n", __func__);
	} else {
		disable_irq_wake(gf_dev->irq);
		disable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 0;
		pr_debug("%s disable interrupt!\n", __func__);
	}
}

static void gfspi_netlink_send(struct gf_device *gf_dev, const int command)
{
	struct nlmsghdr *nlh = NULL;
	struct sk_buff *skb = NULL;
	int ret;

	if (gf_dev->nl_sk == NULL) {
		pr_err("%s : invalid socket\n", __func__);
		return;
	}

	if (gf_dev->pid == 0) {
		pr_err("%s : invalid native process pid\n",	__func__);
		return;
	}

	/* alloc data buffer for sending to native */
	/* malloc data space at least 1500 bytes, which is ethernet data length */
	skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
	if (skb == NULL)
		return;

	nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
	if (!nlh) {
		pr_err("%s : nlmsg_put failed\n", __func__);
		kfree_skb(skb);
		return;
	}

	NETLINK_CB(skb).portid = 0;
	NETLINK_CB(skb).dst_group = 0;

	*(char *)NLMSG_DATA(nlh) = command;
	ret = netlink_unicast(gf_dev->nl_sk, skb, gf_dev->pid, MSG_DONTWAIT);
	if (ret == 0) {
		pr_err("%s : send failed\n", __func__);
		return;
	}

}

static void gfspi_netlink_recv(struct sk_buff *__skb)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh = NULL;
	char str[128];

	skb = skb_get(__skb);
	if (skb == NULL) {
		pr_err("%s : skb_get return NULL\n", __func__);
		return;
	}

	/* presume there is 5byte payload at leaset */
	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		memcpy(str, NLMSG_DATA(nlh), sizeof(str));
		g_data->pid = nlh->nlmsg_pid;
		pr_info("%s : pid: %d, msg: %s\n",
				__func__, g_data->pid, str);
	} else {
		pr_err("%s : not enough data length\n", __func__);
	}

	kfree_skb(skb);
}

static int gfspi_netlink_init(struct gf_device *gf_dev)
{
	struct netlink_kernel_cfg cfg;

	memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
	cfg.input = gfspi_netlink_recv;

	gf_dev->nl_sk =
		netlink_kernel_create(&init_net, GF_NETLINK_ROUTE, &cfg);
	if (gf_dev->nl_sk == NULL) {
		pr_err("%s : netlink create failed\n", __func__);
		return -1;
	}

	pr_info("%s : netlink create success\n", __func__);
	return 0;
}

static int gfspi_netlink_destroy(struct gf_device *gf_dev)
{
	if (gf_dev->nl_sk != NULL) {
		netlink_kernel_release(gf_dev->nl_sk);
		gf_dev->nl_sk = NULL;
		return 0;
	}

	pr_err("%s : no netlink socket yet\n", __func__);
	return -1;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gfspi_early_suspend(struct early_suspend *handler)
{
	struct gf_device *gf_dev = NULL;

	gf_dev = container_of(handler, struct gf_device, early_suspend);
	pr_info("%s\n", __func__);

	gfspi_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
}

static void gfspi_late_resume(struct early_suspend *handler)
{
	struct gf_device *gf_dev = NULL;

	gf_dev = container_of(handler, struct gf_device, early_suspend);
	pr_info("%s\n", __func__);

	gfspi_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
}
#else
static int gfspi_fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct gf_device *gf_dev = NULL;
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EARLY_EVENT_BLANK) {
		pr_debug("%s event = %ld", __func__, event);
		return 0;
	}

	gf_dev = container_of(self, struct gf_device, notifier);
	blank = *(int *)evdata->data;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		pr_debug("%s : lcd on notify\n", __func__);
		gfspi_netlink_send(gf_dev, GF_NETLINK_SCREEN_ON);
		break;

	case FB_BLANK_POWERDOWN:
		pr_debug("%s : lcd off notify\n", __func__);
		gfspi_netlink_send(gf_dev, GF_NETLINK_SCREEN_OFF);
		break;

	default:
		pr_debug("%s : other notifier, ignore\n", __func__);
		break;
	}
	return retval;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static ssize_t gfspi_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	return -EFAULT;
}

static ssize_t gfspi_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *f_pos)
{
	return -EFAULT;
}

static irqreturn_t gfspi_irq(int irq, void *handle)
{
	struct gf_device *gf_dev = (struct gf_device *)handle;

	pr_info("%s\n", __func__);
	wake_lock_timeout(&gf_dev->wake_lock,
			msecs_to_jiffies(WAKELOCK_HOLD_TIME));
	gfspi_netlink_send(gf_dev, GF_NETLINK_IRQ);
	gf_dev->interrupt_count++;

	return IRQ_HANDLED;
}

static long gfspi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_device *gf_dev = NULL;
	unsigned int onoff = 0;
	int retval = 0;
	u8  buf    = 0;
	u8 netlink_route = GF_NETLINK_ROUTE;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -EINVAL;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		retval = !access_ok(VERIFY_WRITE, (void __user *)arg,
				_IOC_SIZE(cmd));

	if (retval == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		retval = !access_ok(VERIFY_READ, (void __user *)arg,
				_IOC_SIZE(cmd));

	if (retval) {
		pr_err("%s: access NOK\n", __func__);
		return -EINVAL;
	}

	gf_dev = (struct gf_device *)filp->private_data;
	if (!gf_dev) {
		pr_err("%s: gf_dev IS NULL\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case GF_IOC_INIT:
		pr_info("%s: GF_IOC_INIT\n", __func__);
		if (copy_to_user((void __user *)arg, (void *)&netlink_route,
					sizeof(u8))) {
			retval = -EFAULT;
			break;
		}

		if (gf_dev->system_status) {
			pr_info("%s: system re-started\n", __func__);
			break;
		}

		gf_dev->sig_count = 0;
		gf_dev->system_status = 1;
		break;

	case GF_IOC_EXIT:
		pr_info("%s: GF_IOC_EXIT\n", __func__);
		gfspi_disable_irq(gf_dev);
		if (gf_dev->irq) {
			free_irq(gf_dev->irq, gf_dev);
			gf_dev->irq = 0;
		}

#ifdef CONFIG_HAS_EARLYSUSPEND
		if (gf_dev->early_suspend.suspend)
			unregister_early_suspend(&gf_dev->early_suspend);
#else
		fb_unregister_client(&gf_dev->notifier);
#endif
		gf_dev->system_status = 0;
		break;

	case GF_IOC_RESET:
		pr_info("%s: GF_IOC_RESET\n", __func__);
		gfspi_hw_reset(gf_dev, 0);
		break;

	case GF_IOC_ENABLE_IRQ:
		pr_info("%s: GF_IOC_ENABLE_IRQ\n", __func__);
		gfspi_enable_irq(gf_dev);
		break;

	case GF_IOC_DISABLE_IRQ:
		pr_info("%s: GF_IOC_DISABLE_IRQ\n", __func__);
		gfspi_disable_irq(gf_dev);
		break;

	case GF_IOC_ENABLE_SPI_CLK:
		pr_debug("%s: GF_IOC_ENABLE_SPI_CLK\n", __func__);
		gfspi_spi_clk_enable(gf_dev);
		break;

	case GF_IOC_DISABLE_SPI_CLK:
		pr_debug("%s: GF_IOC_DISABLE_SPI_CLK\n", __func__);
		gfspi_spi_clk_disable(gf_dev);
		break;

	case GF_IOC_ENABLE_POWER:
		pr_debug("%s: GF_IOC_ENABLE_POWER\n", __func__);
		gfspi_hw_power_enable(gf_dev, 1);
		break;

	case GF_IOC_DISABLE_POWER:
		pr_debug("%s: GF_IOC_DISABLE_POWER\n", __func__);
		gfspi_hw_power_enable(gf_dev, 0);
		break;

	case GF_IOC_POWER_CONTROL:
		if (copy_from_user(&onoff, (void __user *)arg,
					sizeof(unsigned int))) {
			pr_err("Failed to copy onoff value from user to kernel\n");
			retval = -EFAULT;
			break;
		}
		pr_info("%s: GF_IOC_POWER_CONTROL %d\n", __func__, onoff);
		gfspi_hw_power_enable(gf_dev, onoff);
		break;

	case GF_IOC_ENTER_SLEEP_MODE:
		break;

	case GF_IOC_GET_FW_INFO:
		buf = gf_dev->need_update;
		buf = 1;
		pr_debug("%s: GET_FW_INFO : 0x%x\n", __func__, buf);
		if (copy_to_user((void __user *)arg, (void *)&buf,
					sizeof(u8))) {
			pr_err("Failed to copy data to user\n");
			retval = -EFAULT;
		}

		break;
	case GF_IOC_REMOVE:
		break;

#ifndef ENABLE_SENSORS_FPRINT_SECURE
	case GF_IOC_TRANSFER_RAW_CMD:
		mutex_lock(&gf_dev->buf_lock);
		retval = gfspi_ioctl_transfer_raw_cmd(gf_dev, arg, bufsiz);
		mutex_unlock(&gf_dev->buf_lock);
		break;
#endif /* !ENABLE_SENSORS_FPRINT_SECURE */
#ifdef ENABLE_SENSORS_FPRINT_SECURE
	case GF_IOC_SET_SENSOR_TYPE:
		if (copy_from_user(&onoff, (void __user *)arg,
				   sizeof(unsigned int)) != 0) {
			pr_err("Failed to copy sensor type from user to kernel\n");
			return -EFAULT;
		}
		if ((int)onoff >= SENSOR_OOO && (int)onoff < SENSOR_MAXIMUM) {
			if ((int)onoff == SENSOR_OOO && gf_dev->sensortype == SENSOR_FAILED) {
				pr_err("%s Maintain type check from out of oder :%s\n",
					__func__, sensor_status[g_data->sensortype + 2]);
			} else {
				gf_dev->sensortype = (int)onoff;
				pr_info("%s SET_SENSOR_TYPE :%s\n",
					__func__,
					sensor_status[g_data->sensortype + 2]);
			}
		} else {
			pr_err("%s SET_SENSOR_TYPE : invalid value %d\n",
			     __func__, (int)onoff);
			gf_dev->sensortype = SENSOR_UNKNOWN;
		}
		break;

	case GF_IOC_SPEEDUP:
		if (copy_from_user(&onoff, (void __user *)arg,
				   sizeof(unsigned int)) != 0) {
			pr_err("Failed to copy speedup from user to kernel\n");
			return -EFAULT;
		}
#if defined(CONFIG_SECURE_OS_BOOSTER_API)
		if (onoff) {
			u8 retry_cnt = 0;

			pr_info("%s CPU_SPEEDUP ON\n", __func__);
			do {
				retval = secos_booster_start(onoff - 1);
				retry_cnt++;
				if (retval) {
					pr_err
					    ("%s: booster start failed. (%d) retry: %d\n",
					     __func__, retval, retry_cnt);
					if (retry_cnt < 7)
						usleep_range(500, 510);
				}
			} while (retval && retry_cnt < 7);
		} else {
			pr_info("%s CPU_SPEEDUP OFF\n", __func__);
			retval = secos_booster_stop();
			if (retval)
				pr_err("%s: booster stop failed. (%d)\n", __func__, retval);
		}
#elif defined(CONFIG_TZDEV_BOOST)
		if (onoff) {
			pr_info("%s CPU_SPEEDUP ON\n", __func__);
			tz_boost_enable();
		} else {
			pr_info("%s CPU_SPEEDUP OFF\n", __func__);
			tz_boost_disable();
		}
#else
		pr_err("%s CPU_SPEEDUP is not used\n", __func__);
#endif
		break;
	case GF_IOC_SET_LOCKSCREEN:
		break;
#endif
	case GF_IOC_GET_ORIENT:
		pr_info("%s: GET_ORIENT: %d\n",	__func__, gf_dev->orient);
		if (copy_to_user((void __user *)arg, &(gf_dev->orient),
					sizeof(gf_dev->orient))) {
			pr_err("Failed to copy data to user\n");
			retval = -EFAULT;
		}
		break;
	default:
		pr_err("%s doesn't support this command(%x)\n", __func__, cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gfspi_compat_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	int retval = 0;

	retval = filp->f_op->unlocked_ioctl(filp, cmd, arg);

	return retval;
}
#endif

static unsigned int gfspi_poll(struct file *filp,
		struct poll_table_struct *wait)
{
	pr_err("Not support poll opertion in TEE version\n");
	return -EFAULT;
}

static int gfspi_open(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int status = -ENXIO;

	pr_info("%s\n", __func__);
	mutex_lock(&device_list_lock);
	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devno == inode->i_rdev) {
			pr_info("%s, Found\n", __func__);
			status = 0;
			break;
		}
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		filp->private_data = gf_dev;
		nonseekable_open(inode, filp);
		pr_info("%s, Success to open device. irq = %d\n",
				__func__, gf_dev->irq);
	} else {
		pr_err("%s, No device for minor %d\n",
				__func__, iminor(inode));
	}
	return status;
}

static int gfspi_release(struct inode *inode, struct file *filp)
{
	struct gf_device *gf_dev = NULL;
	int    status = 0;

	pr_info("%s\n", __func__);
	gf_dev = filp->private_data;
	if (gf_dev->irq)
		gfspi_disable_irq(gf_dev);
	gf_dev->need_update = 0;
	return status;
}

static const struct file_operations gfspi_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.	It'll simplify things
	 * too, except for the locking.
	 */
	.write =	gfspi_write,
	.read =		gfspi_read,
	.unlocked_ioctl = gfspi_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gfspi_compat_ioctl,
#endif
	.open =		gfspi_open,
	.release =	gfspi_release,
	.poll	= gfspi_poll,
};


static void gfspi_work_func_debug(struct work_struct *work)
{
	struct gf_device *gf_dev = NULL;
	u8 ldo_value = -1;
	u8 rst_value = -1;
	u8 irq_value = -1;

	gf_dev = container_of(work, struct gf_device, work_debug);

	if (gf_dev->pwr_gpio)
		ldo_value = gpio_get_value(gf_dev->pwr_gpio);
	if (gf_dev->reset_gpio)
		rst_value = gpio_get_value(gf_dev->reset_gpio);
	if (gf_dev->irq_gpio)
		irq_value = gpio_get_value(gf_dev->irq_gpio);

	pr_info("%s ldo: %d, sleep: %d, irq: %d tz: %d type: %s\n",
		__func__,
		ldo_value, rst_value, irq_value, gf_dev->tz_mode,
		sensor_status[gf_dev->sensortype + 2]);
}

static void gfspi_enable_debug_timer(struct gf_device *gf_dev)
{
	mod_timer(&gf_dev->dbg_timer,
		round_jiffies_up(jiffies + FPSENSOR_DEBUG_TIMER_SEC));
}

static void gfspi_disable_debug_timer(struct gf_device *gf_dev)
{
	del_timer_sync(&gf_dev->dbg_timer);
	cancel_work_sync(&gf_dev->work_debug);
}

static void gfspi_timer_func(unsigned long ptr)
{
	queue_work(g_data->wq_dbg, &g_data->work_debug);
	mod_timer(&g_data->dbg_timer,
		round_jiffies_up(jiffies + FPSENSOR_DEBUG_TIMER_SEC));
}

static int gfspi_set_timer(struct gf_device *gf_dev)
{
	int status = 0;

	setup_timer(&gf_dev->dbg_timer,
			gfspi_timer_func, (unsigned long)gf_dev);
	gf_dev->wq_dbg = create_singlethread_workqueue("gf_debug_wq");
	if (!gf_dev->wq_dbg) {
		status = -ENOMEM;
		pr_err("%s could not create workqueue\n", __func__);
		return status;
	}
	INIT_WORK(&gf_dev->work_debug, gfspi_work_func_debug);
	return status;
}

void gfspi_hw_power_enable(struct gf_device *gf_dev, u8 onoff)
{
	if (onoff && !gf_dev->ldo_onoff) {
		gfspi_pin_control(gf_dev, 1);
		if (gf_dev->pwr_gpio)
			gpio_set_value(gf_dev->pwr_gpio, 1);
#if defined(ENABLE_SENSORS_FPRINT_SECURE) && defined(CONFIG_TZDEV)
		pr_info("%s: FP_SET_POWERON_INACTIVE ret = %d\n", __func__, 
			exynos_smc(FP_CSMC_HANDLER_ID, FP_HANDLER_MAIN, FP_SET_POWERON_INACTIVE, 0));
#endif
		if (gf_dev->reset_gpio) {
			usleep_range(11000, 11050);
			gpio_set_value(gf_dev->reset_gpio, 1);
		}
		gf_dev->ldo_onoff = 1;
	} else if (!onoff && gf_dev->ldo_onoff) {
#ifdef ENABLE_SENSORS_FPRINT_SECURE
#if defined (CONFIG_ARCH_EXYNOS9) || defined(CONFIG_ARCH_EXYNOS8)\
			|| defined (CONFIG_ARCH_EXYNOS7)
#if defined(CONFIG_TZDEV)
		pr_info("%s: FP_SET_POWEROFF ret = %d\n", __func__, 
			exynos_smc(FP_CSMC_HANDLER_ID, FP_HANDLER_MAIN, FP_SET_POWEROFF, 0));
#else
		pr_info("%s: cs_set smc ret = %d\n", __func__,
			exynos_smc(MC_FC_FP_CS_SET, 0, 0, 0));
#endif
#endif
#endif
		if (gf_dev->reset_gpio) {
			gpio_set_value(gf_dev->reset_gpio, 0);
			usleep_range(11000, 11050);
		}
		if (gf_dev->pwr_gpio)
			gpio_set_value(gf_dev->pwr_gpio, 0);
		gf_dev->ldo_onoff = 0;
		gfspi_pin_control(gf_dev, 0);
	} else if (onoff == 0 || onoff == 1) {
		pr_err("%s power is already %s\n",
				__func__,
				(gf_dev->ldo_onoff ? "Enabled" : "Disabled"));
	} else {
		pr_err("%s can't support this value:%d\n", __func__, onoff);
	}
	pr_info("%s status = %d\n", __func__, gf_dev->ldo_onoff);
}

void gfspi_hw_reset(struct gf_device *gf_dev, u8 delay)
{
	if (gf_dev == NULL) {
		pr_err("%s, Input buff is NULL.\n", __func__);
		return;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	usleep_range(3000, 3050);
	gpio_set_value(gf_dev->reset_gpio, 1);
	usleep_range((delay * 1000), ((delay * 1000) + 50));
	gf_dev->reset_count++;
}

#ifndef ENABLE_SENSORS_FPRINT_SECURE
int gfspi_type_check(struct gf_device *gf_dev)
{
	int status = -ENODEV;
	unsigned char chipid[4] = {0x00, 0x00, 0x00, 0x00};
	u32 chipid32 = 0;

	gfspi_hw_power_enable(gf_dev, 1);
	usleep_range(4950, 5000);
	gfspi_hw_reset(gf_dev, 0);

	gfspi_spi_read_bytes(gf_dev, 0x0000, 4, chipid);
	chipid32 = (chipid[2] << 16 | chipid[3] << 8 | chipid[0]);
	if (GF_GW32J_CHIP_ID == chipid32) {
		gf_dev->sensortype = SENSOR_GOODIX;
		pr_info("%s sensor type is GW32J (%s:0x%x)\n", __func__,
		sensor_status[gf_dev->sensortype + 2], chipid32);
		status = 0;
	} else if (GF_GW32N_CHIP_ID == chipid32) {
		gf_dev->sensortype = SENSOR_GOODIX;
		pr_info("%s sensor type is GW32N (%s:0x%x)\n", __func__,
		sensor_status[gf_dev->sensortype + 2], chipid32);
		status = 0;
	} else if (GF_GW36H_CHIP_ID == chipid32) {
		gf_dev->sensortype = SENSOR_GOODIX;
		pr_info("%s sensor type is GW36H (%s:0x%x)\n", __func__,
		sensor_status[gf_dev->sensortype + 2], chipid32);
		status = 0;
	} else if (GF_GW36C_CHIP_ID == chipid32) {
		gf_dev->sensortype = SENSOR_GOODIX;
		pr_info("%s sensor type is GW36C (%s:0x%x)\n", __func__,
		sensor_status[gf_dev->sensortype + 2], chipid32);
		status = 0;
	} else {
		gf_dev->sensortype = SENSOR_FAILED;
		pr_err("%s sensor type is FAILED 0x%x\n",
				__func__, chipid32);
	}
	gfspi_hw_power_enable(gf_dev, 0);
	return status;
}
#endif


static int gfspi_probe(struct spi_device *spi)
{
	struct gf_device *gf_dev = NULL;
	int status = -EINVAL;
#ifndef ENABLE_SENSORS_FPRINT_SECURE
	int retry = 0;
#endif
	pr_info("%s\n", __func__);

	/* Allocate driver data */
	gf_dev = kzalloc(sizeof(struct gf_device), GFP_KERNEL);
	if (!gf_dev) {
		status = -ENOMEM;
		return status;
	}

	spin_lock_init(&gf_dev->spi_lock);
	mutex_init(&gf_dev->buf_lock);
	mutex_init(&gf_dev->release_lock);

	INIT_LIST_HEAD(&gf_dev->device_entry);

	gf_dev->device_count = 0;
	gf_dev->system_status = 0;
	gf_dev->need_update = 0;
	gf_dev->ldo_onoff = 0;
	gf_dev->pid = 0;
	gf_dev->reset_count = 0;
	gf_dev->interrupt_count = 0;
#ifdef ENABLE_SENSORS_FPRINT_SECURE
	gf_dev->enabled_clk = 0;
	gf_dev->tz_mode = true;
	fpsensor_goto_suspend = 0;
#else
	gf_dev->tz_mode			= false;
#endif

	/* Initialize the driver data */
	gf_dev->spi = spi;
	g_data = gf_dev;

	gf_dev->irq = 0;
	spi_set_drvdata(spi, gf_dev);

	/* allocate buffer for SPI transfer */
#ifndef ENABLE_SENSORS_FPRINT_SECURE
	gf_dev->spi_buffer = kzalloc(bufsiz, GFP_KERNEL);
	if (gf_dev->spi_buffer == NULL) {
		status = -ENOMEM;
		goto err_buf;
	}
#endif

	/* get gpio info from dts or defination */
	status = gfspi_get_gpio_dts_info(&spi->dev, gf_dev);
	if (status < 0) {
		pr_err("%s, Failed to get gpio info:%d\n", __func__, status);
		goto err_get_gpio;
	}
	gfspi_spi_setup_conf(gf_dev, 4);

	/* create class */
	gf_dev->class = class_create(THIS_MODULE, GF_CLASS_NAME);
	if (IS_ERR(gf_dev->class)) {
		pr_err("%s, Failed to create class.\n", __func__);
		status = -ENODEV;
		goto err_class_create;
	}

	/* get device no */
	if (GF_DEV_MAJOR > 0) {
		gf_dev->devno = MKDEV(GF_DEV_MAJOR, gf_dev->device_count++);
		status = register_chrdev_region(gf_dev->devno, 1, GF_DEV_NAME);
	} else {
		status = alloc_chrdev_region(&gf_dev->devno,
				gf_dev->device_count++, 1, GF_DEV_NAME);
	}
	if (status < 0) {
		pr_err("%s, Failed to alloc devno.\n", __func__);
		goto err_devno;
	} else {
		pr_info("%s, major=%d, minor=%d\n",
				__func__, MAJOR(gf_dev->devno),
				MINOR(gf_dev->devno));
	}

	/* create device */
	gf_dev->fp_device = device_create(gf_dev->class, &spi->dev,
			gf_dev->devno, gf_dev, GF_DEV_NAME);
	if (IS_ERR(gf_dev->fp_device)) {
		pr_err("%s, Failed to create device.\n", __func__);
		status = -ENODEV;
		goto err_device;
	} else {
		mutex_lock(&device_list_lock);
		list_add(&gf_dev->device_entry, &device_list);
		mutex_unlock(&device_list_lock);
		pr_info("%s, device create success.\n", __func__);
	}

#ifdef ENABLE_SENSORS_FPRINT_SECURE
	gf_dev->sensortype = SENSOR_UNKNOWN;
#else
	/* sensor hw type check */
	do {
		status = gfspi_type_check(gf_dev);
		pr_info("%s type (%u), retry (%d)\n",
				__func__, gf_dev->sensortype, retry);
	} while (!gf_dev->sensortype && ++retry < 3);

	if (status == -ENODEV)
		pr_err("%s type_check failed\n", __func__);
#endif

#if defined(DISABLED_GPIO_PROTECTION)
	etspi_pin_control(etspi, 0);
#endif

	/* create sysfs */
	status = fingerprint_register(gf_dev->fp_device,
		gf_dev, fp_attrs, "fingerprint");
	if (status) {
		pr_err("%s sysfs register failed\n", __func__);
		goto err_sysfs;
	}

	/* cdev init and add */
	cdev_init(&gf_dev->cdev, &gfspi_fops);
	gf_dev->cdev.owner = THIS_MODULE;
	status = cdev_add(&gf_dev->cdev, gf_dev->devno, 1);
	if (status) {
		pr_err("%s, Failed to add cdev.\n", __func__);
		goto err_cdev;
	}

	/* netlink interface init */
	status = gfspi_netlink_init(gf_dev);
	if (status == -1) {
		pr_err("%s, Failed to init netlink.\n", __func__);
		goto err_netlink_init;
	}

	wake_lock_init(&gf_dev->wake_lock, WAKE_LOCK_SUSPEND, "gf_wake_lock");
	gf_dev->irq = gpio_to_irq(gf_dev->irq_gpio);
	status = request_threaded_irq(gf_dev->irq, NULL, gfspi_irq,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "goodix_fp_irq",
			gf_dev);
	if (status) {
		pr_err("%s irq thread request failed, retval=%d\n",
				__func__, status);
		goto err_request_irq;
	}

	enable_irq_wake(gf_dev->irq);
	gf_dev->irq_enabled = 1;
	gfspi_disable_irq(gf_dev);

	status = gfspi_set_timer(gf_dev);
	if (status)
		goto err_debug_timer;
	gfspi_enable_debug_timer(gf_dev);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	pr_info("%s : register_early_suspend\n", __func__);
	gf_dev->early_suspend.level = (EARLY_SUSPEND_LEVEL_DISABLE_FB - 1);
	gf_dev->early_suspend.suspend = gfspi_early_suspend,
	gf_dev->early_suspend.resume = gfspi_late_resume,
	register_early_suspend(&gf_dev->early_suspend);
#else
	/* register screen on/off callback */
	gf_dev->notifier.notifier_call = gfspi_fb_notifier_callback;
	fb_register_client(&gf_dev->notifier);
#endif
#ifdef ENABLE_SENSORS_FPRINT_SECURE
#if defined (CONFIG_ARCH_EXYNOS9) || defined(CONFIG_ARCH_EXYNOS8)\
			|| defined (CONFIG_ARCH_EXYNOS7)
	/* prevent spi_cs line floating */
#if !defined(CONFIG_TZDEV)
	pr_info("%s: cs_set smc ret = %d\n", __func__,
		exynos_smc(MC_FC_FP_CS_SET, 0, 0, 0));
#endif
#endif
#endif
	pr_info("%s probe finished\n", __func__);
	return 0;

err_debug_timer:
	free_irq(gf_dev->irq, gf_dev);
err_request_irq:
	gfspi_netlink_destroy(gf_dev);
err_netlink_init:
	cdev_del(&gf_dev->cdev);
err_cdev:
	fingerprint_unregister(gf_dev->fp_device, fp_attrs);
err_sysfs:
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);
err_device:
	unregister_chrdev_region(gf_dev->devno, 1);
err_devno:
	class_destroy(gf_dev->class);
err_class_create:
err_get_gpio:
#ifndef ENABLE_SENSORS_FPRINT_SECURE
	kfree(gf_dev->spi_buffer);
err_buf:
#endif
	mutex_destroy(&gf_dev->buf_lock);
	mutex_destroy(&gf_dev->release_lock);
	spi_set_drvdata(spi, NULL);
	gf_dev->spi = NULL;
	kfree(gf_dev);
	gf_dev = NULL;

	pr_err("%s failed. %d", __func__, status);
	return status;
}

static int gfspi_remove(struct spi_device *spi)
{
	struct gf_device *gf_dev = spi_get_drvdata(spi);

	pr_info("%s\n", __func__);

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq) {
		free_irq(gf_dev->irq, gf_dev);
		gf_dev->irq_enabled = 0;
		gf_dev->irq = 0;
	}

	gfspi_disable_debug_timer(gf_dev);
	wake_lock_destroy(&gf_dev->wake_lock);

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (gf_dev->early_suspend.suspend)
		unregister_early_suspend(&gf_dev->early_suspend);
#else
	fb_unregister_client(&gf_dev->notifier);
#endif

#ifndef ENABLE_SENSORS_FPRINT_SECURE
	mutex_lock(&gf_dev->release_lock);
	if (gf_dev->spi_buffer != NULL) {
		kfree(gf_dev->spi_buffer);
		gf_dev->spi_buffer = NULL;
	}
	mutex_unlock(&gf_dev->release_lock);
#endif
	fingerprint_unregister(gf_dev->fp_device, fp_attrs);
	gfspi_netlink_destroy(gf_dev);
	cdev_del(&gf_dev->cdev);
	device_destroy(gf_dev->class, gf_dev->devno);
	list_del(&gf_dev->device_entry);

	unregister_chrdev_region(gf_dev->devno, 1);
	class_destroy(gf_dev->class);
	gfspi_hw_power_enable(gf_dev, 0);

	spin_lock_irq(&gf_dev->spi_lock);
	spi_set_drvdata(spi, NULL);
	gf_dev->spi = NULL;
	spin_unlock_irq(&gf_dev->spi_lock);

	mutex_destroy(&gf_dev->buf_lock);
	mutex_destroy(&gf_dev->release_lock);

	kfree(gf_dev);
	return 0;
}

static int gfspi_pm_suspend(struct device *dev)
{
	struct gf_device *gf_dev = dev_get_drvdata(dev);
#ifdef ENABLE_SENSORS_FPRINT_SECURE
#if defined (CONFIG_ARCH_EXYNOS9) || defined(CONFIG_ARCH_EXYNOS8)\
		|| defined (CONFIG_ARCH_EXYNOS7)
	int ret = 0;

	fpsensor_goto_suspend = 1; /* used by pinctrl_samsung.c */
	
	if (!gf_dev->ldo_onoff) {
#if defined(CONFIG_TZDEV)
		ret = exynos_smc(FP_CSMC_HANDLER_ID, FP_HANDLER_MAIN, FP_SET_POWEROFF, 0);
		pr_info("%s: FP_SET_POWEROFF ret = %d\n", __func__, ret);
#else
		ret = exynos_smc(MC_FC_FP_PM_SUSPEND, 0, 0, 0);
		pr_info("%s: suspend smc ret = %d\n", __func__, ret);
#endif
	} else {
#if defined(CONFIG_TZDEV)
		ret = exynos_smc(FP_CSMC_HANDLER_ID, FP_HANDLER_MAIN, FP_SET_POWERON_INACTIVE, 0);
		pr_info("%s: FP_SET_POWERON_INACTIVE ret = %d\n", __func__, ret);
#else
		ret = exynos_smc(MC_FC_FP_PM_SUSPEND_CS_HIGH, 0, 0, 0);
		pr_info("%s: suspend_cs_high smc ret = %d\n", __func__, ret);
#endif
	}
#endif
#else
	pr_info("%s\n", __func__);
#endif
	gfspi_disable_debug_timer(gf_dev);
	return 0;
}

static int gfspi_pm_resume(struct device *dev)
{
	struct gf_device *gf_dev = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	gfspi_enable_debug_timer(gf_dev);
#if defined(ENABLE_SENSORS_FPRINT_SECURE)
	if (fpsensor_goto_suspend) {
		fps_resume_set();
	}
#endif
	return 0;
}

static const struct dev_pm_ops gfspi_pm_ops = {
	.suspend = gfspi_pm_suspend,
	.resume = gfspi_pm_resume
};

static struct spi_driver gfspi_spi_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.pm = &gfspi_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = gfspi_of_match,
#endif
	},
	.probe = gfspi_probe,
	.remove = gfspi_remove,
};

static int __init gfspi_init(void)
{
	int status = 0;

	pr_info("%s\n", __func__);

	status = spi_register_driver(&gfspi_spi_driver);
	if (status < 0) {
		pr_err("%s, Failed to register SPI driver.\n",
				__func__);
		return -EINVAL;
	}
	return status;
}
module_init(gfspi_init);

static void __exit gfspi_exit(void)
{
	pr_info("%s\n", __func__);
	spi_unregister_driver(&gfspi_spi_driver);
}
module_exit(gfspi_exit);


MODULE_AUTHOR("Samgsung");
MODULE_DESCRIPTION("Goodix FP sensor");
MODULE_LICENSE("GPL");
