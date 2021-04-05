/*
 * SAMSUNG NFC Controller
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Author: Woonki Lee <woonki84.lee@samsung.com>
 *         Heejae Kim <heejae12.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Last update: 2014-07-15
 *
 */
#define pr_fmt(fmt)     "[sec_nfc] %s: " fmt, __func__

#ifdef CONFIG_SEC_NFC_IF_I2C_GPIO
#define CONFIG_SEC_NFC_IF_I2C
#endif

#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "sec_nfc.h"
#include "./nfc_logger/nfc_logger.h"

#ifdef CONFIG_SEC_NFC_CLK_REQ
#include <linux/interrupt.h>
#endif
#include <linux/wakelock.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#ifdef CONFIG_SEC_NFC_LDO_CONTROL
#include <linux/regulator/consumer.h>
#endif


#ifndef CONFIG_SEC_NFC_IF_I2C
struct sec_nfc_i2c_info {};
#define sec_nfc_read			NULL
#define sec_nfc_write			NULL
#define sec_nfc_poll			NULL
#define sec_nfc_i2c_irq_clear(x)

#define SEC_NFC_GET_INFO(dev) platform_get_drvdata(to_platform_device(dev))

#else /* CONFIG_SEC_NFC_IF_I2C */
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#ifdef CONFIG_ESE_SECURE
#include <linux/smc.h>
#endif

#define SEC_NFC_GET_INFO(dev) i2c_get_clientdata(to_i2c_client(dev))
enum sec_nfc_irq {
	SEC_NFC_SKIP = -1,
	SEC_NFC_NONE,
	SEC_NFC_INT,
	SEC_NFC_READ_TIMES,
};

struct sec_nfc_i2c_info {
	struct i2c_client *i2c_dev;
	struct mutex read_mutex;
	enum sec_nfc_irq read_irq;
	wait_queue_head_t read_wait;
	size_t buflen;
	u8 *buf;
};

#endif

struct sec_nfc_info {
	struct miscdevice miscdev;
	struct mutex mutex;
	enum sec_nfc_mode mode;
	struct device *dev;
	struct sec_nfc_platform_data *pdata;
	struct sec_nfc_i2c_info i2c_info;
	struct wake_lock nfc_wake_lock;
#ifdef	CONFIG_SEC_NFC_CLK_REQ
	bool clk_ctl;
	bool clk_state;
#endif
	void __iomem	*clkctrl;
	int clk_irq;
};

#define FEATURE_SEC_NFC_TEST
#ifdef FEATURE_SEC_NFC_TEST
static struct sec_nfc_info *g_nfc_info;
static bool on_nfc_test;
static bool nfc_int_wait;
#endif
extern unsigned int lpcharge;
#ifdef CONFIG_SEC_NFC_IF_I2C
static irqreturn_t sec_nfc_irq_thread_fn(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;
	struct sec_nfc_platform_data *pdata = info->pdata;

	NFC_LOG_REC("irq\n");

#ifdef FEATURE_SEC_NFC_TEST
	if (on_nfc_test) {
		nfc_int_wait = true;
		NFC_LOG_INFO("NFC_TEST: interrupt is raised\n");
		wake_up_interruptible(&info->i2c_info.read_wait);
		return IRQ_HANDLED;
	}
#endif

	if (gpio_get_value(pdata->irq) == 0) {
		NFC_LOG_REC("irq-gpio state is low!\n");
		return IRQ_HANDLED;
	}
	mutex_lock(&info->i2c_info.read_mutex);
	/* Skip interrupt during power switching
	 * It is released after first write */
	if (info->i2c_info.read_irq == SEC_NFC_SKIP) {
		NFC_LOG_REC("Now power swiching. Skip this IRQ\n");
		mutex_unlock(&info->i2c_info.read_mutex);
		return IRQ_HANDLED;
	}

	info->i2c_info.read_irq += SEC_NFC_READ_TIMES;
	mutex_unlock(&info->i2c_info.read_mutex);

	wake_up_interruptible(&info->i2c_info.read_wait);
	wake_lock_timeout(&info->nfc_wake_lock, 2*HZ);

	return IRQ_HANDLED;
}

static irqreturn_t sec_nfc_clk_irq(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;
	struct sec_nfc_platform_data *pdata = info->pdata;
	struct device_node *np = info->dev->of_node;
	struct property *prop;
	
	prop = of_find_property(np, "sec-nfc,manual_clkctrl", NULL);
	if (prop) {
		unsigned int val = readl(info->clkctrl);
		if (gpio_get_value(pdata->clk_irq))
			val &= ~SEC_NFC_CLKCTRL_REQ_POLA;
		else
			val |= SEC_NFC_CLKCTRL_REQ_POLA;
		writel(val, info->clkctrl);
	}
	return IRQ_HANDLED;
}

static int nfc_state_print(struct sec_nfc_info *info)
{
	int en = gpio_get_value(info->pdata->ven);
	int firm = gpio_get_value(info->pdata->firm);
	int irq = gpio_get_value(info->pdata->irq);
	int pvdd = 0;

#ifdef CONFIG_SEC_NFC_LDO_CONTROL
	struct regulator *regulator_nfc_pvdd;

	regulator_nfc_pvdd = regulator_get(NULL, info->pdata->i2c_1p8);
	if (IS_ERR(regulator_nfc_pvdd) || regulator_nfc_pvdd == NULL) {
		NFC_LOG_ERR("%s - nfc_pvdd regulator_get fail\n", __func__);
		return -ENODEV;
	}
	pvdd = regulator_is_enabled(regulator_nfc_pvdd);
	regulator_put(regulator_nfc_pvdd);
#else
	pvdd = gpio_get_value(info->pdata->pvdd_en);
#endif

	NFC_LOG_INFO("%s en: %d, firm: %d power: %d irq: %d\n", __func__, en, firm, pvdd, irq);
	NFC_LOG_INFO("%s mode %d\n",__func__ , info->mode);

	return 0;
}

static ssize_t sec_nfc_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	enum sec_nfc_irq irq;
	int ret = 0;

#ifdef FEATURE_SEC_NFC_TEST
	if (on_nfc_test)
		return 0;
#endif

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		NFC_LOG_ERR("read() nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	mutex_lock(&info->i2c_info.read_mutex);
	if (count == 0) {
		if (info->i2c_info.read_irq >= SEC_NFC_INT)
			info->i2c_info.read_irq--;
		mutex_unlock(&info->i2c_info.read_mutex);
		goto out;
	}

	irq = info->i2c_info.read_irq;
	mutex_unlock(&info->i2c_info.read_mutex);
	if (irq == SEC_NFC_NONE) {
		if (file->f_flags & O_NONBLOCK) {
			NFC_LOG_ERR("read() it is nonblock\n");
			ret = -EAGAIN;
			goto out;
		}
	}

	/* i2c recv */
	if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

	if (count > SEC_NFC_MSG_MAX_SIZE) {
		NFC_LOG_ERR("read() user required wrong size :%d\n", (u32)count);
		ret = -EINVAL;
		goto out;
	}

	NFC_LOG_REC("read(%zu)\n", count);
	mutex_lock(&info->i2c_info.read_mutex);
	memset(info->i2c_info.buf, 0, count);
	ret = i2c_master_recv(info->i2c_info.i2c_dev, info->i2c_info.buf, (u32)count);

	if (ret == -EREMOTEIO) {
		ret = -ERESTART;
		goto read_error;
	} else if (ret != count) {
		NFC_LOG_ERR("read failed: return: %d count: %d\n",
			ret, (u32)count);
		/*ret = -EREMOTEIO;*/
		goto read_error;
	}

	if (info->i2c_info.read_irq >= SEC_NFC_INT)
		info->i2c_info.read_irq--;

	if (info->i2c_info.read_irq == SEC_NFC_READ_TIMES)
		wake_up_interruptible(&info->i2c_info.read_wait);

	mutex_unlock(&info->i2c_info.read_mutex);

	if (copy_to_user(buf, info->i2c_info.buf, ret)) {
		NFC_LOG_ERR("read() copy failed to user\n");
		ret = -EFAULT;
	}

	goto out;

read_error:
	NFC_LOG_ERR("read error %d\n", ret);
	nfc_state_print(info);
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);
out:
	mutex_unlock(&info->mutex);

	return ret;
}

static ssize_t sec_nfc_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	int ret = 0;

	NFC_LOG_DBG("write() count %d\n", (u32)count);

#ifdef FEATURE_SEC_NFC_TEST
	if (on_nfc_test)
		return 0;
#endif

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		NFC_LOG_ERR("write() nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

	if (count > SEC_NFC_MSG_MAX_SIZE) {
		NFC_LOG_ERR("write() user required wrong size :%d\n", (u32)count);
		ret = -EINVAL;
		goto out;
	}

	if (copy_from_user(info->i2c_info.buf, buf, count)) {
		NFC_LOG_ERR("write() copy failed from user\n");
		ret = -EFAULT;
		goto out;
	}

	/* Skip interrupt during power switching
	 * It is released after first write
	 */
	NFC_LOG_REC("write(%d)\n", count);
	mutex_lock(&info->i2c_info.read_mutex);
	ret = i2c_master_send(info->i2c_info.i2c_dev, info->i2c_info.buf, count);
	if (info->i2c_info.read_irq == SEC_NFC_SKIP)
		info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);

	if (ret == -EREMOTEIO) {
		NFC_LOG_ERR("write failed: return: %d count: %d\n",
			ret, (u32)count);
		ret = -ERESTART;
		goto write_error;
	}
	if (ret != count) {
		NFC_LOG_ERR("write failed: return: %d count: %d\n",
			ret, (u32)count);
		ret = -EREMOTEIO;
		goto write_error;
	}

	goto out;

write_error:
	nfc_state_print(info);
out:
	mutex_unlock(&info->mutex);

	return ret;
}

static unsigned int sec_nfc_poll(struct file *file, poll_table *wait)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	enum sec_nfc_irq irq;

	int ret = 0;

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		NFC_LOG_ERR("poll() nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	poll_wait(file, &info->i2c_info.read_wait, wait);

	mutex_lock(&info->i2c_info.read_mutex);
	irq = info->i2c_info.read_irq;
	if (irq == SEC_NFC_READ_TIMES)
		ret = (POLLIN | POLLRDNORM);
	mutex_unlock(&info->i2c_info.read_mutex);

out:
	mutex_unlock(&info->mutex);

	return ret;
}

#ifdef CONFIG_SEC_NFC_LDO_CONTROL
static int sec_nfc_regulator_onoff(struct sec_nfc_platform_data *data, int onoff)
{
	int rc = 0;
	struct regulator *regulator_i2c_1p8;

	regulator_i2c_1p8 = regulator_get(NULL, data->i2c_1p8);
	if (IS_ERR(regulator_i2c_1p8) || regulator_i2c_1p8 == NULL) {
		NFC_LOG_ERR("nfc_pvdd regulator_get fail\n");
		return -ENODEV;
	}

	NFC_LOG_INFO("regulator onoff = %d\n", onoff);

	if (onoff == NFC_I2C_LDO_ON) {
		rc = regulator_enable(regulator_i2c_1p8);
		if (rc) {
			NFC_LOG_ERR("regulator enable nfc_pvdd failed, rc=%d\n", rc);
			goto done;
		}
	} else {
		rc = regulator_disable(regulator_i2c_1p8);
		if (rc) {
			NFC_LOG_ERR("regulator disable nfc_pvdd failed, rc=%d\n", rc);
			goto done;
		}
	}

done:
	regulator_put(regulator_i2c_1p8);

	return rc;
}
#endif

void sec_nfc_i2c_irq_clear(struct sec_nfc_info *info)
{
	/* clear interrupt. Interrupt will be occurred at power off */
	mutex_lock(&info->i2c_info.read_mutex);
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);
}

int sec_nfc_i2c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct sec_nfc_platform_data *pdata = info->pdata;
	struct property *prop;
	struct device_node *np = dev->of_node;
	int ret;

	NFC_LOG_INFO("probe() start\n");

	info->i2c_info.buflen = SEC_NFC_MAX_BUFFER_SIZE;
	info->i2c_info.buf = kzalloc(SEC_NFC_MAX_BUFFER_SIZE, GFP_KERNEL);
	if (!info->i2c_info.buf) {
		NFC_LOG_ERR("probe() failed to allocate memory\n");
		return -ENOMEM;
	}
	info->i2c_info.i2c_dev = client;
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_init(&info->i2c_info.read_mutex);
	init_waitqueue_head(&info->i2c_info.read_wait);
	i2c_set_clientdata(client, info);

	ret = gpio_request(pdata->irq, "nfc_int");
	if (ret) {
		NFC_LOG_ERR("probe() GPIO request is failed to register IRQ\n");
		goto err_irq_req;
	}
	client->irq = gpio_to_irq(pdata->irq);
	NFC_LOG_INFO("push interrupt no = %d\n", client->irq);

	ret = request_threaded_irq(client->irq, NULL, sec_nfc_irq_thread_fn,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, SEC_NFC_DRIVER_NAME,
			info);
	if (ret < 0) {
		NFC_LOG_ERR("probe() failed to register IRQ handler\n");
		kfree(info->i2c_info.buf);
		return ret;
	}

	prop = of_find_property(np, "sec-nfc,nfc_clkint", NULL);
	if(prop){
		info->clk_irq = gpio_to_irq(pdata->clk_irq);
		ret = request_threaded_irq(info->clk_irq, NULL, sec_nfc_clk_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "nfc_clk",
				info);
		if (ret < 0) {
			NFC_LOG_ERR("failed to register IRQ handler\n");
			kfree(info->i2c_info.buf);
			return ret;
		}
		ret = enable_irq_wake(info->clk_irq);
		if (ret < 0)
			NFC_LOG_ERR("%s: Failed to Enable Wakeup Source(%d)\n", __func__, ret);
	}

	prop = of_find_property(np, "sec-nfc,pvdd_en", NULL);
	if(prop){
		ret = gpio_request(pdata->pvdd_en, "nfc_pvdd_en");
		if (ret < 0){
			NFC_LOG_ERR("failed to request about pvdd_en pin\n");
			return -ENODEV;
		}
		if(!lpcharge) {
			gpio_direction_output(pdata->pvdd_en, 1);
		}
		NFC_LOG_INFO("pvdd en: %d\n", gpio_get_value(pdata->pvdd_en));
	}
#ifdef CONFIG_SEC_NFC_LDO_CONTROL
	if (pdata->i2c_1p8 != NULL) {
		if(!lpcharge) {
			ret = sec_nfc_regulator_onoff(pdata, NFC_I2C_LDO_ON);
			if (ret < 0)
				NFC_LOG_ERR("max86900_regulator_on fail err = %d\n", ret);
			usleep_range(1000, 1100);
		}
	}
#endif
	NFC_LOG_INFO("probe() success\n");
	return 0;

err_irq_req:
	return ret;
}

void sec_nfc_i2c_remove(struct device *dev)
{
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_info.i2c_dev;
	struct sec_nfc_platform_data *pdata = info->pdata;
	free_irq(client->irq, info);
	gpio_free(pdata->irq);
}
#endif /* CONFIG_SEC_NFC_IF_I2C */

static void set_pd(struct sec_nfc_info *info, int power)
{
	struct device_node *np = info->dev->of_node;
	struct property *prop;
	prop = of_find_property(np, "sec-nfc,ven-gpio", NULL);
	if (prop) {
		struct sec_nfc_platform_data *pdata = info->pdata;
		gpio_set_value(pdata->ven, power);
	}
	else {
		unsigned int val = readl(info->clkctrl);
		int pd_active = (val & SEC_NFC_CLKCTRL_PD_POLA);

		if (pd_active > 0) {
			if (power == SEC_NFC_PW_ON)
				val |= SEC_NFC_CLKCTRL_PD;
			else
				val &= ~SEC_NFC_CLKCTRL_PD;
		} else {
			if (power == SEC_NFC_PW_ON)
				val &= ~SEC_NFC_CLKCTRL_PD;
			else
				val |= SEC_NFC_CLKCTRL_PD;
		}
		writel(val, info->clkctrl);
	}
}

#ifdef	CONFIG_SEC_NFC_CLK_REQ
static irqreturn_t sec_nfc_clk_irq_thread(int irq, void *dev_id)
{
	struct sec_nfc_info *info = dev_id;
	struct sec_nfc_platform_data *pdata = info->pdata;
	bool value;

	value = gpio_get_value(pdata->clk_req) > 0 ? true : false;
	NFC_LOG_REC("clock req: %d\n", value);

	value = gpio_get_value(pdata->clk_req) > 0 ? 1 : 0;
	gpio_set_value(pdata->clk, value);

	info->clk_state = value;

	return IRQ_HANDLED;
}

void sec_nfc_clk_ctl_enable(struct sec_nfc_info *info)
{
	struct sec_nfc_platform_data *pdata = info->pdata;
	unsigned int irq = gpio_to_irq(pdata->clk_req);
	int ret;

	if (info->clk_ctl)
		return;

	info->clk_state = false;
	ret = request_threaded_irq(irq, NULL, sec_nfc_clk_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			SEC_NFC_DRIVER_NAME, info);
	if (ret < 0) {
		NFC_LOG_ERR(info->dev, "failed to register CLK REQ IRQ handler\n");
	}
	info->clk_ctl = true;
}
void sec_nfc_clk_ctl_disable(struct sec_nfc_info *info)
{
	struct sec_nfc_platform_data *pdata = info->pdata;
	unsigned int irq = gpio_to_irq(pdata->clk_req);

	if (!info->clk_ctl)
		return;

	free_irq(irq, info);
	if (info->clk_state)
		clk_disable_unprepare(pdata->clk);

	info->clk_state = false;
	info->clk_ctl = false;
}
#else
#define sec_nfc_clk_ctl_enable(x)
#define sec_nfc_clk_ctl_disable(x)
#endif /* CONFIG_SEC_NFC_CLK_REQ */

static void sec_nfc_set_mode(struct sec_nfc_info *info,
					enum sec_nfc_mode mode)
{
	struct sec_nfc_platform_data *pdata = info->pdata;

	/* intfo lock is aleady gotten before calling this function */
	if (info->mode == mode) {
		NFC_LOG_DBG("power mode is already %d", mode);
		return;
	}
	info->mode = mode;

#ifdef CONFIG_SEC_NFC_IF_I2C
	/* Skip interrupt during power switching
	 * It is released after first write
	 */
	mutex_lock(&info->i2c_info.read_mutex);
	info->i2c_info.read_irq = SEC_NFC_SKIP;
	mutex_unlock(&info->i2c_info.read_mutex);
#endif

	set_pd(info, SEC_NFC_PW_OFF);
	if (pdata->firm) gpio_set_value(pdata->firm, SEC_NFC_FW_OFF);

	if (mode == SEC_NFC_MODE_BOOTLOADER)
		if (pdata->firm) gpio_set_value(pdata->firm, SEC_NFC_FW_ON);

	if (mode != SEC_NFC_MODE_OFF) {
		msleep(SEC_NFC_VEN_WAIT_TIME);
		set_pd(info, SEC_NFC_PW_ON);
#ifdef CONFIG_SEC_NFC_IF_I2C
		enable_irq_wake(info->i2c_info.i2c_dev->irq);
#endif
		msleep(SEC_NFC_VEN_WAIT_TIME/2);
	} else {
#ifdef CONFIG_SEC_NFC_IF_I2C
		disable_irq_wake(info->i2c_info.i2c_dev->irq);
#endif
	}

	if (wake_lock_active(&info->nfc_wake_lock))
		wake_unlock(&info->nfc_wake_lock);

	NFC_LOG_INFO("NFC mode is : %d\n", mode);
}

static long sec_nfc_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	struct sec_nfc_platform_data *pdata = info->pdata;
	unsigned int new = (unsigned int)arg;
	int ret = 0;

	NFC_LOG_DBG("cmd: 0x%x\n", cmd);

	mutex_lock(&info->mutex);

	switch (cmd) {
	case SEC_NFC_DEBUG:
		NFC_LOG_ERR("SEC_NFC_DEBUG\n");
		nfc_state_print(info);
		break;
	case SEC_NFC_SET_MODE:
		NFC_LOG_DBG("%s: SEC_NFC_SET_MODE\n", __func__);

		if (info->mode == new)
			break;

		if (new >= SEC_NFC_MODE_COUNT) {
			NFC_LOG_ERR("wrong mode (%d)\n", new);
			ret = -EFAULT;
			break;
		}
		sec_nfc_set_mode(info, new);

		break;

#if defined(CONFIG_SEC_NFC_PRODUCT_N5)
	case SEC_NFC_SLEEP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER) {
			if (wake_lock_active(&info->nfc_wake_lock))
				wake_unlock(&info->nfc_wake_lock);
			gpio_set_value(pdata->wake, SEC_NFC_WAKE_SLEEP);
		}
		break;

	case SEC_NFC_WAKEUP:
		if (info->mode != SEC_NFC_MODE_BOOTLOADER) {
			gpio_set_value(pdata->wake, SEC_NFC_WAKE_UP);
			if (!wake_lock_active(&info->nfc_wake_lock))
				wake_lock(&info->nfc_wake_lock);
		}
		break;
#endif
	case SEC_NFC_SET_NPT_MODE:
		if (SEC_NFC_NPT_CMD_ON == new) {
			NFC_LOG_INFO("NPT: NFC OFF mode NPT - Turn on VEN.\n");
			info->mode = SEC_NFC_MODE_FIRMWARE;
			mutex_lock(&info->i2c_info.read_mutex);
			info->i2c_info.read_irq = SEC_NFC_SKIP;
			mutex_unlock(&info->i2c_info.read_mutex);
			set_pd(info, SEC_NFC_PW_ON);
#ifdef  CONFIG_SEC_NFC_CLK_REQ
			sec_nfc_clk_ctl_enable(info);
#endif
			msleep(20);
			if (pdata->firm) gpio_set_value(pdata->firm, SEC_NFC_FW_ON);
			enable_irq_wake(info->i2c_info.i2c_dev->irq);
		} else if(SEC_NFC_NPT_CMD_OFF == new) {
			NFC_LOG_INFO("NPT: NFC OFF mode NPT - Turn off VEN.\n");
			info->mode = SEC_NFC_MODE_OFF;
			if (pdata->firm) gpio_set_value(pdata->firm, SEC_NFC_FW_OFF);
			set_pd(info, SEC_NFC_PW_OFF);
#ifdef  CONFIG_SEC_NFC_CLK_REQ
			sec_nfc_clk_ctl_disable(info);
#endif
			disable_irq_wake(info->i2c_info.i2c_dev->irq);
		}
		break;
	default:
		NFC_LOG_ERR("Unknown ioctl 0x%x\n", cmd);
		ret = -ENOIOCTLCMD;
		break;
	}

	mutex_unlock(&info->mutex);

	return ret;
}

static int sec_nfc_open(struct inode *inode, struct file *file)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	int ret = 0;

	NFC_LOG_INFO("%s\n", __func__);

	mutex_lock(&info->mutex);
	if (info->mode != SEC_NFC_MODE_OFF) {
		NFC_LOG_ERR("open() nfc is busy\n");
		nfc_state_print(info);
		ret = -EBUSY;
		goto out;
	}

	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);

out:
	mutex_unlock(&info->mutex);
	return ret;
}

static int sec_nfc_close(struct inode *inode, struct file *file)
{
	struct sec_nfc_info *info = container_of(file->private_data,
						struct sec_nfc_info, miscdev);
	nfc_state_print(info);

	NFC_LOG_INFO("%s\n", __func__);

	mutex_lock(&info->mutex);
	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	mutex_unlock(&info->mutex);

	return 0;
}

static const struct file_operations sec_nfc_fops = {
	.owner		= THIS_MODULE,
	.read		= sec_nfc_read,
	.write		= sec_nfc_write,
	.poll		= sec_nfc_poll,
	.open		= sec_nfc_open,
	.release	= sec_nfc_close,
	.unlocked_ioctl	= sec_nfc_ioctl,
	.compat_ioctl = sec_nfc_ioctl,
};

#ifdef CONFIG_PM
static int sec_nfc_suspend(struct device *dev)
{
	struct sec_nfc_info *info = SEC_NFC_GET_INFO(dev);
	int ret = 0;

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_BOOTLOADER)
		ret = -EPERM;

	mutex_unlock(&info->mutex);

	return ret;
}

static int sec_nfc_resume(struct device *dev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(sec_nfc_pm_ops, sec_nfc_suspend, sec_nfc_resume);
#endif

#ifdef CONFIG_OF
/*device tree parsing*/
static int sec_nfc_parse_dt(struct device *dev,
	struct sec_nfc_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;

	prop = of_find_property(np, "sec-nfc,ven-gpio", NULL);
	if (prop)
		pdata->ven = of_get_named_gpio(np, "sec-nfc,ven-gpio", 0);
	else {
		if (of_property_read_u32(np, "clkctrl-reg", (u32 *)&pdata->clkctrl_addr))
			return -EINVAL;
	}
	pdata->firm = of_get_named_gpio(np, "sec-nfc,firm-gpio", 0);
	pdata->wake = pdata->firm;

	prop = of_find_property(np, "sec-nfc,nfc_clkint", NULL);
	if(prop)
		pdata->clk_irq = of_get_named_gpio(np, "sec-nfc,nfc_clkint", 0);

#ifdef CONFIG_SEC_NFC_IF_I2C
	pdata->irq = of_get_named_gpio(np, "sec-nfc,irq-gpio", 0);
#endif
#ifdef CONFIG_SEC_NFC_CLK_REQ
	pdata->clk_req = of_get_named_gpio(np, "sec-nfc,clk_req-gpio", 0);
#endif
	prop = of_find_property(np, "sec-nfc,pvdd_en", NULL);
	if(prop)
		pdata->pvdd_en = of_get_named_gpio(np, "sec-nfc,pvdd_en",0);

#ifdef CONFIG_SEC_NFC_LDO_CONTROL
	if (of_property_read_string(np, "sec-nfc,i2c_1p8",
		&pdata->i2c_1p8) < 0) {
		NFC_LOG_ERR("%s - get i2c_1p8 error\n", __func__);
		pdata->i2c_1p8 = NULL;
	}
#endif
	if (prop) {
		NFC_LOG_INFO("%s: irq : %d, firm : %d\n",__func__, pdata->irq, pdata->firm);
	} else
		NFC_LOG_INFO("%s: clk_req_ addr , irq : %d, \n",__func__,pdata->irq);
	return 0;
}
#else
static int sec_nfc_parse_dt(struct device *dev,
	struct sec_nfc_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#ifdef FEATURE_SEC_NFC_TEST
static int sec_nfc_i2c_read(char *buf, int count)
{
	struct sec_nfc_info *info = g_nfc_info;
	int ret = 0;

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		NFC_LOG_ERR("NFC_TEST: sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	/* i2c recv */
	if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

	if (count > SEC_NFC_MSG_MAX_SIZE) {
		NFC_LOG_ERR("NFC_TEST: user required wrong size :%d\n", (u32)count);
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&info->i2c_info.read_mutex);
	memset(buf, 0, count);
	ret = i2c_master_recv(info->i2c_info.i2c_dev, buf, (u32)count);
	NFC_LOG_INFO("NFC_TEST: recv size : %d\n", ret);

	if (ret == -EREMOTEIO) {
		ret = -ERESTART;
		goto read_error;
	} else if (ret != count) {
		NFC_LOG_ERR("NFC_TEST: read failed: return: %d count: %d\n",
			ret, (u32)count);
		goto read_error;
	}

	mutex_unlock(&info->i2c_info.read_mutex);

	goto out;

read_error:
	info->i2c_info.read_irq = SEC_NFC_NONE;
	mutex_unlock(&info->i2c_info.read_mutex);
out:
	mutex_unlock(&info->mutex);

	return ret;
}

static int sec_nfc_i2c_write(char *buf,	int count)
{
	struct sec_nfc_info *info = g_nfc_info;
	int ret = 0;

	mutex_lock(&info->mutex);

	if (info->mode == SEC_NFC_MODE_OFF) {
		NFC_LOG_ERR("NFC_TEST: sec_nfc is not enabled\n");
		ret = -ENODEV;
		goto out;
	}

	if (count > info->i2c_info.buflen)
		count = info->i2c_info.buflen;

	if (count > SEC_NFC_MSG_MAX_SIZE) {
		NFC_LOG_ERR("NFC_TEST: user required wrong size :%d\n", (u32)count);
		ret = -EINVAL;
		goto out;
	}

	mutex_lock(&info->i2c_info.read_mutex);
	ret = i2c_master_send(info->i2c_info.i2c_dev, buf, count);
	mutex_unlock(&info->i2c_info.read_mutex);
	if (ret == -EREMOTEIO) {
		NFC_LOG_ERR("NFC_TEST: send failed: return: %d count: %d\n",
		ret, (u32)count);
		ret = -ERESTART;
		goto out;
	}

	if (ret != count) {
		NFC_LOG_ERR("NFC_TEST: send failed: return: %d count: %d\n",
		ret, (u32)count);
		ret = -EREMOTEIO;
	}

out:
	mutex_unlock(&info->mutex);

	return ret;
}

static ssize_t sec_nfc_test_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	char cmd[8] = {0x0, 0x1, 0x0, 0x0,}; /*bootloader fw check*/
	enum sec_nfc_mode old_mode = g_nfc_info->mode;
	int size;
	int ret = 0;
	int timeout = 1;

	on_nfc_test = true;
	nfc_int_wait = false;
	sec_nfc_set_mode(g_nfc_info, SEC_NFC_MODE_BOOTLOADER);
	ret = sec_nfc_i2c_write(cmd, 4);

	if (ret < 0) {
		NFC_LOG_INFO("NFC_TEST: i2c write error %d\n", ret);
		size = sprintf(buf, "NFC_TEST: i2c write error %d\n", ret);
		goto exit;
	}

	timeout = wait_event_interruptible_timeout(g_nfc_info->i2c_info.read_wait, nfc_int_wait, 100);
	ret = sec_nfc_i2c_read(buf, 16);

	if (ret < 0) {
		NFC_LOG_INFO("NFC_TEST: i2c read error %d\n", ret);
		size = sprintf(buf, "NFC_TEST: i2c read error %d\n", ret);
		goto exit;
	}
	NFC_LOG_INFO("NFC_TEST: BL ver: %02X %02X %02X %02X, INT: %s\n", buf[0],
					buf[1],	buf[2], buf[3], timeout ? "OK":"NOK");
	size = sprintf(buf, "BL ver: %02X.%02X.%02X.%02X, INT: %s\n", buf[0],
					buf[1], buf[2],	buf[3], timeout ? "OK":"NOK");

exit:
	sec_nfc_set_mode(g_nfc_info, old_mode);
	on_nfc_test = false;

	return size;
}
static ssize_t sec_nfc_test_store(struct class *dev,
					struct class_attribute *attr,
					const char *buf, size_t size)
{
	return size;
}

static CLASS_ATTR(test, 0664, sec_nfc_test_show, sec_nfc_test_store);
#endif

static ssize_t sec_nfc_support_show(struct class *class,
					struct class_attribute *attr,
					char *buf)
{
	NFC_LOG_INFO("\n");
	return 0;
}
static CLASS_ATTR(nfc_support, 0444, sec_nfc_support_show, NULL);

static int __sec_nfc_probe(struct device *dev)
{
	struct sec_nfc_info *info;
	struct sec_nfc_platform_data *pdata = NULL;
	int ret = 0;
	struct device_node *np = dev->of_node;
	struct property *prop;
	unsigned int val = 0;
	int nfc_support = 0;
#ifdef FEATURE_SEC_NFC_TEST
	struct class *nfc_class;
#endif
	NFC_LOG_INFO("probe start\n");
	if (dev->of_node) {
		pdata = devm_kzalloc(dev,
			sizeof(struct sec_nfc_platform_data), GFP_KERNEL);
		if (!pdata) {
			NFC_LOG_ERR("probe() Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = sec_nfc_parse_dt(dev, pdata);
		if (ret)
			return ret;
	} else {
		pdata = dev->platform_data;
	}

	if (!pdata) {
		NFC_LOG_ERR("probe() No platform data\n");
		ret = -ENOMEM;
		goto err_pdata;
	}

	info = kzalloc(sizeof(struct sec_nfc_info), GFP_KERNEL);
	if (!info) {
		NFC_LOG_ERR("probe() failed to allocate memory for sec_nfc_info\n");
		ret = -ENOMEM;
		goto err_info_alloc;
	}
	info->dev = dev;
	info->pdata = pdata;
	info->mode = SEC_NFC_MODE_OFF;

	mutex_init(&info->mutex);
	dev_set_drvdata(dev, info);

	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = SEC_NFC_DRIVER_NAME;
	info->miscdev.fops = &sec_nfc_fops;
	info->miscdev.parent = dev;

	/*separate NFC / non NFC using GPIO*/
	prop = of_find_property(np, "sec-nfc,check_nfc", NULL);
	if (prop) {
		nfc_support = gpio_get_value(of_get_named_gpio(np, "sec-nfc,check_nfc", 0));
		if (nfc_support > 0) {
			NFC_LOG_INFO("%s : nfc support model : %d\n", __func__, nfc_support);
		}else{
			NFC_LOG_INFO("%s : nfc not support model : %d\n", __func__, nfc_support);
			return -ENXIO;
		}
	}

	ret = misc_register(&info->miscdev);
	if (ret < 0) {
		NFC_LOG_ERR("probe() failed to register Device\n");
		goto err_dev_reg;
	}

	prop = of_find_property(np, "sec-nfc,ven-gpio", NULL);
	if (prop) {
		ret = gpio_request(pdata->ven, "nfc_ven");
		if (ret) {
			NFC_LOG_ERR("failed to get gpio ven\n");
			goto err_gpio_ven;
		}
		gpio_direction_output(pdata->ven, SEC_NFC_PW_OFF);
	} else {
		if (pdata->clkctrl_addr != 0) {
			info->clkctrl = ioremap_nocache(pdata->clkctrl_addr, 0x4);
			if (!info->clkctrl) {
				NFC_LOG_ERR("cannot remap register\n");
				ret = -ENXIO;
				goto err_iomap;
			}
			val |= (SEC_NFC_CLKCTRL_REQ_POLA | SEC_NFC_CLKCTRL_CLK_ENABLE);
			writel(val, info->clkctrl);
		}
	}

	if (pdata->firm)
	{
		ret = gpio_request(pdata->firm, "nfc_firm");
		if (ret) {
			NFC_LOG_ERR("probe() failed to get gpio firm\n");
			goto err_gpio_firm;
		}
		gpio_direction_output(pdata->firm, SEC_NFC_FW_OFF);
	}

	wake_lock_init(&info->nfc_wake_lock, WAKE_LOCK_SUSPEND, "nfc_wake_lock");

#ifdef FEATURE_SEC_NFC_TEST
	g_nfc_info = info;
	nfc_class = class_create(THIS_MODULE, "nfc_test");
	if (IS_ERR(&nfc_class))
		NFC_LOG_ERR("NFC: failed to create nfc_test class\n");
	else {
		ret = class_create_file(nfc_class, &class_attr_test);
		if (ret)
			NFC_LOG_ERR("NFC: failed to create attr_test\n");
	}
#endif
	nfc_class = class_create(THIS_MODULE, "nfc");
	if (IS_ERR(&nfc_class))
		NFC_LOG_ERR("NFC: failed to create nfc class\n");
	else {
		ret = class_create_file(nfc_class, &class_attr_nfc_support);
		if (ret)
			NFC_LOG_ERR("NFC: failed to create attr_nfc_support\n");
	}
	NFC_LOG_INFO("probe() success\n");

	return 0;

err_gpio_firm:
	if (prop)
	gpio_free(pdata->ven);
err_gpio_ven:
	if (!prop)
		iounmap(info->clkctrl);
err_iomap:
err_dev_reg:
	kfree(info);
err_info_alloc:
err_pdata:
	return ret;
}

static int __sec_nfc_remove(struct device *dev)
{
	struct sec_nfc_info *info = dev_get_drvdata(dev);
	struct sec_nfc_platform_data *pdata = info->pdata;
	struct device_node *np = dev->of_node;
	struct property *prop;

	NFC_LOG_DBG("remove\n");

	misc_deregister(&info->miscdev);
	sec_nfc_set_mode(info, SEC_NFC_MODE_OFF);
	gpio_set_value(pdata->firm, 0);
	prop = of_find_property(np, "sec-nfc,ven-gpio", NULL);
	if (prop)
	gpio_free(pdata->ven);
	else
		iounmap(info->clkctrl);
	if (pdata->firm) gpio_free(pdata->firm);
	wake_lock_destroy(&info->nfc_wake_lock);

	kfree(info);

	return 0;
}

#ifdef CONFIG_SEC_NFC_IF_I2C
MODULE_DEVICE_TABLE(i2c, sec_nfc_id_table);
typedef struct i2c_driver sec_nfc_driver_type;
#define SEC_NFC_INIT(driver)	i2c_add_driver(driver);
#define SEC_NFC_EXIT(driver)	i2c_del_driver(driver);

static int sec_nfc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;

	nfc_logger_init();
#ifdef CONFIG_ESE_SECURE
	ret = exynos_smc(0x83000032, 0 , 0, 0);
	if (ret == EBUSY) { 
		NFC_LOG_ERR("[NFC] eSE spi secure fail!\n");
		return -EBUSY;
	}
#endif
	ret = __sec_nfc_probe(&client->dev);
	if (ret)
		return ret;

	if (sec_nfc_i2c_probe(client))
		__sec_nfc_remove(&client->dev);

	return ret;
}

static int sec_nfc_remove(struct i2c_client *client)
{
	sec_nfc_i2c_remove(&client->dev);
	return __sec_nfc_remove(&client->dev);
}

static struct i2c_device_id sec_nfc_id_table[] = {
	{ SEC_NFC_DRIVER_NAME, 0 },
	{ }
};

#else	/* CONFIG_SEC_NFC_IF_I2C */
MODULE_DEVICE_TABLE(platform, sec_nfc_id_table);
typedef struct platform_driver sec_nfc_driver_type;
#define SEC_NFC_INIT(driver)	platform_driver_register(driver);
#define SEC_NFC_EXIT(driver)	platform_driver_unregister(driver);

static int sec_nfc_probe(struct platform_device *pdev)
{
	return __sec_nfc_probe(&pdev->dev);
}

static int sec_nfc_remove(struct platform_device *pdev)
{
	return __sec_nfc_remove(&pdev->dev);
}

static struct platform_device_id sec_nfc_id_table[] = {
	{ SEC_NFC_DRIVER_NAME, 0 },
	{ }
};

#endif /* CONFIG_SEC_NFC_IF_I2C */

#ifdef CONFIG_OF
static struct of_device_id nfc_match_table[] = {
	{ .compatible = SEC_NFC_DRIVER_NAME,},
	{},
};
#else
#define nfc_match_table NULL
#endif

static sec_nfc_driver_type sec_nfc_driver = {
	.probe = sec_nfc_probe,
	.id_table = sec_nfc_id_table,
	.remove = sec_nfc_remove,
	.driver = {
		.name = SEC_NFC_DRIVER_NAME,
#ifdef CONFIG_PM
		.pm = &sec_nfc_pm_ops,
#endif
		.of_match_table = nfc_match_table,
	},
};

static int __init sec_nfc_init(void)
{
	return SEC_NFC_INIT(&sec_nfc_driver);
}

static void __exit sec_nfc_exit(void)
{
	SEC_NFC_EXIT(&sec_nfc_driver);
}

module_init(sec_nfc_init);
module_exit(sec_nfc_exit);

MODULE_DESCRIPTION("Samsung sec_nfc driver");
MODULE_LICENSE("GPL");
