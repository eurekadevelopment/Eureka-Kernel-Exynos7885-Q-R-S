/*
 * haptic motor driver for s2mu004 - max77673_haptic.c
 *
 * Copyright (C) 2011 ByungChang Cha <bc.cha@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include "../staging/android/timed_output.h"
#include <linux/hrtimer.h>
#include <linux/pwm.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/s2mu004_haptic.h>
#include <linux/kthread.h>
//#include <plat/devs.h>
#include <linux/delay.h>
#include <linux/sec_sysfs.h>
#include <linux/power_supply.h>

#if defined(CONFIG_SSP_MOTOR_CALLBACK)
#include <linux/ssp_motorcallback.h>
#endif

#define TEST_MODE_TIME 10000
#define MAX_INTENSITY 10000

#define MOTOR_EN	(1<<0)
#define AUTO_CAL	(1<<2)
#define MOTOR_LRA	(1<<4)
#define MOTOR_ERM	(0<<4)
#define MOTOR_MODE	0x10
#define EXT_PWM		(1<<5)

#define ERM_MODE	0

struct s2mu004_haptic_data {
	struct s2mu004_dev *s2mu004;
	struct i2c_client *i2c;
	struct s2mu004_haptic_platform_data *pdata;
	u32 intensity;
	struct pwm_device *pwm;
	struct timed_output_dev tout_dev;
	struct hrtimer timer;
	unsigned int timeout;
	struct mutex mutex;
	struct kthread_worker kworker;
	struct kthread_work kwork;
	spinlock_t lock;
	bool running;
	int mot_con_type;
};

static int s2mu004_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret, i = 0;

	ret = i2c_smbus_write_byte_data(client, reg,  data);
	if (ret < 0) {
		for (i = 0; i < 3; i++) {
			ret = i2c_smbus_write_byte_data(client, reg,  data);
			if (ret >= 0)
				break;
		}

		if (i >= 3)
			dev_err(&client->dev, "%s: Error(%d)\n", __func__, ret);
	}

	return ret;
}

static int s2mu004_read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		pr_info("%s reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}
	ret &= 0xff;
	*data = ret;

	return 0;
}

static int s2mu004_update_reg(struct i2c_client *client, u8 reg, u8 data, u8 mask)
{
	int ret;
	u8 old_val, new_val;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret >= 0) {
		old_val = ret & 0xff;
		new_val = (data & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(client, reg, new_val);
	}
	return ret;
}

static void s2mu004_haptic_i2c(struct s2mu004_haptic_data *hap_data, bool en)
{
	int ret = 0;
	u8 temp = 0;

	pr_debug("[VIB] %s %d\n", __func__, en);

	if (en) {
		ret = s2mu004_update_reg(hap_data->i2c,
			S2MU004_REG_HAPTIC_MODE, MOTOR_EN, MOTOR_EN);
	} else {
		ret = s2mu004_update_reg(hap_data->i2c,
			S2MU004_REG_HAPTIC_MODE, 0, MOTOR_EN);
	}

	ret = s2mu004_read_reg(hap_data->i2c, S2MU004_REG_HAPTIC_MODE, &temp);

	if (ret)
		pr_err("[VIB] i2c MOTOR_EN_PWM update error %d\n", ret);
}

static int haptic_get_time(struct timed_output_dev *tout_dev)
{
	struct s2mu004_haptic_data *hap_data
		= container_of(tout_dev, struct s2mu004_haptic_data, tout_dev);

	struct hrtimer *timer = &hap_data->timer;
	if (hrtimer_active(timer)) {
		ktime_t remain = hrtimer_get_remaining(timer);
		struct timeval t = ktime_to_timeval(remain);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	}
	return 0;
}

static void haptic_enable(struct timed_output_dev *tout_dev, int value)
{
	struct s2mu004_haptic_data *hap_data
		= container_of(tout_dev, struct s2mu004_haptic_data, tout_dev);
	struct s2mu004_haptic_platform_data *pdata = hap_data->pdata;
	struct hrtimer *timer = &hap_data->timer;
	int ret;

	flush_kthread_worker(&hap_data->kworker);
	hrtimer_cancel(timer);

	value = min_t(int, value, (int)pdata->max_timeout);
	hap_data->timeout = value;
	hap_data->running = false;

	pr_info("[VIB] %s : %u ms\n", __func__, value);

	if (value > 0) {
		mutex_lock(&hap_data->mutex);

		hap_data->running = true;
		/* pwm set && mortor run */
		s2mu004_haptic_i2c(hap_data, true);
		ret = pwm_config(hap_data->pwm, hap_data->pdata->duty,
				hap_data->pdata->period);
		if (ret)
				pr_err("%s: pwm_config error to enable pwm %d\n", __func__, ret);

		ret = pwm_enable(hap_data->pwm);
		if (ret)
				pr_err("%s: pwm_enable error to enable pwm %d\n", __func__, ret);

#if defined(CONFIG_SSP_MOTOR_CALLBACK)
		setSensorCallback(true, value);
#endif
		mutex_unlock(&hap_data->mutex);
		hrtimer_start(timer, ns_to_ktime((u64)value * NSEC_PER_MSEC),
					HRTIMER_MODE_REL);
	} else {
		mutex_lock(&hap_data->mutex);
		
		hap_data->running = false;
		pwm_disable(hap_data->pwm);
		s2mu004_haptic_i2c(hap_data, false);

#if defined(CONFIG_SSP_MOTOR_CALLBACK)
		setSensorCallback(false, 0);
#endif
		mutex_unlock(&hap_data->mutex);
		pr_debug("[VIB] %s : off\n", __func__);
	}

}

static enum hrtimer_restart haptic_timer_func(struct hrtimer *timer)
{
	struct s2mu004_haptic_data *hap_data
		= container_of(timer, struct s2mu004_haptic_data, timer);
	pr_info("[VIB] : %s\n", __func__);

	hap_data->timeout = 0;
	queue_kthread_work(&hap_data->kworker, &hap_data->kwork);
	return HRTIMER_NORESTART;
}

static void haptic_work(struct kthread_work *work)
{
	struct s2mu004_haptic_data *hap_data
		= container_of(work, struct s2mu004_haptic_data, kwork);
	
	mutex_lock(&hap_data->mutex);
	pr_info("[VIB] %s : hap_data->running = %d\n", __func__, hap_data->running);

	if (!hap_data->running) {
		mutex_unlock(&hap_data->mutex);
		return;
	}
	hap_data->running = false;
	pwm_disable(hap_data->pwm);
	s2mu004_haptic_i2c(hap_data, false);

#if defined(CONFIG_SSP_MOTOR_CALLBACK)
	setSensorCallback(false,0);
#endif

	mutex_unlock(&hap_data->mutex);
	return;
}

static ssize_t intensity_store(struct device *dev,
        struct device_attribute *devattr, const char *buf, size_t count)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct s2mu004_haptic_data *haptic = container_of(tdev, struct s2mu004_haptic_data, tout_dev);

        int intensity = 0, ret = 0;
	u8 max = 0x7F;
	u8 value = 0x3F;

	if (haptic->pdata->max_duty)
		max = haptic->pdata->max_duty;

        ret = kstrtoint(buf, 0, &intensity);

        if (intensity < 0 || MAX_INTENSITY < intensity) {
		pr_err("out of rage\n");
		return -EINVAL;
	}

	if (MAX_INTENSITY == intensity)
		value = max;
	else if (0 != intensity) {
                long long tmp;
                tmp = max *  abs(intensity) / MAX_INTENSITY;
                value = (u8)tmp;
        }

	haptic->intensity = intensity;
	s2mu004_update_reg(haptic->i2c, S2MU004_REG_AMPCOEF1, value, 0x7F);
	pr_info("[VIB] %s, intensity = %d, setting data = 0x%2x\n", __func__, intensity, value);

        return count;
}

static ssize_t intensity_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct s2mu004_haptic_data *haptic = container_of(tdev, struct s2mu004_haptic_data, tout_dev);

        return sprintf(buf, "intensity: %u\n", haptic->intensity);
}

static DEVICE_ATTR(intensity, 0660, intensity_show, intensity_store);

static ssize_t vib_enable_store(struct device *dev,
        struct device_attribute *devattr, const char *buf, size_t count)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct s2mu004_haptic_data *hap_data = container_of(tdev, struct s2mu004_haptic_data, tout_dev);

        int enable = 0;
	int ret;

        ret = kstrtoint(buf, 0, &enable);

	if (enable == 1) {
		s2mu004_haptic_i2c(hap_data, true);
		pwm_config(hap_data->pwm, hap_data->pdata->duty,
				hap_data->pdata->period);
		pwm_enable(hap_data->pwm);
	} else if (enable == 0) {
		pwm_disable(hap_data->pwm);
		s2mu004_haptic_i2c(hap_data, false);
        } else {
		pr_err("out of rage\n");
		return -EINVAL;
	}

	pr_info("%s, VIB %s\n", __func__, ((enable == 1) ? "ENABLE" : "DISABLE") );

        return count;
}

static ssize_t vib_enable_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "echo 1 > vib_enable\necho 0 > vib_enable\n");
}

static DEVICE_ATTR(vib_enable, 0660, vib_enable_show, vib_enable_store);

#if defined(CONFIG_OF)
static int s2mu004_haptic_parse_dt(struct device *dev, struct s2mu004_haptic_data *haptic)
{
	struct device_node *np_haptic;
	struct device_node *np_root = dev->parent->of_node;
	struct device_node *np = dev->of_node;
	u32 temp;
	int ret;

	pr_info("%s : start dt parsing\n", __func__);

	np = of_find_node_by_name(np_root, 	"motor");
	
	if (np == NULL) {
		pr_err("%s : motor dts fail : error to get dt node\n", __func__);
		goto err_parsing_dt;
	}

	ret = of_property_read_u32(np, "motor,motor_type", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s : error to get dt node motor_type \n", __func__);
		goto err_parsing_dt;
	} else if (temp != IFMPIC_TYPE) {
			printk("[VIB] %s : IFPMIC use case : probe error return : motor type = %d\n",__func__, temp);
					goto err_parsing_dt;
	} else
		printk("[VIB] : %s : motor_type value %d \n", __func__, temp);
	haptic->mot_con_type = temp;

	np_haptic = of_find_node_by_name(np_root, "haptic");
	if (np_haptic == NULL) {
		pr_err("[VIB] %s : dts error to get haptic dt node\n", __func__);
		goto err_parsing_dt;
	}

	ret = of_property_read_u32(np_haptic, "haptic,max_timeout", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("[VIB] %s : error to get dt node max_timeout\n", __func__);
		goto err_parsing_dt;
	} else
		haptic->pdata->max_timeout = (u16)temp;

	ret = of_property_read_u32(np_haptic, "haptic,period", &temp);
	if (IS_ERR_VALUE(ret)) {
		haptic->pdata->period = 4878048;
	} else
		haptic->pdata->period = temp;

	ret = of_property_read_u32(np_haptic, "haptic,duty_rate", &temp);
	if (IS_ERR_VALUE(ret)) {
		haptic->pdata->duty = 2439024;
	} else
		haptic->pdata->duty = temp;

	ret = of_property_read_u32(np_haptic, "haptic,alert_duty", &temp);
	if (IS_ERR_VALUE(ret)) {
		haptic->pdata->max_duty = 0x5b;
	} else
		haptic->pdata->max_duty = (u16)temp;

	ret = of_property_read_u32(np_haptic, "haptic,pwm_id", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("[VIB] %s : error to get dt node pwm_id\n", __func__);
		goto err_parsing_dt;
	} else
		haptic->pdata->pwm_id = (u16)temp;

	/* debugging */
	pr_info("[VIB] %s : max_timeout = %d\n", __func__, haptic->pdata->max_timeout);
	pr_info("[VIB] %s : duty = %d\n", __func__, haptic->pdata->duty);
	pr_info("[VIB] %s : period = %d\n", __func__, haptic->pdata->period);
	pr_info("[VIB] %s : max_duty = 0x%x\n", __func__, haptic->pdata->max_duty);
	pr_info("[VIB] %s : pwm_id = %d\n", __func__, haptic->pdata->pwm_id);

	haptic->pdata->init_hw = NULL;
	haptic->pdata->motor_en = NULL;

	return 0;

err_parsing_dt:
	return -1;
}
#endif

static struct of_device_id s2mu004_haptic_match_table[] = {
	{ .compatible = "sec,s2mu004-haptic",},
	{},
};

static int s2mu004_haptic_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int error = 0, ret = 0;
	struct s2mu004_haptic_data *haptic;
	struct task_struct *kworker_task;
	u8 temp = 0;


	haptic = kzalloc(sizeof(struct s2mu004_haptic_data), GFP_KERNEL);
	pr_err("[VIB] %d probe  %s\n",__LINE__,  __func__);

	if (!haptic) {
		pr_err("[VIB] %s: no hap_pdata\n", __func__);
		//kfree(pdata);
		return -ENOMEM;
	}

	haptic->i2c = client;

	if (client->dev.of_node) {
		haptic->pdata = devm_kzalloc(&client->dev,
			sizeof(*(haptic->pdata)), GFP_KERNEL);
		if (!haptic->pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			error = -ENOMEM;
			goto err_kthread;
		}
		ret = s2mu004_haptic_parse_dt(&client->dev, haptic);
		if (ret < 0) {
			error = -EFAULT;
			goto err_kthread;
		}
	} else {
		haptic->pdata = client->dev.platform_data;
	}

	i2c_set_clientdata(client, haptic);
	
	init_kthread_worker(&haptic->kworker);
	kworker_task = kthread_run(kthread_worker_fn,
		   &haptic->kworker, "s2mu004_haptic");

	if (IS_ERR(kworker_task)) {
		pr_err("Failed to create message pump task\n");
		error = -ENOMEM;
		goto err_kthread;
	}
	init_kthread_work(&haptic->kwork, haptic_work);

	spin_lock_init(&(haptic->lock));
	mutex_init(&haptic->mutex);

	haptic->pwm = pwm_request(haptic->pdata->pwm_id, "vibrator");

	if (IS_ERR(haptic->pwm)) {
		error = -EFAULT;
		pr_err("[VIB] Failed to request pwm, err num: %d\n", error);
		goto err_pwm_request;
	}
	pr_err("[VIB] request pwm, success: \n");

	pwm_config(haptic->pwm, haptic->pdata->duty, haptic->pdata->period);

	/* hrtimer init */
	hrtimer_init(&haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	haptic->timer.function = haptic_timer_func;

	/* timed_output_dev init*/
	haptic->tout_dev.name = "vibrator";
	haptic->tout_dev.get_time = haptic_get_time;
	haptic->tout_dev.enable = haptic_enable;

	haptic->intensity = 50;
	error = timed_output_dev_register(&haptic->tout_dev);
	if (error < 0) {
		error = -EFAULT;
		pr_err("[VIB] Failed to register timed_output : %d\n", error);
		goto err_timed_output_register;
	}

	ret = sysfs_create_file(&haptic->tout_dev.dev->kobj,
				&dev_attr_intensity.attr);
	ret = sysfs_create_file(&haptic->tout_dev.dev->kobj,
				&dev_attr_vib_enable.attr);
	if (ret < 0) {
		pr_err("Failed to register sysfs : %d\n", ret);
		goto err_timed_output_register;
	}
	/* set Motor enable stablity */
	s2mu004_update_reg(haptic->i2c, 0x9B, 0x03, 0x03);
	error = s2mu004_write_reg(haptic->i2c, S2MU004_REG_HAPTIC_MODE, 0x20);
	if (error < 0) {
		pr_err("[VIB] %s Failed to write reg to MODE.\n",
			__func__);
	}
	else {
		ret = s2mu004_read_reg(haptic->i2c, S2MU004_REG_HAPTIC_MODE, &temp);
		printk("[VIB] %s : S2MU004_REG_HAPTIC_MODE[0x%x] = 0x%x \n",__func__, S2MU004_REG_HAPTIC_MODE, temp);
	}
	s2mu004_write_reg(haptic->i2c, S2MU004_REG_SINECOEF1, 0x7B);
	s2mu004_write_reg(haptic->i2c, S2MU004_REG_SINECOEF2, 0xB3);
	s2mu004_write_reg(haptic->i2c, S2MU004_REG_SINECOEF3, 0x35);
	s2mu004_update_reg(haptic->i2c, S2MU004_REG_AMPCOEF1, 0x3F, 0x7F);

	s2mu004_write_reg(haptic->i2c, S2MU004_REG_PERI_TAR1, 0x00);
	s2mu004_write_reg(haptic->i2c, S2MU004_REG_PERI_TAR2, 0x00);
	s2mu004_write_reg(haptic->i2c, S2MU004_REG_DUTY_TAR1, 0x00);
	s2mu004_write_reg(haptic->i2c, S2MU004_REG_DUTY_TAR2, 0x01);

	return error;

err_timed_output_register:
	pwm_free(haptic->pwm);
err_pwm_request:
err_kthread:
	kfree(haptic);
	return error;
}

static int s2mu004_haptic_remove(struct i2c_client *client)
{
	struct s2mu004_haptic_data *haptic = i2c_get_clientdata(client);
	timed_output_dev_unregister(&haptic->tout_dev);
	pwm_free(haptic->pwm);
	s2mu004_haptic_i2c(haptic, false);
	kfree(haptic);
	return 0;
}

static int s2mu004_haptic_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct s2mu004_haptic_data *haptic = i2c_get_clientdata(client);

	pr_debug("[VIB] %s\n", __func__);
	flush_kthread_worker(&haptic->kworker);
	hrtimer_cancel(&haptic->timer);
	s2mu004_haptic_i2c(haptic, false);
	return 0;
}
static int s2mu004_haptic_resume(struct device *dev)
{
	return 0;
}


static SIMPLE_DEV_PM_OPS(s2mu004_haptic_pm_ops, s2mu004_haptic_suspend, s2mu004_haptic_resume);
static const struct i2c_device_id s2mu004_haptic_id[] = {
	{"s2mu004-haptic", 0},
	{}
};
static struct i2c_driver s2mu004_haptic_driver = {
	.driver = {
		.name	= "s2mu004-haptic",
		.owner	= THIS_MODULE,
		.pm	= &s2mu004_haptic_pm_ops,
		.of_match_table = s2mu004_haptic_match_table,
	},
	.probe		= s2mu004_haptic_probe,
	.remove		= s2mu004_haptic_remove,
	.id_table	= s2mu004_haptic_id,
};

static int __init s2mu004_haptic_init(void)
{
	pr_info("[VIB] %s\n", __func__);
	return i2c_add_driver(&s2mu004_haptic_driver);
}
late_initcall(s2mu004_haptic_init);

static void __exit s2mu004_haptic_exit(void)
{
	i2c_del_driver(&s2mu004_haptic_driver);
}
module_exit(s2mu004_haptic_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("s2mu004 haptic driver");
