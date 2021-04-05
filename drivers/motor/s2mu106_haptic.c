/*
 * haptic motor driver for s2mu106 - s2mu106_haptic.c
 *
 * Copyright (C) 2018 Suji Lee <suji0908.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "[VIB] " fmt

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
#include <linux/s2mu106_haptic.h>
#include <linux/kthread.h>
#include <linux/mfd/samsung/s2mu106.h>
#include <linux/delay.h>
#include <linux/sec_sysfs.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>

#if defined(CONFIG_SSP_MOTOR_CALLBACK)
#include <linux/ssp_motorcallback.h>
#endif

#define TEST_MODE_TIME 10000
#define MAX_INTENSITY 100

struct s2mu106_haptic_data {
	struct s2mu106_dev *s2mu106;
	struct i2c_client *i2c;
	struct s2mu106_haptic_platform_data *pdata;
	struct device *dev;

	enum s2mu106_haptic_operation_type hap_mode;
	u32 intensity;
	int motor_en;
	struct pwm_device *pwm;
	struct mutex mutex;
	spinlock_t lock;

	bool running;

	struct timed_output_dev tout_dev;
	struct hrtimer timer;
	unsigned int timeout;

	struct kthread_worker kworker;
	struct kthread_work kwork;
};

static void s2mu106_set_boost_voltage(struct s2mu106_haptic_data *haptic, int voltage)
{
	u8 data;
	if (voltage <= 3150)
		data = 0x00;
	else if (voltage > 3150 && voltage <= 6300)
		data = (voltage - 3150) / 50;
	else
		data = 0xFF;
	pr_info("%s: boost voltage %d, 0x%02x\n", __func__, voltage, data);

	s2mu106_update_reg(haptic->i2c, S2MU106_REG_HBST_CTRL1,
				data, HAPTIC_BOOST_VOLTAGE_MASK);
}

static void s2mu106_set_intensity(struct s2mu106_haptic_data *haptic, int intensity)
{
	int data = 0x3FFFF;
	int max = 0x7FFFF;
	u8 val1, val2, val3;

	if (intensity == MAX_INTENSITY)
		data = max;
	else if (intensity != 0) {
		long long tmp;

		tmp = (intensity * max) / 100;
		data = (int)tmp;
	} else
		data = 0;
	data = (data * haptic->pdata->intensity) / 100;	
	data &= 0x7FFFF;
	val1 = data & 0x0000F;
	val2 = (data & 0x00FF0) >> 4;
	val3 = (data & 0x7F000) >> 12;

	s2mu106_update_reg(haptic->i2c, S2MU106_REG_AMPCOEF1, val3, 0x7F);
	s2mu106_write_reg(haptic->i2c, S2MU106_REG_AMPCOEF2, val2);
	s2mu106_update_reg(haptic->i2c, S2MU106_REG_AMPCOEF3, val1 << 4, 0xF0);

	pr_info("%s, intensity = %d, coef1 = 0x%2x, coef2 = 0x%2x, coef3 = 0x%2x\n",
				__func__, intensity, val3, val2, val1);
}

static void s2mu106_haptic_onoff(struct s2mu106_haptic_data *haptic, bool en)
{
	pr_info("%s intensity = %d, %d\n", __func__, haptic->intensity, en);

	if (en) {
		if (haptic->running)
			return;
		haptic->running = true;
		pr_info("Motor Enable\n");

		switch (haptic->hap_mode) {
		case S2MU106_HAPTIC_LRA:
			s2mu106_write_reg(haptic->i2c, S2MU106_REG_HAPTIC_MODE, LRA_MODE_EN);
			pwm_config(haptic->pwm, haptic->pdata->duty,
					haptic->pdata->period);
			pwm_enable(haptic->pwm);
			break;
		case S2MU106_HAPTIC_ERM_GPIO:
			if (gpio_is_valid(haptic->motor_en))
				gpio_direction_output(haptic->motor_en, 1);
			break;
		case S2MU106_HAPTIC_ERM_I2C:
			s2mu106_write_reg(haptic->i2c, S2MU106_REG_HAPTIC_MODE, ERM_MODE_ON);
			break;
		default:
			break;
		}
	} else {
		if (!haptic->running)
			return;
		haptic->running = false;
		pr_info("Motor Disable\n");

		switch (haptic->hap_mode) {
		case S2MU106_HAPTIC_LRA:
			pwm_disable(haptic->pwm);
			s2mu106_write_reg(haptic->i2c, S2MU106_REG_HAPTIC_MODE, HAPTIC_MODE_OFF);
			break;
		case S2MU106_HAPTIC_ERM_GPIO:
			if (gpio_is_valid(haptic->motor_en))
				gpio_direction_output(haptic->motor_en, 0);
			break;
		case S2MU106_HAPTIC_ERM_I2C:
			s2mu106_write_reg(haptic->i2c, S2MU106_REG_HAPTIC_MODE, HAPTIC_MODE_OFF);
			break;
		default:
			break;
		}
	}
}

static int haptic_get_time(struct timed_output_dev *tout_dev)
{
	struct s2mu106_haptic_data *hap_data
		= container_of(tout_dev, struct s2mu106_haptic_data, tout_dev);

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
	struct s2mu106_haptic_data *hap_data
		= container_of(tout_dev, struct s2mu106_haptic_data, tout_dev);
	struct s2mu106_haptic_platform_data *pdata = hap_data->pdata;
	struct hrtimer *timer = &hap_data->timer;

	flush_kthread_worker(&hap_data->kworker);
	hrtimer_cancel(timer);

	value = min_t(int, value, (int)pdata->max_timeout);
	hap_data->timeout = value;

	pr_info("%s : %u ms\n", __func__, value);

	if (value > 0) {
		mutex_lock(&hap_data->mutex);
		/* motor run */
		hap_data->running = false;
		s2mu106_haptic_onoff(hap_data, true);

#if defined(CONFIG_SSP_MOTOR_CALLBACK)
		setSensorCallback(true, value);
#endif
		mutex_unlock(&hap_data->mutex);
		hrtimer_start(timer, ns_to_ktime((u64)value * NSEC_PER_MSEC),
					HRTIMER_MODE_REL);
	} else {
		mutex_lock(&hap_data->mutex);
		/* motor stop */
		s2mu106_haptic_onoff(hap_data, false);

#if defined(CONFIG_SSP_MOTOR_CALLBACK)
		setSensorCallback(false, 0);
#endif
		mutex_unlock(&hap_data->mutex);
		pr_debug("%s : off\n", __func__);
	}

}

static enum hrtimer_restart haptic_timer_func(struct hrtimer *timer)
{
	struct s2mu106_haptic_data *hap_data
		= container_of(timer, struct s2mu106_haptic_data, timer);
	pr_info("%s\n", __func__);

	hap_data->timeout = 0;
	queue_kthread_work(&hap_data->kworker, &hap_data->kwork);
	return HRTIMER_NORESTART;
}

static void haptic_work(struct kthread_work *work)
{
	struct s2mu106_haptic_data *hap_data
		= container_of(work, struct s2mu106_haptic_data, kwork);
	
	mutex_lock(&hap_data->mutex);
	pr_info("%s : hap_data->running = %d\n", __func__, hap_data->running);

	if (!hap_data->running) {
		mutex_unlock(&hap_data->mutex);
		return;
	}
	s2mu106_haptic_onoff(hap_data, false);

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
	struct s2mu106_haptic_data *haptic = container_of(tdev, struct s2mu106_haptic_data, tout_dev);

        int intensity = 0, ret = 0;

        ret = kstrtoint(buf, 0, &intensity);
		intensity = intensity / 100;
        if (intensity < 0 || MAX_INTENSITY < intensity) {
		pr_err("out of rage\n");
		return -EINVAL;
	}

	haptic->intensity = intensity;
	s2mu106_set_intensity(haptic, haptic->intensity);

	pr_debug("%s, intensity = %d\n", __func__, intensity);

        return count;
}

static ssize_t intensity_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
        struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct s2mu106_haptic_data *haptic = container_of(tdev, struct s2mu106_haptic_data, tout_dev);

        return sprintf(buf, "intensity: %u\n", haptic->intensity);
}

static DEVICE_ATTR(intensity, 0660, intensity_show, intensity_store);

static ssize_t vib_enable_store(struct device *dev,
        struct device_attribute *devattr, const char *buf, size_t count)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct s2mu106_haptic_data *hap_data = container_of(tdev, struct s2mu106_haptic_data, tout_dev);

        int enable = 0;
	int ret;
	
        ret = kstrtoint(buf, 0, &enable);
		
	if (enable == 1)
		s2mu106_haptic_onoff(hap_data, true);
	else if (enable == 0)
		s2mu106_haptic_onoff(hap_data, false);
        else {
		s2mu106_haptic_onoff(hap_data, false);
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

static ssize_t motor_type_show(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
    struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct s2mu106_haptic_data *haptic = container_of(tdev, struct s2mu106_haptic_data, tout_dev);

        return sprintf(buf, "%s\n", haptic->pdata->vib_type);
}

DEVICE_ATTR(motor_type, 0660, motor_type_show, NULL);

#if defined(CONFIG_OF)
static int s2mu106_haptic_parse_dt(struct device *dev,
			struct s2mu106_haptic_platform_data *pdata)
{
	struct device_node *np = of_find_node_by_name(NULL, "s2mu106-haptic");
	u32 temp;
	int ret;
	const char *type;

	pr_info("%s : start dt parsing\n", __func__);

	if (np == NULL) {
		pr_err("%s : error to get dt node\n", __func__);
		goto err_parsing_dt;
	}
	/* 30.08kHz 99% duty */
	ret = of_property_read_u32(np, "haptic,duty", &temp);
	if (ret < 0)
		pdata->duty = 32911;
	else
		pdata->duty = temp;

	ret = of_property_read_u32(np, "haptic,period", &temp);
	if (ret < 0)
		pdata->period = 33244;
	else
		pdata->period = temp;
	ret = of_property_read_u32(np, "haptic,pwm_id", &temp);
	if (ret < 0) {
		pr_err("%s : haptic motor ERM_GPIO mode\n", __func__);
	} else
		pdata->pwm_id = (u16)temp;

	ret = of_property_read_u32(np, "haptic,intensity", &temp);
	if (ret < 0) {
		pr_info("%s : intensity set to 100%%\n", __func__);
		pdata->intensity = 100;
	} else {
		pr_info("%s : intensity set to %d%%\n", __func__,temp);
		pdata->intensity = (u32)temp;
	}

	/*	Haptic operation mode
		0 : S2MU106_HAPTIC_ERM_I2C
		1 : S2MU106_HAPTIC_ERM_GPIO
		2 : S2MU106_HAPTIC_LRA
		default : S2MU106_HAPTIC_ERM_GPIO
	*/
	pdata->hap_mode = 1;
	ret = of_property_read_u32(np, "haptic,operation_mode", &temp);
	if (ret < 0) {
		pr_err("%s : eror to get operation mode\n", __func__);
		goto err_parsing_dt;
	} else
		pdata->hap_mode = temp;

	ret = pdata->motor_en = of_get_named_gpio(np, "haptic,motor_en", 0);
	if (ret < 0) {
		pr_err("%s : can't get motor_en\n", __func__);
	}
	
	ret = of_property_read_u32(np, "haptic,max_timeout", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s : error to get dt node max_timeout\n", __func__);
	} else
		pdata->max_timeout = (u16)temp;
	
	ret = of_property_read_string(np,
				"haptic,type", &type);
	if (ret) {
		pr_err("%s : error to get dt node type\n", __func__);
		pdata->vib_type = NULL;
	} else
		pdata->vib_type = type;

	/* Haptic boost setting */
	pdata->hbst.en = (of_find_property(np, "haptic,hbst_en", NULL)) ? true : false;

	pdata->hbst.automode =
		(of_find_property(np, "haptic,hbst_automode", NULL)) ? true : false;

	ret = of_property_read_u32(np, "haptic,boost_level", &temp);
	if (ret < 0)
		pdata->hbst.level = 5500;
	else
		pdata->hbst.level = temp;

	/* parsing info */
	pr_info("%s :operation_mode = %d, HBST_EN %s, HBST_AUTO_MODE %s\n", __func__,
			pdata->hap_mode,
			pdata->hbst.en ? "enabled" : "disabled",
			pdata->hbst.automode ? "enabled" : "disabled");
	pdata->init_hw = NULL;

	return 0;

err_parsing_dt:
	return -1;
}
#endif

static void s2mu106_haptic_initial(struct s2mu106_haptic_data *haptic)
{
	u8 data;

	haptic->hap_mode = haptic->pdata->hap_mode;

	/* Haptic Boost initial setting */
	if (haptic->pdata->hbst.en){
		pr_info("%s : Haptic Boost Enable - Auto mode(%s)\n", __func__,
				haptic->pdata->hbst.automode ? "enabled" : "disabled");
		/* Boost voltage level setting
			default : 5.5V */
		s2mu106_set_boost_voltage(haptic, haptic->pdata->hbst.level);

		if (haptic->pdata->hbst.automode) {
			s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP0,
						HBST_OK_MASK_EN, HBST_OK_MASK_EN);
			s2mu106_update_reg(haptic->i2c,	S2MU106_REG_HBST_CTRL0,
						0, SEL_HBST_HAPTIC_MASK);
		} else {
			s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP0,
						0, HBST_OK_MASK_EN);
			s2mu106_update_reg(haptic->i2c,	S2MU106_REG_HBST_CTRL0,
						SEL_HBST_HAPTIC_MASK, SEL_HBST_HAPTIC_MASK);
		}
	} else {
		pr_info("%s : HDVIN - Vsys HDVIN voltage : Min 3.5V\n", __func__);
#if IS_ENABLED(CONFIG_MOTOR_VOLTAGE_3P3)
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP2, 0x40, VCEN_SEL_MASK);
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP3, 0x01, VCENUP_TRIM_MASK);
#else
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP2, 0x00, VCEN_SEL_MASK);
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_HT_OTP3, 0x03, VCENUP_TRIM_MASK);
#endif
	}
	
	/* Intensity setting */
	s2mu106_set_intensity(haptic, haptic->intensity);
	haptic->running = false;

	/* mode setting */
	switch (haptic->hap_mode) {
	case S2MU106_HAPTIC_LRA:
		data = HAPTIC_MODE_OFF;
		pwm_config(haptic->pwm, haptic->pdata->duty,
				haptic->pdata->period);
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_OV_BK_OPTION,
					LRA_MODE_SET_MASK, LRA_MODE_SET_MASK);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_FILTERCOEF1, 0x7F);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_FILTERCOEF2, 0x5A);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_FILTERCOEF3, 0x02);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_PWM_CNT_NUM, 0x40);
		s2mu106_update_reg(haptic->i2c, S2MU106_REG_OV_WAVE_NUM, 0xF0, 0xF0);
		break;
	case S2MU106_HAPTIC_ERM_GPIO:
		data = ERM_HDPWM_MODE_EN;
		if (gpio_is_valid(haptic->motor_en)) {
			pr_info("%s : MOTOR_EN enable\n", __func__);
			haptic->motor_en = haptic->pdata->motor_en;
			gpio_request_one(haptic->motor_en, GPIOF_OUT_INIT_LOW, "MOTOR_EN");
			gpio_free(haptic->motor_en);
		}
		break;
	case S2MU106_HAPTIC_ERM_I2C:
		data = HAPTIC_MODE_OFF;
		break;
	default:
		data = ERM_HDPWM_MODE_EN;
		break;
	}
	s2mu106_write_reg(haptic->i2c, S2MU106_REG_HAPTIC_MODE, data);

	if (haptic->hap_mode == S2MU106_HAPTIC_ERM_I2C ||
		haptic->hap_mode == S2MU106_HAPTIC_ERM_GPIO) {
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_PERI_TAR1, 0x00);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_PERI_TAR2, 0x00);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_DUTY_TAR1, 0x00);
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_DUTY_TAR2, 0x00);
#if IS_ENABLED(CONFIG_MOTOR_VOLTAGE_3P3)
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_AMPCOEF1, 0x5D);
#else
		s2mu106_write_reg(haptic->i2c, S2MU106_REG_AMPCOEF1, 0x74);
#endif
	}

	pr_info("%s, haptic operation mode = %d\n", __func__, haptic->hap_mode);
}

static struct of_device_id s2mu106_haptic_match_table[] = {
	{ .compatible = "sec,s2mu106-haptic",},
	{},
};

static int s2mu106_haptic_probe(struct platform_device *pdev)
{
	struct s2mu106_dev *s2mu106 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu106_haptic_data *haptic;
	struct task_struct *kworker_task;
	int ret = 0;
	int error = 0;

	pr_info("%s Start\n", __func__);
	haptic = devm_kzalloc(&pdev->dev,
			sizeof(struct s2mu106_haptic_data), GFP_KERNEL);

	if (!haptic) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	haptic->dev = &pdev->dev;
	haptic->i2c = s2mu106->haptic;

	haptic->pdata = devm_kzalloc(&pdev->dev, sizeof(*(haptic->pdata)), GFP_KERNEL);
	if (!haptic->pdata) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_kthread;
	}

	ret = s2mu106_haptic_parse_dt(&pdev->dev, haptic->pdata);
	if (ret < 0)
		goto err_kthread;

	platform_set_drvdata(pdev, haptic);

	init_kthread_worker(&haptic->kworker);
	kworker_task = kthread_run(kthread_worker_fn, &haptic->kworker, "s2mu106_haptic");
	if (IS_ERR(kworker_task)) {
		pr_err("Failed to create message pump task\n");
		error = -ENOMEM;
		goto err_kthread;
	}

	init_kthread_work(&haptic->kwork, haptic_work);
	spin_lock_init(&(haptic->lock));
	mutex_init(&haptic->mutex);

	if (haptic->pdata->hap_mode == S2MU106_HAPTIC_LRA) {
		haptic->pwm = pwm_request(haptic->pdata->pwm_id, "vibrator");
		if (IS_ERR(haptic->pwm)) {
			error = -EFAULT;
			pr_err("Failed to request pwm, err num: %d\n", error);
			goto err_pwm_request;
		}
		pr_err("request pwm, success: \n");
		pwm_config(haptic->pwm, haptic->pdata->duty, haptic->pdata->period);
	}

	/* hrtimer init */
	hrtimer_init(&haptic->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	haptic->timer.function = haptic_timer_func;

	/* timed_output_dev init*/
	haptic->tout_dev.name = "vibrator";
	haptic->tout_dev.get_time = haptic_get_time;
	haptic->tout_dev.enable = haptic_enable;

	haptic->intensity = 100;
	error = timed_output_dev_register(&haptic->tout_dev);
	if (error < 0) {
		error = -EFAULT;
		pr_err("Failed to register timed_output : %d\n", error);
		goto err_timed_output_register;
	}
	if(haptic->pdata->hap_mode == S2MU106_HAPTIC_LRA) {
	ret = sysfs_create_file(&haptic->tout_dev.dev->kobj,
				&dev_attr_intensity.attr);
	}
	ret = sysfs_create_file(&haptic->tout_dev.dev->kobj,
				&dev_attr_vib_enable.attr);
	ret = sysfs_create_file(&haptic->tout_dev.dev->kobj,
				&dev_attr_motor_type.attr);
	if (ret < 0) {
		pr_err("Failed to register sysfs : %d\n", ret);
		goto err_timed_output_register;
	}

	s2mu106_haptic_initial(haptic);

	return error;

err_timed_output_register:
	if (haptic->pdata->hap_mode == S2MU106_HAPTIC_LRA)
		pwm_free(haptic->pwm);
err_pwm_request:
err_kthread:
	return error;
}

static int s2mu106_haptic_remove(struct platform_device *pdev)
{
	struct s2mu106_haptic_data *haptic = platform_get_drvdata(pdev);

	timed_output_dev_unregister(&haptic->tout_dev);
	if (haptic->hap_mode == S2MU106_HAPTIC_LRA)
		pwm_free(haptic->pwm);
	s2mu106_haptic_onoff(haptic, false);
	return 0;
}

static int s2mu106_haptic_suspend(struct device *dev)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct s2mu106_haptic_data *haptic = platform_get_drvdata(pdev);

	pr_info("%s\n", __func__);
	flush_kthread_worker(&haptic->kworker);
	hrtimer_cancel(&haptic->timer);
	s2mu106_haptic_onoff(haptic, false);
	return 0;
}

static int s2mu106_haptic_resume(struct device *dev)
{
	pr_info("%s\n", __func__);
	return 0;
}


static SIMPLE_DEV_PM_OPS(s2mu106_haptic_pm_ops, s2mu106_haptic_suspend, s2mu106_haptic_resume);
static struct platform_driver s2mu106_haptic_driver = {
	.driver = {
		.name	= "s2mu106-haptic",
		.owner	= THIS_MODULE,
		.pm	= &s2mu106_haptic_pm_ops,
		.of_match_table = s2mu106_haptic_match_table,
	},
	.probe		= s2mu106_haptic_probe,
	.remove		= s2mu106_haptic_remove,
};

static int __init s2mu106_haptic_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&s2mu106_haptic_driver);
}
late_initcall(s2mu106_haptic_init);

static void __exit s2mu106_haptic_exit(void)
{
	platform_driver_unregister(&s2mu106_haptic_driver);
}
module_exit(s2mu106_haptic_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("s2mu106 haptic driver");
