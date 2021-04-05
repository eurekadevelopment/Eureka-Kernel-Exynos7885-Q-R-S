/*
 *
 * Copyright 2017 SEC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)	"flip_cover :" fmt

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#if IS_ENABLED(CONFIG_SEC_SYSFS)
#include <linux/sec_sysfs.h>
#endif
#if IS_ENABLED(CONFIG_SEC_FACTORY)
#include <linux/delay.h>
#endif
#include <linux/sec_class.h>

#if defined(CONFIG_HALL_NEW_NODE)
struct device *hall_ic;
EXPORT_SYMBOL(hall_ic);
#endif

struct flip_cover_hall_data {
	struct delayed_work dwork;
	struct wake_lock wlock;
	struct input_dev *input;
	int gpio;
	int irq;
	int state;
	int active_low;
	unsigned int event;
	const char *name;
};

struct flip_cover_pdata {
	struct flip_cover_hall_data *hall;
	unsigned int nhalls;
};

struct flip_cover_drvdata {
	struct flip_cover_pdata *pdata;
	struct input_dev *input;
	struct work_struct work;
	struct device *dev;
};

#if IS_ENABLED(CONFIG_SEC_SYSFS)
struct flip_cover_drvdata *gddata;
static ssize_t flip_cover_detect_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_HALL_EVENT_REVERSE
	if (test_bit(SW_FLIP, gddata->input->sw))
		sprintf(buf, "CLOSE\n");
	else
		sprintf(buf, "OPEN\n");
#else
	if (test_bit(SW_FLIP, gddata->input->sw))
		sprintf(buf, "OPEN\n");
	else
		sprintf(buf, "CLOSE\n");
#endif
	return strlen(buf);
}
static ssize_t certify_hall_detect_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	if (!test_bit(SW_CERTIFYHALL, gddata->input->sw))
		sprintf(buf, "OPEN\n");
	else
		sprintf(buf, "CLOSE\n");
	return strlen(buf);
}
static DEVICE_ATTR(hall_detect, 0664, flip_cover_detect_show, NULL);
static DEVICE_ATTR(certify_hall_detect, 0664, certify_hall_detect_show, NULL);

static struct attribute *flip_cover_attrs[] = {
	&dev_attr_hall_detect.attr,
	&dev_attr_certify_hall_detect.attr,
	NULL,
};

static struct attribute_group flip_cover_attrs_group = {
	.attrs = flip_cover_attrs,
};

static struct attribute *flip_cover_attr[] = {
	&dev_attr_hall_detect.attr,
	NULL,
};

static struct attribute_group flip_cover_attr_group = {
	.attrs = flip_cover_attr,
};

static struct attribute *certify_cover_attr[] = {
	&dev_attr_certify_hall_detect.attr,
	NULL,
};

static struct attribute_group certify_cover_attr_group = {
	.attrs = certify_cover_attr,
};

#endif

#if IS_ENABLED(CONFIG_SEC_FACTORY)
static void flip_cover_work(struct work_struct *work)
{
	struct flip_cover_hall_data *hall = container_of(work,
		struct flip_cover_hall_data, dwork.work);
	int first, second, state;

	first = gpio_get_value_cansleep(hall->gpio);
	msleep(50);
	second = gpio_get_value_cansleep(hall->gpio);
	if (first == second) {
		hall->state = first;
		state = first ^ hall->active_low;
		pr_info("%s %s\n", hall->name,
			hall->state ? "open" : "close");
#ifdef CONFIG_HALL_EVENT_REVERSE
		if (!strncmp(hall->name, "hall", 4))
			input_report_switch(hall->input, hall->event, !state);
		else
			input_report_switch(hall->input, hall->event, state);
#else
		input_report_switch(hall->input, hall->event, state);
#endif
		input_sync(hall->input);
	} else
		pr_info("%s %d,%d\n", hall->name,
			first, second);
}
#else
static void flip_cover_work(struct work_struct *work)
{
	struct flip_cover_hall_data *hall = container_of(work,
		struct flip_cover_hall_data, dwork.work);
	int state;

	hall->state = !!gpio_get_value_cansleep(hall->gpio);
	state = hall->state ^ hall->active_low;
	pr_info("%s %s\n", hall->name,
		hall->state ? "open" : "close");
#ifdef CONFIG_HALL_EVENT_REVERSE
	if (!strncmp(hall->name, "hall", 4))
		input_report_switch(hall->input, hall->event, !state);
	else
		input_report_switch(hall->input, hall->event, state);
#else
	input_report_switch(hall->input, hall->event, state);
#endif
	input_sync(hall->input);
}
#endif

static irqreturn_t flip_cover_detect(int irq, void *dev_id)
{
	struct flip_cover_hall_data *hall = dev_id;
	int state = !!gpio_get_value_cansleep(hall->gpio);

	pr_info("%s : %d\n", hall->name, state);
	cancel_delayed_work_sync(&hall->dwork);
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	schedule_delayed_work(&hall->dwork, HZ / 20);
#else
	if (state)	{
		wake_lock_timeout(&hall->wlock, HZ / 20);
		schedule_delayed_work(&hall->dwork, HZ  / 100);
	} else {
		wake_unlock(&hall->wlock);
		schedule_delayed_work(&hall->dwork, 0);
	}
#endif
	return IRQ_HANDLED;
}

static int flip_cover_open(struct input_dev *input)
{
	struct flip_cover_drvdata *ddata = input_get_drvdata(input);
	int i = 0;

	for (i = 0; i < ddata->pdata->nhalls; i++) {
		struct flip_cover_hall_data *hall = &ddata->pdata->hall[i];

		schedule_delayed_work(&hall->dwork, HZ / 2);
	}
	input_sync(input);
	return 0;
}

static void flip_cover_close(struct input_dev *input)
{
}

static int flip_cover_setup_halls(struct flip_cover_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	int i = 0;
	int ret = 0;

	for (i = 0; i < ddata->pdata->nhalls; i++) {
		struct flip_cover_hall_data *hall = &ddata->pdata->hall[i];

		hall->input = input;
		hall->state = gpio_get_value_cansleep(hall->gpio);
		input_set_capability(input, EV_SW, hall->event);
		wake_lock_init(&hall->wlock, WAKE_LOCK_SUSPEND,
						"flip_cover_wlock");
		INIT_DELAYED_WORK(&hall->dwork, flip_cover_work);
		ret = request_threaded_irq(hall->irq, NULL, flip_cover_detect,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			hall->name, hall);
		if (ret < 0)
			pr_err("failed to request irq %d(%d)\n",
							hall->irq, ret);
	}

	return ret;
}

static struct flip_cover_pdata *flip_cover_parsing_dt(struct device *dev)
{
	struct device_node *node = dev->of_node, *pp;
	struct flip_cover_pdata *pdata;
	struct flip_cover_hall_data *hall;
	int nhalls, ret, i = 0;

	if (!node)
		return ERR_PTR(-ENODEV);
	nhalls = of_get_child_count(node);
	if (nhalls == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);
	pdata->hall = devm_kzalloc(dev, nhalls * sizeof(*hall), GFP_KERNEL);
	if (!pdata->hall)
		return ERR_PTR(-ENOMEM);
	pdata->nhalls = nhalls;
	for_each_child_of_node(node, pp) {
		struct flip_cover_hall_data *hall = &pdata->hall[i++];
		enum of_gpio_flags flags;

		hall->gpio = of_get_gpio_flags(pp, 0, &flags);
		if (hall->gpio < 0) {
			ret = hall->gpio;
			if (ret) {
				pr_err("Failed to get gpio flags %d\n", ret);
				return ERR_PTR(ret);
			}
		}
		hall->active_low = flags & OF_GPIO_ACTIVE_LOW;
		hall->irq = gpio_to_irq(hall->gpio);
		hall->name = of_get_property(pp, "name", NULL);
		if (of_property_read_u32(pp, "event", &hall->event)) {
			pr_err("failed to get event: 0x%x\n", hall->event);
			return ERR_PTR(-EINVAL);
		}
	}
	return pdata;
}

static int flip_cover_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
#if IS_ENABLED(CONFIG_SEC_SYSFS)
#if !defined(CONFIG_HALL_NEW_NODE)
	struct device *sec_key_dev;
#endif
#endif
	struct flip_cover_pdata *pdata = dev_get_platdata(dev);
	struct flip_cover_drvdata *ddata;
	struct input_dev *input;
	int ret = 0;
	int wakeup = 0;

	if (!pdata) {
		pdata = flip_cover_parsing_dt(dev);
		if (IS_ERR(pdata)) {
			pr_info("%s : fail to get the DT\n", __func__);
			goto fail1;
		}
	}
	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata) {
		pr_err("failed to allocate drvdata\n");
		ret = -ENOMEM;
		goto fail1;
	}
	input = input_allocate_device();
	if (!input) {
		pr_err("failed to allocate state\n");
		ret = -ENOMEM;
		goto fail2;
	}
	ddata->input = input;
	ddata->pdata = pdata;
	input->name = pdev->name;
	input->phys = pdev->name;
	input->dev.parent = &pdev->dev;
	input->open = flip_cover_open;
	input->close = flip_cover_close;
	device_init_wakeup(&pdev->dev, wakeup);
	platform_set_drvdata(pdev, ddata);
	ret = flip_cover_setup_halls(ddata);
	if (ret) {
		pr_err("failed to set up hall : %d\n", ret);
		goto fail3;
	}
	ret = input_register_device(input);
	if (ret) {
		pr_err("failed to register input device : %d\n", ret);
		goto fail3;
	}
	input_set_drvdata(input, ddata);
#if IS_ENABLED(CONFIG_SEC_SYSFS)
#if defined(CONFIG_HALL_NEW_NODE)
	hall_ic = sec_device_create(ddata, "hall_ic");
	if (pdata->nhalls == 1) {
		struct flip_cover_hall_data *hall = &ddata->pdata->hall[0];

		if (!strncmp(hall->name, "hall", 4))
			ret = sysfs_create_group(&hall_ic->kobj,
						&flip_cover_attr_group);
		else
			ret = sysfs_create_group(&hall_ic->kobj,
						&certify_cover_attr_group);
	} else if (pdata->nhalls == 2) {
		ret = sysfs_create_group(&hall_ic->kobj,
					&flip_cover_attrs_group);
	}
#else
	sec_key_dev = sec_device_find("sec_key");
	if (pdata->nhalls == 1) {
		struct flip_cover_hall_data *hall = &ddata->pdata->hall[0];

		if (!strncmp(hall->name, "hall", 4))
			ret = sysfs_create_group(&sec_key_dev->kobj,
						&flip_cover_attr_group);
		else
			ret = sysfs_create_group(&sec_key_dev->kobj,
						&certify_cover_attr_group);
	} else if (pdata->nhalls == 2) {
		ret = sysfs_create_group(&sec_key_dev->kobj,
					&flip_cover_attrs_group);
	}
#endif
	if (ret) {
		pr_err("failed to create sysfs %d\n", ret);
		goto fail4;
	}
	gddata = ddata;
#endif
	return 0;
fail4:
	input_free_device(input);
fail3:
	platform_set_drvdata(pdev, NULL);
fail2:
	devm_kfree(dev, ddata);
fail1:
	devm_kfree(dev, pdata);
	return ret;
}

static int flip_cover_remove(struct platform_device *pdev)
{
	struct flip_cover_drvdata *ddata = platform_get_drvdata(pdev);
#if IS_ENABLED(CONFIG_SEC_SYSFS)
#if defined(CONFIG_HALL_NEW_NODE)
	hall_ic  = sec_device_find("hall_ic");
	if (ddata->pdata->nhalls == 1) {
		struct flip_cover_hall_data *hall = &ddata->pdata->hall[0];

		if (!strncmp(hall->name, "hall", 4))
			sysfs_remove_group(&hall_ic->parent->kobj,
						&flip_cover_attr_group);
		else
			sysfs_remove_group(&hall_ic->parent->kobj,
						&certify_cover_attr_group);
	} else if (ddata->pdata->nhalls == 2)
		sysfs_remove_group(&hall_ic->parent->kobj,
						&flip_cover_attrs_group);
#else
	struct device *sec_key_dev;

	sec_key_dev = sec_device_find("sec_key");
	if (ddata->pdata->nhalls == 1) {
		struct flip_cover_hall_data *hall = &ddata->pdata->hall[0];

		if (!strncmp(hall->name, "hall", 4))
			sysfs_remove_group(&sec_key_dev->parent->kobj,
						&flip_cover_attr_group);
		else
			sysfs_remove_group(&sec_key_dev->parent->kobj,
						&certify_cover_attr_group);
	} else if (ddata->pdata->nhalls == 2)
		sysfs_remove_group(&sec_key_dev->parent->kobj,
						&flip_cover_attrs_group);
#endif
#endif

	device_init_wakeup(&pdev->dev, 0);
	input_unregister_device(ddata->input);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, ddata->pdata);
	devm_kfree(&pdev->dev, ddata);
	return 0;
}

static const struct of_device_id flip_cover_dt_ids[] = {
	{ .compatible = "flip_cover", },
	{ },
};
MODULE_DEVICE_TABLE(of, flip_cover_dt_ids);

static int flip_cover_suspend(struct device *dev)
{
	struct flip_cover_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ddata->pdata->nhalls; i++) {
		struct flip_cover_hall_data *hall = &ddata->pdata->hall[i];

		enable_irq_wake(hall->irq);
	}
	return 0;
}

static int flip_cover_resume(struct device *dev)
{
	struct flip_cover_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ddata->pdata->nhalls; i++) {
		struct flip_cover_hall_data *hall = &ddata->pdata->hall[i];
		int state = !!gpio_get_value_cansleep(hall->gpio);

		pr_info("%s %s : %s\n", __func__,
			hall->name, state ? "open" : "close");
		disable_irq_wake(hall->irq);
	}
	input_sync(ddata->input);
	return 0;
}

static SIMPLE_DEV_PM_OPS(flip_cover_pm_ops,
	flip_cover_suspend, flip_cover_resume);

static struct platform_driver flip_cover_device_driver = {
	.probe		= flip_cover_probe,
	.remove		= flip_cover_remove,
	.driver		= {
		.name	= "flip_cover",
		.owner	= THIS_MODULE,
		.pm		= &flip_cover_pm_ops,
		.of_match_table	= flip_cover_dt_ids,
	}
};

static int __init flip_cover_init(void)
{
	return platform_driver_register(&flip_cover_device_driver);
}

static void __exit flip_cover_exit(void)
{
	platform_driver_unregister(&flip_cover_device_driver);
}

late_initcall(flip_cover_init);
module_exit(flip_cover_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hall IC driver for SEC Flip cover");
