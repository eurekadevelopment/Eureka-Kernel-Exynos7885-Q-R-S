/*
 *
 *  Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *  Insun Choi <insun77.choi@samsung.com>, Le Hoang Anh <anh.lh@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* SUB-PBA Detector using ADC */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/iio/consumer.h>
#include <linux/platform_data/sec_spd_adc.h>
#include <linux/sec_class.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sec_sysfs.h>

#define ADC_SAMPLING_CNT	5

struct sec_adc_info {
	struct device *dev;
	struct device *sec_dev;
	struct device *hwmon_dev;
	struct sec_adc_platform_data *pdata;
	struct iio_channel *chan;
	const char *name;
	struct device_node *np;
};

static int pcb_val;

#ifdef CONFIG_OF
static const struct of_device_id sec_adc_match[] = {
	{ .compatible = "samsung,sec-adc-detector", },
	{ },
};
MODULE_DEVICE_TABLE(of, sec_adc_match);

static struct sec_adc_platform_data *sec_adc_parse_dt(struct platform_device *pdev) // save adc table in dt
{
	struct device_node *np = pdev->dev.of_node;
	struct sec_adc_platform_data *pdata;
	u32 adc_min_len, adc_max_len;
	int i;
	u32 adc_min, adc_max;
	u32 sub_rev;

	if (!np)
		return NULL;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);
	
	if (of_property_read_string(np, "adc_name", &pdata->name)) {
		pr_info("%s failed to get adc name\n",__func__);
		return NULL;
	}
	if (!of_get_property(np, "adc_min", &adc_min_len))
		return ERR_PTR(-ENOENT);
	if (!of_get_property(np, "adc_max", &adc_max_len))
		return ERR_PTR(-ENOENT);

	if (adc_min_len != adc_max_len) {
		pr_info("%s: invalid array length(%u,%u)\n",
				__func__, adc_min_len, adc_max_len);
		return ERR_PTR(-EINVAL);
	}

	pdata->adc_arr_size = adc_min_len / sizeof(u32);
	pdata->adc_table = devm_kzalloc(&pdev->dev,
			sizeof(*pdata->adc_table) * pdata->adc_arr_size,
			GFP_KERNEL);
	if (!pdata->adc_table)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < pdata->adc_arr_size; i++) {
		if (of_property_read_u32_index(np, "adc_min", i, &adc_min))
			return ERR_PTR(-EINVAL);
		if (of_property_read_u32_index(np, "adc_max", i, &adc_max))
			return ERR_PTR(-EINVAL);
		if (of_property_read_u32_index(np, "sub_rev", i, &sub_rev))
			return ERR_PTR(-EINVAL);

		pdata->adc_table[i].adc_min = (int)adc_min;
		pdata->adc_table[i].adc_max = (int)adc_max;
		pdata->adc_table[i].sub_rev = (int)sub_rev;
	}
	return pdata;
}
#else
static int sec_adc_parse_dt(struct platform_device *pdev) { return NULL; }
#endif

static int sec_adc_get_data(struct sec_adc_info *info) // get adc value on board
{
	int adc_data;
	int adc_max = 0, adc_min = 0, adc_total = 0;
	int i;

	for (i = 0; i < ADC_SAMPLING_CNT; i++) {
		int ret = iio_read_channel_raw(info->chan, &adc_data);

		if (ret < 0) {
			pr_info("%s : err(%d) returned, skip read\n",
				__func__, adc_data);
			return ret;
		}
		pr_info("%s: adc_data = %d\n",__func__,adc_data);
		if (i != 0) {
			if (adc_data > adc_max)
				adc_max = adc_data;
			else if (adc_data < adc_min)
				adc_min = adc_data;
		} else {
			adc_max = adc_data;
			adc_min = adc_data;
		}
		adc_total += adc_data;
	}

	return (adc_total - adc_max - adc_min) / (ADC_SAMPLING_CNT - 2);
}

static int sec_adc_check_pcb(struct sec_adc_info *info, int adc)
{
	int tmp_adc_min, tmp_adc_max;
	int i;
	
	for (i = 0; i < info->pdata->adc_arr_size; i++) {
		tmp_adc_min = info->pdata->adc_table[i].adc_min;
		tmp_adc_max = info->pdata->adc_table[i].adc_max;
		if (adc >= tmp_adc_min && adc <= tmp_adc_max)
			return info->pdata->adc_table[i].sub_rev;
	}
	
	return -1;
}

static ssize_t sec_adc_show_pcb(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%d\n", pcb_val);
}

static ssize_t sec_adc_show_val(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct sec_adc_info *info = dev_get_drvdata(dev);
	int adc;

	adc = sec_adc_get_data(info);

	return sprintf(buf, "%d\n", adc);
}

static ssize_t sec_adc_show_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sec_adc_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", info->name);
}

static DEVICE_ATTR(pcb_val, 0444, sec_adc_show_pcb, NULL);
static DEVICE_ATTR(adc_val, 0444, sec_adc_show_val, NULL);
static DEVICE_ATTR(name, 0444, sec_adc_show_name, NULL);

static struct attribute *sec_adc_attributes[] = {
	&dev_attr_pcb_val.attr,
	&dev_attr_adc_val.attr,
	&dev_attr_name.attr,
	NULL
};

static int check_subfpcb_type(struct seq_file *m, void *v) 
{
	pr_info("%s: fpcb gpio value is %d\n", __func__, pcb_val);
	seq_printf(m, "%u\n", pcb_val);

	return 0;
}

static const struct attribute_group sec_adc_attr_group = {
	.attrs = sec_adc_attributes,
};

static int check_subfpcb_type_open(struct inode *inode, struct file *file) {
	return single_open(file, check_subfpcb_type, NULL);
}

static const struct file_operations check_subfpcb_type_fops = {
	.open		= check_subfpcb_type_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int sec_adc_probe(struct platform_device *pdev)
{
	struct sec_adc_platform_data *pdata;
	struct sec_adc_info *info;
	struct proc_dir_entry *entry;
	int ret;

	pr_info("%s: SEC ADC Driver Loading\n", __func__);

	pdata = sec_adc_parse_dt(pdev);
	if (IS_ERR(pdata)) {
		pr_info("%s: fail to parse dt\n", __func__);
		return IS_ERR(pdata);
	}else if (pdata == NULL)
		pdata = pdev->dev.platform_data;
	
	if (!pdata) {
		pr_info("No platform init data supplied.\n");
		return -ENODEV;
	}
	
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->pdata = pdata;
	info->name = pdata->name;

	platform_set_drvdata(pdev, info);
	
	info->chan = iio_channel_get(&pdev->dev, NULL);
	if (IS_ERR(info->chan)) {
		pr_info("%s: fail to get iio channel\n", __func__);
		return PTR_ERR(info->chan);
	}
	
	info->sec_dev = sec_device_create(info, info->name);
	if (IS_ERR(info->sec_dev)) {
		pr_info("%s: fail to create sec_dev\n", __func__);
		return PTR_ERR(info->sec_dev);
	}

	ret = sysfs_create_group(&info->sec_dev->kobj, &sec_adc_attr_group);
	if (ret) {
		pr_info("failed to create sysfs group\n");
		goto err_create_sysfs;
	}

	info->hwmon_dev = hwmon_device_register(info->dev);
	if (IS_ERR(info->hwmon_dev)) {
		pr_info("unable to register as hwmon device.\n");
		ret = PTR_ERR(info->hwmon_dev);
		goto err_register_hwmon;
	}
	
	entry = proc_create("subfpcb_type", S_IRUGO, NULL, &check_subfpcb_type_fops);
	if (!entry) {
		pr_info("%s: failed to create a proc fs node\n", __func__);
		return -ENOMEM;
	}
	
	pcb_val = sec_adc_check_pcb(info, sec_adc_get_data(info));
	
	dev_info(info->dev, "%s successfully probed.\n", info->name);

	return 0;

err_register_hwmon:
	sysfs_remove_group(&info->sec_dev->kobj, &sec_adc_attr_group);
err_create_sysfs:
	sec_device_destroy(info->sec_dev->devt);
	return ret;
}

static int sec_adc_remove(struct platform_device *pdev)
{
	struct sec_adc_info *info = platform_get_drvdata(pdev);

	if (!info)
		return 0;

	hwmon_device_unregister(info->hwmon_dev);

	sysfs_remove_group(&info->sec_dev->kobj, &sec_adc_attr_group);

	iio_channel_release(info->chan);
	sec_device_destroy(info->sec_dev->devt);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver sec_adc_driver = {
	.driver = {
		.name = "sec-adc-detector",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sec_adc_match),
	},
	.probe = sec_adc_probe,
	.remove = sec_adc_remove,
};

module_platform_driver(sec_adc_driver);

MODULE_DESCRIPTION("SEC ADC Detector Driver");
MODULE_AUTHOR("Insun Choi <insun77.choi@samsung.com>, Le Hoang Anh <anh.lh@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sec-adc-detector");

