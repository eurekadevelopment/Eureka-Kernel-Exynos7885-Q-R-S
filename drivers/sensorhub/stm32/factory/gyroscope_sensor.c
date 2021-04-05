/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include <linux/kernel.h>
#include "../ssp.h"
#include "ssp_factory.h"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

static ssize_t gyro_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->gyro_ops->get_gyro_name(buf);
}


static ssize_t gyro_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->gyro_ops->get_gyro_vendor(buf);
}

static ssize_t gyro_power_off(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->gyro_ops->get_gyro_power_off(buf);
}

static ssize_t gyro_power_on(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->gyro_ops->get_gyro_power_on(buf);
}


static ssize_t gyro_temp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->gyro_ops->get_gyro_temperature(data, buf);
}


static ssize_t gyro_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->gyro_ops->get_gyro_selftest(data, buf);
}

static ssize_t gyro_selftest_dps_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->gyro_ops->get_gyro_selftest_dps(data, buf);
}

static ssize_t gyro_selftest_dps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->gyro_ops->set_gyro_selftest_dps(data, buf);
}

static DEVICE_ATTR(name, S_IRUGO, gyro_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, gyro_vendor_show, NULL);
static DEVICE_ATTR(power_off, S_IRUGO, gyro_power_off, NULL);
static DEVICE_ATTR(power_on, S_IRUGO, gyro_power_on, NULL);
static DEVICE_ATTR(temperature, S_IRUGO, gyro_temp_show, NULL);
static DEVICE_ATTR(selftest, S_IRUGO, gyro_selftest_show, NULL);
static DEVICE_ATTR(selftest_dps, S_IRUGO | S_IWUSR | S_IWGRP,
	gyro_selftest_dps_show, gyro_selftest_dps_store);

static struct device_attribute *gyro_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_selftest,
	&dev_attr_power_on,
	&dev_attr_power_off,
	&dev_attr_temperature,
	&dev_attr_selftest_dps,
	NULL,
};

void initialize_gyro_factorytest(struct ssp_data *data)
{
#if defined(CONFIG_SENSORS_SSP_GYROSCOPE_LSM6DSL)
	gyroscope_lsm6dsl_function_pointer_initialize(data);
#elif defined(CONFIG_SENSORS_SSP_GYROSCOPE_K6DS3TR)
	gyroscope_k6ds3tr_function_pointer_initialize(data);
#elif defined(CONFIG_SENSORS_SSP_GYROSCOPE_MPU6500)
	gyroscope_mpu6500_function_pointer_initialize(data);
#else
	gyroscope_bmi168_function_pointer_initialize(data);
#endif
	
	sensors_register(data->devices[SENSOR_TYPE_GYROSCOPE], data, gyro_attrs,
			data->info[SENSOR_TYPE_GYROSCOPE].name);
}

void remove_gyro_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->devices[SENSOR_TYPE_GYROSCOPE], gyro_attrs);
}
