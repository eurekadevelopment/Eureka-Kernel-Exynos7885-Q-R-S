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
#include "../ssp.h"
#include "ssp_factory.h"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

static ssize_t accel_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->accel_ops->get_accel_name(buf);
}

static ssize_t accel_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->accel_ops->get_accel_vendor(buf);
}

static ssize_t accel_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{	
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->accel_ops->get_accel_calibration(data, buf);
}

static ssize_t accel_calibration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;
	
	ret = data->accel_ops->set_accel_calibration(data, buf);

	if (ret < 0)
		pr_err("[SSP]: %s - failed = %d\n", __func__, ret);

	return size;
}

static ssize_t raw_data_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->accel_ops->get_accel_raw_data(data, buf);
}

static ssize_t accel_reactive_alert_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->accel_ops->get_accel_reactive_alert(data, buf);
}

static ssize_t accel_reactive_alert_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->accel_ops->set_accel_reactive_alert(data, buf);

	if (ret < 0)
		pr_err("[SSP]: %s - failed = %d\n", __func__, ret);

	return size;
}

static ssize_t accel_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->accel_ops->get_accel_selftest(data, buf);
}


static ssize_t accel_lowpassfilter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->accel_ops->set_accel_lowpassfilter(data, buf);

	if (ret < 0)
		pr_err("[SSP]: %s - failed = %d\n", __func__, ret);

	return size;
}

static DEVICE_ATTR(name, S_IRUGO, accel_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, accel_vendor_show, NULL);
static DEVICE_ATTR(calibration, S_IRUGO | S_IWUSR | S_IWGRP,
	accel_calibration_show, accel_calibration_store);
static DEVICE_ATTR(raw_data, S_IRUGO, raw_data_read, NULL);
static DEVICE_ATTR(reactive_alert, S_IRUGO | S_IWUSR | S_IWGRP,
	accel_reactive_alert_show, accel_reactive_alert_store);
static DEVICE_ATTR(selftest, S_IRUGO, accel_selftest_show, NULL);
static DEVICE_ATTR(lowpassfilter, S_IWUSR | S_IWGRP,
	NULL, accel_lowpassfilter_store);

static struct device_attribute *acc_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_calibration,
	&dev_attr_raw_data,
	&dev_attr_reactive_alert,
	&dev_attr_selftest,
	&dev_attr_lowpassfilter,
	NULL,
};

void initialize_accel_factorytest(struct ssp_data *data)
{
#if defined(CONFIG_SENSORS_SSP_ACCELOMETER_LSM6DSL)
	accelometer_lsm6dsl_function_pointer_initialize(data);
#elif defined(CONFIG_SENSORS_SSP_ACCELOMETER_K6DS3TR)
	accelometer_k6ds3tr_function_pointer_initialize(data);
#elif defined(CONFIG_SENSORS_SSP_ACCELOMETER_MPU6500)
	accelometer_mpu6500_function_pointer_initialize(data);
#else
	accelometer_bmi168_function_pointer_initialize(data);
#endif
	
	sensors_register(data->devices[SENSOR_TYPE_ACCELEROMETER], data, acc_attrs,
			data->info[SENSOR_TYPE_ACCELEROMETER].name);
}

void remove_accel_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->devices[SENSOR_TYPE_ACCELEROMETER], acc_attrs);
}
