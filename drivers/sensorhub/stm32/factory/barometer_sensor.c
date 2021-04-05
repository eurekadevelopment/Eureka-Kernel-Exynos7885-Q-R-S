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

#define BOSCH_ID	0
#define	VENDOR		"BOSCH"
#define	CHIP_ID		"BMP280"

#define STM_ID		1
#define VENDOR_STM	"STM"
#define CHIP_ID_STM	"LPS25H"

#define CALIBRATION_FILE_PATH		"/efs/FactoryApp/baro_delta"

#define	PR_ABS_MAX	8388607		/* 24 bit 2'compl */
#define	PR_ABS_MIN	-8388608

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/


static ssize_t pressure_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->barometer_ops->get_barometer_name(buf);
}

static ssize_t pressure_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->barometer_ops->get_barometer_vendor(buf);
}

static ssize_t sea_level_pressure_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	data->barometer_ops->set_barometer_sea_level_pressure(data, buf);
	
	return size;
}

static ssize_t pressure_cabratioin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->barometer_ops->get_barometer_calibration(data, buf);
}

static ssize_t pressure_cabratioin_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	data->barometer_ops->set_barometer_calibration(data, buf);

	return size;
}

static ssize_t eeprom_check_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->barometer_ops->get_barometer_eeprom_check(data, buf);
}

static ssize_t pressure_temperature_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->barometer_ops->get_barometer_temperature(data, buf);
}

static DEVICE_ATTR(vendor,  S_IRUGO, pressure_vendor_show, NULL);
static DEVICE_ATTR(name,  S_IRUGO, pressure_name_show, NULL);
static DEVICE_ATTR(eeprom_check, S_IRUGO, eeprom_check_show, NULL);
static DEVICE_ATTR(calibration,  S_IRUGO | S_IWUSR | S_IWGRP,
	pressure_cabratioin_show, pressure_cabratioin_store);
static DEVICE_ATTR(sea_level_pressure, S_IWUSR | S_IWGRP,
	NULL, sea_level_pressure_store);
static DEVICE_ATTR(temperature, S_IRUGO, pressure_temperature_show, NULL);

static struct device_attribute *pressure_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_calibration,
	&dev_attr_sea_level_pressure,
	&dev_attr_eeprom_check,
	&dev_attr_temperature,
	NULL,
};

void initialize_barometer_factorytest(struct ssp_data *data)
{
#if defined(CONFIG_SENSORS_SSP_BAROMETER_LPS25H)
	barometer_lps25h_function_pointer_initialize(data);
#elif defined(CONFIG_SENSORS_SSP_BAROMETER_LPS22H)
	barometer_lps22h_function_pointer_initialize(data);
#else
	barometer_bmp280_function_pointer_initialize(data);
#endif
	
	sensors_register(data->devices[SENSOR_TYPE_PRESSURE], data, pressure_attrs,
		"barometer_sensor");
}

void remove_barometer_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->devices[SENSOR_TYPE_PRESSURE], pressure_attrs);
}
