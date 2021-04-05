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

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define MODEL_NAME	"STM32F410TBY6TR"

ssize_t mcu_revision_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "ST01%u,ST01%u\n", data->curr_fw_rev, get_module_rev(data));
}

ssize_t mcu_model_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", MODEL_NAME);
}

ssize_t mcu_update_kernel_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool is_success = false;
	int ret = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("mcu binany update!");

	data->is_reset_from_sysfs = true;
	ret = forced_to_download_binary(data, UMS_BINARY);
	if (ret == SUCCESS) {
		is_success = true;
		goto out;
	}

	ret = forced_to_download_binary(data, KERNEL_BINARY);
	if (ret == SUCCESS)
		is_success = true;
	else
	{
		is_success = false;
		data->is_reset_from_sysfs = false;
	}
out:
	return sprintf(buf, "%s\n", (is_success ? "OK" : "NG"));
}

ssize_t mcu_update_kernel_crashed_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	int ret = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("mcu binany update!");

	data->is_reset_from_sysfs = true;
	ret = forced_to_download_binary(data, UMS_BINARY);
	if (ret == SUCCESS) {
		bSuccess = true;
		goto out;
	}

	ret = forced_to_download_binary(data, KERNEL_CRASHED_BINARY);
	if (ret == SUCCESS)
		bSuccess = true;
	else
	{
		bSuccess = false;
		data->is_reset_from_sysfs = false;
	}
out:
	return sprintf(buf, "%s\n", (bSuccess ? "OK" : "NG"));
}

ssize_t mcu_update_ums_bin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bSuccess = false;
	int ret = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("mcu binany update!");

	data->is_reset_from_sysfs = true;
	ret = forced_to_download_binary(data, UMS_BINARY);
	if (ret == SUCCESS)
		bSuccess = true;
	else
	{
		bSuccess = false;
		data->is_reset_from_sysfs = false;
	}

	return sprintf(buf, "%s\n", (bSuccess ? "OK" : "NG"));
}

ssize_t mcu_reset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	data->is_reset_started = true;
	data->is_reset_from_sysfs = true;
	reset_mcu(data);

	return sprintf(buf, "OK\n");
}

ssize_t mcu_dump_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	struct ssp_data *data = dev_get_drvdata(dev);
	int status = 1, iDelaycnt = 0;

	data->is_ongoing_dump = true;
	set_big_data_start(data, BIG_TYPE_DUMP, 0);
	msleep(300);
	while (data->is_ongoing_dump) {
		mdelay(10);
		if (iDelaycnt++ > 1000) {
			status = 0;
			break;
		}
	}
	return sprintf(buf, "%s\n", status ? "OK" : "NG");
}

static char buffer[FACTORY_DATA_MAX];

ssize_t mcu_factorytest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (sysfs_streq(buf, "1")) {
		struct ssp_msg *msg;
		
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = MCU_FACTORY;
		msg->length = 5;
		msg->options = AP2HUB_READ;
		msg->buffer = buffer;
		msg->free_buffer = 0;

		memset(msg->buffer, 0, 5);

		ret = ssp_spi_async(data, msg);

	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	ssp_info("MCU Factory Test Start! - %d", ret);

	return size;
}

ssize_t mcu_factorytest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool bMcuTestSuccessed = false;

	ssp_info("MCU Factory Test Data : %u, %u, %u, %u, %u",
		buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);

	/* system clock, RTC, I2C Master, I2C Slave, externel pin */
	if ((buffer[0] == SUCCESS)
			&& (buffer[1] == SUCCESS)
			&& (buffer[2] == SUCCESS)
			&& (buffer[3] == SUCCESS)
			&& (buffer[4] == SUCCESS))
		bMcuTestSuccessed = true;

	ssp_info("MCU Factory Test Result - %s, %s, %s\n", MODEL_NAME,
		(bMcuTestSuccessed ? "OK" : "NG"), "OK");

	return sprintf(buf, "%s,%s,%s\n", MODEL_NAME,
		(bMcuTestSuccessed ? "OK" : "NG"), "OK");
}

ssize_t mcu_sleep_factorytest_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1")) {
		int ret = 0;
		struct ssp_msg *msg;
		
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = MCU_SLEEP_FACTORY;
		msg->length = FACTORY_DATA_MAX;
		msg->options = AP2HUB_READ;
		msg->buffer = buffer;
		msg->free_buffer = 0;

		ret = ssp_spi_async(data, msg);
		if(ret < 0)
		{
			pr_err("[SSP]: %s - MCU_SLEEP_FACTORY Fail %d\n", __func__,ret);
			return -EIO;
		}
	
	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	ssp_info("MCU Sleep Factory Test Start! - %d", 1);

	return size;
}

ssize_t mcu_sleep_factorytest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int iDataIdx, iSensorData = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct sensor_value fsb[SENSOR_TYPE_MAX];
	u16 chLength = 0;

	memcpy(&chLength, buffer, 2);
	memset(fsb, 0, sizeof(struct sensor_value) * SENSOR_TYPE_MAX);

	for (iDataIdx = 2; iDataIdx < chLength + 2;) {
		iSensorData = (int)buffer[iDataIdx++];

		if ((iSensorData < 0) ||
			(iSensorData >= (SENSOR_TYPE_MAX - 1))) {
			pr_err("[SSP]: %s - Mcu data frame error %d\n",
				__func__, iSensorData);
			goto exit;
		}

		get_sensordata(data, (char *)buffer, &iDataIdx,
				iSensorData, &(fsb[iSensorData]));
	}

	fsb[SENSOR_TYPE_PRESSURE].pressure
			-= data->buf[SENSOR_TYPE_PRESSURE].pressure_cal;

exit:
	ssp_infof("Result\n"
		"[SSP] accel %d,%d,%d\n"
		"[SSP] gyro %d,%d,%d\n"
		"[SSP] mag %d,%d,%d\n"
		"[SSP] baro %d,%d\n"
		"[SSP] prox %u,%u\n"
		"[SSP] light %u,%u,%u,%u,%u,%u"
		"[SSP]: temp %d,%d,%d\n",
		fsb[SENSOR_TYPE_ACCELEROMETER].x, fsb[SENSOR_TYPE_ACCELEROMETER].y,
		fsb[SENSOR_TYPE_ACCELEROMETER].z, fsb[SENSOR_TYPE_GYROSCOPE].x,
		fsb[SENSOR_TYPE_GYROSCOPE].y, fsb[SENSOR_TYPE_GYROSCOPE].z,
		fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_x, fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_y,
		fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_z, fsb[SENSOR_TYPE_PRESSURE].pressure,
		fsb[SENSOR_TYPE_PRESSURE].temperature,
		fsb[SENSOR_TYPE_PROXIMITY].prox, fsb[SENSOR_TYPE_PROXIMITY].prox_ex,
		fsb[SENSOR_TYPE_LIGHT].r, fsb[SENSOR_TYPE_LIGHT].g, fsb[SENSOR_TYPE_LIGHT].b,
		fsb[SENSOR_TYPE_LIGHT].w,
		fsb[SENSOR_TYPE_LIGHT].a_time, fsb[SENSOR_TYPE_LIGHT].a_gain,
		fsb[SENSOR_TYPE_TEMPERATURE].x, fsb[SENSOR_TYPE_TEMPERATURE].y,
		fsb[SENSOR_TYPE_TEMPERATURE].z);

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,"
		"%u,%u,%u,%u,%u,%u,0,0,0,0,%d,%d\n",
		fsb[SENSOR_TYPE_ACCELEROMETER].x, fsb[SENSOR_TYPE_ACCELEROMETER].y,
		fsb[SENSOR_TYPE_ACCELEROMETER].z, fsb[SENSOR_TYPE_GYROSCOPE].x,
		fsb[SENSOR_TYPE_GYROSCOPE].y, fsb[SENSOR_TYPE_GYROSCOPE].z,
		fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_x, fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_y,
		fsb[SENSOR_TYPE_GEOMAGNETIC_FIELD].cal_z, fsb[SENSOR_TYPE_PRESSURE].pressure,
		fsb[SENSOR_TYPE_PRESSURE].temperature, fsb[SENSOR_TYPE_PROXIMITY].prox_ex,
		fsb[SENSOR_TYPE_LIGHT].r, fsb[SENSOR_TYPE_LIGHT].g, fsb[SENSOR_TYPE_LIGHT].b,
		fsb[SENSOR_TYPE_LIGHT].w,
		fsb[SENSOR_TYPE_LIGHT].a_time, fsb[SENSOR_TYPE_LIGHT].a_gain,
		/*fsb[GESTURE_SENSOR].data[0], fsb[GESTURE_SENSOR].data[1],
		fsb[GESTURE_SENSOR].data[2], fsb[GESTURE_SENSOR].data[3],*/
		fsb[SENSOR_TYPE_TEMPERATURE].x,	fsb[SENSOR_TYPE_TEMPERATURE].y);
}
