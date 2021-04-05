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

#define VENDOR_INV		"INVENSENSE"
#define CHIP_ID_INV		"MPU6500"

#define VENDOR_STM		"STM"
#define CHIP_ID_STM		"K6DS3TR"
//#define CHIP_ID_STM		"LSM303AH"


#define VENDOR_BOSCH		"BOSCH"
#define CHIP_ID_BOSCH		"BMI168"

#define MAX_ACCEL_1G		4096
#define MAX_ACCEL_2G		8192
#define MIN_ACCEL_2G		-8192
#define MAX_ACCEL_4G		16384

#define CALIBRATION_FILE_PATH	"/efs/FactoryApp/calibration_data"
#define CALIBRATION_DATA_AMOUNT	20

static ssize_t accel_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	if (data->acc_type == SIX_AXIS_MPU6500)
		return sprintf(buf, "%s\n", VENDOR_INV);
	else if (data->acc_type == SIX_AXIS_K6DS3)
		return sprintf(buf, "%s\n", VENDOR_STM);
	else if (data->acc_type == SIX_AXIS_LSM303AH)
		return sprintf(buf, "%s\n", VENDOR_STM);
	else
		return sprintf(buf, "%s\n", VENDOR_BOSCH);
}

static ssize_t accel_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	if (data->acc_type == SIX_AXIS_MPU6500)
		return sprintf(buf, "%s\n", CHIP_ID_INV);
	else if (data->acc_type == SIX_AXIS_K6DS3)
		return sprintf(buf, "%s\n", CHIP_ID_STM);
	else if (data->acc_type == SIX_AXIS_LSM303AH)
		return sprintf(buf, "%s\n", CHIP_ID_STM);
	else
		return sprintf(buf, "%s\n", CHIP_ID_BOSCH);
}

int accel_open_calibration(struct ssp_data *data)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0660);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);

		data->accelcal.x = 0;
		data->accelcal.y = 0;
		data->accelcal.z = 0;

		return ret;
	}

	ret = vfs_read(cal_filp, (char *)&data->accelcal,
		3 * sizeof(int), &cal_filp->f_pos);
	if (ret != 3 * sizeof(int))
		ret = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ssp_infof("open accel calibration %d, %d, %d\n",
		data->accelcal.x, data->accelcal.y, data->accelcal.z);

	if ((data->accelcal.x == 0) && (data->accelcal.y == 0)
		&& (data->accelcal.z == 0))
		return ERROR;

	return ret;
}

int set_accel_cal(struct ssp_data *data)
{
	int ret = 0;
	struct ssp_msg *msg;
	s16 accel_cal[3] = {0, };

	if (!(data->uSensorState & (1 << SENSOR_TYPE_ACCELEROMETER))) {
		pr_info("[SSP]: %s - Skip this function!!!"\
			", accel sensor is not connected(0x%llx)\n",
			__func__, data->uSensorState);
		return ret;
	}

	accel_cal[0] = data->accelcal.x;
	accel_cal[1] = data->accelcal.y;
	accel_cal[2] = data->accelcal.z;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_AP_MCU_SET_ACCEL_CAL;
	msg->length = 6;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *)accel_cal;

	msg->free_buffer = 0;

	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, ret);
		ret = ERROR;
	}

	pr_info("[SSP] Set accel cal data %d, %d, %d\n",
		data->accelcal.x, data->accelcal.y, data->accelcal.z);

	return ret;
}

static int enable_accel_for_cal(struct ssp_data *data)
{
	u8 buf[9] = { 0, };
	s32 dMsDelay = get_msdelay(data->delay[SENSOR_TYPE_ACCELEROMETER]);
	memcpy(&buf[0], &dMsDelay, 4);

	if (atomic64_read(&data->aSensorEnable) & (1 << SENSOR_TYPE_ACCELEROMETER)) {
		if (get_msdelay(data->delay[SENSOR_TYPE_ACCELEROMETER]) != 10) {
			send_instruction(data, CHANGE_DELAY,
				SENSOR_TYPE_ACCELEROMETER, buf, 9);
			return SUCCESS;
		}
	} else {
		send_instruction(data, ADD_SENSOR,
			SENSOR_TYPE_ACCELEROMETER, buf, 9);
	}

	return FAIL;
}

static void disable_accel_for_cal(struct ssp_data *data, int delay)
{
	u8 buf[9] = { 0, };
	s32 dMsDelay = get_msdelay(data->delay[SENSOR_TYPE_ACCELEROMETER]);
	memcpy(&buf[0], &dMsDelay, 4);

	if (atomic64_read(&data->aSensorEnable) & (1 << SENSOR_TYPE_ACCELEROMETER)) {
		if (delay)
			send_instruction(data, CHANGE_DELAY,
				SENSOR_TYPE_ACCELEROMETER, buf, 9);
	} else {
		send_instruction(data, REMOVE_SENSOR,
			SENSOR_TYPE_ACCELEROMETER, buf, 4);
	}
}

static int accel_do_calibrate(struct ssp_data *data, int enable)
{
	int iSum[3] = { 0, };
	int ret = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;

	if (enable) {
		int count;
		data->accelcal.x = 0;
		data->accelcal.y = 0;
		data->accelcal.z = 0;
		set_accel_cal(data);

		ret = enable_accel_for_cal(data);
		msleep(300);

		for (count = 0; count < CALIBRATION_DATA_AMOUNT; count++) {
			iSum[0] += data->buf[SENSOR_TYPE_ACCELEROMETER].x;
			iSum[1] += data->buf[SENSOR_TYPE_ACCELEROMETER].y;
			iSum[2] += data->buf[SENSOR_TYPE_ACCELEROMETER].z;
			mdelay(10);
		}
		disable_accel_for_cal(data, ret);

		data->accelcal.x = (iSum[0] / CALIBRATION_DATA_AMOUNT);
		data->accelcal.y = (iSum[1] / CALIBRATION_DATA_AMOUNT);
		data->accelcal.z = (iSum[2] / CALIBRATION_DATA_AMOUNT);

		if (data->accelcal.z > 0)
			data->accelcal.z -= MAX_ACCEL_1G;
		else if (data->accelcal.z < 0)
			data->accelcal.z += MAX_ACCEL_1G;
	} else {
		data->accelcal.x = 0;
		data->accelcal.y = 0;
		data->accelcal.z = 0;
	}

	ssp_info("do accel calibrate %d, %d, %d",
		data->accelcal.x, data->accelcal.y, data->accelcal.z);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0660);
	if (IS_ERR(cal_filp)) {
		pr_err("[SSP]: %s - Can't open calibration file\n", __func__);
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);
		return ret;
	}

	ret = vfs_write(cal_filp, (char *)&data->accelcal,
		3 * sizeof(int), &cal_filp->f_pos);
	if (ret != 3 * sizeof(int)) {
		pr_err("[SSP]: %s - Can't write the accelcal to file\n",
			__func__);
		ret = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);
	set_accel_cal(data);
	return ret;
}

static ssize_t accel_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;
	struct ssp_data *data = dev_get_drvdata(dev);

	ret = accel_open_calibration(data);
	if (ret < 0)
		pr_err("[SSP]: %s - calibration open failed(%d)\n",
			__func__, ret);

	ssp_info("Cal data : %d %d %d - %d",
		data->accelcal.x, data->accelcal.y, data->accelcal.z, ret);

	return sprintf(buf, "%d %d %d %d\n", ret, data->accelcal.x,
			data->accelcal.y, data->accelcal.z);
}

static ssize_t accel_calibration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int64_t enable;
	struct ssp_data *data = dev_get_drvdata(dev);

	ret = kstrtoll(buf, 10, &enable);
	if (ret < 0)
		return ret;

	ret = accel_do_calibrate(data, (int)enable);
	if (ret < 0)
		pr_err("[SSP]: %s - accel_do_calibrate() failed\n", __func__);

	return size;
}

static ssize_t raw_data_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
		data->buf[SENSOR_TYPE_ACCELEROMETER].x,
		data->buf[SENSOR_TYPE_ACCELEROMETER].y,
		data->buf[SENSOR_TYPE_ACCELEROMETER].z);
}

static ssize_t accel_reactive_alert_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	char temp = 1;
	struct ssp_data *data = dev_get_drvdata(dev);

	struct ssp_msg *msg;

	if (sysfs_streq(buf, "1")) {
		ssp_infof("on");
	} else if (sysfs_streq(buf, "0")) {
		ssp_infof("off");
	} else if (sysfs_streq(buf, "2")) {
		ssp_infof("factory");

		data->is_accel_alert = 0;

		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = ACCELEROMETER_FACTORY;
		msg->length = 1;
		msg->options = AP2HUB_READ;
		msg->data = temp;
		msg->buffer = &temp;
		msg->free_buffer = 0;

		ret = ssp_spi_sync(data, msg, 3000);
		data->is_accel_alert = temp;

		if (ret != SUCCESS) {
			pr_err("[SSP]: %s - accel Selftest Timeout!!\n",
				__func__);
			goto exit;
		}

		ssp_infof("factory test success!");
	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
exit:
	return size;
}

static ssize_t accel_reactive_alert_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	bool success = false;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (data->is_accel_alert == true)
		success = true;
	else
		success = false;

	data->is_accel_alert = false;
	return sprintf(buf, "%u\n", success);
}

static ssize_t mpu6500_accel_selftest(char *buf, struct ssp_data *data)
{
	char temp[8] = { 2, 0, };
	s8 init_status = 0, result = -1;
	s16 shift_ratio[3] = { 0, };
	int ret;
	struct ssp_msg *msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = ACCELEROMETER_FACTORY;
	msg->length = 8;
	msg->options = AP2HUB_READ;
	msg->data = temp[0];
	msg->buffer = temp;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 3000);
	if (ret != SUCCESS) {
		pr_err("[SSP] %s - accel hw selftest Timeout!!\n", __func__);
		return sprintf(buf, "%d,%d,%d,%d\n", -5, 0, 0, 0);
	}

	init_status = temp[0];
	shift_ratio[0] = (s16)((temp[2] << 8) + temp[1]);
	shift_ratio[1] = (s16)((temp[4] << 8) + temp[3]);
	shift_ratio[2] = (s16)((temp[6] << 8) + temp[5]);
	result = temp[7];

	pr_info("[SSP] %s - %d, %d, %d, %d, %d\n",
		__func__, init_status, result,
		shift_ratio[0], shift_ratio[1], shift_ratio[2]);

	return sprintf(buf, "%d,%d.%d,%d.%d,%d.%d\n", result,
		shift_ratio[0] / 10, shift_ratio[0] % 10,
		shift_ratio[1] / 10, shift_ratio[1] % 10,
		shift_ratio[2] / 10, shift_ratio[2] % 10);
}

static ssize_t accel_selftest(char *buf, struct ssp_data *data)
{
	char temp[12] = { 2, 0, };
	s8 init_status = 0, result = -1;
	u16 diff_axis[3] = { 0, };
	int ret;
	struct ssp_msg *msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = ACCELEROMETER_FACTORY;
	msg->length = sizeof(temp);
	msg->options = AP2HUB_READ;
	msg->data = temp[0];
	msg->buffer = temp;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 3000);
	if (ret != SUCCESS) {
		pr_err("[SSP] %s - accel hw selftest Timeout!!\n", __func__);
		return sprintf(buf, "%d,%d,%d,%d\n", -5, 0, 0, 0);
	}

	init_status = temp[0];
	diff_axis[0] = ((s16)(temp[2] << 8)) + temp[1];
	diff_axis[1] = ((s16)(temp[4] << 8)) + temp[3];
	diff_axis[2] = ((s16)(temp[6] << 8)) + temp[5];
	result = temp[7];

	pr_info("[SSP] %s - %d, %d, %d, %d, %d\n", __func__,
		init_status, result, diff_axis[0], diff_axis[1], diff_axis[2]);
	return sprintf(buf, "%d,%d,%d,%d\n",
		result, diff_axis[0], diff_axis[1], diff_axis[2]);
}


static ssize_t accel_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	if (data->acc_type == SIX_AXIS_MPU6500)
		return mpu6500_accel_selftest(buf, data);
	else if (data->acc_type == SIX_AXIS_K6DS3)
		return accel_selftest(buf, data);
	else
		return accel_selftest(buf, data);
}


static ssize_t accel_lowpassfilter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0, new_enable = 1;
	char temp = 0;
	struct ssp_data *data = dev_get_drvdata(dev);
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	if (sysfs_streq(buf, "1"))
		new_enable = 1;
	else if (sysfs_streq(buf, "0"))
		new_enable = 0;
	else
		ssp_info(" invalid value!");

	temp = new_enable;

	msg->cmd = MSG2SSP_AP_SENSOR_LPF;
	msg->length = 1;
	msg->options = AP2HUB_WRITE;
	msg->buffer = &temp;

	msg->free_buffer = 0;

	ret = ssp_spi_async(data, msg);
	if (ret != SUCCESS)
		pr_err("[SSP] %s - fail %d\n", __func__, ret);
	else
		pr_info("[SSP] %s - %d\n", __func__, new_enable);

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
	sensors_register(data->devices[SENSOR_TYPE_ACCELEROMETER], data, acc_attrs,
			data->info[SENSOR_TYPE_ACCELEROMETER].name);
}

void remove_accel_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->devices[SENSOR_TYPE_ACCELEROMETER], acc_attrs);
}
