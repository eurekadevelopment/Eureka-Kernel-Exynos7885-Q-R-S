/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include "../../ssp.h"
#include "../ssp_factory.h"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define CALIBRATION_FILE_PATH   "/efs/FactoryApp/calibration_data"
#define CALIBRATION_DATA_AMOUNT 20

#define MAX_ACCEL_1G            8192
#define MAX_ACCEL_2G            16384
#define MIN_ACCEL_2G            -16383
#define MAX_ACCEL_4G            32768

ssize_t get_accel_bmi168_name(char *buf)
{
	return sprintf(buf, "%s\n", "BMI168");
}

ssize_t get_accel_bmi168_vendor(char *buf)
{
	return sprintf(buf, "%s\n", "BOSCH");
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

	ret = vfs_read(cal_filp, (char *)&data->accelcal, 3 * sizeof(int), &cal_filp->f_pos);
	if (ret != 3 * sizeof(int)) {
		ret = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ssp_infof("open accel calibration %d, %d, %d\n",
	          data->accelcal.x, data->accelcal.y, data->accelcal.z);

	if ((data->accelcal.x == 0) && (data->accelcal.y == 0)
	    && (data->accelcal.z == 0)) {
		return ERROR;
	}

	return ret;
}

int set_accel_cal(struct ssp_data *data)
{
	int ret = 0;
	//struct ssp_msg *msg;
	s16 accel_cal[3];

	if (!(data->uSensorState & (1 << SENSOR_TYPE_ACCELEROMETER))) {
		pr_info("[SSP]: %s - Skip this function!!!"\
		        ", accel sensor is not connected(0x%llx)\n",
		        __func__, data->uSensorState);
		return ret;
	}
	accel_cal[0] = data->accelcal.x;
	accel_cal[1] = data->accelcal.y;
	accel_cal[2] = data->accelcal.z;

#if 0
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_AP_MCU_SET_ACCEL_CAL;
	msg->length = 6;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(6, GFP_KERNEL);

	msg->free_buffer = 1;
	memcpy(msg->buffer, accel_cal, 6);

	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, ret);
		ret = ERROR;
	}
#endif
	pr_info("[SSP] Set accel cal data %d, %d, %d\n", accel_cal[0], accel_cal[1],
	        accel_cal[2]);
	return ret;
}

int enable_accel_for_cal(struct ssp_data *data)
{
	u8 uBuf[9] = { 0, };
	s32 dMsDelay = get_msdelay(data->delay[SENSOR_TYPE_ACCELEROMETER]);
	memcpy(&uBuf[0], &dMsDelay, 4);

	if (atomic64_read(&data->aSensorEnable) & (1 << SENSOR_TYPE_ACCELEROMETER)) {
		if (get_msdelay(data->delay[SENSOR_TYPE_ACCELEROMETER]) != 10) {
			make_command(data, CHANGE_DELAY,
			             SENSOR_TYPE_ACCELEROMETER, uBuf, 9);
			return SUCCESS;
		}
	} else {
		make_command(data, ADD_SENSOR,
		             SENSOR_TYPE_ACCELEROMETER, uBuf, 9);
	}

	return FAIL;
}

void disable_accel_for_cal(struct ssp_data *data, int iDelayChanged)
{
	u8 uBuf[9] = { 0, };
	s32 dMsDelay = get_msdelay(data->delay[SENSOR_TYPE_ACCELEROMETER]);
	memcpy(&uBuf[0], &dMsDelay, 4);

	if (atomic64_read(&data->aSensorEnable) & (1 << SENSOR_TYPE_ACCELEROMETER)) {
		if (iDelayChanged)
			make_command(data, CHANGE_DELAY,
			             SENSOR_TYPE_ACCELEROMETER, uBuf, 9);
	} else {
		make_command(data, REMOVE_SENSOR,
		             SENSOR_TYPE_ACCELEROMETER, uBuf, 4);
	}
}

int accel_do_calibrate(struct ssp_data *data, int iEnable)
{
	int iSum[3] = { 0, };
	int ret = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;

	if (iEnable) {
		int iCount;
		data->accelcal.x = 0;
		data->accelcal.y = 0;
		data->accelcal.z = 0;
		set_accel_cal(data);

		ret = enable_accel_for_cal(data);
		msleep(300);

		for (iCount = 0; iCount < CALIBRATION_DATA_AMOUNT; iCount++) {
			iSum[0] += data->buf[SENSOR_TYPE_ACCELEROMETER].x;
			iSum[1] += data->buf[SENSOR_TYPE_ACCELEROMETER].y;
			iSum[2] += data->buf[SENSOR_TYPE_ACCELEROMETER].z;
			mdelay(10);
		}
		disable_accel_for_cal(data, ret);

		data->accelcal.x = (iSum[0] / CALIBRATION_DATA_AMOUNT);
		data->accelcal.y = (iSum[1] / CALIBRATION_DATA_AMOUNT);
		data->accelcal.z = (iSum[2] / CALIBRATION_DATA_AMOUNT);

		if (data->accelcal.z > 0) {
			data->accelcal.z -= MAX_ACCEL_1G;
		} else if (data->accelcal.z < 0) {
			data->accelcal.z += MAX_ACCEL_1G;
		}
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

	ret = vfs_write(cal_filp, (char *)&data->accelcal, 3 * sizeof(int), &cal_filp->f_pos);
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

ssize_t get_accel_bmi168_calibration(struct ssp_data *data, char *buf)
{
	int ret;

	ret = accel_open_calibration(data);
	if (ret < 0) {
		pr_err("[SSP]: %s - calibration open failed(%d)\n", __func__, ret);
	}

	ssp_info("Cal data : %d %d %d - %d",
	         data->accelcal.x, data->accelcal.y, data->accelcal.z, ret);

	return sprintf(buf, "%d %d %d %d\n", ret, data->accelcal.x,
	               data->accelcal.y, data->accelcal.z);
}

ssize_t set_accel_bmi168_calibration(struct ssp_data *data, const char *buf)
{
	int ret;
	int64_t dEnable;

	ret = kstrtoll(buf, 10, &dEnable);
	if (ret < 0) {
		return ret;
	}

	ret = accel_do_calibrate(data, (int)dEnable);
	if (ret < 0) {
		pr_err("[SSP]: %s - accel_do_calibrate() failed\n", __func__);
	}

	return ret;
}

ssize_t get_accel_bmi168_raw_data(struct ssp_data *data, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
	                data->buf[SENSOR_TYPE_ACCELEROMETER].x,
	                data->buf[SENSOR_TYPE_ACCELEROMETER].y,
	                data->buf[SENSOR_TYPE_ACCELEROMETER].z);
}

ssize_t get_accel_bmi168_reactive_alert(struct ssp_data *data, char *buf)
{
	bool bSuccess = false;

	if (data->is_accel_alert == true) {
		bSuccess = true;
	} else {
		bSuccess = false;
	}

	data->is_accel_alert = false;
	return sprintf(buf, "%u\n", bSuccess);
}


ssize_t set_accel_bmi168_reactive_alert(struct ssp_data *data, const char *buf)
{
	int ret = 0;
	//char chTempBuf = 1;

	//struct ssp_msg *msg;

	if (sysfs_streq(buf, "1")) {
		ssp_infof("on");
	} else if (sysfs_streq(buf, "0")) {
		ssp_infof("off");
	} else if (sysfs_streq(buf, "2")) {
		ssp_infof("factory");

		data->is_accel_alert = 0;
#if 0
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = ACCELEROMETER_FACTORY;
		msg->length = 1;
		msg->options = AP2HUB_READ;
		msg->data = chTempBuf;
		msg->buffer = &chTempBuf;
		msg->free_buffer = 0;

		ret = ssp_spi_sync(data, msg, 3000);
		data->is_accel_alert = chTempBuf;

		if (ret != SUCCESS) {
			pr_err("[SSP]: %s - accel Selftest Timeout!!\n", __func__);
			goto exit;
		}
#endif
		ssp_infof("factory test success!");
	} else {
		pr_err("[SSP]: %s - invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
//exit:
	return ret;
}

ssize_t get_accel_bmi168_selftest(struct ssp_data *data, char *buf)
{
	char chTempBuf[8] = { 2, 0, };
	char init_status = 0;
	char result = -1;
	u16 diff_axis[3] = { 0, };
	//int ret = 0;;
	//struct ssp_data *data = dev_get_drvdata(dev);
	//struct ssp_msg *msg;

#if 0
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = ACCELEROMETER_FACTORY;
	msg->length = sizeof(chTempBuf);
	msg->options = AP2HUB_READ;
	msg->data = chTempBuf[0];
	msg->buffer = chTempBuf;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 3000);
	if (ret != SUCCESS) {
		pr_err("[SSP] %s - accel hw selftest Timeout!!\n", __func__);
		return sprintf(buf, "%d,%d,%d,%d\n", -5, 0, 0, 0);
	}
#endif

	init_status = chTempBuf[0];
	diff_axis[0] = (s16)((chTempBuf[2] << 8) + chTempBuf[1]);
	diff_axis[1] = (s16)((chTempBuf[4] << 8) + chTempBuf[3]);
	diff_axis[2] = (s16)((chTempBuf[6] << 8) + chTempBuf[5]);
	result = chTempBuf[7];

	pr_info("[SSP] %s - %d, %d, %d, %d, %d\n", __func__,
	        init_status, result, diff_axis[0], diff_axis[1], diff_axis[2]);
	return sprintf(buf, "%d,%d,%d,%d\n",
	               result, diff_axis[0], diff_axis[1], diff_axis[2]);
}

ssize_t set_accel_bmi168_lowpassfilter(struct ssp_data *data, const char *buf)
{
	int ret = 0;
	int new_enable = 1;
	//struct ssp_data *data = dev_get_drvdata(dev);
	//struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	if (sysfs_streq(buf, "1")) {
		new_enable = 1;
	} else if (sysfs_streq(buf, "0")) {
		new_enable = 0;
	} else {
		ssp_info(" invalid value!");
	}

#if 0
	msg->cmd = MSG2SSP_AP_SENSOR_LPF;
	msg->length = 1;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(1, GFP_KERNEL);
	if (msg->buffer == NULL) {
		pr_err("[SSP] %s, failed to alloc memory\n", __func__);
		kfree(msg);
		goto exit;
	}

	*msg->buffer = new_enable;
	msg->free_buffer = 1;

	ret = ssp_spi_async(data, msg);
	if (ret != SUCCESS) {
		pr_err("[SSP] %s - fail %d\n", __func__, ret);
	} else {
		pr_info("[SSP] %s - %d\n", __func__, new_enable);
	}
#endif
//exit:
	return ret;
}

struct accelometer_sensor_operations accel_bmi168_ops = {
	.get_accel_name = get_accel_bmi168_name,
	.get_accel_vendor = get_accel_bmi168_vendor,
	.get_accel_calibration = get_accel_bmi168_calibration,
	.set_accel_calibration = set_accel_bmi168_calibration,
	.get_accel_raw_data = get_accel_bmi168_raw_data,
	.get_accel_reactive_alert = get_accel_bmi168_reactive_alert,
	.set_accel_reactive_alert = set_accel_bmi168_reactive_alert,
	.get_accel_selftest = get_accel_bmi168_selftest,
	.set_accel_lowpassfilter = set_accel_bmi168_lowpassfilter,
};

void accelometer_bmi168_function_pointer_initialize(struct ssp_data *data)
{
	data->accel_ops = &accel_bmi168_ops;
}
