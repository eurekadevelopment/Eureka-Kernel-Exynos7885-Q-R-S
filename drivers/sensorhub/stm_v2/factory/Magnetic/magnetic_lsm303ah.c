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
#include "../../ssp.h"
#include "../ssp_factory.h"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define MAG_HW_OFFSET_FILE_PATH "/efs/FactoryApp/hw_offset"
#define LSM303HA_STATIC_ELLIPSOID_MATRIX        {10000, 0, 0, 0, 10000, 0, 0, 0, 10000}

int mag_open_hwoffset(struct ssp_data *data)
{
	int ret = 0;
#if 0
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(MAG_HW_OFFSET_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(cal_filp)) {
		pr_err("[SSP] %s: filp_open failed\n", __func__);
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);

		data->magoffset.x = 0;
		data->magoffset.y = 0;
		data->magoffset.z = 0;

		return iRet;
	}

	iRet = cal_filp->f_op->read(cal_filp, (char *)&data->magoffset,
	                            3 * sizeof(char), &cal_filp->f_pos);
	if (iRet != 3 * sizeof(char)) {
		pr_err("[SSP] %s: filp_open failed\n", __func__);
		iRet = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ssp_dbg("[SSP]: %s: %d, %d, %d\n", __func__,
	        (s8)data->magoffset.x,
	        (s8)data->magoffset.y,
	        (s8)data->magoffset.z);

	if ((data->magoffset.x == 0) && (data->magoffset.y == 0)
	    && (data->magoffset.z == 0)) {
		return ERROR;
	}
#endif
	return ret;
}


int set_static_matrix(struct ssp_data *data)
{
	int ret = SUCCESS;
	struct ssp_msg *msg;
	s16 static_matrix[9] = LSM303HA_STATIC_ELLIPSOID_MATRIX;

	if (!(data->uSensorState & 0x04)) {
		pr_info("[SSP]: %s - Skip this function!!!"\
		        ", magnetic sensor is not connected(0x%llx)\n",
		        __func__, data->uSensorState);
		return ret;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n", __func__);
		return -ENOMEM;
	}
	msg->cmd = MSG2SSP_AP_SET_MAGNETIC_STATIC_MATRIX;
	msg->length = 18;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char *) kzalloc(18, GFP_KERNEL);

	msg->free_buffer = 1;
	if (data->static_matrix == NULL) {
		memcpy(msg->buffer, static_matrix, 18);
	} else {
		memcpy(msg->buffer, data->static_matrix, 18);
	}

	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, ret);
		ret = ERROR;
	}
	pr_info("[SSP]: %s: finished \n", __func__);

	return ret;
}

#if 0 // sangmin, it needs 
int check_raw_data_spec(struct ssp_data *data)
{
	if ((data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x == 0) &&
	    (data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y == 0) &&
	    (data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z == 0)) {
		return FAIL;
	} else {
		return SUCCESS;
	}
}

int check_adc_data_spec(struct ssp_data *data)
{
	if ((data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].x == 0) &&
	    (data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].y == 0) &&
	    (data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].z == 0)) {
		return FAIL;
	} else if ((data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].x > 16383) ||
	           (data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].x < -16383) ||
	           (data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].y > 16383) ||
	           (data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].y < -16383) ||
	           (data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].z > 16383) ||
	           (data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].z < -16383)) {
		return FAIL;
	} else {
		return SUCCESS;
	}
}
#endif

ssize_t get_magnetic_lsm303ah_name(char *buf)
{
	return sprintf(buf, "%s\n", "LSM303HA");
}


ssize_t get_magnetic_lsm303ah_vendor(char *buf)
{
	return sprintf(buf, "%s\n", "STM");
}


ssize_t get_magnetic_lsm303ah_adc(struct ssp_data *data, char *buf)
{
	bool bSuccess = false;
	u8 chTempbuf[4] = { 0 };
	s16 sensor_buf[3] = {0, };
	//int retries = 10;
	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);

	data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].x = 0;
	data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].y = 0;
	data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].z = 0;

#if 0
	if (!(atomic64_read(&data->aSensorEnable) & (1 <<
	                                             SENSOR_TYPE_GEOMAGNETIC_FIELD)))
		make_command(data, ADD_SENSOR, SENSOR_TYPE_GEOMAGNETIC_FIELD,
		             chTempbuf, 4);

	do {
		msleep(60);
		if (check_adc_data_spec(data) == SUCCESS) {
			break;
		}
	} while (--retries);


	if (retries > 0) {
		bSuccess = true;
	}

	sensor_buf[0] = data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].x;
	sensor_buf[1] = data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].y;
	sensor_buf[2] = data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].z;

	if (!(atomic64_read(&data->aSensorEnable) & (1 <<
	                                             SENSOR_TYPE_GEOMAGNETIC_FIELD)))
		make_command(data, REMOVE_SENSOR, SENSOR_TYPE_GEOMAGNETIC_FIELD,
		             chTempbuf, 4);
#endif

	pr_info("[SSP]: %s - x = %d, y = %d, z = %d\n", __func__,
	        sensor_buf[0], sensor_buf[1], sensor_buf[2]);

	return sprintf(buf, "%s,%d,%d,%d\n", (bSuccess ? "OK" : "NG"),
	               sensor_buf[0], sensor_buf[1], sensor_buf[2]);
}

ssize_t get_magnetic_lsm303ah_raw_data(struct ssp_data *data, char *buf)
{
	pr_info("[SSP] %s - %d,%d,%d\n", __func__,
	        data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x,
	        data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y,
	        data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z);

	if (data->bGeomagneticRawEnabled == false) {
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x = -1;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y = -1;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z = -1;
	}

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
	                data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x,
	                data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y,
	                data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z);
}

ssize_t set_magnetic_lsm303ah_raw_data(struct ssp_data *data, const char *buf)
{
	char chTempbuf[4] = { 0 };
	int ret = 0;
	int64_t dEnable;
	//struct ssp_data *data = dev_get_drvdata(dev);
	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);

	ret = kstrtoll(buf, 10, &dEnable);
	if (ret < 0) {
		return ret;
	}
#if 0
	if (dEnable) {
		int iRetries = 50;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x = 0;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y = 0;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z = 0;

		make_command(data, ADD_SENSOR, SENSOR_TYPE_GEOMAGNETIC_POWER,
		             chTempbuf, 4);

		do {
			msleep(20);
			if (check_raw_data_spec(data) == SUCCESS) {
				break;
			}
		} while (--iRetries);

		if (iRetries > 0) {
			pr_info("[SSP] %s - success, %d\n", __func__, iRetries);
			data->bGeomagneticRawEnabled = true;
		} else {
			pr_err("[SSP] %s - wait timeout, %d\n", __func__,
			       iRetries);
			data->bGeomagneticRawEnabled = false;
		}
	} else {
		make_command(data, REMOVE_SENSOR, SENSOR_TYPE_GEOMAGNETIC_POWER,
		             chTempbuf, 4);
		data->bGeomagneticRawEnabled = false;
	}
#endif

	return ret;
}

ssize_t get_magnetic_lsm303ah_matrix(struct ssp_data *data, char *buf)
{
	return sprintf(buf,
	               "%d %d %d %d %d %d %d %d %d\n", data->static_matrix[0], data->static_matrix[1],
	               data->static_matrix[2]
	               , data->static_matrix[3], data->static_matrix[4], data->static_matrix[5]
	               , data->static_matrix[6], data->static_matrix[7], data->static_matrix[8]);
}

ssize_t set_magnetic_lsm303ah_matrix(struct ssp_data *data, const char *buf)
{
	int ret = 0;
	int i;
	s16 val[9] = {0, };
	char *token;
	char *str;

	str = (char *)buf;

	for (i = 0; i < 9; i++) {
		token = strsep(&str, " \n");
		if (token == NULL) {
			pr_err("[SSP] %s : too few arguments (9 needed)", __func__);
			return -EINVAL;
		}

		ret = kstrtos16(token, 10, &val[i]);
		if (ret < 0) {
			pr_err("[SSP] %s : kstros16 error %d", __func__, ret);
			return ret;
		}
	}

	for (i = 0; i < 9; i++) {
		data->static_matrix[i] = val[i];
	}

	pr_info("[SSP] %s : %d %d %d %d %d %d %d %d %d\n", __func__,
	        data->static_matrix[0], data->static_matrix[1], data->static_matrix[2]
	        , data->static_matrix[3], data->static_matrix[4], data->static_matrix[5]
	        , data->static_matrix[6], data->static_matrix[7], data->static_matrix[8]);

	set_static_matrix(data);

	return ret;
}

ssize_t get_magnetic_lsm303ah_hw_offset(struct ssp_data *data, char *buf)
{
#if 0
	struct ssp_data *data = dev_get_drvdata(dev);

	mag_open_hwoffset(data);

	pr_info("[SSP] %s: %d %d %d\n", __func__,
	        (s8)data->magoffset.x,
	        (s8)data->magoffset.y,
	        (s8)data->magoffset.z);

	return sprintf(buf, "%d %d %d\n",
	               (s8)data->magoffset.x,
	               (s8)data->magoffset.y,
	               (s8)data->magoffset.z);
#endif
	return sprintf(buf, "%d", 1);
}

ssize_t get_magnetic_lsm303ah_selftest(struct ssp_data *data, char *buf)
{
	char chTempBuf[12] = { 0,  };
	//int ret = 0;
	s8 id = 0;
	s16 x_diff = 0, y_diff = 0, z_diff = 0;
	s16 diff_max = 0, diff_min = 0;
	s8 result = 0;
	s8 err[7] = {0, };
	//struct ssp_data *data = dev_get_drvdata(dev);

#if 0
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	msg->cmd = GEOMAGNETIC_FACTORY;
	msg->length = 12;
	msg->options = AP2HUB_READ;
	msg->buffer = chTempBuf;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - Magnetic Selftest Timeout!! %d\n", __func__, ret);
		goto exit;
	}
#endif

	id = (s8)(chTempBuf[0]);
	x_diff = ((s16)(chTempBuf[2] << 8)) + chTempBuf[1];
	y_diff = ((s16)(chTempBuf[4] << 8)) + chTempBuf[3];
	z_diff = ((s16)(chTempBuf[6] << 8)) + chTempBuf[5];
	diff_max = ((s16)(chTempBuf[8] << 8)) + chTempBuf[7];
	diff_min = ((s16)(chTempBuf[10] << 8)) + chTempBuf[9];
	result = chTempBuf[10];

	if (id != 0x1) {
		err[0] = -1;
	}
	if (x_diff < diff_min || x_diff > diff_max) {
		err[1] = -1;
	}
	if (y_diff < diff_min || y_diff > diff_max) {
		err[2] = -1;
	}
	if (z_diff < diff_min || z_diff > diff_max) {
		err[3] = -1;
	}
	if (result == 0) {
		err[4] = -1;
	}

	pr_info("[SSP] %s\n"
	        "[SSP] Test1 - err = %d, id = %d \n"
	        "[SSP] Test2 - err = %d, x_diff = %d \n"
	        "[SSP] Test3 - err = %d, y_diff = %d \n"
	        "[SSP] Test4 - err = %d, z_diff = %d \n"
	        "[SSP] Test5 - err = %d, result = %d d\n",
	        __func__, err[0], id, err[1], x_diff, err[2], y_diff, err[3], z_diff, err[4],
	        result);

//exit:
	return sprintf(buf,
	               "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d \n",
	               err[0], id, err[1], x_diff, err[2], y_diff, err[3], z_diff, err[4], result);
}

int initialize_magnetic_sensor(struct ssp_data *data)
{
	int ret = 0;

	ret = set_static_matrix(data);
	if (ret < 0)
		pr_err("[SSP]: %s - set_magnetic_static_matrix failed %d\n",
		       __func__, ret);

	return ret < 0 ? ret : SUCCESS;
}

struct magnetic_sensor_operations magnetic_lsm303ah_ops = {
	.get_magnetic_name = get_magnetic_lsm303ah_name,
	.get_magnetic_vendor = get_magnetic_lsm303ah_vendor,
	.get_magnetic_adc = get_magnetic_lsm303ah_adc,
	.get_magnetic_raw_data = get_magnetic_lsm303ah_raw_data,
	.set_magnetic_raw_data = set_magnetic_lsm303ah_raw_data,
	.get_magnetic_matrix = get_magnetic_lsm303ah_matrix,
	.set_magnetic_matrix = set_magnetic_lsm303ah_matrix,
	.get_magnetic_hw_offset = get_magnetic_lsm303ah_hw_offset,
	.get_magnetic_selftest = get_magnetic_lsm303ah_selftest,
};

void magnetic_lsm303ah_function_pointer_initialize(struct ssp_data *data)
{
	data->magnetic_ops = &magnetic_lsm303ah_ops;
}

