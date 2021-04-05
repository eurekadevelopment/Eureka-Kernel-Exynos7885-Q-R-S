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

#define GM_YAS_DATA_SPEC_MIN	-6500
#define GM_YAS_DATA_SPEC_MAX	6500

#define GM_SELFTEST_X_SPEC_MIN	-200
#define GM_SELFTEST_X_SPEC_MAX	200
#define GM_SELFTEST_Y_SPEC_MIN	-200
#define GM_SELFTEST_Y_SPEC_MAX	200
#define GM_SELFTEST_Z_SPEC_MIN	-1000
#define GM_SELFTEST_Z_SPEC_MAX	-200

#define YAS_STATIC_ELLIPSOID_MATRIX 	{10000, 0, 0, 0, 10000, 0, 0, 0, 10000}
#define MAG_HW_OFFSET_FILE_PATH	"/efs/FactoryApp/hw_offset"


int check_adc_data_spec(struct ssp_data *data, int sensortype)
{
	int data_spec_max = 0;
	int data_spec_min = 0;

    data_spec_max = GM_YAS_DATA_SPEC_MAX;
    data_spec_min = GM_YAS_DATA_SPEC_MIN;

	
	if ((data->buf[sensortype].x == 0) &&
		(data->buf[sensortype].y == 0) &&
		(data->buf[sensortype].z == 0))
		return FAIL;
	else if ((data->buf[sensortype].x > data_spec_max)
		|| (data->buf[sensortype].x < data_spec_min)
		|| (data->buf[sensortype].y > data_spec_max)
		|| (data->buf[sensortype].y < data_spec_min)
		|| (data->buf[sensortype].z > data_spec_max)
		|| (data->buf[sensortype].z < data_spec_min))
		return FAIL;
	else
		return SUCCESS;
}


int set_static_matrix(struct ssp_data *data)
{
	int ret = SUCCESS;
	struct ssp_msg *msg;
	s16 static_matrix[9] = YAS_STATIC_ELLIPSOID_MATRIX;

	if (!(data->uSensorState & 0x04)) {
		pr_info("[SSP]: %s - Skip this function!!!"\
			", magnetic sensor is not connected(0x%llx)\n",
			__func__, data->uSensorState);
		return ret;
	}

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	msg->cmd = MSG2SSP_AP_SET_MAGNETIC_STATIC_MATRIX;
	msg->length = 18;
	msg->options = AP2HUB_WRITE;

	if (data->static_matrix == NULL)
		msg->buffer = (char *)static_matrix;
	else
		msg->buffer = (char *)data->static_matrix;

	msg->free_buffer = 0;

	if (data->static_matrix == NULL)
		memcpy(msg->buffer, static_matrix, 18);
	else
		memcpy(msg->buffer, data->static_matrix, 18);


	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, ret);
		ret = ERROR;
	}
	pr_info("[SSP]: %s: finished \n", __func__);

	return ret;
}

int mag_open_hwoffset(struct ssp_data *data)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(MAG_HW_OFFSET_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(cal_filp)) {
		pr_err("[SSP] %s: filp_open failed\n", __func__);
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);

		data->magoffset.x = 0;
		data->magoffset.y = 0;
		data->magoffset.z = 0;

		return ret;
	}

	ret = vfs_read(cal_filp, (char *)&data->magoffset,
		3 * sizeof(char), &cal_filp->f_pos);
	if (ret != 3 * sizeof(char)) {
		pr_err("[SSP] %s: filp_open failed\n", __func__);
		ret = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ssp_dbg("[SSP]: %s: %d, %d, %d\n", __func__,
		(s8)data->magoffset.x,
		(s8)data->magoffset.y,
		(s8)data->magoffset.z);

	if ((data->magoffset.x == 0) && (data->magoffset.y == 0)
		&& (data->magoffset.z == 0))
		return ERROR;

	return ret;
}


int get_hw_offset(struct ssp_data *data)
{
	int ret = 0;
	char buffer[3] = { 0, };

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	msg->cmd = MSG2SSP_AP_GET_MAGNETIC_HWOFFSET;
	msg->length = 3;
	msg->options = AP2HUB_READ;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	data->magoffset.x = 0;
	data->magoffset.y = 0;
	data->magoffset.z = 0;

	ret = ssp_spi_sync(data, msg, 1000);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, ret);
		ret = ERROR;
	}

	data->magoffset.x = buffer[0];
	data->magoffset.y = buffer[1];
	data->magoffset.z = buffer[2];

	pr_info("[SSP]: %s: x: %d, y: %d, z: %d\n", __func__,
		(s8)data->magoffset.x,
		(s8)data->magoffset.y,
		(s8)data->magoffset.z);
	return ret;
}

int set_hw_offset(struct ssp_data *data)
{
	int ret = 0;
	struct ssp_msg *msg;
	char magoffset[3] = {0, };

	if (!(data->uSensorState & 0x04)) {
		pr_info("[SSP]: %s - Skip this function!!!"\
			", magnetic sensor is not connected(0x%llx)\n",
			__func__, data->uSensorState);
		return ret;
	}

	magoffset[0] = data->magoffset.x;
	magoffset[1] = data->magoffset.y;
	magoffset[2] = data->magoffset.z;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	msg->cmd = MSG2SSP_AP_SET_MAGNETIC_HWOFFSET;
	msg->length = 3;
	msg->options = AP2HUB_WRITE;
	msg->buffer = magoffset;
	msg->free_buffer = 0;

	ret = ssp_spi_async(data, msg);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, ret);
		ret = ERROR;
	}

	pr_info("[SSP]: %s: x: %d, y: %d, z: %d\n", __func__,
		(s8)data->magoffset.x, (s8)data->magoffset.y, (s8)data->magoffset.z);
	return ret;
}

int mag_store_hwoffset(struct ssp_data *data)
{
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;

	if (get_hw_offset(data) < 0) {
		pr_err("[SSP]: %s - get_hw_offset failed\n", __func__);
		return ERROR;
	} else {
		int ret = 0;
	
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		cal_filp = filp_open(MAG_HW_OFFSET_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0660);
		if (IS_ERR(cal_filp)) {
			pr_err("[SSP]: %s - Can't open hw_offset file\n",
				__func__);
			set_fs(old_fs);
			ret = PTR_ERR(cal_filp);
			return ret;
		}
		ret = vfs_write(cal_filp,
			(char *)&data->magoffset,
			3 * sizeof(char), &cal_filp->f_pos);
		if (ret != 3 * sizeof(char)) {
			pr_err("[SSP]: %s - Can't write the hw_offset"
				" to file\n", __func__);
			ret = -EIO;
		}
		filp_close(cal_filp, current->files);
		set_fs(old_fs);
		return ret;
	}
}



ssize_t get_magnetic_yas539_name(char *buf)
{
	return sprintf(buf, "%s\n", "YAS539");
}

ssize_t get_magnetic_yas539_vendor(char *buf)
{
	return sprintf(buf, "%s\n", "YAMAHA");
}

ssize_t get_magnetic_yas539_adc(struct ssp_data *data, char *buf)
{
	bool bSuccess = false;
	u8 chTempbuf[4] = { 0 };
	s16 sensor_buf[3] = {0, };
	int retries = 10;
	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);

	data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].x = 0;
	data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].y = 0;
	data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].z = 0;


	if (!(atomic64_read(&data->aSensorEnable) & (1 << SENSOR_TYPE_GEOMAGNETIC_FIELD)))
		send_instruction(data, ADD_SENSOR, SENSOR_TYPE_GEOMAGNETIC_FIELD,
			chTempbuf, 4);

	do {
		msleep(60);
		if (check_adc_data_spec(data, SENSOR_TYPE_GEOMAGNETIC_FIELD) == SUCCESS)
			break;
	} while (--retries);

	if (retries > 0)
		bSuccess = true;

	sensor_buf[0] = data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].x;
	sensor_buf[1] = data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].y;
	sensor_buf[2] = data->buf[SENSOR_TYPE_GEOMAGNETIC_FIELD].z;

	if (!(atomic64_read(&data->aSensorEnable) & (1 << SENSOR_TYPE_GEOMAGNETIC_FIELD)))
		send_instruction(data, REMOVE_SENSOR, SENSOR_TYPE_GEOMAGNETIC_FIELD,
			chTempbuf, 4);


	pr_info("[SSP] %s - x = %d, y = %d, z = %d\n", __func__,
		sensor_buf[0], sensor_buf[1], sensor_buf[2]);

	return sprintf(buf, "%s,%d,%d,%d\n", (bSuccess ? "OK" : "NG"),
		sensor_buf[0], sensor_buf[1], sensor_buf[2]);
}

ssize_t get_magnetic_yas539_dac(struct ssp_data *data, char *strbuf)
{
	bool bSuccess = false;
	char chTempBuf[22] = { 0,  };


	if (!data->uMagCntlRegData) {
		bSuccess = true;
	} else {
		int ret;
		struct ssp_msg *msg;
		
		pr_info("[SSP] %s - check cntl register before selftest",
			__func__);
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = GEOMAGNETIC_FACTORY;
		msg->length = 22;
		msg->options = AP2HUB_READ;
		msg->buffer = chTempBuf;
		msg->free_buffer = 0;

		ret = ssp_spi_sync(data, msg, 1000);

		if (ret != SUCCESS) {
			pr_err("[SSP] %s - spi sync failed due to Timeout!! %d\n",
					__func__, ret);
		}

		data->uMagCntlRegData = chTempBuf[21];
		bSuccess = !data->uMagCntlRegData;
	}


	pr_info("[SSP] %s - CTRL : 0x%x\n", __func__,
				data->uMagCntlRegData);

	data->uMagCntlRegData = 1;	/* reset the value */

	return sprintf(strbuf, "%s,%d,%d,%d\n",
		(bSuccess ? "OK" : "NG"), (bSuccess ? 1 : 0), 0, 0);
}

ssize_t get_magnetic_yas539_raw_data(struct ssp_data *data, char *buf)
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

ssize_t set_magnetic_yas539_raw_data(struct ssp_data *data, const char *buf)
{
	char chTempbuf[4] = { 0 };
	int ret;
	int64_t dEnable;
	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);

	ret = kstrtoll(buf, 10, &dEnable);
	if (ret < 0)
		return ret;

	if (dEnable) {
		int retries = 50;
		
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x = 0;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y = 0;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z = 0;

		send_instruction(data, ADD_SENSOR, SENSOR_TYPE_GEOMAGNETIC_POWER,
			chTempbuf, 4);

		do {
			msleep(20);
			if (check_adc_data_spec(data, SENSOR_TYPE_GEOMAGNETIC_POWER) == SUCCESS)
				break;
		} while (--retries);

		if (retries > 0)
		{
			pr_info("[SSP] %s - success, %d\n", __func__, retries);
            data->bGeomagneticRawEnabled = true;
		}
		else
		{
			pr_err("[SSP] %s - wait timeout, %d\n", __func__,
				retries);
            data->bGeomagneticRawEnabled = false;
		}


	} else {
		send_instruction(data, REMOVE_SENSOR, SENSOR_TYPE_GEOMAGNETIC_POWER,
			chTempbuf, 4);
		data->bGeomagneticRawEnabled = false;
	}

	return ret;
}


ssize_t get_magnetic_yas539_status(struct ssp_data *data, char *buf)
{
	bool bSuccess;

	if ((data->uFuseRomData[0] == 0) ||
		(data->uFuseRomData[0] == 0xff) ||
		(data->uFuseRomData[1] == 0) ||
		(data->uFuseRomData[1] == 0xff) ||
		(data->uFuseRomData[2] == 0) ||
		(data->uFuseRomData[2] == 0xff))
		bSuccess = false;
	else
		bSuccess = true;

	return sprintf(buf, "%s,%u\n", (bSuccess ? "OK" : "NG"), bSuccess);
}


ssize_t get_magnetic_yas539_logging_data(struct ssp_data *data,  char *buf)
{
	char buffer[21] = {0, };
	int ret = 0;
	int logging_data[8] = {0, };
	struct ssp_msg *msg;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	msg->cmd = MSG2SSP_AP_GEOMAG_LOGGING;
	msg->length = 21;
	msg->options = AP2HUB_READ;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);
	if (ret != SUCCESS) {
		pr_err("[SSP] %s - Magnetic logging data Timeout!! %d\n",
			__func__, ret);
		goto exit;
	}

	logging_data[0] = buffer[0];	/* ST1 Reg */
	logging_data[1] = (short)((buffer[3] << 8) + buffer[2]);
	logging_data[2] = (short)((buffer[5] << 8) + buffer[4]);
	logging_data[3] = (short)((buffer[7] << 8) + buffer[6]);
	logging_data[4] = buffer[1];	/* ST2 Reg */
	logging_data[5] = (short)((buffer[9] << 8) + buffer[8]);
	logging_data[6] = (short)((buffer[11] << 8) + buffer[10]);
	logging_data[7] = (short)((buffer[13] << 8) + buffer[12]);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			logging_data[0], logging_data[1],
			logging_data[2], logging_data[3],
			logging_data[4], logging_data[5],
			logging_data[6], logging_data[7],
			data->uFuseRomData[0], data->uFuseRomData[1],
			data->uFuseRomData[2]);

exit:
	return snprintf(buf, PAGE_SIZE, "-1,0,0,0,0,0,0,0,0,0,0\n");
}

ssize_t get_magnetic_yas539_matrix(struct ssp_data *data, char *buf)
{
	return sprintf(buf,
		"%d %d %d %d %d %d %d %d %d\n", data->static_matrix[0], data->static_matrix[1], data->static_matrix[2]
		, data->static_matrix[3], data->static_matrix[4], data->static_matrix[5]
		, data->static_matrix[6], data->static_matrix[7], data->static_matrix[8]);

}

ssize_t set_magnetic_yas539_matrix(struct ssp_data *data, const char *buf)
{
	s16 val[9] = {0, };
 	int ret;
 	int i;
	char* token;
	char* str;
	str = (char*)buf;

	for(i = 0; i < 9; i++)
 	{
		token = strsep(&str, " \n");
		if(token == NULL)
		{
			pr_err("[SSP] %s : too few arguments (9 needed)",__func__);
 			return -EINVAL;
		}

		ret = kstrtos16(token, 10, &val[i]);
		if (ret<0) {
 			pr_err("[SSP] %s : kstros16 error %d",__func__,ret);
 			return ret;
 		}
 	}		
	
	for(i = 0; i < 9; i++)
		data->static_matrix[i] = val[i];

	pr_info("[SSP] %s : %d %d %d %d %d %d %d %d %d\n",__func__,data->static_matrix[0], data->static_matrix[1], data->static_matrix[2]
		, data->static_matrix[3], data->static_matrix[4], data->static_matrix[5]
		, data->static_matrix[6], data->static_matrix[7], data->static_matrix[8]);
	
	set_static_matrix(data);
	
	return ret;
}


ssize_t get_magnetic_yas539_hw_offset(struct ssp_data *data, char *buf)
{
	mag_open_hwoffset(data);

	pr_info("[SSP] %s: %d %d %d\n", __func__,
		(s8)data->magoffset.x,
		(s8)data->magoffset.y,
		(s8)data->magoffset.z);

	return sprintf(buf, "%d %d %d\n",
		(s8)data->magoffset.x,
		(s8)data->magoffset.y,
		(s8)data->magoffset.z);
}

ssize_t get_magnetic_yas539_selftest(struct ssp_data *data, char *buf)
{
	char chTempBuf[24] = { 0,  };
	int ret = 0;
	s8 id = 0, x = 0, y1 = 0, y2 = 0, dir = 0;
	s16 sx = 0, sy1 = 0, sy2 = 0, ohx = 0, ohy = 0, ohz = 0;
	s8 err[7] = {-1, };

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);

	msg->cmd = GEOMAGNETIC_FACTORY;
	msg->length = 24;
	msg->options = AP2HUB_READ;
	msg->buffer = chTempBuf;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - Magnetic Selftest Timeout!! %d\n", __func__, ret);
		goto exit;
	}

	id = (s8)(chTempBuf[0]);
	err[0] = (s8)(chTempBuf[1]);
	err[1] = (s8)(chTempBuf[2]);
	err[2] = (s8)(chTempBuf[3]);
	x = (s8)(chTempBuf[4]);
	y1 = (s8)(chTempBuf[5]);
	y2 = (s8)(chTempBuf[6]);
	err[3] = (s8)(chTempBuf[7]);
	dir = (s8)(chTempBuf[8]);
	err[4] = (s8)(chTempBuf[9]);
	ohx = (s16)((chTempBuf[10] << 8) + chTempBuf[11]);
	ohy = (s16)((chTempBuf[12] << 8) + chTempBuf[13]);
	ohz = (s16)((chTempBuf[14] << 8) + chTempBuf[15]);
	err[6] = (s8)(chTempBuf[16]);
	sx = (s16)((chTempBuf[17] << 8) + chTempBuf[18]);
	sy1 = (s16)((chTempBuf[19] << 8) + chTempBuf[20]);
	sy2 = (s16)((chTempBuf[21] << 8) + chTempBuf[22]);
	err[5] = (s8)(chTempBuf[23]);

	if (unlikely(id != 0x8))
		err[0] = -1;
	if (unlikely(x < -30 || x > 30))
		err[3] = -1;
	if (unlikely(y1 < -30 || y1 > 30))
		err[3] = -1;
	if (unlikely(y2 < -30 || y2 > 30))
		err[3] = -1;
	if (unlikely(sx < 16544 || sx > 17024))
		err[5] = -1;
	if (unlikely(sy1 < 16517 || sy1 > 17184))
		err[5] = -1;
	if (unlikely(sy2 < 15584 || sy2 > 16251))
		err[5] = -1;
	if (unlikely(ohx < -1000 || ohx > 1000))
		err[6] = -1;
	if (unlikely(ohy < -1000 || ohy > 1000))
		err[6] = -1;
	if (unlikely(ohz < -1000 || ohz > 1000))
		err[6] = -1;

	pr_info("[SSP] %s\n"
		"[SSP] Test1 - err = %d, id = %d\n"
		"[SSP] Test3 - err = %d\n"
		"[SSP] Test4 - err = %d, offset = %d,%d,%d\n"
		"[SSP] Test5 - err = %d, direction = %d\n"
		"[SSP] Test6 - err = %d, sensitivity = %d,%d,%d\n"
		"[SSP] Test7 - err = %d, offset = %d,%d,%d\n"
		"[SSP] Test2 - err = %d\n",
		__func__, err[0], id, err[2], err[3], x, y1, y2, err[4], dir,
		err[5], sx, sy1, sy2, err[6], ohx, ohy, ohz, err[1]);

exit:
	return sprintf(buf,
			"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			err[0], id, err[2], err[3], x, y1, y2, err[4], dir,
			err[5], sx, sy1, sy2, err[6], ohx, ohy, ohz, err[1]);

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

struct magnetic_sensor_operations magnetic_yas539_ops = {
	.get_magnetic_name = get_magnetic_yas539_name,
	.get_magnetic_vendor = get_magnetic_yas539_vendor,
	.get_magnetic_adc = get_magnetic_yas539_adc,
	.get_magnetic_dac = get_magnetic_yas539_dac,
	.get_magnetic_raw_data = get_magnetic_yas539_raw_data,
	.set_magnetic_raw_data = set_magnetic_yas539_raw_data,
	.get_magnetic_status = get_magnetic_yas539_status,
	.get_magnetic_logging_data = get_magnetic_yas539_logging_data,
	.get_magnetic_matrix = get_magnetic_yas539_matrix, 
	.set_magnetic_matrix = set_magnetic_yas539_matrix,
	.get_magnetic_hw_offset = get_magnetic_yas539_hw_offset,
	.get_magnetic_selftest = get_magnetic_yas539_selftest,
};

void magnetic_yas539_function_pointer_initialize(struct ssp_data *data)
{
	data->magnetic_ops = &magnetic_yas539_ops;
}

