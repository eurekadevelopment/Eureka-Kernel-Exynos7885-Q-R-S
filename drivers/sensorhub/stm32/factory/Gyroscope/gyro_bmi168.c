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
#include "../../ssp.h"
#include "../ssp_factory.h"

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

#define CALIBRATION_FILE_PATH              "/efs/FactoryApp/gyro_cal_data"
//#define CALIBRATION_DATA_AMOUNT            20
//#define SELFTEST_DATA_AMOUNT               64
#define SELFTEST_LIMITATION_OF_ERROR       5250

ssize_t get_gyro_bmi168_name(char *buf)
{
	return sprintf(buf, "%s\n", "BMI168");
}

ssize_t get_gyro_bmi168_vendor(char *buf)
{
	return sprintf(buf, "%s\n", "BOSCH");
}

int gyro_open_calibration(struct ssp_data *data)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cal_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY | O_NOFOLLOW, 0660);
	if (IS_ERR(cal_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(cal_filp);

		data->gyrocal.x = 0;
		data->gyrocal.y = 0;
		data->gyrocal.z = 0;

		return ret;
	}

	ret = vfs_read(cal_filp, (char *)&data->gyrocal,
		sizeof(data->gyrocal), &cal_filp->f_pos);
	if (ret != sizeof(data->gyrocal))
		ret = -EIO;

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	ssp_info("open gyro calibration %d, %d, %d",
		data->gyrocal.x, data->gyrocal.y, data->gyrocal.z);
	return ret;
}


int save_gyro_caldata(struct ssp_data *data, s16 *iCalData)
{
	int iRet = 0;
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;

	if (data->bSspShutdown)
		return -EIO;

	data->gyrocal.x = iCalData[0];
	data->gyrocal.y = iCalData[1];
	data->gyrocal.z = iCalData[2];

	ssp_info("do gyro calibrate %d, %d, %d",
		data->gyrocal.x, data->gyrocal.y, data->gyrocal.z);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY | O_NOFOLLOW, 0660);
	if (IS_ERR(cal_filp)) {
		pr_err("[SSP]: %s - Can't open calibration file\n", __func__);
		set_fs(old_fs);
		iRet = PTR_ERR(cal_filp);
		return -EIO;
	}

	iRet = vfs_write(cal_filp, (char *)&data->gyrocal,
		sizeof(data->gyrocal), &cal_filp->f_pos);
	if (iRet != sizeof(data->gyrocal)) {
		pr_err("[SSP]: %s - Can't write gyro cal to file\n", __func__);
		iRet = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return iRet;
}


int set_gyro_cal(struct ssp_data *data)
{
	int ret = 0;
	struct ssp_msg *msg;
	s16 gyro_cal[3];
	if (!(data->uSensorState & (1 << SENSOR_TYPE_GYROSCOPE))) {
		pr_info("[SSP]: %s - Skip this function!!!"\
			", gyro sensor is not connected(0x%llx)\n",
			__func__, data->uSensorState);
		return ret;
	}

	gyro_cal[0] = data->gyrocal.x;
	gyro_cal[1] = data->gyrocal.y;
	gyro_cal[2] = data->gyrocal.z;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_AP_MCU_SET_GYRO_CAL;
	msg->length = 6;
	msg->options = AP2HUB_WRITE;
	msg->buffer = (char*) kzalloc(6, GFP_KERNEL);

	msg->free_buffer = 1;
	memcpy(msg->buffer, gyro_cal, 6);

	iRet = ssp_spi_async(data, msg);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - i2c fail %d\n", __func__, iRet);
		iRet = ERROR;
	}

	pr_info("[SSP] Set gyro cal data %d, %d, %d\n", gyro_cal[0], gyro_cal[1], gyro_cal[2]);
	return ret;
}

ssize_t get_gyro_bmi168_power_off(char *buf)
{
	ssp_infof();

	return sprintf(buf, "%d\n", 1);
}

ssize_t get_gyro_bmi168_power_on(char *buf)
{
	ssp_infof();

	return sprintf(buf, "%d\n", 1);
}

ssize_t get_gyro_bmi168_temperature(struct ssp_data *data, char *buf)
{
	char chTempBuf[2] = { 0};
	unsigned char reg[2];
	short temperature = 0;
	int ret = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = GYROSCOPE_TEMP_FACTORY;
	msg->length = 2;
	msg->options = AP2HUB_READ;
	msg->buffer = chTempBuf;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 3000);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Gyro Temp Timeout!!\n", __func__);
		goto exit;
	}

	reg[0] = chTempBuf[1];
	reg[1] = chTempBuf[0];
	temperature = (short) (((reg[0]) << 8) | reg[1]);
	ssp_infof("%d", temperature);

exit:
	return sprintf(buf, "%d\n", temperature);
}

ssize_t get_gyro_bmi168_selftest(struct ssp_data *data, char *buf)
{
	char chTempBuf[19] = {0, };
	u8 bist=0, selftest = 0;
	int datax_check = 0;
	int datay_check = 0;
	int dataz_check = 0;
	s16 cal_data[3] = {0, };
	int ret = 0;

	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->cmd = GYROSCOPE_FACTORY;
	msg->length = 19;
	msg->options = AP2HUB_READ;
	msg->buffer = chTempBuf;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 3000);
	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Gyro Selftest Timeout!!\n", __func__);
		selftest = 1;
		goto exit;
	}

	pr_info("[SSP]: %s - %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
		__func__, chTempBuf[0], chTempBuf[1], chTempBuf[2],
		chTempBuf[3], chTempBuf[4], chTempBuf[5], chTempBuf[6],
		chTempBuf[7], chTempBuf[8], chTempBuf[9], chTempBuf[10],
		chTempBuf[11], chTempBuf[12]);

	data->cnt_timeout = 0;

	/* 1: X axis fail, 2: X axis fail, 4: X axis fail, 8: Bist fail*/
	selftest = chTempBuf[0];
	if (selftest == 0)
		bist = 1;
	else
		bist =0;

	datax_check = (int)((chTempBuf[4] << 24) + (chTempBuf[3] << 16)
		+(chTempBuf[2] << 8) + chTempBuf[1]);
	datay_check = (int)((chTempBuf[8] << 24) + (chTempBuf[7] << 16)
		+(chTempBuf[6] << 8) + chTempBuf[5]);
	dataz_check = (int)((chTempBuf[12] << 24) + (chTempBuf[11] << 16)
		+(chTempBuf[10] << 8) + chTempBuf[9]);

	cal_data[0] = (s16)((chTempBuf[14] << 8) + chTempBuf[13]);
	cal_data[1] = (s16)((chTempBuf[16] << 8) + chTempBuf[15]);
	cal_data[2] = (s16)((chTempBuf[18] << 8) + chTempBuf[17]);

	pr_info("[SSP]: %s - bist: %d, selftest: %d\n",
		__func__, bist, selftest);
	pr_info("[SSP]: %s - X: %d, Y: %d, Z: %d\n",
		__func__, datax_check, datay_check, dataz_check);
	pr_info("[SSP]: %s - CalData X: %d, Y: %d, Z: %d\n",
		__func__, cal_data[0], cal_data[1], cal_data[2]);

	if ((datax_check <= SELFTEST_LIMITATION_OF_ERROR)
		&& (datay_check <= SELFTEST_LIMITATION_OF_ERROR)
		&& (dataz_check <= SELFTEST_LIMITATION_OF_ERROR)) {
		pr_info("[SSP]: %s - Gyro zero rate OK!- Gyro selftest Pass\n",
			__func__);
		/* save_gyro_caldata(data, iCalData); */
	} else {
		pr_info("[SSP]: %s - Gyro zero rate NG!- Gyro selftest fail!\n",
			__func__);
		selftest |= 1;
	}
exit:
	pr_info("[SSP] %s - %d,%d,%d.%03d,%d.%03d,%d.%03d\n", __func__,
			selftest ? 0 : 1, bist,
			(datax_check / 1000), (int)abs(datax_check % 1000),
			(datay_check / 1000), (int)abs(datay_check % 1000),
			(dataz_check / 1000), (int)abs(dataz_check % 1000));

	return sprintf(buf, "%d,%d,%d.%03d,%d.%03d,%d.%03d,"\
			"%d,%d,%d,%d,%d,%d,%d,%d" "\n",
			selftest ? 0 : 1, bist,
			(datax_check / 1000), (int)abs(datax_check % 1000),
			(datay_check / 1000), (int)abs(datay_check % 1000),
			(dataz_check / 1000), (int)abs(dataz_check % 1000),
			ret ,ret ,ret ,ret ,ret ,ret ,ret ,ret);
}

ssize_t get_gyro_bmi168_selftest_dps(struct ssp_data *data, char *buf)
{
	return sprintf(buf, "%u\n", data->buf[SENSOR_TYPE_GYROSCOPE].gyro_dps);
}

ssize_t set_gyro_bmi168_selftest_dps(struct ssp_data *data, const char *buf)
{
	int new_dps = 0;
	int ret = 0;
	char chTempBuf = 0;

	struct ssp_msg *msg;

	if (!(data->uSensorState & (1 << SENSOR_TYPE_GYROSCOPE)))
		goto exit;

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = GYROSCOPE_DPS_FACTORY;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->buffer = &chTempBuf;
	msg->free_buffer = 0;

	sscanf(buf, "%9d", &iNewDps);

	if (iNewDps == GYROSCOPE_DPS250)
		msg->options |= 0 << SSP_GYRO_DPS;
	else if (iNewDps == GYROSCOPE_DPS500)
		msg->options |= 1 << SSP_GYRO_DPS;
	else if (iNewDps == GYROSCOPE_DPS2000)
		msg->options |= 2 << SSP_GYRO_DPS;
	else {
		msg->options |= 1 << SSP_GYRO_DPS;
		iNewDps = GYROSCOPE_DPS500;
	}

	iRet = ssp_spi_sync(data, msg, 3000);

	if (iRet != SUCCESS) {
		pr_err("[SSP]: %s - Gyro Selftest DPS Timeout!!\n", __func__);
		goto exit;
	}

	if (chTempBuf != SUCCESS) {
		pr_err("[SSP]: %s - Gyro Selftest DPS Error!!\n", __func__);
		goto exit;
	}

	data->buf[SENSOR_TYPE_GYROSCOPE].gyro_dps = (unsigned int)new_dps;
	pr_err("[SSP]: %s - %u dps stored\n", __func__,
			data->buf[SENSOR_TYPE_GYROSCOPE].gyro_dps);
exit:
	return ret;
}

struct gyroscope_sensor_operations gyro_bmi168_ops = {
	.get_gyro_name = get_gyro_bmi168_name,
	.get_gyro_vendor = get_gyro_bmi168_vendor,
	.get_gyro_power_off = get_gyro_bmi168_power_off,
	.get_gyro_power_on = get_gyro_bmi168_power_on,
	.get_gyro_temperature = get_gyro_bmi168_temperature,
	.get_gyro_selftest = get_gyro_bmi168_selftest,
	.get_gyro_selftest_dps = get_gyro_bmi168_selftest_dps,
	.set_gyro_selftest_dps = set_gyro_bmi168_selftest_dps,
};

void gyroscope_bmi168_function_pointer_initialize(struct ssp_data *data)
{
	data->gyro_ops = &gyro_bmi168_ops;
}
