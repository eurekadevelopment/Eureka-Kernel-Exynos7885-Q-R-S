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

#define GM_DATA_SPEC_MIN        -1600
#define GM_DATA_SPEC_MAX        1600

#define GM_SELFTEST_X_SPEC_MIN  -30
#define GM_SELFTEST_X_SPEC_MAX  30
#define GM_SELFTEST_Y_SPEC_MIN  -30
#define GM_SELFTEST_Y_SPEC_MAX  30
#define GM_SELFTEST_Z_SPEC_MIN  -400
#define GM_SELFTEST_Z_SPEC_MAX  -50


int get_fuserom_data(struct ssp_data *data)
{
	//int ret = 0;
	//char buffer[3] = { 0, };

#if 0
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_AP_FUSEROM;
	msg->length = 3;
	msg->options = AP2HUB_READ;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);

	if (ret) {
		data->uFuseRomData[0] = buffer[0];
		data->uFuseRomData[1] = buffer[1];
		data->uFuseRomData[2] = buffer[2];
	} else {
		data->uFuseRomData[0] = 0;
		data->uFuseRomData[1] = 0;
		data->uFuseRomData[2] = 0;
		return FAIL;
	}
#endif

	pr_info("[SSP] FUSE ROM Data %d , %d, %d\n", data->uFuseRomData[0],
	        data->uFuseRomData[1], data->uFuseRomData[2]);

	return SUCCESS;
}

#if 0 // sangmin, it needs 
int check_adc_data_spec(struct ssp_data *data, int sensortype)
{
	if ((data->buf[sensortype].x == 0) &&
	    (data->buf[sensortype].y == 0) &&
	    (data->buf[sensortype].z == 0)) {
		return FAIL;
	} else if ((data->buf[sensortype].x > GM_DATA_SPEC_MAX)
	           || (data->buf[sensortype].x < GM_DATA_SPEC_MIN)
	           || (data->buf[sensortype].y > GM_DATA_SPEC_MAX)
	           || (data->buf[sensortype].y < GM_DATA_SPEC_MIN)
	           || (data->buf[sensortype].z > GM_DATA_SPEC_MAX)
	           || (data->buf[sensortype].z < GM_DATA_SPEC_MIN)) {
		return FAIL;
	} else {
		return SUCCESS;
	}
}
#endif

int set_pdc_matrix(struct ssp_data *data)
{
	int ret = 0;
	//struct ssp_msg *msg;

	if (!(data->uSensorState & 0x04)) {
		pr_info("[SSP] %s - Skip this function!!!"\
		        ", magnetic sensor is not connected(0x%llx)\n",
		        __func__, data->uSensorState);
		return ret;
	}

#if 0
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_AP_SET_MAGNETIC_STATIC_MATRIX;
	msg->length = sizeof(data->pdc_matrix);
	msg->options = AP2HUB_WRITE;
	msg->buffer = data->pdc_matrix;

	msg->free_buffer = 0;

	ret = ssp_spi_async(data, msg);
	if (ret != SUCCESS) {
		pr_err("[SSP] %s - i2c fail %d\n", __func__, ret);
		ret = ERROR;
	}
#endif

	pr_info("[SSP] %s: finished\n", __func__);

	return ret;
}

ssize_t get_magnetic_ak09911_name(char *buf)
{
	return sprintf(buf, "%s\n", "AK09911");
}

ssize_t get_magnetic_ak09911_vendor(char *buf)
{
	return sprintf(buf, "%s\n", "AKM");
}

ssize_t get_magnetic_ak09911_adc(struct ssp_data *data, char *buf)
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
		if (check_adc_data_spec(data, SENSOR_TYPE_GEOMAGNETIC_FIELD) == SUCCESS) {
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

	pr_info("[SSP] %s - x = %d, y = %d, z = %d\n", __func__,
	        sensor_buf[0], sensor_buf[1], sensor_buf[2]);

	return sprintf(buf, "%s,%d,%d,%d\n", (bSuccess ? "OK" : "NG"),
	               sensor_buf[0], sensor_buf[1], sensor_buf[2]);
}

ssize_t get_magnetic_ak09911_dac(struct ssp_data *data, char *strbuf)
{
	bool bSuccess = false;
	//char chTempBuf[22] = { 0,  };

#if 0
	if (!data->uMagCntlRegData) {
		bSuccess = true;
	} else {
		int ret;
		struct ssp_msg *msg;

		pr_info("[SSP] %s - check cntl register before selftest",
		        __func__);
		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		if (msg == NULL) {
			pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
			       __func__);
			return -ENOMEM;
		}
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
#endif

	pr_info("[SSP] %s - CTRL : 0x%x\n", __func__,
	        data->uMagCntlRegData);

	data->uMagCntlRegData = 1;      /* reset the value */

	return sprintf(strbuf, "%s,%d,%d,%d\n",
	               (bSuccess ? "OK" : "NG"), (bSuccess ? 1 : 0), 0, 0);
}

ssize_t get_magnetic_ak09911_raw_data(struct ssp_data *data, char *buf)
{

	pr_info("[SSP] %s - %d,%d,%d\n", __func__,
	        data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x,
	        data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y,
	        data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z);

	if (data->is_geomag_raw_enabled == false) {
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x = -1;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y = -1;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z = -1;
	}

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
	                data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x,
	                data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y,
	                data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z);
}

ssize_t set_magnetic_ak09911_raw_data(struct ssp_data *data, const char *buf)
{
	char chTempbuf[4] = { 0 };
	int ret;
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
		int retries = 50;

		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x = 0;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y = 0;
		data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z = 0;

		make_command(data, ADD_SENSOR, SENSOR_TYPE_GEOMAGNETIC_POWER,
		             chTempbuf, 4);

		do {
			msleep(20);
			if (check_adc_data_spec(data, SENSOR_TYPE_GEOMAGNETIC_POWER) == SUCCESS) {
				break;
			}
		} while (--retries);

		if (retries > 0) {
			pr_info("[SSP] %s - success, %d\n", __func__, retries);
		} else
			pr_err("[SSP] %s - wait timeout, %d\n", __func__,
			       retries);

		data->is_geomag_raw_enabled = true;
	} else {
		make_command(data, REMOVE_SENSOR, SENSOR_TYPE_GEOMAGNETIC_POWER,
		             chTempbuf, 4);
		data->is_geomag_raw_enabled = false;
	}
#endif

	return ret;
}

ssize_t get_magnetic_ak09911_asa(struct ssp_data *data, char *buf)
{
	return sprintf(buf, "%d,%d,%d\n", (s16)data->uFuseRomData[0],
	               (s16)data->uFuseRomData[1], (s16)data->uFuseRomData[2]);
}

ssize_t get_magnetic_ak09911_status(struct ssp_data *data, char *buf)
{
	bool bSuccess;

	if ((data->uFuseRomData[0] == 0) ||
	    (data->uFuseRomData[0] == 0xff) ||
	    (data->uFuseRomData[1] == 0) ||
	    (data->uFuseRomData[1] == 0xff) ||
	    (data->uFuseRomData[2] == 0) ||
	    (data->uFuseRomData[2] == 0xff)) {
		bSuccess = false;
	} else {
		bSuccess = true;
	}

	return sprintf(buf, "%s,%u\n", (bSuccess ? "OK" : "NG"), bSuccess);
}

ssize_t get_magnetic_ak09911_logging_data(struct ssp_data *data, char *buf)
{
	//char buffer[21] = {0, };
	//int ret = 0;
	//int logging_data[8] = {0, };
	//struct ssp_data *data = dev_get_drvdata(dev);
	//struct ssp_msg *msg;

#if 0
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
		       __func__);
		goto exit;
	}

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

	logging_data[0] = buffer[0];    /* ST1 Reg */
	logging_data[1] = (short)((buffer[3] << 8) + buffer[2]);
	logging_data[2] = (short)((buffer[5] << 8) + buffer[4]);
	logging_data[3] = (short)((buffer[7] << 8) + buffer[6]);
	logging_data[4] = buffer[1];    /* ST2 Reg */
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
#endif
//exit:
	return snprintf(buf, PAGE_SIZE, "-1,0,0,0,0,0,0,0,0,0,0\n");
}

ssize_t get_magnetic_ak09911_selftest(struct ssp_data *data, char *buf)
{
	s8 result[4] = {-1, -1, -1, -1};
	//char bufSelftset[22] = {0, };
	//char bufAdc[4] = {0, };
	s16 iSF_X = 0, iSF_Y = 0, iSF_Z = 0;
	s16 iADC_X = 0, iADC_Y = 0, iADC_Z = 0;
	//s32 dMsDelay = 20;
	//int ret = 0;
	int spec_out_retries = 0;
	//struct ssp_msg *msg;

	pr_info("[SSP] %s in\n", __func__);

	/* STATUS */
	if ((data->uFuseRomData[0] == 0) ||
	    (data->uFuseRomData[0] == 0xff) ||
	    (data->uFuseRomData[1] == 0) ||
	    (data->uFuseRomData[1] == 0xff) ||
	    (data->uFuseRomData[2] == 0) ||
	    (data->uFuseRomData[2] == 0xff)) {
		result[0] = -1;
	} else {
		result[0] = 0;
	}

#if 0
Retry_selftest:
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP] %s, failed to alloc memory for ssp_msg\n",
		       __func__);
		goto exit;
	}
	msg->cmd = GEOMAGNETIC_FACTORY;
	msg->length = 22;
	msg->options = AP2HUB_READ;
	msg->buffer = bufSelftset;
	msg->free_buffer = 0;

	ret = ssp_spi_sync(data, msg, 1000);
	if (ret != SUCCESS) {
		pr_err("[SSP] %s - Magnetic Selftest Timeout!! %d\n",
		       __func__, ret);
		goto exit;
	}


	/* read 6bytes data registers */
	iSF_X = (s16)((bufSelftset[13] << 8) + bufSelftset[14]);
	iSF_Y = (s16)((bufSelftset[15] << 8) + bufSelftset[16]);
	iSF_Z = (s16)((bufSelftset[17] << 8) + bufSelftset[18]);

	/* DAC (store Cntl Register value to check power down) */
	result[2] = bufSelftset[21];

	iSF_X = (s16)(((iSF_X * data->uFuseRomData[0]) >> 7) + iSF_X);
	iSF_Y = (s16)(((iSF_Y * data->uFuseRomData[1]) >> 7) + iSF_Y);
	iSF_Z = (s16)(((iSF_Z * data->uFuseRomData[2]) >> 7) + iSF_Z);

	pr_info("[SSP] %s: self test x = %d, y = %d, z = %d\n",
	        __func__, iSF_X, iSF_Y, iSF_Z);

	if ((iSF_X >= GM_SELFTEST_X_SPEC_MIN)
	    && (iSF_X <= GM_SELFTEST_X_SPEC_MAX)) {
		pr_info("[SSP] x passed self test, expect -30<=x<=30\n");
	} else {
		pr_info("[SSP] x failed self test, expect -30<=x<=30\n");
	}
	if ((iSF_Y >= GM_SELFTEST_Y_SPEC_MIN)
	    && (iSF_Y <= GM_SELFTEST_Y_SPEC_MAX)) {
		pr_info("[SSP] y passed self test, expect -30<=y<=30\n");
	} else {
		pr_info("[SSP] y failed self test, expect -30<=y<=30\n");
	}
	if ((iSF_Z >= GM_SELFTEST_Z_SPEC_MIN)
	    && (iSF_Z <= GM_SELFTEST_Z_SPEC_MAX)) {
		pr_info("[SSP] z passed self test, expect -400<=z<=-50\n");
	} else {
		pr_info("[SSP] z failed self test, expect -400<=z<=-50\n");
	}

	/* SELFTEST */
	if ((iSF_X >= GM_SELFTEST_X_SPEC_MIN)
	    && (iSF_X <= GM_SELFTEST_X_SPEC_MAX)
	    && (iSF_Y >= GM_SELFTEST_Y_SPEC_MIN)
	    && (iSF_Y <= GM_SELFTEST_Y_SPEC_MAX)
	    && (iSF_Z >= GM_SELFTEST_Z_SPEC_MIN)
	    && (iSF_Z <= GM_SELFTEST_Z_SPEC_MAX)) {
		result[1] = 0;
	}

	if ((result[1] == -1) && (spec_out_retries++ < 5)) {
		pr_err("[SSP] %s, selftest spec out. Retry = %d", __func__,
		       spec_out_retries);
		goto Retry_selftest;
	}

	spec_out_retries = 10;

	/* ADC */
	memcpy(&bufAdc[0], &dMsDelay, 4);

	data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x = 0;
	data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y = 0;
	data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z = 0;

	if (!(atomic64_read(&data->aSensorEnable) & (1 <<
	                                             SENSOR_TYPE_GEOMAGNETIC_POWER)))
		make_command(data, ADD_SENSOR, SENSOR_TYPE_GEOMAGNETIC_POWER,
		             bufAdc, 4);

	do {
		msleep(60);
		if (check_adc_data_spec(data, SENSOR_TYPE_GEOMAGNETIC_POWER) == SUCCESS) {
			break;
		}
	} while (--spec_out_retries);

	if (spec_out_retries > 0) {
		result[3] = 0;
	}

	iADC_X = data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].x;
	iADC_Y = data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].y;
	iADC_Z = data->buf[SENSOR_TYPE_GEOMAGNETIC_POWER].z;

	if (!(atomic64_read(&data->aSensorEnable) & (1 <<
	                                             SENSOR_TYPE_GEOMAGNETIC_POWER)))
		make_command(data, REMOVE_SENSOR, SENSOR_TYPE_GEOMAGNETIC_POWER,
		             bufAdc, 4);
#endif
	pr_info("[SSP] %s -adc, x = %d, y = %d, z = %d, retry = %d\n",
	        __func__, iADC_X, iADC_Y, iADC_Z, spec_out_retries);

//exit:
	pr_info("[SSP] %s out. Result = %d %d %d %d\n",
	        __func__, result[0], result[1], result[2], result[3]);

	return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
	               result[0], result[1], iSF_X, iSF_Y, iSF_Z,
	               result[2], result[3], iADC_X, iADC_Y, iADC_Z);
}



struct magnetic_sensor_operations magnetic_ak09911_ops = {
	.get_magnetic_name = get_magnetic_ak09911_name,
	.get_magnetic_vendor = get_magnetic_ak09911_vendor,
	.get_magnetic_adc = get_magnetic_ak09911_adc,
	.get_magnetic_dac = get_magnetic_ak09911_dac,
	.get_magnetic_raw_data = get_magnetic_ak09911_raw_data,
	.set_magnetic_raw_data = set_magnetic_ak09911_raw_data,
	.get_magnetic_asa = get_magnetic_ak09911_asa,
	.get_magnetic_status = get_magnetic_ak09911_status,
	.get_magnetic_logging_data = get_magnetic_ak09911_logging_data,
	.get_magnetic_selftest = get_magnetic_ak09911_selftest,
};

int initialize_magnetic_sensor(struct ssp_data *data)
{
	int ret;

	ret = get_fuserom_data(data);
	if (ret < 0)
		pr_err("[SSP] %s - get_fuserom_data failed %d\n",
		       __func__, ret);

	ret = set_pdc_matrix(data);
	if (ret < 0)
		pr_err("[SSP] %s - set_magnetic_pdc_matrix failed %d\n",
		       __func__, ret);

	return ret < 0 ? ret : SUCCESS;
}

void magnetic_ak09911_function_pointer_initialize(struct ssp_data *data)
{
	data->magnetic_ops = &magnetic_ak09911_ops;
}
