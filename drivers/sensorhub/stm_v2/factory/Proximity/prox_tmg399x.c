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

#define CANCELATION_FILE_PATH   "/efs/FactoryApp/prox_cal"

/*************************************************************************/
/* Functions                                                             */
/*************************************************************************/

u16 get_proximity_tmg399x_raw_data(struct ssp_data *data)
{
	u16 uRowdata = 0;
	char chTempbuf[4] = { 0 };

	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);

	if (data->is_proxraw_enabled == false) {
		make_command(data, ADD_SENSOR, SENSOR_TYPE_PROXIMITY_RAW, chTempbuf, 4);
		msleep(200);
		uRowdata = data->buf[SENSOR_TYPE_PROXIMITY_RAW].prox_raw[0];
		make_command(data, REMOVE_SENSOR, SENSOR_TYPE_PROXIMITY_RAW,
		             chTempbuf, 4);
	} else {
		uRowdata = data->buf[SENSOR_TYPE_PROXIMITY_RAW].prox_raw[0];
	}

	return uRowdata;
}

/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

ssize_t get_proximity_tmg399x_name(char *buf)
{
	return sprintf(buf, "%s\n", "TMD399x");
}

ssize_t get_proximity_tmg399x_vendor(char *buf)
{
	return sprintf(buf, "%s\n", "AMS");
}

ssize_t get_proximity_tmg399x_avg_raw_data(struct ssp_data *data, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
	                data->buf[SENSOR_TYPE_PROXIMITY_RAW].prox_raw[1],
	                data->buf[SENSOR_TYPE_PROXIMITY_RAW].prox_raw[2],
	                data->buf[SENSOR_TYPE_PROXIMITY_RAW].prox_raw[3]);
}

ssize_t set_proximity_tmg399x_avg_raw_data(struct ssp_data *data,
                                           const char *buf)
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
		make_command(data, ADD_SENSOR, SENSOR_TYPE_PROXIMITY_RAW, chTempbuf, 4);
		data->is_proxraw_enabled = true;
	} else {
		make_command(data, REMOVE_SENSOR, SENSOR_TYPE_PROXIMITY_RAW,
		             chTempbuf, 4);
		data->is_proxraw_enabled = false;
	}
#endif
	return ret;
}


int proximity_open_calibration(struct ssp_data *data)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cancel_filp = NULL;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cancel_filp = filp_open(CANCELATION_FILE_PATH, O_RDONLY, 0660);
	if (IS_ERR(cancel_filp)) {
		ret = PTR_ERR(cancel_filp);
		if (ret != -ENOENT)
			pr_err("[SSP]: %s - Can't open cancelation file\n",
			       __func__);
		set_fs(old_fs);
		goto exit;
	}

	ret = vfs_read(cancel_filp, (u8 *)&data->uProxCanc, sizeof(unsigned int), &cancel_filp->f_pos);
	if (ret != sizeof(u8)) {
		pr_err("[SSP]: %s - Can't read the cancel data\n", __func__);
		ret = -EIO;
	}

	if (data->uProxCanc != 0) {
		/*If there is an offset cal data. */
		data->uProxHiThresh =
		        data->uProxHiThresh_default + data->uProxCanc;
		data->uProxLoThresh =
		        data->uProxLoThresh_default + data->uProxCanc;
	}

	pr_info("[SSP] %s: proximity ps_canc = %d, ps_thresh hi - %d lo - %d\n",
	        __func__, data->uProxCanc, data->uProxHiThresh,
	        data->uProxLoThresh);

	filp_close(cancel_filp, current->files);
	set_fs(old_fs);

exit:
	return ret;
}

int calculate_proximity_threshold(struct ssp_data *data)
{
	int cal_hi_thres, cal_low_thres;

#if defined(CONFIG_SENSORS_SSP_TMG3992)
	cal_hi_thres = 120;
	cal_low_thres = 55;
#else
	cal_hi_thres = 2000;
	cal_low_thres = 840;
#endif
	if (data->uCrosstalk < cal_low_thres) {
		data->uProxCanc = 0;
		data->uProxCalResult = 2;
	} else if (data->uCrosstalk <= cal_hi_thres) {
		data->uProxCanc = data->uCrosstalk * 5 / 10;
		data->uProxCalResult = 1;
	} else {
		data->uProxCanc = 0;
		data->uProxCalResult = 0;
		pr_info("[SSP] crosstalk > %d, calibration failed\n", cal_hi_thres);
		return ERROR;
	}
	data->uProxHiThresh = data->uProxHiThresh_default + data->uProxCanc;
	data->uProxLoThresh = data->uProxLoThresh_default + data->uProxCanc;

	pr_info("[SSP] %s - crosstalk_offset = %u(%u), HI_THD = %u, LOW_THD = %u\n",
	        __func__, data->uProxCanc, data->uCrosstalk,
	        data->uProxHiThresh, data->uProxLoThresh);

	return SUCCESS;
}

int proximity_store_cancelation(struct ssp_data *data, int iCalCMD)
{
	int ret = 0;
	mm_segment_t old_fs;
	struct file *cancel_filp = NULL;

	if (iCalCMD) {
		data->uCrosstalk = get_proximity_tmg399x_raw_data(data);
		ret = calculate_proximity_threshold(data);
	} else {
		data->uProxHiThresh = data->uProxHiThresh_default;
		data->uProxLoThresh = data->uProxLoThresh_default;
		data->uProxCanc = 0;
	}

	if (ret != ERROR) {
		set_proximity_threshold(data);
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cancel_filp = filp_open(CANCELATION_FILE_PATH,
	                        O_CREAT | O_TRUNC | O_WRONLY | O_SYNC, 0660);
	if (IS_ERR(cancel_filp)) {
		pr_err("%s: Can't open cancelation file\n", __func__);
		set_fs(old_fs);
		ret = PTR_ERR(cancel_filp);
		return ret;
	}

	ret = vfs_write(cancel_filp, (u8 *)&data->uProxCanc, sizeof(unsigned int), &cancel_filp->f_pos);
	if (ret != sizeof(unsigned int)) {
		pr_err("%s: Can't write the cancel data to file\n", __func__);
		ret = -EIO;
	}

	filp_close(cancel_filp, current->files);
	set_fs(old_fs);

	return ret;
}

ssize_t get_proximity_tmg399x_cancel(struct ssp_data *data, char *buf)
{
	ssp_info("uProxThresh : hi : %u lo : %u, uProxCanc = %u",
	         data->uProxHiThresh, data->uProxLoThresh, data->uProxCanc);

	return sprintf(buf, "%u,%u,%u\n", data->uProxCanc,
	               data->uProxHiThresh, data->uProxLoThresh);
}

ssize_t set_proximity_tmg399x_cancel(struct ssp_data *data, const char *buf)
{
	int iCalCMD = 0, ret = 0;

	if (sysfs_streq(buf, "1")) { /* calibrate cancelation value */
		iCalCMD = 1;
	} else if (sysfs_streq(buf, "0")) { /* reset cancelation value */
		iCalCMD = 0;
	} else {
		pr_debug("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	ret = proximity_store_cancelation(data, iCalCMD);
	if (ret < 0) {
		pr_err("[SSP]: - %s proximity_store_cancelation() failed\n",
		       __func__);
		return ret;
	}

	ssp_infof("%u", iCalCMD);
	return ret;
}

ssize_t get_tmg399x_threshold_high(struct ssp_data *data, char *buf)
{
	ssp_info("uProxThresh = hi - %u, lo - %u",
	         data->uProxHiThresh, data->uProxLoThresh);

	return sprintf(buf, "%u,%u\n", data->uProxHiThresh,
	               data->uProxLoThresh);
}

ssize_t set_tmg399x_threshold_high(struct ssp_data *data, const char *buf)
{
	u16 uNewThresh;
	int ret = 0;

	ret = kstrtou16(buf, 10, &uNewThresh);
	if (ret < 0) {
		pr_err("[SSP]: %s - kstrtoint failed.(%d)\n", __func__, ret);
	} else {
		if (uNewThresh & 0xc000) {
			pr_err("[SSP]: %s - allow 14bits.(%d)\n", __func__, uNewThresh);
		} else {
			uNewThresh &= 0x3fff;
			data->uProxHiThresh = data->uProxHiThresh_default = uNewThresh;
			set_proximity_threshold(data);
		}
	}

	ssp_infof("new prox threshold : hi - %u, lo - %u",
	          data->uProxHiThresh, data->uProxLoThresh);

	return ret;
}

ssize_t get_tmg399x_threshold_low(struct ssp_data *data, char *buf)
{
	ssp_info("uProxThresh = hi - %u, lo - %u",
	         data->uProxHiThresh, data->uProxLoThresh);

	return sprintf(buf, "%u,%u\n", data->uProxHiThresh,
	               data->uProxLoThresh);
}

ssize_t set_tmg399x_threshold_low(struct ssp_data *data, const char *buf)
{
	u16 uNewThresh;
	int ret = 0;

	ret = kstrtou16(buf, 10, &uNewThresh);
	if (ret < 0) {
		pr_err("[SSP]: %s - kstrtoint failed.(%d)\n", __func__, ret);
	} else {
		if (uNewThresh & 0xc000) {
			pr_err("[SSP]: %s - allow 14bits.(%d)\n", __func__, uNewThresh);
		} else {
			uNewThresh &= 0x3fff;
			data->uProxLoThresh = data->uProxLoThresh_default = uNewThresh;
			set_proximity_threshold(data);
		}
	}

	ssp_infof("new prox threshold : hi - %u, lo - %u",
	          data->uProxHiThresh, data->uProxLoThresh);

	return ret;
}

ssize_t get_proximity_tmg399x_cancel_pass(struct ssp_data *data, char *buf)
{
	pr_info("[SSP] %s, %u\n", __func__, data->uProxCalResult);
	return snprintf(buf, PAGE_SIZE, "%u\n", data->uProxCalResult);
}

ssize_t get_proximity_tmg399x_trim_value(struct ssp_data *data, char *buf)
{
	//struct ssp_data *data = dev_get_drvdata(dev);
	//int reet, iReties = 0;
	//struct ssp_msg *msg;
	//u8 buffer[8] = {0, };
	int trim = 0;;

#if 0
retries:
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (msg == NULL) {
		pr_err("[SSP]: %s - failed to allocate memory\n", __func__);
		return FAIL;
	}
	msg->cmd = MSG2SSP_AP_PROX_GET_TRIM;
	msg->length = 1;
	msg->options = AP2HUB_READ;
	msg->buffer = buffer;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 1000);
	if (iRet != SUCCESS) {
		pr_err("[SSP] %s fail %d\n", __func__, iRet);

		if (iReties++ < 2) {
			pr_err("[SSP] %s fail, retry\n", __func__);
			mdelay(5);
			goto retries;
		}
		return FAIL;
	}

	trim = (int)buffer[0];
#endif

	pr_info("[SSP] %s - %d \n", __func__, trim);

	return snprintf(buf, PAGE_SIZE, "%d\n", trim);
}

struct proximity_sensor_operations prox_tmg399x_ops = {
	.get_proximity_name = get_proximity_tmg399x_name,
	.get_proximity_vendor = get_proximity_tmg399x_vendor,
	.get_proximity_avg_raw_data = get_proximity_tmg399x_avg_raw_data,
	.set_proximity_avg_raw_data = set_proximity_tmg399x_avg_raw_data,
	.get_proximity_cancel = get_proximity_tmg399x_cancel,
	.set_proximity_cancel = set_proximity_tmg399x_cancel,
	.get_proximity_cancel_pass = get_proximity_tmg399x_cancel_pass,
	.get_threshold_high = get_tmg399x_threshold_high,
	.set_threshold_high = set_tmg399x_threshold_high,
	.get_threshold_low = get_tmg399x_threshold_low,
	.set_threshold_low = set_tmg399x_threshold_low,
	.get_proximity_trim_value = get_proximity_tmg399x_trim_value,
	.get_proximity_raw_data = get_proximity_tmg399x_raw_data,
};

void proximity_tmg399x_function_pointer_initialize(struct ssp_data *data)
{
	data->proximity_ops = &prox_tmg399x_ops;
}