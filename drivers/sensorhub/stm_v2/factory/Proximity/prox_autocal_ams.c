#include "../../ssp.h"
#include "../ssp_factory.h"

#define PROX_ADC_BITS_NUM               14

/*************************************************************************/
/* Functions                                                             */
/*************************************************************************/

u16 get_proximity_tmd3725_auto_cal_raw_data(struct ssp_data *data)
{
	u16 uRowdata = 0;
	char chTempbuf[8] = { 0, };

	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);

	if (data->is_proxraw_enabled == false) {
		make_command(data, ADD_SENSOR, SENSOR_TYPE_PROXIMITY_RAW, chTempbuf, 8);
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

ssize_t get_proximity_tmd3725_auto_cal_name(char *buf)
{
	return sprintf(buf, "%s\n", "TMD3725");
}

ssize_t get_proximity_tmd3725_auto_cal_vendor(char *buf)
{
	return sprintf(buf, "%s\n", "AMS");
}

ssize_t get_proximity_tmd3725_probe_status(struct ssp_data *data, char *buf)
{
	bool probe_pass_fail = FAIL;

	if (data->uSensorState & (1ULL << SENSOR_TYPE_PROXIMITY)) {
		probe_pass_fail = SUCCESS;
	} else {
		probe_pass_fail = FAIL;
	}

	pr_info("[SSP]: %s - All sensor 0x%llx, prox_sensor %d \n",
	        __func__, data->uSensorState, probe_pass_fail);

	return snprintf(buf, PAGE_SIZE, "%d\n", probe_pass_fail);
}

ssize_t get_tmd3725_threshold_high(struct ssp_data *data, char *buf)
{
	ssp_dbg("[SSP]: ProxThresh = hi - %u \n", data->uProxHiThresh);

	return sprintf(buf, "%u\n", data->uProxHiThresh);
}

ssize_t set_tmd3725_threshold_high(struct ssp_data *data, const char *buf)
{
	u16 uNewThresh;
	int ret, i = 0;
	u16 prox_bits_mask = 0, prox_non_bits_mask = 0;

	while (i < PROX_ADC_BITS_NUM) {
		prox_bits_mask += (1 << i++);
	}

	while (i < 16) {
		prox_non_bits_mask += (1 << i++);
	}

	ret = kstrtou16(buf, 10, &uNewThresh);
	if (ret < 0) {
		pr_err("[SSP]: %s - kstrto16 failed.(%d)\n", __func__, ret);
	} else {
		if (uNewThresh & prox_non_bits_mask) {
			pr_err("[SSP]: %s - allow %ubits.(%d)\n", __func__, PROX_ADC_BITS_NUM,
			       uNewThresh);
		} else {
			uNewThresh &= prox_bits_mask;
			data->uProxHiThresh = uNewThresh;
		}
	}

	ssp_dbg("[SSP]: %s - new prox threshold : hi - %u \n", __func__,
	        data->uProxHiThresh);

	return ret;
}

ssize_t get_tmd3725_threshold_low(struct ssp_data *data, char *buf)
{
	ssp_dbg("[SSP]: ProxThresh = lo - %u \n", data->uProxLoThresh);

	return sprintf(buf, "%u\n", data->uProxLoThresh);
}

ssize_t set_tmd3725_threshold_low(struct ssp_data *data, const char *buf)
{
	u16 uNewThresh;
	int ret, i = 0;
	u16 prox_bits_mask = 0, prox_non_bits_mask = 0;

	while (i < PROX_ADC_BITS_NUM) {
		prox_bits_mask += (1 << i++);
	}

	while (i < 16) {
		prox_non_bits_mask += (1 << i++);
	}

	ret = kstrtou16(buf, 10, &uNewThresh);
	if (ret < 0) {
		pr_err("[SSP]: %s - kstrto16 failed.(%d)\n", __func__, ret);
	} else {
		if (uNewThresh & prox_non_bits_mask) {
			pr_err("[SSP]: %s - allow %ubits.(%d)\n", __func__, PROX_ADC_BITS_NUM,
			       uNewThresh);
		} else {
			uNewThresh &= prox_bits_mask;
			data->uProxLoThresh = uNewThresh;
		}
	}

	ssp_dbg("[SSP]: %s - new prox threshold : lo - %u \n", __func__,
	        data->uProxLoThresh);

	return ret;
}

ssize_t get_tmd3725_threshold_detect_high(struct ssp_data *data, char *buf)
{
	ssp_dbg("[SSP]: ProxThresh = hidetect - %u \n", data->uProxHiThresh_detect);

	return sprintf(buf, "%u\n", data->uProxHiThresh_detect);
}

ssize_t set_tmd3725_threshold_detect_high(struct ssp_data *data,
                                          const char *buf)
{
	u16 uNewThresh;
	int ret, i = 0;
	u16 prox_bits_mask = 0, prox_non_bits_mask = 0;

	while (i < PROX_ADC_BITS_NUM) {
		prox_bits_mask += (1 << i++);
	}

	while (i < 16) {
		prox_non_bits_mask += (1 << i++);
	}

	ret = kstrtou16(buf, 10, &uNewThresh);
	if (ret < 0) {
		pr_err("[SSP]: %s - kstrto16 failed.(%d)\n", __func__, ret);
	} else {
		if (uNewThresh & prox_non_bits_mask) {
			pr_err("[SSP]: %s - allow %ubits.(%d)\n", __func__, PROX_ADC_BITS_NUM,
			       uNewThresh);
		} else {
			uNewThresh &= prox_bits_mask;
			data->uProxHiThresh_detect = uNewThresh;
		}
	}

	ssp_dbg("[SSP]: %s - new prox threshold : hidetect - %u \n", __func__,
	        data->uProxHiThresh_detect);

	return ret;
}

ssize_t get_tmd3725_threshold_detect_low(struct ssp_data *data, char *buf)
{
	ssp_dbg("[SSP]: ProxThresh = lodetect - %u \n", data->uProxLoThresh_detect);

	return sprintf(buf, "%u\n", data->uProxLoThresh_detect);
}

ssize_t set_tmd3725_threshold_detect_low(struct ssp_data *data, const char *buf)
{
	u16 uNewThresh;
	int ret, i = 0;
	u16 prox_bits_mask = 0, prox_non_bits_mask = 0;

	while (i < PROX_ADC_BITS_NUM) {
		prox_bits_mask += (1 << i++);
	}

	while (i < 16) {
		prox_non_bits_mask += (1 << i++);
	}

	ret = kstrtou16(buf, 10, &uNewThresh);
	if (ret < 0) {
		pr_err("[SSP]: %s - kstrto16 failed.(%d)\n", __func__, ret);
	} else {
		if (uNewThresh & prox_non_bits_mask) {
			pr_err("[SSP]: %s - allow %ubits.(%d)\n", __func__, PROX_ADC_BITS_NUM,
			       uNewThresh);
		} else {
			uNewThresh &= prox_bits_mask;
			data->uProxLoThresh_detect = uNewThresh;
		}
	}

	ssp_dbg("[SSP]: %s - new prox threshold : lodetect - %u \n", __func__,
	        data->uProxLoThresh_detect);

	return ret;
}

ssize_t get_proximity_tmd3725_trim_value(struct ssp_data *data, char *buf)
{
	int ret = 0;
	//int iReties = 0;
	//u8 buffer[8] = {0, };
	char *buffer = NULL;
	int buffer_length = 0;
	int trim = 0;

	if (!(data->uSensorState & (1ULL << SENSOR_TYPE_PROXIMITY))) {
		ssp_infof("Skip this function!, proximity sensor is not connected(0x%llx)",
		          data->uSensorState);
		return FAIL;
	}


	if (data->uSensorState == 0 || data->is_reset_started == true) {
		return FAIL;
	}

	if ((data->fw_dl_state != FW_DL_STATE_DONE) &&
	    (data->fw_dl_state != FW_DL_STATE_NONE)) {
		pr_err("[SSP]: %s - firmware downloading state = %d",
		       __func__, data->fw_dl_state);

		return FAIL;
	}

	ret = ssp_send_command(data, CMD_GETVALUE, SENSOR_TYPE_PROXIMITY,
	                       PROXIMITY_OFFSET, 1000, NULL, 0, &buffer, &buffer_length);

	if (ret != SUCCESS) {
		ssp_errf("ssp_send_command Fail %d", ret);
		if (buffer != NULL) {
			kfree(buffer);
		}
		return FAIL;
	}

	if (buffer == NULL) {
		ssp_errf("buffer is null");
		return -EINVAL;
	}

	if (buffer_length != 2) {
		ssp_errf("buffer length error %d", buffer_length);
		ret = snprintf(buf, PAGE_SIZE, "-1,0,0,0,0,0,0,0,0,0,0\n");
		if (buffer != NULL) {
			kfree(buffer);
		}
		return -EINVAL;
	}

	if (buffer[1] > 0) {
		data->prox_trim = (buffer[0]) * (-1);
	} else {
		data->prox_trim = buffer[0];
	}
	trim = buffer[0];

	pr_info("[SSP] %s - %d, 0x%x, 0x%x \n", __func__, trim, buffer[1], buffer[0]);

	ret = snprintf(buf, PAGE_SIZE, "%d\n", trim);

	if (buffer != NULL) {
		kfree(buffer);
	}

	return ret;
}

ssize_t get_proximity_tmd3725_trim_check(struct ssp_data *data, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", data->prox_trim);
}

ssize_t get_proximity_tmd3725_avg_raw_data(struct ssp_data *data, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n",
	                data->buf[SENSOR_TYPE_PROXIMITY_RAW].prox_raw[1],
	                data->buf[SENSOR_TYPE_PROXIMITY_RAW].prox_raw[2],
	                data->buf[SENSOR_TYPE_PROXIMITY_RAW].prox_raw[3]);
}

/*
        Proximity Raw Sensor Register/Unregister
*/
ssize_t set_proximity_tmd3725_avg_raw_data(struct ssp_data *data,
                                           const char *buf)
{
	char chTempbuf[8] = { 0, };
	int ret;
	int64_t dEnable;

	s32 dMsDelay = 20;
	memcpy(&chTempbuf[0], &dMsDelay, 4);

	ret = kstrtoll(buf, 10, &dEnable);
	if (ret < 0) {
		return ret;
	}

	if (dEnable) {
		make_command(data, ADD_SENSOR, SENSOR_TYPE_PROXIMITY_RAW, chTempbuf, 8);
		data->is_proxraw_enabled = true;
	} else {
		make_command(data, REMOVE_SENSOR, SENSOR_TYPE_PROXIMITY_RAW,
		             chTempbuf, 4);
		data->is_proxraw_enabled = false;
	}

	return ret;
}


ssize_t get_proximity_tmd3725_setting(char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", 1);
}

ssize_t set_proximity_tmd3725_setting(struct ssp_data *data, const char *buf)
{
	int ret;
	u8 val[2] = {0, };
	char *token;
	char *str;

	pr_info("[SSP] %s - %s\n", __func__, buf);

	//parsing
	str = (char *)buf;
	token = strsep(&str, " \n");
	if (token == NULL) {
		pr_err("[SSP] %s : too few arguments (2 needed)", __func__);
		return -EINVAL;
	}

	ret = kstrtou8(token, 10, &val[0]);
	if (ret < 0) {
		pr_err("[SSP] %s : kstrtou8 error %d", __func__, ret);
		return ret;
	}

	token = strsep(&str, " \n");
	if (token == NULL) {
		pr_err("[SSP] %s : too few arguments (2 needed)", __func__);
		return -EINVAL;
	}

	ret = kstrtou8(token, 16, &val[1]);
	if (ret < 0) {
		pr_err("[SSP] %s : kstrtou8 error %d", __func__, ret);
		return ret;
	}

	pr_info("[SSP] %s - index = %d value = 0x%x\n", __func__, val[0], val[1]);

	ret = ssp_send_command(data, CMD_SETVALUE, SENSOR_TYPE_PROXIMITY,
	                       PROXIMITY_SETTING, 0, &val[0], 2 * sizeof(char), NULL, NULL);

	if (ret != SUCCESS) {
		ssp_errf("ssp_send_command Fail %d", ret);
	}

	return ret;
}


struct proximity_sensor_operations prox_tmd3725_auto_cal_ops = {
	.get_proximity_name = get_proximity_tmd3725_auto_cal_name,
	.get_proximity_vendor = get_proximity_tmd3725_auto_cal_vendor,
	.get_proximity_probe_status = get_proximity_tmd3725_probe_status,
	.get_threshold_high = get_tmd3725_threshold_high,
	.set_threshold_high = set_tmd3725_threshold_high,
	.get_threshold_low = get_tmd3725_threshold_low,
	.set_threshold_low = set_tmd3725_threshold_low,
	.get_threshold_detect_high = get_tmd3725_threshold_detect_high,
	.set_threshold_detect_high = set_tmd3725_threshold_detect_high,
	.get_threshold_detect_low = get_tmd3725_threshold_detect_low,
	.set_threshold_detect_low = set_tmd3725_threshold_detect_low,
	.get_proximity_avg_raw_data = get_proximity_tmd3725_avg_raw_data,
	.set_proximity_avg_raw_data = set_proximity_tmd3725_avg_raw_data,
	.get_proximity_setting = get_proximity_tmd3725_setting,
	.set_proximity_setting = set_proximity_tmd3725_setting,
	.get_proximity_raw_data = get_proximity_tmd3725_auto_cal_raw_data,
	.get_proximity_trim_value = get_proximity_tmd3725_trim_value,
	.get_proximity_trim_check = get_proximity_tmd3725_trim_check,
};

void proximity_tmd3725_auto_cal_function_pointer_initialize(
        struct ssp_data *data)
{
	data->proximity_ops = &prox_tmd3725_auto_cal_ops;
}

