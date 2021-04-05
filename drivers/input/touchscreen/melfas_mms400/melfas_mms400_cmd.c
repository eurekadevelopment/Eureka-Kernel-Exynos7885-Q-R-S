/*
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Command Functions (Optional)
 *
 */

#include "melfas_mms400.h"
#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
#include <linux/trustedui.h>
#endif

#if MMS_USE_CMD_MODE

static ssize_t scrub_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	char buff[256] = { 0 };

#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
	input_info(true, &info->client->dev,
			"%s: scrub_id: %d\n", __func__, info->scrub_id);
#else
	input_info(true, &info->client->dev,
			"%s: scrub_id: %d, X:%d, Y:%d\n", __func__,
			info->scrub_id, info->scrub_x, info->scrub_y);
#endif

	snprintf(buff, sizeof(buff), "%d %d %d", info->scrub_id, info->scrub_x, info->scrub_y);

	info->scrub_id = 0;
	info->scrub_x = 0;
	info->scrub_y = 0;

	return snprintf(buf, PAGE_SIZE, "%s", buff);
}

/**
 * Command : Update firmware
 */
static void cmd_fw_update(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };
	int fw_location = 0;

	sec_cmd_set_default_result(sec);

	fw_location = sec->cmd_param[0];
#if defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
	if (sec->cmd_param[0] == 1) {
		snprintf(buff, sizeof(buff), "%s", "OK");
		sec->cmd_state = SEC_CMD_STATUS_OK;	
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		input_info(true, &info->client->dev, "%s: user_ship, success \n", __func__);
		return;
	}
#endif

	/* Factory cmd for firmware update
	 * argument represent what is source of firmware like below.
	 *
	 * 0 : [BUILT_IN] Getting firmware which is for user.
	 * 1 : [UMS] Getting firmware from sd card.
	 * 2 : none
	 * 3 : [FFU] Getting firmware from air.
	 */

	switch (fw_location) {
	case 0:
		if (mms_fw_update_from_kernel(info, true))
			goto ERROR;
		break;
	case 1:
		if (mms_fw_update_from_storage(info, true))
			goto ERROR;
		break;
	case 3:
		if (mms_fw_update_from_ffu(info, true))
			goto ERROR;
		break;
	default:
		goto ERROR;
	}

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;	
	goto EXIT;

ERROR:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	goto EXIT;

EXIT:
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buff, sec->cmd_state);
}

/**
 * Command : Get firmware version from MFSB file
 */
static void cmd_get_fw_ver_bin(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[16] = { 0 };
	const char *fw_name = info->dtdata->fw_name;
	const struct firmware *fw;
	struct mms_bin_hdr *fw_hdr;
	struct mms_fw_img **img;
	u8 ver_file[MMS_FW_MAX_SECT_NUM * 2];
	int i = 0;
	int offset = sizeof(struct mms_bin_hdr);

	sec_cmd_set_default_result(sec);

	request_firmware(&fw, fw_name, &info->client->dev);

	if (!fw)
		goto EXIT;

	fw_hdr = (struct mms_bin_hdr *)fw->data;
	if (fw_hdr->section_num > MMS_FW_MAX_SECT_NUM) {
		release_firmware(fw);
		goto EXIT;
	}
	img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);

	for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img)) {
		img[i] = (struct mms_fw_img *)(fw->data + offset);
		ver_file[i * 2] = ((img[i]->version) >> 8) & 0xFF;
		ver_file[i * 2 + 1] = (img[i]->version) & 0xFF;
	}

	release_firmware(fw);

	snprintf(buff, sizeof(buff), "ME%02X%02X%02X", info->dtdata->panel, ver_file[3], ver_file[5]);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "FW_VER_BIN");

	sec->cmd_state = SEC_CMD_STATUS_OK;

	kfree(img);
	return;

EXIT:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;

	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buff, sec->cmd_state);
	return;
}

/**
 * Command : Get firmware version from IC
 */
static void cmd_get_fw_ver_ic(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[16] = { 0 };
	u8 rbuf[16];

	sec_cmd_set_default_result(sec);

	if (mms_get_fw_version(info, rbuf)) {
		snprintf(buff, sizeof(buff), "%s", "NG");
		sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	info->boot_ver_ic = rbuf[1];
	info->core_ver_ic = rbuf[3];
	info->config_ver_ic = rbuf[5];

	snprintf(buff, sizeof(buff),"ME%02X%02X%02X",
		info->dtdata->panel, info->core_ver_ic, info->config_ver_ic);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "FW_VER_IC");

	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buff, sec->cmd_state);
}

/**
 * Command : Get chip vendor
 */
static void cmd_get_chip_vendor(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	char buf[64] = { 0 };

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), "MELFAS");
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "IC_VENDOR");
	
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);

	return;
}

static void check_connection(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	sec_cmd_set_default_result(sec);

	if (mms_run_test(info, MIP_TEST_TYPE_OPEN))
		goto EXIT;

	input_info(true, &info->client->dev, "%s: connection check(%d)\n", __func__, info->image_buf[0]);

	if (!info->image_buf[0])
		goto EXIT;
	
	sprintf(buf, "%s", "OK");
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_info(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);

	return;

EXIT:
	sprintf(buf, "%s", "NG");
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Get chip name
 */
static void cmd_get_chip_name(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };
	//u8 rbuf[64];

	sec_cmd_set_default_result(sec);

	snprintf(buf, sizeof(buf), CHIP_NAME);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "IC_NAME");

	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);

	return;
}

static void cmd_get_config_ver(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	sec_cmd_set_default_result(sec);
	snprintf(buf, sizeof(buf), "%s_ME_%02d%02d",
		info->product_name, info->fw_month, info->fw_date);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));

	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

static void get_checksum_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };
	u8 rbuf[64];
	u8 wbuf[64];
	int val;

	sec_cmd_set_default_result(sec);

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_CHECKSUM_REALTIME;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	val = rbuf[0];

	snprintf(buf, sizeof(buf), "%d", val);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));

	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	input_err(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}
/**
 * Command : Get X ch num
 */
static void cmd_get_x_num(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };
	u8 rbuf[64];
	u8 wbuf[64];
	int val;

	sec_cmd_set_default_result(sec);

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_NODE_NUM_X;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	val = rbuf[0];

	snprintf(buf, sizeof(buf), "%d", val);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));

	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Get Y ch num
 */
static void cmd_get_y_num(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };
	u8 rbuf[64];
	u8 wbuf[64];
	int val;

	sec_cmd_set_default_result(sec);

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_NODE_NUM_Y;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	val = rbuf[0];

	snprintf(buf, sizeof(buf), "%d", val);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));

	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Get X resolution
 */
static void cmd_get_max_x(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };
	u8 rbuf[64];
	u8 wbuf[64];
	int val;

	sec_cmd_set_default_result(sec);

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 2)) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	val = (rbuf[0]) | (rbuf[1] << 8);

	snprintf(buf, sizeof(buf), "%d", val);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));

	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Get Y resolution
 */
static void cmd_get_max_y(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };
	u8 rbuf[64];
	u8 wbuf[64];
	int val;

	sec_cmd_set_default_result(sec);

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_Y;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 2)) {
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	val = (rbuf[0]) | (rbuf[1] << 8);

	snprintf(buf, sizeof(buf), "%d", val);
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));

	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Power off
 */
static void cmd_module_off_master(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	sec_cmd_set_default_result(sec);

	mms_power_control(info, 0);

	snprintf(buf, sizeof(buf), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Power on
 */
static void cmd_module_on_master(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	sec_cmd_set_default_result(sec);

	mms_power_control(info, 1);

	snprintf(buf, sizeof(buf), "%s", "OK");
	sec->cmd_state = SEC_CMD_STATUS_OK;

	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Read intensity image
 */
static void cmd_read_intensity(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	int min = 999999;
	int max = -999999;
	int i = 0;

	sec_cmd_set_default_result(sec);

	if (mms_get_image(info, MIP_IMG_TYPE_INTENSITY)) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];

		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	snprintf(buf, sizeof(buf), "%d,%d", min, max);
	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Get intensity data
 */
static void cmd_get_intensity(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	int x = sec->cmd_param[0];
	int y = sec->cmd_param[1];
	int idx = 0;

	sec_cmd_set_default_result(sec);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = y * info->node_x + x;

	snprintf(buf, sizeof(buf), "%d", info->image_buf[idx]);
	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Read rawdata image
 */
static void cmd_read_rawdata(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	int min = 999999;
	int max = -999999;
	int i = 0;

	sec_cmd_set_default_result(sec);

	if (mms_get_image(info, MIP_IMG_TYPE_RAWDATA)) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];

		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	snprintf(buf, sizeof(buf), "%d,%d", min, max);
	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Get rawdata
 */
static void cmd_get_rawdata(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	int x = sec->cmd_param[0];
	int y = sec->cmd_param[1];
	int idx = 0;

	sec_cmd_set_default_result(sec);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = y * info->node_x + x;

	snprintf(buf, sizeof(buf), "%d", info->image_buf[idx]);
	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

/**
 * Command : Run cm delta test
 */
static void cmd_run_test_cm_delta(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	int min = 999999;
	int max = -999999;
	int i = 0;

	sec_cmd_set_default_result(sec);

	if (mms_run_test(info, MIP_TEST_TYPE_CM_DELTA)) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];

		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	snprintf(buf, sizeof(buf), "%d,%d", min, max);
	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buf, strnlen(buf, sizeof(buf)), "CM_DELTA");	
}

/**
 * Command : Get result of cm delta test
 */
static void cmd_get_cm_delta(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };
	int x = sec->cmd_param[0];
	int y = sec->cmd_param[1];
	int idx = 0;

	sec_cmd_set_default_result(sec);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = y * info->node_x + x;

	snprintf(buf, sizeof(buf), "%d", info->image_buf[idx]);
	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
}

/**
 * Command : Run cm abs test
 */
static void cmd_run_test_cm_abs(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	char buf[64] = { 0 };

	int min = 999999;
	int max = -999999;
	int i = 0;

	sec_cmd_set_default_result(sec);

	if (mms_run_test(info, MIP_TEST_TYPE_CM_ABS)) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	for (i = 0; i < (info->node_x * info->node_y); i++) {
		if (info->image_buf[i] > max)
			max = info->image_buf[i];

		if (info->image_buf[i] < min)
			min = info->image_buf[i];
	}

	snprintf(buf, sizeof(buf), "%d,%d", min, max);
	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
}

/**
 * Command : Get result of cm abs test
 */
static void cmd_get_cm_abs(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	int x = sec->cmd_param[0];
	int y = sec->cmd_param[1];
	int idx = 0;

	sec_cmd_set_default_result(sec);

	if ((x < 0) || (x >= info->node_x) || (y < 0) || (y >= info->node_y)) {
		snprintf(buf, sizeof(buf), "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto EXIT;
	}

	idx = y * info->node_x + x;

	snprintf(buf, sizeof(buf), "%d", info->image_buf[idx]);
	sec->cmd_state = SEC_CMD_STATUS_OK;

EXIT:
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));
	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

static void cmd_get_threshold(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buf[64] = { 0 };

	sec_cmd_set_default_result(sec);
	snprintf(buf, sizeof(buf), "55");
	sec_cmd_set_cmd_result(sec, buf, strnlen(buf, sizeof(buf)));

	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_err(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buf, sec->cmd_state);
}

static void get_intensity_all_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	int ret;
	int length;

	sec_cmd_set_default_result(sec);

	ret = mms_get_image(info, MIP_IMG_TYPE_INTENSITY);
	if (ret < 0) {
		input_err(true, &info->client->dev, "%s: failed to read intensity, %d\n", __func__, ret);
		sprintf(info->print_buf, "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	length = strlen(info->print_buf);
	sec->cmd_state = SEC_CMD_STATUS_OK;

out:
	sec_cmd_set_cmd_result(sec, info->print_buf, length);
}

static void get_rawdata_all_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	int ret;
	int length;

	sec_cmd_set_default_result(sec);

	ret = mms_get_image(info, MIP_IMG_TYPE_RAWDATA);
	if (ret < 0) {
		input_err(true, &info->client->dev, "%s: failed to read raw data, %d\n", __func__, ret);
		sprintf(info->print_buf, "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;		
		goto out;
	}

	length = strlen(info->print_buf);
	sec->cmd_state = SEC_CMD_STATUS_OK;
out:
	sec_cmd_set_cmd_result(sec, info->print_buf, length);
}

static void get_cm_delta_all_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	int ret;
	int length;

	sec_cmd_set_default_result(sec);

	ret = mms_run_test(info, MIP_TEST_TYPE_CM_DELTA);
	if (ret < 0) {
		input_err(true, &info->client->dev, "%s: failed to read cm delta, %d\n", __func__, ret);
		sprintf(info->print_buf, "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;		
		goto out;
	}

	length = strlen(info->print_buf);
	sec->cmd_state = SEC_CMD_STATUS_OK;
out:
	sec_cmd_set_cmd_result(sec, info->print_buf, length);
}

static void get_cm_abs_all_data(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	int ret;
	int length;

	sec_cmd_set_default_result(sec);

	ret = mms_run_test(info, MIP_TEST_TYPE_CM_ABS);
	if (ret < 0) {
		input_err(true, &info->client->dev, "%s: failed to read cm abs, %d\n", __func__, ret);
		snprintf(info->print_buf, PAGE_SIZE, "%s", "NG");
		sec->cmd_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	length = strlen(info->print_buf);
	sec->cmd_state = SEC_CMD_STATUS_OK;
out:
	sec_cmd_set_cmd_result(sec, info->print_buf, length);
}

static void dead_zone_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };
	int enable = sec->cmd_param[0];
	u8 wbuf[4];
	int status;

	sec_cmd_set_default_result(sec);

	input_info(true, &info->client->dev, "%s %d\n", __func__, enable);

	if (enable)
		status = 0;
	else
		status = 2;

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_DISABLE_EDGE_EXPAND;
	wbuf[2] = status;

	if ((enable == 0) || (enable == 1)) {
		if (mms_i2c_write(info, wbuf, 3)) {
			input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
			goto out;
		} else
			input_info(true, &info->client->dev, "%s - value[%d]\n", __func__, wbuf[2]);
	} else {
		input_err(true, &info->client->dev, "%s [ERROR] Unknown value[%d]\n", __func__, status);
		goto out;
	}
	input_dbg(true, &info->client->dev, "%s [DONE]\n", __func__);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);
	return;
out:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_exit(sec);
}

#ifdef GLOVE_MODE
static void glove_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };
	int length = 0;
	int enable = sec->cmd_param[0];
	u8 wbuf[4];

	sec_cmd_set_default_result(sec);

	input_info(true, &info->client->dev, "%s %d\n", __func__, enable);

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_GLOVE_MODE;
	wbuf[2] = enable;

	if ((enable == 0) || (enable == 1)) {
		if (mms_i2c_write(info, wbuf, 3)) {
			input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
			goto out;
		} else
			input_info(true, &info->client->dev, "%s - value[%d]\n", __func__, wbuf[2]);
	} else {
		input_err(true, &info->client->dev, "%s [ERROR] Unknown value[%d]\n", __func__, enable);
		goto out;
	}
	input_dbg(true, &info->client->dev, "%s [DONE]\n", __func__);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);
	return;
out:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_exit(sec);
}
#endif

#ifdef COVER_MODE
static void clear_cover_mode(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };
	int enable = sec->cmd_param[0];
	u8 wbuf[4];

	sec_cmd_set_default_result(sec);

	input_info(true, &info->client->dev, "%s %d\n", __func__, enable);

	if (!info->enabled) {
		input_err(true, &info->client->dev,
			"%s : tsp disabled\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "NG");
		goto out;
	}

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_WINDOW_MODE;
	wbuf[2] = enable;

	if ((enable >= 0) || (enable <= 3)) {
		if (mms_i2c_write(info, wbuf, 3)) {
			input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
			goto out;
		} else{
			input_info(true, &info->client->dev, "%s - value[%d]\n", __func__, wbuf[2]);
		}
	} else {
		input_err(true, &info->client->dev, "%s [ERROR] Unknown value[%d]\n", __func__, enable);
		goto out;
	}

	if (enable > 0)
		info->cover_mode = true;
	else
		info->cover_mode = false;

	input_dbg(true, &info->client->dev, "%s [DONE]\n", __func__);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);
	return;
out:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_exit(sec);
}
#endif

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
static void tui_mode_cmd(struct mms_ts_info *info)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	char buff[16] = "TUImode:FAIL";

	sec_cmd_set_default_result(sec);
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));

	sec->cmd_state = SEC_CMD_STATUS_WAITING;
	input_err(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
		  (int)strnlen(buff, sizeof(buff)));
}
#endif

static void spay_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };

	sec_cmd_set_default_result(sec);

	if (!info->dtdata->support_lpm) {
		input_err(true, &info->client->dev, "%s not supported\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "not supported");
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		goto out;
	}

	if (sec->cmd_param[0]) {
		info->lowpower_mode = true;
		info->lowpower_flag = info->lowpower_flag | MMS_LPM_FLAG_SPAY;
	} else {
		info->lowpower_flag = info->lowpower_flag & ~(MMS_LPM_FLAG_SPAY);
		if (!info->lowpower_flag)
			info->lowpower_mode = false;
	}

	input_info(true, &info->client->dev, "%s: %s mode, %x\n",
			__func__, info->lowpower_mode ? "LPM" : "normal",
			info->lowpower_flag);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);

out:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
}

static void aod_enable(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };

	sec_cmd_set_default_result(sec);

	if (!info->dtdata->support_lpm) {
		input_err(true, &info->client->dev, "%s not supported\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "not supported");
		sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
		goto out;
	}

	if (sec->cmd_param[0]) {
		info->lowpower_mode = true;
		info->lowpower_flag = info->lowpower_flag | MMS_LPM_FLAG_AOD;
	} else {
		info->lowpower_flag = info->lowpower_flag & ~(MMS_LPM_FLAG_AOD);
		if (!info->lowpower_flag)
			info->lowpower_mode = false;
	}
	input_info(true, &info->client->dev, "%s: %s mode, %x\n",
			__func__, info->lowpower_mode ? "LPM" : "normal",
			info->lowpower_flag);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);

out:
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);
}

static void set_aod_rect(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };
	u8 data[11] = {0};
	int i;

	sec_cmd_set_default_result(sec);

	if (!info->enabled) {
		input_err(true, &info->client->dev,
			  "%s: [ERROR] Touch is stopped\n", __func__);
		snprintf(buff, sizeof(buff), "%s", "TSP turned off");
		goto out;
	}

	input_info(true, &info->client->dev, "%s: w:%d, h:%d, x:%d, y:%d\n",
			__func__, sec->cmd_param[0], sec->cmd_param[1],
			sec->cmd_param[2], sec->cmd_param[3]);

	data[0] = MIP_R0_AOT;
	data[1] = MIP_R0_AOT_BOX_W;
	for (i = 0; i < 4; i++) {
		data[i * 2 + 2] = sec->cmd_param[i] & 0xFF;
		data[i * 2 + 3] = (sec->cmd_param[i] >> 8) & 0xFF;
	}

	disable_irq(info->client->irq);

	if (mms_i2c_write(info, data, 10)) {
		input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
		enable_irq(info->client->irq);
		goto out;
	}

	enable_irq(info->client->irq);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);
	return;

out:
	enable_irq(info->client->irq);

	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);	
}


static void get_aod_rect(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };
	u8 wbuf[16];
	u8 rbuf[16];
	u16 rect_data[4] = {0, };
	int i;

	sec_cmd_set_default_result(sec);

	disable_irq(info->client->irq);

	wbuf[0] = MIP_R0_AOT;
	wbuf[1] = MIP_R0_AOT_BOX_W;

	if (mms_i2c_read(info, wbuf, 2, rbuf, 8)) {
		input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
		goto out;
	}

	enable_irq(info->client->irq);

	for (i = 0; i < 4; i++)
		rect_data[i] = (rbuf[i * 2 + 1] & 0xFF) << 8 | (rbuf[i * 2] & 0xFF);

	input_info(true, &info->client->dev, "%s: w:%d, h:%d, x:%d, y:%d\n",
			__func__, rect_data[0], rect_data[1], rect_data[2], rect_data[3]);

	snprintf(buff, sizeof(buff), "%s", "OK");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_OK;
	sec_cmd_set_cmd_exit(sec);
	return;
out:
	enable_irq(info->client->irq);
	
	snprintf(buff, sizeof(buff), "%s", "NG");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_FAIL;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: %s\n", __func__, buff);	
}

/**
 * Command : Check SRAM failure
 */
static void cmd_check_sram(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[64] = { 0 };
	int val;

	sec_cmd_set_default_result(sec);

	val = (int) info->sram_addr[0];

	if (val != 0)
		snprintf(buff, sizeof(buff), "0x%x", val);
	else
		snprintf(buff, sizeof(buff), "%s", "0");

	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	if (sec->cmd_all_factory_state == SEC_CMD_STATUS_RUNNING)
		sec_cmd_set_cmd_result_all(sec, buff, strnlen(buff, sizeof(buff)), "SRAM");

	sec->cmd_state = SEC_CMD_STATUS_OK;

	input_dbg(true, &info->client->dev, "%s - cmd[%s] state[%d]\n",
		__func__, buff, sec->cmd_state);
		
	return;
}

/**
 * Command : Unknown cmd
 */
static void cmd_unknown_cmd(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[16] = { 0 };

	sec_cmd_set_default_result(sec);
	snprintf(buff, sizeof(buff), "%s", "not_support_cmd");
	sec_cmd_set_cmd_result(sec, buff, strnlen(buff, sizeof(buff)));
	sec->cmd_state = SEC_CMD_STATUS_NOT_APPLICABLE;
	sec_cmd_set_cmd_exit(sec);

	input_info(true, &info->client->dev, "%s: \"%s\"\n", __func__, buff);
}

static void factory_cmd_result_all(void *device_data)
{
	struct sec_cmd_data *sec = (struct sec_cmd_data *)device_data;
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	char buff[16] = { 0 };

	sec->item_count = 0;
	memset(sec->cmd_result_all, 0x00, SEC_CMD_RESULT_STR_LEN);

	if (!info->enabled) {
		input_err(true, &info->client->dev, "%s: IC is power off\n", __func__);
		sec->cmd_all_factory_state = SEC_CMD_STATUS_FAIL;
		goto out;
	}

	sec->cmd_all_factory_state = SEC_CMD_STATUS_RUNNING;

	snprintf(buff, sizeof(buff), "%d", info->dtdata->item_version);
	sec_cmd_set_cmd_result_all(sec, buff, sizeof(buff), "ITEM_VERSION");

	cmd_get_chip_vendor(sec);
	cmd_get_chip_name(sec);
	cmd_get_fw_ver_bin(sec);
	cmd_get_fw_ver_ic(sec);

	cmd_run_test_cm_delta(sec);
	cmd_check_sram(sec);

	sec->cmd_all_factory_state = SEC_CMD_STATUS_OK;

out:
	input_info(true, &info->client->dev, "%s: %d%s\n", __func__, sec->item_count, sec->cmd_result_all);
}

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
static void tui_mode_cmd(struct mms_ts_info *info);
#endif

/**
 * List of command functions
 */
static struct sec_cmd sec_cmds[] = {
	{SEC_CMD("fw_update", cmd_fw_update),},
	{SEC_CMD("get_fw_ver_bin", cmd_get_fw_ver_bin),},
	{SEC_CMD("get_fw_ver_ic", cmd_get_fw_ver_ic),},
	{SEC_CMD("get_chip_vendor", cmd_get_chip_vendor),},
	{SEC_CMD("get_chip_name", cmd_get_chip_name),},
	{SEC_CMD("get_checksum_data", get_checksum_data),},
	{SEC_CMD("get_x_num", cmd_get_x_num),},
	{SEC_CMD("get_y_num", cmd_get_y_num),},
	{SEC_CMD("get_max_x", cmd_get_max_x),},
	{SEC_CMD("get_max_y", cmd_get_max_y),},
	{SEC_CMD("module_off_master", cmd_module_off_master),},
	{SEC_CMD("module_on_master", cmd_module_on_master),},
	{SEC_CMD("run_intensity_read", cmd_read_intensity),},
	{SEC_CMD("get_intensity", cmd_get_intensity),},
	{SEC_CMD("run_rawdata_read", cmd_read_rawdata),},
	{SEC_CMD("get_rawdata", cmd_get_rawdata),},
	{SEC_CMD("run_inspection_read", cmd_run_test_cm_delta),},
	{SEC_CMD("get_inspection", cmd_get_cm_delta),},
	{SEC_CMD("run_cm_delta_read", cmd_run_test_cm_delta),},
	{SEC_CMD("get_cm_delta", cmd_get_cm_delta),},
	{SEC_CMD("run_cm_abs_read", cmd_run_test_cm_abs),},
	{SEC_CMD("get_cm_abs", cmd_get_cm_abs),},
	{SEC_CMD("get_config_ver", cmd_get_config_ver),},
	{SEC_CMD("get_threshold", cmd_get_threshold),},
	{SEC_CMD("get_intensity_all_data", get_intensity_all_data),},
	{SEC_CMD("get_rawdata_all_data", get_rawdata_all_data),},
	{SEC_CMD("get_cm_delta_all_data", get_cm_delta_all_data),},
	{SEC_CMD("get_cm_abs_all_data", get_cm_abs_all_data),},
	{SEC_CMD("dead_zone_enable", dead_zone_enable),},
#ifdef GLOVE_MODE
	{SEC_CMD("glove_mode", glove_mode),},
#endif
#ifdef COVER_MODE
	{SEC_CMD("clear_cover_mode", clear_cover_mode),},
#endif
	{SEC_CMD("spay_enable", spay_enable),},
	{SEC_CMD("aod_enable", aod_enable),},
	{SEC_CMD("set_aod_rect", set_aod_rect),},
	{SEC_CMD("get_aod_rect", get_aod_rect),},
	{SEC_CMD("check_sram", cmd_check_sram),},
	{SEC_CMD("check_connection", check_connection),},
	{SEC_CMD("factory_cmd_result_all", factory_cmd_result_all),},	
	{SEC_CMD("not_support_cmd", cmd_unknown_cmd),},
};

static DEVICE_ATTR(scrub_pos, S_IRUGO, scrub_position_show, NULL);

static ssize_t read_multi_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	input_info(true, &info->client->dev, "%s: %d\n", __func__, info->multi_count);

	return snprintf(buf, PAGE_SIZE, "%d", info->multi_count);
}

static ssize_t clear_multi_count_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	info->multi_count = 0;
	input_info(true, &info->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t read_comm_err_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	input_info(true, &info->client->dev, "%s: %d\n", __func__, info->comm_err_count);

	return snprintf(buf, PAGE_SIZE, "%d", info->comm_err_count);
}

static ssize_t clear_comm_err_count_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	info->comm_err_count = 0;

	input_info(true, &info->client->dev, "%s: clear\n", __func__);

	return count;
}

static ssize_t read_module_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	return snprintf(buf, PAGE_SIZE, "ME%02X%02X%02X0000",
		info->dtdata->panel, info->core_ver_ic, info->config_ver_ic);
}

static ssize_t read_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "MELFAS");
}

static ssize_t sensitivity_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);
	
	u8 wbuf[64];
	u8 rbuf[64];
	int ret;
	int i;
	u16 sTspSensitivity[5] = {0, };

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_TS_READ_SENSITIVITY_VALUE;			
			
	ret = mms_i2c_read(info, wbuf, 2, rbuf, 10);
	
	if (ret != 0) {
		input_err(true, &info->client->dev, "%s: i2c fail!, %d\n", __func__, ret);
		return ret;
	}
	
	for(i = 0; i < 5; i++)
		sTspSensitivity[i] = (rbuf[i * 2 + 1] & 0xFF) << 8 | (rbuf[i * 2] & 0xFF);
		
	input_info(true, &info->client->dev, "%s: sensitivity mode,%d,%d,%d,%d,%d\n", __func__,
		sTspSensitivity[0], sTspSensitivity[1], sTspSensitivity[2], sTspSensitivity[3], sTspSensitivity[4]);
		
	return snprintf(buf, PAGE_SIZE,"%d,%d,%d,%d,%d",
			sTspSensitivity[0], sTspSensitivity[1], sTspSensitivity[2], sTspSensitivity[3], sTspSensitivity[4]);
		
}

static ssize_t sensitivity_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

	struct sec_cmd_data *sec = dev_get_drvdata(dev);
	struct mms_ts_info *info = container_of(sec, struct mms_ts_info, sec);

	u8 wbuf[64];
	int ret;
	//u8 temp;
	unsigned long value = 0;

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_GLOVE_MODE;

	if (count > 2)
		return -EINVAL;

	ret = kstrtoul(buf, 10, &value);
	if (ret != 0)
		return ret;

	input_err(true, &info->client->dev, "%s: enable:%d\n", __func__, value);
	
	if (value == 1) {
		wbuf[2] = 1; // enable
		//temp = 0x1;
		ret = mms_i2c_write(info, wbuf, 3);
		if (ret < 0) {
			input_err(true, &info->client->dev, "%s: send sensitivity mode on fail!\n", __func__);
			return ret;
		}
		input_info(true, &info->client->dev, "%s: enable end\n", __func__);
	} else {
		wbuf[2] = 0; // disable
		//temp = 0x1;
		ret = mms_i2c_write(info, wbuf, 3);
		if (ret < 0) {
			input_err(true, &info->client->dev, "%s: send sensitivity mode off fail!\n", __func__);
			return ret;
		}
		input_info(true, &info->client->dev, "%s: disable end\n", __func__);
	}

	input_info(true, &info->client->dev, "%s: done\n", __func__);

	return count;
}

static DEVICE_ATTR(multi_count, S_IRUGO | S_IWUSR | S_IWGRP, read_multi_count_show, clear_multi_count_store);
static DEVICE_ATTR(comm_err_count, S_IRUGO | S_IWUSR | S_IWGRP, read_comm_err_count_show, clear_comm_err_count_store);
static DEVICE_ATTR(module_id, S_IRUGO, read_module_id_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, read_vendor_show, NULL);
static DEVICE_ATTR(sensitivity_mode, S_IRUGO | S_IWUSR | S_IWGRP, sensitivity_mode_show, sensitivity_mode_store);

/**
 * Sysfs - cmd attr info
 */
static struct attribute *mms_cmd_attr[] = {
	&dev_attr_scrub_pos.attr,
	&dev_attr_multi_count.attr,
	&dev_attr_comm_err_count.attr,
	&dev_attr_module_id.attr,
	&dev_attr_vendor.attr,
	&dev_attr_sensitivity_mode.attr,
	NULL,
};

/**
 * Sysfs - cmd attr group info
 */
static const struct attribute_group cmd_attr_group = {
	.attrs = mms_cmd_attr,
};

/**
 * Create sysfs command functions
 */
int mms_sysfs_cmd_create(struct mms_ts_info *info)
{
	int retval;

	info->print_buf = kzalloc(sizeof(u8) * 4096, GFP_KERNEL);	

	retval = sec_cmd_init(&info->sec, sec_cmds,
			ARRAY_SIZE(sec_cmds), SEC_CLASS_DEVT_TSP);
	if (retval < 0) {
		input_err(true, &info->client->dev,
				"%s: Failed to sec_cmd_init\n", __func__);
		goto exit;
	}

	retval = sysfs_create_group(&info->sec.fac_dev->kobj,
			&cmd_attr_group);
	if (retval < 0) {
		input_err(true, &info->client->dev,
				"%s: Failed to create sysfs attributes\n", __func__);
		goto exit;
	}

	retval = sysfs_create_link(&info->sec.fac_dev->kobj,
			&info->input_dev->dev.kobj, "input");
	if (retval < 0) {
		input_err(true, &info->client->dev,
				"%s: Failed to create input symbolic link\n",
				__func__);
		goto exit;
	}

	return 0;
exit:
	return retval;	
}

/**
 * Remove sysfs command functions
 */
void mms_sysfs_cmd_remove(struct mms_ts_info *info)
{
	input_err(true, &info->client->dev, "%s\n", __func__);

	sysfs_delete_link(&info->sec.fac_dev->kobj, &info->input_dev->dev.kobj, "input");

	sysfs_remove_group(&info->sec.fac_dev->kobj,
			&cmd_attr_group);

	sec_cmd_exit(&info->sec, SEC_CLASS_DEVT_TSP);

}

#endif
