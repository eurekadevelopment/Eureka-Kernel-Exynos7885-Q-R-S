/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <exynos-fimc-is-sensor.h>
#include "fimc-is-hw.h"
#include "fimc-is-core.h"
#include "fimc-is-param.h"
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-resourcemgr.h"
#include "fimc-is-dt.h"
#include "fimc-is-cis-sr846.h"
#include "fimc-is-cis-sr846-setA.h"
#include "fimc-is-cis-sr846-setB.h"

#include "fimc-is-helper-i2c.h"

#define SENSOR_NAME "SR846"
/* #define DEBUG_SR846_PLL */

#define POLL_TIME_MS 1
#define STREAM_OFF_POLL_TIME_MS 100

static const struct v4l2_subdev_ops subdev_ops;

static const u32 *sensor_sr846_global;
static u32 sensor_sr846_global_size;
static const u32 **sensor_sr846_setfiles;
static const u32 *sensor_sr846_setfile_sizes;
static u32 sensor_sr846_max_setfile_num;
static const struct sensor_pll_info **sensor_sr846_pllinfos;

static const u32 *sensor_sr846_fsync_normal;
static u32 sensor_sr846_fsync_normal_size;
static const u32 *sensor_sr846_fsync_master;
static u32 sensor_sr846_fsync_master_size;
static const u32 *sensor_sr846_fsync_slave;
static u32 sensor_sr846_fsync_slave_size;

static void sensor_sr846_cis_data_calculation(const struct sensor_pll_info *pll_info, cis_shared_data *cis_data)
{
	u32 pll_voc_a = 0, vt_pix_clk_hz = 0;
	u32 frame_rate = 0, max_fps = 0, frame_valid_us = 0;

	BUG_ON(!pll_info);

	/* 1. mipi data rate calculation (Mbps/Lane) */
	/* ToDo: using output Pixel Clock Divider Value */
	/* pll_voc_b = pll_info->ext_clk / pll_info->secnd_pre_pll_clk_div * pll_info->secnd_pll_multiplier * 2;
	op_sys_clk_hz = pll_voc_b / pll_info->op_sys_clk_div;
	if(gpsSensorExInfo) {
		gpsSensorExInfo->uiMIPISpeedBps = op_sys_clk_hz;
		gpsSensorExInfo->uiMCLK = sensorInfo.ext_clk;
	} */

	/* 2. pixel rate calculation (Mpps) */
	pll_voc_a = pll_info->ext_clk / pll_info->pre_pll_clk_div * pll_info->pll_multiplier;
	vt_pix_clk_hz = (pll_voc_a / pll_info->vt_pix_clk_div / pll_info->vt_sys_clk_div) * 4;

	dbg_sensor(1, "ext_clock(%d) / pre_pll_clk_div(%d) * pll_multiplier(%d) = pll_voc_a(%d)\n",
						pll_info->ext_clk, pll_info->pre_pll_clk_div,
						pll_info->pll_multiplier, pll_voc_a);
	dbg_sensor(1, "pll_voc_a(%d) / (vt_sys_clk_div(%d) * vt_pix_clk_div(%d)) = pixel clock (%d hz)\n",
						pll_voc_a, pll_info->vt_sys_clk_div,
						pll_info->vt_pix_clk_div, vt_pix_clk_hz);

	/* 3. the time of processing one frame calculation (us) */
	cis_data->min_frame_us_time = (pll_info->frame_length_lines * pll_info->line_length_pck
					/ (vt_pix_clk_hz / (1000 * 1000)));
	cis_data->cur_frame_us_time = cis_data->min_frame_us_time;

	/* 4. FPS calculation */
	frame_rate = vt_pix_clk_hz / (pll_info->frame_length_lines * pll_info->line_length_pck);
	dbg_sensor(1, "frame_rate (%d) = vt_pix_clk_hz(%d) / (pll_info->frame_length_lines(%d) * pll_info->line_length_pck(%d))\n",
		frame_rate, vt_pix_clk_hz, pll_info->frame_length_lines, pll_info->line_length_pck);

	/* calculate max fps */
	max_fps = (vt_pix_clk_hz * 10) / (pll_info->frame_length_lines * pll_info->line_length_pck);
	max_fps = (max_fps % 10 >= 5 ? frame_rate + 1 : frame_rate);

	cis_data->pclk = vt_pix_clk_hz;
	cis_data->max_fps = max_fps;
	cis_data->frame_length_lines = pll_info->frame_length_lines;
	cis_data->line_length_pck = pll_info->line_length_pck;
	cis_data->line_readOut_time = sensor_cis_do_div64((u64)cis_data->line_length_pck * (u64)(1000 * 1000 * 1000), cis_data->pclk);
	cis_data->rolling_shutter_skew = (cis_data->cur_height - 1) * cis_data->line_readOut_time;
	cis_data->stream_on = false;

	/* Frame valid time calcuration */
	frame_valid_us = sensor_cis_do_div64((u64)cis_data->cur_height * (u64)cis_data->line_length_pck * (u64)(1000 * 1000), cis_data->pclk);
	cis_data->frame_valid_us_time = (int)frame_valid_us;

	dbg_sensor(1, "%s\n", __func__);
	dbg_sensor(1, "Sensor size(%d x %d) setting: SUCCESS!\n",
					cis_data->cur_width, cis_data->cur_height);
	dbg_sensor(1, "Frame Valid(us): %d\n", frame_valid_us);
	dbg_sensor(1, "rolling_shutter_skew: %lld\n", cis_data->rolling_shutter_skew);

	dbg_sensor(1, "Fps: %d, max fps(%d)\n", frame_rate, cis_data->max_fps);
	dbg_sensor(1, "min_frame_time(%d us)\n", cis_data->min_frame_us_time);
	dbg_sensor(1, "Pixel rate(Mbps): %d\n", cis_data->pclk / 1000000);
	/* dbg_sensor(1, "Mbps/lane : %d Mbps\n", pll_voc_b / pll_info->op_sys_clk_div / 1000 / 1000); */

	/* Frame period calculation */
	cis_data->frame_time = (cis_data->line_readOut_time * cis_data->cur_height / 1000);
	cis_data->rolling_shutter_skew = (cis_data->cur_height - 1) * cis_data->line_readOut_time;

	dbg_sensor(1, "[%s] frame_time(%d), rolling_shutter_skew(%lld)\n", __func__, cis_data->frame_time, cis_data->rolling_shutter_skew);

	/* Constant values */
	cis_data->min_fine_integration_time = SENSOR_SR846_FINE_INTEGRATION_TIME_MIN;
	cis_data->max_fine_integration_time = SENSOR_SR846_FINE_INTEGRATION_TIME_MAX;
	cis_data->min_coarse_integration_time = SENSOR_SR846_COARSE_INTEGRATION_TIME_MIN;
	cis_data->max_margin_coarse_integration_time = SENSOR_SR846_COARSE_INTEGRATION_TIME_MAX_MARGIN;
}

static int sensor_sr846_wait_stream_off_status(cis_shared_data *cis_data)
{
	int ret = 0;
	u32 timeout = 0;

	BUG_ON(!cis_data);

#define STREAM_OFF_WAIT_TIME 250
	while (timeout < STREAM_OFF_WAIT_TIME) {
		if (cis_data->is_active_area == false &&
				cis_data->stream_on == false) {
			pr_debug("actual stream off\n");
			break;
		}
		timeout++;
	}

	if (timeout == STREAM_OFF_WAIT_TIME) {
		pr_err("actual stream off wait timeout\n");
		ret = -1;
	}

	return ret;
}

/* CIS OPS */
int sensor_sr846_cis_init(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	u32 setfile_index = 0;
	cis_setting_info setinfo;
#ifdef USE_CAMERA_HW_BIG_DATA
	struct cam_hw_param *hw_param = NULL;
	struct fimc_is_device_sensor_peri *sensor_peri = NULL;
#endif

	setinfo.param = NULL;
	setinfo.return_value = 0;

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	if (!cis) {
		err("cis is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	BUG_ON(!cis->cis_data);
	memset(cis->cis_data, 0, sizeof(cis_shared_data));
	cis->rev_flag = false;

	ret = sensor_cis_check_rev(cis);
	if (ret < 0) {
#ifdef USE_CAMERA_HW_BIG_DATA
		sensor_peri = container_of(cis, struct fimc_is_device_sensor_peri, cis);
		if (sensor_peri)
			fimc_is_sec_get_hw_param(&hw_param, sensor_peri->module->position);
		if (hw_param)
			hw_param->i2c_sensor_err_cnt++;
#endif
		warn("sensor_sr846_check_rev is fail when cis init");
		cis->rev_flag = true;
		goto p_err;
	}

	cis->cis_data->cur_width = SENSOR_SR846_MAX_WIDTH;
	cis->cis_data->cur_height = SENSOR_SR846_MAX_HEIGHT;
	cis->cis_data->low_expo_start = 33000;
	cis->need_mode_change = false;

	sensor_sr846_cis_data_calculation(sensor_sr846_pllinfos[setfile_index], cis->cis_data);

	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_min_exposure_time, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] min exposure time : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_max_exposure_time, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] max exposure time : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_min_analog_gain, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] min again : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_max_analog_gain, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] max again : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_min_digital_gain, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] min dgain : %d\n", __func__, setinfo.return_value);
	setinfo.return_value = 0;
	CALL_CISOPS(cis, cis_get_max_digital_gain, subdev, &setinfo.return_value);
	dbg_sensor(1, "[%s] max dgain : %d\n", __func__, setinfo.return_value);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_sr846_cis_log_status(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client = NULL;
	u8 data8 = 0;
	u16 data16 = 0;

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	if (!cis) {
		err("cis is NULL");
		ret = -ENODEV;
		goto p_err;
	}

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -ENODEV;
		goto p_err;
	}

	pr_err("[SEN:DUMP] *******************************\n");
	ret = fimc_is_sensor_read8(client, 0x0F14, &data8);
	if (unlikely(!ret)) printk("[SEN:DUMP] sensor_id(%x)\n", data8);
	ret = fimc_is_sensor_read16(client, 0x0F16, &data16);
	if (unlikely(!ret)) printk("[SEN:DUMP] model_id(%x)\n", data16);
	pr_err("[SEN:DUMP] *******************************\n");

p_err:
	return ret;
}

#if USE_GROUP_PARAM_HOLD
static int sensor_sr846_cis_group_param_hold_func(struct v4l2_subdev *subdev, unsigned int hold)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;
	struct i2c_client *client = NULL;

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	if (hold == cis->cis_data->group_param_hold) {
		pr_debug("already group_param_hold (%d)\n", cis->cis_data->group_param_hold);
		goto p_err;
	}

	ret = fimc_is_sensor_write8(client, 0x0046, hold);
	if (ret < 0)
		goto p_err;

	cis->cis_data->group_param_hold = hold;
	ret = 1;
p_err:
	return ret;
}
#else
static inline int sensor_sr846_cis_group_param_hold_func(struct v4l2_subdev *subdev, unsigned int hold)
{
	return 0;
}
#endif

/* Input
 *	hold : true - hold, flase - no hold
 * Output
 *      return: 0 - no effect(already hold or no hold)
 *		positive - setted by request
 *		negative - ERROR value
 */
int sensor_sr846_cis_group_param_hold(struct v4l2_subdev *subdev, bool hold)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	ret = sensor_sr846_cis_group_param_hold_func(subdev, hold);
	if (ret < 0)
		goto p_err;

p_err:
	return ret;
}

int sensor_sr846_cis_set_global_setting(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	BUG_ON(!cis);

	/* setfile global setting is at camera entrance */
	ret = sensor_cis_set_registers(subdev, sensor_sr846_global, sensor_sr846_global_size);

	if (ret < 0) {
		err("sensor_sr846_set_registers fail!!");
		goto p_err;
	}

	dbg_sensor(1, "[%s] global setting done\n", __func__);

p_err:
	return ret;
}

int sensor_sr846_cis_mode_change(struct v4l2_subdev *subdev, u32 mode)
{
	int ret = 0;
	u32 max_setfile_num = 0;
	struct fimc_is_cis *cis = NULL;

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	max_setfile_num = sensor_sr846_max_setfile_num;

	if (mode > max_setfile_num) {
		err("invalid mode(%d)!!", mode);
		ret = -EINVAL;
		goto p_err;
	}

	/* If check_rev fail when cis_init, one more check_rev in mode_change */
	if (cis->rev_flag == true) {
		cis->rev_flag = false;
		ret = sensor_cis_check_rev(cis);
		if (ret < 0) {
			err("sensor_sr846_check_rev is fail");
			goto p_err;
		}
	}

	sensor_sr846_cis_data_calculation(sensor_sr846_pllinfos[mode], cis->cis_data);
	ret = sensor_cis_set_registers(subdev, sensor_sr846_setfiles[mode], sensor_sr846_setfile_sizes[mode]);

	if (ret < 0) {
		err("sensor_sr846_set_registers fail!!");
		goto p_err;
	}

	dbg_sensor(1, "[%s] mode changed(%d)\n", __func__, mode);

p_err:
	return ret;
}

/* TODO: Sensor set size sequence(sensor done, sensor stop, 3AA done in FW case */
int sensor_sr846_cis_set_size(struct v4l2_subdev *subdev, cis_shared_data *cis_data)
{
	int ret = 0;
	bool binning = false;
	u32 ratio_w = 0, ratio_h = 0, start_x = 0, start_y = 0, end_x = 0, end_y = 0;
	struct i2c_client *client = NULL;
	struct fimc_is_cis *cis = NULL;
#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif
	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	BUG_ON(!cis);

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	if (unlikely(!cis_data)) {
		err("cis data is NULL");
		if (unlikely(!cis->cis_data)) {
			ret = -EINVAL;
			goto p_err;
		} else {
			cis_data = cis->cis_data;
		}
	}

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	/* Wait actual stream off */
	ret = sensor_sr846_wait_stream_off_status(cis_data);
	if (ret) {
		err("Must stream off\n");
		ret = -EINVAL;
		goto p_err;
	}

	binning = cis_data->binning;
	if (binning) {
		ratio_w = (SENSOR_SR846_MAX_WIDTH / cis_data->cur_width);
		ratio_h = (SENSOR_SR846_MAX_HEIGHT / cis_data->cur_height);
	} else {
		ratio_w = 1;
		ratio_h = 1;
	}

	if (((cis_data->cur_width * ratio_w) > SENSOR_SR846_MAX_WIDTH) ||
		((cis_data->cur_height * ratio_h) > SENSOR_SR846_MAX_HEIGHT)) {
		err("Config max sensor size over~!!\n");
		ret = -EINVAL;
		goto p_err;
	}

	start_x = ((SENSOR_SR846_MAX_WIDTH - cis_data->cur_width * ratio_w) / 2) & (~0x1);
	start_y = ((SENSOR_SR846_MAX_WIDTH - cis_data->cur_height * ratio_h) / 2) & (~0x1);
	end_x = start_x + (cis_data->cur_width * ratio_w - 1);
	end_y = start_y + (cis_data->cur_height * ratio_h - 1);

	if (!(end_x & (0x1)) || !(end_y & (0x1))) {
		err("Sensor pixel end address must odd\n");
		ret = -EINVAL;
		goto p_err;
	}
	info("start_x(%x)end_x(%x)start_y(%x)end_y(%x)\n", start_x, end_x, start_y, end_y);

	ret = fimc_is_sensor_write16(client, 0x0120, start_x);
	if (ret < 0)
		 goto p_err;
	ret = fimc_is_sensor_write16(client, 0x0026, start_y);
	if (ret < 0)
		 goto p_err;
	ret = fimc_is_sensor_write16(client, 0x0122, end_x);
	if (ret < 0)
		 goto p_err;
	ret = fimc_is_sensor_write16(client, 0x002C, end_y);
	if (ret < 0)
		 goto p_err;

	/* 3. output address setting */

	ret = fimc_is_sensor_write16(client, 0x0A12, cis_data->cur_width);
	if (ret < 0)
		 goto p_err;
	ret = fimc_is_sensor_write16(client, 0x0A14, cis_data->cur_height);
	if (ret < 0)
		 goto p_err;

	/* If not use to binning, sensor image should set only crop */
	if (!binning) {
		dbg_sensor(1, "Sensor size set is not binning\n");
		goto p_err;
	}

	cis_data->frame_time = (cis_data->line_readOut_time * cis_data->cur_height / 1000);
	cis->cis_data->rolling_shutter_skew = (cis->cis_data->cur_height - 1) * cis->cis_data->line_readOut_time;
	dbg_sensor(1, "[%s] frame_time(%d), rolling_shutter_skew(%lld)\n", __func__, cis->cis_data->frame_time, cis->cis_data->rolling_shutter_skew);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec) * 1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_sr846_cis_stream_on(struct v4l2_subdev *subdev)
{
	int ret = 0;
	int fsync_mode = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	/* Sensor fsync on/off */
	fsync_mode = fimc_is_vender_fsync_mode_on(cis_data);
	switch (fsync_mode) {
	case AA_CAMERATYPE_SINGLE:
		info("[%s]normal mode\n", __func__);
		ret = sensor_cis_set_registers(subdev, sensor_sr846_fsync_normal, sensor_sr846_fsync_normal_size);
		if (ret < 0)
			err("[%s] sensor_sr846_fsync_normal fail\n", __func__);
		break;
	case AA_CAMERATYPE_WIDE:
		info("[%s]master mode\n", __func__);
		ret = sensor_cis_set_registers(subdev, sensor_sr846_fsync_master, sensor_sr846_fsync_master_size);
		if (ret < 0)
			err("[%s] sensor_sr846_fsync_master fail\n", __func__);
		break;
	case AA_CAMERATYPE_TELE:
		info("[%s]slave mode\n", __func__);
		ret = sensor_cis_set_registers(subdev, sensor_sr846_fsync_slave, sensor_sr846_fsync_slave_size);
		if (ret < 0)
			err("[%s] sensor_sr846_fsync_slave fail\n", __func__);
		break;
	default:
		warn("%s incorrect fsync mode(%d) use normal mode\n", __func__,  fsync_mode);
		ret = sensor_cis_set_registers(subdev, sensor_sr846_fsync_normal, sensor_sr846_fsync_normal_size);
		if (ret < 0)
			err("[%s] sensor_sr846_fsync_normal fail\n", __func__);
		break;
	}

	ret = sensor_sr846_cis_group_param_hold_func(subdev, 0x00);
	if (ret < 0)
		err("[%s] sensor_sr846_cis_group_param_hold_func fail\n", __func__);

#ifdef DEBUG_SR846_PLL
	{
	u8 pll_8;
	u16 pll;
	dbg_sensor(1, "______ vt_pix_clk_div(%x)\n", 0x03);
	ret = fimc_is_sensor_read8(client, 0x0F34, &pll_8);
	dbg_sensor(1, "______ vt_sys_clk_div1(%x)vt_sys_clk_div2(%x)\n", (pll_8 & 0x07), (pll_8 & 0x18));
	ret = fimc_is_sensor_read8(client, 0x0F32, &pll_8);
	dbg_sensor(1, "______ pre_pll_clk_div(%x)\n", pll_8);
	ret = fimc_is_sensor_read8(client, 0x0F33, &pll_8);
	dbg_sensor(1, "______ pll_multiplier(%x)\n", pll_8);

	ret = fimc_is_sensor_read16(client, 0x0006, &pll);
	dbg_sensor(1, "______ frame_length_lines(%x)\n", pll);
	ret = fimc_is_sensor_read16(client, 0x0008, &pll);
	dbg_sensor(1, "______ line_length_pck(%x)\n", pll);
	}
#endif

	/* Sensor stream on */
	ret = fimc_is_sensor_write16(client, 0x0A20, 0x0100);
	if (ret < 0)
		err("i2c transfer fail addr(%x), val(%x), ret = %d\n", 0x0A20, 0x0100, ret);

	usleep_range(1000, 1000);

	cis_data->stream_on = true;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_sr846_cis_stream_off(struct v4l2_subdev *subdev)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s\n", cis->id, __func__);

	ret = sensor_sr846_cis_group_param_hold_func(subdev, 0x00);
	if (ret < 0)
		err("[%s] sensor_sr846_cis_group_param_hold_func fail\n", __func__);

	/* Sensor stream off */
	ret = fimc_is_sensor_write16(client, 0x0F10, 0x0A00);
	if (ret < 0)
		err("i2c transfer fail addr(%x), val(%x), ret = %d\n", 0x0F10, 0x00, ret);

	ret = fimc_is_sensor_write16(client, 0x0A20, 0x0000);
	if (ret < 0)
		err("i2c transfer fail addr(%x), val(%x), ret = %d\n", 0x0A20, 0x00, ret);

	cis_data->stream_on = false;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_sr846_cis_set_exposure_time(struct v4l2_subdev *subdev, struct ae_param *target_exposure)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u32 vt_pic_clk_freq_mhz = 0;
	u32 long_coarse_int = 0;
	u32 short_coarse_int = 0;
	u32 line_length_pck = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!target_exposure);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	if ((target_exposure->long_val <= 0) || (target_exposure->short_val <= 0)) {
		err("[%s] invalid target exposure(%d, %d)\n", __func__,
				target_exposure->long_val, target_exposure->short_val);
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), target long(%d), short(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, target_exposure->long_val, target_exposure->short_val);

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	line_length_pck = cis_data->line_length_pck;

	long_coarse_int = (target_exposure->long_val * vt_pic_clk_freq_mhz) / line_length_pck;
	short_coarse_int = (target_exposure->short_val * vt_pic_clk_freq_mhz) / line_length_pck;

	if (long_coarse_int > cis_data->max_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), long coarse(%d) max(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, long_coarse_int, cis_data->max_coarse_integration_time);
		long_coarse_int = cis_data->max_coarse_integration_time;
	}

	if (short_coarse_int > cis_data->max_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), short coarse(%d) max(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, short_coarse_int, cis_data->max_coarse_integration_time);
		short_coarse_int = cis_data->max_coarse_integration_time;
	}

	if (long_coarse_int < cis_data->min_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), long coarse(%d) min(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, long_coarse_int, cis_data->min_coarse_integration_time);
		long_coarse_int = cis_data->min_coarse_integration_time;
	}

	if (short_coarse_int < cis_data->min_coarse_integration_time) {
		dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), short coarse(%d) min(%d)\n", cis->id, __func__,
			cis_data->sen_vsync_count, short_coarse_int, cis_data->min_coarse_integration_time);
		short_coarse_int = cis_data->min_coarse_integration_time;
	}

	hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	/* integration time */
	ret = fimc_is_sensor_write16(client, 0x0074, (u16)(short_coarse_int & 0xFFFF));
	if (ret < 0)
		goto p_err;

	dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), vt_pic_clk_freq_mhz (%d), line_length_pck(%d)\n", cis->id, __func__,
		cis_data->sen_vsync_count, vt_pic_clk_freq_mhz, line_length_pck);
	dbg_sensor(1, "[MOD:D:%d] %s, vsync_cnt(%d), frame_length_lines(%#x), long_coarse_int %#x, short_coarse_int %#x\n", cis->id, __func__,
		cis_data->sen_vsync_count, cis_data->frame_length_lines, long_coarse_int, short_coarse_int);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_sr846_cis_get_min_exposure_time(struct v4l2_subdev *subdev, u32 *min_expo)
{
	int ret = 0;
	struct fimc_is_cis *cis = NULL;
	cis_shared_data *cis_data = NULL;
	u32 min_integration_time = 0;
	u32 min_coarse = 0;
	u32 vt_pic_clk_freq_mhz = 0;
	u32 line_length_pck = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!min_expo);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	cis_data = cis->cis_data;

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	if (vt_pic_clk_freq_mhz == 0) {
		pr_err("[MOD:D:%d] %s, Invalid vt_pic_clk_freq_mhz(%d)\n", cis->id, __func__, vt_pic_clk_freq_mhz);
		goto p_err;
	}
	line_length_pck = cis_data->line_length_pck;
	min_coarse = cis_data->min_coarse_integration_time;

	min_integration_time = line_length_pck * min_coarse / vt_pic_clk_freq_mhz;
	*min_expo = min_integration_time;

	dbg_sensor(1, "[%s] min integration time %d\n", __func__, min_integration_time);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_sr846_cis_get_max_exposure_time(struct v4l2_subdev *subdev, u32 *max_expo)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;
	u32 max_integration_time = 0;
	u32 max_coarse_margin = 0;
	u32 max_fine_margin = 0;
	u32 max_coarse = 0;
	u32 vt_pic_clk_freq_mhz = 0;
	u32 line_length_pck = 0;
	u32 frame_length_lines = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!max_expo);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	cis_data = cis->cis_data;

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	if (vt_pic_clk_freq_mhz == 0) {
		pr_err("[MOD:D:%d] %s, Invalid vt_pic_clk_freq_mhz(%d)\n", cis->id, __func__, vt_pic_clk_freq_mhz);
		goto p_err;
	}
	line_length_pck = cis_data->line_length_pck;
	frame_length_lines = cis_data->frame_length_lines;

	max_coarse_margin = cis_data->max_margin_coarse_integration_time;
	max_fine_margin = line_length_pck - cis_data->min_fine_integration_time;
	max_coarse = frame_length_lines - max_coarse_margin;

	max_integration_time = line_length_pck * max_coarse / vt_pic_clk_freq_mhz;

	*max_expo = max_integration_time;

	/* TODO: Is this values update hear? */
	cis_data->max_margin_fine_integration_time = max_fine_margin;
	cis_data->max_coarse_integration_time = max_coarse;

	dbg_sensor(1, "[%s] max integration time %d, max margin fine integration %d, max coarse integration %d\n",
			__func__, max_integration_time, cis_data->max_margin_fine_integration_time, cis_data->max_coarse_integration_time);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_sr846_cis_adjust_frame_duration(struct v4l2_subdev *subdev,
						u32 input_exposure_time,
						u32 *target_duration)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;

	u32 vt_pic_clk_freq_mhz = 0;
	u32 line_length_pck = 0;
	u32 frame_length_lines = 0;
	u32 frame_duration = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!target_duration);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	cis_data = cis->cis_data;

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	line_length_pck = cis_data->line_length_pck;
	frame_length_lines = ((vt_pic_clk_freq_mhz * input_exposure_time) / line_length_pck);
	frame_length_lines += cis_data->max_margin_coarse_integration_time;

	frame_duration = (frame_length_lines * line_length_pck) / vt_pic_clk_freq_mhz;

	dbg_sensor(1, "[%s](vsync cnt = %d) input exp(%d), adj duration, frame duraion(%d), min_frame_us(%d)\n",
			__func__, cis_data->sen_vsync_count, input_exposure_time, frame_duration, cis_data->min_frame_us_time);
	dbg_sensor(1, "[%s](vsync cnt = %d) adj duration, frame duraion(%d), min_frame_us(%d)\n",
			__func__, cis_data->sen_vsync_count, frame_duration, cis_data->min_frame_us_time);

	*target_duration = MAX(frame_duration, cis_data->min_frame_us_time);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_sr846_cis_set_frame_duration(struct v4l2_subdev *subdev, u32 frame_duration)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u32 vt_pic_clk_freq_mhz = 0;
	u32 line_length_pck = 0;
	u16 frame_length_lines = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	if (frame_duration < cis_data->min_frame_us_time) {
		dbg_sensor(1, "frame duration is less than min(%d)\n", frame_duration);
		frame_duration = cis_data->min_frame_us_time;
	}

	vt_pic_clk_freq_mhz = cis_data->pclk / (1000 * 1000);
	line_length_pck = cis_data->line_length_pck;

	frame_length_lines = (u16)((vt_pic_clk_freq_mhz * frame_duration) / line_length_pck);

	dbg_sensor(1, "[MOD:D:%d] %s, vt_pic_clk_freq_mhz(%#x) frame_duration = %d us, (line_length_pck%#x), frame_length_lines(%#x)\n",
			cis->id, __func__, vt_pic_clk_freq_mhz, frame_duration, line_length_pck, frame_length_lines);

	hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_write16(client, 0x0006, frame_length_lines);
	if (ret < 0)
		goto p_err;

	cis_data->cur_frame_us_time = frame_duration;
	cis_data->frame_length_lines = frame_length_lines;
	cis_data->max_coarse_integration_time = cis_data->frame_length_lines - cis_data->max_margin_coarse_integration_time;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_sr846_cis_set_frame_rate(struct v4l2_subdev *subdev, u32 min_fps)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;

	u32 frame_duration = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	cis_data = cis->cis_data;

	if (min_fps > cis_data->max_fps) {
		err("[MOD:D:%d] %s, request FPS is too high(%d), set to max(%d)\n",
			cis->id, __func__, min_fps, cis_data->max_fps);
		min_fps = cis_data->max_fps;
	}

	if (min_fps == 0) {
		err("[MOD:D:%d] %s, request FPS is 0, set to min FPS(1)\n",
			cis->id, __func__);
		min_fps = 1;
	}

	frame_duration = (1 * 1000 * 1000) / min_fps;

	dbg_sensor(1, "[MOD:D:%d] %s, set FPS(%d), frame duration(%d)\n",
			cis->id, __func__, min_fps, frame_duration);

	ret = sensor_sr846_cis_set_frame_duration(subdev, frame_duration);
	if (ret < 0) {
		err("[MOD:D:%d] %s, set frame duration is fail(%d)\n",
			cis->id, __func__, ret);
		goto p_err;
	}

	cis_data->min_frame_us_time = frame_duration;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:

	return ret;
}

/* AG = ( (1/16) * code ) + 1 */
u32 sensor_sr846_cis_calc_again_code(u32 permile)
{
	return ((permile - 1000) * 16 / 1000);
}

u32 sensor_sr846_cis_calc_again_permile(u32 code)
{
	return ((code * 1000 / 16 + 1000));
}

u32 sensor_sr846_cis_calc_dgain_code(u32 permile)
{
	u16 buf[2] = {0, 0};
	buf[0] = (u16)((int)permile / 1000);
	buf[1] = (u16)((int)(((permile - (u32)(buf[0] * 1000)) * 512) / 1000));

	return (buf[0] << 9 | buf[1]);
}

u32 sensor_sr846_cis_calc_dgain_permile(u32 code)
{
	return (((code & 0x1E00) >> 9) * 1000 + ((int)((code & 0x01FF) * 1000 / 512)));
}

int sensor_sr846_cis_adjust_analog_gain(struct v4l2_subdev *subdev, u32 input_again, u32 *target_permile)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;

	u32 again_code = 0;
	u32 again_permile = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!target_permile);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	cis_data = cis->cis_data;

	again_code = sensor_sr846_cis_calc_again_code(input_again);

	if (again_code > cis_data->max_analog_gain[0]) {
		again_code = cis_data->max_analog_gain[0];
	} else if (again_code < cis_data->min_analog_gain[0]) {
		again_code = cis_data->min_analog_gain[0];
	}

	again_permile = sensor_sr846_cis_calc_again_permile(again_code);

	dbg_sensor(1, "[%s] min again(%d), max(%d), input_again(%d), code(%d), permile(%d)\n", __func__,
			cis_data->max_analog_gain[0],
			cis_data->min_analog_gain[0],
			input_again,
			again_code,
			again_permile);

	*target_permile = again_permile;

	return ret;
}

int sensor_sr846_cis_set_analog_gain(struct v4l2_subdev *subdev, struct ae_param *again)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;

	u16 analog_gain = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	analog_gain = (u16)sensor_sr846_cis_calc_again_code(again->val);

	if (analog_gain < cis->cis_data->min_analog_gain[0]) {
		analog_gain = cis->cis_data->min_analog_gain[0];
	}

	if (analog_gain > cis->cis_data->max_analog_gain[0]) {
		analog_gain = cis->cis_data->max_analog_gain[0];
	}

	dbg_sensor(1, "[MOD:D:%d] %s(vsync cnt = %d), input_again = %d us, analog_gain(%#x)\n",
		cis->id, __func__, cis->cis_data->sen_vsync_count, again->val, analog_gain);

	hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_write8(client, 0x0077, analog_gain);
	if (ret < 0)
		goto p_err;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_sr846_cis_get_analog_gain(struct v4l2_subdev *subdev, u32 *again)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;

	u8 analog_gain = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_read8(client, 0x0077, &analog_gain);
	if (ret < 0)
		goto p_err;

	*again = sensor_sr846_cis_calc_again_permile(analog_gain);

	dbg_sensor(1, "[MOD:D:%d] %s, cur_again = %d us, analog_gain(%#x)\n",
			cis->id, __func__, *again, analog_gain);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_sr846_cis_get_min_analog_gain(struct v4l2_subdev *subdev, u32 *min_again)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 read_value = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!min_again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	read_value = 0x0; /* x1, AG =(1/16*value) + 1 */

	cis_data->min_analog_gain[0] = read_value;

	cis_data->min_analog_gain[1] = sensor_sr846_cis_calc_again_permile(read_value);

	*min_again = cis_data->min_analog_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__, cis_data->min_analog_gain[0], cis_data->min_analog_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_sr846_cis_get_max_analog_gain(struct v4l2_subdev *subdev, u32 *max_again)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 read_value = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!max_again);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	read_value = 0xf0; /* x16, AG =(1/16*value) + 1 */

	cis_data->max_analog_gain[0] = read_value;

	cis_data->max_analog_gain[1] = sensor_sr846_cis_calc_again_permile(read_value);

	*max_again = cis_data->max_analog_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__, cis_data->max_analog_gain[0], cis_data->max_analog_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	return ret;
}

int sensor_sr846_cis_set_digital_gain(struct v4l2_subdev *subdev, struct ae_param *dgain)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	u16 long_gain = 0;
	u16 short_gain = 0;
	u16 dgains[4] = {0};

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!dgain);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	cis_data = cis->cis_data;

	long_gain = (u16)sensor_sr846_cis_calc_dgain_code(dgain->long_val);
	short_gain = (u16)sensor_sr846_cis_calc_dgain_code(dgain->short_val);

	if (long_gain < cis->cis_data->min_digital_gain[0]) {
		long_gain = cis->cis_data->min_digital_gain[0];
	}
	if (long_gain > cis->cis_data->max_digital_gain[0]) {
		long_gain = cis->cis_data->max_digital_gain[0];
	}

	if (short_gain < cis->cis_data->min_digital_gain[0]) {
		short_gain = cis->cis_data->min_digital_gain[0];
	}
	if (short_gain > cis->cis_data->max_digital_gain[0]) {
		short_gain = cis->cis_data->max_digital_gain[0];
	}

	dbg_sensor(1, "[MOD:D:%d] %s(vsync cnt = %d), input_dgain = %d/%d us, long_gain(%#x), short_gain(%#x)\n",
			cis->id, __func__, cis->cis_data->sen_vsync_count, dgain->long_val, dgain->short_val, long_gain, short_gain);

	hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	dgains[0] = dgains[1] = dgains[2] = dgains[3] = short_gain;
	/* Short digital gain */
	ret = fimc_is_sensor_write16_array(client, 0x0078, dgains, 4);
	if (ret < 0)
		goto p_err;

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_sr846_cis_get_digital_gain(struct v4l2_subdev *subdev, u32 *dgain)
{
	int ret = 0;
	int hold = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;

	u16 digital_gain = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!dgain);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);

	client = cis->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x01);
	if (hold < 0) {
		ret = hold;
		goto p_err;
	}

	ret = fimc_is_sensor_read16(client, 0x0078, &digital_gain);
	if (ret < 0)
		goto p_err;

	*dgain = sensor_sr846_cis_calc_dgain_permile(digital_gain);

	dbg_sensor(1, "[MOD:D:%d] %s, cur_dgain = %d us, digital_gain(%#x)\n",
			cis->id, __func__, *dgain, digital_gain);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

p_err:
	if (hold > 0) {
		hold = sensor_sr846_cis_group_param_hold_func(subdev, 0x00);
		if (hold < 0)
			ret = hold;
	}

	return ret;
}

int sensor_sr846_cis_get_min_digital_gain(struct v4l2_subdev *subdev, u32 *min_dgain)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;

	u16 read_value = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!min_dgain);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	cis_data = cis->cis_data;

	read_value = 0x0001;

	cis_data->min_digital_gain[0] = read_value;

	cis_data->min_digital_gain[1] = sensor_sr846_cis_calc_dgain_permile(read_value);

	*min_dgain = cis_data->min_digital_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__, cis_data->min_digital_gain[0], cis_data->min_digital_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_sr846_cis_get_max_digital_gain(struct v4l2_subdev *subdev, u32 *max_dgain)
{
	int ret = 0;
	struct fimc_is_cis *cis;
	cis_shared_data *cis_data;

	u16 read_value = 0;

#ifdef DEBUG_SENSOR_TIME
	struct timeval st, end;
	do_gettimeofday(&st);
#endif

	BUG_ON(!subdev);
	BUG_ON(!max_dgain);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);

	BUG_ON(!cis);
	BUG_ON(!cis->cis_data);

	cis_data = cis->cis_data;

	/* sr846 cannot read min/max digital gain */
	read_value = 0x1FFF;

	cis_data->max_digital_gain[0] = read_value;

	cis_data->max_digital_gain[1] = sensor_sr846_cis_calc_dgain_permile(read_value);

	*max_dgain = cis_data->max_digital_gain[1];

	dbg_sensor(1, "[%s] code %d, permile %d\n", __func__, cis_data->max_digital_gain[0], cis_data->max_digital_gain[1]);

#ifdef DEBUG_SENSOR_TIME
	do_gettimeofday(&end);
	dbg_sensor(1, "[%s] time %lu us\n", __func__, (end.tv_sec - st.tv_sec)*1000000 + (end.tv_usec - st.tv_usec));
#endif

	return ret;
}

int sensor_sr846_cis_wait_streamoff(struct v4l2_subdev *subdev)
{
	int ret = 0;
	u32 poll_time_ms = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	if (unlikely(!cis)) {
	    err("cis is NULL");
	    ret = -EINVAL;
	    goto p_err;
	}

	cis_data = cis->cis_data;
	if (unlikely(!cis_data)) {
	    err("cis_data is NULL");
	    ret = -EINVAL;
	    goto p_err;
	}

	client = cis->client;
	if (unlikely(!client)) {
	    err("client is NULL");
	    ret = -EINVAL;
	    goto p_err;
	}

	/* Checking stream off */
	do {
		u8 read_value = 0;
		ret = fimc_is_sensor_read8(client, 0x0F02, &read_value);
		if (ret < 0) {
			err("i2c transfer fail addr(%x) ret = %d\n", 0x0F02, ret);
			break;
		}
		read_value = (read_value & 0x01); // 0x0F02 B[0]
		if ((read_value != 0x01)) {
			break;
		}
		msleep(POLL_TIME_MS);
		poll_time_ms += POLL_TIME_MS;
	} while (poll_time_ms < STREAM_OFF_POLL_TIME_MS);

	if (poll_time_ms < STREAM_OFF_POLL_TIME_MS)
		info("%s: finished after %d ms\n", __func__, poll_time_ms);
	else
		warn("%s: finished : polling timeout occured after %d ms\n", __func__, poll_time_ms);

p_err:
	return ret;
}

int sensor_sr846_cis_wait_streamon(struct v4l2_subdev *subdev)
{
#if 0
	int ret = 0;
	u32 poll_time_ms = 0;
	struct fimc_is_cis *cis;
	struct i2c_client *client;
	cis_shared_data *cis_data;

	BUG_ON(!subdev);

	cis = (struct fimc_is_cis *)v4l2_get_subdevdata(subdev);
	if (unlikely(!cis)) {
	    err("cis is NULL");
	    ret = -EINVAL;
	    goto p_err;
	}

	cis_data = cis->cis_data;
	if (unlikely(!cis_data)) {
	    err("cis_data is NULL");
	    ret = -EINVAL;
	    goto p_err;
	}

	client = cis->client;
	if (unlikely(!client)) {
	    err("client is NULL");
	    ret = -EINVAL;
	    goto p_err;
	}

	/* Checking stream on */
	do {
		u16 read_value = 0;
		ret = fimc_is_sensor_read16(client, 0x0C2A, &read_value);
		if (ret < 0) {
			err("i2c transfer fail addr(%x) ret = %d\n", 0x0C2A, ret);
			break;
		}
		if ((read_value != 0x00)) {
			break;
		}
		msleep(POLL_TIME_MS);
		poll_time_ms += POLL_TIME_MS;
	} while (poll_time_ms < STREAM_OFF_POLL_TIME_MS);

	if (poll_time_ms < STREAM_OFF_POLL_TIME_MS)
		info("%s: finished after %d ms\n", __func__, poll_time_ms);
	else
		warn("%s: finished : polling timeout occured after %d ms\n", __func__, poll_time_ms);

p_err:
	return ret;
#else
	return 0;
#endif
}

static struct fimc_is_cis_ops cis_ops = {
	.cis_init = sensor_sr846_cis_init,
	.cis_log_status = sensor_sr846_cis_log_status,
	.cis_group_param_hold = sensor_sr846_cis_group_param_hold,
	.cis_set_global_setting = sensor_sr846_cis_set_global_setting,
	.cis_mode_change = sensor_sr846_cis_mode_change,
	.cis_set_size = sensor_sr846_cis_set_size,
	.cis_stream_on = sensor_sr846_cis_stream_on,
	.cis_stream_off = sensor_sr846_cis_stream_off,
	.cis_set_exposure_time = sensor_sr846_cis_set_exposure_time,
	.cis_get_min_exposure_time = sensor_sr846_cis_get_min_exposure_time,
	.cis_get_max_exposure_time = sensor_sr846_cis_get_max_exposure_time,
	.cis_adjust_frame_duration = sensor_sr846_cis_adjust_frame_duration,
	.cis_set_frame_duration = sensor_sr846_cis_set_frame_duration,
	.cis_set_frame_rate = sensor_sr846_cis_set_frame_rate,
	.cis_adjust_analog_gain = sensor_sr846_cis_adjust_analog_gain,
	.cis_set_analog_gain = sensor_sr846_cis_set_analog_gain,
	.cis_get_analog_gain = sensor_sr846_cis_get_analog_gain,
	.cis_get_min_analog_gain = sensor_sr846_cis_get_min_analog_gain,
	.cis_get_max_analog_gain = sensor_sr846_cis_get_max_analog_gain,
	.cis_set_digital_gain = sensor_sr846_cis_set_digital_gain,
	.cis_get_digital_gain = sensor_sr846_cis_get_digital_gain,
	.cis_get_min_digital_gain = sensor_sr846_cis_get_min_digital_gain,
	.cis_get_max_digital_gain = sensor_sr846_cis_get_max_digital_gain,
	.cis_compensate_gain_for_extremely_br = sensor_cis_compensate_gain_for_extremely_br,
	.cis_wait_streamoff = sensor_sr846_cis_wait_streamoff,
	.cis_wait_streamon = sensor_sr846_cis_wait_streamon,
};

int cis_sr846_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;

	struct fimc_is_core *core = NULL;
	struct v4l2_subdev *subdev_cis = NULL;
	struct fimc_is_cis *cis = NULL;
	struct fimc_is_device_sensor *device = NULL;
	struct fimc_is_device_sensor_peri *sensor_peri = NULL;
	u32 sensor_id = 0;
	char const *setfile;
	struct device *dev;
	struct device_node *dnode;

	BUG_ON(!client);
	BUG_ON(!fimc_is_dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		probe_info("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	dev = &client->dev;
	dnode = dev->of_node;

	ret = of_property_read_u32(dnode, "id", &sensor_id);
	if (ret) {
		err("sensor id read is fail(%d)\n", ret);
		goto p_err;
	}

	probe_info("%s sensor id %d\n", __func__, sensor_id);

	device = &core->sensor[sensor_id];

	sensor_peri = find_peri_by_cis_id(device, SENSOR_NAME_SR846);
	if (!sensor_peri) {
		probe_info("sensor peri is not yet probed");
		return -EPROBE_DEFER;
	}

	cis = &sensor_peri->cis;
	if (!cis) {
		err("cis is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	subdev_cis = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_cis) {
		probe_err("subdev_cis is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	sensor_peri->subdev_cis = subdev_cis;

	cis->id = SENSOR_NAME_SR846;
	cis->subdev = subdev_cis;
	cis->device = 0;
	cis->client = client;
	sensor_peri->module->client = cis->client;
	cis->ctrl_delay = N_PLUS_TWO_FRAME;

	cis->cis_data = kzalloc(sizeof(cis_shared_data), GFP_KERNEL);
	if (!cis->cis_data) {
		err("cis_data is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	cis->cis_ops = &cis_ops;

	/* belows are depend on sensor cis. MUST check sensor spec */
	cis->bayer_order = OTF_INPUT_ORDER_BAYER_GB_RG;

	if (of_property_read_bool(dnode, "sensor_f_number")) {
		ret = of_property_read_u32(dnode, "sensor_f_number", &cis->aperture_num);
		if (ret) {
			warn("f-number read is fail(%d)",ret);
		}
	} else {
		cis->aperture_num = F1_9;
	}

	probe_info("%s f-number %d\n", __func__, cis->aperture_num);

	cis->use_dgain = true;
	cis->hdr_ctrl_by_again = false;

	ret = of_property_read_string(dnode, "setfile", &setfile);
	if (ret) {
		err("setfile index read fail(%d), take default setfile!!", ret);
		setfile = "default";
	}

	if (strcmp(setfile, "default") == 0 ||
			strcmp(setfile, "setA") == 0) {
		probe_info("%s : setfile_A\n", __func__);
		sensor_sr846_global = sensor_sr846_setfile_A_Global;
		sensor_sr846_global_size = sizeof(sensor_sr846_setfile_A_Global) / sizeof(sensor_sr846_setfile_A_Global[0]);
		sensor_sr846_setfiles = sensor_sr846_setfiles_A;
		sensor_sr846_setfile_sizes = sensor_sr846_setfile_A_sizes;
		sensor_sr846_max_setfile_num = sizeof(sensor_sr846_setfiles_A) / sizeof(sensor_sr846_setfiles_A[0]);
		sensor_sr846_pllinfos = sensor_sr846_pllinfos_A;
		sensor_sr846_fsync_normal = sensor_sr846_setfile_A_Fsync_Normal;
		sensor_sr846_fsync_normal_size = sizeof(sensor_sr846_setfile_A_Fsync_Normal) / sizeof(sensor_sr846_setfile_A_Fsync_Normal[0]);
		sensor_sr846_fsync_master = sensor_sr846_setfile_A_Fsync_Master;
		sensor_sr846_fsync_master_size = sizeof(sensor_sr846_setfile_A_Fsync_Master) / sizeof(sensor_sr846_setfile_A_Fsync_Master[0]);
		sensor_sr846_fsync_slave = sensor_sr846_setfile_A_Fsync_Slave;
		sensor_sr846_fsync_slave_size = sizeof(sensor_sr846_setfile_A_Fsync_Slave) / sizeof(sensor_sr846_setfile_A_Fsync_Slave[0]);
	} else if (strcmp(setfile, "setB") == 0) {
		probe_info("%s : setfile_B\n", __func__);
		sensor_sr846_global = sensor_sr846_setfile_B_Global;
		sensor_sr846_global_size = sizeof(sensor_sr846_setfile_B_Global) / sizeof(sensor_sr846_setfile_B_Global[0]);
		sensor_sr846_setfiles = sensor_sr846_setfiles_B;
		sensor_sr846_setfile_sizes = sensor_sr846_setfile_B_sizes;
		sensor_sr846_max_setfile_num = sizeof(sensor_sr846_setfiles_B) / sizeof(sensor_sr846_setfiles_B[0]);
		sensor_sr846_pllinfos = sensor_sr846_pllinfos_B;
		sensor_sr846_fsync_normal = sensor_sr846_setfile_B_Fsync_Normal;
		sensor_sr846_fsync_normal_size = sizeof(sensor_sr846_setfile_B_Fsync_Normal) / sizeof(sensor_sr846_setfile_B_Fsync_Normal[0]);
		sensor_sr846_fsync_master = sensor_sr846_setfile_B_Fsync_Master;
		sensor_sr846_fsync_master_size = sizeof(sensor_sr846_setfile_B_Fsync_Master) / sizeof(sensor_sr846_setfile_B_Fsync_Master[0]);
		sensor_sr846_fsync_slave = sensor_sr846_setfile_B_Fsync_Slave;
		sensor_sr846_fsync_slave_size = sizeof(sensor_sr846_setfile_B_Fsync_Slave) / sizeof(sensor_sr846_setfile_B_Fsync_Slave[0]);
	} else {
		err("%s setfile index out of bound, take default (setfile_A)", __func__);
		sensor_sr846_global = sensor_sr846_setfile_A_Global;
		sensor_sr846_global_size = sizeof(sensor_sr846_setfile_A_Global) / sizeof(sensor_sr846_setfile_A_Global[0]);
		sensor_sr846_setfiles = sensor_sr846_setfiles_A;
		sensor_sr846_setfile_sizes = sensor_sr846_setfile_A_sizes;
		sensor_sr846_max_setfile_num = sizeof(sensor_sr846_setfiles_A) / sizeof(sensor_sr846_setfiles_A[0]);
		sensor_sr846_pllinfos = sensor_sr846_pllinfos_A;
		sensor_sr846_fsync_normal = sensor_sr846_setfile_A_Fsync_Normal;
		sensor_sr846_fsync_normal_size = sizeof(sensor_sr846_setfile_A_Fsync_Normal) / sizeof(sensor_sr846_setfile_A_Fsync_Normal[0]);
		sensor_sr846_fsync_master = sensor_sr846_setfile_A_Fsync_Master;
		sensor_sr846_fsync_master_size = sizeof(sensor_sr846_setfile_A_Fsync_Master) / sizeof(sensor_sr846_setfile_A_Fsync_Master[0]);
		sensor_sr846_fsync_slave = sensor_sr846_setfile_A_Fsync_Slave;
		sensor_sr846_fsync_slave_size = sizeof(sensor_sr846_setfile_A_Fsync_Slave) / sizeof(sensor_sr846_setfile_A_Fsync_Slave[0]);
	}

	v4l2_i2c_subdev_init(subdev_cis, client, &subdev_ops);
	v4l2_set_subdevdata(subdev_cis, cis);
	v4l2_set_subdev_hostdata(subdev_cis, device);
	snprintf(subdev_cis->name, V4L2_SUBDEV_NAME_SIZE, "cis-subdev.%d", cis->id);

	probe_info("%s done\n", __func__);

p_err:
	return ret;
}

static int cis_sr846_remove(struct i2c_client *client)
{
	int ret = 0;
	return ret;
}

static const struct of_device_id exynos_fimc_is_cis_sr846_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-cis-sr846",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_cis_sr846_match);

static const struct i2c_device_id cis_sr846_idt[] = {
	{ SENSOR_NAME, 0 },
	{},
};

static struct i2c_driver cis_sr846_driver = {
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = exynos_fimc_is_cis_sr846_match
	},
	.probe	= cis_sr846_probe,
	.remove	= cis_sr846_remove,
	.id_table = cis_sr846_idt
};
module_i2c_driver(cis_sr846_driver);
