/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2018 Samsung Electronics Co., Ltd
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
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include <exynos-fimc-is-sensor.h>
#include "fimc-is-hw.h"
#include "fimc-is-core.h"
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-resourcemgr.h"
#include "fimc-is-dt.h"

#include "fimc-is-device-module-base.h"
#include "fimc-is-device-module-imx576.h"

// Reference Version : 'IMX576-AAKH5_SAM-Set-26MHz-DPHY_RegisterSetting_ver2.00-9.00_MP_180712.xlsx'
// Reference Version : 'IMX576-AAKH5_SAM-Set-26MHz-DPHY_RegisterSetting_ver2.00-8.00_MP0_180712.xlsx'

/*
 * [Mode Information]
 *	- Global Setting -
 *
 *	- 2X2 BINNING -
 *	[0] REG_A: Single Still Preview (4:3)   : 2832x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[1] REG_B: Single Still Preview (16:9)  : 2832x1592@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[2] REG_C: Single Still Preview (18.5:9): 2832x1376@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[3] REG_D: Single Still Preview (1:1)   : 2124x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- QBC_HDR -
 *	[4] REG_E: Single Still 3HDR (4:3)      : 2832x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[5] REG_F: Single Still 3HDR (16:9)     : 2832x1592@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[6] REG_G: Single Still 3HDR (18.5:9)   : 2832x1376@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[7] REG_H: Single Still 3HDR (1:1)      : 2124x2124@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- QBC_REMOSAIC -
 *	[8] REG_I: Single Still Capture (4:3)   : 5664X4248@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[9] REG_J: Single Still Capture (16:9)  : 5664X3184@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[10]REG_K: Single Still Capture (18.5:9): 5664X2752@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[11]REG_L: Single Still Capture (1:1)   : 4248X4248@30,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *
 *	- Super Slow Motion (SSM) -
 *	[12]REG_M: Super Slow Motion (16:9)     : 1872x1052@240,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[13]REG_N: Super Slow Motion (16:9)     : 1920x1080@120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[14]REG_O: Super Slow Motion (16:9)     : 1280x720 @240,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[15]REG_U: Super Slow Motion (16:9)     : 1280x720 @120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054 
 *
 *	- FAST AE -
 *	[16]REG_R: Single Preview Fast(4:3)     : 2832x2124@117,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[17]REG_S: Single Preview Fast(4:3)     : 2832x2124@ 60,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 *	[18]REG_T: Single Preview Fast(16:9)    : 2832x1592@120,  MIPI lane: 4, MIPI data rate(Mbps/lane) Sensor: 2054
 */

static struct fimc_is_sensor_cfg config_imx576[] = {
	/* 0 : REG_A: 2832x2124@30fps 2X2BIN */
	/* 1 : REG_B: 2832x1592@30fps 2X2BIN */
	/* 2 : REG_C: 2832x1376@30fps 2X2BIN */
	/* 3 : REG_D: 2124x2124@30fps 2X2BIN */
	FIMC_IS_SENSOR_CFG_EXT(2832, 2124,  30, 46,  0, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(2832, 1592,  30, 46,  1, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(2832, 1376,  30, 46,  2, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(2124, 2124,  30, 46,  3, CSI_DATA_LANES_4, 2054, 0, 0, 0),

	/* 4 : REG_E: 2832x2124@30fps QBC_HDR */
	/* 5 : REG_F: 2832x1592@30fps QBC_HDR */
	/* 6 : REG_G: 2832x1376@30fps QBC_HDR */
	/* 7 : REG_H: 2124x2124@30fps QBC_HDR */
	FIMC_IS_SENSOR_CFG_EXT(2832, 2124,  30, 46,  4, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(2832, 1592,  30, 46,  5, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(2832, 1376,  30, 46,  6, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(2124, 2124,  30, 46,  7, CSI_DATA_LANES_4, 2054, 0, 0, 0),

	/* 8 : REG_I: 5664X4248@30fps QBC_REMOSAIC */
	/* 9 : REG_J: 5664X3184@30fps QBC_REMOSAIC */
	/* 10: REG_K: 5664X2752@30fps QBC_REMOSAIC */
	/* 11: REG_L: 4248X4248@30fps QBC_REMOSAIC */
	FIMC_IS_SENSOR_CFG_EXT(5664, 4248,  30, 46,  8, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(5664, 3184,  30, 46,  9, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(5664, 2752,  30, 46, 10, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(4248, 4248,  30, 46, 11, CSI_DATA_LANES_4, 2054, 0, 0, 0),

	/* 12: REG_M: 1872x1052@240fps Super Slow Motion (SSM) */
	/* 13: REG_N: 1920x1080@120fps Super Slow Motion (SSM) */
	/* 14: REG_O: 1280x720 @240fps Super Slow Motion (SSM) */
	/* 15: REG_U: 1280x720 @120fps Super Slow Motion (SSM) */
	FIMC_IS_SENSOR_CFG_EXT(1872, 1052, 240, 46, 12, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(1920, 1080, 120, 46, 13, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(1280,  720, 240, 46, 14, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(1280,  720, 120, 46, 15, CSI_DATA_LANES_4, 2054, 0, 0, 0),

	/* 16: REG_R: 2832x2124@117fps FAST AE */
	/* 17: REG_S: 2832x2124@60fps FAST AE */
	/* 18: REG_T: 2832x1592@120 FAST AE */
	FIMC_IS_SENSOR_CFG_EXT(2832, 2124, 117, 46, 16, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(2832, 2124,  60, 46, 17, CSI_DATA_LANES_4, 2054, 0, 0, 0),
	FIMC_IS_SENSOR_CFG_EXT(2832, 1592, 120, 46, 18, CSI_DATA_LANES_4, 2054, 0, 0, 0),
};

static struct fimc_is_vci vci_module_imx576[] = {
	{
		.pixelformat = V4L2_PIX_FMT_SBGGR10,
		.config = {{0, HW_FORMAT_RAW10}, {1, HW_FORMAT_UNKNOWN}, {2, HW_FORMAT_EMBEDDED_8BIT}, {3, 0}}
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR12,
		.config = {{0, HW_FORMAT_RAW10}, {1, HW_FORMAT_UNKNOWN}, {2, HW_FORMAT_EMBEDDED_8BIT}, {3, 0}}
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR16,
		.config = {{0, HW_FORMAT_RAW10}, {1, HW_FORMAT_UNKNOWN}, {2, HW_FORMAT_EMBEDDED_8BIT}, {3, 0}}
	}
};

static const struct v4l2_subdev_core_ops core_ops = {
	.init = sensor_module_init,
	.g_ctrl = sensor_module_g_ctrl,
	.s_ctrl = sensor_module_s_ctrl,
	.g_ext_ctrls = sensor_module_g_ext_ctrls,
	.s_ext_ctrls = sensor_module_s_ext_ctrls,
	.ioctl = sensor_module_ioctl,
	.log_status = sensor_module_log_status,
};

static const struct v4l2_subdev_video_ops video_ops = {
	.s_stream = sensor_module_s_stream,
	.s_parm = sensor_module_s_param
};

static const struct v4l2_subdev_pad_ops pad_ops = {
	.set_fmt = sensor_module_s_format
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
	.video = &video_ops,
	.pad = &pad_ops
};

#ifdef CONFIG_OF
static int module_imx576_power_setpin(struct device *dev,
	struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	struct device_node *af_np;
	u8 actuator = 0;
	int gpio_reset = 0;
	int gpio_mclk = 0;
	int gpio_none = 0;
	int gpio_cam_dcdc_en = 0;
	int gpio_cam_core_en = 0;
	int gpio_camio_1p8_en = 0;
	int gpio_camaf_2p8_en = 0;
	struct fimc_is_core *core;
	bool shared_mclk = false;
	bool shared_camio_1p8 = false;

	BUG_ON(!dev);

	dnode = dev->of_node;

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core is NULL");
		return -EINVAL;
	}

	dev_info(dev, "%s E v4\n", __func__);

	af_np = of_find_node_by_name(dnode, "af");
	if (!af_np)
		actuator = SENSOR_MODULE_IMX576_WITHOUT_ACTUATOR;
	else
		actuator = SENSOR_MODULE_IMX576_WITH_ACTUATOR;

	dev_info(dev, "actuator value is %d\n", actuator);

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(dev, "failed to get PIN_RESET\n");
		return -EINVAL;
	} else {
		gpio_request_one(gpio_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_reset);
	}

	gpio_mclk = of_get_named_gpio(dnode, "gpio_mclk", 0);
	if (!gpio_is_valid(gpio_mclk)) {
		dev_err(dev, "failed to get gpio_mclk\n");
		return -EINVAL;
	} else {
		gpio_request_one(gpio_mclk, GPIOF_OUT_INIT_LOW, "CAM_MCLK_OUTPUT_LOW");
		gpio_free(gpio_mclk);
	}

	gpio_cam_dcdc_en = of_get_named_gpio(dnode, "gpio_cam_dcdc_en", 0);
	if (!gpio_is_valid(gpio_cam_dcdc_en)) {
		dev_err(dev, "failed to get gpio_cam_dcdc_en\n");
	} else {
		gpio_request_one(gpio_cam_dcdc_en, GPIOF_OUT_INIT_LOW, "CAM_DCDC_1P8_EN");
		gpio_free(gpio_cam_dcdc_en);
	}

	gpio_cam_core_en = of_get_named_gpio(dnode, "gpio_cam_core_en", 0);
	if (!gpio_is_valid(gpio_cam_core_en)) {
		dev_err(dev, "failed to get gpio_cam_core_en\n");
		return -EINVAL;
	} else {
		gpio_request_one(gpio_cam_core_en, GPIOF_OUT_INIT_LOW, "CAM_CORE_AVDD_EN");
		gpio_free(gpio_cam_core_en);
	}

	gpio_camio_1p8_en = of_get_named_gpio(dnode, "gpio_camio_1p8_en", 0);
	if (!gpio_is_valid(gpio_camio_1p8_en)) {
		dev_err(dev, "failed to get gpio_camio_1p8_en\n");
		return -EINVAL;
	} else {
		gpio_request_one(gpio_camio_1p8_en, GPIOF_OUT_INIT_LOW, "CAM_VDDIO_EN");
		gpio_free(gpio_camio_1p8_en);
	}

	if(actuator) {
		gpio_camaf_2p8_en = of_get_named_gpio(dnode, "gpio_camaf_2p8_en", 0);
		if (!gpio_is_valid(gpio_camaf_2p8_en)) {
			dev_err(dev, "failed to get gpio_camaf_2p8_en\n");
			return -EINVAL;
		} else {
			gpio_request_one(gpio_camaf_2p8_en, GPIOF_OUT_INIT_LOW, "CAM_AFVDD_EN");
			gpio_free(gpio_camaf_2p8_en);
		}
	}

	shared_mclk = of_property_read_bool(dnode, "shared_mclk");
	shared_camio_1p8 = of_property_read_bool(dnode, "shared_camio_1p8");

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF);

	/* Normal on */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_dcdc_en, "cam_dcdc_en", PIN_OUTPUT, 1, 1000);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 1, 1000);
	if(shared_camio_1p8)
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
				&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);

	if(actuator)
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_camaf_2p8_en, "camaf_2p8_en", PIN_OUTPUT, 1, 500);
	
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 3000);
	if(shared_mclk) {
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
				&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 1);
	}
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 10000);

	/* Normal off */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "delay", PIN_NONE, 0, 5000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	if(shared_mclk) {
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
				&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 0);
	}
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);

	if(actuator)
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_camaf_2p8_en, "camaf_2p8_en", PIN_OUTPUT, 0, 1000);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 0, 0);
	if(shared_camio_1p8)
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
				&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 0, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_dcdc_en, "cam_dcdc_en", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "delay", PIN_NONE, 0, 10000);

	/* READ_ROM - POWER ON */
	if(actuator)
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_camaf_2p8_en, "camaf_2p8_en", PIN_OUTPUT, 1, 2000);
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 1, 5000);
	
	/* READ_ROM - POWER OFF */
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 0, 1000);
	if(actuator)	
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_camaf_2p8_en, "camaf_2p8_en", PIN_OUTPUT, 0, 5000);

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}
#endif /* CONFIG_OF */

int sensor_module_imx576_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct fimc_is_core *core;
	struct v4l2_subdev *subdev_module;
	struct fimc_is_module_enum *module;
	struct fimc_is_device_sensor *device;
	struct sensor_open_extended *ext;
	struct exynos_platform_fimc_is_module *pdata;
	struct device *dev;
	struct pinctrl_state *s;

	BUG_ON(!fimc_is_dev);

	probe_info("%s start\n", __func__);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		probe_err("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	dev = &pdev->dev;

	fimc_is_module_parse_dt(dev, module_imx576_power_setpin);

	pdata = dev_get_platdata(dev);
	device = &core->sensor[pdata->id];

	subdev_module = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_module) {
		probe_err("subdev_module is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	probe_info("%s pdata->id(%d), module_enum id(%d), position(%d) \n", 
		__func__, pdata->id, atomic_read(&device->module_count), pdata->position);
	module = &device->module_enum[atomic_read(&device->module_count)];
	atomic_inc(&device->module_count);
	clear_bit(FIMC_IS_MODULE_GPIO_ON, &module->state);
	module->pdata = pdata;
	module->dev = dev;
	module->sensor_id = SENSOR_NAME_IMX576;
	module->subdev = subdev_module;
	module->device = pdata->id;
	module->client = NULL;
	module->active_width = 5664;
	module->active_height = 4248;
	module->margin_left = 0;
	module->margin_right = 0;
	module->margin_top = 0;
	module->margin_bottom = 0;
	module->pixel_width = module->active_width + 0;
	module->pixel_height = module->active_height + 0;
	module->max_framerate = 300;
	module->position = pdata->position;
	module->mode = CSI_MODE_DT_ONLY;
	module->lanes = CSI_DATA_LANES_4;
	module->bitwidth = 10;
	module->vcis = ARRAY_SIZE(vci_module_imx576);
	module->vci = vci_module_imx576;
	module->sensor_maker = "SONY";
	module->sensor_name = "IMX576";
	if(pdata->id == 1)
		module->setfile_name = "setfile_imx576_front.bin";
	else
		module->setfile_name = "setfile_imx576.bin";
	module->cfgs = ARRAY_SIZE(config_imx576);
	module->cfg = config_imx576;
	module->ops = NULL;
	
    /* Sensor peri */
	module->private_data = kzalloc(sizeof(struct fimc_is_device_sensor_peri), GFP_KERNEL);
	if (!module->private_data) {
		probe_err("fimc_is_device_sensor_peri is NULL");
		ret = -ENOMEM;
		goto p_err;
	}
	fimc_is_sensor_peri_probe((struct fimc_is_device_sensor_peri*)module->private_data);
	PERI_SET_MODULE(module);

	ext = &module->ext;
	ext->mipi_lane_num = module->lanes;
	ext->I2CSclk = 0;
#ifdef CONFIG_SENSOR_RETENTION_USE
	ext->use_retention_mode = SENSOR_RETENTION_DISABLE;
#endif

	ext->sensor_con.product_name = module->sensor_id /*SENSOR_NAME_IMX576*/;
	ext->sensor_con.peri_type = SE_I2C;
	ext->sensor_con.peri_setting.i2c.channel = pdata->sensor_i2c_ch;
	ext->sensor_con.peri_setting.i2c.slave_address = pdata->sensor_i2c_addr;
	ext->sensor_con.peri_setting.i2c.speed = 1000000;

	ext->actuator_con.product_name = ACTUATOR_NAME_NOTHING;
	ext->flash_con.product_name = FLADRV_NAME_NOTHING;
	ext->from_con.product_name = FROMDRV_NAME_NOTHING;
	ext->preprocessor_con.product_name = PREPROCESSOR_NAME_NOTHING;
	ext->ois_con.product_name = OIS_NAME_NOTHING;

	if (pdata->af_product_name !=  ACTUATOR_NAME_NOTHING) {
		ext->actuator_con.product_name = pdata->af_product_name;
		ext->actuator_con.peri_type = SE_I2C;
		ext->actuator_con.peri_setting.i2c.channel = pdata->af_i2c_ch;
		ext->actuator_con.peri_setting.i2c.slave_address = pdata->af_i2c_addr;
		ext->actuator_con.peri_setting.i2c.speed = 400000;
	}

	if (pdata->flash_product_name != FLADRV_NAME_NOTHING) {
		ext->flash_con.product_name = pdata->flash_product_name;
		ext->flash_con.peri_type = SE_GPIO;
		ext->flash_con.peri_setting.gpio.first_gpio_port_no = pdata->flash_first_gpio;
		ext->flash_con.peri_setting.gpio.second_gpio_port_no = pdata->flash_second_gpio;
	}

	ext->from_con.product_name = FROMDRV_NAME_NOTHING;

	if (pdata->preprocessor_product_name != PREPROCESSOR_NAME_NOTHING) {
		ext->preprocessor_con.product_name = pdata->preprocessor_product_name;
		ext->preprocessor_con.peri_info0.valid = true;
		ext->preprocessor_con.peri_info0.peri_type = SE_SPI;
		ext->preprocessor_con.peri_info0.peri_setting.spi.channel = pdata->preprocessor_spi_channel;
		ext->preprocessor_con.peri_info1.valid = true;
		ext->preprocessor_con.peri_info1.peri_type = SE_I2C;
		ext->preprocessor_con.peri_info1.peri_setting.i2c.channel = pdata->preprocessor_i2c_ch;
		ext->preprocessor_con.peri_info1.peri_setting.i2c.slave_address = pdata->preprocessor_i2c_addr;
		ext->preprocessor_con.peri_info1.peri_setting.i2c.speed = 400000;
		ext->preprocessor_con.peri_info2.valid = true;
		ext->preprocessor_con.peri_info2.peri_type = SE_DMA;
		if (pdata->preprocessor_dma_channel == DMA_CH_NOT_DEFINED)
			ext->preprocessor_con.peri_info2.peri_setting.dma.channel = FLITE_ID_D;
		else
			ext->preprocessor_con.peri_info2.peri_setting.dma.channel = pdata->preprocessor_dma_channel;
	}

	if (pdata->ois_product_name != OIS_NAME_NOTHING) {
		ext->ois_con.product_name = pdata->ois_product_name;
		ext->ois_con.peri_type = SE_I2C;
		ext->ois_con.peri_setting.i2c.channel = pdata->ois_i2c_ch;
		ext->ois_con.peri_setting.i2c.slave_address = pdata->ois_i2c_addr;
		ext->ois_con.peri_setting.i2c.speed = 400000;
	} else {
		ext->ois_con.product_name = pdata->ois_product_name;
		ext->ois_con.peri_type = SE_NULL;
	}

	v4l2_subdev_init(subdev_module, &subdev_ops);

	v4l2_set_subdevdata(subdev_module, module);
	v4l2_set_subdev_hostdata(subdev_module, device);
	snprintf(subdev_module->name, V4L2_SUBDEV_NAME_SIZE, "sensor-subdev.%d", module->sensor_id);

	s = pinctrl_lookup_state(pdata->pinctrl, "release");

	if (pinctrl_select_state(pdata->pinctrl, s) < 0) {
		probe_err("pinctrl_select_state is fail\n");
		goto p_err;
	}
p_err:
	probe_info("%s done(%d)\n", __func__, ret);
	return ret;
}

static int sensor_module_imx576_remove(struct platform_device *pdev)
{
	int ret = 0;

	info("%s\n", __func__);

	return ret;
}

static const struct of_device_id exynos_fimc_is_sensor_module_imx576_match[] = {
	{
		.compatible = "samsung,sensor-rear-module-imx576",
	},
	{
		.compatible = "samsung,sensor-front-module-imx576",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_sensor_module_imx576_match);

static struct platform_driver rear_sensor_module_imx576_driver = {
	.probe  = sensor_module_imx576_probe,
	.remove = sensor_module_imx576_remove,
	.driver = {
		.name   = REAR_SENSOR_MODULE_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = exynos_fimc_is_sensor_module_imx576_match,
	}
};

module_platform_driver(rear_sensor_module_imx576_driver);

static struct platform_driver front_sensor_module_imx576_driver = {
	.probe  = sensor_module_imx576_probe,
	.remove = sensor_module_imx576_remove,
	.driver = {
		.name   = FRONT_SENSOR_MODULE_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = exynos_fimc_is_sensor_module_imx576_match,
	}
};

module_platform_driver(front_sensor_module_imx576_driver);

