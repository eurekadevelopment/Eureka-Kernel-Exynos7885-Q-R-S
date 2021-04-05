/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
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
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-resourcemgr.h"
#include "fimc-is-dt.h"

#include "fimc-is-device-module-base.h"

static struct fimc_is_sensor_cfg config_module_4ha[] = {
	/* 3264x2448@30fps */
	FIMC_IS_SENSOR_CFG(3264, 2448, 30, 20, 0, CSI_DATA_LANES_3),
	/* 3264x1836@30fps */
	FIMC_IS_SENSOR_CFG(3264, 1836, 30, 20, 1, CSI_DATA_LANES_3),
	/* 3264x1588_30fps */
	FIMC_IS_SENSOR_CFG(3264, 1588, 30, 20, 2, CSI_DATA_LANES_3),
	/* 2448x2448_30fps */
	FIMC_IS_SENSOR_CFG(2448, 2448, 30, 20, 3, CSI_DATA_LANES_3),
	/* 1632x1224_30fps */
	FIMC_IS_SENSOR_CFG(1632, 1224, 30, 20, 4, CSI_DATA_LANES_3),
	/* 1632x1224_60fps */
	FIMC_IS_SENSOR_CFG(1632, 1224, 60, 20, 5, CSI_DATA_LANES_3),
	/* 800x600_120fps */
	FIMC_IS_SENSOR_CFG(800, 600, 120, 20, 6, CSI_DATA_LANES_3),
};

static struct fimc_is_sensor_cfg config_module_4ha_4lane[] = {
	/* 3264x2448@30fps */
	FIMC_IS_SENSOR_CFG(3264, 2448, 30, 14, 0, CSI_DATA_LANES_4),
	/* 3264x2330@30fps */
	FIMC_IS_SENSOR_CFG(3264, 2330, 30, 14, 1, CSI_DATA_LANES_4),
	/* 3264x1836@30fps */
	FIMC_IS_SENSOR_CFG(3264, 1836, 30, 14, 2, CSI_DATA_LANES_4),
	/* 3264x1588_30fps */
	FIMC_IS_SENSOR_CFG(3264, 1588, 30, 14, 3, CSI_DATA_LANES_4),
	/* 3264x1504_30fps */
	FIMC_IS_SENSOR_CFG(3264, 1504, 30, 14, 4, CSI_DATA_LANES_4),
	/* 3264x2040@30fps */
	FIMC_IS_SENSOR_CFG(3264, 2040, 30, 14, 5, CSI_DATA_LANES_4),
	/* 3264x1400@30fps */
	FIMC_IS_SENSOR_CFG(3264, 1400, 30, 14, 6, CSI_DATA_LANES_4),
	/* 2608x1956_30fps */
	FIMC_IS_SENSOR_CFG(2608, 1956, 30, 14, 7, CSI_DATA_LANES_4),
	/* 2608x1468_30fps */
	FIMC_IS_SENSOR_CFG(2608, 1468, 30, 14, 8, CSI_DATA_LANES_4),
	/* 2608x1204_30fps */
	FIMC_IS_SENSOR_CFG(2608, 1204, 30, 14, 9, CSI_DATA_LANES_4),
	/* 2448x2448_30fps */
	FIMC_IS_SENSOR_CFG(2448, 2448, 30, 14, 10, CSI_DATA_LANES_4),
	/* 1968x1968_30fps */
	FIMC_IS_SENSOR_CFG(1968, 1968, 30, 14, 11, CSI_DATA_LANES_4),
	/* 1920x1080@30fps */
	FIMC_IS_SENSOR_CFG(1920, 1080, 30, 14, 12, CSI_DATA_LANES_4),
	/* 1632x1224_30fps */
	FIMC_IS_SENSOR_CFG(1632, 1224, 30, 14, 13, CSI_DATA_LANES_4),
	/* 1632x1224_60fps */
	FIMC_IS_SENSOR_CFG(1632, 1224, 60, 14, 14, CSI_DATA_LANES_4),
	/* 1304x978_30fps */
	FIMC_IS_SENSOR_CFG(1304, 978, 30, 14, 15, CSI_DATA_LANES_4),
	/* 984x984_30fps */
	FIMC_IS_SENSOR_CFG(984, 984, 30, 14, 16, CSI_DATA_LANES_4),
	/* 800x600_120fps */
	FIMC_IS_SENSOR_CFG(800, 600, 120, 14, 17, CSI_DATA_LANES_4),
	/* 640x480@120fps */
	FIMC_IS_SENSOR_CFG(640, 480, 120, 14, 18, CSI_DATA_LANES_4),
};


static struct fimc_is_vci vci_module_4ha[] = {
	{
		.pixelformat = V4L2_PIX_FMT_SBGGR10,
		.config = {{0, HW_FORMAT_RAW10}, {1, HW_FORMAT_UNKNOWN}, {2, HW_FORMAT_USER}, {3, 0}}
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR12,
		.config = {{0, HW_FORMAT_RAW10}, {1, HW_FORMAT_UNKNOWN}, {2, HW_FORMAT_USER}, {3, 0}}
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR16,
		.config = {{0, HW_FORMAT_RAW10}, {1, HW_FORMAT_UNKNOWN}, {2, HW_FORMAT_USER}, {3, 0}}
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

static int sensor_module_4ha_power_setpin_with_common_ldo_en(struct device *dev,
	struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_reset = 0;
	int gpio_mclk = 0;
	int gpio_cam_ldo_en = 0;
	int gpio_camio_1p8_en = 0;
	int gpio_none = 0;
	bool shared_camio_1p8 = false;

	struct fimc_is_core *core;

	BUG_ON(!dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core is NULL");
		return -EINVAL;
	}

	dnode = dev->of_node;

	dev_info(dev, "%s E v4\n", __func__);

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
	} else {
		gpio_request_one(gpio_mclk, GPIOF_OUT_INIT_LOW, "CAM_MCLK_OUTPUT_LOW");
		gpio_free(gpio_mclk);
	}

	gpio_camio_1p8_en = of_get_named_gpio(dnode, "gpio_camio_1p8_en", 0);
	if (!gpio_is_valid(gpio_camio_1p8_en)) {
		dev_err(dev, "failed to get gpio_camio_1p8_en\n");
	} else {
		gpio_request_one(gpio_camio_1p8_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_camio_1p8_en);
	}

	gpio_cam_ldo_en = of_get_named_gpio(dnode, "gpio_cam_ldo_en", 0);
	if (!gpio_is_valid(gpio_cam_ldo_en)) {
		dev_err(dev, "failed to get gpio_cam_ldo_en\n");
	} else {
		gpio_request_one(gpio_cam_ldo_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_ldo_en);
	}

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF);

	/* SENSOR_SCENARIO_NORMAL on */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 1000);
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 1, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);
		}
	}

	if (gpio_is_valid(gpio_cam_ldo_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_ldo_en, "rcam_ldo_en", PIN_OUTPUT, 1, 3000);
	}

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 2000);

	/* SENSOR_SCENARIO_NORMAL off */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 100);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 100);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 1000);

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 1500);
	if (gpio_is_valid(gpio_cam_ldo_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_ldo_en, "rcam_ldo_en", PIN_OUTPUT, 0, 0);
	}
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 0, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);
		}
	}

	/* SENSOR_SCENARIO_READ_ROM on */
	if (gpio_is_valid(gpio_cam_ldo_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_cam_ldo_en, "rcam_ldo_en", PIN_OUTPUT, 1, 5000);
	}
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 1, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, SRT_ACQUIRE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);
		}
	}

	/* SENSOR_SCENARIO_READ_ROM off */
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 0, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, SRT_RELEASE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);
		}
	}
	if (gpio_is_valid(gpio_cam_ldo_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_cam_ldo_en, "rcam_ldo_en", PIN_OUTPUT, 0, 0);
	}

	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "delay", PIN_NONE, 0, 500);

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

static int sensor_module_4ha_power_setpin(struct device *dev,
	struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_reset = 0;
	int gpio_mclk = 0;
	int gpio_camaf_2p8_en = 0;
	int gpio_cam_a2p8_en = 0;
	int gpio_cam_core_en = 0;
	int gpio_camio_1p8_en = 0;
	int gpio_none = 0;
	char *cam_dvdd_1p2 = NULL;
	bool exist_actuator = false;
	bool disable_core_ldo = false;
	bool disable_control_of_af_2p8 = false;
	bool shared_camio_1p8 = false;
	struct fimc_is_core *core;

	BUG_ON(!dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core is NULL");
		return -EINVAL;
	}

	dnode = dev->of_node;

	dev_info(dev, "%s E v4\n", __func__);

	if (pdata->af_product_name != ACTUATOR_NAME_NOTHING) {
		exist_actuator = true;
	} else {
		dev_info(dev, "%s : This is module without actuator \n", __func__);
	}

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
	} else {
		gpio_request_one(gpio_mclk, GPIOF_OUT_INIT_LOW, "CAM_MCLK_OUTPUT_LOW");
		gpio_free(gpio_mclk);
	}

	gpio_cam_a2p8_en = of_get_named_gpio(dnode, "gpio_cam_a2p8_en", 0);
	if (!gpio_is_valid(gpio_cam_a2p8_en)) {
		dev_warn(dev, "failed to get gpio_cam_a2p8_en\n");
	} else {
		gpio_request_one(gpio_cam_a2p8_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_a2p8_en);
	}

	if (exist_actuator == true) {
		gpio_camaf_2p8_en = of_get_named_gpio(dnode, "gpio_camaf_2p8_en", 0);
		if (!gpio_is_valid(gpio_camaf_2p8_en)) {
			dev_warn(dev, "failed to get gpio_camaf_2p8_en\n");
		} else {
			gpio_request_one(gpio_camaf_2p8_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
			gpio_free(gpio_camaf_2p8_en);
		}
	}

	gpio_camio_1p8_en = of_get_named_gpio(dnode, "gpio_camio_1p8_en", 0);
	if (!gpio_is_valid(gpio_camio_1p8_en)) {
		dev_warn(dev, "failed to get gpio_camio_1p8_en\n");
	} else {
		gpio_request_one(gpio_camio_1p8_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_camio_1p8_en);
	}

	gpio_cam_core_en = of_get_named_gpio(dnode, "gpio_cam_core_en", 0);
	if (!gpio_is_valid(gpio_cam_core_en)) {
		dev_warn(dev, "failed to get gpio_cam_core_en\n");
	} else {
		gpio_request_one(gpio_cam_core_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_core_en);
	}

	disable_core_ldo = of_property_read_bool(dnode, "disable_core_ldo");
	if(disable_core_ldo) {
		dev_info(dev, "[%s] Disable 1.2V core LDO\n", __func__);
	}

	disable_control_of_af_2p8 = of_property_read_bool(dnode, "disable_control_of_af_2p8");
	if(disable_control_of_af_2p8) {
		dev_info(dev, "[%s] Disable control of AF 2.8V\n", __func__);
	}

	if (of_property_read_string(dnode, "cam_dvdd_1p2", (const char **) &cam_dvdd_1p2)) {
		dev_info(dev, "Failed to get regulator_dvdd name property\n");
	}

	shared_camio_1p8 = of_property_read_bool(dnode, "shared_camio_1p8");

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF);

	/* SENSOR_SCENARIO_NORMAL on */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 0);
	if (gpio_is_valid(gpio_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_a2p8_en, "cam_a2p8_en", PIN_OUTPUT, 1, 0);
	} else {
#if !defined(AVDD_2P8_REGULATOR)
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "FCAM_AVDD_2P8", PIN_REGULATOR, 1, 0);
#endif
	}
	if (gpio_is_valid(gpio_cam_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 1, 0);
	} else {
		if (cam_dvdd_1p2) {
			SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, cam_dvdd_1p2, PIN_REGULATOR, 1, 0);
		} else { 
			if(!disable_core_ldo) {
				SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "FCAM_DVDD_1P2", PIN_REGULATOR, 1, 0);
			}
		}
	}
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 1, 2000);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 1, 2000);
	}
	if (exist_actuator == true) {
		if (gpio_is_valid(gpio_camaf_2p8_en)) {
			if(!disable_control_of_af_2p8) {
				SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_camaf_2p8_en, "camaf_2p8_en", PIN_OUTPUT, 1, 5000);
			}
		} else {
			SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDAF_2.8V_CAM", PIN_REGULATOR, 1, 5000);
		}
	}

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 3000);

	/* SENSOR_SCENARIO_NORMAL off */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 100);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 100);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 1000);
	if (exist_actuator == true) {
		if (gpio_is_valid(gpio_camaf_2p8_en)) {
			if(!disable_control_of_af_2p8) {
				SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_camaf_2p8_en, "camaf_2p8_en", PIN_OUTPUT, 0, 1500);
			}
		} else {
			SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDAF_2.8V_CAM", PIN_REGULATOR, 0, 1500);
		}
	}
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 1500);
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 0, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 0, 0);
	}
	if (gpio_is_valid(gpio_cam_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 0, 0);
	} else {
		if (cam_dvdd_1p2) {
			SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, cam_dvdd_1p2, PIN_REGULATOR, 0, 0);
		} else { 
			if(!disable_core_ldo) {
				SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "FCAM_DVDD_1P2", PIN_REGULATOR, 0, 0);
			}
		}
	}
	if (gpio_is_valid(gpio_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_a2p8_en, "cam_a2p8_en", PIN_OUTPUT, 0, 0);
	} else {
#if !defined(AVDD_2P8_REGULATOR)
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "FCAM_AVDD_2P8", PIN_REGULATOR, 0, 0);
#endif
	}
	/* SENSOR_SCENARIO_READ_ROM on */
	if (exist_actuator == true) {
		if (gpio_is_valid(gpio_camaf_2p8_en)) {
			if(!disable_control_of_af_2p8) {
				SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_camaf_2p8_en, "camaf_2p8_en", PIN_OUTPUT, 1, 5000);
			}
		} else {
			SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "VDDAF_2.8V_CAM", PIN_REGULATOR, 1, 5000);
		}
	}
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 1, 2000);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, SRT_ACQUIRE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 1, 2000);
	}

	/* SENSOR_SCENARIO_READ_ROM off */
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "camio_1p8_en", PIN_OUTPUT, 0, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, SRT_RELEASE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 0, 0);
	}

	if (exist_actuator == true) {
		if (gpio_is_valid(gpio_camaf_2p8_en)) {
			if(!disable_control_of_af_2p8) {
				SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_camaf_2p8_en, "camaf_2p8_en", PIN_OUTPUT, 0, 1500);
			}
		} else {
			SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "VDDAF_2.8V_CAM", PIN_REGULATOR, 0, 1500);
		}
	}

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

int sensor_module_4ha_probe(struct platform_device *pdev)
{
	int ret = 0;
	int num_lanes=CSI_DATA_LANES_3;
	struct fimc_is_core *core;
	struct v4l2_subdev *subdev_module;
	struct fimc_is_module_enum *module;
	struct fimc_is_device_sensor *device;
	struct sensor_open_extended *ext;
	struct exynos_platform_fimc_is_module *pdata;
	struct device *dev;
	int num_of_gpio_ldo_en = 0; //A10e ver 0.3 uses only one gpio for LDO en. default is '0'

	BUG_ON(!fimc_is_dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		probe_info("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	dev = &pdev->dev;

	ret = of_property_read_u32(dev->of_node, "num_of_gpio_ldo_en", &num_of_gpio_ldo_en);
	if (ret) {
		probe_info("[warning] Reading num_of_gpio_ldo_en is not correct, resetting to 0 (%d)", num_of_gpio_ldo_en);
		num_of_gpio_ldo_en = 0;
	}

	if (num_of_gpio_ldo_en == 1) {
		//A10e ver 0.3 uses LDO regulator
		probe_info("probe using power sequence with common ldo enable gpio");
		fimc_is_module_parse_dt(dev, sensor_module_4ha_power_setpin_with_common_ldo_en);
	} else {
		probe_info("probe using power sequence with gpio & regulator");
		fimc_is_module_parse_dt(dev, sensor_module_4ha_power_setpin);
	}

	ret = of_property_read_u32(dev->of_node, "num_of_lanes", &num_lanes);
	num_lanes=num_lanes-1;
	if (ret) {
		num_lanes=CSI_DATA_LANES_3;
		probe_info("No of lanes is set default (%d)", num_lanes);
		ret=0;
	}

	pdata = dev_get_platdata(dev);
	device = &core->sensor[pdata->id];

	subdev_module = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_module) {
		probe_err("subdev_module is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	probe_info("%s pdta->id(%d), module_enum id = %d \n", __func__, pdata->id, atomic_read(&device->module_count));
	module = &device->module_enum[atomic_read(&device->module_count)];
	atomic_inc(&device->module_count);
	clear_bit(FIMC_IS_MODULE_GPIO_ON, &module->state);
	module->pdata = pdata;
	module->dev = dev;
	module->sensor_id = SENSOR_NAME_S5K4HA;
	module->subdev = subdev_module;
	module->device = pdata->id;
	module->client = NULL;
	module->active_width = 3264;
	module->active_height = 2448;
	module->margin_left = 0;
	module->margin_right = 0;
	module->margin_top = 0;
	module->margin_bottom = 0;
	module->pixel_width = module->active_width;
	module->pixel_height = module->active_height;
	module->max_framerate = 120;
	module->position = pdata->position;
	module->mode = CSI_MODE_DT_ONLY;
	module->lanes = num_lanes;
	module->bitwidth = 10;
	module->vcis = ARRAY_SIZE(vci_module_4ha);
	module->vci = vci_module_4ha;
	module->sensor_maker = "SLSI";
	module->sensor_name = "S5K4HA";
	if (pdata->position == SENSOR_POSITION_REAR || pdata->position == SENSOR_POSITION_REAR2 || pdata->position == SENSOR_POSITION_REAR3) {
		module->setfile_name = "setfile_4ha.bin";
	} else if (pdata->position == SENSOR_POSITION_FRONT || pdata->position == SENSOR_POSITION_FRONT2) {
		module->setfile_name = "setfile_4ha_front.bin";
	}
	if (num_lanes==CSI_DATA_LANES_4) {
		module->cfgs = ARRAY_SIZE(config_module_4ha_4lane);
		module->cfg = config_module_4ha_4lane;
	} else {
		module->cfgs = ARRAY_SIZE(config_module_4ha);
		module->cfg = config_module_4ha;
	}
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

	ext->sensor_con.product_name = module->sensor_id;
	ext->sensor_con.peri_type = SE_I2C;
	ext->sensor_con.peri_setting.i2c.channel = pdata->sensor_i2c_ch;
	ext->sensor_con.peri_setting.i2c.slave_address = pdata->sensor_i2c_addr;
	ext->sensor_con.peri_setting.i2c.speed = 400000;

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
		ext->preprocessor_con.peri_info2.peri_setting.dma.channel = FLITE_ID_D;
	} else {
		ext->preprocessor_con.product_name = pdata->preprocessor_product_name;
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

	probe_info("%s done\n", __func__);

p_err:
	return ret;
}

static const struct of_device_id exynos_fimc_is_sensor_module_4ha_match[] = {
	{
		.compatible = "samsung,sensor-module-4ha",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_sensor_module_4ha_match);

static struct platform_driver sensor_module_4ha_driver = {
	.driver = {
		.name   = "FIMC-IS-SENSOR-MODULE-4HA",
		.owner  = THIS_MODULE,
		.of_match_table = exynos_fimc_is_sensor_module_4ha_match,
	}
};


static int __init fimc_is_sensor_module_4ha_init(void)
{
	int ret;

	ret = platform_driver_probe(&sensor_module_4ha_driver,
				sensor_module_4ha_probe);
	if (ret)
		err("failed to probe %s driver: %d\n",
			sensor_module_4ha_driver.driver.name, ret);

	return ret;
}
late_initcall(fimc_is_sensor_module_4ha_init);

