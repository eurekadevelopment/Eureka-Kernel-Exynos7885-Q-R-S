/*
 * Samsung Exynos5 SoC series Sensor driver
 *
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
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

#define MAX_2P6_SETPIN_CNT 2

enum sensor_module_2p6_actuator {
	SENSOR_MODULE_2P6_WITHOUT_ACTUATOR = 0,
	SENSOR_MODULE_2P6_WITH_ACTUATOR = 1,
};

enum sensor_module_2p6_position {
	SENSOR_MODULE_2P6_REAR = 0,
	SENSOR_MODULE_2P6_FRONT = 1,
};

static struct fimc_is_sensor_cfg config_module_2p6[] = {
	/* 4608x3456@30fps */
	FIMC_IS_SENSOR_CFG_EXT(4608, 3456, 30, 32, 0, CSI_DATA_LANES_4, 1443, SET_VC(VC_TAIL_MODE_PDAF, 288, 1728), 0, 0),
	/* 4608x2624@30fps */
	FIMC_IS_SENSOR_CFG_EXT(4608, 2624, 30, 32, 1, CSI_DATA_LANES_4, 1443, SET_VC(VC_TAIL_MODE_PDAF, 288, 1312), 0, 0),
	/** 4608x2592@30fps */
	FIMC_IS_SENSOR_CFG_EXT(4608, 2592, 30, 32, 2, CSI_DATA_LANES_4, 1443, SET_VC(VC_TAIL_MODE_PDAF, 288, 1296), 0, 0),
	/* 4608x2240@30fps */
	FIMC_IS_SENSOR_CFG_EXT(4608, 2240, 30, 32, 3, CSI_DATA_LANES_4, 1443, SET_VC(VC_TAIL_MODE_PDAF, 288, 1120), 0, 0),
	/** 4608x2176@30fps */
	FIMC_IS_SENSOR_CFG_EXT(4608, 2176, 30, 32, 4, CSI_DATA_LANES_4, 1443, SET_VC(VC_TAIL_MODE_PDAF, 288, 1088), 0, 0),
	/** 3456x3456@30fps */
	FIMC_IS_SENSOR_CFG_EXT(3456, 3456, 30, 32, 5, CSI_DATA_LANES_4, 1443, SET_VC(VC_TAIL_MODE_PDAF, 216, 1728), 0, 0),
	/* 2304x1728@30fps */
	FIMC_IS_SENSOR_CFG_EXT(2304, 1728, 30, 30, 6, CSI_DATA_LANES_4, 1352, SET_VC(VC_TAIL_MODE_PDAF, 288, 1728), 0, 0),
	/* 2304x1728@15fps */
	FIMC_IS_SENSOR_CFG_EXT(2304, 1728, 15, 30, 7, CSI_DATA_LANES_4, 1352, SET_VC(VC_TAIL_MODE_PDAF, 288, 1728), 0, 0),
	/* 2304x1312@30fps */
	FIMC_IS_SENSOR_CFG_EXT(2304, 1312, 30, 30, 8, CSI_DATA_LANES_4, 1352, SET_VC(VC_TAIL_MODE_PDAF, 288, 1312), 0, 0),
	/* 2304x1296@30fps */
	FIMC_IS_SENSOR_CFG_EXT(2304, 1296, 30, 30, 9, CSI_DATA_LANES_4, 1352, SET_VC(VC_TAIL_MODE_PDAF, 288, 1296), 0, 0),
	/* 2304x1120@30fps */
	FIMC_IS_SENSOR_CFG_EXT(2304, 1120, 30, 30, 10, CSI_DATA_LANES_4, 1352, SET_VC(VC_TAIL_MODE_PDAF, 288, 1120), 0, 0),
	/* 1280x720@120fps */
	FIMC_IS_SENSOR_CFG_EXT(1280, 720, 120, 30, 11, CSI_DATA_LANES_4, 1352, 0, 0, 0),
	/* 1152x864@120fps */
	FIMC_IS_SENSOR_CFG_EXT(1152, 864, 120, 30, 12, CSI_DATA_LANES_4, 1352, 0, 0, 0),
	/* 1152x656@120fps */
	FIMC_IS_SENSOR_CFG_EXT(1152, 656, 120, 30, 13, CSI_DATA_LANES_4, 1352, 0, 0, 0),
};

static struct fimc_is_vci vci_module_2p6[] = {
	{
		.pixelformat = V4L2_PIX_FMT_SBGGR10,
		.config = {{0, HW_FORMAT_RAW10, VCI_DMA_NORMAL},
			{1, HW_FORMAT_RAW10, VCI_DMA_INTERNAL},
			{2, 0, VCI_DMA_NORMAL},
			{3, 0, VCI_DMA_NORMAL}}
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR12,
		.config = {{0, HW_FORMAT_RAW10, VCI_DMA_NORMAL},
			{1, HW_FORMAT_RAW10, VCI_DMA_INTERNAL},
			{2, 0, VCI_DMA_NORMAL},
			{3, 0, VCI_DMA_NORMAL}}
	}, {
		.pixelformat = V4L2_PIX_FMT_SBGGR16,
		.config = {{0, HW_FORMAT_RAW10, VCI_DMA_NORMAL},
			{1, HW_FORMAT_RAW10, VCI_DMA_INTERNAL},
			{2, 0, VCI_DMA_NORMAL},
			{3, 0, VCI_DMA_NORMAL}}
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

static int sensor_module_2p6_power_setpin_with_af(struct device *dev,
	struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_reset = 0;
	int gpio_mclk = 0;
	int gpio_none = 0;
	int gpio_core_en = 0;
	int gpio_cam_avdd_en = 0;
	int gpio_cam_af_en = 0;
	int gpio_cam_io_en = 0;

	BUG_ON(!dev);

	dnode = dev->of_node;

	dev_info(dev, "%s E v4\n", __func__);

	gpio_mclk = of_get_named_gpio(dnode, "gpio_mclk", 0);
	if (!gpio_is_valid(gpio_mclk)) {
		dev_err(dev, "failed to get gpio_mclk\n");
		return -EINVAL;
	} else {
		gpio_request_one(gpio_mclk, GPIOF_OUT_INIT_LOW, "CAM_MCLK_OUTPUT_LOW");
		gpio_free(gpio_mclk);
	}

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(dev, "failed to get gpio_reset\n");
		return -EINVAL;
	} else {
		gpio_request_one(gpio_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_reset);
	}

	gpio_core_en = of_get_named_gpio(dnode, "gpio_core_en", 0);
	if (!gpio_is_valid(gpio_core_en)) {
		dev_warn(dev, "failed to get gpio_core_en\n");
	} else {
		gpio_request_one(gpio_core_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_core_en);
	}

	gpio_cam_avdd_en = of_get_named_gpio(dnode, "gpio_cam_avdd_en", 0);
	if (!gpio_is_valid(gpio_cam_avdd_en)) {
		dev_warn(dev, "failed to get gpio_cam_avdd_en\n");
	} else {
		gpio_request_one(gpio_cam_avdd_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_avdd_en);
	}

	gpio_cam_af_en = of_get_named_gpio(dnode, "gpio_cam_af_en", 0);
	if (!gpio_is_valid(gpio_cam_af_en)) {
		dev_warn(dev,"failed to get gpio_cam_af_en\n");
	} else {
		gpio_request_one(gpio_cam_af_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_af_en);
	}

	gpio_cam_io_en = of_get_named_gpio(dnode, "gpio_cam_io_en", 0);
	if (!gpio_is_valid(gpio_cam_io_en)) {
		dev_warn(dev, "failed to get gpio_cam_io_en\n");
	} else {
		gpio_request_one(gpio_cam_io_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_io_en);
	}

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF);

	/* BACK CAMERA - POWER ON */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "pdaf sen_rst low", PIN_OUTPUT, 0, 0);

	if (gpio_is_valid(gpio_cam_avdd_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_avdd_en, "pdaf gpio_cam_avdd_en", PIN_OUTPUT, 1, 200);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "RCAM1_AVDD_2P8", PIN_REGULATOR, 1, 200);
	}

	if (gpio_is_valid(gpio_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_core_en, "pdaf gpio_core_en", PIN_OUTPUT, 1, 200);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "RCAM1_DVDD_1P05", PIN_REGULATOR, 1, 200);
	}

	if (gpio_is_valid(gpio_cam_io_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_io_en, "pdaf gpio_cam_io_en", PIN_OUTPUT, 1, 5000);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 1, 500);
	}

	if (gpio_is_valid(gpio_cam_af_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_af_en, "pdaf gpio_cam_af_en", PIN_OUTPUT, 1, 7000);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDD_CAM_AF_2P8", PIN_REGULATOR, 1, 7000);
	}

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "pdaf sen_rst high", PIN_OUTPUT, 1, 500);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 5000);

	/* BACK CAEMRA - POWER OFF */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin_none", PIN_NONE, 1, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "pdaf sen_rst", PIN_OUTPUT, 0, 3000);

	if (gpio_is_valid(gpio_cam_af_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_af_en, "pdaf gpio_cam_af_en", PIN_OUTPUT, 0, 200);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDD_CAM_AF_2P8", PIN_REGULATOR, 0, 200);
	}

	if (gpio_is_valid(gpio_cam_io_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_io_en, "pdaf gpio_cam_io_en", PIN_OUTPUT, 0, 100);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 0, 100);
	}

	if (gpio_is_valid(gpio_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_core_en, "pdaf gpio_core_en", PIN_OUTPUT, 0, 100);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "RCAM1_DVDD_1P05", PIN_REGULATOR, 0, 100);
	}

	if (gpio_is_valid(gpio_cam_avdd_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_avdd_en, "pdaf gpio_cam_avdd_en", PIN_OUTPUT, 0, 100);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "RCAM1_AVDD_2P8", PIN_REGULATOR, 0, 100);
	}

	/* READ_ROM - POWER ON */
	if (gpio_is_valid(gpio_cam_io_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_cam_io_en, "pdaf gpio_cam_io_en", PIN_OUTPUT, 1, 2000);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 1, 2000);
	}

	if (gpio_is_valid(gpio_cam_af_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_cam_af_en, "pdaf gpio_cam_af_en", PIN_OUTPUT, 1, 3000);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "VDD_CAM_AF_2P8", PIN_REGULATOR, 1, 3000);
	}

	/* READ_ROM - POWER OFF */
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "pin_none", PIN_NONE, 1, 500);
	
	if (gpio_is_valid(gpio_cam_io_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_cam_io_en, "pdaf gpio_cam_io_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 0, 0);
	}

	if (gpio_is_valid(gpio_cam_af_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_cam_af_en, "pdaf gpio_cam_af_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "VDD_CAM_AF_2P8", PIN_REGULATOR, 0, 0);
	}

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

static int sensor_module_2p6_power_setpin(struct device *dev,
	struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_reset = 0;
	int gpio_mclk = 0;
	int gpio_none = 0;
	int gpio_core_en = 0;
	int gpio_cam_avdd_en = 0;
	int gpio_cam_io_en = 0;

	BUG_ON(!dev);

	dnode = dev->of_node;

	dev_info(dev, "%s E v4\n", __func__);

	gpio_mclk = of_get_named_gpio(dnode, "gpio_mclk", 0);
	if (!gpio_is_valid(gpio_mclk)) {
		dev_err(dev, "failed to get gpio_mclk\n");
		return -EINVAL;
	} else {
		gpio_request_one(gpio_mclk, GPIOF_OUT_INIT_LOW, "CAM_MCLK_OUTPUT_LOW");
		gpio_free(gpio_mclk);
	}

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(dev, "failed to get gpio_reset\n");
		return -EINVAL;
	} else {
		gpio_request_one(gpio_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_reset);
	}

	gpio_core_en = of_get_named_gpio(dnode, "gpio_core_en", 0);
	if (!gpio_is_valid(gpio_core_en)) {
		dev_warn(dev, "failed to get gpio_core_en\n");
	} else {
		gpio_request_one(gpio_core_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_core_en);
	}

	gpio_cam_avdd_en = of_get_named_gpio(dnode, "gpio_cam_avdd_en", 0);
	if (!gpio_is_valid(gpio_cam_avdd_en)) {
		dev_warn(dev, "failed to get gpio_cam_avdd_en\n");
	} else {
		gpio_request_one(gpio_cam_avdd_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_avdd_en);
	}

	gpio_cam_io_en = of_get_named_gpio(dnode, "gpio_cam_io_en", 0);
	if (!gpio_is_valid(gpio_cam_io_en)) {
		dev_warn(dev, "failed to get gpio_cam_io_en\n");
	} else {
		gpio_request_one(gpio_cam_io_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_io_en);
	}

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF);

	/* BACK CAMERA - POWER ON */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 0);

	if (gpio_is_valid(gpio_cam_avdd_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_avdd_en, "gpio_cam_avdd_en", PIN_OUTPUT, 1, 200);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "RCAM1_AVDD_2P8", PIN_REGULATOR, 1, 200);
	}

	if (gpio_is_valid(gpio_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_core_en, "gpio_core_en", PIN_OUTPUT, 1, 200);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "RCAM1_DVDD_1P05", PIN_REGULATOR, 1, 200);
	}

	if (gpio_is_valid(gpio_cam_io_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_io_en, "gpio_cam_io_en", PIN_OUTPUT, 1, 1000);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 1, 5000);
	}

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 500);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 3000);

	/* BACK CAEMRA - POWER OFF */
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin_none", PIN_NONE, 1, 1000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 3000);

	if (gpio_is_valid(gpio_cam_io_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_io_en, "gpio_cam_io_en", PIN_OUTPUT, 0, 100);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 0, 100);
	}

	if (gpio_is_valid(gpio_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_core_en, "gpio_core_en", PIN_OUTPUT, 0, 100);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "RCAM1_DVDD_1P05", PIN_REGULATOR, 0, 100);
	}

	if (gpio_is_valid(gpio_cam_avdd_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_avdd_en, "gpio_cam_avdd_en", PIN_OUTPUT, 0, 100);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "RCAM1_AVDD_2P8", PIN_REGULATOR, 0, 100);
	}

	/* READ_ROM - POWER ON */
	if (gpio_is_valid(gpio_cam_io_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_cam_io_en, "gpio_cam_io_en", PIN_OUTPUT, 1, 2000);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 1, 2000);
	}

	/* READ_ROM - POWER OFF */
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "pin_none", PIN_NONE, 1, 500);

	if (gpio_is_valid(gpio_cam_io_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_cam_io_en, "gpio_cam_io_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "CAM_VDDIO_1P8", PIN_REGULATOR, 0, 0);
	}

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

static int (* module_2p6_power_setpin[MAX_2P6_SETPIN_CNT])(struct device *pdev,
	struct exynos_platform_fimc_is_module *pdata) = {
	sensor_module_2p6_power_setpin,
	sensor_module_2p6_power_setpin_with_af
};

int sensor_module_2p6_probe(struct platform_device *pdev)
{
	int ret = 0;
#ifdef USE_AP_PDAF
	bool use_pdaf = false;
#endif
	u8 exist_actuator = 0;
	int ch, t;
	struct fimc_is_core *core;
	struct v4l2_subdev *subdev_module;
	struct fimc_is_module_enum *module;
	struct fimc_is_device_sensor *device;
	struct sensor_open_extended *ext;
	struct exynos_platform_fimc_is_module *pdata;
	struct device *dev;
	struct device_node *af_np;

	BUG_ON(!fimc_is_dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		probe_info("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	dev = &pdev->dev;

#ifdef USE_AP_PDAF
	if (of_property_read_bool(dev->of_node, "use_pdaf")) {
		use_pdaf = true;
	}

	probe_info("%s use_pdaf(%d)\n", __func__, use_pdaf);
#endif

	af_np = of_find_node_by_name(dev->of_node, "af");
	if (!af_np)
		exist_actuator = SENSOR_MODULE_2P6_WITHOUT_ACTUATOR;
	else
		exist_actuator = SENSOR_MODULE_2P6_WITH_ACTUATOR;

	probe_info("%s exist_actuator(%d)\n", __func__, exist_actuator);

	fimc_is_module_parse_dt(dev, module_2p6_power_setpin[exist_actuator]);

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
	module->sensor_id = SENSOR_NAME_S5K2P6;
	module->subdev = subdev_module;
	module->device = pdata->id;
	module->client = NULL;
	module->active_width = 4608 + 0;
	module->active_height = 3456 + 0;
	module->margin_left = 0;
	module->margin_right = 0;
	module->margin_top = 0;
	module->margin_bottom = 0;
	module->pixel_width = module->active_width;
	module->pixel_height = module->active_height;
	module->max_framerate = 120;
	module->position = pdata->position;
	module->mode = CSI_MODE_VC_DT;
	module->lanes = CSI_DATA_LANES_4;
	module->bitwidth = 10;
	module->vcis = ARRAY_SIZE(vci_module_2p6);
	module->vci = vci_module_2p6;
	module->sensor_maker = "SLSI";
#ifdef USE_AP_PDAF
	if (use_pdaf == true) {
		module->sensor_name = "S5K2P6SX"; /* pdaf sensor */
	} else
#endif
	{
		module->sensor_name = "S5K2P6"; /* default */
	}
	if (pdata->position == SENSOR_MODULE_2P6_REAR) {
		module->setfile_name = "setfile_2p6.bin";
	} else if (pdata->position == SENSOR_MODULE_2P6_FRONT) {
		module->setfile_name = "setfile_2p6_front.bin";
	}
	module->cfgs = ARRAY_SIZE(config_module_2p6);
	module->cfg = config_module_2p6;
	module->ops = NULL;
	for (ch = 1; ch < CSI_VIRTUAL_CH_MAX; ch++) {
		module->internal_vc[ch] = pdata->internal_vc[ch];
		module->vc_buffer_offset[ch] = pdata->vc_buffer_offset[ch];
	}

	for (t = VC_BUF_DATA_TYPE_SENSOR_STAT1; t < VC_BUF_DATA_TYPE_MAX; t++) {
		module->vc_max_size[t].stat_type = VC_STAT_TYPE_INVALID;
		module->vc_max_size[t].sensor_mode = VC_SENSOR_MODE_INVALID;
		module->vc_max_size[t].width = 0;
		module->vc_max_size[t].height = 0;
		module->vc_max_size[t].element_size = 0;

		switch (t) {
		case VC_BUF_DATA_TYPE_SENSOR_STAT1:
			module->vc_max_size[t].stat_type
				= VC_STAT_TYPE_TAIL_FOR_SW_PDAF;

			module->vc_max_size[t].sensor_mode = VC_SENSOR_MODE_ULTRA_PD_TAIL;
			module->vc_max_size[t].width = 288;
			module->vc_max_size[t].height = 1728;
			module->vc_max_size[t].element_size = 2;
			break;
		}
	}

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

static int sensor_module_2p6_remove(struct platform_device *pdev)
{
        int ret = 0;

        info("%s\n", __func__);

        return ret;
}

static const struct of_device_id exynos_fimc_is_sensor_module_2p6_match[] = {
	{
		.compatible = "samsung,sensor-module-2p6",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_sensor_module_2p6_match);

static struct platform_driver sensor_module_2p6_driver = {
	.probe  = sensor_module_2p6_probe,
	.remove = sensor_module_2p6_remove,
	.driver = {
		.name   = "FIMC-IS-SENSOR-MODULE-2P6",
		.owner  = THIS_MODULE,
		.of_match_table = exynos_fimc_is_sensor_module_2p6_match,
	}
};

module_platform_driver(sensor_module_2p6_driver);
