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

static int sensor_module_5e9_power_setpin_with_eeprom(struct device *dev,
		struct exynos_platform_fimc_is_module *pdata);
static int sensor_module_5e9_power_setpin_with_otp(struct device *dev,
		struct exynos_platform_fimc_is_module *pdata);

enum sensor_module_5e9_cal_memory {
	MODULE_5E9_CAL_MEMORY_EEPROM = 0,
	MODULE_5E9_CAL_MEMORY_OTP = 1,
	MODULE_5E9_CAL_MEMORY_MAX = 2
};

#define MAX_5E9_SETPIN_CNT MODULE_5E9_CAL_MEMORY_MAX

static int (* module_5e9_power_setpin[MAX_5E9_SETPIN_CNT])(struct device *pdev,
	struct exynos_platform_fimc_is_module *pdata) = {
	sensor_module_5e9_power_setpin_with_eeprom,
	sensor_module_5e9_power_setpin_with_otp
};

static struct fimc_is_sensor_cfg config_module_5e9[] = {
	/* 2592x1944@30fps */
	FIMC_IS_SENSOR_CFG(2592, 1944,  30, 19, 0, CSI_DATA_LANES_2),
	/* 2592x1460@30fps */
	FIMC_IS_SENSOR_CFG(2592, 1460,  30, 19, 1, CSI_DATA_LANES_2),
	/* 2592x1458@30fps */
	FIMC_IS_SENSOR_CFG(2592, 1458,  30, 19, 2, CSI_DATA_LANES_2),
	/* 2592x1260@30fps */
	FIMC_IS_SENSOR_CFG(2592, 1260,  30, 19, 3, CSI_DATA_LANES_2),
	/* 1936x1936@30fps */
	FIMC_IS_SENSOR_CFG(1936, 1936,  30, 19, 4, CSI_DATA_LANES_2),
	/* 2592x1460@24fps */
	FIMC_IS_SENSOR_CFG(2592, 1460,  24, 19, 5, CSI_DATA_LANES_2),
	/* 2592x1458@24fps */
	FIMC_IS_SENSOR_CFG(2592, 1458,  24, 19, 6, CSI_DATA_LANES_2),
	/* 2592x1260@24fps */
	FIMC_IS_SENSOR_CFG(2592, 1260,  24, 19, 7, CSI_DATA_LANES_2),
	/* 1936x1936@24fps */
	FIMC_IS_SENSOR_CFG(1936, 1936,  24, 19, 8, CSI_DATA_LANES_2),
	/* 1296x972@58fps */
	FIMC_IS_SENSOR_CFG(1296, 972 ,  58, 19, 9, CSI_DATA_LANES_2),
	/* 2576x1932@30fps */
	FIMC_IS_SENSOR_CFG(2576, 1932,  30, 19, 10, CSI_DATA_LANES_2),
	/* 2576x1188@30fps */
	FIMC_IS_SENSOR_CFG(2576, 1188,  30, 19, 11, CSI_DATA_LANES_2),
	/* 2560x1440@30fps */
	FIMC_IS_SENSOR_CFG(2560, 1440,  30, 19, 12, CSI_DATA_LANES_2),
	/* 2224x1080@30fps */
	FIMC_IS_SENSOR_CFG(2224, 1080,  30, 19, 13, CSI_DATA_LANES_2),
	/* 1920x1920@30fps */
	FIMC_IS_SENSOR_CFG(1920, 1920,  30, 19, 14, CSI_DATA_LANES_2),
	/* 1280x960@30fps */
	FIMC_IS_SENSOR_CFG(1280, 960,  30, 19, 15, CSI_DATA_LANES_2),
	/* 1280x720@30fps */
	FIMC_IS_SENSOR_CFG(1280, 720,  30, 19, 16, CSI_DATA_LANES_2),
	/* 640x480@120fps */
	FIMC_IS_SENSOR_CFG(640 , 480 , 120, 19, 17, CSI_DATA_LANES_2),
};

static struct fimc_is_vci vci_module_5e9[] = {
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

static int sensor_module_5e9_power_setpin_with_eeprom(struct device *dev,
		struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_reset = 0;
	int gpio_mclk = 0;
	int gpio_none = 0;
	int gpio_camio_1p8_en = 0;
#ifdef USE_COMMON_CAM_IO_PWR
	int gpio_cam_1p2_cam_a2p8_en = 0;
#else
	int gpio_cam_a2p8_en = 0;
	int gpio_cam_core_en = 0;
#endif
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

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(dev, "failed to get gpio_reset\n");
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
		dev_err(dev, "failed to get PIN_POWER_EN\n");
	} else {
		gpio_request_one(gpio_camio_1p8_en, GPIOF_OUT_INIT_LOW, "CAM_VDDIO_EN");
		gpio_free(gpio_camio_1p8_en);
	}

#ifdef USE_COMMON_CAM_IO_PWR
	gpio_cam_1p2_cam_a2p8_en = of_get_named_gpio(dnode, "gpio_cam_1p2_cam_a2p8_en", 0);
	if (!gpio_is_valid(gpio_cam_1p2_cam_a2p8_en)) {
		dev_err(dev, "failed to get PIN_POWER_EN\n");
	} else {
		gpio_request_one(gpio_cam_1p2_cam_a2p8_en, GPIOF_OUT_INIT_LOW, "CAM_CORE_AVDD_EN");
		gpio_free(gpio_cam_1p2_cam_a2p8_en);
	}
#else
	gpio_cam_a2p8_en = of_get_named_gpio(dnode, "gpio_cam_a2p8_en", 0);
	if (!gpio_is_valid(gpio_cam_a2p8_en)) {
		dev_err(dev, "failed to get gpio_cam_a2p8_en\n");
	} else {
		gpio_request_one(gpio_cam_a2p8_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_a2p8_en);
	}

	gpio_cam_core_en = of_get_named_gpio(dnode, "gpio_cam_core_en", 0);
	if (!gpio_is_valid(gpio_cam_core_en)) {
		dev_err(dev, "failed to get gpio_cam_core_en\n");
	} else {
		gpio_request_one(gpio_cam_core_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_core_en);
	}
#endif

	shared_mclk = of_property_read_bool(dnode, "shared_mclk");
	shared_camio_1p8 = of_property_read_bool(dnode, "shared_camio_1p8");

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF);

	/***** [ Normal On ] ************************************************************/
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 100);

#ifdef USE_COMMON_CAM_IO_PWR
	if (gpio_is_valid(gpio_cam_1p2_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_1p2_cam_a2p8_en, "sensor_1p2_2p8_en", PIN_OUTPUT, 1, 500);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDAR2_2.8V_CAM", PIN_REGULATOR, 1, 0);	//LDO07
	}
#else
	if (gpio_is_valid(gpio_cam_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 1, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDDF_1.2V_CAM", PIN_REGULATOR, 1, 0);
	}
	if (gpio_is_valid(gpio_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_a2p8_en, "gpio_cam_a2p8_en", PIN_OUTPUT, 1, 0);
	}
#endif

	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "sensor_1p8_en", PIN_OUTPUT, 1, 500);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDIOR1_1.8V_CAM", PIN_REGULATOR, 1, 500);	//LDO03
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDDR2_1.2V_CAM", PIN_REGULATOR, 1, 500);	//LDO04
	}


	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 0);
	if(shared_mclk) {
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
				&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 1);
	}

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 3000);

	/***** [ Normal Off ] **********************************************************/
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 500);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	if(shared_mclk) {
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
				&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 0);
	}

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);

#ifdef USE_COMMON_CAM_IO_PWR
	if (gpio_is_valid(gpio_cam_1p2_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_1p2_cam_a2p8_en, "sensor_1p2_2p8_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDAR2_2.8V_CAM", PIN_REGULATOR, 0, 0);	//LDO07
	}
#else
	if (gpio_is_valid(gpio_cam_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDDF_1.2V_CAM", PIN_REGULATOR, 0, 0);
	}
	if (gpio_is_valid(gpio_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_a2p8_en, "gpio_cam_a2p8_en", PIN_OUTPUT, 0, 0);
	}
#endif
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "sensor_1p8_en", PIN_OUTPUT, 0, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDIOR1_1.8V_CAM", PIN_REGULATOR, 0, 0);	//LDO03
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDDR2_1.2V_CAM", PIN_REGULATOR, 0, 0);	//LDO04
	}

	/***** [ ROM powerOn ] *************************************************************/
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "sensor_1p8_en", PIN_OUTPUT, 1, 500);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, SRT_ACQUIRE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "VDDIOR1_1.8V_CAM", PIN_REGULATOR, 1, 500);	//LDO03
	}

	/***** [ ROM power Off ] ***********************************************************/
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "sensor_1p8_en", PIN_OUTPUT, 0, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, SRT_RELEASE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "VDDIOR1_1.8V_CAM", PIN_REGULATOR, 0, 0);	//ldO03
	}

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

static int sensor_module_5e9_power_setpin_with_otp(struct device *dev,
		struct exynos_platform_fimc_is_module *pdata)
{
	struct device_node *dnode;
	int gpio_reset = 0;
	int gpio_mclk = 0;
	int gpio_none = 0;
	int gpio_camio_1p8_en = 0;
#ifdef USE_COMMON_CAM_IO_PWR
	int gpio_cam_1p2_cam_a2p8_en = 0;
#else
	int gpio_cam_a2p8_en = 0;
	int gpio_cam_core_en = 0;
#endif
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

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		dev_err(dev, "failed to get gpio_reset\n");
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
		dev_err(dev, "failed to get PIN_POWER_EN\n");
	} else {
		gpio_request_one(gpio_camio_1p8_en, GPIOF_OUT_INIT_LOW, "CAM_VDDIO_EN");
		gpio_free(gpio_camio_1p8_en);
	}

#ifdef USE_COMMON_CAM_IO_PWR
	gpio_cam_1p2_cam_a2p8_en = of_get_named_gpio(dnode, "gpio_cam_1p2_cam_a2p8_en", 0);
	if (!gpio_is_valid(gpio_cam_1p2_cam_a2p8_en)) {
		dev_err(dev, "failed to get PIN_POWER_EN\n");
	} else {
		gpio_request_one(gpio_cam_1p2_cam_a2p8_en, GPIOF_OUT_INIT_LOW, "CAM_CORE_AVDD_EN");
		gpio_free(gpio_cam_1p2_cam_a2p8_en);
	}
#else
	gpio_cam_a2p8_en = of_get_named_gpio(dnode, "gpio_cam_a2p8_en", 0);
	if (!gpio_is_valid(gpio_cam_a2p8_en)) {
		dev_err(dev, "failed to get gpio_cam_a2p8_en\n");
	} else {
		gpio_request_one(gpio_cam_a2p8_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_a2p8_en);
	}

	gpio_cam_core_en = of_get_named_gpio(dnode, "gpio_cam_core_en", 0);
	if (!gpio_is_valid(gpio_cam_core_en)) {
		dev_err(dev, "failed to get gpio_cam_core_en\n");
	} else {
		gpio_request_one(gpio_cam_core_en, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
		gpio_free(gpio_cam_core_en);
	}
#endif

	shared_mclk = of_property_read_bool(dnode, "shared_mclk");
	shared_camio_1p8 = of_property_read_bool(dnode, "shared_camio_1p8");

	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON);
	SET_PIN_INIT(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF);

	/***** [ Normal On ] ************************************************************/
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 100);

#ifdef USE_COMMON_CAM_IO_PWR
	if (gpio_is_valid(gpio_cam_1p2_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_1p2_cam_a2p8_en, "sensor_1p2_2p8_en", PIN_OUTPUT, 1, 500);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDAR2_2.8V_CAM", PIN_REGULATOR, 1, 0);	//LDO07
	}
#else
	if (gpio_is_valid(gpio_cam_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 1, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDDF_1.2V_CAM", PIN_REGULATOR, 1, 0);
	}
	if (gpio_is_valid(gpio_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_cam_a2p8_en, "gpio_cam_a2p8_en", PIN_OUTPUT, 1, 0);
	}
#endif

	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "sensor_1p8_en", PIN_OUTPUT, 1, 500);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDIOR1_1.8V_CAM", PIN_REGULATOR, 1, 500);	//LDO03
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDDR2_1.2V_CAM", PIN_REGULATOR, 1, 500);	//LDO04
	}


	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 0);
	if(shared_mclk) {
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, SRT_ACQUIRE,
				&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 1);
	}

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 3000);

	/***** [ Normal Off ] **********************************************************/
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 500);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	if(shared_mclk) {
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
				&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 0);
	}

	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);

#ifdef USE_COMMON_CAM_IO_PWR
	if (gpio_is_valid(gpio_cam_1p2_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_1p2_cam_a2p8_en, "sensor_1p2_2p8_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDAR2_2.8V_CAM", PIN_REGULATOR, 0, 0);	//LDO07
	}
#else
	if (gpio_is_valid(gpio_cam_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDDF_1.2V_CAM", PIN_REGULATOR, 0, 0);
	}
	if (gpio_is_valid(gpio_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_cam_a2p8_en, "gpio_cam_a2p8_en", PIN_OUTPUT, 0, 0);
	}
#endif
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "sensor_1p8_en", PIN_OUTPUT, 0, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, SRT_RELEASE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDIOR1_1.8V_CAM", PIN_REGULATOR, 0, 0);	//LDO03
		SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF, gpio_none, "VDDDR2_1.2V_CAM", PIN_REGULATOR, 0, 0);	//LDO04
	}


	/***** [ ROM powerOn ] *************************************************************/
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 100);
#ifdef USE_COMMON_CAM_IO_PWR
	if (gpio_is_valid(gpio_cam_1p2_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_cam_1p2_cam_a2p8_en, "sensor_1p2_2p8_en", PIN_OUTPUT, 1, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "VDDAR2_2.8V_CAM", PIN_REGULATOR, 1, 0);	//LDO07
	}
#else
	if (gpio_is_valid(gpio_cam_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 1, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "VDDDF_1.2V_CAM", PIN_REGULATOR, 1, 0);
	}
	if (gpio_is_valid(gpio_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_cam_a2p8_en, "gpio_cam_a2p8_en", PIN_OUTPUT, 1, 0);
	}
#endif
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_camio_1p8_en, "sensor_1p8_en", PIN_OUTPUT, 1, 500);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, SRT_ACQUIRE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 1);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "VDDIOR1_1.8V_CAM", PIN_REGULATOR, 1, 500);	//LDO03
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "VDDDR2_1.2V_CAM", PIN_REGULATOR, 1, 500);	//LDO04
	}
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 0);
	if(shared_mclk) {
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, SRT_ACQUIRE,
				&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 1);
	}
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 1000);

	/***** [ ROM power Off ] ***********************************************************/
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 1, 0);
	if(shared_mclk) {
		SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, SRT_RELEASE,
				&core->shared_rsc_slock[SHARED_PIN0], &core->shared_rsc_count[SHARED_PIN0], 0);
	}
	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "pin", PIN_FUNCTION, 0, 0);

	SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_reset, "sen_rst", PIN_OUTPUT, 0, 100);

#ifdef USE_COMMON_CAM_IO_PWR
	if (gpio_is_valid(gpio_cam_1p2_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_cam_1p2_cam_a2p8_en, "sensor_1p2_2p8_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "VDDAR2_2.8V_CAM", PIN_REGULATOR, 0, 0);	//LDO07
	}
#else
	if (gpio_is_valid(gpio_cam_core_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_cam_core_en, "cam_core_en", PIN_OUTPUT, 0, 0);
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "VDDDF_1.2V_CAM", PIN_REGULATOR, 0, 0);
	}
	if (gpio_is_valid(gpio_cam_a2p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_cam_a2p8_en, "gpio_cam_a2p8_en", PIN_OUTPUT, 0, 0);
	}
#endif
	if (gpio_is_valid(gpio_camio_1p8_en)) {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_camio_1p8_en, "sensor_1p8_en", PIN_OUTPUT, 0, 0);
		if(shared_camio_1p8) {
			SET_PIN_SHARED(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, SRT_RELEASE,
					&core->shared_rsc_slock[SHARED_PIN1], &core->shared_rsc_count[SHARED_PIN1], 0);
		}
	} else {
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "VDDIOR1_1.8V_CAM", PIN_REGULATOR, 0, 0);	//ldO03
		SET_PIN(pdata, SENSOR_SCENARIO_READ_ROM, GPIO_SCENARIO_OFF, gpio_none, "VDDDR2_1.2V_CAM", PIN_REGULATOR, 0, 0);	//ldO04
	}

	dev_info(dev, "%s X v4\n", __func__);

	return 0;
}

static int __init sensor_module_5e9_probe(struct platform_device *pdev)
{
	int ret = 0;
	unsigned int use_cal_memory = 0;
	struct fimc_is_core *core;
	struct v4l2_subdev *subdev_module;
	struct fimc_is_module_enum *module;
	struct fimc_is_device_sensor *device;
	struct sensor_open_extended *ext;
	struct exynos_platform_fimc_is_module *pdata;
	struct device *dev;
	struct pinctrl_state *s;

	BUG_ON(!fimc_is_dev);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		probe_info("core device is not yet probed");
		return -EPROBE_DEFER;
	}

	dev = &pdev->dev;

#ifdef SENSOR_OTP_5E9
	use_cal_memory = MODULE_5E9_CAL_MEMORY_OTP;
#else
	use_cal_memory = MODULE_5E9_CAL_MEMORY_EEPROM;
#endif
	fimc_is_module_parse_dt(dev, module_5e9_power_setpin[use_cal_memory]);

	pdata = dev_get_platdata(dev);
	device = &core->sensor[pdata->id];

	subdev_module = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!subdev_module) {
		probe_err("subdev_module is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	module = &device->module_enum[atomic_read(&device->module_count)];
	atomic_inc(&device->module_count);
	clear_bit(FIMC_IS_MODULE_GPIO_ON, &module->state);
	module->pdata = pdata;
	module->dev = dev;
	module->sensor_id = SENSOR_NAME_S5K5E9;
	module->subdev = subdev_module;
	module->device = pdata->id;
	module->client = NULL;
	module->active_width = 2592;
	module->active_height = 1944;
	module->margin_left = 0;
	module->margin_right = 0;
	module->margin_top = 0;
	module->margin_bottom = 0;
	module->pixel_width = module->active_width;
	module->pixel_height = module->active_height;
	module->max_framerate = 120;
	module->position = pdata->position;
	module->mode = CSI_MODE_DT_ONLY;
	module->lanes = CSI_DATA_LANES_2;
	module->bitwidth = 10;
	module->vcis = ARRAY_SIZE(vci_module_5e9);
	module->vci = vci_module_5e9;
	module->sensor_maker = "SLSI";
	module->sensor_name = "S5K5E9";
	module->setfile_name = "setfile_5e9.bin";
	module->cfgs = ARRAY_SIZE(config_module_5e9);
	module->cfg = config_module_5e9;
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

	s = pinctrl_lookup_state(pdata->pinctrl, "release");

	if (pinctrl_select_state(pdata->pinctrl, s) < 0) {
		probe_err("pinctrl_select_state is fail\n");
		goto p_err;
	}
p_err:
	probe_info("%s done(%d)\n", __func__, ret);
	return ret;
}

static int sensor_module_5e9_remove(struct platform_device *pdev)
{
	int ret = 0;

	info("%s\n", __func__);

	return ret;
}

static const struct of_device_id exynos_fimc_is_sensor_module_5e9_match[] = {
	{
		.compatible = "samsung,sensor-module-5e9",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_sensor_module_5e9_match);

static struct platform_driver sensor_module_5e9_driver = {
	.probe  = sensor_module_5e9_probe,
	.remove = sensor_module_5e9_remove,
	.driver = {
		.name   = "FIMC-IS-SENSOR-MODULE-5E9",
		.owner  = THIS_MODULE,
		.of_match_table = exynos_fimc_is_sensor_module_5e9_match,
	}
};

module_platform_driver(sensor_module_5e9_driver);
