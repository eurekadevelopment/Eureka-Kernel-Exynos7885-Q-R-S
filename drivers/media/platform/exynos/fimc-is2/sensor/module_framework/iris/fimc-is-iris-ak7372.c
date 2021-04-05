/*
 * Samsung Exynos SoC series Actuator driver
 *
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <dt-bindings/camera/fimc_is.h>

#include "fimc-is-iris-ak7372.h"
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-helper-i2c.h"
#include "interface/fimc-is-interface-library.h"
#include "fimc-is-vender-specific.h"

#define IRIS_NAME		"IRIS_AK7372"

struct i2c_client *fimc_is_aperture_i2c_get_client(struct fimc_is_core *core)
{
	struct i2c_client *client = NULL;
	struct fimc_is_vender_specific *specific = core->vender.private_data;
	u32 sensor_idx = specific->iris_sensor_index;

	if (core->sensor[sensor_idx].iris != NULL)
		client = core->sensor[sensor_idx].iris->client;

	return client;
};

int sensor_ak7372_aperture_init(struct v4l2_subdev *subdev, u32 val)
{
	int ret = 0;
	int pos = 0;
	u8 data[2] = {0, };
	struct fimc_is_iris *iris;
	struct i2c_client *client = NULL;
#ifdef USE_CAMERA_HW_BIG_DATA
	struct cam_hw_param *hw_param = NULL;
	struct fimc_is_device_sensor *device = NULL;
#endif

	WARN_ON(!subdev);

	dbg_iris("%s\n", __func__);

	iris = (struct fimc_is_iris *)v4l2_get_subdevdata(subdev);
	WARN_ON(!iris);

	client = iris->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto p_err;
	}

	I2C_MUTEX_LOCK(iris->i2c_lock);

	iris->cur_value = F2_4;
	iris->new_value = F2_4;

	ret = fimc_is_sensor_addr8_write8(client, 0xAE, 0x3B); /* setting mode */
	pos = 511;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	ret |= fimc_is_sensor_addr8_write8(client, 0x02, 0x00); /* active mode */
	ret |= fimc_is_sensor_addr8_write8(client, 0xA6, 0x7B); /* open mode */
	pos = 1023;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	msleep(30);
	ret |= fimc_is_sensor_addr8_write8(client, 0xA6, 0x00); /* close code */
	pos = 511;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	usleep_range(10000, 11000);

	if (ret < 0) {
#ifdef USE_CAMERA_HW_BIG_DATA
		device = v4l2_get_subdev_hostdata(subdev);
		if (device)
			fimc_is_sec_get_hw_param(&hw_param, device->position);
		if (hw_param)
			hw_param->i2c_iris_err_cnt++;
#endif
		goto p_err;
	}

p_err:
	I2C_MUTEX_UNLOCK(iris->i2c_lock);

	return ret;
}

int sensor_ak7372_aperture_set_value(struct v4l2_subdev *subdev, int value)
{
	int ret = 0;
	int pos1 = 0, pos2 = 0;
	u8 data[2] = {0, };
	struct fimc_is_iris *iris;
	struct i2c_client *client = NULL;

	WARN_ON(!subdev);

	dbg_iris("%s value = %d\n", __func__, value);

	iris = (struct fimc_is_iris *)v4l2_get_subdevdata(subdev);
	WARN_ON(!iris);

	client = iris->client;
	if (unlikely(!client)) {
		err("client is NULL");
		ret = -EINVAL;
		goto exit;
	}

	if (value == iris->cur_value)
		return ret;

	switch (value) {
	case F1_5:
		pos1 = 0;
		pos2 = 511;
		break;
	case F2_4:
		pos1 = 1023;
		pos2 = 511;
		break;
	default:
		dbg_iris("%s: mode is not set.(mode = %d)\n", __func__, value);
		goto exit;
	}

	I2C_MUTEX_LOCK(iris->i2c_lock);

	ret = fimc_is_sensor_addr8_write8(client, 0xA6, 0x7B); /* open mode */
	data[0] = pos1 >> 2 & 0xFF;
	data[1] = pos1 << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	msleep(30);
	ret |= fimc_is_sensor_addr8_write8(client, 0xA6, 0x00); /* close code */
	data[0] = pos2 >> 2 & 0xFF;
	data[1] = pos2 << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	usleep_range(10000, 11000);

	if (ret < 0) {
		err("i2c fail occurred.");
	}

	iris->cur_value = value;

	I2C_MUTEX_UNLOCK(iris->i2c_lock);

exit:
	return ret;
}

int sensor_ak7372_aperture_check_value(struct v4l2_subdev *subdev, int value)
{
	struct fimc_is_iris *iris;

	WARN_ON(!subdev);

	dbg_iris("%s value = %d\n", __func__, value);

	iris = (struct fimc_is_iris *)v4l2_get_subdevdata(subdev);
	WARN_ON(!iris);

	if (value != iris->cur_value)
		return true;
	else
		return false;
}

int sensor_ak7372_iris_init_temp(struct fimc_is_core *core)
{
	int ret = 0;
	int pos = 0;
	u8 data[2] = {0, };
	struct i2c_client *client = fimc_is_aperture_i2c_get_client(core);
#ifdef USE_CAMERA_HW_BIG_DATA
	struct cam_hw_param *hw_param = NULL;
	struct fimc_is_device_sensor *device = NULL;
#endif

	ret = fimc_is_sensor_addr8_write8(client, 0xAE, 0x3B); /* setting mode */
	pos = 511;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	ret |= fimc_is_sensor_addr8_write8(client, 0x02, 0x00); /* active mode */
	ret |= fimc_is_sensor_addr8_write8(client, 0xA6, 0x7B); /* open mode */
	pos = 1023;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	msleep(30);
	ret |= fimc_is_sensor_addr8_write8(client, 0xA6, 0x00); /* close code */
	pos = 511;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	usleep_range(10000, 11000);

	if (ret < 0) {
		err("i2c fail occurred.");
		goto p_err;
	}

p_err:
	return ret;
}

int sensor_ak7372_iris_high_temp(struct fimc_is_core *core)
{
	int ret = 0;
	int pos = 0;
	u8 data[2] = {0, };
	struct i2c_client *client = fimc_is_aperture_i2c_get_client(core);

	ret = fimc_is_sensor_addr8_write8(client, 0xA6, 0x7B); /* open mode */
	pos = 0;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	msleep(30);
	ret |= fimc_is_sensor_addr8_write8(client, 0xA6, 0x00); /* close code */
	pos = 511;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	usleep_range(10000, 11000);

	if (ret < 0) {
		err("i2c fail occurred.");
		goto p_err;
	}

p_err:
	return ret;
}

int sensor_ak7372_iris_low_temp(struct fimc_is_core *core)
{
	int ret = 0;
	int pos = 0;
	u8 data[2] = {0, };
	struct i2c_client *client = fimc_is_aperture_i2c_get_client(core);

	ret = fimc_is_sensor_addr8_write8(client, 0xA6, 0x7B); /* open mode */
	pos = 1023;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	msleep(30);
	ret |= fimc_is_sensor_addr8_write8(client, 0xA6, 0x00); /* close code */
	pos = 511;
	data[0] = pos >> 2 & 0xFF;
	data[1] = pos << 6 & 0xC0;
	ret |= fimc_is_sensor_addr8_write8(client, 0x00, data[0]); /* position code */
	ret |= fimc_is_sensor_addr8_write8(client, 0x01, data[1]);
	usleep_range(10000, 11000);

	if (ret < 0) {
		err("i2c fail occurred.");
		goto p_err;
	}

p_err:
	return ret;
}

static struct fimc_is_iris_ops iris_ops = {
	.set_aperture_value = sensor_ak7372_aperture_set_value,
	.check_aperture_value = sensor_ak7372_aperture_check_value,
};

static const struct v4l2_subdev_core_ops core_ops = {
	.init = sensor_ak7372_aperture_init,
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
};

int sensor_ak7372_iris_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct fimc_is_core *core = NULL;
	struct v4l2_subdev *subdev_iris = NULL;
	struct fimc_is_device_sensor *device;
	struct fimc_is_iris *iris = NULL;
	struct device *dev;
	struct device_node *dnode;
	u32 sensor_id[FIMC_IS_SENSOR_COUNT] = {0, };
	const u32 *sensor_id_spec;
	u32 sensor_id_len;
	int i = 0;

	WARN_ON(!fimc_is_dev);
	WARN_ON(!client);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core device is not yet probed");
		ret = -EPROBE_DEFER;
		goto p_err;
	}

	dev = &client->dev;
	dnode = dev->of_node;

	sensor_id_spec = of_get_property(dnode, "id", &sensor_id_len);
	if (!sensor_id_spec) {
		err("sensor_id num read is fail(%d)", ret);
		goto p_err;
	}

	sensor_id_len /= sizeof(*sensor_id_spec);

	ret = of_property_read_u32_array(dnode, "id", sensor_id, sensor_id_len);
	if (ret) {
		err("sensor_id read is fail(%d)", ret);
		goto p_err;
	}

	for (i = 0; i < sensor_id_len; i++) {
		device = &core->sensor[sensor_id[i]];
		if (!device) {
			err("sensor device is NULL");
			ret = -EPROBE_DEFER;
			goto p_err;
		}

		iris = kzalloc(sizeof(struct fimc_is_iris), GFP_KERNEL);
		if (!iris) {
			err("iris is NULL");
			ret = -ENOMEM;
			goto p_err;
		}

		subdev_iris = kzalloc(sizeof(struct v4l2_subdev), GFP_KERNEL);
		if (!subdev_iris) {
			err("subdev_iris is NULL");
			ret = -ENOMEM;
			kfree(iris);
			goto p_err;
		}

		iris->id = IRIS_NAME_AK7372;
		iris->subdev = subdev_iris;
		iris->device = sensor_id[i];
		iris->client = client;
		iris->i2c_lock = NULL;
		iris->iris_ops = &iris_ops;
		core->client5 = client;

		device->subdev_iris = subdev_iris;
		device->iris = iris;

		v4l2_i2c_subdev_init(subdev_iris, client, &subdev_ops);
		v4l2_set_subdevdata(subdev_iris, iris);
		v4l2_set_subdev_hostdata(subdev_iris, device);
	}

p_err:
	probe_info("%s done\n", __func__);
	return ret;
}

static int sensor_ak7372_iris_remove(struct i2c_client *client)
{
	int ret = 0;

	return ret;
}

static const struct of_device_id exynos_fimc_is_iris_ak7372_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-iris-ak7372",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_iris_ak7372_match);

static const struct i2c_device_id iris_ak7372_idt[] = {
	{ IRIS_NAME, 0 },
	{},
};

static struct i2c_driver iris_ak7372_driver = {
	.driver = {
		.name	= IRIS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = exynos_fimc_is_iris_ak7372_match
	},
	.probe	= sensor_ak7372_iris_probe,
	.remove	= sensor_ak7372_iris_remove,
	.id_table = iris_ak7372_idt
};
module_i2c_driver(iris_ak7372_driver);
