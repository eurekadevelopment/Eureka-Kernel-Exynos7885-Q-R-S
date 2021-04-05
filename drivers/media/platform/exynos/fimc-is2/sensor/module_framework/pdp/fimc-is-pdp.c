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

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/of.h>

#include <media/v4l2-subdev.h>

#include "fimc-is-config.h"
#include "fimc-is-device-sensor-peri.h"
#include "fimc-is-pdp.h"
#include "fimc-is-hw-pdp.h"

static int debug_pdp;
module_param(debug_pdp, int, 0644);

static struct fimc_is_pdp pdp_device[MAX_NUM_OF_PDP];

int pdp_register(struct fimc_is_module_enum *module, int pdp_ch)
{
	int ret = 0;
	struct fimc_is_pdp *pdp;
	struct fimc_is_device_sensor_peri *sensor_peri = module->private_data;

	if (test_bit(FIMC_IS_SENSOR_PDP_AVAILABLE, &sensor_peri->peri_state)) {
		err("already registered");
		ret = -EINVAL;
		goto p_err;
	}

	if (pdp_ch >= MAX_NUM_OF_PDP) {
		err("A pdp channel is invalide");
		ret = -EINVAL;
		goto p_err;
	}

	pdp = &pdp_device[pdp_ch];
	sensor_peri->pdp = pdp;
	sensor_peri->subdev_pdp = pdp->subdev;
	set_bit(FIMC_IS_SENSOR_PDP_AVAILABLE, &sensor_peri->peri_state);
	v4l2_set_subdev_hostdata(pdp->subdev, module);

	info("[PDP:%d] %s(ret:%d)\n", pdp->id, __func__, ret);
	return ret;
p_err:
	return ret;
}

int pdp_unregister(struct fimc_is_module_enum *module)
{
	int ret = 0;
	struct fimc_is_device_sensor_peri *sensor_peri = module->private_data;
	struct fimc_is_pdp *pdp;

	if (!test_bit(FIMC_IS_SENSOR_PDP_AVAILABLE, &sensor_peri->peri_state)) {
		err("already unregistered");
		ret = -EINVAL;
		goto p_err;
	}

	pdp = (struct fimc_is_pdp *)v4l2_get_subdevdata(sensor_peri->subdev_pdp);
	if (!pdp) {
		err("A subdev data of PDP is null");
		ret = -ENODEV;
		goto p_err;
	}

	sensor_peri->pdp = NULL;
	sensor_peri->subdev_pdp = NULL;
	clear_bit(FIMC_IS_SENSOR_PDP_AVAILABLE, &sensor_peri->peri_state);

	info("[PDP:%d] %s(ret:%d)\n", pdp->id, __func__, ret);
	return ret;
p_err:
	return ret;
}

int pdp_init(struct v4l2_subdev *subdev, u32 val)
{
	return 0;
}

static int pdp_s_stream(struct v4l2_subdev *subdev, int pd_mode)
{
	int ret = 0;
	struct fimc_is_pdp *pdp;

	pdp = (struct fimc_is_pdp *)v4l2_get_subdevdata(subdev);
	if (!pdp) {
		err("A subdev data of PDP is null");
		ret = -ENODEV;
		goto p_err;
	}

	pdp_hw_enable(pdp->base_reg, pd_mode);

	if (debug_pdp)
		pdp_hw_dump(pdp->base_reg);

	info("[PDP:%d] %s(mode: %d)(ret:%d)\n", pdp->id, __func__, pd_mode, ret);
	return ret;
p_err:
	return ret;
}

static int pdp_s_param(struct v4l2_subdev *subdev, struct v4l2_streamparm *param)
{
	return 0;
}

static int pdp_s_format(struct v4l2_subdev *subdev,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *fmt)
{
	int ret = 0;
	size_t width, height;
	struct fimc_is_pdp *pdp;
	struct fimc_is_module_enum *module;

	pdp = (struct fimc_is_pdp *)v4l2_get_subdevdata(subdev);
	if (!pdp) {
		err("A subdev data of PDP is null");
		ret = -ENODEV;
		goto p_err;
	}

	width = fmt->format.width;
	height = fmt->format.height;
	pdp->width = width;
	pdp->height = height;

	module = (struct fimc_is_module_enum *)v4l2_get_subdev_hostdata(subdev);
	if (!module) {
		err("[PDP:%d] A host data of PDP is null", pdp->id);
		ret = -ENODEV;
		goto p_err;
	}

	info("[PDP:%d] %s(size:%lu X %lu)(ret:%d)\n", pdp->id, __func__, width, height, ret);
	return ret;
p_err:
	return ret;
}

static const struct v4l2_subdev_core_ops core_ops = {
	.init = pdp_init
};

static const struct v4l2_subdev_video_ops video_ops = {
	.s_stream = pdp_s_stream,
	.s_parm = pdp_s_param,
};

static const struct v4l2_subdev_pad_ops pad_ops = {
	.set_fmt = pdp_s_format
};

static const struct v4l2_subdev_ops subdev_ops = {
	.core = &core_ops,
	.video = &video_ops,
	.pad = &pad_ops
};

static int pdp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *mem_res;
	struct fimc_is_pdp *pdp;
	struct device_node *dnode;
	struct device *dev;

	BUG_ON(!pdev);
	BUG_ON(!pdev->dev.of_node);

	dev = &pdev->dev;
	dnode = dev->of_node;

	ret = of_property_read_u32(dnode, "id", &pdev->id);
	if (ret) {
		dev_err(dev, "id read is fail(%d)\n", ret);
		goto err_get_id;
	}

	pdp = &pdp_device[pdev->id];

	/* Get SFR base register */
	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		dev_err(dev, "Failed to get io memory region(%p)\n", mem_res);
		ret = -EBUSY;
		goto err_get_resource;
	}

	pdp->regs_start = mem_res->start;
	pdp->regs_end = mem_res->end;
	pdp->base_reg =  devm_ioremap_nocache(&pdev->dev, mem_res->start, resource_size(mem_res));
	if (!pdp->base_reg) {
		dev_err(dev, "Failed to remap io region(%p)\n", pdp->base_reg);
		ret = -ENOMEM;
		goto err_ioremap;
	}

	pdp->id = pdev->id;
	platform_set_drvdata(pdev, pdp);

	pdp->subdev = devm_kzalloc(&pdev->dev, sizeof(struct v4l2_subdev), GFP_KERNEL);
	if (!pdp->subdev) {
		ret = -ENOMEM;
		goto err_subdev_alloc;
	}

	v4l2_subdev_init(pdp->subdev, &subdev_ops);

	v4l2_set_subdevdata(pdp->subdev, pdp);
	snprintf(pdp->subdev->name, V4L2_SUBDEV_NAME_SIZE, "pdp-subdev.%d", pdp->id);

	probe_info("%s(%s)\n", __func__, dev_name(&pdev->dev));

	return ret;

err_subdev_alloc:
err_ioremap:
err_get_resource:
err_get_id:
	return ret;
}

static int pdp_remove(struct platform_device *pdev)
{
	int ret = 0;
	return ret;
}

static const struct of_device_id exynos_fimc_is_pdp_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is-pdp",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_pdp_match);

static struct platform_driver pdp_driver = {
	.probe  = pdp_probe,
	.remove = pdp_remove,
	.driver = {
		.name   = "FIMC-IS-PDP",
		.owner  = THIS_MODULE,
		.of_match_table = exynos_fimc_is_pdp_match,
	}
};
module_platform_driver(pdp_driver);
