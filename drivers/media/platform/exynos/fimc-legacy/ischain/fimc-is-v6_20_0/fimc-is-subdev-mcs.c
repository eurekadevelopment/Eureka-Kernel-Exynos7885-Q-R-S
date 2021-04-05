/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is video functions
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "fimc-is-device-ischain.h"
#include "fimc-is-subdev-ctrl.h"
#include "fimc-is-config.h"
#include "fimc-is-param.h"
#include "fimc-is-video.h"
#include "fimc-is-type.h"

static int fimc_is_ischain_mcs_bypass(struct fimc_is_subdev *leader,
	void *device_data,
	struct fimc_is_frame *frame,
	bool bypass)
{
	int ret = 0;
	struct fimc_is_group *group;
	struct param_mcs_output *output;
	struct fimc_is_device_ischain *device;
	struct fimc_is_subdev *subdev = NULL, *subdev_temp = NULL, *temp;
	u32 lindex = 0, hindex = 0, indexes = 0;

	device = (struct fimc_is_device_ischain *)device_data;

	BUG_ON(!leader);
	BUG_ON(!device);

	group = &device->group_mcs;
	
	list_for_each_entry_safe(subdev_temp, temp, &group->subdev_list, list) {
		if (subdev_temp == &group->leader)
			continue;

		if (test_bit(FIMC_IS_SUBDEV_START, &subdev_temp->state)) {
			subdev = subdev_temp;
			goto find_subdev;
		}
	}

find_subdev:
	if (!subdev) {
		mgerr("subdev is NULL\n", device, group);
		goto p_err;
	}

	output = fimc_is_itf_g_param(device, frame, subdev->param_dma_ot);

	if (bypass)
		output->otf_cmd = OTF_OUTPUT_COMMAND_DISABLE;
	else
		output->otf_cmd = OTF_OUTPUT_COMMAND_ENABLE;

	lindex |= LOWBIT_OF(subdev->param_dma_ot);
	hindex |= HIGHBIT_OF(subdev->param_dma_ot);
	indexes++;

	ret = fimc_is_itf_s_param(device, frame, lindex, hindex, indexes);
	if (ret) {
		mrerr("fimc_is_itf_s_param is fail(%d)", device, frame, ret);
		goto p_err;
	}

	minfo("VRA is connected at %s\n", device, group->junction->name);

p_err:
	return ret;
}

static int fimc_is_ischain_mcs_cfg(struct fimc_is_subdev *leader,
	void *device_data,
	struct fimc_is_frame *frame,
	struct fimc_is_crop *incrop,
	struct fimc_is_crop *otcrop,
	u32 *lindex,
	u32 *hindex,
	u32 *indexes)
{
	int ret = 0;
	struct fimc_is_fmt *format;
	struct fimc_is_group *group;
	struct fimc_is_queue *queue;
	struct param_mcs_input *input;
	struct param_control *control;
	struct fimc_is_device_ischain *device;
	struct fimc_is_subdev *subdev = NULL, *subdev_temp = NULL, *temp;

	device = (struct fimc_is_device_ischain *)device_data;

	BUG_ON(!leader);
	BUG_ON(!device);
	BUG_ON(!device->sensor);
	BUG_ON(!incrop);
	BUG_ON(!lindex);
	BUG_ON(!hindex);
	BUG_ON(!indexes);

	group = &device->group_mcs;
	queue = GET_SUBDEV_QUEUE(leader);
	if (!queue) {
		merr("queue is NULL", device);
		ret = -EINVAL;
		goto p_err;
	}

	format = queue->framecfg.format;

	input = fimc_is_itf_g_param(device, frame, PARAM_MCS_INPUT);
	if (test_bit(FIMC_IS_GROUP_OTF_INPUT, &group->state)) {
		input->otf_cmd = OTF_INPUT_COMMAND_ENABLE;
		input->dma_cmd = DMA_INPUT_COMMAND_DISABLE;
		input->width = incrop->w;
		input->height = incrop->h;
	} else {
		input->otf_cmd = OTF_INPUT_COMMAND_DISABLE;
		input->dma_cmd = DMA_INPUT_COMMAND_ENABLE;
		input->width = leader->input.canv.w;
		input->height = leader->input.canv.h;
		input->dma_crop_offset_x = leader->input.canv.x + incrop->x;
		input->dma_crop_offset_y = leader->input.canv.y + incrop->y;
		input->dma_crop_width = incrop->w;
		input->dma_crop_height = incrop->h;
	}

	input->otf_format = OTF_INPUT_FORMAT_YUV422;
	input->otf_bitwidth = OTF_INPUT_BIT_WIDTH_8BIT;
	input->otf_order = OTF_INPUT_ORDER_BAYER_GR_BG;
	input->dma_format = DMA_INPUT_FORMAT_YUV422;
	input->dma_bitwidth = DMA_INPUT_BIT_WIDTH_8BIT;
	input->dma_order = DMA_INPUT_ORDER_YCbYCr;
	input->plane = DMA_INPUT_PLANE_1;

#if !defined(ENABLE_IS_CORE)
	/*
	 * HW spec: DMA stride should be aligned by 16 byte.
	 */
	if (!test_bit(FIMC_IS_GROUP_OTF_INPUT, &group->state)) {
		input->dma_stride_y = ALIGN(max(input->dma_crop_width * format->bitsperpixel[0] / BITS_PER_BYTE,
					queue->framecfg.bytesperline[0]), 16);
		input->dma_stride_c = ALIGN(max(input->dma_crop_width * format->bitsperpixel[1] / BITS_PER_BYTE,
					queue->framecfg.bytesperline[1]), 16);
	}
#endif

	*lindex |= LOWBIT_OF(PARAM_MCS_INPUT);
	*hindex |= HIGHBIT_OF(PARAM_MCS_INPUT);
	(*indexes)++;

	/* Configure Control */
	control = fimc_is_itf_g_param(device, frame, PARAM_MCS_CONTROL);
	control->cmd = CONTROL_COMMAND_START;
#ifdef ENABLE_DNR_IN_MCSC
	control->buffer_address = device->minfo->dvaddr_mcsc_dnr;
#endif
	*lindex |= LOWBIT_OF(PARAM_MCS_CONTROL);
	*hindex |= HIGHBIT_OF(PARAM_MCS_CONTROL);
	(*indexes)++;

	leader->input.crop = *incrop;

	if (test_bit(FIMC_IS_ISCHAIN_REPROCESSING, &device->state)) {
		mswarn("reprocessing cannot connect to VRA\n", device, leader);
		goto p_err;
	}


	list_for_each_entry_safe(subdev_temp, temp, &group->subdev_list, list) {
		if (subdev_temp == &group->leader)
			continue;

		if (test_bit(FIMC_IS_SUBDEV_START, &subdev_temp->state)) {
			subdev = subdev_temp;
			goto find_subdev;
		}
	}

find_subdev:
	if (!subdev) {
		mgerr("subdev is NULL\n", device, group);
		goto p_err;
	}

	CALL_SOPS(subdev, cfg, device, frame, incrop, NULL, lindex, hindex, indexes);

p_err:
	return ret;
}

static int fimc_is_ischain_mcs_tag(struct fimc_is_subdev *subdev,
	void *device_data,
	struct fimc_is_frame *frame,
	struct camera2_node *node)
{
	int ret = 0;
	struct fimc_is_group *group;
	struct mcs_param *mcs_param;
	struct fimc_is_crop inparm;
	struct fimc_is_crop *incrop, *otcrop;
	struct fimc_is_subdev *leader;
	struct fimc_is_device_ischain *device;
	u32 lindex, hindex, indexes;

	device = (struct fimc_is_device_ischain *)device_data;

	BUG_ON(!subdev);
	BUG_ON(!device);
	BUG_ON(!device->is_region);
	BUG_ON(!frame);

	mdbgs_ischain(4, "MCSC TAG\n", device);

	incrop = (struct fimc_is_crop *)node->input.cropRegion;
	otcrop = (struct fimc_is_crop *)node->output.cropRegion;

	group = &device->group_mcs;
	leader = subdev->leader;
	lindex = hindex = indexes = 0;
	mcs_param = &device->is_region->parameter.mcs;

	if (test_bit(FIMC_IS_GROUP_OTF_INPUT, &group->state)) {
		inparm.x = 0;
		inparm.y = 0;
		inparm.w = mcs_param->input.width;
		inparm.h = mcs_param->input.height;
	} else {
		inparm.x = mcs_param->input.dma_crop_offset_x;
		inparm.y = mcs_param->input.dma_crop_offset_y;
		inparm.w = mcs_param->input.dma_crop_width;
		inparm.h = mcs_param->input.dma_crop_height;
	}

	if (IS_NULL_CROP(incrop))
		*incrop = inparm;

	if (!COMPARE_CROP(incrop, &inparm) ||
		test_bit(FIMC_IS_SUBDEV_FORCE_SET, &leader->state)) {
		ret = fimc_is_ischain_mcs_cfg(subdev,
			device,
			frame,
			incrop,
			otcrop,
			&lindex,
			&hindex,
			&indexes);
		if (ret) {
			merr("fimc_is_ischain_mcs_start is fail(%d)", device, ret);
			goto p_err;
		}

		msrinfo("in_crop[%d, %d, %d, %d]\n", device, subdev, frame,
			incrop->x, incrop->y, incrop->w, incrop->h);
	}

	ret = fimc_is_itf_s_param(device, frame, lindex, hindex, indexes);
	if (ret) {
		mrerr("fimc_is_itf_s_param is fail(%d)", device, frame, ret);
		goto p_err;
	}

p_err:
	return ret;
}

const struct fimc_is_subdev_ops fimc_is_subdev_mcs_ops = {
	.bypass			= fimc_is_ischain_mcs_bypass,
	.cfg			= fimc_is_ischain_mcs_cfg,
	.tag			= fimc_is_ischain_mcs_tag,
};
