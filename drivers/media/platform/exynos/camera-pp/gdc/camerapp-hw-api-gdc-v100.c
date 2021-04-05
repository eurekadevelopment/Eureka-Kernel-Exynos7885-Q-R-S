/*
 * Samsung EXYNOS CAMERA PostProcessing GDC driver
 *
 * Copyright (C) 2016 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "camerapp-hw-api-gdc.h"

#if defined CONFIG_CAMERA_PP_GDC_V1_0_0_OBJ
#include "camerapp-hw-reg-gdc-v100.h"
#elif defined CONFIG_CAMERA_PP_GDC_V1_1_0_OBJ
#include "camerapp-hw-reg-gdc-v110.h"
#endif


void camerapp_hw_gdc_start(void __iomem *base_addr)
{
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_FRAMESTART_CMD], &gdc_fields[GDC_F_FRAMESTART_CMD], 1);
}

void camerapp_hw_gdc_stop(void __iomem *base_addr)
{
	/* camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_FRAMESTART_CMD], &gdc_fields[GDC_F_FRAMESTART_CMD], 0); */
}

u32 camerapp_hw_gdc_sw_reset(void __iomem *base_addr)
{
	u32 reset_count = 0;

	/* request to stop bus IF */
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_REQ_STOP], &gdc_fields[GDC_F_EN_RESET], 1);

	/* wait bus IF reset complete */
	do {
		reset_count++;
		if (reset_count > 10000)
			return reset_count;
	} while (camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_REQ_STOP], &gdc_fields[GDC_F_RESET_OK]) != 1);

	/* request to gdc hw */
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_SW_RESET], &gdc_fields[GDC_F_SW_RESET], 1);

	/* wait reset complete */
	do {
		reset_count++;
		if (reset_count > 10000)
			return reset_count;
	} while (camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_GDC_SW_RESET], &gdc_fields[GDC_F_SW_RESET]) != 0);

	return 0;
}

void camerapp_hw_gdc_clear_intr_all(void __iomem *base_addr)
{
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_GDC_INT_STATUS], 0x0001);
}

void camerapp_hw_gdc_mask_intr(void __iomem *base_addr, u32 intr_mask)
{
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_GDC_INT_MASK], intr_mask);
}

u32 camerapp_hw_gdc_get_intr_status(void __iomem *base_addr)
{
	u32 intStatus;

	intStatus = camerapp_sfr_get_reg(base_addr, &gdc_regs[GDC_R_GDC_INT_STATUS]);

	return intStatus;
}

u32 camerapp_hw_gdc_get_intr_status_and_clear(void __iomem *base_addr)
{
	u32 intStatus;

	intStatus = camerapp_sfr_get_reg(base_addr, &gdc_regs[GDC_R_GDC_INT_STATUS]);
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_GDC_INT_STATUS], intStatus);

	return intStatus;
}

void camerapp_hw_gdc_update_grid_param(void __iomem *base_addr, struct gdc_grid_param *grid_param)
{
	u32 i = 0, j = 0;
	u32 sfr_offset = 0x0004;
	u32 sfr_start_x = 0x0100;
	u32 sfr_start_y = 0x0200;

	if (grid_param->is_valid == true) {
		for (i = 0; i < 7; i++) {
			for (j = 0; j < 9; j++) {
                u32 cal_sfr_offset = (sfr_offset * i * 9) + (sfr_offset * j);
				writel((u32)grid_param->dx[i][j], base_addr + sfr_start_x + cal_sfr_offset);
				writel((u32)grid_param->dy[i][j], base_addr + sfr_start_y + cal_sfr_offset);
			}
		}
	}
}

void camerapp_hw_gdc_update_scale_parameters(void __iomem *base_addr, struct gdc_frame *s_frame,
						struct gdc_frame *d_frame, struct gdc_crop_param *crop_param)
{
	u32 gdc_input_width;
	u32 gdc_input_height;
	u32 gdc_crop_width;
	u32 gdc_crop_height;
	u32 gdc_input_width_start;
	u32 gdc_input_height_start;
	u32 gdc_scale_shifter_x;
	u32 gdc_scale_shifter_y;
	u32 gdc_scale_x;
	u32 gdc_scale_y;
	u32 gdc_inv_scale_x;
	u32 gdc_inv_scale_y;
	u32 gdc_output_width;
	u32 gdc_output_height;
	u32 gdc_output_width_start;
	u32 gdc_output_height_start;
	u32 scaleShifterX;
	u32 scaleShifterY;
	u32 imagewidth;
	u32 imageheight;
	u32 out_scaled_width;
	u32 out_scaled_height;

	/* grid size setting : assume no crop */
	gdc_input_width = s_frame->width;
	gdc_input_height = s_frame->height;
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_CONFIG], &gdc_fields[GDC_F_GDC_MIRROR_X], 0);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_CONFIG], &gdc_fields[GDC_F_GDC_MIRROR_Y], 0);

	/* Need to change : Crop_size */
	gdc_crop_width = gdc_input_width;
	gdc_crop_height = gdc_input_height;

	gdc_inv_scale_x = gdc_input_width;
	gdc_inv_scale_y = ((gdc_input_height << 3) + 3) / 6;
	gdc_input_width_start = 0;
	gdc_input_height_start = 0;

	scaleShifterX  = DS_SHIFTER_MAX;
	imagewidth = gdc_input_width << 1;
	while ((imagewidth <= MAX_VIRTUAL_GRID_X) && (scaleShifterX > 0)) {
		imagewidth <<= 1;
		scaleShifterX--;
	}
	gdc_scale_shifter_x = scaleShifterX;

	scaleShifterY  = DS_SHIFTER_MAX;
	imageheight = gdc_input_height << 1;
	while ((imageheight <= MAX_VIRTUAL_GRID_Y) && (scaleShifterY > 0)) {
		imageheight <<= 1;
		scaleShifterY--;
	}
	gdc_scale_shifter_y = scaleShifterY;

	gdc_scale_x = MIN(65535,
		((MAX_VIRTUAL_GRID_X << (DS_FRACTION_BITS + scaleShifterX)) + gdc_input_width / 2) / gdc_input_width);
	gdc_scale_y = MIN(65535,
		((MAX_VIRTUAL_GRID_Y << (DS_FRACTION_BITS + scaleShifterY)) + gdc_input_height / 2) / gdc_input_height);

	gdc_output_width = d_frame->width;
	gdc_output_height = d_frame->height;
	if ((gdc_output_width & 0x1) || (gdc_output_height & 0x1)) {
		gdc_dbg("gdc output size is not even.\n");
		gdc_output_width = ALIGN(gdc_output_width, 2);
		gdc_output_height = ALIGN(gdc_output_height, 2);
	}

	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INPUT_ORG_SIZE],
		&gdc_fields[GDC_F_GDC_ORG_HEIGHT], gdc_input_height);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INPUT_ORG_SIZE],
		&gdc_fields[GDC_F_GDC_ORG_WIDTH], gdc_input_width);

	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INPUT_CROP_START],
		&gdc_fields[GDC_F_GDC_CROP_START_X], gdc_input_width_start);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INPUT_CROP_START],
		&gdc_fields[GDC_F_GDC_CROP_START_Y], gdc_input_height_start);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INPUT_CROP_SIZE],
		&gdc_fields[GDC_F_GDC_CROP_WIDTH], gdc_crop_width);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INPUT_CROP_SIZE],
		&gdc_fields[GDC_F_GDC_CROP_HEIGHT], gdc_crop_height);

	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_SCALE],
		&gdc_fields[GDC_F_GDC_SCALE_X], gdc_scale_x);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_SCALE],
		&gdc_fields[GDC_F_GDC_SCALE_Y], gdc_scale_y);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_SCALE_SHIFTER],
		&gdc_fields[GDC_F_GDC_SCALE_SHIFTER_X], gdc_scale_shifter_x);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_SCALE_SHIFTER],
		&gdc_fields[GDC_F_GDC_SCALE_SHIFTER_Y], gdc_scale_shifter_y);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INV_SCALE],
		&gdc_fields[GDC_F_GDC_INV_SCALE_X], gdc_inv_scale_x);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INV_SCALE],
		&gdc_fields[GDC_F_GDC_INV_SCALE_Y], gdc_inv_scale_y);

	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_OUT_CROP_SIZE],
		&gdc_fields[GDC_F_GDC_IMAGE_ACTIVE_WIDTH], gdc_output_width);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_OUT_CROP_SIZE],
		&gdc_fields[GDC_F_GDC_IMAGE_ACTIVE_HEIGHT], gdc_output_height);
	/* Meaningful only when out_crop_bypass = 0, x <= (gdc_crop_width - gdc_image_active_width) */
	gdc_output_width_start = ALIGN((gdc_crop_width - gdc_output_width) >> 1, 2);
	gdc_output_height_start = ALIGN((gdc_crop_height - gdc_output_height) >> 1, 2);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_OUT_CROP_START],
		&gdc_fields[GDC_F_GDC_IMAGE_CROP_PRE_X], gdc_output_width_start);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_OUT_CROP_START],
		&gdc_fields[GDC_F_GDC_IMAGE_CROP_PRE_Y], gdc_output_height_start);

	/* if GDC is scaled up : 128(default) = no scaling, 64 = 2 times scaling */
	/* now is selected no scaling. => calcuration (128 * in / out) */
	if (gdc_crop_width < gdc_output_width)	/* only for scale up */
		out_scaled_width = 128 * gdc_input_width / gdc_output_width;
	else									/* default value */
		out_scaled_width = 128;

	if (gdc_crop_height < gdc_output_height)
		out_scaled_height = 128 * gdc_input_height / gdc_output_height;
	else
		out_scaled_height = 128;

	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_OUT_SCALE],
		&gdc_fields[GDC_F_GDC_OUT_SCALE_Y], out_scaled_height);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_OUT_SCALE],
		&gdc_fields[GDC_F_GDC_OUT_SCALE_X], out_scaled_width);

	gdc_dbg("gdc in(%dx%d) crop(%dx%d) -> out(%dx%d)\n",
		gdc_input_width, gdc_input_height, gdc_crop_width, gdc_crop_height,
		gdc_output_width, gdc_output_height);
}

void camerapp_hw_gdc_update_dma_size(void __iomem *base_addr, struct gdc_frame *s_frame, struct gdc_frame *d_frame)
{
	u32 input_stride_lum_w, input_stride_chroma_w;
	u32 output_stride_lum_w, output_stride_chroma_w;
	u32 in_dma_width, out_dma_width;

	/*
	 * supported format
	 * - YUV422_2P_8bit/10bit
	 * - YUV420_2P_8bit/10bit
	 */
	out_dma_width = camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_GDC_OUT_CROP_SIZE],
		&gdc_fields[GDC_F_GDC_IMAGE_ACTIVE_WIDTH]);
	in_dma_width = camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_GDC_INPUT_ORG_SIZE],
		&gdc_fields[GDC_F_GDC_ORG_WIDTH]);

	/* dma size : $%16 == 0 */
	/* same as lum, chroma value : 420 bit only */
	input_stride_lum_w = (u32)(((in_dma_width + 15) / 16) * 16);
	input_stride_chroma_w = (u32)(((in_dma_width + 15) / 16) * 16);

	output_stride_lum_w = (u32)(((out_dma_width + 15) / 16) * 16);
	output_stride_chroma_w = (u32)(((out_dma_width + 15) / 16) * 16);

	gdc_dbg("s_w = %d, lum stride_w = %d, chroma stride_w = %d\n",
		s_frame->width, input_stride_lum_w, input_stride_chroma_w);
	gdc_dbg("d_w = %d, lum stride_w = %d, chroma stride_w = %d\n",
		d_frame->width, output_stride_lum_w, output_stride_chroma_w);

	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_PXC_STRIDE_LUM], input_stride_lum_w);		/* RDMA */
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_PXC_STRIDE_CHROMA], input_stride_chroma_w);
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_WAXI_STRIDE_LUM], output_stride_lum_w);		/* WDMA */
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_WAXI_STRIDE_CHROMA], output_stride_chroma_w);
}

void camerapp_hw_gdc_set_dma_address(void __iomem *base_addr, struct gdc_frame *s_frame, struct gdc_frame *d_frame)
{
	/* buffer address setting : must be [1:0] == 0 */
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_GDC_DMA_ADDRESS], d_frame->addr.y);
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_GDC_DMA_ADDRESS_2], d_frame->addr.cb);

	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_PXC_DPB_BASE_LUM], s_frame->addr.y);
	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_PXC_DPB_BASE_CHROMA], s_frame->addr.cb);
}

#if defined CONFIG_CAMERA_PP_GDC_V1_1_0_OBJ
void camerapp_hw_gdc_set_format(void __iomem *base_addr, struct gdc_frame *s_frame, struct gdc_frame *d_frame)
{
	u32 input_pix_format, output_pix_format;
	u32 input_yuv_format, output_yuv_format;

	/*PIXEL_FORMAT		0: NV12 (2plane Y/UV order), 1: NV21 (2plane Y/VU order) */
	/* YUV format: 0 - YUV422, 1 - YUV420 */

	/* input */
	switch (s_frame->gdc_fmt->pixelformat) {
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV21:
		input_pix_format = 1;
		input_yuv_format = 1;
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12:
		input_pix_format = 0;
		input_yuv_format = 1;
		break;
	case V4L2_PIX_FMT_NV61M:
	case V4L2_PIX_FMT_NV61:
		input_pix_format = 1;
		input_yuv_format = 0;
		break;
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV16:
		input_pix_format = 0;
		input_yuv_format = 0;
		break;
	default:
		pr_info("[Error] : check for GDC input format\n");
		input_pix_format = 0;
		input_yuv_format = 1;
		break;
	}

	/* output */
	switch (d_frame->gdc_fmt->pixelformat) {
	case V4L2_PIX_FMT_NV21M:
	case V4L2_PIX_FMT_NV21:
		output_pix_format = 1;
		output_yuv_format = 1;
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV12:
		output_pix_format = 0;
		output_yuv_format = 1;
		break;
	case V4L2_PIX_FMT_NV61M:
	case V4L2_PIX_FMT_NV61:
		output_pix_format = 1;
		output_yuv_format = 0;
		break;
	case V4L2_PIX_FMT_NV16M:
	case V4L2_PIX_FMT_NV16:
		output_pix_format = 0;
		output_yuv_format = 0;
		break;
	default:
		pr_info("[Error] : check for GDC output format\n");
		output_pix_format = 0;
		output_yuv_format = 1;
		break;
	}

	if (input_yuv_format != output_yuv_format)
		pr_info("[Error] : Need to check : GDC in/out YUV format is different.\n");

	/* IN/OUT Format */
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_YUV_FORMAT],
		&gdc_fields[GDC_F_DST_PIXEL_FORMAT], output_pix_format);

	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_YUV_FORMAT],
		&gdc_fields[GDC_F_SRC_PIXEL_FORMAT], input_pix_format);

	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_YUV_FORMAT],
		&gdc_fields[GDC_F_GDC_YUV_FORMAT], input_yuv_format);

}
#endif

void camerapp_hw_gdc_update_param(void __iomem *base_addr, struct gdc_dev *gdc)
{
	struct gdc_frame *d_frame, *s_frame;
	struct gdc_crop_param *crop_param;

	s_frame = &gdc->current_ctx->s_frame;
	d_frame = &gdc->current_ctx->d_frame;
	crop_param = &gdc->current_ctx->crop_param;

	/* Interpolation type: 0 - all closest / 1 - Y bilinear, UV closest / 2 - all bilinear / 3 - Y bi-cubic, UV bilinear */
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INTERPOLATION], &gdc_fields[GDC_F_GDC_INTERP_TYPE], 3);
	/* Clamping type: 0 - none / 1 - min/max / 2 - near pixels min/max */
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_INTERPOLATION], &gdc_fields[GDC_F_GDC_CLAMP_TYPE], 2);

	/* gdc grid scale setting */
	camerapp_hw_gdc_update_scale_parameters(base_addr, s_frame, d_frame, crop_param);
	/* gdc grid setting */
	camerapp_hw_gdc_update_grid_param(base_addr, &gdc->current_ctx->grid_param);

	/* dma buffer size & address setting */
	camerapp_hw_gdc_set_dma_address(base_addr, s_frame, d_frame);
	camerapp_hw_gdc_update_dma_size(base_addr, s_frame, d_frame);

#if defined CONFIG_CAMERA_PP_GDC_V1_0_0_OBJ
	/*
	 * HACK:
	 * Don't want color conversion at a input G2D cause
	 * not performance to a 120fps mode
	 *
	 * Pixel format = 0: NV12 (2plane Y/UV order) / 1: NV21 (2plane Y/VU order) No 3plane
	 * 0:  SRC(NV12M) DST(NV12M)
	 * 1:  SRC(NV21M) DST(NV12M)
	 */
	if (s_frame->gdc_fmt->pixelformat == d_frame->gdc_fmt->pixelformat)
		camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_PXC_PIXEL_FORMAT], &gdc_fields[GDC_F_PIXELFORMAT], 0);
	else
		camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_PXC_PIXEL_FORMAT], &gdc_fields[GDC_F_PIXELFORMAT], 1);
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_PXC_PIXEL_FORMAT], &gdc_fields[GDC_F_ENDIAN], 0);	/* 0 : Little endian  / 1:  Big endian */
#elif defined CONFIG_CAMERA_PP_GDC_V1_1_0_OBJ
	/* in / out data Format */
	camerapp_hw_gdc_set_format(base_addr, s_frame, d_frame);
#endif

	/* The values are multiples of 32, value : 32 ~ 512 */
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_PRO_SIZE], &gdc_fields[GDC_F_GDC_PRO_WIDTH], 64);
	/* The values are multiples of 8, value : 8 ~ 512 */
	camerapp_sfr_set_field(base_addr, &gdc_regs[GDC_R_GDC_PRO_SIZE], &gdc_fields[GDC_F_GDC_PRO_HEIGHT], 64);

	camerapp_sfr_set_reg(base_addr, &gdc_regs[GDC_R_GDC_PROCESSING], 1);
}


void camerapp_hw_gdc_status_read(void __iomem *base_addr)
{
	u32 ReqCntLum;
	u32 HitCntLum;
	u32 isPixelBusy;
	u32 isPortBusy;
	u32 cacheStatus;
	u32 hitFifoStatus;
	u32 missFifoStatus;
	u32 isPending;

	/* read Total request count of luminance cache channel #0 */
	ReqCntLum = camerapp_sfr_get_reg(base_addr, &gdc_regs[GDC_R_PXC_REQ_CNT_LUM_0]);
	HitCntLum = camerapp_sfr_get_reg(base_addr, &gdc_regs[GDC_R_PXC_HIT_CNT_LUM_0]);
	isPixelBusy = camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_PXC_DEBUG_BUSY], &gdc_fields[GDC_F_PXCBUSYLUM]);
	isPortBusy = camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_PXC_DEBUG_BUSY], &gdc_fields[GDC_F_PORT0BUSY]);
	cacheStatus = camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_PXC_DEBUG_LUM0], &gdc_fields[GDC_F_CHROMACACHEST]);
	hitFifoStatus = camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_PXC_DEBUG_LUM0], &gdc_fields[GDC_F_HITFIFOST]);
	missFifoStatus = camerapp_sfr_get_field(base_addr, &gdc_regs[GDC_R_PXC_DEBUG_LUM0], &gdc_fields[GDC_F_MISSFIFOST]);
	isPending = camerapp_sfr_get_reg(base_addr, &gdc_regs[GDC_R_PEND]);

	return;
}

void camerapp_gdc_sfr_dump(void __iomem *base_addr)
{
	u32 i = 0;
	u32 reg_value = 0;

	gdc_dbg("gdc v1.0");

	for(i = 0; i < GDC_REG_CNT; i++) {
		reg_value = readl(base_addr + gdc_regs[i].sfr_offset);
		gdc_dbg("[DUMP] reg:[%s][0x%04X], value:[0x%08X]\n",
			gdc_regs[i].reg_name, gdc_regs[i].sfr_offset, reg_value);
	}
}
