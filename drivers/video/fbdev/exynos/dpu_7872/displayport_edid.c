/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung SoC DisplayPort EDID driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/fb.h>
#include <media/v4l2-dv-timings.h>
#include <uapi/linux/v4l2-dv-timings.h>

#include "displayport.h"

#define EDID_SEGMENT_ADDR	(0x60 >> 1)
#define EDID_ADDR		(0xA0 >> 1)
#define EDID_SEGMENT_IGNORE	(2)
#define EDID_BLOCK_SIZE		128
#define EDID_SEGMENT(x)		((x) >> 1)
#define EDID_OFFSET(x)		(((x) & 1) * EDID_BLOCK_SIZE)
#define EDID_EXTENSION_FLAG	0x7E
#define EDID_NATIVE_FORMAT	0x83
#define EDID_BASIC_AUDIO	(1 << 6)

struct displayport_supported_preset displayport_supported_presets[] = {
	{V4L2_DV_BT_CEA_720X480P59_94,	720,  480,  59, FB_VMODE_NONINTERLACED, "720x480p@60"},
	{V4L2_DV_BT_CEA_720X576P50,	720,  576,  50, FB_VMODE_NONINTERLACED, "720x576p@50"},
	{V4L2_DV_BT_CEA_1280X720P50,	1280, 720,  50, FB_VMODE_NONINTERLACED, "1280x720p@50"},
	{V4L2_DV_BT_CEA_1280X720P60,	1280, 720,  60, FB_VMODE_NONINTERLACED, "1280x720p@60"},
	{V4L2_DV_BT_CEA_1920X1080P24,	1920, 1080, 24, FB_VMODE_NONINTERLACED, "1920x1080p@24"},
	{V4L2_DV_BT_CEA_1920X1080P25,	1920, 1080, 25, FB_VMODE_NONINTERLACED, "1920x1080p@25"},
	{V4L2_DV_BT_CEA_1920X1080P30,	1920, 1080, 30, FB_VMODE_NONINTERLACED, "1920x1080p@30"},
	{V4L2_DV_BT_CEA_1920X1080P50,	1920, 1080, 50, FB_VMODE_NONINTERLACED, "1920x1080p@50"},
	{V4L2_DV_BT_CEA_1920X1080P60,	1920, 1080, 60, FB_VMODE_NONINTERLACED, "1920x1080p@60"},
	{V4L2_DV_BT_CEA_3840X2160P24,	3840, 2160, 24, FB_VMODE_NONINTERLACED, "3840x2160p@24"},
	{V4L2_DV_BT_CEA_3840X2160P25,	3840, 2160, 25, FB_VMODE_NONINTERLACED, "3840x2160p@25"},
	{V4L2_DV_BT_CEA_3840X2160P30,	3840, 2160, 30, FB_VMODE_NONINTERLACED, "3840x2160p@30"},
	{V4L2_DV_BT_CEA_3840X2160P50,	3840, 2160, 50, FB_VMODE_NONINTERLACED, "3840x2160p@50"},
	{V4L2_DV_BT_CEA_3840X2160P60,	3840, 2160, 60, FB_VMODE_NONINTERLACED, "3840x2160p@60"},
	{V4L2_DV_BT_CEA_4096X2160P24,	4096, 2160, 24, FB_VMODE_NONINTERLACED, "4096x2160p@24"},
	{V4L2_DV_BT_CEA_4096X2160P25,	4096, 2160, 25, FB_VMODE_NONINTERLACED, "4096x2160p@25"},
	{V4L2_DV_BT_CEA_4096X2160P30,	4096, 2160, 30, FB_VMODE_NONINTERLACED, "4096x2160p@30"},
	{V4L2_DV_BT_CEA_4096X2160P50,	4096, 2160, 50, FB_VMODE_NONINTERLACED, "4096x2160p@50"},
	{V4L2_DV_BT_CEA_4096X2160P60,	4096, 2160, 60, FB_VMODE_NONINTERLACED, "4096x2160p@60"},
};

static struct fb_videomode ud_mode_h14b_vsdb[] = {
	{"3840x2160p@30", 30, 3840, 2160, 297000000, 0, 0, 0, 0, 0, 0, 0, FB_VMODE_NONINTERLACED, 0},
	{"3840x2160p@25", 25, 3840, 2160, 297000000, 0, 0, 0, 0, 0, 0, 0, FB_VMODE_NONINTERLACED, 0},
	{"3840x2160p@24", 24, 3840, 2160, 297000000, 0, 0, 0, 0, 0, 0, 0, FB_VMODE_NONINTERLACED, 0},
	{"4096x2160p@24", 24, 4096, 2160, 297000000, 0, 0, 0, 0, 0, 0, 0, FB_VMODE_NONINTERLACED, 0},
};

const int displayport_pre_cnt = ARRAY_SIZE(displayport_supported_presets);

static struct v4l2_dv_timings preferred_preset;
static u32 edid_misc;
static int max_audio_channels;
static int audio_bit_rates;
static int audio_sample_rates;

static int edid_read_block(struct displayport_device *hdev, int block, u8 *buf, size_t len)
{
	int ret, i;
	u8 offset = EDID_OFFSET(block);
	u8 sum = 0;

	if (len < EDID_BLOCK_SIZE)
		return -EINVAL;

	ret = displayport_reg_edid_read(offset, EDID_BLOCK_SIZE, buf);
	if (ret)
		return ret;

	for (i = 0; i < EDID_BLOCK_SIZE; i++)
		sum += buf[i];

	if (sum) {
		displayport_err("%s: checksum error block=%d sum=%d\n", __func__, block, sum);
		return -EPROTO;
	}

	return 0;
}

static int edid_read(struct displayport_device *hdev, u8 **data)
{
	u8 block0[EDID_BLOCK_SIZE];
	u8 *edid;
	int block = 0;
	int block_cnt, ret;

	ret = edid_read_block(hdev, 0, block0, sizeof(block0));
	if (ret)
		return ret;

	block_cnt = block0[EDID_EXTENSION_FLAG] + 1;
	displayport_dbg("block_cnt = %d\n", block_cnt);

	edid = kmalloc(block_cnt * EDID_BLOCK_SIZE, GFP_KERNEL);
	if (!edid)
		return -ENOMEM;

	memcpy(edid, block0, sizeof(block0));

	while (++block < block_cnt) {
		ret = edid_read_block(hdev, block,
			edid + block * EDID_BLOCK_SIZE,
			EDID_BLOCK_SIZE);

		if (edid[EDID_NATIVE_FORMAT] & EDID_BASIC_AUDIO)
			edid_misc = FB_MISC_HDMI;

		if (ret) {
			kfree(edid);
			return ret;
		}
	}

	*data = edid;

	return block_cnt;
}

static unsigned int get_ud_timing(struct fb_vendor *vsdb, unsigned int vic_idx)
{
	unsigned char val = 0;
	unsigned int idx = 0;

	val = vsdb->vic_data[vic_idx];
	switch (val) {
	case 0x01:
		idx = 0;
		break;
	case 0x02:
		idx = 1;
		break;
	case 0x03:
		idx = 2;
		break;
	case 0x04:
		idx = 3;
		break;
	}

	return idx;
}

bool edid_find_max_resolution(const struct v4l2_dv_timings *t1,
			const struct v4l2_dv_timings *t2)
{
	if ((t1->bt.width * t1->bt.height < t2->bt.width * t2->bt.height) ||
		((t1->bt.width * t1->bt.height == t2->bt.width * t2->bt.height) &&
		(t1->bt.pixelclock < t2->bt.pixelclock)))
		return true;

	return false;
}

static bool edid_find_preset(const struct fb_videomode *mode, bool first)
{
	int i;

	for (i = 0; i < displayport_pre_cnt; i++) {
		if (mode->refresh == displayport_supported_presets[i].refresh &&
			mode->xres == displayport_supported_presets[i].xres &&
			mode->yres == displayport_supported_presets[i].yres &&
			mode->vmode == displayport_supported_presets[i].vmode) {
			if (displayport_supported_presets[i].edid_support_match == false) {
				displayport_dbg("EDID: found %s\n", displayport_supported_presets[i].name);
				displayport_supported_presets[i].edid_support_match = true;
			}

			if (first && displayport_supported_presets[i].edid_support_match) {
				preferred_preset = displayport_supported_presets[i].dv_timings;
				first = false;
				displayport_info("EDID: preferred_preset = %s\n", displayport_supported_presets[i].name);
			}

			if (displayport_supported_presets[i].edid_support_match) {
				if (edid_find_max_resolution(&preferred_preset,
					&displayport_supported_presets[i].dv_timings)) {
					preferred_preset = displayport_supported_presets[i].dv_timings;
					displayport_info("EDID: preferred_preset = %s\n", displayport_supported_presets[i].name);
				}
			}
		}
	}

	return first;
}

static void edid_use_default_preset(void)
{
	int i;

	preferred_preset = displayport_supported_presets[EDID_DEFAULT_TIMINGS_IDX].dv_timings;
	for (i = 0; i < displayport_pre_cnt; i++) {
		displayport_supported_presets[i].edid_support_match =
			v4l2_match_dv_timings(&displayport_supported_presets[i].dv_timings,
					&preferred_preset, 0);
	}

	max_audio_channels = 2;
}

void edid_parse_vsdb(unsigned char *edid_ext_blk, struct fb_vendor *vsdb, int block_cnt)
{
	int i, j;
	int vsdb_len, hdmi_vic_len;

	for (i = 0; i < block_cnt * 128; i++) {
		if ((edid_ext_blk[i] & VSDB_TAG_CODE_MASK) == (VSDB_TAG_CODE << VSDB_TAG_CODE_BIT_POSITION)
			&& edid_ext_blk[i + 1] == IEEE_REGISTRATION_IDENTIFIER_0
			&& edid_ext_blk[i + 2] == IEEE_REGISTRATION_IDENTIFIER_1
			&& edid_ext_blk[i + 3] == IEEE_REGISTRATION_IDENTIFIER_2) {
			vsdb_len = edid_ext_blk[i] & VSDB_LENGTH_MASK;

			displayport_dbg("EDID: find vsdb\n");

			if (vsdb_len > VSDB_VIC_FIELD_OFFSET) {
				hdmi_vic_len = (edid_ext_blk[i + VSDB_VIC_FIELD_OFFSET]
					& VSDB_VIC_LENGTH_MASK) >> VSDB_TAG_CODE_BIT_POSITION;

				if (hdmi_vic_len > 0) {
					vsdb->vic_len = hdmi_vic_len;

					for (j = 1; j < hdmi_vic_len; j++)
						vsdb->vic_data[j] = edid_ext_blk[i + VSDB_VIC_FIELD_OFFSET + j];

					break;
				}
			} else {
				vsdb->vic_len = 0;
				displayport_dbg("EDID: no hdmi vic data in vsdb\n");
				break;
			}
		}
	}

	if (i >= block_cnt * 128) {
		vsdb->vic_len = 0;
		displayport_dbg("EDID: can't find vsdb block\n");
	}
}

void edid_extension_update(struct fb_vendor *vsdb)
{
	unsigned int udmode_idx, vic_idx;

	if (!vsdb)
		return;

	/* find UHD preset in HDMI 1.4 vsdb block*/
	if (vsdb->vic_len) {
		for (vic_idx = 0; vic_idx < vsdb->vic_len; vic_idx++) {
			udmode_idx = get_ud_timing(vsdb, vic_idx);
			edid_find_preset(&ud_mode_h14b_vsdb[udmode_idx], false);
		}
	}
}

int edid_update(struct displayport_device *hdev)
{
	struct fb_monspecs specs;
	struct fb_vendor vsdb;
	bool first = true;
	u8 *edid = NULL;
	int block_cnt = 0;
	int i;
#if 0
	int channels_max = 0, support_bit_rates = 0, support_sample_rates = 0;
#endif

	edid_misc = 0;

	block_cnt = edid_read(hdev, &edid);
	if (block_cnt < 0)
		goto out;

	fb_edid_to_monspecs(edid, &specs);

	for (i = 1; i < block_cnt; i++)
		fb_edid_add_monspecs(edid + i * EDID_BLOCK_SIZE, &specs);

	preferred_preset = displayport_supported_presets[EDID_DEFAULT_TIMINGS_IDX].dv_timings;

	for (i = 0; i < displayport_pre_cnt; i++)
		displayport_supported_presets[i].edid_support_match = false;

	/* find 2D preset */
	for (i = 0; i < specs.modedb_len; i++)
		first = edid_find_preset(&specs.modedb[i], first);

	/* number of 128bytes blocks to follow */
	if (block_cnt > 1) {
		edid_parse_vsdb(edid+EDID_BLOCK_SIZE, &vsdb, block_cnt);
		edid_extension_update(&vsdb);
	} else
		goto out;

	if (!edid_misc)
		edid_misc = specs.misc;

	displayport_dbg("EDID: misc flags %08x", edid_misc);

	for (i = 0; i < displayport_pre_cnt; i++)
		displayport_dbg("displayport_supported_presets[%d].edid_support_match = %d\n",
				i, displayport_supported_presets[i].edid_support_match);
#if 0
	if (!specs.audiodb)
		goto out;

	for (i = 0; i < specs.audiodb_len; i++) {
		if (specs.audiodb[i].format != FB_AUDIO_LPCM)
			continue;
		if (specs.audiodb[i].channel_count > channels_max) {
			channels_max = specs.audiodb[i].channel_count;
			support_sample_rates = specs.audiodb[i].sample_rates;
			support_bit_rates = specs.audiodb[i].bit_rates;
		}
	}

	if (edid_misc & FB_MISC_HDMI) {
		if (channels_max) {
			max_audio_channels = channels_max;
			audio_sample_rates = support_sample_rates;
			audio_bit_rates = support_bit_rates;
		} else {
			max_audio_channels = 2;
			audio_sample_rates = FB_AUDIO_44KHZ; /*default audio info*/
			audio_bit_rates = FB_AUDIO_16BIT;
		}
	} else {
		max_audio_channels = 0;
		audio_sample_rates = 0;
		audio_bit_rates = 0;
	}
#else
	audio_sample_rates = FB_AUDIO_44KHZ; /*default audio info*/
	audio_bit_rates = FB_AUDIO_16BIT;
#endif
	displayport_dbg("EDID: Audio channels %d", max_audio_channels);

out:
	/* No supported preset found, use default */
	if (first) {
		displayport_dbg("edid_use_default_preset\n");
		edid_use_default_preset();
	}

	if (block_cnt == -EPROTO)
		edid_misc = FB_MISC_HDMI;

	kfree(edid);
	return block_cnt;
}

struct v4l2_dv_timings edid_preferred_preset(void)
{
	return preferred_preset;
}

bool edid_supports_hdmi(struct displayport_device *hdev)
{
	return edid_misc & FB_MISC_HDMI;
}

u32 edid_audio_informs(struct displayport_device *hdev)
{
	u32 value = 0, ch_info = 0;

	if (max_audio_channels > 0)
		ch_info |= (1 << (max_audio_channels - 1));
	if (max_audio_channels > 6)
		ch_info |= (1 << 5);
	value = ((audio_sample_rates << 19) | (audio_bit_rates << 16) |
			ch_info);
	return value;
}
