/* drivers/video/exynos/panels/s6e3fa0_lcd_ctrl.c
 *
 * Samsung SoC MIPI LCD CONTROL functions
 *
 * Copyright (c) 2014 Samsung Electronics
 *
 * Jiun Yu, <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include "lcd_ctrl.h"

#include "../dsim.h"
#include <video/mipi_display.h>

#define GAMMA_PARAM_SIZE 26

#define EXTEND_BRIGHTNESS	365
#define UI_MAX_BRIGHTNESS	255
#define UI_DEFAULT_BRIGHTNESS	128
#define NORMAL_TEMPERATURE	25	/* 25 degrees Celsius */

#define GAMMA_CMD_CNT				((u16)ARRAY_SIZE(SEQ_GAMMA_CONDITION_SET))
#define ACL_CMD_CNT					((u16)ARRAY_SIZE(SEQ_ACL_OFF))
#define OPR_CMD_CNT					((u16)ARRAY_SIZE(SEQ_ACL_OPR_OFF))
#define ELVSS_CMD_CNT				((u16)ARRAY_SIZE(SEQ_ELVSS_SET))
#define AID_CMD_CNT					((u16)ARRAY_SIZE(SEQ_AID_SETTING))
#define TSET_CMD_CNT				((u16)ARRAY_SIZE(SEQ_TSET_SETTING))

#define LDI_REG_ELVSS				0xB6
#define LDI_REG_COORDINATE			0xA1
#define LDI_REG_DATE				LDI_REG_MTP
#define LDI_REG_ID					0x04
#define LDI_REG_CHIP_ID				0xD6
#define LDI_REG_MTP					0xC8
#define LDI_REG_HBM					0xB4
#define LDI_REG_RDDPM				0x0A
#define LDI_REG_RDDSM				0x0E
#define LDI_REG_ESDERR				0xEE
#define LDI_REG_MANUFACTURE_INFO	0xC9

/* len is read length */
#define LDI_LEN_ELVSS				(ELVSS_CMD_CNT - 1)
#define LDI_LEN_COORDINATE			4
#define LDI_LEN_DATE				7
#define LDI_LEN_ID					3
#define LDI_LEN_CHIP_ID				5
#define LDI_LEN_MTP					35
#define LDI_LEN_HBM					28
#define LDI_LEN_RDDPM				1
#define LDI_LEN_RDDSM				1
#define LDI_LEN_ESDERR				1
#define LDI_LEN_MANUFACTURE_INFO	21

/* offset is position including addr, not only para */
#define LDI_OFFSET_AOR_1	1
#define LDI_OFFSET_AOR_2	2

#define LDI_OFFSET_ELVSS_1	1	/* B6h 1st Para: MPS_CON */
#define LDI_OFFSET_ELVSS_2	2	/* B6h 2nd Para: ELVSS_Dim_offset */
#define LDI_OFFSET_ELVSS_3	22	/* B6h 22nd Para: ELVSS Temp Compensation */

#define LDI_OFFSET_OPR_1	1	/* B5h 1st Para: Frame Avg. */
#define LDI_OFFSET_OPR_2	2	/* B5h 2nd Para: Start Point */
#define LDI_OFFSET_OPR_3	4	/* B5h 22nd Para: ACL Percent */

#define LDI_OFFSET_ACL		1
#define LDI_OFFSET_TSET		1

#define LDI_GPARA_DATE		40	/* 0xC8 41st Para */
#define LDI_GPARA_HBM_ELVSS	22	/* 0xB6 23th para */

/*
 * 3FAH3 lcd init sequence
 *
 * Parameters
 *	- mic : if mic is enabled, MIC_ENABLE command must be sent
 *	- mode : LCD init sequence depends on command or video mode
 */

static unsigned char SEQ_SLEEP_OUT[] = {
	0x11
};

static unsigned char SEQ_DISPLAY_ON[] = {
	0x29
};

static unsigned char SEQ_TEST_KEY_ON_F0[] = {
	0xF0,
	0x5A, 0x5A
};

static unsigned char SEQ_TEST_KEY_OFF_F0[] = {
	0xF0,
	0xA5, 0xA5
};

static unsigned char SEQ_TEST_KEY_ON_FC[] = {
	0xFC,
	0x5A, 0x5A
};

static unsigned char SEQ_TEST_KEY_OFF_FC[] = {
	0xFC,
	0xA5, 0xA5
};

static const unsigned char SEQ_TEST_KEY_ON_F2[] = {
	0xF2,
	0x01, 0x00, 0x0A, 0x5A, 0xF0, 0x09, 0x07
};

static unsigned char SEQ_TE_ON[] = {
	0x35,
	0x00
};

static unsigned char SEQ_PCD_SET_DET_LOW[] = {
	0xCC,
	0x5C
};

static unsigned char SEQ_ERR_FG_SETTING[] = {
	0xED,
	0x44
};

static unsigned char SEQ_AVC_SETTING[] = {
	0xFD,
	0xA1, 0x20, 0x08, 0x04, 0x44, 0x00, 0x0C, 0x07,
	0x00, 0x00, 0xFF, 0x00, 0x0A, 0xAA, 0x0A, 0xAA,
	0x0A, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x5A, 0x80, 0x57, 0x65,
	0xA8			/* AVC -6.4V Setting */
};

static unsigned char SEQ_PARTIAL_MODE_ON[] = {
	0x12
};

static unsigned char SEQ_NORMAL_MODE_ON[] = {
	0x13
};

static unsigned char SEQ_PARTIAL_AREA_SETTING[] = {
	0x30,
	0x07, 0x7E, 0x07, 0x7F
};

static unsigned char SEQ_GAMMA_CONDITION_SET[] = {
	0xCA,
	0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x00, 0x00, 0x00, 0x00, 0x00
};

static unsigned char SEQ_AID_SETTING[] = {
	0xB2,
	0x00, 0x14
};

static unsigned char SEQ_ELVSS_SET[] = {
	0xB6,
	0xBC,	/* B6h 1st Para: MPS_CON */
	0x0A,	/* B6h 2nd Para: ELVSS_Dim_offset */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
	0x00,	/* B6h 22nd Para: ELVSS Temp Compensation */
	0x00	/* B6h 23rd Para: OTP for B6h 22nd Para of HBM Interpolation */
};

static unsigned char SEQ_GAMMA_UPDATE[] = {
	0xF7,
	0x03
};

static const unsigned char SEQ_HBM_OFF[] = {
	0x53,
	0x00
};

static unsigned char SEQ_ACL_OPR_OFF[] = {
	0xB5,
	0x40,	/* 16 Frame Avg. at ACL Off */
	0x7F,	/* Start Point 50% */
	0x14,
	0x14	/* ACL 15% */
};
#if 0
static unsigned char SEQ_ACL_OPR_08P[] = {
	0xB5,
	0x50,	/* 32 Frame Avg. at ACL On */
	0x99,	/* Start Point 60% */
	0x14,
	0x0A	/* ACL 8% */
};

static unsigned char SEQ_ACL_OPR_15P[] = {
	0xB5,
	0x50,	/* 32 Frame Avg. at ACL On */
	0x7F,	/* Start Point 50% */
	0x14,
	0x14	/* ACL 15% */
};
#endif
static unsigned char SEQ_ACL_OFF[] = {
	0x55,
	0x00
};
#if 0
static unsigned char SEQ_ACL_ON[] = {
	0x55,
	0x02	/* 0x02 : ACL 8% (Default) */
};
#endif
static unsigned char SEQ_TSET_SETTING[] = {
	0xB8,
	0x19	/* (ex) 25 degree : 0x19 */
};

static unsigned char SEQ_OSC_FREQ_ADJ[] = {
	0xCE,
	0x29, 0xD0, 0x4B, 0x4C,	/* 0x4B, 0x4C : 60.0Hz */
	0x4B, 0x38, 0x06, 0x03,
	0x00, 0xFF, 0x0A, 0x3F,
	0x3F, 0x3F, 0x3F
};

static int dsi_write(u32 id, const unsigned char *wbuf, int size)
{
	int ret = 0;

	if (size == 1)
		ret = dsim_wr_data(id, MIPI_DSI_DCS_SHORT_WRITE, wbuf[0], 0);
	else
		ret = dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)wbuf, size);

	mdelay(12);

	return ret;
}
#if 0
static int dsi_read(u32 id, const u8 addr, u16 len, u8 *buf)
{
	int ret = 0;

	ret = dsim_rd_data(id, MIPI_DSI_DCS_READ, addr, len, buf);

	return ret;
}
#endif
static int s6e3fa3_read_id(u32 id)
{
#if 0
	int i, ret, retry_cnt = 1;
	u8 buf[LDI_LEN_ID];

/* to LSI: if your dsi rx function is not stable, do not try to read until it is verified */
return 1;

	for (i = 0; i < LDI_LEN_ID; i++)
		buf[i] = 0;
retry:
	ret = dsi_read(id, LDI_REG_ID, LDI_LEN_ID, (u8 *)buf);
	if (ret <= 0) {
		if (retry_cnt) {
			dsim_err("%s: retry: %d\n", __func__, retry_cnt);
			retry_cnt--;
			goto retry;
		} else
			dsim_err("%s: 0x%02x\n", __func__, LDI_REG_ID);
		return 0;
	}

	for (i = 0; i < LDI_LEN_ID; i++)
		dsim_dbg("%x, ", buf[i]);
	dsim_dbg("\n");

	return ret;
#else
	return 1;
#endif
}

void lcd_init(int id, struct decon_lcd *lcd)
{
	dsim_dbg("%s +\n", __func__);

	/* 7. Sleep Out(11h) */
	if (dsi_write(id, SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT)) < 0)
		dsim_err("fail to send SEQ_SLEEP_OUT command.\n");

	/* 8. Wait 20ms */
	msleep(20);

	/* 9. ID READ */
	s6e3fa3_read_id(id);

	/* Test Key Enable */
	if (dsi_write(id, SEQ_TEST_KEY_ON_F0, ARRAY_SIZE(SEQ_TEST_KEY_ON_F0)) < 0)
		dsim_err("fail to send SEQ_TEST_KEY_ON_F0 command.\n");
	if (dsi_write(id, SEQ_TEST_KEY_ON_FC, ARRAY_SIZE(SEQ_TEST_KEY_ON_FC)) < 0)
		dsim_err("fail to send SEQ_TEST_KEY_ON_FC command.\n");

	if (lcd->mode == DECON_VIDEO_MODE) {
		if (dsi_write(id, SEQ_TEST_KEY_ON_F2, ARRAY_SIZE(SEQ_TEST_KEY_ON_F2)) < 0)
			dsim_err("fail to send SEQ_TEST_KEY_ON_F2 command.\n");
	}

	/* 10. Common Setting */
	/* 4.1.6 FFC Setting */
	if (dsi_write(id, SEQ_OSC_FREQ_ADJ, ARRAY_SIZE(SEQ_OSC_FREQ_ADJ)) < 0)
		dsim_err("fail to send SEQ_OSC_FREQ_ADJ command.\n");

	/* 4.1.5 Partial Mode Setting */
	if (dsi_write(id, SEQ_PARTIAL_MODE_ON, ARRAY_SIZE(SEQ_PARTIAL_MODE_ON)) < 0)
		dsim_err("fail to send SEQ_PARTIAL_MODE_ON command.\n");
	if (dsi_write(id, SEQ_PARTIAL_AREA_SETTING, ARRAY_SIZE(SEQ_PARTIAL_AREA_SETTING)) < 0)
		dsim_err("fail to send SEQ_PARTIAL_AREA_SETTING command.\n");

	/* 4.1.2 PCD Setting */
	if (dsi_write(id, SEQ_PCD_SET_DET_LOW, ARRAY_SIZE(SEQ_PCD_SET_DET_LOW)) < 0)
		dsim_err("fail to send SEQ_PCD_SET_DET_LOW command.\n");
	/* 4.1.3 ERR_FG Setting */
	if (dsi_write(id, SEQ_ERR_FG_SETTING, ARRAY_SIZE(SEQ_ERR_FG_SETTING)) < 0)
		dsim_err("fail to send SEQ_ERR_FG_SETTING command.\n");
	/* 4.1.4 AVC Setting */
	if (dsi_write(id, SEQ_AVC_SETTING, ARRAY_SIZE(SEQ_AVC_SETTING)) < 0)
		dsim_err("fail to send SEQ_AVC_SETTING command.\n");

	/* 11. Brightness control */
	if (dsi_write(id, SEQ_GAMMA_CONDITION_SET, ARRAY_SIZE(SEQ_GAMMA_CONDITION_SET)) < 0)
		dsim_err("fail to send SEQ_GAMMA_CONDITION_SET command.\n");
	if (dsi_write(id, SEQ_AID_SETTING, ARRAY_SIZE(SEQ_AID_SETTING)) < 0)
		dsim_err("fail to send SEQ_AID_SETTING command.\n");
	if (dsi_write(id, SEQ_ELVSS_SET, ARRAY_SIZE(SEQ_ELVSS_SET)) < 0)
		dsim_err("fail to send SEQ_ELVSS_SET command.\n");
	if (dsi_write(id, SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE)) < 0)
		dsim_err("fail to send SEQ_GAMMA_UPDATE command.\n");

	/* 4.2.4 ACL ON/OFF */
	if (dsi_write(id, SEQ_ACL_OPR_OFF, ARRAY_SIZE(SEQ_ACL_OPR_OFF)) < 0)
		dsim_err("fail to send SEQ_ACL_OPR_OFF command.\n");
	if (dsi_write(id, SEQ_ACL_OFF, ARRAY_SIZE(SEQ_ACL_OFF)) < 0)
		dsim_err("fail to send SEQ_ACL_OFF command.\n");

	/* 4.2.3 HBM */
	if (dsi_write(id, SEQ_HBM_OFF, ARRAY_SIZE(SEQ_HBM_OFF)) < 0)
		dsim_err("fail to send SEQ_HBM_OFF command.\n");

	/* 4.3 ELVSS Temp Compensation */
	if (dsi_write(id, SEQ_TSET_SETTING, ARRAY_SIZE(SEQ_TSET_SETTING)) < 0)
		dsim_err("fail to send SEQ_TSET_SETTING command.\n");

	if (lcd->mode == DECON_MIPI_COMMAND_MODE) {
		/* Test Key Disable */
		if (dsi_write(id, SEQ_TEST_KEY_OFF_F0, ARRAY_SIZE(SEQ_TEST_KEY_OFF_F0)) < 0)
			dsim_err("fail to send SEQ_TEST_KEY_OFF_F0 command.\n");
		if (dsi_write(id, SEQ_TEST_KEY_OFF_FC, ARRAY_SIZE(SEQ_TEST_KEY_OFF_FC))  < 0)
			dsim_err("fail to send SEQ_TEST_KEY_OFF_FC command.\n");
	}

	/* 4.1.1 TE(Vsync) ON/OFF */
#if 0
	if (dsi_write(id, SEQ_TE_ON, ARRAY_SIZE(SEQ_TE_ON)) < 0)
		dsim_err("fail to send SEQ_TE_ON command.\n");
#else
	if (lcd->mode == DECON_MIPI_COMMAND_MODE) {
		if (dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)SEQ_TE_ON,
					ARRAY_SIZE(SEQ_TE_ON)) < 0)
			dsim_err("fail to send SEQ_TE_ON command.\n");
	}
#endif
	/* 12. Wait 80ms */
	msleep(80);

	dsim_dbg("%s -\n", __func__);
}

void lcd_enable(int id)
{
	dsim_dbg("%s +\n", __func__);

	/* 16. Display On(29h) */
	if (dsi_write(id, SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON)) < 0)
		dsim_err("fail to send SEQ_DISPLAY_ON command.\n");

	/* 17. Wait 10ms */
	/*msleep(10);*/

	/* 18. Normal Mode On(13h) */
	if (dsi_write(id, SEQ_NORMAL_MODE_ON, ARRAY_SIZE(SEQ_NORMAL_MODE_ON)) < 0)
		dsim_err("fail to send SEQ_NORMAL_MODE_ON command.\n");

	dsim_dbg("%s -\n", __func__);
}

void lcd_disable(int id)
{
	/* This function needs to implement */
}

/*
 * Set gamma values
 *
 * Parameter
 *	- backlightlevel : It is from 0 to 26.
 */
int lcd_gamma_ctrl(int id, u32 backlightlevel)
{
#if 0
	int ret;

	ret = dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)gamma22_table[backlightlevel],
			GAMMA_PARAM_SIZE);
	if (ret) {
		dsim_err("fail to write gamma value.\n");
		return ret;
	}
#endif
	return 0;
}

int lcd_gamma_update(int id)
{
#if 0
	int ret;

	ret = dsim_wr_data(id, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)SEQ_GAMMA_UPDATE,
			ARRAY_SIZE(SEQ_GAMMA_UPDATE));
	if (ret) {
		dsim_err("fail to update gamma value.\n");
		return ret;
	}
#endif
	return 0;
}
