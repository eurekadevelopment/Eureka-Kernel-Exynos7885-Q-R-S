/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Header file for Samsung EXYNOS SoC DisplayPort driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _DISPLAYPORT_H_
#define _DISPLAYPORT_H_

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <media/v4l2-subdev.h>
#include <linux/switch.h>

#include "regs-displayport.h"
#include "./panels/decon_lcd.h"

extern int displayport_log_level;

#define DISPLAYPORT_MODULE_NAME "exynos-displayport"

#define displayport_err(fmt, ...)						\
	do {									\
		if (displayport_log_level >= 3) {				\
			pr_err(pr_fmt(fmt), ##__VA_ARGS__);			\
			exynos_ss_printk(fmt, ##__VA_ARGS__);			\
		}								\
	} while (0)

#define displayport_warn(fmt, ...)						\
	do {									\
		if (displayport_log_level >= 4) {				\
			pr_warn(pr_fmt(fmt), ##__VA_ARGS__);			\
			exynos_ss_printk(fmt, ##__VA_ARGS__);			\
		}								\
	} while (0)

#define displayport_info(fmt, ...)						\
	do {									\
		if (displayport_log_level >= 6)					\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);			\
	} while (0)

#define displayport_dbg(fmt, ...)						\
	do {									\
		if (displayport_log_level >= 7)					\
			pr_info(pr_fmt(fmt), ##__VA_ARGS__);			\
	} while (0)

extern struct displayport_device *displayport_drvdata;

enum displayport_state {
	DISPLAYPORT_STATE_INIT,
	DISPLAYPORT_STATE_ON,
	DISPLAYPORT_STATE_OFF
};

enum displayport_dynamic_range_type {
	VESA_RANGE = 0,   /* (0 ~ 255) */
	CEA_RANGE = 1,    /* (16 ~ 235) */
};

struct displayport_resources {
	int aux_ch_mux_gpio;
	int irq;
	void __iomem *regs;
};

enum displayport_aux_ch_command_type {
	I2C_WRITE = 0x4,
	I2C_READ = 0x5,
	DPCD_WRITE = 0x8,
	DPCD_READ = 0x9,
};

typedef enum {
	NORAMAL_DATA = 0,
	TRAINING_PATTERN_1 = 1,
	TRAINING_PATTERN_2 = 2,
	TRAINING_PATTERN_3 = 3,
} displayport_training_pattern;

enum displayport_interrupt_mask {
	VSYNC_DET_INT_MASK,
	PLL_LOCK_CHG_INT_MASK,
	VID_FORMAT_CHG_INT_MASK,
	VID_CLK_CHG_INT_MASK,
	HOTPLUG_CHG_INT_MASK,
	HPD_LOST_INT_MASK,
	PLUG_INT_MASK,
	INT_HPD_INT_MASK,
	RPLY_RECEIV_INT_MASK,
	AUX_ERR_INT_MASK,
	HDCP_LINK_CHECK_INT_MASK,
	HDCP_LINK_FAIL_INT_MASK,
	HW_HDCP_DONE_INT_MASK,
	HW_AUTH_CHG_INT_MASK,
	HDCP_R0_READY_INT_MASK,
/*
	AUDIO_FIFO_UNDER_RUN_INT_MASK,
	AUDIO_FIFO_OVER_RUN_INT_MASK,
*/
	ALL_INT_MASK
};

#define MAX_LANE_CNT 4
#define DPCD_BUF_SIZE 10

#define FB_AUDIO_LPCM	1

#define FB_AUDIO_192KHZ	(1 << 6)
#define FB_AUDIO_176KHZ	(1 << 5)
#define FB_AUDIO_96KHZ	(1 << 4)
#define FB_AUDIO_88KHZ	(1 << 3)
#define FB_AUDIO_48KHZ	(1 << 2)
#define FB_AUDIO_44KHZ	(1 << 1)
#define FB_AUDIO_32KHZ	(1 << 0)

#define FB_AUDIO_24BIT	(1 << 2)
#define FB_AUDIO_20BIT	(1 << 1)
#define FB_AUDIO_16BIT	(1 << 0)

struct fb_audio {
	u8 format;
	u8 channel_count;
	u8 sample_rates;
	u8 bit_rates;
};

struct fb_vendor {
	u8 vic_len;
	u8 vic_data[16];
};

#define DPCD_ADD_MAX_LINK_RATE 0x00001
#define LINK_RATE_1_62Gbps 0x06
#define LINK_RATE_2_7Gbps 0x0A
#define LINK_RATE_5_4Gbps 0x14

#define DPCD_ADD_MAX_LANE_COUNT 0x00002
#define MAX_LANE_COUNT (0x1F << 0)

#define DPCD_ADD_MAX_DOWNSPREAD 0x00003
#define NO_AUX_HANDSHAKE_LINK_TRANING (1 << 6)

#define DPCD_ADD_TRAINING_AUX_RD_INTERVAL 0x0000E
#define TRANING_AUX_RD_INTERVAL_400us 0x00
#define TRANING_AUX_RD_INTERVAL_4ms 0x01
#define TRANING_AUX_RD_INTERVAL_8ms 0x02
#define TRANING_AUX_RD_INTERVAL_12ms 0x03
#define TRANING_AUX_RD_INTERVAL_16ms 0x04

#define DPCD_ADD_LINK_BW_SET 0x00100

#define DPCD_ADD_LANE_COUNT_SET 0x00101

#define DPCD_ADD_TRANING_PATTERN_SET 0x00102
#define TRAINING_PTTERN_SELECT (3 << 0)
#define RECOVERED_CLOCK_OUT_EN (1 << 4)
#define DPCD_SCRAMBLING_DISABLE (1 << 5)

#define DPCD_ADD_TRANING_LANE0_SET 0x00103
#define VOLTAGE_SWING_SET (3 << 0)
#define MAX_SWING_REACHED (1 << 2)
#define PRE_EMPHASIS_SWING_SET (3 << 3)
#define MAX_PRE_EMPHASIS_REACHED (1 << 5)

#define DPCD_ADD_TRANING_LANE1_SET 0x00104

#define DPCD_ADD_TRANING_LANE2_SET 0x00105

#define DPCD_ADD_TRANING_LANE3_SET 0x00106

#define DPCD_ADD_DEVICE_SERVICE_IRQ_VECTOR 0x00201
#define AUTOMATED_TEST_REQUEST (1 << 1)
#define CP_IRQ (1 << 2)
#define MCCS_IRQ (1 << 3)
#define DOWN_REP_MSG_RDY (1 << 4)
#define UP_REQ_MSG_RDY (1 << 5)
#define SINK_SPECIFIC_IRQ (1 << 6)

#define DPCD_ADD_LANE0_1_STATUS 0x00202
#define LANE0_CR_DONE (1 << 0)
#define LANE0_CHANNEL_EQ_DONE (1 << 1)
#define LANE0_SYMBOL_LOCKED (1 << 2)
#define LANE1_CR_DONE (1 << 4)
#define LANE1_CHANNEL_EQ_DONE (1 << 5)
#define LANE1_SYMBOL_LOCKED (1 << 6)

#define DPCD_ADD_LANE2_3_STATUS 0x00203
#define LANE2_CR_DONE (1 << 0)
#define LANE2_CHANNEL_EQ_DONE (1 << 1)
#define LANE2_SYMBOL_LOCKED (1 << 2)
#define LANE3_CR_DONE (1 << 4)
#define LANE3_CHANNEL_EQ_DONE (1 << 5)
#define LANE3_SYMBOL_LOCKED (1 << 6)

#define DPCD_ADD_LANE_ALIGN_STATUS_UPDATE 0x00204
#define INTERLANE_ALIGN_DONE (1 << 0)
#define DOWNSTREAM_PORT_STATUS_CHANGED (1 << 6)
#define LINK_STATUS_UPDATE (1 << 7)

#define DPCD_ADD_ADJUST_REQUEST_LANE0_1 0x00206
#define VOLTAGE_SWING_LANE0 (3 << 0)
#define PRE_EMPHASIS_LANE0 (3 << 2)
#define VOLTAGE_SWING_LANE1 (3 << 4)
#define PRE_EMPHASIS_LANE1 (3 << 6)

#define DPCD_ADD_ADJUST_REQUEST_LANE2_3 0x00207
#define VOLTAGE_SWING_LANE2 (3 << 0)
#define PRE_EMPHASIS_LANE2 (3 << 2)
#define VOLTAGE_SWING_LANE3 (3 << 4)
#define PRE_EMPHASIS_LANE3 (3 << 6)

#define DPCD_ADD_SET_POWER 0x00600
#define SET_POWER_STATE (3 << 0)
#define SET_POWER_DOWN 0x02
#define SET_POWER_NORMAL 0x01

typedef enum {
	PIXEL_CLOCK_27_000,
	PIXEL_CLOCK_27_027,
	PIXEL_CLOCK_74_250,
	PIXEL_CLOCK_148_500,
	PIXEL_CLOCK_297_000,
	PIXEL_CLOCK_594_000,
} pixelclock;

typedef struct {
	u32 p;
	u32 m;
	u32 s;
	u32 divide;
} pms_info;

typedef enum {
	v720x480p_60Hz,
	v1280x720p_60Hz,
	v1920x1080p_60Hz,

	v720x576p_50Hz,
	v1280x720p_50Hz,
	v1920x1080p_50Hz,

	v3840x2160p_24Hz,
	v3840x2160p_25Hz,
	v3840x2160p_30Hz,
	v3840x2160p_50Hz,
	v3840x2160p_60Hz,

	v4096x2160p_24Hz,
	v4096x2160p_25Hz,
	v4096x2160p_30Hz,
	v4096x2160p_50Hz,
	v4096x2160p_60Hz,

	sa_crc_640x10_60Hz,
} videoformat;

typedef struct {
	videoformat video_format;
	u32 total_pixel;
	u32 active_pixel;
	u32 v_f_porch;
	u32 v_sync;
	u32 v_b_porch;
	u32 total_line;
	u32 active_line;
	u32 h_f_porch;
	u32 h_sync;
	u32 h_b_porch;
	u32 fps;
	pixelclock pixel_clock;
} videoformat_info;

typedef enum{
	ASYNC_MODE = 0,
	SYNC_MODE,
} audio_sync_mode;

enum audio_sampling_frequency {
	FS_32KHZ	= 0,
	FS_44KHZ	= 1,
	FS_48KHZ	= 2,
	FS_88KHZ	= 3,
	FS_96KHZ	= 4,
	FS_176KHZ	= 5,
	FS_192KHZ	= 6,
};

enum audio_bit_per_channel {
	AUDIO_16_BIT = 0,
	AUDIO_20_BIT,
	AUDIO_24_BIT,
};

enum audio_16bit_dma_mode {
	NORMAL_MODE = 0,
	PACKED_MODE = 1,
	PACKED_MODE2 = 2,
};

enum audio_dma_word_length {
	WORD_LENGTH_1 = 0,
	WORD_LENGTH_2,
	WORD_LENGTH_3,
	WORD_LENGTH_4,
	WORD_LENGTH_5,
	WORD_LENGTH_6,
	WORD_LENGTH_7,
	WORD_LENGTH_8,
};

struct displayport_device {
	enum displayport_state state;
	struct device *dev;
	struct displayport_resources res;

	unsigned int data_lane;
	u32 data_lane_cnt;
	struct phy *phy;
	spinlock_t slock;

	struct dsim_lcd_driver *panel_ops;
	struct decon_lcd lcd_info;

	struct v4l2_subdev sd;
	struct switch_dev hpd_switch;
	struct v4l2_dv_timings cur_timings;

	struct mutex cmd_lock;
};

extern videoformat_info videoformat_parameters[];
extern pms_info pms_parameters[];
extern videoformat g_displayport_videoformat;

static inline struct displayport_device *get_displayport_drvdata(void)
{
	return displayport_drvdata;
}

/* register access subroutines */
static inline u32 displayport_read(u32 reg_id)
{
	struct displayport_device *displayport = get_displayport_drvdata();

	return readl(displayport->res.regs + reg_id);
}

static inline u32 displayport_read_mask(u32 reg_id, u32 mask)
{
	u32 val = displayport_read(reg_id);

	val &= (mask);
	return val;
}

static inline void displayport_write(u32 reg_id, u32 val)
{
	struct displayport_device *displayport = get_displayport_drvdata();

	writel(val, displayport->res.regs + reg_id);
}

static inline void displayport_write_mask(u32 reg_id, u32 val, u32 mask)
{
	struct displayport_device *displayport = get_displayport_drvdata();
	u32 old = displayport_read(reg_id);
	u32 bit_shift;

	for (bit_shift = 0; bit_shift < 32; bit_shift++) {
		if ((mask >> bit_shift) & 0x00000001)
			break;
	}

	val = ((val<<bit_shift) & mask) | (old & ~mask);
	writel(val, displayport->res.regs + reg_id);
}

void displayport_reg_set_dpu_cmu_mux(void);
void displayport_reg_set_usb_high_z(void);
void displayport_reg_set_pixel_clock(videoformat video_format);
void displayport_reg_init(void);
void displayport_reg_set_interrupt_mask(enum displayport_interrupt_mask param, u8 set);
void displayport_reg_start(void);
void displayport_reg_stop(void);
void displayport_reg_set_video_configuration(void);
int displayport_reg_dpcd_write(u32 address, u32 length, u8 *data);
int displayport_reg_dpcd_read(u32 address, u32 length, u8 *data);
int displayport_reg_dpcd_write_burst(u32 address, u32 length, u8 *data);
int displayport_reg_dpcd_read_burst(u32 address, u32 length, u8 *data);
int displayport_reg_edid_write(u8 edid_addr_offset, u32 length, u8 *data);
int displayport_reg_edid_read(u8 edid_addr_offset, u32 length, u8 *data);
void displayport_reg_phy_reset(u32 en);
void displayport_reg_lane_reset(u32 en);
void displayport_reg_set_link_bw(u8 link_rate);
void displayport_reg_set_lane_count(u8 lane_cnt);
void displayport_reg_wait_phy_pll_lock(void);
void displayport_reg_set_training_pattern(displayport_training_pattern pattern);
void displayport_reg_set_voltage_and_pre_emphasis(u8 *voltage, u8 *pre_emphasis);
void displayport_reg_get_voltage_and_pre_emphasis_max_reach(u8 *max_reach_value);
void displayport_reg_set_bist_video_configuration(videoformat video_format);
void displayport_reg_set_bist_mode(u32 en);
void displayport_reg_set_phy_clk_bw(u8 link_rate);
u32 displayport_reg_get_cmn_ctl_sfr_ctl_mode(void);

void displayport_reg_set_audio_m_n(audio_sync_mode audio_sync_mode,
		enum audio_sampling_frequency audio_sampling_freq);
void displayport_reg_set_audio_function_enable(u32 en);
void displayport_reg_set_audio_master_mode(void);
void displayport_reg_set_dma_burst_size(enum audio_dma_word_length word_length);
void displayport_reg_set_dma_pack_mode(enum audio_16bit_dma_mode dma_mode);
void displayport_reg_set_audio_ch(u32 audio_ch_cnt);
void displayport_reg_set_audio_fifo_function_enable(u32 en);
void displayport_reg_set_audio_sampling_frequency
		(enum audio_sampling_frequency audio_sampling_freq);
void displayport_reg_set_dp_audio_enable(u32 en);
void displayport_reg_set_audio_master_mode_enable(u32 en);

void displayport_reg_set_hdcp22_lane_count(void);
void displayport_reg_set_hdcp22_system_enable(u32 en);
void displayport_reg_set_hdcp22_mode(u32 en);
void displayport_reg_set_hdcp22_encryption_enable(u32 en);

void displayport_reg_set_lane_map(u32 lane0, u32 lane1, u32 lane2, u32 lane3);

void displayport_hpd_changed(int state);

/* EDID functions */
/* default preset configured on probe */
#define EDID_DEFAULT_TIMINGS_IDX (8)

#define EDID_ADDRESS 0x50
#define AUX_DATA_BUF_COUNT 16
#define EDID_BUF_COUNT 256

#define VSDB_TAG_CODE 3
#define VSDB_TAG_CODE_MASK 0xE0
#define VSDB_TAG_CODE_BIT_POSITION 5
#define VSDB_LENGTH_MASK 0x1F
#define VSDB_VIC_FIELD_OFFSET 14
#define VSDB_VIC_LENGTH_MASK 0xE0
#define IEEE_REGISTRATION_IDENTIFIER_0 0x03
#define IEEE_REGISTRATION_IDENTIFIER_1 0x0C
#define IEEE_REGISTRATION_IDENTIFIER_2 0x00

int edid_update(struct displayport_device *hdev);
struct v4l2_dv_timings edid_preferred_preset(void);

struct displayport_supported_preset {
	struct v4l2_dv_timings dv_timings;
	u16 xres;
	u16 yres;
	u16 refresh;
	u32 vmode;
	char *name;
	bool edid_support_match;
};

extern const int displayport_pre_cnt;
extern struct displayport_supported_preset displayport_supported_presets[];
extern const int videoformat_parameters_cnt;

struct exynos_displayport_data {
	enum {
		EXYNOS_DISPLAYPORT_STATE_PRESET = 0,
		EXYNOS_DISPLAYPORT_STATE_ENUM_PRESET,
		EXYNOS_DISPLAYPORT_STATE_RECONNECTION,
		EXYNOS_DISPLAYPORT_STATE_HDCP,
		EXYNOS_DISPLAYPORT_STATE_AUDIO,
	} state;
	struct	v4l2_dv_timings timings;
	struct	v4l2_enum_dv_timings etimings;
	__u32	audio_info;
	int	hdcp;
};

struct displayport_audio_config_data {
	u32 audio_enable;
	u32 audio_channel_cnt;
	enum audio_sampling_frequency audio_fs;
	enum audio_bit_per_channel audio_bit;
	enum audio_16bit_dma_mode audio_packed_mode;
	enum audio_dma_word_length audio_word_length;
};

/* HDCP 1.3 */
typedef struct{
	u8 HDCP13_BKSV[5];
	u8 HDCP13_R0[2];
	u8 HDCP13_AKSV[5];
	u8 HDCP13_AN[8];
	u8 HDCP13_V_H0[4];
	u8 HDCP13_V_H1[4];
	u8 HDCP13_V_H2[4];
	u8 HDCP13_V_H3[4];
	u8 HDCP13_V_H4[4];
	u8 HDCP13_BCAP[1];
	u8 HDCP13_BSTATUS[1];
	u8 HDCP13_BINFO[2];
	u8 HDCP13_KSV_FIFO[15];
	u8 HDCP13_AINFO[1];
} HDCP13;

enum HDCP13_STATE {
	HDCP13_STATE_NOT_AUTHENTICATED,
	HDCP13_STATE_SECOND_AUTH_DONE,
	HDCP13_STATE_AUTHENTICATED
};

struct hdcp13_info {
	int running;
	u8 is_repeater;
	u8 device_cnt;
	u8 revocation_check;
	u8 r0_read_flag;
	int link_check;
	enum HDCP13_STATE auth_state;
};

#define ADDR_HDCP13_BKSV 0x68000
#define ADDR_HDCP13_R0 0x68005
#define ADDR_HDCP13_AKSV 0x68007
#define ADDR_HDCP13_AN 0x6800C
#define ADDR_HDCP13_V_H0 0x68014
#define ADDR_HDCP13_V_H1 0x68018
#define ADDR_HDCP13_V_H2 0x6801C
#define ADDR_HDCP13_V_H3 0x68020
#define ADDR_HDCP13_V_H4 0x68024
#define ADDR_HDCP13_BCAP 0x68028
#define ADDR_HDCP13_BSTATUS 0x68029
#define ADDR_HDCP13_BINFO 0x6802A
#define ADDR_HDCP13_KSV_FIFO 0x6802C
#define ADDR_HDCP13_AINFO 0x6803B
#define ADDR_HDCP13_RSVD 0x6803C
#define ADDR_HDCP13_DBG 0x680C0

#define BSTATUS_READ (1<<0)
#define BSTATUS_R0_AVAILABLE (1<<1)
#define BSTATUS_LINK_INTEGRITY_FAIL (1<<2)
#define BSTATUS_REAUTH_REQ (1<<3)

#define RI_AVAILABLE_WAITING 2
#define RI_DELAY 100

enum{
	LINK_CHECK_PASS = 0,
	LINK_CHECK_NEED = 1,
	LINK_CHECK_FAIL = 2,
};

enum{
	FIRST_AUTH  = 0,
	SECOND_AUTH = 1,
};

#define DISPLAYPORT_IOC_DUMP			_IOW('V', 0, u32)
#define DISPLAYPORT_IOC_GET_ENUM_DV_TIMINGS	_IOW('V', 1, u8)
#define DISPLAYPORT_IOC_SET_RECONNECTION	_IOW('V', 2, u8)
#endif
