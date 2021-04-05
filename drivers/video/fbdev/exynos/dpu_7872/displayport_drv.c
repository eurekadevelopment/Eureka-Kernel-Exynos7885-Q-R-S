/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung SoC DisplayPort driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/pm_runtime.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/module.h>
#include <video/mipi_display.h>

#include "displayport.h"
#include "decon.h"

int displayport_log_level = 6;
videoformat g_displayport_videoformat = v1920x1080p_60Hz;

struct displayport_device *displayport_drvdata;
EXPORT_SYMBOL(displayport_drvdata);

static int displayport_runtime_suspend(struct device *dev);
static int displayport_runtime_resume(struct device *dev);

static void displayport_dump_registers(struct displayport_device *displayport)
{
	displayport_info("=== DisplayPort SFR DUMP ===\n");

	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
			displayport->res.regs, 0xC0, false);
}

static int displayport_remove(struct platform_device *pdev)
{
	struct displayport_device *displayport = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	mutex_destroy(&displayport->cmd_lock);
	kfree(displayport);
	displayport_info("displayport driver removed\n");

	return 0;
}
#if 0 /* HACK */
static irqreturn_t displayport_irq_handler(int irq, void *dev_id)
{
	unsigned int int_src;
	struct displayport_device *displayport = dev_id;
	int active;

	spin_lock(&displayport->slock);

	active = pm_runtime_active(displayport->dev);
	if (!active) {
		displayport_info("displayport power(%d), state(%d)\n", active, displayport->state);
		spin_unlock(&displayport->slock);
		return IRQ_HANDLED;
	}

	int_src = readl(displayport->res.regs + Interrupt_Status_Top);

	/* displayport_reg_clear_int(int_src); */

	spin_unlock(&displayport->slock);

	return IRQ_HANDLED;
}
#endif
static int displayport_get_gpios(struct displayport_device *displayport)
{
	struct device *dev = displayport->dev;
	struct displayport_resources *res = &displayport->res;

	displayport_info("%s +\n", __func__);

	if (of_get_property(dev->of_node, "gpios", NULL) != NULL)  {
		/* panel reset */
		res->aux_ch_mux_gpio = of_get_gpio(dev->of_node, 0);
		if (res->aux_ch_mux_gpio < 0) {
			displayport_err("failed to get aux ch mux GPIO");
			return -ENODEV;
		}
	}

	displayport_info("%s -\n", __func__);
	return 0;
}

static int displayport_full_link_training(void)
{
	u8 link_rate;
	u8 lane_cnt;
	u8 training_aux_rd_interval;
	u8 pre_emphasis[MAX_LANE_CNT];
	u8 drive_current[MAX_LANE_CNT];
	u8 voltage_swing_lane[MAX_LANE_CNT];
	u8 pre_emphasis_lane[MAX_LANE_CNT];
	u8 max_reach_value[MAX_LANE_CNT];
	int training_retry_no, i;
	u8 val[DPCD_BUF_SIZE];
	u8 lane_cr_done;
	u8 lane_channel_eq_done;
	u8 lane_symbol_locked_done;
	u8 interlane_align_done;
	int ret = 0;

	displayport_reg_set_lane_map(3, 2, 1, 0); /* HACK for universal board, need modify */

	displayport_reg_dpcd_read_burst(DPCD_ADD_MAX_LINK_RATE, 2, val);
	link_rate = val[0];
	lane_cnt = val[1] & MAX_LANE_COUNT;

	displayport_reg_dpcd_read(DPCD_ADD_TRAINING_AUX_RD_INTERVAL, 1, val);
	training_aux_rd_interval = val[0];

Reduce_Link_Rate_Retry:
	displayport_dbg("Reduce_Link_Rate_Retry\n");

	for (i = 0; i < 4; i++) {
		pre_emphasis[i] = 0;
		drive_current[i] = 0;
	}

	training_retry_no = 0;

	displayport_reg_phy_reset(1);

	displayport_reg_set_link_bw(link_rate);

	if (displayport_reg_get_cmn_ctl_sfr_ctl_mode()) {
		displayport_reg_set_phy_clk_bw(link_rate);
		displayport_dbg("displayport_reg_set_phy_clk_bw\n");
	}

	displayport_reg_set_lane_count(lane_cnt);

	displayport_dbg("link_rate = %x\n", link_rate);
	displayport_dbg("lane_cnt = %x\n", lane_cnt);

	displayport_reg_phy_reset(0);

	val[0] = link_rate;
	val[1] = lane_cnt;
	displayport_reg_dpcd_write_burst(DPCD_ADD_LINK_BW_SET, 2, val);

	displayport_reg_wait_phy_pll_lock();

	displayport_reg_set_training_pattern(TRAINING_PATTERN_1);

	val[0] = 0x21;	/* SCRAMBLING_DISABLE, TRAINING_PATTERN_1 */
	displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, val);

Voltage_Swing_Retry:
	displayport_dbg("Voltage_Swing_Retry\n");

	displayport_reg_set_voltage_and_pre_emphasis((u8 *)drive_current, (u8 *)pre_emphasis);
	displayport_reg_get_voltage_and_pre_emphasis_max_reach((u8 *)max_reach_value);

	val[0] = (pre_emphasis[0]<<3) | drive_current[0] | max_reach_value[0];
	val[1] = (pre_emphasis[1]<<3) | drive_current[1] | max_reach_value[0];
	val[2] = (pre_emphasis[2]<<3) | drive_current[2] | max_reach_value[0];
	val[3] = (pre_emphasis[3]<<3) | drive_current[3] | max_reach_value[0];
	displayport_reg_dpcd_write_burst(DPCD_ADD_TRANING_LANE0_SET, 4, val);

	udelay((training_aux_rd_interval*4000)+400);

	lane_cr_done = 0;

	displayport_reg_dpcd_read(DPCD_ADD_LANE0_1_STATUS, 2, val);
	lane_cr_done |= ((val[0] & LANE0_CR_DONE) >> 0);
	lane_cr_done |= ((val[0] & LANE1_CR_DONE) >> 3);
	lane_cr_done |= ((val[1] & LANE2_CR_DONE) << 2);
	lane_cr_done |= ((val[1] & LANE3_CR_DONE) >> 1);

	displayport_dbg("lane_cr_done = %x\n", lane_cr_done);

	if (lane_cnt == 0x04) {
		if (lane_cr_done == 0x0F) {
			displayport_dbg("lane_cr_done\n");
			goto EQ_Training_Start;
		}
	} else if (lane_cnt == 0x02) {
		if (lane_cr_done == 0x03) {
			displayport_dbg("lane_cr_done\n");
			goto EQ_Training_Start;
		}
	} else {
		if (lane_cr_done == 0x01) {
			displayport_dbg("lane_cr_done\n");
			goto EQ_Training_Start;
		}
	}

	if (!(drive_current[0] == 3 && drive_current[1] == 3
		&& drive_current[2] == 3 && drive_current[3] == 3)) {
		displayport_reg_dpcd_read_burst(DPCD_ADD_ADJUST_REQUEST_LANE0_1, 2, val);
		voltage_swing_lane[0] = (val[0] & VOLTAGE_SWING_LANE0);
		pre_emphasis_lane[0] = (val[0] & PRE_EMPHASIS_LANE0) >> 2;
		voltage_swing_lane[1] = (val[0] & VOLTAGE_SWING_LANE1) >> 4;
		pre_emphasis_lane[1] = (val[0] & PRE_EMPHASIS_LANE1) >> 6;

		voltage_swing_lane[2] = (val[1] & VOLTAGE_SWING_LANE2);
		pre_emphasis_lane[2] = (val[1] & PRE_EMPHASIS_LANE2) >> 2;
		voltage_swing_lane[3] = (val[1] & VOLTAGE_SWING_LANE3) >> 4;
		pre_emphasis_lane[3] = (val[1] & PRE_EMPHASIS_LANE3) >> 6;

		if (drive_current[0] == voltage_swing_lane[0] &&
			drive_current[1] == voltage_swing_lane[1] &&
			drive_current[2] == voltage_swing_lane[2] &&
			drive_current[3] == voltage_swing_lane[3]) {
			if (training_retry_no == 5)
				goto Check_Link_rate;
			else
				training_retry_no++;
		} else
			training_retry_no = 1;

		for (i = 0; i < 4; i++) {
			drive_current[i] = voltage_swing_lane[i];
			pre_emphasis[i] = pre_emphasis_lane[i];
		}

		goto Voltage_Swing_Retry;
	}

Check_Link_rate:
	displayport_dbg("Check_Link_rate\n");

	if (link_rate == LINK_RATE_5_4Gbps) {
		link_rate = LINK_RATE_2_7Gbps;
		goto Reduce_Link_Rate_Retry;
	} else if (link_rate == LINK_RATE_2_7Gbps) {
		link_rate = LINK_RATE_1_62Gbps;
		goto Reduce_Link_Rate_Retry;
	} else if (link_rate == LINK_RATE_1_62Gbps) {
		displayport_err("Full Link Training Fail !!!");
		return -EINVAL;
	}

EQ_Training_Start:
	displayport_dbg("EQ_Training_Start\n");

	displayport_reg_set_training_pattern(TRAINING_PATTERN_2);

	val[0] = 0x22;	/* SCRAMBLING_DISABLE, TRAINING_PATTERN_2 */
	displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, val);

EQ_Training_Retry:
	displayport_dbg("EQ_Training_Retry\n");

	displayport_reg_set_voltage_and_pre_emphasis((u8 *)drive_current, (u8 *)pre_emphasis);
	displayport_reg_get_voltage_and_pre_emphasis_max_reach((u8 *)max_reach_value);

	val[0] = (pre_emphasis[0]<<3) | drive_current[0] | max_reach_value[0];
	val[1] = (pre_emphasis[1]<<3) | drive_current[1] | max_reach_value[1];
	val[2] = (pre_emphasis[2]<<3) | drive_current[2] | max_reach_value[2];
	val[3] = (pre_emphasis[3]<<3) | drive_current[3] | max_reach_value[3];
	displayport_reg_dpcd_write_burst(DPCD_ADD_TRANING_LANE0_SET, 4, val);

	lane_cr_done = 0;
	lane_channel_eq_done = 0;
	lane_symbol_locked_done = 0;
	interlane_align_done = 0;

	udelay((training_aux_rd_interval*4000)+400);

	displayport_reg_dpcd_read_burst(DPCD_ADD_LANE0_1_STATUS, 3, val);
	lane_cr_done |= ((val[0] & LANE0_CR_DONE) >> 0);
	lane_cr_done |= ((val[0] & LANE1_CR_DONE) >> 3);
	lane_channel_eq_done |= ((val[0] & LANE0_CHANNEL_EQ_DONE) >> 1);
	lane_channel_eq_done |= ((val[0] & LANE1_CHANNEL_EQ_DONE) >> 4);
	lane_symbol_locked_done |= ((val[0] & LANE0_SYMBOL_LOCKED) >> 2);
	lane_symbol_locked_done |= ((val[0] & LANE1_SYMBOL_LOCKED) >> 5);

	lane_cr_done |= ((val[1] & LANE2_CR_DONE) << 2);
	lane_cr_done |= ((val[1] & LANE3_CR_DONE) >> 1);
	lane_channel_eq_done |= ((val[1] & LANE2_CHANNEL_EQ_DONE) << 1);
	lane_channel_eq_done |= ((val[1] & LANE3_CHANNEL_EQ_DONE) >> 2);
	lane_symbol_locked_done |= ((val[1] & LANE2_SYMBOL_LOCKED) >> 0);
	lane_symbol_locked_done |= ((val[1] & LANE3_SYMBOL_LOCKED) >> 3);

	interlane_align_done |= (val[2] & INTERLANE_ALIGN_DONE);

	if (lane_cnt == 0x04) {
		if (lane_cr_done != 0x0F)
			goto Check_Link_rate;
	} else if (lane_cnt == 0x02) {
		if (lane_cr_done != 0x03)
			goto Check_Link_rate;
	} else {
		if (lane_cr_done != 0x01)
			goto Check_Link_rate;
	}

	displayport_dbg("lane_cr_done = %x\n", lane_cr_done);
	displayport_dbg("lane_channel_eq_done = %x\n", lane_channel_eq_done);
	displayport_dbg("lane_symbol_locked_done = %x\n", lane_symbol_locked_done);
	displayport_dbg("interlane_align_done = %x\n", interlane_align_done);

	if (lane_cnt == 0x04) {
		if ((lane_channel_eq_done == 0x0F) && (lane_symbol_locked_done == 0x0F)
			&& (interlane_align_done == 1)) {
			displayport_reg_set_training_pattern(NORAMAL_DATA);

			val[0] = 0x00;	/* SCRAMBLING_ENABLE, NORMAL_DATA */
			displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, val);

			displayport_dbg("Link Training Finish\n");
			return ret;
		}
	} else if (lane_cnt == 0x02) {
		if ((lane_channel_eq_done == 0x03) && (lane_symbol_locked_done == 0x03)
			&& (interlane_align_done == 1)) {
			displayport_reg_set_training_pattern(NORAMAL_DATA);

			val[0] = 0x00;	/* SCRAMBLING_ENABLE, NORMAL_DATA */
			displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, val);

			displayport_dbg("Link Training Finish\n");
			return ret;
		}
	} else {
		if ((lane_channel_eq_done == 0x01) && (lane_symbol_locked_done == 0x01)
			&& (interlane_align_done == 1)) {
			displayport_reg_set_training_pattern(NORAMAL_DATA);

			val[0] = 0x00;	/* SCRAMBLING_ENABLE, NORMAL_DATA */
			displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, val);

			displayport_dbg("Link Training Finish\n");
			return ret;
		}
	}

	if (training_retry_no == 5)
		goto Check_Link_rate;

	displayport_reg_dpcd_read_burst(DPCD_ADD_ADJUST_REQUEST_LANE0_1, 2, val);
	voltage_swing_lane[0] = (val[0] & VOLTAGE_SWING_LANE0);
	pre_emphasis_lane[0] = (val[0] & PRE_EMPHASIS_LANE0) >> 2;
	voltage_swing_lane[1] = (val[0] & VOLTAGE_SWING_LANE1) >> 4;
	pre_emphasis_lane[1] = (val[0] & PRE_EMPHASIS_LANE1) >> 6;

	voltage_swing_lane[2] = (val[1] & VOLTAGE_SWING_LANE2);
	pre_emphasis_lane[2] = (val[1] & PRE_EMPHASIS_LANE2) >> 2;
	voltage_swing_lane[3] = (val[1] & VOLTAGE_SWING_LANE3) >> 4;
	pre_emphasis_lane[3] = (val[1] & PRE_EMPHASIS_LANE3) >> 6;

	for (i = 0; i < 4; i++) {
		drive_current[i] = voltage_swing_lane[i];
		pre_emphasis[i] = pre_emphasis_lane[i];

		displayport_dbg("drive_current[%d] = %x\n", i, drive_current[i]);
		displayport_dbg("pre_emphasis[%d] = %x\n", i, pre_emphasis[i]);
	}

	goto EQ_Training_Retry;
}

static int displayport_fast_link_training(void)
{
	u8 link_rate;
	u8 lane_cnt;
	u8 pre_emphasis[4];
	u8 drive_current[4];
	u8 max_reach_value[4];
	int i;
	u8 val;
	u8 lane_cr_done;
	u8 lane_channel_eq_done;
	u8 lane_symbol_locked_done;
	u8 interlane_align_done;
	int ret = 0;

	displayport_reg_dpcd_read(DPCD_ADD_MAX_LINK_RATE, 1, &val);
	link_rate = val;

	displayport_reg_dpcd_read(DPCD_ADD_MAX_LANE_COUNT, 1, &val);
	lane_cnt = val & MAX_LANE_COUNT;

	for (i = 0; i < 4; i++) {
		pre_emphasis[i] = 1;
		drive_current[i] = 2;
	}

	displayport_reg_phy_reset(1);

	displayport_reg_set_link_bw(link_rate);

	if (displayport_reg_get_cmn_ctl_sfr_ctl_mode()) {
		displayport_reg_set_phy_clk_bw(link_rate);
		displayport_dbg("displayport_reg_set_phy_clk_bw\n");
	}

	displayport_reg_set_lane_count(lane_cnt);

	displayport_dbg("link_rate = %x\n", link_rate);
	displayport_dbg("lane_cnt = %x\n", lane_cnt);

	displayport_reg_phy_reset(0);

	displayport_reg_dpcd_write(DPCD_ADD_LINK_BW_SET, 1, &link_rate);
	displayport_reg_dpcd_write(DPCD_ADD_LANE_COUNT_SET, 1, &lane_cnt);

	displayport_reg_lane_reset(1);

	displayport_reg_set_voltage_and_pre_emphasis((u8 *)drive_current, (u8 *)pre_emphasis);
	displayport_reg_get_voltage_and_pre_emphasis_max_reach((u8 *)max_reach_value);

	displayport_reg_lane_reset(0);

	val = (pre_emphasis[0]<<3) | drive_current[0] | max_reach_value[0];
	displayport_reg_dpcd_write(DPCD_ADD_TRANING_LANE0_SET, 1, &val);
	val = (pre_emphasis[1]<<3) | drive_current[1] | max_reach_value[0];
	displayport_reg_dpcd_write(DPCD_ADD_TRANING_LANE1_SET, 1, &val);
	val = (pre_emphasis[2]<<3) | drive_current[2] | max_reach_value[0];
	displayport_reg_dpcd_write(DPCD_ADD_TRANING_LANE2_SET, 1, &val);
	val = (pre_emphasis[3]<<3) | drive_current[3] | max_reach_value[0];
	displayport_reg_dpcd_write(DPCD_ADD_TRANING_LANE3_SET, 1, &val);

	displayport_reg_wait_phy_pll_lock();

	displayport_reg_set_training_pattern(TRAINING_PATTERN_1);

	udelay(500);

	lane_cr_done = 0;

	displayport_reg_dpcd_read(DPCD_ADD_LANE0_1_STATUS, 1, &val);
	lane_cr_done |= ((val & LANE0_CR_DONE) >> 0);
	lane_cr_done |= ((val & LANE1_CR_DONE) >> 3);

	displayport_reg_dpcd_read(DPCD_ADD_LANE2_3_STATUS, 1, &val);
	lane_cr_done |= ((val & LANE2_CR_DONE) << 2);
	lane_cr_done |= ((val & LANE3_CR_DONE) >> 1);

	displayport_dbg("lane_cr_done = %x\n", lane_cr_done);

	if (lane_cnt == 0x04) {
		if (lane_cr_done != 0x0F) {
			displayport_dbg("Fast Link Training Fail\n");
			return -EINVAL;
		}
	} else if (lane_cnt == 0x02) {
		if (lane_cr_done != 0x03) {
			displayport_dbg("Fast Link Training Fail\n");
			return -EINVAL;
		}
	} else {
		if (lane_cr_done != 0x01) {
			displayport_dbg("Fast Link Training Fail\n");
			return -EINVAL;
		}
	}

	displayport_reg_set_training_pattern(TRAINING_PATTERN_2);

	udelay(500);

	lane_cr_done = 0;
	lane_channel_eq_done = 0;
	lane_symbol_locked_done = 0;
	interlane_align_done = 0;

	displayport_reg_dpcd_read(DPCD_ADD_LANE0_1_STATUS, 1, &val);
	lane_cr_done |= ((val & LANE0_CR_DONE) >> 0);
	lane_cr_done |= ((val & LANE1_CR_DONE) >> 3);
	lane_channel_eq_done |= ((val & LANE0_CHANNEL_EQ_DONE) >> 1);
	lane_channel_eq_done |= ((val & LANE1_CHANNEL_EQ_DONE) >> 4);
	lane_symbol_locked_done |= ((val & LANE0_SYMBOL_LOCKED) >> 2);
	lane_symbol_locked_done |= ((val & LANE1_SYMBOL_LOCKED) >> 5);

	displayport_reg_dpcd_read(DPCD_ADD_LANE2_3_STATUS, 1, &val);
	lane_cr_done |= ((val & LANE2_CR_DONE) << 2);
	lane_cr_done |= ((val & LANE3_CR_DONE) >> 1);
	lane_channel_eq_done |= ((val & LANE2_CHANNEL_EQ_DONE) << 1);
	lane_channel_eq_done |= ((val & LANE3_CHANNEL_EQ_DONE) >> 2);
	lane_symbol_locked_done |= ((val & LANE2_SYMBOL_LOCKED) >> 0);
	lane_symbol_locked_done |= ((val & LANE3_SYMBOL_LOCKED) >> 3);

	displayport_reg_dpcd_read(DPCD_ADD_LANE_ALIGN_STATUS_UPDATE, 1, &val);
	interlane_align_done |= (val & INTERLANE_ALIGN_DONE);

	if (lane_cnt == 0x04) {
		if (lane_cr_done != 0x0F) {
			displayport_dbg("Fast Link Training Fail\n");
			return -EINVAL;
		}
	} else if (lane_cnt == 0x02) {
		if (lane_cr_done != 0x03) {
			displayport_dbg("Fast Link Training Fail\n");
			return -EINVAL;
		}
	} else {
		if (lane_cr_done != 0x01)
			displayport_dbg("Fast Link Training Fail\n");
			return -EINVAL;
	}

	displayport_dbg("lane_cr_done = %x\n", lane_cr_done);
	displayport_dbg("lane_channel_eq_done = %x\n", lane_channel_eq_done);
	displayport_dbg("lane_symbol_locked_done = %x\n", lane_symbol_locked_done);
	displayport_dbg("interlane_align_done = %x\n", interlane_align_done);

	if (lane_cnt == 0x04) {
		if ((lane_channel_eq_done == 0x0F) && (lane_symbol_locked_done == 0x0F)
			&& (interlane_align_done == 1)) {
			displayport_reg_set_training_pattern(NORAMAL_DATA);

			val = 0x00;	/* SCRAMBLING_ENABLE, NORMAL_DATA */
			displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, &val);

			displayport_dbg("Fast Link Training Finish\n");
			return ret;
		}
	} else if (lane_cnt == 0x02) {
		if ((lane_channel_eq_done == 0x03) && (lane_symbol_locked_done == 0x03)
			&& (interlane_align_done == 1)) {
			displayport_reg_set_training_pattern(NORAMAL_DATA);

			val = 0x00;	/* SCRAMBLING_ENABLE, NORMAL_DATA */
			displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, &val);

			displayport_dbg("Fast Link Training Finish\n");
			return ret;
		}
	} else {
		if ((lane_channel_eq_done == 0x01) && (lane_symbol_locked_done == 0x01)
			&& (interlane_align_done == 1)) {
			displayport_reg_set_training_pattern(NORAMAL_DATA);

			val = 0x00;	/* SCRAMBLING_ENABLE, NORMAL_DATA */
			displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, &val);

			displayport_dbg("Fast Link Training Finish\n");
			return ret;
		}
	}

	displayport_reg_set_training_pattern(NORAMAL_DATA);

	val = 0x00;	/* SCRAMBLING_ENABLE, NORMAL_DATA */
	displayport_reg_dpcd_write(DPCD_ADD_TRANING_PATTERN_SET, 1, &val);

	displayport_dbg("Fast Link Training Fail\n");
	return -EINVAL;
}

static int displayport_link_training(void)
{
	u8 val;
	int ret = 0;

	displayport_reg_dpcd_read(DPCD_ADD_MAX_DOWNSPREAD, 1, &val);
	displayport_dbg("DPCD_ADD_MAX_DOWNSPREAD = %x\n", val);

	if (val & NO_AUX_HANDSHAKE_LINK_TRANING)
		ret = displayport_fast_link_training();
	else
		ret = displayport_full_link_training();

	return ret;
}

void displayport_hpd_changed(int state)
{
	int ret;
	struct displayport_device *displayport = get_displayport_drvdata();

	if (state) {
		displayport_reg_set_usb_high_z();

		/* PHY power on */
		phy_power_on(displayport->phy);
		displayport_reg_init(); /* for AUX ch read/write. */

		ret = edid_update(displayport);
		if (ret < 0) {
			displayport_dbg("failed to update edid\n");
			return;
		}

		displayport->cur_timings = edid_preferred_preset();

		ret = displayport_link_training();
		if (ret < 0) {
			displayport_dbg("link training fail\n");
			return;
		}
	}

	switch_set_state(&displayport->hpd_switch, state);

	displayport_dbg("HPD status = %d\n", state);
}

int displayport_audio_config(struct displayport_audio_config_data audio_config_data)
{
	u8 val;
	int ret = 0;

	displayport_reg_dpcd_read(DPCD_ADD_MAX_DOWNSPREAD, 1, &val);
	displayport_dbg("displayport_audio_config\n");
	displayport_dbg("audio_enable = %d\n", audio_config_data.audio_enable);
	displayport_dbg("audio_channel_cnt = %d\n", audio_config_data.audio_channel_cnt);
	displayport_dbg("audio_fs = %d\n", audio_config_data.audio_fs);
	displayport_dbg("audio_bit = %d\n", audio_config_data.audio_bit);
	displayport_dbg("audio_packed_mode = %d\n", audio_config_data.audio_packed_mode);
	displayport_dbg("audio_word_length = %d\n", audio_config_data.audio_word_length);

	displayport_reg_set_audio_m_n(ASYNC_MODE, audio_config_data.audio_fs);
	displayport_reg_set_audio_function_enable(audio_config_data.audio_enable);
	displayport_reg_set_audio_master_mode();
	displayport_reg_set_dma_burst_size(audio_config_data.audio_word_length);
	displayport_reg_set_dma_pack_mode(audio_config_data.audio_packed_mode);
	displayport_reg_set_audio_ch(audio_config_data.audio_channel_cnt);
	displayport_reg_set_audio_fifo_function_enable(audio_config_data.audio_enable);
	displayport_reg_set_audio_sampling_frequency(audio_config_data.audio_fs);
	displayport_reg_set_dp_audio_enable(audio_config_data.audio_enable);
	displayport_reg_set_audio_master_mode_enable(audio_config_data.audio_enable);

	return ret;
}

int displayport_dpcd_read_for_hdcp22(u32 address, u32 length, u8 *data)
{
	int ret;

	ret = displayport_reg_dpcd_read_burst(address, length, data);

	if (ret != 0)
		displayport_err("displayport_dpcd_read_for_hdcp22 fail\n");

	return ret;
}

int displayport_dpcd_write_for_hdcp22(u8 address, u32 length, u8 *data)
{
	int ret;

	ret = displayport_reg_dpcd_write_burst(address, length, data);

	if (ret != 0)
		displayport_err("displayport_dpcd_write_for_hdcp22 fail\n");

	return ret;
}

void displayport_hdcp22_enable(u32 en)
{
	if (en) {
		displayport_reg_set_hdcp22_lane_count();
		displayport_reg_set_hdcp22_system_enable(1);
		displayport_reg_set_hdcp22_mode(1);
		displayport_reg_set_hdcp22_encryption_enable(1);
	} else {
		displayport_reg_set_hdcp22_system_enable(0);
		displayport_reg_set_hdcp22_mode(0);
		displayport_reg_set_hdcp22_encryption_enable(0);
	}
}

static int displayport_enable(struct displayport_device *displayport)
{
	int ret = 0;

	displayport_dbg("displayport_enable\n");

	if (displayport->state == DISPLAYPORT_STATE_ON)
		return 0;

	displayport_reg_set_pixel_clock(g_displayport_videoformat);
#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_get_sync(displayport->dev);
#else
	displayport_runtime_resume(displayport->dev);
#endif
	enable_irq(displayport->res.irq);

	displayport_reg_set_video_configuration();
	displayport_reg_start();

	displayport->state = DISPLAYPORT_STATE_ON;

	return ret;
}

static int displayport_disable(struct displayport_device *displayport)
{
	if (displayport->state == DISPLAYPORT_STATE_OFF)
		return 0;

	/* Wait for current read & write CMDs. */
	mutex_lock(&displayport->cmd_lock);
	displayport->state = DISPLAYPORT_STATE_OFF;
	mutex_unlock(&displayport->cmd_lock);

	displayport_reg_set_bist_mode(0);
	displayport_reg_stop();
	disable_irq(displayport->res.irq);

	phy_power_off(displayport->phy);

	displayport_hpd_changed(0); /* for test */

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_put_sync(displayport->dev);
#else
	displayport_runtime_suspend(displayport->dev);
#endif
	return 0;
}

bool displayport_match_timings(const struct v4l2_dv_timings *t1,
			const struct v4l2_dv_timings *t2,
			unsigned pclock_delta)
{
	if (t1->type != t2->type)
		return false;

	if (t1->bt.width == t2->bt.width &&
		t1->bt.height == t2->bt.height &&
		t1->bt.interlaced == t2->bt.interlaced &&
		t1->bt.polarities == t2->bt.polarities &&
		t1->bt.pixelclock >= t2->bt.pixelclock - pclock_delta &&
		t1->bt.pixelclock <= t2->bt.pixelclock + pclock_delta &&
		t1->bt.hfrontporch == t2->bt.hfrontporch &&
		t1->bt.vfrontporch == t2->bt.vfrontporch &&
		t1->bt.vsync == t2->bt.vsync &&
		t1->bt.vbackporch == t2->bt.vbackporch &&
		(!t1->bt.interlaced ||
		(t1->bt.il_vfrontporch == t2->bt.il_vfrontporch &&
		t1->bt.il_vsync == t2->bt.il_vsync &&
		t1->bt.il_vbackporch == t2->bt.il_vbackporch)))
		return true;

	return false;
}

bool displayport_match_video_params(u32 i, u32 j)
{
	if (displayport_supported_presets[i].xres == videoformat_parameters[j].active_pixel &&
		displayport_supported_presets[i].yres == videoformat_parameters[j].active_line &&
		displayport_supported_presets[i].refresh == videoformat_parameters[j].fps)
		return true;

	return false;
}

static videoformat displayport_timing2conf(struct v4l2_dv_timings *timings)
{
	u32 i, j;

	for (i = 0; i < displayport_pre_cnt; i++) {
		if (displayport_match_timings(&displayport_supported_presets[i].dv_timings,
					timings, 0)) {
			for (j = 0; j < videoformat_parameters_cnt; j++) {
				if (displayport_match_video_params(i, j))
					return j; /* eum matching between displayport_conf and videoformat */
			}
		}
	}

	return -EINVAL;
}

static int displayport_s_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings)
{
	struct displayport_device *displayport = container_of(sd, struct displayport_device, sd);
	videoformat displayport_setting_videoformat;

	displayport_setting_videoformat = displayport_timing2conf(timings);
	if (displayport_setting_videoformat < 0) {
		displayport_err("displayport timings not supported\n");
		return -EINVAL;
	}

	g_displayport_videoformat = displayport_setting_videoformat;
	displayport->cur_timings = *timings;

	displayport_dbg("New g_displayport_videoformat = %d\n", g_displayport_videoformat);

	return 0;
}

static int displayport_g_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings)
{
	struct displayport_device *displayport = container_of(sd, struct displayport_device, sd);

	*timings = displayport->cur_timings;

	displayport_dbg("displayport_g_dv_timings\n");

	return 0;
}

static int displayport_enum_dv_timings(struct v4l2_subdev *sd,
	struct v4l2_enum_dv_timings *timings)
{
	if (timings->index >= displayport_pre_cnt) {
		displayport_dbg("displayport_enum_dv_timings -EOVERFLOW\n");
		return -E2BIG;
	}

	if (timings->index == 0) /* index 0 is max resolution */
		timings->timings = edid_preferred_preset();
	else {
		if (displayport_supported_presets[timings->index - 1].edid_support_match) {
			displayport_dbg("displayport_enum_dv_timings edid_support_match index = %d\n", timings->index - 1);
			timings->timings = displayport_supported_presets[timings->index].dv_timings;
		} else {
			displayport_dbg("displayport_enum_dv_timings -EINVAL\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int displayport_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct displayport_device *displayport = container_of(sd, struct displayport_device, sd);

	if (enable)
		return displayport_enable(displayport);
	else
		return displayport_disable(displayport);
}

static long displayport_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct displayport_device *displayport = container_of(sd, struct displayport_device, sd);
	int ret = 0;
	struct v4l2_enum_dv_timings *enum_timings;

	switch (cmd) {
	case DISPLAYPORT_IOC_DUMP:
		displayport_dump_registers(displayport);
		break;

	case DISPLAYPORT_IOC_GET_ENUM_DV_TIMINGS:
		enum_timings = (struct v4l2_enum_dv_timings *)arg;

		ret = displayport_enum_dv_timings(sd, enum_timings);

		displayport_dbg("DISPLAYPORT_IOC_GET_ENUM_DV_TIMINGS ioctl\n");
		displayport_dbg("enum_dv_timings index id = %d\n", enum_timings->index);
		break;

	case DISPLAYPORT_IOC_SET_RECONNECTION:	/* for restart without hpd change */
		displayport_hpd_changed(1);
		displayport_info("DISPLAYPORT_IOC_SET_RECONNECTION ioctl\n");
		break;

	default:
		displayport_err("unsupported ioctl");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_subdev_core_ops displayport_sd_core_ops = {
	.ioctl = displayport_ioctl,
};

static const struct v4l2_subdev_video_ops displayport_sd_video_ops = {
	.s_dv_timings = displayport_s_dv_timings,
	.g_dv_timings = displayport_g_dv_timings,
	.s_stream = displayport_s_stream,
};

static const struct v4l2_subdev_ops displayport_subdev_ops = {
	.core = &displayport_sd_core_ops,
	.video = &displayport_sd_video_ops,
};

static void displayport_init_subdev(struct displayport_device *displayport)
{
	struct v4l2_subdev *sd = &displayport->sd;

	v4l2_subdev_init(sd, &displayport_subdev_ops);
	sd->owner = THIS_MODULE;
	snprintf(sd->name, sizeof(sd->name), "%s", "displayport-sd");
	v4l2_set_subdevdata(sd, displayport);
}

static int displayport_parse_dt(struct displayport_device *displayport, struct device *dev)
{
	if (IS_ERR_OR_NULL(dev->of_node)) {
		displayport_err("no device tree information\n");
		return -EINVAL;
	}

	displayport->phy = devm_phy_get(dev, "displayport_phy");
	if (IS_ERR_OR_NULL(displayport->phy)) {
		displayport_err("failed to get displayport phy\n");
		return PTR_ERR(displayport->phy);
	}

	displayport->dev = dev;
	displayport_get_gpios(displayport);

	return 0;
}

static int displayport_init_resources(struct displayport_device *displayport, struct platform_device *pdev)
{
	struct resource *res;
	/* int ret; */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		displayport_err("failed to get mem resource\n");
		return -ENOENT;
	}

	displayport_info("res: start(0x%x), end(0x%x)\n", (u32)res->start, (u32)res->end);

	displayport->res.regs = devm_ioremap_resource(displayport->dev, res);
	if (!displayport->res.regs) {
		displayport_err("failed to remap DisplayPort SFR region\n");
		return -EINVAL;
	}
#if 0 /* HACK */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		displayport_err("failed to get irq resource\n");
		return -ENOENT;
	}

	displayport->res.irq = res->start;
	ret = devm_request_irq(displayport->dev, res->start,
			displayport_irq_handler, 0, pdev->name, displayport);
	if (ret) {
		displayport_err("failed to install DisplayPort irq\n");
		return -EINVAL;
	}
	disable_irq(displayport->res.irq);
#endif
	return 0;
}

static int displayport_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct displayport_device *displayport = NULL;

	displayport = devm_kzalloc(dev, sizeof(struct displayport_device), GFP_KERNEL);
	if (!displayport) {
		displayport_err("failed to allocate displayport device.\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = displayport_parse_dt(displayport, dev);
	if (ret)
		goto err_dt;

	displayport_drvdata = displayport;

	spin_lock_init(&displayport->slock);
	mutex_init(&displayport->cmd_lock);

	ret = displayport_init_resources(displayport, pdev);
	if (ret)
		goto err_dt;

	displayport_init_subdev(displayport);
	platform_set_drvdata(pdev, displayport);

	/* register the switch device for HPD */
	displayport->hpd_switch.name = "displayport";
	ret = switch_dev_register(&displayport->hpd_switch);
	if (ret) {
		displayport_err("request switch class failed.\n");
		goto err_dt;
	}
	displayport_info("success register switch device\n");

	pm_runtime_enable(dev);

	phy_init(displayport->phy);

	displayport->state = DISPLAYPORT_STATE_INIT;

	displayport_info("displayport driver has been probed.\n");
	return 0;

err_dt:
	kfree(displayport);
err:
	return ret;
}

static void displayport_shutdown(struct platform_device *pdev)
{
	struct displayport_device *displayport = platform_get_drvdata(pdev);

	/* DPU_EVENT_LOG(DPU_EVT_DP_SHUTDOWN, &displayport->sd, ktime_set(0, 0)); */
	displayport_info("%s + state:%d\n", __func__, displayport->state);

	displayport_disable(displayport);

	displayport_info("%s -\n", __func__);
}

static int displayport_runtime_suspend(struct device *dev)
{
	/* struct displayport_device *displayport = dev_get_drvdata(dev); */

	/* DPU_EVENT_LOG(DPU_EVT_DP_SUSPEND, &displayport->sd, ktime_set(0, 0)); */
	displayport_dbg("%s +\n", __func__);

	displayport_dbg("%s -\n", __func__);
	return 0;
}

static int displayport_runtime_resume(struct device *dev)
{
	/* struct displayport_device *displayport = dev_get_drvdata(dev); */

	/* DPU_EVENT_LOG(DPU_EVT_DP_RESUME, &displayport->sd, ktime_set(0, 0)); */
	displayport_dbg("%s: +\n", __func__);

	displayport_dbg("%s -\n", __func__);
	return 0;
}

static const struct of_device_id displayport_of_match[] = {
	{ .compatible = "samsung,exynos-displayport" },
	{},
};
MODULE_DEVICE_TABLE(of, displayport_of_match);

static const struct dev_pm_ops displayport_pm_ops = {
	.runtime_suspend	= displayport_runtime_suspend,
	.runtime_resume		= displayport_runtime_resume,
};

static struct platform_driver displayport_driver __refdata = {
	.probe			= displayport_probe,
	.remove			= displayport_remove,
	.shutdown		= displayport_shutdown,
	.driver = {
		.name		= DISPLAYPORT_MODULE_NAME,
		.owner		= THIS_MODULE,
		.pm		= &displayport_pm_ops,
		.of_match_table	= of_match_ptr(displayport_of_match),
	}
};

static int __init displayport_init(void)
{
	int ret = platform_driver_register(&displayport_driver);

	if (ret)
		pr_err("displayport driver register failed\n");

	return ret;
}
late_initcall(displayport_init);

static void __exit displayport_exit(void)
{
	platform_driver_unregister(&displayport_driver);
}

module_exit(displayport_exit);
MODULE_AUTHOR("Kwangje Kim <kj1.kim@samsung.com>");
MODULE_DESCRIPTION("Samusung EXYNOS DisplayPort driver");
MODULE_LICENSE("GPL");
