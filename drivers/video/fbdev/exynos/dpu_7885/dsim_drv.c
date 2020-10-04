/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung SoC MIPI-DSIM driver.
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
#include <soc/samsung/cal-if.h>
#include <dt-bindings/clock/exynos7885.h>

#include "decon.h"
#include "dsim.h"
#include "regs-dsim.h"

#include "decon_board.h"
#include "panels/dsim_panel.h"
#include "panels/dd.h"

/* #define BRINGUP_DSIM_BIST */
/* #define PHY_FRAMEWORK_DIS */

#if defined(PHY_FRAMEWORK_DIS)
	void __iomem *dphy_isolation;
#endif

int dsim_log_level = 6;

struct dsim_device *dsim_drvdata[MAX_DSIM_CNT];
EXPORT_SYMBOL(dsim_drvdata);

static int dsim_runtime_suspend(struct device *dev);
static int dsim_runtime_resume(struct device *dev);

static void __dsim_dump(struct dsim_device *dsim)
{
	/* change to updated register read mode (meaning: SHADOW in DECON) */
	dsim_reg_enable_shadow_read(dsim->id, 0);
	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_ADDRESS, 32, 4,
			dsim->res.regs, 0xFC, false);
	/* restore to avoid size mismatch (possible config error at DECON) */
	dsim_reg_enable_shadow_read(dsim->id, 1);
}

static void dsim_dump(struct dsim_device *dsim)
{
	dsim_info("=== DSIM SFR DUMP ===\n");
	__dsim_dump(dsim);

	/* Show panel status */
	/* call_panel_ops(dsim, dump, dsim); */
}

static void dsim_long_data_wr(struct dsim_device *dsim, unsigned long d0, u32 d1)
{
	unsigned int data_cnt = 0, payload = 0;

	/* in case that data count is more then 4 */
	for (data_cnt = 0; data_cnt < d1; data_cnt += 4) {
		/*
		 * after sending 4bytes per one time,
		 * send remainder data less then 4.
		 */
		if ((d1 - data_cnt) < 4) {
			if ((d1 - data_cnt) == 3) {
				payload = *(u8 *)(d0 + data_cnt) |
				    (*(u8 *)(d0 + (data_cnt + 1))) << 8 |
					(*(u8 *)(d0 + (data_cnt + 2))) << 16;
			dsim_dbg("count = 3 payload = %x, %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)));
			} else if ((d1 - data_cnt) == 2) {
				payload = *(u8 *)(d0 + data_cnt) |
					(*(u8 *)(d0 + (data_cnt + 1))) << 8;
			dsim_dbg("count = 2 payload = %x, %x %x\n", payload,
				*(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)));
			} else if ((d1 - data_cnt) == 1) {
				payload = *(u8 *)(d0 + data_cnt);
			}

			dsim_reg_wr_tx_payload(dsim->id, payload);
		/* send 4bytes per one time. */
		} else {
			payload = *(u8 *)(d0 + data_cnt) |
				(*(u8 *)(d0 + (data_cnt + 1))) << 8 |
				(*(u8 *)(d0 + (data_cnt + 2))) << 16 |
				(*(u8 *)(d0 + (data_cnt + 3))) << 24;

			dsim_dbg("count = 4 payload = %x, %x %x %x %x\n",
				payload, *(u8 *)(d0 + data_cnt),
				*(u8 *)(d0 + (data_cnt + 1)),
				*(u8 *)(d0 + (data_cnt + 2)),
				*(u8 *)(d0 + (data_cnt + 3)));

			dsim_reg_wr_tx_payload(dsim->id, payload);
		}
	}
	dsim->pl_cnt += d1;
}

static int dsim_wait_for_cmd_fifo_empty(struct dsim_device *dsim, bool must_wait)
{
	int ret = 0;

	if (!must_wait) {
		/* timer is running, but already command is transferred */
		if (dsim_reg_header_fifo_is_empty(dsim->id))
			del_timer(&dsim->cmd_timer);

		dsim_dbg("%s Doesn't need to wait fifo_completion\n", __func__);
		goto exit;
	} else {
		del_timer(&dsim->cmd_timer);
		dsim_dbg("%s Waiting for fifo_completion...\n", __func__);
	}

	if (!wait_for_completion_timeout(&dsim->ph_wr_comp, MIPI_WR_TIMEOUT)) {
		if (dsim_reg_header_fifo_is_empty(dsim->id)) {
			reinit_completion(&dsim->ph_wr_comp);
			dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
			return 0;
		}
		ret = -ETIMEDOUT;
	}

	if ((dsim->state == DSIM_STATE_ON) && (ret == -ETIMEDOUT)) {
		dsim_err("%s have timed out\n", __func__);
		__dsim_dump(dsim);
		/*dsim_reg_set_fifo_ctrl(dsim->id, DSIM_FIFOCTRL_INIT_SFR);*/
	}

exit:
	return ret;
}

#define DSIM_FIFO_EMPTY_BUSY_WAIT 1
#define WRITE_TIME_OUT (50 * 1000)

/* wait for until SFR fifo is empty */
int dsim_wait_for_cmd_done(struct dsim_device *dsim)
{
	int ret = 0;
	/* FIXME: hiber only support for DECON0 */
	struct decon_device *decon = get_decon_drvdata(0);
#if defined(DSIM_FIFO_EMPTY_BUSY_WAIT)
	int delay_time = 10;
	int cnt = WRITE_TIME_OUT/ delay_time;
#endif

	decon_hiber_block_exit(decon);

	mutex_lock(&dsim->cmd_lock);
	ret = dsim_wait_for_cmd_fifo_empty(dsim, true);

#if defined(DSIM_FIFO_EMPTY_BUSY_WAIT)
	do {
		if (dsim_reg_header_fifo_is_empty(dsim->id))
			break;
		cnt--;
		udelay(delay_time);
	} while (cnt);
#endif

	mutex_unlock(&dsim->cmd_lock);

	decon_hiber_unblock(decon);

	return ret;
}

static bool dsim_is_writable_pl_fifo_status(struct dsim_device *dsim, u32 word_cnt)
{
	if ((DSIM_PL_FIFO_THRESHOLD - dsim->pl_cnt) > word_cnt)
		return true;
	else
		return false;
}

static bool dsim_is_fifo_empty_status(struct dsim_device *dsim)
{
	if (dsim_reg_header_fifo_is_empty(dsim->id) && dsim_reg_payload_fifo_is_empty(dsim->id)) {
		dsim->pl_cnt = 0;
		return true;
	} else
		return false;
}

int dsim_check_ph_threshold(struct dsim_device *dsim)
{
	int cnt = 5000;
	u32 available = 0;

	available = dsim_reg_is_writable_ph_fifo_state(dsim->id, &dsim->lcd_info);

	/* Wait FIFO empty status during 50ms */
	if (!available) {
		do {
			if (dsim_reg_header_fifo_is_empty(dsim->id))
				break;
			udelay(10);
			cnt--;
		} while (cnt);
	}
	return cnt;
}

int dsim_check_linecount(struct dsim_device *dsim)
{
	int cnt = 5000;
	bool fifo_empty = 0;
	int line_cnt = 0;

	dsim->line_cnt = dsim_reg_get_linecount(dsim->id, dsim->lcd_info.mode);
	if (dsim->line_cnt == 0) {
		do {
			fifo_empty = dsim_is_fifo_empty_status(dsim);
			line_cnt = dsim_reg_get_linecount(dsim->id, dsim->lcd_info.mode);
			if (fifo_empty || line_cnt)
				break;
			udelay(10);
			cnt--;
		} while (cnt);
	}
	return cnt;
}

int dsim_check_pl_threshold(struct dsim_device *dsim, u32 d1)
{
	int cnt = 5000;

	if (!dsim_is_writable_pl_fifo_status(dsim, d1)) {
		do {
			if (dsim_reg_payload_fifo_is_empty(dsim->id)) {
				dsim->pl_cnt = 0;
				break;
			}
			udelay(10);
			cnt--;
		} while (cnt);
	}

	return cnt;
}

int dsim_write_data(struct dsim_device *dsim, u32 id, unsigned long d0, u32 d1)
{
	int ret = 0;
	struct decon_device *decon = get_decon_drvdata(0);

	if (!dsim->priv.lcdconnected)
		return 0;

	decon_hiber_block_exit(decon);

	mutex_lock(&dsim->cmd_lock);
	if (dsim->state != DSIM_STATE_ON) {
		dsim_err("DSIM is not ready. state(%d)\n", dsim->state);
		ret = -EINVAL;
		goto err_exit;
	}
	DPU_EVENT_LOG_CMD(&dsim->sd, id, d0, d1);
	dsim_write_data_dump(dsim, id, d0, d1);

	reinit_completion(&dsim->ph_wr_comp);
	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);

	/* Check available status of PH FIFO before writing command */
	if (!dsim_check_ph_threshold(dsim)) {
		ret = -EINVAL;
		dsim_err("ID(%d): DSIM cmd wr timeout @ don't available ph 0x%lx\n", id, d0);
		goto err_exit;
	}

	/* Check linecount value for seperating idle and active range */
	if (!dsim_check_linecount(dsim)) {
		ret = -EINVAL;
		dsim_err("ID(%d): DSIM cmd wr timeout @ line count '0' 0x%lx, pl_cnt = %d\n", id, d0, dsim->pl_cnt);
		goto err_exit;
	}

	/* Run write-fail dectector */
	mod_timer(&dsim->cmd_timer, jiffies + MIPI_WR_TIMEOUT);

	switch (id) {
	/* short packet types of packet types for command. */
	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
	case MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE:
	case MIPI_DSI_DSC_PRA:
	case MIPI_DSI_COLOR_MODE_OFF:
	case MIPI_DSI_COLOR_MODE_ON:
	case MIPI_DSI_SHUTDOWN_PERIPHERAL:
	case MIPI_DSI_TURN_ON_PERIPHERAL:
		/* Enable packet go for blocking command transfer */
		dsim_reg_enable_packetgo(dsim->id, true);
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, false);
		break;

	case MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM:
	case MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM:
	case MIPI_DSI_DCS_READ:
		/* Enable packet go for blocking command transfer */
		dsim_reg_enable_packetgo(dsim->id, true);
		dsim_reg_wr_tx_header(dsim->id, id, d0, d1, true);
		break;

	/* long packet types of packet types for command. */
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
	case MIPI_DSI_DSC_PPS:
		if (d1 > DSIM_PL_FIFO_THRESHOLD) {
			dsim_err("Don't support payload size that exceeds 2048byte\n");
			ret = -EINVAL;
			goto err_exit;
		}
		if (!dsim_check_pl_threshold(dsim, d1)) {
			ret = -EINVAL;
			dsim_err("ID(%d): DSIM don't available pl 0x%lx\n, pl_cnt : %d, wc : %d",
					id, d0, dsim->pl_cnt, d1);
			goto err_exit;
		}

		/* Enable packet go for blocking command transfer */
		dsim_reg_enable_packetgo(dsim->id, true);
		dsim_long_data_wr(dsim, d0, d1);
		dsim_reg_wr_tx_header(dsim->id, id, d1 & 0xff,
				(d1 & 0xff00) >> 8, false);
		break;

	default:
		dsim_info("data id %x is not supported.\n", id);
		ret = -EINVAL;
	}

	dsim_reg_enable_packetgo(dsim->id, false);

err_exit:
	DPU_EVENT_LOG_CMD(&dsim->sd, id, d0, d1);
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;
}

static int dsim_wait_linecnt_is_safe_timeout(u32 id,
				unsigned long timeout, unsigned int limit)
{
	unsigned long delay_time = 10;
	unsigned long cnt = timeout / delay_time;
	u32 val, status, linecnt;

	do {
		val = dsim_read(id, DSIM_LINK_STATUS0);
		status = DSIM_LINK_STATUS0_VIDEO_MODE_STATUS_GET(val);
		linecnt = DSIM_LINK_STATUS0_VM_LINE_CNT_GET(val);

		if (!status || linecnt < limit)
			break;
		cnt--;
		udelay(delay_time);
	} while (cnt);

	if (!cnt)
		dsim_err("wait timeout linecount is safe(%u)\n", val);

	return 0;
}

int dsim_read_data(struct dsim_device *dsim, u32 id, u32 addr, u32 cnt, u8 *buf)
{
	u32 rx_fifo, rx_size = 0;
	int i, j, ret = 0;
	u32 rx_fifo_depth = DSIM_RX_FIFO_MAX_DEPTH;
	struct decon_device *decon = get_decon_drvdata(0);

	if (!dsim->priv.lcdconnected)
		return 0;

	decon_hiber_block_exit(decon);

	if (dsim->state != DSIM_STATE_ON) {
		dsim_err("DSIM is not ready. state(%d)\n", dsim->state);
		decon_hiber_unblock(decon);
		return -EINVAL;
	}

	reinit_completion(&dsim->rd_comp);

	/* Init RX FIFO before read and clear DSIM_INTSRC */
	/*dsim_reg_set_fifo_ctrl(dsim->id, DSIM_FIFOCTRL_INIT_RX);*/

	dsim_reg_clear_int(dsim->id, DSIM_INTSRC_RX_DATA_DONE);

	/* Set the maximum packet size returned */
	dsim_write_data(dsim,
		MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE, cnt, 0);

	/* Read request will be sent at safe region */
	if (dsim->lcd_info.mode == DECON_VIDEO_MODE)
		dsim_wait_linecnt_is_safe_timeout(dsim->id, 35 * 1000, (dsim->lcd_info.yres >> 1));

	/* Read request */
	dsim_write_data(dsim, id, addr, 0);
	dsim_wait_for_cmd_done(dsim);
	if (!wait_for_completion_timeout(&dsim->rd_comp, MIPI_RD_TIMEOUT)) {
		dsim_err("MIPI DSIM read Timeout! %2X, %2X, %d\n", id, addr, cnt);
		return -ETIMEDOUT;
	}

	mutex_lock(&dsim->cmd_lock);
	DPU_EVENT_LOG_CMD(&dsim->sd, id, (char)addr, 0);

	do {
		rx_fifo = dsim_reg_get_rx_fifo(dsim->id);

		/* Parse the RX packet data types */
		switch (rx_fifo & 0xff) {
		case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
			ret = dsim_reg_rx_err_handler(dsim->id, rx_fifo);
			if (ret < 0) {
				__dsim_dump(dsim);
				goto exit;
			}
			break;
		case MIPI_DSI_RX_END_OF_TRANSMISSION:
			dsim_dbg("EoTp was received from LCD module.\n");
			break;
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
		case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
			dsim_dbg("Short Packet was received from LCD module.\n");
			for (i = 0; i < cnt; i++)
				buf[i] = (rx_fifo >> (8 + i * 8)) & 0xff;
			rx_size = cnt;
			break;
		case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
			dsim_dbg("Long Packet was received from LCD module.\n");
			rx_size = (rx_fifo & 0x00ffff00) >> 8;
			dsim_dbg("rx fifo : %8x, response : %x, rx_size : %d\n",
					rx_fifo, rx_fifo & 0xff, rx_size);
			if (rx_size > cnt) {
				dsim_err("rx size is invalid, rx_size: %d, cnt: %d\n", rx_size, cnt);
				rx_size = cnt;
			}
			/* Read data from RX packet payload */
			for (i = 0; i < rx_size >> 2; i++) {
				rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
				for (j = 0; j < 4; j++)
					buf[(i*4)+j] = (u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			if (rx_size % 4) {
				rx_fifo = dsim_reg_get_rx_fifo(dsim->id);
				for (j = 0; j < rx_size % 4; j++)
					buf[4 * i + j] =
						(u8)(rx_fifo >> (j * 8)) & 0xff;
			}
			break;
		default:
			dsim_err("Packet format is invaild. %x\n", rx_fifo);
			ret = -EBUSY;
			break;
		}
	} while (!dsim_reg_rx_fifo_is_empty(dsim->id) && --rx_fifo_depth);

	ret = rx_size;
	if (!rx_fifo_depth) {
		dsim_err("Check DPHY values about HS clk.\n");
		__dsim_dump(dsim);
		ret = -EBUSY;
	}
exit:
	mutex_unlock(&dsim->cmd_lock);
	decon_hiber_unblock(decon);

	return ret;
}

static void dsim_cmd_fail_detector(unsigned long arg)
{
	struct dsim_device *dsim = (struct dsim_device *)arg;
	struct decon_device *decon = get_decon_drvdata(0);

	decon_hiber_block(decon);

	dsim_dbg("%s +\n", __func__);
	if (dsim->state != DSIM_STATE_ON) {
		dsim_err("%s: DSIM is not ready. state(%d)\n", __func__,
				dsim->state);
		goto exit;
	}

	/* If already FIFO empty even though the timer is no pending */
	if (!timer_pending(&dsim->cmd_timer)
			&& dsim_reg_header_fifo_is_empty(dsim->id)) {
		reinit_completion(&dsim->ph_wr_comp);
		dsim_reg_clear_int(dsim->id, DSIM_INTSRC_SFR_PH_FIFO_EMPTY);
		goto exit;
	}

	__dsim_dump(dsim);
	dsim->cmd_timeout_cnt++;
exit:
	decon_hiber_unblock(decon);
	dsim_dbg("%s -\n", __func__);
}

#if defined(CONFIG_PM_DEVFREQ)
static void dsim_bts_print_info(struct bts_decon_info *info)
{
	int i;

	for (i = 0; i < BTS_DPP_MAX; ++i) {
		if (!info->dpp[i].used)
			continue;

		dsim_info("\t\tDPP[%d] (%d) (%4d %4d) (%4d %4d %4d %4d)\n",
				info->dpp[i].idma_type, info->dpp[i].bpp,
				info->dpp[i].src_w, info->dpp[i].src_h,
				info->dpp[i].dst.x1, info->dpp[i].dst.x2,
				info->dpp[i].dst.y1, info->dpp[i].dst.y2);
	}
}
#endif
static void dsim_underrun_info(struct dsim_device *dsim)
{
#if defined(CONFIG_PM_DEVFREQ)
	struct decon_device *decon = get_decon_drvdata(0);
	unsigned long mif = 0, iint = 0, disp = 0;

	if (decon == NULL)
		return;

	dsim_info("\tMIF(%lu), INT(%lu), DISP(%lu)\n",
			mif = cal_dfs_get_rate(ACPM_DVFS_MIF),
			iint = cal_dfs_get_rate(ACPM_DVFS_INT),
			disp = cal_dfs_get_rate(ACPM_DVFS_DISP));

	if (decon) {
		dsim_info("\tDECON%d: total(%u %u), max(%u %u), peak(%u)\n",
				decon->id,
				decon->bts.prev_total_bw,
				decon->bts.total_bw,
				decon->bts.prev_max_disp_freq,
				decon->bts.max_disp_freq,
				decon->bts.peak);
		dsim_bts_print_info(&decon->bts.bts_info);

		decon_abd_save_udr(&decon->abd, mif, iint, disp);
	}
#endif
}

static irqreturn_t dsim_irq_handler(int irq, void *dev_id)
{
	unsigned int int_src;
	struct dsim_device *dsim = dev_id;
	struct decon_device *decon = get_decon_drvdata(0);
#if defined(CONFIG_PM)
	int active;
#endif
	spin_lock(&dsim->slock);

#if defined(CONFIG_PM)
	active = pm_runtime_active(dsim->dev);
	if (!active) {
		dsim_info("dsim power(%d), state(%d)\n", active, dsim->state);
		spin_unlock(&dsim->slock);
		return IRQ_HANDLED;
	}
#endif
	int_src = readl(dsim->res.regs + DSIM_INTSRC);

	if (int_src & DSIM_INTSRC_SFR_PL_FIFO_EMPTY) {
		dsim->pl_cnt = 0;
		DPU_EVENT_LOG(DPU_EVT_DSIM_PL_FIFO_EMPTY, &dsim->sd, ktime_set(0, 0));
	}
	if (int_src & DSIM_INTSRC_SFR_PH_FIFO_EMPTY) {
		del_timer(&dsim->cmd_timer);
		complete(&dsim->ph_wr_comp);
		DPU_EVENT_LOG(DPU_EVT_DSIM_PH_FIFO_EMPTY, &dsim->sd, ktime_set(0, 0));
		dsim_dbg("dsim%d PH_FIFO_EMPTY irq occurs\n", dsim->id);
	}
	if (int_src & DSIM_INTSRC_RX_DATA_DONE)
		complete(&dsim->rd_comp);
	if (int_src & DSIM_INTSRC_FRAME_DONE) {
		dsim->continuous_underrun_cnt = 0;
		dsim_dbg("dsim%d framedone irq occurs\n", dsim->id);
	}
	if (int_src & DSIM_INTSRC_ERR_RX_ECC) {
		if (decon) {
			decon->frm_status |= DPU_FRM_RX_ECC;
		}
		dsim_err("RX ECC Multibit error was detected!\n");
	}
	if (int_src & DSIM_INTSRC_UNDER_RUN) {
		if (decon) {
			decon->frm_status |= DPU_FRM_UNDERRUN;
		}
		dsim->total_underrun_cnt++;
		dsim->continuous_underrun_cnt++;
		dsim_info("dsim%d underrun irq occurs(%d) (%d)\n", dsim->id,
				dsim->total_underrun_cnt, dsim->continuous_underrun_cnt);

		dsim_underrun_info(dsim);
		if (dsim->lcd_info.mode == DECON_VIDEO_MODE) {
			if (decon && dsim->continuous_underrun_max &&
				(dsim->continuous_underrun_cnt >= dsim->continuous_underrun_max)) {
				decon_dump(decon);
				BUG();
			}
			__dsim_dump(dsim);
		}
	}
	if (int_src & DSIM_INTSRC_VT_STATUS) {
		dsim_dbg("dsim%d vt_status(vsync) irq occurs\n", dsim->id);
		if (decon) {
			decon->vsync.timestamp = ktime_get();
			wake_up_interruptible_all(&decon->vsync.wait);
		}
	}

	dsim_reg_clear_int(dsim->id, int_src);

	spin_unlock(&dsim->slock);

	return IRQ_HANDLED;
}

static void dsim_clocks_info(struct dsim_device *dsim)
{
}

static int dsim_get_clocks(struct dsim_device *dsim)
{
	return 0;
}

static int dsim_reset_panel(struct dsim_device *dsim)
{
	dsim_info("%s\n", __func__);

	run_list(dsim->dev, __func__);

	call_panel_ops(dsim, after_reset, dsim);

	return 0;
}

int dsim_set_panel_power_early(struct dsim_device *dsim)
{
	dsim_info("%s +\n", __func__);

	run_list(dsim->dev, __func__);

	dsim_info("%s -\n", __func__);

	return 0;
}

static int dsim_set_panel_power(struct dsim_device *dsim, bool on)
{
	dsim_info("%s: %d\n", __func__, on);

	if (on)
		run_list(dsim->dev, "dsim_set_panel_power_enable");
	else
		run_list(dsim->dev, "dsim_set_panel_power_disable");

	return 0;
}

int dsim_runtime_reset(struct dsim_device *dsim)
{
	int ret = 0;

	if (dsim->state == DSIM_STATE_OFF)
		return 0;

	dsim->runtime_reset_cnt++;

	dsim_info("%s:(%d) +\n", __func__, dsim->runtime_reset_cnt);

	dsim_reg_runtime_stop(dsim->id, dsim->data_lane);

	dsim_reg_set_clocks(dsim->id, &dsim->clks, &dsim->lcd_info.dphy_pms, 1);

	dsim_reg_set_lanes(dsim->id, dsim->data_lane, 1);

	dsim_reg_set_esc_clk_on_lane(dsim->id, 1, dsim->data_lane);
	dsim_reg_enable_word_clock(dsim->id, 1);

	if (dsim_reg_init(dsim->id, &dsim->lcd_info, dsim->data_lane_cnt,
			&dsim->clks) < 0) {
		dsim_info("dsim_%d already enabled", dsim->id);
		ret = -EBUSY;
	} else {
		dsim_info("dsim_%d enabled", dsim->id);
		/* Panel reset should be set after LP-11 */
		dsim_reset_panel(dsim);
	}

	dsim_reg_start(dsim->id);
	dsim->state = DSIM_STATE_ON;

	dsim_info("%s -\n", __func__);

	return ret;
}

#if defined(PHY_FRAMEWORK_DIS)
void dphy_power_on(struct dsim_device *dsim, int on)
{
	int val;

	if (on) {
		/* for DPHY isolation release */
		dphy_isolation = ioremap(0x11c80678, 0x4);
		val = (0x1<<0);
		writel(val, dphy_isolation);
	} else {
		/* for DPHY isolation enable */
		val = (0x0<<0);
		writel(val, dphy_isolation);
		iounmap(dphy_isolation);
	}
}
#else
void dphy_power_on(struct dsim_device *dsim, int on)
{
	int ret = 0;

	if (on)
		ret = phy_power_on(dsim->phy);
	else
		ret = phy_power_off(dsim->phy);

	if (ret < 0)
		dsim_err("%s: err: %d\n", __func__, ret);
}
#endif

static void __iomem *dispaud;	/* 0x11C84020 */
static void __iomem *dphydsi;	/* 0x11C80678 */
static int dsim_enable(struct dsim_device *dsim)
{
	int ret = 0;

	enum dsim_state state = dsim->state;

	if (dsim->state == DSIM_STATE_ON) {
#if defined(CONFIG_EXYNOS_DOZE)
		if (IS_DOZE(dsim->doze_state))
			call_panel_ops(dsim, displayon, dsim);
#endif
		goto exit;
	}

	dsim_info("+ %s\n", __func__);

#if defined(CONFIG_PM)
	pm_runtime_get_sync(dsim->dev);
#else
	dsim_runtime_resume(dsim->dev);
#endif

	dpu_sysreg_set_lpmux(dsim->res.ss_regs);

	dphy_power_on(dsim, 1);

	dsim_set_panel_power_early(dsim);

	if (state != DSIM_STATE_INIT)
		call_panel_ops(dsim, resume_early, dsim);

	/* Panel power on */
#if defined(CONFIG_EXYNOS_DOZE)
	if (IS_DOZE(dsim->doze_state))
		dsim_info("%s: exit_doze\n", __func__);
	else
		dsim_set_panel_power(dsim, 1);
#else
	dsim_set_panel_power(dsim, 1);
#endif

	if (!dispaud) {
		dispaud = ioremap(0x11C84020, SZ_4);
		dphydsi = ioremap(0x11C80678, SZ_4);
	}

	mutex_lock(&dsim->phy->mutex);
	dsim_info("%s: power_count: %d, disable_depth: %d, direct_complete: %d\n", __func__,
		dsim->phy->power_count, dsim->phy->dev.power.disable_depth, dsim->phy->dev.power.direct_complete);
	dsim_info("%s: usage_count: %d, dispaud: %X, dphydsi: %X\n", __func__,
		atomic_read(&dsim->dev->power.usage_count), readl(dispaud), readl(dphydsi));
	mutex_unlock(&dsim->phy->mutex);

	/* check whether the bootloader init has been done */
	if (dsim->state == DSIM_STATE_INIT) {
		if (dsim_reg_is_pll_stable(dsim->id)) {
			dsim_info("dsim%d PLL is stabled in bootloader, so skip DSIM link/DPHY init.\n", dsim->id);
			goto init_end;
		}
	}

	/* Do DPHY reset */
	dsim_reg_dphy_reset(dsim->id);

	dsim_reg_sw_reset(dsim->id);

	dsim_reg_set_clocks(dsim->id, &dsim->clks, &dsim->lcd_info.dphy_pms, 1);

	dsim_reg_set_lanes(dsim->id, dsim->data_lane, 1);

	dsim_reg_set_esc_clk_on_lane(dsim->id, 1, dsim->data_lane);
	dsim_reg_enable_word_clock(dsim->id, 1);

	if (dsim_reg_init(dsim->id, &dsim->lcd_info, dsim->data_lane_cnt,
				&dsim->clks) < 0) {
		dsim_info("dsim_%d already enabled", dsim->id);
		enable_irq(dsim->res.irq);
		ret = -EBUSY;
	} else {

		dsim_info("dsim_%d enabled", dsim->id);
		/* Panel reset should be set after LP-11 */
#if defined(CONFIG_EXYNOS_DOZE)
		if (IS_DOZE(dsim->doze_state))
			dsim_info("%s: exit_doze\n", __func__);
		else
			dsim_reset_panel(dsim);
#else
		dsim_reset_panel(dsim);
#endif
	}

init_end:
	dsim_reg_start(dsim->id);

	dsim->state = DSIM_STATE_ON;

	enable_irq(dsim->res.irq);

	if (state != DSIM_STATE_INIT) {
		dsim_info("Panel init commands are transfered\n");
		call_panel_ops(dsim, displayon, dsim);
	}

exit:
#if defined(CONFIG_EXYNOS_DOZE)
	dsim->doze_state = DOZE_STATE_NORMAL;
#endif
	dsim_info("- %s\n", __func__);

	return ret;
}

static int dsim_disable(struct dsim_device *dsim)
{
	if (dsim->state == DSIM_STATE_OFF)
		goto exit;

	dsim_info("+ %s\n", __func__);

	if (dsim->lcd_info.mode != DECON_VIDEO_MODE)
		call_panel_ops(dsim, suspend, dsim);

#if defined(CONFIG_EXYNOS_DOZE)
	dsim->doze_state = DOZE_STATE_SUSPEND;
#endif

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	del_timer(&dsim->cmd_timer);
	dsim->state = DSIM_STATE_OFF;
	mutex_unlock(&dsim->cmd_lock);

	disable_irq(dsim->res.irq);
	dsim_reg_stop(dsim->id, dsim->data_lane);

	dphy_power_on(dsim, 0);

	dsim_set_panel_power(dsim, 0);

#if defined(CONFIG_PM)
	pm_runtime_put_sync(dsim->dev);
#else
	dsim_runtime_suspend(dsim->dev);
#endif

exit:
	dsim_info("- %s\n", __func__);

	return 0;
}

static int dsim_enter_ulps(struct dsim_device *dsim)
{
	int ret = 0;

	DPU_EVENT_START();
	dsim_dbg("%s +\n", __func__);
	exynos_ss_printk("%s:state %d: active %d:+\n", __func__,
				dsim->state, pm_runtime_active(dsim->dev));

	if (dsim->state != DSIM_STATE_ON) {
		ret = -EBUSY;
		goto err;
	}

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	dsim->state = DSIM_STATE_ULPS;
	mutex_unlock(&dsim->cmd_lock);

	disable_irq(dsim->res.irq);

	/* disable interrupts */
	dsim_reg_set_int(dsim->id, 0);

	ret = dsim_reg_stop_and_enter_ulps(dsim->id, dsim->lcd_info.ddi_type,
			dsim->data_lane);
	if (ret < 0)
		dsim_dump(dsim);

	dphy_power_on(dsim, 0);

#if defined(CONFIG_PM)
	pm_runtime_put_sync(dsim->dev);
#else
	dsim_runtime_suspend(dsim->dev);
#endif

	DPU_EVENT_LOG(DPU_EVT_ENTER_ULPS, &dsim->sd, start);
err:
	dsim_dbg("%s -\n", __func__);
	exynos_ss_printk("%s:state %d: active %d:-\n", __func__,
				dsim->state, pm_runtime_active(dsim->dev));

	return ret;
}

static int dsim_exit_ulps(struct dsim_device *dsim)
{
	int ret = 0;

	DPU_EVENT_START();
	dsim_dbg("%s +\n", __func__);
	exynos_ss_printk("%s:state %d: active %d:+\n", __func__,
				dsim->state, pm_runtime_active(dsim->dev));

	if (dsim->state != DSIM_STATE_ULPS) {
		ret = -EBUSY;
		goto err;
	}

#if defined(CONFIG_PM)
	pm_runtime_get_sync(dsim->dev);
#else
	dsim_runtime_resume(dsim->dev);
#endif

	dpu_sysreg_set_lpmux(dsim->res.ss_regs);

	dphy_power_on(dsim, 1);

	dsim_reg_dphy_reset(dsim->id);
	dsim_reg_sw_reset(dsim->id);

	dsim_reg_set_clocks(dsim->id, &dsim->clks,
			&dsim->lcd_info.dphy_pms, 1);

	dsim_reg_set_lanes(dsim->id, dsim->data_lane, 1);

	dsim_reg_set_esc_clk_on_lane(dsim->id, 1, dsim->data_lane);
	dsim_reg_enable_word_clock(dsim->id, 1);

	if (dsim_reg_init(dsim->id, &dsim->lcd_info, dsim->data_lane_cnt,
				&dsim->clks) < 0) {
		dsim_info("dsim_%d already enabled", dsim->id);
		enable_irq(dsim->res.irq);
		return -EBUSY;
	}

	ret = dsim_reg_exit_ulps_and_start(dsim->id, dsim->lcd_info.ddi_type,
			dsim->data_lane);
	if (ret < 0)
		dsim_dump(dsim);

	dsim->state = DSIM_STATE_ON;

	enable_irq(dsim->res.irq);

	DPU_EVENT_LOG(DPU_EVT_EXIT_ULPS, &dsim->sd, start);
err:
	dsim_dbg("%s -\n", __func__);
	exynos_ss_printk("%s:state %d: active %d:-\n", __func__,
				dsim->state, pm_runtime_active(dsim->dev));

	return 0;
}

static int dsim_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);

	if (enable)
		return dsim_enable(dsim);
	else
		return dsim_disable(dsim);
}

static long dsim_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct dsim_device *dsim = container_of(sd, struct dsim_device, sd);
	int ret = 0;

	switch (cmd) {
	case DSIM_IOC_GET_LCD_INFO:
		v4l2_set_subdev_hostdata(sd, &dsim->lcd_info);
		break;

	case DSIM_IOC_ENTER_ULPS:
		if ((unsigned long)arg)
			ret = dsim_enter_ulps(dsim);
		else
			ret = dsim_exit_ulps(dsim);
		break;

	case DSIM_IOC_DUMP:
		dsim_dump(dsim);
		break;

	case DSIM_IOC_GET_WCLK:
		v4l2_set_subdev_hostdata(sd, &dsim->clks.word_clk);
		break;
#if defined(CONFIG_EXYNOS_DOZE)
	case DSIM_IOC_DOZE:
		ret = dsim_doze(dsim);
		if (ret)
			dsim_err("DSIM:ERR:%s:failed to doze\n", __func__);
		break;

	case DSIM_IOC_DOZE_SUSPEND:
		ret = dsim_doze_suspend(dsim);
		if (ret)
			dsim_err("DSIM:ERR:%s:failed to doze suspend\n", __func__);
		break;
#endif
	default:
		dsim_err("unsupported ioctl");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_subdev_core_ops dsim_sd_core_ops = {
	.ioctl = dsim_ioctl,
};

static const struct v4l2_subdev_video_ops dsim_sd_video_ops = {
	.s_stream = dsim_s_stream,
};

static const struct v4l2_subdev_ops dsim_subdev_ops = {
	.core = &dsim_sd_core_ops,
	.video = &dsim_sd_video_ops,
};

static void dsim_init_subdev(struct dsim_device *dsim)
{
	struct v4l2_subdev *sd = &dsim->sd;

	v4l2_subdev_init(sd, &dsim_subdev_ops);
	sd->owner = THIS_MODULE;
	sd->grp_id = dsim->id;
	snprintf(sd->name, sizeof(sd->name), "%s.%d", "dsim-sd", dsim->id);
	v4l2_set_subdevdata(sd, dsim);
}

static int dsim_cmd_sysfs_write(struct dsim_device *dsim, bool on)
{
	int ret = 0;

	if (on)
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE,
			MIPI_DCS_SET_DISPLAY_ON, 0);
	else
		ret = dsim_write_data(dsim, MIPI_DSI_DCS_SHORT_WRITE,
			MIPI_DCS_SET_DISPLAY_OFF, 0);
	if (ret < 0)
		dsim_err("Failed to write test data!\n");
	else
		dsim_dbg("Succeeded to write test data!\n");

	return ret;
}

static int dsim_cmd_sysfs_read(struct dsim_device *dsim)
{
	int ret = 0;
	unsigned int id;
	u8 buf[4];

	/* dsim sends the request for the lcd id and gets it buffer */
	ret = dsim_read_data(dsim, MIPI_DSI_DCS_READ,
		MIPI_DCS_GET_DISPLAY_ID, DSIM_DDI_ID_LEN, buf);
	id = *(unsigned int *)buf;
	if (ret < 0)
		dsim_err("Failed to read panel id!\n");
	else
		dsim_info("Suceeded to read panel id : 0x%08x\n", id);

	return ret;
}

static ssize_t dsim_cmd_sysfs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t dsim_cmd_sysfs_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long cmd;
	struct dsim_device *dsim = dev_get_drvdata(dev);

	ret = kstrtoul(buf, 0, &cmd);
	if (ret)
		return ret;

	switch (cmd) {
	case 1:
		ret = dsim_cmd_sysfs_read(dsim);
		call_panel_ops(dsim, dump, dsim);
		if (ret)
			return ret;
		break;
	case 2:
		ret = dsim_cmd_sysfs_write(dsim, true);
		dsim_info("Dsim write command, display on!!\n");
		if (ret)
			return ret;
		break;
	case 3:
		ret = dsim_cmd_sysfs_write(dsim, false);
		dsim_info("Dsim write command, display off!!\n");
		if (ret)
			return ret;
		break;
	default:
		dsim_info("unsupportable command\n");
		break;
	}

	return count;
}
static DEVICE_ATTR(cmd_rw, 0644, dsim_cmd_sysfs_show, dsim_cmd_sysfs_store);

int dsim_create_cmd_rw_sysfs(struct dsim_device *dsim)
{
	int ret = 0;

	ret = device_create_file(dsim->dev, &dev_attr_cmd_rw);
	if (ret)
		dsim_err("failed to create command read & write sysfs\n");

	return ret;
}

static void dsim_parse_lcd_info(struct dsim_device *dsim)
{
	u32 res[3];
	struct device_node *node;

	node = of_parse_phandle(dsim->dev->of_node, "lcd_info", 0);

	of_property_read_u32(node, "mode", &dsim->lcd_info.mode);
	dsim_dbg("%s mode\n", dsim->lcd_info.mode ? "command" : "video");

	of_property_read_u32_array(node, "resolution", res, 2);
	dsim->lcd_info.xres = res[0];
	dsim->lcd_info.yres = res[1];
	dsim_info("LCD(%s) resolution: xres(%d), yres(%d)\n",
			of_node_full_name(node), res[0], res[1]);

	of_property_read_u32_array(node, "size", res, 2);
	dsim->lcd_info.width = res[0];
	dsim->lcd_info.height = res[1];
	dsim_dbg("LCD size: width(%d), height(%d)\n", res[0], res[1]);

	of_property_read_u32(node, "timing,refresh", &dsim->lcd_info.fps);
	dsim_dbg("LCD refresh rate(%d)\n", dsim->lcd_info.fps);

	of_property_read_u32_array(node, "timing,h-porch", res, 3);
	dsim->lcd_info.hbp = res[0];
	dsim->lcd_info.hfp = res[1];
	dsim->lcd_info.hsa = res[2];
	dsim_dbg("hbp(%d), hfp(%d), hsa(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32_array(node, "timing,v-porch", res, 3);
	dsim->lcd_info.vbp = res[0];
	dsim->lcd_info.vfp = res[1];
	dsim->lcd_info.vsa = res[2];
	dsim_dbg("vbp(%d), vfp(%d), vsa(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32(node, "timing,dsi-hs-clk", &dsim->lcd_info.hs_clk);
	dsim->clks.hs_clk = dsim->lcd_info.hs_clk;
	dsim_dbg("requested hs clock(%d)\n", dsim->lcd_info.hs_clk);

	of_property_read_u32_array(node, "timing,pms", res, 3);
	dsim->lcd_info.dphy_pms.p = res[0];
	dsim->lcd_info.dphy_pms.m = res[1];
	dsim->lcd_info.dphy_pms.s = res[2];
	dsim_dbg("p(%d), m(%d), s(%d)\n", res[0], res[1], res[2]);

	of_property_read_u32(node, "timing,dsi-escape-clk",
			&dsim->lcd_info.esc_clk);
	dsim->clks.esc_clk = dsim->lcd_info.esc_clk;
	dsim_dbg("requested escape clock(%d)\n", dsim->lcd_info.esc_clk);

	of_property_read_u32(node, "mic_en", &dsim->lcd_info.mic_enabled);
	dsim_info("mic enabled (%d)\n", dsim->lcd_info.mic_enabled);

	of_property_read_u32(node, "type_of_ddi", &dsim->lcd_info.ddi_type);
	dsim_dbg("ddi type(%d)\n", dsim->lcd_info.ddi_type);

	of_property_read_u32(node, "dsc_en", &dsim->lcd_info.dsc_enabled);
	dsim_info("dsc is %s\n", dsim->lcd_info.dsc_enabled ? "enabled" : "disabled");

	of_property_read_u32(node, "eotp_disabled", &dsim->lcd_info.eotp_disabled);
	dsim_info("eotp is %s\n", dsim->lcd_info.eotp_disabled ? "disabled" : "enabled");

	if (dsim->lcd_info.dsc_enabled) {
		of_property_read_u32(node, "dsc_cnt", &dsim->lcd_info.dsc_cnt);
		dsim_info("dsc count(%d)\n", dsim->lcd_info.dsc_cnt);
		of_property_read_u32(node, "dsc_slice_num",
				&dsim->lcd_info.dsc_slice_num);
		dsim_info("dsc slice count(%d)\n", dsim->lcd_info.dsc_slice_num);
		of_property_read_u32(node, "dsc_slice_h",
				&dsim->lcd_info.dsc_slice_h);
		dsim_info("dsc slice height(%d)\n", dsim->lcd_info.dsc_slice_h);
	}

	of_property_read_u32(node, "data_lane", &dsim->data_lane_cnt);
	dsim_info("using data lane count(%d)\n", dsim->data_lane_cnt);

	dsim->lcd_info.data_lane = dsim->data_lane_cnt;

	if (dsim->lcd_info.mode == DECON_MIPI_COMMAND_MODE) {
		of_property_read_u32(node, "cmd_underrun_lp_ref",
			&dsim->lcd_info.cmd_underrun_lp_ref);
		dsim_info("cmd_underrun_lp_ref(%d)\n", dsim->lcd_info.cmd_underrun_lp_ref);
	} else {
		of_property_read_u32(node, "vt_compensation",
			&dsim->lcd_info.vt_compensation);
		dsim_info("vt_compensation(%d)\n", dsim->lcd_info.vt_compensation);

		of_property_read_u32(node, "clklane_onoff", &dsim->lcd_info.clklane_onoff);
		dsim_info("clklane onoff(%d)\n", dsim->lcd_info.clklane_onoff);
	}

	if (IS_ENABLED(CONFIG_FB_WINDOW_UPDATE)) {
		if (!of_property_read_u32_array(node, "update_min", res, 2)) {
			dsim->lcd_info.update_min_w = res[0];
			dsim->lcd_info.update_min_h = res[1];
			dsim_info("update_min_w(%d) update_min_h(%d) \n",
				dsim->lcd_info.update_min_w, dsim->lcd_info.update_min_h);
		} else { /* If values are not difined on DT, Set to full size */
			dsim->lcd_info.update_min_w = dsim->lcd_info.xres;
			dsim->lcd_info.update_min_h = dsim->lcd_info.yres;
			dsim_info("ERR: no update_min in DT!! update_min_w(%d) update_min_h(%d)\n",
			dsim->lcd_info.update_min_w, dsim->lcd_info.update_min_h);
		}
	}
}

static int dsim_parse_dt(struct dsim_device *dsim, struct device *dev)
{
	if (IS_ERR_OR_NULL(dev->of_node)) {
		dsim_err("no device tree information\n");
		return -EINVAL;
	}

	dsim->id = of_alias_get_id(dev->of_node, "dsim");
	dsim_info("dsim(%d) probe start..\n", dsim->id);

#if !defined(PHY_FRAMEWORK_DIS)
	dsim->phy = devm_phy_get(dev, "dsim_dphy");
	if (IS_ERR_OR_NULL(dsim->phy)) {
		dsim_err("failed to get phy\n");
		return PTR_ERR(dsim->phy);
	}
#endif

	dsim->dev = dev;

	dsim_parse_lcd_info(dsim);

	return 0;
}

static void dsim_register_panel(struct dsim_device *dsim)
{
#if IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3HA2K)
	dsim->panel_ops = &s6e3ha2k_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3HF4)
	dsim->panel_ops = &s6e3hf4_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_EMUL_DISP)
	dsim->panel_ops = &emul_disp_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3FA0)
	dsim->panel_ops = &s6e3fa0_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3FA3)
	dsim->panel_ops = &s6e3fa3_mipi_lcd_driver;
#elif IS_ENABLED(CONFIG_EXYNOS_DECON_LCD_S6E3FA7)
	dsim->panel_ops = &s6e3fa7_mipi_lcd_driver;
#else
	dsim->panel_ops = mipi_lcd_driver;
#endif
}

static int dsim_get_data_lanes(struct dsim_device *dsim)
{
	int i;

	if (dsim->data_lane_cnt > MAX_DSIM_DATALANE_CNT) {
		dsim_err("%d data lane couldn't be supported\n",
				dsim->data_lane_cnt);
		return -EINVAL;
	}

	dsim->data_lane = DSIM_LANE_CLOCK;
	for (i = 1; i < dsim->data_lane_cnt + 1; ++i)
		dsim->data_lane |= 1 << i;

	dsim_info("%s: lanes(0x%x)\n", __func__, dsim->data_lane);

	return 0;
}

static int dsim_init_resources(struct dsim_device *dsim, struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dsim_err("failed to get mem resource\n");
		return -ENOENT;
	}
	dsim_info("res: start(0x%x), end(0x%x)\n", (u32)res->start, (u32)res->end);

	dsim->res.regs = devm_ioremap_resource(dsim->dev, res);
	if (!dsim->res.regs) {
		dsim_err("failed to remap DSIM SFR region\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dsim_err("failed to get irq resource\n");
		return -ENOENT;
	}

	dsim->res.irq = res->start;
	dsim_info("dsim irq (%d)\n", (u32)dsim->res.irq);
	ret = devm_request_irq(dsim->dev, res->start,
			dsim_irq_handler, IRQF_PERF_CRITICAL,
			pdev->name, dsim);
	if (ret) {
		dsim_err("failed to install DSIM irq\n");
		return -EINVAL;
	}
	disable_irq(dsim->res.irq);

	dsim->res.ss_regs = dpu_get_sysreg_addr();
	if (IS_ERR_OR_NULL(dsim->res.ss_regs)) {
		dsim_err("failed to get sysreg addr\n");
		return -EINVAL;
	}

	return 0;
}

static int dsim_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct dsim_device *dsim = NULL;

	dsim = devm_kzalloc(dev, sizeof(struct dsim_device), GFP_KERNEL);
	if (!dsim) {
		dsim_err("failed to allocate dsim device.\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = dsim_parse_dt(dsim, dev);
	if (ret)
		goto err_dt;

	dsim_drvdata[dsim->id] = dsim;
	ret = dsim_get_clocks(dsim);
	if (ret)
		goto err_dt;

	spin_lock_init(&dsim->slock);
	mutex_init(&dsim->cmd_lock);
	init_completion(&dsim->ph_wr_comp);
	init_completion(&dsim->rd_comp);

	ret = dsim_init_resources(dsim, pdev);
	if (ret)
		goto err_dt;

	dsim_init_subdev(dsim);
	platform_set_drvdata(pdev, dsim);
	dsim_register_panel(dsim);
	setup_timer(&dsim->cmd_timer, dsim_cmd_fail_detector,
			(unsigned long)dsim);

#if defined(CONFIG_PM)
	pm_runtime_enable(dev);
#endif

	ret = dsim_get_data_lanes(dsim);
	if (ret)
		goto err_dt;

#if !defined(PHY_FRAMEWORK_DIS)
	/* HACK */
	phy_init(dsim->phy);
#endif
	dsim->state = DSIM_STATE_INIT;
	dsim->continuous_underrun_max = 5;
	dsim_enable(dsim);

	/* TODO: If you want to enable DSIM BIST mode. you must turn on LCD here */

#if defined(BRINGUP_DSIM_BIST)
	call_panel_ops(dsim, displayon, dsim);
	/* TODO: This is for dsim BIST mode in zebu emulator. only for test*/
	dsim_set_bist(dsim->id, true);
#else
	call_panel_ops(dsim, probe, dsim);
#endif

	dsim_clocks_info(dsim);
	dsim_create_cmd_rw_sysfs(dsim);

	dsim_info("dsim%d driver(%s mode) has been probed.\n", dsim->id,
		dsim->lcd_info.mode == DECON_MIPI_COMMAND_MODE ? "cmd" : "video");

#if defined(BRINGUP_DSIM_BIST)
	/* dump dsim sfrs */
	dsim_dump(dsim);
#endif

	return 0;

err_dt:
	kfree(dsim);
err:
	return ret;
}

static int dsim_remove(struct platform_device *pdev)
{
	struct dsim_device *dsim = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	mutex_destroy(&dsim->cmd_lock);
	dsim_info("dsim%d driver removed\n", dsim->id);

	return 0;
}

static void dsim_shutdown(struct platform_device *pdev)
{
	struct dsim_device *dsim = platform_get_drvdata(pdev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_SHUTDOWN, &dsim->sd, ktime_set(0, 0));
	dsim_info("%s + state:%d\n", __func__, dsim->state);

	dsim_disable(dsim);

	dsim_info("%s -\n", __func__);
}

static int dsim_runtime_suspend(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_SUSPEND, &dsim->sd, ktime_set(0, 0));
	return 0;
}

static int dsim_runtime_resume(struct device *dev)
{
	struct dsim_device *dsim = dev_get_drvdata(dev);

	DPU_EVENT_LOG(DPU_EVT_DSIM_RESUME, &dsim->sd, ktime_set(0, 0));
	return 0;
}

static const struct of_device_id dsim_of_match[] = {
	{ .compatible = "samsung,exynos7885-dsim" },
	{},
};
MODULE_DEVICE_TABLE(of, dsim_of_match);

static const struct dev_pm_ops dsim_pm_ops = {
	.runtime_suspend	= dsim_runtime_suspend,
	.runtime_resume		= dsim_runtime_resume,
};

static struct platform_driver dsim_driver __refdata = {
	.probe			= dsim_probe,
	.remove			= dsim_remove,
	.shutdown		= dsim_shutdown,
	.driver = {
		.name		= DSIM_MODULE_NAME,
		.owner		= THIS_MODULE,
		.pm		= &dsim_pm_ops,
		.of_match_table	= of_match_ptr(dsim_of_match),
		.suppress_bind_attrs = true,
	}
};

static int __init dsim_init(void)
{
	int ret = platform_driver_register(&dsim_driver);

	if (ret)
		pr_err("dsim driver register failed\n");

	return ret;
}
late_initcall(dsim_init);

static void __exit dsim_exit(void)
{
	platform_driver_unregister(&dsim_driver);
}

module_exit(dsim_exit);
MODULE_AUTHOR("Yeongran Shin <yr613.shin@samsung.com>");
MODULE_AUTHOR("SeungBeom Park <sb1.park@samsung.com>");
MODULE_DESCRIPTION("Samusung EXYNOS DSIM driver");
MODULE_LICENSE("GPL");
