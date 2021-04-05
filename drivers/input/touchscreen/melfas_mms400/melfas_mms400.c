/*
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 */

#include "melfas_mms400.h"

#if MMS_USE_NAP_MODE
struct wake_lock mms_wake_lock;
#endif

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
#include <linux/trustedui.h>
#endif

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
extern int tui_force_close(uint32_t arg);
struct mms_ts_info *tui_tsp_info;
#endif
/**
 * Reboot chip
 *
 * Caution : IRQ must be disabled before mms_reboot and enabled after mms_reboot.
 */
void mms_reboot(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	input_dbg(true, &info->client->dev, "%s [START]\n", __func__);

	i2c_lock_adapter(adapter);

	mms_power_control(info, 0);
	mms_power_control(info, 1);

	i2c_unlock_adapter(adapter);

	msleep(30);

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);
}

/**
 * I2C Read
 */
int mms_i2c_read(struct mms_ts_info *info, char *write_buf, unsigned int write_len,
				char *read_buf, unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

	struct i2c_msg msg[] = {
		{
			.addr = info->client->addr,
			.flags = 0,
			.buf = write_buf,
			.len = write_len,
		}, {
			.addr = info->client->addr,
			.flags = I2C_M_RD,
			.buf = read_buf,
			.len = read_len,
		},
	};

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_INPUT_SECURED & trustedui_get_current_mode()) {
		input_err(&info->client->dev,
			"%s TSP no accessible from Linux, TUI is enabled!\n", __func__);
		return -EIO;
	}
#endif

	while (retry--) {
		res = i2c_transfer(info->client->adapter, msg, ARRAY_SIZE(msg));

		if (res == ARRAY_SIZE(msg)) {
			goto DONE;
		} else if (res < 0) {
			input_err(true, &info->client->dev,
				"%s [ERROR] i2c_transfer - errno[%d]\n", __func__, res);
			info->comm_err_count++;
		} else if (res != ARRAY_SIZE(msg)) {
			input_err(true, &info->client->dev,
				"%s [ERROR] i2c_transfer - result[%d]\n",
				__func__, res);
			info->comm_err_count++;
		} else {
			input_err(true, &info->client->dev,
				"%s [ERROR] unknown error [%d]\n", __func__, res);
			info->comm_err_count++;
		}
	}

	goto ERROR_REBOOT;

ERROR_REBOOT:
	mms_reboot(info);
	return 1;

DONE:
	return 0;
}


/**
 * I2C Read (Continue)
 */
int mms_i2c_read_next(struct mms_ts_info *info, char *read_buf, int start_idx,
				unsigned int read_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;
	u8 rbuf[read_len];

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_INPUT_SECURED & trustedui_get_current_mode()) {
		input_err(&info->client->dev,
			"%s TSP no accessible from Linux, TUI is enabled!\n", __func__);
		return -EIO;
	}
#endif

	while (retry--) {
		res = i2c_master_recv(info->client, rbuf, read_len);

		if (res == read_len) {
			goto DONE;
		} else if (res < 0) {
			input_err(true, &info->client->dev,
				"%s [ERROR] i2c_master_recv - errno [%d]\n", __func__, res);
			info->comm_err_count++;
		} else if (res != read_len) {
			input_err(true, &info->client->dev,
				"%s [ERROR] length mismatch - read[%d] result[%d]\n",
				__func__, read_len, res);
			info->comm_err_count++;
		} else {
			input_err(true, &info->client->dev,
				"%s [ERROR] unknown error [%d]\n", __func__, res);
			info->comm_err_count++;
		}
	}

	goto ERROR_REBOOT;

ERROR_REBOOT:
	mms_reboot(info);
	return 1;

DONE:
	memcpy(&read_buf[start_idx], rbuf, read_len);

	return 0;
}

/**
 * I2C Write
 */
int mms_i2c_write(struct mms_ts_info *info, char *write_buf, unsigned int write_len)
{
	int retry = I2C_RETRY_COUNT;
	int res;

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_INPUT_SECURED & trustedui_get_current_mode()) {
		input_err(&info->client->dev,
			"%s TSP no accessible from Linux, TUI is enabled!\n", __func__);
		return -EIO;
	}
#endif

	while (retry--) {
		res = i2c_master_send(info->client, write_buf, write_len);

		if (res == write_len) {
			goto DONE;
		} else if (res < 0) {
			input_err(true, &info->client->dev,
				"%s [ERROR] i2c_master_send - errno [%d]\n", __func__, res);
			info->comm_err_count++;
		} else if (res != write_len) {
			input_err(true, &info->client->dev,
				"%s [ERROR] length mismatch - write[%d] result[%d]\n",
				__func__, write_len, res);
			info->comm_err_count++;
		} else {
			input_err(true, &info->client->dev,
				"%s [ERROR] unknown error [%d]\n", __func__, res);
			info->comm_err_count++;
		}
	}

	goto ERROR_REBOOT;

ERROR_REBOOT:
	mms_reboot(info);
	return 1;

DONE:
	return 0;
}

/**
 * Enable device
 */
int mms_enable(struct mms_ts_info *info)
{
#ifdef COVER_MODE
	u8 wbuf[4];
#endif
	input_dbg(true, &info->client->dev, "%s [START]\n", __func__);

	if (info->enabled) {
		input_err(true, &info->client->dev,
			"%s : already enabled\n", __func__);
		return 0;
	}

	mutex_lock(&info->lock);

	if (!info->init)
		mms_power_control(info, 1);

	enable_irq(info->client->irq);
	info->enabled = true;

	mutex_unlock(&info->lock);

	if (info->disable_esd == true)
		mms_disable_esd_alert(info);

#ifdef CONFIG_VBUS_NOTIFIER
	if (info->ta_stsatus)
		mms_charger_attached(info, true);
#endif
#ifdef COVER_MODE
	if (info->cover_mode) {
		input_info(true, &info->client->dev, "%s clear_cover_mode on\n", __func__);

		wbuf[0] = MIP_R0_CTRL;
		wbuf[1] = MIP_R1_CTRL_WINDOW_MODE;
		wbuf[2] = 3;

		if (mms_i2c_write(info, wbuf, 3))
			input_err(true, &info->client->dev, "%s [ERROR] clear_cover_mode mms_i2c_write\n", __func__);
	}
#endif

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/**
 * Disable device
 */
int mms_disable(struct mms_ts_info *info)
{
	input_dbg(true, &info->client->dev, "%s [START]\n", __func__);

	if (!info->enabled) {
		input_err(true, &info->client->dev,
			"%s : already disabled\n", __func__);
		return 0;
	}

	mutex_lock(&info->lock);

	info->enabled = false;
	disable_irq(info->client->irq);
	mms_clear_input(info);
	mms_power_control(info, 0);

	mutex_unlock(&info->lock);

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

#if MMS_USE_INPUT_OPEN_CLOSE
/**
 * Open input device
 */
static int mms_input_open(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);

	if (!info->init) {
		if (!info->info_work_done) {
			input_err(true, &info->client->dev, "%s not finished info work\n", __func__);
			return 0;
		}

		input_info(true, &info->client->dev, "%s %s\n",
				__func__, info->lowpower_mode ? "exit LPM mode" : "");

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
		if (TRUSTEDUI_MODE_TUI_SESSION & trustedui_get_current_mode()) {
			input_err(&info->client->dev, "%s TUI cancel event call!\n", __func__);
			msleep(100);
			tui_force_close(1);
			msleep(200);
			if (TRUSTEDUI_MODE_TUI_SESSION & trustedui_get_current_mode()) {
				input_err(&info->client->dev, "%s TUI flag force clear!\n",	__func__);
				trustedui_clear_mask(TRUSTEDUI_MODE_VIDEO_SECURED|TRUSTEDUI_MODE_INPUT_SECURED);
				trustedui_set_mode(TRUSTEDUI_MODE_OFF);
			}
		}
#endif
		if (info->ic_status >= LPM_RESUME) {
			if (device_may_wakeup(&info->client->dev))
				disable_irq_wake(info->client->irq);

			disable_irq(info->client->irq);
			mms_reboot(info);
			enable_irq(info->client->irq);
#ifdef CONFIG_VBUS_NOTIFIER
			if (info->ta_stsatus)
				mms_charger_attached(info, true);
#endif
		} else {
			mms_enable(info);
		}
		info->ic_status = PWR_ON;
	}

	return 0;
}

/**
 * Close input device
 */
static void mms_input_close(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);

	if (!info->info_work_done) {
		input_err(true, &info->client->dev, "%s not finished info work\n", __func__);
		return;
	}

	input_info(true, &info->client->dev, "%s %s\n",
			__func__, info->lowpower_mode ? "enter LPM mode" : "");

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	if (TRUSTEDUI_MODE_TUI_SESSION & trustedui_get_current_mode()) {
		input_err(&info->client->dev, "%s TUI cancel event call!\n", __func__);
		msleep(100);
		tui_force_close(1);
		msleep(200);
		if (TRUSTEDUI_MODE_TUI_SESSION & trustedui_get_current_mode()) {
			input_err(&info->client->dev, "%s TUI flag force clear!\n",	__func__);
			trustedui_clear_mask(TRUSTEDUI_MODE_VIDEO_SECURED|TRUSTEDUI_MODE_INPUT_SECURED);
			trustedui_set_mode(TRUSTEDUI_MODE_OFF);
		}
	}
#endif

	if (info->lowpower_mode) {
		mms_lowpower_mode(info, 1);
		if (device_may_wakeup(&info->client->dev))
			enable_irq_wake(info->client->irq);
		mms_clear_input(info);
		info->ic_status = LPM_RESUME;
	} else {
		mms_disable(info);
		info->ic_status = PWR_OFF;
	}
}
#endif

/**
 * Get ready status
 */
int mms_get_ready_status(struct mms_ts_info *info)
{
	u8 wbuf[16];
	u8 rbuf[16];
	int ret = 0;

	input_dbg(true, &info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_READY_STATUS;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
		goto ERROR;
	}
	ret = rbuf[0];

	//check status
	if ((ret == MIP_CTRL_STATUS_NONE) || (ret == MIP_CTRL_STATUS_LOG)
		|| (ret == MIP_CTRL_STATUS_READY)) {
		input_info(true, &info->client->dev, "%s - status [0x%02X]\n", __func__, ret);
	} else{
		input_err(true, &info->client->dev,
			"%s [ERROR] Unknown status [0x%02X]\n", __func__, ret);
		goto ERROR;
	}

	if (ret == MIP_CTRL_STATUS_LOG) {
		//skip log event
		wbuf[0] = MIP_R0_LOG;
		wbuf[1] = MIP_R1_LOG_TRIGGER;
		wbuf[2] = 0;
		if (mms_i2c_write(info, wbuf, 3))
			input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
	}

	input_dbg(true, &info->client->dev, "%s [DONE]\n", __func__);
	return ret;

ERROR:
	input_err(true, &info->client->dev, "%s [ERROR]\n", __func__);
	return -1;
}

/**
 * Read chip firmware version
 */
int mms_get_fw_version(struct mms_ts_info *info, u8 *ver_buf)
{
	u8 rbuf[8];
	u8 wbuf[2];
	int i;

	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_VERSION_BOOT;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 8)) {
		goto ERROR;
	};

	for (i = 0; i < MMS_FW_MAX_SECT_NUM; i++) {
		ver_buf[0 + i * 2] = rbuf[1 + i * 2];
		ver_buf[1 + i * 2] = rbuf[0 + i * 2];
	}

	info->boot_ver_ic = ver_buf[1];
	info->core_ver_ic = ver_buf[3];
	info->config_ver_ic = ver_buf[5];

	input_info(true, &info->client->dev,
			"%s: boot:%x.%x core:%x.%x custom:%x.%d parameter:%x.%x\n",
			__func__, ver_buf[0], ver_buf[1], ver_buf[2], ver_buf[3], ver_buf[4],
			ver_buf[5], ver_buf[6], ver_buf[7]);

	return 0;

ERROR:
	input_err(true, &info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/**
 * Read chip firmware version for u16
 */
int mms_get_fw_version_u16(struct mms_ts_info *info, u16 *ver_buf_u16)
{
	u8 rbuf[8];
	int i;

	if (mms_get_fw_version(info, rbuf))
		goto ERROR;

	for (i = 0; i < MMS_FW_MAX_SECT_NUM; i++)
		ver_buf_u16[i] = (rbuf[0 + i * 2] << 8) | rbuf[1 + i * 2];

	return 0;

ERROR:
	input_err(true, &info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/**
 * Disable ESD alert
 */
int mms_disable_esd_alert(struct mms_ts_info *info)
{
	u8 wbuf[4];
	u8 rbuf[4];

	input_info(true, &info->client->dev, "%s [START]\n", __func__);

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_DISABLE_ESD_ALERT;
	wbuf[2] = 1;
	if (mms_i2c_write(info, wbuf, 3)) {
		input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
		goto ERROR;
	}

	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_read\n", __func__);
		goto ERROR;
	}

	if (rbuf[0] != 1) {
		input_info(true, &info->client->dev, "%s [ERROR] failed\n", __func__);
		goto ERROR;
	}

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	input_err(true, &info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

/**
 * Alert event handler - ESD
 */
static int mms_alert_handler_esd(struct mms_ts_info *info, u8 *rbuf)
{
	u8 frame_cnt = rbuf[1];

	input_info(true, &info->client->dev, "%s [START] - frame_cnt[%d]\n",
		__func__, frame_cnt);

	if (frame_cnt == 0) {
		//sensor crack, not ESD
		info->esd_cnt++;
		input_info(true, &info->client->dev, "%s - esd_cnt[%d]\n",
			__func__, info->esd_cnt);

		if (info->disable_esd == true) {
			mms_disable_esd_alert(info);
		} else if (info->esd_cnt > ESD_COUNT_FOR_DISABLE) {
			//Disable ESD alert
			if (mms_disable_esd_alert(info))
				input_err(true, &info->client->dev,
					"%s - fail to disable esd alert\n", __func__);
			else
				info->disable_esd = true;
		} else {
			//Reset chip
			mms_reboot(info);
		}
	} else {
		//ESD detected
		//Reset chip
		mms_reboot(info);
		info->esd_cnt = 0;
	}

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

/*
 * Alert event handler - SRAM failure
 */
static int mms_alert_handler_sram(struct mms_ts_info *info, u8 *data)
{
	int i;

	input_dbg(true, &info->client->dev, "%s [START]\n", __func__);

	info->sram_addr_num = (unsigned int) (data[0] | (data[1] << 8));
	input_info(true, &info->client->dev, "%s - sram_addr_num [%d]\n", __func__, info->sram_addr_num);

	if (info->sram_addr_num > 8) {
		input_err(true, &info->client->dev, "%s [ERROR] sram_addr_num [%d]\n",
						__func__, info->sram_addr_num);
		goto error;
	}

	for (i = 0; i < info->sram_addr_num; i++) {
		info->sram_addr[i] = data[2 + 4 * i] | (data[2 + 4 * i + 1] << 8) |
							(data[2 + 4 * i + 2] << 16) | (data[2 + 4 * i + 3] << 24);
		input_info(true, &info->client->dev, "%s - sram_addr #%d [0x%08X]\n",
						__func__, i, info->sram_addr[i]);
	}
	for (i = info->sram_addr_num; i < 8; i++) {
		info->sram_addr[i] = 0;
		input_info(true, &info->client->dev, "%s - sram_addr #%d [0x%08X]\n",
						__func__, i, info->sram_addr[i]);
	}

	input_dbg(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;

error:
	input_err(true, &info->client->dev, "%s [ERROR]\n", __func__);
	return 1;
}

#ifdef CONFIG_VBUS_NOTIFIER
int mms_charger_attached(struct mms_ts_info *info, bool status)
{
	u8 wbuf[4];

	input_info(true, &info->client->dev, "%s [START] %s\n", __func__, status ? "connected" : "disconnected");

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_CHARGER_MODE;
	wbuf[2] = status;

	if ((status == 0) || (status == 1)) {
		if (mms_i2c_write(info, wbuf, 3))
			input_err(true, &info->client->dev, "%s [ERROR] mms_i2c_write\n", __func__);
		else
			input_info(true, &info->client->dev, "%s - value[%d]\n", __func__, wbuf[2]);
	} else {
		input_err(true, &info->client->dev, "%s [ERROR] Unknown value[%d]\n", __func__, status);
	}
	input_dbg(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}
#endif

/**
 * Interrupt handler
 */
static irqreturn_t mms_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 wbuf[8];
	u8 rbuf[256];
	unsigned int size = 0;
//	int event_size = info->event_size;
	u8 category = 0;
	u8 alert_type = 0;

	if (info->lowpower_mode)
		pm_wakeup_event(info->input_dev->dev.parent, 1000);

	input_dbg(true, &client->dev, "%s [START]\n", __func__);

	// AOT function
	if (info->lowpower_mode && info->ic_status >= LPM_RESUME) {
		wbuf[0] = MIP_R0_AOT;
		wbuf[1] = MIP_R0_AOT_EVENT;
		if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
			input_err(true, &client->dev, "%s [ERROR] Read AOT event info\n", __func__);
			goto ERROR;
		}

		input_info(true, &info->client->dev, "%s start, event:%x! gpio:%d\n",
					__func__, rbuf[0], gpio_get_value(info->dtdata->gpio_intr));

		alert_type = rbuf[0] >> 1;

		if (alert_type & MMS_LPM_FLAG_SPAY) {
			info->scrub_id = SPONGE_EVENT_TYPE_SPAY;
			info->scrub_x = 0;
			info->scrub_y = 0;

			input_info(true, &client->dev, "%s: [Gesture] Spay, flag%x\n",
						__func__, info->lowpower_flag);
		} else if (alert_type & MMS_LPM_FLAG_AOD) {
			info->scrub_id = SPONGE_EVENT_TYPE_AOD_DOUBLETAB;

			wbuf[0] = MIP_R0_AOT;
			wbuf[1] = MIP_R0_AOT_POSITION_X;
			if (mms_i2c_read(info, wbuf, 2, rbuf, 4)) {
				input_err(true, &client->dev, "%s [ERROR] Read AOT event info\n", __func__);
				goto ERROR;
			}

			input_info(true, &client->dev, "%s - double tap event(%x, %x, %x, %x)",
						__func__, rbuf[0], rbuf[1], rbuf[2], rbuf[3]);

			info->scrub_x = ((rbuf[0] & 0xFF) << 0) | ((rbuf[1] & 0xFF) << 8);
			info->scrub_y = ((rbuf[2] & 0xFF) << 0) | ((rbuf[3] & 0xFF) << 8);
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
			input_info(true, &client->dev, "%s: aod: %d\n",	__func__, info->scrub_id);
#else
			input_info(true, &client->dev, "%s: aod: %d, %d, %d\n",
						__func__, info->scrub_id, info->scrub_x, info->scrub_y);
#endif

		} else {
			input_err(true, &client->dev, "%s [ERROR] Read a wrong aot action %x\n", __func__, alert_type);
			return IRQ_HANDLED;
		}
		input_report_key(info->input_dev, KEY_BLACK_UI_GESTURE, 1);
		input_sync(info->input_dev);
		input_report_key(info->input_dev, KEY_BLACK_UI_GESTURE, 0);
		input_sync(info->input_dev);
		return IRQ_HANDLED;
	}

	//Read packet info
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_INFO;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		input_err(true, &client->dev, "%s [ERROR] Read packet info\n", __func__);
		goto ERROR;
	}

	input_dbg(true, &client->dev, "%s - info [0x%02X]\n", __func__, rbuf[0]);

	//Check event
	size = (rbuf[0] & 0x7F);
	if (size <= 0) {
		input_err(true, &client->dev, "%s [ERROR] packet size = 0\n", __func__);
		goto ERROR;
	}

	category = ((rbuf[0] >> 7) & 0x1);

	input_dbg(false, &client->dev, "%s - packet size [%d]\n", __func__, size);

	//Read packet data
	wbuf[0] = MIP_R0_EVENT;
	wbuf[1] = MIP_R1_EVENT_PACKET_DATA;
	if (mms_i2c_read(info, wbuf, 2, rbuf, size)) {
		input_err(true, &client->dev, "%s [ERROR] Read packet data\n", __func__);
		goto ERROR;
	}

	if (category == 0) {
		//Touch event
		info->esd_cnt = 0;
		mms_input_event_handler(info, size, rbuf);
	} else {
		//Alert event
		alert_type = rbuf[0];

		input_dbg(true, &client->dev, "%s - alert type [%d]\n", __func__, alert_type);

		if (alert_type == MIP_ALERT_ESD) {
			//ESD detection
			if (mms_alert_handler_esd(info, rbuf))
				goto ERROR;

		} else if (alert_type == MIP_ALERT_WAKEUP) {
			if (info->lowpower_flag & MMS_LPM_FLAG_SPAY) {
				info->scrub_id = 0x04;
				input_report_key(info->input_dev, KEY_BLACK_UI_GESTURE, 1);
				input_sync(info->input_dev);
				input_report_key(info->input_dev, KEY_BLACK_UI_GESTURE, 0);
				input_sync(info->input_dev);
				input_info(true, &client->dev, "%s: [Gesture] Spay, flag%x\n",
						__func__, info->lowpower_flag);
			}
		} else if (alert_type == MIP_ALERT_SRAM_FAILURE) {
			//SRAM failure
			if (mms_alert_handler_sram(info, &rbuf[1]))
				goto ERROR;

		} else {
			input_err(true, &client->dev, "%s [ERROR] Unknown alert type [%d]\n",
				__func__, alert_type);
			goto ERROR;
		}
	}

	input_dbg(true, &client->dev, "%s [DONE]\n", __func__);
	return IRQ_HANDLED;

ERROR:
	input_err(true, &client->dev, "%s [ERROR]\n", __func__);
	if (RESET_ON_EVENT_ERROR) {
		input_info(true, &client->dev, "%s - Reset on error\n", __func__);

		mms_disable(info);
		mms_clear_input(info);
		mms_enable(info);
	}
	return IRQ_HANDLED;
}

/**
 * Update firmware from kernel built-in binary
 */
int mms_fw_update_from_kernel(struct mms_ts_info *info, bool force)
{
	const char *fw_name = info->dtdata->fw_name;
	const struct firmware *fw;
	int retires = 3;
	int ret;

	if (!fw_name)
		goto ERROR;

	input_info(true, &info->client->dev, "%s [START]\n", __func__);

	//Disable IRQ
	mutex_lock(&info->lock);
	disable_irq(info->client->irq);
	mms_clear_input(info);

	//Get firmware
	request_firmware(&fw, fw_name, &info->client->dev);

	if (!fw) {
		input_err(true, &info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		goto ERROR;
	}

	//Update fw
	do {
		ret = mms_flash_fw(info, fw->data, fw->size, force, true);
		if (ret >= fw_err_none)
			break;
	} while (--retires);

	if (!retires) {
		input_err(true, &info->client->dev, "%s [ERROR] mms_flash_fw failed\n", __func__);
		ret = -1;
	}

	release_firmware(fw);

	//Enable IRQ
	enable_irq(info->client->irq);
	mutex_unlock(&info->lock);

	if (ret < 0)
		goto ERROR;

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	input_err(true, &info->client->dev, "%s [ERROR]\n", __func__);
	return -1;
}

/**
 * Update firmware from external storage
 */
int mms_fw_update_from_storage(struct mms_ts_info *info, bool force)
{
	struct file *fp;
	mm_segment_t old_fs;
	size_t fw_size, nread;
	int ret = 0;

	input_info(true, &info->client->dev, "%s [START]\n", __func__);

	//Disable IRQ
	mutex_lock(&info->lock);
	disable_irq(info->client->irq);
	mms_clear_input(info);

	//Get firmware
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(EXTERNAL_FW_PATH, O_RDONLY, 0400);
	if (IS_ERR(fp)) {
		input_err(true, &info->client->dev, "%s [ERROR] file_open - path[%s]\n",
			__func__, EXTERNAL_FW_PATH);
		ret = fw_err_file_open;
		goto ERROR;
	}

	fw_size = fp->f_path.dentry->d_inode->i_size;
	if (fw_size > 0) {
		unsigned char *fw_data;

		fw_data = kzalloc(fw_size, GFP_KERNEL);
		nread = vfs_read(fp, (char __user *)fw_data, fw_size, &fp->f_pos);
		input_info(true, &info->client->dev, "%s - path [%s] size [%zu]\n",
			__func__, EXTERNAL_FW_PATH, fw_size);

		if (nread != fw_size) {
			input_err(true, &info->client->dev, "%s [ERROR] vfs_read - size[%zu] read[%zu]\n",
				__func__, fw_size, nread);
			ret = fw_err_file_read;
		} else {
			//Update fw
			ret = mms_flash_fw(info, fw_data, fw_size, force, true);
		}

		kfree(fw_data);
	} else {
		input_err(true, &info->client->dev, "%s [ERROR] fw_size [%zu]\n", __func__, fw_size);
		ret = fw_err_file_read;
	}

	filp_close(fp, current->files);

ERROR:
	set_fs(old_fs);

	//Enable IRQ
	enable_irq(info->client->irq);
	mutex_unlock(&info->lock);

	if (ret == 0)
		input_err(true, &info->client->dev, "%s [DONE]\n", __func__);
	else
		input_err(true, &info->client->dev, "%s [ERROR] %d\n", __func__, ret);

	return ret;
}
/**
 * Getting firmware from air
 */
int mms_fw_update_from_ffu(struct mms_ts_info *info, bool force)
{
	const struct firmware *fw;
	int retires = 3;
	int ret;

	input_info(true, &info->client->dev, "%s [START]\n", __func__);

	//Disable IRQ
	mutex_lock(&info->lock);
	disable_irq(info->client->irq);
	mms_clear_input(info);

	//Get firmware
	request_firmware(&fw, FFU_FW_PATH, &info->client->dev);

	if (!fw) {
		input_err(true, &info->client->dev, "%s [ERROR] request_firmware\n", __func__);
		goto ERROR;
	}

	//Update fw
	do {
		ret = mms_flash_fw(info, fw->data, fw->size, force, true);
		if (ret >= fw_err_none)
			break;
	} while (--retires);

	if (!retires) {
		input_err(true, &info->client->dev, "%s [ERROR] mms_flash_fw failed\n", __func__);
		ret = -1;
	}

	release_firmware(fw);

	//Enable IRQ
	enable_irq(info->client->irq);
	mutex_unlock(&info->lock);

	if (ret < 0)
		goto ERROR;

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	enable_irq(info->client->irq);
	mutex_unlock(&info->lock);
	input_err(true, &info->client->dev, "%s [ERROR]\n", __func__);
	return -1;
}

static ssize_t mms_sys_fw_update(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);
	int result = 0;
	u8 data[255];
	int ret = 0;

	memset(info->print_buf, 0, PAGE_SIZE);

	input_info(true, &info->client->dev, "%s [START]\n", __func__);

	ret = mms_fw_update_from_storage(info, true);

	switch (ret) {
	case fw_err_none:
		sprintf(data, "F/W update success\n");
		break;
	case fw_err_uptodate:
		sprintf(data, "F/W is already up-to-date\n");
		break;
	case fw_err_download:
		sprintf(data, "F/W update failed : Download error\n");
		break;
	case fw_err_file_type:
		sprintf(data, "F/W update failed : File type error\n");
		break;
	case fw_err_file_open:
		sprintf(data, "F/W update failed : File open error [%s]\n", EXTERNAL_FW_PATH);
		break;
	case fw_err_file_read:
		sprintf(data, "F/W update failed : File read error\n");
		break;
	default:
		sprintf(data, "F/W update failed\n");
		break;
	}

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);

	strcat(info->print_buf, data);
	result = snprintf(buf, PAGE_SIZE, "%s\n", info->print_buf);
	return result;
}
static DEVICE_ATTR(fw_update, S_IWUSR | S_IWGRP, mms_sys_fw_update, NULL);

/**
 * Sysfs attr info
 */
static struct attribute *mms_attrs[] = {
	&dev_attr_fw_update.attr,
	NULL,
};

/**
 * Sysfs attr group info
 */
static const struct attribute_group mms_attr_group = {
	.attrs = mms_attrs,
};

/**
 * Initial config
 */
static int mms_init_config(struct mms_ts_info *info)
{
	u8 wbuf[8];
	u8 rbuf[32];
	u8 tmp[4] = MMS_CONFIG_DATE;

	input_info(true, &info->client->dev, "%s [START]\n", __func__);

	/* read product name */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_PRODUCT_NAME;
	mms_i2c_read(info, wbuf, 2, rbuf, 16);
	memcpy(info->product_name, rbuf, 16);
	input_info(true, &info->client->dev, "%s - product_name[%s]\n",
		__func__, info->product_name);

	/* read fw version */
	mms_get_fw_version(info, rbuf);

	/* read fw build date */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_BUILD_DATE;
	mms_i2c_read(info, wbuf, 2, rbuf, 4);

	if (!strncmp(rbuf, "", 4))
		memcpy(rbuf, tmp, 4);

	info->fw_year = (rbuf[0] << 8) | (rbuf[1]);
	info->fw_month = rbuf[2];
	info->fw_date = rbuf[3];

	input_info(true, &info->client->dev, "%s - fw build date : %d/%d/%d\n",
		__func__, info->fw_year, info->fw_month, info->fw_date);

	/* read checksum */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_CHECKSUM_PRECALC;
	mms_i2c_read(info, wbuf, 2, rbuf, 4);

	info->pre_chksum = (rbuf[0] << 8) | (rbuf[1]);
	info->rt_chksum = (rbuf[2] << 8) | (rbuf[3]);
	input_info(true, &info->client->dev,
		"%s - precalced checksum:%04X, real-time checksum:%04X\n",
		__func__, info->pre_chksum, info->rt_chksum);


	/* Set resolution using chip info */
	wbuf[0] = MIP_R0_INFO;
	wbuf[1] = MIP_R1_INFO_RESOLUTION_X;
	mms_i2c_read(info, wbuf, 2, rbuf, 7);

	info->max_x = (rbuf[0]) | (rbuf[1] << 8);
	info->max_y = (rbuf[2]) | (rbuf[3] << 8);
	input_info(true, &info->client->dev, "%s - max_x[%d] max_y[%d]\n",
		__func__, info->max_x, info->max_y);

	info->node_x = rbuf[4];
	info->node_y = rbuf[5];
	info->node_key = rbuf[6];
	input_info(true, &info->client->dev, "%s - node_x[%d] node_y[%d] node_key[%d]\n",
		__func__, info->node_x, info->node_y, info->node_key);

#if MMS_USE_TOUCHKEY
	/* Enable touchkey */
	if (info->node_key > 0)
		info->tkey_enable = true;
#endif
	info->event_size = 8;

	input_info(true, &info->client->dev, "%s [DONE]\n", __func__);
	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_DUMP_MODE)
#include <linux/sec_debug.h>
extern struct tsp_dump_callbacks dump_callbacks;
static struct delayed_work *p_ghost_check;

static void mms_run_rawdata(struct mms_ts_info *info)
{
	info->tsp_dump_lock = 1;
	input_raw_data_clear();
	input_raw_info(true, &info->client->dev, "%s: start ##\n", __func__);
	if (mms_run_test(info, MIP_TEST_TYPE_CM_DELTA)) {
		input_err(true, &info->client->dev, "%s cm_delta error\n", __func__);
	}
	if (mms_run_test(info, MIP_TEST_TYPE_CM_ABS)) {
		input_err(true, &info->client->dev, "%s cm_abs error\n", __func__);
	}
	if (mms_run_test(info, MIP_TEST_TYPE_CM_JITTER)) {
		input_err(true, &info->client->dev, "%s cm_jitter error\n", __func__);
	}
	if (mms_run_test(info, MIP_TEST_TYPE_OPEN)) {
		input_err(true, &info->client->dev, "%s open error\n", __func__);
	}	
	input_raw_info(true, &info->client->dev, "%s: done ##\n", __func__);
	info->tsp_dump_lock = 0;
}

static void mms_check_rawdata(struct work_struct *work)
{
	struct mms_ts_info *info = container_of(work, struct mms_ts_info, 
			ghost_check.work);

	if (info->tsp_dump_lock == 1) {
		input_info(true, &info->client->dev, "%s: ignored ## already checking..\n", __func__);
		return;
	}

	if (info->ic_status == PWR_OFF) {
		input_info(true, &info->client->dev, "%s: ignored ## IC is power off\n", __func__);
		return;
	}

	mms_run_rawdata(info);
}

static void dump_tsp_log(void)
{
	pr_info("%s: %s %s: start\n", MMS_DEVICE_NAME, SECLOG, __func__);

#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge == 1) {
		pr_err("%s: %s %s: ignored ## lpm charging Mode!!\n", MMS_DEVICE_NAME, SECLOG, __func__);
		return;
	}
#endif

	if (p_ghost_check == NULL) {
		pr_err("%s: %s %s: ignored ## tsp probe fail!!\n", MMS_DEVICE_NAME, SECLOG, __func__);
		return;
	}
	schedule_delayed_work(p_ghost_check, msecs_to_jiffies(100));
}
#endif

static void mms_read_info_work(struct work_struct *work)
{
	struct mms_ts_info *info = container_of(work, struct mms_ts_info, 
			work_read_info.work);

	mutex_lock(&info->modechange);
	input_log_fix();
	mms_run_rawdata(info);
	info->info_work_done = true;
	mutex_unlock(&info->modechange);
}

/**
 * Initialize driver
 */
static int mms_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;

	input_err(true, &client->dev, "%s [START]\n", __func__);

#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge == 1) {
		input_err(true, &client->dev, "%s : Do not load driver due to : lpm %d\n",
			 __func__, lpcharge);
		return -ENODEV;
	}
#endif

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		input_err(true, &client->dev,
			"%s [ERROR] i2c_check_functionality\n", __func__);
		ret = -EIO;
		goto ERROR;
	}

	info = kzalloc(sizeof(struct mms_ts_info), GFP_KERNEL);
	if (!info) {
		goto err_mem_alloc;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		input_err(true, &client->dev, "%s [ERROR] input alloc\n", __func__);
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->irq = -1;
	info->init = true;
	mutex_init(&info->lock);
	info->touch_count = 0;

#if MMS_USE_DEVICETREE
	if (client->dev.of_node) {
		info->dtdata  =
			devm_kzalloc(&client->dev,
				sizeof(struct mms_devicetree_data), GFP_KERNEL);
		if (!info->dtdata) {
			input_err(true, &client->dev,
				"%s [ERROR] dtdata devm_kzalloc\n", __func__);
			goto err_devm_alloc;
		}
		mms_parse_devicetree(&client->dev, info);
	} else
#endif
	{
		info->dtdata = client->dev.platform_data;
		if (info->dtdata == NULL) {
			input_err(true, &client->dev, "%s [ERROR] dtdata is null\n", __func__);
			ret = -EINVAL;
			goto err_platform_data;
		}
	}

	info->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(info->pinctrl)) {
		input_err(true, &client->dev, "%s: Failed to get pinctrl data\n", __func__);
		ret = PTR_ERR(info->pinctrl);
		goto err_platform_data;
	}

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	tui_tsp_info = info;
#endif

	snprintf(info->phys, sizeof(info->phys), "%s/input0", dev_name(&client->dev));
	input_dev->name = "sec_touchscreen";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
#if MMS_USE_INPUT_OPEN_CLOSE
	input_dev->open = mms_input_open;
	input_dev->close = mms_input_close;
#endif
	//set input event buffer size
	input_set_events_per_packet(input_dev, 200);

	input_set_drvdata(input_dev, info);
	i2c_set_clientdata(client, info);

	ret = input_register_device(input_dev);
	if (ret) {
		input_err(true, &client->dev, "%s [ERROR] input_register_device\n", __func__);
		ret = -EIO;
		goto err_input_register_device;
	}

	mms_power_control(info, 1);

#if MMS_USE_AUTO_FW_UPDATE
	ret = mms_fw_update_from_kernel(info, false);
	if (ret) {
		input_err(true, &client->dev, "%s [ERROR] mms_fw_update_from_kernel\n", __func__);
		goto err_fw_update;
	}
#endif

	mms_init_config(info);
	mms_config_input(info);

#ifdef USE_TSP_TA_CALLBACKS
	info->register_cb = mms_register_callback;
	info->callbacks.inform_charger = mms_charger_status_cb;
	if (info->register_cb)
		info->register_cb(&info->callbacks);
#endif
#ifdef CONFIG_VBUS_NOTIFIER
	vbus_notifier_register(&info->vbus_nb, mms_vbus_notification,
				VBUS_NOTIFY_DEV_CHARGER);
#endif

	ret = request_threaded_irq(client->irq, NULL, mms_interrupt,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, MMS_DEVICE_NAME, info);
	if (ret) {
		input_err(true, &client->dev, "%s [ERROR] request_threaded_irq\n", __func__);
		goto err_request_irq;
	}

	disable_irq(client->irq);
	info->irq = client->irq;

#if MMS_USE_NAP_MODE
	//Wake lock for nap mode
	wake_lock_init(&mms_wake_lock, WAKE_LOCK_SUSPEND, "mms_wake_lock");
#endif

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
	trustedui_set_tsp_irq(info->irq);
	input_err(&client->dev, "%s[%d] called!\n",
		__func__, info->irq);
#endif

	mms_enable(info);

#if MMS_USE_DEV_MODE
	if (mms_dev_create(info)) {
		input_err(true, &client->dev, "%s [ERROR] mms_dev_create\n", __func__);
		ret = -EAGAIN;
		goto err_test_dev_create;
	}

	info->class = class_create(THIS_MODULE, MMS_DEVICE_NAME);
	device_create(info->class, NULL, info->mms_dev, NULL, MMS_DEVICE_NAME);
#endif

#if MMS_USE_TEST_MODE
	if (mms_sysfs_create(info)) {
		input_err(true, &client->dev, "%s [ERROR] mms_sysfs_create\n", __func__);
		ret = -EAGAIN;
		goto err_test_sysfs_create;
	}
#endif

#if MMS_USE_CMD_MODE
	if (mms_sysfs_cmd_create(info)) {
		input_err(true, &client->dev, "%s [ERROR] mms_sysfs_cmd_create\n", __func__);
		ret = -EAGAIN;
		goto err_fac_cmd_create;
	}
#endif

	if (sysfs_create_group(&client->dev.kobj, &mms_attr_group)) {
		input_err(true, &client->dev, "%s [ERROR] sysfs_create_group\n", __func__);
		ret = -EAGAIN;
		goto err_create_attr_group;
	}

	if (sysfs_create_link(NULL, &client->dev.kobj, MMS_DEVICE_NAME)) {
		input_err(true, &client->dev, "%s [ERROR] sysfs_create_link\n", __func__);
		ret = -EAGAIN;
		goto err_create_dev_link;
	}

#if defined(CONFIG_TOUCHSCREEN_DUMP_MODE)
	dump_callbacks.inform_dump = dump_tsp_log;
	INIT_DELAYED_WORK(&info->ghost_check, mms_check_rawdata);
	p_ghost_check = &info->ghost_check;
#endif
	INIT_DELAYED_WORK(&info->work_read_info, mms_read_info_work);
	mutex_init(&info->modechange);
	schedule_delayed_work(&info->work_read_info, msecs_to_jiffies(5000));

	device_init_wakeup(&client->dev, true);
	info->init = false;
	info->ic_status = PWR_ON;
	input_info(true, &client->dev,
		"MELFAS " CHIP_NAME " Touchscreen is initialized successfully\n");
	input_log_fix();	
	return 0;


err_create_dev_link:
	sysfs_remove_group(&client->dev.kobj, &mms_attr_group);
err_create_attr_group:
#if MMS_USE_CMD_MODE
	mms_sysfs_cmd_remove(info);
err_fac_cmd_create:
#endif
#if MMS_USE_TEST_MODE
	mms_sysfs_remove(info);
err_test_sysfs_create:
#endif
#if MMS_USE_DEV_MODE
	device_destroy(info->class, info->mms_dev);
	class_destroy(info->class);
err_test_dev_create:
#endif
	mms_disable(info);
	free_irq(info->irq, info);
err_request_irq:
err_fw_update:
	mms_power_control(info, 0);
	input_unregister_device(info->input_dev);
	info->input_dev = NULL;
err_input_register_device:
err_platform_data:
#if MMS_USE_DEVICETREE
err_devm_alloc:
#endif
	if (info->input_dev)
		input_free_device(info->input_dev);
err_input_alloc:
	kfree(info);
err_mem_alloc:
ERROR:
#ifdef CONFIG_TOUCHSCREEN_DUMP_MODE
		p_ghost_check = NULL;
#endif
	pr_err("MELFAS " CHIP_NAME " Touchscreen initialization failed.\n");
	input_log_fix();	
	return ret;
}

#ifdef CONFIG_TRUSTONIC_TRUSTED_UI
void trustedui_mode_on(void)
{
	input_err(&tui_tsp_info->client->dev, "%s, release all finger..\n",	__func__);
	mms_clear_input(tui_tsp_info);
}
#endif

/**
 * Remove driver
 */
static int mms_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	if (info->irq >= 0)
		free_irq(info->irq, info);

#if MMS_USE_CMD_MODE
	mms_sysfs_cmd_remove(info);
#endif

#if MMS_USE_TEST_MODE
	mms_sysfs_remove(info);
#endif

	sysfs_remove_group(&info->client->dev.kobj, &mms_attr_group);
	sysfs_remove_link(NULL, MMS_DEVICE_NAME);

#if MMS_USE_DEV_MODE
	device_destroy(info->class, info->mms_dev);
	class_destroy(info->class);
#endif
#ifdef CONFIG_TOUCHSCREEN_DUMP_MODE
	p_ghost_check = NULL;
#endif
	cancel_delayed_work_sync(&info->work_read_info);
	flush_delayed_work(&info->work_read_info);

	input_unregister_device(info->input_dev);

	kfree(info->fw_name);
	kfree(info);

	return 0;
}

static void mms_shutdown(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	input_err(true, &info->client->dev, "%s\n", __func__);

	mms_disable(info);
}

#ifdef CONFIG_PM
static int mms_suspend(struct device *dev)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);

	if (info->ic_status == LPM_RESUME)
		info->ic_status = LPM_SUSPEND;

	return 0;
}

static int mms_resume(struct device *dev)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);

	if (info->ic_status == LPM_SUSPEND)
		info->ic_status = LPM_RESUME;

	return 0;
}

static const struct dev_pm_ops mms_dev_pm_ops = {
	.suspend = mms_suspend,
	.resume = mms_resume,
};
#endif

#if MMS_USE_DEVICETREE
/**
 * Device tree match table
 */
static const struct of_device_id mms_match_table[] = {
	{ .compatible = "melfas,mms_ts",},
	{},
};
MODULE_DEVICE_TABLE(of, mms_match_table);
#endif

/**
 * I2C Device ID
 */
static const struct i2c_device_id mms_id[] = {
	{MMS_DEVICE_NAME, 0},
};
MODULE_DEVICE_TABLE(i2c, mms_id);

/**
 * I2C driver info
 */
static struct i2c_driver mms_driver = {
	.id_table	= mms_id,
	.probe = mms_probe,
	.remove = mms_remove,
	.shutdown = mms_shutdown,
	.driver = {
		.name = MMS_DEVICE_NAME,
		.owner = THIS_MODULE,
#if MMS_USE_DEVICETREE
		.of_match_table = mms_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &mms_dev_pm_ops,
#endif
	},
};

/**
 * Init driver
 */
static int __init mms_init(void)
{
	pr_err("%s\n", __func__);
#if defined(CONFIG_SAMSUNG_LPM_MODE)
	if (poweroff_charging) {
		pr_notice("%s : LPM Charging Mode!!\n", __func__);
		return 0;
	}
#endif
	return i2c_add_driver(&mms_driver);
}

/**
 * Exit driver
 */
static void __exit mms_exit(void)
{
	i2c_del_driver(&mms_driver);
}

module_init(mms_init);
module_exit(mms_exit);

MODULE_DESCRIPTION("MELFAS MMS400 Touchscreen");
MODULE_VERSION("2014.12.05");
MODULE_AUTHOR("Jee, SangWon <jeesw@melfas.com>");
MODULE_LICENSE("GPL");
