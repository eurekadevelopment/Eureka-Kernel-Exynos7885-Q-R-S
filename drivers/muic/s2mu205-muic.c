/*
 * driver/muic/s2mu205.c - S2MU205 micro USB switch device driver
 *
 * Copyright (C) 2019 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#define pr_fmt(fmt)	"[MUIC] " fmt

#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/muic/muic.h>
#include <linux/mfd/samsung/s2mu205.h>
#include <linux/muic/s2mu205-muic.h>

#if IS_ENABLED(CONFIG_MUIC_SYSFS)
#include <linux/muic/muic_sysfs.h>
#endif

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif

#if IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
#include <linux/usb_notify.h>
#endif
#include <linux/muic/muic_interface.h>

#if IS_ENABLED(CONFIG_CCIC_NOTIFIER)
#include <linux/ccic/ccic_notifier.h>
#endif
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
#include <linux/vbus_notifier.h>
#endif
#if IS_ENABLED(CONFIG_MUIC_UART_SWITCH)
#include <mach/pinctrl-samsung.h>
#endif
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
#include <soc/samsung/exynos-modem-ctrl.h>
#endif

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
#include <linux/fb.h>
#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
#include <linux/sec_batt.h>
#endif
#endif

/* Prototypes of the Static symbols of s2mu205-muic */
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
static void s2mu205_muic_detect_dev_ccic(struct s2mu205_muic_data *muic_data,
		muic_attached_dev_t new_dev);
int s2mu205_muic_bcd_rescan(struct s2mu205_muic_data *muic_data);
#endif
static int s2mu205_muic_detect_dev_bc1p2(struct s2mu205_muic_data *muic_data);
static void s2mu205_muic_handle_attached_dev(struct s2mu205_muic_data *muic_data);
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
static void s2mu205_muic_set_water_state(struct s2mu205_muic_data *muic_data, bool en);
static void s2mu205_muic_set_rid_for_water(struct s2mu205_muic_data *muic_data, bool en);
static void s2mu205_muic_set_rid_int_mask_en(struct s2mu205_muic_data *muic_data, bool en);
#endif
void s2mu205_muic_get_detect_info(struct s2mu205_muic_data *muic_data);

/*
 * Debuging functions
 */
static const char *dev_to_str(muic_attached_dev_t n)
{
	char *ret;

	switch (n) {
	ENUM_STR(ATTACHED_DEV_NONE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_USB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CDP_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_OTG_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_ANY_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_USB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_CDP_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNDEFINED_CHARGING_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_DESKDOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNKNOWN_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_DESKDOCK_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CARDOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_FG_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_ON_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_UART_ON_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_USB_OFF_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_JIG_USB_ON_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_USB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNIVERSAL_MMDOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AUDIODOCK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_MHL_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CHARGING_CABLE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_PREPARE_DUPLI_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_5V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_5V_DUPLI_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_9V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_ERR_V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_ERR_V_DUPLI_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_PREPARE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_5V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_ERR_V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_9V_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_UNDEFINED_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_UNSUPPORTED_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_SUPPORTED_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_HMT_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_VZW_ACC_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_VZW_INCOMPATIBLE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_USB_LANHUB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TYPE2_CHG_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TYPE3_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TYPE3_MUIC_TA, ret);
	ENUM_STR(ATTACHED_DEV_TYPE3_ADAPTER_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TYPE3_CHARGER_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_NONE_TYPE3_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNSUPPORTED_ID_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNSUPPORTED_ID_VB_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_TIMEOUT_OPEN_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_WIRELESS_PAD_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_POWERPACK_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNDEFINED_RANGE_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CHK_WATER_REQ, ret);
	ENUM_STR(ATTACHED_DEV_CHK_WATER_DRY_REQ, ret);
	ENUM_STR(ATTACHED_DEV_GAMEPAD_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_CHECK_OCP, ret);
	ENUM_STR(ATTACHED_DEV_RDU_TA_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_9V_DUPLI_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_UNKNOWN_MUIC, ret);
	ENUM_STR(ATTACHED_DEV_NUM, ret);
	default:
		return "invalid";
	}
	return ret;
}

#if IS_ENABLED(CONFIG_MUIC_DEBUG)
#define MAX_LOG 25
#define READ 0
#define WRITE 1

static u8 s2mu205_log_cnt;
static u8 s2mu205_log[MAX_LOG][3];

static void s2mu205_reg_log(u8 reg, u8 value, u8 rw)
{
	s2mu205_log[s2mu205_log_cnt][0] = reg;
	s2mu205_log[s2mu205_log_cnt][1] = value;
	s2mu205_log[s2mu205_log_cnt][2] = rw;
	s2mu205_log_cnt++;
	if (s2mu205_log_cnt >= MAX_LOG)
		s2mu205_log_cnt = 0;
}

static void s2mu205_print_reg_log(void)
{
	int i;
	u8 reg, value, rw;
	char mesg[256] = "";

	for (i = 0; i < MAX_LOG; i++) {
		reg = s2mu205_log[s2mu205_log_cnt][0];
		value = s2mu205_log[s2mu205_log_cnt][1];
		rw = s2mu205_log[s2mu205_log_cnt][2];
		s2mu205_log_cnt++;

		if (s2mu205_log_cnt >= MAX_LOG)
			s2mu205_log_cnt = 0;
		sprintf(mesg + strlen(mesg), "%x(%x)%x ", reg, value, rw);
	}
	pr_info("%s %s\n", __func__, mesg);
}

void s2mu205_read_reg_dump(struct s2mu205_muic_data *muic, char *mesg)
{
	u8 val;

	mutex_lock(&muic->muic_mutex);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_MUIC_CTRL1, &val);
	sprintf(mesg + strlen(mesg), "CTRL1:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_MANUAL_SW_CTRL, &val);
	sprintf(mesg + strlen(mesg), "SW_CTRL:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_MUIC_INT1_MASK, &val);
	sprintf(mesg + strlen(mesg), "IM1:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_MUIC_INT2_MASK, &val);
	sprintf(mesg + strlen(mesg), "IM2:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_CHG_TYP, &val);
	sprintf(mesg + strlen(mesg), "CHG_T:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_DEVICE_APPLE, &val);
	sprintf(mesg + strlen(mesg), "APPLE_DT:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_ADC_VALUE, &val);
	sprintf(mesg + strlen(mesg), "ADC:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_DEVICE_TYP1, &val);
	sprintf(mesg + strlen(mesg), "DT1:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_DEVICE_TYP2, &val);
	sprintf(mesg + strlen(mesg), "DT2:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_DEVICE_TYP3, &val);
	sprintf(mesg + strlen(mesg), "DT3:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_MANUAL_SW_CTRL, &val);
	sprintf(mesg + strlen(mesg), "SW_CTRL:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_MUIC_INT1, &val);
	sprintf(mesg + strlen(mesg), "INT1:%x ", val);
	s2mu205_read_reg(muic->i2c, S2MU205_REG_MUIC_INT2, &val);
	sprintf(mesg + strlen(mesg), "INT2:%x ", val);
	mutex_unlock(&muic->muic_mutex);
}

void s2mu205_print_reg_dump(struct s2mu205_muic_data *muic_data)
{
	char mesg[256] = "";
	s2mu205_read_reg_dump(muic_data, mesg);
	pr_info("%s %s\n", __func__, mesg);
}

void s2mu205_read_reg_dump_otp(struct s2mu205_muic_data *muic_data)
{
	int i;
	u8 val;
	pr_info("%s=======================START\n", __func__);
	for (i = 0x6D; i <= 0xD9; i++) {
		s2mu205_read_reg(muic_data->i2c, i, &val);
		pr_info("0x%X : 0x%X\n", i, val);
	}
	pr_info("%s=======================END\n", __func__);
}
#endif

/*
 * Unit Functions
 */
int s2mu205_i2c_read_byte(struct i2c_client *client, u8 command)
{
	int ret = 0;
	int retry = 0;
	u8 data = 0;

	ret = s2mu205_read_reg(client, command, &data);
	while (ret < 0) {
		pr_info("failed to read reg(0x%x) retry(%d)\n", command, retry);
		if (retry > 10) {
			pr_err("%s  retry failed!!\n", __func__);
			break;
		}
		msleep(100);
		ret = s2mu205_read_reg(client, command, &data);
		retry++;
	}

#if IS_ENABLED(DEBUG_MUIC)
	s2mu205_reg_log(command, ret, retry << 1 | READ);
#endif
	return data;
}

int s2mu205_i2c_write_byte(struct i2c_client *client, u8 command, u8 value)
{
	int ret_r = 0;
	int ret_w = 0;
	int retry = 0;
	u8 written = 0;

	ret_w = s2mu205_write_reg(client, command, value);
	while (ret_w < 0) {
		pr_info("failed to write reg(0x%x) retry(%d)\n", command, retry);
		ret_r = s2mu205_read_reg(client, command, &written);
		if (ret_r < 0)
			pr_err("%s reg(0x%x)\n", __func__, command);
		msleep(100);
		ret_w = s2mu205_write_reg(client, command, value);
		retry++;
	}
#if IS_ENABLED(DEBUG_MUIC)
	s2mu205_reg_log(command, value, retry << 1 | WRITE);
#endif
	return ret_w;
}

static int _s2mu205_i2c_guaranteed_wbyte(struct i2c_client *client,
		u8 command, u8 value)
{
	int ret;
	int retry = 0;
	int written;

	ret = s2mu205_i2c_write_byte(client, command, value);
	written = s2mu205_i2c_read_byte(client, command);
	while (written != value) {
		pr_info("reg(0x%x): written(0x%x) != value(0x%x)\n",
			command, written, value);
		if (retry > 10) {
			pr_err("%s  retry failed!!\n", __func__);
			break;
		}
		msleep(100);
		retry++;
		ret = s2mu205_i2c_write_byte(client, command, value);
		written = s2mu205_i2c_read_byte(client, command);
	}
	return ret;
}

static int _s2mu205_i2c_update_bit(struct i2c_client *i2c, u8 reg,
		u8 mask, u8 shift, u8 value)
{
	int ret;
	u8 reg_val = 0;

	reg_val = s2mu205_i2c_read_byte(i2c, reg);
	reg_val &= ~mask;
	reg_val |= value << shift;
	ret = s2mu205_i2c_write_byte(i2c, reg, reg_val);
	pr_info("%s reg(0x%x) value(0x%x)\n", __func__, reg, reg_val);
	if (ret < 0)
		pr_err("%s  Reg = 0x%X, mask = 0x%X, val = 0x%X write err : %d\n",
				__func__, reg, mask, value, ret);

	return ret;
}

static void _s2mu205_muic_set_chg_det(struct s2mu205_muic_data *muic_data,
		bool enable)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 r_val = 0, w_val = 0;

	r_val = s2mu205_i2c_read_byte(i2c, S2MU205_REG_RID_DISCHARGE);
	if (enable) {
		w_val = r_val & ~(RID_DISCHARGE_CHG_DET_OFF_MASK);
	} else {
		w_val = r_val | RID_DISCHARGE_CHG_DET_OFF_MASK;
	}
	if(w_val != r_val) {
		pr_info("%s en(%d)\n", __func__, enable);
		s2mu205_i2c_write_byte(i2c, S2MU205_REG_RID_DISCHARGE, w_val);
	}
}

static int _s2mu205_muic_sel_path(struct s2mu205_muic_data *muic_data,
		t_path_data path_data)
{
	int ret = 0;
	u8 reg_val1, reg_val2;

	reg_val1 = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MANUAL_SW_CTRL);
	reg_val2 = reg_val1 & ~(MANUAL_SW_CTRL_DM_SWITCHING_MASK | MANUAL_SW_CTRL_DP_SWITCHING_MASK);

#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
	if (muic_data->is_hiccup_mode)
		reg_val2 |= MANUAL_SW_CTRL_UART2_MASK;
	else {
#endif
		switch (path_data) {
		case S2MU205_PATH_USB:
			reg_val2 |= MANUAL_SW_CTRL_USB_MASK;
			break;
		case S2MU205_PATH_UART_AP:
			reg_val2 |= MANUAL_SW_CTRL_UART1_MASK;
			break;
		case S2MU205_PATH_UART_CP:
#if IS_ENABLED(CONFIG_PMU_UART_SWITCH)
			reg_val2 |= MANUAL_SW_CTRL_UART1_MASK;
#else
			reg_val2 |= MANUAL_SW_CTRL_UART2_MASK;
#endif
			break;
		case S2MU205_PATH_OPEN:
		default:
			reg_val2 |= MANUAL_SW_CTRL_OPEN_MASK;
			break;
		}
#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
	}
#endif
	if (reg_val1 != reg_val2) {
		ret = s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MANUAL_SW_CTRL, reg_val2);
		reg_val2 = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MANUAL_SW_CTRL);
		pr_info("%s manual_sw_ctrl(%#x->%#x)\n", __func__, reg_val1, reg_val2);
	} else
		pr_info("%s Skip to set same path val(%#x)\n", __func__, reg_val1);

    	return ret;
}

static int _s2mu205_muic_set_path_mode(struct s2mu205_muic_data *muic_data,
		t_mode_data mode_data)
{
	int ret = 0;
	u8 reg_val;

	pr_info("%s new mode(%d)\n", __func__, mode_data);
	reg_val = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_CTRL1);

	switch (mode_data) {
	case S2MU205_MODE_MANUAL:
			reg_val &= ~MUIC_CTRL1_MANUAL_SW_MASK;
		break;
	case S2MU205_MODE_AUTO:
	default:
			reg_val |= MUIC_CTRL1_MANUAL_SW_MASK;
		break;
	}

	ret = s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MUIC_CTRL1, reg_val);

    return ret;
}

static int _s2mu205_muic_control_rid_adc(struct s2mu205_muic_data *muic_data, bool enable)
{
	int ret = 0;
	u8 reg_val = 0;

	pr_info("%s (%s)\n", __func__, enable ? "Enable" : "Disable");
	reg_val = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_CTRL2);

	if (enable)
		reg_val &= ~MUIC_CTRL2_ADC_OFF_MASK;
	else
		reg_val |= MUIC_CTRL2_ADC_OFF_MASK;

	ret = s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MUIC_CTRL2, reg_val);

	return 0;
}

static int _s2mu205_muic_set_bcd_rescan_reg(struct s2mu205_muic_data *muic_data)
{
	u8 reg_val = 0;
	reg_val = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_BCD_RESCAN);
	reg_val |= BCD_RESCAN_BCD_RESCAN_MASK;
	s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_BCD_RESCAN, reg_val);
	return 0;
}

static inline int _s2mu205_muic_get_rid_adc(struct s2mu205_muic_data *muic_data)
{
	return s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_ADC_VALUE) & ADC_VALUE_ADCVAL_MASK;
}

static int _s2mu205_muic_com_to_uart(struct s2mu205_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int ret = 0;

	if (muic_data->pdata->is_rustproof) {
		pr_info("%s rustproof mode\n", __func__);
		return ret;
	}

	if (muic_pdata->uart_path == MUIC_PATH_UART_AP) {
		ret = _s2mu205_muic_sel_path(muic_data, S2MU205_PATH_UART_AP);
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
		send_uart_noti_to_modem(MODEM_CTRL_UART_AP);
#endif
	} else {
		ret = _s2mu205_muic_sel_path(muic_data, S2MU205_PATH_UART_CP);
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
		send_uart_noti_to_modem(MODEM_CTRL_UART_CP);
#endif
	}

	return ret;
}

int _s2mu205_muic_set_jig_on(struct s2mu205_muic_data *muic_data)
{
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	bool en = muic_pdata->is_jig_on;
	int reg = 0, ret = 0;

	pr_err("%s: %s\n", __func__, en ? "on" : "off");
	reg = s2mu205_i2c_read_byte(muic_data->i2c,
		S2MU205_REG_MANUAL_SW_CTRL);

	if (en)
		reg |= MANUAL_SW_CTRL_JIG_MASK;
	else
		reg &= ~(MANUAL_SW_CTRL_JIG_MASK);

	ret = s2mu205_i2c_write_byte(muic_data->i2c,
			S2MU205_REG_MANUAL_SW_CTRL, (u8)reg);

	return ret;
#else
	pr_err("%s: Skip the jig control, Not Factory Mode.\n", __func__);
	return 0;
#endif
}

#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
int _s2mu205_muic_recheck_adc(struct s2mu205_muic_data *muic_data)
{
	_s2mu205_muic_control_rid_adc(muic_data, false);
	usleep_range(10000, 12000);
	_s2mu205_muic_control_rid_adc(muic_data, true);
	msleep(200);
	return _s2mu205_muic_get_rid_adc(muic_data);
}

int s2mu205_muic_refresh_adc(struct s2mu205_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int adc = 0;
	u8 reg_data, b_Rid_en = 0;

	reg_data = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_CTRL2);
	if (!(reg_data & MUIC_CTRL2_ADC_OFF_MASK)) {
		b_Rid_en = 1;
	} else {
		pr_info("%s, enable the RID\n", __func__);
		reg_data &= ~MUIC_CTRL2_ADC_OFF_MASK;
		s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_CTRL2, reg_data);
		msleep(35);
	}

	adc = (s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_ADC_VALUE)) & ADC_VALUE_ADCVAL_MASK;
	pr_info("%s, adc : 0x%X\n", __func__, adc);

	if (!b_Rid_en) {
		pr_info("%s, disable the RID\n", __func__);
		reg_data |= MUIC_CTRL2_ADC_OFF_MASK;
		s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_CTRL2, reg_data);
	}

	return adc;
}
#endif

static inline int _s2mu205_muic_get_vbus_state(struct s2mu205_muic_data *muic_data)
{
	return (s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_DEVICE_APPLE)
		& DEVICE_APPLE_VBUS_WAKEUP_MASK) >> DEVICE_APPLE_VBUS_WAKEUP_SHIFT;
}

void s2mu205_muic_get_detect_info(struct s2mu205_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	struct i2c_client *i2c = muic_data->i2c;

	muic_data->reg[DEVICE_TYPE1] = s2mu205_i2c_read_byte(i2c, S2MU205_REG_DEVICE_TYP1);
	muic_data->reg[DEVICE_TYPE2] = s2mu205_i2c_read_byte(i2c, S2MU205_REG_DEVICE_TYP2);
	muic_data->reg[DEVICE_TYPE3] = s2mu205_i2c_read_byte(i2c, S2MU205_REG_DEVICE_TYP3);
	muic_data->reg[DEVICE_APPLE] = s2mu205_i2c_read_byte(i2c, S2MU205_REG_DEVICE_APPLE);
	muic_data->reg[CHG_TYPE] = s2mu205_i2c_read_byte(i2c, S2MU205_REG_CHG_TYP);
	muic_data->reg[ADC] = _s2mu205_muic_get_rid_adc(muic_data);
	muic_data->vbvolt = muic_pdata->vbvolt = _s2mu205_muic_get_vbus_state(muic_data);
	muic_data->adc = muic_pdata->adc = muic_data->reg[ADC];

	pr_info("dev[1:0x%02x, 2:0x%02x, 3:0x%02x]\n", muic_data->reg[DEVICE_TYPE1],
		muic_data->reg[DEVICE_TYPE2], muic_data->reg[DEVICE_TYPE3]);
	pr_info("adc:0x%02x, vbvolt:0x%02x, apple:0x%02x\n",
		muic_data->adc, muic_data->vbvolt, muic_data->reg[DEVICE_APPLE]);
	pr_info("chg_type:0x%02x\n", muic_data->reg[CHG_TYPE]);
}

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
int s2mu205_muic_bcd_rescan(struct s2mu205_muic_data *muic_data)
{
	int ret = 0;

	pr_info("%s call\n", __func__);
	muic_data->rescan_cnt++;
	/* muic mux switch open */
	ret = _s2mu205_muic_sel_path(muic_data, S2MU205_PATH_OPEN);
	if (ret < 0)
		pr_err("%s, fail to open mansw\n", __func__);

	_s2mu205_muic_set_chg_det(muic_data, true);
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	_s2mu205_muic_control_rid_adc(muic_data, MUIC_DISABLE);
	msleep(150);
	cancel_delayed_work(&muic_data->rescan_validity_checker);
	schedule_delayed_work(&muic_data->rescan_validity_checker,
		msecs_to_jiffies(1200));
#endif /* CONFIG_S2MU205_TYPEC_WATER */
	_s2mu205_muic_set_bcd_rescan_reg(muic_data);

    return 0;
}
#endif

static void s2mu205_muic_dcd_recheck(struct work_struct *work)
{
	struct s2mu205_muic_data *muic_data =
		container_of(work, struct s2mu205_muic_data, dcd_recheck.work);
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	/* Driver re-detects the rescan type. */
	int det_ret = S2MU205_DETECT_NONE;

	if (muic_core_get_ccic_cable_state(muic_pdata)) {
		pr_info("%s Tried to rescan, but cc type detected.\n", __func__);
		return;
	}

	mutex_lock(&muic_data->bcd_rescan_mutex);
	if (!muic_data->vbvolt) {
		goto skip_dcd_recheck;
	}

	s2mu205_muic_bcd_rescan(muic_data);
	msleep(650);
	s2mu205_muic_get_detect_info(muic_data);

	if (!muic_data->vbvolt || muic_core_get_ccic_cable_state(muic_data->pdata)) {
		goto skip_dcd_recheck;
	}

	/* Detect Type & Handle the result */
	det_ret = s2mu205_muic_detect_dev_bc1p2(muic_data);

	if (det_ret == S2MU205_DETECT_NONE) {
		pr_err("%s abnormal detection : set general TA\n", __func__);
		muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
	}
	s2mu205_muic_handle_attached_dev(muic_data);

skip_dcd_recheck:
	mutex_unlock(&muic_data->bcd_rescan_mutex);
}

/*
 * Interface Functions
 */
#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
static int s2mu205_if_get_adc(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);

	ret = _s2mu205_muic_get_rid_adc(muic_data);

	if (ret == ADC_OPEN && muic_data->is_cable_inserted)
		ret = ADC_GND;

	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}
#endif

#if defined(CONFIG_MUIC_SUPPORT_PRSWAP)
static void s2mu205_if_prswap_work(void *mdata, int mode)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int adc = 0, vbvolt = 0;

	pr_info("%s+ dev(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	if (muic_pdata->attached_dev == ATTACHED_DEV_USB_MUIC
			|| muic_pdata->attached_dev == ATTACHED_DEV_OTG_MUIC
			|| muic_pdata->attached_dev == ATTACHED_DEV_TIMEOUT_OPEN_MUIC) {
		pr_err("%s(%d) invalid status\n", __func__, __LINE__);
		goto work_done;
	}

	mutex_lock(&muic_data->muic_mutex);
	_s2mu205_muic_set_chg_det(muic_data, false);
	_s2mu205_muic_sel_path(muic_data, S2MU205_PATH_USB);

	switch (mode) {
		case MUIC_PRSWAP_TO_SINK:
			adc = _s2mu205_muic_get_rid_adc(muic_data);
			vbvolt = _s2mu205_muic_get_vbus_state(muic_data);
			muic_core_handle_attach(muic_data->pdata, ATTACHED_DEV_USB_MUIC,
				adc, !!vbvolt);
			break;
		case MUIC_PRSWAP_TO_SRC:
			muic_pdata->attached_dev = ATTACHED_DEV_OTG_MUIC;
			break;
		default:
			pr_err("%s(%d) invalid value\n", __func__, __LINE__);
			goto work_done;
			break;
	}
	mutex_unlock(&muic_data->muic_mutex);
work_done:
	pr_info("%s- dev(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));
}
#endif

static int s2mu205_if_com_to_open(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);

	ret = _s2mu205_muic_sel_path(muic_data, S2MU205_PATH_OPEN);
	if (ret)
		pr_err("%s set_com_open err\n", __func__);

	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu205_if_com_to_usb(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);

	ret = _s2mu205_muic_sel_path(muic_data, S2MU205_PATH_USB);
	if (ret)
		pr_err("%s set_com_usb err\n", __func__);

	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu205_if_com_to_uart(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);

	ret = _s2mu205_muic_com_to_uart(muic_data);
	if (ret)
		pr_err("%s set_com_uart err\n", __func__);

	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu205_if_get_vbus(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	return _s2mu205_muic_get_vbus_state(muic_data);
}

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
static void s2mu205_if_set_cable_state(void *mdata, muic_attached_dev_t new_dev)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	s2mu205_muic_detect_dev_ccic(muic_data, new_dev);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);
}

static void s2mu205_if_cable_recheck(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);
	mutex_lock(&muic_data->switch_mutex);

	s2mu205_muic_bcd_rescan(muic_data);

	mutex_unlock(&muic_data->switch_mutex);
	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);
}

static int s2mu205_if_check_usb_killer(void *mdata)
{
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	return MUIC_NORMAL_OTG;
#else
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;
	struct i2c_client *i2c = muic_data->i2c;
	int ret = MUIC_NORMAL_OTG;
	u8 reg_val = 0;
	bool lc_cable_inserted = false;

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	int wait_ret = 0;
	u8 muic_int1_mask = 0, muic_int2_mask = 0, muic_ctrl2 = 0;
	u8 adc = 0;

	if (muic_data->water_status != S2MU205_WATER_MUIC_IDLE)
		return MUIC_ABNORMAL_OTG;

	if (!muic_data->is_cable_inserted) {
		muic_ctrl2 = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_CTRL2);
		if (muic_ctrl2 & MUIC_CTRL2_ADC_OFF_MASK) {
			muic_int1_mask = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_INT1_MASK);
			muic_int2_mask = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_INT2_MASK);

			s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MUIC_INT1_MASK, 0xff);
			s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MUIC_INT2_MASK, 0xff);
			_s2mu205_muic_control_rid_adc(muic_data, true);
			msleep(100);

			adc = _s2mu205_muic_get_rid_adc(muic_data);
			pr_info("%s adc(%d)\n", __func__, adc);

			_s2mu205_muic_control_rid_adc(muic_data, false);
			usleep_range(20000, 22000);

			muic_int1_mask = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_INT1);
			muic_int2_mask = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_INT2);
			pr_info("%s muic_int(%#x, %#x)\n", __func__, muic_int1_mask, muic_int2_mask);
			s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MUIC_INT1_MASK, muic_int1_mask);
			s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MUIC_INT2_MASK, muic_int2_mask);

			if (adc != ADC_GND) {
				pr_err("%s not cable state.\n", __func__);
				return MUIC_ABNORMAL_OTG;
			} else {
				lc_cable_inserted = true;
			}
		} else {
			wait_ret = wait_event_interruptible_timeout(muic_data->cable_wait,
				muic_data->is_cable_inserted == true,
				msecs_to_jiffies(300));

			if ((wait_ret < 0) || (!wait_ret)) {
				pr_err("%s not cable state.\n", __func__);
				return MUIC_ABNORMAL_OTG;
			}
			pr_info("%s, cable detected after while.", __func__);
		}
	}
#endif
	pr_info("%s entered\n", __func__);

	/* Set Data Path to Open. */
	reg_val = s2mu205_i2c_read_byte(i2c, S2MU205_REG_MANUAL_SW_CTRL);
	if (reg_val & MANUAL_SW_CTRL_DM_SWITCHING_MASK) {
		ret = _s2mu205_muic_sel_path(muic_data, S2MU205_PATH_OPEN);
		msleep(50);
	}

	if (!muic_data->is_cable_inserted && lc_cable_inserted == false) {
		pr_info("%s Not Cable.\n", __func__);
		return MUIC_ABNORMAL_OTG;
	}

	/* Release Killer INT Mask. */
	s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT1_MASK, INT_USBKILLER_MASK1);

	/* Trigger BC 1.2 */
	_s2mu205_i2c_update_bit(i2c, S2MU205_REG_LDOADC_VSETL,
			LDOADC_VSETL_CHG_TYPE_DET_ALWAYS_ONB_MASK,
			LDOADC_VSETL_CHG_TYPE_DET_ALWAYS_ONB_SHIFT, 0x0);
	_s2mu205_i2c_update_bit(i2c, S2MU205_REG_CHARGER_DET_OTP,
			CHARGER_DET_OTP_MAN_VBUSTMR_ON_MASK,
			CHARGER_DET_OTP_MAN_VBUSTMR_ON_SHIFT, 0x1);
	msleep(150);

	/* Readback Dev Type */
	reg_val = s2mu205_i2c_read_byte(i2c, S2MU205_REG_DEVICE_TYP1);
	if (reg_val & DEVICE_TYP1_USB_MASK && reg_val & DEVICE_TYP1_DCPCHG_MASK) {
		ret = MUIC_NORMAL_OTG;
	} else if (reg_val & DEVICE_TYP1_DCPCHG_MASK) {
		_s2mu205_i2c_update_bit(i2c, S2MU205_REG_LDOADC_VSETH,
			LDOADC_VSETH_LDOADC_VSETH_MASK,
			LDOADC_VSETH_LDOADC_VSETH_SHIFT,
			LDOADC_VSETH_LDOADC_VSETH_maxV);

		_s2mu205_i2c_update_bit(i2c, S2MU205_REG_MUIC_RSVD1,
			MUIC_RSVD1_USBKILL_EN_MASK,
			MUIC_RSVD1_USBKILL_EN_SHIFT, 0x1);
		muic_data->killer_status = S2MU205_KILLER_WAIT_STATUS;
		pr_info("%s Triggered the IRQ\n", __func__);

		msleep(40);

		if (muic_data->killer_status == S2MU205_KILLER_DETECTED) {
			ret = MUIC_ABNORMAL_OTG;
		} else {
			muic_data->killer_status = S2MU205_KILLER_NONE;
			ret = MUIC_NORMAL_OTG;
		}

		_s2mu205_i2c_update_bit(i2c, S2MU205_REG_LDOADC_VSETH,
				LDOADC_VSETH_LDOADC_VSETH_MASK,
				LDOADC_VSETH_LDOADC_VSETH_SHIFT,
				LDOADC_VSETH_LDOADC_VSETH_2p8V);

		_s2mu205_i2c_update_bit(i2c, S2MU205_REG_MUIC_RSVD1,
				MUIC_RSVD1_USBKILL_EN_MASK,
				MUIC_RSVD1_USBKILL_EN_SHIFT, 0x0);
	} else {
		ret = MUIC_NORMAL_OTG;
	}

	s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT1_MASK, INT_MUIC_MASK1);

	_s2mu205_i2c_update_bit(i2c, S2MU205_REG_LDOADC_VSETL,
		LDOADC_VSETL_CHG_TYPE_DET_ALWAYS_ONB_MASK,
		LDOADC_VSETL_CHG_TYPE_DET_ALWAYS_ONB_SHIFT, 0x1);

	_s2mu205_i2c_update_bit(i2c, S2MU205_REG_CHARGER_DET_OTP,
		CHARGER_DET_OTP_MAN_VBUSTMR_ON_MASK,
		CHARGER_DET_OTP_MAN_VBUSTMR_ON_SHIFT, 0x0);

	if (muic_data->water_status != S2MU205_WATER_MUIC_IDLE)
		return MUIC_ABNORMAL_OTG;

	return ret;
#endif
}
#endif

static int s2mu205_if_set_bcd_rescan_reg(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	return _s2mu205_muic_set_bcd_rescan_reg(muic_data);
}

static int s2mu205_if_control_rid_adc(void *mdata, bool enable)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	return _s2mu205_muic_control_rid_adc(muic_data, enable);
}

static int s2mu205_if_set_gpio_uart_sel(void *mdata, int uart_path)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	return s2mu205_set_gpio_uart_sel(muic_data, uart_path);
}

static int s2mu205_if_set_jig_ctrl_on(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	return _s2mu205_muic_set_jig_on(muic_data);
}

#if IS_ENABLED(CONFIG_MUIC_SYSFS)
static int s2mu205_if_show_register(void *mdata, char *mesg)
{
#if IS_ENABLED(CONFIG_MUIC_DEBUG)
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	if (mesg != NULL)
		s2mu205_read_reg_dump(muic_data, mesg);
	pr_info("%s %s\n", __func__, mesg);
#endif
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
static void s2mu205_if_set_water_det(void *mdata, bool val)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	pr_info("%s water_status : (%d)\n", __func__, muic_data->water_status);
	if (muic_data->water_dry_status == S2MU205_WATER_DRY_MUIC_DET) {
		if (val)
			muic_data->water_dry_status = S2MU205_WATER_DRY_MUIC_CCIC_INVALID;
		else
			muic_data->water_dry_status = S2MU205_WATER_DRY_MUIC_CCIC_DET;
		wake_up_interruptible(&muic_data->wait);
	} else {
		if (muic_data->water_status >= S2MU205_WATER_MUIC_DET) {
			if (val)
				muic_data->water_status = S2MU205_WATER_MUIC_CCIC_DET;
			else
				muic_data->water_status = S2MU205_WATER_MUIC_CCIC_INVALID;
			pr_info("%s water_status : (%d)\n", __func__,
				muic_data->water_status);
			wake_up_interruptible(&muic_data->wait);
		} else {
			pr_err("%s wrong status\n", __func__);
		}
	}
	mutex_unlock(&muic_data->muic_mutex);
}

#if !IS_ENABLED(CONFIG_SEC_FACTORY)
static void s2mu205_if_set_water_det_from_boot(void *mdata, bool val)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	if (val)
		s2mu205_muic_set_water_state(muic_data, MUIC_ENABLE);
}
#endif
#endif

#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
static int s2mu205_if_set_hiccup_mode(void *mdata, bool val)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;
	muic_data->is_hiccup_mode = val;
	return 0;
}

static int s2mu205_if_get_hiccup_mode(void *mdata)
{
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;
	return muic_data->is_hiccup_mode;
}
#endif

static void s2mu205_if_set_bypass(void *mdata)
{
	/*
	 * To prevent charging operation after entering bypass mode
	 */
	struct s2mu205_muic_data *muic_data = (struct s2mu205_muic_data *)mdata;

	pr_info("%s\n", __func__);

	s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MUIC_INT1_MASK, 0xff);
	s2mu205_i2c_write_byte(muic_data->i2c, S2MU205_REG_MUIC_INT2_MASK, 0xff);

	muic_core_handle_detach(muic_data->pdata);
}

int s2mu205_set_gpio_uart_sel(struct s2mu205_muic_data *muic_data, int uart_sel)
{
	const char *mode;
	struct muic_platform_data *muic_pdata = muic_data->pdata;
#if !IS_ENABLED(CONFIG_MUIC_UART_SWITCH)
	int uart_sel_gpio = muic_pdata->gpio_uart_sel;
	int uart_sel_val;
	int ret;

	ret = gpio_request(uart_sel_gpio, "GPIO_UART_SEL");
	if (ret) {
		pr_err("failed to gpio_request GPIO_UART_SEL\n");
		return ret;
	}

	uart_sel_val = gpio_get_value(uart_sel_gpio);

	switch (uart_sel) {
	case MUIC_PATH_UART_AP:
		mode = "AP_UART";
		if (gpio_is_valid(uart_sel_gpio))
			gpio_direction_output(uart_sel_gpio, 1);
		break;
	case MUIC_PATH_UART_CP:
		mode = "CP_UART";
		if (gpio_is_valid(uart_sel_gpio))
			gpio_direction_output(uart_sel_gpio, 0);
		break;
	default:
		mode = "Error";
		break;
	}

	uart_sel_val = gpio_get_value(uart_sel_gpio);

	gpio_free(uart_sel_gpio);

	pr_info("%s, GPIO_UART_SEL(%d)=%c\n",
		mode, uart_sel_gpio, (uart_sel_val == 0 ? 'L' : 'H'));
#else
	switch (uart_sel) {
	case MUIC_PATH_UART_AP:
		mode = "AP_UART";
		pin_config_set(muic_pdata.uart_addr, muic_pdata.uart_rxd,
			       PINCFG_PACK(PINCFG_TYPE_FUNC, 0x2));
		pin_config_set(muic_pdata.uart_addr, muic_pdata.uart_txd,
			       PINCFG_PACK(PINCFG_TYPE_FUNC, 0x2));
		break;
	case MUIC_PATH_UART_CP:
		mode = "CP_UART";
		pin_config_set(muic_pdata.uart_addr, muic_pdata.uart_rxd,
			       PINCFG_PACK(PINCFG_TYPE_FUNC, 0x3));
		pin_config_set(muic_pdata.uart_addr, muic_pdata.uart_txd,
			       PINCFG_PACK(PINCFG_TYPE_FUNC, 0x3));
		break;
	default:
		mode = "Error";
		break;
	}

	pr_info("%s %s\n", __func__, mode);
#endif /* CONFIG_MUIC_UART_SWITCH */
	return 0;
}

static int s2mu205_muic_reg_init(struct s2mu205_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0, data = 0;
	u8 reg_val = 0;

	pr_info("%s\n", __func__);

	s2mu205_muic_get_detect_info(muic_data);

	s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT1_MASK, INT_MUIC_MASK1);
	s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT2_MASK, INT_MUIC_MASK2);

	reg_val = s2mu205_i2c_read_byte(i2c, S2MU205_REG_TIMER_SET3);
	reg_val &= ~TIMER_SET3_DCDTMRSET_MASK;
	reg_val |= TIMER_SET3_DCDTMRSET_300ms;
	pr_info("%s(%d) reg_val(%#x)\n", __func__, __LINE__, reg_val);
	s2mu205_i2c_write_byte(i2c, S2MU205_REG_TIMER_SET3, reg_val);

#if !IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
	/* Masking ADC, RID interrupt */
	reg_val = s2mu205_i2c_read_byte(i2c, S2MU205_REG_MUIC_INT1_MASK);
	reg_val |= MUIC_INT1_MASK_RID_CHG_Im_MASK;
	s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT1_MASK, reg_val);

	reg_val = s2mu205_i2c_read_byte(i2c, S2MU205_REG_MUIC_INT2_MASK);
	reg_val |= MUIC_INT2_MASK_ADCCHANGE_Im_MASK;
	s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT2_MASK, reg_val);
#endif
	_s2mu205_muic_set_chg_det(muic_data, false);

	/* for usb killer */
	_s2mu205_i2c_update_bit(muic_data->i2c, S2MU205_REG_RID_DISCHARGE,
		RID_DISCHARGE_USBKILL_OTG_OPTION_MASK,
		RID_DISCHARGE_USBKILL_OTG_OPTION_SHIFT, 0x1);
#if IS_ENABLED(CONFIG_MUIC_S2MU205_ENABLE_AUTOSW)
	reg_val = MUIC_CTRL1_SWITCH_OPEN_MASK | MUIC_CTRL1_WAIT_MASK
		| MUIC_CTRL1_MANUAL_SW_MASK;
#else
	reg_val = MUIC_CTRL1_SWITCH_OPEN_MASK | MUIC_CTRL1_WAIT_MASK;
#endif
	ret = _s2mu205_i2c_guaranteed_wbyte(i2c, S2MU205_REG_MUIC_CTRL1, reg_val);
	if (ret < 0)
		pr_err("failed to write ctrl(%d)\n", ret);

	data = s2mu205_i2c_read_byte(i2c, S2MU205_REG_MANUAL_SW_CTRL);
	pr_info("%s init path(0x%x)\n", __func__, data);
	if (data)
		_s2mu205_muic_sel_path(muic_data, S2MU205_PATH_OPEN);

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	/*
	 * These registers represents the RID ADC LDO voltage control.
	 * Low / High LDO initialized to 3V, 2.7V each.
	 */
	_s2mu205_muic_control_rid_adc(muic_data, MUIC_ENABLE);

	_s2mu205_i2c_update_bit(muic_data->i2c, S2MU205_REG_RID_DISCHARGE,
		RID_DISCHARGE_RID_DISCHARGE_ON_MASK,
		RID_DISCHARGE_RID_DISCHARGE_ON_SHIFT, 0x1);

	s2mu205_muic_set_rid_for_water(muic_data, MUIC_DISABLE);
#endif

#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL) && !IS_ENABLED(CONFIG_SEC_FACTORY)
	/* Masking ADC, RID interrupt */
	reg_val = s2mu205_i2c_read_byte(i2c, S2MU205_REG_MUIC_INT1_MASK);
	reg_val |= MUIC_INT1_MASK_RID_CHG_Im_MASK;
	s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT1_MASK, reg_val);

	data = s2mu205_i2c_read_byte(i2c, S2MU205_REG_MUIC_INT2_MASK);
	data |= MUIC_INT2_MASK_ADCCHANGE_Im_MASK;
	s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT2_MASK, data);
	pr_info("%s INT1_MASK(%#x), INT2_MASK(%#x)\n", __func__, reg_val, data);
#endif

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle((!!muic_data->vbvolt) ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */

	return ret;
}

static void s2mu205_muic_handle_legacy_detach(struct s2mu205_muic_data *muic_data)
{
	pr_info("%s\n", __func__);

	_s2mu205_muic_set_chg_det(muic_data, false);
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	muic_data->rescan_cnt = 0;
	_s2mu205_muic_control_rid_adc(muic_data, MUIC_ENABLE);
	if (muic_data->is_cable_inserted) {
		muic_data->is_cable_inserted = false;
	}
	s2mu205_muic_set_rid_int_mask_en(muic_data, MUIC_DISABLE);
#endif
}

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
static void s2mu205_muic_detect_dev_ccic(struct s2mu205_muic_data *muic_data,
		muic_attached_dev_t new_dev)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int adc = 0, vbvolt = 0;

	pr_info("%s new dev(%s)\n", __func__, dev_to_str(new_dev));

	if (muic_pdata->attached_dev == new_dev) {
		pr_err("%s: Skip to handle duplicated type\n", __func__);
		return;
	}

	if (new_dev == ATTACHED_DEV_NONE_MUIC) {
		/* Detach from CCIC */
		if (muic_core_get_ccic_cable_state(muic_data->pdata) == false) {
			pr_err("%s: Skip to detach legacy type\n", __func__);
			return;
		}
		muic_pdata->attached_dev = ATTACHED_DEV_NONE_MUIC;
		muic_data->killer_status = S2MU205_KILLER_NONE;
		if (!muic_data->is_cable_inserted) {
			s2mu205_muic_set_rid_int_mask_en(muic_data, MUIC_DISABLE);
		}
#if IS_ENABLED(CONFIG_MUIC_SUPPORT_PRSWAP)
		if (muic_pdata->attached_dev == ATTACHED_DEV_USB_MUIC) {
			s2mu205_muic_handle_legacy_detach(muic_data);
		}
#endif
	} else {
		/* Attach from CCIC */
		pr_info("%s DETECTED\n", dev_to_str(new_dev));

#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		muic_data->is_timeout_attached = false;
#endif

		switch (new_dev) {
		case ATTACHED_DEV_OTG_MUIC:
			_s2mu205_muic_sel_path(muic_data, S2MU205_PATH_USB);
			break;
		case ATTACHED_DEV_TYPE3_CHARGER_MUIC: /* PD Charger */
			_s2mu205_muic_sel_path(muic_data, S2MU205_PATH_OPEN);
			return;
		default:
			break;
		}
	}

	if (new_dev != ATTACHED_DEV_NONE_MUIC) {
		adc = _s2mu205_muic_get_rid_adc(muic_data);
		vbvolt = _s2mu205_muic_get_vbus_state(muic_data);
		muic_core_handle_attach(muic_data->pdata, new_dev, adc, !!vbvolt);
		muic_pdata->attached_dev = new_dev;
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
		_s2mu205_muic_control_rid_adc(muic_data, MUIC_ENABLE);
#endif
	} else if (muic_pdata->attached_dev == ATTACHED_DEV_NONE_MUIC) {
		muic_core_handle_detach(muic_data->pdata);
	}
}
#endif

static int s2mu205_muic_detect_dev_bc1p2(struct s2mu205_muic_data *muic_data)
{
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;

	muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;

	/* Attached */
	switch (muic_data->reg[DEVICE_TYPE1]) {
	case DEVICE_TYP1_CDPCHG_MASK:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_CDP_MUIC;
			pr_info("USB_CDP DETECTED\n");
		}
		break;
	case DEVICE_TYP1_USB_MASK:
		if (muic_data->vbvolt) {
			pr_info("USB DETECTED\n");
				muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
		}
		break;
	case DEVICE_TYP1_DCPCHG_MASK:
	case DEVICE_TYP1_DCPCHG_MASK | DEVICE_TYP1_USB_MASK:
	case DEVICE_TYP1_DCPCHG_MASK | DEVICE_TYP1_CDPCHG_MASK:
		if (muic_data->vbvolt) {
			muic_if->is_dcp_charger = true;
			muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
			muic_data->afc_check = true;
			pr_info("DEDICATED CHARGER DETECTED\n");
		}
		break;
#if IS_ENABLED(CONFIG_MUIC_SUPPORT_TYPEB) && IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
	case DEVICE_TYP1_USBOTG_MASK:
		muic_data->new_dev = ATTACHED_DEV_OTG_MUIC;
		pr_info("USB_OTG DETECTED\n");
		break;
#endif
	case DEVICE_TYP1_CARKIT_MASK:
		if (muic_data->vbvolt) {
			/* 200K, 442K should be checkef */
			if (muic_data->adc == ADC_CEA936ATYPE2_CHG) {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
				pr_info("CEA936ATYPE2_CHG DETECTED\n");
				muic_data->afc_check = false;
			} else {
				muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
				pr_info("T1_T2_CHG DETECTED\n");
			}
		}
		break;
	default:
		break;
	}

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC &&
			muic_data->new_dev != ATTACHED_DEV_NONE_MUIC)
		goto detect_done;

	switch (muic_data->reg[DEVICE_TYPE2]) {
	case DEVICE_TYP2_SDP_1P8S_MASK:
		if (muic_data->vbvolt) {
#if IS_ENABLED(CONFIG_SEC_FACTORY)
			pr_info("%s:%s: SDP_1P8S=>USB DETECTED\n", MUIC_DEV_NAME, __func__);
			muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
#else
			pr_info("SDP_1P8S DETECTED\n");
			muic_data->new_dev = ATTACHED_DEV_TIMEOUT_OPEN_MUIC;
#endif
		}
		break;
	default:
		break;
	}

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC &&
			muic_data->new_dev != ATTACHED_DEV_NONE_MUIC)
		goto detect_done;

	if (muic_data->vbvolt &&
			((muic_data->reg[DEVICE_APPLE] & DEVICE_APPLE_APPLE2P4A_CHG_MASK)
			|| (muic_data->reg[DEVICE_APPLE] & DEVICE_APPLE_APPLE2A_CHG_MASK)
			|| (muic_data->reg[DEVICE_APPLE] & DEVICE_APPLE_APPLE1A_CHG_MASK)
			|| (muic_data->reg[DEVICE_APPLE] & DEVICE_APPLE_APPLE0P5A_CHG_MASK))) {
		pr_info("APPLE_CHG DETECTED\n");
		muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
		muic_data->afc_check = false;
	}

	if ((muic_data->reg[CHG_TYPE] & DEV_TYPE_CHG_TYPE) &&
			(muic_data->new_dev == ATTACHED_DEV_UNKNOWN_MUIC)) {
		muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
		muic_data->afc_check = false;
		pr_info("CHG_TYPE DETECTED\n");
		if (muic_data->rescan_cnt > 1) {
			muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
			muic_data->afc_check = false;
		} else {
			schedule_delayed_work(&muic_data->dcd_recheck, msecs_to_jiffies(100));
			return S2MU205_DETECT_SKIP;
		}
	}

detect_done:
	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC)
		return S2MU205_DETECT_DONE;
	else
		return S2MU205_DETECT_NONE;
}

#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
static int s2mu205_muic_detect_dev_rid_array(struct s2mu205_muic_data *muic_data)
{
	muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
	switch (muic_data->adc) {
	case ADC_JIG_UART_OFF:
		pr_info("ADC_JIG_UART_OFF\n");
		if (muic_data->vbvolt)
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		else
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		break;
	case ADC_JIG_USB_ON:
	case ADC_JIG_USB_OFF:
	case ADC_DESKDOCK:
		muic_data->new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		pr_info("ADC JIG_USB_ON DETECTED\n");
		break;
	case ADC_JIG_UART_ON:
		muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
		pr_info("ADC JIG_UART_ON DETECTED\n");
		break;
	default:
		pr_info("%s unsupported ADC(0x%02x)\n", __func__, muic_data->adc);
	}

	return muic_data->new_dev;
}

#if IS_ENABLED(CONFIG_MUIC_SUPPORT_TYPEB)
static int s2mu205_muic_detect_dev_jig_type(struct s2mu205_muic_data *muic_data)
{
	switch (muic_data->reg[DEVICE_TYPE2]) {
	case DEVICE_TYP2_JIGUARTOFF_MASK:
		if (muic_data->vbvolt)
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		else
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		pr_info("JIG_UART_OFF DETECTED\n");
		break;
	case DEVICE_TYP2_JIGUSBOFF_MASK:
		if (!muic_data->vbvolt)
			break;
		muic_data->new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
		pr_info("JIG_USB_OFF DETECTED\n");
		break;
	case DEVICE_TYP2_JIGUSBON_MASK:
		if (!muic_data->vbvolt)
			break;
		muic_data->new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		pr_info("JIG_USB_ON DETECTED\n");
		break;
	case DEVICE_TYP2_JIGUARTON_MASK:
		if (muic_data->new_dev != ATTACHED_DEV_JIG_UART_ON_MUIC) {
			if (!muic_data->vbvolt) {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
				pr_info("ADC JIG_UART_ON DETECTED\n");
			} else {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
				pr_info("ADC JIG_UART_ON_VB DETECTED\n");
			}
		}
		break;
	default:
		break;
	}

	return muic_data->new_dev;
}

static int s2mu205_muic_detect_dev_rid_device(struct s2mu205_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;

	switch (muic_data->adc) {
	case ADC_CEA936ATYPE1_CHG:	/*200k ohm */
		if (muic_data->vbvolt) {
			/* This is workaournd for LG USB cable which has 219k ohm ID */
			muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("TYPE1 CHARGER DETECTED(USB)\n");
		}
		break;
	case ADC_CEA936ATYPE2_CHG:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
			muic_data->afc_check = false;
			pr_info("%s unsupported ADC(0x%02x)\n", __func__, muic_data->adc);
		}
		break;
	case ADC_JIG_USB_OFF:	/* 255k */
		if (!muic_data->vbvolt)
			break;
		if (muic_data->new_dev != ATTACHED_DEV_JIG_USB_OFF_MUIC) {
			muic_data->new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
			pr_info("ADC JIG_USB_OFF DETECTED\n");
		}
		break;
	case ADC_JIG_USB_ON:
		if (!muic_data->vbvolt)
			break;
		if (muic_data->new_dev != ATTACHED_DEV_JIG_USB_ON_MUIC) {
			if (!muic_data->vbvolt) {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
				pr_info("ADC JIG_UART_ON DETECTED\n");
			} else {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
				pr_info("ADC JIG_UART_ON_VB DETECTED\n");
			}
		}
		break;
	case ADC_JIG_UART_OFF:
		if (muic_pdata->is_otg_test) {
			mdelay(100);
			if (muic_data->vmid == 0x4) {
				pr_info("OTG_TEST DETECTED, vmid = %d\n", muic_data->vmid);
				muic_data->vbvolt = 1;
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC;
			} else
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		} else if (muic_data->vbvolt)
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		else
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;

		pr_info("ADC JIG_UART_OFF DETECTED\n");
		break;
	case ADC_JIG_UART_ON:
		if (muic_data->new_dev != ATTACHED_DEV_JIG_UART_ON_MUIC) {
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
			pr_info("ADC JIG_UART_ON DETECTED\n");
		}
		break;
	case ADC_SMARTDOCK: /* 0x10000 40.2K ohm */
		/* SMARTDOCK is not supported */
		/* force not to charge the device with SMARTDOCK */
		muic_data->new_dev = ATTACHED_DEV_UNSUPPORTED_ID_VB_MUIC;
		pr_info("%s unsupported ADC(0x%02x) but charging\n",
			__func__, muic_data->adc);
		break;
	case ADC_HMT:	/* 0x10001 49.9K ohm */
		muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
		pr_info("%s unsupported ADC(0x%02x) not charging\n",
			__func__, muic_data->adc);
		break;
	case ADC_AUDIODOCK:
#if IS_ENABLED(CONFIG_MUIC_S2MU205_SUPPORT_AUDIODOCK)
		muic_data->new_dev = ATTACHED_DEV_AUDIODOCK_MUIC;
#else
		muic_data->new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
#endif
		pr_info("ADC AUDIODOCK DETECTED\n");
		break;
	case ADC_UNIVERSAL_MMDOCK:
		muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
		pr_info("%s unsupported ADC(0x%02x) not charging\n",
			__func__, muic_data->adc);
		break;
	case ADC_OPEN:
		/* sometimes muic fails to catch JIG_UART_OFF detaching */
		/* double check with ADC */
		if (muic_data->new_dev == ATTACHED_DEV_JIG_UART_OFF_MUIC) {
			muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
			pr_info("ADC OPEN DETECTED\n");
		}
		break;
	default:
		muic_data->new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
		pr_info("%s unsupported ADC(0x%02x)\n", __func__, muic_data->adc);
	}

	return muic_data->new_dev;
}

static int s2mu205_muic_detect_dev_mrid_adc(struct s2mu205_muic_data *muic_data)
{
	muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;

	if (muic_data->adc & ADC_CONVERSION_ERR_MASK) {
		pr_info("%s ADC conversion error! (adc=%#x)\n", __func__, muic_data->adc);
		return S2MU205_DETECT_NONE;
	}

	muic_data->new_dev = s2mu205_muic_detect_dev_jig_type(muic_data);
	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC)
		return S2MU205_DETECT_DONE;

	return S2MU205_DETECT_NONE;
}
#endif /* CONFIG_MUIC_SUPPORT_TYPEB */
#endif /* CONFIG_MUIC_S2MU205_RID */

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
static void s2mu205_muic_set_hiccup_mode(struct s2mu205_muic_data *muic_data, bool en)
{
	pr_info("%s en : %d\n",	__func__, (int)en);
	muic_data->is_hiccup_mode = en;
	_s2mu205_muic_sel_path(muic_data, S2MU205_PATH_OPEN);
}
#endif

static void s2mu205_muic_set_rid_int_mask_en(struct s2mu205_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c = muic_data->i2c;

	pr_info("%s en : %d\n",	__func__, (int)en);
	if (en) {
		s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT1_MASK, INT_WATER_MASK1);
		s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT2_MASK, INT_WATER_MASK2);
	} else {
		s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT1_MASK, INT_MUIC_MASK1);
		s2mu205_i2c_write_byte(i2c, S2MU205_REG_MUIC_INT2_MASK, INT_MUIC_MASK2);
	}
}

static void s2mu205_muic_set_rid_for_water(struct s2mu205_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c;
	u8 reg_rid_ldo;

	if (muic_data == NULL) {
		pr_err("%s no data\n",	__func__);
		return;
	}
	i2c = muic_data->i2c;

	reg_rid_ldo = s2mu205_i2c_read_byte(i2c, S2MU205_REG_RID_DISCHARGE);

	pr_info("%s en : %d\n",	__func__, (int)en);
	if (en) {
		reg_rid_ldo |= ADCBIAS_OTP4_ADC_STDBY_BIAS_SEL_MASK;
	} else {
		reg_rid_ldo &= ~ADCBIAS_OTP4_ADC_STDBY_BIAS_SEL_MASK;
	}

	s2mu205_i2c_write_byte(i2c, S2MU205_REG_RID_DISCHARGE, reg_rid_ldo);
}

static void s2mu205_muic_put_dry_chk_time(struct s2mu205_muic_data *muic_data)
{
	struct timeval time;

	do_gettimeofday(&time);
	pr_info("%s Dry check time : %ld\n", __func__, (long)time.tv_sec);
	muic_data->dry_chk_time = (long)time.tv_sec;
}

#if !IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
static int s2mu205_muic_water_judge(struct s2mu205_muic_data *muic_data)
{
	int i, adc = 0;

	pr_info("%s : enter\n", __func__);
	for (i = 0; i < WATER_DET_RETRY_CNT; i++) {
		adc = _s2mu205_muic_recheck_adc(muic_data);
		if (adc == ADC_GND) {
			pr_info("%s : NOT WATER\n", __func__);
			return adc;
		}
		if (_s2mu205_muic_get_vbus_state(muic_data)) {
			pr_info("%s : vbus while detecting\n", __func__);
			return ADC_GND;
		}

		pr_info("%s : %d st try : adc(0x%x)\n", __func__, i, adc);
	}

	return adc;
}
#endif

static void s2mu205_muic_set_water_state(struct s2mu205_muic_data *muic_data, bool en)
{
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	int vbvolt = 0;
#endif
	pr_info("%s en : %d\n",	__func__, (int)en);
	if (en) {
		pr_info("%s: WATER DETECT!!!\n", __func__);
		muic_data->dry_cnt = 0;
		muic_data->dry_duration_sec = WATER_DRY_RETRY_INTERVAL_SEC;
		MUIC_SEND_NOTI_ATTACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
		muic_data->water_status = S2MU205_WATER_MUIC_CCIC_STABLE;
		muic_data->water_dry_status = S2MU205_WATER_DRY_MUIC_IDLE;
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
		vbvolt = _s2mu205_muic_get_vbus_state(muic_data);
		vbus_notifier_handle(vbvolt ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
		s2mu205_muic_put_dry_chk_time(muic_data);
		cancel_delayed_work(&muic_data->water_dry_handler);
		schedule_delayed_work(&muic_data->water_dry_handler,
				msecs_to_jiffies(1800000));
		_s2mu205_muic_control_rid_adc(muic_data, MUIC_DISABLE);
		pr_info("%s %d WATER DETECT stabled adc : 0x%X\n", __func__, __LINE__, muic_data->adc);
	} else {
		pr_info("%s WATER DRIED!!!\n", __func__);
		muic_data->water_status = S2MU205_WATER_MUIC_IDLE;
		muic_data->water_dry_status = S2MU205_WATER_DRY_MUIC_IDLE;
#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
		s2mu205_muic_set_hiccup_mode(muic_data, MUIC_DISABLE);
#endif
		muic_data->dry_duration_sec = WATER_DRY_RETRY_INTERVAL_SEC;
		muic_data->dry_cnt = 0;
		MUIC_SEND_NOTI_DETACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
		MUIC_SEND_NOTI_TO_CCIC_DETACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
	}
}

static void s2mu205_muic_water_detect_handler(struct work_struct *work)
{
	struct s2mu205_muic_data *muic_data =
		container_of(work, struct s2mu205_muic_data, water_detect_handler.work);
	int wait_ret = 0;

	mutex_lock(&muic_data->water_det_mutex);
	wake_lock(&muic_data->water_wake_lock);
	if (muic_data->water_status != S2MU205_WATER_MUIC_VERIFY) {
		pr_info("%s %d exit detect, due to status mismatch\n", __func__, __LINE__);
		goto EXIT_DETECT;
	}

	pr_info("%s\n", __func__);

	muic_data->water_status = S2MU205_WATER_MUIC_DET;
	_s2mu205_muic_control_rid_adc(muic_data, false);
	MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_CHK_WATER_REQ);

	wait_ret = wait_event_interruptible_timeout(muic_data->wait,
		muic_data->water_status >= S2MU205_WATER_MUIC_CCIC_DET,
		msecs_to_jiffies(WATER_CCIC_WAIT_DURATION_MS));

	if ((wait_ret < 0) || (!wait_ret)) {
		pr_err("%s wait_q abnormal, status : %d\n", __func__, muic_data->water_status);
		muic_data->water_status = S2MU205_WATER_MUIC_IDLE;
		_s2mu205_muic_control_rid_adc(muic_data, true);
	} else {
		if (muic_data->water_status == S2MU205_WATER_MUIC_CCIC_DET) {
			s2mu205_muic_set_water_state(muic_data, MUIC_ENABLE);
		} else if (muic_data->water_status == S2MU205_WATER_MUIC_CCIC_INVALID) {
			pr_info("%s Not Water From CCIC.\n", __func__);
			muic_data->water_status = S2MU205_WATER_MUIC_IDLE;
			_s2mu205_muic_control_rid_adc(muic_data, true);
		}
	}
EXIT_DETECT:
	wake_unlock(&muic_data->water_wake_lock);
	mutex_unlock(&muic_data->water_det_mutex);
}

static void s2mu205_muic_water_dry_handler(struct work_struct *work)
{
	struct s2mu205_muic_data *muic_data =
		container_of(work, struct s2mu205_muic_data, water_dry_handler.work);
	int adc, i = 0;
	int wait_ret = 0;

	mutex_lock(&muic_data->water_dry_mutex);
	wake_lock(&muic_data->water_dry_wake_lock);

	if (muic_data->water_status != S2MU205_WATER_MUIC_CCIC_STABLE) {
		pr_info("%s Invalid status for Dry check\n", __func__);
		goto EXIT_DRY_STATE;
	}

	pr_info("%s Dry check start\n", __func__);
	s2mu205_muic_put_dry_chk_time(muic_data);
	s2mu205_muic_set_rid_int_mask_en(muic_data, true);

	if (muic_data->dry_cnt++ > 5) {
		muic_data->dry_duration_sec = 1800;
		pr_info("%s Dry check cnt : %d\n", __func__, muic_data->dry_cnt);
	}

	for (i = 0; i < WATER_DET_RETRY_CNT; i++) {
		adc = _s2mu205_muic_recheck_adc(muic_data);
		pr_info("%s, %d th try, adc : 0x%X\n", __func__, i, (char)adc);
		if (adc < ADC_OPEN) {
			pr_info("%s WATER IS NOT DRIED YET!!!\n", __func__);
			_s2mu205_muic_control_rid_adc(muic_data, false);
			cancel_delayed_work(&muic_data->water_dry_handler);
			schedule_delayed_work(&muic_data->water_dry_handler,
					msecs_to_jiffies(1800000));
			msleep(1000);
			goto EXIT_DRY;
		}
	}

	s2mu205_muic_set_rid_int_mask_en(muic_data, false);
	muic_data->water_dry_status = S2MU205_WATER_DRY_MUIC_DET;
	MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_CHK_WATER_DRY_REQ);
	wait_ret = wait_event_interruptible_timeout(muic_data->wait,
			muic_data->water_dry_status >= S2MU205_WATER_DRY_MUIC_CCIC_DET,
			msecs_to_jiffies(WATER_CCIC_WAIT_DURATION_MS));

	if ((wait_ret < 0) || (!wait_ret)
			|| muic_data->water_dry_status == S2MU205_WATER_DRY_MUIC_CCIC_INVALID) {
		pr_err("%s wait_q abnormal, status : %d\n",
			__func__, muic_data->water_dry_status);
		_s2mu205_muic_control_rid_adc(muic_data, false);
		muic_data->water_dry_status = S2MU205_WATER_DRY_MUIC_IDLE;
		msleep(1000);
		muic_data->water_status = S2MU205_WATER_MUIC_CCIC_STABLE;
		cancel_delayed_work(&muic_data->water_dry_handler);
		schedule_delayed_work(&muic_data->water_dry_handler,
			msecs_to_jiffies(WATER_DRY_RETRY_INTERVAL_MS));
	} else if (muic_data->water_dry_status == S2MU205_WATER_DRY_MUIC_CCIC_DET) {
		msleep(500);
		s2mu205_muic_set_water_state(muic_data, MUIC_DISABLE);
	}

EXIT_DRY:
	pr_info("%s %d Exit DRY handler!!!\n", __func__, __LINE__);
	s2mu205_muic_set_rid_int_mask_en(muic_data, false);
EXIT_DRY_STATE:
	wake_unlock(&muic_data->water_dry_wake_lock);
	mutex_unlock(&muic_data->water_dry_mutex);
}

static void s2mu205_muic_sleep_dry_checker(struct work_struct *work)
{
	struct s2mu205_muic_data *muic_data =
		container_of(work, struct s2mu205_muic_data, sleep_dry_checker.work);
	struct timeval time;
	long duration;

	pr_info("%s entered\n", __func__);
	if (muic_data->water_status == S2MU205_WATER_MUIC_CCIC_STABLE && muic_data->lcd_on) {
		if (!_s2mu205_muic_get_vbus_state(muic_data)) {
			do_gettimeofday(&time);
			duration = (long)time.tv_sec - muic_data->dry_chk_time;
			pr_info("%s dry check duration : (%ld), compare : (%ld)\n",
				__func__, duration, muic_data->dry_duration_sec);
			if ((duration > muic_data->dry_duration_sec) || (duration < 0)) {
				cancel_delayed_work(&muic_data->water_dry_handler);
				schedule_delayed_work(&muic_data->water_dry_handler, 0);
			}
		}
	}
}

static void s2mu205_muic_rescan_validity_checker(struct work_struct *work)
{
	struct s2mu205_muic_data *muic_data =
		container_of(work, struct s2mu205_muic_data, rescan_validity_checker.work);
	struct muic_platform_data *muic_pdata = muic_data->pdata;
#if defined(CONFIG_NONE_WATERPROOF_MODEL)
	struct muic_interface_t *muic_if = muic_data->if_data;
#endif
	u8 reg_val = 0;

	pr_info("%s entered\n", __func__);

	if (!_s2mu205_muic_get_vbus_state(muic_data)
#if !IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		|| !muic_data->is_cable_inserted
#endif
			) {
#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		pr_info("%s NONE_WATERPROOF_MODEL\n", __func__);
#else
		pr_info("%s vbus detached while checking.\n", __func__);
#endif
		return;
#if defined(CONFIG_NONE_WATERPROOF_MODEL)
	} else if (muic_if->is_bypass) {
		pr_info("%s is_bypass(%d)\n", __func__, muic_if->is_bypass);
		return;
#endif
	}

	if (!MUIC_IS_ATTACHED(muic_pdata->attached_dev)) {
		pr_info("%s detected dev(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));
		reg_val = s2mu205_i2c_read_byte(muic_data->i2c, S2MU205_REG_MUIC_CTRL2);
		if (reg_val & MUIC_CTRL2_ADC_OFF_MASK) {
			muic_data->invalid_rescanned = true;
		}
#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		pr_info("TIMEOUT DETECTED\n");
		muic_data->is_timeout_attached = true;
		muic_data->new_dev = ATTACHED_DEV_TIMEOUT_OPEN_MUIC;
		s2mu205_muic_handle_attached_dev(muic_data);
#endif
	}
}

static int muic_fb_notifier_event(struct notifier_block *this,
        unsigned long val, void *v)
{
	struct fb_event *evdata = v;
	struct s2mu205_muic_data *muic_data =
		container_of(this, struct s2mu205_muic_data, fb_notifier);
	int fb_blank = 0;

	if (evdata->info->node)
		return NOTIFY_DONE;

	switch (val) {
	case FB_EVENT_BLANK:
		break;
	default:
		return NOTIFY_DONE;
	}

	fb_blank = *(int *)evdata->data;

	if (fb_blank == FB_BLANK_UNBLANK) {
		pr_info("%s: lcd on\n", __func__);
		muic_data->lcd_on = true;
		pr_info("%s entered, w_state : %d\n", __func__, muic_data->water_status);
		if (muic_data->water_status == S2MU205_WATER_MUIC_CCIC_STABLE) {
			cancel_delayed_work(&muic_data->sleep_dry_checker);
			schedule_delayed_work(&muic_data->sleep_dry_checker,
				msecs_to_jiffies(WATER_WAKEUP_WAIT_DURATION_MS));
		}
	} else
		muic_data->lcd_on = false;

	return NOTIFY_DONE;
}
#endif

static void s2mu205_muic_handle_attached_dev(struct s2mu205_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC &&
				muic_data->new_dev != muic_pdata->attached_dev) {
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
		muic_manager_set_legacy_dev(muic_pdata->muic_if, muic_data->new_dev);
#endif
		muic_core_handle_attach(muic_pdata, muic_data->new_dev,
			muic_pdata->adc, muic_pdata->vbvolt);
	}
}

static void _s2mu205_muic_resend_jig_type(struct s2mu205_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	if (muic_data->vbvolt
			&& (muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_ON_MUIC)) {
		muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
		s2mu205_muic_handle_attached_dev(muic_data);
	} else if (!muic_data->vbvolt
			&& (muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_ON_VB_MUIC)) {
		muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
		s2mu205_muic_handle_attached_dev(muic_data);
	}
}
#if IS_ENABLED(CONFIG_MUIC_SUPPORT_CCIC)
static bool s2mu205_muic_is_opmode_typeC(struct s2mu205_muic_data *muic_data)
{
	struct muic_interface_t *muic_if;

	if (muic_data->if_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return false;
	}

	if (muic_data->if_data)
		muic_if = muic_data->if_data;

	if (muic_if->opmode & OPMODE_CCIC)
		return true;
	else
		return false;
}
#endif

static irqreturn_t s2mu205_muic_attach_isr(int irq, void *data)
{
	struct s2mu205_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;
	int det_ret = S2MU205_DETECT_NONE;

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}
	muic_pdata = muic_data->pdata;

	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	s2mu205_muic_get_detect_info(muic_data);
	if (s2mu205_muic_is_opmode_typeC(muic_data)) {
		if (!muic_data->is_cable_inserted
			&& muic_data->adc == ADC_GND) {
			muic_data->is_cable_inserted = true;
			wake_up_interruptible(&muic_data->cable_wait);
		}
	}

	if (MUIC_IS_ATTACHED(muic_pdata->attached_dev)) {
		pr_err("%s Cable type already was attached\n", __func__);
		goto attach_skip;
	}

	pr_info("%s start(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	pr_info("%s rescan cnt : %d\n", __func__, muic_data->rescan_cnt);
	if (s2mu205_muic_is_opmode_typeC(muic_data)) {
		if (IS_WATER_ADC(muic_data->adc)
			|| IS_WATER_STATUS(muic_data->water_status)) {
			goto attach_skip;
		} else if (muic_data->rescan_cnt == 0
				&& muic_data->adc == ADC_GND
				&& muic_data->vbvolt
				&& !muic_core_get_ccic_cable_state(muic_pdata)
				&& !IS_WATER_STATUS(muic_data->water_status)) {
			s2mu205_muic_bcd_rescan(muic_data);
			goto attach_skip;
		}
	}
#endif

	det_ret = s2mu205_muic_detect_dev_bc1p2(muic_data);
	if (det_ret == S2MU205_DETECT_DONE)
		goto attach_done;
	else if (det_ret == S2MU205_DETECT_SKIP)
		goto attach_skip;

#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID) && IS_ENABLED(CONFIG_MUIC_SUPPORT_TYPEB)
	det_ret = s2mu205_muic_detect_dev_mrid_adc(muic_data);
#endif
attach_done:
	s2mu205_muic_handle_attached_dev(muic_data);

attach_skip:
	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t s2mu205_muic_detach_isr(int irq, void *data)
{
	struct s2mu205_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_pdata = muic_data->pdata;

	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	pr_info("%s start(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	if (MUIC_IS_ATTACHED(muic_pdata->attached_dev) == false) {
		pr_err("%s Cable type already was detached\n", __func__);
		goto detach_skip;
	}

	s2mu205_muic_get_detect_info(muic_data);
#if IS_ENABLED(CONFIG_MUIC_SUPPORT_CCIC)
	if (s2mu205_muic_is_opmode_typeC(muic_data)) {
		if (!muic_core_get_ccic_cable_state(muic_data->pdata)) {
			muic_core_handle_detach(muic_data->pdata);
			s2mu205_muic_handle_legacy_detach(muic_data);
		}
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
		else if (muic_data->adc == ADC_OPEN && muic_data->is_cable_inserted) {
			muic_data->is_cable_inserted = false;
		}
#endif
	} else
		muic_core_handle_detach(muic_data->pdata);
#else
	muic_core_handle_detach(muic_data->pdata);
#endif

detach_skip:
	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t s2mu205_muic_vbus_on_isr(int irq, void *data)
{
	struct s2mu205_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_pdata = muic_data->pdata;

	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	pr_info("%s start(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	s2mu205_muic_get_detect_info(muic_data);
	if (s2mu205_muic_is_opmode_typeC(muic_data)) {
		muic_data->invalid_rescanned = false;
		cancel_delayed_work(&muic_data->rescan_validity_checker);
		schedule_delayed_work(&muic_data->rescan_validity_checker,
			msecs_to_jiffies(1200));
		if (!muic_data->is_cable_inserted
				&& muic_data->adc == ADC_GND) {
			muic_data->is_cable_inserted = true;
			wake_up_interruptible(&muic_data->cable_wait);
		}
		if (muic_data->rescan_cnt == 0
				&& muic_data->adc == ADC_GND
				&& !muic_core_get_ccic_cable_state(muic_pdata)
				&& !IS_WATER_STATUS(muic_data->water_status)) {
			msleep(150);
			s2mu205_muic_bcd_rescan(muic_data);
		}
	}
#else
	muic_pdata->vbvolt = muic_data->vbvolt = _s2mu205_muic_get_vbus_state(muic_data);
#endif

#if IS_ENABLED(CONFIG_HICCUP_CHARGER) && IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	if (!lpcharge &&
			IS_WATER_STATUS(muic_data->water_status)) {
		s2mu205_muic_set_hiccup_mode(muic_data, MUIC_ENABLE);
	}
#endif

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle(STATUS_VBUS_HIGH);
#endif /* CONFIG_VBUS_NOTIFIER */
	_s2mu205_muic_resend_jig_type(muic_data);

	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t s2mu205_muic_vbus_off_isr(int irq, void *data)
{
	struct s2mu205_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_pdata = muic_data->pdata;

	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	pr_info("%s start(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));
	muic_pdata->vbvolt = muic_data->vbvolt = _s2mu205_muic_get_vbus_state(muic_data);
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	muic_data->rescan_cnt = 0;
#endif

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle(STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
	_s2mu205_muic_resend_jig_type(muic_data);

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	if (muic_data->invalid_rescanned
			&& !MUIC_IS_ATTACHED(muic_pdata->attached_dev)) {
		_s2mu205_muic_control_rid_adc(muic_data, MUIC_ENABLE);
		muic_data->invalid_rescanned = false;
	}
#endif
#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		if (muic_data->is_timeout_attached) {
			muic_data->is_timeout_attached = false;
			muic_core_handle_detach(muic_data->pdata);
		}
#endif

	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t s2mu205_muic_usb_killer_isr(int irq, void *data)
{
	struct s2mu205_muic_data *muic_data = data;

	if (muic_data == NULL) {
		pr_err("%s, data NULL\n", __func__);
		return IRQ_NONE;
	}

	pr_info("%s\n", __func__);
	if (muic_data->killer_status == S2MU205_KILLER_WAIT_STATUS)
		muic_data->killer_status = S2MU205_KILLER_DETECTED;

	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
static irqreturn_t s2mu205_muic_rid_chg_isr(int irq, void *data)
{
	struct s2mu205_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_pdata = muic_data->pdata;

	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	muic_data->adc = muic_pdata->adc = _s2mu205_muic_get_rid_adc(muic_data);;

	pr_info("%s Vbus(%s), rid_adc(%#x), Type(%s)\n", __func__,
			(_s2mu205_muic_get_vbus_state(muic_data) ? "High" : "Low"),
			muic_data->adc, dev_to_str(muic_pdata->attached_dev));

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t s2mu205_muic_adc_change_isr(int irq, void *data)
{
	struct s2mu205_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	struct muic_interface_t *muic_if;
#endif
#endif

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_pdata = muic_data->pdata;

	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	muic_if = muic_data->if_data;

	if (muic_if == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}
#endif
#endif

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	muic_data->adc = muic_pdata->adc = _s2mu205_muic_get_rid_adc(muic_data);
	muic_data->vbvolt = _s2mu205_muic_get_vbus_state(muic_data);

	pr_info("%s Vbus(%s), rid_adc(%#x), Type(%s), opmode : %s\n", __func__,
			(muic_data->vbvolt ? "High" : "Low"),
			muic_data->adc, dev_to_str(muic_pdata->attached_dev),
			(muic_if->opmode ? "CCIC" : "MUIC"));

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
	if (muic_if->opmode & OPMODE_CCIC) {
#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		pr_info("%s, water det is skipeed, non waterproof model, adc : %d\n",
				__func__, muic_data->adc);
		if ((IS_WATER_ADC(muic_data->adc)
			|| (muic_data->adc & ADC_CONVERSION_ERR_MASK))
			&& (!muic_data->vbvolt)) {
				pr_info("%s, water detected\n",	__func__);
		}
		goto exit_adc_chg;
#else
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
		if ((IS_WATER_ADC(muic_data->adc)
				|| (muic_data->adc & ADC_CONVERSION_ERR_MASK))
				&& (!muic_data->vbvolt)
				&& (muic_data->water_status == S2MU205_WATER_MUIC_IDLE)) {
			muic_data->water_status = S2MU205_WATER_MUIC_VERIFY;
			msleep(100);
			muic_data->adc = s2mu205_muic_water_judge(muic_data);
			muic_data->vbvolt = _s2mu205_muic_get_vbus_state(muic_data);
			if (IS_WATER_ADC(muic_data->adc) && (!muic_data->vbvolt)) {
				cancel_delayed_work(&muic_data->water_detect_handler);
				schedule_delayed_work(&muic_data->water_detect_handler,
					msecs_to_jiffies(0));
				goto exit_adc_chg;
			} else {
				muic_data->water_status = S2MU205_WATER_MUIC_IDLE;
			}
		}
#endif
#endif
	} else {
		s2mu205_muic_detect_dev_rid_array(muic_data);
		s2mu205_muic_handle_attached_dev(muic_data);
	}
#endif

#if IS_ENABLED(CONFIG_MUIC_SUPPORT_TYPEB)
	if (!MUIC_IS_ATTACHED(muic_pdata->attached_dev)) {
		muic_data->new_dev = s2mu205_muic_detect_dev_rid_device(muic_data);
		if (MUIC_IS_ATTACHED(muic_data->new_dev) &&
			muic_data->new_dev != muic_pdata->attached_dev) {
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
			muic_manager_set_legacy_dev(muic_pdata->muic_if, muic_data->new_dev);
#endif
			muic_core_handle_attach(muic_pdata, muic_data->new_dev,
					muic_pdata->adc, muic_pdata->vbvolt);
		}
	} else {
		pr_info("%s, but (%s) is attached.\n",
			__func__, dev_to_str(muic_pdata->attached_dev));
	}
#endif

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
exit_adc_chg:
#endif
	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}
#endif

static irqreturn_t s2mu205_muic_reserved_isr(int irq, void *data)
{
	pr_info("%s irq_num(%d)\n", __func__, irq);

	return IRQ_HANDLED;
}

static int s2mu205_muic_irq_init(struct s2mu205_muic_data *muic_data)
{
	int ret = 0;

	if (muic_data->mfd_pdata && (muic_data->mfd_pdata->irq_base > 0)) {
		int irq_base = muic_data->mfd_pdata->irq_base;

		/* request MUIC IRQ */
		muic_data->irq_attach = irq_base + S2MU205_MUIC_IRQ1_ATTACH;
		REQUEST_IRQ(muic_data->irq_attach, muic_data,
			"muic-attach", &s2mu205_muic_attach_isr);

		muic_data->irq_detach = irq_base + S2MU205_MUIC_IRQ1_DETACH;
		REQUEST_IRQ(muic_data->irq_detach, muic_data,
			"muic-detach", &s2mu205_muic_detach_isr);

#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
		muic_data->irq_rid_chg = irq_base + S2MU205_MUIC_IRQ1_RID_CHG;
		REQUEST_IRQ(muic_data->irq_rid_chg, muic_data,
			"muic-rid_chg", &s2mu205_muic_rid_chg_isr);
#endif

		muic_data->irq_usb_killer = irq_base + S2MU205_MUIC_IRQ1_USB_Killer;
		REQUEST_IRQ(muic_data->irq_usb_killer, muic_data,
			"muic-usb_killer", &s2mu205_muic_usb_killer_isr);

		muic_data->irq_vbus_on = irq_base + S2MU205_MUIC_IRQ2_VBUS_ON;
		REQUEST_IRQ(muic_data->irq_vbus_on, muic_data,
			"muic-vbus_on", &s2mu205_muic_vbus_on_isr);

		muic_data->irq_rsvd_attach = irq_base + S2MU205_MUIC_IRQ2_RSVD_ATTACH;
		REQUEST_IRQ(muic_data->irq_rsvd_attach, muic_data,
			"muic-rsvd_attach", &s2mu205_muic_reserved_isr);

#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
		muic_data->irq_adc_change = irq_base + S2MU205_MUIC_IRQ2_ADC_CHANGE;
		REQUEST_IRQ(muic_data->irq_adc_change, muic_data,
			"muic-adc_change", &s2mu205_muic_adc_change_isr);
#endif

		muic_data->irq_av_charge = irq_base + S2MU205_MUIC_IRQ2_AV_CHARGE;
		REQUEST_IRQ(muic_data->irq_av_charge, muic_data,
			"muic-av_charge", &s2mu205_muic_reserved_isr);

		muic_data->irq_vbus_off = irq_base + S2MU205_MUIC_IRQ2_VBUS_OFF;
		REQUEST_IRQ(muic_data->irq_vbus_off, muic_data,
			"muic-vbus_off", &s2mu205_muic_vbus_off_isr);

	}

	pr_info("%s muic-attach(%d), muic-detach(%d), muic-rid_chg(%d), muic-vbus_on(%d)",
		__func__, muic_data->irq_attach, muic_data->irq_detach, muic_data->irq_rid_chg,
			muic_data->irq_vbus_on);
	pr_info("muic-rsvd_attach(%d), muic-adc_change(%d), muic-av_charge(%d), muic-vbus_off(%d)\n",
		muic_data->irq_rsvd_attach, muic_data->irq_adc_change,
		muic_data->irq_av_charge, muic_data->irq_vbus_off);

	return ret;
}

static void s2mu205_muic_free_irqs(struct s2mu205_muic_data *muic_data)
{
	pr_info("%s\n", __func__);

	/* free MUIC IRQ */
	FREE_IRQ(muic_data->irq_attach, muic_data, "muic-attach");
	FREE_IRQ(muic_data->irq_detach, muic_data, "muic-detach");
#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
	FREE_IRQ(muic_data->irq_rid_chg, muic_data, "muic-rid_chg");
#endif
	FREE_IRQ(muic_data->irq_vbus_on, muic_data, "muic-vbus_on");
	FREE_IRQ(muic_data->irq_rsvd_attach, muic_data, "muic-rsvd_attach");
#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
	FREE_IRQ(muic_data->irq_adc_change, muic_data, "muic-adc_change");
#endif
	FREE_IRQ(muic_data->irq_av_charge, muic_data, "muic-av_charge");
	FREE_IRQ(muic_data->irq_vbus_off, muic_data, "muic-vbus_off");
}

#if IS_ENABLED(CONFIG_OF)
static int of_s2mu205_muic_dt(struct device *dev,
			      struct s2mu205_muic_data *muic_data)
{
	struct device_node *np, *np_muic;
	int ret = 0;

	np = dev->parent->of_node;
	if (!np) {
		pr_err("%s : could not find np\n", __func__);
		return -ENODEV;
	}

	np_muic = of_find_node_by_name(np, "muic");
	if (!np_muic) {
		pr_err("%s : could not find muic sub-node np_muic\n", __func__);
		return -EINVAL;
	}

/* FIXME */
#if !IS_ENABLED(CONFIG_MUIC_UART_SWITCH)
	if (of_gpio_count(np_muic) < 1) {
		pr_err("%s : could not find muic gpio\n", __func__);
		muic_data->pdata->gpio_uart_sel = 0;
	} else
		muic_data->pdata->gpio_uart_sel = of_get_gpio(np_muic, 0);
#else
	muic_data->pdata->uart_addr =
	    (const char *)of_get_property(np_muic, "muic,uart_addr", NULL);
	muic_data->pdata->uart_txd =
	    (const char *)of_get_property(np_muic, "muic,uart_txd", NULL);
	muic_data->pdata->uart_rxd =
	    (const char *)of_get_property(np_muic, "muic,uart_rxd", NULL);
#endif

	return ret;
}
#endif /* CONFIG_OF */

static void s2mu205_muic_init_drvdata(struct s2mu205_muic_data *muic_data,
				      struct s2mu205_dev *s2mu205,
				      struct platform_device *pdev,
				      struct s2mu205_platform_data *mfd_pdata)
{
	/* save platfom data for gpio control functions */
	muic_data->s2mu205_dev = s2mu205;
	muic_data->dev = &pdev->dev;
	muic_data->i2c = s2mu205->muic;
	muic_data->mfd_pdata = mfd_pdata;
	muic_data->afc_check = false;
	muic_data->rescan_cnt = 0;
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	muic_data->water_status = S2MU205_WATER_MUIC_IDLE;
	muic_data->water_dry_status = S2MU205_WATER_DRY_MUIC_IDLE;
	muic_data->dry_chk_time = 0;
	muic_data->dry_cnt = 0;
	muic_data->dry_duration_sec = WATER_DRY_RETRY_INTERVAL_SEC;
	muic_data->invalid_rescanned = false;
#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
	muic_data->is_hiccup_mode = false;
#endif
	muic_data->killer_status = S2MU205_KILLER_NONE;
	muic_data->is_cable_inserted = false;
#endif
#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
	muic_data->is_timeout_attached = false;
#endif

}

static void s2mu205_muic_init_interface(struct s2mu205_muic_data *muic_data,
					struct muic_interface_t *muic_if)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s, muic_if : 0x%p, muic_data : 0x%p\n",
        __func__, muic_if, muic_data);

	muic_if->muic_data = (void *)muic_data;

	muic_if->set_com_to_open = s2mu205_if_com_to_open;
	muic_if->set_switch_to_usb = s2mu205_if_com_to_usb;
	muic_if->set_com_to_otg = s2mu205_if_com_to_usb;
	muic_if->set_switch_to_uart = s2mu205_if_com_to_uart;
	muic_if->get_vbus = s2mu205_if_get_vbus;
	muic_if->set_gpio_uart_sel = s2mu205_if_set_gpio_uart_sel;
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
	muic_if->set_cable_state = s2mu205_if_set_cable_state;
	muic_if->set_dcd_rescan = s2mu205_if_cable_recheck;
	muic_if->check_usb_killer = s2mu205_if_check_usb_killer;
#endif
	muic_if->bcd_rescan = s2mu205_if_set_bcd_rescan_reg;
	muic_if->control_rid_adc = s2mu205_if_control_rid_adc;
#if IS_ENABLED(CONFIG_MUIC_S2MU205_RID)
	muic_if->get_adc = s2mu205_if_get_adc;
#endif
	muic_if->set_jig_ctrl_on = s2mu205_if_set_jig_ctrl_on;
#if IS_ENABLED(CONFIG_MUIC_SYSFS)
	muic_if->show_register = s2mu205_if_show_register;
#endif
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	muic_if->set_water_detect = s2mu205_if_set_water_det;
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
	muic_if->set_water_detect_from_boot = s2mu205_if_set_water_det_from_boot;
#endif
#endif
#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
	muic_if->set_hiccup_mode = s2mu205_if_set_hiccup_mode;
	muic_if->get_hiccup_mode = s2mu205_if_get_hiccup_mode;
#endif
#if defined(CONFIG_MUIC_SUPPORT_PRSWAP)
	muic_if->prswap_work = s2mu205_if_prswap_work;
#endif
	muic_if->set_bypass = s2mu205_if_set_bypass;
	muic_data->if_data = muic_if;
	muic_pdata->muic_if = muic_if;
}

static int s2mu205_muic_probe(struct platform_device *pdev)
{
	struct s2mu205_dev *s2mu205 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu205_platform_data *mfd_pdata = dev_get_platdata(s2mu205->dev);
	struct s2mu205_muic_data *muic_data;
	struct muic_platform_data *muic_pdata;
	struct muic_interface_t *muic_if;
	int ret = 0;

	pr_info("%s start\n", __func__);
	muic_data = devm_kzalloc(&pdev->dev, sizeof(*muic_data), GFP_KERNEL);
	if (unlikely(!muic_data)) {
		pr_err("%s out of memory\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}

	if (unlikely(!mfd_pdata)) {
		pr_err("%s failed to get s2mu205 mfd platform data\n",
		       __func__);
		ret = -ENOMEM;
		goto err_kfree1;
	}

	muic_pdata = muic_core_init(muic_data);
	if (unlikely(!muic_pdata))
		goto err_kfree1;

	muic_data->pdata = muic_pdata;

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
	muic_if = muic_manager_init(muic_pdata, muic_data);
#else
	muic_if = kzalloc(sizeof(*muic_if), GFP_KERNEL);
#endif

	if (!muic_if) {
		pr_err("%s failed to init muic manager, ret : 0x%X\n",
		       __func__, ret);
		goto err_init_if;
	}

	s2mu205_muic_init_interface(muic_data, muic_if);
	s2mu205_muic_init_drvdata(muic_data, s2mu205, pdev, mfd_pdata);
#if IS_ENABLED(CONFIG_OF)
	ret = of_s2mu205_muic_dt(&pdev->dev, muic_data);
	if (ret < 0)
		pr_err("no muic dt! ret[%d]\n", ret);
#endif /* CONFIG_OF */

	mutex_init(&muic_data->muic_mutex);
	mutex_init(&muic_data->switch_mutex);
	mutex_init(&muic_data->bcd_rescan_mutex);
	wake_lock_init(&muic_data->wake_lock, WAKE_LOCK_SUSPEND, "muic_wake");
	platform_set_drvdata(pdev, muic_data);

	if (muic_data->pdata->init_gpio_cb)
		ret = muic_data->pdata->init_gpio_cb(muic_data->pdata, get_switch_sel());
	if (ret) {
		pr_err("%s failed to init gpio(%d)\n", __func__, ret);
		goto fail_init_gpio;
	}

	pr_info("%s: usb_path(%d), uart_path(%d)\n", __func__,
		muic_pdata->usb_path, muic_pdata->uart_path);

#if IS_ENABLED(CONFIG_MUIC_SYSFS)
	ret = muic_sysfs_init(muic_pdata);
	if (ret) {
		pr_err("failed to create sysfs\n");
		goto fail_init_sysfs;
	}
#endif

	ret = s2mu205_muic_reg_init(muic_data);
	if (ret) {
		pr_err("failed to init muic(%d)\n", ret);
		goto fail;
	}

	if (muic_pdata->is_rustproof) {
		pr_err("%s rustproof is enabled\n", __func__);
		ret = _s2mu205_muic_sel_path(muic_data, S2MU205_PATH_OPEN);
	}

	pr_info("%s muic_if->opmode(%d)\n", __func__, muic_if->opmode);

	INIT_DELAYED_WORK(&muic_data->dcd_recheck, s2mu205_muic_dcd_recheck);

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	init_waitqueue_head(&muic_data->wait);
	init_waitqueue_head(&muic_data->cable_wait);

	INIT_DELAYED_WORK(&muic_data->water_dry_handler,
		s2mu205_muic_water_dry_handler);
	INIT_DELAYED_WORK(&muic_data->water_detect_handler,
		s2mu205_muic_water_detect_handler);
	INIT_DELAYED_WORK(&muic_data->sleep_dry_checker,
		s2mu205_muic_sleep_dry_checker);
	INIT_DELAYED_WORK(&muic_data->rescan_validity_checker,
		s2mu205_muic_rescan_validity_checker);

	mutex_init(&muic_data->water_det_mutex);
	mutex_init(&muic_data->water_dry_mutex);
	wake_lock_init(&muic_data->water_wake_lock,
		WAKE_LOCK_SUSPEND, "muic_water_wake");
	wake_lock_init(&muic_data->water_dry_wake_lock,
		WAKE_LOCK_SUSPEND, "muic_water_dry_wake");
	muic_data->fb_notifier.priority = -1;
	muic_data->fb_notifier.notifier_call = muic_fb_notifier_event;
	fb_register_client(&muic_data->fb_notifier);
#endif

	ret = muic_manager_psy_init(muic_if, &pdev->dev);
	if (ret) {
		pr_err("%s failed to init psy(%d)\n", __func__, ret);
	}

	ret = s2mu205_muic_irq_init(muic_data);
	if (ret) {
		pr_err("%s failed to init irq(%d)\n", __func__, ret);
		goto fail_init_irq;
	}

	if (muic_if->opmode == OPMODE_MUIC) {
		s2mu205_muic_adc_change_isr(-1, muic_data);
	} else {
		s2mu205_muic_get_detect_info(muic_data);
#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		cancel_delayed_work(&muic_data->rescan_validity_checker);
		schedule_delayed_work(&muic_data->rescan_validity_checker,
			msecs_to_jiffies(1200));
#endif
#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
		/* Initial Check */
		if (muic_data->rescan_cnt == 0
				&& muic_data->vbvolt
				&& muic_data->adc == ADC_GND
				&& !muic_core_get_ccic_cable_state(muic_pdata)
				&& !IS_WATER_STATUS(muic_data->water_status)) {
			/* In case of normal charger cable */
			s2mu205_muic_bcd_rescan(muic_data);
		} else if (!muic_data->is_cable_inserted
				&& muic_data->adc == ADC_GND) {
			/* In case of OTG */
			muic_data->is_cable_inserted = true;
			wake_up_interruptible(&muic_data->cable_wait);
		} else if (muic_data->adc != ADC_GND
				&& !_s2mu205_muic_get_vbus_state(muic_data)) {
			/* In case of None Cable
			 * Need to check the water.
			 */
			muic_data->adc = _s2mu205_muic_recheck_adc(muic_data);
	}
#else
		s2mu205_muic_attach_isr(-1, muic_data);
#endif
	}
	return 0;

fail_init_irq:
fail:
#if IS_ENABLED(CONFIG_MUIC_SYSFS)
	muic_sysfs_deinit(muic_pdata);
fail_init_sysfs:
#endif
fail_init_gpio:
	mutex_destroy(&muic_data->muic_mutex);
err_init_if:
err_kfree1:
err_return:
	return ret;
}

/* if need to set s2mu205 pdata */
static const struct of_device_id s2mu205_muic_match_table[] = {
	{.compatible = "samsung,s2mu205-muic",},
	{},
};

static int s2mu205_muic_remove(struct platform_device *pdev)
{
	struct s2mu205_muic_data *muic_data = platform_get_drvdata(pdev);

	if (muic_data) {
		pr_info("%s\n", __func__);

#if IS_ENABLED(CONFIG_MUIC_SYSFS)
		if (muic_data->pdata != NULL)
			muic_sysfs_deinit(muic_data->pdata);
#endif

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
		if (muic_data->if_data != NULL)
			muic_manager_exit(muic_data->if_data);
#else
		kfree(muic_data->if_data);
#endif
		muic_core_exit(muic_data->pdata);

		disable_irq_wake(muic_data->i2c->irq);
		s2mu205_muic_free_irqs(muic_data);
		mutex_destroy(&muic_data->muic_mutex);
		mutex_destroy(&muic_data->switch_mutex);
		i2c_set_clientdata(muic_data->i2c, NULL);
	}

	return 0;
}

static void s2mu205_muic_shutdown(struct platform_device *pdev)
{
	struct s2mu205_muic_data *muic_data = platform_get_drvdata(pdev);
	int ret;

	pr_info("%s\n", __func__);

	if (!muic_data->i2c) {
		pr_err("%s no muic i2c client\n", __func__);
		return;
	}
	_s2mu205_muic_set_chg_det(muic_data, true);

	ret = _s2mu205_muic_sel_path(muic_data, S2MU205_PATH_OPEN);
	if (ret < 0)
		pr_err("fail to open mansw\n");

	/* set auto sw mode before shutdown to make sure device goes into */
	/* LPM charging when TA or USB is connected during off state */
	ret = _s2mu205_muic_set_path_mode(muic_data, S2MU205_MODE_AUTO);
	if (ret < 0) {
		pr_err("%s fail to update reg\n", __func__);
		return;
	}
}

#if IS_ENABLED(CONFIG_PM)
static int s2mu205_muic_suspend(struct device *dev)
{
	struct s2mu205_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	muic_pdata->suspended = true;

#if IS_ENABLED(CONFIG_S2MU205_TYPEC_WATER)
	if (muic_data->water_status == S2MU205_WATER_MUIC_CCIC_STABLE) {
		cancel_delayed_work(&muic_data->sleep_dry_checker);
	}
#endif

	return 0;
}

static int s2mu205_muic_resume(struct device *dev)
{
	struct s2mu205_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	muic_pdata->suspended = false;

	if (muic_pdata->need_to_noti) {
		if (muic_pdata->attached_dev) {
			MUIC_SEND_NOTI_ATTACH(muic_pdata->attached_dev);
		} else {
			MUIC_SEND_NOTI_DETACH(muic_pdata->attached_dev);
		}

		muic_pdata->need_to_noti = false;
	}

	return 0;
}
#else
#define s2mu205_muic_suspend NULL
#define s2mu205_muic_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(s2mu205_muic_pm_ops, s2mu205_muic_suspend,
		s2mu205_muic_resume);

static struct platform_driver s2mu205_muic_driver = {
	.probe = s2mu205_muic_probe,
	.remove = s2mu205_muic_remove,
	.shutdown = s2mu205_muic_shutdown,
	.driver = {
		   .name = "s2mu205-muic",
		   .owner = THIS_MODULE,
		   .of_match_table = s2mu205_muic_match_table,
#if IS_ENABLED(CONFIG_PM)
		   .pm = &s2mu205_muic_pm_ops,
#endif
		   },
};

static int __init s2mu205_muic_init(void)
{
	return platform_driver_register(&s2mu205_muic_driver);
}
module_init(s2mu205_muic_init);

static void __exit s2mu205_muic_exit(void)
{
	platform_driver_unregister(&s2mu205_muic_driver);
}
module_exit(s2mu205_muic_exit);

MODULE_DESCRIPTION("Samsung S2MU205 Micro USB IC driver");
MODULE_LICENSE("GPL");
