/*
 * driver/muic/s2mu004.c - S2MU004 micro USB switch device driver
 *
 * Copyright (C) 2015 Samsung Electronics
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
 * This driver is based on max77843-muic-afc.c
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
#include <linux/mfd/samsung/s2mu004.h>
#include <linux/mfd/samsung/s2mu004-private.h>
#include <linux/muic/s2mu004-muic.h>
#include <linux/muic/s2mu004-muic-sysfs.h>

#include <linux/battery/sec_charging_common.h>
#include <linux/power_supply.h>
#include <linux/ccic/usbpd-s2mu004.h>
#include <linux/muic/s2mu004-muic-hv.h>

#include <linux/sec_debug.h>
#include <linux/sec_ext.h>
#include <linux/muic/muic_notifier.h>
#include <linux/ccic/ccic_notifier.h>
#include <linux/usb_notify.h>
#include <linux/muic/muic_interface.h>
#include <linux/sec_batt.h>
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
#include <linux/vbus_notifier.h>
#endif
#if IS_ENABLED(CONFIG_MUIC_UART_SWITCH)
#include <mach/pinctrl-samsung.h>
#endif
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
#include <soc/samsung/exynos-modem-ctrl.h>
#endif

#if IS_ENABLED(CONFIG_MUIC_SUPPORT_CCIC)
#include <linux/fb.h>
#endif

#define ENUM_STR(x) {case(x): return #x; }
static const char *mode_to_str(enum s2mu004_muic_mode n)
{
	switch (n) {
	ENUM_STR(S2MU004_NONE_CABLE);
	ENUM_STR(S2MU004_FIRST_ATTACH);
	ENUM_STR(S2MU004_SECOND_ATTACH);
	ENUM_STR(S2MU004_MUIC_DETACH);
	ENUM_STR(S2MU004_MUIC_OTG);
	ENUM_STR(S2MU004_MUIC_JIG);
	default:
		return "invalid";
	}
	return "invalid";
}

static const char *dev_to_str(muic_attached_dev_t n)
{
	switch (n) {
	ENUM_STR(ATTACHED_DEV_NONE_MUIC);
	ENUM_STR(ATTACHED_DEV_USB_MUIC);
	ENUM_STR(ATTACHED_DEV_CDP_MUIC);
	ENUM_STR(ATTACHED_DEV_OTG_MUIC);
	ENUM_STR(ATTACHED_DEV_TA_MUIC);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_MUIC);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_TA_MUIC);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_MUIC);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_TA_MUIC);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_ANY_MUIC);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_USB_MUIC);
	ENUM_STR(ATTACHED_DEV_UNOFFICIAL_ID_CDP_MUIC);
	ENUM_STR(ATTACHED_DEV_UNDEFINED_CHARGING_MUIC);
	ENUM_STR(ATTACHED_DEV_DESKDOCK_MUIC);
	ENUM_STR(ATTACHED_DEV_UNKNOWN_VB_MUIC);
	ENUM_STR(ATTACHED_DEV_DESKDOCK_VB_MUIC);
	ENUM_STR(ATTACHED_DEV_CARDOCK_MUIC);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_MUIC);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_MUIC);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC);
	ENUM_STR(ATTACHED_DEV_JIG_UART_OFF_VB_FG_MUIC);
	ENUM_STR(ATTACHED_DEV_JIG_UART_ON_MUIC);
	ENUM_STR(ATTACHED_DEV_JIG_UART_ON_VB_MUIC);
	ENUM_STR(ATTACHED_DEV_JIG_USB_OFF_MUIC);
	ENUM_STR(ATTACHED_DEV_JIG_USB_ON_MUIC);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_MUIC);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_VB_MUIC);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_TA_MUIC);
	ENUM_STR(ATTACHED_DEV_SMARTDOCK_USB_MUIC);
	ENUM_STR(ATTACHED_DEV_UNIVERSAL_MMDOCK_MUIC);
	ENUM_STR(ATTACHED_DEV_AUDIODOCK_MUIC);
	ENUM_STR(ATTACHED_DEV_MHL_MUIC);
	ENUM_STR(ATTACHED_DEV_CHARGING_CABLE_MUIC);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_PREPARE_MUIC);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_PREPARE_DUPLI_MUIC);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_5V_MUIC);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_5V_DUPLI_MUIC);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_9V_MUIC);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_ERR_V_MUIC);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_ERR_V_DUPLI_MUIC);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_PREPARE_MUIC);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_5V_MUIC);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_ERR_V_MUIC);
	ENUM_STR(ATTACHED_DEV_QC_CHARGER_9V_MUIC);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_UNDEFINED_MUIC);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_UNSUPPORTED_MUIC);
	ENUM_STR(ATTACHED_DEV_HV_ID_ERR_SUPPORTED_MUIC);
	ENUM_STR(ATTACHED_DEV_HMT_MUIC);
	ENUM_STR(ATTACHED_DEV_VZW_ACC_MUIC);
	ENUM_STR(ATTACHED_DEV_VZW_INCOMPATIBLE_MUIC);
	ENUM_STR(ATTACHED_DEV_USB_LANHUB_MUIC);
	ENUM_STR(ATTACHED_DEV_TYPE2_CHG_MUIC);
	ENUM_STR(ATTACHED_DEV_TYPE3_MUIC);
	ENUM_STR(ATTACHED_DEV_TYPE3_MUIC_TA);
	ENUM_STR(ATTACHED_DEV_TYPE3_ADAPTER_MUIC);
	ENUM_STR(ATTACHED_DEV_TYPE3_CHARGER_MUIC);
	ENUM_STR(ATTACHED_DEV_NONE_TYPE3_MUIC);
	ENUM_STR(ATTACHED_DEV_UNSUPPORTED_ID_MUIC);
	ENUM_STR(ATTACHED_DEV_UNSUPPORTED_ID_VB_MUIC);
	ENUM_STR(ATTACHED_DEV_TIMEOUT_OPEN_MUIC);
	ENUM_STR(ATTACHED_DEV_WIRELESS_PAD_MUIC);
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	ENUM_STR(ATTACHED_DEV_CARKIT_MUIC);
#endif
	ENUM_STR(ATTACHED_DEV_POWERPACK_MUIC);
	ENUM_STR(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
	ENUM_STR(ATTACHED_DEV_HICCUP_MUIC);
	ENUM_STR(ATTACHED_DEV_CHK_WATER_REQ);
	ENUM_STR(ATTACHED_DEV_CHK_WATER_DRY_REQ);
	ENUM_STR(ATTACHED_DEV_GAMEPAD_MUIC);
	ENUM_STR(ATTACHED_DEV_CHECK_OCP);
	ENUM_STR(ATTACHED_DEV_RDU_TA_MUIC);
	ENUM_STR(ATTACHED_DEV_AFC_CHARGER_9V_DUPLI_MUIC);
	ENUM_STR(ATTACHED_DEV_UNKNOWN_MUIC);
	ENUM_STR(ATTACHED_DEV_NUM);
	default:
		return "invalid";
	}
	return "invalid";
}

static struct s2mu004_muic_data *static_data;
#if defined(CONFIG_CCIC_S2MU004)
static void s2mu004_muic_set_water_adc_ldo_wa(struct s2mu004_muic_data *muic_data, bool en);
#if !IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
static int s2mu004_muic_water_judge(struct s2mu004_muic_data *muic_data);
#endif
#endif
static int s2mu004_i2c_update_bit(struct i2c_client *client,
	u8 reg, u8 mask, u8 shift, u8 value);
static int s2mu004_muic_set_com_sw(struct s2mu004_muic_data *muic_data,
	u8 reg_val);
static void s2mu004_muic_set_dn_ready_for_killer(struct s2mu004_muic_data *muic_data);
#if defined(CONFIG_HV_MUIC_S2MU004_AFC)
int s2mu004_muic_check_afc_ready(struct s2mu004_muic_data *muic_data);
#endif

#if IS_ENABLED(DEBUG_MUIC)
#define MAX_LOG 25
#define READ 0
#define WRITE 1

static u8 s2mu004_log_cnt;
static u8 s2mu004_log[MAX_LOG][3];

static void s2mu004_reg_log(u8 reg, u8 value, u8 rw)
{
	s2mu004_log[s2mu004_log_cnt][0] = reg;
	s2mu004_log[s2mu004_log_cnt][1] = value;
	s2mu004_log[s2mu004_log_cnt][2] = rw;
	s2mu004_log_cnt++;
	if (s2mu004_log_cnt >= MAX_LOG)
		s2mu004_log_cnt = 0;
}

static void s2mu004_print_reg_log(void)
{
	int i;
	u8 reg, value, rw;
	char mesg[256] = "";

	for (i = 0; i < MAX_LOG; i++) {
		reg = s2mu004_log[s2mu004_log_cnt][0];
		value = s2mu004_log[s2mu004_log_cnt][1];
		rw = s2mu004_log[s2mu004_log_cnt][2];
		s2mu004_log_cnt++;

		if (s2mu004_log_cnt >= MAX_LOG)
			s2mu004_log_cnt = 0;
		sprintf(mesg+strlen(mesg), "%x(%x)%x ", reg, value, rw);
	}
	pr_info("%s %s\n", __func__, mesg);
}

void s2mu004_read_reg_dump(struct s2mu004_muic_data *muic, char *mesg)
{
	u8 val;

	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_CTRL1, &val);
	sprintf(mesg+strlen(mesg), "CTRL1:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_SW_CTRL, &val);
	sprintf(mesg+strlen(mesg), "SW_CTRL:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_INT1_MASK, &val);
	sprintf(mesg+strlen(mesg), "IM1:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_INT2_MASK, &val);
	sprintf(mesg+strlen(mesg), "IM2:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_CHG_TYPE, &val);
	sprintf(mesg+strlen(mesg), "CHG_T:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_DEVICE_APPLE, &val);
	sprintf(mesg+strlen(mesg), "APPLE_DT:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_ADC, &val);
	sprintf(mesg+strlen(mesg), "ADC:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_DEVICE_TYPE1, &val);
	sprintf(mesg+strlen(mesg), "DT1:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_DEVICE_TYPE2, &val);
	sprintf(mesg+strlen(mesg), "DT2:%x ", val);
	s2mu004_read_reg(muic->i2c, S2MU004_REG_MUIC_DEVICE_TYPE3, &val);
	sprintf(mesg+strlen(mesg), "DT3:%x ", val);
}

void s2mu004_print_reg_dump(struct s2mu004_muic_data *muic_data)
{
	char mesg[256] = "";

	s2mu004_read_reg_dump(muic_data, mesg);

	pr_info("%s %s\n", __func__, mesg);
}
#endif

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
/* interface functions */
static int s2mu004_if_com_to_open_with_vbus(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_open_with_vbus(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_com_to_open(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_open(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_com_to_audio(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_audio(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_com_to_otg(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_otg(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_get_adc(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_recheck_adc(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret & 0x1F;
}

static int s2mu004_if_switch_to_usb(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	ret = s2mu004_muic_com_to_usb(muic_data);
	if (ret)
		pr_err("%s set_com_usb err\n", __func__);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_switch_to_uart(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s\n", __func__);
	ret = s2mu004_muic_com_to_uart(muic_data);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu004_if_get_vbus(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_get_vbus_state(muic_data);
}

static void s2mu004_if_set_jig_state(void *mdata, bool val)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	muic_data->jig_state = val;
	pr_info("%s jig_state : (%d)\n", __func__, muic_data->jig_state);
	mutex_unlock(&muic_data->muic_mutex);
}
#endif

#if defined(CONFIG_CCIC_S2MU004)
static void s2mu004_if_set_water_det(void *mdata, bool val)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	pr_info("%s water_status : (%d)\n",
		__func__, muic_data->water_status);
	if (muic_data->water_dry_status == S2MU004_WATER_DRY_MUIC_DET) {
		if (val)
			muic_data->water_dry_status = S2MU004_WATER_DRY_MUIC_CCIC_INVALID;
		else
			muic_data->water_dry_status = S2MU004_WATER_DRY_MUIC_CCIC_DET;
		wake_up_interruptible(&muic_data->wait);
	} else {
		if (muic_data->water_status >= S2MU004_WATER_MUIC_DET) {
			if (val)
				muic_data->water_status = S2MU004_WATER_MUIC_CCIC_DET;
			else
				muic_data->water_status = S2MU004_WATER_MUIC_CCIC_INVALID;
			pr_info("%s water_status : (%d)\n",
				__func__, muic_data->water_status);
			wake_up_interruptible(&muic_data->wait);
		} else {
			pr_err("%s wrong status\n", __func__);
		}
	}
	mutex_unlock(&muic_data->muic_mutex);
}

#ifndef CONFIG_SEC_FACTORY
static void s2mu004_if_set_water_det_from_boot(void *mdata, bool val)
{
	u8 val_vbadc;
	int ret;
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	if (val) {
		muic_data->water_status = S2MU004_WATER_MUIC_CCIC_DET;

		s2mu004_i2c_update_bit(muic_data->i2c,
				S2MU004_REG_LDOADC_VSETH,
				LDOADC_VSETH_WAKE_HYS_MASK,
				LDOADC_VSETH_WAKE_HYS_SHIFT, 0x1);
		s2mu004_muic_set_water_adc_ldo_wa(muic_data, true);
		muic_data->water_status = S2MU004_WATER_MUIC_CCIC_STABLE;
		muic_data->water_dry_status = S2MU004_WATER_DRY_MUIC_IDLE;

		/* readback vbadc */
		ret = s2mu004_read_reg(muic_data->i2c, S2MU004_REG_AFC_STATUS, &val_vbadc);
		if (ret)
			pr_err("%s fail to read muic reg(%d)\n", __func__, ret);

		pr_info("%s, AFC_STATUS : 0x%x\n", __func__, val_vbadc);

#if defined(CONFIG_HV_MUIC_S2MU004_AFC)
		/* reset afc */
		s2mu004_muic_hv_update_reg(muic_data->i2c, S2MU004_REG_AFC_LOGIC_CTRL2, 0x0, 0x04, 0);
		s2mu004_muic_hv_update_reg(muic_data->i2c, S2MU004_REG_AFC_CTRL1, 0x0a, 0x1e, 0);
#endif

		MUIC_SEND_NOTI_ATTACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
	}
}
#endif
#endif

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
static void s2mu004_if_set_cable_state(void *mdata, muic_attached_dev_t new_dev)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	struct muic_platform_data *pdata = muic_data->pdata;

	mutex_lock(&muic_data->muic_mutex);

	pr_info("%s new dev=%s\n", __func__, dev_to_str(new_dev));

	switch (new_dev) {
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
		pdata->attached_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
		muic_data->jig_state = true;
		break;
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		pdata->attached_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		muic_data->jig_state = true;
		break;
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
		pdata->attached_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		muic_data->jig_state = true;
		break;
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
		pdata->attached_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		muic_data->jig_state = true;
		break;
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
		pdata->attached_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
		muic_data->jig_state = true;
		break;
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:
		pdata->attached_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
		muic_data->jig_state = true;
		break;
	case ATTACHED_DEV_OTG_MUIC:
		pdata->attached_dev = ATTACHED_DEV_OTG_MUIC;
		muic_data->attach_mode = S2MU004_MUIC_OTG;
		/* enable vbus det for interrupt */
		s2mu004_muic_control_vbus_det(muic_data, true);
		muic_data->jig_state = false;
		pr_info("USB_OTG DETECTED\n");
		break;
	case ATTACHED_DEV_USB_MUIC:
		pdata->attached_dev = ATTACHED_DEV_USB_MUIC;
		muic_data->attach_mode = S2MU004_SECOND_ATTACH;
		muic_data->jig_state = false;
		break;
	case ATTACHED_DEV_NONE_MUIC:
		if (pdata->attached_dev == ATTACHED_DEV_OTG_MUIC) {
			/* disable vbus det for interrupt */
			s2mu004_muic_control_vbus_det(muic_data, false);
		} else if (muic_data->jig_state)
			muic_data->jig_state = false;

		if (muic_core_get_ccic_cable_state(muic_data->pdata)) {
			pdata->attached_dev = ATTACHED_DEV_NONE_MUIC;
			muic_core_handle_detach(muic_data->pdata);
		}

		pdata->attached_dev = ATTACHED_DEV_NONE_MUIC;
		muic_data->attach_mode = S2MU004_NONE_CABLE;
		break;
	default:
		break;
	}

	if (muic_data->jig_state == true) {
		/* enable rid detect for waterproof */
		s2mu004_muic_control_rid_adc(muic_data, S2MU004_ENABLE);
	}

	pm_relax(muic_data->dev);
	mutex_unlock(&muic_data->muic_mutex);
}

static void s2mu004_if_set_otg_detect_en(void *mdata, bool en)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	s2mu004_muic_control_vbus_det(muic_data, en);
}

static void s2mu004_if_dcd_rescan(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	muic_data->is_dcd_recheck = true;
	s2mu004_muic_dcd_rescan(muic_data);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);
}

static int s2mu004_if_bcd_rescan(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
				(struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_bcd_rescan(muic_data);
}

static int s2mu004_if_control_rid_adc(void *mdata, bool enable)
{
	struct s2mu004_muic_data *muic_data =
				(struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_control_rid_adc(muic_data, enable);
}
#endif

#if defined(CONFIG_HV_MUIC_S2MU004_AFC)
static int s2mu004_if_set_afc_reset(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	struct muic_interface_t *muic_if = muic_data->if_data;

	pr_info("%s\n", __func__);
	muic_if->is_afc_reset = true;

	return 0;
}

static muic_attached_dev_t s2mu004_if_check_id_err(void *mdata,
		muic_attached_dev_t new_dev)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	return hv_muic_check_id_err(muic_data, new_dev);
}

static int s2mu004_if_reset_hvcontrol_reg(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	s2mu004_hv_muic_reset_hvcontrol_reg(muic_data);

	return 0;
}

static int s2mu004_if_check_afc_ready(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_check_afc_ready(muic_data);
}

static int s2mu004_if_reset_afc_register(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_reset_afc_register(muic_data);
}

static void s2mu004_if_set_afc_ready(void *mdata, bool en)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	pr_info("%s Enter en : %d\n", __func__, (int)en);
	if (en)
		s2mu004_muic_check_afc_ready(muic_data);
}
#endif

int s2mu004_if_check_usb_killer(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val = 0, muic_sw_ctrl = 0;
	u8 afc_int_mask, afc_otp4, afc_ctrl1, afc_ctrl2, is_dnres, is_vdnmon;
	int ret = MUIC_NORMAL_OTG;

	pr_info("%s, enter", __func__);

	/* muic path open */
	mutex_lock(&muic_data->switch_mutex);
	muic_sw_ctrl = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_SW_CTRL);
	s2mu004_muic_set_com_sw(muic_data, MANSW_OPEN);

	/* AFC Block Enable & INT Masking */
	afc_int_mask = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_INT_MASK);
	reg_val = afc_int_mask | INTm_VDNMon_MASK;
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_INT_MASK, reg_val);

	afc_ctrl1 = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_CTRL1);
	reg_val = afc_ctrl1 | MUIC_AFC_CTRL1_AFC_EN_MASK | MUIC_AFC_CTRL1_DPDNVD_EN_MASK;
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL1, reg_val);

	s2mu004_muic_set_dn_ready_for_killer(muic_data);

	/* 1st check */
	reg_val = (MUIC_AFC_CTRL1_VD_SEL_0_6V << MUIC_AFC_CTRL1_DPVD_SEL_SHIFT) |
			MUIC_AFC_CTRL1_AFC_EN_MASK | MUIC_AFC_CTRL1_DPDNVD_EN_MASK;
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL1, reg_val);

	mdelay(50);

	is_vdnmon = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_STATUS)
			& MUIC_AFC_STATUS_VDNMON_MASK;
	if (!is_vdnmon) {
		pr_info("%s, 1st chk: Normal OTG.", __func__);
		goto exit_chk;
	}

	/* 2nd check */
	afc_otp4 = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_OTP4);
	reg_val = afc_otp4 | MUIC_AFC_OTP4_CTRL_IDM_ON_REG_SELL_MASK;
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_OTP4, reg_val);

	afc_ctrl2 = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_CTRL2);
	reg_val = afc_ctrl2 | MUIC_AFC_CTRL2_DNRESEN_MASK;
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL2, reg_val);

	reg_val = reg_val | MUIC_AFC_CTRL1_CTRL_IDM_ON_MASK |
		MUIC_AFC_CTRL1_AFC_EN_MASK | MUIC_AFC_CTRL1_DPDNVD_EN_MASK;
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL1, reg_val);

	msleep(50);

	reg_val = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_STATUS);
	is_dnres = reg_val & MUIC_AFC_STATUS_DNRES_MASK;

	if (!is_dnres) {
		pr_info("%s, USB Killer is detected.", __func__);
		ret = MUIC_ABNORMAL_OTG;
	}

	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_OTP4, afc_otp4);
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL2, afc_ctrl2);
exit_chk:
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_INT_MASK, afc_int_mask);
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL1, afc_ctrl1);

	/* restore muic path */
	s2mu004_muic_set_com_sw(muic_data, muic_sw_ctrl);
	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
static int s2mu004_if_jig_on(void *mdata)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	return s2mu004_muic_jig_on(muic_data);
}

static int s2mu004_if_set_gpio_uart_sel(void *mdata, int uart_path)
{
	struct s2mu004_muic_data *muic_data =
		(struct s2mu004_muic_data *)mdata;

	return s2mu004_set_gpio_uart_sel(muic_data, uart_path);
}
#endif

int s2mu004_i2c_read_byte(struct i2c_client *client, u8 command)
{
	int ret = 0;
	int retry = 0;
	u8 data = 0;

	ret = s2mu004_read_reg(client, command, &data);

	while (ret < 0) {
		pr_info("failed to read reg(0x%x) retry(%d)\n", command, retry);
		if (retry > 10) {
			pr_err("%s  retry failed!!\n", __func__);
			break;
		}
		msleep(100);
		ret = s2mu004_read_reg(client, command, &data);
		retry++;
	}

#if IS_ENABLED(DEBUG_MUIC)
	s2mu004_reg_log(command, ret, retry << 1 | READ);
#endif
	return data;
}

int s2mu004_i2c_write_byte(struct i2c_client *client,
			u8 command, u8 value)
{
	int ret_r = 0;
	int ret_w = 0;
	int retry = 0;
	u8 written = 0;

	ret_w = s2mu004_write_reg(client, command, value);

	while (ret_w < 0) {
		pr_info("failed to write reg(0x%x) retry(%d)\n", command, retry);
		ret_r = s2mu004_read_reg(client, command, &written);
		if (ret_r < 0)
			pr_err("%s reg(0x%x)\n", __func__, command);
		msleep(100);
		ret_w = s2mu004_write_reg(client, command, value);
		retry++;
	}
#if IS_ENABLED(DEBUG_MUIC)
	s2mu004_reg_log(command, value, retry << 1 | WRITE);
#endif
	return ret_w;
}
static int s2mu004_i2c_guaranteed_wbyte(struct i2c_client *client,
			u8 command, u8 value)
{
	int ret;
	int retry = 0;
	int written;

	ret = s2mu004_i2c_write_byte(client, command, value);
	written = s2mu004_i2c_read_byte(client, command);
	while (written != value) {
		pr_info("reg(0x%x): written(0x%x) != value(0x%x)\n",
			command, written, value);
		if (retry > 10) {
			pr_err("%s  retry failed!!\n", __func__);
			break;
		}
		msleep(100);
		retry++;
		ret = s2mu004_i2c_write_byte(client, command, value);
		written = s2mu004_i2c_read_byte(client, command);
	}
	return ret;
}

static int s2mu004_i2c_update_bit(struct i2c_client *i2c,
			u8 reg, u8 mask, u8 shift, u8 value)
{
	int ret;
	u8 reg_val = 0;

	reg_val = s2mu004_i2c_read_byte(i2c, reg);
	reg_val &= ~mask;
	reg_val |= value << shift;
	ret = s2mu004_i2c_write_byte(i2c, reg, reg_val);
	pr_info("%s reg(0x%x) value(0x%x)\n", __func__, reg, reg_val);
	if (ret < 0)
		pr_err("%s  Reg = 0x%X, mask = 0x%X, val = 0x%X write err : %d\n",
				__func__, reg, mask, value, ret);

	return ret;
}

static void s2mu004_muic_set_dn_ready_for_killer(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val, is_vdnmon;
	int i, vdnmon_gnd_cnt;

	usleep_range(10000, 11000);
	is_vdnmon = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_STATUS)
			& MUIC_AFC_STATUS_VDNMON_MASK;

	if (is_vdnmon) {
		reg_val = MUIC_AFC_CTRL1_AFC_EN_MASK | MUIC_AFC_CTRL1_DPDNVD_EN_MASK |
				(MUIC_AFC_CTRL1_VD_SEL_GND << MUIC_AFC_CTRL1_DPVD_SEL_SHIFT) |
				(MUIC_AFC_CTRL1_VD_SEL_GND << MUIC_AFC_CTRL1_DNVD_SEL_SHIFT);
		s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL1, reg_val);

		msleep(50);

		is_vdnmon = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_STATUS)
				& MUIC_AFC_STATUS_VDNMON_MASK;
		if (!is_vdnmon) {
			pr_info("%s done(Line:%d)", __func__, __LINE__);
			goto vdnmon_chk_done;
		}

		vdnmon_gnd_cnt = 0;
		for (i = 0; i < 10; i++) {
			reg_val = MUIC_AFC_CTRL1_AFC_EN_MASK | MUIC_AFC_CTRL1_DPDNVD_EN_MASK |
					(MUIC_AFC_CTRL1_VD_SEL_GND << MUIC_AFC_CTRL1_DPVD_SEL_SHIFT) |
					(MUIC_AFC_CTRL1_VD_SEL_GND << MUIC_AFC_CTRL1_DNVD_SEL_SHIFT);
			s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL1, reg_val);

			usleep_range(20000, 21000);

			reg_val &= ~((MUIC_AFC_CTRL1_VD_SEL_GND << MUIC_AFC_CTRL1_DNVD_SEL_SHIFT) |
					(MUIC_AFC_CTRL1_VD_SEL_GND << MUIC_AFC_CTRL1_DPVD_SEL_SHIFT));
			s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL1, reg_val);

			usleep_range(10000, 11000);
			is_vdnmon = s2mu004_i2c_read_byte(i2c, S2MU004_REG_AFC_STATUS)
					& MUIC_AFC_STATUS_VDNMON_MASK;
			if (!is_vdnmon) {
				vdnmon_gnd_cnt++;
				if (vdnmon_gnd_cnt >= 2) {
					pr_info("%s done(Line:%d)", __func__, __LINE__);
					goto vdnmon_chk_done;
				}
			}
		}
		pr_info("%s done(Line:%d)", __func__, __LINE__);

vdnmon_chk_done:
		reg_val &= ~(MUIC_AFC_CTRL1_VD_SEL_GND << MUIC_AFC_CTRL1_DNVD_SEL_SHIFT);
		s2mu004_i2c_write_byte(i2c, S2MU004_REG_AFC_CTRL1, reg_val);
	}
}

int s2mu004_muic_control_rid_adc(struct s2mu004_muic_data *muic_data, bool enable)
{
	int data = 0;

	pr_info("%s (%s)\n", __func__, enable ? "Enable" : "Disable");

	data = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_RID_CTRL);

	if (enable)
		data &= ~MUIC_CTRL2_ADC_OFF_MASK;
	else
		data |= MUIC_CTRL2_ADC_OFF_MASK;

	s2mu004_i2c_write_byte(muic_data->i2c, S2MU004_REG_MUIC_RID_CTRL, data);

	return 0;
}

int s2mu004_muic_bcd_rescan(struct s2mu004_muic_data *muic_data)
{
	int data = 0;

	pr_info("%s\n", __func__);

	/* start secondary dp dm detect */
	data = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_BCD_RESCAN);
	data |= MUIC_BCD_RESCAN_MASK;
	s2mu004_i2c_write_byte(muic_data->i2c, S2MU004_REG_MUIC_BCD_RESCAN, data);

	return 0;
}

#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC)
int s2mu004_muic_reset_afc_register(struct s2mu004_muic_data *muic_data)
{
	int data = 0;

	pr_info("%s\n", __func__);

	data = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_AFC_LOGIC_CTRL2);
	data |= MUIC_AFC_LOGIC_CTRL2_AFC_RST_MASK;
	s2mu004_i2c_write_byte(muic_data->i2c, S2MU004_REG_AFC_LOGIC_CTRL2, data);

	return 0;
}

int s2mu004_muic_check_afc_ready(struct s2mu004_muic_data *muic_data)
{
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
	struct muic_platform_data *muic_pdata = muic_data->pdata;
#if IS_ENABLED(CONFIG_CCIC_S2MU004)
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;
#endif

	pr_info("%s\n", __func__);

	/* check muic ready for afc */
	if (muic_pdata->afc_disable)
		pr_info("%s AFC Disable(%d) by USER!\n",
				__func__, muic_pdata->afc_disable);
#if IS_ENABLED(CONFIG_CCIC_S2MU004)
	else if (muic_if->is_afc_pdic_ready == false)
		pr_info("%s AFC Disable(%d) by PDIC\n",
				__func__, muic_if->is_afc_pdic_ready);
#endif
	else {
		pr_info("%s ready:%d  afc_check:%d\n", __func__,
				muic_data->is_afc_muic_ready, muic_data->afc_check);
		if (muic_data->is_afc_muic_ready == false && muic_data->afc_check) {
			cancel_delayed_work(&muic_data->prepare_afc_charger);
			schedule_delayed_work(&muic_data->prepare_afc_charger,
					msecs_to_jiffies(200));
		}
	}
#else
	cancel_delayed_work(&muic_data->prepare_afc_charger);
	schedule_delayed_work(&muic_data->prepare_afc_charger, msecs_to_jiffies(200));
#endif

	return 0;
}
#endif

void s2mu004_muic_control_vbus_det(struct s2mu004_muic_data *muic_data, bool enable)
{
	int data = 0;

	pr_info("%s (%s)\n", __func__, enable ? "Enable" : "Disable");

	/* enable vbus det for interrupt */
	data = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_AFC_OTP6);

	if (enable)
		data |= MUIC_AFC_OTP6_EN_VBUS_DET_MUIC_MASK;
	else
		data &= ~MUIC_AFC_OTP6_EN_VBUS_DET_MUIC_MASK;

	s2mu004_i2c_write_byte(muic_data->i2c, S2MU004_REG_AFC_OTP6, data);

}

#if defined(GPIO_USB_SEL)
static int s2mu004_set_gpio_usb_sel(int uart_sel)
{
	return 0;
}
#endif /* GPIO_USB_SEL */

int s2mu004_set_gpio_uart_sel(struct s2mu004_muic_data *muic_data, int uart_sel)
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
#endif/* CONFIG_MUIC_UART_SWITCH */
	return 0;
}

#if IS_ENABLED(GPIO_DOC_SWITCH)
static int s2mu004_set_gpio_doc_switch(struct s2mu004_muic_data *muic_data, int val)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int doc_switch_gpio = muic_pdata->gpio_doc_switch;
	int doc_switch_val;
	int ret;

	ret = gpio_request(doc_switch_gpio, "GPIO_DOC_SWITCH");
	if (ret) {
		pr_err("failed to gpio_request GPIO_DOC_SWITCH\n");
		return ret;
	}

	doc_switch_val = gpio_get_value(doc_switch_gpio);

	if (gpio_is_valid(doc_switch_gpio))
		gpio_set_value(doc_switch_gpio, val);
	doc_switch_val = gpio_get_value(doc_switch_gpio);

	gpio_free(doc_switch_gpio);

	pr_info("%s (%d)%c\n", __func__,
		doc_switch_gpio, (doc_switch_val == 0 ? 'L' : 'H'));

	return 0;
}
#endif /* GPIO_DOC_SWITCH */

#if IS_ENABLED(CONFIG_SEC_FACTORY)
int s2mu004_muic_set_otg_reg(struct s2mu004_muic_data *muic_data, bool on)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val;
	int ret = 0;

	if (on) {
		muic_data->attach_mode = S2MU004_MUIC_OTG;
		/* enable vbus det for interrupt */
		s2mu004_muic_control_vbus_det(muic_data, true);
	} else {
		/* disable vbus det for interrupt */
		s2mu004_muic_control_vbus_det(muic_data, false);
	}

	/* 0x1e : hidden register */
	ret = s2mu004_i2c_read_byte(i2c, 0x1e);
	if (ret < 0)
		pr_err("%s err read 0x1e reg(%d)\n", __func__, ret);

	/* Set 0x1e[5:4] bit to 0x11 or 0x01 */
	if (on)
		reg_val = ret | (0x1 << 5);
	else
		reg_val = ret & ~(0x1 << 5);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu004_i2c_guaranteed_wbyte(i2c, 0x1e, reg_val);
		if (ret < 0)
			pr_err("%s err write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n", __func__, reg_val, ret);
		return 0;
	}

	ret = s2mu004_i2c_read_byte(i2c, 0x1e);
	if (ret < 0)
		pr_err("%s err read reg 0x1e(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n", __func__, ret);

	return ret;
}

static int s2mu004_muic_init_otg_reg(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val;
	int ret = 0;

	/* 0x73 : check EVT0 or EVT1 */
	ret = s2mu004_i2c_read_byte(i2c, 0x73);
	if (ret < 0)
		pr_err("%s err read 'reg 0x73'(%d)\n", __func__, ret);

	if ((ret&0xF) > 0)
		return 0;

	/* 0x89 : hidden register */
	ret = s2mu004_i2c_read_byte(i2c, 0x89);
	if (ret < 0)
		pr_err("%s err read 'reg 0x89'(%d)\n", __func__, ret);

	/* Set 0x89[1] bit : T_DET_VAL */
	reg_val = ret | (0x1 << 1);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu004_i2c_guaranteed_wbyte(i2c, 0x89, reg_val);
		if (ret < 0)
			pr_err("%s err write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n", __func__, reg_val, ret);
		return 0;
	}

	ret = s2mu004_i2c_read_byte(i2c, 0x89);
	if (ret < 0)
		pr_err("%s err read 'reg 0x89'(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n", __func__, ret);

	/* 0x92 : hidden register */
	ret = s2mu004_i2c_read_byte(i2c, 0x92);
	if (ret < 0)
		pr_err("%s err read 'reg 0x92'(%d)\n", __func__, ret);

	/* Set 0x92[7] bit : EN_JIG_AP */
	reg_val = ret | (0x1 << 7);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu004_i2c_guaranteed_wbyte(i2c, 0x92, reg_val);
		if (ret < 0)
			pr_err("%s err write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n",	__func__, reg_val, ret);
		return 0;
	}

	ret = s2mu004_i2c_read_byte(i2c, 0x92);
	if (ret < 0)
		pr_err("%s err read 'reg 0x92'(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n", __func__, ret);

	return ret;
}
#endif /* CONFIG_SEC_FACTORY */

/* TODO: There is no needs to use JIGB pin by MUIC if CCIC is supported */
int s2mu004_muic_jig_on(struct s2mu004_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	bool en = muic_pdata->is_jig_on;
	int reg = 0, ret = 0;

	pr_err("%s: %s\n", __func__, en ? "on" : "off");

	reg = s2mu004_i2c_read_byte(muic_data->i2c,
		S2MU004_REG_MUIC_SW_CTRL);

	if (en)
		reg |= MANUAL_SW_JIG_EN;
	else
		reg &= ~(MANUAL_SW_JIG_EN);

	ret = s2mu004_i2c_write_byte(muic_data->i2c,
			S2MU004_REG_MUIC_SW_CTRL, (u8)reg);

	return ret;

}

static int s2mu004_muic_set_ctrl_reg(struct s2mu004_muic_data *muic_data, int shift, bool on)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val;
	int ret = 0;

	ret = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_CTRL1);
	if (ret < 0)
		pr_err("%s err read CTRL(%d)\n", __func__, ret);

	if (on)
		reg_val = ret | (0x1 << shift);
	else
		reg_val = ret & ~(0x1 << shift);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n",
			__func__, reg_val, ret);

		ret = s2mu004_i2c_guaranteed_wbyte(i2c, S2MU004_REG_MUIC_CTRL1,
				reg_val);
		if (ret < 0)
			pr_err("%s err write(%d)\n",
				__func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n",
			__func__, reg_val, ret);
		return 0;
	}

	ret = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_CTRL1);
	if (ret < 0)
		pr_err("%s err read CTRL(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n",
			__func__, ret);

	return ret;
}

static int s2mu004_muic_set_int_mask(struct s2mu004_muic_data *muic_data, bool on)
{
	int shift = CTRL_INT_MASK_SHIFT;
	int ret = 0;

	ret = s2mu004_muic_set_ctrl_reg(muic_data, shift, on);

	return ret;
}

static int s2mu004_muic_set_com_sw(struct s2mu004_muic_data *muic_data,
	u8 reg_val)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0;
	int temp = 0;

	/*  --- MANSW [7:5][4:2][1][0] : DM DP RSVD JIG  --- */
	temp = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_SW_CTRL);
	if (temp < 0)
		pr_err("%s err read MANSW(0x%x)\n", __func__, temp);

#if defined(CONFIG_CCIC_S2MU004) && defined(CONFIG_HICCUP_CHARGER)
	if (muic_data->is_hiccup_mode && IS_WATER_STATUS(muic_data->water_status))
		reg_val = MANSW_HICCUP;
#endif

	if ((reg_val & MANUAL_SW_DM_DP_MASK) != (temp & MANUAL_SW_DM_DP_MASK)) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__,
			(reg_val & MANUAL_SW_DM_DP_MASK), (temp & MANUAL_SW_DM_DP_MASK));

		ret = s2mu004_i2c_guaranteed_wbyte(i2c,
			S2MU004_REG_MUIC_SW_CTRL, ((reg_val & MANUAL_SW_DM_DP_MASK)|(temp & 0x03)));
		if (ret < 0)
			pr_err("%s err write MANSW(0x%x)\n", __func__,
			(reg_val & MANUAL_SW_DM_DP_MASK) | (temp & 0x03));
	} else {
		pr_info("%s MANSW reg(0x%x), just pass\n", __func__, reg_val);
	}

	return ret;
}

int s2mu004_muic_com_to_open_with_vbus(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;

	reg_val = MANSW_OPEN_WITH_VBUS;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_sw err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_open(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;
	u8 vbvolt;

	pr_info("%s\n", __func__);

	vbvolt = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_DEVICE_APPLE);
	vbvolt &= DEV_TYPE_APPLE_VBUS_WAKEUP;
	if (vbvolt) {
		reg_val = MANSW_OPEN_WITH_VBUS;
		ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
		if (ret)
			pr_err("%s s2mu004_muic_set_com_sw err\n", __func__);
	}

	reg_val = MANSW_OPEN;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_sw err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_usb(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;

	reg_val = MANSW_USB;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_usb err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_otg(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;

	reg_val = MANSW_OTG;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_otg err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_uart(struct s2mu004_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	u8 reg_val;
	int ret = 0;

	if (muic_data->pdata->is_rustproof) {
		pr_info("%s rustproof mode\n", __func__);
		return ret;
	}

	if (muic_pdata->uart_path == MUIC_PATH_UART_AP) {
		reg_val = MANSW_UART_AP;
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
		send_uart_noti_to_modem(MODEM_CTRL_UART_AP);
#endif
	} else {
		reg_val = MANSW_UART_CP;
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
		send_uart_noti_to_modem(MODEM_CTRL_UART_CP);
#endif
	}

	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_uart err\n", __func__);

	return ret;
}

int s2mu004_muic_com_to_audio(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;

	reg_val = MANSW_AUDIO;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_audio err\n", __func__);

	return ret;
}

#if defined(CONFIG_HICCUP_CHARGER)
int s2mu004_muic_com_to_gnd(struct s2mu004_muic_data *muic_data)
{
	u8 reg_val;
	int ret = 0;

	reg_val = MANSW_HICCUP;
	ret = s2mu004_muic_set_com_sw(muic_data, reg_val);
	if (ret)
		pr_err("%s set_com_audio err\n", __func__);

	return ret;
}
#endif

static int s2mu004_muic_set_rid_adc_en(struct s2mu004_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0;

	pr_info("%s rid en : (%d)\n", __func__, en);
	if (en) {
		/* enable rid detection for muic dp dm detect */
		ret = s2mu004_i2c_update_bit(i2c,
				S2MU004_REG_MUIC_RID_CTRL, RID_CTRL_ADC_OFF_MASK, RID_CTRL_ADC_OFF_SHIFT, 0x0);
	} else {
		/* disable rid detection for muic dp dm detect */
		ret = s2mu004_i2c_update_bit(i2c,
				S2MU004_REG_MUIC_RID_CTRL, RID_CTRL_ADC_OFF_MASK, RID_CTRL_ADC_OFF_SHIFT, 0x1);
	}

	return ret;
}

#if defined(CONFIG_CCIC_S2MU004)
static void s2mu004_muic_set_rid_int_mask_en(struct s2mu004_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 irq_reg[S2MU004_IRQ_GROUP_NR] = {0};

	pr_info("%s en : %d\n",	__func__, (int)en);

	if (en) {
		s2mu004_i2c_update_bit(i2c,
			S2MU004_REG_MUIC_INT2_MASK,
			INT_ADC_CHANGE_MASK | INT_RSRV_ATTACH_MASK,
			0, INT_ADC_CHANGE_MASK | INT_RSRV_ATTACH_MASK);
		s2mu004_i2c_update_bit(i2c,
			S2MU004_REG_MUIC_INT1_MASK,
			INT_DETACH_MASK | INT_ATTACH_MASK,
			0, INT_DETACH_MASK | INT_ATTACH_MASK);
	} else {
		s2mu004_bulk_read(i2c, S2MU004_REG_MUIC_INT1,
				S2MU004_NUM_IRQ_MUIC_REGS, &irq_reg[MUIC_INT1]);

		s2mu004_i2c_update_bit(i2c,
				S2MU004_REG_MUIC_INT2_MASK,
				INT_ADC_CHANGE_MASK | INT_RSRV_ATTACH_MASK,
				0, 0);

		s2mu004_i2c_update_bit(i2c,
				S2MU004_REG_MUIC_INT1_MASK,
				INT_DETACH_MASK | INT_ATTACH_MASK,
				0, 0);
	}
}
#endif

int s2mu004_muic_recheck_adc(struct s2mu004_muic_data *muic_data)
{
	int i = 0;
	struct i2c_client *i2c = muic_data->i2c;
	int adc = ADC_OPEN;
	u8 chk_int;

	s2mu004_muic_set_rid_adc_en(muic_data, false);
	usleep_range(10000, 12000);
	s2mu004_read_reg(i2c, 0x6, &chk_int);
	s2mu004_muic_set_rid_adc_en(muic_data, true);
	usleep_range(20000, 21000);

	for (i = 0; i < 50; i++) {
		usleep_range(1000, 1050);
		adc = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_ADC) & ADC_MASK;
		if (adc != ADC_OPEN) {
			pr_info("%s, %d th try, adc : 0x%x\n", __func__, i, adc);
			return adc;
		}
	}

	usleep_range(10000, 10500);
	pr_info("%s, after delay\n", __func__);
	return s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_ADC) & ADC_MASK;
}

int s2mu004_muic_refresh_adc(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int adc = 0;
	u8 reg_data, b_Rid_en = 0;

	reg_data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_RID_CTRL);
	if (!(reg_data & 0x2)) {
		b_Rid_en = 1;
	} else {
		pr_info("%s, enable the RID\n", __func__);
		reg_data &= ~(0x01 << 1);
		s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_RID_CTRL, reg_data);
		msleep(35);
	}

	adc = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_ADC);
	pr_info("%s, adc : 0x%X\n", __func__, adc);

	if (!b_Rid_en) {
		pr_info("%s, disable the RID\n", __func__);
		reg_data |= (0x01 << 1);
		s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_RID_CTRL, reg_data);
	}
	return adc;
}

int s2mu004_muic_get_vbus_state(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val = 0;
	int vbus = 0;

	reg_val = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_APPLE);
	vbus = !!(reg_val & DEV_TYPE_APPLE_VBUS_WAKEUP);
	pr_info("%s vbus : (%d)\n", __func__, vbus);
	return vbus;
}
static int s2mu004_muic_reg_init(struct s2mu004_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0, adc = 0;
	int read_val[READ_VAL_MAX_NUM] = {0, };
#if defined(CONFIG_CCIC_S2MU004)
	u8 reg_data = 0;
#endif
#if !IS_ENABLED(CONFIG_MUIC_S2MU004_NON_USB_C_TYPE)
#if !IS_ENABLED(CONFIG_CCIC_S2MU004)
	int data = 0;
#endif /* CONFIG_CCIC_S2MU004 */
#endif /* CONFIG_MUIC_S2MU004_NON_USB_C_TYPE */
#if defined(CONFIG_VBUS_NOTIFIER)
	u8 vbvolt;
#endif

	pr_info("%s\n", __func__);

#if defined(CONFIG_VBUS_NOTIFIER)
	vbvolt = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_DEVICE_APPLE);
	vbvolt &= DEV_TYPE_APPLE_VBUS_WAKEUP;
#endif

	read_val[READ_VAL_DEVICE_TYPE1] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE1);
	read_val[READ_VAL_DEVICE_TYPE2] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE2);
	read_val[READ_VAL_DEVICE_TYPE3] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE3);
	read_val[READ_VAL_ADC] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_ADC);
	adc = read_val[READ_VAL_ADC];
	pr_info("dev[1:0x%x, 2:0x%x, 3:0x%x], adc:0x%x\n", read_val[READ_VAL_DEVICE_TYPE1],
		read_val[READ_VAL_DEVICE_TYPE2], read_val[READ_VAL_DEVICE_TYPE3], adc);

#if !IS_ENABLED(CONFIG_MUIC_S2MU004_NON_USB_C_TYPE)
	pr_info("%s %s s2mu004 usb-c type\n", MFD_DEV_NAME, __func__);
#if IS_ENABLED(CONFIG_CCIC_S2MU004)
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_INT1_MASK, INT_PDIC_MASK1);
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_INT2_MASK, INT_PDIC_MASK2);

	/* set dcd timer out to 0.6s */
	reg_data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_TIMER_SET3);
	reg_data &= ~TIMER_SET3_DCDTMRSET_MASK;
	reg_data |= (TIMER_SET3_DCDTMRSET_MASK | 0x4);
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_TIMER_SET3, reg_data);

	/* enable rid detect for waterproof */
	s2mu004_muic_control_rid_adc(muic_data, S2MU004_ENABLE);
#else
	s2mu004_muic_control_rid_adc(muic_data, S2MU004_DISABLE);

	/* adc, RID int masking */
	data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_INT1_MASK);
	data |= (INTm_ATTACH_MASK | INTm_KP_MASK);
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_INT1_MASK, data);

	data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_INT2_MASK);
	data |= INTm_ADC_CHANGE_MASK;
	s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_INT2_MASK, data);
#endif /* CONFIG_CCIC_S2MU004 */
#endif /* CONFIG_MUIC_S2MU004_NON_USB_C_TYPE */

	s2mu004_muic_control_vbus_det(muic_data, false);
	ret = s2mu004_i2c_guaranteed_wbyte(i2c,
			S2MU004_REG_MUIC_CTRL1, CTRL_MASK);
	if (ret < 0)
		pr_err("failed to write ctrl(%d)\n", ret);

	/*
	* These registers represents the RID ADC LDO voltage control.
	* Low / High LDO initialized to 3V, 2.7V each.
	*/
	s2mu004_i2c_update_bit(i2c,
			S2MU004_REG_LDOADC_VSETL, LDOADC_VSETH_MASK, 0, LDOADC_VSET_3V);
	s2mu004_i2c_update_bit(i2c,
			S2MU004_REG_LDOADC_VSETH, LDOADC_VSETH_MASK, 0, LDOADC_VSET_2_7V);

	s2mu004_i2c_update_bit(i2c,
			S2MU004_REG_LDOADC_VSETH,
			LDOADC_VSETH_WAKE_HYS_MASK,
			LDOADC_VSETH_WAKE_HYS_SHIFT, 0x1);

#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL) && !IS_ENABLED(CONFIG_SEC_FACTORY)
    reg_data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_INT1_MASK);
    reg_data |= INTm_RID_CHG_MASK;
    s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_INT1_MASK, reg_data);

    reg_data = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_INT2_MASK);
    reg_data |= INTm_ADC_CHANGE_MASK;
    s2mu004_i2c_write_byte(i2c, S2MU004_REG_MUIC_INT2_MASK, reg_data);
#endif

#if defined(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle((!!vbvolt) ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */

	return ret;
}
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
int s2mu004_muic_get_otg_state(void)
{
	struct power_supply *psy_otg;
	union power_supply_propval val;
	int ret = 0;

	psy_otg = get_power_supply_by_name("otg");
	if (psy_otg)
		ret = psy_otg->desc->get_property(psy_otg, POWER_SUPPLY_PROP_CHARGE_POWERED_OTG_CONTROL, &val);
	else
		pr_info("%s get psy otg failed\n", __func__);

	if (ret) {
		pr_info("%s get prop fail\n", __func__);
	} else {
		pr_info("%s is ocp ?: %d\n", __func__, val.intval);
		return val.intval;
	}
	return 0;
}
#endif

#if defined(CONFIG_CCIC_S2MU004)
static int s2mu004_muic_detect_ccic_jig_cable(struct s2mu004_muic_data *muic_data,
	int vbvolt, int *intr, muic_attached_dev_t *new_dev)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	if (vbvolt) {
		if (muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_OFF_VB_MUIC ||
			muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_OFF_MUIC) {
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
			pr_info("MUIC JIG UART OFF VB OFF\n");
			return 0;
		} else if (muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_ON_VB_MUIC ||
			muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_ON_MUIC) {
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
			pr_info("MUIC JIG UART ON VB OFF\n");
			return 0;
		} else
			return -1;
	} else {
		if (muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_OFF_VB_MUIC ||
			muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_OFF_MUIC) {
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
			pr_info("MUIC JIG UART OFF VB OFF\n");
			return 0;
		} else if (muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_ON_VB_MUIC ||
			muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_ON_MUIC) {
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
			pr_info("MUIC JIG UART ON VB OFF\n");
			return 0;
		} else
			return -1;
	}
}

static void s2mu004_muic_detect_first_attach(struct s2mu004_muic_data *muic_data, int vbvolt)
{
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;
	int check_adc = 0;

	muic_data->jig_state = false;
	muic_data->re_detect = 0;
	muic_data->afc_check = false;

	if (muic_data->is_dcd_recheck)
		muic_data->is_dcd_recheck = false;

	s2mu004_muic_control_rid_adc(muic_data, S2MU004_ENABLE);

	msleep(100);

	check_adc = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_MUIC_ADC);
	pr_info("%s : check adc at detach (%x)\n", __func__, check_adc);

	if (check_adc == 0) {
		muic_data->attach_mode = S2MU004_FIRST_ATTACH;
		muic_if->is_dcdtmr_intr = true;
		MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_TYPE3_MUIC);
		if (vbvolt) {
			pr_info("%s change mode to second attach!\n", __func__);
			schedule_delayed_work(&muic_data->dcd_recheck, 0);

			muic_data->attach_mode = S2MU004_SECOND_ATTACH;
		}
	} else {
		muic_data->attach_mode = S2MU004_NONE_CABLE;
		MUIC_SEND_NOTI_TO_CCIC_DETACH(muic_data->pdata->attached_dev);
	}

	if (muic_data->otg_state)
		muic_data->otg_state = false;
}

static void s2mu004_muic_detect_second_attach(struct s2mu004_muic_data *muic_data)
{
	pr_info("%s change mode to second attach!\n", __func__);
	/* disable rid detection for muic dp dm detect */
	s2mu004_muic_control_rid_adc(muic_data, S2MU004_DISABLE);
	msleep(100);
	/* start secondary dp dm detect */
	s2mu004_muic_bcd_rescan(muic_data);

	muic_data->attach_mode = S2MU004_SECOND_ATTACH;
}

static int s2mu004_muic_detect_array_jig_cable(struct s2mu004_muic_data *muic_data,
	int adc, int vbvolt, int *intr, muic_attached_dev_t *new_dev)
{
	int ret = 0;

	if (adc == ADC_JIG_UART_OFF) {
		pr_info("ADC_JIG_UART_OFF\n");
		*intr = MUIC_INTR_ATTACH;
		if (vbvolt)
			*new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		else
			*new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		muic_data->jig_state = true;
		ret = true;
	} else if (adc == ADC_JIG_USB_ON || adc == ADC_JIG_USB_OFF ||
		adc == ADC_DESKDOCK) {
		*intr = MUIC_INTR_ATTACH;
		*new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		pr_info("ADC JIG_USB_ON DETECTED\n");
		muic_data->jig_state = true;
		ret = true;
#if defined(CONFIG_SEC_FACTORY)
	} else if (adc == ADC_CEA936ATYPE1_CHG) {
		if (!vbvolt)
			return -1;
		else {
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_CARKIT_MUIC;
			pr_info("SMD DL 255k 200k charger disable\n");
		}
		muic_data->jig_state = true;
		ret = true;
#endif
	}
	return ret;
}

static int s2mu004_muic_detect_with_ccic(struct s2mu004_muic_data *muic_data,
	int adc, int vbvolt, int *intr, muic_attached_dev_t *new_dev)
{
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;

	if (muic_if == NULL) {
		pr_err("%s if data NULL\n", __func__);
		return S2MU004_DETECT_SKIP;
	}

#if defined(CONFIG_CCIC_S2MU004)
	if (muic_data->attach_mode == S2MU004_MUIC_DETACH) {
		/* FIXME: for VB on-off case */
		if ((adc == 0) && muic_data->jig_state) {
#if defined(CONFIG_VBUS_NOTIFIER)
			vbus_notifier_handle((!!vbvolt) ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
			muic_data->attach_mode = S2MU004_FIRST_ATTACH;
			if (s2mu004_muic_detect_ccic_jig_cable(muic_data, vbvolt, intr, new_dev))
				return S2MU004_DETECT_SKIP;
			else
				return S2MU004_DETECT_JIG;
		} else {
			s2mu004_muic_detect_first_attach(muic_data, vbvolt);
		}
	} else if (muic_data->attach_mode == S2MU004_NONE_CABLE) {
#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		if (muic_if->opmode & OPMODE_MUIC) {
			if (s2mu004_muic_detect_array_jig_cable(muic_data, adc, vbvolt, intr, new_dev))
				return S2MU004_DETECT_JIG;
		}
#else
		if (s2mu004_muic_detect_array_jig_cable(muic_data, adc, vbvolt, intr, new_dev))
			return S2MU004_DETECT_JIG;
#endif
		/*
		* opmode usage :
		* When it's an array status,
		* the attach mode should maintain the none cable type.
		* Since the vbvolt can be set in case of the array,
		* this first attach should be running only when it's not the array status.
		*/
		if ((muic_if->opmode & OPMODE_CCIC) && ((adc == ADC_GND) || vbvolt)) {
			pr_info("%s change mode to first attach!\n", __func__);
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_TYPE3_MUIC;
			muic_data->attach_mode = S2MU004_FIRST_ATTACH;
			MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_TYPE3_MUIC);
			if (vbvolt) {
				if (muic_data->jig_state == true) {
					pr_info("%s : not enter second attach for jig\n", __func__);
					if (s2mu004_muic_detect_ccic_jig_cable(muic_data,
												vbvolt, intr, new_dev))
						return S2MU004_DETECT_SKIP;
					else
						return S2MU004_DETECT_JIG;
				}
				s2mu004_muic_detect_second_attach(muic_data);
			}
			return S2MU004_DETECT_SKIP;
		}
	} else if (muic_data->attach_mode == S2MU004_FIRST_ATTACH) {
		if (vbvolt) {
			if (muic_data->jig_state == true) {
				pr_info("%s : not enter second attach for jig\n", __func__);
				if (s2mu004_muic_detect_ccic_jig_cable(muic_data, vbvolt, intr, new_dev))
					return S2MU004_DETECT_SKIP;
				else
					return S2MU004_DETECT_JIG;
			}
			MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_TYPE3_MUIC);
			s2mu004_muic_detect_second_attach(muic_data);
		}
		return S2MU004_DETECT_SKIP;
	} else if (!vbvolt)
		return S2MU004_DETECT_SKIP;

	if (muic_data->attach_mode == S2MU004_MUIC_OTG && vbvolt) {
#if defined(CONFIG_VBUS_NOTIFIER)
		vbus_notifier_handle((!!vbvolt) ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
		return S2MU004_DETECT_SKIP;
	}
#endif /* CONFIG_CCIC_S2MU004 */
	return S2MU004_DETECT_DONE;
}
#endif

static void s2mu004_muic_get_detect_info(struct s2mu004_muic_data *muic_data,
	int *read_val, int *adc, int *vbvolt, int *vmid)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	struct i2c_client *i2c = muic_data->i2c;

	read_val[READ_VAL_DEVICE_TYPE1] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE1);
	read_val[READ_VAL_DEVICE_TYPE2] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE2);
	read_val[READ_VAL_DEVICE_TYPE3] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_TYPE3);
	read_val[READ_VAL_REV_ID] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_REV_ID);
	read_val[READ_VAL_ADC] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_ADC);
	read_val[READ_VAL_DEVICE_APPLE] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_DEVICE_APPLE);
	read_val[READ_VAL_CHG_TYPE] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_CHG_TYPE);
	read_val[READ_VAL_SC_STATUS2] = s2mu004_i2c_read_byte(i2c, S2MU004_REG_SC_STATUS2);

	muic_pdata->adc = *adc = read_val[READ_VAL_ADC];
	muic_pdata->vbvolt = *vbvolt = !!(read_val[READ_VAL_DEVICE_APPLE] & DEV_TYPE_APPLE_VBUS_WAKEUP);
	*vmid = !!(read_val[READ_VAL_SC_STATUS2] & 0x7);

	pr_info("dev[1:0x%02x, 2:0x%02x, 3:0x%02x]\n",
		read_val[READ_VAL_DEVICE_TYPE1],
		read_val[READ_VAL_DEVICE_TYPE2],
		read_val[READ_VAL_DEVICE_TYPE3]);
	pr_info("adc:0x%02x, vbvolt:0x%02x, apple:0x%02x\n",
		*adc, *vbvolt, read_val[READ_VAL_DEVICE_APPLE]);
	pr_info("chg_type:0x%02x, vmid:0x%02x, dev_id:0x%02x\n",
		read_val[READ_VAL_CHG_TYPE], *vmid, read_val[READ_VAL_REV_ID]);
}

static int s2mu004_muic_detect_by_dpdm(struct s2mu004_muic_data *muic_data,
	int *read_val, int adc, int vbvolt, int *intr, muic_attached_dev_t *new_dev)
{
#if defined(CONFIG_MUIC_MANAGER)
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;
#endif
	/* Attached */
	switch (read_val[READ_VAL_DEVICE_TYPE1]) {
	case DEV_TYPE1_CDP:
		if (vbvolt) {
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_CDP_MUIC;
			pr_info("USB_CDP DETECTED\n");
		}
		break;
	case DEV_TYPE1_USB:
		if (vbvolt) {
#ifdef	CONFIG_MUIC_MANAGER
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("USB DETECTED\n");
#else /* CONFIG_MUIC_MANAGER */
			if (muic_data->bcd_rescan_cnt++ < MAX_BCD_RESCAN_CNT) {
				schedule_delayed_work(&muic_data->dcd_recheck, msecs_to_jiffies(10));
				return S2MU004_DETECT_SKIP;
			} else {
				*intr = MUIC_INTR_ATTACH;
				*new_dev = ATTACHED_DEV_USB_MUIC;
				pr_info("USB DETECTED\n");
			}
#endif
		}
		break;
	case DEV_TYPE1_DEDICATED_CHG:
	case 0x44:
	case 0x60:
		if (vbvolt) {
#ifdef	CONFIG_MUIC_MANAGER
			muic_if->is_dcp_charger = true;
			if (muic_data->is_dcd_recheck) {
				muic_data->is_dcd_recheck = false;
				cancel_delayed_work(&muic_data->incomplete_check);
			}
#endif /* CONFIG_MUIC_MANAGER */
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_TA_MUIC;
			muic_data->afc_check = true;
			pr_info("DEDICATED CHARGER DETECTED\n");
		}
		break;
#ifdef CONFIG_MUIC_S2MU004_NON_USB_C_TYPE
	case DEV_TYPE1_USB_OTG:
		*intr = MUIC_INTR_ATTACH;
		*new_dev = ATTACHED_DEV_OTG_MUIC;
		pr_info("USB_OTG DETECTED\n");
		break;
#endif /* CONFIG_MUIC_S2MU004_NON_USB_C_TYPE */
	case DEV_TYPE1_T1_T2_CHG:
		if (vbvolt) {
			*intr = MUIC_INTR_ATTACH;
			/* 200K, 442K should be checkef */
			if (adc == ADC_CEA936ATYPE2_CHG) {
				*new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
				pr_info("CEA936ATYPE2_CHG DETECTED\n");
				muic_data->afc_check = false;
			} else {
				*new_dev = ATTACHED_DEV_USB_MUIC;
				pr_info("T1_T2_CHG DETECTED\n");
			}
		}
		break;
	default:
		break;
	}

	switch (read_val[READ_VAL_DEVICE_TYPE2]) {
	case DEV_TYPE2_SDP_1P8S:
		if (vbvolt) {
#if IS_ENABLED(CONFIG_SEC_FACTORY)
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("%s:%s: SDP_1P8S=>USB DETECTED\n", MUIC_DEV_NAME, __func__);
#else
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_TIMEOUT_OPEN_MUIC;
			pr_info("%s:%s: SDP_1P8S DETECTED\n", MUIC_DEV_NAME, __func__);
			muic_if->is_dcdtmr_intr = true;
			schedule_delayed_work(&muic_data->dcd_recheck, 0);
#else
			if (muic_data->bcd_rescan_cnt++ < MAX_BCD_RESCAN_CNT) {
				schedule_delayed_work(&muic_data->dcd_recheck, msecs_to_jiffies(10));
				return S2MU004_DETECT_SKIP;
			} else {
				*intr = MUIC_INTR_ATTACH;
				*new_dev = ATTACHED_DEV_TIMEOUT_OPEN_MUIC;
				pr_info("%s:%s: SDP_1P8S DETECTED\n", MUIC_DEV_NAME, __func__);
			}
#endif	/* CONFIG_MUIC_MANAGER */
#endif	/* CONFIG_SEC_FACTORY */
		}
		break;
	default:
		break;
	}
	return S2MU004_DETECT_DONE;
}

#if defined(CONFIG_MUIC_MANAGER)
static int s2mu004_muic_detect_jig_dev_type(struct s2mu004_muic_data *muic_data,
	int *read_val, int vbvolt, int *intr, muic_attached_dev_t *new_dev)
{
	pr_info("%s:%s s2mu004 non usb-c type\n", MFD_DEV_NAME, __func__);
	switch (read_val[READ_VAL_DEVICE_TYPE2]) {
	case DEV_TYPE2_JIG_UART_OFF:
		*intr = MUIC_INTR_ATTACH;
		if (vbvolt)
			*new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		else
			*new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		pr_info("JIG_UART_OFF DETECTED\n");
		break;
	case DEV_TYPE2_JIG_USB_OFF:
		if (!vbvolt)
			break;
		*intr = MUIC_INTR_ATTACH;
		*new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
		pr_info("JIG_USB_OFF DETECTED\n");
		break;
	case DEV_TYPE2_JIG_USB_ON:
		if (!vbvolt)
			break;
		*intr = MUIC_INTR_ATTACH;
		*new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		pr_info("JIG_USB_ON DETECTED\n");
		break;
	case DEV_TYPE2_JIG_UART_ON:
		if (*new_dev != ATTACHED_DEV_JIG_UART_ON_MUIC) {
			*intr = MUIC_INTR_ATTACH;
#if defined(CONFIG_MUIC_SUPPORT_TYPEB)
			*new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
			pr_info("ADC JIG_UART_ON DETECTED\n");
#else
			if (!vbvolt) {
				*new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
				pr_info("ADC JIG_UART_ON DETECTED\n");
			} else {
				*new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
				pr_info("ADC JIG_UART_ON_VB DETECTED\n");
			}
#endif
		}
		break;
	default:
		break;
	}

    switch (read_val[READ_VAL_DEVICE_TYPE3]) {
    case DEV_TYPE3_VBUS_R255:
	    if (!vbvolt)
            break;
        *intr = MUIC_INTR_ATTACH;
        *new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
        pr_info("JIG_USB_OFF DETECTED\n");
		break;
	default:
		break;
	}
	return S2MU004_DETECT_DONE;
}
#endif

#ifdef CONFIG_MUIC_S2MU004_NON_USB_C_TYPE
static void s2mu004_muic_detect_jig_by_adc(struct s2mu004_muic_data *muic_data,
	int *read_val, int adc, int vbvolt, int *vmid, int *intr, muic_attached_dev_t *new_dev)
{
	if (*new_dev == ATTACHED_DEV_UNKNOWN_MUIC) {
		switch (adc) {
		case ADC_CEA936ATYPE1_CHG: /*200k ohm */
			if (vbvolt) {
				*intr = MUIC_INTR_ATTACH;
				/* This is workaournd for LG USB cable which has 219k ohm ID */
				*new_dev = ATTACHED_DEV_USB_MUIC;
				pr_info("TYPE1 CHARGER DETECTED(USB)\n");
			}
			break;
		case ADC_CEA936ATYPE2_CHG:
			if (vbvolt) {
				*intr = MUIC_INTR_ATTACH;
				*new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
				muic_data->afc_check = false;
				pr_info("%s unsupported ADC(0x%02x)\n",
				__func__, adc);
			}
			break;
		case ADC_JIG_USB_OFF: /* 255k */
			if (!vbvolt)
				break;
			if (*new_dev != ATTACHED_DEV_JIG_USB_OFF_MUIC) {
				*intr = MUIC_INTR_ATTACH;
				*new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
				pr_info("ADC JIG_USB_OFF DETECTED\n");
			}
			break;
		case ADC_JIG_USB_ON:
			if (!vbvolt)
				break;
			if (*new_dev != ATTACHED_DEV_JIG_USB_ON_MUIC) {
				*intr = MUIC_INTR_ATTACH;
				*new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
				pr_info("ADC JIG_USB_ON DETECTED\n");
			}
			break;
		case ADC_JIG_UART_OFF:
			*intr = MUIC_INTR_ATTACH;
			if (muic_data->pdata->is_otg_test) {
				mdelay(100);
				read_val[READ_VAL_SC_STATUS2] = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_SC_STATUS2);
				*vmid = read_val[READ_VAL_SC_STATUS2] & 0x7;
				if (*vmid == 0x4) {
					pr_info("OTG_TEST DETECTED, vmid = %d\n", *vmid);
					vbvolt = 1;
					*new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC;
				} else
					*new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
			} else if (vbvolt)
				*new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
			else
				*new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;

			pr_info("ADC JIG_UART_OFF DETECTED\n");
			break;
		case ADC_JIG_UART_ON:
			if (*new_dev != ATTACHED_DEV_JIG_UART_ON_MUIC) {
				*intr = MUIC_INTR_ATTACH;
#if defined(CONFIG_MUIC_SUPPORT_TYPEB)
				*new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
				pr_info("ADC JIG_UART_ON DETECTED\n");
#else
				if (!vbvolt) {
					*new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
					pr_info("ADC JIG_UART_ON DETECTED\n");
				} else {
					*new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
					pr_info("ADC JIG_UART_ON_VB DETECTED\n");
				}
#endif
			}
			break;
		case ADC_SMARTDOCK: /* 0x10000 40.2K ohm */
			/* SMARTDOCK is not supported */
			/* force not to charge the device with SMARTDOCK */
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_UNSUPPORTED_ID_VB_MUIC;
			pr_info("%s unsupported ADC(0x%02x) but charging\n",
				__func__, adc);
			break;
		case ADC_HMT: /* 0x10001 49.9K ohm */
			*new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
			*intr = MUIC_INTR_ATTACH;
			pr_info("%s unsupported ADC(0x%02x) not charging\n",
				__func__, adc);
			break;
		case ADC_AUDIODOCK:
			*intr = MUIC_INTR_ATTACH;
#ifdef CONFIG_MUIC_S2MU004_SUPPORT_AUDIODOCK
			*new_dev = ATTACHED_DEV_AUDIODOCK_MUIC;
#else
			*new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
#endif
			pr_info("ADC AUDIODOCK DETECTED\n");
			break;
		case ADC_UNIVERSAL_MMDOCK:
			*new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
			*intr = MUIC_INTR_ATTACH;
			pr_info("%s unsupported ADC(0x%02x) not charging\n",
				__func__, adc);
			break;
		case ADC_OPEN:
			/* sometimes muic fails to catch JIG_UART_OFF detaching */
			/* double check with ADC */
			if (*new_dev == ATTACHED_DEV_JIG_UART_OFF_MUIC) {
				*new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
				*intr = MUIC_INTR_DETACH;
				pr_info("ADC OPEN DETECTED\n");
			}
			break;
		default:
			*intr = MUIC_INTR_ATTACH;
			*new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
			pr_info("%s unsupported ADC(0x%02x)\n",
				__func__, adc);
			break;
		}
	}
}
#endif

static void s2mu004_muic_detect_extra_cable(struct s2mu004_muic_data *muic_data,
	int *read_val, int vbvolt, int *intr, muic_attached_dev_t *new_dev)
{
#if defined(CONFIG_MUIC_MANAGER)
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;
#endif
	/* This is for Apple cables */
	if (vbvolt && ((read_val[READ_VAL_DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE2P4A_CHG)
		|| (read_val[READ_VAL_DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE2A_CHG)
		|| (read_val[READ_VAL_DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE1A_CHG)
		|| (read_val[READ_VAL_DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE0P5A_CHG))) {
		*intr = MUIC_INTR_ATTACH;
		*new_dev = ATTACHED_DEV_TA_MUIC;
		muic_data->afc_check = false;
		pr_info("APPLE_CHG DETECTED\n");
#ifdef	CONFIG_MUIC_MANAGER
		muic_if->is_dcdtmr_intr = true;
		schedule_delayed_work(&muic_data->dcd_recheck, 0);
#endif
	}

	if ((read_val[READ_VAL_CHG_TYPE] & DEV_TYPE_CHG_TYPE) &&
		(*new_dev == ATTACHED_DEV_UNKNOWN_MUIC)) {
		*intr = MUIC_INTR_ATTACH;
		*new_dev = ATTACHED_DEV_TA_MUIC;
		muic_data->afc_check = false;
		pr_info("CHG_TYPE DETECTED\n");
#ifdef	CONFIG_MUIC_MANAGER
		muic_if->is_dcdtmr_intr = true;
		schedule_delayed_work(&muic_data->dcd_recheck, 0);
#endif
	}
}

static void update_jig_state(struct muic_platform_data *pdata)
{
	int jig_state;

	switch (pdata->attached_dev) {
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:     /* VBUS enabled */
	case ATTACHED_DEV_JIG_UART_OFF_VB_OTG_MUIC:    /* for otg test */
	case ATTACHED_DEV_JIG_UART_OFF_VB_FG_MUIC:    /* for fg test */
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:       /* VBUS enabled */
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		jig_state = true;
		break;
	default:
		jig_state = false;
		break;
	}
	pr_info("%s jig_state : %d\n", __func__, jig_state);

	pdata->jig_uart_cb(jig_state);
}

static void s2mu004_muic_detect_dev(struct s2mu004_muic_data *muic_data)
{
	muic_attached_dev_t new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
	int intr = MUIC_INTR_DETACH;
	int vbvolt = 0, vmid = 0, adc = 0;
	int read_val[READ_VAL_MAX_NUM] = {0, };
	int ret;
#if defined(CONFIG_MUIC_MANAGER)
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;
#endif
	usleep_range(15000, 18000);
	s2mu004_muic_get_detect_info(muic_data, read_val, &adc, &vbvolt, &vmid);

#if defined(CONFIG_CCIC_S2MU004)
	ret = s2mu004_muic_detect_with_ccic(muic_data, adc, vbvolt, &intr, &new_dev);
	switch (ret) {
	case S2MU004_DETECT_SKIP:
		return;
		break;
	case S2MU004_DETECT_JIG:
		goto jig;
		break;
	case S2MU004_DETECT_DONE:
	default:
		break;
	}
#endif
	ret = s2mu004_muic_detect_by_dpdm(muic_data, read_val, adc, vbvolt, &intr, &new_dev);
	if (ret == S2MU004_DETECT_SKIP)
		return;

#if defined(CONFIG_MUIC_MANAGER)
	if (!(muic_if->opmode & OPMODE_CCIC)) {
		s2mu004_muic_detect_jig_dev_type(muic_data,	read_val, vbvolt, &intr, &new_dev);
	}
#endif

#ifdef CONFIG_MUIC_S2MU004_NON_USB_C_TYPE
	s2mu004_muic_detect_jig_by_adc(muic_data, read_val, adc, vbvolt, &vmid, &intr, &new_dev);
#endif

	s2mu004_muic_detect_extra_cable(muic_data, read_val, vbvolt, &intr, &new_dev);

#if defined(CONFIG_CCIC_S2MU004)
/* FIXME: for VB on-off case */
jig:
#endif/* CONFIG_CCIC_S2MU004 */
#ifndef CONFIG_CCIC_S2MU004
		muic_data->bcd_rescan_cnt = 0;
#endif

	if (intr == MUIC_INTR_ATTACH) {
#ifdef	CONFIG_MUIC_MANAGER
		muic_manager_set_legacy_dev(muic_if, new_dev);
#endif
		muic_core_handle_attach(muic_data->pdata, new_dev, adc, vbvolt);
	} else {
		if (muic_data->attach_mode == S2MU004_SECOND_ATTACH)
			return;
#ifdef	CONFIG_MUIC_MANAGER
		if (muic_if->opmode & OPMODE_CCIC) {
			if (muic_core_get_ccic_cable_state(muic_data->pdata) && (muic_if->is_ccic_attached == true)) {
				pr_info("[muic] %s, skipped handle detach!\n", __func__);
				return;
			}
		}
#endif
#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC)
		ret = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_AFC_CTRL1);
		if (ret) {
			pr_info("%s, need to AFC detach\n", __func__);
			s2mu004_hv_muic_reset_hvcontrol_reg(muic_data);
		}
#endif
		muic_core_handle_detach(muic_data->pdata);
	}

	if (muic_data->pdata->jig_uart_cb)
		update_jig_state(muic_data->pdata);

#if defined(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle((!!vbvolt) ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
}

#if defined(CONFIG_CCIC_S2MU004)
static int s2mu004_muic_check_irq_exeptions(struct s2mu004_muic_data *muic_data,
	struct muic_interface_t *muic_if, int irq_num, int adc, int vbvolt)
{
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
	if ((irq_num == S2MU004_MUIC_IRQ2_VBUS_OFF) &&
		(muic_if->opmode & OPMODE_CCIC) &&
		!muic_data->jig_state &&
		s2mu004_muic_get_otg_state())
		MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_CHECK_OCP);
#endif

#if !IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
	if (((adc > 0 && adc < ADC_OPEN) || (adc & 0x80))
		&& !muic_data->re_detect && !vbvolt
		&& (muic_if->opmode & OPMODE_CCIC)
		&& ((irq_num == S2MU004_MUIC_IRQ2_ADC_CHANGE)
			|| (irq_num == S2MU004_MUIC_IRQ1_ATTATCH))) {
		adc = s2mu004_muic_water_judge(muic_data);
	}
#endif

#if defined(CONFIG_HICCUP_CHARGER)
	if (!lpcharge && vbvolt &&
		IS_WATER_STATUS(muic_data->water_status)) {
		muic_data->is_hiccup_mode = true;
		s2mu004_muic_com_to_gnd(muic_data);
	}
#endif

	pr_info("%s adc 0x%x, vbvolt : %d, irq_num : %d\n",
		__func__, adc, vbvolt, irq_num);
	adc &= ADC_MASK;
	if (((irq_num == S2MU004_MUIC_IRQ2_ADC_CHANGE) || (irq_num == S2MU004_MUIC_IRQ1_ATTATCH))
			&& !vbvolt && adc != ADC_GND && (muic_if->opmode & OPMODE_CCIC)) {
		pr_info("%s:%d adc : 0x%X, water_status : %d, vbvolt : %d\n",
					__func__, __LINE__, adc, muic_data->water_status, vbvolt);
#if !IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
		if (adc < 0x1F && muic_data->water_status == S2MU004_WATER_MUIC_IDLE) {
			cancel_delayed_work(&muic_data->water_detect_handler);
			schedule_delayed_work(&muic_data->water_detect_handler,
				msecs_to_jiffies(0));
		} else if (adc == 0x1F && IS_WATER_STATUS(muic_data->water_status)) {
			cancel_delayed_work(&muic_data->water_dry_handler);
			schedule_delayed_work(&muic_data->water_dry_handler,
				msecs_to_jiffies(WATER_DET_STABLE_DURATION_MS + 1000));
			msleep(100);
		} else if ((muic_data->water_status == S2MU004_WATER_MUIC_CCIC_DET ||
				muic_data->water_status == S2MU004_WATER_MUIC_CCIC_STABLE ||
				muic_data->water_status == S2MU004_WATER_MUIC_DET) &&
				IS_WATER_ADC(adc)) {
			s2mu004_muic_set_rid_adc_en(muic_data, false);
			pr_info("%s WATER Toggling(audio),, adc : 0x%X\n", __func__, adc);
		}

		return S2MU004_IRQ_SKIP;
	} else if (muic_data->water_status == S2MU004_WATER_MUIC_CCIC_STABLE) {
		if (irq_num == S2MU004_MUIC_IRQ2_VBUS_OFF ||
			irq_num == S2MU004_MUIC_IRQ2_VBUS_ON) {
			MUIC_SEND_NOTI_DETACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
			MUIC_SEND_NOTI_ATTACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
#if defined(CONFIG_VBUS_NOTIFIER)
			vbus_notifier_handle(vbvolt ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
		}
		pr_info("%s WATER DETECT : Wet cable inserted\n", __func__);
		pr_info("%s : skipped by water detected condition\n", __func__);
		return S2MU004_IRQ_SKIP;
#endif
	} else if (irq_num == S2MU004_MUIC_IRQ2_VBUS_OFF) {
		if (muic_data->attach_mode == S2MU004_MUIC_OTG) {
			s2mu004_muic_control_vbus_det(muic_data, false);
			muic_data->otg_state = true;

			if (s2mu004_muic_refresh_adc(muic_data) == ADC_JIG_UART_OFF) {
				pr_info("%s OTG - VBUS off case\n", __func__);
				muic_data->attach_mode = S2MU004_NONE_CABLE;
			}
		} else
			muic_data->attach_mode = S2MU004_MUIC_DETACH;

		/* FIXME: for VB on-off case */
		/* muic_data->jig_state = false; */

	} else if (irq_num == S2MU004_MUIC_IRQ1_DETACH &&
			muic_data->attach_mode == S2MU004_FIRST_ATTACH) {

		muic_data->attach_mode = S2MU004_MUIC_DETACH;
	}
	return S2MU004_IRQ_CHECK_DONE;
}
#endif

static irqreturn_t s2mu004_muic_irq_thread(int irq, void *data)
{
	struct s2mu004_muic_data *muic_data = data;
	struct irq_desc *desc = irq_to_desc(irq);
#if defined(CONFIG_CCIC_S2MU004)
	struct muic_interface_t *muic_if = muic_data->if_data;
	struct i2c_client *i2c = muic_data->i2c;
	int ret;
	int irq_num = irq - muic_data->s2mu004_dev->irq_base;
	int vbvolt, adc;
#endif

	pr_info("%s %s start\n",
		desc ? desc->action->name : "-1",
		mode_to_str(muic_data->attach_mode));

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

#if defined(CONFIG_CCIC_S2MU004)
	/* NONE_CABLE    : default muic mode that cable is empty
	*   DETACH        : MUIC real DETACH occur
	*   SECOND ATTACH : there is vbus, so re-detect dp, dm for TA & USB
	*   FRIST ATTACH  : there is cable but no vbus
	*   OTG		 : there is otg cable
	*/

	/* divide timing that call the detect_dev() */
	/* when Vbus off, force rid to enable */
	vbvolt = s2mu004_muic_get_vbus_state(muic_data);
	adc = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_ADC);

#if defined(CONFIG_VBUS_NOTIFIER)
	if (irq_num == S2MU004_MUIC_IRQ2_VBUS_OFF)
		vbus_notifier_handle(STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */

	ret = s2mu004_muic_check_irq_exeptions(muic_data, muic_if, irq_num, adc, vbvolt);
	if (ret == S2MU004_IRQ_SKIP)
		goto EOH;

	pr_info("%s attach_mode : %d, irq_num: %d\n",
		__func__, muic_data->attach_mode, irq_num);

#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
	if (irq_num == S2MU004_MUIC_IRQ2_VBUS_ON) {
		cancel_delayed_work(&muic_data->bad_cable_checker);
		schedule_delayed_work(&muic_data->bad_cable_checker,
			msecs_to_jiffies(1500));
	}
#endif
	if ((muic_data->attach_mode == S2MU004_NONE_CABLE) ||
		((muic_data->attach_mode == S2MU004_MUIC_DETACH) &&
		!((vbvolt) && (irq_num == S2MU004_MUIC_IRQ2_VBUS_OFF)))	||
		((muic_data->attach_mode == S2MU004_SECOND_ATTACH) &&
				(irq_num == S2MU004_MUIC_IRQ1_ATTATCH) &&
						!muic_data->jig_state) ||
		((muic_data->attach_mode == S2MU004_FIRST_ATTACH) &&
				(irq_num == S2MU004_MUIC_IRQ2_VBUS_ON)) ||
		((muic_data->attach_mode == S2MU004_MUIC_OTG) &&
				(irq_num == S2MU004_MUIC_IRQ2_VBUS_ON)))
		s2mu004_muic_detect_dev(muic_data);
EOH:
#else
	s2mu004_muic_detect_dev(muic_data);
#endif

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	pr_info("%s %s done\n",
		dev_to_str(muic_data->pdata->attached_dev),
		mode_to_str(muic_data->attach_mode));

	return IRQ_HANDLED;
}

#if IS_ENABLED(CONFIG_MUIC_SUPPORT_CCIC)
static void s2mu004_muic_put_dry_chk_time(struct s2mu004_muic_data *muic_data)
{
	struct timeval time;

	do_gettimeofday(&time);
	pr_info("%s Dry check time : %ld\n", __func__, (long)time.tv_sec);
	muic_data->dry_chk_time = (long)time.tv_sec;
}

static void s2mu004_muic_set_water_adc_ldo_wa(struct s2mu004_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c = muic_data->i2c;

	pr_info("%s en : (%d)\n", __func__, (int)en);
	if (en) {
		/* W/A apply */
		s2mu004_i2c_update_bit(i2c,
				S2MU004_REG_LDOADC_VSETH, LDOADC_VSETH_MASK, 0, LDOADC_VSET_1_2V);
		usleep_range(WATER_TOGGLE_WA_DURATION_US, WATER_TOGGLE_WA_DURATION_US + 1000);
		s2mu004_i2c_update_bit(i2c,
				S2MU004_REG_LDOADC_VSETH, LDOADC_VSETH_MASK, 0, LDOADC_VSET_1_4V);
	} else {
		/* W/A unapply */
		s2mu004_i2c_update_bit(i2c,
				S2MU004_REG_LDOADC_VSETH, LDOADC_VSETH_MASK, 0, LDOADC_VSET_2_7V);
	}
}

#if !IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
static int s2mu004_muic_water_judge(struct s2mu004_muic_data *muic_data)
{
	int i, adc_recheck = 0;

	pr_info("%s : enter\n", __func__);
	for (i = 0; i < WATER_DET_RETRY_CNT; i++) {
		adc_recheck = s2mu004_muic_recheck_adc(muic_data);
		if (adc_recheck == ADC_GND) {
			pr_info("%s : NOT WATER\n", __func__);
			return adc_recheck;
		}
		if (s2mu004_muic_get_vbus_state(muic_data)) {
			pr_info("%s : vbus while detecting\n", __func__);
			return ADC_GND;
		}

		pr_info("%s : %d st try : adc(0x%x)\n", __func__, i, adc_recheck);
	}

	if (adc_recheck != ADC_OPEN)
		muic_data->re_detect = 1;
	else
		muic_data->re_detect = 0;

	return adc_recheck;
}
#endif

static void s2mu004_muic_water_detect_handler(struct work_struct *work)
{
	struct s2mu004_muic_data *muic_data =
		container_of(work, struct s2mu004_muic_data, water_detect_handler.work);
	int wait_ret = 0, adc = 0;
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	int vbvolt = 0;
#endif

	mutex_lock(&muic_data->water_det_mutex);
	wake_lock(&muic_data->water_wake_lock);
	if (muic_data->water_status != S2MU004_WATER_MUIC_IDLE) {
		pr_info("%s %d exit detect, due to status mismatch\n", __func__, __LINE__);
		goto EXIT_DETECT;
	}

	pr_info("%s\n", __func__);

	s2mu004_muic_set_rid_adc_en(muic_data, false);
	muic_data->water_status = S2MU004_WATER_MUIC_DET;
	MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_CHK_WATER_REQ);

	wait_ret = wait_event_interruptible_timeout(muic_data->wait,
		muic_data->water_status >= S2MU004_WATER_MUIC_CCIC_DET,
		msecs_to_jiffies(WATER_CCIC_WAIT_DURATION_MS));

	if ((wait_ret < 0) || (!wait_ret)) {
		pr_err("%s wait_q abnormal, status : %d\n", __func__, muic_data->water_status);
		muic_data->water_status = S2MU004_WATER_MUIC_IDLE;
		muic_data->re_detect = 0;
		s2mu004_muic_set_water_adc_ldo_wa(muic_data, false);
		s2mu004_muic_set_rid_adc_en(muic_data, true);
	} else {
		if (muic_data->water_status == S2MU004_WATER_MUIC_CCIC_DET) {
			pr_info("%s: WATER DETECT!!!\n", __func__);
			muic_data->dry_cnt = 0;
			muic_data->dry_duration_sec = WATER_DRY_RETRY_INTERVAL_SEC;
			MUIC_SEND_NOTI_ATTACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
			s2mu004_i2c_update_bit(muic_data->i2c,
					0xD5,
					LDOADC_VSETH_WAKE_HYS_MASK,
					LDOADC_VSETH_WAKE_HYS_SHIFT, 0x1);
			s2mu004_muic_set_water_adc_ldo_wa(muic_data, true);
			msleep(100);
			adc = s2mu004_muic_recheck_adc(muic_data);
			msleep(2000);
			muic_data->water_status = S2MU004_WATER_MUIC_CCIC_STABLE;
			muic_data->water_dry_status = S2MU004_WATER_DRY_MUIC_IDLE;
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
			vbvolt = s2mu004_muic_get_vbus_state(muic_data);
			vbus_notifier_handle(vbvolt ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
			s2mu004_muic_put_dry_chk_time(muic_data);
			cancel_delayed_work(&muic_data->water_dry_handler);
			schedule_delayed_work(&muic_data->water_dry_handler,
				msecs_to_jiffies(1800000));
			pr_info("%s %d WATER DETECT stabled adc : 0x%X\n", __func__, __LINE__, adc);
		} else if (muic_data->water_status == S2MU004_WATER_MUIC_CCIC_INVALID) {
			pr_info("%s Not Water From CCIC.\n", __func__);
			muic_data->water_status = S2MU004_WATER_MUIC_IDLE;
			muic_data->re_detect = 0;
			s2mu004_muic_set_water_adc_ldo_wa(muic_data, false);
			s2mu004_muic_set_rid_adc_en(muic_data, true);
		}
	}
EXIT_DETECT:
	wake_unlock(&muic_data->water_wake_lock);
	mutex_unlock(&muic_data->water_det_mutex);
}

static void s2mu004_muic_water_dry_handler(struct work_struct *work)
{
	struct s2mu004_muic_data *muic_data =
		container_of(work, struct s2mu004_muic_data, water_dry_handler.work);
	int adc, i, wait_ret = 0;

	mutex_lock(&muic_data->water_dry_mutex);
	wake_lock(&muic_data->water_dry_wake_lock);

	if (muic_data->water_status != S2MU004_WATER_MUIC_CCIC_STABLE) {
		pr_info("%s Invalid status for Dry check\n", __func__);
		goto EXIT_DRY_STATE;
	}

	pr_info("%s Dry check start\n", __func__);
	s2mu004_muic_put_dry_chk_time(muic_data);
	s2mu004_muic_set_rid_int_mask_en(muic_data, true);
	s2mu004_muic_set_water_adc_ldo_wa(muic_data, false);

	if (muic_data->dry_cnt++ > 5) {
		muic_data->dry_duration_sec = 1800;
		pr_info("%s Dry check cnt : %d\n", __func__, muic_data->dry_cnt);
	}

	for (i = 0; i < WATER_DET_RETRY_CNT; i++) {
		adc = s2mu004_muic_recheck_adc(muic_data);
		pr_info("%s, %d th try, adc : 0x%X\n", __func__, i, (char)adc);
		if (adc < 0x1F) {
			pr_info("%s WATER IS NOT DRIED YET!!!\n", __func__);
			s2mu004_muic_set_rid_adc_en(muic_data, false);
			cancel_delayed_work(&muic_data->water_dry_handler);
			schedule_delayed_work(&muic_data->water_dry_handler,
				msecs_to_jiffies(1800000));
			msleep(1000);
			goto EXIT_DRY;
		}
	}
	muic_data->water_dry_status = S2MU004_WATER_DRY_MUIC_DET;
	MUIC_SEND_NOTI_TO_CCIC_ATTACH(ATTACHED_DEV_CHK_WATER_DRY_REQ);
	wait_ret = wait_event_interruptible_timeout(muic_data->wait,
		muic_data->water_dry_status >= S2MU004_WATER_DRY_MUIC_CCIC_DET,
		msecs_to_jiffies(WATER_CCIC_WAIT_DURATION_MS));

	if ((wait_ret < 0) || (!wait_ret)
			|| muic_data->water_dry_status == S2MU004_WATER_DRY_MUIC_CCIC_INVALID) {
		pr_err("%s wait_q abnormal, status : %d\n",
			__func__, muic_data->water_dry_status);
		s2mu004_muic_set_rid_adc_en(muic_data, false);
		muic_data->water_dry_status = S2MU004_WATER_DRY_MUIC_IDLE;
		s2mu004_muic_set_water_adc_ldo_wa(muic_data, true);
		msleep(1000);
		muic_data->water_status = S2MU004_WATER_MUIC_CCIC_STABLE;
		cancel_delayed_work(&muic_data->water_dry_handler);
		schedule_delayed_work(&muic_data->water_dry_handler,
			msecs_to_jiffies(WATER_DRY_RETRY_INTERVAL_MS));
	} else if (muic_data->water_dry_status == S2MU004_WATER_DRY_MUIC_CCIC_DET) {
		pr_info("%s WATER DRIED!!!\n", __func__);
		s2mu004_muic_set_water_adc_ldo_wa(muic_data, false);
		msleep(500);
		MUIC_SEND_NOTI_DETACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
		MUIC_SEND_NOTI_TO_CCIC_DETACH(ATTACHED_DEV_UNDEFINED_RANGE_MUIC);
		muic_data->attach_mode = S2MU004_NONE_CABLE;
		muic_data->water_status = S2MU004_WATER_MUIC_IDLE;
		muic_data->water_dry_status = S2MU004_WATER_DRY_MUIC_IDLE;
		muic_data->re_detect = 0;
#if defined(CONFIG_HICCUP_CHARGER)
		muic_data->is_hiccup_mode = false;
#endif
		muic_data->dry_duration_sec = WATER_DRY_RETRY_INTERVAL_SEC;
		muic_data->dry_cnt = 0;
	}
EXIT_DRY:
	pr_info("%s %d Exit DRY handler!!!\n", __func__, __LINE__);
	s2mu004_muic_set_rid_int_mask_en(muic_data, false);
EXIT_DRY_STATE:
	wake_unlock(&muic_data->water_dry_wake_lock);
	mutex_unlock(&muic_data->water_dry_mutex);
}

static void s2mu004_muic_sleep_dry_checker(struct work_struct *work)
{
	struct s2mu004_muic_data *muic_data =
		container_of(work, struct s2mu004_muic_data, sleep_dry_checker.work);
	struct timeval time;
	long duration;

	if (muic_data->water_status == S2MU004_WATER_MUIC_CCIC_STABLE && muic_data->lcd_on) {
		if (!s2mu004_muic_get_vbus_state(muic_data)) {
			do_gettimeofday(&time);
			duration = (long)time.tv_sec - muic_data->dry_chk_time;
			pr_info("%s dry check duration : (%ld)\n", __func__, duration);
			if (duration > muic_data->dry_duration_sec || duration < 0) {
				cancel_delayed_work(&muic_data->water_dry_handler);
				schedule_delayed_work(&muic_data->water_dry_handler, 0);
			}
		}
	}
}

#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
static void s2mu004_muic_bad_cable_checker(struct work_struct *work)
{
	struct s2mu004_muic_data *muic_data =
		container_of(work, struct s2mu004_muic_data, bad_cable_checker.work);
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s entered\n", __func__);

	if (!s2mu004_muic_get_vbus_state(muic_data)) {
		pr_info("%s vbus detached while checking.\n", __func__);
		return;
	}

	if (!MUIC_IS_ATTACHED(muic_pdata->attached_dev)) {
		pr_info("%s detected dev(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
		muic_manager_set_legacy_dev(muic_pdata->muic_if, ATTACHED_DEV_TIMEOUT_OPEN_MUIC);
#endif
		muic_core_handle_attach(muic_pdata, ATTACHED_DEV_TIMEOUT_OPEN_MUIC,
					muic_pdata->adc, muic_pdata->vbvolt);
	}
}
#endif
#endif

#ifdef	CONFIG_CCIC_S2MU004
static void s2mu004_muic_incomplete_chk_handler(struct work_struct *work)
{
	struct s2mu004_muic_data *muic_data =
		container_of(work, struct s2mu004_muic_data, incomplete_check.work);

	pr_info("%s, is_dcd_recheck : %d\n", __func__, (int)muic_data->is_dcd_recheck);

	if (muic_data->is_dcd_recheck)
		s2mu004_muic_dcd_rescan(muic_data);
}
#endif

static void s2mu004_muic_dcd_recheck(struct work_struct *work)
{
	struct s2mu004_muic_data *muic_data =
		container_of(work, struct s2mu004_muic_data, dcd_recheck.work);

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
	struct muic_interface_t *muic_if = muic_data->if_data;

	pr_info("%s\n", __func__);

	muic_manager_dcd_rescan(muic_if);
#else
	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	pr_info("%s\n", __func__);

	s2mu004_muic_dcd_rescan(muic_data);
	s2mu004_muic_detect_dev(muic_data);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);
#endif /* CONFIG_MUIC_MANAGER */
}

void muic_disable_otg_detect(void)
{

	pr_info("%s\n", __func__);

	s2mu004_muic_control_vbus_det(static_data, S2MU004_DISABLE);
}

void s2mu004_muic_dcd_rescan(struct s2mu004_muic_data *muic_data)
{
	int ret = 0;
	int reg_val = 0;
	struct i2c_client *i2c = muic_data->i2c;

	mutex_lock(&muic_data->switch_mutex);
	pr_info("%s call\n", __func__);

	/* muic mux switch open */
	reg_val = s2mu004_i2c_read_byte(i2c, S2MU004_REG_MUIC_SW_CTRL);
	ret = s2mu004_muic_com_to_open(muic_data);
	if (ret < 0)
		pr_err("%s, fail to open mansw\n", __func__);

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
	s2mu004_muic_control_rid_adc(muic_data, S2MU004_ENABLE);
	msleep(50);
	s2mu004_muic_control_rid_adc(muic_data, S2MU004_DISABLE);
	msleep(100);
	s2mu004_muic_bcd_rescan(muic_data);
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
	msleep(650);
#else
	msleep(50);
#endif
#else
	s2mu004_muic_bcd_rescan(muic_data);
	msleep(190);
#endif /* CONFIG_MUIC_MANAGER */

	/* restore muic mux switch */
	ret = s2mu004_i2c_guaranteed_wbyte(i2c,	S2MU004_REG_MUIC_SW_CTRL, reg_val);
	if (ret < 0)
		pr_err("%s err write MANSW(0x%x)\n", __func__, reg_val);

	mutex_unlock(&muic_data->switch_mutex);
}

static int s2mu004_init_rev_info(struct s2mu004_muic_data *muic_data)
{
	u8 dev_id;
	int ret = 0;

	dev_id = s2mu004_i2c_read_byte(muic_data->i2c, S2MU004_REG_REV_ID);
	if (dev_id < 0) {
		pr_err("%s(%d)\n", __func__, dev_id);
		ret = -ENODEV;
	} else {
		muic_data->muic_vendor = 0x05;
		muic_data->muic_version = (dev_id & 0x0F);
		muic_data->ic_rev_id = (dev_id & 0xF0) >> 4;
		pr_info("%s : vendor=0x%x, ver=0x%x, dev_id=0x%x\n rev id =0x%x\n",
			__func__, muic_data->muic_vendor, muic_data->muic_version,
			dev_id, muic_data->ic_rev_id);
	}
	return ret;
}

static int s2mu004_muic_irq_init(struct s2mu004_muic_data *muic_data)
{
	int ret = 0;

	if (muic_data->mfd_pdata && (muic_data->mfd_pdata->irq_base > 0)) {
		int irq_base = muic_data->mfd_pdata->irq_base;

		/* request MUIC IRQ */
		muic_data->irq_attach = irq_base + S2MU004_MUIC_IRQ1_ATTATCH;
		REQUEST_IRQ(muic_data->irq_attach, muic_data, "muic-attach");

		muic_data->irq_detach = irq_base + S2MU004_MUIC_IRQ1_DETACH;
		REQUEST_IRQ(muic_data->irq_detach, muic_data, "muic-detach");

		muic_data->irq_rid_chg = irq_base + S2MU004_MUIC_IRQ1_RID_CHG;
		REQUEST_IRQ(muic_data->irq_rid_chg, muic_data, "muic-rid_chg");

		muic_data->irq_vbus_on = irq_base + S2MU004_MUIC_IRQ2_VBUS_ON;
		REQUEST_IRQ(muic_data->irq_vbus_on, muic_data, "muic-vbus_on");

		muic_data->irq_rsvd_attach = irq_base + S2MU004_MUIC_IRQ2_RSVD_ATTACH;
		REQUEST_IRQ(muic_data->irq_rsvd_attach, muic_data, "muic-rsvd_attach");

		muic_data->irq_adc_change = irq_base + S2MU004_MUIC_IRQ2_ADC_CHANGE;
		REQUEST_IRQ(muic_data->irq_adc_change, muic_data, "muic-adc_change");

		muic_data->irq_av_charge = irq_base + S2MU004_MUIC_IRQ2_AV_CHARGE;
		REQUEST_IRQ(muic_data->irq_av_charge, muic_data, "muic-av_charge");

		muic_data->irq_vbus_off = irq_base + S2MU004_MUIC_IRQ2_VBUS_OFF;
		REQUEST_IRQ(muic_data->irq_vbus_off, muic_data, "muic-vbus_off");

	}

	pr_info("%s muic-attach(%d), muic-detach(%d), muic-rid_chg(%d), muic-vbus_on(%d)",
		__func__, muic_data->irq_attach, muic_data->irq_detach, muic_data->irq_rid_chg,
			muic_data->irq_vbus_on);
	pr_info("muic-rsvd_attach(%d), muic-adc_change(%d), muic-av_charge(%d), muic-vbus_off(%d)\n",
		muic_data->irq_rsvd_attach, muic_data->irq_adc_change, muic_data->irq_av_charge, muic_data->irq_vbus_off);

	return ret;
}

static void s2mu004_muic_free_irqs(struct s2mu004_muic_data *muic_data)
{
	pr_info("%s\n", __func__);

	/* free MUIC IRQ */
	FREE_IRQ(muic_data->irq_attach, muic_data, "muic-attach");
	FREE_IRQ(muic_data->irq_detach, muic_data, "muic-detach");
	FREE_IRQ(muic_data->irq_rid_chg, muic_data, "muic-rid_chg");
	FREE_IRQ(muic_data->irq_vbus_on, muic_data, "muic-vbus_on");
	FREE_IRQ(muic_data->irq_rsvd_attach, muic_data, "muic-rsvd_attach");
	FREE_IRQ(muic_data->irq_adc_change, muic_data, "muic-adc_change");
	FREE_IRQ(muic_data->irq_av_charge, muic_data, "muic-av_charge");
	FREE_IRQ(muic_data->irq_vbus_off, muic_data, "muic-vbus_off");
}

#if IS_ENABLED(CONFIG_OF)
static int of_s2mu004_muic_dt(struct device *dev,
	struct s2mu004_muic_data *muic_data)
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

static void s2mu004_muic_init_drvdata(struct s2mu004_muic_data *muic_data,
	struct s2mu004_dev *s2mu004, struct platform_device *pdev,
	struct s2mu004_platform_data *mfd_pdata)
{
	/* save platfom data for gpio control functions */
	muic_data->s2mu004_dev = s2mu004;
	muic_data->dev = &pdev->dev;
	muic_data->i2c = s2mu004->i2c;
	muic_data->mfd_pdata = mfd_pdata;
	muic_data->temp = 0;
	muic_data->attach_mode = S2MU004_NONE_CABLE;
	muic_data->jig_state = false;
	muic_data->re_detect = 0;
	muic_data->afc_check = false;
	muic_data->otg_state = false;
	muic_data->is_dcd_recheck = false;
#if defined(CONFIG_CCIC_S2MU004)
	muic_data->water_status = S2MU004_WATER_MUIC_IDLE;
	muic_data->water_dry_status = S2MU004_WATER_DRY_MUIC_IDLE;
	muic_data->dry_chk_time = 0;
	muic_data->dry_cnt = 0;
	muic_data->dry_duration_sec = WATER_DRY_RETRY_INTERVAL_SEC;
#endif
}

#if IS_ENABLED(CONFIG_MUIC_MANAGER)
static void s2mu004_muic_init_interface(struct s2mu004_muic_data *muic_data,
	struct muic_interface_t *muic_if)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s, muic_if : 0x%p, muic_data : 0x%p\n",
				__func__, muic_if, muic_data);

	muic_if->muic_data = (void *)muic_data;
	muic_if->set_com_to_open_with_vbus = s2mu004_if_com_to_open_with_vbus;
	muic_if->set_com_to_open = s2mu004_if_com_to_open;
	muic_if->set_switch_to_usb = s2mu004_if_switch_to_usb;
	muic_if->set_switch_to_uart = s2mu004_if_switch_to_uart;
	muic_if->get_vbus = s2mu004_if_get_vbus;
	muic_if->set_jig_state = s2mu004_if_set_jig_state;
	muic_if->set_cable_state = s2mu004_if_set_cable_state;
	muic_if->set_otg_detect_en = s2mu004_if_set_otg_detect_en;
	muic_if->set_dcd_rescan = s2mu004_if_dcd_rescan;
	muic_if->set_jig_ctrl_on = s2mu004_if_jig_on;
	muic_if->set_com_to_audio = s2mu004_if_com_to_audio;
	muic_if->set_com_to_otg = s2mu004_if_com_to_otg;
	muic_if->get_adc = s2mu004_if_get_adc;
	muic_if->set_gpio_uart_sel = s2mu004_if_set_gpio_uart_sel;
	muic_if->bcd_rescan = s2mu004_if_bcd_rescan;
	muic_if->control_rid_adc = s2mu004_if_control_rid_adc;
#if defined(CONFIG_HV_MUIC_S2MU004_AFC)
	muic_if->set_afc_reset = s2mu004_if_set_afc_reset;
	muic_if->check_id_err = s2mu004_if_check_id_err;
	muic_if->reset_hvcontrol_reg = s2mu004_if_reset_hvcontrol_reg;
	muic_if->check_afc_ready = s2mu004_if_check_afc_ready;
	muic_if->reset_afc_register = s2mu004_if_reset_afc_register;
	muic_if->set_afc_ready = s2mu004_if_set_afc_ready;
#endif
	muic_if->check_usb_killer = s2mu004_if_check_usb_killer;
#if defined(CONFIG_CCIC_S2MU004)
	muic_if->set_water_detect = s2mu004_if_set_water_det;
#endif
#ifndef CONFIG_SEC_FACTORY
#if defined(CONFIG_CCIC_S2MU004)
	muic_if->set_water_detect_from_boot = s2mu004_if_set_water_det_from_boot;
#endif
#endif
	muic_data->if_data = muic_if;
	muic_pdata->muic_if = muic_if;
}
#endif

#if IS_ENABLED(CONFIG_MUIC_SUPPORT_CCIC)
static int muic_fb_notifier_event(struct notifier_block *this,
        unsigned long val, void *v)
{
	struct fb_event *evdata = v;
	struct s2mu004_muic_data *muic_data =
		container_of(this, struct s2mu004_muic_data, fb_notifier);
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
	} else
		muic_data->lcd_on = false;

	return NOTIFY_DONE;
}
#endif

static int s2mu004_muic_probe(struct platform_device *pdev)
{
	struct s2mu004_dev *s2mu004 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu004_platform_data *mfd_pdata = dev_get_platdata(s2mu004->dev);
	struct s2mu004_muic_data *muic_data;
	struct muic_platform_data *muic_pdata;
	int ret = 0;
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
	struct muic_interface_t *muic_if;
#endif

	pr_info("%s start\n", __func__);

	muic_data = devm_kzalloc(&pdev->dev, sizeof(*muic_data), GFP_KERNEL);
	if (unlikely(!muic_data)) {
		pr_err("%s out of memory\n", __func__);
		ret = -ENOMEM;
		goto err_return;
	}
	static_data = muic_data;

	if (unlikely(!mfd_pdata)) {
		pr_err("%s failed to get s2mu004 mfd platform data\n",
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
	if (!muic_if) {
		pr_err("%s failed to init muic manager, ret : 0x%X\n",
			__func__, ret);
		goto err_init_manager;
	}

	s2mu004_muic_init_interface(muic_data, muic_if);
#endif

	s2mu004_muic_init_drvdata(muic_data, s2mu004, pdev, mfd_pdata);

#if IS_ENABLED(CONFIG_MUIC_SUPPORT_CCIC)
	muic_data->fb_notifier.priority = -1;
       muic_data->fb_notifier.notifier_call = muic_fb_notifier_event;
       fb_register_client(&muic_data->fb_notifier);
#endif

#if defined(CONFIG_OF)
	ret = of_s2mu004_muic_dt(&pdev->dev, muic_data);
	if (ret < 0)
		pr_err("no muic dt! ret[%d]\n", ret);
#endif /* CONFIG_OF */

	mutex_init(&muic_data->muic_mutex);
	mutex_init(&muic_data->switch_mutex);
	wake_lock_init(&muic_data->wake_lock, WAKE_LOCK_SUSPEND, "muic_wake");
#ifdef CONFIG_CCIC_S2MU004
	mutex_init(&muic_data->water_det_mutex);
	mutex_init(&muic_data->water_dry_mutex);
	wake_lock_init(&muic_data->water_wake_lock,
		WAKE_LOCK_SUSPEND, "muic_water_wake");
	wake_lock_init(&muic_data->water_dry_wake_lock,
		WAKE_LOCK_SUSPEND, "muic_water_dry_wake");
#endif
	init_waitqueue_head(&muic_data->wait);
	platform_set_drvdata(pdev, muic_data);

	if (muic_data->pdata->init_gpio_cb)
		ret = muic_data->pdata->init_gpio_cb(muic_data->pdata, get_switch_sel());
	if (ret) {
		pr_err("%s failed to init gpio(%d)\n", __func__, ret);
		goto fail_init_gpio;
	}

	ret = s2mu004_muic_init_sysfs(muic_data);
	if (ret) {
		pr_err("failed to create sysfs\n");
		goto fail_init_sysfs;
	}

	ret = s2mu004_init_rev_info(muic_data);
	if (ret) {
		pr_err("failed to init muic(%d)\n", ret);
		goto fail;
	}

	ret = s2mu004_muic_reg_init(muic_data);
	if (ret) {
		pr_err("failed to init muic(%d)\n", ret);
		goto fail;
	}

#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC)
	/* initial hv cable detection */
	if (muic_data->is_afc_muic_ready)
		s2mu004_hv_muic_init_detect(muic_data);

	s2mu004_hv_muic_initialize(muic_data);
#endif /* CONFIG_HV_MUIC_S2MU004_AFC */

#if !defined(CONFIG_SEC_FACTORY) && defined(CONFIG_MUIC_SUPPORT_TYPEB)
	muic_pdata->is_rustproof = muic_pdata->rustproof_on;
#endif
	if (muic_pdata->is_rustproof) {
		pr_err("%s rustproof is enabled\n", __func__);
		s2mu004_muic_com_to_open_with_vbus(muic_data);
	}

#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC)
	if (get_afc_mode() == CH_MODE_AFC_DISABLE_VAL) {
		pr_info("AFC mode disabled\n");
		muic_data->pdata->afc_disable = true;
	} else {
		pr_info("AFC mode enabled\n");
		muic_data->pdata->afc_disable = false;
	}
#endif /* CONFIG_HV_MUIC_S2MU004_AFC */

	ret = s2mu004_muic_irq_init(muic_data);
	if (ret) {
		pr_err("%s failed to init irq(%d)\n", __func__, ret);
		goto fail_init_irq;
	}

#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC)
	ret = s2mu004_afc_muic_irq_init(muic_data);
	if (ret < 0) {
		pr_err("%s Failed to initialize HV MUIC irq:%d\n",
				__func__, ret);
		s2mu004_hv_muic_free_irqs(muic_data);
	}
#endif /* CONFIG_HV_MUIC_S2MU004_AFC */

	/* initial cable detection */
	s2mu004_muic_set_int_mask(muic_data, false);
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	s2mu004_muic_init_otg_reg(muic_data);
#endif

#ifdef	CONFIG_CCIC_S2MU004
	INIT_DELAYED_WORK(&muic_data->water_dry_handler, s2mu004_muic_water_dry_handler);
	INIT_DELAYED_WORK(&muic_data->water_detect_handler, s2mu004_muic_water_detect_handler);
	INIT_DELAYED_WORK(&muic_data->incomplete_check, s2mu004_muic_incomplete_chk_handler);
	INIT_DELAYED_WORK(&muic_data->sleep_dry_checker, s2mu004_muic_sleep_dry_checker);
#endif
	INIT_DELAYED_WORK(&muic_data->dcd_recheck, s2mu004_muic_dcd_recheck);
#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
	INIT_DELAYED_WORK(&muic_data->bad_cable_checker,
		s2mu004_muic_bad_cable_checker);
#endif
	s2mu004_muic_irq_thread(-1, muic_data);

#if IS_ENABLED(CONFIG_NONE_WATERPROOF_MODEL)
	if (muic_if->opmode & OPMODE_CCIC) {
		cancel_delayed_work(&muic_data->bad_cable_checker);
		schedule_delayed_work(&muic_data->bad_cable_checker,
			msecs_to_jiffies(3000));
	}
#endif

	if (!s2mu004_muic_get_vbus_state(muic_data)) {
		pr_info("%s : init adc : 0x%X\n", __func__,
			s2mu004_muic_recheck_adc(muic_data));
	}

	return 0;

fail_init_irq:
fail:
#ifdef CONFIG_SEC_SYSFS
	s2mu004_muic_deinit_sysfs(muic_data);
#endif
fail_init_sysfs:
fail_init_gpio:
	mutex_destroy(&muic_data->muic_mutex);
#if IS_ENABLED(CONFIG_MUIC_MANAGER)
err_init_manager:
#endif
err_kfree1:
err_return:
	return ret;
}

/* if need to set s2mu004 pdata */
static const struct of_device_id s2mu004_muic_match_table[] = {
	{ .compatible = "samsung,s2mu004-muic",},
	{},
};

static int s2mu004_muic_remove(struct platform_device *pdev)
{
	struct s2mu004_muic_data *muic_data = platform_get_drvdata(pdev);

	if (muic_data) {
		pr_info("%s\n", __func__);

#ifdef CONFIG_SEC_SYSFS
		s2mu004_muic_deinit_sysfs(muic_data);
#endif
		muic_manager_exit(muic_data->if_data);
		muic_core_exit(muic_data->pdata);

		disable_irq_wake(muic_data->i2c->irq);
		s2mu004_muic_free_irqs(muic_data);
		mutex_destroy(&muic_data->muic_mutex);
		mutex_destroy(&muic_data->switch_mutex);
		mutex_destroy(&muic_data->water_det_mutex);
		mutex_destroy(&muic_data->water_dry_mutex);
		i2c_set_clientdata(muic_data->i2c, NULL);
	}

	return 0;
}

static void s2mu004_muic_shutdown(struct platform_device *pdev)
{
	struct s2mu004_muic_data *muic_data = platform_get_drvdata(pdev);

	pr_info("%s\n", __func__);

	if (!muic_data->i2c) {
		pr_err("%s no muic i2c client\n", __func__);
		return;
	}

#if IS_ENABLED(CONFIG_HV_MUIC_S2MU004_AFC)
	s2mu004_hv_muic_remove(muic_data);
#endif
}

#if IS_ENABLED(CONFIG_PM)
static int s2mu004_muic_suspend(struct device *dev)
{
	struct s2mu004_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	muic_pdata->suspended = true;

#if defined(CONFIG_CCIC_S2MU004)
	if (muic_data->water_status == S2MU004_WATER_MUIC_CCIC_STABLE) {
		cancel_delayed_work(&muic_data->sleep_dry_checker);
	}
#endif

	return 0;
}

static int s2mu004_muic_resume(struct device *dev)
{
	struct s2mu004_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	muic_pdata->suspended = false;

	if (muic_pdata->need_to_noti) {
		if (muic_pdata->attached_dev)
			MUIC_SEND_NOTI_ATTACH(muic_pdata->attached_dev);
		else
			MUIC_SEND_NOTI_DETACH(muic_pdata->attached_dev);
		muic_pdata->need_to_noti = false;
	}

#if defined(CONFIG_CCIC_S2MU004)
	if (muic_data->water_status == S2MU004_WATER_MUIC_CCIC_STABLE) {
		cancel_delayed_work(&muic_data->sleep_dry_checker);
		schedule_delayed_work(&muic_data->sleep_dry_checker,
			msecs_to_jiffies(WATER_WAKEUP_WAIT_DURATION_MS));
	}
#endif

	return 0;
}
#else
#define s2mu004_muic_suspend NULL
#define s2mu004_muic_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(s2mu004_muic_pm_ops, s2mu004_muic_suspend,
			 s2mu004_muic_resume);

static struct platform_driver s2mu004_muic_driver = {
	.probe = s2mu004_muic_probe,
	.remove = s2mu004_muic_remove,
	.shutdown = s2mu004_muic_shutdown,
	.driver = {
		.name = "s2mu004-muic",
		.owner	= THIS_MODULE,
		.of_match_table = s2mu004_muic_match_table,
#if IS_ENABLED(CONFIG_PM)
		.pm = &s2mu004_muic_pm_ops,
#endif
	},
};

static int __init s2mu004_muic_init(void)
{
	return platform_driver_register(&s2mu004_muic_driver);
}
module_init(s2mu004_muic_init);

static void __exit s2mu004_muic_exit(void)
{
	platform_driver_unregister(&s2mu004_muic_driver);
}
module_exit(s2mu004_muic_exit);

MODULE_DESCRIPTION("Samsung S2MU004 Micro USB IC driver");
MODULE_LICENSE("GPL");
