/*
 * driver/misc/s2mu005.c - S2MU005 micro USB switch device driver
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
#include <linux/mfd/samsung/s2mu005.h>
#include <linux/mfd/samsung/s2mu005-private.h>

#if defined(CONFIG_BATTERY_SAMSUNG_V2)
#include "../battery_v2/include/sec_charging_common.h"
#endif

#include <linux/muic/s2mu005-muic.h>
#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
#include <linux/vbus_notifier.h>
#endif /* CONFIG_VBUS_NOTIFIER */
#include "muic-internal.h"
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
#include <soc/samsung/exynos-modem-ctrl.h>
#endif

#define ENUM_STR(x)	{case(x): return #x; }
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
	ENUM_STR(ATTACHED_DEV_FACTORY_UART_MUIC);
	ENUM_STR(ATTACHED_DEV_UNKNOWN_MUIC);
	ENUM_STR(ATTACHED_DEV_NUM);
	default:
		return "invalid";
	}
	return "invalid";
}

static void s2mu005_muic_handle_attach(struct s2mu005_muic_data *muic_data,
			muic_attached_dev_t new_dev, int adc, u8 vbvolt);
static void s2mu005_muic_handle_detach(struct s2mu005_muic_data *muic_data);
static int attach_uart(struct s2mu005_muic_data *muic_data);
static void s2mu005_muic_detect_dev(struct s2mu005_muic_data *muic_data);
#ifndef CONFIG_SEC_FACTORY
static void s2mu005_muic_set_water_wa(struct s2mu005_muic_data *muic_data, bool en);
static int s2mu005_i2c_update_bit(struct i2c_client *i2c,
			u8 reg, u8 mask, u8 shift, u8 value);
#endif

#define DEBUG_MUIC

#if IS_ENABLED(DEBUG_MUIC)
#define MAX_LOG 25
#define READ 0
#define WRITE 1

static u8 s2mu005_log_cnt;
static u8 s2mu005_log[MAX_LOG][3];

static void s2mu005_reg_log(u8 reg, u8 value, u8 rw)
{
	s2mu005_log[s2mu005_log_cnt][0] = reg;
	s2mu005_log[s2mu005_log_cnt][1] = value;
	s2mu005_log[s2mu005_log_cnt][2] = rw;
	s2mu005_log_cnt++;
	if (s2mu005_log_cnt >= MAX_LOG)
		s2mu005_log_cnt = 0;
}
static void s2mu005_print_reg_log(void)
{
	int i;
	u8 reg, value, rw;
	char mesg[256] = "";

	for (i = 0; i < MAX_LOG; i++) {
		reg = s2mu005_log[s2mu005_log_cnt][0];
		value = s2mu005_log[s2mu005_log_cnt][1];
		rw = s2mu005_log[s2mu005_log_cnt][2];
		s2mu005_log_cnt++;

		if (s2mu005_log_cnt >= MAX_LOG)
			s2mu005_log_cnt = 0;
		sprintf(mesg+strlen(mesg), "%x(%x)%x ", reg, value, rw);
	}
	pr_info("%s %s\n", __func__, mesg);
}
void s2mu005_read_reg_dump(struct s2mu005_muic_data *muic, char *mesg)
{
	u8 val;

	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_CTRL1, &val);
	sprintf(mesg+strlen(mesg), "CTRL1:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_SW_CTRL, &val);
	sprintf(mesg+strlen(mesg), "SW_CTRL:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_INT1_MASK, &val);
	sprintf(mesg+strlen(mesg), "IM1:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_INT2_MASK, &val);
	sprintf(mesg+strlen(mesg), "IM2:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_CHG_TYPE, &val);
	sprintf(mesg+strlen(mesg), "CHG_T:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_DEVICE_APPLE, &val);
	sprintf(mesg+strlen(mesg), "APPLE_DT:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_ADC, &val);
	sprintf(mesg+strlen(mesg), "ADC:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_DEVICE_TYPE1, &val);
	sprintf(mesg+strlen(mesg), "DT1:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_DEVICE_TYPE2, &val);
	sprintf(mesg+strlen(mesg), "DT2:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_DEVICE_TYPE3, &val);
	sprintf(mesg+strlen(mesg), "DT3:%x ", val);
}
void s2mu005_print_reg_dump(struct s2mu005_muic_data *muic_data)
{
	char mesg[256] = "";

	s2mu005_read_reg_dump(muic_data, mesg);

	pr_info("%s %s\n", __func__, mesg);
}
#endif

static int s2mu005_i2c_read_byte(struct i2c_client *client, u8 command)
{
	u8 ret;
	int retry = 0;

	s2mu005_read_reg(client, command, &ret);

	while (ret < 0) {
		pr_info("failed to read reg(0x%x) retry(%d)\n", command, retry);
		if (retry > 5) {
			pr_err("%s retry failed!!\n", __func__);
			break;
		}
		msleep(100);
		s2mu005_read_reg(client, command, &ret);
		retry++;
	}

#if IS_ENABLED(DEBUG_MUIC)
	s2mu005_reg_log(command, ret, retry << 1 | READ);
#endif
	return ret;
}

static int s2mu005_i2c_write_byte(struct i2c_client *client,
			u8 command, u8 value)
{
	int ret;
	int retry = 0;
	u8 written = 0;

	ret = s2mu005_write_reg(client, command, value);

	while (ret < 0) {
		pr_info("failed to write(0x%x) retry(%d)\n", command, retry);
		if (retry > 5) {
			pr_err("%s  retry failed!!\n", __func__);
			break;
		}
		msleep(100);
		s2mu005_read_reg(client, command, &written);
		if (written < 0)
			pr_info("failed to read reg(0x%x)\n", command);
		ret = s2mu005_write_reg(client, command, value);
		retry++;
	}
#if IS_ENABLED(DEBUG_MUIC)
	s2mu005_reg_log(command, value, retry << 1 | WRITE);
#endif
	return ret;
}

#ifndef CONFIG_SEC_FACTORY
static int s2mu005_i2c_update_bit(struct i2c_client *i2c,
			u8 reg, u8 mask, u8 shift, u8 value)
{
	int ret;
	u8 val = 0;

	val = s2mu005_i2c_read_byte(i2c, reg);
	val &= ~mask;
	val |= value << shift;
	ret = s2mu005_i2c_write_byte(i2c, reg, val);
	pr_info("%s reg(0x%x) : 0x%x\n", __func__, reg, val);
	if (ret < 0)
		pr_err("failed to write mask(0x%x) ret(%d)\n", mask, ret);

	return ret;
}
#endif

void muic_disable_otg_detect(void)
{
	/* TBD */
}

#if IS_ENABLED(GPIO_DOC_SWITCH)
static int s2mu005_set_gpio_doc_switch(int val)
{
	int gpio = muic_pdata.gpio_doc_switch;
	int val;
	int ret;

	ret = gpio_request(gpio, "GPIO_DOC_SWITCH");
	if (ret) {
		pr_err("failed to gpio_request DOC_SWITCH\n");
		return ret;
	}
	gpio_set_value(gpio, val);
	val = gpio_get_value(gpio);
	gpio_free(gpio);

	pr_info("GPIO_DOC_SWITCH %c\n", val ? 'H' : 'L');

	return 0;
}
#endif /* GPIO_DOC_SWITCH */

#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
static int set_otg_reg(struct s2mu005_muic_data *muic_data, bool on)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 val;
	int ret = 0;

	/* 0x1e : hidden register */
	ret = s2mu005_i2c_read_byte(i2c, 0x1e);
	if (ret < 0)
		pr_err("%s failed to read 0x1e reg(%d)\n", __func__, ret);

	/* Set 0x1e[5:4] bit to 0x11 or 0x01 */
	if (on)
		val = ret | (0x1 << 5);
	else
		val = ret & ~(0x1 << 5);

	if (val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, val, ret);
		ret = s2mu005_i2c_write_byte(i2c, 0x1e, val);
		if (ret < 0)
			pr_err("%s failed to write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n", __func__, val, ret);
		return 0;
	}

	ret = s2mu005_i2c_read_byte(i2c, 0x1e);
	if (ret < 0)
		pr_err("%s failed to read reg 0x1e(%d)\n", __func__, ret);

	return ret;
}

static int init_otg_reg(struct s2mu005_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val;
	int ret = 0;

	/* 0x73 : check EVT0 or EVT1 */
	ret = s2mu005_i2c_read_byte(i2c, 0x73);
	if (ret < 0)
		pr_err("%s failed to read reg 0x73(%d)\n", __func__, ret);

	if ((ret&0xF) > 0)
		return 0;

	/* 0x89 : hidden register */
	ret = s2mu005_i2c_read_byte(i2c, 0x89);
	if (ret < 0)
		pr_err("%s failed to read reg 0x89(%d)\n", __func__, ret);

	/* Set 0x89[1] bit : T_DET_VAL */
	reg_val = ret | (0x1 << 1);
	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu005_i2c_write_byte(i2c, 0x89, reg_val);
		if (ret < 0)
			pr_err("%s failed to write(%d)\n", __func__, ret);
	}

	ret = s2mu005_i2c_read_byte(i2c, 0x89);
	if (ret < 0)
		pr_err("%s failed to read reg 0x89(%d)\n", __func__, ret);

	/* 0x92 : hidden register */
	ret = s2mu005_i2c_read_byte(i2c, 0x92);
	if (ret < 0)
		pr_err("%s failed to read reg 0x92(%d)\n", __func__, ret);

	/* Set 0x92[7] bit : EN_JIG_AP */
	reg_val = ret | (0x1 << 7);
	if (reg_val ^ ret) {
		pr_info("%s  0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu005_i2c_write_byte(i2c, 0x92, reg_val);
		if (ret < 0)
			pr_err("%s failed to write(%d)\n", __func__, ret);
	}

	ret = s2mu005_i2c_read_byte(i2c, 0x92);
	if (ret < 0)
		pr_err("%s failed to read reg 0x92(%d)\n", __func__, ret);

	return ret;
}
#endif

static int set_jig_sw(struct s2mu005_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c = muic_data->i2c;
	bool cur = false;
	int reg_val = 0, ret = 0;

	reg_val = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_SW_CTRL);
	if (reg_val < 0)
		pr_err("%s failed to read 0x%x\n", __func__, reg_val);

	cur = !!(reg_val & MANUAL_SW_JIG_EN);

	if (!muic_data->jigonb_enable)
		en = false;

	if (en != cur) {
		pr_info("%s  0x%x != 0x%x, update\n", __func__,
			reg_val, reg_val);

		if (en)
			reg_val |= (MANUAL_SW_JIG_EN);
		else
			reg_val &= ~(MANUAL_SW_JIG_EN);

		ret = s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_SW_CTRL, reg_val);
		if (ret < 0)
			pr_err("%s failed to write 0x%x\n", __func__, reg_val);
	}

	return ret;
}

static ssize_t uart_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	int ret = 0;

	if (!muic_data->is_rustproof) {
		pr_info("%s UART ENABLE\n", __func__);
		ret = sprintf(buf, "1\n");
	} else {
		pr_info("%s UART DISABLE\n", __func__);
		ret = sprintf(buf, "0\n");
	}

	return ret;
}

static ssize_t uart_en_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	if (!strncmp(buf, "1", 1))
		muic_data->is_rustproof = false;
	else if (!strncmp(buf, "0", 1))
		muic_data->is_rustproof = true;

	pr_info("%s %s\n", __func__, buf);

	return count;
}

static ssize_t uart_sel_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *pdata = muic_data->pdata;
	const char *mode = "UNKNOWN\n";

	switch (pdata->uart_path) {
	case MUIC_PATH_UART_AP:
		mode = "AP\n";
		break;
	case MUIC_PATH_UART_CP:
		mode = "CP\n";
		break;
	default:
		break;
	}

	pr_info("%s %s", __func__, mode);
	return snprintf(buf, strlen(mode) + 1, "%s", mode);
}

static ssize_t uart_sel_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	struct muic_platform_data *pdata = muic_data->pdata;

	if (!strncasecmp(buf, "AP", 2)) {
		pdata->uart_path = MUIC_PATH_UART_AP;
		attach_uart(muic_data);
	} else if (!strncasecmp(buf, "CP", 2)) {
		pdata->uart_path = MUIC_PATH_UART_CP;
		attach_uart(muic_data);
	} else {
		pr_warn("%s invalid value\n", __func__);
	}

	pr_info("%s %s\n", __func__, buf);

	return count;
}

static ssize_t usb_en_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	muic_attached_dev_t new_dev = ATTACHED_DEV_USB_MUIC;

	if (!strncasecmp(buf, "1", 1))
		s2mu005_muic_handle_attach(muic_data, new_dev, 0, 0);
	else if (!strncasecmp(buf, "0", 1))
		s2mu005_muic_handle_detach(muic_data);

	pr_info("%s %s\n", __func__, buf);

	return count;
}

static ssize_t adc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&muic_data->muic_mutex);
	ret = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_ADC);
	mutex_unlock(&muic_data->muic_mutex);
	if (ret < 0) {
		pr_err("%s failed to read adc reg(%d)\n", __func__, ret);
		return sprintf(buf, "UNKNOWN\n");
	}

	return sprintf(buf, "%x\n", (ret & ADC_MASK));
}

static ssize_t usb_state_show(struct device *dev,
	    struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	static unsigned long swtich_slot_time;

	if (printk_timed_ratelimit(&swtich_slot_time, 5000))
		pr_info("%s attached_dev[%d]\n",
			__func__, muic_data->attached_dev);

	switch (muic_data->attached_dev) {
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		return sprintf(buf, "USB_STATE_CONFIGURED\n");
	default:
		break;
	}

	return 0;
}

#if IS_ENABLED(DEBUG_MUIC)
static ssize_t mansw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&muic_data->muic_mutex);
	ret = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_SW_CTRL);
	mutex_unlock(&muic_data->muic_mutex);
	if (ret < 0) {
		pr_err("%s fail to read muic reg\n", __func__);
		return sprintf(buf, "UNKNOWN\n");
	}
	pr_info("%s %d\n", __func__, ret);
	return sprintf(buf, "0x%x\n", ret);
}
static ssize_t interrupt_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	int st1, st2;

	mutex_lock(&muic_data->muic_mutex);
	st1 = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_INT1);
	st2 = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_INT2);
	mutex_unlock(&muic_data->muic_mutex);

	pr_info("%s st1:0x%x st2:0x%x buf%s\n", __func__, st1, st2, buf);
	if (st1 < 0 || st2 < 0) {
		pr_err("%s fail to read muic reg\n", __func__);
		return sprintf(buf, "UNKNOWN\n");
	}
	return sprintf(buf, "st1:0x%x st2:0x%x\n", st1, st2);
}
static ssize_t registers_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	char mesg[256] = "";

	mutex_lock(&muic_data->muic_mutex);
	s2mu005_read_reg_dump(muic_data, mesg);
	mutex_unlock(&muic_data->muic_mutex);
	pr_info("%s %s\n", __func__, mesg);

	return sprintf(buf, "%s\n", mesg);
}
#endif

#if IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
static ssize_t otg_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	int ret;
	u8 val = 0;

	mutex_lock(&muic_data->muic_mutex);
	ret = s2mu005_i2c_read_byte(muic_data->i2c,
		S2MU005_REG_MUIC_INT2_MASK);
	mutex_unlock(&muic_data->muic_mutex);

	if (ret < 0) {
		pr_err("%s  fail to read muic reg\n", __func__);
		return sprintf(buf, "UNKNOWN\n");
	}

	pr_info("%s ret:%d val:%x buf%s\n", __func__, ret, val, buf);

	val &= INT_VBUS_ON_MASK;
	return sprintf(buf, "%x\n", val);
}

static ssize_t otg_test_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s %s\n", __func__, buf);

	/*
	*	The otg_test is set 0 durring the otg test. Not 1 !!!
	*/

	if (!strncmp(buf, "0", 1)) {
		muic_data->is_otg_test = 1;
#if IS_ENABLED(CONFIG_SEC_FACTORY)
		set_otg_reg(muic_data, 1);
#endif
	} else if (!strncmp(buf, "1", 1)) {
		muic_data->is_otg_test = 0;
#if IS_ENABLED(CONFIG_SEC_FACTORY)
		set_otg_reg(muic_data, 0);
#endif
	} else {
		pr_info("%s Wrong command\n", __func__);
		return count;
	}

	return count;
}
#endif

static ssize_t usb_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	return sprintf(buf, "%s attached_dev[%d]\n",
			__func__, muic_data->attached_dev);
}

static ssize_t jig_disable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	if (muic_data->jig_disable)
		return sprintf(buf, "DISABLE\n");
	else
		return sprintf(buf, "ENABLE\n");
}

static ssize_t jig_disable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s : %s\n", __func__, buf);
	if (!strncasecmp(buf, "DISABLE", 7)) {
		muic_data->jig_disable = true;
		set_jig_sw(muic_data, false);
	} else {
		muic_data->jig_disable = false;
		set_jig_sw(muic_data, true);
	}

	return count;
}

#if IS_ENABLED(CONFIG_NEW_FACTORY_UART)
static ssize_t factory_uart_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	if (muic_data->pdata->is_factory_uart)
		return sprintf(buf, "ENABLE\n");
	else
		return sprintf(buf, "DISABLE\n");
}

static ssize_t factory_uart_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s : %s\n", __func__, buf);
	if (!strncasecmp(buf, "ENABLE", 6)) {
		muic_data->pdata->is_factory_uart = true;
		set_jig_sw(muic_data, false);
	} else {
		muic_data->pdata->is_factory_uart = false;
		set_jig_sw(muic_data, true);
	}

	return count;
}
#endif	/* CONFIG_NEW_FACTORY_UART */

static ssize_t attached_dev_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s attached_dev[%d]\n", __func__, muic_data->attached_dev);

	switch (muic_data->attached_dev) {
	case ATTACHED_DEV_NONE_MUIC:
		return sprintf(buf, "No VPS\n");
	case ATTACHED_DEV_USB_MUIC:
		return sprintf(buf, "USB\n");
	case ATTACHED_DEV_CDP_MUIC:
		return sprintf(buf, "CDP\n");
	case ATTACHED_DEV_OTG_MUIC:
		return sprintf(buf, "OTG\n");
	case ATTACHED_DEV_TA_MUIC:
		return sprintf(buf, "TA\n");
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
		return sprintf(buf, "JIG UART OFF\n");
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
		return sprintf(buf, "JIG UART OFF/VB\n");
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
		return sprintf(buf, "JIG UART ON\n");
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:
		return sprintf(buf, "JIG UART ON/VB\n");
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
		return sprintf(buf, "JIG USB OFF\n");
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		return sprintf(buf, "JIG USB ON\n");
	case ATTACHED_DEV_DESKDOCK_MUIC:
	case ATTACHED_DEV_DESKDOCK_VB_MUIC:
		return sprintf(buf, "DESKDOCK\n");
	case ATTACHED_DEV_CHARGING_CABLE_MUIC:
		return sprintf(buf, "PS CABLE\n");
	case ATTACHED_DEV_FACTORY_UART_MUIC:
		return sprintf(buf, "FACTORY UART\n");
	default:
		break;
	}

	return sprintf(buf, "UNKNOWN\n");
}

static ssize_t is_jig_powered_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s  attached_dev[%d]\n", __func__, muic_data->attached_dev);

	switch (muic_data->attached_dev) {
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		return sprintf(buf, "1");
	case ATTACHED_DEV_NONE_MUIC:
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_OTG_MUIC:
	case ATTACHED_DEV_TA_MUIC:
	case ATTACHED_DEV_DESKDOCK_MUIC:
	case ATTACHED_DEV_DESKDOCK_VB_MUIC:
	case ATTACHED_DEV_CHARGING_CABLE_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	default:
		break;
	}

	return sprintf(buf, "0");
}

static ssize_t apo_factory_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);
	const char *mode;

	/* true: Factory mode, false: not Factory mode */
	if (muic_data->is_factory_start)
		mode = "FACTORY_MODE";
	else
		mode = "NOT_FACTORY_MODE";

	pr_info("%s mode[%s]\n", __func__, mode);

	return sprintf(buf, "%s\n", mode);
}

static ssize_t apo_factory_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s %s\n", __func__, buf);

	/* "FACTORY_START": factory mode */
	if (!strncmp(buf, "FACTORY_START", 13)) {
		muic_data->is_factory_start = true;
	} else {
		pr_info("%s Wrong command\n", __func__);
		return count;
	}

	return count;
}

#ifndef CONFIG_SEC_FACTORY
static void s2mu005_muic_set_water_wa(struct s2mu005_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c = muic_data->i2c;

	muic_data->is_water_wa = en;
	pr_info("%s %sable\n", __func__, en ? "en" : "dis");
	if (en) {
		/* W/A apply */
		s2mu005_i2c_update_bit(i2c,
			S2MU005_REG_MUIC_LDOADC_VSETH,
			LDOADC_VSET_MASK, 0,
			LDOADC_VSET_1_2V);
		usleep_range(WATER_TOGGLE_WA_MIN_DURATION_US,
			WATER_TOGGLE_WA_MAX_DURATION_US);
		s2mu005_i2c_update_bit(i2c,
			S2MU005_REG_MUIC_LDOADC_VSETH,
			LDOADC_VSET_MASK, 0,
			LDOADC_VSET_1_5V);
	} else {
		/* W/A unapply */
		s2mu005_i2c_update_bit(i2c,
			S2MU005_REG_MUIC_LDOADC_VSETH,
			LDOADC_VSET_MASK, 0,
			LDOADC_VSET_3V);
	}
}
#endif

static DEVICE_ATTR_RW(uart_en);
static DEVICE_ATTR_RW(uart_sel);
static DEVICE_ATTR_RO(adc);
#if IS_ENABLED(DEBUG_MUIC)
static DEVICE_ATTR_RO(mansw);
static DEVICE_ATTR_RO(dump_registers);
static DEVICE_ATTR_RO(int_status);
#endif
static DEVICE_ATTR_RO(usb_state);
#if IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
static DEVICE_ATTR_RW(otg_test);
#endif
static DEVICE_ATTR_RO(attached_dev);
static DEVICE_ATTR_RO(is_jig_powered);
static DEVICE_ATTR_RW(apo_factory);
static DEVICE_ATTR_RW(usb_en);
static DEVICE_ATTR_RW(jig_disable);
#if IS_ENABLED(CONFIG_NEW_FACTORY_UART)
static DEVICE_ATTR_RW(factory_uart);
#endif

static struct attribute *s2mu005_muic_attributes[] = {
	&dev_attr_uart_en.attr,
	&dev_attr_uart_sel.attr,
	&dev_attr_adc.attr,
#if IS_ENABLED(DEBUG_MUIC)
	&dev_attr_mansw.attr,
	&dev_attr_dump_registers.attr,
	&dev_attr_int_status.attr,
#endif
	&dev_attr_usb_state.attr,
#if IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
	&dev_attr_otg_test.attr,
#endif
	&dev_attr_attached_dev.attr,
	&dev_attr_is_jig_powered.attr,
	&dev_attr_apo_factory.attr,
	&dev_attr_usb_en.attr,
	&dev_attr_jig_disable.attr,
#if IS_ENABLED(CONFIG_NEW_FACTORY_UART)
	&dev_attr_factory_uart.attr,
#endif
	NULL
};

static const struct attribute_group s2mu005_muic_group = {
	.attrs = s2mu005_muic_attributes,
};

static int set_ctrl_reg(struct s2mu005_muic_data *muic_data, int shift, bool on)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val;
	int ret = 0;

	ret = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_CTRL1);
	if (ret < 0)
		pr_err("%s failed to read CTRL(%d)\n", __func__, ret);

	if (on)
		reg_val = ret | (0x1 << shift);
	else
		reg_val = ret & ~(0x1 << shift);

	if (reg_val ^ ret) {
		pr_info("%s 0x%x != 0x%x, update\n", __func__, reg_val, ret);

		ret = s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_CTRL1, reg_val);
		if (ret < 0)
			pr_err("%s failed to write(%d)\n", __func__, ret);
	} else {
		pr_info("%s 0x%x == 0x%x, just return\n",
			__func__, reg_val, ret);
		return 0;
	}

	ret = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_CTRL1);
	if (ret < 0)
		pr_err("%s failed to read CTRL(%d)\n", __func__, ret);
	else
		pr_info("%s after change(0x%x)\n", __func__, ret);

	return ret;
}

static int set_com_sw(struct s2mu005_muic_data *muic_data,
			enum s2mu005_reg_manual_sw_value reg_val)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0;
	int temp = 0;

	/*  --- MANSW [7:5][4:2][1][0] : DM DP RSVD JIG  --- */
	temp = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_SW_CTRL);
	if (temp < 0)
		pr_err("%s failed to read 0x%x\n", __func__, temp);

	if (muic_data->jigonb_enable && !muic_data->jig_disable)
		reg_val |= (MANUAL_SW_JIG_EN);

	if (reg_val != temp) {
		pr_info("%s  0x%x != 0x%x, update\n", __func__,
			reg_val, temp);

		ret = s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_SW_CTRL, reg_val);
		if (ret < 0)
			pr_err("%s failed to write 0x%x\n", __func__, reg_val);
	}

	return ret;
}

static int com_to_open(struct s2mu005_muic_data *muic_data)
{
	enum s2mu005_reg_manual_sw_value val;
	int ret = 0;

	pr_info("%s\n", __func__);
	val = MANSW_OPEN;
	ret = set_com_sw(muic_data, val);
	if (ret)
		pr_err("set_com_sw err(%d)\n", ret);

	return ret;
}

static int com_to_usb(struct s2mu005_muic_data *muic_data)
{
	enum s2mu005_reg_manual_sw_value val;
	int ret = 0;

	pr_info("%s\n", __func__);
	val = MANSW_USB;
	ret = set_com_sw(muic_data, val);
	if (ret)
		pr_err("set_com_usb err(%d)\n", ret);

	return ret;
}

static int com_to_uart(struct s2mu005_muic_data *muic_data)
{
	struct muic_platform_data *pdata = muic_data->pdata;
	enum s2mu005_reg_manual_sw_value val;
	int ret = 0;

	pr_info("%s rustproof mode[%d]\n", __func__, muic_data->is_rustproof);

	if (muic_data->is_rustproof)
		return ret;

	if (pdata->uart_path == MUIC_PATH_UART_AP)
		val = MANSW_AP_UART;
	else
		val = MANSW_CP_UART;

	ret = set_com_sw(muic_data, val);
	if (ret)
		pr_err("%s  set_com_uart err\n", __func__);
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
	if (pdata->uart_path == MUIC_PATH_UART_AP)
		send_uart_noti_to_modem(MODEM_CTRL_UART_AP);
	else
		send_uart_noti_to_modem(MODEM_CTRL_UART_CP);
#endif

	return ret;
}

static int attach_usb(struct s2mu005_muic_data *muic_data)
{
	return com_to_usb(muic_data);
}

static int attach_uart(struct s2mu005_muic_data *muic_data)
{
	return com_to_uart(muic_data);
}

static void s2mu005_muic_cable_rescan(struct s2mu005_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	u8 devt = 0, dp = 0, bcd = 0, int_mask = 0;

	devt = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE1);
	if ((devt & DEV_TYPE1_DEDICATED_CHG) && (devt > DEV_TYPE1_DEDICATED_CHG)) {
		pr_info("%s cable type duplicated(%x#x)\n", __func__, devt);

		dp = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_TEST3);
		if (dp == 0xA0)
			s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_TEST3, 0x0);

		s2mu005_i2c_write_byte(i2c, 0xB5, 0x0);
		msleep(50);

		int_mask = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_INT1_MASK);
		int_mask |= 0x1;
		s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_INT1_MASK, int_mask);

		bcd = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_BCD_RESCAN);
		bcd |= 0x1;
		s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_BCD_RESCAN, bcd);
		msleep(200);

		s2mu005_muic_detect_dev(muic_data);

		int_mask = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_INT1_MASK);
		int_mask &= (~0x1);
		s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_INT1_MASK, int_mask);
	}
}

#if IS_ENABLED(CONFIG_S2MU005_MUIC_DEBUG)
static void s2mu005_muic_debug(struct work_struct *work)
{
	struct s2mu005_muic_data *muic_data =
		container_of(work, struct s2mu005_muic_data, debug_dwrok.work);
	struct i2c_client *i2c = muic_data->i2c;
	u8 devt1 = 0, devt2 = 0, devt3 = 0, devt_app = 0;
	u8 vbus = 0, adc = 0;
	u8 vbus_ldo = 0, dp = 0;
	u8 hid = 0, hid2 = 0;

	adc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_ADC) & ADC_MASK;
	devt1 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE1);
	devt2 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE2);
	devt3 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE3);
	devt_app = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_APPLE);
	vbus = !!(devt_app & DEV_TYPE_APPLE_VBUS_WAKEUP);
	vbus_ldo = s2mu005_i2c_read_byte(i2c, 0x7E);
	dp = s2mu005_i2c_read_byte(i2c, 0x55);
	hid  = s2mu005_i2c_read_byte(i2c, 0x76);
	hid2 = s2mu005_i2c_read_byte(i2c, 0x2a);

	pr_info("dev[0x%x, 0x%x, 0x%x], adc: 0x%X\n", devt1, devt2, devt3, adc);
	pr_info("vbus(0x%x), vbus_ldo(0x%x), dp(0x%x)\n", vbus, vbus_ldo, dp);
	pr_info("0x76(0x%x), 0x2a(0x%x)\n", hid, hid2);
	pr_info("attach(%d), detach(%d), vbus_on(%d), adc(%d), vbus_off(%d)\n",
		muic_data->irq_attach_cnt, muic_data->irq_detach_cnt,
		muic_data->irq_vbus_on_cnt, muic_data->irq_adc_change_cnt,
		muic_data->irq_vbus_off_cnt);

	schedule_delayed_work(&muic_data->debug_dwrok, msecs_to_jiffies(4000));
}
#endif	/* CONFIG_S2MU005_MUIC_DEBUG */

#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
static int s2mu005_muic_dp_0_6V(struct s2mu005_muic_data *muic_data, bool en)
{
	struct i2c_client *i2c = muic_data->i2c;
	int val, ret;
	int devt1 = 0, devt2 = 0, devt3 = 0, devt_app = 0;
	int vbvolt = 0;
	u8 vbus_ldo = 0, dp = 0;

	cancel_delayed_work(&muic_data->dp_0p6v);

	pr_info("%s en : %d\n", __func__, (int)en);
	val = s2mu005_i2c_read_byte(i2c, 0x55);

	if (en) {
		val = 0xA0;
		schedule_delayed_work(&muic_data->dp_0p6v, msecs_to_jiffies(40000));
	} else
		val = 0x00;

	ret = s2mu005_i2c_write_byte(i2c, 0x55, (u8)val);

	msleep(50);

	devt1 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE1);
	devt2 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE2);
	devt3 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE3);
	devt_app = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_APPLE);
	vbvolt = !!(devt_app & DEV_TYPE_APPLE_VBUS_WAKEUP);
	vbus_ldo = s2mu005_i2c_read_byte(i2c, 0x7E);
	dp = s2mu005_i2c_read_byte(i2c, 0x55);

	pr_info("dev[0x%x, 0x%x, 0x%x]\n", devt1, devt2, devt3);
	pr_info("vbvlot(0x%x), vbus_ldo(0x%x), dp(0x%x)\n", vbvolt, vbus_ldo, dp);

	return ret;
}

static void s2mu005_muic_dp_0p6v(struct work_struct *work)
{
	struct s2mu005_muic_data *muic_data =
		container_of(work, struct s2mu005_muic_data, dp_0p6v.work);

	s2mu005_muic_dp_0_6V(muic_data, false);
}

static void s2mu005_muic_bcd_rescan(struct s2mu005_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret = 0;
	u8 reg_val = 0;

	pr_info("%s\n", __func__);

	ret = set_com_sw(muic_data, MANSW_OPEN);
	if (ret < 0)
		pr_err("%s, fail to open mansw\n", __func__);

	msleep(50);

	reg_val = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_BCD_RESCAN);
	reg_val |= 0x1;
	s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_BCD_RESCAN, reg_val);
}

static void s2mu005_muic_cable_recheck(struct work_struct *work)
{
	struct s2mu005_muic_data *muic_data =
		container_of(work, struct s2mu005_muic_data, cable_recheck.work);
	struct i2c_client *i2c = muic_data->i2c;
	u8 dp = 0;

	mutex_lock(&muic_data->recheck_mutex);

	pr_info("%s\n", __func__);

	dp = s2mu005_i2c_read_byte(i2c, 0x55);
	if (dp)
		s2mu005_muic_dp_0_6V(muic_data, false);

	s2mu005_muic_bcd_rescan(muic_data);
	msleep(190);

	s2mu005_muic_detect_dev(muic_data);

	mutex_unlock(&muic_data->recheck_mutex);
}

static void s2mu005_muic_handle_vbus_off(struct s2mu005_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int devt1 = 0, devt2 = 0, devt3 = 0, devt_app = 0;
	int vbvolt = 0;

	devt1 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE1);
	devt2 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE2);
	devt3 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE3);
	devt_app = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_APPLE);
	vbvolt = !!(devt_app & DEV_TYPE_APPLE_VBUS_WAKEUP);
	if (!devt1 && !devt2 && !devt3 && !vbvolt
		&& !(devt_app & DEV_TYPE_APPLE_APPLE_CHG)) {
		union power_supply_propval val = {0, };
		val.intval = SEC_BAT_CHG_MODE_CHARGING_OFF;
		psy_do_property("s2mu005-charger", set,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, val);
	}
}
#endif

static void s2mu005_muic_handle_attach(struct s2mu005_muic_data *muic_data,
			muic_attached_dev_t new_dev, int adc, u8 vbvolt)
{
	struct muic_platform_data *pdata = muic_data->pdata;
	int ret = 0;
#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
	bool noti = (new_dev != muic_data->attached_dev) ? true : false;
#endif

	pr_info("%s %s -> %s, suspended[%s]\n",
		__func__, dev_to_str(muic_data->attached_dev),
		dev_to_str(new_dev), muic_data->suspended ? "T" : "F");

	if (new_dev == muic_data->attached_dev) {
		pr_info("%s duplicated\n", __func__);
		return;
	}

	/* Logically Detach Accessary */
	switch (muic_data->attached_dev) {
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
	case ATTACHED_DEV_OTG_MUIC:
	case ATTACHED_DEV_CHARGING_CABLE_MUIC:
	case ATTACHED_DEV_TA_MUIC:
	case ATTACHED_DEV_UNDEFINED_CHARGING_MUIC:
	case ATTACHED_DEV_TIMEOUT_OPEN_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
	case ATTACHED_DEV_VZW_INCOMPATIBLE_MUIC:
	case ATTACHED_DEV_FACTORY_UART_MUIC:
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
	case ATTACHED_DEV_UNDEFINED_RANGE_MUIC:
#endif
#if IS_ENABLED(CONFIG_MUIC_S2MU005_SUPPORT_HMT)
	case ATTACHED_DEV_HMT_MUIC:
#endif
#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_MUIC_S2MU005_DISCHARGING_WA)
	case ATTACHED_DEV_CARKIT_MUIC:
#endif
		s2mu005_muic_handle_detach(muic_data);
		break;
	case ATTACHED_DEV_DESKDOCK_MUIC:
	case ATTACHED_DEV_DESKDOCK_VB_MUIC:
		switch (new_dev) {
		case ATTACHED_DEV_DESKDOCK_MUIC:
		case ATTACHED_DEV_DESKDOCK_VB_MUIC:
			break;
		default:
			s2mu005_muic_handle_detach(muic_data);
			break;
		}
		break;
	default:
		break;
	}

	muic_data->jigonb_enable = false;

	/* Attach Accessary */
#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
	noti = true;
#endif
	switch (new_dev) {
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
		/*
		* USB BC 1.2 certification :
		* 100ms delay between attach dev,
		* and path setting / start chg.
		*/
		msleep(100);
#endif
	case ATTACHED_DEV_OTG_MUIC:
#if IS_ENABLED(CONFIG_MUIC_S2MU005_SUPPORT_HMT)
	case ATTACHED_DEV_HMT_MUIC:
#endif
		ret = attach_usb(muic_data);
		break;
	case ATTACHED_DEV_TA_MUIC:
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
		/*
		* USB BC 1.2 certification :
		* 100ms delay between attach dev,
		* and path setting / start chg.
		* DP 0.6V when DCP is detected.
		*/
		msleep(100);
		if (muic_data->is_dcp)
			s2mu005_muic_dp_0_6V(muic_data, true);
#endif
	case ATTACHED_DEV_CHARGING_CABLE_MUIC:
	case ATTACHED_DEV_UNDEFINED_CHARGING_MUIC:
	case ATTACHED_DEV_TIMEOUT_OPEN_MUIC:
	case ATTACHED_DEV_UNKNOWN_MUIC:
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
	case ATTACHED_DEV_UNDEFINED_RANGE_MUIC:
#endif
		com_to_open(muic_data);
		break;
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
		if (pdata->is_new_factory)
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
			muic_data->jigonb_enable = false;
#else
			muic_data->jigonb_enable = true;
#endif
		else
			muic_data->jigonb_enable = true;
		ret = attach_uart(muic_data);
		break;
	case ATTACHED_DEV_FACTORY_UART_MUIC:
		muic_data->jigonb_enable = false;
		ret = attach_uart(muic_data);
		break;
	case ATTACHED_DEV_JIG_UART_ON_VB_MUIC:
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
		if (pdata->is_new_factory)
			muic_data->jigonb_enable = false;
		else
			muic_data->jigonb_enable = true;
		ret = attach_uart(muic_data);
		break;
#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_MUIC_S2MU005_DISCHARGING_WA)
	case ATTACHED_DEV_CARKIT_MUIC:
#endif
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
		if (pdata->is_new_factory)
			muic_data->jigonb_enable = false;
		else
			muic_data->jigonb_enable = true;
		ret = attach_usb(muic_data);
		break;
	case ATTACHED_DEV_DESKDOCK_MUIC:
	case ATTACHED_DEV_DESKDOCK_VB_MUIC:
	case ATTACHED_DEV_VZW_INCOMPATIBLE_MUIC:
		break;
	default:
#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
		noti = false;
#endif
		pr_info("%s unsupported dev=%d, adc=0x%x, vbus=%c\n",
				__func__, new_dev, adc, (vbvolt ? 'O' : 'X'));
		break;
	}

	if (ret)
		pr_err("%s error(%d)\n", __func__, ret);

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
	if (noti) {
		if (!muic_data->suspended)
			muic_notifier_attach_attached_dev(new_dev);
		else
			muic_data->need_to_noti = true;
	}
#endif /* CONFIG_MUIC_NOTIFIER */

	muic_data->attached_dev = new_dev;
}

static void s2mu005_muic_handle_detach(struct s2mu005_muic_data *muic_data)
{
#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
	bool noti = true;
#endif

	muic_data->jigonb_enable = false;
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	muic_data->usb_type_rechecked = false;
#endif

	if (muic_data->attached_dev == ATTACHED_DEV_NONE_MUIC) {
		pr_info("%s Detach duplicated(NONE)\n", __func__);
		goto out_without_noti;
	}

	pr_info("%s %s\n", __func__, dev_to_str(muic_data->attached_dev));
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	if (muic_data->is_dcp) {
		s2mu005_muic_dp_0_6V(muic_data, false);
		muic_data->is_dcp = false;
	}
#endif

#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
	if (noti) {
		if (!muic_data->suspended)
			muic_notifier_detach_attached_dev(muic_data->attached_dev);
		else
			muic_data->need_to_noti = true;
	}
#endif /* CONFIG_MUIC_NOTIFIER */

out_without_noti:
	com_to_open(muic_data);
	muic_data->attached_dev = ATTACHED_DEV_NONE_MUIC;
}

static void s2mu005_muic_detect_dev(struct s2mu005_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	muic_attached_dev_t new_dev = ATTACHED_DEV_UNKNOWN_MUIC;
	int vbvolt = 0, vmid = 0;
	int devt1, devt2, devt3, devt_app, chgt, sc, adc;
	u8 vbus_ldo = 0, dp = 0;

	devt1 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE1);
	devt2 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE2);
	devt3 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE3);
	adc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_ADC);
	devt_app = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_APPLE);
	chgt = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_CHG_TYPE);
	sc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_SC_STATUS2);
	vbus_ldo = s2mu005_i2c_read_byte(i2c, 0x7E);
	dp = s2mu005_i2c_read_byte(i2c, 0x55);

	vbvolt = !!(devt_app & DEV_TYPE_APPLE_VBUS_WAKEUP);
	vmid = (sc & 0x7);

	pr_info("dev[0x%x, 0x%x, 0x%x]\n", devt1, devt2, devt3);
	pr_info("adc : 0x%x, chgt : 0x%x, apple : 0x%x\n", adc, chgt, devt_app);
	pr_info("vbvolt : 0x%x, vmid : 0x%x, ver : 0x%x%x\n",
		vbvolt, vmid, muic_data->muic_version, muic_data->ic_rev_id);
	pr_info("vbus_ldo(0x%x), dp(0x%x)\n", vbus_ldo, dp);

	if (ADC_CONVERSION_MASK & adc) {
		pr_err("%s ADC conversion error(%d)\n", __func__, adc);
		return;
	}

	/* Detected */
	switch (devt1) {
	case DEV_TYPE1_CDP:
		if (vbvolt)
			new_dev = ATTACHED_DEV_CDP_MUIC;
		break;
	case DEV_TYPE1_USB:
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
		if (vbvolt) {
		   if (muic_data->usb_type_rechecked) {
			   muic_data->usb_type_rechecked = false;
			   new_dev = ATTACHED_DEV_USB_MUIC;
		   } else {
			   muic_data->usb_type_rechecked = true;
			   schedule_delayed_work(&muic_data->cable_recheck, 0);
			   return;
		   }
		}
#else
		if (vbvolt)
			new_dev = ATTACHED_DEV_USB_MUIC;
#endif
		break;
	case DEV_TYPE1_DEDICATED_CHG:
	case DEV_TYPE1_DEDICATED_CHG2:
	case DEV_TYPE1_CHG_TYPES:
		if (vbvolt) {
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
			muic_data->is_dcp = true;
#endif
			new_dev = ATTACHED_DEV_TA_MUIC;
		}
		break;
	case DEV_TYPE1_USB_OTG:
		new_dev = ATTACHED_DEV_OTG_MUIC;
		if (vmid == 0x4) {
			pr_info("USB_OTG VMID DETECTED[%d]\n", vmid);
			vbvolt = 1;
		}
		break;
	case DEV_TYPE1_T1_T2_CHG:
		if (vbvolt) {
			/* 200K, 442K should be checked */
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
			new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
#elif IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_MUIC_S2MU005_DISCHARGING_WA)
			new_dev = ATTACHED_DEV_CARKIT_MUIC;
#else
			if (adc == ADC_CEA936ATYPE2_CHG)
				new_dev = ATTACHED_DEV_TA_MUIC;
			else
				new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("T1_T2 CHARGER DETECTED\n");
#endif
		} else {
			/* W/A, 442k without VB changes to 523K (JIG_UART_OFF)
			* To prevent to keep sleep mode
			*/
			new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
			pr_info("442K->523K JIG_USB_OFF DETECTED\n");
		}
		break;
	default:
		break;
	}

	switch (devt2) {
	case DEV_TYPE2_SDP_1P8S:
		if (vbvolt) {
			struct muic_platform_data *pdata = muic_data->pdata;

			if (pdata->dcd_timeout)
				new_dev = ATTACHED_DEV_TIMEOUT_OPEN_MUIC;
			else
				new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("SDP_1P8S DETECTED\n");
		}
		break;
	case DEV_TYPE2_JIG_UART_OFF:
		if (muic_data->is_otg_test) {
			mdelay(1000);
			sc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_SC_STATUS2);
			vmid = sc & 0x7;
			pr_info("%s  vmid : 0x%x\n", __func__, vmid);
			if (vmid == 0x4) {
				pr_info("OTG_TEST vmid = %d\n", vmid);
				vbvolt = 1;
			}
			new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		} else if (vbvolt) {
			new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		} else {
			new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		}
		break;
	case DEV_TYPE2_JIG_UART_ON:
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
		if (vbvolt) {
			new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
		}
		else {
			new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
		}
#else
		new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
#endif
		break;
	case DEV_TYPE2_JIG_USB_OFF:
		if (!vbvolt)
			break;
		new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
		break;
	case DEV_TYPE2_JIG_USB_ON:
		if (!vbvolt)
			break;
		new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		break;
	default:
		break;
	}

	switch (devt3) {
	case DEV_TYPE3_MHL:
		pr_info("%s: MHL DETECTED\n", __func__);
		if (muic_data->pdata->is_factory_uart)
			new_dev = ATTACHED_DEV_FACTORY_UART_MUIC;
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
		else if (vbvolt) {
			new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
		}
#endif
		break;
	default:
		break;
	}

	/* This is for Apple cables */
	if (vbvolt && (devt_app & DEV_TYPE_APPLE_APPLE_CHG)) {
		new_dev = ATTACHED_DEV_TA_MUIC;
		pr_info("%s: APPLE_CHG DETECTED\n", __func__);
	}

#if !defined(CONFIG_TYPEB_WATERPROOF_MODEL)
	if ((chgt & DEV_TYPE_CHG_TYPE) &&
		(new_dev == ATTACHED_DEV_UNKNOWN_MUIC)) {
		/* This is workaround for LG USB cable which has 219k ohm ID */
		if (adc == ADC_CEA936ATYPE1_CHG || adc == ADC_JIG_USB_OFF) {
			new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("[muic] TYPE1_CHARGER DETECTED (USB)\n");
		} else {
			new_dev = ATTACHED_DEV_TA_MUIC;
			pr_info("[muic] TYPE3_CHARGER DETECTED\n");
		}
	}
#endif

	if (devt2 & DEV_TYPE2_AV || devt3 & DEV_TYPE3_AV_WITH_VBUS) {
		if (vbvolt)
			new_dev = ATTACHED_DEV_DESKDOCK_VB_MUIC;
		else
			new_dev = ATTACHED_DEV_DESKDOCK_MUIC;
	}

	/* If there is no matching device found using device type registers
	 *  use ADC to find the attached device
	 */
	if (new_dev == ATTACHED_DEV_UNKNOWN_MUIC) {
		switch (adc) {
#if IS_ENABLED(CONFIG_MUIC_INCOMPATIBLE_VZW)
		case ADC_INCOMPATIBLE_VZW:
			new_dev = ATTACHED_DEV_VZW_INCOMPATIBLE_MUIC;
			break;
#endif
		case ADC_CHARGING_CABLE:
			break;
		case ADC_CEA936ATYPE1_CHG: /*200k ohm */
			/* This is workaround for LG USB cable which has 219k ohm ID */
			if (vbvolt) {
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
				new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
#elif defined(CONFIG_SEC_FACTORY) && defined(CONFIG_MUIC_S2MU005_DISCHARGING_WA)
				new_dev = ATTACHED_DEV_CARKIT_MUIC;
#else
				new_dev = ATTACHED_DEV_USB_MUIC;
#endif
				pr_info("%s: ADC TYPE1 CHARGER DETECTED\n", __func__);

			}
			break;
		case ADC_CEA936ATYPE2_CHG:
			if (vbvolt) {
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
				new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
#else
				new_dev = ATTACHED_DEV_TA_MUIC;
#endif
				pr_info("%s: ADC TYPE2 CHARGER DETECTED\n", __func__);
			}
			break;
		case ADC_JIG_USB_OFF: /* 255k */
			if (!vbvolt)
				break;
			new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
			break;
		case ADC_JIG_USB_ON:
			if (!vbvolt)
				break;
			new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
			break;
		case ADC_JIG_UART_OFF:
			if (muic_data->is_otg_test) {
				mdelay(1000);
				sc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_SC_STATUS2);
				vmid = sc & 0x7;
				pr_info("%s: vmid : 0x%x\n", __func__, vmid);
				if (vmid == 0x4) {
					pr_info("%s: ADC OTG_TEST DETECTED, vmid = %d\n",
							__func__, vmid);
					vbvolt = 1;
				}
				new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
			} else if (vbvolt) {
				new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
			} else {
				new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
			}
			break;
		case ADC_JIG_UART_ON:
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
			if (vbvolt) {
				new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
			}
			else {
				new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
			}
#else
			new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
#endif
			break;
		case ADC_DESKDOCK:
			if (vbvolt)
				new_dev = ATTACHED_DEV_DESKDOCK_VB_MUIC;
			else
				new_dev = ATTACHED_DEV_DESKDOCK_MUIC;
			break;
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
		case ADC_SMARTDOCK:
			if(vbvolt) {
				new_dev = ATTACHED_DEV_UNDEFINED_CHARGING_MUIC;
			}
			break;
#endif
#if IS_ENABLED(CONFIG_MUIC_S2MU005_SUPPORT_HMT)
		case ADC_HMT:
			new_dev = ATTACHED_DEV_HMT_MUIC;
			break;
#endif
		case ADC_OPEN:
			break;
		default:
			pr_info("%s: unsupported ADC(0x%02x)\n", __func__, adc);
			break;
		}
	}

	if ((new_dev == ATTACHED_DEV_UNKNOWN_MUIC) && (adc != ADC_OPEN)) {
		if (vbvolt) {
#if defined(CONFIG_TYPEB_WATERPROOF_MODEL)
			new_dev = ATTACHED_DEV_UNDEFINED_RANGE_MUIC;
			pr_info("%s: UNDEFINED RANGE DETECTED\n", __func__);
#else
			new_dev = ATTACHED_DEV_UNDEFINED_CHARGING_MUIC;
			pr_info("%s: UNDEFINED VB DETECTED\n", __func__);
#endif
		}
	}

	pr_info("%s new dev=%s\n", __func__, dev_to_str(new_dev));

	if (new_dev != ATTACHED_DEV_UNKNOWN_MUIC)
		s2mu005_muic_handle_attach(muic_data, new_dev, adc, vbvolt);
	else
		s2mu005_muic_handle_detach(muic_data);

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle((!!vbvolt) ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
}

static int s2mu005_muic_reg_init(struct s2mu005_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret;
	int devt1, devt2, devt3, val, adc, ctrl1;
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	u8 reg_val = 0, tmp_val = 0, vldo_set = 0;
#endif

	devt1 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE1);
	devt2 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE2);
	devt3 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE3);
	val = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_SW_CTRL);
	val &= 0x03;
	adc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_ADC);
	pr_info("%s dev[1:0x%x, 2:0x%x, 3:0x%x], adc:0x%x\n",
		__func__, devt1, devt2, devt3, adc);

	if ((devt1 & DEV_TYPE1_USB_TYPES) ||
		(devt2 & DEV_TYPE2_JIG_USB_TYPES)) {
		ret = s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_SW_CTRL, val | MANUAL_SW_USB);
		if (ret < 0)
			pr_err("%s usb type detect err(%d)\n", __func__, ret);
	} else if (devt2 & DEV_TYPE2_JIG_UART_TYPES) {
		ret = s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_SW_CTRL, val | MANUAL_SW_UART);
		if (ret < 0)
			pr_err("%s uart type detect err(%d)\n", __func__, ret);
	}

	ret = s2mu005_i2c_write_byte(i2c,
			S2MU005_REG_MUIC_CTRL1, CTRL_MASK);
	if (ret < 0)
		pr_err("%s failed to write ctrl(%d)\n", __func__, ret);
	ctrl1 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_CTRL1);
	pr_info("%s CTRL1:0x%02x\n", __func__, ctrl1);

#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	vldo_set = s2mu005_i2c_read_byte(i2c, 0x54);
	if (!(vldo_set & 0x80)) {
		vldo_set |= 0x80;
		s2mu005_i2c_write_byte(i2c, 0x54, vldo_set);

		muic_data->vbus_ldo = reg_val = s2mu005_i2c_read_byte(i2c, 0x7E);
		if ((reg_val >> 4) > 3)
			tmp_val = ((reg_val >> 4) - 3);
		else
			tmp_val = 0;

		reg_val &= ~(0xf << 4);
		reg_val |= (tmp_val << 4);
		s2mu005_i2c_write_byte(i2c, 0x7E, reg_val);
		pr_info("%s vbus_ldo(%#x->%#x)\n", __func__, muic_data->vbus_ldo, reg_val);
	}
#endif
#ifndef CONFIG_SEC_FACTORY
	s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_LDOADC_VSETL, LDOADC_VSET_3V);
	s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_LDOADC_VSETH, LDOADC_VSET_3V);
#endif

	return ret;
}

static irqreturn_t s2mu005_muic_irq_thread(int irq, void *data)
{
	struct s2mu005_muic_data *muic_data = data;
	struct i2c_client *i2c = muic_data->i2c;
	enum s2mu005_reg_manual_sw_value val;
	struct irq_desc *desc = irq_to_desc(irq);
	int ctrl, ret = 0;
#ifndef CONFIG_SEC_FACTORY
	int vbvolt = 0, adc = 0;
#endif

	pr_info("%s %s\n", __func__,
		desc ? desc->action->name : "-1");

#if IS_ENABLED(CONFIG_S2MU005_MUIC_DEBUG)
	if (irq == muic_data->irq_attach)
		muic_data->irq_attach_cnt++;
	else if (irq == muic_data->irq_detach)
		muic_data->irq_detach_cnt++;
	else if (irq == muic_data->irq_vbus_on)
		muic_data->irq_vbus_on_cnt++;
	else if (irq == muic_data->irq_adc_change)
		muic_data->irq_adc_change_cnt++;
	else if (irq == muic_data->irq_vbus_off)
		muic_data->irq_vbus_off_cnt++;
#endif	/* CONFIG_S2MU005_MUIC_DEBUG */

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);
#ifndef CONFIG_SEC_FACTORY
	vbvolt = !!(s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_APPLE)
				& DEV_TYPE_APPLE_VBUS_WAKEUP);
	adc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_ADC) & ADC_MASK;
	pr_info("vbvolt : %d, adc: 0x%X, irq : %d\n", vbvolt, adc, irq);
	if (!vbvolt) {
		if (IS_AUDIO_ADC(adc) && !muic_data->is_water_wa) {
			if (irq == muic_data->irq_adc_change)
				s2mu005_muic_set_water_wa(muic_data, true);
		} else if (IS_WATER_ADC(adc) && !muic_data->is_water_wa) {
			if (irq == muic_data->irq_attach)
				s2mu005_muic_set_water_wa(muic_data, true);
		}
	}

	if (adc == ADC_OPEN &&
		irq ==  muic_data->irq_adc_change &&
		muic_data->is_water_wa) {
		usleep_range(WATER_TOGGLE_WA_MIN_DURATION_US, WATER_TOGGLE_WA_MAX_DURATION_US);
		s2mu005_muic_set_water_wa(muic_data, false);
	}
#endif
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	if (irq == muic_data->irq_vbus_off)
		s2mu005_muic_handle_vbus_off(muic_data);
#endif
	/* check for muic reset and re-initialize registers */
	ctrl = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_CTRL1);

	/* CONTROL register is reset to DF */
	if (ctrl == 0xDF) {
#if IS_ENABLED(DEBUG_MUIC)
		s2mu005_print_reg_log();
		s2mu005_print_reg_dump(muic_data);
#endif
		pr_err("failed to muic could have been reseted. Initilize\n");
		s2mu005_muic_reg_init(muic_data);
		if (muic_data->is_rustproof) {
			pr_info("%s rustproof is enabled\n", __func__);
			val = MANSW_OPEN;
			ret = s2mu005_i2c_write_byte(i2c,
				S2MU005_REG_MUIC_SW_CTRL, val);
			if (ret < 0)
				pr_err("%s failed to write MANSW\n", __func__);
			ret = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_SW_CTRL);
			pr_info("%s MUIC_SW_CTRL=0x%x\n ", __func__, ret);
		} else {
			val = MANSW_AP_UART;
			ret = s2mu005_i2c_write_byte(i2c,
				S2MU005_REG_MUIC_SW_CTRL, val);
			if (ret < 0)
				pr_err("%s failed to write MANSW\n", __func__);
			ret = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_SW_CTRL);
			pr_info("%s MUIC_SW_CTRL=0x%x\n ", __func__, ret);
		}
#if IS_ENABLED(DEBUG_MUIC)
		s2mu005_print_reg_dump(muic_data);
#endif
		/* MUIC Interrupt On */
		ret = set_ctrl_reg(muic_data, CTRL_INT_MASK_SHIFT, false);
#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
		init_otg_reg(muic_data);
#endif
	}

	/* device detection */
	s2mu005_muic_detect_dev(muic_data);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

static int s2mu005_init_rev_info(struct s2mu005_muic_data *muic_data)
{
	u8 dev_id;
	int ret = 0;

	dev_id = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_REV_ID);
	if (dev_id < 0) {
		pr_err("dev_id(%d)\n", dev_id);
		ret = -ENODEV;
	} else {
		muic_data->muic_version = (dev_id & 0x0F);
		muic_data->ic_rev_id = (dev_id & 0xF0) >> 4;
		pr_info("dev_id=0x%x%x\n",
			muic_data->ic_rev_id, muic_data->muic_version);
	}
	return ret;
}

#define REQUEST_IRQ(_irq, _dev_id, _name)				\
do {									\
	ret = request_threaded_irq(_irq, NULL,		\
			s2mu005_muic_irq_thread, 0, _name, _dev_id);	\
	if (ret < 0)						\
		pr_err("Failed to request IRQ #%d: %d\n", _irq, ret);	\
} while (0)

static int s2mu005_muic_irq_init(struct s2mu005_muic_data *muic_data)
{
	int ret = 0;

	if (muic_data->mfd_pdata && (muic_data->mfd_pdata->irq_base > 0)) {
		int irq_base = muic_data->mfd_pdata->irq_base;

		/* request MUIC IRQ */
		muic_data->irq_attach = irq_base + S2MU005_MUIC_IRQ1_ATTATCH;
		REQUEST_IRQ(muic_data->irq_attach, muic_data, "muic-attach");

		muic_data->irq_detach = irq_base + S2MU005_MUIC_IRQ1_DETACH;
		REQUEST_IRQ(muic_data->irq_detach, muic_data, "muic-detach");

		muic_data->irq_vbus_on = irq_base + S2MU005_MUIC_IRQ2_VBUS_ON;
		REQUEST_IRQ(muic_data->irq_vbus_on, muic_data, "muic-vbus_on");

		muic_data->irq_adc_change = irq_base + S2MU005_MUIC_IRQ2_ADC_CHANGE;
		REQUEST_IRQ(muic_data->irq_adc_change, muic_data, "muic-adc_change");

		muic_data->irq_vbus_off = irq_base + S2MU005_MUIC_IRQ2_VBUS_OFF;
		REQUEST_IRQ(muic_data->irq_vbus_off, muic_data, "muic-vbus_off");
	}

	pr_info("attach(%d), detach(%d), vbus_on(%d), adc(%d), vbus_off(%d)\n",
		muic_data->irq_attach, muic_data->irq_detach,
		muic_data->irq_vbus_on, muic_data->irq_adc_change,
		muic_data->irq_vbus_off);

	return ret;
}

#define FREE_IRQ(_irq, _dev_id, _name)					\
do {									\
	if (_irq) {							\
		free_irq(_irq, _dev_id);				\
		pr_info("IRQ(%d):%s free done\n", _irq, _name);	\
	}								\
} while (0)

static void s2mu005_muic_free_irqs(struct s2mu005_muic_data *muic_data)
{
	pr_info("%s\n", __func__);

	/* free MUIC IRQ */
	FREE_IRQ(muic_data->irq_attach, muic_data, "muic-attach");
	FREE_IRQ(muic_data->irq_detach, muic_data, "muic-detach");
	FREE_IRQ(muic_data->irq_vbus_on, muic_data, "muic-vbus_on");
	FREE_IRQ(muic_data->irq_adc_change, muic_data, "muic-adc_change");
	FREE_IRQ(muic_data->irq_vbus_off, muic_data, "muic-vbus_off");
}

#if IS_ENABLED(CONFIG_OF)
static int of_s2mu005_muic_dt(struct device *dev, struct s2mu005_muic_data *muic_data)
{
	struct device_node *np, *np_muic;
	struct muic_platform_data *pdata = muic_data->pdata;
	int ret = 0;

	np = dev->parent->of_node;
	if (!np) {
		pr_err("failed to find np\n");
		return -ENODEV;
	}

	np_muic = of_find_node_by_name(NULL, "muic");
	if (!np_muic) {
		pr_err("failed to find muic sub-node np_muic\n");
		return -EINVAL;
	}

#if !IS_ENABLED(CONFIG_MUIC_UART_SWITCH)
	if (of_gpio_count(np_muic) < 1) {
		pr_err("%s failed to find muic gpio\n", __func__);
		pdata->gpio_uart_sel = 0;
	} else
		pdata->gpio_uart_sel = of_get_gpio(np_muic, 0);
#endif

	pdata->is_new_factory = of_property_read_bool(np_muic, "new_factory");
	pdata->dcd_timeout = of_property_read_bool(np_muic, "dcd_timeout");

	return ret;
}
#endif /* CONFIG_OF */

/* if need to set s2mu005 pdata */
static struct of_device_id s2mu005_muic_match_table[] = {
	{ .compatible = "s2mu005-muic",},
	{},
};

static int s2mu005_muic_probe(struct platform_device *pdev)
{
	struct s2mu005_dev *s2mu005 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu005_platform_data *mfd_pdata = dev_get_platdata(s2mu005->dev);
	struct s2mu005_muic_data *muic_data;
	int ret = 0;

	if (unlikely(!mfd_pdata))
		return PTR_ERR(mfd_pdata);

	muic_data = devm_kzalloc(&pdev->dev, sizeof(*muic_data), GFP_KERNEL);
	if (unlikely(!muic_data)) {
		pr_err("failed to allocate driver data\n");
		return PTR_ERR(muic_data);
	}

	muic_data->dev = &pdev->dev;
	muic_data->i2c = s2mu005->i2c;
	muic_data->mfd_pdata = mfd_pdata;
	muic_data->pdata = &muic_pdata;
	pr_info("factory_uart: %d\n", muic_data->pdata->is_factory_uart);
#if IS_ENABLED(CONFIG_OF)
	ret = of_s2mu005_muic_dt(&pdev->dev, muic_data);
	if (ret < 0)
		pr_err("no muic dt(%d)\n", ret);
#endif /* CONFIG_OF */

	mutex_init(&muic_data->muic_mutex);
	muic_data->is_factory_start = false;
	muic_data->attached_dev = ATTACHED_DEV_UNKNOWN_MUIC;
	muic_data->is_usb_ready = false;
	muic_data->jigonb_enable = false;
	muic_data->jig_disable = false;
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	mutex_init(&muic_data->recheck_mutex);
	muic_data->is_dcp = false;
	muic_data->usb_type_rechecked = false;
#endif
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
	muic_data->is_water_wa = false;
#endif
	platform_set_drvdata(pdev, muic_data);
	if (muic_data->pdata->init_gpio_cb) {
		ret = muic_data->pdata->init_gpio_cb(muic_data->pdata, get_switch_sel());
		if (ret) {
			pr_err("failed to init gpio(%d)\n", ret);
			goto err_init_gpio;
		}
	}

#if IS_ENABLED(CONFIG_SEC_SYSFS)
	ret = sysfs_create_group(&switch_device->kobj, &s2mu005_muic_group);
	if (ret) {
		pr_err("failed to create sysfs\n");
		goto err_sysfs;
	}
	dev_set_drvdata(switch_device, muic_data);
#endif

	ret = s2mu005_init_rev_info(muic_data);
	if (ret) {
		pr_err("failed to get rev info(%d)\n", ret);
		goto err_init_rev;
	}

	ret = s2mu005_muic_reg_init(muic_data);
	if (ret) {
		pr_err("failed to init muic(%d)\n", ret);
		goto err_init_reg;
	}

	/* For Rustproof */
	muic_data->is_rustproof = muic_data->pdata->rustproof_on;
	if (muic_data->is_rustproof) {
		pr_info("rustproof is enabled\n");
		com_to_open(muic_data);
	}

	if (muic_data->pdata->init_switch_dev_cb)
		muic_data->pdata->init_switch_dev_cb();

	ret = s2mu005_muic_irq_init(muic_data);
	if (ret) {
		pr_err("failed to init irq(%d)\n", ret);
		goto err_init_irq;
	}

	wake_lock_init(&muic_data->wake_lock, WAKE_LOCK_SUSPEND, "muic_wake");

	/* initial cable detection */
	ret = set_ctrl_reg(muic_data, CTRL_INT_MASK_SHIFT, false);
#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
	init_otg_reg(muic_data);
#endif
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	INIT_DELAYED_WORK(&muic_data->cable_recheck, s2mu005_muic_cable_recheck);
	INIT_DELAYED_WORK(&muic_data->dp_0p6v, s2mu005_muic_dp_0p6v);
#endif

#if IS_ENABLED(CONFIG_S2MU005_MUIC_DEBUG)
	INIT_DELAYED_WORK(&muic_data->debug_dwrok, s2mu005_muic_debug);
#endif
	s2mu005_muic_irq_thread(-1, muic_data);

	s2mu005_muic_cable_rescan(muic_data);

	return 0;

err_init_irq:
	s2mu005_muic_free_irqs(muic_data);
err_init_reg:
err_init_rev:
#if IS_ENABLED(CONFIG_SEC_SYSFS)
	sysfs_remove_group(&switch_device->kobj, &s2mu005_muic_group);
#endif
	mutex_destroy(&muic_data->muic_mutex);
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	mutex_destroy(&muic_data->recheck_mutex);
#endif
err_sysfs:
err_init_gpio:
	return ret;
}

static int s2mu005_muic_remove(struct platform_device *pdev)
{
	struct s2mu005_muic_data *muic_data = platform_get_drvdata(pdev);

	pr_info("%s\n", __func__);
#if IS_ENABLED(CONFIG_SEC_SYSFS)
	sysfs_remove_group(&switch_device->kobj, &s2mu005_muic_group);
#endif

	disable_irq_wake(muic_data->i2c->irq);
	s2mu005_muic_free_irqs(muic_data);
	mutex_destroy(&muic_data->muic_mutex);
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	mutex_destroy(&muic_data->recheck_mutex);
#endif
	i2c_set_clientdata(muic_data->i2c, NULL);
	return 0;
}

static void s2mu005_muic_shutdown(struct platform_device *pdev)
{
	struct s2mu005_muic_data *muic_data = platform_get_drvdata(pdev);
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	struct i2c_client *i2c = muic_data->i2c;
	u8 reg_val, vldo_set = 0;
#endif

	pr_info("%s\n", __func__);

	if (!muic_data->i2c)
		return;

#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	if (muic_data->is_dcp) {
		s2mu005_muic_dp_0_6V(muic_data, false);
		muic_data->is_dcp = false;
	}
	muic_data->usb_type_rechecked = false;

	vldo_set = s2mu005_i2c_read_byte(i2c, 0x54);
	if (vldo_set & 0x80) {
		vldo_set &= ~0x80;
		s2mu005_i2c_write_byte(i2c, 0x54, vldo_set);

		reg_val = s2mu005_i2c_read_byte(muic_data->i2c, 0x7E);
		s2mu005_i2c_write_byte(muic_data->i2c, 0x7E, muic_data->vbus_ldo);
		pr_info("%s vbus_ldo(%#x->%#x)\n", __func__, reg_val, muic_data->vbus_ldo);
	}
#endif
}

#if IS_ENABLED(CONFIG_PM)
static int s2mu005_muic_suspend(struct device *dev)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	muic_data->suspended = true;
#if IS_ENABLED(CONFIG_S2MU005_MUIC_DEBUG)
	cancel_delayed_work(&muic_data->debug_dwrok);
#endif
	return 0;
}

static int s2mu005_muic_resume(struct device *dev)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s\n", __func__);
	muic_data->suspended = false;
	if (muic_data->need_to_noti) {
		if (muic_data->attached_dev)
			muic_notifier_attach_attached_dev(muic_data->attached_dev);
		else
			muic_notifier_detach_attached_dev(muic_data->attached_dev);
		muic_data->need_to_noti = false;
	}
#if IS_ENABLED(CONFIG_S2MU005_MUIC_DEBUG)
	schedule_delayed_work(&muic_data->debug_dwrok, msecs_to_jiffies(4000));
#endif
	return 0;
}
#else
#define s2mu005_muic_suspend NULL
#define s2mu005_muic_resume NULL
#endif

static SIMPLE_DEV_PM_OPS(s2mu005_muic_pm_ops,
	s2mu005_muic_suspend, s2mu005_muic_resume);

static struct platform_driver s2mu005_muic_driver = {
	.driver = {
		.name = "s2mu005-muic",
		.owner	= THIS_MODULE,
		.of_match_table = s2mu005_muic_match_table,
#if IS_ENABLED(CONFIG_PM)
		.pm = &s2mu005_muic_pm_ops,
#endif
	},
	.probe = s2mu005_muic_probe,
	.remove = s2mu005_muic_remove,
	.shutdown = s2mu005_muic_shutdown,
};

module_platform_driver(s2mu005_muic_driver);

MODULE_DESCRIPTION("S.LSI S2MU005 MUIC driver");
MODULE_LICENSE("GPL");
