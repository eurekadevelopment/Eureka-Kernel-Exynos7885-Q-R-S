/*
 * Copyright (C) 2018 Samsung Electronics
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

#if IS_ENABLED(CONFIG_MUIC_SYSFS)
#include <linux/muic/muic_sysfs.h>
#endif

#if defined(CONFIG_BATTERY_SAMSUNG_V2)
#include "../battery_v2/include/sec_charging_common.h"
#endif

#include <linux/muic/s2mu005-muic-typec.h>
#if IS_ENABLED(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#include <linux/muic/muic_interface.h>
#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
#include <linux/vbus_notifier.h>
#endif /* CONFIG_VBUS_NOTIFIER */
#include "muic-internal.h"
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
#include <soc/samsung/exynos-modem-ctrl.h>
#endif

#define MUIC_CCIC_NOTI_ATTACH (1)
#define MUIC_CCIC_NOTI_DETACH (-1)

/* Prototypes of the Static symbols of s2mu005-muic */
static void s2mu005_muic_detect_dev_ccic(struct s2mu005_muic_data *muic_data,
		muic_attached_dev_t new_dev);
static int s2mu005_muic_detect_dev_bc1p2(struct s2mu005_muic_data *muic_data);
static void s2mu005_muic_handle_attached_dev(struct s2mu005_muic_data *muic_data);
static irqreturn_t s2mu005_muic_attach_isr(int irq, void *data);
static int _s2mu005_muic_sel_path(struct s2mu005_muic_data *muic_data,
		t_path_data path_data);

/*
 * Debuging functions
 */
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

#define DEBUG_MUIC
#define TBD 0

#if IS_ENABLED(DEBUG_MUIC)
#define MAX_LOG 25
#define READ 0
#define WRITE 1

static u8 s2mu005_log_cnt;
static u8 s2mu005_log[MAX_LOG][3];

/*
 * Debug Functions
 */
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

	mutex_lock(&muic->muic_mutex);
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
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_INT1, &val);
	sprintf(mesg + strlen(mesg), "INT1:%x ", val);
	s2mu005_read_reg(muic->i2c, S2MU005_REG_MUIC_INT2, &val);
	sprintf(mesg + strlen(mesg), "INT2:%x ", val);
	mutex_unlock(&muic->muic_mutex);
}

void s2mu005_print_reg_dump(struct s2mu005_muic_data *muic_data)
{
	char mesg[256] = "";

	s2mu005_read_reg_dump(muic_data, mesg);

	pr_info("%s %s\n", __func__, mesg);
}
#endif

/*
 * Unit Functions
 */
int s2mu005_i2c_read_byte(struct i2c_client *client, u8 command)
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

int s2mu005_set_gpio_uart_sel(struct s2mu005_muic_data *muic_data, int uart_sel)
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

#if TBD
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
	pr_info("%s %s\n", __func__, buf);

	return count;
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

static ssize_t is_jig_powered_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct s2mu005_muic_data *muic_data = dev_get_drvdata(dev);

	pr_info("%s  attached_dev[%d]\n", __func__, muic_data->attached_dev);

	switch (muic_data->attached_dev) {
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
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

		s2mu005_muic_attach_isr(-1, muic_data);

		int_mask = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_INT1_MASK);
		int_mask &= (~0x1);
		s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_INT1_MASK, int_mask);
	}
}
#endif /* TBD */

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

int _s2mu005_muic_set_jig_on(struct s2mu005_muic_data *muic_data)
{
#if IS_ENABLED(CONFIG_SEC_FACTORY)
    struct muic_platform_data *muic_pdata = muic_data->pdata;
    bool en = muic_pdata->is_jig_on;
    int reg = 0, ret = 0;

    pr_err("%s: %s\n", __func__, en ? "on" : "off");
    reg = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_SW_CTRL);

    if (muic_pdata->is_jig_on && !muic_pdata->jig_disable)
        reg |= MANUAL_SW_CTRL_JIG_MASK;
    else
        reg &= ~(MANUAL_SW_CTRL_JIG_MASK);

    ret = s2mu005_i2c_write_byte(muic_data->i2c, S2MU005_REG_MUIC_SW_CTRL, (u8)reg);

    return ret;
#else
    pr_err("%s: Skip the jig control, Not Factory Mode.\n", __func__);

    return 0;
#endif
}

static inline int _s2mu005_muic_get_vbus_state(struct s2mu005_muic_data *muic_data)
{
	return ((s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_DEVICE_APPLE)
				& DEV_TYPE_APPLE_VBUS_WAKEUP) >> DEV_TYPE_APPLE_VBUS_WAKEUP_SHIFT);
}

static inline int _s2mu005_muic_get_rid_adc(struct s2mu005_muic_data *muic_data)
{
	return (s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_ADC) & ADC_MASK);
}

static int _s2mu005_muic_sel_path(struct s2mu005_muic_data *muic_data,
		t_path_data path_data)
{
	int ret = 0;
	u8 reg_val1, reg_val2;

	reg_val1 = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_SW_CTRL);
	reg_val2 = reg_val1 & ~(MANUAL_SW_CTRL_DM_SWITCHING_MASK | MANUAL_SW_CTRL_DP_SWITCHING_MASK);

#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
	if (muic_data->is_hiccup_mode) {
		reg_val2 |= MANUAL_SW_CTRL_UART2_MASK;
		goto DONE;
	}
#endif
	switch (path_data) {
	case S2MU005_PATH_USB:
		reg_val2 |= MANUAL_SW_CTRL_USB_MASK;
		break;
	case S2MU005_PATH_UART_AP:
		reg_val2 |= MANUAL_SW_CTRL_UART1_MASK;
		break;
	case S2MU005_PATH_UART_CP:
#if IS_ENABLED(CONFIG_PMU_UART_SWITCH)
		reg_val2 |= MANUAL_SW_CTRL_UART1_MASK;
		break;
#else
		reg_val2 |= MANUAL_SW_CTRL_UART2_MASK;
		break;
#endif
	case S2MU005_PATH_OPEN:
	default:
		reg_val2 |= MANUAL_SW_CTRL_OPEN_MASK;
		break;
	}

#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
DONE:
#endif

	if (reg_val1 != reg_val2) {
		ret = s2mu005_i2c_write_byte(muic_data->i2c, S2MU005_REG_MUIC_SW_CTRL, reg_val2);
		reg_val2 = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_SW_CTRL);
		pr_info("%s manual_sw_ctrl(%#x->%#x)\n", __func__, reg_val1, reg_val2);
	} else
		pr_info("%s Skip to set same path val(%#x)\n", __func__, reg_val1);

    return ret;
}

static int _s2mu005_muic_com_to_uart(struct s2mu005_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int ret = 0;

	if (muic_data->pdata->is_rustproof) {
		pr_info("%s rustproof mode\n", __func__);
		return ret;
	}

	if (muic_pdata->uart_path == MUIC_PATH_UART_AP) {
		ret = _s2mu005_muic_sel_path(muic_data, S2MU005_PATH_UART_AP);
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
		send_uart_noti_to_modem(MODEM_CTRL_UART_AP);
#endif
	} else {
		ret = _s2mu005_muic_sel_path(muic_data, S2MU005_PATH_UART_CP);
#if IS_ENABLED(CONFIG_CP_UART_NOTI)
		send_uart_noti_to_modem(MODEM_CTRL_UART_CP);
#endif
	}

	return ret;
}

static int _s2mu005_muic_control_rid_adc(struct s2mu005_muic_data *muic_data, bool enable)
{
    int ret = 0;
	u8 reg_val = 0;

	pr_info("%s (%s)\n", __func__, enable ? "Enable" : "Disable");
	reg_val = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_MUIC_CTRL2);

	if (enable)
		reg_val &= ~MUIC_CTRL2_ADC_OFF_MASK;
	else
		reg_val |= MUIC_CTRL2_ADC_OFF_MASK;

	ret = s2mu005_i2c_write_byte(muic_data->i2c, S2MU005_REG_MUIC_CTRL2, reg_val);

	return 0;
}

static int s2mu005_muic_get_vbus_voltage(struct s2mu005_muic_data *muic_data)
{
	int val = 0;

	if(_s2mu005_muic_get_vbus_state(muic_data))
		val = 5;

	return val;
}
static void s2mu005_muic_get_detect_info(struct s2mu005_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	struct i2c_client *i2c = muic_data->i2c;

	muic_data->reg[DEVICE_TYPE1] = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE1);
	muic_data->reg[DEVICE_TYPE2] = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE2);
	muic_data->reg[DEVICE_TYPE3] = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_TYPE3);
	muic_data->reg[DEVICE_APPLE] = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_DEVICE_APPLE);
	muic_data->reg[CHG_TYPE] = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_CHG_TYPE);
    muic_data->reg[ADC] = _s2mu005_muic_get_rid_adc(muic_data);
	muic_data->vbvolt = muic_pdata->vbvolt = _s2mu005_muic_get_vbus_state(muic_data);
	muic_data->adc = muic_pdata->adc = muic_data->reg[ADC];

	pr_info("dev[1:0x%02x, 2:0x%02x, 3:0x%02x]\n", muic_data->reg[DEVICE_TYPE1],
		muic_data->reg[DEVICE_TYPE2], muic_data->reg[DEVICE_TYPE3]);
	pr_info("adc:0x%02x, vbvolt:0x%02x, apple:0x%02x\n",
		muic_data->adc, muic_data->vbvolt, muic_data->reg[DEVICE_APPLE]);
	pr_info("chg_type:0x%02x\n", muic_data->reg[CHG_TYPE]);
}

static void s2mu005_muic_handle_attached_dev(struct s2mu005_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC &&
			muic_data->new_dev != muic_pdata->attached_dev) {
		muic_manager_set_legacy_dev(muic_pdata->muic_if, muic_data->new_dev);
		muic_core_handle_attach(muic_pdata, muic_data->new_dev,
				muic_pdata->adc, muic_pdata->vbvolt);
	}
}

static void _s2mu005_muic_resend_jig_type(struct s2mu005_muic_data *muic_data)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	if (muic_data->vbvolt
		&& (muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_ON_MUIC)) {
		muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_VB_MUIC;
		s2mu005_muic_handle_attached_dev(muic_data);
	} else if (!muic_data->vbvolt
				&& (muic_pdata->attached_dev == ATTACHED_DEV_JIG_UART_ON_VB_MUIC)) {
		muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
		s2mu005_muic_handle_attached_dev(muic_data);
	}
}

static void s2mu005_muic_detect_dev_ccic
    (struct s2mu005_muic_data *muic_data, muic_attached_dev_t new_dev)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;
	int adc = 0, vbvolt = 0;

	pr_info("%s new dev(%s)\n", __func__, dev_to_str(new_dev));

	if (muic_pdata->attached_dev == new_dev) {
		pr_err("%s: Skip to handle duplicated type\n", __func__);
		return;
	}

	if (new_dev == ATTACHED_DEV_NONE_MUIC) {
		if (muic_pdata->attached_dev == ATTACHED_DEV_USB_MUIC)
			muic_core_handle_detach(muic_data->pdata);
		/* Detach from CCIC */
		else if (muic_core_get_ccic_cable_state(muic_data->pdata) == false) {
			pr_err("%s: Skip to detach legacy type\n", __func__);
			return;
		}
	} else {
		/* Attach from CCIC */
		pr_info("%s DETECTED\n", dev_to_str(new_dev));

		switch (new_dev) {
		case ATTACHED_DEV_OTG_MUIC:
			_s2mu005_muic_sel_path(muic_data, S2MU005_PATH_USB);
			break;
		case ATTACHED_DEV_TYPE3_CHARGER_MUIC: /* PD Charger */
			_s2mu005_muic_sel_path(muic_data, S2MU005_PATH_OPEN);
			return;
		default:
			break;
		}
	}

	if (new_dev != ATTACHED_DEV_NONE_MUIC) {
		adc = _s2mu005_muic_get_rid_adc(muic_data);
		vbvolt = _s2mu005_muic_get_vbus_state(muic_data);
		muic_core_handle_attach(muic_data->pdata, new_dev, adc, !!vbvolt);
	} else if (new_dev == ATTACHED_DEV_NONE_MUIC) {
		muic_core_handle_detach(muic_data->pdata);
	}
}

static int s2mu005_muic_detect_dev_bc1p2(struct s2mu005_muic_data *muic_data)
{
	struct muic_interface_t *muic_if = (struct muic_interface_t *)muic_data->if_data;

	muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;

	/* Attached */
	switch (muic_data->reg[DEVICE_TYPE1]) {
	case DEV_TYPE1_CDP:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_CDP_MUIC;
			pr_info("USB_CDP DETECTED\n");
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
			/*
			 * USB BC 1.2 certification :
			 * 100ms delay between attach dev,
			 * and path setting / start chg.
			 */
			msleep(100);
#endif
		}
		break;
	case DEV_TYPE1_USB:
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("USB DETECTED\n");
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
			msleep(100);
#endif
		}
		break;
	case DEV_TYPE1_DEDICATED_CHG:
	case DEV_TYPE1_DEDICATED_CHG2:
	case DEV_TYPE1_CHG_TYPES:
		if (muic_data->vbvolt) {
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
			/*
			* USB BC 1.2 certification : 100ms delay between attach dev,
			* and path setting / start chg. DP 0.6V when DCP is detected.
			 */
		msleep(100);
			muic_data->is_dcp = true;
			s2mu005_muic_dp_0_6V(muic_data, true);
			msleep(100);
#endif
			muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
			pr_info("DEDICATED CHARGER DETECTED\n");
			muic_if->is_dcp_charger = true;
		}
		break;
#if TBD
	case DEV_TYPE1_USB_OTG:
		new_dev = ATTACHED_DEV_OTG_MUIC;
		if (vmid == 0x4) {
			pr_info("USB_OTG VMID DETECTED[%d]\n", vmid);
			vbvolt = 1;
		}
		break;
#endif
	case DEV_TYPE1_T1_T2_CHG:
		if (muic_data->vbvolt) {
			/* 200K, 442K should be checkef */
#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_MUIC_S2MU005_DISCHARGING_WA)
			muic_data->new_dev = ATTACHED_DEV_CARKIT_MUIC;
#else
			if (muic_data->adc == ADC_CEA936ATYPE2_CHG)
				muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
			else
				muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("T1_T2 CHARGER DETECTED\n");
#endif
		} else {
			/* W/A, 442k without VB changes to 523K (JIG_UART_OFF)
			* To prevent to keep sleep mode
			*/
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
			pr_info("442K->523K JIG_USB_OFF DETECTED\n");
		}
		break;
	default:
		break;
	}

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC &&
			muic_data->new_dev != ATTACHED_DEV_NONE_MUIC)
		goto detect_done;

	switch (muic_data->reg[DEVICE_TYPE2]) {
	case DEV_TYPE2_SDP_1P8S:
		if (muic_data->vbvolt) {
			struct muic_platform_data *pdata = muic_data->pdata;
			if (pdata->dcd_timeout)
				muic_data->new_dev = ATTACHED_DEV_TIMEOUT_OPEN_MUIC;
			else
				muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("SDP_1P8S DETECTED\n");
		}
		break;
	case DEV_TYPE2_JIG_UART_OFF:
		if (muic_data->is_otg_test) {
#if TBD
			mdelay(1000);
			sc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_SC_STATUS2);
			vmid = sc & 0x7;
			pr_info("%s  vmid : 0x%x\n", __func__, vmid);
			if (vmid == 0x4) {
				pr_info("OTG_TEST vmid = %d\n", vmid);
				vbvolt = 1;
			}
#endif
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		} else if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
		} else {
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
		}
		break;
	case DEV_TYPE2_JIG_UART_ON:
		muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
		break;
	case DEV_TYPE2_JIG_USB_OFF:
		if (!muic_data->vbvolt)
			break;
		muic_data->new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
		break;
	case DEV_TYPE2_JIG_USB_ON:
		if (!muic_data->vbvolt)
			break;
		muic_data->new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
		break;
	default:
		break;
	}

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC &&
			muic_data->new_dev != ATTACHED_DEV_NONE_MUIC)
		goto detect_done;

	switch (muic_data->reg[DEVICE_TYPE3]) {
	case DEV_TYPE3_MHL:
		if (muic_data->pdata->is_factory_uart)
			muic_data->new_dev = ATTACHED_DEV_FACTORY_UART_MUIC;
		break;
	default:
		break;
	}

	/* This is for Apple cables */
	if (muic_data->vbvolt && (
				muic_data->reg[DEVICE_APPLE] & DEV_TYPE_APPLE_APPLE_CHG)) {
		muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
		pr_info("%s: APPLE_CHG DETECTED\n", __func__);
	}

	if ((muic_data->reg[CHG_TYPE] & DEV_TYPE_CHG_TYPE) &&
			(muic_data->new_dev == ATTACHED_DEV_UNKNOWN_MUIC)) {
		/* This is workaround for LG USB cable which has 219k ohm ID */
		if (muic_data->adc == ADC_CEA936ATYPE1_CHG || muic_data->adc == ADC_JIG_USB_OFF) {
			muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
			pr_info("TYPE1_CHARGER DETECTED (USB)\n");
		} else {
			muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
			pr_info("TYPE3_CHARGER DETECTED\n");
		}
	}

	if (muic_data->reg[DEVICE_TYPE2] & DEV_TYPE2_AV ||
			muic_data->reg[DEVICE_TYPE3] & DEV_TYPE3_AV_WITH_VBUS) {
		if (muic_data->vbvolt)
			muic_data->new_dev = ATTACHED_DEV_DESKDOCK_VB_MUIC;
		else
			muic_data->new_dev = ATTACHED_DEV_DESKDOCK_MUIC;
	}

detect_done:
	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC)
		return S2MU005_DETECT_DONE;
	else
		return S2MU005_DETECT_NONE;
}

static int s2mu005_muic_detect_dev_adc(struct s2mu005_muic_data *muic_data)
{
	muic_data->new_dev = ATTACHED_DEV_UNKNOWN_MUIC;

	if (muic_data->adc & ADC_CONVERSION_ERR_MASK) {
		pr_info("%s ADC conversion error! (adc=%#x)\n", __func__, muic_data->adc);
		return S2MU005_DETECT_NONE;
	}

	if (muic_data->new_dev == ATTACHED_DEV_UNKNOWN_MUIC) {
		switch (muic_data->adc) {
#if IS_ENABLED(CONFIG_MUIC_INCOMPATIBLE_VZW)
		case ADC_INCOMPATIBLE_VZW:
			muic_data->new_dev = ATTACHED_DEV_VZW_INCOMPATIBLE_MUIC;
			break;
#endif
		case ADC_CHARGING_CABLE:
			break;
		case ADC_CEA936ATYPE1_CHG: /*200k ohm */
			/* This is workaround for LG USB cable which has 219k ohm ID */
			if (muic_data->vbvolt) {
#if defined(CONFIG_SEC_FACTORY) && defined(CONFIG_MUIC_S2MU005_DISCHARGING_WA)
				muic_data->new_dev = ATTACHED_DEV_CARKIT_MUIC;
#else
				muic_data->new_dev = ATTACHED_DEV_USB_MUIC;
				pr_info("%s: ADC TYPE1 CHARGER DETECTED(USB)\n", __func__);
#endif
			}
			break;
		case ADC_CEA936ATYPE2_CHG:
			if (muic_data->vbvolt) {
				muic_data->new_dev = ATTACHED_DEV_TA_MUIC;
				pr_info("%s: ADC TYPE2 CHARGER DETECTED(TA)\n", __func__);
			}
			break;
		case ADC_JIG_USB_OFF: /* 255k */
			if (!muic_data->vbvolt)
				break;
			muic_data->new_dev = ATTACHED_DEV_JIG_USB_OFF_MUIC;
			break;
		case ADC_JIG_USB_ON:
			if (!muic_data->vbvolt)
				break;
			muic_data->new_dev = ATTACHED_DEV_JIG_USB_ON_MUIC;
			break;
		case ADC_JIG_UART_OFF:
			if (muic_data->is_otg_test) {
#if TBD
				mdelay(1000);
				sc = s2mu005_i2c_read_byte(i2c, S2MU005_REG_SC_STATUS2);
				vmid = sc & 0x7;
				pr_info("%s: vmid : 0x%x\n", __func__, vmid);
				if (vmid == 0x4) {
					pr_info("%s: ADC OTG_TEST DETECTED, vmid = %d\n",
							__func__, vmid);
					vbvolt = 1;
				}
#endif
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
			} else if (muic_data->vbvolt) {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_VB_MUIC;
			} else {
				muic_data->new_dev = ATTACHED_DEV_JIG_UART_OFF_MUIC;
			}
			break;
		case ADC_JIG_UART_ON:
			muic_data->new_dev = ATTACHED_DEV_JIG_UART_ON_MUIC;
			break;
		case ADC_DESKDOCK:
			if (muic_data->vbvolt)
				muic_data->new_dev = ATTACHED_DEV_DESKDOCK_VB_MUIC;
			else
				muic_data->new_dev = ATTACHED_DEV_DESKDOCK_MUIC;
			break;
#if IS_ENABLED(CONFIG_MUIC_S2MU005_SUPPORT_HMT)
		case ADC_HMT:
			muic_data->new_dev = ATTACHED_DEV_HMT_MUIC;
			break;
#endif
		case ADC_OPEN:
			break;
		default:
			pr_info("%s: unsupported ADC(0x%02x)\n", __func__, muic_data->adc);
			break;
		}
	}

	if ((muic_data->new_dev == ATTACHED_DEV_UNKNOWN_MUIC)
			&& (muic_data->adc != ADC_OPEN)) {
		if (muic_data->vbvolt) {
			muic_data->new_dev = ATTACHED_DEV_UNDEFINED_CHARGING_MUIC;
			pr_info("UNDEFINED VB DETECTED\n");
		}
	}

	if (muic_data->new_dev != ATTACHED_DEV_UNKNOWN_MUIC)
		return S2MU005_DETECT_DONE;
	else
		return S2MU005_DETECT_NONE;
}

/*
 * Interface Functions
 */
static int s2mu005_if_set_jig_ctrl_on(void *mdata)
{
    struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;

    return _s2mu005_muic_set_jig_on(muic_data);
}

static int s2mu005_if_get_adc(void *mdata)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);

	ret = _s2mu005_muic_get_rid_adc(muic_data);

	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}
static int s2mu005_if_com_to_open(void *mdata)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);

	ret = _s2mu005_muic_sel_path(muic_data, S2MU005_PATH_OPEN);
	if (ret)
		pr_err("%s set_com_open err\n", __func__);

	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu005_if_com_to_usb(void *mdata)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);

	ret = _s2mu005_muic_sel_path(muic_data, S2MU005_PATH_USB);
	if (ret)
		pr_err("%s set_com_usb err\n", __func__);

	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu005_if_com_to_uart(void *mdata)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;
	int ret = 0;

	mutex_lock(&muic_data->switch_mutex);

	ret = _s2mu005_muic_com_to_uart(muic_data);
	if (ret)
		pr_err("%s set_com_uart err\n", __func__);

	mutex_unlock(&muic_data->switch_mutex);

	return ret;
}

static int s2mu005_if_get_vbus(void *mdata)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;

	return _s2mu005_muic_get_vbus_state(muic_data);
}

static void s2mu005_if_set_cable_state(void *mdata, muic_attached_dev_t new_dev)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	s2mu005_muic_detect_dev_ccic(muic_data, new_dev);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);
}

static int s2mu005_if_set_gpio_uart_sel(void *mdata, int uart_path)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;

	return s2mu005_set_gpio_uart_sel(muic_data, uart_path);
}

static int s2mu005_if_control_rid_adc(void *mdata, bool enable)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;

	return _s2mu005_muic_control_rid_adc(muic_data, enable);
}

static int s2mu005_if_get_vbus_voltage(void *mdata)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;

	return s2mu005_muic_get_vbus_voltage(muic_data);
}

#if IS_ENABLED(CONFIG_MUIC_SYSFS)
#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
static int s2mu005_if_set_otg_reg(void *mdata, bool enable)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;

	return set_otg_reg(muic_data, enable);
}
#endif

static int s2mu005_if_show_register(void *mdata, char *mesg)
{
#if IS_ENABLED(CONFIG_MUIC_DEBUG)
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;

	if (mesg != NULL)
		s2mu005_read_reg_dump(muic_data, mesg);
	pr_info("%s %s\n", __func__, mesg);
#endif
	return 0;
}
#endif

#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
static void s2mu005_if_set_water_det(void *mdata, bool val)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;
	pr_info("%s en : %d\n", __func__, (int)val);
	muic_data->is_hiccup_mode = val;
	_s2mu005_muic_sel_path(muic_data, S2MU005_PATH_OPEN);
}

static int s2mu005_if_set_hiccup_mode(void *mdata, bool val)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;
	muic_data->is_hiccup_mode = val;
	return 0;
}

static int s2mu005_if_get_hiccup_mode(void *mdata)
{
	struct s2mu005_muic_data *muic_data = (struct s2mu005_muic_data *)mdata;
	return muic_data->is_hiccup_mode;
}
#endif

/*
 * ISR Functions
 */
static irqreturn_t s2mu005_muic_attach_isr(int irq, void *data)
{
	struct s2mu005_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;
	int det_ret = S2MU005_DETECT_NONE;

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_pdata = muic_data->pdata;
	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

#if IS_ENABLED(CONFIG_S2MU005_MUIC_DEBUG)
	muic_data->irq_attach_cnt++;
#endif	/* CONFIG_S2MU005_MUIC_DEBUG */

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	if (MUIC_IS_ATTACHED(muic_pdata->attached_dev)) {
		pr_err("%s Cable type already was attached\n", __func__);
		goto skip;
	}

	s2mu005_muic_get_detect_info(muic_data);

	det_ret = s2mu005_muic_detect_dev_bc1p2(muic_data);
	if (det_ret == S2MU005_DETECT_DONE)
		goto done;
	else if (det_ret == S2MU005_DETECT_SKIP)
		goto skip;

	det_ret = s2mu005_muic_detect_dev_adc(muic_data);

done:
	s2mu005_muic_handle_attached_dev(muic_data);
skip:
	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	return IRQ_HANDLED;
}

static irqreturn_t s2mu005_muic_detach_isr(int irq, void *data)
{
	struct s2mu005_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;
	struct muic_interface_t *muic_if;

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_pdata = muic_data->pdata;
	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_if = muic_data->if_data;
	if (muic_if == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	pr_info("%s start(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	if (MUIC_IS_ATTACHED(muic_pdata->attached_dev) == false) {
		pr_err("%s Cable type already was detached\n", __func__);
		goto done;
	}

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	if (muic_if->opmode == OPMODE_CCIC) {
		if (muic_core_get_ccic_cable_state(muic_data->pdata))
			pr_err("%s Skip to handle ccic dev type\n", __func__);
		else if (muic_pdata->attached_dev == ATTACHED_DEV_USB_MUIC) {
			if (muic_if->ccic->ccic_evt_attached == MUIC_CCIC_NOTI_DETACH)
				muic_core_handle_detach(muic_data->pdata);
		} else
			muic_core_handle_detach(muic_data->pdata);
	} else if (muic_if->opmode == OPMODE_MUIC) {
		muic_core_handle_detach(muic_data->pdata);
	}

done:
	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	pr_info("%s done(%s)\n", __func__, dev_to_str(muic_pdata->attached_dev));

	return IRQ_HANDLED;
}

static irqreturn_t s2mu005_muic_vbus_isr(int irq, void *data)
{
	struct s2mu005_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata ;
#if IS_ENABLED(CONFIG_SEC_FACTORY)
	int vmid = 0;
	int sc;
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

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	msleep(20);
	muic_pdata->vbvolt = muic_data->vbvolt = _s2mu005_muic_get_vbus_state(muic_data);
#if IS_ENABLED(CONFIG_SEC_FACTORY)
    /* For OTGGTEST : Notify VBUS high if VMID is valid */
    if (muic_pdata->is_otg_test && muic_pdata->vbvolt == 0) {
            mdelay(1000);
            sc = s2mu005_i2c_read_byte(muic_data->i2c, S2MU005_REG_SC_STATUS2);
            vmid = sc & 0x7;
            pr_info("%s  vmid : 0x%x\n", __func__, vmid);
            if (vmid == 0x4) {
                pr_info("OTG_TEST vmid = %d\n", vmid);
                muic_pdata->vbvolt = muic_data->vbvolt = 1;
            }
    }
#endif

	pr_info("%s Vbus_INT(%s), Vbus(%s), Type(%s)\n", __func__,
			(irq == muic_data->irq_vbus_on ? "On" : "Off"), (muic_data->vbvolt ? "High" : "Low"),
			dev_to_str(muic_pdata->attached_dev));

#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	if (irq == muic_data->irq_vbus_off)
		s2mu005_muic_handle_vbus_off(muic_data);
#endif

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	vbus_notifier_handle(muic_data->vbvolt ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */
	_s2mu005_muic_resend_jig_type(muic_data);

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

static irqreturn_t s2mu005_muic_adc_change_isr(int irq, void *data)
{
	struct s2mu005_muic_data *muic_data = data;
	struct muic_platform_data *muic_pdata;
	struct muic_interface_t *muic_if;

	if (muic_data == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_pdata = muic_data->pdata;
	if (muic_pdata == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	muic_if = muic_data->if_data;
	if (muic_if == NULL) {
		pr_err("%s data NULL\n", __func__);
		return IRQ_NONE;
	}

	mutex_lock(&muic_data->muic_mutex);
	wake_lock(&muic_data->wake_lock);

	muic_data->adc = muic_pdata->adc = _s2mu005_muic_get_rid_adc(muic_data);
	muic_data->vbvolt = _s2mu005_muic_get_vbus_state(muic_data);
	pr_info("%s adc(%#x), vbvolt(%#x)\n",  __func__, muic_data->adc, muic_data->vbvolt);

     if (muic_if->opmode == OPMODE_MUIC) {
         s2mu005_muic_detect_dev_adc(muic_data);
         s2mu005_muic_handle_attached_dev(muic_data);
    }

	wake_unlock(&muic_data->wake_lock);
	mutex_unlock(&muic_data->muic_mutex);

	return IRQ_HANDLED;
}

/*
 * Init Functions
 */
 static int s2mu005_muic_reg_init(struct s2mu005_muic_data *muic_data)
{
	struct i2c_client *i2c = muic_data->i2c;
	int ret;
	u8 ctrl1 = 0, ctrl2;
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	u8 reg_val = 0, tmp_val = 0, vldo_set = 0;
#endif

	s2mu005_muic_get_detect_info(muic_data);

	if (muic_data->reg[DEVICE_TYPE1] & DEV_TYPE1_USB_TYPES) {
		ret = _s2mu005_muic_sel_path(muic_data, S2MU005_PATH_USB);
		if (ret < 0)
			pr_err("%s usb type detect err(%d)\n", __func__, ret);
	}

	ret = s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_CTRL1, CTRL_MASK);
	if (ret < 0)
		pr_err("%s failed to write ctrl(%d)\n", __func__, ret);
	ctrl1 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_CTRL1);
        ctrl2 = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_CTRL2);
	pr_info("%s CTRL1:0x%02x, CTRL2:%#x\n", __func__, ctrl1, ctrl2);

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

	reg_val = s2mu005_i2c_read_byte(i2c, S2MU005_REG_MUIC_TIMER_SET3);
	reg_val &= ~(MUIC_TIMER_SET3_DCDTMRSET_MASK);
	reg_val |= (MUIC_TIMER_SET3_DCDTMRSET_600MS);
	s2mu005_i2c_write_byte(i2c, S2MU005_REG_MUIC_TIMER_SET3, reg_val);
	pr_info("%s MUIC_TIMER_SET3(%#x)\n", __func__, reg_val);

	reg_val = s2mu005_i2c_read_byte(i2c, 0xB9);
	reg_val &= ~(0xf);
	reg_val |= (0x9);
	s2mu005_i2c_write_byte(i2c, 0xB9, reg_val);
	pr_info("%s 0xB9(%#x)\n", __func__, reg_val);
#endif

	return ret;
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

static void s2mu005_muic_init_drvdata(struct s2mu005_muic_data *muic_data,
		struct s2mu005_dev *s2mu005, struct platform_device *pdev,
		struct s2mu005_platform_data *mfd_pdata)
{
	/* save platfom data for gpio control functions */
	muic_data->s2mu005_dev = s2mu005;
	muic_data->dev = &pdev->dev;
	muic_data->i2c = s2mu005->i2c;
	muic_data->mfd_pdata = mfd_pdata;
	pr_info("factory_uart: %d\n", muic_data->pdata->is_factory_uart);
	muic_data->is_factory_start = false;
	muic_data->attached_dev = ATTACHED_DEV_UNKNOWN_MUIC;
	muic_data->is_usb_ready = false;

#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	muic_data->is_dcp = false;
#endif
#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
	muic_data->is_hiccup_mode = false;
#endif

}

static void s2mu005_muic_init_interface(struct s2mu005_muic_data *muic_data,
		struct muic_interface_t *muic_if)
{
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s, muic_if : 0x%p, muic_data : 0x%p\n",
        __func__, muic_if, muic_data);

	muic_if->muic_data = (void *)muic_data;


    muic_if->set_com_to_open = s2mu005_if_com_to_open;
    muic_if->set_switch_to_usb = s2mu005_if_com_to_usb;
    muic_if->set_switch_to_uart = s2mu005_if_com_to_uart;
    muic_if->get_vbus = s2mu005_if_get_vbus;
    muic_if->get_adc = s2mu005_if_get_adc;
    muic_if->set_jig_ctrl_on = s2mu005_if_set_jig_ctrl_on;
    muic_if->set_cable_state = s2mu005_if_set_cable_state;
	muic_if->set_gpio_uart_sel = s2mu005_if_set_gpio_uart_sel;
	muic_if->control_rid_adc = s2mu005_if_control_rid_adc;
	muic_if->get_vbus_voltage = s2mu005_if_get_vbus_voltage;
#if TBD
    muic_if->set_com_to_otg = s2mu005_if_com_to_usb;
	muic_if->set_dcd_rescan = s2mu005_if_cable_recheck;
	muic_if->check_usb_killer = s2mu005_if_check_usb_killer;
	muic_if->bcd_rescan = s2mu005_if_set_bcd_rescan_reg;
#endif
#if IS_ENABLED(CONFIG_MUIC_SYSFS)
	muic_if->show_register = s2mu005_if_show_register;
#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
	muic_if->set_otg_reg = s2mu005_if_set_otg_reg;
#endif
#endif
#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
	muic_if->set_water_detect = s2mu005_if_set_water_det;
	muic_if->set_hiccup_mode = s2mu005_if_set_hiccup_mode;
	muic_if->get_hiccup_mode = s2mu005_if_get_hiccup_mode;
#if !IS_ENABLED(CONFIG_SEC_FACTORY)
	muic_if->set_water_detect_from_boot = s2mu005_if_set_water_det;
#endif
#endif
	muic_data->if_data = muic_if;
	muic_pdata->muic_if = muic_if;
}

static int s2mu005_muic_irq_init(struct s2mu005_muic_data *muic_data)
{
	int ret = 0;

	if (muic_data->mfd_pdata && (muic_data->mfd_pdata->irq_base > 0)) {
		int irq_base = muic_data->mfd_pdata->irq_base;

		/* request MUIC IRQ */
		muic_data->irq_attach = irq_base + S2MU005_MUIC_IRQ1_ATTATCH;
		REQUEST_IRQ(muic_data->irq_attach, muic_data, "muic-attach",
				&s2mu005_muic_attach_isr);

		muic_data->irq_detach = irq_base + S2MU005_MUIC_IRQ1_DETACH;
		REQUEST_IRQ(muic_data->irq_detach, muic_data, "muic-detach",
				&s2mu005_muic_detach_isr);

		muic_data->irq_vbus_on = irq_base + S2MU005_MUIC_IRQ2_VBUS_ON;
		REQUEST_IRQ(muic_data->irq_vbus_on, muic_data, "muic-vbus_on",
				&s2mu005_muic_vbus_isr);

		muic_data->irq_adc_change = irq_base + S2MU005_MUIC_IRQ2_ADC_CHANGE;
		REQUEST_IRQ(muic_data->irq_adc_change, muic_data, "muic-adc_change",
				&s2mu005_muic_adc_change_isr);

		muic_data->irq_vbus_off = irq_base + S2MU005_MUIC_IRQ2_VBUS_OFF;
		REQUEST_IRQ(muic_data->irq_vbus_off, muic_data, "muic-vbus_off",
				&s2mu005_muic_vbus_isr);
	}

	pr_info("attach(%d), detach(%d), vbus_on(%d), adc(%d), vbus_off(%d)\n",
		muic_data->irq_attach, muic_data->irq_detach,
		muic_data->irq_vbus_on, muic_data->irq_adc_change,
		muic_data->irq_vbus_off);

	return ret;
}

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
    struct muic_platform_data *muic_pdata;
	struct muic_interface_t *muic_if;
	int ret = 0;

	if (unlikely(!mfd_pdata))
		return PTR_ERR(mfd_pdata);

	muic_data = devm_kzalloc(&pdev->dev, sizeof(*muic_data), GFP_KERNEL);
	if (unlikely(!muic_data)) {
		pr_err("failed to allocate driver data\n");
		return PTR_ERR(muic_data);
	}

	if (unlikely(!mfd_pdata)) {
		pr_err("%s failed to get s2mu005 mfd platform data\n", __func__);
		ret = -ENOMEM;
		goto err_kfree1;
	}

	muic_pdata = muic_core_init(muic_data);
 	if (unlikely(!muic_pdata))
		goto err_kfree1;
	muic_data->pdata = muic_pdata;

	muic_if = muic_manager_init(muic_pdata, muic_data);
	if (!muic_if) {
		pr_err("%s failed to init muic manager, ret : 0x%X\n",
				__func__, ret);
		goto err_init_if;
	}

	s2mu005_muic_init_interface(muic_data, muic_if);
	s2mu005_muic_init_drvdata(muic_data, s2mu005, pdev, mfd_pdata);

#if IS_ENABLED(CONFIG_OF)
	ret = of_s2mu005_muic_dt(&pdev->dev, muic_data);
	if (ret < 0)
		pr_err("no muic dt(%d)\n", ret);
#endif /* CONFIG_OF */

	mutex_init(&muic_data->muic_mutex);
	mutex_init(&muic_data->switch_mutex);

	platform_set_drvdata(pdev, muic_data);
	if (muic_data->pdata->init_gpio_cb) {
		ret = muic_data->pdata->init_gpio_cb(muic_data->pdata, get_switch_sel());
		if (ret) {
			pr_err("failed to init gpio(%d)\n", ret);
			goto fail_init_gpio;
		}
	}

#if IS_ENABLED(CONFIG_MUIC_SYSFS)
	ret = muic_sysfs_init(muic_pdata);
	if (ret) {
		pr_err("failed to create sysfs\n");
		goto fail_init_sysfs;
	}
#endif

	ret = s2mu005_init_rev_info(muic_data);
	if (ret) {
		pr_err("failed to get rev info(%d)\n", ret);
		goto fail;
	}

	ret = s2mu005_muic_reg_init(muic_data);
	if (ret) {
		pr_err("failed to init muic(%d)\n", ret);
		goto fail;
	}

	/* For Rustproof */
	muic_data->is_rustproof = muic_data->pdata->rustproof_on;
	if (muic_data->is_rustproof) {
		pr_info("rustproof is enabled\n");
		ret = _s2mu005_muic_sel_path(muic_data, S2MU005_PATH_OPEN);
	}

	if (muic_data->pdata->init_switch_dev_cb)
		muic_data->pdata->init_switch_dev_cb();

	ret = s2mu005_muic_irq_init(muic_data);
	if (ret) {
		pr_err("failed to init irq(%d)\n", ret);
		goto fail_init_irq;
	}

	wake_lock_init(&muic_data->wake_lock, WAKE_LOCK_SUSPEND, "muic_wake");

	ret = set_ctrl_reg(muic_data, CTRL_INT_MASK_SHIFT, false);
#if IS_ENABLED(CONFIG_SEC_FACTORY) && IS_ENABLED(CONFIG_USB_HOST_NOTIFY)
	init_otg_reg(muic_data);
#endif
#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	INIT_DELAYED_WORK(&muic_data->dp_0p6v, s2mu005_muic_dp_0p6v);
#endif

#if IS_ENABLED(CONFIG_S2MU005_MUIC_DEBUG)
	INIT_DELAYED_WORK(&muic_data->debug_dwrok, s2mu005_muic_debug);
#endif

#if IS_ENABLED(CONFIG_VBUS_NOTIFIER)
	/* initial vbus state notification */
	vbus_notifier_handle(muic_data->vbvolt ? STATUS_VBUS_HIGH : STATUS_VBUS_LOW);
#endif /* CONFIG_VBUS_NOTIFIER */

	/* initial cable detection */
	if (muic_if->opmode == OPMODE_MUIC) {
		s2mu005_muic_adc_change_isr(-1, muic_data);
	} else {
		s2mu005_muic_attach_isr(-1, muic_data);
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
    return ret;
}

static int s2mu005_muic_remove(struct platform_device *pdev)
{
	struct s2mu005_muic_data *muic_data = platform_get_drvdata(pdev);

	pr_info("%s\n", __func__);

#if IS_ENABLED(CONFIG_MUIC_SYSFS)
	if (muic_data->pdata != NULL)
		muic_sysfs_deinit(muic_data->pdata);
#endif

	disable_irq_wake(muic_data->i2c->irq);
	s2mu005_muic_free_irqs(muic_data);
	mutex_destroy(&muic_data->muic_mutex);
	mutex_destroy(&muic_data->switch_mutex);
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
	struct muic_platform_data *muic_pdata = muic_data->pdata;

	pr_info("%s\n", __func__);
	muic_data->suspended = false;
	if (muic_pdata->need_to_noti) {
		if (muic_pdata->attached_dev) {
			MUIC_SEND_NOTI_ATTACH(muic_pdata->attached_dev);
		} else {
			MUIC_SEND_NOTI_DETACH(muic_pdata->attached_dev);
		}
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
