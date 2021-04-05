/*
 * s2mu205_charger.c - S2MU205 Charger Driver
 *
 * Copyright (C) 2016 Samsung Electronics Co.Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/mfd/samsung/s2mu205.h>
#include "include/charger/s2mu205_charger.h"
#include <linux/version.h>
#include <linux/sec_batt.h>
#if defined(CONFIG_LEDS_S2MU205_FLASH)
#include <linux/leds-s2mu205.h>
#endif

#define ENABLE 1
#define DISABLE 0

#define IVR_WORK_DELAY 150

static char *s2mu205_supplied_to[] = {
	"battery",
};

static enum power_supply_property s2mu205_charger_props[] = {
};

static enum power_supply_property s2mu205_otg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int s2mu205_get_charging_health(struct s2mu205_charger_data *charger);

static void s2mu205_test_read(struct i2c_client *i2c)
{
	u8 data;
	char str[1016] = {0,};
	int i;

	for (i = S2MU205_CHG_INT1M ; i <= S2MU205_CHG_INT3M ; i++) {
		s2mu205_read_reg(i2c, i, &data);
		sprintf(str+strlen(str), "0x%02x:0x%02x, ", i, data);
	}

	s2mu205_read_reg(i2c, 0x0C, &data);
	sprintf(str+strlen(str), "0x%02x:0x%02x, ", 0x0C, data);

	for (i = S2MU205_CHG_STATUS0 ; i <= S2MU205_CHG_CTRL15 ; i++) {
		s2mu205_read_reg(i2c, i, &data);
		sprintf(str+strlen(str), "0x%02x:0x%02x, ", i, data);
	}

	s2mu205_read_reg(i2c, 0x33, &data);
	sprintf(str+strlen(str), "0x%02x:0x%02x, ", 0x33, data);
	s2mu205_read_reg(i2c, 0x3F, &data);
	sprintf(str+strlen(str), "0x%02x:0x%02x, ", 0x3F, data);
	s2mu205_read_reg(i2c, 0x48, &data);
	sprintf(str+strlen(str), "0x%02x:0x%02x, ", 0x48, data);
	s2mu205_read_reg(i2c, 0x49, &data);
	sprintf(str+strlen(str), "0x%02x:0x%02x, ", 0x49, data);
	s2mu205_read_reg(i2c, 0x4F, &data);
	sprintf(str+strlen(str), "0x%02x:0x%02x, ", 0x4F, data);
	s2mu205_read_reg(i2c, 0xF5, &data);
	sprintf(str+strlen(str), "0x%02x:0x%02x, ", 0xF5, data);
	s2mu205_read_reg(i2c, 0xA7, &data);
	sprintf(str+strlen(str), "0x%02x:0x%02x, ", 0xA7, data);

	pr_err("%s: %s\n", __func__, str);
}

static int s2mu205_charger_otg_control(
		struct s2mu205_charger_data *charger, bool enable)
{
	u8 chg_sts4, chg_ctrl0;
	pr_info("%s: called charger otg control : %s\n", __func__,
			enable ? "ON" : "OFF");

	mutex_lock(&charger->charger_mutex);
#if 0
	if (charger->is_charging) {
		pr_info("%s: Charger is enabled and OTG noti received!!!\n", __func__);
		pr_info("%s: is_charging: %d, otg_on: %d",
				__func__, charger->is_charging, charger->otg_on);
		s2mu205_test_read(charger->i2c);
		goto out;
	}
#endif
	if (charger->otg_on == enable)
		goto out;

	if (!enable) {
		/* CHGIN Vref Soft start 1024us->128us (default) */
		s2mu205_update_reg(charger->i2c, 0x92, 0x0C, 0x0C);

		s2mu205_update_reg(charger->i2c,
				S2MU205_CHG_CTRL0, CHG_MODE, REG_MODE_MASK);
	} else {
		/* OTG OCP level 1.5A */
		s2mu205_update_reg(charger->i2c,
				S2MU205_CHG_CTRL12,
				S2MU205_SET_OTG_OCP_1500mA << SET_OTG_OCP_SHIFT,
				SET_OTG_OCP_MASK);

		/* CHGIN Vref Soft start 128us -> 1024us */
		s2mu205_update_reg(charger->i2c, 0x92, 0x00, 0x0C);

		msleep(30);

		/* OTG mode */
		s2mu205_update_reg(charger->i2c,
				S2MU205_CHG_CTRL0, OTG_BST_MODE, REG_MODE_MASK);

		charger->cable_type = SEC_BATTERY_CABLE_OTG;
	}
	charger->otg_on = enable;

out:
	mutex_unlock(&charger->charger_mutex);

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS4, &chg_sts4);
	s2mu205_read_reg(charger->i2c, S2MU205_CHG_CTRL0, &chg_ctrl0);
	pr_info("%s S2MU205_CHG_STATUS4: 0x%x\n", __func__, chg_sts4);
	pr_info("%s S2MU205_CHG_CTRL0: 0x%x\n", __func__, chg_ctrl0);

	power_supply_changed(charger->psy_otg);
	return enable;
}

static void s2mu205_enable_charger_switch(
	struct s2mu205_charger_data *charger, int onoff)
{
	mutex_lock(&charger->charger_mutex);

	if (factory_mode) {
		pr_info("%s: Factory Mode Skip CHG_EN Control\n", __func__);
		goto out;
	}

	if (charger->otg_on) {
		pr_info("[DEBUG] %s: skipped set(%d) : OTG is on\n", __func__, onoff);
		charger->is_charging = false;
		goto out;
	}

	if (onoff > 0) {
		pr_info("[DEBUG]%s: turn on charger\n", __func__);
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL0, CHG_MODE, REG_MODE_MASK);

		/* timer fault set 16hr(max) */
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL14,
				S2MU205_FC_CHG_TIMER_16hr << TIME_FC_CHG_SHIFT,
				TIME_FC_CHG_MASK);

	} else {
		pr_info("[DEBUG] %s: turn off charger\n", __func__);
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL0, BUCK_MODE, REG_MODE_MASK);
	}
out:
	mutex_unlock(&charger->charger_mutex);
}

static void s2mu205_set_buck(
	struct s2mu205_charger_data *charger, int enable) {

	if (enable) {
		pr_info("[DEBUG]%s: set buck on\n", __func__);
		s2mu205_enable_charger_switch(charger, charger->is_charging);
	} else {
		pr_info("[DEBUG]%s: set buck off (charger off mode)\n", __func__);
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL0, CHARGER_OFF_MODE, REG_MODE_MASK);
	}
}

static void s2mu205_set_regulation_vsys(
	struct s2mu205_charger_data *charger, int vsys)
{
	u8 data;

	pr_info("[DEBUG]%s: VSYS regulation %d\n", __func__, vsys);
	if (vsys <= 3700)
		data = 0;
	else if (vsys > 3700 && vsys <= 4400)
		data = (vsys - 3700) / 100;
	else
		data = 0x07;

	s2mu205_update_reg(charger->i2c,
		S2MU205_CHG_CTRL6, data << SET_VF_VSYS_SHIFT, SET_VF_VSYS_MASK);
}

static void s2mu205_set_regulation_voltage(
		struct s2mu205_charger_data *charger, int float_voltage)
{
	u8 data;

	if (factory_mode)
		return;

	pr_info("[DEBUG]%s: float_voltage %d\n", __func__, float_voltage);
	if (float_voltage <= 3900)
		data = 0;
	else if (float_voltage > 3900 && float_voltage <= 4535)
		data = (float_voltage - 3900) / 5;
	else
		data = 0x7F;

	s2mu205_update_reg(charger->i2c,
			S2MU205_CHG_CTRL3, data << SET_VF_VBAT_SHIFT, SET_VF_VBAT_MASK);
}

static int s2mu205_get_regulation_voltage(struct s2mu205_charger_data *charger)
{
	u8 reg_data = 0;
	int float_voltage;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_CTRL3, &reg_data);
	reg_data &= 0x7F;
	float_voltage = reg_data * 5 + 3900;
	pr_debug("%s: battery cv reg : 0x%x, float voltage val : %d\n",
			__func__, reg_data, float_voltage);

	return float_voltage;
}

static void s2mu205_set_input_current_limit(
		struct s2mu205_charger_data *charger, int charging_current)
{
	u8 data;

	if (factory_mode)
		return;

	if (charging_current <= 100)
		data = 0x02;
	else if (charging_current > 100 && charging_current <= 3000)
		data = (charging_current - 50) / 25;
	else
		data = 0x62;

	s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL1,
			data << SET_IIN_CHGIN_ILIM_SHIFT, SET_IIN_CHGIN_ILIM_MASK);

	pr_info("[DEBUG]%s: current  %d, 0x%x\n", __func__, charging_current, data);

#if EN_TEST_READ
	s2mu205_test_read(charger->i2c);
#endif
}

static int s2mu205_get_input_current_limit(struct s2mu205_charger_data *charger)
{
	u8 data;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_CTRL1, &data);
	if (data < 0)
		return data;

	data = data & SET_IIN_CHGIN_ILIM_MASK;
	if (data > 0x76) {
		pr_err("%s: Invalid current limit in register\n", __func__);
		data = 0x76;
	}
	return  data * 25 + 50;
}

//TO DO need to set wcin current
static void s2mu205_set_fast_charging_current(
		struct s2mu205_charger_data *charger, int charging_current)
{
	u8 data;

	if (factory_mode)
		return;

	if (charging_current <= 100)
		data = 0x01;
	else if (charging_current > 100 && charging_current <= 3200)
		data = (charging_current / 50) - 1;
	else
		data = 0x3D;

	s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL5,
			data << SET_FAST_CHG_CUR_SHIFT, SET_FAST_CHG_CUR_MASK);

	pr_info("[DEBUG]%s: current  %d, 0x%02x\n", __func__, charging_current, data);

#if EN_TEST_READ
	s2mu205_test_read(charger->i2c);
#endif
}

static int s2mu205_get_fast_charging_current(
		struct s2mu205_charger_data *charger)
{
	u8 data;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_CTRL5, &data);
	if (data < 0)
		return data;

	data = data & SET_FAST_CHG_CUR_MASK;

	if (data > 0x3F) {
		pr_err("%s: Invalid fast charging current in register\n", __func__);
		data = 0x3F;
	}
	return (data + 1) * 50;
}

static void s2mu205_set_topoff_current(
		struct s2mu205_charger_data *charger,
		int eoc_1st_2nd, int current_limit)
{
	int data;

	pr_info("[DEBUG]%s: current  %d\n", __func__, current_limit);
	if (current_limit <= 100)
		data = 0;
	else if (current_limit > 100 && current_limit <= 475)
		data = (current_limit - 100) / 25;
	else
		data = 0x0F;

	switch (eoc_1st_2nd) {
	case 1:
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL10,
				data << SET_1ST_TOP_OFF_SHIFT, SET_1ST_TOP_OFF_MASK);
		break;
	case 2:
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL10,
				data << SET_2ND_TOP_OFF_SHIFT, SET_2ND_TOP_OFF_MASK);
		break;
	default:
		break;
	}
}

static int s2mu205_get_topoff_setting(
		struct s2mu205_charger_data *charger)
{
	u8 data;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_CTRL10, &data);
	if (data < 0)
		return data;

	data = data & SET_1ST_TOP_OFF_MASK;

	if (data > 0x0F)
		data = 0x0F;
	return data * 25 + 100;
}

static bool s2mu205_chg_init(struct s2mu205_charger_data *charger)
{
	u8 temp;

	/* Set default regulation voltage 4.35v
	s2mu205_update_reg(charger->i2c,
			S2MU205_CHG_CTRL5, 0x5A << SET_VF_VBAT_SHIFT, SET_VF_VBAT_MASK);
*/
	/* ensure OTG start-up */
	s2mu205_update_reg(charger->i2c, 0x90, 0x00, 0x01 << 2);

	s2mu205_update_reg(charger->i2c, 0x8F, 0x00, 0x0C);

	/* To prevent entering watchdog issue case we set WDT_CLR to not clear before enabling WDT */
	s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL13, 0x00, WDT_CLR_MASK);

	s2mu205_update_reg(charger->i2c, 0x8F, 0x00, 0x3 << 2);

	/* set watchdog timer to 80 seconds */
	s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL13,
			S2MU205_WDT_TIMER_80s << SET_WDT_TIME_SHIFT,
			SET_WDT_TIME_MASK);

	/* enable Watchdog timer and only Charging off */
	s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL13,
			ENABLE << EN_WDT_SHIFT | DISABLE << EN_WDT_AP_RESET_SHIFT,
			EN_WDT_MASK | EN_WDT_AP_RESET_MASK);

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_CTRL13, &temp);
	pr_info("%s : for WDT setting S2MU205_CHG_CTRL13 : 0x%x\n", __func__, temp);

	/* TODO : need to check previous history */
	/* Type-C reset off */
	//s2mu205_update_reg(charger->i2c, 0xEC, 0x00, 0x80);

	/* MRSTB 1s set */
	//s2mu205_write_reg(charger->i2c, 0xE5, 0x08);

#ifndef CONFIG_SEC_FACTORY
	/* Prevent sudden power off when detected water is dried */
	if (!factory_mode) {
		pr_info ("%s Normal booting\n", __func__);
		s2mu205_update_reg(charger->i2c, 0x95, 0x00, 0x04);
	}
#endif
	return true;
}

static int s2mu205_get_charging_status(
		struct s2mu205_charger_data *charger)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int ret;
	u8 chg_sts0, chg_sts1;
	union power_supply_propval value;
	struct power_supply *psy;

	ret = s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS0, &chg_sts0);
	ret = s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS1, &chg_sts1);

	psy = power_supply_get_by_name(charger->pdata->fuelgauge_name);
	if (!psy)
		return -EINVAL;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_CURRENT_AVG, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	if (ret < 0)
		return status;

	/* TODO : Discharging Condition? */
	if (chg_sts0 & 0x04 || chg_sts0 & 0x02) {
		pr_info("%s: full check curr_avg(%d), topoff_curr(%d)\n",
				__func__, value.intval, charger->topoff_current);
		if (value.intval < charger->topoff_current)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else if ((chg_sts0 & CHGIN_STATUS_MASK) == 0x50 || (chg_sts0 & CHGIN_STATUS_MASK) == 0x30)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;

#if EN_TEST_READ
	s2mu205_test_read(charger->i2c);
#endif
	return status;
}

static int s2mu205_get_vbus_status(
		struct s2mu205_charger_data *charger)
{
	int status = 0;
	int ret;
	u8 chg_sts;

	ret = s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS0, &chg_sts);


	if ((chg_sts & CHGIN_STATUS_MASK) == 0x50 || (chg_sts & CHGIN_STATUS_MASK) == 0x30)
		status = 1;
	else
		status = 0;

	return status;
}

static int s2mu205_get_otg_status(
		struct s2mu205_charger_data *charger)
{
	int status = 0;
	int ret;
	u8 otg_sts;

	ret = s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS4, &otg_sts);


	if ((otg_sts & 0x0c) == 0x0c)
		status = 1;
	else
		status = 0;

	return status;
}

static int s2mu205_get_charge_type(struct s2mu205_charger_data *charger)
{
	int status = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	u8 ret;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS5, &ret);
	if (ret < 0)
		pr_err("%s fail\n", __func__);

	switch ((ret & BAT_STATUS_MASK) >> BAT_STATUS_SHIFT) {
	case 0x5:
		status = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case 0x4:
		/* pre-charge mode */
		status = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	}

	return status;
}

static bool s2mu205_get_batt_present(struct s2mu205_charger_data *charger)
{
	u8 ret;

	/* Enable charger CLK for BAT_ID check */
/*	s2mu205_update_reg(charger->i2c, 0xF1, 0x01, 0x01);
	msleep(30);
*/
	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS5, &ret);
	if (ret < 0)
		return false;

	//s2mu205_update_reg(charger->i2c, 0xF1, 0x00, 0x01);

	return (ret & BATID_STATUS_MASK) ? true : false;
}

static void s2mu205_wdt_clear(struct s2mu205_charger_data *charger)
{
	u8 reg_data, chg_fault_status;

	/* watchdog kick */
	s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL13,
			0x1 << WDT_CLR_SHIFT, WDT_CLR_MASK);

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS1, &reg_data);
	chg_fault_status = (reg_data & CHG_FAULT_STATUS_MASK) >> CHG_FAULT_STATUS_SHIFT;

	if ((chg_fault_status == CHG_STATUS_WD_SUSPEND) ||
			(chg_fault_status == CHG_STATUS_WD_RST)) {
		pr_info("%s: watchdog error status(0x%02x,%d)\n",
				__func__, reg_data, chg_fault_status);
		if (charger->is_charging) {
			pr_info("%s: toggle charger\n", __func__);
			s2mu205_enable_charger_switch(charger, false);
			s2mu205_enable_charger_switch(charger, true);
		}
	}
}

static int s2mu205_get_charging_health(struct s2mu205_charger_data *charger)
{

	u8 ret;
	union power_supply_propval value;
	struct power_supply *psy;

	if (charger->is_charging)
		s2mu205_wdt_clear(charger);

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS0, &ret);
	pr_info("[DEBUG] %s: S2MU205_CHG_STATUS0 0x%x\n", __func__, ret);
	if (ret < 0)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	ret = (ret & (CHGIN_STATUS_MASK)) >> CHGIN_STATUS_SHIFT;

	switch (ret) {
	case 0x03:
	case 0x05:
		charger->ovp = false;
		charger->unhealth_cnt = 0;
		return POWER_SUPPLY_HEALTH_GOOD;
	default:
		break;
	}

	charger->unhealth_cnt++;
	if (charger->unhealth_cnt < HEALTH_DEBOUNCE_CNT)
		return POWER_SUPPLY_HEALTH_GOOD;

	/* 005 need to check ovp & health count */
	charger->unhealth_cnt = HEALTH_DEBOUNCE_CNT;
	if (charger->ovp)
		return POWER_SUPPLY_HEALTH_OVERVOLTAGE;

	psy = power_supply_get_by_name("battery");
	if (!psy)
		return -EINVAL;
	ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	if (ret < 0)
		pr_err("%s: Fail to execute property\n", __func__);

	if (value.intval == SEC_BATTERY_CABLE_PDIC)
		return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	else
		return POWER_SUPPLY_HEALTH_GOOD;

#if EN_TEST_READ
	s2mu205_test_read(charger->i2c);
#endif
}

static int s2mu205_chg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int chg_curr, aicr;
	struct s2mu205_charger_data *charger =
		power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->is_charging ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s2mu205_get_charging_status(charger);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = s2mu205_get_charging_health(charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = s2mu205_get_input_current_limit(charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (charger->charging_current) {
			aicr = s2mu205_get_input_current_limit(charger);
			chg_curr = s2mu205_get_fast_charging_current(charger);
			val->intval = MINVAL(aicr, chg_curr);
		} else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = s2mu205_get_fast_charging_current(charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_FULL:
		val->intval = s2mu205_get_topoff_setting(charger);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = s2mu205_get_charge_type(charger);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = s2mu205_get_regulation_voltage(charger);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = s2mu205_get_batt_present(charger);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = charger->is_charging;
		break;
	case POWER_SUPPLY_PROP_VBUS_CHECK:
		val->intval = s2mu205_get_vbus_status(charger);
		break;
	case POWER_SUPPLY_PROP_OTG_CHECK:
		val->intval = s2mu205_get_otg_status(charger);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s2mu205_chg_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct s2mu205_charger_data *charger = power_supply_get_drvdata(psy);
	enum power_supply_ext_property ext_psp = psp;
	int buck_state = ENABLE;
	union power_supply_propval value;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		charger->status = val->intval;
		break;
		/* val->intval : type */
	case POWER_SUPPLY_PROP_ONLINE:
		charger->cable_type = val->intval;
		charger->ivr_on = false;
		if (charger->cable_type != SEC_BATTERY_CABLE_OTG) {
			if (charger->cable_type == SEC_BATTERY_CABLE_NONE ||
			charger->cable_type == SEC_BATTERY_CABLE_UNKNOWN) {
				pr_err("[DEBUG]%s:[BATT] Type Battery\n", __func__);
				value.intval = 0;
			} else {
				value.intval = 1;
			}

			psy = power_supply_get_by_name(charger->pdata->fuelgauge_name);
			if (!psy)
				return -EINVAL;
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_ENERGY_AVG, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);

			if (charger->cable_type == SEC_BATTERY_CABLE_NONE) {
				/* At cable removal enable IVR IRQ if it was disabled */
				if (charger->irq_ivr_enabled == 0) {
					u8 reg_data;

					charger->irq_ivr_enabled = 1;
					/* Unmask IRQ */
					s2mu205_update_reg(charger->i2c, S2MU205_CHG_INT2M,
						0 << IVR_M_SHIFT, IVR_M_MASK);
					enable_irq(charger->irq_ivr);
					s2mu205_read_reg(charger->i2c, S2MU205_CHG_INT2M, &reg_data);
					pr_info("%s : enable ivr : 0x%x\n", __func__, reg_data);
				}
			}
		} else {
			pr_info("[DEBUG]%s: Cable Type OTG \n", __func__);
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		{
			int input_current = val->intval;

			s2mu205_set_input_current_limit(charger, input_current);
			charger->input_current = input_current;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pr_info("[DEBUG] %s: is_charging %d\n", __func__, charger->is_charging);
		charger->charging_current = val->intval;
		/* set charging current */
		s2mu205_set_fast_charging_current(charger, charger->charging_current);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		break;
	case POWER_SUPPLY_PROP_CURRENT_FULL:
		charger->topoff_current = val->intval;
		if (charger->pdata->chg_eoc_dualpath) {
			s2mu205_set_topoff_current(charger, 1, val->intval);
			s2mu205_set_topoff_current(charger, 2, 100);
		} else
			s2mu205_set_topoff_current(charger, 1, val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pr_info("[DEBUG]%s: float voltage(%d)\n", __func__, val->intval);
		charger->pdata->chg_float_voltage = val->intval;
		s2mu205_set_regulation_voltage(charger,
				charger->pdata->chg_float_voltage);
		break;
	case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
		s2mu205_charger_otg_control(charger, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		charger->charge_mode = val->intval;

		psy = power_supply_get_by_name("battery");
		if (!psy)
			return -EINVAL;
		ret = power_supply_get_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		if (value.intval != SEC_BATTERY_CABLE_OTG) {
			switch (charger->charge_mode) {
			case SEC_BAT_CHG_MODE_BUCK_OFF:
				buck_state = DISABLE;
			case SEC_BAT_CHG_MODE_CHARGING_OFF:
				charger->is_charging = false;
				break;
			case SEC_BAT_CHG_MODE_CHARGING:
				charger->is_charging = true;
				break;
			}

			if (buck_state)
				s2mu205_enable_charger_switch(charger, charger->is_charging);
			else
				s2mu205_set_buck(charger, buck_state);

			value.intval = charger->is_charging;

			psy = power_supply_get_by_name(charger->pdata->fuelgauge_name);
			if (!psy)
				return -EINVAL;
			ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGING_ENABLED, &value);
			if (ret < 0)
				pr_err("%s: Fail to execute property\n", __func__);
		} else {
			pr_info("[DEBUG]%s: SKIP CHARGING CONTROL while OTG(%d)\n",
					__func__, value.intval);
		}
		break;
#ifndef CONFIG_SEC_FACTORY
	case POWER_SUPPLY_PROP_FACTORY_MODE:
		/* prevent sudden power off when detected water is dried */
		if (val->intval) {
			pr_info("%s : 523K, 301K, 255K\n", __func__);
			s2mu205_update_reg(charger->i2c, 0x95, 0x04, 0x04);
		} else {
			pr_info("%s : 619K, OPEN\n", __func__);
			s2mu205_update_reg(charger->i2c, 0x95, 0x00, 0x04);
		}
		break;
#endif
	case POWER_SUPPLY_PROP_ENERGY_NOW: /* Factory Mode Scenario */
		if (val->intval) {
			pr_info("%s: Factory Mode Setting, 523K/301K\n", __func__);
#if defined(CONFIG_LEDS_S2MU205_FLASH)
			/* FLED driver TA only mode set, 0x5C[7:6] -> 0x02*/
			s2mu205_set_operation_mode(S2MU205_FACTORY);
#endif
			/* Factory Mode Setting Sequence */
			s2mu205_set_regulation_vsys(charger, 4300);
			s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL6, 0x80, 0x80);
			s2mu205_update_reg(charger->i2c, 0x96, 0x0, 0x1);
			s2mu205_update_reg(charger->i2c, 0xA7, 0x0, 0x1F);
			s2mu205_update_reg(charger->i2c, 0xE5, 0x8, 0xF);
			s2mu205_set_input_current_limit(charger, 2000);

			value.intval = SEC_BAT_FGSRC_SWITCHING_OFF;
			psy_do_property("s2mu205-fuelgauge", set,
				POWER_SUPPLY_EXT_PROP_INBAT_VOLTAGE_FGSRC_SWITCHING, value);

 		} else {
			pr_info("%s: Factory Mode Release, 619K\n", __func__);
#if defined(CONFIG_LEDS_S2MU205_FLASH)
			/* FLED driver Auto control mode set, 0x5C[7:6] -> 0x00*/
			s2mu205_set_operation_mode(S2MU205_NORMAL);
#endif

			s2mu205_update_reg(charger->i2c, 0x2A, 0x40, 0xC0);

			/* Factory Mode Setting Sequence */
			s2mu205_update_reg(charger->i2c, 0xA7, 0x10, 0x1F);
			s2mu205_update_reg(charger->i2c, 0x96, 0x1, 0x1);
			s2mu205_update_reg(charger->i2c, 0xE5, 0xE, 0xF);
			
			s2mu205_set_input_current_limit(charger, 2000);
			s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL6, 0x0, 0x80);
			s2mu205_set_regulation_vsys(charger, 4400);

			value.intval = SEC_BAT_FGSRC_SWITCHING_ON;
			psy_do_property("s2mu205-fuelgauge", set,
				POWER_SUPPLY_EXT_PROP_INBAT_VOLTAGE_FGSRC_SWITCHING, value);
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION: /* Bypass Mode Scenario */
		if (val->intval) {
			pr_info("%s: Disable VBUS UVLO\n", __func__);
			s2mu205_update_reg(charger->i2c, 0x32, 0x8, 0x8);
			msleep(100);
		}
		break;
	case POWER_SUPPLY_PROP_AUTHENTIC: /* Bypass Mode Scenario */
		if (val->intval) {
			pr_info("%s: Bypass Mode\n", __func__);

			/*
			 * Charger/muic interrupt can occur by entering Bypass mode
			 * Disable all interrupt mask for testing current measure.
			 */
			value.intval = true;
			psy_do_property("muic-manager", set,
					POWER_SUPPLY_EXT_PROP_CURRENT_MEASURE, value);

			/* Enter Bypass */
			s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL0, 0x10, 0x30);
			s2mu205_write_reg(charger->i2c, 0x70, 0x0);
			s2mu205_update_reg(charger->i2c, 0x8D, 0x1, 0x1);
			s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL0, 0x30, 0x30);
			s2mu205_update_reg(charger->i2c, 0xE5, 0xF, 0xF);
			s2mu205_update_reg(charger->i2c, 0xEF, 0x0, 0x1);
			s2mu205_update_reg(charger->i2c, 0xEA, 0x80, 0x80);
			s2mu205_update_reg(charger->i2c, 0x6D, 0x80, 0x80);

			psy_do_property("s2mu205-usbpd", set,
					POWER_SUPPLY_PROP_AUTHENTIC, value);

			pr_info("%s complete %d\n", __func__, __LINE__);
		} else {
			pr_info("%s: Exit Bypass Mode, Set Off\n", __func__);
			s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL0, 0x00, REG_MODE_MASK);
		}
		break;
	case POWER_SUPPLY_PROP_FUELGAUGE_RESET:
/*
		s2mu205_update_reg(charger->i2c, 0xE3, 0x03 << 6, 0x03 << 6);
		msleep(1000);
		s2mu205_update_reg(charger->i2c, 0xE3, 0x00 << 6, 0x03 << 6);
		msleep(50);
		pr_info("%s: reset fuelgauge when surge occur!\n", __func__);
*/
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
      {
		 u8 temp = 0;
       		if (!factory_mode) {
       				s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS3, &temp);
       				if (temp & IVR_STATUS) {
               				pr_info("%s: IVR Start\n", __func__);
               				wake_lock(&charger->ivr_wake_lock);
               				/* Mask IRQ */
               				s2mu205_update_reg(charger->i2c,
                        				       S2MU205_CHG_INT2M, 1 << IVR_M_SHIFT, IVR_M_MASK);
               				queue_delayed_work(charger->charger_wqueue, &charger->ivr_work,
                        				       msecs_to_jiffies(IVR_WORK_DELAY));
       				}
			}

		break;
	}
	case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
		switch (ext_psp) {
		case POWER_SUPPLY_EXT_PROP_FACTORY_VOLTAGE_REGULATION:
			/* enable EN_JIG_AP */
			pr_info("%s: factory voltage regulation (%d)\n", __func__, val->intval);
			s2mu205_set_regulation_vsys(charger, val->intval);
			s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL6, 0x80, 0x80);
			break;
		case POWER_SUPPLY_EXT_PROP_CURRENT_MEASURE:	/* From Nomal Mode to Bypass Mode with keystring on run-time */
			if (val->intval) {
				pr_info("%s: Bypass Mode for Current Measure\n", __func__);

				/*
				 * Charger/muic interrupt can occur by entering Bypass mode
				 * Disable all interrupt mask for testing current measure.
				 */
				value.intval = true;
				psy_do_property("muic-manager", set,
					POWER_SUPPLY_EXT_PROP_CURRENT_MEASURE, value);

				/* VBUS UVLO disable */
				s2mu205_update_reg(charger->i2c, 0x32, 0xC0, 0xC0);
				/* T_DET_CHGIN_TO_BAT */
				s2mu205_update_reg(charger->i2c, 0x32, 0x0C, 0x0C);

				/* Bypass Mode Setting Sequence */
				s2mu205_update_reg(charger->i2c, 0x8D, 0x1, 0x1);
				s2mu205_write_reg(charger->i2c, 0x70, 0x0);
				s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL0, 0x30, 0x30);

				msleep(100);

				/* QBAT off for prevent SMPL when detach cable */
				s2mu205_update_reg(charger->i2c, 0x2A, 0xC0, 0xC0);
				s2mu205_update_reg(charger->i2c, 0x96, 0x0, 0x1);
				s2mu205_update_reg(charger->i2c, 0x31, 0x0, 0x3);

				s2mu205_update_reg(charger->i2c, 0xE5, 0x8, 0xF);

			} else {
				pr_info("%s: Exit Bypass Mode for Current Measure\n", __func__);
				s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL0, 0x00, REG_MODE_MASK);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_SET_UVLO:
		if (val->intval)
			s2mu205_update_reg(charger->i2c, 0x90, 0x00, 0x03);
		else
			s2mu205_update_reg(charger->i2c, 0x90, 0x02, 0x03);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s2mu205_otg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct s2mu205_charger_data *charger = power_supply_get_drvdata(psy);
	u8 reg;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->otg_on;
		break;
	case POWER_SUPPLY_PROP_CHARGE_POWERED_OTG_CONTROL:
		s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS4, &reg);
		pr_info("%s: S2MU205_CHG_STATUS4 : 0x%X\n", __func__, reg);
		if ((reg & 0xC0) == 0x80)
			val->intval = 1;
		else
			val->intval = 0;
		s2mu205_read_reg(charger->i2c, S2MU205_CHG_CTRL0, &reg);
		pr_info("%s: S2MU205_CHG_CTRL0 : 0x%X\n", __func__, reg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int s2mu205_otg_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct s2mu205_charger_data *charger =  power_supply_get_drvdata(psy);
	union power_supply_propval value;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		value.intval = val->intval;
		pr_info("%s: OTG %s\n", __func__, value.intval > 0 ? "ON" : "OFF");

		psy = power_supply_get_by_name(charger->pdata->charger_name);
		if (!psy)
			return -EINVAL;
		ret = power_supply_set_property(psy, POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, &value);
		if (ret < 0)
			pr_err("%s: Fail to execute property\n", __func__);

		power_supply_changed(charger->psy_otg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void s2mu205_charger_otg_vbus_work(struct work_struct *work)
{
	struct s2mu205_charger_data *charger = container_of(work,
			struct s2mu205_charger_data,
			otg_vbus_work.work);

	s2mu205_write_reg(charger->i2c, S2MU205_CHG_CTRL11, 0x16);
}

#if EN_BAT_DET_IRQ
/* s2mu205 interrupt service routine */
static irqreturn_t s2mu205_det_bat_isr(int irq, void *data)
{
	struct s2mu205_charger_data *charger = data;
	u8 val;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS5, &val);
	if ((val & BATID_STATUS_MASK) == 0) {
		s2mu205_enable_charger_switch(charger, 0);
		pr_err("charger-off if battery removed\n");
	}
	return IRQ_HANDLED;
}
#endif

static irqreturn_t s2mu205_done_isr(int irq, void *data)
{
	struct s2mu205_charger_data *charger = data;
	u8 val;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS0, &val);
	pr_info("%s , %02x\n", __func__, val);
	if (val & (DONE_STATUS_MASK)) {
		pr_err("add self chg done\n");
		/* add chg done code here */
	}
	return IRQ_HANDLED;
}

static irqreturn_t s2mu205_chg_isr(int irq, void *data)
{
	struct s2mu205_charger_data *charger = data;
	u8 val;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS0, &val);
	pr_info("%s , %02x\n", __func__, val);
	return IRQ_HANDLED;
}


static irqreturn_t s2mu205_chg_fault_isr(int irq, void *data)
{
	struct s2mu205_charger_data *charger = data;
//	union power_supply_propval value;
	u8 val;
	u8 fault;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS1, &val);
	pr_info("%s , %02x\n", __func__, val);

	fault = (val & CHG_FAULT_STATUS_MASK) >> CHG_FAULT_STATUS_SHIFT;

	/* TODO : need to check previous history */
/*
	if (fault == CHG_STATUS_WD_SUSPEND || fault == CHG_STATUS_WD_RST) {
		value.intval = 1;
		pr_info("%s, reset USBPD\n", __func__);
		psy_do_property("s2mu205-usbpd", set,
					POWER_SUPPLY_PROP_USBPD_RESET, value);
	}
*/

	return IRQ_HANDLED;
}

static irqreturn_t s2mu205_ovp_isr(int irq, void *data)
{
	struct s2mu205_charger_data *charger = data;
	u8 val;

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS0, &val);
	pr_info("%s ovp %02x\n", __func__, val);

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS1, &val);
	pr_info("%s ovp1 %02x\n", __func__, val);

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS2, &val);
	pr_info("%s ovp2 %02x\n", __func__, val);

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS3, &val);
	pr_info("%s ovp3 %02x\n", __func__, val);

	s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS4, &val);
	pr_info("%s ovp4 %02x\n", __func__, val);

	return IRQ_HANDLED;
}

static void reduce_input_current(struct s2mu205_charger_data *charger)
{
	int old_input_current, new_input_current;
	int data;

	old_input_current = s2mu205_get_input_current_limit(charger);
	new_input_current = (old_input_current > MINIMUM_INPUT_CURRENT + REDUCE_CURRENT_STEP) ?
		(old_input_current - REDUCE_CURRENT_STEP) : MINIMUM_INPUT_CURRENT;

	if (old_input_current <= new_input_current) {
		pr_info("%s: Same or less new input current:(%d, %d, %d)\n", __func__,
			old_input_current, new_input_current, charger->input_current);
	} else {
		pr_info("%s: input currents:(%d, %d, %d)\n", __func__,
			old_input_current, new_input_current, charger->input_current);

		data = (new_input_current - 50) / 25;
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_CTRL1,
			data << SET_IIN_CHGIN_ILIM_SHIFT, SET_IIN_CHGIN_ILIM_MASK);

		charger->input_current = s2mu205_get_input_current_limit(charger);
	}
	charger->ivr_on = true;
}

static void s2mu205_ivr_irq_work(struct work_struct *work)
{
	struct s2mu205_charger_data *charger = container_of(work,
				struct s2mu205_charger_data, ivr_work.work);
	u8 ivr_state;
	int ret;
	int ivr_cnt = 0;

	pr_info("%s:\n", __func__);

	if (charger->cable_type == SEC_BATTERY_CABLE_NONE) {
		u8 ivr_mask;

		pr_info("%s : skip\n", __func__);
		s2mu205_read_reg(charger->i2c, S2MU205_CHG_INT2M, &ivr_mask);
		if (ivr_mask & 0x02) {
			/* Unmask IRQ */
			s2mu205_update_reg(charger->i2c, S2MU205_CHG_INT2M,
					0 << IVR_M_SHIFT, IVR_M_MASK);
		}
		wake_unlock(&charger->ivr_wake_lock);
		return;
	}

	ret = s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS3, &ivr_state);
	if (ret < 0) {
		wake_unlock(&charger->ivr_wake_lock);
		pr_info("%s : I2C error\n", __func__);
		/* Unmask IRQ */
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_INT2M,
				0 << IVR_M_SHIFT, IVR_M_MASK);
		return;
	}
	pr_info("%s: ivr_status 0x13:0x%02x\n", __func__, ivr_state);

	mutex_lock(&charger->charger_mutex);

	while ((ivr_state & IVR_STATUS) &&
			charger->cable_type != SEC_BATTERY_CABLE_NONE) {

		if (s2mu205_read_reg(charger->i2c, S2MU205_CHG_STATUS3, &ivr_state)) {
			pr_err("%s: Error reading S2MU205_CHG_STATUS3\n", __func__);
			break;
		}
		pr_info("%s: ivr_status 0x13:0x%02x\n", __func__, ivr_state);

		if (++ivr_cnt >= 2) {
			reduce_input_current(charger);
			ivr_cnt = 0;
		}
		msleep(50);

		if (!(ivr_state & IVR_STATUS)) {
			pr_info("%s: EXIT IVR WORK: check value (0x13:0x%02x, input current:%d)\n", __func__,
				ivr_state, charger->input_current);
			break;
		}

		if (s2mu205_get_input_current_limit(charger) <= MINIMUM_INPUT_CURRENT)
			break;
	}

	if (charger->ivr_on) {
		union power_supply_propval value;

		if ((charger->irq_ivr_enabled == 1) &&
			(charger->input_current <= MINIMUM_INPUT_CURRENT)) {
			/* Disable IVR IRQ, can't reduce current any more */
			u8 reg_data;

			charger->irq_ivr_enabled = 0;
			disable_irq_nosync(charger->irq_ivr);
			/* Mask IRQ */
			s2mu205_update_reg(charger->i2c,
				    S2MU205_CHG_INT2M, 1 << IVR_M_SHIFT, IVR_M_MASK);
			s2mu205_read_reg(charger->i2c, S2MU205_CHG_INT2M, &reg_data);
			pr_info("%s : disable ivr : 0x%x\n", __func__, reg_data);
		}

		value.intval = s2mu205_get_input_current_limit(charger);
		psy_do_property("battery", set,
				POWER_SUPPLY_EXT_PROP_AICL_CURRENT, value);
	}

	if (charger->irq_ivr_enabled == 1) {
		/* Unmask IRQ */
		s2mu205_update_reg(charger->i2c, S2MU205_CHG_INT2M,
			0 << IVR_M_SHIFT, IVR_M_MASK);
	}
	mutex_unlock(&charger->charger_mutex);
	wake_unlock(&charger->ivr_wake_lock);
}

static irqreturn_t s2mu205_ivr_isr(int irq, void *data)
{
	struct s2mu205_charger_data *charger = data;

	pr_info("%s: Start\n", __func__);
	wake_lock(&charger->ivr_wake_lock);
	/* Mask IRQ */
	s2mu205_update_reg(charger->i2c,
		    S2MU205_CHG_INT2M, 1 << IVR_M_SHIFT, IVR_M_MASK);
	queue_delayed_work(charger->charger_wqueue, &charger->ivr_work,
		msecs_to_jiffies(IVR_WORK_DELAY));
	pr_info("%s: irq(%d)\n", __func__, irq);

	return IRQ_HANDLED;
}

static int s2mu205_charger_parse_dt(struct device *dev,
		struct s2mu205_charger_platform_data *pdata)
{
	struct device_node *np = of_find_node_by_name(NULL, "s2mu205-charger");
	int ret = 0;

	if (!np) {
		pr_err("%s np NULL(s2mu205-charger)\n", __func__);
	} else {
		ret = of_property_read_u32(np, "battery,chg_switching_freq",
				&pdata->chg_switching_freq);
		if (ret < 0)
			pr_info("%s: Charger switching FRQ is Empty\n", __func__);
	}

	np = of_find_node_by_name(NULL, "battery");
	if (!np) {
		pr_err("%s np NULL\n", __func__);
	} else {
		ret = of_property_read_string(np,
				"battery,fuelgauge_name",
				(char const **)&pdata->fuelgauge_name);
		if (ret < 0)
			pr_info("%s: Fuel-gauge name is Empty\n", __func__);

		ret = of_property_read_u32(np, "battery,chg_float_voltage",
				&pdata->chg_float_voltage);
		if (ret) {
			pr_info("%s: battery,chg_float_voltage is Empty\n", __func__);
			pdata->chg_float_voltage = 4200;
		}
		pr_info("%s: battery,chg_float_voltage is %d\n",
				__func__, pdata->chg_float_voltage);

		pdata->chg_eoc_dualpath = of_property_read_bool(np,
				"battery,chg_eoc_dualpath");
	}

	np = of_find_node_by_name(NULL, "sec-multi-charger");
	if (!np) {
		pr_err("%s np NULL(sec-multi-charger)\n", __func__);
	} else {
		ret = of_property_read_string(np,
				"charger,main_charger",
				(char const **)&pdata->charger_name);
		if (ret < 0)
			pr_info("%s: Charger name is Empty\n", __func__);
	}
#if 0
		p = of_get_property(np, "battery,input_current_limit", &len);
		if (!p)
			return 1;

		len = len / sizeof(u32);

		pdata->charging_current =
			kzalloc(sizeof(sec_charging_current_t) * len,
					GFP_KERNEL);

		for (i = 0; i < len; i++) {
			ret = of_property_read_u32_index(np,
					"battery,input_current_limit", i,
					&pdata->charging_current[i].input_current_limit);
			if (ret)
				pr_info("%s : Input_current_limit is Empty\n",
						__func__);

			ret = of_property_read_u32_index(np,
					"battery,fast_charging_current", i,
					&pdata->charging_current[i].fast_charging_current);
			if (ret)
				pr_info("%s : Fast charging current is Empty\n",
						__func__);

			ret = of_property_read_u32_index(np,
					"battery,full_check_current", i,
					&pdata->charging_current[i].full_check_current);
			if (ret)
				pr_info("%s : Full check current is Empty\n",
						__func__);
		}
	}
#endif

	pr_info("%s DT file parsed succesfully, %d\n", __func__, ret);
	return ret;
}

/* if need to set s2mu205 pdata */
static const struct of_device_id s2mu205_charger_match_table[] = {
	{ .compatible = "samsung,s2mu205-charger",},
	{},
};

static int s2mu205_charger_probe(struct platform_device *pdev)
{
	struct s2mu205_dev *s2mu205 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu205_platform_data *pdata = dev_get_platdata(s2mu205->dev);
	struct s2mu205_charger_data *charger;
	struct power_supply_config psy_cfg = {};
	int ret = 0;

	pr_info("%s:[BATT] S2MU205 Charger driver probe\n", __func__);
	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	mutex_init(&charger->charger_mutex);
	charger->otg_on = false;
	charger->ivr_on = false;
	charger->cable_type = SEC_BATTERY_CABLE_NONE;

	charger->dev = &pdev->dev;
	charger->i2c = s2mu205->i2c;

	charger->pdata = devm_kzalloc(&pdev->dev, sizeof(*(charger->pdata)),
			GFP_KERNEL);
	if (!charger->pdata) {
		ret = -ENOMEM;
		goto err_parse_dt_nomem;
	}
	ret = s2mu205_charger_parse_dt(&pdev->dev, charger->pdata);
	if (ret < 0)
		goto err_parse_dt;

	platform_set_drvdata(pdev, charger);

	if (charger->pdata->charger_name == NULL)
		charger->pdata->charger_name = "s2mu205-charger";
	if (charger->pdata->fuelgauge_name == NULL)
		charger->pdata->fuelgauge_name = "s2mu205-fuelgauge";

	charger->psy_chg_desc.name           = charger->pdata->charger_name;
	charger->psy_chg_desc.type           = POWER_SUPPLY_TYPE_UNKNOWN;
	charger->psy_chg_desc.get_property   = s2mu205_chg_get_property;
	charger->psy_chg_desc.set_property   = s2mu205_chg_set_property;
	charger->psy_chg_desc.properties     = s2mu205_charger_props;
	charger->psy_chg_desc.num_properties = ARRAY_SIZE(s2mu205_charger_props);

	charger->psy_otg_desc.name           = "otg";
	charger->psy_otg_desc.type           = POWER_SUPPLY_TYPE_OTG;
	charger->psy_otg_desc.get_property   = s2mu205_otg_get_property;
	charger->psy_otg_desc.set_property   = s2mu205_otg_set_property;
	charger->psy_otg_desc.properties     = s2mu205_otg_props;
	charger->psy_otg_desc.num_properties = ARRAY_SIZE(s2mu205_otg_props);

	s2mu205_chg_init(charger);
	charger->input_current = s2mu205_get_input_current_limit(charger);
	charger->charging_current = s2mu205_get_fast_charging_current(charger);

	psy_cfg.drv_data = charger;
	psy_cfg.supplied_to = s2mu205_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(s2mu205_supplied_to);

	charger->psy_chg = power_supply_register(&pdev->dev, &charger->psy_chg_desc, &psy_cfg);
	if (IS_ERR(charger->psy_chg)) {
		pr_err("%s: Failed to Register psy_chg\n", __func__);
		ret = PTR_ERR(charger->psy_chg);
		goto err_power_supply_register;
	}

	charger->psy_otg = power_supply_register(&pdev->dev, &charger->psy_otg_desc, &psy_cfg);
	if (IS_ERR(charger->psy_otg)) {
		pr_err("%s: Failed to Register psy_otg\n", __func__);
		ret = PTR_ERR(charger->psy_otg);
		goto err_power_supply_register_otg;
	}

	charger->charger_wqueue = create_singlethread_workqueue("charger-wq");
	if (!charger->charger_wqueue) {
		pr_info("%s: failed to create wq.\n", __func__);
		ret = -ESRCH;
		goto err_create_wq;
	}

	wake_lock_init(&charger->ivr_wake_lock, WAKE_LOCK_SUSPEND,
		"charger-ivr");
	INIT_DELAYED_WORK(&charger->otg_vbus_work, s2mu205_charger_otg_vbus_work);
	INIT_DELAYED_WORK(&charger->ivr_work, s2mu205_ivr_irq_work);

	/*
	 * irq request
	 * if you need to add irq , please refer below code.
	 */
	charger->irq_sys = pdata->irq_base + S2MU205_CHG1_IRQ_SYS;
	ret = request_threaded_irq(charger->irq_sys, NULL,
			s2mu205_ovp_isr, 0, "sys-irq", charger);
	if (ret < 0) {
		dev_err(s2mu205->dev, "%s: Fail to request SYS in IRQ: %d: %d\n",
				__func__, charger->irq_sys, ret);
		goto err_reg_irq;
	}

#if EN_BAT_DET_IRQ
	charger->irq_det_bat = pdata->irq_base + S2MU205_CHG2_IRQ_DET_BAT;
	ret = request_threaded_irq(charger->irq_det_bat, NULL,
			s2mu205_det_bat_isr, 0, "det_bat-irq", charger);
	if (ret < 0) {
		dev_err(s2mu205->dev, "%s: Fail to request DET_BAT in IRQ: %d: %d\n",
				__func__, charger->irq_det_bat, ret);
		goto err_reg_irq;
	}
#endif

#if EN_CHG1_IRQ_CHGIN
	charger->irq_chgin = pdata->irq_base + S2MU205_CHG1_IRQ_CHGIN;
	ret = request_threaded_irq(charger->irq_chgin, NULL,
			s2mu205_chg_isr, 0, "chgin-irq", charger);
	if (ret < 0) {
		dev_err(s2mu205->dev, "%s: Fail to request CHGIN in IRQ: %d: %d\n",
				__func__, charger->irq_chgin, ret);
		goto err_reg_irq;
	}
#endif

	charger->irq_rst = pdata->irq_base + S2MU205_CHG1_IRQ_CHG_RSTART;
	ret = request_threaded_irq(charger->irq_rst, NULL,
			s2mu205_chg_isr, 0, "restart-irq", charger);
	if (ret < 0) {
		dev_err(s2mu205->dev, "%s: Fail to request CHG_Restart in IRQ: %d: %d\n",
				__func__, charger->irq_rst, ret);
		goto err_reg_irq;
	}

	charger->irq_done = pdata->irq_base + S2MU205_CHG1_IRQ_DONE;
	ret = request_threaded_irq(charger->irq_done, NULL,
			s2mu205_done_isr, 0, "done-irq", charger);
	if (ret < 0) {
		dev_err(s2mu205->dev, "%s: Fail to request DONE in IRQ: %d: %d\n",
				__func__, charger->irq_done, ret);
		goto err_reg_irq;
	}

	charger->irq_chg_fault = pdata->irq_base + S2MU205_CHG1_IRQ_CHG_Fault;
	ret = request_threaded_irq(charger->irq_chg_fault, NULL,
			s2mu205_chg_fault_isr, 0, "chg_fault-irq", charger);
	if (ret < 0) {
		dev_err(s2mu205->dev, "%s: Fail to request CHG_Fault in IRQ: %d: %d\n",
				__func__, charger->irq_chg_fault, ret);
		goto err_reg_irq;
	}

	charger->irq_ivr = pdata->irq_base + S2MU205_CHG2_IRQ_IVR;
	charger->irq_ivr_enabled = 1;
	ret = request_threaded_irq(charger->irq_ivr, NULL,
			s2mu205_ivr_isr, 0, "ivr-irq", charger);
	if (ret < 0) {
		pr_err("%s: Fail to request IVR_INT IRQ: %d: %d\n",
					__func__, charger->irq_ivr, ret);
		charger->irq_ivr_enabled = -1;
		goto err_reg_irq;
	}

	/* Do max charging by freq. change, when duty is max */
	s2mu205_update_reg(charger->i2c, 0x7A, 0x1 << 4, 0x1 << 4);
#if EN_TEST_READ
	s2mu205_test_read(charger->i2c);
#endif
	pr_info("%s:[BATT] S2MU205 charger driver loaded OK\n", __func__);

	return 0;

err_reg_irq:
	destroy_workqueue(charger->charger_wqueue);
err_create_wq:
	power_supply_unregister(charger->psy_otg);
err_power_supply_register_otg:
	power_supply_unregister(charger->psy_chg);
err_power_supply_register:
err_parse_dt:
err_parse_dt_nomem:
	mutex_destroy(&charger->charger_mutex);
	kfree(charger);
	return ret;
}

static int s2mu205_charger_remove(struct platform_device *pdev)
{
	struct s2mu205_charger_data *charger =
		platform_get_drvdata(pdev);

	power_supply_unregister(charger->psy_chg);
	mutex_destroy(&charger->charger_mutex);
	kfree(charger);
	return 0;
}

#if defined CONFIG_PM
static int s2mu205_charger_suspend(struct device *dev)
{
	return 0;
}

static int s2mu205_charger_resume(struct device *dev)
{
	return 0;
}
#else
#define s2mu205_charger_suspend NULL
#define s2mu205_charger_resume NULL
#endif

static void s2mu205_charger_shutdown(struct device *dev)
{
	pr_info("%s: S2MU205 Charger driver shutdown\n", __func__);
}

static SIMPLE_DEV_PM_OPS(s2mu205_charger_pm_ops, s2mu205_charger_suspend,
		s2mu205_charger_resume);

static struct platform_driver s2mu205_charger_driver = {
	.driver         = {
		.name   = "s2mu205-charger",
		.owner  = THIS_MODULE,
		.of_match_table = s2mu205_charger_match_table,
		.pm     = &s2mu205_charger_pm_ops,
		.shutdown   =   s2mu205_charger_shutdown,
	},
	.probe          = s2mu205_charger_probe,
	.remove     = s2mu205_charger_remove,
};

static int __init s2mu205_charger_init(void)
{
	int ret = 0;
	pr_info("%s start\n", __func__);
	ret = platform_driver_register(&s2mu205_charger_driver);

	return ret;
}
module_init(s2mu205_charger_init);

static void __exit s2mu205_charger_exit(void)
{
	platform_driver_unregister(&s2mu205_charger_driver);
}
module_exit(s2mu205_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Charger driver for S2MU205");
