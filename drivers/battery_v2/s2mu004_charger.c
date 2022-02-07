/*
 * s2mu004_charger.c - S2MU004 Charger Driver
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/mfd/samsung/s2mu004.h>
#include "include/charger/s2mu004_charger.h"
#include <linux/version.h>
#include <linux/sec_batt.h>

#define ENABLE_MIVR 0

#define EN_OVP_IRQ 1
#define EN_IEOC_IRQ 1
#define EN_TOPOFF_IRQ 1
#define EN_RECHG_REQ_IRQ 0
#define EN_TR_IRQ 0
#define EN_MIVR_SW_REGULATION 0
#define EN_BST_IRQ 0
#define EN_BAT_DET_IRQ 0
#if defined(CONFIG_CHARGER_S2MU004_IVR_IRQ)
#define EN_IVR_IRQ 1
#else
#define EN_IVR_IRQ 0
#endif
#define MINVAL(a, b) ((a <= b) ? a : b)
#define EOC_DEBOUNCE_CNT 2
#define HEALTH_DEBOUNCE_CNT 1
#define DEFAULT_CHARGING_CURRENT 500

#define EOC_SLEEP 200
#define EOC_TIMEOUT (EOC_SLEEP * 6)
#ifndef EN_TEST_READ
#define EN_TEST_READ 1
#endif

#define ENABLE 1
#define DISABLE 0

#define IVR_WORK_DELAY 50

static struct device_attribute s2mu004_charger_attrs[] = {
	S2MU004_CHARGER_ATTR(chip_id),
};

static char *s2mu004_supplied_to[] = {
	"s2mu004-charger",
};

static enum power_supply_property s2mu004_charger_props[] = {
};

static enum power_supply_property s2mu004_otg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int s2mu004_get_charging_health(struct s2mu004_charger_data *charger);
static int s2mu004_get_input_current_limit(struct s2mu004_charger_data *charger);
#if EN_IVR_IRQ
static void s2mu004_enable_ivr_irq(struct s2mu004_charger_data *charger);
#endif

static void s2mu004_test_read(struct i2c_client *i2c)
{
	static int reg_list[] = {
		0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
		0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D,
		0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x33, 0x03, 0x71,
		0x74, 0x91, 0x96, 0xA5
	};
	u8 data;
	char str[1016] = {0,};
	int i = 0, reg_list_size = 0;

	reg_list_size = ARRAY_SIZE(reg_list);
	for (i = 0; i < reg_list_size; i++) {
		s2mu004_read_reg(i2c, reg_list[i], &data);
		sprintf(str+strlen(str), "0x%02x:0x%02x, ", reg_list[i], data);
	}

	pr_info("[DEBUG][CHG]%s: %s\n", __func__, str);
}

static int s2mu004_charger_otg_control(
	struct s2mu004_charger_data *charger, bool enable)
{
	u8 chg_sts2, chg_ctrl0, temp;

	pr_info("%s: called charger otg control : %s\n", __func__,
			enable ? "ON" : "OFF");

	if (charger->otg_on == enable || lpcharge)
		return 0;

	mutex_lock(&charger->charger_mutex);
	if (!enable) {
		if (charger->is_charging) {
			pr_info("%s: Charger is enabled and OTG Disable received. Disable OTG\n", __func__);
			pr_info("%s: is_charging: %d, otg_on: %d",
				__func__, charger->is_charging, charger->otg_on);
		}
		s2mu004_update_reg(charger->i2c,
			S2MU004_CHG_CTRL0, CHG_MODE, REG_MODE_MASK);
		s2mu004_update_reg(charger->i2c, 0xAE, 0x80, 0xF0);
	} else {
#if 0
		if (charger->is_charging) {
			pr_info("%s: Charger is enabled and OTG Enabled received. Skip OTG Enable\n", __func__);
			pr_info("%s: is_charging: %d, otg_on: %d",
				__func__, charger->is_charging, charger->otg_on);
			s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS2, &chg_sts2);
			s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL0, &chg_ctrl0);
			pr_info("%s S2MU004_CHG_STATUS2: 0x%x\n", __func__, chg_sts2);
			pr_info("%s S2MU004_CHG_CTRL0: 0x%x\n", __func__, chg_ctrl0);
			mutex_unlock(&charger->charger_mutex);
			return 0;
		}
#endif

#ifndef CONFIG_SEC_FACTORY
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL7, 0x0 << SET_VF_VBYP_SHIFT, SET_VF_VBYP_MASK);
#endif
		s2mu004_update_reg(charger->i2c,
			S2MU004_CHG_CTRL4,
			S2MU004_SET_OTG_OCP_1500mA << SET_OTG_OCP_SHIFT,
			SET_OTG_OCP_MASK);
		msleep(30);
		s2mu004_update_reg(charger->i2c, 0xAE, 0x00, 0xF0);
		s2mu004_update_reg(charger->i2c,
			S2MU004_CHG_CTRL0, OTG_BST_MODE, REG_MODE_MASK);
		charger->cable_type = SEC_BATTERY_CABLE_OTG;
#ifndef CONFIG_SEC_FACTORY
		cancel_delayed_work(&charger->otg_vbus_work);
		schedule_delayed_work(&charger->otg_vbus_work, msecs_to_jiffies(1500));
#endif
	}
	charger->otg_on = enable;
	mutex_unlock(&charger->charger_mutex);

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS2, &chg_sts2);
	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL0, &chg_ctrl0);
	s2mu004_read_reg(charger->i2c, 0xAE, &temp);
	pr_info("%s S2MU004_CHG_STATUS2: 0x%x\n", __func__, chg_sts2);
	pr_info("%s S2MU004_CHG_CTRL0: 0x%x\n", __func__, chg_ctrl0);
	pr_info("%s 0xAE: 0x%x\n", __func__, temp);

	power_supply_changed(charger->psy_otg);
	return enable;
}

#if EN_IVR_IRQ
static void reduce_input_current(struct s2mu004_charger_data *charger)
{
	int old_input_current, new_input_current;
	int data;

	old_input_current = s2mu004_get_input_current_limit(charger);
	new_input_current = (old_input_current > MINIMUM_INPUT_CURRENT + REDUCE_CURRENT_STEP) ?
		(old_input_current - REDUCE_CURRENT_STEP) : MINIMUM_INPUT_CURRENT;

	if (old_input_current <= new_input_current) {
		pr_info("%s: Same or less new input current:(%d, %d, %d)\n", __func__,
			old_input_current, new_input_current, charger->input_current);
	} else {
		pr_info("%s: input currents:(%d, %d, %d)\n", __func__,
			old_input_current, new_input_current, charger->input_current);

		data = (new_input_current - 50) / 25;
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL2,
					data << INPUT_CURRENT_LIMIT_SHIFT,
					INPUT_CURRENT_LIMIT_MASK);

		charger->input_current = s2mu004_get_input_current_limit(charger);
	}
	charger->ivr_on = true;
}
#endif

#if !defined(CONFIG_SEC_FACTORY)
static void s2mu004_analog_ivr_switch(
	struct s2mu004_charger_data *charger, int enable)
{
	u8 reg_data = 0;
	int cable_type = SEC_BATTERY_CABLE_NONE;
#if defined(CONFIG_BATTERY_SWELLING)
	int swelling_mode = 0;
#endif
	union power_supply_propval value;

	if (charger->dev_id >= 0x3) {
		/* control IVRl only under PMIC REV < 0x3 */
		return;
	}

	if (factory_mode) {
		pr_info("%s: Factory Mode Skip Analog IVR Control\n", __func__);
		return;
	}

#if defined(CONFIG_BATTERY_SWELLING)
	psy_do_property("battery", get,
		POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, value);
	swelling_mode = value.intval;
#endif
	psy_do_property("battery", get,
		POWER_SUPPLY_PROP_ONLINE, value);
	cable_type = value.intval;

	if (charger->charge_mode == SEC_BAT_CHG_MODE_CHARGING_OFF ||
		charger->charge_mode == SEC_BAT_CHG_MODE_BUCK_OFF ||
#if defined(CONFIG_BATTERY_SWELLING)
		swelling_mode ||
#endif
		(is_hv_wire_type(cable_type)) ||
		(cable_type == SEC_BATTERY_CABLE_PDIC) ||
		(cable_type == SEC_BATTERY_CABLE_PREPARE_TA)) {
		pr_info("[DEBUG]%s(%d): digital IVR\n", __func__, __LINE__);
		enable = 0;
	}

	s2mu004_read_reg(charger->i2c, 0xB3, &reg_data);
	pr_info("%s : 0xB3 : 0x%x\n", __func__, reg_data);

	if (enable) {
		if (!(reg_data & 0x08)) {
			/* Enable Analog IVR */
			pr_info("[DEBUG]%s: Enable Analog IVR\n", __func__);
			s2mu004_update_reg(charger->i2c, 0xB3, 0x1 << 3, 0x1 << 3);
		}
	} else {
		if (reg_data & 0x08) {
			/* Disable Analog IVR - Digital IVR enable*/
			pr_info("[DEBUG]%s: Disable Analog IVR - Digital IVR enable\n",
				__func__);
			s2mu004_update_reg(charger->i2c, 0xB3, 0x0, 0x1 << 3);
		}
	}
}
#endif

static void s2mu004_enable_charger_switch(
	struct s2mu004_charger_data *charger, int onoff)
{
	if (factory_mode) {
		pr_info("%s: Factory Mode Skip CHG_EN Control\n", __func__);
		return;
	}

	if (charger->otg_on) {
		pr_info("[DEBUG] %s: skipped set(%d) : OTG is on\n", __func__, onoff);
		return;
	}

	if (onoff > 0) {
		pr_info("[DEBUG]%s: turn on charger\n", __func__);
#if !defined(CONFIG_SEC_FACTORY)
		if (charger->dev_id < 0x3) {
			int cable_type = SEC_BATTERY_CABLE_NONE;
#if defined(CONFIG_BATTERY_SWELLING)
			int swelling_mode = 0;
#endif
			union power_supply_propval value;

#if defined(CONFIG_BATTERY_SWELLING)
			psy_do_property("battery", get,
				POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, value);
			swelling_mode = value.intval;
#endif
			psy_do_property("battery", get,
				POWER_SUPPLY_PROP_ONLINE, value);
			cable_type = value.intval;

			if ((is_hv_wire_type(cable_type)) ||
				(cable_type == SEC_BATTERY_CABLE_PREPARE_TA) ||
#if defined(CONFIG_BATTERY_SWELLING)
				swelling_mode ||
#endif
				(cable_type == SEC_BATTERY_CABLE_PDIC) ||
				(cable_type == SEC_BATTERY_CABLE_UARTOFF)) {
				/* Digital IVR */
				s2mu004_analog_ivr_switch(charger, DISABLE);
			}
		}
#endif
		/* forced ASYNC */
		s2mu004_update_reg(charger->i2c, 0x30, 0x03, 0x03);

		msleep(30);

		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL0, CHG_MODE, REG_MODE_MASK);

		/* timer fault set 16hr(max) */
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL16,
				S2MU004_FC_CHG_TIMER_16hr << SET_TIME_CHG_SHIFT,
				SET_TIME_CHG_MASK);

		msleep(100);

		/* Auto SYNC to ASYNC - default */
		s2mu004_update_reg(charger->i2c, 0x30, 0x01, 0x03);

		/* async off */
		s2mu004_update_reg(charger->i2c, 0x96, 0x00, 0x01 << 3);
	} else {
		pr_info("[DEBUG] %s: turn off charger\n", __func__);

#if !defined(CONFIG_SEC_FACTORY)
		if (charger->dev_id < 0x3) {
			/* Disable Analog IVR - Digital IVR enable*/
			s2mu004_analog_ivr_switch(charger, DISABLE);
		}
#endif
		msleep(30);
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL0, BUCK_MODE, REG_MODE_MASK);

		/* async on */
		s2mu004_update_reg(charger->i2c, 0x96, 0x01 << 3, 0x01 << 3);
		msleep(100);
	}
}

static void s2mu004_set_buck(struct s2mu004_charger_data *charger, int enable)
{

	if (enable) {
		pr_info("[DEBUG]%s: set buck on\n", __func__);
		s2mu004_enable_charger_switch(charger, charger->is_charging);
	} else {
		pr_info("[DEBUG]%s: set buck off (charger off mode)\n", __func__);

#if !defined(CONFIG_SEC_FACTORY)
		if (charger->dev_id < 0x3) {
			/* Disable Analog IVR - Digital IVR enable*/
			s2mu004_analog_ivr_switch(charger, DISABLE);
		}
#endif
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL0, CHARGER_OFF_MODE, REG_MODE_MASK);

		/* async on */
		s2mu004_update_reg(charger->i2c, 0x96, 0x01 << 3, 0x01 << 3);
		msleep(100);
	}
}

static void s2mu004_set_regulation_vsys(
	struct s2mu004_charger_data *charger, int vsys)
{
	u8 data;

	pr_info("[DEBUG]%s: VSYS regulation %d\n", __func__, vsys);
	if (vsys <= 3800)
		data = 0;
	else if (vsys > 3800 && vsys <= 4400)
		data = (vsys - 3800) / 100;
	else
		data = 0x06;

	s2mu004_update_reg(charger->i2c,
		S2MU004_CHG_CTRL7, data << SET_VSYS_SHIFT, SET_VSYS_MASK);
}

static void s2mu004_set_regulation_voltage(
	struct s2mu004_charger_data *charger, int float_voltage)
{
	u8 data;

	if (factory_mode)
		return;

	pr_info("[DEBUG]%s: float_voltage %d\n", __func__, float_voltage);
	if (float_voltage <= 3900)
		data = 0;
	else if (float_voltage > 3900 && float_voltage <= 4530)
		data = (float_voltage - 3900) / 10;
	else
		data = 0x3f;

	s2mu004_update_reg(charger->i2c,
		S2MU004_CHG_CTRL6, data << SET_VF_VBAT_SHIFT, SET_VF_VBAT_MASK);
}

static int s2mu004_get_regulation_voltage(struct s2mu004_charger_data *charger)
{
	u8 reg_data = 0;
	int float_voltage;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL6, &reg_data);
	reg_data &= 0x3F;
	float_voltage = reg_data * 10 + 3900;
	pr_debug("%s: battery cv reg : 0x%x, float voltage val : %d\n",
		__func__, reg_data, float_voltage);

	return float_voltage;
}

static void s2mu004_set_input_current_limit(
	struct s2mu004_charger_data *charger, int charging_current)
{
	u8 data;

	if (factory_mode)
		return;

	mutex_lock(&charger->charger_mutex);
	if (is_wireless_type(charger->cable_type)) {
		pr_info("[DEBUG]%s: Wireless current limit %d\n",
			__func__, charging_current);
		if (charging_current <= 50)
			data = 0x02;
		else if (charging_current > 50 && charging_current <= 1025) {
			/* Need to re-write dts file if we need to use 5 digit current (eg. 1.0125A) */
			charging_current = charging_current * 10;
			data = (charging_current - 250) / 125;
		} else {
			pr_err("%s: Invalid WC current limit in register, set setting to maximum\n",
				__func__);
			data = 0x62;
		}

		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL3,
			data << INPUT_CURRENT_LIMIT_SHIFT, INPUT_CURRENT_LIMIT_MASK);
	} else {
		if (charging_current <= 100)
			data = 0x02;
		else if (charging_current > 100 && charging_current <= 2500)
			data = (charging_current - 50) / 25;
		else {
			pr_err("%s: Invalid current limit in register, set setting to maximum\n",
				__func__);
			data = 0x62;
		}

		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL2,
				data << INPUT_CURRENT_LIMIT_SHIFT, INPUT_CURRENT_LIMIT_MASK);
	}
	mutex_unlock(&charger->charger_mutex);

	pr_info("[DEBUG]%s: current  %d, 0x%x\n", __func__, charging_current, data);

#if EN_TEST_READ
	s2mu004_test_read(charger->i2c);
#endif
}

static int s2mu004_get_input_current_limit(struct s2mu004_charger_data *charger)
{
	u8 data;
	int w_current;

	if (is_wireless_type(charger->cable_type)) {
		data = s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL3, &data);
		if (data < 0)
			return data;

		data = data & INPUT_CURRENT_LIMIT_MASK;

		if (data > 0x62) {
			pr_err("%s: Invalid WC current limit in register: 0x%x\n",
				__func__, data);
			data = 0x62;
		}

		/* note: if use value with 5 digits the fractional 0.5 will be truncated */
		w_current = (data * 125 + 250) / 10;

		pr_debug("[DEBUG]%s: Wireless current limit out: %d\n",
					__func__, w_current);

		return w_current;
	} else {
		s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL2, &data);
		if (data < 0)
			return data;

		data = data & INPUT_CURRENT_LIMIT_MASK;

		if (data > 0x62) {
			pr_err("%s: Invalid current limit in register: 0x%x\n",
				__func__, data);
			data = 0x62;
		}
		return  data * 25 + 50;
	}
}

static void s2mu004_set_fast_charging_current(
	struct s2mu004_charger_data *charger, int charging_current)
{
	u8 data;

	if (factory_mode)
		return;

	if (charging_current <= 100)
		data = 0x03;
	else if (charging_current > 100 && charging_current <= 3150)
		data = (charging_current / 25) - 1;
	else
		data = 0x7D;

	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL9,
		data << FAST_CHARGING_CURRENT_SHIFT, FAST_CHARGING_CURRENT_MASK);

	pr_info("[DEBUG]%s: current  %d, 0x%02x\n", __func__, charging_current, data);

	if (data > 0x11)
		data = 0x11; /* 0x11 : 450mA */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL8,
		data << COOL_CHARGING_CURRENT_SHIFT, COOL_CHARGING_CURRENT_MASK);

#if EN_TEST_READ
	s2mu004_test_read(charger->i2c);
#endif
}

static int s2mu004_get_fast_charging_current(
	struct s2mu004_charger_data *charger)
{
	u8 data;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL9, &data);
	if (data < 0)
		return data;

	data = data & FAST_CHARGING_CURRENT_MASK;

	if (data > 0x7D) {
		pr_err("%s: Invalid fast charging current in register\n", __func__);
		data = 0x7D;
	}
	return (data + 1) * 25;
}

static void s2mu004_set_topoff_current(
	struct s2mu004_charger_data *charger,
	int eoc_1st_2nd, int current_limit)
{
	int data;
	union power_supply_propval value;
	struct power_supply *psy;

	pr_info("[DEBUG]%s: current  %d\n", __func__, current_limit);
	
	if (current_limit <= 100)
		data = 0;
	else if (current_limit > 100 && current_limit <= 475)
		data = (current_limit - 100) / 25;
	else
		data = 0x0F;

	switch (eoc_1st_2nd) {
	case 1:
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL11,
			data << FIRST_TOPOFF_CURRENT_SHIFT, FIRST_TOPOFF_CURRENT_MASK);
		psy = power_supply_get_by_name(charger->pdata->fuelgauge_name);
		if (!psy)
			pr_err("%s, fail to set topoff current to FG\n", __func__);
		else {
			value.intval = current_limit;
			power_supply_set_property(psy, POWER_SUPPLY_PROP_CURRENT_FULL, &value);
		}
		break;
	case 2:
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL11,
			data << SECOND_TOPOFF_CURRENT_SHIFT, SECOND_TOPOFF_CURRENT_MASK);
		break;
	default:
		break;
	}
}

static int s2mu004_get_topoff_setting(
	struct s2mu004_charger_data *charger)
{
	u8 data;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL11, &data);
	if (data < 0)
		return data;

	data = data & FIRST_TOPOFF_CURRENT_MASK;

	if (data > 0x0F)
		data = 0x0F;
	return data * 25 + 100;
}

enum {
	S2MU004_CHG_2L_IVR_4300MV = 0,
	S2MU004_CHG_2L_IVR_4500MV,
	S2MU004_CHG_2L_IVR_4700MV,
	S2MU004_CHG_2L_IVR_4900MV,
};

#if ENABLE_MIVR
/* charger input regulation voltage setting */
static void s2mu004_set_ivr_level(struct s2mu004_charger_data *charger)
{
	int chg_2l_ivr = S2MU004_CHG_2L_IVR_4500MV;

	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL5,
		chg_2l_ivr << SET_CHG_2L_DROP_SHIFT, SET_CHG_2L_DROP_MASK);
}
#endif /*ENABLE_MIVR*/

static bool s2mu004_chg_init(struct s2mu004_charger_data *charger)
{
	u8 temp;
	/* Read Charger IC Dev ID */
	s2mu004_read_reg(charger->i2c, S2MU004_REG_REV_ID, &temp);
	charger->dev_id = (temp & 0xF0) >> 4;

	pr_info("%s : DEV ID : 0x%x\n", __func__, charger->dev_id);

	/* Poor-Chg-INT Masking */
	s2mu004_update_reg(charger->i2c, 0x32, 0x03, 0x03);

	/*
	 * When Self Discharge Function is activated, Charger doesn't stop charging.
	 * If you write 0xb0[4]=1, charger will stop the charging, when self discharge
	 * condition is satisfied.
	 */
	s2mu004_update_reg(charger->i2c, 0xb0, 0x0, 0x1 << 4);

	s2mu004_update_reg(charger->i2c, S2MU004_REG_SC_INT1_MASK,
			Poor_CHG_INT_MASK, Poor_CHG_INT_MASK);

	s2mu004_write_reg(charger->i2c, 0x02, 0x0);
	s2mu004_write_reg(charger->i2c, 0x03, 0x0);

	/* ready for self-discharge, 0x76 */
	s2mu004_update_reg(charger->i2c, S2MU004_REG_SELFDIS_CFG3,
			SELF_DISCHG_MODE_MASK, SELF_DISCHG_MODE_MASK);

	/* Set Top-Off timer to 90 minutes */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL17,
			S2MU004_TOPOFF_TIMER_90m << TOP_OFF_TIME_SHIFT,
			TOP_OFF_TIME_MASK);

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL17, &temp);
	pr_info("%s : S2MU004_CHG_CTRL17 : 0x%x\n", __func__, temp);

	/* To prevent entering watchdog issue case we set WDT_CLR to not clear before enabling WDT */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL14,
			0x0 << WDT_CLR_SHIFT, WDT_CLR_MASK);

	/* enable Watchdog timer and only Charging off */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL13,
			ENABLE << SET_EN_WDT_SHIFT | DISABLE << SET_EN_WDT_AP_RESET_SHIFT,
			SET_EN_WDT_MASK | SET_EN_WDT_AP_RESET_MASK);
	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL13, &temp);
	pr_info("%s : S2MU004_CHG_CTRL13 : 0x%x\n", __func__, temp);

	/* set watchdog timer to 80 seconds */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL17,
			S2MU004_WDT_TIMER_80s << WDT_TIME_SHIFT,
			WDT_TIME_MASK);

	/* IVR Recovery enable */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL13,
		0x1 << SET_IVR_Recovery_SHIFT, SET_IVR_Recovery_MASK);

	/* Boost OSC 1Mhz */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL15,
		0x02 << SET_OSC_BST_SHIFT, SET_OSC_BST_MASK);

	/* QBAT switch speed config */
	s2mu004_update_reg(charger->i2c, 0xB2, 0x0, 0xf << 4);

	/* Top off debounce time set 1 sec */
	s2mu004_update_reg(charger->i2c, 0xC0, 0x3 << 6, 0x3 << 6);

	/* SC_CTRL21 register Minimum Charging OCP Level set to 6A */
	s2mu004_write_reg(charger->i2c, 0x29, 0x04);

	if (charger->pdata->chg_freq_ctrl) {
		switch (charger->pdata->chg_switching_freq) {
		case S2MU004_OSC_BUCK_FRQ_750kHz:
			s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL12,
				S2MU004_OSC_BUCK_FRQ_750kHz << SET_OSC_BUCK_SHIFT, SET_OSC_BUCK_MASK);
			s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL12,
				S2MU004_OSC_BUCK_FRQ_750kHz << SET_OSC_BUCK_3L_SHIFT, SET_OSC_BUCK_3L_MASK);
			break;
		default:
			/* Set OSC BUCK/BUCK 3L frequencies to default 1MHz */
			s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL12,
				S2MU004_OSC_BUCK_FRQ_1MHz << SET_OSC_BUCK_SHIFT, SET_OSC_BUCK_MASK);
			s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL12,
				S2MU004_OSC_BUCK_FRQ_1MHz << SET_OSC_BUCK_3L_SHIFT, SET_OSC_BUCK_3L_MASK);
			break;
		}
	}
	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL12, &temp);
	pr_info("%s : S2MU004_CHG_CTRL12 : 0x%x\n", __func__, temp);

	/*
	 * Disable auto-restart charging feature.
	 * Prevent charging restart after top-off timer expires
	 */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL7, 0x0 << 7, EN_CHG_RESTART_MASK);
	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL7, &temp);
	pr_info("%s : S2MU004_CHG_CTRL7 : 0x%x\n", __func__, temp);

	return true;
}

static int s2mu004_get_charging_status(
	struct s2mu004_charger_data *charger)
{
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	int ret;
	u8 chg_sts0, chg_sts1;
	union power_supply_propval value;

	ret = s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS0, &chg_sts0);
	ret = s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS1, &chg_sts1);

	psy_do_property(charger->pdata->fuelgauge_name, get,
		POWER_SUPPLY_PROP_CURRENT_AVG, value);

	if (ret < 0)
		return status;

	if (chg_sts1 & 0x80)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (chg_sts1 & 0x02 || chg_sts1 & 0x01) {
		pr_info("%s: full check curr_avg(%d), topoff_curr(%d)\n",
			__func__, value.intval, charger->topoff_current);
		if (value.intval < charger->topoff_current)
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_CHARGING;
	} else if ((chg_sts0 & 0xE0) == 0xA0 || (chg_sts0 & 0xE0) == 0x60)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;

#if EN_TEST_READ
	s2mu004_test_read(charger->i2c);
#endif
	return status;
}

static bool s2mu004_get_batt_present(struct s2mu004_charger_data *charger)
{
	u8 ret;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS3, &ret);
	if (ret < 0)
		return false;

	return (ret & DET_BAT_STATUS_MASK) ? true : false;
}

static void s2mu004_wdt_clear(struct s2mu004_charger_data *charger)
{
	u8 reg_data, chg_fault_status, en_chg;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL13, &reg_data);
	if (!(reg_data & 0x02)) {
		pr_info("%s : WDT disabled BUT Clear requested!\n", __func__);
		pr_info("%s : S2MU004_CHG_CTRL13 : 0x%x\n", __func__, reg_data);

		/* To prevent entering watchdog issue case we set WDT_CLR to not clear before enabling WDT */
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL14,
				0x0 << WDT_CLR_SHIFT, WDT_CLR_MASK);

		/* enable Watchdog timer and only Charging off */
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL13,
				ENABLE << SET_EN_WDT_SHIFT | DISABLE << SET_EN_WDT_AP_RESET_SHIFT,
				SET_EN_WDT_MASK | SET_EN_WDT_AP_RESET_MASK);

		s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL13, &reg_data);
		pr_info("%s : S2MU004_CHG_CTRL13 : 0x%x\n", __func__, reg_data);
	}

	/* watchdog kick */
	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL14,
			0x1 << WDT_CLR_SHIFT, WDT_CLR_MASK);

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS1, &reg_data);
	chg_fault_status = (reg_data & CHG_FAULT_STATUS_MASK) >> CHG_FAULT_STATUS_SHIFT;

	if ((chg_fault_status == CHG_STATUS_WD_SUSPEND) ||
		(chg_fault_status == CHG_STATUS_WD_RST)) {
		pr_info("%s: watchdog error status(0x%02x,%d)\n",
			__func__, reg_data, chg_fault_status);
		if (charger->is_charging) {
			pr_info("%s: toggle charger\n", __func__);
			s2mu004_enable_charger_switch(charger, false);
			s2mu004_enable_charger_switch(charger, true);
		}
	}

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL0, &en_chg);
	if (!(en_chg & 0x80))
		s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL0,
			0x1 << EN_CHG_SHIFT, EN_CHG_MASK);
}

static int s2mu004_get_charging_health(struct s2mu004_charger_data *charger)
{

	u8 ret;
	union power_supply_propval value;
	if (charger->is_charging)
		s2mu004_wdt_clear(charger);

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS0, &ret);
	pr_info("[DEBUG] %s: S2MU004_CHG_STATUS0 0x%x\n", __func__, ret);
	if (ret < 0)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (is_wireless_type(charger->cable_type)) {
		ret = (ret & (WCIN_STATUS_MASK)) >> WCIN_STATUS_SHIFT;
	} else {
		ret = (ret & (CHGIN_STATUS_MASK)) >> CHGIN_STATUS_SHIFT;
	}

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

	psy_do_property("battery", get, POWER_SUPPLY_PROP_ONLINE, value);
	if (value.intval == SEC_BATTERY_CABLE_PDIC)
		return POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static int s2mu004_chg_create_attrs(struct device *dev)
{
	unsigned long i;
	int rc;

	for (i = 0; i < ARRAY_SIZE(s2mu004_charger_attrs); i++) {
		rc = device_create_file(dev, &s2mu004_charger_attrs[i]);
		if (rc)
			goto create_attrs_failed;
	}
	return rc;

create_attrs_failed:
	dev_err(dev, "%s: failed (%d)\n", __func__, rc);
	while (i--)
		device_remove_file(dev, &s2mu004_charger_attrs[i]);
	return rc;
}

ssize_t s2mu004_chg_show_attrs(struct device *dev, struct device_attribute *attr, char *buf)
{
	const ptrdiff_t offset = attr - s2mu004_charger_attrs;
	int i = 0;

	switch (offset) {
	case CHIP_ID:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n", "S2MU004");
		break;
	default:
		return -EINVAL;
	}
	return i;
}

ssize_t s2mu004_chg_store_attrs(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	const ptrdiff_t offset = attr - s2mu004_charger_attrs;
	int ret = 0;

	switch (offset) {
	case CHIP_ID:
		ret = count;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int s2mu004_chg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int chg_curr, aicr;
	struct s2mu004_charger_data *charger = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->charging_current ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s2mu004_get_charging_status(charger);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = s2mu004_get_charging_health(charger);
#if EN_TEST_READ
		s2mu004_test_read(charger->i2c);
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = s2mu004_get_input_current_limit(charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (charger->charging_current) {
			aicr = s2mu004_get_input_current_limit(charger);
			chg_curr = s2mu004_get_fast_charging_current(charger);
			val->intval = MINVAL(aicr, chg_curr);
		} else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = s2mu004_get_fast_charging_current(charger);
		break;
	case POWER_SUPPLY_PROP_CURRENT_FULL:
		val->intval = s2mu004_get_topoff_setting(charger);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if ((!charger->is_charging) || (charger->cable_type == SEC_BATTERY_CABLE_NONE))
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
#if defined(CONFIG_BATTERY_SWELLING) || defined(CONFIG_BATTERY_SWELLING_SELF_DISCHARGING)
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = s2mu004_get_regulation_voltage(charger);
		break;
#endif
#if defined(CONFIG_AFC_CHARGER_MODE)
	case POWER_SUPPLY_PROP_AFC_CHARGER_MODE:
		return -ENODATA;
#endif
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = s2mu004_get_batt_present(charger);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = charger->is_charging;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
		break;
	case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
		return -ENODATA;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s2mu004_chg_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct s2mu004_charger_data *charger = power_supply_get_drvdata(psy);
	enum power_supply_ext_property ext_psp = psp;
	int buck_state = ENABLE;
	union power_supply_propval value;

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
				value.intval = 0;
			} else {
#if ENABLE_MIVR
				s2mu004_set_ivr_level(charger);
#endif
				value.intval = 1;
			}
			psy_do_property(charger->pdata->fuelgauge_name,
					set, POWER_SUPPLY_PROP_ENERGY_AVG, value);
		}

#if EN_IVR_IRQ
		if (charger->cable_type == SEC_BATTERY_CABLE_NONE) {
			/* At cable removal enable IVR IRQ if it was disabled */
			if (charger->irq_ivr_enabled == 0) {
				u8 reg_data;

				charger->irq_ivr_enabled = 1;
				/* Unmask IRQ */
				s2mu004_update_reg(charger->i2c, S2MU004_REG_SC_INT2_MASK,
					0 << IVR_M_SHIFT, IVR_M_MASK);
				enable_irq(charger->irq_ivr);
				s2mu004_read_reg(charger->i2c,
					S2MU004_REG_SC_INT2_MASK, &reg_data);
				pr_info("%s : enable ivr : 0x%x\n", __func__, reg_data);
			}
		}
#endif
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		{
			int input_current = val->intval;

			s2mu004_set_input_current_limit(charger, input_current);
			charger->input_current = input_current;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		pr_info("[DEBUG] %s: is_charging %d\n", __func__, charger->is_charging);
		charger->charging_current = val->intval;
		/* set charging current */
		s2mu004_set_fast_charging_current(charger, charger->charging_current);
#if EN_TEST_READ
		s2mu004_test_read(charger->i2c);
#endif
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		break;
	case POWER_SUPPLY_PROP_CURRENT_FULL:
		charger->topoff_current = val->intval;
		if (charger->pdata->chg_eoc_dualpath) {
			s2mu004_set_topoff_current(charger, 1, val->intval);
			s2mu004_set_topoff_current(charger, 2, 100);
		} else
			s2mu004_set_topoff_current(charger, 1, val->intval);
		break;

#if defined(CONFIG_BATTERY_SWELLING) || defined(CONFIG_BATTERY_SWELLING_SELF_DISCHARGING)
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		pr_info("[DEBUG]%s: float voltage(%d)\n", __func__, val->intval);
		charger->pdata->chg_float_voltage = val->intval;
		s2mu004_set_regulation_voltage(charger,
				charger->pdata->chg_float_voltage);
		break;
#endif
	case POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL:
		s2mu004_charger_otg_control(charger, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		charger->charge_mode = val->intval;
		psy_do_property("battery", get, POWER_SUPPLY_PROP_ONLINE, value);
		charger->cable_type = value.intval;

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
			value.intval = charger->is_charging;
			psy_do_property("s2mu004-fuelgauge", set,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, value);

			if (buck_state) {
				s2mu004_enable_charger_switch(charger, charger->is_charging);
			} else {
				/* set buck off only if SEC_BAT_CHG_MODE_BUCK_OFF */
				s2mu004_set_buck(charger, buck_state);
			}
		} else {
			pr_info("[DEBUG]%s: SKIP CHARGING CONTROL while OTG(%d)\n",
				__func__, value.intval);
		}
		break;
	case POWER_SUPPLY_PROP_ENERGY_NOW:
#if 0
		/* Switch-off charger if JIG is connected */
		if (val->intval && factory_mode) {
			pr_info("%s: JIG Connection status: %d\n", __func__, val->intval);
			s2mu004_enable_charger_switch(charger, false);
		}
#endif
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
		if (val->intval) {
			pr_info("%s: Relieve VBUS2BAT\n", __func__);
			s2mu004_write_reg(charger->i2c, 0x2F, 0xDD);
			s2mu004_update_reg(charger->i2c, 0xDA, 0x10, 0x10);
		}
		break;
	case POWER_SUPPLY_PROP_AUTHENTIC:
		if (val->intval) {
			pr_info("%s: Bypass set\n", __func__);
			s2mu004_update_reg(charger->i2c, 0x22, 0xC0, 0xC0);
			s2mu004_update_reg(charger->i2c, 0x29, 0x01 << 1, 0x01 << 1);
			s2mu004_update_reg(charger->i2c, 0x9F, 0x0, 0x01 << 7);
			s2mu004_update_reg(charger->i2c, 0x10, 0x01 << 5, 0x01 << 5);
			/* USB LDO off */
			s2mu004_update_reg(charger->i2c, S2MU004_PWRSEL_CTRL0,
				0 << PWRSEL_CTRL0_SHIFT, PWRSEL_CTRL0_MASK);
			pr_info("%s additional setting start %d\n", __func__, __LINE__);
			s2mu004_update_reg(charger->i2c, 0x93, 0x40, 0x40);
			s2mu004_update_reg(charger->i2c, 0xC6, 0x00, 0x40);
			s2mu004_update_reg(charger->i2c, 0x8B, 0x00, 0xFF);
			s2mu004_update_reg(charger->i2c, 0x71, 0x0F, 0xFF);
			pr_info("%s complete %d\n", __func__, __LINE__);
		}
		break;
#if EN_IVR_IRQ
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		{
			u8 reg_data = 0;

			s2mu004_enable_ivr_irq(charger);
			s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS3, &reg_data);
			if (reg_data & IVR_STATUS)
				queue_delayed_work(charger->charger_wqueue,
					&charger->ivr_work, msecs_to_jiffies(IVR_WORK_DELAY));
			break;
		}
#endif
#if defined(CONFIG_AFC_CHARGER_MODE)
	case POWER_SUPPLY_PROP_AFC_CHARGER_MODE:
#if defined(CONFIG_HV_MUIC_S2MU004_AFC)
		s2mu004_hv_muic_charger_init();
#endif
		break;
#endif
	case POWER_SUPPLY_PROP_MAX ... POWER_SUPPLY_EXT_PROP_MAX:
		switch (ext_psp) {
		case POWER_SUPPLY_EXT_PROP_FUELGAUGE_RESET:
			s2mu004_write_reg(charger->i2c, 0x6F, 0xC4);
			msleep(1000);
			s2mu004_write_reg(charger->i2c, 0x6F, 0x04);
			msleep(50);
			pr_info("%s: reset fuelgauge when surge occur!\n", __func__);
			break;
		case POWER_SUPPLY_EXT_PROP_FACTORY_VOLTAGE_REGULATION:
			/* enable EN_JIG_AP */
			s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL1,
					1 << EN_JIG_REG_AP_SHIFT, EN_JIG_REG_AP_MASK);
			pr_info("%s: factory voltage regulation (%d)\n", __func__, val->intval);
			s2mu004_set_regulation_vsys(charger, val->intval);
			break;
		case POWER_SUPPLY_EXT_PROP_ANDIG_IVR_SWITCH:
#if !defined(CONFIG_SEC_FACTORY)
			if (charger->dev_id < 0x3) {
				s2mu004_analog_ivr_switch(charger, val->intval);
			}
#endif
			break;
		case POWER_SUPPLY_EXT_PROP_CURRENT_MEASURE:
			if (val->intval) {
				pr_info("%s: Bypass set for current measure\n", __func__);
				/*
				 * Charger/muic interrupt can occur by entering Bypass mode
				 * Disable all interrupt mask for testing current measure.
				 */
				s2mu004_write_reg(charger->i2c, S2MU004_REG_SC_INT1_MASK, 0xFF);
				s2mu004_write_reg(charger->i2c, S2MU004_REG_SC_INT2_MASK, 0xFF);
				s2mu004_write_reg(charger->i2c, S2MU004_REG_AFC_INT_MASK, 0xFF);
				s2mu004_write_reg(charger->i2c, S2MU004_REG_MUIC_INT1_MASK, 0xFF);
				s2mu004_write_reg(charger->i2c, S2MU004_REG_MUIC_INT2_MASK, 0xFF);

				/* Enter Bypass mode set for current measure */
				s2mu004_update_reg(charger->i2c, 0x10, 0x01 << 4, 0x01 << 4);
				s2mu004_write_reg(charger->i2c, 0x12, 0x7f);
				s2mu004_write_reg(charger->i2c, 0x2f, 0xdd);
				msleep(500);
				s2mu004_update_reg(charger->i2c, 0x22, 0xc0, 0xc0);
				s2mu004_update_reg(charger->i2c, 0x29, 0x01 << 1, 0x01 << 1);
				s2mu004_update_reg(charger->i2c, 0x9F, 0x00, 0x80);
				s2mu004_update_reg(charger->i2c, 0x10, 0x01 << 5, 0x01 << 5);
				/* USB LDO off */
				s2mu004_update_reg(charger->i2c, S2MU004_PWRSEL_CTRL0,
					0 << PWRSEL_CTRL0_SHIFT, PWRSEL_CTRL0_MASK);
				psy_do_property("s2mu004-fuelgauge", set,
					POWER_SUPPLY_EXT_PROP_FUELGAUGE_FACTORY, value);
			} else {
				pr_info("%s: Bypass exit for current measure\n", __func__);
				s2mu004_update_reg(charger->i2c, 0x29, 0x0, 0x01 << 1);
				s2mu004_write_reg(charger->i2c, 0x10, 0x00);
				/* USB LDO on */
				s2mu004_update_reg(charger->i2c, S2MU004_PWRSEL_CTRL0,
					1 << PWRSEL_CTRL0_SHIFT, PWRSEL_CTRL0_MASK);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int s2mu004_otg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct s2mu004_charger_data *charger = power_supply_get_drvdata(psy);
	u8 reg;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = charger->otg_on;
		break;
	case POWER_SUPPLY_PROP_CHARGE_POWERED_OTG_CONTROL:
		s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS2, &reg);
		pr_info("%s: S2MU004_CHG_STATUS2 : 0x%X\n", __func__, reg);
		if ((reg & 0xE0) == 0x60) {
			val->intval = 1;
		} else {
			val->intval = 0;
		}
		s2mu004_read_reg(charger->i2c, S2MU004_CHG_CTRL0, &reg);
		pr_info("%s: S2MU004_CHG_CTRL0 : 0x%X\n", __func__, reg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int s2mu004_otg_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct s2mu004_charger_data *charger = power_supply_get_drvdata(psy);
	union power_supply_propval value;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		value.intval = val->intval;
		pr_info("%s: OTG %s\n", __func__, value.intval > 0 ? "ON" : "OFF");
		psy_do_property(charger->pdata->charger_name, set,
					POWER_SUPPLY_PROP_CHARGE_OTG_CONTROL, value);
		power_supply_changed(charger->psy_otg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#if defined(CONFIG_S2MU004_WIRELESS_CHARGER)
static void wpc_detect_work(struct work_struct *work)
{
	struct s2mu004_charger_data *charger = container_of(work,
						struct s2mu004_charger_data,
						wpc_work.work);
	int wc_w_state;
	int retry_cnt;
	union power_supply_propval value;
	u8 reg_data;

	pr_info("%s\n", __func__);

	retry_cnt = 0;
	do {
		s2mu004_read_reg(charger->i2c,
			S2MU004_CHG_STATUS0, &reg_data);
		reg_data = (reg_data & (WCIN_STATUS_MASK)) >> WCIN_STATUS_SHIFT;

		pr_info("%s S2MU004_CHG_STATUS0: 0x%x\n", __func__, reg_data);

		wc_w_state = ((reg_data == 0x05) || (reg_data == 0x03) ||
					(reg_data == 0x07) || (reg_data == 0x01));

		if (wc_w_state == 0)
			msleep(50);
	} while ((retry_cnt++ < 2) && (wc_w_state == 0));

	if ((charger->wc_w_state == 0) && (wc_w_state == 1)) {
		value.intval = 1;
		psy_do_property("wireless", set,
				POWER_SUPPLY_PROP_ONLINE, value);
		value.intval = SEC_BATTERY_CABLE_WIRELESS;
		pr_info("%s: wpc activated, set V_INT as PN\n",
				__func__);
	} else if ((charger->wc_w_state == 1) && (wc_w_state == 0)) {
		if (!charger->is_charging)
			s2mu004_enable_charger_switch(charger, true);

		retry_cnt = 0;
		do {
			s2mu004_read_reg(charger->i2c,
				S2MU004_CHG_STATUS0, &reg_data);
			reg_data = (reg_data & (WCIN_STATUS_MASK)) >> WCIN_STATUS_SHIFT;
			msleep(50);
		} while ((retry_cnt++ < 2) &&
			((reg_data != 0x05) || (reg_data != 0x03)));
		pr_info("%s: reg_data: 0x%x, charging: %d\n", __func__,
			reg_data, charger->is_charging);
		if (!charger->is_charging)
			s2mu004_enable_charger_switch(charger, false);
		/* To-Do: CHECK for reg_data value 0x01 cases */
		if (((reg_data == 0x05) || (reg_data == 0x03) ||
			(reg_data == 0x07) || (reg_data == 0x01)) &&
			(is_wireless_type(charger->cable_type))) {
			pr_info("%s: wpc uvlo, but charging\n", __func__);

			if ((reg_data == 0x07) || (reg_data == 0x01)) {
				pr_info(
					"%s: Abnormal WPC state, maintain charging: reg_data: 0x%x\n",
					__func__, reg_data);
			}
			queue_delayed_work(charger->charger_wqueue, &charger->wpc_work,
					   msecs_to_jiffies(500));
			return;
		} else {
			value.intval = 0;
			psy_do_property("wireless", set,
					POWER_SUPPLY_PROP_ONLINE, value);

			/* this code is for preventing reactivation of the wireless charger outside of pad*/
			value.intval = POWER_SUPPLY_HEALTH_UNDERVOLTAGE;
			psy_do_property(charger->pdata->wireless_charger_name, set,
					POWER_SUPPLY_PROP_HEALTH, value);

			pr_info("%s: wpc deactivated, set V_INT as PD\n",
					__func__);
		}
	}
	pr_info("%s: w(%d to %d)\n", __func__,
		charger->wc_w_state, wc_w_state);

	charger->wc_w_state = wc_w_state;

	/* Do unmask again. (for frequent wcin irq problem) */
	s2mu004_update_reg(charger->i2c, S2MU004_REG_SC_INT1_MASK,
		0 << WCIN_M_SHIFT, WCIN_M_MASK);

	wake_unlock(&charger->wpc_wake_lock);
}
#endif

#ifndef CONFIG_SEC_FACTORY
static void s2mu004_charger_otg_vbus_work(struct work_struct *work)
{
	struct s2mu004_charger_data *charger = container_of(work,
						struct s2mu004_charger_data,
						otg_vbus_work.work);

	s2mu004_update_reg(charger->i2c, S2MU004_CHG_CTRL7, 0x2 << SET_VF_VBYP_SHIFT, SET_VF_VBYP_MASK);
	return;
}
#endif

#if EN_BAT_DET_IRQ
/* s2mu004 interrupt service routine */
static irqreturn_t s2mu004_det_bat_isr(int irq, void *data)
{
	struct s2mu004_charger_data *charger = data;
	u8 val;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS3, &val);
	pr_info("[IRQ] %s, STATUS3 : %02x\n", __func__, val);
	if ((val & DET_BAT_STATUS_MASK) == 0) {
		s2mu004_enable_charger_switch(charger, 0);
		pr_err("charger-off if battery removed\n");
	}
	return IRQ_HANDLED;
}
#endif

static irqreturn_t s2mu004_done_isr(int irq, void *data)
{
	struct s2mu004_charger_data *charger = data;
	u8 val;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS1, &val);
	pr_info("[IRQ] %s, STATUS1 : %02x\n", __func__, val);
	if (val & (DONE_STATUS_MASK)) {
		pr_err("add self chg done\n");
		/* add chg done code here */
	}
	return IRQ_HANDLED;
}

static irqreturn_t s2mu004_chg_isr(int irq, void *data)
{
	struct s2mu004_charger_data *charger = data;
	union power_supply_propval value;
	u8 val;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS0, &val);
	pr_info("[IRQ] %s, STATUS0 : 0x%02x\n", __func__, val);
#if EN_OVP_IRQ
	if ((val & CHGIN_STATUS_MASK) == (2 << CHGIN_STATUS_SHIFT))	{
		charger->ovp = true;
		pr_info("%s: OVP triggered\n", __func__);
		value.intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		s2mu004_update_reg(charger->i2c, 0xBE, 0x10, 0x10);
		psy_do_property("battery", set,
			POWER_SUPPLY_PROP_HEALTH, value);
	} else if ((val & CHGIN_STATUS_MASK) == (3 << CHGIN_STATUS_SHIFT) ||
			(val & CHGIN_STATUS_MASK) == (5 << CHGIN_STATUS_SHIFT)) {
		pr_info("%s: Vbus status 0x%x\n", __func__, val);
		charger->unhealth_cnt = HEALTH_DEBOUNCE_CNT;
		if (charger->ovp == true)
			pr_info("%s: recover from OVP\n", __func__);
		charger->ovp = false;
		value.intval = POWER_SUPPLY_HEALTH_GOOD;
		s2mu004_update_reg(charger->i2c, 0xBE, 0x00, 0x10);
		psy_do_property("battery", set,
			POWER_SUPPLY_PROP_HEALTH, value);

	}
#endif
	return IRQ_HANDLED;
}

static irqreturn_t s2mu004_event_isr(int irq, void *data)
{
	struct s2mu004_charger_data *charger = data;
	u8 val0, val1, val2, val3;

	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS0, &val0);
	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS1, &val1);
	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS2, &val2);
	s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS3, &val3);
	pr_info("[IRQ] %s, STATUS0:0x%02x, STATUS1:0x%02x, STATUS2:0x%02x, STATUS3:0x%02x\n",
			__func__, val0, val1, val2, val3);
	return IRQ_HANDLED;
}

static irqreturn_t s2mu004_ovp_isr(int irq, void *data)
{
	pr_info("%s ovp!\n", __func__);

	return IRQ_HANDLED;
}

#if EN_IVR_IRQ
static void s2mu004_ivr_irq_work(struct work_struct *work)
{
	struct s2mu004_charger_data *charger = container_of(work,
				struct s2mu004_charger_data, ivr_work.work);
	u8 ivr_state;
	int ret;
	int ivr_cnt = 0;

	pr_info("%s:\n", __func__);

	if (charger->cable_type == SEC_BATTERY_CABLE_NONE) {
		u8 ivr_mask;

		pr_info("%s : skip\n", __func__);
		s2mu004_read_reg(charger->i2c, S2MU004_REG_SC_INT2_MASK, &ivr_mask);
		if (ivr_mask & 0x02) {
			/* Unmask IRQ */
			s2mu004_update_reg(charger->i2c, S2MU004_REG_SC_INT2_MASK,
					0 << IVR_M_SHIFT, IVR_M_MASK);
		}
		wake_unlock(&charger->ivr_wake_lock);
		return;
	}

	ret = s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS3, &ivr_state);
	if (ret < 0) {
		wake_unlock(&charger->ivr_wake_lock);
		pr_info("%s : I2C error\n", __func__);
		/* Unmask IRQ */
		s2mu004_update_reg(charger->i2c, S2MU004_REG_SC_INT2_MASK,
				0 << IVR_M_SHIFT, IVR_M_MASK);
		return;
	}
	pr_info("%s: ivr_status 0x0D:0x%02x\n", __func__, ivr_state);

	mutex_lock(&charger->charger_mutex);

	while ((ivr_state & IVR_STATUS) &&
			charger->cable_type != SEC_BATTERY_CABLE_NONE) {

		if (s2mu004_read_reg(charger->i2c, S2MU004_CHG_STATUS3, &ivr_state)) {
			pr_err("%s: Error reading S2MU004_CHG_STATUS3\n", __func__);
			break;
		}
		pr_info("%s: ivr_status 0x0D:0x%02x\n", __func__, ivr_state);

		if (++ivr_cnt >= 2) {
			reduce_input_current(charger);
			ivr_cnt = 0;
		}
		msleep(50);

		if (!(ivr_state & IVR_STATUS)) {
			pr_info("%s: EXIT IVR WORK: check value (0x0D:0x%02x, input current:%d)\n", __func__,
				ivr_state, charger->input_current);
			break;
		}

		if (s2mu004_get_input_current_limit(charger) <= MINIMUM_INPUT_CURRENT)
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
			s2mu004_update_reg(charger->i2c,
				    S2MU004_REG_SC_INT2_MASK, S2MU004_IVR_M, S2MU004_IVR_M);
			s2mu004_read_reg(charger->i2c, S2MU004_REG_SC_INT2_MASK, &reg_data);
			pr_info("%s : disable ivr : 0x%x\n", __func__, reg_data);
		}

		value.intval = s2mu004_get_input_current_limit(charger);
		psy_do_property("battery", set,
				POWER_SUPPLY_EXT_PROP_AICL_CURRENT, value);
	}

	if (charger->irq_ivr_enabled == 1) {
		/* Unmask IRQ */
		s2mu004_update_reg(charger->i2c, S2MU004_REG_SC_INT2_MASK,
			0 << IVR_M_SHIFT, IVR_M_MASK);
	}
	mutex_unlock(&charger->charger_mutex);
	wake_unlock(&charger->ivr_wake_lock);
}

static irqreturn_t s2mu004_ivr_isr(int irq, void *data)
{
	struct s2mu004_charger_data *charger = data;

	pr_info("%s: Start\n", __func__);
	wake_lock(&charger->ivr_wake_lock);
	/* Mask IRQ */
	s2mu004_update_reg(charger->i2c,
			    S2MU004_REG_SC_INT2_MASK, S2MU004_IVR_M, S2MU004_IVR_M);
	queue_delayed_work(charger->charger_wqueue, &charger->ivr_work,
		msecs_to_jiffies(IVR_WORK_DELAY));
	pr_info("%s: irq(%d)\n", __func__, irq);

	return IRQ_HANDLED;
}

static void s2mu004_enable_ivr_irq(struct s2mu004_charger_data *charger)
{
	int ret;

	ret = request_threaded_irq(charger->irq_ivr, NULL,
			s2mu004_ivr_isr, 0, "ivr-irq", charger);
	if (ret < 0) {
		pr_err("%s: Fail to request IVR_INT IRQ: %d: %d\n",
					__func__, charger->irq_ivr, ret);
		charger->irq_ivr_enabled = -1;
	} else {
		/* Unmask IRQ */
		s2mu004_update_reg(charger->i2c, S2MU004_REG_SC_INT2_MASK,
			0 << IVR_M_SHIFT, IVR_M_MASK);
		charger->irq_ivr_enabled = 1;
	}
	pr_info("%s enabled : %d\n", __func__, charger->irq_ivr_enabled);
}
#endif

#if defined(CONFIG_S2MU004_WIRELESS_CHARGER)
static irqreturn_t s2mu004_chg_wpcin_isr(int irq, void *data)
{
	struct s2mu004_charger_data *charger = data;
	unsigned long delay;

	/* Mask WCIN to prevent frequent WPC interrupts */
	s2mu004_update_reg(charger->i2c, S2MU004_REG_SC_INT1_MASK,
		1 << WCIN_M_SHIFT, WCIN_M_MASK);

#ifdef CONFIG_SAMSUNG_BATTERY_FACTORY
	delay = msecs_to_jiffies(0);
#else
	if (charger->wc_w_state)
		delay = msecs_to_jiffies(500);
	else
		delay = msecs_to_jiffies(0);
#endif
	pr_info("IRQ=%d delay = %ld\n", irq, delay);

	wake_lock(&charger->wpc_wake_lock);
	queue_delayed_work(charger->charger_wqueue, &charger->wpc_work, delay);

	return IRQ_HANDLED;
}
#endif

static int s2mu004_charger_parse_dt(struct device *dev,
	struct s2mu004_charger_platform_data *pdata)
{
	struct device_node *np = of_find_node_by_name(NULL, "s2mu004-charger");
	int ret = 0;

	if (!np) {
		pr_err("%s np NULL(s2mu004-charger)\n", __func__);
	} else {
		pdata->chg_freq_ctrl = of_property_read_bool(np,
			"battery,chg_freq_ctrl");
		ret = of_property_read_u32(np, "battery,chg_switching_freq",
			&pdata->chg_switching_freq);
		if (ret < 0) {
			pr_info("%s: Charger switching FRQ is Empty\n", __func__);
		} else {
			pr_info("%s: Charger switching is: 0x%x\n", __func__,
				pdata->chg_switching_freq);
		}
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

	pr_info("%s DT file parsed successfully, %d\n", __func__, ret);
	return ret;
}

/* if need to set s2mu004 pdata */
static struct of_device_id s2mu004_charger_match_table[] = {
	{ .compatible = "samsung,s2mu004-charger",},
	{},
};

static const struct power_supply_desc s2mu004_charger_power_supply_desc = {
	.name           = "s2mu004-charger",
	.type           = POWER_SUPPLY_TYPE_UNKNOWN,
	.get_property   = s2mu004_chg_get_property,
	.set_property   = s2mu004_chg_set_property,
	.properties     = s2mu004_charger_props,
	.num_properties = ARRAY_SIZE(s2mu004_charger_props),
};

static const struct power_supply_desc otg_power_supply_desc = {
	.name		= "otg",
	.type		= POWER_SUPPLY_TYPE_OTG,
	.get_property	= s2mu004_otg_get_property,
	.set_property	= s2mu004_otg_set_property,
	.properties	= s2mu004_otg_props,
	.num_properties	= ARRAY_SIZE(s2mu004_otg_props),
};

static int s2mu004_charger_probe(struct platform_device *pdev)
{
	struct s2mu004_dev *s2mu004 = dev_get_drvdata(pdev->dev.parent);
	struct s2mu004_platform_data *pdata = dev_get_platdata(s2mu004->dev);
	struct s2mu004_charger_data *charger;
	struct power_supply_config psy_cfg = {};

	int ret = 0;

	pr_info("%s:[BATT] S2MU004 Charger driver probe\n", __func__);
	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	mutex_init(&charger->charger_mutex);
	charger->otg_on = false;
	charger->ivr_on = false;

	charger->dev = &pdev->dev;
	charger->i2c = s2mu004->i2c;

	charger->pdata = devm_kzalloc(&pdev->dev, sizeof(*(charger->pdata)),
			GFP_KERNEL);
	if (!charger->pdata) {
		dev_err(&pdev->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err_parse_dt_nomem;
	}
	ret = s2mu004_charger_parse_dt(&pdev->dev, charger->pdata);
	if (ret < 0)
		goto err_parse_dt;

	platform_set_drvdata(pdev, charger);

	if (charger->pdata->charger_name == NULL)
		charger->pdata->charger_name = "s2mu004-charger";
	if (charger->pdata->fuelgauge_name == NULL)
		charger->pdata->fuelgauge_name = "s2mu004-fuelgauge";

	s2mu004_chg_init(charger);
	charger->input_current = s2mu004_get_input_current_limit(charger);
	charger->charging_current = s2mu004_get_fast_charging_current(charger);

	psy_cfg.drv_data = charger;
	psy_cfg.supplied_to = s2mu004_supplied_to;
	psy_cfg.num_supplicants = ARRAY_SIZE(s2mu004_supplied_to),

	charger->psy_chg = power_supply_register(&pdev->dev, &s2mu004_charger_power_supply_desc, &psy_cfg);
	if (ret) {
		goto err_power_supply_register;
	}

	charger->psy_otg = power_supply_register(&pdev->dev, &otg_power_supply_desc, &psy_cfg);
	if (ret) {
		goto err_power_supply_register_otg;
	}

	charger->charger_wqueue = create_singlethread_workqueue("charger-wq");
	if (!charger->charger_wqueue) {
		pr_info("%s: failed to create wq.\n", __func__);
		ret = -ESRCH;
		goto err_create_wq;
	}

#if EN_IVR_IRQ
	wake_lock_init(&charger->ivr_wake_lock, WAKE_LOCK_SUSPEND,
		"charger-ivr");
	INIT_DELAYED_WORK(&charger->ivr_work, s2mu004_ivr_irq_work);
#endif

	/*
	 * irq request
	 * if you need to add an irq, please refer to the code below.
	 */
	charger->irq_sys = pdata->irq_base + S2MU004_CHG1_IRQ_SYS;
	ret = request_threaded_irq(charger->irq_sys, NULL,
			s2mu004_ovp_isr, 0, "sys-irq", charger);
	if (ret < 0) {
		dev_err(s2mu004->dev, "%s: Fail to request SYS in IRQ: %d: %d\n",
					__func__, charger->irq_sys, ret);
		goto err_reg_irq;
	}

#if EN_BAT_DET_IRQ
	charger->irq_det_bat = pdata->irq_base + S2MU004_CHG2_IRQ_DET_BAT;
	ret = request_threaded_irq(charger->irq_det_bat, NULL,
			s2mu004_det_bat_isr, 0, "det_bat-irq", charger);
	if (ret < 0) {
		dev_err(s2mu004->dev, "%s: Fail to request DET_BAT in IRQ: %d: %d\n",
					__func__, charger->irq_det_bat, ret);
		goto err_reg_irq;
	}
#endif
	charger->irq_chgin = pdata->irq_base + S2MU004_CHG1_IRQ_CHGIN;
	ret = request_threaded_irq(charger->irq_chgin, NULL,
			s2mu004_chg_isr, 0, "chgin-irq", charger);
	if (ret < 0) {
		dev_err(s2mu004->dev, "%s: Fail to request CHGIN in IRQ: %d: %d\n",
					__func__, charger->irq_chgin, ret);
		goto err_reg_irq;
	}

	charger->irq_rst = pdata->irq_base + S2MU004_CHG1_IRQ_CHG_RSTART;
	ret = request_threaded_irq(charger->irq_rst, NULL,
			s2mu004_chg_isr, 0, "restart-irq", charger);
	if (ret < 0) {
		dev_err(s2mu004->dev, "%s: Fail to request CHG_Restart in IRQ: %d: %d\n",
					__func__, charger->irq_rst, ret);
		goto err_reg_irq;
	}

	charger->irq_done = pdata->irq_base + S2MU004_CHG1_IRQ_DONE;
	ret = request_threaded_irq(charger->irq_done, NULL,
			s2mu004_done_isr, 0, "done-irq", charger);
	if (ret < 0) {
		dev_err(s2mu004->dev, "%s: Fail to request DONE in IRQ: %d: %d\n",
					__func__, charger->irq_done, ret);
		goto err_reg_irq;
	}

	charger->irq_chg_fault = pdata->irq_base + S2MU004_CHG1_IRQ_CHG_Fault;
	ret = request_threaded_irq(charger->irq_chg_fault, NULL,
			s2mu004_event_isr, 0, "chg_fault-irq", charger);
	if (ret < 0) {
		dev_err(s2mu004->dev, "%s: Fail to request CHG_Fault in IRQ: %d: %d\n",
					__func__, charger->irq_chg_fault, ret);
		goto err_reg_irq;
	}

	charger->irq_bat = pdata->irq_base + S2MU004_CHG2_IRQ_BAT;
	ret = request_threaded_irq(charger->irq_bat, NULL,
			s2mu004_event_isr, 0, "bat-irq", charger);
	if (ret < 0) {
		dev_err(s2mu004->dev, "%s: Fail to request DET_BAT in IRQ: %d: %d\n",
					__func__, charger->irq_bat, ret);
		goto err_reg_irq;
	}

#if EN_IVR_IRQ
	charger->irq_ivr_enabled = -1;
	charger->irq_ivr = pdata->irq_base + S2MU004_CHG2_IRQ_IVR;
#endif

	ret = s2mu004_chg_create_attrs(&charger->psy_chg->dev);
	if (ret) {
		dev_err(charger->dev,"%s : Failed to create_attrs\n", __func__);
		goto err_reg_irq;
	}

#ifndef CONFIG_SEC_FACTORY
	INIT_DELAYED_WORK(&charger->otg_vbus_work, s2mu004_charger_otg_vbus_work);
#endif
#if EN_TEST_READ
	s2mu004_test_read(charger->i2c);
#endif
	pr_info("%s:[BATT] S2MU004 charger driver loaded OK\n", __func__);

	return 0;

err_reg_irq:
	destroy_workqueue(charger->charger_wqueue);
	power_supply_unregister(charger->psy_otg);
err_create_wq:
err_power_supply_register_otg:
	power_supply_unregister(charger->psy_chg);
err_power_supply_register:
err_parse_dt:
err_parse_dt_nomem:
	mutex_destroy(&charger->charger_mutex);
	kfree(charger);
	return ret;
}

static int s2mu004_charger_remove(struct platform_device *pdev)
{
	struct s2mu004_charger_data *charger =
		platform_get_drvdata(pdev);

	power_supply_unregister(charger->psy_chg);
	mutex_destroy(&charger->charger_mutex);
	kfree(charger);
	return 0;
}

#if defined CONFIG_PM
static int s2mu004_charger_suspend(struct device *dev)
{
	return 0;
}

static int s2mu004_charger_resume(struct device *dev)
{
	return 0;
}
#else
#define s2mu004_charger_suspend NULL
#define s2mu004_charger_resume NULL
#endif

static void s2mu004_charger_shutdown(struct device *dev)
{
	pr_info("%s: S2MU004 Charger driver shutdown\n", __func__);
}

static SIMPLE_DEV_PM_OPS(s2mu004_charger_pm_ops, s2mu004_charger_suspend,
		s2mu004_charger_resume);

static struct platform_driver s2mu004_charger_driver = {
	.driver         = {
		.name	= "s2mu004-charger",
		.owner	= THIS_MODULE,
		.of_match_table = s2mu004_charger_match_table,
		.pm		= &s2mu004_charger_pm_ops,
		.shutdown	=	s2mu004_charger_shutdown,
	},
	.probe          = s2mu004_charger_probe,
	.remove		= s2mu004_charger_remove,
};

static int __init s2mu004_charger_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&s2mu004_charger_driver);

	return ret;
}
module_init(s2mu004_charger_init);

static void __exit s2mu004_charger_exit(void)
{
	platform_driver_unregister(&s2mu004_charger_driver);
}
module_exit(s2mu004_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Charger driver for S2MU004");
