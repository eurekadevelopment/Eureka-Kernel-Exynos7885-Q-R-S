/*
 *  sec_step_charging.c
 *  Samsung Mobile Battery Driver
 *
 *  Copyright (C) 2018 Samsung Electronics
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "include/sec_battery.h"

#define STEP_CHARGING_CONDITION_VOLTAGE			0x01
#define STEP_CHARGING_CONDITION_SOC				0x02
#define STEP_CHARGING_CONDITION_CHARGE_POWER 	0x04
#define STEP_CHARGING_CONDITION_ONLINE 			0x08
#define STEP_CHARGING_CONDITION_CURRENT_NOW		0x10
#define STEP_CHARGING_CONDITION_FLOAT_VOLTAGE	0x20

void sec_bat_reset_step_charging(struct sec_battery_info *battery)
{
	battery->step_charging_status = -1;
}

void sec_bat_exit_step_charging(struct sec_battery_info *battery)
{
	battery->pdata->charging_current[battery->cable_type].fast_charging_current =
		battery->pdata->step_charging_current[battery->step_charging_step-1];
	if ((battery->step_charging_type & STEP_CHARGING_CONDITION_FLOAT_VOLTAGE) &&
		(battery->swelling_mode == SWELLING_MODE_NONE)) {
		union power_supply_propval val;
	
		pr_info("%s : float voltage = %d \n", __func__,
			battery->pdata->step_charging_float_voltage[battery->step_charging_step-1]);
		val.intval = battery->pdata->step_charging_float_voltage[battery->step_charging_step-1];
		psy_do_property(battery->pdata->charger_name, set,
			POWER_SUPPLY_PROP_VOLTAGE_MAX, val);
	}
	sec_bat_reset_step_charging(battery);
}

/*
 * true: step is changed
 * false: not changed
 */
bool sec_bat_check_step_charging(struct sec_battery_info *battery)
{
	int i = 0, value = 0, soc_condition = 0;
	static int curr_cnt = 0;

	pr_info("%s\n", __func__);
#if defined(CONFIG_SEC_FACTORY)
	return false;
#endif

	if (!battery->step_charging_type)
		return false;

	if (battery->step_charging_type & STEP_CHARGING_CONDITION_ONLINE)
		if (!is_hv_wire_type(battery->cable_type) && !(battery->cable_type == SEC_BATTERY_CABLE_PDIC))
			return false;

	if (battery->step_charging_type & STEP_CHARGING_CONDITION_CHARGE_POWER) {
		if (battery->max_charge_power < battery->step_charging_charge_power) {
			/* In case of max_charge_power falling by AICL during step-charging ongoing */
			if (battery->step_charging_status >= 0 &&
				battery->step_charging_status < battery->step_charging_step)
				sec_bat_exit_step_charging(battery);
			return false;
		}
	}

	if (battery->step_charging_status < 0)
		i = 0;
	else
		i = battery->step_charging_status;

	if (battery->step_charging_type & STEP_CHARGING_CONDITION_VOLTAGE) {
		value = battery->voltage_avg;
	} else if (battery->step_charging_type & STEP_CHARGING_CONDITION_SOC) {
		value = battery->capacity;
		if (battery->siop_level < 100 || battery->lcd_status) {
			soc_condition = battery->pdata->step_charging_condition[i] + 15;
			curr_cnt = 0;
		}
	} else {
		return false;
	}

	while (i < battery->step_charging_step - 1) {
		if (value < battery->pdata->step_charging_condition[i]) {
			break;
		}
		i++;
		if (battery->step_charging_status != -1)
			break;
	}

	if (i != battery->step_charging_status) {
		/* this is only for no consuming current */
		if ((battery->step_charging_type & STEP_CHARGING_CONDITION_CURRENT_NOW) &&
			(battery->siop_level >= 100 && !battery->lcd_status) &&
			battery->step_charging_status >= 0) {
			int condition_curr;
			condition_curr = max(battery->current_avg, battery->current_now);
			if(condition_curr < battery->pdata->step_charging_condition_curr[0]) {
				curr_cnt++;
				pr_info("%s : cnt = %d, current avg(%d)mA < current condition(%d)mA\n", __func__,
					curr_cnt, condition_curr, battery->pdata->step_charging_condition_curr[0]);
				if(curr_cnt < 3)
					return false;
			} else {
				pr_info("%s : clear count, current avg(%d)mA >= current condition(%d)mA or"
					" < 0mA this log is for debug\n", __func__,
					condition_curr, battery->pdata->step_charging_condition_curr[0]);
				curr_cnt = 0;
				return false;
			}
		}

		pr_info("%s : prev=%d, new=%d, value=%d, current=%d, curr_cnt=%d\n", __func__,
			battery->step_charging_status, i, value, battery->pdata->step_charging_current[i], curr_cnt);
		battery->pdata->charging_current[battery->cable_type].fast_charging_current = battery->pdata->step_charging_current[i];
		battery->step_charging_status = i;

		if ((battery->step_charging_type & STEP_CHARGING_CONDITION_FLOAT_VOLTAGE) &&
			(battery->swelling_mode == SWELLING_MODE_NONE)) {
			union power_supply_propval val;

			pr_info("%s : float voltage = %d \n", __func__, battery->pdata->step_charging_float_voltage[i]);
			val.intval = battery->pdata->step_charging_float_voltage[i];
			psy_do_property(battery->pdata->charger_name, set,
				POWER_SUPPLY_PROP_VOLTAGE_MAX, val);
		}
		return true;
	}
	return false;
}

#if defined(CONFIG_BATTERY_AGE_FORECAST)
void sec_bat_set_aging_info_step_charging(struct sec_battery_info *battery)
{
	if (battery->step_charging_type & STEP_CHARGING_CONDITION_FLOAT_VOLTAGE)
		battery->pdata->step_charging_float_voltage[battery->step_charging_step-1] = battery->pdata->chg_float_voltage;
	battery->pdata->step_charging_condition[0] = 
		battery->pdata->age_data[battery->pdata->age_step].step_charging_condition;

	dev_info(battery->dev,
		 "%s: float_v(%d), step_conditon(%d)\n",
		 __func__,
		 battery->pdata->step_charging_float_voltage[battery->step_charging_step-1],
		 battery->pdata->step_charging_condition[0]);	
}
#endif

void sec_step_charging_init(struct sec_battery_info *battery, struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret, len;
	sec_battery_platform_data_t *pdata = battery->pdata;
	unsigned int i;
	const u32 *p;

	ret = of_property_read_u32(np, "battery,step_charging_type",
			&battery->step_charging_type);
	pr_err("%s: step_charging_type 0x%x\n", __func__, battery->step_charging_type);
	if (ret) {
		pr_err("%s: step_charging_type is Empty\n", __func__);
		battery->step_charging_type = 0;
		return;
	}
	ret = of_property_read_u32(np, "battery,step_charging_charge_power",
			&battery->step_charging_charge_power);
	if (ret) {
		pr_err("%s: step_charging_charge_power is Empty\n", __func__);
		battery->step_charging_charge_power = 20000;
	}
	p = of_get_property(np, "battery,step_charging_condition", &len);
	if (!p) {
		battery->step_charging_step = 0;
	} else {
		len = len / sizeof(u32);
		battery->step_charging_step = len;
		pdata->step_charging_condition = kzalloc(sizeof(u32) * len, GFP_KERNEL);
		ret = of_property_read_u32_array(np, "battery,step_charging_condition",
				pdata->step_charging_condition, len);
		if (ret) {
			pr_info("%s : step_charging_condition read fail\n", __func__);
			battery->step_charging_step = 0;
		} 

		pdata->step_charging_condition_curr = kzalloc(sizeof(u32) * len, GFP_KERNEL);
		ret = of_property_read_u32_array(np, "battery,step_charging_condition_curr",
				pdata->step_charging_condition_curr, len);		
		if (ret) {
			pr_info("%s : step_charging_condition_curr read fail\n", __func__);
			battery->step_charging_step = 0;
		}

		if (len > 0) {
			pdata->step_charging_float_voltage = kzalloc(sizeof(u32) * len, GFP_KERNEL);
			ret = of_property_read_u32_array(np, "battery,step_charging_float_voltage",
					pdata->step_charging_float_voltage, len);
			if (ret) {
				pr_info("%s : step_charging_float_voltage read fail\n", __func__);
			} else {
				for (i = 0; i < len; i++) {
					pr_info("%s : step condition(%d), float voltage(%d)\n",
					__func__, pdata->step_charging_condition[i],
					pdata->step_charging_float_voltage[i]);
				}
			}

			pdata->step_charging_current = kzalloc(sizeof(u32) * len, GFP_KERNEL);
			ret = of_property_read_u32_array(np, "battery,step_charging_current",
					pdata->step_charging_current, len);
			if (ret) {
				pr_info("%s : step_charging_current read fail\n", __func__);
				battery->step_charging_step = 0;
			} else {
				battery->step_charging_status = -1;
				for (i = 0; i < len; i++) {
					pr_info("%s : step condition(%d), current(%d)\n",
					__func__, pdata->step_charging_condition[i],
					pdata->step_charging_current[i]);
				}
			}
		}
	}
}
