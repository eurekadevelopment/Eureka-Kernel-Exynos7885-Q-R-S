/*
 * s2mu004_charger_common.h
 *
 * Copyright (C) 2012 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __S2MU004_CHARGER_COMMON_H
#define __S2MU004_CHARGER_COMMON_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/power_supply.h>

/* definitions */
#define	SEC_SIZEOF_POWER_SUPPLY_TYPE	POWER_SUPPLY_TYPE_MAX

enum sec_battery_voltage_mode {
	/* average voltage */
	SEC_BATTERY_VOLTAGE_AVERAGE = 0,
	/* open circuit voltage */
	SEC_BATTERY_VOLTAGE_OCV,
};

enum sec_battery_current_mode {
	/* uA */
	SEC_BATTERY_CURRENT_UA = 0,
	/* mA */
	SEC_BATTERY_CURRENT_MA,
};

enum sec_battery_capacity_mode {
	/* designed capacity */
	SEC_BATTERY_CAPACITY_DESIGNED = 0,
	/* absolute capacity by fuel gauge */
	SEC_BATTERY_CAPACITY_ABSOLUTE,
	/* temperary capacity in the time */
	SEC_BATTERY_CAPACITY_TEMPERARY,
	/* current capacity now */
	SEC_BATTERY_CAPACITY_CURRENT,
	/* cell aging information */
	SEC_BATTERY_CAPACITY_AGEDCELL,
	/* charge count */
	SEC_BATTERY_CAPACITY_CYCLE,
};

/* charging mode */
enum sec_battery_charging_mode {
	/* no charging */
	SEC_BATTERY_CHARGING_NONE = 0,
	/* 1st charging */
	SEC_BATTERY_CHARGING_1ST,
	/* 2nd charging */
	SEC_BATTERY_CHARGING_2ND,
	/* recharging */
	SEC_BATTERY_CHARGING_RECHARGING,
};

enum sec_battery_charge_mode {
	SEC_BAT_CHG_MODE_CHARGING = 0,
	SEC_BAT_CHG_MODE_CHARGING_OFF,
	SEC_BAT_CHG_MODE_BUCK_OFF,
};

enum power_supply_ext_property {
	POWER_SUPPLY_EXT_PROP_CHECK_SLAVE_I2C = POWER_SUPPLY_PROP_MAX,
	POWER_SUPPLY_EXT_PROP_MULTI_CHARGER_MODE,
	POWER_SUPPLY_EXT_PROP_WIRELESS_OP_FREQ,
	POWER_SUPPLY_EXT_PROP_WIRELESS_TX_CMD,
	POWER_SUPPLY_EXT_PROP_WIRELESS_TX_VAL,
	POWER_SUPPLY_EXT_PROP_AICL_CURRENT,
	POWER_SUPPLY_EXT_PROP_CHECK_MULTI_CHARGE,
	POWER_SUPPLY_EXT_PROP_INBAT_VOLTAGE_FGSRC_SWITCHING,
	POWER_SUPPLY_EXT_PROP_FUELGAUGE_RESET,
	POWER_SUPPLY_EXT_PROP_FACTORY_VOLTAGE_REGULATION,
	POWER_SUPPLY_EXT_PROP_ANDIG_IVR_SWITCH,
};

/* full charged check : POWER_SUPPLY_PROP_STATUS */
enum sec_battery_full_charged {
	SEC_BATTERY_FULLCHARGED_NONE = 0,
	/* current check by ADC */
	SEC_BATTERY_FULLCHARGED_ADC,
	/* fuel gauge current check */
	SEC_BATTERY_FULLCHARGED_FG_CURRENT,
	/* time check */
	SEC_BATTERY_FULLCHARGED_TIME,
	/* SOC check */
	SEC_BATTERY_FULLCHARGED_SOC,
	/* charger GPIO, NO additional full condition */
	SEC_BATTERY_FULLCHARGED_CHGGPIO,
	/* charger interrupt, NO additional full condition */
	SEC_BATTERY_FULLCHARGED_CHGINT,
	/* charger power supply property, NO additional full condition */
	SEC_BATTERY_FULLCHARGED_CHGPSY,
};

#define sec_battery_full_charged_t \
	enum sec_battery_full_charged

struct sec_charging_current {
#ifdef CONFIG_OF
	unsigned int input_current_limit;
	unsigned int fast_charging_current;
	unsigned int full_check_current_1st;
	unsigned int full_check_current_2nd;
#else
	int input_current_limit;
	int fast_charging_current;
	int full_check_current_1st;
	int full_check_current_2nd;
#endif
};

#define sec_charging_current_t \
	struct sec_charging_current

static inline struct power_supply *get_power_supply_by_name(char *name)
{
	if (!name)
		return (struct power_supply *)NULL;
	else
		return power_supply_get_by_name(name);
}

#define psy_do_property(name, function, property, value) \
{   \
	struct power_supply *psy;   \
	int ret;    \
	psy = get_power_supply_by_name((name)); \
	if (!psy) { \
		pr_err("%s: Fail to "#function" psy (%s)\n",    \
			__func__, (name));  \
		value.intval = 0;   \
	} else {    \
		if (psy->desc->function##_property != NULL) { \
			ret = psy->desc->function##_property(psy, (property), &(value)); \
			if (ret < 0) {  \
				pr_err("%s: Fail to %s "#function" (%d=>%d)\n", \
						__func__, name, (property), ret);   \
				value.intval = 0;   \
			}   \
		}   \
	}   \
}

#define psy_do_property_dup(name, function, property, value) \
{   \
	int ret;    \
	psy = get_power_supply_by_name((name)); \
	if (!psy) { \
		value.intval = 0;   \
	} else {    \
		ret = psy->desc->function##_property(psy, (property), &(value)); \
		if (ret < 0) {  \
			value.intval = 0;   \
		}   \
	}   \
}


/* SEC_FUELGAUGE_CAPACITY_TYPE_ATOMIC
  * change capacity value by only -1 or +1
  * no sudden change of capacity
  */
#define SEC_FUELGAUGE_CAPACITY_TYPE_ATOMIC  8

/* SEC_FUELGAUGE_CAPACITY_TYPE_SKIP_ABNORMAL
  * skip current capacity value
  * if it is abnormal value
  */
#define SEC_FUELGAUGE_CAPACITY_TYPE_SKIP_ABNORMAL   16

#define SEC_FUELGAUGE_CAPACITY_TYPE_RAW 1

/* SEC_FUELGAUGE_CAPACITY_TYPE_SCALE
  * rescale capacity by scaling, need min and max value for scaling
  */
#define SEC_FUELGAUGE_CAPACITY_TYPE_SCALE   2

/* SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE
  * change only maximum capacity dynamically
  * to keep time for every SOC unit
  */
#define SEC_FUELGAUGE_CAPACITY_TYPE_DYNAMIC_SCALE   4

/* SEC_FUELGAUGE_CAPACITY_TYPE_RESET
  * use capacity information to reset fuel gauge
  * (only for driver algorithm, can NOT be set by user)
  */
#define SEC_FUELGAUGE_CAPACITY_TYPE_RESET   (-1)

#endif /* __S2MU004_CHARGER_COMMON_H */
