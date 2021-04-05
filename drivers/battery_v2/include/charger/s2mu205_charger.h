/*
 * s2mu205_charger.h - Header of S2MU205 Charger Driver
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

#ifndef S2MU205_CHARGER_H
#define S2MU205_CHARGER_H
#include <linux/mfd/samsung/s2mu205.h>

#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic.h>
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#include "../sec_charging_common.h"

/* define function if need */
#define ENABLE_MIVR 0

/* define IRQ function if need */
#define EN_BAT_DET_IRQ 0
#define EN_CHG1_IRQ_CHGIN 0

/* Test debug log enable */
#define EN_TEST_READ 1

#define HEALTH_DEBOUNCE_CNT 1

#define MINVAL(a, b) ((a <= b) ? a : b)
#define MASK(width, shift)	(((0x1 << (width)) - 1) << shift)

#define ENABLE 1
#define DISABLE 0

#define S2MU205_CHG_INT1		0x01
#define S2MU205_CHG_INT2		0x02
#define S2MU205_CHG_INT3		0x03

#define S2MU205_CHG_INT1M		0x08
#define S2MU205_CHG_INT2M		0x09
#define S2MU205_CHG_INT3M		0x0A

#define S2MU205_CHG_STATUS0		0x10
#define S2MU205_CHG_STATUS1		0x11
#define S2MU205_CHG_STATUS2		0x12
#define S2MU205_CHG_STATUS3		0x13
#define S2MU205_CHG_STATUS4		0x14
#define S2MU205_CHG_STATUS5		0x15

#define S2MU205_CHG_CTRL0		0x18
#define S2MU205_CHG_CTRL1		0x19
#define S2MU205_CHG_CTRL2		0x1A
#define S2MU205_CHG_CTRL3		0x1B
#define S2MU205_CHG_CTRL4		0x1C
#define S2MU205_CHG_CTRL5		0x1D
#define S2MU205_CHG_CTRL6		0x1E
#define S2MU205_CHG_CTRL7		0x1F
#define S2MU205_CHG_CTRL8		0x20
#define S2MU205_CHG_CTRL9		0x21
#define S2MU205_CHG_CTRL10		0x22

#define S2MU205_CHG_CTRL11		0x23
#define S2MU205_CHG_CTRL12		0x24
#define S2MU205_CHG_CTRL13		0x25
#define S2MU205_CHG_CTRL14		0x26
#define S2MU205_CHG_CTRL15		0x27
#define S2MU205_CHG_CTRL16		0x28
#define S2MU205_CHG_CTRL17		0x29
#define S2MU205_CHG_CTRL18		0x2A

/* S2MU205_CHG_STATUS0 */
#define CHGIN_STATUS_SHIFT	4
#define CHGIN_STATUS_WIDTH	3
#define CHGIN_STATUS_MASK	MASK(CHGIN_STATUS_WIDTH, CHGIN_STATUS_SHIFT)
#define CV_STATUS_SHIFT				3
#define CV_STATUS_MASK				BIT(CV_STATUS_SHIFT)
#define TOP_OFF_STATUS_SHIFT		2
#define TOP_OFF_STATUS_MASK			BIT(TOP_OFF_STATUS_SHIFT)
#define DONE_STATUS_SHIFT			1
#define DONE_STATUS_MASK			BIT(DONE_STATUS_SHIFT)
#define CHG_RESTART_STATUS_SHIFT	0
#define CHG_RESTART_STATUS_MASK		BIT(CHG_RESTART_STATUS_SHIFT)

/* S2MU205_CHG_STATUS1 */
#define CHG_FAULT_STATUS_SHIFT		0
#define CHG_FAULT_STATUS_WIDTH		4
#define CHG_FAULT_STATUS_MASK		MASK(CHG_FAULT_STATUS_WIDTH,\
					 CHG_FAULT_STATUS_SHIFT)

#define CHG_STATUS_PRE_CHARGE		1
#define CHG_STATUS_FAST_CHARGE		2
#define CHG_STATUS_WD_SUSPEND		3
#define CHG_STATUS_WD_RST			4
#define CHG_STATUS_TSD				5
#define CHG_STATUS_TFB				6

/* S2MU205_CHG_STATUS2 */
#define INPUT_ICR_CODE_STATUS_SHIFT		0
#define INPUT_ICR_CODE_STATUS_WIDTH		6
#define INPUT_ICR_CODE_STATUS_MASK		MASK(INPUT_ICR_CODE_STATUS_WIDTH, \
						INPUT_ICR_CODE_STATUS_SHIFT)

/* S2MU205_CHG_STATUS3 */
#define VBUS_SHORT_STATUS_SHIFT		4
#define VBUS_SHORT_STATUS_MASK		BIT(VBUS_SHORT_STATUS_SHIFT)
#define AICL_STATUS_SHIFT			2
#define AICL_STATUS_WIDTH			2
#define AICL_STATUS_MASK			MASK(AICL_STATUS_WIDTH, AICL_STATUS_SHIFT)
#define IVR_STATUS_SHIFT	1
#define IVR_STATUS_MASK		BIT(IVR_STATUS_SHIFT)
#define ICR_STATUS_SHIFT	0
#define ICR_STATUS_MASK		BIT(ICR_STATUS_SHIFT)

/* S2MU205_CHG_STATUS4 */
#define VSYS_STATUS_SHIFT			4
#define VSYS_STATUS_WIDTH			2
#define VSYS_STATUS_MASK			MASK(VSYS_STATUS_WIDTH, VSYS_STATUS_SHIFT)
#define OTG_STATUS_SHIFT			2
#define OTG_STATUS_WIDTH			2
#define OTG_STATUS_MASK				MASK(OTG_STATUS_WIDTH, OTG_STATUS_SHIFT)
#define BST_STATUS_SHIFT			0
#define BST_STATUS_WIDTH			2
#define BST_STATUS_MASK				MASK(BST_STATUS_WIDTH, BST_STATUS_SHIFT)

/* S2MU205_CHG_STATUS5 */
#define BATP_OPEN_STATUS_SHIFT		7
#define BATP_OPEN_STATUS_MASK		BIT(BATP_OPEN_STATUS_SHIFT)
#define BATN_OPEN_STATUS_SHIFT		6
#define BATN_OPEN_STATUS_MASK		BIT(BATN_OPEN_STATUS_SHIFT)
#define BATID_STATUS_SHIFT			4
#define BATID_STATUS_WIDTH			2
#define BATID_STATUS_MASK			MASK(BATID_STATUS_WIDTH, BATID_STATUS_SHIFT)
#define BAT_STATUS_SHIFT			0
#define BAT_STATUS_WIDTH			3
#define BAT_STATUS_MASK				MASK(BAT_STATUS_WIDTH, BAT_STATUS_SHIFT)

/* S2MU205_SC_CTRL0 */
#define EN_JIG_BYPASS_MODE_SHIFT	5
#define EN_JIG_BYPASS_MODE_MASK		BIT(EN_JIG_BYPASS_MODE_SHIFT)
#define JIG_ON_AP_SHIFT				4
#define JIG_ON_AP_MASK				BIT(JIG_ON_AP_SHIFT)
#define REG_MODE_SHIFT				0
#define REG_MODE_WIDTH				3
#define REG_MODE_MASK				MASK(REG_MODE_WIDTH, REG_MODE_SHIFT)

#define CHARGER_OFF_MODE	0
#define CHG_MODE			3
#define BUCK_MODE			1
#define OTG_BST_MODE		6

/* S2MU205_SC_CTRL1 */
#define SET_IIN_CHGIN_ILIM_SHIFT	0
#define SET_IIN_CHGIN_ILIM_WIDTH	7
#define SET_IIN_CHGIN_ILIM_MASK		MASK(SET_IIN_CHGIN_ILIM_WIDTH,\
						SET_IIN_CHGIN_ILIM_SHIFT)

/* S2MU205_SC_CTRL2 */
#define SET_IVR_SHIFT		0
#define SET_IVR_WIDTH		3
#define SET_IVR_MASK		MASK(SET_IVR_WIDTH, SET_IVR_SHIFT)

/* S2MU205_SC_CTRL3 */
#define SET_VF_VBAT_SHIFT		0
#define SET_VF_VBAT_WIDTH		7
#define SET_VF_VBAT_MASK		MASK(SET_VF_VBAT_WIDTH, SET_VF_VBAT_SHIFT)

/* S2MU205_SC_CTRL4 */
#define EN_COOL_CHG_SHIFT			7
#define EN_COOL_CHG_MASK			BIT(EN_COOL_CHG_SHIFT)
#define SET_COOL_CHG_CUR_SHIFT		0
#define SET_COOL_CHG_CUR_WIDTH		6
#define SET_COOL_CHG_CUR_MASK		MASK(SET_COOL_CHG_CUR_WIDTH, SET_COOL_CHG_CUR_SHIFT)

/* S2MU205_SC_CTRL5 */
#define SET_FAST_CHG_CUR_SHIFT		0
#define SET_FAST_CHG_CUR_WIDTH		6
#define SET_FAST_CHG_CUR_MASK	MASK(SET_FAST_CHG_CUR_WIDTH, SET_FAST_CHG_CUR_SHIFT)

/* S2MU205_SC_CTRL6 */
#define SET_VF_VSYS_SHIFT		0
#define SET_VF_VSYS_WIDTH		3
#define SET_VF_VSYS_MASK		MASK(SET_VF_VSYS_WIDTH, SET_VF_VSYS_SHIFT)

/* S2MU205_SC_CTRL10 */
#define SET_1ST_TOP_OFF_SHIFT		4
#define SET_1ST_TOP_OFF_WIDTH		4
#define SET_1ST_TOP_OFF_MASK		MASK(SET_1ST_TOP_OFF_WIDTH, SET_1ST_TOP_OFF_SHIFT)
#define SET_2ND_TOP_OFF_SHIFT		0
#define SET_2ND_TOP_OFF_WIDTH		4
#define SET_2ND_TOP_OFF_MASK		MASK(SET_2ND_TOP_OFF_WIDTH, SET_2ND_TOP_OFF_SHIFT)

/* S2MU205_SC_CTRL12 */
#define OTG_OCP_SW_ON_SHIFT			3
#define OTG_OCP_SW_ON_MASK			BIT(OTG_OCP_SW_ON_SHIFT)
#define OTG_OCP_SW_OFF_SHIFT		2
#define OTG_OCP_SW_OFF_MASK			BIT(OTG_OCP_SW_OFF_SHIFT)
#define SET_OTG_OCP_SHIFT		0
#define SET_OTG_OCP_WIDTH		2
#define SET_OTG_OCP_MASK		MASK(SET_OTG_OCP_WIDTH, SET_OTG_OCP_SHIFT)

/* S2MU205_SC_CTRL13 */
#define SET_WDT_TIME_SHIFT			1
#define SET_WDT_TIME_WIDTH			3
#define SET_WDT_TIME_MASK			MASK(SET_WDT_TIME_WIDTH, SET_WDT_TIME_SHIFT)
#define EN_WDT_SHIFT				4
#define EN_WDT_MASK 				BIT(EN_WDT_SHIFT)
#define EN_WDT_AP_RESET_SHIFT		5
#define EN_WDT_AP_RESET_MASK		BIT(EN_WDT_AP_RESET_SHIFT)
#define WDT_CLR_SHIFT 				0
#define WDT_CLR_MASK				BIT(WDT_CLR_SHIFT)

/* S2MU205_SC_CTRL14 */
#define TIME_FC_CHG_SHIFT	3
#define TIME_FC_CHG_WIDTH	3
#define TIME_FC_CHG_MASK	MASK(TIME_FC_CHG_WIDTH, TIME_FC_CHG_SHIFT)

/* S2MU205_SC_CTRL15 */
#define TOP_OFF_TIME_SHIFT	0
#define TOP_OFF_TIME_WIDTH	3
#define TOP_OFF_TIME_MASK	MASK(TOP_OFF_TIME_WIDTH, TOP_OFF_TIME_SHIFT)

#define IVR_M_SHIFT	1
#define IVR_M_MASK	BIT(IVR_M_SHIFT)
#define IVR_STATUS	0x02

#define REDUCE_CURRENT_STEP         25
#define MINIMUM_INPUT_CURRENT           300

#define FAKE_BAT_LEVEL          50

enum {
	CHIP_ID = 0,
};

ssize_t s2mu205_chg_show_attrs(struct device *dev,
		struct device_attribute *attr, char *buf);

ssize_t s2mu205_chg_store_attrs(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

#define S2MU205_CHARGER_ATTR(_name)				\
{							                    \
	.attr = {.name = #_name, .mode = 0664},	    \
	.show = s2mu205_chg_show_attrs,			    \
	.store = s2mu205_chg_store_attrs,			\
}

enum {
	CHG_REG = 0,
	CHG_DATA,
	CHG_REGS,
};

enum {
	S2MU205_TOPOFF_TIMER_5m		= 0x1,
	S2MU205_TOPOFF_TIMER_10m	= 0x2,
	S2MU205_TOPOFF_TIMER_30m	= 0x3,
	S2MU205_TOPOFF_TIMER_50m	= 0x4,
	S2MU205_TOPOFF_TIMER_70m	= 0x5,
	S2MU205_TOPOFF_TIMER_90m	= 0x6,
	S2MU205_TOPOFF_TIMER_DIS	= 0x7,
};

enum {
	S2MU205_WDT_TIMER_40s	= 0x1,
	S2MU205_WDT_TIMER_50s	= 0x2,
	S2MU205_WDT_TIMER_60s	= 0x3,
	S2MU205_WDT_TIMER_70s	= 0x4,
	S2MU205_WDT_TIMER_80s	= 0x5,
	S2MU205_WDT_TIMER_90s	= 0x6,
	S2MU205_WDT_TIMER_100s	= 0x7,
};

enum {
	S2MU205_FC_CHG_TIMER_4hr	= 0x1,
	S2MU205_FC_CHG_TIMER_6hr	= 0x2,
	S2MU205_FC_CHG_TIMER_8hr	= 0x3,
	S2MU205_FC_CHG_TIMER_10hr	= 0x4,
	S2MU205_FC_CHG_TIMER_12hr	= 0x5,
	S2MU205_FC_CHG_TIMER_14hr	= 0x6,
	S2MU205_FC_CHG_TIMER_16hr	= 0x7,
};

enum {
	S2MU205_SET_OTG_OCP_500mA   = 0x0,
	S2MU205_SET_OTG_OCP_900mA   = 0x1,
	S2MU205_SET_OTG_OCP_1200mA  = 0x2,
	S2MU205_SET_OTG_OCP_1500mA  = 0x3,
};

typedef struct s2mu205_charger_platform_data {
	int chg_float_voltage;
	char *charger_name;
	char *fuelgauge_name;
	bool chg_eoc_dualpath;
	int recharge_vcell;
	uint32_t is_1MHz_switching:1;
	int chg_switching_freq;
} s2mu205_charger_platform_data_t;


struct s2mu205_charger_data {
	struct i2c_client       *i2c;
	struct device *dev;
	struct s2mu205_platform_data *s2mu205_pdata;
	struct delayed_work otg_vbus_work;
	struct delayed_work ivr_work;
	struct wake_lock ivr_wake_lock;

	struct workqueue_struct *charger_wqueue;
	struct power_supply *psy_chg;
	struct power_supply_desc psy_chg_desc;
	struct power_supply *psy_otg;
	struct power_supply_desc psy_otg_desc;

	s2mu205_charger_platform_data_t *pdata;
	int dev_id;
	int input_current;
	int charging_current;
	int topoff_current;
	int cable_type;
	bool is_charging;
	struct mutex charger_mutex;


	bool ovp;
	bool otg_on;

	int unhealth_cnt;
	int status;
	int health;

	int irq_det_bat;
	int irq_chg;
	int irq_chgin;
	int irq_chg_fault;
	int irq_vbus;
	int irq_rst;
	int irq_done;
	int irq_sys;
	int irq_event;
	int irq_ivr;

	int charge_mode;

	int irq_ivr_enabled;
	int ivr_on;

#if defined(CONFIG_MUIC_NOTIFIER)
	struct notifier_block cable_check;
#endif
};

#endif /*S2MU205_CHARGER_H*/
