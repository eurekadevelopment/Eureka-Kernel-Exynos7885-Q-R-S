/*
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

#ifndef __S2MU005_MUIC_H__
#define __S2MU005_MUIC_H__

#include <linux/muic/muic.h>
#include <linux/wakelock.h>

#define MUIC_DEV_NAME	"muic-s2mu005"

/* s2mu005 muic register read/write related information defines. */

/* S2MU005 Control register */
#define CTRL_SWITCH_OPEN_SHIFT	4
#define CTRL_RAW_DATA_SHIFT		3
#define CTRL_MANUAL_SW_SHIFT	2
#define CTRL_WAIT_SHIFT			1
#define CTRL_INT_MASK_SHIFT		0

#define CTRL_SWITCH_OPEN_MASK	(0x1 << CTRL_SWITCH_OPEN_SHIFT)
#define CTRL_RAW_DATA_MASK		(0x1 << CTRL_RAW_DATA_SHIFT)
#define CTRL_MANUAL_SW_MASK		(0x1 << CTRL_MANUAL_SW_SHIFT)
#define CTRL_WAIT_MASK			(0x1 << CTRL_WAIT_SHIFT)
#define CTRL_INT_MASK_MASK		(0x1 << CTRL_INT_MASK_SHIFT)

#if IS_ENABLED(CONFIG_MUIC_S2MU005_ENABLE_AUTOSW)
#define CTRL_MASK			(CTRL_SWITCH_OPEN_MASK | \
						CTRL_MANUAL_SW_MASK | CTRL_WAIT_MASK | \
						CTRL_INT_MASK_MASK)
#else
#define CTRL_MASK			(CTRL_SWITCH_OPEN_MASK | \
						CTRL_WAIT_MASK | CTRL_INT_MASK_MASK)
#endif

/* S2MU005 MUIC Interrupt 1 register */
#define INT_RID_CHG_SHIFT		5
#define INT_LKR_SHIFT			4
#define INT_LKP_SHIFT			3
#define INT_KP_SHIFT			2
#define INT_DETACH_SHIFT		1
#define INT_ATTACH_SHIFT		0

#define INT_RID_CHG_MASK		(0x1 << INT_OVP_EN_SHIFT)
#define INT_LKR_MASK			(0x1 << INT_LKR_SHIFT)
#define INT_LKP_MASK			(0x1 << INT_LKP_SHIFT)
#define INT_KP_MASK				(0x1 << INT_KP_SHIFT)
#define INT_DETACH_MASK			(0x1 << INT_DETACH_SHIFT)
#define INT_ATTACH_MASK			(0x1 << INT_ATTACH_SHIFT)

/* S2MU005 MUIC Interrupt 2 register */
#define INT_ADC_CHANGE_SHIFT	2
#define INT_RSRV_ATTACH_SHIFT	1
#define INT_CHG_DET_SHIFT		0

#define INT_ADC_CHANGE_MASK		(0x1 << INT_ADC_CHANGE_SHIFT)
#define INT_RSRV_ATTACH_MASK	(0x1 << INT_RSRV_ATTACH_SHIFT)
#define INT_VBUS_ON_MASK		(0x1 << INT_CHG_DET_SHIFT)

/* S2MU005 ADC register */
#define ADC_MASK				(0x1f)

#define ADC_CONVERSION_MASK	(0x1 << 7)
#define ADC_CONVERSION_ERR_MASK		(0x1 << 7)

/* S2MU005 Timing Set 1 & 2 register Timing table */
#define KEY_PRESS_TIME_100MS		(0x00)
#define KEY_PRESS_TIME_200MS		(0x10)
#define KEY_PRESS_TIME_300MS		(0x20)
#define KEY_PRESS_TIME_700MS		(0x60)

#define LONGKEY_PRESS_TIME_300MS	(0x00)
#define LONGKEY_PRESS_TIME_500MS	(0x02)
#define LONGKEY_PRESS_TIME_1000MS	(0x07)
#define LONGKEY_PRESS_TIME_1500MS	(0x0C)

#define SWITCHING_WAIT_TIME_10MS	(0x00)
#define SWITCHING_WAIT_TIME_210MS	(0xa0)

/* S2MU005 MUIC Device Type 1 register */
#define DEV_TYPE1_USB_OTG		(0x1 << 7)
#define DEV_TYPE1_DEDICATED_CHG	(0x1 << 6)
#define DEV_TYPE1_CDP			(0x1 << 5)
#define DEV_TYPE1_T1_T2_CHG		(0x1 << 4)
#define DEV_TYPE1_UART			(0x1 << 3)
#define DEV_TYPE1_USB			(0x1 << 2)
#define DEV_TYPE1_AUDIO_2		(0x1 << 1)
#define DEV_TYPE1_AUDIO_1		(0x1 << 0)
#define DEV_TYPE1_USB_TYPES		(DEV_TYPE1_CDP | DEV_TYPE1_USB)
#define DEV_TYPE1_CHG_TYPES		(DEV_TYPE1_DEDICATED_CHG | DEV_TYPE1_CDP)
#define DEV_TYPE1_DEDICATED_CHG2	(0x44)

/* S2MU005 MUIC Device Type 2 register */
#define DEV_TYPE2_SDP_1P8S		(0x1 << 7)
#define DEV_TYPE2_AV			(0x1 << 6)
#define DEV_TYPE2_TTY			(0x1 << 5)
#define DEV_TYPE2_PPD			(0x1 << 4)
#define DEV_TYPE2_JIG_UART_OFF		(0x1 << 3)
#define DEV_TYPE2_JIG_UART_ON		(0x1 << 2)
#define DEV_TYPE2_JIG_USB_OFF		(0x1 << 1)
#define DEV_TYPE2_JIG_USB_ON		(0x1 << 0)
#define DEV_TYPE2_JIG_USB_TYPES		(DEV_TYPE2_JIG_USB_OFF | DEV_TYPE2_JIG_USB_ON)
#define DEV_TYPE2_JIG_UART_TYPES	(DEV_TYPE2_JIG_UART_OFF)
#define DEV_TYPE2_JIG_TYPES		(DEV_TYPE2_JIG_UART_TYPES | DEV_TYPE2_JIG_USB_TYPES)

/* S2MU005 MUIC Device Type 3 register */
#define DEV_TYPE3_U200_CHG		(0x1 << 7)
#define DEV_TYPE3_AV_WITH_VBUS	(0x1 << 4)
#define DEV_TYPE3_VBUS_R255		(0x1 << 1)
#define DEV_TYPE3_MHL			(0x1 << 0)
#define DEV_TYPE3_CHG_TYPE		(DEV_TYPE3_U200_CHG | DEV_TYPE3_VBUS_R255)

/* S2MU005 MUIC APPLE Device Type register */
#define DEV_TYPE_APPLE_VBUS_WAKEUP_SHIFT	(1)

#define DEV_TYPE_APPLE_APPLE0P5A_CHG		(0x1 << 7)
#define DEV_TYPE_APPLE_APPLE1A_CHG			(0x1 << 6)
#define DEV_TYPE_APPLE_APPLE2A_CHG			(0x1 << 5)
#define DEV_TYPE_APPLE_APPLE2P4A_CHG		(0x1 << 4)
#define DEV_TYPE_APPLE_SDP_DCD_OUT			(0x1 << 3)
#define DEV_TYPE_APPLE_RID_WAKEUP			(0x1 << 2)
#define DEV_TYPE_APPLE_VBUS_WAKEUP			(0x1 << DEV_TYPE_APPLE_VBUS_WAKEUP_SHIFT)
#define DEV_TYPE_APPLE_BCV1P2_OR_OPEN		(0x1 << 0)
#define DEV_TYPE_APPLE_APPLE_CHG			(0xf << 4)

/* S2MU005 MUIC CHG Type register */
#define CHG_TYPE_VBUS_R255	(0x1 << 7)
#define DEV_TYPE_U200		(0x1 << 4)
#define DEV_TYPE_SDP_1P8S	(0x1 << 3)
#define DEV_TYPE_USB		(0x1 << 2)
#define DEV_TYPE_CDPCHG		(0x1 << 1)
#define DEV_TYPE_DCPCHG		(0x1 << 0)
#define DEV_TYPE_CHG_TYPE		(CHG_TYPE_VBUS_R255 | DEV_TYPE_U200 | DEV_TYPE_SDP_1P8S)

/* S2MU005 MUIC MANUAL_SW_CTRL Register (0xB5) */
#define MANUAL_SW_CTRL_DM_SWITCHING_SHIFT 	5
#define MANUAL_SW_CTRL_DP_SWITCHING_SHIFT 	2
#define MANUAL_SW_CTRL_JIG_SHIFT 			0

#define MANUAL_SW_CTRL_DM_SWITCHING_MASK 	(0x7 << MANUAL_SW_CTRL_DM_SWITCHING_SHIFT)
#define MANUAL_SW_CTRL_DP_SWITCHING_MASK 	(0x7 << MANUAL_SW_CTRL_DP_SWITCHING_SHIFT)
#define MANUAL_SW_CTRL_JIG_MASK 			(0x1 << MANUAL_SW_CTRL_JIG_SHIFT)

#define MANUAL_SW_JIG_EN		(0x1 << 0)
/*
 * Manual Switch
 * D- [7:5] / D+ [4:2] / CHARGER[1] / OTGEN[0]
 * 000: Open all / 001: USB / 010: AUDIO / 011: UART / 100: V_AUDIO
 * 00: Vbus to Open / 01: Vbus to Charger / 10: Vbus to MIC / 11: Vbus to VBout
 */
#define MANSW_OPEN		0
#define MANSW_USB		1
#define MANSW_UART1		2
#define MANSW_UART2		3

#define MANUAL_SW_CTRL_OPEN_MASK		(MANSW_OPEN << MANUAL_SW_CTRL_DM_SWITCHING_SHIFT | MANSW_OPEN << MANUAL_SW_CTRL_DP_SWITCHING_SHIFT)
#define MANUAL_SW_CTRL_USB_MASK			(MANSW_USB << MANUAL_SW_CTRL_DM_SWITCHING_SHIFT | MANSW_USB << MANUAL_SW_CTRL_DP_SWITCHING_SHIFT)
#define MANUAL_SW_CTRL_UART1_MASK		(MANSW_UART1 << MANUAL_SW_CTRL_DM_SWITCHING_SHIFT | MANSW_UART1 << MANUAL_SW_CTRL_DP_SWITCHING_SHIFT)
#define MANUAL_SW_CTRL_UART2_MASK		(MANSW_UART2 << MANUAL_SW_CTRL_DM_SWITCHING_SHIFT | MANSW_UART2 << MANUAL_SW_CTRL_DP_SWITCHING_SHIFT)

/* S2MU005 MUIC CTRL2 Register (0xB6) */
#define MUIC_TIMER_SET3_DCDTMRSET_SHIFT 	0
#define MUIC_TIMER_SET3_DCDTMRSET_MASK		(0x7 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)

#define MUIC_TIMER_SET3_DCDTMRSET_250MS		(0x0 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)
#define MUIC_TIMER_SET3_DCDTMRSET_300MS		(0x1 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)
#define MUIC_TIMER_SET3_DCDTMRSET_350MS		(0x2 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)
#define MUIC_TIMER_SET3_DCDTMRSET_400MS		(0x3 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)
#define MUIC_TIMER_SET3_DCDTMRSET_600MS		(0x4 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)
#define MUIC_TIMER_SET3_DCDTMRSET_1200MS	(0x5 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)
#define MUIC_TIMER_SET3_DCDTMRSET_1800MS	(0x6 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)
#define MUIC_TIMER_SET3_DCDTMRSET_2400MS	(0x7 << MUIC_TIMER_SET3_DCDTMRSET_SHIFT)

/* S2MU005 MUIC CTRL2 Register (0xB7) */
#define MUIC_CTRL2_ADC_OFF_SHIFT (1)
#define MUIC_CTRL2_ADC_OFF_MASK (0x1 << MUIC_CTRL2_ADC_OFF_SHIFT)

enum s2mu005_muic_detect_dev_read_val {
	DEVICE_TYPE1 = 0,
	DEVICE_TYPE2,
	DEVICE_TYPE3,
	ADC,
	DEVICE_APPLE,
	CHG_TYPE,
	SC_STATUS2,
	MAX_NUM,
};

typedef enum {
	S2MU005_PATH_NONE = -1,
	S2MU005_PATH_OPEN,
	S2MU005_PATH_USB,
	S2MU005_PATH_UART_AP,
	S2MU005_PATH_UART_CP,
	S2MU005_PATH_RSVD,
	S2MU005_PATH_MAX,
} t_path_data;

typedef enum {
	S2MU005_DETECT_NONE,
	S2MU005_DETECT_DONE,
	S2MU005_DETECT_SKIP,
} t_detect_status;

#if !IS_ENABLED(CONFIG_SEC_FACTORY)
/* S2MU005_REG_LDOADC_VSET register */
#define LDOADC_VSET_MASK        0x1F
#define LDOADC_VSET_3V          0x1F
#define LDOADC_VSET_2_6V        0x0E
#define LDOADC_VSET_2_0V        0x08
#define LDOADC_VSET_2_2V        0x0A
#define LDOADC_VSET_2_4V        0x0C
#define LDOADC_VSET_1_5V        0x03
#define LDOADC_VSET_1_4V        0x02
#define LDOADC_VSET_1_2V        0x00

/* Range of ADC */
#define IS_WATER_ADC(adc)(((adc) > (ADC_GND)) && ((adc) < (ADC_OPEN)) ? 1 : 0)
#define IS_AUDIO_ADC(adc)(((adc) >= (ADC_SEND_END)) && ((adc) <= (ADC_REMOTE_S12)) ? 1 : 0)

#define WATER_TOGGLE_WA_MIN_DURATION_US	20000
#define WATER_TOGGLE_WA_MAX_DURATION_US	21000
#endif

/* s2mu005-muic macros */
#define ENUM_STR(x)	{case(x): return #x; }

#define REQUEST_IRQ(_irq, _dev_id, _name, _func)				\
do {									\
	ret = request_threaded_irq(_irq, NULL, _func,	\
				0, _name, _dev_id);	\
	if (ret < 0) {							\
		pr_err("%s:%s Failed to request IRQ #%d: %d\n",		\
				MUIC_DEV_NAME, __func__, _irq, ret);	\
		_irq = 0;						\
	}								\
} while (0)

#define FREE_IRQ(_irq, _dev_id, _name)					\
do {									\
	if (_irq) {							\
		free_irq(_irq, _dev_id);				\
		pr_info("%s:%s IRQ(%d):%s free done\n", MUIC_DEV_NAME,	\
				__func__, _irq, _name);			\
	}								\
} while (0)
/* end of macros */

/* muic chip specific internal data structure
 * that setted at muic-xxxx.c file
 */
struct s2mu005_muic_data {
        struct device *dev;
	struct i2c_client *i2c; /* i2c addr: 0x7A; MUIC */
        struct s2mu005_dev *s2mu005_dev;
	struct mutex muic_mutex;
        struct mutex switch_mutex;

	/* model dependant mfd platform data */
	struct s2mu005_platform_data	*mfd_pdata;

        /* model dependant muic platform data */
	struct muic_platform_data *pdata;

	int irq_attach;
	int irq_detach;
	int irq_rid_chg;
	int irq_vbus_on;
	int irq_rsvd_attach;
	int irq_adc_change;
	int irq_av_charge;
	int irq_vbus_off;

	/* muic common callback driver internal data */
	struct sec_switch_data *switch_data;

        void *if_data;

	struct wake_lock wake_lock;

	/* muic support vps list */
	bool muic_support_list[ATTACHED_DEV_NUM];

	/* muic current attached device */
	muic_attached_dev_t	attached_dev;

	/* muic Device ID */
	u8 muic_vendor;			/* Vendor ID */
	u8 muic_version;		/* Version ID */
	u8 ic_rev_id;			/* Rev ID */

	struct delayed_work dp_0p6v;

	muic_attached_dev_t new_dev;

	int reg[MAX_NUM];
        int adc;
	int vbvolt;
	int vmid;

	bool	is_usb_ready;
	bool	is_factory_start;
	bool	is_rustproof;
	bool	is_otg_test;
	bool	is_dcp;
	bool    jigonb_enable;
	bool    jig_disable;

	/* W/A waiting for the charger ic */
	bool suspended;
	bool need_to_noti;

#if IS_ENABLED(CONFIG_HICCUP_CHARGER)
	bool is_hiccup_mode;
#endif

#if IS_ENABLED(CONFIG_S2MU005_SUPPORT_BC1P2_CERTI)
	u8 vbus_ldo;
#endif /* CONFIG_S2MU005_SUPPORT_BC1P2_CERTI */

#if IS_ENABLED(CONFIG_S2MU005_MUIC_DEBUG)
	struct delayed_work debug_dwrok;
	int irq_attach_cnt;
	int irq_detach_cnt;
	int irq_vbus_on_cnt;
	int irq_adc_change_cnt;
	int irq_vbus_off_cnt;
#endif	/* CONFIG_S2MU005_MUIC_DEBUG */

	struct workqueue_struct *muic_wqueue;

	int rev_id;
};

extern struct muic_platform_data muic_pdata;

#endif /* __S2MU005_MUIC_H__ */
