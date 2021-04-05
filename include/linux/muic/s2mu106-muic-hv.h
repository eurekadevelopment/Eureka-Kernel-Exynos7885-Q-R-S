/*
 * s2mu106-muic-hv.h - MUIC for the Samsung s2mu004
 *
 * Copyright (C) 2018 Samsung Electrnoics
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

#ifndef __S2MU106_MUIC_HV_H__
#define __S2MU106_MUIC_HV_H__

#define MUIC_HV_DEV_NAME		"muic-s2mu106-hv"

/* S2MU106 AFC INT MASK register(0x08) */
#define INTMASK_VBADC_SHIFT			0
#define INTMASK_VDNMON_SHIFT		1
#define INTMASK_DNRES_SHIFT			2
#define INTMASK_MPNACK_SHIFT		3
#define INTMASK_MRXBUFOW_SHIFT		4
#define INTMASK_MRXTRF_SHIFT		5
#define INTMASK_MRXPERR_SHIFT		6
#define INTMASK_MRXRDY_SHIFT		7

#define INTMASK_VBADC_MASK			(0x1 << INTMASK_VBADC_SHIFT)
#define INTMASK_VDNMON_MASK			(0x1 << INTMASK_VDNMON_SHIFT)
#define INTMASK_DNRES_MASK			(0x1 << INTMASK_DNRES_SHIFT)
#define INTMASK_MPNACK_MASK			(0x1 << INTMASK_MPNACK_SHIFT)
#define INTMASK_MRXBUFOW_MASK		(0x1 << INTMASK_MRXBUFOW_SHIFT)
#define INTMASK_MRXTRF_MASK			(0x1 << INTMASK_MRXTRF_SHIFT)
#define INTMASK_MRXPERR_MASK		(0x1 << INTMASK_MRXPERR_SHIFT)
#define INTMASK_MRXRDY_MASK			(0x1 << INTMASK_MRXRDY_SHIFT)

/* S2MU106 AFC STATUS register(0x10) */
#define STATUS_VBADC_SHIFT		0
#define STATUS_VDNMON_SHIFT		4
#define STATUS_DNRES_SHIFT		5
#define STATUS_MPNACK_SHIFT		6
#define STATUS_ADCEOC_SHIFT		7

#define STATUS_VBADC_MASK		(0xf << STATUS_VBADC_SHIFT)
#define STATUS_VDNMON_MASK		(0x1 << STATUS_VDNMON_SHIFT)
#define STATUS_DNRES_MASK		(0x1 << STATUS_DNRES_SHIFT)
#define STATUS_MPNACK_MASK		(0x1 << STATUS_MPNACK_SHIFT)
#define STATUS_ADCEOC_MASK		(0x1 << STATUS_ADCEOC_SHIFT)

/* S2MU106 AFC CONTROL 1 register(0x2B) */
#define AFCCTRL1_DPDNVDEN_SHIFT		0
#define AFCCTRL1_DNVD_SHIFT		1
#define AFCCTRL1_DPVD_SHIFT		3
#define AFCCTRL1_VBUSADCEN_SHIFT	5
#define AFCCTRL1_CTRLIDMON_SHIFT	6
#define AFCCTRL1_AFCEN_SHIFT		7

#define AFCCTRL1_DPDNVDEN_MASK		(0x1 << AFCCTRL1_DPDNVDEN_SHIFT)
#define AFCCTRL1_DNVD_MASK		(0x3 << AFCCTRL1_DNVD_SHIFT)
#define AFCCTRL1_DPVD_MASK		(0x3 << AFCCTRL1_DPVD_SHIFT)
#define AFCCTRL1_VBUSADCEN_MASK		(0x1 << AFCCTRL1_VBUSADCEN_SHIFT)
#define AFCCTRL1_CTRLIDMON_MASK		(0x1 << AFCCTRL1_CTRLIDMON_SHIFT)
#define AFCCTRL1_AFCEN_MASK		(0x1 << AFCCTRL1_AFCEN_SHIFT)

#define DPDN_HIZ			(0x0)
#define DPDN_GND			(0x1)
#define DPDN_0p6V			(0x2)
#define DPDN_3p3V			(0x3)
#define DP_HIZ_MASK			(DPDN_HIZ << AFCCTRL1_DPVD_SHIFT)
#define DP_GND_MASK			(DPDN_GND << AFCCTRL1_DPVD_SHIFT)
#define DP_0p6V_MASK			(DPDN_0p6V << AFCCTRL1_DPVD_SHIFT)
#define DP_3p3V_MASK			(DPDN_3p3V << AFCCTRL1_DPVD_SHIFT)
#define DN_HIZ_MASK			(DPDN_HIZ << AFCCTRL1_DNVD_SHIFT)
#define DN_GND_MASK			(DPDN_GND << AFCCTRL1_DNVD_SHIFT)
#define DN_0p6V_MASK			(DPDN_0p6V << AFCCTRL1_DNVD_SHIFT)
#define DN_3p3V_MASK			(DPDN_3p3V << AFCCTRL1_DNVD_SHIFT)

/* S2MU106 AFC CONTROL 2 register(0x2C) */
#define AFCCTRL2_DP06EN_SHIFT			1
#define AFCCTRL2_DNRESEN_SHIFT			2
#define AFCCTRL2_MTXEN_SHIFT			3
#define AFCCTRL2_MPING_SHIFT			4
#define AFCCTRL2_RSTDM100UI_SHIFT		7

#define AFCCTRL2_DP06EN_MASK			(0x1 << AFCCTRL2_DP06EN_SHIFT)
#define AFCCTRL2_DNRESEN_MASK			(0x1 << AFCCTRL2_DNRESEN_SHIFT)
#define AFCCTRL2_MTXEN_MASK			(0x1 << AFCCTRL2_MTXEN_SHIFT)
#define AFCCTRL2_MPING_MASK			(0x1 << AFCCTRL2_MPING_SHIFT)
#define AFCCTRL2_RSTDM100UI_MASK		(0x1 << AFCCTRL2_RSTDM100UI_SHIFT)

/* S2MU106 AFC OTP 6 register(0x2C) */
#define AFCOTP6_CTRL_IDM_ON_REG_SEL_SHIFT	6

#define AFCOTP6_CTRL_IDM_ON_REG_SEL_MASK	(0x1 << AFCOTP6_CTRL_IDM_ON_REG_SEL_SHIFT)

/* S2MU106 AFC TX BYTE DATA */
#define AFCTXBYTE_VOL_SHIFT 4
#define AFCTXBYTE_VOL_MASK (0xf << AFCTXBYTE_VOL_SHIFT)
#define AFCTXBYTE_CUR_SHIFT 0
#define AFCTXBYTE_CUR_MASK (0xf << AFCTXBYTE_CUR_SHIFT)

#define AFCTXBYTE_5V 0x0
#define AFCTXBYTE_6V 0x1
#define AFCTXBYTE_7V 0x2
#define AFCTXBYTE_8V 0x3
#define AFCTXBYTE_9V 0x4
#define AFCTXBYTE_10V 0x5
#define AFCTXBYTE_11V 0x6
#define AFCTXBYTE_12V 0x7
#define AFCTXBYTE_13V 0x8
#define AFCTXBYTE_14V 0x9
#define AFCTXBYTE_15V 0xA
#define AFCTXBYTE_16V 0xB
#define AFCTXBYTE_17V 0xC
#define AFCTXBYTE_18V 0xD
#define AFCTXBYTE_19V 0xE
#define AFCTXBYTE_20V 0xF

#define AFCTXBYTE_0p75A 0x0
#define AFCTXBYTE_0p90A 0x1
#define AFCTXBYTE_1p05A 0x2
#define AFCTXBYTE_1p20A 0x3
#define AFCTXBYTE_1p35A 0x4
#define AFCTXBYTE_1p50A 0x5
#define AFCTXBYTE_1p65A 0x6
#define AFCTXBYTE_1p80A 0x7
#define AFCTXBYTE_1p95A 0x8
#define AFCTXBYTE_2p10A 0x9
#define AFCTXBYTE_2p25A 0xA
#define AFCTXBYTE_2p40A 0xB
#define AFCTXBYTE_2p55A 0xC
#define AFCTXBYTE_2p70A 0xD
#define AFCTXBYTE_2p85A 0xE
#define AFCTXBYTE_3p00A 0xF

/* S2MU004 HVRXBYTE register */
#define HVRXBYTE_MAX			16

/* S2MU004 AFC charger W/A Check NUM */
#define AFC_CHARGER_WA_PING		5

/* S2MU004 MPing miss SW Workaround - delay time */
#define MPING_MISS_WA_TIME		2000

#define AFC_MRXRDY_CNT_LIMIT (3)
#define AFC_MPING_RETRY_CNT_LIMIT (20)
#define AFC_QC_RETRY_CNT_LIMIT (3)
#define VCHGIN_CHECK_CNT_LIMIT (3)

#define IS_VCHGIN_9V(x) ((8000 <= x) && (x <= 10300))
#define IS_VCHGIN_5V(x) ((4000 <= x) && (x <= 6000))

typedef enum {
	MU106_IRQ_VDNMON = 1,
	MU106_IRQ_DNRES,
	MU106_IRQ_MPNACK,
	MU106_IRQ_MRXBUFOW,
	MU106_IRQ_MRXTRF,
	MU106_IRQ_MRXPERR,
	MU106_IRQ_MRXRDY = 7,
	MU106_IRQ_VCHGIN = 8,
} afc_int_t;

typedef enum {
	MU106_NOT_MASK = 0,
	MU106_MASK = 1,
} int_mask_t;

typedef enum {
	MU106_QC_PROTOCOL,
	MU106_AFC_PROTOCOL,
} protocol_sw_t;

typedef enum {
	QC_UNKHOWN,
	QC_5V,
	QC_9V,
	QC_12V,
} qc_2p0_type_t;

typedef enum {
	VDNMON_LOW		= 0x00,
	VDNMON_HIGH		= (0x1 << STATUS_VDNMON_SHIFT),

	VDNMON_DONTCARE		= 0xff,
} vdnmon_t;

/* MUIC afc irq type */
typedef enum {
	MUIC_AFC_IRQ_VDNMON = 0,
	MUIC_AFC_IRQ_MRXRDY,
	MUIC_AFC_IRQ_VBADC,
	MUIC_AFC_IRQ_MPNACK,
	MUIC_AFC_IRQ_DONTCARE = 0xff,
} muic_afc_irq_t;

typedef enum tx_data{
    MUIC_HV_5V = 0,
    MUIC_HV_9V,
} muic_afc_txdata_t;

struct s2mu106_muic_data;
extern int s2mu106_hv_muic_init(struct s2mu106_muic_data *muic_data);
extern void s2mu106_hv_muic_remove(struct s2mu106_muic_data *muic_data);
extern muic_attached_dev_t hv_muic_check_id_err(struct s2mu106_muic_data *muic_data,
		muic_attached_dev_t new_dev);
#ifdef CONFIG_HV_MUIC_VOLTAGE_CTRL
extern int muic_afc_set_voltage(int vol);
#endif
#endif /* __S2MU106_MUIC_HV_H__ */
