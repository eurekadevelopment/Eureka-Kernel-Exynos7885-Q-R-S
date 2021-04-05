/*
 * s2mu205-private.h - Voltage regulator driver for the s2mu205
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

#ifndef __LINUX_MFD_S2MU205_PRIV_H
#define __LINUX_MFD_S2MU205_PRIV_H

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define MFD_DEV_NAME "s2mu205"

/* Slave Address for fuelgauge */
#define I2C_ADDR_FG	(0x3B)
/*
 * Slave Address for the MFD
 * includes :
 * MUIC, AFC, PM, MST.
 * 1 bit right-shifted.
 */
#define I2C_ADDR_MUIC_SLAVE	((0x7A) >> 1)

/* Master Register Addr */
#define S2MU205_REG_IPINT			0x00
#define S2MU205_REG_IPINT_MASK		0x08
#define S2MU205_REG_PMICID			0xF5
#define S2MU205_REG_PMICID_MASK		0x0F
#define S2MU205_REG_INVALID 		0xFF

/* IRQ */
enum s2mu205_irq_source {
#if defined(CONFIG_CHARGER_S2MU205)
	CHG_INT1,
	CHG_INT2,
	CHG_INT3,
#endif
#if defined(CONFIG_LEDS_S2MU205_FLASH)
	FLED_INT1,
#endif
#if defined(CONFIG_MUIC_S2MU205)
	MUIC_INT1,
	MUIC_INT2,
#endif
	S2MU205_IRQ_GROUP_NR,
};

#define S2MU205_NUM_IRQ_CHG_REGS	3
#define S2MU205_NUM_IRQ_LED_REGS	1
#define S2MU205_NUM_IRQ_MUIC_REGS	2

#define S2MU205_IRQSRC_MUIC	(1 << 0)
#define S2MU205_IRQSRC_FLED	(1 << 1)
#define S2MU205_IRQSRC_CHG	(1 << 2)

enum s2mu205_irq {
#if defined(CONFIG_CHARGER_S2MU205)
	S2MU205_CHG1_IRQ_SYS,
	S2MU205_CHG1_IRQ_CV,
	S2MU205_CHG1_IRQ_CHG_Fault,
	S2MU205_CHG1_IRQ_CHG_RSTART,
	S2MU205_CHG1_IRQ_DONE,
	S2MU205_CHG1_IRQ_TOP_OFF,
	S2MU205_CHG1_IRQ_CHGIN,

	S2MU205_CHG2_IRQ_ICR,
	S2MU205_CHG2_IRQ_IVR,
	S2MU205_CHG2_IRQ_AICL,
	S2MU205_CHG2_IRQ_VBUS_Short,
	S2MU205_CHG2_IRQ_BST,
	S2MU205_CHG2_IRQ_OTG,
	S2MU205_CHG2_IRQ_BAT,
	S2MU205_CHG2_IRQ_MaxDuty,

	S2MU205_CHG3_IRQ_BATID,
	S2MU205_CHG3_IRQ_BATID_DONE,
	S2MU205_CHG3_IRQ_QBAT_OFF,
	S2MU205_CHG3_IRQ_BATN_OPEN,
	S2MU205_CHG3_IRQ_BATP_OPEN,
	S2MU205_CHG3_IRQ_BAT_Contact_CK_Done,
#endif
#if defined(CONFIG_LEDS_S2MU205_FLASH)
	S2MU205_FLED1_IRQ_OPEN_CH2,
	S2MU205_FLED1_IRQ_OPEN_CH1,
	S2MU205_FLED1_IRQ_SHORT_CH2,
	S2MU205_FLED1_IRQ_SHORT_CH1,
#endif
#if defined(CONFIG_MUIC_S2MU205)
	S2MU205_MUIC_IRQ1_DETACH,
	S2MU205_MUIC_IRQ1_ATTACH,
	S2MU205_MUIC_IRQ1_KP,
	S2MU205_MUIC_IRQ1_LKP,
	S2MU205_MUIC_IRQ1_LKR,
	S2MU205_MUIC_IRQ1_RID_CHG,
	S2MU205_MUIC_IRQ1_USB_Killer,
	S2MU205_MUIC_IRQ1_RID_WAKEUP,

	S2MU205_MUIC_IRQ2_VBUS_ON,
	S2MU205_MUIC_IRQ2_RSVD_ATTACH,
	S2MU205_MUIC_IRQ2_ADC_CHANGE,
	S2MU205_MUIC_IRQ2_STUCK,
	S2MU205_MUIC_IRQ2_STUCKRCV,
	S2MU205_MUIC_IRQ2_MHDL,
	S2MU205_MUIC_IRQ2_AV_CHARGE,
	S2MU205_MUIC_IRQ2_VBUS_OFF,
#endif
	S2MU205_IRQ_NR,
};

struct s2mu205_platform_data {
	/* IRQ */
	int irq_base;
	int irq_gpio;
	bool wakeup;
};

struct s2mu205_dev {
	struct device *dev;
	struct i2c_client *i2c;		/* Slave addr = 0x3D */
	struct i2c_client *muic;	/* Slave addr = (0x7A>>1) = 0x3D */
	struct i2c_client *fg;	/* Slave addr = 0x3B */
	struct mutex i2c_lock;

	int type;

	int irq;
	int irq_base;
	int irq_gpio;
	bool wakeup;
	struct mutex irqlock;
	int irq_masks_cur[S2MU205_IRQ_GROUP_NR];
	int irq_masks_cache[S2MU205_IRQ_GROUP_NR];

	/* pmic VER/REV register */
	u8 pmic_rev;	/* pmic Rev */
	u8 pmic_ver;	/* pmic version */

	struct s2mu205_platform_data *pdata;
};

enum s2mu205_types {
	TYPE_S2MU205,
};

extern int s2mu205_irq_init(struct s2mu205_dev *s2mu205);
extern void s2mu205_irq_exit(struct s2mu205_dev *s2mu205);

/* s2mu205 shared i2c API function */
extern int s2mu205_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest);
extern int s2mu205_bulk_read(struct i2c_client *i2c, u8 reg, int count,
				u8 *buf);
extern int s2mu205_write_reg(struct i2c_client *i2c, u8 reg, u8 value);
extern int s2mu205_bulk_write(struct i2c_client *i2c, u8 reg, int count,
				u8 *buf);
extern int s2mu205_write_word(struct i2c_client *i2c, u8 reg, u16 value);
extern int s2mu205_read_word(struct i2c_client *i2c, u8 reg);

extern int s2mu205_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask);

#endif /* __LINUX_MFD_S2MU205_PRIV_H */
