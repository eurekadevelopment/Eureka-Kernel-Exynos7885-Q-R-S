/*
 * s2mu004_haptic.h
 * Samsung S2MU004 Fuel Gauge Header
 *
 * Copyright (C) 2015 Samsung Electronics, Inc.
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

#ifndef __S2MU004_HAPTIC_H
#define __S2MU004_HAPTIC_H __FILE__

/* Slave address should be shifted to the right 1bit.
 * R/W bit should NOT be included.
 */

#define S2MU004_REG_HAPTIC_IRQ		0x00
#define S2MU004_REG_HAPTIC_MASK		0x01
#define S2MU004_REG_FCYCLE_TAR		0x03
#define S2MU004_REG_CYCLE_TAR		0x04
#define S2MU004_REG_REPEAT1_TAR		0x05
#define S2MU004_REG_REPEAT2_TAR		0x06
#define S2MU004_REG_MUTE1_TAR		0x07
#define S2MU004_REG_MUTE2_TAR		0x08
#define S2MU004_REG_IM_TAR		0x09
#define S2MU004_REG_PERI_TAR1		0x0A
#define S2MU004_REG_PERI_TAR2		0x0B
#define S2MU004_REG_DUTY_TAR1		0x0C
#define S2MU004_REG_DUTY_TAR2		0x0D
#define S2MU004_REG_HAPTIC_MODE		0x0E
#define S2MU004_REG_SINECOEF1		0x0F
#define S2MU004_REG_SINECOEF2		0x10
#define S2MU004_REG_SINECOEF3		0x11
#define S2MU004_REG_AMPCOEF1		0x16
#define S2MU004_REG_AMPCOEF2		0x17
#define S2MU004_REG_AMPCOEF3		0x18
#define S2MU004_REG_SINE_CTRL		0x19

/* 0x0E */
#define HAPTIC_PMODE_SHIFT	7
#define HAPTIC_PMODE_MASK	BIT(HAPTIC_PMODE_SHIFT)

#define HAPTIC_IMODE_SHIFT	6
#define HAPTIC_IMODE_MASK	BIT(HAPTIC_IMODE_SHIFT)

#define HAPTIC_EMODE_SHIFT	5
#define HAPTIC_EMODE_MASK	BIT(HAPTIC_EMODE_SHIFT)

#define HAPTIC_SMODE_SHIFT	4
#define HAPTIC_SMODE_MASK	BIT(HAPTIC_SMODE_SHIFT)

#define AUTO_CAL_EN_SHIFT	1
#define AUTO_CAL_EN_MASK	BIT(AUTO_CAL_EN_SHIFT)

#define HAPTIC_EN_SHIFT		0
#define HAPTIC_EN_MASK		BIT(HAPTIC_EN_SHIFT)
#define	HAPTIC_EN		1
#define	HAPTIC_OFF		0


enum s2mu004_haptic_motor_type {
	S2MU004_HAPTIC_ERM,
	S2MU004_HAPTIC_LRA,
};

enum s2mu004_haptic_pulse_mode {
	S2MU004_EXTERNAL_MODE,
	S2MU004_INTERNAL_MODE,
};

struct s2mu004_haptic_platform_data {

	u16 max_timeout;
	u32 duty;
	u32 period;
	u32 max_duty;
	u16 reg2;
	char *regulator_name;
	unsigned int pwm_id;

	void (*init_hw) (void);
	void (*motor_en) (bool);
};

enum motor_control_type {
	IFMPIC_TYPE = 1,
	EXTERNAL_DRIVING_IC	
};

#endif /* __S2MU004_HAPTIC_H */
