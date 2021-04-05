/*
 * RGB-led driver for IF PMIC s2mu004
 *
 * Copyright (C) 2015 Samsung Electronics
 * Suji Lee <suji0908.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LEDS_S2MU004_RGB_H__
#define __LEDS_S2MU004_RGB_H__

/* s2mu004 LED numbers */
#define S2MU004_LED_NUM	4

/* s2mu004_REG_LED_DUR */
#define S2MU004_LED_ON_DUR	0xF0
#define S2MU004_LED_OFF_DUR	0x0F

/* s2mu004_REG_LEDRMP */
#define S2MU004_RAMP_UP		0xF0
#define S2MU004_RAMP_DN		0x0F

#define LED_R_MASK		0x00FF0000
#define LED_G_MASK		0x0000FF00
#define LED_B_MASK		0x000000FF
#define LED_MAX_CURRENT		0xFF

/* s2mu004_Reset */
#define RGBLED_REG_RESET_MASK	0x01

/* s2mu004_STATE*/
#define LED_DISABLE		0
#define LED_ALWAYS_ON	1
#define LED_BLINK		2

#define RGBLED_ENMASK		3
#define LEDBLNK_ON(time)	((time < 100) ? 0 :			\
				(time < 500) ? time/100-1 :		\
				(time < 3250) ? (time-500)/250+4 : 15)

#define LEDBLNK_OFF(time)	((time < 500) ? 0x00 :			\
				(time < 5000) ? time/500 :		\
				(time < 8000) ? (time-5000)/1000+10 :	 \
				(time < 12000) ? (time-8000)/2000+13 : 15)

extern unsigned int lcdtype;

enum led_color {
	LED_RED,
	LED_GREEN,
	LED_BLUE,
	LED_WHITE,
};

enum s2mu004_led_pattern {
	PATTERN_OFF,
	CHARGING,
	CHARGING_ERR,
	MISSED_NOTI,
	LOW_BATTERY,
	FULLY_CHARGED,
	POWERING,
};

struct s2mu004_rgb_pdata {
	const char *name;
	int col[S2MU004_LED_NUM];
	u32 brightness;
	u32 lp_brightness;
	u32 nleds;
	u32 octa_color;
	u32 ratio[S2MU004_LED_NUM];
	u32 ratio_low[S2MU004_LED_NUM];
};

#endif /* __LEDS_MAX77843_RGB_H__ */
