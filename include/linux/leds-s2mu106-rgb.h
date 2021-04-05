/*
 * leds-s2mu106-rgb.h - RGB-led driver for Samsung S2MU106
 *
 * Copyright (C) 2019 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LEDS_S2MU106_RGB_H__
#define __LEDS_S2MU106_RGB_H__

/* s2mu106 LED numbers */
#define S2MU106_LED_NUM	4

/* s2mu106_REG_LED_DUR */
#define S2MU106_LED_ON_DUR	0xF0
#define S2MU106_LED_OFF_DUR	0x0F

/* s2mu106_REG_LEDRMP */
#define S2MU106_RAMP_UP		0xF0
#define S2MU106_RAMP_DN		0x0F

#define LED_R_MASK		0x00FF0000
#define LED_G_MASK		0x0000FF00
#define LED_B_MASK		0x000000FF
#define LED_MAX_CURRENT		0xFF

/* s2mu106_Reset */
#define RGBLED_REG_RESET_MASK	0x01

/* s2mu106_STATE*/
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

enum s2mu106_led_pattern {
	PATTERN_OFF,
	CHARGING,
	CHARGING_ERR,
	MISSED_NOTI,
	LOW_BATTERY,
	FULLY_CHARGED,
	POWERING,
};

struct s2mu106_rgb_pdata {
	const char *name;
	int col[S2MU106_LED_NUM];
	u32 brightness;
	u32 lp_brightness;
	u32 nleds;
	u32 octa_color;
	u32 ratio[S2MU106_LED_NUM];
	u32 ratio_low[S2MU106_LED_NUM];
};

#define S2MU106_RGB_LED_EN	0x43
#define S2MU106_RGB_LED1_CURR	0x44
#define S2MU106_RGB_LED2_CURR	0x45
#define S2MU106_RGB_LED3_CURR	0x46
#define S2MU106_RGB_LED4_CURR	0x47
#define S2MU106_RGB_LED1_RAMP	0x48
#define S2MU106_RGB_LED1_DUR	0x49
#define S2MU106_RGB_LED2_RAMP	0x4A
#define S2MU106_RGB_LED2_DUR	0x4B
#define S2MU106_RGB_LED3_RAMP	0x4C
#define S2MU106_RGB_LED3_DUR	0x4D
#define S2MU106_RGB_LED4_RAMP	0x4E
#define S2MU106_RGB_LED4_DUR	0x4F
#define S2MU106_RGB_LED_CTRL0	0x51
#define S2MU106_RGB_LED_CTRL1	0x52

#endif
