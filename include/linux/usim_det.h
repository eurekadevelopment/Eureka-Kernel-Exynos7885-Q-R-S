/*
 * Copyright (C) 2016 Samsung Electronics Co.Ltd
 * http://www.samsung.com
 *
 * USIM_DET driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
*/

#ifndef __USIM_DET_H__
#define __USIM_DET_H__

#define USIM_DETECT_NAME		"usim_detect"
#define USIM_DETECT_NAME0		"usim_detect0"
#define USIM_DETECT_NAME1		"usim_detect1"
#define USIM_LOW_DETECT_COUNT_DEFAULT		1
#define USIM_HIGH_DETECT_COUNT_DEFAULT		5
#define USIM_CHECK_DELAY_MSEC_DEFAULT		100

struct usim_det_data {
	char *name;

	u32 num_of_usim_det;
	
	u32 usim_check_delay_msec;
	u32 usim_high_detect_count;
	u32 usim_low_detect_count;

	int usim_det0_irq;
	int gpio_usim_det0;
	u32 usim_det0_gpio_flags;

	int usim_det1_irq;
	int gpio_usim_det1;
	u32 usim_det1_gpio_flags;

	bool usim0_det;
	bool usim1_det;

	unsigned int int_usim0_det;
	unsigned int int_usim1_det;

	unsigned int mbx_ap_united_status;
	unsigned int sbi_usim0_det_mask;
	unsigned int sbi_usim0_det_pos;
	unsigned int sbi_usim1_det_mask;
	unsigned int sbi_usim1_det_pos;

	struct work_struct usim_det0_work;
	struct work_struct usim_det1_work;

	struct notifier_block modem_nb;
	int modem_state;
};

#endif
