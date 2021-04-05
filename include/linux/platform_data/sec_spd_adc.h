/*
 * sec_adc_detector.h - SEC ADC Detector
 *
 *  Copyright (c) 2019 Samsung Electronics Co., Ltd.
 *      http://www.samsung.com
 *  Insun Choi <insun77.choi@samsung.com>, Le Hoang Anh <anh.lh@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __LINUX_SEC_ADC_DETECTOR_H
#define __LINUX_SEC_ADC_DETECTOR_H __FILE__

/**
 * struct sec_adc_table - table for check adc value for sec adc detector
 * driver
 * @adc_min: minimum adc value
 * @adc_max: maximum adc value
 */
struct sec_adc_table {
	int adc_min;
	int adc_max;
	int sub_rev;
};

/**
 * struct sec_adc_plaform_data - init data for sec adc detector driver
 * @adc_channel: adc channel that connected to adc detector
 * @adc_table: array of adc to voltage data
 * @adc_arr_size: size of adc_table
 */
struct sec_adc_platform_data {
	unsigned int adc_channel;
	unsigned int adc_arr_size;
	struct sec_adc_table *adc_table;
	const char *name;
};

#endif /* __LINUX_SEC_ADC_DETECTOR_H */
