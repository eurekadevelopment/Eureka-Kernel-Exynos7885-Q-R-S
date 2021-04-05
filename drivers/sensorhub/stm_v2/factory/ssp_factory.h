/*
 *  Copyright (C) 2015, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#ifndef __SSP_FACTORY_H__
#define __SSP_FACTORY_H__

#include "../ssp.h"

/* Gyroscope DPS */
#define GYROSCOPE_DPS250                250
#define GYROSCOPE_DPS500                500
#define GYROSCOPE_DPS2000               2000

/* Proxy threshold */
#define DEFUALT_HIGH_THRESHOLD          50
#define DEFUALT_LOW_THRESHOLD           35
#define DEFUALT_DETECT_HIGH_THRESHOLD           200
#define DEFUALT_DETECT_LOW_THRESHOLD            190

/* Magnetic */
#define PDC_SIZE                        27

struct ssp_data;

#ifdef CONFIG_SENSORS_SSP_ACCELOMETER
/* accleometer sensor */
struct accelometer_sensor_operations {
	ssize_t (*get_accel_name)(char *);
	ssize_t (*get_accel_vendor)(char *);
	ssize_t (*get_accel_calibration)(struct ssp_data *, char *);
	ssize_t (*set_accel_calibration)(struct ssp_data *, const char *);
	ssize_t (*get_accel_raw_data)(struct ssp_data *, char *);
	ssize_t (*get_accel_reactive_alert)(struct ssp_data *, char *);
	ssize_t (*set_accel_reactive_alert)(struct ssp_data *, const char *);
	ssize_t (*get_accel_selftest)(struct ssp_data *, char *);
	ssize_t (*set_accel_lowpassfilter)(struct ssp_data *, const char *);
};

void initialize_accel_factorytest(struct ssp_data *);
void remove_accel_factorytest(struct ssp_data *);
int accel_open_calibration(struct ssp_data *);
int set_accel_cal(struct ssp_data *);

#if defined(CONFIG_SENSORS_SSP_ACCELOMETER_LSM6DSL)
void accelometer_lsm6dsl_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_ACCELOMETER_K6DS3TR)
void accelometer_k6ds3tr_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_ACCELOMETER_MPU6500)
void accelometer_mpu6500_function_pointer_initialize(struct ssp_data *);
#else
void accelometer_bmi168_function_pointer_initialize(struct ssp_data *);
#endif
#endif

#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
/* gyroscope sensor */
void initialize_gyro_factorytest(struct ssp_data *);
void remove_gyro_factorytest(struct ssp_data *);
int gyro_open_calibration(struct ssp_data *data);
int set_gyro_cal(struct ssp_data *data);
int save_gyro_cal_data(struct ssp_data *data, s16 *cal_data);

struct gyroscope_sensor_operations {
	ssize_t (*get_gyro_name)(char *);
	ssize_t (*get_gyro_vendor)(char *);
	ssize_t (*get_gyro_power_off)(char *);
	ssize_t (*get_gyro_power_on)(char *);
	ssize_t (*get_gyro_temperature)(struct ssp_data *, char *);
	ssize_t (*get_gyro_selftest)(struct ssp_data *, char *);
	ssize_t (*get_gyro_selftest_dps)(struct ssp_data *, char *);
	ssize_t (*set_gyro_selftest_dps)(struct ssp_data *, const char *);
};

#if defined(CONFIG_SENSORS_SSP_GYROSCOPE_LSM6DSL)
void gyroscope_lsm6dsl_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_GYROSCOPE_K6DS3TR)
void gyroscope_k6ds3tr_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_GYROSCOPE_MPU6500)
void gyroscope_mpu6500_function_pointer_initialize(struct ssp_data *);
#else
void gyroscope_bmi168_function_pointer_initialize(struct ssp_data *);
#endif
#endif

#ifdef CONFIG_SENSORS_SSP_MAGNETIC
/* magnetic sensor */
void initialize_magnetic_factorytest(struct ssp_data *);
void remove_magnetic_factorytest(struct ssp_data *);
int initialize_magnetic_sensor(struct ssp_data *);

struct magnetic_sensor_operations {
	ssize_t (*get_magnetic_name)(char *);
	ssize_t (*get_magnetic_vendor)(char *);
	ssize_t (*get_magnetic_adc)(struct ssp_data *, char *);
	ssize_t (*get_magnetic_dac)(struct ssp_data *, char *);
	ssize_t (*get_magnetic_raw_data)(struct ssp_data *, char *);
	ssize_t (*set_magnetic_raw_data)(struct ssp_data *, const char *);
	ssize_t (*get_magnetic_asa)(struct ssp_data *, char *);
	ssize_t (*get_magnetic_status)(struct ssp_data *, char *);
	ssize_t (*get_magnetic_logging_data)(struct ssp_data *, char *);
	ssize_t (*get_magnetic_hw_offset)(struct ssp_data *, char *);
	ssize_t (*get_magnetic_matrix)(struct ssp_data *, char *);
	ssize_t (*set_magnetic_matrix)(struct ssp_data *, const char *);
	ssize_t (*get_magnetic_selftest)(struct ssp_data *, char *);
};

#if defined(CONFIG_SENSORS_SSP_MAGNETIC_AK09918C)
void magnetic_ak09918c_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_MAGNETIC_AK09916C)
void magnetic_ak09916c_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_MAGNETIC_AK09911)
void magnetic_ak09911_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_MAGNETIC_LSM303AH)
void magnetic_lsm303ah_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_MAGNETIC_YAS539)
void magnetic_yas539_function_pointer_initialize(struct ssp_data *);
#endif
#endif

#ifdef CONFIG_SENSORS_SSP_PROXIMITY
/* proximity sensor */
struct proximity_sensor_operations {
	ssize_t (*get_proximity_name)(char *);
	ssize_t (*get_proximity_vendor)(char *);
	ssize_t (*get_proximity_probe_status)(struct ssp_data *, char *);
	ssize_t (*get_threshold_high)(struct ssp_data *, char *);
	ssize_t (*set_threshold_high)(struct ssp_data *, const char *);
	ssize_t (*get_threshold_low)(struct ssp_data *, char *);
	ssize_t (*set_threshold_low)(struct ssp_data *, const char *);
	ssize_t (*get_threshold_detect_high)(struct ssp_data *, char *);
	ssize_t (*set_threshold_detect_high)(struct ssp_data *, const char *);
	ssize_t (*get_threshold_detect_low)(struct ssp_data *, char *);
	ssize_t (*set_threshold_detect_low)(struct ssp_data *, const char *);

	ssize_t (*get_proximity_cancel)(struct ssp_data *, char *);
	ssize_t (*set_proximity_cancel)(struct ssp_data *, const char *);
	ssize_t (*get_proximity_cancel_pass)(struct ssp_data *, char *);
	u16(*get_proximity_raw_data)(struct ssp_data *);

	ssize_t (*get_proximity_trim_value)(struct ssp_data *, char *);
	ssize_t (*get_proximity_avg_raw_data)(struct ssp_data *, char *);
	ssize_t (*set_proximity_avg_raw_data)(struct ssp_data *, const char *);

	ssize_t (*get_proximity_setting)(char *);
	ssize_t (*set_proximity_setting)(struct ssp_data *, const char *);
	
	ssize_t (*get_proximity_trim_check)(struct ssp_data *, char *);	
};

void initialize_prox_factorytest(struct ssp_data *);
void remove_prox_factorytest(struct ssp_data *);
int proximity_open_calibration(struct ssp_data *);
void get_proximity_threshold(struct ssp_data *);

#if defined(CONFIG_SENSORS_SSP_PROXIMITY_TMD3700)
void proximity_tmd3700_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
void proximity_tmd3725_auto_cal_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_PROXIMITY_TMD3725)
void proximity_tmd3725_function_pointer_initialize(struct ssp_data *);
#else
void proximity_tmg399x_function_pointer_initialize(struct ssp_data *);
#endif
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT
/* light sensor */
struct light_sensor_operations {
	ssize_t (*get_light_name)(char *);
	ssize_t (*get_light_vendor)(char *);
	ssize_t (*get_lux)(struct ssp_data *, char *);
	ssize_t (*get_light_data)(struct ssp_data *, char *);
	ssize_t (*get_light_coefficient)(struct ssp_data *, char *);
};

void initialize_light_factorytest(struct ssp_data *);
void remove_light_factorytest(struct ssp_data *);

#if defined(CONFIG_SENSORS_SSP_LIGHT_TMD3700)
void light_tmd3700_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_LIGHT_TMD3725)
void light_tmd3725_function_pointer_initialize(struct ssp_data *);
#else
void light_tmg399x_function_pointer_initialize(struct ssp_data *);
#endif
#endif

#ifdef CONFIG_SENSORS_SSP_BAROMETER
/* barometer sensor */
void initialize_barometer_factorytest(struct ssp_data *);
void remove_barometer_factorytest(struct ssp_data *);
int pressure_open_calibration(struct ssp_data *);

struct barometer_sensor_operations {
	ssize_t (*get_barometer_name)(char *);
	ssize_t (*get_barometer_vendor)(char *);
	ssize_t (*get_barometer_eeprom_check)(struct ssp_data *, char *);
	ssize_t (*get_barometer_calibration)(struct ssp_data *, char *);
	ssize_t (*set_barometer_calibration)(struct ssp_data *, const char *);
	ssize_t (*set_barometer_sea_level_pressure)(struct ssp_data *, const char *);
	ssize_t (*get_barometer_temperature)(struct ssp_data *, char *);
};

#if defined(CONFIG_SENSORS_SSP_BAROMETER_LPS22H)
void barometer_lps22h_function_pointer_initialize(struct ssp_data *);
#elif defined(CONFIG_SENSORS_SSP_BAROMETER_LPS25H)
void barometer_lps25h_function_pointer_initialize(struct ssp_data *);
#else
void barometer_bmp280_function_pointer_initialize(struct ssp_data *);
#endif
#endif

#ifdef CONFIG_SENSORS_SSP_MOBEAM
void initialize_mobeam(struct ssp_data *data);
void remove_mobeam(struct ssp_data *data);
#endif

#endif
