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

#ifndef __SSP_SENSORS_H__
#define __SSP_SENSORS_H__

#define SENSOR_NAME_MAX_LEN             35

/* Sensors's reporting mode */
#define REPORT_MODE_CONTINUOUS          0
#define REPORT_MODE_ON_CHANGE           1
#define REPORT_MODE_SPECIAL             2
#define REPORT_MODE_UNKNOWN             3

/*
        char name[SENSOR_NAME_MAX_LEN];
        bool enable;
        int report_mode;
        int get_data_len;
        int report_data_len;
*/

#ifdef CONFIG_SENSORS_SSP_ACCELOMETER
#define ACCELEROMETER_ENABLED 		true
#else
#define ACCELEROMETER_ENABLED 		false
#endif
#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
#define GYROSCOPE_ENABLED 			true
#else
#define GYROSCOPE_ENABLED 			false
#endif
#ifdef CONFIG_SENSORS_SSP_MAGNETIC
#define MAGNETIC_ENABLED			true
#else
#define MAGNETIC_ENABLED			false
#endif
#ifdef CONFIG_SENSORS_SSP_PROXIMITY
#define PROXIMITY_ENABLED			true
#else
#define PROXIMITY_ENABLED			false
#endif
#ifdef CONFIG_SENSORS_SSP_LIGHT
#define LIGHT_ENABLED				true
#else
#define LIGHT_ENABLED				false
#endif
#ifdef CONFIG_SENSORS_SSP_BAROMETER
#define PRESSURE_ENABLED			true
#else
#define PRESSURE_ENABLED			false
#endif

#define SIG_MOTION_ENABLED			true
#define STEP_DETECTOR_ENABLED 		true
#define STEP_COUNTER_ENABLED		true
#define	GEO_RV_ENABLED				true
#define	RV_ENABLED					false
#define	TILT_DETECTOR_ENABLED		true
#define PICK_UP_ENABLED				true
#define INT_GYRO_ENABLED			true
#define WAKE_UP_MOTION_ENABLED      true

#define SENSOR_INFO_UNKNOWN                                     {"", false, REPORT_MODE_UNKNOWN, 0, 0}
#define SENSOR_INFO_ACCELEROMETER                               {"accelerometer_sensor", ACCELEROMETER_ENABLED, REPORT_MODE_CONTINUOUS, 6, 6}
#define SENSOR_INFO_GEOMAGNETIC_FIELD                           {"geomagnetic_sensor", MAGNETIC_ENABLED, REPORT_MODE_CONTINUOUS, 7, 7}
#define SENSOR_INFO_GYRO                                        {"gyro_sensor", GYROSCOPE_ENABLED, REPORT_MODE_CONTINUOUS, 6, 6}
#define SENSOR_INFO_PRESSURE                                    {"pressure_sensor", PRESSURE_ENABLED, REPORT_MODE_CONTINUOUS, 6, 14}
#define SENSOR_INFO_ROTATION_VECTOR                             {"rotation_vector_sensor", RV_ENABLED, REPORT_MODE_CONTINUOUS, 17, 17}
#define SENSOR_INFO_MAGNETIC_FIELD_UNCALIBRATED                 {"uncal_geomagnetic_sensor", MAGNETIC_ENABLED, REPORT_MODE_CONTINUOUS, 12, 12}
#define SENSOR_INFO_GYRO_UNCALIBRATED                           {"uncal_gyro_sensor", GYROSCOPE_ENABLED, REPORT_MODE_CONTINUOUS, 12, 12}
#define SENSOR_INFO_SIGNIFICANT_MOTION                          {"sig_motion_sensor", SIG_MOTION_ENABLED, REPORT_MODE_SPECIAL, 1, 1}
#define SENSOR_INFO_STEP_DETECTOR                               {"step_det_sensor", STEP_DETECTOR_ENABLED, REPORT_MODE_ON_CHANGE, 1, 1}
#define SENSOR_INFO_STEP_COUNTER                                {"step_cnt_sensor", STEP_COUNTER_ENABLED, REPORT_MODE_ON_CHANGE, 4, 12}
#define SENSOR_INFO_GEOMAGNETIC_ROTATION_VECTOR                 {"geomagnetic_rotation_vector_sensor", GEO_RV_ENABLED, REPORT_MODE_CONTINUOUS, 17, 17}
#define SENSOR_INFO_TILT_DETECTOR                               {"tilt_detector", TILT_DETECTOR_ENABLED, REPORT_MODE_ON_CHANGE, 1, 1}
#define SENSOR_INFO_PICK_UP_GESTURE                             {"pickup_gesture", PICK_UP_ENABLED, REPORT_MODE_CONTINUOUS, 1, 1}
#define SENSOR_INFO_PROXIMITY_RAW                               {"", PROXIMITY_ENABLED, REPORT_MODE_ON_CHANGE, 1, 0}
#define SENSOR_INFO_GEOMAGNETIC_POWER                           {"geomagnetic_power", MAGNETIC_ENABLED, REPORT_MODE_CONTINUOUS, 6, 6}
#define SENSOR_INFO_INTERRUPT_GYRO                              {"interrupt_gyro_sensor", INT_GYRO_ENABLED, REPORT_MODE_ON_CHANGE, 6, 6}
#define SENSOR_INFO_SCONTEXT                                    {"scontext_iio", true, REPORT_MODE_CONTINUOUS, 0, 64}
#define SENSOR_INFO_WAKE_UP_MOTION                              {"wake_up_motion", WAKE_UP_MOTION_ENABLED, REPORT_MODE_ON_CHANGE, 1, 1}

#if ANDROID_VERSION >= 90000
#define SENSOR_INFO_LIGHT                                       {"light_sensor", LIGHT_ENABLED, REPORT_MODE_ON_CHANGE, 18, 4}
#define SENSOR_INFO_LIGHT_CCT                                   {"light_cct_sensor", LIGHT_ENABLED, REPORT_MODE_ON_CHANGE, 18, 8}
#define SENSOR_INFO_PROXIMITY                                   {"proximity_sensor", PROXIMITY_ENABLED, REPORT_MODE_ON_CHANGE, 2, 1}
#else
#define SENSOR_INFO_LIGHT                                       {"light_sensor", LIGHT_ENABLED, REPORT_MODE_ON_CHANGE, 18, 18}
#define SENSOR_INFO_LIGHT_CCT                                   {"light_cct_sensor", LIGHT_ENABLED, REPORT_MODE_ON_CHANGE, 18, 18}
#define SENSOR_INFO_PROXIMITY                                   {"proximity_sensor", PROXIMITY_ENABLED, REPORT_MODE_ON_CHANGE, 2, 2}
#endif
#endif
