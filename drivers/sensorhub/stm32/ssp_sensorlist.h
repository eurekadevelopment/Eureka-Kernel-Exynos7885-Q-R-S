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

#define SENSOR_NAME_MAX_LEN		35


/* Sensors's reporting mode */
#define REPORT_MODE_CONTINUOUS	0
#define REPORT_MODE_ON_CHANGE	1
#define REPORT_MODE_SPECIAL	2
#define REPORT_MODE_UNKNOWN	3

/*
	char name[SENSOR_NAME_MAX_LEN];
	bool enable;
	int report_mode;
	int get_data_len;
	int report_data_len;
*/
#define SENSOR_INFO_UNKNOWN				{"", false,	REPORT_MODE_UNKNOWN, 0, 0}
#define SENSOR_INFO_ACCELEROMETER 		{"accelerometer_sensor", true,	REPORT_MODE_CONTINUOUS,	6, 6}
#define SENSOR_INFO_GEOMAGNETIC_FIELD 	{"geomagnetic_sensor", true, REPORT_MODE_CONTINUOUS, 7, 7}
#define SENSOR_INFO_GYRO 				{"gyro_sensor", true, REPORT_MODE_CONTINUOUS, 6, 6}
#define SENSOR_INFO_LIGHT				{"light_sensor", true, REPORT_MODE_ON_CHANGE, 18, 18}
#define SENSOR_INFO_PRESSURE				{"pressure_sensor",	true, REPORT_MODE_CONTINUOUS, 6, 14}
#define SENSOR_INFO_PROXIMITY			{"proximity_sensor", true, REPORT_MODE_ON_CHANGE, 2, 2}
#define SENSOR_INFO_ROTATION_VECTOR		{"rotation_vector_sensor", false, REPORT_MODE_CONTINUOUS, 17, 17}
#define SENSOR_INFO_MAGNETIC_FIELD_UNCALIBRATED 	{"uncal_geomagnetic_sensor", true, REPORT_MODE_CONTINUOUS, 12, 12}
#define SENSOR_INFO_GYRO_UNCALIBRATED 	{"uncal_gyro_sensor", true, REPORT_MODE_CONTINUOUS, 12, 12}
#define SENSOR_INFO_SIGNIFICANT_MOTION 	{"sig_motion_sensor", true, REPORT_MODE_SPECIAL, 1, 1}
#define SENSOR_INFO_STEP_DETECTOR		{"step_det_sensor", true, REPORT_MODE_ON_CHANGE, 1, 1}
#define SENSOR_INFO_STEP_COUNTER		{"step_cnt_sensor", true, REPORT_MODE_ON_CHANGE, 4, 12}
#define SENSOR_INFO_GEOMAGNETIC_ROTATION_VECTOR {"geomagnetic_rotation_vector_sensor", true, REPORT_MODE_CONTINUOUS, 17, 17}
#define SENSOR_INFO_TILT_DETECTOR		{"tilt_detector", true, REPORT_MODE_ON_CHANGE, 1, 1}
#define SENSOR_INFO_PICK_UP_GESTURE		{"pickup_gesture", true, REPORT_MODE_CONTINUOUS, 1, 1}
#define SENSOR_INFO_PROXIMITY_RAW		{"", true, REPORT_MODE_ON_CHANGE, 1, 0}
#define SENSOR_INFO_GEOMAGNETIC_POWER	{"geomagnetic_power", true, REPORT_MODE_CONTINUOUS, 6, 6}
#define SENSOR_INFO_INTERRUPT_GYRO		{"interrupt_gyro_sensor", true, REPORT_MODE_ON_CHANGE, 6, 6}
#define SENSOR_INFO_META				{"meta_event", true, REPORT_MODE_CONTINUOUS, 8, 8}

#endif
