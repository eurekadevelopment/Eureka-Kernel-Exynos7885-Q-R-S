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

#ifndef __SSP_PRJ_H__
#define __SSP_PRJ_H__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/rtc.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/battery/sec_battery.h>

#include "../../staging/android/android_alarm.h"
#include "factory/ssp_factory.h"
#include "factory/ssp_mcu.h"
#include "ssp_sensorlist.h"
#include "ssp_data.h"
#include "ssp_debug.h"
#include "ssp_dev.h"
#include "ssp_firmware.h"
#include "ssp_iio.h"
#include "ssp_sensorhub.h"
#include "ssp_spi.h"
#include "ssp_sysfs.h"
#include "sensors_core.h"


#undef CONFIG_SEC_DEBUG
#define CONFIG_SEC_DEBUG	0

#define DEFUALT_POLLING_DELAY	(200 * NSEC_PER_MSEC)
#define DATA_PACKET_SIZE	2000

/* AP -> SSP Instruction */
#define MSG2SSP_INST_BYPASS_SENSOR_ADD		0xA1
#define MSG2SSP_INST_BYPASS_SENSOR_REMOVE	0xA2
#define MSG2SSP_INST_REMOVE_ALL			0xA3
#define MSG2SSP_INST_CHANGE_DELAY		0xA4
#define MSG2SSP_INST_LIBRARY_ADD		0xB1
#define MSG2SSP_INST_LIBRARY_REMOVE		0xB2
#define MSG2SSP_INST_LIB_NOTI			0xB4
#define MSG2SSP_INST_LIB_DATA			0xC1

#define MSG2SSP_AP_FUSEROM			0X01

#define MSG2SSP_AP_MCU_SET_GYRO_CAL		0xCD
#define MSG2SSP_AP_MCU_SET_ACCEL_CAL		0xCE
#define MSG2SSP_AP_STATUS_SHUTDOWN		0xD0
#define MSG2SSP_AP_STATUS_WAKEUP		0xD1
#define MSG2SSP_AP_STATUS_SLEEP			0xD2
#define MSG2SSP_AP_STATUS_RESUME		0xD3
#define MSG2SSP_AP_STATUS_SUSPEND		0xD4
#define MSG2SSP_AP_STATUS_RESET			0xD5
#define MSG2SSP_AP_STATUS_POW_CONNECTED		0xD6
#define MSG2SSP_AP_STATUS_POW_DISCONNECTED	0xD7
#define MSG2SSP_AP_TEMPHUMIDITY_CAL_DONE	0xDA
#define MSG2SSP_AP_MCU_SET_DUMPMODE		0xDB
#define MSG2SSP_AP_MCU_BATCH_FLUSH		0xDD
#define MSG2SSP_AP_MCU_BATCH_COUNT		0xDF

#define MSG2SSP_AP_WHOAMI			0x0F
#define MSG2SSP_AP_FIRMWARE_REV			0xF0
#define MSG2SSP_AP_SENSOR_FORMATION		0xF1
#define MSG2SSP_AP_SENSOR_PROXTHRESHOLD		0xF2
#define MSG2SSP_AP_SENSOR_BARCODE_EMUL		0xF3
#define MSG2SSP_AP_SENSOR_SCANNING		0xF4
#define MSG2SSP_AP_SET_MAGNETIC_HWOFFSET	0xF5
#define MSG2SSP_AP_GET_MAGNETIC_HWOFFSET	0xF6
#define MSG2SSP_AP_SENSOR_GESTURE_CURRENT	0xF7
#define MSG2SSP_AP_GET_THERM			0xF8
#define MSG2SSP_AP_GET_BIG_DATA			0xF9
#define MSG2SSP_AP_SET_BIG_DATA			0xFA
#define MSG2SSP_AP_START_BIG_DATA		0xFB
#define MSG2SSP_AP_SET_MAGNETIC_STATIC_MATRIX	0xFD
#define MSG2SSP_AP_SET_HALL_THRESHOLD		0xE9
#define MSG2SSP_AP_SENSOR_TILT			0xEA
#define MSG2SSP_AP_MCU_SET_TIME			0xFE
#define MSG2SSP_AP_MCU_GET_TIME			0xFF
#define MSG2SSP_AP_MOBEAM_DATA_SET		0x31
#define MSG2SSP_AP_MOBEAM_REGISTER_SET		0x32
#define MSG2SSP_AP_MOBEAM_COUNT_SET		0x33
#define MSG2SSP_AP_MOBEAM_START			0x34
#define MSG2SSP_AP_MOBEAM_STOP			0x35
#define MSG2SSP_AP_GEOMAG_LOGGING		0x36
#define MSG2SSP_AP_SENSOR_LPF			0x37
#define MSG2SSP_AP_IRDATA_SEND			0x38
#define MSG2SSP_AP_IRDATA_SEND_RESULT		0x39
#define MSG2SSP_AP_PROX_GET_TRIM		0x40
#define MSG2SSP_AP_SET_6AXIS_PIN		0x7D
#define MSG2SSP_AP_WHOAMI_6AXIS			0x7F
#define MSG2SSP_AP_SET_PROX_SETTING		0x48
#define MSG2SSP_AP_SET_LIGHT_COEF 		0x49
#define MSG2SSP_AP_GET_LIGHT_COEF		0x50

/* gyro calibration state*/
#define SH_MSG2AP_GYRO_CALIBRATION_START   0x43
#define SH_MSG2AP_GYRO_CALIBRATION_STOP    0x44
#define SH_MSG2AP_GYRO_CALIBRATION_EVENT_OCCUR  0x45

#define SUCCESS	1
#define FAIL	0
#define ERROR	-1

/* gyro calibration state*/
#define GYRO_CALIBRATION_STATE_NOT_YET		0
#define GYRO_CALIBRATION_STATE_REGISTERED 	1
#define GYRO_CALIBRATION_STATE_EVENT_OCCUR  2
#define GYRO_CALIBRATION_STATE_DONE 		3

#define BIG_DATA_SENSOR_TYPE_MAX								31

#define ssp_dbg(fmt, ...) do { \
	pr_debug("[SSP] " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define ssp_info(fmt, ...) do { \
	pr_info("[SSP] " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define ssp_err(fmt, ...) do { \
	pr_err("[SSP] " fmt "\n", ##__VA_ARGS__); \
	} while (0)

#define ssp_dbgf(fmt, ...) do { \
	pr_debug("[SSP] %20s(%4d): " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)

#define ssp_infof(fmt, ...) do { \
	pr_info("[SSP] %20s(%4d): " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)

#define ssp_errf(fmt, ...) do { \
	pr_err("[SSP] %20s(%4d): " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); \
	} while (0)


/* SSP Binary Type */
enum {
	KERNEL_BINARY = 0,
	KERNEL_CRASHED_BINARY,
	UMS_BINARY,
};

/*
 * SENSOR_DELAY_SET_STATE
 * Check delay set to avoid sending ADD instruction twice
 */
enum {
	INITIALIZATION_STATE = 0,
	NO_SENSOR_STATE,
	ADD_SENSOR_STATE,
	RUNNING_SENSOR_STATE,
};

/* Firmware download STATE */
enum {
	FW_DL_STATE_FAIL = -1,
	FW_DL_STATE_NONE = 0,
	FW_DL_STATE_NEED_TO_SCHEDULE,
	FW_DL_STATE_SCHEDULED,
	FW_DL_STATE_DOWNLOADING,
	FW_DL_STATE_SYNC,
	FW_DL_STATE_DONE,
};

/* SSP_INSTRUCTION_CMD */
enum {
	REMOVE_SENSOR = 0,
	ADD_SENSOR,
	CHANGE_DELAY,
	GO_SLEEP,
	REMOVE_LIBRARY,
	ADD_LIBRARY,
};

/* SENSOR_TYPE */
#if 0
enum {
	ACCELEROMETER_SENSOR = 1,
	GYROSCOPE_SENSOR,
	GEOMAGNETIC_UNCALIB_SENSOR,
	GEOMAGNETIC_RAW, 
	GEOMAGNETIC_SENSOR,
	SENSOR_TYPE_PRESSURE,
	PROXIMITY_SENSOR,
	LIGHT_SENSOR,
	PROXIMITY_RAW,
	ORIENTATION_SENSOR,
	STEP_DETECTOR = 12,
	SIG_MOTION_SENSOR,
	GYRO_UNCALIB_SENSOR,
	GAME_ROTATION_VECTOR = 15,
	ROTATION_VECTOR,
	STEP_COUNTER,
	TILT_DETECTOR,
	SENSOR_TYPE_PICK_UP_GESTURE,
	META_SENSOR,//
	SENSOR_TYPE_MAX = 30,
	BATCH_META_SENSOR = 200,//
};
#endif

#define BATCH_META_SENSOR 200

/* Sensor types defined by android */
#define SENSOR_TYPE_ACCELEROMETER                    (1)
#define SENSOR_TYPE_GEOMAGNETIC_FIELD                (2)
#define SENSOR_TYPE_ORIENTATION                      (3)
#define SENSOR_TYPE_GYROSCOPE                        (4)
#define SENSOR_TYPE_LIGHT                            (5)
#define SENSOR_TYPE_PRESSURE                         (6)
#define SENSOR_TYPE_TEMPERATURE                      (7) //Deprecated
#define SENSOR_TYPE_PROXIMITY                        (8)
#define SENSOR_TYPE_GRAVITY                          (9)
#define SENSOR_TYPE_LINEAR_ACCELERATION             (10)
#define SENSOR_TYPE_ROTATION_VECTOR                 (11)
#define SENSOR_TYPE_RELATIVE_HUMIDITY               (12)
#define SENSOR_TYPE_AMBIENT_TEMPERATURE             (13)
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED     (14)
#define SENSOR_TYPE_GAME_ROTATION_VECTOR            (15)
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED          (16)
#define SENSOR_TYPE_SIGNIFICANT_MOTION              (17)
#define SENSOR_TYPE_STEP_DETECTOR                   (18)
#define SENSOR_TYPE_STEP_COUNTER                    (19)
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR     (20)
#define SENSOR_TYPE_HEART_RATE                      (21)
#define SENSOR_TYPE_TILT_DETECTOR                   (22)
#define SENSOR_TYPE_WAKE_GESTURE                    (23)
#define SENSOR_TYPE_GLANCE_GESTURE                  (24)
#define SENSOR_TYPE_PICK_UP_GESTURE                 (25)
#define SENSOR_TYPE_WRIST_TILT_GESTURE              (26)
#define SENSOR_TYPE_META                            (27)

/* Sensor types defined by Samsung */
#define SENSOR_TYPE_DEVICE_PRIVATE_BASE             (SENSOR_TYPE_META)
#define SENSOR_TYPE_ACCELEROMETER_INT               (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 1)
#define SENSOR_TYPE_PROXIMITY_RAW                   (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 2)
#define SENSOR_TYPE_GEOMAGNETIC_POWER               (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 3)
#define SENSOR_TYPE_INTERRUPT_GYRO                  (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 4)
#define SENSOR_TYPE_MAX                             (SENSOR_TYPE_DEVICE_PRIVATE_BASE + 5)
#define SENSOR_CONTROL_BASE                         (100)
#define SENSOR_TYPE_MOBEAM                          (SENSOR_CONTROL_BASE + 1)

enum {
	AP2HUB_READ = 0,
	AP2HUB_WRITE,
	HUB2AP_WRITE,
	AP2HUB_READY,
	AP2HUB_RETURN
};

enum {
	BIG_TYPE_DUMP = 0,
	BIG_TYPE_READ_LIB,
	BIG_TYPE_MAX,
};

enum {
	SIX_AXIS_MPU6500 = 0,
	SIX_AXIS_K6DS3,
	SIX_AXIS_BMI160,
	SIX_AXIS_LSM303AH,
	SIX_AXIS_MAX,
};

extern struct class *sensors_event_class;

struct sensor_value {
	union {
		struct { /* accel, gyro, mag */
			s16 x;
			s16 y;
			s16 z;
			u32 gyro_dps;
		} __attribute__((__packed__));
		struct { /*calibrated mag, gyro*/
			s16 cal_x;
			s16 cal_y;
			s16 cal_z;
			u8 accuracy;
		} __attribute__((__packed__));
		struct { /*uncalibrated mag, gyro*/
			s16 uncal_x;
			s16 uncal_y;
			s16 uncal_z;
			s16 offset_x;
			s16 offset_y;
			s16 offset_z;
		} __attribute__((__packed__));
		struct { /* rotation vector */
			s32 quat_a;
			s32 quat_b;
			s32 quat_c;
			s32 quat_d;
			u8 acc_rot;
		} __attribute__((__packed__));
		struct { /* light */
			u32 lux;
			s32 cct;
			u16 r;
			u16 g;
			u16 b;
			u16 w;
			u8 a_time;
			u8 a_gain;
		} __attribute__((__packed__));
		struct { /* pressure */
			s32 pressure;
			s16 temperature;
			s32 pressure_cal;
			s32 pressure_sealevel;
		} __attribute__((__packed__));
		struct { /* step detector */
			u8 step_det;
		};
		struct { /* step counter */
			u32 step_diff;
			u64 step_total;
		} __attribute__((__packed__));
		struct { /* proximity */
			u8 prox;
			u8 prox_ex;
		} __attribute__((__packed__));
		struct { /* proximity raw */
			u8 prox_raw[4];
		};
		struct { /* significant motion */
			u8 sig_motion;
		};
		struct { /* tilt detector */
			u8 tilt_detector;
		};
		struct { /* pickup gesture */
			u8 pickup_gesture;
		};
		struct meta_data_event { /* meta data */
			s32 what;
			s32 sensor;
		} __attribute__((__packed__)) meta_data;
		u8 data[20];
	};
	u64 timestamp;
} __attribute__((__packed__));

struct calibraion_data {
	s16 x;
	s16 y;
	s16 z;
};

struct hw_offset_data {
	char x;
	char y;
	char z;
};

struct ssp_msg {
	u8 cmd;
	u16 length;
	u16 options;
	u32 data;

	struct list_head list;
	struct completion *done;
	char *buffer;
	u8 free_buffer;
	bool *dead_hook;
	bool dead;
} __attribute__((__packed__));

enum {
	BATCH_MODE_NONE = 0,
	BATCH_MODE_RUN,
};

struct sensor_info
{
	char name[SENSOR_NAME_MAX_LEN];
	bool enable;
	int report_mode;
	int get_data_len;
	int report_data_len;
};

struct ssp_data {
	struct sensor_info info[SENSOR_TYPE_MAX];
	struct sensor_value buf[SENSOR_TYPE_MAX];
	struct iio_dev *indio_devs[SENSOR_TYPE_MAX];
	struct iio_chan_spec indio_channels[SENSOR_TYPE_MAX];
	struct device *devices[SENSOR_TYPE_MAX];

	struct spi_device *spi;
	struct wake_lock ssp_wake_lock;
	struct timer_list debug_timer;
	struct workqueue_struct *debug_wq;
	struct work_struct work_debug;
	struct calibraion_data accelcal;
	struct calibraion_data gyrocal;
	struct device *mcu_device;
	struct device *irled_device;
	struct device *mobeam_device;
	struct delayed_work work_firmware;
	struct delayed_work work_refresh;
	struct miscdevice shtc1_device;
	struct miscdevice batch_io_device;

	struct hw_offset_data magoffset;

	bool is_ssp_shutdown;
	bool is_accel_alert;
	bool is_proxraw_enabled;
	bool is_geomag_raw_enabled;
	bool is_barcode_enabled;
	bool is_mcu_dump_mode;
	bool is_binary_crashed;
	bool is_probe_done;
	bool is_ongoing_dump;
	bool is_time_syncing;
	bool is_reset_from_kernel;
	bool is_reset_from_sysfs;
	bool is_reset_started;

	unsigned int uProxCanc;
	unsigned int uCrosstalk;
	unsigned int uProxCalResult;
	unsigned int uProxHiThresh;
	unsigned int uProxLoThresh;
	unsigned int uProxHiThresh_detect;
	unsigned int uProxLoThresh_detect;
	unsigned int uProxHiThresh_default;
	unsigned int uProxLoThresh_default;
	unsigned int uIr_Current;
	unsigned char uFuseRomData[3];
	unsigned char uMagCntlRegData;

	char *pchLibraryBuf;
	char chLcdLdi[2];
	int irq;
	int iLibraryLength;
	int aiCheckStatus[SENSOR_TYPE_MAX];

	unsigned int cnt_com_fail;
	unsigned int cnt_reset;
	unsigned int cnt_timeout;
	unsigned int cnt_irq;
	unsigned int cnt_dump;

	uint64_t uSensorState;
	unsigned int curr_fw_rev;
	unsigned int uFactoryProxAvg[4];
	char uLastResumeState;
	char uLastAPState;

	atomic64_t aSensorEnable;
	int64_t delay[SENSOR_TYPE_MAX];
	s32 batch_max_latency[SENSOR_TYPE_MAX];
	s8 batch_opt[SENSOR_TYPE_MAX];
	u64 latest_timestamp[SENSOR_TYPE_MAX];
	bool is_data_reported[SENSOR_TYPE_MAX];

	struct ssp_sensorhub_data *hub_data;
	int accel_position;
	int accel_dot;
	int mag_position;
	int fw_dl_state;
	unsigned char pdc_matrix[PDC_SIZE];
	s16 *static_matrix;
	bool bGeomagneticRawEnabled;
	struct mutex comm_mutex;
	struct mutex pending_mutex;
	struct mutex enable_mutex;

	int mcu_int1;
	int mcu_int2;
	int ap_int;
	int rst;
	int boot0;

	struct list_head pending_list;

	void (*ssp_big_task[BIG_TYPE_MAX])(struct work_struct *);
	u64 timestamp;

	struct file *realtime_dump_file;
	int total_dump_size;
	int acc_type;
	int pressure_type;
	int mag_type;
	int gyro_lib_state;
	int first_gyro_cal;
	int project_select;
	int light_coef[7];
	int light_log_cnt;
	int prox_trim;
	
	bool debug_enable;
	char sensor_state[BIG_DATA_SENSOR_TYPE_MAX+1];
        struct  accelometer_sensor_operations *accel_ops;
        struct  gyroscope_sensor_operations *gyro_ops;
        struct  magnetic_sensor_operations *magnetic_ops;
        struct  proximity_sensor_operations *proximity_ops;
        struct  light_sensor_operations *light_ops;
        struct  barometer_sensor_operations *barometer_ops;
};

struct ssp_big {
	struct ssp_data *data;
	struct work_struct work;
	u32 length;
	u32 addr;
};

u64 get_current_timestamp(void);
#endif
