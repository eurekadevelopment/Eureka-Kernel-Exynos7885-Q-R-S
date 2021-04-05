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
#include "ssp_cmd_define.h"
#include "ssp_sensorlist.h"
#include "ssp_data.h"
#include "ssp_debug.h"
#include "ssp_dev.h"
#include "ssp_firmware.h"
#include "ssp_iio.h"
#include "ssp_sensorhub.h"
#include "ssp_comm.h"
#include "ssp_sysfs.h"
#include "sensors_core.h"

#define DEFUALT_POLLING_DELAY   (200 * NSEC_PER_MSEC)
#define DATA_PACKET_SIZE        2000

#define SUCCESS 0
#define FAIL    -2
#define ERROR   -1

#define BIG_DATA_SENSOR_TYPE_MAX        31

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

enum {
	RESET_INIT_VALUE = 0,
	RESET_KERNEL_NO_EVENT = 1,
	RESET_KERNEL_TIME_OUT,
	RESET_KERNEL_SYSFS,
	RESET_MCU_CRASHED,
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
	u8 type;
	u8 subcmd;
	u16 length;
	char *buffer;
	u8 res;         /* success : 1 fail : 0 */
	bool clean_pending_list_flag;
	struct completion *done;
	struct list_head list;
} __attribute__((__packed__));

enum {
	BATCH_MODE_NONE = 0,
	BATCH_MODE_RUN,
};

struct sensor_info {
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
	struct device *mobeam_device;
	struct delayed_work work_firmware;
	struct delayed_work work_refresh;
	struct miscdevice batch_io_device;
#ifdef SENSOR_TYPE_SCONTEXT
	struct miscdevice scontext_device;
#endif

	bool is_ssp_shutdown;
	bool is_accel_alert;
	bool is_proxraw_enabled;
	bool is_geomag_raw_enabled;
	bool is_barcode_enabled;
	bool is_probe_done;
	bool is_time_syncing;
	bool is_reset_from_kernel;
	bool is_reset_from_sysfs;
	bool is_reset_started;
	int reset_type;

	char *pchLibraryBuf;
	char chLcdLdi[2];
	int irq;
	int iLibraryLength;
	int aiCheckStatus[SENSOR_TYPE_MAX];

	unsigned int cnt_com_fail;
	unsigned int cnt_reset;
	unsigned int cnt_timeout;
	unsigned int cnt_irq;
	unsigned int cnt_no_event_reset;

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
	int mag_position;
	int fw_dl_state;
	struct mutex comm_mutex;
	struct mutex pending_mutex;
	struct mutex enable_mutex;
	struct mutex cmd_mutex;

	int mcu_int1;
	int mcu_int2;
	int ap_int;
	int rst;
	int boot0;

	struct list_head pending_list;

	u64 timestamp;

	struct file *realtime_dump_file;
	int total_dump_size;
	unsigned int cnt_dump;
	bool is_ongoing_dump;
	bool debug_enable;
	char sensor_state[BIG_DATA_SENSOR_TYPE_MAX + 1];
	char *sensor_dump[SENSOR_TYPE_MAX];
	char *callstack_data;
#ifdef CONFIG_SSP_ENG_DEBUG
	char register_value[5];
#endif


#ifdef CONFIG_SENSORS_SSP_ACCELOMETER
	struct  accelometer_sensor_operations *accel_ops;
	int acc_type;
#endif
#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
	struct  gyroscope_sensor_operations *gyro_ops;
	int first_gyro_cal;
#endif
#ifdef CONFIG_SENSORS_SSP_MAGNETIC
	struct  magnetic_sensor_operations *magnetic_ops;
	unsigned char pdc_matrix[PDC_SIZE];
	s16 *static_matrix;
	bool bGeomagneticRawEnabled;
	int mag_type;
	unsigned char uFuseRomData[3];
	unsigned char uMagCntlRegData;
	struct hw_offset_data magoffset;
#endif
#ifdef CONFIG_SENSORS_SSP_PROXIMITY
	struct  proximity_sensor_operations *proximity_ops;
	unsigned int uProxCanc;
	unsigned int uCrosstalk;
	unsigned int uProxCalResult;
	unsigned int uProxHiThresh;
	unsigned int uProxLoThresh;
	unsigned int uProxHiThresh_detect;
	unsigned int uProxLoThresh_detect;
	unsigned int uProxHiThresh_default;
	unsigned int uProxLoThresh_default;
#if defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
	int prox_trim;
#endif
#endif
#ifdef CONFIG_SENSORS_SSP_LIGHT
	struct  light_sensor_operations *light_ops;
	int light_coef[7];
	int light_log_cnt;
	int project_select;
#endif
#ifdef CONFIG_SENSORS_SSP_BAROMETER
	struct  barometer_sensor_operations *barometer_ops;
	int pressure_type;

#endif
};

u64 get_current_timestamp(void);
#endif
