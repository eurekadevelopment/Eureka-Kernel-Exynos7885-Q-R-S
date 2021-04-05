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

#include "ssp_sysfs.h"

struct batch_config {
	int64_t timeout;
	int64_t delay;
	int flag;
};

/*************************************************************************/
/* SSP data delay function                                              */
/*************************************************************************/
int get_msdelay(int64_t delay)
{
	/*
	 * From Android 5.0, There is MaxDelay Concept.
	 * If App request lower frequency then MaxDelay,
	 * Sensor have to work with MaxDelay.
	 */
	if (delay > 200000000)
		delay = 200000000;
	return div_s64(delay, NSEC_PER_MSEC);
}

static void enable_sensor(struct ssp_data *data,
	int type, int64_t delay)
{
	u8 uBuf[9];
	u64 new_enable = 0;
	s32 batch_latency = 0;
	s8 batch_options = 0;
	int64_t temp_delay = data->delay[type];
	s32 ms_delay = get_msdelay(delay);
	int ret = 0;

	data->delay[type] = delay;
	batch_latency = data->batch_max_latency[type];
	batch_options = data->batch_opt[type];

	switch (data->aiCheckStatus[type]) {
	case ADD_SENSOR_STATE:
		/*ssp_infof("add %u, New = %lldns", 1 << type, delay);*/
		ssp_infof("ADD %s , type %d DELAY %lld ns", data->info[type].name, type, delay);

		if (type == SENSOR_TYPE_PROXIMITY) {
			set_proximity_threshold(data);
			ssp_infof("send threshold hi %d, low %d \n", data->uProxHiThresh, data->uProxLoThresh);
		}
		else if(type == SENSOR_TYPE_LIGHT) {
			data->light_log_cnt = 0;
		}

		memcpy(&uBuf[0], &ms_delay, 4);
		memcpy(&uBuf[4], &batch_latency, 4);
		uBuf[8] = batch_options;

		ret = send_instruction(data, ADD_SENSOR,
				type, uBuf, 9);
		ssp_infof("delay %d, timeout %d, flag=%d, ret%d",
			ms_delay, batch_latency, uBuf[8], ret);
		if (ret <= 0) {
			new_enable =
				(uint64_t)atomic64_read(&data->aSensorEnable)
				& (~(uint64_t)(1 << type));
			atomic64_set(&data->aSensorEnable, new_enable);

			data->aiCheckStatus[type] = NO_SENSOR_STATE;
			break;
		}

		data->aiCheckStatus[type] = RUNNING_SENSOR_STATE;

		break;
	case RUNNING_SENSOR_STATE:
#if 0 /* Declaration problem. */
		change_sensor_delay(data, type, delay);
#else
		if (get_msdelay(temp_delay)
			== get_msdelay(data->delay[type]))
			break;

		ssp_infof("Change %u, New = %lldns",
			1 << type, delay);

		memcpy(&uBuf[0], &ms_delay, 4);
		memcpy(&uBuf[4], &batch_latency, 4);
		uBuf[8] = batch_options;
		send_instruction(data, CHANGE_DELAY, type, uBuf, 9);

#endif
		break;
	default:
		data->aiCheckStatus[type] = ADD_SENSOR_STATE;
	}
}

static void change_sensor_delay(struct ssp_data *data,
	int type, int64_t delay)
{
	u8 uBuf[9];
	s32 batch_latency = 0;
	s8 batch_options = 0;
	int64_t temp_delay = data->delay[type];
	s32 ms_delay = get_msdelay(delay);
	u64 timestamp = 0;

	data->delay[type] = delay;
	data->batch_max_latency[type] = batch_latency;
	data->batch_opt[type] = batch_options;

	switch (data->aiCheckStatus[type]) {
	case RUNNING_SENSOR_STATE:
		if (get_msdelay(temp_delay)
			== get_msdelay(data->delay[type]))
			break;

		ssp_infof("Change %u, New = %lldns",
			1 << type, delay);

		memcpy(&uBuf[0], &ms_delay, 4);
		memcpy(&uBuf[4], &batch_latency, 4);
		uBuf[8] = batch_options;
		send_instruction(data, CHANGE_DELAY, type, uBuf, 9);

		timestamp = get_current_timestamp();
		//pr_info("[SSP] compare c %lld l %lld\n", timestamp, data->latest_timestamp[sensor]);

		if(data->latest_timestamp[type] < timestamp)
			data->latest_timestamp[type] = timestamp;
		break;

	default:
		break;
	}
}

/*************************************************************************/
/* SSP data enable function                                              */
/*************************************************************************/

static int ssp_remove_sensor(struct ssp_data *data,
	unsigned int type, uint64_t new_enable)
{
	u8 uBuf[4];
	int64_t delay = data->delay[type];

	ssp_infof("remove sensor = %d, current state = %lld",
		(1 << type), new_enable);

	data->delay[type] = DEFUALT_POLLING_DELAY;
	data->batch_max_latency[type] = 0;
	data->batch_opt[type] = 0;

	if (type == SENSOR_TYPE_ORIENTATION) {
		if (!(atomic64_read(&data->aSensorEnable)
			& (1 << SENSOR_TYPE_ACCELEROMETER))) {
			type = SENSOR_TYPE_ACCELEROMETER;
		} else {
			change_sensor_delay(data, SENSOR_TYPE_ACCELEROMETER,
				data->delay[SENSOR_TYPE_ACCELEROMETER]);
			return 0;
		}
	} else if (type == SENSOR_TYPE_ACCELEROMETER) {
		if (atomic64_read(&data->aSensorEnable)
			& (1 << SENSOR_TYPE_ORIENTATION)) {
			change_sensor_delay(data, SENSOR_TYPE_ORIENTATION,
				data->delay[SENSOR_TYPE_ORIENTATION]);
			return 0;
		}
	}

	if (!data->is_ssp_shutdown)
		if (atomic64_read(&data->aSensorEnable) & (1 << type)) {
			s32 ms_delay = get_msdelay(delay);
			memcpy(&uBuf[0], &ms_delay, 4);

			send_instruction(data, REMOVE_SENSOR,
					type, uBuf, 4);
		}
	data->aiCheckStatus[type] = NO_SENSOR_STATE;

	return 0;
}

/*************************************************************************/
/* ssp Sysfs                                                             */
/*************************************************************************/
static ssize_t show_enable_irq(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_infof("%d", !data->is_ssp_shutdown);

	return snprintf(buf, PAGE_SIZE,	"%d\n", !data->is_ssp_shutdown);
}

static ssize_t set_enable_irq(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	u8 dTemp;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtou8(buf, 10, &dTemp) < 0)
		return -EINVAL;

	ssp_infof("%d start", dTemp);
	if (dTemp) {
		data->is_reset_from_sysfs = true;
		reset_mcu(data);
		enable_debug_timer(data);
	} else if (!dTemp) {
		disable_debug_timer(data);
		ssp_enable(data, 0);
	} else
		ssp_errf("invalid value");
	ssp_infof("%d end", dTemp);
	return size;
}

static ssize_t show_sensors_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE,
		"%llu\n", (uint64_t)atomic64_read(&data->aSensorEnable));
}

static ssize_t set_sensors_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dTemp;
	int ret;
	uint64_t new_enable = 0, type = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->enable_mutex);
	if (kstrtoll(buf, 10, &dTemp) < 0) {
		mutex_unlock(&data->enable_mutex);
		return -EINVAL;
	}

	new_enable = (uint64_t)dTemp;
	ssp_infof("new_enable = %llu, old_enable = %llu",
		 new_enable, (uint64_t)atomic64_read(&data->aSensorEnable));

	if ((new_enable != atomic64_read(&data->aSensorEnable)) &&
		!(data->uSensorState
			& (new_enable - atomic64_read(&data->aSensorEnable)))) {
		ssp_infof("%llu is not connected(sensor state: 0x%llx)",
			new_enable - atomic64_read(&data->aSensorEnable),
			data->uSensorState);
		mutex_unlock(&data->enable_mutex);
		return -EINVAL;
	}

	if (new_enable == atomic64_read(&data->aSensorEnable)) {
		mutex_unlock(&data->enable_mutex);
		return size;
	}

	for (type = 0; type < SENSOR_TYPE_MAX; type++) {
		if ((atomic64_read(&data->aSensorEnable) & (1 << type))
			!= (new_enable & (1 << type))) {

			if (!(new_enable & (1 << type))) {
				data->is_data_reported[type] = false;
				ssp_remove_sensor(data, type,
					new_enable); /* disable */
			} else { /* Change to ADD_SENSOR_STATE from KitKat */
				if (data->aiCheckStatus[type]
						== INITIALIZATION_STATE) {
					if (type == SENSOR_TYPE_ACCELEROMETER) {
						accel_open_calibration(data);
						ret = set_accel_cal(data);
						if (ret < 0)
							ssp_errf("set_accel_cal failed %d", ret);
					} else if (type == SENSOR_TYPE_PRESSURE) {
						pressure_open_calibration(data);
					} else if (type == SENSOR_TYPE_PROXIMITY) {
						set_proximity_threshold(data);
					}
				}
				data->aiCheckStatus[type] = ADD_SENSOR_STATE;

				enable_sensor(data, type, data->delay[type]);
			}
			break;
		}
	}
	atomic64_set(&data->aSensorEnable, new_enable);
	mutex_unlock(&data->enable_mutex);

	return size;
}

static ssize_t set_flush(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dTemp;
	u8 sensor_type = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dTemp) < 0)
		return -EINVAL;

	sensor_type = (u8)dTemp;
	if (!(atomic64_read(&data->aSensorEnable) & (1 << sensor_type)))
		return -EINVAL;

	if (flush(data, sensor_type) < 0) {
		ssp_err("ssp returns error for flush(%x)", sensor_type);
		return -EINVAL;
	}
	return size;
}

static ssize_t show_sensor_delay(struct device *dev, char *buf, int type)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%lld\n", data->delay[type]);
}

static ssize_t show_acc_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_ACCELEROMETER);
}

static ssize_t set_acc_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	if ((atomic64_read(&data->aSensorEnable) & (1 << SENSOR_TYPE_ORIENTATION)) &&
		(data->delay[SENSOR_TYPE_ORIENTATION] < delay))
		data->delay[SENSOR_TYPE_ACCELEROMETER] = delay;
	else
		change_sensor_delay(data, SENSOR_TYPE_ACCELEROMETER, delay);

	return size;
}

static ssize_t show_gyro_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_GYROSCOPE);
}

static ssize_t set_gyro_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_GYROSCOPE, delay);
	return size;
}

static ssize_t show_mag_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_GEOMAGNETIC_FIELD);
}

static ssize_t set_mag_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_GEOMAGNETIC_FIELD, delay);

	return size;
}

static ssize_t show_uncal_mag_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED);
}

static ssize_t set_uncal_mag_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED, delay);

	return size;
}

static ssize_t show_rot_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_ROTATION_VECTOR);
}

static ssize_t set_rot_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_ROTATION_VECTOR, delay);

	return size;
}

static ssize_t show_game_rot_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_GAME_ROTATION_VECTOR);
}

static ssize_t set_game_rot_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_GAME_ROTATION_VECTOR, delay);

	return size;
}

static ssize_t show_step_det_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_STEP_DETECTOR);
}

static ssize_t set_step_det_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_STEP_DETECTOR, delay);
	return size;
}

static ssize_t show_sig_motion_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_SIGNIFICANT_MOTION);
}

static ssize_t set_sig_motion_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_SIGNIFICANT_MOTION, delay);
	return size;
}

static ssize_t show_step_cnt_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_STEP_COUNTER);
}

static ssize_t set_step_cnt_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_STEP_COUNTER, delay);
	return size;
}

static ssize_t show_uncalib_gyro_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
}

static ssize_t set_uncalib_gyro_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, delay);

	return size;
}

static ssize_t show_pressure_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_PRESSURE);
}

static ssize_t set_pressure_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_PRESSURE, delay);
	return size;
}

static ssize_t show_light_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_LIGHT);
}

static ssize_t set_light_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_LIGHT, delay);
	return size;
}

static ssize_t show_prox_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_PROXIMITY);
}

static ssize_t set_prox_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_PROXIMITY, delay);
	return size;
}

static ssize_t show_tilt_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_TILT_DETECTOR);
}

static ssize_t set_tilt_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_TILT_DETECTOR, delay);
	return size;
}

static ssize_t show_pickup_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return show_sensor_delay(dev, buf, SENSOR_TYPE_PICK_UP_GESTURE);
}

static ssize_t set_pickup_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t delay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &delay) < 0)
		return -EINVAL;

	change_sensor_delay(data, SENSOR_TYPE_PICK_UP_GESTURE, delay);
	return size;
}

static ssize_t show_debug_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", data->debug_enable);
}

static ssize_t set_debug_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	int64_t debug_enable;

	if (kstrtoll(buf, 10, &debug_enable) < 0)
		return -EINVAL;

	if (debug_enable != 1 && debug_enable != 0)
		return -EINVAL;

	data->debug_enable = (bool)debug_enable;
	return size;
}

static ssize_t show_sensor_axis(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d: %d\n%d: %d\n",
		SENSOR_TYPE_ACCELEROMETER, data->accel_position,
		SENSOR_TYPE_GEOMAGNETIC_FIELD, data->mag_position);
}

static ssize_t set_sensor_axis(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	int sensor = 0;
	int position = 0;
	int ret = 0;

	sscanf(buf, "%9d,%9d", &sensor, &position);

	if (position < 0 || position > 7)
		return -EINVAL;

	if (sensor == SENSOR_TYPE_ACCELEROMETER)
		data->accel_position = position;
	else if (sensor == SENSOR_TYPE_GEOMAGNETIC_FIELD)
		data->mag_position = position;
	else
		return -EINVAL;

	ret = set_sensor_position(data);
	if (ret < 0) {
		ssp_errf("set_sensor_position failed");
		return -EIO;
	}

	return size;
}

static ssize_t show_sensor_dot(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", data->accel_dot);
}

static ssize_t set_sensor_dot(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int64_t new_dot = 0;
	int old_dot = data->accel_dot;
	int ret = 0;

	if (kstrtoll(buf, 10, &new_dot) < 0)
		return -EINVAL;

	if (new_dot < 0 || new_dot > 7)
		return -EINVAL;

	data->accel_dot = (int)new_dot;

	ret = set_6axis_dot(data);
	if (ret < 0) {
		data->accel_dot = old_dot;
		ssp_errf("set_6axis_dot failed");
		return -EIO;
	}

	return size;
}

static ssize_t set_send_instruction(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	int instruction[4] = { 0, };
	int ret = 0;

	sscanf(buf, "%9d,%9d,%9d,%9d",
		&instruction[0], &instruction[1],
		&instruction[2], &instruction[3]);

	ret = send_instruction(data,
			(unsigned char)instruction[0],
			(unsigned char)instruction[1],
			(unsigned char *)&instruction[2], 2);

	if (ret < 0) {
		ssp_errf("send_instruction failed");
		return -EIO;
	}

	return size;
}

static ssize_t show_sensor_state(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", data->sensor_state);
}


static DEVICE_ATTR(mcu_rev, S_IRUGO, mcu_revision_show, NULL);
static DEVICE_ATTR(mcu_name, S_IRUGO, mcu_model_name_show, NULL);
static DEVICE_ATTR(mcu_update, S_IRUGO, mcu_update_kernel_bin_show, NULL);
static DEVICE_ATTR(mcu_update2, S_IRUGO,
	mcu_update_kernel_crashed_bin_show, NULL);
static DEVICE_ATTR(mcu_update_ums, S_IRUGO, mcu_update_ums_bin_show, NULL);
static DEVICE_ATTR(mcu_reset, S_IRUGO, mcu_reset_show, NULL);
static DEVICE_ATTR(mcu_dump, S_IRUGO, mcu_dump_show, NULL);

static DEVICE_ATTR(mcu_test, S_IRUGO | S_IWUSR | S_IWGRP,
	mcu_factorytest_show, mcu_factorytest_store);
static DEVICE_ATTR(mcu_sleep_test, S_IRUGO | S_IWUSR | S_IWGRP,
	mcu_sleep_factorytest_show, mcu_sleep_factorytest_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	show_sensors_enable, set_sensors_enable);
static DEVICE_ATTR(enable_irq, S_IRUGO | S_IWUSR | S_IWGRP,
	show_enable_irq, set_enable_irq);

static DEVICE_ATTR(accel_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_acc_delay, set_acc_delay);
static DEVICE_ATTR(gyro_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_gyro_delay, set_gyro_delay);
static DEVICE_ATTR(uncal_mag_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_uncal_mag_delay, set_uncal_mag_delay);
static DEVICE_ATTR(mag_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_mag_delay, set_mag_delay);
static DEVICE_ATTR(pressure_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_pressure_delay, set_pressure_delay);
static DEVICE_ATTR(prox_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_prox_delay, set_prox_delay);
static DEVICE_ATTR(light_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_light_delay, set_light_delay);
static DEVICE_ATTR(step_det_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_step_det_delay, set_step_det_delay);
static DEVICE_ATTR(sig_motion_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_sig_motion_delay, set_sig_motion_delay);
static DEVICE_ATTR(uncal_gyro_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_uncalib_gyro_delay, set_uncalib_gyro_delay);
static DEVICE_ATTR(game_rot_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_game_rot_delay, set_game_rot_delay);
static DEVICE_ATTR(rot_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_rot_delay, set_rot_delay);
static DEVICE_ATTR(step_cnt_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_step_cnt_delay, set_step_cnt_delay);
static DEVICE_ATTR(tilt_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_tilt_delay, set_tilt_delay);
static DEVICE_ATTR(pickup_poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_pickup_delay, set_pickup_delay);
static DEVICE_ATTR(ssp_flush, S_IWUSR | S_IWGRP,
	NULL, set_flush);
static DEVICE_ATTR(debug_enable, S_IRUGO | S_IWUSR | S_IWGRP,
	show_debug_enable, set_debug_enable);
static DEVICE_ATTR(sensor_axis, S_IRUGO | S_IWUSR | S_IWGRP,
	show_sensor_axis, set_sensor_axis);
static DEVICE_ATTR(sensor_dot, S_IRUGO | S_IWUSR | S_IWGRP,
	show_sensor_dot, set_sensor_dot);
static DEVICE_ATTR(send_instruction, S_IWUSR | S_IWGRP,
	NULL, set_send_instruction);
static DEVICE_ATTR(sensor_state, S_IRUGO, show_sensor_state, NULL);

static struct device_attribute *mcu_attrs[] = {
	&dev_attr_enable,
	&dev_attr_mcu_rev,
	&dev_attr_mcu_name,
	&dev_attr_mcu_test,
	&dev_attr_mcu_reset,
	&dev_attr_mcu_dump,
	&dev_attr_mcu_update,
	&dev_attr_mcu_update2,
	&dev_attr_mcu_update_ums,
	&dev_attr_mcu_sleep_test,
	&dev_attr_enable_irq,
	&dev_attr_accel_poll_delay,
	&dev_attr_gyro_poll_delay,
	&dev_attr_uncal_mag_poll_delay,
	&dev_attr_mag_poll_delay,
	&dev_attr_pressure_poll_delay,
	&dev_attr_prox_poll_delay,
	&dev_attr_light_poll_delay,
	&dev_attr_step_det_poll_delay,
	&dev_attr_sig_motion_poll_delay,
	&dev_attr_uncal_gyro_poll_delay,
	&dev_attr_game_rot_poll_delay,
	&dev_attr_rot_poll_delay,
	&dev_attr_step_cnt_poll_delay,
	&dev_attr_tilt_poll_delay,
	&dev_attr_pickup_poll_delay,
	&dev_attr_ssp_flush,
	&dev_attr_debug_enable,
	&dev_attr_sensor_axis,
	&dev_attr_sensor_dot,
	&dev_attr_send_instruction,
	&dev_attr_sensor_state,
	NULL,
};

static long ssp_batch_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct ssp_data *data
		= container_of(file->private_data,
			struct ssp_data, batch_io_device);

	struct batch_config batch;

	void __user *argp = (void __user *)arg;
	int retries = 2;
	int ret = 0;
	int sensor_type, ms_delay;
	int timeout_ms = 0;
	u8 uBuf[9];

	sensor_type = (cmd & 0xFF);

	ssp_info("cmd = %d, sensor_type = %d", cmd, sensor_type);

	if ((cmd >> 8 & 0xFF) != BATCH_IOCTL_MAGIC) {
		ssp_err("Invalid BATCH CMD %x", cmd);
		return -EINVAL;
	}

	if(sensor_type >= SENSOR_TYPE_MAX)
	{
		ssp_err("Invalid sensor_type %d", sensor_type);
		return -EINVAL;
	}

	while (retries--) {
		ret = copy_from_user(&batch, argp, sizeof(batch));
		if (likely(!ret))
			break;
	}
	if (unlikely(ret)) {
		ssp_err("batch ioctl err(%d)", ret);
		return -EINVAL;
	}

	ms_delay = get_msdelay(batch.delay);
	timeout_ms = div_s64(batch.timeout, NSEC_PER_MSEC);
	memcpy(&uBuf[0], &ms_delay, 4);
	memcpy(&uBuf[4], &timeout_ms, 4);
	uBuf[8] = batch.flag;

	if (batch.timeout) { /* add or dry */

		/* real batch, NOT DRY, change delay */
		if (!(batch.flag & SENSORS_BATCH_DRY_RUN)) {
			ret = 1;
			/* if sensor is not running state, enable will be called.
			   MCU return fail when receive chage delay inst during NO_SENSOR STATE */
			if (data->aiCheckStatus[sensor_type]
					== RUNNING_SENSOR_STATE)
				ret = send_instruction_sync(data, CHANGE_DELAY,
						sensor_type, uBuf, 9);

			if (ret > 0) { /* ret 1 is success */
				data->batch_opt[sensor_type] = (u8)batch.flag;
				data->batch_max_latency[sensor_type] = timeout_ms;
				data->delay[sensor_type] = batch.delay;
			}
		} else { /* real batch, DRY RUN */
			ret = send_instruction_sync(data, CHANGE_DELAY,
						sensor_type, uBuf, 9);
			if (ret > 0) { /* ret 1 is success */
				data->batch_opt[sensor_type] = (u8)batch.flag;
				data->batch_max_latency[sensor_type] = timeout_ms;
				data->delay[sensor_type] = batch.delay;
			}
		}
	} else { /* remove batch or normal change delay, remove or add will be called */

		/* no batch, NOT DRY, change delay */
		if (!(batch.flag & SENSORS_BATCH_DRY_RUN)) {
			data->batch_opt[sensor_type] = 0;
			data->batch_max_latency[sensor_type] = 0;
			data->delay[sensor_type] = batch.delay;
			if (data->aiCheckStatus[sensor_type]
					== RUNNING_SENSOR_STATE) {
				send_instruction(data, CHANGE_DELAY,
						sensor_type, uBuf, 9);
			}
		}
	}

	ssp_info("batch %d: delay %lld, timeout %lld, flag %d, ret %d",
		sensor_type, batch.delay, batch.timeout, batch.flag, ret);
	if (!batch.timeout)
		return 0;
	if (ret <= 0)
		return -EINVAL;
	else
		return 0;
}


static struct file_operations ssp_batch_fops = {
	.owner = THIS_MODULE,
	.open = nonseekable_open,
	.unlocked_ioctl = ssp_batch_ioctl,
};

static void initialize_mcu_factorytest(struct ssp_data *data)
{
	sensors_register(data->mcu_device, data, mcu_attrs, "ssp_sensor");
}

static void remove_mcu_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->mcu_device, mcu_attrs);
}

int initialize_sysfs(struct ssp_data *data)
{
	data->batch_io_device.minor = MISC_DYNAMIC_MINOR;
	data->batch_io_device.name = "batch_io";
	data->batch_io_device.fops = &ssp_batch_fops;
	if (misc_register(&data->batch_io_device))
		goto err_batch_io_dev;

	initialize_accel_factorytest(data);
	initialize_gyro_factorytest(data);
	initialize_prox_factorytest(data);
	initialize_light_factorytest(data);
	initialize_barometer_factorytest(data);
	initialize_magnetic_factorytest(data);
	initialize_mcu_factorytest(data);
#ifdef CONFIG_SENSORS_SSP_TMD4903
	initialize_irled_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_MOBEAM
	initialize_mobeam(data);
#endif

	return SUCCESS;
err_batch_io_dev:
	ssp_err("error init sysfs");
	return ERROR;
}

void remove_sysfs(struct ssp_data *data)
{
	ssp_batch_fops.unlocked_ioctl = NULL;
	misc_deregister(&data->batch_io_device);
	remove_accel_factorytest(data);
	remove_gyro_factorytest(data);
	remove_prox_factorytest(data);
	remove_light_factorytest(data);
	remove_barometer_factorytest(data);
	remove_magnetic_factorytest(data);
	remove_mcu_factorytest(data);
#ifdef CONFIG_SENSORS_SSP_TMD4903
	remove_irled_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_MOBEAM
	remove_mobeam(data);
#endif

	destroy_sensor_class();
}
