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

#include "ssp_debug.h"

static mm_segment_t backup_fs;

int debug_crash_dump(struct ssp_data *data, char *pchRcvDataFrame, int iLength)
{
	unsigned char datacount = pchRcvDataFrame[1];
	unsigned int databodysize = iLength - 2;
	char *databody = &pchRcvDataFrame[2];
	int ret_write = 0;

	if (data->is_ssp_shutdown) {
		ssp_infof("ssp shutdown, stop dumping");
		return FAIL;
	}

	wake_lock(&data->ssp_wake_lock);

	if (data->realtime_dump_file == NULL) {
		struct file *dump_time_file;
		char time_file_body[50] = {0,};
		mm_segment_t old_fs;
		struct timespec ts;
		struct rtc_time tm;

		old_fs = get_fs();
		set_fs(get_ds());

		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);

		snprintf(time_file_body, sizeof(time_file_body), "%d-%02d-%02d %02d:%02d:%02d UTC",
		         tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

		dump_time_file = filp_open(DEBUG_DUMP_TIME_FILE_PATH,
		                           O_RDWR | O_CREAT | O_TRUNC, 0664);

		if (IS_ERR(dump_time_file)) {
			ssp_errf("Can't open dump time file(%d)", (int)PTR_ERR(dump_time_file));
			set_fs(old_fs);
			wake_unlock(&data->ssp_wake_lock);
			return FAIL;
		}

		ret_write = vfs_write(dump_time_file,
		                      (char __user *)time_file_body, sizeof(time_file_body),
		                      &dump_time_file->f_pos);

		filp_close(dump_time_file, current->files);
		set_fs(old_fs);

		if (ret_write < 0) {
			ssp_errf("Can't write dump time to file (%d)", ret_write);
			wake_unlock(&data->ssp_wake_lock);
			return FAIL;
		}

		backup_fs = get_fs();
		set_fs(get_ds());

		data->realtime_dump_file = filp_open(DEBUG_DUMP_FILE_PATH,
		                                     O_RDWR | O_CREAT | O_TRUNC, 0664);

		if (IS_ERR(data->realtime_dump_file)) {
			ssp_errf("Can't open dump file(%d)", (int)PTR_ERR(data->realtime_dump_file));
			set_fs(backup_fs);
			wake_unlock(&data->ssp_wake_lock);
			return FAIL;
		}

		data->is_ongoing_dump = true;
		ssp_infof("save_crash_dump : open file(%s, %s)", DEBUG_DUMP_FILE_PATH, time_file_body);

	}

	if (data->is_ongoing_dump == true) {
		data->total_dump_size += databodysize;
		ret_write = vfs_write(data->realtime_dump_file,
		                      (char __user *)databody, databodysize,
		                      &data->realtime_dump_file->f_pos);


		if (ret_write < 0) {
			ssp_errf("Can't write dump to file(size = %d)", data->total_dump_size);
			filp_close(data->realtime_dump_file, current->files);
			set_fs(backup_fs);
			data->total_dump_size = 0;
			data->is_ongoing_dump = false;

			wake_unlock(&data->ssp_wake_lock);
			return FAIL;
		} else {
			ssp_infof("length(%d)", databodysize);
		}
	}

	if (datacount == DEBUG_DUMP_DATA_COMPLETE) {
		if (data->is_ongoing_dump == true) {
			ssp_infof("close file(size=%d)", data->total_dump_size);

			filp_close(data->realtime_dump_file, current->files);
			set_fs(backup_fs);

			data->cnt_dump++;
			data->total_dump_size = 0;
			data->is_ongoing_dump = false;
		}

		data->realtime_dump_file = NULL;
	}

	wake_unlock(&data->ssp_wake_lock);

	return SUCCESS;
}

/*************************************************************************/
/* SSP Debug timer function                                              */
/*************************************************************************/
int print_mcu_debug(char *pchRcvDataFrame, int *pDataIdx,
                    int iRcvDataFrameLength)
{
	u16 length = 0;
	int cur = *pDataIdx;

	memcpy(&length, pchRcvDataFrame + *pDataIdx, 1);
	*pDataIdx += 1;

	if (length > iRcvDataFrameLength - *pDataIdx || length <= 0) {
		ssp_infof("[M] invalid debug length(%u/%d/%d)",
		          length, iRcvDataFrameLength, cur);
		return length ? length : ERROR;
	}

	ssp_info("[M] %s", &pchRcvDataFrame[*pDataIdx]);
	*pDataIdx += length;
	return 0;
}

void reset_mcu(struct ssp_data *data)
{
	ssp_infof();
	ssp_enable(data, false);
	clean_pending_list(data);
	toggle_mcu_reset(data);
	ssp_enable(data, true);
}

void sync_sensor_state(struct ssp_data *data)
{
	u8 buf[8] = {0,};
	u32 uSensorCnt;
	int ret = 0;

#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
	gyro_open_calibration(data);
	ret = set_gyro_cal(data);
	if (ret < 0) {
		ssp_errf("set_gyro_cal failed\n");
	}
#endif
#ifdef CONFIG_SENSORS_SSP_ACCELOMETER
	ret = set_accel_cal(data);
	if (ret < 0) {
		ssp_errf("set_accel_cal failed\n");
	}
#endif
	udelay(10);

	for (uSensorCnt = 0; uSensorCnt < SENSOR_TYPE_MAX; uSensorCnt++) {
		mutex_lock(&data->enable_mutex);
		if (atomic64_read(&data->aSensorEnable) & (1ULL << uSensorCnt)) {
			s32 dMsDelay
			        = get_msdelay(data->delay[uSensorCnt]);
			memcpy(&buf[0], &dMsDelay, 4);
			memcpy(&buf[4], &data->batch_max_latency[uSensorCnt], 4);
			make_command(data, ADD_SENSOR, uSensorCnt, buf, 8);
			udelay(10);
		}
		mutex_unlock(&data->enable_mutex);
	}

	if (data->is_proxraw_enabled == true) {
		s32 dMsDelay = 20;

		memcpy(&buf[0], &dMsDelay, 4);
		memcpy(&buf[4], &data->batch_max_latency[SENSOR_TYPE_PROXIMITY_RAW], 4);
		make_command(data, ADD_SENSOR, SENSOR_TYPE_PROXIMITY_RAW, buf, 8);
	}

#ifdef CONFIG_SENSORS_SSP_LIGHT
	set_light_coef(data);
#endif 
#ifdef CONFIG_SENSORS_SSP_PROXIMITY
	set_proximity_threshold(data);
#endif
}

static void print_sensordata(struct ssp_data *data, unsigned int uSensor)
{
	switch (uSensor) {
	case SENSOR_TYPE_ACCELEROMETER:
	case SENSOR_TYPE_GYROSCOPE:
	case SENSOR_TYPE_INTERRUPT_GYRO:
		ssp_info("%u : %d, %d, %d (%ums, %dms)", uSensor,
		         data->buf[uSensor].x, data->buf[uSensor].y,
		         data->buf[uSensor].z,
		         get_msdelay(data->delay[uSensor]),
		         data->batch_max_latency[uSensor]);
		break;
	case SENSOR_TYPE_GEOMAGNETIC_FIELD:
		ssp_info("%u : %d, %d, %d, %d (%ums)", uSensor,
		         data->buf[uSensor].cal_x, data->buf[uSensor].cal_y,
		         data->buf[uSensor].cal_z, data->buf[uSensor].accuracy,
		         get_msdelay(data->delay[uSensor]));
		break;
	case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
		ssp_info("%u : %d, %d, %d, %d, %d, %d (%ums)", uSensor,
		         data->buf[uSensor].uncal_x,
		         data->buf[uSensor].uncal_y,
		         data->buf[uSensor].uncal_z,
		         data->buf[uSensor].offset_x,
		         data->buf[uSensor].offset_y,
		         data->buf[uSensor].offset_z,
		         get_msdelay(data->delay[uSensor]));
		break;
	case SENSOR_TYPE_PRESSURE:
		ssp_info("%u : %d, %d (%ums, %dms)", uSensor,
		         data->buf[uSensor].pressure,
		         data->buf[uSensor].temperature,
		         get_msdelay(data->delay[uSensor]),
		         data->batch_max_latency[uSensor]);
		break;
	case SENSOR_TYPE_LIGHT:
		ssp_info("%u : %u, %u, %u, %u, %u, %u (%ums)", uSensor,
		         data->buf[uSensor].r, data->buf[uSensor].g,
		         data->buf[uSensor].b, data->buf[uSensor].w,
		         data->buf[uSensor].a_time, data->buf[uSensor].a_gain,
		         get_msdelay(data->delay[uSensor]));
		break;
	case SENSOR_TYPE_PROXIMITY:
		ssp_info("%u : %d, %d (%ums)", uSensor,
		         data->buf[uSensor].prox, data->buf[uSensor].prox_ex,
		         get_msdelay(data->delay[uSensor]));
		break;
	case SENSOR_TYPE_STEP_DETECTOR:
		ssp_info("%u : %u (%ums, %dms)", uSensor,
		         data->buf[uSensor].step_det,
		         get_msdelay(data->delay[uSensor]),
		         data->batch_max_latency[uSensor]);
		break;
	case SENSOR_TYPE_GAME_ROTATION_VECTOR:
	case SENSOR_TYPE_ROTATION_VECTOR:
		ssp_info("%u : %d, %d, %d, %d, %d (%ums, %dms)", uSensor,
		         data->buf[uSensor].quat_a, data->buf[uSensor].quat_b,
		         data->buf[uSensor].quat_c, data->buf[uSensor].quat_d,
		         data->buf[uSensor].acc_rot,
		         get_msdelay(data->delay[uSensor]),
		         data->batch_max_latency[uSensor]);
		break;
	case SENSOR_TYPE_SIGNIFICANT_MOTION:
		ssp_info("%u : %u(%ums)", uSensor,
		         data->buf[uSensor].sig_motion,
		         get_msdelay(data->delay[uSensor]));
		break;
	case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		ssp_info("%u : %d, %d, %d, %d, %d, %d (%ums)", uSensor,
		         data->buf[uSensor].uncal_x, data->buf[uSensor].uncal_y,
		         data->buf[uSensor].uncal_z, data->buf[uSensor].offset_x,
		         data->buf[uSensor].offset_y,
		         data->buf[uSensor].offset_z,
		         get_msdelay(data->delay[uSensor]));
		break;
	case SENSOR_TYPE_STEP_COUNTER:
		ssp_info("%u : %u(%ums)", uSensor,
		         data->buf[uSensor].step_diff,
		         get_msdelay(data->delay[uSensor]));
		break;
	default:
		ssp_info("Wrong sensorCnt: %u", uSensor);
		break;
	}
}

static void recovery_mcu(struct ssp_data *data)
{
	if (data->cnt_com_fail < LIMIT_RESET_CNT) {
		ssp_infof("- cnt_timeout(%u), pending(%u)",
		          data->cnt_timeout, !list_empty(&data->pending_list));
		data->cnt_com_fail++;
		data->is_reset_from_kernel = true;
		reset_mcu(data);
	} else {
		ssp_enable(data, false);
	}

	data->cnt_timeout = 0;
}

/*
check_sensor_event
 - return true : there is no accel or light sensor event
                over 5sec when sensor is registered
*/
bool check_no_event(struct ssp_data *data)
{
	u64 timestamp = get_current_timestamp();
	int check_sensors[] = {SENSOR_TYPE_ACCELEROMETER, SENSOR_TYPE_LIGHT};
	int len = sizeof(check_sensors) / sizeof(check_sensors[0]);
	int i, sensor;
	bool res = false;

	for (i = 0 ; i < len ; i++) {
		sensor = check_sensors[i];
		/* The sensor is registered
		   And none batching mode
		   And there is no sensor event over 5sec */
		if ((atomic64_read(&data->aSensorEnable) & (1ULL << sensor))
		    && data->batch_max_latency[sensor] == 0
		    && data->latest_timestamp[sensor] + 5000000000ULL < timestamp) {

			ssp_infof("sensor(%d) last = %lld, cur = %lld", sensor, data->latest_timestamp[sensor], timestamp);
			res = true;
		}
	}

	if (res == true) {
		data->cnt_no_event_reset++;
	}

	return res;
}

static void debug_work_func(struct work_struct *work)
{
	unsigned int type;
	struct ssp_data *data = container_of(work, struct ssp_data, work_debug);

	ssp_infof("(%u) - Sensor state: 0x%llx, Reset cnt: %u, Comm fail: %u, Dump cnt : %u, Time out: %u No event : %u",
	          data->cnt_irq, data->uSensorState, data->cnt_reset, data->cnt_com_fail, data->cnt_dump,
	          data->cnt_timeout, data->cnt_no_event_reset);

	switch (data->fw_dl_state) {
	case FW_DL_STATE_FAIL:
	case FW_DL_STATE_DOWNLOADING:
	case FW_DL_STATE_SYNC:
		ssp_infof("firmware downloading state = %d", data->fw_dl_state);
		return;
	}

	for (type = 0; type < SENSOR_TYPE_MAX; type++)
		if ((atomic64_read(&data->aSensorEnable) & (1ULL << type))
		    || data->batch_max_latency[type]) {
			print_sensordata(data, type);
		}

	if (data->cnt_timeout > LIMIT_TIMEOUT_CNT){
		data->reset_type = RESET_KERNEL_TIME_OUT;
		recovery_mcu(data);
	} else if(check_no_event(data)) {
		ssp_dbgf("no event, no sensorhub reset");
		/*
		data->reset_type = RESET_KERNEL_NO_EVENT;
		recovery_mcu(data);
		*/
	}

	data->cnt_irq = 0;

#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
	if (data->first_gyro_cal == true) {
		int ret = 0;
		gyro_open_calibration(data);
		ret = set_gyro_cal(data);
		if (ret < 0) {
			ssp_errf("set_gyro_cal failed\n");
		}
		data->first_gyro_cal = false;
	}
#endif
}

static void debug_timer_func(unsigned long ptr)
{
	struct ssp_data *data = (struct ssp_data *)ptr;

	queue_work(data->debug_wq, &data->work_debug);
	mod_timer(&data->debug_timer,
	          round_jiffies_up(jiffies + SSP_DEBUG_TIMER_SEC));
}

void enable_debug_timer(struct ssp_data *data)
{
	mod_timer(&data->debug_timer,
	          round_jiffies_up(jiffies + SSP_DEBUG_TIMER_SEC));
}

void disable_debug_timer(struct ssp_data *data)
{
	del_timer_sync(&data->debug_timer);
	cancel_work_sync(&data->work_debug);
}

int initialize_debug_timer(struct ssp_data *data)
{
	setup_timer(&data->debug_timer, debug_timer_func, (unsigned long)data);

	data->debug_wq = create_singlethread_workqueue("ssp_debug_wq");
	if (!data->debug_wq) {
		return ERROR;
	}

	INIT_WORK(&data->work_debug, debug_work_func);
	return SUCCESS;
}

void print_dataframe(struct ssp_data *data, char *dataframe, int frame_len)
{
	char *raw_data;
	int size = 0;
	int i = 0;

	raw_data = kzalloc(frame_len * 4, GFP_KERNEL);

	for (i = 0; i < frame_len; i++) {
		size += snprintf(raw_data + size, PAGE_SIZE, "%d ",
		                 *(dataframe + i));
	}

	ssp_info("%s", raw_data);
	kfree(raw_data);
}
