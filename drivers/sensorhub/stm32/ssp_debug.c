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
	struct timeval cur_time;

	unsigned char datacount = pchRcvDataFrame[1];
	unsigned int databodysize = iLength - 2;
	char *databody = &pchRcvDataFrame[2];
/*
	if (iLength != DEBUG_DUMP_DATA_SIZE) {
		ssp_errf("data length error(%d)", iLength);
		return FAIL;
	} else
		ssp_errf("length(%d)", databodysize);
*/
	ssp_errf("length(%d)", databodysize);

	if (data->is_ssp_shutdown) {
		ssp_infof("ssp shutdown, stop dumping");
		return FAIL;
	}

	if (data->is_mcu_dump_mode == true)	{
		int ret_write = 0;
		
		wake_lock(&data->ssp_wake_lock);

		if (data->realtime_dump_file == NULL) {
			char strFilePath[100];

			backup_fs = get_fs();
			set_fs(get_ds());

			do_gettimeofday(&cur_time);

			snprintf(strFilePath, sizeof(strFilePath), "%s%d.dump",
				DEBUG_DUMP_FILE_PATH, (int)cur_time.tv_sec);
			data->realtime_dump_file = filp_open(strFilePath,
					O_RDWR | O_CREAT | O_APPEND, 0660);

			ssp_err("save_crash_dump : open file(%s)", strFilePath);

			if (IS_ERR(data->realtime_dump_file)) {
				ssp_errf("Can't open dump file");
				set_fs(backup_fs);
				data->realtime_dump_file = NULL;
				wake_unlock(&data->ssp_wake_lock);
				return FAIL;
			}
		}

		data->total_dump_size += databodysize;
		/* ssp_errf("total receive size(%d)", data->total_dump_size); */
		ret_write = vfs_write(data->realtime_dump_file,
					(char __user *)databody, databodysize,
					&data->realtime_dump_file->f_pos);
		if (ret_write < 0) {
			ssp_errf("Can't write dump to file");
			wake_unlock(&data->ssp_wake_lock);
			return FAIL;
		}

		if (datacount == DEBUG_DUMP_DATA_COMPLETE) {
			ssp_errf("close file(size=%d)", data->total_dump_size);
			filp_close(data->realtime_dump_file, current->files);
			set_fs(backup_fs);
			data->cnt_dump++;
			data->total_dump_size = 0;
			data->realtime_dump_file = NULL;
			data->is_ongoing_dump = false;
		}

		wake_unlock(&data->ssp_wake_lock);

		/*
		if (iLength == 2 * 1024)
			queue_refresh_task(data, 0);
		*/
	}

	return SUCCESS;
}

void ssp_dump_task(struct work_struct *work)
{
#if CONFIG_SEC_DEBUG
	struct ssp_big *big;
	struct file *dump_file;
	struct ssp_msg *msg;
	char *buffer;
	char strFilePath[100];
	struct timeval cur_time;
	mm_segment_t fs;
	int buf_len, packet_len, residue;
	int index = 0, ret_trans = 0, ret_write = 0;

	big = container_of(work, struct ssp_big, work);
	ssp_errf("start ssp dumping (%d)(%d)",
		big->data->is_mcu_dump_mode, big->data->cnt_dump);

	big->data->cnt_dump++;
	wake_lock(&big->data->ssp_wake_lock);

	fs = get_fs();
	set_fs(get_ds());

	if (big->data->is_mcu_dump_mode == true) {
		do_gettimeofday(&cur_time);
#ifdef CONFIG_SENSORS_SSP_ENG
		snprintf(strFilePath, sizeof(strFilePath), "%s%d.dump",
			DUMP_FILE_PATH,	(int)cur_time.tv_sec);
		dump_file = filp_open(strFilePath,
				O_RDWR | O_CREAT | O_APPEND, 0660);
#else
		snprintf(strFilePath, sizeof(strFilePath), "%s.dump",
			DUMP_FILE_PATH);
		dump_file = filp_open(strFilePath,
				O_RDWR | O_CREAT | O_TRUNC, 0660);
#endif

		if (IS_ERR(dump_file)) {
			int ret = PTR_ERR(dump_file);
			ssp_errf("Can't open dump file %d",ret);
			set_fs(fs);
			wake_unlock(&big->data->ssp_wake_lock);
			kfree(big);
			return;
		}
	} else {
		dump_file = NULL;
	}

	buf_len = big->length > DATA_PACKET_SIZE
			? DATA_PACKET_SIZE : big->length;
	buffer = kzalloc(buf_len, GFP_KERNEL);
	residue = big->length;

	while (residue > 0) {
		packet_len = residue > DATA_PACKET_SIZE
				? DATA_PACKET_SIZE : residue;

		msg = kzalloc(sizeof(*msg), GFP_KERNEL);
		msg->cmd = MSG2SSP_AP_GET_BIG_DATA;
		msg->length = packet_len;
		msg->options = AP2HUB_READ | (index++ << SSP_INDEX);
		msg->data = big->addr;
		msg->buffer = buffer;
		msg->free_buffer = 0;

		ret_trans = ssp_spi_sync(big->data, msg, 1000);
		if (ret_trans != SUCCESS) {
			ssp_errf("Fail to receive data %d (%d)",
				ret_trans, residue);
			break;
		}

		if (big->data->is_mcu_dump_mode == true) {
			ret_write = vfs_write(dump_file, (char __user *)buffer,
						packet_len, &dump_file->f_pos);
			if (ret_write < 0) {
				ssp_errf("Can't write dump to file");
				break;
			}
		}
		residue -= packet_len;
	}

	if (big->data->is_mcu_dump_mode == true) {
		if (ret_trans != SUCCESS || ret_write < 0) { /* error case */
			char FAILSTRING[100];

			snprintf(FAILSTRING, sizeof(FAILSTRING),
				"FAIL OCCURED(%d)(%d)(%d)", ret_trans,
				ret_write, big->length);
			vfs_write(dump_file, (char __user *)FAILSTRING,
					strlen(FAILSTRING), &dump_file->f_pos);
		}

		filp_close(dump_file, current->files);
	}

	big->data->is_ongoing_dump = false;

	set_fs(fs);

	wake_unlock(&big->data->ssp_wake_lock);
	kfree(buffer);
	kfree(big);
#endif
	ssp_errf("done");
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
	u8 buf[9] = {0,};
	u32 uSensorCnt;
	int ret = 0;

	gyro_open_calibration(data);
	ret = set_gyro_cal(data);
	if (ret < 0)
		ssp_errf("set_gyro_cal failed\n");

	ret = set_accel_cal(data);
	if (ret < 0)
		ssp_errf("set_accel_cal failed\n");

	udelay(10);

	for (uSensorCnt = 0; uSensorCnt < SENSOR_TYPE_MAX; uSensorCnt++) {
		mutex_lock(&data->enable_mutex);
		if (atomic64_read(&data->aSensorEnable) & (1 << uSensorCnt)) {
			s32 dMsDelay
				= get_msdelay(data->delay[uSensorCnt]);
			memcpy(&buf[0], &dMsDelay, 4);
			memcpy(&buf[4], &data->batch_max_latency[uSensorCnt], 4);
			buf[8] = data->batch_opt[uSensorCnt];
			send_instruction(data, ADD_SENSOR, uSensorCnt, buf, 9);
			udelay(10);
		}
		mutex_unlock(&data->enable_mutex);
	}

	if (data->is_proxraw_enabled == true) {
		s32 dMsDelay = 20;

		memcpy(&buf[0], &dMsDelay, 4);
		send_instruction(data, ADD_SENSOR, SENSOR_TYPE_PROXIMITY_RAW, buf, 4);
	}

	set_light_coef(data);
	set_proximity_threshold(data);
	data->buf[SENSOR_TYPE_PROXIMITY].prox = 0;
	report_sensor_data(data, SENSOR_TYPE_PROXIMITY, &data->buf[SENSOR_TYPE_PROXIMITY]);

#if 0
	if (sec_debug_get_debug_level() > 0) {
		data->is_mcu_dump_mode = true;
		ssp_info("Mcu Dump Enabled");
	}

	ret = ssp_send_cmd(data, MSG2SSP_AP_MCU_SET_DUMPMODE,
			data->is_mcu_dump_mode);
	if (ret < 0)
		ssp_errf("MSG2SSP_AP_MCU_SET_DUMPMODE failed");

#else
#if CONFIG_SEC_DEBUG
	data->is_mcu_dump_mode = sec_debug_is_enabled();
	ret = ssp_send_cmd(data, MSG2SSP_AP_MCU_SET_DUMPMODE,
			data->is_mcu_dump_mode);
	if (ret < 0)
		ssp_errf("MSG2SSP_AP_MCU_SET_DUMPMODE failed");
#endif
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
bool check_wait_event(struct ssp_data *data)
{
	u64 timestamp = get_current_timestamp();
	int check_sensors[2] = {SENSOR_TYPE_ACCELEROMETER, SENSOR_TYPE_LIGHT};
	int i, sensor;
	bool res = false;

	for (i = 0 ; i < 2 ; i++) {
		sensor = check_sensors[i];
		/* The sensor is registered
		   And none batching mode
		   And there is no sensor event over 5sec */
		if ((atomic64_read(&data->aSensorEnable) & (1 << sensor))
			&& data->batch_max_latency[sensor] == 0
			&& data->latest_timestamp[sensor] + 5000000000ULL < timestamp) {

			ssp_info("%s - sensor(%d) last = %lld, cur = %lld",
				__func__, sensor, data->latest_timestamp[sensor], timestamp);
			res = true;
		}
	}

	return res;
}

static void debug_work_func(struct work_struct *work)
{
	unsigned int type;
	struct ssp_data *data = container_of(work, struct ssp_data, work_debug);

	ssp_infof("(%u) - Sensor state: 0x%llx, RC: %u, CC: %u DC: %u TC: %u",
		data->cnt_irq, data->uSensorState,
		data->cnt_reset, data->cnt_com_fail, data->cnt_dump,
		data->cnt_timeout);

	switch (data->fw_dl_state) {
	case FW_DL_STATE_FAIL:
	case FW_DL_STATE_DOWNLOADING:
	case FW_DL_STATE_SYNC:
		ssp_infof("firmware downloading state = %d",
				data->fw_dl_state);
		return;
	}

	for (type = 0; type < SENSOR_TYPE_MAX; type++)
		if ((atomic64_read(&data->aSensorEnable) & (1 << type))
			|| data->batch_max_latency[type])
			print_sensordata(data, type);

	if (((atomic64_read(&data->aSensorEnable) & (1 << SENSOR_TYPE_ACCELEROMETER))
		&& (data->batch_max_latency[SENSOR_TYPE_ACCELEROMETER] == 0)
		&& (data->cnt_irq == 0) && (data->cnt_timeout > 0))
		|| (data->cnt_timeout > LIMIT_TIMEOUT_CNT))
		recovery_mcu(data);

	data->cnt_irq = 0;

	if(data->first_gyro_cal == true)
	{
		int ret = 0;		
		gyro_open_calibration(data);
		ret = set_gyro_cal(data);
		if (ret < 0)
			ssp_errf("set_gyro_cal failed\n");

		data->first_gyro_cal = false;
	}
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
	if (!data->debug_wq)
		return ERROR;

	INIT_WORK(&data->work_debug, debug_work_func);
	return SUCCESS;
}

void print_dataframe(struct ssp_data *data, char *dataframe, int frame_len)
{
	char *raw_data;
	int size = 0;
	int i = 0;

	raw_data = kzalloc(frame_len*4, GFP_KERNEL);
	if (raw_data == NULL)
		return;

	for (i = 0; i < frame_len; i++) {
		size += snprintf(raw_data+size, PAGE_SIZE, "%d ",
			*(dataframe + i));
	}

	ssp_info("%s", raw_data);
	kfree(raw_data);
}
