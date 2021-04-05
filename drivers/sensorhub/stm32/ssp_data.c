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

#include "ssp_data.h"

#define U64_US2NS 1000ULL

/*************************************************************************/
/* SSP parsing the dataframe                                             */
/*************************************************************************/
static void get_timestamp(struct ssp_data *data, char *dataframe,
		int *ptr_data, struct sensor_value *event,
		u16 mode, int type)
{
	u32 ts_delta_us = 0;
	u64 ts_delta_ns = 0;
	u64 current_timestamp = get_current_timestamp();

	memset(&ts_delta_us, 0, 4);
	memcpy(&ts_delta_us, dataframe + *ptr_data, 4);

	ts_delta_ns = (((u64) ts_delta_us) * U64_US2NS);

	if(data->info[type].report_mode == REPORT_MODE_CONTINUOUS)
	{	

		if(data->latest_timestamp[type] == -1) /* first sensordata after resume*/
		{
			data->latest_timestamp[type] = current_timestamp; 
		}
		else
		{
			data->latest_timestamp[type] += ts_delta_ns;
			if(data->latest_timestamp[type] > current_timestamp)
			{
				//ssp_infof("future timestamp : last = %lld, cur = %lld",data->latest_timestamp[type],current_timestamp);
				data->latest_timestamp[type] = current_timestamp;
			}
		}
	}
	else
	{
		data->latest_timestamp[type] = current_timestamp;
	}
	
	event->timestamp = data->latest_timestamp[type];

	*ptr_data += 4;
}

void get_sensordata(struct ssp_data *data, char *dataframe,
		int *ptr_data, int type, struct sensor_value *event)
{
	memcpy(event, dataframe + *ptr_data, data->info[type].get_data_len);
	*ptr_data += data->info[type].get_data_len;
}

int handle_big_data(struct ssp_data *data, char *dataframe, int *ptr_data)
{
	u8 big_type = 0;
	struct ssp_big *big = kzalloc(sizeof(*big), GFP_KERNEL);

	big->data = data;
	big_type = dataframe[(*ptr_data)++];
	memcpy(&big->length, dataframe + *ptr_data, 4);
	*ptr_data += 4;
	memcpy(&big->addr, dataframe + *ptr_data, 4);
	*ptr_data += 4;

	if (big_type >= BIG_TYPE_MAX) {
		kfree(big);
		return FAIL;
	}

	INIT_WORK(&big->work, data->ssp_big_task[big_type]);
	queue_work(data->debug_wq, &big->work);

	return SUCCESS;
}

void refresh_task(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
			struct ssp_data, work_refresh);

	ssp_dbgf("REFESH TASK");
	
	if (data->is_ssp_shutdown == true) {
		ssp_errf("ssp already shutdown");
		return;
	}

	wake_lock(&data->ssp_wake_lock);
	ssp_errf();
	data->cnt_reset++;
	if (initialize_mcu(data) > 0) {
		sync_sensor_state(data);
		ssp_sensorhub_report_notice(data, MSG2SSP_AP_STATUS_RESET);
		if (data->uLastAPState != 0)
			ssp_send_cmd(data, data->uLastAPState, 0);
		if (data->uLastResumeState != 0)
			ssp_send_cmd(data, data->uLastResumeState, 0);
		data->cnt_timeout = 0;
	} else
		data->uSensorState = 0;

	data->is_reset_started = false;
	wake_unlock(&data->ssp_wake_lock);
}

int queue_refresh_task(struct ssp_data *data, int delay)
{
	cancel_delayed_work_sync(&data->work_refresh);

	ssp_dbgf();

	queue_delayed_work(data->debug_wq, &data->work_refresh,
			msecs_to_jiffies(delay));
	return SUCCESS;
}

int parse_dataframe(struct ssp_data *data, char *dataframe, int frame_len)
{
	struct sensor_value event;
	u16 batch_event_count;
	u16 mode;

	int type, index;
	u16 length = 0;
	s16 caldata[3] = {0, };

	if (data->is_ssp_shutdown) {
		ssp_infof("ssp shutdown, do not parse");
		return SUCCESS;
	}

	if (data->debug_enable)
		print_dataframe(data, dataframe, frame_len);

	memset(&event, 0, sizeof(event));

	for (index = 0; index < frame_len;) {
		switch (dataframe[index++]) {
		case MSG2AP_INST_BYPASS_DATA:
			type = dataframe[index++];
			if ((type < 0) || (type >= SENSOR_TYPE_MAX)) {
				ssp_errf("Mcu bypass dataframe err %d", type);
				return ERROR;
			}

			memcpy(&length, dataframe + index, 2);
			index += 2;
			batch_event_count = length;
			mode = length > 1 ? BATCH_MODE_RUN : BATCH_MODE_NONE;

			do {
				get_sensordata(data, dataframe, &index, type, &event);
				get_timestamp(data, dataframe, &index, &event, mode, type);
				report_sensor_data(data, type, &event);

				batch_event_count--;
			} while ((batch_event_count > 0) && (index < frame_len));

			if (batch_event_count > 0)
				ssp_errf("batch count error (%d)", batch_event_count);

			data->is_data_reported[type] = true;
			break;
		case MSG2AP_INST_DEBUG_DATA:
			type = print_mcu_debug(dataframe, &index, frame_len);
			if (type) {
				ssp_errf("Mcu debug dataframe err %d", type);
				return ERROR;
			}
			break;
		case MSG2AP_INST_LIBRARY_DATA:
			memcpy(&length, dataframe + index, 2);
			index += 2;
			ssp_sensorhub_handle_data(data, dataframe, index,
					index + length);
			index += length;
			break;
		case MSG2AP_INST_BIG_DATA:
			handle_big_data(data, dataframe, &index);
			break;
		case MSG2AP_INST_META_DATA:
			event.meta_data.what = dataframe[index++];
			event.meta_data.sensor = dataframe[index++];
			report_meta_data(data, SENSOR_TYPE_META, &event);
			break;
		case MSG2AP_INST_TIME_SYNC:
			data->is_time_syncing = true;
			break;
		case MSG2AP_INST_RESET:
			data->uSensorState = 0;
			ssp_infof("Reset MSG received from MCU");
			if(data->is_probe_done == true)
				queue_refresh_task(data, 0);
			else
				ssp_infof("skip reset msg");
			break;
		case MSG2AP_INST_GYRO_CAL:
			ssp_infof("Gyro caldata received from MCU\n");
			memcpy(caldata, dataframe + index, sizeof(caldata));
			wake_lock(&data->ssp_wake_lock);
			save_gyro_cal_data(data, caldata);
			wake_unlock(&data->ssp_wake_lock);
			index += sizeof(caldata);
			break;
		case SH_MSG2AP_GYRO_CALIBRATION_EVENT_OCCUR:
			data->gyro_lib_state = GYRO_CALIBRATION_STATE_EVENT_OCCUR;
			ssp_infof("Gyro caldata event occur received from MCU\n");
			break;
		case MSG2AP_INST_DUMP_DATA:
			debug_crash_dump(data, dataframe, frame_len);
			break;
		}
	}

	return SUCCESS;
}

void initialize_function_pointer(struct ssp_data *data)
{
	data->ssp_big_task[BIG_TYPE_DUMP] = ssp_dump_task;
	data->ssp_big_task[BIG_TYPE_READ_LIB] = ssp_read_big_library_task;
}
