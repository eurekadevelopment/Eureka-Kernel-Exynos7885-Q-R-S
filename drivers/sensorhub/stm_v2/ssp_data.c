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
void get_timestamp(struct ssp_data *data, char *dataframe,
                   int *ptr_data, struct sensor_value *event,
                   u16 mode, int type)
{
	u32 ts_delta_us = 0;
	u64 ts_delta_ns = 0;
	u64 current_timestamp = get_current_timestamp();

	memset(&ts_delta_us, 0, 4);
	memcpy(&ts_delta_us, dataframe + *ptr_data, 4);

	ts_delta_ns = (((u64) ts_delta_us) * U64_US2NS);

	if (data->info[type].report_mode == REPORT_MODE_CONTINUOUS) {

		if (data->latest_timestamp[type] == 0) { /* first sensordata after resume*/
			data->latest_timestamp[type] = current_timestamp;
		} else {
			data->latest_timestamp[type] += ts_delta_ns;
			if (data->latest_timestamp[type] > current_timestamp) {
				//ssp_infof("future timestamp : last = %lld, cur = %lld",data->latest_timestamp[type],current_timestamp);
				data->latest_timestamp[type] = current_timestamp;
			}
		}
	} else {
		data->latest_timestamp[type] = current_timestamp;
	}

	event->timestamp = data->latest_timestamp[type];

	*ptr_data += 4;
}

int set_sensor_position(struct ssp_data *data)
{
	int ret[3] = {0,};
	ssp_infof();

	ret[0] = ssp_send_command(data, CMD_SETVALUE, SENSOR_TYPE_ACCELEROMETER,
	                          SENSOR_AXIS, 0, (char *) & (data->accel_position), sizeof(data->accel_position),
	                          NULL, NULL);
	ret[1] = ssp_send_command(data, CMD_SETVALUE, SENSOR_TYPE_GYROSCOPE,
	                          SENSOR_AXIS, 0, (char *) & (data->accel_position), sizeof(data->accel_position),
	                          NULL, NULL);
	ret[2] = ssp_send_command(data, CMD_SETVALUE, SENSOR_TYPE_GEOMAGNETIC_FIELD,
	                          SENSOR_AXIS, 0, (char *) & (data->mag_position), sizeof(data->mag_position),
	                          NULL, NULL);

	ssp_info("Sensor Posision A : %u, G : %u, M: %u, P: 0",
	         data->accel_position, data->accel_position, data->mag_position);

	if ((ret[0] & ret[1] & ret[2]) != SUCCESS) {
		ssp_errf("fail to set_sensor_position %d %d %d", ret[0], ret[1], ret[2]);
		return ERROR;
	}

	return 0;
}

#ifdef CONFIG_SENSORS_SSP_PROXIMITY
void set_proximity_threshold(struct ssp_data *data)
{
	int ret = 0;
	char prox_th[4] = {0, };

	if (!(data->uSensorState & (1ULL << SENSOR_TYPE_PROXIMITY))) {
		ssp_infof("Skip this function!, proximity sensor is not connected(0x%llx)",
		          data->uSensorState);
		return;
	}

	prox_th[0] = (char) data->uProxHiThresh;
	prox_th[1] = (char) data->uProxLoThresh;
	prox_th[2] = (char) data->uProxHiThresh_detect;
	prox_th[3] = (char) data->uProxLoThresh_detect;

	ret = ssp_send_command(data, CMD_SETVALUE, SENSOR_TYPE_PROXIMITY,
	                       PROXIMITY_THRESHOLD, 0, prox_th, sizeof(prox_th), NULL, NULL);

	if (ret != SUCCESS) {
		ssp_err("SENSOR_PROXTHRESHOLD CMD fail %d", ret);
		return;
	}

	ssp_info("Proximity Threshold - %u, %u, %u, %u", data->uProxHiThresh,
	         data->uProxLoThresh,
	         data->uProxHiThresh_detect, data->uProxLoThresh_detect);
}
#endif

void set_proximity_barcode_enable(struct ssp_data *data, bool bEnable)
{
	data->is_barcode_enabled = bEnable;

	ssp_info("Proximity Barcode En : %u", bEnable);
}

#ifdef CONFIG_SENSORS_SSP_LIGHT
void set_light_coef(struct ssp_data *data)
{
	int ret = 0;

	if (!(data->uSensorState & (1ULL << SENSOR_TYPE_LIGHT))) {
		pr_info("[SSP]: %s - Skip this function!!!,"\
		        "light sensor is not connected(0x%llx)\n",
		        __func__, data->uSensorState);
		return;
	}

	ret = ssp_send_command(data, CMD_SETVALUE, SENSOR_TYPE_LIGHT, LIGHT_COEF, 0,
	                       (char *)data->light_coef, sizeof(data->light_coef), NULL, NULL);

	if (ret != SUCCESS) {
		pr_err("[SSP]: %s - MSG2SSP_AP_SET_LIGHT_COEF CMD fail %d\n",
		       __func__, ret);
		return;
	}

	pr_info("[SSP]: %s - %d %d %d %d %d %d %d\n", __func__,
	        data->light_coef[0], data->light_coef[1], data->light_coef[2],
	        data->light_coef[3], data->light_coef[4], data->light_coef[5],
	        data->light_coef[6]);
}
#endif

uint64_t get_sensor_scanning_info(struct ssp_data *data)
{
	int ret = 0, z = 0;
	uint64_t result = 0;
	char *buffer = NULL;
	unsigned int buffer_length;

	char sensor_scanning_state[SENSOR_TYPE_MAX + 1];

	ret = ssp_send_command(data, CMD_GETVALUE, TYPE_MCU, SENSOR_SCAN_RESULT, 1000,
	                       NULL, 0, &buffer, &buffer_length);
	if (ret < 0) {
		ssp_errf("MSG2SSP_AP_SENSOR_SCANNING fail %d", ret);
	} else if (buffer_length != sizeof(uint64_t)) {
		ssp_errf("SENSOR_SCAN_RESULT length is wrong");
	} else {
		memcpy(&result, buffer, buffer_length);
	}

	sensor_scanning_state[SENSOR_TYPE_MAX] = '\0';
	for (z = 0; z < SENSOR_TYPE_MAX; z++)
		sensor_scanning_state[SENSOR_TYPE_MAX - 1 - z]
		        = (result & (1ULL << z)) ? '1' : '0';

	ssp_info("state(0x%llx): %s", result, sensor_scanning_state);

	/*change state format same as flagship*/
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX] = '\0';
	for (z = 0; z < BIG_DATA_SENSOR_TYPE_MAX; z++) {
		data->sensor_state[z] = '0';
	}

	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 -
	                                            BIG_DATA_SENSOR_TYPE_ACCELEROMETER] =
	                           sensor_scanning_state[SENSOR_TYPE_MAX - 1 - SENSOR_TYPE_ACCELEROMETER];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 -
	                                            BIG_DATA_SENSOR_TYPE_GYROSCOPE] =
	                           sensor_scanning_state[SENSOR_TYPE_MAX - 1 - SENSOR_TYPE_GYROSCOPE];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 -
	                                            BIG_DATA_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED] =
	                           sensor_scanning_state[SENSOR_TYPE_MAX - 1 -
	                                                                 SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 -
	                                            BIG_DATA_SENSOR_TYPE_GEOMAGNETIC_POWER] =
	                           sensor_scanning_state[SENSOR_TYPE_MAX - 1 - SENSOR_TYPE_GEOMAGNETIC_POWER];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 -
	                                            BIG_DATA_SENSOR_TYPE_GEOMAGNETIC_FIELD] =
	                           sensor_scanning_state[SENSOR_TYPE_MAX - 1 - SENSOR_TYPE_GEOMAGNETIC_FIELD];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 - BIG_DATA_SENSOR_TYPE_PRESSURE]
	        =
	                sensor_scanning_state[SENSOR_TYPE_MAX - 1 - SENSOR_TYPE_PRESSURE];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 -
	                                            BIG_DATA_SENSOR_TYPE_PROXIMITY] =
	                           sensor_scanning_state[SENSOR_TYPE_MAX - 1 - SENSOR_TYPE_PROXIMITY];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 - BIG_DATA_SENSOR_TYPE_LIGHT] =
	        sensor_scanning_state[SENSOR_TYPE_MAX - 1 - SENSOR_TYPE_LIGHT];
	data->sensor_state[BIG_DATA_SENSOR_TYPE_MAX - 1 -
	                                            BIG_DATA_SENSOR_TYPE_PROXIMITY_RAW] =
	                           sensor_scanning_state[SENSOR_TYPE_MAX - 1 - SENSOR_TYPE_PROXIMITY_RAW];

	ssp_info("state2: %s", data->sensor_state);

	if (buffer != NULL) {
		kfree(buffer);
	}

	return result;
}

unsigned int get_firmware_rev(struct ssp_data *data)
{
	int ret;
	u32 result = SSP_INVALID_REVISION;
	char *buffer = NULL;
	int buffer_length;

	ret = ssp_send_command(data, CMD_GETVALUE, TYPE_MCU, VERSION_INFO, 1000, NULL,
	                       0, &buffer, &buffer_length);

	if (ret != SUCCESS) {
		ssp_errf("transfer fail %d", ret);
	} else if (buffer_length != sizeof(result)) {
		ssp_errf("VERSION_INFO length is wrong");
	} else {
		memcpy(&result, buffer, buffer_length);
	}

	if (buffer != NULL) {
		kfree(buffer);
	}

	return result;
}

void get_sensordata(struct ssp_data *data, char *dataframe,
                    int *ptr_data, int type, struct sensor_value *event)
{
	memcpy(event, dataframe + *ptr_data, data->info[type].get_data_len);
	*ptr_data += data->info[type].get_data_len;
	memcpy(&data->buf[type], (char *)event, data->info[type].get_data_len);
}

void save_callstack(struct ssp_data *data, char *dataframe, int *index)
{
	u8 size;
	memcpy(&size, dataframe + (*index), 1);
	*index += 1;

	if (size % 4 == 0) {
		int i;
		u32 *callstack_buffer = (u32 *)(dataframe + (*index));
		int num = size / sizeof(u32);

		if (data->callstack_data != NULL) {
			kfree(data->callstack_data);
		}

		data->callstack_data = kzalloc((num * 12) - 1, GFP_KERNEL);
		for (i = 0; i < num; i++) {
			if (i == num - 1) {
				snprintf(&(data->callstack_data[i * 12]), PAGE_SIZE, "0x%08x", callstack_buffer[i]);
			} else {
				snprintf(&(data->callstack_data[i * 12]), PAGE_SIZE, "0x%08x, ", callstack_buffer[i]);
			}
		}

		ssp_infof("%s", data->callstack_data);
	} else {
		ssp_errf("length error %d", size);
	}
	*index += size;
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
	if (initialize_mcu(data) >= 0) {
		sync_sensor_state(data);
#ifdef SENSOR_TYPE_SCONTEXT
		report_scontext_notice_data(data, SCONTEXT_AP_STATUS_RESET);
#else
		ssp_sensorhub_report_notice(data, SCONTEXT_AP_STATUS_RESET);
#endif
		if (data->uLastAPState != 0) {
			ssp_send_status(data, data->uLastAPState);
		}
		data->cnt_timeout = 0;
	} else {
		data->uSensorState = 0;
	}

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
	bool parsing_error = false;

	if (data->is_ssp_shutdown) {
		ssp_infof("ssp shutdown, do not parse");
		return SUCCESS;
	}

	if (data->debug_enable) {
		print_dataframe(data, dataframe, frame_len);
	}

	memset(&event, 0, sizeof(event));

	for (index = 0; index < frame_len && !parsing_error;) {
		switch (dataframe[index++]) {
		case MSG2AP_INST_BYPASS_DATA:
			type = dataframe[index++];
			if ((type < 0) || (type >= SENSOR_TYPE_MAX)) {
				ssp_errf("Mcu bypass dataframe err %d", type);
				parsing_error = true;
				break;
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

			if (batch_event_count > 0) {
				ssp_errf("batch count error (%d)", batch_event_count);
			}

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
#ifdef SENSOR_TYPE_SCONTEXT
			report_scontext_data(data, dataframe + index, length);
#else
			ssp_sensorhub_handle_data(data, dataframe, index,
			                          index + length);
#endif
			index += length;
			break;
		case MSG2AP_INST_META_DATA:
			event.meta_data.what = dataframe[index++];
			event.meta_data.sensor = dataframe[index++];
			if ((event.meta_data.sensor < 0) || (event.meta_data.sensor >= SENSOR_TYPE_MAX)) {
				ssp_errf("mcu meta data sensor dataframe err %d", event.meta_data.sensor);
				parsing_error = true;
				break;
			}
			report_meta_data(data, &event);
			break;
		case MSG2AP_INST_RESET:
			data->uSensorState = 0;
			ssp_infof("Reset MSG received from MCU");
			if (data->is_probe_done == true) {
				queue_refresh_task(data, 0);
			} else {
				ssp_infof("skip reset msg");
			}
			break;
#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
		case MSG2AP_INST_GYRO_CAL:
			{
				s16 caldata[3] = {0, };
				ssp_infof("Gyro caldata received from MCU\n");
				memcpy(caldata, dataframe + index, sizeof(caldata));

				wake_lock(&data->ssp_wake_lock);
				save_gyro_cal_data(data, caldata);
				wake_unlock(&data->ssp_wake_lock);
				index += sizeof(caldata);
			}
			break;
#endif
		case MSG2AP_INST_DUMP_DATA:
			debug_crash_dump(data, dataframe, frame_len);
			index += 1025;
			break;
		case MSG2AP_INST_CALLSTACK :
			save_callstack(data, dataframe, &index);
			break;
		default :
			ssp_errf("0x%x cmd doesn't support", dataframe[index++]);
			parsing_error = true;
			break;
		}
	}

	if(parsing_error){
		print_dataframe(data, dataframe, frame_len);
		return ERROR;
	}

	return SUCCESS;
}
