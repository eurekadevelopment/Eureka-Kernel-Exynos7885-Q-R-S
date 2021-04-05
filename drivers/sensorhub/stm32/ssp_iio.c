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

#include "ssp_iio.h"

static void init_sensorlist(struct ssp_data *data)
{
	struct sensor_info sensorinfo[SENSOR_TYPE_MAX] = {
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_ACCELEROMETER,
		SENSOR_INFO_GEOMAGNETIC_FIELD,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_GYRO,
		SENSOR_INFO_LIGHT,
		SENSOR_INFO_PRESSURE,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_PROXIMITY,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_ROTATION_VECTOR,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_MAGNETIC_FIELD_UNCALIBRATED,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_GYRO_UNCALIBRATED,
		SENSOR_INFO_SIGNIFICANT_MOTION,
		SENSOR_INFO_STEP_DETECTOR,
		SENSOR_INFO_STEP_COUNTER,
		SENSOR_INFO_GEOMAGNETIC_ROTATION_VECTOR, 
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_TILT_DETECTOR,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_PICK_UP_GESTURE,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_META,
		SENSOR_INFO_UNKNOWN,
		SENSOR_INFO_PROXIMITY_RAW,
		SENSOR_INFO_GEOMAGNETIC_POWER,
		SENSOR_INFO_INTERRUPT_GYRO,
};	

	memcpy(&data->info, sensorinfo, sizeof(data->info));
}

static int ssp_preenable(struct iio_dev *indio_dev)
{
	return 0;
}

static int ssp_predisable(struct iio_dev *indio_dev)
{
	return 0;
}

static const struct iio_buffer_setup_ops ssp_iio_ring_setup_ops = {
	.preenable = &ssp_preenable,
	.predisable = &ssp_predisable,
};

static int ssp_iio_configure_ring(struct iio_dev *indio_dev)
{
	struct iio_buffer *ring;

	ring = iio_kfifo_allocate();
	if (!ring)
		return -ENOMEM;

	ring->scan_timestamp = true;
	ring->bytes_per_datum = 8;
	indio_dev->buffer = ring;
	indio_dev->setup_ops = &ssp_iio_ring_setup_ops;
	indio_dev->modes |= INDIO_BUFFER_SOFTWARE;

	return 0;
}

static void ssp_iio_push_buffers(struct iio_dev *indio_dev, u64 timestamp,
				char *data, int data_len)
{
	char buf[data_len + sizeof(timestamp)];

	if (!indio_dev || !data)
		return;

	memcpy(buf, data, data_len);
	memcpy(buf + data_len, &timestamp, sizeof(timestamp));
	mutex_lock(&indio_dev->mlock);
	iio_push_to_buffers(indio_dev, buf);
	mutex_unlock(&indio_dev->mlock);
}

static void report_prox_raw_data(struct ssp_data *data, int type,
	struct sensor_value *proxrawdata)
{
	if (data->uFactoryProxAvg[0]++ >= PROX_AVG_READ_NUM) {
		data->uFactoryProxAvg[2] /= PROX_AVG_READ_NUM;
		data->buf[type].prox_raw[1] = (u16)data->uFactoryProxAvg[1];
		data->buf[type].prox_raw[2] = (u16)data->uFactoryProxAvg[2];
		data->buf[type].prox_raw[3] = (u16)data->uFactoryProxAvg[3];

		data->uFactoryProxAvg[0] = 0;
		data->uFactoryProxAvg[1] = 0;
		data->uFactoryProxAvg[2] = 0;
		data->uFactoryProxAvg[3] = 0;
	} else {
		data->uFactoryProxAvg[2] += proxrawdata->prox_raw[0];

		if (data->uFactoryProxAvg[0] == 1)
			data->uFactoryProxAvg[1] = proxrawdata->prox_raw[0];
		else if (proxrawdata->prox_raw[0] < data->uFactoryProxAvg[1])
			data->uFactoryProxAvg[1] = proxrawdata->prox_raw[0];

		if (proxrawdata->prox_raw[0] > data->uFactoryProxAvg[3])
			data->uFactoryProxAvg[3] = proxrawdata->prox_raw[0];
	}

	data->buf[type].prox_raw[0] = proxrawdata->prox_raw[0];
}

void report_sensor_data(struct ssp_data *data, int type,
			struct sensor_value *event)
{
	if (type == SENSOR_TYPE_PROXIMITY) {
		ssp_info("Proximity Sensor Detect : %u, raw : %u",
			event->prox, event->prox_ex);

	} else if (type == SENSOR_TYPE_PROXIMITY_RAW) {
		report_prox_raw_data(data, type, event);
		return;

	} else if (type == SENSOR_TYPE_LIGHT) {
		event->a_gain &= 0x03;
		if(data->light_log_cnt < 3)
		{
			ssp_info("Light Sensor : r=%d g=%d b=%d c=%d atime=%d again=%d",
			data->buf[SENSOR_TYPE_LIGHT].r,data->buf[SENSOR_TYPE_LIGHT].g,data->buf[SENSOR_TYPE_LIGHT].b,
			data->buf[SENSOR_TYPE_LIGHT].w,data->buf[SENSOR_TYPE_LIGHT].a_time,data->buf[SENSOR_TYPE_LIGHT].a_gain);	
			data->light_log_cnt++;
		}
	} else if (type == SENSOR_TYPE_STEP_COUNTER) {
		data->buf[type].step_total += event->step_diff;
	}

	memcpy(&data->buf[type], (char *)event, data->info[type].get_data_len);
	ssp_iio_push_buffers(data->indio_devs[type], event->timestamp,
			(char *)&data->buf[type], data->info[type].report_data_len);

	/* wake-up sensor */
	if (type == SENSOR_TYPE_PROXIMITY || type == SENSOR_TYPE_SIGNIFICANT_MOTION
		|| type == SENSOR_TYPE_TILT_DETECTOR || type == SENSOR_TYPE_PICK_UP_GESTURE) {
		wake_lock_timeout(&data->ssp_wake_lock, 0.3 * HZ);
	}
}

void report_meta_data(struct ssp_data *data, int type, struct sensor_value *s)
{
	ssp_infof("what: %d, sensor: %d",
		s->meta_data.what, s->meta_data.sensor);

	if ((s->meta_data.sensor == SENSOR_TYPE_ACCELEROMETER)
		|| (s->meta_data.sensor == SENSOR_TYPE_GEOMAGNETIC_FIELD)
		|| (s->meta_data.sensor == SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED)
		|| (s->meta_data.sensor == SENSOR_TYPE_GYROSCOPE)
		|| (s->meta_data.sensor == SENSOR_TYPE_PRESSURE)
		|| (s->meta_data.sensor == SENSOR_TYPE_ROTATION_VECTOR)
		|| (s->meta_data.sensor == SENSOR_TYPE_GAME_ROTATION_VECTOR)
		|| (s->meta_data.sensor == SENSOR_TYPE_STEP_DETECTOR)
		|| (s->meta_data.sensor == SENSOR_TYPE_INTERRUPT_GYRO)) {
		char *meta_event
			= kzalloc(data->info[s->meta_data.sensor].report_data_len,
					GFP_KERNEL);
		if (!meta_event) {
			ssp_errf("fail to allocate memory for meta event");
			return;
		}

		memset(meta_event, META_EVENT,
			data->info[s->meta_data.sensor].report_data_len);
		ssp_iio_push_buffers(data->indio_devs[s->meta_data.sensor],
				META_TIMESTAMP, meta_event,
				data->info[s->meta_data.sensor].report_data_len);
		kfree(meta_event);
	} else {
		ssp_iio_push_buffers(data->indio_devs[type],
				META_TIMESTAMP, (char *)&s->meta_data,
				sizeof(s->meta_data));
	}
}

static void *init_indio_device(struct ssp_data *data,
			const struct iio_info *info,
			const struct iio_chan_spec *channels,
			const char *device_name)
{
	struct iio_dev *indio_dev;
	int ret = 0;

	indio_dev = iio_device_alloc(0);
	if (!indio_dev)
		goto err_alloc;

	indio_dev->name = device_name;
	indio_dev->dev.parent = &data->spi->dev;
	indio_dev->info = info;
	indio_dev->channels = channels;
	indio_dev->num_channels = 1;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	ret = ssp_iio_configure_ring(indio_dev);
	if (ret)
		goto err_config_ring;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_register_device;

	return indio_dev;


err_register_device:
	ssp_err("fail to register %s device", device_name);
	iio_kfifo_free(indio_dev->buffer);
err_config_ring:
	pr_err("[SSP]: failed to configure %s buffer\n", indio_dev->name);
	iio_device_unregister(indio_dev);
err_alloc:
	ssp_err("fail to allocate memory for iio %s device", device_name);

	return NULL;
}

static const struct iio_info indio_info = {
	.driver_module = THIS_MODULE,
};

int initialize_indio_dev(struct ssp_data *data)
{
	int timestamp_len = 0;
	int type;

	init_sensorlist(data);

	for (type = 0; type < SENSOR_TYPE_MAX; type++) {
		if (!data->info[type].enable || (data->info[type].report_data_len == 0))
			continue;

		timestamp_len = sizeof(data->buf[type].timestamp);

		data->indio_channels[type].type = IIO_TIMESTAMP;
		data->indio_channels[type].channel = IIO_CHANNEL;
		data->indio_channels[type].scan_index = IIO_SCAN_INDEX;
		data->indio_channels[type].scan_type.sign = IIO_SIGN;
		data->indio_channels[type].scan_type.realbits =
			(data->info[type].report_data_len+timestamp_len)*BITS_PER_BYTE;
		data->indio_channels[type].scan_type.storagebits =
			(data->info[type].report_data_len+timestamp_len)*BITS_PER_BYTE;
		data->indio_channels[type].scan_type.shift = IIO_SHIFT;

		data->indio_devs[type]
			= (struct iio_dev *)init_indio_device(data,
				&indio_info, &data->indio_channels[type],
				data->info[type].name);
		
		ssp_err("init %s iio dev, type = %d", data->info[type].name, type);
		
		if (!data->indio_devs[type]) {
			ssp_err("fail to init %s iio dev", data->info[type].name);
			remove_indio_dev(data);
			return ERROR;
		}
	}

	return SUCCESS;
}

void remove_indio_dev(struct ssp_data *data)
{
	int type;

	for (type = SENSOR_TYPE_MAX-1; type >= 0; type--) {
		if (data->indio_devs[type])
			iio_device_unregister(data->indio_devs[type]);
	}
}
