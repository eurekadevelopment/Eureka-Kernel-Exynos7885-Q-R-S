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
#include "ssp_sensor_dump.h"

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
	return div_s64(delay, NSEC_PER_MSEC);
}

static void enable_sensor(struct ssp_data *data,
                          int type, int64_t delay)
{
	u8 uBuf[8];
	u64 new_enable = 0;
	s32 batch_latency = 0;
	int64_t temp_delay = data->delay[type];
	s32 ms_delay = get_msdelay(delay);
	int ret = 0;

	data->delay[type] = delay;
	batch_latency = data->batch_max_latency[type];

	switch (data->aiCheckStatus[type]) {
	case ADD_SENSOR_STATE:
		ssp_infof("ADD %s , type %d DELAY %lld ns", data->info[type].name, type, delay);

		if (type == SENSOR_TYPE_PROXIMITY) {
#ifdef CONFIG_SENSORS_SSP_PROXIMITY
			set_proximity_threshold(data);
			ssp_infof("send threshold hi %d, low %d \n", data->uProxHiThresh,
			          data->uProxLoThresh);
#endif
#ifdef SENSOR_TYPE_LIGHT_CCT
		} else if (type == SENSOR_TYPE_LIGHT || type == SENSOR_TYPE_LIGHT_CCT) {
#ifdef CONFIG_SENSORS_SSP_LIGHT
			data->light_log_cnt = 0;
#endif
		}
#else
		} else if (type == SENSOR_TYPE_LIGHT) {
#ifdef CONFIG_SENSORS_SSP_LIGHT
			data->light_log_cnt = 0;
#endif
		}
#endif

		memcpy(&uBuf[0], &ms_delay, 4);
		memcpy(&uBuf[4], &batch_latency, 4);

		ret = make_command(data, ADD_SENSOR,
		                   type, uBuf, 8);
		ssp_infof("delay %d, timeout %d, ret%d",
		          ms_delay, batch_latency, ret);
		if (ret < 0) {
			new_enable =
			        (uint64_t)atomic64_read(&data->aSensorEnable)
			        & (~(uint64_t)(1ULL << type));
			atomic64_set(&data->aSensorEnable, new_enable);

			data->aiCheckStatus[type] = NO_SENSOR_STATE;
			break;
		} else {
			new_enable =
			        (uint64_t)atomic64_read(&data->aSensorEnable)
			        | ((uint64_t)(1ULL << type));

			atomic64_set(&data->aSensorEnable, new_enable);
		}

		data->aiCheckStatus[type] = RUNNING_SENSOR_STATE;

		break;
	case RUNNING_SENSOR_STATE:
		if (get_msdelay(temp_delay)
		    == get_msdelay(data->delay[type])) {
			break;
		}

		ssp_infof("Change %llu, New = %lldns",
		          (1ULL << type), delay);

		memcpy(&uBuf[0], &ms_delay, 4);
		memcpy(&uBuf[4], &batch_latency, 4);
		make_command(data, CHANGE_DELAY, type, uBuf, 8);

		break;
	default:
		data->aiCheckStatus[type] = ADD_SENSOR_STATE;
	}
}

/*************************************************************************/
/* SSP data enable function                                              */
/*************************************************************************/

static int ssp_remove_sensor(struct ssp_data *data,
                             unsigned int type, uint64_t new_enable)
{
	u8 uBuf[4] = {0, };
	int64_t delay = data->delay[type];

	ssp_infof("REMOVE %s, type %d, current state = %lld",
	          data->info[type].name, type, new_enable);

	data->delay[type] = DEFUALT_POLLING_DELAY;
	data->batch_max_latency[type] = 0;
	data->batch_opt[type] = 0;

	if (!data->is_ssp_shutdown)
		if (atomic64_read(&data->aSensorEnable) & (1ULL << type)) {
			s32 ms_delay = get_msdelay(delay);
			memcpy(&uBuf[0], &ms_delay, 4);

			make_command(data, REMOVE_SENSOR,
			             type, uBuf, 4);

			new_enable =
			        (uint64_t)atomic64_read(&data->aSensorEnable)
			        & (~(uint64_t)(1ULL << type));
			atomic64_set(&data->aSensorEnable, new_enable);
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

	return snprintf(buf, PAGE_SIZE, "%d\n", !data->is_ssp_shutdown);
}

static ssize_t set_enable_irq(struct device *dev,
                              struct device_attribute *attr, const char *buf, size_t size)
{
	u8 dTemp;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtou8(buf, 10, &dTemp) < 0) {
		return -EINVAL;
	}

	ssp_infof("%d start", dTemp);
	if (dTemp) {
		data->is_reset_from_sysfs = true;
		reset_mcu(data);
		enable_debug_timer(data);
	} else if (!dTemp) {
		disable_debug_timer(data);
		ssp_enable(data, 0);
	} else {
		ssp_errf("invalid value");
	}
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
		if ((atomic64_read(&data->aSensorEnable) & (1ULL << type))
		    != (new_enable & (1ULL << type))) {

			if (!(new_enable & (1ULL << type))) {
				data->is_data_reported[type] = false;
				ssp_remove_sensor(data, type,
				                  new_enable); /* disable */
			} else { /* Change to ADD_SENSOR_STATE from KitKat */
				if (data->aiCheckStatus[type]
				    == INITIALIZATION_STATE) {
					if (type == SENSOR_TYPE_ACCELEROMETER) {
#ifdef CONFIG_SENSORS_SSP_ACCELOMETER
						int ret = 0;
						accel_open_calibration(data);
						ret = set_accel_cal(data);
						if (ret < 0) {
							ssp_errf("set_accel_cal failed %d", ret);
						}
#endif
					} else if (type == SENSOR_TYPE_PRESSURE) {
#ifdef CONFIG_SENSORS_SSP_BAROMETER
						pressure_open_calibration(data);
#endif
					} else if (type == SENSOR_TYPE_PROXIMITY) {
#ifdef CONFIG_SENSORS_SSP_PROXIMITY
						set_proximity_threshold(data);
#endif
					}
				}
				data->aiCheckStatus[type] = ADD_SENSOR_STATE;

				enable_sensor(data, type, data->delay[type]);
			}
			break;
		}
	}

	mutex_unlock(&data->enable_mutex);

	return size;
}

static ssize_t set_flush(struct device *dev,
                         struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dTemp;
	u8 sensor_type = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dTemp) < 0) {
		return -EINVAL;
	}

	sensor_type = (u8)dTemp;
	if (!(atomic64_read(&data->aSensorEnable) & (1ULL << sensor_type))) {
		ssp_infof("ssp sensor is not enabled(%d)", sensor_type);
	}

	if (flush(data, sensor_type) < 0) {
		ssp_errf("ssp returns error for flush(%x)", sensor_type);
		return -EINVAL;
	}
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

	if (kstrtoll(buf, 10, &debug_enable) < 0) {
		return -EINVAL;
	}

	if (debug_enable != 1 && debug_enable != 0) {
		return -EINVAL;
	}

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

	if (position < 0 || position > 7) {
		return -EINVAL;
	}

	if (sensor == SENSOR_TYPE_ACCELEROMETER) {
		data->accel_position = position;
	} else if (sensor == SENSOR_TYPE_GEOMAGNETIC_FIELD) {
		data->mag_position = position;
	} else {
		return -EINVAL;
	}

	ret = set_sensor_position(data);
	if (ret < 0) {
		ssp_errf("set_sensor_position failed");
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

static ssize_t show_reset_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	ssize_t ret = 0;
	
	if(data->reset_type == RESET_KERNEL_NO_EVENT) {
		ret = sprintf(buf, "No Event\n");	
	} else if(data->reset_type == RESET_KERNEL_TIME_OUT) {
		ret = sprintf(buf, "Time Out\n");
	} else if(data->reset_type == RESET_MCU_CRASHED) {
		ret = sprintf(buf, "%s\n", data->callstack_data);
	} 

	data->reset_type = RESET_INIT_VALUE;
	
	return ret;
}

static ssize_t sensor_dump_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	int types[] = SENSOR_DUMP_SENSOR_LIST;
	char str_no_sensor_dump[] = "there is no sensor dump";
	int i = 0, ret;
	char *sensor_dump;
	char temp[sensor_dump_length(DUMPREGISTER_MAX_SIZE) + LENGTH_SENSOR_TYPE_MAX + 2] = {0,};

	sensor_dump = (char *)kzalloc((sensor_dump_length(DUMPREGISTER_MAX_SIZE) + LENGTH_SENSOR_TYPE_MAX +
	                               3) * (sizeof(types) / sizeof(types[0])), GFP_KERNEL);

	for (i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
		if (data->sensor_dump[types[i]] != NULL) {
			snprintf(temp, (int)strlen(data->sensor_dump[types[i]]) + LENGTH_SENSOR_TYPE_MAX + 3,
			         "%3d\n%s\n\n", types[i],
			         data->sensor_dump[types[i]]);                  /* %3d -> 3 : LENGTH_SENSOR_TYPE_MAX */
			strcpy(&sensor_dump[(int)strlen(sensor_dump)], temp);
		}
	}

	if ((int)strlen(sensor_dump) == 0) {
		ret = snprintf(buf, (int)strlen(str_no_sensor_dump) + 1, "%s\n", str_no_sensor_dump);
	} else {
		ret = snprintf(buf, (int)strlen(sensor_dump) + 1, "%s\n", sensor_dump);
	}

	kfree(sensor_dump);

	return ret;
}

static ssize_t sensor_dump_store(struct device *dev, struct device_attribute *attr, const char *buf,
                                 size_t size)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	int sensor_type, ret;
	char name[LENGTH_SENSOR_NAME_MAX + 1] = {0,};

	sscanf(buf, "%30s", name);              /* 30 : LENGTH_SENSOR_NAME_MAX */

	if ((strcmp(name, "all")) == 0) {
		ret = send_all_sensor_dump_command(data);
	} else {
		if (strcmp(name, "accelerometer") == 0) {
			sensor_type = SENSOR_TYPE_ACCELEROMETER;
		} else if (strcmp(name, "gyroscope") == 0) {
			sensor_type = SENSOR_TYPE_GYROSCOPE;
		} else if (strcmp(name, "magnetic") == 0) {
			sensor_type = SENSOR_TYPE_GEOMAGNETIC_FIELD;
		} else if (strcmp(name, "pressure") == 0) {
			sensor_type = SENSOR_TYPE_PRESSURE;
		} else if (strcmp(name, "proximity") == 0) {
			sensor_type = SENSOR_TYPE_PROXIMITY;
		} else if (strcmp(name, "light") == 0) {
			sensor_type = SENSOR_TYPE_LIGHT;
		} else {
			ssp_errf("is not supported : %s", buf);
			sensor_type = -1;
			return -EINVAL;
		}
		ret = send_sensor_dump_command(data, sensor_type);
	}

	return (ret == SUCCESS) ? size : ret;
}

#ifdef CONFIG_SSP_ENG_DEBUG
int htou8(char input)
{
	int ret = 0;
	if ('0' <= input && input <= '9') {
		return ret = input - '0';
	} else if ('a' <= input && input <= 'f') {
		return ret = input - 'a' + 10;
	} else if ('A' <= input && input <= 'F') {
		return ret = input - 'A' + 10;
	} else {
		return 0;
	}
}

static ssize_t set_make_command(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data  = dev_get_drvdata(dev);
	int ret = 0;
	u8 cmd = 0, type = 0, subcmd = 0;
	char *send_buf = NULL;
	int send_buf_len = 0;

	char *input_str, *tmp, *dup_str = NULL;
	int index = 0, i = 0;

	ssp_infof("%s", buf);

	if (strlen(buf) == 0) {
		return size;
	}

	input_str = kzalloc(strlen(buf)+1, GFP_KERNEL);
	memcpy(input_str, buf, strlen(buf));
	dup_str = kstrdup(input_str, GFP_KERNEL);

	while (((tmp = strsep(&dup_str, " ")) != NULL)) {
		switch (index) {
		case 0 :
			if (kstrtou8(tmp, 10, &cmd) < 0) {
				ssp_errf("invalid cmd(%d)", cmd);
				goto exit;
			}
			break;
		case 1 :
			if (kstrtou8(tmp, 10, &type) < 0) {
				ssp_errf("invalid type(%d)", type);
				goto exit;
			}
			break;
		case 2 :
			if (kstrtou8(tmp, 10, &subcmd) < 0) {
				ssp_errf("invalid subcmd(%d)", subcmd);
				goto exit;
			}
			break;
		case 3 :
			if ((strlen(tmp) - 1) % 2 != 0) {
				ssp_errf("not match buf len(%d) != %d", (int)strlen(tmp), send_buf_len);
				goto exit;
			}
			send_buf_len = (strlen(tmp) - 1) / 2;
			send_buf = kzalloc(send_buf_len, GFP_KERNEL);
			for (i = 0; i < send_buf_len; i++) {
				send_buf[i] = (u8)((htou8(tmp[2 * i]) << 4) | htou8(tmp[2 * i + 1]));
			}
			break;
		default:
			goto exit;
			break;
		}
		index++;
	}

	if (index < 2) {
		ssp_errf("need more input");
		goto exit;
	}

	ret = ssp_send_command(data, cmd, type, subcmd, 0, send_buf, send_buf_len, NULL, NULL);

	if (ret < 0) {
		ssp_errf("ssp_send_command failed");
		return -EIO;
	}

exit:
	if (send_buf != NULL) {
		kfree(send_buf);
	}

	kfree(dup_str);
	kfree(input_str);

	return size;
}

static ssize_t register_rw_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	if (data->register_value[1] == 'r')      {
		return sprintf(buf, "sensor(%d) %c regi(0x%x) val(0x%x) ret(%d)\n", data->register_value[0],
		               data->register_value[1], data->register_value[2], data->register_value[3], data->register_value[4]);
	} else {
		if (data->register_value[4] == true) {
			return sprintf(buf, "sensor(%d) %c regi(0x%x) val(0x%x) SUCCESS\n", data->register_value[0],
			               data->register_value[1], data->register_value[2], data->register_value[3]);
		} else {
			return sprintf(buf, "sensor(%d) %c regi(0x%x) val(0x%x) FAIL\n", data->register_value[0],
			               data->register_value[1], data->register_value[2], data->register_value[3]);
		}
	}
}

static ssize_t register_rw_store(struct device *dev,
                                 struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int index = 0, ret = 0;
	u8 sensor_type, send_val[2];
	char rw_cmd;

	char input_str[20] = {0,};
	char *dup_str = NULL;
	char *tmp;
	memcpy(input_str, buf, strlen(buf));
	dup_str = kstrdup(input_str, GFP_KERNEL);

	while (((tmp = strsep(&dup_str, " ")) != NULL)) {
		switch (index) {
		case 0 :
			if (kstrtou8(tmp, 10, &sensor_type) < 0 || (sensor_type >= SENSOR_TYPE_MAX)) {
				ssp_errf("invalid type(%d)", sensor_type);
				goto exit;
			}
			break;
		case 1 :
			if (tmp[0] == 'r' || tmp[0] == 'w') {
				rw_cmd = tmp[0];
			} else {
				ssp_errf("invalid cmd(%c)", tmp[0]);
				goto exit;
			}
			break;
		case 2 :
		case 3 :
			if ((strlen(tmp) == 4) && tmp[0] != '0' && tmp[1] != 'x') {
				ssp_errf("invalid value(0xOO) %s", tmp);
				goto exit;
			}
			send_val[index - 2] = (u8)((htou8(tmp[2]) << 4) | htou8(tmp[3]));
			break;
		default:
			goto exit;
			break;
		}
		index++;
	}

	data->register_value[0] = sensor_type;
	data->register_value[1] = rw_cmd;
	data->register_value[2] = send_val[0];

	if (rw_cmd == 'r') {
		char *rec_buf = NULL;
		int rec_buf_len;
		ret = ssp_send_command(data, CMD_GETVALUE, sensor_type, SENSOR_REGISTER_RW, 1000, send_val, 1,
		                       &rec_buf, &rec_buf_len);
		data->register_value[4] = true;

		if (ret != SUCCESS) {
			data->register_value[4] = false;
			ssp_errf("ssp_send_command fail %d", ret);
			if (rec_buf != NULL) {
				kfree(rec_buf);
			}
			goto exit;
		}

		if (rec_buf == NULL) {
			ssp_errf("buffer is null");
			ret = -EINVAL;
			goto exit;
		}

		data->register_value[3] = rec_buf[0];

		kfree(rec_buf);
	} else { /* rw_cmd == w */
		ret = ssp_send_command(data, CMD_SETVALUE, sensor_type, SENSOR_REGISTER_RW, 0, send_val, 2, NULL,
		                       NULL);
		data->register_value[3] = send_val[1];
		data->register_value[4] = true;

		if (ret != SUCCESS) {
			data->register_value[4] = false;
			ssp_errf("ssp_send_command fail %d", ret);
			goto exit;
		}
	}

exit:
	kfree(dup_str);
	return size;
}

static DEVICE_ATTR(make_command, S_IWUSR | S_IWGRP,
                   NULL, set_make_command);

static DEVICE_ATTR(register_rw, S_IRUGO | S_IWUSR | S_IWGRP, register_rw_show, register_rw_store);
#endif  /* CONFIG_SSP_REGISTER_RW */

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
static DEVICE_ATTR(ssp_flush, S_IWUSR | S_IWGRP,
                   NULL, set_flush);
static DEVICE_ATTR(debug_enable, S_IRUGO | S_IWUSR | S_IWGRP,
                   show_debug_enable, set_debug_enable);
static DEVICE_ATTR(sensor_axis, S_IRUGO | S_IWUSR | S_IWGRP,
                   show_sensor_axis, set_sensor_axis);
static DEVICE_ATTR(sensor_state, S_IRUGO, show_sensor_state, NULL);

static DEVICE_ATTR(sensor_dump, S_IRUGO | S_IWUSR | S_IWGRP,     sensor_dump_show,
                   sensor_dump_store);

static DEVICE_ATTR(reset_info, S_IRUGO, show_reset_info, NULL);

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
	&dev_attr_ssp_flush,
	&dev_attr_debug_enable,
	&dev_attr_sensor_axis,
	&dev_attr_sensor_state,
	&dev_attr_sensor_dump,
#ifdef CONFIG_SSP_ENG_DEBUG
	&dev_attr_make_command,
	&dev_attr_register_rw,
#endif
	&dev_attr_reset_info,
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
	int ret = ERROR;
	int sensor_type, ms_delay;
	int timeout_ms = 0;
	u8 uBuf[8];

	sensor_type = (cmd & 0xFF);

	ssp_infof("cmd = 0x%x, sensor_type = %d", cmd, sensor_type);

	if ((cmd >> 8 & 0xFF) != BATCH_IOCTL_MAGIC) {
		ssp_err("Invalid BATCH CMD %x", cmd);
		return -EINVAL;
	}

	if (sensor_type >= SENSOR_TYPE_MAX) {
		ssp_err("Invalid sensor_type %d", sensor_type);
		return -EINVAL;
	}

	while (retries--) {
		ret = copy_from_user(&batch, argp, sizeof(batch));
		if (likely(!ret)) {
			break;
		}
	}
	if (unlikely(ret)) {
		ssp_err("batch ioctl err(%d)", ret);
		return -EINVAL;
	}

	ms_delay = get_msdelay(batch.delay);
	timeout_ms = div_s64(batch.timeout, NSEC_PER_MSEC);
	memcpy(&uBuf[0], &ms_delay, 4);
	memcpy(&uBuf[4], &timeout_ms, 4);

	if ((data->batch_max_latency[sensor_type] != timeout_ms || data->delay[sensor_type] != batch.delay)
	    && data->aiCheckStatus[sensor_type] == RUNNING_SENSOR_STATE) {
		ret = make_command(data, CHANGE_DELAY, sensor_type, uBuf, 8);
	}

	data->batch_opt[sensor_type] = 0;
	data->batch_max_latency[sensor_type] = timeout_ms;
	data->delay[sensor_type] =  batch.delay;

	ssp_info("batch %d: delay %lld, timeout %lld, flag %d, ret %d",
	         sensor_type, batch.delay, batch.timeout, batch.flag, ret);

	if (ret < 0) {
		return -EINVAL;
	} else {
		return 0;
	}
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
	if (misc_register(&data->batch_io_device)) {
		goto err_batch_io_dev;
	}
	
	initialize_mcu_factorytest(data);
#ifdef CONFIG_SENSORS_SSP_ACCELOMETER
	initialize_accel_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
	initialize_gyro_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_PROXIMITY
	initialize_prox_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_LIGHT
	initialize_light_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_BAROMETER
	initialize_barometer_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_MAGNETIC
	initialize_magnetic_factorytest(data);
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
	remove_mcu_factorytest(data);
#ifdef CONFIG_SENSORS_SSP_ACCELOMETER
	remove_accel_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
	remove_gyro_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_PROXIMITY
	remove_prox_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_LIGHT
	remove_light_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_BAROMETER
	remove_barometer_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_MAGNETIC
	remove_magnetic_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_MOBEAM
	remove_mobeam(data);
#endif
}
