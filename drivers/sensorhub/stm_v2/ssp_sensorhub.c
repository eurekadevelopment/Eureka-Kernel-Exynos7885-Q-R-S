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

#include "ssp_sensorhub.h"

void ssp_sensorhub_log(const char *func_name,
                       const char *data, int length)
{
	char buf[6];
	char *log_str;
	int log_size;
	int i;

	if (likely(length <= BIG_DATA_SIZE)) {
		log_size = length;
	} else {
		log_size = PRINT_TRUNCATE * 2 + 1;
	}

	log_size = sizeof(buf) * log_size + 1;
	log_str = kzalloc(log_size, GFP_ATOMIC);
	if (unlikely(!log_str)) {
		ssp_errf("allocate memory for data log err");
		return;
	}

	for (i = 0; i < length; i++) {
		if (length < BIG_DATA_SIZE ||
		    i < PRINT_TRUNCATE || i >= length - PRINT_TRUNCATE) {
			snprintf(buf, sizeof(buf), "0x%x", (signed char)data[i]);
			strlcat(log_str, buf, log_size);

			if (i < length - 1) {
				strlcat(log_str, ", ", log_size);
			}
		}
		if (length > BIG_DATA_SIZE && i == PRINT_TRUNCATE) {
			strlcat(log_str, "..., ", log_size);
		}
	}

	ssp_info("%s(%d): %s", func_name, length, log_str);
	kfree(log_str);
}

static int ssp_scontext_send_cmd(struct ssp_data *data,
                                 const char *buf, int count)
{
	int ret = 0;

	if (buf[2] < SCONTEXT_AP_STATUS_WAKEUP ||
	    buf[2] >= SCONTEXT_AP_STATUS_CALL_ACTIVE) {
		ssp_errf("INST_LIB_NOTI err(%d)", buf[2]);
		return -EINVAL;
	}

	ret = ssp_send_status(data, buf[2]);

	if (buf[2] == SCONTEXT_AP_STATUS_WAKEUP ||
	    buf[2] == SCONTEXT_AP_STATUS_SLEEP) {
		data->uLastAPState = buf[2];
	}

	if (buf[2] == SCONTEXT_AP_STATUS_SUSPEND ||
	    buf[2] == SCONTEXT_AP_STATUS_RESUME) {
		data->uLastResumeState = buf[2];
	}

	return ret;
}

int convert_scontext_putvalue_subcmd(int subcmd)
{
	int ret = -1;
	switch (subcmd) {
	case SCONTEXT_VALUE_CURRENTSYSTEMTIME :
		ret = CURRENT_SYSTEM_TIME;
		break;
	case SCONTEXT_VALUE_PEDOMETER_USERHEIGHT :
		ret = PEDOMETER_USERHEIGHT;
		break;
	case SCONTEXT_VALUE_PEDOMETER_USERWEIGHT:
		ret = PEDOMETER_USERWEIGHT;
		break;
	case SCONTEXT_VALUE_PEDOMETER_USERGENDER:
		ret = PEDOMETER_USERGENDER;
		break;
	case SCONTEXT_VALUE_PEDOMETER_INFOUPDATETIME:
		ret = PEDOMETER_INFOUPDATETIME;
		break;
	default:
		ret = ERROR;
	}

	return ret;
}

int convert_scontext_getvalue_subcmd(int subcmd)
{
	int ret = -1;
	switch (subcmd) {
	case SCONTEXT_VALUE_CURRENTSTATUS :
		ret = LIBRARY_CURRENTSTATUS;
		break;
	case SCONTEXT_VALUE_CURRENTSTATUS_BATCH :
		ret = LIBRARY_CURRENTSTATUS_BATCH;
		break;
	case SCONTEXT_VALUE_VERSIONINFO:
		ret = LIBRARY_VERSIONINFO;
		break;
	default:
		ret = ERROR;
	}

	return ret;
}

static int ssp_scontext_send_instruction(struct ssp_data *data,
                                         const char *buf, int count)
{
	char command, type, sub_cmd = 0;
	int ret;
	char *buffer = (char *)(buf + 2);
	int length = count - 2;

	if (buf[0] == SCONTEXT_INST_LIBRARY_REMOVE) {
		command = CMD_REMOVE;
		type = buf[1] + SS_SENSOR_TYPE_BASE;
		ssp_infof("REMOVE LIB, type %d", type);
	} else if (buf[0] == SCONTEXT_INST_LIBRARY_ADD) {
		command = CMD_ADD;
		type = buf[1] + SS_SENSOR_TYPE_BASE;
		ssp_infof("ADD LIB, type %d", type);
	} else if (buf[0] == SCONTEXT_INST_LIB_SET_DATA) {
		command = CMD_SETVALUE;
		if (buf[1] != SCONTEXT_VALUE_LIBRARY_DATA) {
			type = TYPE_MCU;
			ret = convert_scontext_putvalue_subcmd(buf[1]);
			if(ret == ERROR) {
				ssp_errf("setvalue subcmd 0x%x is not supported", buf[1]);
				return ERROR;
			} else {
				sub_cmd = ret;
			}
		} else {
			type = buf[2] + SS_SENSOR_TYPE_BASE;
			sub_cmd = LIBRARY_DATA;
			length = count - 3;
			if (length > 0) {
				buffer = (char *)(buf + 3);
			} else {
				buffer = NULL;
			}
		}
	} else if (buf[0] == SCONTEXT_INST_LIB_GET_DATA) {
		command = CMD_GETVALUE;
		type = buf[1] + SS_SENSOR_TYPE_BASE;
		ret =  convert_scontext_getvalue_subcmd(buf[2]);
		if(ret == ERROR) {
			ssp_errf("getvalue subcmd 0x%x is not supported", buf[2]);
			return ERROR;

		} else {
			sub_cmd = ret;
		}
		
		length = count - 3;
		if (length > 0) {
			buffer = (char *)(buf + 3);
		} else {
			buffer = NULL;
		}
	} else {
		ssp_errf("0x%x is not supported", buf[0]);
		return ERROR;
	}

	return ssp_send_command(data, command, type, sub_cmd, 0,
	                        buffer, length, NULL, NULL);
}

#ifdef SENSOR_TYPE_SCONTEXT
static ssize_t ssp_scontext_write(struct file *file, const char __user *buf,
                                  size_t count, loff_t *pos)
{
	struct ssp_data *data = container_of(file->private_data, struct ssp_data, scontext_device);
	int ret = 0;
	char *buffer;

	if (data->is_ssp_shutdown) {
		ssp_errf("stop sending library data(shutdown)");
		return -EIO;
	}

	if (unlikely(count < 2)) {
		ssp_errf("library data length err(%d)", (int)count);
		return -EINVAL;
	}

	buffer = kzalloc(count * sizeof(char), GFP_KERNEL);

	ret = copy_from_user(buffer, buf, count);
	if (unlikely(ret)) {
		ssp_errf("memcpy for kernel buffer err");
		ret = -EFAULT;
		goto exit;
	}

	ssp_sensorhub_log(__func__, buffer, count);

	if (buffer[0] == SCONTEXT_INST_LIB_NOTI) {
		ret = ssp_scontext_send_cmd(data, buffer, count);
	} else {
		ret = ssp_scontext_send_instruction(data, buffer, count);
	}

	if (unlikely(ret < 0)) {
		ssp_errf("send library data err(%d)", ret);
		if (ret == ERROR) {
			ret = -EIO;
		}

		else if (ret == FAIL) {
			ret = -EAGAIN;
		}

		goto exit;
	}

	ret = count;

exit:
	kfree(buffer);
	return ret;
}

static struct file_operations ssp_scontext_fops = {
	.owner = THIS_MODULE,
	.open = nonseekable_open,
	.write = ssp_scontext_write,
};

int ssp_scontext_initialize(struct ssp_data *data)
{
	int ret;
	ssp_dbgf("----------");

	/* register scontext misc device */
	data->scontext_device.minor = MISC_DYNAMIC_MINOR;
	data->scontext_device.name = "ssp_sensorhub";
	data->scontext_device.fops = &ssp_scontext_fops;

	ret = misc_register(&data->scontext_device);
	if (ret < 0) {
		ssp_errf("register scontext misc device err(%d)", ret);
	}

	return ret;
}

void ssp_scontext_remove(struct ssp_data *data)
{
	ssp_scontext_fops.write = NULL;
	misc_deregister(&data->scontext_device);
}

#else
static ssize_t ssp_sensorhub_write(struct file *file, const char __user *buf,
                                   size_t count, loff_t *pos)
{
	struct ssp_sensorhub_data *hub_data
	        = container_of(file->private_data,
	                       struct ssp_sensorhub_data, sensorhub_device);
	int ret = 0;
	char *buffer;

	if (hub_data->ssp_data->is_ssp_shutdown) {
		ssp_errf("stop sending library data(shutdown)");
		return -EIO;
	}

	if (unlikely(count < 2)) {
		ssp_errf("library data length err(%d)", (int)count);
		return -EINVAL;
	}

	buffer = kzalloc(count * sizeof(char), GFP_KERNEL);

	ret = copy_from_user(buffer, buf, count);
	if (unlikely(ret)) {
		ssp_errf("memcpy for kernel buffer err");
		ret = -EFAULT;
		goto exit;
	}

	ssp_sensorhub_log(__func__, buffer, count);

	if (buffer[0] == SCONTEXT_INST_LIB_NOTI) {
		ret = ssp_scontext_send_cmd(hub_data->ssp_data, buffer, count);
	} else {
		ret = ssp_scontext_send_instruction(hub_data->ssp_data, buffer, count);
	}

	if (unlikely(ret < 0)) {
		ssp_errf("send library data err(%d)", ret);
		/* i2c transfer fail */
		if (ret == ERROR) {
			ret = -EIO;
		}
		/* i2c transfer done but no ack from MCU */
		else if (ret == FAIL) {
			ret = -EAGAIN;
		}

		goto exit;
	}

	ret = count;

exit:
	kfree(buffer);
	return ret;
}


static ssize_t ssp_sensorhub_read(struct file *file, char __user *buf,
                                  size_t count, loff_t *pos)
{
	struct ssp_sensorhub_data *hub_data
	        = container_of(file->private_data,
	                       struct ssp_sensorhub_data, sensorhub_device);
	struct sensorhub_event *event;
	int retries = MAX_DATA_COPY_TRY;
	int length = 0;
	int ret = 0;

	spin_lock_bh(&hub_data->sensorhub_lock);
	if (unlikely(kfifo_is_empty(&hub_data->fifo))) {
		ssp_infof("no library data");
		goto err;
	}

	/* first in first out */
	ret = kfifo_out_peek(&hub_data->fifo, &event, sizeof(void *));
	if (unlikely(!ret)) {
		ssp_errf("kfifo out peek err(%d)", ret);
		ret = EIO;
		goto err;
	}

	length = event->library_length;

	while (retries--) {
		ret = copy_to_user(buf,
		                   event->library_data, event->library_length);
		if (likely(!ret)) {
			break;
		}
	}
	if (unlikely(ret)) {
		ssp_errf("read library data err(%d)", ret);
		goto err;
	}

	ssp_sensorhub_log(__func__,
	                  event->library_data, event->library_length);

	/* remove first event from the list */
	ret = kfifo_out(&hub_data->fifo, &event, sizeof(void *));
	if (unlikely(ret != sizeof(void *))) {
		ssp_errf("kfifo out err(%d)", ret);
		ret = EIO;
		goto err;
	}

	complete(&hub_data->read_done);
	spin_unlock_bh(&hub_data->sensorhub_lock);

	return length;

err:
	spin_unlock_bh(&hub_data->sensorhub_lock);
	return ret ? -ret : 0;
}

static struct file_operations ssp_sensorhub_fops = {
	.owner = THIS_MODULE,
	.open = nonseekable_open,
	.write = ssp_sensorhub_write,
	.read = ssp_sensorhub_read,
};

void ssp_sensorhub_report_notice(struct ssp_data *ssp_data, char notice)
{
	struct ssp_sensorhub_data *hub_data = ssp_data->hub_data;

	if (notice == SCONTEXT_AP_STATUS_RESET) {
		if (ssp_data->is_reset_from_sysfs == true) {
			input_report_rel(hub_data->sensorhub_input_dev, NOTICE,
			                 RESET_REASON_SYSFS_REQUEST);
			ssp_data->is_reset_from_sysfs = false;
			ssp_data->reset_type = RESET_KERNEL_SYSFS;
		} else if (ssp_data->is_reset_from_kernel == true) {
			input_report_rel(hub_data->sensorhub_input_dev, NOTICE,
			                 RESET_REASON_KERNEL_RESET);
			ssp_data->is_reset_from_kernel = false;
		} else {
			input_report_rel(hub_data->sensorhub_input_dev, NOTICE,
			                 RESET_REASON_MCU_CRASHED);
			ssp_data->reset_type = RESET_MCU_CRASHED;
		}
	} else {
		input_report_rel(hub_data->sensorhub_input_dev, NOTICE, notice);
	}
	input_sync(hub_data->sensorhub_input_dev);

	if (notice == SCONTEXT_AP_STATUS_WAKEUP) {
		ssp_infof("wake up");
	} else if (notice == SCONTEXT_AP_STATUS_SLEEP) {
		ssp_infof("sleep");
	} else if (notice == SCONTEXT_AP_STATUS_RESET) {
		ssp_infof("reset");
	} else {
		ssp_errf("invalid notice(0x%x)", notice);
	}
}

static void ssp_sensorhub_report_library(struct ssp_sensorhub_data *hub_data)
{
	input_report_rel(hub_data->sensorhub_input_dev, DATA, DATA);
	input_sync(hub_data->sensorhub_input_dev);
	wake_lock_timeout(&hub_data->sensorhub_wake_lock, WAKE_LOCK_TIMEOUT);
}

static int ssp_sensorhub_list(struct ssp_sensorhub_data *hub_data,
                              char *dataframe, int length)
{
	struct sensorhub_event *event;
	int ret = 0;

	if (unlikely(length <= 0 || length >= PAGE_SIZE)) {
		ssp_errf("library length err(%d)", length);
		return -EINVAL;
	}

	ssp_sensorhub_log(__func__, dataframe, length);

	/* overwrite new event if list is full */
	if (unlikely(kfifo_is_full(&hub_data->fifo))) {
		ret = kfifo_out(&hub_data->fifo, &event, sizeof(void *));
		if (unlikely(ret != sizeof(void *))) {
			ssp_errf("kfifo out err(%d)", ret);
			return -EIO;
		}
		ssp_infof("overwrite event");
	}

	/* allocate memory for new event */
	kfree(hub_data->events[hub_data->event_number].library_data);
	hub_data->events[hub_data->event_number].library_data
	        = kzalloc(length * sizeof(char), GFP_ATOMIC);
	if (unlikely(!hub_data->events[hub_data->event_number].library_data)) {
		ssp_errf("allocate memory for library err");
		return -ENOMEM;
	}

	/* copy new event into memory */
	memcpy(hub_data->events[hub_data->event_number].library_data,
	       dataframe, length);
	hub_data->events[hub_data->event_number].library_length = length;

	/* add new event into the end of list */
	event = &hub_data->events[hub_data->event_number];
	ret = kfifo_in(&hub_data->fifo, &event, sizeof(void *));
	if (unlikely(ret != sizeof(void *))) {
		ssp_errf("kfifo in err(%d)", ret);
		return -EIO;
	}

	/* not to overflow max list capacity */
	if (hub_data->event_number++ >= LIST_SIZE - 1) {
		hub_data->event_number = 0;
	}

	return kfifo_len(&hub_data->fifo) / sizeof(void *);
}

int ssp_sensorhub_handle_data(struct ssp_data *ssp_data, char *dataframe,
                              int start, int end)
{
	struct ssp_sensorhub_data *hub_data = ssp_data->hub_data;
	int ret = 0;

	/* add new sensorhub event into list */
	spin_lock_bh(&hub_data->sensorhub_lock);
	ret = ssp_sensorhub_list(hub_data, dataframe + start, end - start);
	spin_unlock_bh(&hub_data->sensorhub_lock);

	if (ret < 0) {
		ssp_errf("sensorhub list err(%d)", ret);
	} else {
		wake_up(&hub_data->sensorhub_wq);
	}

	return ret;
}

static int ssp_sensorhub_thread(void *arg)
{
	struct ssp_sensorhub_data *hub_data = (struct ssp_sensorhub_data *)arg;
	int ret = 0;

	while (likely(!kthread_should_stop())) {
		/* run thread if list is not empty */
		wait_event_interruptible(hub_data->sensorhub_wq,
		                         kthread_should_stop() ||
		                         !kfifo_is_empty(&hub_data->fifo));

		/* exit thread if kthread should stop */
		if (unlikely(kthread_should_stop())) {
			ssp_infof("kthread_stop()");
			break;
		}

		if (likely(!kfifo_is_empty(&hub_data->fifo))) {
			/* report sensorhub event to user */
			ssp_sensorhub_report_library(hub_data);
			/* wait until transfer finished */
			ret = wait_for_completion_timeout(
			              &hub_data->read_done, COMPLETION_TIMEOUT);
			if (unlikely(!ret)) {
				ssp_errf("wait for read timed out");
			} else if (unlikely(ret < 0)) {
				ssp_errf("read completion err(%d)", ret);
			}
		}

	}

	return 0;
}

int ssp_sensorhub_initialize(struct ssp_data *ssp_data)
{
	struct ssp_sensorhub_data *hub_data;
	int ret;

	ssp_dbgf("----------");

	/* allocate memory for sensorhub data */
	hub_data = kzalloc(sizeof(*hub_data), GFP_KERNEL);

	hub_data->ssp_data = ssp_data;
	ssp_data->hub_data = hub_data;

	/* init wakelock, list, waitqueue, completion and spinlock */
	wake_lock_init(&hub_data->sensorhub_wake_lock, WAKE_LOCK_SUSPEND,
	               "ssp_sensorhub_wake_lock");
	init_waitqueue_head(&hub_data->sensorhub_wq);
	init_completion(&hub_data->read_done);
	spin_lock_init(&hub_data->sensorhub_lock);

	/* allocate sensorhub input device */
	hub_data->sensorhub_input_dev = input_allocate_device();
	if (!hub_data->sensorhub_input_dev) {
		ssp_errf("allocate sensorhub input device err");
		ret = -ENOMEM;
		goto err_input_allocate_device_sensorhub;
	}

	/* set sensorhub input device */
	input_set_drvdata(hub_data->sensorhub_input_dev, hub_data);
	hub_data->sensorhub_input_dev->name = "ssp_context";
	input_set_capability(hub_data->sensorhub_input_dev, EV_REL, DATA);
	input_set_capability(hub_data->sensorhub_input_dev, EV_REL, NOTICE);

	/* register sensorhub input device */
	ret = input_register_device(hub_data->sensorhub_input_dev);
	if (ret < 0) {
		ssp_errf("register sensorhub input device err(%d)", ret);
		input_free_device(hub_data->sensorhub_input_dev);
		goto err_input_register_device_sensorhub;
	}

	/* register sensorhub misc device */
	hub_data->sensorhub_device.minor = MISC_DYNAMIC_MINOR;
	hub_data->sensorhub_device.name = "ssp_sensorhub";
	hub_data->sensorhub_device.fops = &ssp_sensorhub_fops;

	ret = misc_register(&hub_data->sensorhub_device);
	if (ret < 0) {
		ssp_errf("register sensorhub misc device err(%d)", ret);
		goto err_misc_register;
	}

	/* allocate fifo */
	ret = kfifo_alloc(&hub_data->fifo,
	                  sizeof(void *) * LIST_SIZE, GFP_KERNEL);
	if (ret) {
		ssp_errf("kfifo allocate err(%d)", ret);
		goto err_kfifo_alloc;
	}

	/* create and run sensorhub thread */
	hub_data->sensorhub_task = kthread_run(ssp_sensorhub_thread,
	                                       (void *)hub_data, "ssp_sensorhub_thread");
	if (IS_ERR(hub_data->sensorhub_task)) {
		ret = PTR_ERR(hub_data->sensorhub_task);
		goto err_kthread_run;
	}

	return 0;

err_kthread_run:
	kfifo_free(&hub_data->fifo);
err_kfifo_alloc:
	misc_deregister(&hub_data->sensorhub_device);
err_misc_register:
	input_unregister_device(hub_data->sensorhub_input_dev);
err_input_register_device_sensorhub:
err_input_allocate_device_sensorhub:
	complete_all(&hub_data->read_done);
	wake_lock_destroy(&hub_data->sensorhub_wake_lock);
	kfree(hub_data);

	return ret;
}

void ssp_sensorhub_remove(struct ssp_data *ssp_data)
{
	struct ssp_sensorhub_data *hub_data = ssp_data->hub_data;

	ssp_sensorhub_fops.write = NULL;
	ssp_sensorhub_fops.read = NULL;
	ssp_sensorhub_fops.unlocked_ioctl = NULL;

	kthread_stop(hub_data->sensorhub_task);
	kfifo_free(&hub_data->fifo);
	misc_deregister(&hub_data->sensorhub_device);
	input_unregister_device(hub_data->sensorhub_input_dev);
	complete_all(&hub_data->read_done);
	wake_lock_destroy(&hub_data->sensorhub_wake_lock);
	kfree(hub_data);
}
#endif
MODULE_DESCRIPTION("Seamless Sensor Platform(SSP) sensorhub driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
