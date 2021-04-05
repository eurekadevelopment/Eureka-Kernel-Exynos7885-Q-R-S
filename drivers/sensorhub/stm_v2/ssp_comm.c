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

#include "ssp_comm.h"

#define SSP_CMD_SIZE 64
#define SSP_MSG_HEADER_SIZE 5
static char ssp_cmd_data[SSP_CMD_SIZE];


/*
        return sub_command
*/
int convert_ap_status(int command)
{
	int ret = -1;
	switch (command) {
	case SCONTEXT_AP_STATUS_SHUTDOWN :
		ret = AP_SHUTDOWN;
		break;
	case SCONTEXT_AP_STATUS_WAKEUP :
		ret = LCD_ON;
		break;
	case SCONTEXT_AP_STATUS_SLEEP :
		ret = LCD_OFF;
		break;
	case SCONTEXT_AP_STATUS_RESUME :
		ret = AP_RESUME;
		break;
	case SCONTEXT_AP_STATUS_SUSPEND :
		ret = AP_SUSPEND;
		break;
#if 0
	case SCONTEXT_AP_STATUS_RESET :
		ret = AP_SHUTDOWN;
		break;
#endif
	case SCONTEXT_AP_STATUS_POW_CONNECTED :
		ret = POW_CONNECTED;
		break;
	case SCONTEXT_AP_STATUS_POW_DISCONNECTED :
		ret = POW_DISCONNECTED;
		break;
	case SCONTEXT_AP_STATUS_CALL_IDLE :
		ret = CALL_IDLE;
		break;
	case SCONTEXT_AP_STATUS_CALL_ACTIVE :
		ret = CALL_ACTIVE;
		break;
	}

	return ret;
}

static void clean_msg(struct ssp_msg *msg)
{
	if (msg->buffer != NULL) {
		ssp_errf("kfree(msg->buffer)");
		kfree(msg->buffer);
	}

	ssp_errf("kfree(msg)");
	kfree(msg);
}

static int do_transfer(struct ssp_data *data, struct ssp_msg *msg, int timeout)
{
	int status = 0;
	int iDelaycnt = 0;
	bool ssp_down = false;
	int ret = 0;

	mutex_lock(&data->comm_mutex);

	memcpy(ssp_cmd_data, msg, SSP_MSG_HEADER_SIZE);
	if (msg->length > 0) {
		memcpy(&ssp_cmd_data[SSP_MSG_HEADER_SIZE], msg->buffer, msg->length);
	} else if (msg->length > (SSP_CMD_SIZE - SSP_MSG_HEADER_SIZE)) {
		ssp_errf("command size over !");
		mutex_unlock(&data->comm_mutex);
		return false;
	}

	gpio_set_value_cansleep(data->ap_int, 0);
	while (gpio_get_value_cansleep(data->mcu_int2)) {
		usleep_range(2900, 3000);
		ssp_down = data->is_ssp_shutdown;
		if (ssp_down || iDelaycnt++ > 500) {
			ssp_errf("exit1 - Time out!!");
			gpio_set_value_cansleep(data->ap_int, 1);
			status = -1;
			goto exit;
		}
	}

	status = spi_write(data->spi, ssp_cmd_data, SSP_CMD_SIZE) >= 0;

	if (status == 0) {
		ssp_errf("spi_write fail!!");
		gpio_set_value_cansleep(data->ap_int, 1);
		status = -1;
		goto exit;
	}

	if (msg->done != NULL) {
		mutex_lock(&data->pending_mutex);
		list_add_tail(&msg->list, &data->pending_list);
		mutex_unlock(&data->pending_mutex);
	}

	iDelaycnt = 0;
	gpio_set_value_cansleep(data->ap_int, 1);
	while (!gpio_get_value_cansleep(data->mcu_int2)) {
		usleep_range(2900, 3000);
		ssp_down = data->is_ssp_shutdown;
		if (ssp_down || iDelaycnt++ > 500) {
			ssp_errf("exit2 - Time out!!");
			status = -2;
			goto exit;
		}
	}

exit:
	memset(ssp_cmd_data, 0, SSP_CMD_SIZE);

	mutex_unlock(&data->comm_mutex);

	if (ssp_down) {
		ssp_errf("ssp down");
	}

	if (status == -1) {
		data->cnt_timeout += ssp_down ? 0 : 1;
		ssp_errf("status is -1, cnt_timeout %d, ssp_down %d !!",
		         data->cnt_timeout, ssp_down);
		return status;
	}

	if ((status == 1) && (msg->done != NULL) && (timeout > 0)) {
		ret = wait_for_completion_timeout(msg->done,
		                                  msecs_to_jiffies(timeout));

		if (msg->clean_pending_list_flag) {
			status = -2;
			msg->clean_pending_list_flag = 0;
			ssp_errf("communication fail so recovery_mcu func call status %d", status);
			return status;
		}

		/* when timeout is happened */
		if (!ret) {
			status = -2;
			msg->done = NULL;
			list_del(&msg->list);
			ssp_errf("mcu_int1 level: %d",
			         gpio_get_value(data->mcu_int1));
		}
	}

	if (status == -2) {
		data->cnt_timeout += ssp_down ? 0 : 1;
		if (msg->done != NULL) {
			list_del(&msg->list);
		}

		ssp_errf("status is -2, cnt_timeout %d, ssp_down %d !!",
		         data->cnt_timeout, ssp_down);
		return status;
	}

	return status;
}

int select_irq_msg(struct ssp_data *data)
{
	struct ssp_msg *msg, *n;
	bool found = false;
	u16 chLength = 0;
	u8 msg_cmd = 0, msg_subcmd = 0, msg_type = 0;
	int ret = 0;
	char *buffer;
	char chTempBuf[SSP_MSG_HEADER_SIZE] = { -1 };

	ret = spi_read(data->spi, chTempBuf, sizeof(chTempBuf));
	if (ret < 0) {
		ssp_errf("spi_read fail, ret = %d", ret);
		return ret;
	}

	memcpy(&msg_cmd, &chTempBuf[0], 1);
	memcpy(&msg_type, &chTempBuf[1], 1);
	memcpy(&msg_subcmd, &chTempBuf[2], 1);
	memcpy(&chLength, &chTempBuf[3], 2);

	if (chLength == 0) {
		ssp_errf("lengh is zero %d %d %d", msg_cmd, msg_type, msg_subcmd);
	}

	//ssp_errf("cmd %d, type %d, sub_cmd %d, length %d", msg_cmd, msg_type, msg_subcmd, chLength);

	if (msg_cmd <= CMD_GETVALUE) {
		mutex_lock(&data->pending_mutex);
		if (!list_empty(&data->pending_list)) {
			list_for_each_entry_safe(msg, n,
			                         &data->pending_list, list) {

				if ((msg->cmd == msg_cmd) && (msg->type == msg_type) &&
				    (msg->subcmd == msg_subcmd)) {
					list_del(&msg->list);
					found = true;
					break;
				}
			}

			if (!found) {
				ssp_errf("%d %d %d - Not match error",  msg_cmd, msg_type, msg_subcmd);
				goto exit;
			}

			if (msg_cmd == CMD_GETVALUE) {
				msg->length = chLength;
				if (msg->length != 0) {
					if (msg->buffer != NULL) {
						kfree(msg->buffer);
					}
					msg->buffer = kzalloc(msg->length, GFP_KERNEL);
					ret = spi_read(data->spi, msg->buffer, msg->length);
				} else {
					msg->res = 0;
				}
			} else {
				/* response value : true/false */
				ret = spi_read(data->spi, &(msg->res), sizeof(msg->res));
			}

			if (msg->done != NULL && !completion_done(msg->done)) {
				complete(msg->done);
			}

		} else {
			ssp_errf("List empty error(%d %d %d)", msg_cmd, msg_type, msg_subcmd);
		}
exit:
		mutex_unlock(&data->pending_mutex);

	} else if (msg_cmd == CMD_REPORT) {
		if (chLength > 0) {
			buffer = kzalloc(chLength, GFP_KERNEL);

			ret = spi_read(data->spi, buffer, chLength);
			if (ret < 0) {
				ssp_errf("spi_read fail");
			} else {
				parse_dataframe(data, buffer, chLength);
			}

			kfree(buffer);
		} else {
			ssp_errf("length is 0");
			ret = -EINVAL;
		}
	} else {
		ssp_errf("msg_cmd does not define. cmd is %d", msg_cmd);
	}

	if (ret < 0) {
		ssp_errf("SSP MSG error %d", ret);
		return ret;
	}

	return SUCCESS;
}

void clean_pending_list(struct ssp_data *data)
{
	struct ssp_msg *msg, *n;

	ssp_infof(" IN");

	mutex_lock(&data->pending_mutex);
	list_for_each_entry_safe(msg, n, &data->pending_list, list) {

		list_del(&msg->list);
		if (msg->done != NULL && !completion_done(msg->done)) {
			msg->clean_pending_list_flag = 1;
			complete(msg->done);
		}
	}
	mutex_unlock(&data->pending_mutex);
	ssp_infof(" OUT");

}

int ssp_send_command(struct ssp_data *data, u8 cmd, u8 type, u8 subcmd,
                     int timeout, char *send_buf, int send_buf_len, char **receive_buf,
                     int *receive_buf_len)
{
	int status = 0;
	struct ssp_msg *msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	DECLARE_COMPLETION_ONSTACK(done);

	msg->cmd = cmd;
	msg->type = type;
	msg->subcmd = subcmd;
	msg->length = send_buf_len;

	if (timeout > 0) {
		if (send_buf != NULL) {
			msg->buffer = kzalloc(send_buf_len, GFP_KERNEL);
			memcpy(msg->buffer, send_buf, send_buf_len);
		} else {
			msg->buffer = send_buf;
		}
		msg->done = &done;
	} else {
		msg->buffer = kzalloc(send_buf_len, GFP_KERNEL);
		memcpy(msg->buffer, send_buf, send_buf_len);
		msg->done = NULL;
	}

	ssp_infof("cmd %d type %d subcmd %d send_buf_len %d timeout %d", cmd, type,
	          subcmd, send_buf_len, timeout);

	if (do_transfer(data, msg, timeout) != 1) {
		status = ERROR;
	}

	ssp_infof("After do_transfer timeout %d, status %d", timeout, status);

	//mutex_lock(&data->cmd_mutex);
	if (((msg->cmd == CMD_GETVALUE) && (receive_buf != NULL) &&
	     ((receive_buf_len != NULL) && (msg->length != 0))) &&
	    (status != ERROR)) {
		if (timeout > 0) {
			*receive_buf = kzalloc(msg->length, GFP_KERNEL);
			*receive_buf_len = msg->length;
			memcpy(*receive_buf, msg->buffer, msg->length);
			ssp_errf("receive_buf length = %d", msg->length);
		} else {
			ssp_errf("CMD_GETVALUE zero timeout");
			//mutex_unlock(&data->cmd_mutex);
			return -EINVAL;
		}
	}

	clean_msg(msg);
	//mutex_unlock(&data->cmd_mutex);
	return status;
}



int ssp_send_status(struct ssp_data *data, char command)
{
	int ret = 0;

	ret = ssp_send_command(data, CMD_SETVALUE, TYPE_MCU, convert_ap_status(command),
	                       0, NULL, 0, NULL, NULL);
	if (ret != SUCCESS) {
		ssp_errf("command 0x%x failed %d", command, ret);
		return ERROR;
	}

	ssp_infof("command 0x%x", command);

	return SUCCESS;
}



int make_command(struct ssp_data *data, u8 uInst,
                 u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char command, sub_cmd = 0;
	int ret = 0;
	char *buffer = NULL;

	if (data->fw_dl_state == FW_DL_STATE_DOWNLOADING) {
		ssp_errf("Skip make command! DL state = %d", data->fw_dl_state);
		return SUCCESS;
	} else if ((!(data->uSensorState & (1ULL << uSensorType)))
	           && (uInst <= CHANGE_DELAY)) {
		ssp_errf("Bypass make command Skip! - %u", uSensorType);
		return FAIL;
	}

	switch (uInst) {
	case REMOVE_SENSOR:
		command = CMD_REMOVE;
		break;
	case ADD_SENSOR:
		command = CMD_ADD;
		data->latest_timestamp[uSensorType] = get_current_timestamp();
		break;
	case CHANGE_DELAY:
		command = CMD_CHANGERATE;
		data->latest_timestamp[uSensorType] = get_current_timestamp();
		break;
	case GO_SLEEP:
		command = CMD_SETVALUE;
		sub_cmd = SCONTEXT_AP_STATUS_SLEEP;
		data->uLastAPState = SCONTEXT_AP_STATUS_SLEEP;
		break;
	case REMOVE_LIBRARY:
		command = CMD_REMOVE;
		break;
	case ADD_LIBRARY:
		command = CMD_ADD;
		break;
	default:
		command = uInst;
		break;
	}

	buffer = kzalloc(uLength, GFP_KERNEL);
	memcpy(buffer, uSendBuf, uLength);

	ret = ssp_send_command(data, command, uSensorType, sub_cmd, 0, buffer, uLength,
	                       NULL, NULL);

	if (ret != SUCCESS) {
		ssp_errf("ssp_send_command Fail %d", ret);
		goto exit;
	}

exit:
	if (buffer != NULL) {
		kfree(buffer);
	}
	return ret;
}

int make_command_sync(struct ssp_data *data, u8 uInst,
                      u8 uSensorType, u8 *uSendBuf, u16 uLength)
{
	char command, sub_cmd = 0;
	int ret = 0;
	char *buffer = NULL;

	if (data->fw_dl_state == FW_DL_STATE_DOWNLOADING) {
		ssp_errf("Skip make command sync! DL state = %d", data->fw_dl_state);
		return SUCCESS;
	} else if ((!(data->uSensorState & (1ULL << uSensorType)))
	           && (uInst <= CHANGE_DELAY)) {
		ssp_errf("Bypass make command Skip! - %u", uSensorType);
		return FAIL;
	}

	switch (uInst) {
	case REMOVE_SENSOR:
		command = CMD_REMOVE;
		break;
	case ADD_SENSOR:
		command = CMD_ADD;
		data->latest_timestamp[uSensorType] = get_current_timestamp();
		break;
	case CHANGE_DELAY:
		command = CMD_CHANGERATE;
		data->latest_timestamp[uSensorType] = get_current_timestamp();
		break;
	case GO_SLEEP:
		command = CMD_SETVALUE;
		sub_cmd = SCONTEXT_AP_STATUS_SLEEP;
		data->uLastAPState = SCONTEXT_AP_STATUS_SLEEP;
		break;
	case REMOVE_LIBRARY:
		command = CMD_REMOVE;
		break;
	case ADD_LIBRARY:
		command = CMD_ADD;
		break;
	default:
		command = uInst;
		break;
	}

	buffer = kzalloc(uLength, GFP_KERNEL);
	memcpy(buffer, uSendBuf, uLength);

	ret = ssp_send_command(data, command, uSensorType, sub_cmd, 1000, buffer,
	                       uLength, NULL, NULL);

	if (ret != SUCCESS) {
		ssp_errf("ssp_send_command Fail %d", ret);
		goto exit;
	}

exit:
	if (buffer != NULL) {
		kfree(buffer);
	}

	return ret;
}

int flush(struct ssp_data *data, u8 uSensorType)
{
	int ret = 0;

	ret = ssp_send_command(data, CMD_GETVALUE, uSensorType, SENSOR_FLUSH, 0, NULL,
	                       0, NULL, NULL);

	if (ret != SUCCESS) {
		ssp_errf("fail %d", ret);
		return ERROR;
	}

	return SUCCESS;
}
