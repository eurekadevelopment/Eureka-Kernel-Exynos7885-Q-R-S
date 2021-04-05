/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
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
#include "../../ssp.h"
#include "../ssp_factory.h"

/*************************************************************************/
/* factory Test                                                          */
/*************************************************************************/
ssize_t get_light_tmd3725_name(char *buf)
{
	return sprintf(buf, "%s\n", "TMD3725");
}

ssize_t get_light_tmd3725_vendor(char *buf)
{
	return sprintf(buf, "%s\n", "AMS");
}

ssize_t get_light_tmd3725_lux(struct ssp_data *data, char *buf)
{
	return sprintf(buf, "%u,%u,%u,%u,%u,%u\n",
		data->buf[SENSOR_TYPE_LIGHT].r, data->buf[SENSOR_TYPE_LIGHT].g,
		data->buf[SENSOR_TYPE_LIGHT].b, data->buf[SENSOR_TYPE_LIGHT].w,
		data->buf[SENSOR_TYPE_LIGHT].a_time, data->buf[SENSOR_TYPE_LIGHT].a_gain);
}

ssize_t get_ams_light_tmd3725_data(struct ssp_data *data, char *buf)
{
	return sprintf(buf, "%u,%u,%u,%u,%u,%u\n",
		data->buf[SENSOR_TYPE_LIGHT].r, data->buf[SENSOR_TYPE_LIGHT].g,
		data->buf[SENSOR_TYPE_LIGHT].b, data->buf[SENSOR_TYPE_LIGHT].w,
		data->buf[SENSOR_TYPE_LIGHT].a_time, data->buf[SENSOR_TYPE_LIGHT].a_gain);
}

ssize_t get_light_tmd3725_coefficient(struct ssp_data *data, char *buf)
{
	int iRet, iReties = 0;
	struct ssp_msg *msg;
	int coef_buf[7] = {0, };

retries:
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	msg->cmd = MSG2SSP_AP_GET_LIGHT_COEF;
	msg->length = 28;
	msg->options = AP2HUB_READ;
	msg->buffer = (u8*)coef_buf;
	msg->free_buffer = 0;

	iRet = ssp_spi_sync(data, msg, 1000);
	if (iRet != SUCCESS) {
		pr_err("[SSP] %s fail %d\n", __func__, iRet);

		if (iReties++ < 2) {
			pr_err("[SSP] %s fail, retry\n", __func__);
			mdelay(5);
			goto retries;
		}
		return FAIL;
	}

	pr_info("[SSP] %s - %d %d %d %d %d %d %d\n",__func__,
		coef_buf[0],coef_buf[1],coef_buf[2],coef_buf[3],coef_buf[4],coef_buf[5],coef_buf[6]);
	
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d,%d\n", 
		coef_buf[0],coef_buf[1],coef_buf[2],coef_buf[3],coef_buf[4],coef_buf[5],coef_buf[6]);
}


struct 	light_sensor_operations light_tmd3725_ops = {
	.get_light_name = get_light_tmd3725_name,
	.get_light_vendor = get_light_tmd3725_vendor,
	.get_lux = get_light_tmd3725_lux,
	.get_light_data = get_ams_light_tmd3725_data,
	.get_light_coefficient = get_light_tmd3725_coefficient,
};

void light_tmd3725_function_pointer_initialize(struct ssp_data *data)
{
	data->light_ops = &light_tmd3725_ops;
}
