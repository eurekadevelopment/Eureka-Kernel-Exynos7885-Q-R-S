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

#ifndef __SSP_SPI_H__
#define __SSP_SPI_H__

#include "ssp.h"

/* ssp_msg options bit */
#define SSP_SPI         0       /* read write mask */
#define SSP_RETURN      2       /* write and read option */
#define SSP_GYRO_DPS    3       /* gyro dps mask */
#define SSP_INDEX       3       /* data index mask */
#define SSP_SPI_MASK    (3 << SSP_SPI)  /* read write mask */

#define BIG_DATA_SENSOR_TYPE_ACCELEROMETER                      (0)
#define BIG_DATA_SENSOR_TYPE_GYROSCOPE                          (1)
#define BIG_DATA_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED        (2)
#define BIG_DATA_SENSOR_TYPE_GEOMAGNETIC_POWER                  (3)
#define BIG_DATA_SENSOR_TYPE_GEOMAGNETIC_FIELD                  (4)
#define BIG_DATA_SENSOR_TYPE_PRESSURE                           (5)
#define BIG_DATA_SENSOR_TYPE_PROXIMITY                          (7)
#define BIG_DATA_SENSOR_TYPE_LIGHT                              (9)
#define BIG_DATA_SENSOR_TYPE_PROXIMITY_RAW                      (10)

struct ssp_data;
struct ssp_msg;

int ssp_send_command(struct ssp_data *data, u8 cmd, u8 type, u8 subcmd,
                     int timeout, char *send_buf, int send_buf_len, char **receive_buf,
                     int *receive_buf_len);
int select_irq_msg(struct ssp_data *data);
void clean_pending_list(struct ssp_data *data);
int ssp_send_status(struct ssp_data *data, char command);
int make_command(struct ssp_data *data, u8 uInst,
                 u8 uSensorType, u8 *uSendBuf, u16 uLength);
int make_command_sync(struct ssp_data *data, u8 uInst,
                      u8 uSensorType, u8 *uSendBuf, u16 uLength);
int flush(struct ssp_data *data, u8 uSensorType);

#endif
