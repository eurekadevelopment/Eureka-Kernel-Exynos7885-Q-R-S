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

#include "ssp_dev.h"

void ssp_enable(struct ssp_data *data, bool enable)
{
	ssp_infof("enable = %d, is_shutdown = %d",
	          enable, data->is_ssp_shutdown);

	if (enable && data->is_ssp_shutdown) {
		data->is_ssp_shutdown = false;
		enable_irq(data->irq);
	} else if (!enable && !data->is_ssp_shutdown) {
		data->is_ssp_shutdown = true;
		disable_irq(data->irq);
	} else {
		ssp_errf("enable error");
	}
}

u64 get_current_timestamp(void)
{
	u64 timestamp;
	struct timespec ts;

	ts = ktime_to_timespec(ktime_get_boottime());
	timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;

	return timestamp;
}

/************************************************************************/
/* interrupt happened due to transition/change of SSP MCU               */
/************************************************************************/
static irqreturn_t sensordata_irq_thread_fn(int irq, void *dev_id)
{
	struct ssp_data *data = dev_id;

	data->timestamp = get_current_timestamp();

	if (gpio_get_value(data->mcu_int1)) {
		ssp_info("MCU int HIGH");
		return IRQ_HANDLED;
	}
	select_irq_msg(data);
	data->cnt_irq++;

	return IRQ_HANDLED;
}

/*************************************************************************/
/* initialize sensor hub                                                 */
/*************************************************************************/
static void initialize_variable(struct ssp_data *data)
{
	int type;
	ssp_errf("initialize_variable.");

	for (type = 0; type < SENSOR_TYPE_MAX; type++) {
		data->delay[type] = DEFUALT_POLLING_DELAY;
		data->aiCheckStatus[type] = INITIALIZATION_STATE;
		data->sensor_dump[type] = NULL;
	}

	data->uSensorState = NORMAL_SENSOR_STATE_K;

	data->is_ssp_shutdown = true;
	data->is_time_syncing = true;

	data->buf[SENSOR_TYPE_GYROSCOPE].gyro_dps = GYROSCOPE_DPS2000;
	
#ifdef CONFIG_SENSORS_SSP_GYROSCOPE
	data->first_gyro_cal = true;
#endif
#ifdef CONFIG_SENSOR_SSP_MAGNETIC
	data->uMagCntlRegData = 1;
	data->bGeomagneticRawEnabled = false;
#endif
	data->is_reset_from_kernel = false;
	data->is_reset_started = false;

	data->uLastResumeState = SCONTEXT_AP_STATUS_RESUME;

	data->realtime_dump_file = NULL;
	data->callstack_data = NULL;
	INIT_LIST_HEAD(&data->pending_list);
}


int initialize_mcu(struct ssp_data *data)
{
	int ret = 0;

	ssp_dbgf();
	clean_pending_list(data);

	ssp_errf("is_shutdown = %d", data->is_ssp_shutdown);

	ret = set_sensor_position(data);
	if (ret < 0) {
		ssp_errf("set_sensor_position failed");
		return ret;
	}

	data->uSensorState = get_sensor_scanning_info(data);
	if (data->uSensorState == 0) {
		ssp_errf("get_sensor_scanning_info failed");
		ret = ERROR;
		return ret;
	} else {
		ssp_infof("sensor state 0x%llx", data->uSensorState);
	}

#ifdef CONFIG_SENSORS_SSP_MAGNETIC
	ret = initialize_magnetic_sensor(data);
	if (ret < 0) {
		ssp_errf("initialize magnetic sensor failed");
	}
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT
	set_light_coef(data);
#endif

	data->curr_fw_rev = get_firmware_rev(data);
	ssp_info("MCU Firm Rev : New = %8u", data->curr_fw_rev);

	ssp_send_status(data, data->uLastResumeState);

	return ret;
}

static int initialize_irq(struct ssp_data *data)
{
	int ret, irq;

	irq = gpio_to_irq(data->mcu_int1);

	ssp_info("requesting IRQ %d", irq);
	ret = request_threaded_irq(irq, NULL, sensordata_irq_thread_fn,
	                           IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "SSP_Int", data);
	if (ret < 0) {
		ssp_errf("request_irq(%d) failed for gpio %d (%d)",
		         irq, irq, ret);
		goto err_request_irq;
	}

	/* start with interrupts disabled */
	data->irq = irq;
	disable_irq(data->irq);
	return 0;

err_request_irq:
	gpio_free(data->mcu_int1);
	return ret;
}

static void work_function_firmware_update(struct work_struct *work)
{
	struct ssp_data *data = container_of((struct delayed_work *)work,
	                                     struct ssp_data, work_firmware);
	int ret;

	ssp_errf();

	ret = forced_to_download_binary(data, KERNEL_BINARY);
	if (ret < 0) {
		ssp_infof("forced_to_download_binary failed!");
		data->uSensorState = 0;
		return;
	}

	queue_refresh_task(data, SSP_SW_RESET_TIME);
}

/* light sensor coefficient -> {R, G, B, C, DGF, CT, CT_OFFSET}*/
int jackpot_project[7] = {-830, 1100, -1180, 1000, 842, 3521, 2095};
int jackpot2_project[7] = {-830, 1100, -1180, 1000, 814, 3521, 2095};
int default_project[7] = {-830, 1100, -1180, 1000, 925, 3521, 2095};

static int ssp_parse_dt(struct device *dev,
                        struct ssp_data *data)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int errorno = 0;

	/* gpio pins */
	data->mcu_int1 = of_get_named_gpio_flags(np, "ssp,mcu_int1-gpio", 0, &flags);
	pr_info("[SSP] mcu_int1=%d\n", data->mcu_int1);
	if (data->mcu_int1 < 0) {
		pr_err("[SSP]: GPIO value not correct1\n");
		goto dt_exit;
	}
	data->mcu_int2 = of_get_named_gpio_flags(np, "ssp,mcu_int2-gpio", 0, &flags);
	pr_info("[SSP] mcu_int2=%d\n", data->mcu_int2);
	if (data->mcu_int2 < 0) {
		pr_err("[SSP]: GPIO value not correct2\n");
		goto dt_exit;
	}
	data->ap_int = of_get_named_gpio_flags(np, "ssp,ap_int-gpio", 0, &flags);
	pr_info("[SSP] ap_int=%d\n", data->ap_int);
	if (data->ap_int < 0) {
		pr_err("[SSP]: GPIO value not correct3\n");
		goto dt_exit;
	}

	data->rst = of_get_named_gpio_flags(np, "ssp,rst-gpio", 0, &flags);
	pr_info("[SSP] rst=%d\n", data->rst);
	if (data->rst < 0) {
		pr_err("[SSP]: GPIO value not correct4\n");
		goto dt_exit;
	}

	data->boot0 = of_get_named_gpio_flags(np, "ssp,boot0-gpio", 0, &flags);
	pr_info("[SSP] boot0=%d\n", data->boot0);
	if (data->boot0 < 0) {
		pr_err("[SSP]: GPIO value not correct5\n");
		goto dt_exit;
	}

	/* Config GPIO */
	errorno = gpio_request(data->mcu_int1, "mcu_ap_int1");
	if (errorno) {
		pr_err("[SSP]: failed to request mcu_ap_int1, ret:%d\n", errorno);
		goto dt_exit;
	}

	errorno = gpio_direction_input(data->mcu_int1);
	if (errorno) {
		pr_err("[SSP]: failed set mcu_ap_int1 as input mode, ret:%d",
		       errorno);
		goto dt_exit;
	}

	errorno = gpio_request(data->mcu_int2, "mcu_ap_int2");
	if (errorno) {
		pr_err("[SSP]: failed to request mcu_ap_int2, ret:%d\n",
		       errorno);
		goto dt_exit;
	}

	errorno = gpio_direction_input(data->mcu_int2);
	if (errorno) {
		pr_err("[SSP]: failed set mcu_ap_int2 as input mode, ret:%d\n",
		       errorno);
		goto dt_exit;
	}

	errorno = gpio_request(data->ap_int, "ap_mcu_int");
	if (errorno) {
		pr_err("[SSP]: failed to request ap_mcu_int, ret:%d\n",
		       errorno);
		goto dt_exit;
	}

	errorno = gpio_direction_output(data->ap_int, 1);
	if (errorno) {
		pr_err("[SSP]: failed set ap_mcu_int as output mode, ret:%d\n",
		       errorno);
		goto dt_exit;
	}

	errorno = gpio_request(data->rst, "mcu_rst");
	if (errorno) {
		pr_err("[SSP]: failed to request mcu_rst, ret:%d\n", errorno);
		goto dt_exit;
	}

	errorno = gpio_direction_output(data->rst, 1);
	if (errorno) {
		pr_err("[SSP]: failed set mcu_rst as output mode, ret:%d\n",
		       errorno);
		goto dt_exit;
	}

	errorno = gpio_request(data->boot0, "mcu_boot0");
	if (errorno) {
		pr_err("[SSP]: failed to request mcu_boot0, ret:%d\n", errorno);
		goto dt_exit;
	}

	errorno = gpio_direction_output(data->boot0, 0);
	if (errorno) {
		pr_err("[SSP]: failed set mcu_boot0 as output mode, ret:%d\n",
		       errorno);
		goto dt_exit;
	}

	/* sensor positions */
	if (of_property_read_u32(np, "ssp,acc-position", &data->accel_position)) {
		data->accel_position = 0;
	}

	if (of_property_read_u32(np, "ssp,mag-position", &data->mag_position)) {
		data->mag_position = 0;
	}

	ssp_info("acc-posi[%d] mag-posi[%d]",
	         data->accel_position,  data->mag_position);

#ifdef CONFIG_SENSORS_SSP_PROXIMITY
	/* prox thresh */
	if (of_property_read_u32(np, "ssp,prox-hi_thresh",
	                         &data->uProxHiThresh)) {
		data->uProxHiThresh = DEFUALT_HIGH_THRESHOLD;
	}

	if (of_property_read_u32(np, "ssp,prox-low_thresh",
	                         &data->uProxLoThresh)) {
		data->uProxLoThresh = DEFUALT_LOW_THRESHOLD;
	}

	ssp_info("hi-thresh[%u] low-thresh[%u]\n",
	         data->uProxHiThresh, data->uProxLoThresh);

	if (of_property_read_u32(np, "ssp,prox-detect_hi_thresh",
	                         &data->uProxHiThresh_detect)) {
		data->uProxHiThresh_detect = DEFUALT_DETECT_HIGH_THRESHOLD;
	}

	if (of_property_read_u32(np, "ssp,prox-detect_low_thresh",
	                         &data->uProxLoThresh_detect)) {
		data->uProxLoThresh_detect = DEFUALT_DETECT_LOW_THRESHOLD;
	}

	ssp_info("detect-hi[%u] detect-low[%u]\n",
	         data->uProxHiThresh_detect, data->uProxLoThresh_detect);
#endif
#ifdef CONFIG_SENSORS_SSP_ACCELOMETER
	/* acc type */
	if (of_property_read_u32(np, "ssp-acc-type", &data->acc_type)) {
		data->acc_type = 0;
	}

	ssp_info("acc-type = %d", data->acc_type);
#endif
#ifdef CONFIG_SENSORS_SSP_BAROMETER
	/* pressure type */
	if (of_property_read_u32(np, "ssp-pressure-type", &data->pressure_type)) {
		data->pressure_type = 0;
	}

	ssp_info("pressure-type = %d", data->pressure_type);
#endif
#ifdef CONFIG_SENSORS_SSP_MAGNETIC
	/* mag type */
	if (of_property_read_u32(np, "ssp-mag-type", &data->mag_type)) {
		data->mag_type = 0;
	}
	pr_info("[SSP] mag-type = %d\n", data->mag_type);

	/* mag matrix */
	if (data->mag_type == 1) {
		if (of_property_read_u8_array(np, "ssp-mag-array",
		                              data->pdc_matrix, sizeof(data->pdc_matrix))) {
			pr_err("no mag-array, set as 0");
		}
	} else {
		u32 len, temp;
		int i;
		if (!of_get_property(np, "ssp-mag-array", &len)) {
			pr_info("[SSP] No static matrix at DT for LSM303AH!(%p)\n",
			        data->static_matrix);
			goto dt_exit;
		}
		if (len / 4 != 9) {
			pr_err("[SSP] Length/4:%d should be 9 for LSM303AH!\n", len / 4);
			goto dt_exit;
		}
		data->static_matrix = kzalloc(9 * sizeof(s16), GFP_KERNEL);
		pr_info("[SSP] static matrix Length:%d, Len/4=%d\n", len, len / 4);

		for (i = 0; i < 9; i++) {
			if (of_property_read_u32_index(np, "ssp-mag-array", i, &temp)) {
				pr_err("[SSP] %s cannot get u32 of array[%d]!\n",
				       __func__, i);
				goto dt_exit;
			}
			*(data->static_matrix + i) = (int)temp;
		}
	}
#endif

#ifdef CONFIG_SENSORS_SSP_LIGHT
	/* Project type, 1 is jackpot, 2 is jackpot2 */
	if (of_property_read_u32(np, "ssp-project-type", &data->project_select)) {
		data->project_select = 0;
	}
	pr_info("[SSP] this project = %d\n", data->project_select);

	if (data->project_select == 1) {
		memcpy(data->light_coef, jackpot_project, sizeof(jackpot_project));
	} else if (data->project_select == 2) {
		memcpy(data->light_coef, jackpot2_project, sizeof(jackpot2_project));
	} else {
		ssp_errf("project is default");
		memcpy(data->light_coef, default_project, sizeof(default_project));
	}
#endif
	return errorno;
dt_exit:
#ifdef CONFIG_SENSORS_SSP_MAGNETIC
	if (data->static_matrix != NULL) {
		kfree(data->static_matrix);
	}
#endif
	return errorno;
}

void ssp_timestamp_resume(struct ssp_data *data)
{
	int type;
	for (type = 0; type < SENSOR_TYPE_MAX; type++) {
		data->latest_timestamp[type] = 0;
	}
}

/* this callback is called before suspend */
static int ssp_prepare(struct device *dev)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	ssp_infof();
	data->uLastResumeState = SCONTEXT_AP_STATUS_SUSPEND;
	disable_debug_timer(data);
	enable_irq_wake(data->irq);

	if (SUCCESS != ssp_send_status(data, SCONTEXT_AP_STATUS_SUSPEND)) {
		ssp_errf("SCONTEXT_AP_STATUS_SUSPEND failed");
	}

	data->is_time_syncing = false;
	return 0;
}

/* this callback is called after resume */
static void ssp_complete(struct device *dev)
{
	struct spi_device *spi_dev = to_spi_device(dev);
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	ssp_infof();
	enable_debug_timer(data);
	disable_irq_wake(data->irq);
	ssp_timestamp_resume(data);

	if (SUCCESS != ssp_send_status(data, SCONTEXT_AP_STATUS_RESUME)) {
		ssp_errf("SCONTEXT_AP_STATUS_RESUME failed");
	}

	data->uLastResumeState = SCONTEXT_AP_STATUS_RESUME;

	return;
}

static const struct dev_pm_ops ssp_pm_ops = {
	.prepare = ssp_prepare,
	.complete = ssp_complete,
};


static int ssp_probe(struct spi_device *spi)
{
	int ret = 0;
	struct ssp_data *data;

	ssp_infof();

	data = kzalloc(sizeof(*data), GFP_KERNEL);

	data->is_probe_done = false;

	if (spi->dev.of_node) {
		ret = ssp_parse_dt(&spi->dev, data);
		if (ret) {
			ssp_errf("Failed to parse DT");
			goto err_setup;
		}
	} else {
		ssp_errf("failed to get device node");
		ret = -ENODEV;
		goto err_setup;
	}

	spi->mode = SPI_MODE_1;
	if (spi_setup(spi)) {
		ssp_errf("failed to setup spi");
		ret = -ENODEV;
		goto err_setup;
	}
	data->fw_dl_state = FW_DL_STATE_NONE;
	data->spi = spi;
	spi_set_drvdata(spi, data);

	mutex_init(&data->comm_mutex);
	mutex_init(&data->pending_mutex);
	mutex_init(&data->enable_mutex);
	mutex_init(&data->cmd_mutex);

	pr_info("\n#####################################################\n");

	INIT_DELAYED_WORK(&data->work_firmware, work_function_firmware_update);
	INIT_DELAYED_WORK(&data->work_refresh, refresh_task);

	wake_lock_init(&data->ssp_wake_lock,
	               WAKE_LOCK_SUSPEND, "ssp_wake_lock");

	initialize_variable(data);
	ssp_dbg("initialize_variable DONE");

	ret = initialize_indio_dev(data);
	if (ret < 0) {
		ssp_errf("could not create input device");
		goto err_input_register_device;
	}

	ret = initialize_debug_timer(data);
	if (ret < 0) {
		ssp_errf("could not create workqueue");
		goto err_create_workqueue;
	}

	ret = initialize_irq(data);
	if (ret < 0) {
		ssp_errf("could not create irq");
		goto err_setup_irq;
	}

	ret = initialize_sysfs(data);
	if (ret < 0) {
		ssp_errf("could not create sysfs");
		goto err_sysfs_create;
	}


#ifdef SENSOR_TYPE_SCONTEXT
	/* init scontext device */
	ret = ssp_scontext_initialize(data);
	if (ret < 0) {
		ssp_errf("ssp_scontext_initialize err(%d)", ret);
		ssp_scontext_remove(data);
	}
#else
	/* init sensorhub device */
	ret = ssp_sensorhub_initialize(data);
	if (ret < 0) {
		ssp_errf("ssp_sensorhub_initialize err(%d)", ret);
		ssp_sensorhub_remove(data);
	}
#endif

	ssp_dbg("Go ssp_enable");

	ssp_enable(data, true);
	/* check boot loader binary */
	data->fw_dl_state = check_fwbl(data);

	if (data->fw_dl_state == FW_DL_STATE_NONE) {
		ret = initialize_mcu(data);
		if (ret == ERROR) {
			toggle_mcu_reset(data);
		} else if (ret < ERROR) {
			ssp_errf("initialize_mcu failed");
			goto err_read_reg;
		}
	}

	enable_debug_timer(data);

	if (data->fw_dl_state == FW_DL_STATE_NEED_TO_SCHEDULE) {
		ssp_info("Firmware update is scheduled");
		schedule_delayed_work(&data->work_firmware,
		                      msecs_to_jiffies(1000));
		data->fw_dl_state = FW_DL_STATE_SCHEDULED;
	} else if (data->fw_dl_state == FW_DL_STATE_FAIL) {
		data->is_ssp_shutdown = true;
	}
	data->is_probe_done = true;
	ret = 0;

	ssp_infof("probe success!");
	goto exit;


err_read_reg:
	remove_sysfs(data);
err_sysfs_create:
	free_irq(data->irq, data);
	gpio_free(data->mcu_int1);
err_setup_irq:
	destroy_workqueue(data->debug_wq);
err_create_workqueue:
	remove_indio_dev(data);
err_input_register_device:
	wake_lock_destroy(&data->ssp_wake_lock);
	mutex_destroy(&data->comm_mutex);
	mutex_destroy(&data->pending_mutex);
	mutex_destroy(&data->enable_mutex);
	mutex_destroy(&data->cmd_mutex);
err_setup:
	kfree(data);
	ssp_errf("probe failed!");

exit:
	pr_info("#####################################################\n\n");
	return ret;
}

static void ssp_shutdown(struct spi_device *spi_dev)
{
	struct ssp_data *data = spi_get_drvdata(spi_dev);

	ssp_infof();
	if (data->is_probe_done == false) {
		goto exit;
	}

	disable_debug_timer(data);

	if (data->fw_dl_state >= FW_DL_STATE_SCHEDULED &&
	    data->fw_dl_state < FW_DL_STATE_DONE) {
		ssp_errf("cancel_delayed_work_sync state = %d",
		         data->fw_dl_state);
		cancel_delayed_work_sync(&data->work_firmware);
	}

	if (SUCCESS != ssp_send_status(data, SCONTEXT_AP_STATUS_SHUTDOWN)) {
		ssp_errf("SCONTEXT_AP_STATUS_SHUTDOWN failed");
	}

	data->is_ssp_shutdown = true;
	disable_irq_wake(data->irq);
	disable_irq(data->irq);

	clean_pending_list(data);

	free_irq(data->irq, data);
	gpio_free(data->mcu_int1);

	remove_sysfs(data);

#ifdef SENSOR_TYPE_SCONTEXT
	ssp_scontext_remove(data);
#else
	ssp_sensorhub_remove(data);
#endif

	del_timer_sync(&data->debug_timer);
	cancel_work_sync(&data->work_debug);
	cancel_delayed_work_sync(&data->work_refresh);
	destroy_workqueue(data->debug_wq);
	wake_lock_destroy(&data->ssp_wake_lock);
	mutex_destroy(&data->comm_mutex);
	mutex_destroy(&data->pending_mutex);
	mutex_destroy(&data->enable_mutex);
	toggle_mcu_reset(data);
	ssp_infof("done");
exit:
	kfree(data);
}

static const struct spi_device_id ssp_id[] = {
	{"ssp", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, ssp_id);

#ifdef CONFIG_OF
static struct of_device_id ssp_match_table[] = {
	{ .compatible = "ssp,STM32F",},
	{},
};
#endif

static struct spi_driver ssp_driver = {
	.probe = ssp_probe,
	.shutdown = ssp_shutdown,
	.id_table = ssp_id,
	.driver = {
		.pm = &ssp_pm_ops,
		.owner = THIS_MODULE,
		.name = "ssp",
#ifdef CONFIG_OF
		.of_match_table = ssp_match_table
#endif
	},
};

static int __init ssp_init(void)
{
	return spi_register_driver(&ssp_driver);
}

static void __exit ssp_exit(void)
{
	spi_unregister_driver(&ssp_driver);
}

late_initcall(ssp_init);
module_exit(ssp_exit);
MODULE_DESCRIPTION("Seamless Sensor Platform(SSP) dev driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
