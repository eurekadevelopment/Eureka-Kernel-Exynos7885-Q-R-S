/*  Himax Android Driver Sample Code for QCT platform

	Copyright (C) 2018 Himax Corporation.

	This software is licensed under the terms of the GNU General Public
	License version 2, as published by the Free Software Foundation, and
	may be copied, distributed, and modified under those terms.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

*/

#include "himax_platform.h"
#include "himax_common.h"

#ifdef CONFIG_SAMSUNG_TUI
#include "stui_inf.h"
#endif

#ifdef CONFIG_SAMSUNG_TUI
struct himax_ts_data *tui_tsp_info;
#endif

int i2c_error_count = 0;
int irq_enable_count = 0;

extern struct himax_ic_data *ic_data;
extern struct himax_ts_data *private_ts;
extern void himax_ts_work(struct himax_ts_data *ts);
extern enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer);

extern int himax_chip_common_init(void);
extern void himax_chip_common_deinit(void);

int himax_dev_set(struct himax_ts_data *ts)
{
	int ret = 0;
	ts->input_dev = input_allocate_device();

	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		input_err(true, &ts->client->dev,
				"%s%s: Failed to allocate input device\n",
				HIMAX_LOG_TAG, __func__);
		return ret;
	}

	ts->input_dev->name = "sec_touchscreen";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->dev.parent = &ts->client->dev;
	return ret;
}

int himax_input_register_device(struct input_dev *input_dev)
{
	return input_register_device(input_dev);
}

#if defined(HX_PLATFOME_DEFINE_KEY)
void himax_platform_key(void)
{
	I("Nothing to be done! Plz cancel it!\n");
}
#endif

void himax_vk_parser(struct device_node *dt,
			struct himax_i2c_platform_data *pdata)
{
	u32 data = 0;
	uint8_t cnt = 0, i = 0;
	uint32_t coords[4] = { 0 };
	struct device_node *node, *pp = NULL;
	struct himax_virtual_key *vk;
	node = of_parse_phandle(dt, "virtualkey", 0);

	if (node == NULL) {
		E(" DT-No vk info in DT\n");
		return;
	} else {
		while ((pp = of_get_next_child(node, pp)))
			cnt++;

		if (!cnt)
			return;

		vk = kzalloc(cnt * sizeof(struct himax_virtual_key),
				GFP_KERNEL);
		pp = NULL;

		while ((pp = of_get_next_child(node, pp))) {
			if (of_property_read_u32(pp, "idx", &data) == 0)
				vk[i].index = data;

			if (of_property_read_u32_array(pp, "range", coords, 4) == 0) {
				vk[i].x_range_min =
					coords[0], vk[i].x_range_max = coords[1];
				vk[i].y_range_min =
					coords[2], vk[i].y_range_max = coords[3];
			} else {
				E(" range faile\n");
			}

			i++;
		}

		pdata->virtual_key = vk;

		for (i = 0; i < cnt; i++)
			I(" vk[%d] idx:%d x_min:%d, y_max:%d\n", i,
				pdata->virtual_key[i].index,
				pdata->virtual_key[i].x_range_min,
				pdata->virtual_key[i].y_range_max);
	}
}

int himax_parse_dt(struct himax_ts_data *ts,
			struct himax_i2c_platform_data *pdata)
{
	int rc, coords_size = 0;
	int tmp[3];
	uint32_t coords[4] = { 0 };
	struct property *prop;
	struct device_node *dt = private_ts->client->dev.of_node;
	u32 data = 0;
	prop = of_find_property(dt, "himax,panel-coords", NULL);

	if (prop) {
		coords_size = prop->length / sizeof(u32);

		if (coords_size != 4)
			D(" %s:Invalid panel coords size %d\n", __func__,
				coords_size);
	}

	if (of_property_read_u32_array(dt, "himax,panel-coords", coords, coords_size) == 0) {
		pdata->abs_x_min = coords[0], pdata->abs_x_max =
			(coords[1] - 1);
		pdata->abs_y_min = coords[2], pdata->abs_y_max =
			(coords[3] - 1);
		input_info(true, &ts->client->dev,
			"%s DT-%s:panel-coords = %d, %d, %d, %d\n",
			HIMAX_LOG_TAG, __func__, pdata->abs_x_min,
			pdata->abs_x_max, pdata->abs_y_min,
			pdata->abs_y_max);
	}

	prop = of_find_property(dt, "himax,display-coords", NULL);

	if (prop) {
		coords_size = prop->length / sizeof(u32);

		if (coords_size != 4)
			D(" %s:Invalid display coords size %d\n", __func__,
				coords_size);
	}

	rc = of_property_read_u32_array(dt, "himax,display-coords", coords,
					coords_size);

	if (rc && (rc != -EINVAL)) {
		D(" %s:Fail to read display-coords %d\n", __func__, rc);
		return rc;
	}

	pdata->screenWidth = coords[1];
	pdata->screenHeight = coords[3];
	input_info(true, &ts->client->dev,
		"%s DT-%s:display-coords = (%d, %d)\n", HIMAX_LOG_TAG,
		__func__, pdata->screenWidth, pdata->screenHeight);

	if (of_property_read_u32_array(dt, "himax,area-size", tmp, 3)) {
		pdata->area_indicator = 63;
		pdata->area_navigation = 126;
		pdata->area_edge = 60;
	} else {
		pdata->area_indicator = tmp[0];
		pdata->area_navigation = tmp[1];
		pdata->area_edge = tmp[2];
	}

	pdata->gpio_irq = of_get_named_gpio(dt, "himax,irq-gpio", 0);

	if (!gpio_is_valid(pdata->gpio_irq)) {
		input_info(true, &ts->client->dev,
			"%s DT:gpio_irq value is not valid\n",
			HIMAX_LOG_TAG);
	}

	pdata->gpio_reset = of_get_named_gpio(dt, "himax,rst-gpio", 0);

	if (!gpio_is_valid(pdata->gpio_reset)) {
		input_info(true, &ts->client->dev,
			"%s DT:gpio_rst value is not valid\n",
			HIMAX_LOG_TAG);
	}

	pdata->gpio_3v3_en = of_get_named_gpio(dt, "himax,3v3-gpio", 0);

	if (!gpio_is_valid(pdata->gpio_3v3_en)) {
		input_info(true, &ts->client->dev,
			"%s DT:gpio_3v3_en value is not valid\n",
			HIMAX_LOG_TAG);
	}

	pdata->support_dual_fw = of_property_read_bool(dt, "support_dual_fw");
	of_property_read_string(dt, "himax,fw-path",
				&pdata->i_CTPM_firmware_name);
	/* CU IC: 11 62 30, AL IC: 8A 62 34*/
	if (pdata->support_dual_fw && lcdtype == 0x8A6234) {
		of_property_read_string(dt, "himax,fw-path_al",
				&pdata->i_CTPM_firmware_name);
		input_info(true, &ts->client->dev, "%s DT:fw for AL IC",HIMAX_LOG_TAG);
	}

	input_info(true, &ts->client->dev,
		"%s DT:gpio_irq=%d, gpio_rst=%d, gpio_3v3_en=%d\n",
		HIMAX_LOG_TAG, pdata->gpio_irq, pdata->gpio_reset,
		pdata->gpio_3v3_en);

	if (of_property_read_u32(dt, "report_type", &data) == 0) {
		pdata->protocol_type = data;
		input_info(true, &ts->client->dev, "%s DT:protocol_type=%d\n",
			HIMAX_LOG_TAG, pdata->protocol_type);
	}

	if (of_property_read_u32
		(dt, "himax,factory_item_version", &pdata->item_version) < 0)
		pdata->item_version = 0;

	of_property_read_string(dt, "himax,project_name",
				&pdata->proj_name);
	I("Now project nam=%s\n", pdata->proj_name);
	
	himax_vk_parser(dt, pdata);
	return 0;
}

int himax_bus_read(uint8_t command, uint8_t * data, uint32_t length,
			uint8_t toRetry)
{
	int retry;
	struct i2c_client *client = private_ts->client;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

#ifdef CONFIG_SAMSUNG_TUI
	if (STUI_MODE_TOUCH_SEC & stui_get_mode())
		return -EBUSY;
#endif

	mutex_lock(&private_ts->rw_lock);

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;

		msleep(20);
		private_ts->comm_err_count++;
	}

	if (retry == toRetry) {
		input_err(true, &client->dev,
			"%s %s: i2c_read_block retry over %d\n",
			HIMAX_LOG_TAG, __func__, toRetry);
		i2c_error_count = toRetry;
		mutex_unlock(&private_ts->rw_lock);
		return -EIO;
	}

	mutex_unlock(&private_ts->rw_lock);
	return 0;
}

int himax_bus_write(uint8_t command, uint8_t * data, uint32_t length,
			uint8_t toRetry)
{
	int retry /*, loop_i */ ;
	uint8_t buf[length + 1];
	struct i2c_client *client = private_ts->client;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

#ifdef CONFIG_SAMSUNG_TUI
	if (STUI_MODE_TOUCH_SEC & stui_get_mode())
		return -EBUSY;
#endif

	mutex_lock(&private_ts->rw_lock);
	buf[0] = command;
	memcpy(buf + 1, data, length);

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;

		msleep(20);
		private_ts->comm_err_count++;
	}

	if (retry == toRetry) {
		input_err(true, &client->dev,
			"%s%s: i2c_write_block retry over %d\n",
			HIMAX_LOG_TAG, __func__, toRetry);
		i2c_error_count = toRetry;
		mutex_unlock(&private_ts->rw_lock);
		return -EIO;
	}

	mutex_unlock(&private_ts->rw_lock);
	return 0;
}

int himax_bus_write_command(uint8_t command, uint8_t toRetry)
{
	return himax_bus_write(command, NULL, 0, toRetry);
}

int himax_bus_master_write(uint8_t * data, uint32_t length, uint8_t toRetry)
{
	int retry /*, loop_i */ ;
	uint8_t buf[length];
	struct i2c_client *client = private_ts->client;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length,
			.buf = buf,
		}
	};

#ifdef CONFIG_SAMSUNG_TUI
	if (STUI_MODE_TOUCH_SEC & stui_get_mode())
		return -EBUSY;
#endif

	mutex_lock(&private_ts->rw_lock);
	memcpy(buf, data, length);

	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;

		msleep(20);
		private_ts->comm_err_count++;
	}

	if (retry == toRetry) {
		input_err(true, &client->dev,
			"%s %s: i2c_write_block retry over %d\n",
			HIMAX_LOG_TAG, __func__, toRetry);
		i2c_error_count = toRetry;
		mutex_unlock(&private_ts->rw_lock);
		return -EIO;
	}

	mutex_unlock(&private_ts->rw_lock);
	return 0;
}

void himax_int_enable(int enable)
{
	int irqnum = 0;
	irqnum = private_ts->client->irq;

	if (enable == 1 && irq_enable_count == 0) {
		enable_irq(irqnum);
		irq_enable_count = 1;
		private_ts->irq_enabled = 1;
	} else if (enable == 0 && irq_enable_count == 1) {
		disable_irq_nosync(irqnum);
		irq_enable_count = 0;
		private_ts->irq_enabled = 0;
	}

	input_info(true, &private_ts->client->dev, "%s irq_enable_count = %d\n",
			HIMAX_LOG_TAG, irq_enable_count);
}

#ifdef HX_RST_PIN_FUNC
void himax_rst_gpio_set(int pinnum, uint8_t value)
{
	gpio_direction_output(pinnum, value);
}
#endif

uint8_t himax_int_gpio_read(int pinnum)
{
	return gpio_get_value(pinnum);
}

#if defined(CONFIG_HMX_DB)
static int himax_regulator_configure(struct himax_i2c_platform_data *pdata)
{
	int retval;
	struct i2c_client *client = private_ts->client;
	pdata->vcc_dig = regulator_get(&client->dev, "vdd");

	if (IS_ERR(pdata->vcc_dig)) {
		input_err(true, &client->dev,
			"%s %s: Failed to get regulator vdd\n", HIMAX_LOG_TAG,
			__func__);
		retval = PTR_ERR(pdata->vcc_dig);
		return retval;
	}

	pdata->vcc_ana = regulator_get(&client->dev, "avdd");

	if (IS_ERR(pdata->vcc_ana)) {
		input_err(true, &client->dev,
			"%s %s: Failed to get regulator avdd\n",
			HIMAX_LOG_TAG, __func__);
		retval = PTR_ERR(pdata->vcc_ana);
		regulator_put(pdata->vcc_ana);
		return retval;
	}

	return 0;
};

static int himax_power_on(struct himax_i2c_platform_data *pdata, bool on)
{
	int retval;

	if (on) {
		retval = regulator_enable(pdata->vcc_dig);

		if (retval) {
			input_err(true, &private_ts->client->dev,
				"%s %s: Failed to enable regulator vdd\n",
				HIMAX_LOG_TAG, __func__);
			return retval;
		}

		msleep(100);
		retval = regulator_enable(pdata->vcc_ana);

		if (retval) {
			input_err(true, &private_ts->client->dev,
				"%s %s: Failed to enable regulator avdd\n",
				HIMAX_LOG_TAG, __func__);
			regulator_disable(pdata->vcc_dig);
			return retval;
		}
	} else {
		regulator_disable(pdata->vcc_dig);
		regulator_disable(pdata->vcc_ana);
	}

	return 0;
}

int himax_gpio_power_config(struct himax_i2c_platform_data *pdata)
{
	int error;
	struct i2c_client *client = private_ts->client;
	error = himax_regulator_configure(pdata);

	if (error) {
		input_err(true, &client->dev,
			"%s Failed to intialize hardware\n", HIMAX_LOG_TAG);
		goto err_regulator_not_on;
	}
#ifdef HX_RST_PIN_FUNC

	if (gpio_is_valid(pdata->gpio_reset)) {
		/* configure touchscreen reset out gpio */
		error = gpio_request(pdata->gpio_reset, "hmx_reset_gpio");

		if (error) {
			input_err(true, &client->dev,
				"%s unable to request gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_reset);
			goto err_regulator_on;
		}

		error = gpio_direction_output(pdata->gpio_reset, 0);

		if (error) {
			input_err(true, &client->dev,
				"%s unable to set direction for gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_reset);
			goto err_gpio_reset_req;
		}
	}
#endif
	error = himax_power_on(pdata, true);

	if (error) {
		input_err(true, &client->dev,
			"%s Failed to power on hardware\n", HIMAX_LOG_TAG);
		goto err_gpio_reset_req;
	}

	if (gpio_is_valid(pdata->gpio_irq)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(pdata->gpio_irq, "hmx_gpio_irq");

		if (error) {
			input_err(true, &client->dev,
				"%s unable to request gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_irq);
			goto err_power_on;
		}

		error = gpio_direction_input(pdata->gpio_irq);

		if (error) {
			input_err(true, &client->dev,
				"%s unable to set direction for gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_irq);
			goto err_gpio_irq_req;
		}

		client->irq = gpio_to_irq(pdata->gpio_irq);
	} else {
		input_err(true, &client->dev, "%s irq gpio not provided\n",
			HIMAX_LOG_TAG);
		goto err_power_on;
	}

	msleep(20);
#ifdef HX_RST_PIN_FUNC

	if (gpio_is_valid(pdata->gpio_reset)) {
		error = gpio_direction_output(pdata->gpio_reset, 1);

		if (error) {
			input_err(true, &client->dev,
				"%s unable to set direction for gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_reset);
			goto err_gpio_irq_req;
		}
	}
#endif

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);

	return 0;
err_gpio_irq_req:
	if (gpio_is_valid(pdata->gpio_irq))
		gpio_free(pdata->gpio_irq);

err_power_on:
	himax_power_on(pdata, false);
err_gpio_reset_req:
#ifdef HX_RST_PIN_FUNC
	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);
err_regulator_on:
#endif
err_regulator_not_on:
	return error;
}

#else
int himax_gpio_power_config(struct himax_i2c_platform_data *pdata)
{
	int error = 0;
	struct i2c_client *client = private_ts->client;
#ifdef HX_RST_PIN_FUNC

	if (pdata->gpio_reset >= 0) {
		error = gpio_request(pdata->gpio_reset, "himax-reset");

		if (error < 0) {
			input_err(true, &client->dev,
				"%s %s: request reset pin failed\n",
				HIMAX_LOG_TAG, __func__);
			return error;
		}

		error = gpio_direction_output(pdata->gpio_reset, 0);

		if (error) {
			input_err(true, &client->dev,
				"%s unable to set direction for gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_reset);
			goto out;
		}
	}
#endif

	if (pdata->gpio_3v3_en >= 0) {
		error = gpio_request(pdata->gpio_3v3_en, "himax-3v3_en");

		if (error < 0) {
			input_err(true, &client->dev,
				"%s %s: request 3v3_en pin failed\n",
				HIMAX_LOG_TAG, __func__);
			goto out;
		}

		gpio_direction_output(pdata->gpio_3v3_en, 1);
		input_info(true, &client->dev, "%s 3v3_en pin =%d\n",
			HIMAX_LOG_TAG, gpio_get_value(pdata->gpio_3v3_en));
	}

	if (gpio_is_valid(pdata->gpio_irq)) {
/* configure touchscreen irq gpio */
		error = gpio_request(pdata->gpio_irq, "himax_gpio_irq");

		if (error) {
			input_err(true, &client->dev,
				"%s unable to request gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_irq);
			goto out;
		}

		error = gpio_direction_input(pdata->gpio_irq);

		if (error) {
			input_err(true, &client->dev,
				"%s unable to set direction for gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_irq);
			goto out;
		}

		client->irq = gpio_to_irq(pdata->gpio_irq);
	} else {
		input_err(true, &client->dev, "%s irq gpio not provided\n",
				HIMAX_LOG_TAG);
		goto out;
	}

	msleep(20);
#ifdef HX_RST_PIN_FUNC

	if (pdata->gpio_reset >= 0) {
		error = gpio_direction_output(pdata->gpio_reset, 1);

		if (error) {
			input_err(true, &client->dev,
				"%s unable to set direction for gpio [%d]\n",
				HIMAX_LOG_TAG, pdata->gpio_reset);
			goto out;
		}
	}
#endif
out:
	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);

	return error;
}

#endif

static void himax_ts_isr_func(struct himax_ts_data *ts)
{
	himax_ts_work(ts);
}

irqreturn_t himax_ts_thread(int irq, void *ptr)
{
	himax_ts_isr_func((struct himax_ts_data *)ptr);

	return IRQ_HANDLED;
}

static void himax_ts_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts =
		container_of(work, struct himax_ts_data, work);
	himax_ts_work(ts);
}

int himax_int_register_trigger(void)
{
	int ret = 0;
	struct himax_ts_data *ts = private_ts;
	struct i2c_client *client = private_ts->client;

	if (ic_data->HX_INT_IS_EDGE) {
		input_info(true, &ts->client->dev,
			"%s %s edge triiger falling\n ", HIMAX_LOG_TAG,
			__func__);
		ret =
			request_threaded_irq(client->irq, NULL, himax_ts_thread,
					 IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					 client->name, ts);
	}

	else {
		input_info(true, &ts->client->dev, "%s %s level trigger low\n ",
				HIMAX_LOG_TAG, __func__);
		ret =
			request_threaded_irq(client->irq, NULL, himax_ts_thread,
					 IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					 client->name, ts);
	}

	return ret;
}

int himax_int_en_set(void)
{
	int ret = NO_ERR;
	ret = himax_int_register_trigger();
	return ret;
}

int himax_ts_register_interrupt(void)
{
	struct himax_ts_data *ts = private_ts;
	struct i2c_client *client = private_ts->client;
	int ret = 0;
	ts->irq_enabled = 0;

	/* Work functon */
	if (client->irq) {	/*INT mode */
		ts->use_irq = 1;
		ret = himax_int_register_trigger();

		if (ret == 0) {
			ts->irq_enabled = 1;
			irq_enable_count = 1;
			input_info(true, &ts->client->dev,
				"%s %s: irq enabled at qpio: %d\n",
				HIMAX_LOG_TAG, __func__, client->irq);
#ifdef HX_SMART_WAKEUP
			irq_set_irq_wake(client->irq, 1);
#endif
		} else {
			ts->use_irq = 0;
			input_err(true, &ts->client->dev,
				"%s %s: request_irq failed\n", HIMAX_LOG_TAG,
				__func__);
		}
	} else {
		input_info(true, &ts->client->dev,
			"%s %s: client->irq is empty, use polling mode.\n",
			HIMAX_LOG_TAG, __func__);
	}

	if (!ts->use_irq) {	/*if use polling mode need to disable HX_ESD_RECOVERY function */
		ts->himax_wq = create_singlethread_workqueue("himax_touch");
		INIT_WORK(&ts->work, himax_ts_work_func);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		input_info(true, &ts->client->dev, "%s: polling mode enabled\n",
				__func__);
	}

	return ret;
}

static int himax_common_suspend(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	input_info(true, &ts->client->dev, "%s %s: enter \n", HIMAX_LOG_TAG,
			__func__);
	himax_chip_common_suspend(ts);
	return 0;
}

static int himax_common_resume(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);
	input_info(true, &ts->client->dev, "%s %s: enter \n", HIMAX_LOG_TAG,
			__func__);
	himax_chip_common_resume(ts);
	return 0;
}

#if defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
			unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct himax_ts_data *ts =
		container_of(self, struct himax_ts_data, fb_notif);
	int *blank = NULL;

	if (evdata == NULL) {
		input_err(true, &ts->client->dev, "%s %s evdata is null\n",
			HIMAX_LOG_TAG, __func__);
		return 0;
	}

	blank = evdata->data;

	if (evdata && evdata->data && ts && ts->client) {
		if (event == FB_EARLY_EVENT_BLANK &&
			*blank == FB_BLANK_POWERDOWN) {
			himax_common_suspend(&ts->client->dev);
		} else if (event == FB_EVENT_BLANK &&
				*blank == FB_BLANK_UNBLANK) {
			himax_common_resume(&ts->client->dev);
		}
	}

	return 0;
}
#endif

unsigned int hx_bootmode;
static int __init get_bootmode(char *arg)
{
	get_option(&arg, &hx_bootmode);

	return 0;
}
early_param("bootmode", get_bootmode);

static int himax_reboot_notifier(struct notifier_block *this,
		unsigned long code, void *unused)
{
	struct himax_ts_data *ts = container_of(this, struct himax_ts_data, reboot_notifier);

	input_info(true, &ts->client->dev, "%s %s: enter\n", HIMAX_LOG_TAG, __func__);

	himax_chip_common_suspend(ts);

	input_info(true, &ts->client->dev, "%s %s: exit\n", HIMAX_LOG_TAG, __func__);

	return NOTIFY_DONE;
}

int himax_chip_common_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;
	struct himax_ts_data *ts;

	input_info(true, &client->dev, "%s %s:Enter \n", HIMAX_LOG_TAG,
			__func__);

	/* Check I2C functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		input_err(true, &client->dev,
			"%s %s: i2c check functionality error\n",
			HIMAX_LOG_TAG, __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	if (hx_bootmode == 2) {
		input_err(true, &client->dev, "%s : Do not load driver due to : device entered recovery mode %d\n",
			__func__, hx_bootmode);
		return -ENODEV;
	}

	if (lcdtype == 0) {
		input_err(true, &client->dev, "%s %s: lcd is not attached\n",
			HIMAX_LOG_TAG, __func__);
		return -ENODEV;
	}

	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		input_err(true, &client->dev,
			"%s %s: allocate himax_ts_data failed\n",
			HIMAX_LOG_TAG, __func__);
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->dev = &client->dev;
	mutex_init(&ts->rw_lock);
	private_ts = ts;
#ifdef CONFIG_SAMSUNG_TUI
	tui_tsp_info = ts;
#endif

	ret = himax_chip_common_init();

	ts->reboot_notifier.notifier_call = himax_reboot_notifier;
	register_reboot_notifier(&ts->reboot_notifier);

err_alloc_data_failed:
err_check_functionality_failed:

	return ret;
}

int himax_chip_common_remove(struct i2c_client *client)
{
	himax_chip_common_deinit();

	return 0;
}

#ifdef CONFIG_SAMSUNG_TUI
extern int stui_i2c_lock(struct i2c_adapter *adap);
extern int stui_i2c_unlock(struct i2c_adapter *adap);

int stui_tsp_enter(void)
{
	int ret = 0;

	if (!tui_tsp_info)
		return -EINVAL;

	disable_irq(tui_tsp_info->client->irq);
	stui_report_all_leave_event(tui_tsp_info);

	ret = stui_i2c_lock(tui_tsp_info->client->adapter);
	if (ret) {
		pr_err("[STUI] stui_i2c_lock failed : %d\n", ret);
		enable_irq(tui_tsp_info->client->irq);
		return -1;
	}

	return 0;
}

int stui_tsp_exit(void)
{
	int ret = 0;

	if (!tui_tsp_info)
		return -EINVAL;

	ret = stui_i2c_unlock(tui_tsp_info->client->adapter);
	if (ret)
		pr_err("[STUI] stui_i2c_unlock failed : %d\n", ret);

	enable_irq(tui_tsp_info->client->irq);

	return ret;
}
#endif

static const struct i2c_device_id himax_common_ts_id[] = {
	{HIMAX_common_NAME, 0},
	{}
};

static const struct dev_pm_ops himax_common_pm_ops = {
#if (!defined(CONFIG_FB))
	.suspend = himax_common_suspend,
	.resume = himax_common_resume,
#endif
};

#ifdef CONFIG_OF
static struct of_device_id himax_match_table[] = {
	{.compatible = "himax,hxcommon",},
	{},
};
#else
#define himax_match_table NULL
#endif

static struct i2c_driver himax_common_driver = {
	.id_table = himax_common_ts_id,
	.probe = himax_chip_common_probe,
	.remove = himax_chip_common_remove,
	.driver = {
			.name = HIMAX_common_NAME,
			.owner = THIS_MODULE,
			.of_match_table = himax_match_table,
#ifdef CONFIG_PM
			.pm = &himax_common_pm_ops,
#endif
			},
};

static int __init himax_common_init(void)
{
#ifdef CONFIG_BATTERY_SAMSUNG
	if (lpcharge == 1) {
		E("%s %s: do not load driver due to : lpm %d\n", "[sec_input]",
			__func__, lpcharge);
		return -ENODEV;
	}
#endif
	I("%s Himax common touch panel driver init\n", "[sec_input]");
	i2c_add_driver(&himax_common_driver);

	return 0;
}

static void __exit himax_common_exit(void)
{
	i2c_del_driver(&himax_common_driver);
}

module_init(himax_common_init);
module_exit(himax_common_exit);

MODULE_DESCRIPTION("Himax_common driver");
MODULE_LICENSE("GPL");
