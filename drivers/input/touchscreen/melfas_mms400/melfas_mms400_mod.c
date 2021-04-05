/*
 * MELFAS MMS400 Touchscreen
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Model dependent functions
 *
 */

#include "melfas_mms400.h"

#ifdef USE_TSP_TA_CALLBACKS
static bool ta_connected;
#endif

/**
 * Control power supply
 */
int mms_power_control(struct mms_ts_info *info, int enable)
{
	int ret = 0;
	struct i2c_client *client = info->client;
	struct regulator *regulator_dvdd = NULL;
	struct regulator *regulator_avdd = NULL;
	struct pinctrl_state *pinctrl_state;
	static bool on;

	input_info(true, &info->client->dev, "%s [START %s]\n",
			__func__, enable ? "on":"off");

	if (on == enable) {
		input_err(true, &client->dev, "%s : TSP power already %s\n",
			__func__, (on) ? "on":"off");
		return ret;
	}

	if (info->dtdata->gpio_io_en) {
		regulator_dvdd = regulator_get(NULL, info->dtdata->gpio_io_en);
		if (IS_ERR_OR_NULL(regulator_dvdd)) {
			input_info(true, &client->dev, "%s: Failed to get %s regulator.\n",
				 __func__, info->dtdata->gpio_io_en);
			ret = PTR_ERR(regulator_dvdd);
			goto out;
		}
	}

	regulator_avdd = regulator_get(NULL, info->dtdata->gpio_vdd_en);
	if (IS_ERR_OR_NULL(regulator_avdd)) {
		input_info(true, &client->dev, "%s: Failed to get %s regulator.\n",
			 __func__, info->dtdata->gpio_vdd_en);
		ret = PTR_ERR(regulator_avdd);
		goto out;
	}

	if (enable) {
		ret = regulator_enable(regulator_avdd);
		if (ret) {
			input_info(true, &client->dev, "%s: Failed to enable avdd: %d\n", __func__, ret);
			goto out;
		}
		if (info->dtdata->gpio_io_en) {
			ret = regulator_enable(regulator_dvdd);
			if (ret) {
				input_info(true, &client->dev, "%s: Failed to enable vdd: %d\n", __func__, ret);
				goto out;
			}
		}
		pinctrl_state = pinctrl_lookup_state(info->pinctrl, "on_state");
	} else {
		if (info->dtdata->gpio_io_en) {
			if (regulator_is_enabled(regulator_dvdd))
				regulator_disable(regulator_dvdd);
		}
		if (regulator_is_enabled(regulator_avdd))
			regulator_disable(regulator_avdd);

		pinctrl_state = pinctrl_lookup_state(info->pinctrl, "off_state");
	}

	if (IS_ERR_OR_NULL(pinctrl_state)) {
		input_info(true, &client->dev, "%s: Failed to lookup pinctrl.\n", __func__);
	} else {
		ret = pinctrl_select_state(info->pinctrl, pinctrl_state);
		if (ret)
			input_info(true, &client->dev, "%s: Failed to configure pinctrl.\n", __func__);
	}

	on = enable;
out:
	if (info->dtdata->gpio_io_en && !IS_ERR_OR_NULL(regulator_dvdd))
		regulator_put(regulator_dvdd);
	if (!IS_ERR_OR_NULL(regulator_avdd))
		regulator_put(regulator_avdd);

	if (!enable)
		usleep_range(10 * 1000, 11 * 1000);
	else
		msleep(50);

	input_info(true, &info->client->dev, "%s [DONE %s]\n",
			__func__, enable ? "on":"off");
	return ret;
}

/**
 * Clear touch input events
 */
void mms_clear_input(struct mms_ts_info *info)
{
	int i;

	input_info(true, &info->client->dev, "%s\n", __func__);

	input_report_key(info->input_dev, BTN_TOUCH, 0);
	input_report_key(info->input_dev, BTN_TOOL_FINGER, 0);

	for (i = 0; i < MAX_FINGER_NUM; i++) {
		info->finger_state[i] = 0;
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
	}

	info->touch_count = 0;
	info->check_multi = 0;

	input_sync(info->input_dev);
}

/**
 * Input event handler - Report touch input event
 */
void mms_input_event_handler(struct mms_ts_info *info, u8 sz, u8 *buf)
{
	struct i2c_client *client = info->client;
	int i;

	input_dbg(true, &client->dev, "%s [START]\n", __func__);
	input_dbg(true, &client->dev, "%s - sz[%d] buf[0x%02X]\n", __func__, sz, buf[0]);

	for (i = 0; i < sz; i += info->event_size) {
		u8 *tmp = &buf[i];

		int id = (tmp[0] & MIP_EVENT_INPUT_ID) - 1;
		int x = tmp[2] | ((tmp[1] & 0xf) << 8);
		int y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);
		int pressure = tmp[4];
		//int size = tmp[5];		// sumsize
		int touch_major = tmp[6];
		int touch_minor = tmp[7];

		int palm = (tmp[0] & MIP_EVENT_INPUT_PALM) >> 4;

		// Report input data
#if MMS_USE_TOUCHKEY
		if ((tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0) {
			//Touchkey Event
			int key = tmp[0] & 0xf;
			int key_state = (tmp[0] & MIP_EVENT_INPUT_PRESS) ? 1 : 0;
			int key_code = 0;

			//Report touchkey event
			switch (key) {
			case 1:
				key_code = KEY_MENU;
				//input_dbg(true, &client->dev, "Key : KEY_MENU\n");
				break;
			case 2:
				key_code = KEY_BACK;
				//input_dbg(true, &client->dev, "Key : KEY_BACK\n");
				break;
			default:
				input_err(true, &client->dev,
					"%s [ERROR] Unknown key code [%d]\n",
					__func__, key);
				continue;
				break;
			}

			input_report_key(info->input_dev, key_code, key_state);

			input_dbg(true, &client->dev, "%s - Key : ID[%d] Code[%d] State[%d]\n",
				__func__, key, key_code, key_state);
		} else
#endif
		{
			//Report touchscreen event
			if ((tmp[0] & MIP_EVENT_INPUT_PRESS) == 0) {
				//Release
				input_mt_slot(info->input_dev, id);
#ifdef CONFIG_SEC_FACTORY
				input_report_abs(info->input_dev, ABS_MT_PRESSURE, 0);
#endif
				input_mt_report_slot_state(info->input_dev,
								MT_TOOL_FINGER, false);
				if (info->finger_state[id] != 0) {
					info->touch_count--;
					if (!info->touch_count) {
						input_report_key(info->input_dev, BTN_TOUCH, 0);
						input_report_key(info->input_dev,
									BTN_TOOL_FINGER, 0);
						info->check_multi = 0;
					}
					info->finger_state[id] = 0;
					input_info(true, &client->dev,
						"R[%d] V[%02x%02x%02x] tc:%d\n",
						id, info->boot_ver_ic, info->core_ver_ic,
						info->config_ver_ic, info->touch_count);
				}

				continue;
			}

			//Press or Move
			input_mt_slot(info->input_dev, id);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
			input_report_key(info->input_dev, BTN_TOUCH, 1);
			input_report_key(info->input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
#ifdef CONFIG_SEC_FACTORY
			if (pressure)
				input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
			else
				input_report_abs(info->input_dev, ABS_MT_PRESSURE, 1);
#endif
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MINOR, touch_minor);
			input_report_abs(info->input_dev, ABS_MT_PALM, palm);
			if (info->finger_state[id] == 0) {
				info->finger_state[id] = 1;
				info->touch_count++;
#ifdef CONFIG_SAMSUNG_PRODUCT_SHIP
				input_info(true, &client->dev,
					"P[%d] z:%d p:%d m:%d,%d tc:%d\n",
					id, pressure, palm, touch_major, touch_minor, info->touch_count);
#else
				input_err(true, &client->dev,
					"P[%d] (%d, %d) z:%d p:%d m:%d,%d tc:%d\n",
					id, x, y, pressure, palm, touch_major, touch_minor, info->touch_count);
#endif
				if ((info->touch_count > 2) && (info->check_multi == 0)) {
					info->check_multi = 1;
					info->multi_count++;
				}
			}
		}
	}

	input_sync(info->input_dev);
	input_dbg(true, &client->dev, "%s [DONE]\n", __func__);
}

#if MMS_USE_DEVICETREE
/**
 * Parse device tree
 */
int mms_parse_devicetree(struct device *dev, struct mms_ts_info *info)
{
	struct device_node *np = dev->of_node;
	int ret;

	input_info(true, dev, "%s [START]\n", __func__);

	ret = of_property_read_u32(np, "melfas,max_x", &info->dtdata->max_x);
	if (ret) {
		input_err(true, dev, "%s [ERROR] max_x\n", __func__);
		info->dtdata->max_x = 720;
	}

	ret = of_property_read_u32(np, "melfas,max_y", &info->dtdata->max_y);
	if (ret) {
		input_err(true, dev, "%s [ERROR] max_y\n", __func__);
		info->dtdata->max_y = 1280;
	}

	info->dtdata->gpio_intr = of_get_named_gpio(np, "melfas,irq-gpio", 0);
	gpio_request(info->dtdata->gpio_intr, "irq-gpio");
	gpio_direction_input(info->dtdata->gpio_intr);
	info->client->irq = gpio_to_irq(info->dtdata->gpio_intr);

	info->dtdata->gpio_scl = of_get_named_gpio(np, "melfas,scl-gpio", 0);
	gpio_request(info->dtdata->gpio_scl, "melfas_scl_gpio");
	info->dtdata->gpio_sda = of_get_named_gpio(np, "melfas,sda-gpio", 0);
	gpio_request(info->dtdata->gpio_sda, "melfas_sda_gpio");

	if (of_property_read_string(np, "melfas,vdd_en", &info->dtdata->gpio_vdd_en))
		input_err(true, dev,  "Failed to get regulator_dvdd name property\n");

	if (of_property_read_string(np, "melfas,io_en", &info->dtdata->gpio_io_en)) {
		input_err(true, dev, "Failed to get regulator_avdd name property\n");
		info->dtdata->gpio_io_en = NULL;
	}

	if (of_property_read_u32(np, "melfas,fw-skip", &info->dtdata->fw_update_skip) >= 0)
		input_info(true, dev, "%s() melfas,fw-skip: %d\n", __func__, info->dtdata->fw_update_skip);

	if (of_property_read_string(np, "melfas,fw_name", &info->dtdata->fw_name))
		input_err(true, dev, "Failed to get fw_name property\n");

	info->dtdata->support_lpm = of_property_read_bool(np, "melfas,support_lpm");

	if (of_property_read_u32(np, "melfas,factory_item_version", &info->dtdata->item_version) < 0)
		info->dtdata->item_version = 0;

	input_info(true, dev, "%s: fw_name %s max_x:%d max_y:%d int:%d irq:%d sda:%d scl:%d support_LPM:%d\n",
		__func__, info->dtdata->fw_name, info->dtdata->max_x, info->dtdata->max_y,
		info->dtdata->gpio_intr, info->client->irq, info->dtdata->gpio_sda,
		info->dtdata->gpio_scl, info->dtdata->support_lpm);


	return 0;
}
#endif

/**
 * Config input interface
 */
void mms_config_input(struct mms_ts_info *info)
{
	struct input_dev *input_dev = info->input_dev;

	input_dbg(true, &info->client->dev, "%s [START]\n", __func__);

	//Screen
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);

	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, MAX_FINGER_NUM, INPUT_MT_DIRECT);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
#ifdef CONFIG_SEC_FACTORY
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, INPUT_TOUCH_MAJOR_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0, INPUT_TOUCH_MINOR_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PALM, 0, 1, 0, 0);

	//Key
	set_bit(EV_KEY, input_dev->evbit);
#if MMS_USE_TOUCHKEY
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
#endif
#if MMS_USE_NAP_MODE
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_POWER, input_dev->keybit);
#endif
	set_bit(KEY_BLACK_UI_GESTURE, input_dev->keybit);
	input_dbg(true, &info->client->dev, "%s [DONE]\n", __func__);
}

#ifdef CONFIG_VBUS_NOTIFIER
int mms_vbus_notification(struct notifier_block *nb,
		unsigned long cmd, void *data)
{
	struct mms_ts_info *info = container_of(nb, struct mms_ts_info, vbus_nb);
	vbus_status_t vbus_type = *(vbus_status_t *)data;

	input_info(true, &info->client->dev, "%s cmd=%lu, vbus_type=%d\n", __func__, cmd, vbus_type);

	switch (vbus_type) {
	case STATUS_VBUS_HIGH:
		input_info(true, &info->client->dev, "%s : attach\n", __func__);
		info->ta_stsatus = true;
		break;
	case STATUS_VBUS_LOW:
		input_info(true, &info->client->dev, "%s : detach\n", __func__);
		info->ta_stsatus = false;
		break;
	default:
		break;
	}

	if (!info->enabled) {
		input_err(true, &info->client->dev, "%s tsp disabled", __func__);
		return 0;
	}

	mms_charger_attached(info, info->ta_stsatus);
	return 0;
}
#endif

/**
 * Callback - get charger status
 */
#ifdef USE_TSP_TA_CALLBACKS
void mms_charger_status_cb(struct tsp_callbacks *cb, int status)
{
	pr_info("%s: TA %s\n",
		__func__, status ? "connected" : "disconnected");

	if (status)
		ta_connected = true;
	else
		ta_connected = false;

	/* not yet defined functions */
}

void mms_register_callback(struct tsp_callbacks *cb)
{
	charger_callbacks = cb;
	pr_info("%s\n", __func__);
}
#endif

int mms_lowpower_mode(struct mms_ts_info *info, int on)
{
	u8 wbuf[3];
	u8 rbuf[1];

	if (!info->dtdata->support_lpm) {
		input_err(true, &info->client->dev, "%s not supported\n", __func__);
		return -EINVAL;
	}

	/**	bit	Power state
	 *	0	active
	 *	1	low power consumption
	 */

	wbuf[0] = MIP_R0_CTRL;
	wbuf[1] = MIP_R1_CTRL_POWER_STATE;
	wbuf[2] = on;

	if (mms_i2c_write(info, wbuf, 3)) {
		input_err(true, &info->client->dev, "%s [ERROR] write %x %x %x\n",
				__func__, wbuf[0], wbuf[1], wbuf[2]);
		return -EIO;
	}

	msleep(20);

	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		input_err(true, &info->client->dev, "%s [ERROR] read %x %x, rbuf %x\n",
				__func__, wbuf[0], wbuf[1], rbuf[0]);
		return -EIO;
	}

	if (rbuf[0] != on) {
		input_err(true, &info->client->dev, "%s [ERROR] not changed to %s mode, rbuf %x\n",
				__func__, on ? "LPM" : "normal", rbuf[0]);
		return -EIO;
	}

	// set AOD or SPAY bit
	wbuf[0] = MIP_R0_AOT;
	wbuf[1] = MIP_R0_AOT_CTRL;
	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		input_err(true, &info->client->dev, "%s [ERROR] read %x %x, rbuf %x\n",
				__func__, wbuf[0], wbuf[1], rbuf[0]);
		return -EIO;
	}

	input_info(true, &info->client->dev, "%s: AOT ctrl register=%x, flag=%x\n",
				__func__, rbuf[0], info->lowpower_flag);

	wbuf[2] = rbuf[0] & (info->lowpower_flag << 1);

	wbuf[0] = MIP_R0_AOT;
	wbuf[1] = MIP_R0_AOT_CTRL;

	if (mms_i2c_write(info, wbuf, 3)) {
		input_err(true, &info->client->dev, "%s [ERROR] write %x %x %x\n",
				__func__, wbuf[0], wbuf[1], wbuf[2]);
		return -EIO;
	}

	msleep(20);

	if (mms_i2c_read(info, wbuf, 2, rbuf, 1)) {
		input_err(true, &info->client->dev, "%s [ERROR] read %x %x, rbuf %x\n",
				__func__, wbuf[0], wbuf[1], rbuf[0]);
		return -EIO;
	}

	input_info(true, &info->client->dev, "%s: AOT ctrl register=%x\n", __func__, rbuf[0]);

	if (rbuf[0] != wbuf[2]) {
		input_err(true, &info->client->dev, "%s [ERROR] not changed to %x mode, rbuf %x\n",
				__func__, wbuf[2], rbuf[0]);
		return -EIO;
	}

	input_info(true, &info->client->dev, "%s: %s mode\n", __func__, on ? "LPM" : "normal");
	return 0;
}
