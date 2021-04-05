/*
 * leds-s2mu005.c - LED class driver for S2MU005 LEDs.
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mfd/samsung/s2mu005.h>
#include <linux/mfd/samsung/s2mu005-private.h>
#include <linux/leds-s2mu005.h>
#include <linux/platform_device.h>
#include <linux/sec_batt.h>

extern struct class *camera_class;
struct device *flash_dev;
bool assistive_light = false;
#ifdef CONFIG_LEDS_SUPPORT_FRONT_FLASH
bool front_assistive_light = false;
int fled_selected_ch = S2MU005_FLED_OFF;
#endif
struct s2mu005_led_data * g_led_datas[S2MU005_LED_MAX];
#define LED_TURN_OFF -1
#define S2MU005_FLED_DEBUG

static u8 leds_cur_max[] = {
	S2MU005_FLASH_OUT_I_1200MA,
	S2MU005_TORCH_OUT_I_400MA,
};

static u8 leds_time_max[] = {
	S2MU005_FLASH_TIMEOUT_992MS,
	S2MU005_TORCH_TIMEOUT_15728MS,
};

struct s2mu005_led_data {
	struct led_classdev cdev;
	struct s2mu005_led *data;
	struct notifier_block batt_nb;
	struct i2c_client *i2c;
	struct work_struct work;
	struct mutex lock;
	spinlock_t value_lock;
	int brightness;
	int test_brightness;
	int attach_ta;
	int attach_sdp;
	bool enable;
	int torch_pin;
	int flash_pin;
	unsigned int flash_brightness;
	unsigned int preflash_brightness;
	unsigned int movie_brightness;
	unsigned int torch_brightness;
	unsigned int factory_brightness;
#if defined(CONFIG_LEDS_SUPPORT_FRONT_FLASH)
	unsigned int front_brightness;
#endif
	unsigned int flashlight_current[S2MU005_FLASH_LIGHT_MAX];
};

u8 CH_FLASH_TORCH_EN = S2MU005_REG_FLED_RSVD;

#ifdef CONFIG_MUIC_NOTIFIER
static void attach_cable_check(muic_attached_dev_t attached_dev,
					int *attach_ta, int *attach_sdp)
{
	if (attached_dev == ATTACHED_DEV_USB_MUIC)
		*attach_sdp = 1;
	else
		*attach_sdp = 0;

	switch (attached_dev) {
	case ATTACHED_DEV_TA_MUIC:
	case ATTACHED_DEV_SMARTDOCK_TA_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_TA_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_TA_MUIC:
	case ATTACHED_DEV_CDP_MUIC:
	case ATTACHED_DEV_USB_MUIC:
	case ATTACHED_DEV_UNOFFICIAL_ID_CDP_MUIC:
		*attach_ta = 1;
		break;
	default:
		*attach_ta = 0;
		break;
	}
}

static int ta_notification(struct notifier_block *nb,
		unsigned long action, void *data)
{
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
	u8 temp;

	int ret = 0;
	struct s2mu005_led_data *led_data =
		container_of(nb, struct s2mu005_led_data, batt_nb);

	switch (action) {
	case MUIC_NOTIFY_CMD_DETACH:
	case MUIC_NOTIFY_CMD_LOGICALLY_DETACH:
		if (!led_data->attach_ta)
			goto err;

		led_data->attach_ta = 0;

		if (!led_data->data->id) {
			pr_info("%s : flash mode\n", __func__);
			goto err;
		}
#ifndef CONFIG_S2MU005_LEDS_I2C
		if (gpio_is_valid(led_data->torch_pin)) {
			ret = devm_gpio_request(led_data->cdev.dev,
					led_data->torch_pin, "s2mu005_gpio");
			if (ret) {
				pr_err("%s : fail to assignment gpio\n",
								__func__);
				goto gpio_free_data;
			}
		}
		if (gpio_get_value(led_data->torch_pin)) {
			gpio_direction_output(led_data->torch_pin, 0);
			gpio_direction_output(led_data->torch_pin, 1);
			goto gpio_free_data;
		}
#else
		s2mu005_read_reg(led_data->i2c,
				CH_FLASH_TORCH_EN, &temp);
		if ((temp & S2MU005_CH1_TORCH_ON_I2C) == S2MU005_CH1_TORCH_ON_I2C) {
			ret = s2mu005_update_reg(led_data->i2c,
				CH_FLASH_TORCH_EN,
				S2MU005_FLASH_TORCH_OFF,
				S2MU005_CH1_TORCH_ENABLE_MASK);

			pr_info("%s : LED OFF\n", __func__);
			if (ret < 0)
				goto err;

			//msleep(10);

			ret = s2mu005_update_reg(led_data->i2c,
				CH_FLASH_TORCH_EN,
				S2MU005_CH1_TORCH_ON_I2C,
				S2MU005_CH1_TORCH_ENABLE_MASK);

			pr_info("%s : LED ON\n", __func__);
			if (ret < 0)
				goto err;
		}
#endif

		if (!factory_mode) {
			/* CHGIN_ENGH = 0 */
			ret = s2mu005_update_reg(led_data->i2c,
				S2MU005_REG_FLED_CTRL1, 0x00, 0x80);
			if (ret < 0)
				goto err;
		}

		break;
	case MUIC_NOTIFY_CMD_ATTACH:
	case MUIC_NOTIFY_CMD_LOGICALLY_ATTACH:
		led_data->attach_ta = 0;
		attach_cable_check(attached_dev, &led_data->attach_ta,
						&led_data->attach_sdp);
		if (led_data->attach_ta) {
			s2mu005_read_reg(led_data->i2c,
				S2MU005_REG_FLED_STATUS, &temp);

			/* if CH1_TORCH_ON or CH2_TORCH_ON setting CHGIN_ENGH bit 1 */
			if (temp & 0x50) {
				 ret = s2mu005_update_reg(led_data->i2c,
					S2MU005_REG_FLED_CTRL1, 0x80, 0x80);
				if (ret < 0)
					goto err;
			}
		}

		return 0;
	default:
		goto err;
		break;
	}

#ifndef CONFIG_S2MU005_LEDS_I2C
gpio_free_data:
	gpio_free(led_data->torch_pin);
	pr_info("%s : gpio free\n", __func__);
#endif
	pr_info("%s : complete detached\n", __func__);
	return 0;
err:
	pr_err("%s : abandond access %d\n", __func__, led_data->attach_ta);
	return 0;
}
#endif

static void torch_led_on_off(int value)
{
	int ret;
	u8 temp;

	pr_info("%s : value(%d), attach_ta(%d)\n",
		__func__, value, g_led_datas[S2MU005_FLASH_LED]->attach_ta);

	if (value && g_led_datas[S2MU005_FLASH_LED]->attach_ta) { //torch on & ta attach
		ret = s2mu005_update_reg(g_led_datas[S2MU005_FLASH_LED]->i2c,
			S2MU005_REG_FLED_CTRL1, 0x80, 0x80);
		if (ret < 0)
			pr_err("%s : CHGIN_ENGH = 1 fail\n", __func__);
	}

	if ((value == 0) && !factory_mode) { // torch off
		s2mu005_read_reg(g_led_datas[S2MU005_FLASH_LED]->i2c,
			S2MU005_REG_FLED_CTRL1, &temp);
		pr_info("%s : 0x%2X read value - 0x%2X\n", __func__,
			S2MU005_REG_FLED_CTRL1, temp);
		if ((temp & 0x80) == 0x80) {
			ret = s2mu005_update_reg(g_led_datas[S2MU005_FLASH_LED]->i2c,
				S2MU005_REG_FLED_CTRL1, 0x00, 0x80);
			if (ret < 0)
				pr_err("%s : CHGIN_ENGH = 0 fail\n", __func__);
		}
	}
}

static void led_set(struct s2mu005_led_data *led_data)
{
	int ret;
	struct s2mu005_led *data = led_data->data;
	int id = data->id;
	u8 mask = 0, reg = 0;

#ifdef CONFIG_S2MU005_LEDS_I2C
	u8 enable_mask = 0, value = 0;
#else
	int gpio_pin;
#endif
	if (id == S2MU005_FLASH_LED) {
		pr_info("%s led mode is flash\n", __func__);
		reg = S2MU005_REG_FLED_CH1_CTRL0;
		mask = S2MU005_FLASH_IOUT_MASK;
#ifndef CONFIG_S2MU005_LEDS_I2C
		pr_info("%s gpio_flash mode\n", __func__);
		gpio_pin = led_data->flash_pin;
#endif
	} else {
		pr_info("%s led mode is torch\n", __func__);
#ifdef CONFIG_LEDS_SUPPORT_FRONT_FLASH
		if (front_assistive_light) {
			reg = S2MU005_REG_FLED_CH2_CTRL1;
			mask = S2MU005_TORCH_IOUT_MASK;
		} else
#endif
		{
			reg = S2MU005_REG_FLED_CH1_CTRL1;
			mask = S2MU005_TORCH_IOUT_MASK;
		}
#ifndef CONFIG_S2MU005_LEDS_I2C
		pr_info("%s gpio_torch mode\n", __func__);
		gpio_pin = led_data->torch_pin;
#endif
	}


#ifndef CONFIG_S2MU005_LEDS_I2C
	if (gpio_is_valid(gpio_pin)) {
		ret = devm_gpio_request(led_data->cdev.dev, gpio_pin,
				"s2mu005_gpio");
		if (ret) {
			pr_err("%s : fail to assignment gpio\n", __func__);
			goto gpio_free_data;
		}
	}
#endif
	pr_info("%s start led_set\n", __func__);

	if (led_data->test_brightness == LED_TURN_OFF) {
#ifdef CONFIG_S2MU005_LEDS_I2C
		ret = s2mu005_update_reg(led_data->i2c, reg, 0, mask);
		if (ret < 0)
			goto error_set_bits;
#else
		ret = s2mu005_update_reg(led_data->i2c, reg,
				led_data->data->brightness, mask);
		if (ret < 0)
			goto error_set_bits;

		gpio_direction_output(gpio_pin, 0);
#endif
		/* torch mode off sequence */
		if (id && led_data->attach_ta) {
			if (!factory_mode) {
				ret = s2mu005_update_reg(led_data->i2c,
					S2MU005_REG_FLED_CTRL1, 0x00, 0x80);
				if (ret < 0)
					goto error_set_bits;
			}
		}
#ifndef CONFIG_S2MU005_LEDS_I2C
		goto gpio_free_data;
#endif
	} else {
		pr_info("%s led on\n", __func__);
		/* torch mode on sequence */
		if (id && led_data->attach_ta) {
			ret = s2mu005_update_reg(led_data->i2c,
				S2MU005_REG_FLED_CTRL1, 0x80, 0x80);
			if (ret < 0)
				goto error_set_bits;
			/* ta attach & sdp mode :  brightness limit 300mA */
			if (led_data->attach_sdp)
				led_data->test_brightness =
					(led_data->test_brightness > S2MU005_TORCH_OUT_I_300MA) ?
					S2MU005_TORCH_OUT_I_300MA : led_data->test_brightness;
		}
		pr_info("%s led brightness = %d\n", __func__, led_data->test_brightness);
		ret = s2mu005_update_reg(led_data->i2c,
				reg, led_data->test_brightness, mask);
		if (ret < 0)
			goto error_set_bits;

#ifdef CONFIG_S2MU005_LEDS_I2C
#ifdef CONFIG_LEDS_SUPPORT_FRONT_FLASH
		if(front_assistive_light)
			value = id ? S2MU005_CH2_TORCH_ON_I2C : S2MU005_CH2_FLASH_ON_I2C;
		else
#endif
			value = id ? S2MU005_CH1_TORCH_ON_I2C : S2MU005_CH1_FLASH_ON_I2C;
#else
		gpio_direction_output(gpio_pin, 1);
		goto gpio_free_data;

#endif
	}
#ifdef CONFIG_S2MU005_LEDS_I2C
#ifdef CONFIG_LEDS_SUPPORT_FRONT_FLASH
	if(front_assistive_light)
		enable_mask = id ? S2MU005_CH2_TORCH_ENABLE_MASK : S2MU005_CH2_FLASH_ENABLE_MASK;
	else
#endif
		enable_mask = id ? S2MU005_CH1_TORCH_ENABLE_MASK : S2MU005_CH1_FLASH_ENABLE_MASK;

	ret = s2mu005_update_reg(led_data->i2c, CH_FLASH_TORCH_EN, value, enable_mask);

	if (ret < 0)
		goto error_set_bits;
#endif
	return;

#ifndef CONFIG_S2MU005_LEDS_I2C
gpio_free_data:
	gpio_free(gpio_pin);
	pr_info("%s : gpio free\n", __func__);
	return;
#endif
error_set_bits:
	pr_err("%s: can't set led level %d\n", __func__, ret);
	return;
}

static void s2mu005_led_set(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	unsigned long flags;
	struct s2mu005_led_data *led_data =
		container_of(led_cdev, struct s2mu005_led_data, cdev);
	u8 max;

	max = led_cdev->max_brightness;

	pr_info("%s value = %d, max = %d\n", __func__, value, max);

	spin_lock_irqsave(&led_data->value_lock, flags);
	led_data->test_brightness = min_t(int, (int)value, (int)max);
	spin_unlock_irqrestore(&led_data->value_lock, flags);

//	schedule_work(&led_data->work);
	led_set(led_data);
	return;
}

static void s2mu005_led_work(struct work_struct *work)
{
	struct s2mu005_led_data *led_data
		= container_of(work, struct s2mu005_led_data, work);

	pr_debug("%s [led]\n", __func__);

	mutex_lock(&led_data->lock);
	led_set(led_data);
	mutex_unlock(&led_data->lock);
}

static int s2mu005_led_setup(struct s2mu005_led_data *led_data)
{
	int ret = 0;
	int mask, value;
	u8 temp;

	ret = s2mu005_read_reg(led_data->i2c, 0x73, &temp);	/* EVT0 0x73[3:0] == 0x0 */
	if (ret < 0)
		goto out;

	if ((temp & 0xf) == 0x00) {
		/* forced BATID recognition 0x89[1:0] = 0x3 */
		ret = s2mu005_update_reg(led_data->i2c, 0x89, 0x03, 0x03);
		if (ret < 0)
			goto out;
		ret = s2mu005_update_reg(led_data->i2c, 0x92, 0x80, 0x80);
		if (ret < 0)
			goto out;
	}

	/* Controlled Channel1, Channel2 independently */
	ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CTRL2,
			0x00, S2MU005_EN_CHANNEL_SHARE_MASK);
	if (ret < 0)
		goto out;

	/* Boost vout flash 4.5V */
	ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CTRL2,
			0x0A, S2MU005_BOOST_VOUT_FLASH_MASK);
	if (ret < 0)
		goto out;

	/* FLED_BOOST_EN */
	ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CTRL1,
			0x40, S2MU005_FLASH_BOOST_EN_MASK);
	if (ret < 0)
		goto out;

	/* Flash timer Maximum mode */
	ret = s2mu005_update_reg(led_data->i2c,
			S2MU005_REG_FLED_CH1_CTRL3, 0x80, 0x80);
	if (ret < 0)
		goto out;

	if (led_data->data->id == S2MU005_FLASH_LED) {
		/* flash timer Maximum set */
		ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH1_CTRL3,
				led_data->data->timeout, S2MU005_TIMEOUT_MAX);
		if (ret < 0)
			goto out;
	} else {
		/* torch timer Maximum set */
		ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH1_CTRL2,
				led_data->data->timeout, S2MU005_TIMEOUT_MAX);
		if (ret < 0)
			goto out;
	}

	/* flash brightness set */
	ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH1_CTRL0,
			led_data->flash_brightness, S2MU005_FLASH_IOUT_MASK);
	if (ret < 0)
		goto out;

	/* torch brightness set */
	ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH1_CTRL1,
			led_data->preflash_brightness, S2MU005_TORCH_IOUT_MASK);
	if (ret < 0)
		goto out;

	/* factory mode additional setting */
	pr_info("%s : factory_mode %d \n", __func__, factory_mode);
	if (factory_mode) {
		ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CTRL1,
						0x80, 0x80);
		if (ret < 0)
			goto out;
		ret = s2mu005_update_reg(led_data->i2c, 0xAC,0x40, 0x40);
		if (ret < 0)
			goto out;
	}

#ifdef CONFIG_S2MU005_LEDS_I2C
	value =	S2MU005_FLASH_TORCH_OFF;
#else
	value = S2MU005_CH1_FLASH_ON_GPIO | S2MU005_CH1_TORCH_ON_GPIO;
#endif
	mask = S2MU005_CH1_TORCH_ENABLE_MASK | S2MU005_CH1_FLASH_ENABLE_MASK
		| S2MU005_CH2_TORCH_ENABLE_MASK;
	ret = s2mu005_update_reg(led_data->i2c, CH_FLASH_TORCH_EN,
		value, mask);
	if (ret < 0)
		goto out;
	pr_info("%s : led setup complete\n", __func__);
	return ret;

out:
	pr_err("%s : led setup fail\n", __func__);
	return ret;
}

#ifdef S2MU005_FLED_DEBUG
int s2mu005_led_dump_reg(void)
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_TORCH_LED];
	int i =0;
	u8 temp;

	pr_info("[LED] S2MU005 FLED DEBUG : S\n");

	for (i = 0x2D; i <= 0x3C; i++) {
		s2mu005_read_reg(led_data->i2c, i, &temp);
		pr_info("[LED] 0x%02X : 0x%02X \n", i, temp);
	}

	pr_info("[LED] S2MU005 FLED DEBUG : X\n");

	return 0;
}
#endif


#ifdef CONFIG_CAMERA_USE_SOC_SENSOR
int s2mu005_led_mode_ctrl(int state)
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_FLASH_LED];
	int gpio_torch = led_data->torch_pin;
	int gpio_flash = led_data->flash_pin;

	pr_info("%s : state = %d\n", __func__, state);

	if (assistive_light == true) {
		pr_info("%s : assistive_light is enabled \n", __func__);
		return 0;
	}

	devm_gpio_request(led_data->cdev.dev, gpio_torch,
				"s2mu005_gpio_torch");

	devm_gpio_request(led_data->cdev.dev, gpio_flash,
				"s2mu005_gpio_flash");

	switch(state) {
		case S2MU005_FLED_MODE_OFF:
			gpio_direction_output(gpio_torch, 0);
			gpio_direction_output(gpio_flash, 0);
			torch_led_on_off(0);
			break;
		case S2MU005_FLED_MODE_PREFLASH:
			gpio_direction_output(gpio_torch, 1);
			break;
		case S2MU005_FLED_MODE_FLASH:
			gpio_direction_output(gpio_flash, 1);
			break;
		case S2MU005_FLED_MODE_MOVIE:
			gpio_direction_output(gpio_torch, 1);
			torch_led_on_off(1);
			break;
		default:
			break;
	}

	gpio_free(gpio_torch);
	gpio_free(gpio_flash);

	return 0;
}
#elif defined(CONFIG_LEDS_SUPPORT_FRONT_FLASH)
int s2mu005_led_mode_ctrl(int mode)
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_TORCH_LED];
	int value = 0;
	int brightness = 0;
	int ret = 0;
	if (assistive_light == true) {
		pr_info("%s : assistive_light is enabled \n", __func__);
		return 0;
	 }

	pr_info("%s : fled_selected_ch(%d), mode = %d\n", __func__,fled_selected_ch, mode);
	if (fled_selected_ch == S2MU005_FLED_CH1) {
		/* Rear Camera use gpio control, because of capture timing */
		if (mode == S2MU005_FLED_MODE_MOVIE)
			torch_led_on_off(1);
		else if (mode == S2MU005_FLED_MODE_OFF)
			torch_led_on_off(0);

		return 0;
	}

	switch(mode) {
		case S2MU005_FLED_MODE_OFF:
			/* Turn off Torch */
			brightness = 0;
			value = S2MU005_FLASH_TORCH_OFF;
			break;
		case S2MU005_FLED_MODE_PREFLASH:
			break;
		case S2MU005_FLED_MODE_FLASH:
			brightness = led_data->front_brightness;
			value = S2MU005_CH2_TORCH_ON_I2C;
			break;
		case S2MU005_FLED_MODE_MOVIE:
			torch_led_on_off(1);
			brightness = led_data->front_brightness;
			value = S2MU005_CH2_TORCH_ON_I2C;
			break;
		default:
			break;
	}
#if !defined(CONFIG_LEDS_SUPPORT_FRONT_FLASH)
	ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH2_CTRL1,
				brightness, S2MU005_TORCH_IOUT_MASK);
	if (ret < 0)
		goto error_set_bits;
#endif
#ifdef CONFIG_S2MU005_LEDS_I2C
	ret = s2mu005_write_reg(led_data->i2c, CH_FLASH_TORCH_EN, value);
	if (ret < 0)
		goto error_set_bits;
#else
	value = S2MU005_CH2_TORCH_ON_GPIO;
	s2mu005_write_reg(led_data->i2c, CH_FLASH_TORCH_EN, value);
#endif

	if (mode == S2MU005_FLED_MODE_OFF)
		torch_led_on_off(0);

	return 0;

error_set_bits:
	pr_err("%s: can't set led level %d\n", __func__, ret);
	return ret;
}
#else
int s2mu005_led_mode_ctrl(int state)
{

	pr_info("%s : state = %d\n", __func__, state);

	if (assistive_light == true) {
		pr_info("%s : assistive_light is enabled \n", __func__);
		return 0;
	}

	switch(state) {
		case S2MU005_FLED_MODE_OFF:
			torch_led_on_off(0);
			break;
		case S2MU005_FLED_MODE_PREFLASH:
			break;
		case S2MU005_FLED_MODE_FLASH:
			break;
		case S2MU005_FLED_MODE_MOVIE:
			torch_led_on_off(1);
			break;
		default:
			break;
	}

	return 0;
}
#endif

#ifdef CONFIG_LEDS_SUPPORT_FRONT_FLASH
int s2mu005_led_set_front_flash_brightness(int brightness)	/*For control brightness of front flash led*/
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_TORCH_LED];
	int ret = 0;
	pr_emerg("[s]g %s begin\n", __func__);

	//printk("Change Frontflash LED receive br = %d  read data = %d\n", brightness, led_data->front_brightness);

	led_data->front_brightness = brightness;

	if (brightness >= 25) {
		pr_info("[s]g %s brightness : %d\n", __func__, brightness);
		ret = s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH2_CTRL1,
					S2MU005_TORCH_BRIGHTNESS(brightness), S2MU005_TORCH_IOUT_MASK);
		if (ret < 0)
			goto error_set_bits;
	} else {
		pr_info("[s]g %s brightness not changed %d\n", __func__, brightness);
	}
	pr_info("[s]g %s end %d\n", __func__, S2MU005_TORCH_BRIGHTNESS(brightness));
	return 0;

error_set_bits:
	pr_err("%s: can't set led level %d\n", __func__, ret);

	return ret;
}

int s2mu005_led_select_ctrl(int ch)
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_FLASH_LED];
	int value = 0;

	pr_info("%s : selected(%d) %s\n", __func__, ch, ch == S2MU005_FLED_CH1 ? "FLED1" :
									(ch == S2MU005_FLED_CH2 ?  "FLED2" : "OFF"));

	fled_selected_ch = ch;
	if (assistive_light == true) {
		pr_info("%s : assistive_light is enabled \n", __func__);
		/* make default setting of FLED2, when front camera-off with front torch-on */
		s2mu005_update_reg(led_data->i2c, CH_FLASH_TORCH_EN,
			S2MU005_FLASH_TORCH_OFF, S2MU005_CH2_TORCH_ENABLE_MASK);

		return 0;
	}

	mutex_lock(&led_data->lock);

	if (ch == S2MU005_FLED_CH1) {
		value = S2MU005_CH1_TORCH_ON_GPIO | S2MU005_CH1_FLASH_ON_GPIO;
		s2mu005_write_reg(led_data->i2c, CH_FLASH_TORCH_EN, value);
		/* brightness set - Rear pre-flash*/
		s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH1_CTRL1,
			led_data->preflash_brightness, S2MU005_TORCH_IOUT_MASK);
	} else if (ch == S2MU005_FLED_CH2) {
		/* brightness set - front torch*/
#if !defined(CONFIG_LEDS_SUPPORT_FRONT_FLASH)
		s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH2_CTRL1,
			led_data->front_brightness, S2MU005_TORCH_IOUT_MASK);
#endif
	} else {
		value = S2MU005_FLASH_TORCH_OFF;
		s2mu005_write_reg(led_data->i2c, CH_FLASH_TORCH_EN, value);
		/* brightness set - Rear pre-flash(default)*/
		s2mu005_update_reg(led_data->i2c, S2MU005_REG_FLED_CH1_CTRL1,
			led_data->preflash_brightness, S2MU005_TORCH_IOUT_MASK);

#ifdef S2MU005_FLED_DEBUG
		s2mu005_led_dump_reg();
#endif
	}

	mutex_unlock(&led_data->lock);
	return 0;
}
#endif

static ssize_t rear_flash_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_TORCH_LED];
	char *str;

	switch (led_data->data->id) {
	case S2MU005_FLASH_LED:
		str = "FLASH";
		break;
	case S2MU005_TORCH_LED:
		str = "TORCH";
		break;
	default:
		str = "NONE";
		break;
	}

	return snprintf(buf, 20, "%s\n", str);
}

static ssize_t rear_flash_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_TORCH_LED];
	struct led_classdev *led_cdev = &led_data->cdev;
	int value = 0;
	int reg_val = 0;
	int brightness = 0;
	u32 torch_current;

	if ((buf == NULL) || kstrtouint(buf, 10, &value)) {
		return -1;
	}

	pr_info("[LED]%s , value:%d\n", __func__, value);
	mutex_lock(&led_data->lock);

	if (led_data->data->id == S2MU005_FLASH_LED) {
		pr_info("%s : flash is not controlled by sysfs", __func__);
		goto err;
	}

	if (value == 0) {
		/* Turn off Torch */
		brightness = LED_TURN_OFF;
		assistive_light = false;
	} else if (value == 1) {
		/* Turn on Torch */
		brightness = led_data->torch_brightness;
		assistive_light = true;
	} else if (value == 100) {
		/* Factory mode Turn on Torch */
		brightness = led_data->factory_brightness;
		assistive_light = true;
	} else if (value == 200) {
		/* Factory mode Turn on Torch */
		brightness = led_data->factory_brightness;
		assistive_light = true;
	} else if (1001 <= value && value <= 1010) {
		/* (value) 1001, 1002, 1004, 1006, 1009 */
		if (value <= 1001)
			torch_current = (led_data->flashlight_current[0]);
		else if (value <= 1002)
			torch_current = (led_data->flashlight_current[1]);
		else if (value <= 1004)
			torch_current = (led_data->flashlight_current[2]);
		else if (value <= 1006)
			torch_current = (led_data->flashlight_current[3]);
		else if (value <= 1009)
			torch_current = (led_data->flashlight_current[4]);
		else
			torch_current = 50;

		brightness= S2MU005_TORCH_BRIGHTNESS(torch_current);
		pr_info("torch current : %d mA\n", torch_current);
		assistive_light = true;
	} else if (2001 <= value && value <= 2016) {
		/* for current check using sysfs (25mA~400mA) */
		torch_current = 25 * (value - 2000);

		brightness= S2MU005_TORCH_BRIGHTNESS(torch_current);
		pr_info("torch current : %d mA\n", torch_current);
		assistive_light = true;
	} else {
		pr_info("[LED]%s , Invalid value:%d\n", __func__, value);
		goto err;
	}

	if (led_cdev->flags & LED_SUSPENDED) {
		pr_info("%s : led suspended\n", __func__);
		goto err;
	}

#ifdef CONFIG_S2MU005_LEDS_I2C
	reg_val = S2MU005_FLASH_TORCH_OFF;
#else
	reg_val = S2MU005_CH1_TORCH_ON_GPIO | S2MU005_CH1_FLASH_ON_GPIO;
	s2mu005_write_reg(led_data->i2c, CH_FLASH_TORCH_EN, reg_val);
#endif
	s2mu005_led_set(led_cdev, brightness);

#ifdef S2MU005_FLED_DEBUG
	if(value == 0)
		s2mu005_led_dump_reg();
#endif

	mutex_unlock(&led_data->lock);
	return size;

err:
	pr_err("%s : led abnormal end\n", __func__);

	mutex_unlock(&led_data->lock);
	return size;
}

static DEVICE_ATTR(rear_flash, 0644, rear_flash_show, rear_flash_store);
static DEVICE_ATTR(rear_torch_flash, 0644, rear_flash_show, rear_flash_store);

#if defined(CONFIG_LEDS_SUPPORT_FRONT_FLASH)
static ssize_t front_flash_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_TORCH_LED];
	char *str;

	switch (led_data->data->id) {
	case S2MU005_FLASH_LED:
		str = "FLASH";
		break;
	case S2MU005_TORCH_LED:
		str = "TORCH";
		break;
	default:
		str = "NONE";
		break;
	}

	return snprintf(buf, 20, "%s\n", str);
}

static ssize_t front_flash_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct s2mu005_led_data *led_data = g_led_datas[S2MU005_TORCH_LED];
	struct led_classdev *led_cdev = &led_data->cdev;
	int value = 0;
	int reg_val = 0;
	int brightness = 0;
	int ret = 0;

	if ((buf == NULL) || kstrtouint(buf, 10, &value)) {
		return -1;
	}

	pr_info("[LED]%s , value:%d\n", __func__, value);
	mutex_lock(&led_data->lock);

	if (led_data->data->id == S2MU005_FLASH_LED) {
		pr_info("%s : flash is not controlled by sysfs", __func__);
		goto err;
	}

	if (value == 0) {
		/* Turn off Torch */
		ret = s2mu005_update_reg(led_data->i2c, CH_FLASH_TORCH_EN,
			S2MU005_FLASH_TORCH_OFF, S2MU005_CH2_TORCH_ENABLE_MASK);
		if (ret < 0)
			goto err;
		front_assistive_light = false;
	} else if (value == 1) {
		/* Turn on Torch */
		brightness = led_data->front_brightness;
		front_assistive_light = true;
		s2mu005_led_set(led_cdev, brightness);
	} else if (value == 100) {
		/* Factory mode Turn on Torch */
		brightness = led_data->factory_brightness;
		front_assistive_light = true;
		s2mu005_led_set(led_cdev, brightness);
	} else {
		pr_info("[LED]%s , Invalid value:%d\n", __func__, value);
		goto err;
	}

	if (led_cdev->flags & LED_SUSPENDED) {
		pr_info("%s : led suspended\n", __func__);
		goto err;
	}

#ifdef CONFIG_S2MU005_LEDS_I2C
	reg_val = S2MU005_FLASH_TORCH_OFF;
#else
	reg_val = S2MU005_CH2_TORCH_ON_GPIO;
	s2mu005_write_reg(led_data->i2c, CH_FLASH_TORCH_EN, reg_val);
#endif

#ifdef S2MU005_FLED_DEBUG
	if(value == 0)
		s2mu005_led_dump_reg();
#endif

	mutex_unlock(&led_data->lock);
	return size;

err:
	pr_err("%s : led abnormal end\n", __func__);

	mutex_unlock(&led_data->lock);
	return size;
}

static DEVICE_ATTR(front_flash, 0644, front_flash_show, front_flash_store);
static DEVICE_ATTR(front_torch_flash, 0644, front_flash_show, front_flash_store);
#endif

#if defined(CONFIG_OF)
static int s2mu005_led_dt_parse_pdata(struct device *dev,
				struct s2mu005_fled_platform_data *pdata)
{
	struct device_node *led_np, *np, *c_np;
	int ret;
	u32 temp;
	const char *temp_str;
	u32 index;
	u32 args[S2MU005_FLASH_LIGHT_MAX];

	led_np = dev->parent->of_node;

	if (!led_np) {
		pr_err("<%s> could not find led sub-node led_np\n", __func__);
		return -ENODEV;
	}

	np = of_find_node_by_name(led_np, "leds");
	if (!np) {
		pr_err("%s : could not find led sub-node np\n", __func__);
		return -EINVAL;
	}

	ret = pdata->torch_pin = of_get_named_gpio(np, "torch-gpio", 0);
	if (ret < 0) {
		pr_err("%s : can't get torch-gpio\n", __func__);
		return ret;
	}

	ret = pdata->flash_pin = of_get_named_gpio(np, "flash-gpio", 0);
	if (ret < 0) {
		pr_err("%s : can't get flash-gpio\n", __func__);
		return ret;
	}

	ret = of_property_read_u32(np, "flash_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->flash_brightness = S2MU005_FLASH_BRIGHTNESS(temp);
	dev_info(dev, "flash_current = <%d>, brightness = %x\n", temp, pdata->flash_brightness);

	ret = of_property_read_u32(np, "preflash_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->preflash_brightness = S2MU005_TORCH_BRIGHTNESS(temp);
	dev_info(dev, "preflash_current = <%d>, brightness = %x\n", temp, pdata->preflash_brightness);

	ret = of_property_read_u32(np, "movie_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->movie_brightness = S2MU005_TORCH_BRIGHTNESS(temp);
	dev_info(dev, "movie_current = <%d>, brightness = %x\n", temp, pdata->movie_brightness);

	ret = of_property_read_u32(np, "torch_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->torch_brightness = S2MU005_TORCH_BRIGHTNESS(temp);
	dev_info(dev, "torch_current = <%d>, brightness = %x\n", temp, pdata->torch_brightness);

	ret = of_property_read_u32(np, "factory_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->factory_brightness = S2MU005_TORCH_BRIGHTNESS(temp);
	dev_info(dev, "factory_current = <%d>, brightness = %x\n", temp, pdata->factory_brightness);

#if defined(CONFIG_LEDS_SUPPORT_FRONT_FLASH)
	ret = of_property_read_u32(np, "front_torch_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->front_brightness = S2MU005_TORCH_BRIGHTNESS(temp);
	dev_info(dev, "front_torch_current = <%d>, brightness = %x\n", temp, pdata->front_brightness);
#endif

	ret = of_property_read_u32_array(np, "flashlight_current",args, S2MU005_FLASH_LIGHT_MAX);
	if(ret < 0) {
		pdata->flashlight_current[0] = STEP0_CURRENT;
		pdata->flashlight_current[1] = STEP1_CURRENT;
		pdata->flashlight_current[2] = STEP2_CURRENT;
		pdata->flashlight_current[3] = STEP3_CURRENT;
		pdata->flashlight_current[4] = STEP4_CURRENT;
	}else {
		pdata->flashlight_current[0] = args[0];
		pdata->flashlight_current[1] = args[1];
		pdata->flashlight_current[2] = args[2];
		pdata->flashlight_current[3] = args[3];
		pdata->flashlight_current[4] = args[4];
	}
	pdata->num_leds = of_get_child_count(np);

	for_each_child_of_node(np, c_np) {
		ret = of_property_read_u32(c_np, "id", &temp);
		if (ret < 0)
			goto dt_err;
		index = temp;

		if (index < S2MU005_LED_MAX) {
			pdata->leds[index].id = temp;

			ret = of_property_read_string(c_np, "ledname", &temp_str);
			if (ret)
				goto dt_err;
			pdata->leds[index].name = temp_str;

			temp = index ? pdata->preflash_brightness : pdata->flash_brightness;
			if (temp > leds_cur_max[index])
				temp = leds_cur_max[index];
			pdata->leds[index].brightness = temp;

			ret = of_property_read_u32(c_np, "timeout", &temp);
			if (ret)
				goto dt_err;
			if (temp > leds_time_max[index])
				temp = leds_time_max[index];
			pdata->leds[index].timeout = temp;
		}
	}
	return 0;
dt_err:
	pr_err("%s failed to get a timeout\n", __func__);
	return ret;
}
#endif /* CONFIG_OF */

int create_flash_sysfs(void)
{
	int err = -ENODEV;

	if (IS_ERR_OR_NULL(camera_class)) {
		pr_err("flash_sysfs: error, camera class not exist");
		return -ENODEV;
	}

	flash_dev = device_create(camera_class, NULL, 0, NULL, "flash");
	if (IS_ERR(flash_dev)) {
		pr_err("flash_sysfs: failed to create device(flash)\n");
		return -ENODEV;
	}
	err = device_create_file(flash_dev, &dev_attr_rear_flash);
	if (unlikely(err < 0)) {
		pr_err("flash_sysfs: failed to create device file, %s\n",
				dev_attr_rear_flash.attr.name);
	}
	err = device_create_file(flash_dev, &dev_attr_rear_torch_flash);
	if (unlikely(err < 0)) {
		pr_err("flash_sysfs: failed to create device file, %s\n",
				dev_attr_rear_torch_flash.attr.name);
	}

#if defined(CONFIG_LEDS_SUPPORT_FRONT_FLASH)
	err = device_create_file(flash_dev, &dev_attr_front_flash);
	if (unlikely(err < 0)) {
		pr_err("flash_sysfs: failed to create device file, %s\n",
				dev_attr_front_flash.attr.name);
	}

	err = device_create_file(flash_dev, &dev_attr_front_torch_flash);
	if (unlikely(err < 0)) {
		pr_err("flash_sysfs: failed to create device file, %s\n",
				dev_attr_front_torch_flash.attr.name);
	}
#endif
	return 0;
}

static int s2mu005_led_probe(struct platform_device *pdev)
{
	int ret = 0, i = 0;
	u8 temp = 0;

	struct s2mu005_dev *s2mu005 = dev_get_drvdata(pdev->dev.parent);
#ifndef CONFIG_OF
	struct s2mu005_mfd_platform_data *s2mu005_pdata = NULL;
#endif
	struct s2mu005_fled_platform_data *pdata;
	struct s2mu005_led_data *led_data;
	struct s2mu005_led *data;
	struct s2mu005_led_data **led_datas;

	pr_info("[%s] s2mu005_fled start\n", __func__);

	if (!s2mu005) {
		dev_err(&pdev->dev, "drvdata->dev.parent not supplied\n");
		return -ENODEV;
	}
#ifdef CONFIG_OF
	pdata = kzalloc(sizeof(struct s2mu005_fled_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("[%s] failed to allocate driver data\n", __func__);
		return -ENOMEM;
	}

	if (s2mu005->dev->of_node) {
		ret = s2mu005_led_dt_parse_pdata(&pdev->dev, pdata);
		if (ret < 0) {
			pr_err("[%s] not found leds dt! ret[%d]\n",
				__func__, ret);
			kfree(pdata);
			return -1;
		}
	}
#else
	s2mu005_pdata = s2mu005->pdata;

	if (!s2mu005_pdata) {
		dev_err(&pdev->dev, "platform data not supplied\n");
		return -ENODEV;
	}
	pdata = s2mu005_pdata->fled_platform_data;
	if (!pdata) {
		pr_err("[%s] no platform data for this led is found\n",
				__func__);
		return -EFAULT;
	}
#endif

	led_datas = devm_kzalloc(s2mu005->dev,
			sizeof(struct s2mu005_led_data *) *
			S2MU005_LED_MAX, GFP_KERNEL);
	if (!led_datas) {
		pr_err("[%s] memory allocation error led_datas", __func__);
		kfree(pdata);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, led_datas);

	pr_info("%s %d leds\n", __func__, pdata->num_leds);

	for (i = 0; i != pdata->num_leds; ++i) {
		pr_info("%s led%d setup ...\n", __func__, i);

		data = devm_kzalloc(s2mu005->dev, sizeof(struct s2mu005_led),
				GFP_KERNEL);
		if (!data) {
			pr_err("[%s] memory allocation error data\n",
					__func__);
			ret = -ENOMEM;
			continue;
		}

		memcpy(data, &(pdata->leds[i]), sizeof(struct s2mu005_led));
		led_data = devm_kzalloc(&pdev->dev,
				sizeof(struct s2mu005_led_data), GFP_KERNEL);

		g_led_datas[i] = led_data;
		led_datas[i] = led_data;

		if (!led_data) {
			pr_err("[%s] memory allocation error led_data\n",
					__func__);
			kfree(data);
			ret = -ENOMEM;
			continue;
		}

		led_data->i2c = s2mu005->i2c;
		led_data->data = data;
		led_data->cdev.name = data->name;
		led_data->cdev.brightness_set = s2mu005_led_set;
		led_data->cdev.flags = 0;
		led_data->cdev.brightness = data->brightness;
		led_data->cdev.max_brightness = led_data->data->id ?
		S2MU005_TORCH_OUT_I_400MA : S2MU005_FLASH_OUT_I_1200MA;

		mutex_init(&led_data->lock);
		spin_lock_init(&led_data->value_lock);
		INIT_WORK(&led_data->work, s2mu005_led_work);
		ret = led_classdev_register(&pdev->dev, &led_data->cdev);
		if (ret < 0) {
			pr_err("unable to register LED\n");
			cancel_work_sync(&led_data->work);
			mutex_destroy(&led_data->lock);
			kfree(data);
			kfree(led_data);
			led_datas[i] = NULL;
			g_led_datas[i] = NULL;
			ret = -EFAULT;
			continue;
		}
		if (led_data->data->id == S2MU005_TORCH_LED) {
			create_flash_sysfs();
		}
#ifndef CONFIG_S2MU005_LEDS_I2C
		if (gpio_is_valid(pdata->torch_pin) &&
				gpio_is_valid(pdata->flash_pin)) {
			if (ret < 0) {
				pr_err("%s : s2mu005 fled gpio allocation error\n",
					__func__);
			} else {
				led_data->torch_pin = pdata->torch_pin;
				led_data->flash_pin = pdata->flash_pin;
				gpio_request_one(pdata->torch_pin, GPIOF_OUT_INIT_LOW, "LED_GPIO_OUTPUT_LOW");
				gpio_request_one(pdata->flash_pin, GPIOF_OUT_INIT_LOW, "LED_GPIO_OUTPUT_LOW");
				gpio_free(pdata->torch_pin);
				gpio_free(pdata->flash_pin);
			}
		}
#endif

		led_data->flash_brightness = pdata->flash_brightness;
		led_data->preflash_brightness = pdata->preflash_brightness;
		led_data->movie_brightness = pdata->movie_brightness;
		led_data->torch_brightness = pdata->torch_brightness;
		led_data->factory_brightness = pdata->factory_brightness;
#if defined(CONFIG_LEDS_SUPPORT_FRONT_FLASH)
		led_data->front_brightness = pdata->front_brightness;
#endif
		led_data->flashlight_current[0] = pdata->flashlight_current[0];
		led_data->flashlight_current[1] = pdata->flashlight_current[1];
		led_data->flashlight_current[2] = pdata->flashlight_current[2];
		led_data->flashlight_current[3] = pdata->flashlight_current[3];
		led_data->flashlight_current[4] = pdata->flashlight_current[4];

		ret = s2mu005_read_reg(led_data->i2c, 0x73, &temp);	/* EVT0 0x73[3:0] == 0x0 */
		if (ret < 0)
			pr_err("%s : s2mu005 reg fled read fail\n",__func__);

		if ((temp & 0xf) == 0x00) {
			/* FLED_CTRL4 = 0x3A : EVT0 */
			CH_FLASH_TORCH_EN = S2MU005_REG_FLED_CTRL4;
		} else {
			/* FLED_CTRL4 = 0x3C : EVT1 */
			CH_FLASH_TORCH_EN = S2MU005_REG_FLED_RSVD;
		}

#ifdef CONFIG_MUIC_NOTIFIER
		muic_notifier_register(&led_data->batt_nb,
				ta_notification,
				MUIC_NOTIFY_DEV_CHARGER);
#endif
		ret = s2mu005_led_setup(led_data);
		if (ret < 0)
			pr_err("%s : failed s2mu005 led reg init\n", __func__);
	}
#ifdef CONFIG_OF
	kfree(pdata);
#endif
	return 0;
}

static int s2mu005_led_remove(struct platform_device *pdev)
{
	struct s2mu005_led_data **led_datas = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i != S2MU005_LED_MAX; ++i) {
		if (led_datas[i] == NULL)
			continue;

		cancel_work_sync(&led_datas[i]->work);
		mutex_destroy(&led_datas[i]->lock);
		led_classdev_unregister(&led_datas[i]->cdev);
		kfree(led_datas[i]->data);
		kfree(led_datas[i]);
		g_led_datas[i] = NULL;
	}
	kfree(led_datas);

	return 0;
}

static const struct platform_device_id s2mu005_leds_id[] = {
	{"s2mu005-flash", 0},
	{},
};

static struct platform_driver s2mu005_led_driver = {
	.driver = {
		.name  = "s2mu005-flash",
		.owner = THIS_MODULE,
		},
	.probe  = s2mu005_led_probe,
	.remove = s2mu005_led_remove,
	.id_table = s2mu005_leds_id,
};

static int __init s2mu005_led_driver_init(void)
{
	return platform_driver_register(&s2mu005_led_driver);
}
module_init(s2mu005_led_driver_init);

static void __exit s2mu005_led_driver_exit(void)
{
	platform_driver_unregister(&s2mu005_led_driver);
}
module_exit(s2mu005_led_driver_exit);

MODULE_AUTHOR("SUJI LEE <suji0908.lee@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG s2mu005 LED Driver");
MODULE_LICENSE("GPL");
