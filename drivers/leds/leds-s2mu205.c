/*
 * leds-s2mu205.c - LED class driver for S2MU205 LEDs.
 *
 * Copyright (C) 2019 Samsung Electronics
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
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/mfd/samsung/s2mu205.h>
#include <linux/leds-s2mu205.h>
#include <linux/platform_device.h>
#include <linux/sec_batt.h>

extern struct class *camera_class;
struct device *flash_dev;
struct s2mu205_led_data *g_led_data[S2MU205_LED_NUM];
static int s2mu205_factory_mode;

static u8 leds_cur_max[] = {
	S2MU205_FLASH_OUT_I_1200MA,
	S2MU205_TORCH_OUT_I_320MA,
};

static u8 leds_time_max[] = {
	S2MU205_FLASH_TIMEOUT_992MS,
	S2MU205_TORCH_TIMEOUT_15728MS,
};

struct s2mu205_led_data {
	struct led_classdev cdev;
	struct s2mu205_led *data;
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
	unsigned int flashlight_brightness[S2MU205_FLASH_LIGHT_MAX];
};

inline int s2mu205_flash_brightness(u32 flash_current)
{
	if (flash_current <= 400)
		return ((flash_current - 25)/25) & S2MU205_CHX_FLASH_IOUT;
	else if (flash_current > 400 && flash_current <= 1200)
		return (((flash_current - 400)/50) + 0x0F) & S2MU205_CHX_FLASH_IOUT;
	else
		return (flash_current & 0x00) | S2MU205_CHX_FLASH_IOUT;
}

inline int s2mu205_torch_brightness(u32 torch_current)
{
	if (torch_current <= 320)
		return (torch_current - 10)/10 & S2MU205_CHX_TORCH_IOUT;
	else
		return (torch_current & 0x00) | S2MU205_CHX_TORCH_IOUT;
}

/* This function is used by Charger */
void s2mu205_set_operation_mode(int value)
{
	struct s2mu205_led_data *fled = g_led_data[S2MU205_FLASH_LED];
	u8 reg;
	s2mu205_factory_mode = value;

	if (value == S2MU205_FACTORY) {
		s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL1,
					0x80, S2MU205_CHGIN_ENGH);
	} else if (value == S2MU205_NORMAL) {
		s2mu205_read_reg(fled->i2c, S2MU205_FLED_STATUS, &reg);
		if (fled->attach_ta &&
			(reg & S2MU205_CH1_TORCH_ON || reg & S2MU205_CH2_TORCH_ON)){
			s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL1,
						0x80, S2MU205_CHGIN_ENGH);
		}
		else{
			s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL1,
						0x00, S2MU205_CHGIN_ENGH);
		}
	}
}
EXPORT_SYMBOL_GPL(s2mu205_set_operation_mode);

static void s2mu205_set_current(struct s2mu205_led_data *fled,
				int chan, int curr)
{
	struct s2mu205_led *data = fled->data;
	int id = data->id;
	u8 mask = 0, reg = 0;

	switch (chan) {
	case 1:
		if (id == S2MU205_FLASH_LED) {
			pr_info("%s: [CH%d] LED mode is flash\n", __func__, chan);
			reg = S2MU205_FLED_CH1_CTRL0;
			mask = S2MU205_CHX_FLASH_IOUT;
			/* Flash LED CH1 Current setting (25mA ~ 1200mA) */
			if (curr > S2MU205_FLASH_OUT_I_1200MA)
				curr = S2MU205_FLASH_OUT_I_1200MA;
		} else if (id == S2MU205_TORCH_LED) {
			pr_info("%s: [CH%d] LED mode is torch\n", __func__, chan);
			reg = S2MU205_FLED_CH1_CTRL1;
			mask = S2MU205_CHX_TORCH_IOUT;
			/* Torch LED CH1 Current setting (10mA ~ 320mA) */
			if (curr > S2MU205_TORCH_OUT_I_320MA)
				curr = S2MU205_TORCH_OUT_I_320MA;
		}

		/* Flash or Torch set current */
		s2mu205_update_reg(fled->i2c, reg, curr, mask);
		pr_info("%s: [CH%d] FLED Current setting = %d\n",
			__func__, chan, curr);
		break;
	case 2:
		if (id == S2MU205_FLASH_LED) {
			pr_info("%s: [CH%d] LED mode is flash\n", __func__, chan);
			reg = S2MU205_FLED_CH2_CTRL0;
			mask = S2MU205_CHX_FLASH_IOUT;
			/* Flash LED CH2 Current setting (25mA ~ 400mA) */
			if (curr > S2MU205_FLASH_OUT_I_400MA)
				curr = S2MU205_FLASH_OUT_I_400MA;
		} else if (id == S2MU205_TORCH_LED) {
			pr_info("%s: [CH%d] LED mode is torch\n", __func__, chan);
			reg = S2MU205_FLED_CH2_CTRL1;
			mask = S2MU205_CHX_TORCH_IOUT;
			/* Torch LED CH2 Current setting (10mA ~ 320mA) */
			if (curr > S2MU205_TORCH_OUT_I_320MA)
				curr = S2MU205_TORCH_OUT_I_320MA;
		}

		/* Flash or Torch set current */
		s2mu205_update_reg(fled->i2c, reg, curr, mask);
		pr_info("%s: [CH%d] FLED Current setting = %d\n",
			__func__, chan, curr);
		break;
	default:
		pr_info("%s: Wrong channel\n", __func__);
		break;
	}
}

static int s2mu205_get_torch_curr(struct s2mu205_led_data *fled,
				int chan)
{
	int curr = -1;
	u8 reg, dest;

	switch (chan) {
	case 1:
		dest = S2MU205_FLED_CH1_CTRL1;
		break;
	case 2:
		dest = S2MU205_FLED_CH2_CTRL1;
		break;
	default:
		return -1;
		break;
	}
	s2mu205_read_reg(fled->i2c, dest, &reg);

	reg &= S2MU205_CHX_TORCH_IOUT;
	curr = (curr * 10) + 10;

	pr_info("%s: CH%02d torch curr = %dmA\n", __func__,
		chan, curr);

	return curr;
}

static int s2mu205_get_flash_curr(struct s2mu205_led_data *fled,
					int chan)
{
	int curr = -1;
	u8 reg, dest;

	switch (chan) {
	case 1:
		dest = S2MU205_FLED_CH1_CTRL0;
		break;
	case 2:
		dest = S2MU205_FLED_CH2_CTRL0;
		break;
	default:
		return -1;
		break;
	}
	s2mu205_read_reg(fled->i2c, dest, &reg);

	reg &= S2MU205_CHX_FLASH_IOUT;

	if (reg <= S2MU205_FLASH_OUT_I_400MA)
		curr = (curr * 25) + 25;
	else
		curr = (curr - 0x0F)*50 + 400;

	pr_info("%s: CH%02d flash curr = %dmA\n", __func__,
		chan, curr);

	return curr;
}

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
	struct s2mu205_led_data *fled = container_of(nb,
						struct s2mu205_led_data,
						batt_nb);
#ifndef CONFIG_S2MU205_LEDS_I2C
	int ret = 0;
#endif
	switch (action) {
	case MUIC_NOTIFY_CMD_DETACH:
	case MUIC_NOTIFY_CMD_LOGICALLY_DETACH:
		if (!fled->attach_ta)
			goto err;

		fled->attach_ta = 0;

		if (!fled->data->id) {
			pr_info("%s : flash mode\n", __func__);
			goto err;
		}

#ifndef CONFIG_S2MU205_LEDS_I2C
		if (gpio_is_valid(fled->torch_pin)) {
			ret = devm_gpio_request(fled->cdev.dev,
						fled->torch_pin,
						"s2mu205_gpio");
			if (ret) {
				pr_err("%s : fail to assignment gpio\n",
					__func__);
				goto gpio_free_data;
			}
		}
		if (gpio_get_value(fled->torch_pin)) {
			gpio_direction_output(fled->torch_pin, 0);
			gpio_direction_output(fled->torch_pin, 1);
			goto gpio_free_data;
		}
#else
		/* check TORCH On/Off */
		s2mu205_read_reg(fled->i2c, S2MU205_FLED_STATUS, &temp);
		if (temp & S2MU205_CH1_TORCH_ON) {
			/* CH1 Torch Off */
			s2mu205_update_reg(fled->i2c,
						S2MU205_FLED_CTRL0,
						S2MU205_TORCH_OFF,
						S2MU205_CH1_TORCH_FLED_EN);
			pr_info("%s : Torch Off\n", __func__);

			/* CH1 Torch On */
			s2mu205_update_reg(fled->i2c,
						S2MU205_FLED_CTRL0,
						S2MU205_CH1_TORCH_ON_I2C,
						S2MU205_CH1_TORCH_FLED_EN);
			pr_info("%s : Torch On\n", __func__);
		}
#endif
		if (s2mu205_factory_mode == S2MU205_NORMAL) {
			/* CHGIN_ENGH = 0 */
			s2mu205_update_reg(fled->i2c,
						S2MU205_FLED_CTRL1,
						0x00, S2MU205_CHGIN_ENGH);
		}

		break;
	case MUIC_NOTIFY_CMD_ATTACH:
	case MUIC_NOTIFY_CMD_LOGICALLY_ATTACH:
		fled->attach_ta = 0;
		attach_cable_check(attached_dev, &fled->attach_ta,
					&fled->attach_sdp);
		if (fled->attach_ta) {
			s2mu205_read_reg(fled->i2c,
					S2MU205_FLED_STATUS, &temp);

			/* if Torch On & Charging On, set CHGIN_ENGH bit 1 */
			if (temp & S2MU205_CH1_TORCH_ON ||
				temp & S2MU205_CH2_TORCH_ON) {
				s2mu205_update_reg(fled->i2c,
							S2MU205_FLED_CTRL1,
							0x80, S2MU205_CHGIN_ENGH);
			}
		}
		return 0;
	default:
		goto err;
		break;
	}

#ifndef CONFIG_S2MU205_LEDS_I2C
gpio_free_data:
	gpio_free(fled->torch_pin);
	pr_info("%s: gpio free\n", __func__);
#endif
	pr_info("%s: complete detached\n", __func__);
	return 0;
err:
	pr_err("%s: abandond access %d\n", __func__, fled->attach_ta);
	return 0;
}
#endif

static void torch_led_on_off(int value)
{
	pr_info("%s: Torch(%d), TA_Attach(%d)\n",
		__func__, value, g_led_data[S2MU205_FLASH_LED]->attach_ta);

	/* Torch On & TA Attach */
	if (value && g_led_data[S2MU205_FLASH_LED]->attach_ta) {
		s2mu205_update_reg(g_led_data[S2MU205_FLASH_LED]->i2c,
					S2MU205_FLED_CTRL1, 0x80, S2MU205_CHGIN_ENGH);
		pr_info("%s: CHGIN_ENGH set 1\n", __func__);
	}

	/* Torch Off */
	if ((value == 0) && s2mu205_factory_mode == S2MU205_NORMAL) {
		s2mu205_update_reg(g_led_data[S2MU205_FLASH_LED]->i2c,
					S2MU205_FLED_CTRL1, 0x00,
					S2MU205_CHGIN_ENGH);
		pr_info("%s: CHGIN_ENGH set 0\n", __func__);
	}
}

static void led_set(struct s2mu205_led_data *fled)
{
	int id = fled->data->id, curr = fled->test_brightness;

#ifndef CONFIG_S2MU205_LEDS_I2C
	int gpio_pin = (id == S2MU205_FLASH_LED) ?
			fled->flash_pin : fled->torch_pin;

	devm_gpio_request(fled->cdev.dev, gpio_pin,
				"s2mu205_gpio_pin");
#endif
	/* FLED Current setting */
	s2mu205_set_current(fled, 1, curr);

	if (curr == LED_TURN_OFF) {
		pr_info("%s: LED Off\n", __func__);
#ifdef CONFIG_S2MU205_LEDS_I2C
		/* I2C set Off */
		if (id == S2MU205_FLASH_LED)
			s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
						S2MU205_FLASH_OFF,
						S2MU205_CH1_FLASH_FLED_EN);
		else if (id == S2MU205_TORCH_LED)
			s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
						S2MU205_TORCH_OFF,
						S2MU205_CH1_TORCH_FLED_EN);
#else
		/* GPIO set Low */
		gpio_direction_output(gpio_pin, 0);
#endif
		/* If torch mode is On, CHGIN_ENGH set 0 */
		if (id == S2MU205_TORCH_LED && fled->attach_ta) {
			if (s2mu205_factory_mode == S2MU205_NORMAL) {
				s2mu205_update_reg(fled->i2c,
							S2MU205_FLED_CTRL1,
							0x00, S2MU205_CHGIN_ENGH);
			}
		}
#ifndef CONFIG_S2MU205_LEDS_I2C
		goto gpio_free_data;
#endif
	} else {
		pr_info("%s: LED On\n", __func__);
		/* Charging & Torch, CHGIN_ENGH set 1 */
		if (id == S2MU205_TORCH_LED && fled->attach_ta) {
			s2mu205_update_reg(fled->i2c,
						S2MU205_FLED_CTRL1,
						0x80, S2MU205_CHGIN_ENGH);
		}
#ifdef CONFIG_S2MU205_LEDS_I2C
		/* I2C set On */
		if (id == S2MU205_FLASH_LED)
			s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
						S2MU205_CH1_FLASH_ON_I2C,
						S2MU205_CH1_FLASH_FLED_EN);
		else if (id == S2MU205_TORCH_LED)
			s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
						S2MU205_CH1_TORCH_ON_I2C,
						S2MU205_CH1_TORCH_FLED_EN);
#else
		/* GPIO set High */
		gpio_direction_output(gpio_pin, 1);
		goto gpio_free_data;
#endif
	}
	return;

#ifndef CONFIG_S2MU205_LEDS_I2C
gpio_free_data:
	gpio_free(gpio_pin);
	pr_info("%s: gpio free\n", __func__);
	return;
#endif

}

static void s2mu205_led_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	unsigned long flags;
	struct s2mu205_led_data *fled;
	u8 max;

	fled = container_of(led_cdev, struct s2mu205_led_data, cdev);
	max = led_cdev->max_brightness;

	pr_info("%s: value = %d, max = %d\n", __func__, value, max);

	spin_lock_irqsave(&fled->value_lock, flags);
	fled->test_brightness = min_t(int, (int)value, (int)max);
	spin_unlock_irqrestore(&fled->value_lock, flags);

	led_set(fled);
}

static void s2mu205_led_work(struct work_struct *work)
{
	struct s2mu205_led_data *fled;

	fled = container_of(work, struct s2mu205_led_data, work);
	pr_debug("%s: [led]\n", __func__);

	mutex_lock(&fled->lock);
	led_set(fled);
	mutex_unlock(&fled->lock);
}

static void s2mu205_led_setup(struct s2mu205_led_data *fled)
{
	pr_info("%s: FLED setup start\n", __func__);

	/* Controlled Channel1, Channel2 independently */
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL1,
				0x00, S2MU205_EN_CHANNEL_SHARE);

	/* Boost vout flash 4.5V */
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL2,
				0x1C, S2MU205_BOOST_VOUT_FLASH);

	/* FLED_BOOST_EN */
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL1,
				0x40, S2MU205_FLED_BOOST_EN);

	/* Flash timer Maximum mode */
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CH1_CTRL3,
				0x80, S2MU205_CHX_FLASH_TMR_MODE);

	if (fled->data->id == S2MU205_FLASH_LED)
		/* flash timer Maximum set */
		s2mu205_update_reg(fled->i2c, S2MU205_FLED_CH1_CTRL3,
					fled->data->timeout,
					S2MU205_CHX_FLASH_TMR_DUR);
	else
		/* torch timer Maximum set */
		s2mu205_update_reg(fled->i2c, S2MU205_FLED_CH1_CTRL2,
					fled->data->timeout,
					S2MU205_CHX_TORCH_TMR_DUR);

	/* flash brightness set */
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CH1_CTRL0,
				fled->flash_brightness, S2MU205_CHX_FLASH_IOUT);

	/* torch brightness set */
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CH1_CTRL1,
				fled->preflash_brightness, S2MU205_CHX_TORCH_IOUT);

	/* factory mode set */
	pr_info("%s: Factory mode[%s]\n", __func__,
		factory_mode ? "Enable" : "Disable");
	s2mu205_set_operation_mode(factory_mode);

	/* I2C or GPIO mode set */
#ifdef CONFIG_S2MU205_LEDS_I2C
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
				S2MU205_TORCH_OFF, S2MU205_FLED_CTRL0_MASK);
#else
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
				S2MU205_CH1_FLASH_ON_GPIO, S2MU205_CH1_FLASH_FLED_EN);
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
				S2MU205_CH1_TORCH_ON_GPIO, S2MU205_CH1_TORCH_FLED_EN);
#endif

#ifdef S2MU205_FLED_DEBUG
	s2mu205_led_dump_reg();
#endif
	pr_info("%s: FLED setup complete\n", __func__);
}

#ifdef S2MU205_FLED_DEBUG
void s2mu205_led_dump_reg(void)
{
	struct s2mu205_led_data *fled = g_led_data[S2MU205_FLASH_LED];
	int i = 0;
	u8 temp;

	pr_info("[LED] S2MU205 FLED DEBUG : S\n");

	/* FLED_STATUS to FLED_TEST3 */
	for (i = 0x3F; i <= 0x4E; i++) {
		s2mu205_read_reg(fled->i2c, i, &temp);
		pr_info("[LED] 0x%02X : 0x%02X \n", i, temp);
	}

	pr_info("[LED] S2MU205 FLED DEBUG : X\n");
}
#endif

int s2mu205_led_mode_ctrl(int state)
{
	struct s2mu205_led_data *fled = g_led_data[S2MU205_FLASH_LED];
	int gpio_torch = fled->torch_pin;
	int gpio_flash = fled->flash_pin;

	pr_info("%s: state = %d\n", __func__, state);

	devm_gpio_request(fled->cdev.dev, gpio_torch,
				"s2mu205_gpio_torch");

	devm_gpio_request(fled->cdev.dev, gpio_flash,
				"s2mu205_gpio_flash");

	switch (state) {
	case S2MU205_FLED_MODE_OFF:
		gpio_direction_output(gpio_torch, 0);
		gpio_direction_output(gpio_flash, 0);
		torch_led_on_off(0);
		break;
	case S2MU205_FLED_MODE_PREFLASH:
		gpio_direction_output(gpio_torch, 1);
		break;
	case S2MU205_FLED_MODE_FLASH:
		gpio_direction_output(gpio_flash, 1);
		break;
	case S2MU205_FLED_MODE_MOVIE:
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

static ssize_t fled_torch_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s2mu205_led_data *fled = g_led_data[S2MU205_TORCH_LED];
	char *str;
	int curr, cnt = 0;

	switch (fled->data->id) {
	case S2MU205_FLASH_LED:
		str = "FLASH";
		break;
	case S2MU205_TORCH_LED:
		str = "TORCH";
		break;
	default:
		str = "NONE";
		break;
	}

	cnt += sprintf(buf + cnt, "FLED mode = %s\n", str);

	/* FLED shows torch current in channel 1 */
	curr = s2mu205_get_torch_curr(fled, 1);
	pr_info("%s: Channel[%d], Curr[%dmA]\n", __func__, 1, curr);
	if (curr >= 0)
		cnt += sprintf(buf + cnt, "CH%02d: %dmA\n", 1, curr);

	return cnt;
}

static ssize_t fled_torch_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct s2mu205_led_data *fled = g_led_data[S2MU205_TORCH_LED];
#ifdef CONFIG_S2MU205_LEDS_I2C
	struct led_classdev *led_cdev = &fled->cdev;
#endif
	int value = 0, brightness = 0;
	u32 torch_current;

	if ((buf == NULL) || kstrtouint(buf, 10, &value)) {
		return -1;
	}

	pr_info("[LED]%s: value: %d\n", __func__, value);
	mutex_lock(&fled->lock);

	if (fled->data->id == S2MU205_FLASH_LED) {
		pr_info("%s: flash is not controlled by sysfs", __func__);
		mutex_unlock(&fled->lock);
		return size;
	}

	if (value == 0) {
		/* Turn off Torch */
		brightness = LED_TURN_OFF;
	} else if (1 <= value && value <= 32) {
		/* for current check using sysfs (10mA~320mA) */
		torch_current = value * 10;
		brightness = s2mu205_torch_brightness(torch_current);
		pr_info("%s: torch current: %dmA\n", __func__, torch_current);
	} else if (value == 100) {
		/* Turn on Torch */
		brightness = fled->torch_brightness;
	} else if (value == 200) {
		/* Factory mode Turn on Torch */
		brightness = fled->factory_brightness;
	} else {
		pr_info("[LED]%s: Invalid value:%d\n", __func__, value);
		mutex_unlock(&fled->lock);
		return size;
	}

#ifdef CONFIG_S2MU205_LEDS_I2C
	s2mu205_led_set(led_cdev, brightness);
#else
	/* FLED Current setting */
	s2mu205_set_current(fled, 1, brightness);

	/* Torch GPIO set high */
	s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
				S2MU205_CH1_TORCH_ON_GPIO,
				S2MU205_CH1_TORCH_FLED_EN);

	if (brightness == LED_TURN_OFF)
		s2mu205_led_mode_ctrl(S2MU205_FLED_MODE_OFF);
	else
		s2mu205_led_mode_ctrl(S2MU205_FLED_MODE_MOVIE);
#endif

#ifdef S2MU205_FLED_DEBUG
	s2mu205_led_dump_reg();
#endif
	mutex_unlock(&fled->lock);
	return size;
}


static ssize_t fled_flash_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s2mu205_led_data *fled = g_led_data[S2MU205_FLASH_LED];
	char *str;
	int curr, cnt = 0;

	switch (fled->data->id) {
	case S2MU205_FLASH_LED:
		str = "FLASH";
		break;
	case S2MU205_TORCH_LED:
		str = "TORCH";
		break;
	default:
		str = "NONE";
		break;
	}

	cnt += sprintf(buf + cnt, "FLED mode = %s\n", str);

	/* FLED shows flash current in channel 1 */
	curr = s2mu205_get_flash_curr(fled, 1);
	pr_info("%s: Channel[%d], Curr[%dmA]\n", __func__, 1, curr);
	if (curr >= 0)
		cnt += sprintf(buf + cnt, "CH%02d: %dmA\n", 1, curr);

	return cnt;
}

static ssize_t fled_flash_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct s2mu205_led_data *fled = g_led_data[S2MU205_FLASH_LED];
	int prev_led_id = 0;
#ifdef CONFIG_S2MU205_LEDS_I2C
	struct led_classdev *led_cdev = &fled->cdev;
#endif
	int value = 0, brightness = 0;
	int mode = -1;

	if ((buf == NULL) || kstrtouint(buf, 10, &value)) {
		return -1;
	}

	pr_info("[LED]%s: value: %d\n", __func__, value);
	mutex_lock(&fled->lock);

	if (fled->data->id == S2MU205_TORCH_LED) {
		pr_info("%s: torch is not controlled by sysfs", __func__);
		mutex_unlock(&fled->lock);
		return size;
	}

	if (value <= 0) {
		mode = S2MU205_FLED_MODE_OFF;
		brightness = LED_TURN_OFF;
	} else if (value == 1) {
		mode = S2MU205_FLED_MODE_MOVIE;
		brightness = fled->torch_brightness;
	} else if (value == 100) {
		/* Factory Torch*/
		pr_info("%s: factory torch current [%d]\n", __func__, fled->factory_brightness);
		brightness = fled->factory_brightness;
		mode = S2MU205_FLED_MODE_MOVIE;
	} else if (value == 200) {
		/* Factory Flash */
		pr_info("%s: factory flash current [%d]\n", __func__, fled->factory_brightness);
		brightness = fled->factory_brightness;
		mode = S2MU205_FLED_MODE_FLASH;
	} else if (value <= 1010 && value >= 1001) {
		mode = S2MU205_FLED_MODE_MOVIE;
		/* (value) 1001, 1002, 1004, 1006, 1009 */
		if (value <= 1001)
			brightness = fled->flashlight_brightness[0];
		else if (value <= 1002)
			brightness = fled->flashlight_brightness[1];
		else if (value <= 1004)
			brightness = fled->flashlight_brightness[2];
		else if (value <= 1006)
			brightness = fled->flashlight_brightness[3];
		else if (value <= 1009)
			brightness = fled->flashlight_brightness[4];
		else
			brightness = fled->torch_brightness;
	} else if (value == 2) {
		mode = S2MU205_FLED_MODE_FLASH;
		brightness = fled->flash_brightness;
	}

	if (mode == S2MU205_FLED_MODE_MOVIE) {
		pr_info("%s: %d: S2MU205_FLED_MODE_MOVIE - %dmA\n", __func__, value, brightness );
		/* torch current set */
#ifdef CONFIG_S2MU205_LEDS_I2C
		 s2mu205_led_set(led_cdev, brightness);
#else

		prev_led_id = fled->data->id;
		fled->data->id = S2MU205_TORCH_LED;

		/* FLED Current setting */
		s2mu205_set_current(fled, 1, brightness);

		fled->data->id = prev_led_id;

		/* Torch GPIO set high */
		s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
					S2MU205_CH1_TORCH_ON_GPIO,
					S2MU205_CH1_TORCH_FLED_EN);

		s2mu205_led_mode_ctrl(S2MU205_FLED_MODE_MOVIE);
#endif
	} else if (mode == S2MU205_FLED_MODE_FLASH) {
		pr_info("%s: %d: S2MU205_FLED_MODE_FLASH - %dmA\n", __func__, value, brightness );
#ifdef CONFIG_S2MU205_LEDS_I2C
		s2mu205_led_set(led_cdev, brightness);
#else
		prev_led_id = fled->data->id;
		fled->data->id = S2MU205_FLASH_LED;

		/* FLED Current setting */
		s2mu205_set_current(fled, 1, brightness);

		fled->data->id = prev_led_id;

		/* Torch GPIO set high */
		s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
					S2MU205_CH1_FLASH_ON_GPIO,
					S2MU205_CH1_FLASH_FLED_EN);

		s2mu205_led_mode_ctrl(S2MU205_FLED_MODE_FLASH);
#endif
	} else if (mode == S2MU205_FLED_MODE_OFF){
		pr_info("%s: %d: S2MU205_FLED_MODE_OFF \n", __func__,value );
#ifdef CONFIG_S2MU205_LEDS_I2C
		s2mu205_led_set(led_cdev, brightness);
#else
		/* FLED Current setting */
		s2mu205_set_current(fled, 1, brightness);

		/* Torch GPIO set high */
		s2mu205_update_reg(fled->i2c, S2MU205_FLED_CTRL0,
					S2MU205_CH1_TORCH_ON_GPIO,
					S2MU205_CH1_TORCH_FLED_EN);
		s2mu205_led_mode_ctrl(S2MU205_FLED_MODE_OFF);
#endif
	}
#ifdef S2MU205_FLED_DEBUG
	s2mu205_led_dump_reg();
#endif
	mutex_unlock(&fled->lock);
	return size;
}

static DEVICE_ATTR(rear_torch_flash, 0644, fled_torch_show, fled_torch_store);
static DEVICE_ATTR(rear_flash, 0644, fled_flash_show, fled_flash_store);

#if defined(CONFIG_OF)
static int s2mu205_led_dt_parse_pdata(struct device *dev,
					struct s2mu205_fled_platform_data *pdata)
{
	struct device_node *led_np, *np, *c_np;
	int ret, fled_index=0;
	u32 temp, index;
	const char *temp_str;

	led_np = dev->parent->of_node;

	if (!led_np) {
		pr_err("%s: could not find led sub-node led_np\n", __func__);
		return -ENODEV;
	}

	np = of_find_node_by_name(led_np, "flash_led");
	if (!np) {
		pr_err("%s: could not find led sub-node np\n", __func__);
		return -EINVAL;
	}

	ret = pdata->torch_pin = of_get_named_gpio(np, "torch-gpio", 0);
	if (ret < 0) {
		pr_err("%s: can't get torch-gpio\n", __func__);
		return ret;
	}

	ret = pdata->flash_pin = of_get_named_gpio(np, "flash-gpio", 0);
	if (ret < 0) {
		pr_err("%s: can't get flash-gpio\n", __func__);
		return ret;
	}

	ret = of_property_read_u32(np, "flash_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->flash_brightness = s2mu205_flash_brightness(temp);
	dev_info(dev, "flash_current = <%d>, brightness = %x\n",
		 temp, pdata->flash_brightness);

	ret = of_property_read_u32(np, "preflash_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->preflash_brightness = s2mu205_torch_brightness(temp);
	dev_info(dev, "preflash_current = <%d>, brightness = %x\n",
		 temp, pdata->preflash_brightness);

	ret = of_property_read_u32(np, "movie_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->movie_brightness = s2mu205_torch_brightness(temp);
	dev_info(dev, "movie_current = <%d>, brightness = %x\n",
		 temp, pdata->movie_brightness);

	ret = of_property_read_u32(np, "torch_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->torch_brightness = s2mu205_torch_brightness(temp);
	dev_info(dev, "torch_current = <%d>, brightness = %x\n",
		 temp, pdata->torch_brightness);

	ret = of_property_read_u32(np, "factory_current", &temp);
	if (ret < 0)
		goto dt_err;
	pdata->factory_brightness = s2mu205_torch_brightness(temp);
	dev_info(dev, "factory_current = <%d>, brightness = %x\n",
		 temp, pdata->factory_brightness);

	ret = of_property_read_u32_array(np, "flashlight_current",
					pdata->flashlight_brightness, S2MU205_FLASH_LIGHT_MAX);
	if (ret < 0) {
		pr_err("%s : could not find flashlight_current\n", __func__);
		//default setting
		pdata->flashlight_brightness[0] = 45;
		pdata->flashlight_brightness[1] = 75;
		pdata->flashlight_brightness[2] = 125;
		pdata->flashlight_brightness[3] = 195;
		pdata->flashlight_brightness[4] = 270;
	}

	for (fled_index = 0; fled_index < S2MU205_FLASH_LIGHT_MAX; fled_index++) {
		pdata->flashlight_brightness[fled_index] = s2mu205_torch_brightness(pdata->flashlight_brightness[fled_index]);
	}
	pdata->num_leds = of_get_child_count(np);

	for_each_child_of_node(np, c_np) {
		ret = of_property_read_u32(c_np, "id", &temp);
		if (ret < 0)
			goto dt_err;
		index = temp;

		if (index < S2MU205_LED_MAX) {
			pdata->leds[index].id = temp;

			ret = of_property_read_string(c_np,
						"ledname", &temp_str);
			if (ret)
				goto dt_err;
			pdata->leds[index].name = temp_str;

			temp = index ? pdata->preflash_brightness :
					pdata->flash_brightness;
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

	return 0;
}

static int s2mu205_led_probe(struct platform_device *pdev)
{
	int ret = 0, i = 0, fled_index = 0;

	struct s2mu205_dev *s2mu205 = dev_get_drvdata(pdev->dev.parent);
#ifndef CONFIG_OF
	struct s2mu205_mfd_platform_data *s2mu205_pdata = NULL;
#endif
	struct s2mu205_fled_platform_data *pdata;
	struct s2mu205_led_data *fled;
	struct s2mu205_led *data;
	struct s2mu205_led_data **fled_data;
	pr_info("%s: [s2mu205] FLED probe start\n", __func__);

	if (!s2mu205) {
		dev_err(&pdev->dev, "drvdata->dev.parent not supplied\n");
		return -ENODEV;
	}
#ifdef CONFIG_OF
	pdata = kzalloc(sizeof(struct s2mu205_fled_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		return -ENOMEM;
	}

	if (s2mu205->dev->of_node) {
		ret = s2mu205_led_dt_parse_pdata(&pdev->dev, pdata);
		if (ret < 0) {
			pr_err("%s: not found leds dt! ret[%d]\n",
				__func__, ret);
			kfree(pdata);
			return -1;
		}
	}
#else
	s2mu205_pdata = s2mu205->pdata;

	if (!s2mu205_pdata) {
		dev_err(&pdev->dev, "platform data not supplied\n");
		return -ENODEV;
	}
	pdata = s2mu205_pdata->fled_platform_data;
	if (!pdata) {
		pr_err("%s: no platform data for this led is found\n",
				__func__);
		return -EFAULT;
	}
#endif

	fled_data = devm_kzalloc(s2mu205->dev,
				 sizeof(struct s2mu205_led_data *) *
				 S2MU205_LED_MAX, GFP_KERNEL);
	if (!fled_data) {
		pr_err("%s: memory allocation error fled_data", __func__);
		kfree(pdata);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, fled_data);

	pr_info("%s: %d leds\n", __func__, pdata->num_leds);

	for (i = 0; i != pdata->num_leds - 1; ++i) {
		pr_info("%s: led[%d] setup ...\n", __func__, i);

		data = devm_kzalloc(s2mu205->dev, sizeof(struct s2mu205_led),
					GFP_KERNEL);
		if (!data) {
			pr_err("%s: memory allocation error data\n",
				__func__);
			ret = -ENOMEM;
			continue;
		}

		memcpy(data, &(pdata->leds[i]), sizeof(struct s2mu205_led));
		fled = devm_kzalloc(&pdev->dev,
					sizeof(struct s2mu205_led_data),
					GFP_KERNEL);

		g_led_data[i] = fled;
		fled_data[i] = fled;

		if (!fled) {
			pr_err("[%s] memory allocation error fled\n",
				__func__);
			kfree(data);
			ret = -ENOMEM;
			continue;
		}

		fled->i2c = s2mu205->i2c;
		fled->data = data;
		fled->cdev.name = data->name;
		fled->cdev.brightness_set = s2mu205_led_set;
		fled->cdev.flags = 0;
		fled->cdev.brightness = data->brightness;
		if(fled->data->id == S2MU205_TORCH_LED)
			fled->cdev.max_brightness = S2MU205_TORCH_OUT_I_320MA;
		else
			fled->cdev.max_brightness = S2MU205_FLASH_OUT_I_1200MA;

		mutex_init(&fled->lock);
		spin_lock_init(&fled->value_lock);
		INIT_WORK(&fled->work, s2mu205_led_work);
		ret = led_classdev_register(&pdev->dev, &fled->cdev);
		if (ret < 0) {
			pr_err("unable to register LED\n");
			cancel_work_sync(&fled->work);
			mutex_destroy(&fled->lock);
			kfree(data);
			kfree(fled);
			fled_data[i] = NULL;
			g_led_data[i] = NULL;
			ret = -EFAULT;
			continue;
		}
		if (fled->data->id == S2MU205_FLASH_LED) {
			create_flash_sysfs();
		}
#ifndef CONFIG_S2MU205_LEDS_I2C
		if (gpio_is_valid(pdata->torch_pin) &&
		    gpio_is_valid(pdata->flash_pin)) {
			pr_info("%s: s2mu205 fled gpio allocation\n",
				__func__);
			fled->torch_pin = pdata->torch_pin;
			fled->flash_pin = pdata->flash_pin;
			gpio_request_one(pdata->torch_pin,
					 GPIOF_OUT_INIT_LOW,
					 "LED_GPIO_OUTPUT_LOW");
			gpio_request_one(pdata->flash_pin,
					 GPIOF_OUT_INIT_LOW,
					 "LED_GPIO_OUTPUT_LOW");
			gpio_free(pdata->torch_pin);
			gpio_free(pdata->flash_pin);
		}
#endif

		fled->flash_brightness = pdata->flash_brightness;
		fled->preflash_brightness = pdata->preflash_brightness;
		fled->movie_brightness = pdata->movie_brightness;
		fled->torch_brightness = pdata->torch_brightness;
		fled->factory_brightness = pdata->factory_brightness;

		for (fled_index = 0; fled_index < S2MU205_FLASH_LIGHT_MAX; fled_index++) {
			fled->flashlight_brightness[fled_index] = pdata->flashlight_brightness[fled_index];
		}

#ifdef CONFIG_MUIC_NOTIFIER
		muic_notifier_register(&fled->batt_nb,
					ta_notification,
					MUIC_NOTIFY_DEV_CHARGER);
#endif
		s2mu205_led_setup(fled);
	}
#ifdef CONFIG_OF
	kfree(pdata);
#endif
	return 0;
}

static int s2mu205_led_remove(struct platform_device *pdev)
{
	struct s2mu205_led_data **fled_data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i != S2MU205_LED_MAX; ++i) {
		if (fled_data[i] == NULL)
			continue;

		cancel_work_sync(&fled_data[i]->work);
		mutex_destroy(&fled_data[i]->lock);
		led_classdev_unregister(&fled_data[i]->cdev);
		kfree(fled_data[i]->data);
		kfree(fled_data[i]);
		g_led_data[i] = NULL;
	}
	kfree(fled_data);

	return 0;
}

static const struct platform_device_id s2mu205_leds_id[] = {
	{"leds-s2mu205", 0},
	{},
};

static struct platform_driver s2mu205_led_driver = {
	.driver = {
		.name  = "leds-s2mu205",
		.owner = THIS_MODULE,
		},
	.probe  = s2mu205_led_probe,
	.remove = s2mu205_led_remove,
	.id_table = s2mu205_leds_id,
};

static int __init s2mu205_led_driver_init(void)
{
	return platform_driver_register(&s2mu205_led_driver);
}
module_init(s2mu205_led_driver_init);

static void __exit s2mu205_led_driver_exit(void)
{
	platform_driver_unregister(&s2mu205_led_driver);
}
module_exit(s2mu205_led_driver_exit);

MODULE_AUTHOR("SUJI LEE <suji0908.lee@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG s2mu205 LED Driver");
MODULE_LICENSE("GPL");
