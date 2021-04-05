/*
 * RGB-led driver for s2mu004
 *
 * Copyright (C) 2015 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

 #define pr_fmt(fmt)	"[LED] " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/sec_sysfs.h>
#include <linux/mfd/samsung/s2mu106.h>
#include <linux/leds-s2mu106-rgb.h>

struct s2mu106_rgb_drvdata {
	struct s2mu106_rgb_pdata *pdata;
	struct i2c_client *i2c;
	struct device *dev;
	u8 brightness;
	u8 lpmode;
	u32 on_delay;
	u32 off_delay;
	u32 ratio[S2MU106_LED_NUM];
};

static void s2mu106_rgb_set_state(struct s2mu106_rgb_drvdata *ddata,
	enum led_color led, u32 brightness, unsigned int led_state)
{
	int ret = 0;
	int col = ddata->pdata->col[led];
	u8 reg, val, mask;
	u32 tmp;

	if (brightness) {
		tmp = (u32)(brightness * ddata->brightness);
		tmp /= ddata->pdata->brightness;
		if (tmp == 0)
			tmp = 1;
		reg = S2MU106_RGB_LED1_CURR + col;
		val = (u8)tmp;
		pr_info("LED[%d] %u\n", led, brightness);
		ret = s2mu106_write_reg(ddata->i2c, reg, val);
		if (IS_ERR_VALUE(ret)) {
			pr_err("failed to write brightness : %d\n", ret);
			return;
		}
		val = led_state << ((S2MU106_LED_NUM - col - 1) << 1);
	} else
		val = LED_DISABLE;

	reg = S2MU106_RGB_LED_EN;
	mask = RGBLED_ENMASK << ((S2MU106_LED_NUM - col - 1) << 1);
	ret = s2mu106_update_reg(ddata->i2c, reg, val, mask);
	if (IS_ERR_VALUE(ret))
		pr_err("failed to LEDEN : %d\n", ret);
}

static int s2mu106_rgb_ramp(struct s2mu106_rgb_drvdata *ddata,
	enum led_color led, int ramp_up, int ramp_down)
{
	int ret = 0;
	int value;
	int col = ddata->pdata->col[led];

	if (ramp_up > 800)
		ramp_up = ((ramp_up - 800) >> 1) + 800;
	ramp_up /= 100;

	if (ramp_down > 800)
		ramp_down = ((ramp_down - 800) >> 1) + 800;
	ramp_down /= 100;

	value = (ramp_down) | (ramp_up << 4);
	ret = s2mu106_write_reg(ddata->i2c,
			S2MU106_RGB_LED1_RAMP + (col << 1), value);
	if (IS_ERR_VALUE(ret)) {
		pr_err("failed to write REG_LEDRMP : %d\n", ret);
		return -ENODEV;
	}

	return 0;
}

static int s2mu106_rgb_blink(struct s2mu106_rgb_drvdata *ddata,
	enum led_color led, unsigned int on, unsigned int off)
{
	int ret = 0;
	int value;
	int col = ddata->pdata->col[led];

	value = (LEDBLNK_ON(on) << 4) | LEDBLNK_OFF(off);
	ret = s2mu106_write_reg(ddata->i2c,
			S2MU106_RGB_LED1_DUR + (col << 1), value);
	if (IS_ERR_VALUE(ret)) {
		pr_err("failed to write REG_LEDBLNK : %d\n", ret);
		return -EINVAL;
	}

	pr_info("LED[%d] 0x%x\n", led, value);
	return ret;
}

static void s2mu106_rgb_reset(struct s2mu106_rgb_drvdata *ddata)
{
	int i = 0;

	for (i = 0; i < ddata->pdata->nleds; i++) {
		s2mu106_rgb_set_state(ddata, i, LED_OFF, LED_DISABLE);
		s2mu106_rgb_ramp(ddata, i, 0, 0);
	}
}

static ssize_t store_s2mu106_rgb_pattern(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct s2mu106_rgb_drvdata *ddata = dev_get_drvdata(dev);
	int mode = 0;
	int ret;

	ret = sscanf(buf, "%1d", &mode);
	if (ret == 0) {
		pr_err("fail to get led_pattern\n");
		return count;
	}
	pr_info("%s : %d lp : %u\n", __func__, mode, ddata->lpmode);
	s2mu106_rgb_reset(ddata);
	switch (mode) {
	case CHARGING:
		s2mu106_rgb_set_state(ddata, LED_RED,
			ddata->ratio[LED_RED], LED_ALWAYS_ON);
		break;
	case CHARGING_ERR:
		s2mu106_rgb_blink(ddata, LED_RED, 500, 500);
		s2mu106_rgb_set_state(ddata, LED_RED,
			ddata->ratio[LED_RED], LED_BLINK);
		break;
	case MISSED_NOTI:
		s2mu106_rgb_blink(ddata, LED_BLUE, 500, 5000);
		s2mu106_rgb_set_state(ddata, LED_BLUE,
			ddata->ratio[LED_BLUE], LED_BLINK);
		break;
	case LOW_BATTERY:
		s2mu106_rgb_blink(ddata, LED_RED, 500, 5000);
		s2mu106_rgb_set_state(ddata, LED_RED,
			ddata->ratio[LED_RED], LED_BLINK);
		break;
	case FULLY_CHARGED:
		s2mu106_rgb_set_state(ddata, LED_GREEN,
			ddata->ratio[LED_GREEN], LED_ALWAYS_ON);
		break;
	case POWERING:
		s2mu106_rgb_ramp(ddata, LED_GREEN, 800, 800);
		s2mu106_rgb_blink(ddata, LED_GREEN, 200, 200);
		s2mu106_rgb_set_state(ddata, LED_BLUE,
			ddata->ratio[LED_BLUE], LED_ALWAYS_ON);
		s2mu106_rgb_set_state(ddata, LED_GREEN,
			ddata->ratio[LED_GREEN], LED_BLINK);
		break;
	case PATTERN_OFF:
	default:
		break;
	}

	return count;
}

static ssize_t store_s2mu106_rgb_blink(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct s2mu106_rgb_drvdata *ddata = dev_get_drvdata(dev);
	int val = 0;
	int on = 0;
	int off = 0;
	int i = 0;
	u8 br[S2MU106_LED_NUM];

	if (sscanf(buf, "0x%8x %5d %5d", &val, &on, &off) != 3) {
		pr_err("fail to get led_blink value.\n");
		return count;
	}
	pr_info("%s on: %dms, off: %dms, col: 0x%x, lp: %u\n",
		__func__, on, off, val, ddata->lpmode);
	s2mu106_rgb_reset(ddata);
	for (i = LED_BLUE; i >= LED_RED; i--) {
		br[i] = val & 0xff;
		val >>= 8;
		if (br[i] > ddata->ratio[i])
			br[i] = ddata->ratio[i];
	}

	for (i = 0; i < ddata->pdata->nleds; i++) {
		if (br[i]) {
			if (!off)
				s2mu106_rgb_set_state(ddata, i, br[i], LED_ALWAYS_ON);
			else {
				s2mu106_rgb_set_state(ddata, i, br[i], LED_BLINK);
				s2mu106_rgb_blink(ddata, i, on, off);
			}
		}
	}
	return count;
}

static ssize_t store_s2mu106_rgb_brightness(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct s2mu106_rgb_drvdata *ddata = dev_get_drvdata(dev);
	int ret;
	u8 brightness;

	ret = kstrtou8(buf, 0, &brightness);
	if (ret != 0) {
		pr_err("fail to get led_brightness.\n");
		return count;
	}
	ddata->lpmode = 0;
	if (brightness > LED_MAX_CURRENT)
		brightness = LED_MAX_CURRENT;
	ddata->brightness = brightness;
	pr_info("led brightness set to %u\n", brightness);
	return count;
}

static ssize_t store_s2mu106_rgb_lowpower(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct s2mu106_rgb_drvdata *ddata = dev_get_drvdata(dev);
	struct s2mu106_rgb_pdata *pdata = ddata->pdata;
	u8 val;
	int i = 0;

	if (kstrtou8(buf, 0, &val)) {
		pr_err("fail to get led_lowpower.\n");
		return count;
	}
	ddata->lpmode = val;
	pr_info("%s set to %u\n", __func__, val);

	ddata->brightness = !val ? pdata->brightness : pdata->lp_brightness;

	for (i = 0 ; i < ddata->pdata->nleds; i++)
		ddata->ratio[i] = !val ? pdata->ratio[i] : pdata->ratio_low[i];

	return count;
}

#define ATTR_STORE_RGB(name, type)		\
static ssize_t store_led_##name(struct device *dev,	\
	struct device_attribute *devattr, const char *buf, size_t count)\
{	\
	struct s2mu106_rgb_drvdata *ddata = dev_get_drvdata(dev);	\
	u8 val;	\
	\
	if (kstrtou8(buf, 0, &val))	\
		pr_err("fail to get brightness.\n");		\
	else		\
		s2mu106_rgb_set_state(ddata, type,	\
			val,  val ? LED_ALWAYS_ON : LED_DISABLE);	\
	pr_info("%s %u\n", __func__, val);	\
	return count;		\
}	\
static DEVICE_ATTR(led_##name, 0660, NULL, store_led_##name)

ATTR_STORE_RGB(r, LED_RED);
ATTR_STORE_RGB(g, LED_GREEN);
ATTR_STORE_RGB(b, LED_BLUE);
static DEVICE_ATTR(led_pattern, 0660, NULL, store_s2mu106_rgb_pattern);
static DEVICE_ATTR(led_blink, 0660, NULL,  store_s2mu106_rgb_blink);
static DEVICE_ATTR(led_brightness, 0660, NULL, store_s2mu106_rgb_brightness);
static DEVICE_ATTR(led_lowpower, 0660, NULL,  store_s2mu106_rgb_lowpower);

static struct attribute *sec_led_attributes[] = {
	&dev_attr_led_r.attr,
	&dev_attr_led_g.attr,
	&dev_attr_led_b.attr,
	&dev_attr_led_pattern.attr,
	&dev_attr_led_blink.attr,
	&dev_attr_led_brightness.attr,
	&dev_attr_led_lowpower.attr,
	NULL,
};

static struct attribute_group sec_led_attr_group = {
	.attrs = sec_led_attributes,
};

static struct s2mu106_rgb_pdata *s2mu106_rgb_parse_dt(struct device *dev)
{
	struct device_node *node, *pp;
	struct s2mu106_rgb_pdata *pdata;
	int i;
	u32 col;
	
	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		return ERR_PTR(-ENOMEM);
	}
	pdata->octa_color = (lcdtype >> 16) & 0x0000000f;
	
	node = of_find_node_by_name(dev->parent->of_node, "s2mu106_led");
	if (unlikely(!node)) {
		pr_err("failed to find dt node\n");
		return ERR_PTR(-EINVAL);
	}
	of_property_read_u32(node, "nleds", &pdata->nleds);
	pdata->name = (const char *)of_get_property(node, "led_color", NULL);
	for (i = 0; i < pdata->nleds; i++) {
		switch (pdata->name[i]) {
		case 'R':
			pdata->col[i] = LED_RED;
			break;
		case 'G':
			pdata->col[i] = LED_GREEN;
			break;
		case 'B':
			pdata->col[i] = LED_BLUE;
			break;
		default:
			break;
		}
	}

	of_property_read_u32(node, "brightness", &pdata->brightness);
	of_property_read_u32(node, "lp_brightness", &pdata->lp_brightness);
	of_property_read_u32_array(node, "ratio",
		pdata->ratio, S2MU106_LED_NUM);
	of_property_read_u32_array(node, "ratio_low",
		pdata->ratio_low, S2MU106_LED_NUM);
	for_each_child_of_node(node, pp) {
		of_property_read_u32(pp, "octa_color", &col);
		if (pdata->octa_color != col)
			continue;
		of_property_read_u32_array(pp, "ratio",
			pdata->ratio, S2MU106_LED_NUM);
		of_property_read_u32_array(pp, "ratio_low",
			pdata->ratio_low, S2MU106_LED_NUM);
	}
	pr_info("brightness : %d, lp_brightness : %d\n",
		pdata->brightness, pdata->lp_brightness);
	pr_info("octa_color 0x%x\n", pdata->octa_color);
	for (i = 0; i < pdata->nleds; i++)
		pr_info("%c ratio : %u ratio_low : %u\n",
			pdata->name[i], pdata->ratio[i], pdata->ratio_low[i]);
	return pdata;
}

static int s2mu106_rgb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mu106_dev *s2mu106_dev = dev_get_drvdata(dev->parent);
	struct s2mu106_rgb_pdata *pdata;
	struct s2mu106_rgb_drvdata *ddata;
	int ret = 0;
	int i = 0;

	pdata = s2mu106_rgb_parse_dt(dev);
	if (unlikely(!pdata))
		return PTR_ERR(pdata);
	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (unlikely(!ddata)) {
		pr_err("out of memory drv data\n");
		return PTR_ERR(ddata);
	}
	platform_set_drvdata(pdev, ddata);
	ddata->pdata = pdata;
	ddata->brightness = pdata->brightness;
	ddata->i2c = s2mu106_dev->i2c;
	dev_set_drvdata(dev, ddata);
	for (i = 0; i < pdata->nleds; i++)
		ddata->ratio[i] = pdata->ratio[i];
	ddata->dev = sec_device_create(ddata, "led");
	if (IS_ERR(ddata->dev)) {
		pr_err("Failed to create device for samsung specific led\n");
		return PTR_ERR(ddata->dev);
	}
	ret = sysfs_create_group(&ddata->dev->kobj, &sec_led_attr_group);
	if (ret < 0) {
		pr_err("Failed to create sysfs group for samsung specific led\n");
		goto err_sysfs_create;
	}
	return 0;

err_sysfs_create:
	sec_device_destroy(ddata->dev->devt);
	return ret;
}

static int s2mu106_rgb_remove(struct platform_device *pdev)
{
	struct s2mu106_rgb_drvdata *ddata = platform_get_drvdata(pdev);

	if (!ddata->i2c)
		return 0;

	s2mu106_rgb_reset(ddata);
	sysfs_remove_group(&ddata->dev->kobj, &sec_led_attr_group);
	return 0;
}

static void s2mu106_rgb_shutdown(struct platform_device *pdev)
{
	struct s2mu106_rgb_drvdata *ddata = platform_get_drvdata(pdev);

	if (!ddata->i2c)
		return;

	s2mu106_rgb_reset(ddata);
	sysfs_remove_group(&ddata->dev->kobj, &sec_led_attr_group);
}
static struct platform_driver s2mu106_fled_driver = {
	.driver		= {
		.name	= "leds-s2mu106-rgb",
		.owner	= THIS_MODULE,
	},
	.probe		= s2mu106_rgb_probe,
	.remove		= s2mu106_rgb_remove,
	.shutdown = s2mu106_rgb_shutdown,
};
module_platform_driver(s2mu106_fled_driver);

MODULE_DESCRIPTION("s2mu106 LED driver");
MODULE_LICENSE("GPL");

