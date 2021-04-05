/*
 * Copyright (c) Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/lcd.h>
#include <linux/backlight.h>
#include <linux/of_device.h>
#include <video/mipi_display.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include "../decon.h"
#include "../decon_notify.h"
#include "../dsim.h"
#include "dsim_panel.h"

#include "hx83112a_m20_param.h"
#include "dd.h"

#define PANEL_STATE_SUSPENED	0
#define PANEL_STATE_RESUMED	1
#define PANEL_STATE_SUSPENDING	2

#define HX83112A_ID_REG			0xDA	/* LCD ID1,ID2,ID3 */
#define HX83112A_ID_LEN			3
#define BRIGHTNESS_REG			0x51

#define get_bit(value, shift, width)	((value >> shift) & (GENMASK(width - 1, 0)))

#define DSI_WRITE(cmd, size)		do {				\
	ret = dsim_write_hl_data(lcd, cmd, size);			\
	if (ret < 0)							\
		dev_info(&lcd->ld->dev, "%s: failed to write %s\n", __func__, #cmd);	\
} while (0)

struct lcd_info {
	unsigned int			connected;
	unsigned int			brightness;
	unsigned int			state;

	struct lcd_device		*ld;
	struct backlight_device		*bd;

	union {
		struct {
			u8		reserved;
			u8		id[HX83112A_ID_LEN];
		};
		u32			value;
	} id_info;

	int				lux;

	struct dsim_device		*dsim;
	struct mutex			lock;

	struct notifier_block		fb_notif_panel;
	struct i2c_client		*blic_client;
};


static int dsim_write_hl_data(struct lcd_info *lcd, const u8 *cmd, u32 cmdsize)
{
	int ret = 0;
	int retry = 2;

	if (!lcd->connected)
		return ret;

try_write:
	if (cmdsize == 1)
		ret = dsim_write_data(lcd->dsim, MIPI_DSI_DCS_SHORT_WRITE, cmd[0], 0);
	else if (cmdsize == 2)
		ret = dsim_write_data(lcd->dsim, MIPI_DSI_DCS_SHORT_WRITE_PARAM, cmd[0], cmd[1]);
	else
		ret = dsim_write_data(lcd->dsim, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)cmd, cmdsize);

	if (ret < 0) {
		if (--retry)
			goto try_write;
		else
			dev_info(&lcd->ld->dev, "%s: fail. %02x, ret: %d\n", __func__, cmd[0], ret);
	}

	return ret;
}

#if defined(CONFIG_SEC_FACTORY)
static int dsim_read_hl_data(struct lcd_info *lcd, u8 addr, u32 size, u8 *buf)
{
	int ret = 0, rx_size = 0;
	int retry = 2;

	if (!lcd->connected)
		return ret;

try_read:
	rx_size = dsim_read_data(lcd->dsim, MIPI_DSI_DCS_READ, (u32)addr, size, buf);
	dev_info(&lcd->ld->dev, "%s: %2d(%2d), %02x, %*ph%s\n", __func__, size, rx_size, addr,
		min_t(u32, min_t(u32, size, rx_size), 5), buf, (rx_size > 5) ? "..." : "");
	if (rx_size != size) {
		if (--retry)
			goto try_read;
		else {
			dev_info(&lcd->ld->dev, "%s: fail. %02x, %d(%d)\n", __func__, addr, size, rx_size);
			ret = -EPERM;
		}
	}

	return ret;
}
#endif

static int lm3632_array_write(struct i2c_client *client, u8 *ptr, u8 len)
{
	unsigned int i = 0;
	int ret = 0;
	u8 command = 0, value = 0;
	struct lcd_info *lcd = NULL;

	if (!client)
		return ret;

	lcd = i2c_get_clientdata(client);
	if (!lcd)
		return ret;

	if (!lcdtype) {
		dev_info(&lcd->ld->dev, "%s: lcdtype: %d\n", __func__, lcdtype);
		return ret;
	}

	if (len % 2) {
		dev_info(&lcd->ld->dev, "%s: length(%d) invalid\n", __func__, len);
		return ret;
	}

	for (i = 0; i < len; i += 2) {
		command = ptr[i + 0];
		value = ptr[i + 1];

		ret = i2c_smbus_write_byte_data(client, command, value);
		if (ret < 0)
			dev_info(&lcd->ld->dev, "%s: fail. %2x, %2x, %d\n", __func__, command, value, ret);
	}

	return ret;
}

static int dsim_panel_set_brightness(struct lcd_info *lcd, int force)
{
	int ret = 0;
	unsigned char bl_reg[3] = {0, };

	mutex_lock(&lcd->lock);

	lcd->brightness = lcd->bd->props.brightness;

	if (!force && lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active state\n", __func__);
		goto exit;
	}

	bl_reg[0] = BRIGHTNESS_REG;
	bl_reg[1] = get_bit(brightness_table[lcd->brightness], 8, 4);
	bl_reg[2] = get_bit(brightness_table[lcd->brightness], 0, 8);

	DSI_WRITE(bl_reg, ARRAY_SIZE(bl_reg));

	dev_info(&lcd->ld->dev, "%s: brightness: %3d, %4d(%2x %2x), lx: %d\n", __func__,
		lcd->brightness, brightness_table[lcd->brightness], bl_reg[1], bl_reg[2], lcd->lux);

exit:
	mutex_unlock(&lcd->lock);

	return ret;
}

static int panel_get_brightness(struct backlight_device *bd)
{
	struct lcd_info *lcd = bl_get_data(bd);

	return brightness_table[lcd->brightness];
}

static int panel_set_brightness(struct backlight_device *bd)
{
	int ret = 0;
	struct lcd_info *lcd = bl_get_data(bd);

	if (lcd->state == PANEL_STATE_RESUMED) {
		ret = dsim_panel_set_brightness(lcd, 0);
		if (ret < 0)
			dev_info(&lcd->ld->dev, "%s: failed to set brightness\n", __func__);
	}

	return ret;
}

static const struct backlight_ops panel_backlight_ops = {
	.get_brightness = panel_get_brightness,
	.update_status = panel_set_brightness,
};

static int hx83112a_read_init_info(struct lcd_info *lcd)
{
	struct panel_private *priv = &lcd->dsim->priv;

	priv->lcdconnected = lcd->connected = lcdtype ? 1 : 0;

	lcd->id_info.id[0] = (lcdtype & 0xFF0000) >> 16;
	lcd->id_info.id[1] = (lcdtype & 0x00FF00) >> 8;
	lcd->id_info.id[2] = (lcdtype & 0x0000FF) >> 0;

	dev_info(&lcd->ld->dev, "%s: %x\n", __func__, cpu_to_be32(lcd->id_info.value));

	return 0;
}

#if defined(CONFIG_SEC_FACTORY)
static int hx83112a_read_id(struct lcd_info *lcd)
{
	struct panel_private *priv = &lcd->dsim->priv;
	int i, ret = 0;
	struct decon_device *decon = get_decon_drvdata(0);
	static char *LDI_BIT_DESC_ID[BITS_PER_BYTE * HX83112A_ID_LEN] = {
		[0 ... 23] = "ID Read Fail",
	};

	lcd->id_info.value = 0;
	priv->lcdconnected = lcd->connected = lcdtype ? 1 : 0;

	for (i = 0; i < HX83112A_ID_LEN; i++) {
		ret = dsim_read_hl_data(lcd, HX83112A_ID_REG + i, 1, &lcd->id_info.id[i]);
		if (ret < 0)
			break;
	}

	if (ret < 0 || !lcd->id_info.value) {
		priv->lcdconnected = lcd->connected = 0;
		dev_info(&lcd->ld->dev, "%s: connected lcd is invalid\n", __func__);

		if (lcdtype && decon)
			decon_abd_save_bit(&decon->abd, BITS_PER_BYTE * HX83112A_ID_LEN, cpu_to_be32(lcd->id_info.value), LDI_BIT_DESC_ID);
	}

	dev_info(&lcd->ld->dev, "%s: %x\n", __func__, cpu_to_be32(lcd->id_info.value));

	return ret;
}
#endif

static int hx83112a_displayon_late(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	DSI_WRITE(SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));

	return ret;
}

static int hx83112a_exit(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	DSI_WRITE(SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	msleep(20);
	DSI_WRITE(SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));
	msleep(70);

	return ret;
}

static int hx83112a_init(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s: ++\n", __func__);

#if defined(CONFIG_SEC_FACTORY)
	hx83112a_read_id(lcd);
#endif

	DSI_WRITE(SEQ_SET_EXTC, ARRAY_SIZE(SEQ_SET_EXTC));
	DSI_WRITE(SEQ_SET_E9_C4, ARRAY_SIZE(SEQ_SET_E9_C4));
	DSI_WRITE(SEQ_SET_STARTEN, ARRAY_SIZE(SEQ_SET_STARTEN));
	DSI_WRITE(SEQ_SET_E9_3F, ARRAY_SIZE(SEQ_SET_E9_3F));
	DSI_WRITE(SEQ_SET_PWM_FREQ, ARRAY_SIZE(SEQ_SET_PWM_FREQ));
	DSI_WRITE(SEQ_HX83112A_BL, ARRAY_SIZE(SEQ_HX83112A_BL));
	DSI_WRITE(SEQ_HX83112A_BLON, ARRAY_SIZE(SEQ_HX83112A_BLON));
	DSI_WRITE(SEQ_HX83112A_55, ARRAY_SIZE(SEQ_HX83112A_55));
	usleep_range(10000, 11000); /*10ms*/
	DSI_WRITE(SEQ_SET_E4, ARRAY_SIZE(SEQ_SET_E4));
	DSI_WRITE(SEQ_SET_OTP, ARRAY_SIZE(SEQ_SET_OTP));
	DSI_WRITE(SEQ_SET_BANK_01, ARRAY_SIZE(SEQ_SET_BANK_01));
	DSI_WRITE(SEQ_SET_E9_DB, ARRAY_SIZE(SEQ_SET_E9_DB));
	DSI_WRITE(SEQ_SET_E4_00, ARRAY_SIZE(SEQ_SET_E4_00));
	DSI_WRITE(SEQ_SET_E9_3F, ARRAY_SIZE(SEQ_SET_E9_3F));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_POWER_SETTING, ARRAY_SIZE(SEQ_POWER_SETTING));
	DSI_WRITE(SEQ_DISPLAY_REGISTER, ARRAY_SIZE(SEQ_DISPLAY_REGISTER));
	DSI_WRITE(SEQ_PANEL_DRIVING_TIMING, ARRAY_SIZE(SEQ_PANEL_DRIVING_TIMING));
	DSI_WRITE(SEQ_SET_BANK_01, ARRAY_SIZE(SEQ_SET_BANK_01));
	DSI_WRITE(SEQ_SET_APO, ARRAY_SIZE(SEQ_SET_APO));
	DSI_WRITE(SEQ_SET_FRE_PATTERN, ARRAY_SIZE(SEQ_SET_FRE_PATTERN));
	DSI_WRITE(SEQ_SET_BANK_02, ARRAY_SIZE(SEQ_SET_BANK_02));
	DSI_WRITE(SEQ_SET_B4, ARRAY_SIZE(SEQ_SET_B4));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_SET_BF, ARRAY_SIZE(SEQ_SET_BF));
	DSI_WRITE(SEQ_SOURCE_BIAS_GAMMA, ARRAY_SIZE(SEQ_SOURCE_BIAS_GAMMA));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_SET_C7, ARRAY_SIZE(SEQ_SET_C7));
	DSI_WRITE(SEQ_SET_S1, ARRAY_SIZE(SEQ_SET_S1));
	DSI_WRITE(SEQ_SET_POWER, ARRAY_SIZE(SEQ_SET_POWER));
	DSI_WRITE(SEQ_SET_GIP_D3, ARRAY_SIZE(SEQ_SET_GIP_D3));
	DSI_WRITE(SEQ_SET_GIP_D5, ARRAY_SIZE(SEQ_SET_GIP_D5));
	DSI_WRITE(SEQ_SET_GIP_D6, ARRAY_SIZE(SEQ_SET_GIP_D6));
	DSI_WRITE(SEQ_SET_GIP_D8_POWER_ON, ARRAY_SIZE(SEQ_SET_GIP_D8_POWER_ON));
	DSI_WRITE(SEQ_SET_BANK_01, ARRAY_SIZE(SEQ_SET_BANK_01));
	DSI_WRITE(SEQ_SET_GIP_D8_01, ARRAY_SIZE(SEQ_SET_GIP_D8_01));
	DSI_WRITE(SEQ_SET_BANK_02, ARRAY_SIZE(SEQ_SET_BANK_02));
	DSI_WRITE(SEQ_SET_GIP_D8_02, ARRAY_SIZE(SEQ_SET_GIP_D8_02));
	DSI_WRITE(SEQ_SET_BANK_03, ARRAY_SIZE(SEQ_SET_BANK_03));
	DSI_WRITE(SEQ_SET_GIP_D8_03_LPWG, ARRAY_SIZE(SEQ_SET_GIP_D8_03_LPWG));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_SET_GIP_E7_00_TOUCH, ARRAY_SIZE(SEQ_SET_GIP_E7_00_TOUCH));
	DSI_WRITE(SEQ_SET_BANK_01, ARRAY_SIZE(SEQ_SET_BANK_01));
	DSI_WRITE(SEQ_SET_GIP_E7_01_TOUCH, ARRAY_SIZE(SEQ_SET_GIP_E7_01_TOUCH));
	DSI_WRITE(SEQ_SET_BANK_02, ARRAY_SIZE(SEQ_SET_BANK_02));
	DSI_WRITE(SEQ_SET_GIP_E7_02_TOUCH, ARRAY_SIZE(SEQ_SET_GIP_E7_02_TOUCH));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_SET_E9_C3, ARRAY_SIZE(SEQ_SET_E9_C3));
	DSI_WRITE(SEQ_SET_CB, ARRAY_SIZE(SEQ_SET_CB));
	DSI_WRITE(SEQ_SET_E9_3F, ARRAY_SIZE(SEQ_SET_E9_3F));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_SET_E9_E8, ARRAY_SIZE(SEQ_SET_E9_E8));
	DSI_WRITE(SEQ_SET_D3_00, ARRAY_SIZE(SEQ_SET_D3_00));
	DSI_WRITE(SEQ_SET_E9_3F, ARRAY_SIZE(SEQ_SET_E9_3F));
	DSI_WRITE(SEQ_SET_BANK_01, ARRAY_SIZE(SEQ_SET_BANK_01));
	DSI_WRITE(SEQ_SET_E9_C6, ARRAY_SIZE(SEQ_SET_E9_C6));
	DSI_WRITE(SEQ_SET_D3_01, ARRAY_SIZE(SEQ_SET_D3_01));
	DSI_WRITE(SEQ_SET_E9_3F, ARRAY_SIZE(SEQ_SET_E9_3F));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_SET_C1_BANK_00, ARRAY_SIZE(SEQ_SET_C1_BANK_00));
	DSI_WRITE(SEQ_SET_BANK_01, ARRAY_SIZE(SEQ_SET_BANK_01));
	DSI_WRITE(SEQ_SET_C1_BANK_01_R, ARRAY_SIZE(SEQ_SET_C1_BANK_01_R));
	DSI_WRITE(SEQ_SET_BANK_02, ARRAY_SIZE(SEQ_SET_BANK_02));
	DSI_WRITE(SEQ_SET_C1_BANK_02_G, ARRAY_SIZE(SEQ_SET_C1_BANK_02_G));
	DSI_WRITE(SEQ_SET_BANK_03, ARRAY_SIZE(SEQ_SET_BANK_03));
	DSI_WRITE(SEQ_SET_C1_BANK_03_B, ARRAY_SIZE(SEQ_SET_C1_BANK_03_B));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_SET_E3_BANK_00_G, ARRAY_SIZE(SEQ_SET_E3_BANK_00_G));
	DSI_WRITE(SEQ_SET_BANK_01, ARRAY_SIZE(SEQ_SET_BANK_01));
	DSI_WRITE(SEQ_SET_E3_BANK_01_R, ARRAY_SIZE(SEQ_SET_E3_BANK_01_R));
	DSI_WRITE(SEQ_SET_BANK_02, ARRAY_SIZE(SEQ_SET_BANK_02));
	DSI_WRITE(SEQ_SET_E3_BANK_02_G, ARRAY_SIZE(SEQ_SET_E3_BANK_02_G));
	DSI_WRITE(SEQ_SET_BANK_03, ARRAY_SIZE(SEQ_SET_BANK_03));
	DSI_WRITE(SEQ_SET_E3_BANK_03_B, ARRAY_SIZE(SEQ_SET_E3_BANK_03_B));
	DSI_WRITE(SEQ_SET_BANK_00, ARRAY_SIZE(SEQ_SET_BANK_00));
	DSI_WRITE(SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	msleep(100); /*100ms*/
	dev_info(&lcd->ld->dev, "%s: --\n", __func__);

	return ret;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct lcd_info *lcd = NULL;
	int fb_blank;

	switch (event) {
	case FB_EVENT_BLANK:
		break;
	default:
		return NOTIFY_DONE;
	}

	lcd = container_of(self, struct lcd_info, fb_notif_panel);

	fb_blank = *(int *)evdata->data;

	dev_info(&lcd->ld->dev, "%s: %d\n", __func__, fb_blank);

	if (evdata->info->node)
		return NOTIFY_DONE;

	if (fb_blank == FB_BLANK_UNBLANK) {
		mutex_lock(&lcd->lock);
		hx83112a_displayon_late(lcd);
		mutex_unlock(&lcd->lock);

		dsim_panel_set_brightness(lcd, 1);
	}

	return NOTIFY_DONE;
}

static int lm3632_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct lcd_info *lcd = NULL;
	int ret = 0;

	if (id && id->driver_data)
		lcd = (struct lcd_info *)id->driver_data;

	if (!lcd) {
		dsim_err("%s: failed to find driver_data for lcd\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_info(&lcd->ld->dev, "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	i2c_set_clientdata(client, lcd);

	lcd->blic_client = client;

	dev_info(&lcd->ld->dev, "%s: %s %s\n", __func__, dev_name(&client->adapter->dev), of_node_full_name(client->dev.of_node));

exit:
	return ret;
}

static struct i2c_device_id blic_i2c_id[] = {
	{"lm3632", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, blic_i2c_id);

static const struct of_device_id blic_i2c_dt_ids[] = {
	{ .compatible = "i2c,lm3632" },
	{ }
};

MODULE_DEVICE_TABLE(of, blic_i2c_dt_ids);

static struct i2c_driver blic_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lm3632",
		.of_match_table	= of_match_ptr(blic_i2c_dt_ids),
	},
	.id_table = blic_i2c_id,
	.probe = lm3632_probe,
};

static int hx83112a_probe(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	lcd->bd->props.max_brightness = EXTEND_BRIGHTNESS;
	lcd->bd->props.brightness = UI_DEFAULT_BRIGHTNESS;

	lcd->state = PANEL_STATE_RESUMED;
	lcd->lux = -1;

	ret = hx83112a_read_init_info(lcd);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: failed to init information\n", __func__);

	lcd->fb_notif_panel.notifier_call = fb_notifier_callback;
	decon_register_notifier(&lcd->fb_notif_panel);

	blic_i2c_id->driver_data = (kernel_ulong_t)lcd;
	i2c_add_driver(&blic_i2c_driver);

	dev_info(&lcd->ld->dev, "- %s\n", __func__);

	return 0;
}

static ssize_t lcd_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	sprintf(buf, "TIA_%02X%02X%02X\n", lcd->id_info.id[0], lcd->id_info.id[1], lcd->id_info.id[2]);

	return strlen(buf);
}

static ssize_t window_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	sprintf(buf, "%02x %02x %02x\n", lcd->id_info.id[0], lcd->id_info.id[1], lcd->id_info.id[2]);

	return strlen(buf);
}

static ssize_t lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	sprintf(buf, "%d\n", lcd->lux);

	return strlen(buf);
}

static ssize_t lux_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	int value;
	int rc;

	rc = kstrtoint(buf, 0, &value);
	if (rc < 0)
		return rc;

	if (lcd->lux != value) {
		mutex_lock(&lcd->lock);
		lcd->lux = value;
		mutex_unlock(&lcd->lock);
	}

	return size;
}

static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);
static DEVICE_ATTR(window_type, 0444, window_type_show, NULL);
static DEVICE_ATTR(lux, 0644, lux_show, lux_store);

static struct attribute *lcd_sysfs_attributes[] = {
	&dev_attr_lcd_type.attr,
	&dev_attr_window_type.attr,
	&dev_attr_lux.attr,
	NULL,
};

static const struct attribute_group lcd_sysfs_attr_group = {
	.attrs = lcd_sysfs_attributes,
};

static void lcd_init_sysfs(struct lcd_info *lcd)
{
	int ret = 0;
	struct i2c_client *clients[] = {lcd->blic_client, NULL};

	ret = sysfs_create_group(&lcd->ld->dev.kobj, &lcd_sysfs_attr_group);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "failed to add lcd sysfs\n");

	init_debugfs_backlight(lcd->bd, brightness_table, clients);

	init_debugfs_param("blic_init", &LM3632_INIT, U8_MAX, ARRAY_SIZE(LM3632_INIT), 2);
}

static int dsim_panel_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd;

	dsim->priv.par = lcd = kzalloc(sizeof(struct lcd_info), GFP_KERNEL);
	if (!lcd) {
		pr_err("%s: failed to allocate for lcd\n", __func__);
		ret = -ENOMEM;
		goto probe_err;
	}

	lcd->ld = lcd_device_register("panel", dsim->dev, lcd, NULL);
	if (IS_ERR(lcd->ld)) {
		pr_err("%s: failed to register lcd device\n", __func__);
		ret = PTR_ERR(lcd->ld);
		goto probe_err;
	}

	lcd->bd = backlight_device_register("panel", dsim->dev, lcd, &panel_backlight_ops, NULL);
	if (IS_ERR(lcd->bd)) {
		pr_err("%s: failed to register backlight device\n", __func__);
		ret = PTR_ERR(lcd->bd);
		goto probe_err;
	}

	mutex_init(&lcd->lock);

	lcd->dsim = dsim;
	ret = hx83112a_probe(lcd);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: failed to probe panel\n", __func__);

	lcd_init_sysfs(lcd);
	dev_info(&lcd->ld->dev, "%s: %s: done\n", kbasename(__FILE__), __func__);
probe_err:
	return ret;
}

static int dsim_panel_resume_early(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	lm3632_array_write(lcd->blic_client, LM3632_INIT, ARRAY_SIZE(LM3632_INIT));

	dev_info(&lcd->ld->dev, "- %s: %d, %d\n", __func__, lcd->state, lcd->connected);

	return 0;
}

static int dsim_panel_displayon(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "+ %s: %d\n", __func__, lcd->state);

	if (lcd->state == PANEL_STATE_SUSPENED)
		hx83112a_init(lcd);

	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_RESUMED;
	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "- %s: %d, %d\n", __func__, lcd->state, lcd->connected);

	return 0;
}

static int dsim_panel_suspend(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "+ %s: %d\n", __func__, lcd->state);

	if (lcd->state == PANEL_STATE_SUSPENED)
		goto exit;

	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_SUSPENDING;
	mutex_unlock(&lcd->lock);

	hx83112a_exit(lcd);

	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_SUSPENED;
	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "- %s: %d, %d\n", __func__, lcd->state, lcd->connected);

exit:
	return 0;
}

struct dsim_lcd_driver hx83112a_mipi_lcd_driver = {
	.name		= "hx83112a",
	.probe		= dsim_panel_probe,
	.resume_early	= dsim_panel_resume_early,
	.displayon	= dsim_panel_displayon,
	.suspend	= dsim_panel_suspend,
};
__XX_ADD_LCD_DRIVER(hx83112a_mipi_lcd_driver);

