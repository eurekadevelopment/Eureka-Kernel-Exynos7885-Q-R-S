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

#if defined(CONFIG_SEC_INCELL)
#include <linux/sec_incell.h>
#endif

#include "../decon.h"
#include "../decon_notify.h"
#include "../dsim.h"
#include "dsim_panel.h"

#include "td4101_j2corepelte_param.h"
#include "dd.h"

#if defined(CONFIG_EXYNOS_DECON_MDNIE)
#include "mdnie.h"
#include "td4101_j2corepelte_mdnie.h"
#endif

#define PANEL_STATE_SUSPENED	0
#define PANEL_STATE_RESUMED	1
#define PANEL_STATE_SUSPENDING	2

#define TD4101_ID_REG			0xDA	/* LCD ID1,ID2,ID3 */
#define TD4101_ID_LEN			3
#define BRIGHTNESS_REG			0x51

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
			u8		id[TD4101_ID_LEN];
		};
		u32			value;
	} id_info;

	int				lux;
	struct class			*mdnie_class;

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

#if defined(CONFIG_SEC_FACTORY) || defined(CONFIG_EXYNOS_DECON_MDNIE)
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

#if defined(CONFIG_EXYNOS_DECON_MDNIE)
static int dsim_write_set(struct lcd_info *lcd, struct lcd_seq_info *seq, u32 num)
{
	int ret = 0, i;

	for (i = 0; i < num; i++) {
		if (seq[i].cmd) {
			ret = dsim_write_hl_data(lcd, seq[i].cmd, seq[i].len);
			if (ret < 0) {
				dev_info(&lcd->ld->dev, "%s: %dth fail\n", __func__, i);
				return ret;
			}
		}
		if (seq[i].sleep)
			usleep_range(seq[i].sleep * 1000, seq[i].sleep * 1100);
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
	unsigned char bl_reg[2] = {0, };

	mutex_lock(&lcd->lock);

	lcd->brightness = lcd->bd->props.brightness;

	if (!force && lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active state\n", __func__);
		goto exit;
	}

	bl_reg[0] = BRIGHTNESS_REG;
	bl_reg[1] = brightness_table[lcd->brightness];

	DSI_WRITE(bl_reg, ARRAY_SIZE(bl_reg));
	dev_info(&lcd->ld->dev, "%s: brightness: %3d, %3d(%2x), lx: %d\n", __func__,
		lcd->brightness, brightness_table[lcd->brightness], bl_reg[1], lcd->lux);
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

static int td4101_read_init_info(struct lcd_info *lcd)
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
static int td4101_read_id(struct lcd_info *lcd)
{
	struct panel_private *priv = &lcd->dsim->priv;
	int i, ret = 0;
	struct decon_device *decon = get_decon_drvdata(0);
	static char *LDI_BIT_DESC_ID[BITS_PER_BYTE * TD4101_ID_LEN] = {
		[0 ... 23] = "ID Read Fail",
	};

	lcd->id_info.value = 0;
	priv->lcdconnected = lcd->connected = lcdtype ? 1 : 0;

	/* Read REQ packet is sent by LPDT */
	dsim_reg_set_cmd_transfer_mode(0, 1);

	for (i = 0; i < TD4101_ID_LEN; i++) {
		ret = dsim_read_hl_data(lcd, TD4101_ID_REG + i, 1, &lcd->id_info.id[i]);
		if (ret < 0)
			break;
	}

	dsim_reg_set_cmd_transfer_mode(0, 0);

	if (ret < 0 || !lcd->id_info.value) {
		priv->lcdconnected = lcd->connected = 0;
		dev_info(&lcd->ld->dev, "%s: connected lcd is invalid\n", __func__);

		if (lcdtype && decon)
			decon_abd_save_bit(&decon->abd, BITS_PER_BYTE * TD4101_ID_LEN, cpu_to_be32(lcd->id_info.value), LDI_BIT_DESC_ID);
	}

	dev_info(&lcd->ld->dev, "%s: %x\n", __func__, cpu_to_be32(lcd->id_info.value));

	return ret;
}
#endif

static int td4101_displayon_late(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	msleep(40);

	DSI_WRITE(SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));
	DSI_WRITE(SEQ_TD4101_BLON, ARRAY_SIZE(SEQ_TD4101_BLON));

	return ret;
}

static int td4101_exit(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	DSI_WRITE(SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));

	msleep(20);

	DSI_WRITE(SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));

	msleep(90);

	return ret;
}

static int td4101_init(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s: ++\n", __func__);

#if defined(CONFIG_SEC_FACTORY)
	td4101_read_id(lcd);
#endif
	/* DSI Lane Video setting is sent by LPDT */
	dsim_reg_set_cmd_transfer_mode(0, 1);

	DSI_WRITE(SEQ_TD4101_B0, ARRAY_SIZE(SEQ_TD4101_B0));
	DSI_WRITE(SEQ_TD4101_B3, ARRAY_SIZE(SEQ_TD4101_B3));
	DSI_WRITE(SEQ_TD4101_B4, ARRAY_SIZE(SEQ_TD4101_B4));
	usleep_range(1000, 1100);
	DSI_WRITE(SEQ_TD4101_B6, ARRAY_SIZE(SEQ_TD4101_B6));

	dsim_reg_set_cmd_transfer_mode(0, 0);

	DSI_WRITE(SEQ_TD4101_B5, ARRAY_SIZE(SEQ_TD4101_B5));
	DSI_WRITE(SEQ_TD4101_B7, ARRAY_SIZE(SEQ_TD4101_B7));
	DSI_WRITE(SEQ_TD4101_BA, ARRAY_SIZE(SEQ_TD4101_BA));
	DSI_WRITE(SEQ_TD4101_BB, ARRAY_SIZE(SEQ_TD4101_BB));
	DSI_WRITE(SEQ_TD4101_BC, ARRAY_SIZE(SEQ_TD4101_BC));
	DSI_WRITE(SEQ_TD4101_BD, ARRAY_SIZE(SEQ_TD4101_BD));
	DSI_WRITE(SEQ_TD4101_BE, ARRAY_SIZE(SEQ_TD4101_BE));
	DSI_WRITE(SEQ_TD4101_BF, ARRAY_SIZE(SEQ_TD4101_BF));
	DSI_WRITE(SEQ_TD4101_C0, ARRAY_SIZE(SEQ_TD4101_C0));
	DSI_WRITE(SEQ_TD4101_C1, ARRAY_SIZE(SEQ_TD4101_C1));
	DSI_WRITE(SEQ_TD4101_C2, ARRAY_SIZE(SEQ_TD4101_C2));
	DSI_WRITE(SEQ_TD4101_C3, ARRAY_SIZE(SEQ_TD4101_C3));
	DSI_WRITE(SEQ_TD4101_C4, ARRAY_SIZE(SEQ_TD4101_C4));
	DSI_WRITE(SEQ_TD4101_C5, ARRAY_SIZE(SEQ_TD4101_C5));
	DSI_WRITE(SEQ_TD4101_C6, ARRAY_SIZE(SEQ_TD4101_C6));
	DSI_WRITE(SEQ_TD4101_C7, ARRAY_SIZE(SEQ_TD4101_C7));
	DSI_WRITE(SEQ_TD4101_C8, ARRAY_SIZE(SEQ_TD4101_C8));
	DSI_WRITE(SEQ_TD4101_C9, ARRAY_SIZE(SEQ_TD4101_C9));
	DSI_WRITE(SEQ_TD4101_CA, ARRAY_SIZE(SEQ_TD4101_CA));
	DSI_WRITE(SEQ_TD4101_CC, ARRAY_SIZE(SEQ_TD4101_CC));
	DSI_WRITE(SEQ_TD4101_CD, ARRAY_SIZE(SEQ_TD4101_CD));
	DSI_WRITE(SEQ_TD4101_CE, ARRAY_SIZE(SEQ_TD4101_CE));
	DSI_WRITE(SEQ_TD4101_CF, ARRAY_SIZE(SEQ_TD4101_CF));
	DSI_WRITE(SEQ_TD4101_D0, ARRAY_SIZE(SEQ_TD4101_D0));
	DSI_WRITE(SEQ_TD4101_D1, ARRAY_SIZE(SEQ_TD4101_D1));
	DSI_WRITE(SEQ_TD4101_D3, ARRAY_SIZE(SEQ_TD4101_D3));
	DSI_WRITE(SEQ_TD4101_D4, ARRAY_SIZE(SEQ_TD4101_D4));
	DSI_WRITE(SEQ_TD4101_D6, ARRAY_SIZE(SEQ_TD4101_D6));
	DSI_WRITE(SEQ_TD4101_D7, ARRAY_SIZE(SEQ_TD4101_D7));
	DSI_WRITE(SEQ_TD4101_D8, ARRAY_SIZE(SEQ_TD4101_D8));
	DSI_WRITE(SEQ_TD4101_DD, ARRAY_SIZE(SEQ_TD4101_DD));
	DSI_WRITE(SEQ_TD4101_DE, ARRAY_SIZE(SEQ_TD4101_DE));

	DSI_WRITE(SEQ_TD4101_BLON, ARRAY_SIZE(SEQ_TD4101_BLON));
	DSI_WRITE(SEQ_TD4101_CABC_MIN, ARRAY_SIZE(SEQ_TD4101_CABC_MIN));
	DSI_WRITE(SEQ_TD4101_CABC_MODE, ARRAY_SIZE(SEQ_TD4101_CABC_MODE));
	DSI_WRITE(SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));

	msleep(100);

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
		td4101_displayon_late(lcd);
		mutex_unlock(&lcd->lock);

		dsim_panel_set_brightness(lcd, 1);
	}

	return NOTIFY_DONE;
}

#if defined(CONFIG_SEC_INCELL)
static void incell_blank_unblank(void *drv_data)
{
	struct fb_info *info = registered_fb[0];
	struct decon_device *decon = get_decon_drvdata(0);
	struct decon_mode_info psr;

	decon_to_psr_info(decon, &psr);

	dsim_info("+ %s\n", __func__);

	if (!lock_fb_info(info))
		return;

	if (decon->state == DECON_STATE_OFF) {
		dsim_info("decon status is inactive\n");
		goto exit;
	}

	info->flags |= FBINFO_MISC_USEREVENT;
	decon->esd_recovery = 1;
	decon->ignore_vsync = 1;
	fb_blank(info, FB_BLANK_POWERDOWN);
	fb_blank(info, FB_BLANK_UNBLANK);
	decon->esd_recovery = 0;
	decon->ignore_vsync = 0;
	info->flags &= ~FBINFO_MISC_USEREVENT;
exit:
	unlock_fb_info(info);

	dsim_info("- %s\n", __func__);
}
#endif

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

static struct i2c_device_id lm3632_i2c_id[] = {
	{"lm3632", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lm3632_i2c_id);

static const struct of_device_id lm3632_i2c_dt_ids[] = {
	{ .compatible = "i2c,lm3632" },
	{ }
};

MODULE_DEVICE_TABLE(of, lm3632_i2c_dt_ids);

static struct i2c_driver lm3632_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lm3632",
		.of_match_table	= of_match_ptr(lm3632_i2c_dt_ids),
	},
	.id_table = lm3632_i2c_id,
	.probe = lm3632_probe,
};

static int td4101_probe(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	lcd->bd->props.max_brightness = EXTEND_BRIGHTNESS;
	lcd->bd->props.brightness = UI_DEFAULT_BRIGHTNESS;

	lcd->state = PANEL_STATE_RESUMED;
	lcd->lux = -1;

	ret = td4101_read_init_info(lcd);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: failed to init information\n", __func__);

	lcd->fb_notif_panel.notifier_call = fb_notifier_callback;
	decon_register_notifier(&lcd->fb_notif_panel);

#if defined(CONFIG_SEC_INCELL)
	incell_data.blank_unblank = incell_blank_unblank;
#endif

	lm3632_i2c_id->driver_data = (kernel_ulong_t)lcd;
	i2c_add_driver(&lm3632_i2c_driver);

	dev_info(&lcd->ld->dev, "- %s\n", __func__);

	return 0;
}

static ssize_t lcd_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	sprintf(buf, "BOE_%02X%02X%02X\n", lcd->id_info.id[0], lcd->id_info.id[1], lcd->id_info.id[2]);

	return strlen(buf);
}

static ssize_t window_type_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);

	sprintf(buf, "%02x %02x %02x\n", lcd->id_info.id[0], lcd->id_info.id[1], lcd->id_info.id[2]);

	return strlen(buf);
}

static ssize_t brightness_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i;
	char *pos = buf;

	for (i = 0; i <= EXTEND_BRIGHTNESS; i++)
		pos += sprintf(pos, "%3d %3d\n", i, brightness_table[i]);

	return pos - buf;
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

#if defined(CONFIG_EXYNOS_DECON_MDNIE)
		attr_store_for_each(lcd->mdnie_class, attr->attr.name, buf, size);
#endif
	}

	return size;
}

static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);
static DEVICE_ATTR(window_type, 0444, window_type_show, NULL);
static DEVICE_ATTR(brightness_table, 0444, brightness_table_show, NULL);
static DEVICE_ATTR(lux, 0644, lux_show, lux_store);

static struct attribute *lcd_sysfs_attributes[] = {
	&dev_attr_lcd_type.attr,
	&dev_attr_window_type.attr,
	&dev_attr_brightness_table.attr,
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
}

#if defined(CONFIG_EXYNOS_DECON_MDNIE)
static int mdnie_send_seq(struct lcd_info *lcd, struct lcd_seq_info *seq, u32 num)
{
	int ret = 0;

	if (lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active\n", __func__);
		return -EIO;
	}

	mutex_lock(&lcd->lock);
	ret = dsim_write_set(lcd, seq, num);
	mutex_unlock(&lcd->lock);

	return ret;
}

static int mdnie_read(struct lcd_info *lcd, u8 addr, u8 *buf, u32 size)
{
	int ret = 0;

	if (lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active\n", __func__);
		return -EIO;
	}

	mutex_lock(&lcd->lock);
	ret = dsim_read_hl_data(lcd, addr, size, buf);
	mutex_unlock(&lcd->lock);

	return ret;
}
#endif

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
	ret = td4101_probe(lcd);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: failed to probe panel\n", __func__);

	lcd_init_sysfs(lcd);

#if defined(CONFIG_EXYNOS_DECON_MDNIE)
	mdnie_register(&lcd->ld->dev, lcd, (mdnie_w)mdnie_send_seq, (mdnie_r)mdnie_read, NULL, &tune_info);
	lcd->mdnie_class = get_mdnie_class();
#endif

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
		td4101_init(lcd);

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

	td4101_exit(lcd);

	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_SUSPENED;
	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "- %s: %d, %d\n", __func__, lcd->state, lcd->connected);

exit:
	return 0;
}

struct dsim_lcd_driver td4101_mipi_lcd_driver = {
	.name		= "td4101",
	.probe		= dsim_panel_probe,
	.resume_early	= dsim_panel_resume_early,
	.displayon	= dsim_panel_displayon,
	.suspend	= dsim_panel_suspend,
};
__XX_ADD_LCD_DRIVER(td4101_mipi_lcd_driver);

