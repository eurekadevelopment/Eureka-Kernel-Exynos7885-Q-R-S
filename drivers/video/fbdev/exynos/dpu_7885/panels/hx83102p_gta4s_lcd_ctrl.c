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
#include <linux/module.h>
#include <linux/gpio.h>

#include "../decon.h"
#include "../decon_board.h"
#include "../decon_notify.h"
#include "../dsim.h"
#include "dsim_panel.h"

#include "hx83102p_gta4s_param.h"
#include "dd.h"

#define PANEL_STATE_SUSPENED	0
#define PANEL_STATE_RESUMED	1
#define PANEL_STATE_SUSPENDING	2

#define HX83102P_ID_REG			0xDA	/* LCD ID1,ID2,ID3 */
#define HX83102P_ID_LEN			3
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
	unsigned int			cabc_check;

	struct lcd_device		*ld;
	struct backlight_device		*bd;

	union {
		struct {
			u8		reserved;
			u8		id[HX83102P_ID_LEN];
		};
		u32			value;
	} id_info;

	int				lux;

	struct dsim_device		*dsim;
	struct mutex			lock;

	struct notifier_block		fb_notif_panel;

	unsigned int			conn_init_done;
	unsigned int			conn_det_enable;
	unsigned int			conn_det_count;

	struct workqueue_struct		*conn_workqueue;
	struct work_struct		conn_work;
};

static int dsim_write_hl_data(struct lcd_info *lcd, const u8 *cmd, u32 cmdsize)
{
	int ret = 0;
	int retry = 2;

	if (!lcd->connected)
		return ret;

try_write:
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

	if (lcd->brightness < 70 && lcd->cabc_check) {
		DSI_WRITE(SEQ_SET_B9_PW, ARRAY_SIZE(SEQ_SET_B9_PW));
		DSI_WRITE(SEQ_SET_HX83102P_CABC_OFF, ARRAY_SIZE(SEQ_SET_HX83102P_CABC_OFF));
		DSI_WRITE(SEQ_SET_B9_CLOSE_PW, ARRAY_SIZE(SEQ_SET_B9_CLOSE_PW));
		lcd->cabc_check = 0;
	} else if (lcd->brightness >= 70 && !lcd->cabc_check) {
		DSI_WRITE(SEQ_SET_B9_PW, ARRAY_SIZE(SEQ_SET_B9_PW));
		DSI_WRITE(SEQ_SET_HX83102P_CABC_ON, ARRAY_SIZE(SEQ_SET_HX83102P_CABC_ON));
		DSI_WRITE(SEQ_SET_B9_CLOSE_PW, ARRAY_SIZE(SEQ_SET_B9_CLOSE_PW));
		lcd->cabc_check = 1;
	}
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

static int hx83102p_read_init_info(struct lcd_info *lcd)
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
static int hx83102p_read_id(struct lcd_info *lcd)
{
	struct panel_private *priv = &lcd->dsim->priv;
	int i, ret = 0;
	struct decon_device *decon = get_decon_drvdata(0);
	static char *LDI_BIT_DESC_ID[BITS_PER_BYTE * HX83102P_ID_LEN] = {
		[0 ... 23] = "ID Read Fail",
	};

	lcd->id_info.value = 0;
	priv->lcdconnected = lcd->connected = lcdtype ? 1 : 0;

	for (i = 0; i < HX83102P_ID_LEN; i++) {
		ret = dsim_read_hl_data(lcd, HX83102P_ID_REG + i, 1, &lcd->id_info.id[i]);
		if (ret < 0)
			break;
	}

	if (ret < 0 || !lcd->id_info.value) {
		priv->lcdconnected = lcd->connected = 0;
		dev_info(&lcd->ld->dev, "%s: connected lcd is invalid\n", __func__);

		if (lcdtype && decon)
			decon_abd_save_bit(&decon->abd, BITS_PER_BYTE * HX83102P_ID_LEN, cpu_to_be32(lcd->id_info.value), LDI_BIT_DESC_ID);
	}

	dev_info(&lcd->ld->dev, "%s: %x\n", __func__, cpu_to_be32(lcd->id_info.value));

	return ret;
}
#endif

static int hx83102p_displayon_late(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	DSI_WRITE(SEQ_SET_B9_PW, ARRAY_SIZE(SEQ_SET_B9_PW));
	DSI_WRITE(SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));
	DSI_WRITE(SEQ_SET_B9_CLOSE_PW, ARRAY_SIZE(SEQ_SET_B9_CLOSE_PW));
	msleep(30);	/* > 20 ms */

	return ret;
}

static int hx83102p_exit(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	DSI_WRITE(SEQ_SET_B9_PW, ARRAY_SIZE(SEQ_SET_B9_PW));
	DSI_WRITE(SEQ_SET_E9_VSOM_SOFT1, ARRAY_SIZE(SEQ_SET_E9_VSOM_SOFT1));
	DSI_WRITE(SEQ_SET_BF_VSOM_SOFT, ARRAY_SIZE(SEQ_SET_BF_VSOM_SOFT));
	DSI_WRITE(SEQ_SET_E9_VSOM_SOFT2, ARRAY_SIZE(SEQ_SET_E9_VSOM_SOFT2));
	DSI_WRITE(SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));
	usleep_range(1000, 2000);
	DSI_WRITE(SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));
	DSI_WRITE(SEQ_SET_B9_CLOSE_PW, ARRAY_SIZE(SEQ_SET_B9_CLOSE_PW));
	msleep(45);	/* > 40 ms*/

	return ret;
}

static int hx83102p_init(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s: ++\n", __func__);
	lcd->cabc_check = 1;
#if defined(CONFIG_SEC_FACTORY)
	hx83102p_read_id(lcd);
#endif

	DSI_WRITE(SEQ_SET_B9_PW, ARRAY_SIZE(SEQ_SET_B9_PW));
	DSI_WRITE(SEQ_SET_E9_OTP_SETTING, ARRAY_SIZE(SEQ_SET_E9_OTP_SETTING));
	DSI_WRITE(SEQ_SET_BB_OTP_SETTING, ARRAY_SIZE(SEQ_SET_BB_OTP_SETTING));
	DSI_WRITE(SEQ_SET_E9_OTP_SETTING2, ARRAY_SIZE(SEQ_SET_E9_OTP_SETTING2));
	DSI_WRITE(SEQ_SET_BA_REGISTER, ARRAY_SIZE(SEQ_SET_BA_REGISTER));
	DSI_WRITE(SEQ_HX83102P_BL, ARRAY_SIZE(SEQ_HX83102P_BL));
	DSI_WRITE(SEQ_HX83102P_BLON, ARRAY_SIZE(SEQ_HX83102P_BLON));
	DSI_WRITE(SEQ_HX83102P_BL_PWM_PREQ, ARRAY_SIZE(SEQ_HX83102P_BL_PWM_PREQ));
	DSI_WRITE(SEQ_SET_CABC_PRIORITY_SETTING, ARRAY_SIZE(SEQ_SET_CABC_PRIORITY_SETTING));
	DSI_WRITE(SEQ_SET_D2_VOP, ARRAY_SIZE(SEQ_SET_D2_VOP));
	DSI_WRITE(SEQ_HX83102P_BL_PAD, ARRAY_SIZE(SEQ_HX83102P_BL_PAD));
	DSI_WRITE(SEQ_SET_D5_GIP_FORWARD, ARRAY_SIZE(SEQ_SET_D5_GIP_FORWARD));
	DSI_WRITE(SEQ_SET_D6_GIP_BACKWARD, ARRAY_SIZE(SEQ_SET_D6_GIP_BACKWARD));
	DSI_WRITE(SEQ_SET_B4_TIMING, ARRAY_SIZE(SEQ_SET_B4_TIMING));
	DSI_WRITE(SEQ_SET_D3_GIP0, ARRAY_SIZE(SEQ_SET_D3_GIP0));
	DSI_WRITE(SEQ_SET_D1_TP_CTRL, ARRAY_SIZE(SEQ_SET_D1_TP_CTRL));
	DSI_WRITE(SEQ_SET_B1_POWER, ARRAY_SIZE(SEQ_SET_B1_POWER));
	DSI_WRITE(SEQ_SET_B2_RESOLUTION, ARRAY_SIZE(SEQ_SET_B2_RESOLUTION));
	DSI_WRITE(SEQ_SET_E0_ANALOG_GAMMA, ARRAY_SIZE(SEQ_SET_E0_ANALOG_GAMMA));
	DSI_WRITE(SEQ_SET_C0_GAMMA_OPT, ARRAY_SIZE(SEQ_SET_C0_GAMMA_OPT));
	DSI_WRITE(SEQ_SET_CC_NORMALLY_BLACK, ARRAY_SIZE(SEQ_SET_CC_NORMALLY_BLACK));
	DSI_WRITE(SEQ_SET_C8_CP, ARRAY_SIZE(SEQ_SET_C8_CP));
	DSI_WRITE(SEQ_SET_BF_GAS, ARRAY_SIZE(SEQ_SET_BF_GAS));
	DSI_WRITE(SEQ_SET_D0_CASCADE, ARRAY_SIZE(SEQ_SET_D0_CASCADE));
	DSI_WRITE(SEQ_SET_BD_BANK01, ARRAY_SIZE(SEQ_SET_BD_BANK01));
	DSI_WRITE(SEQ_SET_D3_GIP1, ARRAY_SIZE(SEQ_SET_D3_GIP1));
	DSI_WRITE(SEQ_SET_BD_BANK02, ARRAY_SIZE(SEQ_SET_BD_BANK02));
	DSI_WRITE(SEQ_SET_B4_GAMMA_BIAS, ARRAY_SIZE(SEQ_SET_B4_GAMMA_BIAS));
	DSI_WRITE(SEQ_SET_BF_VCOM, ARRAY_SIZE(SEQ_SET_BF_VCOM));
	DSI_WRITE(SEQ_SET_D8_GIP, ARRAY_SIZE(SEQ_SET_D8_GIP));
	DSI_WRITE(SEQ_SET_BD_BANK00, ARRAY_SIZE(SEQ_SET_BD_BANK00));
	DSI_WRITE(SEQ_SET_E7_TOUCH_BANK0, ARRAY_SIZE(SEQ_SET_E7_TOUCH_BANK0));
	DSI_WRITE(SEQ_SET_BD_BANK01, ARRAY_SIZE(SEQ_SET_BD_BANK01));
	DSI_WRITE(SEQ_SET_E7_TOUCH_BANK1, ARRAY_SIZE(SEQ_SET_E7_TOUCH_BANK1));
	DSI_WRITE(SEQ_SET_BD_BANK02, ARRAY_SIZE(SEQ_SET_BD_BANK02));
	DSI_WRITE(SEQ_SET_E7_TOUCH_BANK2, ARRAY_SIZE(SEQ_SET_E7_TOUCH_BANK2));
	DSI_WRITE(SEQ_SET_BD_BANK00, ARRAY_SIZE(SEQ_SET_BD_BANK00));
	DSI_WRITE(SEQ_SET_C1_DGC_ON, ARRAY_SIZE(SEQ_SET_C1_DGC_ON));
	DSI_WRITE(SEQ_SET_BD_BANK01, ARRAY_SIZE(SEQ_SET_BD_BANK01));
	DSI_WRITE(SEQ_SET_C1_DGC_GAMMA1, ARRAY_SIZE(SEQ_SET_C1_DGC_GAMMA1));
	DSI_WRITE(SEQ_SET_BD_BANK02, ARRAY_SIZE(SEQ_SET_BD_BANK02));
	DSI_WRITE(SEQ_SET_C1_DGC_GAMMA2, ARRAY_SIZE(SEQ_SET_C1_DGC_GAMMA2));
	DSI_WRITE(SEQ_SET_BD_BANK03, ARRAY_SIZE(SEQ_SET_BD_BANK03));
	DSI_WRITE(SEQ_SET_C1_DGC_GAMMA3, ARRAY_SIZE(SEQ_SET_C1_DGC_GAMMA3));
	DSI_WRITE(SEQ_SET_BD_BANK00, ARRAY_SIZE(SEQ_SET_BD_BANK00));
	DSI_WRITE(SEQ_SET_HX83102P_CABC_ON, ARRAY_SIZE(SEQ_SET_HX83102P_CABC_ON));
	usleep_range(2000, 3000);	/* > 2 ms */
	DSI_WRITE(SEQ_SET_E4_CABC_BANK0, ARRAY_SIZE(SEQ_SET_E4_CABC_BANK0));
	DSI_WRITE(SEQ_SET_BD_BANK01, ARRAY_SIZE(SEQ_SET_BD_BANK01));
	DSI_WRITE(SEQ_SET_E4_CABC_BANK1, ARRAY_SIZE(SEQ_SET_E4_CABC_BANK1));
	DSI_WRITE(SEQ_SET_BD_BANK00, ARRAY_SIZE(SEQ_SET_BD_BANK00));
	DSI_WRITE(SEQ_SLEEP_OUT, ARRAY_SIZE(SEQ_SLEEP_OUT));
	DSI_WRITE(SEQ_SET_B9_CLOSE_PW, ARRAY_SIZE(SEQ_SET_B9_CLOSE_PW));
	msleep(130);	/* > 120ms */
	dev_info(&lcd->ld->dev, "%s: --\n", __func__);

	return ret;
}

static int hx83102p_power_disable(struct lcd_info *lcd)
{
	int ret = 0;
	struct dsim_device *dsim = get_dsim_drvdata(0);

	dev_info(&lcd->ld->dev, "%s\n", __func__);
	run_list(dsim->dev, "dsim_set_panel_power_disable_notaot");

	return ret;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	struct lcd_info *lcd = NULL;
	int fb_blank;
#if defined(CONFIG_SEC_AOT)
	struct decon_device *decon = get_decon_drvdata(0);
#endif
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
		hx83102p_displayon_late(lcd);
		mutex_unlock(&lcd->lock);

		dsim_panel_set_brightness(lcd, 1);
		msleep(30);	/* > 20ms */
	} else if (fb_blank == FB_BLANK_POWERDOWN) {
#if defined(CONFIG_SEC_AOT)
		if (decon_is_enter_shutdown(decon) || !aot_enabled)
			hx83102p_power_disable(lcd);
#else
		hx83102p_power_disable(lcd);
#endif
	}
	return NOTIFY_DONE;
}

static int hx83102p_probe(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	lcd->bd->props.max_brightness = EXTEND_BRIGHTNESS;
	lcd->bd->props.brightness = UI_DEFAULT_BRIGHTNESS;

	lcd->state = PANEL_STATE_RESUMED;
	lcd->lux = -1;

	ret = hx83102p_read_init_info(lcd);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: failed to init information\n", __func__);

	lcd->fb_notif_panel.notifier_call = fb_notifier_callback;
	decon_register_notifier(&lcd->fb_notif_panel);

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
	}

	return size;
}

static ssize_t cabc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	unsigned int value;
	int ret;

	if (lcd->state != PANEL_STATE_RESUMED)
		return -EINVAL;

	ret = kstrtouint(buf, 0, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&lcd->lock);

	if (value == 1) {
		DSI_WRITE(SEQ_SET_B9_PW, ARRAY_SIZE(SEQ_SET_B9_PW));
		DSI_WRITE(SEQ_SET_HX83102P_CABC_ON, ARRAY_SIZE(SEQ_SET_HX83102P_CABC_ON));
		DSI_WRITE(SEQ_SET_B9_CLOSE_PW, ARRAY_SIZE(SEQ_SET_B9_CLOSE_PW));
	} else {
		DSI_WRITE(SEQ_SET_B9_PW, ARRAY_SIZE(SEQ_SET_B9_PW));
		DSI_WRITE(SEQ_SET_HX83102P_CABC_OFF, ARRAY_SIZE(SEQ_SET_HX83102P_CABC_OFF));
		DSI_WRITE(SEQ_SET_B9_CLOSE_PW, ARRAY_SIZE(SEQ_SET_B9_CLOSE_PW));
	}

	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "%s: %d\n", __func__, value);

	return size;
}

static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);
static DEVICE_ATTR(window_type, 0444, window_type_show, NULL);
static DEVICE_ATTR(brightness_table, 0444, brightness_table_show, NULL);
static DEVICE_ATTR(lux, 0644, lux_show, lux_store);
static DEVICE_ATTR(cabc, 0644, NULL, cabc_store);

static struct attribute *lcd_sysfs_attributes[] = {
	&dev_attr_lcd_type.attr,
	&dev_attr_window_type.attr,
	&dev_attr_brightness_table.attr,
	&dev_attr_lux.attr,
	&dev_attr_cabc.attr,
	NULL,
};

static const struct attribute_group lcd_sysfs_attr_group = {
	.attrs = lcd_sysfs_attributes,
};

static void lcd_init_sysfs(struct lcd_info *lcd)
{
	int ret = 0;

	ret = sysfs_create_group(&lcd->ld->dev.kobj, &lcd_sysfs_attr_group);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "failed to add lcd sysfs\n");

	init_debugfs_backlight(lcd->bd, brightness_table, NULL);
}

void hx83102p_update_dphy_timing(u32 hs_clk, struct dphy_timing_value *t)
{
	int val;

	val  = (dphy_timing[0][0] - hs_clk) / 10;

	dphy_timing[val][1] = t->clk_prepare;
	dphy_timing[val][2] = t->clk_zero;
	dphy_timing[val][3] = t->clk_post;
	dphy_timing[val][4] = t->clk_trail;
	dphy_timing[val][5] = t->hs_prepare;
	dphy_timing[val][6] = t->hs_zero;
	dphy_timing[val][7] = t->hs_trail;
	dphy_timing[val][8] = t->lpx;
	dphy_timing[val][9] = t->hs_exit;
}

static int dsim_panel_probe(struct dsim_device *dsim)
{
	int ret = 0;
	struct lcd_info *lcd;
	struct dphy_timing_value t;

	dsim->priv.par = lcd = kzalloc(sizeof(struct lcd_info), GFP_KERNEL);
	if (!lcd) {
		pr_err("%s: failed to allocate for lcd\n", __func__);
		ret = -ENOMEM;
		goto probe_err;
	}

	/* custom dphy timing for hs_clk 958 MHz*/
	t.clk_prepare = 8;
	t.clk_zero = 42;
	t.clk_post = 13;
	t.clk_trail = 9;
	t.hs_prepare = 8;
	t.hs_zero = 15;
	t.hs_trail = 11;
	t.lpx = 6;
	t.hs_exit = 12;

	hx83102p_update_dphy_timing(dsim->clks.hs_clk, &t);

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
	ret = hx83102p_probe(lcd);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: failed to probe panel\n", __func__);

	lcd_init_sysfs(lcd);
	dev_info(&lcd->ld->dev, "%s: %s: done\n", kbasename(__FILE__), __func__);
probe_err:
	return ret;
}

static int dsim_panel_displayon(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "+ %s: %d\n", __func__, lcd->state);

	if (lcd->state == PANEL_STATE_SUSPENED)
		hx83102p_init(lcd);

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

	hx83102p_exit(lcd);

	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_SUSPENED;
	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "- %s: %d, %d\n", __func__, lcd->state, lcd->connected);

exit:
	return 0;
}

struct dsim_lcd_driver hx83102p_mipi_lcd_driver = {
	.name		= "hx83102p",
	.probe		= dsim_panel_probe,
	.displayon	= dsim_panel_displayon,
	.suspend	= dsim_panel_suspend,
};
__XX_ADD_LCD_DRIVER(hx83102p_mipi_lcd_driver);

static void panel_conn_uevent(struct lcd_info *lcd)
{
	char *uevent_conn_str[3] = {"CONNECTOR_NAME=UB_CONNECT", "CONNECTOR_TYPE=HIGH_LEVEL", NULL};

	if (!IS_ENABLED(CONFIG_SEC_FACTORY))
		return;

	if (!lcd->conn_det_enable)
		return;

	kobject_uevent_env(&lcd->ld->dev.kobj, KOBJ_CHANGE, uevent_conn_str);

	dev_info(&lcd->ld->dev, "%s: %s, %s\n", __func__, uevent_conn_str[0], uevent_conn_str[1]);
}

static void panel_conn_work(struct work_struct *work)
{
	struct lcd_info *lcd = container_of(work, struct lcd_info, conn_work);

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	panel_conn_uevent(lcd);
}

static irqreturn_t panel_conn_det_handler(int irq, void *dev_id)
{
	struct lcd_info *lcd = (struct lcd_info *)dev_id;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	queue_work(lcd->conn_workqueue, &lcd->conn_work);

	lcd->conn_det_count++;

	return IRQ_HANDLED;
}

static ssize_t conn_det_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	int gpio_active = of_gpio_get_active("gpio_con");

	if (gpio_active < 0)
		sprintf(buf, "%d\n", -1);
	else
		sprintf(buf, "%s\n", gpio_active ? "disconnected" : "connected");

	dev_info(&lcd->ld->dev, "%s: %s\n", __func__, buf);

	return strlen(buf);
}

static ssize_t conn_det_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	unsigned int value;
	int rc;
	int gpio_active = of_gpio_get_active("gpio_con");

	rc = kstrtouint(buf, 0, &value);
	if (rc < 0)
		return rc;

	if (gpio_active < 0)
		return -EINVAL;

	if (lcd->conn_det_enable != value) {
		dev_info(&lcd->ld->dev, "%s: %u, %u\n", __func__, lcd->conn_det_enable, value);

		mutex_lock(&lcd->lock);
		lcd->conn_det_enable = value;
		mutex_unlock(&lcd->lock);

		dev_info(&lcd->ld->dev, "%s: %s\n", __func__, gpio_active ? "disconnected" : "connected");
		if (lcd->conn_det_enable && gpio_active)
			panel_conn_uevent(lcd);
	}

	return size;
}

static DEVICE_ATTR(conn_det, 0644, conn_det_show, conn_det_store);

static void panel_conn_register(struct lcd_info *lcd)
{
	struct decon_device *decon = get_decon_drvdata(0);
	struct abd_protect *abd = &decon->abd;
	int gpio = 0, gpio_active = 0;

	if (!decon) {
		dev_info(&lcd->ld->dev, "%s: decon is invalid\n", __func__);
		return;
	}

	if (!lcd->connected) {
		dev_info(&lcd->ld->dev, "%s: lcd connected: %d\n", __func__, lcd->connected);
		return;
	}

	gpio = of_get_gpio_with_name("gpio_con");
	if (gpio < 0) {
		dev_info(&lcd->ld->dev, "%s: gpio_con is %d\n", __func__, gpio);
		return;
	}

	gpio_active = of_gpio_get_active("gpio_con");
	if (gpio_active) {
		dev_info(&lcd->ld->dev, "%s: gpio_con_active is %d\n", __func__, gpio_active);
		return;
	}

	INIT_WORK(&lcd->conn_work, panel_conn_work);

	lcd->conn_workqueue = create_singlethread_workqueue("lcd_conn_workqueue");
	if (!lcd->conn_workqueue) {
		dev_info(&lcd->ld->dev, "%s: create_singlethread_workqueue fail\n", __func__);
		return;
	}

	decon_abd_pin_register_handler(abd, gpio_to_irq(gpio), panel_conn_det_handler, lcd);

	if (!IS_ENABLED(CONFIG_SEC_FACTORY))
		return;

	decon_abd_con_register(abd);
	device_create_file(&lcd->ld->dev, &dev_attr_conn_det);
}

static int __init panel_conn_init(void)
{
	struct lcd_info *lcd = NULL;
	struct dsim_device *pdata = NULL;
	struct platform_device *pdev = NULL;

	pdev = of_find_dsim_platform_device();
	if (!pdev) {
		dsim_info("%s: of_find_dsim_platform_device fail\n", __func__);
		return 0;
	}

	pdata = platform_get_drvdata(pdev);
	if (!pdata) {
		dsim_info("%s: platform_get_drvdata fail\n", __func__);
		return 0;
	}

	if (!pdata->panel_ops) {
		dsim_info("%s: panel_ops invalid\n", __func__);
		return 0;
	}

	if (pdata->panel_ops != this_driver)
		return 0;

	lcd = pdata->priv.par;
	if (!lcd) {
		dsim_info("lcd_info invalid\n");
		return 0;
	}

	if (unlikely(!lcd->conn_init_done)) {
		lcd->conn_init_done = 1;
		panel_conn_register(lcd);
	}

	dev_info(&lcd->ld->dev, "%s: %s: done\n", kbasename(__FILE__), __func__);

	return 0;
}
late_initcall_sync(panel_conn_init);

