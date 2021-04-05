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
#include <linux/debugfs.h>

#include "../decon.h"
#include "../decon_notify.h"
#include "../dsim.h"
#include "dsim_panel.h"

#include "sn65dsi86_gview2_param.h"
#include "dd.h"

#if defined(CONFIG_DISPLAY_USE_INFO)
#include "dpui.h"
#endif

#define PANEL_STATE_SUSPENED		0
#define PANEL_STATE_RESUMED		1
#define PANEL_STATE_SUSPENDING	2

#define HX8876_ID_LEN			3

struct bridge_log {
	ktime_t stamp;
	unsigned int onoff;
	u8 dump[256];
};

struct bridge_trace {
	const char *name;
	unsigned int count;
	unsigned int lcdon_flag;

	struct bridge_log *log;
};

struct lcd_info {
	unsigned int			connected;
	unsigned int			brightness;
	unsigned int			state;

	struct lcd_device		*ld;
	struct backlight_device		*bd;

	union {
		struct {
			unsigned char	reserved;
			unsigned char	id[HX8876_ID_LEN];
		};
		u32			value;
	} id_info;

	struct dsim_device		*dsim;
	struct mutex			lock;

	struct notifier_block		fb_notif_panel;

	struct i2c_client		*blic_1;
	struct i2c_client		*blic_2;
	struct i2c_client		*bridge;

	struct dentry			*debug_root;
	struct bridge_trace		b_first;
	struct bridge_trace		b_lcdon;
};

static int sn65dsi86_wait(struct i2c_client *client, u8 command, u8 value, u8 mask, u32 timeout_ms)
{
	u32 i, cnt = timeout_ms;
	int ret = 0;
	struct lcd_info *lcd = i2c_get_clientdata(client);

	if (!lcdtype) {
		dev_info(&lcd->ld->dev, "%s: lcdtype: %d\n", __func__, lcdtype);
		return ret;
	}

	for (i = 0; i < cnt; i++) {
		ret = i2c_smbus_read_byte_data(client, command);
		if (ret < 0) {
			dev_info(&lcd->ld->dev, "%s: fail. %2x, %2x, cnt: %3d, %d\n", __func__, command, value, i, ret);
			break;
		} else if (mask && (ret & value)) {
			dev_info(&lcd->ld->dev, "%s: pass. %2x, %2x, cnt: %3d, %x\n", __func__, command, value, i, ret);
			break;
		} else if (ret == value) {
			/* dev_info(&lcd->ld->dev, "%s: pass. %2x, %2x, cnt: %3d, %x\n", __func__, command, value, i, ret); */
			break;
		}

		usleep_range(1000, 1100);
	}

	if (i >= cnt) {
		dev_info(&lcd->ld->dev, "%s: pass. %2x, %2x, cnt: %3d, %x, timeout(%d)\n", __func__, command, value, i, ret, timeout_ms);
		ret = -EPERM;
	}

	return ret;
}

static int sn65dsi86_array_write(struct i2c_client *client, u8 *ptr, u8 len)
{
	unsigned int i = 0;
	int ret = 0;
	u8 type = 0, command = 0, value = 0;
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

	if (len % 3) {
		dev_info(&lcd->ld->dev, "%s: length(%d) invalid\n", __func__, len);
		return ret;
	}

	for (i = 0; i < len; i += 3) {
		type = ptr[i + 0];
		command = ptr[i + 1];
		value = ptr[i + 2];

		if (type == TYPE_DELAY)
			(command < 20) ? mdelay(command) : msleep(command);
		else if (type == TYPE_CHECK)
			ret = sn65dsi86_wait(client, command, value, 0, 255);
		else {
			ret = i2c_smbus_write_byte_data(client, command, value);
			if (ret < 0) {
				dev_info(&lcd->ld->dev, "%s: fail. %2x, %2x, %d\n", __func__, command, value, ret);
				break;
			}
		}
	}

	return ret;
}

static int lp8558_array_write(struct i2c_client *client, u8 *ptr, u8 len)
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

static int sn65dsi86_dpcd_tx(struct i2c_client *client, u32 addr, u8 value)
{
	struct lcd_info *lcd = i2c_get_clientdata(client);
	u8 dpcd_param[] = {
		TYPE_WRITE, 0x74, 0x00,	/* AUX_ADDR[19:16] */
		TYPE_WRITE, 0x75, 0x00,	/* AUX_ADDR[15:8] */
		TYPE_WRITE, 0x76, 0x00,	/* AUX_ADDR[7:0] */
		TYPE_WRITE, 0x77, 0x01,	/* AUX_LENGTH */
		TYPE_WRITE, 0x64, 0x00,	/* AUX_WDATA0 */
		TYPE_WRITE, 0x78, AUX_CMD_NATIVE_AUX_W << 4 | BIT(0),	/* AUX_CMD, SEND */
		TYPE_CHECK, 0x78, AUX_CMD_NATIVE_AUX_W << 4,		/* AUX_CMD, check SEND bit */
	};
	u8 addr_msb, addr_lsb;
	int ret = 0;

	addr_msb = (addr >> 8) & 0xff;
	addr_lsb = (addr >> 0) & 0xff;

	dpcd_param[1 * 3 + 2] = addr_msb;
	dpcd_param[2 * 3 + 2] = addr_lsb;
	dpcd_param[4 * 3 + 2] = value;

	ret = sn65dsi86_array_write(client, dpcd_param, ARRAY_SIZE(dpcd_param));
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: %x, i2c_tx errno: %d\n",  __func__, addr, ret);

	return ret;
}

static int sn65dsi86_dpcd_rx(struct i2c_client *client, u32 addr)
{
	struct lcd_info *lcd = i2c_get_clientdata(client);
	u8 dpcd_param[] = {
		TYPE_WRITE, 0x74, 0x00,	/* AUX_ADDR[19:16] */
		TYPE_WRITE, 0x75, 0x00,	/* AUX_ADDR[15:8] */
		TYPE_WRITE, 0x76, 0x00,	/* AUX_ADDR[7:0] */
		TYPE_WRITE, 0x77, 0x01,	/* AUX_LENGTH */
		TYPE_WRITE, 0x78, AUX_CMD_NATIVE_AUX_R << 4 | BIT(0),	/* AUX_CMD, SEND */
		TYPE_CHECK, 0x78, AUX_CMD_NATIVE_AUX_R << 4,		/* AUX_CMD, check SEND bit */
	};
	u8 addr_msb, addr_lsb;
	int ret = 0;

	addr_msb = (addr >> 8) & 0xff;
	addr_lsb = (addr >> 0) & 0xff;

	dpcd_param[1 * 3 + 2] = addr_msb;
	dpcd_param[2 * 3 + 2] = addr_lsb;

	ret = sn65dsi86_array_write(client, dpcd_param, ARRAY_SIZE(dpcd_param));
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: %x, i2c_tx errno: %d\n", __func__, addr, ret);

	ret = i2c_smbus_read_byte_data(client, 0x79);	/* AUX_RDATA0 */
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: %x, i2c_rx errno: %d\n",  __func__, addr, ret);

	return ret;
}

static int dsim_panel_set_brightness(struct lcd_info *lcd, int force)
{
	int ret = 0;

	mutex_lock(&lcd->lock);

	lcd->brightness = lcd->bd->props.brightness;

	if (!force && lcd->state != PANEL_STATE_RESUMED) {
		dev_info(&lcd->ld->dev, "%s: panel is not active state\n", __func__);
		goto exit;
	}

	sn65dsi86_dpcd_tx(lcd->bridge, 0x722, brightness_table[lcd->brightness]);

	dev_info(&lcd->ld->dev, "%s: %d %d\n", __func__, lcd->brightness, brightness_table[lcd->brightness]);

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

static int sn65dsi86_hx8876_read_init_info(struct lcd_info *lcd)
{
	struct panel_private *priv = &lcd->dsim->priv;

	priv->lcdconnected = lcd->connected = lcdtype ? 1 : 0;

	lcd->id_info.id[0] = (lcdtype & 0xFF0000) >> 16;
	lcd->id_info.id[1] = (lcdtype & 0x00FF00) >> 8;
	lcd->id_info.id[2] = (lcdtype & 0x0000FF) >> 0;

	dev_info(&lcd->ld->dev, "%s: %x\n", __func__, cpu_to_be32(lcd->id_info.value));

	return 0;
}

static void sn65dsi86_abd_save_log(struct lcd_info *lcd, struct bridge_trace *trace, u8 *dump, u32 on)
{
	struct bridge_trace *first = &lcd->b_first;

	struct bridge_log *first_log = &first->log[0];
	struct bridge_log *trace_log = &trace->log[(trace->count % ABD_LOG_MAX)];

	memset(trace_log, 0, sizeof(struct bridge_log));
	trace_log->stamp = ktime_get();
	trace_log->onoff = on;
	memcpy(&trace_log->dump, dump, 256);

	if (!first->count) {
		memset(first_log, 0, sizeof(struct bridge_log));
		memcpy(first_log, trace_log, sizeof(struct bridge_log));
		first->count++;
	}

	trace->count++;
}

static int sn65dsi86_abd_print(struct seq_file *m, struct bridge_trace *trace, u32 log_max)
{
	int i, j;
	struct timeval tv;
	struct bridge_log *log;

	if (!trace->count)
		return 0;

	seq_printf(m, "%s total count: %d\n", trace->name, trace->count);

	for (i = 0; i < log_max; i++) {
		log = &trace->log[i];
		if (!log->stamp.tv64)
			continue;
		tv = ns_to_timeval(log->stamp.tv64);
		seq_printf(m, "time: %lu.%06lu onoff: %d\n",
			(unsigned long)tv.tv_sec, tv.tv_usec, log->onoff);

		seq_puts(m, "[--] ");
		for (j = 0; j < 16; j++)
			seq_printf(m, "%02x ", j);
		seq_puts(m, " ");
		seq_puts(m, "[--] ");
		for (j = 0; j < 16; j++)
			seq_printf(m, "%02x ", j);
		seq_puts(m, "\n");

		for (j = 0; j <= 0xff; j += 32) {
			seq_printf(m, "[%02x] %16ph  ", j, &log->dump[j]);
			seq_printf(m, "[%02x] %16ph\n", j + 16, &log->dump[j + 16]);
		}
	}
	seq_puts(m, "\n");

	return 0;
}

static int panel_debug_show(struct seq_file *m, void *unused)
{
	struct lcd_info *lcd = m->private;

	sn65dsi86_abd_print(m, &lcd->b_first, 1);
	sn65dsi86_abd_print(m, &lcd->b_lcdon, ABD_LOG_MAX);

	return 0;
}

static int panel_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, panel_debug_show, inode->i_private);
}

static const struct file_operations panel_debug_fops = {
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
	.open = panel_debug_open,
};

static void sn65dsi86_abd_register(struct lcd_info *lcd)
{
	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	lcd->b_first.name = "first";
	lcd->b_lcdon.name = "lcdon";

	lcd->b_first.log = kzalloc(sizeof(struct bridge_log), GFP_KERNEL);
	lcd->b_lcdon.log = kzalloc(sizeof(struct bridge_log) * ABD_LOG_MAX, GFP_KERNEL);

	lcd->debug_root = debugfs_create_dir("panel", NULL);
	debugfs_create_file("debug", 0444, lcd->debug_root, lcd, &panel_debug_fops);

	dev_info(&lcd->ld->dev, "- %s\n", __func__);
}

static void sn65dsi86_dump(struct lcd_info *lcd, struct seq_file *m)
{
	int i, rx_value = 0, len;
	u8 rx_dump[0xff + 1] = {0, };
	u8 type = 0, command = 0, value = 0;
	u8 *ptr;

	dev_info(&lcd->ld->dev, "+ %s: %x\n", __func__, cpu_to_be32(lcd->id_info.value));

	if (!lcd->id_info.value)
		return;

	for (i = 0; i <= 0xff; i++) {
		rx_value = i2c_smbus_read_byte_data(lcd->bridge, i);
		if (rx_value < 0)
			break;
		rx_dump[i] = rx_value;
	}

	if (m) {
		seq_puts(m, "[--] 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\n");
		for (i = 0; i <= 0xff; i += 16)
			seq_printf(m, "[%02x] %16ph\n", i, &rx_dump[i]);
	} else {
		dev_info(&lcd->ld->dev, "[--] 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\n");
		for (i = 0; i <= 0xff; i += 16)
			dev_info(&lcd->ld->dev, "[%02x] %16ph\n", i, &rx_dump[i]);
	}

	ptr = SN65DSI86_INIT;
	len = ARRAY_SIZE(SN65DSI86_INIT);
	for (i = 0; i < len; i += 3) {
		type = ptr[i + 0];
		command = ptr[i + 1];
		value = ptr[i + 2];

		if (type == TYPE_DELAY || type == TYPE_CHECK)
			continue;

		if (value != rx_dump[command] && m) {
			seq_printf(m, "[%2d][%2x, %2x]: %2x, %s\n", (i / 3) + 1,
			command, value, rx_dump[command], (value != rx_dump[command]) ? "X" : "");
		} else if (value != rx_dump[command]) {
			dev_info(&lcd->ld->dev, "[%2d][%2x, %2x]: %2x, %s\n", (i / 3) + 1,
			command, value, rx_dump[command], (value != rx_dump[command]) ? "X" : "");
		}
	}

	dev_info(&lcd->ld->dev, "- %s\n", __func__);

	sn65dsi86_abd_save_log(lcd, &lcd->b_lcdon, rx_dump, 1);
}

static void sn65dsi86_check_lt_fail(struct lcd_info *lcd)
{
	int rx_val = 0;

/*	[0xF8 register info]
 *	bit 0 : LT_PASS
 *	bit 1 : LP_FAIL
 */

	if (!lcd->id_info.value)
		return;

	rx_val = i2c_smbus_read_byte_data(lcd->bridge, 0xF8);

	if (rx_val < 0) {
		dev_info(&lcd->ld->dev, "%s: read fail. [0x%02x]\n", __func__, rx_val);
		return;
	}


	if (rx_val & 0x02) {
		inc_dpui_u32_field(DPUI_KEY_PNSDRE, 1);
		dev_info(&lcd->ld->dev, "%s: LT_FAIL [0x%02x]\n", __func__, rx_val);
	}
}

static int sn65dsi86_hx8876_displayon_late(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	dsim_panel_set_brightness(lcd, 1);

	return ret;
}

static int sn65dsi86_hx8876_exit(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	return ret;
}

static int hx8876_init(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	sn65dsi86_dpcd_tx(lcd->bridge, 0x721, 0x02);
	sn65dsi86_dpcd_tx(lcd->bridge, 0x722, 0x00);

	dev_info(&lcd->ld->dev, "- %s\n", __func__);

	return ret;
}

static int sn65dsi86_init(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	sn65dsi86_array_write(lcd->bridge, SN65DSI86_INIT, ARRAY_SIZE(SN65DSI86_INIT));
	usleep_range(10000, 11000);

	dev_info(&lcd->ld->dev, "- %s\n", __func__);

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
		sn65dsi86_hx8876_displayon_late(lcd);
		mutex_lock(&lcd->lock);
		sn65dsi86_dump(lcd, NULL);
		sn65dsi86_check_lt_fail(lcd);
		mutex_unlock(&lcd->lock);
	}

	return NOTIFY_DONE;
}

static struct i2c_device_id sn65dsi86_i2c_id[] = {
	{"sn65dsi86", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, sn65dsi86_i2c_id);

static const struct of_device_id sn65dsi86_i2c_dt_ids[] = {
	{ .compatible = "i2c,sn65dsi86" },
	{ }
};

MODULE_DEVICE_TABLE(of, sn65dsi86_i2c_dt_ids);

static int sn65dsi86_probe(struct i2c_client *client,
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

	lcd->bridge = client;

	dev_info(&lcd->ld->dev, "%s: %s: %s %s\n", __func__, id->name, dev_name(&client->adapter->dev), of_node_full_name(client->dev.of_node));

exit:
	return ret;
}

static int sn65dsi86_command(struct i2c_client *client, unsigned int num, void *arg)
{
	struct lcd_info *lcd = i2c_get_clientdata(client);
	int ret = 0;
	struct i2c_msg *xfer = arg;
	unsigned int command = 0, value = 0, i = 0;

	if (!client) {
		dev_info(&lcd->ld->dev, "%s: client is null\n", __func__);
		return ret;
	}

	if (!lcd) {
		dev_info(&lcd->ld->dev, "%s: lcd is null\n", __func__);
		return ret;
	}

	if (!arg) {
		dev_info(&lcd->ld->dev, "%s: arg is null\n", __func__);
		return ret;
	}

	if (num > 2) {
		dev_info(&lcd->ld->dev, "%s: num(%d) is invalid\n", __func__, num);
		return ret;
	}

	for (i = 0; i < num; i++, xfer++) {
		if (!xfer) {
			dev_info(&lcd->ld->dev, "%s: %02d xfer is null\n", __func__, i);
			return ret;
		}

		if (xfer->buf) {
			dev_info(&lcd->ld->dev, "%s: %02d buf is null\n", __func__, i);
			return ret;
		}

		if ((xfer->flags & I2C_M_RD) && xfer->len != 1) {
			dev_info(&lcd->ld->dev, "%s: %02d rx len(%d) is invalid\n", __func__, i, xfer->len);
			return ret;
		}

		if (!(xfer->flags & I2C_M_RD) && (xfer->flags & I2C_M_TEN) && xfer->len != 3) {
			dev_info(&lcd->ld->dev, "%s: %02d tx len(%d) is invalid for I2C_M_TEN\n", __func__, i, xfer->len);
			return ret;
		}

		if (!(xfer->flags & I2C_M_RD) && !(xfer->flags & I2C_M_TEN) && xfer->len != 2) {
			dev_info(&lcd->ld->dev, "%s: %02d tx len(%d) is invalid\n", __func__, i, xfer->len);
			return ret;
		}
	}

	if (xfer[0].flags & I2C_M_TEN) {
		command = xfer[0].buf[0] << 8 || xfer[0].buf[1];
		value = xfer[0].buf[2];
	} else {
		command = xfer[0].buf[0];
		value = xfer[0].buf[1];
	}

	if (num == 2) {
		dev_info(&lcd->ld->dev, "%s: rx: %x\n", __func__, command);
		ret = (command > U8_MAX) ? sn65dsi86_dpcd_rx(client, command) : i2c_smbus_read_byte_data(client, command);
		if (ret < 0)
			dev_info(&lcd->ld->dev, "%s: %02x, i2c_rx errno: %d\n", __func__, command, ret);
		else
			xfer[1].buf[0] = ret;
	} else {
		dev_info(&lcd->ld->dev, "%s: tx: %x, %x\n", __func__, command, value);
		ret = (command > U8_MAX) ? sn65dsi86_dpcd_tx(client, command, value) : i2c_smbus_write_byte_data(client, command, value);
		if (ret < 0)
			dev_info(&lcd->ld->dev, "%s: %02x, i2c_tx errno: %d\n", __func__, command, ret);
	}

	return ret;
}

static struct i2c_driver sn65dsi86_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "sn65dsi86",
		.of_match_table	= of_match_ptr(sn65dsi86_i2c_dt_ids),
	},
	.id_table = sn65dsi86_i2c_id,
	.probe = sn65dsi86_probe,
	.command = sn65dsi86_command,
};

static struct i2c_device_id lp8558_i2c_id[] = {
	{"lp8558_1", 0},
	{"lp8558_2", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lp8558_i2c_id);

static const struct of_device_id lp8558_i2c_dt_ids[] = {
	{ .compatible = "i2c,lp8558_1" },
	{ .compatible = "i2c,lp8558_2" },
	{ }
};

MODULE_DEVICE_TABLE(of, lp8558_i2c_dt_ids);

static int lp8558_probe(struct i2c_client *client,
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

	if (!strcmp(id->name, lp8558_i2c_id[0].name))
		lcd->blic_1 = client;
	else
		lcd->blic_2 = client;

	dev_info(&lcd->ld->dev, "%s: %s: %s %s\n", __func__, id->name, dev_name(&client->adapter->dev), of_node_full_name(client->dev.of_node));

exit:
	return ret;
}

static struct i2c_driver lp8558_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "lp8558",
		.of_match_table	= of_match_ptr(lp8558_i2c_dt_ids),
	},
	.id_table = lp8558_i2c_id,
	.probe = lp8558_probe,
};

static int sn65dsi86_hx8876_probe(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	lcd->bd->props.max_brightness = EXTEND_BRIGHTNESS;
	lcd->bd->props.brightness = UI_DEFAULT_BRIGHTNESS;

	lcd->state = PANEL_STATE_RESUMED;

	ret = sn65dsi86_hx8876_read_init_info(lcd);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "%s: failed to init information\n", __func__);

	lcd->fb_notif_panel.notifier_call = fb_notifier_callback;
	decon_register_notifier(&lcd->fb_notif_panel);

	sn65dsi86_i2c_id[0].driver_data = (kernel_ulong_t)lcd;
	i2c_add_driver(&sn65dsi86_i2c_driver);

	lp8558_i2c_id[0].driver_data = (kernel_ulong_t)lcd;
	lp8558_i2c_id[1].driver_data = (kernel_ulong_t)lcd;
	i2c_add_driver(&lp8558_i2c_driver);

	sn65dsi86_abd_register(lcd);
	sn65dsi86_dump(lcd, NULL);

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

static ssize_t bridge_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lcd_info *lcd = dev_get_drvdata(dev);
	struct seq_file m = {
		.buf = buf,
		.size = PAGE_SIZE - 1,
	};

	sn65dsi86_dump(lcd, &m);

	return strlen(buf);
}

#if defined(CONFIG_DISPLAY_USE_INFO)
/*
 * HW PARAM LOGGING SYSFS NODE
 */
static ssize_t dpui_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret;

	update_dpui_log(DPUI_LOG_LEVEL_INFO, DPUI_TYPE_PANEL);
	ret = get_dpui_log(buf, DPUI_LOG_LEVEL_INFO, DPUI_TYPE_PANEL);
	if (ret < 0) {
		pr_err("%s failed to get log %d\n", __func__, ret);
		return ret;
	}

	pr_info("%s\n", buf);

	return ret;
}

static ssize_t dpui_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (buf[0] == 'C' || buf[0] == 'c')
		clear_dpui_log(DPUI_LOG_LEVEL_INFO, DPUI_TYPE_PANEL);

	return size;
}

static DEVICE_ATTR(dpui, 0660, dpui_show, dpui_store);
#endif

static DEVICE_ATTR(lcd_type, 0444, lcd_type_show, NULL);
static DEVICE_ATTR(window_type, 0444, window_type_show, NULL);
static DEVICE_ATTR(bridge_dump, 0444, bridge_dump_show, NULL);

static struct attribute *lcd_sysfs_attributes[] = {
	&dev_attr_lcd_type.attr,
	&dev_attr_window_type.attr,
	&dev_attr_bridge_dump.attr,
#if defined(CONFIG_DISPLAY_USE_INFO)
	&dev_attr_dpui.attr,
#endif
	NULL,
};

static const struct attribute_group lcd_sysfs_attr_group = {
	.attrs = lcd_sysfs_attributes,
};

static void lcd_init_sysfs(struct lcd_info *lcd)
{
	int ret = 0;
	struct i2c_client *clients[] = {lcd->bridge, lcd->blic_1, lcd->blic_2, NULL};

	ret = sysfs_create_group(&lcd->ld->dev.kobj, &lcd_sysfs_attr_group);
	if (ret < 0)
		dev_info(&lcd->ld->dev, "failed to add lcd sysfs\n");

	init_debugfs_backlight(lcd->bd, brightness_table, clients);

	init_debugfs_param("bridge", &SN65DSI86_INIT, U8_MAX, ARRAY_SIZE(SN65DSI86_INIT), 3);

	init_debugfs_param("blic_init", &LP8558_INIT, U8_MAX, ARRAY_SIZE(LP8558_INIT), 2);
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
	ret = sn65dsi86_hx8876_probe(lcd);
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

	lp8558_array_write(lcd->blic_1, LP8558_INIT, ARRAY_SIZE(LP8558_INIT));
	lp8558_array_write(lcd->blic_2, LP8558_INIT, ARRAY_SIZE(LP8558_INIT));
	mdelay(1);

	dev_info(&lcd->ld->dev, "- %s: %d, %d\n", __func__, lcd->state, lcd->connected);

	return 0;
}

static int dsim_panel_after_reset(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "+ %s: %d\n", __func__, lcd->state);

	if (lcd->state == PANEL_STATE_SUSPENED)
		sn65dsi86_init(lcd);

	dev_info(&lcd->ld->dev, "- %s: %d, %d\n", __func__, lcd->state, lcd->connected);

	return 0;
}

static int dsim_panel_displayon(struct dsim_device *dsim)
{
	struct lcd_info *lcd = dsim->priv.par;

	dev_info(&lcd->ld->dev, "+ %s: %d\n", __func__, lcd->state);

	if (lcd->state == PANEL_STATE_SUSPENED)
		hx8876_init(lcd);

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

	sn65dsi86_hx8876_exit(lcd);

	mutex_lock(&lcd->lock);
	lcd->state = PANEL_STATE_SUSPENED;
	mutex_unlock(&lcd->lock);

	dev_info(&lcd->ld->dev, "- %s: %d, %d\n", __func__, lcd->state, lcd->connected);

exit:
	return 0;
}

struct dsim_lcd_driver sn65dsi86_mipi_lcd_driver = {
	.name		= "sn65dsi86",
	.probe		= dsim_panel_probe,
	.resume_early	= dsim_panel_resume_early,
	.after_reset	= dsim_panel_after_reset,
	.displayon	= dsim_panel_displayon,
	.suspend	= dsim_panel_suspend,
};
__XX_ADD_LCD_DRIVER(sn65dsi86_mipi_lcd_driver);

