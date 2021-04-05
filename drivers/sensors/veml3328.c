/* VEML3328 Optical Sensor Driver
 *
 * Copyright (C) 2018 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>
#include <linux/sensor/sensors_core.h>

#include "veml3328.h"

#define I2C_M_WR          0 /* for i2c Write */
#define I2c_M_RD          1 /* for i2c Read */
#define I2C_RETRY_CNT     5

#define DEFAULT_DELAY_MS  200

#define VENDOR_NAME        "CAPELLA"
#define CHIP_NAME          "VEML3328"
#define MODULE_NAME        "light_sensor"
#define MODULE_NAME_HIDDEN "hidden_hole"

#define LIGHT_LOG_TIME	  15 /* 15 * light_poll_delay SEC */

struct veml3328_data {
	struct device *ls_dev;
	struct input_dev *input_dev;

	struct i2c_client *i2c_client;

	u8 als_enable;

	struct hrtimer light_timer;
	struct workqueue_struct *light_wq;
	struct work_struct light_work;
	ktime_t light_poll_delay;

	u16 red;
	u16 green;
	u16 blue;
	u16 clear;
	int gain_level;
	int coef[EFS_SAVE_NUMS];

	int debug_count;

	struct mutex control_mutex;
	bool is_first_enable;
};

enum {
	GAIN_HALF = 0,
	GAIN_X2,
	GAIN_X8
};

enum {
	OFF = 0,
	ON
};

#define DGF      1060
#define Rcoef    148
#define Gcoef    302
#define Bcoef    116
#define Wcoef    (-114)
#define CoefA    933
#define CToffset 1758

struct {
	int version;
	int octa_id;
	int data[EFS_SAVE_NUMS];
} hidden_table[ID_INDEX_NUMS] = {
	{190107, ID_BLACK,
	{DGF, Rcoef, Gcoef, Bcoef, Wcoef, CoefA, CToffset, 0, 0, 0, 4203} },
	{190107, ID_WHITE,
	{DGF, Rcoef, Gcoef, Bcoef, Wcoef, CoefA, CToffset, 0, 0, 0, 4203} },
};

int hidden_hole_init_work(struct veml3328_data *drv_data);
int veml3328_i2c_read(struct i2c_client *client, u8 reg, u8 *val, int len)
{
	int ret;
	struct i2c_msg msg[2];
	int retry = 0;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	/* Write slave address */
	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_WR;
	msg[0].len = 1;
	msg[0].buf = &reg;

	/* Read data */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = val;

	for (retry = 0; retry < I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret == 2)
			break;

		SENSOR_ERR("i2c read error, ret=%d retry=%d\n", ret, retry);

		if (retry < I2C_RETRY_CNT - 1)
			usleep_range(2000, 2000);
	}

	return (ret == 2) ? 0 : ret;
}

int veml3328_i2c_read_word(struct veml3328_data *drv_data, u8 reg, u16 *val)
{
	u8 buf[2] = {0,0};
	int ret = 0;

	ret = veml3328_i2c_read(drv_data->i2c_client, reg, buf, 2);
	if (!ret)
		*val = (buf[1] << 8) | buf[0];

	return ret;
}

int veml3328_i2c_write(struct i2c_client *client, u8 reg, u8 *val, int len)
{
	int ret;
	struct i2c_msg msg;
	unsigned char data[11];
	int retry = 0;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	if (len >= 10) {
		SENSOR_ERR("Exceeded length limit, len=%d\n", len);
		return -EINVAL;
	}

	data[0] = reg;
	memcpy(&data[1], val, len);

	msg.addr = client->addr;
	msg.flags = I2C_M_WR;
	msg.len = len + 1;
	msg.buf = data;

	for (retry = 0; retry < I2C_RETRY_CNT; retry++) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;

		SENSOR_ERR("i2c write error, ret=%d retry=%d\n", ret, retry);

		if (retry < I2C_RETRY_CNT - 1)
			usleep_range(2000, 2000);
	}

	return (ret == 1) ? 0 : ret;
}

int veml3328_i2c_write_word(struct veml3328_data *drv_data, u8 reg, u16 val)
{
	u8 buf[2];
	int ret = 0;

	buf[0] = (val & 0x00FF);
	buf[1] = (val & 0xFF00) >> 8;

	ret = veml3328_i2c_write(drv_data->i2c_client, reg, buf, 2);

	return ret;
}

static int veml3328_light_get_cct(struct veml3328_data *drv_data)
{
	int cct = 0;
	if (drv_data->red != 0)
		cct = (drv_data->coef[5] * drv_data->blue) / drv_data->red;

	cct += drv_data->coef[6];
	return cct;
}

static int veml3328_light_get_lux(struct veml3328_data *drv_data)
{	
	s32 calculated_lux = 0;
	int divide_lux = 0;
	int gain_table[3] = {VEML3328_AUTO_GAIN_HALF, VEML3328_CONF_GAIN_2, VEML3328_CONF_GAIN_8};
	int atime_table[4] = {50, 100, 200, 400};
	u16 val = 0;

	veml3328_i2c_read_word(drv_data, VEML3328_R_DATA, &drv_data->red);
	veml3328_i2c_read_word(drv_data, VEML3328_G_DATA, &drv_data->green);
	veml3328_i2c_read_word(drv_data, VEML3328_B_DATA, &drv_data->blue);
	veml3328_i2c_read_word(drv_data, VEML3328_C_DATA, &drv_data->clear);

	input_sync(drv_data->input_dev);

	veml3328_i2c_read_word(drv_data, VEML3328_CONF, &val);
	if (drv_data->red <= 100 || drv_data->green <= 100 || drv_data->blue <= 100 || drv_data->clear <= 100) {
		drv_data->gain_level++;
		if (drv_data->gain_level > GAIN_X8)
			drv_data->gain_level = GAIN_X8;
		val &= VEML3328_CONF_GAIN_MASK;
		val |= gain_table[drv_data->gain_level];
		veml3328_i2c_write_word(drv_data, VEML3328_CONF, val);
	} else if (drv_data->red >= 60000 || drv_data->green >= 60000 || drv_data->blue >= 60000  || drv_data->clear >= 60000) {
		drv_data->gain_level--;
		if (drv_data->gain_level < GAIN_HALF)
			drv_data->gain_level = GAIN_HALF;
		val &= VEML3328_CONF_GAIN_MASK;
		val |= gain_table[drv_data->gain_level];
		veml3328_i2c_write_word(drv_data, VEML3328_CONF, val);
	}

	// Get ATIME
	val = (val & 0x30) >> 4;

	// divide_lux = ATIME * AGAIN
	if (drv_data->gain_level == GAIN_HALF)
		divide_lux = atime_table[val] >> 1;
	else if (drv_data->gain_level == GAIN_X2)
		divide_lux = atime_table[val] << 1;
	else if (drv_data->gain_level == GAIN_X8)
		divide_lux = atime_table[val] << 3;

	// LUX = (DGF * (R*Rcoef+G*Gcoef+B*Bcoef+W*Wcoef)) / (ATIME*AGAIN)
	// Calculate LUX
	calculated_lux = ((drv_data->red * drv_data->coef[1]) + (drv_data->green * drv_data->coef[2]) +
		(drv_data->blue * drv_data->coef[3]) + (drv_data->clear * drv_data->coef[4])) / 1000;

	if (divide_lux != 0)
		calculated_lux = (drv_data->coef[0] * calculated_lux) / divide_lux;

	calculated_lux -= 2;
	if (calculated_lux < 0)
		calculated_lux = 0;

	return calculated_lux;
}

static void light_work_func(struct work_struct *work)
{
	struct veml3328_data *drv_data = container_of(work,
			struct veml3328_data, light_work);

	int lux = veml3328_light_get_lux(drv_data);
	int cct = veml3328_light_get_cct(drv_data);

	input_report_rel(drv_data->input_dev, REL_MISC, lux + 1);
	input_report_rel(drv_data->input_dev, REL_WHEEL, cct);

	if ((ktime_to_ns(drv_data->light_poll_delay) * (int64_t)drv_data->debug_count)
		>= ((int64_t)LIGHT_LOG_TIME * NSEC_PER_SEC)) {
		SENSOR_INFO("red=%d green=%d blue=%d clear=%d lux=%d cct=%d\n",
				drv_data->red, drv_data->green, drv_data->blue,
				drv_data->clear, lux, cct);
		drv_data->debug_count = 0;
	} else
		drv_data->debug_count++;
}

static enum hrtimer_restart light_timer_func(struct hrtimer *timer)
{
	struct veml3328_data *drv_data = container_of(timer,
			struct veml3328_data, light_timer);

	queue_work(drv_data->light_wq, &drv_data->light_work);
	hrtimer_forward_now(&drv_data->light_timer, drv_data->light_poll_delay);

	return HRTIMER_RESTART;
}

static int lightsensor_enable(struct veml3328_data *drv_data)
{
	int ret = 0;
	u16 val;

	mutex_lock(&drv_data->control_mutex);

	SENSOR_INFO("\n");

	val = VEML3328_CONF_IT_100MS | VEML3328_CONF_GAIN_8;

	ret = veml3328_i2c_write_word(drv_data, VEML3328_CONF, val);
	if (ret) {
		SENSOR_ERR("failed, ret=%d", ret);
	} else {
		hrtimer_start(&drv_data->light_timer, drv_data->light_poll_delay, HRTIMER_MODE_REL);	
	}

	mutex_unlock(&drv_data->control_mutex);

	return ret;
}

static int lightsensor_disable(struct veml3328_data *drv_data)
{
	int ret = 0;
	u16 val;

	mutex_lock(&drv_data->control_mutex);

	SENSOR_INFO("\n");

	hrtimer_cancel(&drv_data->light_timer);
	cancel_work_sync(&drv_data->light_work);

	val = VEML3328_CONF_SD;

	ret = veml3328_i2c_write_word(drv_data, VEML3328_CONF, val);
	if (ret)
		SENSOR_ERR("failed, ret=%d", ret);

	mutex_unlock(&drv_data->control_mutex);

	return ret;
}

static ssize_t light_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct veml3328_data *drv_data = dev_get_drvdata(dev);
	int ret = 0;

	ret = sprintf(buf, "%d\n", drv_data->als_enable);

	return ret;
}

static ssize_t light_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct veml3328_data *drv_data = dev_get_drvdata(dev);
	int ret = 0;
	u8 enable;

	ret = kstrtou8(buf, 2, &enable);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	if (enable != 0 && enable != 1)
		return -EINVAL;

	SENSOR_INFO("old_en=%d en=%d\n", drv_data->als_enable, enable);

	if (enable && !drv_data->als_enable) {
		ret = lightsensor_enable(drv_data);
		drv_data->als_enable = ON;

		/* hidden hole */
		if(drv_data->is_first_enable) {
			if(hidden_hole_init_work(drv_data) < 0) {			
				drv_data->coef[0] = DGF;
				drv_data->coef[1] = Rcoef;
				drv_data->coef[2] = Gcoef;
				drv_data->coef[3] = Bcoef;
				drv_data->coef[4] = Wcoef;
				drv_data->coef[5] = CoefA;
				drv_data->coef[6] = CToffset;
			};
			drv_data->is_first_enable = false;
		}
	} else if (!enable && drv_data->als_enable) {
		ret = lightsensor_disable(drv_data);
		drv_data->als_enable = OFF;
	}

	return ret;
}

static ssize_t light_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct veml3328_data *drv_data = dev_get_drvdata(dev);
	int ret = 0;

	ret = sprintf(buf, "%llu\n", ktime_to_ns(drv_data->light_poll_delay));

	return ret;
}

static ssize_t light_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct veml3328_data *drv_data = dev_get_drvdata(dev);
	u64 new_delay;
	int ret;

	ret = kstrtoull(buf, 10, &new_delay);
	if (ret) {
		pr_err("[SENSOR]: %s - Invalid Argument\n", __func__);
		return ret;
	}

	if (new_delay <= 100000000)
		new_delay = 100000000;
	SENSOR_INFO("delay=%llu ns\n", new_delay);

	drv_data->light_poll_delay = ns_to_ktime(new_delay);

	return ret;
}

static ssize_t light_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_NAME);
}

static ssize_t light_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t light_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct veml3328_data *drv_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", veml3328_light_get_lux(drv_data));
}

static ssize_t light_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct veml3328_data *drv_data = dev_get_drvdata(dev);
	int auto_gain = 0;

	/* BY H/W REQUEST */
	if (drv_data->gain_level == GAIN_HALF)
		auto_gain = 0;
	else if (drv_data->gain_level == GAIN_X2)
		auto_gain = 2;
	else if (drv_data->gain_level == GAIN_X8)
		auto_gain = 8;

	return snprintf(buf, PAGE_SIZE, "%u,%u,%u,%u,%d\n",
			drv_data->red, drv_data->green, drv_data->blue, drv_data->clear, auto_gain);
}

static ssize_t light_reg_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct veml3328_data *drv_data = dev_get_drvdata(dev);
	u8 reg = 0;
	int offset = 0;
	u16 val = 0;

	for (reg = 0x00; reg <= 0x08; reg++) {
		veml3328_i2c_read_word(drv_data, reg, &val);
		SENSOR_INFO("Read Reg: 0x%2x Value: 0x%4x\n", reg, val);
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Reg: 0x%2x Value: 0x%4x\n", reg, val);
	}

	return offset;
}

static ssize_t light_reg_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct veml3328_data *drv_data = dev_get_drvdata(dev);
	int reg, val, ret;

	if (sscanf(buf, "%2x,%4x", &reg, &val) != 2) {
		SENSOR_ERR("invalid value\n");
		return count;
	}

	ret = veml3328_i2c_write_word(drv_data, reg, val);
	if(!ret)
		SENSOR_INFO("Register(0x%2x) data(0x%4x)\n", reg, val);
	else
		SENSOR_ERR("failed %d\n", ret);

	return count;
}

static int read_window_type(void)
{
	struct file *type_filp = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	char window_type[10] = {0, };

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	type_filp = filp_open(WINDOW_TYPE_FILE_PATH, O_RDONLY, 0440);
	if (IS_ERR(type_filp)) {
		ret = PTR_ERR(type_filp);
		SENSOR_ERR("open fail window_type:%d\n", ret);
		goto err_open_exit;
	}

	ret = vfs_read(type_filp, (char *)window_type,
		10 * sizeof(char), &type_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd read fail:%d\n", ret);
		ret = -EIO;
		goto err_read_exit;
	}

	SENSOR_INFO("0x%x, 0x%x, 0x%x",
		window_type[0], window_type[1], window_type[2]);
	ret = (window_type[1] - '0') & 0x0f;

err_read_exit:
	filp_close(type_filp, current->files);
err_open_exit:
	set_fs(old_fs);

	return ret;
}

static char *strtok_first_dot(char *str)
{
	static char *s;
	int i, len;

	if (str == NULL || *str == '\0')
		return NULL;

	s = str;
	len = (int)strlen(str);
	for (i = 0 ; i < len; i++) {
		if (s[i] == '.') {
			s[i] = '\0';
			return s;
		}
	}

	return s;
}

static int need_update_coef_efs(void)
{
	struct file *type_filp = NULL;
	mm_segment_t old_fs;
	int ret = 0, current_coef_version = 0;
	char coef_version[VERSION_FILE_NAME_LEN] = {0, };
	char *temp_version;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	type_filp = filp_open("/efs/FactoryApp/hh_version", O_RDONLY, 0440);
	if (PTR_ERR(type_filp) == -ENOENT || PTR_ERR(type_filp) == -ENXIO) {
		SENSOR_ERR("no version file\n");
		set_fs(old_fs);
		return true;
	} else if (IS_ERR(type_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(type_filp);
		SENSOR_ERR("open fail version:%d\n", ret);
		return ret;
	}

	ret = vfs_read(type_filp, (char *)coef_version,
		VERSION_FILE_NAME_LEN * sizeof(char), &type_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd read fail:%d\n", ret);
		ret = -EIO;
	}

	filp_close(type_filp, current->files);
	set_fs(old_fs);

	temp_version = strtok_first_dot(coef_version);
	if (temp_version == '\0') {
		SENSOR_ERR("Dot NULL.\n");
		return false;
	}

	ret = kstrtoint(temp_version, 10, &current_coef_version);
	SENSOR_INFO("%s,%s,%d\n",
		coef_version, temp_version, current_coef_version);

	if (ret < 0) {
		SENSOR_ERR("kstrtoint failed:%d\n", ret);
		return ret;
	}

	if (current_coef_version < hidden_table[ID_INDEX_NUMS - 1].version) {
		SENSOR_ERR("small:%d:%d", current_coef_version,
			hidden_table[ID_INDEX_NUMS - 1].version);
		return true;
	}

	return false;
}

int check_crc_table(int index)
{
	int i, sum = 0;

	for (i = 0; i < SUM_CRC; i++)
		sum += hidden_table[index].data[i];

	return (sum == hidden_table[index].data[SUM_CRC]) ? true : false;
}

int make_coef_efs(int index)
{
	struct file *type_filp = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	char *predefine_value_path = kzalloc(PATH_LEN + 1, GFP_KERNEL);
	char *write_buf = kzalloc(FILE_BUF_LEN, GFP_KERNEL);

	snprintf(predefine_value_path, PATH_LEN,
		"/efs/FactoryApp/predefine%d", hidden_table[index].octa_id);

	SENSOR_INFO("path:%s\n", predefine_value_path);

	sprintf(write_buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		hidden_table[index].data[0], hidden_table[index].data[1],
		hidden_table[index].data[2], hidden_table[index].data[3],
		hidden_table[index].data[4], hidden_table[index].data[5],
		hidden_table[index].data[6], hidden_table[index].data[7],
		hidden_table[index].data[8], hidden_table[index].data[9],
		hidden_table[index].data[10]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	type_filp = filp_open(predefine_value_path,
			O_TRUNC | O_RDWR | O_CREAT, 0660);

	if (IS_ERR(type_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(type_filp);
		SENSOR_ERR("open fail predefine_value_path:%d\n", ret);
		kfree(write_buf);
		kfree(predefine_value_path);
		return ret;
	}

	ret = vfs_write(type_filp, (char *)write_buf,
		FILE_BUF_LEN * sizeof(char), &type_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd write %d\n", ret);
		ret = -EIO;
	}

	filp_close(type_filp, current->files);
	set_fs(old_fs);
	kfree(write_buf);
	kfree(predefine_value_path);

	return ret;
}

static void veml3328_itoa(char *buf, int v)
{
	int mod[10];
	int i;

	for (i = 0; i < 3; i++) {
		mod[i] = (v % 10);
		v = v / 10;
		if (v == 0)
			break;
	}

	if (i == 3)
		i--;

	if (i >= 1)
		*buf = (char) ('a' + mod[0]);
	else
		*buf = (char) ('0' + mod[0]);

	buf++;
	*buf = '\0';
}

int update_coef_version(void)
{
	struct file *type_filp = NULL;
	mm_segment_t old_fs;
	char version[VERSION_FILE_NAME_LEN] = {0,};
	char tmp[5] = {0,};
	int i = 0, ret = 0;

	sprintf(version, "%d.",	hidden_table[ID_INDEX_NUMS - 1].version);

	for (i = 0 ; i < ID_INDEX_NUMS ; i++) {
		if (check_crc_table(i)) {
			veml3328_itoa(tmp, hidden_table[i].octa_id);
			strncat(version, tmp, 1);
			SENSOR_ERR("version:%s\n", version);
		}
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	type_filp = filp_open("/efs/FactoryApp/hh_version",
			O_TRUNC | O_RDWR | O_CREAT, 0660);
	if (IS_ERR(type_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(type_filp);
		SENSOR_ERR("open fail version:%d\n", ret);
		return ret;
	}

	ret = vfs_write(type_filp, (char *)version,
		VERSION_FILE_NAME_LEN * sizeof(char), &type_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd write fail:%d\n", ret);
		ret = -EIO;
	}

	filp_close(type_filp, current->files);
	set_fs(old_fs);

	return ret;
}

int hidden_hole_init_work(struct veml3328_data *drv_data)
{
	struct file *hole_filp = NULL;
	mm_segment_t old_fs;
	int ret = 0, win_type = 0, i;
	char *predefine_value_path = kzalloc(PATH_LEN + 1, GFP_KERNEL);
	char *read_buf = kzalloc(FILE_BUF_LEN * sizeof(char), GFP_KERNEL);

	SENSOR_INFO("start!\n");
	if (need_update_coef_efs()) {
		for (i = 0 ; i < ID_INDEX_NUMS ; i++) {
			if (!check_crc_table(i)) {
				SENSOR_ERR("CRC check fail (%d)\n", i);
				ret = -1;
			}
		}

		if (ret == 0) {
			for (i = 0 ; i < ID_INDEX_NUMS ; i++) {
				ret = make_coef_efs(i);
				if (ret < 0)
					pr_err("[FACTORY] %s: NUCE fail:%d\n",
					__func__, i);
			}
			update_coef_version();
		} else {
			SENSOR_ERR("can't not update/make coef_efs\n");
		}
	}

	win_type = read_window_type();
	if (win_type >= 0)
		snprintf(predefine_value_path, PATH_LEN,
			"/efs/FactoryApp/predefine%d", win_type);
	else {
		SENSOR_ERR("win_type fail\n");
		ret = -1;
		goto exit;
	}

	SENSOR_INFO("win_type:%d, %s\n", win_type, predefine_value_path);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	hole_filp = filp_open(predefine_value_path, O_RDONLY, 0440);
	if (IS_ERR(hole_filp)) {
		ret = PTR_ERR(hole_filp);
		SENSOR_ERR("Can't open hidden hole file:%d\n", ret);
		set_fs(old_fs);
		goto exit;
	}

	ret = vfs_read(hole_filp, (char *)read_buf,
		FILE_BUF_LEN * sizeof(char), &hole_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd read fail:%d\n", ret);
		filp_close(hole_filp, current->files);
		set_fs(old_fs);
		goto exit;
	}

	filp_close(hole_filp, current->files);
	set_fs(old_fs);

	ret = sscanf(read_buf, "%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d",
		&drv_data->coef[0], &drv_data->coef[1], &drv_data->coef[2],
		&drv_data->coef[3], &drv_data->coef[4], &drv_data->coef[5],
		&drv_data->coef[6], &drv_data->coef[7], &drv_data->coef[8],
		&drv_data->coef[9], &drv_data->coef[10]);
	if (ret != EFS_SAVE_NUMS) {
		SENSOR_ERR("sscanf fail:%d\n", ret);
		ret = -1;
		goto exit;
	}

	SENSOR_INFO("data: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		drv_data->coef[0], drv_data->coef[1], drv_data->coef[2],
		drv_data->coef[3], drv_data->coef[4], drv_data->coef[5],
		drv_data->coef[6], drv_data->coef[7], drv_data->coef[8],
		drv_data->coef[9], drv_data->coef[10]);
exit:
	kfree(read_buf);
	kfree(predefine_value_path);

	return ret;
}

static int hh_check_crc(void)
{
	struct file *hole_filp = NULL;
	int msg_buf[EFS_SAVE_NUMS];
	mm_segment_t old_fs;
	int i, j, sum = 0, ret = 0;
	char efs_version[VERSION_FILE_NAME_LEN] = {0, };
	char *predefine_value_path = kzalloc(PATH_LEN + 1, GFP_KERNEL);
	char *read_buf = kzalloc(FILE_BUF_LEN, GFP_KERNEL);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	hole_filp = filp_open("/efs/FactoryApp/hh_version", O_RDONLY, 0440);
	if (IS_ERR(hole_filp)) {
		SENSOR_INFO("open fail\n");
		ret = PTR_ERR(hole_filp);
		goto crc_err_open_ver;
	}

	ret = vfs_read(hole_filp, (char *)&efs_version,
		VERSION_FILE_NAME_LEN * sizeof(char), &hole_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd read fail:%d\n", ret);
		goto crc_err_read_ver;
	}
	efs_version[VERSION_FILE_NAME_LEN - 1] = '\0';

	SENSOR_INFO("efs_version:%s\n", efs_version);

	filp_close(hole_filp, current->files);
	set_fs(old_fs);

	i = ID_MAX;
	while (efs_version[i] >= '0' && efs_version[i] <= 'f') {
		if (efs_version[i] >= 'a')
			snprintf(predefine_value_path, PATH_LEN,
				"/efs/FactoryApp/predefine%d",
				efs_version[i] - 'a' + 10);
		else
			snprintf(predefine_value_path, PATH_LEN,
				"/efs/FactoryApp/predefine%d",
				efs_version[i] - '0');
		SENSOR_INFO("path:%s\n", predefine_value_path);

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		hole_filp = filp_open(predefine_value_path, O_RDONLY, 0440);
		if (IS_ERR(hole_filp)) {
			set_fs(old_fs);
			ret = PTR_ERR(hole_filp);
			SENSOR_ERR("Can't open hidden hole file:%d\n", ret);
			goto crc_err_open;
		}

		ret = vfs_read(hole_filp, (char *)read_buf,
			FILE_BUF_LEN * sizeof(char), &hole_filp->f_pos);
		if (ret < 0) {
			SENSOR_ERR("fd read fail:%d\n", ret);
			goto crc_err_read;
		}

		filp_close(hole_filp, current->files);
		set_fs(old_fs);

		ret = sscanf(read_buf,
			"%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d",
			&msg_buf[0], &msg_buf[1], &msg_buf[2], &msg_buf[3],
			&msg_buf[4], &msg_buf[5], &msg_buf[6], &msg_buf[7],
			&msg_buf[8], &msg_buf[9], &msg_buf[10]);
		if (ret != EFS_SAVE_NUMS) {
			SENSOR_ERR("sscanf fail:%d\n", ret);
			goto crc_err_sum;
		}

		for (j = 0, sum = 0; j < SUM_CRC; j++)
			sum += msg_buf[j];
		if (sum != msg_buf[SUM_CRC]) {
			SENSOR_ERR("CRC error %d:%d\n", sum, msg_buf[SUM_CRC]);
			ret = -1;
			goto crc_err_sum;
		}
		i++;
	}

	goto exit;

crc_err_read_ver:
crc_err_read:
	filp_close(hole_filp, current->files);
crc_err_open_ver:
	set_fs(old_fs);
crc_err_open:
crc_err_sum:
exit:
	kfree(read_buf);
	kfree(predefine_value_path);
	return ret;
}

static ssize_t hh_ver_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{

	struct file *type_filp = NULL;
	mm_segment_t old_fs;
	int ret = 0;
	char version[VERSION_FILE_NAME_LEN] = {0, };

	if ((size < 2) || (size > VERSION_FILE_NAME_LEN)) {
		SENSOR_ERR("size %d error\n", (int)size);
		return -EINVAL;
	}

	strlcpy(version, &buf[1], size);
	pr_info("[FACTORY] %s: buf: %s\n", __func__, version);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	type_filp = filp_open("/efs/FactoryApp/hh_version",
		O_TRUNC | O_RDWR | O_CREAT, 0660);
	if (IS_ERR(type_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(type_filp);
		SENSOR_ERR("open fail version:%d\n", ret);
		return size;
	}

	ret = vfs_write(type_filp, version,
		VERSION_FILE_NAME_LEN * sizeof(char), &type_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd write fail:%d\n", ret);
		ret = -EIO;
	}

	filp_close(type_filp, current->files);
	set_fs(old_fs);

	return size;
}

static ssize_t hh_ver_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct file *ver_filp = NULL;
	mm_segment_t old_fs;
	char efs_version[VERSION_FILE_NAME_LEN] = {0, };
	char table_version[VERSION_FILE_NAME_LEN] = {0, };
	char tmp[5] = {0,};
	int i = 0, ret = 0;

	ret = hh_check_crc();
	if (ret < 0) {
		SENSOR_ERR("CRC check error:%d\n", ret);
		goto err_check_crc;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	ver_filp = filp_open("/efs/FactoryApp/hh_version", O_RDONLY, 0440);
	if (IS_ERR(ver_filp)) {
		SENSOR_ERR("open fail\n");
		goto err_open_fail;
	}

	ret = vfs_read(ver_filp, (char *)&efs_version,
		VERSION_FILE_NAME_LEN * sizeof(char), &ver_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd read fail:%d\n", ret);
		goto err_fail;
	}
	efs_version[VERSION_FILE_NAME_LEN - 1] = '\0';

	filp_close(ver_filp, current->files);
	set_fs(old_fs);

	sprintf(table_version, "%d.", hidden_table[ID_INDEX_NUMS - 1].version);

	for (i = 0 ; i < ID_INDEX_NUMS ; i++) {
		veml3328_itoa(tmp, hidden_table[i].octa_id);
		strlcat(table_version, tmp, sizeof(table_version));
		SENSOR_ERR("version:%s\n", table_version);
	}

	SENSOR_INFO(" ver:%s:%s\n", efs_version, table_version);

	return snprintf(buf, PAGE_SIZE, "P%s,P%s\n",
		efs_version, table_version);
err_fail:
	filp_close(ver_filp, current->files);
err_open_fail:
	set_fs(old_fs);
err_check_crc:
	pr_info("[FACTORY] %s: fail\n", __func__);
	return snprintf(buf, PAGE_SIZE, "0,0\n");
}

static ssize_t hh_write_all_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct file *type_filp = NULL;
	int msg_buf[EFS_SAVE_NUMS];
	mm_segment_t old_fs;
	int ret = 0, octa_id = 0;
	char *predefine_value_path = kzalloc(PATH_LEN + 1, GFP_KERNEL);
	char *write_buf = kzalloc(FILE_BUF_LEN, GFP_KERNEL);

	/* D_FACTOR,
	R_COEF,G_COEF,B_COEF,C_COEF,CT_COEF,CT_OFFSET,
	THD_HIGH,THD_LOW,IRIS_PROX_THD,SUM_CRC,EFS_SAVE_NUMS, */

	ret = sscanf(buf, "%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d,%9d",
		&octa_id, &msg_buf[0], &msg_buf[1], &msg_buf[2], &msg_buf[3],
		&msg_buf[4], &msg_buf[5], &msg_buf[6], &msg_buf[7], &msg_buf[8],
		&msg_buf[9], &msg_buf[10]);
	if (ret != EFS_SAVE_NUMS + 1) {
		SENSOR_ERR("sscanf fail:%d\n", ret);
		kfree(write_buf);
		kfree(predefine_value_path);
		return size;
	}

	SENSOR_INFO("ID:%d, DATA: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		octa_id, msg_buf[0], msg_buf[1], msg_buf[2],
		msg_buf[3], msg_buf[4], msg_buf[5], msg_buf[6], msg_buf[7],
		msg_buf[8], msg_buf[9], msg_buf[10]);

	snprintf(write_buf, FILE_BUF_LEN, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		msg_buf[0], msg_buf[1], msg_buf[2], msg_buf[3], msg_buf[4],
		msg_buf[5], msg_buf[6], msg_buf[7], msg_buf[8], msg_buf[9],
		msg_buf[10]);

	snprintf(predefine_value_path, PATH_LEN,
		"/efs/FactoryApp/predefine%d", octa_id);

	SENSOR_INFO("path:%s\n", predefine_value_path);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	type_filp = filp_open(predefine_value_path,
			O_TRUNC | O_RDWR | O_CREAT, 0660);
	if (IS_ERR(type_filp)) {
		set_fs(old_fs);
		ret = PTR_ERR(type_filp);
		SENSOR_ERR("open fail predefine_value_path:%d\n", ret);
		kfree(write_buf);
		kfree(predefine_value_path);
		return size;
	}

	ret = vfs_write(type_filp, (char *)write_buf,
		FILE_BUF_LEN * sizeof(char), &type_filp->f_pos);
	if (ret < 0) {
		SENSOR_ERR("fd write:%d\n", ret);
		ret = -EIO;
	}

	filp_close(type_filp, current->files);
	set_fs(old_fs);
	kfree(write_buf);
	kfree(predefine_value_path);
	return size;
}

static ssize_t hh_write_all_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int i;

	for (i = 0; i < ID_INDEX_NUMS ; i++) {
		if (!check_crc_table(i)) {
			SENSOR_ERR("CRC check fail (%d)\n", i);
			return snprintf(buf, PAGE_SIZE, "%s\n", "FALSE");
		}
	}

	for (i = 0; i < ID_INDEX_NUMS ; i++) {
		ret = make_coef_efs(i);
		if (ret < 0)
			SENSOR_ERR("make_coef_efs fail:%d\n", i);
	}

	ret = hh_check_crc();
	SENSOR_INFO("success to write all data:%d\n", ret);

	if (ret < 0)
		return snprintf(buf, PAGE_SIZE, "%s\n", "FALSE");
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "TRUE");
}

static ssize_t hh_is_exist_efs_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct file *hole_filp = NULL;
	mm_segment_t old_fs;
	int retry_cnt = 0, win_type;
	bool is_exist = false;
	char *predefine_value_path = kzalloc(PATH_LEN + 1, GFP_KERNEL);

	win_type = read_window_type();
	if (win_type >= 0)
		snprintf(predefine_value_path, PATH_LEN,
			"/efs/FactoryApp/predefine%d", win_type);
	else {
		SENSOR_ERR("win_type fail\n");
		kfree(predefine_value_path);
		return snprintf(buf, PAGE_SIZE, "%s\n", "FALSE");
	}

	SENSOR_INFO("win:%d:%s\n", win_type, predefine_value_path);
	old_fs = get_fs();
	set_fs(KERNEL_DS);

retry_open_efs:
	hole_filp = filp_open(predefine_value_path, O_RDONLY, 0440);

	if (IS_ERR(hole_filp)) {
		SENSOR_ERR("open fail fail:%d\n", IS_ERR(hole_filp));
		if (retry_cnt < RETRY_MAX) {
			retry_cnt++;
			goto retry_open_efs;
		} else
			is_exist = false;
	} else {
		filp_close(hole_filp, current->files);
		is_exist = true;
	}

	set_fs(old_fs);
	kfree(predefine_value_path);

	SENSOR_INFO("is_exist:%d, retry :%d\n", is_exist, retry_cnt);

	if (is_exist)
		return snprintf(buf, PAGE_SIZE, "%s\n", "TRUE");
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "FALSE");
}

static DEVICE_ATTR(name, 0444, light_name_show, NULL);
static DEVICE_ATTR(vendor, 0444, light_vendor_show, NULL);
static DEVICE_ATTR(lux, 0444, light_lux_show, NULL);
static DEVICE_ATTR(raw_data, 0444, light_data_show, NULL);
static DEVICE_ATTR(reg_data, 0664, light_reg_data_show, light_reg_data_store);

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_lux,
	&dev_attr_raw_data,
	&dev_attr_reg_data,
	NULL,
};

static DEVICE_ATTR(hh_ver, 0664, hh_ver_show, hh_ver_store);
static DEVICE_ATTR(hh_write_all_data, 0664,
	hh_write_all_data_show, hh_write_all_data_store);
static DEVICE_ATTR(hh_is_exist_efs, 0444, hh_is_exist_efs_show, NULL);

static struct device_attribute *hidden_sensor_attrs[] = {
	&dev_attr_hh_ver,
	&dev_attr_hh_write_all_data,
	&dev_attr_hh_is_exist_efs,
	NULL,
};

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		light_poll_delay_show, light_poll_delay_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		light_enable_show, light_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL,
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static int veml3328_init_input_device(struct veml3328_data *drv_data)
{
	int ret = 0;
	struct input_dev *dev;

	/* allocate lightsensor input_device */
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = MODULE_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_MISC);
	input_set_capability(dev, EV_REL, REL_WHEEL);
	input_set_drvdata(dev, drv_data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		dev = NULL;
		return ret;
	}

	ret = sensors_create_symlink(&dev->dev.kobj, dev->name);
	if (ret < 0) {
		input_unregister_device(dev);
		return ret;
	}

	ret = sysfs_create_group(&dev->dev.kobj, &light_attribute_group);
	if (ret < 0) {
		sensors_remove_symlink(&dev->dev.kobj, dev->name);
		input_unregister_device(dev);
		return ret;
	}

	drv_data->input_dev = dev;
	return 0;
}

static int veml3328_init_registers(struct veml3328_data *drv_data)
{
	int ret = 0;
	u16 val = 0;

	// Set integration time
	val = VEML3328_CONF_IT_100MS;

	// Set gain
	val |= VEML3328_CONF_GAIN1_X4;
	val |= VEML3328_CONF_GAIN2_X4;

	// Shut down VEML3328
	val |= VEML3328_CONF_SD;

	ret = veml3328_i2c_write_word(drv_data, VEML3328_CONF, val);
	if(ret)
		SENSOR_INFO("failed, ret=%d\n", ret);

	return ret;
}

static int veml3328_device_id_check(struct veml3328_data *drv_data)
{
	int ret = 0;
	u16 val = 0;

	ret = veml3328_i2c_read_word(drv_data, VEML3328_DEVICE_ID_REG, &val);

	if (!ret) {
		val = val & 0x00FF;
		if (val == VEML3328_DEVICE_ID_VAL) {
			SENSOR_INFO("device matched, id=%d\n", val);
			return 0;
		} else {
			SENSOR_INFO("device not matched, id=%d\n", val);
			return -ENODEV;
		}
	}

	return ret;
}

static int veml3328_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct veml3328_data *drv_data;

	SENSOR_INFO("\n");

	drv_data = kzalloc(sizeof(struct veml3328_data), GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	drv_data->i2c_client = client;
	i2c_set_clientdata(client, drv_data);

	// Check if VEML3328 IC exists
	ret = veml3328_device_id_check(drv_data);
	if (ret) {
		SENSOR_ERR("VEML3328 not found, ret=%d\n", ret);
		goto err_device_id_check;
	}

	ret = veml3328_init_registers(drv_data);
	if (ret) {
		SENSOR_ERR("Register init failed, ret=%d\n", ret);
		goto err_init_registers;
	}

	ret = veml3328_init_input_device(drv_data);
	if (ret) {
		SENSOR_ERR("Input device init failed, ret=%d\n", ret);
		goto err_init_input_device;
	}

	hrtimer_init(&drv_data->light_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv_data->light_poll_delay = ns_to_ktime(DEFAULT_DELAY_MS * NSEC_PER_MSEC);
	drv_data->light_timer.function = light_timer_func;

	drv_data->light_wq = create_singlethread_workqueue("veml3328_light_wq");
	if (!drv_data->light_wq) {
		SENSOR_ERR("Can't create workqueue\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	INIT_WORK(&drv_data->light_work, light_work_func);

	mutex_init(&drv_data->control_mutex);

	/* set sysfs for light sensor */
	ret = sensors_register(&drv_data->ls_dev, drv_data,
		sensor_attrs, MODULE_NAME);
	if (ret < 0) {
		SENSOR_ERR("Sensor registration failed, ret=%d\n", ret);
		goto err_sensors_register;
	}

	/* set sysfs for hidden sysfs */
	ret = sensors_register(&drv_data->ls_dev, drv_data, 
		hidden_sensor_attrs, MODULE_NAME_HIDDEN);
	if (ret < 0) {
		SENSOR_ERR("Sensor registration failed, ret=%d\n", ret);
		goto err_hidden_sensors_register;
	}
	
	drv_data->als_enable = OFF;
	drv_data->gain_level = GAIN_X8;
	drv_data->is_first_enable = true;

	SENSOR_INFO("Probe success!\n");

	return ret;

err_hidden_sensors_register:
	sensors_unregister(drv_data->ls_dev, sensor_attrs);
err_sensors_register:
	mutex_destroy(&drv_data->control_mutex);
	destroy_workqueue(drv_data->light_wq);
err_create_singlethread_workqueue:
	sensors_remove_symlink(&drv_data->input_dev->dev.kobj, drv_data->input_dev->name);
	input_unregister_device(drv_data->input_dev);
err_init_input_device:
err_init_registers:
err_device_id_check:
	kfree(drv_data);
	return ret;
}

static void veml3328_shutdown(struct i2c_client *client)
{
	struct veml3328_data *data = i2c_get_clientdata(client);

	pr_info("[SENSOR]: %s\n", __func__);
	if (data->als_enable)
		lightsensor_disable(data);
}

static int veml3328_suspend(struct device *dev)
{
	struct veml3328_data *data = dev_get_drvdata(dev);

	pr_info("[SENSOR]: %s\n", __func__);
	if (data->als_enable)
		lightsensor_disable(data);

	return 0;
}

static int veml3328_resume(struct device *dev)
{
	struct veml3328_data *data = dev_get_drvdata(dev);

	pr_info("[SENSOR]: %s\n", __func__);
	if (data->als_enable)
		lightsensor_enable(data);

	return 0;
}

static const struct i2c_device_id veml3328_i2c_id[] = {
	{CHIP_NAME, 0},
	{}
};

static const struct dev_pm_ops veml3328_pm_ops = {
	.suspend = veml3328_suspend,
	.resume = veml3328_resume
};

#ifdef CONFIG_OF
  static struct of_device_id veml3328_match_table[] = {
		  { .compatible = "capella,veml3328",},
		  { },
  };
#else
  #define veml3328_match_table NULL
#endif

static struct i2c_driver veml3328_driver = {
	.id_table = veml3328_i2c_id,
	.probe = veml3328_probe,
	.shutdown = veml3328_shutdown,	
	.driver = {
		.name = CHIP_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(veml3328_match_table),
		.pm = &veml3328_pm_ops
	},
};

static int __init veml3328_init(void)
{
	return i2c_add_driver(&veml3328_driver);
}

static void __exit veml3328_exit(void)
{
	i2c_del_driver(&veml3328_driver);
}

module_init(veml3328_init);
module_exit(veml3328_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Ambient light sensor driver for capella veml3328");
MODULE_LICENSE("GPL");
