/* driver/sensor/cm36686.c
 * Copyright (c) 2011 SAMSUNG
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/types.h>
#include <linux/regulator/consumer.h>

#include <linux/sensor/sensors_core.h>

#define PROXIMITY_FOR_TEST	/* for HW to tune up */

#define MODULE_NAME_PROX   "proximity_sensor"
#define MODULE_NAME_LIGHT  "light_sensor"

#define VENDOR                  "CAPELLA"
#define CHIP_ID                 "CM36686"

#define I2C_M_WR                0 /* for i2c Write */

#define ALS_REG_NUM             2

/* light sensor register addresses */
#define REG_CS_CONF1            0x00
#define REG_ALS_DATA            0x09
#define REG_WHITE_DATA          0x0A

/* register settings */
static u16 als_reg_setting[ALS_REG_NUM][2] = {
	{REG_CS_CONF1, 0x0000}, /* enable */
	{REG_CS_CONF1, 0x0001}, /* disable */
};

enum {
	LIGHT_ENABLED = BIT(0),
	PROXIMITY_ENABLED = BIT(1),
};

enum {
	PS_CONF1 = 0,
	PS_CONF3,
	PS_THD_LOW,
	PS_THD_HIGH,
	PS_CANCEL,
	PS_REG_NUM,
};

enum {
	REG_ADDR = 0,
	CMD,
};

/* proximity sensor regsiter addresses */
#define REG_PS_CONF1            0x03
#define REG_PS_CONF3            0x04
#define REG_PS_CANC             0x05
#define REG_PS_THD_LOW          0x06
#define REG_PS_THD_HIGH         0x07
#define REG_PS_DATA             0x08

/* proximity sensor default value for register */
#define DEFAULT_HI_THD          0x0015
#define DEFAULT_LOW_THD         0x000F
#define CANCEL_HI_THD           0x000F
#define CANCEL_LOW_THD          0x000A
#define DEFAULT_CONF1           0x03A4
#define DEFAULT_CONF3           0x4210
#define DEFAULT_TRIM            0x0000

static u16 ps_reg_init_setting[PS_REG_NUM][2] = {
	{REG_PS_CONF1, DEFAULT_CONF1}, /* REG_PS_CONF1 */
	{REG_PS_CONF3, DEFAULT_CONF3}, /* REG_PS_CONF3 */
	{REG_PS_THD_LOW, DEFAULT_LOW_THD}, /* REG_PS_THD_LOW */
	{REG_PS_THD_HIGH, DEFAULT_HI_THD}, /* REG_PS_THD_HIGH */
	{REG_PS_CANC, DEFAULT_TRIM}, /* REG_PS_CANC */
};

 /* Intelligent Cancelation*/
#define CANCELATION_FILE_PATH   "/efs/FactoryApp/prox_cal"
#define CAL_SKIP_ADC    10 /* nondetect threshold *60% */
#define CAL_FAIL_ADC    18 /* detect threshold */
enum {
	CAL_FAIL = 0,
	CAL_CANCELATION,
	CAL_SKIP,
};

 /*lightsnesor log time 6SEC 200mec X 30*/
#define LIGHT_LOG_TIME  30
#define PROX_READ_NUM   40

enum {
	OFF = 0,
	ON,
};

/* driver data */
struct cm36686_data {
	struct i2c_client *i2c_client;
	struct wake_lock prox_wake_lock;
	struct input_dev *proximity_input_dev;
	struct input_dev *light_input_dev;
	struct mutex power_lock;
	struct mutex read_lock;
	struct hrtimer light_timer;
	struct hrtimer prox_timer;
	struct workqueue_struct *light_wq;
	struct workqueue_struct *prox_wq;
	struct work_struct work_light;
	struct work_struct work_prox;
	struct device *proximity_dev;
	struct device *light_dev;
	struct regulator *vdd;
	struct regulator *vled;

	ktime_t light_poll_delay;
	ktime_t prox_poll_delay;

	int ps_conf1;
	int ps_conf3;
	int default_hi_thd;
	int default_low_thd;
	int cancel_hi_thd;
	int cancel_low_thd;
	int default_trim;

	int cal_skip_adc;
	int cal_fail_adc;

	int vdd_always_on; /* 1: vdd is always on, 0: enable only when proximity is on */
	int vled_same_vdd;

	int vled_ldo; /*0: vled(anode) source regulator, other: get power by LDO control */

	int irq;
	int irq_gpio;

	u8 power_state;
	int avg[3];
	u16 als_data;
	u16 white_data;
	int count_log_time;
	unsigned int prox_cal_result;
};

static int sensor_vdd_onoff(struct device *dev, bool onoff);
static int proximity_vled_onoff(struct device *dev, bool onoff);

int cm36686_i2c_read_word(struct cm36686_data *cm36686, u8 command, u16 *val)
{
	int err = 0;
	int retry = 3;
	struct i2c_client *client = cm36686->i2c_client;
	struct i2c_msg msg[2];
	unsigned char data[2] = {0,};
	u16 value = 0;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	while (retry--) {
		/* send slave address & command */
		msg[0].addr = client->addr;
		msg[0].flags = I2C_M_WR;
		msg[0].len = 1;
		msg[0].buf = &command;

		/* read word data */
		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = 2;
		msg[1].buf = data;

		err = i2c_transfer(client->adapter, msg, 2);

		if (err >= 0) {
			value = (u16)data[1];
			*val = (value << 8) | (u16)data[0];
			return err;
		}
	}
	SENSOR_ERR("i2c transfer error ret=%d\n", err);

	return err;
}

int cm36686_i2c_write_word(struct cm36686_data *cm36686, u8 command, u16 val)
{
	int err = 0;
	struct i2c_client *client = cm36686->i2c_client;
	int retry = 3;

	if ((client == NULL) || (!client->adapter))
		return -ENODEV;

	while (retry--) {
		err = i2c_smbus_write_word_data(client, command, val);
		if (err >= 0)
			return 0;
	}
	SENSOR_ERR("i2c transfer error(%d)\n", err);
	return err;
}

static void cm36686_light_enable(struct cm36686_data *cm36686)
{
	/* enable setting */
	cm36686_i2c_write_word(cm36686, REG_CS_CONF1, als_reg_setting[0][1]);
	hrtimer_start(&cm36686->light_timer, ns_to_ktime(200 * NSEC_PER_MSEC),
		HRTIMER_MODE_REL);
}

static void cm36686_light_disable(struct cm36686_data *cm36686)
{
	/* disable setting */
	cm36686_i2c_write_word(cm36686, REG_CS_CONF1, als_reg_setting[1][1]);
	hrtimer_cancel(&cm36686->light_timer);
	cancel_work_sync(&cm36686->work_light);
}

static ssize_t cm36686_poll_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%lld\n",
		ktime_to_ns(cm36686->light_poll_delay));
}

static ssize_t cm36686_poll_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = kstrtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	mutex_lock(&cm36686->power_lock);
	if (new_delay != ktime_to_ns(cm36686->light_poll_delay)) {
		cm36686->light_poll_delay = ns_to_ktime(new_delay);
		if (cm36686->power_state & LIGHT_ENABLED) {
			cm36686_light_disable(cm36686);
			cm36686_light_enable(cm36686);
		}
		SENSOR_INFO("poll_delay = %lld\n", new_delay);
	}
	mutex_unlock(&cm36686->power_lock);

	return size;
}

static ssize_t light_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}
	SENSOR_INFO("new_value = %d\n", new_value);

	mutex_lock(&cm36686->power_lock);
	if (new_value && !(cm36686->power_state & LIGHT_ENABLED)) {
		cm36686->power_state |= LIGHT_ENABLED;
		cm36686_light_enable(cm36686);
	} else if (!new_value && (cm36686->power_state & LIGHT_ENABLED)) {
		cm36686_light_disable(cm36686);
		cm36686->power_state &= ~LIGHT_ENABLED;
	}
	mutex_unlock(&cm36686->power_lock);

	return size;
}

static ssize_t light_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
		(cm36686->power_state & LIGHT_ENABLED) ? 1 : 0);
}

static int proximity_open_cancelation(struct cm36686_data *data)
{
	struct file *cancel_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cancel_filp = filp_open(CANCELATION_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(cancel_filp)) {
		err = PTR_ERR(cancel_filp);
		if (err != -ENOENT)
			SENSOR_ERR("Can't open cancelation file(%d)\n", err);
		set_fs(old_fs);
		return err;
	}

	err = vfs_read(cancel_filp,
		(char *)&ps_reg_init_setting[PS_CANCEL][CMD],
		sizeof(u16), &cancel_filp->f_pos);
	if (err != sizeof(u16)) {
		SENSOR_ERR("Can't read the cancel data from file(%d)\n", err);
		err = -EIO;
	}

	/*If there is an offset cal data. */
	if (ps_reg_init_setting[PS_CANCEL][CMD] != data->default_trim) {
		ps_reg_init_setting[PS_THD_HIGH][CMD] =
			data->cancel_hi_thd ?
			data->cancel_hi_thd :
			CANCEL_HI_THD;
		ps_reg_init_setting[PS_THD_LOW][CMD] =
			data->cancel_low_thd ?
			data->cancel_low_thd :
			CANCEL_LOW_THD;
	}

	SENSOR_INFO("prox_cal = 0x%x, high_thd = 0x%x, low_thd = 0x%x\n",
		ps_reg_init_setting[PS_CANCEL][CMD],
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);

	filp_close(cancel_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int proximity_store_cancelation(struct device *dev, bool do_calib)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	struct file *cancel_filp = NULL;
	mm_segment_t old_fs;
	int err;
	u16 ps_data = 0;

	if (do_calib) {
		mutex_lock(&cm36686->read_lock);
		cm36686_i2c_read_word(cm36686, REG_PS_DATA, &ps_data);
		mutex_unlock(&cm36686->read_lock);

		SENSOR_INFO("do cal read data %d\n", ps_data);

		if (ps_data < cm36686->cal_skip_adc) {
			/* SKIP. CAL_SKIP_ADC */
			ps_reg_init_setting[PS_CANCEL][CMD] =
				cm36686->default_trim;
			SENSOR_INFO("crosstalk < %d SKIP!!\n", cm36686->cal_skip_adc);
			cm36686->prox_cal_result = CAL_SKIP;
		} else if (ps_data <= cm36686->cal_fail_adc) {
			/* CANCELATION. CAL_FAIL_ADC */
			ps_reg_init_setting[PS_CANCEL][CMD] =
				cm36686->default_trim + ps_data;
			SENSOR_INFO("crosstalk_offset = %u Canceled", ps_data);
			cm36686->prox_cal_result = CAL_CANCELATION;
		} else {
			/*FAIL*/
			ps_reg_init_setting[PS_CANCEL][CMD] =
				cm36686->default_trim;
			SENSOR_INFO("crosstalk > %d\n", cm36686->cal_fail_adc);
			cm36686->prox_cal_result = CAL_FAIL;
		}

		if (cm36686->prox_cal_result == CAL_CANCELATION) {
			ps_reg_init_setting[PS_THD_HIGH][CMD] =
				cm36686->cancel_hi_thd ?
				cm36686->cancel_hi_thd :
				CANCEL_HI_THD;
			ps_reg_init_setting[PS_THD_LOW][CMD] =
				cm36686->cancel_low_thd ?
				cm36686->cancel_low_thd :
				CANCEL_LOW_THD;
		} else {
			ps_reg_init_setting[PS_THD_HIGH][CMD] =
				cm36686->default_hi_thd ?
				cm36686->default_hi_thd :
				DEFAULT_HI_THD;
			ps_reg_init_setting[PS_THD_LOW][CMD] =
				cm36686->default_low_thd ?
				cm36686->default_low_thd :
				DEFAULT_LOW_THD;
		}
	} else { /* reset */
		ps_reg_init_setting[PS_CANCEL][CMD] =
			cm36686->default_trim;
		ps_reg_init_setting[PS_THD_HIGH][CMD] =
			cm36686->default_hi_thd ?
			cm36686->default_hi_thd :
			DEFAULT_HI_THD;
		ps_reg_init_setting[PS_THD_LOW][CMD] =
			cm36686->default_low_thd ?
			cm36686->default_low_thd :
			DEFAULT_LOW_THD;
	}

	if ((cm36686->prox_cal_result == CAL_CANCELATION) || !do_calib) {
		err = cm36686_i2c_write_word(cm36686, REG_PS_CANC,
			ps_reg_init_setting[PS_CANCEL][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36686_ps_canc_reg is failed. %d\n", err);
		err = cm36686_i2c_write_word(cm36686, REG_PS_THD_HIGH,
			ps_reg_init_setting[PS_THD_HIGH][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36686_ps_high_reg is failed. %d\n", err);
		err = cm36686_i2c_write_word(cm36686, REG_PS_THD_LOW,
			ps_reg_init_setting[PS_THD_LOW][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36686_ps_low_reg is failed. %d\n", err);
	}

	SENSOR_INFO("prox_cal = 0x%x, high_thd = 0x%x, low_thd = 0x%x\n",
		ps_reg_init_setting[PS_CANCEL][CMD],
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cancel_filp = filp_open(CANCELATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY | O_SYNC, 0660);
	if (IS_ERR(cancel_filp)) {
		set_fs(old_fs);
		err = PTR_ERR(cancel_filp);
		SENSOR_ERR("Can't open cancelation file(%d)\n", err);
		return err;
	}

	err = vfs_write(cancel_filp,
		(char *)&ps_reg_init_setting[PS_CANCEL][CMD],
		sizeof(u16), &cancel_filp->f_pos);
	if (err != sizeof(u16)) {
		SENSOR_ERR("Can't write the cancel data to file(%d)\n", err);
		err = -EIO;
	}

	filp_close(cancel_filp, current->files);
	set_fs(old_fs);

	if (!do_calib) /* delay for clearing */
		msleep(150);

	return err;
}

static ssize_t proximity_cancel_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	bool do_calib;
	int err;

	if (sysfs_streq(buf, "1")) /* calibrate cancelation value */
		do_calib = true;
	else if (sysfs_streq(buf, "0")) /* reset cancelation value */
		do_calib = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	err = proximity_store_cancelation(dev, do_calib);
	if (err < 0) {
		SENSOR_ERR("proximity_store_cancelation() failed(%d)\n", err);
		return err;
	}

	return size;
}

static ssize_t proximity_cancel_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u,%u,%u\n",
		ps_reg_init_setting[PS_CANCEL][CMD],
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);
}

static ssize_t proximity_cancel_pass_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	SENSOR_INFO("%u\n", cm36686->prox_cal_result);
	return snprintf(buf, PAGE_SIZE, "%u\n", cm36686->prox_cal_result);
}

static ssize_t proximity_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	SENSOR_INFO("new_value = %d\n", new_value);
	mutex_lock(&cm36686->power_lock);
	if (new_value && !(cm36686->power_state & PROXIMITY_ENABLED)) {
		u8 val = 1;
		int i;
		int err = 0;

		cm36686->power_state |= PROXIMITY_ENABLED;

		if (!cm36686->vled_same_vdd)
			proximity_vled_onoff(dev, ON);

		/* open cancelation data */
		err = proximity_open_cancelation(cm36686);
		if (err < 0 && err != -ENOENT)
			SENSOR_ERR("proximity_open_cancelation() failed(%d)\n",
				err);

		/* enable settings */
		for (i = 0; i < PS_REG_NUM; i++)
			cm36686_i2c_write_word(cm36686,
				ps_reg_init_setting[i][REG_ADDR],
				ps_reg_init_setting[i][CMD]);

		/*send far for input update*/
		input_report_abs(cm36686->proximity_input_dev, ABS_DISTANCE,
			val);
		val = gpio_get_value(cm36686->irq_gpio);
		/* 0 is close, 1 is far */
		input_report_abs(cm36686->proximity_input_dev, ABS_DISTANCE,
			val);
		input_sync(cm36686->proximity_input_dev);

		enable_irq(cm36686->irq);
		enable_irq_wake(cm36686->irq);
	} else if (!new_value && (cm36686->power_state & PROXIMITY_ENABLED)) {
		cm36686->power_state &= ~PROXIMITY_ENABLED;

		disable_irq_wake(cm36686->irq);
		disable_irq(cm36686->irq);
		/* disable settings */
		cm36686_i2c_write_word(cm36686, REG_PS_CONF1, 0x0001);

		if (!cm36686->vled_same_vdd)
			proximity_vled_onoff(dev, OFF);
	}
	mutex_unlock(&cm36686->power_lock);
	return size;
}

static ssize_t proximity_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
		(cm36686->power_state & PROXIMITY_ENABLED) ? 1 : 0);
}

static DEVICE_ATTR(poll_delay, 0664,
	cm36686_poll_delay_show, cm36686_poll_delay_store);

static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, 0664,
	light_enable_show, light_enable_store);
static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, 0664,
	proximity_enable_show, proximity_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

/* sysfs for vendor & name */
static ssize_t cm36686_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t cm36686_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}
static struct device_attribute dev_attr_prox_sensor_vendor =
	__ATTR(vendor, 0444, cm36686_vendor_show, NULL);
static struct device_attribute dev_attr_light_sensor_vendor =
	__ATTR(vendor, 0444, cm36686_vendor_show, NULL);
static struct device_attribute dev_attr_prox_sensor_name =
	__ATTR(name, 0444, cm36686_name_show, NULL);
static struct device_attribute dev_attr_light_sensor_name =
	__ATTR(name, 0444, cm36686_name_show, NULL);

/* proximity sensor sysfs */
static ssize_t proximity_trim_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", cm36686->default_trim);
}

#if defined(PROXIMITY_FOR_TEST)
static ssize_t proximity_trim_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36686_data *data = dev_get_drvdata(dev);
	u16 trim_value;
	int err;

	err = kstrtou16(buf, 10, &trim_value);
	if (err < 0) {
		SENSOR_ERR("kstrtoint failed.\n");
		return size;
	}
	SENSOR_INFO("trim_value: %u\n", trim_value);

	if (trim_value > -1) {
		data->default_trim = trim_value;
		ps_reg_init_setting[PS_CANCEL][CMD] = trim_value;
		data->default_trim = trim_value;
		err = cm36686_i2c_write_word(data, REG_PS_CANC,
			ps_reg_init_setting[PS_CANCEL][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36686_ps_canc is failed. %d\n", err);
		SENSOR_INFO("new trim_value = %u\n", trim_value);
		msleep(150);
	} else
		SENSOR_ERR("wrong trim_value (%u)!!\n", trim_value);

	return size;
}
#endif

static ssize_t proximity_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", cm36686->avg[0],
		cm36686->avg[1], cm36686->avg[2]);
}

static ssize_t proximity_avg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	bool new_value = false;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	SENSOR_INFO("average enable = %d\n", new_value);

	mutex_lock(&cm36686->power_lock);
	if (new_value) {
		if (!(cm36686->power_state & PROXIMITY_ENABLED))
			cm36686_i2c_write_word(cm36686, REG_PS_CONF1,
				ps_reg_init_setting[PS_CONF1][CMD]);
		hrtimer_start(&cm36686->prox_timer, cm36686->prox_poll_delay,
			HRTIMER_MODE_REL);
	} else if (!new_value) {
		hrtimer_cancel(&cm36686->prox_timer);
		cancel_work_sync(&cm36686->work_prox);
		if (!(cm36686->power_state & PROXIMITY_ENABLED))
			cm36686_i2c_write_word(cm36686, REG_PS_CONF1,
				0x0001);
	}
	mutex_unlock(&cm36686->power_lock);

	return size;
}

static ssize_t proximity_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	u16 ps_data;

	mutex_lock(&cm36686->read_lock);
	cm36686_i2c_read_word(cm36686, REG_PS_DATA, &ps_data);
	mutex_unlock(&cm36686->read_lock);
	return snprintf(buf, PAGE_SIZE, "%u\n", ps_data);
}

static ssize_t proximity_thresh_high_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	SENSOR_INFO("%u,%u\n",
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);
	return snprintf(buf, PAGE_SIZE, "%u,%u\n",
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);
}

static ssize_t proximity_thresh_high_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	u16 thresh_value = ps_reg_init_setting[PS_THD_HIGH][CMD];
	int err;

	err = kstrtou16(buf, 10, &thresh_value);
	if (err < 0)
		SENSOR_ERR("kstrtoint failed.");

	SENSOR_INFO("thresh_value:%u\n", thresh_value);
	if (thresh_value > 2) {
		ps_reg_init_setting[PS_THD_HIGH][CMD] = thresh_value;
		err = cm36686_i2c_write_word(cm36686, REG_PS_THD_HIGH,
			ps_reg_init_setting[PS_THD_HIGH][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36686_ps_high_reg is failed. %d\n", err);
		SENSOR_INFO("new high threshold = 0x%x\n", thresh_value);
		msleep(150);
	} else
		SENSOR_ERR("wrong high threshold value(0x%x)\n", thresh_value);

	return size;
}

static ssize_t proximity_thresh_low_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	SENSOR_INFO("%u,%u\n",
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);

	return snprintf(buf, PAGE_SIZE, "%u,%u\n",
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);
}

static ssize_t proximity_thresh_low_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	u16 thresh_value = ps_reg_init_setting[PS_THD_LOW][CMD];
	int err;

	err = kstrtou16(buf, 10, &thresh_value);
	if (err < 0)
		SENSOR_ERR("kstrtoint failed.");

	SENSOR_INFO("thresh_value:%u\n", thresh_value);
	if (thresh_value > 2) {
		ps_reg_init_setting[PS_THD_LOW][CMD] = thresh_value;
		err = cm36686_i2c_write_word(cm36686, REG_PS_THD_LOW,
			ps_reg_init_setting[PS_THD_LOW][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36686_ps_low_reg is failed. %d\n", err);
		SENSOR_INFO("new low threshold = 0x%x\n", thresh_value);
		msleep(150);
	} else
		SENSOR_ERR("wrong low threshold value(0x%x)\n", thresh_value);

	return size;
}

#if defined(PROXIMITY_FOR_TEST)
static ssize_t proximity_register_write_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int regist = 0, val = 0;
	struct cm36686_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%2x,%4x", &regist, &val) != 2) {
		SENSOR_ERR("The number of data are wrong\n");
		return -EINVAL;
	}

	cm36686_i2c_write_word(data, regist, val);
	SENSOR_INFO("Register(0x%2x) 16:data(0x%4x) 10:%d\n",
			regist, val, val);

	return count;
}

static ssize_t proximity_register_read_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u16 val[10], i;
	struct cm36686_data *data = dev_get_drvdata(dev);

	for (i = 0; i < 10; i++) {
		cm36686_i2c_read_word(data, i, &val[i]);
		SENSOR_INFO("Register(0x%2x) data(0x%4x)\n", i, val[i]);
	}

	return snprintf(buf, PAGE_SIZE,
		"0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		val[0], val[1], val[2], val[3], val[4],
		val[5], val[6], val[7], val[8], val[9]);
}
#endif

static ssize_t proximity_dhr_sensor_info_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);
	u16 cs_conf1;
	u16 ps_conf1, ps_conf3;
	u8 als_it;
	u8 ps_period, ps_pers, ps_it;
	u8 ps_smart_pers, ps_led_current;
	u16 ps_canc;
	u16 ps_low_thresh, ps_hi_thresh;

	mutex_lock(&cm36686->read_lock);
	cm36686_i2c_read_word(cm36686, REG_CS_CONF1, &cs_conf1);
	cm36686_i2c_read_word(cm36686, REG_PS_CONF1, &ps_conf1);
	cm36686_i2c_read_word(cm36686, REG_PS_CONF3, &ps_conf3);
	cm36686_i2c_read_word(cm36686, REG_PS_CANC, &ps_canc);
	cm36686_i2c_read_word(cm36686, REG_PS_THD_LOW, &ps_low_thresh);
	cm36686_i2c_read_word(cm36686, REG_PS_THD_HIGH, &ps_hi_thresh);
	mutex_unlock(&cm36686->read_lock);

	als_it = (cs_conf1 & 0x0c) >> 2;
	ps_period = (ps_conf1 & 0xc0) >> 6;
	ps_pers = (ps_conf1 & 0x30) >> 4;
	ps_it = (ps_conf1 & 0xc000) >> 14;
	ps_smart_pers = (ps_conf1 & 0x02) >> 1;
	ps_led_current = (ps_conf3 & 0x0700) >> 8;

	return snprintf(buf, PAGE_SIZE,
			"\"THD\":\"%u %u\", \"ALS_IT\":\"0x%x\", \"PS_PERIOD\":\"0x%x\", \"PS_PERS\":\"0x%x\", \"PS_IT\":\"0x%x\", \"PS_SMART_PERS\":\"0x%x\", \"PS_LED_CURRENT\":\"0x%x\", \"PS_CANC\":\"0x%x\"\n",
			ps_low_thresh, ps_hi_thresh,
			als_it, ps_period, ps_pers, ps_it,
			ps_smart_pers, ps_led_current, ps_canc);
}

static DEVICE_ATTR(prox_cal, 0664,
	proximity_cancel_show, proximity_cancel_store);
static DEVICE_ATTR(prox_offset_pass, 0444, proximity_cancel_pass_show,
	NULL);
static DEVICE_ATTR(prox_avg, 0664,
	proximity_avg_show, proximity_avg_store);
static DEVICE_ATTR(state, 0444, proximity_state_show, NULL);
static struct device_attribute dev_attr_prox_raw = __ATTR(raw_data,
	0444, proximity_state_show, NULL);
static DEVICE_ATTR(thresh_high, 0664,
	proximity_thresh_high_show, proximity_thresh_high_store);
static DEVICE_ATTR(thresh_low, 0664,
	proximity_thresh_low_show, proximity_thresh_low_store);
#if defined(PROXIMITY_FOR_TEST)
static DEVICE_ATTR(prox_trim, 0664,
	proximity_trim_show, proximity_trim_store);
static DEVICE_ATTR(prox_register, 0664,
	proximity_register_read_show, proximity_register_write_store);
#else
static DEVICE_ATTR(prox_trim, 0440,
	proximity_trim_show, NULL);
#endif
static DEVICE_ATTR(dhr_sensor_info, 0440,
	proximity_dhr_sensor_info_show, NULL);

static struct device_attribute *prox_sensor_attrs[] = {
	&dev_attr_prox_sensor_vendor,
	&dev_attr_prox_sensor_name,
	&dev_attr_prox_avg,
	&dev_attr_state,
	&dev_attr_thresh_high,
	&dev_attr_thresh_low,
	&dev_attr_prox_raw,
	&dev_attr_prox_trim,
#if defined(PROXIMITY_FOR_TEST)
	&dev_attr_prox_register,
#endif
	&dev_attr_prox_cal,
	&dev_attr_prox_offset_pass,
	&dev_attr_dhr_sensor_info,
	NULL,
};

/* light sensor sysfs */
static ssize_t light_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u,%u\n", cm36686->als_data,
		cm36686->white_data);
}

static ssize_t light_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u,%u\n", cm36686->als_data,
		cm36686->white_data);
}

static DEVICE_ATTR(lux, 0444, light_lux_show, NULL);
static DEVICE_ATTR(raw_data, 0444, light_data_show, NULL);

static struct device_attribute *light_sensor_attrs[] = {
	&dev_attr_light_sensor_vendor,
	&dev_attr_light_sensor_name,
	&dev_attr_lux,
	&dev_attr_raw_data,
	NULL,
};

/* interrupt happened due to transition/change of near/far proximity state */
irqreturn_t cm36686_irq_thread_fn(int irq, void *data)
{
	struct cm36686_data *cm36686 = data;
	u8 val;
	u16 ps_data = 0;

	val = gpio_get_value(cm36686->irq_gpio);
	cm36686_i2c_read_word(cm36686, REG_PS_DATA, &ps_data);

	if (cm36686->power_state & PROXIMITY_ENABLED) {
#ifdef CONFIG_SEC_FACTORY
		SENSOR_INFO("FACTORY: near/far=%d, ps data = %d (close:0, far:1)\n",
				val, ps_data);
#else
		SENSOR_INFO("near/far=%d, ps data = %d (close:0, far:1)\n",
				val, ps_data);
		if (((!val) && (ps_data >= cm36686->default_hi_thd)) ||
			(val && (ps_data <= cm36686->default_low_thd)))
#endif
		{
			/* 0 is close, 1 is far */
			input_report_abs(cm36686->proximity_input_dev, ABS_DISTANCE,
				val);
			input_sync(cm36686->proximity_input_dev);
		}
	}
	wake_lock_timeout(&cm36686->prox_wake_lock, 3 * HZ);

	return IRQ_HANDLED;
}

/* This function is for light sensor. It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart cm36686_light_timer_func(struct hrtimer *timer)
{
	struct cm36686_data *cm36686
		= container_of(timer, struct cm36686_data, light_timer);

	queue_work(cm36686->light_wq, &cm36686->work_light);
	hrtimer_forward_now(&cm36686->light_timer, cm36686->light_poll_delay);
	return HRTIMER_RESTART;
}

static void cm36686_work_func_light(struct work_struct *work)
{
	struct cm36686_data *cm36686 = container_of(work, struct cm36686_data,
						work_light);
	struct timespec ts = ktime_to_timespec(ktime_get_boottime());
	u64 timestamp = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
	int time_hi = (int)((timestamp & TIME_HI_MASK) >> TIME_HI_SHIFT);
	int time_lo = (int)(timestamp & TIME_LO_MASK);

	if (!(cm36686->power_state & LIGHT_ENABLED)) {
		SENSOR_INFO("light disabled.\n");
		return;
	}

	mutex_lock(&cm36686->read_lock);
	cm36686_i2c_read_word(cm36686, REG_ALS_DATA, &cm36686->als_data);
	cm36686_i2c_read_word(cm36686, REG_WHITE_DATA, &cm36686->white_data);
	mutex_unlock(&cm36686->read_lock);

	input_report_rel(cm36686->light_input_dev, REL_DIAL,
		cm36686->als_data + 1);
	input_report_rel(cm36686->light_input_dev, REL_WHEEL,
		cm36686->white_data + 1);
	input_report_rel(cm36686->light_input_dev, REL_X, time_hi);
	input_report_rel(cm36686->light_input_dev, REL_Y, time_lo);
	input_sync(cm36686->light_input_dev);

	if (cm36686->count_log_time >= LIGHT_LOG_TIME) {
		SENSOR_INFO("%u,%u\n", cm36686->als_data, cm36686->white_data);
		cm36686->count_log_time = 0;
	} else
		cm36686->count_log_time++;
}

static void cm36686_get_avg_val(struct cm36686_data *cm36686)
{
	int min = 0, max = 0, avg = 0;
	int i;
	u16 ps_data = 0;

	for (i = 0; i < PROX_READ_NUM; i++) {
		msleep(40);
		cm36686_i2c_read_word(cm36686, REG_PS_DATA, &ps_data);
		avg += ps_data;

		if (!i)
			min = ps_data;
		else if (ps_data < min)
			min = ps_data;

		if (ps_data > max)
			max = ps_data;
	}
	avg /= PROX_READ_NUM;

	cm36686->avg[0] = min;
	cm36686->avg[1] = avg;
	cm36686->avg[2] = max;
}

static void cm36686_work_func_prox(struct work_struct *work)
{
	struct cm36686_data *cm36686 = container_of(work, struct cm36686_data,
						  work_prox);

	cm36686_get_avg_val(cm36686);
}

static enum hrtimer_restart cm36686_prox_timer_func(struct hrtimer *timer)
{
	struct cm36686_data *cm36686
			= container_of(timer, struct cm36686_data, prox_timer);

	queue_work(cm36686->prox_wq, &cm36686->work_prox);
	hrtimer_forward_now(&cm36686->prox_timer, cm36686->prox_poll_delay);
	return HRTIMER_RESTART;
}

static int sensor_vdd_onoff(struct device *dev, bool onoff)
{
	struct cm36686_data *data = dev_get_drvdata(dev);
	int ret;

	SENSOR_INFO("%s\n", (onoff) ? "on" : "off");

	if (!data->vdd) {
		SENSOR_INFO("VDD get regulator\n");
		data->vdd = devm_regulator_get(dev, "cm36686,vdd");
		if (IS_ERR(data->vdd)) {
			SENSOR_ERR("cannot get vdd\n");
			data->vdd = NULL;
			return -ENOMEM;
		}

		if (!regulator_get_voltage(data->vdd))
			regulator_set_voltage(data->vdd, 2850000, 2850000);
	}

	if (onoff) {
		if (regulator_is_enabled(data->vdd)) {
			SENSOR_INFO("Regulator already enabled\n");
			return 0;
		}

		ret = regulator_enable(data->vdd);
		if (ret)
			SENSOR_ERR("Failed to enable vdd.\n");
		usleep_range(10000, 11000);
	} else {
		ret = regulator_disable(data->vdd);
		if (ret)
			SENSOR_ERR("Failed to disable vdd.\n");
	}

	SENSOR_INFO("end\n");
	return 0;
}

static int proximity_vled_onoff(struct device *dev, bool onoff)
{
	struct cm36686_data *data = dev_get_drvdata(dev);
	int ret;

	SENSOR_INFO("%s, ldo:%d\n",
		(onoff) ? "on" : "off", data->vled_ldo);

	/* ldo control */
	if (data->vled_ldo) {
		gpio_set_value(data->vled_ldo, onoff);
		return 0;
	}

	/* regulator control */
	if (!data->vled) {
		SENSOR_INFO("VLED get regulator\n");
		data->vled = devm_regulator_get(dev, "cm36686,vled");
		if (IS_ERR(data->vled)) {
			SENSOR_ERR("cannot get vled\n");
			data->vled = NULL;
			return -ENOMEM;
		}
	}

	if (onoff) {
		if (regulator_is_enabled(data->vled)) {
			SENSOR_INFO("Regulator already enabled\n");
			return 0;
		}

		ret = regulator_enable(data->vled);
		if (ret)
			SENSOR_ERR("Failed to enable vled.\n");
		usleep_range(10000, 11000);
	} else {
		ret = regulator_disable(data->vled);
		if (ret)
			SENSOR_ERR("Failed to disable vled.\n");
	}

	return 0;
}

static int cm36686_setup_reg(struct cm36686_data *cm36686)
{
	int err, i;
	u16 tmp = 0;

	/* ALS initialization */
	err = cm36686_i2c_write_word(cm36686,
			als_reg_setting[0][0],
			als_reg_setting[0][1]);
	if (err < 0) {
		SENSOR_ERR("cm36686_als_reg is failed. %d\n", err);
		return err;
	}
	/* PS initialization */
	for (i = 0; i < PS_REG_NUM; i++) {
		err = cm36686_i2c_write_word(cm36686,
			ps_reg_init_setting[i][REG_ADDR],
			ps_reg_init_setting[i][CMD]);
		if (err < 0) {
			SENSOR_ERR("cm36686_ps_reg is failed. %d\n", err);
			return err;
		}
	}

	/* printing the inital proximity value with no contact */
	msleep(50);
	mutex_lock(&cm36686->read_lock);
	err = cm36686_i2c_read_word(cm36686, REG_PS_DATA, &tmp);
	mutex_unlock(&cm36686->read_lock);
	if (err < 0) {
		SENSOR_ERR("read ps_data failed\n");
		err = -EIO;
	}
	SENSOR_INFO("initial proximity value = %d\n", tmp);

	/* turn off */
	cm36686_i2c_write_word(cm36686, REG_CS_CONF1, 0x0001);
	cm36686_i2c_write_word(cm36686, REG_PS_CONF1, 0x0001);
	cm36686_i2c_write_word(cm36686, REG_PS_CONF3, 0x0000);

	SENSOR_INFO("is success.");
	return err;
}

static int cm36686_setup_irq(struct cm36686_data *cm36686)
{
	int rc;

	rc = gpio_request(cm36686->irq_gpio, "gpio_proximity_out");
	if (rc < 0) {
		SENSOR_ERR("gpio %d request failed (%d)\n", cm36686->irq_gpio, rc);
		return rc;
	}

	rc = gpio_direction_input(cm36686->irq_gpio);
	if (rc < 0) {
		SENSOR_ERR("failed to set gpio %d as input (%d)\n",
			cm36686->irq_gpio, rc);
		gpio_free(cm36686->irq_gpio);
		return rc;
	}

	cm36686->irq = gpio_to_irq(cm36686->irq_gpio);
	/* add IRQF_NO_SUSPEND option in case of Spreadtrum AP */
	rc = request_threaded_irq(cm36686->irq, NULL, cm36686_irq_thread_fn,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"proximity_int", cm36686);
	if (rc < 0) {
		SENSOR_ERR("irq:%d failed for qpio:%d err:%d\n",
			cm36686->irq, cm36686->irq_gpio, rc);
		gpio_free(cm36686->irq_gpio);
		return rc;
	}

	/* start with interrupts disabled */
	disable_irq(cm36686->irq);

	SENSOR_ERR("success\n");
	return rc;
}

#ifdef CONFIG_OF
/* device tree parsing function */
static int cm36686_parse_dt(struct device *dev,
	struct cm36686_data *cm36686)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int ret;

	cm36686->irq_gpio = of_get_named_gpio_flags(np, "cm36686,irq_gpio", 0, &flags);
	if (cm36686->irq_gpio < 0) {
		SENSOR_ERR("get prox_int error\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "cm36686,vled_same_vdd",
		&cm36686->vled_same_vdd);
	if ((ret < 0) || (!cm36686->vled_same_vdd)) {
		SENSOR_ERR("vled is controled\n");
		cm36686->vled_same_vdd = 0;

		cm36686->vled_ldo = of_get_named_gpio_flags(np, "cm36686,vled_ldo",
			0, &flags);
		if (cm36686->vled_ldo < 0) {
			SENSOR_ERR("fail to get vled_ldo but using regulator\n");
			cm36686->vled_ldo = 0;
		} else {
			ret = gpio_request(cm36686->vled_ldo, "prox_vled_en");
			if (ret < 0) {
				SENSOR_ERR("gpio %d request failed (%d)\n",
					cm36686->vled_ldo, ret);
				return ret;
			}
			gpio_direction_output(cm36686->vled_ldo, 0);
		}
	} else
		SENSOR_ERR("vled & vdd is same regulator\n");

	ret = of_property_read_u32(np, "cm36686,vdd_always_on",
		&cm36686->vdd_always_on);
	if (ret < 0) {
		SENSOR_ERR("vdd is controled\n");
		cm36686->vdd_always_on = 0;
	}

	ret = of_property_read_u32(np, "cm36686,ps_conf1",
		&cm36686->ps_conf1);
	if (ret < 0) {
		SENSOR_ERR("Cannot set ps_conf1 through DTSI\n");
		cm36686->ps_conf1 = DEFAULT_CONF1;
	}

	ret = of_property_read_u32(np, "cm36686,ps_conf3",
		&cm36686->ps_conf3);
	if (ret < 0) {
		SENSOR_ERR("Cannot set ps_conf3 through DTSI\n");
		cm36686->ps_conf3 = DEFAULT_CONF3;
	}

	ret = of_property_read_u32(np, "cm36686,default_hi_thd",
		&cm36686->default_hi_thd);
	if (ret < 0) {
		SENSOR_ERR("Cannot set default_hi_thd through DTSI\n");
		cm36686->default_hi_thd = DEFAULT_HI_THD;
	}

	ret = of_property_read_u32(np, "cm36686,default_low_thd",
		&cm36686->default_low_thd);
	if (ret < 0) {
		SENSOR_ERR("Cannot set default_low_thd through DTSI\n");
		cm36686->default_low_thd = DEFAULT_LOW_THD;
	}

	ret = of_property_read_u32(np, "cm36686,cancel_hi_thd",
		&cm36686->cancel_hi_thd);
	if (ret < 0) {
		SENSOR_ERR("Cannot set cancel_hi_thd through DTSI\n");
		cm36686->cancel_hi_thd = CANCEL_HI_THD;
	}

	ret = of_property_read_u32(np, "cm36686,cancel_low_thd",
		&cm36686->cancel_low_thd);
	if (ret < 0) {
		SENSOR_ERR("Cannot set cancel_low_thd through DTSI\n");
		cm36686->cancel_low_thd = CANCEL_LOW_THD;
	}

	ret = of_property_read_u32(np, "cm36686,cal_skip_adc",
		&cm36686->cal_skip_adc);
	if (ret < 0) {
		SENSOR_ERR("Cannot set cal_skip_adc through DTSI\n");
		cm36686->cal_skip_adc = CAL_SKIP_ADC;
	}

	ret = of_property_read_u32(np, "cm36686,cal_fail_adc",
		&cm36686->cal_fail_adc);
	if (ret < 0) {
		SENSOR_ERR("Cannot set cal_fail_adc through DTSI\n");
		cm36686->cal_fail_adc = CAL_FAIL_ADC;
	}

	ret = of_property_read_u32(np, "cm36686,default_trim",
			&cm36686->default_trim);
	if (ret < 0) {
		SENSOR_ERR("Cannot set default_trim\n");
		cm36686->default_trim = DEFAULT_TRIM;
	}

	ps_reg_init_setting[PS_CONF1][CMD] = cm36686->ps_conf1;
	ps_reg_init_setting[PS_CONF3][CMD] = cm36686->ps_conf3;
	ps_reg_init_setting[PS_THD_LOW][CMD] = cm36686->default_low_thd;
	ps_reg_init_setting[PS_THD_HIGH][CMD] = cm36686->default_hi_thd;
	ps_reg_init_setting[PS_CANCEL][CMD] = cm36686->default_trim;

	return 0;
}
#else
static int cm36686_parse_dt(struct device *dev, struct cm36686_data cm36686)
{
	return -ENODEV;
}
#endif

static int cm36686_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	struct cm36686_data *cm36686 = NULL;

	SENSOR_INFO("start\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_ERR("i2c functionality check failed!\n");
		return -ENODEV;
	}

	cm36686 = kzalloc(sizeof(struct cm36686_data), GFP_KERNEL);
	if (!cm36686) {
		SENSOR_ERR("failed to alloc memory for module data\n");
		return -ENOMEM;
	}

	ret = cm36686_parse_dt(&client->dev, cm36686);
	if (ret) {
		SENSOR_ERR("error in device tree");
		goto err_devicetree;
	}

	cm36686->i2c_client = client;
	i2c_set_clientdata(client, cm36686);

	mutex_init(&cm36686->power_lock);
	mutex_init(&cm36686->read_lock);

	/* wake lock init for proximity sensor */
	wake_lock_init(&cm36686->prox_wake_lock, WAKE_LOCK_SUSPEND,
			"prox_wake_lock");

	sensor_vdd_onoff(&client->dev, ON);

	if (!cm36686->vled_same_vdd)
		proximity_vled_onoff(&client->dev, ON);

	/* Check if the device is there or not. */
	ret = cm36686_i2c_write_word(cm36686, REG_CS_CONF1, 0x0001);
	if (ret < 0) {
		SENSOR_ERR("cm36686 is not connected.(%d)\n", ret);
		goto err_setup_reg;
	}

	/* setup initial registers */
	ret = cm36686_setup_reg(cm36686);
	if (ret < 0) {
		SENSOR_ERR("could not setup regs\n");
		goto err_setup_reg;
	}

	/* allocate proximity input_device */
	cm36686->proximity_input_dev = input_allocate_device();
	if (!cm36686->proximity_input_dev) {
		SENSOR_ERR("could not allocate proximity input device\n");
		goto err_input_allocate_device_proximity;
	}

	input_set_drvdata(cm36686->proximity_input_dev, cm36686);
	cm36686->proximity_input_dev->name = MODULE_NAME_PROX;
	input_set_capability(cm36686->proximity_input_dev, EV_ABS,
		ABS_DISTANCE);
	input_set_abs_params(cm36686->proximity_input_dev, ABS_DISTANCE,
		0, 1, 0, 0);

	ret = input_register_device(cm36686->proximity_input_dev);
	if (ret < 0) {
		input_free_device(cm36686->proximity_input_dev);
		SENSOR_ERR("could not register input device\n");
		goto err_input_register_device_proximity;
	}

	ret = sensors_create_symlink(&cm36686->proximity_input_dev->dev.kobj,
					cm36686->proximity_input_dev->name);
	if (ret < 0) {
		SENSOR_ERR("create_symlink error\n");
		goto err_sensors_create_symlink_prox;
	}

	ret = sysfs_create_group(&cm36686->proximity_input_dev->dev.kobj,
		&proximity_attribute_group);
	if (ret) {
		SENSOR_ERR("could not create sysfs group\n");
		goto err_sysfs_create_group_proximity;
	}

	/* setup irq */
	ret = cm36686_setup_irq(cm36686);
	if (ret) {
		SENSOR_ERR("could not setup irq\n");
		goto err_setup_irq;
	}

	/* For factory test mode, we use timer to get average proximity data. */
	/* prox_timer settings. we poll for light values using a timer. */
	hrtimer_init(&cm36686->prox_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cm36686->prox_poll_delay = ns_to_ktime(2000 * NSEC_PER_MSEC);/*2 sec*/
	cm36686->prox_timer.function = cm36686_prox_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	cm36686->prox_wq = create_singlethread_workqueue("cm36686_prox_wq");
	if (!cm36686->prox_wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create prox workqueue\n");
		goto err_create_prox_workqueue;
	}
	/* this is the thread function we run on the work queue */
	INIT_WORK(&cm36686->work_prox, cm36686_work_func_prox);

	/* allocate lightsensor input_device */
	cm36686->light_input_dev = input_allocate_device();
	if (!cm36686->light_input_dev) {
		SENSOR_ERR("could not allocate light input device\n");
		goto err_input_allocate_device_light;
	}

	input_set_drvdata(cm36686->light_input_dev, cm36686);
	cm36686->light_input_dev->name = MODULE_NAME_LIGHT;
	input_set_capability(cm36686->light_input_dev, EV_REL, REL_DIAL);
	input_set_capability(cm36686->light_input_dev, EV_REL, REL_WHEEL);
	input_set_capability(cm36686->light_input_dev, EV_REL, REL_X);
	input_set_capability(cm36686->light_input_dev, EV_REL, REL_Y);

	ret = input_register_device(cm36686->light_input_dev);
	if (ret < 0) {
		input_free_device(cm36686->light_input_dev);
		SENSOR_ERR("could not register input device\n");
		goto err_input_register_device_light;
	}

	ret = sensors_create_symlink(&cm36686->light_input_dev->dev.kobj,
					cm36686->light_input_dev->name);
	if (ret < 0) {
		SENSOR_ERR("create_symlink error\n");
		goto err_sensors_create_symlink_light;
	}

	ret = sysfs_create_group(&cm36686->light_input_dev->dev.kobj,
				 &light_attribute_group);
	if (ret) {
		SENSOR_ERR("could not create sysfs group\n");
		goto err_sysfs_create_group_light;
	}

	/* light_timer settings. we poll for light values using a timer. */
	hrtimer_init(&cm36686->light_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	cm36686->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	cm36686->light_timer.function = cm36686_light_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	   to read the i2c (can be slow and blocking). */
	cm36686->light_wq = create_singlethread_workqueue("cm36686_light_wq");
	if (!cm36686->light_wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create light workqueue\n");
		goto err_create_light_workqueue;
	}

	/* this is the thread function we run on the work queue */
	INIT_WORK(&cm36686->work_light, cm36686_work_func_light);

	/* set sysfs for proximity sensor */
	ret = sensors_register(&cm36686->proximity_dev,
		cm36686, prox_sensor_attrs, MODULE_NAME_PROX);
	if (ret) {
		SENSOR_ERR("can't register prox sensor device(%d)\n", ret);
		goto prox_sensor_register_failed;
	}

	/* set sysfs for light sensor */
	ret = sensors_register(&cm36686->light_dev,
		cm36686, light_sensor_attrs, MODULE_NAME_LIGHT);
	if (ret) {
		SENSOR_ERR("can't register light sensor device(%d)\n", ret);
		goto light_sensor_register_failed;
	}

	if (!cm36686->vled_same_vdd)
		proximity_vled_onoff(&client->dev, OFF);

	SENSOR_INFO("success\n");
	return ret;

/* error, unwind it all */
light_sensor_register_failed:
	sensors_unregister(cm36686->proximity_dev, prox_sensor_attrs);
prox_sensor_register_failed:
	destroy_workqueue(cm36686->light_wq);
err_create_light_workqueue:
	sysfs_remove_group(&cm36686->light_input_dev->dev.kobj,
			   &light_attribute_group);
err_sysfs_create_group_light:
	sensors_remove_symlink(&cm36686->light_input_dev->dev.kobj,
			cm36686->light_input_dev->name);
err_sensors_create_symlink_light:
	input_unregister_device(cm36686->light_input_dev);
err_input_register_device_light:
err_input_allocate_device_light:
	destroy_workqueue(cm36686->prox_wq);
err_create_prox_workqueue:
	free_irq(cm36686->irq, cm36686);
	gpio_free(cm36686->irq_gpio);
err_setup_irq:
	sysfs_remove_group(&cm36686->proximity_input_dev->dev.kobj,
		&proximity_attribute_group);
err_sysfs_create_group_proximity:
	sensors_remove_symlink(&cm36686->proximity_input_dev->dev.kobj,
			cm36686->proximity_input_dev->name);
err_sensors_create_symlink_prox:
	input_unregister_device(cm36686->proximity_input_dev);
err_input_register_device_proximity:
err_input_allocate_device_proximity:
err_setup_reg:
	proximity_vled_onoff(&client->dev, OFF);
	if (cm36686->vled_ldo)
		gpio_free(cm36686->vled_ldo);
	sensor_vdd_onoff(&client->dev, OFF);
	wake_lock_destroy(&cm36686->prox_wake_lock);
	mutex_destroy(&cm36686->read_lock);
	mutex_destroy(&cm36686->power_lock);
err_devicetree:
	kfree(cm36686);

	SENSOR_ERR("failed (%d)\n", ret);
	return ret;
}

static int cm36686_suspend(struct device *dev)
{
	/* We disable power only if proximity is disabled.  If proximity
	   is enabled, we leave power on because proximity is allowed
	   to wake up device.  We remove power without changing
	   cm36686->power_state because we use that state in resume.
	 */
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	if (cm36686->power_state & LIGHT_ENABLED)
		cm36686_light_disable(cm36686);
	SENSOR_INFO("is called.\n");

	return 0;
}

static int cm36686_resume(struct device *dev)
{
	struct cm36686_data *cm36686 = dev_get_drvdata(dev);

	if (cm36686->power_state & LIGHT_ENABLED)
		cm36686_light_enable(cm36686);
	SENSOR_INFO("is called.\n");

	return 0;
}

static int cm36686_i2c_remove(struct i2c_client *client)
{
	SENSOR_INFO("\n");
	return 0;
}

static void cm36686_i2c_shutdown(struct i2c_client *client)
{
	struct cm36686_data *cm36686 = i2c_get_clientdata(client);

	if (cm36686->power_state & LIGHT_ENABLED)
		cm36686_light_disable(cm36686);

	if (cm36686->power_state & PROXIMITY_ENABLED) {
		disable_irq_wake(cm36686->irq);
		disable_irq(cm36686->irq);
		cm36686_i2c_write_word(cm36686, REG_PS_CONF1, 0x0001);
	}

	SENSOR_INFO("done\n");
}

#ifdef CONFIG_OF
static struct of_device_id cm36686_match_table[] = {
	{ .compatible = "cm36686",},
	{},
};
#else
#define cm36686_match_table NULL
#endif


static const struct i2c_device_id cm36686_device_id[] = {
	{"cm36686", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cm36686_device_id);

static const struct dev_pm_ops cm36686_pm_ops = {
	.suspend = cm36686_suspend,
	.resume = cm36686_resume
};

static struct i2c_driver cm36686_i2c_driver = {
	.driver = {
		   .name = "cm36686",
		   .owner = THIS_MODULE,
		   .of_match_table = cm36686_match_table,
		   .pm = &cm36686_pm_ops
	},
	.probe = cm36686_i2c_probe,
	.shutdown  = cm36686_i2c_shutdown,
	.remove = cm36686_i2c_remove,
	.id_table = cm36686_device_id,
};

static int __init cm36686_init(void)
{
	return i2c_add_driver(&cm36686_i2c_driver);
}

static void __exit cm36686_exit(void)
{
	i2c_del_driver(&cm36686_i2c_driver);
}

module_init(cm36686_init);
module_exit(cm36686_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("RGB Sensor device driver for cm36686");
MODULE_LICENSE("GPL");
