/* driver/sensor/cm36672p.c
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <linux/sensor/sensors_core.h>

#define PROXIMITY_FOR_TEST	/* for HW to tune up */

#define MODULE_NAME      "proximity_sensor"
#define VENDOR          "CAPELLA"
#define CHIP_ID         "CM36672P"

#define I2C_M_WR        0 /* for i2c Write */

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

/* proximity sensor regisiter addresses */
#define REG_PS_CONF1    0x03
#define REG_PS_CONF3    0x04
#define REG_PS_CANC     0x05
#define REG_PS_THD_LOW  0x06
#define REG_PS_THD_HIGH 0x07
#define REG_PS_DATA     0x08

/* proximity sensor default value for register */
#define DEFAULT_HI_THD  0x0011
#define DEFAULT_LOW_THD 0x000d
#define CANCEL_HI_THD   0x000a
#define CANCEL_LOW_THD  0x0007

#define DEFAULT_CONF1   0x0320 /* PS_INT = (1:1), PS_PERS = (1:0) */
#if defined(CONFIG_SENSORS_CM36672P_SMART_PERS)
#define DEFAULT_CONF3   0x4010 /* PS_MS = 1, PS_SMART_PERS = 1 */
#else
#define DEFAULT_CONF3   0x4000 /* PS_MS = 1, PS_SMART_PERS = 0 */
#endif
#define DEFAULT_TRIM    0x0000

/*
 * NOTE:
 * Since PS Duty, PS integration time and LED current
 * would be different by HW rev or Project,
 * we move the setting value to device tree.
 * Please refer to the value below.
 *
 * PS_DUTY (CONF1, 0x03_L)
 * 1/40 = 0, 1/80 = 1, 1/160 = 2, 1/320 = 3
 *
 * PS_IT (CONF1, 0x03_L)
 * 1T = 0, 1.5T = 1, 2T = 2, 2.5T = 3, 3T = 4, 3.5T = 5, 4T = 6, 8T = 7
 *
 * LED_I (CONF3, 0x04_H)
 * 50mA = 0, 75mA = 1, 100mA = 2, 120mA = 3,
 * 140mA = 4, 160mA = 5, 180mA = 6, 200mA = 7
 */

static u16 ps_reg_init_setting[PS_REG_NUM][2] = {
	{REG_PS_CONF1, DEFAULT_CONF1}, /* REG_PS_CONF1 */
	{REG_PS_CONF3, DEFAULT_CONF3}, /* REG_PS_CONF3 */
	{REG_PS_THD_LOW, DEFAULT_LOW_THD}, /* REG_PS_THD_LOW */
	{REG_PS_THD_HIGH, DEFAULT_HI_THD}, /* REG_PS_THD_HIGH */
	{REG_PS_CANC, DEFAULT_TRIM}, /* REG_PS_CANC */
};

/* Intelligent Cancellation*/
#define CM36672P_CANCELLATION
#ifdef CM36672P_CANCELLATION
#define CANCELLATION_FILE_PATH   "/efs/FactoryApp/prox_cal"
#define CAL_SKIP_ADC    8 /* nondetect threshold *60% */
#define CAL_FAIL_ADC    20 /* detect threshold */
enum {
	CAL_FAIL = 0,
	CAL_CANCELLATION,
	CAL_SKIP,
};
#endif

#define PROX_READ_NUM   40

enum {
	OFF = 0,
	ON,
};

/* driver data */
struct cm36672p_data {
	struct i2c_client *i2c_client;
	struct wake_lock prox_wake_lock;
	struct input_dev *proximity_input_dev;
	struct mutex read_lock;
	struct hrtimer prox_timer;
	struct workqueue_struct *prox_wq;
	struct work_struct work_prox;
	struct device *proximity_dev;
	struct regulator *vdd;
	struct regulator *vled;

	ktime_t prox_poll_delay;
	atomic_t enable;

	int avg[3];
	unsigned int prox_cal_result;


	int default_hi_thd;
	int default_low_thd;
	int cancel_hi_thd;
	int cancel_low_thd;
	int offset_range_hi;
	int offset_range_low;
	int default_trim;

	int irq;
	int irq_gpio;		/* proximity-sensor irq gpio */

	int vled_ldo;
	int vdd_ldo;

	int vdd_always_on; /* 1: vdd is always on, 0: enable only when proximity is on */
	int regulator_divided; /* 1: regulator divided, 0: regulator not divided */
};

static int proximity_vdd_onoff(struct device *dev, bool onoff);
static int proximity_vled_onoff(struct device *dev, bool onoff);

int cm36672p_i2c_read_word(struct cm36672p_data *data, u8 command, u16 *val)
{
	int err = 0;
	int retry = 3;
	struct i2c_client *client = data->i2c_client;
	struct i2c_msg msg[2];
	unsigned char tmp[2] = {0,};
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
		msg[1].buf = tmp;

		err = i2c_transfer(client->adapter, msg, 2);

		if (err >= 0) {
			value = (u16)tmp[1];
			*val = (value << 8) | (u16)tmp[0];
			return err;
		}
	}
	SENSOR_ERR("i2c transfer error ret=%d\n", err);
	return err;
}

int cm36672p_i2c_write_word(struct cm36672p_data *data, u8 command,
	u16 val)
{
	int err = 0;
	struct i2c_client *client = data->i2c_client;
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

#ifdef CM36672P_CANCELLATION
static int proximity_open_cancellation(struct cm36672p_data *data)
{
	struct file *cancel_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cancel_filp = filp_open(CANCELLATION_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(cancel_filp)) {
		err = PTR_ERR(cancel_filp);
		if (err != -ENOENT)
			SENSOR_ERR("Can't open cancellation file(%d)\n", err);
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

static int proximity_store_cancellation(struct device *dev, bool do_calib)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);
	struct file *cancel_filp = NULL;
	mm_segment_t old_fs;
	int err;
	u16 ps_data = 0;

	if (do_calib) {
		mutex_lock(&data->read_lock);
		cm36672p_i2c_read_word(data, REG_PS_DATA, &ps_data);
		mutex_unlock(&data->read_lock);

		if (ps_data  < data->offset_range_low) {
			/* SKIP. CAL_SKIP_ADC */
			ps_reg_init_setting[PS_CANCEL][CMD] =
				data->default_trim;
			SENSOR_INFO("crosstalk < %d/100\n",
				(data->default_low_thd * 50));
			data->prox_cal_result = CAL_SKIP;
		} else if (ps_data <= data->offset_range_hi) {
			/* CANCELLATION */
			ps_reg_init_setting[PS_CANCEL][CMD] =
				data->default_trim + ps_data;
			SENSOR_INFO("crosstalk_offset = %u", ps_data);
			data->prox_cal_result = CAL_CANCELLATION;
		} else {
			/*FAIL*/
			ps_reg_init_setting[PS_CANCEL][CMD] =
				data->default_trim;
			SENSOR_INFO("crosstalk > %d\n",
				data->default_hi_thd);
			data->prox_cal_result = CAL_FAIL;
		}

		if (data->prox_cal_result == CAL_CANCELLATION) {
			ps_reg_init_setting[PS_THD_HIGH][CMD] =
				data->cancel_hi_thd ?
				data->cancel_hi_thd :
				CANCEL_HI_THD;
			ps_reg_init_setting[PS_THD_LOW][CMD] =
				data->cancel_low_thd ?
				data->cancel_low_thd :
				CANCEL_LOW_THD;
		} else {
			ps_reg_init_setting[PS_THD_HIGH][CMD] =
				data->default_hi_thd ?
				data->default_hi_thd :
				DEFAULT_HI_THD;
			ps_reg_init_setting[PS_THD_LOW][CMD] =
				data->default_low_thd ?
				data->default_low_thd :
				DEFAULT_LOW_THD;
		}
	} else { /* reset */
		ps_reg_init_setting[PS_CANCEL][CMD] = data->default_trim;
		ps_reg_init_setting[PS_THD_HIGH][CMD] =
			data->default_hi_thd ?
			data->default_hi_thd :
			DEFAULT_HI_THD;
		ps_reg_init_setting[PS_THD_LOW][CMD] =
			data->default_low_thd ?
			data->default_low_thd :
			DEFAULT_LOW_THD;
	}

	if ((data->prox_cal_result == CAL_CANCELLATION) || !do_calib) {
		err = cm36672p_i2c_write_word(data, REG_PS_CANC,
			ps_reg_init_setting[PS_CANCEL][CMD]);
		if (err < 0)
			SENSOR_ERR("ps_canc_reg is failed. %d\n", err);
		err = cm36672p_i2c_write_word(data, REG_PS_THD_HIGH,
			ps_reg_init_setting[PS_THD_HIGH][CMD]);
		if (err < 0)
			SENSOR_ERR("ps_high_reg is failed. %d\n", err);
		err = cm36672p_i2c_write_word(data, REG_PS_THD_LOW,
			ps_reg_init_setting[PS_THD_LOW][CMD]);
		if (err < 0)
			SENSOR_ERR("ps_low_reg is failed. %d\n", err);
	}

	SENSOR_INFO("prox_cal = 0x%x, high_thd = 0x%x, low_thd = 0x%x\n",
		ps_reg_init_setting[PS_CANCEL][CMD],
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cancel_filp = filp_open(CANCELLATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY | O_SYNC, 0660);
	if (IS_ERR(cancel_filp)) {
		set_fs(old_fs);
		err = PTR_ERR(cancel_filp);
		SENSOR_ERR("Can't open cancellation file(%d)\n", err);
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

	if (sysfs_streq(buf, "1")) /* calibrate cancellation value */
		do_calib = true;
	else if (sysfs_streq(buf, "0")) /* reset cancellation value */
		do_calib = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	err = proximity_store_cancellation(dev, do_calib);
	if (err < 0) {
		SENSOR_ERR("proximity_store_cancellation() failed(%d)\n", err);
		return err;
	}

	return size;
}

static ssize_t proximity_cancel_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u,%u,%u\n",
		ps_reg_init_setting[PS_CANCEL][CMD] - data->default_trim,
		ps_reg_init_setting[PS_THD_HIGH][CMD],
		ps_reg_init_setting[PS_THD_LOW][CMD]);
}

static ssize_t proximity_cancel_pass_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);

	SENSOR_INFO("%u\n", data->prox_cal_result);
	return snprintf(buf, PAGE_SIZE, "%u\n", data->prox_cal_result);
}
#endif

static ssize_t proximity_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);
	bool new_value;
	int pre_enable;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return -EINVAL;
	}

	pre_enable = atomic_read(&data->enable);
	SENSOR_INFO("new_value = %d, pre_enable = %d\n",
		new_value, pre_enable);

	if (new_value && !pre_enable) {
		int i, ret;

		if (!data->vdd_always_on)
			proximity_vdd_onoff(dev, ON);
		if (!data->regulator_divided)
			proximity_vled_onoff(dev, ON);

		atomic_set(&data->enable, ON);
#ifdef CM36672P_CANCELLATION
		/* open cancellation data */
		ret = proximity_open_cancellation(data);
		if (ret < 0 && ret != -ENOENT)
			SENSOR_ERR("proximity_open_cancellation() failed\n");

#endif
		/* enable settings */
		for (i = 0; i < PS_REG_NUM; i++)
			cm36672p_i2c_write_word(data,
				ps_reg_init_setting[i][REG_ADDR],
				ps_reg_init_setting[i][CMD]);

		/* 0 is close, 1 is far */
		input_report_abs(data->proximity_input_dev, ABS_DISTANCE, 1);
		input_sync(data->proximity_input_dev);

		enable_irq(data->irq);
		enable_irq_wake(data->irq);
	} else if (!new_value && pre_enable) {
		disable_irq_wake(data->irq);
		disable_irq(data->irq);
		/* disable settings */
		cm36672p_i2c_write_word(data, REG_PS_CONF1, 0x0001);
		atomic_set(&data->enable, OFF);
		if (!data->regulator_divided)
			proximity_vled_onoff(dev, OFF);
		if (!data->vdd_always_on)
			proximity_vdd_onoff(dev, OFF);
	}
	SENSOR_INFO("enable = %d\n", atomic_read(&data->enable));

	return size;
}

static ssize_t proximity_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
		atomic_read(&data->enable));
}

static DEVICE_ATTR(enable, 0664,
	proximity_enable_show, proximity_enable_store);

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = {
	.attrs = proximity_sysfs_attrs,
};

/* sysfs for vendor & name */
static ssize_t cm36672p_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t cm36672p_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_ID);
}

static ssize_t proximity_trim_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->default_trim);
}

#if defined(PROXIMITY_FOR_TEST)
static ssize_t proximity_trim_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);
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
		err = cm36672p_i2c_write_word(data, REG_PS_CANC,
			ps_reg_init_setting[PS_CANCEL][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36672p_ps_canc is failed. %d\n", err);
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
	struct cm36672p_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", data->avg[0],
		data->avg[1], data->avg[2]);
}

static ssize_t proximity_avg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);
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
	if (new_value) {
		if (atomic_read(&data->enable) == OFF) {
			if (!data->vdd_always_on)
				proximity_vdd_onoff(dev, ON);
			if (!data->regulator_divided)
				proximity_vled_onoff(dev, ON);

			cm36672p_i2c_write_word(data, REG_PS_CONF1,
				ps_reg_init_setting[PS_CONF1][CMD]);
		}
		hrtimer_start(&data->prox_timer, data->prox_poll_delay,
			HRTIMER_MODE_REL);
	} else if (!new_value) {
		hrtimer_cancel(&data->prox_timer);
		cancel_work_sync(&data->work_prox);
		if (atomic_read(&data->enable) == OFF) {
			cm36672p_i2c_write_word(data, REG_PS_CONF1,
				0x0001);
			if (!data->regulator_divided)
				proximity_vled_onoff(dev, OFF);
			if (!data->vdd_always_on)
				proximity_vdd_onoff(dev, OFF);
		}
	}

	return size;
}

static ssize_t proximity_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);
	u16 ps_data;

	if (atomic_read(&data->enable) == OFF) {
		if (!data->vdd_always_on)
			proximity_vdd_onoff(dev, ON);
		if (!data->regulator_divided)
			proximity_vled_onoff(dev, ON);

		cm36672p_i2c_write_word(data, REG_PS_CONF1,
			ps_reg_init_setting[PS_CONF1][CMD]);
	}

	mutex_lock(&data->read_lock);
	cm36672p_i2c_read_word(data, REG_PS_DATA, &ps_data);
	mutex_unlock(&data->read_lock);

	if (atomic_read(&data->enable) == OFF) {
		cm36672p_i2c_write_word(data, REG_PS_CONF1, 0x0001);
		if (!data->regulator_divided)
			proximity_vled_onoff(dev, OFF);
		if (!data->vdd_always_on)
			proximity_vdd_onoff(dev, OFF);
	}

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
	struct cm36672p_data *data = dev_get_drvdata(dev);
	u16 thresh_value = ps_reg_init_setting[PS_THD_HIGH][CMD];
	int err;

	err = kstrtou16(buf, 10, &thresh_value);
	if (err < 0)
		SENSOR_ERR("kstrtoint failed\n");

	if (thresh_value > 2) {
		ps_reg_init_setting[PS_THD_HIGH][CMD] = thresh_value;
		err = cm36672p_i2c_write_word(data, REG_PS_THD_HIGH,
			ps_reg_init_setting[PS_THD_HIGH][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36672_ps_high_reg is failed. %d\n", err);
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
	struct cm36672p_data *data = dev_get_drvdata(dev);
	u16 thresh_value = ps_reg_init_setting[PS_THD_LOW][CMD];
	int err;

	err = kstrtou16(buf, 10, &thresh_value);
	if (err < 0)
		SENSOR_ERR("kstrtoint failed\n");

	SENSOR_INFO("thresh_value:%u\n", thresh_value);
	if (thresh_value > 2) {
		ps_reg_init_setting[PS_THD_LOW][CMD] = thresh_value;
		err = cm36672p_i2c_write_word(data, REG_PS_THD_LOW,
			ps_reg_init_setting[PS_THD_LOW][CMD]);
		if (err < 0)
			SENSOR_ERR("cm36672_ps_low_reg is failed. %d\n", err);
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
	struct cm36672p_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%2x,%4x", &regist, &val) != 2) {
		SENSOR_ERR("The number of data are wrong\n");
		return -EINVAL;
	}

	cm36672p_i2c_write_word(data, regist, val);
	SENSOR_INFO("Register(0x%2x) 16:data(0x%4x) 10:%d\n",
			regist, val, val);

	return count;
}

static ssize_t proximity_register_read_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u16 val[10], i;
	struct cm36672p_data *data = dev_get_drvdata(dev);

	for (i = 0; i < 10; i++) {
		cm36672p_i2c_read_word(data, i, &val[i]);
		SENSOR_INFO("Register(0x%2x) data(0x%4x)\n", i, val[i]);
	}

	return snprintf(buf, PAGE_SIZE,
		"0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",
		val[0], val[1], val[2], val[3], val[4],
		val[5], val[6], val[7], val[8], val[9]);
}
#endif

static DEVICE_ATTR(vendor, 0444, cm36672p_vendor_show, NULL);
static DEVICE_ATTR(name, 0444, cm36672p_name_show, NULL);
#ifdef CM36672P_CANCELLATION
static DEVICE_ATTR(prox_cal, 0664,
	proximity_cancel_show, proximity_cancel_store);
static DEVICE_ATTR(prox_offset_pass, 0444, proximity_cancel_pass_show,
	NULL);
#endif
static DEVICE_ATTR(prox_avg, 0664,
	proximity_avg_show, proximity_avg_store);
static DEVICE_ATTR(state, 0444, proximity_state_show, NULL);
static DEVICE_ATTR(raw_data, 0444, proximity_state_show, NULL);
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

static struct device_attribute *prox_sensor_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_prox_avg,
	&dev_attr_state,
	&dev_attr_thresh_high,
	&dev_attr_thresh_low,
	&dev_attr_raw_data,
	&dev_attr_prox_trim,
#if defined(PROXIMITY_FOR_TEST)
	&dev_attr_prox_register,
#endif
#ifdef CM36672P_CANCELLATION
	&dev_attr_prox_cal,
	&dev_attr_prox_offset_pass,
#endif
	NULL,
};

/* interrupt happened due to transition/change of near/far proximity state */
irqreturn_t proximity_irq_thread_fn(int irq, void *user_data)
{
	struct cm36672p_data *data = user_data;
	u8 val;
	u16 ps_data = 0;
	int enabled;

	enabled = atomic_read(&data->enable);
	val = gpio_get_value(data->irq_gpio);
	cm36672p_i2c_read_word(data, REG_PS_DATA, &ps_data);

	if (enabled) {
#ifdef CONFIG_SEC_FACTORY
		SENSOR_INFO("FACTORY: near/far=%d, ps code = %d\n",
				val, ps_data);
#else
		SENSOR_INFO("near/far=%d, ps code = %d\n",
				val, ps_data);
		if (((!val) && (ps_data >= ps_reg_init_setting[PS_THD_HIGH][CMD])) ||
			(val && (ps_data <= ps_reg_init_setting[PS_THD_LOW][CMD])))
#endif
		{
				/* 0 is close, 1 is far */
				input_report_abs(data->proximity_input_dev, ABS_DISTANCE,
					val);
				input_sync(data->proximity_input_dev);
		}
	}
	wake_lock_timeout(&data->prox_wake_lock, 3 * HZ);

	return IRQ_HANDLED;
}

static void proximity_get_avg_val(struct cm36672p_data *data)
{
	int min = 0, max = 0, avg = 0;
	int i;
	u16 ps_data = 0;

	for (i = 0; i < PROX_READ_NUM; i++) {
		msleep(40);
		cm36672p_i2c_read_word(data, REG_PS_DATA,
			&ps_data);
		avg += ps_data;

		if (!i)
			min = ps_data;
		else if (ps_data < min)
			min = ps_data;

		if (ps_data > max)
			max = ps_data;
	}
	avg /= PROX_READ_NUM;

	data->avg[0] = min;
	data->avg[1] = avg;
	data->avg[2] = max;
}

static void cm36672_work_func_prox(struct work_struct *work)
{
	struct cm36672p_data *data = container_of(work,
		struct cm36672p_data, work_prox);
	proximity_get_avg_val(data);
}

static enum hrtimer_restart cm36672_prox_timer_func(struct hrtimer *timer)
{
	struct cm36672p_data *data = container_of(timer,
		struct cm36672p_data, prox_timer);
	queue_work(data->prox_wq, &data->work_prox);
	hrtimer_forward_now(&data->prox_timer, data->prox_poll_delay);
	return HRTIMER_RESTART;
}

static int proximity_vdd_onoff(struct device *dev, bool onoff)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);
	int ret;

	SENSOR_INFO("%s\n", (onoff) ? "on" : "off");

	/* ldo control */
	if (data->vdd_ldo) {
		gpio_set_value(data->vdd_ldo, onoff);
		if (onoff)
			msleep(20);
		SENSOR_INFO("end (%d) \n", gpio_get_value(data->vdd_ldo));
		return 0;
	}

	if (!data->vdd) {
		SENSOR_INFO("VDD get regulator\n");
		data->vdd = devm_regulator_get(dev, "cm36672p,vdd");
		if (IS_ERR(data->vdd)) {
			SENSOR_ERR("cannot get vdd\n");
			data->vdd = NULL;
			return -ENOMEM;
		}

		if (!regulator_get_voltage(data->vdd))
			regulator_set_voltage(data->vdd, 3000000, 3300000);
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
	struct cm36672p_data *data = dev_get_drvdata(dev);
	int ret;

	SENSOR_INFO("%s, ldo:%d\n",
		(onoff) ? "on" : "off", data->vled_ldo);

	/* ldo control */
	if (data->vled_ldo) {
		gpio_set_value(data->vled_ldo, onoff);
		if (onoff)
			msleep(20);
		return 0;
	}

	/* regulator(PMIC) control */
	if (!data->vled) {
		SENSOR_INFO("VLED get regulator\n");
		data->vled = devm_regulator_get(dev, "cm36672p,vled");
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

static int setup_reg_cm36672p(struct cm36672p_data *data)
{
	int ret, i;
	u16 tmp;

	/* PS initialization */
	for (i = 0; i < PS_REG_NUM; i++) {
		ret = cm36672p_i2c_write_word(data,
			ps_reg_init_setting[i][REG_ADDR],
			ps_reg_init_setting[i][CMD]);
		if (ret < 0) {
			SENSOR_ERR("cm36672_ps_reg is failed. %d\n", ret);
			return ret;
		}
	}

	/* printing the initial proximity value with no contact */
	msleep(50);
	mutex_lock(&data->read_lock);
	ret = cm36672p_i2c_read_word(data, REG_PS_DATA, &tmp);
	mutex_unlock(&data->read_lock);
	if (ret < 0) {
		SENSOR_ERR("read ps_data failed\n");
		ret = -EIO;
	}

	/* turn off */
	cm36672p_i2c_write_word(data, REG_PS_CONF1, 0x0001);
	cm36672p_i2c_write_word(data, REG_PS_CONF3, 0x0000);

	return ret;
}

static int setup_irq_cm36672p(struct cm36672p_data *data)
{
	int ret;

	ret = gpio_request(data->irq_gpio, "gpio_proximity_out");
	if (ret < 0) {
		SENSOR_ERR("gpio %d request failed(%d)\n", data->irq_gpio, ret);
		return ret;
	}

	ret = gpio_direction_input(data->irq_gpio);
	if (ret < 0) {
		SENSOR_ERR("failed to set gpio %d as input(%d)\n",
						data->irq_gpio, ret);
		gpio_free(data->irq_gpio);
		return ret;
	}

	data->irq = gpio_to_irq(data->irq_gpio);
	/* add IRQF_NO_SUSPEND option in case of Spreadtrum AP */
	ret = request_threaded_irq(data->irq, NULL, proximity_irq_thread_fn,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		"proximity_int", data);
	if (ret < 0) {
		SENSOR_ERR("request_irq(%d) failed for gpio %d (%d)\n",
			data->irq, data->irq_gpio, ret);
		gpio_free(data->irq_gpio);
		return ret;
	}

	/* start with interrupts disabled */
	disable_irq(data->irq);

	SENSOR_ERR("success\n");
	return ret;
}

/* device tree parsing function */
static int cm36672p_parse_dt(struct device *dev,
	struct cm36672p_data *data)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int ret;
	u32 temp;

	if (!data) {
		SENSOR_ERR("missing pdata\n");
		return -ENOMEM;
	}

	data->vdd_ldo = of_get_named_gpio_flags(np, "cm36672p,vdd_ldo",
		0, &flags);
	if (data->vdd_ldo < 0) {
		SENSOR_INFO("Cannot set vdd_ldo through DTSI\n");
		data->vdd_ldo = 0;
	} else {
		ret = gpio_request(data->vdd_ldo, "prox_vdd_en");
		if (ret < 0)
			SENSOR_ERR("gpio %d request failed (%d)\n",
				data->vdd_ldo, ret);
		else
			gpio_direction_output(data->vdd_ldo, 0);
	}

	ret = of_property_read_u32(np, "cm36672p,vdd_always_on",
		&data->vdd_always_on);

	ret = of_property_read_u32(np, "cm36672p,regulator_divided",
		&data->regulator_divided);

	data->irq_gpio = of_get_named_gpio_flags(np, "cm36672p,irq_gpio", 0,
		&flags);
	if (data->irq < 0) {
		SENSOR_ERR("get prox_int error\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(np, "cm36672p,default_hi_thd",
		&data->default_hi_thd);
	if (ret < 0) {
		SENSOR_ERR("Cannot set default_hi_thd\n");
		data->default_hi_thd = DEFAULT_HI_THD;
	}

	ret = of_property_read_u32(np, "cm36672p,default_low_thd",
		&data->default_low_thd);
	if (ret < 0) {
		SENSOR_ERR("Cannot set default_low_thd\n");
		data->default_low_thd = DEFAULT_LOW_THD;
	}

	ret = of_property_read_u32(np, "cm36672p,cancel_hi_thd",
		&data->cancel_hi_thd);
	if (ret < 0) {
		SENSOR_ERR("Cannot set cancel_hi_thd\n");
		data->cancel_hi_thd = CANCEL_HI_THD;
	}

	ret = of_property_read_u32(np, "cm36672p,cancel_low_thd",
		&data->cancel_low_thd);
	if (ret < 0) {
		SENSOR_ERR("Cannot set cancel_low_thd\n");
		data->cancel_low_thd = CANCEL_LOW_THD;
	}

	ret = of_property_read_u32(np, "cm36672p,offset_range_hi",
		&data->offset_range_hi);
	if (ret < 0) {
		SENSOR_ERR("Cannot set offset_range_hi\n");
		data->offset_range_hi = data->default_hi_thd;
	}

	ret = of_property_read_u32(np, "cm36672p,offset_range_low",
		&data->offset_range_low);
	if (ret < 0) {
		SENSOR_ERR("Cannot set offset_range_low\n");
		data->offset_range_low = (int)((data->default_low_thd)*50/100);
	}

	SENSOR_INFO("offset_range_hi = 0x%X, offset_range_low = 0x%X\n",
		data->offset_range_hi,
		data->offset_range_low);

	/* Proximity Duty ratio Register Setting */
	ret = of_property_read_u32(np, "cm36672p,ps_duty", &temp);
	if (ret < 0) {
		SENSOR_ERR("Cannot set ps_duty\n");
		ps_reg_init_setting[PS_CONF1][CMD] |= DEFAULT_CONF1;
	} else {
		temp = temp << 6;
		ps_reg_init_setting[PS_CONF1][CMD] |= temp;
	}

	/* Proximity Interrupt Persistence Register Setting */
	ret = of_property_read_u32(np, "cm36672p,ps_pers", &temp);
	if (ret < 0) {
		SENSOR_ERR("Cannot set ps_pers\n");
		ps_reg_init_setting[PS_CONF1][CMD] |= DEFAULT_CONF1;
	} else {
		temp = temp << 4;
		ps_reg_init_setting[PS_CONF1][CMD] |= temp;
	}

	/* Proximity Integration Time Register Setting */
	ret = of_property_read_u32(np, "cm36672p,ps_it", &temp);
	if (ret < 0) {
		SENSOR_ERR("Cannot set ps_it\n");
		ps_reg_init_setting[PS_CONF1][CMD] |= DEFAULT_CONF1;
	} else {
		temp = temp << 1;
		ps_reg_init_setting[PS_CONF1][CMD] |= temp;
	}

	/* Proximity LED Current Register Setting */
	ret = of_property_read_u32(np, "cm36672p,led_current", &temp);
	if (ret < 0) {
		SENSOR_ERR("Cannot set led_current\n");
		ps_reg_init_setting[PS_CONF3][CMD] |= DEFAULT_CONF3;
	} else {
		temp = temp << 8;
		ps_reg_init_setting[PS_CONF3][CMD] |= temp;
	}

	/* Proximity Smart Persistence Register Setting */
	ret = of_property_read_u32(np, "cm36672p,ps_smart_pers", &temp);
	if (ret < 0) {
		SENSOR_ERR("Cannot set ps_smart_pers\n");
		ps_reg_init_setting[PS_CONF3][CMD] |= DEFAULT_CONF3;
	} else {
		temp = temp << 4;
		ps_reg_init_setting[PS_CONF3][CMD] |= temp;
	}


	ret = of_property_read_u32(np, "cm36672p,default_trim",
		&data->default_trim);
	if (ret < 0) {
		SENSOR_ERR("Cannot set default_trim\n");
		data->default_trim = DEFAULT_TRIM;
	}

	ps_reg_init_setting[PS_THD_LOW][CMD] = data->default_low_thd;
	ps_reg_init_setting[PS_THD_HIGH][CMD] = data->default_hi_thd;
	ps_reg_init_setting[PS_CANCEL][CMD] = data->default_trim;

	SENSOR_INFO("initial CONF1 = 0x%X, CONF3 = 0x%X, vdd_alwayson_on: %d, vled_ldo: %d\n",
		ps_reg_init_setting[PS_CONF1][CMD],
		ps_reg_init_setting[PS_CONF3][CMD],
		data->vdd_always_on, data->vled_ldo);

	return 0;
}

static int cm36672p_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	struct cm36672p_data *data = NULL;

	SENSOR_INFO("start\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SENSOR_ERR("i2c functionality check failed!\n");
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct cm36672p_data), GFP_KERNEL);
	if (!data) {
		SENSOR_ERR("failed to alloc memory for sensor drv data\n");
		return -ENOMEM;
	}

	if (client->dev.of_node) {
		ret = cm36672p_parse_dt(&client->dev, data);
		if (ret)
			goto err_parse_dt;
	}

	data->i2c_client = client;
	i2c_set_clientdata(client, data);

	mutex_init(&data->read_lock);
	/* wake lock init for proximity sensor */
	wake_lock_init(&data->prox_wake_lock, WAKE_LOCK_SUSPEND,
		"prox_wake_lock");

	proximity_vdd_onoff(&client->dev, ON);
	if (!data->regulator_divided)
		proximity_vled_onoff(&client->dev, ON);

	/* Check if the device is there or not. */
	ret = cm36672p_i2c_write_word(data, REG_PS_CONF1, 0x0001);
	if (ret < 0) {
		SENSOR_ERR("cm36672 is not connected(%d)\n", ret);
		goto err_setup_dev;
	}

	/* setup initial registers */
	ret = setup_reg_cm36672p(data);
	if (ret < 0) {
		SENSOR_ERR("could not setup regs\n");
		goto err_setup_dev;
	}

	/* allocate proximity input_device */
	data->proximity_input_dev = input_allocate_device();
	if (!data->proximity_input_dev) {
		SENSOR_ERR("could not allocate proximity input device\n");
		goto err_input_alloc_device;
	}

	input_set_drvdata(data->proximity_input_dev, data);
	data->proximity_input_dev->name = MODULE_NAME;
	input_set_capability(data->proximity_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(data->proximity_input_dev, ABS_DISTANCE,
		0, 1, 0, 0);

	ret = input_register_device(data->proximity_input_dev);
	if (ret < 0) {
		input_free_device(data->proximity_input_dev);
		SENSOR_ERR("could not register input device\n");
		goto err_input_register_device;
	}

	ret = sensors_create_symlink(&data->proximity_input_dev->dev.kobj,
		data->proximity_input_dev->name);
	if (ret < 0) {
		SENSOR_ERR("create_symlink error\n");
		goto err_sensors_create_symlink_prox;
	}

	ret = sysfs_create_group(&data->proximity_input_dev->dev.kobj,
		&proximity_attribute_group);
	if (ret) {
		SENSOR_ERR("could not create sysfs group\n");
		goto err_sysfs_create_group_proximity;
	}

	/* setup irq */
	ret = setup_irq_cm36672p(data);
	if (ret) {
		SENSOR_ERR("could not setup irq\n");
		goto err_setup_irq;
	}

	/* For factory test mode, we use timer to get average proximity data. */
	/* prox_timer settings. we poll for light values using a timer. */
	hrtimer_init(&data->prox_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->prox_poll_delay = ns_to_ktime(2000 * NSEC_PER_MSEC);/*2 sec*/
	data->prox_timer.function = cm36672_prox_timer_func;

	/* the timer just fires off a work queue request. */
	/* we need a thread to read the i2c (can be slow and blocking). */
	data->prox_wq = create_singlethread_workqueue("cm36672_prox_wq");
	if (!data->prox_wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create prox workqueue\n");
		goto err_create_prox_workqueue;
	}
	/* this is the thread function we run on the work queue */
	INIT_WORK(&data->work_prox, cm36672_work_func_prox);

	/* set sysfs for proximity sensor */
	ret = sensors_register(&data->proximity_dev,
		data, prox_sensor_attrs, MODULE_NAME);
	if (ret) {
		SENSOR_ERR("failed to register proximity dev(%d)\n", ret);
		goto err_prox_sensor_register;
	}
	if (!data->regulator_divided)
		proximity_vled_onoff(&client->dev, OFF);
	if (!data->vdd_always_on)
		proximity_vdd_onoff(&client->dev, OFF);

	SENSOR_INFO("success\n");
	return ret;

/* error, unwind it all */
err_prox_sensor_register:
	destroy_workqueue(data->prox_wq);
err_create_prox_workqueue:
	free_irq(data->irq, data);
	gpio_free(data->irq_gpio);
err_setup_irq:
	sysfs_remove_group(&data->proximity_input_dev->dev.kobj,
		&proximity_attribute_group);
err_sysfs_create_group_proximity:
	sensors_remove_symlink(&data->proximity_input_dev->dev.kobj,
		data->proximity_input_dev->name);
err_sensors_create_symlink_prox:
	input_unregister_device(data->proximity_input_dev);
err_input_register_device:
	input_free_device(data->proximity_input_dev);
err_input_alloc_device:
err_setup_dev:
	if (!data->regulator_divided)
		proximity_vled_onoff(&client->dev, OFF);
	if (data->vled_ldo)
		gpio_free(data->vled_ldo);
	proximity_vdd_onoff(&client->dev, OFF);
	if (data->vdd_ldo)
		gpio_free(data->vdd_ldo);
	wake_lock_destroy(&data->prox_wake_lock);
	mutex_destroy(&data->read_lock);
err_parse_dt:
	kfree(data);

	SENSOR_ERR("failed (%d)\n", ret);
	return ret;
}

static int cm36672p_i2c_remove(struct i2c_client *client)
{
	SENSOR_INFO("\n");
	return 0;
}

static void cm36672p_i2c_shutdown(struct i2c_client *client)
{
	struct cm36672p_data *data = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&data->enable);

	SENSOR_INFO("pre_enable = %d\n", pre_enable);

	if (pre_enable == 1) {
		disable_irq_wake(data->irq);
		disable_irq(data->irq);
		cm36672p_i2c_write_word(data, REG_PS_CONF1, 0x0001);
		if (!data->regulator_divided)
			proximity_vled_onoff(&client->dev, OFF);
	}
	proximity_vdd_onoff(&client->dev, OFF);
	SENSOR_INFO("done\n");
}

static int cm36672p_suspend(struct device *dev)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);
	int enable;

	SENSOR_INFO("is called.\n");

	enable = atomic_read(&data->enable);

	if (enable)
		disable_irq(data->irq);

	return 0;
}

static int cm36672p_resume(struct device *dev)
{
	struct cm36672p_data *data = dev_get_drvdata(dev);
	int enable;

	SENSOR_INFO("is called.\n");

	enable = atomic_read(&data->enable);

	if (enable)
		enable_irq(data->irq);

	return 0;
}

static const struct of_device_id cm36672p_match_table[] = {
	{ .compatible = "cm36672p",},
	{},
};

static const struct i2c_device_id cm36672p_device_id[] = {
	{"cm36672p", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cm36672p_device_id);

static const struct dev_pm_ops cm36672p_pm_ops = {
	.suspend = cm36672p_suspend,
	.resume = cm36672p_resume
};

static struct i2c_driver cm36672p_i2c_driver = {
	.driver = {
		   .name = "cm36672p",
		   .owner = THIS_MODULE,
		   .of_match_table = cm36672p_match_table,
		   .pm = &cm36672p_pm_ops
	},
	.probe = cm36672p_i2c_probe,
	.remove = cm36672p_i2c_remove,
	.shutdown = cm36672p_i2c_shutdown,
	.id_table = cm36672p_device_id,
};

static int __init cm36672p_init(void)
{
	return i2c_add_driver(&cm36672p_i2c_driver);
}

static void __exit cm36672p_exit(void)
{
	i2c_del_driver(&cm36672p_i2c_driver);
}

module_init(cm36672p_init);
module_exit(cm36672p_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Proximity Sensor device driver for CM36672P");
MODULE_LICENSE("GPL");
