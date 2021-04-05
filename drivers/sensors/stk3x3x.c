/* STK3X3X Proximity Sensor Driver
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/pm.h>
#include <linux/string.h>
#include <linux/sensor/sensors_core.h>
#ifdef CONFIG_OF
	#include <linux/of_gpio.h>
#endif
#include "stk3x3x.h"

#define MODULE_NAME         "proximity_sensor"
#define VENDOR_NAME         "Sensortek"
#define CHIP_NAME           "STK3031"
#define PROX_READ_NUM       25
#define PS_WAIT             52000
#define PROX_MIN_DATA          0
#define PDATA_MIN           0
#define PDATA_MAX           0xFFFF
#define FIRST_CAL_ADC_LIMIT 120
#define MIN_INT             -99999
#define MAX_INT             99999
#define AVG_SAMPLE_COUNT     6
#define CAL_FAIL_MAX_MIN_RANGE  20
#define CAL_RESET_CLOSE_CNT  3
#define POCKET_DATA_NUM 3

stk3x3x_register_table stk3x3x_default_register_table[] = {
	{STK3X3X_PSCTRL_REG,            (STK3X3X_PS_PRS4 | STK3X3X_PS_GAIN8 | STK3X3X_PS_IT400),    0xFF},
	{STK3X3X_LEDCTRL_REG,           STK3X3X_LED_150mA,                                          0xFF},
	{STK3X3X_INT_REG,               STK3X3X_INT_NF_EN,                                          0xFF},
	{STK3X3X_WAIT_REG,              STK3X3X_WAIT50,                                             0xFF},
	{0xDB,                          0x14,                                                       0xFF},
	{0xF4,                          0xC0,                                                       0xFF},
#ifdef STK_INTELLI_PERSIST
	{STK3X3X_INTELLI_WAIT_PS_REG,   STK3X3X_INTELL_13,                                          0xFF},
#endif
#ifdef STK_CTIR
	{0xF3,                          0x00,                                                       0x07},
#endif
#ifdef STK_BGIR
	{0xA0,                          0x10,                                                       0xFF},
	{0xAA,                          0x00,                                                       0xFF},
#endif
};

uint8_t stk3x3x_reg_map[] =
	{
		STK3X3X_STATE_REG,
		STK3X3X_PSCTRL_REG,
		STK3X3X_ALSCTRL_REG,
		STK3X3X_LEDCTRL_REG,
		STK3X3X_INT_REG,
		STK3X3X_WAIT_REG,
		STK3X3X_THDH1_PS_REG,
		STK3X3X_THDH2_PS_REG,
		STK3X3X_THDL1_PS_REG,
		STK3X3X_THDL2_PS_REG,
		STK3X3X_THDH1_ALS_REG,
		STK3X3X_THDH2_ALS_REG,
		STK3X3X_THDL1_ALS_REG,
		STK3X3X_THDL2_ALS_REG,
		STK3X3X_FLAG_REG,
		STK3X3X_DATA1_PS_REG,
		STK3X3X_DATA2_PS_REG,
		STK3X3X_DATA1_ALS_REG,
		STK3X3X_DATA2_ALS_REG,
		STK3X3X_DATA1_R_REG,
		STK3X3X_DATA2_R_REG,
		STK3X3X_DATA1_G_REG,
		STK3X3X_DATA2_G_REG,
		STK3X3X_DATA1_B_REG,
		STK3X3X_DATA2_B_REG,
		STK3X3X_DATA1_C_REG,
		STK3X3X_DATA2_C_REG,
		STK3X3X_DATA1_OFFSET_REG,
		STK3X3X_DATA2_OFFSET_REG,
		STK3X3X_DATA_CTIR1_REG,
		STK3X3X_DATA_CTIR2_REG,
		STK3X3X_DATA_CTIR3_REG,
		STK3X3X_DATA_CTIR4_REG,
		STK3X3X_PDT_ID_REG,
		STK3X3X_RSRVD_REG,
		STK3X3X_ALSCTRL2_REG,
		STK3X3X_INTELLI_WAIT_PS_REG,
		0xFA,
		STK3X3X_SW_RESET_REG
	};

static int stk3x3x_reg_read(struct stk3x3x_data *drv_data,
							unsigned char reg)
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(drv_data->client, reg);
	if (ret < 0)
		SENSOR_ERR("failed, reg=0x%x, ret=%d\n", reg, ret);

	return ret;
}

static int stk3x3x_reg_write(struct stk3x3x_data *drv_data,
							 unsigned char reg, unsigned char val)
{
	int ret = 0;

	ret = i2c_smbus_write_byte_data(drv_data->client, reg, val);
	if (ret < 0)
		SENSOR_ERR("failed, reg=0x%x, ret=%d\n", reg, ret);

	return ret;
}

static int stk3x3x_reg_write_block(struct stk3x3x_data *drv_data,
					unsigned char reg, unsigned char *val, unsigned char length)
{
	int ret = 0;

	ret = i2c_smbus_write_i2c_block_data(drv_data->client, reg, length, val);
	if (ret < 0)
		SENSOR_ERR("failed, reg=0x%x, ret=%d\n", reg, ret);

	return ret;
}

static int stk3x3x_reg_read_modify_write(struct stk3x3x_data *drv_data,
		unsigned char reg, unsigned char val, unsigned char mask)
{
	uint8_t rw_buffer = 0;
	int ret = 0;

	if ((mask == 0xFF) || (mask == 0x0)) {
		ret = stk3x3x_reg_write(drv_data, reg, val);
		if (ret < 0)
			SENSOR_ERR("failed, reg=0x%x, ret=%d\n", reg, ret);
	} else {
		rw_buffer = (uint8_t)stk3x3x_reg_read(drv_data, reg);
		if (rw_buffer < 0) {
			SENSOR_ERR("failed, reg=0x%x, rw_buffer=%d\n", reg, rw_buffer);
			return rw_buffer;
		} else {
			rw_buffer = (rw_buffer & (~mask)) | (val & mask);

			ret = stk3x3x_reg_write(drv_data, reg, rw_buffer);
			if (ret < 0)
				SENSOR_ERR("failed, reg=0x%x, ret=%d\n", reg, ret);
		}
	}

	return ret;
}

static int stk3x3x_reg_read_block(struct stk3x3x_data *drv_data,
								  unsigned char reg, int count, void *buf)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(drv_data->client, reg, count, buf);
	if (ret < 0)
		SENSOR_ERR("failed, reg=0x%x, ret=%d\n", reg, ret);

	return ret;
}



static irqreturn_t stk3x3x_irq_handler(int irq, void *data)
{
	struct stk3x3x_data *drv_data = data;

	SENSOR_INFO("\n");

	disable_irq_nosync(irq);
	queue_work(drv_data->prox_irq_wq, &drv_data->prox_irq_work);

	return IRQ_HANDLED;
}

static int32_t stk3x3x_get_mid(struct stk3x3x_data *drv_data)
{
	int32_t ret = 0;

	ret = STK3X3X_REG_READ(drv_data, 0xE0);
	if (ret < 0)
		SENSOR_ERR("failed, ret=%d\n", ret);

	return ret;
}

static int32_t stk3x3x_prst_cnt_rst_sel(struct stk3x3x_data *drv_data)
{
	uint8_t reg_value = 0x0;
	int32_t ret = 0;

	if (0 == (stk3x3x_get_mid(drv_data) & 0x1))
		reg_value = 0x1;

	ret = STK3X3X_REG_READ_MODIFY_WRITE(drv_data, 0xFA, reg_value, 0xFF);
	if (ret < 0)
		SENSOR_ERR("failed, ret=%d\n", ret);

	return ret;
}

static int32_t stk3x3x_device_id_check(struct stk3x3x_data *drv_data)
{
	u8 val;
	int ret;

	ret = STK3X3X_REG_READ(drv_data, STK3X3X_PDT_ID_REG);
	if (ret < 0) {
		SENSOR_ERR("Device id read failed, ret=%d\n", ret);
	} else {
		val = (u8) ret;

		if (val == STK3331_DEVICE_ID_VAL || val == STK3031_DEVICE_ID_VAL) {
			SENSOR_INFO("Device matched, device_id=0x%2x\n", val);
			return 0;
		} else {
			SENSOR_ERR("Device not matched, device_id=0x%2x\n", val);
			return -ENODEV;
		}
	}

	return ret;
}

static int32_t stk3x3x_software_reset(struct stk3x3x_data *drv_data)
{
	int32_t ret = 0;

	ret = STK3X3X_REG_WRITE(drv_data, STK3X3X_SW_RESET_REG, 0x0);
	if (ret < 0) {
		SENSOR_ERR("failed, ret=%d\n", ret);
		return ret;
	}

	usleep_range(13000, 15000);

	return 0;
}

int stk3x3x_bgir_check(struct stk3x3x_data *drv_data)
{
	int32_t ret = 0;
	uint8_t ps_invalid_flag, bgir_raw_data[4] = {0};
	bool bgir_out_of_range = false;
	uint8_t i;

	ret = STK3X3X_REG_READ(drv_data, 0xA7);
	if (ret < 0) {
		SENSOR_ERR("0xA7 read failed, ret=%d\n", ret);
		return ret;
	}

	ps_invalid_flag = (uint8_t)ret;

	ret = STK3X3X_REG_BLOCK_READ(drv_data, 0x34, 4, &bgir_raw_data[0]);
	if (ret < 0) {
		SENSOR_ERR("0x34 read failed, ret=%d\n", ret);
		return ret;
	}

	for (i = 0; i < 4; i++) {
		if (*(bgir_raw_data + i) > STK3X3X_PS_BGIR_THRESHOLD) {
			bgir_out_of_range = true;
			SENSOR_ERR("BGIR invalid, bgir[%d]=0x%X\n", i, *(bgir_raw_data + i));
			break;
		}
	}

	if (((ps_invalid_flag >> 5) & 0x1) || bgir_out_of_range) {
		ret = 0x7FFF0001;
	}

	return ret;
}

int32_t stk3x3x_set_ps_thd(struct stk3x3x_data *drv_data, uint16_t thd_h, uint16_t thd_l)
{
	unsigned char val[4];
	int ret;

	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	val[2] = (thd_l & 0xFF00) >> 8;
	val[3] = thd_l & 0x00FF;

	ret = STK3X3X_REG_WRITE_BLOCK(drv_data, STK3X3X_THDH1_PS_REG, val, sizeof(val));
	if (ret < 0)
		SENSOR_ERR("failed, ret=%d\n", ret);
	else {
		drv_data->prox_thd_h = thd_h;
		drv_data->prox_thd_l = thd_l;
	}

	return ret;
}

static uint32_t stk3x3x_get_ps_reading(struct stk3x3x_data *ps_data)
{
	unsigned char value[2];
	int ret;

	ret = STK3X3X_REG_BLOCK_READ(ps_data, STK3X3X_DATA1_PS_REG, 2, &value[0]);

	if (ret < 0) {
		SENSOR_ERR("DATA1 fail, ret=%d\n", ret);
		return ret;
	}

	return (value[0]<<8) | value[1];
}

static int32_t stk3x3x_first_cal_enable_ps(struct stk3x3x_data *drv_data, bool en)
{
	int32_t ret = 0;
	uint8_t reg_value = 0;

	ret = STK3X3X_REG_READ(drv_data, STK3X3X_STATE_REG);
	if (ret < 0) {
		goto done;
	} else {
		reg_value = (uint8_t)ret;
	}

	reg_value &= (~(STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK));

	if (en) {
			SENSOR_INFO("First Cal Enable\n");
			stk3x3x_set_ps_thd(drv_data, drv_data->prox_thd_h, drv_data->prox_thd_l);

			reg_value |= (STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK);

			ret = STK3X3X_REG_READ_MODIFY_WRITE(drv_data, STK3X3X_STATE_REG, reg_value, 0xFF);
			if (ret < 0)
				goto done;
	} else {
		SENSOR_INFO("First Cal Disable\n");

		reg_value &= (~(STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK));

		ret = STK3X3X_REG_READ_MODIFY_WRITE(drv_data, STK3X3X_STATE_REG, reg_value, 0xFF);
		if (ret < 0)
			goto done;
	}

done:

	return ret;
}

static void stk3x3x_prox_cal(struct stk3x3x_data *ps_data)
{
	int i, sum_cnt = 0, ret;
	uint16_t read_value, avg_value = 0, min = PDATA_MAX, max = PDATA_MIN;
	uint32_t sum = 0;
	uint8_t sunlight_protection_mode = 0;

	if (ps_data->enable == false) {
		if (ps_data->cal_status == STK3X3X_FIRST_CAL) {
			stk3x3x_first_cal_enable_ps(ps_data, 1);
		} else {
			ps_data->cal_status = STK3X3X_CAL_DISABLED;
			SENSOR_ERR("prox sensor is disabled\n");
			return;
		}
	}

	// set wait time as 10ms
	ret = STK3X3X_REG_WRITE(ps_data, STK3X3X_WAIT_REG, 0x5);
	if(ret < 0) {
		SENSOR_ERR("WAIT_REG failed %d\n", ret);
		goto exit;
	}
	if (ps_data->cal_status == STK3X3X_FIRST_CAL) {
		usleep_range(500000, 500000);
	} else 
		usleep_range(60000, 60000);

	for (i = 0; i < AVG_SAMPLE_COUNT; i++) {
		if (ps_data->ps_it == STK3X3X_PS_IT1540) // wait-time = 9.24ms and IT-PS time = 1.54 ms
		    usleep_range(12000, 12000);
		else
		    usleep_range(10000, 10000); // wait-time = 9.24ms and IT-PS time = 368 us

		if (ps_data->cal_status == STK3X3X_CAL_ONGOING && ps_data->enable == false) {
			SENSOR_ERR("sensor disabled exit\n");
			goto exit;
		}

		// check sunlight mode
		ret = STK3X3X_REG_READ(ps_data, STK3X3X_SUNLIGHT_CHECK_REG);
		if (ret < 0) {
			SENSOR_ERR("STK3X3X_SUNLIGHT_CHECK_REG read fail, ret=%d\n", ret);
			goto exit;
		}

		sunlight_protection_mode = (uint8_t)ret;

		// read adc value
		read_value = stk3x3x_get_ps_reading(ps_data);
		
		if (read_value > max)
			max = read_value;
		if (read_value < min)
			min = read_value;

		SENSOR_INFO("read_value = %d, (0x%x)\n", read_value, sunlight_protection_mode);
		if (((sunlight_protection_mode >> 5) & 0x1) && read_value == 0) {
			if (ps_data->cal_status == STK3X3X_FIRST_CAL) {
				if (ps_data->sunlight_thd_h != PDATA_MAX && ps_data->sunlight_thd_l != PDATA_MAX) {
					stk3x3x_set_ps_thd(ps_data, ps_data->sunlight_thd_h, ps_data->sunlight_thd_l);
					SENSOR_INFO("SUNLIGHT PROTECTION, set as sunlight thd h(%u) l(%u)\n",
						ps_data->sunlight_thd_h, ps_data->sunlight_thd_l);
				} else
					SENSOR_INFO("SUNLIGHT PROTECTION, set as default thd h(%u) l(%u)\n",
						ps_data->prox_thd_h, ps_data->prox_thd_l);
			} else
				SENSOR_ERR("SUNLIGHT PROTECTION, calibration is failed, (0x%x)\n"
					, sunlight_protection_mode);
			goto exit;
		} else if ((ps_data->cal_status == STK3X3X_CAL_ONGOING)
			&& (read_value > ps_data->prox_thd_l - 10)) {
			SENSOR_ERR("cal failed ps_data = %d, thd l %u\n", read_value, ps_data->prox_thd_l);
			goto exit;
		} else if (!ps_data->first_limit_skip && (ps_data->cal_status == STK3X3X_FIRST_CAL) 
				&& read_value > ps_data->first_cal_adc_limit) {
			if (ps_data->first_cal_thd_h != PDATA_MAX && ps_data->first_cal_thd_l != PDATA_MAX) {
				stk3x3x_set_ps_thd(ps_data, ps_data->first_cal_thd_h, ps_data->first_cal_thd_l);
				SENSOR_INFO("first cal adc is too big. set as thd h(%u) l(%u)\n",
					ps_data->first_cal_thd_h, ps_data->first_cal_thd_l);
			} else
				SENSOR_INFO("first cal adc is too big. set as default thd h(%u) l(%u)\n",
					ps_data->prox_thd_h, ps_data->prox_thd_l);
			goto exit;
		} else if (read_value >= PDATA_MIN && read_value <= PDATA_MAX) {
			sum += read_value;
			sum_cnt++;
		} else {
			SENSOR_ERR("ps_data is invalid(%d)\n", read_value);
			goto exit;
		}
	}

	if (max - min > CAL_FAIL_MAX_MIN_RANGE) {
		SENSOR_ERR("cal failed max(%u) - min(%u) = %u\n", max, min, max-min);
		goto exit;
	}

	if (sum_cnt == AVG_SAMPLE_COUNT) {
		avg_value = sum/AVG_SAMPLE_COUNT;
		SENSOR_INFO("sum = %d, avg = %d", sum, avg_value);
		ps_data->prox_thd_h = avg_value + ps_data->thd_h_offset;
		if (ps_data->cal_status == STK3X3X_FIRST_CAL)
			ps_data->prox_thd_l = avg_value + 30;
		else
			ps_data->prox_thd_l = avg_value + 20;
		stk3x3x_set_ps_thd(ps_data, ps_data->prox_thd_h, ps_data->prox_thd_l);
		SENSOR_INFO("cal done h=%u, l=%u\n", ps_data->prox_thd_h,
			ps_data->prox_thd_l);
	} else
		SENSOR_ERR("cal is failed (%d)\n", sum_cnt);

exit :
	ret = STK3X3X_REG_WRITE(ps_data, STK3X3X_WAIT_REG, STK3X3X_WAIT50);
	if (ret < 0)
		SENSOR_ERR("WAIT_REG failed %d\n", ret);

	if (ps_data->cal_status == STK3X3X_FIRST_CAL && !ps_data->factory_cal)
		stk3x3x_first_cal_enable_ps(ps_data, 0);
	ps_data->cal_status = STK3X3X_CAL_DISABLED;
	ps_data->factory_cal = false;
}

static void stk3x3x_work_func_prox_cal(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work,
	struct stk3x3x_data, work_cal_prox);

	mutex_lock(&ps_data->control_mutex);
	stk3x3x_prox_cal(ps_data);
	mutex_unlock(&ps_data->control_mutex);
}

static void stk3x3x_work_func_pocket_read(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work,
		struct stk3x3x_data, work_pocket);

	int32_t ret = 0;
	uint16_t read_adc = 0;
	uint8_t reg_value = 0, read_val[2] = {0,};
	uint8_t sunlight_protection_mode = 0;
	int i = 0;

	SENSOR_INFO("start\n");
	ps_data->pocket_prox = STK3X3X_POCKET_UNKNOWN;

	ret = STK3X3X_REG_READ(ps_data, STK3X3X_STATE_REG);
	if (ret < 0)
		goto exit;
	else
		reg_value = (uint8_t)ret;

		reg_value &= (~(STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK));

	stk3x3x_set_ps_thd(ps_data, ps_data->prox_thd_h, ps_data->prox_thd_l);

		reg_value |= (STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK);


	ret = STK3X3X_REG_READ_MODIFY_WRITE(ps_data, STK3X3X_STATE_REG, reg_value, 0xFF);
	if (ret < 0)
		goto exit;

	// set wait time as 1.74ms
	ret = STK3X3X_REG_WRITE(ps_data, STK3X3X_WAIT_REG, 0x1);
	if(ret < 0)
		SENSOR_ERR("WAIT_REG failed %d\n", ret);

	usleep_range(10000, 10000);

	// check sunlight mode
	ret = STK3X3X_REG_READ(ps_data, STK3X3X_SUNLIGHT_CHECK_REG);
	if (ret < 0) {
		SENSOR_ERR("STK3X3X_SUNLIGHT_CHECK_REG read fail, ret=%d\n", ret);
		goto exit;
	}

	sunlight_protection_mode = (uint8_t)ret;

	for (i = 0; i < POCKET_DATA_NUM; i++) {
		ret = STK3X3X_REG_BLOCK_READ(ps_data, STK3X3X_DATA1_PS_REG, 2, &read_val[0]);
		if (ret < 0) {
			SENSOR_ERR("ADC read failed, ret=%d\n", ret);
			return;
		} else {
			read_adc += (read_val[0] << 8 | read_val[1]);
			if (((sunlight_protection_mode >> 5) & 0x1) && read_adc == 0) {
				SENSOR_ERR("SUNLIGHT PROTECTION, prox read is failed, (0x%x)\n"
					, sunlight_protection_mode);
				ps_data->pocket_prox = STK3X3X_POCKET_FAR_AWAY;
				// turn off proximity sensor
				reg_value = 0;
				STK3X3X_REG_READ(ps_data, STK3X3X_STATE_REG);
				reg_value &= (~(STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK));
				STK3X3X_REG_READ_MODIFY_WRITE(ps_data, STK3X3X_STATE_REG, reg_value, 0xFF);
				goto exit;
			}
		}
		if (i < POCKET_DATA_NUM - 1)
			usleep_range(10000, 10000);
	}
	read_adc = read_adc / POCKET_DATA_NUM;

	reg_value = 0;
	ret = STK3X3X_REG_READ(ps_data, STK3X3X_STATE_REG);
	if (ret < 0) {
		goto exit;
	} else {
		reg_value = (uint8_t)ret;
	}

	reg_value &= (~(STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK));

	ret = STK3X3X_REG_READ_MODIFY_WRITE(ps_data, STK3X3X_STATE_REG, reg_value, 0xFF);
	if (ret < 0)
		goto exit;

	if (read_adc < ps_data->prox_thd_h)
		ps_data->pocket_prox = STK3X3X_POCKET_FAR_AWAY;
	else 
		ps_data->pocket_prox = STK3X3X_POCKET_NEAR_BY;

	SENSOR_INFO("adc=%d, thd_h=%d, prox=%d\n", read_adc, ps_data->prox_thd_h, ps_data->pocket_prox);


exit:  
	ps_data->pocket_running = false;
}

static void stk_ps_report(struct stk3x3x_data *drv_data, int nf)
{
	uint8_t reg_value = 0x0;
	uint8_t state_reg = 0x00;
	int32_t ret;

	input_report_rel(drv_data->prox_input_dev, REL_MISC, nf + 1);
	input_sync(drv_data->prox_input_dev);
	wake_lock_timeout(&drv_data->prox_wakelock, 3 * HZ);

	// A20 model only 
	if(drv_data->intel_prst == 0) {
		ret = STK3X3X_REG_READ(drv_data, STK3X3X_STATE_REG);
		if (ret < 0) {
			SENSOR_ERR("state register read failed, ret=%d\n", ret);		
		} else {
			state_reg = (uint8_t)ret;
			SENSOR_INFO("state register read value, state_reg=%d\n", state_reg);
		}
		
		if (nf == STK3X3X_PRX_NEAR_BY) {
			state_reg &= (~STK3X3X_STATE_EN_INTELL_PRST_MASK); //disable intel persistance
		} else if (nf == STK3X3X_PRX_FAR_AWAY) {
			state_reg |= (STK3X3X_STATE_EN_INTELL_PRST_MASK);  //enable intel persistance
		}
		
		SENSOR_INFO("state register write value, state_reg=%d\n", state_reg);
		ret = STK3X3X_REG_READ_MODIFY_WRITE(drv_data, STK3X3X_STATE_REG, state_reg, 0xFF);
		if (ret < 0)
			SENSOR_ERR("state register write failed, ret=%d\n", ret);
	}	
	// PS persistance adjustment
	if (nf == STK3X3X_PRX_NEAR_BY) {
		reg_value = (STK3X3X_PS_PRS2 | STK3X3X_PS_GAIN8);
		drv_data->pocket_prox = STK3X3X_POCKET_NEAR_BY;
	} else if (nf == STK3X3X_PRX_FAR_AWAY) {
		reg_value = (STK3X3X_PS_PRS4 | STK3X3X_PS_GAIN8);
		drv_data->pocket_prox = STK3X3X_POCKET_FAR_AWAY;
	}

	reg_value = (reg_value | drv_data->ps_it);

	ret = STK3X3X_REG_READ_MODIFY_WRITE(drv_data, STK3X3X_PSCTRL_REG, reg_value, 0xFF);
	if (ret < 0) {
		SENSOR_ERR("Adjust persistance failed, ret=%d\n", ret);
	}

	if (nf == STK3X3X_PRX_FAR_AWAY) { 
		if (drv_data->cal_status == STK3X3X_CAL_DISABLED) {
			SENSOR_INFO("call calibration work\n"); 
			drv_data->cal_status = STK3X3X_CAL_ONGOING;
			// control mutex already applied
			stk3x3x_prox_cal(drv_data);
		}
		if (drv_data->cal_status == STX3X3X_CAL_SKIP)
			drv_data->cal_status = STK3X3X_CAL_DISABLED;
	}
}

static void check_first_far_event(struct stk3x3x_data *drv_data) 
{
	int32_t ret = 0;
	uint8_t reg_value[2];

	ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_DATA1_PS_REG, 2, &reg_value[0]);
	if (ret < 0) {
		SENSOR_ERR("ADC read failed, ret=%d\n", ret);
		return;
	} else {
		drv_data->adc = (reg_value[0] << 8 | reg_value[1]);
	}

	SENSOR_INFO("adc=%d\n", drv_data->adc);

	if (drv_data->adc < drv_data->prox_thd_h) {
		SENSOR_INFO("First far event reported\n");
		if (drv_data->adc < drv_data->prox_thd_l && drv_data->check_far_state != 1) {
			drv_data->close_cnt = 0;
			SENSOR_ERR("stk close_cnt(%u) \n", drv_data->close_cnt);
		}
		drv_data->cal_status = STX3X3X_CAL_SKIP;
		stk_ps_report(drv_data, STK3X3X_PRX_FAR_AWAY);
	}
}

static int32_t stk3x3x_enable_ps(struct stk3x3x_data *drv_data, bool en)
{
	int32_t ret = 0;
	uint8_t reg_value = 0;

	if (drv_data->pocket_running == true) {
		SENSOR_INFO("pockek_prox cancel work\n");
		cancel_work_sync(&drv_data->work_pocket);
		drv_data->pocket_running = false;
	}

	if (drv_data->enable == en) {
		SENSOR_INFO("Prox sensor already on/off, en=%d\n", en);
		goto done;
	}

	ret = STK3X3X_REG_READ(drv_data, STK3X3X_STATE_REG);
	if (ret < 0) {
		goto done;
	} else {
		reg_value = (uint8_t)ret;
	}

	reg_value &= (~(STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK));

	if (en) {
			SENSOR_INFO("Enable ( close_cnt : %u)\n", drv_data->close_cnt);
			drv_data->pocket_prox = STK3X3X_POCKET_UNKNOWN;
			enable_irq_wake(drv_data->irq);
			enable_irq(drv_data->irq);

			stk3x3x_set_ps_thd(drv_data, drv_data->prox_thd_h, drv_data->prox_thd_l);

			reg_value |= (STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK);

			ret = STK3X3X_REG_READ_MODIFY_WRITE(drv_data, STK3X3X_STATE_REG, reg_value, 0xFF);
			if (ret < 0)
				goto done;

			ret = STK3X3X_REG_WRITE(drv_data, STK3X3X_WAIT_REG, STK3X3X_WAIT50);
			if(ret < 0)
				SENSOR_ERR("WAIT_REG failed %d\n", ret);

			usleep_range(60000, 60000);
			drv_data->check_far_state = 0;
			check_first_far_event(drv_data);
	} else {
		SENSOR_INFO("Disable (close_cnt : %u)\n", drv_data->close_cnt);
		if (drv_data->cal_status == STK3X3X_CAL_ONGOING) {
			drv_data->cal_status = STK3X3X_CAL_DISABLED;
			cancel_work_sync(&drv_data->work_cal_prox);
		}
		disable_irq(drv_data->irq);
		disable_irq_wake(drv_data->irq);

		reg_value &= (~(STK3X3X_STATE_EN_PS_MASK | STK3X3X_STATE_EN_WAIT_MASK | STK3X3X_STATE_EN_INTELL_PRST_MASK));

		ret = STK3X3X_REG_READ_MODIFY_WRITE(drv_data, STK3X3X_STATE_REG, reg_value, 0xFF);
		if (ret < 0)
			goto done;

		drv_data->adc = 0;
	}

	drv_data->enable = en;

done:

	return ret;
}

static int32_t stk3x3x_get_data_and_report(struct stk3x3x_data *drv_data, uint8_t flag)
{
	int32_t ret = 0;
	uint8_t reg_value[2];

	int nf_state = flag & STK3X3X_FLG_NF_MASK;

	if ((flag >> 6) & 0x1) {
		ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_DATA1_PS_REG, 2, &reg_value[0]);
		if (ret < 0) {
			SENSOR_ERR("ADC read failed, ret=%d\n", ret);
			return ret;
		} else {
			drv_data->adc = (reg_value[0] << 8 | reg_value[1]);
		}
	}

	SENSOR_INFO("nf_state=%d, adc=%d\n", nf_state, drv_data->adc);
	ret = stk3x3x_bgir_check(drv_data);

	if (ret < 0) {
		SENSOR_ERR("stk3x3x_bgir_check read failed, ret=%d\n", ret);
		return ret;
	} else {
		if (ret == 0x7FFF0001) { //BGIR failed
			SENSOR_ERR("stk3x3x_bgir_check failed, ret=%d\n", ret);
			if (drv_data->close_cnt > 0) {
				drv_data->close_cnt--;
				SENSOR_ERR("stk close_cnt(%u) \n", drv_data->close_cnt);
			}
			drv_data->cal_status = STX3X3X_CAL_SKIP;
			stk_ps_report(drv_data, STK3X3X_PRX_FAR_AWAY);
		} else {
			if (nf_state == STK3X3X_PRX_NEAR_BY) {
				drv_data->close_cnt++;
				if (drv_data->close_cnt >= CAL_RESET_CLOSE_CNT) {
					SENSOR_ERR("stk close_cnt(%u) threshold reset as default cur (h:%u, l:%u) default (h:%u, l:%u)\n",\
						drv_data->close_cnt, drv_data->prox_thd_h, drv_data->prox_thd_l,\
						drv_data->prox_default_thd_h, drv_data->prox_default_thd_l);
					// set as default
					drv_data->close_cnt = 0;
					drv_data->prox_thd_h = drv_data->prox_default_thd_h;
					drv_data->prox_thd_l = drv_data->prox_default_thd_l;
					stk3x3x_set_ps_thd(drv_data, drv_data->prox_thd_h, drv_data->prox_thd_l);         
				}
			} else if (nf_state == STK3X3X_PRX_FAR_AWAY) {
				drv_data->close_cnt = 0;
				SENSOR_ERR("stk close_cnt(%u) \n", drv_data->close_cnt);
			}
			stk_ps_report(drv_data, nf_state);
		}
	}

	return ret;
}

int32_t stk3x3x_get_data(struct stk3x3x_data *drv_data, uint16_t *raw_data)
{
	int32_t ret = 0;
	uint8_t flag_value;
	uint8_t reg_value[2];

	ret = STK3X3X_REG_READ(drv_data, STK3X3X_FLAG_REG);
	if (ret < 0) {
		SENSOR_ERR("STK3X3X_FLAG_REG read failed, ret=%d\n", ret);
		return ret;
	}

	flag_value = (uint8_t)ret;

	if ((flag_value >> 6) & 0x1) {
		ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_DATA1_PS_REG, 2, &reg_value[0]);
		if (ret < 0) {
			SENSOR_ERR("ADC read failed, ret=%d\n", ret);
			return ret;
		}

		*raw_data = (reg_value[0] << 8 | reg_value[1]);
		drv_data->adc =  *raw_data;
	} else {
			*raw_data = drv_data->adc;
	}

	SENSOR_INFO("adc=%d\n", drv_data->adc);

	return ret;
}

static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	uint16_t reading;

	stk3x3x_get_data(drv_data, &reading);
	return scnprintf(buf, PAGE_SIZE, "%d\n", (uint32_t)reading);
}

static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int32_t ret;
	struct stk3x3x_data *drv_data =  dev_get_drvdata(dev);
	ret = STK3X3X_REG_READ(drv_data, STK3X3X_STATE_REG);

	if (ret < 0)
		return ret;

	ret = (ret & STK3X3X_STATE_EN_PS_MASK) ? 1 : 0;
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static ssize_t stk_ps_enable_store(struct device *dev,
								   struct device_attribute *attr,
								   const char *buf,
								   size_t size)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	unsigned int en;
	int ret;

	ret = kstrtouint(buf, 10, &en);
	if (ret) {
		SENSOR_ERR("kstrtouint failed, ret=%d\n", ret);
		return ret;
	}
	
	if ((1 == en) || (0 == en)) {
		SENSOR_INFO("en=%d\n", en);
		mutex_lock(&drv_data->control_mutex);
		stk3x3x_enable_ps(drv_data, en);
		mutex_unlock(&drv_data->control_mutex);
	}

	return size;
}

static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *drv_data =  dev_get_drvdata(dev);
	int32_t nf = 1;
	int32_t ret;

	ret = STK3X3X_REG_READ(drv_data, STK3X3X_FLAG_REG);
	if (ret < 0)
		return ret;

	nf = (ret & STK3X3X_FLG_NF_MASK) ? 1 : 0;
	stk_ps_report(drv_data, nf);

	SENSOR_INFO("ps input event=%d\n", nf);

	return scnprintf(buf, PAGE_SIZE, "%d\n", nf);
}

static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);

	if (ret < 0) {
		SENSOR_ERR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}

	stk_ps_report(drv_data, value);
	SENSOR_INFO("ps input event=%d\n", (int)value);

	return size;
}

static void proximity_get_avg_val(struct stk3x3x_data *ps_data)
{
	int min = 0, max = 0, avg = 0;
	int i;
	uint32_t read_value;

	for (i = 0; i < PROX_READ_NUM; i++) {
		usleep_range(PS_WAIT, PS_WAIT);

		read_value = stk3x3x_get_ps_reading(ps_data);
		if( read_value > PROX_MIN_DATA) {
			avg += read_value;
			if (!i)
				min = read_value;
			else if (read_value < min)
				min = read_value;
			if (read_value > max)
				max = read_value;
		}else {
			read_value =PROX_MIN_DATA;
		}
	}
	avg /= PROX_READ_NUM;

	ps_data->avg[0] = min;
	ps_data->avg[1] = avg;
	ps_data->avg[2] = max;
}

static void stk3x3x_work_func_prox(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work,
		struct stk3x3x_data, work_prox);

	proximity_get_avg_val(ps_data);
}
 
static enum hrtimer_restart stk3x3x_prox_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *data = container_of(timer,
		struct stk3x3x_data, prox_timer);

	queue_work(data->prox_wq, &data->work_prox);
	hrtimer_forward_now(&data->prox_timer, data->prox_poll_delay);
	return HRTIMER_RESTART;
}

static ssize_t proximity_avg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", ps_data->avg[0],
		ps_data->avg[1], ps_data->avg[2]);
}

static ssize_t proximity_check_far_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->check_far_state);
}

static ssize_t proximity_check_far_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &ret) != 1) {
		SENSOR_ERR("invalid value\n");
		return size;
	}

	ps_data->check_far_state = ret;

	if(ret == 1)
		check_first_far_event(ps_data);

	SENSOR_INFO("check_far_state = %d\n",  ret);

	return size;
}

static ssize_t pocket_prox_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	SENSOR_INFO("stk pocket_prox = %u\n",  ps_data->pocket_prox);
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->pocket_prox);
}

static ssize_t pocket_prox_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int en;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	if (sscanf(buf, "%d", &en) != 1) {
		SENSOR_ERR("invalid value\n");
		return size;
	}

	ps_data->pocket_prox = STK3X3X_POCKET_UNKNOWN;
	if (en == 1)
		ps_data->pocket_enable = true;
	else {
		if (ps_data->pocket_running) {
			cancel_work_sync(&ps_data->work_pocket);
			ps_data->pocket_running = false;
		}
		ps_data->pocket_enable = false;
	}

	SENSOR_INFO("stk pocket_enable = %u\n",  ps_data->pocket_enable);

	return size;
}


static ssize_t proximity_avg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	bool new_value = false;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		SENSOR_ERR("invalid value\n");
		return -EINVAL;
	}

	SENSOR_INFO("average enable = %d\n",  new_value);
	if (new_value) {
		hrtimer_start(&ps_data->prox_timer, ps_data->prox_poll_delay,
			HRTIMER_MODE_REL);

	} else if (!new_value) {
		hrtimer_cancel(&ps_data->prox_timer);
		cancel_work_sync(&ps_data->work_prox);
	}

	return size;
}

static ssize_t stk_ps_code_cal_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t ps_thd[2] = {0};
	int16_t ps_thd_l = 0;
	int16_t ps_thd_h = 0;
	int ret;
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);

	ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_THDL1_PS_REG, 2, ps_thd);

	if (ret < 0) {
		SENSOR_ERR("fail, ret=0x%x", ret);
		return -EINVAL;
	}

	ps_thd_l = (int16_t)((ps_thd[0] << 8) | ps_thd[1]);

	ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_THDH1_PS_REG, 2, ps_thd);
	if (ret < 0) {
		SENSOR_ERR("failed, ret=0x%x", ret);
		return -EINVAL;
	}

	ps_thd_h = (int16_t)((ps_thd[0] << 8) | ps_thd[1]);

	return scnprintf(buf, PAGE_SIZE, "%d,%d,%d\n", drv_data->offset ,ps_thd_l, ps_thd_h);
}

static ssize_t stk_ps_code_cal_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "1")){ /* calibrate cancelation value */
		SENSOR_INFO("call calibration work\n"); 
		drv_data->cal_status = STK3X3X_FIRST_CAL;
		drv_data->factory_cal = true;
		mutex_lock(&drv_data->control_mutex);
		stk3x3x_prox_cal(drv_data);
		mutex_unlock(&drv_data->control_mutex);
	} else {
		SENSOR_ERR("invalid value %d\n", *buf);
		return size;
	}

	return size;
}

static ssize_t stk_fac_cal_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "1\n");
}

static ssize_t stk_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t ps_thd[2] = {0};
	int16_t ps_thd_l = 0;
	int ret;
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_THDL1_PS_REG, 2, ps_thd);

	if (ret < 0) {
		SENSOR_ERR("fail, ret=0x%x", ret);
		return -EINVAL;
	}

	ps_thd_l = (int16_t)((ps_thd[0] << 8) | ps_thd[1]);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l);
}


static ssize_t stk_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	unsigned long value = 0;
	uint8_t ps_thd[2] = {0};
	int ret;
	ret = kstrtoul(buf, 10, &value);

	if (ret < 0) {
		SENSOR_ERR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}

	ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_THDH1_PS_REG, 2, ps_thd);
	if (ret < 0) {
		SENSOR_ERR("Read PS THD failed, ret=0x%x\n", ret);
		return ret;
	}

	stk3x3x_set_ps_thd(drv_data, (uint16_t)((ps_thd[0] << 8) | ps_thd[1]), value);
	return size;
}

static ssize_t stk_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t ps_thd[2] = {0};
	int16_t ps_thd_h = 0;
	int ret;
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);

	ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_THDH1_PS_REG, 2, ps_thd);
	if (ret < 0) {
		SENSOR_ERR("failed, ret=0x%x", ret);
		return -EINVAL;
	}

	ps_thd_h = (int16_t)((ps_thd[0] << 8) | ps_thd[1]);

	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_h);
}

static ssize_t stk_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	unsigned long value = 0;
	unsigned char ps_thd[2] = {0};
	int ret;
	ret = kstrtoul(buf, 10, &value);

	if (ret < 0) {
		SENSOR_ERR("kstrtoul failed, ret=0x%x\n", ret);
		return ret;
	}

	ret = STK3X3X_REG_BLOCK_READ(drv_data, STK3X3X_THDL1_PS_REG, 2, ps_thd);

	if (ret < 0) {
		SENSOR_ERR("Read PS THD failed, ret=0x%x\n", ret);
		return ret;
	}

	stk3x3x_set_ps_thd(drv_data, value, ((ps_thd[0] << 8) | ps_thd[1]));

	return size;
}

static ssize_t stk_ps_code_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int16_t reg_data;
	uint8_t i = 0;
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	uint16_t reg_total = sizeof(stk3x3x_reg_map) / sizeof(stk3x3x_reg_map[0]);
	int offset = 0;

	for (i = 0; i < reg_total; i++) {
		reg_data = STK3X3X_REG_READ(drv_data, stk3x3x_reg_map[i]);

		if (reg_data < 0) {
			SENSOR_ERR("failed, ret=%d", reg_data);
			return reg_data;
		}
		else {
			SENSOR_INFO("Reg[0x%2X] = 0x%2X\n", stk3x3x_reg_map[i], (uint8_t)reg_data);
			offset += scnprintf(buf + offset, PAGE_SIZE - offset, "Reg[0x%2X] = 0x%2X\n", stk3x3x_reg_map[i], (uint8_t)reg_data);
		}
	}

	return offset;
}

static ssize_t stk_ps_code_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int reg, val;
	int ret;
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);

	if (sscanf(buf, "%2x,%2x", &reg, &val) != 2) {
		SENSOR_ERR("invalid value\n");
		return size;
	}

	ret = STK3X3X_REG_WRITE(drv_data, reg, val);
	if(ret < 0)
		SENSOR_ERR("failed %d\n", ret);
	else
		SENSOR_INFO("Register(0x%2x) data(0x%2x)\n", reg, val);

	return size;
}

static ssize_t stk_ps_code_trim_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", drv_data->offset);
}

int32_t stk3x3x_clr_flag(struct stk3x3x_data *drv_data, uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	int32_t ret;

	w_flag = org_flag_reg | STK3X3X_FLG_PSINT_MASK;
	w_flag &= (~clr);

	ret = STK3X3X_REG_WRITE(drv_data, STK3X3X_FLAG_REG, w_flag);
	if (ret < 0)
		SENSOR_ERR("STK3X3X_FLAG_REG write failed, ret=%d\n", ret);

	return ret;
}

static void stk3x3x_irq_work_func(struct work_struct *work)
{
	struct stk3x3x_data *drv_data = container_of(work, struct stk3x3x_data, prox_irq_work);

	int32_t ret;
	uint8_t prox_flag;

	mutex_lock(&drv_data->control_mutex);

	// Normally, this condition will never occur
	if (drv_data->cal_status == STK3X3X_CAL_ONGOING) {
		pr_info("%s, calibration is on going skip!!!!\n", __func__);
		goto err_i2c_rw;
	}

	ret = STK3X3X_REG_READ(drv_data, STK3X3X_FLAG_REG);
	if (ret < 0) {
		SENSOR_ERR("STK3X3X_FLAG_REG read failed, ret=%d\n", ret);
		goto err_i2c_rw;
	}

	prox_flag = (uint8_t)ret;
	SENSOR_INFO("prox_flag=%d\n", prox_flag);

	ret = stk3x3x_clr_flag(drv_data, prox_flag, (prox_flag & STK3X3X_FLG_PSINT_MASK));
	if (ret < 0) {
		SENSOR_ERR("stk3x3x_clr_flag failed, ret=%d\n", ret);
		goto err_i2c_rw;
	}

	if (prox_flag & STK3X3X_FLG_PSINT_MASK) {
		stk3x3x_get_data_and_report(drv_data, prox_flag);
	}

	usleep_range(1000, 2000);
	mutex_unlock(&drv_data->control_mutex);
	enable_irq(drv_data->irq);

	return;

err_i2c_rw:
	usleep_range(30000, 30000);
	mutex_unlock(&drv_data->control_mutex);
	enable_irq(drv_data->irq);
	return;
}

static ssize_t stk_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t stk_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_NAME);
}

static DEVICE_ATTR(psenable,          0664,   stk_ps_enable_show,         stk_ps_enable_store);
static DEVICE_ATTR(psdistance,        0664,   stk_ps_distance_show,       stk_ps_distance_store);
static DEVICE_ATTR(raw_data,          0444,   stk_ps_code_show,           NULL);
static DEVICE_ATTR(thresh_low,        0664,   stk_ps_code_thd_l_show,     stk_ps_code_thd_l_store);
static DEVICE_ATTR(thresh_high,       0664,   stk_ps_code_thd_h_show,     stk_ps_code_thd_h_store);
static DEVICE_ATTR(prox_register,     0664,   stk_ps_code_register_show,  stk_ps_code_register_store);
static DEVICE_ATTR(prox_trim,         0444,   stk_ps_code_trim_show,      NULL);
static DEVICE_ATTR(prox_cal,          0664,   stk_ps_code_cal_show,       stk_ps_code_cal_store);
static DEVICE_ATTR(prox_offset_pass,  0444,   stk_fac_cal_show,           NULL);
static DEVICE_ATTR(vendor,            0444,   stk_vendor_show,            NULL);
static DEVICE_ATTR(name,              0444,   stk_name_show,              NULL);
static DEVICE_ATTR(prox_avg,          0664,   proximity_avg_show,         proximity_avg_store);
static DEVICE_ATTR(check_far_state,   0664,   proximity_check_far_state_show,   proximity_check_far_state_store);
static DEVICE_ATTR(pocket_prox,       0664,   pocket_prox_show, pocket_prox_store);

static struct device_attribute *prox_sysfs_attrs[] = {
	&dev_attr_psenable,
	&dev_attr_psdistance,
	&dev_attr_raw_data,
	&dev_attr_thresh_low,
	&dev_attr_thresh_high,
	&dev_attr_prox_register,
	&dev_attr_prox_trim,
	&dev_attr_prox_cal,
	&dev_attr_prox_offset_pass,
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_prox_avg,
	&dev_attr_check_far_state,
	&dev_attr_pocket_prox,
	NULL
};

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	stk_ps_enable_show, stk_ps_enable_store);

static struct attribute *prox_input_attrs[] = {
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group prox_input_attribute_group = {
	.attrs = prox_input_attrs,
};

static int stk3x3x_setup_irq(struct stk3x3x_data *drv_data)
{
	int irq, ret;

	drv_data->prox_irq_wq = create_singlethread_workqueue("prox_irq_wq");
	INIT_WORK(&drv_data->prox_irq_work, stk3x3x_irq_work_func);

	ret = gpio_request(drv_data->irq_gpio, "prox_int");
	if (ret < 0) {
		SENSOR_ERR("gpio_request failed, ret=%d\n", ret);
		goto err_gpio_request;
	}

	ret = gpio_direction_input(drv_data->irq_gpio);
	if (ret < 0) {
		SENSOR_ERR("gpio_direction_input failed, ret=%d\n", ret);
		goto err_gpio_direction_input;
	}

	irq = gpio_to_irq(drv_data->irq_gpio);
	if (irq < 0) {
		SENSOR_INFO("gpio_to_irq failed, irq=%d, gpio=%d\n", irq, drv_data->irq_gpio);
		goto err_gpio_to_irq;
	}

	SENSOR_INFO("irq=%d, gpio=%d\n", irq, drv_data->irq_gpio);

	drv_data->irq = irq;

	ret = request_any_context_irq(irq, stk3x3x_irq_handler,
				IRQF_TRIGGER_FALLING, "prox_irq", drv_data);
	if (ret < 0) {
		SENSOR_INFO("request_any_context_irq failed, ret=%d\n", ret);
		goto err_request_any_context_irq;
	}

	disable_irq(irq);

	SENSOR_INFO("Success\n");

	return 0;

err_request_any_context_irq:
err_gpio_to_irq:
err_gpio_direction_input:
	gpio_free(drv_data->irq_gpio);
err_gpio_request:
	destroy_workqueue(drv_data->prox_irq_wq);

	return ret;
}

static int32_t stk3x3x_init_registers(struct stk3x3x_data *drv_data)
{
	int32_t ret;
	u16 reg_count = 0;
	u16 reg_num = sizeof(stk3x3x_default_register_table) / sizeof(stk3x3x_register_table);

	ret = stk3x3x_software_reset(drv_data);
	if (ret < 0)
		return ret;

	for(reg_count = 0; reg_count < reg_num; reg_count++) {
		if (stk3x3x_default_register_table[reg_count].address == STK3X3X_PSCTRL_REG) {
			if (drv_data->ps_it == 0xff) {
				drv_data->ps_it = STK3X3X_PS_IT400;
			} else {
				stk3x3x_default_register_table[reg_count].value = (STK3X3X_PS_PRS4 | STK3X3X_PS_GAIN8 | drv_data->ps_it);
			}
			SENSOR_INFO("PS_IT = %d\n", drv_data->ps_it);
		}
		ret = STK3X3X_REG_READ_MODIFY_WRITE(drv_data,
										stk3x3x_default_register_table[reg_count].address,
										stk3x3x_default_register_table[reg_count].value,
										stk3x3x_default_register_table[reg_count].mask_bit);
		if (ret < 0)
			return ret;
	}

	ret = stk3x3x_prst_cnt_rst_sel(drv_data);
	if (ret < 0)
		return ret;

	SENSOR_INFO("success\n");

	return 0;
}

int stk3x3x_suspend(struct device *dev)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	SENSOR_INFO("\n");

	if (drv_data->pocket_running == true) {
		cancel_work_sync(&drv_data->work_pocket);
		drv_data->pocket_running = false;
	}

	return 0;
}

int stk3x3x_resume(struct device *dev)
{
	struct stk3x3x_data *drv_data = dev_get_drvdata(dev);
	SENSOR_INFO("\n");
	if (drv_data->enable == false && drv_data->pocket_enable == true) {
		drv_data->pocket_prox = STK3X3X_POCKET_UNKNOWN;
		drv_data->pocket_running = true;
		queue_work(drv_data->prox_pocket_wq, &drv_data->work_pocket);
	}

	return 0;
}

static int stk3x3x_parse_dt(struct device *dev, struct stk3x3x_data *drv_data)
{
	struct device_node *np = dev->of_node;
	u32 temp_val;
	int ret;

	ret = of_get_named_gpio_flags(np, "stk,gpio_int", 0, NULL);
	if (ret < 0) {
		SENSOR_ERR("stk,gpio_int read failed, ret=%d\n", ret);
		return ret;
	} else {
		drv_data->irq_gpio = ret;
		SENSOR_INFO("irq_gpio=%d\n", drv_data->irq_gpio);
	}

	ret = of_property_read_u32(np, "stk,prox_thd_h", &temp_val);
	if (ret < 0) {
		SENSOR_ERR("stk,prox_thd_h read failed, ret=%d\n", ret);
		return ret;
	} else {
		drv_data->prox_default_thd_h = drv_data->prox_thd_h = (u16) temp_val;
		SENSOR_INFO("prox_thd_h=%d\n", drv_data->prox_thd_h);
	}

	ret = of_property_read_u32(np, "stk,prox_thd_l", &temp_val);
	if (ret < 0) {
		SENSOR_ERR("stk,prox_thd_l read failed, ret=%d\n", ret);
		return ret;
	} else {
		drv_data->prox_default_thd_l = drv_data->prox_thd_l = (u16) temp_val;
		SENSOR_INFO("prox_thd_l=%d\n", drv_data->prox_thd_l);
	}

	ret = of_property_read_u32(np, "stk,thd_h_offset", &temp_val);
	if (ret < 0) {
		drv_data->thd_h_offset = 40;
		SENSOR_ERR("stk,thd_h_offset read failed, ret=%d\n", ret);
	} else {
		drv_data->thd_h_offset = (u16) temp_val;
		SENSOR_INFO("thd_h_offset=%d\n", drv_data->thd_h_offset);
	}

	ret = of_property_read_u32(np, "stk,sunlight_thd_h", &temp_val);
	if (ret < 0) {
		drv_data->sunlight_thd_h = PDATA_MAX;
		SENSOR_INFO("stk,sunlight_thd_h read failed, ret=%d\n", ret);
	} else {
		drv_data->sunlight_thd_h = (u16) temp_val;
		SENSOR_INFO("sunlight_thd_h=%d\n", drv_data->sunlight_thd_h);
	}

	ret = of_property_read_u32(np, "stk,sunlight_thd_l", &temp_val);
	if (ret < 0) {
		drv_data->sunlight_thd_l = PDATA_MAX;
		SENSOR_INFO("stk,sunlight_thd_l read failed, ret=%d\n", ret);
	} else {
		drv_data->sunlight_thd_l = (u16) temp_val;
		SENSOR_INFO("sunlight_thd_l=%d\n", drv_data->sunlight_thd_l);
	}

	ret = of_property_read_u32(np, "stk,first_cal_adc_limit", &temp_val);
	if (ret < 0) {
		drv_data->first_cal_adc_limit = FIRST_CAL_ADC_LIMIT;
		SENSOR_INFO("stk,first_cal_adc_limit read failed, ret=%d\n", ret);
	} else {
		drv_data->first_cal_adc_limit = (u16) temp_val;
		SENSOR_INFO("first_cal_adc_limit=%d\n", drv_data->first_cal_adc_limit);
	}

	ret = of_property_read_u32(np, "stk,first_cal_thd_h", &temp_val);
	if (ret < 0) {
		drv_data->first_cal_thd_h = PDATA_MAX;
		SENSOR_INFO("stk,first_cal_thd_h read failed, ret=%d\n", ret);
	} else {
		drv_data->first_cal_thd_h = (u16) temp_val;
		SENSOR_INFO("first_cal_thd_h=%d\n", drv_data->first_cal_thd_h);
	}

	ret = of_property_read_u32(np, "stk,first_cal_thd_l", &temp_val);
	if (ret < 0) {
		drv_data->first_cal_thd_l = PDATA_MAX;
		SENSOR_INFO("stk,first_cal_thd_l read failed, ret=%d\n", ret);
	} else {
		drv_data->first_cal_thd_l = (u16) temp_val;
		SENSOR_INFO("first_cal_thd_l=%d\n", drv_data->first_cal_thd_l);
	}

	if (drv_data->prox_thd_h == 0 && drv_data->prox_thd_l == 0)
			drv_data->first_limit_skip = true;

	ret = of_property_read_u32(np, "stk,intel_prst", &temp_val);
	if (ret < 0) {
		drv_data->intel_prst = 0x01;
		SENSOR_ERR("stk,intel_prst read failed, ret=%d prst=%d", ret , drv_data->intel_prst);
	} else {
		drv_data->intel_prst = (u8) temp_val;
		SENSOR_INFO("intel_prst=%d\n", drv_data->intel_prst);
	}

	ret = of_property_read_u32(np, "stk,ps_it", &temp_val);
	if (ret < 0) {
		drv_data->ps_it = 0xff;
		SENSOR_ERR("stk,ps_it read failed, ret=%d ps_it=%d\n", ret, drv_data->ps_it);
	} else {
		drv_data->ps_it = (u8) temp_val;
		SENSOR_INFO("stk,ps_it=%d\n", drv_data->ps_it);
	}

	return 0;
}

static int stk3x3x_init_input_device(struct stk3x3x_data *drv_data)
{
	int ret = 0;
	struct input_dev *dev;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = MODULE_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_MISC);
	input_set_drvdata(dev, drv_data);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		goto err_register_input_dev;
	}

	ret = sensors_create_symlink(&dev->dev.kobj, dev->name);
	if (ret < 0)
		goto err_create_sensor_symlink;

	ret = sysfs_create_group(&dev->dev.kobj, &prox_input_attribute_group);
	if (ret < 0)
		goto err_create_sysfs_group;

	drv_data->prox_input_dev = dev;
	return 0;

err_create_sysfs_group:
	sensors_remove_symlink(&dev->dev.kobj, dev->name);
err_create_sensor_symlink:
	input_unregister_device(dev);
err_register_input_dev:
	dev = NULL;
	return ret;
}

static const struct stk3x3x_bus_ops stk3x3x_i2c_bops =
{
	.bustype            = BUS_I2C,
	.write              = stk3x3x_reg_write,
	.write_block        = stk3x3x_reg_write_block,
	.read               = stk3x3x_reg_read,
	.read_block         = stk3x3x_reg_read_block,
	.read_modify_write  = stk3x3x_reg_read_modify_write,
};

int stk3x3x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stk3x3x_data *drv_data;
	int ret = 0;

	SENSOR_INFO("probe called\n");

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!ret) {
		SENSOR_ERR("i2c_check_functionality  failed\n");
		return ret;
	}

	drv_data = kzalloc(sizeof(struct stk3x3x_data), GFP_KERNEL);
	if (!drv_data) {
		SENSOR_ERR("drv_data memory allocation failed\n");
		return -ENOMEM;
	}

	drv_data->client = client;
	drv_data->dev    = &client->dev;
	drv_data->bops   = &stk3x3x_i2c_bops;

	i2c_set_clientdata(client, drv_data);

	// Check if STK3331 IC exists
	ret = stk3x3x_device_id_check(drv_data);
	if (ret) {
		SENSOR_ERR("STK3331 not found, ret=%d\n", ret);
		goto err_stk3x3x_device_id_check;
	}

	mutex_init(&drv_data->control_mutex);
	wake_lock_init(&drv_data->prox_wakelock, WAKE_LOCK_SUSPEND, "prox_wakelock");
	device_init_wakeup(&client->dev, true);

	drv_data->first_limit_skip = false;
	ret = stk3x3x_parse_dt(&client->dev, drv_data);
	if (ret) {
		SENSOR_ERR("stk3x3x_parse_dt failed, ret=%d\n", ret);
		goto err_stk3x3x_parse_dt;
	}

	ret = stk3x3x_init_registers(drv_data);
	if (ret < 0) {
		SENSOR_ERR("stk3x3x_init_registers failed, ret=%d\n", ret);
		goto err_stk3x3x_init_registers;
	}

	ret = stk3x3x_init_input_device(drv_data);
	if (ret < 0) {
		SENSOR_ERR("stk3x3x_init_input_device failed, ret=%d\n", ret);
		goto err_stk3x3x_init_input_device;
	}

	ret = sensors_register(&drv_data->dev, drv_data, prox_sysfs_attrs, MODULE_NAME);
	if (ret < 0) {
		SENSOR_ERR("sensors_register failed, ret=%d\n", ret);
		goto err_sensors_register;
	}
	/* For factory test mode, we use timer to get average proximity data. */
	/* prox_timer settings. we poll for light values using a timer. */
	hrtimer_init(&drv_data->prox_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	drv_data->prox_poll_delay = ns_to_ktime(2000 * NSEC_PER_MSEC);/*2 sec*/
	drv_data->prox_timer.function = stk3x3x_prox_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	to read the i2c (can be slow and blocking). */
	drv_data->prox_wq = create_singlethread_workqueue("stk3x3x_prox_wq");
	if (!drv_data->prox_wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create prox workqueue\n");
		goto err_create_prox_workqueue;
	}
	INIT_WORK(&drv_data->work_prox, stk3x3x_work_func_prox);

	drv_data->prox_cal_wq = create_singlethread_workqueue("stk3x3x_prox_cal_wq");
	if (!drv_data->prox_cal_wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create prox cal workqueue\n");
		goto err_create_prox_cal_workqueue;
	}

	drv_data->prox_pocket_wq = create_singlethread_workqueue("stk3x3x_prox_pocket");
	if (!drv_data->prox_pocket_wq) {
		ret = -ENOMEM;
		SENSOR_ERR("could not create prox_pocket workqueue\n");
		goto err_create_prox_pocket_workqueue;
	}

	INIT_WORK(&drv_data->work_cal_prox, stk3x3x_work_func_prox_cal);
	INIT_WORK(&drv_data->work_pocket, stk3x3x_work_func_pocket_read);

	ret = stk3x3x_setup_irq(drv_data);
	if (ret < 0) {
		SENSOR_ERR("stk3x3x_setup_irq failed, ret=%d\n", ret);
		goto err_stk3x3x_setup_irq;
	}
	drv_data->cal_status = STK3X3X_FIRST_CAL;
	drv_data->factory_cal = false;
	drv_data->pocket_running = false;
	drv_data->pocket_enable = false;
	SENSOR_INFO("call calibration work\n");
	queue_work(drv_data->prox_cal_wq, &drv_data->work_cal_prox);
	drv_data->check_far_state = 0;
	drv_data->close_cnt = 0;
	drv_data->pocket_prox = STK3X3X_POCKET_UNKNOWN;

	SENSOR_INFO("probe done\n");

	return 0;

err_stk3x3x_setup_irq:
	destroy_workqueue(drv_data->prox_pocket_wq);
err_create_prox_pocket_workqueue:
	destroy_workqueue(drv_data->prox_cal_wq);
err_create_prox_cal_workqueue:    
	destroy_workqueue(drv_data->prox_wq);
err_create_prox_workqueue:
	sensors_unregister(drv_data->dev, prox_sysfs_attrs);
err_sensors_register:
		sensors_remove_symlink(&drv_data->prox_input_dev->dev.kobj, drv_data->prox_input_dev->name);
		sysfs_remove_group(&drv_data->prox_input_dev->dev.kobj, &prox_input_attribute_group);
		input_unregister_device(drv_data->prox_input_dev);
err_stk3x3x_init_input_device:
err_stk3x3x_init_registers:
err_stk3x3x_parse_dt:
	wake_lock_destroy(&drv_data->prox_wakelock);
	mutex_destroy(&drv_data->control_mutex);
err_stk3x3x_device_id_check:
	kfree(drv_data);

	return ret;
}

int stk3x3x_remove(struct i2c_client *client)
{
	struct stk3x3x_data *drv_data = i2c_get_clientdata(client);

	stk3x3x_enable_ps(drv_data, 0);

	free_irq(drv_data->irq, drv_data);
	gpio_free(drv_data->irq_gpio);

	if (drv_data->cal_status != STK3X3X_CAL_DISABLED)
		cancel_work_sync(&drv_data->work_cal_prox);
	destroy_workqueue(drv_data->prox_cal_wq);
	destroy_workqueue(drv_data->prox_irq_wq);
    if (drv_data->pocket_running)
		cancel_work_sync(&drv_data->work_pocket);
	destroy_workqueue(drv_data->prox_pocket_wq);

	sensors_unregister(drv_data->dev, prox_sysfs_attrs);
	destroy_workqueue(drv_data->prox_wq);

	sensors_remove_symlink(&drv_data->prox_input_dev->dev.kobj, drv_data->prox_input_dev->name);
	sysfs_remove_group(&drv_data->prox_input_dev->dev.kobj, &prox_input_attribute_group);
	input_unregister_device(drv_data->prox_input_dev);

	wake_lock_destroy(&drv_data->prox_wakelock);
	mutex_destroy(&drv_data->control_mutex);

	kfree(drv_data);

	return 0;
}

static void stk3x3x_shutdown(struct i2c_client *client)
{
	struct stk3x3x_data *drv_data = i2c_get_clientdata(client);

	mutex_lock(&drv_data->control_mutex);
	stk3x3x_enable_ps(drv_data, 0);
	mutex_unlock(&drv_data->control_mutex);
}

static const struct dev_pm_ops stk3x3x_pm_ops = {
	.suspend = stk3x3x_suspend,
	.resume = stk3x3x_resume,
};

static const struct i2c_device_id stk3x3x_ps_id[] =
{
	{CHIP_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, stk3x3x_ps_id);

static struct of_device_id stk3x3x_match_table[] =
{
	{.compatible = "stk,stk3x3x",},
	{},
};

static struct i2c_driver stk3x3x_ps_driver =
{
	.probe = stk3x3x_probe,
	.remove = stk3x3x_remove,
	.shutdown = stk3x3x_shutdown,
	.id_table = stk3x3x_ps_id,
	.driver = {
		.name = CHIP_NAME,
		.owner = THIS_MODULE,
		.of_match_table = stk3x3x_match_table,
		.pm = &stk3x3x_pm_ops,
	},
};

static int __init stk3x3x_init(void)
{
	SENSOR_INFO("\n");
	return i2c_add_driver(&stk3x3x_ps_driver);
}

static void __exit stk3x3x_exit(void)
{
	SENSOR_INFO("\n");
	i2c_del_driver(&stk3x3x_ps_driver);
}

module_init(stk3x3x_init);
module_exit(stk3x3x_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("Proximity sensor driver for sensortek stk3x3x IC");
MODULE_LICENSE("GPL");