/*
* STMicroelectronics lps22hb driver
*
* Copyright 2016 STMicroelectronics Inc.
*
* Authors: HESA BU - Application Team
*        : Adalberto Muhuho (adalberto.muhuho@st.com)
*        : Mario Tesi (mario.tesi@st.com)
* 
* The structure of this driver is based on reference code previously
* delivered by Lorenzo Sarchi
*
* Version: 0.0.3
* Date: 2016/May/16
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
*
* Read pressures and temperatures output can be converted in units of
* measurement by dividing them respectively for SENSITIVITY_P and SENSITIVITY_T.
* Temperature values must then be added by the constant float TEMPERATURE_OFFSET
* expressed as Celsius degrees.
*
* Obtained values are then expessed as
* mbar (=0.1 kPa) and Celsius degrees.
*
*/
/******************************************************************************
 Revision history:

 Revision 0.0.1 2015/Dec/11: 1st beta version

 Revision 0.0.2 2016/Apr/19:
	Revision 0.0.2 downgrades previous License to GPLv2

 Revision 0.0.2 2016/May/16:
	Added SPI support
******************************************************************************/

#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/pm.h>
#include <linux/sensor/sensors_core.h>
#include "lps22hb.h"

#define	PR_ABS_MAX	8388607		/* 24 bit 2'compl */
#define	PR_ABS_MIN	-8388608
#ifdef SHRT_MAX
#define	TEMP_MAX	SHRT_MAX
#define TEMP_MIN	SHRT_MIN
#else
#define	TEMP_MAX	SHORT_MAX
#define TEMP_MIN	SHORT_MIN
#endif
#define VENDOR_NAME                     "STM"
#define CHIP_NAME                       "LPS22HB"
#define MODULE_NAME                     "barometer_sensor"

static int sea_level_pressure;
static int pressure_cal;
/* Device ID */
#define	WHOAMI_LPS22_PRS	0xB1

/*	REGISTERS */
#define	INT_CFG_REG		0x0B	/* interrupt config reg */
#define	THS_P_L			0x0C	/* pressure threshold */
#define	THS_P_H			0x0D	/* pressure threshold */
#define	WHO_AM_I		0x0F	/* Device ID register */
#define	CTRL_REG1		0x10	/* Control register 1 */
#define	CTRL_REG2		0x11	/* Control register 2 */
#define	CTRL_REG3		0x12	/* Control register 3 */
#define	FIFO_CTRL		0x14	/* Fifo control register */
#define	REF_P_XL		0x15	/* pressure reference */
#define	REF_P_L			0x16	/* pressure reference */
#define	REF_P_H			0x17	/* pressure reference */
#define	RPDS_TRM_L		0x18	/* NEW */
#define	RPDS_TRM_H		0x19	/* NEW */
#define	RESOL_CONF		0x1A	/* Resolution configuration */
#define	CTRL_REG4		0x23	/* Control register 4 */
#define	INT_SRC_REG		0x25	/* interrupt source reg	*/
#define	FIFO_STATUS		0x26	/* Fifo Status reg */
#define	STATUS_REG		0X27	/* Status reg */
#define	PRESS_OUT_XL		0x28	/* press output (3 regs) */
#define	TEMP_OUT_L		0x2B	/* temper output (2 regs) */

/*	REGISTERS ALIASES	*/
#define	P_REF_INDATA_REG	REF_P_XL
#define	P_THS_INDATA_REG	THS_P_L
#define	P_OUTDATA_REG		PRESS_OUT_XL
#define	T_OUTDATA_REG		TEMP_OUT_L
#define	OUTDATA_REG		PRESS_OUT_XL

/* Bitmasks */
#define	ODR_MASK		0x70
#define	DIFF_MASK		0x08
#define	BDU_MASK		0x02
#define	RESET_AZ_MASK		0x10
#define	LC_EN_MASK   		0x01
#define EN_LPFP_MASK    	0x08
#define LPF_CFG_MASK    	0x04
#define RESET_ARP_MASK  	0x40
#define DIFF_EN_MASK    	0x08
#define PLE_MASK		0x02
#define PHE_MASK		0x01
#define FIFO_EN_MASK    	0x40
#define FIFO_MODE_MASK		0xE0
#define FIFO_SAMPLE_MASK	0x1F
#define	AUTOZ_MASK		0x20
#define	AUTOZ_OFF		0x00
#define	AUTORIFP_MASK		0x80
#define STOP_ON_FTH_MASK	0x20
#define SW_RESET_MASK		0x04

/* Barometer and Termometer output data rate ODR */
#define	ODR_ONESH	0x00	/* one shot both */
#define	ODR_1_1		0x10	/*  1  Hz baro,  1  Hz term ODR	*/
#define	ODR_10_10	0x20	/* 10  Hz baro, 10  Hz term ODR	*/
#define	ODR_25_25	0x30	/* 25  Hz baro, 25  Hz term ODR	*/
#define	ODR_50_50	0x40	/* 50  Hz baro, 50  Hz term ODR	*/
#define	ODR_75_75	0x50	/* 75  Hz baro, 75  Hz term ODR	*/

/* Additional operating modes defines */
#define	AUTOZ_ENABLE	1
#define	AUTOZ_DISABLE	0
#define RES_MAX 	0
#define	FUZZ		0
#define	FLAT		0

/* RESUME STATE INDICES */
#define	RES_REF_P_XL	0
#define	RES_REF_P_L	1
#define	RES_REF_P_H	2
#define	RES_REFT_L	3
#define	RES_REFT_H	4
#define	RES_RESOL_CONF	5
#define	RES_CTRL_REG1	6
#define	RES_CTRL_REG2	7
#define	RES_CTRL_REG3	8
#define	RES_CTRL_REG4	9
#define	RES_INT_CFG_REG	10
#define	RES_FIFO_CTRL	11
#define	RES_THS_P_L	12
#define	RES_THS_P_H	13
#define	RES_RPSD_TRIM_L	14
#define	RES_RPSD_TRIM_H	15

#define SELFTEST_READ_TIMES 5
enum {
	FAIL = 0,
	PASS = 1,
};

/* end RESUME STATE INDICES */
#ifdef DEBUG
static u8 decimator_count;
static u8 logout_decimation = 1;
static u8 hex_measr_logging;
#endif

static const struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lps22_prs_odr_table[] = {
	{ 13, ODR_75_75 },
	{ 20, ODR_50_50 },
	{ 40, ODR_25_25 },
	{ 100, ODR_10_10 },
	{ 1000, ODR_1_1 },
};

struct outputdata {
	s32 press;
	s16 temperature;
};

static const struct lps22_prs_platform_data default_lps22_pdata = {
	.poll_interval = 1000,
	.min_interval = LPS22_PRS_MIN_POLL_PERIOD_MS,
};

static u8 snsdata[3];

static int lps22_prs_hw_init(struct lps22_prs_data *prs)
{
	int err;
	u8 buf[6];

	SENSOR_INFO("%s: hw init start\n", LPS22_PRS_DEV_NAME);
	dev_dbg(prs->dev,"%s: hw init start\n", LPS22_PRS_DEV_NAME);

	buf[0] = prs->resume_state[RES_REF_P_XL];
	buf[1] = prs->resume_state[RES_REF_P_L];
	buf[2] = prs->resume_state[RES_REF_P_H];

	err = prs->tf->write(prs, P_REF_INDATA_REG, 3, buf);
	if (err < 0)
		goto err_resume_state;
		
	SENSOR_INFO("hw_init: REF_P pass \r\n");
	buf[0] = prs->resume_state[RES_RESOL_CONF];
	err = prs->tf->write(prs, RESOL_CONF, 1, buf);
	if (err < 0)
		goto err_resume_state;

	SENSOR_INFO("hw_init: RESOL_CONF pass \r\n");
	buf[0] = prs->resume_state[RES_THS_P_L];
	buf[1] = prs->resume_state[RES_THS_P_H];
	err = prs->tf->write(prs, P_THS_INDATA_REG, 2, buf);
	if (err < 0)
		goto err_resume_state;
#ifdef DEBUG
	SENSOR_INFO("hw_init: P_THS_INDATA_REG pass \r\n");
#endif

 	buf[0] = (prs->resume_state[RES_CTRL_REG2]) | 0x10;
	buf[1] = prs->resume_state[RES_CTRL_REG3];

	err = prs->tf->write(prs, CTRL_REG2, 2, buf);
	if (err < 0)
		goto err_resume_state;
#ifdef DEBUG
	SENSOR_INFO("hw_init: P_CTRL_REGS23 pass \r\n");
#endif

	buf[0] = prs->resume_state[RES_INT_CFG_REG];
	err = prs->tf->write(prs, INT_CFG_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;
#ifdef DEBUG
	SENSOR_INFO("hw_init: INT_CFG_REG pass \r\n");
#endif

	buf[0] = prs->resume_state[RES_CTRL_REG1];
	err = prs->tf->write(prs, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;
#ifdef DEBUG
	SENSOR_INFO("hw_init: CTRL_REG1 pass \r\n");
#endif

	prs->hw_initialized = 1;

	SENSOR_INFO("%s: hw init done\n", LPS22_PRS_DEV_NAME);
	dev_dbg(prs->dev, "%s: hw init done\n", LPS22_PRS_DEV_NAME);

	return 0;

err_resume_state:
	prs->hw_initialized = 0;
	SENSOR_ERR("hw init error 0x%02x,0x%02x: %d\n", buf[0],
		buf[1], err);
	return err;
}

static void lps22_prs_device_power_off(struct lps22_prs_data *prs)
{
	int err;
	u8 buf[5];

	// Power Down
	buf[0] = 0x00;
	err = prs->tf->write(prs, CTRL_REG1, 1, buf);
	if (err < 0)
		dev_err(prs->dev, "soft power off failed: %d\n", err);

	// Sofware Reset
	buf[0] = SW_RESET_MASK;
	err = prs->tf->write(prs, CTRL_REG2, 1, buf);
	if (err < 0)
		dev_err(prs->dev, "soft power off failed: %d\n", err);

	// Give time to exit reset on the device (1us target, 2us upper bound)
	udelay(2);

	// Restart from consistent status
	err = prs->tf->read(prs, INT_SRC_REG, 1, buf);
	if (err < 0)
		dev_err(prs->dev, "status reset 1 failed: %d\n", err);

	// Dummy measurement read for status_reg reset and update blocking due to BDU
	err = prs->tf->read(prs, P_REF_INDATA_REG, 5, buf);
	if (err < 0)
		dev_err(prs->dev, "status reset 2 failed: %d\n", err);

	if (prs->pdata->power_off)
		prs->pdata->power_off();

	prs->hw_initialized = 0;
}

int lps22_prs_update_odr(struct lps22_prs_data *prs, int poll_period_ms)
{
	int err = -1;
	u8 buf[2];
	u8 init_val, updated_val;
	u8 curr_val, new_val;
#ifdef DEBUG
	int i;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (longest period) backward (shortest
	 * period), to support the poll_interval requested by the system.
	 * It must be the longest period shorter then the set poll period.*/
	for (i = ARRAY_SIZE(lps22_prs_odr_table) - 1; i >= 0; i--) {

		SENSOR_INFO("poll period tab index %d \r\n",i);
		SENSOR_INFO("poll period tab cutoff %d \r\n",lps22_prs_odr_table[i].cutoff_ms);
		SENSOR_INFO("poll period tab mask %02x \r\n",lps22_prs_odr_table[i].mask);

		if ((lps22_prs_odr_table[i].cutoff_ms <= poll_period_ms) ||
		    (i == 0))
			break;
	}
	SENSOR_INFO("\r\n");
	SENSOR_INFO("new poll period setting: %d \r\n",poll_period_ms);
#endif

	new_val = lps22_prs_odr_table[3].mask;

#ifdef DEBUG
	SENSOR_INFO("new ODR bits: %02x \r\n",new_val);
#endif

	/* before to change the ODR it is mandatory to power down
	the device */
	err = prs->tf->read(prs, CTRL_REG1, 1, buf);
	if (err < 0)
		goto error;
	/* work on all but ENABLE bits */
	/* power down */
	init_val = buf[0];
	prs->resume_state[RES_CTRL_REG1] = init_val ;
	curr_val = init_val & 0x0F;

	buf[0] = curr_val;
	err = prs->tf->write(prs, CTRL_REG1, 1, buf);
	if (err < 0)
		goto error;

	/* set new ODR*/
	buf[0] = CTRL_REG1;
	updated_val = ((ODR_MASK & new_val) | ((~ODR_MASK) & curr_val) | BDU_MASK);

	err = prs->tf->write(prs, CTRL_REG1, 1, &updated_val);
	if (err < 0)
		goto error;

	prs->resume_state[RES_CTRL_REG1] = updated_val;

	 /* force set to 180ms */
	prs->delta_ts = ktime_set(0, 1000000 * 180);

	return err;

error:
	SENSOR_ERR("update odr failed 0x%02x,0x%02x: %d\n",
		buf[0], buf[1], err);

	return err;
}

static int lps22_prs_device_power_on(struct lps22_prs_data *prs)
{
	int err = -1;

	if (prs->pdata->power_on) {
		err = prs->pdata->power_on();
		if (err < 0) {
			SENSOR_ERR("power_on failed: %d\n", err);
			return err;
		}
	}

	if (!prs->hw_initialized) {
		err = lps22_prs_hw_init(prs);
		lps22_prs_update_odr(prs, prs->pdata->poll_interval);
		if (prs->hw_working == 1 && err < 0) {
			lps22_prs_device_power_off(prs);
			return err;
		}
	}

	return 0;
}

static int lps22_prs_set_press_reference(struct lps22_prs_data *prs,
					 s32 new_reference)
{
	int err;
	u8 bit_valuesXL,bit_valuesL, bit_valuesH;
	u8 buf[4];

	bit_valuesXL = (u8) (new_reference & 0x0000FF);
	bit_valuesL = (u8)((new_reference & 0x00FF00) >> 8);
	bit_valuesH = (u8)((new_reference & 0xFF0000) >> 16);

	buf[0] = bit_valuesXL;
	buf[1] = bit_valuesL;
	buf[2] = bit_valuesH;

	err = prs->tf->write(prs, P_REF_INDATA_REG, 3, buf);
	if (err < 0)
		return err;

	prs->resume_state[RES_REF_P_XL] = bit_valuesXL;
	prs->resume_state[RES_REF_P_L] = bit_valuesL;
	prs->resume_state[RES_REF_P_H] = bit_valuesH;

#ifdef DEBUG
	SENSOR_INFO("LPS22HB new reference pressure setting : %d \r\n",
	       (((u32)bit_valuesH) << 16) +
	       (((u32)bit_valuesL) << 8) + ((u32)(bit_valuesXL)));
#endif

	return err;
}

static int lps22_prs_get_press_reference(struct lps22_prs_data *prs,
					 s32 *buf32)
{
	int err;
	u8 bit_valuesXL, bit_valuesL, bit_valuesH;
	u8 buf[3];
	u16 temp = 0;

	err = prs->tf->read(prs, P_REF_INDATA_REG, 3, buf);
	if (err < 0)
		return err;
	bit_valuesXL = buf[0];
	bit_valuesL = buf[1];
	bit_valuesH = buf[2];


	temp = (bit_valuesH << 8 ) | bit_valuesL;
	*buf32 = (s32)((((s16)temp) << 8) | bit_valuesXL);
#ifdef DEBUG
	SENSOR_INFO("%s val: %+d", LPS22_PRS_DEV_NAME, *buf32 );
#endif
	return err;
}

static int lps22_prs_get_presstemp_data(struct lps22_prs_data *prs,
					struct outputdata *out)
{
	int err;
	/* Data bytes from hardware PRESS_OUT_XL,PRESS_OUT_L,PRESS_OUT_H,
	   TEMP_OUT_L, TEMP_OUT_H */
	u8 prs_data[5];
	s32 pressure;
	s16 temperature;

	err = prs->tf->read(prs, OUTDATA_REG, 5, prs_data);
	if (err < 0)
		return err;

#ifdef DEBUG
    if (hex_measr_logging)
		SENSOR_INFO("temp out tH = 0x%02x, tL = 0x%02x,"
			"press_out: pH = 0x%02x, pL = 0x%02x, pXL= 0x%02x\n",
			prs_data[4], prs_data[3], prs_data[2], prs_data[1],
			prs_data[0]);
#endif

	pressure = (s32)((((s8)prs_data[2]) << 16) | (prs_data[1] << 8) |
			 prs_data[0]);
	temperature = (s16)((((s8)prs_data[4]) << 8) | prs_data[3]);

#ifdef DEBUG
	if ((decimator_count%logout_decimation) == 0)
		SENSOR_INFO("%d \n", (int32_t)pressure);
	decimator_count++;
#endif

	out->press = pressure;
	out->temperature = (temperature)/100;

	return err;
}

static void lps22_prs_report_values(struct lps22_prs_data *prs,
				    struct outputdata *out)
{
        int press_data = out->press - pressure_cal;
	input_event(prs->input_dev_pres, INPUT_EVENT_TYPE, INPUT_EVENT_X, press_data);
	//input_event(prs->input_dev_pres, INPUT_EVENT_TYPE, INPUT_EVENT_Y, out->temperature);
	input_sync(prs->input_dev_pres);

}

static int lps22_prs_enable(struct lps22_prs_data *prs)
{
	int err;

	err = lps22_prs_device_power_on(prs);
	if (err < 0) {
		SENSOR_INFO("power on err");
		return err;
	}
	hrtimer_start(&prs->hr_timer, prs->delta_ts, HRTIMER_MODE_REL);

	return 0;
}

static int lps22_prs_disable(struct lps22_prs_data *prs)
{
	hrtimer_cancel(&prs->hr_timer);
	lps22_prs_device_power_off(prs);

	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);

	mutex_lock(&prs->lock);
	val = prs->pdata->poll_interval;
	mutex_unlock(&prs->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max((unsigned int)interval_ms,prs->pdata->min_interval);

	mutex_lock(&prs->lock);
	prs->pdata->poll_interval = interval_ms;
	lps22_prs_update_odr(prs, interval_ms);
	mutex_unlock(&prs->lock);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int val = atomic_read(&prs->enabled);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int pre_enable = atomic_read(&prs->enabled);
	unsigned long val;

#ifdef DEBUG
	SENSOR_INFO("%s Value= %s", LPS22_PRS_DEV_NAME, buf);
#endif

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

#ifdef DEBUG
	SENSOR_INFO("%s Valid val: %lu ", LPS22_PRS_DEV_NAME, val);
#endif

	if (val) {
		if (pre_enable == 0) {   
			lps22_prs_enable(prs);
			atomic_set(&prs->enabled, 1);
		}
	} else {
		if (pre_enable == 1) {
			lps22_prs_disable(prs);
			atomic_set(&prs->enabled, 0);
		}
	}
	return size;
}

static ssize_t attr_get_press_ref(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	s32 val = 0;

	mutex_lock(&prs->lock);
	err = lps22_prs_get_press_reference(prs, &val);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_press_ref(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	int err = -1;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	long val = 0;

	if (kstrtol(buf, 10, &val))
		return -EINVAL;

	if (val < PR_ABS_MIN || val > PR_ABS_MAX)
		return -EINVAL;

	mutex_lock(&prs->lock);
	err = lps22_prs_set_press_reference(prs, val);
	mutex_unlock(&prs->lock);

	if (err < 0)
		return err;
	return size;
}

static ssize_t attr_set_autozero(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val, updated_val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, INT_CFG_REG, 1, &init_val);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	prs->resume_state[RES_INT_CFG_REG] = init_val;
	updated_val = ((AUTOZ_MASK & (((u8)val) << 5)) |
		       ((~AUTOZ_MASK) & init_val));
	err = prs->tf->write(prs, INT_CFG_REG, 1, &updated_val);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit :
	return size;
}

static ssize_t attr_reset_autozero(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, INT_CFG_REG, 1, snsdata);
	if (err < 0) {
        mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_INT_CFG_REG] = init_val;

	updated_val = ((RESET_AZ_MASK & (((u8)val)<<4)) |
		       ((~RESET_AZ_MASK) & init_val));
	snsdata[0] = updated_val;
	err = prs->tf->write(prs, INT_CFG_REG, 1, snsdata);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit:
	return size;
}

static ssize_t attr_set_autorifp(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val, updated_val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, INT_CFG_REG, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_INT_CFG_REG] = init_val;
	updated_val = ((AUTORIFP_MASK & (((u8)val) << 7)) |
		       ((~AUTORIFP_MASK) & init_val));
	err = prs->tf->write(prs, INT_CFG_REG, 1, &updated_val);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit :
	return size;
}

static ssize_t attr_reset_autorifp(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);

	err = prs->tf->read(prs, INT_CFG_REG, 1, snsdata);
	if (err < 0) {
        mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_INT_CFG_REG] = init_val;

	updated_val = ((RESET_ARP_MASK & (((u8)val) << 6)) |
		       ((~RESET_ARP_MASK) & init_val));
	snsdata[0] = updated_val;
	err = prs->tf->write(prs, INT_CFG_REG, 1, snsdata);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit:
	return size;
}

static ssize_t attr_set_pthreshold(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&prs->lock);

	snsdata[0] = ((u16)val) & 0xFF;
	snsdata[1] = (((u16)val) >> 8) & 0xFF;
	err = prs->tf->write(prs, THS_P_L, 2, snsdata);

	if (err >= 0) {
		prs->resume_state[RES_THS_P_L] = snsdata[0];
		prs->resume_state[RES_THS_P_H] = snsdata[1];
	}

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

#ifdef DEBUG
	SENSOR_INFO("LPS22HB new pressure threshold setting : %d \r\n",
	       (((u16)snsdata[1]) << 8) + (u16)(snsdata[0]));
#endif

	return size;
}

static ssize_t attr_get_pthreshold(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u16 val = 0;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, THS_P_L, 2, snsdata);
	val = ((u16)snsdata[1] << 8) + snsdata[0];
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_pthreshold_enable(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 mask = ((u8)(DIFF_EN_MASK | PLE_MASK | PHE_MASK));

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, INT_CFG_REG, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_INT_CFG_REG] = init_val;

	updated_val = ((~mask) & init_val);
	if (val == 1)
		updated_val |= mask;

	snsdata[0] = updated_val;
	err = prs->tf->write(prs, INT_CFG_REG, 1, snsdata);
	if (err >= 0)
		prs->resume_state[RES_INT_CFG_REG] = updated_val;

	mutex_unlock(&prs->lock);
	
	if (err < 0)
		return err;
exit:
	return size;
}

static ssize_t attr_get_pthreshold_enable(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;
	u8 mask = ((u8)(DIFF_EN_MASK | PLE_MASK | PHE_MASK));

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, INT_CFG_REG, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	val = snsdata[0] & mask;
	mutex_unlock(&prs->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_watermark_enable(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);

	err = prs->tf->read(prs, CTRL_REG2, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_CTRL_REG2] = init_val;

	updated_val = ((STOP_ON_FTH_MASK & ((u8)val) << 5) |
		       ((~STOP_ON_FTH_MASK) & init_val));

	snsdata[0] = updated_val;
	err = prs->tf->write(prs, CTRL_REG2, 1, snsdata);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	else
		prs->resume_state[RES_CTRL_REG2] = updated_val;
exit:
	return size;
}

static ssize_t attr_get_watermark_enable(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, CTRL_REG2, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	val = ((snsdata[0] & 0x20)>>5);
	mutex_unlock(&prs->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_lc_mode_enable(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

	mutex_lock(&prs->lock);

	err = prs->tf->read(prs, RESOL_CONF, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_RESOL_CONF] = init_val;
	updated_val = ((LC_EN_MASK & ((u8)val)) | ((~LC_EN_MASK) & init_val));

	/* power down the device before going ahead with LC enable update */
	snsdata[0] = prs->resume_state[RES_CTRL_REG1] & 0x0F;
	err = prs->tf->write(prs, CTRL_REG1, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}
	/* power down transaction end */
	snsdata[0] = updated_val;
	err = prs->tf->write(prs, RESOL_CONF, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	} else
		prs->resume_state[RES_RESOL_CONF] = updated_val;

	/* power up the device or at least get CTRL_REG1 back to its previous state */
	snsdata[0] = prs->resume_state[RES_CTRL_REG1] & 0x7F;
	err = prs->tf->write(prs, CTRL_REG1, 1, snsdata);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

exit:
	return size;
}

static ssize_t attr_get_lc_mode_enable(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, RESOL_CONF, 1, snsdata);
	val = snsdata[1] & 0x1;
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_lpf_enable(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = EN_LPFP_MASK;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	if ((val != 0) & (val != 1))
		goto exit;

	mutex_lock(&prs->lock);

	err = prs->tf->read(prs, CTRL_REG1, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_CTRL_REG1] = init_val;

	updated_val = ((mask & ((u8)val)<<3) | ((~mask) & init_val));

	snsdata[0] = updated_val;
	err =  prs->tf->write(prs, CTRL_REG1, 1, snsdata);

	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	else
		prs->resume_state[RES_CTRL_REG1] = updated_val;
exit:
	return size;
}

static ssize_t attr_get_lpf_enable(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, CTRL_REG1, 1, snsdata);
	val = ((snsdata[1] & 0x08) >> 3);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_set_lpf_cutoff_freq(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;
	u8 const mask = LPF_CFG_MASK;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	if ((val!=0) && (val!=1))
		goto exit;

	mutex_lock(&prs->lock);

	err = prs->tf->read(prs, CTRL_REG1, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_CTRL_REG1] = init_val;

	updated_val = (((mask) & (((u8)val)<<2)) | ((~mask) & init_val));

	snsdata[0] = updated_val;
	err = prs->tf->write(prs, CTRL_REG1, 1, snsdata);
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;
	else
		prs->resume_state[RES_CTRL_REG1] = updated_val;

exit:
	return size;
}

static ssize_t attr_get_lpf_cutoff_freq(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, CTRL_REG1, 1, snsdata);
	val = ((snsdata[0] & LPF_CFG_MASK) >> 2) & 0x1;
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_get_fifo_status(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, FIFO_STATUS, 1, snsdata);
	val = snsdata[0];
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_get_status(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, STATUS_REG, 1, snsdata);
	val = snsdata[0];
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_get_interrupt_source(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int err;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 val = 0;

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, INT_SRC_REG, 1, snsdata);
	val = snsdata[0];
	mutex_unlock(&prs->lock);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", val);
}

static ssize_t attr_set_fifo(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	int err = -1;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;
	static u8 init_val,updated_val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	if ((val != 0) && (val != 1))
		goto exit;

#ifdef DEBUG
	SENSOR_INFO("\n%s Valid val to put in reg2: %lu \r\n", LPS22_PRS_DEV_NAME, val);
#endif

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, CTRL_REG2, 1, snsdata);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	init_val = snsdata[0];
	prs->resume_state[RES_CTRL_REG2] = init_val;
	updated_val = (((FIFO_EN_MASK) & (((u8)val)<<6)) | ((~FIFO_EN_MASK) & (init_val)));

	snsdata[0] = updated_val;
	err = prs->tf->write(prs, CTRL_REG2, 1, snsdata);

	mutex_unlock(&prs->lock);

	if (err < 0)
		return err;
	else
		prs->resume_state[RES_CTRL_REG2] = updated_val;

exit:
	return size;
}

static ssize_t attr_fifo_mode(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t size)
{
        unsigned long val;
	int err = -1;
	u8 new_val;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 x;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

#ifdef DEBUG
	SENSOR_INFO("\n%s Valid val: %lu ", LPS22_PRS_DEV_NAME, val);
#endif

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, FIFO_CTRL, 1, &x);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	new_val = ( ((u8)val << 5) | (x & ~FIFO_MODE_MASK) );
	err = prs->tf->write(prs, FIFO_CTRL, 1, &new_val);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	} else
		prs->resume_state[RES_FIFO_CTRL] = new_val;

	mutex_unlock(&prs->lock);

	return size;
}

static ssize_t attr_set_samples_fifo(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
        unsigned long val;
	int err= -1;
	u8 new_val;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 x;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

#ifdef DEBUG
	SENSOR_INFO("\n%s Valid val: %lu ", LPS22_PRS_DEV_NAME, val);
#endif

	mutex_lock(&prs->lock);
	err = prs->tf->read(prs, FIFO_CTRL, 1, &x);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	}

	new_val = ((((u8)val) - 1) | (x & FIFO_MODE_MASK));

	err = prs->tf->write(prs, FIFO_CTRL, 1, &new_val);
	if (err < 0) {
		mutex_unlock(&prs->lock);
		return err;
	} else
		prs->resume_state[RES_FIFO_CTRL] = new_val;

	mutex_unlock(&prs->lock);

	return size;
}

#ifdef DEBUG
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t size)
{
	int rc;
        unsigned long val;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	u8 reg, data;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&prs->lock);
	reg = prs->reg_addr;
	mutex_unlock(&prs->lock);
	data = (u8)val;
	rc = prs->tf->write(prs, reg, 1, &data);
	if (rc < 0)
		return rc;

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int rc;
	u8 reg, data;

	mutex_lock(&prs->lock);
	reg = prs->reg_addr;
	mutex_unlock(&prs->lock);
	rc = prs->tf->read(prs, reg, 1, &data);
	if (rc < 0)
		return rc;

	return sprintf(buf, "0x%02x\n", data);
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&prs->lock);
	prs->reg_addr = val;
	mutex_unlock(&prs->lock);

	return size;
}

static ssize_t attr_reg_dump(struct device *dev, struct device_attribute *attr,char *buf)
{
	ssize_t ret;
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int err;
	u8 data = 0;
	u8 addr;

	SENSOR_INFO("\r\n");
	mutex_lock(&prs->lock);
	for(addr = 0x0B; addr <= 0x2C;addr++) {
		err = prs->tf->read(prs, addr, 1, snsdata);
		if (err < 0) {
			SENSOR_ERR("Error reading from register %02x \r\n",addr);
		} else {
			SENSOR_INFO("register addr: %02x value: %02x \r\n",addr,snsdata[0]);
			if (addr == 0x0F)
				data = snsdata[0];
		}
	}

	mutex_unlock(&prs->lock);
	ret = sprintf(buf, "0x%02x\n", data);

	return ret;
}

static ssize_t attr_set_logout_decimation(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&prs->lock);
	logout_decimation = val;
	mutex_unlock(&prs->lock);

	return size;
}

static ssize_t attr_set_hex_measr_log(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	if ((val != 0) && (val!=1))
		goto exit;

	mutex_lock(&prs->lock);
	hex_measr_logging = val;
	mutex_unlock(&prs->lock);

exit:
	return size;
}
#endif
static ssize_t stm_barometer_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_NAME);
}

static ssize_t set_barometer_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t pressure_cabratioin_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int temp = 0, error = 0;

	error = kstrtoint(buf, 10, &temp);
	if (error < 0) {
		pr_err("[FACTORY] %s : kstrtoint failed.(%d)", __func__, error);
		return error;
	}

	if (temp < PR_ABS_MIN || temp > PR_ABS_MAX)
		return -EINVAL;

	pressure_cal = temp;

	return size;
}

static ssize_t pressure_cabratioin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "%d\n", pressure_cal);
}

static ssize_t temperature_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	struct outputdata output;
	int err;
	err = lps22_prs_get_presstemp_data(prs, &output);
	if (err < 0)
		SENSOR_ERR("get_pressure_data failed\n");

	return snprintf(buf, PAGE_SIZE, "%d\n",
		output.temperature);
}

static ssize_t sea_level_pressure_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", sea_level_pressure);
}

static ssize_t sea_level_pressure_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (sscanf(buf, "%10d", &sea_level_pressure) != 1) {
		pr_err("[FACTORY] %s: sscanf error\n", __func__);
		return size;
	}

	sea_level_pressure = sea_level_pressure / 100;

	pr_info("[FACTORY] %s: sea_level_pressure = %d\n", __func__,
		sea_level_pressure);

	return size;
}

static ssize_t pressure_poll_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%lld\n",
		ktime_to_ns(prs->delta_ts));
}

static ssize_t pressure_poll_delay_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = kstrtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	if (new_delay != ktime_to_ns(prs->delta_ts)) {
		SENSOR_INFO("poll_delay = %lld\n", new_delay);
		prs->delta_ts = ns_to_ktime(new_delay);
	}

	return size;
}

static ssize_t barometer_selftest_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct lps22_prs_data *prs = dev_get_drvdata(dev);
	int err;
	int i;
	int retry = 3;
	bool selftest_result = false;
	s32 pressure = 0, pre_pressure = 0;
	int selftest_enable = 0;
	u8 prs_data[5] = {0,};
	
	selftest_enable = atomic_read(&prs->enabled);
	if (!selftest_enable) { 
		SENSOR_INFO("selftest enable\n");
		lps22_prs_enable(prs);
	}
	/* read chip id */
	while (retry--) {
		err = prs->tf->read(prs, WHO_AM_I, 1, buf);
		if (err < 0) 
			SENSOR_ERR("Error reading WHO_AM_I: is device "
			 	"available/working?\n");
		else
			prs->hw_working = 1;

		if (buf[0] != WHOAMI_LPS22_PRS) {
			SENSOR_ERR("device unknown. Expected: 0x%02x,"
				" Replies: 0x%02x\n", WHOAMI_LPS22_PRS, buf[0]);
			err = -1;
		} else {
			break;
		}
		msleep(20);
	}
	if (retry < 0)
		goto exit;


	SENSOR_INFO("%s ID Chip OK \n", LPS22_PRS_DEV_NAME);

	for(i = 0; i < SELFTEST_READ_TIMES; i++) {
		err = prs->tf->read(prs, OUTDATA_REG, 5, prs_data);
		if (err < 0)
			SENSOR_ERR("Error reading pressure data\n");
		if (!pre_pressure) {
			pre_pressure = (s32)((((s8)prs_data[2]) << 16) | (prs_data[1] << 8) |
		 		prs_data[0]);
			SENSOR_INFO("pre_pressure:%d\n",pre_pressure);

		} else {
			pressure = (s32)((((s8)prs_data[2]) << 16) | (prs_data[1] << 8) |
			 	prs_data[0]);
			if ((pre_pressure != pressure) && pressure) {
				SENSOR_INFO("pressure:%d\n",pressure);
				selftest_result = true;
				break;
			}
		}
		msleep(110);
	}
 exit:
	selftest_enable = atomic_read(&prs->enabled);

	if (!selftest_enable) {
		lps22_prs_disable(prs);
		SENSOR_INFO("selftest disable\n");
	}

	SENSOR_INFO("selftest result is  = %d\n", selftest_result);
	
	if(selftest_result)
		return snprintf(buf, PAGE_SIZE, "%d\n", PASS);
	else
		return snprintf(buf, PAGE_SIZE, "%d\n", FAIL);
}

static DEVICE_ATTR(name, S_IRUGO, stm_barometer_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, set_barometer_vendor_show, NULL);
static DEVICE_ATTR(poll_period_ms, 0664, attr_get_polling_rate, attr_set_polling_rate);
static DEVICE_ATTR(pressure_reference_level, 0664, attr_get_press_ref, attr_set_press_ref);
static DEVICE_ATTR(pressure_threshold, 0664, attr_get_pthreshold, attr_set_pthreshold);
static DEVICE_ATTR(enable_pthreshold_detection, 0664, attr_get_pthreshold_enable, attr_set_pthreshold_enable);
static DEVICE_ATTR(enable_lc_mode, 0664, attr_get_lc_mode_enable, attr_set_lc_mode_enable);
static DEVICE_ATTR(enable_lpf, 0664, attr_get_lpf_enable, attr_set_lpf_enable);
static DEVICE_ATTR(lpf_cutoff_freq, 0664, attr_get_lpf_cutoff_freq, attr_set_lpf_cutoff_freq);
static DEVICE_ATTR(enable_watermark, 0664, attr_get_watermark_enable, attr_set_watermark_enable);
static DEVICE_ATTR(enable_autozero, 0220, NULL, attr_set_autozero);
static DEVICE_ATTR(reset_autozero, 0220, NULL, attr_reset_autozero);
static DEVICE_ATTR(enable_autorifp, 0220, NULL, attr_set_autorifp);
static DEVICE_ATTR(reset_autorifp, 0220, NULL, attr_reset_autorifp);
static DEVICE_ATTR(fifo_status, 0444, attr_get_fifo_status, NULL);
static DEVICE_ATTR(status, 0444, attr_get_status, NULL);
static DEVICE_ATTR(int_source, 0444, attr_get_interrupt_source, NULL);
static DEVICE_ATTR(enable_fifo, 0220, NULL, attr_set_fifo);
static DEVICE_ATTR(num_samples_fifo, 0220, NULL, attr_set_samples_fifo);
static DEVICE_ATTR(fifo_mode, 0220, NULL, attr_fifo_mode);
static DEVICE_ATTR(calibration, 0664, pressure_cabratioin_show, pressure_cabratioin_store);
static DEVICE_ATTR(sea_level_pressure, 0664, sea_level_pressure_show, sea_level_pressure_store);
static DEVICE_ATTR(temperature, 0444, temperature_show, NULL);
static DEVICE_ATTR(selftest, 0444, barometer_selftest_show, NULL);
#ifdef DEBUG
static DEVICE_ATTR(reg_value, 0664, attr_reg_get, attr_reg_set);
static DEVICE_ATTR(reg_addr, 0220, NULL, attr_addr_set);
static DEVICE_ATTR(reg_dump, 0444, attr_reg_dump, NULL);
static DEVICE_ATTR(dmesg_decimation, 0220, NULL,attr_set_logout_decimation);
static DEVICE_ATTR(hex_measr_log, 0220, NULL,attr_set_hex_measr_log);
#endif

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_vendor,
	&dev_attr_poll_period_ms,
	&dev_attr_pressure_reference_level,
	&dev_attr_pressure_threshold,
	&dev_attr_enable_pthreshold_detection,
	&dev_attr_enable_lc_mode,
	&dev_attr_enable_lpf,
	&dev_attr_lpf_cutoff_freq,
	&dev_attr_enable_watermark,
	&dev_attr_enable_autozero,
	&dev_attr_reset_autozero,
	&dev_attr_enable_autorifp,
	&dev_attr_reset_autorifp,
	&dev_attr_fifo_status,
	&dev_attr_status,
	&dev_attr_int_source,
	&dev_attr_enable_fifo,
	&dev_attr_num_samples_fifo,
	&dev_attr_fifo_mode,
	&dev_attr_calibration,
	&dev_attr_sea_level_pressure,
	&dev_attr_temperature,
	&dev_attr_selftest,
#ifdef DEBUG
	&dev_attr_reg_value,
	&dev_attr_reg_addr,
	&dev_attr_reg_dump,
	&dev_attr_dmesg_decimation,
	&dev_attr_hex_measr_log,
#endif
	NULL,
};

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		attr_get_enable, attr_set_enable);
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		pressure_poll_delay_show, pressure_poll_delay_store);
static struct attribute *barometer_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL,
};

static struct attribute_group barometer_attribute_group = {
	.attrs = barometer_sysfs_attrs,
};

#if 0
static int create_sysfs_interfaces(struct device *dev)
{
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		ret = device_create_file(dev, attributes + i);
		if (ret < 0)
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	SENSOR_ERR("%s:Unable to create interface\n", __func__);

	return ret;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}
#endif
static void lps22_prs_input_work_func(struct work_struct *work)
{
	struct lps22_prs_data *prs = container_of((struct work_struct *)work,
						  struct lps22_prs_data,
						  input_work);
	struct outputdata output;
	int err;

	mutex_lock(&prs->lock);
	err = lps22_prs_get_presstemp_data(prs, &output);
	if (err < 0)
		SENSOR_ERR("get_pressure_data failed\n");
	else
		lps22_prs_report_values(prs, &output);

	mutex_unlock(&prs->lock);

	hrtimer_start(&prs->hr_timer, prs->delta_ts, HRTIMER_MODE_REL);
}

static enum hrtimer_restart lps22_prs_poll_function_read(struct hrtimer *timer)
{
	struct lps22_prs_data *prs;

	prs = container_of((struct hrtimer *)timer, struct lps22_prs_data,
			   hr_timer);

	queue_work(prs->workqueue, &prs->input_work);

	return HRTIMER_NORESTART;
}

static int lps22_prs_validate_pdata(struct lps22_prs_data *prs)
{
	/* checks for correctness of minimal polling period */
	prs->pdata->min_interval = (unsigned int)LPS22_PRS_MIN_POLL_PERIOD_MS;
	prs->pdata->poll_interval = max(prs->pdata->poll_interval,
					prs->pdata->min_interval);

	/* Checks polling interval relative to minimum polling interval */
	if (prs->pdata->poll_interval < prs->pdata->min_interval) {
		SENSOR_ERR("minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}


static int lps22_prs_input_init(struct lps22_prs_data *prs)
{
	int err;

	prs->workqueue = create_workqueue(prs->name);
	if (!prs->workqueue)
		return -ENOMEM;

	INIT_WORK(&prs->input_work, lps22_prs_input_work_func);
	hrtimer_init(&prs->hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	prs->hr_timer.function = &lps22_prs_poll_function_read;

	prs->input_dev_pres = input_allocate_device();
	if (!prs->input_dev_pres) {
		SENSOR_ERR("input device allocate failed\n");

		return -ENOMEM;
	}

	prs->input_dev_pres->name = MODULE_NAME;
	prs->input_dev_pres->id.bustype = BUS_I2C;
	input_set_drvdata(prs->input_dev_pres, prs);

	__set_bit(INPUT_EVENT_TYPE, prs->input_dev_pres->evbit);
	__set_bit(INPUT_EVENT_X, prs->input_dev_pres->mscbit);
	__set_bit(INPUT_EVENT_Y, prs->input_dev_pres->mscbit);

	err = input_register_device(prs->input_dev_pres);
	if (err) {
		SENSOR_ERR("unable to register input polled device %s\n",
			prs->input_dev_pres->name);
		input_free_device(prs->input_dev_pres);
		goto err_register_input_dev;
	}
	err = sensors_create_symlink(&prs->input_dev_pres->dev.kobj, prs->input_dev_pres->name);
	if (err < 0)
		goto err_create_sensor_symlink;

	err = sysfs_create_group(&prs->input_dev_pres->dev.kobj, &barometer_attribute_group);
	if (err < 0)
		goto err_create_sysfs_group;

	return 0;
err_create_sysfs_group:
	sensors_remove_symlink(&prs->input_dev_pres->dev.kobj, prs->input_dev_pres->name);
err_create_sensor_symlink:
	input_unregister_device(prs->input_dev_pres);
err_register_input_dev:
	prs->input_dev_pres = NULL;
	return err;
}

static void lps22_prs_input_cleanup(struct lps22_prs_data *prs)
{
	input_unregister_device(prs->input_dev_pres);
	input_free_device(prs->input_dev_pres);
}

int lps22hb_common_probe(struct lps22_prs_data *prs)
{
	int err = -1;
	u8 buf[5];
	int retry = 5;

	SENSOR_INFO("%s: probe start.\n", LPS22_PRS_DEV_NAME);

	mutex_init(&prs->lock);
	mutex_init(&prs->tb.buf_lock);
	mutex_lock(&prs->lock);

	prs->pdata = kzalloc(sizeof(*prs->pdata), GFP_KERNEL);
	if(prs->pdata == NULL) {
		err = -ENOMEM;
		SENSOR_ERR("failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlockfreedata;
	}

	if (prs->dev->platform_data == NULL) {	
		memcpy(prs->pdata, &default_lps22_pdata,
		       sizeof(struct lps22_prs_platform_data));
		SENSOR_INFO("using default plaform_data for lps22\n");
	} else {
		memcpy(prs->pdata, prs->dev->platform_data,
		       sizeof(struct lps22_prs_platform_data));
		SENSOR_INFO("using user plaform_data for lps22\n");
        }

	if (prs->pdata->init) {
		err = prs->pdata->init();
		if (err < 0) {
			SENSOR_ERR("lps pdata init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	/* read chip id */
	while (retry--) {
		err = prs->tf->read(prs, WHO_AM_I, 1, buf);
		if (err < 0) 
			SENSOR_ERR("Error reading WHO_AM_I: is device "
			 	"available/working?\n");
		else
			prs->hw_working = 1;

		if (buf[0] != WHOAMI_LPS22_PRS) {
			SENSOR_ERR("device unknown. Expected: 0x%02x,"
				" Replies: 0x%02x\n", WHOAMI_LPS22_PRS, buf[0]);
			err = -1;
		}
		else 
			break;

		msleep(20);
	}
	if (retry < 0)
		goto err_mutexunlockfreedata;

	SENSOR_INFO("%s ID Chip OK \n", LPS22_PRS_DEV_NAME);

	err = lps22_prs_validate_pdata(prs);
	if (err < 0) {
		SENSOR_ERR("failed to validate platform data\n");
		goto err_exit_kfree_pdata;
	}

	if (prs->pdata->init) {
		err = prs->pdata->init();
		if (err < 0) {
			SENSOR_ERR("init failed: %d\n", err);
			goto err_exit_pointer;
		}
	}

	memset(prs->resume_state, 0, ARRAY_SIZE(prs->resume_state));

	/* init registers which need values different from zero */
	prs->resume_state[RES_CTRL_REG1] = (ODR_MASK & ODR_1_1) | (BDU_MASK);

	err = lps22_prs_device_power_on(prs);
	if (err < 0) {
		SENSOR_ERR("power on failed: %d\n", err);
		goto err_exit_pointer;
	}

	atomic_set(&prs->enabled, 1);

	err = lps22_prs_update_odr(prs, prs->pdata->poll_interval);
	if (err < 0) {
		SENSOR_ERR("update_odr failed\n");
		goto err_power_off;
	}

	err = lps22_prs_input_init(prs);
	if (err < 0) {
		SENSOR_ERR("input init failed\n");
		goto err_power_off;
	}

	/* set sysfs for light sensor */
	err = sensors_register(&prs->factory_dev, prs, sensor_attrs, MODULE_NAME);
	if (err < 0) {
		SENSOR_ERR("Sensor registration failed, ret=%d\n", err);
		goto err_sensors_register;
	}

	SENSOR_INFO("Probe ok.\n");
	
	lps22_prs_device_power_off(prs);

	/* As default, do not report information */
	atomic_set(&prs->enabled, 0);

	mutex_unlock(&prs->lock);

	SENSOR_INFO("%s: probed\n", LPS22_PRS_DEV_NAME);

	buf[0] = 0x10;
	err = prs->tf->write(prs, CTRL_REG2, 1, buf);
	if (err < 0) {
		SENSOR_ERR("Sensor write failed, ret=%d\n", err);
		goto err_sensors_register;
	}

	return 0;
#if 0
err_input_cleanup:
	lps22_prs_input_cleanup(prs);
#endif
err_sensors_register:
	sensors_remove_symlink(&prs->input_dev_pres->dev.kobj, prs->input_dev_pres->name);
	sysfs_remove_group(&prs->input_dev_pres->dev.kobj, &barometer_attribute_group);
	input_unregister_device(prs->input_dev_pres);       
err_power_off:
	lps22_prs_device_power_off(prs);
err_exit_pointer:
	if (prs->pdata->exit)
		prs->pdata->exit();
err_pdata_init:
	if (prs->pdata->exit)
		prs->pdata->exit();
err_exit_kfree_pdata:
	kfree(prs->pdata);

err_mutexunlockfreedata:
	mutex_unlock(&prs->lock);
	SENSOR_ERR("%s: Driver Init failed\n", LPS22_PRS_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lps22hb_common_probe);

int lps22hb_common_remove(struct lps22_prs_data *prs)
{
	lps22_prs_disable(prs);
	lps22_prs_input_cleanup(prs);
	lps22_prs_device_power_off(prs);
	//remove_sysfs_interfaces(prs->dev);

	if (prs->workqueue) {
		flush_workqueue(prs->workqueue);
		destroy_workqueue(prs->workqueue);
		prs->workqueue = NULL;
	}

	if (prs->pdata->exit)
		prs->pdata->exit();
	kfree(prs->pdata);

	return 0;
}
EXPORT_SYMBOL(lps22hb_common_remove);

int lps22hb_common_resume(struct lps22_prs_data *prs)
{
	if (prs->on_before_suspend)
		return lps22_prs_enable(prs);

	return 0;
}
EXPORT_SYMBOL(lps22hb_common_resume);

int lps22hb_common_suspend(struct lps22_prs_data *prs)
{
	prs->on_before_suspend = atomic_read(&prs->enabled);

	return lps22_prs_disable(prs);
}
EXPORT_SYMBOL(lps22hb_common_suspend);

void lps22hb_common_shutdown(struct lps22_prs_data *prs)
{
	int value = atomic_read(&prs->enabled);
	if(value)
		lps22_prs_disable(prs);
	
}
EXPORT_SYMBOL(lps22hb_common_shutdown);


MODULE_DESCRIPTION("STMicrolelectronics lps22 pressure sensor driver");
MODULE_AUTHOR("HESA BU, STMicroelectronics");
MODULE_LICENSE("GPL v2");

