/*
 * Copyright by ams AG
 * All rights are reserved.
 *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING
 * THE SOFTWARE.
 *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY
 * EXCLUDED.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/gpio/consumer.h>
#include <linux/sensor/sensors_core.h>
#include <linux/kfifo.h>

#include "ams_dax_reg.h"
#include "ams_dax.h"
#include "ams_i2c.h"

#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif

#define VENDOR_NAME                     "AMS"
#define CHIP_NAME                       "TCS3701"
#define MODULE_NAME                     "light_sensor"

#define REL_DELTA                       REL_DIAL
#define REL_2ND_MIN                     REL_HWHEEL
#define REL_2ND_MAX                     REL_WHEEL
#define REL_LUX                         REL_MISC

#define LIGHT_LOG_TIME                  40

#define HIGH_BRIGHTNESS                 78
#define DEFAULT_JAMES_DELAY_MS          20
#define DEFAULT_RGBC_DELAY_MS           100
#define FIFO_OV                         0x80
#define DEFAULT_BRIGHTNESS_LEVEL        110
#define ASCII_TO_DEC(x)                 (x - 48)
#define GAIN_FACTOR                     (512)
#define CMD_CAM_LUX_DISABLE              2
#define EVENT_CAM_LUX_DISABLE           -2
#define EVENT_INIT_MOVING_AVERAGE       -3
#define TCS3701_CHIP_ID                 0x18

#define STANDARD_DEVIATION_HIGH_THRESHOLD 10000
#define STANDARD_DEVIATION_LOW_THRESHOLD  8100

u8 smux_data[20] = {
	0x12, 0x10, 0x21, 0x21,
	0x11, 0x20, 0x12, 0x22,
	0x01, 0x21, 0x20, 0x12,
	0x12, 0x22, 0x21, 0x12,
	0x11, 0x02, 0x00, 0x76
};

u8 smux_default_data[20] = {
	0x14, 0x25, 0x23, 0x41,
	0x33, 0x12, 0x14, 0x24,
	0x53, 0x23, 0x15, 0x14,
	0x32, 0x44, 0x21, 0x23,
	0x13, 0x54, 0x00, 0x76
};

const u16 alsGain_conversion[] = {
	1,
	1,
	2,
	4,
	8,
	16,
	32,
	64,
	128,
	256,
	512,
	1024,
	2048
};

static u16 ams_alsTimeUsToReg(u32 x)
{
	u16 regValue;

	regValue = (x / 2780) - 1;
	return regValue;
}

static u8 ams_alsGainToReg(u32 x)
{
	int i;

	for (i = sizeof(alsGain_conversion)/sizeof(u16)-1; i != 0; i--)
		if (x >= alsGain_conversion[i])
			break;

	return (i << 0);
}

static ssize_t ams_light_name_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", CHIP_NAME);
}

static ssize_t ams_light_reg_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	u8 reg = 0;
	int offset = 0;
	u8 val = 0;

	for (reg = 0x00; reg <= 0xFF; reg++) {
		val = i2c_smbus_read_byte_data(chip->client, reg);
		SENSOR_INFO("Read Reg: 0x%2x Value: 0x%2x\n", reg, val);
		offset += snprintf(buf + offset, PAGE_SIZE - offset,
			"Reg: 0x%2x Value: 0x%2x\n", reg, val);
	}

	return offset;
}

static ssize_t ams_light_reg_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ams_chip *chip = dev_get_drvdata(dev);
	int reg, val, ret;

	if (sscanf(buf, "%2x,%4x", &reg, &val) != 2) {
		SENSOR_ERR("invalid value\n");
		return count;
	}

	ret = ams_i2c_write(chip->client, chip->shadow, reg, val);
	if (!ret)
		SENSOR_INFO("Register(0x%2x) data(0x%4x)\n", reg, val);
	else
		SENSOR_ERR("failed %d\n", ret);

	return count;
}

static void ams_get_itime(struct ams_chip *chip)
{
	if (chip->brightness_level <= BRIGHTNESS_CODE_LEVEL_1)
		chip->itime = 2060; /* ((4.167/sample) -0.163)*1000 */
	else if (chip->brightness_level <= BRIGHTNESS_CODE_LEVEL_2)
		chip->itime = 1320;
	else if (chip->brightness_level <= BRIGHTNESS_CODE_LEVEL_3)
		chip->itime = 925;
	else if (chip->brightness_level <= BRIGHTNESS_CODE_LEVEL_MAX)
		chip->itime = 690;
	else
		chip->itime = 690;
}

static void ams_change_fifo_sample(struct ams_chip *chip)
{
	int alg_current_itime;

	if (chip->cur_algo_mode == ALS_ALGO_HIGH)
		return;

	alg_current_itime = chip->itime;
	ams_get_itime(chip);

	if (alg_current_itime != chip->itime) {
		cancel_delayed_work_sync(&chip->main_work);
		SENSOR_INFO("change itime : %d\n", chip->itime);
		ams_i2c_write(chip->client, chip->shadow, AMS_REG_ENABLE, 0x01);
		i2c_smbus_write_word_data(chip->client, AMS_REG_ASTEPL,
			ams_alsTimeUsToReg(chip->itime * 1000));
		ams_i2c_write(chip->client, chip->shadow, AMS_REG_ENABLE, 0x03);
		ams_i2c_set_field(chip->client, chip->shadow,
			AMS_REG_CONTROL, 0x1, 0x1, 0x1);
		chip->fifoCnt = 0;
		schedule_delayed_work(&chip->main_work,
			nsecs_to_jiffies(atomic_read(&chip->delay)));
	}
}

static void ams_change_algo_mode(struct ams_chip *chip, int algo_mode)
{
	int alg_current_mode = chip->cur_algo_mode;

	if (algo_mode == ALS_ALGO_HIGH || chip->brightness_level == 0) {
		chip->itime = 16000;
		chip->cur_algo_mode = ALS_ALGO_HIGH;
	} else {
		chip->cur_algo_mode = ALS_ALGO_MID;
	}

	if (alg_current_mode != chip->cur_algo_mode) {
		cancel_delayed_work_sync(&chip->main_work);

		switch (chip->cur_algo_mode) {
		case ALS_ALGO_MID:
			SENSOR_INFO("START MID ALGORITHM\n");
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ENABLE, 0x01);
			/* SMUX write command */
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_CFG6, 0x10);
			/* Send smux remap sequence */
			ams_i2c_reg_blk_write(chip->client,
				AMS_REG_RAM_START, smux_data, 20);
			/* execute the smux command */
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ENABLE, 0x11);
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_CFG6, 0x00);
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ALS_CHANNEL_CTRL, 0x3c);
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ATIME, 0x00);
			i2c_smbus_write_byte_data(chip->client,
				AMS_REG_WTIME, 0x00);
			chip->again = 1024;
			i2c_smbus_write_byte_data(chip->client,
				AMS_REG_CFG1, ams_alsGainToReg(chip->again));
			ams_get_itime(chip);
			i2c_smbus_write_word_data(chip->client, AMS_REG_ASTEPL,
				ams_alsTimeUsToReg(chip->itime * 1000));
			i2c_smbus_write_byte_data(chip->client,
				AMS_REG_FIFO_MAP, 0x06);
			i2c_smbus_write_byte_data(chip->client,
				AMS_REG_CFG8, 0x18);
			ams_i2c_set_field(chip->client,
				chip->shadow, AMS_REG_PCFG1, 0x3, 0x1, 0x1);
			ams_i2c_set_field(chip->client,
				chip->shadow, AMS_REG_CONTROL, 0x1, 0x1, 0x1);
			chip->fifoCnt = 0;
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ENABLE, 0x03);
			atomic_set(&chip->delay,
				DEFAULT_JAMES_DELAY_MS * NSEC_PER_MSEC);
			break;
		case ALS_ALGO_HIGH:
			SENSOR_INFO("START HIGH ALGORITHM\n");
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ENABLE, 0x01);
			/* SMUX write command */
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_CFG6, 0x10);
			/* Send smux remap sequence */
			ams_i2c_reg_blk_write(chip->client,
				AMS_REG_RAM_START, smux_default_data, 20);
			/* execute the smux command */
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ENABLE, 0x11);
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_CFG6, 0x00);
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ALS_CHANNEL_CTRL, 0x00);
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ATIME, 0x00);
			i2c_smbus_write_byte_data(chip->client,
				AMS_REG_WTIME, 0x00);
			i2c_smbus_write_byte_data(chip->client,
				AMS_REG_FIFO_MAP, 0x00);
			ams_i2c_set_field(chip->client,
				chip->shadow, AMS_REG_CFG8, 0x2, 0x1, 0x1);

			/* 16msec integration time for HIGH BRIGHTNESS ALG */
			i2c_smbus_write_word_data(chip->client, AMS_REG_ASTEPL,
				ams_alsTimeUsToReg(chip->itime * 1000));
			ams_i2c_write(chip->client,
				chip->shadow, AMS_REG_ENABLE, 0x03);
			atomic_set(&chip->delay,
				DEFAULT_RGBC_DELAY_MS * NSEC_PER_MSEC);
			chip->previousGain = 0;
			break;
		default:
			break;
		}
		schedule_delayed_work(&chip->main_work,
			nsecs_to_jiffies(atomic_read(&chip->delay)));
	}
}

static void ams_report_events(struct ams_chip *chip)
{
	if (chip->lux < 0)
		chip->lux = 0;

	input_report_rel(chip->input_dev, REL_LUX, chip->lux + 1);
	input_sync(chip->input_dev);
}

static void ams_report_high_brightness_events(struct ams_chip *chip)
{
	input_report_rel(chip->input_dev, REL_DELTA, chip->ch0_delta + 1);
	input_report_rel(chip->input_dev, REL_2ND_MIN, chip->ch0_2nd_min + 1);
	input_report_rel(chip->input_dev, REL_2ND_MAX, chip->ch0_2nd_max + 1);
	input_sync(chip->input_dev);
}

static bool ams_highBrightness_alsCalcLux(struct ams_chip *chip,
	u32 (*min_buf)[JAMES_FIFO_MAX_CNT],
	u32 (*max_buf)[JAMES_FIFO_MAX_CNT])
{
	int i;
	u32 ch0_min = 0xfffff;
	u32 ch0_max = 0;

	chip->ch0_2nd_max = 0;
	chip->ch0_2nd_min = 0xfffff;

	for (i = 0; i < JAMES_FIFO_MAX_CNT; i++) {
		if (max_buf[CH0][i] > ch0_max) {
			chip->ch0_2nd_max = ch0_max;
			ch0_max = max_buf[CH0][i];
		} else if (max_buf[CH0][i] > chip->ch0_2nd_max) {
			chip->ch0_2nd_max = max_buf[CH0][i];
		}

		if (min_buf[CH0][i] < ch0_min) {
			chip->ch0_2nd_min = ch0_min;
			ch0_min = min_buf[CH0][i];
		} else if (min_buf[CH0][i] < chip->ch0_2nd_min) {
			chip->ch0_2nd_min = min_buf[CH0][i];
		}
	}

	chip->ch0_2nd_max = chip->ch0_2nd_max * GAIN_FACTOR / chip->again;
	chip->ch0_2nd_min = chip->ch0_2nd_min * GAIN_FACTOR / chip->again;

	chip->ch0_delta = (4 * chip->ch0_2nd_max) - (4 * chip->ch0_2nd_min);

	if (!chip->ch0_delta)
		chip->ch0_delta = 1;

	if (chip->count_log_time >= LIGHT_LOG_TIME) {
		SENSOR_INFO("[MAX-MIN] max: %d min: %d Br: %d\n",
			chip->ch0_2nd_max, chip->ch0_2nd_min,
			chip->brightness_level);
		chip->count_log_time = 0;
	} else {
		chip->count_log_time++;
	}

	return true;
}

static bool ams_alsCalcLux(struct ams_chip *chip,
		u32 (*buf)[JAMES_FIFO_MAX_CNT])
{
	int i;
	u32 ch0_min = 0xffff, ch1_min = 0xffff;
	int64_t ch0_sum = 0, ch1_sum = 0;

	for (i = 0; i < JAMES_FIFO_MAX_CNT; i++) {
		ch0_sum += buf[CH0][i];
		if (ch0_min > buf[CH0][i])
			ch0_min = buf[CH0][i];
		ch1_sum += buf[CH1][i];
		if (ch1_min > buf[CH1][i])
			ch1_min = buf[CH1][i];
	}

	ch0_sum = ch0_sum - ch0_min;
	ch1_sum = ch1_sum - ch1_min;

	if (((ch0_sum == ch1_sum) && (ch0_sum != 0)) || (ch0_sum < ch1_sum)) {
		chip->lux = 0;
	} else {
		ch0_sum = ch0_sum * AGAIN_FACTOR / chip->again;
		ch1_sum = ch1_sum * AGAIN_FACTOR / chip->again;

		chip->lux = JAMES_DGF * (CH0_COEF * ch0_sum - CH1_COEF * ch1_sum) / 1000;
		chip->lux = chip->lux * ATIME_FACTOR / (chip->itime * (JAMES_FIFO_MAX_CNT - 1));
	}

	if (chip->count_log_time >= LIGHT_LOG_TIME) {
		SENSOR_INFO("[JAMES] lux: %d, Br: %d\n",
			chip->lux, chip->brightness_level);
		chip->count_log_time = 0;
	} else {
		chip->count_log_time++;
	}

	return false;
}

static bool ams_acLight_alsCalcLux(struct ams_chip *chip,
		u32 (*buf)[JAMES_FIFO_MAX_CNT])
{
	int i;
	u32 ch0_min = 0xffff, ch1_min = 0xffff;
	int64_t ch0_sum = 0, ch1_sum = 0;

	for (i = 0; i < JAMES_FIFO_MAX_CNT; i++) {
		ch0_sum += buf[CH0][i];
		if (ch0_min > buf[CH0][i])
			ch0_min = buf[CH0][i];
		ch1_sum += buf[CH1][i];
		if (ch1_min > buf[CH1][i])
			ch1_min = buf[CH1][i];
	}

	ch0_sum = ch0_sum - ch0_min;
	ch1_sum = ch1_sum - ch1_min;

	if (ch0_sum < ch1_sum) {
		chip->lux = 0;
	} else {
		ch0_sum = ch0_sum * AGAIN_FACTOR / chip->again;
		ch1_sum = ch1_sum * AGAIN_FACTOR / chip->again;

		chip->lux = JAMES_DGF * (CH0_COEF * ch0_sum) / 1000 * 3 / 10;
		chip->lux = chip->lux * ATIME_FACTOR / (chip->itime * (JAMES_FIFO_MAX_CNT - 1));
	}

	if (chip->count_log_time >= LIGHT_LOG_TIME) {
		SENSOR_INFO("[AC-MODE] lux: %d, Br: %d\n",
			chip->lux, chip->brightness_level);
		chip->count_log_time = 0;
	} else {
		chip->count_log_time++;
	}

	return false;
}

static void ams_process_mid(struct ams_chip *chip)
{
	int j = 0;
	int i = 0;
	bool isHbMode;
	u16 fifo_size;
	uint8_t fifo_lvl;
	uint8_t status6;
	u16 sample_cnt = 0;
	u16 quotient = 0, remainder = 0;
	uint64_t sum[CH_MAX_CNT] = {0, 0};
	u32 tmp;
	uint64_t temp;
	u32 recommendedGain;
	u32 max_count;
	u32 adcObjective;
	u32 stddev_ch0, stddev_ch1;

	ams_i2c_read(chip->client, AMS_REG_STATUS6, &status6);
	ams_i2c_read(chip->client, AMS_REG_FIFO_LVL, &fifo_lvl);

	if (fifo_lvl < 1) {
		SENSOR_INFO("FIFO EMPTY\n");
		return;
	}

	if (status6 & FIFO_OV) {
		ams_i2c_set_field(chip->client,	chip->shadow,
			AMS_REG_CONTROL, 0x1, 0x1, 0x1);
		SENSOR_INFO("FIFO OVERFLOW\n");
		return;
	}

	fifo_size = (u16)fifo_lvl * 2;
	sample_cnt = fifo_size / 4; /* ch0 2byte + ch1 2byte = 4byte */

	if (sample_cnt < 1) {
		SENSOR_INFO("FIFO Not enough for ALG\n");
		return;
	}

	quotient = fifo_size / 32;
	remainder = fifo_size % 32;

	memset(&chip->fifodata[0], 0x00, sizeof(uint8_t) * 256);
	if (quotient == 0) { /* fifo size is less than 32 , reading remainder */
		i2c_smbus_read_i2c_block_data(chip->client,
			AMS_REG_FDATAL, remainder, (u8 *)&chip->fifodata[0]);
	} else {
		for (i = 0; i < quotient; i++)
			i2c_smbus_read_i2c_block_data(chip->client,
				AMS_REG_FDATAL, 32,
				(u8 *)&chip->fifodata[i * 32]);

		if (remainder != 0)
			i2c_smbus_read_i2c_block_data(chip->client,
				AMS_REG_FDATAL, remainder,
				(u8 *)&chip->fifodata[i * 32]);
	}

	i = i * 32 + remainder;

	chip->chMinBuf[CH0][chip->fifoCnt] = 0xffff;
	chip->chMinBuf[CH1][chip->fifoCnt] = 0xffff;
	chip->chMaxBuf[CH0][chip->fifoCnt] = 0x0;
	chip->chMaxBuf[CH1][chip->fifoCnt] = 0x0;

	for (j = 0; j < sample_cnt; j++) {
		chip->data_buf[CH0][j] = (u16)((chip->fifodata[j * 4] << 0) | (chip->fifodata[j * 4 + 1] << 8));
		chip->data_buf[CH1][j] = (u16)((chip->fifodata[j * 4 + 2] << 0) | (chip->fifodata[j * 4 + 2 + 1] << 8));
		if (chip->data_buf[CH0][j] < chip->data_buf[CH1][j]) {
			tmp = chip->data_buf[CH0][j];
			chip->data_buf[CH0][j] = chip->data_buf[CH1][j];
			chip->data_buf[CH1][j] = tmp;
		}

		if (chip->chMinBuf[CH0][chip->fifoCnt] > chip->data_buf[CH0][j])
			chip->chMinBuf[CH0][chip->fifoCnt] = chip->data_buf[CH0][j];
		if (chip->chMinBuf[CH1][chip->fifoCnt] > chip->data_buf[CH1][j])
			chip->chMinBuf[CH1][chip->fifoCnt] = chip->data_buf[CH1][j];

		if (chip->chMaxBuf[CH0][chip->fifoCnt] < chip->data_buf[CH0][j])
			chip->chMaxBuf[CH0][chip->fifoCnt] = chip->data_buf[CH0][j];
		if (chip->chMaxBuf[CH1][chip->fifoCnt] < chip->data_buf[CH1][j])
			chip->chMaxBuf[CH1][chip->fifoCnt] = chip->data_buf[CH1][j];

		sum[CH0] += chip->data_buf[CH0][j];
		sum[CH1] += chip->data_buf[CH1][j];
#if 0
		SENSOR_INFO("[CH_RAW] %u, %u\n", chip->data_buf[0][j], chip->data_buf[1][j]);
#endif
	}

	chip->chAvg[CH0][chip->fifoCnt] = sum[CH0] / sample_cnt;
	chip->chAvg[CH1][chip->fifoCnt] = sum[CH1] / sample_cnt;

	sum[CH0] = 0;
	sum[CH1] = 0;

	for (j = 0; j < sample_cnt; j++) {
		if (chip->chAvg[CH0][chip->fifoCnt] > chip->data_buf[CH0][j])
			tmp = (int32_t)chip->chAvg[CH0][chip->fifoCnt] - chip->data_buf[CH0][j];
		else
			tmp = (int32_t)chip->data_buf[CH0][j] - chip->chAvg[CH0][chip->fifoCnt];
		sum[CH0] += (tmp * tmp);

		if (chip->chAvg[CH1][chip->fifoCnt] > chip->data_buf[CH1][j])
			tmp = (int32_t)chip->chAvg[CH1][chip->fifoCnt] - chip->data_buf[CH1][j];
		else
			tmp = (int32_t)chip->data_buf[CH1][j] - chip->chAvg[CH1][chip->fifoCnt];
		sum[CH1] += (tmp * tmp);
	}
	sum[CH0] = sum[CH0] / sample_cnt;
	sum[CH1] = sum[CH1] / sample_cnt;

	if (!((chip->itime == 0) || (chip->again == 0)
		|| (chip->again > 2048))) {
		tmp = (2048 * 1000) / chip->again;
		stddev_ch0 = (u32)(sum[CH0] * tmp * tmp / (chip->itime * chip->itime));
		stddev_ch1 = (u32)(sum[CH1] * tmp * tmp / (chip->itime * chip->itime));
	} else {
		stddev_ch0 = 0;
		stddev_ch1 = 0;
	}

	chip->fifoCnt++;

	max_count = (1000 * chip->itime) / 2780;
	adcObjective = max_count * 128;
	adcObjective /= 160; /* about 80% (128 / 160) */
	temp = (uint64_t)adcObjective * 2048;
	if (chip->chMaxBuf[CH0][chip->fifoCnt - 1] != 0)
		temp /= chip->chMaxBuf[CH0][chip->fifoCnt - 1];
	temp *= chip->again;
	temp /= 2048;
	recommendedGain = temp & 0xffffffff;
	recommendedGain = ams_alsGainToReg(recommendedGain);
	recommendedGain = alsGain_conversion[recommendedGain];

	i2c_smbus_write_byte_data(chip->client, AMS_REG_FIFO_MAP, 0x06);
	if (recommendedGain != chip->again) {
		SENSOR_INFO("gain chg to: %u, (ch0: %u)\n",
			recommendedGain,
			chip->chMaxBuf[CH0][chip->fifoCnt - 1]);
		chip->again = recommendedGain;
		ams_i2c_modify(chip->client, chip->shadow,
			AMS_REG_CFG1, AMS_MASK_AGAIN,
			ams_alsGainToReg(recommendedGain));
		chip->fifoCnt = 0;
		chip->acLightDefenceOnCnt = 0;
		chip->acLightDefenceOffCnt = 0;
		ams_i2c_set_field(chip->client, chip->shadow,
			AMS_REG_CONTROL, 0x1, 0x1, 0x1);
		return;
	}

	ams_i2c_set_field(chip->client, chip->shadow,
		AMS_REG_CONTROL, 0x1, 0x1, 0x1);

	if ((stddev_ch0 > STANDARD_DEVIATION_HIGH_THRESHOLD)
		|| (stddev_ch1 > STANDARD_DEVIATION_HIGH_THRESHOLD))
		chip->acLightDefenceOnCnt++;
	else
		chip->acLightDefenceOnCnt = 0;
	if ((stddev_ch0 < STANDARD_DEVIATION_LOW_THRESHOLD)
		&& (stddev_ch1 < STANDARD_DEVIATION_LOW_THRESHOLD))
		chip->acLightDefenceOffCnt++;
	else
		chip->acLightDefenceOffCnt = 0;

	chip->rawdata.red = chip->chMinBuf[CH0][chip->fifoCnt - 1];
	chip->rawdata.green = chip->chMinBuf[CH1][chip->fifoCnt - 1];
	chip->rawdata.blue = chip->chMaxBuf[CH0][chip->fifoCnt - 1];
	chip->rawdata.clear = chip->chMaxBuf[CH1][chip->fifoCnt - 1];
	if (chip->fifoCnt >= JAMES_FIFO_MAX_CNT) {
		if (chip->acLightDefenceOnCnt >= JAMES_FIFO_MAX_CNT)
			chip->isAcLightDefenceMode = true;
		else if (chip->acLightDefenceOffCnt >= JAMES_FIFO_MAX_CNT)
			chip->isAcLightDefenceMode = false;

		if (chip->brightness_level >= HIGH_BRIGHTNESS_CODE) {
			isHbMode = ams_highBrightness_alsCalcLux(chip,
				chip->chMinBuf, chip->chMaxBuf);
		} else if (chip->isAcLightDefenceMode) {
			isHbMode = ams_acLight_alsCalcLux(chip, chip->chAvg);
		} else {
			isHbMode = ams_alsCalcLux(chip, chip->chMinBuf);
		}
		chip->acLightDefenceOnCnt = 0;
		chip->acLightDefenceOffCnt = 0;

		chip->fifoCnt = 0;
		if (isHbMode)
			ams_report_high_brightness_events(chip);
		else
			ams_report_events(chip);
	}
}

static void ams_als_compute_data(struct ams_chip *chip)
{
	long long lux = 0;

	if (!(chip->rawdata.clear <= 1 &&
		(chip->rawdata.clear + chip->rawdata.red
		+ chip->rawdata.green + chip->rawdata.blue) <= 4)) {
		lux = (long long)DEFAULT_C_COEF * chip->rawdata.clear
			+ (long long)DEFAULT_R_COEF * chip->rawdata.red
			+ (long long)DEFAULT_G_COEF * chip->rawdata.green
			+ (long long)DEFAULT_B_COEF * chip->rawdata.blue;

		if (chip->cpl != 0)
			lux = lux / chip->cpl;
	}

	chip->lux = (int)lux;
	if (chip->count_log_time >= LIGHT_LOG_TIME) {
		SENSOR_INFO("r:%d, g:%d, b:%d, c:%d, cpl:%d, LUX:%d\n",
			chip->rawdata.red, chip->rawdata.green,
			chip->rawdata.blue, chip->rawdata.clear,
			chip->cpl, chip->lux);
		chip->count_log_time = 0;
	} else {
		chip->count_log_time++;
	}

	ams_report_events(chip);
}

static void ams_process_high(struct ams_chip *chip)
{
	u8 gain_reg;

	ams_i2c_read(chip->client, AMS_REG_ASTATUS, &gain_reg);
	chip->again = alsGain_conversion[gain_reg & 0x0f];
	ams_i2c_blk_read_direct(chip->client, AMS_REG_ADATAL0,
		(uint8_t *)(&chip->rawdata), 8);

	if (chip->previousGain != chip->again) {
		chip->cpl = chip->itime * chip->again / DEFAULT_DFG;
		chip->previousGain = chip->again;
		SENSOR_INFO("change gain %d\n", chip->again);
	} else {
		ams_als_compute_data(chip);
	}
}

static void ams_work_func_light(struct work_struct *work)
{
	struct ams_chip *chip = container_of((struct delayed_work *)work,
					struct ams_chip, main_work);

	if (chip->cur_algo_mode == ALS_ALGO_MID)
		ams_process_mid(chip);
	else
		ams_process_high(chip);

	schedule_delayed_work(&chip->main_work,
		nsecs_to_jiffies(atomic_read(&chip->delay)));
}

static ssize_t ams_light_brightness_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	chip->brightness_level = ASCII_TO_DEC(buf[0]) * 100
		+ ASCII_TO_DEC(buf[1]) * 10 + ASCII_TO_DEC(buf[2]);

	if (chip->als_enabled) {
		if (chip->cur_algo_mode == ALS_ALGO_MID
			&& chip->brightness_level == 0)
			ams_change_algo_mode(chip, ALS_ALGO_HIGH);
		else if (chip->cur_algo_mode == ALS_ALGO_HIGH
			&& chip->algo_mode == ALS_ALGO_MID
			&& chip->brightness_level != 0)
			ams_change_algo_mode(chip, ALS_ALGO_MID);
		else if (chip->cur_algo_mode == ALS_ALGO_MID)
			ams_change_fifo_sample(chip);
	}

	return size;
}

static ssize_t ams_light_algo_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	u8 algo_mode;
	int ret;
	struct ams_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 2, &algo_mode);
	if (ret || algo_mode > 1) {
		SENSOR_ERR("Invalid Argument %d, %d\n", ret, algo_mode);
		return ret;
	}

	if (chip->als_enabled)
		ams_change_algo_mode(chip, algo_mode + 1);

	chip->algo_mode = algo_mode + 1;

	return size;
}

static void ams_light_enable(struct ams_chip *chip)
{
	SENSOR_INFO("\n");

	chip->algo_mode = ALS_ALGO_MID;
	chip->cur_algo_mode = ALS_ALGO_NONE;
	chip->acLightDefenceOnCnt = 0;
	chip->acLightDefenceOffCnt = 0;
	chip->isAcLightDefenceMode = false;
	chip->count_log_time = LIGHT_LOG_TIME;

	ams_change_algo_mode(chip, chip->algo_mode);
}

static void ams_light_disable(struct ams_chip *chip)
{
	SENSOR_INFO("\n");
	cancel_delayed_work_sync(&chip->main_work);
	ams_i2c_write(chip->client, chip->shadow, AMS_REG_ENABLE, 0x01);
}

static ssize_t ams_light_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	u8 enable;
	int ret;
	struct ams_chip *chip = dev_get_drvdata(dev);

	ret = kstrtou8(buf, 0, &enable);
	if (ret) {
		SENSOR_ERR("Invalid Argument %d\n", ret);
		return ret;
	}

	if (enable == CMD_CAM_LUX_DISABLE) {
		input_report_rel(chip->input_dev,
			REL_LUX, EVENT_CAM_LUX_DISABLE + 1);
		input_sync(chip->input_dev);
		return size;
	}

	SENSOR_INFO("new_value = %u\n", enable);

	if (enable && !chip->als_enabled)
		ams_light_enable(chip);
	else if (!enable && chip->als_enabled)
		ams_light_disable(chip);

	chip->als_enabled = enable;

	return size;
}

static ssize_t ams_light_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t ams_light_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t ams_light_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t ams_light_vendor_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR_NAME);
}

static ssize_t ams_light_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d\n",
		chip->rawdata.red, chip->rawdata.green,
		chip->rawdata.blue, chip->rawdata.clear,
		chip->itime, chip->again);
}

static ssize_t ams_light_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d\n",
		chip->rawdata.red, chip->rawdata.green,
		chip->rawdata.blue, chip->rawdata.clear,
		chip->itime, chip->again);
}

static ssize_t ams_light_brightness_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	SENSOR_INFO("%d\n", chip->brightness_level);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->brightness_level);
}

static ssize_t ams_light_circle_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d.%2.2d %d.%2.2d %d.%2.2d\n",
		chip->light_position[0], chip->light_position[1],
		chip->light_position[2], chip->light_position[3],
		chip->light_position[4], chip->light_position[5]);
}

static DEVICE_ATTR(name, S_IRUGO, ams_light_name_show, NULL);
static DEVICE_ATTR(reg_data, S_IRUGO,
		ams_light_reg_data_show, ams_light_reg_data_store);
static DEVICE_ATTR(vendor, S_IRUGO, ams_light_vendor_show, NULL);
static DEVICE_ATTR(lux, S_IRUGO, ams_light_lux_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, ams_light_data_show, NULL);
static DEVICE_ATTR(brightness, 0664,
		ams_light_brightness_show, ams_light_brightness_store);
static DEVICE_ATTR(light_circle, 0440, ams_light_circle_show, NULL);
static DEVICE_ATTR(algo_mode, 0220, NULL, ams_light_algo_mode_store);

static struct device_attribute *sensor_attrs[] = {
	&dev_attr_name,
	&dev_attr_reg_data,
	&dev_attr_vendor,
	&dev_attr_lux,
	&dev_attr_raw_data,
	&dev_attr_brightness,
	&dev_attr_light_circle,
	&dev_attr_algo_mode,
	NULL,
};

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		ams_light_poll_delay_show, ams_light_poll_delay_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		ams_light_enable_show, ams_light_enable_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL,
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static int ams_parse_dt(struct device *dev, struct ams_chip *chip)
{
	struct device_node *np = dev->of_node;

	if (of_property_read_u32_array(np, "ams,light_position",
		chip->light_position,
		sizeof(chip->light_position) / sizeof(chip->light_position[0])) < 0) {
		SENSOR_ERR("no ams light_position, set as 0\n");
		return -ENODEV;
	}

	SENSOR_INFO("light-position - %u.%u %u.%u %u.%u\n",
		chip->light_position[0], chip->light_position[1],
		chip->light_position[2], chip->light_position[3],
		chip->light_position[4], chip->light_position[5]);

	return 0;
}

static int ams_init_input_device(struct ams_chip *chip)
{
	int ret = 0;
	struct input_dev *dev;

	/* allocate lightsensor input_device */
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = MODULE_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_REL, REL_DELTA);
	input_set_capability(dev, EV_REL, REL_2ND_MIN);
	input_set_capability(dev, EV_REL, REL_2ND_MAX);
	input_set_capability(dev, EV_REL, REL_LUX);

	input_set_drvdata(dev, chip);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		goto err_register_input_dev;
	}

	ret = sensors_create_symlink(&dev->dev.kobj, dev->name);
	if (ret < 0)
		goto err_create_sensor_symlink;

	ret = sysfs_create_group(&dev->dev.kobj, &light_attribute_group);
	if (ret < 0)
		goto err_create_sysfs_group;

	chip->input_dev = dev;

	return 0;

err_create_sysfs_group:
	sensors_remove_symlink(&dev->dev.kobj, dev->name);
err_create_sensor_symlink:
	input_unregister_device(dev);
err_register_input_dev:
	dev = NULL;
	return ret;
}

static int ams_get_id(struct ams_chip *chip)
{
	int ret;
	u8 id, rev, aux;

	ret = ams_i2c_read(chip->client, AMS_REG_AUXID, &aux);
	if (ret < 0)
		return ret;

	ret = ams_i2c_read(chip->client, AMS_REG_REVID, &rev);
	if (ret < 0)
		return ret;

	ret = ams_i2c_read(chip->client, AMS_REG_ID, &id);
	if (ret < 0)
		return ret;

	if (id != TCS3701_CHIP_ID) {
		SENSOR_ERR("device id:%02x fail!\n", id);
		return -1;
	}

	SENSOR_INFO("device id:%02x device revid:%02x device aux:%02x\n",
		id, rev, aux);

	return 0;
}

static int ams_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	int ret;
	struct ams_chip *chip;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_BYTE_DATA)) {
		SENSOR_ERR("i2c smbus byte data unsupported\n");
		return -EOPNOTSUPP;
	}

	chip = kzalloc(sizeof(struct ams_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	i2c_set_clientdata(client, chip);

	ret = ams_get_id(chip);
	if (ret < 0) {
		SENSOR_ERR("i2c failed, ret=%d\n", ret);
		goto err_i2c_fail;
	}

	ret = ams_parse_dt(&client->dev, chip);
	if (ret)
		SENSOR_ERR("ams_parse_dt failed, ret=%d\n", ret);

	ams_i2c_write(chip->client, chip->shadow, AMS_REG_ENABLE, 0x00);
	ams_i2c_write(chip->client, chip->shadow, AMS_REG_ENABLE, 0x01);
	ams_i2c_write(chip->client, chip->shadow, AMS_REG_AILTL, 0x00);
	ams_i2c_write(chip->client, chip->shadow, AMS_REG_AILTH, 0x00);
	ams_i2c_write(chip->client, chip->shadow, AMS_REG_AIHTL, 0xFF);
	ams_i2c_write(chip->client, chip->shadow, AMS_REG_AIHTH, 0xFF);
	ams_i2c_write(chip->client, chip->shadow, AMS_REG_PERS, 0x00);
	i2c_smbus_write_byte_data(chip->client, AMS_REG_INTENAB, 0x04);

	ret = ams_init_input_device(chip);
	if (ret) {
		SENSOR_ERR("Input device init failed, ret=%d\n", ret);
		goto err_init_input_device;
	}

	INIT_DELAYED_WORK(&chip->main_work, ams_work_func_light);
	atomic_set(&chip->delay, DEFAULT_RGBC_DELAY_MS * NSEC_PER_MSEC);
	chip->brightness_level = DEFAULT_BRIGHTNESS_LEVEL;

	/* set sysfs for light sensor */
	ret = sensors_register(&chip->ls_dev, chip, sensor_attrs, MODULE_NAME);
	if (ret < 0) {
		SENSOR_ERR("Sensor registration failed, ret=%d\n", ret);
		goto err_sensors_register;
	}

	SENSOR_INFO("Probe ok.\n");

	return 0;

err_sensors_register:
	sensors_remove_symlink(&chip->input_dev->dev.kobj,
		chip->input_dev->name);
	sysfs_remove_group(&chip->input_dev->dev.kobj, &light_attribute_group);
	input_unregister_device(chip->input_dev);
err_init_input_device:
err_i2c_fail:
	kfree(chip);
	SENSOR_ERR("Probe failed.\n");
	return ret;
}

static int ams_suspend(struct device *dev)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	SENSOR_INFO("\n");

	if (chip->als_enabled)
		ams_light_disable(chip);

	return 0;
}

static int ams_resume(struct device *dev)
{
	struct ams_chip *chip = dev_get_drvdata(dev);

	SENSOR_INFO("\n");

	if (chip->als_enabled) {
		ams_light_enable(chip);
		input_report_rel(chip->input_dev, REL_LUX,
			EVENT_INIT_MOVING_AVERAGE + 1);
		input_sync(chip->input_dev);
	}
	return 0;
}

static int ams_remove(struct i2c_client *client)
{
	return 0;
}

static void ams_shutdown(struct i2c_client *client)
{
	struct ams_chip *chip = i2c_get_clientdata(client);

	SENSOR_INFO("\n");

	if (chip->als_enabled)
		ams_light_disable(chip);
}

static struct i2c_device_id ams_idtable[] = {
	{ CHIP_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ams_idtable);

#ifdef CONFIG_OF
static const struct of_device_id ams_of_match[] = {
	{ .compatible = "ams,tcs3701",},
};
#else
#define ams_of_match NULL
#endif

static const struct dev_pm_ops ams_pm_ops = {
	.suspend = ams_suspend,
	.resume	= ams_resume,
};

static struct i2c_driver ams_driver = {
	.driver = {
		.name = CHIP_NAME,
		.pm = &ams_pm_ops,
		.of_match_table = ams_of_match,
	},
	.id_table = ams_idtable,
	.probe = ams_probe,
	.remove = ams_remove,
	.shutdown = ams_shutdown,
};

module_i2c_driver(ams_driver);

MODULE_DESCRIPTION("AMS Dvice Driver");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
