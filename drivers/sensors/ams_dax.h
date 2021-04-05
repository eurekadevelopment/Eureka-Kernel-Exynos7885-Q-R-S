/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*! \file
* \brief Device driver for monitoring ambient light intensity in (lux)
* proximity detection (prox), and color temperature functionality within the
* AMS-TAOS AMS family of devices.
*/

#ifndef __AMS_H
#define __AMS_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>

/* Default Params */
#define CH0 0
#define CH1 1
#define CH_MAX_CNT 2
#define MAX_REGS 256
#define JAMES_FIFO_MAX_CNT 5
#define ATIME_FACTOR 1000 /* based on 1ms */
#define AGAIN_FACTOR 512 /* based on 512x */
#define JAMES_DGF 12
#define CH0_COEF 497
#define CH1_COEF 433

#define ALS_ALGO_NONE 0
#define ALS_ALGO_MID 1
#define ALS_ALGO_HIGH 2

#define BRIGHTNESS_CODE_LEVEL_1 15
#define BRIGHTNESS_CODE_LEVEL_2 40
#define BRIGHTNESS_CODE_LEVEL_3 50
#define BRIGHTNESS_CODE_LEVEL_MAX 500

#define JAMES_LUX_HIGH_THRESHOLD 3000
#define JAMES_LUX_LOW_THRESHOLD  1000

#define HIGH_BRIGHTNESS_CODE 77

#define DEFAULT_DFG    2772
#define DEFAULT_C_COEF 573
#define DEFAULT_R_COEF -422
#define DEFAULT_G_COEF 981
#define DEFAULT_B_COEF -1059

#define AMS_ROUND_SHFT_VAL        4
#define AMS_ROUND_ADD_VAL         (1 << (AMS_ROUND_SHFT_VAL - 1))

typedef struct _fifo {
	uint16_t clear;
	uint16_t red;
	uint16_t green;
	uint16_t blue;
} adcDataSet_t;

struct ams_chip {
	struct device *ls_dev;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work main_work;
	adcDataSet_t rawdata;
	atomic_t delay;
	u8 shadow[MAX_REGS];
	u8 algo_mode;
	u8 cur_algo_mode;
	bool als_enabled;
	bool isAcLightDefenceMode;
	uint8_t fifodata[256];
	uint32_t data_buf[CH_MAX_CNT][64];
	uint32_t chMinBuf[CH_MAX_CNT][JAMES_FIFO_MAX_CNT];
	uint32_t chMaxBuf[CH_MAX_CNT][JAMES_FIFO_MAX_CNT];
	uint32_t chAvg[CH_MAX_CNT][JAMES_FIFO_MAX_CNT];
	int brightness_level;
	int fifoCnt;
	int acLightDefenceOnCnt;
	int acLightDefenceOffCnt;
	int ch0_delta;
	int ch0_2nd_max;
	int ch0_2nd_min;
	u32 cpl;
	u32 light_position[6];
	u32 itime;
	u32 again;
	int lux;
	int previousGain;
	int count_log_time;
};

#endif /* __AMS_H */
