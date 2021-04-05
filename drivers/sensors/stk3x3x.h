/*
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
 
#ifndef __STK3X3X_H__
#define __STK3X3X_H__

#define DRIVER_VERSION  "4.1.1"

#define STK3331_DEVICE_ID_VAL 0x53
#define STK3031_DEVICE_ID_VAL 0x36

/* Driver Settings */
#define STK_PS_MULTI_PERSIST
#define STK_INTELLI_PERSIST
#define STK_BGIR
#include <linux/wakelock.h>

/* Define Register Map */
#define STK3X3X_STATE_REG                   0x00
#define STK3X3X_PSCTRL_REG                  0x01
#define STK3X3X_ALSCTRL_REG                 0x02
#define STK3X3X_LEDCTRL_REG                 0x03
#define STK3X3X_INT_REG                     0x04
#define STK3X3X_WAIT_REG                    0x05
#define STK3X3X_THDH1_PS_REG                0x06
#define STK3X3X_THDH2_PS_REG                0x07
#define STK3X3X_THDL1_PS_REG                0x08
#define STK3X3X_THDL2_PS_REG                0x09
#define STK3X3X_THDH1_ALS_REG               0x0A
#define STK3X3X_THDH2_ALS_REG               0x0B
#define STK3X3X_THDL1_ALS_REG               0x0C
#define STK3X3X_THDL2_ALS_REG               0x0D
#define STK3X3X_FLAG_REG                    0x10
#define STK3X3X_DATA1_PS_REG                0x11
#define STK3X3X_DATA2_PS_REG                0x12
#define STK3X3X_DATA1_ALS_REG               0x13
#define STK3X3X_DATA2_ALS_REG               0x14
#define STK3X3X_DATA1_R_REG                 0x15
#define STK3X3X_DATA2_R_REG                 0x16
#define STK3X3X_DATA1_G_REG                 0x17
#define STK3X3X_DATA2_G_REG                 0x18
#define STK3X3X_DATA1_B_REG                 0x19
#define STK3X3X_DATA2_B_REG                 0x1A
#define STK3X3X_DATA1_C_REG                 0x1B
#define STK3X3X_DATA2_C_REG                 0x1C
#define STK3X3X_DATA1_OFFSET_REG            0x1D
#define STK3X3X_DATA2_OFFSET_REG            0x1E
#define STK3X3X_DATA_CTIR1_REG              0x20
#define STK3X3X_DATA_CTIR2_REG              0x21
#define STK3X3X_DATA_CTIR3_REG              0x22
#define STK3X3X_DATA_CTIR4_REG              0x23
#define STK3X3X_PDT_ID_REG                  0x3E
#define STK3X3X_RSRVD_REG                   0x3F
#define STK3X3X_ALSCTRL2_REG                0x4E
#define STK3X3X_INTELLI_WAIT_PS_REG         0x4F
#define STK3X3X_SW_RESET_REG                0x80
#define STK3X3X_SUNLIGHT_CHECK_REG          0xA7


/* Define state reg */

#define STK3X3X_STATE_EN_CT_AUTOK_SHIFT        4
#define STK3X3X_STATE_EN_INTELL_PRST_SHIFT     3
#define STK3X3X_STATE_EN_WAIT_SHIFT            2
#define STK3X3X_STATE_EN_ALS_SHIFT             1
#define STK3X3X_STATE_EN_PS_SHIFT              0

#define STK3X3X_STATE_EN_CT_AUTOK_MASK         0x10
#define STK3X3X_STATE_EN_INTELL_PRST_MASK      0x08
#define STK3X3X_STATE_EN_IRO_MASK              0x10
#define STK3X3X_STATE_EN_WAIT_MASK             0x04
#define STK3X3X_STATE_EN_ALS_MASK              0x02
#define STK3X3X_STATE_EN_PS_MASK               0x01

/* Define PS ctrl reg */
#define STK3X3X_PS_PRS_SHIFT            6
#define STK3X3X_PS_GAIN_SHIFT           4
#define STK3X3X_PS_IT_SHIFT             0

#define STK3X3X_PS_PRS_MASK             0xC0
#define STK3X3X_PS_GAIN_MASK            0x30
#define STK3X3X_PS_IT_MASK              0x0F

/* Define ALS ctrl reg */
#define STK3X3X_ALS_PRS_SHIFT           6
#define STK3X3X_ALS_GAIN_SHIFT          4
#define STK3X3X_ALS_IT_SHIFT            0

#define STK3X3X_ALS_PRS_MASK            0xC0
#define STK3X3X_ALS_GAIN_MASK           0x30
#define STK3X3X_ALS_IT_MASK             0x0F

/* Define LED ctrl reg */
#define STK3X3X_EN_CTIR_SHIFT           0
#define STK3X3X_EN_CTIRFC_SHIFT         1
#define STK3X3X_LED_IRDR_SHIFT          6
#define STK3X3X_EN_CTIR_MASK            0x01
#define STK3X3X_EN_CTIRFC_MASK          0x02
#define STK3X3X_LED_IRDR_MASK           0xC0

/* Define interrupt reg */
#define STK3X3X_INT_CTRL_SHIFT          7
#define STK3X3X_INT_INVALID_PS_SHIFT    5
#define STK3X3X_INT_ALS_SHIFT           3
#define STK3X3X_INT_PS_SHIFT            0
#define STK3X3X_INT_CTRL_MASK           0x80
#define STK3X3X_INT_INVALID_PS_MASK     0x20
#define STK3X3X_INT_ALS_MASK            0x08
#define STK3X3X_INT_PS_MASK             0x07
#define STK3X3X_INT_ALS                 0x08

/* Define flag reg */
#define STK3X3X_FLG_ALSDR_SHIFT         7
#define STK3X3X_FLG_PSDR_SHIFT          6
#define STK3X3X_FLG_ALSINT_SHIFT        5
#define STK3X3X_FLG_PSINT_SHIFT         4
#define STK3X3X_FLG_OUI_SHIFT           2
#define STK3X3X_FLG_IR_RDY_SHIFT        1
#define STK3X3X_FLG_NF_SHIFT            0

#define STK3X3X_FLG_ALSDR_MASK          0x80
#define STK3X3X_FLG_PSDR_MASK           0x40
#define STK3X3X_FLG_ALSINT_MASK         0x20
#define STK3X3X_FLG_PSINT_MASK          0x10
#define STK3X3X_FLG_OUI_MASK            0x04
#define STK3X3X_FLG_IR_RDY_MASK         0x02
#define STK3X3X_FLG_NF_MASK             0x01

/* Define ALS CTRL-2 reg */
#define  STK3X3X_ALSC_GAIN_SHIFT         0x04
#define  STK3X3X_ALSC_GAIN_MASK          0x30

/* Define INT-2 reg */
#define  STK3X3X_INT_ALS_DR_SHIFT        0x01
#define  STK3X3X_INT_PS_DR_SHIFT         0x00
#define  STK3X3X_INT_ALS_DR_MASK         0x02
#define  STK3X3X_INT_PS_DR_MASK          0x01

/* Define ALS/PS parameters */
#define  STK3X3X_PS_PRS1                0x00
#define  STK3X3X_PS_PRS2                0x40
#define  STK3X3X_PS_PRS4                0x80
#define  STK3X3X_PS_PRS16               0xC0

#define  STK3X3X_PS_GAIN1               0x00
#define  STK3X3X_PS_GAIN2               0x10
#define  STK3X3X_PS_GAIN4               0x20
#define  STK3X3X_PS_GAIN8               0x30

#define  STK3X3X_PS_IT100               0x00
#define  STK3X3X_PS_IT200               0x01
#define  STK3X3X_PS_IT400               0x02
#define  STK3X3X_PS_IT800               0x03
#define  STK3X3X_PS_IT1540              0x04

#define  STK3X3X_ALS_PRS1               0x00
#define  STK3X3X_ALS_PRS2               0x40
#define  STK3X3X_ALS_PRS4               0x80
#define  STK3X3X_ALS_PRS8               0xC0

#define  STK3X3X_ALS_GAIN1              0x00
#define  STK3X3X_ALS_GAIN4              0x10
#define  STK3X3X_ALS_GAIN16             0x20
#define  STK3X3X_ALS_GAIN64             0x30

#define  STK3X3X_ALS_IT25               0x00
#define  STK3X3X_ALS_IT50               0x01
#define  STK3X3X_ALS_IT100              0x02
#define  STK3X3X_ALS_IT200              0x03

#define  STK3X3X_ALSC_GAIN1             0x00
#define  STK3X3X_ALSC_GAIN4             0x10
#define  STK3X3X_ALSC_GAIN16            0x20
#define  STK3X3X_ALSC_GAIN64            0x30

#define  STK3X3X_LED_3_125mA            0x00
#define  STK3X3X_LED_6_25mA             0x20
#define  STK3X3X_LED_12_5mA             0x40
#define  STK3X3X_LED_25mA               0x60
#define  STK3X3X_LED_50mA               0x80
#define  STK3X3X_LED_100mA              0xA0
#define  STK3X3X_LED_150mA              0xC0

#define  STK3X3X_WAIT20                 0x0C
#define  STK3X3X_WAIT50                 0x20
#define  STK3X3X_WAIT100                0x40

#define STK3X3X_INT_NF_EN               0x01			

#define  STK3X3X_INTELL_13              0x21
#define  STK3X3X_INTELL_20              0x32
#define  STK3X3X_BGIR_PS_INVALID        0x28
/* misc define */
#define MIN_ALS_POLL_DELAY_NS       60000000

#define STK3X3X_ALS_THRESHOLD           10

#define STK3X3X_PS_BGIR_INVALID         0x7FFF0001
#define STK3X3X_PS_DATA_UNAVAILABLE     0x7FFF0002

#define DEVICE_NAME                     "stk_alps"
#define ALS_NAME                        "lightsensor-level"
#define PS_NAME                         "proximity"
#define STK3X3X_CALI_FILE               "/persist/sensors/stkalpscali.conf"
#define STK3X3X_PID_LIST_NUM            12
#define STK_PS_CALI_DATA_NUM            3
#define STK3X3X_PS_BGIR_THRESHOLD       0x0//0x64
#define STK3X3X_ALS_CALI_TARGET_LUX     500
#define STK3X3X_PS_CALI_TIMES           5
#define STK3X3X_PS_CALI_MAX_CROSSTALK   3000
#define STK3X3X_PS_CALI_DIFF            40
#define STK3X3X_REG_READ(stk, reg)  ((stk)->bops->read((stk), reg))
#define STK3X3X_REG_WRITE(stk, reg, val)    ((stk)->bops->write((stk), reg, val))
#define STK3X3X_REG_WRITE_BLOCK(stk, reg, val, len)    ((stk)->bops->write_block((stk), reg, val, len))
#define STK3X3X_REG_READ_MODIFY_WRITE(stk, reg, val, mask)  ((stk)->bops->read_modify_write((stk), reg, val, mask))
#define STK3X3X_REG_BLOCK_READ(stk, reg, count, buf)    ((stk)->bops->read_block((stk), reg, count, buf))
#define STK_ABS(x)              ((x < 0)? (-x):(x))

#ifdef STK_ALS_MID_FIR
typedef struct
{
    uint16_t raw[STK_ALS_MID_FIR_LEN];
    uint32_t number;
    uint32_t index;
} stk3x3x_data_filter;
#endif

typedef enum
{
    STK3X3X_NONE        = 0x0,
    STK3X3X_ALS         = 0x1,
    STK3X3X_PS          = 0x2,
    STK3X3X_ALL         = 0x4,
} stk3x3x_sensor_type;

typedef enum
{
    STK3X3X_CALI_TIMER_ALS,
    STK3X3X_CALI_TIMER_PS,
} stk3x3x_cali_timer_type;

typedef struct stk3x3x_register_table
{
    uint8_t address;
    uint8_t value;
    uint8_t mask_bit;
} stk3x3x_register_table;

typedef enum
{
    STK3X3X_PRX_NEAR_BY,
    STK3X3X_PRX_FAR_AWAY,
} stk3x3x_prx_nearby_type;

typedef enum
{
    STK3X3X_POCKET_UNKNOWN = 0,
    STK3X3X_POCKET_NEAR_BY = 1,
    STK3X3X_POCKET_FAR_AWAY = 2
} stk3x3x_pocket_nearby_type;

typedef struct stk3x3x_irq_info
{
    bool                        irq_is_active;
    bool                        irq_is_exist;
} stk3x3x_irq_info;

typedef enum
{
    STK3X3X_FIRST_CAL,
    STK3X3X_CAL_ONGOING,
    STK3X3X_CAL_DISABLED,
    STX3X3X_CAL_SKIP
} stk3x3x_cal_status;

typedef struct stk3x3x_data
{
    struct i2c_client *client;
    struct device *dev;
    struct stk3x3x_platform_data *pdata;
    const struct stk3x3x_bus_ops *bops;
    struct mutex            control_mutex;
    struct input_dev        *prox_input_dev;
    int                     irq_gpio;
    int32_t                 irq;
    struct work_struct      prox_irq_work;
    struct workqueue_struct *prox_irq_wq;
    stk3x3x_irq_info        irq_status;
    int                     offset;
#if (defined(STK_ALS_CALI) || defined(STK_PS_CALI))
    struct hrtimer          cali_timer;
    struct workqueue_struct *stk_cali_wq;
    struct work_struct      stk_cali_work;
    ktime_t                 cali_poll_delay;
#endif
    atomic_t                recv_reg;
    bool                    first_init;

    struct wake_lock        prox_wakelock;

    bool                    enable;
    bool                    first_limit_skip;
    uint16_t                prox_default_thd_h;
    uint16_t                prox_default_thd_l;
    bool                    pocket_enable;
    bool                    pocket_running; 
    uint16_t                prox_thd_h;
    uint16_t                prox_thd_l;
    uint16_t                thd_h_offset;
    uint16_t                sunlight_thd_h;
    uint16_t                sunlight_thd_l;
    uint16_t                first_cal_adc_limit;
    uint16_t                first_cal_thd_h;
    uint16_t                first_cal_thd_l;
    uint16_t                adc;
    int                     avg[3];
    struct hrtimer          prox_timer;
    struct workqueue_struct *prox_wq;
    struct work_struct      work_prox;
    ktime_t	                 prox_poll_delay;

    struct workqueue_struct *prox_cal_wq;
    struct workqueue_struct *prox_pocket_wq;
    struct work_struct      work_cal_prox;
    struct work_struct      work_pocket;
    char                    cal_status;
    bool                    factory_cal;
    int                     check_far_state;
    int                     pocket_prox;
    uint8_t                 intel_prst;
    uint8_t                 ps_it;
    uint8_t                 close_cnt;
} stk3x3x_data;

struct stk3x3x_bus_ops
{
    u16 bustype;
    int (*read)(struct stk3x3x_data *, unsigned char);
    int (*read_block)(struct stk3x3x_data *, unsigned char, int, void *);
    int (*write)(struct stk3x3x_data *, unsigned char, unsigned char);
    int (*write_block)(struct stk3x3x_data *, unsigned char, unsigned char *, unsigned char);
    int (*read_modify_write)(struct stk3x3x_data *, unsigned char, unsigned char, unsigned char);
};

int stk3x3x_remove(struct i2c_client *client);
int stk3x3x_suspend(struct device *dev);
int stk3x3x_resume(struct device *dev);

int32_t stk3x3x_cali_als(struct stk3x3x_data *drv_data);
int32_t stk3x3x_cali_ps(struct stk3x3x_data *drv_data);
void stk3x3x_get_reg_default_setting(uint8_t reg, uint16_t* value);
int32_t stk3x3x_ps_get_data_without_notify(struct stk3x3x_data *drv_data, uint16_t *raw_data);
int32_t stk3x3x_set_ps_thd(struct stk3x3x_data *drv_data, uint16_t thd_h, uint16_t thd_l);

int stk3x3x_ps_val(struct stk3x3x_data *drv_data);
int32_t stk3x3x_ps_offset_tune(struct stk3x3x_data *drv_data);

#endif // __STK3X3X_H__
