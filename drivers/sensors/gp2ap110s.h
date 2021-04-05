#ifndef __GP2AP110S_H__
#define __GP2AP110S_H__

//// Register address ////
#define REG_COM1              0x80
#define REG_COM2              0x81
#define REG_COM3              0x82
#define REG_COM4              0x83

#define REG_D0_LSB              0x90
#define REG_D0_MSB              0x91
#define REG_REV_CODE            0xA0
#define REG_DYNAMIC_CAL_RESULT  0xC0

#define DYNAMIC_CALIBRATION_DELAY   1000
#define OFFSET_TUNE_DELAY 1000
#define AUTO_CALIB_OFFSET_MAX 4500

#define OFFSET_DELTA 40
#define MAX_OFFSET 5000

struct gp2ap_data
{
	struct mutex		mutex_ps_onoff;
	struct mutex		mutex_enable;
	struct mutex		mutex_interrupt;
	struct i2c_client	*client;
	struct hrtimer		prox_timer;
	struct workqueue_struct	*prox_wq;
	struct work_struct	work_prox;
	struct delayed_work offset_work;
	ktime_t			prox_poll_delay;
	atomic_t		prox_enable;
	int			avg[3];
	int			p_out;
	int			ps_irq;
	int			irq_flags;
	struct input_dev	*ps_input_dev;
	struct device		*dev;
	struct work_struct	ps_int_work;
	int			ps_enabled;
	int			ps_distance;
	int			ps_count;
	int			calib_target;
	int         led_reg_val_1;
	int			ps_high_th_1;
	int			ps_low_th_1;
	int			ps_high_th_2;
	int			ps_low_th_2;
	int         led_reg_val_2;
	int         bytes;	
	int			prox_settings;
	int         led_reg_val;
	int         ps_high_th;
	int         ps_low_th;
	int         settings_thd_low;
	int         settings_thd_high;
	int         force_close_thd_low;
	int         force_close_thd_high;
	int			dynamic_calib_enabled;
	int			dynamic_calib_done;
	int			tune_adc_count;
	int			zero_detect;
	int			pre_test;
	bool			handle_high_offset;
	u16			high_offset;
	u16			min_close_offset;
	enum of_gpio_flags	irq_gpio_flags;
};

#endif
