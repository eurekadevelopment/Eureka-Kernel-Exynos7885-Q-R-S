#ifndef __ZH915_H__
#define __ZH915_H__


#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/kthread.h>

#define HAPTICS_DEVICE_NAME "zh915"

#define	REG_MODE				0x00
#define MODE_MASK				0x07
#define STOP_MODE				0x01
#define PWM_MODE				0x04
#define I2C_MODE				0x05

#define	REG_STRENGTH_WRITE			0x01

#define	REG_CONTROL				0x03
#define CONTROL_POWER_CAL_MASK			0x08
#define CONTROL_LOOP_MASK			0x04
#define CONTROL_BRAKE_MASK			0x02
#define CONTROL_MOTOR_MASK			0x01

#define	REG_STRENGTH_READ			0x0C

#define REG_ADAPTIVE_OVERDRIVE			0x2A
#define OVERDRIVE_ENABLE_MASK			0x80
#define OVERDRIVE_VALUE_MASK			0x7E

#define REG_DS_RATIO				0x2B
#define DS_RATIO_MASK				0x3F

#define REG_RESONANCE_FREQ			0x2E

#define REG_SLAVE_ADDRESS			0x2F
#define SLAVE_ADDRESS_MASK			0x7F

#define MAX_TIMEOUT			 	10000 /* 10s */

#define MAX_INTENSITY				10000
#define VIB_BUFSIZE				30
#define PACKET_MAX_SIZE				1000
#define ZH915_MAX_DEC		127

#define YES					1
#define NO					0

#define HOMEKEY_PRESS_FREQ              5
#define HOMEKEY_RELEASE_FREQ            6
#define HOMEKEY_DURATION                7

struct vib_packet {
	int time;
	int intensity;
	int freq;
	int overdrive;
};

enum enable_type {
	DISABLE,
	ENABLE
};

enum actuator_type {
	LRA,
	ERM
};

enum loop_type {
	CLOSE_LOOP,
	OPEN_LOOP	
};

enum motor_control_type {
	IFMPIC_TYPE = 1,
	EXTERNAL_DRIVING_IC	
};

struct zh915_data {
	unsigned char mnDeviceID;
	struct device *dev;
	struct i2c_client *i2c;
	struct regmap *mpRegmap;
	struct device *motor_dev;

	int running;
	int packet_running;

	int intensity;
	int overdrive_state;

	/* for multi-frequency */
	int multi_frequency;
	u8 freq;
	u8 duty;
	u8 max_duty;
	int freq_num;
	u32 *multi_freq;
	u32 *multi_duty;

	int boost_en;

	int timeout;
	bool f_packet_en;
	int packet_size;
	int packet_cnt;
	struct vib_packet test_pac[PACKET_MAX_SIZE];

	int force_touch_intensity;

	/* for ic degug */
	u8 ic_reg_address;  /* save register address from adb request */

	struct wake_lock wklock;
	struct hrtimer timer;
	struct mutex lock;
	struct kthread_worker kworker;
	struct kthread_work vibrator_work;
	struct timed_output_dev to_dev;		
};
#endif
