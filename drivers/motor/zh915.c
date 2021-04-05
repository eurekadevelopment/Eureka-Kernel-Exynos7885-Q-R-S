#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/zh915.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include "../staging/android/timed_output.h"
#include <linux/sec_sysfs.h>
#include <linux/kthread.h>
#include <linux/of_gpio.h>
#include <linux/printk.h>
#if defined(CONFIG_SSP_MOTOR_CALLBACK)
#include <linux/ssp_motorcallback.h>
#endif

static struct zh915_data *g_drvdata;

static int zh915_reg_read(struct zh915_data *zh915data, unsigned char reg)
{
	unsigned int val;
	int ret;
	
	ret = regmap_read(zh915data->mpRegmap, reg, &val);
    
	if (ret < 0){
		dev_err(zh915data->dev, 
			"%s reg=0x%x error %d\n", __FUNCTION__, reg, ret);
		return ret;
	}
	else
		return val;
}

static int zh915_reg_write(struct zh915_data *zh915data, 
	unsigned char reg, unsigned char val)
{
	int ret;
	
	ret = regmap_write(zh915data->mpRegmap, reg, val);
	if (ret < 0){
		dev_err(zh915data->dev, 
			"%s reg=0x%x, value=0%x error %d\n", 
			__FUNCTION__, reg, val, ret);
	}
	
	return ret;
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct zh915_data *zh915data = container_of(dev, struct zh915_data, to_dev);

	if (hrtimer_active(&zh915data->timer)) {
		ktime_t r = hrtimer_get_remaining(&zh915data->timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void zh915_stop(struct zh915_data *zh915data)
{
	if(zh915data->running == YES) {
		hrtimer_cancel(&zh915data->timer);

		zh915_reg_write(zh915data, REG_MODE, STOP_MODE);

		zh915data->running = NO;
		wake_unlock(&zh915data->wklock);
		pr_info("[VIB] %s\n", __func__);
	}
}

static int zh915_haptic_set_frequency(struct zh915_data *zh915data, int num)
{
	int set_reg;
	int h_freq = 0;
	
	if (num >= 0 && num < zh915data->multi_frequency) {
		zh915data->freq = zh915data->multi_freq[num];
		zh915data->duty = zh915data->multi_duty[num];
		zh915data->freq_num = num;
		
//		printk("[VIB] %s: freq_num=%d,  RESONANCE FREQUENCY: 0x%x\n", __func__, num, zh915data->freq);

	} else if (num >= 1200 && num <= 3500) {

		/**
		 *   Calc frequency
		 *   Resonace_Rrequency[7:0] = (12.5Mhz/Frequency - 50000)/512
		 *   freq_num = 1650 : 165.0 Hz
		 */
		h_freq = num / 10;
		set_reg = (12500000/h_freq - 50000) / 512;
		
		zh915data->freq = set_reg;
		/* Set default duty value */
		if (zh915data->overdrive_state)
			zh915data->duty = zh915data->multi_duty[HOMEKEY_PRESS_FREQ];
		else
			zh915data->duty = zh915data->multi_duty[0];

//		printk("[VIB] %s: REG_RESONANCE_FREQ: 0x%x : strength = 0x%x \n", __func__, zh915data->freq, zh915data->duty);		
	} else {
		pr_err("%s out of range %d\n", __func__, num);
		return -EINVAL;
	}

	return 0;
}

/* function paramter : intensity meas required strength value */
/*                                Max resolution is 10000                        */

static int zh915_haptic_set_intensity(struct zh915_data *zh915data, int intensity)
{
	u8 duty = zh915data->duty;
	int req_duty = 0;
	
	if (intensity < -(MAX_INTENSITY) || MAX_INTENSITY < intensity) {
		pr_err("%s out of range %d\n", __func__, intensity);
		return -EINVAL;
	}

	if (MAX_INTENSITY == intensity)
		req_duty = duty;
	else if (intensity == -(MAX_INTENSITY))
		req_duty = ~duty;
	else if (0 != intensity) {		
		req_duty = duty * abs(intensity) / MAX_INTENSITY;
		//duty = (u8)((ZH915_MAX_DEC * req_duty / 100) +1);

		if (intensity < 0)
			req_duty = ~ req_duty;			
	}
	
//	printk("[VIB] %s : intensity = 0x%x /duty = 0x%x / req_duty = 0x%x \n", __func__, intensity, (unsigned int)duty, req_duty);
	if (intensity == 0)
		req_duty = 0x00;
	
	zh915data->intensity = intensity;
	zh915data->duty = req_duty;

	return 0;
}

static int zh915_haptic_set_overdrive_state(struct zh915_data *zh915data, int overdrive)
{
	if(overdrive)
		zh915data->overdrive_state = true;
	else
		zh915data->overdrive_state = false;
	return 0;
}

static void zh915_haptic_engine_set_packet(struct zh915_data *zh915data,
		struct vib_packet haptic_packet)
{
	int frequency = haptic_packet.freq;
	int intensity = haptic_packet.intensity;
	int overdrive = haptic_packet.overdrive;
	int prev_packet=0;

	if (!zh915data->f_packet_en) {
		pr_err("[VIB] haptic packet is empty\n");
		return;
	}

	zh915_haptic_set_overdrive_state(zh915data, overdrive);
	zh915_haptic_set_frequency(zh915data, frequency);
	zh915_haptic_set_intensity(zh915data, intensity);

	if (intensity == 0) {
		if (zh915data->packet_running) {
//			pr_info("[VIB] haptic engine: motor stop\n");
			zh915_reg_write(zh915data, REG_MODE, STOP_MODE);
		}
		zh915data->packet_running = false;
	} else {
		if (!zh915data->packet_running) {
//			pr_info("[VIB] haptic engine [%d] : motor run : freq=0x%x, duty = 0x%x\n", zh915data->packet_cnt, zh915data->freq, zh915data->duty);
			
			zh915_reg_write(zh915data, REG_CONTROL, CONTROL_LOOP_MASK);  // LPA, LOOP OPEN
			zh915_reg_write(zh915data, REG_RESONANCE_FREQ, zh915data->freq); // Frequency set
			zh915_reg_write(zh915data, REG_STRENGTH_WRITE, zh915data->duty); // strength set
			zh915_reg_write(zh915data, REG_MODE, I2C_MODE);
		} else {			
			zh915_reg_write(zh915data, REG_RESONANCE_FREQ, zh915data->freq); // Frequency set
			if (zh915data->packet_cnt)
				prev_packet = zh915data->packet_cnt -1;
			if (intensity != zh915data->test_pac[prev_packet].intensity)
				zh915_reg_write(zh915data, REG_STRENGTH_WRITE, zh915data->duty); // strength set				
		}
		zh915data->packet_running = true;
	}

	pr_info("[VIB] haptic_engine : freq:%d, intensity:%d, time:%d overdrive: %d\n", frequency, intensity, zh915data->timeout, zh915data->overdrive_state);
}

static ssize_t haptic_engine_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct zh915_data *zh915data
		= container_of(to_dev, struct zh915_data, to_dev);
	int i = 0, _data = 0, tmp = 0;

	if (buf == NULL) {
		pr_err("%s, buf is NULL, Please check packet data again\n",
			__func__);
		zh915data->f_packet_en = false;
		return count;
	}

	if (sscanf(buf, "%d", &_data) == 1) {
		if (_data > PACKET_MAX_SIZE * 4)
			pr_info("%s, [%d] packet size over\n", __func__, _data);
		else {
			zh915data->packet_size = _data / 4;
			zh915data->packet_cnt = 0;
			zh915data->f_packet_en = true;

			buf = strstr(buf, " ");

			for (i = 0; i < zh915data->packet_size; i++) {
				for (tmp = 0; tmp < 4; tmp++) {
					if (sscanf(buf++, "%d", &_data) == 1) {
						switch (tmp){
							case 0:
								zh915data->test_pac[i].time = _data;
								break;
							case 1:
								zh915data->test_pac[i].intensity = _data;
								break;
							case 2:
								zh915data->test_pac[i].freq = _data;
								break;
							case 3:
								zh915data->test_pac[i].overdrive = _data;
								break;
						}
						buf = strstr(buf, " ");
					} else {
						pr_info("%s packet data error\n", __func__);
						zh915data->f_packet_en = false;
						return count;
					}
				}
			}
		}
	}

	return count;
}

static ssize_t force_touch_intensity_store(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
		struct zh915_data *zh915data
		= container_of(tdev, struct zh915_data, to_dev);
	int intensity = 0, ret = 0;

	ret = kstrtoint(buf, 0, &intensity);
	if (ret) {
		pr_err("fail to get intensity\n");
		return -EINVAL;
	}

	pr_info("[VIB] %s %d\n", __func__, intensity);

	zh915data->force_touch_intensity  = intensity;
	ret = zh915_haptic_set_intensity(zh915data, intensity);
	if (ret)
		return ret;

	return count;
}

static ssize_t force_touch_intensity_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
		struct zh915_data *zh915data
		= container_of(tdev, struct zh915_data, to_dev);

	pr_info("[VIB] %s %d\n", __func__, zh915data->force_touch_intensity);
	
	return snprintf(buf, VIB_BUFSIZE, "force touch intensity: %u\n", zh915data->force_touch_intensity);
}

static DEVICE_ATTR(force_touch_intensity, 0660, force_touch_intensity_show, force_touch_intensity_store);

static ssize_t haptic_engine_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct zh915_data *zh915data
		= container_of(to_dev, struct zh915_data, to_dev);
	int i = 0;
	char *bufp = buf;

	bufp += snprintf(bufp, VIB_BUFSIZE, "\n");
	for (i = 0; i < zh915data->packet_size && zh915data->f_packet_en; i++) {
		bufp+= snprintf(bufp, VIB_BUFSIZE, "%u,", zh915data->test_pac[i].time);
		bufp+= snprintf(bufp, VIB_BUFSIZE, "%u,", zh915data->test_pac[i].intensity);
		bufp+= snprintf(bufp, VIB_BUFSIZE, "%u,", zh915data->test_pac[i].freq);
		bufp+= snprintf(bufp, VIB_BUFSIZE, "%u,", zh915data->test_pac[i].overdrive);
	}
	bufp += snprintf(bufp, VIB_BUFSIZE, "\n");

	return strlen(buf);
}

static DEVICE_ATTR(haptic_engine, 0660, haptic_engine_show, haptic_engine_store);

static void vibrator_enable( struct timed_output_dev *dev, int value)
{
	struct zh915_data *zh915data = 
		container_of(dev, struct zh915_data, to_dev);
	struct pinctrl *i2c_pinctrl;
	int cnt = 1000;
	
	pr_info("[VIB] %s %dms\n", __func__, value);

	flush_kthread_worker(&zh915data->kworker);
	hrtimer_cancel(&zh915data->timer);

	mutex_lock(&zh915data->lock);
	
	i2c_pinctrl = devm_pinctrl_get_select(zh915data->dev, "motor_en_high");
	if (IS_ERR(i2c_pinctrl))
		dev_err(zh915data->dev, "[VIB]: %s : motor_en_high / could not set motor_en pins\n",__func__);
//	usleep_range(1 * 1000, 1 * 1000);
	
	value = min_t(int, value, MAX_TIMEOUT);
	zh915data->timeout = value;
	zh915data->packet_running = false;

	if (value > 0) {
		wake_lock(&zh915data->wklock);
		
		gpio_direction_output(zh915data->boost_en, 1);
		while (!gpio_get_value(zh915data->boost_en))
		{
			mdelay(1);
			if (--cnt < 0 )
			{
				printk("[VIB] : %s : boost_en high fail \n",__func__);
				break;
			}
		}
		/* zh915 startup time need */
		msleep(5);
		
		if (zh915data->f_packet_en) {
			zh915data->timeout = zh915data->test_pac[0].time;
			zh915_haptic_engine_set_packet(zh915data, zh915data->test_pac[0]);
#if defined(CONFIG_SSP_MOTOR_CALLBACK)
			if(zh915data->intensity == 0)
				setSensorCallback(false, 0);
			else
				setSensorCallback(true, zh915data->timeout);
#endif
		} else {
			zh915data->running = YES;
			// intensity
			// Frequency write
			zh915_reg_write(zh915data, REG_CONTROL, CONTROL_LOOP_MASK);  // LPA, LOOP OPEN
			zh915_reg_write(zh915data, REG_RESONANCE_FREQ, zh915data->freq); // Frequency set
			zh915_reg_write(zh915data, REG_STRENGTH_WRITE, zh915data->duty); // strength set
			zh915_reg_write(zh915data, REG_MODE, I2C_MODE & MODE_MASK);       // motor enable
#if defined(CONFIG_SSP_MOTOR_CALLBACK)
			setSensorCallback(true, value);
#endif
			printk("[VIB] %s: motor on : frequency = 0x%x, duty = 0x%x, intensity = 0x%x, freq_num = %d \n",__func__, zh915data->freq, zh915data->duty, zh915data->intensity, zh915data->freq_num);
		}
		pr_info("[VIB] timer setting\n");
		hrtimer_start(&zh915data->timer, 
			ns_to_ktime((u64)zh915data->timeout * NSEC_PER_MSEC), HRTIMER_MODE_REL);
	}
	else if (value == 0) {
		zh915data->f_packet_en = false;
		zh915data->packet_cnt = 0;
		zh915data->packet_size = 0;

		if (zh915data->running == YES){
			pr_info("[VIB] turn OFF\n");
			zh915_stop(zh915data);
		}
		else{
			pr_info("[VIB] already OFF\n");		
		}
#if defined(CONFIG_SSP_MOTOR_CALLBACK)
		setSensorCallback(false, 0);
#endif
		gpio_direction_output(zh915data->boost_en, 0);
	}
	
	mutex_unlock(&zh915data->lock);
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct zh915_data *zh915data = 
		container_of(timer, struct zh915_data, timer);
	pr_info("[VIB] %s\n", __func__);
	
	queue_kthread_work(&zh915data->kworker, &zh915data->vibrator_work);

	return HRTIMER_NORESTART;
}

static void vibrator_work_routine(struct kthread_work *work)
{
	struct zh915_data *zh915data = 
		container_of(work, struct zh915_data, vibrator_work);
	struct hrtimer *timer = &zh915data->timer;
	pr_info("[VIB] %s\n", __func__);

	mutex_lock(&zh915data->lock);

	if (zh915data->f_packet_en) {
		if (++zh915data->packet_cnt >= zh915data->packet_size) {
			zh915data->f_packet_en = false;
			zh915data->packet_cnt = 0;
			zh915data->packet_size = 0;
		} else {
			zh915_haptic_engine_set_packet(zh915data, zh915data->test_pac[zh915data->packet_cnt]);
#if defined(CONFIG_SSP_MOTOR_CALLBACK)
			if(drvdata->intensity == 0)
				setSensorCallback(false, 0);
			else
				setSensorCallback(true, zh915data->test_pac[zh915data->packet_cnt].time);
#endif
			hrtimer_start(timer, ns_to_ktime((u64)zh915data->test_pac[zh915data->packet_cnt].time * NSEC_PER_MSEC), HRTIMER_MODE_REL);
			goto unlock;
		}
	}

	zh915_stop(zh915data);
	gpio_direction_output(zh915data->boost_en, 0);
#if defined(CONFIG_SSP_MOTOR_CALLBACK)
	setSensorCallback(false,0);
#endif
unlock:
	mutex_unlock(&zh915data->lock);
}
 
static int Haptics_init(struct zh915_data *zh915data)
{
	int ret = 0;
	struct task_struct *kworker_task;

	zh915data->to_dev.name = "vibrator";
	zh915data->to_dev.get_time = vibrator_get_time;
	zh915data->to_dev.enable = vibrator_enable;

	ret = timed_output_dev_register(&(zh915data->to_dev));
	if ( ret < 0){
		dev_err(zh915data->dev, 
			"zh915: fail to create timed output dev\n");
		return ret;
	}
	
	hrtimer_init(&zh915data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	zh915data->timer.function = vibrator_timer_func;

	init_kthread_worker(&zh915data->kworker);
	kworker_task = kthread_run(kthread_worker_fn,
			&zh915data->kworker, "zh915_haptic");
	if (IS_ERR(kworker_task)) {
		pr_err("Failed to create message pump task\n");
	}
	init_kthread_work(&zh915data->vibrator_work, vibrator_work_routine);

	wake_lock_init(&zh915data->wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&zh915data->lock);

	return 0;
}

static struct regmap_config zh915_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static void zh915_reg_init(struct zh915_data *zh915data) {

	int val;

//	zh915_reg_write(zh915data, REG_MODE, I2C_MODE & MODE_MASK);
	zh915_reg_write(zh915data, REG_CONTROL, CONTROL_LOOP_MASK);
	/* Power Calibration disable, Open Loop,
	 * Brake disable, LRA motor
	 */
	zh915_reg_write(zh915data, REG_RESONANCE_FREQ, 0x33); // 164Hz
	zh915_reg_write(zh915data, REG_STRENGTH_WRITE, 0x78); // 2.0Vrms

	val = zh915_reg_read(zh915data, REG_CONTROL);
	printk("[VIB] : %s : zh915 REG_CONTROL(0x02) = 0x%x \n", __func__, val);
	val = zh915_reg_read(zh915data, REG_RESONANCE_FREQ);
	printk("[VIB] : %s : zh915 REG_RESONANCE_FREQ(0x%x) = 0x%x \n", __func__, REG_RESONANCE_FREQ, val);
	val = zh915_reg_read(zh915data, REG_STRENGTH_READ);
	printk("[VIB] : %s : zh915 REG_STRENGTH_READ(0x%x) = 0x%x \n", __func__, REG_STRENGTH_READ, val);
}

static ssize_t set_chip_register(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	struct zh915_data *zh915data = dev_get_drvdata(dev);
	char buff[10] = {0,};
	int cnt, ret;
	u16 reg;
	u8 set_reg;
	u8 value, val;

	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : 9;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';

	ret = kstrtou16(buff, 0, &reg);
	if (ret != 0) {
		dev_err(dev, "[VIB] fail to get period.\n");
		return count;
	}
	set_reg = (u8)((reg & 0xFF00) >> 8);
	value =(u8)(reg & 0xFF);

	zh915_reg_write(zh915data, set_reg, value);

	if (set_reg == 0x00)
	{
		// Print Current Setting value
		val = zh915_reg_read(zh915data, REG_CONTROL);
		printk("[VIB] : %s : zh915 REG_CONTROL(0x02) = 0x%x \n", __func__, val);
		val = zh915_reg_read(zh915data, REG_RESONANCE_FREQ);
		printk("[VIB] : %s : zh915 REG_RESONANCE_FREQ(0x%x) = 0x%x \n", __func__, REG_RESONANCE_FREQ, val);
		val = zh915_reg_read(zh915data, REG_STRENGTH_READ);
		printk("[VIB] : %s : zh915 REG_STRENGTH_READ(0x%x) = 0x%x \n", __func__, REG_STRENGTH_READ, val);
		
		val = zh915_reg_read(zh915data, REG_MODE);
		printk("[VIB] : %s : zh915 REG_MODE(0x%x) = 0x%x \n", __func__, REG_MODE, val);
		
	}
	printk("[VIB] Set Register : 0x%x Value : 0x%x\n", set_reg, value);

	return count;
}

static ssize_t get_chip_register(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	struct zh915_data *zh915data = dev_get_drvdata(dev);
	char buff[10] = {0,};
	int cnt, ret;
	u8 reg;
	int value;

	cnt = count;
	cnt = (buf[cnt-1] == '\n') ? cnt-1 : cnt;
	memcpy(buff, buf, cnt);
	buff[cnt] = '\0';

	ret = kstrtou8(buff, 0, &reg);
	if (ret != 0) {
		dev_err(dev, "[VIB] fail to get register.\n");
		return count;
	}
	zh915data->ic_reg_address = reg;
	value = zh915_reg_read(zh915data, reg);
	printk("[VIB] Get Register : 0x%x Value : 0x%x\n", reg, value);

	return count; //sprintf(buf, "Register :  0x%x Value :  0x%x\n", reg, value);
}

static ssize_t init_chip_register(struct device *dev,
		struct device_attribute *devattr, char *buf)

{
	struct zh915_data *zh915data = dev_get_drvdata(dev);

	zh915_reg_init(zh915data);

	printk("[VIB] Init Register\n");

	return sprintf(buf, "Init success\n");
}

static ssize_t zh915_power_control(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	u16 val;
	int ret;
	struct zh915_data *zh915data = dev_get_drvdata(dev);
	
	ret = kstrtou16(buf, 0, &val);
	if (val)
	{
		gpio_direction_output(zh915data->boost_en, 1);
		while (!gpio_get_value(zh915data->boost_en));
		printk("[VIB] %s : motor_boost_en high \n",__func__);
		mdelay(5);
	} else {
	 	gpio_direction_output(zh915data->boost_en, 0);
		while (gpio_get_value(zh915data->boost_en));
		printk("[VIB] %s : motor_boost_en Low \n",__func__);
	}
	return count;
}

static ssize_t zh915_power_cntrol_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct zh915_data *zh915data = dev_get_drvdata(dev);
	int value;

	value = gpio_get_value(zh915data->boost_en);
	printk("[VIB] %s : mot_boost_en gpio Value : 0x%x\n", __func__, value);

	return sprintf(buf, "mot_boost_en gpio Value :  0x%x\n",value);
}

static ssize_t get_register_data(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct zh915_data *zh915data = dev_get_drvdata(dev);
	int value;

	if ( zh915data->ic_reg_address == 0x01)
		zh915data->ic_reg_address = 0x0c;
	value = zh915_reg_read(zh915data, zh915data->ic_reg_address);
	printk("[VIB] Get Register : 0x%x Value : 0x%x\n", zh915data->ic_reg_address, value);

	return sprintf(buf, "Register :  0x%x Value :  0x%x\n", zh915data->ic_reg_address, value);
}

/* Force touch */
static ssize_t  store_duty(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct zh915_data *zh915data = dev_get_drvdata(dev);

	int ret;
	u16 duty;

	ret = kstrtou16(buf, 0, &duty);
	if (ret != 0) {
		dev_err(dev, "fail to get duty.\n");
		return count;
	}

	if (duty > 100)
		duty = 100;
	zh915data->duty = 127 * duty / 100 + 1;
	printk("[VIB] %s : duty = 0x%x , strength = 0x%x \n",__func__, duty, (u32)zh915data->duty);
	return count;
}

static ssize_t store_frequency(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct zh915_data *zh915data
		= dev_get_drvdata(dev);

	int ret;
	u16 freq;
	int temp;

	ret = kstrtou16(buf, 0, &freq);
	if (ret != 0) {
		dev_err(dev, "fail to get period.\n");
		return count;
	}

	if (freq <= 120)
		freq = 120;
	else if (freq >= 350)
		freq = 350;
	
	temp = (12500000/freq - 50000) / 512;
	zh915data->freq = temp;
	printk("[VIB] %s : req_freq = 0x%x / setting value = 0x%x\n",__func__, freq, (u32)zh915data->freq);

	return count;
}

static ssize_t show_duty_frequency(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct zh915_data *zh915data = dev_get_drvdata(dev);

	printk("[VIB] %s : duty = 0x%x, period = 0x%x \n",__func__, (u32)zh915data->duty, (u32)zh915data->freq);

	return snprintf(buf, VIB_BUFSIZE, "duty: 0x%x, period: 0x%x\n",
			(u32)zh915data->duty,
			(u32)zh915data->freq);
}

static DEVICE_ATTR(set_register, 0220, NULL, set_chip_register);
static DEVICE_ATTR(get_register, 0660, get_register_data, get_chip_register);
static DEVICE_ATTR(init_register, 0440, init_chip_register, NULL);
static DEVICE_ATTR(power_control, 0660, zh915_power_cntrol_show, zh915_power_control);
static DEVICE_ATTR(set_duty, 0220, NULL, store_duty);
static DEVICE_ATTR(set_frequency, 0220, NULL, store_frequency);
static DEVICE_ATTR(show_duty_frequency, 0440, show_duty_frequency, NULL);

static struct attribute *sec_motor_attributes[] = {
	&dev_attr_set_register.attr,
	&dev_attr_get_register.attr,
	&dev_attr_init_register.attr,
	&dev_attr_power_control.attr,
	&dev_attr_set_duty.attr,
	&dev_attr_set_frequency.attr,
	&dev_attr_show_duty_frequency.attr,
	NULL,
};

static struct attribute_group sec_motor_attr_group = {
	.attrs = sec_motor_attributes,
};

static ssize_t intensity_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct timed_output_dev *t_dev = dev_get_drvdata(dev);
        struct zh915_data *zh915data = container_of(t_dev, struct zh915_data, to_dev);
	int intensity = 0, ret = 0;


	ret = kstrtoint(buf, 0, &intensity);
	if (ret) {
		pr_err("[VIB] fail to get intensity\n");
		return -EINVAL;
	}

	if ((intensity < 0) || (MAX_INTENSITY < intensity)) {
		pr_err("[VIB] out of rage\n");
		return -EINVAL;
	}

	zh915data->intensity = intensity;
	printk("[VIB] %s : Set intensity : %d\n",__func__, zh915data->intensity);

	zh915_haptic_set_intensity(zh915data, zh915data->intensity);

	return count;
}

static ssize_t intensity_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *t_dev = dev_get_drvdata(dev);
        struct zh915_data *zh915data = container_of(t_dev, struct zh915_data, to_dev);

	return snprintf(buf, VIB_BUFSIZE, "[VIB] %s :  intensity: %d\n",__func__, zh915data->intensity);
}

static DEVICE_ATTR(intensity, 0660, intensity_show, intensity_store);


static ssize_t frequency_store(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count)
{
	struct timed_output_dev *t_dev = dev_get_drvdata(dev);
        struct zh915_data *zh915data = container_of(t_dev, struct zh915_data, to_dev);
	int freq_num = 0, ret = 0;
	
	ret = kstrtoint(buf, 0, &freq_num);
	if (ret) {
		pr_err("[VIB] fail to get frequency\n");
		return -EINVAL;
	}

	if (freq_num < 0 || freq_num >= zh915data->multi_frequency) {
		pr_err("out of rage\n");
		return -EINVAL;
	}
	
	zh915data->freq_num = freq_num;
	printk("[VIB] %s : freq_num: %d\n", __func__, zh915data->freq_num);

	zh915_haptic_set_frequency(zh915data, zh915data->freq_num);

	return count;
}

static ssize_t frequency_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct timed_output_dev *t_dev = dev_get_drvdata(dev);
        struct zh915_data *zh915data = container_of(t_dev, struct zh915_data, to_dev);
	return snprintf(buf, VIB_BUFSIZE, "[VIB] %s : freq_num: %d\n",__func__, zh915data->freq_num);
}

static DEVICE_ATTR(multi_freq, 0660, frequency_show, frequency_store);

#ifdef CONFIG_OF
static int zh915_parse_dt(struct device *dev,
		struct zh915_data *zh915data)
{
	struct device_node *np_root = dev->parent->of_node;
	struct device_node *np = dev->of_node;
	int ret;
	u32 temp;
	int i;

	zh915data->boost_en = of_get_named_gpio(np, "mot_boost_en", 0);
	pr_info("[VIB] %s : motor_mode zh915data->boost_en gpio = %d\n", __func__, zh915data->boost_en);

	np = of_find_node_by_name(np_root,
			"motor");
	
	if (np == NULL) {
		pr_err("%s : error to get dt node\n", __func__);
		goto err_parsing_dt;
	}

	ret = of_property_read_u32(np, "motor,motor_type", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s : error to get dt node motor_type \n", __func__);
		goto err_parsing_dt;
	} else if (temp != EXTERNAL_DRIVING_IC) {
			printk("[VIB] %s : IFPMIC use case : zh915 probe error return : motor type = %d\n",__func__, temp);
					goto err_parsing_dt;
	} else
		printk("[VIB] : %s : motor_type value %d \n", __func__, temp);

	np = of_find_node_by_name(np_root,
			"haptic");

	if (np == NULL) {
		pr_err("%s : error to get dt node\n", __func__);
		goto err_parsing_dt;
	}

	ret = of_property_read_u32(np, "haptic,multi_frequency", &temp);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s : error to get dt node multi_frequency\n", __func__);
		zh915data->multi_frequency = 0;
	} else
		zh915data->multi_frequency = (int)temp;

	if (zh915data->multi_frequency) {
		zh915data->multi_freq
			= devm_kzalloc(dev, sizeof(u32)*zh915data->multi_frequency, GFP_KERNEL);
		if (!zh915data->multi_freq) {
			pr_err("%s: failed to allocate freq data\n", __func__);
			goto err_parsing_dt;
		}

		zh915data->multi_duty
			= devm_kzalloc(dev, sizeof(u32)*zh915data->multi_frequency, GFP_KERNEL);
		if (!zh915data->multi_duty) {
			pr_err("%s: failed to allocate duty data \n", __func__);
			goto err_parsing_dt_freq;
		}

		ret = of_property_read_u32_array(np, "haptic,freq", zh915data->multi_freq,
				zh915data->multi_frequency);
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s : error to get dt freq data\n", __func__);
			goto err_parsing_dt_duty;
		}

		ret = of_property_read_u32_array(np, "haptic,duty", zh915data->multi_duty,
				zh915data->multi_frequency);
		if (IS_ERR_VALUE(ret)) {
			pr_err("%s : error to get dt duty data\n", __func__);
			goto err_parsing_dt_duty;
		}
		zh915data->duty = zh915data->multi_duty[0];
		zh915data->freq = zh915data->multi_freq[0];
		zh915data->freq_num = 0;
	}

	/* debugging */
	if (zh915data->multi_frequency) {
		pr_debug("multi frequency = %d\n", zh915data->multi_frequency);
		for (i = 0; i < zh915data->multi_frequency; i++) {
			pr_debug("freq[%d] = %d\n", i, zh915data->multi_freq[i]);
		}
	}

	return 0;

err_parsing_dt_duty:
	kfree(zh915data->multi_duty);
err_parsing_dt_freq:
	kfree(zh915data->multi_freq);
err_parsing_dt:
	return -ENODEV;
}
#endif

static int zh915_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
	struct zh915_data *zh915data;
	int err = 0, ret = 0;

	pr_info("[VIB] %s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_err(&client->dev, "%s:I2C check failed\n", __FUNCTION__);
		return -ENODEV;
	}

	zh915data = devm_kzalloc(&client->dev, sizeof(struct zh915_data), GFP_KERNEL);
	if (zh915data == NULL){
		dev_err(&client->dev, "%s:no memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	zh915data->dev = &client->dev;
	zh915data->i2c = client;
	zh915data->mpRegmap = devm_regmap_init_i2c(client, &zh915_i2c_regmap);
	if (IS_ERR(zh915data->mpRegmap)) {
		err = PTR_ERR(zh915data->mpRegmap);
		dev_err(zh915data->dev, 
			"%s:Failed to allocate register map: %d\n",__FUNCTION__,err);
		goto exit_probe;
	}

	ret = zh915_parse_dt(&client->dev, zh915data);
	if (ret) {
		pr_err("[%s] zh915 parse dt failed\n", __func__);
		err = -ENODEV;
		goto exit_probe;
	}

	if (gpio_is_valid(zh915data->boost_en)) {	
		ret = gpio_request(zh915data->boost_en, "zh915_power_en");
		if (ret)
		{
			dev_err(zh915data->dev, "[VIB]: %s : motor enable fail \n",__func__);
			goto exit_sec_devices;
		}	
		ret = gpio_direction_output(zh915data->boost_en, 1);

		if (ret) {
			dev_err(zh915data->dev, "[VIB]: Unable to set zh915_power_en: output direction, error %d\n", ret);			
			goto exit_sec_devices;
		}
		// zh915 ic need to startup time
		msleep(2);
	} else {
		dev_err(zh915data->dev, "[VIB]: %s: gpio is invalid (%d)\n", __func__, zh915data->boost_en);	
	}

	zh915data->f_packet_en = false;
	zh915data->packet_cnt = 0;
	zh915data->packet_size = 0;
	zh915data->force_touch_intensity = -1;

	ret = Haptics_init(zh915data);
	if (ret<0) {
		goto exit_sec_devices;
	}

	err = sysfs_create_file(&zh915data->to_dev.dev->kobj,
				&dev_attr_intensity.attr);
	if (err < 0) {
		pr_err("Failed to register sysfs : %d\n", err);
		goto err_timed_output_register;
	}

	err = sysfs_create_file(&zh915data->to_dev.dev->kobj,
				&dev_attr_multi_freq.attr);
	if (err < 0) {
		pr_err("Failed to register multi_freq sysfs : %d\n", err);
		goto err_timed_output_register;
	}
	err = sysfs_create_file(&zh915data->to_dev.dev->kobj,
				&dev_attr_haptic_engine.attr);
	if (err < 0) {
		pr_err("Failed to register haptic_engine sysfs : %d\n", err);
		goto err_timed_output_register;
	}

	err = sysfs_create_file(&zh915data->to_dev.dev->kobj,
				&dev_attr_force_touch_intensity.attr);
	if (err < 0) {
		pr_err("Failed to register force touch intensity sysfs : %d\n", err);
		goto err_timed_output_register;
	}

	/* create sysfs */
	zh915data->motor_dev = sec_device_create(zh915data, "motor");
	if (IS_ERR(zh915data->motor_dev)) {
		err = -ENODEV;
		pr_err("[VIB] Failed to create device\
				for samsung specific motor, err num: %d\n", err);
		goto err_timed_output_register;
	}
	err = sysfs_create_group(&zh915data->motor_dev->kobj, &sec_motor_attr_group);
	if (err) {
		err = -ENODEV;
		pr_err("[VIB] Failed to create sysfs group\
				for samsung specific motor, err num: %d\n", err);
		goto exit_sysfs;
	}

	i2c_set_clientdata(client,zh915data);

	zh915_reg_init(zh915data);

	g_drvdata= zh915data;
	dev_info(zh915data->dev, 
		"zh915 probe succeeded\n");
	gpio_direction_output(zh915data->boost_en, 0);

	return 0;
	
exit_sysfs:
	sec_device_destroy(zh915data->motor_dev->devt);
err_timed_output_register:
	sysfs_remove_group(&zh915data->motor_dev->kobj, &sec_motor_attr_group);
exit_sec_devices:	
	dev_err(zh915data->dev, 
		"%s failed, err=%d\n",
		__FUNCTION__, err);
	kfree(zh915data->multi_duty);
	kfree(zh915data->multi_freq);
exit_probe:
	return err;
}

static int zh915_remove(struct i2c_client* client)
{
	return 0;
}

extern int haptic_homekey_press(void)
{
	struct hrtimer *timer;
	struct zh915_data *zh915data;
	int loop_cnt = 5;

	if(g_drvdata == NULL)
		return -1;
	timer = &g_drvdata->timer;

	zh915data = g_drvdata;

	// mot_boost_on 
	gpio_set_value(zh915data->boost_en, 1);
	while (!gpio_get_value(zh915data->boost_en)  || !loop_cnt)
	{
		mdelay(1);
		loop_cnt--;
	}
	mdelay(2);
	zh915_reg_write(zh915data, REG_CONTROL, CONTROL_LOOP_MASK);  // LPA, LOOP OPEN
	
	mutex_lock(&zh915data->lock);
	zh915data->timeout = HOMEKEY_DURATION;
	zh915_haptic_set_frequency(zh915data, HOMEKEY_PRESS_FREQ);
	if (zh915data->force_touch_intensity == -1)
		zh915_haptic_set_intensity(zh915data, MAX_INTENSITY);
	else
		zh915_haptic_set_intensity(zh915data, zh915data->force_touch_intensity);
	// vibrator enable 	
	zh915_reg_write(zh915data, REG_RESONANCE_FREQ, zh915data->freq); // Frequency set
	zh915_reg_write(zh915data, REG_STRENGTH_WRITE, zh915data->duty); // strength set
	zh915_reg_write(zh915data, REG_MODE, I2C_MODE & MODE_MASK);       // motor enable
	
	pr_info("[VIB] %s homekey press freq:%d, intensity:%d, time:%d\n", __func__, HOMEKEY_PRESS_FREQ, zh915data->force_touch_intensity, zh915data->timeout);
	zh915data->running = true;
	mutex_unlock(&zh915data->lock);

	hrtimer_start(timer, ns_to_ktime((u64)zh915data->timeout * NSEC_PER_MSEC),
		HRTIMER_MODE_REL);

	return 0;
}

extern int haptic_homekey_release(void)
{
	struct hrtimer *timer;
	struct zh915_data *zh915data;
	int loop_cnt = 5;

	if(g_drvdata == NULL)
		return -1;
	zh915data = g_drvdata;
	timer = &zh915data->timer;

	// mot_boost_on
	if (!gpio_get_value(zh915data->boost_en)) {
		gpio_set_value(zh915data->boost_en, 1);
		while (!gpio_get_value(zh915data->boost_en)  || !loop_cnt)
		{
			mdelay(1);
			loop_cnt--;
		}
		mdelay(2);
	}
	zh915_reg_write(zh915data, REG_CONTROL, CONTROL_LOOP_MASK);  // LPA, LOOP OPEN

	mutex_lock(&zh915data->lock);
	zh915data->timeout = HOMEKEY_DURATION;
	zh915_haptic_set_frequency(zh915data, HOMEKEY_RELEASE_FREQ);
	if (zh915data->force_touch_intensity == -1)
		zh915_haptic_set_intensity(zh915data, MAX_INTENSITY);
	else
		zh915_haptic_set_intensity(zh915data, zh915data->force_touch_intensity);

	zh915_reg_write(zh915data, REG_RESONANCE_FREQ, zh915data->freq); // Frequency set
	zh915_reg_write(zh915data, REG_STRENGTH_WRITE, zh915data->duty); // strength set
	zh915_reg_write(zh915data, REG_MODE, I2C_MODE & MODE_MASK);       // motor enable

	pr_info("%s homekey release freq:%d, intensity:%d, time:%d\n", __func__, HOMEKEY_RELEASE_FREQ, zh915data->force_touch_intensity, zh915data->timeout);
	zh915data->running = true;
	mutex_unlock(&zh915data->lock);

	hrtimer_start(timer, ns_to_ktime((u64)zh915data->timeout * NSEC_PER_MSEC),
		HRTIMER_MODE_REL);

	return 0;
}

static struct i2c_device_id zh915_id_table[] =
{
    { HAPTICS_DEVICE_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, zh915_id_table);

#ifdef CONFIG_OF
static struct of_device_id zh915_dt_ids[] =
{
	{ .compatible = "zh915" },
	{ },
};
MODULE_DEVICE_TABLE(of, zh915_dt_ids);
#endif

static struct i2c_driver zh915_driver =
{
    .driver = {
        .name = HAPTICS_DEVICE_NAME,
	.owner = THIS_MODULE,
#ifdef CONFIG_OF
	.of_match_table = zh915_dt_ids,
#endif
    },
    .id_table = zh915_id_table,
    .probe = zh915_probe,
    .remove = zh915_remove,
};

static int __init zh915_init(void)
{
	pr_info("[VIB] %s\n", __func__);
	return i2c_add_driver(&zh915_driver);
}

static void __exit zh915_exit(void)
{
	i2c_del_driver(&zh915_driver);
}

module_init(zh915_init);
module_exit(zh915_exit);

MODULE_AUTHOR("jwon88.jung@samsung.com");
MODULE_DESCRIPTION("Driver for zh925");
