/*
 * drivers/cpufreq/cpufreq_ondemandplus.c
 * Copyright (C) 2013 Boy Petersen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * based upon:
 *
 *
 *	drivers/cpufreq/cpufreq_interactive.c
 *
 *	Copyright (C) 2010 Google, Inc.
 *
 *	This software is licensed under the terms of the GNU General Public
 *	License version 2, as published by the Free Software Foundation, and
 *	may be copied, distributed, and modified under those terms.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	Author: Mike Chan (mike@android.com)
 *
 *
 * and:
 *
 *	drivers/cpufreq/cpufreq_ondemand.c
 *
 *	Copyright (C)  2001 Russell King
 *		  (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *			    Jun Nakajima <jun.nakajima@intel.com>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *	published by the Free Software Foundation.
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/kernel_stat.h>
#include <asm/cputime.h>
#include <linux/module.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_ondemandplus.h>

static atomic_t active_count = ATOMIC_INIT(0);

struct cpufreq_ondemandplus_cpuinfo {
	struct timer_list cpu_timer;
	struct mutex timer_mutex;
	int timer_idlecancel;
	u64 time_in_idle;
	u64 idle_exit_time;
	u64 timer_run_time;
	int idling;
	u64 target_set_time;
	u64 target_set_time_in_idle;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	unsigned int target_freq;
	int governor_enabled;
	unsigned int cpu;
};

static DEFINE_PER_CPU(struct cpufreq_ondemandplus_cpuinfo, cpuinfo);

/* realtime thread handles frequency scaling */
static struct task_struct *speedchange_task;
static cpumask_t speedchange_cpumask;
static spinlock_t speedchange_cpumask_lock;

/*
 * Tunables start
 */

#define DEFAULT_TIMER_RATE (20 * USEC_PER_MSEC)
static unsigned long timer_rate;

#define DEFAULT_UP_THRESHOLD 80
static unsigned long up_threshold;

#define DEFAULT_DOWN_DIFFERENTIAL 62
static unsigned long down_differential;

#define DEFAULT_INTER_HIFREQ 1586000
static u64 inter_hifreq;

#define DEFAULT_INTER_LOFREQ 1144000
static u64 inter_lofreq;

#define DEFAULT_INTER_STAYCYCLES 2
static unsigned long inter_staycycles;

#define DEFAULT_STAYCYCLES_RESETFREQ 676000
static u64 staycycles_resetfreq;

#define DEFAULT_IO_IS_BUSY 0
static unsigned int io_is_busy;

static u64 screen_on_min_freq = 449000;

/*
 * Tunables end
 */

static void cpufreq_ondemandplus_timer(unsigned long data)
{
	unsigned int delta_idle;
	unsigned int delta_time;
	int cpu_load;
	unsigned int load_freq;
	int load_since_change;
	u64 time_in_idle;
	u64 idle_exit_time;
	struct cpufreq_ondemandplus_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	u64 now_idle;
	unsigned int new_freq = 0;
	unsigned int index;
	static unsigned int stay_counter;
	unsigned long flags;
	static unsigned int i = 0;
	static unsigned int last_cpu_freqs[5] = {0};
	static unsigned int avg_cpu_freq = 0;
	static unsigned int lo_avg_cpu_freq;
	static unsigned int hi_avg_cpu_freq;
	static unsigned int low_timer_rate = 0;

	smp_rmb();

	if (!pcpu->governor_enabled)
		goto exit;

	/*
	 * Once pcpu->timer_run_time is updated to >= pcpu->idle_exit_time,
	 * this lets idle exit know the current idle time sample has
	 * been processed, and idle exit can generate a new sample and
	 * re-arm the timer.  This prevents a concurrent idle
	 * exit on that CPU from writing a new set of info at the same time
	 * the timer function runs (the timer function can't use that info
	 * until more time passes).
	 */

	time_in_idle = pcpu->time_in_idle;
	idle_exit_time = pcpu->idle_exit_time;
	now_idle = get_cpu_idle_time(data, &pcpu->timer_run_time, io_is_busy);
	smp_wmb();

	/* If we raced with cancelling a timer, skip. */
	if (!idle_exit_time)
		goto exit;

	delta_idle = (unsigned int) (now_idle - time_in_idle);
	delta_time = (unsigned int) (pcpu->timer_run_time - idle_exit_time);

	/*
	 * If timer ran less than 1ms after short-term sample started, retry.
	 */
	if (delta_time < 1000)
		goto rearm;

	if (delta_idle > delta_time)
		cpu_load = 0;
	else
		cpu_load = 100 * (delta_time - delta_idle) / delta_time;

	delta_idle = (unsigned int) (now_idle - pcpu->target_set_time_in_idle);
	delta_time = (unsigned int) (pcpu->timer_run_time - pcpu->target_set_time);

	if ((delta_time == 0) || (delta_idle > delta_time))
		load_since_change = 0;
	else
		load_since_change =
			100 * (delta_time - delta_idle) / delta_time;

	/*
	 * If short-term load (since last idle timer started or
	 * timer function re-armed itself) is higher than long-term
	 * load (since last frequency change), use short-term load
	 * to be able to scale up quickly.
	 * When long-term load is higher than short-term load,
	 * use the average of short-term load and long-term load
	 * (instead of just long-term load) to be able to scale
	 * down faster, with the long-term load being able to delay
	 * down scaling a little to maintain responsiveness.
	 */
	if (load_since_change > cpu_load) {
		cpu_load = (cpu_load + load_since_change) / 2;
	}

	load_freq = cpu_load * pcpu->target_freq;

	new_freq = pcpu->target_freq;

	/* Check for frequency increase */
	if (load_freq > up_threshold * pcpu->target_freq) {
		/* if we are already at full speed then break out early */
		if (pcpu->target_freq < pcpu->policy->max) {

			if (stay_counter == 0 && inter_staycycles != 0) {
				new_freq = inter_lofreq;
				stay_counter++;
			} else if (stay_counter == 1 && inter_staycycles != 1) {
				new_freq = inter_hifreq;
				stay_counter++;
			} else if (stay_counter < inter_staycycles) {
				stay_counter++;
				goto rearm;
			} else {
				new_freq = pcpu->policy->max;
			}
		}
	/* Check for frequency decrease */

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus 10 points under the threshold.
	 */
	} else if (load_freq < (up_threshold - down_differential) *
			pcpu->target_freq) {

		if (pcpu->target_freq != screen_on_min_freq) {
			new_freq = load_freq /
					(up_threshold - down_differential);

			if (new_freq <= staycycles_resetfreq) {
				stay_counter = 0;
			}

			if (new_freq < screen_on_min_freq) {
				new_freq = screen_on_min_freq;
			}
		}
	} else if (pcpu->target_freq == pcpu->policy->max &&
			load_freq < (up_threshold - down_differential / 2) *
			pcpu->target_freq) {
		new_freq = load_freq / (up_threshold - down_differential * 2 / 3);
	}

	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
					   new_freq, CPUFREQ_RELATION_H,
					   &index)) {
		pr_warn_once("timer %d: cpufreq_frequency_table_target error\n",
			     (int) data);
		goto rearm;
	}

	new_freq = pcpu->freq_table[index].frequency;

	if (pcpu->target_freq == new_freq) {
		trace_cpufreq_ondemandplus_already(data, cpu_load,
						  pcpu->target_freq, new_freq);
		goto rearm_if_notmax;
	}

	trace_cpufreq_ondemandplus_target(data, cpu_load, pcpu->target_freq,
					 new_freq);
	pcpu->target_set_time_in_idle = now_idle;
	pcpu->target_set_time = pcpu->timer_run_time;

	pcpu->target_freq = new_freq;
	spin_lock_irqsave(&speedchange_cpumask_lock, flags);
	cpumask_set_cpu(data, &speedchange_cpumask);
	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);
	wake_up_process(speedchange_task);

rearm_if_notmax:
	/*
	 * Already set max speed and don't see a need to change that,
	 * wait until next idle to re-evaluate, don't need timer.
	 */
	if (pcpu->target_freq == pcpu->policy->max)
		goto exit;

rearm:
	if (!timer_pending(&pcpu->cpu_timer)) {
		/*
		 * If already at min: if that CPU is idle, don't set timer.
		 * Else cancel the timer if that CPU goes idle.  We don't
		 * need to re-evaluate speed until the next idle exit.
		 */
		if (pcpu->target_freq == screen_on_min_freq) {
			smp_rmb();

			if (pcpu->idling)
				goto exit;

			pcpu->timer_idlecancel = 1;
		}

		/*
		 * Calculate the average CPU frequency of the last 5 timer
		 * cycles. Then check if the new to-be-requested frequency
		 * is within a divergent range of 15% for lower frequencies,
		 * or is equal to the average for higher frequencies.
		 * If yes, slow down the timer.
		 */
		if (i >= 4 && !pcpu->idling) {
			unsigned int k;
			avg_cpu_freq = 0;
			for (k = 0; k <= 4; k++) {
				avg_cpu_freq += last_cpu_freqs[k];
			}
			avg_cpu_freq /= 5;
			hi_avg_cpu_freq = (avg_cpu_freq * 115) / 100;
			lo_avg_cpu_freq = (avg_cpu_freq * 100) / 115;

			if ((new_freq < inter_lofreq && hi_avg_cpu_freq > new_freq &&
						lo_avg_cpu_freq < new_freq) || (new_freq >=
						inter_lofreq && avg_cpu_freq == new_freq)) {
				low_timer_rate = timer_rate * 2;
			} else {
				low_timer_rate = 0;
			}
			i = 0;
		} else if ((new_freq < inter_lofreq && (hi_avg_cpu_freq < new_freq ||
					lo_avg_cpu_freq > new_freq)) || (new_freq >=
					inter_lofreq && avg_cpu_freq != new_freq)) {
			low_timer_rate = 0;
		}

		/*
		 * Re-arm timer
		 */
		pcpu->time_in_idle = get_cpu_idle_time(
			data, &pcpu->idle_exit_time, io_is_busy);
		if (!low_timer_rate) {
			mod_timer(&pcpu->cpu_timer,
				jiffies + usecs_to_jiffies(timer_rate));
		} else {
			mod_timer(&pcpu->cpu_timer,
				jiffies + usecs_to_jiffies(low_timer_rate));
		}
	}

exit:
	/*
	 * Write CPU frequency of new timer cycle into the correct
	 * last_cpu_freqs array-field
	 */
	if (pcpu->idling) {
		memset(last_cpu_freqs, 0, sizeof(last_cpu_freqs));
		i = 0;
	}
	return;
}

static void cpufreq_ondemandplus_idle_start(void)
{
	struct cpufreq_ondemandplus_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	int pending;

	if (!pcpu->governor_enabled)
		return;

	pcpu->idling = 1;
	smp_wmb();
	pending = timer_pending(&pcpu->cpu_timer);

	if (pcpu->target_freq != pcpu->policy->min) {
#ifdef CONFIG_SMP
		/*
		 * Entering idle while not at lowest speed.  On some
		 * platforms this can hold the other CPU(s) at that speed
		 * even though the CPU is idle. Set a timer to re-evaluate
		 * speed so this idle CPU doesn't hold the other CPUs above
		 * min indefinitely.  This should probably be a quirk of
		 * the CPUFreq driver.
		 */
		if (!pending) {
			pcpu->time_in_idle = get_cpu_idle_time(
				smp_processor_id(), &pcpu->idle_exit_time, io_is_busy);
			pcpu->timer_idlecancel = 0;
			mod_timer(&pcpu->cpu_timer,
				  jiffies + usecs_to_jiffies(timer_rate));
		}
#endif
	} else {
		/*
		 * If at min speed and entering idle after load has
		 * already been evaluated, and a timer has been set just in
		 * case the CPU suddenly goes busy, cancel that timer.  The
		 * CPU didn't go busy; we'll recheck things upon idle exit.
		 */
		if (pending && pcpu->timer_idlecancel) {
			del_timer(&pcpu->cpu_timer);
			/*
			 * Ensure last timer run time is after current idle
			 * sample start time, so next idle exit will always
			 * start a new idle sampling period.
			 */
			pcpu->idle_exit_time = 0;
			pcpu->timer_idlecancel = 0;
		}
	}

}

static void cpufreq_ondemandplus_idle_end(void)
{
	struct cpufreq_ondemandplus_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

	pcpu->idling = 0;
	smp_wmb();

	/*
	 * Arm the timer for 1-2 ticks later if not already, and if the timer
	 * function has already processed the previous load sampling
	 * interval.  (If the timer is not pending but has not processed
	 * the previous interval, it is probably racing with us on another
	 * CPU.  Let it compute load based on the previous sample and then
	 * re-arm the timer for another interval when it's done, rather
	 * than updating the interval start time to be "now", which doesn't
	 * give the timer function enough time to make a decision on this
	 * run.)
	 */
	if (timer_pending(&pcpu->cpu_timer) == 0 &&
	    pcpu->timer_run_time >= pcpu->idle_exit_time &&
	    pcpu->governor_enabled) {
		pcpu->time_in_idle =
			get_cpu_idle_time(smp_processor_id(),
					     &pcpu->idle_exit_time, io_is_busy);
		pcpu->timer_idlecancel = 0;
		mod_timer(&pcpu->cpu_timer,
			  jiffies + usecs_to_jiffies(timer_rate));
	}

}

static int cpufreq_ondemandplus_speedchange_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_ondemandplus_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&speedchange_cpumask_lock, flags);

		if (cpumask_empty(&speedchange_cpumask)) {
			spin_unlock_irqrestore(&speedchange_cpumask_lock,
						flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = speedchange_cpumask;
		cpumask_clear(&speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			unsigned int j;
			unsigned int max_freq = 0;

			pcpu = &per_cpu(cpuinfo, cpu);
			smp_rmb();

			if (!pcpu->governor_enabled)
				continue;

			for_each_cpu(j, pcpu->policy->cpus) {
				struct cpufreq_ondemandplus_cpuinfo *pjcpu =
					&per_cpu(cpuinfo, j);

				if (pjcpu->target_freq > max_freq)
					max_freq = pjcpu->target_freq;
			}

			if (max_freq != pcpu->policy->cur)
				__cpufreq_driver_target(pcpu->policy,
							max_freq,
							CPUFREQ_RELATION_H);
			trace_cpufreq_ondemandplus_setspeed(cpu,
						pcpu->target_freq,
						pcpu->policy->cur);
		}
	}

	return 0;
}

static ssize_t show_timer_rate(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", timer_rate);
}

static ssize_t store_timer_rate(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	timer_rate = val;
	return count;
}

static struct kobj_attribute timer_rate_attr = __ATTR(timer_rate, 0644,
		show_timer_rate, store_timer_rate);

static ssize_t show_up_threshold(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", up_threshold);
}

static ssize_t store_up_threshold(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > 100)
		val = 100;

	if (val < 1)
		val = 1;

	up_threshold = val;
	return count;
}

static struct kobj_attribute up_threshold_attr = __ATTR(up_threshold, 0644,
		show_up_threshold, store_up_threshold);

static ssize_t show_down_differential(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", down_differential);
}

static ssize_t store_down_differential(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > 100)
		val = 100;

	down_differential = val;
	return count;
}

static struct kobj_attribute down_differential_attr = __ATTR(down_differential, 0644,
		show_down_differential, store_down_differential);

static ssize_t show_inter_hifreq(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%llu\n", inter_hifreq);
}

static ssize_t store_inter_hifreq(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf,
				  size_t count)
{
	int ret;
	u64 val;
	struct cpufreq_ondemandplus_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	unsigned int index;

	ret = kstrtoull(buf, 0, &val);
	if (ret < 0)
		return ret;

	index = 0;
	cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table, val,
		CPUFREQ_RELATION_L, &index);
	val = pcpu->freq_table[index].frequency;

	if (val > pcpu->policy->max)
		val = pcpu->policy->max;

	if (val < screen_on_min_freq)
		val = screen_on_min_freq;

	inter_hifreq = val;
	return count;
}

static struct kobj_attribute inter_hifreq_attr = __ATTR(inter_hifreq, 0644,
		show_inter_hifreq, store_inter_hifreq);

static ssize_t show_inter_lofreq(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%llu\n", inter_lofreq);
}

static ssize_t store_inter_lofreq(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf,
				  size_t count)
{
	int ret;
	u64 val;
	struct cpufreq_ondemandplus_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	unsigned int index;

	ret = kstrtoull(buf, 0, &val);
	if (ret < 0)
		return ret;

	index = 0;
	cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table, val,
			CPUFREQ_RELATION_H, &index);
	val = pcpu->freq_table[index].frequency;

	if (val > pcpu->policy->max)
		val = pcpu->policy->max;

	if (val < screen_on_min_freq)
		val = screen_on_min_freq;

	inter_lofreq = val;
	return count;
}

static struct kobj_attribute inter_lofreq_attr = __ATTR(inter_lofreq, 0644,
		show_inter_lofreq, store_inter_lofreq);

static ssize_t show_inter_staycycles(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", inter_staycycles);
}

static ssize_t store_inter_staycycles(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > 10)
		val = 10;

	inter_staycycles = val;
	return count;
}

static struct kobj_attribute inter_staycycles_attr = __ATTR(inter_staycycles, 0644,
		show_inter_staycycles, store_inter_staycycles);

static ssize_t show_staycycles_resetfreq(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%llu\n", staycycles_resetfreq);
}

static ssize_t store_staycycles_resetfreq(struct kobject *kobj,
				  struct kobj_attribute *attr, const char *buf,
				  size_t count)
{
	int ret;
	u64 val;
	struct cpufreq_ondemandplus_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

	ret = kstrtoull(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val > pcpu->policy->max)
		val = pcpu->policy->max;

	if (val < screen_on_min_freq)
		val = screen_on_min_freq;

	staycycles_resetfreq = val;
	return count;
}

static struct kobj_attribute staycycles_resetfreq_attr = __ATTR(staycycles_resetfreq, 0644,
		show_staycycles_resetfreq, store_staycycles_resetfreq);

static ssize_t show_io_is_busy(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", io_is_busy);
}

static ssize_t store_io_is_busy(struct kobject *kobj,
			struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	io_is_busy = val;
	return count;
}

static struct kobj_attribute io_is_busy_attr = __ATTR(io_is_busy, 0644,
		show_io_is_busy, store_io_is_busy);

static struct attribute *ondemandplus_attributes[] = {
	&timer_rate_attr.attr,
	&up_threshold_attr.attr,
	&down_differential_attr.attr,
	&inter_hifreq_attr.attr,
	&inter_lofreq_attr.attr,
	&inter_staycycles_attr.attr,
	&staycycles_resetfreq_attr.attr,
	&io_is_busy_attr.attr,
	NULL,
};

static struct attribute_group ondemandplus_attr_group = {
	.attrs = ondemandplus_attributes,
	.name = "ondemandplus",
};

static int cpufreq_ondemandplus_idle_notifier(struct notifier_block *nb,
					     unsigned long val,
					     void *data)
{
	switch (val) {
	case IDLE_START:
		cpufreq_ondemandplus_idle_start();
		break;
	case IDLE_END:
		cpufreq_ondemandplus_idle_end();
		break;
	}

	return 0;
}

static struct notifier_block cpufreq_ondemandplus_idle_nb = {
	.notifier_call = cpufreq_ondemandplus_idle_notifier,
};

static int cpufreq_governor_ondemandplus(struct cpufreq_policy *policy,
		unsigned int event)
{
	int rc;
	unsigned int cpu;
	unsigned int j;
	struct cpufreq_ondemandplus_cpuinfo *pcpu;
	struct cpufreq_frequency_table *freq_table;

	pcpu = &per_cpu(cpuinfo, policy->cpu);
	cpu = pcpu->cpu;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(cpu) || (!policy->cur))
			return -EINVAL;

		freq_table =
			cpufreq_frequency_get_table(cpu);

		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			pcpu->target_set_time_in_idle =
				get_cpu_idle_time(j,
					     &pcpu->target_set_time, io_is_busy);
			/* update min freq static value */
			if (policy->cpu == 0)
				screen_on_min_freq = policy->min;
			pcpu->governor_enabled = 1;
			smp_wmb();
		}

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (atomic_inc_return(&active_count) > 1)
			return 0;

		rc = sysfs_create_group(cpufreq_global_kobject,
				&ondemandplus_attr_group);
		if (rc)
			return rc;

		mutex_init(&pcpu->timer_mutex);

		idle_notifier_register(&cpufreq_ondemandplus_idle_nb);
		break;

	case CPUFREQ_GOV_STOP:
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->governor_enabled = 0;
			smp_wmb();
			del_timer_sync(&pcpu->cpu_timer);

			/*
			 * Reset idle exit time since we may cancel the timer
			 * before it can run after the last idle exit time,
			 * to avoid tripping the check in idle exit for a timer
			 * that is trying to run.
			 */
			pcpu->idle_exit_time = 0;
		}

		if (atomic_dec_return(&active_count) > 0)
			return 0;

		mutex_destroy(&pcpu->timer_mutex);

		idle_notifier_unregister(&cpufreq_ondemandplus_idle_nb);
		sysfs_remove_group(cpufreq_global_kobject,
				&ondemandplus_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		/* If device is being removed, skip set limits */
		if (!pcpu->policy->cur)
			break;
		mutex_lock(&pcpu->timer_mutex);
		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
		/* update min freq static value */
		if (cpu == 0)
			screen_on_min_freq = policy->min;
		mutex_unlock(&pcpu->timer_mutex);
		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMANDPLUS
static
#endif
struct cpufreq_governor cpufreq_gov_ondemandplus = {
	.name = "ondemandplus",
	.governor = cpufreq_governor_ondemandplus,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

static int __init cpufreq_ondemandplus_init(void)
{
	unsigned int i;
	struct cpufreq_ondemandplus_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };

	timer_rate = DEFAULT_TIMER_RATE;
	up_threshold = DEFAULT_UP_THRESHOLD;
	down_differential = DEFAULT_DOWN_DIFFERENTIAL;
	inter_hifreq = DEFAULT_INTER_HIFREQ;
	inter_lofreq = DEFAULT_INTER_LOFREQ;
	inter_staycycles = DEFAULT_INTER_STAYCYCLES;
	staycycles_resetfreq = DEFAULT_STAYCYCLES_RESETFREQ;
	io_is_busy = DEFAULT_IO_IS_BUSY;

	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);
		init_timer(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_ondemandplus_timer;
		pcpu->cpu_timer.data = i;
		mutex_init(&pcpu->timer_mutex);
	}

	spin_lock_init(&speedchange_cpumask_lock);
	speedchange_task =
		kthread_create(cpufreq_ondemandplus_speedchange_task, NULL,
				"cfondemandplus");
	if (IS_ERR(speedchange_task))
		return PTR_ERR(speedchange_task);

	sched_setscheduler_nocheck(speedchange_task, SCHED_FIFO, &param);
	get_task_struct(speedchange_task);

	/* NB: wake up so the thread does not look hung to the freezer */
	wake_up_process(speedchange_task);

	return cpufreq_register_governor(&cpufreq_gov_ondemandplus);

	put_task_struct(speedchange_task);
	return -ENOMEM;
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMANDPLUS
fs_initcall(cpufreq_ondemandplus_init);
#else
module_init(cpufreq_ondemandplus_init);
#endif

static void __exit cpufreq_ondemandplus_exit(void)
{
	unsigned int i;

	cpufreq_unregister_governor(&cpufreq_gov_ondemandplus);
	for_each_possible_cpu(i) {
		struct cpufreq_ondemandplus_cpuinfo *pcpu =
			&per_cpu(cpuinfo, i);
		mutex_destroy(&pcpu->timer_mutex);
	}
	kthread_stop(speedchange_task);
	put_task_struct(speedchange_task);
}

module_exit(cpufreq_ondemandplus_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_DESCRIPTION("'cpufreq_ondemandplus' - A cpufreq governor for "
        "semi-aggressive scaling");
MODULE_LICENSE("GPL");
