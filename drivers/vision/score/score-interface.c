/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/random.h>
#include <linux/slab.h>

#include "score-config.h"
#include "score-interface.h"
#include "score-regs.h"
#include "score-regs-user-def.h"
#include "../score-vertexmgr.h"
#include "../score-device.h"
#include "score-debug.h"
#include "score-lock.h"

#define TURN_AROUND_TIME (HZ/20)

#define init_request_barrier(itf) mutex_init(&itf->request_barrier)
#define enter_request_barrier(itf) mutex_lock(&itf->request_barrier);
#define exit_request_barrier(itf) mutex_unlock(&itf->request_barrier);
#define init_process_barrier(itf) spin_lock_init(&itf->process_barrier);
#define enter_process_barrier(itf) spin_lock_irq(&itf->process_barrier);
#define exit_process_barrier(itf) spin_unlock_irq(&itf->process_barrier);

/* #define USE_EMUL_TIMER */

static int __score_interface_cleanup(struct score_interface *interface);
static irqreturn_t interface_isr(int irq, void *data);

#ifdef USE_EMUL_TIMER
/* emul timer */
static void interface_timer(unsigned long data);
#endif


int score_interface_probe(struct score_interface *interface,
		struct device *dev,
		void __iomem *regs,
		resource_size_t regs_size,
		u32 irq0)
{
	int ret = 0;
	struct score_device *device;
	struct score_system *system;
	struct score_framemgr *framemgr;

	SCORE_TP();
	BUG_ON(!interface);
	BUG_ON(!dev);
	BUG_ON(!regs);

	system = container_of(interface, struct score_system, interface);
	device = container_of(system, struct score_device, system);

	init_request_barrier(interface);
	init_process_barrier(interface);
	init_waitqueue_head(&interface->init_wait_queue);
	init_waitqueue_head(&interface->reply_wait_queue);

	interface->regs = regs;
	interface->regs_size = regs_size;
	interface->cookie = (void *)&device->vertexmgr;
	interface->request_msg = NULL;
	interface->process_msg = NULL;
	interface->task_itf = NULL;
	clear_bit(SCORE_ITF_STATE_OPEN, &interface->state);
	clear_bit(SCORE_ITF_STATE_START, &interface->state);
	clear_bit(SCORE_ITF_STATE_REPLY, &interface->state);
	interface->private_data = NULL;

	ret = devm_request_irq(dev, irq0, interface_isr, 0, dev_name(dev), interface);
	if (ret) {
		probe_err("devm_request_irq(0) is fail(%d)\n", ret);
		goto p_err;
	}

	framemgr = &interface->framemgr;
	framemgr->id = SCORE_MAX_VERTEX;
	framemgr->sindex = 0;
	spin_lock_init(&framemgr->slock);

	ret = score_frame_init(framemgr, interface);
	if (ret) {
		probe_err("score_frame_init is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_interface_open(struct score_interface *interface)
{
	int ret = 0;
	char name[30];
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	SCORE_TP();
	BUG_ON(!interface);

	init_kthread_worker(&interface->worker);
	snprintf(name, sizeof(name), "score_interface");
	interface->task_itf = kthread_run(kthread_worker_fn, &interface->worker, name);
	if (IS_ERR_OR_NULL(interface->task_itf)) {
		score_err("kthread_run is fail\n");
		ret = -EINVAL;
		goto p_err;
	}

	ret = sched_setscheduler_nocheck(interface->task_itf, SCHED_FIFO, &param);
	if (ret) {
		score_err("sched_setscheduler_nocheck is fail(%d)\n", ret);
		goto p_err;
	}

	ret = score_vertexmgr_itf_register(interface->cookie, interface);
	if (ret) {
		score_err("score_vertex_itf_register is fail(%d)\n", ret);
		goto p_err;
	}

	interface->request_msg = NULL;
	interface->process_msg = NULL;
	interface->done_cnt = 0;
	set_bit(SCORE_ITF_STATE_OPEN, &interface->state);
	clear_bit(SCORE_ITF_STATE_START, &interface->state);
	clear_bit(SCORE_ITF_STATE_REPLY, &interface->state);

p_err:
	return ret;
}

int score_interface_close(struct score_interface *interface)
{
	int ret = 0;

	BUG_ON(!interface);

	ret = __score_interface_cleanup(interface);
	if (ret)
		score_err("__score_interface_cleanup is fail(%d)\n", ret);

	ret = score_vertexmgr_itf_unregister(interface->cookie, interface);
	if (ret)
		score_err("score_vertex_itf_unregister is fail(%d)\n", ret);

	kthread_stop(interface->task_itf);

	clear_bit(SCORE_ITF_STATE_OPEN, &interface->state);

	return ret;
}

int score_interface_start(struct score_interface *interface)
{
	int ret = 0;

	set_bit(SCORE_ITF_STATE_START, &interface->state);

#ifdef USE_EMUL_TIMER
	init_timer(&interface->timer);
	interface->timer.expires = jiffies + TURN_AROUND_TIME;
	interface->timer.data = (unsigned long)interface;
	interface->timer.function = interface_timer;
	add_timer(&interface->timer);
#endif

	return ret;
}

int score_interface_stop(struct score_interface *interface)
{
	int errcnt = 0;
	struct score_framemgr *iframemgr;
	u32 retry;

	iframemgr = &interface->framemgr;

	retry = 10;
	while (--retry && iframemgr->frame_req_cnt) {
		score_warn("waiting %d request completion...(%d)\n", iframemgr->frame_req_cnt, retry);
		msleep(30);
	}

	if (!retry) {
		score_err("request completion is fail\n");
		errcnt++;
	}

	retry = 10;
	while (--retry && iframemgr->frame_pro_cnt) {
		score_warn("waiting %d process completion...(%d)\n", iframemgr->frame_pro_cnt, retry);
		msleep(30);
	}

	if (!retry) {
		score_err("process completion is fail\n");
		errcnt++;
	}

	retry = 10;
	while (--retry && iframemgr->frame_com_cnt) {
		score_warn("waiting %d complete completion...(%d)\n", iframemgr->frame_com_cnt, retry);
		msleep(30);
	}

	if (!retry) {
		score_err("complete completion is fail\n");
		errcnt++;
	}

#ifdef USE_EMUL_TIMER
	del_timer(&interface->timer);
	clear_bit(SCORE_ITF_STATE_START, &interface->state);
#endif

	return errcnt;
}

static int __score_interface_cleanup(struct score_interface *interface)
{
	int ret = 0;
	struct score_framemgr *framemgr;

	framemgr = &interface->framemgr;

	if (framemgr->frame_req_cnt) {
		score_err("[ITF] request count is NOT zero(%d)\n", framemgr->frame_req_cnt);
		ret = -EINVAL;
		goto p_err;
	}

	if (framemgr->frame_pro_cnt) {
		score_err("[ITF] process count is NOT zero(%d)\n", framemgr->frame_pro_cnt);
		ret = -EINVAL;
		goto p_err;
	}

	if (framemgr->frame_com_cnt) {
		score_err("[ITF] complete count is NOT zero(%d)\n", framemgr->frame_com_cnt);
		ret = -EINVAL;
		goto p_err;
	}

p_err:
	return ret;
}

static void score_int_code_clear_bits(struct score_interface *interface, unsigned int bits)
{
	unsigned int temp;

	SCORE_BAKERY_LOCK(interface->regs, CPU_ID);
	temp = readl(interface->regs + SCORE_INT_CODE);
	temp = (temp & (~bits)) & 0xFFFFFFFF;
	writel(temp, interface->regs + SCORE_INT_CODE);
	SCORE_BAKERY_UNLOCK(interface->regs, CPU_ID);
}

static irqreturn_t interface_isr(int irq, void *data)
{
	unsigned long flag;
	struct score_interface *interface = (struct score_interface *)data;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;

	unsigned int irq_code, irq_code_req;

	SCORE_BAKERY_LOCK(interface->regs, CPU_ID);
	irq_code = readl(interface->regs + SCORE_INT_CODE);
	SCORE_BAKERY_UNLOCK(interface->regs, CPU_ID);

	if (irq_code & SCORE_INT_ABNORMAL_MASK) {
		// HW exception case
		irq_code_req = irq_code & SCORE_INT_HW_EXCEPTION_MASK;
		if (irq_code_req) {
			switch (irq_code_req) {
				case SCORE_INT_BUS_ERROR:
					score_err("Bus error occurred, SCORE_INT_CODE : 0x%08X\n", irq_code);
					score_int_code_clear_bits(interface, SCORE_INT_BUS_ERROR);
					break;

				case SCORE_INT_DMA_CONFLICT:
					score_err("Dma conflict occurred, SCORE_INT_CODE : 0x%08X\n", irq_code);
					score_int_code_clear_bits(interface, SCORE_INT_DMA_CONFLICT);
					break;

				case SCORE_INT_CACHE_UNALIGNED:
					score_err("Cache unaligned occurred, SCORE_INT_CODE : 0x%08X\n", irq_code);
					score_int_code_clear_bits(interface, SCORE_INT_CACHE_UNALIGNED);
					break;

				default:
					score_err("Multiple hw exception occurred, SCORE_INT_CODE : 0x%08X\n", irq_code);
					score_int_code_clear_bits(interface, SCORE_INT_HW_EXCEPTION_MASK);
					break;
			}
		}

		// Related to SCore dump and Profiler
		irq_code_req = irq_code & SCORE_INT_DUMP_PROFILER_MASK;
		if (irq_code_req) {
			switch (irq_code_req) {
				case SCORE_INT_CORE_DUMP_HAPPEN:
					score_err("SCore dump happened, SCORE_INT_CODE : 0x%08X\n", irq_code);
					//TODO: need next step
					score_int_code_clear_bits(interface, SCORE_INT_CORE_DUMP_HAPPEN);
					break;

				case SCORE_INT_CORE_DUMP_DONE:
					score_err("SCore dump done occurred, SCORE_INT_CODE : 0x%08X\n", irq_code);
					//TODO: need next step
					score_int_code_clear_bits(interface, SCORE_INT_CORE_DUMP_DONE);
					break;

				case SCORE_INT_PROFILER_START:
					score_err("SCore profiler started, SCORE_INT_CODE : 0x%08X\n", irq_code);
					//TODO: need next step
					score_int_code_clear_bits(interface, SCORE_INT_PROFILER_START);
					break;

				case SCORE_INT_PROFILER_END:
					score_err("SCore profiler ended, SCORE_INT_CODE : 0x%08X\n", irq_code);
					//TODO: need next step
					score_int_code_clear_bits(interface, SCORE_INT_PROFILER_END);
					break;

				default:
					score_err("Multiple SCore dump interrupt occurred, SCORE_INT_CODE : 0x%08X\n", irq_code);
					//TODO: need next step
					score_int_code_clear_bits(interface, SCORE_INT_DUMP_PROFILER_MASK);
					break;
			}
		}

		// SCore sw assertion case
		irq_code_req = irq_code & SCORE_INT_SW_ASSERT;
		if (irq_code_req) {
			score_err("SCore sw assertion occurred, SCORE_INT_CODE : 0x%08X\n", irq_code);
			//TODO: need next step
			score_int_code_clear_bits(interface, SCORE_INT_SW_ASSERT);
		}
	} else {
#ifdef DBG_ISR
		score_event_msg("SCORE_SCORE2HOST %X\n", readl(interface->regs + SCORE_SCORE2HOST));
		score_event_msg("SCORE_DSP_INT %X\n", readl(interface->regs + SCORE_DSP_INT));
		score_event_msg("I_H/T(0x%x 0x%x) O_H/T(0x%x 0x%x) \n",
				readl(interface->regs + SCORE_INPUT_QUEUE_HEAD),
				readl(interface->regs + SCORE_INPUT_QUEUE_TAIL),
				readl(interface->regs + SCORE_OUTPUT_QUEUE_HEAD),
				readl(interface->regs + SCORE_OUTPUT_QUEUE_TAIL));
#endif
//		writel(0x0, interface->regs + SCORE_SCORE2HOST);

		iframemgr = &interface->framemgr;
		interface->done_cnt++;

	/* if (!test_bit(SCORE_ITF_STATE_START, &interface->state))
	*	return -1;
	*/

		score_frame_process_head(iframemgr, &iframe);
		if (!iframe) {
			score_err("iframe is NULL \n");

			goto p_next;
		}

		if (iframe->message == SCORE_FRAME_PROCESS) {
			framemgr_e_barrier_irqs(iframemgr, 0, flag);
			score_frame_trans_pro_to_com(iframemgr, iframe);
			framemgr_x_barrier_irqr(iframemgr, 0, flag);

			iframe->message = SCORE_FRAME_DONE;
			/* score_vertexmgr_queue(interface->cookie, iframe); */
		} else {
			framemgr_e_barrier_irqs(iframemgr, 0, flag);
			score_frame_trans_pro_to_com(iframemgr, iframe);
			framemgr_x_barrier_irqr(iframemgr, 0, flag);
		}

		queue_kthread_work(&interface->worker, &iframe->work);
	}
p_next:
	writel(0x0, interface->regs + SCORE_SCORE2HOST);

	/*
	* get_random_bytes(&random, sizeof(random));
	* random = random % TURN_AROUND_TIME;
	* mod_timer(&interface->timer, jiffies + random);
	*/

	return IRQ_HANDLED;
}

#ifdef USE_EMUL_TIMER
static void interface_timer(unsigned long data)
{
	unsigned long flag;
	struct score_interface *interface = (struct score_interface *)data;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;
	u32 random;

	iframemgr = &interface->framemgr;
	interface->done_cnt++;

	if (!test_bit(SCORE_ITF_STATE_START, &interface->state))
		return;

	score_frame_process_head(iframemgr, &iframe);
	if (!iframe)
		goto p_next;

	if (iframe->message == SCORE_FRAME_PROCESS) {
		framemgr_e_barrier_irqs(iframemgr, 0, flag);
		score_frame_trans_pro_to_com(iframemgr, iframe);
		framemgr_x_barrier_irqr(iframemgr, 0, flag);

		iframe->message = SCORE_FRAME_DONE;
		score_vertexmgr_queue(interface->cookie, iframe);
	} else {
		framemgr_e_barrier_irqs(iframemgr, 0, flag);
		score_frame_trans_pro_to_com(iframemgr, iframe);
		framemgr_x_barrier_irqr(iframemgr, 0, flag);

		/* __set_reply(interface, iframe->message); */
		/* wake_up(&interface->reply_wait_queue); */
	}

p_next:
	get_random_bytes(&random, sizeof(random));
	random = random % TURN_AROUND_TIME;
	mod_timer(&interface->timer, jiffies + random);
}
#endif
