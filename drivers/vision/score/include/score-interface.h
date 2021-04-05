/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef SCORE_INTERFACE_H_
#define SCORE_INTERFACE_H_

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/timer.h>

#include "score-framemgr.h"

#define MAX_WORK_COUNT			10
#define SCORE_COMMAND_TIMEOUT		(3 * HZ)

struct score_work {
	struct list_head		list;
	u32				fcount;
	struct score_frame		*frame;
};

struct score_work_list {
	u32				id;
	struct score_work			work[MAX_WORK_COUNT];
	spinlock_t			slock_free;
	spinlock_t			slock_request;
	struct list_head		work_free_head;
	u32				work_free_cnt;
	struct list_head		work_request_head;
	u32				work_request_cnt;
	wait_queue_head_t		wait_queue;
};

enum score_interface_state {
	SCORE_ITF_STATE_OPEN,
	SCORE_ITF_STATE_START,
	SCORE_ITF_STATE_REPLY
};

struct score_interface {
	void __iomem			*code;
	resource_size_t			code_size;
	void __iomem			*regs;
	resource_size_t			regs_size;
	unsigned long			state;

	struct score_framemgr		framemgr;
	void				*cookie;
	u32				done_cnt;

	spinlock_t			process_barrier;
	struct score_frame		*process_msg;
	struct mutex			request_barrier;
	struct score_frame		*request_msg;

	wait_queue_head_t		init_wait_queue;
	wait_queue_head_t		reply_wait_queue;

	struct kthread_worker		worker;
	struct task_struct		*task_itf;

	struct timer_list		timer;
	void				*private_data;
};

int score_interface_probe(struct score_interface *interface,
		struct device *dev,
		void __iomem *regs,
		resource_size_t regs_size,
		u32 irq0);
int score_interface_open(struct score_interface *interface);
int score_interface_close(struct score_interface *interface);
int score_interface_start(struct score_interface *interface);
int score_interface_stop(struct score_interface *interface);


#endif
