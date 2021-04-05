/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef SCORE_VERTEXMGR_H_
#define SCORE_VERTEXMGR_H_

#include <linux/kthread.h>
#include <linux/types.h>

#include "score-framemgr.h"
#include "score-vertex.h"
#include "score-interface.h"

enum score_gframe_state {
	SCORE_GFRAME_STATE_INVALID,
	SCORE_GFRAME_STATE_FREE,
	SCORE_GFRAME_STATE_READY,
	SCORE_GFRAME_STATE_REQUEST,
	SCORE_GFRAME_STATE_PROCESS,
	SCORE_GFRAME_STATE_COMPLETE
};

struct score_gframe {
	struct list_head		list;
	u32				index;
	u32				priority;
	struct score_vertex		*vertex;
	struct score_frame		*frame;
	u32				ticks;
	u32				state;
};

enum score_vertexmgr_state {
	SCORE_VERTEXMGR_OPEN
};

enum score_vertexmgr_client {
	SCORE_VERTEXMGR_CLIENT_VERTEX = 1,
	SCORE_VERTEXMGR_CLIENT_INTERFACE
};

struct score_vertexmgr {
	struct score_vertex_ctx		*vctx[SCORE_MAX_VERTEX];
	atomic_t			periodics;
	unsigned long			state;

	struct mutex			mlock;
	spinlock_t			slock;
	u32				tick_cnt;
	u32				tick_pos;
	u32				sched_cnt;
	u32				sched_pos;
	struct kthread_worker		worker;
	struct task_struct		*task_vertex;
	struct task_struct		*task_timer;

	struct score_resource		*resource;
	struct score_interface		*interface;

	struct score_gframe		gframe[SCORE_MAX_GFRAME];
	struct list_head		gframe_free_list;
	struct list_head		gframe_ready_list;
	struct list_head		gframe_request_list;
	struct list_head		gframe_process_list;
	struct list_head		gframe_complete_list;
	u32				gframe_fre_cnt;
	u32				gframe_rdy_cnt;
	u32				gframe_req_cnt;
	u32				gframe_pro_cnt;
	u32				gframe_com_cnt;
};

int score_vertexmgr_probe(struct score_vertexmgr *vertexmgr);
int score_vertexmgr_open(struct score_vertexmgr *vertexmgr);
int score_vertexmgr_close(struct score_vertexmgr *vertexmgr);
int score_vertexmgr_grp_register(struct score_vertexmgr *vertexmgr, struct score_vertex_ctx *vertex);
int score_vertexmgr_grp_unregister(struct score_vertexmgr *vertexmgr, struct score_vertex_ctx *vertex);
int score_vertexmgr_grp_start(struct score_vertexmgr *vertexmgr, struct score_vertex_ctx *vertex);
int score_vertexmgr_grp_stop(struct score_vertexmgr *vertexmgr, struct score_vertex_ctx *vertex);
int score_vertexmgr_itf_register(struct score_vertexmgr *vertexmgr, struct score_interface *interface);
int score_vertexmgr_itf_unregister(struct score_vertexmgr *vertexmgr, struct score_interface *interface);
void score_vertexmgr_queue(struct score_vertexmgr *vertexmgr, struct score_frame *frame);
#endif
