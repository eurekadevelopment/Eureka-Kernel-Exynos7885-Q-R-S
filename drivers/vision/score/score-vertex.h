/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SCORE_VERTEX_H_
#define SCORE_VERTEX_H_

#define SCORE_VERTEX_NAME			"vertex"
#define SCORE_VERTEX_MINOR			1

#include "vision-dev.h"
#include "vision-ioctl.h"
#include "score-queue.h"
#include "score-memory.h"
#include "score-buftracker.h"

struct score_vertex;

enum score_vertex_state {
	SCORE_VERTEX_OPEN,
	SCORE_VERTEX_GRAPH,
	SCORE_VERTEX_FORMAT,
	SCORE_VERTEX_START,
	SCORE_VERTEX_STOP
};

struct score_vertex_refcount {
	atomic_t			refcount;
	struct score_vertex		*vertex;
	int				(*first)(struct score_vertex *vertex);
	int				(*final)(struct score_vertex *vertex);
};

struct score_request_ops {
	int (*control)(struct score_vertex *vertex, struct score_frame *frame);
	int (*request)(struct score_vertex *vertex, struct score_frame *frame);
	int (*process)(struct score_vertex *vertex, struct score_frame *frame);
	int (*cancel)(struct score_vertex *vertex, struct score_frame *frame);
	int (*done)(struct score_vertex *vertex, struct score_frame *frame);
};

struct score_vertex {
	struct mutex			lock;
	struct vision_device		vd;
	struct score_vertex_refcount	open_cnt;
	struct score_vertex_refcount	start_cnt;
	const struct score_request_ops	*rops;
	u32				done_cnt;
	u32				recent;
};

struct score_vertex_ctx {
	u32				state;
	u32				id;
	struct mutex			lock;
	struct score_vertex		*vertex;

	u32				priority;
	struct score_framemgr		framemgr;
	struct score_frame		control;
	wait_queue_head_t		control_wq;
	struct score_memory		memory;
	struct score_queue		queue;
	void				*cookie;
	struct score_buftracker		buftracker;

	/* HACK */
	struct score_frame		*processing_frame;
};

int score_vertex_probe(struct score_vertex *vertex, struct device *parent);

#define CALL_ROPS(g, op, ...)	(((g)->rops->op) ? ((g)->rops->op(g, ##__VA_ARGS__)) : 0)

#endif
