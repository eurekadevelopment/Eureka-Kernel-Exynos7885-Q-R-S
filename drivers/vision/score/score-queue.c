/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include "score-queue.h"
#include "score-framemgr.h"
#include "score-debug.h"

extern const struct vb_ops vb_score_ops;
extern const struct score_queue_ops score_queue_ops;

int score_queue_open(struct score_queue *queue,
	struct score_memory *memory,
	struct mutex *lock)
{
	int ret = 0;
	struct vb_queue *inq, *otq;

	queue->qops = &score_queue_ops;
	inq = &queue->inqueue;
	otq = &queue->otqueue;
	inq->private_data = queue;
	otq->private_data = queue;

	ret = vb_queue_init(inq, memory->alloc_ctx, memory->vb2_mem_ops, &vb_score_ops, lock, VS4L_DIRECTION_IN);
	if (ret) {
		score_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_init(otq, memory->alloc_ctx, memory->vb2_mem_ops, &vb_score_ops, lock, VS4L_DIRECTION_OT);
	if (ret) {
		score_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_queue_s_format(struct score_queue *queue, struct vs4l_format_list *f)
{
	int ret = 0;
	struct vb_queue *q, *inq, *otq;

	BUG_ON(!queue);
	BUG_ON(!f);

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	if (f->direction == VS4L_DIRECTION_IN)
		q = inq;
	else
		q = otq;

	ret = CALL_QOPS(queue, format, f);
	if (ret) {
		score_err("CALL_QOPS(format) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_s_format(q, f);
	if (ret) {
		score_err("vb_queue_s_format is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_queue_start(struct score_queue *queue)
{
	int ret = 0;
	struct vb_queue *inq, *otq;

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	ret = vb_queue_start(inq);
	if (ret) {
		score_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_start(otq);
	if (ret) {
		score_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = CALL_QOPS(queue, start);
	if (ret) {
		score_err("CALL_QOPS(start) is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_queue_stop(struct score_queue *queue)
{
	int ret = 0;
	struct vb_queue *inq, *otq;

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	ret = CALL_QOPS(queue, stop);
	if (ret) {
		score_err("CALL_QOPS(stop) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_stop(inq);
	if (ret) {
		score_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb_queue_stop(otq);
	if (ret) {
		score_err("vb_queue_init is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_queue_poll(struct score_queue *queue, struct file *file, poll_table *poll)
{
	int ret = 0;
	struct vb_queue *inq, *otq;
	unsigned long events;

	BUG_ON(!queue);
	BUG_ON(!file);
	BUG_ON(!poll);

	events = poll_requested_events(poll);

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	if (events & POLLIN) {
		if (list_empty(&inq->done_list))
			poll_wait(file, &inq->done_wq, poll);

		if (list_empty(&inq->done_list))
			ret |= POLLIN | POLLWRNORM;
	}

	if (events & POLLOUT) {
		if (list_empty(&otq->done_list))
			poll_wait(file, &otq->done_wq, poll);

		if (list_empty(&otq->done_list))
			ret |= POLLOUT | POLLWRNORM;
	}

	return ret;
}

static int __score_buf_prepare(struct vb_queue *q, struct vb_bundle *bundle)
{
	int ret = 0;
	u32 i, j, k;
	struct vb_format *format;
	struct vb_container *container;
	struct vb_buffer *buffer;

	BUG_ON(!q);
	BUG_ON(!bundle);

	/* if (test_bit(VS4L_CL_FLAG_PREPARE, &bundle->flags)) */
	/*	goto p_err; */

	for (i = 0; i < bundle->clist.count; ++i) {
		container = &bundle->clist.containers[i];
		format = container->format;

		switch (container->type) {
		case VS4L_BUFFER_LIST:
			k = container->count;
			break;
		case VS4L_BUFFER_ROI:
			k = container->count;
			break;
		case VS4L_BUFFER_PYRAMID:
			k = container->count;
			break;
		default:
			score_err("unsupported container type\n");
			goto p_err;
		}

		switch (container->memory) {
		case VS4L_MEMORY_DMABUF:
			break;
		case VS4L_MEMORY_VIRTPTR:
			for (j = 0; j < k; ++j) {
				buffer = &container->buffers[j];
				score_note("(%d %d %d %d) (%d) 0x%lx \n",
					buffer->roi.x, buffer->roi.y,
					buffer->roi.w, buffer->roi.h,
					format->size[format->plane],
					buffer->m.userptr);
			}
			break;
		default:
			score_err("unsupported container memory type\n");
			goto p_err;
		}
	}

p_err:
	return ret;
}

static int __score_buf_unprepare(struct vb_queue *q, struct vb_bundle *bundle)
{
	int ret = 0;
	u32 i, j, k;
	struct vb_format *format;
	struct vb_container *container;
	struct vb_buffer *buffer;
	void *mbuf = NULL;

	BUG_ON(!q);
	BUG_ON(!bundle);

	/* if (!test_bit(VS4L_CL_FLAG_PREPARE, &bundle->flags)) */
	/*	goto p_err; */

	for (i = 0; i < bundle->clist.count; ++i) {
		container = &bundle->clist.containers[i];
		format = container->format;

		switch (container->type) {
		case VS4L_BUFFER_LIST:
			k = container->count;
			break;
		case VS4L_BUFFER_ROI:
			k = container->count;
			break;
		case VS4L_BUFFER_PYRAMID:
			k = container->count;
			break;
		default:
			score_err("unsupported container type\n");
			goto p_err;
		}

		switch (container->memory) {
		case VS4L_MEMORY_DMABUF:
			break;
		case VS4L_MEMORY_VIRTPTR:
			for (j = 0; j < k; ++j) {
				buffer = &container->buffers[j];
				score_note("(%d %d %d %d) 0x%lx \n",
					buffer->roi.x, buffer->roi.y,
					buffer->roi.w, buffer->roi.h,
					buffer->m.userptr);

				ret = copy_to_user((void *)buffer->reserved,
						(void *)buffer->m.userptr,
						format->size[format->plane]);
				if (ret) {
					score_err("copy_to_user() is fail(%d)\n", ret);
					goto p_err;
				}

				/* switch kernel space to user space */
				if (buffer->reserved != 0) {
					mbuf = (void *)buffer->m.userptr;
					kfree(mbuf);

					mbuf = NULL;
					buffer->m.userptr = buffer->reserved;;
					buffer->reserved = 0;
				}

				score_note("(%d %d %d %d) 0x%lx \n",
					buffer->roi.x, buffer->roi.y,
					buffer->roi.w, buffer->roi.h,
					buffer->m.userptr);

			}
			break;
		default:
			score_err("unsupported container memory type\n");
			goto p_err;
		}
	}

p_err:
	return ret;
}

int score_queue_qbuf(struct score_queue *queue, struct vs4l_container_list *c)
{
	int ret = 0;
	struct vb_queue *q, *inq, *otq;
	struct vb_bundle *invb, *otvb;
	struct vb_bundle *bundle_map;

#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	inq = &queue->inqueue;
	otq = &queue->otqueue;

	if (c->direction == VS4L_DIRECTION_IN)
		q = inq;
	else
		q = otq;

	ret = vb_queue_qbuf(q, c);
	if (ret) {
		score_err("vb_queue_qbuf is fail(%d)\n", ret);
		goto p_err;
	}

	bundle_map = q->bufs[c->index];
	ret = __score_buf_prepare(q, bundle_map);
	if (ret) {
		score_err("__score_buf_prepare is fail(%d)\n", ret);
		goto p_err;
	}

	if (list_empty(&inq->queued_list))
		goto p_err;

	if (list_empty(&otq->queued_list))
		goto p_err;

	invb = list_first_entry(&inq->queued_list, struct vb_bundle, queued_entry);
	otvb = list_first_entry(&otq->queued_list, struct vb_bundle, queued_entry);

	vb_queue_process(inq, invb);
	vb_queue_process(otq, otvb);

	ret = CALL_QOPS(queue, queue, &invb->clist, &otvb->clist);
	if (ret) {
		score_err("CALL_QOPS(queue) is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_queue_dqbuf(struct score_queue *queue, struct vs4l_container_list *c, bool nonblocking)
{
	int ret = 0;
	struct vb_queue *q;
	struct vb_bundle *bundle;
	struct vb_bundle *bundle_map;

	BUG_ON(!queue);

	if (c->direction == VS4L_DIRECTION_IN)
		q = &queue->inqueue;
	else
		q = &queue->otqueue;

	ret = vb_queue_dqbuf(q, c, nonblocking);
	if (ret) {
		score_err("vb_queue_dqbuf is fail(%d)\n", ret);
		goto p_err;
	}

	if (c->index >= SCORE_MAX_BUFFER) {
		score_err("container index(%d) is invalid\n", c->index);
		ret = -EINVAL;
		goto p_err;
	}

	bundle = q->bufs[c->index];
	if (!bundle) {
		score_err("bundle(%d) is NULL\n", c->index);
		ret = -EINVAL;
		goto p_err;
	}

	if (bundle->clist.index != c->index) {
		score_err("index is NOT matched(%d != %d)\n", bundle->clist.index, c->index);
		ret = -EINVAL;
		goto p_err;
	}

	bundle_map = q->bufs[c->index];
	ret = __score_buf_unprepare(q, bundle_map);
	if (ret) {
		score_err("__score_buf_unprepare is fail(%d)\n", ret);
		goto p_err;
	}

	ret = CALL_QOPS(queue, deque, &bundle->clist);
	if (ret) {
		score_err("CALL_QOPS(deque) is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

void score_queue_done(struct score_queue *queue,
	struct vb_container_list *incl,
	struct vb_container_list *otcl,
	unsigned long flags)
{
	struct vb_queue *inq, *otq;
	struct vb_bundle *invb, *otvb;

	BUG_ON(!queue);
	BUG_ON(!incl);
	BUG_ON(!otcl);

	inq = &queue->inqueue;
	otq = &queue->otqueue;

	if (list_empty(&inq->process_list)) {
		score_err("inqueue is empty\n");
		BUG();
	}

	if (list_empty(&otq->process_list)) {
		score_err("otqueue is empty\n");
		BUG();
	}

	invb = container_of(incl, struct vb_bundle, clist);
	otvb = container_of(otcl, struct vb_bundle, clist);

	if (invb->state != VB_BUF_STATE_PROCESS) {
		score_err("invb state(%d) is invalid\n", invb->state);
		BUG();
	}

	if (otvb->state != VB_BUF_STATE_PROCESS) {
		score_err("otvb state(%d) is invalid\n", otvb->state);
		BUG();
	}

	otvb->flags |= flags;
	vb_queue_done(otq, otvb);

	invb->flags |= flags;
	vb_queue_done(inq, invb);
}
