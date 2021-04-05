/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <video/videonode.h>
#include <media/exynos_mc.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/bug.h>

#include "score-config.h"
#include "vision-ioctl.h"
#include "score-vertex.h"
#include "vision-dev.h"
#include "score-device.h"
#include "score-control.h"
#include "score-debug.h"

const struct score_request_ops score_request_ops;
const struct vision_file_ops score_vertex_fops;
const const struct vertex_ioctl_ops score_vertex_ioctl_ops;

static int __vref_open(struct score_vertex *vertex)
{
	struct score_device *device = container_of(vertex, struct score_device, vertex);
	atomic_set(&vertex->start_cnt.refcount, 0);
	return score_device_open(device);
}

static int __vref_close(struct score_vertex *vertex)
{
	struct score_device *device = container_of(vertex, struct score_device, vertex);
	return score_device_close(device);
}

static int __vref_start(struct score_vertex *vertex)
{
	struct score_device *device = container_of(vertex, struct score_device, vertex);
	return score_device_start(device);
}

static int __vref_stop(struct score_vertex *vertex)
{
	struct score_device *device = container_of(vertex, struct score_device, vertex);
	return score_device_stop(device);
}

static inline void __vref_init(struct score_vertex_refcount *vref,
	struct score_vertex *vertex, int (*first)(struct score_vertex *vertex), int (*final)(struct score_vertex *vertex))
{
	vref->vertex = vertex;
	vref->first = first;
	vref->final = final;
	atomic_set(&vref->refcount, 0);
}

static inline int __vref_get(struct score_vertex_refcount *vref)
{
	return (atomic_inc_return(&vref->refcount) == 1) ? vref->first(vref->vertex) : 0;
}

static inline int __vref_put(struct score_vertex_refcount *vref)
{
	return (atomic_dec_return(&vref->refcount) == 0) ? vref->final(vref->vertex) : 0;
}

int score_vertex_probe(struct score_vertex *vertex, struct device *parent)
{
	int ret = 0;
	struct vision_device *vdev;

	SCORE_TP();
	BUG_ON(!vertex);
	BUG_ON(!parent);

	get_device(parent);
	mutex_init(&vertex->lock);
	__vref_init(&vertex->open_cnt, vertex, __vref_open, __vref_close);
	__vref_init(&vertex->start_cnt, vertex, __vref_start, __vref_stop);

	vertex->rops		= &score_request_ops;
	vertex->done_cnt	= 0;
	vertex->recent		= 0;

	vdev = &vertex->vd;
	snprintf(vdev->name, sizeof(vdev->name), "%s", SCORE_VERTEX_NAME);
	vdev->fops		= &score_vertex_fops;
	vdev->ioctl_ops		= &score_vertex_ioctl_ops;
	vdev->release		= NULL;
	vdev->lock		= NULL;
	vdev->parent		= parent;
	vdev->type		= VISION_DEVICE_TYPE_VERTEX;
	dev_set_drvdata(&vdev->dev, vertex);

	ret = vision_register_device(vdev, SCORE_VERTEX_MINOR, score_vertex_fops.owner);
	if (ret) {
		probe_err("vision_register_device is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

/*
 * =============================================================================
 * Video File Opertation
 * =============================================================================
 */

static int score_vertex_open(struct file *file)
{
	int ret = 0;
	struct score_vertex *vertex = dev_get_drvdata(&vision_devdata(file)->dev);
	struct score_device *device = container_of(vertex, struct score_device, vertex);
	struct score_system *system;
	struct mutex *lock = &vertex->lock;
	struct score_vertex_ctx *vctx;
	struct score_framemgr *framemgr;
	struct score_buftracker *buftracker;

	SCORE_TP();
	BUG_ON(!device);

	if (mutex_lock_interruptible(lock)) {
		score_err("mutex_lock_interruptible is fail\n");
		return -ERESTARTSYS;
	}

	ret = __vref_get(&vertex->open_cnt);
	if (ret) {
		score_err("vref_get is fail(%d)", ret);
		goto p_err;
	}

	/* HACK : static allocation */
	vctx = kzalloc(sizeof(struct score_vertex_ctx), GFP_KERNEL);

	ret = score_vertexmgr_grp_register(&device->vertexmgr, vctx);
	if (ret) {
		score_err("score_vertexmgr_grp_register is fail(%d)", ret);
		goto p_err;
	}

	system = &device->system;
	vctx->vertex = vertex;
	vctx->cookie = (void *)&device->vertexmgr;
	vctx->memory = system->memory;
	buftracker = &vctx->buftracker;
	mutex_init(&vctx->lock);

	ret = score_buftracker_init(buftracker);
	if (ret) {
		score_err("score_buftracker_init is fail(%d)", ret);
		goto p_err;
	}

	ret = score_queue_open(&vctx->queue, &device->system.memory, &vctx->lock);
	if (ret) {
		score_err("score_queue_open is fail(%d)", ret);
		goto p_err;
	}

	file->private_data = vctx;
	vctx->state = BIT(SCORE_VERTEX_OPEN);

	framemgr = &vctx->framemgr;
	framemgr->id = vctx->id;
	framemgr->sindex = 0;
	spin_lock_init(&framemgr->slock);

	ret = score_frame_init(framemgr, (void *)vctx);
	if (ret) {
		score_err("vctx_frame_init is fail(%d)\n", ret);
		kfree(vctx);
		goto p_err;
	}

p_err:
	score_iinfo("%s() vctx:%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

static int score_vertex_close(struct file *file)
{
	int ret = 0;
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_vertex *vertex = vctx->vertex;
	struct score_device *device = container_of(vertex, struct score_device, vertex);
	struct mutex *lock = &vertex->lock;
	int id = vctx->id;
	struct score_buftracker *buftracker = &vctx->buftracker;

	SCORE_TP();
	if (mutex_lock_interruptible(lock)) {
		score_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	score_vertexmgr_grp_unregister(&device->vertexmgr, vctx);

	ret = score_buftracker_dump(buftracker);

	/*
	* ret = score_buftracker_remove_all(buftracker);
	* if (ret) {
	*	score_ierr("score__buftracker_remove_all is fail(%d)\n", vctx, ret);
	*	goto p_err;
	* }
	*/

	kfree(vctx);

	ret = __vref_put(&vertex->open_cnt);
	if (ret) {
		score_err("vref_put is fail(%d)", ret);
		goto p_err;
	}

p_err:
	score_event_msg("[I%d]:%d\n", id, ret);
	mutex_unlock(lock);
	return ret;
}

static unsigned int score_vertex_poll(struct file *file,
	poll_table *poll)
{
	int ret = 0;
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_queue *queue = &vctx->queue;

#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	if (!(vctx->state & BIT(SCORE_VERTEX_START))) {
		score_ierr("invalid state(%X)", vctx, vctx->state);
		ret |= POLLERR;
		goto p_err;
	}

	ret = score_queue_poll(queue, file, poll);

p_err:
	return ret;
}

const struct vision_file_ops score_vertex_fops = {
	.owner		= THIS_MODULE,
	.open		= score_vertex_open,
	.release	= score_vertex_close,
	.poll		= score_vertex_poll,
	.ioctl		= vertex_ioctl,
	.compat_ioctl	= vertex_compat_ioctl32
};

/*
 * =============================================================================
 * Video Ioctl Opertation
 * =============================================================================
 */
static int score_vertex_s_format(struct file *file, struct vs4l_format_list *flist)
{
	int ret = 0;
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;

#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	if (mutex_lock_interruptible(lock)) {
		score_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	/*
	* if (!(vctx->state & (BIT(SCORE_VERTEX_GRAPH) | BIT(SCORE_VERTEX_FORMAT)))) {
	*	score_ierr("invalid state(%X)\n", vctx, vctx->state);
	*	ret = -EINVAL;
	*	goto p_err;
	* }
	*/

	ret = score_queue_s_format(queue, flist);
	if (ret) {
		score_ierr("score_queue_s_format is fail(%d)\n", vctx, ret);
		goto p_err;
	}

	/* vctx->state = BIT(SCORE_VERTEX_FORMAT); */

p_err:
	score_iinfo("%s() vctx:%d\n", vctx, __func__, ret);
	mutex_unlock(lock);

	return ret;
}

static int score_vertex_s_param(struct file *file, struct vs4l_param_list *plist)
{
	int ret = 0;
#if 0
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_vertex *vertex = dev_get_drvdata(&vision_devdata(file)->dev);
	struct score_device *device = container_of(vertex, struct score_device, vertex);
	struct mutex *lock = &vctx->lock;

#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	if (mutex_lock_interruptible(lock)) {
		score_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	if (!(vctx->state & BIT(SCORE_VERTEX_STOP))) {
		score_ierr("invalid state(%X)\n", vctx, vctx->state);
		ret = -EINVAL;
		goto p_err;
	}

	ret = score_device_param(device, plist);
	if (ret) {
		score_err("score_device_param is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	score_iinfo("%s() vctx:%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
#endif
	return ret;
}

static int score_vertex_s_ctrl(struct file *file, struct vs4l_ctrl *ctrl)
{
	int ret = 0;
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_vertex *vertex = vctx->vertex;
	struct score_device *device = container_of(vertex, struct score_device, vertex);
	struct mutex *lock = &vctx->lock;
	/* int id = vctx->id; */

	if (mutex_lock_interruptible(lock)) {
		score_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	switch (ctrl->ctrl) {
		break;
	case SCORE_CTRL_FWINDEX:
		device->fw_index = ctrl->value;
		score_device_fw_ld(device, device->fw_index);
		break;
	default:
		score_ierr("request control is invalid(%d)\n", vctx, ctrl->ctrl);
		ret = -EINVAL;
		break;
	}

	score_iinfo("%s(%d):%d\n", vctx, __func__, ctrl->ctrl, ret);
	mutex_unlock(lock);
	return ret;
}

static int score_vertex_qbuf(struct file *file, struct vs4l_container_list *clist)
{
	int ret = 0;
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;

#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	if (mutex_lock_interruptible(lock)) {
		score_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	/*
	* if (!(vctx->state & BIT(SCORE_VERTEX_START))) {
	*	score_ierr("(%d) invalid state(%X)\n", vctx, clist->direction, vctx->state);
	*	ret = -EINVAL;
	*	goto p_err;
	* }
	*/

	ret = score_queue_qbuf(queue, clist);
	if (ret) {
		score_ierr("(%d) score_queue_qbuf is fail(%d)\n", vctx, clist->direction, ret);
		goto p_err;
	}

p_err:
	mutex_unlock(lock);
	return ret;
}

static int score_vertex_dqbuf(struct file *file, struct vs4l_container_list *clist)
{
	int ret = 0;
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;
	bool nonblocking = file->f_flags & O_NONBLOCK;

#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	if (mutex_lock_interruptible(lock)) {
		score_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	/*
	* if (!(vctx->state & BIT(SCORE_VERTEX_START))) {
	*	score_ierr("(%d) invalid state(%X)\n", vctx, clist->direction, vctx->state);
	*	ret = -EINVAL;
	*	goto p_err;
	* }
	*/

	ret = score_queue_dqbuf(queue, clist, nonblocking);
	if (ret) {
		score_ierr("(%d) score_queue_dqbuf is fail(%d)\n", vctx, clist->direction, ret);
		goto p_err;
	}

p_err:
	mutex_unlock(lock);
	return ret;
}

static int score_vertex_streamon(struct file *file)
{
	int ret = 0;
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_vertex *vertex = vctx->vertex;
	struct score_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;

#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	if (mutex_lock_interruptible(lock)) {
		score_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	/*
	* if (!(vctx->state & (BIT(SCORE_VERTEX_FORMAT) | BIT(SCORE_VERTEX_STOP)))) {
	*	score_ierr("invalid state(%X)\n", vctx, vctx->state);
	*	ret = -EINVAL;
	*	goto p_err;
	* }
	*/

	ret = __vref_get(&vertex->start_cnt);
	if (ret) {
		score_err("vref_get is fail(%d)\n", ret);
		goto p_err;
	}

	ret = score_queue_start(queue);
	if (ret) {
		score_ierr("score_queue_start is fail(%d)\n", vctx, ret);
		goto p_err;
	}

	vctx->state = BIT(SCORE_VERTEX_START);

p_err:
	score_iinfo("%s() vctx:%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

static int score_vertex_streamoff(struct file *file)
{
	int ret = 0;
	struct score_vertex_ctx *vctx = file->private_data;
	struct score_vertex *vertex = vctx->vertex;
	struct score_queue *queue = &vctx->queue;
	struct mutex *lock = &vctx->lock;
	struct score_buftracker *buftracker = &vctx->buftracker;

#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	if (mutex_lock_interruptible(lock)) {
		score_ierr("mutex_lock_interruptible is fail\n", vctx);
		return -ERESTARTSYS;
	}

	/*
	* if (!(vctx->state & BIT(SCORE_VERTEX_START))) {
	*	score_ierr("invalid state(%X)\n", vctx, vctx->state);
	*	ret = -EINVAL;
	*	goto p_err;
	* }
	*/

	/* score_buftracker_dump(buftracker); */

	ret = score_buftracker_remove_all(buftracker);
	if (ret) {
		score_ierr("score_buftracker_remove_all is fail(%d)\n",
				vctx, ret);
		goto p_err;
	}

	ret = score_buftracker_remove_userptr_all(buftracker);
	if (ret) {
		score_ierr("remove_userptr_all is fail(%d)\n", vctx, ret);
		goto p_err;
	}

	ret = score_queue_stop(queue);
	if (ret) {
		score_ierr("score_queue_stop is fail(%d)\n", vctx, ret);
		goto p_err;
	}

	ret = __vref_put(&vertex->start_cnt);
	if (ret) {
		score_err("vref_put is fail(%d)\n", ret);
		goto p_err;
	}

	vctx->state = BIT(SCORE_VERTEX_STOP);

p_err:
	score_iinfo("%s() vctx:%d\n", vctx, __func__, ret);
	mutex_unlock(lock);
	return ret;
}

const struct vertex_ioctl_ops score_vertex_ioctl_ops = {
	.vertexioc_s_format	= score_vertex_s_format,
	.vertexioc_s_param	= score_vertex_s_param,
	.vertexioc_s_ctrl	= score_vertex_s_ctrl,
	.vertexioc_qbuf		= score_vertex_qbuf,
	.vertexioc_dqbuf	= score_vertex_dqbuf,
	.vertexioc_streamon	= score_vertex_streamon,
	.vertexioc_streamoff	= score_vertex_streamoff
};

int score_vertex_start(struct score_queue *queue)
{
	int ret = 0;
	struct score_vertex_ctx *vctx;

	BUG_ON(!queue);

	vctx = container_of(queue, struct score_vertex_ctx, queue);

	return ret;
}

int score_vertex_stop(struct score_queue *queue)
{
	int ret = 0;
	struct score_vertex_ctx *vctx;

	BUG_ON(!queue);

	vctx = container_of(queue, struct score_vertex_ctx, queue);

	return ret;
}

int score_vertex_format(struct score_queue *queue, struct vs4l_format_list *f)
{
	int ret = 0;
#if 0
	struct score_vertex_ctx *vctx;

	BUG_ON(!queue);
	BUG_ON(!f);

	vctx = container_of(queue, struct score_vertex_ctx, queue);
#endif
	return ret;
}

static int score_vertex_queue(struct score_queue *queue, struct vb_container_list *incl, struct vb_container_list *otcl)
{
	int ret = 0;
	unsigned long flag;
	struct score_vertex_ctx *vctx;
	struct score_framemgr *framemgr;
	struct score_frame *frame;

	BUG_ON(!queue);
	BUG_ON(!incl);
	BUG_ON(!incl->index >= SCORE_MAX_FRAME);
	BUG_ON(!otcl);
	BUG_ON(!otcl->index >= SCORE_MAX_FRAME);

	vctx = container_of(queue, struct score_vertex_ctx, queue);
	framemgr = &vctx->framemgr;

	/*
	* if (incl->id != otcl->id) {
	*	score_warn("buffer id is incoincidence(%d, %d)\n", incl->id, otcl->id);
	*	otcl->id = incl->id;
	* }
	*/

	framemgr_e_barrier_irqs(framemgr, 0, flag);
	score_frame_pick_fre_to_req(framemgr, &frame);
	framemgr_x_barrier_irqr(framemgr, 0, flag);

	if (!frame) {
		score_ierr("frame is lack\n", vctx);
		score_frame_print_all(framemgr);
		ret = -ENOMEM;
		goto p_err;
	}

	/* HACK */
	/*
	* graph->inhash[incl->index] = frame->index;
	* graph->othash[otcl->index] = frame->index;
	* graph->input_cnt++;
	*/

	frame->id = incl->id;
	frame->incl = incl;
	frame->otcl = otcl;
	frame->message = SCORE_FRAME_REQUEST;
	clear_bit(VS4L_CL_FLAG_TIMESTAMP, &frame->flags);

	/*
	* if ((incl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP)) ||
	*	(otcl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP))) {
	*	set_bit(VS4L_CL_FLAG_TIMESTAMP, &frame->flags);
	*	score_get_timestamp(&frame->time[SCORE_TMP_QUEUE]);
	* }
	*/

	framemgr_e_barrier_irqs(framemgr, 0, flag);
	score_frame_trans_req_to_pro(framemgr, frame);
	framemgr_x_barrier_irqr(framemgr, 0, flag);

	score_vertexmgr_queue((struct score_vertexmgr *)vctx->cookie, frame);

p_err:
	return ret;
}

static int score_vertex_deque(struct score_queue *queue, struct vb_container_list *clist)
{
	int ret = 0;
	/* u32 findex; */
	struct score_vertex_ctx *vctx;
	struct score_framemgr *framemgr;
	struct score_frame *frame;

	BUG_ON(!queue);
	BUG_ON(!clist);
	BUG_ON(!clist->index >= SCORE_MAX_FRAME);

	vctx = container_of(queue, struct score_vertex_ctx, queue);
	framemgr = &vctx->framemgr;
	/* HACK */
	frame = vctx->processing_frame;
#if 0
	if (!test_bit(SCORE_GRAPH_STATE_START, &graph->state)) {
		score_ierr("graph is NOT start\n", graph);
		ret = -EINVAL;
		goto p_err;
	}

	if (clist->direction == VS4L_DIRECTION_IN)
		findex = graph->inhash[clist->index];
	else
		findex = graph->othash[clist->index];

	if (findex >= SCORE_MAX_FRAME) {
		score_ierr("frame index(%d) invalid\n", graph, findex);
		BUG();
	}

	frame = &framemgr->frame[findex];
	if (frame->state != SCORE_FRAME_STATE_COMPLETE) {
		score_ierr("frame state(%d) is invalid\n", graph, frame->state);
		BUG();
	}

	if (clist->direction == VS4L_DIRECTION_IN) {
		if (frame->incl != clist) {
			score_ierr("incl ptr is invalid(%p != %p)\n", graph, frame->incl, clist);
			BUG();
		}

		graph->inhash[clist->index] = SCORE_MAX_FRAME;
		frame->incl = NULL;
	} else {
		if (frame->otcl != clist) {
			score_ierr("otcl ptr is invalid(%p != %p)\n", graph, frame->otcl, clist);
			BUG();
		}

		graph->othash[clist->index] = SCORE_MAX_FRAME;
		frame->otcl = NULL;
	}

	if (frame->incl || frame->otcl)
		goto p_err;

p_err:
#endif
	return ret;
}

static int score_vertex_prepare(struct vb_queue *q, struct vb_container_list *clist)
{
	int ret = 0;

	return ret;
}

static int score_vertex_unprepare(struct vb_queue *q, struct vb_container_list *clist)
{
	int ret = 0;

	return ret;
}

static int score_request_control(struct score_vertex *vertex, struct score_frame *frame)
{
	int ret = 0;
#if 0
	struct score_framemgr *framemgr;

	BUG_ON(!request);
	BUG_ON(!frame);
#endif

	return ret;
}

static int score_request_request(struct score_vertex *vertex, struct score_frame *frame)
{
	int ret = 0;
#if 0
	unsigned long flags;
	struct score_framemgr *framemgr;

	BUG_ON(!vertex);
	BUG_ON(!frame);

	framemgr = &graph->framemgr;

	if (frame->state != SCORE_FRAME_STATE_REQUEST) {
		score_ierr("frame state(%d) is invalid\n", graph, frame->state);
		BUG();
	}

	framemgr_e_barrier_irqs(framemgr, 0, flags);
	score_frame_trans_req_to_pre(framemgr, frame);
	framemgr_x_barrier_irqr(framemgr, 0, flags);

	if (test_bit(VS4L_CL_FLAG_TIMESTAMP, &frame->flags))
		score_get_timestamp(&frame->time[SCORE_TMP_REQUEST]);
#endif

	return ret;
}

static int score_request_process(struct score_vertex *vertex, struct score_frame *frame)
{
	int ret = 0;
#if 0
	unsigned long flags;
	struct score_framemgr *framemgr;

	BUG_ON(!vertex);
	BUG_ON(!frame);

	framemgr = &graph->framemgr;

	if (frame->state != SCORE_FRAME_STATE_PREPARE) {
		score_ierr("frame state(%d) is invalid\n", graph, frame->state);
		BUG();
	}

	framemgr_e_barrier_irqs(framemgr, FMGR_IDX_0, flags);
	score_frame_trans_pre_to_pro(framemgr, frame);
	framemgr_x_barrier_irqr(framemgr, FMGR_IDX_0, flags);

	if (test_bit(VS4L_CL_FLAG_TIMESTAMP, &frame->flags))
		score_get_timestamp(&frame->time[SCORE_TMP_PROCESS]);
#endif

	return ret;
}

static int score_request_cancel(struct score_vertex *vertex, struct score_frame *frame)
{
	int ret = 0;

	return ret;
}

static int score_request_done(struct score_vertex *vertex, struct score_frame *frame)
{
	int ret = 0;
	/* unsigned long flags; */
	unsigned long result;
	struct score_framemgr *framemgr;
	struct score_queue *queue;
	struct vb_container_list *incl, *otcl;
	struct score_vertex_ctx *vctx;
	/* struct score_gframe *gframe; */

	BUG_ON(!vertex);
	BUG_ON(!frame);

	vctx = frame->owner;
	vertex = vctx->vertex;
	framemgr = &vctx->framemgr;
	queue = &vctx->queue;
	incl = frame->incl;
	otcl = frame->otcl;
	result = 0;

	/*
	* if (!test_bit(SCORE_GRAPH_STATE_START, &graph->state)) {
	*	score_ierr("graph is NOT start\n", graph);
	*	BUG();
	* }

	* if (frame->state != SCORE_FRAME_STATE_PROCESS) {
	*	score_ierr("frame state(%d) is invalid\n", graph, frame->state);
	*	BUG();
	* }
	*/

	if (frame->message == SCORE_FRAME_DONE) {
#ifdef DBG_STREAMING
		score_iinfo("DONE(%d, %d)\n", vctx, frame->index, frame->id);
#endif
		set_bit(VS4L_CL_FLAG_DONE, &result);
	} else {
#ifdef DBG_STREAMING
		score_iinfo("NDONE(%d, %d)\n", vctx, frame->index, frame->id);
#endif
		set_bit(VS4L_CL_FLAG_DONE, &result);
		set_bit(VS4L_CL_FLAG_INVALID, &result);
	}

	if (test_bit(VS4L_CL_FLAG_TIMESTAMP, &frame->flags)) {
		/* score_get_timestamp(&frame->time[SCORE_TMP_DONE]); */

		if (incl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP))
			memcpy(incl->timestamp, frame->time, sizeof(frame->time));

		if (otcl->flags & (1 << VS4L_CL_FLAG_TIMESTAMP))
			memcpy(otcl->timestamp, frame->time, sizeof(frame->time));

#if 0
#ifdef DBG_TIMEMEASURE
		score_irinfo("[TM] QR(%ld), RR(%ld), RP(%ld), PD(%ld)\n", graph, frame,
			SCORE_TIME_IN_US(frame->time[SCORE_TMP_REQUEST]) -
			SCORE_TIME_IN_US(frame->time[SCORE_TMP_QUEUE]),
			SCORE_TIME_IN_US(frame->time[SCORE_TMP_RESOURCE]) -
			SCORE_TIME_IN_US(frame->time[SCORE_TMP_REQUEST]),
			SCORE_TIME_IN_US(frame->time[SCORE_TMP_PROCESS]) -
			SCORE_TIME_IN_US(frame->time[SCORE_TMP_RESOURCE]),
			SCORE_TIME_IN_US(frame->time[SCORE_TMP_DONE]) -
			SCORE_TIME_IN_US(frame->time[SCORE_TMP_PROCESS]));
#endif
#endif
	}

	/* HACK */
	/* framemgr_e_barrier_irqs(framemgr, 0, flags); */
	/* score_frame_trans_pro_to_com(framemgr, frame); */
	/* framemgr_x_barrier_irqr(framemgr, 0, flags); */

	vertex->recent = frame->id;
	vertex->done_cnt++;
	/* HACK */
	vctx->processing_frame = frame;

	score_queue_done(queue, incl, otcl, result);

	return ret;
}

const struct score_queue_ops score_queue_ops = {
	.start		= score_vertex_start,
	.stop		= score_vertex_stop,
	.format 	= score_vertex_format,
	.queue		= score_vertex_queue,
	.deque		= score_vertex_deque
};

const struct vb_ops vb_score_ops = {
	.buf_prepare = score_vertex_prepare,
	.buf_unprepare = score_vertex_unprepare
};

const struct score_request_ops score_request_ops = {
	.control	= score_request_control,
	.request	= score_request_request,
	.process	= score_request_process,
	.cancel 	= score_request_cancel,
	.done		= score_request_done,
};
