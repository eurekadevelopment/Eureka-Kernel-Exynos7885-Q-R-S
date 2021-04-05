/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
#include <linux/bug.h>

#include "score-framemgr.h"
#include "score-device.h"
#include "score-vertexmgr.h"
#include "score-debug.h"

#define TIMER_CHECK_POINT(v)	vertexmgr->tick_pos = v
#define VERTEX_CHECK_POINT(v)	vertexmgr->sched_pos = v

void __score_gframe_s_free(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
#if 1
	gframe->state = SCORE_GFRAME_STATE_FREE;

	list_add_tail(&gframe->list, &vertexmgr->gframe_free_list);
	vertexmgr->gframe_fre_cnt++;
#endif
}

void __score_gframe_g_free(struct score_vertexmgr *vertexmgr, struct score_gframe **gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);

#if 1
	if (vertexmgr->gframe_fre_cnt &&
		(*gframe = container_of(vertexmgr->gframe_free_list.next, struct score_gframe, list))) {
		list_del(&(*gframe)->list);
		vertexmgr->gframe_fre_cnt--;
		(*gframe)->state = SCORE_FRAME_STATE_INVALID;
	} else {
		*gframe = NULL;
	}
#endif
}

void __score_gframe_free_head(struct score_vertexmgr *vertexmgr, struct score_gframe **gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);

#if 1
	if (vertexmgr->gframe_fre_cnt)
		*gframe = container_of(vertexmgr->gframe_free_list.next, struct score_gframe, list);
	else
		*gframe = NULL;
#endif
}

void __score_gframe_s_ready(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
#if 1
	gframe->state = SCORE_GFRAME_STATE_FREE;

	list_add_tail(&gframe->list, &vertexmgr->gframe_ready_list);
	vertexmgr->gframe_rdy_cnt++;
#endif
}

static inline void __score_gframe_g_ready(struct score_vertexmgr *vertexmgr,
	struct score_gframe **gframe)
{
#if 1
	if (vertexmgr->gframe_rdy_cnt &&
		(*gframe = container_of(vertexmgr->gframe_ready_list.next, struct score_gframe, list))) {
		list_del(&(*gframe)->list);
		vertexmgr->gframe_rdy_cnt--;
		(*gframe)->state = SCORE_FRAME_STATE_INVALID;
	} else {
		*gframe = NULL;
	}
#endif
}

void __score_gframe_ready_head(struct score_vertexmgr *vertexmgr, struct score_gframe **gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
#if 1
	if (vertexmgr->gframe_rdy_cnt)
		*gframe = container_of(vertexmgr->gframe_ready_list.next, struct score_gframe, list);
	else
		*gframe = NULL;
#endif
}

#if 0
static void __score_gframe_s_request(struct score_vertexmgr *vertexmgr,
	struct score_gframe *prev,
	struct score_gframe *gframe,
	struct score_gframe *next)
{
	gframe->state = SCORE_GFRAME_STATE_REQUEST;

	if (prev && next) {
		next->list.prev = &gframe->list;
	        gframe->list.next = &next->list;
	        gframe->list.prev = &prev->list;
	        prev->list.next = &gframe->list;
	} else if (prev) {
		list_add_tail(&gframe->list, &vertexmgr->gframe_request_list);
	} else {
		list_add(&gframe->list, &vertexmgr->gframe_request_list);
	}

	vertexmgr->gframe_req_cnt++;
}
#endif

void __score_gframe_request_head(struct score_vertexmgr *vertexmgr, struct score_gframe **gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
#if 0
	if (vertexmgr->gframe_req_cnt)
		*gframe = container_of(vertexmgr->gframe_request_list.next, struct score_gframe, list);
	else
		*gframe = NULL;
#endif
}

void __score_gframe_s_process(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
#if 1
	gframe->state = SCORE_GFRAME_STATE_PROCESS;
	list_add_tail(&gframe->list, &vertexmgr->gframe_process_list);
	vertexmgr->gframe_pro_cnt++;
#endif
}

static void __score_gframe_s_complete(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);

	gframe->state = SCORE_GFRAME_STATE_COMPLETE;
	list_add_tail(&gframe->list, &vertexmgr->gframe_complete_list);
	vertexmgr->gframe_com_cnt++;
}

void __score_gframe_trans_fre_to_rdy(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
	BUG_ON(!vertexmgr->gframe_fre_cnt);
#if 1
	list_del(&gframe->list);
	vertexmgr->gframe_fre_cnt--;
	__score_gframe_s_ready(vertexmgr, gframe);
#endif
}

void __score_gframe_trans_rdy_to_fre(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
	BUG_ON(!vertexmgr->gframe_rdy_cnt);
#if 0
	list_del(&gframe->list);
	vertexmgr->gframe_rdy_cnt--;
	__score_gframe_s_free(vertexmgr, gframe);
#endif
}

void __score_gframe_trans_req_to_pro(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
	BUG_ON(!vertexmgr->gframe_req_cnt);
#if 0
	list_del(&gframe->list);
	vertexmgr->gframe_req_cnt--;
	__score_gframe_s_process(vertexmgr, gframe);
#endif
}

void __score_gframe_trans_req_to_fre(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
	BUG_ON(!vertexmgr->gframe_req_cnt);
#if 0
	list_del(&gframe->list);
	vertexmgr->gframe_req_cnt--;
	__score_gframe_s_free(vertexmgr, gframe);
#endif
}

void __score_gframe_trans_pro_to_fre(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
	BUG_ON(!vertexmgr->gframe_pro_cnt);
#if 1
	list_del(&gframe->list);
	vertexmgr->gframe_pro_cnt--;
	__score_gframe_s_free(vertexmgr, gframe);
#endif
}
#if 1
static void __score_gframe_trans_pro_to_com(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
	BUG_ON(!vertexmgr->gframe_pro_cnt);

	list_del(&gframe->list);
	vertexmgr->gframe_pro_cnt--;
	__score_gframe_s_complete(vertexmgr, gframe);
}
#endif
static void __score_gframe_trans_com_to_fre(struct score_vertexmgr *vertexmgr, struct score_gframe *gframe)
{
	BUG_ON(!vertexmgr);
	BUG_ON(!gframe);
	BUG_ON(!vertexmgr->gframe_com_cnt);

	list_del(&gframe->list);
	vertexmgr->gframe_com_cnt--;
	__score_gframe_s_free(vertexmgr, gframe);
}
#if 0
static int __score_itf_enum(struct score_interface *interface,
	struct score_vertex *vertex)
{
	int ret = 0;
	unsigned long flags;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;

	BUG_ON(!interface);
	BUG_ON(!vertex);

	iframemgr = &interface->framemgr;

	ret = score_hw_enum(interface);
	if (ret) {
		score_err("score_hw_enum is fail(%d)\n", ret);
		goto p_err;
	}

	framemgr_e_barrier_irqs(iframemgr, 0, flags);
	score_frame_pick_fre_to_req(iframemgr, &iframe);
	framemgr_x_barrier_irqr(iframemgr, 0, flags);

	if (!iframe) {
		score_err("iframe is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

	iframe->message = SCORE_FRAME_INIT;
	iframe->gindex = SCORE_MAX_GFRAME;
	iframe->findex = SCORE_MAX_FRAME;
	iframe->param0 = 0;
	iframe->param1 = vertex->id;
	iframe->param2 = 0;
	iframe->param3 = 0;

	ret = score_hw_init(interface, iframe);
	if (ret) {
		score_err("score_hw_init is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

static int __score_itf_create(struct score_interface *interface,
	struct score_vertex *vertex)
{
	int ret = 0;
	unsigned long flags;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;

	BUG_ON(!interface);
	BUG_ON(!vertex);

	iframemgr = &interface->framemgr;

	framemgr_e_barrier_irqs(iframemgr, 0, flags);
	score_frame_pick_fre_to_req(iframemgr, &iframe);
	framemgr_x_barrier_irqr(iframemgr, 0, flags);

	if (!iframe) {
		score_err("iframe is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

	iframe->message = SCORE_FRAME_CREATE;
	iframe->gindex = SCORE_MAX_GFRAME;
	iframe->findex = SCORE_MAX_FRAME;
	iframe->param0 = (ulong)vertex->desc_mtask;
	iframe->param1 = vertex->id;
	iframe->param2 = 0;
	iframe->param3 = 0;

	ret = score_hw_create(interface, iframe);
	if (ret) {
		score_err("score_hw_create is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

static int __score_itf_destroy(struct score_interface *interface,
	struct score_vertex *vertex)
{
	int ret = 0;
	unsigned long flags;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;

	BUG_ON(!interface);
	BUG_ON(!vertex);

	iframemgr = &interface->framemgr;

	framemgr_e_barrier_irqs(iframemgr, 0, flags);
	score_frame_pick_fre_to_req(iframemgr, &iframe);
	framemgr_x_barrier_irqr(iframemgr, 0, flags);

	if (!iframe) {
		score_err("iframe is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

	iframe->message = SCORE_FRAME_DESTROY;
	iframe->gindex = SCORE_MAX_GFRAME;
	iframe->findex = SCORE_MAX_FRAME;
	iframe->param0 = (ulong)vertex->desc_mtask;
	iframe->param1 = vertex->id;
	iframe->param2 = 0;
	iframe->param3 = 0;

	ret = score_hw_destroy(interface, iframe);
	if (ret) {
		score_err("score_hw_destory is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

static int __score_itf_allocate(struct score_interface *interface,
	struct score_vertex *vertex)
{
	int ret = 0;
	unsigned long flags;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;

	BUG_ON(!interface);
	BUG_ON(!vertex);

	iframemgr = &interface->framemgr;

	framemgr_e_barrier_irqs(iframemgr, 0, flags);
	score_frame_pick_fre_to_req(iframemgr, &iframe);
	framemgr_x_barrier_irqr(iframemgr, 0, flags);

	if (!iframe) {
		score_err("iframe is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

	iframe->message = SCORE_FRAME_ALLOCATE;
	iframe->gindex = SCORE_MAX_GFRAME;
	iframe->findex = SCORE_MAX_FRAME;
	iframe->param0 = (ulong)vertex->desc_mtask;
	iframe->param1 = vertex->id;
	iframe->param2 = 0;
	iframe->param3 = 0;

	ret = score_hw_allocate(interface, iframe);
	if (ret) {
		score_err("score_hw_allocate is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

static int __score_itf_process(struct score_interface *interface,
	struct score_vertex *vertex,
	struct score_frame *frame)
{
	int ret = 0;
	unsigned long flags;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;

	BUG_ON(!interface);
	BUG_ON(!vertex);
	BUG_ON(!frame);

	iframemgr = &interface->framemgr;

	framemgr_e_barrier_irqs(iframemgr, 0, flags);
	score_frame_pick_fre_to_req(iframemgr, &iframe);
	framemgr_x_barrier_irqr(iframemgr, 0, flags);

	if (!iframe) {
		score_err("iframe is NULL\n");
		ret = -ENOMEM;
		goto p_err;
	}

	iframe->message = SCORE_FRAME_PROCESS;
	iframe->gindex = frame->gindex;
	iframe->findex = frame->index;
	iframe->param0 = (ulong)vertex->desc_mtask;
	iframe->param1 = vertex->id;
	iframe->param2 = 0;
	iframe->param3 = 0;

	ret = CALL_GOPS(vertex, process, frame);
	if (ret) {
		score_err("GOPS(process) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = score_hw_process(interface, iframe);
	if (ret) {
		score_err("score_hw_process is fail(%d)\n", ret);
		goto p_err;
	}

#ifdef DBG_STREAMING
	score_iinfo("PROCESS(%d, %d)\n", vertex, frame->index, frame->id);
#endif

p_err:
	return ret;
}

static void __score_vertexmgr_sched(struct score_vertexmgr *vertexmgr)
{
	int ret = 0;
	unsigned long flags;
	struct score_vertex *vertex;
	struct score_frame *frame;
	struct score_gframe *ready, *request, *process, *prev, *next, *temp;
	struct score_interface *interface;
	struct score_resource *resource;

	interface = vertexmgr->interface;
	resource = vertexmgr->resource;
	ready = NULL;
	request = NULL;
	prev = NULL;
	next = NULL;

	VERTEX_CHECK_POINT(20);
	spin_lock_irqsave(&vertexmgr->slock, flags);

	/* 1. priority order */
	while (1) {
		__score_gframe_g_ready(vertexmgr, &ready);
		if (!ready)
			break;

		list_for_each_entry_safe(next, temp, &vertexmgr->gframe_request_list, list) {
			if (ready->priority > next->priority)
				break;

			prev = next;
			next = NULL;
		}

		__score_gframe_s_request(vertexmgr, prev, ready, next);
	}

	VERTEX_CHECK_POINT(21);
	list_for_each_entry_safe(request, temp, &vertexmgr->gframe_request_list, list) {
		vertex = request->vertex;
		frame = request->frame;

		ret = CALL_GOPS(vertex, get_resource, frame);
		if (ret)
			continue;

		__score_gframe_trans_req_to_pro(vertexmgr, request);
	}

	spin_unlock_irqrestore(&vertexmgr->slock, flags);
	VERTEX_CHECK_POINT(22);

	list_for_each_entry_safe(process, temp, &vertexmgr->gframe_process_list, list) {
		vertex = process->vertex;
		frame = process->frame;

		ret = __score_itf_allocate(interface, vertex);
		if (ret) {
			score_err("__score_itf_allocate is fail(%d)\n", ret);

			ret = CALL_GOPS(vertex, cancel, frame);
			if (ret) {
				score_err("CALL_GOPS(cancel) is fail(%d)\n", ret);
				BUG();
			}

			ret = CALL_GOPS(vertex, put_resource, frame);
			if (ret) {
				score_err("CALL_GOPS(put_resource) is fail(%d)\n", ret);
				BUG();
			}

			process->vertex = NULL;
			process->frame = NULL;
			__score_gframe_trans_pro_to_fre(vertexmgr, process);
			continue;
		}

		ret = __score_itf_process(interface, vertex, frame);
		if (ret) {
			score_err("__score_itf_process is fail(%d)\n", ret);

			ret = CALL_GOPS(vertex, cancel, frame);
			if (ret) {
				score_err("CALL_GOPS(cancel) is fail(%d)\n", ret);
				BUG();
			}

			ret = CALL_GOPS(vertex, put_resource, frame);
			if (ret) {
				score_err("CALL_GOPS(put_resource) is fail(%d)\n", ret);
				BUG();
			}

			process->vertex = NULL;
			process->frame = NULL;
			__score_gframe_trans_pro_to_fre(vertexmgr, process);
			continue;
		}

		__score_gframe_trans_pro_to_com(vertexmgr, process);
	}

	VERTEX_CHECK_POINT(23);
	vertexmgr->sched_cnt++;
}
#endif

#if 0
static int score_time_thread(void *data)
{
	struct score_vertexmgr *vertexmgr = (struct score_vertexmgr *)data;
#if 0
	struct score_gframe *request, *temp1, *temp2;
	struct score_gframe *next, *prev;
	struct score_vertex *vertex;
#endif
	BUG_ON(!vertexmgr);

	while (!kthread_should_stop()) {
#if 0
		TIMER_CHECK_POINT(1);
		msleep(SCORE_TIME_TICK);

		if (!test_bit(SCORE_VERTEXMGR_OPEN, &vertexmgr->state))
			return 0;

		if (!atomic_read(&vertexmgr->periodics))
			goto p_noperiodic;

	spin_lock(&vertexmgr->slock);

		TIMER_CHECK_POINT(2);
		list_for_each_entry_safe(request, temp1, &vertexmgr->gframe_request_list, list) {
			vertex = request->vertex;

			if (!test_bit(SCORE_VERTEX_STATE_START, &vertex->state)) {
				score_err("vertex %d is NOT start\n", vertex->id);
				BUG();
			}

			if (test_bit(VS4L_VERTEX_FLAG_PERIODIC, &vertex->flags)) {
				if (--request->ticks > 0)
					continue;

				TIMER_CHECK_POINT(3);
				score_iinfo("priority changed(%d -> %d)\n", vertex,
					request->priority, SCORE_VERTEX_MAX_PRIORITY + 1);
				request->priority = SCORE_VERTEX_MAX_PRIORITY + 1;
				request->ticks = vertex->period_ticks;

				list_del(&request->list);
				vertexmgr->gframe_req_cnt--;

				next = NULL;
				prev = NULL;
				list_for_each_entry_safe(next, temp2, &vertexmgr->gframe_request_list, list) {
					if (request->priority > next->priority)
						break;

					prev = next;
					next = NULL;
				}

				TIMER_CHECK_POINT(4);
				__score_gframe_s_request(vertexmgr, prev, request, next);
			}
		}

	spin_unlock(&vertexmgr->slock);

p_noperiodic:
		TIMER_CHECK_POINT(5);
		vertexmgr->tick_cnt++;
#endif
	}

	return 0;
}
#endif

void m_dumpBuffer(struct score_frame *frame)
{
	unsigned int loop = 0;
	unsigned int packet_size = 100;
	unsigned char *dumpData = NULL;

	struct vb_container_list *incl = NULL;
	struct vb_container *containers = NULL;
	struct vb_buffer *buffers = NULL;
	struct vb_buffer buffer;

	incl = frame->incl;
	containers = incl->containers;
	buffers = containers->buffers;
	buffer = buffers[0];

	dumpData = (unsigned char*)buffer.m.userptr;

	for (loop = 0; loop < packet_size; loop ++) {
		score_event_msg("{%02X }", (unsigned char)(dumpData[loop]));
		if ((loop % 8) == 3)
			score_event_msg("\n");
	}
	score_event_msg("____END___ \n");

	return;
}

inline int sc_cmd_get(void **param, unsigned char *pkt, unsigned int pos, unsigned int size)
{
	/* (*param) = (void*)(cmd->param + cmd->pos); */
	(*param) = (void*)(pkt + pos);
	/* cmd->pos += size; */
	return 0;
}

#define fw_command_get(X, CMD, POS, SIZE) \
	sc_cmd_get((void**)(X), (unsigned char*)(CMD), (POS), (SIZE))

#define SCV_BASE_PIXEL_SIZE 8
unsigned int calsBufferSize(struct sc_buffer *buffer)
{
	unsigned int buffer_size = 0;
	unsigned int pixel_size = 0;
	struct data_buf_type *temp_type;
	temp_type = (struct data_buf_type *)&buffer->type;

	pixel_size += temp_type->plane0 / SCV_BASE_PIXEL_SIZE;
	pixel_size += temp_type->plane1 / SCV_BASE_PIXEL_SIZE;
	pixel_size += temp_type->plane2 / SCV_BASE_PIXEL_SIZE;
	pixel_size += temp_type->plane3 / SCV_BASE_PIXEL_SIZE;

	buffer_size = buffer->width * buffer->height * pixel_size;
	score_event_msg(" calsBufferSize(%d) \n", buffer_size);

	return buffer_size;
}

void change_buffer_addr(struct score_frame *frame, struct score_ipc_packet *packet)
{
	int ret = 0;

	struct score_vertex_ctx *vctx;
	struct score_buftracker *buftracker;
	struct score_device *device;
	struct score_system *system;
	struct score_memory *memory;
	struct score_vertexmgr *vertexmgr;
	struct score_fw_dev *score_fw_device;

	unsigned int *size_word = NULL;
	unsigned int *header_word = NULL;

	unsigned int group_count = 0;
	struct score_packet_group *group = NULL;
	unsigned int loop, loop_fd_bitmap = 0;

	vctx = frame->owner;
	buftracker = &vctx->buftracker;
	vertexmgr = vctx->cookie;

	device = container_of(vertexmgr, struct score_device, vertexmgr);
	system = &device->system;
	memory = &system->memory;
	score_fw_device = &device->fw_dev;

	BUG_ON(!vctx);
	BUG_ON(!vertexmgr);
	BUG_ON(!system);
	BUG_ON(!device);
	BUG_ON(!memory);
	BUG_ON(!score_fw_device);

	size_word   = (unsigned int *)&packet->size;
	header_word = (unsigned int *)&packet->header;

	group_count = packet->size.group_count;
	group = packet->group;
/*
	score_event_msg("size.size(%d), reserved(%d), group_count(%d)\n",
	        packet->size.packet_size,
		packet->size.reserved,
	        packet->size.group_count);

	score_event_msg("header.queue_id(%d) kernel_name(%d) task_id(%d), reserved(%d) worker_name(%d) \n",
	        packet->header.queue_id,
		packet->header.kernel_name,
	        packet->header.task_id,
		packet->header.reserved,
	        packet->header.worker_name);

	score_event_msg("group_header.valid_is(%d) fd_bitmap(%d) \n",
	        packet->group[0].header.valid_size,
		packet->group[0].header.fd_bitmap);
*/
	/* score_fw_queue_dump_packet_word2(packet); */

	for (loop = 0; loop < packet->size.group_count; loop ++) {
		struct score_packet_group_header *packet_group_header = NULL;
		struct score_packet_group_data *packet_group_data = NULL;

		struct score_packet_group *packet_group = &packet->group[loop];
		packet_group_header = &packet_group->header;
		packet_group_data = &packet_group->data;

		score_event_msg("group_header.valid_is(%d) fd_bitmap(%d) \n",
			        packet_group_header->valid_size,
			        packet_group_header->fd_bitmap);

		for (loop_fd_bitmap = 0; loop_fd_bitmap < 26; loop_fd_bitmap ++) {
			if ((packet_group_header->fd_bitmap & (0x1 << loop_fd_bitmap)) != 0x0) {
				struct score_memory_buffer driver_buffer;
				struct sc_packet_buffer *fw_buffer = NULL;
				memset(&driver_buffer, 0x0, sizeof(struct score_memory_buffer));

				driver_buffer.kvaddr = NULL;

				score_event_msg("FdBitmap(%d) \n",
						loop_fd_bitmap);

				fw_command_get(&fw_buffer, packet_group_data,
						loop_fd_bitmap * sizeof(unsigned int),
						sizeof(struct sc_buffer));
				driver_buffer.memory = fw_buffer->host_buf.memory_type;

				switch (driver_buffer.memory) {
				case VS4L_MEMORY_DMABUF:

					score_event_msg("fw_buffer fd(%d) size(%d x %d) memory_type(0x%x) \n",
							fw_buffer->host_buf.fd,
							fw_buffer->buf.width,
							fw_buffer->buf.height,
							fw_buffer->host_buf.memory_type);

					driver_buffer.m.fd = fw_buffer->host_buf.fd;
					driver_buffer.size = calsBufferSize(&fw_buffer->buf);

					ret = score_buftracker_add(buftracker, &driver_buffer);
					if (ret) {
						score_err("score_buftracker_add is fail (%d) \n", ret);

						goto p_err;
					}
#ifdef BUF_MAP_KVADDR
#if 0
					memset(driver_buffer.kvaddr, 'A', driver_buffer.size);
#endif
#if 0
					print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
						driver_buffer.kvaddr, 0x200, false);
#endif
#endif

					/* HACK */
					/* fw_buffer->memory.addr = driver_buffer.dvaddr; */
					fw_buffer->host_buf.fd = (unsigned int)driver_buffer.dvaddr;
					fw_buffer->buf.addr = (unsigned int)driver_buffer.dvaddr;

					/* score_event_msg("[%s(%d)] addr(0x%lx) dvaddr(0x%llx) \n", */
					score_event_msg("addr(0x%x) dvaddr(0x%llx) \n",
							fw_buffer->buf.addr,
							driver_buffer.dvaddr);
					break;
				case VS4L_MEMORY_USERPTR:
					driver_buffer.m.userptr = fw_buffer->host_buf.addr32;
					driver_buffer.size = calsBufferSize(&fw_buffer->buf);

					ret = score_buftracker_map_userptr(buftracker, &driver_buffer);
					if (ret) {
						score_err("score_buftracker_add is fail (%d) \n", ret);

						goto p_err;
					}

					fw_buffer->host_buf.fd = (unsigned int)driver_buffer.dvaddr;
					fw_buffer->buf.addr = (unsigned int)driver_buffer.dvaddr;

					score_memory_invalid_or_flush_userptr(&driver_buffer,
							SCORE_MEMORY_SYNC_FOR_DEVICE,
							DMA_TO_DEVICE);

					break;
				default:
					pr_err("ERROR \n");
					break;
				}
			}
		}
	}

p_err:

	return;
}

void change_fd_addr(struct score_frame *frame)
{
	struct vb_container_list *incl = NULL;
	struct vb_container_list *otcl = NULL;
	struct vb_container *containers = NULL;
	struct vb_buffer *buffers = NULL;
	struct vb_buffer buffer;

	struct score_vertex_ctx *vctx;
	struct score_device *device;
	struct score_system *system;
	struct score_vertexmgr *vertexmgr;
	struct score_fw_dev *score_fw_device;
	struct score_ipc_packet *request_packet = NULL;
	struct score_ipc_packet *result_packet = NULL;

	vctx = frame->owner;
	vertexmgr = vctx->cookie;

	device = container_of(vertexmgr, struct score_device, vertexmgr);
	system = &device->system;
	score_fw_device = &device->fw_dev;

	incl = frame->incl;
	containers = incl->containers;
	buffers = containers->buffers;
	buffer = buffers[0];
	request_packet = (struct score_ipc_packet *)buffer.m.userptr;

	otcl = frame->otcl;
	containers = otcl->containers;
	buffers = containers->buffers;
	buffer = buffers[0];
	result_packet = (struct score_ipc_packet *)buffer.m.userptr;

	change_buffer_addr(frame, request_packet);

	return;
}

void send_request(struct score_frame *frame)
{
	struct vb_container_list *incl = NULL;
	struct vb_container_list *otcl = NULL;
	struct vb_container *containers = NULL;
	struct vb_buffer *buffers = NULL;
	struct vb_buffer buffer;

	struct score_vertex_ctx *vctx;
	struct score_device *device;
	struct score_system *system;
	struct score_vertexmgr *vertexmgr;
	struct score_fw_dev *score_fw_device;
	struct score_ipc_packet *request_packet = NULL;
	struct score_ipc_packet *result_packet = NULL;

	vctx = frame->owner;
	vertexmgr = vctx->cookie;

	device = container_of(vertexmgr, struct score_device, vertexmgr);
	system = &device->system;
	score_fw_device = &device->fw_dev;

	incl = frame->incl;
	containers = incl->containers;
	buffers = containers->buffers;
	buffer = buffers[0];
	request_packet = (struct score_ipc_packet *)buffer.m.userptr;

	otcl = frame->otcl;
	containers = otcl->containers;
	buffers = containers->buffers;
	buffer = buffers[0];
	result_packet = (struct score_ipc_packet *)buffer.m.userptr;

	/* score_fw_queue_dump_packet_word2(request_packet); */
	/* score_fw_queue_dump_packet_word2(result_packet); */

	score_event_msg("receive command [task_id:%d]\n", request_packet->header.task_id);
	score_fw_queue_put(score_fw_device->in_queue, request_packet);

	return;
}

static void score_vertex_thread(struct kthread_work *work)
{
	unsigned long flags;
	struct score_vertex_ctx *vctx;
	struct score_vertexmgr *vertexmgr;
	struct score_framemgr *framemgr;
	struct score_frame *frame;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;
	struct score_interface *interface;
	struct score_gframe *gframe;

	BUG_ON(!work);

	frame = container_of(work, struct score_frame, work);
	vctx = frame->owner;
	vertexmgr = vctx->cookie;
	interface = vertexmgr->interface;
	framemgr = &interface->framemgr;

	/* VERTEX_CHECK_POINT(1); */
	score_event_msg("msg(%d) \n", frame->message);

	switch (frame->message) {
	case SCORE_FRAME_REQUEST:

		iframemgr = &interface->framemgr;
		framemgr_e_barrier_irqs(iframemgr, 0, flags);
		score_frame_pick_fre_to_req(iframemgr, &iframe);
		framemgr_x_barrier_irqr(iframemgr, 0, flags);

		iframe->message = SCORE_FRAME_PROCESS;
		iframe->incl = frame->incl;
		iframe->otcl = frame->otcl;
		framemgr_e_barrier_irqs(iframemgr, 0, flags);
		score_frame_trans_req_to_pro(iframemgr, iframe);
		framemgr_x_barrier_irqr(iframemgr, 0, flags);

		__score_gframe_g_free(vertexmgr, &gframe);
		gframe->frame = frame;
		frame->gindex = gframe->index;
		iframe->gindex = gframe->index;
		__score_gframe_s_process(vertexmgr, gframe);

		send_request(frame);
#if 0
		VERTEX_CHECK_POINT(2);
		ret = CALL_GOPS(vertex, request, frame);
		if (ret) {
			score_err("CALL_GOPS(request) is fail(%d)\n", ret);
			BUG();
		}

		__score_gframe_free_head(vertexmgr, &gframe);
		if (!gframe) {
			score_err("gframe is NULL\n");
			BUG();
		}

		frame->gindex = gframe->index;
		gframe->vertex = vertex;
		gframe->frame = frame;
		gframe->priority = vertex->priority;
		gframe->ticks = vertex->period_ticks;
		__score_gframe_trans_fre_to_rdy(vertexmgr, gframe);
		VERTEX_CHECK_POINT(3);
#endif
		break;
#if 0
	case SCORE_CTRL_STOP:
		VERTEX_CHECK_POINT(4);
		list_for_each_entry_safe(gframe, temp, &vertexmgr->gframe_ready_list, list) {
			if (gframe->vertex->id != vertex->id)
				continue;

			ret = CALL_GOPS(vertex, cancel, gframe->frame);
			if (ret) {
				score_err("CALL_GOPS(cancel) is fail(%d)\n", ret);
				BUG();
			}

			gframe->vertex = NULL;
			gframe->frame = NULL;
			__score_gframe_trans_rdy_to_fre(vertexmgr, gframe);
		}

		VERTEX_CHECK_POINT(5);
	spin_lock_irqsave(&vertexmgr->slock, flags);

		list_for_each_entry_safe(gframe, temp, &vertexmgr->gframe_request_list, list) {
			if (gframe->vertex->id != vertex->id)
				continue;

			ret = CALL_GOPS(vertex, cancel, gframe->frame);
			if (ret) {
				score_err("CALL_GOPS(cancel) is fail(%d)\n", ret);
				BUG();
			}

			ret = CALL_GOPS(vertex, put_resource, gframe->frame);
			if (ret) {
				score_err("CALL_GOPS(put_resource) is fail(%d)\n", ret);
				BUG();
			}

			gframe->vertex = NULL;
			gframe->frame = NULL;
			__score_gframe_trans_req_to_fre(vertexmgr, gframe);
		}

	spin_unlock_irqrestore(&vertexmgr->slock, flags);
		VERTEX_CHECK_POINT(6);

		ret = CALL_GOPS(vertex, control, frame);
		if (ret) {
			score_err("CALL_GOPS(control) is fail(%d)\n", ret);
			BUG();
		}
		return;
#endif
	default:
		BUG();
		break;
	}

	/* VERTEX_CHECK_POINT(7); */
	/* score_vertexmgr_sched(vertexmgr); */
}

static void score_interface_thread(struct kthread_work *work)
{
	int ret = 0;
	unsigned long flags;
	struct score_framemgr *framemgr;
	struct score_framemgr *iframemgr;
	struct score_frame *iframe;
	struct score_interface *interface;


	struct vb_container_list *incl = NULL;
	struct vb_container_list *otcl = NULL;
	struct vb_container *containers = NULL;
	struct vb_buffer *buffers = NULL;
	struct vb_buffer buffer;

	struct score_device *device;
	struct score_system *system;
	struct score_vertexmgr *vertexmgr;
	struct score_fw_dev *score_fw_device;
	struct score_ipc_packet *request_packet = NULL;
	struct score_ipc_packet *result_packet = NULL;

	struct score_vertex *vertex;
	struct score_vertex_ctx *vctx;
	struct score_gframe *gframe;
	struct score_frame *frame;
	struct score_buftracker *buftracker;

	u32 gframe_index;

	BUG_ON(!work);

	iframe = container_of(work, struct score_frame, work);
	interface = iframe->owner;
	iframemgr = &interface->framemgr;
	vertexmgr = interface->cookie;

	device = container_of(vertexmgr, struct score_device, vertexmgr);
	system = &device->system;
	score_fw_device = &device->fw_dev;

	incl = iframe->incl;
	containers = incl->containers;
	buffers = containers->buffers;
	buffer = buffers[0];
	request_packet = (struct score_ipc_packet *)buffer.m.userptr;

	otcl = iframe->otcl;
	containers = otcl->containers;
	buffers = containers->buffers;
	buffer = buffers[0];
	result_packet = (struct score_ipc_packet *)buffer.m.userptr;

	/* VERTEX_CHECK_POINT(10); */

	score_event_msg("msg(%d) \n", iframe->message);

	switch (iframe->message) {
	case SCORE_FRAME_DONE:
	case SCORE_FRAME_NDONE:
		ret = score_fw_queue_get(score_fw_device->out_queue, result_packet);

		/* HACK */
		gframe_index = iframe->gindex;
		gframe = &vertexmgr->gframe[gframe_index];
		frame = gframe->frame;
		vctx = frame->owner;
		vertex = vctx->vertex;
		framemgr = &vctx->framemgr;
		buftracker = &vctx->buftracker;

		frame->message = iframe->message;
		frame->ret = ret;
		score_event_msg("frame->ret(%d) \n", frame->ret);
		score_info("frame->ret(%d) \n", frame->ret);

		/* HACK */
		framemgr_e_barrier_irqs(framemgr, 0, flags);
		score_frame_trans_pro_to_com(framemgr, frame);
		framemgr_x_barrier_irqr(framemgr, 0, flags);

		framemgr_e_barrier_irqs(framemgr, 0, flags);
		score_frame_trans_com_to_fre(framemgr, frame);
		framemgr_x_barrier_irqr(framemgr, 0, flags);

		framemgr_e_barrier_irqs(iframemgr, 0, flags);
		score_frame_trans_com_to_fre(iframemgr, iframe);
		framemgr_x_barrier_irqr(iframemgr, 0, flags);

		ret = score_buftracker_invalid_or_flush_userptr_all(buftracker,
				SCORE_MEMORY_SYNC_FOR_CPU,
				DMA_FROM_DEVICE);
		if (ret)
			score_err("invalid_or_flush is fail(%d)\n", ret);

		ret = score_buftracker_remove_userptr_all(buftracker);
		if (ret)
			score_err("remove_usertpr_all is fail(%d)\n", ret);

		ret = CALL_ROPS(vertex, done, frame);
		if (ret) {
			score_err("CALL_GOPS(done) is fail(%d)\n", ret);
			BUG();
		}
#if 0
		VERTEX_CHECK_POINT(11);
		frame_index = iframe->findex;
		gframe_index = iframe->gindex;

		if (gframe_index >= SCORE_MAX_GFRAME) {
			score_err("gframe index(%d) is invalid\n", gframe_index);
			BUG();
		}

		if (frame_index >= SCORE_MAX_FRAME) {
			score_err("frame index(%d) is invalid\n", frame_index);
			BUG();
		}

		gframe = &vertexmgr->gframe[gframe_index];
		if (gframe->state != SCORE_GFRAME_STATE_COMPLETE) {
			score_err("gframe state is invalid(%d)\n", gframe->state);
			BUG();
		}

		vertex = gframe->vertex;
		if (!vertex) {
			score_err("vertex is NULL(%d)\n", gframe_index);
			BUG();
		}

		frame = gframe->frame;
		if (!frame) {
			score_err("frame is NULL(%d)\n", gframe_index);
			BUG();
		}

		frame->message = iframe->message;
		frame->gindex = SCORE_MAX_GFRAME;

		ret = CALL_GOPS(vertex, put_resource, frame);
		if (ret) {
			score_err("CALL_GOPS(put_resource) is fail(%d)\n", ret);
			BUG();
		}

		ret = CALL_GOPS(vertex, done, frame);
		if (ret) {
			score_err("CALL_GOPS(done) is fail(%d)\n", ret);
			BUG();
		}
#endif
		gframe->vertex = NULL;
		gframe->frame = NULL;
		gframe->ticks = 0;
		__score_gframe_trans_pro_to_com(vertexmgr, gframe);
		__score_gframe_trans_com_to_fre(vertexmgr, gframe);

		break;
	default:
		BUG();
		break;
	}
#if 0
	VERTEX_CHECK_POINT(12);
	__score_vertexmgr_sched(vertexmgr);
#endif
	return;
}

int score_vertexmgr_grp_register(struct score_vertexmgr *vertexmgr, struct score_vertex_ctx *vctx)
{
	int ret = 0;
	u32 index;

	SCORE_TP();
	BUG_ON(!vertexmgr);
	BUG_ON(!vctx);

	mutex_lock(&vertexmgr->mlock);
	for (index = 0; index < SCORE_MAX_VERTEX; index++) {
		if (!vertexmgr->vctx[index]) {
			vertexmgr->vctx[index] = vctx;
			vctx->id = index;
			break;
		}
	}
	mutex_unlock(&vertexmgr->mlock);

	if (index >= SCORE_MAX_VERTEX) {
		score_err("vertex slot is lack\n");
		ret = -EINVAL;
		goto p_err;
	}

	init_kthread_work(&vctx->control.work, score_vertex_thread);
	for (index = 0; index < SCORE_MAX_FRAME; ++index)
		init_kthread_work(&vctx->framemgr.frame[index].work, score_vertex_thread);

p_err:
	return ret;
}

int score_vertexmgr_grp_unregister(struct score_vertexmgr *vertexmgr, struct score_vertex_ctx *vctx)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!vertexmgr);
	BUG_ON(!vctx);

	mutex_lock(&vertexmgr->mlock);
	vertexmgr->vctx[vctx->id] = NULL;
	mutex_unlock(&vertexmgr->mlock);

	return ret;
}

int score_vertexmgr_grp_start(struct score_vertexmgr *vertexmgr, struct score_vertex_ctx *vctx)
{
	int ret = 0;

	SCORE_TP();
	/*
	* if (test_bit(VS4L_VERTEX_FLAG_PERIODIC, &vertex->flags))
	*	atomic_inc(&vertexmgr->periodics);

	* ret = __score_itf_enum(vertexmgr->interface, vertex);
	* if (ret) {
	*	score_err("__score_itf_enum is fail(%d)\n", ret);
	*	goto p_err;
	* }
	*/

	/* ret = __score_itf_create(vertexmgr->interface, vertex); */
	if (ret) {
		score_err("__score_itf_create is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_vertexmgr_grp_stop(struct score_vertexmgr *vertexmgr, struct score_vertex_ctx *vertex)
{
	int ret = 0;

	/*
	* if (test_bit(VS4L_VERTEX_FLAG_PERIODIC, &vertex->flags))
	*	atomic_dec(&vertexmgr->periodics);
	*/

	/* ret = __score_itf_destroy(vertexmgr->interface, vertex); */
	if (ret) {
		score_err("__score_itf_destroy is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_vertexmgr_itf_register(struct score_vertexmgr *vertexmgr, struct score_interface *interface)
{
	int ret = 0;
	u32 index;

	SCORE_TP();
	BUG_ON(!vertexmgr);
	BUG_ON(!interface);

	vertexmgr->interface = interface;

	for (index = 0; index < SCORE_MAX_FRAME; ++index)
		init_kthread_work(&interface->framemgr.frame[index].work, score_interface_thread);

	return ret;
}

int score_vertexmgr_itf_unregister(struct score_vertexmgr *vertexmgr, struct score_interface *interface)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!vertexmgr);
	BUG_ON(!interface);

	vertexmgr->interface = NULL;

	return ret;
}

void score_vertexmgr_queue(struct score_vertexmgr *vertexmgr, struct score_frame *frame)
{
#ifdef DBG_PER_FRAME_LOG
	SCORE_TP();
#endif
	BUG_ON(!vertexmgr);
	BUG_ON(!frame);

	change_fd_addr(frame);
#ifdef DBG_UNUSE_THREAD
	do {
		struct score_device *device;
		struct score_system *system;

		device = container_of(vertexmgr, struct score_device, vertexmgr);
		system = &device->system;

		mdelay(500);
		score_fw_dump_regs(system->regs + 0x7000, 56 * 4);

		score_event_msg("505c(0x%x)55(0x%x)56(0x%x)57(0x%x)RET(0x%x) \n",
			readl(system->regs + 0x505c),
			readl(system->regs + SCORE_PARAM55),
			readl(system->regs + SCORE_PARAM56),
			readl(system->regs + SCORE_PARAM57),
			readl(system->regs + SCORE_VERIFY_RESULT));
		mdelay(500);
		score_fw_dump_regs(system->regs + 0x7000, 56 * 4);

		score_event_msg("505c(0x%x)55(0x%x)56(0x%x)57(0x%x)RET(0x%x) \n",
			readl(system->regs + 0x505c),
			readl(system->regs + SCORE_PARAM55),
			readl(system->regs + SCORE_PARAM56),
			readl(system->regs + SCORE_PARAM57),
			readl(system->regs + SCORE_VERIFY_RESULT));
	} while (0);
#else
	queue_kthread_work(&vertexmgr->worker, &frame->work);
#endif
}

int score_vertexmgr_probe(struct score_vertexmgr *vertexmgr)
{
	int ret = 0;
	u32 index;
	struct score_device *device = container_of(vertexmgr, struct score_device, vertexmgr);

	SCORE_TP();
	BUG_ON(!vertexmgr);
	BUG_ON(!device);

	vertexmgr->tick_cnt = 0;
	vertexmgr->tick_pos = 0;
	vertexmgr->sched_cnt = 0;
	vertexmgr->sched_pos = 0;
	vertexmgr->task_vertex = NULL;
	/* vertexmgr->task_timer = NULL; */
	/* vertexmgr->resource = resource; */
	atomic_set(&vertexmgr->periodics, 0);
	mutex_init(&vertexmgr->mlock);
	spin_lock_init(&vertexmgr->slock);
	clear_bit(SCORE_VERTEXMGR_OPEN, &vertexmgr->state);

	for (index = 0; index < SCORE_MAX_VERTEX; ++index)
		vertexmgr->vctx[index] = NULL;

	INIT_LIST_HEAD(&vertexmgr->gframe_free_list);
	INIT_LIST_HEAD(&vertexmgr->gframe_ready_list);
	INIT_LIST_HEAD(&vertexmgr->gframe_request_list);
	INIT_LIST_HEAD(&vertexmgr->gframe_process_list);
	INIT_LIST_HEAD(&vertexmgr->gframe_complete_list);

	vertexmgr->gframe_fre_cnt = 0;
	vertexmgr->gframe_rdy_cnt = 0;
	vertexmgr->gframe_req_cnt = 0;
	vertexmgr->gframe_pro_cnt = 0;
	vertexmgr->gframe_com_cnt = 0;

	for (index = 0; index < SCORE_MAX_GFRAME; ++index) {
		vertexmgr->gframe[index].index = index;
		vertexmgr->gframe[index].vertex = NULL;
		vertexmgr->gframe[index].frame = NULL;
		vertexmgr->gframe[index].state = SCORE_GFRAME_STATE_INVALID;
		vertexmgr->gframe[index].ticks = 0;
		__score_gframe_s_free(vertexmgr, &vertexmgr->gframe[index]);
	}

	return ret;
}

int score_vertexmgr_open(struct score_vertexmgr *vertexmgr)
{
	int ret = 0;
	char name[30];
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	SCORE_TP();
	BUG_ON(!vertexmgr);

	init_kthread_worker(&vertexmgr->worker);
	snprintf(name, sizeof(name), "score_vertex");
	vertexmgr->task_vertex = kthread_run(kthread_worker_fn, &vertexmgr->worker, name);
	if (IS_ERR_OR_NULL(vertexmgr->task_vertex)) {
		score_err("kthread_run is fail\n");
		ret = -EINVAL;
		goto p_err;
	}

	SCORE_TP();
	ret = sched_setscheduler_nocheck(vertexmgr->task_vertex, SCHED_FIFO, &param);
	if (ret) {
		score_err("sched_setscheduler_nocheck is fail(%d)\n", ret);
		goto p_err;
	}

#if 0
	SCORE_TP();
	snprintf(name, sizeof(name), "score_timer");
	vertexmgr->task_timer = kthread_run(score_time_thread, vertexmgr, name);
	if (IS_ERR_OR_NULL(vertexmgr->task_timer)) {
		score_err("kthread_run is fail\n");
		ret = -EINVAL;
		goto p_err;
	}

	SCORE_TP();
	ret = sched_setscheduler_nocheck(vertexmgr->task_timer, SCHED_FIFO, &param);
	if (ret) {
		score_err("sched_setscheduler_nocheck is fail(%d)\n", ret);
		goto p_err;
	}
#endif
	SCORE_TP();
	set_bit(SCORE_VERTEXMGR_OPEN, &vertexmgr->state);

p_err:
	return ret;
}

int score_vertexmgr_close(struct score_vertexmgr *vertexmgr)
{
	int ret = 0;
	struct score_gframe *gframe, *temp;

	SCORE_TP();
	BUG_ON(!vertexmgr);

	if (vertexmgr->gframe_rdy_cnt > 0) {
		ret++;
		score_err("ready gframe is NOT empty(%d)\n", vertexmgr->gframe_rdy_cnt);

		list_for_each_entry_safe(gframe, temp, &vertexmgr->gframe_ready_list, list) {
			list_del(&gframe->list);
			vertexmgr->gframe_rdy_cnt--;
			__score_gframe_s_free(vertexmgr, gframe);
		}
	}

	if (vertexmgr->gframe_req_cnt > 0) {
		ret++;
		score_err("request gframe is NOT empty(%d)\n", vertexmgr->gframe_req_cnt);

		list_for_each_entry_safe(gframe, temp, &vertexmgr->gframe_request_list, list) {
			list_del(&gframe->list);
			vertexmgr->gframe_req_cnt--;
			__score_gframe_s_free(vertexmgr, gframe);
		}
	}

	if (vertexmgr->gframe_pro_cnt > 0) {
		ret++;
		score_err("process gframe is NOT empty(%d)\n", vertexmgr->gframe_pro_cnt);

		list_for_each_entry_safe(gframe, temp, &vertexmgr->gframe_process_list, list) {
			list_del(&gframe->list);
			vertexmgr->gframe_pro_cnt--;
			__score_gframe_s_free(vertexmgr, gframe);
		}
	}

	/* kthread_stop(vertexmgr->task_timer); */
	kthread_stop(vertexmgr->task_vertex);

	clear_bit(SCORE_VERTEXMGR_OPEN, &vertexmgr->state);

	return ret;
}
