/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "score-debug.h"
#include "score-framemgr.h"

void score_frame_s_free(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

#ifdef DBG_FRAME_STATE
#ifdef DBG_FRAME_STATE_CALLSTACK
	WARN_ON(1);
#endif
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif

	frame->state = SCORE_FRAME_STATE_FREE;

	list_add_tail(&frame->list, &framemgr->frame_free_list);
	framemgr->frame_fre_cnt++;

#ifdef DBG_FRAME_STATE
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif
}

void score_frame_g_free(struct score_framemgr *framemgr, struct score_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->frame_fre_cnt &&
		(*frame = container_of(framemgr->frame_free_list.next, struct score_frame, list))) {
		list_del(&(*frame)->list);
		framemgr->frame_fre_cnt--;
		(*frame)->state = SCORE_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void score_frame_free_head(struct score_framemgr *framemgr, struct score_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->frame_fre_cnt)
		*frame = container_of(framemgr->frame_free_list.next, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_free_tail(struct score_framemgr *framemgr, struct score_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->frame_fre_cnt)
		*frame = container_of(framemgr->frame_free_list.prev, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_print_free_list(struct score_framemgr *framemgr)
{
	/* DLOG_INIT(); */
	struct score_frame *frame, *temp;

	/* DLOG("[FRM] fre(%d, %d) :", framemgr->id, framemgr->frame_fre_cnt); */
	score_info("[FRM] fre(%d, %d) :", framemgr->id, framemgr->frame_fre_cnt);
	list_for_each_entry_safe(frame, temp, &framemgr->frame_free_list, list) {
		/* DLOG("%d->", frame->index); */
		score_info("%d->", frame->index);
	}
	/* DLOG("X"); */
	score_info("X");

	/* score_info("%s\n", DLOG_OUT()); */
}

void score_frame_s_request(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

#ifdef DBG_FRAME_STATE
#ifdef DBG_FRAME_STATE_CALLSTACK
	WARN_ON(1);
#endif
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif

	frame->state = SCORE_FRAME_STATE_REQUEST;

	list_add_tail(&frame->list, &framemgr->frame_request_list);
	framemgr->frame_req_cnt++;

#ifdef DBG_FRAME_STATE
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif
}

void score_frame_g_request(struct score_framemgr *framemgr, struct score_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->frame_req_cnt &&
		(*frame = container_of(framemgr->frame_request_list.next, struct score_frame, list))) {
		list_del(&(*frame)->list);
		framemgr->frame_req_cnt--;
		(*frame)->state = SCORE_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void score_frame_request_head(struct score_framemgr *framemgr, struct score_frame **frame)
{
	if (framemgr->frame_req_cnt)
		*frame = container_of(framemgr->frame_request_list.next, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_request_tail(struct score_framemgr *framemgr, struct score_frame **frame)
{
	if (framemgr->frame_req_cnt)
		*frame = container_of(framemgr->frame_request_list.prev, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_print_request_list(struct score_framemgr *framemgr)
{
	/*
	* DLOG_INIT();
	* struct score_frame *frame, *temp;

	* DLOG("[FRM] req(%d, %d) :", framemgr->id, framemgr->frame_req_cnt);
	* list_for_each_entry_safe(frame, temp, &framemgr->frame_request_list, list) {
	*	DLOG("%d->", frame->index);
	* }
	* DLOG("X");

	* score_info("%s\n", DLOG_OUT());
	*/
}

void score_frame_s_prepare(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

#ifdef DBG_FRAME_STATE
#ifdef DBG_FRAME_STATE_CALLSTACK
	WARN_ON(1);
#endif
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif

	frame->state = SCORE_FRAME_STATE_PREPARE;

	list_add_tail(&frame->list, &framemgr->frame_prepare_list);
	framemgr->frame_pre_cnt++;

#ifdef DBG_FRAME_STATE
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif
}

void score_frame_g_prepare(struct score_framemgr *framemgr, struct score_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->frame_pre_cnt &&
		(*frame = container_of(framemgr->frame_prepare_list.next, struct score_frame, list))) {
		list_del(&(*frame)->list);
		framemgr->frame_pre_cnt--;
		(*frame)->state = SCORE_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void score_frame_prepare_head(struct score_framemgr *framemgr, struct score_frame **frame)
{
	if (framemgr->frame_pre_cnt)
		*frame = container_of(framemgr->frame_prepare_list.next, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_prepare_tail(struct score_framemgr *framemgr, struct score_frame **frame)
{
	if (framemgr->frame_pre_cnt)
		*frame = container_of(framemgr->frame_prepare_list.prev, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_print_prepare_list(struct score_framemgr *framemgr)
{
	/*
	* DLOG_INIT();
	* struct score_frame *frame, *temp;

	* DLOG("[FRM] pre(%d, %d) :", framemgr->id, framemgr->frame_pre_cnt);
	* list_for_each_entry_safe(frame, temp, &framemgr->frame_prepare_list, list) {
	*	DLOG("%d->", frame->index);
	* }
	* DLOG("X");

	* score_info("%s\n", DLOG_OUT());
	*/
}

void score_frame_s_process(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

#ifdef DBG_FRAME_STATE
#ifdef DBG_FRAME_STATE_CALLSTACK
	WARN_ON(1);
#endif
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif

	frame->state = SCORE_FRAME_STATE_PROCESS;

	list_add_tail(&frame->list, &framemgr->frame_process_list);
	framemgr->frame_pro_cnt++;

#ifdef DBG_FRAME_STATE
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif
}

void score_frame_g_process(struct score_framemgr *framemgr, struct score_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->frame_pro_cnt &&
		(*frame = container_of(framemgr->frame_process_list.next, struct score_frame, list))) {
		list_del(&(*frame)->list);
		framemgr->frame_pro_cnt--;
		(*frame)->state = SCORE_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void score_frame_process_head(struct score_framemgr *framemgr, struct score_frame **frame)
{
	if (framemgr->frame_pro_cnt)
		*frame = container_of(framemgr->frame_process_list.next, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_process_tail(struct score_framemgr *framemgr, struct score_frame **frame)
{
	if (framemgr->frame_pro_cnt)
		*frame = container_of(framemgr->frame_process_list.prev, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_print_process_list(struct score_framemgr *framemgr)
{
	/* DLOG_INIT();
	* struct score_frame *frame, *temp;

	* DLOG("[FRM] pro(%d, %d) :", framemgr->id, framemgr->frame_pro_cnt);
	* list_for_each_entry_safe(frame, temp, &framemgr->frame_process_list, list) {
	*	DLOG("%d->", frame->index);
	* }
	* DLOG("X");

	* score_info("%s\n", DLOG_OUT());
	*/
}

void score_frame_s_complete(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

#ifdef DBG_FRAME_STATE
#ifdef DBG_FRAME_STATE_CALLSTACK
	WARN_ON(1);
#endif
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif

	frame->state = SCORE_FRAME_STATE_COMPLETE;

	list_add_tail(&frame->list, &framemgr->frame_complete_list);
	framemgr->frame_com_cnt++;

#ifdef DBG_FRAME_STATE
	score_info("(%d) pro(%d) com(%d) \n",
		framemgr->id,
		framemgr->frame_pro_cnt,
		framemgr->frame_com_cnt);
#endif
}

void score_frame_g_complete(struct score_framemgr *framemgr, struct score_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	if (framemgr->frame_com_cnt &&
		(*frame = container_of(framemgr->frame_complete_list.next, struct score_frame, list))) {
		list_del(&(*frame)->list);
		framemgr->frame_com_cnt--;
		(*frame)->state = SCORE_FRAME_STATE_INVALID;
	} else {
		*frame = NULL;
	}
}

void score_frame_complete_head(struct score_framemgr *framemgr, struct score_frame **frame)
{
	if (framemgr->frame_com_cnt)
		*frame = container_of(framemgr->frame_complete_list.next, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_complete_tail(struct score_framemgr *framemgr, struct score_frame **frame)
{
	if (framemgr->frame_com_cnt)
		*frame = container_of(framemgr->frame_complete_list.prev, struct score_frame, list);
	else
		*frame = NULL;
}

void score_frame_print_complete_list(struct score_framemgr *framemgr)
{
	/*
	* DLOG_INIT();
	* struct score_frame *frame, *temp;

	* DLOG("[FRM] com(%d, %d) :", framemgr->id, framemgr->frame_com_cnt);
	* list_for_each_entry_safe(frame, temp, &framemgr->frame_complete_list, list) {
	*	DLOG("%d->", frame->index);
	* }
	* DLOG("X");

	* score_info("%s\n", DLOG_OUT());
	*/
}

void score_frame_trans_fre_to_req(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_fre_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_FREE);

	list_del(&frame->list);
	framemgr->frame_fre_cnt--;
	score_frame_s_request(framemgr, frame);
}

void score_frame_trans_req_to_pre(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_req_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_REQUEST);

	list_del(&frame->list);
	framemgr->frame_req_cnt--;
	score_frame_s_prepare(framemgr, frame);
}

void score_frame_trans_req_to_pro(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_req_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_REQUEST);

	list_del(&frame->list);
	framemgr->frame_req_cnt--;
	score_frame_s_process(framemgr, frame);
}

void score_frame_trans_req_to_com(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_req_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_REQUEST);

	list_del(&frame->list);
	framemgr->frame_req_cnt--;
	score_frame_s_complete(framemgr, frame);
}

void score_frame_trans_req_to_fre(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_req_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_REQUEST);

	list_del(&frame->list);
	framemgr->frame_req_cnt--;
	score_frame_s_free(framemgr, frame);
}

void score_frame_trans_pre_to_pro(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_pre_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_PREPARE);

	list_del(&frame->list);
	framemgr->frame_pre_cnt--;
	score_frame_s_process(framemgr, frame);
}

void score_frame_trans_pre_to_com(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_pre_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_PREPARE);

	list_del(&frame->list);
	framemgr->frame_pre_cnt--;
	score_frame_s_complete(framemgr, frame);
}

void score_frame_trans_pre_to_fre(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_pre_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_PREPARE);

	list_del(&frame->list);
	framemgr->frame_pre_cnt--;
	score_frame_s_free(framemgr, frame);
}

void score_frame_trans_pro_to_com(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_pro_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_PROCESS);

	list_del(&frame->list);
	framemgr->frame_pro_cnt--;
	score_frame_s_complete(framemgr, frame);
}

void score_frame_trans_pro_to_fre(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_pro_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_PROCESS);

	list_del(&frame->list);
	framemgr->frame_pro_cnt--;
	score_frame_s_free(framemgr, frame);
}

void score_frame_trans_com_to_fre(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);
	BUG_ON(!framemgr->frame_com_cnt);
	BUG_ON(frame->state != SCORE_FRAME_STATE_COMPLETE);

	list_del(&frame->list);
	framemgr->frame_com_cnt--;
	score_frame_s_free(framemgr, frame);
}

void score_frame_trans_any_to_fre(struct score_framemgr *framemgr, struct score_frame *frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	list_del(&frame->list);
	switch (frame->state) {
	case SCORE_FRAME_STATE_REQUEST:
		framemgr->frame_req_cnt--;
		break;
	case SCORE_FRAME_STATE_PREPARE:
		framemgr->frame_pre_cnt--;
		break;
	case SCORE_FRAME_STATE_PROCESS:
		framemgr->frame_pro_cnt--;
		break;
	case SCORE_FRAME_STATE_COMPLETE:
		framemgr->frame_com_cnt--;
		break;
	default:
		BUG();
		break;
	}

	score_frame_s_free(framemgr, frame);
}

void score_frame_pick_fre_to_req(struct score_framemgr *framemgr, struct score_frame **frame)
{
	BUG_ON(!framemgr);
	BUG_ON(!frame);

	score_frame_free_head(framemgr, frame);
	if (*frame)
		score_frame_trans_fre_to_req(framemgr, *frame);
}

void score_frame_print_all(struct score_framemgr *framemgr)
{
	BUG_ON(!framemgr);

	score_frame_print_free_list(framemgr);
	score_frame_print_request_list(framemgr);
	score_frame_print_prepare_list(framemgr);
	score_frame_print_process_list(framemgr);
	score_frame_print_complete_list(framemgr);
}

int score_frame_init(struct score_framemgr *framemgr, void *owner)
{
	int ret = 0;
	unsigned long flags;
	u32 index;

	spin_lock_irqsave(&framemgr->slock, flags);

	INIT_LIST_HEAD(&framemgr->frame_free_list);
	INIT_LIST_HEAD(&framemgr->frame_request_list);
	INIT_LIST_HEAD(&framemgr->frame_prepare_list);
	INIT_LIST_HEAD(&framemgr->frame_process_list);
	INIT_LIST_HEAD(&framemgr->frame_complete_list);

	framemgr->frame_cnt = SCORE_MAX_FRAME;
	framemgr->frame_fre_cnt = 0;
	framemgr->frame_req_cnt = 0;
	framemgr->frame_pre_cnt = 0;
	framemgr->frame_pro_cnt = 0;
	framemgr->frame_com_cnt = 0;

	for (index = 0; index < framemgr->frame_cnt; ++index) {
		framemgr->frame[index].index = index;
		framemgr->frame[index].owner = owner;
		framemgr->frame[index].findex = SCORE_MAX_FRAME;
		framemgr->frame[index].gindex = SCORE_MAX_GFRAME;
		framemgr->frame[index].ret = 0;
		score_frame_s_free(framemgr, &framemgr->frame[index]);
	}

	spin_unlock_irqrestore(&framemgr->slock, flags);

	return ret;
}

int score_frame_deinit(struct score_framemgr *framemgr)
{
	int ret = 0;
	unsigned long flags;
	u32 index;

	spin_lock_irqsave(&framemgr->slock, flags);

	for (index = 0; index < framemgr->frame_cnt; ++index)
		score_frame_s_free(framemgr, &framemgr->frame[index]);

	spin_unlock_irqrestore(&framemgr->slock, flags);

	return ret;
}

void score_frame_flush(struct score_framemgr *framemgr)
{
	unsigned long flag;
	struct score_frame *frame, *temp;

	BUG_ON(!framemgr);

	spin_lock_irqsave(&framemgr->slock, flag);

	list_for_each_entry_safe(frame, temp, &framemgr->frame_request_list, list) {
		score_frame_trans_req_to_fre(framemgr, frame);
		score_warn("req frame%d is flushed(count : %d)", frame->id, framemgr->frame_req_cnt);
	}

	list_for_each_entry_safe(frame, temp, &framemgr->frame_prepare_list, list) {
		score_frame_trans_pre_to_fre(framemgr, frame);
		score_warn("pre frame%d is flushed(count : %d)", frame->id, framemgr->frame_pre_cnt);
	}

	list_for_each_entry_safe(frame, temp, &framemgr->frame_process_list, list) {
		score_frame_trans_pro_to_fre(framemgr, frame);
		score_warn("pro frame%d is flushed(count : %d)", frame->id, framemgr->frame_pro_cnt);
	}

	list_for_each_entry_safe(frame, temp, &framemgr->frame_complete_list, list) {
		score_frame_trans_com_to_fre(framemgr, frame);
		score_warn("com frame%d is flushed(count : %d)", frame->id, framemgr->frame_com_cnt);
	}

	spin_unlock_irqrestore(&framemgr->slock, flag);

	BUG_ON(framemgr->frame_fre_cnt != framemgr->frame_cnt);
}
