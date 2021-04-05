/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef SCORE_FRAMEMGR_H_
#define SCORE_FRAMEMGR_H_

#include <linux/types.h>
#include <linux/kthread.h>

#include "score-config.h"
#include "score-time.h"
#include "vs4l.h"

#if 0
#define FMGR_IDX_0		(1 << 0 ) /* graphmgr thread */
#define FMGR_IDX_1		(1 << 1 )
#define FMGR_IDX_2		(1 << 2 )
#define FMGR_IDX_3		(1 << 3 )
#define FMGR_IDX_4		(1 << 4 )
#define FMGR_IDX_5		(1 << 5 )
#define FMGR_IDX_6		(1 << 6 )
#define FMGR_IDX_7		(1 << 7 )
#define FMGR_IDX_8		(1 << 8 )
#define FMGR_IDX_9		(1 << 9 )
#define FMGR_IDX_10		(1 << 10)
#define FMGR_IDX_11		(1 << 11)
#define FMGR_IDX_12		(1 << 12)
#define FMGR_IDX_13		(1 << 13)
#define FMGR_IDX_14		(1 << 14)
#define FMGR_IDX_15		(1 << 15)
#define FMGR_IDX_16		(1 << 16)
#define FMGR_IDX_17		(1 << 17)
#define FMGR_IDX_18		(1 << 18)
#define FMGR_IDX_19		(1 << 19)
#define FMGR_IDX_20		(1 << 20)
#define FMGR_IDX_21		(1 << 21)
#define FMGR_IDX_22		(1 << 22)
#define FMGR_IDX_23		(1 << 23)
#define FMGR_IDX_24		(1 << 24)
#define FMGR_IDX_25		(1 << 25)
#define FMGR_IDX_26		(1 << 26)
#define FMGR_IDX_27		(1 << 27)
#define FMGR_IDX_28		(1 << 28)
#define FMGR_IDX_29		(1 << 29)
#define FMGR_IDX_30		(1 << 30)
#define FMGR_IDX_31		(1 << 31)
#endif

#if 1
#define framemgr_e_barrier_irqs(framemgr, index, flag) \
	framemgr->sindex |= index; spin_lock_irqsave(&framemgr->slock, flag)
#define framemgr_x_barrier_irqr(framemgr, index, flag) \
	spin_unlock_irqrestore(&framemgr->slock, flag); framemgr->sindex &= ~index
#define framemgr_e_barrier_irq(framemgr, index) \
	framemgr->sindex |= index; spin_lock_irq(&framemgr->slock)
#define framemgr_x_barrier_irq(framemgr, index) \
	spin_unlock_irq(&framemgr->slock); framemgr->sindex &= ~index
#define framemgr_e_barrier(framemgr, index) \
	framemgr->sindex |= index; spin_lock(&framemgr->slock)
#define framemgr_x_barrier(framemgr, index) \
	spin_unlock(&framemgr->slock); framemgr->sindex &= ~index
#else
#define framemgr_e_barrier_irqs(framemgr, index, flag) \
	score_err("disabled spin_lock_irqsave \n"); flag = 0;
#define framemgr_x_barrier_irqr(framemgr, index, flag) \
	score_err("disabled spin_unlock_irqrestore \n"); flag = 0;
#define framemgr_e_barrier_irq(framemgr, index) \
	score_err("disabled spin_lock_irq \n");
#define framemgr_x_barrier_irq(framemgr, index) \
	score_err("disabled spin_unlock_irq \n")
#define framemgr_e_barrier(framemgr, index) \
	score_err("disabled spin_lock \n");
#define framemgr_x_barrier(framemgr, index) \
	score_err("disabled spin_unlock \n");
#endif
enum score_frame_state {
	SCORE_FRAME_STATE_FREE = 1,
	SCORE_FRAME_STATE_REQUEST,
	SCORE_FRAME_STATE_PREPARE,
	SCORE_FRAME_STATE_PROCESS,
	SCORE_FRAME_STATE_COMPLETE,
	SCORE_FRAME_STATE_INVALID
};

enum score_frame_message {
	SCORE_FRAME_REQUEST = 1,	/* 1 */
	SCORE_FRAME_INIT,		/* 2 */
	SCORE_FRAME_CREATE,		/* 3 */
	SCORE_FRAME_DESTROY,		/* 4 */
	SCORE_FRAME_ALLOCATE,		/* 5 */
	SCORE_FRAME_PROCESS,		/* 6 */
	SCORE_FRAME_DONE,		/* 7 */
	SCORE_FRAME_NDONE		/* 8 */
};

enum score_control_message {
	SCORE_CTRL_NONE = 100,
	SCORE_CTRL_STOP,
	SCORE_CTRL_STOP_DONE
};

struct score_frame {
	struct list_head		list;
	struct kthread_work		work;
	u32				state;

	u32				message;
	ulong				param0;
	ulong				param1;
	ulong				param2;
	ulong				param3;

	struct vb_container_list	*incl;
	struct vb_container_list	*otcl;
	ulong				flags;

	u32				id;
	u32				index;
	u32				findex;
	u32				gindex;
	void				*owner;

	struct score_time 		time[SCORE_TMP_COUNT];
	int				ret;
};

struct score_framemgr {
	u32				id;
	u32				sindex;
	spinlock_t			slock;
	struct score_frame		frame[SCORE_MAX_FRAME];

	struct list_head		frame_free_list;
	struct list_head		frame_request_list;
	struct list_head		frame_prepare_list;
	struct list_head		frame_process_list;
	struct list_head		frame_complete_list;

	u32				frame_cnt;
	u32				frame_fre_cnt;
	u32				frame_req_cnt;
	u32				frame_pre_cnt;
	u32				frame_pro_cnt;
	u32				frame_com_cnt;
};

void score_frame_s_free(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_g_free(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_free_head(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_free_tail(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_print_free_list(struct score_framemgr *framemgr);

void score_frame_s_request(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_g_request(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_request_head(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_request_tail(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_print_request_list(struct score_framemgr *framemgr);

void score_frame_s_prepare(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_g_prepare(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_prepare_head(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_prepare_tail(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_print_prepare_list(struct score_framemgr *framemgr);

void score_frame_s_process(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_g_process(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_process_head(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_process_tail(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_print_process_list(struct score_framemgr *framemgr);

void score_frame_s_complete(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_g_complete(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_complete_head(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_complete_tail(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_print_complete_list(struct score_framemgr *framemgr);

void score_frame_trans_fre_to_req(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_req_to_pre(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_req_to_pro(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_req_to_com(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_req_to_fre(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_pre_to_pro(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_pre_to_com(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_pre_to_fre(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_pro_to_com(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_pro_to_fre(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_com_to_fre(struct score_framemgr *framemgr, struct score_frame *frame);
void score_frame_trans_any_to_fre(struct score_framemgr *framemgr, struct score_frame *frame);

void score_frame_pick_fre_to_req(struct score_framemgr *framemgr, struct score_frame **frame);
void score_frame_print_all(struct score_framemgr *framemgr);

int score_frame_init(struct score_framemgr *framemgr, void *owner);
int score_frame_deinit(struct score_framemgr *framemgr);
void score_frame_flush(struct score_framemgr *framemgr);

#endif
