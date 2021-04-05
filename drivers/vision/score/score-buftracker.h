
/* * Samsung Exynos SoC series SCORE driver *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef SCORE_BUFTRACKER_H_
#define SCORE_BUFTRACKER_H_

#include "vision-buffer.h"
#include "score-memory.h"

#define BUF_MAX_COUNT				(64)

enum score_buftracker_search_state {
	SCORE_BUFTRACKER_SEARCH_STATE_NEW			= 1,
	SCORE_BUFTRACKER_SEARCH_STATE_INSERT			= 2,
	SCORE_BUFTRACKER_SEARCH_STATE_ALREADY_REGISTERED	= 3,
	SCORE_BUFTRACKER_SEARCH_STATE_NONE
};

enum score_buftracker_state {
	SCORE_BUFTRACKER_STATE_OPEN,
	SCORE_BUFTRACKER_STATE_START
};

struct score_buftracker {
	struct score_memory_buffer	buffer_slot[BUF_MAX_COUNT];
	unsigned int			pos;

	struct list_head		buf_list;
	u32				buf_cnt;
	spinlock_t			buf_lock;

	struct list_head		userptr_list;
	u32				userptr_cnt;
	spinlock_t			userptr_lock;
};

int score_buftracker_init(struct score_buftracker *buftracker);
int score_buftracker_add(struct score_buftracker *buftracker,
		struct score_memory_buffer *buffer);
int score_buftracker_add_list(struct score_buftracker *buftracker,
		struct score_memory_buffer *buffer);
void score_buftracker_dump_list(struct score_buftracker *buftracker);
int score_buftracker_remove(struct score_buftracker *buftracker,
		struct score_memory_buffer *buffer);
int score_buftracker_dump(struct score_buftracker *buftracker);
int score_buftracker_remove_all(struct score_buftracker *buftracker);
int score_buftracker_map_userptr(struct score_buftracker *buftracker,
		struct score_memory_buffer *userptr_buffer);
int score_buftracker_remove_userptr_all(struct score_buftracker *buftracker);
int score_buftracker_invalid_or_flush_userptr(struct score_buftracker *buftracker,
		struct score_memory_buffer *buffer,
		int sync_for, enum dma_data_direction dir);
int score_buftracker_invalid_or_flush_userptr_all(struct score_buftracker *buftracker,
		int sync_for, enum dma_data_direction dir);

#endif
