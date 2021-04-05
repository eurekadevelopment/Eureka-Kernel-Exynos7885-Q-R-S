/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SCORE_MEMORY_H_
#define SCORE_MEMORY_H_

#include <linux/platform_device.h>
#include <media/videobuf2-core.h>
#if defined(CONFIG_VIDEOBUF2_CMA_PHYS)
#include <media/videobuf2-cma-phys.h>
#elif defined(CONFIG_VIDEOBUF2_ION)
#include <media/videobuf2-ion.h>
#endif

#define SCORE_MEMORY_INTERNAL_SIZE	0x01000000 /* 16 MB */

#define SCORE_MEMORY_FW_MSG_SIZE	0x00100000 /*  1 MB */

enum score_memory_sync_type {
	SCORE_MEMORY_SYNC_FOR_CPU				= 0,
	SCORE_MEMORY_SYNC_FOR_DEVICE				= 1,
	SCORE_MEMORY_SYNC_FOR_BYDIRECTIONAL			= 2,
	SCORE_MEMORY_SYNC_FOR_NONE
};

struct score_memory_buffer {
	struct list_head		list;
	union {
		unsigned long		userptr;
		int			fd;
	} m;
	unsigned int			memory;
	dma_addr_t			dvaddr;
	void				*kvaddr;
	void				*mem_priv;
	void				*cookie;
	void				*dbuf;
	size_t				size;
};

struct score_memory_info {
	void		*cookie;
	void		*kvaddr;
	dma_addr_t	dvaddr;

	ulong		kvaddr_debug_cnt;
	u32		dvaddr_debug;
	ulong		kvaddr_debug;
};

struct score_mem_ops {
	void (*cleanup)(void *alloc_ctx);
	int (*resume)(void *alloc_ctx);
	void (*suspend)(void *alloc_ctx);
	void (*set_cacheable)(void *alloc_ctx, bool cacheable);
};

struct score_memory {
	struct device			*dev;
	struct vb2_alloc_ctx		*alloc_ctx;
	const struct score_mem_ops	*score_mem_ops;
	const struct vb2_mem_ops	*vb2_mem_ops;

	spinlock_t			im_lock;
	struct list_head		im_list;
	u32				im_count;
	struct score_memory_info		info;
};

int score_memory_probe(struct score_memory *memory, struct device *dev);
int score_memory_open(struct score_memory *memory);
int score_memory_close(struct score_memory *memory);
int score_memory_map(struct score_memory *memory,
		struct score_memory_buffer *buffer);
int score_memory_unmap(struct score_memory *memory,
		struct score_memory_buffer *buffer);
int score_memory_map_userptr(struct score_memory *memory,
		struct score_memory_buffer *buffer);
int score_memory_unmap_userptr(struct score_memory *memory,
		struct score_memory_buffer *buffer);
int score_memory_invalid_or_flush_userptr(struct score_memory_buffer *buffer,
		int sync_for, enum dma_data_direction dir);
#endif
