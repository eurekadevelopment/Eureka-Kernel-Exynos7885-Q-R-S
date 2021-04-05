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
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/exynos_iovmm.h>
#include <linux/pm_runtime.h>

#include "score-buftracker.h"
#include "score-vertex.h"
#include "score-device.h"

/* #define TEST_BUFTRACKER */

int score_buftracker_init(struct score_buftracker *buftracker)
{
#ifdef TEST_BUFTRACKER
	int loop = 0;
	struct score_buftracker test_buftracker;
	struct score_memory_buffer *test_buffer;
#endif
	int ret = 0;

	buftracker->pos = 0;
	buftracker->buf_cnt = 0;
	INIT_LIST_HEAD(&buftracker->buf_list);
	spin_lock_init(&buftracker->buf_lock);

	buftracker->userptr_cnt = 0;
	INIT_LIST_HEAD(&buftracker->userptr_list);
	spin_lock_init(&buftracker->userptr_lock);
#ifdef TEST_BUFTRACKER
	test_buftracker.buf_cnt = 0;
	INIT_LIST_HEAD(&test_buftracker.buf_list);
	spin_lock_init(&test_buftracker.buf_lock);

	test_buffer = kzalloc(sizeof(struct score_memory_buffer) * 10, GFP_KERNEL);

	test_buffer[0].fd = 1;
	test_buffer[1].fd = 3;
	test_buffer[2].fd = 5;
	test_buffer[3].fd = 21;
	test_buffer[4].fd = 15;
	test_buffer[5].fd = 31;
	test_buffer[6].fd = 41;
	test_buffer[7].fd = 7;
	test_buffer[8].fd = 100;
	test_buffer[9].fd = 15;

	for (loop = 0; loop < 10; loop ++) {
		int ret = score_buftracker_add_list(&test_buftracker, &test_buffer[loop]);
		printk("add2 ret(%d) \n", ret);
		ret = loop;
	}

	score_buftracker_dump_list(&test_buftracker);

	kfree(test_buffer);
#endif

	return ret;
}

void score_buftracker_dump_list(struct score_buftracker *buftracker)
{
	struct score_memory_buffer *target_buf, *temp_buf;
	list_for_each_entry_safe(target_buf, temp_buf, &buftracker->buf_list, list) {
		printk("result:fd(%d) \n", target_buf->m.fd);
	}

	return;
}

int score_buftracker_add_list(struct score_buftracker *buftracker,
		struct score_memory_buffer *buffer)
{
	unsigned long flag;
	struct score_memory_buffer *target_buf, *temp_buf, *prev_buf;
	int ret = 0;

	score_note("list_fd(%d) \n", buffer->m.fd);
	spin_lock_irqsave(&buftracker->buf_lock, flag);
	if (buftracker->buf_cnt) {
		list_for_each_entry_safe(target_buf, temp_buf, &buftracker->buf_list, list) {
			if (target_buf->m.fd < buffer->m.fd) {
				prev_buf = container_of(target_buf->list.prev, struct score_memory_buffer, list);
				score_note("INSERT:buffer(%d) head_buf(%d) \n",
						buffer->m.fd, prev_buf->m.fd);
				list_add(&buffer->list, &prev_buf->list);
				buftracker->buf_cnt ++;
				ret = SCORE_BUFTRACKER_SEARCH_STATE_INSERT;
				goto exit;
			} else if (target_buf->m.fd == buffer->m.fd) {
				ret = SCORE_BUFTRACKER_SEARCH_STATE_ALREADY_REGISTERED;
				score_note("ALREADY:fd(%d) \n", buffer->m.fd);
				goto exit;
			}
		}
	}

	score_note("NEW:fd(%d) \n", buffer->m.fd);
	list_add_tail(&buffer->list, &buftracker->buf_list);
	buftracker->buf_cnt ++;
	ret = SCORE_BUFTRACKER_SEARCH_STATE_NEW;

exit:
	spin_unlock_irqrestore(&buftracker->buf_lock, flag);

	return ret;
}

int score_buftracker_add(struct score_buftracker *buftracker,
		struct score_memory_buffer *buffer)
{
	int ret = 0;
	int buf_ret = 0;
	struct score_vertex_ctx *vctx;
	struct score_memory *memory;

	vctx = container_of(buftracker, struct score_vertex_ctx, buftracker);
	memory = (struct score_memory *)&vctx->memory;

	/* copy buffer infomations */
#if 1
	memcpy((void *)&buftracker->buffer_slot[buftracker->pos],
			(void *)buffer,
			sizeof(struct score_memory_buffer));
#else
	buftracker->buffer_slot[buftracker->pos].m.fd = buffer->m.fd;
	buftracker->buffer_slot[buftracker->pos].dvaddr = buffer->dvaddr;
	buftracker->buffer_slot[buftracker->pos].kvaddr = buffer->kvaddr;
	buftracker->buffer_slot[buftracker->pos].mem_priv = buffer->mem_priv;
	buftracker->buffer_slot[buftracker->pos].cookie = buffer->cookie;
	buftracker->buffer_slot[buftracker->pos].dbuf = buffer->dbuf;
	buftracker->buffer_slot[buftracker->pos].size = buffer->size;
#endif
	/* add list to compare */
	buf_ret = score_buftracker_add_list(buftracker,
			&buftracker->buffer_slot[buftracker->pos]);

	/* if not exist buffer in list, completing this job */
	if ((buf_ret == SCORE_BUFTRACKER_SEARCH_STATE_NEW) ||
			(buf_ret == SCORE_BUFTRACKER_SEARCH_STATE_INSERT)) {
		switch (buffer->memory) {
		case VS4L_MEMORY_DMABUF:
			ret = score_memory_map(memory, buffer);
			break;
		case VS4L_MEMORY_USERPTR:
			ret = score_memory_map_userptr(memory, buffer);
			break;
		default:
			score_err("memory type is invalid (%d) \n", buffer->memory);
			break;
		}

		if (ret) {
			struct score_memory_buffer *target_buf, *temp_buf;
			score_err("score_memory_map is fail (%d) \n", ret);
			/* score_memory_unmap(memory, &score_buffer); */

			switch (buffer->memory) {
			case VS4L_MEMORY_DMABUF:
				score_memory_unmap(memory, buffer);
				break;
			case VS4L_MEMORY_USERPTR:
				score_memory_unmap_userptr(memory, buffer);
				break;
			default:
				score_err("memory type is invalid (%d) \n", buffer->memory);
				break;
			}

			list_for_each_entry_safe(target_buf, temp_buf,
					&buftracker->buf_list, list) {
				if (target_buf->m.fd == buffer->m.fd) {
					list_del(&buffer->list);

					break;
				}
			}

			memset(&buftracker->buffer_slot[buftracker->pos],
					0x0,
					sizeof(struct score_memory_buffer));

			goto p_err;
		}

		buftracker->buffer_slot[buftracker->pos].m.fd = buffer->m.fd;
		buftracker->buffer_slot[buftracker->pos].dvaddr = buffer->dvaddr;
		buftracker->buffer_slot[buftracker->pos].mem_priv = buffer->mem_priv;
		buftracker->buffer_slot[buftracker->pos].cookie = buffer->cookie;
		buftracker->buffer_slot[buftracker->pos].dbuf = buffer->dbuf;
		buftracker->buffer_slot[buftracker->pos].size = buffer->size;
#ifdef BUF_MAP_KVADDR
		buftracker->buffer_slot[buftracker->pos].kvaddr = buffer->kvaddr;
#endif
		buftracker->pos ++;

		/* score_buftracker_dump_list(buftracker); */
	} else {
		/* removed canditated buffer infomation to use next time */
		memset(&buftracker->buffer_slot[buftracker->pos],
				0x0,
				sizeof(struct score_memory_buffer));
	}

	/* score_buftracker_dump_list(buftracker); */
	/* score_buftracker_dump(buftracker); */

	return ret;

p_err:
	return -1;
}

int score_buftracker_remove(struct score_buftracker *buftracker,
	struct score_memory_buffer *buffer)
{
	int ret = 0;

	return ret;
}

int score_buftracker_dump(struct score_buftracker *buftracker)
{
	int loop, ret = 0;

	for (loop = 0; loop < buftracker->pos; loop ++) {
#ifdef BUF_MAP_KVADDR
		score_info("(%d) fd(%d) dvaddr(0x%llx) (%p, %p, %p, %p) size(%ld) \n",
				loop,
				buftracker->buffer_slot[loop].m.fd,
				buftracker->buffer_slot[loop].dvaddr,
				buftracker->buffer_slot[loop].kvaddr,
				buftracker->buffer_slot[loop].mem_priv,
				buftracker->buffer_slot[loop].cookie,
				buftracker->buffer_slot[loop].dbuf,
				buftracker->buffer_slot[loop].size);
#else
		score_info("(%d) fd(%d) dvaddr(0x%llx) (%p, %p, %p) size(%ld) \n",
				loop,
				buftracker->buffer_slot[loop].m.fd,
				buftracker->buffer_slot[loop].dvaddr,
				buftracker->buffer_slot[loop].mem_priv,
				buftracker->buffer_slot[loop].cookie,
				buftracker->buffer_slot[loop].dbuf,
				buftracker->buffer_slot[loop].size);
#endif
#ifdef BUF_DUMP_KVADDR
		if (buftracker->buffer_slot[loop].kvaddr != NULL) {
			print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
					buftracker->buffer_slot[loop].kvaddr,
					buftracker->buffer_slot[loop].size,
					false);
		}
#endif
	}

	return ret;
}

int score_buftracker_remove_all(struct score_buftracker *buftracker)
{
	int ret = 0;
	struct score_vertex_ctx *vctx;
	struct score_memory *memory;

	vctx = container_of(buftracker, struct score_vertex_ctx, buftracker);
	memory = (struct score_memory *)&vctx->memory;

	score_note("memory %p pos(%d) \n", memory, buftracker->pos);
	/* score_buftracker_dump_list(buftracker); */
	/* score_buftracker_dump(buftracker); */

	/* HACK */
	/* need to change logic */
	while (buftracker->pos > 0) {
		switch (buftracker->buffer_slot[buftracker->pos -1].memory) {
		case VS4L_MEMORY_DMABUF:
			ret = score_memory_unmap(memory,
					&buftracker->buffer_slot[buftracker->pos - 1]);
			break;
		case VS4L_MEMORY_USERPTR:
			/* DO NOTHING */
			break;
		default:
			score_err("memory type is invalid (%d) \n",
					buftracker->buffer_slot[buftracker->pos - 1].memory);
			break;
		}

		if (ret)
			score_err("score_memory_unmap is fail (%d) \n", ret);

		buftracker->pos --;
	}

	return ret;
}

int score_buftracker_remove_userptr_all(struct score_buftracker *buftracker)
{
	int ret = 0;
	struct score_vertex_ctx *vctx;
	struct score_memory *memory;
	const struct vb2_mem_ops *mem_ops;
	struct score_memory_buffer *buffer;
	unsigned long flags;

	vctx = container_of(buftracker, struct score_vertex_ctx, buftracker);
	memory = (struct score_memory *)&vctx->memory;
	mem_ops = memory->vb2_mem_ops;

	BUG_ON(!memory);
	BUG_ON(!mem_ops);

	while(buftracker->userptr_cnt > 0) {
		buffer = container_of(buftracker->userptr_list.next,
				struct score_memory_buffer, list);

		if (buffer->mem_priv)
			mem_ops->put_userptr(buffer->mem_priv);

		spin_lock_irqsave(&buftracker->userptr_lock, flags);
		list_del(&buffer->list);
		buftracker->userptr_cnt--;
		kfree(buffer);
		spin_unlock_irqrestore(&buftracker->userptr_lock, flags);
	}

	return ret;
}

int score_buftracker_map_userptr(struct score_buftracker *buftracker, struct score_memory_buffer *userptr_buffer)
{
	int ret = 0;
	void *cookie;
	struct score_memory_buffer *buffer;
	const struct vb2_mem_ops *mem_ops;
	unsigned long flags;
	struct score_vertex_ctx *vctx;
	struct score_memory *memory;

	vctx = container_of(buftracker, struct score_vertex_ctx, buftracker);
	memory = (struct score_memory *)&vctx->memory;

#ifdef BUF_MAP_KVADDR
	void *kvaddr = NULL;
#endif
	cookie = NULL;

	BUG_ON(!memory);
	BUG_ON(!userptr_buffer);

	mem_ops = memory->vb2_mem_ops;
	BUG_ON(!mem_ops);

	buffer = kzalloc(sizeof(struct score_memory_buffer), GFP_KERNEL);
	buffer->m.userptr = userptr_buffer->m.userptr;
	buffer->size = userptr_buffer->size;

	ret = score_memory_map_userptr(memory, buffer);
	if (ret) {
		score_err("score_memory_map_userptr is fail(%d)\n", ret);
		goto p_err;
	}

	spin_lock_irqsave(&buftracker->userptr_lock, flags);
	list_add_tail(&buffer->list, &buftracker->userptr_list);
	buftracker->userptr_cnt++;
	spin_unlock_irqrestore(&buftracker->userptr_lock, flags);

	userptr_buffer->dvaddr = buffer->dvaddr;
	userptr_buffer->cookie = buffer->cookie;

	return 0;

p_err:
	if (buffer)
		kfree(buffer);

	return ret;
}

int score_buftracker_invalid_or_flush_userptr_all(struct score_buftracker *buftracker,
		int sync_for, enum dma_data_direction dir)
{
	int ret = 0;
	struct score_memory_buffer *buffer;
	struct score_memory_buffer *temp_buf;

	list_for_each_entry_safe(buffer, temp_buf,
			&buftracker->userptr_list, list) {

		ret = score_memory_invalid_or_flush_userptr(buffer, sync_for, dir);
	}

	return ret;
}
