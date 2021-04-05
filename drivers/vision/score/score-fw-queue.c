/*
 * linux/drivers/soc/score/score-fw-queue.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include <asm/cacheflush.h>

#include <linux/workqueue.h>

/* for proc */
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include "score-fw-queue.h"
#include "score-fw-common.h"
#include "score-device.h"
#include "score-system.h"

#include "score-debug-print.h"

/* #define SCORE_FW_MSG_DEBUG */

extern struct score_fw_dev *score_fw_device;

void score_fw_queue_dump_packet_word(struct score_ipc_packet *cmd)
{
	unsigned int size = cmd->size.packet_size;
	unsigned int *dump = (unsigned int *)cmd;
	int i;

	for (i = 0; i < size; ++i) {
		score_debug("Packet[%d] is %08X\n", i, dump[i]);
	}
}

void score_fw_queue_dump_packet_word2(struct score_ipc_packet *cmd)
{
	unsigned int size = cmd->size.packet_size;
	unsigned int *dump = (unsigned int *)cmd;
	int i;

	score_info("Packet are (size %d)--------------------------- \n", size);
	for (i = 0; i < size; ++i) {
		score_info("[%d][0x%08X] \n", i, dump[i]);
	}
	score_info("End --------------------------------- \n");
}

void score_fw_dump_regs(void __iomem *base_addr, u32 size)
{
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
		base_addr, size, false);
}

static inline unsigned int score_fw_queue_get_head(struct score_fw_queue *queue)
{
	return SCORE_READ_REG(queue->head_info);
}

static inline unsigned int score_fw_queue_get_tail(struct score_fw_queue *queue)
{
	return SCORE_READ_REG(queue->tail_info);
}

static int score_fw_queue_put_data_direct(struct score_fw_queue *queue, unsigned int pos,
						unsigned int data)
{
	unsigned int reg_pos;

	if (pos >= queue->size)
		return -EINVAL;

	reg_pos = queue->start + (SCORE_REG_SIZE * pos);
	SCORE_WRITE_REG(data, reg_pos);

	return 0;
}

static int score_fw_queue_get_data_direct(struct score_fw_queue *queue, unsigned int pos,
						unsigned int *data)
{
	unsigned int reg_pos;

	if (pos >= queue->size)
		return -EINVAL;

	reg_pos = queue->start + (SCORE_REG_SIZE * pos);
	*data = SCORE_READ_REG(reg_pos);

	return 0;
}

unsigned int score_queue_get_size(struct score_fw_queue *queue)
{
	return queue->size;
}

static unsigned int score_fw_queue_get_used(struct score_fw_queue *queue)
{
	unsigned int used;
	unsigned int head, tail;
	unsigned int head_idx, tail_idx;

	head = score_fw_queue_get_head(queue);
	tail = score_fw_queue_get_tail(queue);
	head_idx = GET_QUEUE_IDX(head, queue->size);
	tail_idx = GET_QUEUE_IDX(tail, queue->size);

	if (head_idx > tail_idx)
		used = head_idx - tail_idx;
	else if (head_idx < tail_idx)
		used = (queue->size - tail_idx) + head_idx;
	else if (head != tail)
		used = queue->size;
	else
		used = 0;

	return used;
}

static void score_fw_queue_inc_head(struct score_fw_queue *queue,
				unsigned int head, unsigned int size)
{
	unsigned int info;

	info = GET_QUEUE_MIRROR_IDX((head + size), queue->size);
	SCORE_WRITE_REG(info, queue->head_info);
}

static void score_fw_queue_inc_tail(struct score_fw_queue *queue,
				unsigned int tail, unsigned int size)
{
	unsigned int info;

	info = GET_QUEUE_MIRROR_IDX((tail + size), queue->size);
	SCORE_WRITE_REG(info, queue->tail_info);
}

int score_fw_queue_is_empty(struct score_fw_queue *queue)
{
	int ret = -1;
	unsigned int head, tail;
	head = score_fw_queue_get_head(queue);
	tail = score_fw_queue_get_tail(queue);

	if (head == tail)
		ret = 0;

	return ret;
}

static unsigned int score_fw_queue_atomic_inc_task_id(struct score_fw_queue *queue)
{
	int val = atomic_read(&queue->task_id);
	int old, new;

	do {
		old = val;
		new = (old+1) & MAX_SCORE_REQUEST;
		val = atomic_cmpxchg(&queue->task_id, old, new);
	} while (val != old);

	return (unsigned int)old;
}

unsigned int score_fw_queue_get_inc_task_id(struct score_fw_queue *queue)
{
	return score_fw_queue_atomic_inc_task_id(queue);
}

static inline unsigned int score_fw_queue_get_remain(struct score_fw_queue *queue)
{
	return (queue->size - score_fw_queue_get_used(queue));
}

void score_fw_queue_dump_packet_group(struct score_ipc_packet *cmd)
{
	struct score_packet_size s = cmd->size;
	struct score_packet_header h = cmd->header;
	unsigned int group_count = s.group_count;
	int i, j;

	score_debug("packet Size : %5d(packet_size), %3d(group_count)\n",
		s.packet_size, group_count);
	score_debug("Packet Header : %d(queue_id), %4d(kernel_name), %3d(task_id), \
		%d(worker_name)\n", h.queue_id, h.kernel_name, h.task_id, h.worker_name);

	for (i = 0; i < group_count; ++i) {
		struct score_packet_group g = cmd->group[i];
		unsigned int valid_size = g.header.valid_size;
		score_debug("Group[%3d] header : %2d(valid_size), %08X(fd_bitmap)\n",
			i, valid_size, g.header.fd_bitmap);
		for (j = 0; j < valid_size; ++j) {
			score_debug("Group[%3d] params[%2d] : %08X\n", i, j, g.data.params[j]);
		}
	}
}

static int score_fw_queue_get_total_valid_size(struct score_ipc_packet *cmd)
{
	struct score_packet_group *group = cmd->group;
	unsigned int group_count = cmd->size.group_count;
	unsigned int size = (sizeof(struct score_packet_size) +
			sizeof(struct score_packet_header)) >> 2;
	unsigned int valid_size;
	unsigned int fd_bitmap;
	int buf_count;
	int i, j;

	for (i = 0; i < group_count; ++i) {
		valid_size = group[i].header.valid_size;
		fd_bitmap  = group[i].header.fd_bitmap;
		buf_count  = 0;

		for (j = 0; j < valid_size; ++j) {
			if (fd_bitmap & (0x1 << j)) {
				buf_count++;
			}
		}

		size = size + valid_size - ((sizeof(struct sc_host_buffer) >> 2) * buf_count);
	}

	return size;
}

#if 0
static int score_fw_queue_put_pending_params(struct score_fw_queue *queue,
						struct score_ipc_packet *cmd)
{
	int ret = 0;
	struct score_fw_pending_param *p_param;
	struct score_fw_param_list *p_list;

	p_param = &queue->pending_param;
	p_list = kmalloc(sizeof(struct score_fw_param_list), GFP_KERNEL);
	if (!p_list) {
		// @@@ change error func
		/* dev_err(&dev->pdev->dev, "Failed to allocate pending param\n"); */
		score_debug("Failed to allocate pending param \n");
		ret = -ENOMEM;
		goto pending_err;
	}

	score_debug("Data is put at pending buffer\n");
	p_list->p = cmd;

	score_fw_queue_dump_packet_group(cmd);

	INIT_LIST_HEAD(&p_list->p_list);

	mutex_lock(&p_param->lock);
	list_add_tail(&p_list->p_list, &p_param->param_list);
	mutex_unlock(&p_param->lock);

pending_err:
	return ret;
}
#endif

static int score_fw_queue_put_direct_params(struct score_fw_queue *queue,
						struct score_ipc_packet *cmd)
{
	int ret = 0;
	int i, j, k, offset = 0;
	unsigned int size = score_fw_queue_get_total_valid_size(cmd);
	unsigned int head = score_fw_queue_get_head(queue);

	unsigned int *size_word = (unsigned int *)&cmd->size;
	unsigned int *header_word = (unsigned int *)&cmd->header;

	unsigned int group_count = cmd->size.group_count;
	struct score_packet_group *group = cmd->group;
	unsigned int valid_size;
	unsigned int fd_bitmap;
	struct sc_packet_buffer *buffer;
	int memory_type;

	/* score_debug("Data is put directly at cmd queue\n"); */

	cmd->size.packet_size = size;
	score_fw_queue_put_direct(queue,
		GET_QUEUE_MIRROR_IDX(head, queue->size), *size_word);
	score_fw_queue_put_direct(queue,
		GET_QUEUE_MIRROR_IDX(head + 1, queue->size), *header_word);

	for (i = 0; i < group_count; ++i) {
		valid_size = group[i].header.valid_size;
		fd_bitmap  = group[i].header.fd_bitmap;

		for (j = 0; j < valid_size; ++j, ++offset) {
			if (fd_bitmap & (0x1 << j)) {
				buffer = (struct sc_packet_buffer*)(&group[i].data.params[j]);
				memory_type = buffer->host_buf.memory_type;
#if 0
				if (memory_type == VS4L_MEMORY_USERPTR) {
				/* TODO: this is temporary code. */
				/* this will be changed in consideration of 32 or 64 bit library and SMMU */
					buffer->buf.addr = buffer->host_buf.addr32;
				} else if (memory_type == VS4L_MEMORY_DMABUF) {
					/* TODO:need memory translating for SCore */
				} else {
					ret = -EINVAL;
					goto exit_enqueue;
				}
#endif
				for (k = 0; k < (sizeof(struct sc_buffer) >> 2); ++k) {
					score_fw_queue_put_direct(queue,
						GET_QUEUE_MIRROR_IDX(head + 2 + offset + k, queue->size),
						group[i].data.params[j + k]);
				}
				j += ((sizeof(struct sc_packet_buffer) >> 2) - 1);
				offset += ((sizeof(struct sc_buffer) >> 2) - 1);
			} else {
				score_fw_queue_put_direct(queue,
					GET_QUEUE_MIRROR_IDX(head + 2 + offset, queue->size),
					group[i].data.params[j]);
			}
		}
	}
	score_fw_queue_inc_head(queue, head, size);

	return ret;
}

int score_fw_queue_put(struct score_fw_queue *queue, struct score_ipc_packet *cmd)
{
	int ret;
	/* int i; */
#if 0
	// @@@ temporary size: -1
	unsigned int remain;
	unsigned int size;

	mutex_lock(&queue->lock);
	remain = score_fw_queue_get_remain(queue);
	if (unlikely(remain > queue->size)) {
		ret = -EFAULT;
		goto put_unlock;
	}

	// @@@ temp
	size = score_fw_queue_get_total_valid_size(cmd);
	score_debug("[%d] size is needed, [%d] size is remained\n", size, remain);

	if (remain < size) {
		mutex_unlock(&queue->lock);
		ret = score_fw_queue_put_pending_params(queue, cmd);
		goto put_ret;
	} else {
		ret = score_fw_queue_put_direct_params(queue, cmd);
	}

put_unlock:
	mutex_unlock(&queue->lock);
put_ret:
#else
	/* score_fw_dump_regs(score_fw_device->sfr + 0x0, 0x100); */
	/* score_fw_queue_dump_packet_word2(cmd); */
	/* score_fw_dump_regs(score_fw_device->sfr + 0x7000, 56 * 4); */
	mutex_lock(&queue->lock);\

	/* HACK : (FW) Access previously buffer issue */
#if 0
	for (i = 0; i < 56; i ++) {
		writel(0x0, score_fw_device->sfr + 0x7000 + (i * 4));
	}
	score_fw_dump_regs(score_fw_device->sfr + 0x7000, 56 * 4);
#endif
	ret = score_fw_queue_put_direct_params(queue, cmd);
	mutex_unlock(&queue->lock);
#endif
	return ret;
}

int score_fw_queue_put_direct(struct score_fw_queue *queue, unsigned int head, unsigned int data)
{
	unsigned int head_idx = GET_QUEUE_IDX(head, queue->size);
#ifdef SCORE_FW_MSG_DEBUG
	unsigned int debug_tail =
        GET_QUEUE_IDX(score_fw_queue_get_tail(queue), queue->size);
	/* TODO: temporary wait: -1 */
	/* --> mutex_lock(&queue->lock); */

	score_debug("Queue size:%d, head:%d, tail:%d\n",
						queue->size, head_idx, debug_tail);
#endif
	score_fw_queue_put_data_direct(queue, head_idx, data);

	/* TODO: cond signal(?) */
	/* --> mutex_unlock(&queue->lock); */

	return 0;
}

int score_fw_queue_get_direct(struct score_fw_queue *queue, unsigned int tail, unsigned int *data)
{
	unsigned int tail_idx = GET_QUEUE_IDX(tail, queue->size);
#ifdef SCORE_FW_MSG_DEBUG
	unsigned int debug_head =
        GET_QUEUE_IDX(score_fw_queue_get_head(queue), queue->size);

	/* TODO: mutex_lock(&queue->lock); */

	score_debug("Queue size:%d, head:%d, tail:%d\n",
						queue->size, debug_head, tail_idx);
#endif
	score_fw_queue_get_data_direct(queue, tail_idx, data);

	/* TODO: cond signal(?) */
	/* TODO: mutex_unlock(&queue->lock); */

	return 0;
}

int score_fw_queue_get(struct score_fw_queue *queue, struct score_ipc_packet *cmd)
{
	int ret = 0;
	unsigned int tail = score_fw_queue_get_tail(queue);
	unsigned int size;

	unsigned int *size_word = (unsigned int *)&cmd->size;
	unsigned int *header_word = (unsigned int *)&cmd->header;

	unsigned int i;

	score_fw_queue_get_direct(queue,
		GET_QUEUE_MIRROR_IDX(tail, queue->size), size_word);
	score_fw_queue_get_direct(queue,
		GET_QUEUE_MIRROR_IDX(tail + 1, queue->size), header_word);

	size = cmd->size.packet_size;

	score_note("packet size(%d) \n", size);
	for (i = 0; i < size - 2; ++i) {
		score_fw_queue_get_direct(queue,
			GET_QUEUE_MIRROR_IDX((tail + 2 + i), queue->size),
		&cmd->group[0].data.params[i]);
		/* HACK */
		if (size == 0)
			break;
	}

	score_fw_queue_inc_tail(queue, tail, size);

	if (cmd->group[0].data.params[0] == 0x11111111) {
		score_fw_dump_regs(score_fw_device->sfr + 0x7000, 56 * 4);

		score_debug("Packet Size   : %08X\n", *size_word);
		score_debug("Packet Header : %08X\n", *header_word);
		score_debug("Out Result    : %08X\n", cmd->group[0].data.params[0]);

		score_fw_dump_regs(score_fw_device->sfr, 28 * 4);

		ret = -1111;
	} else {
#ifdef SCORE_FW_MSG_DEBUG
		score_fw_dump_regs(score_fw_device->sfr + 0x7000, 56 * 4);

		score_debug("Packet Size   : %08X\n", *size_word);
		score_debug("Packet Header : %08X\n", *header_word);
		score_debug("Out Result    : %08X\n", cmd->group[0].data.params[0]);

		score_fw_dump_regs(score_fw_device->sfr, 28 * 4);
#endif
	}

	return ret;
}

static void score_fw_queue_param_work(struct work_struct *work)
{
	struct score_fw_queue *queue;
	struct score_fw_pending_param *p_param;
	struct score_fw_param_list *p_list;
	unsigned int remain;
	unsigned int size;
	int ret;

	queue = score_fw_device->in_queue;
	p_param = &queue->pending_param;

	while (!list_empty(&p_param->param_list)) {
		p_list = list_entry(p_param->param_list.next, typeof(*p_list), p_list);
		remain = score_fw_queue_get_remain(queue);
		size = score_fw_queue_get_total_valid_size(p_list->p);

		score_debug("[%d] size is needed, [%d] size is remained\n", size, remain);

		if (size <= remain) {
			ret = score_fw_queue_put_direct_params(queue, p_list->p);
			if (ret) {
				continue;
			}

			mutex_lock(&p_param->lock);
			list_del(&p_list->p_list);
			mutex_unlock(&p_param->lock);

			kfree(p_list);
		} else {
			score_debug("Remained queue size is not enough\n");
			break;
		}
	}
}

int score_fw_queue_init(struct score_fw_dev *dev)
{
	int ret = 0;
	struct score_fw_queue *in_queue;
	struct score_fw_queue *out_queue;
	struct score_device *device;
	struct score_system *system;

	device = container_of(dev, struct score_device, fw_dev);
	system = &device->system;

	dev->sfr = system->regs;

	/* @@@ need to use devm_kzalloc */
	/* dev->in_queue = devm_kzalloc(&dev->pdev->dev, sizeof(struct score_fw_queue), GFP_KERNEL); */
	dev->in_queue = kzalloc(sizeof(struct score_fw_queue), GFP_KERNEL);
	if (!dev->in_queue) {
		dev_err(&dev->pdev->dev, "Failed to allocate memory for in_queue\n");
		return -ENOMEM;
	}

	/* dev->out_queue = devm_kzalloc(&dev->pdev->dev, sizeof(struct score_fw_queue), GFP_KERNEL); */
	dev->out_queue = kzalloc(sizeof(struct score_fw_queue), GFP_KERNEL);
	if (!dev->out_queue) {
		dev_err(&dev->pdev->dev, "Failed to allocate memory for out_queue\n");
		return -ENOMEM;
	}

	in_queue = dev->in_queue;
	out_queue = dev->out_queue;

	in_queue->type = SCORE_IN_QUEUE;
	in_queue->head_info = SCORE_IN_QUEUE_HEAD_INFO;
	in_queue->tail_info = SCORE_IN_QUEUE_TAIL_INFO;
	in_queue->start = SCORE_IN_QUEUE_START;
	in_queue->size = SCORE_IN_QUEUE_REG_SIZE;
	atomic_set(&in_queue->task_id, 0);
	mutex_init(&in_queue->lock);
	spin_lock_init(&in_queue->slock);
	INIT_LIST_HEAD(&in_queue->wait_list);
	mutex_init(&in_queue->pending_param.lock);
	INIT_LIST_HEAD(&in_queue->pending_param.param_list);
	INIT_WORK(&in_queue->pending_param.param_work, score_fw_queue_param_work);

	out_queue->type = SCORE_OUT_QUEUE;
	out_queue->head_info = SCORE_OUT_QUEUE_HEAD_INFO;
	out_queue->tail_info = SCORE_OUT_QUEUE_TAIL_INFO;
	out_queue->start = SCORE_OUT_QUEUE_START;
	out_queue->size = SCORE_OUT_QUEUE_REG_SIZE;

	/* TODO: need to check belows
	* atomic_set(&out_queue->task_id, 0);
	* mutex_init(&out_queue->lock);
	* spin_lock_init(&out_queue->slock);
	* INIT_LIST_HEAD(&out_queue->wait_list);
	* mutex_init(&out_queue->pending_param.lock);
	* INIT_LIST_HEAD(&out_queue->pending_param.param_list);
	* @@@ need ?
	* INIT_WORK(&out_queue->pending_param.param_work, score_fw_queue_param_work);
	*/

	ret = score_printf_buf_init(dev,
			(void *)system->fw_msg_kvaddr,
			system->fw_msg_dvaddr);
	if (ret)
		score_err("score_printf_buf_init is fail(%d)\n", ret);

	score_fw_dump_regs(score_fw_device->sfr + 0x0, 0x70);

	return 0;
}

void score_fw_queue_exit(struct score_fw_dev *dev)
{
	struct score_fw_queue *in_queue;
	struct score_fw_queue *out_queue;

	score_printf_buf_release(dev);

	in_queue = dev->in_queue;
	out_queue = dev->out_queue;

	if (in_queue != NULL) {
		mutex_destroy(&in_queue->lock);
		kfree(in_queue);
		in_queue = NULL;
	}

	if (out_queue != NULL) {
		mutex_destroy(&out_queue->lock);
		kfree(out_queue);
		out_queue = NULL;
	}
}

void score_wake_up_wait_task(void)
{
	struct score_ipc_packet *cmd;
	struct score_fw_wait_task *task;
	struct score_fw_queue *in_queue;
	struct score_fw_queue *out_queue;

	in_queue = score_fw_device->in_queue;
	out_queue = score_fw_device->out_queue;

	cmd = kmalloc(sizeof(struct score_ipc_packet) +
		sizeof(struct score_packet_group), GFP_KERNEL);
	if (!cmd) {
		/* dev_err(&dev->pdev->dev, "Failed to allocate score ipc packet\n"); */
		score_debug("Failed to allocate score ipc packet \n");
		/* TODO: Change value of ORQ */
		return;
	}
	score_fw_queue_get(out_queue, cmd);

	spin_lock(&in_queue->slock);
	list_for_each_entry(task, &in_queue->wait_list, wait_queue) {
		if (task->task_id == cmd->header.task_id) {
			score_debug("task_id[%d] is completed\n", task->task_id);
			list_del(&task->wait_queue);
			task->result = cmd->group[0].data.params[0];

			/* wake_up_interruptible(&task->wait); */
			clear_bit(SCORE_WAIT_STATE, &task->wait_state);
			wake_up(&task->wait);
			break;
		}
	}
	spin_unlock(&in_queue->slock);

	kfree(cmd);
}
