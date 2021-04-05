/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/device.h>
#include <media/videobuf2-ion.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
#include <linux/exynos_iovmm.h>
#else
#include <plat/iovmm.h>
#endif

#include "score-config.h"
#include "score-binary.h"
#include "score-memory.h"
#include "score-debug.h"

#if defined(CONFIG_VIDEOBUF2_ION)
const struct score_mem_ops score_mem_ops = {
	.cleanup	= vb2_ion_destroy_context,
	.resume		= vb2_ion_attach_iommu,
	.suspend	= vb2_ion_detach_iommu,
	.set_cacheable	= vb2_ion_set_cached,
};
#endif

static int __score_memory_alloc(struct score_memory *memory)
{
	int ret = 0;
	void *cookie, *kvaddr;
	dma_addr_t dvaddr;
	struct score_memory_info *info;

	cookie = NULL;
	kvaddr = NULL;
	dvaddr = 0;
	info = &memory->info;

#if defined(CONFIG_VIDEOBUF2_ION)
	cookie = vb2_ion_private_alloc(memory->alloc_ctx, SCORE_MEMORY_INTERNAL_SIZE);
	if (IS_ERR(cookie)) {
		score_err("vb2_ion_private_alloc is failed");
		return PTR_ERR(cookie);
	}

	ret = vb2_ion_dma_address(cookie, &dvaddr);
	if (ret) {
		score_err("vb2_ion_dma_address is fail(%d)\n", ret);
		vb2_ion_private_free(cookie);
		return ret;
	}

	kvaddr = vb2_ion_private_vaddr(cookie);
	if (IS_ERR_OR_NULL(kvaddr)) {
		score_err("vb2_ion_private_vaddr is failed");
		vb2_ion_private_free(cookie);
		return PTR_ERR(kvaddr);
	}

	info->cookie = cookie;
	info->dvaddr = dvaddr;
	info->kvaddr = kvaddr;
#endif

	score_info("internal dvaddr : 0x%llX\n", dvaddr);
	score_info("internal kvaddr : %p\n", kvaddr);
	return ret;
}

int score_memory_probe(struct score_memory *memory, struct device *dev)
{
	u32 ret = 0;
	struct vb2_alloc_ctx *alloc_ctx;

	SCORE_TP();
	BUG_ON(!memory);
	BUG_ON(!dev);

	alloc_ctx = NULL;

#if defined(CONFIG_VIDEOBUF2_ION)
	alloc_ctx = vb2_ion_create_context(dev, SZ_4K, VB2ION_CTX_IOMMU | VB2ION_CTX_VMCONTIG);
	if (IS_ERR(alloc_ctx)) {
		probe_err("vb2_ion_create_context is fail\n");
		return PTR_ERR(alloc_ctx);
	}

	memory->dev = dev;
	memory->alloc_ctx = alloc_ctx;
	memory->score_mem_ops = &score_mem_ops;
	memory->vb2_mem_ops = &vb2_ion_memops;
#endif

	spin_lock_init(&memory->im_lock);
	INIT_LIST_HEAD(&memory->im_list);
	memory->im_count = 0;

#ifdef PROBE_ALLOC_INTERNAL_MEM
	ret = __score_memory_alloc(memory);
	if (ret) {
		score_err("__score_memory_alloc is fail(%d)\n", ret);
	}
#endif


	return ret;
}

int score_memory_open(struct score_memory *memory)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!memory);

#ifndef PROBE_ALLOC_INTERNAL_MEM
	ret = __score_memory_alloc(memory);
	if (ret) {
		score_err("__score_memory_alloc is fail(%d)\n", ret);
		goto p_err;
	}
p_err:
#endif
	return ret;
}

int score_memory_close(struct score_memory *memory)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!memory);
#ifndef PROBE_ALLOC_INTERNAL_MEM
#if defined(CONFIG_VIDEOBUF2_ION)
	vb2_ion_private_free(memory->info.cookie);
#endif
#endif

	return ret;
}

int score_memory_map(struct score_memory *memory, struct score_memory_buffer *buffer)
{
	int ret = 0;
	void *mem_priv;
	void *cookie;
	dma_addr_t dvaddr;
	struct dma_buf *dbuf;
	const struct vb2_mem_ops *mem_ops;
	unsigned long flags;

#ifdef BUF_MAP_KVADDR
	void *kvaddr = NULL;
#endif
	cookie = NULL;

	SCORE_TP();
	BUG_ON(!memory);
	BUG_ON(!buffer);

	mem_ops = memory->vb2_mem_ops;
	BUG_ON(!mem_ops);

	if (buffer->m.fd <= 0) {
		score_err("fd(%d) is invalid\n", buffer->m.fd);
		ret = -EINVAL;
		goto p_err;
	}

	dbuf = dma_buf_get(buffer->m.fd);
	if (IS_ERR_OR_NULL(dbuf)) {
		score_err("dma_buf_get is fail(%d:%p)\n", buffer->m.fd, dbuf);
		ret = -EINVAL;
		goto p_err;
	}

	buffer->dbuf = dbuf;

	if (dbuf->size < buffer->size) {
		score_err("user buffer size is small(%zd, %zd)\n", dbuf->size, buffer->size);
		ret = -EINVAL;
		goto p_err;
	}

	if (buffer->size != dbuf->size) {
		score_info("(%d)buffer size is changed to dbuf(%zd, %zd)\n",
				buffer->m.fd, dbuf->size, buffer->size);
		score_event_msg("(%d)buffer size is changed to dbuf(%zd, %zd)\n",
				buffer->m.fd, dbuf->size, buffer->size);
		buffer->size = dbuf->size;
	}

	/* Acquire each plane's memory */
	mem_priv = mem_ops->attach_dmabuf(memory->alloc_ctx, dbuf, buffer->size, 1);
	if (IS_ERR(mem_priv)) {
		score_err("call_memop(attach_dmabuf) is fail(%p, %zd)\n", memory->alloc_ctx, buffer->size);
		ret = PTR_ERR(mem_priv);
		goto p_err;
	}

	buffer->mem_priv = mem_priv;

	cookie = mem_ops->cookie(mem_priv);
	if (IS_ERR_OR_NULL(cookie)) {
		score_err("call_memop(cookie) is fail(%p, %p)\n", mem_priv, cookie);
		ret = PTR_ERR(mem_priv);
		goto p_err;
	}

	buffer->cookie = cookie;

	ret = mem_ops->map_dmabuf(mem_priv);
	if (ret) {
		score_err("call_memop(map_dmabuf) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vb2_ion_dma_address(cookie, &dvaddr);
	if (ret) {
		score_err("vb2_ion_dma_address is fail(%d)\n", ret);
		goto p_err;
	}

	buffer->dvaddr = dvaddr;

#ifdef BUF_MAP_KVADDR
	kvaddr = mem_ops->vaddr(mem_priv);
	if (IS_ERR_OR_NULL(kvaddr)) {
		score_err("vb2_ion_private_vaddr is failed");
		goto p_err;
	}

	buffer->kvaddr = kvaddr;
#endif

	spin_lock_irqsave(&memory->im_lock, flags);
	/* HACK */
	/* list_add_tail(&buffer->list, &memory->im_list); */
	memory->im_count++;
	spin_unlock_irqrestore(&memory->im_lock, flags);

	score_event_msg("mem_fd(%d), priv(%p), dvaddr(0x%llx) dbuf(%p) size(%d) \n",
				buffer->m.fd,
				buffer->mem_priv,
				buffer->dvaddr,
				buffer->dbuf,
				buffer->size);

	return 0;

p_err:
	if (buffer->dvaddr)
		mem_ops->unmap_dmabuf(buffer->mem_priv);

	if (buffer->mem_priv)
		mem_ops->detach_dmabuf(buffer->mem_priv);

	if (buffer->dbuf)
		dma_buf_put(buffer->dbuf);

	return ret;
}

int score_memory_unmap(struct score_memory *memory, struct score_memory_buffer *buffer)
{
	int ret = 0;
	const struct vb2_mem_ops *mem_ops;
	unsigned long flags;

	SCORE_TP();
	BUG_ON(!memory);
	BUG_ON(!buffer);

	mem_ops = memory->vb2_mem_ops;

	score_event_msg("mem_fd(%d), priv(%p), dvaddr(0x%llx) dbuf(%p) size(%d) \n",
				buffer->m.fd,
				buffer->mem_priv,
				buffer->dvaddr,
				buffer->dbuf,
				buffer->size);

	if (buffer->dvaddr)
		mem_ops->unmap_dmabuf(buffer->mem_priv);

	if (buffer->mem_priv)
		mem_ops->detach_dmabuf(buffer->mem_priv);

	if (buffer->dbuf)
		dma_buf_put(buffer->dbuf);

	spin_lock_irqsave(&memory->im_lock, flags);
	/* HACK */
	/* list_del(&buffer->list); */
	memory->im_count--;
	spin_unlock_irqrestore(&memory->im_lock, flags);

	return ret;
}

int score_memory_unmap_userptr(struct score_memory *memory,
		struct score_memory_buffer *buffer)
{
	int ret = 0;
	const struct vb2_mem_ops *mem_ops;

	SCORE_TP();
	BUG_ON(!memory);
	BUG_ON(!buffer);

	mem_ops = memory->vb2_mem_ops;

	score_event_msg("userptr(0x%lx), priv(%p), dvaddr(0x%llx) dbuf(%p) size(%ld) \n",
				buffer->m.userptr,
				buffer->mem_priv,
				buffer->dvaddr,
				buffer->dbuf,
				buffer->size);

	if (buffer->mem_priv)
		mem_ops->put_userptr(buffer->mem_priv);

	return ret;
}

int score_memory_map_userptr(struct score_memory *memory, struct score_memory_buffer *buffer)
{
	int ret = 0;
	void *mem_priv;
	void *cookie;
	dma_addr_t dvaddr;
	const struct vb2_mem_ops *mem_ops;

#ifdef BUF_MAP_KVADDR
	void *kvaddr = NULL;
#endif
	cookie = NULL;
	mem_ops = memory->vb2_mem_ops;

	SCORE_TP();
	BUG_ON(!memory);
	BUG_ON(!mem_ops);

	/* Acquire each plane's memory */
	mem_priv = mem_ops->get_userptr(memory->alloc_ctx, buffer->m.userptr, buffer->size, 1);
	if (IS_ERR(mem_priv)) {
		score_err("call_memop(attach_dmabuf) is fail(%p, %zd)\n", memory->alloc_ctx, buffer->size);
		ret = PTR_ERR(mem_priv);
		goto p_err;
	}

	buffer->mem_priv = mem_priv;

	cookie = mem_ops->cookie(mem_priv);
	if (IS_ERR_OR_NULL(cookie)) {
		score_err("call_memop(cookie) is fail(%p, %p)\n", mem_priv, cookie);
		ret = PTR_ERR(mem_priv);
		goto p_err;
	}

	buffer->cookie = cookie;

	ret = vb2_ion_dma_address(cookie, &dvaddr);
	if (ret) {
		score_err("vb2_ion_dma_address is fail(%d)\n", ret);
		goto p_err;
	}

	buffer->dvaddr = dvaddr;

#ifdef BUF_MAP_KVADDR
	kvaddr = mem_ops->vaddr(mem_priv);
	if (IS_ERR_OR_NULL(kvaddr)) {
		score_err("vb2_ion_private_vaddr is failed");
		goto p_err;
	}

	buffer->kvaddr = kvaddr;
#endif

	score_event_msg("userptr(0x%lx), priv(%p), dvaddr(0x%llx) dbuf(%p) size(%ld) \n",
				buffer->m.userptr,
				buffer->mem_priv,
				buffer->dvaddr,
				buffer->dbuf,
				buffer->size);

	return 0;

p_err:
	if (buffer->dvaddr)
		mem_ops->unmap_dmabuf(buffer->mem_priv);

	if (buffer->mem_priv)
		mem_ops->detach_dmabuf(buffer->mem_priv);

	if (buffer->dbuf)
		dma_buf_put(buffer->dbuf);

	if (buffer)
		kfree(buffer);

	return ret;
}

int score_memory_invalid_or_flush_userptr(struct score_memory_buffer *buffer,
		int sync_for, enum dma_data_direction dir)
{
	int ret = 0;
	/* struct device *dev; */
	/* struct score_vertex_ctx *vctx; */
	/* struct score_vertex *vertex; */
	/* struct score_device *device; */
	/* vctx = container_of(buftracker, struct score_vertex_ctx, buftracker); */
	/* vertex = vctx->vertex; */
	/* device = container_of(vertex, struct score_device, vertex); */
	/* dev = device->dev; */

	score_event_msg("userptr(0x%lx), priv(%p), dvaddr(0x%llx) \
			dbuf(%p) size(%ld) sync/dir(%d %d)\n",
			buffer->m.userptr,
			buffer->mem_priv,
			buffer->dvaddr,
			buffer->dbuf,
			buffer->size,
			sync_for,
			dir);

#ifdef USE_DIRECT_CACHE_CALL
	if (sync_for == SCORE_MEMORY_SYNC_FOR_CPU)
		exynos_iommu_sync_for_cpu(dev, buffer->dvaddr, buffer->size, dir);
	else if(sync_for == SCORE_MEMORY_SYNC_FOR_DEVICE)
		exynos_iommu_sync_for_device(dev, buffer->dvaddr, buffer->size, dir);
#else
	if (sync_for == SCORE_MEMORY_SYNC_FOR_CPU)
		vb2_ion_sync_for_cpu(buffer->cookie, 0, buffer->size, dir);
	else if(sync_for == SCORE_MEMORY_SYNC_FOR_DEVICE)
		vb2_ion_sync_for_device(buffer->cookie, 0, buffer->size, dir);
#endif

	return ret;
}
