/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  linux/fs/ntfs3/debug.h
 *
 * Copyright (C) 2019-2020 Paragon Software GmbH, All rights reserved.
 *
 * useful functions for debuging
 */

#ifndef Add2Ptr
#define Add2Ptr(P, I) (void *)((u8 *)(P) + (I))
#define PtrOffset(B, O) ((size_t)((size_t)(O) - (size_t)(B)))
#endif

#define QuadAlign(n) (((n) + 7u) & (~7u))
#define IsQuadAligned(n) (!((size_t)(n)&7u))
#define Quad2Align(n) (((n) + 15u) & (~15u))
#define IsQuad2Aligned(n) (!((size_t)(n)&15u))
#define Quad4Align(n) (((n) + 31u) & (~31u))
#define IsSizeTAligned(n) (!((size_t)(n) & (sizeof(size_t) - 1)))
#define DwordAlign(n) (((n) + 3u) & (~3u))
#define IsDwordAligned(n) (!((size_t)(n)&3u))
#define WordAlign(n) (((n) + 1u) & (~1u))
#define IsWordAligned(n) (!((size_t)(n)&1u))

__printf(3, 4) void __ntfs_trace(const struct super_block *sb,
				 const char *level, const char *fmt, ...);
__printf(3, 4) void __ntfs_fs_error(struct super_block *sb, int report,
				    const char *fmt, ...);
__printf(3, 4) void __ntfs_inode_trace(struct inode *inode, const char *level,
				       const char *fmt, ...);

#define ntfs_trace(sb, fmt, args...) __ntfs_trace(sb, KERN_NOTICE, fmt, ##args)
#define ntfs_error(sb, fmt, args...) __ntfs_trace(sb, KERN_ERR, fmt, ##args)
#define ntfs_warning(sb, fmt, args...)                                         \
	__ntfs_trace(sb, KERN_WARNING, fmt, ##args)

#define ntfs_fs_error(sb, fmt, args...) __ntfs_fs_error(sb, 1, fmt, ##args)
#define ntfs_inode_error(inode, fmt, args...)                                  \
	__ntfs_inode_trace(inode, KERN_ERR, fmt, ##args)
#define ntfs_inode_warning(inode, fmt, args...)                                \
	__ntfs_inode_trace(inode, KERN_WARNING, fmt, ##args)

static inline void *ntfs_alloc(size_t size, int zero)
{
	void *p = kmalloc(size, zero ? (GFP_NOFS | __GFP_ZERO) : GFP_NOFS);

	return p;
}

static inline void ntfs_free(void *p)
{
	if (!p)
		return;
	kfree(p);
}

static inline void trace_mem_report(int on_exit)
{
}

static inline void ntfs_init_trace_file(void)
{
}

static inline void ntfs_close_trace_file(void)
{
}

static inline void *ntfs_memdup(const void *src, size_t len)
{
	void *p = ntfs_alloc(len, 0);

	if (p)
		memcpy(p, src, len);
	return p;
}
