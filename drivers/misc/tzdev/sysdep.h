/*
 * Copyright (C) 2012-2016 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SYSDEP_H__
#define __SYSDEP_H__

#include <linux/idr.h>
#include <linux/kfifo.h>
#include <linux/migrate.h>
#include <linux/of.h>
#include <linux/version.h>

#include <asm/cacheflush.h>

#if defined(CONFIG_ARM64)
#define outer_inv_range(s, e)
#else
#define __flush_dcache_area(s, e)	__cpuc_flush_dcache_area(s, e)
#endif

#if defined(CONFIG_TZDEV_PAGE_MIGRATION)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 16, 0)
#define sysdep_migrate_pages(list, alloc, free)		migrate_pages((list), (alloc), (free), 0, MIGRATE_SYNC, MR_MEMORY_FAILURE)
#define sysdep_putback_isolated_pages(list)		putback_movable_pages(list)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
#define sysdep_migrate_pages(list, alloc, free)		({(void)free; migrate_pages((list), (alloc), 0, MIGRATE_SYNC, MR_MEMORY_FAILURE); })
#define sysdep_putback_isolated_pages(list)		putback_lru_pages(list)
#else
#define sysdep_migrate_pages(list, alloc, free)		({(void)free; migrate_pages((list), (alloc), 0, false, MIGRATE_SYNC); })
#define sysdep_putback_isolated_pages(list)		putback_lru_pages(list)
#endif
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
#define sysdep_kfifo_put(fifo, val) kfifo_put(fifo, val)
#else
#define sysdep_kfifo_put(fifo, val) kfifo_put(fifo, &val)
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 13, 0)
#define U8_MAX		((u8)~0U)
#define S8_MAX		((s8)(U8_MAX>>1))
#define S8_MIN		((s8)(-S8_MAX - 1))
#define U16_MAX		((u16)~0U)
#define S16_MAX		((s16)(U16_MAX>>1))
#define S16_MIN		((s16)(-S16_MAX - 1))
#define U32_MAX		((u32)~0U)
#define S32_MAX		((s32)(U32_MAX>>1))
#define S32_MIN		((s32)(-S32_MAX - 1))
#define U64_MAX		((u64)~0ULL)
#define S64_MAX		((s64)(U64_MAX>>1))
#define S64_MIN		((s64)(-S64_MAX - 1))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
#define idr_for_each_entry(idp, entry, id) \
		for (id = 0; ((entry) = idr_get_next(idp, &(id))) != NULL; ++id)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
#define for_each_cpu_mask(cpu, mask)	for_each_cpu((cpu), &(mask))
#define cpu_isset(cpu, mask)		cpumask_test_cpu((cpu), &(mask))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 5, 0)
#if !defined(CONFIG_OF)
static inline unsigned int irq_of_parse_and_map(struct device_node *dev, int index)
{
	(void)dev;
	(void)index;

	return 0;
}
#endif /*!defined(CONFIG_OF) */
#endif

int sysdep_idr_alloc(struct idr *idr, void *mem);

#endif /* __SYSDEP_H__ */
