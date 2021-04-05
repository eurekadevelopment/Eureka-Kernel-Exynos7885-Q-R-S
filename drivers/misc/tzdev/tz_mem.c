/*
 * Copyright (C) 2016 Samsung Electronics, Inc.
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

#include <linux/delay.h>
#include <linux/list.h>
#include <linux/migrate.h>
#include <linux/mmzone.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include "sysdep.h"
#include "tzdev.h"
#include "tz_iwio.h"
#include "tz_mem.h"
#include "tz_shmem_validator.h"

#define TZDEV_MIGRATION_MAX_RETRIES 20

#define TZDEV_PFNS_PER_PAGE		(PAGE_SIZE / sizeof(sk_pfn_t))
#define TZDEV_IWSHMEM_IDS_PER_PAGE	(PAGE_SIZE / sizeof(uint32_t))

#define TZDEV_IWSHMEM_REG_FLAG_WRITE	(1 << 0)
#define TZDEV_IWSHMEM_REG_FLAG_KERNEL	(1 << 1)

static void *tzdev_mem_release_buf;
static DEFINE_IDR(tzdev_mem_map);
static DEFINE_MUTEX(tzdev_mem_mutex);

int isolate_lru_page(struct page *page);

static __maybe_unused unsigned int gup_flags(int write, int force)
{
	unsigned int flags = 0;

	if (write)
		flags |= FOLL_WRITE;
	if (force)
		flags |= FOLL_FORCE;

	return flags;
}

static unsigned long __tzdev_get_user_pages(struct task_struct *task,
		struct mm_struct *mm, unsigned long start, unsigned long nr_pages,
		int write, int force, struct page **pages,
		struct vm_area_struct **vmas)
{
	struct page **cur_pages = pages;
	unsigned long nr_pinned = 0;
	int res;

	while (nr_pinned < nr_pages) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 168)
		res = get_user_pages(task, mm, start, nr_pages - nr_pinned,
				gup_flags(write, force), cur_pages, vmas);
#else
		res = get_user_pages(task, mm, start, nr_pages - nr_pinned, write,
				force, cur_pages, vmas);
#endif
		if (res < 0)
			return nr_pinned;

		start += res * PAGE_SIZE;
		nr_pinned += res;
		cur_pages += res;
	}

	return nr_pinned;
}

/* This is the same approach to pinning user memory
 * as used in Infiniband drivers.
 * Refer to drivers/inifiniband/core/umem.c */
static int tzdev_get_user_pages(struct task_struct *task, struct mm_struct *mm,
		unsigned long start, unsigned long nr_pages, int write,
		int force, struct page **pages, struct vm_area_struct **vmas)
{
	unsigned long i, locked, lock_limit, nr_pinned;

	if (!can_do_mlock())
		return -EPERM;

	down_write(&mm->mmap_sem);

	locked = nr_pages + mm->pinned_vm;
	lock_limit = rlimit(RLIMIT_MEMLOCK) >> PAGE_SHIFT;

	if ((locked > lock_limit) && !capable(CAP_IPC_LOCK)) {
		up_write(&mm->mmap_sem);
		return -ENOMEM;
	}

	nr_pinned = __tzdev_get_user_pages(task, mm, start, nr_pages, write,
						force, pages, vmas);
	if (nr_pinned != nr_pages)
		goto fail;

	mm->pinned_vm = locked;
	up_write(&mm->mmap_sem);

	return 0;

fail:
	for (i = 0; i < nr_pinned; i++)
		put_page(pages[i]);

	up_write(&mm->mmap_sem);

	return -EFAULT;
}

static void tzdev_put_user_pages(struct page **pages, unsigned long nr_pages)
{
	unsigned long i;

	for (i = 0; i < nr_pages; i++) {
		/* NULL pointers may appear here due to unsuccessful migration */
		if (pages[i])
			put_page(pages[i]);
	}
}

static void tzdev_decrease_pinned_vm(struct mm_struct *mm, unsigned long nr_pages)
{
	down_write(&mm->mmap_sem);
	mm->pinned_vm -= nr_pages;
	up_write(&mm->mmap_sem);
}

static void tzdev_mem_free(int id, struct tzdev_mem_reg *mem, unsigned int is_user)
{
	struct task_struct *task;
	struct mm_struct *mm;

	if (!mem->pid) {
		if (!is_user) {
			idr_remove(&tzdev_mem_map, id);
			kfree(mem);
		}

		/* Nothing to do for kernel memory */
		return;
	}

	idr_remove(&tzdev_mem_map, id);

	tzdev_put_user_pages(mem->pages, mem->nr_pages);

	task = get_pid_task(mem->pid, PIDTYPE_PID);
	put_pid(mem->pid);
	if (!task)
		goto out;

	mm = get_task_mm(task);
	put_task_struct(task);
	if (!mm)
		goto out;

	tzdev_decrease_pinned_vm(mm, mem->nr_pages);
	mmput(mm);

out:
	kfree(mem->pages);
	kfree(mem);
}

static void tzdev_mem_list_release(unsigned char *buf, unsigned int cnt)
{
	uint32_t *ids;
	unsigned int i;
	struct tzdev_mem_reg *mem;

	ids = (uint32_t *)buf;
	for (i = 0; i < cnt; i++) {
		mem = idr_find(&tzdev_mem_map, ids[i]);
		BUG_ON(!mem);
		tzdev_mem_free(ids[i], mem, 0);
	}
}

static void _tzdev_mem_release_all(unsigned int is_user)
{
	struct tzdev_mem_reg *mem;
	unsigned int id;

	mutex_lock(&tzdev_mem_mutex);
	idr_for_each_entry(&tzdev_mem_map, mem, id)
		tzdev_mem_free(id, mem, is_user);
	mutex_unlock(&tzdev_mem_mutex);

	if (!is_user)
		idr_destroy(&tzdev_mem_map);
}

static int _tzdev_mem_release(int id, unsigned int is_user)
{
	struct tzdev_mem_reg *mem;
	struct tz_iwio_aux_channel *ch;
	long cnt;
	int ret = 0;

	mutex_lock(&tzdev_mem_mutex);

	mem = idr_find(&tzdev_mem_map, id);
	if (!mem) {
		ret = -ENOENT;
		goto out;
	}

	if (is_user != !!mem->pid) {
		ret = -EPERM;
		goto out;
	}

	ch = tz_iwio_get_aux_channel();
	cnt = tzdev_smc_shmem_list_rls(id);
	if (cnt > 0) {
		BUG_ON(cnt > TZDEV_IWSHMEM_IDS_PER_PAGE);

		memcpy(tzdev_mem_release_buf, ch->buffer, cnt * sizeof(uint32_t));
		tz_iwio_put_aux_channel();

		tzdev_mem_list_release(tzdev_mem_release_buf, cnt);
	} else {
		ret = cnt;
		tz_iwio_put_aux_channel();
	}

out:
	mutex_unlock(&tzdev_mem_mutex);

	return ret;
}

static int _tzdev_mem_register(struct tzdev_mem_reg *mem, sk_pfn_t *pfns,
		unsigned long nr_pages, unsigned int flags)
{
	struct tz_iwio_aux_channel *ch;
	int i, id;

	mutex_lock(&tzdev_mem_mutex);
	id = sysdep_idr_alloc(&tzdev_mem_map, mem);
	if (id < 0)
		goto out;

	ch = tz_iwio_get_aux_channel();

	for (i = 0; i < nr_pages; i += TZDEV_PFNS_PER_PAGE) {
		memcpy(ch->buffer, &pfns[i], min(nr_pages - i, TZDEV_PFNS_PER_PAGE) * sizeof(sk_pfn_t));
		if (tzdev_smc_shmem_list_reg(id, nr_pages, flags)) {
			idr_remove(&tzdev_mem_map, id);
			id = -EFAULT;
			break;
		}
	}

	tz_iwio_put_aux_channel();

out:
	mutex_unlock(&tzdev_mem_mutex);

	return id;
}

#if defined(CONFIG_TZDEV_PAGE_MIGRATION)

static struct page *tzdev_alloc_kernel_page(struct page *page, unsigned long private, int **x)
{
	return alloc_page(GFP_KERNEL);
}

static void tzdev_free_kernel_page(struct page *page, unsigned long private)
{
	__free_page(page);
}

static unsigned long tzdev_get_migratetype(struct page *page)
{
	struct zone *zone;
	unsigned long flags;
	unsigned long migrate_type;

	/* Zone lock must be held to avoid race with
	 * set_pageblock_migratetype() */
	zone = page_zone(page);
	spin_lock_irqsave(&zone->lock, flags);
	migrate_type = get_pageblock_migratetype(page);
	spin_unlock_irqrestore(&zone->lock, flags);

	return migrate_type;
}

static int tzdev_verify_migration_page(struct page *page)
{
	unsigned long migrate_type;

	migrate_type = tzdev_get_migratetype(page);
	if (migrate_type == MIGRATE_CMA || migrate_type == MIGRATE_ISOLATE)
		return -EFAULT;

	return 0;
}

static int tzdev_verify_migration(struct page **pages, unsigned long nr_pages)
{
	unsigned int i;
	int ret;

	for (i = 0; i < nr_pages; i++) {
		ret = tzdev_verify_migration_page(pages[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int __tzdev_migrate_pages(struct task_struct *task, struct mm_struct *mm,
		unsigned long start, unsigned long nr_pages, int write,
		int force, struct page **pages, unsigned long *verified_bitmap)
{
	unsigned long i = 0, migrate_nr = 0, nr_pin = 0;
	unsigned long cur_pages_index, cur_start, pinned, migrate_type;
	int res;
	struct page **cur_pages;
	LIST_HEAD(pages_list);
	int ret = 0;

	/* Add migrating pages to the list */
	while ((i = find_next_zero_bit(verified_bitmap, nr_pages, i)) < nr_pages) {
		migrate_type = tzdev_get_migratetype(pages[i]);
		/* Skip pages that is currently isolated by somebody.
		 * Isolated page may originally have MIGRATE_CMA type,
		 * so caller should repeat migration for such pages */
		if (migrate_type == MIGRATE_ISOLATE) {
			ret = -EAGAIN;
			i++;
			continue;
		}

		/* Mark non-CMA pages as verified and skip them */
		if (migrate_type != MIGRATE_CMA) {
			bitmap_set(verified_bitmap, i, 1);
			i++;
			continue;
		}

		/* Call migrate_prep() once if migration necessary */
		if (migrate_nr == 0)
			migrate_prep();

		/* Pages should be isolated from an LRU list before migration.
		 * If isolation failed skip this page and inform caller to
		 * repeat migrate operation */
		res = isolate_lru_page(pages[i]);
		if (res < 0) {
			ret = -EAGAIN;
			i++;
			continue;
		}

		list_add_tail(&pages[i]->lru, &pages_list);
		put_page(pages[i]);
		/* pages array will be refilled with migrated pages later */
		pages[i] = NULL;
		migrate_nr++;
		i++;
	}

	if (!migrate_nr)
		return ret;

	/* make migration */
	res = sysdep_migrate_pages(&pages_list, tzdev_alloc_kernel_page, tzdev_free_kernel_page);
	if (res) {
		sysdep_putback_isolated_pages(&pages_list);
		return -EFAULT;
	}

	/* pin migrated pages */
	i = 0;
	do {
		nr_pin = 0;

		/* find index of the next migrated page */
		while (i < nr_pages && pages[i])
			i++;

		cur_pages = &pages[i];
		cur_pages_index = i;
		cur_start = start + i * PAGE_SIZE;

		/* find continuous migrated pages range */
		while (i < nr_pages && !pages[i]) {
			nr_pin++;
			i++;
		}

		/* and pin it */
		down_write(&mm->mmap_sem);
		pinned = __tzdev_get_user_pages(task, mm, cur_start, nr_pin,
						write, force, cur_pages, NULL);
		up_write(&mm->mmap_sem);
		if (pinned != nr_pin)
			return -EFAULT;

		/* Check that migrated pages are not MIGRATE_CMA or MIGRATE_ISOLATE
		 * and mark them as verified. If it is not true inform caller
		 * to repeat migrate operation */
		if (tzdev_verify_migration(cur_pages, nr_pin) == 0)
			bitmap_set(verified_bitmap, cur_pages_index, nr_pin);
		else
			ret = -EAGAIN;

		migrate_nr -= nr_pin;
	} while (migrate_nr);

	return ret;
}

static int tzdev_migrate_pages(struct task_struct *task, struct mm_struct *mm,
		unsigned long start, unsigned long nr_pages, int write,
		int force, struct page **pages)
{
	int ret;
	unsigned int retries = 0;
	unsigned long *verified_bitmap;
	size_t bitmap_size = DIV_ROUND_UP(nr_pages, BITS_PER_LONG);

	verified_bitmap = kcalloc(bitmap_size, sizeof(unsigned long), GFP_KERNEL);
	if (!verified_bitmap)
		return -ENOMEM;

	do {
		ret = __tzdev_migrate_pages(task, mm, start, nr_pages, write,
				force, pages, verified_bitmap);

		if (ret != -EAGAIN || (retries++ >= TZDEV_MIGRATION_MAX_RETRIES))
			break;
		msleep(1);
	} while (1);

	kfree(verified_bitmap);

	return ret;
}
#endif /* CONFIG_TZDEV_PAGE_MIGRATION */

int tzdev_mem_init(void)
{
	struct page *page;

	page = alloc_page(GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	tzdev_mem_release_buf = page_address(page);

	tzdev_print(0, "AUX channels mem release buffer allocated\n");

	return 0;
}

void tzdev_mem_fini(void)
{
	_tzdev_mem_release_all(0);
	__free_page(virt_to_page(tzdev_mem_release_buf));
}

int tzdev_mem_register_user(struct tzio_mem_register *s)
{
	struct pid *pid;
	struct task_struct *task;
	struct mm_struct *mm;
	struct page **pages;
	struct tzdev_mem_reg *mem;
	sk_pfn_t *pfns;
	unsigned long start, end;
	unsigned long nr_pages = 0;
	int ret, res, i, id;
	unsigned int flags = 0;

	if (s->size) {
		if (!access_ok(s->write ? VERIFY_WRITE : VERIFY_READ, s->ptr, s->size))
			return -EFAULT;

		start = (unsigned long)s->ptr >> PAGE_SHIFT;
		end = ((unsigned long)s->ptr + s->size + PAGE_SIZE - 1) >> PAGE_SHIFT;
		nr_pages = end - start;
	}

	if (s->write)
		flags |= TZDEV_IWSHMEM_REG_FLAG_WRITE;

	if (tz_shmem_validator_approve(s->pid, s->ptr, s->size))
		return -EPERM;

	pid = find_get_pid(s->pid);
	if (!pid)
		return -ESRCH;

	task = get_pid_task(pid, PIDTYPE_PID);
	if (!task) {
		ret = -ESRCH;
		goto out_pid;
	}

	mm = get_task_mm(task);
	if (!mm) {
		ret = -ESRCH;
		goto out_task;
	}

	pages = kcalloc(nr_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		ret = -ENOMEM;
		goto out_mm;
	}

	pfns = kmalloc(nr_pages * sizeof(sk_pfn_t), GFP_KERNEL);
	if (!pfns) {
		ret = -ENOMEM;
		goto out_pages;
	}

	mem = kmalloc(sizeof(struct tzdev_mem_reg), GFP_KERNEL);
	if (!mem) {
		ret = -ENOMEM;
		goto out_pfns;
	}

	mem->pid = pid;
	mem->nr_pages = nr_pages;
	mem->pages = pages;

	res = tzdev_get_user_pages(task, mm, (unsigned long)s->ptr,
			nr_pages, 1, !s->write, pages, NULL);
	if (res) {
		tzdev_print(0, "Failed to pin user pages (%d)\n", res);
		ret = res;
		goto out_mem;
	}
#if defined(CONFIG_TZDEV_PAGE_MIGRATION)
	/*
	 * In case of enabled migration it is possible that userspace pages
	 * will be migrated from current physical page to some other
	 * To avoid fails of CMA migrations we have to move pages to other
	 * region which can not be inside any CMA region. This is done by
	 * allocations with GFP_KERNEL flag to point UNMOVABLE memblock
	 * to be used for such allocations.
	 */
	res = tzdev_migrate_pages(task, mm, (unsigned long)s->ptr, nr_pages,
			1, !s->write, pages);
	if (res < 0) {
		tzdev_print(0, "Failed to migrate CMA pages (%d)\n", res);
		ret = res;
		goto out_pin;
	}
#endif /* CONFIG_TZDEV_PAGE_MIGRATION */
	for (i = 0; i < nr_pages; i++)
		pfns[i] = page_to_pfn(pages[i]);

	id = _tzdev_mem_register(mem, pfns, nr_pages, flags);
	if (id < 0) {
		ret = id;
		goto out_pin;
	}

	s->id = id;

	kfree(pfns);

	mmput(mm);
	put_task_struct(task);

	return 0;

out_pin:
	tzdev_put_user_pages(pages, nr_pages);
	tzdev_decrease_pinned_vm(mm, nr_pages);
out_mem:
	kfree(mem);
out_pfns:
	kfree(pfns);
out_pages:
	kfree(pages);
out_mm:
	mmput(mm);
out_task:
	put_task_struct(task);
out_pid:
	put_pid(pid);
	return ret;
}

int tzdev_mem_register(void *ptr, unsigned long size, unsigned int write)
{
	struct tzdev_mem_reg *mem;
	struct page *page;
	sk_pfn_t *pfns;
	unsigned long addr, start, end;
	unsigned long nr_pages = 0;
	int ret, i, id;
	unsigned int flags = TZDEV_IWSHMEM_REG_FLAG_KERNEL;

	if (size) {
		addr = (unsigned long)ptr;

		BUG_ON(addr + size <= addr || !IS_ALIGNED(addr | size, PAGE_SIZE));

		start = addr >> PAGE_SHIFT;
		end = (addr + size) >> PAGE_SHIFT;
		nr_pages = end - start;
	}

	if (write)
		flags |= TZDEV_IWSHMEM_REG_FLAG_WRITE;

	pfns = kmalloc(nr_pages * sizeof(sk_pfn_t), GFP_KERNEL);
	if (!pfns)
		return -ENOMEM;

	mem = kmalloc(sizeof(struct tzdev_mem_reg), GFP_KERNEL);
	if (!mem) {
		ret = -ENOMEM;
		goto out_pfns;
	}

	mem->pid = NULL;

	for (i = 0; i < nr_pages; i++) {
		page = virt_to_page(addr + PAGE_SIZE * i);
#if defined(CONFIG_TZDEV_PAGE_MIGRATION)
		ret = tzdev_verify_migration_page(page);
		if (ret)
			goto out_mem;
#endif
		pfns[i] = page_to_pfn(page);
	}

	id = _tzdev_mem_register(mem, pfns, nr_pages, flags);
	if (id < 0) {
		ret = id;
		goto out_mem;
	}

	kfree(pfns);

	return id;

out_mem:
	kfree(mem);
out_pfns:
	kfree(pfns);

	return ret;
}

int tzdev_mem_pages_register(struct page **pages, unsigned int nr_pages, unsigned int write)
{
	int ret, id;
	unsigned int i;
	struct tzdev_mem_reg *mem;
	sk_pfn_t *pfns;
	unsigned int flags = TZDEV_IWSHMEM_REG_FLAG_KERNEL;

	if (write)
		flags |= TZDEV_IWSHMEM_REG_FLAG_WRITE;

	pfns = kmalloc(nr_pages * sizeof(sk_pfn_t), GFP_KERNEL);
	if (!pfns)
		return -ENOMEM;

	mem = kmalloc(sizeof(struct tzdev_mem_reg), GFP_KERNEL);
	if (!mem) {
		ret = -ENOMEM;
		goto out_pfns;
	}

	mem->pid = NULL;

	for (i = 0; i < nr_pages; i++) {
#if defined(CONFIG_TZDEV_PAGE_MIGRATION)
		ret = tzdev_verify_migration_page(pages[i]);
		if (ret)
			goto out_mem;
#endif
		pfns[i] = page_to_pfn(pages[i]);
	}

	id = _tzdev_mem_register(mem, pfns, nr_pages, flags);
	if (id < 0) {
		ret = id;
		goto out_mem;
	}

	kfree(pfns);

	return id;

out_mem:
	kfree(mem);

out_pfns:
	kfree(pfns);

	return ret;
}

int tzdev_mem_release_user(unsigned int id)
{
	return _tzdev_mem_release(id, 1);
}

int tzdev_mem_release(unsigned int id)
{
	return _tzdev_mem_release(id, 0);
}

void tzdev_mem_release_all_user(void)
{
	_tzdev_mem_release_all(1);
}

int tzdev_is_mem_exist(unsigned int id, unsigned int *is_user)
{
	struct tzdev_mem_reg *mem;

	mutex_lock(&tzdev_mem_mutex);
	mem = idr_find(&tzdev_mem_map, id);
	if (!mem) {
		mutex_unlock(&tzdev_mem_mutex);
		return 0;
	}

	*is_user = !!mem->pid;
	mutex_unlock(&tzdev_mem_mutex);

	return 1;
}
