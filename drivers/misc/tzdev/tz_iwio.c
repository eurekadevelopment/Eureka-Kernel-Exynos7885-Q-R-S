/*
 * Copyright (C) 2013-2016 Samsung Electronics, Inc.
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

#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
#include <asm/pgtable.h>

#include "tzdev.h"
#include "tz_iwio.h"
#include "tzlog.h"

#ifdef CONFIG_TZDEV_SK_MULTICORE

static DEFINE_PER_CPU(struct tz_iwio_aux_channel *, aux_channel);

#define aux_channel_get(ch)		get_cpu_var(ch)
#define aux_channel_put(ch)		put_cpu_var(ch)
#define aux_channel_init(ch, cpu)	per_cpu(ch, cpu)

#else /* CONFIG_TZDEV_SK_MULTICORE */

static DEFINE_MUTEX(aux_channel_lock);
static struct tz_iwio_aux_channel *aux_channel[NR_CPUS];

static struct tz_iwio_aux_channel *aux_channel_get(struct tz_iwio_aux_channel *ch[])
{
	mutex_lock(&aux_channel_lock);
	BUG_ON(smp_processor_id() != 0);
	return ch[0];
}

#define aux_channel_put(ch)		mutex_unlock(&aux_channel_lock)
#define aux_channel_init(ch, cpu)	ch[cpu]

#endif /* CONFIG_TZDEV_SK_MULTICORE */

struct tz_iwio_aux_channel *tz_iwio_get_aux_channel(void)
{
	return aux_channel_get(aux_channel);
}

void tz_iwio_put_aux_channel(void)
{
	aux_channel_put(aux_channel);
}

int tz_iwio_alloc_aux_channel(int cpu)
{
	struct tz_iwio_aux_channel *ch;
	struct page *page;
	int ret;

	page = alloc_page(GFP_KERNEL);
	if (!page)
		return -ENOMEM;

	ch = page_address(page);

	tzdev_print(0, "AUX Channel[%d] = 0x%p\n", cpu, ch);

	ret = tzdev_smc_connect_aux(page_to_pfn(page));
	if (ret) {
		__free_page(page);
		return ret;
	}

	aux_channel_init(aux_channel, cpu) = ch;

	return 0;
}

void *tz_iwio_alloc_iw_channel(unsigned int mode, unsigned int num_pages)
{
	void *buffer;
	struct page **pages;
	sk_pfn_t *pfns;
	struct tz_iwio_aux_channel *aux_ch;
	unsigned int i, j;
	unsigned long offset, num_pfns;
	unsigned long pfns_in_buf = TZ_IWIO_AUX_BUF_SIZE / sizeof(sk_pfn_t);

	pages = kcalloc(num_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		tzdev_print(0, "TZDev IW buffer pages allocation failed\n");
		return NULL;
	}

	pfns = kmalloc(num_pages * sizeof(sk_pfn_t), GFP_KERNEL);
	if (!pfns) {
		tzdev_print(0, "TZDev IW buffer pfns allocation failed\n");
		goto free_pages_arr;
	}

	/* Allocate non-contiguous buffer to reduce page allocator pressure */
	for (i = 0; i < num_pages; i++) {
		pages[i] = alloc_page(GFP_KERNEL);
		if (!pages[i]) {
			tzdev_print(0, "TZDev IW buffer creation failed\n");
			goto free_pfns_arr;
		}
		pfns[i] = page_to_pfn(pages[i]);
	}

	buffer = vmap(pages, num_pages, VM_MAP, PAGE_KERNEL);
	if (!buffer) {
		tzdev_print(0, "TZDev IW buffer mapping failed\n");
		goto free_pfns_arr;
	}

	/* Push PFNs list into aux channel */
	aux_ch = tz_iwio_get_aux_channel();

	for (offset = 0; offset < num_pages; offset += pfns_in_buf) {
		num_pfns = min(pfns_in_buf, num_pages - offset);

		memcpy(aux_ch->buffer, pfns + offset,
				num_pfns * sizeof(sk_pfn_t));

		if (tzdev_smc_connect(mode, num_pfns)) {
			tz_iwio_put_aux_channel();
			vunmap(buffer);
			tzdev_print(0, "TZDev IW buffer registration failed\n");
			goto free_pfns_arr;
		}
	}

	tz_iwio_put_aux_channel();

	kfree(pfns);
	kfree(pages);

	return buffer;

free_pfns_arr:
	kfree(pfns);

	for (j = 0; j < i; j++)
		__free_page(pages[j]);

free_pages_arr:
	kfree(pages);

	return NULL;
}
