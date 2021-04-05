/*
 * PGD allocation/freeing
 *
 * Copyright (C) 2012 ARM Ltd.
 * Author: Catalin Marinas <catalin.marinas@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/mm.h>
#include <linux/gfp.h>
#include <linux/highmem.h>
#include <linux/slab.h>

#include <asm/pgalloc.h>
#include <asm/page.h>
#include <asm/tlbflush.h>

#include "mm.h"

#ifdef CONFIG_UH_RKP
#include <linux/uh.h>
#include <linux/rkp.h>
#endif

static struct kmem_cache *pgd_cache;
#ifndef CONFIG_UH_RKP
pgd_t *pgd_alloc(struct mm_struct *mm)
{
	if (PGD_SIZE == PAGE_SIZE)
		return (pgd_t *)__get_free_page(PGALLOC_GFP);
	else
		return kmem_cache_alloc(pgd_cache, PGALLOC_GFP);
}
#else
pgd_t *pgd_alloc(struct mm_struct *mm)
{
	pgd_t *ret = NULL;
#ifdef CONFIG_KNOX_KAP
	if (boot_mode_security)
#endif
		ret = (pgd_t *) rkp_ro_alloc();
	if (!ret) {
		if (PGD_SIZE == PAGE_SIZE)
			ret = (pgd_t *)__get_free_page(PGALLOC_GFP);
		else
			ret = kmem_cache_alloc(pgd_cache, PGALLOC_GFP);
		}

	if(unlikely(!ret)) {
		pr_warn("%s: pgd alloc is failed\n", __func__);
		return ret;
	}
#ifdef CONFIG_KNOX_KAP
	if (boot_mode_security)
#endif
		uh_call(UH_APP_RKP, RKP_NEW_PGD, (u64)ret, 0, 0, 0);
	return ret;
}
#endif
#ifndef CONFIG_UH_RKP
void pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
	if (PGD_SIZE == PAGE_SIZE)
		free_page((unsigned long)pgd);
	else
		kmem_cache_free(pgd_cache, pgd);
}
#else
void pgd_free(struct mm_struct *mm, pgd_t *pgd)
{
	int rkp_do = 0;
#ifdef CONFIG_KNOX_KAP
	if (boot_mode_security)
#endif
		rkp_do = 1;
	if (rkp_do)
		uh_call(UH_APP_RKP, RKP_FREE_PGD, (u64)pgd, 0, 0, 0);
	/* if pgd memory come from read only buffer, the put it back */
	/*TODO: use a macro*/
	if (rkp_do && is_rkp_ro_page((u64)pgd))
		rkp_ro_free((void*)pgd);
	else
	{
		if (PGD_SIZE == PAGE_SIZE)
			free_page((unsigned long)pgd);
		else
			kmem_cache_free(pgd_cache, pgd);
	}
}
#endif
void __init pgd_cache_init(void)
{
	if (PGD_SIZE == PAGE_SIZE)
		return;

	/*
	 * Naturally aligned pgds required by the architecture.
	 */
	pgd_cache = kmem_cache_create("pgd_cache", PGD_SIZE, PGD_SIZE,
				      SLAB_PANIC, NULL);
}
