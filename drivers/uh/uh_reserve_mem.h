#ifndef _UH_MEM_H
#define _UH_MEM_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/highmem.h>
#include <linux/uh.h>
#include <linux/rkp.h>

#ifdef CONFIG_NO_BOOTMEM
#include <linux/memblock.h>
#endif

#ifdef CONFIG_UH_RKP
#define UH_MAXNUM_MEM	UH_NUM_MEM + RKP_NUM_MEM
#else
#define UH_MAXNUM_MEM	UH_NUM_MEM
#endif

#ifdef CONFIG_NO_BOOTMEM
typedef phys_addr_t uh_mem_t;
#else
typedef unsigned long uh_mem_t;
#endif

#endif /* _UH_MEM_H */
