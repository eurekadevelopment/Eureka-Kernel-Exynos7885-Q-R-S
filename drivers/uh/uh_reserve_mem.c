#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uh.h>
#ifdef CONFIG_UH_RKP
#include <linux/rkp.h>
#endif
#include "uh_reserve_mem.h"

#ifdef CONFIG_NO_BOOTMEM
#include <linux/memblock.h>
#endif

struct uh_mem_info {
#ifdef CONFIG_NO_BOOTMEM
	phys_addr_t start_add;
	phys_addr_t size;
#else
	unsigned long start_add;
	unsigned long size;
#endif
} uh_mem[UH_MAXNUM_MEM] __initdata;

static int __init fill_uh_mem_info(void)
{
	int cnt = 0;

	uh_mem[cnt].start_add 	= EL2_START;
	uh_mem[cnt++].size 	= EL2_SIZE;
#ifdef CONFIG_UH_RKP
	do{
		u64 el2_heap_base = EL2_START + EL2_SIZE , el2_heap_size = 0;
		uh_call(UH_APP_INIT, 4, (u64)&el2_heap_base, (u64)&el2_heap_size, 0, 0);
		uh_mem[cnt].start_add 	= el2_heap_base;
		uh_mem[cnt++].size 	= el2_heap_size;
	}while(0);
#endif
	return cnt;
}

int __init uh_reserve_mem(void)
{
	int num, i = 0;
	init_dt_scan_uh_nodes();
	num = fill_uh_mem_info();

	for (i = 0; i < num; i++) {
#ifdef CONFIG_NO_BOOTMEM
		if (memblock_is_region_reserved(uh_mem[i].start_add, uh_mem[i].size) ||
		    memblock_reserve(uh_mem[i].start_add, uh_mem[i].size)) {
#else
		if (reserve_bootmem(uh_mem[i].start_add, uh_mem[i].size, BOOTMEM_EXCLUSIVE)) {
#endif
			pr_err("%s: uh failed reserving size 0x%x at base 0x%x\n",
				__func__, (unsigned int)uh_mem[i].size, (unsigned int)uh_mem[i].start_add);
			return -1;
		}
		pr_info("%s: base:0x%x, size:0x%x\n",
			__func__, (unsigned int)uh_mem[i].start_add, (unsigned int)uh_mem[i].size);
	}
	return 0;
}
