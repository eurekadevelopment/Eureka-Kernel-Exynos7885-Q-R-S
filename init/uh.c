#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/cacheflush.h>
#include <asm/irqflags.h>
#include <linux/fs.h>
#include <asm/tlbflush.h>
#include <linux/init.h>
#include <asm/io.h>

#include <linux/uh.h>
#include <linux/uh_fault_handler.h>

#define UH_32BIT_SMC_CALL_MAGIC 0x82000400
#define UH_64BIT_SMC_CALL_MAGIC 0xC2000400

#define UH_STACK_OFFSET 0x2000

#define UH_MODE_AARCH32 0
#define UH_MODE_AARCH64 1

extern char _start_uh;
extern char _end_uh;
extern char _uh_disable;

int __init uh_disable(void)
{
	_uh_goto_EL2(UH_64BIT_SMC_CALL_MAGIC, (void *)virt_to_phys(&_uh_disable), UH_STACK_OFFSET, UH_MODE_AARCH64, NULL, 0);

	printk(KERN_ALERT "%s\n", __FUNCTION__);

	return 0;
}
extern void flush_cache_range(struct vm_area_struct *vma, unsigned long start, unsigned long end);
extern void flush_icache_range(unsigned long start, unsigned long end);
int __init uh_init(void)
{
	int status;
	void *uh_valias = _uh_map((phys_addr_t)UH_CODE_BASE);
	size_t uh_size = (size_t)(&_end_uh - &_start_uh);
	if(smp_processor_id() != 0) { return 0; }

	printk(KERN_ALERT "%s: bin 0x%p, 0x%x\n", __func__, &_start_uh, (int)uh_size);
	memcpy(uh_valias,  &_start_uh, uh_size);
	__flush_dcache_area(uh_valias, (unsigned long)uh_size);
	flush_icache_range((u64)uh_valias, (u64)(uh_valias + uh_size));

	status = _uh_goto_EL2(UH_64BIT_SMC_CALL_MAGIC, (void *)(UH_CODE_BASE + UH_TEXT_OFFSET),
				UH_STACK_OFFSET, UH_MODE_AARCH64, (void *)UH_CODE_BASE, UH_CODE_SIZE);

	printk(KERN_ALERT "uh add: %p, size: 0x%x, status: %x\n", (void *)UH_CODE_BASE, (int)UH_CODE_SIZE, status);
	uh_reserve_mem();
	uh_call(UH_APP_INIT, 0, uh_get_fault_handler(), kimage_voffset, 0, 0);
#ifdef CONFIG_UH_RKP
	uh_call(UH_APP_RKP, RKP_GET_RKP_GET_BUFFER_BITMAP, (u64)&rkp_s_bitmap_buffer, 0, 0, 0);
#endif
	return 0;
}
