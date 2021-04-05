#include <linux/uh.h>

#define uh_ELF_PATH "init/uh.elf"

#define SMC_64BIT_RET_MAGIC 0xC2000401

.section .init.text, "ax"
.global _uh_goto_EL2
_uh_goto_EL2:
	smc #0
	isb
	ret

.global _uh_disable
_uh_disable:
	ldr x0, =SMC_64BIT_RET_MAGIC
	smc #0
	isb

.global _start_uh
_start_uh:
.incbin uh_ELF_PATH
.global _end_uh
_end_uh:
.section .text
