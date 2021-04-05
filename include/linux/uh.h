#ifndef __UH_H__
#define __UH_H__

#ifndef __ASSEMBLY__

/* For uH Command */
#define	APP_INIT	0
#define	APP_SAMPLE	1
#define APP_RKP		2

#define UH_PREFIX  UL(0xc300c000)
#define UH_APPID(APP_ID)  ((UL(APP_ID) & UL(0xFF)) | UH_PREFIX)

enum __UH_APP_ID {
	UH_APP_INIT 	= UH_APPID(APP_INIT),
	UH_APP_SAMPLE 	= UH_APPID(APP_SAMPLE),
	UH_APP_RKP 	= UH_APPID(APP_RKP),
};

/* For uH Memory */
#define UH_NUM_MEM		0x02

#define EL2_START			(0xb0600000ULL)
#define EL2_SIZE			(0x200000ULL)

#define UH_DEBUG_LOG_SIZE		(0x40000ULL)
#define UH_DEBUG_LOG_ADDR	(EL2_START)
#define UH_CODE_SIZE			(0x100000ULL)
#define UH_CODE_BASE			(EL2_START + 0x100000)
#define UH_BIGDATA_SIZE			(960)
#define UH_BIGDATA_START		(UH_CODE_BASE + UH_CODE_SIZE - UH_BIGDATA_SIZE)

#define UH_LOG_START	UH_DEBUG_LOG_ADDR
#define UH_LOG_SIZE		UH_DEBUG_LOG_SIZE

#define UH_TEXT_OFFSET			(0x1000ULL)

#ifdef CONFIG_KNOX_KAP
extern int boot_mode_security;
#endif

struct test_case_struct {
	int (* fn)(void); //test case func
	char * describe;
};

int uh_init(void);
int uh_disable(void);
int _uh_goto_EL2(int magic, void *label, int offset, int mode, void *base, int size);
unsigned long long uh_call(u64 app_id, u64 command, u64 arg0, u64 arg1, u64 arg2, u64 arg3);
void * _uh_map(phys_addr_t uh_phys);
void init_dt_scan_uh_nodes(void);
int uh_reserve_mem(void);

#endif //__ASSEMBLY__
#endif //__UH_H__
