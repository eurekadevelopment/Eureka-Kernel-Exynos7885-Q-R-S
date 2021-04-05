#ifndef _SCORE_COMMON_H
#define _SCORE_COMMON_H

#include <linux/sizes.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>

#include "score-config.h"
#include "score-regs.h"

#define STATIC_DEBUG
#ifdef STATIC_DEBUG
# define STATIC
#else
# define STATIC static
#endif

#if 0
#define SCORE_DEBUG
#ifdef SCORE_DEBUG
#define score_debug(fmt, args...)		\
	do {					\
		printk("%s: " fmt,		\
			__func__, ##args);	\
	} while(0)
#else
#define score_debug(fmt, args...)
#endif
#endif

#define VLIW_NAME	"PMw.bin"
#define DATA_NAME	"DMb.bin"

#define SCORE_DEV_MINOR	(250)

#define SFR_ADDR_PA		0x80000000

#define SFR_SIZE		(32 * 1024)
#define PMEM_SIZE		(32 * 1024)
#define DMEM_SIZE		(32 * 1024)

#define SFR_BASE		(0)
#define PMEM_BASE		(SFR_BASE+SFR_SIZE)
#define DMEM_BASE		(SFR_BASE+SFR_SIZE+PMEM_SIZE)

/* @@@ temporary size */
#define PMEM_CACHE_SIZE		(SZ_1M * 5)
#define DMEM_CACHE_SIZE		(SZ_1M * 5)

#define SCORE_INTERRUPT					(0x0008)
/* HOST Command Done Interrupt, 0x0 : No Interrupt, 0x1 : CA7 Interrupt */
#define SCORE_CLOCK_EN					(0x0044)
/* score clock enable, 0x1 : Core Enable, 0x2 : DMA Enable */
#define SCORE_CACHE_MGMT				(0x0048)
/* 0x1 : D-Cache Flush, 0x2 : D-Cache Invalidate, 0x4 : I-Cache Invalidate */
#define SCORE_SRAM_MODE					(0x004C)
/* 0x0 : Cache Mode, 0x1 : SRAM Mode, 0x3 : SRAM Upload Mode */
#define IRQ_SCORE		61

#define score_read_reg(offset)		readl(score_fw_device->sfr + (offset))
#define score_write_reg(data, offset)	writel((data), score_fw_device->sfr + (offset))

#define SCORE_READ_AREG(offset)		(score_fw_device->sfr + (offset))
#define SCORE_READ_REG(offset)		score_read_reg(offset)
#define SCORE_WRITE_REG(data, offset)	score_write_reg(data,offset)

#define SCORE_CACHE_OFFSET(va)		\
	(virt_to_phys(va) - SCORE_READ_REG(SCORE_DATA_START_ADDR))

enum score_fw_queue_type {
	SCORE_IN_QUEUE,
	SCORE_OUT_QUEUE
};

/* TODO: time out */
/* #define SCORE_WAIT_TIMEOUT 		(5 * HZ) */	/* 5 seconds */
#define SCORE_WAIT_TIMEOUT		(100 * HZ)	/* 100 seconds */

#define SCORE_WAIT_STATE		(1)
struct score_fw_wait_task {
	unsigned int task_id;
	unsigned int result;
	unsigned long wait_state;
	wait_queue_head_t wait;
	struct list_head wait_queue;
};

#define SCORE_KERNEL_MAX_PARAM  (42)
#define SCORE_MAX_RESULT  (3)

/* TODO:merge with ScoreIpcPacket of host */
/* @ { */
#define SCORE_MAX_GROUP   (2)
#define NUM_OF_GRP_PARAM  (26)

struct score_packet_group_data {
	unsigned int params[NUM_OF_GRP_PARAM];
};

struct score_packet_group_header {
	unsigned int valid_size : 6;
	unsigned int fd_bitmap  : 26;
};

struct score_packet_group {
	struct score_packet_group_header header;
	struct score_packet_group_data data;
};

struct score_packet_header {
	unsigned int queue_id    : 2;
	unsigned int kernel_name : 12;
	unsigned int task_id     : 7;
	unsigned int reserved    : 8;
	unsigned int worker_name : 3;
};
struct score_packet_size {
	unsigned int packet_size : 14;
	unsigned int reserved    : 10;
	unsigned int group_count : 8;
};

struct score_ipc_packet {
	struct score_packet_size size;
	struct score_packet_header header;
	struct score_packet_group group[0];
};
/* @} */

struct score_fw_param_list {
	struct score_ipc_packet *p;
	struct list_head p_list;
};

struct score_fw_pending_param {
	struct mutex		lock;
	struct list_head	param_list;
	struct work_struct	param_work;
};

struct score_fw_queue {
	enum score_fw_queue_type   type;
	unsigned int		head_info;
	unsigned int		tail_info;
	unsigned int		start;
	unsigned int		size;
	atomic_t		task_id;
	struct mutex		lock;
	spinlock_t		slock;
	struct list_head	wait_list;
	struct score_fw_pending_param pending_param;
};

/* For SCORE firmware */
struct score_fw_info {
	const struct firmware *vliw;		/* VLIW */
	const struct firmware *data;		/* DATA */

	unsigned char *base_va;			/* Virtual address of base */
	unsigned char *vliw_va;			/* Virtual address of VLIW */
	unsigned char *data_va;			/* Virtual address of DATA */
	unsigned char *data_va_temp;		/* Virtual address of DATA, for SRAM mode */
	unsigned int base_pa;			/* Physical address of base */
	unsigned int vliw_pa;			/* Physical address of VLIW */
	unsigned int data_pa;			/* Physical address of DATA */

	unsigned long vliw_size;		/* Size of VLIW */
	unsigned long data_size;		/* Size of DATA */
};

struct score_fw_dev {
	struct platform_device	*pdev;
	struct score_fw_info	fw_info;
	struct clk		*clk;

	unsigned int base;
	unsigned long sfr_size;			/* score control register */
	unsigned long pmem_size;		/* score_vliw.bin */
	unsigned long dmem_size;		/* score_data.bin */

	void __iomem	*sfr;
	void __iomem	*pmem;
	void __iomem	*dmem;

	struct score_fw_queue *in_queue;
	struct score_fw_queue *out_queue;

	struct workqueue_struct *param_wq;
};

/* HACK */
#if 1
struct data_buf_type {
  unsigned int	sc_type  : 8;
  unsigned int	plane0   : 6;
  unsigned int	plane1   : 6;
  unsigned int	plane2   : 6;
  unsigned int	plane3   : 6;
};
#endif
#if 0
#pragma pack(push,4)
struct sc_buffer {
	unsigned int			width;
	unsigned int			height;
	struct data_buf_type		type;
	unsigned int			memory_type;
	union {
		int			fd;
		unsigned int		addr;
		/* HACK */
		/* unsigned long		addr; */
		/* unsigned long long	dummy; */
	} memory;
};
#pragma pack(pop)
#else
struct sc_buffer {
	unsigned int width;
	unsigned int height;
	unsigned int type;
	unsigned int addr;
};

#pragma pack(push,4)
struct sc_host_buffer {
	int                memory_type;
	int                fd;
	unsigned int       addr32;
	unsigned long long addr64;
};
#pragma pack(pop)

struct sc_packet_buffer {
	struct sc_buffer      buf;
	struct sc_host_buffer host_buf;
};
#endif
#endif
