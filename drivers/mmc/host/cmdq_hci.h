/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef LINUX_MMC_CQ_HCI_H
#define LINUX_MMC_CQ_HCI_H
#include <linux/mmc/core.h>

/* registers */
/* version */
#define CQVER		0x00
/* capabilities */
#define CQCAP		0x04
/* configuration */
#define CQCFG		0x08
#define CQ_DCMD		0x00001000
#define CQ_TASK_DESC_SZ 0x00000100
#define CQ_ENABLE	0x00000001

/* control */
#define CQCTL		0x0C
#define CLEAR_ALL_TASKS 0x00000100
#define HALT		0x00000001

/* interrupt status */
#define CQIS		0x10
#define CQIS_HAC	(1 << 0)
#define CQIS_TCC	(1 << 1)
#define CQIS_RED	(1 << 2)
#define CQIS_TCL	(1 << 3)
#define CQIS_BRE	(1 << 4)

/* interrupt status enable */
#define CQISTE		0x14

/* interrupt signal enable */
#define CQISGE		0x18

/* interrupt coalescing */
#define CQIC		0x1C
#define CQIC_ENABLE	(1 << 31)
#define CQIC_RESET	(1 << 16)
#define CQIC_ICCTHWEN	(1 << 15)
#define CQIC_ICCTH(x)	((x & 0x1F) << 8)
#define CQIC_ICTOVALWEN (1 << 7)
#define CQIC_ICTOVAL(x) (x & 0x7F)

/* task list base address */
#define CQTDLBA		0x20

/* task list base address upper */
#define CQTDLBAU	0x24

/* door-bell */
#define CQTDBR		0x28

/* task completion notification */
#define CQTCN		0x2C

/* device queue status */
#define CQDQS		0x30

/* device pending tasks */
#define CQDPT		0x34

/* task clear */
#define CQTCLR		0x38

/* send status config 1 */
#define CQSSC1		0x40
/*
 * Value n means CQE would send CMD13 during the transfer of data block
 * BLOCK_CNT-n
 */
#define SEND_QSR_INTERVAL 0x70000
#define CQSSC1_CIT_EN	(1 << 20)

/* send status config 2 */
#define CQSSC2		0x44

/* response for dcmd */
#define CQCRDCT		0x48

/* response mode error mask */
#define CQRMEM		0x50
#define CQ_EXCEPTION	(1 << 6)

/* write protection violation */
#define WP_ERASE_SKIP	(1 << 15)
#define WP_EXCEPTION    (1 << 26)

/* task error info */
#define CQTERRI		0x54

/* CQTERRI bit fields */
#define CQ_RMECI	0x1F
#define CQ_RMETI	(0x1F << 8)
#define CQ_RMEFV	(1 << 15)
#define CQ_DTECI	(0x3F << 16)
#define CQ_DTETI	(0x1F << 24)
#define CQ_DTEFV	(1 << 31)

#define GET_CMD_ERR_TAG(__r__) ((__r__ & CQ_RMETI) >> 8)
#define GET_DAT_ERR_TAG(__r__) ((__r__ & CQ_DTETI) >> 24)

/* command response index */
#define CQCRI		0x58

/* command response argument */
#define CQCRA		0x5C

/* CQCMD */
#define CQCMD44		0x100
#define CQCMD45		0x104
#define CQCMD46		0x108
#define CQCMD47		0x10C
#define CQCMD13		0x110

/* Debug */
#define CQDEBUG0       0x118
#define CQDEBUG1       0x11c

#define CQ_INT_ALL	0x17

/* Int mask */
#define CQDATAINTMASK1	0x124
#define CQCMDINTMASK2	0x128

#define CQIC_DEFAULT_ICCTH 31
#define CQIC_DEFAULT_ICTOVAL 1

/* attribute fields */
#define VALID(x)	((x & 1) << 0)
#define END(x)		((x & 1) << 1)
#define INT(x)		((x & 1) << 2)
#define ACT(x)		((x & 0x7) << 3)

/* data command task descriptor fields */
#define FORCED_PROG(x)	((x & 1) << 6)
#define CONTEXT(x)	((x & 0xF) << 7)
#define DATA_TAG(x)	((x & 1) << 11)
#define DATA_DIR(x)	((x & 1) << 12)
#define PRIORITY(x)	((x & 1) << 13)
#define QBAR(x)		((x & 1) << 14)
#define REL_WRITE(x)	((x & 1) << 15)
#define BLK_COUNT(x)	((x & 0xFFFF) << 16)
#define BLK_ADDR(x)	((x & 0xFFFFFFFF) << 32)

/* direct command task descriptor fields */
#define CMD_INDEX(x)	((x & 0x3F) << 16)
#define CMD_TIMING(x)	((x & 1) << 22)
#define RESP_TYPE(x)	((x & 0x3) << 23)

/* transfer descriptor fields */
#define DAT_LENGTH(x)	((x & 0xFFFF) << 16)
#define DAT_ADDR_LO(x)	((x & 0xFFFFFFFF) << 32)
#define DAT_ADDR_HI(x)	((x & 0xFFFFFFFF) << 0)

#define CQ_VENDOR_CFG	0x100
#define CMDQ_SEND_STATUS_TRIGGER (1 << 31)

#define NUM_SLOTS 32

/* CMD Interrupt MASK 1 */
#define RESP_ERR		(1 << 0)
#define CMD_DONE		(1 << 1)
#define RESP_CRC_ERR		(1 << 2)
#define RESP_TIMEOUT		(1 << 3)
#define HW_LOCK_ERR		(1 << 4)

/* Data Interrupt MASK */
#define DATA_DONE		(1 << 0)
#define DATA_CRC_ERR		(1 << 1)
#define DATA_RTIMEOUT		(1 << 2)
#define HOST_TIMEOUT		(1 << 3)
#define FIFO_UNDERRUN		(1 << 4)
#define START_BIT_ERR		(1 << 5)
#define END_BIT_ERR		(1 << 6)

/* Device Reseponse */
#define RESP_DEVICE_STATE	0xFDF9A080

struct cmdq_host {
	const struct cmdq_host_ops *ops;
	void __iomem *mmio;
	struct mmc_host *mmc;

	/* 64 bit DMA */
	bool dma64;
	u8 num_slots;

	u32 dcmd_slot;
	u32 caps;
#define CMDQ_TASK_DESC_SZ_128 0x1

	u32 quirks;
#define CMDQ_QUIRK_SHORT_TXFR_DESC_SZ 0x1
#define CMDQ_QUIRK_NO_DCMD	0x2

	bool enabled;
	bool halted;
	bool init_done;
	bool halt_failed;
	bool sw_reset;

	u32 cnt_recovery;
	u32 cnt_recovery_halt_pass;
	u32 cnt_recovery_halt_fail;

	u8 *desc_base;

	/* total descriptor size */
	u8 slot_sz;

	/* 64/128 bit depends on CQCFG */
	u8 task_desc_len;

	/* 64 bit on 32-bit arch, 128 bit on 64-bit */
	u8 link_desc_len;

	u8 *trans_desc_base;
	/* same length as transfer descriptor */
	u8 trans_desc_len;

	dma_addr_t desc_dma_base;
	dma_addr_t trans_desc_dma_base;

	struct completion halt_comp;
	struct mmc_request **mrq_slot;
	u32 cmd_log_idx[NUM_SLOTS];
	void *private;

	struct list_head	active_mrq;
	spinlock_t lock;
	spinlock_t list_lock;
	struct cmdq_sfr_ramdump		*cq_dump;
};

struct cmdq_sfr_ramdump {
	u32 cqver;
	u32 cqcap;
	u32 cqcfg;
	u32 cqctl;
	u32 cqis;
	u32 cqiste;
	u32 cqisge;
	u32 cqic;
	u32 cqtlba;
	u32 cqtlbau;
	u32 cqtdbr;
	u32 cqtcn;
	u32 cqdqs;
	u32 cqdpt;
	u32 cqtclr;
	u32 cqssc1;
	u32 cqssc2;
	u32 cqrdct;
	u32 cqrmem;
	u32 cqterri;
	u32 cqcri;
	u32 cqcra;
	u32 cqcmd;
	u32 cqdebug0;
	u32 cqdebug1;
	u32 cqdataintmask1;
	u32 cqdataintmask2;
};

enum dw_mci_cq_log_cmd {
	CQ_LOG_CMD_READ = 1,
	CQ_LOG_CMD_WRITE,
	CQ_LOG_CMD_DISCARD,
	CQ_LOG_CMD_FLUSH,
};

struct cmdq_log_ctx {
	u32	idx;

	u32	x0;	/* data0: tag, data1: tag */
	u32	x1;	/* data0: dbr, data1: dbr */
	u32	x2;	/* data0: cmd, data1:  */
	u32	x3;	/* data0: lba, data1:  */
	u32	x4;	/* data0: sct, data1:  */
};

struct cmdq_host_ops {
	void (*set_tranfer_params)(struct mmc_host *mmc);
	void (*set_data_timeout)(struct mmc_host *mmc, u32 val);
	void (*clear_set_irqs)(struct mmc_host *mmc, bool clear);
	void (*set_block_size)(struct mmc_host *mmc);
	void (*dump_vendor_regs)(struct mmc_host *mmc);
	void (*write_l)(struct cmdq_host *host, u32 val, int reg);
	u32 (*read_l)(struct cmdq_host *host, int reg);
	void (*clear_set_dumpregs)(struct mmc_host *mmc, bool set);
	void (*enhanced_strobe_mask)(struct mmc_host *mmc, bool set);
	int (*reset)(struct mmc_host *mmc);
	int (*crypto_engine_cfg)(struct mmc_host *mmc, void *desc,
				struct mmc_data *data, struct page *page,
				int sector_offset, bool cmdq_enabled);
	int (*crypto_engine_clear)(struct mmc_host *mmc, void *desc,
				bool cmdq_enabled);
	void (*cmdq_log)(struct mmc_host *mmc, bool new_cmd,
					struct cmdq_log_ctx *log_ctx);
	void (*crypto_cfg_reset)(struct mmc_host *mmc, unsigned int slot);
	void (*hwacg_control)(struct mmc_host *mmc, bool set);
	int (*hwacg_control_direct)(struct mmc_host *mmc, bool set);
	void (*post_cqe_halt)(struct mmc_host *mmc);
	bool (*busy_waiting)(struct mmc_host *mmc, struct mmc_request *mrq);
	void (*pm_qos_lock)(struct mmc_host *mmc, bool set);
	void (*int_mask_set)(struct mmc_host *mmc, bool enable);
	void (*sicd_control)(struct mmc_host *mmc, bool enable);
	void (*transferred_cnt)(struct mmc_host *mmc, struct mmc_request *mrq);
	void (*resume_skip)(struct mmc_host *mmc);
};

static inline void cmdq_writel(struct cmdq_host *host, u32 val, int reg)
{
	if (unlikely(host->ops->write_l))
		host->ops->write_l(host, val, reg);
	else
		writel_relaxed(val, host->mmio + reg);
}

static inline u32 cmdq_readl(struct cmdq_host *host, int reg)
{
	if (unlikely(host->ops->read_l))
		return host->ops->read_l(host, reg);
	else
		return readl_relaxed(host->mmio + reg);
}

extern irqreturn_t cmdq_irq(struct mmc_host *mmc, int err);
extern int cmdq_init(struct cmdq_host *cq_host, struct mmc_host *mmc,
		     bool dma64);
extern int cmdq_free(struct cmdq_host *cq_host);
extern struct cmdq_host *cmdq_pltfm_init(struct platform_device *pdev);
extern void cmdq_dumpregs(struct cmdq_host *cq_host);
#endif
