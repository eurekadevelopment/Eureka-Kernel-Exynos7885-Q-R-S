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

#include <linux/delay.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include <linux/blkdev.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>

#include <crypto/fmp.h>

#include "cmdq_hci.h"

#define DCMD_SLOT 31

#if CQ_DBG
static int tcc_miss_period;
#endif

/* 1 sec */
#define HALT_TIMEOUT_MS 1000

#ifdef CONFIG_PM_RUNTIME
static int cmdq_runtime_pm_get(struct cmdq_host *host)
{
	return pm_runtime_get_sync(host->mmc->parent);
}
static int cmdq_runtime_pm_put(struct cmdq_host *host)
{
	pm_runtime_mark_last_busy(host->mmc->parent);
	return pm_runtime_put_autosuspend(host->mmc->parent);
}
#else
static inline int cmdq_runtime_pm_get(struct cmdq_host *host)
{
	return 0;
}
static inline int cmdq_runtime_pm_put(struct cmdq_host *host)
{
	return 0;
}
#endif
static inline struct mmc_request *get_req_by_tag(struct cmdq_host *cq_host,
					  unsigned int tag)
{
	return cq_host->mrq_slot[tag];
}

static inline u8 *get_desc(struct cmdq_host *cq_host, u8 tag)
{
	return cq_host->desc_base + (tag * cq_host->slot_sz);
}

static inline u8 *get_link_desc(struct cmdq_host *cq_host, u8 tag)
{
	u8 *desc = get_desc(cq_host, tag);

	return desc + cq_host->task_desc_len;
}

static inline dma_addr_t get_trans_desc_dma(struct cmdq_host *cq_host, u8 tag)
{
	return cq_host->trans_desc_dma_base +
		(cq_host->trans_desc_len * cq_host->mmc->max_segs * tag);
}

static inline u8 *get_trans_desc(struct cmdq_host *cq_host, u8 tag)
{
	return cq_host->trans_desc_base +
		(cq_host->trans_desc_len * cq_host->mmc->max_segs * tag);
}

static void setup_trans_desc(struct cmdq_host *cq_host, u8 tag)
{
	u8 *link_temp;
	dma_addr_t trans_temp;

	link_temp = get_link_desc(cq_host, tag);
	trans_temp = get_trans_desc_dma(cq_host, tag);

	memset(link_temp, 0, cq_host->link_desc_len);
	if (cq_host->link_desc_len > 8)
		*(link_temp + 8) = 0;

	if (tag == DCMD_SLOT) {
		*link_temp = VALID(0) | ACT(0) | END(1);
		return;
	}

	*link_temp = VALID(1) | ACT(0x6) | END(0);

	if (cq_host->dma64) {
		__le64 *data_addr = (__le64 __force *)(link_temp + 4);
		data_addr[0] = cpu_to_le64(trans_temp);
	} else {
		__le32 *data_addr = (__le32 __force *)(link_temp + 4);
		data_addr[0] = cpu_to_le32(trans_temp);
	}
}

static void cmdq_interrupt_mask_set(struct cmdq_host *cq_host, bool enable)
{
	u32 data_mask;
	u32 cmd_mask;
	u32 err_mask;

	data_mask = cmdq_readl(cq_host, CQDATAINTMASK1);
	cmd_mask = cmdq_readl(cq_host, CQCMDINTMASK2);
	err_mask = cmdq_readl(cq_host, CQRMEM);

	if (enable) {
		data_mask |= (DATA_DONE | DATA_CRC_ERR | DATA_RTIMEOUT |
			HOST_TIMEOUT | FIFO_UNDERRUN | START_BIT_ERR | END_BIT_ERR);
		cmd_mask |= (RESP_ERR | CMD_DONE | RESP_CRC_ERR |
			RESP_TIMEOUT | HW_LOCK_ERR);
		err_mask = RESP_DEVICE_STATE;
		/* disable write protection violation indication */
		err_mask &= ~(WP_EXCEPTION | WP_ERASE_SKIP);
	} else {
		data_mask &= ~(DATA_DONE | DATA_CRC_ERR | DATA_RTIMEOUT |
			HOST_TIMEOUT | FIFO_UNDERRUN | START_BIT_ERR | END_BIT_ERR);
		cmd_mask &= ~(RESP_ERR | CMD_DONE | RESP_CRC_ERR |
			RESP_TIMEOUT | HW_LOCK_ERR);
		err_mask = 0;
	}

	cmdq_writel(cq_host, data_mask, CQDATAINTMASK1);
	cmdq_writel(cq_host, cmd_mask, CQCMDINTMASK2);
	cmdq_writel(cq_host, err_mask, CQRMEM);
}

static void cmdq_clear_set_irqs(struct cmdq_host *cq_host, u32 clear, u32 set)
{
	u32 ier;

	ier = cmdq_readl(cq_host, CQISTE);
	ier &= ~clear;
	ier |= set;
	cmdq_writel(cq_host, ier, CQISTE);
	cmdq_writel(cq_host, ier, CQISGE);
	/* ensure the writes are done */
	mb();
}


#define DRV_NAME "mmc-cmdq-host"

void cmdq_host_debug_parser(u32 reg)
{
	static char * const debug0[] = {
		"CTRL_IDLE",			/* bit 0 */
		"CTRL_DCMD",
		"CTRL_CMD44_AND_CMD45",
		"CTRL_CMD13",
		"CTRL_CMD46_OR_CMD47",
		"CTRL_PRE_HALT",		/* bit 5 */
		"CTRL_HALT",
		"DATA_IDLE",
		"DATA_WAIT_DTO",
		"DATA_DTO_CLEAR_REQ",
		"DATA_DTO_CLEAR_GRANT",		/* bit 10 */
		"DATA_DTO_CLEAR",
		"DATA_DTO_CLEAR_WRITE_DONE",
		"DATA_WBUSY_CHECK",
		"SEQSUB_IDLE",
		"SEQSUB_SFR",			/* bit 15 */
		"SEQSUB_CMD",
		"SFR_IDLE",
		"SFR_REQ",
		"SFR_GRANT",
		"SFR_WRITE",			/* bit 20 */
		"SFR_WRITE_DONE",
		"CMD_DONE",
		"CMD_DONE_CLEAR_WRITE_REQ",
		"CMD_DONE_CLEAR_WRITE_GRANT",
		"CMD_DONE_CLEAR_WRITE",		/* bit 25 */
		"CMD_DONE_CLEAR_WRITE_DONE",
		"CMD_DONE_BUSY_CHECK"
	};

	if (unlikely(reg == 0)) {
		pr_err("Debug SFR is empty!!!");
		return;
	}

	if (reg & (0x7F))
		pr_info(DRV_NAME ": CQE_MAIN STATEMACHNE   : %s\n", debug0[ffs(reg & (0x7F)) - 1]);
	if (reg & (0x7F << 7))
		pr_info(DRV_NAME ": CQE_DATA STATEMACHNE   : %s\n", debug0[ffs(reg & (0x7F << 7)) - 1]);
	if (reg & (0x7 << 14))
		pr_info(DRV_NAME ": CQE_SEQSUB STATEMACHNE : %s\n", debug0[ffs(reg & (0x7 << 14)) - 1]);
	if (reg & (0x3FF << 17))
		pr_info(DRV_NAME ": CQE_SFR STATEMACHNE    : %s\n", debug0[ffs(reg & (0x3FF << 17)) - 1]);
}

void cmdq_dumpregs(struct cmdq_host *cq_host)
{
	struct mmc_host *mmc = cq_host->mmc;
	u32 reg = 0;

	pr_err(DRV_NAME ": ========== REGISTER DUMP (%s)==========\n",
		mmc_hostname(mmc));

	pr_err(DRV_NAME ": Caps: 0x%08x		  | Version:  0x%08x\n",
		cq_host->cq_dump->cqcap = cmdq_readl(cq_host, CQCAP),
		cq_host->cq_dump->cqver = cmdq_readl(cq_host, CQVER));
	pr_err(DRV_NAME ": Queing config: 0x%08x  | Queue Ctrl:  0x%08x\n",
		cq_host->cq_dump->cqcfg = cmdq_readl(cq_host, CQCFG),
		cq_host->cq_dump->cqctl = cmdq_readl(cq_host, CQCTL));
	pr_err(DRV_NAME ": Int stat: 0x%08x	  | Int enab:  0x%08x\n",
		cq_host->cq_dump->cqis = cmdq_readl(cq_host, CQIS),
		cq_host->cq_dump->cqiste = cmdq_readl(cq_host, CQISTE));
	pr_err(DRV_NAME ": Int sig: 0x%08x	  | Int Coal:  0x%08x\n",
		cq_host->cq_dump->cqisge = cmdq_readl(cq_host, CQISGE),
		cq_host->cq_dump->cqic = cmdq_readl(cq_host, CQIC));
	pr_err(DRV_NAME ": TDL base: 0x%08x	  | TDL up32:  0x%08x\n",
		cq_host->cq_dump->cqtlba = cmdq_readl(cq_host, CQTDLBA),
		cq_host->cq_dump->cqtlbau = cmdq_readl(cq_host, CQTDLBAU));
	pr_err(DRV_NAME ": Doorbell: 0x%08x	  | Comp Notif:  0x%08x\n",
		cq_host->cq_dump->cqtdbr = cmdq_readl(cq_host, CQTDBR),
		cq_host->cq_dump->cqtcn = cmdq_readl(cq_host, CQTCN));
	pr_err(DRV_NAME ": Dev queue: 0x%08x	  | Dev Pend:  0x%08x\n",
		cq_host->cq_dump->cqdqs = cmdq_readl(cq_host, CQDQS),
		cq_host->cq_dump->cqdpt = cmdq_readl(cq_host, CQDPT));
	pr_err(DRV_NAME ": Task clr: 0x%08x	  | Send stat 1:  0x%08x\n",
		cq_host->cq_dump->cqtclr = cmdq_readl(cq_host, CQTCLR),
		cq_host->cq_dump->cqssc1 = cmdq_readl(cq_host, CQSSC1));
	pr_err(DRV_NAME ": Send stat 2: 0x%08x	  | DCMD resp:  0x%08x\n",
		cq_host->cq_dump->cqssc2 = cmdq_readl(cq_host, CQSSC2),
		cq_host->cq_dump->cqrdct = cmdq_readl(cq_host, CQCRDCT));
	pr_err(DRV_NAME ": Resp err mask: 0x%08x  | Task err:  0x%08x\n",
		cq_host->cq_dump->cqrmem = cmdq_readl(cq_host, CQRMEM),
		cq_host->cq_dump->cqterri = cmdq_readl(cq_host, CQTERRI));
	pr_err(DRV_NAME ": Resp idx 0x%08x	  | Resp arg:  0x%08x\n",
		cq_host->cq_dump->cqcri = cmdq_readl(cq_host, CQCRI),
		cq_host->cq_dump->cqcra = cmdq_readl(cq_host, CQCRA));
	pr_info(DRV_NAME ": Debug0 0x%08x\n",
			cq_host->cq_dump->cqdebug0 = cmdq_readl(cq_host, CQDEBUG0));
	pr_info(DRV_NAME ": Debug1 0x%08x\n",
			cq_host->cq_dump->cqdebug1 = cmdq_readl(cq_host, CQDEBUG1));
	pr_info(DRV_NAME ": CQCMD44 0x%08x\n", cmdq_readl(cq_host, CQCMD44));
	pr_info(DRV_NAME ": CQCMD45 0x%08x\n", cmdq_readl(cq_host, CQCMD45));
	pr_info(DRV_NAME ": CQCMD46 0x%08x\n", cmdq_readl(cq_host, CQCMD46));
	pr_info(DRV_NAME ": CQCMD47 0x%08x\n", cmdq_readl(cq_host, CQCMD47));
	pr_info(DRV_NAME ": CQCMD13 0x%08x\n", cmdq_readl(cq_host, CQCMD13));
	pr_info(DRV_NAME ": DATA_INTMASK 0x%08x\n",
			cq_host->cq_dump->cqdataintmask1 =
			cmdq_readl(cq_host, CQDATAINTMASK1));
	pr_info(DRV_NAME ": CMD_INTMASK 0x%08x\n",
			cq_host->cq_dump->cqdataintmask2 =
			cmdq_readl(cq_host, CQCMDINTMASK2));
	pr_info(DRV_NAME ": ===========================================\n");

	pr_info(DRV_NAME ": <<<<<<<<<< Debug0 Parsing >>>>>>>>>>\n");
	reg = cmdq_readl(cq_host, CQDEBUG0);
	cmdq_host_debug_parser(reg);

	if (cq_host->ops->dump_vendor_regs)
		cq_host->ops->dump_vendor_regs(mmc);

#if defined(CONFIG_MMC_TEST_MODE)
		/* do not recover system if test mode is enabled */
		BUG();
#endif
}

/**
 * The allocated descriptor table for task, link & transfer descritors
 * looks like:
 * |----------|
 * |task desc |  |->|----------|
 * |----------|  |  |trans desc|
 * |link desc-|->|  |----------|
 * |----------|          .
 *      .                .
 *  no. of slots      max-segs
 *      .           |----------|
 * |----------|
 * The idea here is to create the [task+trans] table and mark & point the
 * link desc to the transfer desc table on a per slot basis.
 */
static int cmdq_host_alloc_tdl(struct cmdq_host *cq_host)
{

	size_t desc_size;
	size_t data_size;
	int i = 0;

	/* task descriptor can be 64/128 bit irrespective of arch */
	if (cq_host->caps & CMDQ_TASK_DESC_SZ_128) {
		cmdq_writel(cq_host, cmdq_readl(cq_host, CQCFG) |
			       CQ_TASK_DESC_SZ, CQCFG);
		cq_host->task_desc_len = 16;
	} else {
		cq_host->task_desc_len = 8;
	}

	/*
	 * 96 bits length of transfer desc instead of 128 bits which means
	 * ADMA would expect next valid descriptor at the 96th bit
	 * or 128th bit
	 */
	if (cq_host->dma64) {
		if (cq_host->quirks & CMDQ_QUIRK_SHORT_TXFR_DESC_SZ)
			cq_host->trans_desc_len = 12;
		else
			cq_host->trans_desc_len = 32 * TRANS_DESC_LEN_MULTIPLIER;
		cq_host->link_desc_len = 32 * TRANS_DESC_LEN_MULTIPLIER;
	} else {
		cq_host->trans_desc_len = 8;
		cq_host->link_desc_len = 8;
	}

	/* total size of a slot: 1 task & 1 transfer (link) */
	cq_host->slot_sz = cq_host->task_desc_len + cq_host->link_desc_len;

	desc_size = cq_host->slot_sz * cq_host->num_slots;

	data_size = cq_host->trans_desc_len * cq_host->mmc->max_segs *
		(cq_host->num_slots - 1);

	pr_info("%s: %s: desc_size: %d data_sz: %d slot-sz: %d\n task_desc_len: %d trans_desc_len %d link_desc_len %d\n",
			mmc_hostname(cq_host->mmc), __func__,
			(int)desc_size, (int)data_size, cq_host->slot_sz,
			cq_host->task_desc_len, cq_host->trans_desc_len, cq_host->link_desc_len);

	/*
	 * allocate a dma-mapped chunk of memory for the descriptors
	 * allocate a dma-mapped chunk of memory for link descriptors
	 * setup each link-desc memory offset per slot-number to
	 * the descriptor table.
	 */
	cq_host->desc_base = dmam_alloc_coherent(mmc_dev(cq_host->mmc),
						 desc_size,
						 &cq_host->desc_dma_base,
						 GFP_KERNEL);
	cq_host->trans_desc_base = dmam_alloc_coherent(mmc_dev(cq_host->mmc),
					      data_size,
					      &cq_host->trans_desc_dma_base,
					      GFP_KERNEL);
	if (!cq_host->desc_base || !cq_host->trans_desc_base)
		return -ENOMEM;

	pr_info("%s: desc-base: 0x%p trans-base: 0x%p\n desc_dma 0x%llx trans_dma: 0x%llx\n",
			mmc_hostname(cq_host->mmc),
			cq_host->desc_base, cq_host->trans_desc_base,
			(unsigned long long)cq_host->desc_dma_base,
			(unsigned long long) cq_host->trans_desc_dma_base);

	for (; i < (cq_host->num_slots); i++)
		setup_trans_desc(cq_host, i);

	return 0;
}

static int cmdq_enable(struct mmc_host *mmc)
{
	int err = 0;
	u32 cqcfg;
	bool dcmd_enable;
	struct cmdq_host *cq_host = mmc_cmdq_private(mmc);

	if (!cq_host || !mmc->card || !mmc_card_cmdq(mmc->card)) {
		err = -EINVAL;
		goto out;
	}

	if (cq_host->enabled)
		goto out;

	if (cq_host->ops->int_mask_set)
		cq_host->ops->int_mask_set(cq_host->mmc, false);
	cmdq_runtime_pm_get(cq_host);
	cqcfg = cmdq_readl(cq_host, CQCFG);
	if (cqcfg & 0x1) {
		pr_info("%s: %s: cq_host is already enabled\n",
				mmc_hostname(mmc), __func__);
		WARN_ON(1);
		goto pm_ref_count;
	}

	if (cq_host->quirks & CMDQ_QUIRK_NO_DCMD)
		dcmd_enable = false;
	else
		dcmd_enable = true;

	cqcfg = ((cq_host->caps & CMDQ_TASK_DESC_SZ_128 ? CQ_TASK_DESC_SZ : 0) |
			(dcmd_enable ? CQ_DCMD : 0));

	/* Interrupt Mask set */
	cmdq_writel(cq_host, 0x0, CQDATAINTMASK1);
	cmdq_writel(cq_host, 0x0, CQCMDINTMASK2);
	cmdq_interrupt_mask_set(cq_host, true);

	cmdq_writel(cq_host, cqcfg, CQCFG);
	/* enable CQ_HOST */
	cmdq_writel(cq_host, cmdq_readl(cq_host, CQCFG) | CQ_ENABLE,
		    CQCFG);

	if (!cq_host->desc_base ||
			!cq_host->trans_desc_base) {
		err = cmdq_host_alloc_tdl(cq_host);
		if (err)
			goto pm_ref_count;
	}

	cmdq_writel(cq_host, lower_32_bits(cq_host->desc_dma_base), CQTDLBA);
	cmdq_writel(cq_host, upper_32_bits(cq_host->desc_dma_base), CQTDLBAU);

	/*
	 * disable all vendor interrupts
	 * enable CMDQ interrupts
	 * enable the vendor error interrupts
	 */
	if (cq_host->ops->clear_set_irqs)
		cq_host->ops->clear_set_irqs(mmc, true);

	cmdq_clear_set_irqs(cq_host, 0x0, CQ_INT_ALL);

	/* cq_host would use this rca to address the card */
	cmdq_writel(cq_host, mmc->card->rca, CQSSC2);

	/* send QSR at lesser intervals than the default */
	{
#if 0
	cmdq_writel(cq_host, cmdq_readl(cq_host, CQSSC1) | SEND_QSR_INTERVAL,
				CQSSC1);
#else
	u32 reg = 0;

	reg = cmdq_readl(cq_host, CQSSC1);
	reg &= ~((0xF << 16) | (0xFFFF << 0));
	reg |= (CQSSC1_CIT_EN | (0x0 << 16) | (0x1 << 0));
	cmdq_writel(cq_host, reg, CQSSC1);
#endif
	}

	/* ensure the writes are done before enabling CQE */
	mb();

	cq_host->enabled = true;

	if (cq_host->ops->set_block_size)
		cq_host->ops->set_block_size(cq_host->mmc);

	if (cq_host->ops->set_data_timeout)
		cq_host->ops->set_data_timeout(mmc, 0xf);

	if (cq_host->ops->clear_set_dumpregs)
		cq_host->ops->clear_set_dumpregs(mmc, 1);

pm_ref_count:
	cmdq_runtime_pm_put(cq_host);
out:
	if (err)
		mmc_cmdq_error_logging(mmc->card, NULL, CQ_EN_DIS_ERR);
	return err;
}

static void cmdq_disable(struct mmc_host *mmc, bool soft)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);

	cmdq_runtime_pm_get(cq_host);
	if (soft) {
		cmdq_writel(cq_host, cmdq_readl(
				    cq_host, CQCFG) & ~(CQ_ENABLE),
			    CQCFG);
	}
	cmdq_runtime_pm_put(cq_host);
	cq_host->enabled = false;

	if (cq_host->ops->int_mask_set)
		cq_host->ops->int_mask_set(cq_host->mmc, true);
}

static void cmdq_enable_after_sw_reset(struct cmdq_host *cq_host,
							unsigned int rca)
{
	struct mmc_host *mmc = cq_host->mmc;
	bool dcmd_enable = (cq_host->quirks & CMDQ_QUIRK_NO_DCMD) ?
							false : true;
	struct mmc_cmdq_context_info *ctx_info = &mmc->cmdq_ctx;
	struct mmc_request *mrq_t;
	struct mmc_request *mrq_n;
	u32 reg;
	unsigned long flags;

	pr_err("[CQ] %s: Enable after SW RESET\n", mmc_hostname(mmc));

	cmdq_runtime_pm_get(cq_host);

	/* CQSSC1, CQSSC2 */
	reg = cmdq_readl(cq_host, CQSSC1);
	reg &= ~((0xF << 16) | (0xFFFF << 0));
	reg |= (CQSSC1_CIT_EN | (0x0 << 16) | (0x1 << 0));
	cmdq_writel(cq_host, reg, CQSSC1);
	cmdq_writel(cq_host, rca, CQSSC2);

	/* CQTDLBA, CQTDLBAU */
	cmdq_writel(cq_host, lower_32_bits(cq_host->desc_dma_base), CQTDLBA);
	cmdq_writel(cq_host, upper_32_bits(cq_host->desc_dma_base), CQTDLBAU);

	/* CQCFG */
	mb();
	reg = ((cq_host->caps & CMDQ_TASK_DESC_SZ_128 ? CQ_TASK_DESC_SZ : 0) |
			(dcmd_enable ? CQ_DCMD : 0));
	reg |= CQ_ENABLE;
	cmdq_writel(cq_host, reg, CQCFG);

	/* To block interrupt handling in legacy driver side */
	if (cq_host->ops->int_mask_set)
		cq_host->ops->int_mask_set(cq_host->mmc, false);

	/* CQDATAINTMASK1, CQCMDINTMASK2, CQISTE, CQISGE */
	cmdq_interrupt_mask_set(cq_host, true);
	cmdq_clear_set_irqs(cq_host, 0x0, CQ_INT_ALL);

	cmdq_runtime_pm_put(cq_host);

	spin_lock_irqsave(&cq_host->list_lock, flags);
	if (!list_empty(&cq_host->active_mrq)) {
		pr_err("[CQ] %s: pending list clear\n", mmc_hostname(mmc));
		list_for_each_entry_safe(mrq_t, mrq_n, &cq_host->active_mrq, cmdq_entry) {
			list_del(&mrq_t->cmdq_entry);

		}
	}
	spin_unlock_irqrestore(&cq_host->list_lock, flags);

	ctx_info->curr_dbr = 0;

	cq_host->enabled = true;
}

static void cmdq_reset(struct mmc_host *mmc, bool soft)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	unsigned int cqcfg;
	unsigned int tdlba;
	unsigned int tdlbau;
	unsigned int rca;
	int ret;
	unsigned long timeout;
	u32 reg;

	/*
	 * In this case, we already did soft reset.
	 * Now we need to enable CQE in a simple manner.
	 */
	if (cq_host->sw_reset)
		return cmdq_enable_after_sw_reset(cq_host, mmc->card->rca);

	cmdq_runtime_pm_get(cq_host);
	cqcfg = cmdq_readl(cq_host, CQCFG);
	tdlba = cmdq_readl(cq_host, CQTDLBA);
	tdlbau = cmdq_readl(cq_host, CQTDLBAU);
	rca = cmdq_readl(cq_host, CQSSC2);

	cmdq_disable(mmc, true);

	if (cq_host->ops->reset) {
		ret = cq_host->ops->reset(mmc);
		if (ret) {
			pr_crit("%s: reset CMDQ controller: failed\n",
				mmc_hostname(mmc));
			BUG();
		}
	}

	cmdq_writel(cq_host, tdlba, CQTDLBA);
	cmdq_writel(cq_host, tdlbau, CQTDLBAU);

	/*
	 * A sequence to be done before resume CQE
	 *
	 * 1. Clear_All_Tasks - all pending tasks cleared
	 * 2. HALT off
	 */
	timeout = jiffies + msecs_to_jiffies(1000);
	reg = cmdq_readl(cq_host, CQCTL) | CLEAR_ALL_TASKS;
	cmdq_writel(cq_host, reg, CQCTL);
	while (time_before(jiffies, timeout)) {
		reg = cmdq_readl(cq_host, CQCTL);
		if ((reg & CLEAR_ALL_TASKS) == 0)
			break;
		cmdq_writel(cq_host, reg | CLEAR_ALL_TASKS, CQCTL);
	}

	cmdq_writel(cq_host, cmdq_readl(cq_host, CQCTL) & ~HALT, CQCTL);
	cmdq_interrupt_mask_set(cq_host, true);

	/* ensure the writes are done before enabling CQE */
	cmdq_clear_set_irqs(cq_host, 0x0, CQ_INT_ALL);

	if (cq_host->ops->int_mask_set)
		cq_host->ops->int_mask_set(cq_host->mmc, false);

	/* cq_host would use this rca to address the card */
	cmdq_writel(cq_host, rca, CQSSC2);
	mb();

	cmdq_writel(cq_host, cqcfg, CQCFG);
	cmdq_runtime_pm_put(cq_host);
	cq_host->enabled = true;
}

static void cmdq_prep_task_desc(struct mmc_request *mrq,
					u64 *data, bool intr, bool qbr)
{
	struct mmc_cmdq_req *cmdq_req = mrq->cmdq_req;
	u32 req_flags = cmdq_req->cmdq_req_flags;

	pr_debug("%s: %s: data-tag: 0x%08x - dir: %d - prio: %d - cnt: 0x%08x -	addr: 0x%llx\n",
		 mmc_hostname(mrq->host), __func__,
		 !!(req_flags & DAT_TAG), !!(req_flags & DIR),
		 !!(req_flags & PRIO), cmdq_req->data.blocks,
		 (u64)mrq->cmdq_req->blk_addr);

	*data = VALID(1) |
		END(1) |
		INT(intr) |
		ACT(0x5) |
		FORCED_PROG(!!(req_flags & FORCED_PRG)) |
		CONTEXT(mrq->cmdq_req->ctx_id) |
		DATA_TAG(!!(req_flags & DAT_TAG)) |
		DATA_DIR(!!(req_flags & DIR)) |
		PRIORITY(!!(req_flags & PRIO)) |
		QBAR(qbr) |
		REL_WRITE(!!(req_flags & REL_WR)) |
		BLK_COUNT(mrq->cmdq_req->data.blocks) |
		BLK_ADDR((u64)mrq->cmdq_req->blk_addr);
}

static int cmdq_dma_map(struct mmc_host *host, struct mmc_request *mrq)
{
	int sg_count;
	struct mmc_data *data = mrq->data;

	if (!data)
		return -EINVAL;

	sg_count = dma_map_sg(mmc_dev(host), data->sg,
			      data->sg_len,
			      (data->flags & MMC_DATA_WRITE) ?
			      DMA_TO_DEVICE : DMA_FROM_DEVICE);
	if (!sg_count) {
		pr_err("%s: %s: sg-len: %d\n", mmc_hostname(host), __func__, data->sg_len);
		return -ENOMEM;
	}

	return sg_count;
}

static void cmdq_set_tran_desc(u8 *desc, dma_addr_t addr, int len,
				bool end, bool is_dma64)
{
	__le32 *attr = (__le32 __force *)desc;

	*attr = (VALID(1) |
		 END(end ? 1 : 0) |
		 INT(0) |
		 ACT(0x4) |
		 DAT_LENGTH(len));

	if (is_dma64) {
		__le64 *dataddr = (__le64 __force *)(desc + 4);

		dataddr[0] = cpu_to_le64(addr);
	} else {
		__le32 *dataddr = (__le32 __force *)(desc + 4);

		dataddr[0] = cpu_to_le32(addr);
	}
}

static int cmdq_prep_tran_desc(struct mmc_host *mmc, struct mmc_request *mrq,
			       struct cmdq_host *cq_host, int tag)
{
	struct mmc_data *data = mrq->data;
	int i, sg_count, len;
	bool end = false;
	dma_addr_t addr;
	u8 *desc;
	struct scatterlist *sg;
	int ret;
	int sector_offset = 0;

	sg_count = cmdq_dma_map(mrq->host, mrq);
	if (sg_count < 0) {
		pr_err("%s: %s: unable to map sg lists, %d\n",
				mmc_hostname(mrq->host), __func__, sg_count);
		return sg_count;
	}

	desc = get_trans_desc(cq_host, tag);
	memset(desc, 0, cq_host->trans_desc_len * cq_host->mmc->max_segs);

	for_each_sg(data->sg, sg, sg_count, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);

		if ((i+1) == sg_count)
			end = true;

		cmdq_set_tran_desc(desc, addr, len, end, cq_host->dma64);
		if (cq_host->ops->crypto_engine_cfg) {
			ret = cq_host->ops->crypto_engine_cfg(mmc, desc, data,
					sg_page(sg), sector_offset, true);
			if (ret) {
				pr_err("%s: %s: failed to configure crypto engine. ret(%d)\n",
						mmc_hostname(mmc), __func__, ret);
				return -1;
			}
			sector_offset += len / 512;
		}
		desc += cq_host->trans_desc_len;
	}

	pr_debug("%s: %s: req: 0x%p tag: %d calc_trans_des: 0x%p sg-cnt: %d\n",
		mmc_hostname(mmc), __func__, mrq->req, tag, desc, sg_count);

	return 0;
}

static void cmdq_prep_dcmd_desc(struct mmc_host *mmc,
				   struct mmc_request *mrq)
{
	u64 *task_desc = NULL;
	u64 data = 0;
	u8 resp_type;
	u8 *desc;
	__le64 *dataddr;
	struct cmdq_host *cq_host = mmc_cmdq_private(mmc);
	u8 timing;

	if (!(mrq->cmd->flags & MMC_RSP_PRESENT)) {
		resp_type = 0x0;
		timing = 0x1;
	} else {
		if (mrq->cmd->flags & MMC_RSP_BUSY) {
			resp_type = 0x3;
			timing = 0x0;
		} else {
			resp_type = 0x2;
			timing = 0x1;
		}
	}

	task_desc = (__le64 __force *)get_desc(cq_host, cq_host->dcmd_slot);
	memset(task_desc, 0, cq_host->task_desc_len);
	data |= (VALID(1) |
		 END(1) |
		 INT(1) |
		 QBAR(1) |
		 ACT(0x5) |
		 CMD_INDEX(mrq->cmd->opcode) |
		 CMD_TIMING(timing) | RESP_TYPE(resp_type));
	*task_desc |= data;
	desc = (u8 *)task_desc;
	pr_debug("%s: cmdq: dcmd: cmd: %d timing: %d resp: %d\n",
		mmc_hostname(mmc), mrq->cmd->opcode, timing, resp_type);
	dataddr = (__le64 __force *)(desc + 4);
	dataddr[0] = cpu_to_le64((u64)mrq->cmd->arg);

}

static int cmdq_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	int err = 0;
	u64 data = 0;
	u64 *task_desc = NULL;
	u32 tag = mrq->cmdq_req->tag;
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	struct mmc_cmdq_context_info *ctx_info = &mmc->cmdq_ctx;
	unsigned long flags;
	int ret = 0;

	if (!cq_host->enabled) {
		pr_err("%s: CMDQ host not enabled yet !!!\n",
		       mmc_hostname(mmc));
		err = -EINVAL;
		goto out;
	}

	cmdq_runtime_pm_get(cq_host);

	spin_lock_irqsave(&cq_host->lock, flags);
	if (cq_host->ops->hwacg_control_direct)
		ret = cq_host->ops->hwacg_control_direct(mmc, true);
	spin_unlock_irqrestore(&cq_host->lock, flags);
	if (ret)
		udelay(1);

	if (cq_host->ops->pm_qos_lock)
		cq_host->ops->pm_qos_lock(mmc, true);

	if (cq_host->ops->sicd_control)
		cq_host->ops->sicd_control(mmc, true);

	if (mrq->cmdq_req->cmdq_req_flags & DCMD) {
		cmdq_prep_dcmd_desc(mmc, mrq);
		cq_host->mrq_slot[DCMD_SLOT] = mrq;
		/* DCMD's are always issued on a fixed slot */
		tag = DCMD_SLOT;
		goto ring_doorbell;
	}

	task_desc = (__le64 __force *)get_desc(cq_host, tag);

	cmdq_prep_task_desc(mrq, &data, 1,
			    (mrq->cmdq_req->cmdq_req_flags & QBR));
	*task_desc = cpu_to_le64(data);

	err = cmdq_prep_tran_desc(mmc, mrq, cq_host, tag);
	if (err) {
		pr_err("%s: %s: failed to setup tx desc: %d\n",
		       mmc_hostname(mmc), __func__, err);
		goto out;
	}

	if (cq_host->ops->transferred_cnt)
		cq_host->ops->transferred_cnt(mmc, mrq);

	cq_host->mrq_slot[tag] = mrq;
	if (cq_host->ops->set_tranfer_params)
		cq_host->ops->set_tranfer_params(mmc);
wait:
	if (test_bit(CMDQ_STATE_DCMD_ACTIVE, &ctx_info->curr_state)) {
		if (test_bit(CMDQ_STATE_DO_RECOVERY, &ctx_info->curr_state)) {
			err = -ETIMEDOUT;
			goto  out;
		}
		goto wait;
	}

	if (test_bit(CMDQ_STATE_ERR_RCV_DONE, &ctx_info->curr_state)) {
		/* cq debug */
		pr_err("[CQ] %s: Orphan request 1: tag %d, flag = 0x%lx\n",
				mmc_hostname(mmc), tag, ctx_info->curr_state);
#ifdef CONFIG_MMC_CMDQ_DEBUG
		exynos_ss_printk("[CQ] Orphan request 1: tag %d, flag = 0x%lx\n",
						tag, ctx_info->curr_state);
#endif
		clear_bit(CMDQ_STATE_ERR_RCV_DONE, &ctx_info->curr_state);
		goto out;
	}

ring_doorbell:
	/* Ensure the task descriptor list is flushed before ringing doorbell */
	wmb();
	if (cmdq_readl(cq_host, CQTDBR) & (1 << tag)) {
		/* cq debug */
		pr_err("[CQ] DBR duplicated: tag %d\n", tag);
		cmdq_dumpregs(cq_host);
		BUG_ON(1);
	}

busy_wait:
	if (mmc->card->quirks & MMC_QUIRK_CMDQ_NEED_BUSYWAIT) {
		/*
		 * If MMC_QUIRK_CMDQ_NEED_BUSYWAIT is set,
		 * it needs busy_wait handling.
		 */
		if (test_bit(CMDQ_STATE_PREV_DCMD, &ctx_info->curr_state)) {
			if (test_bit(CMDQ_STATE_DO_RECOVERY, &ctx_info->curr_state)) {
				err = -ETIMEDOUT;
				goto  out;
			}
			if (cq_host->ops->busy_waiting) {
				if (cq_host->ops->busy_waiting(cq_host->mmc, mrq) == false) {
					goto busy_wait;
				}
			}
			clear_bit(CMDQ_STATE_PREV_DCMD, &ctx_info->curr_state);
		}
	} else {
		/*
		 * CQE might not work properly
		 * when issuing a DCMD during processing previous DCMD.
		 * In this cases, it need a sort of barrier to seperate
		 * two tasks' process window.
		 */
		if ((mrq->cmdq_req->cmdq_req_flags & DCMD) &&
				test_bit(CMDQ_STATE_PREV_DCMD, &ctx_info->curr_state)) {
			if (cq_host->ops->busy_waiting) {
				if (cq_host->ops->busy_waiting(cq_host->mmc, mrq) == false)
					goto busy_wait;
			}
		}
		clear_bit(CMDQ_STATE_PREV_DCMD, &ctx_info->curr_state);
	}

	if (test_bit(CMDQ_STATE_ERR_RCV_DONE, &ctx_info->curr_state)) {
		/* cq debug */
		pr_err("[CQ] %s: Orphan request 2: tag %d, flag = 0x%lx\n",
				mmc_hostname(mmc), tag, ctx_info->curr_state);
#ifdef CONFIG_MMC_CMDQ_DEBUG
		exynos_ss_printk("[CQ] Orphan request 2: tag %d, flag = 0x%lx\n",
						tag, ctx_info->curr_state);
#endif
		clear_bit(CMDQ_STATE_ERR_RCV_DONE, &ctx_info->curr_state);
		goto out;
	}

#if defined(CONFIG_MMC_DW_DEBUG)
	if (cq_host->ops->cmdq_log) {
		struct cmdq_log_ctx log_ctx;

		log_ctx.x0 = tag;
		log_ctx.x1 = cmdq_readl(cq_host, CQTDBR);

		if (mrq->cmdq_req->cmdq_req_flags & DCMD) {
			if (mrq->cmd->opcode >= MMC_ERASE_GROUP_START &&
					mrq->cmd->opcode <= MMC_ERASE) {
				/* erase case */
				log_ctx.x2 = CQ_LOG_CMD_DISCARD;
				log_ctx.x3 = mrq->cmd->arg;
			} else if (mrq->cmd->opcode == MMC_SWITCH &&
					((mrq->cmd->arg & 0xFFFF00) >> 16) ==
					EXT_CSD_FLUSH_CACHE) {
				/* flush case */
				log_ctx.x2 = CQ_LOG_CMD_FLUSH;
				log_ctx.x3 = mrq->cmd->arg;
			} else {
				/*
				 * unexpected case
				 *
				 * I'm wondering there is a case of CMD13..
				 */
				log_ctx.x2 = mrq->cmd->opcode;
				log_ctx.x3 = mrq->cmd->arg;
			}
			log_ctx.x4 = 0;
		} else {
			log_ctx.x2 = (mrq->cmdq_req->cmdq_req_flags & DIR) ?
						CQ_LOG_CMD_READ	:
						CQ_LOG_CMD_WRITE;
			log_ctx.x3 = mrq->cmdq_req->blk_addr;
			log_ctx.x4 = mrq->cmdq_req->data.blocks;
		}

		cq_host->ops->cmdq_log(cq_host->mmc, true, &log_ctx);

		cq_host->cmd_log_idx[tag] = log_ctx.idx;
	}
#endif
#ifdef CONFIG_MMC_CMDQ_DEBUG
	/* cq debug */
	if (mrq->cmdq_req->cmdq_req_flags & DCMD)
		exynos_ss_printk("[CQ] D: D %d, tag %d\n",
					mrq->cmd->opcode, tag);
	else if (mrq->cmdq_req->cmdq_req_flags & DIR)
		exynos_ss_printk("[CQ] D: R, tag %d\n", tag);
	else
		exynos_ss_printk("[CQ] D: W, tag %d\n", tag);
#endif
	spin_lock_irqsave(&cq_host->list_lock, flags);
	if (test_bit(CMDQ_STATE_DO_RECOVERY, &ctx_info->curr_state)) {
		err = -ETIMEDOUT;
		spin_unlock_irqrestore(&cq_host->list_lock, flags);
		goto  out;
	}
	list_add_tail(&mrq->cmdq_entry, &cq_host->active_mrq);
	spin_unlock_irqrestore(&cq_host->list_lock, flags);

	spin_lock_irqsave(&cq_host->lock, flags);
	set_bit(tag, &ctx_info->curr_dbr);
	spin_unlock_irqrestore(&cq_host->lock, flags);
	cmdq_writel(cq_host, 1 << tag, CQTDBR);
	/* Commit the doorbell write immediately */
	wmb();
out:
	if (err) {
		/* Error return */
		if (mrq->cmd)
			mrq->cmd->error = err;
		else
			mrq->data->error = err;
		mrq->done(mrq);
	}
	return 0;
}

static void cmdq_clear_all_data(struct mmc_host *mmc)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	struct mmc_cmdq_context_info *ctx_info = &mmc->cmdq_ctx;
	struct mmc_request *mrq_t;
	struct mmc_request *mrq_n;
	unsigned int tag = 0;
	unsigned long flags;

	if (list_empty(&cq_host->active_mrq))
		return;

	spin_lock_irqsave(&cq_host->list_lock, flags);
	set_bit(CMDQ_STATE_DO_RECOVERY, &ctx_info->curr_state);
	list_for_each_entry_safe(mrq_t, mrq_n, &cq_host->active_mrq, cmdq_entry) {
		list_del(&mrq_t->cmdq_entry);

		if (mrq_t->cmdq_req->cmdq_req_flags & DCMD)
			tag = DCMD_SLOT;
		else
			tag = mrq_t->cmdq_req->tag;

		pr_err("[CQ] %s: clear pending tag : %d\n",
				mmc_hostname(mmc), tag);

		if (mrq_t->cmd)
			mrq_t->cmd->error = -ETIMEDOUT;
		else
			mrq_t->data->error = -ETIMEDOUT;

		if (tag == cq_host->dcmd_slot)
			set_bit(CMDQ_STATE_PREV_DCMD, &ctx_info->curr_state);
#if defined(CONFIG_MMC_DW_DEBUG)
		if (cq_host->ops->cmdq_log) {
			struct cmdq_log_ctx log_ctx;
			u32 dbr = cmdq_readl(cq_host, CQTDBR);

			log_ctx.x0 = tag;
			log_ctx.x1 = dbr;

			if (cq_host->cmd_log_idx[tag] != 0xDEADBEAF) {
				log_ctx.idx = cq_host->cmd_log_idx[tag];
				cq_host->ops->cmdq_log(cq_host->mmc, false, &log_ctx);
			} else {
				WARN_ON(1);
			}
			cq_host->cmd_log_idx[tag] = 0xDEADBEAF;
		}
#endif
		mrq_t->done(mrq_t);
	}
	spin_unlock_irqrestore(&cq_host->list_lock, flags);
}

static void cmdq_finish_data(struct mmc_host *mmc, unsigned int tag)
{
	struct mmc_request *mrq;
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	struct mmc_cmdq_context_info *ctx_info = &mmc->cmdq_ctx;
	u32 dbr = cmdq_readl(cq_host, CQTDBR);
	unsigned long flags;

	if (test_bit(CMDQ_STATE_ERR, &ctx_info->curr_state)) {
		pr_err("[CQ] %s: err state is request ignored!\n", mmc_hostname(mmc));
		return;
	}

	mrq = get_req_by_tag(cq_host, tag);
	if (tag == cq_host->dcmd_slot) {
		mrq->cmd->resp[0] = cmdq_readl(cq_host, CQCRDCT);
		set_bit(CMDQ_STATE_PREV_DCMD, &ctx_info->curr_state);
	}

	spin_lock_irqsave(&cq_host->list_lock, flags);
	if (!list_empty(&cq_host->active_mrq))
		list_del(&mrq->cmdq_entry);
	spin_unlock_irqrestore(&cq_host->list_lock, flags);

	clear_bit(tag, &ctx_info->curr_dbr);
#if defined(CONFIG_MMC_DW_DEBUG)
	if (cq_host->ops->cmdq_log) {
		struct cmdq_log_ctx log_ctx;

		log_ctx.x0 = tag;
		log_ctx.x1 = dbr;

		if (cq_host->cmd_log_idx[tag] != 0xDEADBEAF) {
			log_ctx.idx = cq_host->cmd_log_idx[tag];
			cq_host->ops->cmdq_log(cq_host->mmc, false, &log_ctx);
		} else {
			WARN_ON(1);
		}
		cq_host->cmd_log_idx[tag] = 0xDEADBEAF;

	}
#endif
#ifdef CONFIG_MMC_CMDQ_DEBUG
	/* cq debug */
	exynos_ss_printk("[CQ] C: tag %d, dbr 0x%x\n", tag, dbr);
#endif
	cmdq_runtime_pm_put(cq_host);
	mrq->done(mrq);
	spin_lock_irqsave(&cq_host->lock, flags);
	if (!(ctx_info->curr_dbr) &&
		!(ctx_info->active_reqs & ~(1<<mrq->req->tag))) {
		spin_unlock_irqrestore(&cq_host->lock, flags);

		if (cq_host->ops->hwacg_control_direct)
			cq_host->ops->hwacg_control_direct(mmc, false);
		if (cq_host->ops->pm_qos_lock)
			cq_host->ops->pm_qos_lock(mmc, false);
		if (cq_host->ops->sicd_control)
			cq_host->ops->sicd_control(mmc, false);
	} else {
		spin_unlock_irqrestore(&cq_host->lock, flags);
	}
}

irqreturn_t cmdq_irq(struct mmc_host *mmc, int err)
{
	u32 status;
	unsigned long tag = 0, comp_status;
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	unsigned long err_info = 0;
	struct mmc_request *mrq;
	struct mmc_cmdq_context_info *ctx_info = &mmc->cmdq_ctx;
	int is_done_dbr;

	status = cmdq_readl(cq_host, CQIS);
	cmdq_writel(cq_host, status, CQIS);

	if (!status && !err)
		return IRQ_NONE;

	if (test_bit(CMDQ_STATE_DO_RECOVERY, &ctx_info->curr_state)) {
		pr_err("%s: err: %d CQIS: %08x Ignore Interrupt In Recovery\n", mmc_hostname(mmc), err, status);
		return IRQ_NONE;
	}

	/*
	 * To stall IO thread
	 */
	if (err || (status & (CQIS_BRE | CQIS_RED)))
		test_and_set_bit(CMDQ_STATE_ERR_HOST,
					&ctx_info->curr_state);

	if (status & CQIS_BRE) {
		pr_err("%s: err: %d CQIS: 0x%08x\n", mmc_hostname(mmc), err, status);
		ctx_info->dump_state = CMDQ_DUMP_CQIS_BRE;
		cmdq_dumpregs(cq_host);
	}

	if (err || (status & CQIS_RED)) {
		err_info = cmdq_readl(cq_host, CQTERRI);
		pr_err("%s: err: %d CQIS: 0x%08x CQTERRI (0x%08lx)\n",
		       mmc_hostname(mmc), err, status, err_info);

		if (err_info & CQ_RMEFV) {
			tag = GET_CMD_ERR_TAG(err_info);
			pr_err("%s: %s: CMD err tag: %lu\n", mmc_hostname(mmc), __func__, tag);
			if (!(test_bit(tag, &ctx_info->curr_dbr))) {
				pr_err("%s: %s: Invailed tag:%lu interrupt!\n",
						mmc_hostname(mmc), __func__, tag);
				ctx_info->dump_state = CMDQ_DUMP_CQIS_INV_TAG;
				cmdq_dumpregs(cq_host);
				cmdq_clear_all_data(mmc);
				return IRQ_NONE;
			}

			/* CMD44/45/46/47 will not have a valid cmd */
			mrq = get_req_by_tag(cq_host, tag);
			if (mrq->cmd)
				mrq->cmd->error = err;
			else
				mrq->data->error = err;

			ctx_info->dump_state = CMDQ_DUMP_CQIS_RMEFV;
			cmdq_dumpregs(cq_host);
		} else if (err_info & CQ_DTEFV) {
			tag = GET_DAT_ERR_TAG(err_info);
			pr_err("%s: %s: Dat err tag: %lu\n", mmc_hostname(mmc), __func__, tag);
			if (!(test_bit(tag, &ctx_info->curr_dbr))) {
				pr_err("%s: %s: Invailed tag:%lu interrupt!\n",
						mmc_hostname(mmc), __func__, tag);
				ctx_info->dump_state = CMDQ_DUMP_CQIS_INV_TAG;
				cmdq_dumpregs(cq_host);
				cmdq_clear_all_data(mmc);
				return IRQ_NONE;
			}

			mrq = get_req_by_tag(cq_host, tag);
			mrq->data->error = err;
			ctx_info->dump_state = CMDQ_DUMP_CQIS_DTEFV;
			cmdq_dumpregs(cq_host);
		} else {
			pr_err("%s: Incorrect RED interrupt occurred\n",
						mmc_hostname(mmc));

			ctx_info->dump_state = CMDQ_DUMP_CQIS_INV_RED;
			cmdq_dumpregs(cq_host);
			cmdq_clear_all_data(mmc);
			return IRQ_NONE;
		}

		/*
		 * CQE detected a reponse error from device
		 * In most cases, this would require a reset.
		 */
		if (status & CQIS_RED) {
			mrq->cmdq_req->resp_err = true;
			pr_err("%s: Response error (0x%08x) from card !!!",
					mmc_hostname(mmc), status);
			ctx_info->dump_state = CMDQ_DUMP_CQIS_RED;
			cmdq_dumpregs(cq_host);
		} else {
			mrq->cmdq_req->resp_idx = cmdq_readl(cq_host, CQCRI);
			mrq->cmdq_req->resp_arg = cmdq_readl(cq_host, CQCRA);
		}

		mmc->err_mrq = mrq;
		cmdq_finish_data(mmc, tag);
	}

#if CQ_DBG
	/*
	 * With less than 400 times, kernel might not work properly
	 * because of heavy stress.
	 */
	if ((++tcc_miss_period % 400) == 0)
		status &= ~(CQIS_TCC);
#endif
	if (status & CQIS_TCC) {
		/* read CQTCN and complete the request */
		comp_status = cmdq_readl(cq_host, CQTCN);
		if (!comp_status)
			goto out;
		/*
		 * The CQTCN must be cleared before notifying req completion
		 * to upper layers to avoid missing completion notification
		 * of new requests with the same tag.
		 */
		cmdq_writel(cq_host, comp_status, CQTCN);
		/*
		 * A write memory barrier is necessary to guarantee that CQTCN
		 * gets cleared first before next doorbell for the same tag is
		 * set but that is already achieved by the barrier present
		 * before setting doorbell, hence one is not needed here.
		 */
		for_each_set_bit(tag, &comp_status, cq_host->num_slots) {
			/* complete the corresponding mrq */
			spin_lock(&cq_host->lock);
			is_done_dbr = test_bit(tag, &ctx_info->curr_dbr);
			spin_unlock(&cq_host->lock);
			if (is_done_dbr) {
				/* complete the corresponding mrq */
				pr_debug("%s: completing tag -> %lu\n",
						mmc_hostname(mmc), tag);
				if (!err)
					cmdq_finish_data(mmc, tag);
			}
		}
	}

	if (status & CQIS_HAC) {
		if (cq_host->ops->post_cqe_halt)
			cq_host->ops->post_cqe_halt(mmc);
		/* halt is completed, wakeup waiting thread */
		complete(&cq_host->halt_comp);
	}

out:
	return IRQ_HANDLED;
}
EXPORT_SYMBOL(cmdq_irq);

extern void exynos_cqe_sw_reset(struct mmc_host *mmc);
static void cmdq_sw_reset(struct mmc_host *mmc, int result, u32 cqctl)
{
	int ret = result;
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);

	/*
	 * In halt falure, we need to do soft reset all logic
	 * for recovery.
	 */
	if (ret < 0) {
		pr_err("[CQ] %s: HALT failed: cqctl = 0x%08x\n",
					mmc_hostname(mmc), cqctl);
		cq_host->cnt_recovery_halt_fail++;
	} else {
		cq_host->cnt_recovery_halt_pass++;
	}
	cq_host->cnt_recovery++;

	pr_err("[CQ] %s: SW RESET: %d\n", mmc_hostname(mmc), ret);
	pr_err("----- cnt_recovery: %u\n", cq_host->cnt_recovery);
	pr_err("----- cnt_recovery_halt_pass: %u\n",
			cq_host->cnt_recovery_halt_pass);
	pr_err("----- cnt_recovery_halt_fail: %u\n",
			cq_host->cnt_recovery_halt_fail);

	/* CQE soft reset, we do it twice */
	exynos_cqe_sw_reset(mmc);
	udelay(1);
	exynos_cqe_sw_reset(mmc);

	/* legacy host reset */
	if (cq_host->ops->reset) {
		ret = cq_host->ops->reset(mmc);
		if (ret) {
			pr_crit("%s: reset CMDQ controller: failed\n",
					mmc_hostname(mmc));
			BUG();
		}
	}
}

/* May sleep */
static int cmdq_halt(struct mmc_host *mmc, bool halt)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	int ret = 0;
	int retries = 3;
	u32 reg;
	struct mmc_cmdq_context_info *ctx_info = &mmc->cmdq_ctx;

	cmdq_runtime_pm_get(cq_host);
	if (halt) {
		while (retries) {
			/* HALT on */
			reg = cmdq_readl(cq_host, CQCTL) | HALT;
			cmdq_writel(cq_host, reg, CQCTL);

			ret = wait_for_completion_timeout(&cq_host->halt_comp,
					  msecs_to_jiffies(HALT_TIMEOUT_MS));
			reg = cmdq_readl(cq_host, CQCTL);
			if (!ret && !(reg & HALT)) {
				ret = -ETIMEDOUT;
				retries--;
				continue;
			} else {
				break;
			}
		}

#ifdef CONFIG_MMC_CMDQ_DEBUG
		/* cq debug */
		exynos_ss_printk("[CQ] halt ctl on: cqctl = 0x%x\n", reg);
#endif
		cmdq_interrupt_mask_set(cq_host, false);
		cmdq_clear_set_irqs(cq_host, CQ_INT_ALL, 0x0);

		cq_host->sw_reset = false;
		if (test_bit(CMDQ_STATE_ERR, &ctx_info->curr_state)) {
			/* Recovery */
			cmdq_sw_reset(mmc, ret, reg);
			cq_host->sw_reset = true;
		}

		if (cq_host->ops->int_mask_set)
			cq_host->ops->int_mask_set(cq_host->mmc, true);

		ret = 0;
	} else {
		if (cq_host->sw_reset) {
			cmdq_enable_after_sw_reset(cq_host, mmc->card->rca);
		} else {
			/*
			 * A sequence to be done before resume CQE
			 *
			 * 1. Clear_All_Tasks - all pending tasks cleared
			 * 2. HALT off
			 */
			reg = cmdq_readl(cq_host, CQCTL) | CLEAR_ALL_TASKS;
			cmdq_writel(cq_host, reg, CQCTL);
			reg = cmdq_readl(cq_host, CQCTL) & ~HALT;
			cmdq_writel(cq_host, reg, CQCTL);

#ifdef CONFIG_MMC_CMDQ_DEBUG
			/* cq debug */
			exynos_ss_printk("[CQ] halt ctl off\n");
#endif
			cmdq_clear_set_irqs(cq_host, 0x0, CQ_INT_ALL);
			cmdq_interrupt_mask_set(cq_host, true);
			if (cq_host->ops->int_mask_set)
				cq_host->ops->int_mask_set(cq_host->mmc, false);
		}
	}
	cmdq_runtime_pm_put(cq_host);
	return ret;
}

static void cmdq_post_req(struct mmc_host *host, struct mmc_request *mrq,
			  int err)
{
	int ret;
	struct mmc_data *data = mrq->data;
	struct cmdq_host *cq_host = mmc_cmdq_private(host);
	u32 tag = mrq->cmdq_req->tag;
	u8 *desc;

	if (data) {
		if (cq_host->ops->crypto_engine_clear) {
			desc = get_trans_desc(cq_host, tag);
			ret = cq_host->ops->crypto_engine_clear(host, desc, true);
			if (ret) {
				pr_err("%s: %s: failed to clear crypto engine(%d)\n",
					mmc_hostname(host), __func__, ret);
			}
		}

		data->error = err;
		dma_unmap_sg(mmc_dev(host), data->sg, data->sg_len,
			     (data->flags & MMC_DATA_READ) ?
			     DMA_FROM_DEVICE : DMA_TO_DEVICE);
		if (err)
			data->bytes_xfered = 0;
		else
			data->bytes_xfered = blk_rq_bytes(mrq->req);
	}
}

static void cmdq_dumpstate(struct mmc_host *mmc)
{
	struct cmdq_host *cq_host = (struct cmdq_host *)mmc_cmdq_private(mmc);
	cmdq_runtime_pm_get(cq_host);
	cmdq_dumpregs(cq_host);
	cmdq_runtime_pm_put(cq_host);
}

static const struct mmc_cmdq_host_ops cmdq_host_ops = {
	.enable = cmdq_enable,
	.disable = cmdq_disable,
	.request = cmdq_request,
	.post_req = cmdq_post_req,
	.halt = cmdq_halt,
	.reset	= cmdq_reset,
	.dumpstate = cmdq_dumpstate,
	.pclear = cmdq_clear_all_data,
};

struct cmdq_host *cmdq_pltfm_init(struct platform_device *pdev)
{
	struct cmdq_host *cq_host;
	struct resource *cmdq_memres = NULL;

	/* check and setup CMDQ interface */
	cmdq_memres = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "cmdq_mem");
	if (!cmdq_memres) {
		dev_dbg(&pdev->dev, "CMDQ not supported\n");
		return ERR_PTR(-EINVAL);
	}

	cq_host = kzalloc(sizeof(*cq_host), GFP_KERNEL);
	if (!cq_host) {
		dev_err(&pdev->dev, "failed to allocate memory for CMDQ\n");
		return ERR_PTR(-ENOMEM);
	}
	cq_host->mmio = devm_ioremap(&pdev->dev,
				     cmdq_memres->start,
				     resource_size(cmdq_memres));
	if (!cq_host->mmio) {
		dev_err(&pdev->dev, "failed to remap cmdq regs\n");
		kfree(cq_host);
		return ERR_PTR(-EBUSY);
	}
	dev_dbg(&pdev->dev, "CMDQ ioremap: done\n");

	return cq_host;
}
EXPORT_SYMBOL(cmdq_pltfm_init);

int cmdq_init(struct cmdq_host *cq_host, struct mmc_host *mmc,
	      bool dma64)
{

	struct cmdq_sfr_ramdump		*dump;
	struct mmc_cmdq_context_info *ctx_info = &mmc->cmdq_ctx;
	int err = 0;

	cq_host->dma64 = dma64;
	cq_host->mmc = mmc;
	cq_host->mmc->cmdq_private = cq_host;

	cq_host->num_slots = NUM_SLOTS;
	cq_host->dcmd_slot = DCMD_SLOT;

	mmc->cmdq_ops = &cmdq_host_ops;

	cq_host->mrq_slot = kzalloc(sizeof(cq_host->mrq_slot) *
				    cq_host->num_slots, GFP_KERNEL);

	if (!cq_host->mrq_slot)
		return -ENOMEM;

	dump = kzalloc(sizeof(*dump), GFP_KERNEL);
	if (!dump)
		return -ENOMEM;

	cq_host->cq_dump = dump;
	cq_host->halt_failed = false;
	cq_host->sw_reset = false;
	cq_host->cnt_recovery = 0;
	cq_host->cnt_recovery_halt_pass = 0;
	cq_host->cnt_recovery_halt_fail = 0;

	ctx_info->dump_state = CMDQ_DUMP_NONE_ERR;

	init_completion(&cq_host->halt_comp);

	spin_lock_init(&cq_host->lock);
	spin_lock_init(&cq_host->list_lock);
	INIT_LIST_HEAD(&cq_host->active_mrq);
#if CQ_DBG
	tcc_miss_period = 0;
#endif
	return err;
}
EXPORT_SYMBOL(cmdq_init);


int cmdq_free(struct cmdq_host *cq_host)
{
	if (cq_host->mrq_slot)
		kzfree(cq_host->mrq_slot);
	if (cq_host->cq_dump)
		kzfree(cq_host->cq_dump);
	return 0;
}
EXPORT_SYMBOL(cmdq_free);