/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/pm_qos.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>
#include <asm/uaccess.h>

#include <soc/samsung/bts.h>
#include "cal_bts7885.h"

#define BTS_DBG(x...)		if (exynos_bts_log) pr_info(x)

#define NUM_CHANNEL		4
#define MIF_UTIL		65
#define INT_UTIL		70

static int exynos_bts_log;
static unsigned int exynos_mif_util = MIF_UTIL;
static unsigned int exynos_int_util = INT_UTIL;

enum bts_index {
	BTS_IDX_G3D,
	BTS_IDX_MFCMSCL,
	BTS_IDX_FSYS,
	BTS_IDX_GNSS,
	BTS_IDX_ISP0,
	BTS_IDX_ISP1,
	BTS_IDX_WLBT,
	BTS_IDX_CP,
	BTS_IDX_CAM,
	BTS_IDX_DPU,
	BTS_IDX_ABOX,
	BTS_IDX_CPU_DMC0,
	BTS_IDX_CPU_DMC1,
};

enum exynos_bts_type {
	BT_TREX,
};

struct bts_table {
	struct bts_status stat;
	struct bts_info *next_bts;
	int prev_scen;
	int next_scen;
};

struct bts_info {
	const char *name;
	unsigned int pa_base;
	void __iomem *va_base;
	bool enable;
	enum exynos_bts_type type;
	struct bts_table table[BS_MAX];
	enum bts_scen_type top_scen;
};

struct bts_scenario {
	const char *name;
	struct bts_info *head;
};

static struct pm_qos_request exynos_mif_bts_qos;
static struct pm_qos_request exynos_int_bts_qos;
static DEFINE_SPINLOCK(bts_lock);
static DEFINE_MUTEX(media_mutex);
static void __iomem *base_drex0;
static void __iomem *base_drex1;

static struct bts_info exynos_bts[] = {
	[BTS_IDX_G3D] = {
		.name ="g3d",
		.pa_base = EXYNOS7885_PA_G3D,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
	},
	[BTS_IDX_MFCMSCL] = {
		.name = "mfcmscl",
		.pa_base = EXYNOS7885_PA_MFCMSCL,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
		.table[BS_DEFAULT].stat.rmo = 0x10,
		.table[BS_DEFAULT].stat.wmo = 0x10,
		.table[BS_MFC_UHD].stat.scen_en = true,
		.table[BS_MFC_UHD].stat.priority = 0x4,
		.table[BS_MFC_UHD].stat.rmo = BTS_MAX_MO,
		.table[BS_MFC_UHD].stat.wmo = BTS_MAX_MO,
	},
	[BTS_IDX_FSYS] = {
		.name = "fsys",
		.pa_base = EXYNOS7885_PA_FSYS,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
	},
	[BTS_IDX_GNSS] = {
		.name = "gnss",
		.pa_base = EXYNOS7885_PA_GNSS,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
	},
	[BTS_IDX_ISP0] = {
		.name = "isp0",
		.pa_base = EXYNOS7885_PA_ISP0,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
	},
	[BTS_IDX_ISP1] = {
		.name = "isp1",
		.pa_base = EXYNOS7885_PA_ISP1,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
	},
	[BTS_IDX_WLBT] = {
		.name = "wlbt",
		.pa_base = EXYNOS7885_PA_WLBT,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
	},
	[BTS_IDX_CP] = {
		.name = "cp",
		.pa_base = EXYNOS7885_PA_CP,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.bypass_en = 0x1,
	},
	[BTS_IDX_CAM] = {
		.name = "cam",
		.pa_base = EXYNOS7885_PA_CAM,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0xC,
	},
	[BTS_IDX_DPU] = {
		.name = "dpu",
		.pa_base = EXYNOS7885_PA_DPU,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0xA,
	},
	[BTS_IDX_ABOX] = {
		.name = "abox",
		.pa_base = EXYNOS7885_PA_ABOX,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
#ifdef CONFIG_EXYNOS7885_BTS_VM
		.table[BS_DEFAULT].stat.priority = 0x7,
#else
		.table[BS_DEFAULT].stat.priority = 0xA,
#endif
	},
	[BTS_IDX_CPU_DMC0] = {
		.name = "cpu_dmc0",
		.pa_base = EXYNOS7885_PA_CPU_DMC0,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
	},
	[BTS_IDX_CPU_DMC1] = {
		.name = "cpu_dmc1",
		.pa_base = EXYNOS7885_PA_CPU_DMC1,
		.type = BT_TREX,
		.enable = true,
		.table[BS_DEFAULT].stat.scen_en = true,
		.table[BS_DEFAULT].stat.priority = 0x4,
	},
};

static struct bts_scenario bts_scen[BS_MAX] = {
	[BS_DEFAULT] = {
		.name = "default",
	},
	[BS_MFC_UHD] = {
		.name = "mfc_uhd",
	},
};

static void bts_set_ip_table(struct bts_info *bts)
{
	enum bts_scen_type scen = bts->top_scen;

	BTS_DBG("[BTS] %s bts scen: [%s]->[%s]\n", bts->name,
			bts_scen[scen].name, bts_scen[scen].name);

	switch (bts->type) {
	case BT_TREX:
		bts_setqos(bts->va_base, &bts->table[scen].stat);
		break;
	default:
		break;
	}

	return;
}

static void bts_add_scen(enum bts_scen_type scen)
{
	struct bts_info *first = bts_scen[scen].head;
	struct bts_info *bts = bts_scen[scen].head;
	int next = 0;
	int prev = 0;

	if (!bts)
		return;

	BTS_DBG("[BTS] scen %s on\n", bts_scen[scen].name);

	do {
		if (bts->enable &&!bts->table[scen].next_scen) {
			if (scen >= bts->top_scen) {
				/* insert at top priority */
				bts->table[scen].prev_scen = bts->top_scen;
				bts->table[bts->top_scen].next_scen = scen;
				bts->top_scen = scen;
				bts->table[scen].next_scen = -1;

				bts_set_ip_table(bts);

			} else {
				/* insert at middle */
				for (prev = bts->top_scen; prev > scen;
				     prev = bts->table[prev].prev_scen)
					next = prev;

				bts->table[scen].prev_scen =
					bts->table[next].prev_scen;
				bts->table[scen].next_scen =
					bts->table[prev].next_scen;
				bts->table[next].prev_scen = scen;
				bts->table[prev].next_scen = scen;
			}
		}

		bts = bts->table[scen].next_bts;
	/* set all bts ip in the current scenario */
	} while (bts && bts != first);
}

static void bts_del_scen(enum bts_scen_type scen)
{
	struct bts_info *first = bts_scen[scen].head;
	struct bts_info *bts = bts_scen[scen].head;
	int next = 0;
	int prev = 0;

	if (!bts)
		return;

	BTS_DBG("[BTS] scen %s off\n", bts_scen[scen].name);

	do {
		if (bts->enable && bts->table[scen].next_scen) {
			if (scen == bts->top_scen) {
				/* revert to prev scenario */
				prev = bts->table[scen].prev_scen;
				bts->top_scen = prev;
				bts->table[prev].next_scen = -1;
				bts->table[scen].next_scen = 0;
				bts->table[scen].prev_scen = 0;

				bts_set_ip_table(bts);
			} else if (scen < bts->top_scen) {
				/* delete mid scenario */
				prev = bts->table[scen].prev_scen;
				next = bts->table[scen].next_scen;

				bts->table[next].prev_scen =
					bts->table[scen].prev_scen;
				bts->table[prev].next_scen =
					bts->table[scen].next_scen;

				bts->table[scen].prev_scen = 0;
				bts->table[scen].next_scen = 0;

			} else {
				BTS_DBG("[BTS]%s scenario couldn't \
					exist above top_scen\n",
					bts_scen[scen].name);
			}
		}

		bts = bts->table[scen].next_bts;
	/* revert all bts ip to prev in the current scenario */
	} while (bts && bts != first);
}

void bts_update_scen(enum bts_scen_type scen, unsigned int val)
{
	bool on = val ? 1 : 0;

	if (scen <= BS_DEFAULT || scen >= BS_MAX)
		return;

	switch (scen) {
	default:
		spin_lock(&bts_lock);
		if (on)
			bts_add_scen(scen);
		else
			bts_del_scen(scen);
		spin_unlock(&bts_lock);
		break;
	}
}

static void scen_chaining(enum bts_scen_type scen)
{
	struct bts_info *prev = NULL;
	struct bts_info *first = NULL;
	struct bts_info *bts;

	for (bts = exynos_bts;
	     bts <= &exynos_bts[ARRAY_SIZE(exynos_bts) - 1]; bts++) {
		if (bts->table[scen].stat.scen_en) {
			if (!first)
				first = bts;
			if (prev)
				prev->table[scen].next_bts = bts;

			prev = bts;
		}
	}

	if (prev)
		prev->table[scen].next_bts = first;

	bts_scen[scen].head = first;
}

#define BIT_PER_BYTE		8

static unsigned int bts_bw_calc(struct bts_decon_info *decon, int idx)
{
	struct bts_dpp_info *dpp = &decon->dpp[idx];
	unsigned int bw;
	unsigned int dst_w, dst_h;

	dst_w = dpp->dst.x2 - dpp->dst.x1;
	dst_h = dpp->dst.y2 - dpp->dst.y1;
	if (!(dst_w && dst_h))
		return 0;
	/* use multifactor for KB/s */
	bw = ((u64)dpp->src_h * dpp->src_w * dpp->bpp * decon->vclk) *
	       (decon->lcd_w*11 + 480) / decon->lcd_w / 10 /
		(BIT_PER_BYTE * dst_h * decon->lcd_w);

	return bw;
}

static unsigned int bts_find_max_bw(struct bts_decon_info *decon,
			 const struct bts_layer_position *input, int idx)
{
	struct bts_layer_position output;
	struct bts_dpp_info *dpp;
	unsigned int max = 0;
	int i;

	for (i = idx; i < BTS_DPP_MAX; i++) {
		dpp = &decon->dpp[i];
		if (!dpp->used)
			continue;
		output.y1 = input->y1 < dpp->dst.y1 ? dpp->dst.y1 : input->y1;
		output.y2 = input->y2 > dpp->dst.y2 ? dpp->dst.y2 : input->y2;
		output.x1 = input->x1 < dpp->dst.x1 ? dpp->dst.x1 : input->x1;
		output.x2 = input->x2 > dpp->dst.x2 ? dpp->dst.x2 : input->x2;
		if (output.y1 < output.y2) {
			unsigned int bw;
			bw = dpp->bw + bts_find_max_bw(decon, &output, i + 1);
			if (bw > max)
				max = bw;
		}
	}
	return max;

}

static unsigned int bts_update_decon_bw(struct bts_decon_info *decon)
{
	unsigned int max = 0;
	struct bts_dpp_info *dpp;
	int i;

	for (i = 0; i < BTS_DPP_MAX; i++) {
		dpp = &decon->dpp[i];
		if (!dpp->used)
			continue;
		dpp->bw = bts_bw_calc(decon, i);
	}
	for (i = 0; i < BTS_DPP_MAX; i++) {
		unsigned int bw;
		dpp = &decon->dpp[i];
		if (!dpp->used)
			continue;
		bw = dpp->bw + bts_find_max_bw(decon, &dpp->dst, i + 1);
		if (bw > max)
			max = bw;
	}

	return max;
}

unsigned int bts_calc_bw(enum bts_bw_type type, void *data)
{
	unsigned int bw;

	switch (type) {
	case BTS_BW_DECON0:
	case BTS_BW_DECON1:
	case BTS_BW_DECON2:
		bw = bts_update_decon_bw(data);
		break;
	default:
		bw = 0;
		break;
	}

	return bw;
}

void bts_update_bw(enum bts_bw_type type, struct bts_bw bw)
{
	static struct bts_bw ip_bw[BTS_BW_MAX];
	unsigned int mif_freq;
	unsigned int int_freq;
	unsigned int total_bw = 0;
	unsigned int bw_r = 0;
	unsigned int bw_w = 0;
	unsigned int int_bw = 0;
	int i;

	if (type >= BTS_BW_MAX)
		return;
	if (ip_bw[type].peak == bw.peak
	    && ip_bw[type].read == bw.read
	    && ip_bw[type].write == bw.write)
		return;
	mutex_lock(&media_mutex);

	ip_bw[type] = bw;
	for (i = 0; i < BTS_BW_MAX; i++) {
		if (int_bw < ip_bw[i].peak)
			int_bw = ip_bw[i].peak;
		bw_r += ip_bw[i].read;
		bw_w += ip_bw[i].write;
	}
	total_bw = bw_r + bw_w;
	if (int_bw < (bw_w / NUM_CHANNEL))
		int_bw = bw_w / NUM_CHANNEL;
	if (int_bw < (bw_r / NUM_CHANNEL))
		int_bw = bw_r / NUM_CHANNEL;

	/* MIF minimum frequency calculation as per BTS guide */
	mif_freq = total_bw * 100 / BUS_WIDTH / exynos_mif_util;
	int_freq = int_bw * 100 / BUS_WIDTH / exynos_int_util;

	pm_qos_update_request(&exynos_mif_bts_qos, mif_freq);
	pm_qos_update_request(&exynos_int_bts_qos, int_freq);

	BTS_DBG("[BTS] BW(KB/s): type%i bw %up %ur %uw, "
		"int %u total %u, read %u, write %u, "
		"freq(Khz): mif %u, int %u\n",
		type, bw.peak, bw.read, bw.write, int_bw, total_bw,
		bw_r, bw_w, mif_freq, int_freq);

	mutex_unlock(&media_mutex);
}

static void bts_drex_init(void __iomem *base)
{
	__raw_writel(0x00000000, base + QOS_TIMEOUT_F);
	__raw_writel(0x00000004, base + QOS_TIMEOUT_E);
	__raw_writel(0x00000008, base + QOS_TIMEOUT_D);
	__raw_writel(0x00000010, base + QOS_TIMEOUT_C);
	__raw_writel(0x00000010, base + QOS_TIMEOUT_B);
	__raw_writel(0x00000010, base + QOS_TIMEOUT_A);
	__raw_writel(0x00000020, base + QOS_TIMEOUT_9);
	__raw_writel(0x00000040, base + QOS_TIMEOUT_8);
#ifdef CONFIG_EXYNOS7885_BTS_VM
	__raw_writel(0x00000080, base + QOS_TIMEOUT_7);
#else
	__raw_writel(0x00000100, base + QOS_TIMEOUT_7);
#endif
	__raw_writel(0x00000100, base + QOS_TIMEOUT_6);
	__raw_writel(0x00000100, base + QOS_TIMEOUT_5);
	__raw_writel(0x00000100, base + QOS_TIMEOUT_4);
	__raw_writel(0x00000100, base + QOS_TIMEOUT_3);
	__raw_writel(0x00000100, base + QOS_TIMEOUT_2);
	__raw_writel(0x00000100, base + QOS_TIMEOUT_1);
}

static void bts_initialize_domains(void)
{
	struct bts_info *bts;

	spin_lock(&bts_lock);
	for (bts = exynos_bts;
	     bts <= &exynos_bts[ARRAY_SIZE(exynos_bts) - 1]; bts++) {
		if (!bts->enable)
			continue;
		bts_set_ip_table(bts);
	}
	spin_unlock(&bts_lock);
	return;
}

static int exynos_bts_syscore_suspend(void)
{
	return 0;
}

static void exynos_bts_syscore_resume(void)
{
	bts_drex_init(base_drex0);
	bts_drex_init(base_drex1);
	bts_initialize_domains();
}

static struct syscore_ops exynos_bts_syscore_ops = {
	.suspend	= exynos_bts_syscore_suspend,
	.resume		= exynos_bts_syscore_resume,
};

#if defined(CONFIG_DEBUG_FS)
static int exynos_qos_status_open_show(struct seq_file *buf, void *d)
{
	struct bts_info *bts;

	spin_lock(&bts_lock);

	for (bts = exynos_bts;
	     bts <= &exynos_bts[ARRAY_SIZE(exynos_bts) - 1]; bts++) {
		if (!bts->enable) {
			seq_printf(buf, "%5s(disabled): ", bts->name);
			continue;
		} else {
			seq_printf(buf, "%5s(%s): ",
					bts->name, bts_scen[bts->top_scen].name);
		}
		switch (bts->type) {
		case BT_TREX:
			bts_showqos(bts->va_base, buf);
			break;
		default:
			seq_puts(buf, "none\n");
		}
	}

	spin_unlock(&bts_lock);

	return 0;
}

static int exynos_timeout_status_open_show(struct seq_file *buf, void *d)
{
	int i;

	seq_puts(buf, "\tqos/timeout\nex)echo 0 100 > timeout\n");
	seq_puts(buf, "DREX0\n");
	for (i = 0; i < 0xf; i++) {
		seq_printf(buf, "[0x%x]: %d\n", i + 1, __raw_readl(base_drex0 +
					QOS_TIMEOUT_1 + 4 * i));
	}

	seq_puts(buf, "DREX1\n");
	for (i = 0; i < 0xf; i++) {
		seq_printf(buf, "[0x%x]: %d\n", i + 1, __raw_readl(base_drex1 +
					QOS_TIMEOUT_1 + 4 * i));
	}

	return 0;
}

static ssize_t exynos_timeout_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[32];
	int qos, timeout, ret;
	ssize_t len;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';

	ret = sscanf(buf, "%d %d\n", &qos, &timeout);
	if (ret != 2) {
		pr_err("%s, Failed at sscanf function: %d\n", __func__, ret);
		return -EINVAL;
	}

	if (qos < 0 || timeout < 0) {
		pr_info("Invalid variable\n");
		return len;
	}
	__raw_writel(timeout, base_drex0 + QOS_TIMEOUT_1 + 4 * (qos - 1));
	__raw_writel(timeout, base_drex1 + QOS_TIMEOUT_1 + 4 * (qos - 1));

	return len;
}


static int exynos_mo_status_open_show(struct seq_file *buf, void *d)
{
	struct bts_info *bts;
	int i, nr_ip = 0;

	seq_puts(buf, "\tIP/Scen/RW/MO\nex)echo 0 0 0 16 > mo\n");
	spin_lock(&bts_lock);

	for (bts = exynos_bts;
			bts <= &exynos_bts[ARRAY_SIZE(exynos_bts) - 1]; bts++) {
		if (!bts->enable) {
			seq_printf(buf, "[%2d]IP: %s is disabled\n", nr_ip++, bts->name);
			continue;
		} else {
			seq_printf(buf, "[%2d]IP: %s\n", nr_ip++, bts->name);
		}
		for (i = 0; i < ARRAY_SIZE(bts_scen) - 1; i++) {
			if (!bts_scen[i].name)
				continue;
			seq_printf(buf, "%6s: rmo:0x%x wmo:0x%x\n",
					bts_scen[i].name,
					bts->table[i].stat.rmo,
					bts->table[i].stat.wmo);
		}
	}

	spin_unlock(&bts_lock);

	return 0;
}

static ssize_t exynos_mo_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct bts_info *bts = NULL;
	char buf[32];
	int ip, scen, rw, mo, ret;
	ssize_t len;
	int nr_ip = ARRAY_SIZE(exynos_bts) - 1;
	int nr_scen = ARRAY_SIZE(bts_scen) - 1;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';

	ret = sscanf(buf, "%d %d %d %d\n", &ip, &scen, &rw, &mo);
	if (ret != 4) {
		pr_err("%s, Failed at sscanf function: %d\n", __func__, ret);
		return -EINVAL;
	}

	if (ip < 0 || ip > nr_ip || scen < 0 ||
			scen > nr_scen || rw < 0 || mo < 0) {
		pr_info("Invalid variable\n");
		return len;
	}

	bts = &exynos_bts[ip];

	spin_lock(&bts_lock);
	if (!rw)
		bts->table[scen].stat.rmo = mo;
	else
		bts->table[scen].stat.wmo = mo;

	if (!bts->table[scen].stat.scen_en) {
		bts->table[scen].stat.scen_en = true;
		scen_chaining(scen);
	}

	if (scen == bts->top_scen)
		bts_setqos(bts->va_base, &bts->table[scen].stat);
	spin_unlock(&bts_lock);

	return len;
}

static int exynos_prio_status_open_show(struct seq_file *buf, void *d)
{
	struct bts_info *bts;
	int i, nr_ip = 0;

	seq_puts(buf, "\tqos IP/Scen/Prio\nex)echo 0 0 8 > priority\n");
	spin_lock(&bts_lock);

	for (bts = exynos_bts;
			bts <= &exynos_bts[ARRAY_SIZE(exynos_bts) - 1]; bts++) {
		if (!bts->enable) {
			seq_printf(buf, "[%2d]IP: %s is disabled\n", nr_ip++, bts->name);
			continue;
		} else {
			seq_printf(buf, "[%2d]IP: %s\n", nr_ip++, bts->name);
		}
		for (i = 0; i < ARRAY_SIZE(bts_scen) - 1; i++) {
			if (!bts_scen[i].name)
				continue;
			seq_printf(buf, "%6s: %d\n",
					bts_scen[i].name, bts->table[i].stat.priority);
		}
	}

	spin_unlock(&bts_lock);

	return 0;
}

static ssize_t exynos_prio_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct bts_info *bts = NULL;
	char buf[32];
	int ip, scen, prio, ret;
	ssize_t len;
	int nr_ip = ARRAY_SIZE(exynos_bts) - 1;
	int nr_scen = ARRAY_SIZE(bts_scen) - 1;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';

	ret = sscanf(buf, "%d %d %d\n", &ip, &scen, &prio);
	if (ret != 3) {
		pr_err("%s, Failed at sscanf function: %d\n", __func__, ret);
		return -EINVAL;
	}

	if (ip < 0 || ip > nr_ip || scen < 0 ||
			scen > nr_scen || prio < 0 || prio > 0xf) {
		pr_info("Invalid variable\n");
		return len;
	}

	bts = &exynos_bts[ip];

	spin_lock(&bts_lock);
	bts->table[scen].stat.priority = prio;

	if (!bts->table[scen].stat.scen_en) {
		bts->table[scen].stat.scen_en = true;
		scen_chaining(scen);
	}

	if (scen == bts->top_scen)
		bts_setqos(bts->va_base, &bts->table[scen].stat);
	spin_unlock(&bts_lock);

	return len;
}

static int exynos_scen_status_open_show(struct seq_file *buf, void *d)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bts_scen) - 1; i++) {
		if (!bts_scen[i].name)
			continue;
		seq_printf(buf, "[%2d]%9s\n", i, bts_scen[i].name);
	}
	return 0;
}

static ssize_t exynos_scen_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char *buf;
	int ret;
	u32 scen, on;

	buf = kmalloc(count, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	ret = copy_from_user(buf, user_buf, count);
	if (ret < 0)
		goto out;

	buf[count] = '\0';

	ret = sscanf(buf, "%u %u", &scen, &on);
	if (ret != 2) {
		pr_err("%s, Failed at sscanf function: %d\n", __func__, ret);
		goto out;
	}

	if (scen >= BS_MAX) {
		pr_err("Invalid	variable\n");
		goto out;
	}

	bts_update_scen((enum bts_scen_type)scen, on);

out:
	kfree(buf);

	return count;
}

static int exynos_addr_status_open_show(struct seq_file *buf, void *d)
{
	struct bts_info *bts;

	spin_lock(&bts_lock);

	for (bts = exynos_bts;
			bts <= &exynos_bts[ARRAY_SIZE(exynos_bts) - 1]; bts++) {
		if (!bts->enable)
			continue;
		seq_printf(buf, "[IP: %9s]:0x%x \n", bts->name, bts->pa_base);
	}

	spin_unlock(&bts_lock);

	return 0;
}

static int exynos_qos_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_qos_status_open_show, inode->i_private);
}

static int exynos_timeout_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_timeout_status_open_show, inode->i_private);
}

static int exynos_mo_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_mo_status_open_show, inode->i_private);
}

static int exynos_prio_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_prio_status_open_show, inode->i_private);
}

static int exynos_scen_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_scen_status_open_show, inode->i_private);
}

static int exynos_addr_open(struct inode *inode, struct file *file)
{
	return single_open(file, exynos_addr_status_open_show, inode->i_private);
}

static const struct file_operations debug_qos_status_fops = {
	.open		= exynos_qos_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations debug_timeout_status_fops = {
	.open		= exynos_timeout_open,
	.read		= seq_read,
	.write		= exynos_timeout_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations debug_mo_status_fops = {
	.open		= exynos_mo_open,
	.read		= seq_read,
	.write		= exynos_mo_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations debug_prio_status_fops = {
	.open		= exynos_prio_open,
	.read		= seq_read,
	.write		= exynos_prio_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations debug_scen_status_fops = {
	.open		= exynos_scen_open,
	.read		= seq_read,
	.write		= exynos_scen_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations debug_addr_status_fops = {
	.open		= exynos_addr_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void bts_debugfs(void)
{
	struct dentry *den;

	den = debugfs_create_dir("bts", NULL);
	if (IS_ERR_OR_NULL(den)) {
		pr_err("%s debugfs create directory failed\n", __func__);
		return;
	}

	debugfs_create_file("qos", 0440, den, NULL, &debug_qos_status_fops);
	debugfs_create_file("mo", 0644, den, NULL, &debug_mo_status_fops);
	debugfs_create_file("timeout", 0644, den, NULL, &debug_timeout_status_fops);
	debugfs_create_file("priority", 0644, den, NULL, &debug_prio_status_fops);
	debugfs_create_file("scenario", 0440, den, NULL, &debug_scen_status_fops);
	debugfs_create_file("address", 0440, den, NULL, &debug_addr_status_fops);
	if (!debugfs_create_u32("log", 0644, den, &exynos_bts_log)) {
		pr_err("[BTS]: could't create debugfs bts log\n");
	}
	if (!debugfs_create_u32("mif_util", 0644, den, &exynos_mif_util)) {
		pr_err("[BTS]: could't create debugfs mif util\n");
	}
	if (!debugfs_create_u32("int_util", 0644, den, &exynos_int_util)) {
		pr_err("[BTS]: could't create debugfs int util\n");
	}

	return;
}
#else
static void bts_debugfs(void)
{
	pr_info("%s is disabled, check configuration\n", __func__);

	return;
}
#endif

static int __init exynos_bts_init(void)
{
	unsigned int i;
	struct bts_info *bts;

	for (bts = exynos_bts;
		bts <= &exynos_bts[ARRAY_SIZE(exynos_bts) - 1]; bts++) {
		bts->va_base = ioremap(bts->pa_base, SZ_2K);
		if (!bts->va_base) {
			pr_err("failed to map bts physical address\n");
			bts->enable = false;
		}
	}

	for (i = BS_DEFAULT + 1; i < BS_MAX; i++)
		scen_chaining(i);

	base_drex0 = ioremap(EXYNOS7885_PA_DREX0, SZ_4K);
	base_drex1 = ioremap(EXYNOS7885_PA_DREX1, SZ_4K);
	bts_drex_init(base_drex0);
	bts_drex_init(base_drex1);
	bts_initialize_domains();

	pm_qos_add_request(&exynos_mif_bts_qos, PM_QOS_BUS_THROUGHPUT, 0);
	pm_qos_add_request(&exynos_int_bts_qos, PM_QOS_DEVICE_THROUGHPUT, 0);
	register_syscore_ops(&exynos_bts_syscore_ops);

	bts_debugfs();
	pr_info("BTS: driver is initialized\n");

	return 0;
}
arch_initcall(exynos_bts_init);
