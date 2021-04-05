/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * IPs Traffic Monitor(ITMON) Driver for Samsung Exynos7872 SOC
 * By Hosung Kim (hosung0.kim@samsung.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/bitops.h>
#include <soc/samsung/exynos-itmon.h>

#define OFFSET_TMOUT_REG		(0x2000)
#define OFFSET_REQ_R			(0x0)
#define OFFSET_REQ_W			(0x20)
#define OFFSET_RESP_R			(0x40)
#define OFFSET_RESP_W			(0x60)
#define OFFSET_ERR_REPT			(0x20)
#define OFFSET_NUM			(0x4)

#define REG_INT_MASK			(0x0)
#define REG_INT_CLR			(0x4)
#define REG_INT_INFO			(0x8)
#define REG_EXT_INFO_0			(0x10)
#define REG_EXT_INFO_1			(0x14)
#define REG_EXT_INFO_2			(0x18)

#define REG_DBG_CTL			(0x10)
#define REG_TMOUT_INIT_VAL		(0x14)
#define REG_TMOUT_FRZ_EN		(0x18)
#define REG_TMOUT_BUF_WR_OFFSET 	(0x20)

#define REG_TMOUT_BUF_POINT_ADDR	(0x20)
#define REG_TMOUT_BUF_ID		(0x24)
#define REG_TMOUT_BUF_PAYLOAD		(0x28)
#define REG_TMOUT_BUF_PAYLOAD_SRAM1	(0x30)
#define REG_TMOUT_BUF_PAYLOAD_SRAM2	(0x34)
#define REG_TMOUT_BUF_PAYLOAD_SRAM3	(0x38)

#define BIT_ERR_CODE(x)			(((x) & (0xF << 28)) >> 28)
#define BIT_ERR_OCCURRED(x)		(((x) & (0x1 << 27)) >> 27)
#define BIT_ERR_VALID(x)		(((x) & (0x1 << 26)) >> 26)
#define BIT_AXID(x)			(((x) & (0xFFFF)))
#define BIT_AXUSER(x)			(((x) & (0xFFFF << 16)) >> 16)
#define BIT_AXBURST(x)			(((x) & (0x3)))
#define BIT_AXPROT(x)			(((x) & (0x3 << 2)) >> 2)
#define BIT_AXLEN(x)			(((x) & (0xF << 16)) >> 16)
#define BIT_AXSIZE(x)			(((x) & (0x7 << 28)) >> 28)

#define M_NODE				(0)
#define T_S_NODE			(1)
#define T_M_NODE			(2)
#define S_NODE				(3)
#define NODE_TYPE			(4)

#define ERRCODE_SLVERR			(0)
#define ERRCODE_DECERR			(1)
#define ERRCODE_UNSUPORTED		(2)
#define ERRCODE_POWER_DOWN		(3)
#define ERRCODE_UNKNOWN_4		(4)
#define ERRCODE_UNKNOWN_5		(5)
#define ERRCODE_TMOUT			(6)

#define BUS_DATA			(0)
#define BUS_PERI			(1)
#define BUS_PATH_TYPE			(2)

#define TRANS_TYPE_WRITE		(0)
#define TRANS_TYPE_READ			(1)
#define TRANS_TYPE_NUM			(2)

#define FROM_PERI			(0)
#define FROM_CPU			(1)
#define FROM_CP				(2)

#define TMOUT				(0xFFFF)
#define TMOUT_TEST			(0x1)

#define PANIC_ALLOWED_THRESHOLD		(0x2)
#define INVALID_REMAPPING		(0x08000000)
#define BAAW_RETURN			(0x08000000)

static bool initial_multi_irq_enable = false;
static struct itmon_dev *g_itmon = NULL;

struct itmon_rpathinfo {
	unsigned int id;
	char *port_name;
	char *dest_name;
	unsigned int bits;
	unsigned int shift_bits;
};

struct itmon_masterinfo {
	char *port_name;
	unsigned int user;
	char *master_name;
	unsigned int bits;
};

struct itmon_nodegroup;

struct itmon_traceinfo {
	char* port;
	char* master;
	char* dest;
	unsigned long target_addr;
	unsigned int errcode;
	bool read;
	bool path_dirty;
	bool snode_dirty;
	bool dirty;
	unsigned long from;
	char buf[SZ_32];
};

struct itmon_tracedata {
	unsigned int int_info;
	unsigned int ext_info_0;
	unsigned int ext_info_1;
	unsigned int ext_info_2;
	unsigned int offset;
	bool logging;
	bool read;
};

struct itmon_nodeinfo {
	unsigned int type;
	char *name;
	unsigned int phy_regs;
	void __iomem *regs;
	unsigned int time_val;
	bool tmout_enabled;
	bool tmout_frz_enabled;
	bool err_enabled;
	bool retention;
	struct itmon_tracedata tracedata;
	struct itmon_nodegroup *group;
	struct list_head list;
};

const static char *itmon_pathtype[] = {
	"DATA Path transaction (0x2000_0000 ~ 0xf_ffff_ffff)",
	"PERI(SFR) Path transaction (0x0 ~ 0x1fff_ffff)",
};

/* Error Code Description */
const static char *itmon_errcode[] = {
	"Error Detect by the Slave(SLVERR)",
	"Decode error(DECERR)",
	"Unsupported transaction error",
	"Power Down access error",
	"Unsupported transaction",
	"Unsupported transaction",
	"Timeout error - response timeout in timeout value",
	"Invalid errorcode",
};

const static char *itmon_nodestring[] = {
	"M_NODE",
	"TAXI_S_NODE",
	"TAXI_M_NODE",
	"S_NODE",
};

struct itmon_nodegroup {
	int irq;
	char *name;
	unsigned int phy_regs;
	void __iomem *regs;
	struct itmon_nodeinfo *nodeinfo;
	unsigned int nodesize;
	unsigned int bus_type;
};

struct itmon_platdata {
	const struct itmon_rpathinfo *rpathinfo;
	const struct itmon_masterinfo *masterinfo;
	struct itmon_nodegroup *nodegroup;
	struct itmon_traceinfo traceinfo[BUS_PATH_TYPE];
	struct list_head tracelist[BUS_PATH_TYPE];
	unsigned int err_cnt;
	bool panic_allowed;
	bool crash_in_progress;
	bool probed;
};

static struct itmon_rpathinfo rpathinfo[] = {
	/* Data BUS */
	{0,	"MFCMSCL",	"DREX_NRT",	GENMASK(2, 0),	3},
	{1,	"G3D",		"DREX_NRT",	GENMASK(2, 0),	3},
	{2,	"FSYS",		"DREX_NRT",	GENMASK(2, 0),	3},
	{3,	"APM",		"DREX_NRT",	GENMASK(2, 0),	3},
	{4,	"ISNRT",	"DREX_NRT",	GENMASK(2, 0),	3},
	{5,	"GNSS",		"DREX_NRT",	GENMASK(2, 0),	3},
	{6,	"WLBT",		"DREX_NRT",	GENMASK(2, 0),	3},

	{0,	"ISRT",		"DREX_RT",	GENMASK(2, 0),	3},
	{1,	"DPU",		"DREX_RT",	GENMASK(2, 0),	3},
	{2,	"ABOX",		"DREX_RT",	GENMASK(2, 0),	3},
	{3,	"GNSS",		"DREX_RT",	GENMASK(2, 0),	3},
	{4,	"WLBT",		"DREX_RT",	GENMASK(2, 0),	3},

	{0,	"CP",		"DREX_CP",	GENMASK(1, 0),	2},
	{1,	"WLBT",		"DREX_CP",	GENMASK(1, 0),	2},
	{2,	"ABOX",		"DREX_CP",	GENMASK(1, 0),	2},

	{0,	"MFCMSCL",	"PERI",		GENMASK(3, 0),	4},
	{1,	"G3D",		"PERI",		GENMASK(3, 0),	4},
	{2,	"FSYS",		"PERI",		GENMASK(3, 0),	4},
	{3,	"APM",		"PERI",		GENMASK(3, 0),	4},
	{4,	"ISNRT",	"PERI",		GENMASK(3, 0),	4},
	{5,	"GNSS",		"PERI",		GENMASK(3, 0),	4},
	{6,	"ISRT",		"PERI",		GENMASK(3, 0),	4},
	{7,	"DPU",		"PERI",		GENMASK(3, 0),	4},
	{8,	"ABOX",		"PERI",		GENMASK(3, 0),	4},
	{9,	"CP",		"PERI",		GENMASK(3, 0),	4},
	{10,	"WLBT",		"PERI",		GENMASK(3, 0),	4},

	/* Peri BUS */
	{0,	"PERI_TO_SFR",	"PERI",		GENMASK(1, 0),	2},
	{1,	"CPU_TO_SFR",	"PERI",		GENMASK(1, 0),  2},
	{2,	"CP_TO_SFR",	"PERI",		GENMASK(1, 0),	2},
};

/* XIU ID Information */
static struct itmon_masterinfo masterinfo[] = {
	/* BLK_MFCMSCL */
	{"MFCMSCL",	0,				"JPEG",		GENMASK(2, 1)},
	{"MFCMSCL",	BIT(1),				"MSCL",		GENMASK(2, 1)},
	{"MFCMSCL",	BIT(2),				"G2D",		GENMASK(2, 1)},
	{"MFCMSCL",	BIT(1) | BIT(2),		"MFC",		GENMASK(2, 1)},

	/* BLK_G3D - Unique ID */
	{"G3D",		0,				"",		0},

	/* BLK_FSYS */
	{"FSYS",	0,				"MMC",		GENMASK(4, 0)},
	{"FSYS",	BIT(2),				"MMC_CARD",	GENMASK(4, 0)},
	{"FSYS",	BIT(3),				"USB",		GENMASK(4, 0)},
	{"FSYS",	BIT(2) | BIT(3),		"SSS",		GENMASK(4, 0)},
	{"FSYS",	BIT(4),				"RTIC",		GENMASK(4, 0)},
	{"FSYS",	BIT(0),				"PDMA",		GENMASK(1, 0)},
	{"FSYS",	BIT(1),				"SPDMA",	GENMASK(1, 0)},

	/* BLK_APM */
	{"APM",		0,				"",		0},

	/* BLK_ISNRT */
	{"ISNRT",	0,				"CSISX2",	GENMASK(3, 1)},
	{"ISNRT",	BIT(1),				"3AA",		GENMASK(3, 1)},
	{"ISNRT",	BIT(2),				"ISP",		GENMASK(3, 1)},
	{"ISNRT",	BIT(1) | BIT(2),		"MCSC",		GENMASK(3, 1)},
	{"ISNRT",	BIT(3),				"VRA",		GENMASK(3, 1)},

	/* BLK_GNSS */
	{"GNSS",	0,				"CM0+",		GENMASK(1, 0)},
	{"GNSS",	BIT(0),				"XDMAC0",	GENMASK(1, 0)},
	{"GNSS",	BIT(1),				"XDMAC1",	GENMASK(1, 0)},

	/* BLK_ISRT */
	{"ISRT",	0,				"CSISX2",	GENMASK(3, 1)},
	{"ISRT",	BIT(1),				"3AA",		GENMASK(3, 1)},
	{"ISRT",	BIT(2),				"ISP",		GENMASK(3, 1)},
	{"ISRT",	BIT(1) | BIT(2),		"MCSC",		GENMASK(3, 1)},
	{"ISRT",	BIT(3),				"VRA",		GENMASK(3, 1)},

	/* BLK_DPU */
	{"DPU",		0,				"IDMA0",	GENMASK(1, 1)},
	{"DPU",		BIT(1),				"ABOX",		GENMASK(1, 1)},

	/* BLK_CP */
	{"CP",		0,				"CR7M",		GENMASK(3, 0)},
	{"CP",		BIT(0) | BIT(1),		"DMAtoL2",	GENMASK(3, 0)},
	{"CP",		BIT(1) | BIT(3),		"DCPUMtoL2",	GENMASK(3, 0)},
	{"CP",		BIT(0) | BIT(3),		"LMACtoL2",	GENMASK(3, 0)},
	{"CP",		BIT(2),				"CSXAP",	GENMASK(9, 0)},
	{"CP",		BIT(0) | BIT(1) | BIT(2),	"MDMtoL2",	GENMASK(3, 0)},
	{"CP",		BIT(0) | BIT(2) | BIT(3),	"HARQMOVERtoL2",GENMASK(9, 0)},

	/* BLK_WLBT */
	{"WLBT",	0,				"SXCR4",	GENMASK(2, 0)},
	{"WLBT",	BIT(0),				"SXDMA",	GENMASK(5, 0)},
	{"WLBT",	BIT(1),				"SXDBG",	GENMASK(5, 0)},
	{"WLBT",	BIT(0) | BIT(1),		"SHDMA",	GENMASK(5, 0)},
	{"WLBT",	BIT(2),				"SXCM4",	GENMASK(5, 0)},

	/* CP_PERI */
	{"CP_TO_SFR",	BIT(5),				"CR7M",		GENMASK(5, 5)},
	{"CP_TO_SFR",	BIT(4),				"DMAtoL2",	GENMASK(4, 2)},
	{"CP_TO_SFR",	0,				"CSXAP",	GENMASK(5, 0)},
	{"CP_TO_SFR",	BIT(3) | BIT(4),		"CR7MP",	GENMASK(5, 3)},
	{"CP_TO_SFR",	BIT(3),				"CR4MtoL2",	GENMASK(5, 3)},

	{"CPU_TO_SFR",	0,				"",		0},
	{"PERI_TO_SFR",	0,				"",		0},
};

/* data_path is sorted by INT_VEC_DEBUG_INTERRUPT_VECTOR_TABLE bits */
static struct itmon_nodeinfo data_path[] = {
	{M_NODE, "ABOX",		0x12463000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "APM",			0x124A3000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "CP",			0x12483000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "DPU",			0x12453000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "FSYS",		0x12423000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "G3D",			0x12413000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "GNSS",		0x12473000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "ISNRT",		0x12443000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "ISRT",		0x12433000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "MFCMSCL",		0x12403000, NULL, 0,	   false, false,  true, false},
	{M_NODE, "WLBT",		0x12493000, NULL, 0,	   false, false,  true, false},

	{S_NODE, "DREX_CP",		0x124D3000, NULL, TMOUT,   true,  false,  true, false},
	{S_NODE, "DREX_NRT",		0x124B3000, NULL, TMOUT,   true,  false,  true, false},
	{S_NODE, "DREX_RT",		0x124C3000, NULL, TMOUT,   true,  false,  true, false},
	{S_NODE, "PERI",		0x124E3000, NULL, TMOUT,   true,  false,  true, false},
};

/* peri_path is sorted by INT_VEC_DEBUG_INTERRUPT_VECTOR_TABLE bits */
static struct itmon_nodeinfo peri_path[] = {
	{M_NODE, "CP_TO_SFR",		0x12613000, NULL, 0,	   false, false, true, false},
	{M_NODE, "CPU_TO_SFR",		0x12603000, NULL, 0,	   false, false, true, false},
	{M_NODE, "PERI_TO_SFR",		0x12623000, NULL, 0,	   false, false, true, false},

	{S_NODE, "APM_SFR",		0x126D3000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "CORE_SFR",		0x126C3000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "CORE_SFR_TREX",	0x126B3000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "CPU_CL0_SFR",		0x12633000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "CPU_CL1_SFR",		0x12643000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "CSSYS_SFR",		0x126E3000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "DPU_SFR",		0x12653000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "FSYS_SFR",		0x12683000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "G3D_SFR",		0x12673000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "GIC_SFR",		0x126F3000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "ISP_SFR",		0x12663000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "MFCMSCL_SFR",		0x12693000, NULL, TMOUT,   true,  false, true, false},
	{S_NODE, "PERI_SFR",		0x126A3000, NULL, TMOUT,   true,  false, true, false},
};

static struct itmon_nodegroup nodegroup[] = {
	{0, "ITMON_DATA",	0x125F3000, NULL, data_path, ARRAY_SIZE(data_path), BUS_DATA},
	{0, "ITMON_PERI",	0x127F3000, NULL, peri_path, ARRAY_SIZE(peri_path), BUS_PERI},
};

struct itmon_dev {
	struct device *dev;
	struct itmon_platdata *pdata;
	struct of_device_id *match;
	int irq;
	int id;
	void __iomem *regs;
	spinlock_t ctrl_lock;
	struct itmon_notifier notifier_info;
};

struct itmon_panic_block {
	struct notifier_block nb_panic_block;
	struct itmon_dev *pdev;
};

/* declare notifier_list */
static ATOMIC_NOTIFIER_HEAD(itmon_notifier_list);

static const struct of_device_id itmon_dt_match[] = {
	{.compatible = "samsung,exynos-itmon",
	 .data = NULL,},
	{},
};
MODULE_DEVICE_TABLE(of, itmon_dt_match);

static struct itmon_rpathinfo *itmon_get_rpathinfo(struct itmon_dev *itmon,
					       unsigned int id,
					       char *dest_name)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_rpathinfo *rpath = NULL;
	int i;

	if (!dest_name)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(rpathinfo); i++) {
		if (pdata->rpathinfo[i].id == (id & pdata->rpathinfo[i].bits)) {
			if (dest_name && !strncmp(pdata->rpathinfo[i].dest_name,
						  dest_name,
						  strlen(pdata->rpathinfo[i].dest_name))) {
				rpath = (struct itmon_rpathinfo *)&pdata->rpathinfo[i];
				break;
			}
		}
	}
	return rpath;
}

static struct itmon_masterinfo *itmon_get_masterinfo(struct itmon_dev *itmon,
						 char *port_name,
						 unsigned int user)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_masterinfo *master = NULL;
	unsigned int val;
	int i;

	if (!port_name)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(masterinfo); i++) {
		if (!strncmp(pdata->masterinfo[i].port_name, port_name, strlen(port_name))) {
			val = user & pdata->masterinfo[i].bits;
			if (val == pdata->masterinfo[i].user) {
				master = (struct itmon_masterinfo *)&pdata->masterinfo[i];
				break;
			}
		}
	}
	return master;
}

static void itmon_init(struct itmon_dev *itmon, bool enabled)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *node;
	unsigned int offset;
	int i, j;

	for (i = 0; i < ARRAY_SIZE(nodegroup); i++) {
		node = pdata->nodegroup[i].nodeinfo;
		for (j = 0; j < pdata->nodegroup[i].nodesize; j++) {
			if (node[j].type == S_NODE && node[j].tmout_enabled) {
				offset = OFFSET_TMOUT_REG;
				/* Enable Timeout setting */
				__raw_writel(enabled, node[j].regs + offset + REG_DBG_CTL);
				/* set tmout interval value */
				__raw_writel(node[j].time_val,
					     node[j].regs + offset + REG_TMOUT_INIT_VAL);
				pr_debug("Exynos ITMON - %s timeout enabled\n", node[j].name);
				if (node[j].tmout_frz_enabled) {
					/* Enable freezing */
					__raw_writel(enabled,
						     node[j].regs + offset + REG_TMOUT_FRZ_EN);
				}
			}
			if (node[j].err_enabled) {
				/* clear previous interrupt of req_read */
				offset = OFFSET_REQ_R;
				if (!pdata->probed || !node->retention)
					__raw_writel(1, node[j].regs + offset + REG_INT_CLR);
				/* enable interrupt */
				__raw_writel(enabled, node[j].regs + offset + REG_INT_MASK);

				/* clear previous interrupt of req_write */
				offset = OFFSET_REQ_W;
				if (pdata->probed || !node->retention)
					__raw_writel(1, node[j].regs + offset + REG_INT_CLR);
				/* enable interrupt */
				__raw_writel(enabled, node[j].regs + offset + REG_INT_MASK);

				/* clear previous interrupt of response_read */
				offset = OFFSET_RESP_R;
				if (!pdata->probed || !node->retention)
					__raw_writel(1, node[j].regs + offset + REG_INT_CLR);
				/* enable interrupt */
				__raw_writel(enabled, node[j].regs + offset + REG_INT_MASK);

				/* clear previous interrupt of response_write */
				offset = OFFSET_RESP_W;
				if (!pdata->probed || !node->retention)
					__raw_writel(1, node[j].regs + offset + REG_INT_CLR);
				/* enable interrupt */
				__raw_writel(enabled, node[j].regs + offset + REG_INT_MASK);
				pr_debug("Exynos ITMON - %s error reporting enabled\n", node[j].name);
			}
		}
	}
}

void itmon_enable(bool enabled)
{
	if (g_itmon) {
		itmon_init(g_itmon, enabled);
	}
}

void itmon_set_errcnt(int cnt)
{
	struct itmon_platdata *pdata;
	if (g_itmon) {
		pdata = g_itmon->pdata;
		pdata->err_cnt = cnt;
	}
}

static void itmon_post_handler_to_notifier(struct itmon_dev *itmon,
					   unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_traceinfo *traceinfo = &pdata->traceinfo[trans_type];

	/* After treatment by port */
	if (!traceinfo->port || strlen(traceinfo->port) < 1)
		return;

	itmon->notifier_info.port = traceinfo->port;
	itmon->notifier_info.master = traceinfo->master;
	itmon->notifier_info.dest = traceinfo->dest;
	itmon->notifier_info.read = traceinfo->read;
	itmon->notifier_info.target_addr = traceinfo->target_addr;
	itmon->notifier_info.errcode = traceinfo->errcode;

	/* call notifier_call_chain of itmon */
	atomic_notifier_call_chain(&itmon_notifier_list, 0, &itmon->notifier_info);
}

static void itmon_post_handler_by_master(struct itmon_dev *itmon,
					unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_traceinfo *traceinfo = &pdata->traceinfo[trans_type];

	/* After treatment by port */
	if (!traceinfo->port || strlen(traceinfo->port) < 1)
		return;

	if (!strncmp(traceinfo->port, "CPU", strlen("CPU"))) {
		/* if master is CPU, then we expect any exception */
		if (pdata->err_cnt > PANIC_ALLOWED_THRESHOLD) {
			pdata->err_cnt = 0;
			itmon_init(itmon, false);
			pr_info("ITMON is turn-off when CPU transaction is detected repeatly\n");
		} else {
			pr_info("ITMON skips CPU transaction detected\n");
		}
	} else if (!strncmp(traceinfo->port, "CP", strlen("CP"))) {
		/* if master is DSP and operation is read, we don't care this */
		if (traceinfo->master && traceinfo->target_addr == INVALID_REMAPPING &&
			!strncmp(traceinfo->master, "CR4MtoL2", strlen(traceinfo->master))) {
			pdata->err_cnt = 0;
			pr_info("ITMON skips CP's DSP(CR4MtoL2) detected\n");
		} else {
			/* Disable busmon all interrupts */
			itmon_init(itmon, false);
			/* TODO: CP Crash operation */
		}
	}
}

void itmon_report_timeout(struct itmon_dev *itmon,
				struct itmon_nodeinfo *node,
				unsigned int trans_type)
{
	unsigned int info, axid, valid, timeout, payload;
	unsigned long addr;
	char *master_name, *port_name;
	struct itmon_rpathinfo *port;
	struct itmon_masterinfo *master;
	int i, num = (trans_type == TRANS_TYPE_READ ? SZ_128 : SZ_64);
	int fz_offset = (trans_type == TRANS_TYPE_READ ? 0 : REG_TMOUT_BUF_WR_OFFSET);

	pr_info("\n      TIMEOUT_BUFFER Information\n\n");
	pr_info("      > NUM|   BLOCK|  MASTER|   VALID| TIMEOUT|      ID|   ADDRESS|    INFO|\n");

	for (i = 0; i < num; i++) {
		writel(i, node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_POINT_ADDR + fz_offset);
		axid = readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_ID + fz_offset);
		payload = readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_PAYLOAD + fz_offset);
		addr = (((unsigned long)readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_PAYLOAD_SRAM1 + fz_offset) &
				GENMASK(15, 0)) << 32ULL);
		addr |= (readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_PAYLOAD_SRAM2 + fz_offset));
		info = readl(node->regs + OFFSET_TMOUT_REG +
				REG_TMOUT_BUF_PAYLOAD_SRAM3 + fz_offset);

		valid = payload & BIT(0);
		timeout = (payload & GENMASK(19, 16)) >> 16;

		port = (struct itmon_rpathinfo *)
				itmon_get_rpathinfo(itmon, axid, node->name);
		if (port) {
			port_name = port->port_name;
			master = (struct itmon_masterinfo *)
				itmon_get_masterinfo(itmon, port_name,
							axid >> port->shift_bits);
			if (master)
				master_name = master->master_name;
			else
				master_name = "Unknown";
		} else {
			port_name = "Unknown";
			master_name = "Unknown";
		}
		pr_info("      > %03d|%8s|%8s|%8u|%8x|%08x|%010zx|%08x|\n",
				i, port_name, master_name, valid, timeout, axid, addr, info);
	}
	pr_info("--------------------------------------------------------------------------\n");
}

static unsigned int power(unsigned int param, unsigned int num)
{
	if (num == 0)
		return 1;
	return param * (power(param, num - 1));
}

static void itmon_report_traceinfo(struct itmon_dev *itmon,
				struct itmon_nodeinfo *node,
				unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_traceinfo *traceinfo = &pdata->traceinfo[trans_type];
	struct itmon_nodegroup *group = NULL;

	if (!traceinfo->dirty)
		return;

	pr_info("--------------------------------------------------------------------------\n"
		"      Transaction Information\n\n"
		"      > Master         : %s %s\n"
		"      > Target         : %s\n"
		"      > Target Address : 0x%lX %s\n"
		"      > Type           : %s\n"
		"      > Error code     : %s\n",
		traceinfo->port, traceinfo->master ? traceinfo->master : "",
		traceinfo->dest ? traceinfo->dest : "Unknown",
		traceinfo->target_addr,
		(unsigned int)traceinfo->target_addr == INVALID_REMAPPING ?
		"(Violation Access by CP maybe)" : "",
		trans_type == TRANS_TYPE_READ ? "READ" : "WRITE",
		itmon_errcode[traceinfo->errcode]);

	if (node) {
		struct itmon_tracedata *tracedata = &node->tracedata;
		pr_info("      > Size           : %u bytes x %u burst => %u bytes\n"
			"      > Burst Type     : %u (0:FIXED, 1:INCR, 2:WRAP)\n"
			"      > Level          : %s\n"
			"      > Protection     : %s\n",
			power(BIT_AXSIZE(tracedata->ext_info_1), 2), BIT_AXLEN(tracedata->ext_info_1) + 1,
			power(BIT_AXSIZE(tracedata->ext_info_1), 2) * (BIT_AXLEN(tracedata->ext_info_1) + 1),
			BIT_AXBURST(tracedata->ext_info_2),
			(BIT_AXPROT(tracedata->ext_info_2) & 0x1) ? "Privileged access" : "Unprivileged access",
			(BIT_AXPROT(tracedata->ext_info_2) & 0x2) ? "Non-secure access" : "Secure access");

		group = node->group;
		pr_info("      > Path Type      : %s\n"
			"--------------------------------------------------------------------------\n",
			itmon_pathtype[group->bus_type]);

	} else {
		pr_info("--------------------------------------------------------------------------\n");
	}
}

static void itmon_report_pathinfo(struct itmon_dev *itmon,
				  struct itmon_nodeinfo *node,
				  unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_tracedata *tracedata = &node->tracedata;
	struct itmon_traceinfo *traceinfo = &pdata->traceinfo[trans_type];

	if (!traceinfo->path_dirty) {
		pr_info("--------------------------------------------------------------------------\n"
			"      ITMON Report (%s)\n"
			"--------------------------------------------------------------------------\n"
			"      PATH Information\n",
			trans_type == TRANS_TYPE_READ ? "READ" : "WRITE");
		traceinfo->path_dirty = true;
	}
	switch (node->type) {
	case M_NODE:
		pr_info("      > %14s, %8s(0x%08X)\n",
			node->name, "M_NODE", node->phy_regs + tracedata->offset);
		break;
	case T_S_NODE:
		pr_info("      > %14s, %8s(0x%08X)\n",
			node->name, "T_S_NODE", node->phy_regs + tracedata->offset);
		break;
	case T_M_NODE:
		pr_info("      > %14s, %8s(0x%08X)\n",
			node->name, "T_M_NODE", node->phy_regs + tracedata->offset);
		break;
	case S_NODE:
		pr_info("      > %14s, %8s(0x%08X)\n",
			node->name, "S_NODE", node->phy_regs + tracedata->offset);
		break;
	}
}

static void itmon_report_tracedata(struct itmon_dev *itmon,
				   struct itmon_nodeinfo *node,
				   unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_tracedata *tracedata = &node->tracedata;
	struct itmon_traceinfo *traceinfo = &pdata->traceinfo[trans_type];
	struct itmon_nodegroup *group = node->group;
	struct itmon_masterinfo *master;
	struct itmon_rpathinfo *port;
	unsigned int errcode, axid, val;

	errcode = BIT_ERR_CODE(tracedata->int_info);
	axid = BIT_AXID(tracedata->int_info);

	switch (node->type) {
	case M_NODE:
		/* In this case, we can get information from M_NODE
		 * Fill traceinfo->port / target_addr / read / master */
		if (BIT_ERR_VALID(tracedata->int_info) && tracedata->ext_info_2) {
			/* If only detecting M_NODE only(DECERR) */
			traceinfo->port = node->name;
			master = (struct itmon_masterinfo *)
				itmon_get_masterinfo(itmon, node->name, axid);
			if (master)
				traceinfo->master = master->master_name;
			else
				traceinfo->master = NULL;

			traceinfo->target_addr =
				(((unsigned long)node->tracedata.ext_info_1
				& GENMASK(3, 0)) << 32ULL);
			traceinfo->target_addr |= node->tracedata.ext_info_0;
			traceinfo->read = tracedata->read;
			traceinfo->errcode = errcode;
			traceinfo->dirty = true;
		} else {
			traceinfo->master = NULL;
			traceinfo->target_addr = 0;
			traceinfo->read = tracedata->read;
			traceinfo->port = node->name;
			traceinfo->errcode = errcode;
			traceinfo->dirty = true;
		}
		itmon_report_pathinfo(itmon, node, trans_type);
		break;
	case S_NODE:
		/*
		 * In DECERR case, the follow information was already filled in M_NODE.
		 */
		if (group->bus_type == BUS_PERI) {
			traceinfo->dest = node->name;
			val = axid & (BIT(0) | BIT(1));
			if (val == FROM_CP) {
				master = (struct itmon_masterinfo *)
						itmon_get_masterinfo(itmon, "CP_PERI",
						axid >> 2);
				if (!traceinfo->port)
					traceinfo->port = "CP_TO_PERI";
				if (master)
					traceinfo->master = master->master_name;

			} else if (val == FROM_CPU) {
				if (!traceinfo->port)
					traceinfo->port = "CPU_TO_PERI";
			} else if (val == FROM_PERI) {
				if (!traceinfo->port)
					traceinfo->port = "refer other node information";
			}
		} else {
			/* If it has traceinfo->port, keep previous information */
			port = (struct itmon_rpathinfo *)
				itmon_get_rpathinfo(itmon, axid, node->name);
			if (port) {
				traceinfo->port = port->port_name;
				master = (struct itmon_masterinfo *)
						itmon_get_masterinfo(itmon, traceinfo->port,
							axid >> port->shift_bits);
				if (master)
					traceinfo->master = master->master_name;
			} else {
				if (!traceinfo->port)
					traceinfo->port = "Unknown";
				if (!traceinfo->master)
					traceinfo->master = "Unknown";
			}
		}
		traceinfo->target_addr =
			(((unsigned long)node->tracedata.ext_info_1
			& GENMASK(3, 0)) << 32ULL);
		traceinfo->target_addr |= node->tracedata.ext_info_0;
		traceinfo->errcode = errcode;
		traceinfo->dest = node->name;
		traceinfo->dirty = true;
		traceinfo->snode_dirty = true;
		itmon_report_pathinfo(itmon, node, trans_type);
		itmon_report_traceinfo(itmon, node, trans_type);
		break;
	default:
		pr_info("Unknown Error - offset:%u\n", tracedata->offset);
		break;
	}
}

static void itmon_report_rawdata(struct itmon_dev *itmon,
				 struct itmon_nodeinfo *node,
				 unsigned int trans_type)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_traceinfo *traceinfo = &pdata->traceinfo[trans_type];
	struct itmon_tracedata *tracedata = &node->tracedata;

	/* Output Raw register information */
	pr_info("      > %s(%s, 0x%08X)\n"
		"      > REG(0x08~0x18) : 0x%08X, 0x%08X, 0x%08X, 0x%08X\n",
		node->name, itmon_nodestring[node->type],
		node->phy_regs + tracedata->offset,
		tracedata->int_info,
		tracedata->ext_info_0,
		tracedata->ext_info_1,
		tracedata->ext_info_2);

	/* If node is to DREX S_NODE, Outputing timeout freezing result */
	if (node->type == S_NODE && traceinfo->errcode == ERRCODE_TMOUT)
		itmon_report_timeout(itmon, node, trans_type);
}

static void itmon_route_tracedata(struct itmon_dev *itmon)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_traceinfo *traceinfo;
	struct itmon_nodeinfo *node, *next_node;
	unsigned int trans_type;
	int i;

	/* To call function is sorted by declaration */
	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		for (i = M_NODE; i < NODE_TYPE; i++) {
			list_for_each_entry(node, &pdata->tracelist[trans_type], list) {
				if (i == node->type)
					itmon_report_tracedata(itmon, node, trans_type);
			}
		}
		/* If there is no S_NODE information, check one more */
		traceinfo = &pdata->traceinfo[trans_type];
		if (!traceinfo->snode_dirty)
			itmon_report_traceinfo(itmon, NULL, trans_type);
	}

	if (pdata->traceinfo[TRANS_TYPE_READ].dirty ||
		pdata->traceinfo[TRANS_TYPE_WRITE].dirty)
			pr_info("      Raw Register Information(ITMON Internal Information)\n\n");

	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		for (i = M_NODE; i < NODE_TYPE; i++) {
			list_for_each_entry_safe(node, next_node, &pdata->tracelist[trans_type], list) {
				if (i == node->type) {
					itmon_report_rawdata(itmon, node, trans_type);
					/* clean up */
					list_del(&node->list);
					kfree(node);
				}
			}
		}
	}

	if (pdata->traceinfo[TRANS_TYPE_READ].dirty ||
		pdata->traceinfo[TRANS_TYPE_WRITE].dirty)
		pr_info("--------------------------------------------------------------------------\n");

	for (trans_type = 0; trans_type < TRANS_TYPE_NUM; trans_type++) {
		itmon_post_handler_to_notifier(itmon, trans_type);
		itmon_post_handler_by_master(itmon, trans_type);
	}
}

static void itmon_trace_data(struct itmon_dev *itmon,
			    struct itmon_nodegroup *group,
			    struct itmon_nodeinfo *node,
			    unsigned int offset)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *new_node = NULL;
	unsigned int int_info, info0, info1, info2;
	bool read = TRANS_TYPE_WRITE;
	bool req = false;

	int_info = __raw_readl(node->regs + offset + REG_INT_INFO);
	info0 = __raw_readl(node->regs + offset + REG_EXT_INFO_0);
	info1 = __raw_readl(node->regs + offset + REG_EXT_INFO_1);
	info2 = __raw_readl(node->regs + offset + REG_EXT_INFO_2);

	switch (offset) {
	case OFFSET_REQ_R:
		read = TRANS_TYPE_READ;
		/* fall down */
	case OFFSET_REQ_W:
		req = true;
		/* Only S-Node is able to make log to registers */
		break;
	case OFFSET_RESP_R:
		read = TRANS_TYPE_READ;
		/* fall down */
	case OFFSET_RESP_W:
		req = false;
		/* Only NOT S-Node is able to make log to registers */
		break;
	default:
		pr_info("Unknown Error - node:%s offset:%u\n", node->name, offset);
		break;
	}

	new_node = kmalloc(sizeof(struct itmon_nodeinfo), GFP_ATOMIC);
	if (new_node) {
		/* Fill detected node information to tracedata's list */
		memcpy(new_node, node, sizeof(struct itmon_nodeinfo));
		new_node->tracedata.int_info = int_info;
		new_node->tracedata.ext_info_0 = info0;
		new_node->tracedata.ext_info_1 = info1;
		new_node->tracedata.ext_info_2 = info2;
		new_node->tracedata.offset = offset;
		new_node->tracedata.read = read;
		new_node->group = group;
		if (BIT_ERR_VALID(int_info))
			node->tracedata.logging = true;
		else
			node->tracedata.logging = false;

		list_add(&new_node->list, &pdata->tracelist[read]);
	} else {
		pr_info("failed to kmalloc for %s node %x offset\n",
			node->name, offset);
	}
}

static int itmon_search_node(struct itmon_dev *itmon, struct itmon_nodegroup *group, bool clear)
{
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodeinfo *node = NULL;
	unsigned int val, offset;
	unsigned long vec, flags, bit = 0;
	int i, j, ret = 0;

	spin_lock_irqsave(&itmon->ctrl_lock, flags);
	memset(pdata->traceinfo, 0, sizeof(struct itmon_traceinfo) * 2);
	if (group) {
		/* Processing only this group and select detected node */
		vec = (unsigned long)__raw_readl(group->regs);
		node = group->nodeinfo;
		if (!vec)
			goto exit;

		for_each_set_bit(bit, &vec, group->nodesize) {
			/* exist array */
			for (i = 0; i < OFFSET_NUM; i++) {
				offset = i * OFFSET_ERR_REPT;
				/* Check Request information */
				val = __raw_readl(node[bit].regs + offset + REG_INT_INFO);
				if (BIT_ERR_OCCURRED(val)) {
					/* This node occurs the error */
					itmon_trace_data(itmon, group, &node[bit], offset);
					if (clear)
						__raw_writel(1, node[bit].regs
								+ offset + REG_INT_CLR);
					ret = true;
				}
			}

		}
	} else {
		/* Processing all group & nodes */
		for (i = 0; i < ARRAY_SIZE(nodegroup); i++) {
			group = &nodegroup[i];
			if (group->phy_regs)
				vec = (unsigned long)__raw_readl(group->regs);
			else
				vec = GENMASK(group->nodesize, 0);

			node = group->nodeinfo;
			bit = 0;

			for_each_set_bit(bit, &vec, group->nodesize) {
				for (j = 0; j < OFFSET_NUM; j++) {
					offset = j * OFFSET_ERR_REPT;
					/* Check Request information */
					val = __raw_readl(node[bit].regs + offset + REG_INT_INFO);
					if (BIT_ERR_OCCURRED(val)) {
						/* This node occurs the error */
						itmon_trace_data(itmon, group, &node[bit], offset);
						if (clear)
							__raw_writel(1, node[bit].regs
									+ offset + REG_INT_CLR);
						ret = true;
					}
				}
			}
		}
	}
	itmon_route_tracedata(itmon);
 exit:
	spin_unlock_irqrestore(&itmon->ctrl_lock, flags);
	return ret;
}

static irqreturn_t itmon_irq_handler(int irq, void *data)
{
	struct itmon_dev *itmon = (struct itmon_dev *)data;
	struct itmon_platdata *pdata = itmon->pdata;
	struct itmon_nodegroup *group = NULL;
	bool ret;
	int i;

	/* Search itmon group */
	for (i = 0; i < ARRAY_SIZE(nodegroup); i++) {
		if (irq == nodegroup[i].irq) {
			group = &pdata->nodegroup[i];
			if (group->phy_regs != 0) {
				pr_info("\nITMON Detected: %d irq, %s group, 0x%x vec, err_cnt:%u\n",
					irq, group->name, __raw_readl(group->regs), pdata->err_cnt);
			} else {
				pr_info("\nITMON Detected: %d irq, %s group, err_cnt:%u\n",
					irq, group->name, pdata->err_cnt);
			}
			break;
		}
	}

	ret = itmon_search_node(itmon, NULL, true);
	if (!ret) {
		pr_info("ITMON could not detect any error\n");
	} else {
		if (pdata->err_cnt++ > PANIC_ALLOWED_THRESHOLD)
			pdata->panic_allowed = true;
	}

	if (pdata->panic_allowed)
		panic("ITMON occurs panic to prevent endless log");

	return IRQ_HANDLED;
}

void itmon_notifier_chain_register(struct notifier_block *block)
{
	atomic_notifier_chain_register(&itmon_notifier_list, block);
}

static int itmon_logging_panic_handler(struct notifier_block *nb,
				     unsigned long l, void *buf)
{
	struct itmon_panic_block *itmon_panic = (struct itmon_panic_block *)nb;
	struct itmon_dev *itmon = itmon_panic->pdev;
	int ret;

	if (!IS_ERR_OR_NULL(itmon)) {
		/* Check error has been logged */
		ret = itmon_search_node(itmon, NULL, false);
		if (!ret)
			pr_info("No found error in %s\n", __func__);
		else
			pr_info("Found errors in %s\n", __func__);
	}
	return 0;
}

static int itmon_probe(struct platform_device *pdev)
{
	struct itmon_dev *itmon;
	struct itmon_panic_block *itmon_panic = NULL;
	struct itmon_platdata *pdata;
	struct itmon_nodeinfo *node;
	unsigned int irq_option = 0, irq;
	char *dev_name;
	int ret, i, j;

	itmon = devm_kzalloc(&pdev->dev, sizeof(struct itmon_dev), GFP_KERNEL);
	if (!itmon) {
		dev_err(&pdev->dev, "failed to allocate memory for driver's "
				    "private data\n");
		return -ENOMEM;
	}
	itmon->dev = &pdev->dev;

	spin_lock_init(&itmon->ctrl_lock);

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct itmon_platdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "failed to allocate memory for driver's "
				    "platform data\n");
		return -ENOMEM;
	}
	itmon->pdata = pdata;
	itmon->pdata->masterinfo = masterinfo;
	itmon->pdata->rpathinfo = rpathinfo;
	itmon->pdata->nodegroup = nodegroup;

	for (i = 0; i < ARRAY_SIZE(nodegroup); i++) {
		dev_name = nodegroup[i].name;
		node = nodegroup[i].nodeinfo;

		if (nodegroup[i].phy_regs) {
			nodegroup[i].regs = devm_ioremap_nocache(&pdev->dev,
							 nodegroup[i].phy_regs, SZ_16K);
			if (nodegroup[i].regs == NULL) {
				dev_err(&pdev->dev, "failed to claim register region - %s\n",
					dev_name);
				return -ENOENT;
			}
		}

		if (initial_multi_irq_enable)
			irq_option = IRQF_GIC_MULTI_TARGET;

		irq = irq_of_parse_and_map(pdev->dev.of_node, i);
		nodegroup[i].irq = irq;

		ret = devm_request_irq(&pdev->dev, irq,
				       itmon_irq_handler, irq_option, dev_name, itmon);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request irq - %s\n", dev_name);
			return -ENOENT;
		} else {
			dev_err(&pdev->dev, "success to register request irq%u - %s\n", irq, dev_name);
		}

		for (j = 0; j < nodegroup[i].nodesize; j++) {
			node[j].regs = devm_ioremap_nocache(&pdev->dev, node[j].phy_regs, SZ_16K);
			if (node[j].regs == NULL) {
				dev_err(&pdev->dev, "failed to claim register region - %s\n",
					dev_name);
				return -ENOENT;
			}
		}
	}

	itmon_panic = devm_kzalloc(&pdev->dev, sizeof(struct itmon_panic_block),
				 GFP_KERNEL);

	if (!itmon_panic) {
		dev_err(&pdev->dev, "failed to allocate memory for driver's "
				    "panic handler data\n");
	} else {
		itmon_panic->nb_panic_block.notifier_call = itmon_logging_panic_handler;
		itmon_panic->pdev = itmon;
		atomic_notifier_chain_register(&panic_notifier_list,
					       &itmon_panic->nb_panic_block);
	}

	platform_set_drvdata(pdev, itmon);

	INIT_LIST_HEAD(&pdata->tracelist[BUS_DATA]);
	INIT_LIST_HEAD(&pdata->tracelist[BUS_PERI]);

	pdata->crash_in_progress = false;
	itmon_init(itmon, true);

	g_itmon = itmon;
	pdata->probed = true;

	dev_info(&pdev->dev, "success to probe Exynos ITMON driver\n");

	return 0;
}

static int itmon_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int itmon_suspend(struct device *dev)
{
	return 0;
}

static int itmon_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct itmon_dev *itmon = platform_get_drvdata(pdev);
	struct itmon_platdata *pdata = itmon->pdata;

	/* re-enable ITMON if cp-crash progress is not starting */
	if (!pdata->crash_in_progress)
		itmon_init(itmon, true);

	return 0;
}

static SIMPLE_DEV_PM_OPS(itmon_pm_ops, itmon_suspend, itmon_resume);
#define ITMON_PM	(itmon_pm_ops)
#else
#define ITM_ONPM	NULL
#endif

static struct platform_driver exynos_itmon_driver = {
	.probe = itmon_probe,
	.remove = itmon_remove,
	.driver = {
		   .name = "exynos-itmon",
		   .of_match_table = itmon_dt_match,
		   .pm = &itmon_pm_ops,
		   },
};

module_platform_driver(exynos_itmon_driver);

MODULE_DESCRIPTION("Samsung Exynos ITMON DRIVER");
MODULE_AUTHOR("Hosung Kim <hosung0.kim@samsung.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:exynos-itmon");
