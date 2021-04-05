/* Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * EXYNOS - BTS CAL code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "cal_bts7885.h"
#include <linux/soc/samsung/exynos-soc.h>

#define LOG(x, ...)					\
({							\
	seq_printf(buf, x, ##__VA_ARGS__);		\
})

#define TREX_CON				0x000
#define TREX_TIMEOUT				0x010
#define TREX_BLOCK_IDMASK			0x018
#define TREX_BLOCK_IDVALUE			0x01C
#define TREX_RCON				0x020
#define TREX_WCON				0x040
#define TREX_RBLOCK_UPPER			0x024
#define TREX_WBLOCK_UPPER			0x044
#define TREX_RBLOCK_UPPER_NORMAL		0x028
#define TREX_WBLOCK_UPPER_NORMAL		0x048
#define TREX_RBLOCK_UPPER_FULL			0x02C
#define TREX_WBLOCK_UPPER_FULL			0x04C
#define TREX_RBLOCK_UPPER_BUSY			0x030
#define TREX_WBLOCK_UPPER_BUSY			0x050
#define TREX_RBLOCK_UPPER_MAX			0x034
#define TREX_WBLOCK_UPPER_MAX			0x054

static unsigned int set_mo(unsigned int mo)
{
	if (mo > BTS_MAX_MO || !mo)
		mo = BTS_MAX_MO;
	return mo;
}

void bts_setqos(void __iomem *base, struct bts_status *stat)
{
	unsigned int tmp_reg = 0;
	bool block_en = false;

	if (!base)
		return;

	if (stat->disable) {
		__raw_writel(0x4000, base + TREX_RCON);
		__raw_writel(0x4000, base + TREX_WCON);
		__raw_writel(0x0, base + TREX_CON);
		return;
	}
	__raw_writel(set_mo(stat->rmo), base + TREX_RBLOCK_UPPER);
	__raw_writel(set_mo(stat->wmo), base + TREX_WBLOCK_UPPER);
	if (stat->max_rmo || stat->max_wmo || stat->full_rmo || stat->full_wmo)
		block_en = true;

	__raw_writel(set_mo(stat->max_rmo), base + TREX_RBLOCK_UPPER_MAX);
	__raw_writel(set_mo(stat->max_wmo), base + TREX_WBLOCK_UPPER_MAX);
	__raw_writel(set_mo(stat->full_rmo), base + TREX_RBLOCK_UPPER_FULL);
	__raw_writel(set_mo(stat->full_wmo), base + TREX_WBLOCK_UPPER_FULL);
	/* override QoS value */
	tmp_reg |= (1 & !stat->bypass_en) << 8;
	tmp_reg |= (stat->priority & 0xf) << 12;

	/* enable Blocking logic */
	tmp_reg |= (1 & block_en) << 0;
	__raw_writel(tmp_reg, base + TREX_RCON);
	__raw_writel(tmp_reg, base + TREX_WCON);

	__raw_writel(((1 & stat->timeout_en) << 20) | 0x1, base + TREX_CON);
}

void bts_showqos(void __iomem *base, struct seq_file *buf)
{
	if (!base)
		return;

	LOG("CON0x%08X qos(%d,%d)0x%Xr%Xw, wmo: %d, rmo: %d\n",
			 __raw_readl(base + TREX_CON),
			(__raw_readl(base + TREX_RCON) >> 8) & 0x1,
			(__raw_readl(base + TREX_WCON) >> 8) & 0x1,
			(__raw_readl(base + TREX_RCON) >> 12) & 0xf,
			(__raw_readl(base + TREX_WCON) >> 12) & 0xf,
			(__raw_readl(base + TREX_WBLOCK_UPPER)),
			(__raw_readl(base + TREX_RBLOCK_UPPER))
	   );
}
