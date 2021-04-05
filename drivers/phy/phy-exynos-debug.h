/**
 * phy-exynos-debug.h -  Samsung EXYNOS SoC series USB DRD PHY Debug Header
 *
 * Phy provider for USB 3.0 DRD controller on Exynos SoC series
 *
 * Copyright (C) 2016 Samsung Electronics Co., Ltd.
 * Author: Kyounghye Yunm <k-hye.yun@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "phy-exynos-usbdrd.h"
#ifdef CONFIG_DEBUG_FS
extern int exynos_usbdrd_debugfs_init(struct exynos_usbdrd_phy *subdrd_phy);
extern void exynos_usbdrd_debugfs_exit(struct exynos_usbdrd_phy *usbdrd_phy);
#else
static inline int exynos_usbdrd_debugfs_init(struct exynos_usbdrd_phy *usbdrd_phy)
{  return 0;  }
static inline void exynos_usbdrd_debugfs_exit(struct exynos_usbdrd_phy *usbdrd_phy)
{  }
#endif

#define dump_register(nm)		\
{					\
	.name = __stringify(nm),	\
	.offset = EXYNOS_USBCON_##nm,	\
}
#define dump_regmap_mask(nm, bit_nm, bit)	\
{						\
	.name = __stringify(nm),		\
	.offset = EXYNOS_USBCON_##nm,		\
	.bitname = #bit_nm,			\
	.bitmask = nm##_##bit_nm##_MASK,	\
	.bitoffset = bit,			\
	.mask = true,				\
}
#define dump_regmap(nm, bit_nm, bit)	\
{					\
	.name = __stringify(nm),	\
	.offset = EXYNOS_USBCON_##nm,	\
	.bitname = #bit_nm,		\
	.bitmask = nm##_##bit_nm,	\
	.bitoffset = bit,		\
	.mask = false,			\
}

struct debugfs_regmap32 {
	char *name;
	char *bitname;
	unsigned int offset;
	unsigned int bitoffset;
	unsigned int bitmask;
	bool mask;
};

struct debugfs_regset_map {
	const struct debugfs_regmap32 *regs;
	int nregs;
};

struct exynos_debugfs_prvdata {
	struct exynos_usbdrd_phy *phy_drd;
	struct dentry	*root;
	struct debugfs_regset32 *regset;
	struct debugfs_regset_map *regmap;
};
