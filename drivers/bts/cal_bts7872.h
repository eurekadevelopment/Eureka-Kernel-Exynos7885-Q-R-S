/* Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * EXYNOS - BTS CAL code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __BTSCAL_H__
#define __BTSCAL_H__

#include <linux/io.h>
#include <linux/debugfs.h>

#define EXYNOS7872_PA_DPU		0x12450000
#define EXYNOS7872_PA_ISRT		0x12430000
#define EXYNOS7872_PA_ISNRT		0x12440000
#define EXYNOS7872_PA_MFCMSCL		0x12400000
#define EXYNOS7872_PA_ABOX		0x12460000
#define EXYNOS7872_PA_FSYS		0x12420000
#define EXYNOS7872_PA_G3D		0x12410000
#define EXYNOS7872_PA_GNSS		0x12470000
#define EXYNOS7872_PA_CP		0x12480000
#define EXYNOS7872_PA_WLBT		0x12490000
#define EXYNOS7872_PA_CPU		0x10550000
#define EXYNOS7872_PA_DREX		0x10400000

#define QOS_TIMEOUT_F			0x33C
#define QOS_TIMEOUT_E			0x338
#define QOS_TIMEOUT_D			0x334
#define QOS_TIMEOUT_C			0x330
#define QOS_TIMEOUT_B			0x32C
#define QOS_TIMEOUT_A			0x328
#define QOS_TIMEOUT_9			0x324
#define QOS_TIMEOUT_8			0x320
#define QOS_TIMEOUT_7			0x31C
#define QOS_TIMEOUT_6			0x318
#define QOS_TIMEOUT_5			0x314
#define QOS_TIMEOUT_4			0x310
#define QOS_TIMEOUT_3			0x30C
#define QOS_TIMEOUT_2			0x308
#define QOS_TIMEOUT_1			0x304

#define BTS_MAX_MO			0xffff

struct bts_status {
	unsigned int priority;
	bool disable;
	bool bypass_en;
	bool timeout_en;
	unsigned int rmo;
	unsigned int wmo;
	unsigned int timeout;
};

void bts_setqos(void __iomem *base, struct bts_status *stat);
void bts_showqos(void __iomem *base, struct seq_file *buf);

#endif
