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

#define EXYNOS7885_PA_G3D		0x12460000
#define EXYNOS7885_PA_MFCMSCL		0x124A0000
#define EXYNOS7885_PA_FSYS		0x12450000
#define EXYNOS7885_PA_GNSS		0x12470000
#define EXYNOS7885_PA_ISP0		0x12480000
#define EXYNOS7885_PA_ISP1		0x12490000
#define EXYNOS7885_PA_WLBT		0x124C0000
#define EXYNOS7885_PA_CP		0x124B0000
#define EXYNOS7885_PA_CAM		0x12420000
#define EXYNOS7885_PA_DPU		0x12440000
#define EXYNOS7885_PA_ABOX		0x12400000
#define EXYNOS7885_PA_CPU_DMC0		0x10480000
#define EXYNOS7885_PA_CPU_DMC1		0x10580000
#define EXYNOS7885_PA_DREX0		0x10400000
#define EXYNOS7885_PA_DREX1		0x10500000

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
	bool scen_en;
	unsigned int rmo;
	unsigned int wmo;
	unsigned int full_rmo;
	unsigned int full_wmo;
	unsigned int busy_rmo;
	unsigned int busy_wmo;
	unsigned int max_rmo;
	unsigned int max_wmo;
	unsigned int timeout;
};

void bts_setqos(void __iomem *base, struct bts_status *stat);
void bts_showqos(void __iomem *base, struct seq_file *buf);

#endif
