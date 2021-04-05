/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SCORE_SYSTEM_H_
#define SCORE_SYSTEM_H_

#include <linux/platform_device.h>
#include <linux/delay.h>

#include "score-interface.h"
#include "score-exynos.h"
#include "score-memory.h"
#include "score-binary.h"

#define FW_STACK_SIZE		(0x8000)
#define FW_SRAM_REGION_SIZE	(32 * 1024)

struct score_fw_load_info {
	struct score_binary			binary_text;
	struct score_binary			binary_data;
	void					*fw_text_kvaddr;
	void					*fw_data_kvaddr;
	dma_addr_t				fw_text_dvaddr;
	dma_addr_t				fw_data_dvaddr;
	resource_size_t 			fw_text_size;
	resource_size_t 			fw_data_size;
	enum score_fw_type			fw_type;
};

struct score_system {
	struct platform_device			*pdev;
	void __iomem				*regs;
	struct score_fw_load_info		fw_info[MAX_FW_COUNT];
	resource_size_t 			regs_size;
	int					irq0;

	u32					int_qos;
	u32					mif_qos;

	struct score_interface 			interface;
	struct score_exynos			exynos;
	struct score_memory			memory;

	void					*fw_msg_kvaddr;
	dma_addr_t				fw_msg_dvaddr;
};

int score_system_probe(struct score_system *system, struct platform_device *pdev);
int score_system_open(struct score_system *system);
int score_system_close(struct score_system *system);
int score_system_resume(struct score_system *system);
int score_system_suspend(struct score_system *system);
int score_system_start(struct score_system *system);
int score_system_stop(struct score_system *system);
void score_dump_regs(void __iomem *base_addr, u32 size);

#endif
