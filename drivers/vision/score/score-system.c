/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm_qos.h>

#include "score-config.h"
#include "score-device.h"
#include "score-system.h"
#include "score-interface.h"
#include "score-debug.h"

static unsigned int score_fw_define_table[MAX_FW_COUNT][FW_TABLE_MAX] = {
/*	index,	type,		text size,	data size,	stack size,	in stack size */
/*
*	{0,	FW_TYPE_CACHE,	0x00090000,	0x00168000,	FW_STACK_SIZE,	0x00000000,},
*	{1,	FW_TYPE_CACHE,	0x00090000,	0x00168000,	FW_STACK_SIZE,	0x00000000,},
*	{2,	FW_TYPE_SRAM,	0x00003000,	0x00006000,	0x00004000,	0x00002000,}
*
*	{0,	FW_TYPE_SRAM,	0x00003000,	0x00006000,	0x00004000,	0x00002000,},
*	{1,	FW_TYPE_CACHE,	0x00090000,	0x00168000,	FW_STACK_SIZE,	0x00000000,},
*	{2,	FW_TYPE_CACHE,	0x00090000,	0x00410000,	FW_STACK_SIZE,	0x00000000,}
*/
	{0,	FW_TYPE_CACHE,	0x00200000,	0x00500000,	FW_STACK_SIZE,	0x00000000,},
	{1,	FW_TYPE_SRAM,	0x00008000,	0x00008000,	0x00004000,	0x00002000,}
};

struct pm_qos_request exynos_score_qos_cam;
struct pm_qos_request exynos_score_qos_mem;

void score_dump_regs(void __iomem *base_addr, u32 size)
{
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 32, 4,
		base_addr, size, false);
}

int score_system_probe(struct score_system *system, struct platform_device *pdev)
{
	unsigned int loop = 0;
	int ret = 0;
	struct device *dev;
	struct resource *res;
	void __iomem *iomem;
	int irq;
	unsigned int offset = 0;

	SCORE_TP();
	BUG_ON(!system);
	BUG_ON(!pdev);

	dev = &pdev->dev;
	system->int_qos = 0;
	system->mif_qos = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		probe_err("platform_get_resource(0) is fail(%p)", res);
		ret = PTR_ERR(res);
		goto p_err;
	}

	iomem =  devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(iomem)) {
		probe_err("devm_ioremap_resource(0) is fail(%p)", iomem);
		ret = PTR_ERR(iomem);
		goto p_err;
	}

	system->regs = iomem;
	system->regs_size = resource_size(res);
	score_event_msg("reg : 0x%p\n", iomem);
	score_event_msg("regs_size : 0x%016llx\n", system->regs_size);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		probe_err("platform_get_irq(0) is fail(%d)", irq);
		ret = -EINVAL;
		goto p_err;
	}

#ifdef DBG_HW_SFR
	score_dump_regs(system->regs, 0x70);
#endif

	system->irq0 = irq;

	ret = score_memory_probe(&system->memory, dev);
	if (ret) {
		score_err("score_memory_probe is fail(%d)\n", ret);
		goto p_err;
	}

	ret = score_interface_probe(&system->interface, dev,
		system->regs, system->regs_size,
		system->irq0);
	if (ret) {
		score_err("score_interface_probe is fail(%d)\n", ret);
		goto p_err;
	}

	offset = 0;
	for (loop = 0; loop < MAX_FW_COUNT; loop ++) {
		system->fw_info[loop].fw_text_kvaddr = (void *)system->memory.info.kvaddr + offset;
		system->fw_info[loop].fw_text_dvaddr = system->memory.info.dvaddr + offset;
		system->fw_info[loop].fw_text_size = score_fw_define_table[loop][FW_TABLE_TEXT_SIZE];

		offset = offset +
			score_fw_define_table[loop][FW_TABLE_TEXT_SIZE] +
			score_fw_define_table[loop][FW_TABLE_STACK_SIZE] -
			score_fw_define_table[loop][FW_TABLE_IN_STACK_SIZE];

		system->fw_info[loop].fw_data_kvaddr = (void *)system->memory.info.kvaddr + offset;
		system->fw_info[loop].fw_data_dvaddr = system->memory.info.dvaddr + offset;
		system->fw_info[loop].fw_data_size = score_fw_define_table[loop][FW_TABLE_DATA_SIZE];

		system->fw_info[loop].fw_type = score_fw_define_table[loop][FW_TABLE_TYPE];

		offset = offset +
			score_fw_define_table[loop][FW_TABLE_DATA_SIZE];

		score_event_msg("(loop %d)(0x%x 0x%x 0x%x 0x%x) (offset 0x%x) \n",
			loop,
			score_fw_define_table[loop][FW_TABLE_TEXT_SIZE],
			score_fw_define_table[loop][FW_TABLE_DATA_SIZE],
			score_fw_define_table[loop][FW_TABLE_STACK_SIZE],
			score_fw_define_table[loop][FW_TABLE_IN_STACK_SIZE],
			offset);
		score_event_msg("(loop %d)(0x%p 0x%llX 0x%x)(0x%p 0x%llX 0x%x)(%d)\n",
			loop,
			system->fw_info[loop].fw_text_kvaddr,
			system->fw_info[loop].fw_text_dvaddr,
			(unsigned int)system->fw_info[loop].fw_text_size,
			system->fw_info[loop].fw_data_kvaddr,
			system->fw_info[loop].fw_data_dvaddr,
			(unsigned int)system->fw_info[loop].fw_data_size,
			system->fw_info[loop].fw_type);

		ret = score_binary_init(&system->fw_info[loop].binary_text, dev,
				system->fw_info[loop].fw_text_kvaddr, system->fw_info[loop].fw_text_size);
		if (ret) {
			score_err("score_binary init text is fail(%d)\n", ret);
			goto p_err;
		}

		ret = score_binary_init(&system->fw_info[loop].binary_data, dev,
				system->fw_info[loop].fw_data_kvaddr, system->fw_info[loop].fw_data_size);
		if (ret) {
			score_err("score_binary init data is fail(%d)\n", ret);
			goto p_err;
		}

		ret = score_binary_set_text_path(&system->fw_info[loop].binary_text, loop, dev);
		if (ret) {
			score_err("score_binary set text path is fail(%d)\n", ret);
			goto p_err;
		}

		ret = score_binary_set_data_path(&system->fw_info[loop].binary_data, loop, dev);
		if (ret) {
			score_err("score_binary set data path is fail(%d)\n", ret);
			goto p_err;
		}
	}

	system->fw_msg_kvaddr = (void *)system->memory.info.kvaddr + \
		SCORE_MEMORY_INTERNAL_SIZE - SCORE_MEMORY_FW_MSG_SIZE;
	system->fw_msg_dvaddr = system->memory.info.dvaddr + \
		SCORE_MEMORY_INTERNAL_SIZE - SCORE_MEMORY_FW_MSG_SIZE;

	ret = score_exynos_probe(&system->exynos, dev, system->regs);
	if (ret) {
		probe_err("score_exynos_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

p_err:
	return ret;
}

int score_system_open(struct score_system *system)
{
	int ret = 0;

	SCORE_TP();
	ret = score_memory_open(&system->memory);
	if (ret) {
		score_err("score_memory_open is fail(%d)\n", ret);
		goto p_err;
	}

	ret = score_interface_open(&system->interface);
	if (ret) {
		score_err("score_interface_open is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_system_close(struct score_system *system)
{
	int ret = 0;

	SCORE_TP();

	ret = score_interface_close(&system->interface);
	if (ret)
		score_err("score_interface_close is fail(%d)\n", ret);

	ret = score_memory_close(&system->memory);
	if (ret)
		score_err("score_memory_close is fail(%d)\n", ret);

	return ret;
}

int score_system_resume(struct score_system *system)
{
	int ret = 0;
	unsigned int loop = 0;
	struct score_exynos *exynos;
	struct score_memory *memory;
	struct score_binary *binary_text;
	struct score_binary *binary_data;

	SCORE_TP();
	BUG_ON(!system);

	exynos = &system->exynos;
	memory = &system->memory;
	binary_text = &system->fw_info[0].binary_text;
	binary_data = &system->fw_info[0].binary_data;

#ifndef DISABLE_CLK_OP
	ret = CLK_OP(exynos, clk_cfg);
	if (ret) {
		score_err("CLK_OP(clk_cfg) is fail(%d)\n", ret);
		goto p_err;
	}
#endif

#if defined(CONFIG_PM_DEVFREQ)
	pm_qos_add_request(&exynos_score_qos_cam, PM_QOS_CAM_THROUGHPUT, SCORE_CAM_L5); /* L5 */
	pm_qos_add_request(&exynos_score_qos_mem, PM_QOS_BUS_THROUGHPUT, SCORE_MIF_L6); /* L6 */
#endif

#ifndef DISABLE_CLK_OP
	ret = CLK_OP(exynos, clk_on);
	if (ret) {
		score_err("CLK_OP(clk_on) is fail(%d)\n", ret);
		goto p_err;
	}
#endif

#if defined(CONFIG_VIDEOBUF2_ION)
#ifdef DBG_SYSTEM_LOG
	score_event_msg("alloc_ctx(%p) \n", memory->alloc_ctx);
#endif
	vb2_ion_attach_iommu(memory->alloc_ctx);
#endif

	for(loop = 0; loop < MAX_FW_COUNT; loop ++) {
		binary_text = &system->fw_info[loop].binary_text;
		binary_data = &system->fw_info[loop].binary_data;

		ret = score_binary_load(binary_text);
		if (ret) {
			score_err("score_binary_text_load is fail(%d)\n", ret);
			goto p_err;
		}
#ifdef DBG_SYSTEM_LOG
		score_event_msg("binary_text \n");
		/* score_dump_regs(system->fw_info[loop].fw_text_kvaddr, 0x100); */
#endif

		ret = score_binary_load(binary_data);
		if (ret) {
			score_err("score_binary_data_load is fail(%d)\n", ret);
			goto p_err;
		}
#ifdef DBG_SYSTEM_LOG
		score_event_msg("binary_data \n");
		/* score_dump_regs(system->fw_info[loop].fw_data_kvaddr, 0x100); */
#endif
	}

#ifndef DISABLE_CLK_OP
	ret = CTL_OP(exynos, ctl_reset);
	if (ret) {
		score_err("CTL_OP(ctl_reset) is fail(%d)\n", ret);
		goto p_err;
	}
#endif

p_err:
	return ret;
}

int score_system_suspend(struct score_system *system)
{
	int ret = 0;
	struct score_exynos *exynos;
	struct score_memory *memory;

	SCORE_TP();
	BUG_ON(!system);

	exynos = &system->exynos;
	memory = &system->memory;

#if defined(CONFIG_VIDEOBUF2_ION)
	vb2_ion_detach_iommu(memory->alloc_ctx);
#endif
	/* HACK : reset to power-off */
	writel(0x1, system->regs + SCORE_SW_RESET);

#ifndef DISABLE_CLK_OP
	ret = CLK_OP(exynos, clk_off);
	if (ret)
		score_err("CLK_OP(clk_off) is fail(%d)\n", ret);

#if defined(CONFIG_PM_DEVFREQ)
	pm_qos_remove_request(&exynos_score_qos_cam);
	pm_qos_remove_request(&exynos_score_qos_mem);
#endif
#endif

	return ret;
}

int score_system_start(struct score_system *system)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!system);

	ret = score_interface_start(&system->interface);
	if (ret) {
		score_err("score_interface_start is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int score_system_stop(struct score_system *system)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!system);

	ret = score_interface_stop(&system->interface);
	if (ret)
		score_err("score_interface_stop is fail(%d)\n", ret);

	return ret;
}
