/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/exynos_iovmm.h>
#include <linux/pm_runtime.h>

#include "score-config.h"
#include "score-device.h"
#include "score-regs.h"
#include "score-debug.h"

static int score_device_runtime_suspend(struct device *dev);
static int score_device_runtime_resume(struct device *dev);

#ifdef ENABLE_SYSFS_SYSTEM
/* sysfs global variable for system */
struct score_sysfs_system sysfs_system;
#endif
#ifdef ENABLE_SYSFS_STATE
/* sysfs global variable for system */
struct score_sysfs_state sysfs_state;
#endif

struct score_fw_dev *score_fw_device;

int score_device_fw_ld(struct score_device *device, int index)
{
	int ret = 0;
	u32 read_val = 0;
	u32 duration = 1000;

	struct score_system *system;
	struct score_memory *memory;

	system = &device->system;
	memory = &system->memory;

	BUG_ON(!device);

	/* SW reset */
	writel(0x1, system->regs + SCORE_SW_RESET);
	udelay(1);

	/* Configure start */
	writel(0x1, system->regs + SCORE_AXIM1ARUSER);
	udelay(1);

	if (system->fw_info[index].fw_type == FW_TYPE_CACHE) {
		writel(system->fw_info[index].fw_text_dvaddr,
			system->regs + SCORE_CODE_START_ADDR);
		writel(system->fw_info[index].fw_data_dvaddr - FW_STACK_SIZE,
			system->regs + SCORE_DATA_START_ADDR);
		udelay(1);

		writel(0x3, system->regs + SCORE_CODE_MODE_ADDR);
		writel(0x3, system->regs + SCORE_DATA_MODE_ADDR);
	} else if (system->fw_info[index].fw_type == FW_TYPE_SRAM) {
		score_event_msg("C_M_ADDR(0x%x) (0x%x) (0x%x)\n",
			readl(system->regs + SCORE_CODE_MODE_ADDR),
			readl(system->regs + 0x1008),
			readl(system->regs + 0x2008));

		writel(0x1, system->regs + SCORE_CODE_MODE_ADDR);
		writel(0x1, system->regs + SCORE_DATA_MODE_ADDR);
		udelay(1);

		score_event_msg("C_M_ADDR(0x%x) (0x%x) (0x%x)\n",
			readl(system->regs + SCORE_CODE_MODE_ADDR),
			readl(system->regs + 0x1008),
			readl(system->regs + 0x2008));

#ifdef DBG_USE_NO_MEMCPY
		do {
			int loopCount = 0;

			unsigned char *pdst = (unsigned char *)(system->regs + 0x8000);
			unsigned char *psrc = (unsigned char *)(system->fw_info[index].fw_text_kvaddr);

			for(loopCount = 0; loopCount < system->fw_info[index].fw_text_size; loopCount ++)
				pdst[loopCount] = psrc[loopCount];


			pdst = (unsigned char *)(system->regs + 0x12000);
			psrc = (unsigned char *)(system->fw_info[index].fw_data_kvaddr);
			for(loopCount = 0; loopCount < system->fw_info[index].fw_data_size; loopCount ++)
				pdst[loopCount] = psrc[loopCount];
		} while (0);
#else
		memcpy(system->regs + FW_SRAM_REGION_SIZE,
			system->fw_info[index].fw_text_kvaddr,
			system->fw_info[index].fw_text_size);
		memcpy(system->regs + (FW_SRAM_REGION_SIZE + FW_SRAM_REGION_SIZE + 0x4000 - 0x2000),
			system->fw_info[index].fw_data_kvaddr,
			system->fw_info[index].fw_data_size);
		/* score_dump_regs(system->fw_info[index].fw_text_kvaddr, 0x100); */
		/* score_dump_regs(system->fw_info[index].fw_data_kvaddr, 0x100); */
#endif
		/* Need Check : Why? */
		writel(FW_SRAM_REGION_SIZE, system->regs + SCORE_DATA_START_ADDR);
		score_event_msg("D_S_ADDR(0x%x) \n", readl(system->regs + SCORE_DATA_START_ADDR));
		/* writel(((32 * 1024) + (32 * 1024) + 0x4000 - 0x2000), */
		/*		system->regs + SCORE_DATA_START_ADDR); */

		score_event_msg("C_M_ADDR(0x%x) (0x%x) (0x%x)\n",
			readl(system->regs + SCORE_CODE_MODE_ADDR),
			readl(system->regs + 0x1008),
			readl(system->regs + 0x2008));

		score_event_msg("(%d) k(0x%p 0x%p) d(0x%llX 0x%llX\n",
			index,
			system->fw_info[index].fw_text_kvaddr,
			system->fw_info[index].fw_data_kvaddr,
			system->fw_info[index].fw_text_dvaddr,
			system->fw_info[index].fw_data_dvaddr);

		udelay(1);

		writel(0x21, system->regs + SCORE_CODE_MODE_ADDR);
		writel(0x21, system->regs + SCORE_DATA_MODE_ADDR);
		mdelay(1);

		score_event_msg("C_M_ADDR(0x%x) (0x%x) (0x%x)\n",
			readl(system->regs + SCORE_CODE_MODE_ADDR),
			readl(system->regs + 0x1008),
			readl(system->regs + 0x2008));
	} else {
		score_err("fw_type is not valid(%d) \n",
			system->fw_info[index].fw_type);

		return -1;
	}
	udelay(1);

	score_event_msg("(%d) d(0x%llX 0x%llX) \n",
		index,
		system->fw_info[index].fw_text_dvaddr,
		system->fw_info[index].fw_data_dvaddr);
	score_event_msg("k(0x%p)\n", system->fw_info[index].fw_text_kvaddr);
	score_event_msg("k(0x%p)\n", system->fw_info[index].fw_data_kvaddr);

	/* Configure end */
	writel(0x0, system->regs + SCORE_AXIM1ARUSER);
	udelay(1);

	writel(system->fw_msg_dvaddr, system->regs + SCORE_PRINTF_BUF_START);
	udelay(1);

	writel(0x1, system->regs + SCORE_ENABLE);

	writel(0x1, system->regs + SCORE_HOST2SCORE);
	udelay(1);

	/* Interrupt Clear */
	writel(0x0, system->regs + SCORE_DSP_INT);
	/* SW reset Clear */
	writel(0x0, system->regs + SCORE_SW_RESET);
	udelay(1);

	do {
#ifdef DBG_SYSMMU_SETTING
		void __iomem *sysMMU;
#endif

		mdelay(5 * 1);
		read_val = readl(system->regs + SCORE_SCORE2HOST);
		score_event_msg("S2H(0x%x) (%d)\n", read_val, duration);
		read_val = readl(system->regs + SCORE_IDLE);
		score_event_msg("HALT(0x%x) \n", read_val);
#ifdef DEBUG_SYSMMU_SETTING
		sysMMU = ioremap(0x13430000, 0x4);
		score_event_msg("[%s(%d)] sysMMU \n", __func__,__LINE__);
		score_dump_regs(sysMMU, 0x10);
#endif
		score_event_msg("55(0x%x)56(0x%x)57(0x%x)RET(0x%x) \n",
			readl(system->regs + SCORE_PARAM55),
			readl(system->regs + SCORE_PARAM56),
			readl(system->regs + SCORE_PARAM57),
			readl(system->regs + SCORE_VERIFY_RESULT));

		duration --;
	} while((readl(system->regs + SCORE_IDLE) != 0) && (duration > 995));

	score_event_msg("SW_RST(0x%x) \n",
		readl(system->regs + SCORE_SW_RESET));
	score_event_msg("HALT(0x%x) \n",
		readl(system->regs + SCORE_IDLE));
	score_event_msg("C_M_ADDR(0x%x) \n",
		readl(system->regs + SCORE_CODE_MODE_ADDR));
	score_event_msg("D_M_ADDR(0x%x) \n",
		readl(system->regs + SCORE_DATA_MODE_ADDR));
	score_event_msg("C_S_ADDR(0x%x) \n",
		readl(system->regs + SCORE_CODE_START_ADDR));
	score_event_msg("D_S_ADDR(0x%x) \n",
		readl(system->regs + SCORE_DATA_START_ADDR));
	score_event_msg("EN(0x%x) \n",
		readl(system->regs + SCORE_ENABLE));
	score_event_msg("DSP_INT(0x%x) \n",
		readl(system->regs + SCORE_DSP_INT));
	score_event_msg("SW_RST(0x%x) \n",
		readl(system->regs + SCORE_SW_RESET));

	score_event_msg("UDEF0(0x%x) \n",
		readl(system->regs + SCORE_USERDEFINDED0));
	score_event_msg("UDEF1(0x%x) \n",
		readl(system->regs + SCORE_USERDEFINDED1));

	score_event_msg("55(0x%x)56(0x%x)57(0x%x)RET(0x%x) \n",
		readl(system->regs + SCORE_PARAM55),
		readl(system->regs + SCORE_PARAM56),
		readl(system->regs + SCORE_PARAM57),
		readl(system->regs + SCORE_VERIFY_RESULT));
	/* score_dump_regs(system->regs + 0x0, 0x100); */

	return ret;
}

#ifdef ENABLE_SYSFS_STATE
static DEVICE_ATTR(state_val,
		0644,
		show_sysfs_state_val,
		store_sysfs_state_val);
static DEVICE_ATTR(state_en,
		0644,
		show_sysfs_state_en,
		store_sysfs_state_en);
#endif
#ifdef ENABLE_SYSFS_SYSTEM
static DEVICE_ATTR(clk_gate_en,
		0644,
		show_sysfs_system_clk_gate_en,
		store_sysfs_system_clk_gate_en);
static DEVICE_ATTR(dvfs_en,
		0644,
		show_sysfs_system_dvfs_en,
		store_sysfs_system_dvfs_en);
#endif
#if (defined(ENABLE_SYSFS_SYSTEM) || defined(ENABLE_SYSFS_STATE))
static struct attribute *score_system_entries[] = {
#ifdef ENABLE_SYSFS_SYSTEM
	&dev_attr_clk_gate_en.attr,
	&dev_attr_dvfs_en.attr,
#endif
#ifdef ENABLE_SYSFS_STATE
	&dev_attr_state_val.attr,
	&dev_attr_state_en.attr,
#endif
	NULL,
};
static struct attribute_group score_system_attr_group = {
	.name   = "system",
	.attrs  = score_system_entries,
};
#endif

void __score_fault_handler(struct score_device *device) {
#if 0
	struct score_system *system;
	struct score_memory *memory;
	struct score_graphmgr *graphmgr;
	struct score_graph *graph;
	struct score_pu *pu, *temp;
	struct vb_container *container;
	struct vb_buffer *buffer;
	u32 i, j, k;

	system = &device->system;
	memory = &system->memory;
	graphmgr = &device->graphmgr;

	/* 1. Internal Memory Infomation */
	score_event_msg("internal memory : 0x%llX\n", memory->info.dvaddr);

	/* 2. Buffer Memroy Information */
	for (i = 0; i < SCORE_MAX_VERTEX; ++i) {
		graph = graphmgr->graph[i];
		if (!graph)
			continue;

		score_event_msg("=================================================\n");
		score_event_msg("GRAPH : %d(%d)\n", graph->id, graph->uid);
		score_event_msg("INPUT--------------------------------------------\n");
		list_for_each_entry_safe(pu, temp, &graph->inleaf_list, gleaf) {
			score_event_msg("PU : %d\n", pu->id);
			score_event_msg("-------------------------------------------------\n");
			for (j = 0; j < SCORE_MAX_BUFFER; ++j) {
				container = pu->container[j];
				if (!container)
					continue;

				if (j == 0) {
					score_event_msg("TYPE : %d\n", container->type);
					score_event_msg("RESOLUTION : %d x %d\n", container->format->width, container->format->height);
					score_event_msg("SIZE : %d\n", container->format->size[container->format->plane]);
					score_event_msg("-------------------------------------------------\n");
				}

				for (k = 0; k < container->count; ++k) {
					buffer = &container->buffers[k];
					score_event_msg("[%d][%d]DVADDR : %llx\n", j, k, buffer->dvaddr);
					score_event_msg("[%d][%d]KVADDR : %p\n", j, k, buffer->kvaddr);
				}

				score_event_msg("-------------------------------------------------\n");
			}
		}

		score_event_msg("OUTPUT-------------------------------------------\n");
		list_for_each_entry_safe(pu, temp, &graph->otleaf_list, gleaf) {
			score_event_msg("PU : %d\n", pu->id);
			score_event_msg("-------------------------------------------------\n");
			for (j = 0; j < SCORE_MAX_BUFFER; ++j) {
				container = pu->container[j];
				if (!container)
					continue;

				if (j == 0) {
					score_event_msg("TYPE : %d\n", container->type);
					score_event_msg("RESOLUTION : %d x %d\n", container->format->width, container->format->height);
					score_event_msg("SIZE : %d\n", container->format->size[container->format->plane]);
					score_event_msg("-------------------------------------------------\n");
				}

				for (k = 0; k < container->count; ++k) {
					buffer = &container->buffers[k];
					score_event_msg("[%d][%d]DVADDR : %llx\n", j, k, buffer->dvaddr);
					score_event_msg("[%d][%d]KVADDR : %p\n", j, k, buffer->kvaddr);
				}

				score_event_msg("-------------------------------------------------\n");
			}
		}
	}
#endif
}

static int __attribute__((unused)) score_fault_handler(struct iommu_domain *domain,
	struct device *dev,
	unsigned long fault_addr,
	int fault_flag,
	void *token)
{
	struct score_device *device;

	pr_err("<SCORE FAULT HANDLER>\n");
	pr_err("Device virtual(0x%X) is invalid access\n", (u32)fault_addr);

	device = dev_get_drvdata(dev);

	__score_fault_handler(device);

	return -EINVAL;
}

static int __score_device_start(struct score_device *device)
{
	int ret = 0;

	SCORE_TP();
	/*
	*if (test_bit(SCORE_DEVICE_STATE_START, &device->state)) {
	*	score_err("already started\n");
	*	ret = -EINVAL;
	*	goto p_err;
	*}
	*/

	ret = score_system_start(&device->system);
	if (ret) {
		score_err("score_system_start is fail(%d)\n", ret);
		goto p_err;
	}

	set_bit(SCORE_DEVICE_STATE_START, &device->state);

p_err:
	return ret;
}

static int __score_device_stop(struct score_device *device)
{
	int ret = 0;

	/*
	*if (!test_bit(SCORE_DEVICE_STATE_START, &device->state))
	*	goto p_err;
	*/

	ret = score_system_stop(&device->system);
	if (ret) {
		score_err("score_system_stop is fail(%d)\n", ret);
		goto p_err;
	}

	/* score_dump_regs(device->system.fw_msg_kvaddr + 1024, 0x20); */

	clear_bit(SCORE_DEVICE_STATE_START, &device->state);

p_err:
	return ret;
}

static int __score_device_power_on(struct score_device *device)
{
	int ret = 0;

	SCORE_TP();
#if defined(CONFIG_PM)
	ret = pm_runtime_get_sync(device->dev);
#else
	ret = score_device_runtime_resume(device->dev);
#endif
	if (ret)
		score_err("runtime resume is fail(%d)", ret);

	return ret;
}

static int __score_device_power_off(struct score_device *device)
{
	int ret = 0;

#if defined(CONFIG_PM)
	ret = pm_runtime_put_sync(device->dev);
#else
	ret = score_device_runtime_suspend(device->dev);
#endif
	if (ret)
		score_err("runtime resume is fail(%d)", ret);

	return ret;
}

static int score_device_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev;
	struct score_device *device;
	struct score_system *system;

	SCORE_TP();
	BUG_ON(!pdev);

	dev = &pdev->dev;

	device = devm_kzalloc(dev, sizeof(struct score_device), GFP_KERNEL);
	if (!device) {
		probe_err("device is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	ret = score_system_probe(&device->system, pdev);
	if (ret) {
		probe_err("score_system_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}
	system = &device->system;

	ret = score_vertex_probe(&device->vertex, dev);
	if (ret) {
		probe_err("score_vertex_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

	/*
	*ret = score_resource_probe(&device->resource, &device->vertex.vd.dev);
	*if (ret) {
	*	probe_err("score_resource_probe is fail(%d)\n", ret);
	*	ret = -EINVAL;
	*	goto p_err;
	*}
	*/

	ret = score_vertexmgr_probe(&device->vertexmgr);
	if (ret) {
		probe_err("score_vertexmgr_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

	ret = score_debug_probe();
	if (ret) {
		probe_err("score_debug_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

	ret = score_fw_queue_init(&device->fw_dev);
	if (ret) {
		probe_err("score_fw_queue_init is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}
	/* HACK : Need to Modify */
	score_fw_device = &device->fw_dev;

	iovmm_set_fault_handler(dev, score_fault_handler, NULL);

#if defined(CONFIG_PM)
	pm_runtime_enable(dev);
#endif

	device->pdev = pdev;
	device->dev = dev;
	/* clear_bit(SCORE_DEVICE_STATE_OPEN, &device->state); */
	/* clear_bit(SCORE_DEVICE_STATE_START, &device->state); */
	dev_set_drvdata(dev, device);

	device->fw_index = 0;

#ifdef ENABLE_SYSFS_SYSTEM
	/* set sysfs for system */
	sysfs_system.en_clk_gate = 0;
	sysfs_system.en_dvfs = 1;
#endif
#ifdef ENABLE_SYSFS_STATE
	sysfs_state.is_en = false;
	sysfs_state.frame_duration = 0;
	sysfs_state.long_time = 3;
	sysfs_state.short_time = 0;
	sysfs_state.long_v_rank = 9;
	sysfs_state.short_v_rank = 0;
	sysfs_state.long_r_rank = 4;
	sysfs_state.short_r_rank = 1;
#endif
#if (defined(ENABLE_SYSFS_SYSTEM) || defined(ENABLE_SYSFS_STATE))
	ret = sysfs_create_group(&device->pdev->dev.kobj, &score_system_attr_group);
#endif

p_err:
	score_event_msg("ret(%d) \n", ret);
	return ret;
}

int score_device_open(struct score_device *device)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!device);

	/*
	*if (test_bit(SCORE_DEVICE_STATE_OPEN, &device->state)) {
	*	score_err("device is already opened\n");
	*	ret = -EINVAL;
	*	goto p_err;
	*}
	*/

	ret = score_system_open(&device->system);
	if (ret) {
		score_err("score_system_open is fail(%d)\n", ret);
		goto p_err;
	}

	ret = score_debug_open(&device->system);
	if (ret) {
		score_err("score_debug_open is fail(%d)\n", ret);
		goto p_err;
	}

	ret = score_vertexmgr_open(&device->vertexmgr);
	if (ret) {
		score_err("score_vertexmgr_open is fail(%d)\n", ret);
		goto p_err;
	}

	ret = __score_device_power_on(device);
	if (ret) {
		score_err("__score_device_power_on is fail(%d)\n", ret);
		goto p_err;
	}

	/* Initialized FW */
	score_device_fw_ld(device, 0);

	/* set_bit(SCORE_DEVICE_STATE_OPEN, &device->state); */

p_err:
	score_event_msg("ret(%d) \n", ret);
	return ret;
}

int score_device_close(struct score_device *device)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!device);

	/*
	*if (!test_bit(SCORE_DEVICE_STATE_OPEN, &device->state)) {
	*	score_err("device is already closed\n");
	*	ret = -EINVAL;
	*	goto p_err;
	*}
	*/

	ret = __score_device_stop(device);
	if (ret)
		score_err("__score_device_stop is fail(%d)\n", ret);

	ret = score_vertexmgr_close(&device->vertexmgr);
	if (ret) {
		score_err("score_vertexmgr_close is fail(%d)\n", ret);
		goto p_err;
	}

	ret = score_system_close(&device->system);
	if (ret)
		score_err("score_system_close is fail(%d)\n", ret);

	ret = score_debug_close();
	if (ret)
		score_err("score_debug_close is fail(%d)\n", ret);

	ret = __score_device_power_off(device);
	if (ret)
		score_err("__score_device_power_off is fail(%d)\n", ret);

	clear_bit(SCORE_DEVICE_STATE_OPEN, &device->state);

p_err:
	score_event_msg("ret(%d)\n", ret);
	return ret;
}

int score_device_start(struct score_device *device)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!device);

	ret = __score_device_start(device);
	if (ret)
		score_err("__score_device_start is fail(%d)\n", ret);

	score_event_msg("ret(%d)\n", ret);

	return ret;
}

int score_device_stop(struct score_device *device)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!device);

	ret = __score_device_stop(device);
	if (ret)
		score_err("__score_device_stop is fail(%d)\n", ret);

	score_event_msg("ret(%d)\n", ret);

	return ret;
}

void score_device_msg_print()
{
	score_debug_info_dump2();

	return;
}

static int score_device_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev;
	struct score_device *device;

	BUG_ON(!pdev);

	SCORE_TP();
	dev = &pdev->dev;
	device = dev_get_drvdata(dev);
	score_fw_queue_exit(&device->fw_dev);

	return ret;
}

static int score_device_suspend(struct device *dev)
{
	SCORE_TP();
	return 0;
}

static int score_device_resume(struct device *dev)
{
	SCORE_TP();
	return 0;
}

static int score_device_runtime_suspend(struct device *dev)
{
	int ret = 0;
	struct score_device *device;
	/* struct score_system *system; */

	SCORE_TP();
	device = dev_get_drvdata(dev);

	/* system = &device->system; */
	/* score_dump_regs(system->fw_msg_kvaddr, 0x20); */

	ret = score_system_suspend(&device->system);
	if (ret)
		score_err("score_system_suspend is fail(%d)\n", ret);

	score_event_msg("ret(%d)\n", ret);
	return ret;
}

static int score_device_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct score_device *device;

	SCORE_TP();
	device = dev_get_drvdata(dev);

	ret = score_system_resume(&device->system);
	if (ret) {
		score_err("score_system_resume is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	score_event_msg("ret(%d)\n", ret);
	return ret;
}

static const struct dev_pm_ops score_pm_ops = {
	.suspend		= score_device_suspend,
	.resume			= score_device_resume,
	.runtime_suspend	= score_device_runtime_suspend,
	.runtime_resume		= score_device_runtime_resume,
};

static const struct of_device_id exynos_score_match[] = {
	{
		.compatible = "samsung,score",
	},
	{}
};
MODULE_DEVICE_TABLE(of, exynos_score_match);

static struct platform_driver score_driver = {
	.probe		= score_device_probe,
	.remove		= score_device_remove,
	.driver = {
		.name	= "exynos-score",
		.owner	= THIS_MODULE,
		.pm	= &score_pm_ops,
		.of_match_table = of_match_ptr(exynos_score_match)
	}
};

static int __init score_device_init(void)
{
	int ret = platform_driver_register(&score_driver);
	if (ret)
		probe_err("platform_driver_register is fail():%d\n", ret);

	probe_info("score device init is loaded");

	return ret;
}
late_initcall(score_device_init);

static void __exit score_device_exit(void)
{
	platform_driver_unregister(&score_driver);
}
module_exit(score_device_exit);

MODULE_AUTHOR("Pilsun Jang<pilsun.jang@samsung.com>");
MODULE_DESCRIPTION("Exynos SCORE driver");
MODULE_LICENSE("GPL");
