/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is video functions
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <asm/neon.h>

#include "fimc-is-config.h"
#include "fimc-is-param.h"
#include "fimc-is-type.h"
#include "fimc-is-regs.h"
#include "fimc-is-core.h"
#include "fimc-is-hw-chain.h"
#include "fimc-is-hw-settle-14nm.h"
#include "fimc-is-device-sensor.h"
#include "fimc-is-device-csi.h"
#include "fimc-is-device-ischain.h"

#include "../../interface/fimc-is-interface-ischain.h"
#include "../../hardware/fimc-is-hw-control.h"
#include "../../hardware/fimc-is-hw-mcscaler-v2.h"

static struct fimc_is_reg sysreg_is_regs[SYSREG_IS_REG_CNT] = {
	{0x0600, "IS_CSISX4_MUX_3AA_VAL"},
	{0x0700, "IS_MUX_ISP_VAL"},
};

static struct fimc_is_field sysreg_is_fields[SYSREG_IS_REG_FIELD_CNT] = {
	{"CSISX4_MUX_3AA0_VAL",		0,	2,	RW,	0x0},	/* IS_CSISX4_MUX_3AA0_VAL */
	{"CSISX4_MUX_3AA1_VAL",		8,	2,	RW,	0x0},	/* IS_CSISX4_MUX_3AA1_VAL */
	{"MUX_ISP_VAL",			0,	1,	RW,	0x0},	/* IS_MUX_ISP_VAL */
};

/* Define default subdev ops if there are not used subdev IP */
const struct fimc_is_subdev_ops fimc_is_subdev_scc_ops;
const struct fimc_is_subdev_ops fimc_is_subdev_scp_ops;
const struct fimc_is_subdev_ops fimc_is_subdev_dis_ops;
const struct fimc_is_subdev_ops fimc_is_subdev_dxc_ops;

/* TODO: Need? */
#if 0
struct fimc_is_clk_gate clk_gate_3aa0;
struct fimc_is_clk_gate clk_gate_3aa1;
struct fimc_is_clk_gate clk_gate_isp0;
struct fimc_is_clk_gate clk_gate_mcsc;
struct fimc_is_clk_gate clk_gate_vra;
#endif

void __iomem *hwfc_rst;

void fimc_is_enter_lib_isr(void)
{
	kernel_neon_begin();
}

void fimc_is_exit_lib_isr(void)
{
	kernel_neon_end();
}

void fimc_is_hw_group_init(struct fimc_is_group *group)
{
	int entry = 0;

	for (entry = 0; entry < ENTRY_END; entry++)
		group->subdev[entry] = NULL;

	INIT_LIST_HEAD(&group->subdev_list);
}

int fimc_is_hw_group_cfg(void *group_data)
{
	int ret = 0;
	struct fimc_is_group *group;
	struct fimc_is_device_sensor *sensor;
	struct fimc_is_device_ischain *device;

	BUG_ON(!group_data);

	group = (struct fimc_is_group *)group_data;

#ifdef CONFIG_USE_SENSOR_GROUP
	if (group->slot == GROUP_SLOT_SENSOR) {
		sensor = group->sensor;
		if (!sensor) {
			err("device is NULL");
			BUG();
		}

		fimc_is_hw_group_init(group);
		group->subdev[ENTRY_SENSOR] = &sensor->group_sensor.leader;
		group->subdev[ENTRY_SSVC0] = &sensor->ssvc0;
		group->subdev[ENTRY_SSVC1] = &sensor->ssvc1;
		group->subdev[ENTRY_SSVC2] = &sensor->ssvc2;
		group->subdev[ENTRY_SSVC3] = &sensor->ssvc3;

		list_add_tail(&sensor->group_sensor.leader.list, &group->subdev_list);
		list_add_tail(&sensor->ssvc0.list, &group->subdev_list);
		list_add_tail(&sensor->ssvc1.list, &group->subdev_list);
		list_add_tail(&sensor->ssvc2.list, &group->subdev_list);
		list_add_tail(&sensor->ssvc3.list, &group->subdev_list);
		return ret;
	}
#endif

	device = group->device;

	if (!device) {
		err("device is NULL");
		BUG();
	}

	switch (group->slot) {
	case GROUP_SLOT_3AA:
		fimc_is_hw_group_init(group);
		group->subdev[ENTRY_3AA] = &device->group_3aa.leader;
		group->subdev[ENTRY_3AC] = &device->txc;
		group->subdev[ENTRY_3AP] = &device->txp;

		list_add_tail(&device->group_3aa.leader.list, &group->subdev_list);
		list_add_tail(&device->txc.list, &group->subdev_list);
		list_add_tail(&device->txp.list, &group->subdev_list);

		device->txc.param_dma_ot = PARAM_3AA_VDMA4_OUTPUT;
		device->txp.param_dma_ot = PARAM_3AA_VDMA2_OUTPUT;
		break;
	case GROUP_SLOT_ISP:
		fimc_is_hw_group_init(group);
		group->subdev[ENTRY_ISP] = &device->group_isp.leader;
		group->subdev[ENTRY_IXC] = &device->ixc;
		group->subdev[ENTRY_IXP] = &device->ixp;

		list_add_tail(&device->group_isp.leader.list, &group->subdev_list);
		list_add_tail(&device->ixc.list, &group->subdev_list);
		list_add_tail(&device->ixp.list, &group->subdev_list);
		break;
	case GROUP_SLOT_DIS:
		break;
	case GROUP_SLOT_MCS:
		fimc_is_hw_group_init(group);
		group->subdev[ENTRY_MCS] = &device->group_mcs.leader;
		group->subdev[ENTRY_M0P] = &device->m0p;
		group->subdev[ENTRY_M1P] = &device->m1p;
		group->subdev[ENTRY_M2P] = &device->m2p;
		group->subdev[ENTRY_M3P] = &device->m3p;
		group->subdev[ENTRY_DNR] = &device->dnr;

		list_add_tail(&device->group_mcs.leader.list, &group->subdev_list);
		list_add_tail(&device->m0p.list, &group->subdev_list);
		list_add_tail(&device->m1p.list, &group->subdev_list);
		list_add_tail(&device->m2p.list, &group->subdev_list);
		list_add_tail(&device->m3p.list, &group->subdev_list);
		list_add_tail(&device->dnr.list, &group->subdev_list);

		device->m0p.param_dma_ot = PARAM_MCS_OUTPUT0;
		device->m1p.param_dma_ot = PARAM_MCS_OUTPUT1;
		device->m2p.param_dma_ot = PARAM_MCS_OUTPUT2;
		device->m3p.param_dma_ot = PARAM_MCS_OUTPUT3;
		break;
	case GROUP_SLOT_VRA:
		fimc_is_hw_group_init(group);
		group->subdev[ENTRY_VRA] = &device->group_vra.leader;

		list_add_tail(&device->group_vra.leader.list, &group->subdev_list);
		break;
	default:
		probe_err("group slot(%d) is invalid", group->slot);
		BUG();
		break;
	}

	/* for hwfc: reset all REGION_IDX registers and outputs */
	hwfc_rst = ioremap(HWFC_INDEX_RESET_ADDR, SZ_4);

	return ret;
}

int fimc_is_hw_group_open(void *group_data)
{
	int ret = 0;
	u32 group_id;
	struct fimc_is_subdev *leader;
	struct fimc_is_group *group;
	struct fimc_is_device_ischain *device;

	BUG_ON(!group_data);

	group = group_data;
	leader = &group->leader;
	device = group->device;
	group_id = group->id;

	switch (group_id) {
#ifdef CONFIG_USE_SENSOR_GROUP
	case GROUP_ID_SS0:
	case GROUP_ID_SS1:
	case GROUP_ID_SS2:
	case GROUP_ID_SS3:
	case GROUP_ID_SS4:
	case GROUP_ID_SS5:
#endif
	case GROUP_ID_3AA0:
	case GROUP_ID_3AA1:
	case GROUP_ID_ISP0:
	case GROUP_ID_MCS0:
		/* width & height constraint_size both increase upto 24MP width(5760)
		 * because it may use rotation and height also set upto 5760
		 */
		leader->constraints_width = 5760;
		leader->constraints_height = 5760;
		break;
	case GROUP_ID_VRA0:
		leader->constraints_width = 640;
		leader->constraints_height = 480;
		break;
	default:
		merr("group id is invalid(%d)", group, group_id);
		break;
	}

	return ret;
}

int fimc_is_hw_camif_cfg(void *sensor_data)
{
	int ret = 0;
	struct fimc_is_device_sensor *sensor;
	struct fimc_is_device_csi *csi;

	BUG_ON(!sensor_data);

	sensor = sensor_data;
	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(sensor->subdev_csi);
	if (!csi) {
		merr("csi is null\n", sensor);
		ret = -ENODEV;
		goto p_err;
	}

	/* always clear csis dummy */
	clear_bit(CSIS_DUMMY, &csi->state);

p_err:
	return ret;
}

int fimc_is_hw_camif_open(void *sensor_data)
{
	int ret = 0;
	struct fimc_is_device_sensor *sensor;
	struct fimc_is_device_csi *csi;

	BUG_ON(!sensor_data);

	sensor = sensor_data;
	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(sensor->subdev_csi);

	switch (csi->instance) {
	case 0:
	case 1:
	case 2:
	case 3:
		set_bit(CSIS_DMA_ENABLE, &csi->state);
#ifdef SOC_SSVC0
		csi->dma_subdev[CSI_VIRTUAL_CH_0] = &sensor->ssvc0;
#else
		csi->dma_subdev[CSI_VIRTUAL_CH_0] = NULL;
#endif
#ifdef SOC_SSVC1
		csi->dma_subdev[CSI_VIRTUAL_CH_1] = &sensor->ssvc1;
#else
		csi->dma_subdev[CSI_VIRTUAL_CH_1] = NULL;
#endif
#ifdef SOC_SSVC2
		csi->dma_subdev[CSI_VIRTUAL_CH_2] = &sensor->ssvc2;
#else
		csi->dma_subdev[CSI_VIRTUAL_CH_2] = NULL;
#endif
#ifdef SOC_SSVC3
		csi->dma_subdev[CSI_VIRTUAL_CH_3] = &sensor->ssvc3;
#else
		csi->dma_subdev[CSI_VIRTUAL_CH_3] = NULL;
#endif
		break;
	default:
		merr("sensor id is invalid(%d)", sensor, csi->instance);
		break;
	}

	return ret;
}

void fimc_is_hw_ischain_qe_cfg(void)
{
	dbg_hw(2, "%s() \n", __func__);
}

int fimc_is_hw_ischain_cfg(void *ischain_data)
{
	int ret = 0;
	struct fimc_is_device_ischain *device;
	struct fimc_is_device_sensor *sensor;
	struct fimc_is_device_csi *csi;
	void __iomem *is_regs;
	u32 is_val = 0, is_backup = 0;
	u32 field_index = 0;

	BUG_ON(!ischain_data);

	device = (struct fimc_is_device_ischain *)ischain_data;
	if (test_bit(FIMC_IS_ISCHAIN_REPROCESSING, &device->state))
		return ret;

	sensor = device->sensor;
	BUG_ON(!sensor);

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(sensor->subdev_csi);
	BUG_ON(!csi);
	mutex_lock(&device->resourcemgr->sysreg_lock);

	is_regs   = fimc_is_hw_get_sysreg(0);

	is_val = fimc_is_hw_get_reg(is_regs, &sysreg_is_regs[SYSREG_IS_R_CSISX4_MUX_3AA_VAL]);
	is_backup = is_val;

	/* set CSISX4 - 3AA path MUX
	 * it is determined by csis instance
	 * field is determined group_3aa id (3AA0 or 3AA1)
	 */
	field_index = (device->group_3aa.id == GROUP_ID_3AA0) ?
			SYSREG_IS_F_CSISX4_MUX_3AA0_VAL : SYSREG_IS_F_CSISX4_MUX_3AA1_VAL;

	switch (csi->instance) {
	case 0:
	case 1:
	case 2:
	case 3:
		is_val = fimc_is_hw_set_field_value(is_val, &sysreg_is_fields[field_index], csi->instance);
		break;
	default:
		break;
	}

	if (is_backup != is_val) {
		minfo("IS_CSISX2_MUX_3AA_VAL:(0x%08X)->(0x%08X)\n",
			device, is_backup, is_val);
		fimc_is_hw_set_reg(is_regs, &sysreg_is_regs[SYSREG_IS_R_CSISX4_MUX_3AA_VAL], is_val);
	}

	iounmap(is_regs);
	mutex_unlock(&device->resourcemgr->sysreg_lock);

	return ret;
}

static inline void fimc_is_isr1_ddk(void *data)
{
	struct fimc_is_interface_hwip *itf_hw = NULL;
	struct hwip_intr_handler *intr_hw = NULL;

	itf_hw = (struct fimc_is_interface_hwip *)data;
	intr_hw = &itf_hw->handler[INTR_HWIP1];

	if (intr_hw->valid) {
		fimc_is_enter_lib_isr();
		intr_hw->handler(intr_hw->id, intr_hw->ctx);
		fimc_is_exit_lib_isr();
	} else {
		err_itfc("[ID:%d](1) empty handler!!", itf_hw->id);
	}
}

static inline void fimc_is_isr2_ddk(void *data)
{
	struct fimc_is_interface_hwip *itf_hw = NULL;
	struct hwip_intr_handler *intr_hw = NULL;

	itf_hw = (struct fimc_is_interface_hwip *)data;
	intr_hw = &itf_hw->handler[INTR_HWIP2];

	if (intr_hw->valid) {
		fimc_is_enter_lib_isr();
		intr_hw->handler(intr_hw->id, intr_hw->ctx);
		fimc_is_exit_lib_isr();
	} else {
		err_itfc("[ID:%d](2) empty handler!!", itf_hw->id);
	}
}

static inline void fimc_is_isr1_host(void *data)
{
	struct fimc_is_interface_hwip *itf_hw = NULL;
	struct hwip_intr_handler *intr_hw = NULL;

	itf_hw = (struct fimc_is_interface_hwip *)data;
	intr_hw = &itf_hw->handler[INTR_HWIP1];

	if (intr_hw->valid)
		intr_hw->handler(intr_hw->id, (void *)itf_hw->hw_ip);
	else
		err_itfc("[ID:%d] empty handler!!", itf_hw->id);

}

static irqreturn_t fimc_is_isr1_3aa0(int irq, void *data)
{
	fimc_is_isr1_ddk(data);
	return IRQ_HANDLED;
}

static irqreturn_t fimc_is_isr2_3aa0(int irq, void *data)
{
	fimc_is_isr2_ddk(data);
	return IRQ_HANDLED;
}

static irqreturn_t fimc_is_isr1_3aa1(int irq, void *data)
{
	fimc_is_isr1_ddk(data);
	return IRQ_HANDLED;
}

static irqreturn_t fimc_is_isr2_3aa1(int irq, void *data)
{
	fimc_is_isr2_ddk(data);
	return IRQ_HANDLED;
}

static irqreturn_t fimc_is_isr1_isp(int irq, void *data)
{
	fimc_is_isr1_ddk(data);
	return IRQ_HANDLED;
}

static irqreturn_t fimc_is_isr2_isp(int irq, void *data)
{
	fimc_is_isr2_ddk(data);
	return IRQ_HANDLED;
}

static irqreturn_t fimc_is_isr1_mcs(int irq, void *data)
{
	fimc_is_isr1_host(data);
	return IRQ_HANDLED;
}

static irqreturn_t fimc_is_isr1_vra(int irq, void *data)
{
	struct fimc_is_interface_hwip *itf_hw = NULL;
	struct hwip_intr_handler *intr_hw = NULL;

	itf_hw = (struct fimc_is_interface_hwip *)data;
	intr_hw = &itf_hw->handler[INTR_HWIP2];

	if (intr_hw->valid) {
		fimc_is_enter_lib_isr();
		intr_hw->handler(intr_hw->id, itf_hw->hw_ip);
		fimc_is_exit_lib_isr();
	} else {
		err_itfc("[ID:%d](2) empty handler!!", itf_hw->id);
	}

	return IRQ_HANDLED;
}

inline int fimc_is_hw_slot_id(int hw_id)
{
	int slot_id = -1;

	switch (hw_id) {
	case DEV_HW_3AA0:
		slot_id = 0;
		break;
	case DEV_HW_3AA1:
		slot_id = 1;
		break;
	case DEV_HW_ISP0:
		slot_id = 2;
		break;
	case DEV_HW_MCSC0:
		slot_id = 3;
		break;
	case DEV_HW_VRA:
		slot_id = 4;
		break;
	default:
		break;
	}

	return slot_id;
}

int fimc_is_get_hw_list(int group_id, int *hw_list)
{
	int i;
	int hw_index = 0;

	/* initialization */
	for (i = 0; i < GROUP_HW_MAX; i++)
		hw_list[i] = -1;

	switch (group_id) {
	case GROUP_ID_3AA0:
		hw_list[hw_index] = DEV_HW_3AA0; hw_index++;
		break;
	case GROUP_ID_3AA1:
		hw_list[hw_index] = DEV_HW_3AA1; hw_index++;
		break;
	case GROUP_ID_ISP0:
		hw_list[hw_index] = DEV_HW_ISP0; hw_index++;
		break;
	case GROUP_ID_MCS0:
		hw_list[hw_index] = DEV_HW_MCSC0; hw_index++;
		break;
	case GROUP_ID_VRA0:
		hw_list[hw_index] = DEV_HW_VRA; hw_index++;
		break;
	default:
		break;
	}

	if (hw_index > GROUP_HW_MAX)
		warn_hw("hw_index(%d) > GROUP_HW_MAX(%d)\n", hw_index, GROUP_HW_MAX);

	return hw_index;
}

static int fimc_is_hw_get_clk_gate(struct fimc_is_hw_ip *hw_ip, int hw_id)
{
	int ret = 0;


/* TODO: Need? */
#if 0
	struct fimc_is_clk_gate *clk_gate = NULL;

	switch (hw_id) {
	case DEV_HW_3AA0:
		clk_gate = &clk_gate_3aa0;
		clk_gate->regs = ioremap_nocache(0x16290084, 0x4);	/*Q-channel protocol related register*/
		if (!clk_gate->regs) {
			probe_err("Failed to remap clk_gate regs\n");
			ret = -ENOMEM;
		}
		hw_ip->clk_gate_idx = 0;
		clk_gate->bit[hw_ip->clk_gate_idx] = 0;
		clk_gate->refcnt[hw_ip->clk_gate_idx] = 0;

		spin_lock_init(&clk_gate->slock);
		break;
	case DEV_HW_3AA1:
		clk_gate = &clk_gate_3aa1;
		clk_gate->regs = ioremap_nocache(0x162A0084, 0x4);
		if (!clk_gate->regs) {
			probe_err("Failed to remap clk_gate regs\n");
			ret = -ENOMEM;
		}
		hw_ip->clk_gate_idx = 0;
		clk_gate->bit[hw_ip->clk_gate_idx] = 0;
		clk_gate->refcnt[hw_ip->clk_gate_idx] = 0;

		spin_lock_init(&clk_gate->slock);
		break;
	case DEV_HW_ISP0:
		clk_gate = &clk_gate_isp0;
		clk_gate->regs = ioremap_nocache(0x16430094, 0x4);
		if (!clk_gate->regs) {
			probe_err("Failed to remap clk_gate regs\n");
			ret = -ENOMEM;
		}
		hw_ip->clk_gate_idx = 0;
		clk_gate->bit[hw_ip->clk_gate_idx] = 0;
		clk_gate->refcnt[hw_ip->clk_gate_idx] = 0;

		spin_lock_init(&clk_gate->slock);
		break;
	case DEV_HW_MCSC0:
		clk_gate = &clk_gate_mcsc;
		clk_gate->regs = ioremap_nocache(0x16440004, 0x4);
		if (!clk_gate->regs) {
			probe_err("Failed to remap clk_gate regs\n");
			ret = -ENOMEM;
		}
		/* TODO : check bit :QACTIVE_ENABLE bit is [1] */

		hw_ip->clk_gate_idx = 0;
		clk_gate->bit[hw_ip->clk_gate_idx] = 0;
		clk_gate->refcnt[hw_ip->clk_gate_idx] = 0;

		spin_lock_init(&clk_gate->slock);
		break;
	case DEV_HW_VRA:
		clk_gate = &clk_gate_vra;
		clk_gate->regs = ioremap_nocache(0x1651b0dc, 0x4);
		if (!clk_gate->regs) {
			probe_err("Failed to remap clk_gate regs\n");
			ret = -ENOMEM;
		}
		hw_ip->clk_gate_idx = 0;
		clk_gate->bit[hw_ip->clk_gate_idx] = 0;
		clk_gate->refcnt[hw_ip->clk_gate_idx] = 0;

		spin_lock_init(&clk_gate->slock);
		break;
	default:
		probe_err("hw_id(%d) is invalid", hw_id);
		ret = -EINVAL;
	}

	hw_ip->clk_gate = clk_gate;
#endif

	return ret;
}

int fimc_is_hw_get_address(void *itfc_data, void *pdev_data, int hw_id)
{
	int ret = 0;
	struct resource *mem_res = NULL;
	struct platform_device *pdev = NULL;
	struct fimc_is_interface_hwip *itf_hwip = NULL;

	BUG_ON(!itfc_data);
	BUG_ON(!pdev_data);

	itf_hwip = (struct fimc_is_interface_hwip *)itfc_data;
	pdev = (struct platform_device *)pdev_data;

	switch (hw_id) {
	case DEV_HW_3AA0:
		mem_res = platform_get_resource(pdev, IORESOURCE_MEM, IORESOURCE_3AA0);
		if (!mem_res) {
			dev_err(&pdev->dev, "Failed to get io memory region\n");
			return -EINVAL;
		}

		itf_hwip->hw_ip->regs_start = mem_res->start;
		itf_hwip->hw_ip->regs_end = mem_res->end;
		itf_hwip->hw_ip->regs = ioremap_nocache(mem_res->start, resource_size(mem_res));
		if (!itf_hwip->hw_ip->regs) {
			dev_err(&pdev->dev, "Failed to remap io region\n");
			return -EINVAL;
		}

		info_itfc("[ID:%2d] 3AA VA(0x%p)\n", hw_id, itf_hwip->hw_ip->regs);
		break;
	case DEV_HW_3AA1:
		mem_res = platform_get_resource(pdev, IORESOURCE_MEM, IORESOURCE_3AA1);
		if (!mem_res) {
			dev_err(&pdev->dev, "Failed to get io memory region\n");
			return -EINVAL;
		}

		itf_hwip->hw_ip->regs_start = mem_res->start;
		itf_hwip->hw_ip->regs_end = mem_res->end;
		itf_hwip->hw_ip->regs = ioremap_nocache(mem_res->start, resource_size(mem_res));
		if (!itf_hwip->hw_ip->regs) {
			dev_err(&pdev->dev, "Failed to remap io region\n");
			return -EINVAL;
		}

		info_itfc("[ID:%2d] 3AA VA(0x%p)\n", hw_id, itf_hwip->hw_ip->regs);
		break;
	case DEV_HW_ISP0:
		mem_res = platform_get_resource(pdev, IORESOURCE_MEM, IORESOURCE_ISP);
		if (!mem_res) {
			dev_err(&pdev->dev, "Failed to get io memory region\n");
			return -EINVAL;
		}

		itf_hwip->hw_ip->regs_start = mem_res->start;
		itf_hwip->hw_ip->regs_end = mem_res->end;
		itf_hwip->hw_ip->regs = ioremap_nocache(mem_res->start, resource_size(mem_res));
		if (!itf_hwip->hw_ip->regs) {
			dev_err(&pdev->dev, "Failed to remap io region\n");
			return -EINVAL;
		}

		info_itfc("[ID:%2d] ISP VA(0x%p)\n", hw_id, itf_hwip->hw_ip->regs);
		break;
	case DEV_HW_MCSC0:
		mem_res = platform_get_resource(pdev, IORESOURCE_MEM, IORESOURCE_MCSC);
		if (!mem_res) {
			dev_err(&pdev->dev, "Failed to get io memory region\n");
			return -EINVAL;
		}

		itf_hwip->hw_ip->regs_start = mem_res->start;
		itf_hwip->hw_ip->regs_end = mem_res->end;
		itf_hwip->hw_ip->regs = ioremap_nocache(mem_res->start, resource_size(mem_res));
		if (!itf_hwip->hw_ip->regs) {
			dev_err(&pdev->dev, "Failed to remap io region\n");
			return -EINVAL;
		}

		info_itfc("[ID:%2d] MCSC0 VA(0x%p)\n", hw_id, itf_hwip->hw_ip->regs);
		break;
	case DEV_HW_VRA:
		mem_res = platform_get_resource(pdev, IORESOURCE_MEM, IORESOURCE_VRA_CH0);
		if (!mem_res) {
			dev_err(&pdev->dev, "Failed to get io memory region\n");
			return -EINVAL;
		}

		itf_hwip->hw_ip->regs_start = mem_res->start;
		itf_hwip->hw_ip->regs_end = mem_res->end;
		itf_hwip->hw_ip->regs = ioremap_nocache(mem_res->start, resource_size(mem_res));
		if (!itf_hwip->hw_ip->regs) {
			dev_err(&pdev->dev, "Failed to remap io region\n");
			return -EINVAL;
		}

		info_itfc("[ID:%2d] VRA0 VA(0x%p)\n", hw_id, itf_hwip->hw_ip->regs);

		mem_res = platform_get_resource(pdev, IORESOURCE_MEM, IORESOURCE_VRA_CH1);
		if (!mem_res) {
			dev_err(&pdev->dev, "Failed to get io memory region\n");
			return -EINVAL;
		}

		itf_hwip->hw_ip->regs_b_start = mem_res->start;
		itf_hwip->hw_ip->regs_b_end = mem_res->end;
		itf_hwip->hw_ip->regs_b = ioremap_nocache(mem_res->start, resource_size(mem_res));
		if (!itf_hwip->hw_ip->regs_b) {
			dev_err(&pdev->dev, "Failed to remap io region\n");
			return -EINVAL;
		}

		info_itfc("[ID:%2d] VRA1 VA(0x%p)\n", hw_id, itf_hwip->hw_ip->regs_b);
		break;
	default:
		probe_err("hw_id(%d) is invalid", hw_id);
		return -EINVAL;
		break;
	}

	ret = fimc_is_hw_get_clk_gate(itf_hwip->hw_ip, hw_id);;
	if (ret)
		dev_err(&pdev->dev, "fimc_is_hw_get_clk_gate is fail\n");

	return ret;
}

int fimc_is_hw_get_irq(void *itfc_data, void *pdev_data, int hw_id)
{
	struct fimc_is_interface_hwip *itf_hwip = NULL;
	struct platform_device *pdev = NULL;
	int ret = 0;

	BUG_ON(!itfc_data);

	itf_hwip = (struct fimc_is_interface_hwip *)itfc_data;
	pdev = (struct platform_device *)pdev_data;

	switch (hw_id) {
	case DEV_HW_3AA0:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 0);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq 3aa0-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 1);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq 3aa0-2\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_3AA1:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 2);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq 3aa1-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 3);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq 3aa1-2\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_ISP0:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 4);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq isp0-1\n");
			return -EINVAL;
		}

		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 5);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq isp0-2\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_MCSC0:
		itf_hwip->irq[INTR_HWIP1] = platform_get_irq(pdev, 6);
		if (itf_hwip->irq[INTR_HWIP1] < 0) {
			err("Failed to get irq scaler\n");
			return -EINVAL;
		}
		break;
	case DEV_HW_VRA:
		itf_hwip->irq[INTR_HWIP2] = platform_get_irq(pdev, 7);
		if (itf_hwip->irq[INTR_HWIP2] < 0) {
			err("Failed to get irq vra \n");
			return -EINVAL;
		}
		break;
	default:
		probe_err("hw_id(%d) is invalid", hw_id);
		return -EINVAL;
		break;
	}

	return ret;
}

//#define DECLARE_FUNC_NAME(NUM, NAME) fimc_is_isr##NUM_##NAME
static inline int __fimc_is_hw_request_irq(struct fimc_is_interface_hwip *itf_hwip,
	const char *name,
	int isr_num,
	irqreturn_t (*func)(int, void *))
{
	size_t name_len = 0;
	int ret = 0;

	name_len = sizeof(itf_hwip->irq_name[isr_num]);
	snprintf(itf_hwip->irq_name[isr_num], name_len, "fimc%s-%d", name, isr_num);
	ret = request_irq(itf_hwip->irq[isr_num], func,
		FIMC_IS_HW_IRQ_FLAG,
		itf_hwip->irq_name[isr_num],
		itf_hwip);
	if (ret) {
		err_itfc("[HW:%s] request_irq [%d] fail", name, isr_num);
		return -EINVAL;
	}
	itf_hwip->handler[isr_num].id = isr_num;
	itf_hwip->handler[isr_num].valid = true;

	return ret;
}

int fimc_is_hw_request_irq(void *itfc_data, int hw_id)
{
	struct fimc_is_interface_hwip *itf_hwip = NULL;
	int ret = 0;

	BUG_ON(!itfc_data);


	itf_hwip = (struct fimc_is_interface_hwip *)itfc_data;

	switch (hw_id) {
	case DEV_HW_3AA0:
		ret = __fimc_is_hw_request_irq(itf_hwip, "3aa0", INTR_HWIP1, fimc_is_isr1_3aa0);
		ret = __fimc_is_hw_request_irq(itf_hwip, "3aa0", INTR_HWIP2, fimc_is_isr2_3aa0);
		break;
	case DEV_HW_3AA1:
		ret = __fimc_is_hw_request_irq(itf_hwip, "3aa1", INTR_HWIP1, fimc_is_isr1_3aa1);
		ret = __fimc_is_hw_request_irq(itf_hwip, "3aa1", INTR_HWIP2, fimc_is_isr2_3aa1);
		break;
	case DEV_HW_ISP0:
		ret = __fimc_is_hw_request_irq(itf_hwip, "isp0", INTR_HWIP1, fimc_is_isr1_isp);
		ret = __fimc_is_hw_request_irq(itf_hwip, "isp0", INTR_HWIP2, fimc_is_isr2_isp);
		break;
	case DEV_HW_MCSC0:
		ret = __fimc_is_hw_request_irq(itf_hwip, "mcs0", INTR_HWIP1, fimc_is_isr1_mcs);
		break;
	case DEV_HW_VRA:
		ret = __fimc_is_hw_request_irq(itf_hwip, "vra", INTR_HWIP2, fimc_is_isr1_vra); /* VRA CH1 */
		break;
	default:
		probe_err("hw_id(%d) is invalid", hw_id);
		return -EINVAL;
		break;
	}

	return ret;
}

int fimc_is_hw_s_ctrl(void *itfc_data, int hw_id, enum hw_s_ctrl_id id, void *val)
{
	int ret = 0;

	switch (id) {
	case HW_S_CTRL_FULL_BYPASS:
		break;
	case HW_S_CTRL_CHAIN_IRQ:
		break;
	case HW_S_CTRL_HWFC_IDX_RESET:
		if (hw_id == FIMC_IS_VIDEO_M1P_NUM) {
			struct fimc_is_video_ctx *vctx = (struct fimc_is_video_ctx *)itfc_data;
			struct fimc_is_device_ischain *device;
			unsigned long data = (unsigned long)val;

			BUG_ON(!vctx);
			BUG_ON(!GET_DEVICE(vctx));

			device = GET_DEVICE(vctx);

			/* reset if this instance is reprocessing */
			if (test_bit(FIMC_IS_ISCHAIN_REPROCESSING, &device->state))
				writel(data, hwfc_rst);
		}
		break;
	default:
		break;
	}

	return ret;
}

int fimc_is_hw_g_ctrl(void *itfc_data, int hw_id, enum hw_g_ctrl_id id, void *val)
{
	int ret = 0;

	switch (id) {
	case HW_G_CTRL_FRM_DONE_WITH_DMA:
		*(bool *)val = true;
		break;
	case HW_G_CTRL_HAS_MCSC:
		*(bool *)val = true;
		break;
	case HW_G_CTRL_HAS_VRA_CH1_ONLY:
		*(bool *)val = true;
		break;
	}

	return ret;
}

int fimc_is_hw_query_cap(void *cap_data, int hw_id)
{
	int ret = 0;

	BUG_ON(!cap_data);

	switch (hw_id) {
	case DEV_HW_MCSC0:
		{
			struct fimc_is_hw_mcsc_cap *cap = (struct fimc_is_hw_mcsc_cap *)cap_data;
			/* v4.20 */
			cap->hw_ver = HW_SET_VERSION(4, 20, 0, 0);
			cap->max_output = 4;
			cap->hwfc = MCSC_CAP_SUPPORT;
			cap->in_otf = MCSC_CAP_SUPPORT;
			cap->in_dma = MCSC_CAP_SUPPORT;
			cap->out_dma[0] = MCSC_CAP_SUPPORT;
			cap->out_dma[1] = MCSC_CAP_SUPPORT;
			cap->out_dma[2] = MCSC_CAP_SUPPORT;
			cap->out_dma[3] = MCSC_CAP_SUPPORT;
			cap->out_dma[4] = MCSC_CAP_NOT_SUPPORT;
			cap->out_dma[5] = MCSC_CAP_NOT_SUPPORT;
			cap->out_otf[0] = MCSC_CAP_NOT_SUPPORT;
			cap->out_otf[1] = MCSC_CAP_NOT_SUPPORT;
			cap->out_otf[2] = MCSC_CAP_NOT_SUPPORT;
			cap->out_otf[3] = MCSC_CAP_NOT_SUPPORT;
			cap->out_otf[4] = MCSC_CAP_NOT_SUPPORT;
			cap->out_otf[5] = MCSC_CAP_NOT_SUPPORT;
			cap->out_hwfc[1] = MCSC_CAP_SUPPORT;
			cap->out_hwfc[2] = MCSC_CAP_SUPPORT;
			cap->enable_shared_output = false;
			cap->tdnr = MCSC_CAP_SUPPORT;
			cap->djag = MCSC_CAP_NOT_SUPPORT;
			cap->ysum = MCSC_CAP_NOT_SUPPORT;
			cap->ds_vra = MCSC_CAP_SUPPORT;
		}
		break;
	default:
		break;
	}

	return ret;
}

void __iomem *fimc_is_hw_get_sysreg(ulong core_regs)
{
	if (core_regs)
		err_itfc("%s: core_regs(%p)\n", __func__, (void *)core_regs);

	return ioremap_nocache(SYSREG_IS_BASE_ADDR, SZ_4K);
}

u32 fimc_is_hw_find_settle(u32 mipi_speed)
{
	u32 align_mipi_speed;
	u32 find_mipi_speed;
	size_t max;
	int s, e, m;

	align_mipi_speed = ALIGN(mipi_speed, 10);
	max = sizeof(fimc_is_csi_settle_table) / sizeof(u32);

	s = 0;
	e = max - 2;

	if (fimc_is_csi_settle_table[s] < align_mipi_speed)
		return fimc_is_csi_settle_table[s + 1];

	if (fimc_is_csi_settle_table[e] > align_mipi_speed)
		return fimc_is_csi_settle_table[e + 1];

	/* Binary search */
	while (s <= e) {
		m = ALIGN((s + e) / 2, 2);
		find_mipi_speed = fimc_is_csi_settle_table[m];

		if (find_mipi_speed == align_mipi_speed)
			break;
		else if (find_mipi_speed > align_mipi_speed)
			s = m + 2;
		else
			e = m - 2;
	}

	return fimc_is_csi_settle_table[m + 1];
}

unsigned int get_dma(struct fimc_is_device_sensor *device, u32 *dma_ch)
{
	struct fimc_is_core *core;
	struct fimc_is_device_csi *csi;
	u32 open_sensor_count = 0;
	int i;
	int ret = 0;

	/* fimc-is-v6_20 CSIS DMA is mapped 1:1 of each CSIS
	 * ex> CSIS0 - DMA0, CSIS3 - DMA3
	 * so dma_ch is delivered by CSIS channel(instance)
	 */
	*dma_ch = 0;
	core = device->private_data;
	for (i = 0; i < FIMC_IS_SENSOR_COUNT; i++) {
		if (test_bit(FIMC_IS_SENSOR_OPEN, &(core->sensor[i].state))) {
			open_sensor_count++;
			csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(core->sensor[i].subdev_csi);
			if (!csi) {
				err("csi is NULL");
				BUG();
			}

			*dma_ch |= (1 << csi->instance);
		}
	}

	if (open_sensor_count != 1 && open_sensor_count != 2) {
		err("invalid open sensor limit(%d)", open_sensor_count);
		ret = -EINVAL;
	}

	return ret;
}
