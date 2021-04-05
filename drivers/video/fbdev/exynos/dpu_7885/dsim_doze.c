/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * dsim doze file for Samsung EXYNOS DPU driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm_runtime.h>
#include <linux/phy/phy.h>

#include "decon.h"
#include "dsim.h"
#include "decon_board.h"

int dsim_doze(struct dsim_device *dsim)
{
	int ret = 0;

	if (dsim->state == DSIM_STATE_ON) {
		if (dsim->doze_state != DOZE_STATE_DOZE)
			call_panel_ops(dsim, doze, dsim);
		goto exit;
	}

	dsim_info("+ %s: %d, %d\n", __func__, dsim->state, dsim->doze_state);

#if defined(CONFIG_PM)
	pm_runtime_get_sync(dsim->dev);
#else
	dsim_runtime_resume(dsim->dev);
#endif

	dpu_sysreg_set_lpmux(dsim->res.ss_regs);

	dphy_power_on(dsim, 1);

	if (dsim->doze_state == DOZE_STATE_SUSPEND) {
		/* Panel power on */
		run_list(dsim->dev, "dsim_set_panel_power_enable");
	}

	/* Do DPHY reset */
	dsim_reg_dphy_reset(dsim->id);

	dsim_reg_sw_reset(dsim->id);

	dsim_reg_set_clocks(dsim->id, &dsim->clks, &dsim->lcd_info.dphy_pms, 1);

	dsim_reg_set_lanes(dsim->id, dsim->data_lane, 1);

	dsim_reg_set_esc_clk_on_lane(dsim->id, 1, dsim->data_lane);
	dsim_reg_enable_word_clock(dsim->id, 1);

	if (dsim_reg_init(dsim->id, &dsim->lcd_info, dsim->data_lane_cnt, &dsim->clks) < 0) {
		dsim_info("dsim_%d already enabled", dsim->id);
		ret = -EBUSY;
	} else {

		dsim_info("dsim_%d enabled", dsim->id);
		/* Panel reset should be set after LP-11 */
		if (dsim->doze_state == DOZE_STATE_SUSPEND)
			run_list(dsim->dev, "dsim_reset_panel");
	}

	dsim_reg_start(dsim->id);

	dsim->state = DSIM_STATE_ON;

	enable_irq(dsim->res.irq);

	if (dsim->doze_state == DOZE_STATE_SUSPEND || dsim->doze_state == DOZE_STATE_DOZE_SUSPEND)
		call_panel_ops(dsim, doze, dsim);

exit:
	dsim->doze_state = DOZE_STATE_DOZE;

	dsim_info("- %s\n", __func__);

	return ret;
}

int dsim_doze_suspend(struct dsim_device *dsim)
{
	if (dsim->state == DSIM_STATE_OFF)
		goto exit;

	dsim_info("+ %s: %d, %d\n", __func__, dsim->state, dsim->doze_state);

	if (dsim->doze_state == DOZE_STATE_NORMAL)
		call_panel_ops(dsim, doze, dsim);

	dsim->doze_state = DOZE_STATE_DOZE_SUSPEND;

	/* Wait for current read & write CMDs. */
	mutex_lock(&dsim->cmd_lock);
	del_timer(&dsim->cmd_timer);
	dsim->state = DSIM_STATE_OFF;
	mutex_unlock(&dsim->cmd_lock);

	disable_irq(dsim->res.irq);
	dsim_reg_stop(dsim->id, dsim->data_lane);

	dphy_power_on(dsim, 0);

#if defined(CONFIG_PM)
	pm_runtime_put_sync(dsim->dev);
#else
	dsim_runtime_suspend(dsim->dev);
#endif

exit:
	dsim_info("- %s\n", __func__);

	return 0;
}

