/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * decon doze file for Samsung EXYNOS DPU driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm_runtime.h>
#include <soc/samsung/exynos-pd.h>

#include "decon.h"
#include "dsim.h"
#include "decon_notify.h"

static void decon_set_black_window(struct decon_device *decon)
{
	struct decon_window_regs win_regs;
	struct decon_lcd *lcd = decon->lcd_info;

	memset(&win_regs, 0, sizeof(struct decon_window_regs));
	win_regs.wincon = wincon(0x8, 0xFF, 0xFF, 0xFF, DECON_BLENDING_NONE,
			decon->dt.dft_win);
	win_regs.start_pos = win_start_pos(0, 0);
	win_regs.end_pos = win_end_pos(0, 0, lcd->xres, lcd->yres);
	decon_dbg("xres %d yres %d win_start_pos %x win_end_pos %x\n",
			lcd->xres, lcd->yres, win_regs.start_pos,
			win_regs.end_pos);
	win_regs.colormap = 0x000000;
	win_regs.pixel_count = lcd->xres * lcd->yres;
	win_regs.whole_w = lcd->xres;
	win_regs.whole_h = lcd->yres;
	win_regs.offset_x = 0;
	win_regs.offset_y = 0;
	decon_dbg("pixel_count(%d), whole_w(%d), whole_h(%d), x(%d), y(%d)\n",
			win_regs.pixel_count, win_regs.whole_w,
			win_regs.whole_h, win_regs.offset_x,
			win_regs.offset_y);
	decon_reg_set_window_control(decon->id, decon->dt.dft_win,
			&win_regs, true);
	decon_reg_update_req_window(decon->id, decon->dt.dft_win);
}

static int decon_set_doze(struct decon_device *decon)
{
	struct decon_mode_info psr;
	struct decon_param p;
	int ret = 0;
	//struct dsim_device *dsim = container_of(decon->out_sd[0], struct dsim_device, sd);

	decon_info("+ %s: %d, %d\n", __func__, decon->state, decon->doze_state);

	mutex_lock(&decon->lock);

	DPU_EVENT_LOG(DPU_EVT_DOZE, &decon->sd, ktime_set(0, 0));

	if (decon->state == DECON_STATE_ON) {
		if (decon->doze_state != DOZE_STATE_DOZE) {
			ret = v4l2_subdev_call(decon->out_sd[0], core, ioctl, DSIM_IOC_DOZE, NULL);
			if (ret)
				decon_err("%s: failed to ioctl: %s\n", __func__, decon->out_sd[0]->name);
			decon->doze_state = DOZE_STATE_DOZE;
		}
		goto err;
	}

	if (decon->state == DECON_STATE_ON) {
		decon_warn("decon%d already enabled\n", decon->id);
		goto err;
	}

#if defined(CONFIG_PM)
	pm_runtime_get_sync(decon->dev);
#else
	decon_runtime_resume(decon->dev);
#endif

	if (decon->dt.psr_mode != DECON_VIDEO_MODE) {
		if (decon->res.pinctrl && decon->res.hw_te_on) {
			if (pinctrl_select_state(decon->res.pinctrl, decon->res.hw_te_on))
				decon_err("failed to turn on Decon_TE\n");
		}
	}

	pm_stay_awake(decon->dev);
	dev_warn(decon->dev, "pm_stay_awake");
	ret = v4l2_subdev_call(decon->out_sd[0], core, ioctl, DSIM_IOC_DOZE, NULL);
	if (ret)
		decon_err("%s: failed to ioctl: %s\n", __func__, decon->out_sd[0]->name);

	decon_to_init_param(decon, &p);
	decon_reg_init(decon->id, decon->dt.out_idx[0], &p);

	decon_to_psr_info(decon, &psr);

	if (decon->dt.out_type != DECON_OUT_WB && decon->dt.out_type != DECON_OUT_DP) {
		if (!IS_DOZE(decon->doze_state)) {
			decon_set_black_window(decon);
			/*
			 * Blender configuration must be set before DECON start.
			 * If DECON goes to start without window and blender configuration,
			 * DECON will go into abnormal state.
			 * DECON2(for DISPLAYPORT) start in winconfig
			 */
			decon_reg_start(decon->id, &psr);
			decon_reg_update_req_and_unmask(decon->id, &psr);
			usleep_range(17000, 18000);
		}
	}

	/*
	 * After turned on LCD, previous update region must be set as FULL size.
	 * DECON, DSIM and Panel are initialized as FULL size during UNBLANK
	 */
	DPU_FULL_RECT(&decon->win_up.prev_up_region, decon->lcd_info);

	if (!decon->id && !decon->eint_status) {
		enable_irq(decon->res.irq);
		decon->eint_status = 1;
	}

	decon->state = DECON_STATE_ON;
	decon_reg_set_int(decon->id, &psr, 1);
	decon->doze_state = DOZE_STATE_DOZE;
	//call_panel_ops(dsim, displayon, dsim);

err:
	mutex_unlock(&decon->lock);

	decon_info("- %s: %d\n", __func__, decon->id);

	return ret;
}

static int decon_set_doze_suspend(struct decon_device *decon)
{
	struct decon_mode_info psr;
	int ret = 0;

	decon_info("+ %s: %d, %d\n", __func__, decon->state, decon->doze_state);

	mutex_lock(&decon->lock);

	DPU_EVENT_LOG(DPU_EVT_DOZE_SUSPEND, &decon->sd, ktime_set(0, 0));

	if (decon->state == DECON_STATE_OFF) {
		decon_info("decon%d already disabled\n", decon->id);
		ret = -EEXIST;
		goto err;
	}

	flush_kthread_worker(&decon->up.worker);

	decon_to_psr_info(decon, &psr);
	decon_reg_set_int(decon->id, &psr, 0);

	if (!decon->id && (decon->vsync.irq_refcount <= 0) &&
			decon->eint_status) {
		disable_irq(decon->res.irq);
		decon->eint_status = 0;
	}

	decon_to_psr_info(decon, &psr);
	ret = decon_reg_stop(decon->id, decon->dt.out_idx[0], &psr);
	if (ret < 0) {
		decon_err("%s, failed to decon_reg_stop\n", __func__);
		/* call decon instant off */
		decon_reg_direct_on_off(decon->id, 0);
	}
	decon_reg_clear_int_all(decon->id);

	/* DMA protection disable must be happen on dpp domain is alive */
	if (decon->dt.out_type != DECON_OUT_WB) {
#if defined(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
		decon_set_protected_content(decon, NULL);
#endif
		decon->cur_using_dpp = 0;
		decon_dpp_stop(decon, false);
	}

	decon->bts.ops->bts_release_bw(decon);

	ret = v4l2_subdev_call(decon->out_sd[0], core, ioctl, DSIM_IOC_DOZE_SUSPEND, NULL);
	if (ret)
		decon_err("%s: failed to ioctl: %s\n", __func__, decon->out_sd[0]->name);

	if (decon->dt.out_type == DECON_OUT_DSI) {
		pm_relax(decon->dev);
		dev_warn(decon->dev, "pm_relax");
	}

	if (decon->dt.psr_mode != DECON_VIDEO_MODE) {
		if (decon->res.pinctrl && decon->res.hw_te_off) {
			if (pinctrl_select_state(decon->res.pinctrl,
						decon->res.hw_te_off)) {
				decon_err("failed to turn off Decon_TE\n");
			}
		}
	}

#if defined(CONFIG_PM)
	pm_runtime_put_sync(decon->dev);
#else
	decon_runtime_suspend(decon->dev);
#endif
#ifdef CONFIG_EXYNOS_PD
	if (decon->exynos_pd && decon->exynos_pd->check_status) {
		struct exynos_pm_domain *exynos_pd = decon->exynos_pd;

		if (exynos_pd->check_status(exynos_pd)) {
			decon_dbg("decon%d pd-dispaud still on\n", decon->id);

			if (decon_reg_instant_stop(decon->id, IDLE_WAIT_TIMEOUT))
				decon_warn("%s: instant stop fail\n", __func__);

			decon_reg_reset(decon->id);
		}
	}
#endif
	decon->state = DECON_STATE_OFF;
	decon->doze_state = DOZE_STATE_DOZE_SUSPEND;

err:
	mutex_unlock(&decon->lock);

	decon_info("- %s: %d\n", __func__, decon->id);

	return ret;
}

int decon_set_doze_mode(struct decon_device *decon, u32 mode)
{
	int ret = 0;

	int (*doze_cb[DECON_PWR_MAX])(struct decon_device *) = {
		NULL,
		decon_set_doze,
		NULL,
		decon_set_doze_suspend,
	};

	if (decon->dt.out_type != DECON_OUT_DSI)
		return ret;

	if (mode >= DECON_PWR_MAX) {
		decon_err("%s: invalid mode: %d\n", __func__, mode);
		return -EINVAL;
	}

	if (doze_cb[mode] == NULL)
		return ret;

	decon_info("%s: decon%d pwr_state %d(%s)\n", __func__, decon->id, mode, (mode == DECON_PWR_DOZE) ? "DOZE" : "DOZE_SUSPEND");

	decon_simple_notifier_call_chain(DECON_EARLY_EVENT_DOZE, (mode == DECON_PWR_DOZE) ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN);

	ret = doze_cb[mode](decon);

	decon_simple_notifier_call_chain(DECON_EVENT_DOZE, (mode == DECON_PWR_DOZE) ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN);

	return ret;
}

