#include "../pmucal_common.h"
#include "../pmucal_cpu.h"
#include "../pmucal_local.h"
#include "../pmucal_rae.h"
#include "../pmucal_system.h"

#include "pmucal_cpu_exynos7872.h"
#include "pmucal_local_exynos7872.h"
#include "pmucal_p2vmap_exynos7872.h"
#include "pmucal_system_exynos7872.h"

#include "cmucal-node.c"
#include "cmucal-qch.c"
#include "cmucal-sfr.c"
#include "cmucal-vclk.c"
#include "cmucal-vclklut.c"

#include "clkout_exynos7872.c"
#include "acpm_dvfs_exynos7872.h"
#include "asv_exynos7872.h"

#include "../ra.h"
/* WPLL_USB_PLL Clock Type */
#define WPLL_USBPLL_CON0	(0x200)
#define WPLL_USBPLL_CON1	(0x204)

#define WIFI2AP_USBPLL_ACK	(1)
#define AP2WIFI_USBPLL_REQ	(0)

#define USBPLL_WPLL_AFC_START	(2)
#define USBPLL_WPLL_EN		(1)
#define USBPLL_WPLL_SEL		(0)

void __iomem	*fsys_reg;

int wpll_usbpll_is_enabled(unsigned int id)
{
	if (get_bit(fsys_reg + WPLL_USBPLL_CON1, USBPLL_WPLL_SEL) == 1) {
		pr_info("%s: USBPLL_WPLL_SEL==1\n", __func__);
		return get_bit(fsys_reg + WPLL_USBPLL_CON1, USBPLL_WPLL_EN);
	}
	pr_info("%s: USBPLL_WPLL_SEL==0\n", __func__);

	return get_bit(fsys_reg + WPLL_USBPLL_CON0, WIFI2AP_USBPLL_ACK);
}

int wpll_usbpll_enable(unsigned int id)
{
	int timeout = 0;
	/* check changed WPLL input selection to AP (AP2WLBT_USBPLL_WPLL_SEL). */
	if (get_bit(fsys_reg + WPLL_USBPLL_CON1, USBPLL_WPLL_SEL) == 0x1) {
		pr_info("%s AP %p\n", __func__, fsys_reg + WPLL_USBPLL_CON1);

		if (get_bit(fsys_reg + WPLL_USBPLL_CON1, USBPLL_WPLL_EN) == 0x1) {
			pr_info("%s USBPLL_WPLL_EN==1\n", __func__);
			return 0;
		}

		/* Set WPLL enable (AP2WLBT_USBPLL_WPLL_EN) */
		set_bit_val(fsys_reg + WPLL_USBPLL_CON1, USBPLL_WPLL_EN, 0x1);

		/* wait 20us for power settle time. */
		for (timeout = 0;; timeout++) {
			if (timeout >= 20)
				break;
			cpu_relax();
		}

		/* Set WPLL AFC Start (AP2WLBT_USBPLL_WPLL_AFC_START) */
		set_bit_val(fsys_reg + WPLL_USBPLL_CON1, USBPLL_WPLL_AFC_START, 0x1);

		/* wait 60us for clock stabilization. */
		for (timeout = 0;; timeout++) {
			if (timeout >= 60)
				break;
			cpu_relax();
		}

	} else {
		pr_info("%s WLBT\n", __func__);

		/* WPLL input selection to WLBT */

		/* (IP) use ack */
		if (get_bit(fsys_reg + WPLL_USBPLL_CON0, WIFI2AP_USBPLL_ACK) == 0x1) {
			pr_info("%s USBPLL_ACK==1, already\n", __func__);
			return 0;
		}

		/* AP2WIFI_USBPLL_REQ */
		set_bit_val(fsys_reg + WPLL_USBPLL_CON0, AP2WIFI_USBPLL_REQ, 0x1);
		pr_info("%s AP2WIFI_USBPLL_REQ=1\n", __func__);
	}

	return 0;
}

int wpll_usbpll_disable(unsigned int id)
{
	if (get_bit(fsys_reg + WPLL_USBPLL_CON1, USBPLL_WPLL_SEL) == 0) {
		set_bit_val(fsys_reg + WPLL_USBPLL_CON0, AP2WIFI_USBPLL_REQ, 0x0);
		pr_info("%s: AP2WIFI_USBPLL_REQ=0\n", __func__);
	}

	return 0;
}


void wpll_usbpll_set_rate(unsigned int id, unsigned int rate)
{
	if (rate == 0) {
		if (wpll_usbpll_is_enabled(id) != 0)
			if (wpll_usbpll_disable(id))
				return;
	} else { /* rate != 0  */
		if (wpll_usbpll_is_enabled(id) == 0)
			wpll_usbpll_enable(id);
	}
}

unsigned int wpll_usbpll_get_rate(unsigned int id)
{
	unsigned int fout;

	if (wpll_usbpll_is_enabled(id) == 0)
		return 0;

	fout = (20 * 1000 * 1000);

	return fout;
}

struct vclk_trans_ops wpll_usb_pll_ops = {
	.set_pll = wpll_usbpll_set_rate,
	.get_pll = wpll_usbpll_get_rate,
	.enable = wpll_usbpll_enable,
	.disable = wpll_usbpll_disable,
};

void exynos7872_cal_data_init(void)
{
	struct vclk *vclk;

	vclk = cmucal_get_node(VCLK_WPLL_USBPLL);
	if (!vclk)
		return;

	vclk->ops = &wpll_usb_pll_ops;
	fsys_reg = ioremap(0x13400000, 0x400);
	if (!fsys_reg)
		pr_err("fsys_reg ioremap failed\n");
}

void (*cal_data_init)(void) = exynos7872_cal_data_init;
