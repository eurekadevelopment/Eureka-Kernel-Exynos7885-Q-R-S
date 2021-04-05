/*
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/wakeup_reason.h>
#include <linux/gpio.h>
#include <linux/syscore_ops.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/psci.h>
#include <linux/debugfs.h>
#include <asm/cpuidle.h>
#include <asm/smp_plat.h>

#include <soc/samsung/exynos-pm.h>
#include <soc/samsung/exynos-pmu.h>
#include <soc/samsung/exynos-powermode.h>

#ifdef CONFIG_SEC_DEBUG
#include <linux/sec_debug.h>

#define	PMU_CP_STAT		0x0038
#define	PMU_GNSS_STAT		0x0048
#define PMU_WIFI_STAT		0x0148
#define PMU_CONNECT_REQ_STATUS	0x00d4
#endif
#ifdef CONFIG_SEC_PM
#define WKUP_STAT_SIZE		32
#define WKUP_STAT_NAME_LENGTH	16

struct wakeup_stat {
	char name[WKUP_STAT_NAME_LENGTH];
	u32 addr;
	unsigned long src_check_bit;
	const char *wkup_src[WKUP_STAT_SIZE];
	int eint_wkup_bit;
};
#endif

extern u32 exynos_eint_to_pin_num(int eint);

struct exynos_wkup_reason {
	u32 wkstat_idx;
	u32 wkstat_bit;
};

struct exynos_pm_info {
	void __iomem *eint_base;		/* GPIO_ALIVE base to check wkup reason */
	void __iomem *gic_base;			/* GICD_ISPENDRn base to check wkup reason */
	unsigned int num_eint;			/* Total number of EINT sources */
	unsigned int num_gic;			/* Total number of GIC sources */
	bool is_early_wakeup;
	bool is_cp_call;
	unsigned int suspend_mode_idx;		/* power mode to be used in suspend scenario */
	unsigned int suspend_psci_idx;		/* psci index to be used in suspend scenario */
	unsigned int cp_call_mode_idx;		/* power mode to be used in cp_call scenario */
	unsigned int cp_call_psci_idx;		/* psci index to be used in cp_call scenario */
	u8 num_wkup_stats;			/* Total number of wakeup_stat registers */
	unsigned int *wkup_stats;		/* Register addresses of WAKEUP_STAT_N registers */
	struct exynos_wkup_reason by_eint;
	struct exynos_wkup_reason by_rtc_alarm;
	unsigned int *eint_wkup_masks;		/* Register addresses of EINT_WAKEUP_MASK_N registers */
	u8 num_eint_wkup_masks;			/* Total number of EINT_WAKEUP_MASK_N registers */
	unsigned int *eint_pends;		/* Register addresses of EINT pending registers */
	u8 num_eint_pends;			/* Total number of EINT pending registers */
	unsigned int conn_req_offset;
	unsigned int prev_conn_req;		/* TCXO, PWR, MIF request status of masters */
#ifdef CONFIG_SEC_PM
	struct wakeup_stat *ws_extra;
	bool print_by_extra;
#endif
};
static struct exynos_pm_info *pm_info;

struct exynos_pm_dbg {
	u32 test_early_wakeup;
	u32 test_cp_call;
};
static struct exynos_pm_dbg *pm_dbg;

static void exynos_show_wakeup_reason_eint(void)
{
	int bit;
	int i, size, cnt;
	long unsigned int ext_int_pend;
	u64 eint_wakeup_mask = 0;
	bool found = 0;
	unsigned int val;

	for (i = 0; i < pm_info->num_eint_wkup_masks; i++) {
		exynos_pmu_read(pm_info->eint_wkup_masks[i], &val);
		eint_wakeup_mask |= (val << (32 * i));
	}

	for (i = 0, size = 8, cnt = 0; i < pm_info->num_eint; i += size, cnt++) {
		ext_int_pend = __raw_readl(pm_info->eint_base + pm_info->eint_pends[cnt]);

		for_each_set_bit(bit, &ext_int_pend, size) {
			u32 gpio;
			int irq;

			if (eint_wakeup_mask & (1 << (i + bit)))
				continue;

			gpio = exynos_eint_to_pin_num(i + bit);
			irq = gpio_to_irq(gpio);

#ifdef CONFIG_SUSPEND
			log_wakeup_reason(irq);
#endif
			found = 1;
		}
	}

	if (!found)
		pr_info("%s Resume caused by unknown EINT\n", EXYNOS_PM_PREFIX);
}

#ifdef CONFIG_SEC_PM
static void __exynos_show_wakeup_registers_extra(void)
{
	int i, bit;
	unsigned int wkup_stats;
	bool is_eint_wkup = false;

	for (i = 0; i < pm_info->num_wkup_stats; i++) {
		exynos_pmu_read(pm_info->ws_extra[i].addr, &wkup_stats);
		pr_info("%s : 0x%08x\n", pm_info->ws_extra[i].name, wkup_stats);

		wkup_stats &= pm_info->ws_extra[i].src_check_bit;

		if (pm_info->ws_extra[i].eint_wkup_bit >= 0 &&
				wkup_stats & (1 << pm_info->ws_extra[i].eint_wkup_bit))
			is_eint_wkup = true;
		else {
			for_each_set_bit(bit, (unsigned long *) &wkup_stats, WKUP_STAT_SIZE)
				pr_info("%s Resume caused by %s\n", EXYNOS_PM_PREFIX,
					pm_info->ws_extra[i].wkup_src[bit]);
		}
	}

	if (is_eint_wkup) {
		pr_info("EINT_PEND: ");
		for (i = 0; i < pm_info->num_eint_pends; i++)
			pr_info("0x%02x ", __raw_readl(pm_info->eint_base + pm_info->eint_pends[i]));

		exynos_show_wakeup_reason_eint();
	}
}

static int exynos_show_wakeup_registers_extra(void)
{
	if (pm_info->print_by_extra) {
		__exynos_show_wakeup_registers_extra();
		return 1;
	}

	return 0;
}
#else
static inline int exynos_show_wakeup_registers_extra(void) { return 0; }
#endif /* !CONFIG_SEC_PM */

static void exynos_show_wakeup_registers(void)
{
	int i;
	int wkup_stats;

	if (exynos_show_wakeup_registers_extra())
		return;

	pr_info("WAKEUP_STAT:\n");
	for (i = 0; i < pm_info->num_wkup_stats; i++) {
		exynos_pmu_read(pm_info->wkup_stats[i], &wkup_stats);
		pr_info("0x%08x\n", wkup_stats);
		if ((i == pm_info->by_eint.wkstat_idx) && (wkup_stats & (1 << pm_info->by_eint.wkstat_bit)))
			exynos_show_wakeup_reason_eint();
		else if ((i == pm_info->by_rtc_alarm.wkstat_idx) && (wkup_stats & (1 << pm_info->by_rtc_alarm.wkstat_bit)))
			pr_info("%s Resume caused by RTC alarm\n", EXYNOS_PM_PREFIX);
	}

	pr_info("EINT_PEND: ");
	for (i = 0; i < pm_info->num_eint_pends; i++)
		pr_info("0x%02x ", __raw_readl(pm_info->eint_base + pm_info->eint_pends[i]));

}

static void exynos_show_wakeup_reason(bool sleep_abort)
{
	int i;

	pr_info("---------- wakeup ----------\n");

	if (sleep_abort) {
		pr_info("%s early wakeup! Dumping pending registers...\n", EXYNOS_PM_PREFIX);

		pr_info("EINT_PEND:\n");
		for (i = 0; i < pm_info->num_eint_pends; i++)
			pr_info("0x%x\n", __raw_readl(pm_info->eint_base + pm_info->eint_pends[i]));

		pr_info("GIC_PEND:\n");
		for (i = 0; i < pm_info->num_gic; i++)
			pr_info("GICD_ISPENDR[%d] = 0x%x\n", i, __raw_readl(pm_info->gic_base + i*4));

		pr_info("%s done.\n", EXYNOS_PM_PREFIX);
		return ;
	}

	exynos_show_wakeup_registers();
}

#ifdef CONFIG_CPU_IDLE
static DEFINE_RWLOCK(exynos_pm_notifier_lock);
static RAW_NOTIFIER_HEAD(exynos_pm_notifier_chain);

int exynos_pm_register_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&exynos_pm_notifier_lock, flags);
	ret = raw_notifier_chain_register(&exynos_pm_notifier_chain, nb);
	write_unlock_irqrestore(&exynos_pm_notifier_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pm_register_notifier);

int exynos_pm_unregister_notifier(struct notifier_block *nb)
{
	unsigned long flags;
	int ret;

	write_lock_irqsave(&exynos_pm_notifier_lock, flags);
	ret = raw_notifier_chain_unregister(&exynos_pm_notifier_chain, nb);
	write_unlock_irqrestore(&exynos_pm_notifier_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pm_unregister_notifier);

static int __exynos_pm_notify(enum exynos_pm_event event, int nr_to_call, int *nr_calls)
{
	int ret;

	ret = __raw_notifier_call_chain(&exynos_pm_notifier_chain, event, NULL,
		nr_to_call, nr_calls);

	return notifier_to_errno(ret);
}

int exynos_pm_notify(enum exynos_pm_event event)
{
	int nr_calls;
	int ret = 0;

	read_lock(&exynos_pm_notifier_lock);
	ret = __exynos_pm_notify(event, -1, &nr_calls);
	read_unlock(&exynos_pm_notifier_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(exynos_pm_notify);
#endif /* CONFIG_CPU_IDLE */

#ifdef CONFIG_SND_SOC_SAMSUNG_VTS
extern bool vts_is_on(void);
#else
static inline bool vts_is_on(void)
{
	return 0;
}
#endif
#ifdef CONFIG_SND_SOC_SAMSUNG_ABOX
extern bool abox_is_on(void);
#else
static inline bool abox_is_on(void)
{
	return 0;
}
#endif

static int exynos_pm_syscore_suspend(void)
{
#ifdef CONFIG_SEC_DEBUG
	unsigned int val;

	if (sec_debug_get_debug_level() <= 1) {
		exynos_pmu_read(PMU_CP_STAT, &val);
		pr_info("CP_STAT (0x%x) = 0x%08x\n", PMU_CP_STAT, val);	
		exynos_pmu_read(PMU_GNSS_STAT, &val);
		pr_info("GNSS_STAT (0x%x) = 0x%08x\n", PMU_GNSS_STAT, val);
		exynos_pmu_read(PMU_WIFI_STAT, &val);
		pr_info("WIFI_STAT (0x%x) = 0x%08x\n", PMU_WIFI_STAT, val);
		exynos_pmu_read(PMU_CONNECT_REQ_STATUS, &val);
		pr_info("PMU_CONNECT_REQ_STATUS (0x%x) = 0x%08x\n", PMU_CONNECT_REQ_STATUS, val);	
	}
#endif
	pm_info->is_cp_call = abox_is_on();
	if (pm_info->is_cp_call || pm_dbg->test_cp_call) {
		exynos_prepare_sys_powerdown(pm_info->cp_call_mode_idx);
		pr_info("%s %s: Enter CP Call scenario. (mode_idx = %d)\n",
				EXYNOS_PM_PREFIX, __func__, pm_info->cp_call_mode_idx);
	} else {
		if (vts_is_on())
			exynos_prepare_sys_powerdown(SYS_SLEEP_VTS_ON);
		else
			exynos_prepare_sys_powerdown(pm_info->suspend_mode_idx);

		pr_info("%s %s: Enter Suspend scenario. (mode_idx = %d)\n",
				EXYNOS_PM_PREFIX,__func__, pm_info->suspend_mode_idx);
	}

	return 0;
}

static void exynos_pm_syscore_resume(void)
{
	if (pm_info->is_cp_call || pm_dbg->test_cp_call)
		exynos_wakeup_sys_powerdown(pm_info->cp_call_mode_idx, pm_info->is_early_wakeup);
	else {
		if (vts_is_on())
			exynos_wakeup_sys_powerdown(SYS_SLEEP_VTS_ON, pm_info->is_early_wakeup);
		else
			exynos_wakeup_sys_powerdown(pm_info->suspend_mode_idx, pm_info->is_early_wakeup);
	}

	exynos_show_wakeup_reason(pm_info->is_early_wakeup);

	if (!pm_info->is_early_wakeup)
		pr_debug("%s %s: post sleep, preparing to return\n",
						EXYNOS_PM_PREFIX, __func__);
}

static struct syscore_ops exynos_pm_syscore_ops = {
	.suspend	= exynos_pm_syscore_suspend,
	.resume		= exynos_pm_syscore_resume,
};

#ifdef CONFIG_SEC_GPIO_DVS
extern void gpio_dvs_check_sleepgpio(void);
#endif

static int exynos_pm_enter(suspend_state_t state)
{
	unsigned int psci_index;
	unsigned int prev_mif = 0, post_mif = 0;

	if (pm_info->is_cp_call || pm_dbg->test_cp_call)
		psci_index = pm_info->cp_call_psci_idx;
	else
		psci_index = pm_info->suspend_psci_idx;

	/* Send an IPI if test_early_wakeup flag is set */
	if (pm_dbg->test_early_wakeup)
		arch_send_call_function_single_ipi(0);

#ifdef CONFIG_SEC_GPIO_DVS
	/************************ Caution !!! ****************************/
	/* This function must be located in appropriate SLEEP position
	 * in accordance with the specification of each BB vendor.
	 */
	/************************ Caution !!! ****************************/
	gpio_dvs_check_sleepgpio();
#endif

	prev_mif = acpm_get_mifdn_count();
	exynos_pmu_read(pm_info->conn_req_offset, &pm_info->prev_conn_req);

	/* This will also act as our return point when
	 * we resume as it saves its own register state and restores it
	 * during the resume. */
	pm_info->is_early_wakeup = (bool)arm_cpuidle_suspend(psci_index);
	if (pm_info->is_early_wakeup)
		pr_info("%s %s: return to originator\n",
				EXYNOS_PM_PREFIX, __func__);

	post_mif = acpm_get_mifdn_count();

	if (post_mif == prev_mif)
		pr_info("%s: MIF blocked. prev_conn_req: 0x%x\n", EXYNOS_PM_PREFIX, pm_info->prev_conn_req);
	else
		pr_info("%s: MIF down. cur_count: %d, acc_count: %d\n",
				EXYNOS_PM_PREFIX, post_mif - prev_mif, post_mif);

	return pm_info->is_early_wakeup;
}

static const struct platform_suspend_ops exynos_pm_ops = {
	.enter		= exynos_pm_enter,
	.valid		= suspend_valid_only_mem,
};

bool is_test_cp_call_set(void)
{
	if (!pm_dbg)
		return false;

	return pm_dbg->test_cp_call;
}
EXPORT_SYMBOL_GPL(is_test_cp_call_set);

#ifdef CONFIG_DEBUG_FS
static void __init exynos_pm_debugfs_init(void)
{
	struct dentry *root, *d;

	root = debugfs_create_dir("exynos-pm", NULL);
	if (!root) {
		pr_err("%s %s: could't create debugfs dir\n", EXYNOS_PM_PREFIX, __func__);
		return;
	}

	d = debugfs_create_u32("test_early_wakeup", 0644, root, &pm_dbg->test_early_wakeup);
	if (!d) {
		pr_err("%s %s: could't create debugfs test_early_wakeup\n",
					EXYNOS_PM_PREFIX, __func__);
		return;
	}

	d = debugfs_create_u32("test_cp_call", 0644, root, &pm_dbg->test_cp_call);
	if (!d) {
		pr_err("%s %s: could't create debugfs test_cp_call\n",
					EXYNOS_PM_PREFIX, __func__);
		return;
	}

	d = debugfs_create_x32("last_mif_blocker", 0444, root, &pm_info->prev_conn_req);
	if (!d) {
		pr_err("%s %s: could't create debugfs last_mif_blocker\n",
					EXYNOS_PM_PREFIX, __func__);
		return;
	}
}
#endif

#if defined(CONFIG_SEC_FACTORY)
enum ids_info {
	tg,
	lg,
	bg,
	g3dg,
	mifg,
	bids,
	gids,
};

extern int asv_ids_information(enum ids_info id);

static ssize_t show_asv_info(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int count = 0;

	/* Set asv group info to buf */
	count += sprintf(&buf[count], "%d ", asv_ids_information(tg));
	count += sprintf(&buf[count], "%03x ", asv_ids_information(bg));
	count += sprintf(&buf[count], "%03x ", asv_ids_information(g3dg));
	count += sprintf(&buf[count], "%u ", asv_ids_information(bids));
	count += sprintf(&buf[count], "%u ", asv_ids_information(gids));
	count += sprintf(&buf[count], "\n");

	return count;
}

static DEVICE_ATTR(asv_info, 0664, show_asv_info, NULL);
#endif /* CONFIG_SEC_FACTORY */

static __init int exynos_pm_drvinit(void)
{
	int ret;

	pm_info = kzalloc(sizeof(struct exynos_pm_info), GFP_KERNEL);
	if (pm_info == NULL) {
		pr_err("%s %s: failed to allocate memory for exynos_pm_info\n",
					EXYNOS_PM_PREFIX, __func__);
		BUG();
	}

	pm_dbg = kzalloc(sizeof(struct exynos_pm_dbg), GFP_KERNEL);
	if (pm_dbg == NULL) {
		pr_err("%s %s: failed to allocate memory for exynos_pm_dbg\n",
					EXYNOS_PM_PREFIX, __func__);
		BUG();
	}

	if (of_have_populated_dt()) {
		struct device_node *np;
		np = of_find_compatible_node(NULL, NULL, "samsung,exynos-pm");
		if (!np) {
			pr_err("%s %s: unabled to find compatible node (%s)\n",
					EXYNOS_PM_PREFIX, __func__, "samsung,exynos-pm");
			BUG();
		}

		pm_info->eint_base = of_iomap(np, 0);
		if (!pm_info->eint_base) {
			pr_err("%s %s: unabled to ioremap EINT base address\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		pm_info->gic_base = of_iomap(np, 1);
		if (!pm_info->gic_base) {
			pr_err("%s %s: unbaled to ioremap GIC base address\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "num-eint", &pm_info->num_eint);
		if (ret) {
			pr_err("%s %s: unabled to get the number of eint from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "num-gic", &pm_info->num_gic);
		if (ret) {
			pr_err("%s %s: unabled to get the number of gic from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "suspend_mode_idx", &pm_info->suspend_mode_idx);
		if (ret) {
			pr_err("%s %s: unabled to get suspend_mode_idx from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "suspend_psci_idx", &pm_info->suspend_psci_idx);
		if (ret) {
			pr_err("%s %s: unabled to get suspend_psci_idx from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "cp_call_mode_idx", &pm_info->cp_call_mode_idx);
		if (ret) {
			pr_err("%s %s: unabled to get cp_call_mode_idx from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_read_u32(np, "cp_call_psci_idx", &pm_info->cp_call_psci_idx);
		if (ret) {
			pr_err("%s %s: unabled to get cp_call_psci_idx from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}

		ret = of_property_count_u32_elems(np, "wkup_stats");
		if (!ret) {
			pr_err("%s %s: unabled to get wakeup_stat value from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		} else {
			pm_info->num_wkup_stats = ret;
			pm_info->wkup_stats = kzalloc(sizeof(unsigned int) * ret, GFP_KERNEL);
			if (pm_info->wkup_stats == NULL) {
				pr_err("%s %s: failed to allocate memory for wkup_stats\n",
							EXYNOS_PM_PREFIX, __func__);
				BUG();
			}
			of_property_read_u32_array(np, "wkup_stats", pm_info->wkup_stats, ret);
		}

		do {
			u32 tmp[2];

			ret = of_property_read_u32_array(np, "wkup_by_eint", tmp, 2);
			if (!ret) {
				pm_info->by_eint.wkstat_idx = tmp[0];
				pm_info->by_eint.wkstat_bit = tmp[1];
			}

			ret = of_property_read_u32_array(np, "wkup_by_rtc_alarm", tmp, 2);
			if (!ret) {
				pm_info->by_rtc_alarm.wkstat_idx = tmp[0];
				pm_info->by_rtc_alarm.wkstat_bit = tmp[1];
			}
		} while(0);

		ret = of_property_count_u32_elems(np, "eint_wkup_masks");
		if (!ret) {
			pr_err("%s %s: unabled to get eint_wakeup_masks value from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		} else {
			pm_info->num_eint_wkup_masks = ret;
			if (ret > 2) {
				pr_err("%s %s: num_eint_wkup_masks should be less than 3.\n",
						EXYNOS_PM_PREFIX, __func__);
				BUG();
			}
			pm_info->eint_wkup_masks = kzalloc(sizeof(unsigned int) * ret, GFP_KERNEL);
			of_property_read_u32_array(np, "eint_wkup_masks", pm_info->eint_wkup_masks, ret);
		}

		ret = of_property_count_u32_elems(np, "eint_pends");
		if (!ret) {
			pr_err("%s %s: unabled to get eint_pends value from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		} else {
			pm_info->num_eint_pends = ret;
			pm_info->eint_pends = kzalloc(sizeof(unsigned int) * ret, GFP_KERNEL);
			of_property_read_u32_array(np, "eint_pends", pm_info->eint_pends, ret);
		}

		ret = of_property_read_u32(np, "conn_req_offset", &pm_info->conn_req_offset);
		if (ret) {
			pr_err("%s %s: unabled to get conn_req_offset value from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			BUG();
		}
		pm_info->prev_conn_req = 0;
	} else {
		pr_err("%s %s: failed to have populated device tree\n",
					EXYNOS_PM_PREFIX, __func__);
		BUG();
	}

	suspend_set_ops(&exynos_pm_ops);
	register_syscore_ops(&exynos_pm_syscore_ops);
#ifdef CONFIG_DEBUG_FS
	exynos_pm_debugfs_init();
#endif

#if defined(CONFIG_SEC_FACTORY)
	/* create sysfs group */
	ret = sysfs_create_file(power_kobj, &dev_attr_asv_info.attr);
	if (ret)
		pr_err("%s: failed to create exynos7885 asv attribute file\n", __func__);
#endif

	return 0;
}
arch_initcall(exynos_pm_drvinit);

#ifdef CONFIG_SEC_PM
static int of_property_read_string_array_position(struct device_node *np,
		const char *propname, const char **out_strs, int size, unsigned long pos)
{
	struct property *prop = of_find_property(np, propname, NULL);
	int bit, l = 0, count = 0;
	const char *p, *end;

	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	p = prop->value;
	end = p + prop->length;

	for_each_set_bit(bit, &pos, size) {
		l = strnlen(p, end - p) + 1;
		if (p + l > end)
			return -EILSEQ;

		*(out_strs + bit) = p;

		p += l;
		count++;
	}

	return count <= 0 ? -ENODATA : count;
}

static int __init exynos_pm_init_extra(void)
{
	struct device_node *np, *w_np;
	struct wakeup_stat *ws_data;
	const char *buf;
	int ret;

	if (!of_have_populated_dt())
		return 0;

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos-pm-extra");
	if (!np) {
		pr_err("%s %s: unabled to find compatible node (%s)\n",
				EXYNOS_PM_PREFIX, __func__, "samsung,exynos-pm-extra");
		return 0;
	}

	np = of_find_node_by_name(np, "wakeup_stats");
	if (!np) {
		pr_err("%s %s: unabled to get wkup_stats nodes from DT\n",
				EXYNOS_PM_PREFIX, __func__);
		return 0;
	}

	pm_info->num_wkup_stats = of_get_child_count(np);

	ws_data = kzalloc(sizeof(*ws_data) * pm_info->num_wkup_stats, GFP_KERNEL);
	if (ws_data == NULL) {
		pr_err("%s %s: could not allocate memory for wakup_stat_data\n",
				EXYNOS_PM_PREFIX, __func__);
		return -ENOMEM;
	}

	pm_info->ws_extra = ws_data;
	for_each_child_of_node(np, w_np) {
		ret = of_property_read_u32(w_np, "addr", &ws_data->addr);
		if (ret) {
			pr_err("%s %s: unabled to get wkup_stat addr value from DT\n",
					EXYNOS_PM_PREFIX, __func__);
			return 0;
		}
		strncpy(ws_data->name, w_np->name, WKUP_STAT_NAME_LENGTH - 1);

		ret = of_property_read_string(w_np, "bits_to_check", &buf);
		if (ret)
			pr_err("%s %s: unabled to get src_check_bit value from DT\n",
					EXYNOS_PM_PREFIX, __func__);
		else
			bitmap_parselist(buf, &ws_data->src_check_bit, WKUP_STAT_SIZE);

		ret = of_property_read_string_array_position(w_np, "wakeup_sources",
				ws_data->wkup_src, WKUP_STAT_SIZE, ws_data->src_check_bit);
		if (ret <= 0 || ret != hweight_long(ws_data->src_check_bit)) {
			pr_err("%s %s: bits_to_check set_bits / wkup_src_num (%lu/%d)\n",
					EXYNOS_PM_PREFIX, __func__,
					hweight_long(ws_data->src_check_bit), ret);

			ws_data->src_check_bit = 0;
		}

		ret = of_property_read_u32(w_np, "eint_wakeup_check_bit",
				&ws_data->eint_wkup_bit);
		if (ret)
			ws_data->eint_wkup_bit = -1;

		ws_data++;
	}
	pm_info->print_by_extra = true;

	return 0;
}
subsys_initcall(exynos_pm_init_extra);
#endif /* CONFIG_SEC_PM */
