#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/suspend.h>
#include <linux/notifier.h>
#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/smc.h>
#include <soc/samsung/exynos-pmu.h>
#include <soc/samsung/cal-if.h>
#include "pmu-gnss.h"
#include "gnss_prj.h"
#include <linux/mcu_ipc.h>

#ifdef USE_IOREMAP_NOPMU

#if defined(CONFIG_SOC_EXYNOS7870)
#define PMU_ADDR		(0x10480000)
#define PMU_SIZE		(SZ_64K)
#elif defined(CONFIG_SOC_EXYNOS7880)
#define PMU_ADDR		(0x106B0000)
#define PMU_SIZE		(SZ_64K)
#elif defined(CONFIG_SOC_EXYNOS7570)
#define PMU_ADDR		(0x11C80000)
#define PMU_SIZE		(SZ_64K)
#elif defined(CONFIG_SOC_EXYNOS7872)
#define PMU_ADDR		(0x11C80000)
#define PMU_SIZE		(SZ_64K)
#endif

static void __iomem *pmu_reg;

static int gnss_nopmu_read(unsigned int reg_offset, unsigned int *ret)
{
	*ret = __raw_readl(pmu_reg + reg_offset);
	return 0;
}

static int gnss_nopmu_write(unsigned int reg_offset, unsigned int val)
{
	unsigned int tmp, tmp2;
	tmp = __raw_readl(pmu_reg + reg_offset);
	__raw_writel(val, pmu_reg + reg_offset);
	tmp2 = __raw_readl(pmu_reg + reg_offset);

	return (tmp == tmp2) ? 0 : -EINVAL;
}

static int gnss_nopmu_update(unsigned int reg_offset, unsigned int mask,
			unsigned int val)
{
	unsigned int memcfg_val;
	unsigned int tmp, tmp2;
	tmp = __raw_readl(pmu_reg + reg_offset);
	memcfg_val = tmp;
	memcfg_val &= ~mask;
	memcfg_val |= val;
	__raw_writel(memcfg_val, pmu_reg + reg_offset);
	tmp2 = __raw_readl(pmu_reg + reg_offset);

	return (memcfg_val == tmp2) ? 0 : -EINVAL;
}

#define gnss_pmu_read	gnss_nopmu_read
#define gnss_pmu_write	gnss_nopmu_write
#define gnss_pmu_update	gnss_nopmu_update

#else

#define gnss_pmu_read	exynos_pmu_read
#define gnss_pmu_write	exynos_pmu_write
#define gnss_pmu_update exynos_pmu_update

#endif /* USE_IOREMAP_NOPMU */

static void __set_shdmem_size(u32 reg_offset, u32 memsz)
{
	memsz /= MEMSIZE_RES;
	gnss_pmu_update(reg_offset, MEMSIZE_MASK, memsz << MEMSIZE_OFFSET);
}

static void set_shdmem_size(u32 memsz)
{
	gif_info("Set shared mem size: %dB\n", memsz);

	__set_shdmem_size(EXYNOS_PMU_GNSS2AP_MEM_CONFIG0, memsz);
}

static void __set_shdmem_base(u32 reg_offset, u32 shmem_base)
{
	u32 base_addr;
	base_addr = (shmem_base >> MEMBASE_ADDR_SHIFT);

	gnss_pmu_update(reg_offset, MEMBASE_ADDR_MASK << MEMBASE_ADDR_OFFSET,
			base_addr << MEMBASE_ADDR_OFFSET);
}

static void set_shdmem_base(u32 shmem_base)
{
	gif_info("Set shared mem baseaddr : 0x%x\n", shmem_base);

	__set_shdmem_base(EXYNOS_PMU_GNSS2AP_MEM_CONFIG1, shmem_base);
}

static void exynos_sys_powerdown_conf_gnss(void)
{
	gnss_pmu_write(EXYNOS_PMU_RESET_AHEAD_GNSS_SYS_PWR_REG, 0);
	gnss_pmu_write(EXYNOS_PMU_CLEANY_BUS_SYS_PWR_REG, 0);
	gnss_pmu_write(EXYNOS_PMU_LOGIC_RESET_GNSS_SYS_PWR_REG, 0);
	gnss_pmu_write(EXYNOS_PMU_TCXO_GATE_GNSS_SYS_PWR_REG, 0);
	gnss_pmu_write(EXYNOS_PMU_GNSS_DISABLE_ISO_SYS_PWR_REG, 1);
	gnss_pmu_write(EXYNOS_PMU_GNSS_RESET_ISO_SYS_PWR_REG, 0);
	gnss_pmu_write(EXYNOS_PMU_CENTRAL_SEQ_GNSS_CONFIGURATION, 0);
}

#ifdef CONFIG_GNSS_PMUCAL
static int gnss_pmu_clear_interrupt(enum gnss_int_clear gnss_int)
{
	int ret = 0;

	gif_debug("gnss_int = %d\n", gnss_int);
	switch (gnss_int) {
	case GNSS_INT_WAKEUP_CLEAR:
		ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_NS,
				GNSS_WAKEUP_REQ_CLR, GNSS_WAKEUP_REQ_CLR);
		break;
	case GNSS_INT_ACTIVE_CLEAR:
		cal_gnss_active_clear();
		break;
	case GNSS_INT_WDT_RESET_CLEAR:
		cal_gnss_reset_req_clear();
		break;
	default:
		gif_err("Unexpected interrupt value!\n");
		return -EIO;
	}

	if (ret < 0) {
		gif_err("ERR! GNSS Reset Fail: %d\n", ret);
		return -EIO;
	}

	return ret;
}

static int gnss_pmu_release_reset(void)
{
	exynos_pmu_shared_reg_enable();
	cal_gnss_reset_release();
	exynos_pmu_shared_reg_disable();
}

static int gnss_pmu_hold_reset(void)
{
	cal_gnss_reset_assert();
}

static int gnss_pmu_power_on(enum gnss_mode mode)
{

	u32 gnss_ctrl = 0;

	gif_err("mode[%d]\n", mode);
	gnss_pmu_read(EXYNOS_PMU_GNSS_CTRL_NS, &gnss_ctrl);

	if (mode == GNSS_POWER_ON && !(gnss_ctrl & GNSS_PWRON)) {
		cal_gnss_init();
	} else {
		gif_err("Something is strange. mode[%d]\n", mode);
		gif_err("PMU_GNSS_CTRL_S[0x%08x]\n", gnss_ctrl);
	}

	return 0;
}
#else
static int gnss_pmu_clear_interrupt(enum gnss_int_clear gnss_int)
{
	int ret = 0;

	gif_debug("gnss_int = %d\n", gnss_int);
	switch (gnss_int) {
	case GNSS_INT_WAKEUP_CLEAR:
		ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_NS,
				GNSS_WAKEUP_REQ_CLR, GNSS_WAKEUP_REQ_CLR);
		break;
	case GNSS_INT_ACTIVE_CLEAR:
		ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_NS,
				GNSS_ACTIVE_REQ_CLR, GNSS_ACTIVE_REQ_CLR);
		break;
	case GNSS_INT_WDT_RESET_CLEAR:
		ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_NS,
				GNSS_RESET_REQ_CLR, GNSS_RESET_REQ_CLR);
		break;
	default:
		gif_err("Unexpected interrupt value!\n");
		return -EIO;
	}

	if (ret < 0) {
		gif_err("ERR! GNSS Reset Fail: %d\n", ret);
		return -EIO;
	}

	return ret;
}

static int gnss_pmu_release_reset(void)
{
	u32 gnss_ctrl = 0;
	int ret = 0;

	exynos_pmu_shared_reg_enable();
	gnss_pmu_read(EXYNOS_PMU_GNSS_CTRL_NS, &gnss_ctrl);
	if (!(gnss_ctrl & GNSS_PWRON)) {
		ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_NS, GNSS_PWRON,
				GNSS_PWRON);
		if (ret < 0) {
			gif_err("ERR! write Fail: %d\n", ret);
			ret = -EIO;
		}
	}
	ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_S, GNSS_START, GNSS_START);
	if (ret < 0) {
		gif_err("ERR! GNSS Release Fail: %d\n", ret);
	} else {
		gnss_pmu_read(EXYNOS_PMU_GNSS_CTRL_S, &gnss_ctrl);
		gif_info("PMU_GNSS_CTRL_S[0x%08x]\n", gnss_ctrl);
		ret = -EIO;
	}
	exynos_pmu_shared_reg_disable();
	return ret;
}

static int gnss_pmu_hold_reset(void)
{
	int ret = 0;

	/* set sys_pwr_cfg registers */
	exynos_sys_powerdown_conf_gnss();

	ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_NS, GNSS_RESET_SET,
			GNSS_RESET_SET);
	if (ret < 0) {
		gif_err("ERR! GNSS Reset Fail: %d\n", ret);
		return -1;
	}

	/* some delay */
	cpu_relax();
	usleep_range(80, 100);

	return ret;
}

static int gnss_pmu_power_on(enum gnss_mode mode)
{
	u32 gnss_ctrl;
	int ret = 0;

	gif_err("mode[%d]\n", mode);

	gnss_pmu_read(EXYNOS_PMU_GNSS_CTRL_NS, &gnss_ctrl);
	if (mode == GNSS_POWER_ON) {
		if (!(gnss_ctrl & GNSS_PWRON)) {
			ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_NS,
					GNSS_PWRON, GNSS_PWRON);
			if (ret < 0)
				gif_err("ERR! write Fail: %d\n", ret);
		}

		ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_S, GNSS_START,
				GNSS_START);
		if (ret < 0)
			gif_err("ERR! write Fail: %d\n", ret);
	} else {
		ret = gnss_pmu_update(EXYNOS_PMU_GNSS_CTRL_NS, GNSS_PWRON, 0);
		if (ret < 0) {
			gif_err("ERR! write Fail: %d\n", ret);
			return ret;
		}
		/* set sys_pwr_cfg registers */
		exynos_sys_powerdown_conf_gnss();
	}

	return ret;
}
#endif

static int gnss_pmu_init_conf(struct gnss_ctl *gc)
{
	u32 shmem_size, shmem_base;

#ifdef USE_IOREMAP_NOPMU
	pmu_reg = devm_ioremap(gc->dev, PMU_ADDR, PMU_SIZE);
	if (pmu_reg == NULL)
		gif_err("%s: pmu ioremap failed.\n", gc->gnss_data->name);
	else
		gif_err("pmu_reg : 0x%p\n", pmu_reg);
#endif

	shmem_size = gc->gnss_data->shmem_size;
	shmem_base = gc->gnss_data->shmem_base;

	set_shdmem_size(shmem_size);
	set_shdmem_base(shmem_base);

	/* set access window for GNSS */
	gnss_pmu_write(EXYNOS_PMU_GNSS2AP_MIF_ACCESS_WIN0, 0x0);
	gnss_pmu_write(EXYNOS_PMU_GNSS2AP_PERI_ACCESS_WIN0, 0x0);
	gnss_pmu_write(EXYNOS_PMU_GNSS2AP_PERI_ACCESS_WIN1, 0x0);

	return 0;
}

static struct gnssctl_pmu_ops pmu_ops = {
	.init_conf = gnss_pmu_init_conf,
	.hold_reset = gnss_pmu_hold_reset,
	.release_reset = gnss_pmu_release_reset,
	.power_on = gnss_pmu_power_on,
	.clear_int = gnss_pmu_clear_interrupt,
};

void gnss_get_pmu_ops(struct gnss_ctl *gc)
{
	gc->pmu_ops = &pmu_ops;
	return;
}
