#ifndef __ACPM_HELPERS_H__
#define __ACPM_HELPERS_H__

#ifdef CONFIG_ACPM_DVFS
extern int exynos_acpm_set_flag(void);
#else
static inline int exynos_acpm_set_flag(void)
{
	return 0;
}
#endif

#endif
