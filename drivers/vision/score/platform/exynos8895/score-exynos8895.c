#include <linux/io.h>
#include "score-config.h"
#include "score-exynos.h"
#include "score-exynos8895.h"

#define CLK_INDEX(name) name
#define REGISTER_CLK(name) {name, NULL}

extern int score_clk_set_rate(struct device *dev, u32 index, ulong frequency);
extern ulong score_clk_get_rate(struct device *dev, u32 index);
extern int  score_clk_enable(struct device *dev, u32 index);
extern int score_clk_disable(struct device *dev, u32 index);

#ifdef DISABLE_CMU
struct score_clk score_clk_array[] = {};
#else
enum score_clk_index {
	CLK_INDEX(score)
};

struct score_clk score_clk_array[] = {
	REGISTER_CLK("dsp")
};
#endif

const u32 score_clk_array_size = ARRAY_SIZE(score_clk_array);

int score_exynos_clk_cfg(struct score_exynos *exynos)
{
	SCORE_P_TP();
	return 0;
}

int score_exynos_clk_on(struct score_exynos *exynos)
{
	SCORE_P_TP();
	score_clk_enable(exynos->dev, 0);
	return 0;
}

int score_exynos_clk_off(struct score_exynos *exynos)
{
	SCORE_P_TP();
	score_clk_disable(exynos->dev, 0);
	return 0;
}

int score_exynos_clk_print(struct score_exynos *exynos)
{
	SCORE_P_TP();
	return 0;
}

const struct score_clk_ops score_clk_ops = {
	.clk_cfg	= score_exynos_clk_cfg,
	.clk_on		= score_exynos_clk_on,
	.clk_off	= score_exynos_clk_off,
	.clk_print	= score_exynos_clk_print
};

int score_exynos_ctl_reset(struct score_exynos *exynos)
{
	SCORE_P_TP();
	BUG_ON(!exynos);

#ifdef CONFIG_EXYNOS_SCORE_HARDWARE
	writel(0, exynos->regs + 0x18);
#endif
	return 0;
}

const struct score_ctl_ops score_ctl_ops = {
	.ctl_reset	= score_exynos_ctl_reset,
};
