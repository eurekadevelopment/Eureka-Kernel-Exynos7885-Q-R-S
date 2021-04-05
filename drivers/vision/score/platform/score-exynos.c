#include <linux/io.h>
#include <linux/pinctrl/consumer.h>

#include "score-config.h"
#include "score-exynos.h"

extern struct score_clk score_clk_array[];
extern const u32 score_clk_array_size;
extern const struct score_clk_ops score_clk_ops;
extern const struct score_ctl_ops score_ctl_ops;

int score_clk_set_rate(struct device *dev, u32 index, ulong frequency)
{
	int ret = 0;
	struct clk *clk;

	SCORE_P_TP();
	if (index >= score_clk_array_size) {
		score_err("index is invalid(%d >= %d)\n", index, score_clk_array_size);
		ret = -EINVAL;
		goto p_err;
	}

	clk= score_clk_array[index].clk;
	if (IS_ERR_OR_NULL(clk)) {
		score_err("clk is NULL(%d)\n", index);
		ret = -EINVAL;
		goto p_err;
	}

	ret = clk_set_rate(clk, frequency);
	if (ret) {
		score_err("clk_set_rate is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

ulong score_clk_get_rate(struct device *dev, u32 index)
{
	ulong frequency;
	struct clk *clk;

	SCORE_P_TP();
	if (index >= score_clk_array_size) {
		score_err("index is invalid(%d >= %d)\n", index, score_clk_array_size);
		frequency = -EINVAL;
		goto p_err;
	}

	clk = score_clk_array[index].clk;
	if (IS_ERR_OR_NULL(clk)) {
		score_err("clk is NULL(%d)\n", index);
		frequency = -EINVAL;
		goto p_err;
	}

	frequency = clk_get_rate(clk);

p_err:
	score_info("%s : %ldMhz \n", score_clk_array[index].name, frequency/1000000);
	return frequency;
}

int  score_clk_enable(struct device *dev, u32 index)
{
	int ret = 0;
	struct clk *clk;

	SCORE_P_TP();
	if (index >= score_clk_array_size) {
		score_err("index is invalid(%d >= %d)\n", index, score_clk_array_size);
		ret = -EINVAL;
		goto p_err;
	}

	clk = score_clk_array[index].clk;
	if (IS_ERR_OR_NULL(clk)) {
		score_err("clk is NULL(%d)\n", index);
		ret = -EINVAL;
		goto p_err;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		score_err("clk_prepare_enable is fail(%s)\n", score_clk_array[index].name);
		goto p_err;
	}

p_err:
	return ret;
}

int score_clk_disable(struct device *dev, u32 index)
{
	int ret = 0;
	struct clk *clk;

	SCORE_P_TP();
	if (index >= score_clk_array_size) {
		score_err("index is invalid(%d >= %d)\n", index, score_clk_array_size);
		ret = -EINVAL;
		goto p_err;
	}

	clk = score_clk_array[index].clk;
	if (IS_ERR_OR_NULL(clk)) {
		score_err("clk is NULL(%d)\n", index);
		ret = -EINVAL;
		goto p_err;
	}

	clk_disable_unprepare(clk);

p_err:
	return ret;
}

static int score_exynos_clk_init(struct device *dev)
{
	int ret = 0;
	const char *name;
	struct clk *clk;
	u32 index;

	SCORE_P_TP();
	for (index = 0; index < score_clk_array_size; ++index) {
		name = score_clk_array[index].name;
		if (!name) {
			probe_err("name is NULL\n");
			ret = -EINVAL;
			break;
		}

		clk = clk_get(dev, name);
		if (IS_ERR_OR_NULL(clk)) {
			probe_err("%s clk is not found\n", name);
			ret = -EINVAL;
			break;
		}

		score_clk_array[index].clk = clk;
	}

	return ret;
}

int score_exynos_probe(struct score_exynos *exynos, struct device *dev, void *regs)
{
	int ret = 0;

	SCORE_P_TP();
	BUG_ON(!exynos);
	BUG_ON(!dev);

	exynos->regs = regs;
	exynos->clk_ops = &score_clk_ops;
	exynos->ctl_ops = &score_ctl_ops;
	exynos->pinctrl = devm_pinctrl_get(dev);

	/*
	* if (IS_ERR_OR_NULL(exynos->pinctrl)) {
	*	probe_err("devm_pinctrl_get is fail");
	*	ret = PTR_ERR(exynos->pinctrl);
	*	goto p_err;
	* }
	*/

	SCORE_P_TP();
	ret = score_exynos_clk_init(dev);
	if (ret) {
		probe_err("score_exynos_clk_init is fail(%d)", ret);
		goto p_err;
	}

p_err:
	return ret;
}

void score_readl(void __iomem *base_addr, struct score_reg *reg, u32 *val)
{
	*val = readl(base_addr + reg->offset);

#ifdef DBG_HW_SFR
	score_info("[REG][%s][0x%04X], val(R):[0x%08X]\n", reg->name, reg->offset, *val);
#endif
}

void score_writel(void __iomem *base_addr, struct score_reg *reg, u32 val)
{
#ifdef DBG_HW_SFR
	score_info("[REG][%s][0x%04X], val(W):[0x%08X]\n", reg->name, reg->offset, val);
#endif

	writel(val, base_addr + reg->offset);
}

void score_readf(void __iomem *base_addr, struct score_reg *reg, struct score_field *field, u32 *val)
{
	*val = (readl(base_addr + reg->offset) >> (field->bit_start)) & ((1 << (field->bit_width)) - 1);

#ifdef DBG_HW_SFR
	score_info("[REG][%s][%s][0x%04X], val(R):[0x%08X]\n", reg->name, field->name, reg->offset, *val);
#endif
}

void score_writef(void __iomem *base_addr, struct score_reg *reg, struct score_field *field, u32 val)
{
	u32 mask, temp;

	mask = ((1 << field->bit_width) - 1);
	temp = readl(base_addr + reg->offset) & ~(mask << field->bit_start);
	temp |= (val & mask) << (field->bit_start);

#ifdef DBG_HW_SFR
	score_info("[REG][%s][%s][0x%04X], val(W):[0x%08X]\n", reg->name, field->name, reg->offset, val);
#endif

	writel(temp, base_addr + reg->offset);
}
