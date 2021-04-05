/*
 * s2mpu08.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/bug.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/mfd/samsung/s2mpu08.h>
#include <linux/mfd/samsung/s2mpu08-private.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/exynos-ss.h>
#include <linux/debugfs.h>
#ifdef CONFIG_SEC_PM
#include <linux/sec_sysfs.h>

#define STATUS1_ACOK	BIT(2)

static struct device *ap_pmic_dev;
#endif

static struct s2mpu08_info *static_info;
static struct regulator_desc regulators[S2MPU08_REGULATOR_MAX];

#ifdef CONFIG_DEBUG_FS
static u8 i2caddr = 0;
static u8 i2cdata = 0;
static struct i2c_client *dbgi2c = NULL;
static struct dentry *s2mpu08_root = NULL;
static struct dentry *s2mpu08_i2caddr = NULL;
static struct dentry *s2mpu08_i2cdata = NULL;
#endif

struct s2mpu08_info {
	struct regulator_dev *rdev[S2MPU08_REGULATOR_MAX];
	unsigned int opmode[S2MPU08_REGULATOR_MAX];
	int num_regulators;
	struct s2mpu08_dev *iodev;
	struct mutex lock;
	struct i2c_client *i2c;
};

static unsigned int s2mpu08_of_map_mode(unsigned int val) {
	switch (val) {
	case SEC_OPMODE_SUSPEND:	/* ON in Standby Mode */
		return 0x1;
	case SEC_OPMODE_MIF:		/* ON in PWREN_MIF mode */
		return 0x2;
	case SEC_OPMODE_ON:		/* ON in Normal Mode */
		return 0x3;
	default:
		return 0x3;
	}
}

/* Some LDOs supports [LPM/Normal]ON mode during suspend state */
static int s2m_set_mode(struct regulator_dev *rdev,
				     unsigned int mode)
{
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret, id = rdev_get_id(rdev);

	val = mode << S2MPU08_ENABLE_SHIFT;

	ret = s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->enable_reg,
				  val, rdev->desc->enable_mask);
	if (ret)
		return ret;

	s2mpu08->opmode[id] = val;
	return 0;
}

static int s2m_enable(struct regulator_dev *rdev)
{
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	u8 val;

	/* if rev_id is not 0 w/a is unnecessary */
	if (s2mpu08->iodev->pmic_rev == 0x0 &&
			(reg_id == S2MPU08_LDO35 || reg_id == S2MPU08_LDO36))
				goto ldo35_36_workaround;

	/* disregard BUCK5 enable */
	if (reg_id == S2MPU08_BUCK5)
		return 0;

	return s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->enable_reg,
				  s2mpu08->opmode[rdev_get_id(rdev)],
				  rdev->desc->enable_mask);

ldo35_36_workaround:

	if (reg_id == S2MPU08_LDO35) {
		s2mpu08_read_reg(s2mpu08->i2c, S2MPU08_PMIC_REG_L36CTRL, &val);
		pr_info("%s : LDO36_CTRL(0x50[7:6]) : [0x%x]\n", __func__, (val & 0xC0));

		/* if LDO36 is on */
		if ((val & 0xC0) != 0x00) {
			s2mpu08_update_reg(s2mpu08->i2c, 0x75, 0x00, 0xF0);
			s2mpu08_update_reg(s2mpu08->i2c, 0xFF, 0x00, 0x08);
			pr_info("%s : LDO36 on / LDO35 on\n", __func__);
			return 0;
		}

		else {
			s2mpu08_update_reg(s2mpu08->i2c, 0x75, 0xF0, 0xF0);
			s2mpu08_update_reg(s2mpu08->i2c, 0xFF, 0x08, 0x08);
			pr_info("%s : LDO36 off / LDO35 on\n", __func__);
			return 0;
		}
	}

	/* reg_id == S2MPU08_LDO36 */
	else {
		s2mpu08_read_reg(s2mpu08->i2c, 0xFF, &val);
		pr_info("%s : EXT_CTRL(0xFF[3]) : [0x%x]\n", __func__, (val & 0x08));

		/* if LDO35 is on */
		if ((val & 0x08) == 0x08) {
			s2mpu08_update_reg(s2mpu08->i2c, 0x75, 0x00, 0xF0);
			s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->enable_reg,
				0xC0, rdev->desc->enable_mask);
			/* LDO35 should be off when LDO36 is on */
			s2mpu08_update_reg(s2mpu08->i2c, 0xFF, 0x00, 0x08);
			pr_info("%s : LDO35 on / LDO36 on\n", __func__);
			return 0;
		}

		else {
			s2mpu08_update_reg(s2mpu08->i2c, 0x75, 0xF0, 0xF0);
			s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->enable_reg,
				0xC0, rdev->desc->enable_mask);
			pr_info("%s : LDO35 off / LDO36 on\n", __func__);
			return 0;
		}
	}
}

static int s2m_disable_regmap(struct regulator_dev *rdev)
{
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	u8 val;

	/* if rev_id is not 0 w/a is unnecessary */
	if (s2mpu08->iodev->pmic_rev == 0x0 &&
			(reg_id == S2MPU08_LDO35 || reg_id == S2MPU08_LDO36))
				goto ldo35_36_workaround;

	/* disregard BUCK5 disable */
	if (reg_id == S2MPU08_BUCK5)
		return 0;

	if (rdev->desc->enable_is_inverted)
		val = rdev->desc->enable_mask;
	else
		val = 0;

	return s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->enable_reg,
				  val, rdev->desc->enable_mask);

ldo35_36_workaround:

	if (reg_id == S2MPU08_LDO35) {
		s2mpu08_read_reg(s2mpu08->i2c, S2MPU08_PMIC_REG_L36CTRL, &val);
		pr_info("%s : LDO36_CTRL(0x50[7:6]) : [0x%x]\n", __func__, (val & 0xC0));

		/* if LDO36 is on */
		if ((val & 0xC0) != 0x00) {
			s2mpu08_update_reg(s2mpu08->i2c, 0x75, 0xF0, 0xF0);
			s2mpu08_update_reg(s2mpu08->i2c, 0xFF, 0x00, 0x08);
			pr_info("%s : LDO36 on / LDO35 off\n", __func__);
			return 0;
		}

		else {
			s2mpu08_update_reg(s2mpu08->i2c, 0x75, 0x00, 0xF0);
			s2mpu08_update_reg(s2mpu08->i2c, 0xFF, 0x00, 0x08);
			pr_info("%s : LDO36 off / LDO35 is off\n", __func__);
			return 0;
		}
	}

	/* reg_id == S2MPU08_LDO36 */
	else {
		s2mpu08_read_reg(s2mpu08->i2c, 0x75, &val);
		pr_info("%s : SEQ44(0x75[7:4]) : [0x%x]\n", __func__, (val & 0xF0));

		/* if LDO35 is on */
		if ((val & 0xF0) == 0x00) {
			s2mpu08_update_reg(s2mpu08->i2c, 0xFF, 0x08, 0x08);
			s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->enable_reg,
				0, rdev->desc->enable_mask);
			s2mpu08_update_reg(s2mpu08->i2c, 0x75, 0xF0, 0xF0);
			pr_info("%s : LDO35 on / LDO36 off\n", __func__);
			return 0;
		}

		else {
			s2mpu08_update_reg(s2mpu08->i2c, 0xFF, 0x00, 0x08);
			s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->enable_reg,
				0x00, rdev->desc->enable_mask);
			pr_info("%s : LDO35 off / LDO36 off\n", __func__);
			return 0;
		}
	}
}

static int s2m_is_enabled_regmap(struct regulator_dev *rdev)
{
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int ret, reg_id = rdev_get_id(rdev);
	u8 val, val2, val3;

	/* if rev_id is not 0 w/a is unnecessary */
	if (s2mpu08->iodev->pmic_rev == 0x0 &&
			reg_id == S2MPU08_LDO35)
				goto ldo35_36_workaround;

	if (reg_id == S2MPU08_BUCK5)
			return 0;

	else {
		ret = s2mpu08_read_reg(s2mpu08->i2c,
				rdev->desc->enable_reg, &val);
		if (ret)
			return ret;
	}

	if (rdev->desc->enable_is_inverted)
		return (val & rdev->desc->enable_mask) == 0;
	else
		return (val & rdev->desc->enable_mask) != 0;

ldo35_36_workaround:

	s2mpu08_read_reg(s2mpu08->i2c, 0xFF, &val);

	if ((val & 0x08) == 0x08)
		return 1;

	else {
		s2mpu08_read_reg(s2mpu08->i2c, S2MPU08_PMIC_REG_L36CTRL, &val2);
		s2mpu08_read_reg(s2mpu08->i2c, 0x75, &val3);

		if (((val2 & 0xC0) != 0x00) && ((val3 & 0xF0) == 0x00))
			return 1;
		else
			return 0;
	}
}

static int get_ramp_delay(int ramp_delay)
{
	unsigned char cnt = 0;

	ramp_delay /= 6;

	while (true) {
		ramp_delay = ramp_delay >> 1;
		if (ramp_delay == 0)
			break;
		cnt++;
	}
	return cnt;
}

static int s2m_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int ramp_shift, reg_id = rdev_get_id(rdev);
	int ramp_mask = 0x03;
	unsigned int ramp_value = 0;

	ramp_value = get_ramp_delay(ramp_delay/1000);
	if (ramp_value > 4) {
		pr_warn("%s: ramp_delay: %d not supported\n",
			rdev->desc->name, ramp_delay);
	}

	switch (reg_id) {
	case S2MPU08_BUCK1:
	case S2MPU08_BUCK6:
	case S2MPU08_BUCK7:
		ramp_shift = 6;
		break;
	case S2MPU08_BUCK2:
	case S2MPU08_BUCK3:
		ramp_shift = 4;
		break;
	case S2MPU08_BUCK4:
	case S2MPU08_BUCK5:
		ramp_shift = 2;
		break;
	case S2MPU08_BUCK8:
		ramp_shift = 0;
		break;
	default:
		return -EINVAL;
	}

	return s2mpu08_update_reg(s2mpu08->i2c, S2MPU08_PMIC_REG_BUCKRAMP,
		ramp_value << ramp_shift, ramp_mask << ramp_shift);
}

static int s2m_get_voltage_sel_regmap(struct regulator_dev *rdev)
{
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int ret;
	u8 val;

	ret = s2mpu08_read_reg(s2mpu08->i2c, rdev->desc->vsel_reg, &val);
	if (ret)
		return ret;

	val &= rdev->desc->vsel_mask;

	return val;
}

static int s2m_set_voltage_sel_regmap(struct regulator_dev *rdev, unsigned sel)
{
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);
	int reg_id = rdev_get_id(rdev);
	int ret;
	char name[16];
	unsigned int voltage;

	/* voltage information logging to snapshot feature */
	snprintf(name, sizeof(name), "LDO%d", (reg_id - S2MPU08_LDO1) + 1);
	voltage = ((sel & rdev->desc->vsel_mask) * S2MPU08_LDO_STEP2) + S2MPU08_LDO_MIN1;
	ret = s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->vsel_reg,
				  sel, rdev->desc->vsel_mask);
	if (ret < 0)
		goto out;

	if (rdev->desc->apply_bit)
		ret = s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->apply_reg,
					 rdev->desc->apply_bit,
					 rdev->desc->apply_bit);
	return ret;
out:
	pr_warn("%s: failed to set voltage_sel_regmap\n", rdev->desc->name);
	return ret;
}

static int s2m_set_voltage_sel_regmap_buck(struct regulator_dev *rdev,
								unsigned sel)
{
	int ret;
	struct s2mpu08_info *s2mpu08 = rdev_get_drvdata(rdev);

	ret = s2mpu08_write_reg(s2mpu08->i2c, rdev->desc->vsel_reg, sel);
	if (ret < 0)
		goto i2c_out;

	if (rdev->desc->apply_bit)
		ret = s2mpu08_update_reg(s2mpu08->i2c, rdev->desc->apply_reg,
					 rdev->desc->apply_bit,
					 rdev->desc->apply_bit);

	return ret;

i2c_out:
	pr_warn("%s: failed to set voltage_sel_regmap\n", rdev->desc->name);
	return ret;
}

static int s2m_set_voltage_time_sel(struct regulator_dev *rdev,
				   unsigned int old_selector,
				   unsigned int new_selector)
{
	unsigned int ramp_delay = 0;
	int old_volt, new_volt;

	if (rdev->constraints->ramp_delay)
		ramp_delay = rdev->constraints->ramp_delay;
	else if (rdev->desc->ramp_delay)
		ramp_delay = rdev->desc->ramp_delay;

	if (ramp_delay == 0) {
		pr_warn("%s: ramp_delay not set\n", rdev->desc->name);
		return -EINVAL;
	}

	/* sanity check */
	if (!rdev->desc->ops->list_voltage)
		return -EINVAL;

	old_volt = rdev->desc->ops->list_voltage(rdev, old_selector);
	new_volt = rdev->desc->ops->list_voltage(rdev, new_selector);

	if (old_selector < new_selector)
		return DIV_ROUND_UP(new_volt - old_volt, ramp_delay);

	return 0;
}

static struct regulator_ops s2mpu08_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2m_is_enabled_regmap,
	.enable			= s2m_enable,
	.disable		= s2m_disable_regmap,
	.get_voltage_sel	= s2m_get_voltage_sel_regmap,
	.set_voltage_sel	= s2m_set_voltage_sel_regmap,
	.set_voltage_time_sel	= s2m_set_voltage_time_sel,
	.set_mode		= s2m_set_mode,
};

static struct regulator_ops s2mpu08_buck_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= s2m_is_enabled_regmap,
	.enable			= s2m_enable,
	.disable		= s2m_disable_regmap,
	.get_voltage_sel	= s2m_get_voltage_sel_regmap,
	.set_voltage_sel	= s2m_set_voltage_sel_regmap_buck,
	.set_voltage_time_sel	= s2m_set_voltage_time_sel,
	.set_mode		= s2m_set_mode,
	.set_ramp_delay		= s2m_set_ramp_delay,
};

#define _BUCK(macro)	S2MPU08_BUCK##macro
#define _buck_ops(num)	s2mpu08_buck_ops##num

#define _LDO(macro)	S2MPU08_LDO##macro
#define _REG(ctrl)	S2MPU08_PMIC_REG##ctrl
#define _ldo_ops(num)	s2mpu08_ldo_ops##num
#define _TIME(macro)	S2MPU08_ENABLE_TIME##macro

#define BUCK_DESC(_name, _id, _ops, m, s, v, e, t)	{	\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2MPU08_BUCK_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2MPU08_BUCK_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= S2MPU08_ENABLE_MASK,			\
	.enable_time	= t,					\
	.of_map_mode	= s2mpu08_of_map_mode			\
}

#define LDO_DESC(_name, _id, _ops, m, s, v, e, t)	{	\
	.name		= _name,				\
	.id		= _id,					\
	.ops		= _ops,					\
	.type		= REGULATOR_VOLTAGE,			\
	.owner		= THIS_MODULE,				\
	.min_uV		= m,					\
	.uV_step	= s,					\
	.n_voltages	= S2MPU08_LDO_N_VOLTAGES,		\
	.vsel_reg	= v,					\
	.vsel_mask	= S2MPU08_LDO_VSEL_MASK,		\
	.enable_reg	= e,					\
	.enable_mask	= S2MPU08_ENABLE_MASK,			\
	.enable_time	= t,					\
	.of_map_mode	= s2mpu08_of_map_mode			\
}

static struct regulator_desc regulators[S2MPU08_REGULATOR_MAX] = {
	/* name, id, ops, min_uv, uV_step, vsel_reg, enable_reg */
	LDO_DESC("LDO1", _LDO(1), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L1CTRL), _REG(_L1CTRL), _TIME(_LDO)),
	LDO_DESC("LDO2", _LDO(2), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L2CTRL1), _REG(_L2CTRL1), _TIME(_LDO)),
	LDO_DESC("LDO3", _LDO(3), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L3CTRL), _REG(_L3CTRL), _TIME(_LDO)),
	LDO_DESC("LDO4", _LDO(4), &_ldo_ops(), _LDO(_MIN2),
	_LDO(_STEP1), _REG(_L4CTRL), _REG(_L4CTRL), _TIME(_LDO)),
	LDO_DESC("LDO5", _LDO(5), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L5CTRL), _REG(_L5CTRL), _TIME(_LDO)),
	LDO_DESC("LDO6", _LDO(6), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L6CTRL), _REG(_L6CTRL), _TIME(_LDO)),
	LDO_DESC("LDO7", _LDO(7), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L7CTRL), _REG(_L7CTRL), _TIME(_LDO)),
	LDO_DESC("LDO8", _LDO(8), &_ldo_ops(), _LDO(_MIN4),
	_LDO(_STEP2), _REG(_L8CTRL), _REG(_L8CTRL), _TIME(_LDO)),
	LDO_DESC("LDO9", _LDO(9), &_ldo_ops(), _LDO(_MIN4),
	_LDO(_STEP2), _REG(_L9CTRL), _REG(_L9CTRL), _TIME(_LDO)),
	LDO_DESC("LDO10", _LDO(10), &_ldo_ops(), _LDO(_MIN4),
	_LDO(_STEP2), _REG(_L10CTRL), _REG(_L10CTRL), _TIME(_LDO)),
	LDO_DESC("LDO11", _LDO(11), &_ldo_ops(), _LDO(_MIN4),
	_LDO(_STEP2), _REG(_L11CTRL), _REG(_L11CTRL), _TIME(_LDO)),
	LDO_DESC("LDO12", _LDO(12), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L12CTRL), _REG(_L12CTRL), _TIME(_LDO)),
	LDO_DESC("LDO13", _LDO(13), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L13CTRL), _REG(_L13CTRL), _TIME(_LDO)),
	LDO_DESC("LDO14", _LDO(14), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L14CTRL), _REG(_L14CTRL), _TIME(_LDO)),
/*	LDO_DESC("LDO15", _LDO(15), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L15CTRL), _REG(_L15CTRL), _TIME(_LDO)),
	LDO_DESC("LDO16", _LDO(16), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L16CTRL), _REG(_L16CTRL), _TIME(_LDO)),
	LDO_DESC("LDO17", _LDO(17), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L17CTRL), _REG(_L17CTRL), _TIME(_LDO)),
	LDO_DESC("LDO18", _LDO(18), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L18CTRL), _REG(_L18CTRL), _TIME(_LDO)),
	LDO_DESC("LDO19", _LDO(19), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L19CTRL), _REG(_L19CTRL), _TIME(_LDO)),
	LDO_DESC("LDO20", _LDO(20), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L20CTRL), _REG(_L20CTRL), _TIME(_LDO)),
	LDO_DESC("LDO21", _LDO(21), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L21CTRL), _REG(_L21CTRL), _TIME(_LDO)),
	LDO_DESC("LDO22", _LDO(22), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L22CTRL), _REG(_L22CTRL), _TIME(_LDO)),
	LDO_DESC("LDO23", _LDO(23), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L23CTRL), _REG(_L23CTRL), _TIME(_LDO)),
	LDO_DESC("LDO24", _LDO(24), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L24CTRL), _REG(_L24CTRL), _TIME(_LDO)),
	LDO_DESC("LDO25", _LDO(25), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L25CTRL), _REG(_L25CTRL), _TIME(_LDO)),
	LDO_DESC("LDO26", _LDO(26), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L26CTRL), _REG(_L26CTRL), _TIME(_LDO)),
	LDO_DESC("LDO27", _LDO(27), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L27CTRL), _REG(_L27CTRL), _TIME(_LDO)),
	LDO_DESC("LDO28", _LDO(28), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L28CTRL), _REG(_L28CTRL), _TIME(_LDO)),
	LDO_DESC("LDO29", _LDO(29), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP1), _REG(_L29CTRL), _REG(_L29CTRL), _TIME(_LDO)),
	LDO_DESC("LDO30", _LDO(30), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L30CTRL), _REG(_L30CTRL), _TIME(_LDO)),
	LDO_DESC("LDO31", _LDO(31), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L31CTRL), _REG(_L31CTRL), _TIME(_LDO)),
	LDO_DESC("LDO32", _LDO(32), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L32CTRL), _REG(_L32CTRL), _TIME(_LDO)),
*/	LDO_DESC("LDO33", _LDO(33), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L33CTRL), _REG(_L33CTRL), _TIME(_LDO)),
	LDO_DESC("LDO34", _LDO(34), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L34CTRL), _REG(_L34CTRL), _TIME(_LDO)),
	LDO_DESC("LDO35", _LDO(35), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L35CTRL), _REG(_L35CTRL), _TIME(_LDO)),
	LDO_DESC("LDO36", _LDO(36), &_ldo_ops(), _LDO(_MIN1),
	_LDO(_STEP2), _REG(_L36CTRL), _REG(_L36CTRL), _TIME(_LDO)),
	LDO_DESC("LDO37", _LDO(37), &_ldo_ops(), _LDO(_MIN3),
	_LDO(_STEP2), _REG(_L37CTRL), _REG(_L37CTRL), _TIME(_LDO)),
	BUCK_DESC("BUCK1", _BUCK(1), &_buck_ops(), _BUCK(_MIN1),
	_BUCK(_STEP1), _REG(_B1CTRL2), _REG(_B1CTRL1), _TIME(_BUCK1)),
	BUCK_DESC("BUCK2", _BUCK(2), &_buck_ops(), _BUCK(_MIN1),
	_BUCK(_STEP1), _REG(_B2CTRL2), _REG(_B2CTRL1), _TIME(_BUCK2)),
	BUCK_DESC("BUCK3", _BUCK(3), &_buck_ops(), _BUCK(_MIN1),
	_BUCK(_STEP1), _REG(_B3CTRL2), _REG(_B3CTRL1), _TIME(_BUCK3)),
	BUCK_DESC("BUCK4", _BUCK(4), &_buck_ops(), _BUCK(_MIN1),
	_BUCK(_STEP1), _REG(_B4CTRL2), _REG(_B4CTRL1), _TIME(_BUCK4)),
	BUCK_DESC("BUCK5", _BUCK(5), &_buck_ops(), _BUCK(_MIN1),
	_BUCK(_STEP1), _REG(_B5CTRL2), _REG(_B5CTRL1), _TIME(_BUCK5)),
	BUCK_DESC("BUCK6", _BUCK(6), &_buck_ops(), _BUCK(_MIN1),
	_BUCK(_STEP1), _REG(_B6CTRL2), _REG(_B6CTRL1), _TIME(_BUCK6)),
	BUCK_DESC("BUCK7", _BUCK(7), &_buck_ops(), _BUCK(_MIN2),
	_BUCK(_STEP2), _REG(_B7CTRL2), _REG(_B7CTRL1), _TIME(_BUCK7)),
	BUCK_DESC("BUCK8", _BUCK(8), &_buck_ops(), _BUCK(_MIN2),
	_BUCK(_STEP2), _REG(_B8CTRL2), _REG(_B8CTRL1), _TIME(_BUCK8)),
/*	BUCK_DESC("BUCK9", _BUCK(9), &_buck_ops(), _BUCK(_MIN1),
	_BUCK(_STEP1), _REG(_B9CTRL2), _REG(_B9CTRL1), _TIME(_BUCK9)),*/
};
#ifdef CONFIG_OF
static int s2mpu08_pmic_dt_parse_pdata(struct s2mpu08_dev *iodev,
					struct s2mpu08_platform_data *pdata)
{
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct s2mpu08_regulator_data *rdata;
	unsigned int i;

	pmic_np = iodev->dev->of_node;
	if (!pmic_np) {
		dev_err(iodev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(iodev->dev, "could not find regulators sub-node\n");
		return -EINVAL;
	}

	/* count the number of regulators to be supported in pmic */
	pdata->num_regulators = 0;
	for_each_child_of_node(regulators_np, reg_np) {
		pdata->num_regulators++;
	}

	rdata = devm_kzalloc(iodev->dev, sizeof(*rdata) *
				pdata->num_regulators, GFP_KERNEL);
	if (!rdata) {
		dev_err(iodev->dev,
			"could not allocate memory for regulator data\n");
		return -ENOMEM;
	}

	pdata->regulators = rdata;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(regulators); i++)
			if (!of_node_cmp(reg_np->name,
					regulators[i].name))
				break;

		if (i == ARRAY_SIZE(regulators)) {
			dev_warn(iodev->dev,
			"don't know how to configure regulator %s\n",
			reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(
						iodev->dev, reg_np,
						&regulators[i]);
		rdata->reg_node = reg_np;
		rdata++;
	}

	return 0;
}
#else
static int s2mpu08_pmic_dt_parse_pdata(struct s2mpu08_pmic_dev *iodev,
					struct s2mpu08_platform_data *pdata)
{
	return 0;
}
#endif /* CONFIG_OF */

#ifdef CONFIG_DEBUG_FS
static ssize_t s2mpu08_i2caddr_read(struct file *file, char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[10];
	ssize_t ret;

	ret = snprintf(buf, sizeof(buf), "0x%x\n", i2caddr);
	if (ret < 0)
		return ret;

	return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static ssize_t s2mpu08_i2caddr_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[10];
	ssize_t len;
	u8 val;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';

	if (!kstrtou8(buf, 0, &val))
		i2caddr = val;

	return len;
}

static ssize_t s2mpu08_i2cdata_read(struct file *file, char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[10];
	ssize_t ret;

	ret = s2mpu08_read_reg(dbgi2c, i2caddr, &i2cdata);
	if (ret)
		return ret;

	ret = snprintf(buf, sizeof(buf), "0x%x\n", i2cdata);
	if (ret < 0)
		return ret;

	return simple_read_from_buffer(user_buf, count, ppos, buf, ret);
}

static ssize_t s2mpu08_i2cdata_write(struct file *file, const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[10];
	ssize_t len, ret;
	u8 val;

	len = simple_write_to_buffer(buf, sizeof(buf) - 1, ppos, user_buf, count);
	if (len < 0)
		return len;

	buf[len] = '\0';

	if (!kstrtou8(buf, 0, &val)) {
		ret = s2mpu08_write_reg(dbgi2c, i2caddr, val);
		if (ret < 0)
			return ret;
	}

	return len;
}

static const struct file_operations s2mpu08_i2caddr_fops = {
	.open = simple_open,
	.read = s2mpu08_i2caddr_read,
	.write = s2mpu08_i2caddr_write,
	.llseek = default_llseek,
};
static const struct file_operations s2mpu08_i2cdata_fops = {
	.open = simple_open,
	.read = s2mpu08_i2cdata_read,
	.write = s2mpu08_i2cdata_write,
	.llseek = default_llseek,
};
#endif

#ifdef CONFIG_SEC_PM
static ssize_t chg_det_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int ret, chg_det;
	u8 val;

	ret = s2mpu08_read_reg(static_info->i2c, S2MPU08_PMIC_REG_STATUS1, &val);

	if(ret)
		chg_det = -1;
	else
		chg_det = !!(val & STATUS1_ACOK); // ACOK active high

	pr_info("%s: ap pmic chg det: %d\n", __func__, chg_det);

	return sprintf(buf, "%d\n", chg_det);
}

static DEVICE_ATTR_RO(chg_det);

static struct attribute *ap_pmic_attributes[] = {
	&dev_attr_chg_det.attr,
	NULL
};

static const struct attribute_group ap_pmic_attr_group = {
	.attrs = ap_pmic_attributes,
};
#endif /* CONFIG_SEC_PM */

static int s2mpu08_pmic_probe(struct platform_device *pdev)
{
	struct s2mpu08_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct s2mpu08_platform_data *pdata = iodev->pdata;
	struct regulator_config config = { };
	struct s2mpu08_info *s2mpu08;
	u8 flag_wtp, lowr_wtp;
	int i, ret;

	pr_info("%s s2mpu08 pmic driver Loading start\n", __func__);

	if (iodev->dev->of_node) {
		ret = s2mpu08_pmic_dt_parse_pdata(iodev, pdata);
		if (ret)
			return ret;
	}

	if (!pdata) {
		dev_err(pdev->dev.parent, "Platform data not supplied\n");
		return -ENODEV;
	}

	s2mpu08 = devm_kzalloc(&pdev->dev, sizeof(struct s2mpu08_info),
				GFP_KERNEL);
	if (!s2mpu08)
		return -ENOMEM;

	s2mpu08->iodev = iodev;
	s2mpu08->i2c = iodev->pmic;

	mutex_init(&s2mpu08->lock);
	static_info = s2mpu08;

	platform_set_drvdata(pdev, s2mpu08);

	for (i = 0; i < pdata->num_regulators; i++) {
		int id = pdata->regulators[i].id;
		config.dev = &pdev->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = s2mpu08;
		config.of_node = pdata->regulators[i].reg_node;
		s2mpu08->opmode[id] =
			regulators[id].enable_mask;

		s2mpu08->rdev[i] = regulator_register(
				&regulators[id], &config);
		if (IS_ERR(s2mpu08->rdev[i])) {
			ret = PTR_ERR(s2mpu08->rdev[i]);
			dev_err(&pdev->dev, "regulator init failed for %d\n",
				i);
			s2mpu08->rdev[i] = NULL;
			goto err;
		}

	}

	s2mpu08->num_regulators = pdata->num_regulators;
	s2mpu08_update_reg(s2mpu08->i2c, S2MPU08_PMIC_REG_RTCBUF, 0x2, 0x2);

	/* SELMIF settings */
	/* LDO2,4,7,5,6,7,8,33,34,35 - controlled by PWREN_MIF */
	/* LDO1,10,11,12,13,14 - controlled by PWREN */
	s2mpu08_write_reg(s2mpu08->i2c, S2MPU08_PMIC_REG_SEL_CTRL1, 0x7E);
	s2mpu08_update_reg(s2mpu08->i2c, S2MPU08_PMIC_REG_SEL_CTRL2, 0x00, 0x7F);

	/* initial conditions for enabling and disabling LDO35,36 */
	/* this w/a will not applied after rev 1 */
	s2mpu08_read_reg(iodev->i2c, S2MPU08_PMIC_REG_PMICID, &SEC_PMIC_REV(iodev));
	if(iodev->pmic_rev == 0x00) {
		s2mpu08_update_reg(s2mpu08->i2c, 0x75, 0xF0, 0xF0);
		s2mpu08_update_reg(s2mpu08->i2c, 0x7C, 0x08, 0x08);
		s2mpu08_update_reg(s2mpu08->i2c, 0x8B, 0x00, 0x60);
	}

	/* changed water out THD in codec side */
	s2mpu08_write_reg(iodev->close, 0x83, 0x74);
	s2mpu08_write_reg(iodev->close, 0x84, 0x0E);

	/* changed water jack in THD in codec side */
	s2mpu08_read_reg(iodev->close, 0x7C, &flag_wtp);
	if ((flag_wtp & BIT(7)) == false) {
		s2mpu08_update_reg(iodev->close, 0x7C, BIT(7), BIT(7));
		s2mpu08_read_reg(iodev->close, 0x82, &lowr_wtp);
		lowr_wtp -= 15;
		s2mpu08_write_reg(iodev->close, 0x82, lowr_wtp);
	}

#ifdef CONFIG_SEC_PM
	ap_pmic_dev = sec_device_create(NULL, "ap_pmic");

	ret = sysfs_create_group(&ap_pmic_dev->kobj, &ap_pmic_attr_group);
	if (ret)
		dev_err(&pdev->dev, "failed to create ap_pmic sysfs group\n");
#endif /* CONFIG_SEC_PM */

#ifdef CONFIG_DEBUG_FS
	dbgi2c = s2mpu08->i2c;
	s2mpu08_root = debugfs_create_dir("s2mpu08-regs", NULL);
	s2mpu08_i2caddr = debugfs_create_file("i2caddr", 0644, s2mpu08_root, NULL, &s2mpu08_i2caddr_fops);
	s2mpu08_i2cdata = debugfs_create_file("i2cdata", 0644, s2mpu08_root, NULL, &s2mpu08_i2cdata_fops);
#endif

	/* Turn off CP regulators for LPM charging: L16 -> L17 -> L15 -> B9 */
	if (lpcharge) {
		s2mpu08_update_reg(s2mpu08->i2c, 0x3C, 0x00, 0xC0);	// LDO16
		s2mpu08_update_reg(s2mpu08->i2c, 0x3D, 0x00, 0xC0);	// LDO17
		s2mpu08_update_reg(s2mpu08->i2c, 0x3B, 0x00, 0xC0);	// LDO15
		s2mpu08_update_reg(s2mpu08->i2c, 0x26, 0x00, 0xC0);	// Buck9
	}

	pr_info("%s s2mpu08 pmic driver Loading end\n", __func__);

	return 0;
err:
	for (i = 0; i < S2MPU08_REGULATOR_MAX; i++)
		regulator_unregister(s2mpu08->rdev[i]);

	return ret;
}

static int s2mpu08_pmic_remove(struct platform_device *pdev)
{
	struct s2mpu08_info *s2mpu08 = platform_get_drvdata(pdev);
	int i;

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(s2mpu08_i2cdata);
	debugfs_remove_recursive(s2mpu08_i2caddr);
	debugfs_remove_recursive(s2mpu08_root);
#endif

	for (i = 0; i < S2MPU08_REGULATOR_MAX; i++)
		regulator_unregister(s2mpu08->rdev[i]);

	return 0;
}

static const struct platform_device_id s2mpu08_pmic_id[] = {
	{ "s2mpu08-regulator", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, s2mpu08_pmic_id);

static struct platform_driver s2mpu08_pmic_driver = {
	.driver = {
		.name = "s2mpu08-regulator",
		.owner = THIS_MODULE,
	},
	.probe = s2mpu08_pmic_probe,
	.remove = s2mpu08_pmic_remove,
	.id_table = s2mpu08_pmic_id,
};

static int __init s2mpu08_pmic_init(void)
{
	return platform_driver_register(&s2mpu08_pmic_driver);
}
subsys_initcall(s2mpu08_pmic_init);

static void __exit s2mpu08_pmic_exit(void)
{
	platform_driver_unregister(&s2mpu08_pmic_driver);
}
module_exit(s2mpu08_pmic_exit);

/* Module information */
MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG S2MPU08 Regulator Driver");
MODULE_LICENSE("GPL");
