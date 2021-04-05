/*
 * s2mu205-irq.c - Interrupt controller support for s2mu205
 *
 * Copyright (C) 2018 Samsung Electronics Co.Ltd
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/mfd/samsung/s2mu205.h>
#include <linux/err.h>

/* TODO : add IP Header file include*/
#if defined(CONFIG_CHARGER_S2MU205)
#include "../battery_v2/include/charger/s2mu205_charger.h"
#endif
#if defined(CONFIG_MUIC_S2MU205)
#include <linux/muic/s2mu205-muic.h>
#endif
#if defined(CONFIG_LEDS_S2MU205_FLASH)
#include <linux/leds-s2mu205.h>
#endif

#define S2MU205_IRQ_GROUP_MAX

static const u8 s2mu205_mask_reg[] = {
#if defined(CONFIG_CHARGER_S2MU205)
	[CHG_INT1] = S2MU205_CHG_INT1M,
	[CHG_INT2] = S2MU205_CHG_INT2M,
	[CHG_INT3] = S2MU205_CHG_INT3M,
#endif
#if defined(CONFIG_LEDS_S2MU205_FLASH)
	[FLED_INT1] = S2MU205_FLED_INT1_MASK,
#endif
#if defined(CONFIG_MUIC_S2MU205)
	[MUIC_INT1] = S2MU205_REG_MUIC_INT1,
	[MUIC_INT2] = S2MU205_REG_MUIC_INT2,
#endif
};

struct s2mu205_irq_data {
	int mask;
	enum s2mu205_irq_source group;
};

static struct i2c_client *get_i2c(struct s2mu205_dev *s2mu205,
					enum s2mu205_irq_source src)
{
	switch (src) {
#if defined(CONFIG_CHARGER_S2MU205)
	case CHG_INT1 ... CHG_INT3:
		return s2mu205->i2c;
#endif
#if defined(CONFIG_LEDS_S2MU205_FLASH)
	case FLED_INT1:
		return s2mu205->i2c;
#endif
#if defined(CONFIG_MUIC_S2MU205)
	case MUIC_INT1 ... MUIC_INT2:
		return s2mu205->muic;
#endif
	default:
		return ERR_PTR(-EINVAL);
	}
}

#define DECLARE_IRQ(idx, _group, _mask)		\
	[(idx)] = { .group = (_group), .mask = (_mask) }
static const struct s2mu205_irq_data s2mu205_irqs[] = {
#if defined(CONFIG_CHARGER_S2MU205)
	DECLARE_IRQ(S2MU205_CHG1_IRQ_SYS,	CHG_INT1,	1 << 0),
	DECLARE_IRQ(S2MU205_CHG1_IRQ_CV,	CHG_INT1,	1 << 1),
	DECLARE_IRQ(S2MU205_CHG1_IRQ_CHG_Fault,	CHG_INT1,	1 << 2),
	DECLARE_IRQ(S2MU205_CHG1_IRQ_CHG_RSTART, CHG_INT1,	1 << 3),
	DECLARE_IRQ(S2MU205_CHG1_IRQ_DONE,	CHG_INT1,	1 << 4),
	DECLARE_IRQ(S2MU205_CHG1_IRQ_TOP_OFF,	CHG_INT1,	1 << 5),
	DECLARE_IRQ(S2MU205_CHG1_IRQ_CHGIN,	CHG_INT1,	1 << 6),

	DECLARE_IRQ(S2MU205_CHG2_IRQ_ICR,	CHG_INT2,	1 << 0),
	DECLARE_IRQ(S2MU205_CHG2_IRQ_IVR,	CHG_INT2,	1 << 1),
	DECLARE_IRQ(S2MU205_CHG2_IRQ_AICL,	CHG_INT2,	1 << 2),
	DECLARE_IRQ(S2MU205_CHG2_IRQ_VBUS_Short,	CHG_INT2,	1 << 3),
	DECLARE_IRQ(S2MU205_CHG2_IRQ_BST,	CHG_INT2,	1 << 4),
	DECLARE_IRQ(S2MU205_CHG2_IRQ_OTG,	CHG_INT2,	1 << 5),
	DECLARE_IRQ(S2MU205_CHG2_IRQ_BAT,	CHG_INT2,	1 << 6),
	DECLARE_IRQ(S2MU205_CHG2_IRQ_MaxDuty,	CHG_INT2,	1 << 7),

	DECLARE_IRQ(S2MU205_CHG3_IRQ_BATID,	CHG_INT3,	1 << 0),
	DECLARE_IRQ(S2MU205_CHG3_IRQ_BATID_DONE,	CHG_INT3,	1 << 1),
	DECLARE_IRQ(S2MU205_CHG3_IRQ_QBAT_OFF,	CHG_INT3,	1 << 2),
	DECLARE_IRQ(S2MU205_CHG3_IRQ_BATN_OPEN, CHG_INT3,	1 << 4),
	DECLARE_IRQ(S2MU205_CHG3_IRQ_BATP_OPEN,	CHG_INT3,	1 << 5),
	DECLARE_IRQ(S2MU205_CHG3_IRQ_BAT_Contact_CK_Done, CHG_INT3, 1 << 6),
#endif
#if defined(CONFIG_LEDS_S2MU205_FLASH)
	DECLARE_IRQ(S2MU205_FLED1_IRQ_OPEN_CH2,	FLED_INT1,	1 << 4),
	DECLARE_IRQ(S2MU205_FLED1_IRQ_OPEN_CH1, FLED_INT1,	1 << 5),
	DECLARE_IRQ(S2MU205_FLED1_IRQ_SHORT_CH2, FLED_INT1,	1 << 6),
	DECLARE_IRQ(S2MU205_FLED1_IRQ_SHORT_CH1, FLED_INT1,	1 << 7),
#endif
#if defined(CONFIG_MUIC_S2MU205)
	/* declare order is priority : DETACH -> ATTACH */
	DECLARE_IRQ(S2MU205_MUIC_IRQ1_DETACH,		MUIC_INT1,	1 << 1),
	DECLARE_IRQ(S2MU205_MUIC_IRQ1_ATTACH,		MUIC_INT1,	1 << 0),
	DECLARE_IRQ(S2MU205_MUIC_IRQ1_KP,			MUIC_INT1,	1 << 2),
	DECLARE_IRQ(S2MU205_MUIC_IRQ1_LKP,			MUIC_INT1,	1 << 3),
	DECLARE_IRQ(S2MU205_MUIC_IRQ1_LKR,			MUIC_INT1,	1 << 4),
	DECLARE_IRQ(S2MU205_MUIC_IRQ1_RID_CHG,		MUIC_INT1,	1 << 5),
	DECLARE_IRQ(S2MU205_MUIC_IRQ1_USB_Killer,	MUIC_INT1,	1 << 6),
	DECLARE_IRQ(S2MU205_MUIC_IRQ1_RID_WAKEUP,	MUIC_INT1,	1 << 7),

	DECLARE_IRQ(S2MU205_MUIC_IRQ2_VBUS_ON,		MUIC_INT2, 1 << 0),
	DECLARE_IRQ(S2MU205_MUIC_IRQ2_RSVD_ATTACH,	MUIC_INT2, 1 << 1),
	DECLARE_IRQ(S2MU205_MUIC_IRQ2_ADC_CHANGE,	MUIC_INT2, 1 << 2),
	DECLARE_IRQ(S2MU205_MUIC_IRQ2_STUCK,		MUIC_INT2, 1 << 3),
	DECLARE_IRQ(S2MU205_MUIC_IRQ2_STUCKRCV,		MUIC_INT2, 1 << 4),
	DECLARE_IRQ(S2MU205_MUIC_IRQ2_MHDL,			MUIC_INT2, 1 << 5),
	DECLARE_IRQ(S2MU205_MUIC_IRQ2_AV_CHARGE,	MUIC_INT2, 1 << 6),
	DECLARE_IRQ(S2MU205_MUIC_IRQ2_VBUS_OFF,		MUIC_INT2, 1 << 7),
#endif
};

static void s2mu205_irq_lock(struct irq_data *data)
{
	struct s2mu205_dev *s2mu205 = irq_get_chip_data(data->irq);

	mutex_lock(&s2mu205->irqlock);
}

static void s2mu205_irq_sync_unlock(struct irq_data *data)
{
	struct s2mu205_dev *s2mu205 = irq_get_chip_data(data->irq);
	int i;
	u8 mask_reg;
	struct i2c_client *i2c;

	for (i = 0; i < S2MU205_IRQ_GROUP_NR; i++) {
		mask_reg = s2mu205_mask_reg[i];
		i2c = get_i2c(s2mu205, i);

		if (mask_reg == S2MU205_REG_INVALID ||
				IS_ERR_OR_NULL(i2c))
			continue;
		s2mu205->irq_masks_cache[i] = s2mu205->irq_masks_cur[i];

		s2mu205_write_reg(i2c, s2mu205_mask_reg[i],
				s2mu205->irq_masks_cur[i]);
	}

	mutex_unlock(&s2mu205->irqlock);
}

static const inline struct s2mu205_irq_data *
irq_to_s2mu205_irq(struct s2mu205_dev *s2mu205, int irq)
{
	return &s2mu205_irqs[irq - s2mu205->irq_base];
}

static void s2mu205_irq_mask(struct irq_data *data)
{
	struct s2mu205_dev *s2mu205 = irq_get_chip_data(data->irq);
	const struct s2mu205_irq_data *irq_data =
	    irq_to_s2mu205_irq(s2mu205, data->irq);

	if (irq_data->group >= S2MU205_IRQ_GROUP_NR)
		return;

	s2mu205->irq_masks_cur[irq_data->group] |= irq_data->mask;
}

static void s2mu205_irq_unmask(struct irq_data *data)
{
	struct s2mu205_dev *s2mu205 = irq_get_chip_data(data->irq);
	const struct s2mu205_irq_data *irq_data =
	    irq_to_s2mu205_irq(s2mu205, data->irq);

	if (irq_data->group >= S2MU205_IRQ_GROUP_NR)
		return;

	s2mu205->irq_masks_cur[irq_data->group] &= ~irq_data->mask;
}

static struct irq_chip s2mu205_irq_chip = {
	.name			= MFD_DEV_NAME,
	.irq_bus_lock	= s2mu205_irq_lock,
	.irq_bus_sync_unlock	= s2mu205_irq_sync_unlock,
	.irq_mask		= s2mu205_irq_mask,
	.irq_unmask		= s2mu205_irq_unmask,
};

static irqreturn_t s2mu205_irq_thread(int irq, void *data)
{
	struct s2mu205_dev *s2mu205 = data;
	u8 irq_reg[S2MU205_IRQ_GROUP_NR] = {0, };
	int i, ret;
	u8 irq_src;

	ret = s2mu205_read_reg(s2mu205->i2c, S2MU205_REG_IPINT, &irq_src);
	if (ret) {
        pr_err("%s:%s Failed to read interrupt source: %d\n",
            MFD_DEV_NAME, __func__, ret);
        return IRQ_NONE;
	}
	pr_info("%s: Top interrupt(0x%02x)\n", __func__, irq_src);

#if defined(CONFIG_CHARGER_S2MU205)
	if (irq_src & S2MU205_IRQSRC_CHG) {
		ret = s2mu205_bulk_read(s2mu205->i2c, S2MU205_CHG_INT1,
			S2MU205_NUM_IRQ_CHG_REGS, &irq_reg[CHG_INT1]);
		if (ret) {
			pr_err("%s:%s Failed to read charger interrupt: %d\n",
				MFD_DEV_NAME, __func__, ret);
			return IRQ_NONE;
		}
		pr_info("%s() CHARGER intrrupt(0x%02x, 0x%02x, 0x%02x)\n",
			__func__, irq_reg[CHG_INT1], irq_reg[CHG_INT2],
			irq_reg[CHG_INT3]);
	}
#endif
#if defined(CONFIG_LEDS_S2MU205_FLASH)
	if (irq_src & S2MU205_IRQSRC_FLED) {
		ret = s2mu205_read_reg(s2mu205->i2c, S2MU205_FLED_INT1,
			&irq_reg[FLED_INT1]);
		if (ret) {
			pr_err("%s:%s Failed to read led interrupt: %d\n",
				MFD_DEV_NAME, __func__, ret);
			return IRQ_NONE;
		}
		pr_info("%s() FLED intrrupt(0x%02x)\n", __func__, irq_reg[FLED_INT1]);
	}
#endif
#if defined(CONFIG_MUIC_S2MU205)
	if (irq_src & S2MU205_IRQSRC_MUIC) {
        s2mu205_read_reg(s2mu205->muic, S2MU205_REG_MUIC_INT1, &irq_reg[MUIC_INT1]);
        s2mu205_read_reg(s2mu205->muic, S2MU205_REG_MUIC_INT2, &irq_reg[MUIC_INT2]);
        pr_info("%s: MUIC interrupt(0x%02x, 0x%02x)\n", __func__, irq_reg[MUIC_INT1],
            irq_reg[MUIC_INT2]);
	}
#endif

	/* Apply masking */
	for (i = 0; i < S2MU205_IRQ_GROUP_NR; i++)
		irq_reg[i] &= ~s2mu205->irq_masks_cur[i];

	/* Report */
	for (i = 0; i < S2MU205_IRQ_NR; i++) {
		if (irq_reg[s2mu205_irqs[i].group] & s2mu205_irqs[i].mask)
			handle_nested_irq(s2mu205->irq_base + i);
	}

	return IRQ_HANDLED;
}
static int irq_is_enable = true;
int s2mu205_irq_init(struct s2mu205_dev *s2mu205)
{
	int i;
	int ret;
	struct i2c_client *i2c = s2mu205->i2c;
	int cur_irq;
	u8 i2c_data;

	if (!s2mu205->irq_gpio) {
		dev_warn(s2mu205->dev, "No interrupt specified.\n");
		s2mu205->irq_base = 0;
		return 0;
	}

	if (!s2mu205->irq_base) {
		dev_err(s2mu205->dev, "No interrupt base specified.\n");
		return 0;
	}

	mutex_init(&s2mu205->irqlock);

	s2mu205->irq = gpio_to_irq(s2mu205->irq_gpio);
	pr_err("%s:%s irq=%d, irq->gpio=%d\n", MFD_DEV_NAME, __func__,
			s2mu205->irq, s2mu205->irq_gpio);

	ret = gpio_request(s2mu205->irq_gpio, "if_pmic_irq");
	if (ret) {
		dev_err(s2mu205->dev, "%s: failed requesting gpio %d\n",
			__func__, s2mu205->irq_gpio);
		return ret;
	}
	gpio_direction_input(s2mu205->irq_gpio);
	gpio_free(s2mu205->irq_gpio);

	/* Mask individual interrupt sources */
	for (i = 0; i < S2MU205_IRQ_GROUP_NR; i++) {

		s2mu205->irq_masks_cur[i] = 0xff;
		s2mu205->irq_masks_cache[i] = 0xff;
        i2c = get_i2c(s2mu205, i);

		if (IS_ERR_OR_NULL(i2c))
			continue;
		if (s2mu205_mask_reg[i] == S2MU205_REG_INVALID)
			continue;
		s2mu205_write_reg(i2c, s2mu205_mask_reg[i], 0xff);
	}

	/* Register with genirq */
	for (i = 0; i < S2MU205_IRQ_NR; i++) {
		cur_irq = 0;
		cur_irq = i + s2mu205->irq_base;
		irq_set_chip_data(cur_irq, s2mu205);
		irq_set_chip_and_handler(cur_irq, &s2mu205_irq_chip,
						handle_level_irq);
		irq_set_nested_thread(cur_irq, 1);
#ifdef CONFIG_ARM
		set_irq_flags(cur_irq, IRQF_VALID);
#else
		irq_set_noprobe(cur_irq);
#endif
	}

	/* Unmask S2MU205 interrupt */
	i2c_data = 0xff;
#if defined(CONFIG_CHARGER_S2MU205)
	i2c_data &= ~(S2MU205_IRQSRC_CHG);
#endif
#if defined(CONFIG_LEDS_S2MU205_FLASH)
	i2c_data &= ~(S2MU205_IRQSRC_FLED);
#endif
#if defined(CONFIG_MUIC_S2MU205)
	i2c_data &= ~(S2MU205_IRQSRC_MUIC);
#endif
	s2mu205_write_reg(s2mu205->i2c, S2MU205_REG_IPINT_MASK, i2c_data);
	pr_info("%s: %s init top-irq mask(0x%02x)\n", MFD_DEV_NAME, __func__, i2c_data);
	pr_info("%s: irq gpio pre-state(0x%02x)\n", __func__,
		gpio_get_value(s2mu205->irq_gpio));

	if (irq_is_enable) {
		ret = request_threaded_irq(s2mu205->irq, NULL,
				s2mu205_irq_thread,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"s2mu205-irq", s2mu205);
	}

	if (ret) {
		dev_err(s2mu205->dev, "Failed to request IRQ %d: %d\n",
			s2mu205->irq, ret);
		return ret;
	}

	return 0;
}

void s2mu205_irq_exit(struct s2mu205_dev *s2mu205)
{
	if (s2mu205->irq)
		free_irq(s2mu205->irq, s2mu205);
}
