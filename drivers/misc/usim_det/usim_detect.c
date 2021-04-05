/*
 * Copyright (C) 2016 Samsung Electronics Co.Ltd
 * http://www.samsung.com
 *
 * USIM Detection driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
*/

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#include <linux/mcu_ipc.h>
#include <linux/usim_det.h>
#include <linux/modem_notifier.h>

static inline void usim_det_set_det0_value(struct usim_det_data *udd, int value)
{
	if (value != udd->usim0_det) {
		mbox_update_value(MCU_CP, udd->mbx_ap_united_status, value,
			udd->sbi_usim0_det_mask, udd->sbi_usim0_det_pos);
		mbox_set_interrupt(MCU_CP, udd->int_usim0_det);
		udd->usim0_det = value;
	}
}

static inline void usim_det_set_det1_value(struct usim_det_data *udd, int value)
{
	if (value != udd->usim1_det) {
		mbox_update_value(MCU_CP, udd->mbx_ap_united_status, value,
			udd->sbi_usim1_det_mask, udd->sbi_usim1_det_pos);
		mbox_set_interrupt(MCU_CP, udd->int_usim1_det);
		udd->usim1_det = value;
	}
}

static void usim_det_set_init_state(struct usim_det_data *udd)
{
	int value;

	if (udd->num_of_usim_det == 1) {
		value = gpio_get_value(udd->gpio_usim_det0);
		mbox_update_value(MCU_CP, udd->mbx_ap_united_status, value,
			udd->sbi_usim0_det_mask, udd->sbi_usim0_det_pos);
		udd->usim0_det = value;
		pr_err("init value of usim_det0: %d\n", value);
	}

	if (udd->num_of_usim_det == 2) {
		value = gpio_get_value(udd->gpio_usim_det0);
		mbox_update_value(MCU_CP, udd->mbx_ap_united_status, value,
			udd->sbi_usim0_det_mask, udd->sbi_usim0_det_pos);
		udd->usim0_det = value;
		pr_err("init value of usim_det0: %d\n", value);

		value = gpio_get_value(udd->gpio_usim_det1);
		mbox_update_value(MCU_CP, udd->mbx_ap_united_status, value,
			udd->sbi_usim1_det_mask, udd->sbi_usim1_det_pos);
		udd->usim1_det = value;
		pr_err("init value of usim_det1: %d\n", value);
	}
}

static irqreturn_t usim_dt_interrupt0(int irq, void *dev_id)
{
	struct usim_det_data *udd = dev_id;
	int value;
	int i, flag;

	if (udd->modem_state != MODEM_EVENT_ONLINE) {
		pr_err("%s: MODEM_STATE is not online, Do not anyting.\n", __func__);
		return IRQ_HANDLED;
	}

	value = gpio_get_value(udd->gpio_usim_det0);

	if (value == 0) {
		/* Check HIGH -> LOW */
		flag = 0;
		for (i = 0; i < udd->usim_low_detect_count; i++) {
			msleep_interruptible(udd->usim_check_delay_msec);
			value = gpio_get_value(udd->gpio_usim_det0);
			if (value == 0)
				flag++;
			else
				break;
		}
		if (flag == udd->usim_low_detect_count) {
			usim_det_set_det0_value(udd, 0);
			pr_err("%s: USIM0_DET: HIGH -> LOW\n", __func__);
		} else {
			pr_err("%s: USIM0_DET HIGH->LOW failed (flag : %d)\n", __func__, flag);
		}

	} else {
		/* Check LOW -> HIGH */
		flag = 0;
		for (i = 0; i < udd->usim_high_detect_count; i++) {
			msleep_interruptible(udd->usim_check_delay_msec);
			value = gpio_get_value(udd->gpio_usim_det0);
			if (value == 1)
				flag++;
			else
				break;
		}
		if (flag == udd->usim_high_detect_count) {
			usim_det_set_det0_value(udd, 1);
			pr_err("%s: USIM0_DET: LOW -> HIGH\n", __func__);
		} else {
			pr_err("%s: USIM0_DET LOW->HIGH failed (flag : %d)\n", __func__, flag);
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t usim_dt_interrupt1(int irq, void *dev_id)
{
	struct usim_det_data *udd = dev_id;
	int value;
	int i, flag;

	if (udd->modem_state != MODEM_EVENT_ONLINE) {
		pr_err("%s: MODEM_STATE is not online, Do not anyting.\n", __func__);
		return IRQ_HANDLED;
	}

	value = gpio_get_value(udd->gpio_usim_det1);

	if (value == 0) {
		/* Check HIGH -> LOW */
		flag = 0;
		for (i = 0; i < udd->usim_low_detect_count; i++) {
			msleep_interruptible(udd->usim_check_delay_msec);
			value = gpio_get_value(udd->gpio_usim_det1);
			if (value == 0)
				flag++;
			else
				break;
		}
		if (flag == udd->usim_low_detect_count) {
			usim_det_set_det1_value(udd, 0);
			pr_err("%s: USIM1_DET: HIGH -> LOW\n", __func__);
		} else {
			pr_err("%s: USIM1_DET HIGH->LOW failed (flag : %d)\n", __func__, flag);
		}
	} else {
		/* Check LOW -> HIGH */
		flag = 0;
		for (i = 0; i < udd->usim_high_detect_count; i++) {
			msleep_interruptible(udd->usim_check_delay_msec);
			value = gpio_get_value(udd->gpio_usim_det1);
			if (value == 1)
				flag++;
			else
				break;
		}
		if (flag == udd->usim_high_detect_count) {
			usim_det_set_det1_value(udd, 1);
			pr_err("%s: USIM1_DET: LOW -> HIGH\n", __func__);
		} else {
			pr_err("%s: USIM1_DET LOW->HIGH failed (flag : %d)\n", __func__, flag);
		}
	}

	return IRQ_HANDLED;
}

static int usim_modem_notifier(struct notifier_block *nb,
		unsigned long action, void *nb_data)
{
	struct usim_det_data *udd = container_of(nb, struct usim_det_data, modem_nb);

	udd->modem_state = action;
	switch (udd->modem_state) {
	case MODEM_EVENT_BOOTING:
		usim_det_set_init_state(udd);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static int usim_detect_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct usim_det_data *udd;
	int err = 0;

	pr_err("%s +++\n", __func__);

	udd = devm_kzalloc(dev, sizeof(struct usim_det_data), GFP_KERNEL);
	if (udd == NULL)
		return -ENOMEM;

	err = of_property_read_u32(dev->of_node, "mif,num_of_usim_det",
			&udd->num_of_usim_det);
	if (err) {
		pr_err("USIM_DET parse error! [num_of_usim_det]\n");
		goto exit_err;
	}

	pr_err("num_of_usim_det: %d\n", udd->num_of_usim_det);

	if (udd->num_of_usim_det == 0 || udd->num_of_usim_det > 2)
		goto exit_err;
	
	/* USIM delay check */
	err = of_property_read_u32(dev->of_node, "usim_check_delay_msec",
			&udd->usim_check_delay_msec);
	if (err) {
		udd->usim_check_delay_msec = USIM_CHECK_DELAY_MSEC_DEFAULT;
	}

	pr_err("usim_check_delay_msec: %d\n", udd->usim_check_delay_msec);
	
	/* USIM high detect count */
	err = of_property_read_u32(dev->of_node, "usim_high_detect_count",
			&udd->usim_high_detect_count);
	if (err) {
		udd->usim_high_detect_count = USIM_HIGH_DETECT_COUNT_DEFAULT;
	}

	pr_err("usim_high_detect_count: %d\n", udd->usim_high_detect_count);
	
	/* USIM low detect count */
	err = of_property_read_u32(dev->of_node, "usim_low_detect_count",
			&udd->usim_low_detect_count);
	if (err) {
		udd->usim_low_detect_count = USIM_LOW_DETECT_COUNT_DEFAULT;
	}

	pr_err("usim_low_detect_count: %d\n", udd->usim_low_detect_count);

	/* USIM0_DET */
	err = of_property_read_u32(dev->of_node,
		"mbx_ap2cp_united_status", &udd->mbx_ap_united_status);
	if (err) {
		pr_err("USIM_DET parse error!\n");
		goto exit_err;
	}
	err = of_property_read_u32(dev->of_node, "mif,int_usim0_det_level",
			&udd->int_usim0_det);
	if (err) {
		pr_err("USIM_DET parse error!\n");
		goto exit_err;
	}
	err = of_property_read_u32(dev->of_node, "sbi_usim0_det_mask",
			&udd->sbi_usim0_det_mask);
	if (err) {
		pr_err("USIM_DET DTS parse error!\n");
		goto exit_err;
	}
	err = of_property_read_u32(dev->of_node, "sbi_usim0_det_pos",
			&udd->sbi_usim0_det_pos);
	if (err) {
		pr_err("USIM_DET DTS parse error!\n");
		goto exit_err;
	}

	udd->gpio_usim_det0 = of_get_named_gpio_flags(np, "mif,usim-det0-gpio",
		       0, &udd->usim_det0_gpio_flags);
	if (!gpio_is_valid(udd->gpio_usim_det0))
		goto exit_err;

	err = gpio_request_one(udd->gpio_usim_det0, GPIOF_IN, "usim_det0");
	if (err) {
		pr_err("%s: unable to request usim_det0 [%d]\n",
				__func__, udd->gpio_usim_det0);
		goto exit_err;
	}
	udd->usim_det0_irq = gpio_to_irq(udd->gpio_usim_det0);

	err = request_threaded_irq(udd->usim_det0_irq, NULL,
		usim_dt_interrupt0, IRQF_TRIGGER_RISING |
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
		USIM_DETECT_NAME0, udd);
	if (err < 0) {
		pr_err("%s:Failed to register interrupt0, ret = %d\n",
				__func__, err);
		goto exit_err;
	}
	enable_irq_wake(udd->usim_det0_irq);

	/* USIM1_DET */
	if (udd->num_of_usim_det == 2) {
		err = of_property_read_u32(dev->of_node,
				"mif,int_usim1_det_level", &udd->int_usim1_det);
		if (err) {
			pr_err("USIM_DET DTS parse error!\n");
			goto exit_err;
		}
		err = of_property_read_u32(dev->of_node, "sbi_usim1_det_mask",
			&udd->sbi_usim1_det_mask);
		if (err) {
			pr_err("USIM_DET parse error!\n");
			goto exit_err;
		}
		err = of_property_read_u32(dev->of_node, "sbi_usim1_det_pos",
			&udd->sbi_usim1_det_pos);
		if (err) {
			pr_err("USIM_DET parse error!\n");
			goto exit_err;
		}

		udd->gpio_usim_det1 = of_get_named_gpio_flags(np,
			"mif,usim-det1-gpio", 0,
			&udd->usim_det1_gpio_flags);
		if (!gpio_is_valid(udd->gpio_usim_det1))
			goto exit_err;

		err = gpio_request_one(udd->gpio_usim_det1, GPIOF_IN,
				"usim_det1");
		if (err) {
			pr_err("%s: unable to request usim_det1 [%d]\n",
					__func__, udd->gpio_usim_det1);
			goto exit_err;
		}
		udd->usim_det1_irq = gpio_to_irq(udd->gpio_usim_det1);

		/* request threaded irq */
		err = request_threaded_irq(udd->usim_det1_irq, NULL,
			usim_dt_interrupt1, IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			USIM_DETECT_NAME1, udd);
		if (err < 0) {
			pr_err("%s:Failed to register interrupt1, ret = %d\n",
				__func__, err);
			goto exit_err;
		}

		enable_irq_wake(udd->usim_det1_irq);
	}

	usim_det_set_init_state(udd);

	udd->modem_nb.notifier_call = usim_modem_notifier;
	udd->modem_state = 0;
	err = register_modem_event_notifier(&udd->modem_nb);
	if (err < 0) {
		pr_err("%s: fail to register modem event notifier\n", __func__);
		goto exit_err;
	}

	platform_set_drvdata(pdev, udd);

	pr_err("%s ---\n", __func__);

	return 0;

exit_err:
	pr_err("%s: ERROR\n", __func__);
	devm_kfree(dev, udd);
	return -EINVAL;
}

static int __exit usim_detect_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usim_det_data *udd = dev_get_drvdata(dev);

	disable_irq_wake(udd->usim_det0_irq);
	if (udd->num_of_usim_det == 2)
		disable_irq_wake(udd->usim_det1_irq);

	devm_kfree(dev, udd);
	return 0;
}

#ifdef CONFIG_PM
static int usim_detect_suspend(struct device *dev)
{
	return 0;
}

static int usim_detect_resume(struct device *dev)
{
	return 0;
}
#else
#define usim_detect_suspend NULL
#define usim_detect_resume NULL
#endif

static const struct dev_pm_ops usim_detect_pm_ops = {
	.suspend = usim_detect_suspend,
	.resume = usim_detect_resume,
};

static const struct of_device_id exynos_usim_detect_dt_match[] = {
		{ .compatible = "samsung,exynos-usim-detect", },
		{},
};
MODULE_DEVICE_TABLE(of, exynos_uart_sel_dt_match);

static struct platform_driver usim_detect_driver = {
	.probe		= usim_detect_probe,
	.remove		= usim_detect_remove,
	.driver		= {
		.name = "usim_detect",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(exynos_usim_detect_dt_match),
		.pm = &usim_detect_pm_ops,
	},
};
module_platform_driver(usim_detect_driver);

MODULE_DESCRIPTION("USIM_DETECT driver");
MODULE_AUTHOR("<tj7.kim@samsung.com>");
MODULE_LICENSE("GPL");
