/* drivers/muic/muic-core.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio.h>

/* switch device header */
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif /* CONFIG_SWITCH */

#include <linux/muic/muic.h>
#include <linux/sec_sysfs.h>
#if defined(CONFIG_MUIC_NOTIFIER)
#include <linux/muic/muic_notifier.h>
#endif /* CONFIG_MUIC_NOTIFIER */

#if defined(CONFIG_MUIC_SUPPORT_CCIC) && defined(CONFIG_CCIC_NOTIFIER)
#include <linux/ccic/ccic_notifier.h>
#endif

#include "muic-internal.h"

struct device *switch_device;

static struct switch_dev switch_dock = {
	.name = "dock",
};

struct switch_dev switch_uart3 = {
	.name = "uart3",    /* sys/class/switch/uart3/state */
};

#if defined(CONFIG_MUIC_NOTIFIER)
static struct notifier_block muic_notifier_block;

void muic_send_dock_intent(int type)
{
	pr_info("%s: MUIC dock type(%d)\n", __func__, type);
#ifdef CONFIG_SWITCH
	switch_set_state(&switch_dock, type);
#endif
}

static void muic_jig_uart_cb(int jig_state)
{
	pr_info("%s(%d)\n", __func__, jig_state);
#ifdef CONFIG_SWITCH
	switch_set_state(&switch_uart3, jig_state);
#endif
}

static int muic_dock_attach_notify(int type, const char *name)
{
	pr_info("%s: %s\n", __func__, name);
	muic_send_dock_intent(type);

	return NOTIFY_OK;
}

static int muic_dock_detach_notify(void)
{
	pr_info("%s\n", __func__);
	muic_send_dock_intent(MUIC_DOCK_DETACHED);

	return NOTIFY_OK;
}

static int muic_handle_notification(struct notifier_block *nb,
			unsigned long action, void *data)
{
#if defined(CONFIG_CCIC_NOTIFIER) && defined(CONFIG_MUIC_SUPPORT_CCIC)
	CC_NOTI_ATTACH_TYPEDEF *pnoti = (CC_NOTI_ATTACH_TYPEDEF *)data;
	muic_attached_dev_t attached_dev = pnoti->cable_type;
#else
	muic_attached_dev_t attached_dev = *(muic_attached_dev_t *)data;
#endif
	bool jig_state = false;
	int type = MUIC_DOCK_DETACHED, ret = NOTIFY_DONE;
	const char *name;

	switch (attached_dev) {
	case ATTACHED_DEV_DESKDOCK_MUIC:
	case ATTACHED_DEV_DESKDOCK_VB_MUIC:
		type = MUIC_DOCK_DESKDOCK;
		name = "Desk Dock Attach";
		break;
	case ATTACHED_DEV_CARDOCK_MUIC:
		type = MUIC_DOCK_CARDOCK;
		name = "Car Dock Attach";
		break;
	case ATTACHED_DEV_SMARTDOCK_MUIC:
	case ATTACHED_DEV_SMARTDOCK_VB_MUIC:
	case ATTACHED_DEV_SMARTDOCK_TA_MUIC:
	case ATTACHED_DEV_SMARTDOCK_USB_MUIC:
		type = MUIC_DOCK_SMARTDOCK;
		name = "Smart Dock Attach";
		break;
	case ATTACHED_DEV_UNIVERSAL_MMDOCK_MUIC:
		type = MUIC_DOCK_SMARTDOCK;
		name = "Universal MMDock Attach";
		break;
	case ATTACHED_DEV_AUDIODOCK_MUIC:
		type = MUIC_DOCK_AUDIODOCK;
		name = "Audio Dock Attach";
		break;
	case ATTACHED_DEV_HMT_MUIC:
		type = MUIC_DOCK_HMT;
		name = "HMT Attach";
		break;
	case ATTACHED_DEV_GAMEPAD_MUIC:
		type = MUIC_DOCK_GAMEPAD;
		name = "Gamepad Attach";
		break;
	case ATTACHED_DEV_JIG_UART_ON_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_MUIC:
	case ATTACHED_DEV_JIG_UART_OFF_VB_MUIC:
	case ATTACHED_DEV_JIG_USB_ON_MUIC:
	case ATTACHED_DEV_JIG_USB_OFF_MUIC:
		if (action == MUIC_NOTIFY_CMD_ATTACH)
			jig_state = true;
		break;
	default:
		pr_info("%s: ignore(%d)\n", __func__, attached_dev);
		break;
	}

	if (type != MUIC_DOCK_DETACHED) {
		if (action == MUIC_NOTIFY_CMD_ATTACH)
			ret = muic_dock_attach_notify(type, name);
		else if (action == MUIC_NOTIFY_CMD_DETACH)
			ret = muic_dock_detach_notify();
	}

	muic_jig_uart_cb(jig_state);

	return ret;
}
#endif /* CONFIG_MUIC_NOTIFIER */

static void muic_init_switch_dev_cb(void)
{
#ifdef CONFIG_SWITCH
	int ret;

	/* for DockObserver */
	ret = switch_dev_register(&switch_dock);
	if (ret < 0) {
		pr_err("%s: Failed to register dock switch(%d)\n",
				__func__, ret);
		return;
	}

	ret = switch_dev_register(&switch_uart3);
	if (ret < 0) {
		pr_err("%s: Failed to register dock switch(%d)\n",
				__func__, ret);
		return;
	}
#endif /* CONFIG_SWITCH */

#if defined(CONFIG_MUIC_NOTIFIER)
	muic_notifier_register(&muic_notifier_block,
			muic_handle_notification, MUIC_NOTIFY_DEV_DOCK);
#endif /* CONFIG_MUIC_NOTIFIER */

	pr_info("%s: done\n", __func__);
}

static void muic_cleanup_switch_dev_cb(void)
{
#if defined(CONFIG_MUIC_NOTIFIER)
	muic_notifier_unregister(&muic_notifier_block);
#endif /* CONFIG_MUIC_NOTIFIER */

	pr_info("%s: done\n", __func__);
}

extern struct muic_platform_data muic_pdata;

/* func : set_switch_sel
 * switch_sel value get from bootloader comand line
 * switch_sel data consist 8 bits (xxxxyyyyzzzz)
 * first 4bits(zzzz) mean path infomation.
 * next 4bits(yyyy) mean if pmic version info
 * next 4bits(xxxx) mean afc disable info
 */
static int set_switch_sel(char *str)
{
	get_option(&str, &muic_pdata.switch_sel);
	muic_pdata.switch_sel = (muic_pdata.switch_sel) & 0xfff;
	pr_info("%s: switch_sel: 0x%03x\n", __func__,
			muic_pdata.switch_sel);

	return muic_pdata.switch_sel;
}
__setup("pmic_info=", set_switch_sel);

int get_switch_sel(void)
{
	return muic_pdata.switch_sel;
}

static int set_factory_uart(char *str)
{
	int mode;

	get_option(&str, &mode);
	muic_pdata.is_factory_uart = !!(mode & 0x1);

	pr_info("factory_uart: %sable\n",
		muic_pdata.is_factory_uart ? "en" : "dis");

	return 0;
}
__setup("androidboot.muic_1k=", set_factory_uart);

/* afc_mode:
 *   0x31 : Disabled
 *   0x30 : Enabled
 */
static int afc_mode = 0;
static int __init set_afc_mode(char *str)
{
	int mode;
	get_option(&str, &mode);
	afc_mode = (mode & 0x0000FF00) >> 8;
	pr_info("%s: afc_mode is 0x%02x\n", __func__, afc_mode);

	return 0;
}
early_param("charging_mode", set_afc_mode);

int get_afc_mode(void)
{
	return afc_mode;
}

bool is_muic_usb_path_ap_usb(void)
{
	if (MUIC_PATH_USB_AP == muic_pdata.usb_path) {
		pr_info("%s: [%d]\n", __func__, muic_pdata.usb_path);
		return true;
	}

	return false;
}

bool is_muic_usb_path_cp_usb(void)
{
	if (MUIC_PATH_USB_CP == muic_pdata.usb_path) {
		pr_info("%s: [%d]\n", __func__, muic_pdata.usb_path);
		return true;
	}

	return false;
}

static int muic_init_gpio_cb(void *data, int switch_sel)
{
	struct muic_platform_data *pdata = (struct muic_platform_data *)data;
	const char *usb_mode;
	const char *uart_mode;
	int ret = 0;

	pr_info("%s (%d)\n", __func__, switch_sel);

	if (switch_sel & SWITCH_SEL_USB_MASK) {
		pdata->usb_path = MUIC_PATH_USB_AP;
		usb_mode = "PDA";
	} else {
		pdata->usb_path = MUIC_PATH_USB_CP;
		usb_mode = "MODEM";
	}

	if (pdata->set_gpio_usb_sel)
		ret = pdata->set_gpio_usb_sel(pdata->uart_path);

	if (switch_sel & SWITCH_SEL_UART_MASK) {
		pdata->uart_path = MUIC_PATH_UART_AP;
		uart_mode = "AP";
	} else {
		pdata->uart_path = MUIC_PATH_UART_CP;
		uart_mode = "CP";
	}

	/* These flags MUST be updated again from probe function */
	pdata->rustproof_on = false;

#if !defined(CONFIG_SEC_FACTORY)
	if (!(switch_sel & SWITCH_SEL_RUSTPROOF_MASK))
		pdata->rustproof_on = true;
#endif /* !CONFIG_SEC_FACTORY */

	pdata->afc_disable = false;

	if (pdata->set_gpio_uart_sel)
		ret = pdata->set_gpio_uart_sel(pdata->uart_path);

	pr_info("%s: usb_path(%s), uart_path(%s)\n", __func__,
			usb_mode, uart_mode);

	switch_device = sec_device_create(NULL, "switch");
	if (IS_ERR(switch_device)) {
		pr_err("%s Failed to create device(switch)!\n", __func__);
		ret = -ENODEV;
	}

	return ret;
}

struct muic_platform_data muic_pdata = {
	.init_switch_dev_cb	= muic_init_switch_dev_cb,
	.cleanup_switch_dev_cb	= muic_cleanup_switch_dev_cb,
	.init_gpio_cb		= muic_init_gpio_cb,
	.jig_uart_cb = muic_jig_uart_cb,
};
