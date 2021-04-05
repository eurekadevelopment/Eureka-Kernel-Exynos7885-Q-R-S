/*
 * SAMSUNG DBMDx event driver
 *
 * Copyright (c) 2014 Samsung Electronics Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/wakelock.h>

#include <sound/dbmdx-export.h>

struct platform_device *this_dev;

struct samsung_dbmdx_event_priv {
	struct class *svoice_class;
	struct device *keyword_dev;
	unsigned int keyword_type;
	struct wake_lock wake_lock;
};

static ssize_t svoice_keyword_type_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct samsung_dbmdx_event_priv *priv= dev_get_drvdata(dev);
	int ret;

	ret = snprintf(buf, PAGE_SIZE, "%x\n", priv->keyword_type);

	return ret;
}

static DEVICE_ATTR(keyword_type, S_IRUGO, svoice_keyword_type_show, NULL);

void samsung_dbmdx_voicewakeup_cb(int word_id)
{

	struct samsung_dbmdx_event_priv *priv;
	char keyword_buf[100];
	char *envp[2];

	memset(keyword_buf, 0, sizeof(keyword_buf));

	dev_info(&this_dev->dev, "%s: word_id = %d\n", __func__, word_id);

	WARN_ON(!this_dev);
	if (!this_dev)
		return;

	priv = platform_get_drvdata(this_dev);
	if (!priv)
		return;

	priv->keyword_type = word_id;

	switch (word_id) {
	case 0:
		snprintf(keyword_buf, sizeof(keyword_buf),
			"VOICE_WAKEUP_WORD_ID=LPSD");
		break;
	case 1:
	case 2:
		snprintf(keyword_buf, sizeof(keyword_buf),
			"VOICE_WAKEUP_WORD_ID=%x", priv->keyword_type);
		break;
	default:
		dev_err(&this_dev->dev, "Invalid word_id =%d", word_id);
		return;
	}

	envp[0] = keyword_buf;
	envp[1] = NULL;

	dev_info(&this_dev->dev, "%s : raise the uevent, string = %s\n",
			__func__, keyword_buf);

	wake_lock_timeout(&priv->wake_lock, 5000);
	kobject_uevent_env(&this_dev->dev.kobj, KOBJ_CHANGE, envp);

	return;
}

static int samsung_dbmdx_probe(struct platform_device *pdev)
{
	struct samsung_dbmdx_event_priv *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev,
			sizeof(struct samsung_dbmdx_event_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* TO DO
	 * It need dbmd patches to register callback func */
	dbmdx_remote_register_event_callback(samsung_dbmdx_voicewakeup_cb);

	wake_lock_init(&priv->wake_lock, WAKE_LOCK_SUSPEND,
				"samsung-voicewakeup");

	priv->svoice_class = class_create(THIS_MODULE, "svoice");
	ret = IS_ERR(priv->svoice_class);
	if (ret) {
		pr_err("Failed to create class(svoice)!\n");
		goto err_class_create;
	}

	priv->keyword_dev = device_create(priv->svoice_class,
			NULL, 0, priv, "keyword");
	ret = IS_ERR(priv->keyword_dev);
	if (ret) {
		pr_err("Failed to create device(keyword)!\n");
		goto err_device_create;
	}

	ret = device_create_file(priv->keyword_dev, &dev_attr_keyword_type);
	if (ret < 0) {
		pr_err("Failed to create device file in sysfs entries(%s)!\n",
				dev_attr_keyword_type.attr.name);
		goto err_device_create_file;
	}

	this_dev = pdev;
	platform_set_drvdata(pdev, priv);

	return 0;

err_device_create_file:
	device_destroy(priv->svoice_class, 0);
err_device_create:
	class_destroy(priv->svoice_class);
err_class_create:
	wake_lock_destroy(&priv->wake_lock);
	devm_kfree(&pdev->dev, priv);
	return ret;
}

static int samsung_dbmdx_remove(struct platform_device *pdev)
{
	struct samsung_dbmdx_event_priv *priv = platform_get_drvdata(pdev);

	device_remove_file(priv->keyword_dev, &dev_attr_keyword_type);
	device_destroy(priv->svoice_class, 0);
	class_destroy(priv->svoice_class);

	wake_lock_destroy(&priv->wake_lock);
	devm_kfree(&pdev->dev, priv);
	return 0;
}

static const struct of_device_id samsung_dbmdx_of_match[] = {
	{.compatible = "samsung,dbmdx-event",},
	{},
};
MODULE_DEVICE_TABLE(of, samsung_dbmdx_of_match);

static struct platform_driver samsung_dbmdx_event_driver = {
	.driver = {
		.name = "dbmdx-event",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(samsung_dbmdx_of_match),
	},
	.probe = samsung_dbmdx_probe,
	.remove = samsung_dbmdx_remove,
};

module_platform_driver(samsung_dbmdx_event_driver);

MODULE_DESCRIPTION("SAMSUNG DBMDx event driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dbmdx-event");
