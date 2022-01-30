#include <linux/device.h>
#include <linux/err.h>

/* CAUTION : Do not be declared as external eureka_class  */
static struct class *eureka_class;
static atomic_t eureka_dev;

static int __init eureka_class_create(void)
{
	eureka_class = class_create(THIS_MODULE, "eureka");
	if (IS_ERR(eureka_class)) {
		pr_err("Failed to create class (EUREKA) %ld\n", PTR_ERR(eureka_class));
		return PTR_ERR(eureka_class);
	}
	return 0;
}

struct device *eureka_device_create(void *drvdata, const char *fmt)
{
	struct device *dev;

	if (IS_ERR(eureka_class)) {
		pr_err("Failed to create class (EUREKA) %ld\n", PTR_ERR(eureka_class));
		BUG();
	}

	if (!eureka_class) {
		pr_err("Not yet created class (EUREKA)!\n");
		BUG();
	}

	dev = device_create(eureka_class, NULL, atomic_inc_return(&eureka_dev),
			drvdata, fmt);
	if (IS_ERR(dev))
		pr_err("Failed to create device %s %ld\n", fmt, PTR_ERR(dev));
	else
		pr_debug("%s : %s : %d\n", __func__, fmt, dev->devt);

	return dev;
}
EXPORT_SYMBOL(eureka_device_create);

static int match_name(struct device *dev, const void *data)
{
	const char *name = data;

	return sysfs_streq(name, dev_name(dev));
}
struct device *eureka_device_find(const char *name)
{
	return class_find_device(eureka_class, NULL,
		(void *)name, match_name);
}
EXPORT_SYMBOL(eureka_device_find);

void eureka_device_destroy(dev_t devt)
{
	pr_info("%s : %d\n", __func__, devt);
	device_destroy(eureka_class, devt);
}
EXPORT_SYMBOL(eureka_device_destroy);

arch_initcall_sync(eureka_class_create);
