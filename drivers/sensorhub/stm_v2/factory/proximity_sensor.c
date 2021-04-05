#include "../ssp.h"
#include "ssp_factory.h"


/*************************************************************************/
/* factory Sysfs                                                         */
/*************************************************************************/

static ssize_t proximity_name_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_name(buf);
}

static ssize_t proximity_vendor_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_vendor(buf);
}

static ssize_t proximity_probe_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_probe_status(data, buf);
}

static ssize_t proximity_thresh_high_show(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_threshold_high(data, buf);
}

static ssize_t proximity_thresh_high_store(struct device *dev,
                                           struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->proximity_ops->set_threshold_high(data, buf);
	if (ret < 0) {
		ssp_errf("- failed = %d", ret);
	}

	return size;
}

static ssize_t proximity_thresh_low_show(struct device *dev,
                                         struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_threshold_low(data, buf);
}

static ssize_t proximity_thresh_low_store(struct device *dev,
                                          struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->proximity_ops->set_threshold_low(data, buf);
	if (ret < 0) {
		ssp_errf("- failed = %d", ret);
	}

	return size;
}

#if defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
static ssize_t proximity_thresh_detect_high_show(struct device *dev,
                                                 struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_threshold_detect_high(data, buf);
}

static ssize_t proximity_thresh_detect_high_store(struct device *dev,
                                                  struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->proximity_ops->set_threshold_detect_high(data, buf);
	if (ret < 0) {
		ssp_errf("- failed = %d", ret);
	}

	return size;
}

static ssize_t proximity_thresh_detect_low_show(struct device *dev,
                                                struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_threshold_detect_low(data, buf);
}

static ssize_t proximity_thresh_detect_low_store(struct device *dev,
                                                 struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->proximity_ops->set_threshold_detect_low(data, buf);
	if (ret < 0) {
		ssp_errf("- failed = %d", ret);
	}
	return size;
}
#else
static ssize_t proximity_cancel_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_cancel(data, buf);
}

static ssize_t proximity_cancel_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->proximity_ops->set_proximity_cancel(data, buf);
	if (ret < 0) {
		ssp_errf("- failed = %d", ret);
	}
	return size;
}

static ssize_t proximity_cancel_pass_show(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_cancel_pass(data, buf);
}
#endif

static ssize_t proximity_raw_data_show(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	unsigned short raw_data = 0;

	raw_data = data->proximity_ops->get_proximity_raw_data(data);

	return sprintf(buf, "%u\n", raw_data);
}

static ssize_t proximity_default_trim_show(struct device *dev,
                                           struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_trim_value(data, buf);
}

#if defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
static ssize_t proximity_default_trim_check_show(struct device *dev,
                                                 struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_trim_value(data, buf);
}
#endif

static ssize_t proximity_avg_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_avg_raw_data(data, buf);
}

static ssize_t proximity_avg_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->proximity_ops->set_proximity_avg_raw_data(data, buf);
	if (ret < 0) {
		ssp_errf("- failed = %d", ret);
	}
	return size;
}

static ssize_t barcode_emul_enable_show(struct device *dev,
                                        struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->is_barcode_enabled);
}

static ssize_t barcode_emul_enable_store(struct device *dev,
                                         struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int64_t dEnable;
	struct ssp_data *data = dev_get_drvdata(dev);

	ret = kstrtoll(buf, 10, &dEnable);
	if (ret < 0) {
		return ret;
	}

	if (dEnable) {
		set_proximity_barcode_enable(data, true);
	} else {
		set_proximity_barcode_enable(data, false);
	}

	return size;
}

#if defined(CONFIG_SSP_ENG_DEBUG)
static ssize_t proximity_setting_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	return data->proximity_ops->get_proximity_setting(buf);
}

static ssize_t proximity_setting_store(struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = data->proximity_ops->set_proximity_setting(data, buf);
	if (ret < 0) {
		ssp_errf("- failed = %d", ret);
	}
	return size;
}
#endif

static DEVICE_ATTR(name, S_IRUGO, proximity_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, proximity_vendor_show, NULL);
static DEVICE_ATTR(prox_probe, S_IRUGO, proximity_probe_show, NULL);
static DEVICE_ATTR(thresh_high, S_IRUGO | S_IWUSR | S_IWGRP,
                   proximity_thresh_high_show, proximity_thresh_high_store);

static DEVICE_ATTR(thresh_low, S_IRUGO | S_IWUSR | S_IWGRP,
                   proximity_thresh_low_show, proximity_thresh_low_store);

#if defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
static DEVICE_ATTR(thresh_detect_high, S_IRUGO | S_IWUSR | S_IWGRP,
                   proximity_thresh_detect_high_show, proximity_thresh_detect_high_store);

static DEVICE_ATTR(thresh_detect_low, S_IRUGO | S_IWUSR | S_IWGRP,
                   proximity_thresh_detect_low_show, proximity_thresh_detect_low_store);
#else
static DEVICE_ATTR(prox_cal, S_IRUGO | S_IWUSR | S_IWGRP,
                   proximity_cancel_show, proximity_cancel_store);
static DEVICE_ATTR(prox_offset_pass, S_IRUGO, proximity_cancel_pass_show, NULL);
#endif
static DEVICE_ATTR(prox_trim, S_IRUGO, proximity_default_trim_show, NULL);
static DEVICE_ATTR(raw_data, S_IRUGO, proximity_raw_data_show, NULL);
static DEVICE_ATTR(prox_avg, S_IRUGO | S_IWUSR | S_IWGRP,
                   proximity_avg_show, proximity_avg_store);

static DEVICE_ATTR(barcode_emul_en, S_IRUGO | S_IWUSR | S_IWGRP,
                   barcode_emul_enable_show, barcode_emul_enable_store);

#if defined(CONFIG_SSP_ENG_DEBUG)
static DEVICE_ATTR(setting, S_IRUGO | S_IWUSR | S_IWGRP,
                   proximity_setting_show, proximity_setting_store);
#endif
#if defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
static DEVICE_ATTR(prox_trim_check, S_IRUGO, proximity_default_trim_check_show, NULL);
#endif

static struct device_attribute *prox_attrs[] = {
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_prox_probe,
	&dev_attr_thresh_high,
	&dev_attr_thresh_low,
#if defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
	&dev_attr_thresh_detect_high,
	&dev_attr_thresh_detect_low,
#else
	&dev_attr_prox_cal,
	&dev_attr_prox_offset_pass,
#endif
	&dev_attr_prox_trim,
	&dev_attr_raw_data,
	&dev_attr_prox_avg,
	&dev_attr_barcode_emul_en,
#if defined(CONFIG_SSP_ENG_DEBUG)
	&dev_attr_setting,
#endif
#if defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
	&dev_attr_prox_trim_check,
#endif
	NULL,
};

void initialize_prox_factorytest(struct ssp_data *data)
{
	//function pointer init
#if defined(CONFIG_SENSORS_SSP_PROXIMITY_TMD3700)
	proximity_tmd3700_function_pointer_initialize(data);
#elif defined(CONFIG_SENSORS_SSP_PROXIMITY_AUTO_CAL_TMD3725)
	proximity_tmd3725_auto_cal_function_pointer_initialize(data);
#elif defined(CONFIG_SENSORS_SSP_PROXIMITY_TMD3725)
	proximity_tmd3725_function_pointer_initialize(data);
#else
	proximity_tmg399x_function_pointer_initialize(data);
#endif

	sensors_register(data->devices[SENSOR_TYPE_PROXIMITY], data,
	                 prox_attrs, "proximity_sensor");
}

void remove_prox_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->devices[SENSOR_TYPE_PROXIMITY], prox_attrs);
}

