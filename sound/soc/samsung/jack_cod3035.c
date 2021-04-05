/* to Support headset detect function for factory 15 mode. */
static ssize_t earjack_state_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);
    struct cod3035x_jack_det *jackdet = &cod3035x->jack_det;
    int status = jackdet->jack_det;
    int report = 0;

    if (status)
        report = 1;

    return sprintf(buf, "%d\n", report);
}

static ssize_t earjack_state_store(struct device *dev,
                    struct device_attribute *attr, const char *buf, size_t size)
{
    return size;
}

static ssize_t earjack_key_state_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);
    struct cod3035x_jack_det *jackdet = &cod3035x->jack_det;
    int report = 0;

    report = jackdet->button_det ? true : false;

    return sprintf(buf, "%d\n", report);
}

static ssize_t earjack_key_state_store(struct device *dev,
                  struct device_attribute *attr, const char *buf, size_t size)
{
    return size;
}

static ssize_t earjack_select_jack_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    return 0;
}

static ssize_t earjack_select_jack_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);

    if ((!size) || (buf[0] != '1'))
        switch_set_state(&cod3035x->sdev, 0);
    else
        switch_set_state(&cod3035x->sdev, 1);

    return size;
}

static ssize_t earjack_mic_adc_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);
    struct cod3035x_jack_det *jackdet = &cod3035x->jack_det;

    return sprintf(buf, "%d\n", jackdet->adc_val);
}

static ssize_t earjack_mic_adc_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
    return size;
}

#if defined (SEC_SYSFS_ADC_EARJACK)
static ssize_t jack_adc_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);

    int val[4] = {0,};

    val[1] = cod3035x->mic_adc_range -1;
    val[2] = cod3035x->mic_adc_range;
    val[3] = 9999;

    return sprintf(buf, "%d %d %d %d\n",val[0],val[1],val[2],val[3]);
}

static ssize_t hook_adc_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);

    int val[2]  = {0,};

    val[0] =  cod3035x->jack_buttons_zones[0].adc_low;
    val[1] =  cod3035x->jack_buttons_zones[0].adc_high;

    return sprintf(buf, "%d %d\n",val[0],val[1]);
}

static ssize_t voc_ast_adc_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);

    int val[2]  = {0,};

    val[0] =  cod3035x->jack_buttons_zones[1].adc_low;
    val[1] =  cod3035x->jack_buttons_zones[1].adc_high;

    return sprintf(buf, "%d %d\n",val[0],val[1]);
}

static ssize_t volup_adc_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);

    int val[2]  = {0,};

    val[0] =  cod3035x->jack_buttons_zones[2].adc_low;
    val[1] =  cod3035x->jack_buttons_zones[2].adc_high;

    return sprintf(buf, "%d %d\n",val[0],val[1]);
}

static ssize_t voldown_adc_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);

    int val[2]  = {0,};

    val[0] =  cod3035x->jack_buttons_zones[3].adc_low;
    val[1] =  cod3035x->jack_buttons_zones[3].adc_high;

    return sprintf(buf, "%d %d\n",val[0],val[1]);
}
#endif

#if defined (CONFIG_SND_SOC_COD30XX_EXT_ANT) && defined(CONFIG_SEC_FACTORY)
static ssize_t force_enable_antenna_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);

	if ((!size) || (buf[0] != '1')) {
		dev_info(dev, "%s: antenna disble\n", __func__);
		cod3035x->jack_det.ignore_ext_ant = 1;
		if (cod3035x->ant_det_gpio)
			disable_irq(gpio_to_irq(cod3035x->ant_det_gpio));

		cancel_work_sync(&cod3035x->jack_det_work);
		queue_work(cod3035x->jack_det_wq, &cod3035x->jack_det_work);
	} else {
		dev_info(dev, "%s: update antenna enable\n", __func__);
		cod3035x->jack_det.ignore_ext_ant = 0;
		
		cancel_work_sync(&cod3035x->jack_det_work);
		queue_work(cod3035x->jack_det_wq, &cod3035x->jack_det_work);

		/* add delay to enable irq after jack_detect_work called */
		msleep(500);

		if (cod3035x->ant_det_gpio) {
			enable_irq(gpio_to_irq(cod3035x->ant_det_gpio));
		}
	}

	return size;
}

static ssize_t audio_antenna_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct cod3035x_priv *cod3035x = dev_get_drvdata(dev);
	int report = 0;

	if (cod3035x->jack_det.ant_det)
		report = 0xA;
	else if (cod3035x->jack_det.mic_det)
		report = 0x1;
	else
		report = 0;

	return snprintf(buf, 4, "%d\n", report);
}
#endif

static DEVICE_ATTR(select_jack, S_IRUGO | S_IWUSR | S_IWGRP,
                        earjack_select_jack_show, earjack_select_jack_store);

static DEVICE_ATTR(key_state, S_IRUGO | S_IWUSR | S_IWGRP,
                        earjack_key_state_show, earjack_key_state_store);

static DEVICE_ATTR(state, S_IRUGO | S_IWUSR | S_IWGRP,
                        earjack_state_show, earjack_state_store);

static DEVICE_ATTR(mic_adc, S_IRUGO | S_IWUSR | S_IWGRP,
                        earjack_mic_adc_show, earjack_mic_adc_store);

#if defined (SEC_SYSFS_ADC_EARJACK)
static DEVICE_ATTR(jacks_adc, S_IRUGO, jack_adc_show, NULL);
static DEVICE_ATTR(send_end_btn_adc, S_IRUGO, hook_adc_show, NULL);
static DEVICE_ATTR(voc_assist_btn_adc, S_IRUGO, voc_ast_adc_show, NULL);
static DEVICE_ATTR(vol_up_btn_adc, S_IRUGO, volup_adc_show, NULL);
static DEVICE_ATTR(vol_down_btn_adc, S_IRUGO, voldown_adc_show, NULL);
#endif

#if defined (CONFIG_SND_SOC_COD30XX_EXT_ANT) && defined(CONFIG_SEC_FACTORY)
static DEVICE_ATTR(force_enable_antenna, S_IRUGO | S_IWUSR | S_IWGRP,
			NULL, force_enable_antenna_store);
static DEVICE_ATTR(antenna_state, S_IRUGO | S_IWUSR | S_IWGRP,
			audio_antenna_state_show, NULL);
#endif

static void create_jack_devices(struct cod3035x_priv *info)
{
    static struct class *jack_class;
    static struct device *jack_dev;

    jack_class = class_create(THIS_MODULE, "audio");

    if (IS_ERR(jack_class)) {
        pr_err("Failed to create class\n");
    }
    jack_dev = device_create(jack_class, NULL, 0, info, "earjack");

    if (device_create_file(jack_dev, &dev_attr_select_jack) < 0) {
        pr_err("Failed to create (%s)\n", dev_attr_select_jack.attr.name);
    }

    if (device_create_file(jack_dev, &dev_attr_key_state) < 0){
        pr_err("Failed to create (%s)\n", dev_attr_key_state.attr.name);
    }

    if (device_create_file(jack_dev, &dev_attr_state) < 0){
        pr_err("Failed to create (%s)\n", dev_attr_state.attr.name);
    }

    if (device_create_file(jack_dev, &dev_attr_mic_adc) < 0){
        pr_err("Failed to create (%s)\n", dev_attr_mic_adc.attr.name);
    }

#if defined (SEC_SYSFS_ADC_EARJACK)
    if (device_create_file(jack_dev, &dev_attr_jacks_adc) < 0){
        pr_err("Failed to create (%s)\n", dev_attr_jacks_adc.attr.name);
    }

    if (device_create_file(jack_dev, &dev_attr_send_end_btn_adc) < 0){
        pr_err("Failed to create (%s)\n", dev_attr_send_end_btn_adc.attr.name);
    }

    if (device_create_file(jack_dev, &dev_attr_voc_assist_btn_adc) < 0){
        pr_err("Failed to create (%s)\n", dev_attr_voc_assist_btn_adc.attr.name);
    }

    if (device_create_file(jack_dev, &dev_attr_vol_up_btn_adc) < 0){
        pr_err("Failed to create (%s)\n", dev_attr_vol_up_btn_adc.attr.name);
    }

    if (device_create_file(jack_dev, &dev_attr_vol_down_btn_adc) < 0){
        pr_err("Failed to create (%s)\n", dev_attr_vol_down_btn_adc.attr.name);
    }
#endif

#if defined (CONFIG_SND_SOC_COD30XX_EXT_ANT) && defined(CONFIG_SEC_FACTORY)
	if (device_create_file(jack_dev, &dev_attr_force_enable_antenna) < 0){
		pr_err("Failed to create (%s)\n", dev_attr_force_enable_antenna.attr.name);
	}

	if (device_create_file(jack_dev, &dev_attr_antenna_state) < 0){
		pr_err("Failed to create (%s)\n", dev_attr_antenna_state.attr.name);
	}
#endif
}

