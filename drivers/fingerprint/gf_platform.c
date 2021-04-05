#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "gf_common.h"

void gfspi_spi_setup_conf(struct gf_device *gf_dev, u32 speed)
{
	u32 max_speed_hz;

	switch (speed) {
	case 1:
	case 4:
	case 6:
	case 8:
	default:
		max_speed_hz = MAX_BAUD_RATE;
		break;
	}

	gf_dev->spi->mode = SPI_MODE_0;
	gf_dev->spi->max_speed_hz = max_speed_hz;
	gf_dev->spi->bits_per_word = 8;

	gf_dev->current_spi_speed = max_speed_hz;
#ifndef ENABLE_SENSORS_FPRINT_SECURE
	if (spi_setup(gf_dev->spi))
		pr_err("%s, failed to setup spi conf\n", __func__);
#endif
}

#ifndef ENABLE_SENSORS_FPRINT_SECURE
int gfspi_spi_read_bytes(struct gf_device *gf_dev, u16 addr,
		u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 *tmp_buf = NULL;

	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
	if (xfer == NULL)
		return -ENOMEM;

	tmp_buf = gf_dev->spi_buffer;

	spi_message_init(&msg);
	*tmp_buf = 0xF0;
	*(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	*(tmp_buf + 2) = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tmp_buf;
	xfer[0].len = 3;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	spi_message_init(&msg);
	/* memset((tmp_buf + 4), 0x00, data_len + 1); */
	/* 4 bytes align */
	*(tmp_buf + 4) = 0xF1;
	xfer[1].tx_buf = tmp_buf + 4;
	xfer[1].rx_buf = tmp_buf + 4;
	xfer[1].len = data_len + 1;
	xfer[1].delay_usecs = 5;
	spi_message_add_tail(&xfer[1], &msg);
	spi_sync(gf_dev->spi, &msg);

	memcpy(rx_buf, (tmp_buf + 5), data_len);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

int gfspi_spi_write_bytes(struct gf_device *gf_dev, u16 addr,
		u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;
	u8 *tmp_buf = NULL;

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if (xfer == NULL)
		return -ENOMEM;

	tmp_buf = gf_dev->spi_buffer;

	spi_message_init(&msg);
	*tmp_buf = 0xF0;
	*(tmp_buf + 1) = (u8)((addr >> 8) & 0xFF);
	*(tmp_buf + 2) = (u8)(addr & 0xFF);
	memcpy(tmp_buf + 3, tx_buf, data_len);
	xfer[0].len = data_len + 3;
	xfer[0].tx_buf = tmp_buf;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

int gfspi_spi_read_byte(struct gf_device *gf_dev, u16 addr, u8 *value)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;

	xfer = kzalloc(sizeof(*xfer) * 2, GFP_KERNEL);
	if (xfer == NULL)
		return -ENOMEM;

	spi_message_init(&msg);
	*gf_dev->spi_buffer = 0xF0;
	*(gf_dev->spi_buffer + 1) = (u8)((addr >> 8) & 0xFF);
	*(gf_dev->spi_buffer + 2) = (u8)(addr & 0xFF);

	xfer[0].tx_buf = gf_dev->spi_buffer;
	xfer[0].len = 3;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	spi_message_init(&msg);
	/* 4 bytes align */
	*(gf_dev->spi_buffer + 4) = 0xF1;
	xfer[1].tx_buf = gf_dev->spi_buffer + 4;
	xfer[1].rx_buf = gf_dev->spi_buffer + 4;
	xfer[1].len = 2;
	xfer[1].delay_usecs = 5;
	spi_message_add_tail(&xfer[1], &msg);
	spi_sync(gf_dev->spi, &msg);

	*value = *(gf_dev->spi_buffer + 5);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

int gfspi_spi_write_byte(struct gf_device *gf_dev, u16 addr, u8 value)
{
	struct spi_message msg;
	struct spi_transfer *xfer = NULL;

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if (xfer == NULL)
		return -ENOMEM;

	spi_message_init(&msg);
	*gf_dev->spi_buffer = 0xF0;
	*(gf_dev->spi_buffer + 1) = (u8)((addr >> 8) & 0xFF);
	*(gf_dev->spi_buffer + 2) = (u8)(addr & 0xFF);
	*(gf_dev->spi_buffer + 3) = value;

	xfer[0].tx_buf = gf_dev->spi_buffer;
	xfer[0].len = 3 + 1;
	xfer[0].delay_usecs = 5;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);

	kfree(xfer);
	if (xfer != NULL)
		xfer = NULL;

	return 0;
}

static int gfspi_spi_transfer_raw(struct gf_device *gf_dev, u8 *tx_buf,
		u8 *rx_buf, u32 len)
{
	struct spi_message msg;
	struct spi_transfer xfer;

	spi_message_init(&msg);
	memset(&xfer, 0, sizeof(struct spi_transfer));

	xfer.tx_buf = tx_buf;
	xfer.rx_buf = rx_buf;
	xfer.len = len;
	spi_message_add_tail(&xfer, &msg);
	spi_sync(gf_dev->spi, &msg);

	return 0;
}

int gfspi_ioctl_transfer_raw_cmd(struct gf_device *gf_dev,
		unsigned long arg, unsigned int bufsiz)
{
	struct gf_ioc_transfer_raw ioc_xraw;
	int retval = 0;
#ifdef CONFIG_SENSORS_FINGERPRINT_32BITS_PLATFORM_ONLY
	struct gf_ioc_transfer_raw_32 ioc_xraw_32;
	u64 read_buf_64;
	u64 write_buf_64;
#endif

	do {
		u8 *tx_buf;
		u8 *rx_buf;
		uint32_t len;

#ifdef CONFIG_SENSORS_FINGERPRINT_32BITS_PLATFORM_ONLY
		if (copy_from_user(&ioc_xraw_32, (void __user *)arg,
				sizeof(struct gf_ioc_transfer_raw_32)))
#else
		if (copy_from_user(&ioc_xraw, (void __user *)arg,
				sizeof(struct gf_ioc_transfer_raw)))
#endif
		{
			pr_err("%s: Failed to copy gf_ioc_transfer_raw from user to kernel\n",
					__func__);
			retval = -EFAULT;
			break;
		}

#ifdef CONFIG_SENSORS_FINGERPRINT_32BITS_PLATFORM_ONLY
		read_buf_64 = (u64)ioc_xraw_32.read_buf;
		write_buf_64 = (u64)ioc_xraw_32.write_buf;
		ioc_xraw.read_buf = (u8 *)read_buf_64;
		ioc_xraw.write_buf = (u8 *)write_buf_64;
		ioc_xraw.high_time = ioc_xraw_32.high_time;
		ioc_xraw.len = ioc_xraw_32.len;
		ioc_xraw.low_time = ioc_xraw_32.low_time;
#endif

		if ((ioc_xraw.len > bufsiz) || (ioc_xraw.len == 0)) {
			pr_err("%s: request transfer length larger than maximum buffer\n",
					__func__);
			pr_err("%s: buf max %x, buf_len %x\n", __func__, bufsiz, ioc_xraw.len);
			retval = -EINVAL;
			break;
		}

		if (ioc_xraw.read_buf == NULL || ioc_xraw.write_buf == NULL) {
			pr_err("%s: read buf and write buf can not equal to NULL simultaneously.\n",
					__func__);
			retval = -EINVAL;
			break;
		}

		/* change speed and set transfer mode */
		gfspi_spi_setup_conf(gf_dev, ioc_xraw.high_time);

		len = ioc_xraw.len;

		tx_buf = kzalloc(len, GFP_KERNEL);
		if (tx_buf == NULL) {
			pr_err("%s: failed to allocate tx buffer\n",
					__func__);
			retval = -EMSGSIZE;
			break;
		}

		rx_buf = kzalloc(len, GFP_KERNEL);
		if (rx_buf == NULL) {
			kfree(tx_buf);
			pr_err("%s: failed to allocate rx buffer\n",
					__func__);
			retval = -EMSGSIZE;
			break;
		}

		if (copy_from_user(tx_buf, (void __user *)ioc_xraw.write_buf,
					ioc_xraw.len)) {
			kfree(tx_buf);
			kfree(rx_buf);
			pr_err("Failed to copy gf_ioc_transfer from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		gfspi_spi_transfer_raw(gf_dev, tx_buf, rx_buf, len);

		if (copy_to_user((void __user *)ioc_xraw.read_buf,
					rx_buf, ioc_xraw.len)) {
			pr_err("Failed to copy gf_ioc_transfer_raw from kernel to user\n");
			retval = -EFAULT;
		}

		kfree(tx_buf);
		kfree(rx_buf);
	} while (0);

	return retval;
}

int gfspi_ioctl_spi_init_cfg_cmd(struct gf_device *gf_dev, unsigned long arg)
{
	return 0;
}
#endif


#ifdef ENABLE_SENSORS_FPRINT_SECURE
static int gfspi_sec_spi_prepare(struct sec_spi_info *spi_info,
		struct spi_device *spi)
{
	struct clk *fp_spi_pclk, *fp_spi_sclk;
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)
	struct clk *fp_spi_dma;
	int ret = 0;
#endif

	fp_spi_pclk = clk_get(NULL, "fp-spi-pclk");
	if (IS_ERR(fp_spi_pclk)) {
		pr_err("%s Can't get fp_spi_pclk\n", __func__);
		return PTR_ERR(fp_spi_pclk);
	}

	fp_spi_sclk = clk_get(NULL, "fp-spi-sclk");
	if (IS_ERR(fp_spi_sclk)) {
		pr_err("%s Can't get fp_spi_sclk\n", __func__);
		return PTR_ERR(fp_spi_sclk);
	}
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)
	fp_spi_dma = clk_get(NULL, "apb_pclk");
	if (IS_ERR(fp_spi_dma)) {
		pr_err("%s Can't get apb_pclk\n", __func__);
		return PTR_ERR(fp_spi_dma);
	}
#endif
	clk_prepare_enable(fp_spi_pclk);
	clk_prepare_enable(fp_spi_sclk);
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)
	ret = clk_prepare_enable(fp_spi_dma);
	if (ret) {
		pr_err("%s clk_finger clk_prepare_enable failed %d\n",
				__func__, ret);
		return ret;
	}
#endif
	clk_set_rate(fp_spi_sclk, spi_info->speed * 2);

	clk_put(fp_spi_pclk);
	clk_put(fp_spi_sclk);
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)
	clk_put(fp_spi_dma);
#endif
	return 0;
}

static int gfspi_sec_spi_unprepare(struct sec_spi_info *spi_info,
		struct spi_device *spi)
{
	struct clk *fp_spi_pclk, *fp_spi_sclk;
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)
	struct clk *fp_spi_dma;
#endif

	fp_spi_pclk = clk_get(NULL, "fp-spi-pclk");
	if (IS_ERR(fp_spi_pclk)) {
		pr_err("%s Can't get fp_spi_pclk\n", __func__);
		return PTR_ERR(fp_spi_pclk);
	}

	fp_spi_sclk = clk_get(NULL, "fp-spi-sclk");
	if (IS_ERR(fp_spi_sclk)) {
		pr_err("%s Can't get fp_spi_sclk\n", __func__);
		return PTR_ERR(fp_spi_sclk);
	}
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)
	fp_spi_dma = clk_get(NULL, "apb_pclk");
	if (IS_ERR(fp_spi_dma)) {
		pr_err("%s Can't get apb_pclk\n", __func__);
		return PTR_ERR(fp_spi_dma);
	}
#endif
	clk_disable_unprepare(fp_spi_pclk);
	clk_disable_unprepare(fp_spi_sclk);
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)
	clk_disable_unprepare(fp_spi_dma);
#endif
	clk_put(fp_spi_pclk);
	clk_put(fp_spi_sclk);
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)
	clk_put(fp_spi_dma);
#endif

	return 0;
}
#endif

/*GPIO pins reference.*/
int gfspi_get_gpio_dts_info(struct device *dev, struct gf_device *gf_dev)
{
	struct device_node *np = dev->of_node;
	int status = 0;

    /*get pwr resource*/
	gf_dev->pwr_gpio = of_get_named_gpio(np, "goodix,gpio_pwr", 0);
	if (!gpio_is_valid(gf_dev->pwr_gpio)) {
		pr_err("%s, PWR GPIO is invalid.\n", __func__);
		return -1;
	}
	pr_info("%s, goodix_pwr:%d\n", __func__, gf_dev->pwr_gpio);
	status = gpio_request(gf_dev->pwr_gpio, "goodix_pwr");
	if (status < 0) {
		pr_err("%s, Failed to request PWR GPIO. rc = %d\n",
				__func__, status);
		return status;
	}
	gpio_direction_output(gf_dev->pwr_gpio, 0);

    /*get reset resource*/
	gf_dev->reset_gpio = of_get_named_gpio(np, "goodix,gpio_reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_err("%s, RESET GPIO is invalid.\n", __func__);
		return -1;
	}
	pr_info("%s, goodix_reset:%d\n",
			__func__, gf_dev->reset_gpio);
	status = gpio_request(gf_dev->reset_gpio, "goodix_reset");
	if (status < 0) {
		pr_err("%s, Failed to request RESET GPIO. rc = %d\n",
				__func__, status);
		return status;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);

    /*get irq resourece*/
	gf_dev->irq_gpio = of_get_named_gpio(np, "goodix,gpio_irq", 0);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_err("%s, IRQ GPIO is invalid.\n", __func__);
		return -1;
	}
	pr_info("%s, irq_gpio:%d\n", __func__, gf_dev->irq_gpio);
	status = gpio_request(gf_dev->irq_gpio, "goodix_irq");
	if (status < 0) {
		pr_err("%s, Failed to request IRQ GPIO. rc = %d\n",
				__func__, status);
		return status;
	}
	gpio_direction_input(gf_dev->irq_gpio);

	if (of_property_read_string_index(np, "goodix,chip_id", 0,
			(const char **)&gf_dev->chipid))
		gf_dev->chipid = '\0';
	pr_info("%s, Chip ID:%s\n", __func__, gf_dev->chipid);

	gf_dev->p = pinctrl_get_select_default(dev);
	if (IS_ERR(gf_dev->p)) {
		status = -EINVAL;
		pr_err("%s: failed pinctrl_get\n", __func__);
		goto fail_pinctrl_get;
	}

#if !defined(ENABLE_SENSORS_FPRINT_SECURE) || defined(DISABLED_GPIO_PROTECTION)
	gf_dev->pins_poweroff = pinctrl_lookup_state(gf_dev->p, "pins_poweroff");
#else
	gf_dev->pins_poweroff = pinctrl_lookup_state(gf_dev->p, "pins_poweroff_tz");
#endif
	if (IS_ERR(gf_dev->pins_poweroff)) {
		pr_err("%s : could not get pins sleep_state (%li)\n",
			__func__, PTR_ERR(gf_dev->pins_poweroff));
		pinctrl_put(gf_dev->p);
		status = -EINVAL;
		goto fail_pinctrl_get;
	}

#if !defined(ENABLE_SENSORS_FPRINT_SECURE) || defined(DISABLED_GPIO_PROTECTION)
	gf_dev->pins_poweron = pinctrl_lookup_state(gf_dev->p, "pins_poweron");
#else
	gf_dev->pins_poweron = pinctrl_lookup_state(gf_dev->p, "pins_poweron_tz");
#endif
	if (IS_ERR(gf_dev->pins_poweron)) {
		pr_err("%s : could not get pins idle_state (%li)\n",
			__func__, PTR_ERR(gf_dev->pins_poweron));
		pinctrl_put(gf_dev->p);
		goto fail_pinctrl_get;
	}

	if (of_property_read_u32(np, "goodix,orient", &gf_dev->orient))
		gf_dev->orient = 0;
	pr_info("%s: orient=%d\n", __func__, gf_dev->orient);

fail_pinctrl_get:
	return status;
}

void gfspi_cleanup_info(struct gf_device *gf_dev)
{
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_debug("%s, remove irq_gpio.\n", __func__);
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_debug("%s, remove reset_gpio.\n", __func__);
	}
	if (gpio_is_valid(gf_dev->pwr_gpio)) {
		gpio_free(gf_dev->pwr_gpio);
		pr_debug("%s, remove pwr_gpio.\n", __func__);
	}
}

int gfspi_spi_clk_enable(struct gf_device *gf_dev)
{
	int ret_val = 0;
#ifdef ENABLE_SENSORS_FPRINT_SECURE
	struct spi_device *spidev = NULL;
	struct sec_spi_info *spi_info = NULL;

	spin_lock_irq(&gf_dev->spi_lock);
	spidev = spi_dev_get(gf_dev->spi);
	spin_unlock_irq(&gf_dev->spi_lock);

	if (!gf_dev->enabled_clk) {
		spi_info = kmalloc(sizeof(struct sec_spi_info), GFP_KERNEL);
		if (spi_info != NULL) {
			spi_info->speed = spidev->max_speed_hz;
			ret_val = gfspi_sec_spi_prepare(spi_info, spidev);
			if (ret_val < 0)
				pr_err("%s: Unable to enable spi clk\n",
					__func__);
			else
				pr_info("%s ENABLE_SPI_CLOCK %ld\n",
					__func__, spi_info->speed);
			kfree(spi_info);
			wake_lock(&gf_dev->wake_lock);
			gf_dev->enabled_clk = true;
		} else {
			ret_val = -ENOMEM;
		}
	}
	spi_dev_put(spidev);
#endif
	return ret_val;
}

int gfspi_spi_clk_disable(struct gf_device *gf_dev)
{
	int ret_val = 0;
#ifdef ENABLE_SENSORS_FPRINT_SECURE
	struct spi_device *spidev = NULL;
	struct sec_spi_info *spi_info = NULL;

	if (gf_dev->enabled_clk) {
		spin_lock_irq(&gf_dev->spi_lock);
		spidev = spi_dev_get(gf_dev->spi);
		spin_unlock_irq(&gf_dev->spi_lock);
		ret_val = gfspi_sec_spi_unprepare(spi_info, spidev);
		if (ret_val < 0)
			pr_err("%s: couldn't disable spi clks\n", __func__);
		spi_dev_put(spidev);
		wake_unlock(&gf_dev->wake_lock);
		gf_dev->enabled_clk = false;
		pr_info("%s, clk disalbed\n", __func__);
	}
#endif
	return ret_val;
}

int gfspi_pin_control(struct gf_device *gf_dev, bool pin_set)
{
	int status = 0;

	if (pin_set) {
		if (!IS_ERR(gf_dev->pins_poweron)) {
			status = pinctrl_select_state(gf_dev->p,
				gf_dev->pins_poweron);
			if (status)
				pr_err("%s: can't set pin default state\n",
					__func__);
			pr_debug("%s idle\n", __func__);
		}
	} else {
		if (!IS_ERR(gf_dev->pins_poweroff)) {
			status = pinctrl_select_state(gf_dev->p,
				gf_dev->pins_poweroff);
			if (status)
				pr_err("%s: can't set pin sleep state\n",
					__func__);
			pr_debug("%s sleep\n", __func__);
		}
	}
	return status;
}
