#ifndef __GF_SPI_DRIVER_H
#define __GF_SPI_DRIVER_H

#include <linux/types.h>
#include <linux/netlink.h>
#include <linux/cdev.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#else
#include <linux/notifier.h>
#endif

#ifdef ENABLE_SENSORS_FPRINT_SECURE
#if defined (CONFIG_ARCH_EXYNOS9) || defined(CONFIG_ARCH_EXYNOS8)\
	|| defined (CONFIG_ARCH_EXYNOS7)
#include <linux/smc.h>
#endif
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spidev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl330.h>
#if defined(CONFIG_SECURE_OS_BOOSTER_API)
#if defined(CONFIG_SOC_EXYNOS7870) || defined(CONFIG_SOC_EXYNOS7880)\
	|| defined(CONFIG_SOC_EXYNOS7570) || defined(CONFIG_SOC_EXYNOS7885)
#include <soc/samsung/secos_booster.h>
#else
#include <mach/secos_booster.h>
#endif
#elif defined(CONFIG_TZDEV_BOOST)
#include <../drivers/misc/tzdev/tz_boost.h>
#endif

struct sec_spi_info {
	int		port;
	unsigned long	speed;
};
#endif
#include <linux/wakelock.h>

/*
 * This feature is temporary for exynos AP only.
 * It's for control GPIO config on enabled TZ before enable GPIO protection.
 * If it's still defined this feature after enable GPIO protection,
 * it will be happened kernel panic
 * So it should be un-defined after enable GPIO protection
 */
#undef DISABLED_GPIO_PROTECTION

#define GF_IOC_MAGIC	'g'

#define GF_GW32J_CHIP_ID	0x00220e
#define GF_GW32N_CHIP_ID	0x002215
#define GF_GW36H_CHIP_ID	0x002504
#define GF_GW36C_CHIP_ID	0x002502

#define MAX_BAUD_RATE 6500000

enum gf_netlink_cmd {
	GF_NETLINK_TEST = 0,
	GF_NETLINK_IRQ = 1,
	GF_NETLINK_SCREEN_OFF,
	GF_NETLINK_SCREEN_ON
};

struct gf_ioc_transfer {
	u8 cmd;    /* spi read = 0, spi  write = 1 */
	u8 reserved;
	u16 addr;
	u32 len;
	u8 *buf;
};

struct gf_ioc_transfer_raw {
	u32 len;
	u8 *read_buf;
	u8 *write_buf;
	uint32_t high_time;
	uint32_t low_time;
};

#ifdef CONFIG_SENSORS_FINGERPRINT_32BITS_PLATFORM_ONLY
struct gf_ioc_transfer_32 {
	u8 cmd;    /* spi read = 0, spi  write = 1 */
	u8 reserved;
	u16 addr;
	u32 len;
	u32 buf;
};

struct gf_ioc_transfer_raw_32 {
	u32 len;
	u32 read_buf;
	u32 write_buf;
	uint32_t high_time;
	uint32_t low_time;
};
#endif

/* define commands */
#define GF_IOC_INIT             _IOR(GF_IOC_MAGIC, 0, u8)
#define GF_IOC_EXIT             _IO(GF_IOC_MAGIC, 1)
#define GF_IOC_RESET            _IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ       _IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ      _IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_SPI_CLK   _IOW(GF_IOC_MAGIC, 5, uint32_t)
#define GF_IOC_DISABLE_SPI_CLK  _IO(GF_IOC_MAGIC, 6)
#define GF_IOC_ENABLE_POWER     _IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER    _IO(GF_IOC_MAGIC, 8)
#define GF_IOC_ENTER_SLEEP_MODE _IO(GF_IOC_MAGIC, 10)
#define GF_IOC_GET_FW_INFO      _IOR(GF_IOC_MAGIC, 11, u8)
#define GF_IOC_REMOVE           _IO(GF_IOC_MAGIC, 12)

/* for SPI REE transfer */
#define GF_IOC_TRANSFER_CMD     _IOWR(GF_IOC_MAGIC, 15, \
		struct gf_ioc_transfer)
#ifndef CONFIG_SENSORS_FINGERPRINT_32BITS_PLATFORM_ONLY
#define GF_IOC_TRANSFER_RAW_CMD _IOWR(GF_IOC_MAGIC, 16, \
		struct gf_ioc_transfer_raw)
#else
#define GF_IOC_TRANSFER_RAW_CMD _IOWR(GF_IOC_MAGIC, 16, \
		struct gf_ioc_transfer_raw_32)
#endif
#ifdef ENABLE_SENSORS_FPRINT_SECURE
#define GF_IOC_SET_SENSOR_TYPE _IOW(GF_IOC_MAGIC, 18, unsigned int)
#endif
#define GF_IOC_POWER_CONTROL _IOW(GF_IOC_MAGIC, 19, unsigned int)
#ifdef ENABLE_SENSORS_FPRINT_SECURE
#define GF_IOC_SPEEDUP			_IOW(GF_IOC_MAGIC, 20, unsigned int)
#define GF_IOC_SET_LOCKSCREEN	_IOW(GF_IOC_MAGIC, 21, unsigned int)
#endif
#define GF_IOC_GET_ORIENT		_IOR(GF_IOC_MAGIC, 22, unsigned int)

#define GF_IOC_MAXNR    23  /* THIS MACRO IS NOT USED NOW... */

struct gf_device {
	dev_t devno;
	struct cdev cdev;
	struct device *fp_device;
	struct class *class;
	struct spi_device *spi;
	int device_count;

	spinlock_t spi_lock;
	struct list_head device_entry;
#ifndef ENABLE_SENSORS_FPRINT_SECURE
	u8 *spi_buffer;
#endif
	struct mutex buf_lock;
	struct mutex release_lock;
	struct sock *nl_sk;
	u8 buf_status;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#else
	struct notifier_block notifier;
#endif

	u8 irq_enabled;
	u8 sig_count;
	u8 system_status;

	u32 pwr_gpio;
	u32 reset_gpio;
	u32 irq_gpio;
	u32 irq;
	u8  need_update;
	/* for netlink use */
	int pid;

	struct work_struct work_debug;
	struct workqueue_struct *wq_dbg;
	struct timer_list dbg_timer;

#ifdef ENABLE_SENSORS_FPRINT_SECURE
	bool enabled_clk;
#endif
	unsigned int current_spi_speed;
	unsigned int orient;
	int sensortype;
	int reset_count;
	int interrupt_count;
	bool ldo_onoff;
	bool tz_mode;
	const char *chipid;
	struct wake_lock wake_lock;

	struct pinctrl *p;
	struct pinctrl_state *pins_poweron;
	struct pinctrl_state *pins_poweroff;
};


int gfspi_get_gpio_dts_info(struct device *dev, struct gf_device *gf_dev);
void gfspi_cleanup_info(struct gf_device *gf_dev);
void gfspi_hw_power_enable(struct gf_device *gf_dev, u8 onoff);
int gfspi_spi_clk_enable(struct gf_device *gf_dev);
int gfspi_spi_clk_disable(struct gf_device *gf_dev);
void gfspi_hw_reset(struct gf_device *gf_dev, u8 delay);
void gfspi_spi_setup_conf(struct gf_device *gf_dev, u32 speed);
int gfspi_pin_control(struct gf_device *gf_dev, bool pin_set);

#ifndef ENABLE_SENSORS_FPRINT_SECURE
int gfspi_spi_read_bytes(struct gf_device *gf_dev, u16 addr,
		u32 data_len, u8 *rx_buf);
int gfspi_spi_write_bytes(struct gf_device *gf_dev, u16 addr,
		u32 data_len, u8 *tx_buf);
int gfspi_spi_read_byte(struct gf_device *gf_dev, u16 addr, u8 *value);
int gfspi_spi_write_byte(struct gf_device *gf_dev, u16 addr, u8 value);
int gfspi_ioctl_transfer_raw_cmd(struct gf_device *gf_dev,
		unsigned long arg, unsigned int bufsiz);
int gfspi_ioctl_spi_init_cfg_cmd(struct gf_device *gf_dev,
		unsigned long arg);
#endif /* !ENABLE_SENSORS_FPRINT_SECURE */

#endif	/* __GF_SPI_DRIVER_H */
