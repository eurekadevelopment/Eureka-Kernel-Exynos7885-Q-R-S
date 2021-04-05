/****************************************************************************
 *
 * Copyright (c) 2014 - 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/kmod.h>
#include <linux/notifier.h>
#include "scsc_mx_impl.h"
#include "miframman.h"
#include "mifmboxman.h"
#include "mxman.h"
#include "srvman.h"
#include "mxmgmt_transport.h"
#include "gdb_transport.h"
#include "mxconf.h"
#include "fwimage.h"
#include "fwhdr.h"
#include "mxlog.h"
#include "mxlogger.h"
#include "fw_panic_record.h"
#include "panicmon.h"
#include "mxproc.h"
#include "mxlog_transport.h"
#include <scsc/kic/slsi_kic_lib.h>
#include <scsc/scsc_release.h>
#include "scsc_mx.h"
#include <linux/fs.h>
#ifdef CONFIG_SCSC_LOG_COLLECTION
#include <scsc/scsc_log_collector.h>
#endif

#include <scsc/scsc_logring.h>
#ifdef CONFIG_SCSC_WLBTD
#include "scsc_wlbtd.h"
#define SCSC_SCRIPT_MOREDUMP	"moredump"
#define SCSC_SCRIPT_LOGGER_DUMP	"mx_logger_dump.sh"
static struct work_struct	wlbtd_work;
#endif

#define STRING_BUFFER_MAX_LENGTH 128
#define NUMBER_OF_STRING_ARGS 1
#define MX_DRAM_SIZE (4 * 1024 * 1024)
#define MX_FW_RUNTIME_LENGTH (1024 * 1024)
#define WAIT_FOR_FW_TO_START_DELAY_MS 1000
#define MBOX2_MAGIC_NUMBER 0xbcdeedcb
#define MBOX_INDEX_0 0
#define MBOX_INDEX_1 1
#define MBOX_INDEX_2 2
#define MBOX_INDEX_3 3
#ifdef CONFIG_SOC_EXYNOS7570
#define MBOX_INDEX_4 4
#define MBOX_INDEX_5 5
#define MBOX_INDEX_6 6
#define MBOX_INDEX_7 7
#endif

#define SCSC_PANIC_ORIGIN_FW   (0x0 << 15)
#define SCSC_PANIC_ORIGIN_HOST (0x1 << 15)

#define SCSC_PANIC_TECH_WLAN   (0x0 << 13)
#define SCSC_PANIC_TECH_CORE   (0x1 << 13)
#define SCSC_PANIC_TECH_BT     (0x2 << 13)
#define SCSC_PANIC_TECH_UNSP   (0x3 << 13)

#define SCSC_PANIC_ORIGIN_MASK  0x8000
#define SCSC_PANIC_TECH_MASK    0x6000
#define SCSC_PANIC_SUBCODE_MASK_LEGACY 0x0FFF
#define SCSC_PANIC_SUBCODE_MASK 0x7FFF

#define SCSC_R4_V2_MINOR_52 52
#define SCSC_R4_V2_MINOR_53 53

#define MM_HALT_RSP_TIMEOUT_MS 100

static char panic_record_dump[PANIC_RECORD_DUMP_BUFFER_SZ];
static BLOCKING_NOTIFIER_HEAD(firmware_chain);
/**
 * This will be returned as fw version ONLY if Maxwell
 * was never found or was unloaded.
 */
static char saved_fw_build_id[FW_BUILD_ID_SZ] = "Maxwell WLBT unavailable";

static bool allow_unidentified_firmware;
module_param(allow_unidentified_firmware, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(allow_unidentified_firmware, "Allow unidentified firmware");

static bool skip_header;
module_param(skip_header, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(skip_header, "Skip header, assuming unidentified firmware");

static bool crc_check_allow_none = true;
module_param(crc_check_allow_none, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(crc_check_allow_none, "Allow skipping firmware CRC checks if CRC is not present");

static int crc_check_period_ms = 30000;
module_param(crc_check_period_ms, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(crc_check_period_ms, "Time period for checking the firmware CRCs");

static ulong mm_completion_timeout_ms = 2000;
module_param(mm_completion_timeout_ms, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mm_completion_timeout_ms, "Timeout wait_for_mm_msg_start_ind (ms) - default 1000. 0 = infinite");

static bool skip_mbox0_check;
module_param(skip_mbox0_check, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(skip_mbox0_check, "Allow skipping firmware mbox0 signature check");

static uint firmware_startup_flags;
module_param(firmware_startup_flags, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(firmware_startup_flags, "0 = Proceed as normal (default); Bit 0 = 1 - spin at start of CRT0; Other bits reserved = 0");

#ifdef CONFIG_SCSC_CHV_SUPPORT
/* First arg controls chv function */
int chv_run;
module_param(chv_run, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(chv_run, "Run chv f/w: 0 = feature disabled, 1 = for continuous checking, 2 = 1 shot, anything else, undefined");

/* Optional array of args for firmware to interpret when chv_run = 1 */
static unsigned int chv_argv[32];
static int chv_argc;

module_param_array(chv_argv, uint, &chv_argc, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(chv_argv, "Array of up to 32 x u32 args for the CHV firmware when chv_run = 1");
#endif

static bool disable_auto_coredump;
module_param(disable_auto_coredump, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_auto_coredump, "Disable driver automatic coredump");

static bool disable_error_handling;
module_param(disable_error_handling, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_error_handling, "Disable error handling");

#if defined(SCSC_SEP_VERSION) && (SCSC_SEP_VERSION >= 10)
int disable_recovery_handling = 2; /* MEMDUMP_FILE_FOR_RECOVERY : for /sys/wifi/memdump */
#else
/* AOSP */
int disable_recovery_handling = 1; /* Recovery disabled, enable in init.rc, not here. */
#endif
module_param(disable_recovery_handling, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_recovery_handling, "Disable recovery handling");
static bool disable_recovery_from_memdump_file = true;
static int memdump = -1;
static bool disable_recovery_until_reboot;

static uint panic_record_delay = 1;
module_param(panic_record_delay, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(panic_record_delay, "Delay in ms before accessing the panic record");

static bool disable_logger = true;
module_param(disable_logger, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(disable_logger, "Disable launch of user space logger");

/*
 * shared between this module and mgt.c as this is the kobject referring to
 * /sys/wifi directory. Core driver is called 1st we create the directory
 * here and share the kobject, so in mgt.c wifi driver can create
 * /sys/wif/mac_addr using sysfs_create_file api using the kobject
 *
 * If both modules tried to create the dir we were getting kernel panic
 * failure due to kobject associated with dir already existed
 */
static struct kobject *wifi_kobj_ref;
static int refcount;
static ssize_t sysfs_show_memdump(struct kobject *kobj, struct kobj_attribute *attr,
				  char *buf);
static ssize_t sysfs_store_memdump(struct kobject *kobj, struct kobj_attribute *attr,
				   const char *buf, size_t count);
static struct kobj_attribute memdump_attr =
		__ATTR(memdump, 0660, sysfs_show_memdump, sysfs_store_memdump);

/* Retrieve memdump in sysfs global */
static ssize_t sysfs_show_memdump(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  char *buf)
{
	return sprintf(buf, "%d\n", memdump);
}

/* Update memdump in sysfs global */
static ssize_t sysfs_store_memdump(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf,
				   size_t count)
{
	int r;

	r = kstrtoint(buf, 10, &memdump);
	if (r < 0)
		memdump = -1;

	switch (memdump) {
	case 0:
	case 2:
		disable_recovery_from_memdump_file = false;
		break;
	case 3:
	default:
		disable_recovery_from_memdump_file = true;
		break;
	}

	SCSC_TAG_INFO(MXMAN, "memdump: %d\n", memdump);

	return (r == 0) ? count : 0;
}

struct kobject *mxman_wifi_kobject_ref_get(void)
{
	if (refcount++ == 0) {
		/* Create sysfs directory /sys/wifi */
		wifi_kobj_ref = kobject_create_and_add("wifi", NULL);
		kobject_get(wifi_kobj_ref);
		kobject_uevent(wifi_kobj_ref, KOBJ_ADD);
		SCSC_TAG_INFO(MXMAN, "wifi_kobj_ref: 0x%p\n", wifi_kobj_ref);
		WARN_ON(refcount == 0);
	}
	return wifi_kobj_ref;
}
EXPORT_SYMBOL(mxman_wifi_kobject_ref_get);

void mxman_wifi_kobject_ref_put(void)
{
	if (--refcount == 0) {
		kobject_put(wifi_kobj_ref);
		kobject_uevent(wifi_kobj_ref, KOBJ_REMOVE);
		wifi_kobj_ref = NULL;
		WARN_ON(refcount < 0);
	}
}
EXPORT_SYMBOL(mxman_wifi_kobject_ref_put);

/* Register memdump override */
void mxman_create_sysfs_memdump(void)
{
	int r;
	struct kobject *kobj_ref = mxman_wifi_kobject_ref_get();

	SCSC_TAG_INFO(MXMAN, "kobj_ref: 0x%p\n", kobj_ref);

	if (kobj_ref) {
		/* Create sysfs file /sys/wifi/memdump */
		r = sysfs_create_file(kobj_ref, &memdump_attr.attr);
		if (r) {
			/* Failed, so clean up dir */
			SCSC_TAG_ERR(MXMAN, "Can't create /sys/wifi/memdump\n");
			mxman_wifi_kobject_ref_put();
			return;
		}
	} else {
		SCSC_TAG_ERR(MXMAN, "failed to create /sys/wifi directory");
	}
}

/* Unregister memdump override */
void mxman_destroy_sysfs_memdump(void)
{
	if (!wifi_kobj_ref)
		return;

	/* Destroy /sys/wifi/memdump file */
	sysfs_remove_file(wifi_kobj_ref, &memdump_attr.attr);

	/* Destroy /sys/wifi virtual dir */
	mxman_wifi_kobject_ref_put();
}

static int firmware_runtime_flags;
/**
 * This mxman reference is initialized/nullified via mxman_init/deinit
 * called by scsc_mx_create/destroy on module probe/remove.
 */
static struct mxman *active_mxman;
static bool send_fw_config_to_active_mxman(uint32_t fw_runtime_flags);

static int fw_runtime_flags_setter(const char *val, const struct kernel_param *kp)
{
	int ret = -EINVAL;
	uint32_t fw_runtime_flags = 0;

	if (!val)
		return ret;
	ret = kstrtouint(val, 10, &fw_runtime_flags);
	if (!ret) {
		if (send_fw_config_to_active_mxman(fw_runtime_flags))
			firmware_runtime_flags = fw_runtime_flags;
		else
			ret = -EINVAL;
	}
	return ret;
}

/**
 * We don't bother to keep an updated copy of the runtime flags effectively
 * currently set into FW...we should add a new message answer handling both in
 * Kenrel and FW side to be sure and this is just to easy debug at the end.
 */
static struct kernel_param_ops fw_runtime_kops = {
	.set = fw_runtime_flags_setter,
	.get = NULL
};

module_param_cb(firmware_runtime_flags, &fw_runtime_kops, NULL, 0200);
MODULE_PARM_DESC(firmware_runtime_flags,
		 "0 = Proceed as normal (default); nnn = Provides FW runtime flags bitmask: unknown bits will be ignored.");

/**
 * Maxwell Agent Management Messages.
 *
 * TODO: common defn with firmware, generated.
 *
 * The numbers here *must* match the firmware!
 */
enum {
	MM_START_IND = 0,
	MM_HALT_REQ = 1,
	MM_FORCE_PANIC = 2,
	MM_HOST_SUSPEND = 3,
	MM_HOST_RESUME = 4,
	MM_FW_CONFIG = 5,
	MM_HALT_RSP = 6,
} ma_msg;

/**
 * Format of the Maxwell agent messages
 * on the Maxwell management transport stream.
 */
struct ma_msg_packet {

	uint8_t ma_msg; /* Message from ma_msg enum */
	uint32_t arg;	/* Optional arg set by f/w in some to-host messages */
} __packed;

static bool send_fw_config_to_active_mxman(uint32_t fw_runtime_flags)
{
	bool ret = false;
	struct srvman *srvman = NULL;

	SCSC_TAG_INFO(MXMAN, "\n");
	if (!active_mxman) {
		SCSC_TAG_ERR(MXMAN, "Active MXMAN NOT FOUND...cannot send running FW config.\n");
		return ret;
	}

	mutex_lock(&active_mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(active_mxman->mx);
	if (srvman && srvman->error) {
		mutex_unlock(&active_mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return ret;
	}

	if (active_mxman->mxman_state == MXMAN_STATE_STARTED) {
		struct ma_msg_packet message = { .ma_msg = MM_FW_CONFIG,
			.arg = fw_runtime_flags };

		SCSC_TAG_INFO(MXMAN, "MM_FW_CONFIG -  firmware_runtime_flags:%d\n", message.arg);
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(active_mxman->mx),
				MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message,
				sizeof(message));
		ret = true;
	} else {
		SCSC_TAG_INFO(MXMAN, "MXMAN is NOT STARTED...cannot send MM_FW_CONFIG msg.\n");
	}
	mutex_unlock(&active_mxman->mxman_mutex);

	return ret;
}

static void mxman_stop(struct mxman *mxman);
static void print_mailboxes(struct mxman *mxman);
#ifdef CONFIG_SCSC_WLBTD
static int _mx_exec(char *prog, int wait_exec) __attribute__((unused));
#else
static int _mx_exec(char *prog, int wait_exec);
#endif
static int wait_for_mm_msg(struct mxman *mxman, struct completion *mm_msg_completion, ulong timeout_ms)
{
	int r;

	(void)mxman; /* unused */

	if (0 == timeout_ms) {
		/* Zero implies infinite wait */
		r = wait_for_completion_interruptible(mm_msg_completion);
		/* r = -ERESTARTSYS if interrupted, 0 if completed */
		return r;
	}
	r = wait_for_completion_timeout(mm_msg_completion, msecs_to_jiffies(timeout_ms));
	if (r == 0) {
		SCSC_TAG_ERR(MXMAN, "timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int wait_for_mm_msg_start_ind(struct mxman *mxman)
{
	return wait_for_mm_msg(mxman, &mxman->mm_msg_start_ind_completion, mm_completion_timeout_ms);
}

static int wait_for_mm_msg_halt_rsp(struct mxman *mxman)
{
	int r;
	(void)mxman; /* unused */

	if (MM_HALT_RSP_TIMEOUT_MS == 0) {
		/* Zero implies infinite wait */
		r = wait_for_completion_interruptible(&mxman->mm_msg_halt_rsp_completion);
		/* r = -ERESTARTSYS if interrupted, 0 if completed */
		return r;
	}

	r = wait_for_completion_timeout(&mxman->mm_msg_halt_rsp_completion, msecs_to_jiffies(MM_HALT_RSP_TIMEOUT_MS));
	if (r)
		SCSC_TAG_INFO(MXMAN, "Received MM_HALT_RSP from firmware");

	return r;
}

#ifndef CONFIG_SCSC_WLBTD
static int coredump_helper(void)
{
	int r;
	int i;
	static char mdbin[128];

	/* Determine path to moredump helper script */
	r = mx140_exe_path(NULL, mdbin, sizeof(mdbin), "moredump");
	if (r) {
		SCSC_TAG_ERR(MXMAN, "moredump path error\n");
		return r;
	}

	for (i = 0; i < 20; i++) {
		r = _mx_exec(mdbin, UMH_WAIT_PROC);
		if (r != -EBUSY)
			break;
		/* If the usermode helper fails with -EBUSY, the userspace is
		 * likely still frozen from suspend. Back off and retry.
		 */
		SCSC_TAG_INFO(MXMAN, "waiting for userspace to thaw...\n");
		msleep(1000);
	}

	/* Application return codes are in the MSB */
	if (r > 0xffL)
		SCSC_TAG_INFO(MXMAN, "moredump.bin exit(%ld), check syslog\n", (r & 0xff00L) >> 8);

	return r;
}
#endif

static int send_mm_msg_stop_blocking(struct mxman *mxman)
{
	int r;
	struct ma_msg_packet message = { .ma_msg = MM_HALT_REQ };

	mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message, sizeof(message));

	r = wait_for_mm_msg_halt_rsp(mxman);
	if (r) {
		/*
		 * MM_MSG_HALT_RSP is not implemented in all versions of firmware, so don't treat it's non-arrival
		 * as an error
		 */
		SCSC_TAG_INFO(MXMAN, "wait_for_MM_HALT_RSP completed");
	}

	return 0;
}

static char *chip_version(u32 rf_hw_ver)
{
	switch (rf_hw_ver & 0x00ff) {
	default:
		break;
	case 0x00b0:
		if ((rf_hw_ver & 0xff00) > 0x1000)
			return "S610/S611";
		else
			return "S610";
	case 0x00b1:
		return "S612";
	case 0x00b2:
		return "S620";
	case 0x0000:
#ifndef CONFIG_SOC_EXYNOS9610
		return "Error: check if RF chip is present";
#else
		return "Unknown";
#endif
	}
	return "Unknown";
}

/*
 * This function is used in this file and in mxproc.c to generate consistent
 * RF CHIP VERSION string for logging on console and for storing the same
 * in proc/drivers/mxman_info/rf_chip_version file.
 */
int mxman_print_rf_hw_version(struct mxman *mxman, char *buf, const size_t bufsz)
{
	int r;

	r = snprintf(buf, bufsz, "RF_CHIP_VERSION: 0x%04x: %s (0x%02x), EVT%x.%x\n",
				  mxman->rf_hw_ver,
				  chip_version(mxman->rf_hw_ver), (mxman->rf_hw_ver & 0x00ff),
				  ((mxman->rf_hw_ver >> 12) & 0xfU), ((mxman->rf_hw_ver >> 8) & 0xfU));

	return r;
}

static void mxman_print_versions(struct mxman *mxman)
{
	char buf[80];

	memset(buf, '\0', sizeof(buf));

	(void)mxman_print_rf_hw_version(mxman, buf, sizeof(buf));

	SCSC_TAG_INFO(MXMAN, "%s", buf);
	SCSC_TAG_INFO(MXMAN, "WLBT FW: %s\n", mxman->fw_build_id);
	SCSC_TAG_INFO(MXMAN, "WLBT Driver: %d.%d.%d.%d\n",
		SCSC_RELEASE_PRODUCT, SCSC_RELEASE_ITERATION, SCSC_RELEASE_CANDIDATE, SCSC_RELEASE_POINT);
#ifdef CONFIG_SCSC_WLBTD
	scsc_wlbtd_get_and_print_build_type();
#endif
}

/** Receive handler for messages from the FW along the maxwell management transport */
static void mxman_message_handler(const void *message, void *data)
{
	struct mxman        *mxman = (struct mxman *)data;

	/* Forward the message to the applicable service to deal with */
	const struct ma_msg_packet *msg = message;

	switch (msg->ma_msg) {
	case MM_START_IND:
		/* The arg can be used to determine the WLBT/S610 hardware revision */
		SCSC_TAG_INFO(MXMAN, "Received MM_START_IND message from the firmware, arg=0x%04x\n", msg->arg);
		mxman->rf_hw_ver = msg->arg;
		mxman_print_versions(mxman);
		atomic_inc(&mxman->boot_count);
		complete(&mxman->mm_msg_start_ind_completion);
		break;
	case MM_HALT_RSP:
		complete(&mxman->mm_msg_halt_rsp_completion);
		SCSC_TAG_INFO(MXMAN, "Received MM_HALT_RSP message from the firmware");
		break;
	default:
		/* HERE: Unknown message, raise fault */
		SCSC_TAG_WARNING(MXMAN, "Received unknown message from the firmware: msg->ma_msg=%d\n", msg->ma_msg);
		break;
	}
}

/*
 * This function calulates and checks two or three (depending on crc32_over_binary flag)
 * crc32 values in the firmware header. The function will check crc32 over the firmware binary
 * (i.e. everything in the file following the header) only if the crc32_over_binary is set to 'true'.
 * This includes initialised data regions so it can be used to check when loading but will not be
 * meaningful once execution starts.
 */
static int do_fw_crc32_checks(char *fw, u32 fw_image_size, struct fwhdr *fwhdr, bool crc32_over_binary)
{
	int r;

	if ((fwhdr->fw_crc32 == 0 || fwhdr->header_crc32 == 0 || fwhdr->const_crc32 == 0) && crc_check_allow_none == 0) {
		SCSC_TAG_ERR(MXMAN, "error: CRC is missing fw_crc32=%d header_crc32=%d crc_check_allow_none=%d\n",
			     fwhdr->fw_crc32, fwhdr->header_crc32, crc_check_allow_none);
		return -EINVAL;
	}

	if (fwhdr->header_crc32 == 0 && crc_check_allow_none == 1) {
		SCSC_TAG_INFO(MXMAN, "Skipping CRC check header_crc32=%d crc_check_allow_none=%d\n",
			      fwhdr->header_crc32, crc_check_allow_none);
	} else {
		/*
		 * CRC-32-IEEE of all preceding header fields (including other CRCs).
		 * Always the last word in the header.
		 */
		r = fwimage_check_fw_header_crc(fw, fwhdr->hdr_length, fwhdr->header_crc32);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "fwimage_check_fw_header_crc() failed\n");
			return r;
		}
	}

	if (fwhdr->const_crc32 == 0 && crc_check_allow_none == 1) {
		SCSC_TAG_INFO(MXMAN, "Skipping CRC check const_crc32=%d crc_check_allow_none=%d\n",
			      fwhdr->const_crc32, crc_check_allow_none);
	} else {
		/*
		 * CRC-32-IEEE over the constant sections grouped together at start of firmware binary.
		 * This CRC should remain valid during execution. It can be used by run-time checker on
		 * host to detect firmware corruption (not all memory masters are subject to MPUs).
		 */
		r = fwimage_check_fw_const_section_crc(fw, fwhdr->const_crc32, fwhdr->const_fw_length, fwhdr->hdr_length);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "fwimage_check_fw_const_section_crc() failed\n");
			return r;
		}
	}

	if (crc32_over_binary) {
		if (fwhdr->fw_crc32 == 0 && crc_check_allow_none == 1)
			SCSC_TAG_INFO(MXMAN, "Skipping CRC check fw_crc32=%d crc_check_allow_none=%d\n",
				      fwhdr->fw_crc32, crc_check_allow_none);
		else {
			/*
			 * CRC-32-IEEE over the firmware binary (i.e. everything
			 * in the file following this header).
			 * This includes initialised data regions so it can be used to
			 * check when loading but will not be meaningful once execution starts.
			 */
			r = fwimage_check_fw_crc(fw, fw_image_size, fwhdr->hdr_length, fwhdr->fw_crc32);
			if (r) {
				SCSC_TAG_ERR(MXMAN, "fwimage_check_fw_crc() failed\n");
				return r;
			}
		}
	}

	return 0;
}


static void fw_crc_wq_start(struct mxman *mxman)
{
	if (mxman->check_crc && crc_check_period_ms)
		queue_delayed_work(mxman->fw_crc_wq, &mxman->fw_crc_work, msecs_to_jiffies(crc_check_period_ms));
}


static void fw_crc_work_func(struct work_struct *work)
{
	int          r;
	struct mxman *mxman = container_of((struct delayed_work *)work, struct mxman, fw_crc_work);

	r = do_fw_crc32_checks(mxman->fw, mxman->fw_image_size, &mxman->fwhdr, false);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "do_fw_crc32_checks() failed r=%d\n", r);
		mxman_fail(mxman, SCSC_PANIC_CODE_HOST << 15, __func__);
		return;
	}
	fw_crc_wq_start(mxman);
}


static void fw_crc_wq_init(struct mxman *mxman)
{
	mxman->fw_crc_wq = create_singlethread_workqueue("fw_crc_wq");
	INIT_DELAYED_WORK(&mxman->fw_crc_work, fw_crc_work_func);
}

static void fw_crc_wq_stop(struct mxman *mxman)
{
	mxman->check_crc = false;
	cancel_delayed_work(&mxman->fw_crc_work);
	flush_workqueue(mxman->fw_crc_wq);
}

static void fw_crc_wq_deinit(struct mxman *mxman)
{
	fw_crc_wq_stop(mxman);
	destroy_workqueue(mxman->fw_crc_wq);
}

static int transports_init(struct mxman *mxman)
{
	struct mxconf  *mxconf;
	int            r;
	struct scsc_mx *mx = mxman->mx;

	/* Initialise mx management stack */
	r = mxmgmt_transport_init(scsc_mx_get_mxmgmt_transport(mx), mx);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "mxmgmt_transport_init() failed %d\n", r);
		return r;
	}

	/* Initialise gdb transport for cortex-R4 */
	r = gdb_transport_init(scsc_mx_get_gdb_transport_r4(mx), mx, GDB_TRANSPORT_R4);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "gdb_transport_init() failed %d\n", r);
		mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mx));
		return r;
	}

	/* Initialise gdb transport for cortex-M4 */
	r = gdb_transport_init(scsc_mx_get_gdb_transport_m4(mx), mx, GDB_TRANSPORT_M4);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "gdb_transport_init() failed %d\n", r);
		gdb_transport_release(scsc_mx_get_gdb_transport_r4(mx));
		mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mx));
		return r;
	}
	/* Initialise mxlog transport */
	r = mxlog_transport_init(scsc_mx_get_mxlog_transport(mx), mx);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "mxlog_transport_init() failed %d\n", r);
		gdb_transport_release(scsc_mx_get_gdb_transport_m4(mx));
		gdb_transport_release(scsc_mx_get_gdb_transport_r4(mx));
		mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mx));
		return r;
	}

	/*
	 * Allocate & Initialise Infrastructre Config Structure
	 * including the mx management stack config information.
	 */
	mxconf = miframman_alloc(scsc_mx_get_ramman(mx), sizeof(struct mxconf), 4, MIFRAMMAN_OWNER_COMMON);
	if (!mxconf) {
		SCSC_TAG_ERR(MXMAN, "miframman_alloc() failed\n");
		gdb_transport_release(scsc_mx_get_gdb_transport_m4(mx));
		gdb_transport_release(scsc_mx_get_gdb_transport_r4(mx));
		mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mx));
		mxlog_transport_release(scsc_mx_get_mxlog_transport(mx));
		return -ENOMEM;
	}
	mxman->mxconf = mxconf;
	mxconf->magic = MXCONF_MAGIC;
	mxconf->version.major = MXCONF_VERSION_MAJOR;
	mxconf->version.minor = MXCONF_VERSION_MINOR;
	/* serialise mxmgmt transport */
	mxmgmt_transport_config_serialise(scsc_mx_get_mxmgmt_transport(mx), &mxconf->mx_trans_conf);
	/* serialise Cortex-R4 gdb transport */
	gdb_transport_config_serialise(scsc_mx_get_gdb_transport_r4(mx), &mxconf->mx_trans_conf_gdb_r4);
	/* serialise Cortex-M4 gdb transport */
	gdb_transport_config_serialise(scsc_mx_get_gdb_transport_m4(mx), &mxconf->mx_trans_conf_gdb_m4);
	/* serialise mxlog transport */
	mxlog_transport_config_serialise(scsc_mx_get_mxlog_transport(mx), &mxconf->mxlogconf);
	SCSC_TAG_DEBUG(MXMAN, "read_bit_idx=%d write_bit_idx=%d buffer=%p num_packets=%d packet_size=%d read_index=%d write_index=%d\n",
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.read_bit_idx,
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.write_bit_idx,
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.buffer,
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.num_packets,
		       scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.packet_size,
		       *scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.read_index,
		       *scsc_mx_get_mxlog_transport(mx)->mif_stream.buffer.write_index
		      );

	return 0;
}

static void transports_release(struct mxman *mxman)
{
	mxlog_transport_release(scsc_mx_get_mxlog_transport(mxman->mx));
	mxmgmt_transport_release(scsc_mx_get_mxmgmt_transport(mxman->mx));
	gdb_transport_release(scsc_mx_get_gdb_transport_r4(mxman->mx));
	gdb_transport_release(scsc_mx_get_gdb_transport_m4(mxman->mx));
	miframman_free(scsc_mx_get_ramman(mxman->mx), mxman->mxconf);
}

static void mbox_init(struct mxman *mxman, u32 firmware_entry_point)
{
	u32                 *mbox0;
	u32                 *mbox1;
	u32                 *mbox2;
	u32                 *mbox3;
	scsc_mifram_ref     mifram_ref;
	struct scsc_mx      *mx = mxman->mx;
	struct scsc_mif_abs *mif = scsc_mx_get_mif_abs(mxman->mx);

	/* Place firmware entry address in MIF MBOX 0 so R4 ROM knows where to jump to! */
	mbox0 = mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif, MBOX_INDEX_0);
	mbox1 = mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif, MBOX_INDEX_1);

	/* Write (and flush) entry point to MailBox 0, config address to MBOX 1 */
	*mbox0 = firmware_entry_point;
	mif->get_mifram_ref(mif, mxman->mxconf, &mifram_ref);
	*mbox1 = mifram_ref; /* must be R4-relative address here */
	/* CPU memory barrier */
	wmb();
	/*
	 * write the magic number "0xbcdeedcb" to MIF Mailbox #2 &
	 * copy the firmware_startup_flags to MIF Mailbox #3 before starting (reset = 0) the R4
	 */
	mbox2 = mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif, MBOX_INDEX_2);
	*mbox2 = MBOX2_MAGIC_NUMBER;
	mbox3 = mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif, MBOX_INDEX_3);
	*mbox3 = firmware_startup_flags;
}

static int fwhdr_init(char *fw, struct fwhdr *fwhdr, bool *fwhdr_parsed_ok, bool *check_crc)
{
	/*
	 * Validate the fw image including checking the firmware header, majic #, version, checksum  so on
	 * then do CRC on the entire image
	 *
	 * Derive some values from header -
	 *
	 * PORT: assumes little endian
	 */
	if (skip_header)
		*fwhdr_parsed_ok = false; /* Allows the forced start address to be used */
	else
		*fwhdr_parsed_ok = fwhdr_parse(fw, fwhdr);
	*check_crc = false;
	if (*fwhdr_parsed_ok) {
		SCSC_TAG_INFO(MXMAN, "FW HEADER version: hdr_major: %d hdr_minor: %d\n", fwhdr->hdr_major, fwhdr->hdr_minor);
		switch (fwhdr->hdr_major) {
		case 0:
			switch (fwhdr->hdr_minor) {
			case 2:
				*check_crc = true;
				break;
			default:
				SCSC_TAG_ERR(MXMAN, "Unsupported FW HEADER version: hdr_major: %d hdr_minor: %d\n",
					fwhdr->hdr_major, fwhdr->hdr_minor);
				return -EINVAL;
			}
			break;
		case 1:
			*check_crc = true;
			break;
		default:
			SCSC_TAG_ERR(MXMAN, "Unsupported FW HEADER version: hdr_major: %d hdr_minor: %d\n",
				fwhdr->hdr_major, fwhdr->hdr_minor);
			return -EINVAL;
		}
		switch (fwhdr->fwapi_major) {
		case 0:
			switch (fwhdr->fwapi_minor) {
			case 2:
				SCSC_TAG_INFO(MXMAN, "FWAPI version: fwapi_major: %d fwapi_minor: %d\n",
					fwhdr->fwapi_major, fwhdr->fwapi_minor);
				break;
			default:
				SCSC_TAG_ERR(MXMAN, "Unsupported FWAPI version: fwapi_major: %d fwapi_minor: %d\n",
					fwhdr->fwapi_major, fwhdr->fwapi_minor);
				return -EINVAL;
			}
			break;
		default:
			SCSC_TAG_ERR(MXMAN, "Unsupported FWAPI version: fwapi_major: %d fwapi_minor: %d\n",
				fwhdr->fwapi_major, fwhdr->fwapi_minor);
			return -EINVAL;
		}
	} else {
		/* This is unidetified pre-header firmware - assume it is built to run at 0xb8000000 == 0 for bootrom */
		if (allow_unidentified_firmware) {
			SCSC_TAG_INFO(MXMAN, "Unidentified firmware override\n");
			fwhdr->firmware_entry_point = 0;
			fwhdr->fw_runtime_length = MX_FW_RUNTIME_LENGTH;
		} else {
			SCSC_TAG_ERR(MXMAN, "Unidentified firmware is not allowed\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int fw_init(struct mxman *mxman, void *start_dram, size_t size_dram, bool *fwhdr_parsed_ok)
{
	int                 r;
	char                *build_id;
	u32                 fw_image_size;
	struct fwhdr        *fwhdr = &mxman->fwhdr;
	char                *fw = start_dram;

	r = mx140_file_download_fw(mxman->mx, start_dram, size_dram, &fw_image_size);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "mx140_file_download_fw() failed (%d)\n", r);
		return r;
	}

	r = fwhdr_init(fw, fwhdr, fwhdr_parsed_ok, &mxman->check_crc);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "fwhdr_init() failed\n");
		return r;
	}
	mxman->fw = fw;
	mxman->fw_image_size = fw_image_size;
	if (mxman->check_crc) {
		/* do CRC on the entire image */
		r = do_fw_crc32_checks(fw, fw_image_size, &mxman->fwhdr, true);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "do_fw_crc32_checks() failed\n");
			return r;
		}
		fw_crc_wq_start(mxman);
	}

	if (*fwhdr_parsed_ok) {
		build_id = fwhdr_get_build_id(fw, fwhdr);
		if (build_id) {
			struct slsi_kic_service_info kic_info;

			(void)snprintf(mxman->fw_build_id, sizeof(mxman->fw_build_id), "%s", build_id);
			SCSC_TAG_INFO(MXMAN, "Firmware BUILD_ID: %s\n", mxman->fw_build_id);
			memcpy(saved_fw_build_id, mxman->fw_build_id,
			       sizeof(saved_fw_build_id));

			(void) snprintf(kic_info.ver_str,
					min(sizeof(mxman->fw_build_id), sizeof(kic_info.ver_str)),
					"%s", mxman->fw_build_id);
			kic_info.fw_api_major = fwhdr->fwapi_major;
			kic_info.fw_api_minor = fwhdr->fwapi_minor;
			kic_info.release_product = SCSC_RELEASE_PRODUCT;
			kic_info.host_release_iteration = SCSC_RELEASE_ITERATION;
			kic_info.host_release_candidate = SCSC_RELEASE_CANDIDATE;

			slsi_kic_service_information(slsi_kic_technology_type_common, &kic_info);
		} else
			SCSC_TAG_ERR(MXMAN, "Failed to get Firmware BUILD_ID\n");
	}

	SCSC_TAG_DEBUG(MXMAN, "firmware_entry_point=0x%x fw_runtime_length=%d\n", fwhdr->firmware_entry_point, fwhdr->fw_runtime_length);

	return 0;

}

static int mxman_start(struct mxman *mxman)
{
	void                *start_dram;
	size_t              size_dram = MX_DRAM_SIZE;
	struct scsc_mif_abs *mif;
	struct fwhdr        *fwhdr = &mxman->fwhdr;
	bool                fwhdr_parsed_ok;
	void                *start_mifram_heap;
	u32                 length_mifram_heap;
	int                 r;

	(void)snprintf(mxman->fw_build_id, sizeof(mxman->fw_build_id), "unknown");

	/* If the option is set to skip header, we must allow unidentified f/w */
	if (skip_header) {
		SCSC_TAG_INFO(MXMAN, "Ignoring firmware header block\n");
		allow_unidentified_firmware = true;
	}

	mif = scsc_mx_get_mif_abs(mxman->mx);
	start_dram = mif->map(mif, &size_dram);

	if (!start_dram) {
		SCSC_TAG_ERR(MXMAN, "Error allocating dram\n");
		return -ENOMEM;
	}

	SCSC_TAG_DEBUG(MXMAN, "Allocated %zu bytes\n", size_dram);

#ifdef CONFIG_SCSC_CHV_SUPPORT
	if (chv_run)
		allow_unidentified_firmware = true;
	/* Set up chv arguments. */

#endif

	mxman->start_dram = start_dram;

	r = fw_init(mxman, start_dram, size_dram, &fwhdr_parsed_ok);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "fw_init() failed\n");
		mif->unmap(mif, mxman->start_dram);
		return r;
	}

	/* set up memory protection (read only) from start_dram to start_dram+fw_length
	 * rounding up the size if required
	 */
	start_mifram_heap = (char *)start_dram + fwhdr->fw_runtime_length;
	length_mifram_heap = size_dram - fwhdr->fw_runtime_length;

	miframman_init(scsc_mx_get_ramman(mxman->mx), start_mifram_heap, length_mifram_heap, start_dram);
	mifmboxman_init(scsc_mx_get_mboxman(mxman->mx));
	mifintrbit_init(scsc_mx_get_intrbit(mxman->mx), mif);

	/* Initialise transports */
	r = transports_init(mxman);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "transports_init() failed\n");
		fw_crc_wq_stop(mxman);
		mifintrbit_deinit(scsc_mx_get_intrbit(mxman->mx));
		miframman_deinit(scsc_mx_get_ramman(mxman->mx));
		mifmboxman_deinit(scsc_mx_get_mboxman(mxman->mx));
		/* Release the MIF memory resources */
		mif->unmap(mif, mxman->start_dram);
		return r;
	}
	mbox_init(mxman, fwhdr->firmware_entry_point);
	init_completion(&mxman->mm_msg_start_ind_completion);
	init_completion(&mxman->mm_msg_halt_rsp_completion);
	mxmgmt_transport_register_channel_handler(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
						  &mxman_message_handler, mxman);
	mxlog_init(scsc_mx_get_mxlog(mxman->mx), mxman->mx, mxman->fw_build_id);
#ifdef CONFIG_SCSC_MXLOGGER
	mxlogger_init(mxman->mx, scsc_mx_get_mxlogger(mxman->mx), MXL_POOL_SZ);
#endif
#ifdef CONFIG_SCSC_CHV_SUPPORT
	if (chv_run) {
		int i;

		u32 *p = (u32 *)((u8 *)start_dram + SCSC_CHV_ARGV_ADDR_OFFSET);

		if (chv_argc == 0) {
			/*
			 * Setup the chv f/w arguments.
			 * Argument of 0 means run once (driver never set this).
			 * Argument of 1 means run forever.
			 */
			SCSC_TAG_INFO(MXMAN, "Setting up CHV arguments: start_dram=%p arg=%p, chv_run=%d\n", start_dram, p, chv_run);
			*p++ = 1;                    /* argc */
			*p++ = chv_run == 1 ? 0 : 1; /* arg */
		} else {
			/* Pass separate args */
			*p++ = chv_argc;  /* argc */
			SCSC_TAG_INFO(MXMAN, "Setting up additional CHV args: chv_argc = %d\n", chv_argc);

			for (i = 0; i < chv_argc; i++) {
				SCSC_TAG_INFO(MXMAN, "Setting up additional CHV args: chv_argv[%d]: *(%p) = 0x%x\n", i, p, (u32)chv_argv[i]);
				*p++ = (u32)chv_argv[i]; /* arg */
			}
		}
	}
#endif
	mxproc_create_ctrl_proc_dir(&mxman->mxproc, mxman);
	panicmon_init(scsc_mx_get_panicmon(mxman->mx), mxman->mx);

	/* Change state to STARTING to allow coredump as we come out of reset */
	mxman->mxman_state = MXMAN_STATE_STARTING;

	/* release Maxwell from reset */
	r = mif->reset(mif, 0);
	if (r) {
#ifdef CONFIG_SCSC_LOG_COLLECTION
		scsc_log_collector_schedule_collection(SCSC_LOG_HOST_COMMON, SCSC_LOG_HOST_COMMON_REASON_START);
#else
		mx140_log_dump();
#endif
	}
	if (fwhdr_parsed_ok) {
		r = wait_for_mm_msg_start_ind(mxman);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "wait_for_MM_START_IND() failed: r=%d\n", r);
			print_mailboxes(mxman);
			if (skip_mbox0_check) {
				SCSC_TAG_ERR(MXMAN, "timeout ignored in skip_mbox0_check mode\n");
				return 0;
			}
			mxman_stop(mxman);
			return r;
		}
#ifdef CONFIG_SCSC_MXLOGGER
		mxlogger_start(scsc_mx_get_mxlogger(mxman->mx));
#endif
	} else {
		msleep(WAIT_FOR_FW_TO_START_DELAY_MS);
	}

	return 0;
}

static bool is_bug_on_enabled(struct scsc_mx *mx)
{
	bool bug_on_enabled;
	const struct firmware *firm;
	int r;

	if ((memdump == 3) && (disable_recovery_handling == MEMDUMP_FILE_FOR_RECOVERY))
		bug_on_enabled = true;
	else
		bug_on_enabled = false;
#ifdef CONFIG_SCSC_LOG_COLLECTION
	return bug_on_enabled;
#else
	/* non SABLE platforms should also follow /sys/wifi/memdump if enabled */
	if (disable_recovery_handling == MEMDUMP_FILE_FOR_RECOVERY)
		return bug_on_enabled;

	/* for legacy platforms (including Andorid P) using .memdump.info */
#if defined(SCSC_SEP_VERSION) && (SCSC_SEP_VERSION >= 9)
	#define MX140_MEMDUMP_INFO_FILE	"/data/vendor/conn/.memdump.info"
#else
	#define MX140_MEMDUMP_INFO_FILE	"/data/misc/conn/.memdump.info"
#endif

	SCSC_TAG_INFO(MX_FILE, "Loading %s file\n", MX140_MEMDUMP_INFO_FILE);
	r = mx140_request_file(mx, MX140_MEMDUMP_INFO_FILE, &firm);
	if (r) {
		SCSC_TAG_WARNING(MX_FILE, "Error Loading %s file %d\n", MX140_MEMDUMP_INFO_FILE, r);
		return bug_on_enabled;
	}
	if (firm->size < sizeof(char))
		SCSC_TAG_WARNING(MX_FILE, "file is too small\n");
	else if (*firm->data == '3')
		bug_on_enabled = true;
	mx140_release_file(mx, firm);
	SCSC_TAG_INFO(MX_FILE, "bug_on_enabled %d\n", bug_on_enabled);
	return bug_on_enabled;
#endif //CONFIG_SCSC_LOG_COLLECTION
}

static void print_panic_code_legacy(u16 code)
{
	u16 tech = code & SCSC_PANIC_TECH_MASK;
	u16 origin = code & SCSC_PANIC_ORIGIN_MASK;

	SCSC_TAG_INFO(MXMAN, "Decoding panic code=0x%x:\n", code);
	switch (origin) {
	default:
		SCSC_TAG_INFO(MXMAN, "Failed to identify panic origin\n");
		break;
	case SCSC_PANIC_ORIGIN_FW:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_ORIGIN_FW\n");
		break;
	case SCSC_PANIC_ORIGIN_HOST:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_ORIGIN_HOST\n");
		break;
	}

	switch (tech) {
	default:
		SCSC_TAG_INFO(MXMAN, "Failed to identify panic technology\n");
		break;
	case SCSC_PANIC_TECH_WLAN:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_TECH_WLAN\n");
		break;
	case SCSC_PANIC_TECH_CORE:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_TECH_CORE\n");
		break;
	case SCSC_PANIC_TECH_BT:
		SCSC_TAG_INFO(MXMAN, "SCSC_PANIC_TECH_BT\n");
		break;
	case SCSC_PANIC_TECH_UNSP:
		SCSC_TAG_INFO(MXMAN, "PANIC_TECH_UNSP\n");
		break;
	}
	SCSC_TAG_INFO(MXMAN, "panic subcode=0x%x\n", code & SCSC_PANIC_SUBCODE_MASK_LEGACY);
}

static void print_panic_code(u16 code)
{
	u16 origin = code & SCSC_PANIC_ORIGIN_MASK;	/* Panic origin (host/fw) */
	u16 subcode = code & SCSC_PANIC_SUBCODE_MASK;	/* The panic code */

	SCSC_TAG_INFO(MXMAN, "Decoding panic code=0x%x:\n", code);
	SCSC_TAG_INFO(MXMAN, "panic subcode=0x%x\n", code & SCSC_PANIC_SUBCODE_MASK);

	switch (origin) {
	default:
		SCSC_TAG_INFO(MXMAN, "Failed to identify panic origin\n");
		break;
	case SCSC_PANIC_ORIGIN_FW:
		SCSC_TAG_INFO(MXMAN, "WLBT FW PANIC: 0x%02x\n", subcode);
		break;
	case SCSC_PANIC_ORIGIN_HOST:
		SCSC_TAG_INFO(MXMAN, "WLBT HOST detected FW failure, service:\n");
		switch (subcode >> SCSC_SYSERR_HOST_SERVICE_SHIFT) {
		case SCSC_SERVICE_ID_WLAN:
			SCSC_TAG_INFO(MXMAN, " WLAN\n");
			break;
		case SCSC_SERVICE_ID_BT:
			SCSC_TAG_INFO(MXMAN, " BT\n");
			break;
		case SCSC_SERVICE_ID_ANT:
			SCSC_TAG_INFO(MXMAN, " ANT\n");
			break;
		case SCSC_SERVICE_ID_CLK20MHZ:
			SCSC_TAG_INFO(MXMAN, " CLK20MHZ\n");
			break;
		default:
			SCSC_TAG_INFO(MXMAN, " Service 0x%x\n", subcode);
			break;
		}
		break;
	}
}

/**
 * Print the last panic record collected to aid in post mortem.
 *
 * Helps when all we have is kernel log showing WLBT failed some time ago
 *
 * Only prints the R4 record
 */
void mxman_show_last_panic(struct mxman *mxman)
{
	u32 r4_panic_record_length = 0;	/* in u32s */

	/* Any valid panic? */
	if (mxman->scsc_panic_code == 0)
		return;

	SCSC_TAG_INFO(MXMAN, "\n\n--- DETAILS OF LAST WLBT FAILURE ---\n\n");

	switch (mxman->scsc_panic_code & SCSC_PANIC_ORIGIN_MASK) {
	case SCSC_PANIC_ORIGIN_HOST:
		SCSC_TAG_INFO(MXMAN, "Last panic was host induced:\n");
		break;

	case SCSC_PANIC_ORIGIN_FW:
		SCSC_TAG_INFO(MXMAN, "Last panic was FW:\n");
		fw_parse_r4_panic_record(mxman->last_panic_rec_r, &r4_panic_record_length);
		break;

	default:
		SCSC_TAG_INFO(MXMAN, "Last panic unknown origin %d\n", mxman->scsc_panic_code & SCSC_PANIC_ORIGIN_MASK);
		break;
	}

	print_panic_code(mxman->scsc_panic_code);

	SCSC_TAG_INFO(MXMAN, "Reason: '%s'\n", mxman->failure_reason[0] ? mxman->failure_reason : "<null>");
	SCSC_TAG_INFO(MXMAN, "Auto-recovery: %s\n", disable_recovery_handling ? "off" : "on");

	if (mxman_recovery_disabled()) {
		/* Labour the point that a reboot is needed when autorecovery is disabled */
		SCSC_TAG_INFO(MXMAN, "\n\n*** HANDSET REBOOT NEEDED TO RESTART WLAN AND BT ***\n\n");
	}

	SCSC_TAG_INFO(MXMAN, "\n\n--- END DETAILS OF LAST WLBT FAILURE ---\n\n");
}

static void process_panic_record(struct mxman *mxman)
{
	u32 *r4_panic_record = NULL;
	u32 *m4_panic_record = NULL;
	u32 r4_panic_record_length = 0;	/* in u32s */
	u32 m4_panic_record_length = 0; /* in u32s */
	bool r4_panic_record_ok = false;
	bool m4_panic_record_ok = false;
	bool r4_sympathetic_panic_flag = false;
	bool m4_sympathetic_panic_flag = false;

	/* some configurable delay before accessing the panic record */
	msleep(panic_record_delay);
	/*
	* Check if the panic was trigered by MX and set the subcode if so.
	*/
	if ((mxman->scsc_panic_code & SCSC_PANIC_ORIGIN_MASK) == SCSC_PANIC_ORIGIN_FW) {
		if (mxman->fwhdr.r4_panic_record_offset) {
			r4_panic_record = (u32 *)(mxman->fw + mxman->fwhdr.r4_panic_record_offset);
			r4_panic_record_ok = fw_parse_r4_panic_record(r4_panic_record, &r4_panic_record_length);
		} else {
			SCSC_TAG_INFO(MXMAN, "R4 panic record doesn't exist in the firmware header\n");
		}
		if (mxman->fwhdr.m4_panic_record_offset) {
			m4_panic_record = (u32 *)(mxman->fw + mxman->fwhdr.m4_panic_record_offset);
			m4_panic_record_ok = fw_parse_m4_panic_record(m4_panic_record, &m4_panic_record_length);
#ifdef CONFIG_SCSC_MX450_GDB_SUPPORT
		} else if (mxman->fwhdr.m4_1_panic_record_offset) {
			m4_1_panic_record = (u32 *)(mxman->fw + mxman->fwhdr.m4_1_panic_record_offset);
			m4_1_panic_record_ok = fw_parse_m4_panic_record(m4_1_panic_record, &m4_1_panic_record_length);
#endif
		} else {
			SCSC_TAG_INFO(MXMAN, "M4 panic record doesn't exist in the firmware header\n");
		}

		/* Extract and print the panic code */
		switch (r4_panic_record_length) {
		default:
			SCSC_TAG_WARNING(MXMAN, "Bad panic record length/subversion\n");
			break;
		case SCSC_R4_V2_MINOR_52:
			if (r4_panic_record_ok)
				mxman->scsc_panic_code |= SCSC_PANIC_SUBCODE_MASK_LEGACY & r4_panic_record[2];
			else if (m4_panic_record_ok)
				mxman->scsc_panic_code |= SCSC_PANIC_SUBCODE_MASK_LEGACY & m4_panic_record[2];
			/* Set unspecified technology for now */
			mxman->scsc_panic_code |= SCSC_PANIC_TECH_UNSP;
			print_panic_code_legacy(mxman->scsc_panic_code);
			break;
		case SCSC_R4_V2_MINOR_53:
			if (r4_panic_record_ok) {
				/* Save the last R4 panic record for future display */
				BUG_ON(sizeof(mxman->last_panic_rec_r) < SCSC_R4_V2_MINOR_53 * sizeof(u32));
				memcpy((u8 *)mxman->last_panic_rec_r, (u8 *)r4_panic_record, SCSC_R4_V2_MINOR_53 * sizeof(u32));
				mxman->last_panic_rec_sz = r4_panic_record_length;

				r4_sympathetic_panic_flag = fw_parse_get_r4_sympathetic_panic_flag(r4_panic_record);
				SCSC_TAG_INFO(MXMAN, "r4_panic_record_ok=%d r4_sympathetic_panic_flag=%d\n",
						r4_panic_record_ok,
						r4_sympathetic_panic_flag
					);
				if (r4_sympathetic_panic_flag == false) {
					/* process R4 record */
					SCSC_TAG_INFO(MXMAN, "process R4 record\n");
					mxman->scsc_panic_code |= SCSC_PANIC_SUBCODE_MASK & r4_panic_record[3];
					print_panic_code(mxman->scsc_panic_code);
					break;
				}
			}
			if (m4_panic_record_ok) {
				m4_sympathetic_panic_flag = fw_parse_get_m4_sympathetic_panic_flag(m4_panic_record);
				SCSC_TAG_INFO(MXMAN, "m4_panic_record_ok=%d m4_sympathetic_panic_flag=%d\n",
						m4_panic_record_ok,
						m4_sympathetic_panic_flag
					);
				if (m4_sympathetic_panic_flag == false) {
					/* process M4 record */
					SCSC_TAG_INFO(MXMAN, "process M4 record\n");
					mxman->scsc_panic_code |= SCSC_PANIC_SUBCODE_MASK & m4_panic_record[3];
				} else if (r4_panic_record_ok) {
					/* process R4 record */
					SCSC_TAG_INFO(MXMAN, "process R4 record\n");
					mxman->scsc_panic_code |= SCSC_PANIC_SUBCODE_MASK & r4_panic_record[3];
				}
				print_panic_code(mxman->scsc_panic_code);
			}
			break;
		}
	}
}

#define MAX_UHELP_TMO_MS	20000
/*
 * workqueue thread
 */
static void mxman_failure_work(struct work_struct *work)
{
	struct mxman  *mxman = container_of(work, struct mxman, failure_work);
	struct srvman *srvman;
	struct scsc_mx *mx = mxman->mx;
	struct scsc_mif_abs *mif = scsc_mx_get_mif_abs(mxman->mx);
	int used = 0, r = 0;

	wake_lock(&mxman->recovery_wake_lock);

	slsi_kic_system_event(slsi_kic_system_event_category_error,
			      slsi_kic_system_events_subsystem_crashed, GFP_KERNEL);

	blocking_notifier_call_chain(&firmware_chain, SCSC_FW_EVENT_FAILURE, NULL);

	SCSC_TAG_INFO(MXMAN, "Complete mm_msg_start_ind_completion\n");
	complete(&mxman->mm_msg_start_ind_completion);
	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);

	if (mxman->mxman_state != MXMAN_STATE_STARTED && mxman->mxman_state != MXMAN_STATE_STARTING) {
		SCSC_TAG_WARNING(MXMAN, "Not in started state: mxman->mxman_state=%d\n", mxman->mxman_state);
		wake_unlock(&mxman->recovery_wake_lock);
		mutex_unlock(&mxman->mxman_mutex);
		return;
	}

	/**
	 * Set error on mxlog and unregister mxlog msg-handlers.
	 * mxlog ISR and kthread will ignore further messages
	 * but mxlog_thread is NOT stopped here.
	 */
	mxlog_transport_set_error(scsc_mx_get_mxlog_transport(mx));
	mxlog_release(scsc_mx_get_mxlog(mx));
	/* unregister channel handler */
	mxmgmt_transport_register_channel_handler(scsc_mx_get_mxmgmt_transport(mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
						  NULL, NULL);
	mxmgmt_transport_set_error(scsc_mx_get_mxmgmt_transport(mx));
	srvman_set_error(srvman);
	fw_crc_wq_stop(mxman);

	mxman->mxman_state = mxman->mxman_next_state;

	if (mxman->mxman_state != MXMAN_STATE_FAILED
	    && mxman->mxman_state != MXMAN_STATE_FREEZED) {
		WARN_ON(mxman->mxman_state != MXMAN_STATE_FAILED
			&& mxman->mxman_state != MXMAN_STATE_FREEZED);
		SCSC_TAG_ERR(MXMAN, "Bad state=%d\n", mxman->mxman_state);
		wake_unlock(&mxman->recovery_wake_lock);
		mutex_unlock(&mxman->mxman_mutex);
		return;
	}
	/* Signal panic to r4 and m4 processors */
	SCSC_TAG_INFO(MXMAN, "Setting MIFINTRBIT_RESERVED_PANIC_R4\n");
	mif->irq_bit_set(mif, MIFINTRBIT_RESERVED_PANIC_R4, SCSC_MIFINTR_TARGET_R4);
	SCSC_TAG_INFO(MXMAN, "Setting MIFINTRBIT_RESERVED_PANIC_M4\n");
	mif->irq_bit_set(mif, MIFINTRBIT_RESERVED_PANIC_M4, SCSC_MIFINTR_TARGET_M4);
	srvman_freeze_services(srvman);
	if (mxman->mxman_state == MXMAN_STATE_FAILED) {
		mxman->last_panic_time = local_clock();
		process_panic_record(mxman);
		SCSC_TAG_INFO(MXMAN, "Trying to schedule coredump\n");

		SCSC_TAG_INFO(MXMAN, "scsc_release %d.%d.%d.%d\n",
			SCSC_RELEASE_PRODUCT,
			SCSC_RELEASE_ITERATION,
			SCSC_RELEASE_CANDIDATE,
			SCSC_RELEASE_POINT);
		SCSC_TAG_INFO(MXMAN, "Auto-recovery: %s\n", mxman_recovery_disabled() ? "off" : "on");
#ifdef CONFIG_SCSC_WLBTD
		scsc_wlbtd_get_and_print_build_type();
#endif

		/* schedule coredump and wait for it to finish */
		if (disable_auto_coredump) {
			SCSC_TAG_INFO(MXMAN, "Driver automatic coredump disabled, not launching coredump helper\n");
		} else {
			/**
			 * Releasing mxman_mutex here gives way to any
			 * eventually running resume process while waiting for
			 * the usermode helper subsystem to be resurrected,
			 * since this last will be re-enabled right at the end
			 * of the resume process itself.
			 */
			mutex_unlock(&mxman->mxman_mutex);
			SCSC_TAG_INFO(MXMAN,
				      "waiting up to %dms for usermode_helper subsystem.\n",
				      MAX_UHELP_TMO_MS);
			/* Waits for the usermode_helper subsytem to be re-enabled. */
			if (usermodehelper_read_lock_wait(msecs_to_jiffies(MAX_UHELP_TMO_MS))) {
				/**
				 * Release immediately the rwsem on usermode_helper
				 * enabled since we anyway already hold a wakelock here
				 */
				usermodehelper_read_unlock();
				/**
				 * We claim back the mxman_mutex immediately to avoid anyone
				 * shutting down the chip while we are dumping the coredump.
				 */
				mutex_lock(&mxman->mxman_mutex);
				SCSC_TAG_INFO(MXMAN, "Invoking coredump helper\n");
				slsi_kic_system_event(slsi_kic_system_event_category_recovery,
					slsi_kic_system_events_coredump_in_progress,
					GFP_KERNEL);
#ifdef CONFIG_SCSC_WLBTD
				/* we can safely call call_wlbtd as we are
				 * in workqueue context
				 */
#ifdef CONFIG_SCSC_LOG_COLLECTION
				/* Collect mxlogger logs */
				scsc_log_collector_schedule_collection(SCSC_LOG_FW_PANIC, mxman->scsc_panic_code);
#else
				r = call_wlbtd(SCSC_SCRIPT_MOREDUMP);
#endif
#else
				r = coredump_helper();
#endif
				if (r >= 0) {
					slsi_kic_system_event(slsi_kic_system_event_category_recovery,
						slsi_kic_system_events_coredump_done, GFP_KERNEL);
				}

				used = snprintf(panic_record_dump,
						PANIC_RECORD_DUMP_BUFFER_SZ,
						"RF HW Ver: 0x%X\n", mxman->rf_hw_ver);
				used += snprintf(panic_record_dump + used,
						 PANIC_RECORD_DUMP_BUFFER_SZ - used,
						 "SCSC Panic Code:: 0x%X\n", mxman->scsc_panic_code);
				used += snprintf(panic_record_dump + used,
						 PANIC_RECORD_DUMP_BUFFER_SZ - used,
						 "SCSC Last Panic Time:: %lld\n", mxman->last_panic_time);
				panic_record_dump_buffer("r4", mxman->last_panic_rec_r,
							 mxman->last_panic_rec_sz,
							 panic_record_dump + used,
							 PANIC_RECORD_DUMP_BUFFER_SZ - used);

				/* Print the host code/reason again so it's near the FW panic
				 * record in the kernel log
				 */
				print_panic_code(mxman->scsc_panic_code);
				SCSC_TAG_INFO(MXMAN, "Reason: '%s'\n", mxman->failure_reason[0] ? mxman->failure_reason : "<null>");

				blocking_notifier_call_chain(&firmware_chain,
							     SCSC_FW_EVENT_MOREDUMP_COMPLETE,
							     &panic_record_dump);
			} else {
				SCSC_TAG_INFO(MXMAN,
					      "timed out waiting for usermode_helper. Skipping coredump.\n");
				mutex_lock(&mxman->mxman_mutex);
			}
		}

		if (is_bug_on_enabled(mx)) {
			SCSC_TAG_ERR(MX_FILE, "Deliberately panic the kernel due to WLBT firmware failure!\n");
			SCSC_TAG_ERR(MX_FILE, "calling BUG_ON(1)\n");
			BUG_ON(1);
		}
		/* Clean up the MIF following error handling */
		if (mif->mif_cleanup && mxman_recovery_disabled())
			mif->mif_cleanup(mif);
	}
	SCSC_TAG_INFO(MXMAN, "Auto-recovery: %s\n",
		mxman_recovery_disabled() ? "off" : "on");

	if (!mxman_recovery_disabled())
		srvman_clear_error(srvman);
	mutex_unlock(&mxman->mxman_mutex);
	if (!mxman_recovery_disabled()) {
		SCSC_TAG_INFO(MXMAN, "Calling srvman_unfreeze_services\n");
		srvman_unfreeze_services(srvman, mxman->scsc_panic_code);
		if (scsc_mx_module_reset() < 0)
			SCSC_TAG_INFO(MXMAN, "failed to call scsc_mx_module_reset\n");
		atomic_inc(&mxman->recovery_count);
	}

	/**
	 * If recovery is disabled and an scsc_mx_service_open has been hold up,
	 * release it, rather than wait for the recovery_completion to timeout.
	 */
	if (mxman_recovery_disabled())
		complete(&mxman->recovery_completion);

	wake_unlock(&mxman->recovery_wake_lock);
}

static void failure_wq_init(struct mxman *mxman)
{
	mxman->failure_wq = create_singlethread_workqueue("failure_wq");
	INIT_WORK(&mxman->failure_work, mxman_failure_work);
}

static void failure_wq_stop(struct mxman *mxman)
{
	cancel_work_sync(&mxman->failure_work);
	flush_workqueue(mxman->failure_wq);
}

static void failure_wq_deinit(struct mxman *mxman)
{
	failure_wq_stop(mxman);
	destroy_workqueue(mxman->failure_wq);
}

static void failure_wq_start(struct mxman *mxman)
{
	if (disable_error_handling)
		SCSC_TAG_INFO(MXMAN, "error handling disabled\n");
	else
		queue_work(mxman->failure_wq, &mxman->failure_work);
}

static void print_mailboxes(struct mxman *mxman)
{
	struct scsc_mif_abs *mif;
	struct mifmboxman   *mboxman;
	int                 i;

	mif = scsc_mx_get_mif_abs(mxman->mx);
	mboxman = scsc_mx_get_mboxman(mxman->mx);

	SCSC_TAG_INFO(MXMAN, "Printing mailbox values:\n");
	for (i = 0; i < MIFMBOX_NUM; i++)
		SCSC_TAG_INFO(MXMAN, "MBOX_%d: 0x%x\n", i, *mifmboxman_get_mbox_ptr(mboxman, mif, i));
}
#ifdef CONFIG_SCSC_WLBTD
static void wlbtd_work_func(struct work_struct *work)
{
	/* require sleep-able workqueue to run successfully */
#ifdef CONFIG_SCSC_LOG_COLLECTION
	/* Collect mxlogger logs */
	/* Extend to scsc_log_collector_collect() if required */
#else
	call_wlbtd(SCSC_SCRIPT_LOGGER_DUMP);
#endif
}

static void wlbtd_wq_init(struct mxman *mx)
{
	INIT_WORK(&wlbtd_work, wlbtd_work_func);
}

static void wlbtd_wq_deinit(struct mxman *mx)
{
	/* flush and block until work is complete */
	flush_work(&wlbtd_work);
}
#endif
/*
 * Check for matching f/w and h/w
 *
 * Returns	0:  f/w and h/w match
 *		1:  f/w and h/w mismatch, try the next config
 *		-ve fatal error
 */
static int mxman_hw_ver_check(struct mxman *mxman)
{
	if (mx140_file_supported_hw(mxman->mx, mxman->rf_hw_ver))
		return 0;
	else
		return 1;
}

/*
 * Select the f/w version to load next
 */
static int mxman_select_next_fw(struct mxman *mxman)
{
	return mx140_file_select_fw(mxman->mx, mxman->rf_hw_ver);
}

/* Boot MX140 with given f/w */
static int __mxman_open(struct mxman *mxman)
{
	int r;
	struct srvman *srvman;

	mx140_basedir_file(mxman->mx);

	mutex_lock(&mxman->mxman_mutex);
	if (mxman->scsc_panic_code) {
		SCSC_TAG_INFO(MXMAN, "Previously recorded crash panic code: scsc_panic_code=0x%x\n", mxman->scsc_panic_code);
		SCSC_TAG_INFO(MXMAN, "Reason: '%s'\n", mxman->failure_reason[0] ? mxman->failure_reason : "<null>");
		print_panic_code(mxman->scsc_panic_code);
	}
	SCSC_TAG_INFO(MXMAN, "Auto-recovery: %s\n", mxman_recovery_disabled() ? "off" : "on");
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && srvman->error) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return -EINVAL;
	}

	/* Reset the state after a previous crash during f/w boot */
	if (mxman->mxman_state == MXMAN_STATE_STARTING)
		mxman->mxman_state = MXMAN_STATE_STOPPED;

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		/* if in the STARTED state there MUST already be some users */
		if (WARN_ON(!mxman->users)) {
			SCSC_TAG_ERR(MXMAN, "ERROR mxman->mxman_state=%d users=%d\n", mxman->mxman_state, mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return -EINVAL;
		}
		mxman->users++;
		SCSC_TAG_INFO(MXMAN, "Already opened: users=%d\n", mxman->users);
		mxman_print_versions(mxman);
		mutex_unlock(&mxman->mxman_mutex);
		return 0;
	} else if (mxman->mxman_state == MXMAN_STATE_STOPPED) {
		r = mxman_start(mxman);
		if (r) {
			SCSC_TAG_ERR(MXMAN, "maxwell_manager_start() failed r=%d users=%d\n", r, mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return r;
		}
		mxman->users++;
		mxman->mxman_state = MXMAN_STATE_STARTED;
		mutex_unlock(&mxman->mxman_mutex);
		/* Start mxlogger */
		if (!disable_logger) {
			static char mxlbin[128];

			r = mx140_exe_path(NULL, mxlbin, sizeof(mxlbin), "mx_logger.sh");
			if (r) {
				/* Not found */
				SCSC_TAG_ERR(MXMAN, "mx_logger.sh path error\n");
			} else {
				/* Launch it */
				_mx_exec(mxlbin, UMH_WAIT_EXEC);
			}
		}
		return 0;
	}
	WARN_ON(mxman->mxman_state != MXMAN_STATE_STARTED && mxman->mxman_state != MXMAN_STATE_STOPPED);
	SCSC_TAG_ERR(MXMAN, "Bad state: mxman->mxman_state=%d\n", mxman->mxman_state);
	mutex_unlock(&mxman->mxman_mutex);
	return -EIO;
}

int mxman_open(struct mxman *mxman)
{
	int r;
	int try = 0;

	struct scsc_mif_abs *mif = scsc_mx_get_mif_abs(mxman->mx);

	for (try = 0; try < 2; try++) {
		/* Boot WLBT. This will determine the h/w version */
		r = __mxman_open(mxman);
		if (r)
			return r;

		/* On retries, restore USBPLL owner as WLBT */
		if (try > 0 && mif->mif_restart)
			mif->mif_restart(mif);

		/* Check the h/w and f/w versions are compatible */
		r = mxman_hw_ver_check(mxman);
		if (r > 0) {
			/* Not compatible, so try next f/w */
			SCSC_TAG_INFO(MXMAN, "Incompatible h/w 0x%04x vs f/w, close and try next\n", mxman->rf_hw_ver);

			/* Temporarily return USBPLL owner to AP to keep USB alive */
			if (mif->mif_cleanup)
				mif->mif_cleanup(mif);

			/* Stop WLBT */
			mxman_close(mxman);

			/* Select the new f/w for this hw ver */
			mxman_select_next_fw(mxman);
		} else
			break; /* Running or given up */
	}

	return r;
}

static void mxman_stop(struct mxman *mxman)
{
	int r;
	struct scsc_mif_abs *mif;

	SCSC_TAG_INFO(MXMAN, "\n");

	(void)snprintf(mxman->fw_build_id, sizeof(mxman->fw_build_id), "unknown");

	mxproc_remove_ctrl_proc_dir(&mxman->mxproc);

	/* Shutdown the hardware */
	mif = scsc_mx_get_mif_abs(mxman->mx);
	r = mif->reset(mif, 1);
	if (r) {
#ifdef CONFIG_SCSC_LOG_COLLECTION
		scsc_log_collector_schedule_collection(SCSC_LOG_HOST_COMMON, SCSC_LOG_HOST_COMMON_REASON_STOP);
#else
		mx140_log_dump();
#endif
	}
	panicmon_deinit(scsc_mx_get_panicmon(mxman->mx));
	transports_release(mxman);

	mxlog_release(scsc_mx_get_mxlog(mxman->mx));
	/* unregister channel handler */
	mxmgmt_transport_register_channel_handler(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT,
						  NULL, NULL);
	fw_crc_wq_stop(mxman);

	/* Unitialise components (they may perform some checks - e.g. all memory freed) */
	mifintrbit_deinit(scsc_mx_get_intrbit(mxman->mx));
	miframman_deinit(scsc_mx_get_ramman(mxman->mx));
	mifmboxman_deinit(scsc_mx_get_mboxman(mxman->mx));

	/* Release the MIF memory resources */
	mif->unmap(mif, mxman->start_dram);
}

void mxman_close(struct mxman *mxman)
{
	int r;
	struct srvman *srvman;

	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && srvman->error) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return;
	}

	SCSC_TAG_INFO(MXMAN, "\n");

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		if (WARN_ON(!mxman->users)) {
			SCSC_TAG_ERR(MXMAN, "ERROR users=%d\n", mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return;
		}
		mxman->users--;
		if (mxman->users) {
			SCSC_TAG_INFO(MXMAN, "Current number of users=%d\n", mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return;
		}
#ifdef CONFIG_SCSC_MXLOGGER
		/**
		 * Deinit mxlogger on last service stop...BUT before asking for HALT
		 */
		mxlogger_deinit(mxman->mx, scsc_mx_get_mxlogger(mxman->mx));
#endif
		/*
		 * Ask the subsystem to stop (MM_STOP_REQ), and wait
		 * for response (MM_STOP_RSP).
		 */
		r = send_mm_msg_stop_blocking(mxman);
		if (r)
			SCSC_TAG_ERR(MXMAN, "send_mm_msg_stop_blocking failed: r=%d\n", r);

		mxman_stop(mxman);
		mxman->mxman_state = MXMAN_STATE_STOPPED;
		mutex_unlock(&mxman->mxman_mutex);
	} else if (mxman->mxman_state == MXMAN_STATE_FAILED) {
		if (WARN_ON(!mxman->users))
			SCSC_TAG_ERR(MXMAN, "ERROR users=%d\n", mxman->users);

		mxman->users--;
		if (mxman->users) {
			SCSC_TAG_INFO(MXMAN, "Current number of users=%d\n", mxman->users);
			mutex_unlock(&mxman->mxman_mutex);
			return;
		}
#ifdef CONFIG_SCSC_MXLOGGER
		/**
		 * Deinit mxlogger on last service stop...BUT before asking for HALT
		 */
		mxlogger_deinit(mxman->mx, scsc_mx_get_mxlogger(mxman->mx));
#endif

		mxman_stop(mxman);
		mxman->mxman_state = MXMAN_STATE_STOPPED;
		mutex_unlock(&mxman->mxman_mutex);
		complete(&mxman->recovery_completion);
	} else {
		WARN_ON(mxman->mxman_state != MXMAN_STATE_STARTED);
		SCSC_TAG_ERR(MXMAN, "Bad state: mxman->mxman_state=%d\n", mxman->mxman_state);
		mutex_unlock(&mxman->mxman_mutex);
		return;
	}
}

void mxman_fail(struct mxman *mxman, u16 scsc_panic_code, const char *reason)
{
	SCSC_TAG_WARNING(MXMAN, "WLBT FW failure\n");

	/* The STARTING state allows a crash during firmware boot to be handled */
	if (mxman->mxman_state == MXMAN_STATE_STARTED || mxman->mxman_state == MXMAN_STATE_STARTING) {
		mxman->mxman_next_state = MXMAN_STATE_FAILED;
		mxman->scsc_panic_code = scsc_panic_code;
		strlcpy(mxman->failure_reason, reason, sizeof(mxman->failure_reason));
		/* If recovery is disabled, don't let it be
		 * re-enabled from now on. Device must reboot
		 */
		if (mxman_recovery_disabled())
			disable_recovery_until_reboot  = true;

		failure_wq_start(mxman);
	} else {
		SCSC_TAG_WARNING(MXMAN, "Not in MXMAN_STATE_STARTED state, ignore (state %d)\n", mxman->mxman_state);
	}
}

void mxman_freeze(struct mxman *mxman)
{
	SCSC_TAG_WARNING(MXMAN, "WLBT FW frozen\n");

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		mxman->mxman_next_state = MXMAN_STATE_FREEZED;
		failure_wq_start(mxman);
	} else {
		SCSC_TAG_WARNING(MXMAN, "Not in MXMAN_STATE_STARTED state, ignore (state %d)\n", mxman->mxman_state);
	}
}

void mxman_init(struct mxman *mxman, struct scsc_mx *mx)
{
	mxman->mx = mx;
	mxman->suspended = 0;
	fw_crc_wq_init(mxman);
	failure_wq_init(mxman);
#ifdef CONFIG_SCSC_WLBTD
	wlbtd_wq_init(mxman);
#endif
	mutex_init(&mxman->mxman_mutex);
	init_completion(&mxman->recovery_completion);
	wake_lock_init(&mxman->recovery_wake_lock, WAKE_LOCK_SUSPEND, "mxman_recovery");

	/* set the initial state */
	mxman->mxman_state = MXMAN_STATE_STOPPED;
	(void)snprintf(mxman->fw_build_id, sizeof(mxman->fw_build_id), "unknown");
	memcpy(saved_fw_build_id, mxman->fw_build_id,
	       sizeof(saved_fw_build_id));
	mxproc_create_info_proc_dir(&mxman->mxproc, mxman);
	active_mxman = mxman;

#if defined(SCSC_SEP_VERSION) && SCSC_SEP_VERSION >= 9
	mxman_create_sysfs_memdump();
#endif
}

void mxman_deinit(struct mxman *mxman)
{
#if defined(SCSC_SEP_VERSION) && SCSC_SEP_VERSION >= 9
	mxman_destroy_sysfs_memdump();
#endif
	active_mxman = NULL;
	mxproc_remove_info_proc_dir(&mxman->mxproc);
	fw_crc_wq_deinit(mxman);
	failure_wq_deinit(mxman);
#ifdef CONFIG_SCSC_WLBTD
	wlbtd_wq_deinit(mxman);
#endif
	wake_lock_destroy(&mxman->recovery_wake_lock);
	mutex_destroy(&mxman->mxman_mutex);
}

int mxman_force_panic(struct mxman *mxman)
{
	struct srvman *srvman;
	struct ma_msg_packet message = { .ma_msg = MM_FORCE_PANIC };

	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && srvman->error) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return -EINVAL;
	}

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message, sizeof(message));
		mutex_unlock(&mxman->mxman_mutex);
		return 0;
	}
	mutex_unlock(&mxman->mxman_mutex);
	return -EINVAL;
}

int mxman_suspend(struct mxman *mxman)
{
	struct srvman *srvman;
	struct ma_msg_packet message = { .ma_msg = MM_HOST_SUSPEND };
	int ret;

	SCSC_TAG_INFO(MXMAN, "\n");

	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && srvman->error) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return 0;
	}

	/* Call Service suspend callbacks */
	ret = srvman_suspend_services(srvman);
	if (ret) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Service Suspend canceled - ignore %d\n", ret);
		return ret;
	}

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		SCSC_TAG_INFO(MXMAN, "MM_HOST_SUSPEND\n");
#ifdef CONFIG_SCSC_MXLOGGER
		mxlogger_generate_sync_record(scsc_mx_get_mxlogger(mxman->mx), MXLOGGER_SYN_SUSPEND);
#endif
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message, sizeof(message));
		mxman->suspended = 1;
		atomic_inc(&mxman->suspend_count);
	}
	mutex_unlock(&mxman->mxman_mutex);
	return 0;
}

void mxman_resume(struct mxman *mxman)
{
	struct srvman *srvman;
	struct ma_msg_packet message = { .ma_msg = MM_HOST_RESUME };
	int ret;

	SCSC_TAG_INFO(MXMAN, "\n");

	mutex_lock(&mxman->mxman_mutex);
	srvman = scsc_mx_get_srvman(mxman->mx);
	if (srvman && srvman->error) {
		mutex_unlock(&mxman->mxman_mutex);
		SCSC_TAG_INFO(MXMAN, "Called during error - ignore\n");
		return;
	}

	if (mxman->mxman_state == MXMAN_STATE_STARTED) {
		SCSC_TAG_INFO(MXMAN, "MM_HOST_RESUME\n");
#ifdef CONFIG_SCSC_MXLOGGER
		mxlogger_generate_sync_record(scsc_mx_get_mxlogger(mxman->mx), MXLOGGER_SYN_RESUME);
#endif
		mxmgmt_transport_send(scsc_mx_get_mxmgmt_transport(mxman->mx), MMTRANS_CHAN_ID_MAXWELL_MANAGEMENT, &message, sizeof(message));
		mxman->suspended = 0;
	}

	/* Call Service Resume callbacks */
	ret = srvman_resume_services(srvman);
	if (ret)
		SCSC_TAG_INFO(MXMAN, "Service Resume error %d\n", ret);

	mutex_unlock(&mxman->mxman_mutex);
}

static void _mx_exec_cleanup(struct subprocess_info *sp_info)
{
	if (!sp_info) {
		SCSC_TAG_ERR(MXMAN, "sp_info is null\n");
		return;
	}
	if (!sp_info->argv) {
		SCSC_TAG_ERR(MXMAN, "argv is null\n");
		return;
	}

	SCSC_TAG_INFO(MXMAN, "0x%p\n", sp_info->argv);
	argv_free(sp_info->argv);
}

/* prog - full path to programme
 * wait_exec - one of UMH_WAIT_EXEC, UMH_WAIT_PROC, UMH_KILLABLE, UMH_NO_WAIT
 */
static int _mx_exec(char *prog, int wait_exec)
{
	static char const      *envp_v[] = { "HOME=/", "PATH=/vendor/bin:/sbin:", NULL }; /* O */
	static char const      *envp_s[] = { "HOME=/", "PATH=/system/bin:/sbin:", NULL }; /* N */
	char **envp;

	const int              exec_string_buffer_len = STRING_BUFFER_MAX_LENGTH;
	const int              exec_string_args = NUMBER_OF_STRING_ARGS;
	char                   **argv;
	char                   argv_str[exec_string_buffer_len];
	int                    argc, result, len;
	struct subprocess_info *sp_info;

	len = snprintf(argv_str, exec_string_buffer_len, "%s", prog);
	if (len >= exec_string_buffer_len) {
		/* snprintf() returns a value of buffer size of greater if it had to truncate the format string. */
		SCSC_TAG_ERR(MXMAN,
			     "exec string buffer insufficient (buffer size=%d, actual string=%d)\n",
			     exec_string_buffer_len, len);
		return -E2BIG;
	}

	/* Kernel library function argv_split() will allocate memory for argv. */
	argc = 0;
	argv = argv_split(GFP_KERNEL, argv_str, &argc);
	if (!argv) {
		SCSC_TAG_ERR(MXMAN, "failed to allocate argv for userspace helper\n");
		return -ENOMEM;
	}

	/* Check the argument count - should be exec_string_args. */
	if (argc != exec_string_args) {
		SCSC_TAG_ERR(MXMAN,
			     "exec string has the wrong number of arguments (has %d, should be %d)\n",
			     argc, exec_string_args);
		argv_free(argv);
		return -E2BIG;
	}

	/* Set path specifically for vendor for selinux reasons */
	if (!strncmp(prog, "/vendor/bin", sizeof("/vendor/bin") - 1))
		envp = (char **)envp_v;
	else
		envp = (char **)envp_s;

	/* Allocate sp_info and initialise pointers to argv and envp. */
	sp_info = call_usermodehelper_setup(argv[0], argv, envp, GFP_KERNEL, NULL, _mx_exec_cleanup, NULL);

	if (!sp_info) {
		SCSC_TAG_ERR(MXMAN, "call_usermodehelper_setup() failed\n");
		argv_free(argv);
		return -EIO;
	}

	/*
	 * Put sp_info into work queue for processing by khelper.
	 * UMH_WAIT_EXEC: wait to see launch
	 */
	SCSC_TAG_INFO(MXMAN, "Launch %s\n", prog);

	result = call_usermodehelper_exec(sp_info, wait_exec);

	if (result != 0) {
		/*
		 * call_usermodehelper_exec() will free sp_info and call any cleanup function
		 * whether it succeeds or fails, so do not free argv.
		 */
		if (result == -ENOENT)
			SCSC_TAG_ERR(MXMAN, "call_usermodehelper() failed with %d, Executable not found %s'\n",
				     result, prog);
		else
			SCSC_TAG_ERR(MXMAN, "call_usermodehelper_exec() failed with %d\n", result);
	}
	return result;
}

#if defined(CONFIG_SCSC_PRINTK) && !defined(CONFIG_SCSC_WLBTD)
static int __stat(const char *file)
{
	struct kstat stat;
	mm_segment_t fs;
	int r;

	fs = get_fs();
	set_fs(get_ds());
	r = vfs_stat(file, &stat);
	set_fs(fs);

	return r;
}
#endif

int mx140_log_dump(void)
{

	int r;
# ifdef CONFIG_SCSC_WLBTD
	r = schedule_work(&wlbtd_work);
# else
	char mxlbin[128];

	r = mx140_exe_path(NULL, mxlbin, sizeof(mxlbin), "mx_logger_dump.sh");
	if (r) {
		SCSC_TAG_ERR(MXMAN, "mx_logger_dump.sh path error\n");
	} else {
#ifndef CONFIG_SCSC_WLBTD
		/*
		 * Test presence of script before invoking, to suppress
		 * unnecessary error message if not installed.
		 */
		r = __stat(mxlbin);
		if (r) {
			SCSC_TAG_DEBUG(MXMAN, "%s not installed\n", mxlbin);
			return r;
		}
#endif
		SCSC_TAG_INFO(MXMAN, "Invoking mx_logger_dump.sh UHM\n");
		r = _mx_exec(mxlbin, UMH_WAIT_EXEC);
		if (r)
			SCSC_TAG_ERR(MXMAN, "mx_logger_dump.sh err:%d\n", r);
	}
# endif /* CONFIG_SCSC_WLBTD */
	return r;
}
EXPORT_SYMBOL(mx140_log_dump);

bool mxman_recovery_disabled(void)
{
	/* If FW has panicked when recovery was disabled, don't allow it to
	 * be enabled. The horse has bolted.
	 */
	if (disable_recovery_until_reboot)
		return true;

	if (disable_recovery_handling == MEMDUMP_FILE_FOR_RECOVERY)
		return disable_recovery_from_memdump_file;
	else
		return disable_recovery_handling ? true : false;
}
EXPORT_SYMBOL(mxman_recovery_disabled);

int mxman_register_firmware_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&firmware_chain, nb);
}
EXPORT_SYMBOL(mxman_register_firmware_notifier);

int mxman_unregister_firmware_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&firmware_chain, nb);
}
EXPORT_SYMBOL(mxman_unregister_firmware_notifier);
/**
 * This returns the last known loaded FW build_id
 * even when the fw is NOT running at the time of the request.
 *
 * It could be used anytime by Android Enhanced Logging
 * to query for fw version.
 */
void mxman_get_fw_version(char *version, size_t ver_sz)
{
	/* unavailable only if chip not probed ! */
	snprintf(version, ver_sz - 1, "%s", saved_fw_build_id);
}
EXPORT_SYMBOL(mxman_get_fw_version);

void mxman_get_driver_version(char *version, size_t ver_sz)
{
	/* IMPORTANT - Do not change the formatting as User space tooling is parsing the string
	* to read SAP fapi versions. */
	snprintf(version, ver_sz - 1, "drv_ver: %d.%d.%d.%d",
		 SCSC_RELEASE_PRODUCT, SCSC_RELEASE_ITERATION, SCSC_RELEASE_CANDIDATE, SCSC_RELEASE_POINT);
#ifdef CONFIG_SCSC_WLBTD
	scsc_wlbtd_get_and_print_build_type();
#endif
}
EXPORT_SYMBOL(mxman_get_driver_version);
