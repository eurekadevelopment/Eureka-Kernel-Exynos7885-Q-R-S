/****************************************************************************
 *
 * Copyright (c) 2012 - 2017 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include <linux/sysfs.h>
#include <linux/poll.h>
#include <linux/cdev.h>

#include "debug.h"
#include "procfs.h"
#include "utils.h"

#ifndef CONFIG_SCSC_DEBUG_COMPATIBILITY
const int   SLSI_INIT_DEINIT;
const int   SLSI_NETDEV             =  1;
const int   SLSI_CFG80211           =  2;
const int   SLSI_MLME               =  3;
const int   SLSI_SUMMARY_FRAMES     =  4;
const int   SLSI_HYDRA              =  5;
const int   SLSI_TX                 =  6;
const int   SLSI_RX                 =  7;
const int   SLSI_UDI                =  8;

const int   SLSI_WIFI_FCQ           =  9;

const int   SLSI_HIP                = 10;
const int   SLSI_HIP_INIT_DEINIT    = 11;
const int   SLSI_HIP_FW_DL          = 12;
const int   SLSI_HIP_SDIO_OP        = 13;
const int   SLSI_HIP_PS             = 14;
const int   SLSI_HIP_TH             = 15;
const int   SLSI_HIP_FH             = 16;
const int   SLSI_HIP_SIG            = 17;

const int   SLSI_FUNC_TRACE         = 18;
const int   SLSI_TEST               = 19;   /* Unit test logging */
const int   SLSI_SRC_SINK           = 20;
const int   SLSI_FW_TEST            = 21;
const int   SLSI_RX_BA              = 22;

const int   SLSI_TDLS               = 23;
const int   SLSI_GSCAN              = 24;
const int   SLSI_MBULK              = 25;
const int   SLSI_FLOWC              = 26;
#endif

static uint offline_dbg_level;
module_param(offline_dbg_level, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(offline_dbg_level, "offline debug level 1-4 [default=0]");

static int slsi_dbg_set_param_cb(const char *val, const struct kernel_param *kp);
static int slsi_dbg_get_param_cb(char *buffer, const struct kernel_param *kp);

static struct kernel_param_ops param_ops_log = {
	.set = slsi_dbg_set_param_cb,
	.get = slsi_dbg_get_param_cb,
};

#define ADD_DEBUG_MODULE_PARAM(name, default_level, filter) \
	int slsi_dbg_lvl_ ## name = default_level; \
	module_param_cb(slsi_dbg_lvl_ ## name, &param_ops_log, (void *)&filter, S_IRUGO | S_IWUSR); \
	MODULE_PARM_DESC(slsi_dbg_lvl_ ## name, " Debug levels (0~4) for the " # name " module (0 = off) default=" # default_level)

#ifndef CONFIG_SCSC_DEBUG_COMPATIBILITY
/*                     Name,             Default, Filter */
ADD_DEBUG_MODULE_PARAM(init_deinit,         3, SLSI_INIT_DEINIT);
ADD_DEBUG_MODULE_PARAM(netdev,              2, SLSI_NETDEV);
ADD_DEBUG_MODULE_PARAM(cfg80211,            1, SLSI_CFG80211);
ADD_DEBUG_MODULE_PARAM(mlme,                2, SLSI_MLME);
ADD_DEBUG_MODULE_PARAM(summary_frames,      0, SLSI_SUMMARY_FRAMES);
ADD_DEBUG_MODULE_PARAM(hydra,               0, SLSI_HYDRA);
ADD_DEBUG_MODULE_PARAM(tx,                  0, SLSI_TX);
ADD_DEBUG_MODULE_PARAM(rx,                  0, SLSI_RX);
ADD_DEBUG_MODULE_PARAM(udi,                 2, SLSI_UDI);

ADD_DEBUG_MODULE_PARAM(wifi_fcq,            0, SLSI_WIFI_FCQ);

ADD_DEBUG_MODULE_PARAM(hip,                 0, SLSI_HIP);
ADD_DEBUG_MODULE_PARAM(hip_init_deinit,     0, SLSI_HIP_INIT_DEINIT);
ADD_DEBUG_MODULE_PARAM(hip_fw_dl,           0, SLSI_HIP_FW_DL);
ADD_DEBUG_MODULE_PARAM(hip_sdio_op,         0, SLSI_HIP_SDIO_OP);
ADD_DEBUG_MODULE_PARAM(hip_ps,              0, SLSI_HIP_PS);
ADD_DEBUG_MODULE_PARAM(hip_th,              0, SLSI_HIP_TH);
ADD_DEBUG_MODULE_PARAM(hip_fh,              0, SLSI_HIP_FH);
ADD_DEBUG_MODULE_PARAM(hip_sig,             0, SLSI_HIP_SIG);

ADD_DEBUG_MODULE_PARAM(func_trace,          0, SLSI_FUNC_TRACE);
ADD_DEBUG_MODULE_PARAM(test,                0, SLSI_TEST);
ADD_DEBUG_MODULE_PARAM(src_sink,            0, SLSI_SRC_SINK);
ADD_DEBUG_MODULE_PARAM(fw_test,             0, SLSI_FW_TEST);
ADD_DEBUG_MODULE_PARAM(rx_ba,               0, SLSI_RX_BA);

ADD_DEBUG_MODULE_PARAM(tdls,                2, SLSI_TDLS);
ADD_DEBUG_MODULE_PARAM(gscan,               3, SLSI_GSCAN);
ADD_DEBUG_MODULE_PARAM(mbulk,               0, SLSI_MBULK);
ADD_DEBUG_MODULE_PARAM(flowc,               0, SLSI_FLOWC);

int       slsi_dbg_lvl_all; /* Override all debug modules */

int       *slsi_dbg_filters[] = {
	&slsi_dbg_lvl_init_deinit,
	&slsi_dbg_lvl_netdev,
	&slsi_dbg_lvl_cfg80211,
	&slsi_dbg_lvl_mlme,
	&slsi_dbg_lvl_summary_frames,
	&slsi_dbg_lvl_hydra,
	&slsi_dbg_lvl_tx,
	&slsi_dbg_lvl_rx,
	&slsi_dbg_lvl_udi,

	&slsi_dbg_lvl_wifi_fcq,

	&slsi_dbg_lvl_hip,
	&slsi_dbg_lvl_hip_init_deinit,
	&slsi_dbg_lvl_hip_fw_dl,
	&slsi_dbg_lvl_hip_sdio_op,
	&slsi_dbg_lvl_hip_ps,
	&slsi_dbg_lvl_hip_th,
	&slsi_dbg_lvl_hip_fh,
	&slsi_dbg_lvl_hip_sig,

	&slsi_dbg_lvl_func_trace,
	&slsi_dbg_lvl_test,
	&slsi_dbg_lvl_src_sink,
	&slsi_dbg_lvl_fw_test,
	&slsi_dbg_lvl_rx_ba,

	&slsi_dbg_lvl_tdls,
	&slsi_dbg_lvl_gscan,
	&slsi_dbg_lvl_mbulk,
	&slsi_dbg_lvl_flowc,
};
#else
int slsi_dbg_lvl_compat_all = 0;
module_param(slsi_dbg_lvl_compat_all, int, S_IRUGO | S_IWUSR);

int       *slsi_dbg_filters[] = {
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
	&slsi_dbg_lvl_compat_all,
};

int       slsi_dbg_lvl_all; /* Override all debug modules */
#endif

const int SLSI_DF_MAX = (sizeof(slsi_dbg_filters) / sizeof(slsi_dbg_filters[0]));

const int SLSI_OVERRIDE_ALL_FILTER = -1; /* This is not a log module but merely a filter option */

/* Convert a string containing a decimal value to an integer */
static int slsi_decstr_to_int(const char *dec_str, int *res)
{
	int        tmp_res = 0;
	int        sign = 0;
	const char *tmp_char = dec_str;

	sign = (*tmp_char == '-') ? -1 : ((*tmp_char == '+') ? 1 : 0);
	if (sign != 0)
		tmp_char++;

	while (*tmp_char) {
		if (*tmp_char == '\n')
			break;
		if ((*tmp_char < '0') || (*tmp_char > '9'))
			return -1;
		tmp_res = tmp_res * 10 + (*tmp_char - '0');
		tmp_char++;
	}

	*res = (sign < 0) ? (-tmp_res) : tmp_res;
	return 0;
}

static int slsi_dbg_set_param_cb(const char *val, const struct kernel_param *kp)
{
	int new_val;
	int filter;

	if (slsi_decstr_to_int(val, &new_val) < 0) {
		pr_info("%s: failed to convert %s to int\n", __func__, val);
		return -1;
	}
	filter = *((int *)(kp->arg));

	if (filter < -1 || filter >= SLSI_DF_MAX) {
		pr_info("%s: filter %d out of range\n", __func__, filter);
		return -1;
	}

	if (filter == SLSI_OVERRIDE_ALL_FILTER) {
		if (new_val == -1) {
			pr_info("Override does not take effect because slsi_dbg_lvl_all=%d\n", new_val);
		} else {
			int i;

			pr_info("Setting all debug modules to level %d\n", new_val);
			for (i = 0; i < SLSI_DF_MAX; i++)
				*slsi_dbg_filters[i] = new_val;

			slsi_dbg_lvl_all = new_val;
		}
	} else {
		pr_info("Setting debug module %d to level %d\n", filter, new_val);
		*slsi_dbg_filters[filter] = new_val;
	}

	return 0;
}

static int slsi_dbg_get_param_cb(char *buffer, const struct kernel_param *kp)
{
#define KERN_PARAM_OPS_MAX_BUF_SIZE (4 * 1024)
	int filter;
	int val = 0;

	filter = *((int *)(kp->arg));

	if (filter == SLSI_OVERRIDE_ALL_FILTER)
		val = slsi_dbg_lvl_all;
	else if (filter < 0 || filter >= SLSI_DF_MAX)
		pr_info("%s: filter %d out of range\n", __func__, filter);
	else
		val = *slsi_dbg_filters[filter];

	return snprintf(buffer, KERN_PARAM_OPS_MAX_BUF_SIZE, "%i", val);
}

module_param_cb(slsi_dbg_lvl_all, &param_ops_log, (void *)&SLSI_OVERRIDE_ALL_FILTER, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(slsi_dbg_lvl_all, "Override debug level (0~4) for all the log modules (-1 = do not override) default=0");

#ifdef CONFIG_SCSC_WLAN_OFFLINE_TRACE

/* ---------------------------------------------------------------------------
 * The slsi_offline_debug_buffer buffer can be used to debug the HIP behaviour offline
 * i.e. without using the tracing functions that change the timing.
 *
 * Call slsi_offline_dbg_printf() with printf arguments to store a string into the buffer.
 * When unifi_debug_buf_dump() is called, the contents of the buffer are dumped into the trace.
 * The buffer is circular and writing to it when it is full simply means the oldest string(s)
 * are overwritten by the new one.
 * All these functions are intended to be called only from bh context.
 * They are not safe to call from multiple contexts
 * The exception is slsi_offline_dbg_mark() which is intended to be called from another context
 *
 * Implementation:
 * Each string of trace is stored in the buffer as a null terminated string
 * (NB this means the buffer contains the null termination for every string, changed from wifi5)
 * At all times slsi_offline_debug_ptr points at where the next string is about to be written
 * If the wrap flag is set, then the buffer is full and new messages are overwriting older ones
 * The wrap from the top of the buffer back to the start is never done part way through a string.
 * Instead if the string does not fit in the remaining space at the top (highest offset) of the buffer then the whole string
 * is written at the bottom (slsi_offline_debug_buffer[0]) of the buffer.
 * After the first wrap, the slsi_offline_debug_ptr is assumed to point part way through an older partially-overwritten string.
 * So for a dump, we search to the end of this partial string (which is discarded) and start printing strings after it.
 *
 * After we have wrapped for the first time slsi_offline_debug_top is always maintained to point
 * one byte beyond the terminating '\0' of the last string that we wrote before the last time we wrapped.
 * Before the first wrap, contents of slsi_offline_debug_top are undefined.
 * ---------------------------------------------------------------------------
 */
#define SLSI_OFFLINE_DEBUG_BUFFER_SIZE  32768
static char slsi_offline_debug_buffer[SLSI_OFFLINE_DEBUG_BUFFER_SIZE];
static char *slsi_offline_debug_ptr = slsi_offline_debug_buffer;
static char *slsi_offline_debug_top;    /* marks where we wrapped last time, undefined until slsi_offline_debug_wrap_flag is set */
static s16  slsi_offline_debug_wrap_flag;
static char slsi_offline_debug_mark_req;

/* ---------------------------------------------------------------------------
 * Puts "MARK" + the passed id value into the offline trace
 * but only when a trace function is next called in bh context.
 * The idea is this allows another context (other than bh) to put something into the trace so
 * that the relative timing of events can be seen in the log
 *
 * Arguments:
 * id        single byte value to identify a point in the log
 *           zero is not an allowed value (a request with 0 will do nothing)
 * ---------------------------------------------------------------------------
 */
void slsi_offline_dbg_mark(struct slsi_dev *sdev, int level, u8 id)
{
	if (level > offline_dbg_level)
		return;

	if (id)
		slsi_offline_debug_mark_req = id;
}

/* ---------------------------------------------------------------------------
 * Puts the contents of the passed buffer into the debug buffer
 * if append_flag is set attempts to append to the previous string
 * (unless too close to end of buffer, in which case, it will be a separate string regardless)
 *
 * Arguments:
 * hex_flag    flag set to print hex string, clear to print null terminated string.
 * append      if set, attempts to append to last string
 * buff        buffer to print as hex (if hex_flag is set)
 *             string to print (if hex_flag is clear)
 * length      number of hex chars to print (ignored if hex_flag is clear)
 * ---------------------------------------------------------------------------
 */
static void slsi_offline_dbg_append(struct slsi_dev *sdev, int level, bool hex, bool append, const char *buff, u16 length)
{
	u8  *dest;
	u32 string_len = hex ? 2 * length : strlen(buff);
	u32 free_space = sizeof(slsi_offline_debug_buffer) / sizeof(slsi_offline_debug_buffer[0]) - (slsi_offline_debug_ptr - slsi_offline_debug_buffer);

	if (level > offline_dbg_level)
		return;

	if (slsi_offline_debug_mark_req) {
		u8 copy = slsi_offline_debug_mark_req;

		slsi_offline_debug_mark_req = 0;
		slsi_offline_dbg_printf(sdev, level, false, "MARK=%u", copy);
	}

	if (free_space >= string_len + 2) /* need one extra byte for slsi_offline_debug_ptr to point to, one for final '\0' */
		dest = (slsi_offline_debug_ptr > slsi_offline_debug_buffer && append) ? slsi_offline_debug_ptr - 1 : slsi_offline_debug_ptr;
	else {
		/* there is not enough space for the whole string so instead put it at the bottom of buffer */
		/* need one extra byte for slsi_offline_debug_ptr to point to, one for terminating '\0' */
		if (sizeof(slsi_offline_debug_buffer) > string_len + 2) {
			slsi_offline_debug_wrap_flag = 1;
			slsi_offline_debug_top = slsi_offline_debug_ptr;
			dest = slsi_offline_debug_buffer;
		} else {
			SLSI_ERR_NODEV("ignoring offline trace too large at %d chars\n", string_len);
			return;
		}
	}
	slsi_offline_debug_ptr = dest + string_len + 1; /* advance over string and terminating null */

	if (hex) {
		for (; length > 0; length--) {
			static const char *hexstring = "0123456789ABCDEF";

			*dest++ = hexstring[(*buff >> 4) & 0x0f];
			*dest++ = hexstring[*buff++ & 0x0f];
		}
		*dest = '\0';
	} else {
		strcpy(dest, buff);
	}
}

/* ---------------------------------------------------------------------------
 * puts the contents of the passed string into the debug buffer
 * as a separate string
 * Arguments:
 *      str         string to print into buffer
 * ---------------------------------------------------------------------------
 */
void slsi_offline_dbg_strcpy(struct slsi_dev *sdev, int level, bool append, const char *str)
{
	if (level > offline_dbg_level)
		return;

	slsi_spinlock_lock(&sdev->offline_dbg_lock);
	slsi_offline_dbg_append(sdev, level, false, append, str, 0);
	slsi_spinlock_unlock(&sdev->offline_dbg_lock);
}

/* ---------------------------------------------------------------------------
 * if append_flag is set attempts to append to the previous string
 * (unless too close to end of buffer, in which case, it will be a separate string regardless)
 * on most implementations likely to mean it appears later in trace on the same line rather than a new one
 * ---------------------------------------------------------------------------
 */
void slsi_offline_dbg_printf(struct slsi_dev *sdev, int level, bool append, const char *fmt, ...)
{
	s32     len;
	s32     free_space;
	char    *dest;
	va_list args;

	if (level > offline_dbg_level)
		return;

	slsi_spinlock_lock(&sdev->offline_dbg_lock);

	free_space = sizeof(slsi_offline_debug_buffer) / sizeof(slsi_offline_debug_buffer[0]) - (slsi_offline_debug_ptr - slsi_offline_debug_buffer);

	if (slsi_offline_debug_mark_req) {
		u8 copy = slsi_offline_debug_mark_req;

		slsi_offline_debug_mark_req = 0;
		slsi_offline_dbg_printf(sdev, level, false, "MARK=%u", copy);
	}

	/* if appending, point ready to overwrite terminating null of previous string.  (ignore beneficial effect on free space) */
	dest = (slsi_offline_debug_ptr > slsi_offline_debug_buffer && append) ? slsi_offline_debug_ptr - 1 : slsi_offline_debug_ptr;

	va_start(args, fmt);
	len = vsnprintf(dest, free_space, fmt, args);
	va_end(args);

	/* remember vsnprintf does not report space needed for terminating null */
	/* also need one extra byte for slsi_offline_debug_ptr to point to, one for final '\0' */
	/* following test also copes with non standard vsnprintf, which for example (incorrectly) returns length actually printed or -1 */
	if ((len >= 0) && (free_space >= len + 2)) {
		slsi_offline_debug_ptr = dest + len + 1; /* advance over string and terminating null */
	} else {
		/* there is not enough space for the whole string so instead put it at the bottom of buffer */
		if (len < (s32)sizeof(slsi_offline_debug_buffer) - 1) { /* need one extra byte for slsi_offline_debug_ptr to point to */
			slsi_offline_debug_wrap_flag = 1;
			slsi_offline_debug_top = slsi_offline_debug_ptr;
			va_start(args, fmt);
			len = vsnprintf(slsi_offline_debug_buffer, sizeof(slsi_offline_debug_buffer), fmt, args);
			va_end(args);

			slsi_offline_debug_ptr = slsi_offline_debug_buffer + len + 1; /* advance over string and terminating null */
		} else {
			SLSI_ERR_NODEV("ignoring offline message because too large\n");
		}
	}

	slsi_spinlock_unlock(&sdev->offline_dbg_lock);
}

/* ---------------------------------------------------------------------------
 *  puts the contents of the passed buffer into the debug buffer as a hex string
 *  attempts to append to the previous string
 * (unless too close to end of buffer in which case, it will be a separate string)
 *
 *  Arguments:
 *      buff         buffer to print as hex
 *      length       number of chars to print
 * ---------------------------------------------------------------------------
 */
void slsi_offline_dbg_hex(struct slsi_dev *sdev, int level, const u8 *buff, u16 length)
{
	if (level > offline_dbg_level)
		return;

	slsi_spinlock_lock(&sdev->offline_dbg_lock);
	slsi_offline_dbg_append(sdev, level, true, true, (char *)buff, length);
	slsi_spinlock_unlock(&sdev->offline_dbg_lock);
}

/* ---------------------------------------------------------------------------
 * Writes the offline log data to a procfs seq_file
 * ---------------------------------------------------------------------------
 */
void slsi_offline_dbg_dump_to_seq_file(struct slsi_dev *sdev, struct seq_file *m)
{
	char *point;

	slsi_spinlock_lock(&sdev->offline_dbg_lock);

	if (slsi_offline_debug_wrap_flag)
		if (slsi_offline_debug_ptr < slsi_offline_debug_top) {
			point = slsi_offline_debug_ptr + strlen(slsi_offline_debug_ptr) + 1; /* discard first, partially overwritten string */
			while (point < slsi_offline_debug_top) {
				seq_printf(m, "%s\n", point);
				point += (strlen(point) + 1);
			}
		}

	for (point = slsi_offline_debug_buffer; point < slsi_offline_debug_ptr; point += (strlen(point) + 1))
		seq_printf(m, "%s\n", point);

	slsi_spinlock_unlock(&sdev->offline_dbg_lock);
}

/* ---------------------------------------------------------------------------
 * Writes the offline log data to the kernel log
 * ---------------------------------------------------------------------------
 */
void slsi_offline_dbg_dump_to_klog(struct slsi_dev *sdev)
{
	char *point;

	slsi_spinlock_lock(&sdev->offline_dbg_lock);

	if (slsi_offline_debug_wrap_flag)
		if (slsi_offline_debug_ptr < slsi_offline_debug_top) {
			point = slsi_offline_debug_ptr + strlen(slsi_offline_debug_ptr) + 1; /* discard first, partially overwritten string */
			while (point < slsi_offline_debug_top) {
				SLSI_INFO(sdev, "%s", point);
				point += (strlen(point) + 1);
			}
		}

	for (point = slsi_offline_debug_buffer; point < slsi_offline_debug_ptr; point += (strlen(point) + 1))
		SLSI_INFO(sdev, "%s", point);

	slsi_spinlock_unlock(&sdev->offline_dbg_lock);
}

#endif /* CONFIG_SCSC_WLAN_OFFLINE_TRACE */

#ifdef CONFIG_SCSC_WLAN_SKB_TRACKING
struct slsi_skb_tracker {
	struct slsi_spinlock lock;
	u32                  device_count;
	struct list_head     tracked;
	u32                  tracked_count;
	u32                  tracked_count_max;
};

struct slsi_tracked_skb {
	struct list_head entry;
	const char       *file;
	int              line;
	struct sk_buff   *skb;
};

static struct slsi_skb_tracker skb_tracker;

void slsi_dbg_track_skb_init(void)
{
	SLSI_DBG4_NODEV(SLSI_TEST, "\n");
	memset(&skb_tracker, 0x00, sizeof(skb_tracker));
	slsi_spinlock_create(&skb_tracker.lock);
	INIT_LIST_HEAD(&skb_tracker.tracked);
}

void slsi_dbg_track_skb_reset(void)
{
	SLSI_DBG4_NODEV(SLSI_TEST, "\n");
	skb_tracker.device_count = 0;
	skb_tracker.tracked_count = 0;
	skb_tracker.tracked_count_max = 0;
}

bool slsi_dbg_track_skb_marker_f(struct sk_buff *skb, const char *file, int line)
{
	struct slsi_tracked_skb *t;
	struct list_head        *pos, *q;
	bool                    r = true;

	if (!skb)
		return r;

	slsi_spinlock_lock(&skb_tracker.lock);
	list_for_each_safe(pos, q, &skb_tracker.tracked) {
		t = list_entry(pos, struct slsi_tracked_skb, entry);
		if (t->skb == skb) {
			SLSI_DBG4_NODEV(SLSI_TEST, "0x%p: %s:%d", skb, file, line);
			t->file = file;
			t->line = line;
			goto exit;
		}
	}
	WARN_ON(1);
	SLSI_ERR_NODEV("SKB Not Tracked: %p: %s:%d", skb, file, line);
	r = false;
exit:
	slsi_spinlock_unlock(&skb_tracker.lock);
	return r;
}

void slsi_dbg_track_skb_f(struct sk_buff *skb, gfp_t flags, const char *file, int line)
{
	struct slsi_tracked_skb *t = kmalloc(sizeof(*t), flags);

	if (!t)
		return;

	t->file = file;
	t->line = line;
	t->skb = skb_get(skb); /* Add a reference to the skb */
	SLSI_DBG4_NODEV(SLSI_TEST, "0x%p: %s:%d", skb, file, line);
	slsi_spinlock_lock(&skb_tracker.lock);
	list_add(&t->entry, &skb_tracker.tracked);
	skb_tracker.tracked_count++;
	if (skb_tracker.tracked_count > skb_tracker.tracked_count_max)
		skb_tracker.tracked_count_max = skb_tracker.tracked_count;
	slsi_spinlock_unlock(&skb_tracker.lock);
}

bool slsi_dbg_untrack_skb_f(struct sk_buff *skb, const char *file, int line)
{
	struct slsi_tracked_skb *t;
	struct list_head        *pos, *q;
	bool                    r = true;

	if (!skb)
		return r;

	slsi_spinlock_lock(&skb_tracker.lock);
	list_for_each_safe(pos, q, &skb_tracker.tracked) {
		t = list_entry(pos, struct slsi_tracked_skb, entry);
		if (t->skb == skb) {
			SLSI_DBG4_NODEV(SLSI_TEST, "0x%p: %s:%d", skb, file, line);
			list_del(pos);
			kfree_skb(t->skb); /* Free the reference we took */
			kfree(t);
			skb_tracker.tracked_count--;
			goto exit;
		}
	}
	WARN_ON(1);
	SLSI_ERR_NODEV("SKB Not Tracked: %p: %s:%d", skb, file, line);
	r = false;
exit:
	slsi_spinlock_unlock(&skb_tracker.lock);
	return r;
}

void slsi_dbg_skb_device_add(void)
{
	slsi_spinlock_lock(&skb_tracker.lock);
	skb_tracker.device_count++;
	slsi_spinlock_unlock(&skb_tracker.lock);
}

void slsi_dbg_skb_device_remove(void)
{
	slsi_spinlock_lock(&skb_tracker.lock);
	if (skb_tracker.device_count)
		skb_tracker.device_count--;
	slsi_spinlock_unlock(&skb_tracker.lock);
}

void slsi_dbg_track_skb_report(void)
{
	struct slsi_tracked_skb *t;
	struct list_head        *pos, *q;

	slsi_spinlock_lock(&skb_tracker.lock);
	if (skb_tracker.device_count) {
		slsi_spinlock_unlock(&skb_tracker.lock);
		return;
	}
	SLSI_INFO_NODEV("Tracked Count Current: %d\n", skb_tracker.tracked_count);
	SLSI_INFO_NODEV("Tracked Count Max    : %d\n", skb_tracker.tracked_count_max);
	list_for_each_safe(pos, q, &skb_tracker.tracked) {
		t = list_entry(pos, struct slsi_tracked_skb, entry);
		if (skb_shared(t->skb))
			SLSI_ERR_NODEV("SKB Leak: 0x%p: %s:%d\n", t->skb, t->file, t->line);
		else
			SLSI_ERR_NODEV("SKB Not Untracked: 0x%p: %s:%d\n", t->skb, t->file, t->line);
		list_del(pos);
		kfree_skb(t->skb); /* Free the reference we took */
		kfree(t);
		skb_tracker.tracked_count--;
	}
	INIT_LIST_HEAD(&skb_tracker.tracked);
	slsi_spinlock_unlock(&skb_tracker.lock);
}
#endif
