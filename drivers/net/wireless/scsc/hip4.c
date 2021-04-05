/******************************************************************************
 *
 * Copyright (c) 2014 - 2018 Samsung Electronics Co., Ltd. All rights reserved
 *
 *****************************************************************************/

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <scsc/scsc_mx.h>
#include <scsc/scsc_mifram.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <scsc/scsc_logring.h>

#include "hip4.h"
#include "mbulk.h"
#include "dev.h"
#include "hip4_sampler.h"

#include "scsc_wifilogger_rings.h"

#include "debug.h"

static bool hip4_system_wq;
module_param(hip4_system_wq, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(hip4_system_wq, "Use system wq instead of named workqueue. (default: N)");

static int max_buffered_frames = 10000;
module_param(max_buffered_frames, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_buffered_frames, "Maximum number of frames to buffer in the driver");

static ktime_t intr_received;
static ktime_t bh_init;
static ktime_t bh_end;
static ktime_t wdt;
static ktime_t send;
static ktime_t closing;

enum rw {
	widx,
	ridx,
};

static u8 hip4_read_index(struct slsi_hip4 *hip, u32 q, enum rw r_w);

/**** OFFSET SHOULD BE 4096 BYTES ALIGNED ***/
#define IMG_MGR_SEC_WLAN_CONFIG_OFFSET  0x00000
#define IMG_MGR_SEC_WLAN_CONFIG_SIZE    0x02000 /* 8 kB*/
#define IMG_MGR_SEC_WLAN_MIB_OFFSET     0x02000
#define IMG_MGR_SEC_WLAN_MIB_SIZE       0x01000 /* 4 kB*/
#define IMG_MGR_SEC_WLAN_TX_OFFSET      0x03000
#define IMG_MGR_SEC_WLAN_TX_SIZE        0x7d000 /* 500 kB*/
#define IMG_MGR_SEC_WLAN_TX_DAT_OFFSET  0x03000 /*   =   */
#define IMG_MGR_SEC_WLAN_TX_DAT_SIZE    0x6d000 /* 436 kB*/
#define IMG_MGR_SEC_WLAN_TX_CTL_OFFSET  0x70000 /*   +   */
#define IMG_MGR_SEC_WLAN_TX_CTL_SIZE    0x10000 /*  64 kB*/
#define IMG_MGR_SEC_WLAN_RX_OFFSET      0x80000
#define IMG_MGR_SEC_WLAN_RX_SIZE        0x100000 /* 1MB */

/* Q mapping V3 - V4 */
/*offset of F/W owned indices */
#define FW_OWN_OFS      (64)
/**
 * HIP queue indices layout in the scoreboard (SC-505612-DD). v3
 *
 *             3        2        1        0
 *         +-----------------------------------+
 *     +0  |  Q3R   |   Q2R  |  Q1W   |  Q0W   |  Owned by the host
 *         +-----------------------------------+
 *     +4  |        |        |  Q5W   |  Q4R   |  Owned by the host
 *         +-----------------------------------+
 *
 *         +-----------------------------------+
 *    +64  |  Q3W   |   Q2W  |  Q1R   |  Q0R   |  Owned by the F/W
 *         +-----------------------------------+
 *    +68  |        |        |  Q5R   |  Q4W   |  Owned by the F/W
 *         +-----------------------------------+
 *
 * The queue indcies which owned by the host are only writable by the host.
 * F/W can only read them. And vice versa.
 */
static int q_idx_layout[6][2] = {
	{               0,  FW_OWN_OFS + 0},	/* mif_q_fh_ctl : 0 */
	{               1,  FW_OWN_OFS + 1},	/* mif_q_fh_dat : 1 */
	{  FW_OWN_OFS + 2,               2},	/* mif_q_fh_rfb : 2 */
	{  FW_OWN_OFS + 3,               3},    /* mif_q_th_ctl : 3 */
	{  FW_OWN_OFS + 4,               4},	/* mif_q_th_dat : 4 */
	{               5,  FW_OWN_OFS + 5}	/* mif_q_th_rfb : 5 */
};

/*offset of F/W owned VIF Status */
#define FW_OWN_VIF      (96)
/**
 * HIP Pause state VIF. v4. 2 bits per PEER
 *
 *         +-----------------------------------+
 *    +96  |        VIF[0] Peers [15-1]        |  Owned by the F/W
 *         +-----------------------------------+
 *    +100 |        VIF[0] Peers [31-16]       |  Owned by the F/W
 *         +-----------------------------------+
 *    +104 |        VIF[1] Peers [15-1]        |  Owned by the F/W
 *         +-----------------------------------+
 *    +108 |        VIF[1] Peers [31-16]       |  Owned by the F/W
 *         +-----------------------------------+
 *    +112 |        VIF[2] Peers [15-1]        |  Owned by the F/W
 *         +-----------------------------------+
 *    +116 |        VIF[2] Peers [31-16]       |  Owned by the F/W
 *         +-----------------------------------+
 *    +120 |        VIF[3] Peers [15-1]        |  Owned by the F/W
 *         +-----------------------------------+
 *    +124 |        VIF[3] Peers [31-16]       |  Owned by the F/W
 *         +-----------------------------------+
 *
 */

/* MAX_STORM. Max Interrupts allowed when platform is in suspend */
#define MAX_STORM            5

#ifdef CONFIG_SCSC_WLAN_DEBUG

/* MAX_HISTORY_RECORDS should be power of two */
#define MAX_HISTORY_RECORDS  32

#define FH                   0
#define TH                   1

struct hip4_history {
	bool    dir;
	u32     signal;
	u32     cnt;
	ktime_t last_time;
} hip4_signal_history[MAX_HISTORY_RECORDS];

static u32 history_record;

/* This function should be called from atomic context */
static void hip4_history_record_add(bool dir, u32 signal_id)
{
	struct hip4_history record;

	record = hip4_signal_history[history_record];

	if (record.signal == signal_id && record.dir == dir) {
		/* If last signal and direction is the same, increment counter */
		record.last_time = ktime_get();
		record.cnt += 1;
		hip4_signal_history[history_record] = record;
		return;
	}

	history_record = (history_record + 1) & (MAX_HISTORY_RECORDS - 1);

	record = hip4_signal_history[history_record];
	record.dir = dir;
	record.signal = signal_id;
	record.cnt = 1;
	record.last_time = ktime_get();
	hip4_signal_history[history_record] = record;
}

#define HIP4_HISTORY(in_seq_file, m, fmt, arg ...)       \
	do {                                             \
		if (in_seq_file)                         \
			seq_printf(m, fmt, ## arg);      \
		else                                     \
			SLSI_ERR_NODEV(fmt, ## arg);     \
	} while (0)

static void hip4_history_record_print(bool in_seq_file, struct seq_file *m)
{
	struct hip4_history record;
	u32 i, pos;
	ktime_t old;

	old = ktime_set(0, 0);

	/* Start with the Next record to print history in order */
	pos = (history_record + 1) & (MAX_HISTORY_RECORDS - 1);

	HIP4_HISTORY(in_seq_file, m, "dir\t signal\t cnt\t last_time(ns) \t\t gap(ns)\n");
	HIP4_HISTORY(in_seq_file, m, "-----------------------------------------------------------------------------\n");
	for (i = 0; i < MAX_HISTORY_RECORDS; i++) {
		record = hip4_signal_history[pos];
		/*next pos*/
		if (record.cnt) {
			HIP4_HISTORY(in_seq_file, m, "%s\t 0x%04x\t %d\t %lld \t%lld\n", record.dir ? "<--TH" : "FH-->",
				     record.signal, record.cnt, ktime_to_ns(record.last_time), ktime_to_ns(ktime_sub(record.last_time, old)));
		}
		old = record.last_time;
		pos = (pos + 1) & (MAX_HISTORY_RECORDS - 1);
	}
}

static int hip4_proc_show_history(struct seq_file *m, void *v)
{
	hip4_history_record_print(true, m);
	return 0;
}

static int hip4_proc_history_open(struct inode *inode, struct file *file)
{
	return single_open(file, hip4_proc_show_history, PDE_DATA(inode));
}

static const struct file_operations hip4_procfs_history_fops = {
	.owner   = THIS_MODULE,
	.open    = hip4_proc_history_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int hip4_proc_show(struct seq_file *m, void *v)
{
	struct slsi_hip4 *hip = m->private;
	struct hip4_hip_control *hip_control;
	struct slsi_dev         *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	u8 i;

	u32 conf_hip4_ver = 0;
	void *hip_ptr;

	if (!hip->hip_priv) {
		seq_puts(m, "HIP4 not active\n");
		return 0;
	}

	conf_hip4_ver = scsc_wifi_get_hip_config_version(&hip->hip_control->init);
	/* Check if the version is supported. And get the index */
	/* This is hardcoded and may change in future versions */
	if (conf_hip4_ver != 4 && conf_hip4_ver != 3) {
		SLSI_ERR_NODEV("FW Version %d not supported or Hip has not been set up\n", conf_hip4_ver);
		return 0;
	}

	/* hip_ref contains the reference of the start of shared memory allocated for WLAN */
	/* hip_ptr is the kernel address of hip_ref*/
	hip_ptr = scsc_mx_service_mif_addr_to_ptr(sdev->service, hip->hip_ref);
	/* Get hip_control pointer on shared memory  */
	hip_control = (struct hip4_hip_control *)(hip_ptr +
		      IMG_MGR_SEC_WLAN_CONFIG_OFFSET);

	seq_puts(m, "-----------------------------------------\n");
	seq_puts(m, "HIP4 CONFIG:\n");
	seq_puts(m, "-----------------------------------------\n");
	seq_printf(m, "config kernel addr  = %p\n", hip_control);
	if (conf_hip4_ver == 4) {
		seq_printf(m, "hip4_version_4 addr = 0x%p\n", &hip_control->config_v4);
		seq_printf(m, "magic_number        = 0x%x\n", hip_control->config_v4.magic_number);
		seq_printf(m, "hip_config_ver      = 0x%x\n", hip_control->config_v4.hip_config_ver);
		seq_printf(m, "config_len          = 0x%x\n", hip_control->config_v4.config_len);
		seq_printf(m, "compat_flag         = 0x%x\n", hip_control->config_v4.compat_flag);
		seq_printf(m, "sap_mlme_ver        = 0x%x\n", hip_control->config_v4.sap_mlme_ver);
		seq_printf(m, "sap_ma_ver          = 0x%x\n", hip_control->config_v4.sap_ma_ver);
		seq_printf(m, "sap_debug_ver       = 0x%x\n", hip_control->config_v4.sap_debug_ver);
		seq_printf(m, "sap_test_ver        = 0x%x\n", hip_control->config_v4.sap_test_ver);
		seq_printf(m, "fw_build_id         = 0x%x\n", hip_control->config_v4.fw_build_id);
		seq_printf(m, "fw_patch_id         = 0x%x\n", hip_control->config_v4.fw_patch_id);
		seq_printf(m, "unidat_req_headroom = 0x%x\n", hip_control->config_v4.unidat_req_headroom);
		seq_printf(m, "unidat_req_tailroom = 0x%x\n", hip_control->config_v4.unidat_req_tailroom);
		seq_printf(m, "bulk_buffer_align   = 0x%x\n", hip_control->config_v4.bulk_buffer_align);
		seq_printf(m, "host_cache_line     = 0x%x\n", hip_control->config_v4.host_cache_line);
		seq_printf(m, "host_buf_loc        = 0x%x\n", hip_control->config_v4.host_buf_loc);
		seq_printf(m, "host_buf_sz         = 0x%x\n", hip_control->config_v4.host_buf_sz);
		seq_printf(m, "fw_buf_loc          = 0x%x\n", hip_control->config_v4.fw_buf_loc);
		seq_printf(m, "fw_buf_sz           = 0x%x\n", hip_control->config_v4.fw_buf_sz);
		seq_printf(m, "mib_buf_loc         = 0x%x\n", hip_control->config_v4.mib_loc);
		seq_printf(m, "mib_buf_sz          = 0x%x\n", hip_control->config_v4.mib_sz);
		seq_printf(m, "log_config_loc      = 0x%x\n", hip_control->config_v4.log_config_loc);
		seq_printf(m, "log_config_sz       = 0x%x\n", hip_control->config_v4.log_config_sz);
		seq_printf(m, "mif_fh_int_n        = 0x%x\n", hip_control->config_v4.mif_fh_int_n);
		seq_printf(m, "mif_th_int_n        = 0x%x\n", hip_control->config_v4.mif_th_int_n);
		seq_printf(m, "scbrd_loc           = 0x%x\n", hip_control->config_v4.scbrd_loc);
		seq_printf(m, "q_num               = 0x%x\n", hip_control->config_v4.q_num);
		seq_printf(m, "q_len               = 0x%x\n", hip_control->config_v4.q_len);
		seq_printf(m, "q_idx_sz            = 0x%x\n", hip_control->config_v4.q_idx_sz);
		for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++)
			seq_printf(m, "q_loc[%d]            = 0x%x\n", i, hip_control->config_v4.q_loc[i]);
	} else if (conf_hip4_ver == 3) {
		seq_printf(m, "hip4_version_3 addr = 0x%p\n", &hip_control->config_v3);
		seq_printf(m, "magic_number        = 0x%x\n", hip_control->config_v3.magic_number);
		seq_printf(m, "hip_config_ver      = 0x%x\n", hip_control->config_v3.hip_config_ver);
		seq_printf(m, "config_len          = 0x%x\n", hip_control->config_v3.config_len);
		seq_printf(m, "compat_flag         = 0x%x\n", hip_control->config_v3.compat_flag);
		seq_printf(m, "sap_mlme_ver        = 0x%x\n", hip_control->config_v3.sap_mlme_ver);
		seq_printf(m, "sap_ma_ver          = 0x%x\n", hip_control->config_v3.sap_ma_ver);
		seq_printf(m, "sap_debug_ver       = 0x%x\n", hip_control->config_v3.sap_debug_ver);
		seq_printf(m, "sap_test_ver        = 0x%x\n", hip_control->config_v3.sap_test_ver);
		seq_printf(m, "fw_build_id         = 0x%x\n", hip_control->config_v3.fw_build_id);
		seq_printf(m, "fw_patch_id         = 0x%x\n", hip_control->config_v3.fw_patch_id);
		seq_printf(m, "unidat_req_headroom = 0x%x\n", hip_control->config_v3.unidat_req_headroom);
		seq_printf(m, "unidat_req_tailroom = 0x%x\n", hip_control->config_v3.unidat_req_tailroom);
		seq_printf(m, "bulk_buffer_align   = 0x%x\n", hip_control->config_v3.bulk_buffer_align);
		seq_printf(m, "host_cache_line     = 0x%x\n", hip_control->config_v3.host_cache_line);
		seq_printf(m, "host_buf_loc        = 0x%x\n", hip_control->config_v3.host_buf_loc);
		seq_printf(m, "host_buf_sz         = 0x%x\n", hip_control->config_v3.host_buf_sz);
		seq_printf(m, "fw_buf_loc          = 0x%x\n", hip_control->config_v3.fw_buf_loc);
		seq_printf(m, "fw_buf_sz           = 0x%x\n", hip_control->config_v3.fw_buf_sz);
		seq_printf(m, "mib_buf_loc         = 0x%x\n", hip_control->config_v3.mib_loc);
		seq_printf(m, "mib_buf_sz          = 0x%x\n", hip_control->config_v3.mib_sz);
		seq_printf(m, "log_config_loc      = 0x%x\n", hip_control->config_v3.log_config_loc);
		seq_printf(m, "log_config_sz       = 0x%x\n", hip_control->config_v3.log_config_sz);
		seq_printf(m, "mif_fh_int_n        = 0x%x\n", hip_control->config_v3.mif_fh_int_n);
		seq_printf(m, "mif_th_int_n        = 0x%x\n", hip_control->config_v3.mif_th_int_n);
		seq_printf(m, "scbrd_loc           = 0x%x\n", hip_control->config_v3.scbrd_loc);
		seq_printf(m, "q_num               = 0x%x\n", hip_control->config_v3.q_num);
		seq_printf(m, "q_len               = 0x%x\n", hip_control->config_v3.q_len);
		seq_printf(m, "q_idx_sz            = 0x%x\n", hip_control->config_v3.q_idx_sz);
		for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++)
			seq_printf(m, "q_loc[%d]            = 0x%x\n", i, hip_control->config_v3.q_loc[i]);
	}
	seq_puts(m, "\n-----------------------------------------\n");
	seq_puts(m, "HIP4 SCOREBOARD INDEXES:\n");
	seq_puts(m, "-----------------------------------------\n");
	seq_printf(m, "ktime start %lld (ns)\n", ktime_to_ns(hip->hip_priv->stats.start));
	seq_printf(m, "ktime now   %lld (ns)\n\n", ktime_to_ns(ktime_get()));

	seq_printf(m, "rx_intr_tohost 0x%x\n", hip->hip_priv->rx_intr_tohost);
	seq_printf(m, "rx_intr_fromhost 0x%x\n\n", hip->hip_priv->rx_intr_fromhost);

	/* HIP statistics */
	seq_printf(m, "HIP IRQs: %u\n", atomic_read(&hip->hip_priv->stats.irqs));
	seq_printf(m, "HIP IRQs spurious: %u\n", atomic_read(&hip->hip_priv->stats.spurious_irqs));
	seq_printf(m, "FW debug-inds: %u\n\n", atomic_read(&sdev->debug_inds));

	seq_puts(m, "Queue\tIndex\tFrames\n");
	seq_puts(m, "-----\t-----\t------\n");
	/* Print scoreboard */
	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++) {
		seq_printf(m, "Q%dW\t0x%x\t\n", i, hip4_read_index(hip, i, widx));
		seq_printf(m, "Q%dR\t0x%x\t%d\n", i, hip4_read_index(hip, i, ridx), hip->hip_priv->stats.q_num_frames[i]);
	}
	seq_puts(m, "\n");

	return 0;
}

static int hip4_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, hip4_proc_show, PDE_DATA(inode));
}

static const struct file_operations hip4_procfs_stats_fops = {
	.owner   = THIS_MODULE,
	.open    = hip4_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 11, 0))
static inline ktime_t ktime_add_ms(const ktime_t kt, const u64 msec)
{
	return ktime_add_ns(kt, msec * NSEC_PER_MSEC);
}
#endif

#define FB_NO_SPC_NUM_RET    10
#define FB_NO_SPC_SLEEP_MS   10

/* Update scoreboard index */
/* Function can be called from BH context */
static void hip4_update_index(struct slsi_hip4 *hip, u32 q, enum rw r_w, u8 value)
{
	struct hip4_priv    *hip_priv = hip->hip_priv;

	write_lock_bh(&hip_priv->rw_scoreboard);
	if (hip->hip_priv->version == 3 || hip->hip_priv->version == 4) {
		*((u8 *)(hip->hip_priv->scbrd_base + q_idx_layout[q][r_w])) = value;
	} else {
		SLSI_ERR_NODEV("Incorrect version\n");
		goto error;
	}

	/* Memory barrier when updating shared mailbox/memory */
	smp_wmb();
	SCSC_HIP4_SAMPLER_Q(hip_priv->minor, q, r_w, value, 0);
error:
	write_unlock_bh(&hip_priv->rw_scoreboard);
}

/* Read scoreboard index */
/* Function can be called from BH context */
static u8 hip4_read_index(struct slsi_hip4 *hip, u32 q, enum rw r_w)
{
	struct hip4_priv    *hip_priv = hip->hip_priv;
	u32                 value = 0;

	read_lock_bh(&hip_priv->rw_scoreboard);
	if (hip->hip_priv->version == 3 || hip->hip_priv->version == 4) {
		value = *((u8 *)(hip->hip_priv->scbrd_base + q_idx_layout[q][r_w]));
	} else {
		SLSI_ERR_NODEV("Incorrect version\n");
		goto error;
	}

	/* Memory barrier when reading shared mailbox/memory */
	smp_rmb();
error:
	read_unlock_bh(&hip_priv->rw_scoreboard);
	return value;
}

static void hip4_dump_dbg(struct slsi_hip4 *hip, struct mbulk *m, struct sk_buff *skb, struct scsc_service *service)
{
	unsigned int        i = 0;
	scsc_mifram_ref     ref;

	SLSI_ERR_NODEV("rx_intr_tohost 0x%x\n", hip->hip_priv->rx_intr_tohost);
	SLSI_ERR_NODEV("rx_intr_fromhost 0x%x\n", hip->hip_priv->rx_intr_fromhost);

	/* Print scoreboard */
	for (i = 0; i < 6; i++) {
		SLSI_ERR_NODEV("Q%dW 0x%x\n", i, hip4_read_index(hip, i, widx));
		SLSI_ERR_NODEV("Q%dR 0x%x\n", i, hip4_read_index(hip, i, ridx));
	}

	if (service)
		scsc_mx_service_mif_dump_registers(service);

	if (m && service) {
		if (scsc_mx_service_mif_ptr_to_addr(service, m, &ref))
			return;
		SLSI_ERR_NODEV("m: %p 0x%x\n", m, ref);
		print_hex_dump(KERN_ERR, SCSC_PREFIX "mbulk ", DUMP_PREFIX_NONE, 16, 1, m, sizeof(struct mbulk), 0);
	}
	if (skb)
		print_hex_dump(KERN_ERR, SCSC_PREFIX "skb   ", DUMP_PREFIX_NONE, 16, 1, skb->data, skb->len > 0xff ? 0xff : skb->len, 0);

	SLSI_ERR_NODEV("time: wdt     %lld\n", ktime_to_ns(wdt));
	SLSI_ERR_NODEV("time: send    %lld\n", ktime_to_ns(send));
	SLSI_ERR_NODEV("time: intr    %lld\n", ktime_to_ns(intr_received));
	SLSI_ERR_NODEV("time: bh_init %lld\n", ktime_to_ns(bh_init));
	SLSI_ERR_NODEV("time: bh_end  %lld\n", ktime_to_ns(bh_end));
	SLSI_ERR_NODEV("time: closing %lld\n", ktime_to_ns(closing));
#ifdef CONFIG_SCSC_WLAN_DEBUG
	/* Discard noise if it is a mbulk/skb issue */
	if (!skb && !m)
		hip4_history_record_print(false, NULL);
#endif
}

#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
static u32     bytes_accum;
static ktime_t to;
#endif

/* Transform skb to mbulk (fapi_signal + payload) */
static struct mbulk *hip4_skb_to_mbulk(struct hip4_priv *hip, struct sk_buff *skb, bool ctrl_packet)
{
	struct mbulk        *m = NULL;
	void                *sig = NULL, *b_data = NULL;
	size_t              payload = 0;
	u8                  pool_id = ctrl_packet ? MBULK_CLASS_FROM_HOST_CTL : MBULK_CLASS_FROM_HOST_DAT;
	u8                  headroom = 0, tailroom = 0;
	enum mbulk_class    clas = ctrl_packet ? MBULK_CLASS_FROM_HOST_CTL : MBULK_CLASS_FROM_HOST_DAT;
	struct slsi_skb_cb *cb = slsi_skb_cb_get(skb);
#ifdef CONFIG_SCSC_WLAN_SG
	u32                 linear_data;
	u32                 offset;
	u8                  i;
#endif

	payload = skb->len - cb->sig_length;

	/* Get headroom/tailroom */
	headroom = hip->unidat_req_headroom;
	tailroom = hip->unidat_req_tailroom;

	/* Allocate mbulk */
	if (payload > 0) {
		/* If signal include payload, add headroom and tailroom */
		m = mbulk_with_signal_alloc_by_pool(pool_id, cb->colour, clas, cb->sig_length + 4,
						    payload + headroom + tailroom);
		if (!m)
			return NULL;
		if (!mbulk_reserve_head(m, headroom))
			return NULL;
	} else {
		/* If it is only a signal do not add headroom */
		m = mbulk_with_signal_alloc_by_pool(pool_id, cb->colour, clas, cb->sig_length + 4, 0);
		if (!m)
			return NULL;
	}

	/* Get signal handler */
	sig = mbulk_get_signal(m);
	if (!sig) {
		mbulk_free_virt_host(m);
		return NULL;
	}

	/* Copy signal */
	/* 4Bytes offset is required for FW fapi header */
	memcpy(sig + 4, skb->data, cb->sig_length);

	/* Copy payload */
	/* If the signal has payload memcpy the data */
	if (payload > 0) {
		/* Get head pointer */
		b_data = mbulk_dat_rw(m);
		if (!b_data) {
			mbulk_free_virt_host(m);
			return NULL;
		}

#ifdef CONFIG_SCSC_WLAN_SG
		/* The amount of non-paged data at skb->data can be calculated as skb->len - skb->data_len.
		 * Helper routine: skb_headlen() .
		 */
		linear_data = skb_headlen(skb) - cb->sig_length;

		offset = 0;
		/* Copy the linear data */
		if (linear_data > 0) {
			/* Copy the linear payload skipping the signal data */
			memcpy(b_data, skb->data + cb->sig_length, linear_data);
			offset = linear_data;
		}

		/* Traverse fragments and copy in to linear DRAM memory */
		for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
			skb_frag_t *frag = NULL;
			void *frag_va_data;
			unsigned int frag_size;

			frag = &skb_shinfo(skb)->frags[i];
			WARN_ON(!frag);
			if (!frag)
				continue;
			frag_va_data = skb_frag_address_safe(frag);
			WARN_ON(!frag_va_data);
			if (!frag_va_data)
				continue;
			frag_size = skb_frag_size(frag);
			/* Copy the fragmented data */
			memcpy(b_data + offset, frag_va_data, frag_size);
			offset += frag_size;
		}

		/* Check whether the driver should perform the checksum */
		if (skb->ip_summed == CHECKSUM_PARTIAL) {
			SLSI_DBG3_NODEV(SLSI_HIP, "CHECKSUM_PARTIAL. Driver performing checksum\n");
			if (skb->protocol == htons(ETH_P_IP)) {
				struct ethhdr *mach = (struct ethhdr *)b_data;
				struct iphdr *iph = (struct iphdr *)((char *)b_data + sizeof(*mach));
				unsigned int len = payload - sizeof(*mach) - (iph->ihl << 2);

				if (iph->protocol == IPPROTO_TCP) {
					struct tcphdr *th = (struct tcphdr *)((char *)b_data + sizeof(*mach) +
							    (iph->ihl << 2));
					th->check = 0;
					th->check = csum_tcpudp_magic(iph->saddr, iph->daddr, len,
						IPPROTO_TCP,
						csum_partial((char *)th, len, 0));
					SLSI_DBG3_NODEV(SLSI_HIP, "th->check 0x%x\n", ntohs(th->check));
				} else if (iph->protocol == IPPROTO_UDP) {
					struct udphdr *uh = (struct udphdr *)((char *)b_data + sizeof(*mach) +
							    (iph->ihl << 2));
					uh->check = 0;
					uh->check = csum_tcpudp_magic(iph->saddr, iph->daddr, len,
						IPPROTO_UDP,
						csum_partial((char *)uh, len, 0));
					SLSI_DBG3_NODEV(SLSI_HIP, "uh->check 0x%x\n", ntohs(uh->check));
				}
			}
		}
#else
		/* Copy payload skipping the signal data */
		memcpy(b_data, skb->data + cb->sig_length, payload);
#endif
		mbulk_append_tail(m, payload);
	}
	m->flag |= MBULK_F_OBOUND;

#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
	if (ktime_to_ms(to) == 0) {
		SLSI_DBG1_NODEV(SLSI_HIP, "init timer");
		to = ktime_add_ms(ktime_get(), 1000);
	}
	bytes_accum += payload;

	if (ktime_compare(ktime_get(), to) > 0) {
		bytes_accum  = 8 * bytes_accum;

		if (bytes_accum < 1000) {
			SCSC_HIP4_SAMPLER_THROUG(hip->minor, (bytes_accum & 0xff00) >> 8, bytes_accum & 0xff);
		} else if ((bytes_accum > 1000) && (bytes_accum < (1000 * 1000))) {
			bytes_accum = bytes_accum / 1000;
			SCSC_HIP4_SAMPLER_THROUG_K(hip->minor, (bytes_accum & 0xff00) >> 8, bytes_accum & 0xff);
		} else {
			bytes_accum = bytes_accum / (1000 * 1000);
			SCSC_HIP4_SAMPLER_THROUG_M(hip->minor, (bytes_accum & 0xff00) >> 8, bytes_accum & 0xff);
		}

		to = ktime_add_ms(ktime_get(), 1000);
		bytes_accum = 0;
	}
#endif
	return m;
}

/* Transform mbulk to skb (fapi_signal + payload) */
static struct sk_buff *hip4_mbulk_to_skb(struct scsc_service *service, struct hip4_priv *hip_priv, struct mbulk *m, scsc_mifram_ref *to_free, bool atomic)
{
	struct slsi_skb_cb        *cb;
	struct mbulk              *next_mbulk[MBULK_MAX_CHAIN];
	struct sk_buff            *skb = NULL;
	scsc_mifram_ref           ref;
	scsc_mifram_ref           m_chain_next;
	u8                        free = 0;
	u8                        i = 0, j = 0;
	u8                        *p;
	size_t                    bytes_to_alloc = 0;

	/* Get the mif ref pointer, check for incorrect mbulk */
	if (scsc_mx_service_mif_ptr_to_addr(service, m, &ref))
		return NULL;

	/* Track mbulk that should be freed */
	to_free[free++] = ref;

	bytes_to_alloc += m->sig_bufsz - 4;
	bytes_to_alloc += m->len;

	/* Detect Chained mbulk to start building the chain */
	if ((MBULK_SEG_IS_CHAIN_HEAD(m)) && (MBULK_SEG_IS_CHAINED(m))) {
		m_chain_next = mbulk_chain_next(m);
		if (!m_chain_next) {
			SLSI_ERR_NODEV("Mbulk is set MBULK_F_CHAIN_HEAD and MBULK_F_CHAIN but m_chain_next is NULL\n");
			goto cont;
		}
		while (1) {
			/* increase number mbulks in chain */
			i++;
			/* Get next_mbulk kernel address space pointer  */
			next_mbulk[i - 1] = scsc_mx_service_mif_addr_to_ptr(service, m_chain_next);
			if (!next_mbulk[i - 1]) {
				SLSI_ERR_NODEV("First Mbulk is set as MBULK_F_CHAIN but next_mbulk is NULL\n");
				return NULL;
			}
			/* Track mbulk to be freed */
			to_free[free++] = m_chain_next;
			bytes_to_alloc += next_mbulk[i - 1]->len;
			if (MBULK_SEG_IS_CHAINED(next_mbulk[i - 1])) {
				/* continue traversing the chain */
				m_chain_next = mbulk_chain_next(next_mbulk[i - 1]);
				if (!m_chain_next)
					break;

				if (i >= MBULK_MAX_CHAIN) {
					SLSI_ERR_NODEV("Max number of chained MBULK reached\n");
					return NULL;
				}
			} else {
				break;
			}
		}
	}

cont:
	if (atomic)
		skb = alloc_skb(bytes_to_alloc, GFP_ATOMIC);
	else {
		spin_unlock_bh(&hip_priv->rx_lock);
		skb = alloc_skb(bytes_to_alloc, GFP_KERNEL);
		spin_lock_bh(&hip_priv->rx_lock);
	}
	if (!skb) {
		SLSI_ERR_NODEV("Error allocating skb\n");
		return NULL;
	}

	cb = slsi_skb_cb_init(skb);
	cb->sig_length = m->sig_bufsz - 4;
	/* fapi_data_append adds to the data_length */
	cb->data_length = cb->sig_length;

	p = mbulk_get_signal(m);
	if (!p) {
		SLSI_ERR_NODEV("No signal in Mbulk\n");
		print_hex_dump(KERN_ERR, SCSC_PREFIX "mbulk ", DUMP_PREFIX_NONE, 16, 1, m, sizeof(struct mbulk), 0);
		slsi_kfree_skb(skb);
		return NULL;
	}
	/* Remove 4Bytes offset coming from FW */
	p += 4;

	/* Don't need to copy the 4Bytes header coming from the FW */
	memcpy(skb_put(skb, cb->sig_length), p, cb->sig_length);

	if (m->len)
		fapi_append_data(skb, mbulk_dat_r(m), m->len);
	for (j = 0; j < i; j++)
		fapi_append_data(skb, mbulk_dat_r(next_mbulk[j]), next_mbulk[j]->len);

	return skb;
}

/* Add signal reference (offset in shared memory) in the selected queue */
/* This function should be called in atomic context. Callers should supply proper locking mechanism */
static int hip4_q_add_signal(struct slsi_hip4 *hip, enum hip4_hip_q_conf conf, scsc_mifram_ref phy_m, struct scsc_service *service)
{
	struct hip4_hip_control *ctrl = hip->hip_control;
	struct hip4_priv        *hip_priv = hip->hip_priv;
	u8                      idx_w;
	u8                      idx_r;

	/* Read the current q write pointer */
	idx_w = hip4_read_index(hip, conf, widx);
	/* Read the current q read pointer */
	idx_r = hip4_read_index(hip, conf, ridx);
	SCSC_HIP4_SAMPLER_Q(hip_priv->minor, conf, widx, idx_w, 1);
	SCSC_HIP4_SAMPLER_Q(hip_priv->minor, conf, ridx, idx_r, 1);

	/* Queueu is full */
	if (idx_r == ((idx_w + 1) & (MAX_NUM - 1)))
		return -ENOSPC;

	/* Update array */
	ctrl->q[conf].array[idx_w] = phy_m;
	/* Memory barrier before updating shared mailbox */
	smp_wmb();
	SCSC_HIP4_SAMPLER_QREF(hip_priv->minor, phy_m, conf);
#ifdef CONFIG_SCSC_WLAN_DEBUG
	hip->hip_priv->stats.q_num_frames[conf] = hip->hip_priv->stats.q_num_frames[conf] + 1;
#endif

	/* Increase index */
	idx_w++;
	idx_w &= (MAX_NUM - 1);

	/* Update the scoreboard */
	hip4_update_index(hip, conf, widx, idx_w);

	send = ktime_get();
	scsc_service_mifintrbit_bit_set(service, hip_priv->rx_intr_fromhost, SCSC_MIFINTR_TARGET_R4);

	return 0;
}

static void hip4_watchdog(unsigned long data)
{
	struct slsi_hip4        *hip = (struct slsi_hip4 *)data;
	struct slsi_dev         *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct scsc_service     *service;
	ktime_t                 intr_ov;
	unsigned long           flags;

	if (!hip || !sdev || !sdev->service || !hip->hip_priv)
		return;

	spin_lock_irqsave(&hip->hip_priv->watchdog_lock, flags);
	if (!atomic_read(&hip->hip_priv->watchdog_timer_active))
		goto exit;

	wdt = ktime_get();

	/* if intr_received > wdt skip as intr has been unblocked */
	if (ktime_compare(intr_received, wdt) > 0) {
		wdt = ktime_set(0, 0);
		goto exit;
	}

	/* Check that wdt is > 1 HZ intr */
	intr_ov = ktime_add_ms(intr_received, jiffies_to_msecs(HZ));
	if (!(ktime_compare(intr_ov, wdt) < 0)) {
		wdt = ktime_set(0, 0);
		/* Retrigger WDT to check flags again in the future */
		mod_timer(&hip->hip_priv->watchdog, jiffies + HZ / 2);
		goto exit;
	}

	/* Unlock irq to avoid __local_bh_enable_ip warning */
	spin_unlock_irqrestore(&hip->hip_priv->watchdog_lock, flags);
	hip4_dump_dbg(hip, NULL, NULL, sdev->service);
	spin_lock_irqsave(&hip->hip_priv->watchdog_lock, flags);

	service = sdev->service;

	SLSI_INFO_NODEV("Hip4 watchdog triggered\n");

	if (scsc_service_mifintrbit_bit_mask_status_get(service) & (1 << hip->hip_priv->rx_intr_tohost)) {
		/* Interrupt might be pending! */
		SLSI_INFO_NODEV("Interrupt Masked. Unmask to restart Interrupt processing\n");
		scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->rx_intr_tohost);
	}
exit:
	spin_unlock_irqrestore(&hip->hip_priv->watchdog_lock, flags);
}

static bool slsi_check_rx_flowcontrol(struct slsi_dev *sdev)
{
	struct netdev_vif *ndev_vif;
	int qlen = 0;

	ndev_vif = netdev_priv(sdev->netdev[SLSI_NET_INDEX_WLAN]);
	if (ndev_vif)
		qlen = skb_queue_len(&ndev_vif->rx_data.queue);

	SLSI_MUTEX_LOCK(sdev->netdev_remove_mutex);
#if defined(SLSI_NET_INDEX_P2PX_SWLAN)
	if (sdev->netdev[SLSI_NET_INDEX_P2PX_SWLAN]) {
		ndev_vif = netdev_priv(sdev->netdev[SLSI_NET_INDEX_P2PX_SWLAN]);
		if (ndev_vif)
			qlen += skb_queue_len(&ndev_vif->rx_data.queue);
	}
#elif defined(SLSI_NET_INDEX_P2PX)
	if (sdev->netdev[SLSI_NET_INDEX_P2PX]) {
		ndev_vif = netdev_priv(sdev->netdev[SLSI_NET_INDEX_P2PX]);
		if (ndev_vif)
			qlen += skb_queue_len(&ndev_vif->rx_data.queue);
	}
#endif
	SLSI_MUTEX_UNLOCK(sdev->netdev_remove_mutex);

	if (qlen > max_buffered_frames) {
		SLSI_DBG1_NODEV(SLSI_HIP, "max qlen reached: %d\n", qlen);
		return true;
	}
	SLSI_DBG3_NODEV(SLSI_HIP, "qlen %d\n", qlen);

	return false;
}

/* Tasklet: high priority, low latency atomic tasks
 * cannot sleep (run atomically in soft IRQ context and are guaranteed to
 * never run on more than one CPU of a given processor, for a given tasklet)
 */

/* Worqueue: Lower priority, run in process context. Can run simultaneously on
 * different CPUs
 */
#ifdef TASKLET
static void hip4_tasklet(unsigned long data)
#else
static void hip4_wq(struct work_struct *data)
#endif
{
#ifdef TASKLET
	struct slsi_hip4        *hip = (struct slsi_hip4 *)data;
	struct hip4_priv        *hip_priv = hip->hip_priv;
	struct hip4_hip_control *ctrl = hip->hip_control;
#else
	struct hip4_priv        *hip_priv = container_of(data, struct hip4_priv, intr_wq);
	struct slsi_hip4        *hip = hip_priv->hip;
	struct hip4_hip_control *ctrl = hip->hip_control;
#endif
	scsc_mifram_ref         ref;
	void                    *mem;
	struct mbulk            *m;
	u8                      idx_r;
	u8                      idx_w;
	struct slsi_dev         *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct scsc_service     *service;
	bool                    update = false;
	bool			no_change = true;
	u8                      retry;
	bool                    rx_flowcontrol = false;

#if defined(CONFIG_SCSC_WLAN_HIP4_PROFILING) || defined(CONFIG_SCSC_WLAN_DEBUG)
	int                     id;
#endif

	if (!sdev || !sdev->service) {
		WARN_ON(1);
		return;
	}

	service = sdev->service;

	atomic_set(&hip->hip_priv->in_rx, 1);
	if (slsi_check_rx_flowcontrol(sdev))
		rx_flowcontrol = true;

	atomic_set(&hip->hip_priv->in_rx, 2);

#ifndef TASKLET
	spin_lock_bh(&hip_priv->rx_lock);
#endif
	atomic_set(&hip->hip_priv->in_rx, 3);
	SCSC_HIP4_SAMPLER_INT(hip_priv->minor);

	bh_init = ktime_get();
	idx_r = hip4_read_index(hip, HIP4_MIF_Q_FH_RFB, ridx);
	idx_w = hip4_read_index(hip, HIP4_MIF_Q_FH_RFB, widx);
	update = false;

	if (idx_r != idx_w) {
		no_change = false;
		SCSC_HIP4_SAMPLER_Q(hip_priv->minor, HIP4_MIF_Q_FH_RFB, ridx, idx_r, 1);
		SCSC_HIP4_SAMPLER_Q(hip_priv->minor, HIP4_MIF_Q_FH_RFB, widx, idx_w, 1);
	}

	while (idx_r != idx_w) {
		struct mbulk *m;
		u16 colour;

		ref = ctrl->q[HIP4_MIF_Q_FH_RFB].array[idx_r];
		SCSC_HIP4_SAMPLER_QREF(hip_priv->minor, ref, HIP4_MIF_Q_FH_RFB);
#ifdef CONFIG_SCSC_WLAN_DEBUG
		hip->hip_priv->stats.q_num_frames[HIP4_MIF_Q_FH_RFB] = hip->hip_priv->stats.q_num_frames[HIP4_MIF_Q_FH_RFB] + 1;
#endif
		mem = scsc_mx_service_mif_addr_to_ptr(service, ref);
		m = (struct mbulk *)mem;
		if (!m) {
			SLSI_ERR_NODEV("FB: Mbulk is NULL 0x%x\n", ref);
			goto consume_fb_mbulk;
		}
		/* colour is defined as: */
		/* u16 register bits:
		 * 0      - do not use
		 * [2:1]  - vif
		 * [7:3]  - peer_index
		 * [10:8] - ac queue
		 */
		colour = ((m->clas & 0xc0) << 2) | (m->pid & 0xfe);
		/* Account ONLY for data RFB */
		if ((m->pid & 0x1)  == MBULK_CLASS_FROM_HOST_DAT) {
#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
			SCSC_HIP4_SAMPLER_VIF_PEER(hip->hip_priv->minor, 0, (colour & 0x6) >> 1, (colour & 0xf8) >> 3);
			/* to profile round-trip */
			{
				u16 host_tag;
				u8 *get_host_tag;
				/* This is a nasty way of getting the host_tag without involving mbulk processing
				 * This hostag value should also be include in the cb descriptor which goes to
				 * mbulk descriptor (no room left at the moment)
				 */
				get_host_tag = (u8 *)m;
				host_tag = get_host_tag[37] << 8 | get_host_tag[36];
				SCSC_HIP4_SAMPLER_PKT_TX_FB(hip->hip_priv->minor, host_tag);
			}
#endif
			/* Ignore return value */
			slsi_hip_tx_done(sdev, colour);
		}
		mbulk_free_virt_host(m);
consume_fb_mbulk:
		/* Increase index */
		idx_r++;
		idx_r &= (MAX_NUM - 1);
		update = true;
	}
	/* Update the scoreboard */
	if (update)
		hip4_update_index(hip, HIP4_MIF_Q_FH_RFB, ridx, idx_r);

	atomic_set(&hip->hip_priv->in_rx, 4);

	idx_r = hip4_read_index(hip, HIP4_MIF_Q_TH_CTRL, ridx);
	idx_w = hip4_read_index(hip, HIP4_MIF_Q_TH_CTRL, widx);
	update = false;

	if (idx_r != idx_w) {
		no_change = false;
		SCSC_HIP4_SAMPLER_Q(hip_priv->minor, HIP4_MIF_Q_TH_CTRL, ridx, idx_r, 1);
		SCSC_HIP4_SAMPLER_Q(hip_priv->minor, HIP4_MIF_Q_TH_CTRL, widx, idx_w, 1);
	}

	while (idx_r != idx_w) {
		struct sk_buff *skb;
		/* TODO: currently the max number to be freed is 2. In future
		 * implementations (i.e. AMPDU) this number may be bigger
		 * list of mbulks to be freedi
		 */
		scsc_mifram_ref to_free[MBULK_MAX_CHAIN + 1] = { 0 };
		u8              i = 0;

		/* Catch-up with idx_w */
		ref = ctrl->q[HIP4_MIF_Q_TH_CTRL].array[idx_r];
		SCSC_HIP4_SAMPLER_QREF(hip_priv->minor, ref, HIP4_MIF_Q_TH_CTRL);
#ifdef CONFIG_SCSC_WLAN_DEBUG
		hip->hip_priv->stats.q_num_frames[HIP4_MIF_Q_TH_CTRL] = hip->hip_priv->stats.q_num_frames[HIP4_MIF_Q_TH_CTRL] + 1;
#endif
		mem = scsc_mx_service_mif_addr_to_ptr(service, ref);
		m = (struct mbulk *)(mem);
		if (!m) {
			SLSI_ERR_NODEV("Ctrl: Mbulk is NULL 0x%x\n", ref);
			goto consume_ctl_mbulk;
		}
		/* Process Control Signal */

		skb = hip4_mbulk_to_skb(service, hip_priv, m, to_free, true);
		if (!skb) {
			SLSI_ERR_NODEV("Ctrl: Error parsing skb\n");
			hip4_dump_dbg(hip, m, skb, service);
			goto consume_ctl_mbulk;
		}

#ifdef CONFIG_SCSC_WLAN_DEBUG
		if (m->flag & MBULK_F_WAKEUP) {
			SLSI_INFO(sdev, "WIFI wakeup by MLME frame 0x%x:\n", fapi_get_sigid(skb));
			SCSC_BIN_TAG_INFO(BINARY, skb->data, skb->len > 128 ? 128 : skb->len);
		}
#else
		if (m->flag & MBULK_F_WAKEUP)
			SLSI_INFO(sdev, "WIFI wakeup by MLME frame 0x%x\n", fapi_get_sigid(skb));
#endif

#if defined(CONFIG_SCSC_WLAN_HIP4_PROFILING) || defined(CONFIG_SCSC_WLAN_DEBUG)
		id = fapi_get_sigid(skb);
#endif
#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
		/* log control signal, not unidata not debug  */
		if (fapi_is_mlme(skb))
			SCSC_HIP4_SAMPLER_SIGNAL_CTRLRX(hip_priv->minor, (id & 0xff00) >> 8, id & 0xff);
#endif
#ifdef CONFIG_SCSC_WLAN_DEBUG
		hip4_history_record_add(TH, id);
#endif
		if (slsi_hip_rx(sdev, skb) < 0) {
			SLSI_ERR_NODEV("Ctrl: Error detected slsi_hip_rx\n");
			hip4_dump_dbg(hip, m, skb, service);
			slsi_kfree_skb(skb);
		}
consume_ctl_mbulk:
		/* Increase index */
		idx_r++;
		idx_r &= (MAX_NUM - 1);

		/* Go through the list of references to free */
		while ((ref = to_free[i++])) {
			/* Set the number of retries */
			retry = FB_NO_SPC_NUM_RET;
			/* return to the firmware */
			while (hip4_q_add_signal(hip, HIP4_MIF_Q_TH_RFB, ref, service) && retry > 0) {
				SLSI_WARN_NODEV("Ctrl: Not enough space in FB, retry: %d/%d\n", retry, FB_NO_SPC_NUM_RET);
				spin_unlock_bh(&hip_priv->rx_lock);
				msleep(FB_NO_SPC_SLEEP_MS);
				spin_lock_bh(&hip_priv->rx_lock);
				retry--;
				if (retry == 0)
					SLSI_ERR_NODEV("Ctrl: FB has not been freed for %d ms\n", FB_NO_SPC_NUM_RET * FB_NO_SPC_SLEEP_MS);
				SCSC_HIP4_SAMPLER_QFULL(hip_priv->minor, HIP4_MIF_Q_TH_RFB);
			}
		}
#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
		if (i > 2)
			SCSC_HIP4_SAMPLER_TOFREE(hip_priv->minor, i - 1);
#endif
		update = true;
	}

	/* Update the scoreboard */
	if (update)
		hip4_update_index(hip, HIP4_MIF_Q_TH_CTRL, ridx, idx_r);

	if (rx_flowcontrol)
		goto skip_data_q;

	atomic_set(&hip->hip_priv->in_rx, 5);

	idx_r = hip4_read_index(hip, HIP4_MIF_Q_TH_DAT, ridx);
	idx_w = hip4_read_index(hip, HIP4_MIF_Q_TH_DAT, widx);
	update = false;

	if (idx_r != idx_w) {
		no_change = false;
		SCSC_HIP4_SAMPLER_Q(hip_priv->minor, HIP4_MIF_Q_TH_DAT, ridx, idx_r, 1);
		SCSC_HIP4_SAMPLER_Q(hip_priv->minor, HIP4_MIF_Q_TH_DAT, widx, idx_w, 1);
	}

	while (idx_r != idx_w) {
		struct sk_buff *skb;
		/* TODO: currently the max number to be freed is 2. In future
		 * implementations (i.e. AMPDU) this number may be bigger
		 */
		/* list of mbulks to be freed */
		scsc_mifram_ref to_free[MBULK_MAX_CHAIN + 1] = { 0 };
		u8              i = 0;

		/* Catch-up with idx_w */
		ref = ctrl->q[HIP4_MIF_Q_TH_DAT].array[idx_r];
		SCSC_HIP4_SAMPLER_QREF(hip_priv->minor, ref, HIP4_MIF_Q_TH_DAT);
#ifdef CONFIG_SCSC_WLAN_DEBUG
		hip->hip_priv->stats.q_num_frames[HIP4_MIF_Q_TH_DAT] = hip->hip_priv->stats.q_num_frames[HIP4_MIF_Q_TH_DAT] + 1;
#endif
		mem = scsc_mx_service_mif_addr_to_ptr(service, ref);
		m = (struct mbulk *)(mem);
		if (!m) {
			SLSI_ERR_NODEV("Dat: Mbulk is NULL 0x%x\n", ref);
			goto consume_dat_mbulk;
		}

		skb = hip4_mbulk_to_skb(service, hip_priv, m, to_free, true);
		if (!skb) {
			SLSI_ERR_NODEV("Dat: Error parsing skb\n");
			hip4_dump_dbg(hip, m, skb, service);
			goto consume_dat_mbulk;
		}

#ifdef CONFIG_SCSC_WLAN_DEBUG
		if (m->flag & MBULK_F_WAKEUP) {
			SLSI_INFO(sdev, "WIFI wakeup by DATA frame:\n");
			SCSC_BIN_TAG_INFO(BINARY, skb->data, skb->len > 128 ? 128 : skb->len);
		}
#else
		if (m->flag & MBULK_F_WAKEUP) {
			SLSI_INFO(sdev, "WIFI wakeup by DATA frame:\n");
			SCSC_BIN_TAG_INFO(BINARY, fapi_get_data(skb), fapi_get_datalen(skb) > 54 ? 54 : fapi_get_datalen(skb));
		}
#endif
#ifdef CONFIG_SCSC_WLAN_DEBUG
		id = fapi_get_sigid(skb);
		hip4_history_record_add(TH, id);
#endif
		if (slsi_hip_rx(sdev, skb) < 0) {
			SLSI_ERR_NODEV("Dat: Error detected slsi_hip_rx\n");
			hip4_dump_dbg(hip, m, skb, service);
			slsi_kfree_skb(skb);
		}
consume_dat_mbulk:
		/* Increase index */
		idx_r++;
		idx_r &= (MAX_NUM - 1);

		/* Go through the list of references to free */
		while ((ref = to_free[i++])) {
			/* Set the number of retries */
			retry = FB_NO_SPC_NUM_RET;
			/* return to the firmware */
			while (hip4_q_add_signal(hip, HIP4_MIF_Q_TH_RFB, ref, service) && retry > 0) {
				SLSI_WARN_NODEV("Dat: Not enough space in FB, retry: %d/%d\n", retry, FB_NO_SPC_NUM_RET);
				spin_unlock_bh(&hip_priv->rx_lock);
				msleep(FB_NO_SPC_SLEEP_MS);
				spin_lock_bh(&hip_priv->rx_lock);
				retry--;
				if (retry == 0)
					SLSI_ERR_NODEV("Dat: FB has not been freed for %d ms\n", FB_NO_SPC_NUM_RET * FB_NO_SPC_SLEEP_MS);
				SCSC_HIP4_SAMPLER_QFULL(hip_priv->minor, HIP4_MIF_Q_TH_RFB);
			}
		}
#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
		if (i > 2)
			SCSC_HIP4_SAMPLER_TOFREE(hip_priv->minor, i - 1);
#endif
		update = true;
	}
	/* Update the scoreboard */
	if (update)
		hip4_update_index(hip, HIP4_MIF_Q_TH_DAT, ridx, idx_r);

	if (no_change)
		atomic_inc(&hip->hip_priv->stats.spurious_irqs);

skip_data_q:
	if (!atomic_read(&hip->hip_priv->closing)) {
		/* Reset status variable. DO THIS BEFORE UNMASKING!!!*/
		atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
		scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->rx_intr_tohost);
	}

	if (wake_lock_active(&hip->hip_priv->hip4_wake_lock)) {
		wake_unlock(&hip->hip_priv->hip4_wake_lock);
		SCSC_WLOG_WAKELOCK(WLOG_LAZY, WL_RELEASED, "hip4_wake_lock", WL_REASON_RX);
	}

	bh_end = ktime_get();
	SCSC_HIP4_SAMPLER_INT_OUT(hip_priv->minor);
	atomic_set(&hip->hip_priv->in_rx, 0);

#ifndef TASKLET
	spin_unlock_bh(&hip_priv->rx_lock);
#endif
}

/* IRQ handler for hip4. The function runs in Interrupt context, so all the
 * asssumptions related to interrupt should be applied (sleep, fast,...)
 */
static void hip4_irq_handler(int irq, void *data)
{
	struct slsi_hip4    *hip = (struct slsi_hip4 *)data;
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);

	(void)irq; /* unused */

	if (!hip || !sdev || !sdev->service || !hip->hip_priv)
		return;

	if (!atomic_read(&hip->hip_priv->rx_ready))
		goto end;

	intr_received = ktime_get();

	if (!wake_lock_active(&hip->hip_priv->hip4_wake_lock)) {
		wake_lock(&hip->hip_priv->hip4_wake_lock);
		SCSC_WLOG_WAKELOCK(WLOG_LAZY, WL_TAKEN, "hip4_wake_lock", WL_REASON_RX);
	}

	/* if wd timer is active system might be in trouble as it should be
	 * cleared in the BH. Ignore updating the timer
	 */
	if (!atomic_read(&hip->hip_priv->watchdog_timer_active)) {
		atomic_set(&hip->hip_priv->watchdog_timer_active, 1);
		mod_timer(&hip->hip_priv->watchdog, jiffies + HZ);
	} else {
		SLSI_ERR_NODEV("INT triggered while WDT is active\n");
		SLSI_ERR_NODEV("bh_init %lld\n", ktime_to_ns(bh_init));
		SLSI_ERR_NODEV("bh_end  %lld\n", ktime_to_ns(bh_end));
#ifndef TASKLET
		SLSI_ERR_NODEV("hip4_wq work_busy %d\n", work_busy(&hip->hip_priv->intr_wq));
#endif
		SLSI_ERR_NODEV("hip4_priv->in_rx %d\n", atomic_read(&hip->hip_priv->in_rx));
	}
	/* If system is not in suspend, mask interrupt to avoid interrupt storm and let BH run */
	if (!atomic_read(&hip->hip_priv->in_suspend)) {
		scsc_service_mifintrbit_bit_mask(sdev->service, hip->hip_priv->rx_intr_tohost);
		hip->hip_priv->storm_count = 0;
	} else if (++hip->hip_priv->storm_count >= MAX_STORM) {
		/* A MAX_STORM number of interrupts has been received
		 * when platform was in suspend. This indicates FW interrupt activity
		 * that should resume the hip4, so it is safe to mask to avoid
		 * interrupt storm.
		 */
		hip->hip_priv->storm_count = 0;
		scsc_service_mifintrbit_bit_mask(sdev->service, hip->hip_priv->rx_intr_tohost);
	}

	atomic_inc(&hip->hip_priv->stats.irqs);
#ifdef TASKLET
	tasklet_schedule(&hip->hip_priv->intr_tq);
#else
	if (hip4_system_wq)
		schedule_work(&hip->hip_priv->intr_wq);
	else
		queue_work(hip->hip_priv->hip4_workq, &hip->hip_priv->intr_wq);
#endif
end:
	/* Clear interrupt */
	scsc_service_mifintrbit_bit_clear(sdev->service, hip->hip_priv->rx_intr_tohost);
}

int hip4_init(struct slsi_hip4 *hip)
{
	void                    *hip_ptr;
	struct hip4_hip_control *hip_control;
	struct scsc_service     *service;
	struct slsi_dev         *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	scsc_mifram_ref         ref, ref_scoreboard;
	int                     i;
	int                     ret;
	u32                     total_mib_len;
	u32                     mib_file_offset;


	if (!sdev || !sdev->service)
		return -EINVAL;

	hip->hip_priv = kzalloc(sizeof(*hip->hip_priv), GFP_ATOMIC);
	if (!hip->hip_priv)
		return -ENOMEM;
#ifdef CONFIG_SCSC_WLAN_DEBUG
	memset(&hip->hip_priv->stats, 0, sizeof(hip->hip_priv->stats));
	hip->hip_priv->stats.start = ktime_get();
	hip->hip_priv->stats.procfs_dir = proc_mkdir("driver/hip4", NULL);
	if (NULL != hip->hip_priv->stats.procfs_dir) {
		proc_create_data("info", S_IRUSR | S_IRGRP,
				 hip->hip_priv->stats.procfs_dir, &hip4_procfs_stats_fops, hip);
		proc_create_data("history", S_IRUSR | S_IRGRP,
				 hip->hip_priv->stats.procfs_dir, &hip4_procfs_history_fops, hip);
	}

	hip->hip_priv->minor = hip4_sampler_register_hip(sdev->maxwell_core);
	if (hip->hip_priv->minor < SCSC_HIP4_INTERFACES) {
		SLSI_DBG1_NODEV(SLSI_HIP, "registered with minor %d\n", hip->hip_priv->minor);
		sdev->minor_prof = hip->hip_priv->minor;
	} else {
		SLSI_DBG1_NODEV(SLSI_HIP, "hip4_sampler is not enabled\n");
	}
#endif

	/* Used in the workqueue */
	hip->hip_priv->hip = hip;

	service = sdev->service;

	hip->hip_priv->host_pool_id_dat = MBULK_CLASS_FROM_HOST_DAT;
	hip->hip_priv->host_pool_id_ctl = MBULK_CLASS_FROM_HOST_CTL;

	/* hip_ref contains the reference of the start of shared memory allocated for WLAN */
	/* hip_ptr is the kernel address of hip_ref*/
	hip_ptr = scsc_mx_service_mif_addr_to_ptr(service, hip->hip_ref);

#ifdef CONFIG_SCSC_WLAN_DEBUG
	/* Configure mbulk allocator - Data QUEUES */
	ret = mbulk_pool_add(MBULK_CLASS_FROM_HOST_DAT, hip_ptr + IMG_MGR_SEC_WLAN_TX_DAT_OFFSET,
			     hip_ptr + IMG_MGR_SEC_WLAN_TX_DAT_OFFSET + IMG_MGR_SEC_WLAN_TX_DAT_SIZE,
			     (IMG_MGR_SEC_WLAN_TX_DAT_SIZE / HIP4_DAT_SLOTS) - sizeof(struct mbulk), 5,
			     hip->hip_priv->minor);
	if (ret)
		return ret;

	/* Configure mbulk allocator - Control QUEUES */
	ret = mbulk_pool_add(MBULK_CLASS_FROM_HOST_CTL, hip_ptr + IMG_MGR_SEC_WLAN_TX_CTL_OFFSET,
			     hip_ptr + IMG_MGR_SEC_WLAN_TX_CTL_OFFSET + IMG_MGR_SEC_WLAN_TX_CTL_SIZE,
			     (IMG_MGR_SEC_WLAN_TX_CTL_SIZE / HIP4_CTL_SLOTS) - sizeof(struct mbulk), 0,
			     hip->hip_priv->minor);
	if (ret)
		return ret;
#else
	/* Configure mbulk allocator - Data QUEUES */
	ret = mbulk_pool_add(MBULK_CLASS_FROM_HOST_DAT, hip_ptr + IMG_MGR_SEC_WLAN_TX_DAT_OFFSET,
			     hip_ptr + IMG_MGR_SEC_WLAN_TX_DAT_OFFSET + IMG_MGR_SEC_WLAN_TX_DAT_SIZE,
			     (IMG_MGR_SEC_WLAN_TX_DAT_SIZE / HIP4_DAT_SLOTS) - sizeof(struct mbulk), 5);
	if (ret)
		return ret;

	/* Configure mbulk allocator - Control QUEUES */
	ret = mbulk_pool_add(MBULK_CLASS_FROM_HOST_CTL, hip_ptr + IMG_MGR_SEC_WLAN_TX_CTL_OFFSET,
			     hip_ptr + IMG_MGR_SEC_WLAN_TX_CTL_OFFSET + IMG_MGR_SEC_WLAN_TX_CTL_SIZE,
			    (IMG_MGR_SEC_WLAN_TX_CTL_SIZE / HIP4_CTL_SLOTS) - sizeof(struct mbulk), 0);
	if (ret)
		return ret;
#endif

	/* Reset hip_control table */
	memset(hip_ptr, 0, sizeof(struct hip4_hip_control));

	/* Reset Sample q values sending 0xff */
	SCSC_HIP4_SAMPLER_RESET(hip->hip_priv->minor);

	/* Set driver is not ready to receive interrupts */
	atomic_set(&hip->hip_priv->rx_ready, 0);

	/* TOHOST Handler allocator */
	hip->hip_priv->rx_intr_tohost =
		scsc_service_mifintrbit_register_tohost(service, hip4_irq_handler, hip);

	/* Mask the interrupt to prevent intr been kicked during start */
	scsc_service_mifintrbit_bit_mask(service, hip->hip_priv->rx_intr_tohost);

	/* FROMHOST Handler allocator */
	hip->hip_priv->rx_intr_fromhost =
		scsc_service_mifintrbit_alloc_fromhost(service, SCSC_MIFINTR_TARGET_R4);

	/* Get hip_control pointer on shared memory  */
	hip_control = (struct hip4_hip_control *)(hip_ptr +
		      IMG_MGR_SEC_WLAN_CONFIG_OFFSET);

	/* Initialize scoreboard */
	if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->scoreboard, &ref_scoreboard))
		return -EFAULT;

	/* Calculate total space used by wlan*.hcf files */
	for (i = 0, total_mib_len = 0; i < SLSI_WLAN_MAX_MIB_FILE; i++)
		total_mib_len += sdev->mib[i].mib_len;

	/* Initialize hip_control table for version 4 */
	/***** VERSION 4 *******/
	hip_control->config_v4.magic_number = 0xcaba0401;
	hip_control->config_v4.hip_config_ver = 4;
	hip_control->config_v4.config_len = sizeof(struct hip4_hip_config_version_4);
	hip_control->config_v4.host_cache_line = 64;
	hip_control->config_v4.host_buf_loc = hip->hip_ref + IMG_MGR_SEC_WLAN_TX_OFFSET;
	hip_control->config_v4.host_buf_sz  = IMG_MGR_SEC_WLAN_TX_SIZE;
	hip_control->config_v4.fw_buf_loc   = hip->hip_ref + IMG_MGR_SEC_WLAN_RX_OFFSET;
	hip_control->config_v4.fw_buf_sz    = IMG_MGR_SEC_WLAN_RX_SIZE;

	/* Copy MIB content in shared memory if any */
	/* Clear the area to avoid picking up old values */
	memset(hip_ptr + IMG_MGR_SEC_WLAN_MIB_OFFSET, 0, IMG_MGR_SEC_WLAN_MIB_SIZE);

	if (total_mib_len > IMG_MGR_SEC_WLAN_MIB_SIZE) {
		SLSI_ERR_NODEV("MIB size (%d), is bigger than the MIB AREA (%d). Aborting memcpy\n", total_mib_len, IMG_MGR_SEC_WLAN_MIB_SIZE);
		hip_control->config_v4.mib_loc      = 0;
		hip_control->config_v4.mib_sz       = 0;
		total_mib_len = 0;
	} else if (total_mib_len) {
		SLSI_INFO_NODEV("Loading MIB into shared memory, size (%d)\n", total_mib_len);
		/* Load each MIB file into shared DRAM region */
		for (i = 0, mib_file_offset = 0;
		     i < SLSI_WLAN_MAX_MIB_FILE;
		     i++) {
			SLSI_INFO_NODEV("Loading MIB %d into shared memory, offset (%d), size (%d), total (%d)\n", i, mib_file_offset, sdev->mib[i].mib_len, total_mib_len);
			if (sdev->mib[i].mib_len) {
				memcpy((u8 *)hip_ptr + IMG_MGR_SEC_WLAN_MIB_OFFSET + mib_file_offset, sdev->mib[i].mib_data, sdev->mib[i].mib_len);
				mib_file_offset += sdev->mib[i].mib_len;
			}
		}
		hip_control->config_v4.mib_loc      = hip->hip_ref + IMG_MGR_SEC_WLAN_MIB_OFFSET;
		hip_control->config_v4.mib_sz       = total_mib_len;
	} else {
		hip_control->config_v4.mib_loc      = 0;
		hip_control->config_v4.mib_sz       = 0;
	}
	hip_control->config_v4.log_config_loc = 0;
	hip_control->config_v4.mif_fh_int_n = hip->hip_priv->rx_intr_fromhost;
	hip_control->config_v4.mif_th_int_n = hip->hip_priv->rx_intr_tohost;
	hip_control->config_v4.scbrd_loc = (u32)ref_scoreboard;
	hip_control->config_v4.q_num = 6;
	hip_control->config_v4.q_len = 256;
	hip_control->config_v4.q_idx_sz = 1;
	/* Initialize q relative positions */
	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++) {
		if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->q[i].array, &ref))
			return -EFAULT;
		hip_control->config_v4.q_loc[i] = (u32)ref;
	}
	/***** END VERSION 4 *******/

	/* Initialize hip_control table for version 3 */
	/***** VERSION 3 *******/
	hip_control->config_v3.magic_number = 0xcaba0401;
	hip_control->config_v3.hip_config_ver = 3;
	hip_control->config_v3.config_len = sizeof(struct hip4_hip_config_version_3);
	hip_control->config_v3.host_cache_line = 64;
	hip_control->config_v3.host_buf_loc = hip->hip_ref + IMG_MGR_SEC_WLAN_TX_OFFSET;
	hip_control->config_v3.host_buf_sz  = IMG_MGR_SEC_WLAN_TX_SIZE;
	hip_control->config_v3.fw_buf_loc   = hip->hip_ref + IMG_MGR_SEC_WLAN_RX_OFFSET;
	hip_control->config_v3.fw_buf_sz    = IMG_MGR_SEC_WLAN_RX_SIZE;

	/* Copy MIB content in shared memory if any */
	/* Clear the area to avoid picking up old values */
	memset(hip_ptr + IMG_MGR_SEC_WLAN_MIB_OFFSET, 0, IMG_MGR_SEC_WLAN_MIB_SIZE);

	if (total_mib_len > IMG_MGR_SEC_WLAN_MIB_SIZE) {
		SLSI_ERR_NODEV("MIB size (%d), is bigger than the MIB AREA (%d). Aborting memcpy\n", total_mib_len, IMG_MGR_SEC_WLAN_MIB_SIZE);
		hip_control->config_v3.mib_loc      = 0;
		hip_control->config_v3.mib_sz       = 0;
	} else if (total_mib_len) {
		SLSI_INFO_NODEV("Loading MIB into shared memory, size (%d)\n", total_mib_len);
		/* Load each MIB file into shared DRAM region */
		for (i = 0, mib_file_offset = 0;
		     i < SLSI_WLAN_MAX_MIB_FILE;
		     i++) {
			SLSI_INFO_NODEV("Loading MIB %d into shared memory, offset (%d), size (%d), total (%d)\n", i, mib_file_offset, sdev->mib[i].mib_len, total_mib_len);
			if (sdev->mib[i].mib_len) {
				memcpy((u8 *)hip_ptr + IMG_MGR_SEC_WLAN_MIB_OFFSET + mib_file_offset, sdev->mib[i].mib_data, sdev->mib[i].mib_len);
				mib_file_offset += sdev->mib[i].mib_len;
			}
		}
		hip_control->config_v3.mib_loc      = hip->hip_ref + IMG_MGR_SEC_WLAN_MIB_OFFSET;
		hip_control->config_v3.mib_sz       = total_mib_len;
	} else {
		hip_control->config_v3.mib_loc      = 0;
		hip_control->config_v3.mib_sz       = 0;
	}
	hip_control->config_v3.log_config_loc = 0;
	hip_control->config_v3.mif_fh_int_n = hip->hip_priv->rx_intr_fromhost;
	hip_control->config_v3.mif_th_int_n = hip->hip_priv->rx_intr_tohost;
	hip_control->config_v3.q_num = 6;
	hip_control->config_v3.q_len = 256;
	hip_control->config_v3.q_idx_sz = 1;
	hip_control->config_v3.scbrd_loc = (u32)ref_scoreboard; /* scoreborad location */

	/* Initialize q relative positions */
	for (i = 0; i < MIF_HIP_CFG_Q_NUM; i++) {
		if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->q[i].array, &ref))
			return -EFAULT;
		hip_control->config_v3.q_loc[i] = (u32)ref;
	}
	/***** END VERSION 3 *******/

	/* Initialzie hip_init configuration */
	hip_control->init.magic_number = 0xcaaa0400;
	if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->config_v4, &ref))
		return -EFAULT;
	hip_control->init.version_a_ref = ref;

	if (scsc_mx_service_mif_ptr_to_addr(service, &hip_control->config_v3, &ref))
		return -EFAULT;
	hip_control->init.version_b_ref = ref;
	/* End hip_init configuration */

	hip->hip_control = hip_control;
	hip->hip_priv->scbrd_base = &hip_control->scoreboard;
#ifndef TASKLET
	spin_lock_init(&hip->hip_priv->rx_lock);
#endif
	atomic_set(&hip->hip_priv->in_rx, 0);
	spin_lock_init(&hip->hip_priv->tx_lock);
	atomic_set(&hip->hip_priv->in_tx, 0);

	wake_lock_init(&hip->hip_priv->hip4_wake_lock, WAKE_LOCK_SUSPEND, "hip4_wake_lock");

	/* Init work structs */
	hip->hip_priv->hip4_workq = create_singlethread_workqueue("hip4_work");
	if (!hip->hip_priv->hip4_workq) {
		SLSI_ERR_NODEV("Error creating singlethread_workqueue\n");
		return -ENOMEM;
	}
#ifdef TASKLET
	/* Init tasklet */
	tasklet_init(&hip->hip_priv->intr_tq, hip4_tasklet, (unsigned long)hip);
#else
	/* Init wq */
	INIT_WORK(&hip->hip_priv->intr_wq, hip4_wq);
#endif
	rwlock_init(&hip->hip_priv->rw_scoreboard);

	/* Setup watchdog timer */
	atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
	spin_lock_init(&hip->hip_priv->watchdog_lock);
	setup_timer(&hip->hip_priv->watchdog, hip4_watchdog, (unsigned long)hip);

	atomic_set(&hip->hip_priv->gmod, HIP4_DAT_SLOTS);
	atomic_set(&hip->hip_priv->gactive, 1);
	spin_lock_init(&hip->hip_priv->gbot_lock);
	hip->hip_priv->saturated = 0;

	return 0;
}

/**
 * This function returns the number of free slots available to
 * transmit control packet.
 */
int hip4_free_ctrl_slots_count(struct slsi_hip4 *hip)
{
	return mbulk_pool_get_free_count(MBULK_CLASS_FROM_HOST_CTL);
}

/**
 * This function is in charge to transmit a frame through the HIP.
 * It does NOT take ownership of the SKB unless it successfully transmit it;
 * as a consequence skb is NOT freed on error.
 * We return ENOSPC on queue related troubles in order to trigger upper
 * layers of kernel to requeue/retry.
 * We free ONLY locally-allocated stuff.
 */
int scsc_wifi_transmit_frame(struct slsi_hip4 *hip, bool ctrl_packet, struct sk_buff *skb)
{
	struct scsc_service       *service;
	scsc_mifram_ref           offset;
	struct mbulk              *m;
	struct slsi_dev           *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct fapi_signal_header *fapi_header;
	int                       ret = 0;
#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
	struct slsi_skb_cb *cb = slsi_skb_cb_get(skb);
#endif

	if (!hip || !sdev || !sdev->service || !skb || !hip->hip_priv)
		return -EINVAL;

	spin_lock_bh(&hip->hip_priv->tx_lock);
	atomic_set(&hip->hip_priv->in_tx, 1);

	if (!wake_lock_active(&hip->hip_priv->hip4_wake_lock)) {
		wake_lock(&hip->hip_priv->hip4_wake_lock);
		SCSC_WLOG_WAKELOCK(WLOG_LAZY, WL_TAKEN, "hip4_wake_lock", WL_REASON_TX);
	}

	service = sdev->service;

	fapi_header = (struct fapi_signal_header *)skb->data;

	m = hip4_skb_to_mbulk(hip->hip_priv, skb, ctrl_packet);
	if (!m) {
		SCSC_HIP4_SAMPLER_MFULL(hip->hip_priv->minor);
		ret = -ENOSPC;
		SLSI_ERR_NODEV("mbulk is NULL\n");
		goto error;
	}

	if (scsc_mx_service_mif_ptr_to_addr(service, m, &offset) < 0) {
		mbulk_free_virt_host(m);
		ret = -EFAULT;
		SLSI_ERR_NODEV("Incorrect reference memory\n");
		goto error;
	}

	if (hip4_q_add_signal(hip, ctrl_packet ? HIP4_MIF_Q_FH_CTRL : HIP4_MIF_Q_FH_DAT, offset, service)) {
		SCSC_HIP4_SAMPLER_QFULL(hip->hip_priv->minor, ctrl_packet ? HIP4_MIF_Q_FH_CTRL : HIP4_MIF_Q_FH_DAT);
		mbulk_free_virt_host(m);
		ret = -ENOSPC;
		SLSI_ERR_NODEV("No space\n");
		goto error;
	}

#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
	/* colour is defined as: */
	/* u8 register bits:
	 * 0      - do not use
	 * [2:1]  - vif
	 * [7:3]  - peer_index
	 */
	if (ctrl_packet) {
		/* Record control signal */
		SCSC_HIP4_SAMPLER_SIGNAL_CTRLTX(hip->hip_priv->minor, (fapi_header->id & 0xff00) >> 8, fapi_header->id & 0xff);
	} else {
		SCSC_HIP4_SAMPLER_PKT_TX_HIP4(hip->hip_priv->minor, fapi_get_u16(skb, u.ma_unitdata_req.host_tag));
		SCSC_HIP4_SAMPLER_VIF_PEER(hip->hip_priv->minor, 1, (cb->colour & 0x6) >> 1, (cb->colour & 0xf8) >> 3);
	}
#endif
#ifdef CONFIG_SCSC_WLAN_DEBUG
	hip4_history_record_add(FH, fapi_header->id);
#endif

	/* Here we push a copy of the bare skb TRANSMITTED data also to the logring
	 * as a binary record. Note that bypassing UDI subsystem as a whole
	 * means we are losing:
	 *   UDI filtering / UDI Header INFO / UDI QueuesFrames Throttling /
	 *   UDI Skb Asynchronous processing
	 * We keep separated DATA/CTRL paths.
	 */
	if (ctrl_packet)
		SCSC_BIN_TAG_DEBUG(BIN_WIFI_CTRL_TX, skb->data, skb_headlen(skb));
	else
		SCSC_BIN_TAG_DEBUG(BIN_WIFI_DATA_TX, skb->data, skb_headlen(skb));
	/* slsi_log_clients_log_signal_fast: skb is copied to all the log clients */
	slsi_log_clients_log_signal_fast(sdev, &sdev->log_clients, skb, SLSI_LOG_DIRECTION_FROM_HOST);
	slsi_kfree_skb(skb);
	atomic_set(&hip->hip_priv->in_tx, 0);
	spin_unlock_bh(&hip->hip_priv->tx_lock);
	return 0;

error:
	if (wake_lock_active(&hip->hip_priv->hip4_wake_lock)) {
		wake_unlock(&hip->hip_priv->hip4_wake_lock);
		SCSC_WLOG_WAKELOCK(WLOG_LAZY, WL_RELEASED, "hip4_wake_lock", WL_REASON_TX);
	}
	atomic_set(&hip->hip_priv->in_tx, 0);
	spin_unlock_bh(&hip->hip_priv->tx_lock);
	return ret;
}

/* HIP4 has been initialize, setup with values
 * provided by FW
 */
int hip4_setup(struct slsi_hip4 *hip)
{
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct scsc_service *service;
	u32 conf_hip4_ver = 0;

	if (!sdev || !sdev->service)
		return -EIO;

	if (atomic_read(&sdev->hip.hip_state) != SLSI_HIP_STATE_STARTED)
		return -EIO;

	service = sdev->service;

	/* Get the Version reported by the FW */
	conf_hip4_ver = scsc_wifi_get_hip_config_version(&hip->hip_control->init);
	/* Check if the version is supported. And get the index */
	/* This is hardcoded and may change in future versions */
	if (conf_hip4_ver != 4 && conf_hip4_ver != 3) {
		SLSI_ERR_NODEV("FW Version %d not supported\n", conf_hip4_ver);
		return -EIO;
	}

	/* If version 4 is used */
	if (conf_hip4_ver == 4) {
		hip->hip_priv->unidat_req_headroom =
			scsc_wifi_get_hip_config_u8(&hip->hip_control, unidat_req_headroom, 4);
		hip->hip_priv->unidat_req_tailroom =
			scsc_wifi_get_hip_config_u8(&hip->hip_control, unidat_req_tailroom, 4);
		hip->hip_priv->version = 4;
	} else {
		/* version 3 */
		hip->hip_priv->unidat_req_headroom =
			scsc_wifi_get_hip_config_u8(&hip->hip_control, unidat_req_headroom, 3);
		hip->hip_priv->unidat_req_tailroom =
			scsc_wifi_get_hip_config_u8(&hip->hip_control, unidat_req_tailroom, 3);
		hip->hip_priv->version = 3;
	}
	/* Unmask interrupts - now host should handle them */
	atomic_set(&hip->hip_priv->stats.irqs, 0);
	atomic_set(&hip->hip_priv->stats.spurious_irqs, 0);
	atomic_set(&sdev->debug_inds, 0);

	atomic_set(&hip->hip_priv->closing, 0);

	/* Driver is ready to process IRQ */
	atomic_set(&hip->hip_priv->rx_ready, 1);
	scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->rx_intr_tohost);

	return 0;
}

/* On suspend hip4 needs to ensure that TH interrupts *are* unmasked */
void hip4_suspend(struct slsi_hip4 *hip)
{
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct scsc_service *service;

	if (!sdev || !sdev->service)
		return;

	if (atomic_read(&sdev->hip.hip_state) != SLSI_HIP_STATE_STARTED)
		return;

	service = sdev->service;

	atomic_set(&hip->hip_priv->in_suspend, 1);
	scsc_service_mifintrbit_bit_unmask(service, hip->hip_priv->rx_intr_tohost);
}

/* TH interrupts can be masked/unmasked */
void hip4_resume(struct slsi_hip4 *hip)
{
	if (!hip || !hip->hip_priv)
		return;

	atomic_set(&hip->hip_priv->in_suspend, 0);
}

void hip4_freeze(struct slsi_hip4 *hip)
{
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct scsc_service *service;

	if (!sdev || !sdev->service)
		return;

	if (atomic_read(&sdev->hip.hip_state) != SLSI_HIP_STATE_STARTED)
		return;

	service = sdev->service;

	closing = ktime_get();
	atomic_set(&hip->hip_priv->closing, 1);

	hip4_dump_dbg(hip, NULL, NULL, service);

	scsc_service_mifintrbit_bit_mask(service, hip->hip_priv->rx_intr_tohost);

#ifdef TASKLET
	tasklet_kill(&hip->hip_priv->intr_tq);
#else
	cancel_work_sync(&hip->hip_priv->intr_wq);
#endif
	flush_workqueue(hip->hip_priv->hip4_workq);
	destroy_workqueue(hip->hip_priv->hip4_workq);
	atomic_set(&hip->hip_priv->rx_ready, 0);
	atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
	/* Deactive the wd timer prior its expiration */
	del_timer_sync(&hip->hip_priv->watchdog);
}

void hip4_deinit(struct slsi_hip4 *hip)
{
	struct slsi_dev     *sdev = container_of(hip, struct slsi_dev, hip4_inst);
	struct scsc_service *service;

	if (!sdev || !sdev->service)
		return;

	wake_lock_destroy(&hip->hip_priv->hip4_wake_lock);

	service = sdev->service;

	closing = ktime_get();
	atomic_set(&hip->hip_priv->closing, 1);

	scsc_service_mifintrbit_bit_mask(service, hip->hip_priv->rx_intr_tohost);

#ifdef TASKLET
	tasklet_kill(&hip->hip_priv->intr_tq);
#else
	cancel_work_sync(&hip->hip_priv->intr_wq);
#endif
	scsc_service_mifintrbit_unregister_tohost(service, hip->hip_priv->rx_intr_tohost);

	flush_workqueue(hip->hip_priv->hip4_workq);
	destroy_workqueue(hip->hip_priv->hip4_workq);

	scsc_service_mifintrbit_free_fromhost(service, hip->hip_priv->rx_intr_fromhost, SCSC_MIFINTR_TARGET_R4);

	/* If we get to that point with rx_lock/tx_lock claimed, trigger BUG() */
	WARN_ON(atomic_read(&hip->hip_priv->in_tx));
	WARN_ON(atomic_read(&hip->hip_priv->in_rx));

	atomic_set(&hip->hip_priv->rx_ready, 0);
	atomic_set(&hip->hip_priv->watchdog_timer_active, 0);
	/* Deactive the wd timer prior its expiration */
	del_timer_sync(&hip->hip_priv->watchdog);

#ifdef CONFIG_SCSC_WLAN_DEBUG
	if (hip->hip_priv->stats.procfs_dir) {
		remove_proc_entry("driver/hip4/info", NULL);
		remove_proc_entry("driver/hip4/history", NULL);
		remove_proc_entry("driver/hip4", NULL);
	}
#endif
	kfree(hip->hip_priv);

	hip->hip_priv = NULL;
}
