/****************************************************************************
 *
 * Copyright (c) 2014 - 2017 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#ifndef __HIP4_H__
#define __HIP4_H__

/**
 * This header file is the public HIP4 interface, which will be accessible by
 * Wi-Fi service driver components.
 *
 * All struct and internal HIP functions shall be moved to a private header
 * file.
 */

#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/skbuff.h>
#include <scsc/scsc_mifram.h>
#include <scsc/scsc_mx.h>
#ifndef SLSI_TEST_DEV
#include <linux/wakelock.h>
#endif
#include "mbulk.h"

#define MIF_HIP_COMPAT_FLAG_NEED_MLME_RESET     BIT(0)
#define MIF_HIP_COMPAT_FLAG_MIB_DAT_BY_FAPI     BIT(1)

#define HIP4_DAT_SLOTS                        218
#define HIP4_CTL_SLOTS                         32

#define MIF_HIP_CFG_Q_NUM       6

/* Current versions supported by this HIP */
#define HIP4_SUPPORTED_V1	3
#define HIP4_SUPPORTED_V2	4

enum hip4_hip_q_conf {
	HIP4_MIF_Q_FH_CTRL,
	HIP4_MIF_Q_FH_DAT,
	HIP4_MIF_Q_FH_RFB,
	HIP4_MIF_Q_TH_CTRL,
	HIP4_MIF_Q_TH_DAT,
	HIP4_MIF_Q_TH_RFB
};

struct hip4_hip_config_version_4 {
	/* Host owned */
	u32 magic_number;       /* 0xcaba0401 */
	u16 hip_config_ver;     /* Version of this configuration structure = 2*/
	u16 config_len;         /* Size of this configuration structure */

	/* FW owned */
	u32 compat_flag;         /* flag of the expected driver's behaviours */

	u16 sap_mlme_ver;        /* Fapi SAP_MLME version*/
	u16 sap_ma_ver;          /* Fapi SAP_MA version */
	u16 sap_debug_ver;       /* Fapi SAP_DEBUG version */
	u16 sap_test_ver;        /* Fapi SAP_TEST version */

	u32 fw_build_id;         /* Firmware Build Id */
	u32 fw_patch_id;         /* Firmware Patch Id */

	u8  unidat_req_headroom; /* Headroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8  unidat_req_tailroom; /* Tailroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8  bulk_buffer_align;   /* 4 */

	/* Host owned */
	u8  host_cache_line;    /* 64 */

	u32 host_buf_loc;       /* location of the host buffer in MIF_ADDR */
	u32 host_buf_sz;        /* in byte, size of the host buffer */
	u32 fw_buf_loc;         /* location of the firmware buffer in MIF_ADDR */
	u32 fw_buf_sz;          /* in byte, size of the firmware buffer */
	u32 mib_loc;            /* MIB location in MIF_ADDR */
	u32 mib_sz;             /* MIB size */
	u32 log_config_loc;     /* Logging Configuration Location in MIF_ADDR */
	u32 log_config_sz;      /* Logging Configuration Size in MIF_ADDR */

	u8  mif_fh_int_n;       /* MIF from-host interrupt bit position */
	u8  mif_th_int_n;       /* MIF to-host interrpt bit position */
	u8  reserved[2];

	u32 scbrd_loc;          /* Scoreboard locatin in MIF_ADDR */

	u16 q_num;              /* 6 */
	u16 q_len;              /* 256 */
	u16 q_idx_sz;           /* 1 */
	u8  reserved2[2];

	u32 q_loc[MIF_HIP_CFG_Q_NUM];

	u8  reserved3[16];
} __packed;

struct hip4_hip_config_version_3 {
	/* Host owned */
	u32 magic_number;       /* 0xcaba0401 */
	u16 hip_config_ver;     /* Version of this configuration structure = 2*/
	u16 config_len;         /* Size of this configuration structure */

	/* FW owned */
	u32 compat_flag;         /* flag of the expected driver's behaviours */

	u16 sap_mlme_ver;        /* Fapi SAP_MLME version*/
	u16 sap_ma_ver;          /* Fapi SAP_MA version */
	u16 sap_debug_ver;       /* Fapi SAP_DEBUG version */
	u16 sap_test_ver;        /* Fapi SAP_TEST version */

	u32 fw_build_id;         /* Firmware Build Id */
	u32 fw_patch_id;         /* Firmware Patch Id */

	u8  unidat_req_headroom; /* Headroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8  unidat_req_tailroom; /* Tailroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8  bulk_buffer_align;   /* 4 */

	/* Host owned */
	u8  host_cache_line;    /* 64 */

	u32 host_buf_loc;       /* location of the host buffer in MIF_ADDR */
	u32 host_buf_sz;        /* in byte, size of the host buffer */
	u32 fw_buf_loc;         /* location of the firmware buffer in MIF_ADDR */
	u32 fw_buf_sz;          /* in byte, size of the firmware buffer */
	u32 mib_loc;            /* MIB location in MIF_ADDR */
	u32 mib_sz;             /* MIB size */
	u32 log_config_loc;     /* Logging Configuration Location in MIF_ADDR */
	u32 log_config_sz;      /* Logging Configuration Size in MIF_ADDR */

	u8  mif_fh_int_n;       /* MIF from-host interrupt bit position */
	u8  mif_th_int_n;       /* MIF to-host interrpt bit position */
	u8  reserved[2];

	u32 scbrd_loc;          /* Scoreboard locatin in MIF_ADDR */

	u16 q_num;              /* 6 */
	u16 q_len;              /* 256 */
	u16 q_idx_sz;           /* 1 */
	u8  reserved2[2];

	u32 q_loc[MIF_HIP_CFG_Q_NUM];

	u8  reserved3[16];
} __packed;

struct hip4_hip_init {
	/* Host owned */
	u32 magic_number;       /* 0xcaaa0400 */
	/* FW owned */
	u32 conf_hip4_ver;
	/* Host owned */
	u32 version_a_ref;      /* Location of Config structure A (old) */
	u32 version_b_ref;      /* Location of Config structure B (new) */
} __packed;

#define MAX_NUM 256
struct hip4_hip_q {
	u32 array[MAX_NUM];
	u8  idx_read;      /* To keep track */
	u8  idx_write;     /* To keep track */
	u8  total;
} __aligned(64);

struct hip4_hip_control {
	struct hip4_hip_init             init;
	struct hip4_hip_config_version_3 config_v3 __aligned(32);
	struct hip4_hip_config_version_4 config_v4 __aligned(32);
	u32                              scoreboard[256] __aligned(64);
	struct hip4_hip_q                q[MIF_HIP_CFG_Q_NUM] __aligned(64);
} __aligned(4096);

struct slsi_hip4;

/* #define TASKLET 1 */
/* This struct is private to the HIP implementation */
struct hip4_priv {
#ifdef TASKLET
	struct tasklet_struct        intr_tq;
#else
	struct work_struct           intr_wq;
#endif
	/* Interrupts cache */
	/* TOHOST */
	u32                          rx_intr_tohost;
	/* FROMHOST */
	u32                          rx_intr_fromhost;

	/* For workqueue */
	struct slsi_hip4             *hip;

	/* Pool for data frames*/
	u8                           host_pool_id_dat;
	/* Pool for ctl frames*/
	u8                           host_pool_id_ctl;

#ifndef TASKLET
	/* rx cycle lock */
	spinlock_t                   rx_lock;
#endif
	/* tx cycle lock */
	spinlock_t                   tx_lock;

	/* Scoreboard update spinlock */
	rwlock_t                     rw_scoreboard;

	/* Watchdog timer */
	struct timer_list            watchdog;
	/* wd spinlock */
	spinlock_t                   watchdog_lock;
	/* wd timer control */
	atomic_t                     watchdog_timer_active;

#ifndef SLSI_TEST_DEV
	/* Wakelock for modem_ctl */
	struct wake_lock             hip4_wake_lock;
#endif

	/* Control the hip4 init */
	atomic_t                     rx_ready;

	/* Control the hip4 deinit */
	atomic_t                     closing;
	atomic_t                     in_tx;
	atomic_t                     in_rx;
	atomic_t                     in_suspend;
	u32                          storm_count;

	struct {
		atomic_t	     irqs;
		atomic_t	     spurious_irqs;
		u32 q_num_frames[MIF_HIP_CFG_Q_NUM];
		ktime_t start;
		struct proc_dir_entry   *procfs_dir;
	} stats;

#ifdef CONFIG_SCSC_WLAN_HIP4_PROFILING
	/*minor*/
	u32                          minor;
#endif
	u8                           unidat_req_headroom; /* Headroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u8                           unidat_req_tailroom; /* Tailroom the host shall reserve in mbulk for MA-UNITDATA.REQ signal */
	u32                          version; /* Version of the running FW */
	void                         *scbrd_base; /* Scbrd_base pointer */

	/* Global domain Q control*/
	atomic_t                     gactive;
	atomic_t                     gmod;
	atomic_t                     gcod;
	int                          saturated;
	int                          guard;
	/* Global domain Q spinlock */
	spinlock_t                   gbot_lock;

	/* Collection artificats */
	void                         *mib_collect;
	u16                          mib_sz;
	/* Mutex to protect hcf file collection if a tear down is triggered */
	struct mutex                 in_collection;

	struct workqueue_struct *hip4_workq;
};

struct scsc_service;

struct slsi_hip4 {
	struct hip4_priv        *hip_priv;
	struct hip4_hip_control *hip_control;
	scsc_mifram_ref         hip_ref;
};

/* Public functions */
int hip4_init(struct slsi_hip4 *hip);
int hip4_setup(struct slsi_hip4 *hip);
void hip4_suspend(struct slsi_hip4 *hip);
void hip4_resume(struct slsi_hip4 *hip);
void hip4_freeze(struct slsi_hip4 *hip);
void hip4_deinit(struct slsi_hip4 *hip);
int hip4_free_ctrl_slots_count(struct slsi_hip4 *hip);

int scsc_wifi_transmit_frame(struct slsi_hip4 *hip, bool ctrl_packet, struct sk_buff *skb);

/* Macros for accessing information stored in the hip_config struct */
#define scsc_wifi_get_hip_config_version_4_u8(buff_ptr, member) le16_to_cpu((((struct hip4_hip_config_version_4 *)(buff_ptr))->member))
#define scsc_wifi_get_hip_config_version_4_u16(buff_ptr, member) le16_to_cpu((((struct hip4_hip_config_version_4 *)(buff_ptr))->member))
#define scsc_wifi_get_hip_config_version_4_u32(buff_ptr, member) le32_to_cpu((((struct hip4_hip_config_version_4 *)(buff_ptr))->member))
#define scsc_wifi_get_hip_config_version_3_u8(buff_ptr, member) le16_to_cpu((((struct hip4_hip_config_version_4 *)(buff_ptr))->member))
#define scsc_wifi_get_hip_config_version_3_u16(buff_ptr, member) le16_to_cpu((((struct hip4_hip_config_version_4 *)(buff_ptr))->member))
#define scsc_wifi_get_hip_config_version_3_u32(buff_ptr, member) le32_to_cpu((((struct hip4_hip_config_version_4 *)(buff_ptr))->member))
#define scsc_wifi_get_hip_config_u8(buff_ptr, member, ver) le16_to_cpu((((struct hip4_hip_config_version_##ver *)(buff_ptr->config_v##ver))->member))
#define scsc_wifi_get_hip_config_u16(buff_ptr, member, ver) le16_to_cpu((((struct hip4_hip_config_version_##ver *)(buff_ptr->config_v##ver))->member))
#define scsc_wifi_get_hip_config_u32(buff_ptr, member, ver) le32_to_cpu((((struct hip4_hip_config_version_##ver *)(buff_ptr->config_v##ver))->member))
#define scsc_wifi_get_hip_config_version(buff_ptr) le32_to_cpu((((struct hip4_hip_init *)(buff_ptr))->conf_hip4_ver))

#endif
