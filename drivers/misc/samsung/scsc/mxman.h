/****************************************************************************
 *
 * Copyright (c) 2014 - 2019 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#ifndef _MAXWELL_MANAGER_H
#define _MAXWELL_MANAGER_H
#include <linux/workqueue.h>
#include "fwhdr.h"
#include "mxmgmt_transport.h"
#include "mxproc.h"
#include "scsc_mx.h"
#include <linux/wakelock.h>

struct mxman;

void mxman_init(struct mxman *mxman, struct scsc_mx *mx);
void mxman_deinit(struct mxman *mxman);
int mxman_open(struct mxman *mxman);
void mxman_close(struct mxman *mxman);
void mxman_fail(struct mxman *mxman, u16 scsc_panic_code, const char *reason);
void mxman_freeze(struct mxman *mxman);
int mxman_force_panic(struct mxman *mxman);
int mxman_suspend(struct mxman *mxman);
void mxman_resume(struct mxman *mxman);
void mxman_show_last_panic(struct mxman *mxman);

enum mxman_state {
	MXMAN_STATE_STOPPED,
	MXMAN_STATE_STARTING,
	MXMAN_STATE_STARTED,
	MXMAN_STATE_FAILED,
	MXMAN_STATE_FREEZED,
};

#define SCSC_FAILURE_REASON_LEN 256

struct mxman {
	struct scsc_mx          *mx;
	int                     users;
	void                    *start_dram;
	struct workqueue_struct *fw_crc_wq;
	struct delayed_work     fw_crc_work;
	struct workqueue_struct *failure_wq;
	struct work_struct      failure_work;
	char                    *fw;
	u32                     fw_image_size;
	struct completion       mm_msg_start_ind_completion;
	struct completion       mm_msg_halt_rsp_completion;
	struct fwhdr            fwhdr;
	struct mxconf           *mxconf;
	enum mxman_state        mxman_state;
	enum mxman_state        mxman_next_state;
	struct mutex            mxman_mutex;
	struct mxproc           mxproc;
	int			suspended;
	atomic_t		suspend_count;
	atomic_t                recovery_count;
	atomic_t                boot_count;
	bool			check_crc;
	char                    fw_build_id[FW_BUILD_ID_SZ]; /* Defined in SC-505846-SW */
	struct completion       recovery_completion;
	struct wake_lock	recovery_wake_lock;
	u32			rf_hw_ver;
	u16			scsc_panic_code;
	u64			last_panic_time;
	u32			last_panic_rec_r[PANIC_RECORD_SIZE]; /* Must be at least SCSC_R4_V2_MINOR_53 */
	u16			last_panic_rec_sz;
	char			failure_reason[SCSC_FAILURE_REASON_LEN]; /* previous failure reason */
};

void mxman_register_gdb_channel(struct scsc_mx *mx, mxmgmt_channel_handler handler, void *data);
void mxman_send_gdb_channel(struct scsc_mx *mx, void *data, size_t length);

#ifdef CONFIG_SCSC_CHV_SUPPORT
#define SCSC_CHV_ARGV_ADDR_OFFSET 0x200008

extern int chv_run;
#endif

#define SCSC_SYSERR_HOST_SERVICE_SHIFT 4

#endif
