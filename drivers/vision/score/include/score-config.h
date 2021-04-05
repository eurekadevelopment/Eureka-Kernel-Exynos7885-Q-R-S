/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef SCORE_CONFIG_H_
#define SCORE_CONFIG_H_

/*
 * =================================================================================================
 * CONFIG - GLOBAL OPTIONS
 * =================================================================================================
 */

#define SCORE_MAX_VERTEX			(16)
#define SCORE_MAX_BUFFER			(16)
#define SCORE_MAX_PLANE				(3)

#define SCORE_MAX_GRAPH				(SCORE_MAX_VERTEX)
#define SCORE_MAX_FRAME				(SCORE_MAX_BUFFER)
#define SCORE_MAX_GFRAME				(SCORE_MAX_VERTEX * SCORE_MAX_FRAME)

#define SCORE_TIME_TICK				(5)

#define SCORE_CAM_L5				(640000)
#define SCORE_MIF_L6				(845000)

/*
 * =================================================================================================
 * CONFIG -PLATFORM CONFIG
 * =================================================================================================
 */

/*
 * =================================================================================================
 * CONFIG - FEATURE ENABLE
 * =================================================================================================
 */

/* #define DISABLE_CMU */
/* #define DISABLE_CLK_OP */

#define ENABLE_SYSFS_SYSTEM
#define ENABLE_SYSFS_STATE

/*
 * =================================================================================================
 * CONFIG - DEBUG OPTIONS
 * =================================================================================================
 */

#define PROBE_ALLOC_INTERNAL_MEM

/* #define DBG_STREAMING */
#define DBG_HISTORY
/* #define DBG_MAILBOX */
#define DBG_TIMEMEASURE
#define DBG_IMGDUMP
#define DBG_MARKING
/* #define DBG_HW_SFR */
/* #define DBG_CALL_PATH_LOG */
#define DBG_SYSTEM_LOG
#define DBG_PER_FRAME_LOG
/* #define DBG_FRAME_STATE */
/* #define DBG_FRAME_STATE_CALLSTACK */
/* #define DBG_ISR */
/* #define DBG_UNUSE_THREAD */
/* #define DBG_USE_NO_MEMCPY */
/* #define DBG_SYSMMU_SETTING */

#define ENABLE_DBG_EVENT
#define ENABLE_DBG_FS

#define VERBOSE_WRITE
/* #define BUF_MAP_KVADDR */
/* #define BUF_DUMP_KVADDR */

/* #define DEBUG_LOG_MEMORY */

#define probe_info(fmt, ...)		pr_info("[V]" fmt, ##__VA_ARGS__)
#define probe_warn(fmt, args...)	pr_warning("[V][WRN]" fmt, ##args)
#define probe_err(fmt, args...) 	pr_err("[V][ERR]%s:%d:" fmt, __func__, __LINE__, ##args)

#ifdef DEBUG_LOG_MEMORY
#define score_err_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define score_warn_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define score_info_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define score_dbg_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
#define score_note_target(fmt, ...)
#else
#define score_err_target(fmt, ...)	pr_err(fmt, ##__VA_ARGS__)
#define score_warn_target(fmt, ...)	pr_warning(fmt, ##__VA_ARGS__)
#define score_info_target(fmt, ...)	printk(KERN_DEBUG fmt, ##__VA_ARGS__)
/* #define score_info_target(fmt, ...)	pr_info(fmt, ##__VA_ARGS__) */
#define score_dbg_target(fmt, ...)	pr_info(fmt, ##__VA_ARGS__)
#define score_note_target(fmt, ...)
#endif

#define score_err(fmt, args...) \
	score_err_target("[V][ERR][%s:%d]:" fmt, __func__, __LINE__, ##args)

#define score_serr(fmt, vctx, args...) \
	score_err_target("[V][I%d][ERR][%s:%d]:" fmt, vctx->id, __func__, __LINE__, ##args)

#define score_ierr(fmt, vctx, args...) \
	score_err_target("[V][I%d][ERR][%s:%d]:" fmt, vctx->id, __func__, __LINE__, ##args)

#define score_irerr(fmt, vctx, frame, args...) \
	score_err_target("[V][I%d][F%d][ERR][%s:%d]:" fmt, vctx->id, frame->id, __func__, __LINE__, ##args)

#define score_warn(fmt, args...) \
	score_warn_target("[V][WRN][%s:%d]:" fmt, __func__, __LINE__, ##args)

#define score_swarn(fmt, vctx, args...) \
	score_warn_target("[V][I%d][WRN][%s:%d]:" fmt, vctx->id, __func__, __LINE__, ##args)

#define score_iwarn(fmt, vctx, args...) \
	score_warn_target("[V][I%d][WRN][%s:%d]:" fmt, vctx->id, __func__, __LINE__, ##args)

#define score_irwarn(fmt, vctx, frame, args...) \
	score_warn_target("[V][I%d][F%d][WRN][%s:%d]:" fmt, vctx->id, frame->id, __func__, __LINE__, ##args)

#define score_info(fmt, args...) \
	score_info_target("[V][INF][%s:%d]:" fmt, __func__, __LINE__, ##args)

#define score_sinfo(fmt, vctx, args...) \
	score_info_target("[V][I%d]" fmt, vctx->id, ##args)

#define score_iinfo(fmt, vctx, args...) \
	score_note_target("[V][I%d]" fmt, vctx->id, ##args)

#define score_irinfo(fmt, vctx, frame, args...) \
	score_info_target("[V][I%d][F%d]" fmt, vctx->id, frame->id, ##args)

#define score_dbg(fmt, args...) \
	score_dbg_target("[V][%s:%d]:" fmt, __func__, __LINE__, ##args)

#define score_debug(fmt, args...) \
	score_dbg_target("[V][%s:%d]:" fmt, __func__, __LINE__, ##args)

#define score_sdbg(fmt, vctx, args...) \
	score_dbg_target("[V][I%d]" fmt, vctx->id, ##args)

#define score_idbg(fmt, vctx, args...) \
	score_dbg_target("[V][I%d]" fmt, vctx->id, ##args)

#define score_irdbg(fmt, vctx, frame, args...) \
	score_dbg_target("[V][I%d][F%d]" fmt, vctx->id, frame->id, ##args)

#define score_note(fmt, args...) \
	score_note_target("[V][NOTE][%s:%d]:" fmt, __func__, __LINE__, ##args)

#endif

#ifdef DBG_CALL_PATH_LOG
#define SCORE_TP() pr_info("[V][%s:%d]\n", __func__,__LINE__)
#define SCORE_P_TP() pr_info("[V][%s:%d]\n", __func__,__LINE__)
#else
#define SCORE_TP() score_event_msg("\n")
#define SCORE_P_TP()
#endif
