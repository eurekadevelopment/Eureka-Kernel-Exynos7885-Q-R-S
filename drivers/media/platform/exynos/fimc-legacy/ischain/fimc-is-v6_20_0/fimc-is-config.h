/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_CONFIG_H
#define FIMC_IS_CONFIG_H

#include <fimc-is-common-config.h>
#include <fimc-is-vendor-config.h>

/*
 * =================================================================================================
 * CONFIG -PLATFORM CONFIG
 * =================================================================================================
 */

#define SOC_30S
#define SOC_30C
#define SOC_30P
#define SOC_I0S
#define SOC_I0C
#define SOC_I0P
#define SOC_31S
#define SOC_31C
#define SOC_31P
/* #define SOC_I1S */
/* #define SOC_I1C */
/* #define SOC_I1P */
/* #define SOC_DRC */
/* #define SOC_D0S */
/* #define SOC_D0C */
/* #define SOC_D1S */
/* #define SOC_D1C */
/* #define SOC_ODC */
#define SOC_DNR
/* #define SOC_SCC */
/* #define SOC_SCP */
#define SOC_MCS
#define SOC_VRA
#define SOC_SSVC0
#define SOC_SSVC1
#define SOC_SSVC2
#define SOC_SSVC3
/* #define SOC_DCP *//* TODO */
/* #define SOC_SRDZ *//* TODO */

/* Post Processing Configruation */
/* #define ENABLE_DRC */
/* #define ENABLE_DIS */
/* #define ENABLE_DNR_IN_TPU */
#define ENABLE_DNR_IN_MCSC
#define ENABLE_DNR_COMPRESSOR_IN_MCSC
#define MCSC_TDNR_YIC_MODE	(0)	/* enc/dec_mode> 0: compression, 1: raw mode */
#define ENABLE_10BIT_MCSC
/* #define ENABLE_DJAG_IN_MCSC */
#define ENABLE_VRA
/*#define ENABLE_VRA_LIBRARY_IMPROVE*/ /* This feature will be defined in vendor config of each model */
#if defined(ENABLE_VRA_LIBRARY_IMPROVE)
#define ENABLE_VRA_CHANGE_SETFILE_PARSING
#undef VRA_OLD_POSES
#else
#undef ENABLE_VRA_CHANGE_SETFILE_PARSING
#define VRA_OLD_POSES
#endif

#define USE_ONE_BINARY
#define USE_RTA_BINARY
#define USE_DDK_SHUT_DOWN_FUNC
#define ENABLE_IRQ_MULTI_TARGET
#define FIMC_IS_ONLINE_CPU_MIN	4
/* #define USE_MCUCTL */

/* #define SOC_3AAISP */
#define SOC_MCS0
/* #define SOC_MCS1 */
/* #define SOC_TPU0 */
/* #define SOC_TPU1 */

#define HW_SLOT_MAX            (5)
#define valid_hw_slot_id(slot_id) \
       (0 <= slot_id && slot_id < HW_SLOT_MAX)
/* #define DISABLE_SETFILE */
/* #define DISABLE_LIB */

#define USE_SENSOR_IF_DPHY
#undef ENABLE_CLOCK_GATE
/* #define ENABLE_DIRECT_CLOCK_GATE */

/*
 * =================================================================================================
 * CONFIG - FEATURE ENABLE
 * =================================================================================================
 */

#define FIMC_IS_MAX_TASK		(40)

#undef OVERFLOW_PANIC_ENABLE_ISCHAIN
#if defined(CONFIG_ARM_EXYNOS7885_BUS_DEVFREQ)
#define CONFIG_FIMC_IS_BUS_DEVFREQ
#endif
#define DDK_OVERFLOW_RECOVERY		(1)	/* 0: do not execute recovery, 1: execute recovery */
#define CAPTURE_NODE_MAX		6
#define OTF_YUV_FORMAT			(OTF_INPUT_FORMAT_YUV422)
#define USE_YUV_RANGE_BY_ISP
/* #define ENABLE_3AA_DMA_CROP */
/* #define ENABLE_ULTRA_FAST_SHOT */
#define ENABLE_HWFC
/* #define FW_SUSPEND_RESUME */
/* #define TPU_COMPRESSOR */
/* #define USE_I2C_LOCK */
#undef ENABLE_FULL_BYPASS
/* #define SENSOR_CONTROL_DELAY		2 */

#ifdef ENABLE_IRQ_MULTI_TARGET
#define FIMC_IS_HW_IRQ_FLAG     IRQF_GIC_MULTI_TARGET
#else
#define FIMC_IS_HW_IRQ_FLAG     0
#endif

/* #define MULTI_SHOT_KTHREAD */
/* #define ENABLE_EARLY_SHOT */

#ifdef USE_I2C_LOCK
#define I2C_MUTEX_LOCK(lock)	mutex_lock(lock)
#define I2C_MUTEX_UNLOCK(lock)	mutex_unlock(lock)
#else
#define I2C_MUTEX_LOCK(lock)
#define I2C_MUTEX_UNLOCK(lock)
#endif

#ifdef USE_FACE_UNLOCK_AE_AWB_INIT
/* init AWB */
#define ENABLE_INIT_AWB
#define WB_GAIN_COUNT		(4)
#define INIT_AWB_COUNT_REAR	(3)
#define INIT_AWB_COUNT_FRONT	(7)
#endif

/* #define ENABLE_DBG_EVENT_PRINT */

#endif
