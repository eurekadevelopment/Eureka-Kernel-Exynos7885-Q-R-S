#ifndef FIMC_IS_VENDOR_CONFIG_JACKPOT_H
#define FIMC_IS_VENDOR_CONFIG_JACKPOT_H

#include "fimc-is-eeprom-rear-2p6_v003.h"
#include "fimc-is-eeprom-front-3p8sp_sr846_vf05.h"

/* This count is defined by count of fimc_is_sensor in the dtsi file */
#define FIMC_IS_HW_SENSOR_COUNT 3

#define VENDER_PATH

#define CAMERA_MODULE_ES_VERSION_REAR 'A'
#define CAMERA_MODULE_ES_VERSION_FRONT 'A'
#define CAL_MAP_ES_VERSION_REAR '3'
//#define CAL_MAP_ES_VERSION_FRONT '1'  /* VF01 version */
#define CAL_MAP_ES_VERSION_FRONT '5'  /* VF05 version */

#define CAMERA_SYSFS_V2

#define USE_AP_PDAF                 /* Support sensor PDAF SW Solution */
#define USE_SENSOR_WDR              /* Support sensor WDR */
#define CAMERA_FRONT2               /* Support Front2 for Dual Camera */
#define CAMERA_FRONT2_SR846
#define USE_COMMON_CAM_IO_PWR
#define USE_COMMON_FRONT_CAL_MAP

#define SAMSUNG_LIVE_OUTFOCUS /* Allocate memory For Dual Camera */

#define USE_SSRM_CAMERA_INFO /* Match with SAMSUNG_SSRM define of Camera Hal side */

#define EEPROM_DEBUG

#define ENABLE_REMOSAIC_CAPTURE

#define USE_CAMERA_HW_BIG_DATA
#ifdef USE_CAMERA_HW_BIG_DATA
/* #define USE_CAMERA_HW_BIG_DATA_FOR_PANIC */
#define CSI_SCENARIO_SEN_REAR	(0) /* This value follows dtsi */
#define CSI_SCENARIO_SEN_FRONT	(1)
#endif

/* this define should be used after A7 2018  */
#define USE_AI_CAMERA_INTERFACE     (0)
#define USE_MFHDR_CAMERA_INTERFACE  (0)

#define USE_FACE_UNLOCK_AE_AWB_INIT /* for Face Unlock */

#endif /* FIMC_IS_VENDOR_CONFIG_JACKPOT_H */
