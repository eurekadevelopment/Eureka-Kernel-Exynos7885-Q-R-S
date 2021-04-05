#ifndef FIMC_IS_VENDOR_CONFIG_J2CORE_H
#define FIMC_IS_VENDOR_CONFIG_J2CORE_H

#include "fimc-is-eeprom-rear-4h5yc_v002.h"
#include "fimc-is-otprom-front-5e3_v001.h"

/* This count is defined by count of fimc_is_sensor in the dtsi file */
#define FIMC_IS_HW_SENSOR_COUNT 2

#define VENDER_PATH

#define CAMERA_MODULE_ES_VERSION_REAR 'A'
#define CAMERA_MODULE_ES_VERSION_FRONT 'A'
#define CAL_MAP_ES_VERSION_REAR '1'
#define CAL_MAP_ES_VERSION_FRONT '1'  /* VF01 version */

//#define SAMSUNG_LIVE_OUTFOCUS		/* Allocate memory For Dual Camera */

#define CAMERA_SYSFS_V2
#define USE_COMMON_CAM_IO_PWR
//#define USE_COMMON_FRONT_CAL_MAP
#define USE_SSRM_CAMERA_INFO /* Match with SAMSUNG_SSRM define of Camera Hal side */
#define EEPROM_DEBUG
//#define SKIP_CHECK_CRC

#define USE_CAMERA_HW_BIG_DATA
#ifdef USE_CAMERA_HW_BIG_DATA
/* #define USE_CAMERA_HW_BIG_DATA_FOR_PANIC */
#define CSI_SCENARIO_SEN_REAR	(0) /* This value follows dtsi */
#define CSI_SCENARIO_SEN_FRONT	(1)
#endif

/* VRA 1.4 improvement - adding VRA 1.4 interface : move from fimc-is-config.h */
/* Be enable this feature for New Model since A7 2018 */
#define ENABLE_VRA_LIBRARY_IMPROVE

#define USE_WDR_INTERFACE /* use for SAMSUNG_AUTO_HDR */

/* this define should be used after A7 2018  */
#define USE_AI_CAMERA_INTERFACE     (1)
#define USE_MFHDR_CAMERA_INTERFACE  (1)

#define USE_FACE_UNLOCK_AE_AWB_INIT /* for Face Unlock */

#endif /* FIMC_IS_VENDOR_CONFIG_J2CORE_H */
