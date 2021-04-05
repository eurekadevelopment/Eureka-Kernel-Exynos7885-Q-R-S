#ifndef FIMC_IS_VENDOR_CONFIG_A6E_H
#define FIMC_IS_VENDOR_CONFIG_A6E_H

#include "fimc-is-eeprom-rear-2p6_noPDAF_v001.h"
#include "fimc-is-eeprom-front-3p8sp_v003.h"

#define VENDER_PATH

#define CAMERA_MODULE_ES_VERSION_REAR 'A'
#define CAMERA_MODULE_ES_VERSION_FRONT 'A'
#define CAL_MAP_ES_VERSION_REAR '1'
#define CAL_MAP_ES_VERSION_FRONT '3'  /* VF01 version */

#define FIMC_IS_HW_SENSOR_COUNT 2

#define CAMERA_SYSFS_V2

#define USE_COMMON_CAM_IO_PWR

#define USE_SSRM_CAMERA_INFO /* Match with SAMSUNG_SSRM define of Camera Hal side */

#define EEPROM_DEBUG

#define ENABLE_REMOSAIC_CAPTURE
#define USE_AF_PWR_READ_EEPROM

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

#endif /* FIMC_IS_VENDOR_CONFIG_A6E_H */
