#ifndef FIMC_IS_VENDOR_CONFIG_J7DUO_H
#define FIMC_IS_VENDOR_CONFIG_J7DUO_H

#include "fimc-is-eeprom-rear-dual_3l2_sr556_v001.h"
#include "fimc-is-eeprom-front-4h5yc_v002.h"

#define VENDER_PATH

#define CAMERA_MODULE_ES_VERSION_REAR 'A'
#define CAMERA_MODULE_ES_VERSION_FRONT 'A'
#define CAL_MAP_ES_VERSION_REAR '1'
#define CAL_MAP_ES_VERSION_FRONT '2'  /* VF01 version */

#define FIMC_IS_HW_SENSOR_COUNT 3

#define CAMERA_SYSFS_V2

#define CAMERA_REAR2               /* Support Rear2 for Dual Camera */
#define CAMERA_REAR2_SR556
#define CAMERA_REAR2_USE_COMMON_EEP /* For case that use 1 EEPROM for DUAL*/

#define SAMSUNG_LIVE_OUTFOCUS /* Allocate memory For Dual Camera */

#define USE_COMMON_CAM_IO_PWR

#define DIVISION_EEP_IO_PWR /* Use Rear IO power for Front EEPROM i2c pull-up power */

#define USE_SSRM_CAMERA_INFO /* Match with SAMSUNG_SSRM define of Camera Hal side */

#define EEPROM_DEBUG
//#define SKIP_CHECK_CRC	/* Skip the EEPROM CAL DATA CRC CHECK */

#define USE_CAMERA_HW_BIG_DATA
#ifdef USE_CAMERA_HW_BIG_DATA
/* #define USE_CAMERA_HW_BIG_DATA_FOR_PANIC */
#define CSI_SCENARIO_SEN_REAR	(0) /* This value follows dtsi */
#define CSI_SCENARIO_SEN_FRONT	(1)
#endif

/* this define should be used after A7 2018  */
#define USE_AI_CAMERA_INTERFACE     (0)
#define USE_MFHDR_CAMERA_INTERFACE  (0)

//#define USE_FACE_UNLOCK_AE_AWB_INIT /* for Face Unlock */

#endif /* FIMC_IS_VENDOR_CONFIG_J7DUO_H */
