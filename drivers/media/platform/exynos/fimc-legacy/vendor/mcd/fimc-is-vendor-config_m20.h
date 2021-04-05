#ifndef FIMC_IS_VENDOR_CONFIG_M20_H
#define FIMC_IS_VENDOR_CONFIG_M20_H

/***** HISTORY *****
 * < HW_REV 00 >
 *  rear eeprom version v001 : fimc-is-eeprom-rear-imx576_v001.h
 *  CAL_MAP_ES_VERSION_REAR  : 1
 *
 * < HW_REV 01 >
 *  rear eeprom version v002 : fimc-is-eeprom-rear-imx576_v002.h
 *  CAL_MAP_ES_VERSION_REAR  : 2
 */

#include "fimc-is-eeprom-rear-3l6_v001.h"
#include "fimc-is-eeprom-front-4ha_v001.h"
#include "fimc-is-eeprom-rear-5e9_v001.h"

#define VENDER_PATH

#define CAMERA_MODULE_ES_VERSION_REAR 'A'
#define CAMERA_MODULE_ES_VERSION_REAR3 'A'

#define CAMERA_MODULE_ES_VERSION_FRONT 'A'

#define CAL_MAP_ES_VERSION_REAR '1'
#define CAL_MAP_ES_VERSION_REAR2 '1'

#define CAL_MAP_ES_VERSION_FRONT '1'

#define FIMC_IS_HW_SENSOR_COUNT 3

#define CAMERA_SYSFS_V2

#define USE_MS_PDAF
#define USE_MS_PDAF_INTERFACE

#define CAMERA_REAR3

#define USE_COMMON_CAM_IO_PWR

#define USE_SSRM_CAMERA_INFO		/* Match with SAMSUNG_SSRM define of Camera Hal side */

#define EEPROM_DEBUG

#define USE_WDR_INTERFACE

#define USE_CAMERA_ACT_DRIVER_SOFT_LANDING 	/* Using NRC based softlanding for DW-9808*/

#define USE_CAMERA_HW_BIG_DATA
#ifdef USE_CAMERA_HW_BIG_DATA
/* #define USE_CAMERA_HW_BIG_DATA_FOR_PANIC */
#define CSI_SCENARIO_SEN_REAR	(0) /* This value follows dtsi */
#define CSI_SCENARIO_SEN_FRONT	(1)
#define CSI_SCENARIO_SEN_REAR3	(2)
#endif

#define ADJUST_MCSC_DNR_SIZE
#ifdef ADJUST_MCSC_DNR_SIZE
#define ADJUSTED_MCSC_DNR_WIDTH  (4128)
#define ADJUSTED_MCSC_DNR_HEIGHT (3096)
#endif /* ADJUST_MCSC_DNR_SIZE */

/* VRA 1.4 improvement - adding VRA 1.4 interface : move from fimc-is-config.h */
/* Be enable this feature for New Model since A7 2018 */
#define ENABLE_VRA_LIBRARY_IMPROVE

/* this define should be used after A7 2018  */
#define USE_AI_CAMERA_INTERFACE     (1)
#define USE_MFHDR_CAMERA_INTERFACE  (1)

#define USE_FACE_UNLOCK_AE_AWB_INIT /* for Face Unlock */
#endif /* FIMC_IS_VENDOR_CONFIG_M20_H */
