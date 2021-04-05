#ifndef FIMC_IS_VENDOR_CONFIG_AAS_V30C_H
#define FIMC_IS_VENDOR_CONFIG_AAS_V30C_H

#define VENDER_PATH

/***** CAL ROM DEFINE *****/
#define COUNT_EXTEND_CAL_DATA   (1)             /* For searching of extend cal data name. If it is not used then set '0' */
//#define SENSOR_OTP_5E9                        /* Support read OTPROM for 5E9 */
#define ROM_DEBUG
//#define ROM_CRC32_DEBUG
//#define SKIP_CHECK_CRC                        /* Skip the CRC CHECK of cal data */


/***** SUPPORT CAMERA DEFINE *****/
#define FIMC_IS_HW_SENSOR_COUNT (4)                              /* Total count of the applied image sensor */
#define REAR_UW_CAMERA          SENSOR_POSITION_REAR2
#define REAR_BOKEH_CAMERA       SENSOR_POSITION_REAR3
#define CAMERA_REAR2            REAR_UW_CAMERA                   /* Support Rear2 */
#define CAMERA_REAR3            REAR_BOKEH_CAMERA                /* Support Rear3 */
#define USE_SHARED_ROM_REAR3                                     /* Rear3 Camera share the ROM with Rear(Wide)*/


/***** SUPPORT FUCNTION DEFINE *****/
#define SAMSUNG_LIVE_OUTFOCUS                     /* Allocate memory For Dual Camera */
#define ENABLE_REMOSAIC_CAPTURE                   /* Base Remosaic */
#define ENABLE_REMOSAIC_CAPTURE_WITH_ROTATION     /* M2M and Rotation is used during Remosaic */
//#define USE_AP_PDAF                             /* Support sensor PDAF SW Solution */
//#define USE_SENSOR_WDR                          /* Support sensor WDR */

/* VRA 1.4 improvement - adding VRA 1.4 interface : This feature is moved to fimc-is-config.h */
/* Be enable this feature for New Model since A7 2018 */
//#define ENABLE_VRA_LIBRARY_IMPROVE

//#define USE_BINARY_PADDING_DATA_ADDED           /* Apply Signed DDK/RTA Binary */

#define USE_CAMERA_ACT_DRIVER_SOFT_LANDING         /* Using NRC based softlanding for DW-9808*/

#define USE_MS_PDAF
#define USE_MS_PDAF_INTERFACE


/***** DDK - DRIVER INTERFACE *****/
#define USE_WDR_INTERFACE                         /* This feature since A7 2018 */


/***** HW DEFENDANT DEFINE *****/
#define USE_COMMON_CAM_IO_PWR
//#define DIVISION_EEP_IO_PWR                     /* Use Rear IO power for Front EEPROM i2c pull-up power */

#define ENABLE_MCSC_TDNR_VENDOR_SIZE
#if defined(ENABLE_MCSC_TDNR_VENDOR_SIZE)
#define MAX_MCSC_DNR_WIDTH		(4128)
#define MAX_MCSC_DNR_HEIGHT		(3096)
#endif


/***** SUPPORT EXTERNEL FUNCTION DEFINE *****/
#define USE_SSRM_CAMERA_INFO                      /* Match with SAMSUNG_SSRM define of Camera Hal side */

#define USE_CAMERA_HW_BIG_DATA
#ifdef USE_CAMERA_HW_BIG_DATA
//#define USE_CAMERA_HW_BIG_DATA_FOR_PANIC
#define CSI_SCENARIO_SEN_REAR	(0)               /* This value follows dtsi */
#define CSI_SCENARIO_SEN_FRONT	(1)
#endif


#endif /* FIMC_IS_VENDOR_CONFIG_A30CHN */
