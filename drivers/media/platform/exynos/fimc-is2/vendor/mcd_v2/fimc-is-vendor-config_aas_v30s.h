#ifndef FIMC_IS_VENDOR_CONFIG_AAS_V30S_H
#define FIMC_IS_VENDOR_CONFIG_AAS_V30S_H

#define VENDER_PATH

/***** CAL ROM DEFINE *****/
#define VENDER_CAL_STRUCT_VER2                            /* For USE the extend cal data ver2 */
#define ROM_DEBUG
//#define ROM_CRC32_DEBUG
//#define SKIP_CHECK_CRC                                  /* Skip the CRC CHECK of cal data */
//#define USE_AF_PWR_READ_EEPROM

#ifdef VENDER_CAL_STRUCT_VER2
#define USE_AE_CAL                                        /* USE Ae cal data in rear camera, Only VENDER_CAL_STRUCT_VER2 */
#endif

/***** SUPPORT CAMERA DEFINE *****/
#define FIMC_IS_HW_SENSOR_COUNT (4)                       /* Total count of the applied image sensor */
#define REAR_UW_CAMERA          SENSOR_POSITION_REAR2
#define REAR_BOKEH_CAMERA       SENSOR_POSITION_REAR3
#define CAMERA_REAR2            REAR_UW_CAMERA            /* Support Rear2 */
#define CAMERA_REAR3            REAR_BOKEH_CAMERA         /* Support Rear3 */
#define USE_SHARED_ROM_REAR3                              /* Rear3 Camera share the ROM with Rear(Wide)*/


/***** SUPPORT FUCNTION DEFINE *****/
#define SAMSUNG_LIVE_OUTFOCUS                             /* Allocate memory For Dual Camera */
#define ENABLE_REMOSAIC_CAPTURE                           /* Base Remosaic */
#define ENABLE_REMOSAIC_CAPTURE_WITH_ROTATION             /* M2M and Rotation is used during Remosaic */
#define USE_AP_PDAF                                       /* Support sensor PDAF SW Solution */
//#define USE_SENSOR_WDR                                  /* Support sensor WDR */

/* VRA 1.4 improvement - adding VRA 1.4 interface : This feature is moved to fimc-is-config.h */
/* Be enable this feature for New Model since A7 2018 */
//#define ENABLE_VRA_LIBRARY_IMPROVE

/* This feature since A50s(ramen) and A30s(lassen).
   In previous projects, only A30 / A20 / A20p is applied for sensor duplication */
#define USE_DEBUG_LIBRARY_VER

//#define USE_BINARY_PADDING_DATA_ADDED                   /* Apply Signed DDK/RTA Binary */

#define USE_CAMERA_ACT_DRIVER_SOFT_LANDING                /* Using NRC based softlanding for DW-9808*/

/***** DDK - DRIVER INTERFACE *****/
#define USE_WDR_INTERFACE                                 /* This feature since A7 2018 */


/***** HW DEFENDANT DEFINE *****/
#define USE_COMMON_CAM_IO_PWR
//#define DIVISION_EEP_IO_PWR                             /* Use Rear IO power for Front EEPROM i2c pull-up power */

#define ENABLE_MCSC_TDNR_VENDOR_SIZE
#if defined(ENABLE_MCSC_TDNR_VENDOR_SIZE)
#define MAX_MCSC_DNR_WIDTH		(4128)
#define MAX_MCSC_DNR_HEIGHT		(3096)
#endif


/***** SUPPORT EXTERNEL FUNCTION DEFINE *****/
#define USE_SSRM_CAMERA_INFO                              /* Match with SAMSUNG_SSRM define of Camera Hal side */

#define USE_CAMERA_HW_BIG_DATA
#ifdef USE_CAMERA_HW_BIG_DATA
//#define USE_CAMERA_HW_BIG_DATA_FOR_PANIC
#define CSI_SCENARIO_SEN_REAR	(0)                       /* This value follows dtsi */
#define CSI_SCENARIO_SEN_FRONT	(1)
#endif

#endif /* FIMC_IS_VENDOR_CONFIG_AAS_V30S_H */
