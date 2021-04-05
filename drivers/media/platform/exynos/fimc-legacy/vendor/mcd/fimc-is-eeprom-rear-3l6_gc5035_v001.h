#ifndef FIMC_IS_EEPROM_REAR_3L6_GC5035_V001_H
#define FIMC_IS_EEPROM_REAR_3L6_GC5035_V001_H

/* EEPROM I2C Addr Section */
#define EEP_I2C_HEADER_VERSION_START_ADDR        0x28
#define EEP_I2C_HEADER_VERSION_START_ADDR_REAR2  0x38
#define EEP_I2C_HEADER_CAL_MAP_VER_START_ADDR    0x48

/* Header Offset Addr Section */
#define EEP_HEADER_VERSION_START_ADDR                   0x28
#define EEP_HEADER_VERSION_START_ADDR_REAR2             0x38
#define EEP_HEADER_CAL_MAP_VER_START_ADDR               0x48
#define EEP_HEADER_CAL_MAP_VER_START_ADDR_REAR2         0x38

#define EEP_HEADER_OEM_START_ADDR                       0x0
#define EEP_HEADER_OEM_END_ADDR                         0x4
#define EEP_HEADER_AWB_START_ADDR                       0x8
#define EEP_HEADER_AWB_END_ADDR                         0xC
#define EEP_HEADER_AP_SHADING_START_ADDR                0x10
#define EEP_HEADER_AP_SHADING_END_ADDR                  0x14
//#define EEP_HEADER_AP_PDAF_START_ADDR                 0x18
//#define EEP_HEADER_AP_PDAF_END_ADDR                   0x1C
#define EEP_HEADER_DUAL_DATA_START_ADDR                 0x20
#define EEP_HEADER_DUAL_DATA_END_ADDR                   0x24

/* HEADER CAL INFO */
#define EEP_HEADER_PROJECT_NAME_START_ADDR              0x54
#define EEP_HEADER_MODULE_ID_ADDR                       0xAE
#define EEP_HEADER_SENSOR_ID_ADDR                       0xB8
#define EEP_HEADER_SENSOR_ID_ADDR_REAR2                 0xC8


/* MTF DATA: AF Position & Resolution */
#define EEP_HEADER_MTF_DATA_ADDR                        0x5C

/* HEADER CHECKSUM */
#define EEP_CHECKSUM_HEADER_ADDR                        0xFC
#define HEADER_CRC32_LEN                                ((0x00EF-0x0000)+0x1)

/***** OEM Referenced Section *****/
#define EEPROM_AF_CAL_PAN_ADDR                          0x0100
#define EEPROM_AF_CAL_MACRO_ADDR                        0x0108

#define EEP_OEM_VER_START_ADDR                          0x01D2
#define EEP_CHECKSUM_OEM_ADDR                           0x01FC
#define OEM_CRC32_LEN                                   ((0x01D1-0x0100)+0x1)

/***** AWB Referenced section *****/
#define EEP_AWB_VER_START_ADDR                          0x0220
#define EEP_CHECKSUM_AWB_ADDR                           0x023C
#define AWB_CRC32_LEN                                  ((0x021F-0x0200)+0x1)

/***** Shading Referenced section *****/
#define EEP_AP_SHADING_VER_START_ADDR                   0x1C30
#define EEP_CHECKSUM_AP_SHADING_ADDR                    0x1C4C
#define SHADING_CRC32_LEN                               ((0x1C2F-0x0240)+0x1)

/******* PADF Cal Data ******/
//#define EEP_AP_PDAF_VER_START_ADDR                      0x1FE0
//#define EEP_CHECKSUM_AP_PDAF_ADDR                       0x1FFC
//#define AP_PDAF_CRC32_LEN                               ((0x21FF-0x1C50)+0x1)

/***** DUAL DATA Referenced section *****/
#define EEP_DUAL_DATA_VER_START_ADDR                    0x2680
#define EEP_CHECKSUM_DUAL_DATA_ADDR                     0x269C
#define DUAL_DATA_CRC32_LEN                             ((0x267F-0x2220)+0x1)

/***** ETC SECTION *****/
#define FIMC_IS_MAX_CAL_SIZE                            (16 * 1024)

/***** REAR2 Cal Dual Calibration Data2 *****/
#define EEP_DUAL_CAL_DATA2_ADDR                  0x2224
#define EEP_DUAL_CAL_DATA2_SIZE                  (512)

#define EEP_REAR2_DUAL_TILT_X                    0x2280
#define EEP_REAR2_DUAL_TILT_Y                    0x2284
#define EEP_REAR2_DUAL_TILT_Z                    0x2288
#define EEP_REAR2_DUAL_TILT_SX                   0x22E0
#define EEP_REAR2_DUAL_TILT_SY                   0x22E4
#define EEP_REAR2_DUAL_TILT_RANGE                0x2504
#define EEP_REAR2_DUAL_TILT_MAX_ERR              0x2508
#define EEP_REAR2_DUAL_TILT_AVG_ERR              0x250C
#define EEP_REAR2_DUAL_TILT_DLL_VERSION          0x2220

#endif  /* FIMC_IS_EEPROM_REAR_3L6_GC5035_V001_H */
