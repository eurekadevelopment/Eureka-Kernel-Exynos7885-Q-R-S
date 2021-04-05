#ifndef FIMC_IS_EEPROM_REAR_3L6_V001_H
#define FIMC_IS_EEPROM_REAR_3L6_V001_H

/* EEPROM I2C Addr Section */
#define EEP_I2C_HEADER_VERSION_START_ADDR               0x30
#define EEP_I2C_HEADER_CAL_MAP_VER_START_ADDR           0x40

/* Header Offset Addr Section */
#define EEP_HEADER_VERSION_START_ADDR                   0x30
#define EEP_HEADER_CAL_MAP_VER_START_ADDR               0x40
#define EEP_HEADER_OEM_START_ADDR                       0x0
#define EEP_HEADER_OEM_END_ADDR                         0x4
#define EEP_HEADER_AWB_START_ADDR                       0x8
#define EEP_HEADER_AWB_END_ADDR                         0xC
#define EEP_HEADER_AP_SHADING_START_ADDR                0x10
#define EEP_HEADER_AP_SHADING_END_ADDR                  0x14
#define EEP_HEADER_SENSOR_CAL_START_ADDR                0x18
#define EEP_HEADER_SENSOR_CAL_END_ADDR                  0x1C
#define EEP_HEADER_DUAL_DATA_START_ADDR                 0x20
#define EEP_HEADER_DUAL_DATA_END_ADDR                   0x24

/* HEADER CAL INFO */
#define EEP_HEADER_PROJECT_NAME_START_ADDR              0x48
#define EEP_HEADER_MODULE_ID_ADDR                       0xAE
#define EEP_HEADER_SENSOR_ID_ADDR                       0xB8

/* MTF DATA: AF Position & Resolution */
#define EEP_HEADER_MTF_DATA_ADDR                        0x50

/* HEADER CHECKSUM */
#define EEP_CHECKSUM_HEADER_ADDR                        0xFC
#define HEADER_CRC32_LEN                                (0xDF-0x00+1)

/***** OEM Referenced Section *****/
#define EEPROM_AF_CAL_PAN_ADDR                          0x0110
#define EEPROM_AF_CAL_MACRO_ADDR                        0x0108

#define EEP_OEM_VER_START_ADDR                          0x01E0
#define EEP_CHECKSUM_OEM_ADDR                           0x01FC
#define OEM_CRC32_LEN                                   (0x1CF-0x100+1)

/***** AWB Referenced section *****/
#define EEP_AWB_VER_START_ADDR                          0x0220
#define EEP_CHECKSUM_AWB_ADDR                           0x023C
#define AWB_CRC32_LEN                                   (0x21F-0x200+1)

/***** Shading Referenced section *****/
#define EEP_AP_SHADING_VER_START_ADDR                   0x1C30
#define EEP_CHECKSUM_AP_SHADING_ADDR                    0x1C4C
#define SHADING_CRC32_LEN                               (0x1C2F-0x0240+1)

/***** PADF CAL Referenced section *****/
//#define EEP_AP_PDAF_VER_START_ADDR                      0x2200
//#define EEP_CHECKSUM_AP_PDAF_ADDR                       0x221C


/***** DUAL DATA Referenced section *****/
#define EEP_DUAL_DATA_VER_START_ADDR                    0x2680
#define EEP_CHECKSUM_DUAL_DATA_ADDR                     0x269C
#define DUAL_DATA_CRC32_LEN                             (0x225F-0x2210+1)

/***** ETC SECTION *****/
#define FIMC_IS_MAX_CAL_SIZE                            (16 * 1024)
#endif  /* FIMC_IS_EEPROM_REAR_3L6_V001_H */
