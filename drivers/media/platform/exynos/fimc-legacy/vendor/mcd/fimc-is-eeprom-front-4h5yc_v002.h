#ifndef FIMC_IS_EEPROM_FRONT_4H5YC_V002_H
#define FIMC_IS_EEPROM_FRONT_4H5YC_V002_H

/* EEPROM I2C Addr Section */
#define EEP_I2C_HEADER_CAL_MAP_VER_START_ADDR_FRONT  0x30
#define EEP_I2C_HEADER_VERSION_START_ADDR_FRONT      0x20

/* Header Offset Addr Section */
#define EEP_HEADER_OEM_START_ADDR_FRONT              0x0
#define EEP_HEADER_OEM_END_ADDR_FRONT                0x4
#define EEP_HEADER_AWB_START_ADDR_FRONT              0x8
#define EEP_HEADER_AWB_END_ADDR_FRONT                0xC
#define EEP_HEADER_AP_SHADING_START_ADDR_FRONT       0x10
#define EEP_HEADER_AP_SHADING_END_ADDR_FRONT         0x14
#define EEP_HEADER_PROJECT_NAME_START_ADDR_FRONT     0x38
#define EEP_HEADER_CAL_MAP_VER_START_ADDR_FRONT      0x30
#define EEP_HEADER_VERSION_START_ADDR_FRONT          0x20
#define EEP_HEADER_MODULE_ID_ADDR_FRONT              0xAE

/* OEM referenced section */
#define EEP_OEM_VER_START_ADDR_FRONT                 0x1E0

/* AWB referenced section */
#define EEP_AWB_VER_START_ADDR_FRONT                 0x2E0

/* AP Shading referenced section */
#define EEP_AP_SHADING_VER_START_ADDR_FRONT          0x1FE0

/* Checksum referenced section */
#define EEP_CHECKSUM_HEADER_ADDR_FRONT               0xFC
#define EEP_CHECKSUM_OEM_ADDR_FRONT                  0x1FC
#define EEP_CHECKSUM_AWB_ADDR_FRONT                  0x2FC
#define EEP_CHECKSUM_AP_SHADING_ADDR_FRONT           0x1FFC

/* etc section */
#define FIMC_IS_MAX_CAL_SIZE_FRONT                   (8 * 1024)
#define HEADER_CRC32_LEN_FRONT                       (216)
#define OEM_CRC32_LEN_FRONT                          (192)
#define AWB_CRC32_LEN_FRONT                          (32)
#define SHADING_CRC32_LEN_FRONT                      (6640)

#endif /* FIMC_IS_EEPROM_FRONT_4H5YC_V002_H */
