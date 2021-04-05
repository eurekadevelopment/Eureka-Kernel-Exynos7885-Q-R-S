#ifndef FIMC_IS_EEPROM_REAR_3L2_V001_H
#define FIMC_IS_EEPROM_REAR_3L2_V001_H

/*temporary added for sr556*/
#define FROM_REAR2_DUAL_CAL2                                    0x6BC0
#define FROM_REAR2_DUAL_CAL2_SIZE                                512

/* EEPROM I2C Addr Section */
#define EEP_I2C_HEADER_VERSION_START_ADDR      0x20
#define EEP_I2C_HEADER_CAL_MAP_VER_START_ADDR  0x30

/* Header Offset Addr Section */
#define EEP_HEADER_VERSION_START_ADDR      0x20
#define EEP_HEADER_CAL_MAP_VER_START_ADDR  0x30
#define EEP_HEADER_OEM_START_ADDR          0x0
#define EEP_HEADER_OEM_END_ADDR            0x4
#define EEP_HEADER_AWB_START_ADDR          0x8
#define EEP_HEADER_AWB_END_ADDR            0xC
#define EEP_HEADER_AP_SHADING_START_ADDR   0x10
#define EEP_HEADER_AP_SHADING_END_ADDR	   0x14
#define EEP_HEADER_PROJECT_NAME_START_ADDR 0x38
#define EEP_HEADER_MODULE_ID_ADDR          0xAE

/* OEM referenced section */
#define EEP_OEM_VER_START_ADDR             0x1E0

/* AWB referenced section */
#define EEP_AWB_VER_START_ADDR             0x220

/* AP Shading referenced section */
#define EEP_AP_SHADING_VER_START_ADDR      0x1FE0

/* Checksum referenced section */
#define EEP_CHECKSUM_HEADER_ADDR           0xFC
#define EEP_CHECKSUM_OEM_ADDR              0x1FC
#define EEP_CHECKSUM_AWB_ADDR              0x2FC
#define EEP_CHECKSUM_AP_SHADING_ADDR       0x1FFC

#if defined(CONFIG_CAMERA_EEPROM_SUPPORT_OIS)
#define EEP_HEADER_OIS_START_ADDR          0x2000
#define EEP_HEADER_OIS_CAL_START_ADDR      0x2000
#define EEP_HEADER_OIS_CAL_END_ADDR        0x2004
#define EEP_HEADER_OIS_SHIFT_START_ADDR    0x2008
#define EEP_HEADER_OIS_SHIFT_END_ADDR      0x200C
#define EEP_HEADER_OIS_FW_SET_START_ADDR   0x2010
#define EEP_HEADER_OIS_FW_SET_END_ADDR     0x2014
#define EEP_HEADER_OIS_FW_FACTORY_START_ADDR 0x2018
#define EEP_HEADER_OIS_FW_FACTORY_END_ADDR 0x201C

#define EEP_HEADER_OIS_FW_VER_START_ADDR   0x2040
#define EEP_HEADER_OIS_CAL_VER_START_ADDR  0x2048
#define EEP_HEADER_OIS_CHIP_INFO_START_ADDR 0x2050
#define EEP_HEADER_OIS_ADJ_FACTOR_START_ADDR 0x2060

#define EEP_HEADER_OIS_CAL_CHECKSUM_ADDR   0x2100
#define EEP_HEADER_OIS_CAL_OFFSET_ADDR     0x2104
#define EEP_HEADER_OIS_CAL_SIZE_ADDR       0x2106
#define EEP_HEADER_OIS_CAL_TARGET_ADDR     0x2108
#define EEP_HEADER_OIS_CAL_DATA_ADDR       0x2110
#define EEP_HEADER_OIS_SHIFT_DATA_ADDR     0x21A0

/* Checksum referenced section */
#define EEP_CHECKSUM_OIS_HEADER_ADDR       0x20FC
#define EEP_CHECKSUM_OIS_CAL_ADDR          0x219C
#define EEP_CHECKSUM_OIS_SHIFT_ADDR        0x21FC
#define EEP_CHECKSUM_OIS_FW_SET_ADDR       0x25FC
#define EEP_CHECKSUM_OIS_FW_FACTORY_ADDR   0x37FC
#endif

/* etc section */
#if defined(CONFIG_CAMERA_EEPROM_SUPPORT_OIS)
#define FIMC_IS_MAX_CAL_SIZE               (16 * 1024)
#define FIMC_IS_MAX_FW_SIZE                (16 * 1024)
#define FIMC_IS_MAX_OIS_SIZE               (12 * 1024)
#define OIS_HEADER_CRC32_LEN               (0x80)
#define OIS_HEADER_CRC32_CAL_LEN           (0x60)
#define OIS_HEADER_CRC32_SHIFT_LEN         (0x30)
#define OIS_HEADER_CRC32_FW_SET_LEN        (0x3FC)
#define OIS_HEADER_CRC32_FW_FACTORY_LEN    (0x11FC)
#else
#define FIMC_IS_MAX_CAL_SIZE               (8 * 1024)
#define FIMC_IS_MAX_FW_SIZE                (8 * 1024)
#endif
#define FIMC_IS_MAX_SETFILE_SIZE           (1120 * 1024)
#define HEADER_CRC32_LEN                   (216)
#define EEPROM_AF_CAL_PAN_ADDR             0x0100
#define EEPROM_AF_CAL_MACRO_ADDR           0x0108

#endif /* FIMC_IS_EEPROM_REAR_3L2_V001_H */
