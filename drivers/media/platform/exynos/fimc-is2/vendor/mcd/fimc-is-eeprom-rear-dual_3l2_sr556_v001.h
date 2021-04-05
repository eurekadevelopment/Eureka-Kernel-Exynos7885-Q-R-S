#ifndef FIMC_IS_EEPROM_REAR_DUAL_3L2_SR556_V001_H
#define FIMC_IS_EEPROM_REAR_DUAL_3L2_SR556_V001_H

/* EEPROM I2C Addr Section */
#define EEP_I2C_HEADER_VERSION_START_ADDR        0x40
#define EEP_I2C_HEADER_VERSION_START_ADDR_REAR2  0x50
#define EEP_I2C_HEADER_CAL_MAP_VER_START_ADDR    0x60

/***** HEADER Referenced Section ******/
/* REAR MAIN */
#define EEP_HEADER_OEM_START_ADDR                0x0
#define EEP_HEADER_OEM_END_ADDR                  0x4
#define EEP_HEADER_AWB_START_ADDR                0x8
#define EEP_HEADER_AWB_END_ADDR                  0xC
#define EEP_HEADER_AP_SHADING_START_ADDR         0x10
#define EEP_HEADER_AP_SHADING_END_ADDR           0x14
/* REAR2 SUB */
#define EEP_HEADER_OEM_START_ADDR_REAR2          0x20
#define EEP_HEADER_OEM_END_ADDR_REAR2            0x24
#define EEP_HEADER_AWB_START_ADDR_REAR2          0x28
#define EEP_HEADER_AWB_END_ADDR_REAR2            0x2C
#define EEP_HEADER_AP_SHADING_START_ADDR_REAR2   0x30
#define EEP_HEADER_AP_SHADING_END_ADDR_REAR2     0x34
/* DUAL DATA */
#define EEP_HEADER_DUAL_DATA_START_ADDR          0x38
#define EEP_HEADER_DUAL_DATA_END_ADDR            0x3C
/* REAR MAIN VERSION */
#define EEP_HEADER_VERSION_START_ADDR            0x40
/* REAR2 SUB VERSION */
#define EEP_HEADER_VERSION_START_ADDR_REAR2      0x50
/* CAL INFO */
#define EEP_HEADER_CAL_MAP_VER_START_ADDR        0x60
#define EEP_HEADER_CAL_DLL_VER_START_ADDR        0x64
#define EEP_HEADER_CAL_DLL_VER_START_ADDR_REAR2  0x68
#define EEP_HEADER_PROJECT_NAME_START_ADDR       0x6C
#define EEP_HEADER_MODULE_ID_ADDR                0xAE
#define EEP_HEADER_SENSOR_ID_ADDR                0xB8
#define EEP_HEADER_SENSOR_ID_ADDR_REAR2          0xC8
/* HEADER CHECKSUM */
#define EEP_CHECKSUM_HEADER_ADDR                 0xFC
#define HEADER_CRC32_LEN                         (232)

/***** OEM Referenced Section *****/
#define EEP_OEM_VER_START_ADDR                   0x1E0
#define EEP_CHECKSUM_OEM_ADDR                    0x1FC
#define OEM_CRC32_LEN                            (210)

#define EEPROM_AF_CAL_PAN_ADDR                   0x100
#define EEPROM_AF_CAL_MACRO_ADDR                 0x108

/***** AWB Referenced section *****/
#define EEP_AWB_VER_START_ADDR                   0x2E0
#define EEP_CHECKSUM_AWB_ADDR                    0x2FC
#define AWB_CRC32_LEN                            (32)

/***** AP Shading Referenced section *****/
#define EEP_AP_SHADING_VER_START_ADDR            0x16E0
#define EEP_CHECKSUM_AP_SHADING_ADDR             0x16FC
#define SHADING_CRC32_LEN                        (4992)

/***** REAR2 OEM Referenced Section *****/
#define EEP_OEM_VER_START_ADDR_REAR2             0x17E0
#define EEP_CHECKSUM_OEM_ADDR_REAR2              0x17FC
#define OEM_CRC32_LEN_REAR2                      (210)

#define EEPROM_AF_CAL2_PAN_ADDR_REAR2            0x1700
#define EEPROM_AF_CAL2_MACRO_ADDR_REAR2          0x1708

/***** REAR2 AWB Referenced Section *****/
#define EEP_AWB_VER_START_ADDR_REAR2             0x18E0
#define EEP_CHECKSUM_AWB_ADDR_REAR2              0x18FC
#define AWB_CRC32_LEN_REAR2                      (32)

/***** REAR2 AP Shading Referenced Section *****/
#define EEP_AP_SHADING_VER_START_ADDR_REAR2      0x2CE0
#define EEP_CHECKSUM_AP_SHADING_ADDR_REAR2       0x2CFC
#define SHADING_CRC32_LEN_REAR2                  (4992)

/***** DUAL DATRA Referenced Section *****/
#define EEP_DUAL_DATA_VER_START_ADDR             0x3FE0
#define EEP_CHECKSUM_DUAL_DATA_ADDR              0x3FFC
#define DUAL_DATA_CRC32_LEN                      (2304)

#define EEP_HEADER_MTF_DATA_ADDR                 0x3A80
#define EEP_HEADER_MTF_DATA2_ADDR                0x3AB6

/***** REAR2 Cal Dual Calibration Data2 *****/
#define EEP_DUAL_CAL_DATA2_ADDR                  0x3590
#define EEP_DUAL_CAL_DATA2_SIZE                  (512)

#define EEP_REAR2_DUAL_TILT_X                    0x35EC
#define EEP_REAR2_DUAL_TILT_Y                    0x35F0
#define EEP_REAR2_DUAL_TILT_Z                    0x35F4
#define EEP_REAR2_DUAL_TILT_SX                   0x364C
#define EEP_REAR2_DUAL_TILT_SY                   0x3650
#define EEP_REAR2_DUAL_TILT_RANGE                0x3770
#define EEP_REAR2_DUAL_TILT_MAX_ERR              0x3774
#define EEP_REAR2_DUAL_TILT_AVG_ERR              0x3778
#define EEP_REAR2_DUAL_TILT_DLL_VERSION	         0x376C

#define EEP_REAR2_DUAL_SHIFT_X                   0x364C
#define EEP_REAR2_DUAL_SHIFT_Y                   0x3650

/***** ETC SECTION *****/
#define FIMC_IS_MAX_CAL_SIZE                    (16 * 1024)

#endif /* FIMC_IS_EEPROM_REAR_DUAL_3L2_SR556_V001_H */
