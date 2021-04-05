#ifndef FIMC_IS_EEPROM_REAR_PDAF_2P6_V001_H
#define FIMC_IS_EEPROM_REAR_PDAF_2P6_V001_H

//Universal EEPROM map data V001_20181206_for A30_Lassen_ZC535_10K_EEPROM_(2P6_16M_PDAF).xlsx

#define FIMC_IS_REAR_MAX_CAL_SIZE (10 * 1024)

#define REAR_HEADER_CHECKSUM_LEN (0x00DF - 0x0000 + 0x1)
#define REAR_OEM_CHECKSUM_LEN (0x01D1 - 0x0100 + 0x1)
#define REAR_AWB_CHECKSUM_LEN (0x021F - 0x0200 + 0x1)
#define REAR_SHADING_CHECKSUM_LEN (0x1C2F - 0x0240 + 0x1)
#define REAR_PDAF_CHECKSUM_LEN (0x21FF - 0x1C50 + 0x1)
#define REAR_DUAL_CHECKSUM_LEN (0x225F - 0x2220 + 0x1)

/********************************************************************************
 * 
 * EERPOM DATA ADDR : 16bit [ C1 C0 A13 A12 A11 A10 A9 A8 A7 A6 A5 A3 A2 A1 A0 ]
 * C[1:0] : EEPROM Command bits
 *			00 : EEPROM READ
 *			01 : EEPROM WRITE
 *			10 : EEPROM Page Erase
 *			11 : EEPROM All Erase
 *
 * A[13:6] : EEPROM Page Addr (0 ~ 159 pages)
 * A[ 5:0] : EEPROM Byte Addr (0 ~ 63 bytes)
 *
*********************************************************************************/
#define ZC535_EEPROM_ADDR_IN_PAGE(a) ((((a/64) << 6) & 0x1FE0) | ((a%64) & 0x1F))

const struct fimc_is_vender_rom_addr rear_2p6_pdaf_cal_addr = {
	/* Set '-1' if not used */

	'A',				//char		camera_module_es_version;
	'1',				//char		cal_map_es_version;
	FIMC_IS_REAR_MAX_CAL_SIZE,		//int32_t		rom_max_cal_size;

	0x00,			//int32_t		rom_header_cal_data_start_addr;
	0x28,			//int32_t		rom_header_main_module_info_start_addr;
	0x38,			//int32_t		rom_header_cal_map_ver_start_addr;
	0x40,			//int32_t		rom_header_project_name_start_addr;
	0xAE,			//int32_t		rom_header_module_id_addr;
	0xB8,			//int32_t		rom_header_main_sensor_id_addr;

	-1,				//int32_t		rom_header_sub_module_info_start_addr;
	-1,				//int32_t		rom_header_sub_sensor_id_addr;

	-1,				//int32_t		rom_header_main_header_start_addr;
	-1,				//int32_t		rom_header_main_header_end_addr;
	0x00,			//int32_t		rom_header_main_oem_start_addr;
	0x04,			//int32_t		rom_header_main_oem_end_addr;
	0x08,			//int32_t		rom_header_main_awb_start_addr;
	0x0C,			//int32_t		rom_header_main_awb_end_addr;
	0x10,			//int32_t		rom_header_main_shading_start_addr;
	0x14,			//int32_t		rom_header_main_shading_end_addr;
	-1,				//int32_t		rom_header_main_sensor_cal_start_addr;
	-1,				//int32_t		rom_header_main_sensor_cal_end_addr;
	0x20,			//int32_t		rom_header_dual_cal_start_addr;
	0x24,			//int32_t		rom_header_dual_cal_end_addr;
	0x18,			//int32_t		rom_header_pdaf_cal_start_addr;
	0x1C,			//int32_t		rom_header_pdaf_cal_end_addr;

	-1,				//int32_t		rom_header_sub_oem_start_addr;
	-1,				//int32_t		rom_header_sub_oem_end_addr;
	-1,				//int32_t		rom_header_sub_awb_start_addr;
	-1,				//int32_t		rom_header_sub_awb_end_addr;
	-1,				//int32_t		rom_header_sub_shading_start_addr;
	-1,				//int32_t		rom_header_sub_shading_end_addr;

	0x48,			//int32_t		rom_header_main_mtf_data_addr;
	-1,				//int32_t		rom_header_sub_mtf_data_addr;

	0xFC,			//int32_t		rom_header_checksum_addr;
	REAR_HEADER_CHECKSUM_LEN,		//int32_t		rom_header_checksum_len;

	0x0100,			//int32_t		rom_oem_af_inf_position_addr;
	0x0108,			//int32_t		rom_oem_af_macro_position_addr;
	0x01D2,			//int32_t		rom_oem_module_info_start_addr;
	0x01FC,			//int32_t		rom_oem_checksum_addr;
	REAR_OEM_CHECKSUM_LEN,			//int32_t		rom_oem_checksum_len;

	0x0220,			//int32_t		rom_awb_module_info_start_addr;
	0x023C,			//int32_t		rom_awb_checksum_addr;
	REAR_AWB_CHECKSUM_LEN,			//int32_t		rom_awb_checksum_len;

	0x1C30,			//int32_t		rom_shading_module_info_start_addr;
	0x1C4C,			//int32_t		rom_shading_checksum_addr;
	REAR_SHADING_CHECKSUM_LEN,		//int32_t		rom_shading_checksum_len;

	-1,				//int32_t		rom_sensor_cal_module_info_start_addr;
	-1,				//int32_t		rom_sensor_cal_checksum_addr;
	-1,				//int32_t		rom_sensor_cal_checksum_len;

	0x2260,			//int32_t		rom_dual_module_info_start_addr;
	0x227C,			//int32_t		rom_dual_checksum_addr;
	-1,				//int32_t		rom_dual_checksum_len;

	0x2200,			//int32_t		rom_pdaf_module_info_start_addr;
	0x221C,			//int32_t		rom_pdaf_checksum_addr;
	REAR_PDAF_CHECKSUM_LEN,		//int32_t		rom_pdaf_checksum_len;

	-1,				//int32_t		rom_sub_oem_af_inf_position_addr;
	-1,				//int32_t		rom_sub_oem_af_macro_position_addr;
	-1,				//int32_t		rom_sub_oem_module_info_start_addr;
	-1,				//int32_t		rom_sub_oem_checksum_addr;
	-1,				//int32_t		rom_sub_oem_checksum_len;

	-1,				//int32_t		rom_sub_awb_module_info_start_addr;
	-1,				//int32_t		rom_sub_awb_checksum_addr;
	-1,				//int32_t		rom_sub_awb_checksum_len;

	-1,				//int32_t		rom_sub_shading_module_info_start_addr;
	-1,				//int32_t		rom_sub_shading_checksum_addr;
	-1,				//int32_t		rom_sub_shading_checksum_len;

	-1,				//int32_t		rom_dual_cal_data2_start_addr;
	-1,				//int32_t		rom_dual_cal_data2_size;
	-1,				//int32_t		rom_dual_tilt_x_addr;
	-1,				//int32_t		rom_dual_tilt_y_addr;
	-1,				//int32_t		rom_dual_tilt_z_addr;
	-1,				//int32_t		rom_dual_tilt_sx_addr;
	-1,				//int32_t		rom_dual_tilt_sy_addr;
	-1,				//int32_t		rom_dual_tilt_range_addr;
	-1,				//int32_t		rom_dual_tilt_max_err_addr;
	-1,				//int32_t		rom_dual_tilt_avg_err_addr;
	-1,				//int32_t		rom_dual_tilt_dll_version_addr;
	-1,				//int32_t		rom_dual_shift_x_addr;
	-1,				//int32_t		rom_dual_shift_y_addr;

	NULL,			//void*			extended_cal_addr;
};


#endif /* FIMC_IS_EEPROM_REAR_PDAF_2P6_V001_H */

