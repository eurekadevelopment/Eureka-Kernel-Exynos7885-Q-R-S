#ifndef FIMC_IS_EEPROM_REAR_3P8SP_V001_H
#define FIMC_IS_EEPROM_REAR_3P8SP_V001_H

//Universal EEPROM map data_V001_20181101_for_A30_M30_LassenO_3P8SP_Front.xlsx
//Universal EEPROM map data_V001_20181101_for_A30_M30_LassenO_Front.xlsx

#define FIMC_IS_FRONT_MAX_CAL_SIZE (8 * 1024)

#define EEP_XTALK_CAL_DATA_SIZE_FRONT (2 * 1024)

#define FRONT_HEADER_CHECKSUM_LEN (0x00D7 - 0x0000 + 0x1)
#define FRONT_OEM_CHECKSUM_LEN (0x01BF - 0x0190 + 0x1)
#define FRONT_AWB_CHECKSUM_LEN (0x021F - 0x0200 + 0x1)
#define FRONT_SHADING_CHECKSUM_LEN (0x168F - 0x0300 + 0x1)
#define FRONT_SENSOR_CHECKSUM_LEN (0x1F0F - 0x1700 + 0x1)

// The start address of checksum is not same with OEM CAL start address

const struct rom_extend_cal_addr front_3p8sp_extend_cal_addr[COUNT_EXTEND_CAL_DATA] = {
	{"oem_checksum_base_addr", 0x0190},
};

const struct fimc_is_vender_rom_addr front_3p8sp_cal_addr = {
	/* Set '-1' if not used */

	'A',				//char		camera_module_es_version;
	'1',				//u8			cal_map_es_version;
	(8 * 1024),		//int32_t		rom_max_cal_size;

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
	0x18,			//int32_t		rom_header_main_sensor_cal_start_addr;
	0x1C,			//int32_t		rom_header_main_sensor_cal_end_addr;
	-1,				//int32_t		rom_header_dual_cal_start_addr;
	-1,				//int32_t		rom_header_dual_cal_end_addr;
	-1,				//int32_t		rom_header_pdaf_cal_start_addr;
	-1,				//int32_t		rom_header_pdaf_cal_end_addr;

	-1,				//int32_t		rom_header_sub_oem_start_addr;
	-1,				//int32_t		rom_header_sub_oem_end_addr;
	-1,				//int32_t		rom_header_sub_awb_start_addr;
	-1,				//int32_t		rom_header_sub_awb_end_addr;
	-1,				//int32_t		rom_header_sub_shading_start_addr;
	-1,				//int32_t		rom_header_sub_shading_end_addr;

	0x48,			//int32_t		rom_header_main_mtf_data_addr;
	-1,				//int32_t		rom_header_sub_mtf_data_addr;

	0xFC,			//int32_t		rom_header_checksum_addr;
	FRONT_HEADER_CHECKSUM_LEN,		//int32_t		rom_header_checksum_len;

	0x0100,			//int32_t		rom_oem_af_inf_position_addr;
	0x0108,			//int32_t		rom_oem_af_macro_position_addr;
	0x01E0,			//int32_t		rom_oem_module_info_start_addr;
	0x01FC,			//int32_t		rom_oem_checksum_addr;
	FRONT_OEM_CHECKSUM_LEN,		//int32_t		rom_oem_checksum_len;

	0x02E0,			//int32_t		rom_awb_module_info_start_addr;
	0x02FC,			//int32_t		rom_awb_checksum_addr;
	FRONT_AWB_CHECKSUM_LEN,		//int32_t		rom_awb_checksum_len;

	0x16E0,			//int32_t		rom_shading_module_info_start_addr;
	0x16FC,			//int32_t		rom_shading_checksum_addr;
	FRONT_SHADING_CHECKSUM_LEN,		//int32_t		rom_shading_checksum_len;

	0x1FE0,			//int32_t		rom_sensor_cal_module_info_start_addr;
	0x1FFC,			//int32_t		rom_sensor_cal_checksum_addr;
	FRONT_SENSOR_CHECKSUM_LEN,		//int32_t		rom_sensor_cal_checksum_len;

	-1,				//int32_t		rom_dual_module_info_start_addr;
	-1,				//int32_t		rom_dual_checksum_addr;
	-1,				//int32_t		rom_dual_checksum_len;

	-1,				//int32_t		rom_pdaf_module_info_start_addr;
	-1,				//int32_t		rom_pdaf_checksum_addr;
	-1,				//int32_t		rom_pdaf_checksum_len;

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

	front_3p8sp_extend_cal_addr,			//void*			extended_cal_addr;
};


#endif /* FIMC_IS_EEPROM_REAR_3P8SP_V001_H */
