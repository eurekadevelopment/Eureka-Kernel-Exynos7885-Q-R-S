#ifndef FIMC_IS_OTPROM_FRONT_5E3_V001_H
#define FIMC_IS_OTPROM_FRONT_5E3_V001_H

/* Header Offset Addr Section */
#define OTP_HEADER_DIRECT_ADDR_FRONT

#define HEADER_START_ADDR_FRONT                      (0xA15 - OTP_START_ADDR)
#define OTP_HEADER_CAL_MAP_VER_START_ADDR_FRONT      (0xA22 - OTP_START_ADDR)
#define OTP_HEADER_VERSION_START_ADDR_FRONT          (0xA15 - OTP_START_ADDR)

/* AWB referenced section */
/*#define OTP_AWB_VER_START_ADDR_FRONT                 0x200*/

/* Checksum referenced section */
#define OTP_CHECKSUM_HEADER_ADDR_FRONT               (0xA3E - OTP_START_ADDR)

/* etc section */
#define FIMC_IS_MAX_CAL_SIZE_FRONT                   (8 * 1024)
#define HEADER_CRC32_LEN_FRONT                       (41)
#define OTP_USED_CAL_SIZE                            (HEADER_CRC32_LEN_FRONT + 4)

#define OTP_BANK
#ifdef OTP_BANK
#define OTP_BANK_ADDR                                0xA12
#define OTP_START_ADDR                               0xA15
static const u32 OTP_first_page_select_reg[] = {
	0x0A00, 0x04, 0x1,
	0x0A02, 0x02, 0x1,
	0x0A00, 0x01, 0x1,
};

static const u32 OTP_first_page_select_reg_size =
	sizeof(OTP_first_page_select_reg) / sizeof(OTP_first_page_select_reg[0]);

static const u32 OTP_second_page_select_reg[] = {
	0x0A00, 0x04, 0x1,
	0x0A02, 0x03, 0x1,
	0x0A00, 0x01, 0x1,
};

static const u32 OTP_second_page_select_reg_size =
	sizeof(OTP_second_page_select_reg) / sizeof(OTP_second_page_select_reg[0]);
#endif

#define OTP_MODE_CHANGE

#ifdef OTP_MODE_CHANGE
static const u32 sensor_mode_change_to_OTP_reg[] = {
	0x0A00, 0x04, 0x1,
	0x0A02, 0x02, 0x1,
	0x0A00, 0x01, 0x1,
};

static const u32 sensor_mode_change_to_OTP_reg_size =
	sizeof(sensor_mode_change_to_OTP_reg) / sizeof(sensor_mode_change_to_OTP_reg[0]);

static const u32 sensor_mode_change_from_OTP_reg[] = {
	0x0A00, 0x04, 0x1,
	0x0A00, 0x00, 0x1,
};

static const u32 sensor_mode_change_from_OTP_reg_size =
	sizeof(sensor_mode_change_from_OTP_reg) / sizeof(sensor_mode_change_from_OTP_reg[0]);
#endif

#endif /* FIMC_IS_OTPROM_FRONT_5E3_V001_H */
