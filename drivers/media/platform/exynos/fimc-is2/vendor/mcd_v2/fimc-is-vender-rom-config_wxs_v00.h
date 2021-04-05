#ifndef FIMC_IS_VENDER_ROM_CONFIG_WXS_V00_H
#define FIMC_IS_VENDER_ROM_CONFIG_WXS_V00_H

/***** [ ROM VERSION HISTORY] *******************************************
 *
 * < HW_REV 00 >
 *  rear eeprom version v001 : fimc-is-eeprom-rear-imx576_v001.h
 *  CAL_MAP_ES_VERSION_REAR  : 1
 *
 * < HW_REV 01 >
 *  rear eeprom version v002 : fimc-is-eeprom-rear-imx576_v002.h
 *  CAL_MAP_ES_VERSION_REAR  : 2
 *
 ***********************************************************************/

#include "fimc-is-eeprom-rear-4ha_v001.h"
#include "fimc-is-otprom-front-5e9_v001.h"

const struct fimc_is_vender_rom_addr *vender_rom_addr[SENSOR_POSITION_MAX] = {
	&rear_4ha_cal_addr,		//[0] SENSOR_POSITION_REAR
	&front_5e9_otp_cal_addr,	//[1] SENSOR_POSITION_FRONT
	NULL,					//[2] SENSOR_POSITION_REAR2
	NULL,					//[3] SENSOR_POSITION_FRONT2
	NULL,					//[4] SENSOR_POSITION_REAR3
	NULL,					//[5] SENSOR_POSITION_FRONT3
};

#endif
