#ifndef FIMC_IS_VENDER_ROM_CONFIG_ATS_V03_H
#define FIMC_IS_VENDER_ROM_CONFIG_ATS_V03_H

/***** [ ROM VERSION HISTORY] *******************************************
 *
 * < HW_REV 00 >
 *  rear eeprom version v001 : fimc-is-eeprom-rear-4ha_v001.h
 *
 * < HW_REV 00 >
 *  front otprom version v001 : fimc-is-otprom-front-5e9_v001.h
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

#endif //FIMC_IS_VENDER_ROM_CONFIG_GTA3XL_H

