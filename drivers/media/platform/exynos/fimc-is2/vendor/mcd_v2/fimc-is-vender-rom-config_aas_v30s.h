#ifndef FIMC_IS_VENDER_ROM_CONFIG_AAS_V30S_H
#define FIMC_IS_VENDER_ROM_CONFIG_AAS_V30S_H

/***** [ ROM VERSION HISTORY] *******************************************
 *
 *
 *
 *
 *
 *
 *
 *
 *
 ***********************************************************************/

#include "fimc-is-eeprom-rear-imx576-gc5035_v001.h"
#include "fimc-is-eeprom-front-hi1631_v001.h"
#include "fimc-is-eeprom-rear2-4ha_v002.h"


const struct fimc_is_vender_rom_addr *vender_rom_addr[SENSOR_POSITION_MAX] = {
	&rear_imx576_gc5035_cal_addr,		//[0] SENSOR_POSITION_REAR
	&front_hi1631_cal_addr,			//[1] SENSOR_POSITION_FRONT
	&rear2_4ha_cal_addr,			//[2] SENSOR_POSITION_REAR2
	NULL,					//[3] SENSOR_POSITION_FRONT2
	NULL,					//[4] SENSOR_POSITION_REAR3
	NULL,					//[5] SENSOR_POSITION_FRONT3
};

#endif/*FIMC_IS_VENDER_ROM_CONFIG_V30S_H*/
