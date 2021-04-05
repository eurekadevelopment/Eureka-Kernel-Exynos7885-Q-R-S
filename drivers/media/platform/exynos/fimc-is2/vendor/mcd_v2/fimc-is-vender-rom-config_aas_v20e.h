#ifndef FIMC_IS_VENDER_ROM_CONFIG_AAS_V20E_H
#define FIMC_IS_VENDER_ROM_CONFIG_AAS_V20E_H

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

#include "fimc-is-eeprom-rear-3l6_v001.h"
#include "fimc-is-eeprom-front-4ha_v001.h"
#include "fimc-is-eeprom-rear2-sr556_v001.h"

const struct fimc_is_vender_rom_addr *vender_rom_addr[SENSOR_POSITION_MAX] = {
	&rear_3l6_cal_addr,			//[0] SENSOR_POSITION_REAR
	&front_4ha_cal_addr,		//[1] SENSOR_POSITION_FRONT
	&rear2_sr556_cal_addr,		//[2] SENSOR_POSITION_REAR2
	NULL,						//[3] SENSOR_POSITION_FRONT2
	NULL,						//[4] SENSOR_POSITION_REAR3
	NULL,						//[5] SENSOR_POSITION_FRONT3
};

#endif
