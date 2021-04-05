#ifndef FIMC_IS_VENDER_ROM_CONFIG_AAS_V30_H
#define FIMC_IS_VENDER_ROM_CONFIG_AAS_V30_H

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

#include "fimc-is-eeprom-rear-2p6_PDAF_v001.h"
#include "fimc-is-eeprom-front-3p8sp_imx471_v001.h"
#include "fimc-is-eeprom-rear2-5e9_v001.h"

const struct fimc_is_vender_rom_addr *vender_rom_addr[SENSOR_POSITION_MAX] = {
	&rear_2p6_pdaf_cal_addr,		//[0] SENSOR_POSITION_REAR
	&front_3p8sp_imx471_cal_addr,	//[1] SENSOR_POSITION_FRONT
	&rear2_5e9_cal_addr,			//[2] SENSOR_POSITION_REAR2
	NULL,						//[3] SENSOR_POSITION_FRONT2
	NULL,						//[4] SENSOR_POSITION_REAR3
	NULL,						//[5] SENSOR_POSITION_FRONT3
};

#endif
