/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SCORE_REGS_USER_DEF_H
#define _SCORE_REGS_USER_DEF_H

#include "score-regs.h"

/*
 * General Purpose Define
 * INPUT QUEUE / OUTPUT QUEUE
 */

#define SCORE_INPUT_QUEUE_HEAD		SCORE_PARAM0
#define SCORE_INPUT_QUEUE_TAIL		SCORE_PARAM1
/* 2 ~ 45 */
#define SCORE_INPUT_QUEUE_START		SCORE_PARAM2
/*
#define SCORE_PARAM3			(0x700C)

#define SCORE_PARAM4			(0x7010)
#define SCORE_PARAM5			(0x7014)
#define SCORE_PARAM6			(0x7018)
#define SCORE_PARAM7			(0x701C)

#define SCORE_PARAM8			(0x7020)
#define SCORE_PARAM9			(0x7024)
#define SCORE_PARAM10			(0x7028)
#define SCORE_PARAM11			(0x702C)

#define SCORE_PARAM12			(0x7030)
#define SCORE_PARAM13			(0x7034)
#define SCORE_PARAM14			(0x7038)
#define SCORE_PARAM15			(0x703C)

#define SCORE_PARAM16			(0x7040)
#define SCORE_PARAM17			(0x7044)
#define SCORE_PARAM18			(0x7048)
#define SCORE_PARAM19			(0x704C)

#define SCORE_PARAM20			(0x7050)
#define SCORE_PARAM21			(0x7054)
#define SCORE_PARAM22			(0x7058)
#define SCORE_PARAM23			(0x705C)

#define SCORE_PARAM24			(0x7060)
#define SCORE_PARAM25			(0x7064)
#define SCORE_PARAM26			(0x7068)
#define SCORE_PARAM27			(0x706C)

#define SCORE_PARAM28			(0x7070)
#define SCORE_PARAM29			(0x7074)
#define SCORE_PARAM30			(0x7078)
#define SCORE_PARAM31			(0x707C)

#define SCORE_PARAM32			(0x7080)
#define SCORE_PARAM33			(0x7084)
#define SCORE_PARAM34			(0x7088)
#define SCORE_PARAM35			(0x708C)

#define SCORE_PARAM36			(0x7090)
#define SCORE_PARAM37			(0x7094)
#define SCORE_PARAM38			(0x7098)
#define SCORE_PARAM39			(0x709C)

#define SCORE_PARAM40			(0x70A0)
#define SCORE_PARAM41			(0x70A4)
#define SCORE_PARAM42			(0x70A8)
#define SCORE_PARAM43			(0x70AC)

#define SCORE_PARAM44			(0x70B0)
#define SCORE_PARAM45			(0x70B4)
*/
#define SCORE_OUTPUT_QUEUE_HEAD		SCORE_PARAM46
#define SCORE_OUTPUT_QUEUE_TAIL		SCORE_PARAM47
/* 48 ~ 51 */
#define SCORE_OUTPUT_QUEUE_START	SCORE_PARAM48
/*
#define SCORE_PARAM49			(0x70C4)
#define SCORE_PARAM50			(0x70C8)
#define SCORE_PARAM51			(0x70CC)
*/
/* reserved: 52 ~ 57 */
/*
#define SCORE_PARAM52			(0x70D0)
#define SCORE_PARAM53			(0x70D4)
#define SCORE_PARAM54			(0x70D8)
#define SCORE_PARAM55			(0x70DC)

#define SCORE_PARAM56			(0x70E0)
#define SCORE_PARAM57			(0x70E4)
*/
/* reserved by algorithm: 58 ~ 63 */
/*
#define SCORE_PARAM58			(0x70E8)
#define SCORE_PARAM59			(0x70EC)

#define SCORE_PARAM60			(0x70F0)
#define SCORE_PARAM61			(0x70F4)
#define SCORE_PARAM62			(0x70F8)
#define SCORE_PARAM63			(0x70FC)
*/

/*
 * Special Purpose Define
 */
#define SCORE_PRINT0			(0x0020)
#define SCORE_PRINT1			(0x0024)
#define SCORE_SCORE2CPU_INT_STAT	(0x0064)
#define SCORE_COREDUMP0			(0x305C)
#define SCORE_COREDUMP1			(0x405C)
#define SCORE_SPARESFR0			(0x505C)
#define SCORE_SPARESFR1			(0x605C)

#endif /* SCORE_REGS_USER_DEF_H */
