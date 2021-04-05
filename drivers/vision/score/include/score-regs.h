/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SCORE_REGS_H
#define _SCORE_REGS_H

/* SCore SFR */
#define SCORE_ENABLE			(0x0000)
/* HOST Command Check, 0x0 : No Command, 0x1 : New Command  */
#define SCORE_HOST2SCORE		(0x0004)
#define SCORE_SCORE2HOST		(0x0008)

#define SCORE_CONFIGURE			(0x003C)
/* score sw reset, 0x1 : Core Reset, 0x2 : DMA Reset, 0x4 : Cache Reset */
#define SCORE_SW_RESET			(0x0040)
#define SCORE_IDLE			(0x0054)
#define SCORE_VERIFY_RESULT		(0x0300)
/* CODE Firmware ADDRESS */
#define SCORE_CODE_START_ADDR		(0x1000)
/* DATA FW ADDRESS */
#define SCORE_DATA_START_ADDR		(0x2000)
/* CODE MODE */
#define SCORE_CODE_MODE_ADDR		(0x1004)
/* DATA MODE */
#define SCORE_DATA_MODE_ADDR		(0x2004)

#define SCORE_USERDEFINDED0		(0x7000)
#define SCORE_USERDEFINDED1		(0x7004)
#define SCORE_USERDEFINDED2		(0x7008)
#define SCORE_USERDEFINDED3		(0x700C)
#define SCORE_USERDEFINDED4		(0x7010)
#define SCORE_USERDEFINDED5		(0x7014)
#define SCORE_USERDEFINDED6		(0x7018)
#define SCORE_USERDEFINDED7		(0x701C)

#define SCORE_PARAM0			(0x7000)
#define SCORE_PARAM1			(0x7004)
#define SCORE_PARAM2			(0x7008)
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
#define SCORE_PARAM46			(0x70B8)
#define SCORE_PARAM47			(0x70BC)

#define SCORE_PARAM48			(0x70C0)
#define SCORE_PARAM49			(0x70C4)
#define SCORE_PARAM50			(0x70C8)
#define SCORE_PARAM51			(0x70CC)

#define SCORE_PARAM52			(0x70D0)
#define SCORE_PARAM53			(0x70D4)
#define SCORE_PARAM54			(0x70D8)
#define SCORE_PARAM55			(0x70DC)

#define SCORE_PARAM56			(0x70E0)
#define SCORE_PARAM57			(0x70E4)
#define SCORE_PARAM58			(0x70E8)
#define SCORE_PARAM59			(0x70EC)

#define SCORE_PARAM60			(0x70F0)
#define SCORE_PARAM61			(0x70F4)
#define SCORE_PARAM62			(0x70F8)
#define SCORE_PARAM63			(0x70FC)

#define SCORE_PRINTF_BUF_START		(0x0020)
#define SCORE_PRINTF_BUF_SIZE		(0x0024)
#define SCORE_AXIMAXLEN			(0x002C)
#define SCORE_AXIM0AWUSER		(0x0030)
#define SCORE_AXIM0ARUSER		(0x0034)
#define SCORE_AXIM1AWUSER		(0x0038)
#define SCORE_AXIM1ARUSER		(0x003C)

#define SCORE_DSP_INT			(0x0050)

/* @brief SFR variable for indicating which interrupts come from SCore to HOST */
#define SCORE_INT_CODE			(0x0064)

/* @brief Information for SCORE_INT_CODE */
#define SCORE_INT_ABNORMAL_MASK		(0x0000FFFF)
#define SCORE_INT_HW_EXCEPTION_MASK	(0x0007 << 0)
#define SCORE_INT_BUS_ERROR		(0x0001 << 0)
#define SCORE_INT_DMA_CONFLICT		(0x0002 << 0)
#define SCORE_INT_CACHE_UNALIGNED	(0x0004 << 0)

#define SCORE_INT_DUMP_PROFILER_MASK	(0x000F << 4)
#define SCORE_INT_CORE_DUMP_HAPPEN	(0x0001 << 4)
#define SCORE_INT_CORE_DUMP_DONE	(0x0002 << 4)
#define SCORE_INT_PROFILER_START	(0x0004 << 4)
#define SCORE_INT_PROFILER_END		(0x0008 << 4)

#define SCORE_INT_SW_ASSERT		(0x0001 << 9)

#define SCORE_INT_NORMAL_MASK		(0xFFFF0000)
#define SCORE_INT_OUTPUT_QUEUE		(0x0001 << 16)
/* register size = 32bit & 4bytes */
#define SCORE_REG_SIZE			(4)

#endif
