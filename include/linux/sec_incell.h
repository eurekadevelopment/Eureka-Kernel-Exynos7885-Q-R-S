/* Copyright (c) 2016, TSamsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SEC_INCELL_H_
#define _SEC_INCELL_H_

struct incell_driver_data {
	void (*blank_unblank)(void *drv_data);
};

extern struct incell_driver_data incell_data;
#endif

