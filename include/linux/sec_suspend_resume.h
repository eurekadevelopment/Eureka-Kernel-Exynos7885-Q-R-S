/* sec_ext.h
 *
 * Copyright (C) 2014 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef SEC_RESUME_SUSPEND_H
#define SEC_RESUME_SUSPEND_H
#ifdef CONFIG_SEC_RESUME_SUSPEND_DEBUG
typedef int (*pm_callback_t)(struct device *);
extern int debug_enable;
#define SEC_RESUME_DEBUG_MIN_TIME		10000
#define SEC_SUSPEND_DEBUG_MIN_TIME		1000
extern void sec_debug_add(pm_callback_t callback, unsigned long long t, int type);
extern void sec_sorted_list(int type);
#else
#define sec_debug_add(a,b)		do { } while(0)	
#define sec_sorted_list(a)      		do { } while(0)
#endif /* CONFIG_SEC_RESUME_SUSPEND_DEBUG */

#endif /* SEC_RESUME_SUSPEND_H */
