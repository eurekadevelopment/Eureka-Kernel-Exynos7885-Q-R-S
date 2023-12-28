/*
 * Copyright (c) 2023 Eureka Team.
 *      https://github.com/eurekadevelopment
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/printk.h>

#define dev_common(dev, fmt, ...)

#ifdef dev_info
#undef dev_info
#define dev_info dev_common
#endif

#ifdef dev_dbg
#undef dev_dbg
#define dev_dbg dev_common
#endif

#ifdef dev_warn
#undef dev_warn
#define dev_warn dev_common
#endif

#define pr_common(fmt, ...)

#ifdef pr_info
#undef pr_info
#define pr_info pr_common
#endif

#ifdef pr_dbg
#undef pr_dbg
#define pr_dbg pr_common
#endif

#ifdef pr_warn
#undef pr_warn
#define pr_warn pr_common
#endif
