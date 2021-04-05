/****************************************************************************
 *
 * Copyright (c) 2014 - 2016 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

/*
 * Copyright (C) 2009 Cambridge Silicon Radio Ltd.
 *
 * Refer to LICENSE.txt included with this source code for details on
 * the license terms.
 */
#ifndef __OSKA_LINUX_TRACE_H
#define __OSKA_LINUX_TRACE_H

#include <linux/kernel.h>

#ifndef OS_TRACE_PREFIX
#  define OS_TRACE_PREFIX ""
#endif

#ifndef pr_warn
#define pr_warn pr_warning
#endif

#define OSKA_DEBUG /* TODO: not getting defined by buildsys */

#define os_trace_err(format, ...)  pr_err(OS_TRACE_PREFIX "cmerr: " format "\n", ## __VA_ARGS__)
#define os_trace_warn(format, ...) pr_warn(OS_TRACE_PREFIX format "\n", ##  __VA_ARGS__)
#ifdef OSKA_DEBUG
#ifdef ANDROID_BUILD
#undef os_trace_warn
#define os_trace_warn(format, ...) pr_err(OS_TRACE_PREFIX format "\n", ##  __VA_ARGS__)
#define os_trace_info(format, ...) pr_err(OS_TRACE_PREFIX format "\n", ## __VA_ARGS__)
#define os_trace_dbg(format, ...)  pr_err(OS_TRACE_PREFIX format "\n", ## __VA_ARGS__)
#else
#define os_trace_info(format, ...) pr_info(OS_TRACE_PREFIX format "\n", ## __VA_ARGS__)
#define os_trace_dbg(format, ...)  pr_info(OS_TRACE_PREFIX format "\n", ## __VA_ARGS__)
#endif
#else
#define os_trace_info(format, ...)
#define os_trace_dbg(format, ...)
#endif
#endif /* #ifndef __OSKA_LINUX_TRACE_H */
