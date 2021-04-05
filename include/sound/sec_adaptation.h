/*
 * ALSA SoC - Samsung Adaptation driver
 *
 * Copyright (c) 2016 Samsung Electronics Co. Ltd.
  *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SEC_ADAPTATION_H
#define __SEC_ADAPTATION_H

#include <sound/maxim_dsm.h>
#include <sound/smart_amp.h>
#include <sound/samsung/abox.h>

#if 0
int maxim_dsm_write(uint32_t *data, int offset, int size);
int maxim_dsm_read(int offset, int size, void *dsm_data);
#else
static inline int maxim_dsm_write(uint32_t *data, int offset, int size)
{
	return -ENOSYS;
}

static inline int maxim_dsm_read(int offset, int size, void *dsm_data)
{
	return -ENOSYS;
}
#endif

//--------------------------------------------------
#ifdef SMART_AMP
int ti_smartpa_write(void *data, int offset, int size);
int ti_smartpa_read(void *data, int offset, int size);
#else
static inline int ti_smartpa_write(void *data, int offset, int size)
{
	return -ENOSYS;
}

static inline int ti_smartpa_read(void *data, int offset, int size)
{
	return -ENOSYS;
}
#endif
static inline int32_t dsm_read_write(void *data)
{
	return -ENOSYS;
}

#endif /* __SEC_ADAPTATION_H */

