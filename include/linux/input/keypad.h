/*
 * Copyright (c) 2018 The MoKee Open Source Project
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

#ifndef __LINUX_INPUT_KEYPAD_H
#define __LINUX_INPUT_KEYPAD_H

extern int keypad_register(const char *name, void *data,
	int (*read) (u32 *code, void *data),
	int (*write) (u32 code, void *data));

#endif /* __LINUX_INPUT_KEYPAD_H */
