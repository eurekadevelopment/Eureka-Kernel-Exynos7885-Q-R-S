/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SCORE_BINARY_H_
#define SCORE_BINARY_H_

#include <linux/device.h>
#include <linux/firmware.h>

#define SCORE_FW_PATH1					"/data/"
#define SCORE_FW_PATH2					"/system/vendor/firmware/"
#define SCORE_FW_TEXT_NAME				"PMw"
#define SCORE_FW_DATA_NAME				"DMb"
#define SCORE_FW_EXT_NAME				".bin"
#define SCORE_FW_NAME_LEN				(100)
#define SCORE_VERSION_SIZE				(42)

/* HACK */
/* #define MAX_FW_COUNT					(3) */
#define MAX_FW_COUNT					(2)

#define SCORE_FW_CACHE_TEXT_SIZE			(0x00080000) /* 512KB */
#define SCORE_FW_CACHE_DATA_SIZE			(0x00180000) /* 1.5MB */
#define SCORE_FW_CACHE_STACK_SIZE			(0x00008000)

#define SCORE_FW_SRAM_TEXT_SIZE				(0x000020A0)
#define SCORE_FW_SRAM_DATA_SIZE				(0x00005020)
#define SCORE_FW_SRAM_STACK_SIZE			(0x00004000)
#define SCORE_FW_SRAM_INSTACK_SIZE			(0x00002000)
#define SCORE_FW_LAYOUT_2MB				(2 * 1024 * 1024)

enum score_fw_type {
	FW_TYPE_CACHE,
	FW_TYPE_SRAM
};

/* index, type, text size, data size, stack size, in stack size */
enum score_fw_table_pos {
	FW_TABLE_INDEX = 0,
	FW_TABLE_TYPE = 1,
	FW_TABLE_TEXT_SIZE = 2,
	FW_TABLE_DATA_SIZE = 3,
	FW_TABLE_STACK_SIZE = 4,
	FW_TABLE_IN_STACK_SIZE = 5,
	FW_TABLE_MAX
};

struct score_binary {
	struct device		*dev;

	/* first try to load */
	char			fpath1[SCORE_FW_NAME_LEN];
	/* second try to load */
	char			fpath2[SCORE_FW_NAME_LEN];
	unsigned int		loaded_fw_index;

	void			*target;
	size_t			target_size;
	size_t			image_size;
};

int score_binary_init(struct score_binary *binary,
		struct device *dev,
		void *target,
		size_t target_size);
int score_binary_load(struct score_binary *binary);
int score_binary_set_text_path(struct score_binary *binary,
		unsigned int index, struct device *dev);
int score_binary_set_data_path(struct score_binary *binary,
		unsigned int index, struct device *dev);
void score_binary_version(struct score_binary *binary);

#endif
