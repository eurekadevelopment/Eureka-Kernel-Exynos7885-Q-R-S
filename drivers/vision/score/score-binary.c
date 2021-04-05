/*
 * Samsung Exynos SoC series SCORE driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/syscalls.h>
#include <linux/vmalloc.h>

#include "score-config.h"
#include "score-binary.h"
#include "score-debug.h"

static noinline_for_stack long __get_file_size(struct file *file)
{
	struct kstat st;

	if (vfs_getattr(&file->f_path, &st))
		return -1;
	if (!S_ISREG(st.mode))
		return -1;
	if (st.size != (long)st.size)
		return -1;

	return st.size;
}

int score_binary_init(struct score_binary *binary,
		struct device *dev,
		void *target,
		size_t target_size)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!binary);
	BUG_ON(!dev);
	BUG_ON(!target);

	binary->dev = dev;
	binary->target = target;
	binary->target_size = target_size;
	binary->image_size = 0;

	return ret;
}

int score_binary_set_text_path(struct score_binary *binary,
		unsigned int index, struct device *dev)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!binary);
	BUG_ON(!dev);

	snprintf(binary->fpath1, sizeof(binary->fpath1),
			"%s%s%d%s", SCORE_FW_PATH1,
			SCORE_FW_TEXT_NAME, index, SCORE_FW_EXT_NAME);
	snprintf(binary->fpath2, sizeof(binary->fpath2),
			"%s%d%s",
			SCORE_FW_TEXT_NAME, index, SCORE_FW_EXT_NAME);
	score_event_msg("(%s) \n", binary->fpath1);
	score_event_msg("(%s) \n", binary->fpath2);

	return ret;
}

int score_binary_set_data_path(struct score_binary *binary,
		unsigned int index, struct device *dev)
{
	int ret = 0;

	SCORE_TP();
	BUG_ON(!binary);
	BUG_ON(!dev);

	snprintf(binary->fpath1, sizeof(binary->fpath1),
			"%s%s%d%s", SCORE_FW_PATH1,
			SCORE_FW_DATA_NAME, index, SCORE_FW_EXT_NAME);
	snprintf(binary->fpath2, sizeof(binary->fpath2),
			"%s%d%s",
			SCORE_FW_DATA_NAME, index, SCORE_FW_EXT_NAME);
	score_event_msg("(%s) \n", binary->fpath1);
	score_event_msg("(%s) \n", binary->fpath2);

	return ret;
}

int score_binary_load(struct score_binary *binary)
{
	int ret = 0;
	const struct firmware *fw_blob;
	u8 *buf;
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;

	SCORE_TP();
	BUG_ON(!binary);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(binary->fpath1, O_RDONLY, 0);

	if (IS_ERR_OR_NULL(fp)) {
		set_fs(old_fs);
		goto request_fw;
	}

	fsize = __get_file_size(fp);
	if (fsize <= 0) {
		score_err("__get_file_size is fail(%ld)\n", fsize);
		ret = -EBADF;
		goto p_err;
	}

	buf = vmalloc(fsize);
	if (!buf) {
		score_err("vmalloc is fail\n");
		ret = -ENOMEM;
		goto p_err;
	}

	nread = kernel_read(fp, 0, buf, fsize);
	if (nread != fsize) {
		score_err("vfs_read is fail(%ld != %ld)\n", nread, fsize);
		ret = -EIO;
		goto p_err;
	}

	if (fsize > binary->target_size) {
		score_err("image size is over(%ld > %ld)\n", fsize, binary->target_size);
		ret = -EIO;
		goto p_err;
	}

	binary->image_size = fsize;
	memcpy(binary->target, (void *)buf, fsize);
	score_event_msg("FW(%s, 0x%lx)was loaded\n", binary->fpath1, fsize);

p_err:
	vfree(buf);
	filp_close(fp, current->files);
	set_fs(old_fs);

	return ret;

request_fw:
	ret = request_firmware(&fw_blob, binary->fpath2, binary->dev);
	if (ret) {
		score_err("request_firmware(%s) is fail(%d)", binary->fpath2, ret);
		ret = -EINVAL;
		goto request_err;
	}

	if (!fw_blob) {
		score_err("fw_blob is NULL\n");
		ret = -EINVAL;
		goto request_err;
	}

	if (!fw_blob->data) {
		score_err("fw_blob->data is NULL\n");
		ret = -EINVAL;
		goto request_err;
	}

	if (fw_blob->size > binary->target_size) {
		score_err("image size is over(%ld > %ld)\n", binary->image_size, binary->target_size);
		ret = -EIO;
		goto p_err;
	}

	binary->image_size = fw_blob->size;
	memcpy(binary->target, fw_blob->data, fw_blob->size);
	score_event_msg("FW(%s, 0x%lx) were applied successfully.\n", binary->fpath2, fw_blob->size);

request_err:
	release_firmware(fw_blob);
	return ret;
}

void score_binary_version(struct score_binary *binary)
{
	char version_str[SCORE_VERSION_SIZE + 1];
	const char *bin;
	size_t size;

	SCORE_TP();
	BUG_ON(!binary);

	bin = binary->target;
	size = binary->image_size;

	memcpy(version_str, &bin[size - SCORE_VERSION_SIZE], SCORE_VERSION_SIZE);
	version_str[SCORE_VERSION_SIZE] = '\0';

	score_event_msg("FW version : %s\n", version_str);
}
