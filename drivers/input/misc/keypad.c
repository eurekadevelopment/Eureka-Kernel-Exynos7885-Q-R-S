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

#include <linux/module.h>

#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/input/keypad.h>

static struct proc_dir_entry *kdir_g = NULL;

struct keypad_button_data {
	void *data;
	int (*read) (u32 *code, void *data);
	int (*write) (u32 code, void *data);
};

static int keypad_open(struct inode *inode, struct file *file)
{
	struct keypad_button_data *bdata = PDE_DATA(inode);
	file->private_data = bdata;
	return nonseekable_open(inode, file);
}

static int keypad_close(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t keypad_read(struct file *file,
			char __user *buf, size_t count, loff_t *lo)
{
	struct keypad_button_data *bdata = file->private_data;
	int rc;
	u32 val;
	char page[10];

	if (!bdata) {
		return -ENODEV;
	}

	rc = bdata->read(&val, bdata->data);
	if (rc) {
		pr_err("%s: Failed to read key code: %d", __func__, rc);
		return -EINVAL;
	}

	sprintf(page, "%d\n", val);

	return simple_read_from_buffer(buf, count, lo, page, strlen(page));
}

static ssize_t keypad_write(struct file *file,
			const char __user *buf, size_t count, loff_t *lo)
{
	struct keypad_button_data *bdata = file->private_data;
	int rc;
	u32 val;
	char page[10] = {0};

	if (!bdata) {
		return -ENODEV;
	}

	rc = copy_from_user(page, buf, count);
	if (rc) {
		pr_err("%s: Failed reading input to write key code: %d", __func__, rc);
		return -EINVAL;
	}

	rc = kstrtoint(page, 10, &val);
	if (rc) {
		pr_err("%s: Failed parsing input to write key code: %d", __func__, rc);
		return -EINVAL;
	}

	rc = bdata->write(val, bdata->data);
	if (rc) {
		pr_err("%s: Failed to write key code: %d", __func__, rc);
		return -EINVAL;
	}

	return count;
}

static const struct file_operations keypad_fops = {
	.owner = THIS_MODULE,
	.llseek  = no_llseek,
	.open    = keypad_open,
	.release = keypad_close,
	.read    = keypad_read,
	.write   = keypad_write,
};

int keypad_register(const char *name, void *data,
	int (*read) (u32 *code, void *data),
	int (*write) (u32 code, void *data))
{
	struct keypad_button_data *bdata;

	bdata = kzalloc(sizeof(struct keypad_button_data), GFP_KERNEL);
	if (!bdata) {
		pr_err("%s: Failed to allocate state\n", __func__);
		return -ENOMEM;
	}

	bdata->data = data;
	bdata->read = read;
	bdata->write = write;

	if (kdir_g == NULL) {
		kdir_g = proc_mkdir("keypad", NULL);
	}

	if (proc_create_data(name, S_IRUSR | S_IWUSR, kdir_g, &keypad_fops, bdata) == NULL) {
		pr_err("%s: Failed to create node\n", __func__);
		return -ENODEV;
	}

	return 0;
}

static int __init keypad_dev_init(void)
{
	return 0;
}

module_init(keypad_dev_init);

static void __exit keypad_dev_exit(void)
{
}

module_exit(keypad_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("XiNGRZ <chenxingyu92@gmail.com>");
MODULE_DESCRIPTION("Generic userspace key code remapper");
