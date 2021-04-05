/*  staging/samsung/sec_resume_suspend_debug.c
*
* Copyright (C) 2019 Samsung Electronics
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
* Author: Nguyen Duy Anh (anh.nd2@samsung.com)
*/

#include <linux/init.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/kobject.h> 
#include <linux/seq_file.h>
#include <linux/sysfs.h> 
#include <linux/sec_suspend_resume.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/kernel.h>

struct sec_debug_data {
	struct list_head list;
	pm_callback_t callback;
	unsigned long long duration;
};
static struct kobject *enable_resume_suspend;
static int count_resume = 0;
static int count_suspend = 0;
static LIST_HEAD(resume_list);
static LIST_HEAD(resume_sorted_list);
static LIST_HEAD(suspend_list);
static LIST_HEAD(suspend_sorted_list);
// type = 0 is suspend
// type = 1 is resume
void sec_sorted_list(int type){
	struct sec_debug_data *data, *data2, *tmp;
	struct list_head *head, *sorted_head;
	bool is_added = false;
	
	if(type == 0){
		head = &suspend_list;
		sorted_head = &suspend_sorted_list;
	}
	else{
		head = &resume_list;
		sorted_head = &resume_sorted_list;
	}
	
	if (!list_empty(head)) {
		if(!list_empty(sorted_head)){
			list_for_each_entry_safe(data, tmp, sorted_head, list){
				list_del(&data->list);
				kfree(data);
			}
			INIT_LIST_HEAD(sorted_head);
		}
		
		list_for_each_entry_safe(data, tmp, head, list) {
			is_added = false;
			list_for_each_entry(data2, sorted_head, list) {
				if ((data2->duration < data->duration) && !is_added) {
					list_add(&data->list, data2->list.prev);
					is_added = true;
					break;
				}
			}
			if (!is_added)
				list_add_tail(&data->list, sorted_head);
		}
		
		if(type == 0)
			count_suspend++;
		else	
			count_resume++;
		INIT_LIST_HEAD(head);
	}
}

static int sec_resume_debug_seq_show(struct seq_file *f, void *v)
{
	struct sec_debug_data *data;
	if(!list_empty(&resume_sorted_list)){
		seq_printf(f, "Resume time\t\t\t\t\t\t%d\n",count_resume);
		seq_puts(f, "function name\t\t\t\t\t\ttime\n");
		seq_puts(f, "-------------------------------------------------------------\n");
		list_for_each_entry(data, &resume_sorted_list, list) {
			if (data->duration > SEC_RESUME_DEBUG_MIN_TIME)
			seq_printf(f, "%-50pF : %8llu\n", data->callback, data->duration);
		}
	}
	else{
		seq_puts(f, "Device have not resume yet\n");
	}
	return 0;
}

static int sec_suspend_debug_seq_show(struct seq_file *f, void *v)
{
	struct sec_debug_data *data;
	if(!list_empty(&suspend_sorted_list)){
		seq_printf(f, "Suspend time\t\t\t\t\t\t%d\n",count_suspend);
		seq_puts(f, "function name\t\t\t\t\t\ttime\n");
		seq_puts(f, "-------------------------------------------------------------\n");
		list_for_each_entry(data, &suspend_sorted_list, list) {
			if (data->duration > SEC_SUSPEND_DEBUG_MIN_TIME)
			seq_printf(f, "%-50pF : %8llu\n", data->callback, data->duration);
		}
	}
	else{
		seq_puts(f, "Device have not suspend yet\n");
	}
	return 0;
}

static int sec_resume_debug_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, sec_resume_debug_seq_show, NULL);
}

static int sec_suspend_debug_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, sec_suspend_debug_seq_show, NULL);
}

static const struct file_operations sec_resume_debug_proc_fops = {
	.open	= sec_resume_debug_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

static const struct file_operations sec_suspend_debug_proc_fops = {
	.open	= sec_suspend_debug_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

void sec_debug_add(pm_callback_t callback, unsigned long long duration, int type)
{
	struct sec_debug_data *ddata;

	ddata = kzalloc(sizeof(struct sec_debug_data), GFP_KERNEL);
	if (!ddata)
		printk(KERN_ERR "suspend : failed to allocate\n");
	else {
		ddata->callback = callback;
		ddata->duration = duration;
		if(type == 0)
			list_add_tail(&ddata->list, &suspend_list);
		else
			list_add_tail(&ddata->list, &resume_list);
	}
}

static int sec_resume_debug_init(void)
{
	proc_create("resume_debug", 0, NULL,
		&sec_resume_debug_proc_fops);
	return 0;
}

static int sec_suspend_debug_init(void)
{
	proc_create("suspend_debug", 0, NULL,
		&sec_suspend_debug_proc_fops);
	return 0;
}

static ssize_t sec_enable_resume_suspend_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", debug_enable);
}

// Change debug_enable value with "echo" funtion. Echo "1" to start end echo "0" to exit
static ssize_t sec_enable_resume_suspend_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret;

    ret = kstrtoint(buf, 0, &debug_enable);
	
	if (ret) {
		pr_info("%s, SEC Resume and Suspend Debug Driver Enabling is failed. err = %d\n", __func__, ret);
		return ret;
	}

	if (debug_enable)
		pr_info("%s, SEC Resume and Suspend Debug Driver is %s\n", __func__, ((debug_enable == 1) ? "ENABLE" : "DISABLE") );
	
    return count;
}

static struct kobj_attribute sec_enable_resume_suspend = __ATTR(debug_enable, 0660, sec_enable_resume_suspend_show, sec_enable_resume_suspend_store);

//Make a sysfs to control debug working
static int __init sec_resume_suspend_debug_init(void)
{
	int error = 0;

        pr_debug("Module debug resume suspend initialized successfully \n");

        enable_resume_suspend = kobject_create_and_add("enable_resume_suspend", kernel_kobj);
        if(!enable_resume_suspend)
                return -ENOMEM;

        error = sysfs_create_file(enable_resume_suspend, &sec_enable_resume_suspend.attr);
        if (error) {
                pr_debug("failed to create the debug file in /sys/kernel/enable_resume_suspend \n");
        }
		sec_resume_debug_init();
		sec_suspend_debug_init();
        return error;
}
late_initcall(sec_resume_suspend_debug_init);
