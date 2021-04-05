/****************************************************************************
 *
 * Copyright (c) 2014 - 2016 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#ifndef _SRVMAN_H
#define _SRVMAN_H

#include <linux/wakelock.h>

struct srvman;

void srvman_init(struct srvman *srvman, struct scsc_mx *mx);
int  srvman_suspend_services(struct srvman *srvman);
int  srvman_resume_services(struct srvman *srvman);
void srvman_freeze_services(struct srvman *srvman);
void srvman_unfreeze_services(struct srvman *srvman, u16 scsc_panic_code);
void srvman_set_error(struct srvman *srvman);
void srvman_clear_error(struct srvman *srvman);
void srvman_deinit(struct srvman *srvman);

struct srvman {
	struct scsc_mx   *mx;
	struct list_head service_list;
	struct mutex     service_list_mutex;
	struct mutex     api_access_mutex;
	bool             error;
	struct wake_lock sm_wake_lock;
};


#endif
