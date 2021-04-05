/*
 * Copyright (c) 2013-2016 TRUSTONIC LIMITED
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MC_NQ_H_
#define _MC_NQ_H_

#include <linux/completion.h>
#include <linux/list.h>
#include <linux/mutex.h>

struct nq_session {
	/* Session id */
	u32			id;
	/* This TA is of Global Platform type, set by upper layer */
	bool			is_gp;
	/* Sessions list (protected by nq sessions_mutex) */
	struct list_head	list;
	/* Notifications list (protected by nq notifications_mutex) */
	struct list_head	notifications_list;
	/* Deferred notif. payload (protected by nq notifications_mutex) */
	u32			notification_payload;
	/* Notification debug (protected by notif_mutex) */
	enum nq_notif_state {
		NQ_NOTIF_IDLE,		/* Nothing happened yet */
		NQ_NOTIF_QUEUED,	/* Notification in overflow queue */
		NQ_NOTIF_SENT,		/* Notification in send queue */
		NQ_NOTIF_RECEIVED,	/* Notification received */
		NQ_NOTIF_CONSUMED,	/* Notification reported to CA */
		NQ_NOTIF_DEAD,		/* Error reported to CA */
	}			notif_state;
	/* Time at notification state change (protected by notif_mutex) */
	u64			notif_cpu_clk;
};

/* Initialisation/cleanup */
int nq_init(void);
void nq_exit(void);

/* Start/stop TEE */
int nq_start(void);
void nq_stop(void);

/* SWd suspend/resume */
int nq_suspend(void);
int nq_resume(void);
bool nq_suspended(void);

/*
 * Get the requested SWd sleep timeout value (ms)
 * - if the timeout is -1, wait indefinitely
 * - if the timeout is 0, re-schedule immediately (timeouts in µs in the SWd)
 * - otherwise sleep for the required time
 * returns true if sleep is required, false otherwise
 */
bool nq_get_idle_timeout(s32 *timeout);
void nq_reset_idle_timeout(void);

/* Services */
/* Callback to scheduler registration */
enum nq_scheduler_commands {
	MC_NQ_YIELD,
	MC_NQ_NSIQ,
};

void nq_register_scheduler(int (*scheduler_cb)(enum nq_scheduler_commands));
void nq_register_crash_handler(void (*crashhandler_cb)(void));
void nq_dump_status(void);
void nq_update_time(void);
union mcp_message *nq_get_mcp_message(void);
void nq_register_notif_handler(void (*handler)(struct nq_session *session,
					       u32 id, u32 payload),
			       bool gp);

/* Notifications */
void nq_session_init(struct nq_session *session, bool is_gp);
void nq_session_set_id(struct nq_session *session, u32 id);
bool nq_session_is_gp(const struct nq_session *session);
u64 nq_session_notif_cpu_clk(const struct nq_session *session);
void nq_session_exit(struct nq_session *session);
void nq_session_state_update(struct nq_session *session, bool dead);
const char *nq_session_state_string(struct nq_session *session);
int nq_session_notify(struct nq_session *session, u32 payload);
bool nq_notifications_flush(void);

#endif /* _MC_NQ_H_ */
