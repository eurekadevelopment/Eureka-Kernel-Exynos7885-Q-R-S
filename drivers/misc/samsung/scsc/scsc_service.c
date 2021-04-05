/****************************************************************************
 *
 * Copyright (c) 2014 - 2017 Samsung Electronics Co., Ltd. All rights reserved
 *
 ****************************************************************************/

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <scsc/scsc_mx.h>
#include <scsc/scsc_logring.h>

#include "mxman.h"
#include "scsc_mx_impl.h"
#include "mifintrbit.h"
#include "miframman.h"
#include "mifmboxman.h"
#include "mxlogger.h"
#include "srvman.h"
#include "servman_messages.h"
#include "mxmgmt_transport.h"

static ulong sm_completion_timeout_ms = 1000;
module_param(sm_completion_timeout_ms, ulong, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(sm_completion_timeout_ms, "Timeout Service Manager start/stop (ms) - default 1000. 0 = infinite");

#define	SCSC_MIFRAM_INVALID_REF		-1
#define SCSC_MX_SERVICE_RECOVERY_TIMEOUT 20000 /* 20 seconds */

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0))
#define reinit_completion(completion) INIT_COMPLETION(*(completion))
#endif



struct scsc_service {
	struct list_head           list;
	struct scsc_mx             *mx;
	enum scsc_service_id       id;
	struct scsc_service_client *client;
	struct completion          sm_msg_start_completion;
	struct completion          sm_msg_stop_completion;
};

void srvman_init(struct srvman *srvman, struct scsc_mx *mx)
{
	SCSC_TAG_INFO(MXMAN, "\n");
	srvman->mx = mx;
	INIT_LIST_HEAD(&srvman->service_list);
	mutex_init(&srvman->service_list_mutex);
	mutex_init(&srvman->api_access_mutex);

	wake_lock_init(&srvman->sm_wake_lock, WAKE_LOCK_SUSPEND, "srvman_wakelock");
}

void srvman_deinit(struct srvman *srvman)
{
	struct scsc_service *service, *next;

	SCSC_TAG_INFO(MXMAN, "\n");
	list_for_each_entry_safe(service, next, &srvman->service_list, list) {
		list_del(&service->list);
		kfree(service);
	}
	mutex_destroy(&srvman->api_access_mutex);
	mutex_destroy(&srvman->service_list_mutex);

	wake_lock_destroy(&srvman->sm_wake_lock);
}

void srvman_set_error(struct srvman *srvman)
{
	struct scsc_service *service;

	SCSC_TAG_INFO(MXMAN, "\n");
	srvman->error = true;
	mutex_lock(&srvman->service_list_mutex);
	list_for_each_entry(service, &srvman->service_list, list) {
		complete(&service->sm_msg_start_completion);
		complete(&service->sm_msg_stop_completion);
	}
	mutex_unlock(&srvman->service_list_mutex);
}

void srvman_clear_error(struct srvman *srvman)
{
	SCSC_TAG_INFO(MXMAN, "\n");
	srvman->error = false;
}

static int wait_for_sm_msg_start_cfm(struct scsc_service *service)
{
	int r;

	if (0 == sm_completion_timeout_ms) {
		/* Zero implies infinite wait, for development use only.
		 * r = -ERESTARTSYS if interrupted (e.g. Ctrl-C), 0 if completed
		 */
		r = wait_for_completion_interruptible(&service->sm_msg_start_completion);
		if (r == -ERESTARTSYS) {
			/* Paranoid sink of any pending event skipped by the interrupted wait */
			r = wait_for_completion_timeout(&service->sm_msg_start_completion, HZ / 2);
			if (r == 0) {
				SCSC_TAG_ERR(MXMAN, "timed out\n");
				return -ETIMEDOUT;
			}
		}
		return r;
	}
	r = wait_for_completion_timeout(&service->sm_msg_start_completion, msecs_to_jiffies(sm_completion_timeout_ms));
	if (r == 0) {
		SCSC_TAG_ERR(MXMAN, "timeout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int wait_for_sm_msg_stop_cfm(struct scsc_service *service)
{
	int r;

	if (0 == sm_completion_timeout_ms) {
		/* Zero implies infinite wait, for development use only.
		 * r = -ERESTARTSYS if interrupted (e.g. Ctrl-C), 0 if completed
		 */
		r = wait_for_completion_interruptible(&service->sm_msg_stop_completion);
		if (r == -ERESTARTSYS) {
			/* Paranoid sink of any pending event skipped by the interrupted wait */
			r = wait_for_completion_timeout(&service->sm_msg_stop_completion, HZ / 2);
			if (r == 0) {
				SCSC_TAG_ERR(MXMAN, "timed out\n");
				return -ETIMEDOUT;
			}
		}
		return r;
	}
	r = wait_for_completion_timeout(&service->sm_msg_stop_completion, msecs_to_jiffies(sm_completion_timeout_ms));
	if (r == 0) {
		SCSC_TAG_ERR(MXMAN, "timeout\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int send_sm_msg_start_blocking(struct scsc_service *service, scsc_mifram_ref ref)
{
	struct scsc_mx          *mx = service->mx;
	struct mxmgmt_transport *mxmgmt_transport = scsc_mx_get_mxmgmt_transport(mx);
	int                     r;
	struct sm_msg_packet    message = { .service_id = service->id,
					    .msg = SM_MSG_START_REQ,
					    .optional_data = ref };

	reinit_completion(&service->sm_msg_start_completion);

	/* Send to FW in MM stream */
	mxmgmt_transport_send(mxmgmt_transport, MMTRANS_CHAN_ID_SERVICE_MANAGEMENT, &message, sizeof(message));
	r = wait_for_sm_msg_start_cfm(service);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "wait_for_sm_msg_start_cfm() failed: r=%d\n", r);

		/* Report the error in order to get a moredump. Avoid auto-recovering this type of failure */
		if (mxman_recovery_disabled())
			scsc_mx_service_service_failed(service, "SM_MSG_START_CFM timeout");
	}
	return r;
}

static int send_sm_msg_stop_blocking(struct scsc_service *service)
{
	struct scsc_mx          *mx = service->mx;
	struct mxman            *mxman = scsc_mx_get_mxman(mx);
	struct mxmgmt_transport *mxmgmt_transport = scsc_mx_get_mxmgmt_transport(mx);
	int                     r;
	struct sm_msg_packet	message = { .service_id = service->id,
					    .msg = SM_MSG_STOP_REQ,
					    .optional_data = 0 };

	if (mxman->mxman_state == MXMAN_STATE_FAILED)
		return 0;

	reinit_completion(&service->sm_msg_stop_completion);

	/* Send to FW in MM stream */
	mxmgmt_transport_send(mxmgmt_transport, MMTRANS_CHAN_ID_SERVICE_MANAGEMENT, &message, sizeof(message));
	r = wait_for_sm_msg_stop_cfm(service);
	if (r)
		SCSC_TAG_ERR(MXMAN, "wait_for_sm_msg_stop_cfm() for service=%p service->id=%d failed: r=%d\n", service, service->id, r);
	return r;
}

/*
 * Receive handler for messages from the FW along the maxwell management transport
 */
static void srv_message_handler(const void *message, void *data)
{
	struct srvman       *srvman = (struct srvman *)data;
	struct scsc_service *service;
	const struct sm_msg_packet *msg = message;
	bool                found = false;

	mutex_lock(&srvman->service_list_mutex);
	list_for_each_entry(service, &srvman->service_list, list) {
		if (service->id == msg->service_id) {
			found = true;
			break;
		}
	}
	if (!found) {
		SCSC_TAG_ERR(MXMAN, "No service for msg->service_id=%d", msg->service_id);
		mutex_unlock(&srvman->service_list_mutex);
		return;
	}
	/* Forward the message to the applicable service to deal with */
	switch (msg->msg) {
	case SM_MSG_START_CFM:
		SCSC_TAG_INFO(MXMAN, "Received SM_MSG_START_CFM message service=%p with service_id=%d from the firmware\n",
			      service, msg->service_id);
		complete(&service->sm_msg_start_completion);
		break;
	case SM_MSG_STOP_CFM:
		SCSC_TAG_INFO(MXMAN, "Received SM_MSG_STOP_CFM message for service=%p with service_id=%d from the firmware\n",
			      service, msg->service_id);
		complete(&service->sm_msg_stop_completion);
		break;
	default:
		/* HERE: Unknown message, raise fault */
		SCSC_TAG_WARNING(MXMAN, "Received unknown message for service=%p with service_id=%d from the firmware: msg->msg=%d\n",
				 service, msg->msg, msg->service_id);
		break;
	}
	mutex_unlock(&srvman->service_list_mutex);
}

int scsc_mx_service_start(struct scsc_service *service, scsc_mifram_ref ref)
{
	struct scsc_mx *mx = service->mx;
	struct srvman *srvman = scsc_mx_get_srvman(mx);
	struct mxman *mxman = scsc_mx_get_mxman(service->mx);
	int                 r;
	struct timeval tval = {};

	SCSC_TAG_INFO(MXMAN, "\n");
#ifdef CONFIG_SCSC_CHV_SUPPORT
	if (chv_run)
		return 0;
#endif
	mutex_lock(&srvman->api_access_mutex);
	wake_lock(&srvman->sm_wake_lock);
	if (srvman->error) {
		tval = ns_to_timeval(mxman->last_panic_time);
		SCSC_TAG_ERR(MXMAN, "error: refused due to previous f/w failure scsc_panic_code=0x%x happened at [%6lu.%06ld]\n",
				mxman->scsc_panic_code, tval.tv_sec, tval.tv_usec);

		/* Print the last panic record to help track ancient failures */
		mxman_show_last_panic(mxman);

		wake_unlock(&srvman->sm_wake_lock);
		mutex_unlock(&srvman->api_access_mutex);
		return -EILSEQ;
	}

	r = send_sm_msg_start_blocking(service, ref);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "send_sm_msg_start_blocking() failed: r=%d\n", r);
		wake_unlock(&srvman->sm_wake_lock);
		mutex_unlock(&srvman->api_access_mutex);
		return r;
	}
	wake_unlock(&srvman->sm_wake_lock);
	mutex_unlock(&srvman->api_access_mutex);
	return 0;
}
EXPORT_SYMBOL(scsc_mx_service_start);

int scsc_mx_list_services(struct mxman *mxman_p, char *buf, const size_t bufsz)
{
	struct scsc_service *service, *next;
	int    pos = 0;
	struct   srvman  *srvman_p = scsc_mx_get_srvman(mxman_p->mx);

	list_for_each_entry_safe(service, next, &srvman_p->service_list, list) {
		switch (service->id) {
		case SCSC_SERVICE_ID_NULL:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "null");
			break;
		case SCSC_SERVICE_ID_WLAN:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "wlan");
			break;
		case SCSC_SERVICE_ID_BT:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "bt");
			break;
		case SCSC_SERVICE_ID_ANT:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "ant");
			break;
		case SCSC_SERVICE_ID_R4DBG:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "r4dbg");
			break;
		case SCSC_SERVICE_ID_ECHO:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "echo");
			break;
		case SCSC_SERVICE_ID_DBG_SAMPLER:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "dbg sampler");
			break;
		case SCSC_SERVICE_ID_CLK20MHZ:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "clk20mhz");
			break;
		case SCSC_SERVICE_ID_INVALID:
			pos += scnprintf(buf + pos, bufsz - pos, "%s\n", "invalid");
			break;
		}
	}
	return pos;
}
EXPORT_SYMBOL(scsc_mx_list_services);

int scsc_mx_service_stop(struct scsc_service *service)
{
	struct scsc_mx *mx = service->mx;
	struct srvman *srvman = scsc_mx_get_srvman(mx);
	struct mxman *mxman = scsc_mx_get_mxman(service->mx);
	int r;
	struct timeval tval = {};

	SCSC_TAG_INFO(MXMAN, "\n");
#ifdef CONFIG_SCSC_CHV_SUPPORT
	if (chv_run)
		return 0;
#endif
	mutex_lock(&srvman->api_access_mutex);
	wake_lock(&srvman->sm_wake_lock);
	if (srvman->error) {
		tval = ns_to_timeval(mxman->last_panic_time);
		SCSC_TAG_ERR(MXMAN, "error: refused due to previous f/w failure scsc_panic_code=0x%x happened at [%6lu.%06ld]\n",
				mxman->scsc_panic_code, tval.tv_sec, tval.tv_usec);

		/* Print the last panic record to help track ancient failures */
		mxman_show_last_panic(mxman);

		wake_unlock(&srvman->sm_wake_lock);
		mutex_unlock(&srvman->api_access_mutex);

		/* Return a special status to allow caller recovery logic to know
		 * that there will never be a recovery
		 */
		if (mxman_recovery_disabled()) {
			SCSC_TAG_ERR(MXMAN, "recovery disabled, return -EPERM (%d)\n", -EPERM);
			return -EPERM; /* failed due to prior failure, recovery disabled */
		} else {
			return -EILSEQ; /* operation rejected due to prior failure */
		}
	}

	r = send_sm_msg_stop_blocking(service);
	if (r) {
		SCSC_TAG_ERR(MXMAN, "send_sm_msg_stop_blocking() failed: r=%d\n", r);
		wake_unlock(&srvman->sm_wake_lock);
		mutex_unlock(&srvman->api_access_mutex);
		return -EIO; /* operation failed */
	}

	wake_unlock(&srvman->sm_wake_lock);
	mutex_unlock(&srvman->api_access_mutex);
	return 0;
}
EXPORT_SYMBOL(scsc_mx_service_stop);


/* Returns 0 if Suspend succeeded, otherwise return error */
int srvman_suspend_services(struct srvman *srvman)
{
	int ret = 0;
	struct scsc_service *service;

	SCSC_TAG_INFO(MXMAN, "\n");
	mutex_lock(&srvman->service_list_mutex);
	list_for_each_entry(service, &srvman->service_list, list) {
		if (service->client->suspend) {
			ret = service->client->suspend(service->client);
			/* If any service returns error message and call resume callbacks */
			if (ret) {
				list_for_each_entry(service, &srvman->service_list, list) {
					if (service->client->resume)
						service->client->resume(service->client);
				}
				SCSC_TAG_INFO(MXMAN, "Service client suspend failure ret: %d\n", ret);
				mutex_unlock(&srvman->service_list_mutex);
				return ret;
			}
		}
	}

	mutex_unlock(&srvman->service_list_mutex);
	SCSC_TAG_INFO(MXMAN, "OK\n");
	return 0;
}

/* Returns always 0. Extend API and return value if required */
int srvman_resume_services(struct srvman *srvman)
{
	struct scsc_service *service;

	SCSC_TAG_INFO(MXMAN, "\n");
	mutex_lock(&srvman->service_list_mutex);
	list_for_each_entry(service, &srvman->service_list, list) {
		if (service->client->resume)
			service->client->resume(service->client);
	}

	mutex_unlock(&srvman->service_list_mutex);
	SCSC_TAG_INFO(MXMAN, "OK\n");

	return 0;
}

void srvman_freeze_services(struct srvman *srvman)
{
	struct scsc_service *service;
	struct mxman        *mxman = scsc_mx_get_mxman(srvman->mx);

	SCSC_TAG_INFO(MXMAN, "\n");
	mutex_lock(&srvman->service_list_mutex);
	list_for_each_entry(service, &srvman->service_list, list) {
		service->client->stop_on_failure(service->client);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0)
	reinit_completion(&mxman->recovery_completion);
#else
	INIT_COMPLETION(mxman->recovery_completion);
#endif
	mutex_unlock(&srvman->service_list_mutex);
	SCSC_TAG_INFO(MXMAN, "OK\n");
}

void srvman_unfreeze_services(struct srvman *srvman, u16 scsc_panic_code)
{
	struct scsc_service *service;

	SCSC_TAG_INFO(MXMAN, "\n");
	mutex_lock(&srvman->service_list_mutex);
	list_for_each_entry(service, &srvman->service_list, list) {
		service->client->failure_reset(service->client, scsc_panic_code);
	}
	mutex_unlock(&srvman->service_list_mutex);
	SCSC_TAG_INFO(MXMAN, "OK\n");
}

/** Signal a failure detected by the Client. This will trigger the systemwide
 * failure handling procedure: _All_ Clients will be called back via
 * their stop_on_failure() handler as a side-effect.
 */
void scsc_mx_service_service_failed(struct scsc_service *service, const char *reason)
{
	struct scsc_mx *mx = service->mx;
	struct srvman  *srvman = scsc_mx_get_srvman(mx);
	u16 host_panic_code;

	host_panic_code = (SCSC_PANIC_CODE_HOST << 15) | (service->id << SCSC_SYSERR_HOST_SERVICE_SHIFT);

	srvman_set_error(srvman);
	switch (service->id) {
	case SCSC_SERVICE_ID_WLAN:
		SCSC_TAG_INFO(MXMAN, "WLAN: %s\n", ((reason != NULL) ? reason : ""));
		break;
	case SCSC_SERVICE_ID_BT:
		SCSC_TAG_INFO(MXMAN, "BT: %s\n", ((reason != NULL) ? reason : ""));
		break;
	default:
		SCSC_TAG_INFO(MXMAN, "service id %d failed\n", service->id);
		break;

	}

	SCSC_TAG_INFO(MXMAN, "Reporting host hang code 0x%02x\n", host_panic_code);

	mxman_fail(scsc_mx_get_mxman(mx), host_panic_code, reason);
}
EXPORT_SYMBOL(scsc_mx_service_service_failed);


int scsc_mx_service_close(struct scsc_service *service)
{
	struct mxman   *mxman = scsc_mx_get_mxman(service->mx);
	struct scsc_mx *mx = service->mx;
	struct srvman  *srvman = scsc_mx_get_srvman(mx);
	bool           empty;
	struct timeval tval = {};

	SCSC_TAG_INFO(MXMAN, "\n");
	mutex_lock(&srvman->api_access_mutex);
	wake_lock(&srvman->sm_wake_lock);

	if (srvman->error) {
		tval = ns_to_timeval(mxman->last_panic_time);
		SCSC_TAG_ERR(MXMAN, "error: refused due to previous f/w failure scsc_panic_code=0x%x happened at [%6lu.%06ld]\n",
				mxman->scsc_panic_code, tval.tv_sec, tval.tv_usec);

		/* Print the last panic record to help track ancient failures */
		mxman_show_last_panic(mxman);

		mutex_unlock(&srvman->api_access_mutex);
		wake_unlock(&srvman->sm_wake_lock);

		/* Return a special status when recovery is disabled, to allow
		 * calling recovery logic to be aware that recovery is disabled,
		 * hence not wait for recovery events.
		 */
		if (mxman_recovery_disabled()) {
			SCSC_TAG_ERR(MXMAN, "recovery disabled, return -EPERM (%d)\n", -EPERM);
			return -EPERM; /* rejected due to prior failure, recovery disabled */
		} else {
			return -EIO;
		}
	}

	/* remove the service from the list and deallocate the service memory */
	mutex_lock(&srvman->service_list_mutex);
	list_del(&service->list);
	empty = list_empty(&srvman->service_list);
	mutex_unlock(&srvman->service_list_mutex);
	if (empty) {
		/* unregister channel handler */
		mxmgmt_transport_register_channel_handler(scsc_mx_get_mxmgmt_transport(mx), MMTRANS_CHAN_ID_SERVICE_MANAGEMENT,
							  NULL, NULL);
	}

	kfree(service);
	mxman_close(mxman);
	wake_unlock(&srvman->sm_wake_lock);
	mutex_unlock(&srvman->api_access_mutex);
	return 0;
}
EXPORT_SYMBOL(scsc_mx_service_close);

/* Consider move to a public scsc_mx interface */
struct scsc_service *scsc_mx_service_open(struct scsc_mx *mx, enum scsc_service_id id, struct scsc_service_client *client, int *status)
{
	int                 ret;
	struct scsc_service *service;
	struct srvman       *srvman = scsc_mx_get_srvman(mx);
	struct mxman        *mxman = scsc_mx_get_mxman(mx);
	bool                empty;
	struct timeval tval = {};

	SCSC_TAG_INFO(MXMAN, "\n");

	mutex_lock(&srvman->api_access_mutex);
	wake_lock(&srvman->sm_wake_lock);
	if (srvman->error) {
		tval = ns_to_timeval(mxman->last_panic_time);
		SCSC_TAG_ERR(MXMAN, "error: refused due to previous f/w failure scsc_panic_code=0x%x happened at [%6lu.%06ld]\n",
				mxman->scsc_panic_code, tval.tv_sec, tval.tv_usec);
		/* Print the last panic record to help track ancient failures */
		mxman_show_last_panic(mxman);
		wake_unlock(&srvman->sm_wake_lock);
		mutex_unlock(&srvman->api_access_mutex);
		*status = -EILSEQ;
		return NULL;
	}

	if (mxman->mxman_state == MXMAN_STATE_FAILED) {
		int r;

		mutex_unlock(&srvman->api_access_mutex);
		r = wait_for_completion_timeout(&mxman->recovery_completion,
						msecs_to_jiffies(SCSC_MX_SERVICE_RECOVERY_TIMEOUT));
		if (r == 0) {
			SCSC_TAG_ERR(MXMAN, "Recovery timeout\n");
			wake_unlock(&srvman->sm_wake_lock);
			*status = -EIO;
			return NULL;
		}

		mutex_lock(&srvman->api_access_mutex);
	}

	service = kmalloc(sizeof(struct scsc_service), GFP_KERNEL);
	if (service) {
		/* MaxwellManager Should allocate Mem and download FW */
		ret = mxman_open(mxman);
		if (ret) {
			kfree(service);
			wake_unlock(&srvman->sm_wake_lock);
			mutex_unlock(&srvman->api_access_mutex);
			*status = ret;
			return NULL;
		}
		/* Initialise service struct here */
		service->mx = mx;
		service->id = id;
		service->client = client;
		init_completion(&service->sm_msg_start_completion);
		init_completion(&service->sm_msg_stop_completion);
		mutex_lock(&srvman->service_list_mutex);
		empty = list_empty(&srvman->service_list);
		mutex_unlock(&srvman->service_list_mutex);
		if (empty)
			mxmgmt_transport_register_channel_handler(scsc_mx_get_mxmgmt_transport(mx), MMTRANS_CHAN_ID_SERVICE_MANAGEMENT,
								  &srv_message_handler, srvman);
		mutex_lock(&srvman->service_list_mutex);
		list_add_tail(&service->list, &srvman->service_list);
		mutex_unlock(&srvman->service_list_mutex);
	} else
		*status = -ENOMEM;

	wake_unlock(&srvman->sm_wake_lock);
	mutex_unlock(&srvman->api_access_mutex);

	return service;
}
EXPORT_SYMBOL(scsc_mx_service_open);


/**
 * Allocate a contiguous block of SDRAM accessible to Client Driver
 *
 * When allocation fails, beside returning -ENOMEM, the IN-param 'ref'
 * is cleared to an INVALID value that can be safely fed to the companion
 * function scsc_mx_service_mifram_free().
 */
int scsc_mx_service_mifram_alloc(struct scsc_service *service, size_t nbytes, scsc_mifram_ref *ref, u32 align)
{
	struct scsc_mx      *mx = service->mx;
	void                *mem;
	int                 ret;

	mem = miframman_alloc(scsc_mx_get_ramman(mx), nbytes, align, service->id);
	if (!mem) {
		SCSC_TAG_ERR(MXMAN, "miframman_alloc() failed\n");
		*ref = SCSC_MIFRAM_INVALID_REF;
		return -ENOMEM;
	}

	SCSC_TAG_DEBUG(MXMAN, "Allocated mem %p\n", mem);

	/* Transform native pointer and get mifram_ref type */
	ret = scsc_mx_service_mif_ptr_to_addr(service, mem, ref);
	if (ret) {
		SCSC_TAG_ERR(MXMAN, "scsc_mx_service_mif_ptr_to_addr() failed: ret=%d", ret);
		miframman_free(scsc_mx_get_ramman(mx), mem);
		*ref = SCSC_MIFRAM_INVALID_REF;
	} else {
		SCSC_TAG_DEBUG(MXMAN, "mem %p ref %d\n", mem, *ref);
	}
	return ret;
}
EXPORT_SYMBOL(scsc_mx_service_mifram_alloc);

/** Free a contiguous block of SDRAM */
void scsc_mx_service_mifram_free(struct scsc_service *service, scsc_mifram_ref ref)
{
	struct scsc_mx *mx = service->mx;
	void           *mem;

	mem = scsc_mx_service_mif_addr_to_ptr(service, ref);

	SCSC_TAG_DEBUG(MXMAN, "**** Freeing %p\n", mem);

	miframman_free(scsc_mx_get_ramman(mx), mem);
}
EXPORT_SYMBOL(scsc_mx_service_mifram_free);

/* MIF ALLOCATIONS */
bool scsc_mx_service_alloc_mboxes(struct scsc_service *service, int n, int *first_mbox_index)
{
	struct scsc_mx *mx = service->mx;

	return mifmboxman_alloc_mboxes(scsc_mx_get_mboxman(mx), n, first_mbox_index);
}
EXPORT_SYMBOL(scsc_mx_service_alloc_mboxes);

void scsc_service_free_mboxes(struct scsc_service *service, int n, int first_mbox_index)
{
	struct scsc_mx *mx = service->mx;

	return mifmboxman_free_mboxes(scsc_mx_get_mboxman(mx), first_mbox_index, n);
}
EXPORT_SYMBOL(scsc_service_free_mboxes);

u32 *scsc_mx_service_get_mbox_ptr(struct scsc_service *service, int mbox_index)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	return mifmboxman_get_mbox_ptr(scsc_mx_get_mboxman(mx), mif_abs, mbox_index);
}
EXPORT_SYMBOL(scsc_mx_service_get_mbox_ptr);

int scsc_service_mifintrbit_bit_mask_status_get(struct scsc_service *service)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	return mif_abs->irq_bit_mask_status_get(mif_abs);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_bit_mask_status_get);

int scsc_service_mifintrbit_get(struct scsc_service *service)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	return mif_abs->irq_get(mif_abs);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_get);

void scsc_service_mifintrbit_bit_set(struct scsc_service *service, int which_bit, enum scsc_mifintr_target dir)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	return mif_abs->irq_bit_set(mif_abs, which_bit, dir);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_bit_set);

void scsc_service_mifintrbit_bit_clear(struct scsc_service *service, int which_bit)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	return mif_abs->irq_bit_clear(mif_abs, which_bit);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_bit_clear);

void scsc_service_mifintrbit_bit_mask(struct scsc_service *service, int which_bit)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	return mif_abs->irq_bit_mask(mif_abs, which_bit);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_bit_mask);

void scsc_service_mifintrbit_bit_unmask(struct scsc_service *service, int which_bit)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	return mif_abs->irq_bit_unmask(mif_abs, which_bit);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_bit_unmask);

int scsc_service_mifintrbit_alloc_fromhost(struct scsc_service *service, enum scsc_mifintr_target dir)
{
	struct scsc_mx *mx = service->mx;

	return mifintrbit_alloc_fromhost(scsc_mx_get_intrbit(mx), dir);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_alloc_fromhost);

int scsc_service_mifintrbit_free_fromhost(struct scsc_service *service, int which_bit, enum scsc_mifintr_target dir)
{
	struct scsc_mx *mx = service->mx;

	return mifintrbit_free_fromhost(scsc_mx_get_intrbit(mx), which_bit, dir);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_free_fromhost);

int scsc_service_mifintrbit_register_tohost(struct scsc_service *service, void (*handler)(int irq, void *data), void *data)
{
	struct scsc_mx *mx = service->mx;

	SCSC_TAG_DEBUG(MXMAN, "Registering %pS\n", handler);

	return mifintrbit_alloc_tohost(scsc_mx_get_intrbit(mx), handler, data);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_register_tohost);

int scsc_service_mifintrbit_unregister_tohost(struct scsc_service *service, int which_bit)
{
	struct scsc_mx *mx = service->mx;

	SCSC_TAG_DEBUG(MXMAN, "Deregistering int for bit %d\n", which_bit);
	return mifintrbit_free_tohost(scsc_mx_get_intrbit(mx), which_bit);
}
EXPORT_SYMBOL(scsc_service_mifintrbit_unregister_tohost);

void *scsc_mx_service_mif_addr_to_ptr(struct scsc_service *service, scsc_mifram_ref ref)
{
	struct scsc_mx      *mx = service->mx;

	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	return mif_abs->get_mifram_ptr(mif_abs, ref);
}
EXPORT_SYMBOL(scsc_mx_service_mif_addr_to_ptr);

void *scsc_mx_service_mif_addr_to_phys(struct scsc_service *service, scsc_mifram_ref ref)
{
	struct scsc_mx      *mx = service->mx;

	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	if (mif_abs->get_mifram_phy_ptr)
		return mif_abs->get_mifram_phy_ptr(mif_abs, ref);
	else
		return NULL;
}
EXPORT_SYMBOL(scsc_mx_service_mif_addr_to_phys);

int scsc_mx_service_mif_ptr_to_addr(struct scsc_service *service, void *mem_ptr, scsc_mifram_ref *ref)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	/* Transform native pointer and get mifram_ref type */
	if (mif_abs->get_mifram_ref(mif_abs, mem_ptr, ref)) {
		SCSC_TAG_ERR(MXMAN, "ooops somethig went wrong");
		return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL(scsc_mx_service_mif_ptr_to_addr);

int scsc_mx_service_mif_dump_registers(struct scsc_service *service)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;

	mif_abs = scsc_mx_get_mif_abs(mx);

	/* Dump registers */
	mif_abs->mif_dump_registers(mif_abs);

	return 0;
}
EXPORT_SYMBOL(scsc_mx_service_mif_dump_registers);

int scsc_mx_service_get_abox_shared_mem(struct scsc_service *service, void **data)
{
	struct scsc_mx      *mx = service->mx;
	struct scsc_mif_abs *mif_abs;
	int ret = -EPERM;

	if (data) {
		mif_abs = scsc_mx_get_mif_abs(mx);

		if (mif_abs->get_abox_shared_mem)
			mif_abs->get_abox_shared_mem(mif_abs, data);

		ret = 0;
	}
	return ret;
}
EXPORT_SYMBOL(scsc_mx_service_get_abox_shared_mem);

struct device *scsc_service_get_device(struct scsc_service *service)
{
	return scsc_mx_get_device(service->mx);
}
EXPORT_SYMBOL(scsc_service_get_device);

/* Force a FW panic for test purposes only */
int scsc_service_force_panic(struct scsc_service *service)
{
	struct mxman   *mxman = scsc_mx_get_mxman(service->mx);

	return mxman_force_panic(mxman);
}
EXPORT_SYMBOL(scsc_service_force_panic);
#ifdef CONFIG_SCSC_MXLOGGER
/* If there is no service/mxman associated, register the observer as global (will affect all the mx instanes)*/
/* Users of these functions should ensure that the registers/unregister functions are balanced (i.e. if observer is registed as global,
 * it _has_ to unregister as global) */
int scsc_service_register_observer(struct scsc_service *service, char *name)
{
	struct scsc_mx      *mx;

	if (!service)
		return mxlogger_register_global_observer(name);

	mx = service->mx;

	if (!mx)
		return -EIO;

	return mxlogger_register_observer(scsc_mx_get_mxlogger(mx), name);
}
EXPORT_SYMBOL(scsc_service_register_observer);

/* If there is no service/mxman associated, unregister the observer as global (will affect all the mx instanes)*/
int scsc_service_unregister_observer(struct scsc_service *service, char *name)
{
	struct scsc_mx      *mx;

	if (!service)
		return mxlogger_unregister_global_observer(name);

	mx = service->mx;

	if (!mx)
		return -EIO;

	return mxlogger_unregister_observer(scsc_mx_get_mxlogger(mx), name);
}
EXPORT_SYMBOL(scsc_service_unregister_observer);
#endif
