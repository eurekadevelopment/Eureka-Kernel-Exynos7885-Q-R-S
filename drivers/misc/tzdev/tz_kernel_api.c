/*
 * Copyright (C) 2012-2019 Samsung Electronics, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wakelock.h>

#include <tzdev/kernel_api.h>

#include "tz_iwio.h"
#include "tz_mem.h"
#include "tzdev.h"

struct kapi_ctx {
	struct list_head link;
	int client_id;
	bool is_in_progress;
};

static struct wake_lock wl;
static atomic_t wake_cntr = ATOMIC_INIT(0);
static LIST_HEAD(ctx_list);
static DEFINE_SPINLOCK(kapi_lock);

void tzdev_kapi_init(void)
{
	wake_lock_init(&wl, WAKE_LOCK_SUSPEND, "tzdev_kapi_wl");
}

static void tzdev_kapi_wake_inc(void)
{
	if (atomic_inc_return(&wake_cntr) == 1)
		wake_lock(&wl);
}

static void tzdev_kapi_wake_dec(void)
{
	if (atomic_dec_and_test(&wake_cntr))
		wake_unlock(&wl);
}

static void tzdev_kapi_work_started(int client_id)
{
	struct kapi_ctx *ctx;

	spin_lock(&kapi_lock);
	list_for_each_entry(ctx, &ctx_list, link) {
		if (ctx->client_id == client_id) {
			ctx->is_in_progress = true;
			tzdev_kapi_wake_inc();
			break;
		}
	}
	spin_unlock(&kapi_lock);
}

static void tzdev_kapi_work_finished(int client_id, bool remove_ctx)
{
	struct kapi_ctx *ctx;

	spin_lock(&kapi_lock);
	list_for_each_entry(ctx, &ctx_list, link) {
		if (ctx->client_id == client_id) {
			if (ctx->is_in_progress) {
				ctx->is_in_progress = false;
				tzdev_kapi_wake_dec();
			}
			if (remove_ctx) {
				list_del(&ctx->link);
				kfree(ctx);
			}
			break;
		}
	}
	spin_unlock(&kapi_lock);
}

int tzdev_kapi_open(const struct tz_uuid *uuid)
{
	struct tz_iwio_aux_channel *ch;
	struct kapi_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(struct kapi_ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ch = tz_iwio_get_aux_channel();
	memcpy(ch->buffer, uuid, TZ_UUID_LEN);
	ret = tzdev_smc_nw_kernel_api_cmd(NW_KERNEL_API_OPEN, 0, 0);
	tz_iwio_put_aux_channel();

	if (ret < 0) {
		kfree(ctx);
		return ret;
	}

	ctx->client_id = ret;

	spin_lock(&kapi_lock);
	list_add(&ctx->link, &ctx_list);
	spin_unlock(&kapi_lock);

	return ret;
}

int tzdev_kapi_close(int client_id)
{
	tzdev_kapi_work_finished(client_id, true);
	return tzdev_smc_nw_kernel_api_cmd(NW_KERNEL_API_CLOSE, client_id, 0);
}

int tzdev_kapi_send(int client_id, const void *data, size_t size)
{
	struct tz_iwio_aux_channel *ch;
	int ret;

	if (size > TZ_IWIO_AUX_BUF_SIZE)
		return -EINVAL;

	ch = tz_iwio_get_aux_channel();
	memcpy(ch->buffer, data, size);
	ret = tzdev_smc_nw_kernel_api_cmd(NW_KERNEL_API_SEND, client_id, size);
	tz_iwio_put_aux_channel();

	if (ret >= 0)
		tzdev_kapi_work_started(client_id);

	return ret;
}

int tzdev_kapi_recv(int client_id, void *buf, size_t size)
{
	struct tz_iwio_aux_channel *ch;
	int ret;

	ch = tz_iwio_get_aux_channel();
	ret = tzdev_smc_nw_kernel_api_cmd(NW_KERNEL_API_RECV, client_id, 0);
	if (ret > 0) {
		if (ret <= size)
			memcpy(buf, ch->buffer, ret);
		else
			ret = -ENOMEM;
	}
	tz_iwio_put_aux_channel();

	tzdev_kapi_work_finished(client_id, false);

	return ret;
}

int tzdev_kapi_mem_register(void *ptr, unsigned long size, unsigned int write)
{
	return tzdev_mem_register(ptr, size, write);
}

int tzdev_kapi_mem_pages_register(struct page **pages, unsigned int nr_pages, unsigned int write)
{
	return tzdev_mem_pages_register(pages, nr_pages, write);
}

int tzdev_kapi_mem_release(unsigned int id)
{
	return tzdev_mem_release(id);
}

int tzdev_kapi_mem_grant(int client_id, int mem_id)
{
	return tzdev_smc_nw_kernel_api_cmd(NW_KERNEL_API_MEM_GRANT, client_id, mem_id);
}

int tzdev_kapi_mem_revoke(int client_id, int mem_id)
{
	return tzdev_smc_nw_kernel_api_cmd(NW_KERNEL_API_MEM_REVOKE, client_id, mem_id);
}
