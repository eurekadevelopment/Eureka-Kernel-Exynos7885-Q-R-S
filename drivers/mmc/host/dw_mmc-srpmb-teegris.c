/*
 * Secure RPMB Driver for Exynos MMC RPMB
 *
 * Copyright (C) 2016 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/smc.h>
#include <linux/blkdev.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/core.h>
#include <linux/mmc/ioctl.h>
#include <linux/mmc/mmc.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/kthread.h>

#include "tz_iwsock.h"

#define GET_WRITE_COUNTER			1
#define WRITE_DATA				2
#define READ_DATA				3

#define RESULT_READ_REQ				0x0005

#define RPMB_PACKET_SIZE 			512
#define RPMB_REQRES                             510
#define RPMB_RESULT                             508

#define WRITE_COUNTER_SECURITY_OUT_ERROR	0x602
#define WRITE_COUNTER_SECURITY_IN_ERROR		0x603
#define WRITE_DATA_SECURITY_OUT_ERROR		0x605
#define WRITE_DATA_SECURITY_IN_ERROR		0x607
#define READ_LEN_ERROR				0x608
#define READ_DATA_SECURITY_OUT_ERROR		0x609
#define READ_DATA_SECURITY_IN_ERROR		0x60A
#define RPMB_INVALID_COMMAND			0x60B
#define RPMB_FAIL_SUSPEND_STATUS		0x60C

#define RPMB_IN_PROGRESS			0xDCDC
#define RPMB_PASSED				0xBABA

#define RPMB_BUF_MAX_SIZE			(32 * 1024)
#define RELIABLE_WRITE_REQ_SET			(1 << 31)

#define MMC_SRPMB_DEVICE_PROPNAME	"samsung,mmc-srpmb"
#define MMC_BLOCK_NAME			"/dev/block/mmcblk0rpmb"
#define TZ_RPMB_SOCK_NAME		"rpmb_socket"

#define RPMB_REQUEST_MAGIC		0x44444444
#define RPMB_REPLY_MAGIC		0x66666666

struct mmc_srpmb_req {
	uint32_t cmd;
	volatile uint32_t status_flag;
	uint32_t type;
	uint32_t data_len;
	uint32_t inlen;
	uint32_t outlen;
	uint8_t rpmb_data[0];
};

struct rpmb_packet {
	uint16_t request;
	uint16_t result;
	uint16_t count;
	uint16_t address;
	uint32_t write_counter;
	uint8_t nonce[16];
	uint8_t data[256];
	uint8_t Key_MAC[32];
	uint8_t stuff[196];
};

struct mmc_srpmb_ctx {
	struct platform_device *pdev;
	struct block_device *bdev;
	const struct block_device_operations *fops;

	int irq;
	irq_hw_number_t hwirq;

	void *wsm_virtaddr;
	dma_addr_t wsm_phyaddr;

	struct mmc_srpmb_req *req;
	spinlock_t lock;

	struct wake_lock wakelock;

	struct rpmb_packet packet;
	struct rpmb_packet result_packet;
};

static struct task_struct *srpmb_kthread;

static void swap_packet(struct rpmb_packet *src, struct rpmb_packet *dst)
{
	int i;
	char *src_buf = (char *)src;
	char *dst_buf = (char *)dst;

	for (i = 0; i < RPMB_PACKET_SIZE; i++)
		dst_buf[i] = src_buf[RPMB_PACKET_SIZE - 1 - i];
}

static void mmc_cmd_init(struct mmc_ioc_cmd *icmd)
{
	icmd->is_acmd = false;
	icmd->arg = 0;
	icmd->flags = MMC_RSP_R1;
	icmd->blksz = RPMB_PACKET_SIZE;
	icmd->blocks = 1;
	icmd->postsleep_min_us = 0;
	icmd->postsleep_max_us = 0;
	icmd->data_timeout_ns = 0;
	icmd->cmd_timeout_ms = 0;
}

static void mmc_srpmb_update_status_flag(struct mmc_srpmb_ctx *ctx, int status)
{
	unsigned long flags;

	spin_lock_irqsave(&ctx->lock, flags);
	ctx->req->status_flag = status;
	spin_unlock_irqrestore(&ctx->lock, flags);
}

static void mmc_srpmb_get_write_counter(struct mmc_srpmb_ctx *ctx)
{
	int ret;
	unsigned int status;
	struct mmc_ioc_cmd icmd;
	struct device *dev = &ctx->pdev->dev;
	struct block_device *bdev = ctx->bdev;
	const struct block_device_operations *fops = ctx->fops;

	mmc_cmd_init(&icmd);

	icmd.write_flag = true;
	icmd.flags = MMC_RSP_R1;
	icmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
	icmd.data_ptr = (unsigned long)ctx->req->rpmb_data;

	wake_lock(&ctx->wakelock);
	fops->get_card(bdev, 1);

	mmc_srpmb_update_status_flag(ctx, RPMB_IN_PROGRESS);

	ret = fops->srpmb_access(bdev, &icmd);
	if (ret) {
		status = WRITE_COUNTER_SECURITY_OUT_ERROR;
		dev_err(dev, "Fail to execute for srpmb write counter security out: %d\n", ret);
		goto out;
	}

	memset(ctx->req->rpmb_data, 0, RPMB_PACKET_SIZE);
	icmd.write_flag = false;
	icmd.flags = MMC_RSP_R1;
	icmd.opcode = MMC_READ_MULTIPLE_BLOCK;

	ret = fops->srpmb_access(bdev, &icmd);
	if (ret) {
		status = WRITE_COUNTER_SECURITY_IN_ERROR;
		dev_err(dev, "Fail to execute for srpmb write counter security in: %d\n", ret);
		goto out;
	}
	if (ctx->req->rpmb_data[RPMB_RESULT] || ctx->req->rpmb_data[RPMB_RESULT+1]) {
		dev_info(dev, "GET_WRITE_COUNTER: REQ/RES = %02x%02x, RESULT = %02x%02x\n",
			 ctx->req->rpmb_data[RPMB_REQRES], ctx->req->rpmb_data[RPMB_REQRES+1],
			 ctx->req->rpmb_data[RPMB_RESULT], ctx->req->rpmb_data[RPMB_RESULT+1]);
	}
	status = RPMB_PASSED;
out:
	mmc_srpmb_update_status_flag(ctx, status);
	fops->get_card(bdev, 0);
	wake_unlock(&ctx->wakelock);
}

static void mmc_srpmb_write(struct mmc_srpmb_ctx *ctx)
{
	int ret;
	unsigned int status;
	struct mmc_ioc_cmd icmd;
	struct device *dev = &ctx->pdev->dev;
	struct block_device *bdev = ctx->bdev;
	const struct block_device_operations *fops = ctx->fops;

	mmc_cmd_init(&icmd);

	icmd.write_flag = RELIABLE_WRITE_REQ_SET;
	icmd.flags = MMC_RSP_R1;
	icmd.blocks = ctx->req->data_len / RPMB_PACKET_SIZE;
	icmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
	icmd.data_ptr = (unsigned long)ctx->req->rpmb_data;

	if (!icmd.blocks) {
		dev_err(dev, "Invalid block size from secure world\n"
				"cmd(%d), type(%d), data length(%d)\n",
				ctx->req->cmd, ctx->req->type, ctx->req->data_len);
		return;
	}

	wake_lock(&ctx->wakelock);
	fops->get_card(bdev, 1);

	mmc_srpmb_update_status_flag(ctx, RPMB_IN_PROGRESS);

	ret = fops->srpmb_access(bdev, &icmd);
	if (ret) {
		status = WRITE_DATA_SECURITY_OUT_ERROR;
		dev_err(dev, "Fail to write block for program data: %d\n", ret);
		goto out;
	}

	icmd.write_flag = true;
	icmd.blocks = 1;
	icmd.data_ptr = (unsigned long)&ctx->result_packet;

	memset(&ctx->packet, 0, RPMB_PACKET_SIZE);
	ctx->packet.request = RESULT_READ_REQ;
	swap_packet(&ctx->packet, &ctx->result_packet);

	ret = fops->srpmb_access(bdev, &icmd);
	if (ret) {
		status = WRITE_DATA_SECURITY_OUT_ERROR;
		dev_err(dev, "Fail to write block for result: %d\n", ret);
		goto out;
	}

	memset(&ctx->result_packet, 0, RPMB_PACKET_SIZE);
	icmd.write_flag = false;
	icmd.blocks = 1;
	icmd.opcode = MMC_READ_MULTIPLE_BLOCK;

	ret = fops->srpmb_access(bdev, &icmd);
	if (ret) {
		status = WRITE_DATA_SECURITY_IN_ERROR;
		dev_err(dev, "Fail to read block for response: %d\n", ret);
		goto out;
	}
	if (ctx->req->rpmb_data[RPMB_RESULT] || ctx->req->rpmb_data[RPMB_RESULT+1]) {
		dev_info(dev, "WRITE_DATA: REQ/RES = %02x%02x, RESULT = %02x%02x\n",
			 ctx->req->rpmb_data[RPMB_REQRES], ctx->req->rpmb_data[RPMB_REQRES+1],
			 ctx->req->rpmb_data[RPMB_RESULT], ctx->req->rpmb_data[RPMB_RESULT+1]);
	}
	memcpy(&ctx->req->rpmb_data, &ctx->result_packet, RPMB_PACKET_SIZE);
	status = RPMB_PASSED;
out:
	mmc_srpmb_update_status_flag(ctx, status);
	fops->get_card(bdev, 0);
	wake_unlock(&ctx->wakelock);
}

static void mmc_srpmb_read(struct mmc_srpmb_ctx *ctx)
{
	int ret;
	unsigned int status;
	struct mmc_ioc_cmd icmd;
	struct device *dev = &ctx->pdev->dev;
	struct block_device *bdev = ctx->bdev;
	const struct block_device_operations *fops = ctx->fops;

	mmc_cmd_init(&icmd);

	icmd.write_flag = true;
	icmd.flags = MMC_RSP_R1;
	icmd.opcode = MMC_WRITE_MULTIPLE_BLOCK;
	icmd.data_ptr = (unsigned long)ctx->req->rpmb_data;

	wake_lock(&ctx->wakelock);
	fops->get_card(bdev, 1);

	mmc_srpmb_update_status_flag(ctx, RPMB_IN_PROGRESS);

	swap_packet((struct rpmb_packet *)ctx->req->rpmb_data, &ctx->result_packet);
	ctx->result_packet.count = 0;
	swap_packet(&ctx->result_packet, (struct rpmb_packet *)ctx->req->rpmb_data);

	ret = fops->srpmb_access(bdev, &icmd);
	if (ret) {
		status = READ_DATA_SECURITY_OUT_ERROR;
		dev_err(dev, "Fail to write block for read data: %d\n", ret);
		goto out;
	}

	memset(ctx->req->rpmb_data, 0, ctx->req->data_len);
	icmd.write_flag = false;
	icmd.opcode = MMC_READ_MULTIPLE_BLOCK;
	icmd.blocks = ctx->req->data_len/RPMB_PACKET_SIZE;

	if (!icmd.blocks) {
		status = READ_LEN_ERROR;
		dev_err(dev, "Invalid block size from secure world\n"
				"cmd(%d), type(%d), data length(%d)\n",
				ctx->req->cmd, ctx->req->type, ctx->req->data_len);
		ret = -EINVAL;
		goto out;
	}

	ret = fops->srpmb_access(bdev, &icmd);
	if (ret) {
		status = READ_DATA_SECURITY_IN_ERROR;
		dev_err(dev, "Fail to read block for response: %d\n", ret);
		goto out;
	}
	if (ctx->req->rpmb_data[RPMB_RESULT] || ctx->req->rpmb_data[RPMB_RESULT+1]) {
		dev_info(dev, "READ_DATA: REQ/RES = %02x%02x, RESULT = %02x%02x\n",
			 ctx->req->rpmb_data[RPMB_REQRES], ctx->req->rpmb_data[RPMB_REQRES+1],
			 ctx->req->rpmb_data[RPMB_RESULT], ctx->req->rpmb_data[RPMB_RESULT+1]);
	}
	status = RPMB_PASSED;
out:
	mmc_srpmb_update_status_flag(ctx, status);
	fops->get_card(bdev, 0);
	wake_unlock(&ctx->wakelock);
}

static struct sock_desc *mmc_srpmb_accept_swd_connection(struct device *dev)
{
	int ret;
	struct sock_desc *srpmb_conn;
	struct sock_desc *srpmb_listen;

	srpmb_listen = tz_iwsock_socket(1);
	if (IS_ERR(srpmb_listen)) {
		dev_err(dev, "failed to create iwd socket, err = %ld\n", PTR_ERR(srpmb_listen));
		return srpmb_listen;
	}
	dev_info(dev, "Created socket\n");

	ret = tz_iwsock_listen(srpmb_listen, TZ_RPMB_SOCK_NAME);
	if (ret) {
		dev_err(dev, "failed make iwd socket listening, err = %d\n", ret);
		srpmb_conn = ERR_PTR(ret);
		goto out;
	}
	dev_info(dev, "Make socket listening\n");

	/* Accept connection */
	srpmb_conn = tz_iwsock_accept(srpmb_listen);
	if (IS_ERR(srpmb_conn)) {
		dev_err(dev, "failed to accept connection, err = %ld\n", PTR_ERR(srpmb_conn));
		goto out;
	}
	dev_info(dev, "Accepted connection\n");
out:
	tz_iwsock_release(srpmb_listen);

	return srpmb_conn;
}

static void mmc_srpmb_release_connection(struct sock_desc *srpmb_conn)
{
	tz_iwsock_release(srpmb_conn);
}

static int mmc_srpmb_wait_request(struct sock_desc *srpmb_conn, struct device *dev)
{
	int ret;
	ssize_t len;
	unsigned int sock_data;

	len = tz_iwsock_read(srpmb_conn, &sock_data, sizeof(sock_data), 0);
	if (len > 0 && len != sizeof(sock_data)) {
		dev_err(dev, "failed to receive request, invalid len = %zd\n", len);
		return -EMSGSIZE;
	} else if (!len) {
		dev_err(dev, "connection was reset by peer\n");
		return -ECONNRESET;
	} else if (len < 0) {
		ret = len;
		dev_err(dev, "error while receiving request from SWd, err = %u\n", ret);
		return ret;
	}
	dev_info(dev, "Read request\n");

	if (sock_data != RPMB_REQUEST_MAGIC) {
		dev_err(dev, "received invalid request, data = %u\n", sock_data);
		return -EINVAL;
	}
	return 0;
}

static int mmc_srpmb_send_reply(struct sock_desc *srpmb_conn, struct device *dev)
{
	int ret = 0;
	ssize_t len;
	unsigned int sock_data;

	sock_data = RPMB_REPLY_MAGIC;
	len = tz_iwsock_write(srpmb_conn, &sock_data, sizeof(sock_data), 0);
	if (len != sizeof(sock_data)) {
		ret = len >= 0 ? -EMSGSIZE : len;
		dev_err(dev, "failed to send reply, err = %d\n", ret);
	} else {
		dev_info(dev, "Sent reply\n");
	}

	return ret;
}

static int mmc_srpmb_handle_unsupported_ops(struct mmc_srpmb_ctx *ctx)
{
	dev_err(&ctx->pdev->dev, "Received unsupported request: %x\n", ctx->req->type);
	mmc_srpmb_update_status_flag(ctx, RPMB_INVALID_COMMAND);

	return -EINVAL;
}

static void release_mmc_srpmb_blkdev(struct mmc_srpmb_ctx *ctx)
{
	blkdev_put(ctx->bdev, FMODE_READ|FMODE_WRITE);
	ctx->bdev = NULL;
}

static int init_mmc_srpmb_blkdev(struct mmc_srpmb_ctx *ctx)
{
	int ret;
	struct device *dev = &ctx->pdev->dev;

	/* get block device for mmc rpmb */
	ctx->bdev = blkdev_get_by_path(MMC_BLOCK_NAME, FMODE_READ|FMODE_WRITE, NULL);
	if (!ctx->bdev) {
		dev_err(dev, "Fail to get block device\n");
		return -ENOENT;
	}
	dev_info(dev, "Found block device\n");

	if (!ctx->bdev->bd_disk) {
		dev_err(dev, "Fail to get gendisk\n");
		ret = -ENOENT;
		goto put_blkdev;
	}

	ctx->fops = ctx->bdev->bd_disk->fops;
	if (!ctx->fops->srpmb_access) {
		dev_err(dev, "No valid function pointer\n");
		ret = -ENOTTY;
		goto put_blkdev;
	}
	dev_info(dev, "Initialized block device fops\n");

	return 0;

put_blkdev:
	release_mmc_srpmb_blkdev(ctx);

	return ret;
}

static int mmc_srpmb_kthread(void *data)
{
	int ret;
	struct sock_desc *srpmb_conn;
	struct mmc_srpmb_ctx *ctx = (struct mmc_srpmb_ctx *)data;
	struct device *dev = &ctx->pdev->dev;

	srpmb_conn = mmc_srpmb_accept_swd_connection(dev);
	if (IS_ERR(srpmb_conn))
		return PTR_ERR(srpmb_conn);

	while (!kthread_should_stop()) {
		ret = mmc_srpmb_wait_request(srpmb_conn, dev);
		if (ret)
			goto out;

		/* There is no any way to wait for rpmb block device creation,
		 * so in case there is no block rpmb device yet we just notify
		 * secure OS about request completion but we doesn't change
		 * status in shared memory to enable secure OS to detect,
		 * transaction error */
		if (!ctx->bdev) {
			ret = init_mmc_srpmb_blkdev(ctx);
			if (ret) {
				dev_err(dev, "Fail to initialize blkdev\n");
				goto send_reply;
			}
		}

		switch(ctx->req->type) {
		case GET_WRITE_COUNTER:
			mmc_srpmb_get_write_counter(ctx);
			break;
		case WRITE_DATA:
			mmc_srpmb_write(ctx);
			break;
		case READ_DATA:
			mmc_srpmb_read(ctx);
			break;
		default:
			ret = mmc_srpmb_handle_unsupported_ops(ctx);
			goto out;
		}
send_reply:
		ret = mmc_srpmb_send_reply(srpmb_conn, dev);
		if (ret)
			goto out;
	}

out:
	if (ctx->bdev)
		release_mmc_srpmb_blkdev(ctx);

	mmc_srpmb_release_connection(srpmb_conn);

	return ret;
}

static struct mmc_srpmb_ctx *create_mmc_srpmb_ctx(struct platform_device *pdev)
{
	struct mmc_srpmb_ctx *ctx;
	struct device *dev = &pdev->dev;

	/* allocation for rpmb context */
	ctx = kzalloc(sizeof(struct mmc_srpmb_ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(dev, "Fail to alloc for mmc rpmb context\n");
		return ERR_PTR(-ENOMEM);
	}
	dev_info(dev, "Allocated mmc_rpmb context\n");

	ctx->pdev = pdev;

	spin_lock_init(&ctx->lock);
	wake_lock_init(&ctx->wakelock, WAKE_LOCK_SUSPEND, "srpmb");

	return ctx;
}

static void destroy_mmc_srpmb_ctx(struct mmc_srpmb_ctx *ctx)
{
	wake_lock_destroy(&ctx->wakelock);
	kfree(ctx);
}

static int init_mmc_srpmb_wsm(struct mmc_srpmb_ctx *ctx)
{
	struct device *dev = &ctx->pdev->dev;

	/* allocation for wsm(world shared memory) */
	ctx->wsm_virtaddr = dma_alloc_coherent(dev,
				sizeof(struct mmc_srpmb_req) + RPMB_BUF_MAX_SIZE,
				&ctx->wsm_phyaddr, GFP_KERNEL);
	if (!ctx->wsm_virtaddr || !ctx->wsm_phyaddr) {
		dev_err(dev, "Fail to alloc for srpmb wsm (world shared memory)\n");
		return -ENOMEM;
	}

	dev_info(dev, "srpmb dma addr: virt(%llx), phy(%llx)\n",
			(uint64_t)ctx->wsm_virtaddr, (uint64_t)ctx->wsm_phyaddr);

	ctx->req = (struct mmc_srpmb_req *)ctx->wsm_virtaddr;

	return 0;
}

static void release_mmc_srpmb_wsm(struct mmc_srpmb_ctx *ctx)
{
	struct device *dev = &ctx->pdev->dev;

	dma_free_coherent(dev, RPMB_BUF_MAX_SIZE, ctx->wsm_virtaddr, ctx->wsm_phyaddr);
}

static irqreturn_t mmc_srpmb_interrupt(int intr, void *arg)
{
	return IRQ_HANDLED;
}

static int init_mmc_srpmb_irq(struct mmc_srpmb_ctx *ctx)
{
	int ret;
	struct resource *res;
	struct irq_data *rpmb_irqd;
	struct device *dev = &ctx->pdev->dev;

	/* get mmc srpmb irq number */
	res = platform_get_resource(ctx->pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "Fail to get IRQ resource\n");
		return -ENOENT;
	}
	dev_info(dev, "Requested mmc_rpmb IRQ resource\n");

	ctx->irq = res->start;
	if (ctx->irq <= 0) {
		dev_err(dev, "Fail to get irq number for mmc rpmb\n");
		return -ENOENT;
	}

	/* Get irq_data from irq number */
	rpmb_irqd = irq_get_irq_data(ctx->irq);
	if (!rpmb_irqd) {
		dev_err(dev, "Fail to get irq_data from irq number\n");
		return -ENOENT;
	}
	dev_info(dev, "Requested IRQ data\n");

	/* Get hwirq from irq_data */
	ctx->hwirq = irqd_to_hwirq(rpmb_irqd);

	/* request irq for mmc rpmb handler */
	ret = request_irq(ctx->irq, mmc_srpmb_interrupt,
			IRQF_TRIGGER_RISING, ctx->pdev->name, ctx);
	if (ret) {
		dev_err(dev, "Fail to request irq handler for mmc srpmb, error=%d\n", ret);
		return ret;
	}
	dev_info(dev, "Requested IRQ %d\n", ctx->irq);

	return 0;
}

static void release_mmc_srpmb_irq(struct mmc_srpmb_ctx *ctx)
{
	free_irq(ctx->irq, ctx);
}

static int register_mmc_srpmb_resources(struct mmc_srpmb_ctx *ctx)
{
	int ret;
	struct device *dev = &ctx->pdev->dev;

	/* Smc call to transfer wsm address to secure world */
	ret = exynos_smc(SMC_SRPMB_WSM, ctx->wsm_phyaddr, ctx->hwirq, 0);
	if (ret)
		dev_err(dev, "Fail to smc call to registed mmc srpmb resources: error=%d\n", ret);

	dev_info(dev, "Registered WSM\n");

	return ret;
}

static int mmc_srpmb_probe(struct platform_device *pdev)
{
	int ret;
	struct mmc_srpmb_ctx *ctx;
	struct device *dev = &pdev->dev;

	BUILD_BUG_ON(sizeof(struct rpmb_packet) != RPMB_PACKET_SIZE);

	ctx = create_mmc_srpmb_ctx(pdev);
	if (IS_ERR(ctx)) {
		dev_err(dev, "Fail to alloc context: error=%ld\n", PTR_ERR(ctx));
		return PTR_ERR(ctx);
	}

	ret = init_mmc_srpmb_wsm(ctx);
	if (ret) {
		dev_err(dev, "Fail to initialize wsm\n");
		goto destroy_ctx;
	}

	ret = init_mmc_srpmb_irq(ctx);
	if (ret) {
		dev_err(dev, "Fail to initialize irq\n");
		goto release_wsm;
	}

	ret = register_mmc_srpmb_resources(ctx);
	if (ret) {
		dev_err(dev, "Fail to register resources\n");
		goto release_irq;
	}

	platform_set_drvdata(pdev, ctx);
	dev_info(dev, "Published  context\n");

	srpmb_kthread = kthread_run(mmc_srpmb_kthread, ctx, "mmc_srpmb_worker");
	if (IS_ERR(srpmb_kthread)) {
		/* Here we can't release IRQ ir WSM due to ATF
		 * doesn't support such option, so system can crash
		 * if we release IRQ or WSM in here without
		 * releasing it in ATF */
		return PTR_ERR(srpmb_kthread);
	}

	dev_info(dev, "Started kernel thread\n");
	dev_info(dev, "Probe done\n");

	return ret;

release_irq:
	release_mmc_srpmb_irq(ctx);
release_wsm:
	release_mmc_srpmb_wsm(ctx);
destroy_ctx:
	destroy_mmc_srpmb_ctx(ctx);

	return ret;
}

static int mmc_srpmb_remove(struct platform_device *pdev)
{
	struct mmc_srpmb_ctx *ctx = platform_get_drvdata(pdev);

	kthread_stop(srpmb_kthread);

	release_mmc_srpmb_irq(ctx);
	release_mmc_srpmb_wsm(ctx);
	destroy_mmc_srpmb_ctx(ctx);

	return 0;
}

#ifdef CONFIG_PM
static int mmc_srpmb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mmc_srpmb_ctx *ctx = platform_get_drvdata(pdev);

	mmc_srpmb_update_status_flag(ctx, RPMB_FAIL_SUSPEND_STATUS);

	return 0;
}

static int mmc_srpmb_resume(struct platform_device *pdev)
{
	struct mmc_srpmb_ctx *ctx = platform_get_drvdata(pdev);

	mmc_srpmb_update_status_flag(ctx, 0);

	return 0;
}
#else
#define mmc_srpmb_suspend NULL
#define mmc_srpmb_resume NULL
#endif

static const struct of_device_id of_match_table[] = {
	{ .compatible = MMC_SRPMB_DEVICE_PROPNAME },
	{ }
};

static struct platform_driver mmc_srpmb_plat_driver = {
	.probe = mmc_srpmb_probe,
	.driver = {
		.name = "exynos-mmc-srpmb",
		.owner = THIS_MODULE,
		.of_match_table = of_match_table,
	},
	.suspend = mmc_srpmb_suspend,
	.resume = mmc_srpmb_resume,
	.remove = mmc_srpmb_remove,
};

static int __init mmc_srpmb_init(void)
{
	return platform_driver_register(&mmc_srpmb_plat_driver);
}

static void __exit mmc_srpmb_exit(void)
{
	platform_driver_unregister(&mmc_srpmb_plat_driver);
}

module_init(mmc_srpmb_init);
module_exit(mmc_srpmb_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MMC SRPMB driver");
