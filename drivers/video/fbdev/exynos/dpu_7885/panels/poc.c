#include <video/mipi_display.h>
#include "poc.h"
#include "dpui.h"
#include "../dsim.h"
#include "../decon.h"
#include "../decon_notify.h"
#include "../decon_board.h"

/* ONLY SUPPORT ERASE AND WRITE */

static u8 poc_wr_img[POC_IMG_SIZE] = { 0, };

const char * const poc_op_string[] = {
	[POC_OP_NONE] = "POC_OP_NONE",
	[POC_OP_ERASE] = "POC_OP_ERASE",
	[POC_OP_WRITE] = "POC_OP_WRITE",
	[POC_OP_CANCEL] = "POC_OP_CANCEL",
};

#define DSI_WRITE(cmd, size)		do {				\
	ret = dsim_write_hl_data(poc_dev, cmd, size);			\
	if (ret < 0) {							\
		dsim_err("%s: failed to write %s\n", __func__, #cmd);	\
		goto tx_fail;	\
	}	\
} while (0)

static int dsim_write_hl_data(struct panel_poc_device *poc_dev, const u8 *cmd, u32 cmdSize)
{
	struct panel_private *priv = &poc_dev->dsim->priv;
	int ret = 0;
	int retry = 2;

	if (!priv->lcdconnected)
		return ret;

try_write:
	if (cmdSize == 1)
		ret = dsim_write_data(poc_dev->dsim, MIPI_DSI_DCS_SHORT_WRITE, cmd[0], 0);
	else if (cmdSize == 2)
		ret = dsim_write_data(poc_dev->dsim, MIPI_DSI_DCS_SHORT_WRITE_PARAM, cmd[0], cmd[1]);
	else
		ret = dsim_write_data(poc_dev->dsim, MIPI_DSI_DCS_LONG_WRITE, (unsigned long)cmd, cmdSize);

	if (ret < 0) {
		if (--retry)
			goto try_write;
		else
			dsim_err("%s: fail. %02x, ret: %d\n", __func__, cmd[0], ret);
	}

	if (cmd[0] == 0xC0 || cmd[0] == 0xC1 || cmd[0] == 0xFE)
		dsim_wait_for_cmd_done(poc_dev->dsim);

	return ret;
}

static int do_poc_tx_cmd(struct panel_poc_device *poc_dev, u32 cmd)
{
	struct panel_poc_info *poc_info = &poc_dev->poc_info;
	int ret = -1;

	u8 ER_STT[] = { 0xC1, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x04 };
	u8 WR_STT[] = { 0xC1, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x01 };
	u8 WR_DAT[] = { 0xC1, 0x00 };

	u8 er_size[3] = { 0x20, 0x52, 0xD8 };

	int er_delay[3] = { 400, 800, 1000 }; //ms

	switch (cmd) {
	case POC_ERASE_ENTER_SEQ:
		DSI_WRITE(KEY1_ENABLE, ARRAY_SIZE(KEY1_ENABLE));
		DSI_WRITE(KEY2_ENABLE, ARRAY_SIZE(KEY2_ENABLE));
		DSI_WRITE(PRE_01, ARRAY_SIZE(PRE_01));
		DSI_WRITE(PRE_02_GP, ARRAY_SIZE(PRE_02_GP));
		DSI_WRITE(PRE_02, ARRAY_SIZE(PRE_02));
		DSI_WRITE(PGM_ENABLE, ARRAY_SIZE(PGM_ENABLE));
		DSI_WRITE(ER_CTRL_01, ARRAY_SIZE(ER_CTRL_01));
		DSI_WRITE(EXEC, ARRAY_SIZE(EXEC));
		udelay(20);
		DSI_WRITE(ER_CTRL_02, ARRAY_SIZE(ER_CTRL_02));
		DSI_WRITE(EXEC, ARRAY_SIZE(EXEC));
		msleep(30);
		break;

	case POC_ERASE_4K_SEQ:
	case POC_ERASE_32K_SEQ:
	case POC_ERASE_64K_SEQ:
		ER_STT[2] = er_size[cmd - POC_ERASE_4K_SEQ];
		ER_STT[3] = (poc_info->waddr & 0xFF0000) >> 16;
		ER_STT[4] = (poc_info->waddr & 0x00FF00) >> 8;
		ER_STT[5] = (poc_info->waddr & 0x0000FF);
		DSI_WRITE(ER_ENABLE, ARRAY_SIZE(ER_ENABLE));
		DSI_WRITE(EXEC, ARRAY_SIZE(EXEC));
		udelay(20);
		DSI_WRITE(ER_STT, ARRAY_SIZE(ER_STT));
		DSI_WRITE(EXEC, ARRAY_SIZE(EXEC));
		msleep(er_delay[cmd - POC_ERASE_4K_SEQ]);
		break;

	case POC_ERASE_EXIT_SEQ:
	case POC_WRITE_EXIT_SEQ:
		DSI_WRITE(PGM_DISABLE, ARRAY_SIZE(PGM_DISABLE));
		DSI_WRITE(KEY1_DISABLE, ARRAY_SIZE(KEY1_DISABLE));
		DSI_WRITE(KEY2_DISABLE, ARRAY_SIZE(KEY2_DISABLE));
		break;

	case POC_WRITE_ENTER_SEQ:
		DSI_WRITE(KEY1_ENABLE, ARRAY_SIZE(KEY1_ENABLE));
		DSI_WRITE(KEY2_ENABLE, ARRAY_SIZE(KEY2_ENABLE));
		DSI_WRITE(PRE_01, ARRAY_SIZE(PRE_01));
		DSI_WRITE(PRE_02_GP, ARRAY_SIZE(PRE_02_GP));
		DSI_WRITE(PRE_02, ARRAY_SIZE(PRE_02));
		DSI_WRITE(PGM_ENABLE, ARRAY_SIZE(PGM_ENABLE));
		DSI_WRITE(RW_CTRL_01, ARRAY_SIZE(RW_CTRL_01));
		DSI_WRITE(EXEC, ARRAY_SIZE(EXEC));
		udelay(20);
		DSI_WRITE(RW_CTRL_02, ARRAY_SIZE(RW_CTRL_02));
		DSI_WRITE(EXEC, ARRAY_SIZE(EXEC));
		msleep(30);
		break;

	case POC_WRITE_STT_SEQ:
		WR_STT[6] = (poc_info->waddr & 0xFF0000) >> 16;
		WR_STT[7] = (poc_info->waddr & 0x00FF00) >> 8;
		WR_STT[8] = (poc_info->waddr & 0x0000FF);
		DSI_WRITE(WR_ENABLE, ARRAY_SIZE(WR_ENABLE));
		DSI_WRITE(EXEC, ARRAY_SIZE(EXEC));
		udelay(20);
		DSI_WRITE(WR_STT, ARRAY_SIZE(WR_STT));
		break;

	case POC_WRITE_DAT_SEQ:
		WR_DAT[1] = *(poc_info->wdata);
		DSI_WRITE(WR_DAT, ARRAY_SIZE(WR_DAT));
		DSI_WRITE(EXEC, ARRAY_SIZE(EXEC));
		udelay(20);
		break;

	case POC_WRITE_END_SEQ:
		DSI_WRITE(WR_END, ARRAY_SIZE(WR_END));
		usleep_range(4000, 5000);
		break;
	}

tx_fail:
	return ret;
}


int poc_erase(struct panel_poc_device *poc_dev, int addr, int len)
{
	struct panel_poc_info *poc_info = &poc_dev->poc_info;
	int ret, sz_block = 0, erased_size = 0, erase_seq_index;
	struct decon_device *decon = get_decon_drvdata(0);

	poc_info->erase_trycount++;

	if (addr % POC_PAGE > 0) {
		pr_err("%s, failed to start erase. invalid addr\n", __func__);
		poc_info->erase_failcount++;
		return -EINVAL;
	}

	if (len < 0 || (addr + len > POC_FA7_TOTAL_SIZE)) {
		pr_err("%s, failed to start erase. range exceeded\n", __func__);
		poc_info->erase_failcount++;
		return -EINVAL;
	}
	len = ALIGN(len, SZ_4K);

	dsim_info("%s poc erase +++, 0x%x, %d\n", __func__, addr, len);

	decon->partial_force_disable = 1;

	flush_kthread_worker(&decon->up.worker);
	msleep(33);

	mutex_lock(poc_dev->lock);

	ret = do_poc_tx_cmd(poc_dev, POC_ERASE_ENTER_SEQ);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to poc-erase-enter-seq\n", __func__);
		goto out_poc_erase;
	}

	while (len > erased_size) {
		if (len >= erased_size + SZ_64K) {
			erase_seq_index = POC_ERASE_64K_SEQ;
			sz_block = SZ_64K;
		} else if (len >= erased_size + SZ_32K) {
			erase_seq_index = POC_ERASE_32K_SEQ;
			sz_block = SZ_32K;
		} else {
			erase_seq_index = POC_ERASE_4K_SEQ;
			sz_block = SZ_4K;
		}

		poc_info->waddr = addr + erased_size;
		ret = do_poc_tx_cmd(poc_dev, erase_seq_index);
		if (unlikely(ret < 0)) {
			pr_err("%s, failed to poc-erase-seq 0x%x\n", __func__, addr + erased_size);
			goto out_poc_erase;
		}
		pr_info("%s erased addr %06X, sz_block %06X\n",
				__func__, addr + erased_size, sz_block);

		if (atomic_read(&poc_dev->cancel)) {
			pr_err("%s, stopped by user at erase 0x%x\n", __func__, erased_size);
			goto cancel_poc_erase;
		}
		erased_size += sz_block;
	}

	ret = do_poc_tx_cmd(poc_dev, POC_ERASE_EXIT_SEQ);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to poc-erase-exit-seq\n", __func__);
		goto out_poc_erase;
	}

	decon->partial_force_disable = 0;
	mutex_unlock(poc_dev->lock);

	pr_info("%s poc erase ---\n", __func__);
	return 0;

cancel_poc_erase:
	ret = do_poc_tx_cmd(poc_dev, POC_ERASE_EXIT_SEQ);
	if (unlikely(ret < 0))
		pr_err("%s, failed to poc-erase-exit-seq\n", __func__);
	ret = -EIO;
	atomic_set(&poc_dev->cancel, 0);

out_poc_erase:
	decon->partial_force_disable = 0;
	poc_info->erase_failcount++;
	mutex_unlock(poc_dev->lock);
	return ret;
}

static int poc_write_data(struct panel_poc_device *poc_dev)
{
	struct panel_poc_info *poc_info = &poc_dev->poc_info;
	int i, ret = 0;
	int size = 0;
	struct decon_device *decon = get_decon_drvdata(0);

	decon->partial_force_disable = 1;

	flush_kthread_worker(&decon->up.worker);
	msleep(33);


	mutex_lock(poc_dev->lock);

	ret = do_poc_tx_cmd(poc_dev, POC_WRITE_ENTER_SEQ);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to read poc-wr-enter-seq\n", __func__);
		goto out_poc_write;
	}

	poc_info->waddr = POC_IMG_ADDR + poc_info->wpos;
	size = poc_info->wsize;

	for (i = 0; i < size; i++) {
		if (atomic_read(&poc_dev->cancel)) {
			pr_err("%s, stopped by user at %d bytes\n",
					__func__, i);
			goto cancel_poc_write;
		}
		poc_info->waddr = POC_IMG_ADDR + poc_info->wpos + i; //target addr
		poc_info->wdata = poc_info->wbuf + poc_info->wpos + i; //data pointer

//		pr_info("%s %x %x\n", __func__, poc_info->waddr , *(poc_info->wdata));
		if (i == 0 || ((poc_info->waddr & 0xFF) == 0)) {
			ret = do_poc_tx_cmd(poc_dev, POC_WRITE_STT_SEQ);
			if (unlikely(ret < 0)) {
				pr_err("%s, failed to write poc-wr-stt seq\n", __func__);
				goto out_poc_write;
			}
		}
		ret = do_poc_tx_cmd(poc_dev, POC_WRITE_DAT_SEQ);
		if (unlikely(ret < 0)) {
			pr_err("%s, failed to write poc-wr-img seq\n", __func__);
			goto out_poc_write;
		}

		if ((i % 4096) == 0)
			pr_info("%s addr: %06X data: 0x%02X\n", __func__, poc_info->waddr, *poc_info->wdata);

		if (((poc_info->waddr & 0xFF) == 0xFF)  || (i == (size - 1))) {
			ret = do_poc_tx_cmd(poc_dev, POC_WRITE_END_SEQ);
			if (unlikely(ret < 0)) {
				pr_err("%s, failed to write poc-wr-exit seq\n", __func__);
				goto out_poc_write;
			}
		}

	}

	ret = do_poc_tx_cmd(poc_dev, POC_WRITE_EXIT_SEQ);
	if (unlikely(ret < 0)) {
		pr_err("%s, failed to write poc-wr-exit seq\n", __func__);
		goto out_poc_write;
	}

	pr_info("%s poc write addr 0x%06X, %d(0x%X) bytes\n",
			__func__, POC_IMG_ADDR + poc_info->wpos, size, size);
	mutex_unlock(poc_dev->lock);

	decon->partial_force_disable = 0;

	return 0;

cancel_poc_write:
	ret = do_poc_tx_cmd(poc_dev, POC_WRITE_EXIT_SEQ);
	if (unlikely(ret < 0))
		pr_err("%s, failed to read poc-wr-exit seq\n", __func__);
	ret = -EIO;
	atomic_set(&poc_dev->cancel, 0);

out_poc_write:
	mutex_unlock(poc_dev->lock);
	decon->partial_force_disable = 0;

	return ret;
}

static long panel_poc_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	return 0;
}

static int panel_poc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *dev = file->private_data;
	struct panel_poc_device *poc_dev = container_of(dev, struct panel_poc_device, dev);
	struct panel_poc_info *poc_info = &poc_dev->poc_info;

	dsim_info("%s was called\n", __func__);

	if (poc_dev->opened) {
		dsim_err("POC:ERR:%s: already opend\n", __func__);
		return -EBUSY;
	}

	if (!poc_dev->enable) {
		dsim_err("POC:WARN:%s:panel is not active %d\n", __func__, poc_dev->enable);
		return -EAGAIN;
	}

	poc_info->state = 0;

	poc_info->wbuf = poc_wr_img;
	poc_info->wpos = 0;
	poc_info->wsize = 0;

	file->private_data = poc_dev;
	poc_dev->opened = 1;

	poc_dev->read = 0;

	atomic_set(&poc_dev->cancel, 0);

	return 0;
}

static int panel_poc_release(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct panel_poc_device *poc_dev = file->private_data;
	struct panel_poc_info *poc_info = &poc_dev->poc_info;

	dsim_info("%s was called\n", __func__);

	poc_info->state = 0;

	poc_info->wbuf = NULL;
	poc_info->wpos = 0;
	poc_info->wsize = 0;

	poc_dev->opened = 0;
	poc_dev->read = 0;

	atomic_set(&poc_dev->cancel, 0);

	dsim_info("%s Erase_try: %d, Erase_fail: %d, Write_try: %d, Write_fail: %d\n",
		__func__,
		poc_info->erase_trycount, poc_info->erase_failcount,
		poc_info->write_trycount,  poc_info->write_failcount);

	return ret;
}

static ssize_t panel_poc_read(struct file *file, char __user *buf, size_t count,
		loff_t *ppos)
{
	/* DO nothing */

	return 0;
}

static ssize_t panel_poc_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct panel_poc_device *poc_dev = file->private_data;
	struct panel_poc_info *poc_info = &poc_dev->poc_info;
	ssize_t res;

	dsim_info("%s : size : %d, ppos %d\n", __func__, (int)count, (int)*ppos);
	poc_info->write_trycount++;

	if (unlikely(!poc_dev->opened)) {
		dsim_err("POC:ERR:%s: poc device not opened\n", __func__);
		poc_info->write_failcount++;
		return -EIO;
	}

	if (!poc_dev->enable) {
		dsim_err("POC:WARN:%s:panel is not active\n", __func__);
		poc_info->write_failcount++;
		return -EAGAIN;
	}

	if (unlikely(!buf)) {
		dsim_err("POC:ERR:%s: invalid write buffer\n", __func__);
		poc_info->write_failcount++;
		return -EINVAL;
	}

	if (unlikely(*ppos + count > POC_IMG_SIZE)) {
		dsim_err("POC:ERR:%s: invalid write size pos %d, size %d\n",
				__func__, (int)*ppos, (int)count);
		poc_info->write_failcount++;
		return -EINVAL;
	}

	poc_info->wbuf = poc_wr_img;
	poc_info->wpos = *ppos;
	poc_info->wsize = count;
	res = simple_write_to_buffer(poc_info->wbuf, POC_IMG_SIZE, ppos, buf, count);

	dsim_info("%s write %ld bytes (count %ld)\n", __func__, res, count);

	res = poc_write_data(poc_dev);
	if (res < 0) {
		poc_info->write_failcount++;
		goto err_write;
	}

	return count;

err_write:
	poc_info->write_failcount++;
	return res;
}

static const struct file_operations panel_poc_fops = {
	.owner = THIS_MODULE,
	.read = panel_poc_read,
	.write = panel_poc_write,
	.unlocked_ioctl = panel_poc_ioctl,
	.open = panel_poc_open,
	.release = panel_poc_release,
};


#ifdef CONFIG_DISPLAY_USE_INFO
static int poc_dpui_callback(struct panel_poc_device *poc_dev)
{
	struct panel_poc_info *poc_info;

	poc_info = &poc_dev->poc_info;

	inc_dpui_u32_field(DPUI_KEY_PNPOC_ER_TRY, poc_info->erase_trycount);
	poc_info->erase_trycount = 0;
	inc_dpui_u32_field(DPUI_KEY_PNPOC_ER_FAIL, poc_info->erase_failcount);
	poc_info->erase_failcount = 0;

	inc_dpui_u32_field(DPUI_KEY_PNPOC_WR_TRY, poc_info->write_trycount);
	poc_info->write_trycount = 0;
	inc_dpui_u32_field(DPUI_KEY_PNPOC_WR_FAIL, poc_info->write_failcount);
	poc_info->write_failcount = 0;

	inc_dpui_u32_field(DPUI_KEY_PNPOC_RD_TRY, poc_info->read_trycount);
	poc_info->read_trycount = 0;
	inc_dpui_u32_field(DPUI_KEY_PNPOC_RD_FAIL, poc_info->read_failcount);
	poc_info->read_failcount = 0;

	return 0;
}

static int poc_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct panel_poc_device *poc_dev;
	struct dpui_info *dpui = data;

	if (dpui == NULL) {
		dsim_err("%s: dpui is null\n", __func__);
		return 0;
	}

	poc_dev = container_of(self, struct panel_poc_device, poc_notif);
	poc_dpui_callback(poc_dev);

	return 0;
}
#endif /* CONFIG_DISPLAY_USE_INFO */

static int poc_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct panel_poc_device *poc_dev;
	struct fb_event *evdata = data;
	int fb_blank;

	switch (event) {
	case FB_EVENT_BLANK:
	case FB_EARLY_EVENT_BLANK:
		break;
	default:
		return NOTIFY_DONE;
	}

	poc_dev = container_of(self, struct panel_poc_device, fb_notif);

	fb_blank = *(int *)evdata->data;

	if (evdata->info->node)
		return NOTIFY_DONE;

	if (event == FB_EVENT_BLANK && fb_blank == FB_BLANK_UNBLANK) {
		mutex_lock(poc_dev->lock);
		poc_dev->enable = 1;
		mutex_unlock(poc_dev->lock);
	} else if (fb_blank == FB_BLANK_POWERDOWN) {
		mutex_lock(poc_dev->lock);
		poc_dev->enable = 0;
		mutex_unlock(poc_dev->lock);
	}

	return NOTIFY_DONE;
}

static int poc_register_fb(struct panel_poc_device *poc_dev)
{
	poc_dev->fb_notif.notifier_call = poc_fb_notifier_callback;
	return decon_register_notifier(&poc_dev->fb_notif);
}

int panel_poc_probe(struct panel_poc_device *poc_dev)
{
	struct panel_poc_info *poc_info;
	int ret = 0;

	if (poc_dev == NULL) {
		dsim_err("POC:ERR:%s: invalid poc_dev\n", __func__);
		return -EINVAL;
	}

	poc_info = &poc_dev->poc_info;
	poc_dev->dev.minor = MISC_DYNAMIC_MINOR;
	poc_dev->dev.name = "poc";
	poc_dev->dev.fops = &panel_poc_fops;
	poc_dev->dev.parent = NULL;

	ret = misc_register(&poc_dev->dev);
	if (ret < 0) {
		dsim_err("PANEL:ERR:%s: failed to register panel misc driver (ret %d)\n",
				__func__, ret);
		goto exit_probe;
	}

	poc_info->erased = false;
	poc_info->poc = 1;	/* default enabled */

	poc_dev->opened = 0;
	poc_dev->enable = 1;

	poc_register_fb(poc_dev);

#ifdef CONFIG_DISPLAY_USE_INFO
	poc_info->total_trycount = -1;
	poc_info->total_failcount = -1;

	poc_dev->poc_notif.notifier_call = poc_notifier_callback;
	ret = dpui_logging_register(&poc_dev->poc_notif, DPUI_TYPE_PANEL);
	if (ret < 0) {
		dsim_err("ERR:PANEL:%s:failed to register dpui notifier callback\n", __func__);
		goto exit_probe;
	}
#endif

exit_probe:
	return ret;
}
