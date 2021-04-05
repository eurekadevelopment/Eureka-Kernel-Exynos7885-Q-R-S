#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include "score-debug-print.h"
#include "score-fw-queue.h"
//#include "score-device.h"
//#include "score-system.h"

#define MAX_PRINT_BUF_SIZE	(1024)
#define SCORE_LOG_BUF_SIZE	SZ_1M

extern struct score_fw_dev *score_fw_device;

struct score_printf_buf_t {
	int front;
	int rear;
	volatile unsigned char* buf_pa;
	volatile unsigned char* buf_va;
};

dma_addr_t score_printf_addr;

volatile struct score_printf_buf_t *score_printf_buf = NULL;
static int score_printf_buf_size = 0;

static struct workqueue_struct *score_printf_wq = NULL;
static struct work_struct score_printf_work;

static inline int score_printf_buf_is_full(void)
{
	return ((score_printf_buf->rear + 1) % score_printf_buf_size)
		== score_printf_buf->front;
}

static inline int score_printf_buf_is_empty(void)
{
	return (score_printf_buf->rear == score_printf_buf->front);
}

static char* score_strcpy(char *dest, const volatile unsigned char* src)
{
	char* tmp = dest;

	while ((*dest++ = *src++) != '\0') {;}

	return tmp;
}

static int score_printf_dequeue(char* buf)
{
	int front;
	volatile unsigned char *printf_buf;

	if (score_printf_buf == NULL) {
		return -1;
	}

	if (score_printf_buf_is_empty()) {
		return -1;
	}
	front = (score_printf_buf->front + 1) % score_printf_buf_size;
	printf_buf = (score_printf_buf->buf_va) + (MAX_PRINT_BUF_SIZE * front);

	score_strcpy(buf, (const unsigned char*)printf_buf);
	score_printf_buf->front = front;

	return 0;
}

static void score_printf_work_func(struct work_struct *work)
{
	char buf[MAX_PRINT_BUF_SIZE];

	while(1) {
		memset(buf, '\0', MAX_PRINT_BUF_SIZE);
		if (score_printf_dequeue(buf) < 0) {
			msleep(500);
		} else {
			printk("%s",buf);
			/* score_info("[S-FW] %s", buf); */
		}
	}
}

void score_printf_work_flush(void)
{
	char buf[MAX_PRINT_BUF_SIZE];

	memset(buf, '\0', MAX_PRINT_BUF_SIZE);
	if (score_printf_dequeue(buf) >= 0)
		printk("[S-FW] %s",buf);
}

static void score_printf_work_init(void)
{
	score_printf_wq = create_singlethread_workqueue("score_printf_work");
	if (!score_printf_wq) {
		printk("Failed to create workqueue for score printf\n");
		return;
	}
	INIT_WORK(&score_printf_work, score_printf_work_func);
	queue_work(score_printf_wq, &score_printf_work);
}

static void score_printf_init(unsigned char* start, int size)
{
	score_printf_buf = (struct score_printf_buf_t*)start;
	score_printf_buf->front = -1;
	score_printf_buf->rear = -1;
	score_printf_buf->buf_va = start + MAX_PRINT_BUF_SIZE;
	score_printf_buf_size =
		(size - sizeof(struct score_printf_buf_t)) / MAX_PRINT_BUF_SIZE;

	score_printf_work_init();
}

int score_printf_buf_init(struct score_fw_dev *dev, void *msg_kva, unsigned long msg_dva)
{
	unsigned char *score_printf_va;
	unsigned int score_printf_pa;

//	score_printf_va = dma_zalloc_coherent(&dev->pdev->dev, SCORE_LOG_BUF_SIZE,
//						&score_printf_addr, GFP_KERNEL);
//	if (score_printf_va == NULL) {
//		dev_err(&dev->pdev->dev, "Failed to allocate SCore_printf_buf memory\n");
//		return -ENOMEM;
//	}
//
//	score_printf_pa = virt_to_phys(score_printf_va);
//	memset(score_printf_va, 0x0, SCORE_LOG_BUF_SIZE);

	score_fw_device = dev;
	pr_info("%p, 0x%lx, %p %p \n", msg_kva, msg_dva, dev, score_fw_device);

	score_printf_va = (unsigned char*) msg_kva;
	score_printf_pa = (unsigned int) msg_dva;
	memset(score_printf_va, 0x0, SCORE_LOG_BUF_SIZE);

	score_write_reg(score_printf_pa, SCORE_PRINTF_BUF_START);
	score_write_reg(SCORE_LOG_BUF_SIZE, SCORE_PRINTF_BUF_SIZE);

	score_printf_init(score_printf_va, SCORE_LOG_BUF_SIZE);

	return 0;
}

int score_printf_buf_release(struct score_fw_dev *dev)
{
	if (score_printf_buf == NULL) {
		printk("SCore_printf_buf is invalid memory\n");
		return -EINVAL;
	}

//	dma_free_coherent(&dev->pdev->dev, SCORE_LOG_BUF_SIZE,
//			(unsigned char*)score_printf_buf, score_printf_addr);

	score_printf_buf = NULL;

	return 0;
}
