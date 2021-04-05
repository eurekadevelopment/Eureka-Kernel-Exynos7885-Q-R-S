#ifndef _SCORE_CMD_QUEUE_H
#define _SCORE_CMD_QUEUE_H

#include "score-fw-common.h"

/* queue sfr register count */
#define SCORE_QUEUE_REG_SIZE		(52)
/* queue informaition register count */
#define SCORE_INFO_QUEUE_REG_SIZE	(2)
/* in:out = 44:4 */
/* queue input register count */
#define SCORE_IN_QUEUE_REG_SIZE		(44)
/* queue output register count */
#define SCORE_OUT_QUEUE_REG_SIZE	(4)

#define SCORE_IN_QUEUE_HEAD_INFO	(SCORE_PARAM1)
#define SCORE_IN_QUEUE_TAIL_INFO	(SCORE_PARAM0)
#define SCORE_OUT_QUEUE_HEAD_INFO	(SCORE_PARAM47)
#define SCORE_OUT_QUEUE_TAIL_INFO	(SCORE_PARAM46)

#define SCORE_PARAM_START		(SCORE_PARAM0)
#define SCORE_IN_QUEUE_START		\
	((SCORE_IN_QUEUE_TAIL_INFO) + ((SCORE_INFO_QUEUE_REG_SIZE)*(SCORE_REG_SIZE)))

#define SCORE_OUT_QUEUE_START		\
	((SCORE_OUT_QUEUE_TAIL_INFO) + ((SCORE_INFO_QUEUE_REG_SIZE)*(SCORE_REG_SIZE)))

#define PTR_MASK(SIZE)			(((SIZE) << 1) - 1)
#define IDX_MASK(SIZE)			((SIZE) - 1)

#define GET_CMD_WORD_SIZE(WORD, BYTE) do { \
  (WORD) = (((BYTE) + 3 + sizeof(union score_packet_header)) >> 2); \
} while (0)

#define GET_MIRROR_STATE(X, SIZE)  (((X) >= (SIZE))? 1:0)
#define GET_QUEUE_IDX(X, SIZE) \
  (GET_MIRROR_STATE((X), (SIZE))? ((X) - (SIZE)):(X))
#define GET_QUEUE_MIRROR_IDX(X, SIZE) \
  (((X) < ((SIZE)<<1))? (X):((X) - ((SIZE)<<1)))

#define MAX_SCORE_KERNEL_NUM		(0xFFF)
#define MAX_SCORE_REQUEST		(0xFFF)
#define MAX_SCORE_ARG			(0xFF)
/*
* //param0-> kernel_num(31~20):task_id(19~8):argument_count(7~0)
*
*#define SCORE_KERNEL_NUM_SHIFT(data)	(((data)<<20)&~(0x000FFFFF))
*#define SCORE_REQUEST_SHIFT(data)	(((data)<<8)&~(0xFFF000FF))
*#define SCORE_ARG_SHIFT(data)		((data)&~(0xFFFFFF00))
*
*#define SCORE_KERNEL_NUM(data)		(((data)&~(0x000FFFFF))>>20)
*#define SCORE_REQUEST(data)		(((data)&~(0xFFF000FF))>>8)
*#define SCORE_ARG(data)			((data)&~(0xFFFFFF00))
*/

int score_fw_queue_is_empty(struct score_fw_queue *queue);

static inline unsigned int score_fw_queue_get_task_id(struct score_fw_queue *queue)
{
	return (unsigned int)atomic_read(&queue->task_id);
}

unsigned int score_fw_queue_get_inc_task_id(struct score_fw_queue *queue);

int score_fw_queue_init(struct score_fw_dev *dev);
void score_fw_queue_exit(struct score_fw_dev *dev);
void score_wake_up_wait_task(void);


#define SCORE_IN_QUEUE_PUT(head, data)	\
	score_fw_queue_put_direct(dev->in_queue, (head), (data))
#define SCORE_OUT_QUEUE_PUT(head, data)	\
	score_fw_queue_put_direct(dev->out_queue, (head), (data))

#define SCORE_IN_QUEUE_GET(tail, data)	\
	score_fw_queue_get_direct(dev->in_queue, (tail), (data))
#define SCORE_OUT_QUEUE_GET(tail, data)	\
	score_fw_queue_get_direct(dev->out_queue, (tail), (data))

int score_fw_queue_put(struct score_fw_queue *queue, struct score_ipc_packet *cmd);
int score_fw_queue_get(struct score_fw_queue *queue, struct score_ipc_packet *cmd);
int score_fw_queue_put_direct(struct score_fw_queue *queue,
					unsigned int head, unsigned int data);
int score_fw_queue_get_direct(struct score_fw_queue *queue,
					unsigned int tail, unsigned int *data);
void score_fw_queue_dump_packet_word(struct score_ipc_packet *cmd);
void score_fw_queue_dump_packet_word2(struct score_ipc_packet *cmd);
void score_fw_dump_regs(void __iomem *base_addr, u32 size);
#endif
