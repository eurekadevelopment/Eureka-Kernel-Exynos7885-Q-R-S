#ifndef _SCORE_DEBUG_PRINT_H_
#define _SCORE_DEBUG_PRINT_H_

#include "score-fw-common.h"

int score_printf_buf_init(struct score_fw_dev *dev, void *msg_kva, unsigned long msg_dva);
int score_printf_buf_release(struct score_fw_dev *dev);
void score_printf_work_flush(void);

#endif
