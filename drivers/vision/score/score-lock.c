//------------------------------------------------------------------------------
/// @file  score_lock.c
/// @ingroup  score
///
/// @brief  SW lock by bakery algorithm.
///         It will be used to send command between devices.
/// @author  Nahyun Kim<nh221.kim@samsung.com>
///
/// @section changelog Change Log
/// 2016/03/04 Nahyun Kim created
///
/// @section copyright_section Copyright
/// &copy; 2016, Samsung Electronics Co., Ltd.
///
//------------------------------------------------------------------------------

#include "score-fw-common.h"
#include "score-lock.h"
#include <linux/io.h>

// The SFRw address used in the bakery algorithm
#define BAKERY_CHOOSING_SCORE (0x70e8)
#define BAKERY_CHOOSING_CPU (0x70ec)
#define BAKERY_CHOOSING_IVA (0x70f0)
#define BAKERY_NUM_SCORE (0x70f4)
#define BAKERY_NUM_CPU (0x70f8)
#define BAKERY_NUM_IVA (0x70fc)

int max_num = 0;

// each device have own variables which are choosing and num.
// choosing and num are writable by a device only and
// readable by all other devices.
unsigned int CHOOSING[3] = {BAKERY_CHOOSING_SCORE,
                            BAKERY_CHOOSING_CPU,
                            BAKERY_CHOOSING_IVA
                           };
unsigned int NUM[3] = {BAKERY_NUM_SCORE,
                       BAKERY_NUM_CPU,
                       BAKERY_NUM_IVA
                      };

extern struct score_fw_dev *score_fw_device;

/// @brief  Lock by bakery algorithm.
/// @param  id Device id like SCORE, CPU or IVA. it is defined in score_lock.h
void score_bakery_lock(volatile void __iomem *addr, int id)
{
	int i, j;
	//while num is being set to non-zero value, choosing is 1
	/* SCORE_WRITE_REG(1, CHOOSING[id]); */
	writel(1, addr + CHOOSING[id]);

	//sets num to one higher then
	//all the nums of other devices
	for (i = 0; i < BAKERYNUM; i++) {
		max_num = (SCORE_READ_REG(NUM[i]) > max_num) ?
		          SCORE_READ_REG(NUM[i]) : max_num;
	}

	/* SCORE_WRITE_REG(max_num + 1, NUM[id]); */
	writel(max_num + 1, addr + NUM[id]);
	/* SCORE_WRITE_REG(0, CHOOSING[id]); */
	writel(0, addr + CHOOSING[id]);

        for (j = 0; j < BAKERYNUM; j++) {
		if (j != id) {
			/* while (SCORE_READ_REG(CHOOSING[j])) { */
			while (readl(addr + CHOOSING[j])) {
				// device first busy waits until device j is not
				// in the middle of choosing a num
                        }

			// busy waits until process j's BAKERY_NUM is either zero
			// or greater than BAKERY_NUM[i]
/* while (SCORE_READ_REG(NUM[j]) != 0 && */
/* ((SCORE_READ_REG(NUM[j]) < SCORE_READ_REG(NUM[id])) || */
/* ((SCORE_READ_REG(NUM[j]) == SCORE_READ_REG(NUM[id])) && (j < id)))) { */
			while (readl(addr + NUM[j]) != 0 &&
					((readl(addr + NUM[j]) < readl(addr + NUM[id])) ||
					((readl(addr + NUM[j]) == readl(addr + NUM[id])) && (j < id)))) {
			}
		}
	}
}

/// @brief  Unlock by bakery algorithm.
/// @param  id Device id like SCORE, CPU or IVA. it is defined in score_lock.h
void score_bakery_unlock(volatile void __iomem *addr, int id)
{
        //upon leaving the critical section, device id zeroes num.
	/* SCORE_WRITE_REG(0, NUM[id]); */
	writel(0, addr + NUM[id]);
}
