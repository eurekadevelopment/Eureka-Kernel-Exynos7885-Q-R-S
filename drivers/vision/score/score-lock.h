/*------------------------------------------------------------------------------
 * @file  score_lock.h
 * @ingroup  score_ipc_queue
 *
 * @brief  Header of sw lock by bakery algorithm
 * @author  Nahyun Kim<nh221.kim@samsung.com>
 *
 * @section changelog Change Log
 * 2016/03/04 Nahyun Kim created
 *
 * @section copyright_section Copyright
 * &copy; 2016, Samsung Electronics Co., Ltd.
 *------------------------------------------------------------------------------
 */

#ifndef _SCORE_LOCK_H
#define _SCORE_LOCK_H

//==============================================================//
// DEFINE
//==============================================================//
// linux device driver side
#define CONFIG_BAKERY_LOCK
#define CONFIG_BAKERY_UNLOCK

// SCore side
#define BAKERY_LOCK
#define BAKERY_UNLOCK

// The number of heterogeneous devices using lock
#define BAKERYNUM (3)
/// @enum bakery_id
///
/// devices is numbered 0,1,2...,BAKERYNUM
enum bakery_id{
    SCORE_ID = 0x0,
    CPU_ID = 0X1,
    IVA_ID = 0X2
};

//==============================================================//
// Function Prototype
//==============================================================//

/// @brief  Lock by bakery algorithm.
/// @param  id Device id like SCORE, CPU or IVA.
void score_bakery_lock(volatile void __iomem *addr, int id);

/// @brief  Unlock by bakery algorithm.
/// @param  id Device id like SCORE, CPU or IVA.
void score_bakery_unlock(volatile void __iomem *addr, int id);

#ifdef CONFIG_BAKERY_LOCK
//#define SCORE_BAKEddRY_LOCK(id) score_bakery_lock(id)
#define SCORE_BAKERY_LOCK(addr, id) score_bakery_lock(addr, id)
#else
#define SCORE_BAKERY_LOCK(id) NULL
#endif

#ifdef CONFIG_BAKERY_UNLOCK
#define SCORE_BAKERY_UNLOCK(addr, id) score_bakery_unlock(addr, id)
#else
#define SCORE_BAKERY_UNLOCK(id) NULL
#endif

#ifdef BAKERY_LOCK
#define SCORE_BAKERY_LOCK(addr, id) score_bakery_lock(addr, id)
#else
#define SCORE_BAKERY_LOCK(id) NULL
#endif

#ifdef BAKERY_UNLOCK
#define SCORE_BAKERY_UNLOCK(addr, id) score_bakery_unlock(addr, id)
#else
#define SCORE_BAKERY_UNLOCK(id) NULL
#endif

#endif
