/*
 * Copyright (C) 2017 Samsung Electronics Co.Ltd
 * http://www.samsung.com
 *
 * EXYNOS MODEM CONTROL driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __EXYNOS_MODEM_CTRL_H
#define __EXYNOS_MODEM_CTRL_H

#define MODEM_CTRL_UART_AP 0
#define MODEM_CTRL_UART_CP 1

extern void send_panic_noti_modemif_ext(void);
extern void send_uart_noti_to_modem(int val);

#endif
