/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_J700_MT7932_DEBUG_H
#define _LINUX_J700_MT7932_DEBUG_H

#include <linux/jiffies.h>
#include <linux/kconfig.h>

#if IS_ENABLED(CONFIG_J700_MT7932_DEBUG_UART)
bool j700_mt7932_debug_uart_wait_ready(unsigned long timeout);
#else
static inline bool j700_mt7932_debug_uart_wait_ready(unsigned long timeout)
{
	return true;
}
#endif

#endif
