/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_APPLE_DART_H
#define _LINUX_APPLE_DART_H

#include <linux/kconfig.h>

#if IS_BUILTIN(CONFIG_APPLE_DART)
void apple_dart_dump_apcie_serror(void);
#else
static inline void apple_dart_dump_apcie_serror(void)
{
}
#endif

#endif /* _LINUX_APPLE_DART_H */
