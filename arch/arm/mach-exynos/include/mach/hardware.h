/* linux/arch/arm/mach-exynos/include/mach/hardware.h
 *
 * Copyright (c) 2014 MEIZU Technology Co., Ltd.
 *
 * Meizu Mobilephone Hardware information support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H __FILE__

u32 meizu_board_version(void);

/*fp supplier*/
#define FP_IDEX 	1
#define FP_GOODIX	2

extern void fp_get(int fp);
#endif /* __ASM_ARCH_HARDWARE_H */
