/*
 * SAMSUNG EXYNOS5430 Flattened Device Tree enabled machine
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/memblock.h>
#include <linux/io.h>
#include <linux/clocksource.h>
#include <linux/exynos_ion.h>
#include <linux/reboot.h>
#include <linux/bq24160_charger.h>

#include <asm/mach/arch.h>
#include <mach/regs-pmu.h>


#include <plat/cpu.h>
#include <plat/mfc.h>
#include <plat/regs-watchdog.h>

#include "common.h"

static void espresso5430_power_off(void)
{
	unsigned int reg=0;
	unsigned int chg_status=0;
	printk("power off the device....\n");

#ifdef CONFIG_CHARGER_BQ24160
	chg_status = bq24160_get_ext_charging_status();
#endif

	if( EXT_CHG_STAT_VALID_CHARGER_CONNECTED == chg_status){
		printk("A charger is detected.. \nTry WDT reset..");

		reg = readl(S3C2410_WTCON);
		reg &= ~(0x1<<5);
		writel(reg, S3C2410_WTCON);

		reg = readl(EXYNOS_AUTOMATIC_WDT_RESET_DISABLE);
		reg &= ~(0x1<<0);
		writel(reg, EXYNOS_AUTOMATIC_WDT_RESET_DISABLE);

		reg = readl(EXYNOS_MASK_WDT_RESET_REQUEST);
		reg &= ~(0x1<<0);
		writel(reg, EXYNOS_MASK_WDT_RESET_REQUEST);

		reg = readl(S3C2410_WTCNT);
		reg = (0x1);
		writel(reg, S3C2410_WTCNT);

		reg = readl(S3C2410_WTCON);
		reg |= (0x1<<0 | 0x1<<5 | 0x1<<15);
		writel(reg, S3C2410_WTCON);
	}else{
		reg = readl(EXYNOS5430_PS_HOLD_CONTROL);
		reg &= ~(0x1<<8);
		writel(reg, EXYNOS5430_PS_HOLD_CONTROL);
	}
}

static void espresso5430_power_off_prepare(void)
{
	printk("power off prepare the deivce....\n");
}

static void __init espresso5430_dt_map_io(void)
{
	exynos_init_io(NULL, 0);
}

static void __init espresso5430_dt_machine_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static void __init espresso5430_power_off_init(void)
{
	//register power off callback
	pm_power_off = espresso5430_power_off;
	pm_power_off_prepare = espresso5430_power_off_prepare;
}

static char const *espresso5430_dt_compat[] __initdata = {
	"samsung,exynos5430",
	NULL
};

static void __init espresso5430_machine_init(void)
{
	espresso5430_dt_machine_init();
	espresso5430_power_off_init();
}

static void __init espresso5430_reserve(void)
{
	init_exynos_ion_contig_heap();
}

DT_MACHINE_START(ESPRESSO5430, "ESPRESSO5430")
	.init_irq	= exynos5_init_irq,
	.smp		= smp_ops(exynos_smp_ops),
	.map_io		= espresso5430_dt_map_io,
	.init_machine	= espresso5430_machine_init,
	.init_late	= exynos_init_late,
	.init_time	= exynos_init_time,
	.dt_compat	= espresso5430_dt_compat,
	.restart        = exynos5_restart,
	.reserve	= espresso5430_reserve,
MACHINE_END
