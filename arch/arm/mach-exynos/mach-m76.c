/*
 * MEIZU M76 Flattened Device Tree enabled machine
 *
 * Copyright (c) 2014 MEIZU Technology Co., Ltd.
 *		http://www.meizu.com
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
#include <linux/power_supply.h>

#include <asm/mach/arch.h>

#include <plat/cpu.h>
#include <plat/mfc.h>
#include <plat/regs-watchdog.h>

#include <mach/regs-pmu.h>
#include <mach/resetreason.h>

#include "common.h"


#define REBOOT_MODE_CHARGE          0x0
#define REBOOT_MODE_WIPE            0x1
#define REBOOT_MODE_UPGRADE         0x2
/* Auto enter uboot fastboot mode */
#define REBOOT_MODE_FASTBOOT        0xFC
/* Auto enter uboot command line */
#define REBOOT_MODE_UBOOT           0xFE
/* custom reboot msg should be custom_xx */
#define REBOOT_MODE_CUSTOM          0xFF

#define REBOOT_MODE_CUSTOM_X(mode)  ((mode) << 8 | REBOOT_MODE_CUSTOM)

#define CUSTOM_UPDATE_AND_WIPE      REBOOT_MODE_CUSTOM_X(0x01)
#define CUSTOM_WIPE_SDCARD          REBOOT_MODE_CUSTOM_X(0x02)
#define CUSTOM_WIPE_USERDATA        REBOOT_MODE_CUSTOM_X(0x03)
#define CUSTOM_WIPE_ALL             REBOOT_MODE_CUSTOM_X(0x04)
#define CUSTOM_UPDATE_LOCATE        REBOOT_MODE_CUSTOM_X(0x05)
#define CUSTOM_ENTER_RECOVERY       REBOOT_MODE_CUSTOM_X(0x08)

static void m76_restart(char mode, const char *cmd)
{

	/* Record the normal reboot reason */
	pr_info("%s: command: %s\n", __func__, cmd);

        record_normal_reboot_reason(cmd);

	if (cmd) {
		if (!strcmp(cmd, "charge")) {
			__raw_writel(REBOOT_MODE_CHARGE, EXYNOS_INFORM4);
		} else if (!strcmp(cmd, "upgrade")) {
			__raw_writel(REBOOT_MODE_UPGRADE, EXYNOS_INFORM4);
		} else if (!strcmp(cmd, "fastboot") || !strcmp(cmd, "fb")) {
			__raw_writel(REBOOT_MODE_FASTBOOT, EXYNOS_INFORM4);
		} else if (!strcmp(cmd, "bootloader") || !strcmp(cmd, "bl")) {
			__raw_writel(REBOOT_MODE_UBOOT, EXYNOS_INFORM4);
		} else if (!strcmp(cmd, "update_and_wipe")) {
			__raw_writel(CUSTOM_UPDATE_AND_WIPE, EXYNOS_INFORM4);
		} else if (!strcmp(cmd, "wipe_sdcard")) {
			__raw_writel(CUSTOM_WIPE_SDCARD, EXYNOS_INFORM4);
		} else if (!strcmp(cmd, "wipe_all")) {
			__raw_writel(CUSTOM_WIPE_ALL, EXYNOS_INFORM4);
		} else if (!strcmp(cmd, "update_locate")) {
			__raw_writel(CUSTOM_UPDATE_LOCATE, EXYNOS_INFORM4);
		} else if (!strcmp(cmd, "recovery")) {
			__raw_writel(CUSTOM_ENTER_RECOVERY, EXYNOS_INFORM4);
		}
	}

	__raw_writel(0x1, EXYNOS_SWRESET);
}

static int m76_charger_online(void)
{
	struct power_supply *charger;
	int charger_online = 0;

	/* get charger power supply */
	charger = power_supply_get_by_name("charger");
	if (!charger) {
		pr_err("%s: Charger not found!\n", __func__);
		return charger_online;
	}

	/* get charger's online property */
	if (charger->get_property) {
		charger->get_property(charger, POWER_SUPPLY_PROP_ONLINE,
				(union power_supply_propval *)&charger_online);
	}

	return charger_online;
}

static void m76_power_off(void)
{
	unsigned int reg=0;
	printk("power off the device....\n");

	if (m76_charger_online()) {
		m76_restart(0, "charge");
	} else {
		reg = readl(EXYNOS5430_PS_HOLD_CONTROL);
		reg &= ~(0x1<<8);
		writel(reg, EXYNOS5430_PS_HOLD_CONTROL);
	}
}

static void m76_power_off_prepare(void)
{
	printk("power off prepare the deivce....\n");
}

static void __init m76_dt_map_io(void)
{
	exynos_init_io(NULL, 0);
}

static void __init m76_dt_machine_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static void __init m76_power_off_init(void)
{
	//register power off callback
	pm_power_off = m76_power_off;
	pm_power_off_prepare = m76_power_off_prepare;
}

static char const *m76_dt_compat[] __initdata = {
	"meizu,m76",
	NULL
};

static void __init m76_machine_init(void)
{
	m76_dt_machine_init();
	m76_power_off_init();
}

static void __init m76_reserve(void)
{
	init_exynos_ion_contig_heap();
}

//DT_MACHINE_START(M76, "ESPRESSO5430")
DT_MACHINE_START(M76, "M76")
	.init_irq	= exynos5_init_irq,
	.smp		= smp_ops(exynos_smp_ops),
	.map_io		= m76_dt_map_io,
	.init_machine	= m76_machine_init,
	.init_late	= exynos_init_late,
	.init_time	= exynos_init_time,
	.dt_compat	= m76_dt_compat,
	.restart        = m76_restart,
	.reserve	= m76_reserve,
MACHINE_END
