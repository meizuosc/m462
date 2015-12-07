/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mmc/dw_mmc.h>
#include <mach/setup-mmc.h>
#include <linux/mmc/core.h>

extern struct platform_device *pdev_wl_dwmci;

#ifdef ENABLE_WL_DW_MCI_MON
struct dw_mci_mon_table exynos_dwmci_tp_mon1_tbl[] = {
	/* Byte/s,      MIF clk,        CPU clk,		KFC clk,	CPU number,	KFC number, 	RW flags */
	{18000000,		825000,		2000000,	1500000,	1,		3,	MMC_DATA_READ | MMC_DATA_WRITE},
	{ 8000000,		633000,		0,		1500000,	1,		3,	MMC_DATA_READ | MMC_DATA_WRITE},
	{ 2000000,		633000,		0,		1500000,	0,		3,	MMC_DATA_READ | MMC_DATA_WRITE},
	{       0,		0,		0,		0,		0,		0,	MMC_DATA_READ | MMC_DATA_WRITE},
};
#endif

struct dw_mci_mon_table exynos_dwmci_tp_mon0_tbl[] = {
	/* Byte/s,      MIF clk,     CPU clk      KFC clk 	CPU number	KFC number	RW flags */
	{20000000,	825000,		1500000,	1500000,	1,	3,	MMC_DATA_READ | MMC_DATA_WRITE},
	{10000000,	825000,		1200000,	1500000,	1,	3,	MMC_DATA_WRITE},
	{ 5000000,	633000,		0,		1200000,	0,	3,      MMC_DATA_WRITE},
	{ 1000000,	413000,		0,		1000000,	0,	3,	MMC_DATA_WRITE},
	{       0,	0,		0,		0,		0,	0,	MMC_DATA_READ | MMC_DATA_WRITE},
};

int exynos_dwmci1_get_bus_wd(u32 slot_id)
{
	pr_info("%s() returns 4\n", __func__);
	return 4;
}

static notify_func dwmci1_notify_func;

int ext_cd_init_dwmci1(notify_func func)
{
	dwmci1_notify_func = func;
	return 0;
}

int ext_cd_cleanup_dwmci1(notify_func func)
{
	dwmci1_notify_func = NULL;
	return 0;
}

/*
 * call this when you need sd stack to recognize insertion or removal of card
 * that can't be told by SDHCI regs
 */
void mmc_force_presence_change(struct platform_device *pdev, int val)
{
	notify_func func = dwmci1_notify_func;

	if (pdev && (pdev == pdev_wl_dwmci) && func)
		func(pdev, val);
	else
		pr_warn("%s: called for device with no notifier\n", __func__);
}
