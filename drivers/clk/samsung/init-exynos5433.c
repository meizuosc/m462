/*
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Common Clock Framework support for Exynos5433 SoC.
 */

#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <mach/regs-clock-exynos5433.h>
#include <mach/regs-pmu.h>

#include "clk.h"
#include "clk-pll.h"

static void usb_init_clock(void)
{
	exynos_set_parent("mout_sclk_usbdrd30_user", "oscclk");

	exynos_set_parent("mout_phyclk_usbdrd30_udrd30_phyclock",
			"phyclk_usbdrd30_udrd30_phyclock_phy");
	exynos_set_parent("mout_phyclk_usbdrd30_udrd30_pipe_pclk",
			"phyclk_usbdrd30_udrd30_pipe_pclk_phy");
}

void crypto_init_clock(void)
{
	exynos_set_rate("dout_aclk_imem_sssx_266", 160 * 1000000);
	exynos_set_rate("dout_aclk_imem_200", 160 * 1000000);
}

static void pcie_init_clock(void)
{
	exynos_set_parent("mout_sclk_pcie_100", "mout_bus_pll_user");
	exynos_set_parent("dout_sclk_pcie_100", "mout_sclk_pcie_100");
	exynos_set_parent("sclk_pcie_100_fsys", "dout_sclk_pcie_100");
	exynos_set_parent("mout_sclk_pcie_100_user", "sclk_pcie_100_fsys");
	exynos_set_rate("dout_sclk_pcie_100", 100000000);
}

void __init exynos5433_clock_init(void)
{
	usb_init_clock();
	crypto_init_clock();
	pcie_init_clock();
}
