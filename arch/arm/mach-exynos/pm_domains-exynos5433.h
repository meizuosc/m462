/*
 * Exynos Generic power domain support.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Implementation of Exynos specific power domain control which is used in
 * conjunction with runtime-pm. Support for both device-tree and non-device-tree
 * based power domain support is included.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <plat/pm.h>
#include <mach/pm_domains.h>
#include <mach/devfreq.h>
#include "pm_domains-exynos5433-cal.h"

void __iomem *decon_vidcon0;
void __iomem *decontv_vidcon0;
void __iomem *sysreg_disp;

bool is_mfc_clk_en = false;
bool is_hevc_clk_en = false;
bool is_gscl_clk_en = false;
bool is_mscl_clk_en = false;
bool is_g2d_clk_en = false;
bool is_g3d_clk_en = false;
bool is_isp_clk_en = false;
bool is_cam0_clk_en = false;
bool is_cam1_clk_en = false;
bool is_disp_clk_en = false;

struct exynos5433_pd_clk {
	const char *name;
	unsigned int num_iptop;
	unsigned int num_aclktop;
	unsigned int num_sclktop;
	struct exynos5430_pd_state *iptop;
	struct exynos5430_pd_state *aclktop;
	struct exynos5430_pd_state *sclktop;
};

static struct exynos5433_pd_clk pd_clk_list[] = {
	{
		.name = "pd-maudio",
	}, {
		.name = "pd-mfc",
		.num_iptop = ARRAY_SIZE(iptop_mfc),
		.num_aclktop = ARRAY_SIZE(aclktop_mfc),
		.iptop = iptop_mfc,
		.aclktop = aclktop_mfc,
	}, {
		.name = "pd-hevc",
		.num_iptop = ARRAY_SIZE(iptop_hevc),
		.num_aclktop = ARRAY_SIZE(aclktop_hevc),
		.iptop = iptop_hevc,
		.aclktop = aclktop_hevc,
	}, {
		.name = "pd-gscl",
		.num_iptop = ARRAY_SIZE(iptop_gscl),
		.num_aclktop = ARRAY_SIZE(aclktop_gscl),
		.iptop = iptop_gscl,
		.aclktop = aclktop_gscl,
	}, {
		.name = "pd-mscl",
		.num_iptop = ARRAY_SIZE(iptop_mscl),
		.num_aclktop = ARRAY_SIZE(aclktop_mscl),
		.num_sclktop = ARRAY_SIZE(sclktop_mscl),
		.iptop = iptop_mscl,
		.aclktop = aclktop_mscl,
		.sclktop = sclktop_mscl,
	}, {
		.name = "pd-g2d",
		.num_iptop = ARRAY_SIZE(iptop_g2d),
		.num_aclktop = ARRAY_SIZE(aclktop_g2d),
		.iptop = iptop_g2d,
		.aclktop = aclktop_g2d,
	}, {
		.name = "pd-isp",
		.num_iptop = ARRAY_SIZE(iptop_isp),
		.num_aclktop = ARRAY_SIZE(aclktop_isp),
		.iptop = iptop_isp,
		.aclktop = aclktop_isp,
	}, {
		.name = "pd-cam0",
		.num_iptop = ARRAY_SIZE(iptop_cam0),
		.num_aclktop = ARRAY_SIZE(aclktop_cam0),
		.iptop = iptop_cam0,
		.aclktop = aclktop_cam0,
	}, {
		.name = "pd-cam1",
		.num_iptop = ARRAY_SIZE(iptop_cam1),
		.num_aclktop = ARRAY_SIZE(aclktop_cam1),
		.num_sclktop = ARRAY_SIZE(sclktop_cam1),
		.iptop = iptop_cam1,
		.aclktop = aclktop_cam1,
		.sclktop = sclktop_cam1,
	}, {
		.name = "pd-g3d",
		.num_iptop = ARRAY_SIZE(iptop_g3d),
		.num_aclktop = ARRAY_SIZE(aclktop_g3d),
		.iptop = iptop_g3d,
		.aclktop = aclktop_g3d,
	}, {
		.name = "pd-disp",
		.num_iptop = ARRAY_SIZE(ipmif_disp),
		.num_aclktop = ARRAY_SIZE(aclkmif_disp),
		.num_sclktop = ARRAY_SIZE(sclkmif_disp),
		.iptop = ipmif_disp,
		.aclktop = aclkmif_disp,
		.sclktop = sclkmif_disp,
	},
};

static struct sleep_save exynos_pd_maudio_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_SRC_SEL_AUD0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_AUD0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_AUD1),
};

static struct sleep_save exynos_pd_g3d_clk_save[] = {
	/* it causes sudden reset */
	/*SAVE_ITEM(EXYNOS5430_DIV_G3D),*/
	/* it causes system hang due to G3D_CLKOUT */
	/*SAVE_ITEM(EXYNOS5430_SRC_SEL_G3D),*/
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_G3D0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_G3D1),
};

static struct sleep_save exynos_pd_mfc_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_DIV_MFC0),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_MFC0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_MFC00),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_MFC01),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_MFC0_SECURE_SMMU_MFC),
};

static struct sleep_save exynos_pd_hevc_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_DIV_HEVC),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_HEVC),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_HEVC0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_HEVC1),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_HEVC_SECURE_SMMU_HEVC),
};

static struct sleep_save exynos_pd_gscl_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_SRC_SEL_GSCL),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_GSCL0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_GSCL1),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_GSCL_SECURE_SMMU_GSCL0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_GSCL_SECURE_SMMU_GSCL1),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_GSCL_SECURE_SMMU_GSCL2),
};

static struct sleep_save exynos_pd_disp_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_DIV_DISP),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_MIF3),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_DISP1),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_MIF4),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_MIF5),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_MIF6),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_DISP0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_DISP0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_DISP1),
	/* Because DISP_PLL_CON SFR is in DISP_BLK,
	 * it can lose its contents when blk power is down.
	 */
	SAVE_ITEM(EXYNOS5430_DISP_PLL_LOCK),
	SAVE_ITEM(EXYNOS5430_DISP_PLL_CON0),
	SAVE_ITEM(EXYNOS5430_DISP_PLL_CON1),
	SAVE_ITEM(EXYNOS5430_DISP_PLL_FREQ_DET),
};

static struct sleep_save exynos_pd_mscl_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_DIV_MSCL),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_MSCL0),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_TOP_MSCL),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_MSCL0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_MSCL1),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_MSCL_SECURE_SMMU_M2MSCALER0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_MSCL_SECURE_SMMU_M2MSCALER1),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_MSCL_SECURE_SMMU_JPEG),
};

static struct sleep_save exynos_pd_g2d_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_DIV_G2D),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_G2D),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_G2D0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_G2D1),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_G2D_SECURE_SMMU_G2D),
};

static struct sleep_save exynos_pd_isp_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_DIV_ISP),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_ISP),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_ISP0),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_ISP1),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_ISP2),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_ISP3),
};

static struct sleep_save exynos_pd_cam0_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_DIV_CAM01),
	SAVE_ITEM(EXYNOS5430_DIV_CAM02),
	SAVE_ITEM(EXYNOS5430_DIV_CAM03),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_CAM00),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_CAM01),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_CAM00),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_CAM01),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_CAM02),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_CAM03),
};

static struct sleep_save exynos_pd_cam1_clk_save[] = {
	SAVE_ITEM(EXYNOS5430_DIV_CAM10),
	SAVE_ITEM(EXYNOS5430_DIV_CAM11),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_CAM10),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_TOP_CAM1),
	SAVE_ITEM(EXYNOS5430_SRC_SEL_CAM11),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_CAM10),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_CAM11),
	SAVE_ITEM(EXYNOS5430_ENABLE_IP_CAM12),
};

