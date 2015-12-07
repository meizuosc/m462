/*
 * Cal header file for Exynos Generic power domain.
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

struct exynos5430_pd_state {
	void __iomem *reg;
	u8 bit_offset;
	struct clk *clock;
};

/* BLK_MFC clocks */
static struct exynos5430_pd_state iptop_mfc[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,		.bit_offset = 1, },
};
static struct exynos5430_pd_state aclktop_mfc[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,    	.bit_offset = 3, },
};

/* BLK_HEVC clocks */
static struct exynos5430_pd_state iptop_hevc[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,		.bit_offset = 3, },
};
static struct exynos5430_pd_state aclktop_hevc[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,    	.bit_offset = 5, },
};

/* BLK_GSCL clocks */
static struct exynos5430_pd_state iptop_gscl[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,		.bit_offset = 7, },
};
static struct exynos5430_pd_state aclktop_gscl[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,    	.bit_offset = 14, },
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,    	.bit_offset = 15, },
};

/* BLK_MSCL clocks */
static struct exynos5430_pd_state iptop_mscl[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,	 	.bit_offset = 10, },
};
static struct exynos5430_pd_state aclktop_mscl[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 19, },
};
static struct exynos5430_pd_state sclktop_mscl[] = {
	{ .reg = EXYNOS5430_ENABLE_SCLK_TOP_MSCL,       .bit_offset = 0, },
};

/* BLK_G2D clocks */
static struct exynos5430_pd_state iptop_g2d[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,		.bit_offset = 0, },
};
static struct exynos5430_pd_state aclktop_g2d[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,    	.bit_offset = 0, },
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,    	.bit_offset = 2, },
};

/* BLK_ISP clocks */
static struct exynos5430_pd_state iptop_isp[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,	 	.bit_offset = 4, },
};
static struct exynos5430_pd_state aclktop_isp[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 6, },
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 7, },
};

/* BLK_CAM0 clocks */
static struct exynos5430_pd_state iptop_cam0[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,	 	.bit_offset = 5, },
};
static struct exynos5430_pd_state aclktop_cam0[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 8, },
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 9, },
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 10, },
};

/* BLK_CAM1 clocks */
static struct exynos5430_pd_state iptop_cam1[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,	 	.bit_offset = 6, },
};
static struct exynos5430_pd_state aclktop_cam1[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 11, },
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 12, },
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 13, },
};
static struct exynos5430_pd_state sclktop_cam1[] = {
	{ .reg = EXYNOS5430_ENABLE_SCLK_TOP_CAM1,	.bit_offset = 0, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_TOP_CAM1,	.bit_offset = 1, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_TOP_CAM1,	.bit_offset = 2, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_TOP_CAM1,	.bit_offset = 4, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_TOP_CAM1,	.bit_offset = 5, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_TOP_CAM1,	.bit_offset = 6, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_TOP_CAM1,	.bit_offset = 7, },
};

/* BLK_G3D clocks */
static struct exynos5430_pd_state iptop_g3d[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_TOP,       	.bit_offset = 18, },
};
static struct exynos5430_pd_state aclktop_g3d[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_TOP,     	.bit_offset = 30, },
};

/* BLK_DISP clocks */
static struct exynos5430_pd_state ipmif_disp[] = {
	{ .reg = EXYNOS5430_ENABLE_IP_MIF3,      	.bit_offset = 1, },
	{ .reg = EXYNOS5430_ENABLE_IP_MIF3,      	.bit_offset = 5, },
	{ .reg = EXYNOS5430_ENABLE_IP_MIF3,      	.bit_offset = 6, },
	{ .reg = EXYNOS5430_ENABLE_IP_MIF3,      	.bit_offset = 7, },
	{ .reg = EXYNOS5430_ENABLE_IP_MIF3,      	.bit_offset = 8, },
	{ .reg = EXYNOS5430_ENABLE_IP_MIF3,      	.bit_offset = 9, },
};
static struct exynos5430_pd_state aclkmif_disp[] = {
	{ .reg = EXYNOS5430_ENABLE_ACLK_MIF3,   	.bit_offset = 1, },
};
static struct exynos5430_pd_state sclkmif_disp[] = {
	{ .reg = EXYNOS5430_ENABLE_SCLK_MIF,     	.bit_offset = 5, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_MIF,     	.bit_offset = 6, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_MIF,     	.bit_offset = 7, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_MIF,     	.bit_offset = 8, },
	{ .reg = EXYNOS5430_ENABLE_SCLK_MIF,     	.bit_offset = 9, },
};
