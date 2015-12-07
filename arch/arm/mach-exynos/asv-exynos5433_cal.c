/* linux/arch/arm/mach-exynos/include/mach/asv-exynos5433_cal.c
*
* Copyright (c) 2014 Samsung Electronics Co., Ltd.
*              http://www.samsung.com/
*
* EXYNOS5433 - Adoptive Support Voltage Header file
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#ifdef CONFIG_SOC_EXYNOS5433
#include <mach/asv-exynos5_cal.h>
#include <mach/asv-exynos5433.h>
#else
#include "asv-exynos5_cal.h"
#include "asv-exynos5433.h"
#endif

static volatile u32 *g_uAbbBase[6] = {
	[SYSC_DVFS_EGL] = CHIPID_ABBG_BASE + 0x0780,
	[SYSC_DVFS_KFC] = CHIPID_ABBG_BASE + 0x0784,
	[SYSC_DVFS_MIF] = CHIPID_ABBG_BASE + 0x078C,
	[SYSC_DVFS_INT] = CHIPID_ABBG_BASE + 0x0788,
	[SYSC_DVFS_G3D] = CHIPID_ABBG_BASE + 0x0790,
	[SYSC_DVFS_CAM] = CHIPID_ABBG_BASE + 0x0788,
};

static const u32 g_uBaseAddrTable[SYSC_DVFS_NUM][5] = {
	[SYSC_DVFS_EGL] = { (const u32) CHIPID_ASV_TBL_BASE + 0x0000, 0, 4, 8, 12},
	[SYSC_DVFS_KFC] = { (const u32) CHIPID_ASV_TBL_BASE + 0x0000, 16, 20, 24, 28},
	[SYSC_DVFS_G3D] = { (const u32) CHIPID_ASV_TBL_BASE + 0x0004, 0, 4, 8, 12},
	[SYSC_DVFS_MIF] = { (const u32) CHIPID_ASV_TBL_BASE + 0x0004, 16, 20, 24, 28},
	[SYSC_DVFS_INT] = { (const u32) CHIPID_ASV_TBL_BASE + 0x0008, 0, 4, 8, 12},
	[SYSC_DVFS_CAM] = { (const u32) CHIPID_ASV_TBL_BASE + 0x0008, 16, 20, 24, 28},

};

static volatile u32 g_uEglIds;
static volatile bool g_bDynamicABB[SYSC_DVFS_NUM];
static volatile u32 g_uDynamicEmaEGL1;
static volatile u32 g_uDynamicEmaEGL2;
static volatile u32 g_uPopType;
static volatile u32 g_uGroupFuse;

#ifdef CONFIG_SOC_EXYNOS5433
u32 re_err(void)
{
	pr_err("ASV: CAL is working wrong. \n");
	return 0;
}
#endif

bool CHIPID_IsFusedSpeedGroup(void)
{
	return g_uGroupFuse;
}

u32 CHIPID_GetDramSize(void)
{
	return g_uPopType ? 2 : 3;
}

u32 CHIPID_GetAsvTableVersion(void)
{
	u32 ver = __raw_readl(CHIPID_ASV_TBL_BASE + 0x000C) & 0x7;

	return ver;
}

u32 CHIPID_GetFusedIdsEgl(void)
{
	return __raw_readl(CHIPID_ABB_TBL_BASE) & 0xff;
}

void DVFS_Initialze(void)
{
	g_uEglIds = CHIPID_GetFusedIdsEgl();
	g_uDynamicEmaEGL1 = CHIPID_GetFusedIdsEgl();
	g_uDynamicEmaEGL2 = 0;
	g_uPopType = GetBits(PKG_ID, 4, 3);
	g_uGroupFuse = GetBits(PKG_ID, 3, 0x1);

	/* if (CHIPID_GetAsvTableVersion() == 0) */
	g_bDynamicABB[SYSC_DVFS_EGL] = true;
	g_bDynamicABB[SYSC_DVFS_KFC] = true;
	g_bDynamicABB[SYSC_DVFS_G3D] = true;
	g_bDynamicABB[SYSC_DVFS_INT] = true;
	g_bDynamicABB[SYSC_DVFS_MIF] = true;
	g_bDynamicABB[SYSC_DVFS_CAM] = false;
}

s32 DVFS_GetMaxLevel(u32 id)
{
	s32 lvl = (id == SYSC_DVFS_EGL) ? SYSC_DVFS_L5 :
		(id == SYSC_DVFS_KFC) ? SYSC_DVFS_L5 :
		(id == SYSC_DVFS_G3D) ? SYSC_DVFS_L1 : SYSC_DVFS_L0;
	return lvl;
}

s32 DVFS_GetMinLevel(u32 eSel)
{
	s32 eLvl = 0;
	switch (eSel) {
	case SYSC_DVFS_EGL:
		eLvl = SYSC_DVFS_END_LVL_EGL;
		break;
	case SYSC_DVFS_KFC:
		eLvl = SYSC_DVFS_END_LVL_KFC;
		break;
	case SYSC_DVFS_G3D:
		eLvl = SYSC_DVFS_END_LVL_G3D;
		break;
	case SYSC_DVFS_MIF:
		eLvl = SYSC_DVFS_END_LVL_MIF;
		break;
	case SYSC_DVFS_INT:
		eLvl = SYSC_DVFS_END_LVL_INT;
		break;
	case SYSC_DVFS_CAM:
		eLvl = SYSC_DVFS_END_LVL_CAM;
		break;
	default:
		Assert(0);
	}
	return eLvl;
}

u32 DVFS_GetIdsGroup(void)
{
	u32 group;

	group =
		(g_uEglIds > 140) ? SYSC_ASV_MAX :
		(g_uEglIds > 135) ? SYSC_ASV_14 :
		(g_uEglIds > 120) ? SYSC_ASV_13 :
		(g_uEglIds > 106) ? SYSC_ASV_12 :
		(g_uEglIds > 93) ? SYSC_ASV_11 :
		(g_uEglIds > 81) ? SYSC_ASV_10 :
		(g_uEglIds > 70) ? SYSC_ASV_9 :
		(g_uEglIds > 60) ? SYSC_ASV_8 :
		(g_uEglIds > 51) ? SYSC_ASV_7 :
		(g_uEglIds > 43) ? SYSC_ASV_6 :
		(g_uEglIds > 36) ? SYSC_ASV_5 :
		(g_uEglIds > 30) ? SYSC_ASV_4 :
		(g_uEglIds > 25) ? SYSC_ASV_3 :
		(g_uEglIds > 20) ? SYSC_ASV_2 : SYSC_ASV_1;

	return group;
}

u32 DVFS_GetMatchSubGroup(u32 id, s32 eLvl)
{
	u32 subgrp = 0;	/*  version 0 */

	switch (id) {
	case SYSC_DVFS_EGL:
		subgrp = (eLvl <= SYSC_DVFS_L8) ? 0 :
			(eLvl <= SYSC_DVFS_L13) ? 1 : 2;
		break;
	case SYSC_DVFS_KFC:
		subgrp = (eLvl <= SYSC_DVFS_L8) ? 0 :
			(eLvl <= SYSC_DVFS_L13) ? 1 : 2;
		break;
	case SYSC_DVFS_G3D:
		subgrp = (eLvl <= SYSC_DVFS_L2) ? 0 :
			(eLvl <= SYSC_DVFS_L4) ? 1 : 2;
		break;
	case SYSC_DVFS_MIF:
		subgrp = (eLvl <= SYSC_DVFS_L2) ? 0 : /*  L0 1066 L1 933 L2:825 */
			(eLvl <= SYSC_DVFS_L5) ? 1 : 2; /* 633,543,413 // 275~ */
		break;
	case SYSC_DVFS_INT:
		subgrp = (eLvl <= SYSC_DVFS_L0) ? 0 : /*  L0_A, L0 */
			(eLvl <= SYSC_DVFS_L3) ? 1 : 2;
		break;
	case SYSC_DVFS_CAM:
		subgrp = (eLvl <= SYSC_DVFS_L0) ? 0 :
			(eLvl <= SYSC_DVFS_L2) ? 1 : 2;
		break;
	default:
		Assert(0);
	}

	return subgrp;
}


u32 DVFS_GetLockingVoltage(u32 id)
{
	u32 volt, lockvalue;
	lockvalue = GetBits(g_uBaseAddrTable[id][0], g_uBaseAddrTable[id][4], 0xf);

	if (lockvalue == 0)
		volt = 0;
	else if (id == SYSC_DVFS_EGL)
		volt = 85000 + lockvalue * 25000;
	else
		volt = 70000 + lockvalue * 25000;

	return volt;
}

u32 DVFS_GetAsvGroup(u32 id, s32 eLvl)
{
	u32 subgrp, uAsvGroup = 0;
	if (CHIPID_IsFusedSpeedGroup())	{
		subgrp = DVFS_GetMatchSubGroup(id, eLvl);
		uAsvGroup = GetBits(g_uBaseAddrTable[id][0], g_uBaseAddrTable[id][1 + subgrp], 0xf);
	} else {
		uAsvGroup = DVFS_GetIdsGroup();
	}

	Assert(uAsvGroup < MAX_ASV_GROUP);
	return uAsvGroup;
}

u32 DVFS_GetTypicalVoltage(u32 id, s32 eLvl)
{
	u32 uVolt, uLockVolt;
	u32 asvgrp;
	u32 minlvl = DVFS_GetMinLevel(id);
	const u32 *pTable;
	u32 idx;
	Assert(eLvl >= 0);

	if (eLvl >= minlvl)
		eLvl = minlvl;
	idx = eLvl;

	pTable = ((id == SYSC_DVFS_EGL) ? g_uVtableEgl_V0[idx] :
			(id == SYSC_DVFS_KFC) ? g_uVtableKfc_V0[idx] :
			(id == SYSC_DVFS_G3D) ? g_uVtableG3d_V0[idx] :
			(id == SYSC_DVFS_MIF) ? g_uVtableMif_V0[idx] :
			(id == SYSC_DVFS_INT) ? g_uVtableInt_V0[idx] :
			(id == SYSC_DVFS_CAM) ? g_uVtableCam_V0[idx] :
			NULL);
	Assert(pTable != NULL);
	if (pTable == NULL)
		return 0;

	asvgrp = DVFS_GetAsvGroup(id, eLvl);
	uVolt = pTable[asvgrp];
	uLockVolt = DVFS_GetLockingVoltage(id);

	if (uLockVolt > uVolt)
		uVolt = uLockVolt;

	return uVolt;
}

u32 DVFS_GetFreqMhz(u32 id, s32 eLvl)
{
	u32 freq = 0;
	switch (id) {
	case SYSC_DVFS_EGL:
		freq = g_uClkTableEgl[eLvl][0];
		break;
	case SYSC_DVFS_KFC:
		freq = g_uClkTableKfc[eLvl][0];
		break;
	case SYSC_DVFS_G3D:
		freq = g_uClkTableG3d[eLvl][0];
		break;
	case SYSC_DVFS_MIF:
		freq = g_uClkTableMif[eLvl][0];
		break;
	case SYSC_DVFS_INT:
		freq = g_uClkTableInt[eLvl][0];
		break;
	case SYSC_DVFS_CAM:
		freq = g_uClkTableCam[eLvl][0];
		break;
	default:
		Assert(0);
	}
	return freq;
}

u32 DVFS_GetABB(u32 id, s32 eLvl)
{
	/* u32 grp = DVFS_GetAsvGroup(id, eLvl); */
	u32 value = ABB_BYPASS;	/* BYPASS */
	return value;
}

bool DVFS_UseDynamicABB(u32 id)
{
	return g_bDynamicABB[id];
}

void DVFS_SetABB(u32 eSel, u32 eAbb)
{
	u32 bits;
	Assert(eSel < SYSC_DVFS_NUM);

	if (eAbb == ABB_BYPASS)	/* bypass */
		bits = 0;
	else
		bits = (1 << 31) | (1 << 7) | (eAbb & 0x1f);

	__raw_writel(bits, g_uAbbBase[eSel]);
}

bool DVFS_UseDynamicEMA(u32 id)
{
	if (id == SYSC_DVFS_EGL)
		return g_uDynamicEmaEGL1;
	else
		return false;
}

void DVFS_SetEMA(u32 id, u32 setvolt)
{
	if (id == SYSC_DVFS_EGL && g_uDynamicEmaEGL1) {
		u32 value = (setvolt <= 900000) ? g_uDynamicEmaEGL1 : 2;
		u32 tmp = __raw_readl(SYSREG_EGL_BASE + 0x330);
		tmp &= ~(7 << 20 | 7 << 8);
		tmp |= ~(value << 20 | value << 8);
		__raw_writel(tmp, SYSREG_EGL_BASE + 0x330);
	}
}

