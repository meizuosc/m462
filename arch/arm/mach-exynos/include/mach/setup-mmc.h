/*
 * MEIZU M76 WL MMC HELPER
 *
 * Copyright (c) 2014 MEIZU Technology Co., Ltd.
 *		http://www.meizu.com
 *
 * Author: QuDao <qudao@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __SETUP_MMC_H__
#define __SETUP_MMC_H__

#define WL_MMC_NUM (1)
#define FLASH_MMC_NUM (0)
#define ENABLE_WL_DW_MCI_MON
#define ENABLE_FLASH_DW_MCI_MON

#ifdef ENABLE_WL_DW_MCI_MON
extern struct dw_mci_mon_table exynos_dwmci_tp_mon1_tbl[];
#else
#define exynos_dwmci_tp_mon1_tbl (NULL)
#endif

#ifdef ENABLE_FLASH_DW_MCI_MON
extern struct dw_mci_mon_table exynos_dwmci_tp_mon0_tbl[];
#else
#define exynos_dwmci_tp_mon0_tbl (NULL)
#endif

typedef void (*notify_func)(struct platform_device *dev, int state);
int ext_cd_init_dwmci1(notify_func func);
int ext_cd_cleanup_dwmci1(notify_func func);
int exynos_dwmci1_get_bus_wd(u32 slot_id);

#endif
