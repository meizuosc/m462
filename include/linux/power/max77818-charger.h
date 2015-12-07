/* linux/power/max77818-charger.h
 *
 * Copyright 2013 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MAX77818_CHARGER_H
#define __LINUX_MAX77818_CHARGER_H
#include <linux/power_supply.h>
#include <linux/notifier.h>
enum max77818_charger_fchgtime
{
	MAX77818_FCHGTIME_DISABLE,
	MAX77818_FCHGTIME_4H,
	MAX77818_FCHGTIME_6H,
	MAX77818_FCHGTIME_8H,
	MAX77818_FCHGTIME_10H,
	MAX77818_FCHGTIME_12H,
	MAX77818_FCHGTIME_14H,
	MAX77818_FCHGTIME_16H,
};

enum max77818_charger_chg_rstrt
{
	MAX77818_CHG_RSTRT_100MV,
	MAX77818_CHG_RSTRT_150MV,
	MAX77818_CHG_RSTRT_200MV,
	MAX77818_CHG_RSTRT_DISABLE,
};

enum max77818_charger_to_ith
{
	MAX77818_CHG_TO_ITH_100MA,
	MAX77818_CHG_TO_ITH_125MA,
	MAX77818_CHG_TO_ITH_150MA,
	MAX77818_CHG_TO_ITH_175MA,
	MAX77818_CHG_TO_ITH_200MA,
	MAX77818_CHG_TO_ITH_250MA,
	MAX77818_CHG_TO_ITH_300MA,
	MAX77818_CHG_TO_ITH_350MA,
};

enum max77818_charger_top_off_timer
{
	MAX77818_CHG_TO_TIME_0MIN,
	MAX77818_CHG_TO_TIME_10MIN,
	MAX77818_CHG_TO_TIME_20MIN,
	MAX77818_CHG_TO_TIME_30MIN,
	MAX77818_CHG_TO_TIME_40MIN,
	MAX77818_CHG_TO_TIME_50MIN,
	MAX77818_CHG_TO_TIME_60MIN,
	MAX77818_CHG_TO_TIME_70MIN,
};

enum max77818_charger_chg_cv_prm
{
	MAX77818_CHG_CV_PRM_3650MV,
	MAX77818_CHG_CV_PRM_3675MV,
	MAX77818_CHG_CV_PRM_3700MV,
	MAX77818_CHG_CV_PRM_3725MV,
	MAX77818_CHG_CV_PRM_3750MV,
	MAX77818_CHG_CV_PRM_3775MV,
	MAX77818_CHG_CV_PRM_3800MV,
	MAX77818_CHG_CV_PRM_3825MV,
	MAX77818_CHG_CV_PRM_3850MV,
	MAX77818_CHG_CV_PRM_3875MV,
	MAX77818_CHG_CV_PRM_3900MV,
	MAX77818_CHG_CV_PRM_3925MV,
	MAX77818_CHG_CV_PRM_3950MV,
	MAX77818_CHG_CV_PRM_3975MV,
	MAX77818_CHG_CV_PRM_4000MV,
	MAX77818_CHG_CV_PRM_4025MV,
	MAX77818_CHG_CV_PRM_4050MV,
	MAX77818_CHG_CV_PRM_4075MV,
	MAX77818_CHG_CV_PRM_4100MV,
	MAX77818_CHG_CV_PRM_4125MV,
	MAX77818_CHG_CV_PRM_4150MV,
	MAX77818_CHG_CV_PRM_4175MV,
	MAX77818_CHG_CV_PRM_4200MV,
	MAX77818_CHG_CV_PRM_4225MV,
	MAX77818_CHG_CV_PRM_4250MV,
	MAX77818_CHG_CV_PRM_4275MV,
	MAX77818_CHG_CV_PRM_4300MV,
	MAX77818_CHG_CV_PRM_4325MV,
	MAX77818_CHG_CV_PRM_4340MV,
	MAX77818_CHG_CV_PRM_4350MV,
	MAX77818_CHG_CV_PRM_4375MV,
	MAX77818_CHG_CV_PRM_4400MV,
	MAX77818_CHG_CV_PRM_4425MV,
	MAX77818_CHG_CV_PRM_4450MV,
	MAX77818_CHG_CV_PRM_4475MV,
	MAX77818_CHG_CV_PRM_4500MV,
	MAX77818_CHG_CV_PRM_4525MV,
	MAX77818_CHG_CV_PRM_4550MV,
	MAX77818_CHG_CV_PRM_4575MV,
	MAX77818_CHG_CV_PRM_4600MV,
	MAX77818_CHG_CV_PRM_4625MV,
	MAX77818_CHG_CV_PRM_4650MV,
	MAX77818_CHG_CV_PRM_4675MV,
	MAX77818_CHG_CV_PRM_4700MV,
};
enum ac_type_enum{
	AC_TYPE_USB_500MA = 0,
	AC_TYPE_APPLE_500MA=1,
	AC_TYPE_STANDARD_1A=2,
	AC_TYPE_UNKNOWN_1A=3,
	AC_TYPE_M76_17A=4,
	AC_TYPE_MAX,
};

struct max77818_charger_pdata
{
	int irq;
	enum max77818_charger_fchgtime fast_charge_time;
	enum max77818_charger_chg_rstrt charging_restart_thresold;
	enum ac_type_enum input_type;
	int input_limit_ac[AC_TYPE_MAX];		/* 100mA ~ 4000mA */
	enum max77818_charger_to_ith top_off_current_thresold;
	enum max77818_charger_top_off_timer top_off_timer;
	enum max77818_charger_chg_cv_prm charger_termination_voltage;
	bool enable_spread_spectrum;
};

struct max77818_charger
{
	struct power_supply ac;
	struct power_supply usb;
	struct max77818 *max77818;
	enum power_supply_type type;
	enum ac_type_enum input_type;
	int input_limit_ac[AC_TYPE_MAX];		/* 0mA ~ 2100mA */
	bool online;
	bool otg;
	int irq;

	struct class *charger_class;
	struct device *charger_device;
	struct notifier_block charger_notifier;
};

#endif