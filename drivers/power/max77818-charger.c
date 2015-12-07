/*
 * Maxim MAX77818 Charger Driver
 *
 * Copyright (C) 2014 Maxim Integrated Product
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This driver is based on max77843-charger.c
 */

#define DEBUG

#define log_level	0

#define CHARGER_DEBUG 0

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/power_supply.h>
#include <linux/mfd/max77818/max77818.h>
#include <linux/mfd/max77818/max77818-charger.h>
#include <linux/muic_tsu6721.h>
#include <linux/major.h>
#include <linux/kdev_t.h>
#include <linux/wakelock.h>

#define DRIVER_DESC    "MAX77818 Charger Driver"
#define DRIVER_NAME    MAX77818_CHARGER_NAME
#define DRIVER_VERSION "1.0"
#define DRIVER_AUTHOR  "TaiEup Kim <clark.kim@maximintegrated.com>"
#define TAG "[max77818_charger]"

#define M2SH	__CONST_FFS

/* Register map */
#define REG_CHG_INT					0xB0
#define REG_CHG_INT_MASK			0xB1
#define BIT_AICL					BIT (7)
#define BIT_CHGIN					BIT (6)
#define BIT_WCIN					BIT (5)
#define BIT_CHG						BIT (4)
#define BIT_BAT						BIT (3)
#define BIT_BATP					BIT (2)
#define BIT_BYP						BIT (0)

#define REG_CHG_INT_OK				0xB2
#define BIT_AICL_OK					BIT (7)
#define BIT_CHGIN_OK				BIT (6)
#define BIT_WCIN_OK					BIT (5)
#define BIT_CHG_OK					BIT (4)
#define BIT_BAT_OK					BIT (3)
#define BIT_BATP_OK					BIT (2)
#define BIT_BYP_OK					BIT (0)

#define REG_CHG_DTLS_00				0xB3
#define BIT_CHGIN_DTLS				BITS(6,5)
#define BIT_WCIN_DTLS				BITS(4,3)
#define BIT_BATP_DTLS				BIT (0)

#define REG_CHG_DTLS_01				0xB4
#define BIT_TREG					BIT (7)
#define BIT_BAT_DTLS				BITS(6,4)
#define BIT_CHG_DTLS				BITS(3,0)

#define REG_CHG_DTLS_02				0xB5
#define BIT_BYP_DTLS				BITS(3,0)
#define BIT_BCKNegILIM				BIT (2)
#define BIT_BSTILIM					BIT (1)
#define BIT_OTGILIM					BIT (0)

#define REG_CHG_CNFG_00				0xB7
#define BIT_OTG_CTRL				BIT (7)
#define BIT_DISIBS					BIT (6)
#define BIT_SPREAD					BIT (5)
#define BIT_WDTEN					BIT (4)
#define BIT_MODE					BITS(3,0)
#define BIT_MODE_BOOST				BIT (3)
#define BIT_MODE_BUCK				BIT (2)
#define BIT_MODE_OTG				BIT (1)
#define BIT_MODE_CHARGER			BIT (0)

#define REG_CHG_CNFG_01				0xB8
#define BIT_PQEN					BIT (7)
#define BIT_LSEL					BIT (6)
#define BIT_CHG_RSTRT				BITS(5,4)
#define BIT_FSW						BIT (3)
#define BIT_FCHGTIME				BITS(2,0)

#define REG_CHG_CNFG_02				0xB9
#define BIT_OTG_ILIM				BITS(7,6)
#define BIT_CHG_CC					BITS(5,0)

#define REG_CHG_CNFG_03				0xBA
#define BIT_ILIM					BITS(7,6)
#define BIT_TO_TIME					BITS(5,3)
#define BIT_TO_ITH					BITS(2,0)

#define REG_CHG_CNFG_04				0xBB
#define BIT_MINVSYS					BITS(7,6)
#define BIT_CHG_CV_PRM				BITS(5,0)

#define REG_CHG_CNFG_06				0xBD
#define BIT_CHGPROT					BITS(3,2)
#define BIT_WDTCLR					BITS(1,0)

#define REG_CHG_CNFG_07				0xBE
#define BIT_REGTEMP					BITS(6,5)

#define REG_CHG_CNFG_09				0xC0
#define BIT_CHGIN_ILIM				BITS(6,0)

#define REG_CHG_CNFG_10				0xC1
#define BIT_WCIN_ILIM				BITS(5,0)

#define REG_CHG_CNFG_11				0xC2
#define BIT_VBYPSET					BITS(6,0)

#define REG_CHG_CNFG_12				0xC3
#define BIT_WCINSEL					BIT (6)
#define BIT_CHGINSEL				BIT (5)
#define BIT_VCHGIN_REG				BITS(4,3)
#define BIT_B2SOVRC					BITS(2,0)

#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)

/* detail register bit description */
enum {
	WCIN_DTLS_UVLO,
	WCIN_DTLS_INVALID_01,
	WCIN_DTLS_OVLO,
	WCIN_DTLS_VALID,
};

enum {
	CHGIN_DTLS_UVLO,
	CHGIN_DTLS_INVALID_01,
	CHGIN_DTLS_OVLO,
	CHGIN_DTLS_VALID,
};

enum {
	CHG_DTLS_PREQUAL,
	CHG_DTLS_FASTCHARGE_CC,
	CHG_DTLS_FASTCHARGE_CV,
	CHG_DTLS_TOPOFF,
	CHG_DTLS_DONE,
	CHG_DTLS_RESEVRED_05,
	CHG_DTLS_OFF_TIMER_FAULT,
	CHG_DTLS_OFF_SUSPEND,
	CHG_DTLS_OFF_INPUT_INVALID,
	CHG_DTLS_RESERVED_09,
	CHG_DTLS_OFF_JUCTION_TEMP,
	CHG_DTLS_OFF_WDT_EXPIRED,
};

enum {
	BAT_DTLS_NO_BATTERY,
	BAT_DTLS_RESERVED_01,
	BAT_DTLS_TIMER_FAULT,
	BAT_DTLS_OKAY,
	BAT_DTLS_OKAY_LOW,
	BAT_DTLS_OVERVOLTAGE,
	BAT_DTLS_OVERCURRENT,
	BAT_DTLS_RESERVED_07,
};

struct max77818_charger_data {
	struct device           *dev;
	struct max77818_dev     *max77818;
	struct regmap			*regmap;

	struct power_supply		psy_chg;

	int						irq;
	int 						byp_irq;
	int						chgin_irq;
	int						chg_irq;
	int						details_0;
	int						details_1;
	int						details_2;
	/* mutex */
	struct mutex				lock;
	struct mutex				plug_lock;

	int						present;
	int						health;
	int						status;
	int						charge_type;
	bool						otg_mode;
	bool						usb_mode;
	bool						fast_usb_mode;

	struct max77818_charger_platform_data *pdata;
	struct notifier_block charger_notifier;
	struct notifier_block temp_notifier;
	struct class *charger_class;
	struct device *charger_device;
	int temp;
	struct delayed_work adjust_work;
	struct delayed_work status_work;
	bool aicl_triggered;
	struct wake_lock charger_wake_lock;
};
static enum power_supply_property max77818_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

#define GET_TO_ITH(X)	(X < 4 ? (X*25+100) : (X*50))	/* mA */

#define SET_TO_ITH(X)	(X < 100 ? 0x00 : 	\
		X < 200 ? (X-100)/25 :	\
		X < 350 ? (X/50) : 0x07)	/* mA */


/* charger API function */
static int max77818_charger_unlock(struct max77818_charger_data *charger)
{
	int rc;

	rc = max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_06, BIT_CHGPROT,
			BIT_CHGPROT);

	if (unlikely(IS_ERR_VALUE(rc))) {
		pr_err("%s: failed to unlock [%d]\n", __func__, rc);
		goto out;
	}

out:
	return rc;
}

static bool max77818_charger_present_input (struct max77818_charger_data *charger)
{
	u8 chg_int_ok = 0;
	int rc;

	rc = max77818_ch_read(charger->regmap, REG_CHG_INT_OK, &chg_int_ok);
	if (unlikely(IS_ERR_VALUE(rc))) {
		pr_err("%s: failed to read REG_CHG_INT_OK [%d]\n", __func__, rc);
		return false;
	}

	if ((chg_int_ok & BIT_CHGIN_OK) == BIT_CHGIN_OK) {
		return true;
	} else {
		/* check whether charging or not in the UVLO condition */
		if (((charger->details_0 & BIT_CHGIN_DTLS) == 0) &&
				(((charger->details_1 & BIT_CHG_DTLS) == CHG_DTLS_FASTCHARGE_CC) ||
				 ((charger->details_1 & BIT_CHG_DTLS) == CHG_DTLS_FASTCHARGE_CV))) {
			return true;
		} else {
			return false;
		}
	}
}

static int max77818_charger_get_input_current(struct max77818_charger_data *charger)
{
	u8 reg_data = 0;
	int steps[3] = { 0, 33, 67 };	/* mA */
	int get_current, quotient, remainder;


	max77818_ch_read(charger->regmap, REG_CHG_CNFG_09, &reg_data);

	quotient = reg_data / 3;
	remainder = reg_data % 3;

	if ((reg_data & BIT_CHGIN_ILIM) < 3)
		get_current = 100;	/* 100mA */
	else if ((reg_data & BIT_CHGIN_ILIM) > 0x78)
		get_current = 4000;	/* 4000mA */
	else
		get_current = quotient*100 + steps[remainder];
	return get_current;
}

static int max77818_charger_set_input_current(struct max77818_charger_data *charger,
		int input_current)
{
	int quotient, remainder;
	u8 reg_data = 0;

	/* unit mA */
	if (!input_current) {
		reg_data = 0;
	} else {
		quotient = input_current / 100;
		remainder = input_current % 100;

		if (remainder >= 67)
			reg_data |= (quotient * 3) + 2;
		else if (remainder >= 33)
			reg_data |= (quotient * 3) + 1;
		else if (remainder < 33)
			reg_data |= quotient * 3;
	}
#if CHARGER_DEBUG
	pr_info("%s: reg_data(0x%02x), input current(%d)\n",
			__func__, reg_data, input_current);
#endif
	return max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_09,
			BIT_CHGIN_ILIM, reg_data);
}
static int max77818_charger_get_charge_current(struct max77818_charger_data *charger)
{
	u8 reg_data = 0;
	int get_current;

	max77818_ch_read(charger->regmap, REG_CHG_CNFG_02, &reg_data);
#if CHARGER_DEBUG
	pr_info("%s reg_data = 0x%x",__func__,reg_data);
#endif
	if ((reg_data & BIT_CHG_CC) < 2)
		get_current = 100;	/* 100mA */
	else if ((reg_data & BIT_CHG_CC) > 0x3C)
		get_current = 3000;	/* 3000mA */
	else
		get_current = (reg_data & BIT_CHG_CC)*50;

	return get_current;
}

static int max77818_charger_set_charge_current(struct max77818_charger_data *charger,
		int fast_charging_current)
{
	int curr_step = 50;
	u8 reg_data = 0;
	int rc;

	if(fast_charging_current > charger->pdata->fast_charge_current)
		fast_charging_current = charger->pdata->fast_charge_current;
	/* unit mA */
	if (!fast_charging_current) {
		rc = max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_02,
				BIT_CHG_CC, 0);

	} else {
		reg_data = (fast_charging_current / curr_step);
		rc = max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_02,
				BIT_CHG_CC, reg_data);
	}
#if CHARGER_DEBUG
	pr_info("%s: reg_data(0x%02x), charging current(%d)\n",
			__func__, reg_data, fast_charging_current);
#endif
	return rc;

}

static int max77818_charger_set_topoff_current(struct max77818_charger_data *charger,
		int termination_current,
		int termination_time)
{
	u8 reg_data;

	/* termination_current (mA) */
	reg_data = SET_TO_ITH(termination_current);

	/* termination_time (min) */
	termination_time = termination_time;
	reg_data |= ((termination_time / 10) << M2SH(BIT_TO_TIME));
#if CHARGER_DEBUG
	pr_info("%s: reg_data(0x%02x), topoff(%d), time(%d)\n",
			__func__, reg_data, termination_current,
			termination_time);
#endif
	return max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_03,
			BIT_TO_ITH | BIT_TO_TIME, reg_data);

}

static int max77818_charger_set_enable (struct max77818_charger_data *charger, int on)
{
	unsigned int val;
	int ret;

	pr_info("%s enable = %d\n",__func__,on);
	if (on)
		val = 0x05 << M2SH(BIT_MODE);    /* charger=on, OTG=off, buck=on, boost=off */
	else
		val = 0x04 << M2SH(BIT_MODE);    /* charger=off, OTG=off, buck=on, boost=off */

	ret = max77818_ch_update_bits(charger->regmap,
			REG_CHG_CNFG_00, BIT_MODE, val);
	if (IS_ERR_VALUE(ret))
		dev_err(charger->dev, "Failed to set CHG_CNFG_00: %d\n", ret);

	return ret;
}
static int max77818_charger_set_reverse (struct max77818_charger_data *charger, int on)
{
	unsigned int val;
	int ret;

	pr_info("%s reverse = %d\n",__func__,on);
	if (on)
		val = 0x0A << M2SH(BIT_MODE);/* charger=off, OTG=on, buck=off, boost=on */
	else
		val = 0x04 << M2SH(BIT_MODE);   /* charger=on, OTG=off, buck=on, boost=off */

	ret = max77818_ch_update_bits(charger->regmap,
			REG_CHG_CNFG_00, BIT_MODE, val);
	if (IS_ERR_VALUE(ret))
		dev_err(charger->dev, "Failed to set CHG_CNFG_00: %d\n", ret);

	return ret;
}

static void max77818_charger_initialize(struct max77818_charger_data *charger)
{
	struct max77818_charger_platform_data *pdata = charger->pdata;
	int rc;
	u8 val=0, temp_val=0;

	pr_info("%s\n", __func__);

	/* unlock charger register */
	rc = max77818_charger_unlock(charger);
	if (unlikely(IS_ERR_VALUE(rc))) {
		goto out;
	}

	/* charge current (mA) */
	rc = max77818_charger_set_charge_current(charger, pdata->fast_charge_current);
	if (unlikely(IS_ERR_VALUE(rc))) {
		goto out;
	}

	/* input current limit (mA) */
	rc = max77818_charger_set_input_current(charger, pdata->input_current_limit);
	if (unlikely(IS_ERR_VALUE(rc))) {
		goto out;
	}

	/* topoff current(mA) and topoff timer(min) */
	rc = max77818_charger_set_topoff_current(charger, pdata->topoff_current, pdata->topoff_timer);
	if (unlikely(IS_ERR_VALUE(rc))) {
		goto out;
	}
	/* charge restart threshold(mV) and fast-charge timer(hr) */
	val = pdata->restart_threshold < 200 ?
		(int)(pdata->restart_threshold - 100)/50 : 0x03;

	temp_val = pdata->fast_charge_timer == 0 ? 0x00 :
		pdata->fast_charge_timer < 4 ? 0x01 :
		pdata->fast_charge_timer < 16 ?
		(int)DIV_ROUND_UP(pdata->fast_charge_timer - 4, 2) + 1 : 0x00;

	val = val<<M2SH(BIT_CHG_RSTRT) | temp_val<<M2SH(BIT_FCHGTIME);

	rc = max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_01,
			(BIT_CHG_RSTRT | BIT_FCHGTIME), val);
	if (unlikely(IS_ERR_VALUE(rc))) {
		goto out;
	}

	/* charge termination voltage (mV) */
	val = pdata->termination_voltage < 3650 ? 0x00 :
		pdata->termination_voltage <= 4325 ?
		(int)DIV_ROUND_UP(pdata->termination_voltage - 3650, 25) :
		pdata->termination_voltage <= 4340 ? 0x1C :
		pdata->termination_voltage <= 4700 ?
		(int)DIV_ROUND_UP(pdata->termination_voltage - 3650, 25) + 1 : 0x2B;
	rc = max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_04, BIT_CHG_CV_PRM,
			val << M2SH(BIT_CHG_CV_PRM));
	if (unlikely(IS_ERR_VALUE(rc))) {
		goto out;
	}
	val = pdata->otg_current_limit/500;
	rc = max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_02, BIT_OTG_ILIM,
			val << M2SH(BIT_OTG_ILIM));
	if (unlikely(IS_ERR_VALUE(rc))) {
		goto out;
	}

	//disable wireless charging
	rc = max77818_ch_update_bits(charger->regmap, REG_CHG_CNFG_12, BIT_WCINSEL,0);
	if (unlikely(IS_ERR_VALUE(rc))) {
		goto out;
	}
out:
	return;
}

struct max77818_charger_status_map {
	int health, status, charge_type;
};

static struct max77818_charger_status_map max77818_charger_status_map[] = {
#define STATUS_MAP(_chg_dtls, _health, _status, _charge_type) \
	[CHG_DTLS_##_chg_dtls] = {\
		.health = POWER_SUPPLY_HEALTH_##_health,\
		.status = POWER_SUPPLY_STATUS_##_status,\
		.charge_type = POWER_SUPPLY_CHARGE_TYPE_##_charge_type,\
	}
	//                 chg_details_xx		health			status			charge_type
	STATUS_MAP(PREQUAL,				GOOD,					CHARGING,		TRICKLE),
	STATUS_MAP(FASTCHARGE_CC,		GOOD,					CHARGING,		FAST),
	STATUS_MAP(FASTCHARGE_CV,		GOOD,					CHARGING,		FAST),
	STATUS_MAP(TOPOFF,				GOOD,					CHARGING,		FAST),
	STATUS_MAP(DONE,				GOOD,					FULL,			NONE),
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	STATUS_MAP(OFF_TIMER_FAULT,		SAFETY_TIMER_EXPIRE,	NOT_CHARGING,	NONE),
#else /* LINUX_VERSION_CODE ... */
	STATUS_MAP(OFF_TIMER_FAULT,     UNKNOWN,				NOT_CHARGING,	NONE),
#endif /* LINUX_VERSION_CODE ... */
	STATUS_MAP(OFF_SUSPEND,			UNKNOWN,				NOT_CHARGING,	NONE),
	STATUS_MAP(OFF_INPUT_INVALID,   UNKNOWN,				NOT_CHARGING,   NONE),
	STATUS_MAP(OFF_JUCTION_TEMP,    UNKNOWN,				NOT_CHARGING,	UNKNOWN),
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	STATUS_MAP(OFF_WDT_EXPIRED,		WATCHDOG_TIMER_EXPIRE,  NOT_CHARGING,	UNKNOWN),
#else /* LINUX_VERSION_CODE ... */
	STATUS_MAP(OFF_WDT_EXPIRED,		UNKNOWN,				NOT_CHARGING,	UNKNOWN),
#endif /* LINUX_VERSION_CODE ... */
};
static int max77818_charger_update (struct max77818_charger_data *charger)
{
	int rc;
	u8 chg_details;
	u8 chg_dtls;

	charger->health      = POWER_SUPPLY_HEALTH_UNKNOWN;
	charger->status      = POWER_SUPPLY_STATUS_UNKNOWN;
	charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	rc = max77818_ch_read(charger->regmap, REG_CHG_DTLS_01, &chg_details);
	if (unlikely(IS_ERR_VALUE(rc))) {
		log_err("CHG_DETAILS read error [%d]\n", rc);
		goto out;
	}
	if (!charger->present) {
		/* no charger present */
		charger->health      = POWER_SUPPLY_HEALTH_UNKNOWN;
		charger->status      = POWER_SUPPLY_STATUS_DISCHARGING;
		charger->charge_type = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		goto out;
	}

	chg_dtls = chg_details & BIT_CHG_DTLS;

	charger->health      = max77818_charger_status_map[chg_dtls].health;
	charger->status      = max77818_charger_status_map[chg_dtls].status;
	charger->charge_type = max77818_charger_status_map[chg_dtls].charge_type;

	if (likely(charger->health != POWER_SUPPLY_HEALTH_UNKNOWN)) {
		goto out;
	}

	/* override health by TREG */
	if ((chg_details & BIT_TREG) != 0)
		charger->health = POWER_SUPPLY_HEALTH_OVERHEAT;

out:
	return rc;
}
static int max77818_charger_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	switch(psp)
	{
		case POWER_SUPPLY_PROP_CURRENT_MAX:
		case	POWER_SUPPLY_PROP_CURRENT_NOW:
		case POWER_SUPPLY_PROP_ONLINE:
			return 1;
		default:
			return -EINVAL;
	}
}

static int max77818_charger_get_property (struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct max77818_charger_data *charger =
		container_of(psy, struct max77818_charger_data, psy_chg);

	int rc = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = charger->present;
			break;

		case POWER_SUPPLY_PROP_HEALTH:
			max77818_charger_update(charger);
			val->intval = charger->health;
			break;

		case POWER_SUPPLY_PROP_STATUS:
			max77818_charger_update(charger);
			val->intval = charger->status;
			break;

		case POWER_SUPPLY_PROP_CHARGE_TYPE:
			max77818_charger_update(charger);
			val->intval = charger->charge_type;
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			val->intval = max77818_charger_get_charge_current(charger);
			break;

		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = max77818_charger_get_input_current(charger);
			break;

		default:
			rc = -EINVAL;
			goto out;
	}

out:
	return rc;
}

static int max77818_charger_set_property (struct power_supply *psy,
		enum power_supply_property psp, const union power_supply_propval *val)
{
	struct max77818_charger_data *charger =
		container_of(psy, struct max77818_charger_data, psy_chg);

	int rc = 0;

	__lock(charger);

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			rc = max77818_charger_set_enable(charger, val->intval);
			if (unlikely(IS_ERR_VALUE(rc))) {
				goto out;
			}

			/* apply charge current */
			rc = max77818_charger_set_charge_current(charger, charger->pdata->fast_charge_current);
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			/* val->intval - uA */
			rc = max77818_charger_set_charge_current(charger, val->intval/1000); /* mA */
			if (unlikely(IS_ERR_VALUE(rc))) {
				goto out;
			}
			charger->pdata->fast_charge_current = val->intval/1000;	/* mA */
			break;

		case POWER_SUPPLY_PROP_CURRENT_MAX:
			rc = max77818_charger_set_input_current(charger, val->intval/1000);	/* mA */
			if (unlikely(IS_ERR_VALUE(rc))) {
				goto out;
			}
			charger->pdata->input_current_limit = val->intval/1000;	/* mA */
			break;

		default:
			rc = -EINVAL;
			goto out;
	}

out:
	pr_info("%s: <set_property> psp %d val %d [%d]\n", __func__, psp, val->intval, rc);
	__unlock(charger);
	return rc;
}
static void max77818_plug_event(struct max77818_charger_data *charger)
{
	mutex_lock(&charger->plug_lock);
	cancel_delayed_work_sync(&charger->adjust_work);
	//step1. set input current
	//step2. set charge current
	if(charger->usb_mode){
		if(charger->fast_usb_mode){
			max77818_charger_set_input_current(charger,charger->pdata->usb_fast_charge_current);
			max77818_charger_set_charge_current(charger,charger->pdata->usb_fast_charge_current);
		}else{
			max77818_charger_set_input_current(charger,charger->pdata->usb_charge_current);
			max77818_charger_set_charge_current(charger,charger->pdata->usb_charge_current);
		}
	}else if(charger->otg_mode){
		//otg mode do nothing about current
	}else{
		if(charger->temp == COLD_TEMP){
			max77818_charger_set_input_current(charger,charger->pdata->input_current_limit);
			max77818_charger_set_charge_current(charger,charger->pdata->fast_charge_current_low);
		}else{
			max77818_charger_set_input_current(charger,charger->pdata->input_current_limit);
			max77818_charger_set_charge_current(charger,charger->pdata->fast_charge_current);
			wake_lock_timeout(&charger->charger_wake_lock,5*HZ);
			schedule_delayed_work(&charger->adjust_work, 1000);
		}
	}
	//step3. enable charge
	if(charger->otg_mode){
		max77818_charger_set_reverse(charger,true);
		charger->present = false;
	}else{
		if(charger->temp == FREEZE_TEMP || charger->temp == HOT_TEMP){
			max77818_charger_set_enable(charger,false);
			pr_info("battery temp too cold or too hot,disable charging\n");
		}else if(max77818_charger_present_input(charger)){
			max77818_charger_set_enable(charger,true);
		}
	}

	mutex_unlock(&charger->plug_lock);
}
static void max77818_unplug_event(struct max77818_charger_data *charger)
{
	mutex_lock(&charger->plug_lock);
	charger->aicl_triggered = false;
	cancel_delayed_work_sync(&charger->adjust_work);
	//step1. set input current
	//step2. set charge current
	max77818_charger_set_input_current(charger,charger->pdata->input_current_limit);
	max77818_charger_set_charge_current(charger,charger->pdata->fast_charge_current);
	//step3. disable charge
	max77818_charger_set_enable(charger,false);
	mutex_unlock(&charger->plug_lock);
}

static void max77818_do_irq(struct max77818_charger_data *charger, int irq)
{
	bool chg_input;


	switch (irq) {
		case CHG_INT_CHGIN_I:
			chg_input = max77818_charger_present_input(charger);
			log_dbg("CHG_INT_CHGIN: Charger input %s\n", chg_input ? "inserted" : "removed");

			if (chg_input) {
				/* charger insert */
				charger->present= true;
				charger->psy_chg.type=POWER_SUPPLY_TYPE_MAINS;
				max77818_plug_event(charger);
			} else {
				/* charger remove */
				charger->present= false;
				charger->usb_mode = false;
				max77818_unplug_event(charger);
			}
			schedule_delayed_work(&charger->status_work,100);
			break;
		case CHG_INT_CHG_I:
			max77818_charger_update(charger);
			break;
		default:
			break;
	}
	return;

}
static irqreturn_t max77818_charger_chgin_isr (int irq, void *data)
{
	struct max77818_charger_data *me = data;
	u8 reg_details[3];
	wake_unlock(&me->charger_wake_lock);
	wake_lock_timeout(&me->charger_wake_lock,5*HZ);
	__lock(me);

	me->irq = irq;
	irq = me->irq - me->byp_irq;
	max77818_ch_bulk_read(me->regmap, REG_CHG_DTLS_00, reg_details, 3);
#if CHARGER_DEBUG
	pr_info("%s: chg_dtls[0]=0x%x, [1]=0x%x, [2]=0x%x\n",
			__func__, reg_details[0], reg_details[1], reg_details[2]);
#endif
	me->details_0 = reg_details[0];
	me->details_1 = reg_details[1];
	me->details_2 = reg_details[2];

	max77818_do_irq(me, irq);
	__unlock(me);
	power_supply_changed(&me->psy_chg);
	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static int max77818_charger_parse_dt(struct max77818_charger_data *charger)
{
	struct device *dev = charger->dev;
	struct device_node *nproot = dev->parent->of_node;
	struct device_node *np = of_find_node_by_name(nproot,"charger");
	struct max77818_charger_platform_data *pdata;

	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(pdata == NULL))
		return -ENOMEM;

	pdata->fast_charge_timer = 0;	// disable
	ret |= of_property_read_u32(np, "fast_charge_timer",
			&pdata->fast_charge_timer);
	log_dbg("property:FCHGTIME		   %uhour\n", pdata->fast_charge_timer);

	pdata->fast_charge_current = 1625;
	ret |= of_property_read_u32(np, "fast_charge_current",
			&pdata->fast_charge_current);
	log_dbg("property:CHG_CC		   %umA\n", pdata->fast_charge_current);

	pdata->termination_voltage = 4350;
	ret |= of_property_read_u32(np, "charge_termination_voltage",
			&pdata->termination_voltage);
	log_dbg("property:CHG_CV_PRM		%umV\n",
			pdata->termination_voltage);

	pdata->topoff_timer = 0;
	ret |= of_property_read_u32(np, "topoff_timer", &pdata->topoff_timer);
	log_dbg("property:TOPOFF_TIME	   %umin\n", pdata->topoff_timer);

	pdata->topoff_current = 120;
	ret |= of_property_read_u32(np, "topoff_current", &pdata->topoff_current);
	log_dbg("property:TOPOFF_ITH		 %umA\n", pdata->topoff_current);

	pdata->restart_threshold = 150;
	ret |= of_property_read_u32(np, "restart_threshold",
			&pdata->restart_threshold);
	log_dbg("property:CHG_RSTRT 	   %umV\n", pdata->restart_threshold);

	pdata->input_current_limit = 1700;
	ret |= of_property_read_u32(np, "input_current_limit",
			&pdata->input_current_limit);
	log_dbg("property:INPUT_CURRENT_LIMIT %umA\n", pdata->input_current_limit);

	pdata->otg_current_limit = 1500;
	ret |= of_property_read_u32(np, "otg_current_limit",
			&pdata->otg_current_limit);
	log_dbg("property:OTG_CURRENT_LIMIT %umA\n", pdata->otg_current_limit);

	pdata->fast_charge_current_low= 975;
	ret |= of_property_read_u32(np, "fast_charge_current_low",
			&pdata->fast_charge_current_low);
	log_dbg("property:fast_charge_current_low %umA\n", pdata->fast_charge_current_low);

	pdata->usb_charge_current= 500;
	ret |= of_property_read_u32(np, "usb_charge_current",
			&pdata->usb_charge_current);
	log_dbg("property:usb_charge_current %umA\n", pdata->usb_charge_current);

	pdata->usb_fast_charge_current= 900;
	ret |= of_property_read_u32(np, "usb_fast_charge_current",
			&pdata->usb_fast_charge_current);
	log_dbg("property:usb_fast_charge_current %umA\n", pdata->usb_fast_charge_current);

	charger->pdata = pdata;
	return 0;
}
#endif
static int temp_event_notify(struct notifier_block *this, unsigned long code,
		void *unused)
{
	struct max77818_charger_data * charger;

	charger = container_of(this,struct max77818_charger_data,temp_notifier);
	wake_unlock(&charger->charger_wake_lock);
	wake_lock_timeout(&charger->charger_wake_lock,5*HZ);
	pr_info("%s\n",__func__);
	switch(code){
		case FREEZE_TEMP:
			charger->temp = FREEZE_TEMP;
			break;
		case COLD_TEMP:
			charger->temp = COLD_TEMP;
			break;
		case ROOM_TEMP:
			charger->temp = ROOM_TEMP;
			break;
		case HOT_TEMP:
			charger->temp = HOT_TEMP;
		case FULL_CAP:
			charger->status = POWER_SUPPLY_STATUS_FULL;
			break;
	}
	max77818_plug_event(charger);
	power_supply_changed(&charger->psy_chg);
	return NOTIFY_DONE;
}

static int muic_event_notify(struct notifier_block *this, unsigned long code,
		void *unused)
{
	struct max77818_charger_data * charger;

	charger = container_of(this,struct max77818_charger_data,charger_notifier);
	wake_unlock(&charger->charger_wake_lock);
	wake_lock_timeout(&charger->charger_wake_lock,5*HZ);
	switch(code){
		case USB_HOST_ATTACH:
			pr_info(TAG"usb host attach notify\n");
			charger->usb_mode = true;
			charger->present = true;
			charger->psy_chg.type=POWER_SUPPLY_TYPE_USB;
			max77818_plug_event(charger);
			break;
		case USB_HOST_DETACH:
			charger->usb_mode = false;
			charger->present = false;
			charger->psy_chg.type=POWER_SUPPLY_TYPE_MAINS;
			max77818_unplug_event(charger);
			pr_info(TAG"usb host detach notify\n");
			break;
		case USB_OTG_ATTACH:
			charger->otg_mode = true;
			max77818_plug_event(charger);
			pr_info(TAG"otg attach notify\n");
			break;
		case USB_OTG_DETACH:
			charger->otg_mode = false;
			max77818_unplug_event(charger);
			pr_info(TAG"otg detach notify\n");
			break;
	}
	power_supply_changed(&charger->psy_chg);
	return NOTIFY_DONE;
}
static ssize_t attr_set_reg(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	char rw[10];
	int reg,value,ret;
	struct max77818_charger_data *charger = dev_get_drvdata(dev);

	sscanf(buf,"%s %x %x",rw,&reg, &value);
	if(!strcmp(rw,"read")){
		ret = max77818_ch_read(charger->regmap,(u8)reg,(u8 *)(&value));
		pr_info(TAG"read from [%x] value = 0x%2x\n", reg, value);
	}else if(!strcmp(rw,"write")){
#ifndef CONFIG_USER_KERNEL
		ret = max77818_ch_write(charger->regmap, (u8)reg, value);
		pr_info(TAG"write to [%x] value = 0x%2x\n", reg, value);
#else
		pr_info(TAG"write is disabled from userspace\n");
#endif
	}
	return size;
}
static ssize_t usb_fast_charge_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int fast_en=0;
	struct max77818_charger_data *charger = dev_get_drvdata(dev);

	sscanf(buf,"%d",&fast_en);
	if(fast_en){
		charger->fast_usb_mode = true;
		pr_info("enable usb fast charge\n");
	}else{
		charger->fast_usb_mode = false;
		pr_info("disable usb fast charge\n");
	}
	max77818_plug_event(charger);
	return size;
}
static ssize_t usb_fast_charge_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct max77818_charger_data *charger = dev_get_drvdata(dev);

	sprintf(buf,"%d\n",charger->fast_usb_mode);
	return 2;
}
static ssize_t attr_dump_reg(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct max77818_charger_data *charger = dev_get_drvdata(dev);
	int i=0,offset=0,ret =0;
	u8 val;
	static char buf_bak[20];
	for(i=0xB0;i<=0xC3;i++){
		//skip these registers
		if(i == 0xb6 || i == 0xbc || i == 0xbf )
			continue;
		ret=max77818_read(charger->regmap, i, &val);
		sprintf(buf_bak,"0x%02x,",val);
		strcat(buf,buf_bak);
		offset +=strlen(buf_bak);
	}
	sprintf(buf_bak,"\n");
	strcat(buf,buf_bak);
	offset +=strlen(buf_bak);
	return offset;
}

static struct device_attribute attributes[] = {
	__ATTR(reg_control, 0200, NULL, attr_set_reg),
	__ATTR(usb_fast_charge_enable, 0600, usb_fast_charge_show, usb_fast_charge_store),
	__ATTR(dump_reg, 0444, attr_dump_reg,NULL ),
};
static int create_sysfs_files(struct device *dev)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(attributes); i++)
		if(device_create_file(dev, attributes + i))
			goto err;
	return 0;
err:
	for(; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	pr_err(TAG"unable to create sysfs interface\n");
	return -1;
}
static dev_t const charger_device_dev_t = MKDEV(MISC_MAJOR, 242);
static void max77818_adjust_work(struct work_struct *work)
{
	u8 chg_int_ok;
	bool aicl_mode;
	struct max77818_charger_data *charger = container_of(work,
			struct max77818_charger_data, adjust_work.work);
	int charge_now = max77818_charger_get_charge_current(charger);
	int input_now = max77818_charger_get_input_current(charger);

	wake_unlock(&charger->charger_wake_lock);
	wake_lock_timeout(&charger->charger_wake_lock,5*HZ);
	max77818_ch_read(charger->regmap,REG_CHG_INT_OK,&chg_int_ok);
	aicl_mode = !(chg_int_ok & BIT_AICL_OK);
	pr_info("%s input_current now %d charge current now %d %s\n", __func__, input_now,charge_now,
			aicl_mode?"AICL mode":"not AICL mode");
	if(aicl_mode){
		charger->aicl_triggered = true;
		max77818_charger_set_input_current(charger,charge_now-50);
		max77818_charger_set_charge_current(charger,charge_now-150);
		schedule_delayed_work(&charger->adjust_work,1000);
	}else{
		if(charger->aicl_triggered){
			max77818_charger_set_input_current(charger,input_now-100);
			charger->aicl_triggered = false;
		}
	}
}
static void max77818_status_work(struct work_struct *work)
{
	struct max77818_charger_data *charger = container_of(work,
			struct max77818_charger_data, status_work.work);
	max77818_charger_update(charger);
	power_supply_changed(&charger->psy_chg);
}

static int max77818_charger_probe(struct platform_device *pdev)
{
	struct max77818_dev *max77818 = dev_get_drvdata(pdev->dev.parent);
	struct max77818_charger_platform_data *pdata=NULL;
	struct max77818_charger_data *charger=NULL;
	int ret = 0;
	u8 val = 0;

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		pr_err("%s: out of memory\n", __func__);
		ret = -ENOMEM;
		goto err_mem_alloc_charger;
	}

	mutex_init(&charger->lock);
	mutex_init(&charger->plug_lock);
	wake_lock_init(&charger->charger_wake_lock,WAKE_LOCK_SUSPEND,"charger_wake");

	charger->max77818 = max77818;
	charger->dev = &pdev->dev;
	charger->regmap = max77818->regmap_chg;
	charger->pdata = pdata;

#if defined(CONFIG_OF)
	ret = max77818_charger_parse_dt(charger);
	if (ret < 0) {
		pr_err("%s not found charger dt! ret[%d]\n",__func__, ret);
	}
#else
	pdata = dev_get_platdata(&pdev->dev);
#endif

	platform_set_drvdata(pdev, charger);

	charger->psy_chg.name		= "charger";
	charger->psy_chg.type		= POWER_SUPPLY_TYPE_MAINS;
	charger->psy_chg.get_property	= max77818_charger_get_property;
	charger->psy_chg.set_property	= max77818_charger_set_property;
	charger->psy_chg.property_is_writeable = max77818_charger_property_is_writeable;
	charger->psy_chg.properties	= max77818_charger_props;
	charger->psy_chg.num_properties	= ARRAY_SIZE(max77818_charger_props);

	ret = power_supply_register(&pdev->dev, &charger->psy_chg);
	if (ret) {
		pr_err("%s: Failed to Register psy_chg\n", __func__);
		goto err_power_supply_register;
	}
	INIT_DEFERRABLE_WORK(&charger->adjust_work,max77818_adjust_work);
	INIT_DEFERRABLE_WORK(&charger->status_work,max77818_status_work);
	max77818_charger_initialize(charger);
	charger->charger_notifier.notifier_call = muic_event_notify;
	charger->temp_notifier.notifier_call = temp_event_notify;
	ret = register_muic_notifier(&charger->charger_notifier);
	ret = register_temp_notifier(&charger->temp_notifier);
	max77818_do_irq(charger, CHG_INT_CHGIN_I);
	check_cable_status();
	if(ret){
		pr_err(TAG"%s register notifer fail\n", __func__);
	}
	charger->charger_class = class_create(THIS_MODULE,"charger_class");
	if(IS_ERR(charger->charger_class)){
		ret = PTR_ERR(charger->charger_class);
		pr_err(TAG"%s charger class create fail\n", __func__);
		goto err_class_create;
	}

	charger->charger_device = device_create(charger->charger_class,
			NULL,charger_device_dev_t,charger,"charger_device");
	if(IS_ERR(charger->charger_device)){
		ret = PTR_ERR(charger->charger_device);
		pr_err(TAG"%s charger device create fail\n", __func__);
		goto err_device_create;
	}

	ret = create_sysfs_files(charger->charger_device);
	if(ret < 0){
		pr_err(TAG"%s sysfiles create fail\n", __func__);
		goto err_create_sys;
	}

	charger->chgin_irq = regmap_irq_get_virq(max77818->irqc_chg, CHG_IRQ_CHGIN_I);
	charger->chg_irq = regmap_irq_get_virq(max77818->irqc_chg, CHG_IRQ_CHG_I);
	charger->byp_irq = regmap_irq_get_virq(max77818->irqc_chg, CHG_IRQ_BYP_I);
	ret = request_threaded_irq(charger->chgin_irq, NULL, max77818_charger_chgin_isr,
			IRQF_TRIGGER_LOW |IRQF_ONESHOT, "charger-chgin", charger);
	if (ret) {
		pr_err("%s: Failed to Reqeust CHGIN IRQ\n", __func__);
		goto err_chgin_irq;
	}

	ret = request_threaded_irq(charger->chg_irq, NULL, max77818_charger_chgin_isr,
			IRQF_TRIGGER_LOW |IRQF_ONESHOT, "charger-chg", charger);
	if (ret) {
		pr_err("%s: Failed to Reqeust CHG IRQ\n", __func__);
		goto err_chgin_irq;
	}
	max77818_ch_read(charger->regmap, REG_CHG_INT, &val);

	pr_info("%s: Max77818 Charger Driver Loaded\n", __func__);

	return 0;

err_chgin_irq:
	power_supply_unregister(&charger->psy_chg);
err_create_sys:
	device_destroy(charger->charger_class,charger_device_dev_t);
err_device_create:
	class_destroy(charger->charger_class);
err_class_create:
	unregister_muic_notifier(&charger->charger_notifier);
err_power_supply_register:
	kfree(charger->pdata);
err_mem_alloc_charger:
	kfree(charger);

	return ret;
}

static int max77818_charger_remove(struct platform_device *pdev)
{
	struct max77818_charger_data *charger =
		platform_get_drvdata(pdev);

	free_irq(charger->chgin_irq, NULL);
	power_supply_unregister(&charger->psy_chg);
	mutex_destroy(&charger->lock);
	wake_lock_destroy(&charger->charger_wake_lock);
	kfree(charger);

	return 0;
}

#if defined CONFIG_PM
static int max77818_charger_suspend(struct device *dev)
{
	return 0;
}

static int max77818_charger_resume(struct device *dev)
{
	return 0;
}
#else
#define max77818_charger_suspend NULL
#define max77818_charger_resume NULL
#endif

#ifdef CONFIG_OF
static struct of_device_id max77818_charger_dt_ids[] = {
	{ .compatible = "maxim,max77818-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77818_charger_dt_ids);
#endif

static SIMPLE_DEV_PM_OPS(max77818_charger_pm_ops, max77818_charger_suspend,
		max77818_charger_resume);

static struct platform_driver max77818_charger_driver = {
	.driver = {
		.name = MAX77818_CHARGER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &max77818_charger_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = max77818_charger_dt_ids,
#endif
	},
	.probe = max77818_charger_probe,
	.remove = max77818_charger_remove,
};
module_platform_driver(max77818_charger_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_VERSION(DRIVER_VERSION);
