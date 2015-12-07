/*
 *  max77818_battery.c
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This driver is based on max17048_battery.c
 */

#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77818/max77818.h>
#include <linux/mfd/max77818/max77818_battery.h>
#include <linux/alarmtimer.h>
#include <linux/reboot.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/power/meizu_adc.h>
#include <mach/hardware.h>
#include <linux/wakelock.h>
#include <linux/fb.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>



#define BIT_SMX				BIT (14)
#define BIT_TMX				BIT (13)
#define BIT_VMX				BIT (12)
#define BIT_SMN				BIT (10)
#define BIT_TMN				BIT (9)
#define BIT_VMN				BIT (8)
#define BIT_dSOCi			BIT (7)
#define STATUS_POR_BIT		BIT(1)
#define	BIT_Aen				BIT (2)
#define BIT_dSOCen			BIT (7)


#define VFSOC0_LOCK		0x0000
#define VFSOC0_UNLOCK	0x0080
#define MODEL_UNLOCK1	0X0059
#define MODEL_UNLOCK2	0X00C4
#define MODEL_LOCK1		0X0000
#define MODEL_LOCK2		0X0000

#define MAX77818_BATTERY_FULL	100
#define MAX77818_BATTERY_LOW	10
#define MAX77818_DEBUG_INFO 1

#define MAX77818_VERSION_NO	0x20B0

#define TAG "[max77818-fuelgauge]"
//save learned params every 3 hours
#define SAVE_LEARN_TIME (3*3600)
u16	atl_cell_char_tbl[MAX77818_CHARACTERIZATION_DATA_SIZE]={
	0xa510,
	0xb750,
	0xb8f0,
	0xbae0,
	0xbc40,
	0xbd20,
	0xbe00,
	0xbec0,
	0xbfb0,
	0xc0a0,
	0xc380,
	0xc650,
	0xc950,
	0xcc40,
	0xd210,
	0xd7f0,
	0x01b0,
	0x0ff0,
	0x0df0,
	0x10f0,
	0x1c10,
	0x1a00,
	0x1d00,
	0x1510,
	0x15d0,
	0x08f0,
	0x08f0,
	0x08f0,
	0x08f0,
	0x06f0,
	0x0680,
	0x0680,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
};
u16 sony_cell_char_tbl[MAX77818_CHARACTERIZATION_DATA_SIZE]={
	0x9610,
	0xa010,
	0xb550,
	0xb7e0,
	0xb830,
	0xba60,
	0xbc50,
	0xbd60,
	0xbe50,
	0xc040,
	0xc2c0,
	0xc540,
	0xc7a0,
	0xcc80,
	0xd220,
	0xd780,
	0x01c0,
	0x00b0,
	0x0730,
	0x2de0,
	0x0af0,
	0x1300,
	0x1810,
	0x1690,
	0x1230,
	0x0ce0,
	0x08c0,
	0x0980,
	0x07f0,
	0x0700,
	0x0660,
	0x0660,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
};
u16 sdi_cell_char_tbl[MAX77818_CHARACTERIZATION_DATA_SIZE]={
	0xa3c0,
	0xb770,
	0xba50,
	0xbb40,
	0xbc40,
	0xbca0,
	0xbd70,
	0xbe70,
	0xbf70,
	0xc100,
	0xc2e0,
	0xc5f0,
	0xc960,
	0xcd40,
	0xd1f0,
	0xd6f0,
	0x0150,
	0x1240,
	0x0b90,
	0x1480,
	0x2210,
	0x1f30,
	0x15c0,
	0x18f0,
	0x0cf0,
	0x0bd0,
	0x0a70,
	0x08f0,
	0x06f0,
	0x07f0,
	0x06d0,
	0x06d0,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,
	0x0100,

};

static struct max77818_config_data atl_config_data_old_board = {
	/* A/D measurement */
	.tgain=0xE259,
	.toff=0x2acc,

	.coff = 0xFFA0,
	/* Alert / Status */
	.config=0x0210,
	.config2=0x00D0,

	/* App data */
	.full_soc_thresh=0x5f00,
	.design_cap=0x1a04,
	.ichgt_term=0x0320,

	/* MG3 config */
	.at_rate=0x4b00,
	.learn_cfg=0x2603,
	.filter_cfg=0XCC24,
	.misc_cfg=0x0810,
	.relax_cfg=0x201F,

	/* MG3 save and restore */
	.fullcapnom=0x1a04,
	.fullcaprep=0x1a04,
	.dqacc=0x0681,
	.dpacc=0x3200,
	.convgcfg=0x2271,
	.qresidual00=0x2f80,
	.qresidual10=0x2100,
	.qresidual20=0x1700,
	.qresidual30=0x1200,
	.v_empty=0xaa56,

	/* Cell Data */
	.rcomp0=0x0082,
	.tcompc0=0x353a,
	.cell_char_tbl = atl_cell_char_tbl,
};
static struct max77818_config_data atl_config_data_new_board = {
	/* A/D measurement */
	.tgain=0xE259,
	.toff=0x2acc,

	.coff = 0x0000,
	/* Alert / Status */
	.config=0x0210,
	.config2=0x00D0,

	/* App data */
	.full_soc_thresh=0x5f00,
	.design_cap=0x1a04,
	.ichgt_term=0x0320,

	/* MG3 config */
	.at_rate=0x4b00,
	.learn_cfg=0x2602,
	.filter_cfg=0xCEA4,
	.misc_cfg=0x09d0,
	.relax_cfg=0x203B,

	/* MG3 save and restore */
	.fullcapnom=0x1a04,
	.fullcaprep=0x1a04,
	.dqacc=0x0681,
	.dpacc=0x3200,
	.convgcfg=0x2271,
	.qresidual00=0x2f80,
	.qresidual10=0x2100,
	.qresidual20=0x1a00,
	.qresidual30=0x1400,
	.v_empty=0xaa56,

	/* Cell Data */
	.rcomp0=0x0082,
	.tcompc0=0x353a,
	.cell_char_tbl = atl_cell_char_tbl,
};
static struct max77818_config_data sony_config_data_new_board = {
	/* A/D measurement */
	.tgain=0xE259,
	.toff=0x2acc,

	.coff = 0x0000,
	/* Alert / Status */
	.config=0x0210,
	.config2=0x00D0,

	/* App data */
	.full_soc_thresh=0x5f00,
	.design_cap=0x1a40,
	.ichgt_term=0x0320,

	/* MG3 config */
	.at_rate=0x28a0,
	.learn_cfg=0x2602,
	.filter_cfg=0xCEA4,
	.misc_cfg=0x09d0,
	.relax_cfg=0x203B,
	.cv_mixcap = 0x13b0,
	.cv_halftime = 0x0600,

	/* MG3 save and restore */
	.fullcapnom=0x1a40,
	.fullcaprep=0x1a40,
	.dqacc=0x0690,
	.dpacc=0x3200,
	.convgcfg=0x2271,
	.qresidual00=0x4f09,
	.qresidual10=0x2c09,
	.qresidual20=0x1a0b,
	.qresidual30=0x140c,
	.v_empty=0xaa56,

	/* Cell Data */
	.rcomp0=0x0081,
	.tcompc0=0x403C,
	.cell_char_tbl = sony_cell_char_tbl,
};
static struct max77818_config_data sony_config_data_old_board = {
	/* A/D measurement */
	.tgain=0xE259,
	.toff=0x2acc,

	.coff = 0xFFA0,
	/* Alert / Status */
	.config=0x0210,
	.config2=0x00D0,

	/* App data */
	.full_soc_thresh=0x5f00,
	.design_cap=0x1a40,
	.ichgt_term=0x0320,

	/* MG3 config */
	.at_rate=0x28a0,
	.learn_cfg=0x2603,
	.filter_cfg=0XCC24,
	.misc_cfg=0x0810,
	.relax_cfg=0x201F,
	.cv_mixcap = 0x13b0,
	.cv_halftime = 0x0600,

	/* MG3 save and restore */
	.fullcapnom=0x1a40,
	.fullcaprep=0x1a40,
	.dqacc=0x0690,
	.dpacc=0x3200,
	.convgcfg=0x2271,
	.qresidual00=0x4f09,
	.qresidual10=0x2c09,
	.qresidual20=0x1a0b,
	.qresidual30=0x140c,
	.v_empty=0xaa56,

	/* Cell Data */
	.rcomp0=0x0081,
	.tcompc0=0x403C,
	.cell_char_tbl = sony_cell_char_tbl,
};
static struct max77818_config_data sdi_config_data_new_board = {
	/* A/D measurement */
	.tgain=0xE259,
	.toff=0x2acc,

	.coff = 0x0000,
	/* Alert / Status */
	.config=0x0210,
	.config2=0x0050,

	/* App data */
	.full_soc_thresh=0x5f00,
	.design_cap=0x19b5,
	.ichgt_term=0x0300,

	/* MG3 config */
	.at_rate=0x1900,
	.learn_cfg=0x2602,
	.filter_cfg=0xcea4,
	.misc_cfg=0x09d0,
	.relax_cfg=0x2039,

	/* MG3 save and restore */
	.fullcapnom=0x19b4,
	.fullcaprep=0x19b4,
	.dqacc=0x066d,
	.dpacc=0x3200,
	.convgcfg=0x2271,
	.qresidual00=0x3400,
	.qresidual10=0x2080,
	.qresidual20=0x1680,
	.qresidual30=0x1280,
	.v_empty=0xaa56,

	/* Cell Data */
	.rcomp0=0x0078,
	.tcompc0=0x2a46,
	.cell_char_tbl = sdi_cell_char_tbl,
};
static struct max77818_config_data sdi_config_data_old_board = {
	/* A/D measurement */
	.tgain=0xE259,
	.toff=0x2acc,

	.coff = 0x0000,
	/* Alert / Status */
	.config=0x0210,
	.config2=0x0050,

	/* App data */
	.full_soc_thresh=0x5f00,
	.design_cap=0x19b5,
	.ichgt_term=0x0300,

	/* MG3 config */
	.at_rate=0x1900,
	.learn_cfg=0x2603,
	.filter_cfg=0xcc24,
	.misc_cfg=0x0810,
	.relax_cfg=0x2039,

	/* MG3 save and restore */
	.fullcapnom=0x19b4,
	.fullcaprep=0x19b4,
	.dqacc=0x066d,
	.dpacc=0x3200,
	.convgcfg=0x2271,
	.qresidual00=0x3400,
	.qresidual10=0x2080,
	.qresidual20=0x1680,
	.qresidual30=0x1280,
	.v_empty=0xaa56,

	/* Cell Data */
	.rcomp0=0x0078,
	.tcompc0=0x2a46,
	.cell_char_tbl = sdi_cell_char_tbl,
};



struct max77818_chip {
	struct device           *dev;
	struct max77818_dev     *max77818;
	struct regmap			*regmap;

	int 					fg_irq;
	struct power_supply		battery;

	/* alert */
	int alert_threshold;

	/* battery voltage */
	int vcell;
	/* battery capacity */
	int soc;
	/* State Of Charge */
	int status;
	/* battery health */
	int health;
	/* battery capacity */
	int capacity_level;

	struct max77818_fg_platform_data	*pdata;
	struct class *fg_class;
	struct device *fg_device;
	struct max77818_learned_params * learned_params;
	struct work_struct init_work;
	int init_complete;
	struct alarm alarm;
	struct work_struct alarm_work;
	bool critical_voltage;
	struct mutex lock;
	struct wake_lock battery_wake;
	bool en_clear;
	struct notifier_block fb_notifier;
	struct notifier_block reboot_notifier;
	bool rebooting;
};
enum battery_manufacturer{
	ATL=0,
	SONY=1,
	SDI = 2,
	NO_BAT=3,
	UNKNOWN=4,
};
#define ATL_BAT_ID 1354
#define SONY_BAT_ID 2001
#define SDI_BAT_ID 1733
#define NO_BATTERY_ID 3784
BLOCKING_NOTIFIER_HEAD(temp_notifier_list);
int register_temp_notifier(struct notifier_block *n)
{
	return blocking_notifier_chain_register(&temp_notifier_list,n);
}
int unregister_temp_notifier(struct notifier_block *n)
{
	return blocking_notifier_chain_unregister(&temp_notifier_list,n);
}

static enum battery_manufacturer get_battery_type(void)
{
	int adc_val = 0;

	adc_val = read_batid_adc();
	if(abs(adc_val-ATL_BAT_ID) < 100){
		return ATL;
	}else if(abs(adc_val-SONY_BAT_ID) < 100){
		return SONY;
	}else if(abs(adc_val-SDI_BAT_ID)<100){
		return SDI;
	}else if(adc_val>=3000){
		return NO_BAT;
	}else{
		return UNKNOWN;
	}

}
static void max77818_fg_get_soc(struct max77818_chip *max77818_fg)
{
	uint16_t soc;
	int rc;

	rc = max77818_fg_read(max77818_fg->regmap, MAX77818_RepSOC, &soc);
	if (rc < 0)
		dev_err(max77818_fg->dev, "%s: err %d\n", __func__, rc);
	else
		max77818_fg->soc = (uint16_t)soc >> 8;

	if ((soc & 0x00FF) * 39/1000 >4)
		max77818_fg->soc++;

	if (max77818_fg->soc > MAX77818_BATTERY_FULL) {
		max77818_fg->soc = MAX77818_BATTERY_FULL;
		max77818_fg->status = POWER_SUPPLY_STATUS_FULL;
		max77818_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		max77818_fg->health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (max77818_fg->soc < MAX77818_BATTERY_LOW) {
		max77818_fg->health = POWER_SUPPLY_HEALTH_DEAD;
		max77818_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		max77818_fg->health = POWER_SUPPLY_HEALTH_GOOD;
		max77818_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
}

static uint16_t max77818_fg_get_version(struct max77818_chip *max77818_fg)
{
	uint16_t version;
	int rc;
	rc = max77818_fg_read(max77818_fg->regmap, MAX77818_DevName, &version);
	if (rc < 0)
		dev_err(max77818_fg->dev, "%s: err %d\n", __func__, rc);

	return version;
}
static int max77818_fg_get_temp(struct max77818_chip *chip)
{
	int ret=0;
	u16 value=0;
	int result=0;

	ret = max77818_fg_read(chip->regmap, MAX77818_TEMP,&value);
	if (ret < 0){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		return -100;
	}

	/* The value is signed. */
	result = value;
	if (result & 0x8000) {
		result = (0x7fff & ~value) + 1;
		result *= -1;
	}
	/* The value is converted into deci-centigrade scale */
	/* Units of LSB = 1 / 256 degree Celsius */
	result *= 10;/*accurate to one decimal place*/
	result /= 256;
	return result;
}

static int max77818_fg_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max77818_chip *chip = container_of(psy,
			struct max77818_chip, battery);
	int ret=0;
	u16 value;
	static int result = 0;
	enum battery_manufacturer bat_type;

	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
			bat_type = get_battery_type();
			if(bat_type == NO_BAT){
				val->intval = 0;
			}else{
				val->intval = 1;
			}
			break;
		case POWER_SUPPLY_PROP_CYCLE_COUNT:
			ret = max77818_fg_read(chip->regmap, MAX77818_Cycles,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
			ret = max77818_fg_read(chip->regmap, MAX77818_MinMaxVolt,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			ret = value;
			val->intval = ret >> 8;
			val->intval *= 20000; /* Units of LSB = 20mV */
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			ret = max77818_fg_read(chip->regmap, MAX77818_V_empty,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value >> 7;
			val->intval *= 10000; /* Units of LSB = 10mV */
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			ret = max77818_fg_read(chip->regmap, MAX77818_VCELL,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value * 625 / 8;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_AVG:
			ret = max77818_fg_read(chip->regmap, MAX77818_AvgVCELL,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value * 625 / 8;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_OCV:
			ret = max77818_fg_read(chip->regmap, MAX77818_OCVInternal,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value * 625 / 8;
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			ret = max77818_fg_read(chip->regmap, MAX77818_FullCAP,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value * 1000 / 2;
			break;
		case POWER_SUPPLY_PROP_CHARGE_COUNTER:
			ret = max77818_fg_read(chip->regmap, MAX77818_QH,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value * 1000 / 2;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			result = max77818_fg_get_temp(chip);
			if(result == -100){
				pr_info(TAG"regmap fail\n");
				val->intval = 10;
			}else{
				val->intval = result;
			}
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			ret = max77818_fg_read(chip->regmap, MAX77818_Current,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value;
			if (val->intval & 0x8000) {
				/* Negative */
				val->intval = ~val->intval & 0x7fff;
				val->intval++;
				val->intval *= -1;
			}
			val->intval *= 156250;
			val->intval /= 1000000;
			break;
		case POWER_SUPPLY_PROP_CURRENT_AVG:
			ret = max77818_fg_read(chip->regmap, MAX77818_AvgCurrent,&value);
			if (ret < 0){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
				return ret;
			}

			val->intval = value;
			if (val->intval & 0x8000) {
				/* Negative */
				val->intval = ~val->intval & 0x7fff;
				val->intval++;
				val->intval *= -1;
			}
			val->intval *= 156250 ;
			val->intval /= 1000000;
			break;
		case POWER_SUPPLY_PROP_STATUS:
			max77818_fg_get_soc(chip);
			val->intval = chip->status;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			max77818_fg_get_soc(chip);
			val->intval = chip->soc;
			if(chip->critical_voltage)
				val->intval = 0;
			bat_type = get_battery_type();
			if(bat_type == NO_BAT){
				val->intval = 50;
			}
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			max77818_fg_get_soc(chip);
			val->intval = chip->health;
			break;
		case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
			max77818_fg_get_soc(chip);
			val->intval = chip->capacity_level;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
			break;
		case POWER_SUPPLY_PROP_MANUFACTURER:
			val->intval = get_battery_type();
			if(val->intval == ATL){
				val->strval = "ATL";
			}else if(val->intval == SONY){
				val->strval = "SONY";
			}else if(val->intval == SDI){
				val->strval = "SDI";
			}else{
				val->strval="UNKNOWN";
			}
			break;
		case POWER_SUPPLY_PROP_MODEL_NAME:
			val->strval="MEIZU";
			break;
		default:
			return -EINVAL;
	}
	return 0;
}
//FIXME: this interface is only for debug.
static int max77818_fg_set_property (struct power_supply *psy,
		enum power_supply_property psp, const union power_supply_propval *val)
{
	struct max77818_chip *chip = container_of(psy,
			struct max77818_chip, battery);
	u16 config;
	s8 temp;
	u16 temp_write;
	int rc;

	switch (psp) {
		case POWER_SUPPLY_PROP_TEMP:
			rc=max77818_fg_read(chip->regmap,MAX77818_CONFIG,&config);
			if(!(config & (1<<8))){
				pr_info("external false temperature is not enabled.please set it ahead\n");
				return 0;
			}
			temp = val->intval/10;
			pr_info("your  temp = %d\n",temp);
			temp_write = (u8)temp;
			temp_write <<=8;
			rc = max77818_fg_write(chip->regmap, MAX77818_TEMP,temp_write);
			if (rc){
				pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
				goto out;
			}
			break;
		default:
			rc = -EINVAL;
			goto out;
	}

out:
	return rc;
}
static void max77818_enable_dSOCen(struct max77818_chip *chip,bool en)
{
	u16 config2;
	int ret;
	ret = max77818_fg_read(chip->regmap,MAX77818_CONFIG2,&config2);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		return;
	}
	if(en)
		config2 |= 1<<7;
	else
		config2 &= ~(1<<7);
	ret = max77818_fg_write(chip->regmap,MAX77818_CONFIG2,config2);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		return;
	}
}
static int max77818_fg_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	switch(psp)
	{
		case POWER_SUPPLY_PROP_TEMP:
			return 1;
		default:
			return -EINVAL;
	}
}

static void max77818_set_talert(struct max77818_chip *chip, u8 max,u8 min)
{
	int ret;
	ret = max77818_fg_write(chip->regmap, MAX77818_TALRT_Th, (max<<8)|min);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		return;
	}
}

static void max77818_fg_mask_int(struct max77818_chip *chip, bool mask)
{
	u16 config;
	int ret;
	ret = max77818_fg_read(chip->regmap,MAX77818_CONFIG,&config);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		return;
	}
	if(mask){
		config &= ~4;
	}else{
		config |= 4;
	}
	ret = max77818_fg_write(chip->regmap,MAX77818_CONFIG,config);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		return;
	}
}

static bool max77818_fg_check_status(struct max77818_chip *max77818_fg)
{
	uint16_t data;
	int thermal;
	u16  vmin;
	u16 temp;
	int avg_current;
	int voltage;
	int ret;

	if ( max77818_fg_read(max77818_fg->regmap, MAX77818_STATUS, &data) < 0){
		pr_err("%s read fail\n",__func__);
		return true;
	}

	if(data==0)
		return false;

	/* clear status reg */
	if (max77818_fg_write(max77818_fg->regmap, MAX77818_STATUS, 0) < 0){
		pr_err("%s read fail\n",__func__);
		return true;
	}
	max77818_fg_get_soc(max77818_fg);
#if MAX77818_DEBUG_INFO
	if(max77818_fg->soc < 5){
		pr_info("current soc = %d, ", max77818_fg->soc);
		ret = max77818_fg_read(max77818_fg->regmap,MAX77818_VCELL, &temp);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		voltage = temp;
		voltage >>= 3;voltage *=625;
		pr_cont("current voltage = %d, ", voltage);
		ret = max77818_fg_read(max77818_fg->regmap,MAX77818_AvgVCELL, &temp);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		voltage = temp;
		voltage >>= 3;voltage *=625;
		pr_cont("average voltage = %d, ", voltage);
		ret = max77818_fg_read(max77818_fg->regmap, MAX77818_Cycles,&temp);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		pr_cont("cycle count = %d\n", temp);
	}else if(max77818_fg->soc > 95){
		pr_info("current soc = %d, ", max77818_fg->soc);
		ret = max77818_fg_read(max77818_fg->regmap, MAX77818_AvgCurrent,&temp);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		avg_current = temp;
		if (avg_current & 0x8000) {
			/* Negative */
			avg_current = ~avg_current & 0x7fff;
			avg_current++;
			avg_current *= -1;
		}
		avg_current *= 156250 ;
		avg_current /= 1000000;
		pr_cont("average current = %d, ",avg_current);

		ret = max77818_fg_read(max77818_fg->regmap, MAX77818_Cycles,&temp);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		pr_cont("cycle count = %d\n", temp);
		if(max77818_fg->soc == 100){
			blocking_notifier_call_chain(&temp_notifier_list,FULL_CAP,NULL);
		}
	}
#endif
	usleep_range(1000, 1500);
	if(data & BIT_TMN){
		thermal = max77818_fg_get_temp(max77818_fg)/10;
		pr_info("Tmin exceeded, ");
		pr_cont("current temperature = %d\n",thermal);
		if(thermal<=-20){
			pr_info("battery temperature is lower than -20. shutdown\n");
			//kernel_power_off();
			//shutdown();
		}else if(thermal<=0){
			//freeze_temp();
			blocking_notifier_call_chain(&temp_notifier_list,FREEZE_TEMP,NULL);
			max77818_set_talert(max77818_fg,0,-20);
		}else if(thermal<=10){
			//cold_temp();
			blocking_notifier_call_chain(&temp_notifier_list,COLD_TEMP,NULL);
			max77818_set_talert(max77818_fg,10,0);
		}else if(thermal<=45){
			//room_temp();
			blocking_notifier_call_chain(&temp_notifier_list,ROOM_TEMP,NULL);
			max77818_set_talert(max77818_fg,45,10);
		}
	}
	if(data & BIT_TMX){
		thermal = max77818_fg_get_temp(max77818_fg)/10;
		pr_info("Tmax exceeded, ");
		pr_cont("current temperature = %d\n",thermal);
		if(thermal >=60){
			pr_info("battery temperature is higher than 60. shutdown\n");
			//kernel_power_off();
			//shutdown();
			power_supply_changed(&max77818_fg->battery);
		}else if(thermal>=45){
			//hot_temp();
			blocking_notifier_call_chain(&temp_notifier_list,HOT_TEMP,NULL);
			max77818_set_talert(max77818_fg,60,45);
		}else if(thermal>=10){
			//room_temp();
			blocking_notifier_call_chain(&temp_notifier_list,ROOM_TEMP,NULL);
			max77818_set_talert(max77818_fg,45,10);
		}else if(thermal>=0){
			//cold_temp();
			blocking_notifier_call_chain(&temp_notifier_list,COLD_TEMP,NULL);
			max77818_set_talert(max77818_fg,10,0);
		}
	}
	if(data & BIT_SMN){
		pr_info("Smin exceeded soc=%d\n",max77818_fg->soc);
		if(max77818_fg->soc <=0){
			pr_info("soc = 0, shutdown & disable salert\n");
			ret = max77818_fg_write(max77818_fg->regmap, MAX77818_SALRT_Th, 0x0700);
			if(ret){
				pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
			}
		}else if(max77818_fg->soc <=7){
			//set salert 7-10
			pr_debug("set salert 0-7\n");
			ret = max77818_fg_write(max77818_fg->regmap, MAX77818_SALRT_Th, 0x0700);
			if(ret){
				pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
			}
		}else if(max77818_fg->soc <=10){
			//set salert 7%
			pr_debug("set salert to 7-10\n");
			ret = max77818_fg_write(max77818_fg->regmap, MAX77818_SALRT_Th, 0x0A07);
			if(ret){
				pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
			}
		}
	}
	if(data & BIT_SMX){
		pr_info("Smax exceeded\n");
		if(max77818_fg->soc >=10){
			//set salert 10%
			pr_debug("set salert to 10\n");
			ret = max77818_fg_write(max77818_fg->regmap, MAX77818_SALRT_Th, 0xFF0A);
			if(ret){
				pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
			}
		}else if(max77818_fg->soc >=7){
			//set salert 7-10
			pr_debug("set salert 7-10\n");
			ret = max77818_fg_write(max77818_fg->regmap, MAX77818_SALRT_Th, 0x0A07);
			if(ret){
				pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
			}
		}
	}
	usleep_range(1000, 1500);
	if(data & BIT_VMN){
		pr_info("Vmin exceeded, ");
		ret = max77818_fg_read(max77818_fg->regmap,MAX77818_VCELL, &temp);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		voltage = temp;
		voltage >>= 3;voltage *=625;
		pr_cont("current voltage = %d\n", voltage);
		ret = max77818_fg_read(max77818_fg->regmap,MAX77818_AvgVCELL, &temp);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		voltage = temp;
		voltage >>= 3;voltage *=625;
		pr_cont("average voltage = %d\n", voltage);
		if(voltage <= 3400000){
			max77818_fg->critical_voltage = true;
			ret = max77818_fg_write(max77818_fg->regmap, MAX77818_VALRT_Th, 0xAA00);
			if(ret){
				pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
			}
		}else{
			ret = max77818_fg_read(max77818_fg->regmap, MAX77818_VALRT_Th, &vmin);
			if(ret){
				pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
			}
			vmin &= 0x00FF;
			if(vmin > 5){
				ret = max77818_fg_write(max77818_fg->regmap, MAX77818_VALRT_Th, 0xAA00|(vmin-5));
				if(ret){
					pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
				}
			}
		}
	}
	if(data & BIT_VMX){
		pr_info("Vmax exceeded, ");
		ret = max77818_fg_read(max77818_fg->regmap,MAX77818_VCELL, &temp);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		voltage  = temp;
		voltage >>= 3;voltage *=625;
		pr_cont("current voltage = %d\n", voltage);
		if(voltage >= 3400000){
			max77818_fg->critical_voltage = false;
			ret = max77818_fg_write(max77818_fg->regmap, MAX77818_VALRT_Th, 0xFFAA);
			if(ret){
				pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
			}
		}
	}
	if(data & BIT_dSOCi){
		pr_debug("dSOC interrupt\n");
	}
	return true;
}

static irqreturn_t max77818_fg_irq_thread(int irq, void *irq_data)
{
	struct max77818_chip *fuelgauge = irq_data;
	bool data = true;

	mutex_lock(&fuelgauge->lock);
	data = max77818_fg_check_status(fuelgauge);
	if(data){
		power_supply_changed(&fuelgauge->battery);
	}
	mutex_unlock(&fuelgauge->lock);
	return IRQ_HANDLED;
}

static enum power_supply_property max77818_fg_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
};
static int max77818_write_verify_reg(struct max77818_chip *chip,u8 reg, u16 value)
{
	int retries = 8;
	int ret;
	u16 read_value;

	do {
		ret = max77818_fg_write(chip->regmap, reg, value);
		if(ret){
			pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		}
		msleep(3);
		ret =  max77818_fg_read(chip->regmap, reg,&read_value);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		if (read_value != value) {
			pr_info("%s verify fail retry, reg = 0x%x\n",__func__,reg);
			ret = -EIO;
			retries--;
		}
	} while (retries && read_value != value);

	if (ret < 0)
		pr_err(TAG"%s: err %d\n", __func__, ret);

	return ret;
}
//step 3: unlock model access
static inline void max77818_unlock_model(struct max77818_chip *chip)
{
	int ret;
	ret = max77818_fg_write(chip->regmap, MAX77818_MLOCKReg1, MODEL_UNLOCK1);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
	}
	ret = max77818_fg_write(chip->regmap, MAX77818_MLOCKReg2, MODEL_UNLOCK2);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
	}
}
//step 5: lock model access
static inline void max77818_lock_model(struct max77818_chip *chip)
{
	int ret;
	ret = max77818_fg_write(chip->regmap, MAX77818_MLOCKReg1, MODEL_LOCK1);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
	}
	ret = max77818_fg_write(chip->regmap, MAX77818_MLOCKReg2, MODEL_LOCK2);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
	}
}
//step 4.1: write model
static inline void max77818_write_model_data(struct max77818_chip *chip,
		u8 addr, int size)
{
	int i;
	for (i = 0; i < size; i++)
		max77818_fg_write(chip->regmap, addr + i,
				chip->pdata->config_data->cell_char_tbl[i]);
}
//step 4.2: read model
static inline void max77818_read_model_data(struct max77818_chip *chip,
		u8 addr, u16 *data, int size)
{
	int i;
	u16 val=0;

	for (i = 0; i < size; i++){
		max77818_fg_read(chip->regmap, addr + i,&val);
		data[i]=val;
	}
}
//step 4.3: verify model
static inline int max77818_model_data_compare(struct max77818_chip *chip,
		u16 *data1, u16 *data2, int size)
{
	int i;

	if (memcmp(data1, data2, size)) {
		pr_err(TAG "%s compare failed\n", __func__);
		for (i = 0; i < size; i++)
			pr_info(TAG "0x%x, 0x%x",
					data1[i], data2[i]);
		pr_info("\n");
		return -EINVAL;
	}
	return 0;
}
/*step 3-6*/
static int max77818_init_model(struct max77818_chip *chip)
{
	int ret;
	int table_size = MAX77818_CHARACTERIZATION_DATA_SIZE;
	u16 *temp_data;
	int retry_count = 3;
	int sum = 0,i = 0;

	temp_data = kcalloc(table_size, sizeof(*temp_data), GFP_KERNEL);
	if (!temp_data)
		return -ENOMEM;
	do{
		max77818_unlock_model(chip);//step 3
		max77818_write_model_data(chip, MAX77818_MODELChrTbl,
				table_size);//step 4.1
		max77818_read_model_data(chip, MAX77818_MODELChrTbl, temp_data,
				table_size);//step 4.2

		ret = max77818_model_data_compare(
				chip,
				chip->pdata->config_data->cell_char_tbl,
				temp_data,
				table_size);//step 4.3
	}while(ret && --retry_count);

	if(retry_count == 0){
		pr_err(TAG"max77818 fg model verify failed\n");
		return  -EIO;
	}

	retry_count = 3;
	do{
		max77818_lock_model(chip);//step 5
		max77818_read_model_data(chip, MAX77818_MODELChrTbl, temp_data,
				table_size);
		sum = 0;
		for(i = 0;i <table_size; i++){
			sum |= temp_data[i];
		}
	}while(sum && --retry_count);
	if(retry_count == 0){//step 6
		pr_err(TAG"max77818 fg model verify locked failed\n");
		return  -EIO;
	}

	kfree(temp_data);

	return ret;
}
static void  max77818_write_custom_regs(struct max77818_chip *chip)
{
	struct max77818_config_data *config = chip->pdata->config_data;
	int ret = 0;

	ret = max77818_write_verify_reg(chip,MAX77818_CONFIG,config->config);
	ret = max77818_write_verify_reg(chip,MAX77818_CONFIG2,config->config2);
	ret = max77818_write_verify_reg(chip,MAX77818_FullSOCthr,config->full_soc_thresh);
	ret = max77818_write_verify_reg(chip,MAX77818_FullCapRep,config->fullcaprep);
	ret = max77818_write_verify_reg(chip,MAX77818_DesignCap,config->design_cap);
	ret = max77818_write_verify_reg(chip,MAX77818_dPacc,config->dpacc);
	ret = max77818_write_verify_reg(chip,MAX77818_dQacc,config->dqacc);
	ret = max77818_write_verify_reg(chip,MAX77818_FullCapNom,config->fullcapnom);
	ret = max77818_write_verify_reg(chip,MAX77818_MiscCFG,config->misc_cfg);
	ret = max77818_write_verify_reg(chip,MAX77818_V_empty,config->v_empty);
	ret = max77818_write_verify_reg(chip,MAX77818_QResidual00,config->qresidual00);
	ret = max77818_write_verify_reg(chip,MAX77818_QResidual10,config->qresidual10);
	ret = max77818_write_verify_reg(chip,MAX77818_QResidual20,config->qresidual20);
	ret = max77818_write_verify_reg(chip,MAX77818_QResidual30,config->qresidual30);
	ret = max77818_write_verify_reg(chip,MAX77818_RCOMP0,config->rcomp0);
	ret = max77818_write_verify_reg(chip,MAX77818_TempCo,config->tcompc0);
	ret = max77818_write_verify_reg(chip,MAX77818_ICHGTerm,config->ichgt_term);
	ret = max77818_write_verify_reg(chip,MAX77818_TGAIN,config->tgain);
	ret = max77818_write_verify_reg(chip,MAx77818_TOFF,config->toff);
	ret = max77818_write_verify_reg(chip,MAX77818_COFF,config->coff);
	ret = max77818_write_verify_reg(chip,MAX77818_CURVE,0);
	ret = max77818_write_verify_reg(chip,SmartChgCfg,0x0);
	ret = max77818_write_verify_reg(chip,MAX77818_TALRT_Th2,0x3200);
	ret = max77818_write_verify_reg(chip,MAX77818_LearnCFG,config->learn_cfg);
	ret = max77818_write_verify_reg(chip,MAX77818_FilterCFG,config->filter_cfg);
	ret = max77818_write_verify_reg(chip,MAX77818_RelaxCFG,config->relax_cfg);
	ret = max77818_write_verify_reg(chip,MAX77818_AtRate,config->at_rate);
	if(config->cv_mixcap){
		ret = max77818_write_verify_reg(chip,MAX77818_CV_MixCap,config->cv_mixcap);
		ret = max77818_write_verify_reg(chip,MAX77818_HalfTime,config->cv_halftime);
	}

	//new parameters
	ret = max77818_fg_write(chip->regmap,MAX77818_VFSOC0Enable,VFSOC0_UNLOCK);
	ret = max77818_fg_write(chip->regmap,MAX77818_ConvgCfg,config->convgcfg);
	ret = max77818_fg_write(chip->regmap,MAX77818_VFSOC0Enable,VFSOC0_LOCK);
}
/*step 8~9*/
static void max77818_load_model(struct max77818_chip *chip)
{
	int ret;
	u16 val;

	//step 8:initiate model loading-new procedure
	ret = max77818_fg_read(chip->regmap,MAX77818_CONFIG2,&val);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
	}
	ret = max77818_fg_write(chip->regmap,MAX77818_CONFIG2,val| 0x0020);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
	}
	//step 9: verify model loading complete
	do{
		ret = max77818_fg_read(chip->regmap,MAX77818_CONFIG2,&val);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		msleep(20);
	}while((val & 0x0020) == 0);

}
static struct crypto_hash *tfm;
static struct scatterlist sg[1];
static struct hash_desc desc;
static const int sha1_digest_size = 20;

static char *do_digest(char * code,int len) {
	char *result;
	int code_len = len;

	tfm = crypto_alloc_hash("sha1", 0, CRYPTO_ALG_ASYNC);
	if(IS_ERR(tfm))
		return 0;
	sg_init_one(sg,code,code_len);
	desc.tfm = tfm;
	desc.flags = 0;

	result = (char *)kmalloc(sizeof(char)*sha1_digest_size,GFP_KERNEL);
	if(result == NULL) {
		crypto_free_hash(tfm);
		return 0;
	}
	memset(result,0,sizeof(char)*sha1_digest_size);
	crypto_hash_digest(&desc,sg,1,result);
	crypto_free_hash(tfm);

	return result;
}

extern int emmc_partition_rw(const char *part_name, int write, loff_t offset,void *buffer, size_t len);
static int save_learned_params2mmc(struct max77818_learned_params *learned_params)
{
	static struct max77818_learned_params temp;
	//invalidate learn params in mmc first
	memset(&temp,0,sizeof(struct max77818_learned_params));
	emmc_partition_rw("bat_model",1,0,&temp,sizeof(struct max77818_learned_params));
	//write-read-compare-verify
	emmc_partition_rw("bat_model",1,0,learned_params,sizeof(struct max77818_learned_params));
	emmc_partition_rw("bat_model",0,0,&temp,sizeof(struct max77818_learned_params));
	return memcmp(&temp,learned_params,sizeof(struct max77818_learned_params));
}
static int restore_learned_params2mmc(struct max77818_chip *chip)
{
	struct max77818_learned_params * learned_params = chip->learned_params;
	char *digest_result=NULL;
	emmc_partition_rw("bat_model",0,0,learned_params,sizeof(struct max77818_learned_params));
	if(learned_params->valid != 0x5a5a){
		pr_info(TAG"%s params invalid",__func__);
		return 1;
	}
	if( learned_params->battery_type != get_battery_type()){
		pr_info(TAG"%s battery changed\n",__func__);
		return 1;
	}
	if(learned_params->version != chip->pdata->bat_mod_version){
		pr_info(TAG"%s battery model updated\n",__func__);
		return 1;
	}
	digest_result = do_digest((char *)learned_params,sizeof(struct max77818_learned_params)-sha1_digest_size);
	if(memcmp( digest_result,learned_params->sha1,sha1_digest_size)){
		pr_err(TAG"%s params integrity fail\n",__func__);
		kfree(digest_result);
		return 2;
	}
	if(learned_params->saved_FullCapNom<0x1000 || learned_params->saved_FullCapNom > 0x1aff
			|| learned_params->saved_FullCapRep> 0x1aff || learned_params->saved_FullCapRep<0x1000){
		pr_err(TAG"%s full capacity is out of range full_cap_nom=0x%4x,full_cap_rep=0x%4x\n",__func__,
				learned_params->saved_FullCapNom,learned_params->saved_FullCapRep);
		return 1;
	}
	kfree(digest_result);
	return 0;
}
/*step 15*/
static int save_learned_parameters(struct max77818_chip *chip)
{
	struct max77818_learned_params * learned_params = chip->learned_params;
	char *digest_result=NULL;
	int ret = 0;

	ret = max77818_fg_read(chip->regmap,MAX77818_RCOMP0,&learned_params->saved_RCOMP0);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_fg_read(chip->regmap,MAX77818_TempCo,&learned_params->saved_TempCo);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_fg_read(chip->regmap,MAX77818_FullCapRep,&learned_params->saved_FullCapRep);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_fg_read(chip->regmap,MAX77818_Cycles,&learned_params->saved_Cycles );
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_fg_read(chip->regmap,MAX77818_FullCapNom,&learned_params->saved_FullCapNom);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_fg_read(chip->regmap,MAX77818_QResidual00,&learned_params->saved_QResidual00);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_fg_read(chip->regmap,MAX77818_QResidual10,&learned_params->saved_QResidual10);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_fg_read(chip->regmap,MAX77818_QResidual20,&learned_params->saved_QResidual20);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_fg_read(chip->regmap,MAX77818_QResidual30,&learned_params->saved_QResidual30);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	learned_params->valid = 0x5a5a;
	learned_params->battery_type = get_battery_type();
	learned_params->version = chip->pdata->bat_mod_version;
	digest_result = do_digest((char *)learned_params,sizeof(struct max77818_learned_params)-sha1_digest_size);
	memcpy(learned_params->sha1,digest_result,sha1_digest_size);
	kfree(digest_result);
	if((ret = save_learned_params2mmc(learned_params))){
		pr_err(TAG"%s failed\n",__func__);
	}
out:
	return ret;
}
/*step 16-19*/
static int restore_learned_parameters(struct max77818_chip *chip)
{
	struct max77818_learned_params * learned_params = chip->learned_params;
	int ret = 0;
	u16 dQ_acc = 0;

	if(restore_learned_params2mmc(chip)){
		pr_info("%s learned params invalide,skip restore\n",__func__);
		return 0;
	}

	pr_info(TAG"%s real restore\n",__func__);
	//step 16:restoring capacity parameters
	ret = max77818_write_verify_reg(chip,MAX77818_RCOMP0,learned_params->saved_RCOMP0);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_write_verify_reg(chip,MAX77818_TempCo,learned_params->saved_TempCo);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_write_verify_reg(chip,MAX77818_FullCapRep,learned_params->saved_FullCapRep);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_write_verify_reg(chip,MAX77818_Cycles,learned_params->saved_Cycles);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_write_verify_reg(chip,MAX77818_FullCapNom,learned_params->saved_FullCapNom);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_write_verify_reg(chip,MAX77818_QResidual00,learned_params->saved_QResidual00);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_write_verify_reg(chip,MAX77818_QResidual10,learned_params->saved_QResidual10);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_write_verify_reg(chip,MAX77818_QResidual20,learned_params->saved_QResidual20);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	ret = max77818_write_verify_reg(chip,MAX77818_QResidual30,learned_params->saved_QResidual30);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}

	//step 17:restoring dQacc & dPacc
	//write dQ_acc to 800% of Capacity and dP_acc to 800%
	dQ_acc = learned_params->saved_FullCapNom /4;
	ret = max77818_write_verify_reg(chip,MAX77818_dPacc,0x3200);
	ret = max77818_write_verify_reg(chip,MAX77818_dQacc,dQ_acc);
	//step 18-19
	max77818_load_model(chip);
out:
	return ret;
}
//enable external temperature
static void max77818_enable_extemp(struct max77818_chip *chip,bool en)
{
	int ret;
	u16 config;

	ret = max77818_fg_read(chip->regmap,MAX77818_CONFIG,&config);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
	if(en)
		config |= 1<<8;
	else
		config &= ~(1<<8);
	ret = max77818_fg_write(chip->regmap,MAX77818_CONFIG,config);
	if(ret){
		pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		goto out;
	}
out:
	return;
}
/*step 11*/
static int identify_battery(struct max77818_chip *chip)
{
	return restore_learned_parameters(chip);
}

static int max77818_fg_init_chip(struct max77818_chip *chip)
{
	int ret;
	uint16_t val;

	/*step 3~6: write cell characterization data */
	ret = max77818_init_model(chip);
	if (ret) {
		pr_err(TAG "%s init failed\n",__func__);
		return -EIO;
	}
	/*step 7:write custom parameters */
	max77818_write_custom_regs(chip);
	/*step 8~9: init complete*/
	max77818_load_model(chip);

	/*step 10: Init complete, Clear the POR bit */
	max77818_fg_read(chip->regmap, MAX77818_STATUS,&val);
	max77818_fg_write(chip->regmap, MAX77818_STATUS,val & (~STATUS_POR_BIT));
	/*step 11: identify battery*/
	identify_battery(chip);

	return 0;
}
static void max77818_init_worker(struct work_struct *work)
{
	struct max77818_chip *chip = container_of(work,
			struct max77818_chip, init_work);
	int ret;

	/* Initialize registers according to values from the platform data */
	if (chip->pdata->config_data) {
		ret = max77818_fg_init_chip(chip);
		if (ret)
			return;
	}
	chip->init_complete = 1;
	save_learned_parameters(chip);
	if(get_battery_type()==NO_BAT){
		pr_info("%s no battery exists.It's a developing board\n",__func__);
		//set temp external
		max77818_enable_extemp(chip,true);
		//write false temp to 25
		ret = max77818_fg_write(chip->regmap,MAX77818_TEMP,0x1900);
		if(ret){
			pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		}
		max77818_fg_mask_int(chip,true);
	}
	pr_info(TAG"fg reinit done\n");
}

//step 1.check for por
static int check_for_por(struct max77818_chip *chip)
{
	u16 reg;
	bool need_init=false;
	int ret;

	if(restore_learned_params2mmc(chip)){
		pr_info(TAG"%s reinit for new bat_mod version\n",__func__);
		need_init = true;
	}
	max77818_fg_read(chip->regmap, MAX77818_STATUS,&reg);
	if ((reg & STATUS_POR_BIT) || need_init) {//goto step 1-11
		pr_info(TAG"%s reinit\n",__func__);
		INIT_WORK(&chip->init_work, max77818_init_worker);
		schedule_work(&chip->init_work);
	} else {//goto step 13
		chip->init_complete = 1;
		save_learned_parameters(chip);
		pr_info(TAG"fg skip init\n");
	}
	if(get_battery_type()==NO_BAT){
		pr_info("%s no battery exists.It's a developing board\n",__func__);
		//set temp external
		max77818_enable_extemp(chip,true);
		//write false temp to 25
		ret = max77818_fg_write(chip->regmap,MAX77818_TEMP,0x1900);
		if(ret){
			pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		}
		max77818_fg_mask_int(chip,true);
	}else{
		max77818_set_talert(chip,45,10);
		//enable Aen
		max77818_fg_mask_int(chip,false);
		//set salert 10%
		ret = max77818_fg_write(chip->regmap, MAX77818_SALRT_Th, 0xFF0A);
		if(ret){
			pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		}
		//set valert 3.4V
		ret = max77818_fg_write(chip->regmap, MAX77818_VALRT_Th, 0xFFAA);
		//ret = max77818_fg_write(chip->regmap, MAX77818_VALRT_Th, 0xFF00);
		if(ret){
			pr_err(TAG"regmap_write fail in func %s line %d\n",__func__, __LINE__);
		}
	}
	return 0;
}

#ifdef CONFIG_OF
static int max77818_fg_parse_dt(struct max77818_chip *fuelgauge)
{
	struct device_node *np = of_find_node_by_name(NULL, "max77818-fuelgauge");
	struct max77818_fg_platform_data *pdata = NULL;
	int ret;
	u32 board_version;
	enum battery_manufacturer bat_type;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	fuelgauge->pdata = pdata;
	/* reset, irq gpio info */
	if (np == NULL) {
		pr_err("%s np NULL\n", __func__);
		return -1;
	} else {
		ret = of_property_read_u32(np, "fuelgauge,fuel_alert_soc",
				&pdata->soc_alert_threshold);
		if (ret < 0)
			pr_err("%s error reading pdata->fuel_alert_soc %d\n",
					__func__, ret);
		ret = of_property_read_u32(np, "bat_mod_version",
				&pdata->bat_mod_version);
		if (ret < 0)
			pr_err("%s error reading pdata->bat_mod_version %d\n",
					__func__, ret);
	}
	pdata->config_data = NULL;
	bat_type = get_battery_type();
	board_version = meizu_board_version();
	pr_info(TAG"current board version = %d\n",board_version);
	if(bat_type == ATL){
		pr_info(TAG"current battery ATL\n");
		if(board_version < 3){
			pdata->config_data = &atl_config_data_old_board;
		}else{
			pdata->config_data = &atl_config_data_new_board;
		}
	}else if(bat_type == SONY){
		pr_info(TAG"current battery SONY\n");
		if(board_version < 3){
			pdata->config_data = &sony_config_data_old_board;
		}else{
			pdata->config_data = &sony_config_data_new_board;
		}
	}else if(bat_type == SDI){
		pr_info(TAG"current battery SDI\n");
		if(board_version < 3){
			pdata->config_data = &sdi_config_data_old_board;
		}else{
			pdata->config_data = &sdi_config_data_new_board;
		}
	}
	return 0;
}
#endif
static ssize_t attr_set_reg(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	char rw[10];
	int reg,value,ret;
	struct max77818_chip *chip = dev_get_drvdata(dev);

	sscanf(buf,"%s %x %x",rw,&reg, &value);
	if(!strcmp(rw,"read")){
		ret=max77818_fg_read(chip->regmap, reg, (u16 *)(&value));
		pr_info(TAG"read from [%x] value = 0x%4x\n", reg, value);
	}
	else if(!strcmp(rw,"write")){
#if 1
		ret =max77818_fg_write(chip->regmap,reg,value);
		pr_info(TAG"write to [%x] value = 0x%4x\n", reg, value);
#else
		pr_info(TAG"write is disabled from userspace\n");
#endif
	}
	return size;
}
static ssize_t attr_dump_reg(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct max77818_chip *chip = dev_get_drvdata(dev);
	int i=0,offset=0,ret =0;
	u16 val;
	static char buf_bak[20];
	for(i=0;i<=0xff;i++){
		//skip these registers
		if((i > 0x4f && i < 0xb0) || (i > 0xbf && i < 0xd0))
			continue;
		ret=max77818_fg_read(chip->regmap, i, &val);
		if(ret){
			pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
		}
		sprintf(buf_bak,"0x%04x,",val);
		strcat(buf,buf_bak);
		offset +=strlen(buf_bak);
	}
	sprintf(buf_bak,"\n");
	strcat(buf,buf_bak);
	offset +=strlen(buf_bak);
	return offset;
}
static ssize_t attr_bat_adc(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int adc_val = 0;

	adc_val = read_batid_adc();
	pr_info(TAG"%s bat_adc = %d\n",__func__,adc_val);
	if(abs(adc_val-ATL_BAT_ID) < 100){
		pr_info(TAG"%s ATL\n",__func__);
	}else if(abs(adc_val-SONY_BAT_ID) < 100){
		pr_info(TAG"%s SONY\n",__func__);
	}else if(abs(adc_val-SDI_BAT_ID)<100){
		pr_info(TAG"%s SDI\n",__func__);
	}else if(adc_val>=3000){
		pr_info(TAG"%s NO_BAT\n",__func__);
	}else{
		pr_info(TAG"%s UNKNOWN\n",__func__);
	}
	return 0;
}

static ssize_t attr_cap_reg(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct max77818_chip *chip = dev_get_drvdata(dev);
	int offset=0,ret =0;
	u16 val;
	static char buf_bak[20];
	ret=max77818_fg_read(chip->regmap, MAX77818_RepCap, &val);
	if(ret){
		pr_err(TAG"regmap_read fail in func %s line %d\n",__func__, __LINE__);
	}
	sprintf(buf_bak,"%d\n",val/2);
	strcat(buf,buf_bak);
	offset +=strlen(buf_bak);
	return offset;
}

static ssize_t attr_clear_model(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	int en_clear=0;
	struct max77818_chip *chip = dev_get_drvdata(dev);
	static struct max77818_learned_params temp;

	sscanf(buf,"%d",&en_clear);
	chip->en_clear = en_clear;
	if(en_clear){
		memset(&temp,0,sizeof(struct max77818_learned_params));
		emmc_partition_rw("bat_model",1,0,&temp,sizeof(struct max77818_learned_params));
		pr_info("model cleared\n");
	}else{
		emmc_partition_rw("bat_model",0,0,&temp,sizeof(struct max77818_learned_params));
#if 1
		pr_info("learned_params->valid = 0x%4x\n",temp.valid);
		pr_info("learned_params->version = 0x%4x\n",temp.version);
		pr_info("learned_params->battery_type = 0x%4x\n",temp.battery_type);
		pr_info("learned_params->saved_Cycles = 0x%4x\n",temp.saved_Cycles);
		pr_info("learned_params->saved_FullCapNom = 0x%4x\n",temp.saved_FullCapNom);
		pr_info("learned_params->saved_FullCapRep = 0x%4x\n",temp.saved_FullCapRep);
		pr_info("learned_params->saved_QResidual00 = 0x%4x\n",temp.saved_QResidual00);
		pr_info("learned_params->saved_QResidual10 = 0x%4x\n",temp.saved_QResidual10);
		pr_info("learned_params->saved_QResidual20 = 0x%4x\n",temp.saved_QResidual20);
		pr_info("learned_params->saved_QResidual30 = 0x%4x\n",temp.saved_QResidual30);
		pr_info("learned_params->saved_RCOMP0 = 0x%4x\n",temp.saved_RCOMP0);
		pr_info("learned_params->saved_TempCo = 0x%4x\n",temp.saved_TempCo);
#endif
	}
	return size;
}


static struct device_attribute attributes[] = {
	__ATTR(reg_control, 0200, NULL, attr_set_reg),
	__ATTR(clear_model, 0200, NULL, attr_clear_model),
	__ATTR(dump_reg,0444,attr_dump_reg,NULL),
	__ATTR(bat_adc,0400,attr_bat_adc,NULL),
	__ATTR(cap_reg,0444,attr_cap_reg,NULL),
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
static dev_t const fg_device_dev_t = MKDEV(MISC_MAJOR, 245);
static void max77818_alarm_work(struct work_struct *work)
{
	struct max77818_chip *chip = container_of(work,
			struct max77818_chip, alarm_work);
	if(!chip->rebooting)
		save_learned_parameters(chip);
}

static enum alarmtimer_restart alarm_func(struct alarm *alarm, ktime_t time)
{
	struct max77818_chip *chip = container_of(alarm,struct max77818_chip, alarm);
	pr_info("%s alarm timer to save learned parameters\n",__func__);
	wake_lock_timeout(&chip->battery_wake,HZ*5);
	schedule_work(&chip->alarm_work);
	alarm_start_relative(&chip->alarm,ktime_set(SAVE_LEARN_TIME,0));
	return ALARMTIMER_NORESTART;
}
static int fb_event_notify(struct notifier_block *this, unsigned long code,
		void *data)
{
	struct max77818_chip * chip;
	struct fb_event *evdata = data;
	unsigned int blank;

	if (code != FB_EVENT_BLANK)
		return 0;

	blank = *(int *)(evdata->data);
	chip = container_of(this,struct max77818_chip,fb_notifier);

	switch(blank) {
		case FB_BLANK_POWERDOWN:
			max77818_enable_dSOCen(chip,false);
			break;
		case FB_BLANK_UNBLANK:
			power_supply_changed(&chip->battery);
			max77818_enable_dSOCen(chip,true);
			break;
	}
	return NOTIFY_OK;
}
static int reboot_event_notify(struct notifier_block *this, unsigned long code,
		void *unused)
{
	struct max77818_chip * chip;

	chip = container_of(this,struct max77818_chip,reboot_notifier);
	chip->rebooting = true;
	return NOTIFY_OK;
}

static int max77818_fg_probe(struct platform_device *pdev)
{
	struct max77818_dev *max77818 = dev_get_drvdata(pdev->dev.parent);
	struct max77818_fg_platform_data *pdata = NULL;
	struct max77818_chip *chip=NULL;
	uint16_t version;
	int ret = 0;

	pr_info("%s: MAX77818 Fuelgauge Driver Loading\n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->learned_params = kzalloc(sizeof(struct max77818_learned_params),GFP_KERNEL);
	if(!chip->learned_params){
		ret = -ENOMEM;
		goto error_alloc_chip;
	}
	chip->dev = &pdev->dev;
	chip->pdata = pdata;
	chip->regmap = max77818->regmap_fuel;
	platform_set_drvdata(pdev,chip);

#if defined(CONFIG_OF)
	ret = max77818_fg_parse_dt(chip);
	if (ret < 0) {
		pr_err("%s not found charger dt! ret[%d]\n",
				__func__, ret);
	}
#endif
	mutex_init(&chip->lock);
	wake_lock_init(&chip->battery_wake,WAKE_LOCK_SUSPEND,"battery_wake");

	version = max77818_fg_get_version(chip);
	if (version != MAX77818_VERSION_NO) {
		dev_err(&pdev->dev, "MAX77818 Fuel-Gauge Ver 0x%x not match\n", version);
		ret = -ENODEV;
		goto error;
	}

	chip->battery.name		= "battery";
	chip->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property	= max77818_fg_get_property;
	chip->battery.set_property = max77818_fg_set_property;
	chip->battery.property_is_writeable = max77818_fg_property_is_writeable;
	chip->battery.properties	= max77818_fg_battery_props;
	chip->battery.num_properties	= ARRAY_SIZE(max77818_fg_battery_props);

	ret = power_supply_register(&pdev->dev, &chip->battery);
	if (ret) {
		dev_err(&pdev->dev, "failed: power supply register\n");
		goto error;
	}

	chip->fg_irq = regmap_irq_get_virq(max77818->irqc_intsrc, MAX77818_FG_INT);

	if (chip->fg_irq > 0) {
		ret = request_threaded_irq(chip->fg_irq, NULL, max77818_fg_irq_thread,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, "fuelgauge-irq", chip);
		if (ret) {
			pr_err("%s: Failed to Reqeust IRQ\n", __func__);
			goto error1;
		}
	}
	ret = check_for_por(chip);
	chip->fg_class = class_create(THIS_MODULE,"fg_class");
	if(IS_ERR(chip->fg_class)){
		ret = PTR_ERR(chip->fg_class);
		goto err_class_create;
	}

	chip->fg_device = device_create(chip->fg_class,
			NULL,fg_device_dev_t,chip,"fg_device");
	if(IS_ERR(chip->fg_device)){
		ret = PTR_ERR(chip->fg_device);
		goto err_device_create;
	}

	ret = create_sysfs_files(chip->fg_device);
	if(ret < 0){
		goto err_create_sys;
	}
	INIT_WORK(&chip->alarm_work,max77818_alarm_work);
	alarm_init(&chip->alarm,ALARM_BOOTTIME,alarm_func);
	alarm_start_relative(&chip->alarm,ktime_set(SAVE_LEARN_TIME,0));
	chip->fb_notifier.notifier_call = fb_event_notify;
	ret = fb_register_client(&chip->fb_notifier);
	chip->reboot_notifier.notifier_call = reboot_event_notify;
	ret = register_reboot_notifier(&chip->reboot_notifier);

	return 0;
err_create_sys:
	device_destroy(chip->fg_class,fg_device_dev_t);
err_device_create:
	class_destroy(chip->fg_class);
err_class_create:
	power_supply_unregister(&chip->battery);
error1:
	power_supply_unregister(&chip->battery);
error:
	kfree(chip->learned_params);
error_alloc_chip:
	kfree(chip);
	return ret;
}

static int max77818_fg_remove(struct platform_device *pdev)
{
	struct max77818_chip *chip = platform_get_drvdata(pdev);

	power_supply_unregister(&chip->battery);
	mutex_destroy(&chip->lock);
	wake_lock_destroy(&chip->battery_wake);
	fb_unregister_client(&chip->fb_notifier);
	kfree(chip);
	return 0;
}

//#ifdef CONFIG_PM
#if 0
static int max77818_fg_suspend(struct device *dev)
{
	struct max77818_chip *chip = dev_get_drvdata(dev);
	max77818_enable_dSOCen(chip,false);
	return 0;
}

static int max77818_fg_resume(struct device *dev)
{
	struct max77818_chip *chip = dev_get_drvdata(dev);
	wake_lock_timeout(&chip->battery_wake,5*HZ);
	power_supply_changed(&chip->battery);
	max77818_enable_dSOCen(chip,true);
	return 0;
}
#else
#define max77818_fg_suspend NULL
#define max77818_fg_resume NULL
#endif /* CONFIG_PM */

#if defined(CONFIG_OF)
static struct of_device_id max77818_fg_dt_ids[] = {
	{ .compatible = "maxim,max77818-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77818_fg_dt_ids);
#endif /* CONFIG_OF */

static SIMPLE_DEV_PM_OPS(max77818_fg_pm_ops, max77818_fg_suspend,
		max77818_fg_resume);

static struct platform_driver max77818_fg_driver = {
	.driver = {
		.name = "max77818-fuelgauge",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &max77818_fg_pm_ops,
#endif
#if defined(CONFIG_OF)
		.of_match_table	= max77818_fg_dt_ids,
#endif /* CONFIG_OF */
	},
	.probe	= max77818_fg_probe,
	.remove	= max77818_fg_remove,
};
module_platform_driver(max77818_fg_driver);
MODULE_AUTHOR("TaiEup Kim <clark.kim@maximintegrated.com>");
MODULE_DESCRIPTION("MAX77818 Fuel Gauge");
MODULE_LICENSE("GPL");
