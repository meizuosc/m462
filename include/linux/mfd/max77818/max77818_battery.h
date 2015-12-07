/*
 * Copyright (C) 2014 Maximintegrated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAX77818_BATTERY_H_
#define __MAX77818_BATTERY_H_
#define MAX77818_STATUS_BattAbsent	(1 << 3)

#define MAX77818_CHARACTERIZATION_DATA_SIZE 48

enum max77818_register {
	MAX77818_STATUS		= 0x00,
	MAX77818_VALRT_Th	= 0x01,
	MAX77818_TALRT_Th	= 0x02,
	MAX77818_SALRT_Th	= 0x03,
	MAX77818_AtRate		= 0x04,
	MAX77818_RepCap		= 0x05,
	MAX77818_RepSOC		= 0x06,
	MAX77818_Age		= 0x07,
	MAX77818_TEMP		= 0x08,
	MAX77818_VCELL		= 0x09,
	MAX77818_Current	= 0x0A,
	MAX77818_AvgCurrent	= 0x0B,
	MAX77818_Qresidual= 0x0C,
	MAX77818_MixSOC=0x0D,
	MAX77818_AvSOC=0x0E,
	MAX77818_MixCap=0x0F,

	MAX77818_FullCAP	= 0x10,
	MAX77818_TTE		= 0x11,
	MAX77818_QResidual00	= 0x12,
	MAX77818_FullSOCthr	= 0x13,
	MAX77818_RSLOW		= 0x14,
	MAX77818_AvgTA		= 0x16,
	MAX77818_Cycles		= 0x17,
	MAX77818_DesignCap	= 0x18,
	MAX77818_AvgVCELL	= 0x19,
	MAX77818_MinMaxTemp	= 0x1A,
	MAX77818_MinMaxVolt	= 0x1B,
	MAX77818_MinMaxCurr	= 0x1C,
	MAX77818_CONFIG		= 0x1D,
	MAX77818_ICHGTerm	= 0x1E,
	MAX77818_AvCap		= 0x1F,

	MAX77818_TTF	= 0x20,
	MAX77818_DevName	= 0x21,
	MAX77818_QResidual10	= 0x22,
	MAX77818_FullCapNom	= 0x23,
	MAX77818_TempNom	= 0x24,
	MAX77818_TempLim	= 0x25,
	MAX77818_TempHot	= 0x26,
	MAX77818_AIN0		= 0x27,
	MAX77818_LearnCFG	= 0x28,
	MAX77818_FilterCFG	= 0x29,
	MAX77818_RelaxCFG	= 0x2A,
	MAX77818_MiscCFG	= 0x2B,
	MAX77818_TGAIN		= 0x2C,
	MAx77818_TOFF		= 0x2D,
	MAX77818_CGAIN		= 0x2E,
	MAX77818_COFF		= 0x2F,

	MAX77818_QResidual20	= 0x32,
	MAX77818_AtTTF	= 0x33,
	MAX77818_FullCapRep       = 0x35,
	MAX77818_LAvg_empty	= 0x36,
	MAX77818_FCTC		= 0x37,
	MAX77818_RCOMP0		= 0x38,
	MAX77818_TempCo		= 0x39,
	MAX77818_V_empty	= 0x3A,
	MAX77818_TIMER	= 0x3E,
	MAX77818_SHDNTIMER	= 0x3F,


	MAX77818_QResidual30	= 0x42,

	MAX77818_dQacc		= 0x45,
	MAX77818_dPacc		= 0x46,
	MAX77818_ConvgCfg	= 0x49,
	MAX77818_VFRemCap	= 0x4A,
	MAX77818_QH		= 0x4D,

	MAX77818_VFSOC0Enable	= 0x60,
	MAX77818_MLOCKReg1	= 0x62,
	MAX77818_MLOCKReg2	= 0x63,

	MAX77818_MODELChrTbl	= 0x80,
	MAX77818_Status2   = 0xB0,
	MAX77818_TALRT_Th2   = 0xB2,
	MAX77818_TTFCFG   = 0xB5,
	MAX77818_CV_MixCap = 0xB6,
	MAX77818_HalfTime = 0xB7,
	MAX77818_CGtempCo=0xB8,
	MAX77818_CURVE = 0xb9,
	MAX77818_CONFIG2   = 0xBB,

	MAX77818_OCV 	   = 0xEE,
	MAX77818_OCVInternal	= 0xFB,

	MAX77818_VFSOC		= 0xFF,
	SmartChgCfg = 0xDB,
	JEITA_VOLT = 0xd9,
	JEITA_CURR=0xda,
	ChargeState0 = 0xd1,
	ChargeState1 = 0xd2,
	ChargeState2 = 0xd3,
	ChargeState3 = 0xd4,
	ChargeState4 = 0xd5,
	ChargeState5 = 0xd6,
	ChargeState6 = 0xd7,
	ChargeState7 = 0xd8,
};

struct max77818_config_data {
	/* A/D measurement */
	u16	tgain;		/* 0x2C */
	u16	toff;		/* 0x2D */
	u16	cgain;		/* 0x2E */
	u16	coff;		/* 0x2F */

	/* Alert / Status */
	u16	valrt_thresh;	/* 0x01 */
	u16	talrt_thresh;	/* 0x02 */
	u16	soc_alrt_thresh;	/* 0x03 */
	u16	config;		/* 0x1D */
	u16  config2;		/*0xBB*/
	u16	shdntimer;	/* 0x03F */

	/* App data */
	u16	full_soc_thresh;	/* 0x13 */
	u16	design_cap;	/* 0x18 */
	u16	ichgt_term;	/* 0x1E */

	/* MG3 config */
	u16	at_rate;	/* 0x04 */
	u16	learn_cfg;	/* 0x28 */
	u16	filter_cfg;	/* 0x29 */
	u16	relax_cfg;	/* 0x2A */
	u16	misc_cfg;	/* 0x2B */
	u16	masksoc;	/* 0x32 */
	u16 cv_mixcap; /* 0xb6 */
	u16 cv_halftime; /*0xb7*/

	/* MG3 save and restore */
	u16	fullcap;	/* 0x10 */
	u16	fullcapnom;	/* 0x23 */
	u16	socempty;	/* 0x33 */
	u16	fullcaprep;	/*0x35*/
	u16	lavg_empty;	/* 0x36 */
	u16	dqacc;		/* 0x45 */
	u16	dpacc;		/* 0x46 */
	u16	convgcfg;		/*0x49*/
	u16	qresidual00;		/* 0x12 */
	u16	qresidual10;		/* 0x22 */
	u16	qresidual20;		/* 0x32 */
	u16	qresidual30;		/* 0x42 */
	u16	v_empty;	/*0x3A*/

	/* Cell technology from power_supply.h */
	u16	cell_technology;

	/* Cell Data */
	u16	temp_nom;	/* 0x24 */
	u16	temp_lim;	/* 0x25 */
	u16	fctc;		/* 0x37 */
	u16	rcomp0;		/* 0x38 */
	u16	tcompc0;	/* 0x39 */
	u16	empty_tempco;	/* 0x3A */
	u16	kempty0;	/* 0x3B */
	u16	*cell_char_tbl;
} __packed;

struct max77818_fg_platform_data {
	struct max77818_config_data *config_data;
	bool enable_por_init; /* Use POR init from Maxim appnote */

	int	soc_alert_threshold;
	int	bat_mod_version;
};
struct max77818_learned_params{
	u16 saved_Cycles; /*0x17*/
	u16 saved_FullCapNom; /*0x23*/
	u16 saved_QResidual00; /*0x12*/
	u16 saved_QResidual10; /*0x22*/
	u16 saved_QResidual20; /*0x32*/
	u16 saved_QResidual30; /*0x42*/
	u16 valid;
	u16 battery_type;
	u16 version;
	u16 saved_RCOMP0; /*0x38*/
	u16 saved_TempCo; /*0x39*/
	u16 saved_FullCapRep; /*0x35*/
	u16 sha1[10];
};
#endif
