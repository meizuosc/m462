/*
* Simple driver for M7X lcd panel
* Copyright (C) 2014 MEIZU
* Author: wangbo@meizu.com
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <video/mipi_display.h>
#include <linux/platform_device.h>

#include "../decon_display/decon_mipi_dsi.h"

/* Tmp used for touch panel during the development phase.
  * Define:
  *        (0 << 16) :  JDI
  *        (1 << 16) :  SHARP
  */
int meizu_lcd_type;

enum {
	LCD_ID_CODE1_INDEX = 0,
	LCD_ID_CODE2_INDEX,
	LCD_ID_CODE3_INDEX,
	MAX_LCD_ID_CODE_INDEX,
};
enum {
	LCD_ID_CODE1 = 0xDA,
	LCD_ID_CODE2,
	LCD_ID_CODE3,
};
static int lcd_id[MAX_LCD_ID_CODE_INDEX] = {0};
static char *lcd_desc;
static char lcd_desc_data[15] = "Unknown ID";
static int lcd_power_type = POWER_SUPPLY_TYPE_0;
static int lcd_version = LCD_VER_0;
module_param_array(lcd_id, int, NULL, S_IRUGO | S_IWUSR | S_IWGRP);
module_param(lcd_desc, charp, 0444);
module_param(lcd_power_type, int, 0444);
module_param(lcd_version, int, 0444);
module_param(meizu_lcd_type, int, 0444);

struct MZ_ID_S {
	unsigned int  type;
	unsigned char id1;
	unsigned char id2;
	unsigned int  tp_type;
	char          tag[10];
};
static struct MZ_ID_S mz_id[] = {
	[0] = {0, 0, 0,  3, "J01"},
	[1] = {0, 0, 1,  3, "J03"},
	[2] = {0, 1, 67, 3, "J04"},
	[3] = {0, 1, 68, 2, "J05"},
	[4] = {1, 67, 1, 2, "ST01"},
	[5] = {1, 66, 2, 3, "S01"},
};

struct lcd_cmd {
	unsigned int type;
	unsigned char *data;
	unsigned int size;
	unsigned int delay;
};

#define INIT_CMD(cmd, type_v, delay_v) \
	static struct lcd_cmd cmd##_s = { \
		.type = type_v, \
		.data = cmd, \
		.size = ARRAY_SIZE(cmd), \
		.delay = delay_v, \
	}

#define WRITE_LCD(dsim, cmd) \
	do { \
		switch (cmd##_s.type) { \
		case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM: \
		case MIPI_DSI_DCS_SHORT_WRITE: \
		case MIPI_DSI_DCS_SHORT_WRITE_PARAM: \
			if (s5p_mipi_dsi_wr_data(dsim, cmd##_s.type, cmd##_s.data[0], cmd##_s.data[1])) \
				pr_err("Error: failed to write %s command!\r\n", #cmd); \
			break; \
		default: \
			if (s5p_mipi_dsi_wr_data(dsim, cmd##_s.type, (unsigned int)cmd##_s.data, cmd##_s.size)) \
				pr_err("Error: failed to write %s command!\r\n", #cmd); \
			break; \
		}; \
		if (cmd##_s.delay) \
			usleep_range(cmd##_s.delay * 1000, cmd##_s.delay * 1000); \
	} while(0)

#define READ_LCD(dsim, addr, count, buf) \
	do { \
		s5p_mipi_dsi_rd_data(dsim, MIPI_DSI_DCS_READ, addr, count, (unsigned char *)buf, 1); \
	} while (0)

static unsigned char TEON[]	 = {0x35, 0x00};
static unsigned char SLEEP_OUT[] = {0x11, 0x00};
static unsigned char DISP_ON[]	 = {0x29, 0x00};
static unsigned char DISP_OFF[]  = {0x28, 0x00};
static unsigned char SLEEP_IN[]  = {0x10, 0x00};
static unsigned char WA[]        = {0xB9, 0x10, 0x09, 0xFC, 0x04}; //2556 [2554 ~ 2562]

static unsigned char J_PASSWD3_0_V1[] = {0xFC, 0x5A, 0x5A};
static unsigned char J_G_PARA_V1[]    = {0xB0, 0x06};
static unsigned char J_MSCAN_V1[]     = {0xFE, 0x12};
static unsigned char J_PASSWD3_1_V1[] = {0xFC, 0xA5, 0xA5};
static unsigned char J_CASET_V1[]     = {0x2A, 0x00, 0x00, 0x05, 0xFF};
static unsigned char J_PASET_V1[]     = {0x2B, 0x00, 0x00, 0x09, 0xFF};
static unsigned char J_MADCTL_V1[]    = {0x36, 0x00};
static unsigned char J_E8_V1[]    = {0xE8, 0x22};
static unsigned char J_EA_V1[]    = {0xEA, 0x12, 0x33, 0x03, 0x25};


static unsigned char J_PASSWD1_0_V2[] = {0xF0, 0x5A, 0x5A};
static unsigned char J_PASSWD2_0_V2[] = {0xF1, 0x5A, 0x5A};
static unsigned char J_DISPTL_V2[]    = {0xB3, 0x02, 0x23, 0x40, 0x00, 0xEB};
static unsigned char J_PECTRL_2_V2[]  = {0xEC, 0x04, 0x08, 0x00, 0x34, 0x15, 0x12, 0x08,
	                                 0x1D, 0x02, 0x02, 0x6C, 0x02, 0x00, 0x00};
static unsigned char J_G_PARA_0_V2[]  = {0xB0, 0x09};
static unsigned char J_SRCRTL_V2[]    = {0xF2, 0x46};
static unsigned char J_PASSWD1_1_V2[] = {0xF0, 0xA5, 0xA5};
static unsigned char J_PASSWD2_1_V2[] = {0xF1, 0xA5, 0xA5};
static unsigned char J_PASSWD3_0_V2[] = {0xFC, 0x5A, 0x5A};
static unsigned char J_G_PARA_1_V2[]  = {0xB0, 0x06};
static unsigned char J_MSCAN_V2[]     = {0xFE, 0x12};
static unsigned char J_PASSWD3_1_V2[] = {0xFC, 0xA5, 0xA5};
static unsigned char J_CASET_V2[]     = {0x2A, 0x00, 0x00, 0x05, 0xFF};
static unsigned char J_PASET_V2[]     = {0x2B, 0x00, 0x00, 0x09, 0xFF};
static unsigned char J_MADCTL_V2[]    = {0x36, 0x00};

static unsigned char S_PASSWD_KEY_EN_0[]   = {0xF0, 0x5A, 0x5A};
static unsigned char S_PASSWD_KEY_EN_1[]   = {0xF1, 0x5A, 0x5A};
static unsigned char S_PASSWD_KEY_EN_2[]   = {0xFC, 0x5A, 0x5A};
static unsigned char S_INTERFACE_SETTING[] = {0xB4, 0xE0, 0x08, 0x00};
static unsigned char S_DDI_GUIDE_CODE_0[]  = {0xB0, 0x06};
static unsigned char S_DDI_GUIDE_CODE_1[]  = {0xFE, 0x12};
static unsigned char S_PASSWD_KEY_DIS_0[]  = {0xF0, 0xA5, 0xA5};
static unsigned char S_PASSWD_KEY_DIS_1[]  = {0xF1, 0xA5, 0xA5};
static unsigned char S_PASSWD_KEY_DIS_2[]  = {0xFC, 0xA5, 0xA5};
static unsigned char S_CASET[]             = {0x2A, 0x00, 0x00, 0x05, 0xFF};
static unsigned char S_PASET[]             = {0x2B, 0x00, 0x00, 0x09, 0xFF};
static unsigned char S_G_PARA_SET[]        = {0xB0, 0x0B};
static unsigned char S_PANEL_CTL1[]        = {0xEB, 0x1C};


INIT_CMD(TEON, 0x15, 0);
INIT_CMD(SLEEP_OUT, 0x05, 120);
INIT_CMD(DISP_ON, 0x05, 0);
INIT_CMD(DISP_OFF, 0x05, 0);
INIT_CMD(SLEEP_IN, 0x05, 120);
INIT_CMD(WA, 0x39, 0);

INIT_CMD(J_PASSWD3_0_V1, 0x29, 0);
INIT_CMD(J_G_PARA_V1, 0x29, 0);
INIT_CMD(J_MSCAN_V1, 0x29, 0);
INIT_CMD(J_PASSWD3_1_V1, 0x29, 0);
INIT_CMD(J_CASET_V1, 0x39, 0);
INIT_CMD(J_PASET_V1, 0x39, 0);
INIT_CMD(J_MADCTL_V1, 0x15, 0);
INIT_CMD(J_E8_V1, 0x15, 0);
INIT_CMD(J_EA_V1, 0x29, 0);

INIT_CMD(J_PASSWD1_0_V2, 0x29, 0);
INIT_CMD(J_PASSWD2_0_V2, 0x29, 0);
INIT_CMD(J_DISPTL_V2, 0x29, 0);
INIT_CMD(J_PECTRL_2_V2, 0x29, 0);
INIT_CMD(J_G_PARA_0_V2, 0x29, 0);
INIT_CMD(J_SRCRTL_V2, 0x29, 0);
INIT_CMD(J_PASSWD1_1_V2, 0x29, 0);
INIT_CMD(J_PASSWD2_1_V2, 0x29, 0);
INIT_CMD(J_PASSWD3_0_V2, 0x29, 0);
INIT_CMD(J_G_PARA_1_V2, 0x29, 0);
INIT_CMD(J_MSCAN_V2, 0x29, 0);
INIT_CMD(J_PASSWD3_1_V2, 0x29, 0);
INIT_CMD(J_CASET_V2, 0x39, 0);
INIT_CMD(J_PASET_V2, 0x39, 0);
INIT_CMD(J_MADCTL_V2, 0x15, 0);

INIT_CMD(S_PASSWD_KEY_EN_0, 0x29, 0);
INIT_CMD(S_PASSWD_KEY_EN_1, 0x29, 0);
INIT_CMD(S_PASSWD_KEY_EN_2, 0x29, 0);
INIT_CMD(S_INTERFACE_SETTING, 0x29, 0);
INIT_CMD(S_DDI_GUIDE_CODE_0, 0x29, 0);
INIT_CMD(S_DDI_GUIDE_CODE_1, 0x29, 0);
INIT_CMD(S_PASSWD_KEY_DIS_0, 0x29, 0);
INIT_CMD(S_PASSWD_KEY_DIS_1, 0x29, 0);
INIT_CMD(S_PASSWD_KEY_DIS_2, 0x29, 0);
INIT_CMD(S_CASET, 0x39, 0);
INIT_CMD(S_PASET, 0x39, 0);
INIT_CMD(S_G_PARA_SET, 0x29, 0);
INIT_CMD(S_PANEL_CTL1, 0x29, 20);


static int m7x_displayon(struct mipi_dsim_device *dsim)
{
	return 1;
}

static int m7x_init(struct mipi_dsim_device *dsim)
{
	if (dsim->power_supply_flags == POWER_SUPPLY_TYPE_0) {
		switch (dsim->lcd_ver) {
		case LCD_VER_0:
			WRITE_LCD(dsim, SLEEP_OUT);
			WRITE_LCD(dsim, J_PASSWD1_0_V2);
			WRITE_LCD(dsim, J_PASSWD2_0_V2);
			WRITE_LCD(dsim, J_PASSWD3_0_V1);
			WRITE_LCD(dsim, J_G_PARA_V1);
			WRITE_LCD(dsim, J_MSCAN_V1);
			WRITE_LCD(dsim, J_E8_V1);
			WRITE_LCD(dsim, J_EA_V1);
			WRITE_LCD(dsim, WA);
			WRITE_LCD(dsim, J_PASSWD1_1_V2);
			WRITE_LCD(dsim, J_PASSWD2_1_V2);
			WRITE_LCD(dsim, J_PASSWD3_1_V1);
			WRITE_LCD(dsim, J_CASET_V1);
			WRITE_LCD(dsim, J_PASET_V1);
			WRITE_LCD(dsim, J_MADCTL_V1);
			WRITE_LCD(dsim, TEON);
			WRITE_LCD(dsim, DISP_ON);
			break;

		case LCD_VER_1:
			WRITE_LCD(dsim, SLEEP_OUT);
			WRITE_LCD(dsim, J_PASSWD1_0_V2);
			WRITE_LCD(dsim, J_PASSWD2_0_V2);
			WRITE_LCD(dsim, J_DISPTL_V2);
			WRITE_LCD(dsim, J_PECTRL_2_V2);
			WRITE_LCD(dsim, J_G_PARA_0_V2);
			WRITE_LCD(dsim, J_SRCRTL_V2);
			WRITE_LCD(dsim, J_E8_V1);
			WRITE_LCD(dsim, J_EA_V1);
			WRITE_LCD(dsim, WA);
			WRITE_LCD(dsim, J_PASSWD1_1_V2);
			WRITE_LCD(dsim, J_PASSWD2_1_V2);
			WRITE_LCD(dsim, J_PASSWD3_0_V2);
			WRITE_LCD(dsim, J_G_PARA_1_V2);
			WRITE_LCD(dsim, J_MSCAN_V2);
			WRITE_LCD(dsim, J_PASSWD3_1_V2);
			WRITE_LCD(dsim, J_CASET_V2);
			WRITE_LCD(dsim, J_PASET_V2);
			WRITE_LCD(dsim, J_MADCTL_V2);
			WRITE_LCD(dsim, TEON);
			WRITE_LCD(dsim, DISP_ON);
			break;
		};
	} else {
		WRITE_LCD(dsim, SLEEP_OUT);
		WRITE_LCD(dsim, S_PASSWD_KEY_EN_0);
		WRITE_LCD(dsim, S_PASSWD_KEY_EN_1);
		WRITE_LCD(dsim, S_PASSWD_KEY_EN_2);
		WRITE_LCD(dsim, S_INTERFACE_SETTING);
		WRITE_LCD(dsim, S_G_PARA_SET);
		WRITE_LCD(dsim, S_PANEL_CTL1);
		WRITE_LCD(dsim, S_DDI_GUIDE_CODE_0);
		WRITE_LCD(dsim, S_DDI_GUIDE_CODE_1);
		WRITE_LCD(dsim, J_E8_V1);
		WRITE_LCD(dsim, J_EA_V1);
		WRITE_LCD(dsim, WA);
		WRITE_LCD(dsim, S_PASSWD_KEY_DIS_0);
		WRITE_LCD(dsim, S_PASSWD_KEY_DIS_1);
		WRITE_LCD(dsim, S_PASSWD_KEY_DIS_2);
		WRITE_LCD(dsim, S_CASET);
		WRITE_LCD(dsim, S_PASET);
		WRITE_LCD(dsim, DISP_ON);
		WRITE_LCD(dsim, TEON);
	}

	return 0;
}

static int m7x_read_id(struct mipi_dsim_device *dsim, unsigned int id_pin)
{
	int ret;
	ret = gpio_request(id_pin, "lcd_id");
	if (ret < 0) {
		pr_err("Failed to get gpio number for the lcd id [%d].\n", id_pin);
		return -EINVAL;
	}

	if(gpio_get_value(id_pin)) {
		dsim->power_supply_flags = POWER_SUPPLY_TYPE_1;
		meizu_lcd_type = (1 << 16);
		lcd_power_type = POWER_SUPPLY_TYPE_1;
	} else {
		dsim->power_supply_flags = POWER_SUPPLY_TYPE_0;
		meizu_lcd_type = (0 << 16);
		lcd_power_type = POWER_SUPPLY_TYPE_0;
	}

	printk("[Info]: Lcd supply to power class %d!\r\n", dsim->power_supply_flags);

	gpio_free(id_pin);
	return 0;
}

static int m7x_read_ver(struct mipi_dsim_device *dsim)
{
	int i;
	struct MZ_ID_S ID;
	struct MZ_ID_S *pID = NULL;

	for (i = LCD_ID_CODE1_INDEX; i < MAX_LCD_ID_CODE_INDEX; i++) {
		READ_LCD(dsim, (LCD_ID_CODE1 + i), 1, &lcd_id[i]);
	}

	switch (dsim->power_supply_flags) {
	case POWER_SUPPLY_TYPE_0:
		if ((lcd_id[LCD_ID_CODE1_INDEX] == mz_id[3].id1) 
			&& (lcd_id[LCD_ID_CODE2_INDEX] == mz_id[3].id2)) {
			dsim->lcd_ver = LCD_VER_1;
			lcd_version = LCD_VER_1;
		} else {
			dsim->lcd_ver = LCD_VER_0;
			lcd_version = LCD_VER_0;
		}
		break;

	case POWER_SUPPLY_TYPE_1:
	default:
		dsim->lcd_ver = LCD_VER_0;
		lcd_version = LCD_VER_0;
		break;
	};

	for (i = 0; i < ARRAY_SIZE(mz_id); i++) {
		if ((dsim->power_supply_flags == mz_id[i].type)
			&& (lcd_id[LCD_ID_CODE1_INDEX] == mz_id[i].id1)
			&& (lcd_id[LCD_ID_CODE2_INDEX] == mz_id[i].id2)) {
			pID = &mz_id[i];
			break;
		}
	}
	if ((pID == NULL) && ((lcd_id[LCD_ID_CODE2_INDEX] & 0xC0) == 0x80)) {
		if ((lcd_id[LCD_ID_CODE1_INDEX] & 0xC0) == 0x00) {
			ID.tp_type = 1;
			sprintf(ID.tag, "JM%d", (lcd_id[LCD_ID_CODE2_INDEX] & 0x1F));
			pID = &ID;
		} else if ((lcd_id[LCD_ID_CODE1_INDEX] & 0xC0) == 0x40) {
			ID.tp_type = 1;
			sprintf(ID.tag, "SM%d", (lcd_id[LCD_ID_CODE2_INDEX] & 0x1F));
			pID = &ID;
		}
	}

	if (pID != NULL) {
		memcpy(lcd_desc_data, pID->tag, sizeof(pID->tag));
	}
	lcd_desc = lcd_desc_data;

	meizu_lcd_type |= ((pID != NULL) ? pID->tp_type : 3);

	printk("[Info]: Lcd version is VER-%d [%d,%d,%d - %s] [TP: 0x%08x]\r\n", dsim->lcd_ver,
		lcd_id[LCD_ID_CODE1_INDEX],
		lcd_id[LCD_ID_CODE2_INDEX],
		lcd_id[LCD_ID_CODE3_INDEX],
		((pID != NULL) ? pID->tag : "UNKNOW ID"),
		meizu_lcd_type);

	return 0;
}

static int m7x_probe(struct mipi_dsim_device *dsim)
{
	return 1;
}

static int m7x_suspend(struct mipi_dsim_device *dsim)
{
	if (dsim->power_supply_flags == POWER_SUPPLY_TYPE_1) {
		WRITE_LCD(dsim, S_PASSWD_KEY_EN_0);
		WRITE_LCD(dsim, S_PASSWD_KEY_EN_1);
		WRITE_LCD(dsim, S_PASSWD_KEY_EN_2);
		WRITE_LCD(dsim, S_G_PARA_SET);
		WRITE_LCD(dsim, S_PANEL_CTL1);
	} else {
		WRITE_LCD(dsim, DISP_OFF);
		WRITE_LCD(dsim, SLEEP_IN);
	}

	return 1;
}

static int m7x_resume(struct mipi_dsim_device *dsim)
{
	return 1;
}

struct mipi_dsim_lcd_driver m7x_mipi_lcd_driver = {
	.probe		= m7x_probe,
	.read_id        = m7x_read_id,
	.read_ver       = m7x_read_ver,
	.init           = m7x_init,
	.displayon	= m7x_displayon,
	.suspend	= m7x_suspend,
	.resume		= m7x_resume,
};

MODULE_DESCRIPTION("MEIZU M7X Lcd panel driver");
MODULE_AUTHOR("WangBo <wangbo@meizu.com>");
MODULE_LICENSE("GPL v2");
