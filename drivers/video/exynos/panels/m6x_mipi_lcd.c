/*
* Simple driver for lcd panel
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

#define PP_NARG(...) \
    PP_NARG_(__VA_ARGS__,PP_RSEQ_N())
#define PP_NARG_(...) \
    PP_ARG_N(__VA_ARGS__)
#define PP_ARG_N( \
     _1, _2, _3, _4, _5, _6, _7, _8, _9,_10, \
    _11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
    _21,_22,_23,_24,_25,_26,_27,_28,_29,_30, \
    _31,_32,_33,_34,_35,_36,_37,_38,_39,_40,_41,_42,\
    _43,_44,_45,_46,_47,_48,N, ...) N

#define PP_RSEQ_N() \
    48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,\
    29,28,27,26,25,24,23,22,21,20, \
    19,18,17,16,15,14,13,12,11,10, \
     9, 8, 7, 6, 5, 4, 3, 2, 1, 0

#define DCS_SHORT(mdelay, ...) {\
	.param = {__VA_ARGS__},\
	.delay = mdelay,\
	.type = MIPI_DSI_DCS_SHORT_WRITE,}
#define DCS_SHORT_PARAM(...) {\
	.param = {__VA_ARGS__},\
	.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM,}
#define DCS_SHORT_PARAM_D(mdelay, ...) {\
	.delay = mdelay,\
	.param = {__VA_ARGS__},\
	.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM,}

#define DCS_LONG( ...) {\
	.param = {__VA_ARGS__},\
	.size = PP_NARG(__VA_ARGS__),\
	.type = MIPI_DSI_DCS_LONG_WRITE,}

#define GCS_SHORT_PARAM(mdelay, ...) {\
	.delay = mdelay,\
	.param = {__VA_ARGS__},\
	.type = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,}

#define GCS_LONG(mdelay, ...) {\
	.delay = mdelay,\
	.param = {__VA_ARGS__},\
	.size = PP_NARG(__VA_ARGS__),\
	.type = MIPI_DSI_GENERIC_LONG_WRITE,}

#define GCS_SHORT_PARAM_0(...) {\
	.param = {__VA_ARGS__},\
	.type = MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM,}

#define GCS_LONG_0(...) {\
	.param = {__VA_ARGS__},\
	.size = PP_NARG(__VA_ARGS__),\
	.type = MIPI_DSI_GENERIC_LONG_WRITE,}

#define LCD_PARAM_DEF_END {.size = -1,}

#define write_cmd(dsim, type, cmd0, cmd1) \
	s5p_mipi_dsi_wr_data(dsim, type, cmd0, cmd1)

#define write_data(dsim, type, array, size)	\
	s5p_mipi_dsi_wr_data(dsim, type, (unsigned int)array, size)

#define read_data(dsim, addr, count, buf) \
	s5p_mipi_dsi_rd_data(dsim, MIPI_DSI_DCS_READ, addr, count, buf)

#define sat_lit_low  0x0F,0x0F,0x0F,0x0F,0x0F,0x0F
#define sat_lit_med  0x1F,0x1F,0x1F,0x1F,0x1F,0x1F
#define sat_lit_high 0x3F,0x3F,0x3F,0x3F,0x3F,0x3F
#define sat_lit_none 0x00,0x00,0x00,0x00,0x00,0x00

#define CHECK_PANEL_RET(func) do {\
	int ret = func;\
	if (ret) {\
		pr_err("#LCD WRITE ERROR: line %d\n", __LINE__);\
		return ret;}\
} while(0);

struct lcd_param {
	char param[48];
	int size;
	int delay;	/* delay time ms */
	int type;
};

enum m6x_lcd_id_reg {
	ID_CODE1 = 0, //0xda
	ID_CODE2, //0xdb
	ID_CODE3, //0xdc
	ID_FACTO, //0xdc
	ID_CODEMAX,
};

static int id_code[ID_CODEMAX];
static int lcd_id[ID_CODEMAX];

/*
 *  JDI
 */
static const struct lcd_param jdi_slpout_seq[] = {
	DCS_SHORT(120, MIPI_DCS_EXIT_SLEEP_MODE, 0x0),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param jdi_gamma_cmd2page0[] = {
	DCS_SHORT_PARAM(0xFF, 0x01),
	DCS_SHORT_PARAM(0xFB, 0x01),
	DCS_SHORT_PARAM(0x75, 0x00),
	DCS_SHORT_PARAM(0x76, 0x00),
	DCS_SHORT_PARAM(0x77, 0x00),
	DCS_SHORT_PARAM(0x78, 0x0E),
	DCS_SHORT_PARAM(0x79, 0x00),
	DCS_SHORT_PARAM(0x7A, 0x27),
	DCS_SHORT_PARAM(0x7B, 0x00),
	DCS_SHORT_PARAM(0x7C, 0x3D),
	DCS_SHORT_PARAM(0x7D, 0x00),
	DCS_SHORT_PARAM(0x7E, 0x50),
	DCS_SHORT_PARAM(0x7F, 0x00),
	DCS_SHORT_PARAM(0x80, 0x61),
	DCS_SHORT_PARAM(0x81, 0x00),
	DCS_SHORT_PARAM(0x82, 0x71),
	DCS_SHORT_PARAM(0x83, 0x00),
	DCS_SHORT_PARAM(0x84, 0x7F),
	DCS_SHORT_PARAM(0x85, 0x00),
	DCS_SHORT_PARAM(0x86, 0x8D),
	DCS_SHORT_PARAM(0x87, 0x00),
	DCS_SHORT_PARAM(0x88, 0xBA),
	DCS_SHORT_PARAM(0x89, 0x00),
	DCS_SHORT_PARAM(0x8A, 0xE0),
	DCS_SHORT_PARAM(0x8B, 0x01),
	DCS_SHORT_PARAM(0x8C, 0x1D),
	DCS_SHORT_PARAM(0x8D, 0x01),
	DCS_SHORT_PARAM(0x8E, 0x4F),
	DCS_SHORT_PARAM(0x8F, 0x01),
	DCS_SHORT_PARAM(0x90, 0x9C),
	DCS_SHORT_PARAM(0x91, 0x01),
	DCS_SHORT_PARAM(0x92, 0xD8),
	DCS_SHORT_PARAM(0x93, 0x01),
	DCS_SHORT_PARAM(0x94, 0xDA),
	DCS_SHORT_PARAM(0x95, 0x02),
	DCS_SHORT_PARAM(0x96, 0x10),
	DCS_SHORT_PARAM(0x97, 0x02),
	DCS_SHORT_PARAM(0x98, 0x4A),
	DCS_SHORT_PARAM(0x99, 0x02),
	DCS_SHORT_PARAM(0x9A, 0x73),
	DCS_SHORT_PARAM(0x9B, 0x02),
	DCS_SHORT_PARAM(0x9C, 0xA6),
	DCS_SHORT_PARAM(0x9D, 0x02),
	DCS_SHORT_PARAM(0x9E, 0xCE),
	DCS_SHORT_PARAM(0x9F, 0x03),
	DCS_SHORT_PARAM(0xA0, 0x02),
	DCS_SHORT_PARAM(0xA2, 0x03),
	DCS_SHORT_PARAM(0xA3, 0x0F),
	DCS_SHORT_PARAM(0xA4, 0x03),
	DCS_SHORT_PARAM(0xA5, 0x20),
	DCS_SHORT_PARAM(0xA6, 0x03),
	DCS_SHORT_PARAM(0xA7, 0x34),
	DCS_SHORT_PARAM(0xA9, 0x03),
	DCS_SHORT_PARAM(0xAA, 0x4D),
	DCS_SHORT_PARAM(0xAB, 0x03),
	DCS_SHORT_PARAM(0xAC, 0x68),
	DCS_SHORT_PARAM(0xAD, 0x03),
	DCS_SHORT_PARAM(0xAE, 0x89),
	DCS_SHORT_PARAM(0xAF, 0x03),
	DCS_SHORT_PARAM(0xB0, 0xB4),
	DCS_SHORT_PARAM(0xB1, 0x03),
	DCS_SHORT_PARAM(0xB2, 0xCA),
	DCS_SHORT_PARAM(0xB3, 0x00),
	DCS_SHORT_PARAM(0xB4, 0x00),
	DCS_SHORT_PARAM(0xB5, 0x00),
	DCS_SHORT_PARAM(0xB6, 0x0E),
	DCS_SHORT_PARAM(0xB7, 0x00),
	DCS_SHORT_PARAM(0xB8, 0x27),
	DCS_SHORT_PARAM(0xB9, 0x00),
	DCS_SHORT_PARAM(0xBA, 0x3D),
	DCS_SHORT_PARAM(0xBB, 0x00),
	DCS_SHORT_PARAM(0xBC, 0x50),
	DCS_SHORT_PARAM(0xBD, 0x00),
	DCS_SHORT_PARAM(0xBE, 0x61),
	DCS_SHORT_PARAM(0xBF, 0x00),
	DCS_SHORT_PARAM(0xC0, 0x71),
	DCS_SHORT_PARAM(0xC1, 0x00),
	DCS_SHORT_PARAM(0xC2, 0x7F),
	DCS_SHORT_PARAM(0xC3, 0x00),
	DCS_SHORT_PARAM(0xC4, 0x8D),
	DCS_SHORT_PARAM(0xC5, 0x00),
	DCS_SHORT_PARAM(0xC6, 0xBA),
	DCS_SHORT_PARAM(0xC7, 0x00),
	DCS_SHORT_PARAM(0xC8, 0xE0),
	DCS_SHORT_PARAM(0xC9, 0x01),
	DCS_SHORT_PARAM(0xCA, 0x1D),
	DCS_SHORT_PARAM(0xCB, 0x01),
	DCS_SHORT_PARAM(0xCC, 0x4F),
	DCS_SHORT_PARAM(0xCD, 0x01),
	DCS_SHORT_PARAM(0xCE, 0x9C),
	DCS_SHORT_PARAM(0xCF, 0x01),
	DCS_SHORT_PARAM(0xD0, 0xD8),
	DCS_SHORT_PARAM(0xD1, 0x01),
	DCS_SHORT_PARAM(0xD2, 0xDA),
	DCS_SHORT_PARAM(0xD3, 0x02),
	DCS_SHORT_PARAM(0xD4, 0x10),
	DCS_SHORT_PARAM(0xD5, 0x02),
	DCS_SHORT_PARAM(0xD6, 0x4A),
	DCS_SHORT_PARAM(0xD7, 0x02),
	DCS_SHORT_PARAM(0xD8, 0x73),
	DCS_SHORT_PARAM(0xD9, 0x02),
	DCS_SHORT_PARAM(0xDA, 0xA6),
	DCS_SHORT_PARAM(0xDB, 0x02),
	DCS_SHORT_PARAM(0xDC, 0xCE),
	DCS_SHORT_PARAM(0xDD, 0x03),
	DCS_SHORT_PARAM(0xDE, 0x02),
	DCS_SHORT_PARAM(0xDF, 0x03),
	DCS_SHORT_PARAM(0xE0, 0x0F),
	DCS_SHORT_PARAM(0xE1, 0x03),
	DCS_SHORT_PARAM(0xE2, 0x20),
	DCS_SHORT_PARAM(0xE3, 0x03),
	DCS_SHORT_PARAM(0xE4, 0x34),
	DCS_SHORT_PARAM(0xE5, 0x03),
	DCS_SHORT_PARAM(0xE6, 0x4D),
	DCS_SHORT_PARAM(0xE7, 0x03),
	DCS_SHORT_PARAM(0xE8, 0x68),
	DCS_SHORT_PARAM(0xE9, 0x03),
	DCS_SHORT_PARAM(0xEA, 0x89),
	DCS_SHORT_PARAM(0xEB, 0x03),
	DCS_SHORT_PARAM(0xEC, 0xB4),
	DCS_SHORT_PARAM(0xED, 0x03),
	DCS_SHORT_PARAM(0xEE, 0xCA),
	DCS_SHORT_PARAM(0xEF, 0x00),
	DCS_SHORT_PARAM(0xF0, 0x89),
	DCS_SHORT_PARAM(0xF1, 0x00),
	DCS_SHORT_PARAM(0xF2, 0x8F),
	DCS_SHORT_PARAM(0xF3, 0x00),
	DCS_SHORT_PARAM(0xF4, 0x9A),
	DCS_SHORT_PARAM(0xF5, 0x00),
	DCS_SHORT_PARAM(0xF6, 0xA5),
	DCS_SHORT_PARAM(0xF7, 0x00),
	DCS_SHORT_PARAM(0xF8, 0xAF),
	DCS_SHORT_PARAM(0xF9, 0x00),
	DCS_SHORT_PARAM(0xFA, 0xB9),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param jdi_gamma_cmd2page1[] = {
	DCS_SHORT_PARAM(0xFF, 0x02),
	DCS_SHORT_PARAM(0xFB, 0x01),
	DCS_SHORT_PARAM(0x00, 0x00),
	DCS_SHORT_PARAM(0x01, 0xC3),
	DCS_SHORT_PARAM(0x02, 0x00),
	DCS_SHORT_PARAM(0x03, 0xCC),
	DCS_SHORT_PARAM(0x04, 0x00),
	DCS_SHORT_PARAM(0x05, 0xD5),
	DCS_SHORT_PARAM(0x06, 0x00),
	DCS_SHORT_PARAM(0x07, 0xF5),
	DCS_SHORT_PARAM(0x08, 0x01),
	DCS_SHORT_PARAM(0x09, 0x11),
	DCS_SHORT_PARAM(0x0A, 0x01),
	DCS_SHORT_PARAM(0x0B, 0x42),
	DCS_SHORT_PARAM(0x0C, 0x01),
	DCS_SHORT_PARAM(0x0D, 0x6B),
	DCS_SHORT_PARAM(0x0E, 0x01),
	DCS_SHORT_PARAM(0x0F, 0xAE),
	DCS_SHORT_PARAM(0x10, 0x01),
	DCS_SHORT_PARAM(0x11, 0xE5),
	DCS_SHORT_PARAM(0x12, 0x01),
	DCS_SHORT_PARAM(0x13, 0xE7),
	DCS_SHORT_PARAM(0x14, 0x02),
	DCS_SHORT_PARAM(0x15, 0x19),
	DCS_SHORT_PARAM(0x16, 0x02),
	DCS_SHORT_PARAM(0x17, 0x52),
	DCS_SHORT_PARAM(0x18, 0x02),
	DCS_SHORT_PARAM(0x19, 0x7A),
	DCS_SHORT_PARAM(0x1A, 0x02),
	DCS_SHORT_PARAM(0x1B, 0xAF),
	DCS_SHORT_PARAM(0x1C, 0x02),
	DCS_SHORT_PARAM(0x1D, 0xD6),
	DCS_SHORT_PARAM(0x1E, 0x03),
	DCS_SHORT_PARAM(0x1F, 0x0A),
	DCS_SHORT_PARAM(0x20, 0x03),
	DCS_SHORT_PARAM(0x21, 0x19),
	DCS_SHORT_PARAM(0x22, 0x03),
	DCS_SHORT_PARAM(0x23, 0x2B),
	DCS_SHORT_PARAM(0x24, 0x03),
	DCS_SHORT_PARAM(0x25, 0x42),
	DCS_SHORT_PARAM(0x26, 0x03),
	DCS_SHORT_PARAM(0x27, 0x60),
	DCS_SHORT_PARAM(0x28, 0x03),
	DCS_SHORT_PARAM(0x29, 0x8A),
	DCS_SHORT_PARAM(0x2A, 0x03),
	DCS_SHORT_PARAM(0x2B, 0xE9),
	DCS_SHORT_PARAM(0x2D, 0x03),
	DCS_SHORT_PARAM(0x2F, 0xF8),
	DCS_SHORT_PARAM(0x30, 0x03),
	DCS_SHORT_PARAM(0x31, 0xFF),
	DCS_SHORT_PARAM(0x32, 0x00),
	DCS_SHORT_PARAM(0x33, 0x89),
	DCS_SHORT_PARAM(0x34, 0x00),
	DCS_SHORT_PARAM(0x35, 0x8F),
	DCS_SHORT_PARAM(0x36, 0x00),
	DCS_SHORT_PARAM(0x37, 0x9A),
	DCS_SHORT_PARAM(0x38, 0x00),
	DCS_SHORT_PARAM(0x39, 0xA5),
	DCS_SHORT_PARAM(0x3A, 0x00),
	DCS_SHORT_PARAM(0x3B, 0xAF),
	DCS_SHORT_PARAM(0x3D, 0x00),
	DCS_SHORT_PARAM(0x3F, 0xB9),
	DCS_SHORT_PARAM(0x40, 0x00),
	DCS_SHORT_PARAM(0x41, 0xC3),
	DCS_SHORT_PARAM(0x42, 0x00),
	DCS_SHORT_PARAM(0x43, 0xCC),
	DCS_SHORT_PARAM(0x44, 0x00),
	DCS_SHORT_PARAM(0x45, 0xD5),
	DCS_SHORT_PARAM(0x46, 0x00),
	DCS_SHORT_PARAM(0x47, 0xF5),
	DCS_SHORT_PARAM(0x48, 0x01),
	DCS_SHORT_PARAM(0x49, 0x11),
	DCS_SHORT_PARAM(0x4A, 0x01),
	DCS_SHORT_PARAM(0x4B, 0x42),
	DCS_SHORT_PARAM(0x4C, 0x01),
	DCS_SHORT_PARAM(0x4D, 0x6B),
	DCS_SHORT_PARAM(0x4E, 0x01),
	DCS_SHORT_PARAM(0x4F, 0xAE),
	DCS_SHORT_PARAM(0x50, 0x01),
	DCS_SHORT_PARAM(0x51, 0xE5),
	DCS_SHORT_PARAM(0x52, 0x01),
	DCS_SHORT_PARAM(0x53, 0xE7),
	DCS_SHORT_PARAM(0x54, 0x02),
	DCS_SHORT_PARAM(0x55, 0x19),
	DCS_SHORT_PARAM(0x56, 0x02),
	DCS_SHORT_PARAM(0x58, 0x52),
	DCS_SHORT_PARAM(0x59, 0x02),
	DCS_SHORT_PARAM(0x5A, 0x7A),
	DCS_SHORT_PARAM(0x5B, 0x02),
	DCS_SHORT_PARAM(0x5C, 0xAF),
	DCS_SHORT_PARAM(0x5D, 0x02),
	DCS_SHORT_PARAM(0x5E, 0xD6),
	DCS_SHORT_PARAM(0x5F, 0x03),
	DCS_SHORT_PARAM(0x60, 0x0A),
	DCS_SHORT_PARAM(0x61, 0x03),
	DCS_SHORT_PARAM(0x62, 0x19),
	DCS_SHORT_PARAM(0x63, 0x03),
	DCS_SHORT_PARAM(0x64, 0x2B),
	DCS_SHORT_PARAM(0x65, 0x03),
	DCS_SHORT_PARAM(0x66, 0x42),
	DCS_SHORT_PARAM(0x67, 0x03),
	DCS_SHORT_PARAM(0x68, 0x60),
	DCS_SHORT_PARAM(0x69, 0x03),
	DCS_SHORT_PARAM(0x6A, 0x8A),
	DCS_SHORT_PARAM(0x6B, 0x03),
	DCS_SHORT_PARAM(0x6C, 0xE9),
	DCS_SHORT_PARAM(0x6D, 0x03),
	DCS_SHORT_PARAM(0x6E, 0xF8),
	DCS_SHORT_PARAM(0x6F, 0x03),
	DCS_SHORT_PARAM(0x70, 0xFF),
	DCS_SHORT_PARAM(0x71, 0x00),
	DCS_SHORT_PARAM(0x72, 0xA0),
	DCS_SHORT_PARAM(0x73, 0x00),
	DCS_SHORT_PARAM(0x74, 0xA5),
	DCS_SHORT_PARAM(0x75, 0x00),
	DCS_SHORT_PARAM(0x76, 0xAF),
	DCS_SHORT_PARAM(0x77, 0x00),
	DCS_SHORT_PARAM(0x78, 0xB9),
	DCS_SHORT_PARAM(0x79, 0x00),
	DCS_SHORT_PARAM(0x7A, 0xC2),
	DCS_SHORT_PARAM(0x7B, 0x00),
	DCS_SHORT_PARAM(0x7C, 0xCB),
	DCS_SHORT_PARAM(0x7D, 0x00),
	DCS_SHORT_PARAM(0x7E, 0xD4),
	DCS_SHORT_PARAM(0x7F, 0x00),
	DCS_SHORT_PARAM(0x80, 0xDD),
	DCS_SHORT_PARAM(0x81, 0x00),
	DCS_SHORT_PARAM(0x82, 0xE5),
	DCS_SHORT_PARAM(0x83, 0x01),
	DCS_SHORT_PARAM(0x84, 0x02),
	DCS_SHORT_PARAM(0x85, 0x01),
	DCS_SHORT_PARAM(0x86, 0x1C),
	DCS_SHORT_PARAM(0x87, 0x01),
	DCS_SHORT_PARAM(0x88, 0x4B),
	DCS_SHORT_PARAM(0x89, 0x01),
	DCS_SHORT_PARAM(0x8A, 0x71),
	DCS_SHORT_PARAM(0x8B, 0x01),
	DCS_SHORT_PARAM(0x8C, 0xB3),
	DCS_SHORT_PARAM(0x8D, 0x01),
	DCS_SHORT_PARAM(0x8E, 0xE8),
	DCS_SHORT_PARAM(0x8F, 0x01),
	DCS_SHORT_PARAM(0x90, 0xE9),
	DCS_SHORT_PARAM(0x91, 0x02),
	DCS_SHORT_PARAM(0x92, 0x1C),
	DCS_SHORT_PARAM(0x93, 0x02),
	DCS_SHORT_PARAM(0x94, 0x53),
	DCS_SHORT_PARAM(0x95, 0x02),
	DCS_SHORT_PARAM(0x96, 0x7C),
	DCS_SHORT_PARAM(0x97, 0x02),
	DCS_SHORT_PARAM(0x98, 0xB2),
	DCS_SHORT_PARAM(0x99, 0x02),
	DCS_SHORT_PARAM(0x9A, 0xD8),
	DCS_SHORT_PARAM(0x9B, 0x03),
	DCS_SHORT_PARAM(0x9C, 0x0C),
	DCS_SHORT_PARAM(0x9D, 0x03),
	DCS_SHORT_PARAM(0x9E, 0x1C),
	DCS_SHORT_PARAM(0x9F, 0x03),
	DCS_SHORT_PARAM(0xA0, 0x2F),
	DCS_SHORT_PARAM(0xA2, 0x03),
	DCS_SHORT_PARAM(0xA3, 0x47),
	DCS_SHORT_PARAM(0xA4, 0x03),
	DCS_SHORT_PARAM(0xA5, 0x66),
	DCS_SHORT_PARAM(0xA6, 0x03),
	DCS_SHORT_PARAM(0xA7, 0x9E),
	DCS_SHORT_PARAM(0xA9, 0x03),
	DCS_SHORT_PARAM(0xAA, 0xF0),
	DCS_SHORT_PARAM(0xAB, 0x03),
	DCS_SHORT_PARAM(0xAC, 0xF8),
	DCS_SHORT_PARAM(0xAD, 0x03),
	DCS_SHORT_PARAM(0xAE, 0xFF),
	DCS_SHORT_PARAM(0xAF, 0x00),
	DCS_SHORT_PARAM(0xB0, 0xA0),
	DCS_SHORT_PARAM(0xB1, 0x00),
	DCS_SHORT_PARAM(0xB2, 0xA5),
	DCS_SHORT_PARAM(0xB3, 0x00),
	DCS_SHORT_PARAM(0xB4, 0xAF),
	DCS_SHORT_PARAM(0xB5, 0x00),
	DCS_SHORT_PARAM(0xB6, 0xB9),
	DCS_SHORT_PARAM(0xB7, 0x00),
	DCS_SHORT_PARAM(0xB8, 0xC2),
	DCS_SHORT_PARAM(0xB9, 0x00),
	DCS_SHORT_PARAM(0xBA, 0xCB),
	DCS_SHORT_PARAM(0xBB, 0x00),
	DCS_SHORT_PARAM(0xBC, 0xD4),
	DCS_SHORT_PARAM(0xBD, 0x00),
	DCS_SHORT_PARAM(0xBE, 0xDD),
	DCS_SHORT_PARAM(0xBF, 0x00),
	DCS_SHORT_PARAM(0xC0, 0xE5),
	DCS_SHORT_PARAM(0xC1, 0x01),
	DCS_SHORT_PARAM(0xC2, 0x02),
	DCS_SHORT_PARAM(0xC3, 0x01),
	DCS_SHORT_PARAM(0xC4, 0x1C),
	DCS_SHORT_PARAM(0xC5, 0x01),
	DCS_SHORT_PARAM(0xC6, 0x4B),
	DCS_SHORT_PARAM(0xC7, 0x01),
	DCS_SHORT_PARAM(0xC8, 0x71),
	DCS_SHORT_PARAM(0xC9, 0x01),
	DCS_SHORT_PARAM(0xCA, 0xB3),
	DCS_SHORT_PARAM(0xCB, 0x01),
	DCS_SHORT_PARAM(0xCC, 0xE8),
	DCS_SHORT_PARAM(0xCD, 0x01),
	DCS_SHORT_PARAM(0xCE, 0xE9),
	DCS_SHORT_PARAM(0xCF, 0x02),
	DCS_SHORT_PARAM(0xD0, 0x1C),
	DCS_SHORT_PARAM(0xD1, 0x02),
	DCS_SHORT_PARAM(0xD2, 0x53),
	DCS_SHORT_PARAM(0xD3, 0x02),
	DCS_SHORT_PARAM(0xD4, 0x7C),
	DCS_SHORT_PARAM(0xD5, 0x02),
	DCS_SHORT_PARAM(0xD6, 0xB2),
	DCS_SHORT_PARAM(0xD7, 0x02),
	DCS_SHORT_PARAM(0xD8, 0xD8),
	DCS_SHORT_PARAM(0xD9, 0x03),
	DCS_SHORT_PARAM(0xDA, 0x0C),
	DCS_SHORT_PARAM(0xDB, 0x03),
	DCS_SHORT_PARAM(0xDC, 0x1C),
	DCS_SHORT_PARAM(0xDD, 0x03),
	DCS_SHORT_PARAM(0xDE, 0x2F),
	DCS_SHORT_PARAM(0xDF, 0x03),
	DCS_SHORT_PARAM(0xE0, 0x47),
	DCS_SHORT_PARAM(0xE1, 0x03),
	DCS_SHORT_PARAM(0xE2, 0x66),
	DCS_SHORT_PARAM(0xE3, 0x03),
	DCS_SHORT_PARAM(0xE4, 0x9E),
	DCS_SHORT_PARAM(0xE5, 0x03),
	DCS_SHORT_PARAM(0xE6, 0xF0),
	DCS_SHORT_PARAM(0xE7, 0x03),
	DCS_SHORT_PARAM(0xE8, 0xF8),
	DCS_SHORT_PARAM(0xE9, 0x03),
	DCS_SHORT_PARAM(0xEA, 0xFF),
	DCS_SHORT_PARAM(0xFF, 0x00),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param jdi_dspon_seq[] = {
	DCS_SHORT_PARAM(0xD3, 0x08),
	DCS_SHORT_PARAM(0xD4, 0x06),
	DCS_SHORT_PARAM(MIPI_DCS_SET_ADDRESS_MODE, 0x00), //NG
	DCS_SHORT(0, MIPI_DCS_SET_DISPLAY_ON, 0x0),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param jdi_video_psr_mode[] = {
	DCS_SHORT(3, 0X01, 0), // soft_reset min 3ms
	GCS_SHORT_PARAM(10, 0xB0,0x00), // MC_Manufacturer Command Access Protect
	GCS_LONG(0, 0xB3,0x14,0x00,0x00,0x22,0x00,0x00), // MC_Frame Memory Access and Interface Setting, Video Mode
	GCS_SHORT_PARAM(0, 0xC0, 0xFF),
	GCS_LONG(0, 0xC2,0x31,0xF7,0x08,0x09,0x03,0x00,0x00), // Set NL(1800=0x708, 1920=0x780), BP(9), FP(3)
	GCS_LONG(0, 0xC4,0x70,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x02),
	GCS_LONG(0, 0xC6,0x84,0x00,0x70,0x00,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x1C,0x05,\
			0x84,0x00,0x70,0x00,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x1C,0x05), // Set RTN0, RTNA0 = 0x80 = 128 clocks
	GCS_LONG(0, 0xD0,0x11,0x81,0xBB,0x16,0x8D,0x4C,0x19,0x19,0x0C,0x00), // Set NL(1800=0x708, 1920=0x780), BP(9), FP(3)
	GCS_SHORT_PARAM(0, 0xD6,0x01),	// MC_Test_Register, No NVM load on exit_sleep_mode, default is 0x81(reload NVM)
	//GCS_LONG(0,0xC1, 0x08), // Red <-> Blue Color Swap
	DCS_SHORT_PARAM(0x3A,0x77),	// set_pixel_format
	DCS_LONG(0x2A,0x00,0x00,0x04,0x37),	// set_column_address for Command mode
	DCS_LONG(0x2B,0x00,0x00,0x07,0x07),	// set_page_address for Command mode
	DCS_SHORT_PARAM(0x35,0x00), // set_tear_on
	LCD_PARAM_DEF_END,
};

static const struct lcd_param jdi_dspoff_seq[] = {
	DCS_SHORT(120, MIPI_DCS_SET_DISPLAY_OFF, 0x0),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param jdi_slpin_seq[] = {
	DCS_SHORT(120, MIPI_DCS_ENTER_SLEEP_MODE, 0x0),
	LCD_PARAM_DEF_END,
};

/*
 *  Sharp
 */
// init setting for Command Mode but start with video mode
static const struct lcd_param sharp_cmd_psr_mode[] = {
	DCS_SHORT(3, 0X01, 0), // soft_reset min 3ms
	GCS_SHORT_PARAM(0, 0xB0,0x00), // MC_Manufacturer Command Access Protect
	GCS_LONG(0, 0xB3,0x0C,0x00,0x00,0x22,0x00,0x00),
	GCS_LONG(0, 0xC2,0x32,0xF7,0x08,0x09,0x03,0x00,0x00), // Set NL(1800=0x708, 1920=0x780), BP(9), FP(3)
	GCS_LONG(0, 0xC4,0x70,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x05,0x09,0x00,0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x00,0x05,0x09),
	GCS_SHORT_PARAM(0, 0xD6,0x01),	// MC_Test_Register, No NVM load on exit_sleep_mode, default is 0x81(reload NVM)
	//GCS_LONG(0,0xC1, 0x08), // Red <-> Blue Color Swap
	DCS_SHORT_PARAM(0x3A,0x77),	// set_pixel_format
	DCS_LONG(0x2A,0x00,0x00,0x04,0x37),	// set_column_address for Command mode
	DCS_LONG(0x2B,0x00,0x00,0x07,0x07),	// set_page_address for Command mode
	DCS_SHORT_PARAM(0x35,0x00), // set_tear_on
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_slpout_seq[] = {
	DCS_SHORT(120, MIPI_DCS_EXIT_SLEEP_MODE, 0x0),
	//GCS_LONG(0, 0xC2,0x30,0xF7,0x08,0x09,0x03,0x00,0x00), // Set NL(1800=0x708, 1920=0x780), BP(9), FP(3)
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_dspon_seq[] = {
	DCS_SHORT(20, MIPI_DCS_SET_DISPLAY_ON, 0x0),
	//DCS_LONG(MIPI_DCS_SET_DISPLAY_ON, 0x0),
	//DCS_SHORT_PARAM(MIPI_DCS_BACKLIGHT_ON, 0x24),
	//DCS_SHORT_PARAM(0x35,0x00), // Te ON
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_dspoff_seq[] = {
	//DCS_SHORT_PARAM(MIPI_DCS_BACKLIGHT_ON, 0x0),
	DCS_SHORT(20, MIPI_DCS_SET_DISPLAY_OFF, 0x0),
	LCD_PARAM_DEF_END,
};

static const struct lcd_param sharp_slpin_seq[] = {
	DCS_SHORT(80, MIPI_DCS_ENTER_SLEEP_MODE, 0x0),
	LCD_PARAM_DEF_END,
};

static int write_to_lcd(struct mipi_dsim_device *dsim,
		const struct lcd_param *param)
{
	int i = 0, ret = 0;

	do {
		ret = param[i].size ?
			write_data(dsim, param[i].type, param[i].param, param[i].size) :
			write_cmd(dsim, param[i].type, param[i].param[0], param[i].param[1]);
		if (param[i].delay)
			usleep_range(param[i].delay *1000, param[i].delay *1000);

	} while (!ret && param[++i].size != -1);

	return ret;
}

/*
 *  JDI
 */
static int lcd_panel_jdi_sleep_out(struct mipi_dsim_device *dsim)
{
	return write_to_lcd(dsim, jdi_slpout_seq);
}

static int lcd_panel_jdi_gamma_seq(struct mipi_dsim_device *dsim)
{
	int ret = 0;
	
	ret = write_to_lcd(dsim, jdi_gamma_cmd2page0);
	ret = write_to_lcd(dsim, jdi_gamma_cmd2page1);
	return ret;
}

static int lcd_panel_jdi_display_on(struct mipi_dsim_device *dsim)
{
	return write_to_lcd(dsim, jdi_dspon_seq); 
}

static int lcd_panel_jdi_display_off(struct mipi_dsim_device *dsim)
{
	return write_to_lcd(dsim, jdi_dspoff_seq);
}

static int lcd_panel_jdi_sleep_in(struct mipi_dsim_device *dsim)
{
	return write_to_lcd(dsim, jdi_slpin_seq);
}

/*
 *  Sharp
 */
static int lcd_panel_sharp_init_code(struct mipi_dsim_device *dsim)
{
	switch (lcd_id[ID_CODE3]) {
	case 0:
		if (lcd_id[ID_FACTO] == 0)
			return write_to_lcd(dsim, jdi_video_psr_mode);
		else
			return write_to_lcd(dsim, sharp_cmd_psr_mode);

	default:
		pr_debug("ID Code(%d) Error! use default gamma settings.\n", lcd_id[ID_CODE3]);
		return 0;
	}
}

static int lcd_panel_sharp_sleep_out(struct mipi_dsim_device *dsim)
{
	return write_to_lcd(dsim, sharp_slpout_seq);
}

static int lcd_panel_sharp_display_on(struct mipi_dsim_device *dsim)
{
	return write_to_lcd(dsim, sharp_dspon_seq);
}

static int lcd_panel_sharp_display_off(struct mipi_dsim_device *dsim)
{
	return write_to_lcd(dsim, sharp_dspoff_seq);
}

static int lcd_panel_sharp_sleep_in(struct mipi_dsim_device *dsim)
{
	return write_to_lcd(dsim, sharp_slpin_seq);
}

static int m6x_displayon(struct mipi_dsim_device *dsim)
{
	return 1;
}

static int m6x_init(struct mipi_dsim_device *dsim)
{
	if (id_code[ID_CODE1] & 0x10) {
		/*JDI init*/
		CHECK_PANEL_RET(lcd_panel_jdi_sleep_out(dsim));
		CHECK_PANEL_RET(lcd_panel_jdi_gamma_seq(dsim));
		CHECK_PANEL_RET(lcd_panel_jdi_display_on(dsim));

		CHECK_PANEL_RET(lcd_panel_jdi_sleep_out(dsim));
		CHECK_PANEL_RET(lcd_panel_jdi_display_on(dsim));
	} else {
		/*Sharp init*/
		CHECK_PANEL_RET(lcd_panel_sharp_init_code(dsim));

		CHECK_PANEL_RET(lcd_panel_sharp_sleep_out(dsim));
		CHECK_PANEL_RET(lcd_panel_sharp_display_on(dsim));
	}

	return 0;
}

static int m6x_read_id(struct mipi_dsim_device *dsim, unsigned int id_pin)
{
	int ret;
	ret = gpio_request(id_pin, "lcd_id");
	if (ret < 0) {
		pr_err("Failed to get gpio number for the lcd id.\n");
		return -EINVAL;
	}

	lcd_id[ID_FACTO] = gpio_get_value(id_pin);
	if (lcd_id[ID_FACTO] == 1)
		pr_info("Sharp Module.\n");
	else
		pr_info("JDI Module.\n");

	gpio_free(id_pin);
	return 0;
}

static int m6x_probe(struct mipi_dsim_device *dsim)
{
	memset(id_code, 0, sizeof(id_code));

	return 1;
}

static int m6x_suspend(struct mipi_dsim_device *dsim)
{
	if ((id_code[ID_CODE1] & 0x10)) {
		CHECK_PANEL_RET(lcd_panel_jdi_display_off(dsim));
		CHECK_PANEL_RET(lcd_panel_jdi_sleep_in(dsim));
	} else {
		CHECK_PANEL_RET(lcd_panel_sharp_display_off(dsim));
		CHECK_PANEL_RET(lcd_panel_sharp_sleep_in(dsim));
	}
	return 1;
}

static int m6x_resume(struct mipi_dsim_device *dsim)
{
	return 1;
}

struct mipi_dsim_lcd_driver m6x_mipi_lcd_driver = {
	.probe		= m6x_probe,
	.read_id        = m6x_read_id,
	.init           = m6x_init,
	.displayon	= m6x_displayon,
	.suspend	= m6x_suspend,
	.resume		= m6x_resume,
};

MODULE_DESCRIPTION("MEIZU Lcd panel driver");
MODULE_AUTHOR("WangBo <wangbo@meizu.com>");
MODULE_LICENSE("GPL v2");
