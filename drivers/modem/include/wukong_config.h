/******************************************************************************
*(C) Copyright 2011 Marvell International Ltd.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 2 as published by
    the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/
/*--------------------------------------------------------------------------------------------------------------------
 *  -------------------------------------------------------------------------------------------------------------------
 *
 *  Filename: wukong_config.h
 *
 *  Description: Export configuration default value/debug macro.
 *
 *  History:
 *   Nov, 11 2011 -  Li Wang(wangli@marvell.com) Creation of file
 *
 *  Notes:
 *
 ******************************************************************************/
#ifndef __WUKONG_CONFIG_H_
#define __WUKONG_CONFIG_H_

/*config gpio*/
#define GPIO_NO_USE -1

/*GPIO for 910+WK*/
#define GPIO_RESET	GPIO_NO_USE
#define GPIO_BOOTUP	GPIO_NO_USE
#define GPIO_WDTRST	GPIO_NO_USE
#define GPIO_EEH	GPIO_NO_USE

/*config debug parameter*/
#define DEBUG_WUKONG_LOAD_DRIVER

/*config WDT DKBI download mode*/
//#define WDT_DOWNLOAD_DKBI_IN_DRIVER

#ifdef DEBUG_WUKONG_LOAD_DRIVER
#define DEBUGMSG(fmt, args ...)     printk("wukong: " fmt, ## args)
#define ERRORMSG(fmt, args ...) printk(KERN_ERR "wukong: " fmt, ## args)
#define ENTER()                 printk("wukong: ENTER %s\n", __FUNCTION__)
#define LEAVE()                 printk("wukong: LEAVE %s\n", __FUNCTION__)
#define FUNC_EXIT()                     printk("wukong: EXIT %s\n", __FUNCTION__)
#define RETAILMSG(fmt, args ...) printk(KERN_INFO "wukong: " fmt, ## args)
#else
#define DEBUGMSG(fmt, args ...)     printk(KERN_DEBUG "wukong: " fmt, ## args)
#define ERRORMSG(fmt, args ...) printk(KERN_ERR "wukong:" fmt, ## args)
#define ENTER()         do {} while (0)
#define LEAVE()         do {} while (0)
#define FUNC_EXIT()     do {} while (0)
#define RETAILMSG(fmt, args ...) printk("wukong: " fmt, ## args)
#endif

#endif
