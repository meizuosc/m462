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
 *  Filename: wukong_load.h
 *
 *  Description: export wukong load externel structure definition and API.
 *
 *  History:
 *   Nov, 11 2011 -  Li Wang(wangli@marvell.com) Creation of file
 *
 *  Notes:
 *
 ******************************************************************************/
#ifndef __WUKONG_LOAD_H_
#define __WUKONG_LOAD_H_

#include<linux/workqueue.h>
#include<linux/device.h>
#include "wukong_config.h"

#define WUKONG_IOC_MAGIC 'W'
#define WUKONG_IOCTL_HOLD		_IOW(WUKONG_IOC_MAGIC, 1, int)
#define WUKONG_IOCTL_RELEASE		_IOW(WUKONG_IOC_MAGIC, 2, int)
#define WUKONG_IOCTL_GET_OBM_INFO	_IOW(WUKONG_IOC_MAGIC, 3, int)
#define WUKONG_IOCTL_SHOW_MODEM_STATE	_IOR(WUKONG_IOC_MAGIC, 4, int)
#define WUKONG_IOCTL_DOWNLOAD_TEST	_IOW(WUKONG_IOC_MAGIC, 5, int)
#define WUKONG_IOCTL_TRIGGER_EEH_DUMP	_IOW(WUKONG_IOC_MAGIC, 6, int)
#define WUKONG_IOCTL_ENABLE_MODEM_IRQ	_IOW(WUKONG_IOC_MAGIC, 7, int)

struct modem_device {
	int power_enable;	// gpio to power on modem
	int reset_gpio;		// gpio to reset modem
	int active_level;	// vaild levet to hold modem
	int boot_up_irq;	//  modem notice AP reday
	int boot_up_gpio;
	int watch_dog_irq;	//modem watch dog time out
	int watch_dog_gpio;
	int assert_eeh_irq;	// modem assert
	int assert_eeh_gpio;
	int sim_det_irq;	// modem sim card detect
	int sim_det_gpio;
	struct workqueue_struct *modem_wq;
	struct device *dev;
};
struct uevent_work {
	struct work_struct work;
	struct modem_device *modem;
	char *env[2];
};
struct delayed_uevent_work {
	struct delayed_work work;
	struct modem_device *modem;
	char *env[2];
};

void set_line(int gpio, int level);
int get_line(int gpio);

#endif
