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
 *  Filename: wukong_boot.c
 *
 *  Description: Boot features init and interupt handling.
 *
 *  History:
 *   Nov, 11 2011 -  Li Wang(wangli@marvell.com) Creation of file
 *
 *  Notes:
 *
 ******************************************************************************/
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <asm/gpio.h>

#include "wukong_boot.h"
#include "wukong_config.h"
#include "modem_state_notify.h"

static struct wake_lock boot_wakeup;
static struct uevent_work boot_uevent_wk;
static struct modem_device *wk_modem;
irqreturn_t modem_bootup(int irq, void *dev_id);
extern int wk_irq_enable;

void hold_modem(int internal)
{
	char *env[2];

	struct modem_device *modem = wk_modem;
	if (NULL == modem)
		return;
	DEBUGMSG("%s: hold_modem operation triggered\n", __func__);

	if (get_modem_state() == HOLD)
		return;

	//ignore the fake irq
	if (wk_irq_enable) {
		disable_irq(modem->boot_up_irq);
		disable_irq(modem->watch_dog_irq);
		disable_irq(modem->assert_eeh_irq);
		wk_irq_enable = 0;
	}

	if (gpio_get_value(modem->power_enable) == 0) {
		set_line(modem->power_enable, modem->active_level);
		set_line(modem->reset_gpio, !modem->active_level);
	} else
		set_line(modem->reset_gpio, !modem->active_level);

	set_modem_state(HOLD, env);

	mdelay(100);

	if (!internal) {
		RETAILMSG("%s: hold uevent is sent\n", __func__);
		kobject_uevent_env(&modem->dev->kobj, KOBJ_CHANGE, env);
	}
}

void release_modem(int internal)
{
	char *env[2];

	struct modem_device *modem = wk_modem;
	if (NULL == modem)
		return;

	DEBUGMSG("%s: release_modem operation triggered\n", __func__);
	if (get_modem_state() != HOLD)
		return;

	set_line(modem->reset_gpio, modem->active_level);
	mdelay(100);

	set_modem_state(RELEASE, env);

	if (!internal) {
		mdelay(100);
		RETAILMSG("%s: release uevent is sent\n", __func__);
		kobject_uevent_env(&modem->dev->kobj, KOBJ_CHANGE, env);
	}
}

void boot_uevent_work(struct work_struct *work)
{
	struct uevent_work *uevent_wq =
	    container_of(work, struct uevent_work, work);
	struct modem_device *modem = wk_modem;

	msleep(50);
	if (uevent_wq->modem->active_level != get_line((modem->boot_up_gpio))) {
		set_modem_state(RELEASE, boot_uevent_wk.env);
		ERRORMSG("%s: fake boot interupt is detected\n", __func__);
		return;
	}

	set_modem_state(BOOTUP, boot_uevent_wk.env);

	//enable_irq(modem->watch_dog_irq);
	//enable_irq(modem->assert_eeh_irq);

	RETAILMSG("%s: boot up uevent is sent\n", __func__);
	kobject_uevent_env(&uevent_wq->modem->dev->kobj, KOBJ_CHANGE,
			   uevent_wq->env);
}

int init_modem_boot(struct modem_device *modem)
{
	int ret = -1;
	wk_modem = modem;
	wake_lock_init(&boot_wakeup, WAKE_LOCK_SUSPEND, "modem_boot_wakeups");

	INIT_WORK(&boot_uevent_wk.work, boot_uevent_work);
	boot_uevent_wk.modem = modem;

#if 0
	if (gpio_request((modem->boot_up_gpio), "CP BOOTUP IRQ")) {
		ERRORMSG("%s: CP BOOTUP IRQ, GPIO[%d] request failed!\n",
			 __func__, (modem->boot_up_gpio));
		return ret;
	}

	gpio_direction_input((modem->boot_up_gpio));
#endif

	ret = request_irq(modem->boot_up_irq, modem_bootup,
			  IRQF_TRIGGER_RISING,
			  "modem boot up", modem);
	if (ret < 0) {
		ERRORMSG("%s:Can't request irq for modem boot:%d!\n",
			 __func__, modem->boot_up_irq);
		return ret;
	}
	/* init with irq disabled */
	disable_irq(modem->boot_up_irq);

	//init to hold status
	//hold_modem(1);

	return 0;
}

void deinit_modem_boot(struct modem_device *modem)
{
	free_irq(modem->boot_up_irq, NULL);
	gpio_free((modem->boot_up_gpio));
	hold_modem(1);
	wk_modem = NULL;
	wake_lock_destroy(&boot_wakeup);
}

irqreturn_t modem_bootup(int irq, void *dev_id)
{
	int sec = 10;
	struct modem_device *modem = (struct modem_device *)dev_id;
	RETAILMSG("%s: bootup irq received. irq[%d] \n", __func__, irq);

	if (get_modem_state() != RELEASE || !wk_irq_enable) {
		ERRORMSG("%s:It is not a expected interrupt!\n", __func__);
		return IRQ_NONE;
	}

	queue_work(modem->modem_wq, &boot_uevent_wk.work);
	wake_lock_timeout(&boot_wakeup, HZ * sec);
	return IRQ_HANDLED;
}
