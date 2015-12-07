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
 *  Filename: wukong_assert.c
 *
 *  Description: Assert features init and interupt handling.
 *
 *  History:
 *   Nov, 11 2011 -  Li Wang(wangli@marvell.com) Creation of file
 *
 *  Notes:
 *
 ******************************************************************************/
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/version.h>

#include <linux/module.h>

#include "wukong_assert.h"
#include "wukong_config.h"
#include "wukong_boot.h"
#include "wukong_load.h"
#include "modem_state_notify.h"

extern int wk_irq_enable;

static struct wake_lock erro_wakeup;

#ifdef WDT_DOWNLOAD_DKBI_IN_DRIVER
static DEFINE_SPINLOCK(uart_download_lock);
#endif

static struct uevent_work assert_uevent_wk;
static struct uevent_work simdet_uevent_wk;
struct delayed_uevent_work wdt_uevent_wk;
irqreturn_t _modem_error(int irq, void *dev_id);

void simdet_uevent_work(struct work_struct *work)
{
	char *env[2];
	struct uevent_work *uevent_wq =
	    container_of(work, struct uevent_work, work);

	msleep(100);

	if (get_modem_state() != BOOTUP) {
		ERRORMSG("%s: Skip sim-det event as CP not bootup\n", __func__);
		return;
	}

	if (uevent_wq->modem->active_level !=
	    get_line((uevent_wq->modem->sim_det_gpio))) {
		env[0] = "STATE=SIMREMOVE";
		env[1] = NULL;
		RETAILMSG("%s: simremove uevent is sent\n", __func__);
		kobject_uevent_env(&uevent_wq->modem->dev->kobj, KOBJ_CHANGE, env);
		return;
	}

	env[0] = "STATE=SIMINSERT";
	env[1] = NULL;
	RETAILMSG("%s: siminsert uevent is sent\n", __func__);
	kobject_uevent_env(&uevent_wq->modem->dev->kobj, KOBJ_CHANGE, env);

	uevent_wq->env[0] = "STATE=RELINK";
	uevent_wq->env[1] = NULL;

	kobject_uevent_env(&uevent_wq->modem->dev->kobj, KOBJ_CHANGE,
			   uevent_wq->env);
	RETAILMSG("%s: simdet uevent[%s][%s] is sent\n", __func__,
		  uevent_wq->env[0], uevent_wq->env[1]);
}

void assert_uevent_work(struct work_struct *work)
{
	struct uevent_work *uevent_wq =
	    container_of(work, struct uevent_work, work);

	msleep(100);
	if ((uevent_wq->modem->active_level !=
	     get_line(uevent_wq->modem->assert_eeh_gpio))
	    || (uevent_wq->modem->active_level ==
		get_line((uevent_wq->modem->watch_dog_gpio)))) {
		ERRORMSG("%s: fake EEH Assert interupt is detected\n",
			 __func__);
		return;
	}

	set_modem_state(ASSERT, assert_uevent_wk.env);

	kobject_uevent_env(&uevent_wq->modem->dev->kobj, KOBJ_CHANGE,
			   uevent_wq->env);
	RETAILMSG("%s: assert error uevent[%s][%s] is sent\n", __func__,
		  uevent_wq->env[0], uevent_wq->env[1]);
}

void wdt_uevent_work(struct work_struct *work)
{
	struct delayed_uevent_work *uevent_wq = container_of(work,
			struct delayed_uevent_work, work.work);

	msleep(100);
	if (uevent_wq->modem->active_level !=
	    get_line((uevent_wq->modem->watch_dog_gpio))) {
		ERRORMSG("%s: fake WDT interupt is detected\n", __func__);
		return;
	}

	set_modem_state(WDT, wdt_uevent_wk.env);

	kobject_uevent_env(&uevent_wq->modem->dev->kobj, KOBJ_CHANGE,
			   uevent_wq->env);
	RETAILMSG("%s: wdt uevent[%s][%s]\n", __func__, uevent_wq->env[0],
		  uevent_wq->env[1]);
}

void report_eeh_dump_event(void)
{
	char *env[2];
	env[0] = "ERROR=EEHDUMP";
	env[1] = NULL;

	RETAILMSG("%s: eehdump uevent is sent\n", __func__);
	kobject_uevent_env(&assert_uevent_wk.modem->dev->kobj, KOBJ_CHANGE,
			   env);
}

int init_modem_assert(struct modem_device *modem)
{
	int ret = -1;

	INIT_WORK(&assert_uevent_wk.work, assert_uevent_work);
	INIT_WORK(&simdet_uevent_wk.work, simdet_uevent_work);
	INIT_DELAYED_WORK(&wdt_uevent_wk.work, wdt_uevent_work);
	wake_lock_init(&erro_wakeup, WAKE_LOCK_SUSPEND, "modem_error_wakeups");
	assert_uevent_wk.modem = modem;
	wdt_uevent_wk.modem = modem;
	simdet_uevent_wk.modem = modem;

#if 0
	if (gpio_request((modem->watch_dog_gpio), "CP WDT IRQ")) {
		ERRORMSG("%s: CP EEH WDTRST IRQ, GPIO[%d] request failed!\n",
			 __func__, (modem->watch_dog_gpio));
		return ret;
	}

	if (gpio_request((modem->assert_eeh_gpio), "CP Assert IRQ")) {
		ERRORMSG("%s: CP EEH IRQ, GPIO[%d] request failed!\n",
			 __func__, (modem->assert_eeh_gpio));
		return ret;
	}

	gpio_direction_input((modem->watch_dog_gpio));
	gpio_direction_input((modem->assert_eeh_gpio));
#endif

	ret = request_irq(modem->assert_eeh_irq, _modem_error,
			IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
			"modem assert", modem);
	if (ret < 0) {
		ERRORMSG("%s:Can't request irq for modem assert:%d!\n",
			 __func__, modem->assert_eeh_irq);
		return ret;
	}
	ret = request_irq(modem->watch_dog_irq, _modem_error,
			IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
			"modem wdt", modem);
	if (ret < 0) {
		ERRORMSG("%s:Can't request irq for modem wdt:%d!\n",
			 __func__, modem->watch_dog_irq);
		free_irq(modem->assert_eeh_irq, NULL);
		return ret;
	}
	ret = request_irq(modem->sim_det_irq, _modem_error,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
			"sim detect", modem);
	if (ret < 0) {
		ERRORMSG("%s:Can't request irq for modem assert:%d!\n",
			 __func__, modem->sim_det_irq);
		return ret;
	}

	/* init with irq disabled */
	disable_irq(modem->watch_dog_irq);
	disable_irq(modem->assert_eeh_irq);

	enable_irq_wake(modem->watch_dog_irq);
	enable_irq_wake(modem->assert_eeh_irq);
	enable_irq_wake(modem->sim_det_irq);
	return 0;
}

void deinit_modem_assert(struct modem_device *modem)
{
	free_irq(modem->assert_eeh_irq, NULL);
	free_irq(modem->watch_dog_irq, NULL);
	free_irq(modem->sim_det_irq, NULL);
	disable_irq_wake(modem->assert_eeh_irq);
	disable_irq_wake(modem->watch_dog_irq);
	disable_irq_wake(modem->sim_det_irq);
	gpio_free((modem->assert_eeh_gpio));
	gpio_free((modem->watch_dog_gpio));
	gpio_free((modem->sim_det_gpio));

	wake_lock_destroy(&erro_wakeup);

}

irqreturn_t _modem_error(int irq, void *dev_id)
{
	struct modem_device *modem = (struct modem_device *)dev_id;
	int sec = 15;

	if (!wk_irq_enable) {
		RETAILMSG
		    ("%s: fake modem irq received. irq[%d] wk_irq_enable:%d\n",
		     __func__, irq, wk_irq_enable);
		goto RET;
	}

	if (irq == modem->watch_dog_irq) {
		RETAILMSG("%s: wdt irq received. irq[%d] hold %ds wake lock\n",
			  __func__, irq, sec);
		queue_work(modem->modem_wq, &wdt_uevent_wk.work.work);
	} else if (irq == modem->assert_eeh_irq) {
		RETAILMSG
		    ("%s: assert irq received. irq[%d] hold %ds wake lock\n",
		     __func__, irq, sec);
		queue_work(modem->modem_wq, &assert_uevent_wk.work);
	} else if (irq == modem->sim_det_irq) {
		RETAILMSG
		    ("%s: SIM-DET irq received. irq[%d] hold %ds wake lock\n",
		     __func__, irq, sec);
		queue_work(modem->modem_wq, &simdet_uevent_wk.work);
	}

RET:
	wake_lock_timeout(&erro_wakeup, HZ * sec);
	return IRQ_HANDLED;
}
