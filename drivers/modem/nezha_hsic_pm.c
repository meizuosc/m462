/******************************************************************************
 * HSIC PM driver
 *
*(C) Copyright 2014 Meizu International Ltd.

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

#define DRIVER_DESC "HSIC PM Driver for Nezha modem"

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/miscdevice.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/uaccess.h>
#include <linux/usb/quirks.h>

#include <linux/irq.h>
#include <linux/gpio.h>

#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/suspend.h>

#include <linux/wakelock.h>

#define MIF_INFO(fmt, ...)	pr_info("NEZHA_LOG %s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define MIF_DEBUG(fmt, ...)	pr_debug("NEZHA_LOG %s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)
#define MIF_ERR(fmt, ...)	pr_err("NEZHA_LOG %s: " pr_fmt(fmt), __func__, ##__VA_ARGS__)

//extern int s5p_wait_for_cp_resume(struct platform_device *pdev, int port);
extern int usb_resume(struct device *dev, pm_message_t msg);

struct link_pm_data {
	spinlock_t lock;
	spinlock_t bh_lock;
	bool dpm_suspending;
	bool roothub_resume_req;
	int if_usb_connected;
	int resume_retry_cnt;
	int resume_requested;
	int rpm_suspending_cnt;
	struct wake_lock l2_wake;
	struct wake_lock rpm_wake;
	struct delayed_work hsic_pm_work;

	// for fw loading
	unsigned int irq_hostwake;
	unsigned int gpio_hostwake;
	unsigned int gpio_slavewake;
	unsigned int gpio_hsic_active;
	unsigned int gpio_cp_hsic_rdy;

	bool hsic_l3;

	struct usb_device *udev;
	struct usb_device *hdev;
	struct notifier_block pm_notifier;
	struct notifier_block usb_notifier;
};

static struct link_pm_data *g_pm;

bool l2_resume_flag = false;

void set_hsic_active(int value)
{
	if (!g_pm)
		return;

	if (!value && !l2_resume_flag)
		g_pm->hsic_l3 = true;

	MIF_INFO("set hsic active value %d\n", value);
	gpio_direction_output(g_pm->gpio_hsic_active, value);
}
EXPORT_SYMBOL(set_hsic_active);

static struct link_pm_data *linkdata_from_udev(struct usb_device *udev)
{
	spin_lock_bh(&g_pm->bh_lock);
	if (udev == g_pm->udev || udev == g_pm->hdev) {
		spin_unlock_bh(&g_pm->bh_lock);
		return g_pm;
	}
	//MIF_DEBUG("udev=%p, %s\n", udev, dev_name(&udev->dev));
	spin_unlock_bh(&g_pm->bh_lock);
	return NULL;
}

/* hooking from generic_suspend and generic_resume */
static int (*_usb_suspend) (struct usb_device *, pm_message_t);
static int (*_usb_resume) (struct usb_device *, pm_message_t);

static inline bool get_hostwake(struct link_pm_data *pm_data)
{
	return !!gpio_get_value(pm_data->gpio_hostwake);
}

static inline void set_slavewake(struct link_pm_data *pm_data, int val)
{
	if (val && gpio_get_value(pm_data->gpio_slavewake)) {
		MIF_INFO("warn.. slavewake toggle\n");
		gpio_set_value(pm_data->gpio_slavewake, 0);
		msleep(20);
	}
	gpio_set_value(pm_data->gpio_slavewake, val);
}

static int xmm626x_gpio_usb_resume(struct link_pm_data *pm_data)
{
	int spin = 20;

	if (get_hostwake(pm_data)) /* CP inititated L2->L0 */
		goto exit;

	/* AP initiated L2->L0 */
	MIF_DEBUG("AP wakeup modem from L2\n");
	set_slavewake(pm_data, 1);

	while (spin-- && !get_hostwake(pm_data))
		usleep_range(5000, 5500);

	if (!get_hostwake(pm_data)) /* Hostwakeup timeout */
		return -ETIMEDOUT;
exit:
	l2_resume_flag = true;
	return 0;
}

static int xmm626x_gpio_l3_resume(struct link_pm_data *pm_data)
{
	pm_data->roothub_resume_req = false;

	// wakeup cp at port resume at L2
	if (!pm_data->hsic_l3)
		return 0;

	if (!get_hostwake(pm_data)) {
		/* AP initiated L3 -> L0 */
		MIF_DEBUG("AP wakeup modem from L3\n");
		set_slavewake(pm_data, 1);
	}

	/* CP initiated L3 -> L0 */

	return 0;
}

static int xmm626x_linkpm_usb_resume(struct usb_device *udev, pm_message_t msg)
{
	struct link_pm_data *pm_data = linkdata_from_udev(udev);
	int ret = 0;

	if (!pm_data) /* unknown devices */
		goto generic_resume;

	dev_dbg(&udev->dev, "%s enter resume\n", __func__);
	if (udev == pm_data->hdev) {
		pm_runtime_mark_last_busy(&pm_data->hdev->dev);
		xmm626x_gpio_l3_resume(pm_data);
		goto generic_resume;
	}

	/* Because HSIC modem skip the hub dpm_resume by quirk, if root hub
	  dpm_suspend was called at runtmie active status, hub resume was not
	  call by port runtime resume. So, it check the L3 status and root hub
	  resume before port resume */
	if (pm_data->roothub_resume_req) {
		MIF_DEBUG("ehci root hub resume first\n");
		pm_runtime_mark_last_busy(&pm_data->hdev->dev);
		ret = usb_resume(&pm_data->hdev->dev, PMSG_RESUME);
		if (ret)
			MIF_ERR("hub resume fail\n");
	}

	// skip port gpio_resume as we toggle slave in root_hub
	if (pm_data->hsic_l3) {
		pm_data->hsic_l3 = false;
		goto generic_resume;
	}

	/* Sometimes IMC modem send remote wakeup with gpio, we should check
	  the runtime status and if aleady resumed, */
	if (udev->dev.power.runtime_status == RPM_ACTIVE) {
		MIF_INFO("aleady resume, skip gpio resume\n");
		goto generic_resume;
	}

	ret = xmm626x_gpio_usb_resume(pm_data);
	if (ret < 0) {
		MIF_ERR("hostwakeup from L2 fail\n");
		/* TODO: exception handing if hoswakeup timeout */
	}

generic_resume:
	return _usb_resume(udev, msg);
}

static int xmm626x_linkpm_usb_suspend(struct usb_device *udev, pm_message_t msg)
{
	struct link_pm_data *pm_data = linkdata_from_udev(udev);

	if (!pm_data) /* unknown devices */
		goto generic_suspend;

	dev_dbg(&udev->dev, "%s enter suspend\n", __func__);

	if (!pm_data->if_usb_connected)
		goto generic_suspend;

	/* dpm suspend to Sleep mode */
	if (msg.event == PM_EVENT_SUSPEND) {
		MIF_INFO("hub suspend with rpm active\n");
		pm_data->roothub_resume_req = true;
	}

	/* unlock when post resume */
	if ((pm_data->udev == udev) && wake_lock_active(&pm_data->l2_wake))
		wake_unlock(&pm_data->l2_wake);

	/* DO nothing yet */

generic_suspend:
	return _usb_suspend(udev, msg);
}

#include "modem_state_notify.h"

static int hsic_usb_notifier_event(struct notifier_block *this,
				unsigned long event, void *ptr)
{
	struct usb_device *udev = ptr;
	struct link_pm_data *pm_data = g_pm;
	const struct usb_device_descriptor *desc = &udev->descriptor;
	struct usb_device_driver *udriver =
					to_usb_device_driver(udev->dev.driver);
	unsigned long flags;

	switch (event) {
	case USB_DEVICE_ADD:
		if (desc->idVendor == 0x1286 && desc->idProduct == 0x8130) {
			if (get_modem_state() != BOOTUP)
				return NOTIFY_DONE;

			if (pm_data->udev) {
				MIF_ERR("pmdata was assigned for udev=%p\n",
								pm_data->udev);
				return NOTIFY_DONE;
			}
			pm_data->udev = udev;
			pm_data->hdev = udev->bus->root_hub;
			pm_data->hsic_l3 = false;
			pm_data->if_usb_connected = 1;
			enable_irq_wake(pm_data->irq_hostwake);
			MIF_DEBUG("udev=%p, hdev=%p\n", udev, pm_data->hdev);

			spin_lock_irqsave(&pm_data->lock, flags);
			if (!_usb_resume && udriver->resume) {
				_usb_resume = udriver->resume;
				udriver->resume = xmm626x_linkpm_usb_resume;
			}
			if (!_usb_suspend && udriver->suspend) {
				_usb_suspend = udriver->suspend;
				udriver->suspend = xmm626x_linkpm_usb_suspend;
			}
			if (pm_data->udev->quirks & USB_QUIRK_NO_DPM_RESUME)
				pm_data->hdev->quirks |= USB_QUIRK_NO_DPM_RESUME;
			spin_unlock_irqrestore(&pm_data->lock, flags);
			MIF_DEBUG("hook: (%pf, %pf), (%pf, %pf)\n",
					_usb_resume, udriver->resume,
					_usb_suspend,	udriver->suspend);
		}
		break;
	case USB_DEVICE_REMOVE:
		if (desc->idVendor == 0x1286 && desc->idProduct == 0x8130) {
			if (!pm_data->udev)
				return NOTIFY_DONE;
			pm_data->hdev->quirks &= ~USB_QUIRK_NO_DPM_RESUME;

			pm_data->hdev = NULL;
			pm_data->udev = NULL;
			pm_data->if_usb_connected = 0;

			MIF_DEBUG("unhook: (%pf, %pf), (%pf, %pf)\n",
					_usb_resume, udriver->resume,
					_usb_suspend,	udriver->suspend);
			spin_lock_irqsave(&pm_data->lock, flags);
			if (_usb_resume) {
				udriver->resume = _usb_resume;
				_usb_resume = NULL;
			}
			if (_usb_suspend) {
				udriver->suspend = _usb_suspend;
				_usb_suspend = NULL;
			}
			spin_unlock_irqrestore(&pm_data->lock, flags);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}

static void hsic_pm_runtime_work(struct work_struct *work)
{
	int ret;
	struct link_pm_data *pm_data =
		container_of(work, struct link_pm_data, hsic_pm_work.work);
	struct usb_device *usbdev = pm_data->udev;
	struct device *dev = &usbdev->dev;

	if (pm_data->dpm_suspending || !pm_data->if_usb_connected)
		return;

	MIF_DEBUG("for dev 0x%p : current %d\n", dev,
				dev->power.runtime_status);

	switch (dev->power.runtime_status) {
	case RPM_ACTIVE:
		pm_data->resume_retry_cnt = 0;
		pm_data->resume_requested = false;
		pm_data->rpm_suspending_cnt = 0;
		return;
	case RPM_SUSPENDED:
		if (pm_data->resume_requested)
			break;
		pm_data->resume_requested = true;
		wake_lock(&pm_data->rpm_wake);
		if (!pm_data->if_usb_connected) {
			MIF_ERR("usb disconnect\n");
			wake_unlock(&pm_data->rpm_wake);
			return;
		}
		ret = pm_runtime_resume(dev);
		if (ret < 0) {
			MIF_ERR("resume error(%d)\n", ret);
			if (!pm_data->if_usb_connected) {
				wake_unlock(&pm_data->rpm_wake);
				return;
			}
			/* force to go runtime idle before retry resume */
			if (dev->power.timer_expires == 0 &&
						!dev->power.request_pending) {
				MIF_ERR("run time idle\n");
				pm_runtime_idle(dev);
			}
		}
		wake_unlock(&pm_data->rpm_wake);
		pm_data->rpm_suspending_cnt = 0;
		break;
	case RPM_SUSPENDING:
		/* checking the usb_runtime_suspend running times */
		wake_lock(&pm_data->rpm_wake);
		pm_data->rpm_suspending_cnt++;
		if (pm_data->rpm_suspending_cnt < 10)
			msleep(20);
		else if (pm_data->rpm_suspending_cnt < 30)
			msleep(50);
		else
			msleep(100);
		wake_unlock(&pm_data->rpm_wake);
		break;
	case RPM_RESUMING:
	default:
		MIF_DEBUG("RPM Resuming, ssuspending_cnt :%d\n",
						pm_data->rpm_suspending_cnt);
		pm_data->rpm_suspending_cnt = 0;
		break;
	}
	pm_data->resume_requested = false;
	/* check until runtime_status goes to active */
	if (dev->power.runtime_status == RPM_ACTIVE) {
		pm_data->resume_retry_cnt = 0;
		pm_data->rpm_suspending_cnt = 0;
	} else if (pm_data->resume_retry_cnt++ > 80) {
		MIF_ERR("runtime_status:%d, retry_cnt:%d, notify MODEM_EVENT_CRASH\n",
			dev->power.runtime_status, pm_data->resume_retry_cnt);
	} else {
		MIF_DEBUG("runtime_status:%d, retry_cnt:%d, redo hsic_pm_work\n",
			dev->power.runtime_status, pm_data->resume_retry_cnt);
		schedule_delayed_work(&pm_data->hsic_pm_work,
							msecs_to_jiffies(50));
	}
}

static irqreturn_t hostwake_irq_handler(int irq, void *data)
{
	int value, slave_value;
	struct link_pm_data *pm_data = data;

	if (!pm_data->if_usb_connected)
		return IRQ_HANDLED;

	if (pm_data->dpm_suspending) {
		MIF_INFO("wakeup when suspending\n");
		/* Ignore HWK but AP got to L2 by suspending fail */
		wake_lock(&pm_data->l2_wake);
		return IRQ_HANDLED;
	}

	value = gpio_get_value(pm_data->gpio_hostwake);
	slave_value = gpio_get_value(pm_data->gpio_slavewake);
	MIF_DEBUG("hostwake tiggered (%d)\n", value);

	if (value == 1 && slave_value == 0) {
		MIF_DEBUG("CP wakeup modem from %s\n", pm_data->hsic_l3 ? "L3" : "L2");
		schedule_delayed_work(&pm_data->hsic_pm_work, 0);
	}

	if (value == 0)
		set_slavewake(pm_data, 0);

	return IRQ_HANDLED;
}

bool sys_suspending = false;

static int nezha_pm_callback(struct notifier_block *this, unsigned long event, void *v)
{
	struct link_pm_data *pm_data =
			container_of(this, struct link_pm_data,	pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		pm_data->dpm_suspending = true;
		sys_suspending = true;
		MIF_INFO("dpm suspending set to true\n");
		break;
	case PM_POST_SUSPEND:
		pm_data->dpm_suspending = false;
		if (get_hostwake(pm_data)) {
			wake_lock(&pm_data->l2_wake);
			schedule_delayed_work(&pm_data->hsic_pm_work, 0);
			MIF_INFO("post resume\n");
		}
		sys_suspending = false;
		MIF_INFO("dpm suspending set to false\n");
		break;
	}

	return NOTIFY_OK;
}

static int hsic_pm_probe(struct platform_device *pdev)
{
	int ret;
	int gpio;
	struct link_pm_data *pm_data;
	struct device *dev = &pdev->dev;

	dev_info(&pdev->dev, "enter\n");

	pm_data = kzalloc(sizeof(struct link_pm_data), GFP_KERNEL);
	if (!pm_data)
		return -ENOMEM;

	gpio = of_get_named_gpio(dev->of_node, "nezha,host-wakeup-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha boot-up gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, "host-wakeup-gpio");
	if (ret)
		dev_err(dev, "can't request nezha host-wakeup gpio %d\n", gpio);

	pm_data->gpio_hostwake = gpio;
	pm_data->irq_hostwake = gpio_to_irq(pm_data->gpio_hostwake);

	gpio = of_get_named_gpio(dev->of_node, "nezha,host-active-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha host-active gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_LOW, "host-active-gpio");
	if (ret)
		dev_err(dev, "can't request nezha host-active gpio %d\n", gpio);

	gpio_direction_output(gpio, 0);
	pm_data->gpio_hsic_active = gpio;

	gpio = of_get_named_gpio(dev->of_node, "nezha,cp-hsic-rdy-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha cp_hsic_rdy gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, "cp-hsic-rdy-gpio");
	if (ret)
		dev_err(dev, "can't request nezha cp-hsic-rdy gpio %d\n", gpio);

	pm_data->gpio_cp_hsic_rdy = gpio;

	gpio = of_get_named_gpio(dev->of_node, "nezha,slave-wakeup-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha slave-wakeup gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_LOW, "slave-wakeup-gpio");
	if (ret)
		dev_err(dev, "can't request nezha slave-wakeup gpio %d\n", gpio);

	gpio_direction_output(gpio, 0);
	pm_data->gpio_slavewake = gpio;

	pm_data->if_usb_connected = 0;

	ret = request_irq(pm_data->irq_hostwake, hostwake_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
			"nezha_hostwake", (void *)pm_data);
	if (ret)
		pr_err("%s: fail to request irq(%d)\n", __func__, ret);

	g_pm = pm_data;
	platform_set_drvdata(pdev, pm_data);

	INIT_DELAYED_WORK(&pm_data->hsic_pm_work, hsic_pm_runtime_work);
	wake_lock_init(&pm_data->l2_wake, WAKE_LOCK_SUSPEND, "l2_hsic");
	wake_lock_init(&pm_data->rpm_wake, WAKE_LOCK_SUSPEND, "rpm_hsic");

	pm_data->usb_notifier.notifier_call = hsic_usb_notifier_event;
	usb_register_notify(&pm_data->usb_notifier);
	pm_data->pm_notifier.notifier_call = nezha_pm_callback;
	register_pm_notifier(&pm_data->pm_notifier);

	spin_lock_init(&pm_data->lock);
	spin_lock_init(&pm_data->bh_lock);
	pm_data->udev = NULL;
	pm_data->hdev = NULL;

	return 0;
}

static int hsic_pm_remove(struct platform_device *pdev)
{
	return 0;
}

static int nezha_pm_suspend(struct device *dev)
{
	struct link_pm_data *pm_data = dev_get_drvdata(dev);

	disable_irq(pm_data->irq_hostwake);
	irq_set_irq_type(pm_data->irq_hostwake,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_HIGH | IRQF_ONESHOT);
	enable_irq_wake(pm_data->irq_hostwake);

	return 0;
}

static int nezha_pm_resume(struct device *dev)
{
	struct link_pm_data *pm_data = dev_get_drvdata(dev);

	disable_irq_wake(pm_data->irq_hostwake);
	irq_set_irq_type(pm_data->irq_hostwake,
		IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	enable_irq(pm_data->irq_hostwake);

	return 0;
}

const struct dev_pm_ops hsic_pm_apm = {
	.suspend = nezha_pm_suspend,
	.resume = nezha_pm_resume,
};

static const struct of_device_id hsic_pm_match[] = {
	{ .compatible = "mrvl,hsic_pm", },
	{},
};
MODULE_DEVICE_TABLE(of, hsic_pm_match);

static struct platform_driver hsic_pm_driver = {
	.probe = hsic_pm_probe,
	.remove = hsic_pm_remove,
	.driver = {
		.name = "nezha_hsic_pm",
		.owner = THIS_MODULE,
		.of_match_table = hsic_pm_match,
		.pm = &hsic_pm_apm,
	},
};

static int __init nezha_hsic_pm_init(void)
{
	return platform_driver_register(&hsic_pm_driver);
}

late_initcall(nezha_hsic_pm_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Liu Jianping<heljoy@meizu.com");
MODULE_DESCRIPTION(DRIVER_DESC);
