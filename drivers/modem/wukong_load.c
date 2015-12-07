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
 *  Filename: wukong_load.c
 *
 *  Description: Init wukong dev, offer ioctl feature for user space.
 *
 *  History:
 *   Nov, 11 2011 -  Li Wang(wangli@marvell.com) Creation of file
 *
 *  Notes:
 *
 ******************************************************************************/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#include <linux/interrupt.h>

#include "wukong_load.h"
#include "wukong_boot.h"
#include "wukong_assert.h"
#include "wukong_config.h"
#include "modem_state_notify.h"

static struct modem_device wukong_modem_910 = {
	.active_level = 1,
	.boot_up_irq = 0,
	.watch_dog_irq = 0,
	.assert_eeh_irq = 0
};

static struct modem_device *wukong_modem = 0;
int wk_irq_enable = 0;

extern struct delayed_uevent_work wdt_uevent_wk;


static int modem_open(struct inode *inode, struct file *filp);
static ssize_t modem_write(struct file *filp, const char __user * buf,
			   size_t count, loff_t * f_pos);
static ssize_t modem_read(struct file *filp, char __user * buf, size_t count,
			  loff_t * f_pos);
static long modem_ioctl(struct file *filp, unsigned int cmd, unsigned long);
static int modem_release(struct inode *inode, struct file *filp);

static int b_can_be_read = 0;

static struct file_operations modem_fops = {
	.open = modem_open,
	.write = modem_write,
	.read = modem_read,
	.release = modem_release,
	.unlocked_ioctl = modem_ioctl,
	.owner = THIS_MODULE
};

static struct miscdevice modem_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "wukong",
	.fops = &modem_fops
};

void set_line(int gpio, int level)
{
	int pin;
	if (gpio < 0) {
		return;
	}
	pin = gpio;		//mfp_to_gpio(gpio);
	gpio_direction_output(pin, level);
	RETAILMSG("%s: GPIO out pin = %d, level = %d\n", __func__, pin,
		  level);
}

int get_line(int gpio)
{
	int pin, val;
	pin = gpio;		//mfp_to_gpio(gpio);
	//gpio_direction_input(pin);
	val = !!gpio_get_value(pin);

	RETAILMSG("%s: GPIO in pin = %d, val = %d\n", __func__, pin, val);
	return val;
}

static int init_modem(struct modem_device *modem)
{
	modem->modem_wq = create_workqueue("modem boot work queue");
	if (modem->modem_wq == NULL) {
		ERRORMSG("%s:Can't create work queue!\n", __func__);
		return -ENOMEM;
	}
	init_modem_notify_chain();
	return 0;
}

static void deinit_modem(struct modem_device *modem)
{
	if (modem->modem_wq != NULL)
		destroy_workqueue(modem->modem_wq);
}

static int wukong_load_probe(struct platform_device *pdev)
{
	int ret;
	int gpio;
	struct pinctrl *pinctrl;
	struct device *dev = &pdev->dev;

	if (!dev->of_node)
		return -EINVAL;

	wukong_modem = &wukong_modem_910;

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev, "no pins associated\n");

	gpio = of_get_named_gpio(dev->of_node, "nezha,boot-up-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha boot-up gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, "boot-up-gpio");
	if (ret)
		dev_err(dev, "can't request nezha boot-up gpio %d\n", gpio);

	wukong_modem->boot_up_gpio = gpio;

	gpio = of_get_named_gpio(dev->of_node, "nezha,cp-wdt-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha cp-wdt gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, "cp-wdt-gpio");
	if (ret)
		dev_err(dev, "can't request nezha cp-wdt gpio %d\n", gpio);

	wukong_modem->watch_dog_gpio = gpio;

	gpio = of_get_named_gpio(dev->of_node, "nezha,cp-assert-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha cp-assert gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, "cp-assert-gpio");
	if (ret)
		dev_err(dev, "can't request nezha cp-assert gpio %d\n", gpio);

	wukong_modem->assert_eeh_gpio = gpio;

	gpio = of_get_named_gpio(dev->of_node, "nezha,ap-reset-cp-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha ap-reset-cp gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_HIGH, "ap-reset-cp-gpio");
	if (ret)
		dev_err(dev, "can't request nezha ap-reset-cp gpio %d\n", gpio);

	gpio_set_value(gpio, 1);
	wukong_modem->reset_gpio = gpio;

	gpio = of_get_named_gpio(dev->of_node, "nezha,cp-pwr-on-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha cp-pwr-on gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_LOW, "cp-pwr-on-gpio");
	if (ret)
		dev_err(dev, "can't request nezha cp-pwr-on gpio %d\n", gpio);

	gpio_set_value(gpio, 0);
	wukong_modem->power_enable = gpio;

	gpio = of_get_named_gpio(dev->of_node, "nezha,sim-det-gpio", 0);
	if (!gpio_is_valid(gpio))
		dev_err(dev, "can't get nezha sim-det gpio\n");

	ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, "sim-det-gpio");
	if (ret)
		dev_err(dev, "can't request nezha sim-det gpio %d\n", gpio);

	wukong_modem->sim_det_gpio = gpio;

	wukong_modem->boot_up_irq = gpio_to_irq(wukong_modem->boot_up_gpio);
	wukong_modem->watch_dog_irq = gpio_to_irq(wukong_modem->watch_dog_gpio);
	wukong_modem->assert_eeh_irq = gpio_to_irq(wukong_modem->assert_eeh_gpio);
	wukong_modem->sim_det_irq = gpio_to_irq(wukong_modem->sim_det_gpio);

	ret = misc_register(&modem_miscdev);
	if (ret < 0) {
		ERRORMSG("%s:Can't register misc device!\n", __func__);
		return ret;
	}

	wukong_modem->dev = modem_miscdev.this_device;
	ret = init_modem(wukong_modem);
	if (ret < 0) {
		ERRORMSG("%s:Can't initialize modem!\n", __func__);
		return ret;
	}

	init_modem_boot(wukong_modem);
	init_modem_assert(wukong_modem);

	dev_info(dev, "load wukong_load driver ok!\n");

	return 0;
}

static int wukong_load_remove(struct platform_device *pdev)
{
	int ret = -1;
	deinit_modem(wukong_modem);

	deinit_modem_boot(wukong_modem);
	deinit_modem_assert(wukong_modem);

	ret = misc_deregister(&modem_miscdev);
	if (ret < 0) {
		ERRORMSG("%s:Can't deregister misc device!\n", __func__);
	}

	return ret;
}

static int modem_open(struct inode *inode, struct file *filp)
{
	filp->private_data = (void *)wukong_modem;

	b_can_be_read = 1;

	return 0;
}

static int modem_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static void report_relink_event(void)
{
	char *env[2];
	env[0] = "STATE=RELINK";
	env[1] = NULL;

	RETAILMSG("%s: relink uevent is sent by %s\n", __func__, current->comm);
	kobject_uevent_env(&wukong_modem->dev->kobj, KOBJ_CHANGE, env);
}

static void report_update_event(void)
{
	char *env[2];
	env[0] = "STATE=UPDATE";
	env[1] = NULL;

	RETAILMSG("%s: update uevent is sent\n", __func__);
	kobject_uevent_env(&wukong_modem->dev->kobj, KOBJ_CHANGE, env);
}

static void report_erase_event(void)
{
	char *env[2];
	env[0] = "STATE=ERASE";
	env[1] = NULL;

	RETAILMSG("%s: erase uevent is sent\n", __func__);
	kobject_uevent_env(&wukong_modem->dev->kobj, KOBJ_CHANGE, env);
}

extern void set_hsic_active(int value);

static ssize_t modem_write(struct file *filp, const char __user * buf,
			   size_t count, loff_t * f_pos)
{
	char c[2];

	DEBUGMSG("%s: vaule count(%d) from user space\n", __func__, count);

	if (count != 2) {
		ERRORMSG("%s: get invalid vaule from user space\n", __func__);
		return -EINVAL;
	} else {
		if (copy_from_user(&c, buf, count)) {
			ERRORMSG("%s: copy vaule from user space failed\n",
				 __func__);
			return -EFAULT;
		}
	}

	switch (c[0]) {
	case '1':
		RETAILMSG ("%s: get '1' from user space, cp reset process triggered\n",
		     __func__);
		report_relink_event();
		break;
	case '2':
		RETAILMSG
		    ("%s: get '2' from user space, cp images update process triggered\n",
		     __func__);
		report_update_event();
		break;
	case '3':
		RETAILMSG
		    ("%s: get '3' from user space, cp images erase process triggered\n",
		     __func__);
		report_erase_event();
		break;
	case '4':
		RETAILMSG
		    ("%s: get '4' from user space, cp images erase process triggered\n",
		     __func__);
		set_line(wukong_modem->power_enable, 0);
		break;
	case '5':
		RETAILMSG
		    ("%s: get '5' from user space, cp images erase process triggered\n",
		     __func__);
		set_line(wukong_modem->power_enable, 1);
		break;
	case '6':
		RETAILMSG
		    ("%s: get '6' from user space, ap hsic rdy low triggered\n",
		     __func__);
		set_hsic_active(0);
		break;
	case '7':
		RETAILMSG
		    ("%s: get '7' from user space, ap hsic rdy high triggered\n",
		     __func__);
		set_hsic_active(1);
		break;
	default:
		ERRORMSG("%s: get invalid vaule from user space\n", __func__);
		break;
	}

	return count;
}

static ssize_t modem_read(struct file *filp, char __user * buf, size_t count,
			  loff_t * f_pos)
{
	char *state = get_modem_state_name(-1);

	int count_t = 0;

	if (!b_can_be_read)
		return 0;

	b_can_be_read = 0;

	if (state == NULL)
		return -EFAULT;

	count_t = (int)strlen(state);

	if (count > count_t)
		count = count_t;

	if (copy_to_user(buf, state, count)) {
		ERRORMSG("%s: copy to user failed\n", __func__);
		return -EFAULT;
	}

	return count;
}

static int show_modem_state(unsigned long arg)
{
	int state = get_modem_state();
	if (copy_to_user((int *)arg, &state, sizeof(int))) {
		ERRORMSG("%s: copy to user failed\n", __func__);
		return -EFAULT;

	}
	return 0;
}

static long modem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct modem_device *modem = (struct modem_device *)filp->private_data;

	if (_IOC_TYPE(cmd) != WUKONG_IOC_MAGIC) {
		ERRORMSG("%s: seh magic number is wrong!\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	case WUKONG_IOCTL_HOLD:
		hold_modem(0);
		break;
	case WUKONG_IOCTL_RELEASE:
		release_modem(0);
		break;
	case WUKONG_IOCTL_GET_OBM_INFO:
		DEBUGMSG("%s: nothing for OBM_INFO\n", __func__);
		break;
	case WUKONG_IOCTL_SHOW_MODEM_STATE:
		show_modem_state(arg);
		break;
	case WUKONG_IOCTL_DOWNLOAD_TEST:
		set_modem_state(WDT, wdt_uevent_wk.env);
		queue_delayed_work(modem->modem_wq, &wdt_uevent_wk.work,
				   HZ / 10);
		break;
	case WUKONG_IOCTL_TRIGGER_EEH_DUMP:
		report_eeh_dump_event();
		break;
	case WUKONG_IOCTL_ENABLE_MODEM_IRQ:
		wk_irq_enable = 1;
		enable_irq(modem->boot_up_irq);
		enable_irq(modem->watch_dog_irq);
		enable_irq(modem->assert_eeh_irq);
		RETAILMSG("%s: enable irq from modem from userspace\n", __func__);
		break;
	default:
		ERRORMSG("%s: illgal command !\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static const struct of_device_id wukong_match[] = {
	{ .compatible = "mrvl,wukong_load", },
	{},
};
MODULE_DEVICE_TABLE(of, wukong_match);

static struct platform_driver wukong_load_driver = {
	.probe = wukong_load_probe,
	.remove = wukong_load_remove,
	.driver = {
		.name = "wukong_load",
		.owner = THIS_MODULE,
		.of_match_table = wukong_match,
	},
};

static int __init wukong_load_driver_init(void)
{
	return platform_driver_register(&wukong_load_driver);
}

static void __init wukong_load_driver_exit(void)
{
	platform_driver_unregister(&wukong_load_driver);
}

module_init(wukong_load_driver_init);
module_exit(wukong_load_driver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell WuKong Loader.");
