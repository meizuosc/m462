/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

//ALERT:relocate pn544.c under .\kernel\drivers\misc

/*
* Makefile//TODO:Here is makefile reference
* obj-$(CONFIG_PN544)+= pn544.o
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/regmap.h>

#include "pn547_i2c_driver.h"

#include <linux/wakelock.h>
#include <linux/suspend.h>

//#define pr_err printk
//#define pr_debug printk
//#define pr_warning printk




#define MAX_BUFFER_SIZE	512

struct pn544_dev
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	struct wake_lock	wake_lock;
	struct notifier_block	pm_notifier;
};

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	if (!gpio_get_value(pn544_dev->irq_gpio))
		return IRQ_HANDLED;

	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	pr_debug("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;
	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) {
		pr_debug("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("IFD->PC:");
	for(i = 0; i < ret; i++)
		pr_debug(" %02X", tmp[i]);
	pr_debug("\n");

	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

/* as chip issue, some IO blocked at addr@0x2b but OK at 0x29
 * set 0x29 if 0x2B blocked three time, but we don't konw the root
 * cause of the hardware issue
 */
static int error_write = 0;
static bool nfc_addr_ok = false;

static int set_nfc_i2c_addr(int nfc_addr)
{
	struct file *fp      = NULL;
	char *nfc_file       = "/data/nfc/nfc_addr";
	char addr_str[10]     = "0x29\n";
	mm_segment_t oldfs;
	int ret;

	fp = filp_open(nfc_file, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp))
		return -1;

	oldfs = get_fs();
	set_fs(get_ds());

	snprintf(addr_str, sizeof(addr_str), "0x%02X\n", nfc_addr);

	ret = fp->f_op->write(fp, (const char *)addr_str, sizeof(addr_str), &fp->f_pos);
	if (ret < 0)
		pr_err("%s:write nfc addr error\n", __func__);

	set_fs(oldfs);
	filp_close(fp, NULL);

	return 0;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		pr_debug("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
		error_write ++;
		if (!nfc_addr_ok && error_write > 3) {
			int nfc_addr = pn544_dev->client->addr;
			int new_addr = nfc_addr == 0x2B ? 0x29 : 0x2B;

			pr_info("%s change nfc addr to %02X\n", __func__, new_addr);
			pn544_dev->client->addr = new_addr;
			error_write = 0;
		}
	}

	if (ret == count && !nfc_addr_ok) {
		int nfc_addr = pn544_dev->client->addr;

		if (nfc_addr == 0x29) {
			pr_info("%s: save nfc addr to file\n", __func__);
			set_nfc_i2c_addr(nfc_addr);
		}

		nfc_addr_ok = true;
		error_write = 0;
	}

	pr_debug("PC->IFD:");
	for(i = 0; i < count; i++)
		pr_debug(" %02X", tmp[i]);
	pr_debug("\n");

	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data, struct pn544_dev, pn544_device);

	filp->private_data = pn544_dev;

	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static int get_nfc_i2c_addr(void)
{
	struct file *fp      = NULL;
	char *nfc_file       = "/data/nfc/nfc_addr";
	char nfcbuffer[10]   = {0};
	int nfc_addr = 0;
	int ret;

	fp = filp_open(nfc_file, O_RDONLY, 0);
	if (IS_ERR(fp))
		return -1;

	ret = kernel_read(fp, 0, nfcbuffer, sizeof(nfcbuffer));
	if (ret < 4) {
		pr_info("%s read addr file error\n", __func__);
		return -2;
	}

	if (strstr(nfcbuffer, "0x29"))
		nfc_addr = 0x29;

	filp_close(fp, NULL);

	return nfc_addr;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			printk("%s power on with firmware\n", __func__);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			printk("%s power on\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			enable_irq_wake(pn544_dev->client->irq);

			/* get real nfc i2c addr */
			if (!nfc_addr_ok) {
				int nfc_addr = get_nfc_i2c_addr();

				if (nfc_addr == 0x29) {
					pr_info("abnormal NFC CHIP, set addr to 0x29\n");
					nfc_addr_ok = true;
					pn544_dev->client->addr = nfc_addr;
				}
			}

			msleep(10);

		} else  if (arg == 0) {
			/* power off */
			printk("%s power off\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			disable_irq_wake(pn544_dev->client->irq);
			msleep(50);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops =
{
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl = pn544_dev_ioctl,
};

static int pn544_pm_callback(struct notifier_block *this, unsigned long event, void *v)
{
	struct pn544_dev *pn544_dev =
			container_of(this, struct pn544_dev, pm_notifier);

	switch (event) {
	case PM_SUSPEND_PREPARE: /* try to suspending */
	case PM_POST_SUSPEND: /* back from suspended */
		if (gpio_get_value(pn544_dev->irq_gpio)) {
			pr_info("########nfc irq trigered########\n");
			wake_lock_timeout(&pn544_dev->wake_lock, 5*HZ);
		}
		break;
	}
	return NOTIFY_OK;
}

static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int gpio;
	struct pinctrl *pinctrl;
	struct pn544_dev *pn544_dev;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;

	dev_info(dev, "probe M76 NFC at i2c@%x\n", client->addr);
	if (!np) {
		dev_err(dev, "no DT node for nfc\n");
		return -EINVAL;
	}

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "no pins associated\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	//IRQ 
	gpio = of_get_gpio(np, 0);
	ret = gpio_request(gpio, "nfc_int");
	if (ret) {
		pr_err("gpio_nfc_int request error\n");
		ret = -ENODEV;
		goto gpio_req_fail;
	}
	pn544_dev->irq_gpio = gpio;

	//FIRM
	gpio = of_get_gpio(np, 1);
	ret = gpio_request(gpio, "nfc_firm");
	if (ret) {
		pr_err("gpio_nfc_firm request error\n");
		ret = -ENODEV;
		goto gpio_req_fail;
	}
	gpio_direction_output(gpio, 0);
	pn544_dev->firm_gpio = gpio;

	//VEN
	gpio = of_get_gpio(np, 2);
	ret = gpio_request(gpio, "nfc_ven");
	if (ret) {
		pr_err("gpio_nfc_ven request error\n");
		ret = -ENODEV;
		goto gpio_req_fail;
	}
	gpio_direction_output(gpio, 0);
	pn544_dev->ven_gpio = gpio;

	pn544_dev->client = client;
	client->irq = gpio_to_irq(pn544_dev->irq_gpio);

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn544_dev_irq_handler,
			IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret) {
		dev_err(dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}

	wake_lock_init(&pn544_dev->wake_lock, WAKE_LOCK_SUSPEND, "nfc_wake");
	pn544_dev->pm_notifier.notifier_call = pn544_pm_callback;
	register_pm_notifier(&pn544_dev->pm_notifier);

	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);

	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
gpio_req_fail:
	if (pn544_dev->irq_gpio)
		gpio_free(pn544_dev->irq_gpio);
	if (pn544_dev->ven_gpio)
		gpio_free(pn544_dev->ven_gpio);
	if (pn544_dev->firm_gpio)
		gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	unregister_pm_notifier(&pn544_dev->pm_notifier);
	wake_lock_destroy(&pn544_dev->wake_lock);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);

	return 0;
}

static struct of_device_id pn547_dt_id[] = {
	{ .compatible = "nxp,pn547-nfc" },
	{ }
};

static const struct i2c_device_id pn544_id[] = {
	{ "pn547-nfc", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, pn544_id);
static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn547-nfc",
		.of_match_table = of_match_ptr(pn547_dt_id),
	},
};

module_i2c_driver(pn544_driver);

MODULE_AUTHOR("Liu Jianping<heljoy@meizu.com>");
MODULE_DESCRIPTION("NFC PN544 driver supported by NXP");
MODULE_LICENSE("GPL");
