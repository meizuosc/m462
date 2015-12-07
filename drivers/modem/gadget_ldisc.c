/******************************************************************************
  ACM over mux module driver

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
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/utsname.h>
#include <linux/module.h>


#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/tty.h>
#include <linux/tty_ldisc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define N_MODEM_MUX 12
static struct muxmaster
{
	struct mutex lock;
	struct tty_struct *mux_tty;
	unsigned int open_count;
	struct list_head pool;
	spinlock_t pool_lock;
}mux_master;

#define MAX_USB_PACKET_SIZE 0x800
struct _MODEM_MSG
{
	unsigned char usb_buf[MAX_USB_PACKET_SIZE];
	int usb_len;
	struct list_head link;
};

/* mux work queue and work */
static struct workqueue_struct *mux_wq;
static struct my_mux_work_struct_t
{
	struct work_struct mux_work;
}my_mux_work;

extern int gs_marvell_modem_send(const unsigned char *buf, int count, unsigned char cid);
typedef void (*marvell_modem_rx_callback)(unsigned char cid, char* packet, int len);
extern marvell_modem_rx_callback gs_marvell_modem_rx_psd_callback;
extern marvell_modem_rx_callback gs_marvell_modem_rx_csd_callback;

static void acm_write_work(struct work_struct* work)
{
	struct _MODEM_MSG* msg = NULL;
	unsigned long flags;
	mutex_lock(&mux_master.lock);
	if(!mux_master.mux_tty)
	{
		pr_err("%s: tty handle is NULL\n", __FUNCTION__);
		mutex_unlock(&mux_master.lock);
		return;
	}
	mutex_unlock(&mux_master.lock);
	while(!list_empty(&mux_master.pool))
	{
		spin_lock_irqsave(&mux_master.pool_lock, flags);
		msg = list_first_entry(&mux_master.pool, struct _MODEM_MSG, link);
		if(NULL == msg)
		{
			spin_unlock_irqrestore(&mux_master.pool_lock, flags);
			pr_err("%s: try to write a NULL msg\n", __FUNCTION__);
			return;
		}
		list_del(&msg->link);
		spin_unlock_irqrestore(&mux_master.pool_lock, flags);
		//pr_info("%s: datalen = %d (%s)\n", __FUNCTION__, usbLen, usbPacket);
		mux_master.mux_tty->ops->write(mux_master.mux_tty, msg->usb_buf, msg->usb_len);
		kfree(msg);
		msg = NULL;
	}
}

static void acm_write_over_mux(unsigned char cid, char* buf, int len)
{
	struct _MODEM_MSG* msg = NULL;
	unsigned long flags;
	if(buf == NULL || len <= 0 || len > MAX_USB_PACKET_SIZE)
	{
		pr_err("%s: error acm write\n", __FUNCTION__);
		return;
	}
	if(NULL == (msg = (kzalloc(sizeof(struct _MODEM_MSG), GFP_ATOMIC))))
	{
		pr_err("%s: malloc buf\n", __FUNCTION__);
		return;
	}
	INIT_LIST_HEAD(&msg->link);
	memcpy(msg->usb_buf, buf, len);
	msg->usb_len = len;
	spin_lock_irqsave(&mux_master.pool_lock, flags);
	list_add_tail(&msg->link, &mux_master.pool);
	spin_unlock_irqrestore(&mux_master.pool_lock, flags);
	//mux_master.mux_tty->ops->write(mux_master.mux_tty, usbPacket, usbLen);
	queue_work(mux_wq, &(my_mux_work.mux_work));
}

///////////////////////////////////////////////////////////////
/////CI OVER MUX
///////////////////////////////////////////////////////////////
static int modem_ldisc_open(struct tty_struct *tty)
{
       mutex_lock(&mux_master.lock);
       if(mux_master.open_count)
       {
               mux_master.open_count++;
               goto exit;
       }
       tty->disc_data = NULL;
       tty->receive_room = 65536;
       mux_master.mux_tty = tty;
       mux_master.open_count = 1;
exit:
       mutex_unlock(&mux_master.lock);
       return 0;
}

static void modem_ldisc_close(struct tty_struct *tty)
{
       mutex_lock(&mux_master.lock);
       if(mux_master.open_count != 1)
       {
               if (mux_master.open_count == 0)
                       WARN_ON(1);
               else
                       --mux_master.open_count;
               goto exit;
       }
       mux_master.open_count = 0;
       mux_master.mux_tty = NULL;
exit:
       mutex_unlock(&mux_master.lock);
}

static void modem_ldisc_wake(struct tty_struct *tty)
{
       printk(KERN_INFO "ts wake\n");
}

static ssize_t modem_ldisc_read(struct tty_struct *tty, struct file *file, unsigned char __user *buf, size_t nr)
{
       return 0;
}

static ssize_t modem_ldisc_write(struct tty_struct *tty, struct file *file, const unsigned char *data, size_t count)
{
       return 0;
}

static void modem_ldisc_rx(struct tty_struct *tty, const u8 *data, char *flag, int size)
{
	gs_marvell_modem_send(data, size, 0);
	//pr_info("%s: size = %d (%s)\n", __FUNCTION__, size, data);
}

static int modem_ldisc_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
       return 0;
}
static struct tty_ldisc_ops modem_ldisc = {
   .owner         = THIS_MODULE,
   .magic         = TTY_LDISC_MAGIC,
   .name          = "modem over mux",
   .open          = modem_ldisc_open,
   .close         = modem_ldisc_close,
   .read          = modem_ldisc_read,
   .write         = modem_ldisc_write,
   .receive_buf   = modem_ldisc_rx,
   .write_wakeup  = modem_ldisc_wake,
   .ioctl         = modem_ldisc_ioctl,
};

static int __init gs_module_init(void)
{
	int status = -1;

	mutex_init(&mux_master.lock);
	spin_lock_init(&mux_master.pool_lock);
	INIT_LIST_HEAD(&mux_master.pool);
	mux_master.open_count = 0;
	mux_wq = create_workqueue("modem-mux");
	if (!mux_wq)
	{
		pr_err(KERN_ERR "%s: can't create workqueue\n", __func__);
		return -ENOMEM;
	}
	INIT_WORK(&(my_mux_work.mux_work), acm_write_work);

	gs_marvell_modem_rx_psd_callback = acm_write_over_mux;
	gs_marvell_modem_rx_csd_callback = acm_write_over_mux;

	status = tty_register_ldisc(N_MODEM_MUX, &modem_ldisc);
	if(status)
	{
		pr_err("%s: cannot register ldisc, err %d\n", __func__, status);
		return -ENOMEM;
	}

	pr_info("init modem over mux driver\n");
	return 0;
}

static void __exit gs_module_exit(void)
{
	gs_marvell_modem_rx_psd_callback = (marvell_modem_rx_callback)NULL;
	gs_marvell_modem_rx_csd_callback = (marvell_modem_rx_callback)NULL;
}

/* Module */
MODULE_DESCRIPTION("Marvell USB Modem");
MODULE_LICENSE("GPL");

module_init(gs_module_init);
module_exit(gs_module_exit);
