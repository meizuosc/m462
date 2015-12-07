/*
    Copyright (C) 2014 Marvell International Ltd.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/module.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include<linux/random.h>
#include<linux/delay.h>
#include <linux/skbuff.h>
#include<linux/workqueue.h>

#include "audio_stub.h"
#include "modem_state_notify.h"

struct notify_event {
	struct list_head link;
	int event;
};

struct notify_work {
	struct work_struct work;
	struct workqueue_struct *notify_wq;
	struct list_head events;
	spinlock_t events_lock;
};
static struct notify_work notify_wk;
static struct tty_struct *lltty;

static int audio_ldisc_open(struct tty_struct *tty)
{
	lltty = tty;
	tty->receive_room = 65536;
	pr_debug("**********************lltty index = %d\n", tty->index);
	return 0;
}

static void audio_ldisc_close(struct tty_struct *tty)
{
	lltty = NULL;
}

static void audio_ldisc_wake(struct tty_struct *tty)
{
	pr_debug("ts wake\n");
}

static ssize_t audio_ldisc_read(struct tty_struct *tty, struct file *file,
				unsigned char __user *buf, size_t nr)
{
	return 0;
}

static ssize_t audio_ldisc_write(struct tty_struct *tty, struct file *file,
				 const unsigned char *data, size_t count)
{
	return 0;
}

static void audio_ldisc_rx(struct tty_struct *tty, const unsigned char *data,
			   char *flags, int size)
{
	struct sk_buff *skb;
	pr_info("%s: size:%d\n", __func__, size);
	skb = alloc_skb(size, GFP_KERNEL);
	if (!skb) {
		pr_err("%s: out of memory.\n", __func__);
		return;
	}
	memcpy(skb_put(skb, size), data, size);
	audio_data_handler(skb);
}

static int audio_ldisc_ioctl(struct tty_struct *tty, struct file *file,
			     unsigned int cmd, unsigned long arg)
{
	return 0;
}

static struct tty_ldisc_ops audio_ldisc = {
	.owner = THIS_MODULE,
	.magic = TTY_LDISC_MAGIC,
	.name = "audio over HSI",
	.open = audio_ldisc_open,
	.close = audio_ldisc_close,
	.read = audio_ldisc_read,
	.write = audio_ldisc_write,
	.receive_buf = audio_ldisc_rx,
	.write_wakeup = audio_ldisc_wake,
	.ioctl = audio_ldisc_ioctl,
};

void register_audio_ldisc(void)
{
	if (tty_register_ldisc(N_LDIS_AUDIO, &audio_ldisc))
		pr_err("oops. cant register ldisc\n");
}

ssize_t send_data_low_level(char *data, int len)
{
	int ret = 0;
	if (likely(len > 0)) {
		if (lltty && lltty->ops->write)
			ret = lltty->ops->write(lltty, data, len);
		else
			ret = -ENODEV;
	}
	return ret;
}

void broadcast_linkstatus(int on)
{
	struct sk_buff *skb;
	struct atc_header *msg;
	int size = 0;
	skb = alloc_skb(32, GFP_ATOMIC);
	if (!skb) {
		pr_err("%s: out of memory.\n", __func__);
		return;
	}

	size = sizeof(*msg);
	msg = (struct atc_header *)skb_put(skb, size);
	msg->cmd_code = AUDIO_CMD_CODE;
	msg->cmd_type = CMD_TYPE_INDICATION;
	msg->data_len = 0;
	if (on)
		msg->sub_cmd = ATC_MSOCKET_LINKUP;
	else
		msg->sub_cmd = ATC_MSOCKET_LINKDOWN;

	audio_data_handler(skb);
}

void audio_notify_event_work(struct work_struct *work)
{
	unsigned long flags;
	struct notify_work *wk = container_of(work, struct notify_work, work);
	while (!list_empty(&wk->events)) {
		struct notify_event *ny_event = NULL;
		int event = -1;
		spin_lock_irqsave(&wk->events_lock, flags);
		ny_event =
		    list_first_entry(&wk->events, struct notify_event, link);
		if (NULL == ny_event) {
			spin_unlock_irqrestore(&wk->events_lock, flags);
			pr_err("%s: try to handle a NULL event\n", __func__);
			return;
		}
		list_del(&ny_event->link);
		spin_unlock_irqrestore(&notify_wk.events_lock, flags);
		event = ny_event->event;
		kfree(ny_event);
		switch (event) {
		case HOLD:
			pr_info("%s: HOLD notify event handled\n", __func__);
			broadcast_linkstatus(0);
			break;
		case RELEASE:
			pr_info("%s: RELEASE notify event handled\n", __func__);
			broadcast_linkstatus(1);
			break;
		default:
			pr_info("%s: %s ignored\n", __func__,
				get_modem_state_name(event));
			break;
		}
	}
}

static int audio_modem_state_event(struct notifier_block *this,
				   unsigned long event, void *ptr)
{
	struct notify_event *ny_event = NULL;
	unsigned long flags;
	pr_info("%s: get modem event:%s\n", __func__,
		get_modem_state_name(event));

	ny_event = kmalloc(sizeof(struct notify_event), GFP_ATOMIC);
	if (NULL == ny_event) {
		pr_err("%s: fail to alloc notify_event\n", __func__);
		return -ENOMEM;
	}
	ny_event->event = event;
	spin_lock_irqsave(&notify_wk.events_lock, flags);
	list_add_tail(&ny_event->link, &notify_wk.events);
	spin_unlock_irqrestore(&notify_wk.events_lock, flags);
	queue_work(notify_wk.notify_wq, &notify_wk.work);
	return 0;
}

static struct notifier_block hsitty_dev_notifier = {
	.notifier_call = audio_modem_state_event,
};

void audio_register_modem_state_notifier(void)
{
	register_modem_state_notifier(&hsitty_dev_notifier);
	INIT_WORK(&notify_wk.work, audio_notify_event_work);
	notify_wk.notify_wq = create_workqueue("audio notify work queue");
	if (notify_wk.notify_wq == NULL)
		pr_info("%s:Can't create work queue!\n", __func__);

	spin_lock_init(&notify_wk.events_lock);
	INIT_LIST_HEAD(&notify_wk.events);
}

void audio_unregister_modem_state_notifier(void)
{
	unregister_modem_state_notifier(&hsitty_dev_notifier);
	if (notify_wk.notify_wq)
		destroy_workqueue(notify_wk.notify_wq);
}
