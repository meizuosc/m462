/*
 * Motorola TS0710 driver.
 *
 * Copyright (C) 2002-2004  Motorola
 * Copyright (C) 2006 Harald Welte <laforge@openezx.org>
 * Copyright (C) 2009 Ilya Petrov <ilya.muromec@gmail.com>
 * Copyright (C) 2009 Daniel Ribeiro <drwyrm@gmail.com>
 *
 * Portions derived from rfcomm.c, original header as follows:
 *
 * Copyright (C) 2000, 2001  Axis Communications AB
 *
 * Author: Mats Friden <mats.friden@axis.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Exceptionally, Axis Communications AB grants discretionary and
 * conditional permissions for additional use of the text contained
 * in the company's release of the AXIS OpenBT Stack under the
 * provisions set forth hereunder.
 *
 * Provided that, if you use the AXIS OpenBT Stack with other files,
 * that do not implement functionality as specified in the Bluetooth
 * System specification, to produce an executable, this does not by
 * itself cause the resulting executable to be covered by the GNU
 * General Public License. Your use of that executable is in no way
 * restricted on account of using the AXIS OpenBT Stack code with it.
 *
 * This exception does not however invalidate any other reasons why
 * the executable file might be covered by the provisions of the GNU
 * General Public License.
 *
 */
#include <linux/module.h>
#include <linux/types.h>

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/serial.h>

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/skbuff.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/mach-types.h>

#include "ts0710.h"
#include "ts0710_mux.h"

static void tx_worker(struct work_struct *work);

static int NR_MUXS;
#define MUX_NUM TS0710_MAX_CHN
static struct list_head tx_mux_head;
static struct workqueue_struct *tx_work_queue;
struct ts0710_mux_queue * mux_queue_array[MUX_NUM+1];
typedef struct{
	u32 queue_length;
	spinlock_t tx_queue_lock;
} queue_state;
typedef struct{
	bool work_running;
	spinlock_t work_lock;
}work_state;
static const int dlci_prority[MUX_NUM]={
	0,
	10,
	10,
	10,
	10,
	10,
	10,
	10,
	10,
	10,
	10,
	50,
	50,
	50,
	50,
	50,
	50,
	50,
	50,
	50,
	50,
	40,
	60,
	30,
	10,
	70

};

static queue_state mux_queue_state;
static work_state tx_work_state;

static DECLARE_WORK(tx_work, tx_worker);

static struct tty_struct *ipc_tty;

static int simple_dump = 1;
static int mux_backup = 1;
static int tx_dump_map = 0;
static int rx_dump_map = 0;
DEFINE_SPINLOCK(dump_lock);


typedef struct {
	u8 cmdtty;
	u8 datatty;
} dlci_tty;

static u8 *iscmdtty;

typedef struct {
	volatile u8 buf[TS0710MUX_SEND_BUF_SIZE];
	volatile u8 *frame;
	unsigned long flags;
	volatile u16 length;
	volatile u8 filled;
	volatile u8 dummy;	/* Allignment to 4*n bytes */
} mux_send_struct;

#define MAX_PACKET_SIZE 2048
struct mux_data {
	enum {INSIDE_PACKET, OUT_OF_PACKET} state;
	size_t chunk_size;
	unsigned char chunk[MAX_PACKET_SIZE];
};

/* Bit number in flags of mux_send_struct */

struct mux_recv_packet_tag {
	u8 *data;
	u32 length;
	struct mux_recv_packet_tag *next;
};
typedef struct mux_recv_packet_tag mux_recv_packet;

struct mux_recv_struct_tag {
	u8 data[TS0710MUX_RECV_BUF_SIZE];
	u32 length;
	u32 total;
	mux_recv_packet *mux_packet;
	struct mux_recv_struct_tag *next;
	int no_tty;
	volatile u8 post_unthrottle;
};

struct mux_device{
	struct tty_port	port;
};
typedef struct mux_recv_struct_tag mux_recv_struct;

static unsigned long mux_recv_flags = 0;

static mux_send_struct *mux_send_info[MUX_NUM];
static volatile u8 mux_send_info_flags[MUX_NUM];
static volatile u8 mux_send_info_idx = MUX_NUM;

static mux_recv_struct *mux_recv_info[MUX_NUM];
static volatile u8 mux_recv_info_flags[MUX_NUM];
static mux_recv_struct *mux_recv_queue = NULL;

/* Local for 2.6? */
static struct tty_driver *mux_driver;

static struct tty_struct *mux_table[MUX_NUM];
static struct mux_device mux_device_table[MUX_NUM];

static volatile short int mux_tty[MUX_NUM];
static struct file *file;

#define MUX_DUMP

#ifdef MUX_DUMP
static struct sk_buff_head mux_backup_queue[2][MUX_NUM];
static void backup_queue_init(void);
static void backup_mux_dump(u32 dlci, int direction);
static void backup_mux_log(u32 dlci, int direction, char * data, u32 length);
#else
static void backup_queue_init(void) { }
static void backup_mux_dump(u32 dlci, int direction) { }
static void backup_mux_log(u32 dlci, int direction, char * data, u32 length) { }
#endif

#ifdef min
#undef min
#define min(a, b)	((a) < (b) ? (a) : (b))
#endif

static ts0710_con ts0710_connection;
static void hex_packet(unsigned char *p, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		if (i && (i % 16) == 0)
			printk("\n");
		printk("%02X ", *p++);
	}
	printk("\n");
}

void schedule_tx(void)
{
	queue_work(tx_work_queue, &tx_work);
}

static  int tx_in_queue(u32 dlci,u8* frame, u32 length, bool block)
{
	ts0710_tx_node * tx_node = kmalloc(sizeof(ts0710_tx_node),GFP_ATOMIC);
	struct ts0710_mux_queue *mux_queue;
	struct ts0710_mux_queue *next_queue;
	unsigned long flags;

	if(NULL == tx_node)
	{
		TS0710_PRINTK("No mem for tx_node\n");
		return -ENOMEM;
	}
	tx_node->dlci = dlci;
	tx_node->length = length;
	tx_node->data = frame;
	mux_queue = mux_queue_array[dlci];
	if(NULL == mux_queue)
	{
		kfree(tx_node);
		TS0710_PRINTK("mux_queue[%d] has been removed!\n",dlci);
		return -ENODEV;
	}
	spin_lock_irqsave(&mux_queue->lock, flags);
	while(mux_queue->length >= 100)
	{
		//TS0710_PRINTK("We have to drop the data in DLCI:%d, queue_length is %d\n", dlci, mux_queue->length);
		if(block)
		{
			int ret;
			mux_queue->status |= MUXQ_STATUS_XMIT_FULL;
			spin_unlock_irqrestore(&mux_queue->lock, flags);
			//TS0710_PRINTK("Tx queue %d is full, wait!\n",dlci);
			ret = wait_event_interruptible(mux_queue->tx_wq, !(mux_queue->status & MUXQ_STATUS_XMIT_FULL));
			if(ret)
			{
				kfree(tx_node);
				TS0710_PRINTK("Interrput by signal:%d!\n",ret);
				return -ERESTARTSYS;
			}
			spin_lock_irqsave(&mux_queue->lock, flags);
			//TS0710_PRINTK("Tx queue %d is waked up!, length:%d\n",dlci,mux_queue->length);
		}
		else
		{
			ts0710_tx_node * drop_node;
			drop_node =list_entry(mux_queue->tx_q_head.next, ts0710_tx_node, queue_list);
			list_del(&drop_node->queue_list);
			mux_queue->length--;
			kfree(drop_node->data);
			kfree(drop_node);
			mux_queue->stat_tx_drop++;
		}
	}
	list_add_tail(&tx_node->queue_list, &mux_queue->tx_q_head);
	mux_queue->length++;
	if (!(mux_queue->status & MUXQ_STATUS_XMIT_QUEUED))
	{
		mux_queue->status |= MUXQ_STATUS_XMIT_QUEUED;
		spin_unlock_irqrestore(&mux_queue->lock, flags);

		spin_lock_irqsave(&mux_queue_state.tx_queue_lock, flags);
		list_for_each_entry(next_queue, &tx_mux_head, tx_list)
		{
			if(next_queue->priority > mux_queue->priority)
				break;
		}
		list_add_tail(&mux_queue->tx_list, &next_queue->tx_list);
		spin_unlock_irqrestore(&mux_queue_state.tx_queue_lock, flags);
	}
	else
		spin_unlock_irqrestore(&mux_queue->lock, flags);

	spin_lock_irqsave(&tx_work_state.work_lock,flags);
	if(!tx_work_state.work_running)
	{
		tx_work_state.work_running = true;
		schedule_tx();
	}
	spin_unlock_irqrestore(&tx_work_state.work_lock,flags);

	return length;
}
static void tx_worker(struct work_struct *work)
{
	ts0710_tx_node * tx_node;
	ts0710_con *ts0710 = &ts0710_connection;

	struct ts0710_mux_queue *mux_queue;
	struct ts0710_mux_queue *next_queue;

	int num = 0;
	int priority = 0;
	int total = 0;
	int dlci;
	while(1)
	{
		spin_lock_irq(&mux_queue_state.tx_queue_lock);
		num = 0;
		priority = 0;

		list_for_each_entry_safe(mux_queue, next_queue, &tx_mux_head, tx_list)
		{
			spin_lock(&mux_queue->lock);
			dlci = mux_queue->dlci;
			if (ts0710->dlci[0].state == FLOW_STOPPED) {
				TS0710_DEBUG("Flow stopped on all channels,continue\n");
				spin_unlock(&mux_queue->lock);
				continue;
			} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
				TS0710_DEBUG("Flow stopped, continue\n");
				spin_unlock(&mux_queue->lock);
				continue;
			}


			if (!priority)
			{
				priority = mux_queue->priority;
			}
			else if(priority != mux_queue->priority)
			{
				spin_unlock(&mux_queue->lock);
				break;
			}
			//else
			{
				tx_node = list_entry(mux_queue->tx_q_head.next, ts0710_tx_node, queue_list);
				//ipc_tty->ops->write(ipc_tty,tx_node->data,tx_node->length);
				num++;
				total++;
				list_del(&tx_node->queue_list);
				mux_queue->length--;
				mux_queue->stat_tx_delt_bytes += tx_node->length;
				//kfree(tx_node->data);
				//kfree(tx_node);
				if(mux_queue->status |= MUXQ_STATUS_XMIT_FULL)
				{
					mux_queue->status &= ~MUXQ_STATUS_XMIT_FULL;
					wake_up_interruptible(&mux_queue->tx_wq);
					//TS0710_PRINTK("Tx queue %d is not full, wake up!\n",mux_queue->dlci);
				}
			}
			if(list_empty_careful(&mux_queue->tx_q_head))
			{
				mux_queue->status &=~MUXQ_STATUS_XMIT_QUEUED;
				list_del(&mux_queue->tx_list);
			}
			spin_unlock(&mux_queue->lock);

			spin_unlock_irq(&mux_queue_state.tx_queue_lock);
			if(ipc_tty)
			{
				ipc_tty->ops->write(ipc_tty,tx_node->data,tx_node->length);
				backup_mux_log(dlci,1, tx_node->data, tx_node->length);
				if(tx_node->dlci != 0)
				{

					if(tx_dump_map & (1 << (tx_node->dlci - 1)))
					{
						spin_lock(&dump_lock);
						printk("dlc:%d tx %d bytes data!--\n", tx_node->dlci, tx_node->length);
						if(!simple_dump)
						{
							hex_packet(tx_node->data,tx_node->length);
						}
						spin_unlock(&dump_lock);
					}
				}
			}
			spin_lock_irq(&mux_queue_state.tx_queue_lock);

			kfree(tx_node->data);
			kfree(tx_node);
		}
		spin_lock(&tx_work_state.work_lock);
		if(num == 0)
		{
			tx_work_state.work_running = false;
			spin_unlock(&tx_work_state.work_lock);
			spin_unlock_irq(&mux_queue_state.tx_queue_lock);
			return;
		}
		spin_unlock(&tx_work_state.work_lock);
		if(total > 64)
		{
			schedule_tx();
			spin_unlock_irq(&mux_queue_state.tx_queue_lock);
			return;
		}
		spin_unlock_irq(&mux_queue_state.tx_queue_lock);
	}
}

static int tx_queue_init(void)
{
	int i;
	spin_lock_init(&tx_work_state.work_lock);
	spin_lock_init(&mux_queue_state.tx_queue_lock);
	tx_work_state.work_running = false;
	INIT_LIST_HEAD(&tx_mux_head);

	tx_work_queue = create_workqueue("mux_work");
	if(!tx_work_queue)
	{
		TS0710_PRINTK("cant not create work queue!\n");
		return -1;
	}
	for(i = 0; i <= MUX_NUM; i++)
		mux_queue_array[i] = NULL;
	return 0;
}
static int mux_queue_init(u32 dlci)
{
	struct ts0710_mux_queue *mux_queue;

	spin_lock_irq(&mux_queue_state.tx_queue_lock);
	TS0710_DEBUG("DLCI:%d init queue!\n",dlci);
	if(mux_queue_array[dlci])
	{
		spin_unlock_irq(&mux_queue_state.tx_queue_lock);
		TS0710_DEBUG("DLCI:%d has already created the mux queue\n",dlci );
		return -EBUSY;
	}
	spin_unlock_irq(&mux_queue_state.tx_queue_lock);
	mux_queue = kmalloc(sizeof(*mux_queue), GFP_KERNEL);
	if(!mux_queue)
	{

		return -ENOMEM;
	}
	INIT_LIST_HEAD(&mux_queue->tx_q_head);
	spin_lock_init(&mux_queue->lock);
	mux_queue->dlci = dlci;
	mux_queue->priority = dlci_prority[dlci];
	mux_queue->status = 0;
	mux_queue->length = 0;
	init_waitqueue_head(&mux_queue->tx_wq);
	mux_queue->stat_tx_drop = 0;
	mux_queue->stat_tx_delt_bytes = 0;
	mux_queue->stat_rx_delt_bytes = 0;
	mux_queue->stat_tx_delt_time = jiffies;
	TS0710_DEBUG("DLCI:mux_queue_array[%d] init queue: %x!\n",dlci,mux_queue);
	spin_lock_irq(&mux_queue_state.tx_queue_lock);
	mux_queue_array[dlci] = mux_queue;
	spin_unlock_irq(&mux_queue_state.tx_queue_lock);

	return 0;
}
static void mux_queue_deinit(u32 dlci)
{
	struct ts0710_mux_queue *mux_queue;
	ts0710_tx_node * tx_node, *next_node;

	spin_lock_irq(&mux_queue_state.tx_queue_lock);
	mux_queue = mux_queue_array[dlci];
	spin_lock(&mux_queue->lock);
	if ((mux_queue->status & MUXQ_STATUS_XMIT_QUEUED))
	{
		list_del(&mux_queue->tx_list);
	}
	list_for_each_entry_safe(tx_node, next_node, &mux_queue->tx_q_head, queue_list)
	{
		list_del(&tx_node->queue_list);
		kfree(tx_node->data);
		kfree(tx_node);
	}
	spin_unlock(&mux_queue->lock);
	mux_queue_array[dlci] = NULL;
	spin_unlock_irq(&mux_queue_state.tx_queue_lock);
	kfree(mux_queue);
}

#ifdef MUX_DUMP
static unsigned long long get_kernel_time(void)
{
	int this_cpu;
	unsigned long flags;
	unsigned long long time;

	preempt_disable();
	raw_local_irq_save(flags);

	this_cpu = smp_processor_id();
	time = cpu_clock(this_cpu);

	preempt_enable();
	raw_local_irq_restore(flags);

	return time;
}

static void backup_queue_init(void)
{
	int dlci;
	int direction;
	for(direction = 0; direction < 2; direction ++)
	{
		for(dlci = 0; dlci < MUX_NUM; dlci ++)
		{
			skb_queue_head_init(&mux_backup_queue[direction][dlci]);
		}
	}
}

static void backup_mux_log(u32 dlci, int direction, char * data, u32 length)
{
	struct sk_buff_head * queue = &mux_backup_queue[direction][dlci];
	struct sk_buff *skb;
	ts0710_backup_header * hdr;
	if(!mux_backup)
		return;
	if(length ==0 || queue == NULL || data == NULL)
		return;

	skb = dev_alloc_skb(sizeof(ts0710_backup_header) + length);
	if (!skb) {
		TS0710_LOG(" ERR!   dev_alloc_skb fail\n");
		return;
	}
	hdr = (ts0710_backup_header *)skb_put(skb, sizeof(ts0710_backup_header));
	hdr->length = length;
	hdr->magic = 0x78563412;
	hdr->time = get_kernel_time();
	memcpy(skb_put(skb, length), data, length);
	while(skb_queue_len(queue) > 50)
	{
		struct sk_buff *skb = skb_dequeue(queue);
		dev_kfree_skb(skb);
	}
	skb_queue_tail(queue, skb);
}

static struct file *mux_open_file(const char *path)
{
	struct file *fp;

	fp = filp_open(path, O_RDWR|O_CREAT|O_TRUNC, 0666);

	if (IS_ERR(fp))
		return NULL;

	return fp;
}

static void backup_mux_dump(u32 dlci, int direction)
{
	struct sk_buff *skb;
	char path[128];
	struct file * fp;
	mm_segment_t old_fs;
	struct sk_buff_head * queue = &mux_backup_queue[direction][dlci];

	if(!mux_backup)
		return;

	sprintf(path, "/data/mux_%s%02d.bin", direction?"tx":"rx",dlci);

	TS0710_DEBUG(" Beging dump to file %s ...\n",path);
	old_fs = get_fs();
	set_fs(get_ds());

	fp = mux_open_file(path);
	if(!fp)
		goto exit;

	while( (skb = skb_dequeue(queue)) != NULL)
	{
		int ret = fp->f_op->write(fp, skb->data, skb->len, &fp->f_pos);
		dev_kfree_skb(skb);
		if(ret < 0)
		{
			TS0710_LOG(" ERR!   dump %s fail\n", path);
		}
	}
	filp_close(fp, NULL);
exit:
	set_fs(old_fs);
	TS0710_DEBUG(" End dump to file %s\n",path);

}
#endif

#define PROC_FILE_NAME		"driver/ts27010"
static void *mux_seq_start(struct seq_file *s, loff_t * pos)
{
	int i = *pos;

	spin_lock_irq(&mux_queue_state.tx_queue_lock);

	if (!i)
		return SEQ_START_TOKEN;

	while (i <= MUX_NUM && !mux_queue_array[i])
		i++;

	/* return a non null value to begin the sequence */
	return i > MUX_NUM ? NULL : (*pos = i, mux_queue_array[i]);
}
static void *mux_seq_next(struct seq_file *s, void *v, loff_t * pos)
{
	int i = *pos + 1;
	while (i <= MUX_NUM  && !mux_queue_array[i])
		i++;

	/* return a non null value to step the sequence */
	return i > MUX_NUM ? NULL : (*pos = i, mux_queue_array[i]);
}
static void mux_seq_stop(struct seq_file *s, void *v)
{
	spin_unlock_irq(&mux_queue_state.tx_queue_lock);
}
static int mux_seq_show(struct seq_file *s, void *v)
{
	struct ts0710_mux_queue *mux_queue = (struct ts0710_mux_queue *)v;

	if (v == SEQ_START_TOKEN)
	{
		seq_puts(s, "\ndlci  ");
		seq_puts(s, "stat_tx_drop  queue_length txspeed(bytes/s) rxspeed(bytes/s)\n");
	}
	else
	{
		u32 time;
		u32 txspeed;
		u32 rxspeed;
		spin_lock(&mux_queue->lock);
		time = jiffies - mux_queue->stat_tx_delt_time;
		txspeed = (mux_queue->stat_tx_delt_bytes / time)*100;
		rxspeed = (mux_queue->stat_rx_delt_bytes / time)*100;
		txspeed += (mux_queue->stat_tx_delt_bytes % time)*100 / time;
		rxspeed += (mux_queue->stat_rx_delt_bytes % time)*100 / time;
		seq_printf(s, "%4d", mux_queue->dlci);
		seq_printf(s, "%9ud", mux_queue->stat_tx_drop);
		seq_printf(s, "%14ud", mux_queue->length);
		seq_printf(s, "%14ud", txspeed);
		seq_printf(s, "%14ud\n", rxspeed);
		mux_queue->stat_tx_delt_time = jiffies;
		mux_queue->stat_tx_delt_bytes = 0;
		mux_queue->stat_rx_delt_bytes = 0;
		spin_unlock(&mux_queue->lock);
	}

	return 0;
}
static struct seq_operations mux_seq_ops = {
	.start = mux_seq_start,
	.next = mux_seq_next,
	.stop = mux_seq_stop,
	.show = mux_seq_show
};
static int mux_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &mux_seq_ops);
};
static struct file_operations mux_proc_fops = {
	.owner = THIS_MODULE,
	.open = mux_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};
static void ts0710_reset_dlci(u8 j)
{
	if (j >= TS0710_MAX_CHN)
		return;

	ts0710_connection.dlci[j].state = DISCONNECTED;
	ts0710_connection.dlci[j].flow_control = 0;
	ts0710_connection.dlci[j].mtu = DEF_TS0710_MTU;
	ts0710_connection.dlci[j].initiated = 0;
	ts0710_connection.dlci[j].initiator = 0;
   	// ts0710_connection.dlci[j].tag = CMDTAG;
       ts0710_connection.dlci[j].tag = 0;
	//init_waitqueue_head(&ts0710_connection.dlci[j].open_wait);
	//init_waitqueue_head(&ts0710_connection.dlci[j].close_wait);
}

static void ts0710_reset_con(void)
{
	u8 j;

	ts0710_connection.initiator = 0;
	ts0710_connection.mtu = DEF_TS0710_MTU + TS0710_MAX_HDR_SIZE;
	ts0710_connection.be_testing = 0;
	ts0710_connection.test_errs = 0;
	init_waitqueue_head(&ts0710_connection.test_wait);

	for (j = 0; j < TS0710_MAX_CHN; j++)
		ts0710_reset_dlci(j);
}

static void ts0710_init(void)
{
	int j = 0;
	fcs_init();

	ts0710_reset_con();

	for (j = 0; j < TS0710_MAX_CHN; j++) {
		init_waitqueue_head(&ts0710_connection.dlci[j].open_wait);
		init_waitqueue_head(&ts0710_connection.dlci[j].close_wait);
	}
}

static void ts0710_upon_disconnect(void)
{
	ts0710_con *ts0710 = &ts0710_connection;
	u8 j;

	for (j = 0; j < TS0710_MAX_CHN; j++) {
		ts0710->dlci[j].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[j].open_wait);
		wake_up_interruptible(&ts0710->dlci[j].close_wait);
	}
	ts0710->be_testing = 0;
	wake_up_interruptible(&ts0710->test_wait);
	ts0710_reset_con();
}

/* Sending packet functions */

/* See TS 07.10's Section 5.2.1.6 */
static u_int8_t fcs_table[0x100];
static void fcs_init(void)
{
	int i, bit;
	u8 reg, nreg;

	for (i = 0; i < 0x100; i++) {
		for (reg = bit = 0; bit < 8; bit++, reg = nreg) {
			nreg = reg >> 1;
			if (((i >> bit) ^ reg) & 1)
				nreg ^= 0xe0;	/* x^8 + x^2 + x + 1 */
		}

		fcs_table[i] = reg;
	}
}

/* Computes the FCS checksum for a chunk of data.  */
static u_int8_t mux_fcs_compute(const u8 payload[], int len)
{
	u8 gen_reg;

	gen_reg = ~0;
	while (len--)
		gen_reg = fcs_table[gen_reg ^ *(payload++)];

	return ~gen_reg;
}

/* Returns 1 if the given chunk of data has a correct FCS appended.  */
/*static int mux_fcs_check(const u8 payload[], int len)
{
	return mux_fcs_compute(payload, len + 1) == 0xcf;
}*/

/* See TS 07.10's Section 5.2.1 */
static int mux_send_frame(u8 dlci, int initiator,
		enum mux_frametype frametype, const u8 data[], int len)
{
	u8 *framebuf = NULL;
	int pos = 0;
	int pf, crc_len, res;
	int cr = initiator & 0x1;
	framebuf = kmalloc(len+32, GFP_KERNEL);
	TS0710_DEBUG("Enter mux_send_frame:send to %d\n",dlci);
	if(!framebuf){
		printk("cant send frame to closed device\n");
		return -ENOMEM;
	}
        if (!ipc_tty) {
	  kfree(framebuf);
          printk("cant send frame to closed device\n");
          return -ENODEV;
        }

	/* FIXME: bitmask? */
	switch (frametype) {
		case MUX_UIH:
		case ACK:
			pf = 0;
			crc_len = len;
			break;
		case MUX_UI:
			pf = 0;
			crc_len = 0;
			break;
		default:
			pf = 1;
			crc_len = 0;
	}

	/* if (dlc->muxer->option == mux_option_basic) */
		framebuf[pos++] = MUX_BASIC_FLAG_SEQ;
	/* else
		framebuf[pos++] = MUX_ADVANCED_FLAG_SEQ; */

	/* Address field.  */
	framebuf[pos++] = MUX_EA | (cr << 1) | (dlci << 2);

	/* Control field.  */
	framebuf[pos++] = frametype | (pf << 4);

	/* Length indicator.  */
	/* if (dlc->muxer->option == mux_option_basic) { */
		if (len & ~0x7f) {
			framebuf[pos ++] = 0 | ((len & 0x7f) << 1);
			framebuf[pos ++] = len >> 7;
		} else
			framebuf[pos ++] = 1 | (len << 1);
	/* } */

	/* Information field.  */
	if (len)
		memcpy(&framebuf[pos], data, len);
	pos += len;

	framebuf[pos] = mux_fcs_compute(framebuf + 1, pos - 1 - crc_len);
	pos ++;

	/*if (dlc->muxer->option == mux_option_advanced)
		pos = mux_frame_escape(framebuf + 1, pos - 1) + 1;

	if (dlc->muxer->option == mux_option_basic) */
		framebuf[pos ++] = MUX_BASIC_FLAG_SEQ;
	/* else
		framebuf[pos ++] = MUX_ADVANCED_FLAG_SEQ; */

/*	if(!dlci || (frametype != MUX_UIH))
	{
		res = ipc_tty->ops->write(ipc_tty, framebuf, pos);

	}
	else*/
	{
		TS0710_DEBUG("Put in the Queue!\n");
		res = tx_in_queue(dlci, framebuf, pos, true);

	}
	if (res != pos) {
		TS0710_DEBUG("mux_send_frame error %d\n", res);
		return -1;
	}

	return res;
}

/* Creates a UA packet and puts it at the beginning of the pkt pointer */

static void send_ua(ts0710_con * ts0710, u8 dlci)
{
	mux_send_frame(dlci, !ts0710->initiator, MUX_UA, 0, 0);
}

/* Creates a DM packet and puts it at the beginning of the pkt pointer */

static void send_dm(ts0710_con * ts0710, u8 dlci)
{
	mux_send_frame(dlci, !ts0710->initiator, MUX_DM, 0, 0);
}

static void send_sabm(ts0710_con * ts0710, u8 dlci)
{
	mux_send_frame(dlci, ts0710->initiator, MUX_SABM, 0, 0);
}

static void send_disc(ts0710_con * ts0710, u8 dlci)
{
	mux_send_frame(dlci, !ts0710->initiator, MUX_DISC, 0, 0);
}

/* Multiplexer command packets functions */

/* Turns on the ts0710 flow control */

static void ts0710_fcon_msg(ts0710_con * ts0710, u8 cr)
{
	mux_send_uih(ts0710, cr, FCON, 0, 0);
}

/* Turns off the ts0710 flow control */

static void ts0710_fcoff_msg(ts0710_con * ts0710, u8 cr)
{
	mux_send_uih(ts0710, cr, FCOFF, 0, 0);
}


/* Sends an PN-messages and sets the not negotiable parameters to their
   default values in ts0710 */

static void send_pn_msg(ts0710_con * ts0710, u8 prior, u32 frame_size,
		       u8 credit_flow, u8 credits, u8 dlci, u8 cr)
{
	u8 data[8];
	pn_t *send = (pn_t *)data;

	send->res1 = 0;
	send->res2 = 0;
	send->dlci = dlci;
	send->frame_type = 0;
	send->credit_flow = credit_flow;
	send->prior = prior;
	send->ack_timer = 0;

	send->frame_sizel = frame_size & 0xFF;
	send->frame_sizeh = frame_size >> 8;

	send->credits = credits;
	send->max_nbrof_retrans = 0;
	TS0710_DEBUG("MUX DLCI:%d Send pn_msg !\n",dlci);
	mux_send_uih(ts0710, cr, PN, data, 8);
}

/* Send a Not supported command - command, which needs 3 bytes */

static void send_nsc_msg(ts0710_con * ts0710, mcc_type cmd, u8 cr)
{
	mux_send_uih(ts0710, cr, NSC, (u8 *) &cmd, 1);
}

static void mux_send_uih(ts0710_con * ts0710, u8 cr, u8 type, u8 *data, int len)
{
	u8 *send = kmalloc(len + 2, GFP_ATOMIC);

	mcc_short_frame_head *head = (mcc_short_frame_head *)send;
	head->type.ea = 1;
	head->type.cr = cr;
	head->type.type = type;
	head->length.ea = 1;
	head->length.len = len;

	if (len)
		memcpy(send + 2, data, len);

	mux_send_frame(CTRL_CHAN, ts0710->initiator, MUX_UIH, send, len + 2);

	kfree(send);
}

static int mux_send_uih_data(ts0710_con * ts0710, u8 dlci, u8 *data, int len)
{
	int ret;
        u8 *send;
        if (ts0710->dlci[dlci].tag) {
	  send = kmalloc(len + 2, GFP_ATOMIC);
	  *send = ts0710->dlci[dlci].tag;

          if (len)
                  memcpy(send + 1, data, len);

          len++;
        } else {
          send = data;
        }

	ret = mux_send_frame(dlci, ts0710->initiator, MUX_UIH, send, len);

	if (ts0710->dlci[dlci].tag)
		kfree(send);

	return ret;
}

static int ts0710_msc_msg(ts0710_con * ts0710, u8 value, u8 cr, u8 dlci)
{
	u8 buf[2];
	msc_t *send = (msc_t *)buf;

	send->dlci.ea = 1;
	send->dlci.cr = 1;
	send->dlci.d = dlci & 1;
	send->dlci.server_chn = (dlci >> 1) & 0x1f;

	send->v24_sigs = value;

	mux_send_uih(ts0710, cr, MSC, buf, 2);

	return 0;
}

/* Parses a multiplexer control channel packet */

void process_mcc(u8 * data, u32 len, ts0710_con * ts0710, int longpkt)
{
	mcc_short_frame *mcc_short_pkt;
	int j;

	if (longpkt)
		mcc_short_pkt = (mcc_short_frame *) (((long_frame *) data)->data);
	else
		mcc_short_pkt = (mcc_short_frame *) (((short_frame *) data)->data);

	TS0710_DEBUG("MUX Received:%d\n", mcc_short_pkt->h.type.type);
	switch (mcc_short_pkt->h.type.type) {

	case FCON:		/*Flow control on command */
		TS0710_PRINTK("MUX Received Flow control(all channels) on command\n");
		if (mcc_short_pkt->h.type.cr == MCC_CMD) {
			ts0710->dlci[0].state = CONNECTED;
			ts0710_fcon_msg(ts0710, MCC_RSP);
		}
		break;

	case FCOFF:		/*Flow control off command */
		TS0710_PRINTK("MUX Received Flow control(all channels) off command\n");
		if (mcc_short_pkt->h.type.cr == MCC_CMD) {
			for (j = 0; j < TS0710_MAX_CHN; j++) {
				ts0710->dlci[j].state = FLOW_STOPPED;
			}
			ts0710_fcoff_msg(ts0710, MCC_RSP);
		}
		break;

	case MSC:		/*Modem status command */
		{
			u8 dlci;
			u8 v24_sigs;

			dlci = (mcc_short_pkt->value[0]) >> 2;
			v24_sigs = mcc_short_pkt->value[1];

			if ((ts0710->dlci[dlci].state != CONNECTED)
			    && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
				send_dm(ts0710, dlci);
				break;
			}

			if (mcc_short_pkt->h.type.cr == MCC_CMD) {
				TS0710_DEBUG("Received Modem status command\n");
				if ((v24_sigs & 2) && (ts0710->dlci[dlci].state == CONNECTED)) {
					TS0710_LOG ("MUX Received Flow off on dlci %d\n", dlci);
					ts0710->dlci[dlci].state = FLOW_STOPPED;
				} else if (((v24_sigs & 2)==0) && MUX_STOPPED(ts0710,dlci)) {
					ts0710->dlci[dlci].state = CONNECTED;
					TS0710_LOG ("MUX Received Flow on on dlci %d\n", dlci);
				}

				ts0710_msc_msg(ts0710, v24_sigs, MCC_RSP, dlci);
			} else {
				TS0710_DEBUG("Received Modem status response\n");

				if (v24_sigs & 2) {
					TS0710_DEBUG("Flow stop accepted\n");
				}
			}
			break;
		}

	case PN:		/*DLC parameter negotiation */
		{
			u8 dlci;
			u16 frame_size;
			pn_msg *pn_pkt;

			pn_pkt = (pn_msg *) data;
			dlci = pn_pkt->dlci;
			frame_size = GET_PN_MSG_FRAME_SIZE(pn_pkt);
			TS0710_DEBUG("Received DLC parameter negotiation, PN\n");

			if (pn_pkt->mcc_s_head.type.cr == MCC_CMD) {
				TS0710_DEBUG("received PN command with:\n");
				TS0710_DEBUG("Frame size:%d\n", frame_size);

				frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
				send_pn_msg(ts0710, pn_pkt->prior, frame_size, 0, 0, dlci, MCC_RSP);
				ts0710->dlci[dlci].mtu = frame_size;
				TS0710_DEBUG("process_mcc : mtu set to %d\n", ts0710->dlci[dlci].mtu);
			} else {
				TS0710_DEBUG("received PN response with:%d\n",frame_size);
				frame_size = min(frame_size, ts0710->dlci[dlci].mtu);
				ts0710->dlci[dlci].mtu = frame_size;


				if (ts0710->dlci[dlci].state == NEGOTIATING) {
					ts0710->dlci[dlci].state = CONNECTING;
					wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
				}
			}
			break;
		}

	case NSC:		/*Non supported command resonse */
		TS0710_LOG("MUX Received Non supported command response\n");
		break;

	default:		/*Non supported command received */
		{
		int count = len;
		int i =0;
		while(count--)
		{
			printk("%02x",data[i++]);
		}
		printk("\n");
		TS0710_LOG("MUX Received a non supported command\n");
		send_nsc_msg(ts0710, mcc_short_pkt->h.type, MCC_RSP);
		break;}
	}
}

static void free_mux_recv_struct(mux_recv_struct * recv_info)
{
	if (!recv_info)
		return;

	kfree(recv_info);
}

static inline void add_post_recv_queue(mux_recv_struct ** head,
				       mux_recv_struct * new_item)
{
	new_item->next = *head;
	*head = new_item;
}

static void ts0710_flow_on(u8 dlci, ts0710_con * ts0710)
{
	TS0710_DEBUG("TS0710: flow on\n");
	if (!(ts0710->dlci[0].state & (CONNECTED | FLOW_STOPPED)))
		return;

	if (!(ts0710->dlci[dlci].state & (CONNECTED | FLOW_STOPPED)))
		return;

	if (!(ts0710->dlci[dlci].flow_control))
		return;

	ts0710_msc_msg(ts0710, EA | RTC | RTR | DV, MCC_CMD, dlci);

	TS0710_DEBUG("MUX send Flow on dlci %d\n", dlci);
	ts0710->dlci[dlci].flow_control = 0;
}

/*static void ts0710_flow_off(struct tty_struct *tty, u8 dlci,
		ts0710_con * ts0710)
{
	int i;

	TS0710_DEBUG("TS0710: flow off\n");

	if (test_and_set_bit(TTY_THROTTLED, &tty->flags))
		return;


	if ((ts0710->dlci[0].state != CONNECTED) && (ts0710->dlci[0].state != FLOW_STOPPED))
		return;

	if ((ts0710->dlci[dlci].state != CONNECTED)
			&& (ts0710->dlci[dlci].state != FLOW_STOPPED))
		return;

	if (ts0710->dlci[dlci].flow_control)
		return;

	for (i = 0; i < 3; i++) {
		if (ts0710_msc_msg(ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci) < 0)
			continue;

		TS0710_DEBUG("MUX send Flow off on dlci %d\n", dlci);
		ts0710->dlci[dlci].flow_control = 1;
		break;
	}
}*/

void ts0710_recv_data_server(ts0710_con * ts0710, short_frame *short_pkt, int len)
{
	u8 be_connecting;
	u8 *uih_data_start;
	u32 uih_len;
	long_frame *long_pkt;
	TS0710_DEBUG("recv from Mux 0:0x%x\n", CLR_PF(short_pkt->h.control));
	switch (CLR_PF(short_pkt->h.control)) {
	case SABM:
		TS0710_DEBUG("SABM-packet received\n");
		TS0710_DEBUG("server channel == 0\n");
		ts0710->dlci[0].state = CONNECTED;

		TS0710_DEBUG("sending back UA - control channel\n");
		send_ua(ts0710, 0);
		wake_up_interruptible(&ts0710->dlci[0].open_wait);

		break;
	case UA:
		TS0710_DEBUG("UA packet received\n");

		TS0710_DEBUG("server channel == 0\n");

		if (ts0710->dlci[0].state == CONNECTING) {
			ts0710->dlci[0].state = CONNECTED;
			wake_up_interruptible(&ts0710->dlci[0].
						      open_wait);
		} else if (ts0710->dlci[0].state == DISCONNECTING) {
			ts0710_upon_disconnect();
		} else {
			TS0710_DEBUG(" Something wrong receiving UA packet\n");
		}

		break;
	case DM:
		TS0710_DEBUG("DM packet received\n");
		TS0710_DEBUG("server channel == 0\n");

		if (ts0710->dlci[0].state == CONNECTING) {
			be_connecting = 1;
		} else {
			be_connecting = 0;
		}
		ts0710_upon_disconnect();
		if (be_connecting) {
			ts0710->dlci[0].state = REJECTED;
		}

		break;
	case DISC:
		TS0710_DEBUG("DISC packet received\n");

		TS0710_DEBUG("server channel == 0\n");

		send_ua(ts0710, 0);
		TS0710_DEBUG("DISC, sending back UA\n");

		ts0710_upon_disconnect();

		break;

	case UIH:
		TS0710_DEBUG("UIH packet received\n");

		if (GET_PF(short_pkt->h.control)) {
			TS0710_LOG("MUX Error %s: UIH packet with P/F set, discard it!\n",
				__FUNCTION__);
			break;
		}

		/* FIXME: connected and flow */

		if ((short_pkt->h.length.ea) == 0) {
			TS0710_DEBUG("Long UIH packet received\n");
			long_pkt = (long_frame *) short_pkt;
			uih_len = GET_LONG_LENGTH(long_pkt->h.length);
			uih_data_start = long_pkt->h.data;
			TS0710_DEBUG("long packet length %d\n", uih_len);

		} else {
			TS0710_DEBUG("Short UIH pkt received\n");
			uih_len = short_pkt->h.length.len;
			uih_data_start = short_pkt->data;

		}
		TS0710_DEBUG("UIH on serv_channel 0\n");
		process_mcc((u8 *)short_pkt, len, ts0710,
				!(short_pkt->h.length.ea));

		break;

	default:
		TS0710_DEBUG("illegal packet\n");
		break;
	}

}

void process_uih(ts0710_con * ts0710, char *data, int len, u8 dlci) {

	short_frame *short_pkt = (short_frame *) data;
	long_frame *long_pkt;
	u8 *uih_data_start;
	u32 uih_len;
	u8 tag;
	u8 tty_idx;
	struct tty_struct *tty;
	mux_recv_struct *recv_info;
	int ret;
	int pushed;

	if ((ts0710->dlci[dlci].state != CONNECTED)
			&& (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		TS0710_LOG("MUX Error %s: DLCI %d not connected, discard it!\n",
				__FUNCTION__, dlci);
		send_dm(ts0710, dlci);
		return;
	}

	if ((short_pkt->h.length.ea) == 0) {
		TS0710_DEBUG("Long UIH packet received\n");
		long_pkt = (long_frame *) data;
		uih_len = GET_LONG_LENGTH(long_pkt->h.length);
		uih_data_start = long_pkt->h.data;
		TS0710_DEBUG("long packet length %d\n", uih_len);

	} else {
		TS0710_DEBUG("Short UIH pkt received\n");
		uih_len = short_pkt->h.length.len;
		uih_data_start = short_pkt->data;

	}

	TS0710_DEBUG("UIH on channel %d\n", dlci);

	if (uih_len > ts0710->dlci[dlci].mtu) {
		TS0710_PRINTK
			("MUX Error:  DLCI:%d, uih_len:%d is bigger than mtu:%d, discard data!\n",
			 dlci, uih_len, ts0710->dlci[dlci].mtu);
		return;
	}
if(ts0710->dlci[dlci].tag)
{
	tag = *uih_data_start;
	uih_data_start++;
	uih_len--;
}
	if (!uih_len)
		return;

	tty_idx = dlci;
	tty = mux_table[tty_idx];

	if ((!mux_tty[tty_idx]) || (!tty)) {
		TS0710_PRINTK
			("MUX: No application waiting for, discard it! /dev/mux%d\n",
			 tty_idx);
		return;

	}

	if ((!mux_recv_info_flags[tty_idx])
			|| (!mux_recv_info[tty_idx])) {
		TS0710_PRINTK
			("MUX Error: No mux_recv_info, discard it! /dev/mux%d\n",
			 tty_idx);
		return;
	}

	recv_info = mux_recv_info[tty_idx];
	if (recv_info->total > 8192) {
		TS0710_PRINTK
			("MUX : discard data for tty_idx:%d, recv_info->total > 8192 \n",
			 tty_idx);
		return;
	}
		if(dlci==99)
		{
			int i = 0;
			int count =uih_len;
			TS0710_LOG(" ts0710_recv_data on channel %d,len:%d\n", dlci,uih_len);

			while(count--)
			{
				printk("%02x ", uih_data_start[i++]);
			}
			printk("\n");
		}

		if(dlci != 0)
		{
			if(rx_dump_map & (1 << (dlci -1)))
			{
				spin_lock(&dump_lock);
				printk("dlc:%d rx %d bytes data!--\n", dlci, uih_len);
				if(!simple_dump)
				{
					hex_packet(uih_data_start,uih_len);
				}
				spin_unlock(&dump_lock);
			}
		}
#if 0
	flow_control = 0;
	recv_room = 65535;
	if (tty->receive_room)
		recv_room = tty->receive_room;

	if ((recv_room - (uih_len + recv_info->total)) <
			ts0710->dlci[dlci].mtu) {
		flow_control = 1;
	}


		tty->ldisc->ops->receive_buf(tty, uih_data_start, NULL, uih_len);

	if (flow_control)
		ts0710_flow_off(tty, dlci, ts0710);

	if(dlci == 99)
	{
		do_sync_write(file, uih_data_start, uih_len, &file->f_pos);
	}
#endif
	mux_queue_array[dlci]->stat_rx_delt_bytes += uih_len;

	ret = 0;
	pushed = 0;
	while( uih_len > 0)
	{
		ret = tty_insert_flip_string(tty->port, uih_data_start+pushed, uih_len);
		uih_len -= ret;
		pushed += ret;
		if(pushed > (tty->receive_room - 128))
		{
			//TS0710_LOG("tty receive_room(%d) is not enough for %d \n", tty->receive_room -128, ret);
		}else
		{
			tty_flip_buffer_push(tty->port);
		}
		if(uih_len > 0)
		{
			TS0710_LOG("tty_insert_flip_string not completed %d left, continue \n", uih_len);
			tty_flip_buffer_push(tty->port);
			usleep_range(1,2);
		}
	}

}

void ts0710_recv_data(ts0710_con * ts0710, char *data, int len)
{
	short_frame *short_pkt;
	u8 dlci;

	short_pkt = (short_frame *) data;

	dlci = short_pkt->h.addr.server_chn << 1 | short_pkt->h.addr.d;
	//TS0710_DEBUG(" ts0710_recv_data on channel %d\n", dlci);
	backup_mux_log(dlci,0, data, len);
	if (!dlci)
		return ts0710_recv_data_server(ts0710, short_pkt, len);

	switch (CLR_PF(short_pkt->h.control)) {
	case SABM:
		TS0710_PRINTK("Incomming connect on channel %d\n", dlci);

		send_ua(ts0710, dlci);

		ts0710->dlci[dlci].state = CONNECTED;
		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);

		break;
	case UA:
		TS0710_DEBUG("Incomming UA on channel %d\n", dlci);

		if (ts0710->dlci[dlci].state == CONNECTING) {
			ts0710->dlci[dlci].state = CONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].
					open_wait);
		} else if (ts0710->dlci[dlci].state == DISCONNECTING) {
			ts0710->dlci[dlci].state = DISCONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].
					open_wait);
			wake_up_interruptible(&ts0710->dlci[dlci].
					close_wait);
			ts0710_reset_dlci(dlci);
		}

		break;
	case DM:
		TS0710_DEBUG("Incomming DM on channel %d\n", dlci);

		if (ts0710->dlci[dlci].state == CONNECTING)
			ts0710->dlci[dlci].state = REJECTED;
		else
			ts0710->dlci[dlci].state = DISCONNECTED;

		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
		ts0710_reset_dlci(dlci);


		break;
	case DISC:
		send_ua(ts0710, dlci);

		ts0710->dlci[dlci].state = DISCONNECTED;
		wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		wake_up_interruptible(&ts0710->dlci[dlci].close_wait);
		ts0710_reset_dlci(dlci);

		break;
	case UIH:
		process_uih(ts0710, data, len, dlci);

		break;
	default:
		TS0710_PRINTK("illegal packet\n");
		break;
	}
}

/* Close ts0710 channel */
static void ts0710_close_channel(u8 dlci)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int try;
	unsigned long t;

	TS0710_DEBUG("ts0710_disc_command on channel %d\n", dlci);

	if ((ts0710->dlci[dlci].state == DISCONNECTED)
	    || (ts0710->dlci[dlci].state == REJECTED)) {
		return;
	} else if (ts0710->dlci[dlci].state == DISCONNECTING) {
		/* Reentry */
		return;
	}

	ts0710->dlci[dlci].state = DISCONNECTING;
	try = 3;
	while (try--) {
		t = jiffies;
		send_disc(ts0710, dlci);

		interruptible_sleep_on_timeout(&ts0710->dlci[dlci].
					       close_wait,
					       TS0710MUX_TIME_OUT);
		if (ts0710->dlci[dlci].state == DISCONNECTED) {
			break;
		} else if (signal_pending(current)) {
			TS0710_PRINTK ("MUX DLCI %d Send DISC got signal!\n", dlci);
			break;
		} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
			TS0710_PRINTK ("MUX DLCI %d Send DISC timeout!\n", dlci);
			continue;
		}
	}

	if (ts0710->dlci[dlci].state != DISCONNECTED) {
		if (dlci == 0) {	/* Control Channel */
			ts0710_upon_disconnect();
		} else {	/* Other Channel */
			ts0710->dlci[dlci].state = DISCONNECTED;
			wake_up_interruptible(&ts0710->dlci[dlci].
					      close_wait);
			ts0710_reset_dlci(dlci);
		}
	}
}

int ts0710_open_channel(u8 dlci)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int try;
	int retval;
	unsigned long t;

	retval = -ENODEV;
	TS0710_DEBUG("Mux DLCI:%d, state:%d,DLC0 stats is :%d\n",dlci, ts0710->dlci[dlci].state,ts0710->dlci[0].state);
	if (dlci == 0) {	/* control channel */
		if (ts0710->dlci[0].state & (CONNECTED | FLOW_STOPPED)) {
			return 0;
		} else if (ts0710->dlci[0].state == CONNECTING) {
			/* Reentry */
			TS0710_PRINTK("MUX DLCI: 0, reentry to open DLCI 0, pid: %d, %s !\n",
				current->pid, current->comm);
			try = 11;
			while (try--) {
				t = jiffies;
				interruptible_sleep_on_timeout(&ts0710->dlci[0].
							       open_wait,
							       TS0710MUX_TIME_OUT);
				if ((ts0710->dlci[0].state == CONNECTED)
				    || (ts0710->dlci[0].state ==
					FLOW_STOPPED)) {
					retval = 0;
					break;
				} else if (ts0710->dlci[0].state == REJECTED) {
					retval = -EREJECTED;
					break;
				} else if (ts0710->dlci[0].state ==
					   DISCONNECTED) {
					break;
				} else if (signal_pending(current)) {
					TS0710_PRINTK ("MUX DLCI:%d Wait for connecting got signal!\n", dlci);
					retval = -EAGAIN;
					break;
				} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
					TS0710_PRINTK
					    ("MUX DLCI:%d Wait for connecting timeout!\n",
					     dlci);
					continue;
				} else if (ts0710->dlci[0].state == CONNECTING) {
					continue;
				}
			}

			if (ts0710->dlci[0].state == CONNECTING) {
				ts0710->dlci[0].state = DISCONNECTED;
			}
		} else if ((ts0710->dlci[0].state != DISCONNECTED)
			   && (ts0710->dlci[0].state != REJECTED)) {
			TS0710_PRINTK("MUX DLCI:%d state is invalid!\n", dlci);
			return retval;
		} else {
			ts0710->initiator = 1;
			ts0710->dlci[0].state = CONNECTING;
			ts0710->dlci[0].initiator = 1;

			t = jiffies;
			send_sabm(ts0710, 0);
			interruptible_sleep_on_timeout(&ts0710->dlci[0].
					open_wait,
					TS0710MUX_TIME_OUT);
			if ((ts0710->dlci[0].state == CONNECTED)
					|| (ts0710->dlci[0].state ==
						FLOW_STOPPED)) {
				retval = 0;
			} else if (ts0710->dlci[0].state == REJECTED) {
				TS0710_PRINTK
					("MUX DLCI:%d Send SABM got rejected!\n",
					 dlci);
				retval = -EREJECTED;
			} else if (signal_pending(current)) {
				TS0710_PRINTK ("MUX DLCI:%d Send SABM got signal!\n", dlci);
				retval = -EAGAIN;
			} else if ((jiffies - t) >= TS0710MUX_TIME_OUT) {
				TS0710_PRINTK
					("MUX DLCI:%d Send SABM timeout!\n",
					 dlci);
				retval = -ENODEV;
			}

			if (ts0710->dlci[0].state == CONNECTING) {
				ts0710->dlci[0].state = DISCONNECTED;
			}
			wake_up_interruptible(&ts0710->dlci[0].open_wait);
		}
	} else {		/* other channel */
		if ((ts0710->dlci[0].state != CONNECTED)
		    && (ts0710->dlci[0].state != FLOW_STOPPED)) {
			return retval;
		} else if ((ts0710->dlci[dlci].state == CONNECTED)
			   || (ts0710->dlci[dlci].state == FLOW_STOPPED)) {
			return 0;
		} else if ((ts0710->dlci[dlci].state == NEGOTIATING)
			   || (ts0710->dlci[dlci].state == CONNECTING)) {
			/* Reentry */

			t = jiffies;
			interruptible_sleep_on_timeout(&ts0710->
						       dlci[dlci].
						       open_wait,
						       TS0710MUX_TIME_OUT);
			if ((ts0710->dlci[dlci].state == CONNECTED)
			    || (ts0710->dlci[dlci].state ==
				FLOW_STOPPED)) {
				retval = 0;
			} else if (ts0710->dlci[dlci].state == REJECTED) {
				retval = -EREJECTED;
			} else if (signal_pending(current)) {
				retval = -EAGAIN;
			}

			if ((ts0710->dlci[dlci].state == NEGOTIATING)
			    || (ts0710->dlci[dlci].state == CONNECTING)) {
				ts0710->dlci[dlci].state = DISCONNECTED;
			}
		} else if ((ts0710->dlci[dlci].state != DISCONNECTED)
			   && (ts0710->dlci[dlci].state != REJECTED)) {
			TS0710_PRINTK("MUX DLCI:%d state is invalid!\n", dlci);
			return retval;
		} else {
			ts0710->dlci[dlci].state = NEGOTIATING;
			ts0710->dlci[dlci].initiator = 1;

			t = jiffies;
			send_pn_msg(ts0710, 7, ts0710->dlci[dlci].mtu,
				    0, 0, dlci, 1);
			interruptible_sleep_on_timeout(&ts0710->
						       dlci[dlci].
						       open_wait,
						       TS0710MUX_TIME_OUT);
			if (signal_pending(current)) {
				TS0710_PRINTK
				    ("MUX DLCI:%d Send pn_msg got signal!\n",
				     dlci);
				retval = -EAGAIN;
			}

			if (ts0710->dlci[dlci].state == CONNECTING) {

				t = jiffies;
				send_sabm(ts0710, dlci);
				interruptible_sleep_on_timeout(&ts0710->
							       dlci
							       [dlci].
							       open_wait,
							       TS0710MUX_TIME_OUT);
				if ((ts0710->dlci[dlci].state ==
				     CONNECTED)
				    || (ts0710->dlci[dlci].state ==
					FLOW_STOPPED)) {
					retval = 0;
				} else if (ts0710->dlci[dlci].state ==
					   REJECTED) {
					TS0710_PRINTK
					    ("MUX DLCI:%d Send SABM got rejected!\n",
					     dlci);
					retval = -EREJECTED;
				} else if (signal_pending(current)) {
					TS0710_PRINTK
					    ("MUX DLCI:%d Send SABM got signal!\n",
					     dlci);
					retval = -EAGAIN;
				}
			}

			if ((ts0710->dlci[dlci].state == NEGOTIATING)
			    || (ts0710->dlci[dlci].state == CONNECTING)) {
				ts0710->dlci[dlci].state = DISCONNECTED;
			}
			wake_up_interruptible(&ts0710->dlci[dlci].open_wait);
		}
	}
	return retval;
}

/****************************
 * TTY driver routines
*****************************/

static void mux_close(struct tty_struct *tty, struct file *filp)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	u8 dlci;

	UNUSED_PARAM(filp);
	TS0710_DEBUG("MUX:Enter close mux driver by process id:%d (\"%s\")\n",current->tgid,current->comm);
	TS0710_DEBUG("Enter Mux Close Function!\n");
	line = tty->index;
	if (MUX_INVALID(line))
		return;
	//if (!(ipc_tty && line))
		//return;
	if (mux_tty[line] > 0)
		mux_tty[line]--;

	dlci = line;

        /* if closed last time and real tty still opened
         * send disconnect packet */
	if (mux_tty[line] == 0 /*&& ipc_tty*/)
	{
		ts0710_close_channel(dlci);
		mux_table[line] = NULL;
	}


	if (mux_tty[line] != 0)
		return;

	if ((mux_send_info_flags[line]) && (mux_send_info[line]))
	{
		mux_send_info_flags[line] = 0;
		kfree(mux_send_info[line]);
		mux_send_info[line] = 0;
		TS0710_DEBUG("Free mux_send_info for /dev/mux%d\n", line);
	}

	if ((mux_recv_info_flags[line])
	    && (mux_recv_info[line])
	    && (mux_recv_info[line]->total == 0)) {
		mux_recv_info_flags[line] = 0;
		free_mux_recv_struct(mux_recv_info[line]);
		mux_recv_info[line] = 0;
		TS0710_DEBUG("Free mux_recv_info for /dev/mux%d\n", line);
	}

        /* real tty already closed, so we cant write data */
        if (!ipc_tty) {
          TS0710_DEBUG("dlci %d closed after mux. dont do so\n", line);
          return;
        }
	if(dlci)
		mux_queue_deinit(dlci);
	ts0710_flow_on(dlci, ts0710);

	wake_up_interruptible(&tty->read_wait);
	wake_up_interruptible(&tty->write_wait);
	tty->packet = 0;
	TS0710_PRINTK("MUX: Disconnect server channel %d successful!\n",dlci);
}

static void mux_throttle(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	int i;
	u8 dlci;

	line = tty->index;
	if (MUX_INVALID(line))
		return;


	TS0710_DEBUG("Enter into %s, minor number is: %d\n", __FUNCTION__,
		     line);

	dlci = line;
	if ((ts0710->dlci[0].state != CONNECTED)
	    && (ts0710->dlci[0].state != FLOW_STOPPED)) {
		return;
	} else if ((ts0710->dlci[dlci].state != CONNECTED)
		   && (ts0710->dlci[dlci].state != FLOW_STOPPED)) {
		return;
	}

	if (ts0710->dlci[dlci].flow_control) {
		return;
	}

	for (i = 0; i < 3; i++) {
		if (ts0710_msc_msg
		    (ts0710, EA | FC | RTC | RTR | DV, MCC_CMD, dlci) < 0) {
			continue;
		} else {
			TS0710_LOG("MUX Send Flow OFF on dlci %d\n", dlci);
			ts0710->dlci[dlci].flow_control = 1;
			break;
		}
	}
}

static void mux_unthrottle(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int line;
	u8 dlci;
	mux_recv_struct *recv_info;

	line = tty->index;
	if (MUX_INVALID(line))
		return;

	if ((!mux_recv_info_flags[line]) || (!mux_recv_info[line])) {
		return;
	}

	TS0710_DEBUG("Enter into %s, minor number is: %d\n", __FUNCTION__,
		     line);

	recv_info = mux_recv_info[line];
	dlci = line;

	if (recv_info->total) {
		recv_info->post_unthrottle = 1;
		/* schedule_work(&post_recv_tqueue); */
	} else {
		TS0710_LOG("MUX Send Flow ON on dlci %d\n", dlci);
		ts0710_flow_on(dlci, ts0710);
	}
}

static int mux_chars_in_buffer(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	int line;
	u8 dlci;
	mux_send_struct *send_info;

	retval = TS0710MUX_MAX_CHARS_IN_BUF;

	line = tty->index;
	if (MUX_INVALID(line)) {
		goto out;
	}

	dlci = line;

	if (! MUX_USABLE(ts0710, dlci))
		goto out;

	if (!(mux_send_info_flags[line])) {
		goto out;
	}
	send_info = mux_send_info[line];
	if (!send_info) {
		goto out;
	}
	if (send_info->filled) {
		goto out;
	}

	retval = 0;

out:
	return retval;
}

static int mux_write(struct tty_struct *tty,
		     const unsigned char *buf, int count)
{
	ts0710_con *ts0710 = &ts0710_connection;
	u8 dlci;
	int written = 0;
	int add_len = 0;
	int length;
	if (!count)
		return 0;

	dlci = tty->index;
	/*if(dlci==10)
	{
	TS0710_LOG("************write MUX2, count is %d\n",size);
	int i = 0;
	while(size--)
	{
		printk("%02x ", buf[i++]);
	}
	printk("\n");

	}*/
	/*
	 * FIXME: split big packets into small one
	 * FIXME: support DATATAG
	 * */
	 if(ts0710->dlci[dlci].tag)
	 	add_len += 1;
	if (count & ~0x7f) {
		add_len += 7;
		} else
		add_len += 6;

	length = count;
	while(length > ts0710->dlci[dlci].mtu)
	{
		written += (mux_send_uih_data(ts0710, dlci, (u8 *)buf, ts0710->dlci[dlci].mtu)- add_len);
		buf += ts0710->dlci[dlci].mtu;
		length -= ts0710->dlci[dlci].mtu;
	}
	written += (mux_send_uih_data(ts0710, dlci, (u8 *)buf, length) - add_len);
	return written;
}

static int mux_write_room(struct tty_struct *tty)
{
	ts0710_con *ts0710 = &ts0710_connection;
	int retval;
	int line;
	u8 dlci;
	mux_send_struct *send_info;

	retval = 0;

	line = tty->index;
	if (MUX_INVALID(line))
		goto out;

	dlci = line;
	if (ts0710->dlci[0].state == FLOW_STOPPED) {
		TS0710_DEBUG("Flow stopped on all channels, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state == FLOW_STOPPED) {
		TS0710_DEBUG("Flow stopped, returning ZERO\n");
		goto out;
	} else if (ts0710->dlci[dlci].state != CONNECTED) {
		TS0710_DEBUG("DLCI %d not connected\n", dlci);
		goto out;
	}

	if (!(mux_send_info_flags[line]))
		goto out;

	send_info = mux_send_info[line];
	if (!send_info)
		goto out;

	if (send_info->filled)
		goto out;
	if(ts0710->dlci[dlci].tag)
		retval = ts0710->dlci[dlci].mtu - 1;
	else
		retval = ts0710->dlci[dlci].mtu;

out:
	return retval;
}

static void mux_flush_buffer(struct tty_struct *tty)
{
	int line;

	line = tty->index;
	if (MUX_INVALID(line))
		return;


	TS0710_DEBUG("MUX %s: line is:%d\n", __FUNCTION__, line);

	if ((mux_send_info_flags[line])
	    && (mux_send_info[line])
	    && (mux_send_info[line]->filled)) {

		mux_send_info[line]->filled = 0;
	}

	wake_up_interruptible(&tty->write_wait);

	if ((tty->flags & (1 << TTY_DO_WRITE_WAKEUP)) &&
	    tty->ldisc->ops->write_wakeup) {
		tty->ldisc->ops->write_wakeup(tty);
	}
}

static int mux_open(struct tty_struct *tty, struct file *filp)
{
	int retval;
	int line;
	u8 dlci;
	mux_send_struct *send_info;
	mux_recv_struct *recv_info;

	UNUSED_PARAM(filp);
	TS0710_DEBUG("MUX:Enter open mux driver by process id:%d (\"%s\")\n",current->tgid,current->comm);
	retval = -ENODEV;

	line = tty->index;
	TS0710_DEBUG("MUX:get  mux driver index %d, ipc_tty %x\n", line, ipc_tty);
	if (!(ipc_tty && line))
		return -ENODEV;
	mux_tty[line]++;
	dlci = line;
	mux_table[line] = tty;
	mux_queue_init(0);
	if(dlci)
		mux_queue_init(dlci);

	TS0710_DEBUG("MUX:Enter open mux driver\n");
	/* Open server channel 0 first */
	if ((retval = ts0710_open_channel(0)) != 0) {
		TS0710_PRINTK("MUX: Can't connect server channel 0!\n");
		fcs_init();
		ts0710_reset_con();
		mux_tty[line]--;
		goto out;
	}
	TS0710_DEBUG("MUX: Connect server channel 0 successful!\n");
	/* Allocate memory first. As soon as connection has been established, MUX may receive */
	if (mux_send_info_flags[line] == 0) {
		send_info =
		    (mux_send_struct *) kmalloc(sizeof(mux_send_struct),
						GFP_KERNEL);
		if (!send_info) {
			retval = -ENOMEM;

			mux_tty[line]--;
			goto out;
		}
		send_info->length = 0;
		send_info->flags = 0;
		send_info->filled = 0;
		mux_send_info[line] = send_info;
		mux_send_info_flags[line] = 1;
		TS0710_DEBUG("Allocate mux_send_info for /dev/mux%d", line);
	}

	if (mux_recv_info_flags[line] == 0) {
		recv_info =
		    (mux_recv_struct *) kmalloc(sizeof(mux_recv_struct),
						GFP_KERNEL);
		if (!recv_info) {
			mux_send_info_flags[line] = 0;
			kfree(mux_send_info[line]);
			mux_send_info[line] = 0;
			TS0710_DEBUG("Free mux_send_info for /dev/mux%d", line);
			retval = -ENOMEM;

			mux_tty[line]--;
			goto out;
		}
		recv_info->length = 0;
		recv_info->total = 0;
		recv_info->next = 0;
		recv_info->no_tty = line;
		recv_info->post_unthrottle = 0;
		mux_recv_info[line] = recv_info;
		mux_recv_info_flags[line] = 1;
		TS0710_DEBUG("Allocate mux_recv_info for /dev/mux%d", line);
	}

	/* Now establish DLCI connection */
	if (mux_tty[dlci] > 0) {
		if ((retval = ts0710_open_channel(dlci)) != 0) {
			TS0710_PRINTK("MUX: Can't connected channel %d!\n",
				      dlci);
			ts0710_reset_dlci(dlci);

			mux_send_info_flags[line] = 0;
			kfree(mux_send_info[line]);
			mux_send_info[line] = 0;
			TS0710_DEBUG("Free mux_send_info for /dev/mux%d", line);

			mux_recv_info_flags[line] = 0;
			free_mux_recv_struct(mux_recv_info[line]);
			mux_recv_info[line] = 0;
			TS0710_DEBUG("Free mux_recv_info for /dev/mux%d", line);

			mux_tty[line]--;
			goto out;
		}
	}
	TS0710_PRINTK("MUX: Connect server channel %d successful!\n",dlci);
	retval = 0;

	tty->port->low_latency = 1;

	if(dlci == 99)
	{
		file = filp_open("/data/diagDataK.bin", O_RDWR | O_CREAT, 0);
	}
out:
	return retval;
}

/* mux dispatcher, call from serial.c receiver_chars() */
void mux_dispatcher(struct tty_struct *tty)
{
	UNUSED_PARAM(tty);

	/* schedule_work(&receive_tqueue); */
}

/*static void send_ack(ts0710_con * ts0710, u8 seq_num)
{
	mux_send_frame(CTRL_CHAN, ts0710->initiator, ACK, &seq_num, 1);
}
*/
static void ts_ldisc_rx_post(struct tty_struct *tty, const u8 *data, char *flags, int count)
{
	int framelen;
	short_frame *short_pkt;
	long_frame *long_pkt;
	TS0710_DEBUG("TTY %s has been ts_ldisc_rx_post!!!!,size:%d\n", tty->name,count);

	/*int i = 0;
	while(count--)
	{
		printk("%02x ", data[i++]);
	}
	printk("\n");*/

	short_pkt = (short_frame *) (data + ADDRESS_FIELD_OFFSET);
	if (short_pkt->h.length.ea == 1) {
		framelen = TS0710_MAX_HDR_SIZE + short_pkt->h.length.len + 1 + SEQ_FIELD_SIZE;
	} else {
		long_pkt = (long_frame *) (data + ADDRESS_FIELD_OFFSET);
		framelen = TS0710_MAX_HDR_SIZE + GET_LONG_LENGTH(long_pkt->h.length) + 2 + SEQ_FIELD_SIZE;
	}

//	if (expect_seq == *(data + SLIDE_BP_SEQ_OFFSET))
	{
		//expect_seq++;
		//if (expect_seq >= 4)
			//expect_seq = 0;

	//	send_ack(&ts0710_connection, expect_seq);

		ts0710_recv_data(&ts0710_connection,
				(char*)(data + ADDRESS_FIELD_OFFSET),
				framelen - 2 - SEQ_FIELD_SIZE);
	}
	TS0710_DEBUG("TTY %s has been ts_ldisc_rx_post OUT!!!!,size:%d\n", tty->name,count);

}

void ts_ldisc_rx(struct tty_struct *tty, const u8 *data, char *flags, int size)
{
	u8 *packet_start = (u8 *)data;
	struct mux_data *st = (struct mux_data *)tty->disc_data;

	TS0710_DEBUG("TTY %s has been ts_ldisc_rx!!!!,size:%d\n", tty->name,size);

	while (size--) {
		if (*data == 0xF9) {
			if (st->state == OUT_OF_PACKET) {
				st->state = INSIDE_PACKET;
				packet_start = (u8 *)data;
			} else {
				/* buffer points at ending 0xF9 */
				int packet_size = (data - packet_start) + 1;
				int framelen;
                		short_frame *short_pkt;
                		long_frame *long_pkt;

				if (packet_size + st->chunk_size == 2) {
					packet_start++;
					data++;
					continue;
				}

#if 1 /* Modified by ykli */
                memcpy(st->chunk + st->chunk_size, packet_start, packet_size);
                st->chunk_size += packet_size;
                packet_start += packet_size;



                short_pkt = (short_frame *) (st->chunk + ADDRESS_FIELD_OFFSET);
                if (short_pkt->h.length.ea == 1) {
                    framelen = TS0710_MAX_HDR_SIZE + short_pkt->h.length.len + 1 + SEQ_FIELD_SIZE;
                } else {
                    long_pkt = (long_frame *) (st->chunk + ADDRESS_FIELD_OFFSET);
                    framelen = TS0710_MAX_HDR_SIZE + GET_LONG_LENGTH(long_pkt->h.length) + 2 + SEQ_FIELD_SIZE;
                }

                if (framelen > st->chunk_size)
                {
                    data++;
                    continue;
                }

                if (framelen == st->chunk_size)
                {
                    ts_ldisc_rx_post(tty, st->chunk, flags,
                                     st->chunk_size);
                }
                else
                {
                    TS0710_PRINTK("!!!Error: Not Sync. Dropping this Frame. len=%d, chunk_size=%d", framelen, st->chunk_size);
                }

                st->chunk_size = 0;
                st->state = OUT_OF_PACKET;
#else
				st->state = OUT_OF_PACKET;

				if (!st->chunk_size)
					ts_ldisc_rx_post(
						tty,
						packet_start,
						flags,
						packet_size
					);
				else { /* use existing chunk */
					memcpy(st->chunk + st->chunk_size, packet_start, packet_size);
					ts_ldisc_rx_post(tty, st->chunk, flags,
						st->chunk_size + packet_size);

					st->chunk_size = 0;
				}
#endif
			}
		}
		data++;
	}
	if (st->state == INSIDE_PACKET) /* create/update chunk */
	{
		size_t new_chunk_size = data - packet_start; /* buffer points right after data end */
		memcpy(st->chunk + st->chunk_size, packet_start, new_chunk_size);
		st->chunk_size += new_chunk_size;
	}
}

static int ts_ldisc_open(struct tty_struct *tty)
{
	struct mux_data *disc_data;

	tty->receive_room = 65536;

	disc_data = kzalloc(sizeof(struct mux_data), GFP_KERNEL);

	disc_data->state = OUT_OF_PACKET;
	disc_data->chunk_size = 0;

	tty->disc_data = disc_data;

	printk("TTY %s has been saved to ipc_tty\n", tty->name);
	ipc_tty = tty;

	return 0;
}

static void ts_ldisc_close(struct tty_struct *tty)
{
	int i = 0;
	ipc_tty = 0;
	for(i = 0; i < MUX_NUM; i++)
	{

		backup_mux_dump(i,0);
		backup_mux_dump(i,1);
		if(mux_table[i])
		{
			tty_hangup(mux_table[i]);
		}
	}
	//ts0710_close_channel(0);
	ts0710_upon_disconnect();
	printk("TTY %s has been closed as ipc_tty\n", tty->name);
}

static void ts_ldisc_wake(struct tty_struct *tty)
{
	printk("ts wake\n");
}

static ssize_t ts_ldisc_read(struct tty_struct *tty, struct file *file,
					unsigned char __user *buf, size_t nr)
{
	return 0;
}

static ssize_t ts_ldisc_write(struct tty_struct *tty, struct file *file,
					const unsigned char *data, size_t count)
{
	return 0;
}

static int ts_ldisc_ioctl(struct tty_struct *tty, struct file * file,
					unsigned int cmd, unsigned long arg)
{

        int ret;
        u8 dlci = tty->index;
        void __user *argp = (void __user *)arg;
        unsigned long val;
        ts0710_con *ts0710 = &ts0710_connection;

        switch (cmd) {
          case TS0710STAG:
            ret = copy_from_user(&ts0710->dlci[dlci].tag, argp, sizeof(val));
          break;

          case TS0710GTAG:
            ret = copy_to_user(argp, &ts0710->dlci[dlci].tag, sizeof(val));
          break;
          default:
            ret = -ENOIOCTLCMD;
        }

	return ret;
}

static unsigned int ts_ldisc_poll(struct tty_struct *tty,
					struct file *filp, poll_table *wait)
{
	return 0;
}


struct tty_operations mux_ops = {
	.open = mux_open,
	.close = mux_close,
	.write = mux_write,
	.write_room = mux_write_room,
	.flush_buffer = mux_flush_buffer,
	.chars_in_buffer = mux_chars_in_buffer,
	.throttle = mux_throttle,
	.unthrottle = mux_unthrottle,
};

static struct tty_ldisc_ops ts_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "ts07.10",
	.open		= ts_ldisc_open,
	.close		= ts_ldisc_close,
	.read		= ts_ldisc_read,
	.write		= ts_ldisc_write,
	.ioctl		= ts_ldisc_ioctl,
	.poll		= ts_ldisc_poll,
	.receive_buf	= ts_ldisc_rx,
	.write_wakeup	= ts_ldisc_wake,
};


static u8 iscmdtty_gen1[16] =
	{ 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

static u8 iscmdtty_gen2[ 23 ] =
	{ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


static void init_mux_device(void)
{
	int j;
	for (j = 0; j < NR_MUXS; j++) {
		tty_port_init(&mux_device_table[j].port);
	}
}

static int __init mux_init(void)
{
	u8 j;
	struct proc_dir_entry *proc_entry;

//todo: remove the machine type check!!
	if (machine_is_ezx_a1200() || machine_is_ezx_e6()) {
		NR_MUXS = 23;
		iscmdtty = iscmdtty_gen2;
		mux_send_info_idx = 23;
	}else{
		NR_MUXS = MUX_NUM;
		iscmdtty = iscmdtty_gen1;
		mux_send_info_idx = MUX_NUM;
	}

	TS0710_PRINTK("initializing mux with %d channels\n", NR_MUXS);

	ts0710_init();

	tx_queue_init();

	backup_queue_init();

	proc_entry = proc_create(PROC_FILE_NAME, 0660, NULL, &mux_proc_fops);

	for (j = 0; j < NR_MUXS; j++) {
		mux_send_info_flags[j] = 0;
		mux_send_info[j] = 0;
		mux_recv_info_flags[j] = 0;
		mux_recv_info[j] = 0;
	}
	mux_send_info_idx = NR_MUXS;
	mux_recv_queue = NULL;
	mux_recv_flags = 0;

	mux_driver = alloc_tty_driver(NR_MUXS);
	if (!mux_driver)
		return -ENOMEM;

	mux_driver->owner = THIS_MODULE;
	mux_driver->driver_name = "ts0710mux";
	mux_driver->name = "mux";
	mux_driver->major = TS0710MUX_MAJOR;
	mux_driver->minor_start = TS0710MUX_MINOR_START;
	mux_driver->type = TTY_DRIVER_TYPE_SERIAL;
	mux_driver->subtype = SERIAL_TYPE_NORMAL;
	mux_driver->flags = TTY_DRIVER_RESET_TERMIOS | TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;


	mux_driver->init_termios = tty_std_termios;
	mux_driver->init_termios.c_iflag = 0;
	mux_driver->init_termios.c_oflag = 0;
	mux_driver->init_termios.c_cflag = B115200 | CS8 | CREAD;
	mux_driver->init_termios.c_lflag = 0;

	/* mux_driver.ttys = mux_table; */
	/* mux_driver.driver_state = mux_state; */
	mux_driver->other = NULL;

	tty_set_operations(mux_driver, &mux_ops);

	if (tty_register_driver(mux_driver))
	{
		printk("Couldn't register mux driver");
		return -ENOMEM;
	}

	init_mux_device();

	for (j = 0; j < NR_MUXS; j++) {

		struct device *tty_dev;

		tty_dev = tty_port_register_device(
				&mux_device_table[j].port,
				mux_driver,
				j,
				NULL);
		if (IS_ERR(tty_dev)) {
			pr_err("%s: failed to register tty for port %d, err %ld",
					__func__, j, PTR_ERR(tty_dev));
			return PTR_ERR(tty_dev);
		}

	}

	if (tty_register_ldisc(N_TS0710, &ts_ldisc))
	{
		printk("oops. cant register ldisc\n");
		return -ENOMEM;
	}

	return 0;
}

static void __exit mux_exit(void)
{
	u8 j;


	mux_send_info_idx = NR_MUXS;
	mux_recv_queue = NULL;
	for (j = 0; j < NR_MUXS; j++) {
		if ((mux_send_info_flags[j]) && (mux_send_info[j])) {
			kfree(mux_send_info[j]);
		}
		mux_send_info_flags[j] = 0;
		mux_send_info[j] = 0;

		if ((mux_recv_info_flags[j]) && (mux_recv_info[j])) {
			free_mux_recv_struct(mux_recv_info[j]);
		}
		mux_recv_info_flags[j] = 0;
		mux_recv_info[j] = 0;
	}

	for (j = 0; j < NR_MUXS; j++)
		tty_unregister_device(mux_driver, j);

	if (tty_unregister_driver(mux_driver))
		printk("Couldn't unregister mux driver");
}

module_init(mux_init);
module_exit(mux_exit);
module_param(simple_dump, int, 0644);
module_param(tx_dump_map, int, 0644);
module_param(rx_dump_map, int, 0644);
module_param(mux_backup, int, 0644);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Harald Welte <laforge@openezx.org>");
MODULE_DESCRIPTION("TS 07.10 Multiplexer");
