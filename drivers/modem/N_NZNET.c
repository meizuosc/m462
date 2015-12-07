/*
    N_NZNET.c Created on: Oct 29, 2012, Zhongmin Wu <zmwu@marvell.com>

    Marvell  CCI net driver for Linux
    Copyright (C) 2010 Marvell International Ltd.

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

#include <linux/pm_qos.h>

#define MODEM_QOS_CNT		2*1024*1024  // 16Mbps
#define MODEM_QOS_CNT_P		4*1024*1024  // 32Mbps
/* CPU 1.0GHz, MIF 825MHz, INT 267MHz */
#define NET_QOS_MIN_CPU_FREQ	1000000
#define NET_QOS_MIN_MIF_FREQ	 825000
#define NET_QOS_MIN_INT_FREQ	 267000
#define NET_QOS_LOCK_TIMEOUT	2*1000*1000  // 2sec

static int transferred_cnt=0;
static DEFINE_SPINLOCK(cnt_lock);

static struct pm_qos_request net_pm_qos_cpu;
static struct pm_qos_request net_pm_qos_mif;
static struct pm_qos_request net_pm_qos_int;

static void modem_net_tp_mon(struct work_struct *work);
static DECLARE_DEFERRABLE_WORK(tp_mon, modem_net_tp_mon);

#include "ccinet.h"

#define N_NZNET 9

#include <linux/perf_mode.h>

static void modem_net_tp_mon(struct work_struct *work)
{
	int is_pm_qos = 0;
	unsigned long flags;

	spin_lock_irqsave(&cnt_lock, flags);
	if (transferred_cnt > MODEM_QOS_CNT)
		is_pm_qos = transferred_cnt;
	transferred_cnt = 0;
	spin_unlock_irqrestore(&cnt_lock, flags);

	if (is_pm_qos > MODEM_QOS_CNT_P)
		return;

	if (is_pm_qos) {
		pr_info("%s: request %d byte/s cpu=%d mif=%d int=%d\n",
						__func__, is_pm_qos,
						NET_QOS_MIN_CPU_FREQ,
						NET_QOS_MIN_MIF_FREQ,
						NET_QOS_MIN_INT_FREQ);
		pm_qos_update_request_timeout(&net_pm_qos_cpu,
				NET_QOS_MIN_CPU_FREQ, NET_QOS_LOCK_TIMEOUT);
		pm_qos_update_request_timeout(&net_pm_qos_mif,
				NET_QOS_MIN_MIF_FREQ, NET_QOS_LOCK_TIMEOUT);
		pm_qos_update_request_timeout(&net_pm_qos_int,
				NET_QOS_MIN_INT_FREQ, NET_QOS_LOCK_TIMEOUT);
	}
}

#define HEX_PACKET
static void hex_packet(const unsigned char *p, int len)
{
#ifdef HEX_PACKET
	int i;
	for (i = 0; i < len; i++) {
		if (i && (i % 16) == 0)
			printk("\n");
		printk("%02X ", *p++);
	}
	printk("\n");
#endif
}
static struct tty_struct *lltty;

static int ci_ldisc_open(struct tty_struct *tty)
{
	lltty = tty;
	tty->receive_room = 65536;
	printk("**********************lltty index = %d \n", tty->index);

	pm_qos_add_request(&net_pm_qos_cpu, PM_QOS_KFC_FREQ_MIN, 0);
	pm_qos_add_request(&net_pm_qos_mif, PM_QOS_BUS_THROUGHPUT, 0);
	pm_qos_add_request(&net_pm_qos_int, PM_QOS_DEVICE_THROUGHPUT, 0);

	return 0;
}

static void ci_ldisc_close(struct tty_struct *tty)
{

	cancel_delayed_work_sync(&tp_mon);
	pm_qos_remove_request(&net_pm_qos_cpu);
	pm_qos_remove_request(&net_pm_qos_mif);
	pm_qos_remove_request(&net_pm_qos_int);

	lltty = NULL;
}

static void ci_ldisc_wake(struct tty_struct *tty)
{
	printk("ts wake\n");
}

static ssize_t ci_ldisc_read(struct tty_struct *tty, struct file *file,
					unsigned char __user *buf, size_t nr)
{
	return 0;
}

static ssize_t ci_ldisc_write(struct tty_struct *tty, struct file *file,
					const unsigned char *data, size_t count)
{
	return 0;
}

static void ci_ldisc_rx(struct tty_struct *tty, const unsigned char *data, char *flags, int size)
{
	int len = size;
	const unsigned char *pkhead = data;
	int padding_size;
	const unsigned char * ippacket;
	const struct ccinethdr	*hdr;
	int iplen;
	int n = 0;
	unsigned long l_flags;
	int is_need_schedule = 0;

	spin_lock_irqsave(&cnt_lock, l_flags);
	if (transferred_cnt == 0)
		is_need_schedule = 1;
	transferred_cnt += len;
	spin_unlock_irqrestore(&cnt_lock, l_flags);
	if (is_need_schedule)
		schedule_delayed_work(&tp_mon, HZ);

	while( len > 0)
	{
		hdr = (struct ccinethdr	*)pkhead;
		iplen = be16_to_cpu(hdr->iplen);
		ippacket = pkhead+sizeof(*hdr)+hdr->offset_len;
		//printk(KERN_DEBUG"push up data to cid %d, len %d\n", hdr->cid, iplen);
		if( hdr->cid >= MAX_CID_NUM )
		{
			printk("cid is error :%d, the whole packet len is %d\n", hdr->cid, size);
			hex_packet(data, size > 100? 100:size);
			printk("current packet len is :%d\n", iplen);
			hex_packet((const unsigned char *)hdr, (iplen + 8) > 100 ? 100:(iplen + 8));
		}
		data_rx(ippacket, iplen, hdr->cid);
		padding_size = rx_padding_size(sizeof(*hdr) + iplen + hdr->offset_len);
		pkhead += sizeof(*hdr) + hdr->offset_len + iplen + padding_size;
		len -= sizeof(*hdr) + hdr->offset_len + iplen + padding_size;
		n ++;
		if(len < 0)
		 printk("some packet is lost!\n");
	}
	//printk(KERN_DEBUG"push up %d  packets on cpu %d\n", n, get_cpu());
//	put_cpu();
}

static int ci_ldisc_ioctl(struct tty_struct * tty, struct file * file,unsigned int cmd, unsigned long arg)
{
	return 0;
}

static struct tty_ldisc_ops nz_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "net over HSI",
	.open		= ci_ldisc_open,
	.close		= ci_ldisc_close,
	.read		= ci_ldisc_read,
	.write		= ci_ldisc_write,
	.receive_buf	= ci_ldisc_rx,
	.write_wakeup	= ci_ldisc_wake,
	.ioctl 		=  ci_ldisc_ioctl,
};

void registerNZNETldisc(void)
{
	if (tty_register_ldisc(N_NZNET, &nz_ldisc))
		printk("oops. cant register ldisc\n");
}

static int getDelayus(int length, int throughput)
{
	return length /((throughput*1024*1024) / (8*1000000));
}

static ssize_t __maybe_unused sendDataVoid(char * data, int len)
{
	int ret = 0;
	int min = getDelayus(len,230);
	int max = getDelayus(len,180);
	//printk(KERN_DEBUG"push down data len %d, sleep %d-%d us\n", len,min,max);
	usleep_range(min, max);
	return ret;
}

static ssize_t __maybe_unused senddataBack(char * data, int len)
{
	ci_ldisc_rx(NULL, data, NULL, len);
	return 0;
}

static int __maybe_unused data_pre_check(const unsigned char *data, int size)
{
	int len = size;
	const unsigned char *pkhead = data;
	int padding_size;
	int extra_padding = 48;
	const unsigned char * ippacket;
	const struct ccinethdr	*hdr;
	int iplen;
	int n = 0;

	if(len > TX_BUFFER_LEN)
		printk("Tx packet too long!!! :%d\n", len);

	while( len > 0)
	{
		hdr = (struct ccinethdr	*)pkhead;
		iplen = be16_to_cpu(hdr->iplen);
		ippacket = pkhead+sizeof(*hdr)+hdr->offset_len;
		//printk(KERN_DEBUG"push up data to cid %d, len %d\n", hdr->cid, iplen);
		if( hdr->cid >= MAX_CID_NUM || iplen > 1500)
		{
			printk("cid is error :%d, the whole packet len is %d\n", hdr->cid, size);
			hex_packet(data, size > 100? 100:size);
			printk("current packet len is :%d\n", iplen);
			hex_packet((const unsigned char *)hdr, (iplen + 8) > 100 ? 100:(iplen + 8));
		}
		padding_size = rx_padding_size(sizeof(*hdr) + iplen + hdr->offset_len);
		padding_size += extra_padding;
		pkhead += sizeof(*hdr) + hdr->offset_len + iplen + padding_size;
		len -= sizeof(*hdr) + hdr->offset_len + iplen + padding_size;
		n ++;
		if(len < 0)
		 printk("some tx packet is error!\n");
	}

	return 0;
}
ssize_t sendDataLowLevel(char * data, int len)
{
#if 1
	int ret = 0;
	unsigned long l_flags;
	int is_need_schedule = 0;

	spin_lock_irqsave(&cnt_lock, l_flags);
	if (transferred_cnt == 0)
		is_need_schedule = 1;
	transferred_cnt += len;
	spin_unlock_irqrestore(&cnt_lock, l_flags);
	if (is_need_schedule)
		schedule_delayed_work(&tp_mon, HZ);

	if(likely(len > 0))
	{
		//data_pre_check(data,len);
		if(lltty && lltty->ops->write)
			ret = lltty->ops->write(lltty, data, len);
		else
			ret = -ENODEV;
	}
	return ret;
#else
	return senddataBack(data, len);
#endif

}
