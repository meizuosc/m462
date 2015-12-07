/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <net/sock.h>
#include <net/checksum.h>
#include <linux/if_ether.h>
#include <linux/if_arp.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/percpu.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/cpu.h>

#include "ccinet.h"



#define	INTFACE_NAME "ccinetnz"


#define HIGHT_WATER_MARK 128
#define LOW_WATER_MARK 64


#define CCINET_IOCTL_SET_V6_CID (SIOCDEVPRIVATE+4)

struct ccinet_priv {
	unsigned char cid;
	struct  net_device_stats net_stats;                     /* status of the network device	*/
	int v6_cid;
};
static struct net_device *net_devices[MAX_CID_NUM];
static int main_cid[MAX_CID_NUM];


struct buff_list{
	struct sk_buff_head buff_head;
};

static struct buff_list tx_list;
static struct buff_list ack_list;

static struct delayed_work tx_worker;
static struct workqueue_struct * ccinet_work_queue;

static struct delayed_work ack_worker;
static struct delayed_work send_data_worker;


wait_queue_head_t tx_wait_queue;
wait_queue_head_t ack_wait_queue;

struct task_struct * tx_task;
struct task_struct * ack_task;


static int longTxPacket = 1;
static unsigned int ackPriority = 20;
static unsigned int longPriority = 1;


static unsigned long tx_long_packet_bytes;
static unsigned long tx_ack_packet_bytes;

static unsigned long rx_long_packet_bytes;
static unsigned long rx_ack_packet_bytes;



#if  1
#define DPRINT(fmt, args ...)     printk(fmt, ## args)
#define DBGMSG(fmt, args ...)     printk(KERN_DEBUG "CIN: " fmt, ## args)
#define ENTER()                 printk(KERN_DEBUG "CIN: ENTER %s\n", __FUNCTION__)
#define LEAVE()                 printk(KERN_DEBUG "CIN: LEAVE %s\n", __FUNCTION__)
#define FUNC_EXIT()                     printk(KERN_DEBUG "CIN: EXIT %s\n", __FUNCTION__)
#define ASSERT(a)  if (!(a)) { \
		while (1) { \
			printk(KERN_ERR "ASSERT FAIL AT FILE %s FUNC %s LINE %d\n", __FILE__, __FUNCTION__, __LINE__); \
		} \
}

#else
#define DPRINT(fmt, args ...)     printk("CIN: " fmt, ## args)
#define DBGMSG(fmt, args ...)     do {} while (0)
#define ENTER()                 do {} while (0)
#define LEAVE()                 do {} while (0)
#define FUNC_EXIT()                     do {} while (0)
#define ASSERT(a) if (!(a)) { \
		while (1) { \
			printk(KERN_CRIT "ASSERT FAIL AT FILE %s FUNC %s LINE %d\n", __FILE__, __FUNCTION__, __LINE__); \
		} \
}
#endif
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
extern void registerNZNETldisc(void);
ssize_t sendDataLowLevel(char * data, int len);

static void schedule_tx(void);
static void schedule_ack_tx(void);


char ack_buffer[TX_BUFFER_LEN];

static void ccinet_ack_work(struct work_struct* work)
{

	struct sk_buff* tx_skb;
	int copied_len = 0;
	int ret = 0;
	unsigned int n = 0;
	if(ackPriority == 0)
		return;
	while( (tx_skb = skb_dequeue(&ack_list.buff_head)) != NULL)
		{
			n ++;
			if(longTxPacket)
			{
				if( (copied_len + tx_skb->len ) > TX_BUFFER_LEN)
				{
					ret = sendDataLowLevel(ack_buffer, copied_len);
					tx_ack_packet_bytes += copied_len;
					copied_len = 0;
				}
				if( tx_skb->len > TX_BUFFER_LEN)
				{
					ret = sendDataLowLevel(tx_skb->data, tx_skb->len);
					tx_ack_packet_bytes += tx_skb->len;
					goto CheckRet;
				}
				memcpy(ack_buffer + copied_len, tx_skb->data, tx_skb->len);
				copied_len += tx_skb->len;
				ret = tx_skb->len;
			}
			else
			{
				ret = sendDataLowLevel(tx_skb->data, tx_skb->len);
				tx_ack_packet_bytes += tx_skb->len;
			}

	CheckRet:
			if (ret == -EBUSY)
			{
				printk("%s: low level device is busy, send data later...\n", __FUNCTION__);
			}
			dev_kfree_skb(tx_skb);
			if(n >= ackPriority)
			{
				//printk(KERN_DEBUG"push down %d ACK packets, break!\n", n);
				schedule_tx();
				break;
			}

		}

		ret = sendDataLowLevel(ack_buffer, copied_len);
		tx_ack_packet_bytes += copied_len;
		if( ret < 0)
		{
			printk("%s: sadly, we lost %d data forever due to %d\n", __FUNCTION__, copied_len, ret);
		}
		if(n>0)
		{
		//	printk(KERN_DEBUG"push down %d ACK packets on cpu %d\n", n, get_cpu());
		//	put_cpu();
		}
}

static char  tx_buffer[TX_BUFFER_LEN];
static void ccinet_tx_work(struct work_struct* work)
{

	struct sk_buff* tx_skb;
	int copied_len = 0;
	int ret = 0;
	int n = 0;
	if(longPriority == 0)
		return;
	while( (tx_skb = skb_dequeue(&tx_list.buff_head)) != NULL)
	{
		n ++;
		if(longTxPacket)
		{
			if( (copied_len + tx_skb->len ) > TX_BUFFER_LEN)
			{
				ret = sendDataLowLevel(tx_buffer, copied_len);
				tx_long_packet_bytes += copied_len;
				copied_len = 0;
			}
			if( tx_skb->len > TX_BUFFER_LEN)
			{
				ret = sendDataLowLevel(tx_skb->data, tx_skb->len);
				tx_long_packet_bytes += tx_skb->len;
				goto CheckRet1;
			}
			memcpy(tx_buffer + copied_len, tx_skb->data, tx_skb->len);
			copied_len += tx_skb->len;
			ret = tx_skb->len;
		}
		else
		{
			ret = sendDataLowLevel(tx_skb->data, tx_skb->len);
			tx_long_packet_bytes += tx_skb->len;

		}

CheckRet1:
		if (ret == -EBUSY)
		{
			printk("%s: low level device is busy, send data later...\n", __FUNCTION__);
		}
		dev_kfree_skb(tx_skb);


		if(n >= longPriority && skb_queue_len(&ack_list.buff_head) )
		{
			//printk(KERN_DEBUG"ack has %d, goto tx ack first", ack_len);
			schedule_ack_tx();
			break;
		}
	}


	ret = sendDataLowLevel(tx_buffer, copied_len);
	tx_long_packet_bytes += copied_len;
	if( ret < 0)
	{
		printk("%s: sadly, we lost %d data forever due to %d\n", __FUNCTION__, copied_len, ret);
	}
}

static int ccinet_ack_task(void * data)
{
	while(!kthread_should_stop())
	{
		//ccinet_ack_work(NULL);
	//	printk(KERN_DEBUG"****ccinet_tx_task waked UP !!! ......\n");
		//You can try to sleep for a while in such task.
		ccinet_ack_work(NULL);
		if(skb_queue_len(&ack_list.buff_head))
			continue;
		else
		{
		//	printk(KERN_DEBUG"******ccinet_tx_task  sleeping ......\n");
			interruptible_sleep_on(&ack_wait_queue);
		}
	}
	return 0;
}
static int ccinet_tx_task(void *data)
{
	while(!kthread_should_stop())
	{
		//ccinet_ack_work(NULL);
		//printk(KERN_DEBUG"****ccinet_ack_task waked UP !!! ......\n");
		ccinet_tx_work(NULL);
		if(skb_queue_len(&tx_list.buff_head))
			continue;
		else
		{
			//printk(KERN_DEBUG"******ccinet_ack_task  sleeping ......\n");
			interruptible_sleep_on(&tx_wait_queue);
		}
	}
	return 0;
}

static char  send_data_buffer[TX_BUFFER_LEN];

static void send_data_work(struct work_struct *work)
{
	struct sk_buff* tx_skb;
	int copied_len = 0;
	int ack_sent = 0;
	int long_sent = 0;
BegingACK:
	ack_sent = 0;
	while( (tx_skb = skb_dequeue(&ack_list.buff_head)) != NULL)
	{

		if (longTxPacket) {
			if ((copied_len + tx_skb->len ) > TX_BUFFER_LEN) {
				sendDataLowLevel(send_data_buffer, copied_len);
				copied_len = 0;
			}
			if (tx_skb->len > TX_BUFFER_LEN)
				sendDataLowLevel(tx_skb->data, tx_skb->len);
			else {
				memcpy(send_data_buffer + copied_len, tx_skb->data, tx_skb->len);
				copied_len += tx_skb->len;
			}
		} else
			sendDataLowLevel(tx_skb->data, tx_skb->len);

		dev_kfree_skb(tx_skb);
		ack_sent ++;
		if(ack_sent >= ackPriority && skb_queue_len(&tx_list.buff_head) > 0 )
			goto BegingLong;
	}
BegingLong:
	long_sent = 0;
	if(longPriority == 0)
		goto BegingACK;
	while( (tx_skb = skb_dequeue(&tx_list.buff_head)) != NULL)
	{

		if (longTxPacket) {
			if ((copied_len + tx_skb->len) > TX_BUFFER_LEN) {
				sendDataLowLevel(send_data_buffer, copied_len);
				copied_len = 0;
			}
			if (tx_skb->len > TX_BUFFER_LEN)
				sendDataLowLevel(tx_skb->data, tx_skb->len);
			else {
				memcpy(send_data_buffer + copied_len, tx_skb->data, tx_skb->len);
				copied_len += tx_skb->len;
			}
		} else
			sendDataLowLevel(tx_skb->data, tx_skb->len);

		dev_kfree_skb(tx_skb);
		long_sent ++;
		if(long_sent >= longPriority && skb_queue_len(&ack_list.buff_head) > 0 )
			goto BegingACK;
	}
	sendDataLowLevel(send_data_buffer, copied_len);
	copied_len = 0;
	if(skb_queue_len(&ack_list.buff_head) > 0 || skb_queue_len(&tx_list.buff_head) > 0)
		goto BegingACK;
}

struct sk_buff * tx_fixup(struct net_device* netdev, struct sk_buff *skb, gfp_t flags)
{
	struct ccinethdr	*hdr;
	struct sk_buff			*skb2;
	unsigned	len;
	unsigned	tailpad;
	struct ccinet_priv* devobj = netdev_priv(netdev);
	struct iphdr *ip_header = (struct iphdr *)ip_hdr(skb);
	unsigned cid = devobj->cid;

	if (ip_header->version == 6) {
		if (devobj->v6_cid > -1)
			cid = devobj->v6_cid;
	}

	len = skb->len;
	tailpad = tx_padding_size(sizeof *hdr + len);
	/* Following CP new design without extro padding
	if(longTxPacket)
		tailpad += 48;
		*/
	if (likely(!skb_cloned(skb))) {
		int	headroom = skb_headroom(skb);
		int tailroom = skb_tailroom(skb);

		/* enough room as-is? */
		if (unlikely(sizeof *hdr + tailpad <= headroom + tailroom)) {
			/* do not need to be readjusted */
			if(sizeof *hdr <= headroom && tailpad <= tailroom)
				goto fill;

			skb->data = memmove(skb->head + sizeof *hdr,
				skb->data, len);
			skb_set_tail_pointer(skb, len);
			goto fill;
		}
	}

	/* create a new skb, with the correct size (and tailpad) */
	skb2 = skb_copy_expand(skb, sizeof *hdr, tailpad + 1, flags);
	dev_kfree_skb_any(skb);
	if (unlikely(!skb2))
		return skb2;
	skb = skb2;

	/* fill out the Nezha header and expand packet length to 8 bytes
	 * alignment.  we won't bother trying to batch packets;
	 */
fill:
	hdr = (void *) __skb_push(skb, sizeof *hdr);
	memset(hdr, 0, sizeof *hdr);
	hdr->iplen = cpu_to_be16(len);
	hdr->cid = cid;
	hdr->offset_len = 0;
	memset(skb_put(skb, tailpad), 0, tailpad);

	return skb;
}



/*static void ccinet_print_status(unsigned long data)
{
	static int sec_interval = 2;
	static unsigned long rx_last;
	static unsigned long tx_last;

	static unsigned long tx_long_last;
	static unsigned long tx_ack_last;

	static unsigned long rx_long_last;
	static unsigned long rx_ack_last;

	struct ccinet_priv* devobj = (struct ccinet_priv*)data;
	unsigned long rx_delta = devobj->net_stats.rx_bytes - rx_last;
	unsigned long tx_delta = devobj->net_stats.tx_bytes - tx_last;
	unsigned long tx_speed = ((tx_delta * 8) / sec_interval);
	unsigned long rx_speed = ((rx_delta * 8) / sec_interval);

	unsigned long tx_long_delta = tx_long_packet_bytes - tx_long_last;
	unsigned long tx_ack_delta = tx_ack_packet_bytes - tx_ack_last;
	unsigned long tx_long_speed = ((tx_long_delta* 8) / sec_interval);
	unsigned long tx_ack_speed = ((tx_ack_delta * 8) / sec_interval);

	unsigned long rx_long_delta = rx_long_packet_bytes - rx_long_last;
	unsigned long rx_ack_delta = rx_ack_packet_bytes - rx_ack_last;
	unsigned long rx_long_speed = ((rx_long_delta* 8) / sec_interval);
	unsigned long rx_ack_speed = ((rx_ack_delta * 8) / sec_interval);

	printk("CCinet %d tx %ld bytes in 5s, speed %ld bit/s, rx %ld bytes in 5s, speed %ld bit/s\n", devobj->cid,
		tx_delta, tx_speed,
		rx_delta, rx_speed);

	printk("CCinet %d tx long %ld bytes in 5s, speed %ld bit/s, tx ack %ld bytes in 5s, speed %ld bit/s\n", devobj->cid,
		tx_long_delta, tx_long_speed,
		tx_ack_delta, tx_ack_speed);

	printk("CCinet %d rx long %ld bytes in 5s, speed %ld bit/s, rx ack %ld bytes in 5s, speed %ld bit/s\n", devobj->cid,
		rx_long_delta, rx_long_speed,
		rx_ack_delta, rx_ack_speed);

	rx_last = devobj->net_stats.rx_bytes;
	tx_last = devobj->net_stats.tx_bytes;
	tx_long_last = tx_long_packet_bytes;
	tx_ack_last = tx_ack_packet_bytes;
	rx_long_last = rx_long_packet_bytes;
	rx_ack_last = rx_ack_packet_bytes;
}*/
///////////////////////////////////////////////////////////////////////////////////////
// Network Operations
///////////////////////////////////////////////////////////////////////////////////////
static int ccinet_open(struct net_device* netdev)
{
	ENTER();
	netif_start_queue(netdev);
	LEAVE();
	return 0;
}

static int ccinet_stop(struct net_device* netdev)
{
	struct ccinet_priv* devobj;
	ENTER();
	devobj = netdev_priv(netdev);
	netif_stop_queue(netdev);
	memset(&devobj->net_stats, 0, sizeof(devobj->net_stats));
	if (devobj->v6_cid > -1) {
		main_cid[devobj->v6_cid] = -1;
		devobj->v6_cid = -1;
	}
	LEAVE();
	return 0;
}

static void schedule_send(void)
{
	if(skb_queue_len(&tx_list.buff_head) <= LOW_WATER_MARK / 2 &&
		skb_queue_len(&ack_list.buff_head) <= ackPriority / 2)
		queue_delayed_work(ccinet_work_queue, &send_data_worker, msecs_to_jiffies(2));
	else
		queue_delayed_work(ccinet_work_queue, &send_data_worker, 0);
}


static void schedule_tx(void)
{
	if(tx_task != NULL)
	{
		//printk(KERN_DEBUG"ccinet_tx: schedule_tx - wake up process\n");
		wake_up_interruptible(&tx_wait_queue);
	}
	else
		queue_delayed_work(ccinet_work_queue, &tx_worker, 0);
}

static void schedule_ack_tx(void)
{
	if(ack_task != NULL)
		wake_up_interruptible(&ack_wait_queue);
	else
		queue_delayed_work(ccinet_work_queue, &ack_worker, 2);
}


static int ccinet_tx(struct sk_buff* skb, struct net_device* netdev)
{
	struct ccinet_priv* devobj = netdev_priv(netdev);
	struct sk_buff* tx_skb;
	struct iphdr* ip_header = (struct iphdr*)skb->data;
	int is_ack = 0;

	if (ip_header->version == 4) {
		if(skb->len <= 96) //20 IP header + 20 TCP header
		{
			is_ack = 1;
		}
	} else if (ip_header->version == 6) {
		if(skb->len <= 128) //36 IP header + 20 TCP header
		{
			is_ack = 1;
		}
	}

	if( skb_queue_len(&tx_list.buff_head) > HIGHT_WATER_MARK && !is_ack)
	{

		printk(KERN_DEBUG"ccinet_tx: queue is full - packet dropped\n");
		devobj->net_stats.tx_dropped ++;
		//schedule_tx();
		dev_kfree_skb_any(skb);
		schedule_send();
		return NETDEV_TX_OK;
	}

	netdev->trans_start = jiffies;
	tx_skb = tx_fixup(netdev,skb,GFP_ATOMIC);

	if(tx_skb != NULL)
	{
		if(is_ack)
		{
			skb_queue_tail(&ack_list.buff_head, tx_skb);
			//schedule_ack_tx();
		}
		else
		{
			skb_queue_tail(&tx_list.buff_head, tx_skb);
			//schedule_tx();
		}
		devobj->net_stats.tx_packets++;
		devobj->net_stats.tx_bytes += skb->len;
		schedule_send();
		return NETDEV_TX_OK;
	}
	else
	{
		printk(KERN_DEBUG"ccinet_tx: tx_fixup faile\n");
		devobj->net_stats.tx_dropped ++;
		return NETDEV_TX_OK;
	}
}

static void ccinet_tx_timeout(struct net_device* netdev)
{
	struct ccinet_priv* devobj = netdev_priv(netdev);

	ENTER();
	devobj->net_stats.tx_errors++;
//	netif_wake_queue(netdev); // Resume tx
	return;
}

static struct net_device_stats *ccinet_get_stats(struct net_device *dev)
{
	struct ccinet_priv* devobj;

	devobj = netdev_priv(dev);
	ASSERT(devobj);
	return &devobj->net_stats;
}

static int ccinet_rx(struct net_device* netdev, const unsigned char* packet, int pktlen)
{

	struct sk_buff *skb;
	struct ccinet_priv *priv = netdev_priv(netdev);
	const struct iphdr* ip_header = (const struct iphdr*)packet;
	int is_ack = 0;
	__be16 protocol;

	if (ip_header->version == 4) {
		protocol = htons(ETH_P_IP);
		if(pktlen <= 96) //20 IP header + 20 TCP header
			is_ack = 1;
	} else if (ip_header->version == 6) {
		protocol = htons(ETH_P_IPV6);
		if(pktlen <= 128) //36 IP header + 20 TCP header
			is_ack = 1;
	} else {
		printk(KERN_ERR "ccinet_rx: invalid ip version: %d\n", ip_header->version);
		priv->net_stats.rx_dropped++;
		hex_packet(packet,pktlen > 100?100:pktlen);
		goto out;
	}

	if (!is_ack)
	{
		rx_long_packet_bytes += pktlen;
	}
	else
	{
		rx_ack_packet_bytes += pktlen;
	}

#if 0
	unsigned int tmp = ip_header->saddr;
	ip_header->saddr = ip_header->daddr;
	ip_header->daddr = tmp;
#endif

	skb = dev_alloc_skb(pktlen);

	if (!skb)
	{
		printk(KERN_NOTICE
			"ccinet_rx: low on mem - packet dropped\n");
		priv->net_stats.rx_dropped++;
		goto out;
	}

	memcpy(skb_put(skb, pktlen), packet, pktlen);

	/* Write metadata, and then pass to the receive level */

	skb->dev = netdev;
	skb->protocol = protocol;
	skb->ip_summed = CHECKSUM_UNNECESSARY; /* don't check it */
	priv->net_stats.rx_packets++;
	priv->net_stats.rx_bytes += pktlen;
	if( netif_rx_ni(skb) != NET_RX_SUCCESS )
	{
		printk(KERN_NOTICE "ccinet_rx: netif_rx faile - packet dropped\n");
		priv->net_stats.rx_dropped++;
	}
	//where to free skb?

	return 0;

 out:
	return -1;
}

static int ccinet_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	int err = 0;
	struct ccinet_priv *priv = netdev_priv(netdev);
	switch (cmd) {
	case CCINET_IOCTL_SET_V6_CID:
	{
		int v6_cid = rq->ifr_ifru.ifru_ivalue;
		if (v6_cid < 0 || v6_cid >= MAX_CID_NUM) {
			err = -EINVAL;
			break;
		}
		priv->v6_cid = v6_cid;
		main_cid[v6_cid] = priv->cid;
		break;
	}
	default:
		err = -EOPNOTSUPP;
		break;
	}
	return err;
}


int data_rx(const unsigned char* packet, int len, unsigned long cid)
{
	struct net_device *dev;
	int i;
	if (cid >= MAX_CID_NUM)
		return -1;
	i = main_cid[cid] >= 0 ? main_cid[cid] : cid;
	dev = net_devices[i];
	if (!(dev->flags & IFF_UP)) {
		printk(KERN_ERR "%s: netdevice %s not up for cid:%d\n",
			       __func__, dev->name, (int)cid);
	}
	ccinet_rx(dev, packet, len);
	return len;
}

///////////////////////////////////////////////////////////////////////////////////////
// Initialization
///////////////////////////////////////////////////////////////////////////////////////


static const struct net_device_ops cci_netdev_ops = {
	.ndo_open		= ccinet_open,
	.ndo_stop		= ccinet_stop,
	.ndo_start_xmit 	= ccinet_tx,
	.ndo_tx_timeout		= ccinet_tx_timeout,
	.ndo_get_stats 	= ccinet_get_stats,
	.ndo_do_ioctl = ccinet_ioctl
};


static void ccinet_setup(struct net_device* netdev)
{
	ENTER();
	netdev->netdev_ops = &cci_netdev_ops;
	netdev->type = ARPHRD_VOID;
	netdev->mtu = 1500;
	netdev->addr_len = 0;
	netdev->tx_queue_len = 1000;
	netdev->flags = IFF_NOARP;
	netdev->hard_header_len = 0;
	netdev->priv_flags  &= ~IFF_XMIT_DST_RELEASE;
	LEAVE();
}

static int task_init(void)
{
	init_waitqueue_head(&tx_wait_queue);
	init_waitqueue_head(&ack_wait_queue);
	tx_task = kthread_create(ccinet_tx_task,NULL, "CCI NET TX TASK");
	ack_task = kthread_create(ccinet_ack_task,NULL, "CCI NET ACK TASK");

	if(IS_ERR(tx_task))
	{
		printk(KERN_ERR"cci net:%s:create tx task faile!\n",__func__);
		tx_task = NULL;
		return -1;
	}
	if(IS_ERR(ack_task))
	{
		printk(KERN_ERR"cci net:%s:create ack task faile!\n",__func__);
		ack_task = NULL;
		return -1;
	}
	wake_up_process(tx_task);
	wake_up_process(ack_task);
	return 0;

}

static int task_deinit(void)
{
	int  err = -1;
	if(tx_task)
	{
		wake_up_interruptible(&tx_wait_queue);
		err = kthread_stop(tx_task);
	}
	tx_task = NULL;
	if(ack_task)
	{
		wake_up_interruptible(&ack_wait_queue);
		err = kthread_stop(ack_task);
	}
	ack_task = NULL;
	return err;
}
static int __init ccinet_init(void)
{
	int i;
	for (i = 0; i < MAX_CID_NUM; i++) {
		char ifname[32];
		struct net_device *dev;
		struct ccinet_priv *priv;
		int ret;

		main_cid[i] = -1;
		sprintf(ifname, INTFACE_NAME"%d", i);
		dev = alloc_netdev(sizeof(struct ccinet_priv), ifname, ccinet_setup);

		if (!dev) {
			printk(KERN_ERR "%s: alloc_netdev for %s fail\n", __FUNCTION__, ifname);
			return -ENOMEM;
		}
		ret = register_netdev(dev);
		if (ret) {
			printk(KERN_ERR "%s: register_netdev for %s fail\n", __FUNCTION__, ifname);
			free_netdev(dev);
			return ret;
		}
		priv = netdev_priv(dev);
		memset(priv, 0, sizeof(struct ccinet_priv));
		priv->cid = i;
		priv->v6_cid = -1;
		net_devices[i] = dev;
	}

	skb_queue_head_init(&tx_list.buff_head);
	skb_queue_head_init(&ack_list.buff_head);

	//ccinet_work_queue = create_singlethread_workqueue("ccinet work queue");
	//ccinet_work_queue = alloc_workqueue("ccinet wq", WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	ccinet_work_queue = alloc_workqueue("ccinet wq",  WQ_MEM_RECLAIM | WQ_NON_REENTRANT |WQ_HIGHPRI, 0);
	//INIT_DELAYED_WORK(&tx_worker,ccinet_tx_work);
	//INIT_DELAYED_WORK(&ack_worker, ccinet_ack_work);
	INIT_DELAYED_WORK(&send_data_worker, send_data_work);
	task_init();
	registerNZNETldisc();
	return 0;
};

static void __exit ccinet_exit(void)
{
	int i;
	for (i = 0; i < MAX_CID_NUM; i++) {
		unregister_netdev(net_devices[i]);
		free_netdev(net_devices[i]);
		net_devices[i] = NULL;
	}
	task_deinit();

}

module_init(ccinet_init);
module_exit(ccinet_exit);
module_param(longTxPacket, int, 0644);
module_param(ackPriority,int,0644);
module_param(longPriority,int,0644);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marvell");
MODULE_DESCRIPTION("Marvell CI Network Driver");

