/******************************************************************************
 * HSIC TTY driver
 *
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

#define DRIVER_DESC "HSIC tty Driver for Nezha modem"
//#define DEBUG

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/uaccess.h>

#include <linux/list.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/pm_qos.h>

#define hsictty_dbg(fmt, args ...)	pr_debug("hsic tty: " fmt, ## args)
#define hsictty_info(fmt, args ...)	printk(KERN_INFO "hsic tty: " fmt, ## args)
#define hsictty_error(fmt, args ...)	printk(KERN_ERR "hsic tty: " fmt, ## args)

/*Nezha modem vendor and id*/
#define NEZHA_MODEM_VENDOR_ID			0x1286
#define NEZHA_MODEM_PRODUCT_ID			0x8130

#define HSIC_TTY_IOC_MAGIC 'H'
#define HSIC_TTY_IOCTL_SET_MULTIMODE	_IOW(HSIC_TTY_IOC_MAGIC, 1, int)
#define HSIC_TTY_IOCTL_HSIC_RESET	_IOW(HSIC_TTY_IOC_MAGIC, 2, int)
#define HSIC_TTY_IOCTL_HSIC_PM_ENABLE	_IOW(HSIC_TTY_IOC_MAGIC, 3, int)

#define N_IN_URB	4
#define N_OUT_URB	4
#define IN_BUFLEN	(16*1024)
#define OUT_BUFLEN	(16*1024)

#define HSIC_CHANNEL_NUMS	7
#define HSIC_DATA_CHANNEL_NUMS	6
#define HSIC_DATA_START_CHANNEL 0

#define HSIC_HANDSHAKE_HANDLE	0x0F
#define HSIC_HANDSHAKE_REQUEST	0xE0

//#define USE_READ_WORK

/*flow control with tty core*/
#define USE_TTY_CORE_BUFFER	/*do not use tty allocated buffer */
#define USB_TTY_THROTTLE_CB	/*use tty throttle callback notify fc. */

#define BACKUP_DATA_DUMP

#ifdef BACKUP_DATA_DUMP
#include <linux/skbuff.h>
static struct sk_buff_head backup_queue[2][HSIC_CHANNEL_NUMS];
static void backup_log(u32 ch, int direction, char *data, u32 length);
static void backup_dump(u32 ch, int direction);
static int dumped;
#endif

static struct pm_qos_request dl_kfc_num_qos;
static struct pm_qos_request dl_kfc_freq_qos;

struct _HSIC_CHANNEL_HANDSHAKE {
	u32 params:16;
	u32 res:4;
	u32 action:4;
	u32 channel:4;
	u32 cmd:4;
} __attribute__ ((packed));

struct _HSICTTY_MSG {
	struct urb *urb;
	struct list_head link;
};

struct hsictty_intf_private {
	spinlock_t susp_lock;
	unsigned int suspended:1;
	int in_flight;

	int multi_channel_mode;
	unsigned long channel_open_flag;

	int support_pm;

	struct semaphore handshake_sem;

	struct wake_lock tx_wakelock;
	struct wake_lock rx_wakelock;
};

struct hsictty_port_private {
	struct urb *in_urbs[N_IN_URB];
	u8 *in_buffer[N_IN_URB];
	struct urb *out_urbs[N_OUT_URB];
	u8 *out_buffer[N_OUT_URB];
	unsigned long out_busy;
	int opened;
	struct usb_anchor delayed_urb;

	unsigned long tx_start_time[N_OUT_URB];

	u8 channel;

	struct _HSICTTY_MSG read_msg[N_IN_URB];
	struct list_head pool;
	spinlock_t pool_lock;
	struct completion rx_push_notifier;
#ifdef USE_READ_WORK
	struct work_struct hsictty_read_work;
#else
	struct completion rx_notifier;
	struct task_struct *rx_task;
	u8 thread_exit;
#endif

	struct semaphore ch_sem_r;
	struct semaphore ch_sem_w;

	struct completion tx_notifier;

	int lch_opened;
};

static int hsictty_handshake_setup(struct usb_serial_port *port, int channel,
				   int open_action)
{
	struct usb_serial *serial = port->serial;
	struct hsictty_intf_private *intfdata = usb_get_serial_data(serial);

	int ret = -1, cmd = 0;

	struct _HSIC_CHANNEL_HANDSHAKE *frame_cmd;
	u32 frame = 0, s_frame = 0;
	u16 frame_len = sizeof(struct _HSIC_CHANNEL_HANDSHAKE);

	hsictty_dbg("%s: channel:%d\n", __func__, channel);

	frame_cmd = (struct _HSIC_CHANNEL_HANDSHAKE *)(&frame);
	cmd = HSIC_HANDSHAKE_HANDLE;
	frame_cmd->cmd = cmd;
	frame_cmd->channel = channel;
	frame_cmd->action = open_action;
	frame_cmd->res = 0;
	frame_cmd->params = 0;

	down(&intfdata->handshake_sem);
	ret = usb_control_msg(serial->dev,
			      usb_sndctrlpipe(serial->dev, 0),
			      HSIC_HANDSHAKE_REQUEST,
			      USB_TYPE_VENDOR | USB_DIR_OUT |
			      USB_RECIP_ENDPOINT, 0, 0, &frame, frame_len,
			      5 * 1000);

	if (ret != frame_len) {
		hsictty_error("%s: handshake write error in channel:%d, ret:%d\n",
			    __func__, channel, ret);
		goto out;
	}

	s_frame = frame;
	frame = 0;

	ret = usb_control_msg(serial->dev,
			      usb_rcvctrlpipe(serial->dev, 0),
			      HSIC_HANDSHAKE_REQUEST,
			      USB_TYPE_VENDOR | USB_DIR_IN | USB_RECIP_ENDPOINT,
			      0, 0, &frame, frame_len, 5 * 1000);

	if (ret != frame_len) {
		hsictty_error("%s: handshake read error in channel:%d, ret:%d\n",
			    __func__, channel, ret);
		goto out;
	} else {
		frame_cmd = (struct _HSIC_CHANNEL_HANDSHAKE *)&frame;
		if ((HSIC_HANDSHAKE_HANDLE == frame_cmd->cmd)
		    && (channel == frame_cmd->channel)
		    && (open_action == frame_cmd->action)) {
			ret = (frame_cmd->res > 0) ? 0 : -1;
			hsictty_dbg("%s: handshake res %s in channel:%d\n",
				    __func__, ((ret < 0) ? "reject" : "succ"),
				    channel);
		} else {
			hsictty_dbg("%s: handshake res error in channel:%d\n",
				    __func__, channel);
		}

		if (ret) {
			unsigned char *p = (unsigned char *)&s_frame;
			hsictty_error("%s: send %02x %02x %02x %02x\n", __func__,
							*p, *(p+1), *(p+2), *(p+3));
			p = (unsigned char *)&frame;
			hsictty_error("%s: recv %02x %02x %02x %02x\n", __func__,
							*p, *(p+1), *(p+2), *(p+3));
		}
	}
out:
	up(&intfdata->handshake_sem);
	return ret;
}

extern bool sys_suspending;

void wakeup_device(struct usb_serial_port *port, u8 if_pull_GPIO)
{
	int err;
	int cnt = 12;
	struct hsictty_port_private *portdata;
	struct hsictty_intf_private *intfdata;

	portdata = usb_get_serial_port_data(port);
	intfdata = usb_get_serial_data(port->serial);

	if (if_pull_GPIO) {
		/* prevent system going to suspend as urb queued
		 * deffer urb at beginning of resume until system ready
		 */
		while (sys_suspending && cnt--)
			usleep_range(10 * 1000, 10 * 1000 + 1000);

		if (sys_suspending)
			hsictty_error("%s: sys not ready, longer ??\n", __func__);

		err = usb_autopm_get_interface(port->serial->interface);
	} else {
		err = usb_autopm_get_interface_async(port->serial->interface);
	}
	if (err < 0) {
		hsictty_error
		    ("%s: auto pm get error[%d] in channel:%d, pm cnt:%d\n",
		     __func__, err, portdata->channel,
		     atomic_read(&port->serial->interface->dev.power.
				 usage_count));
		usb_autopm_get_interface_no_resume(port->serial->interface);
	}
}

static int check_port_free(struct hsictty_port_private *portdata)
{
	int i = 0;
	for (i = 0; i < N_OUT_URB; i++) {
		if (test_bit(i, &portdata->out_busy)) {
			continue;
		} else
			break;
	}

	if (i == N_OUT_URB) {
		hsictty_dbg("%s: no free port in channel:%d now,\n", __func__,
			    portdata->channel);
		return 0;
	} else {
		hsictty_dbg("%s: one free msg detected in channel:%d\n",
			    __func__, portdata->channel);
		return 1;
	}
}

static int get_read_msg_index(struct hsictty_port_private *portdata)
{
	int i = 0;
	unsigned long flags;
	spin_lock_irqsave(&portdata->pool_lock, flags);

	for (i = 0; i < N_IN_URB; i++) {
		if (portdata->read_msg[i].urb == NULL) {
			spin_unlock_irqrestore(&portdata->pool_lock, flags);
			return i;
		}
	}
	spin_unlock_irqrestore(&portdata->pool_lock, flags);

	return -1;
}

static int channel_verify(struct usb_serial_port *port, int channel)
{
	struct hsictty_port_private *portdata;
	struct hsictty_intf_private *intfdata;
	portdata = usb_get_serial_port_data(port);
	intfdata = usb_get_serial_data(port->serial);

	if (!intfdata->multi_channel_mode && channel != HSIC_DATA_START_CHANNEL)
		return 0;

	if ((intfdata->multi_channel_mode)
	    && (channel < HSIC_DATA_START_CHANNEL
		|| channel >= HSIC_DATA_CHANNEL_NUMS))
		return 0;

	return 1;
}

/* Write */
int hsictty_write(struct tty_struct *tty, struct usb_serial_port *port,
		  const unsigned char *buf, int count)
{
	struct hsictty_port_private *portdata;
	struct hsictty_intf_private *intfdata;
	int i;
	int left, todo;
	struct urb *this_urb = NULL;
	int err;
	unsigned long flags;
	int rc = -1;
	int ret = -EINVAL;
	int channel = -1;

	portdata = usb_get_serial_port_data(port);
	intfdata = usb_get_serial_data(port->serial);

	down(&portdata->ch_sem_w);

	channel = tty->index;
	if (!channel_verify(port, channel)) {
		hsictty_error("%s: invalid channel[%d]!\n", __func__, channel);
		goto out;
	}

	hsictty_dbg("%s: write (%d chars) in channel[%d]\n", __func__, count,
		    channel);

	i = 0;
	left = count;

write_wait:
	if (!check_port_free(portdata)) {
		up(&portdata->ch_sem_w);
		rc = wait_for_completion_interruptible_timeout(&portdata->tx_notifier,
							       5 * HZ);
		down(&portdata->ch_sem_w);
		INIT_COMPLETION(portdata->tx_notifier);
		if (rc <= 0) {
			ret = rc < 0 ? -EINTR : -EBUSY;
			hsictty_error
			    ("%s: No free URB msg left in channel:%d(%s), inflight:%d\n",
			     __func__, channel, rc < 0 ? "EINTR" : "EBUSY",
			     intfdata->in_flight);
			if (rc < 0)
				goto out;
		}
		if (port->serial->disconnected || !portdata->opened) {
			hsictty_info
			    ("%s: detect disconnect or close while writing on channel:%d, need exit\n",
			     __func__, channel);
			goto out;
		} else
			goto write_wait;
	}

	for (i = 0; left > 0 && i < N_OUT_URB; i++) {
		todo = left;
		if (todo > OUT_BUFLEN)
			todo = OUT_BUFLEN;

		this_urb = portdata->out_urbs[i];
		if (test_and_set_bit(i, &portdata->out_busy)) {
			if (time_before(jiffies, portdata->tx_start_time[i] + 10 * HZ))
				continue;
			hsictty_error
			    ("%s: write URB(%d) on channel:%d pending too long, in_flight:%d\n",
			     __func__, i, channel, intfdata->in_flight);
			//usb_unlink_urb(this_urb);
			continue;
		}
		hsictty_dbg
		    ("%s: find a URB for channel:%d, endpoint:%d, buf %d\n",
		     __func__, channel, usb_pipeendpoint(this_urb->pipe), i);

		wake_lock_timeout(&intfdata->tx_wakelock, 0.5 * HZ);
		wakeup_device(port, 1);

		/* send the data */
		memcpy(this_urb->transfer_buffer, buf, todo);
		this_urb->transfer_buffer_length = todo;

		spin_lock_irqsave(&intfdata->susp_lock, flags);
		if (intfdata->suspended) {
			usb_anchor_urb(this_urb, &portdata->delayed_urb);
			spin_unlock_irqrestore(&intfdata->susp_lock, flags);
			hsictty_info
			    ("%s: anchor URB %p for channel:%d, endpoint:%d, buf %d, in_flight:%d, pm use cnt:%d\n",
			     __func__, this_urb, channel,
			     usb_pipeendpoint(this_urb->pipe), i,
			     intfdata->in_flight,
			     atomic_read(&port->serial->interface->dev.
					 power.usage_count));
		} else {
			intfdata->in_flight++;
			spin_unlock_irqrestore(&intfdata->susp_lock, flags);
			err = usb_submit_urb(this_urb, GFP_ATOMIC);
			if (err) {
				hsictty_error
				    ("usb_submit_urb (write bulk) failed in channel:%d, endpoint:%d"
				     "(%d)", usb_pipeendpoint(this_urb->pipe),
				     channel, err);
				clear_bit(i, &portdata->out_busy);
				spin_lock_irqsave(&intfdata->susp_lock, flags);
				intfdata->in_flight--;
				spin_unlock_irqrestore(&intfdata->susp_lock,
						       flags);
				usb_autopm_put_interface_async(port->serial->
							       interface);
				break;
			}
		}

		portdata->tx_start_time[i] = jiffies;
		buf += todo;
		left -= todo;
	}

	count -= left;
	hsictty_dbg("%s: wrote (did %d) in channel:%d, endpoint:%d\n", __func__,
		    count, channel, usb_pipeendpoint(this_urb->pipe));
	ret = count;
out:
	up(&portdata->ch_sem_w);
	return ret;
}

static void hsictty_write_callback(struct urb *urb)
{
	struct usb_serial_port *port;
	struct hsictty_port_private *portdata;
	struct hsictty_intf_private *intfdata;
	int i;
	port = urb->context;
	intfdata = usb_get_serial_data(port->serial);
	portdata = usb_get_serial_port_data(port);

	if (urb->actual_length <= 0) {
		hsictty_error
		    ("%s: write failed, write length: %d in channel:%d, endpoint:%d\n",
		     __func__, urb->actual_length, portdata->channel,
		     usb_pipeendpoint(urb->pipe));
	} else {
		hsictty_dbg("%s: write length: %d in channel:%d, endpoint:%d\n",
			    __func__, urb->actual_length, portdata->channel,
			    usb_pipeendpoint(urb->pipe));
	}
#ifdef BACKUP_DATA_DUMP
	if (!dumped)
		backup_log(portdata->channel, 1,
				urb->transfer_buffer, urb->transfer_buffer_length);
#endif

	usb_serial_port_softint(port);

	usb_autopm_put_interface_async(port->serial->interface);
	portdata = usb_get_serial_port_data(port);
	spin_lock(&intfdata->susp_lock);
	intfdata->in_flight--;
	spin_unlock(&intfdata->susp_lock);

	for (i = 0; i < N_OUT_URB; ++i) {
		if (portdata->out_urbs[i] == urb) {
			smp_mb__before_clear_bit();
			hsictty_dbg
			    ("%s: urb(%d) freed on channel:%d, endpoint:%d, in_flight:%d, pm use cnt:%d\n",
			     __func__, i, portdata->channel,
			     usb_pipeendpoint(urb->pipe), intfdata->in_flight,
			     atomic_read(&port->serial->interface->dev.power.
					 usage_count));
			clear_bit(i, &portdata->out_busy);
			complete_all(&portdata->tx_notifier);
			break;
		}
	}
}

void process_rx_data(struct hsictty_port_private *portdata)
{
	struct _HSICTTY_MSG *msg = NULL;
	unsigned long flags;

//      hsictty_dbg("%s: \n", __func__);

	while (!list_empty(&portdata->pool)) {
		struct tty_struct *tty = NULL;
		struct usb_serial_port *port;
		struct hsictty_intf_private *intfdata;
		int status;
		int err;
		int endpoint;
		struct urb *urb = NULL;
		unsigned char *data;
		u16 channel;
		channel = portdata->channel;
		spin_lock_irqsave(&portdata->pool_lock, flags);
		msg = list_first_entry(&portdata->pool, struct _HSICTTY_MSG, link);
		if (NULL == msg) {
			spin_unlock_irqrestore(&portdata->pool_lock, flags);
			hsictty_error("%s: try to push a NULL msg\n", __func__);
			goto out;
		}
		list_del(&msg->link);
		urb = msg->urb;
		spin_unlock_irqrestore(&portdata->pool_lock, flags);

		port = urb->context;
		intfdata = usb_get_serial_data(port->serial);
		status = urb->status;
		data = urb->transfer_buffer;
		endpoint = usb_pipeendpoint(urb->pipe);

		hsictty_dbg("read length: %d in channel:%d, endpoint:%d\n",
			    urb->actual_length, channel, endpoint);

		wake_lock_timeout(&intfdata->rx_wakelock, HZ);
		down(&portdata->ch_sem_r);
		if (port->serial && !port->serial->disconnected && portdata->opened) {
			tty = tty_port_tty_get(&port->port);
		} else {
			hsictty_info
			    ("%s: tty[%d] already close, no need push\n",
			     __func__, channel);
		}
		if (tty) {
			if (urb->actual_length) {
#ifdef USE_TTY_CORE_BUFFER	//do not use tty buffer
				u32 nleft = 0, npushed = 0, once =
				    2048, throttle_limit = 120;
				unsigned char *ptr = NULL;
				nleft = urb->actual_length;
				ptr = data;
				hsictty_dbg("nleft = %d\n", nleft);
				while (nleft > 0) {
					int receive_room = 0;
					u32 receive_room_limit = IN_BUFLEN;
					if (tty->receive_room >
					    receive_room_limit) {
						tty->ldisc->ops->
						    receive_buf(tty, ptr, NULL,
								nleft);
						nleft = 0;
						continue;
					}
wait_rx_allowed:
					//Note: should not use tty receive_room, it is not protected in tty core, will cause data lost.
					receive_room = tty->receive_room;
					if (receive_room <= throttle_limit) {
						int rc;
						hsictty_dbg
						    ("ch:%d wait tty room,room left(%d)\n",
						     channel, tty->receive_room);
#ifdef USB_TTY_THROTTLE_CB
						up(&portdata->ch_sem_r);
						rc = wait_for_completion_interruptible_timeout(&portdata->rx_push_notifier, 5 * HZ);
						down(&portdata->ch_sem_r);
						//INIT_COMPLETION(portdata->rx_push_notifier);
						if (rc <= 0) {
							hsictty_error
							    ("%s: error wait push in in channel:%d, endpoint:%d, error(%s)\n\n",
							     __func__,
							     channel, endpoint,
							     (rc < 0) ? "-EINT" : "-EBUSY");
							if (rc < 0)
								break;
						}
						if ((port->serial && port->serial->disconnected) || !portdata->opened) {
							hsictty_info
							    ("%s: detect disconnect or close while reading on channel:%d, need exit\n",
							     __func__, channel);
							break;
						} else
							goto wait_rx_allowed;
#else
						if (waitqueue_active(&tty->read_wait))
							wake_up_interruptible(&tty->read_wait);
						usleep_range(1, 2);
#endif
						goto wait_rx_allowed;
					}

					if (nleft >= receive_room - throttle_limit) {
						once = receive_room - throttle_limit;
						npushed = once;
						nleft -= once;
					} else {
						npushed = nleft;
						nleft = 0;
					}
					hsictty_dbg
					    ("%s:ch:%d,ep:%d npushed:%d,recvroom:%d\n",
					     __func__, channel, endpoint,
					     npushed, tty->receive_room);
					tty->ldisc->ops->receive_buf(tty, ptr, NULL, npushed);
					ptr += npushed;
				}
#else
				u32 nleft = 0, npushed = 0, once = 16000;
				unsigned char *ptr = NULL;
				nleft = urb->actual_length;
				ptr = data;
				while (nleft > 0) {
					npushed = tty_insert_flip_string(tty->port,
								   ptr, nleft);
					tty_flip_buffer_push(tty->port);
					nleft -= npushed;
					ptr += npushed;
					//usleep_range(1,2);
					//udelay(5);
				}
#endif
			}
			tty_kref_put(tty);
		}
		up(&portdata->ch_sem_r);
		/* Resubmit urb so we continue receiving */
		spin_lock_irqsave(&portdata->pool_lock, flags);
		msg->urb = NULL;
		spin_unlock_irqrestore(&portdata->pool_lock, flags);
		if (portdata->opened) {
			err = usb_submit_urb(urb, GFP_ATOMIC);
			if (err) {
				if (err != -EPERM) {
					hsictty_error
					    ("%s: resubmit read urb failed in channel:%d.\n"
					     "(%d)", __func__, channel, err);
					/* busy also in error unless we are killed */
					usb_mark_last_busy(port->serial->dev);
				}
			} else
				usb_mark_last_busy(port->serial->dev);
		}
		usb_autopm_put_interface_async(port->serial->interface);
	}
out:
	;			//hsictty_dbg("%s: exit\n", __func__);
}

#ifdef USE_READ_WORK
static void hsictty_read_work(struct work_struct *work)
{
	struct hsictty_port_private *portdata =
	    container_of(work, struct hsictty_port_private, hsictty_read_work);

	process_rx_data(portdata);
}
#else
static int rx_threadfn(void *x_)
{
	struct hsictty_port_private *portdata = x_;
	long rc = 0;
	struct sched_param param = {.sched_priority = 50 };

	sched_setscheduler(current, SCHED_FIFO, &param);

	while (!kthread_should_stop()) {
		if (portdata->thread_exit) {
			msleep(5);
			continue;
		}
		process_rx_data(portdata);
		rc = wait_for_completion_timeout(&portdata->rx_notifier,
						 5 * HZ);
		INIT_COMPLETION(portdata->rx_notifier);
	}

	return 0;
}
#endif

static void hsictty_read_callback(struct urb *urb)
{
	int endpoint;
	struct _HSICTTY_MSG *msg = NULL;
	int msg_index = -1;
	struct usb_serial_port *port;
	int status = urb->status;
	struct hsictty_port_private *portdata;
	unsigned long flags;
	int err = -1;
	u8 channel = 0;
	struct hsictty_intf_private *intfdata;
	static int error_times = 0;
	int error_times_limits = 50;

	hsictty_dbg("%s: %p\n", __func__, urb);

	endpoint = usb_pipeendpoint(urb->pipe);
	port = urb->context;
	portdata = usb_get_serial_port_data(port);
	intfdata = usb_get_serial_data(port->serial);
	channel = portdata->channel;

	if (status) {
		hsictty_dbg
		    ("%s: nonzero status: %d on channel:%d, endpoint %02x.\n",
		     __func__, status, channel, endpoint);
		if (intfdata->multi_channel_mode) {
			if (((status == -EPROTO) || (status == -EOVERFLOW))
			    && error_times++ < error_times_limits) {
				hsictty_error
				    ("%s: an halted error detected, will try again, status: %d on channel:%d, endpoint %02x.\n",
				     __func__, status, channel, endpoint);
				err = usb_submit_urb(urb, GFP_ATOMIC);
				if (err) {
					if (err != -EPERM) {
						printk(KERN_ERR
						       "%s: resubmit read urb failed in channel:%d.\n"
						       "(%d)", __func__,
						       channel, err);
						/* busy also in error unless we are killed */
						usb_mark_last_busy(port->
								   serial->dev);
					}
				} else {
					usb_mark_last_busy(port->serial->dev);
				}
			} else if (status == -EPROTO) {
				hsictty_error
				    ("%s: unrecorvery halted error detected, please check the hsic connection\n",
				     __func__);
			}
		}
	} else {
		error_times = 0;
		port = urb->context;
		portdata = usb_get_serial_port_data(port);

		if ((msg_index = get_read_msg_index(portdata)) < 0) {
			hsictty_error
			    ("%s: get read msg fail in channel:%d, endpoint:%d.\n",
			     __func__, channel, endpoint);
			err = usb_submit_urb(urb, GFP_ATOMIC);
			if (err) {
				if (err != -EPERM) {
					printk(KERN_ERR
					       "%s: resubmit read urb failed in channel:%d.\n"
					       "(%d)", __func__, channel, err);
					/* busy also in error unless we are killed */
					usb_mark_last_busy(port->serial->dev);
				}
			} else
				usb_mark_last_busy(port->serial->dev);
			return;
		}
		msg = &portdata->read_msg[msg_index];
#ifdef BACKUP_DATA_DUMP
		if (!dumped)
			backup_log(portdata->channel, 0,
					urb->transfer_buffer,
					urb->actual_length);
#endif
		INIT_LIST_HEAD(&msg->link);
		msg->urb = urb;
		wakeup_device(port, 0);
		wake_lock_timeout(&intfdata->rx_wakelock, HZ);
		spin_lock_irqsave(&portdata->pool_lock, flags);
		list_add_tail(&msg->link, &portdata->pool);
		spin_unlock_irqrestore(&portdata->pool_lock, flags);
#ifdef USE_READ_WORK
		queue_work(intfdata->hsictty_read_wq,
			   &(portdata->hsictty_read_work));
#else
		complete_all(&portdata->rx_notifier);
#endif
	}

}

int hsictty_write_room(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct hsictty_port_private *portdata;
	int i;
	int data_len = 0;
	struct urb *this_urb;
	int channel = -1;
	int ret = -EINVAL;

	portdata = usb_get_serial_port_data(port);
	down(&portdata->ch_sem_w);
	channel = tty->index;
	if (!channel_verify(port, channel)) {
		hsictty_error("%s: invalid channel (%d)!\n", __func__, channel);
		goto out;
	}

	for (i = 0; i < N_OUT_URB; i++) {
		this_urb = portdata->out_urbs[i];
		if (this_urb && !test_bit(i, &portdata->out_busy))
			data_len += OUT_BUFLEN;
	}

	hsictty_dbg("%s in channel(%d): %d\n", __func__, channel, data_len);
	ret = data_len;
out:
	up(&portdata->ch_sem_w);
	return ret;
}

int hsictty_chars_in_buffer(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct hsictty_port_private *portdata;
	int i;
	int data_len = 0;
	struct urb *this_urb;
	int channel = -1;
	int ret = 0;

	portdata = usb_get_serial_port_data(port);
	down(&portdata->ch_sem_w);
	channel = tty->index;
	if (!channel_verify(port, channel)) {
		hsictty_error("%s: invalid channel (%d)!\n", __func__, channel);
		goto out;
	}

	for (i = 0; i < N_OUT_URB; i++) {
		this_urb = portdata->out_urbs[i];
		/* FIXME: This locking is insufficient as this_urb may
		   go unused during the test */
		if (this_urb && test_bit(i, &portdata->out_busy))
			data_len += this_urb->transfer_buffer_length;
	}
	hsictty_dbg("%s in channel(%d): %d\n", __func__, channel, data_len);
	ret = data_len;
out:
	up(&portdata->ch_sem_w);
	return ret;
}

int hsictty_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct hsictty_port_private *portdata;
	struct hsictty_intf_private *intfdata;
	struct usb_serial *serial = port->serial;
	int i, err;
	struct urb *urb;
	int channel = -1;
	int ret = -EINVAL;

	portdata = usb_get_serial_port_data(port);
	intfdata = usb_get_serial_data(serial);

	down(&portdata->ch_sem_w);
	down(&portdata->ch_sem_r);
	channel = tty->index;
	if (!channel_verify(port, channel)) {
		hsictty_dbg("%s: invalid channel (%d)!\n", __func__, channel);
		goto out;
	}

	hsictty_dbg("%s: hsictty[%d] requested by process id %d (\"%s\")\n",
		    __func__, channel, current->tgid, current->comm);

	/*tty exclusive access and logcical channel handshake */
	if (test_and_set_bit(channel, &intfdata->channel_open_flag)) {
		ret = -EBUSY;
		hsictty_error("%s: channel:%d busy\n", __func__, channel);
		goto out;
	}

	if (intfdata->multi_channel_mode) {
		if (hsictty_handshake_setup(port, channel, 1) < 0) {
			ret = -EBUSY;
			hsictty_info
			    ("%s: Can't connect hsic logical channel %d\n",
			     __func__, channel);
			portdata->lch_opened = 0;
			clear_bit(channel, &intfdata->channel_open_flag);
			goto out;
		} else
			portdata->lch_opened = 1;
	}

	/* Start reading from the IN endpoint */
	for (i = 0; i < N_IN_URB; i++) {
		urb = portdata->in_urbs[i];
		if (!urb)
			continue;
		err = usb_submit_urb(urb, GFP_KERNEL);
		if (err) {
			hsictty_error
			    ("%s: submit urb %d failed (%d) in channel:%d\n",
			     __func__, i, err, channel);
		}
	}

	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);

	serial->interface->needs_remote_wakeup = 0;	//clear usb bus related feature due to using GPIO wakeup.
	spin_lock_irq(&intfdata->susp_lock);
	portdata->opened = 1;
	spin_unlock_irq(&intfdata->susp_lock);

	ret = 0;
	hsictty_dbg("hsic tty[%d] is opened by process id:%d (\"%s\")\n",
		     channel, current->tgid, current->comm);

	usb_autopm_put_interface(serial->interface);
out:
	up(&portdata->ch_sem_w);
	up(&portdata->ch_sem_r);
	return ret;
}

static int check_read_msg_all_free(struct usb_serial_port *port)
{
	int i = 0;
	unsigned long flags;
	struct hsictty_port_private *portdata = usb_get_serial_port_data(port);

	spin_lock_irqsave(&portdata->pool_lock, flags);
	for (i = 0; i < N_IN_URB; i++) {
		if (portdata->read_msg[i].urb != NULL) {
			spin_unlock_irqrestore(&portdata->pool_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&portdata->pool_lock, flags);
	return 1;
}

void hsictty_close(struct usb_serial_port *port)
{
	int i;
	struct tty_struct *tty = NULL;
	struct usb_serial *serial = port->serial;
	struct hsictty_port_private *portdata;
	struct hsictty_intf_private *intfdata = usb_get_serial_data(serial);
	int channel = -1;

	portdata = usb_get_serial_port_data(port);
	wakeup_device(port, 1);

	down(&portdata->ch_sem_w);
	down(&portdata->ch_sem_r);
	tty = tty_port_tty_get(&port->port);
	channel = portdata->channel;
	if (!tty)
		goto out;

	if (!channel_verify(port, channel))
		hsictty_dbg("%s: invalid channel (%d)!\n", __func__, channel);
	//goto out;

	if (intfdata->multi_channel_mode && portdata->lch_opened) {
		if (hsictty_handshake_setup(port, channel, 0) < 0)
			hsictty_info("%s: Can't connect server channel %d!\n",
				     __func__, channel);
		portdata->lch_opened = 0;
	}

	if (test_and_clear_bit(channel, &intfdata->channel_open_flag))
		clear_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	tty_port_tty_set(&port->port, NULL);

out:
	hsictty_dbg("hsic tty[%d] is closed by process id:%d (\"%s\")\n",
		     channel, current->tgid, current->comm);

	spin_lock_irq(&intfdata->susp_lock);
	portdata->opened = 0;
	spin_unlock_irq(&intfdata->susp_lock);

	if (serial->dev) {
		/* Stop reading/writing urbs */
		for (i = 0; i < N_IN_URB; i++)
			usb_kill_urb(portdata->in_urbs[i]);
		for (i = 0; i < N_OUT_URB; i++)
			usb_kill_urb(portdata->out_urbs[i]);

		serial->interface->needs_remote_wakeup = 0;
	}

	complete_all(&portdata->tx_notifier);
	complete_all(&portdata->rx_notifier);

	if (tty)
		tty_kref_put(tty);
	//delete to balance usb-serial

	up(&portdata->ch_sem_w);
	up(&portdata->ch_sem_r);

	/*safe close*/
	if (!check_read_msg_all_free(port)) {
		int timeout = 0, timeout_max = 200;

		hsictty_info("hsic tty[%d] waiting safe close\n", channel);
		complete_all(&portdata->rx_push_notifier);

		while (!check_read_msg_all_free(port) && ++timeout < timeout_max) {
			msleep(10);
		}
		if (timeout >= timeout_max)
			hsictty_error("hsic tty[%d] wait safe close error\n", channel);
		else
			hsictty_info("hsic tty[%d] wait safe close success\n", channel);
	}


	return;
}

static void stop_read_write_urbs(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct hsictty_port_private *portdata;

	/* Stop reading/writing urbs */
	for (i = 0; i < serial->num_ports; ++i) {
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);
		for (j = 0; j < N_IN_URB; j++)
			usb_kill_urb(portdata->in_urbs[j]);
		for (j = 0; j < N_OUT_URB; j++)
			usb_kill_urb(portdata->out_urbs[j]);
	}
}

void hsictty_disconnect(struct usb_serial *serial)
{
	int count = 0;
	int count_times = 100;

	/*TODO: NULL pointer bug fix tmp */
	//stop_read_write_urbs(serial);

	/*wait tty port totally closed */
	while ((atomic_read(&serial->kref.refcount) > 1) && count < count_times) {
		hsictty_dbg("%s: wait hsic tty port shutdown...\n", __func__);
		count++;
		msleep(100);
	}

	if (count >= count_times) {
		//hsictty_info("%s:1 ref_count:%d\n", __func__, atomic_read(&tty->kref.refcount));
		//hsictty_info("%s:2 ref_count:%d\n", __func__, atomic_read(&tty->kref.refcount));
		hsictty_info("%s: wait hsic tty port shutdown timeout!\n",
			     __func__);
	} else {
		//hsictty_info("%s:1 ref_count:%d\n", __func__, atomic_read(&tty->kref.refcount));
		//hsictty_info("%s:2 ref_count:%d\n", __func__, atomic_read(&tty->kref.refcount));
		hsictty_info("%s: all hsic tty port shutdown successfully!\n",
			     __func__);
	}
}

#ifdef CONFIG_PM
int hsictty_suspend(struct usb_serial *serial, pm_message_t message)
{
	struct hsictty_intf_private *intfdata = usb_get_serial_data(serial);
	int b;

	hsictty_dbg
	    ("%s entered,PM type:(%x:%s), pm cnt:%d, intfdata->in_flight:%d\n",
	     __func__, message.event,
	     (message.event & PM_EVENT_AUTO) ? "auto suspend" : "no auto suspend",
	     atomic_read(&serial->interface->dev.power.usage_count),
	     intfdata->in_flight);

	if (message.event & PM_EVENT_AUTO) {
		spin_lock_irq(&intfdata->susp_lock);
		b = intfdata->in_flight;
		spin_unlock_irq(&intfdata->susp_lock);

		if (b)
			return -EBUSY;
	}
	spin_lock_irq(&intfdata->susp_lock);
	intfdata->suspended = 1;
	spin_unlock_irq(&intfdata->susp_lock);
	stop_read_write_urbs(serial);

	return 0;
}

static void unbusy_queued_urb(struct urb *urb,
			      struct hsictty_port_private *portdata)
{
	int i;

	for (i = 0; i < N_OUT_URB; i++) {
		if (urb == portdata->out_urbs[i]) {
			clear_bit(i, &portdata->out_busy);
			break;
		}
	}
}

static void play_delayed(struct usb_serial_port *port)
{
	struct hsictty_intf_private *data;
	struct hsictty_port_private *portdata;
	struct urb *urb;
	int err;

	portdata = usb_get_serial_port_data(port);
	data = usb_get_serial_data(port->serial);
	while ((urb = usb_get_from_anchor(&portdata->delayed_urb))) {
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (!err) {
			hsictty_dbg
			    ("%s re-submit URB %p, write data may lost\n",
			     __func__, urb);
			data->in_flight++;
		} else {
			/* we have to throw away the rest */
			hsictty_error
			    ("%s re-submit flight URB error, write data may lost\n",
			     __func__);
			do {
				unbusy_queued_urb(urb, portdata);
				usb_autopm_put_interface_no_suspend
				    (port->serial->interface);
			} while ((urb =
				  usb_get_from_anchor(&portdata->delayed_urb)));
			break;
		}
	}
}

int hsictty_resume(struct usb_serial *serial)
{
	int i, j;
	struct usb_serial_port *port;
	struct hsictty_intf_private *intfdata = usb_get_serial_data(serial);
	struct hsictty_port_private *portdata;
	struct urb *urb;
	int err = 0;
	unsigned long flags;

	hsictty_dbg("%s entered", __func__);

	spin_lock_irq(&intfdata->susp_lock);
	intfdata->suspended = 0;
	spin_unlock_irq(&intfdata->susp_lock);

	/* get the interrupt URBs resubmitted unconditionally */
	for (i = 0; i < serial->num_ports; i++) {
		port = serial->port[i];
		if (!port->interrupt_in_urb) {
			hsictty_dbg("%s: No interrupt URB for port %d\n",
				    __func__, i);
			continue;
		}
		err = usb_submit_urb(port->interrupt_in_urb, GFP_NOIO);
		hsictty_dbg("Submitted interrupt URB for port %d (result %d)\n",
			    i, err);
		if (err < 0) {
			hsictty_error
			    ("%s: Error %d for interrupt URB of port%d\n",
			     __func__, err, i);
			goto err_out;
		}
	}

	for (i = 0; i < serial->num_ports; i++) {
		/* walk all ports */
		port = serial->port[i];
		portdata = usb_get_serial_port_data(port);

		/* skip closed ports */
		spin_lock_irq(&intfdata->susp_lock);
		if (!portdata->opened) {
			spin_unlock_irq(&intfdata->susp_lock);
			continue;
		}

		for (j = 0; j < N_IN_URB; j++) {
			spin_lock_irqsave(&portdata->pool_lock, flags);
			if (portdata->read_msg[j].urb != NULL) {
				spin_unlock_irqrestore(&portdata->pool_lock,
						       flags);
				continue;
			}
			urb = portdata->in_urbs[j];
			err = usb_submit_urb(urb, GFP_ATOMIC);
			spin_unlock_irqrestore(&portdata->pool_lock, flags);
			if (err < 0) {
				hsictty_error("%s: Error %d for bulk URB %d\n",
					      __func__, err, j);
				spin_unlock_irq(&intfdata->susp_lock);
				goto err_out;
			}
		}
		play_delayed(port);
		spin_unlock_irq(&intfdata->susp_lock);
	}
err_out:
	return err;
}

static int hsictty_reset_resume(struct usb_serial *serial)
{
	hsictty_info("%s entered", __func__);
	return hsictty_resume(serial);
}
#endif

static int hsictty_ioctl(struct tty_struct *tty, unsigned int cmd,
			 unsigned long arg)
{
	struct usb_serial_port *port = tty->driver_data;
	struct hsictty_port_private *portdata;
	struct hsictty_intf_private *intfdata;
	int channel = -1;
	int ret = -EINVAL;

	portdata = usb_get_serial_port_data(port);
	intfdata = usb_get_serial_data(port->serial);
	down(&portdata->ch_sem_w);
	down(&portdata->ch_sem_r);
	channel = tty->index;

	if (!intfdata->multi_channel_mode && channel != HSIC_DATA_START_CHANNEL) {
		hsictty_dbg("%s: invalid channel mode -- ch:%d!\n",
			    __func__, channel);
		goto out;
	}

	switch (cmd) {
	case HSIC_TTY_IOCTL_SET_MULTIMODE:
		{
			int mode = 1;
			int i = 0, j = 0;
			struct usb_serial_port *port_tmp;
			struct hsictty_port_private *portdata_tmp;
			struct urb *urb;
			if (copy_from_user(&mode, (int *)arg, sizeof(int))) {
				hsictty_error("%s: copy param failed\n",
					      __func__);
				ret = -EFAULT;
				goto out;
			}
			intfdata->multi_channel_mode = mode;
			/*set tx zlp support for multi-channel, clear tx zlp support for bootrom */
			for (i = 0; i < port->serial->num_ports; ++i) {
				port_tmp = port->serial->port[i];
				portdata_tmp =
				    usb_get_serial_port_data(port_tmp);
				for (j = 0; j < N_OUT_URB; j++) {
					urb = portdata_tmp->out_urbs[j];
					if (urb) {
						if (mode)
							urb->transfer_flags |=
							    URB_ZERO_PACKET;
						else
							urb->transfer_flags &=
							    ~URB_ZERO_PACKET;
					}
				}
			}
			hsictty_dbg
			    ("%s: set hsic tty multi-channel mode to [%d][%s] from user space!\n",
			     __func__, mode,
			     mode ? "multi-channel mode" : "single channel mode");
		}
		break;
	case HSIC_TTY_IOCTL_HSIC_RESET:
		{
			int i = 0;
			struct usb_serial_port *port_tmp;
			struct hsictty_port_private *portdata_tmp;
			//struct tty_struct *tty = NULL;
			pm_qos_update_request_timeout(&dl_kfc_num_qos, 3, 15 * 1000 * 1000);
			pm_qos_update_request_timeout(&dl_kfc_freq_qos, 1200000, 15 * 1000 * 1000);

			if (intfdata->support_pm) {
				pm_runtime_resume(&port->serial->dev->dev);
				usb_disable_autosuspend(port->serial->dev);
			}

			intfdata->multi_channel_mode = 0;
			for (i = 1; i < port->serial->num_ports; ++i) {
				port_tmp = port->serial->port[i];
				portdata_tmp =
				    usb_get_serial_port_data(port_tmp);
				portdata_tmp->opened = 0;
#ifndef USE_READ_WORK
				complete_all(&portdata_tmp->rx_notifier);
#endif
				complete_all(&portdata_tmp->tx_notifier);
				/* hangup here before diconncest marked
				 * may casue tty abnormally opened in serial core.
				tty = tty_port_tty_get(&port_tmp->port);
				if (tty) {
					tty_vhangup(tty);
					tty_kref_put(tty);
				} */
				clear_bit(i, &intfdata->channel_open_flag);
			}
#ifdef BACKUP_DATA_DUMP
			if (!dumped) {
				dumped = 1;
				backup_dump(HSIC_DATA_START_CHANNEL, 0);
				backup_dump(HSIC_DATA_START_CHANNEL, 1);
			}
#endif
			hsictty_info
			    ("%s: hsic tty reset triggerred from userspace!\n",
			     __func__);
		}
		break;
	case HSIC_TTY_IOCTL_HSIC_PM_ENABLE:
		if (intfdata->support_pm) {
			usb_enable_autosuspend(port->serial->dev);
			//pm_runtime_set_autosuspend_delay(&port->serial->dev->dev, 200);
			/* enable ehci root_hub runtime_pm */
			pm_runtime_allow(port->serial->dev->dev.parent->parent);
		}
		hsictty_info("%s: hsic pm enable from userspace!\n", __func__);
		break;
	default:
		hsictty_error("%s: illgal command !\n", __func__);
		ret = -ENOIOCTLCMD;
		goto out;

	}
	ret = 0;
out:
	up(&portdata->ch_sem_w);
	up(&portdata->ch_sem_r);

	return ret;
}

static void hsictty_throttle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct hsictty_port_private *portdata;

	portdata = usb_get_serial_port_data(port);
	INIT_COMPLETION(portdata->rx_push_notifier);
	hsictty_dbg("%s - port channel:%d", __func__, portdata->channel);
}

static void hsictty_unthrottle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct hsictty_port_private *portdata;

	portdata = usb_get_serial_port_data(port);
	complete_all(&portdata->rx_push_notifier);
	hsictty_dbg("%s - port channel:%d", __func__, portdata->channel);
}

static void hsictty_init_termios(struct tty_struct *tty)
{
	hsictty_dbg("%s: \n", __func__);
	tty->termios.c_iflag = 0;
	tty->termios.c_oflag = 0;
	tty->termios.c_lflag = 0;
}

#ifdef BACKUP_DATA_DUMP
struct backup_header {
	u32 magic;
	unsigned long long time;
	u32 length;
} __packed;

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
	int ch;
	int direction;
	for (direction = 0; direction < 2; direction++)
		for (ch = 0; ch < HSIC_CHANNEL_NUMS; ch++)
			skb_queue_head_init(&backup_queue[direction][ch]);
}

static void backup_log(u32 ch, int direction, char *data, u32 length)
{
	struct sk_buff_head *queue = &backup_queue[direction][ch];
	struct sk_buff *skb;
	struct backup_header *hdr;

	if (length == 0 || queue == NULL || data == NULL)
		return;

	if (ch != HSIC_DATA_START_CHANNEL)
		return;

	skb = dev_alloc_skb(sizeof(struct backup_header) + length);
	if (!skb) {
		hsictty_error(" ERR! dev_alloc_skb fail len(%u)\n", length);
		return;
	}
	hdr = (struct backup_header *)skb_put(skb,
			sizeof(struct backup_header));
	hdr->length = length;
	hdr->magic = 0x78563412;
	hdr->time = get_kernel_time();
	memcpy(skb_put(skb, length), data, length);
	while (skb_queue_len(queue) > 50) {
		struct sk_buff *skb = skb_dequeue(queue);
		dev_kfree_skb(skb);
	}
	skb_queue_tail(queue, skb);
}

struct file *backup_open_file(const char *path)
{
	struct file *fp;

	fp = filp_open(path, O_RDWR | O_CREAT | O_TRUNC, 0666);

	if (IS_ERR(fp))
		return NULL;

	return fp;
}

static void backup_dump(u32 ch, int direction)
{
	struct sk_buff *skb;
	char path[128];
	struct file *fp;
	mm_segment_t old_fs;
	struct sk_buff_head *queue = &backup_queue[direction][ch];

	sprintf(path, "/data/hsic_%s%02d.bin",
			direction ? "tx" : "rx", ch);

	hsictty_dbg(" Beging dump to file %s ...\n", path);
	old_fs = get_fs();
	set_fs(get_ds());

	fp = backup_open_file(path);
	if (!fp)
		goto exit;

	while ((skb = skb_dequeue(queue)) != NULL) {
		int ret = fp->f_op->write(fp, skb->data, skb->len, &fp->f_pos);
		dev_kfree_skb(skb);
		if (ret < 0)
			hsictty_info(" ERR! dump %s fail\n", path);
	}
	filp_close(fp, NULL);

	skb_queue_purge(queue);
exit:
	set_fs(old_fs);
	hsictty_dbg(" End dump to file %s\n", path);
}
#endif

static int hsic_s_probe(struct usb_serial *serial,
			const struct usb_device_id *id)
{
	struct hsictty_intf_private *spriv;

	spriv = kzalloc(sizeof(struct hsictty_intf_private), GFP_KERNEL);
	if (!spriv)
		return -ENOMEM;

	spin_lock_init(&spriv->susp_lock);
	sema_init(&spriv->handshake_sem, 1);

	spriv->channel_open_flag = 0;
	spriv->multi_channel_mode = 0;

	spriv->support_pm = 1;
	usb_disable_autosuspend(serial->dev);
	wake_lock_init(&spriv->tx_wakelock, WAKE_LOCK_SUSPEND, "hsic_tx");
	wake_lock_init(&spriv->rx_wakelock, WAKE_LOCK_SUSPEND, "hsic_rx");

#ifdef USE_READ_WORK
	spriv->hsictty_read_wq = create_workqueue("hsic tty task");
	if (!spriv->hsictty_read_wq) {
		hsictty_error("%s: can't create workqueue\n", __func__);
		return -EINVAL;
	}
#endif

#ifdef BACKUP_DATA_DUMP
	backup_queue_init();
	dumped = 0;
#endif
	usb_set_serial_data(serial, spriv);

	return 0;
}

static int hsictty_port_probe(struct usb_serial_port *port)
{
	int i;
	struct urb *urb;
	struct hsictty_port_private *portdata;
	struct usb_serial *serial = port->serial;
#ifndef USE_READ_WORK
	char task_name[50] = { 0 };
#endif

	portdata = kzalloc(sizeof(*portdata), GFP_KERNEL);
	if (!portdata) {
		hsictty_error("%s: alloc mem failed\n", __func__);
		return -ENOMEM;
	}
	portdata->opened = 0;
	portdata->lch_opened = 0;
	portdata->channel = port->number;

	INIT_LIST_HEAD(&portdata->pool);
	spin_lock_init(&portdata->pool_lock);
	sema_init(&portdata->ch_sem_w, 1);
	sema_init(&portdata->ch_sem_r, 1);
	init_completion(&portdata->tx_notifier);
	init_completion(&portdata->rx_push_notifier);
	init_usb_anchor(&portdata->delayed_urb);
#ifdef USE_READ_WORK
	INIT_WORK(&portdata->hsictty_read_work, hsictty_read_work);
#else
	sprintf(task_name, "hsictty_rx_task%d", portdata->channel);
	portdata->thread_exit = 0;
	portdata->rx_task = kthread_create(rx_threadfn, portdata, task_name);
	init_completion(&portdata->rx_notifier);
	if (portdata->rx_task)
		wake_up_process(portdata->rx_task);
#endif

	for (i = 0; i < ARRAY_SIZE(portdata->in_urbs); ++i) {
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (urb == NULL) {
			hsictty_dbg("%s: in urb alloc failed.\n", __func__);
			goto error;
		}
		portdata->in_urbs[i] = urb;

		portdata->in_buffer[i] =
		    usb_alloc_coherent(serial->dev, IN_BUFLEN, GFP_KERNEL,
				       &urb->transfer_dma);
		if (!portdata->in_buffer[i]) {
			hsictty_dbg
			    ("%s: in urb dma buffer alloc failed.\n", __func__);
			goto error;
		}

		/* Fill URB using supplied data. */
		usb_fill_bulk_urb(urb, serial->dev,
				  usb_sndbulkpipe(serial->dev,
						  port->bulk_in_endpointAddress)
				  | USB_DIR_IN, portdata->in_buffer[i],
				  IN_BUFLEN, hsictty_read_callback, port);
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}

	for (i = 0; i < ARRAY_SIZE(portdata->out_urbs); ++i) {
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (urb == NULL) {
			hsictty_dbg("%s: in urb alloc failed.\n", __func__);
			goto error;
		}
		portdata->out_urbs[i] = urb;

		portdata->out_buffer[i] =
		    usb_alloc_coherent(serial->dev, OUT_BUFLEN, GFP_KERNEL,
				       &urb->transfer_dma);
		if (!portdata->out_buffer[i]) {
			hsictty_dbg
			    ("%s: in urb dma buffer alloc failed.\n", __func__);
			goto error;
		}
		/* Fill URB using supplied data. */
		usb_fill_bulk_urb(urb, serial->dev,
				  usb_sndbulkpipe(serial->dev,
						  port->
						  bulk_out_endpointAddress)
				  | USB_DIR_OUT, portdata->out_buffer[i],
				  OUT_BUFLEN, hsictty_write_callback, port);
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}

	usb_set_serial_port_data(port, portdata);

	return 0;

error:
	for (i = 0; i < ARRAY_SIZE(portdata->out_urbs); ++i) {
		urb = portdata->out_urbs[i];
		if (!urb)
			continue;
		if (portdata->out_buffer[i])
			usb_free_coherent(serial->dev, OUT_BUFLEN,
					  portdata->out_buffer[i],
					  urb->transfer_dma);
		usb_free_urb(urb);
	}

	for (i = 0; i < ARRAY_SIZE(portdata->in_urbs); ++i) {
		urb = portdata->in_urbs[i];
		if (!urb)
			continue;
		if (portdata->in_buffer[i])
			usb_free_coherent(serial->dev, IN_BUFLEN,
					  portdata->in_buffer[i],
					  urb->transfer_dma);
		usb_free_urb(urb);
	}
#ifndef USE_READ_WORK
	if (portdata->rx_task) {
		portdata->thread_exit = 1;
		complete_all(&portdata->rx_notifier);
		kthread_stop(portdata->rx_task);
		portdata->rx_task = NULL;
	}
#endif


	kfree(portdata);

	return -EINVAL;
}

static int hsictty_port_remove(struct usb_serial_port *port)
{
	int i;
	struct urb *urb;
	struct hsictty_port_private *portdata;
	struct usb_serial *serial = port->serial;

	portdata = usb_get_serial_port_data(port);

	for (i = 0; i < ARRAY_SIZE(portdata->out_urbs); ++i) {
		urb = portdata->out_urbs[i];
		usb_free_coherent(serial->dev, OUT_BUFLEN,
				portdata->out_buffer[i], urb->transfer_dma);
		usb_free_urb(urb);
	}

	for (i = 0; i < ARRAY_SIZE(portdata->in_urbs); ++i) {
		urb = portdata->in_urbs[i];
		usb_free_coherent(serial->dev, IN_BUFLEN,
				portdata->in_buffer[i], urb->transfer_dma);
		usb_free_urb(urb);
	}
#ifndef USE_READ_WORK
	if (portdata->rx_task) {
		portdata->thread_exit = 1;
		complete_all(&portdata->rx_notifier);
		kthread_stop(portdata->rx_task);
		portdata->rx_task = NULL;
	}
#endif
	kfree(portdata);

	return 0;
}

static int hsictty_startup(struct usb_serial *serial)
{
	hsictty_dbg("%s\n", __func__);

	return 0;
}

static void hsictty_release(struct usb_serial *serial)
{
	struct hsictty_intf_private *spriv = usb_get_serial_data(serial);

	hsictty_dbg("%s\n", __func__);

#ifdef USE_READ_WORK
	if (spriv->hsictty_read_wq)
		destroy_workqueue(spriv->hsictty_read_wq);
#endif
	wake_lock_destroy(&spriv->tx_wakelock);
	wake_lock_destroy(&spriv->rx_wakelock);

	kfree(usb_get_serial_data(serial));
}

static const struct usb_device_id nezha_ids[] = {
	{USB_DEVICE(NEZHA_MODEM_VENDOR_ID, NEZHA_MODEM_PRODUCT_ID)},
	{}
};

MODULE_DEVICE_TABLE(usb, nezha_ids);

static struct usb_serial_driver hsic_s_driver = {
	.driver = {
		   .owner =	THIS_MODULE,
		   .name =	"nezha_hsic",
		   },
	.description =		"Nezha modem",
	.id_table =		nezha_ids,
	.num_ports =		HSIC_CHANNEL_NUMS,

	.probe =		hsic_s_probe,
	.attach =		hsictty_startup,
	.disconnect =		hsictty_disconnect,
	.release =		hsictty_release,
	.port_probe =		hsictty_port_probe,
	.port_remove =		hsictty_port_remove,

	.suspend =		hsictty_suspend,
	.resume =		hsictty_resume,
	.reset_resume =		hsictty_reset_resume,

	.open =			hsictty_open,
	.close =		hsictty_close,
	.write =		hsictty_write,
	.write_room =		hsictty_write_room,
	.chars_in_buffer =	hsictty_chars_in_buffer,
	.throttle =		hsictty_throttle,
	.unthrottle =		hsictty_unthrottle,
	.ioctl =		hsictty_ioctl,
	.init_termios =		hsictty_init_termios,

	.read_bulk_callback =	hsictty_read_callback,
	.write_bulk_callback =	hsictty_write_callback,
};

static struct usb_serial_driver *const hsic_s_drivers[] = {
	&hsic_s_driver, NULL
};

module_usb_serial_driver(hsic_s_drivers, nezha_ids);

static int __init modem_kfc_qos_init(void)
{
	pm_qos_add_request(&dl_kfc_num_qos, PM_QOS_KFC_NUM_MIN, 0);
	pm_qos_add_request(&dl_kfc_freq_qos, PM_QOS_KFC_FREQ_MIN, 0);

	return 0;
}

device_initcall(modem_kfc_qos_init);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
