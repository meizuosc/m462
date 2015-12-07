/***********************************************************
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: - p61.c
* Description: Source file for PN65T spi driver.
				
** Version: 1.0
** Date : 2014/03/18	
** Author: yuyi@Dep.Group.Module
** 

** --------------------------- Revision History: --------------------------------
** <author>		                      <data> 	<version >  <desc>
** ------------------------------------------------------------------------------
** Yuyi@Driver.NFC  2014/04/03   1.1	    modify for debug spi
****************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/timer.h>

#include <linux/of.h>
#include <linux/of_gpio.h>

#define DBG_MODULE 0
//#define TIMER_ENABLE 
#define TIMER_ENABLE 0
#define IRQ_ENABLE 1
#if DBG_MODULE
#define NFC_DBG_MSG(msg...) printk(KERN_INFO "[NFC PN61] :  " msg);
#else
#define NFC_DBG_MSG(msg...)
#endif

#define NFC_ERR_MSG(msg...) printk(KERN_ERR "[NFC PN61] : " msg );

#define P61_MAGIC 0xE8
/*
 * PN544 power control via ioctl
 * PN544_SET_PWR(0): power off
 * PN544_SET_PWR(1): power on
 * PN544_SET_PWR(2): reset and power on with firmware download enabled */
#define P61_SET_PWR _IOW(P61_MAGIC, 0x01, unsigned int)

#define MAX_BUFFER_SIZE 4096	//4k
static ssize_t sendFrame(struct file *filp, const char __user data[], char mode,
		  int count);
static ssize_t sendChainedFrame(struct file *filp, const unsigned char __user data[],
			 int count);
static unsigned char *apduBuffer;
static unsigned char *gRecvBuff;
static unsigned char *checksum;
static int apduBufferidx = 0;
const char PH_SCAL_T1_CHAINING = 0x20;
const char PH_SCAL_T1_SINGLE_FRAME = 0x00;
const char PH_SCAL_T1_R_BLOCK = 0x80;
const char PH_SCAL_T1_S_BLOCK = 0xC0;
const char PH_SCAL_T1_HEADER_SIZE_NO_NAD = 0x02;
static unsigned char seqCounterCard = 0;
static unsigned char seqCounterTerm = 1;
static short ifs = 254;
static short headerSize = 3;
static unsigned char sof = 0xA5;
static unsigned char csSize = 1;
const char C_TRANSMIT_NO_STOP_CONDITION = 0x01;
const char C_TRANSMIT_NO_START_CONDITION = 0x02;
const char C_TRANSMIT_NORMAL_SPI_OPERATION = 0x04;

typedef struct respData {
	unsigned char *data;
	int len;
} respData_t;

static respData_t *gStRecvData = NULL;
static unsigned char *gSendframe = NULL;
static unsigned char *gDataPackage = NULL;
static unsigned char *data1 = NULL;

#define MEM_CHUNK_SIZE (256)
static unsigned char *lastFrame = NULL;
static int lastFrameLen;
static void init(void);
static void setAddress(short address);
static void setBitrate(short bitrate);
static int nativeSetAddress(short address);
static int nativeSetBitrate(short bitrate);
static unsigned char helperComputeLRC(unsigned char data[], int offset, int length);
static void receiveAcknowledge(struct file *filp);
static void receiveAndCheckChecksum(struct file *filp, short rPcb, short rLen,
			     unsigned char data[], int len);
static respData_t *receiveHeader(struct file *filp);
static respData_t *receiveFrame(struct file *filp, short rPcb, short rLen);
static int send(struct file *filp, unsigned char **data, unsigned char mode, int len);
static int receive(struct file *filp, unsigned char **data, int len,
	    unsigned char mode);
static respData_t *receiveChainedFrame(struct file *filp, short rPcb, short rLen);
static void sendAcknowledge(struct file *filp);
static ssize_t p61_dev_write(struct file *filp, const char __user * buf,
			     size_t count, loff_t * offset);
static ssize_t p61_dev_read(struct file *filp, char __user * buf,
			    size_t count, loff_t * offset);
static ssize_t p61_dev_receiveData(struct file *filp, char __user * buf,
				   size_t count, loff_t * offset);
/*static ssize_t p61_dev_sendData(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset);
	*/
static respData_t *p61_dev_receiveData_internal(struct file *filp);

poll_table *wait;

struct p61_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct spi_device *spi;
	struct miscdevice p61_device;
	char *rp, *wp;
	unsigned int ven_gpio;
	unsigned int irq_gpio;	// IRQ will be used later (P61 will interrupt DH for any ntf)
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	wait_queue_head_t *inq, *outq;
	int buffersize;
	struct spi_message msg;
	struct spi_transfer transfer;
	bool firm_gpio;
};

struct p61_control {
	struct spi_message msg;
	struct spi_transfer transfer;
	unsigned char *tx_buff;
	unsigned char *rx_buff;
};

static int timer_expired = 0;
#ifdef TIMER_ENABLE
static struct timer_list recovery_timer;
static int timer_started = 0;
static void p61_disable_irq(struct p61_dev *p61_dev);

static void my_timer_callback(unsigned long data)
{
	timer_expired = 1;
}

static int start_timer(void)
{
	int ret;
	setup_timer(&recovery_timer, my_timer_callback, 0);
	ret = mod_timer(&recovery_timer, jiffies + msecs_to_jiffies(2000));
	if (ret) {
		printk(KERN_INFO "Error in mod_timer\n");
	} else {
		timer_started = 1;
	}
	return 0;
}

static void cleanup_timer(void)
{
	int ret;

	ret = del_timer(&recovery_timer);
	return;
}
#endif
static int p61_dev_open(struct inode *inode, struct file *filp)
{
	struct p61_dev *p61_dev = container_of(filp->private_data,
					       struct p61_dev,
					       p61_device);
	filp->private_data = p61_dev;

	gRecvBuff = (unsigned char *)kmalloc(300, GFP_KERNEL);
	if (!gRecvBuff)
		goto out;
	gStRecvData = (respData_t *) kmalloc(sizeof(respData_t), GFP_KERNEL);
	if (!gStRecvData)
		goto out;
	checksum = (unsigned char *)kmalloc(csSize, GFP_KERNEL);
	if (!checksum)
		goto out;
	gSendframe = (unsigned char *)kmalloc(300, GFP_KERNEL);
	if (!gSendframe)
		goto out;
	gDataPackage = (unsigned char *)kmalloc(300, GFP_KERNEL);
	if (!gDataPackage)
		goto out;
	apduBuffer = (unsigned char *)kmalloc(256, GFP_KERNEL);
	if (!apduBuffer)
		goto out;
	data1 = (unsigned char *)kmalloc(1, GFP_KERNEL);
	if (!data1)
		goto out;
	NFC_DBG_MSG("%s :  Major No: %d, Minor No: %d\n", __func__,
		    imajor(inode), iminor(inode));

	return 0;
out:
	if (apduBuffer != NULL)
		kfree(apduBuffer);
	if (gDataPackage != NULL)
		kfree(gDataPackage);
	if (gSendframe != NULL)
		kfree(gSendframe);
	if (checksum != NULL)
		kfree(checksum);
	if (gStRecvData != NULL)
		kfree(gStRecvData);
	if (gRecvBuff != NULL)
		kfree(gRecvBuff);
	
	return -ENODEV;
}

static int p61_dev_close(struct inode *inode, struct file *filp)
{
	if (checksum != NULL)
		kfree(checksum);
	if (gRecvBuff != NULL)
		kfree(gRecvBuff);
	if (gStRecvData != NULL)
		kfree(gStRecvData);
	if (gSendframe != NULL)
		kfree(gSendframe);
	if (gDataPackage != NULL)
		kfree(gDataPackage);
	if (apduBuffer != NULL)
		kfree(apduBuffer);
	if (data1 != NULL)
		kfree(data1);
	return 0;
}

void SpiReadSingle(struct spi_device *spi, unsigned char *pTxbuf,
		   unsigned char *pbuf, unsigned int length)
{
	while (length) {
		length--;
		spi_write_then_read(spi, pTxbuf, 1, pbuf, 1);
		pbuf++;
		pTxbuf++;
	}
}

//static unsigned int  p61_dev_poll(struct file *filp, char *buf,
//              size_t count, loff_t *offset)

static unsigned int p61_dev_poll(struct file *filp,
				 struct poll_table_struct *buf)
{
	int mask = -1;
	int left = 0;
	struct p61_dev *p61_dev = filp->private_data;

	NFC_DBG_MSG("p61_dev_poll called  \n");
	left =
	    (p61_dev->rp + p61_dev->buffersize -
	     p61_dev->wp) % (p61_dev->buffersize);
	poll_wait(filp, p61_dev->inq, wait);

	if (p61_dev->rp != p61_dev->wp)
		mask |= POLLIN | POLLRDNORM;
	if (left != 1)
		mask |= POLLOUT | POLLRDNORM;
	return mask;
}

/**
     * Entry point function for receiving data. Based on the PCB byte this function
     * either receives a single frame or a chained frame.
     *
     */
/*static ssize_t p61_dev_receiveData(struct file *filp, char *buf,
		size_t count, loff_t *offset)
		*/
static ssize_t p61_dev_receiveData(struct file *filp, char __user * buf,
				   size_t count, loff_t * offset)
{
	respData_t *rsp = p61_dev_receiveData_internal(filp);
	//if(count >= rsp->len)
	//{
	count = rsp->len;
	//}
	if (0 < count) {
		//memcpy(buf,rsp->data,count);
		if (copy_to_user(buf, rsp->data, count)) {
			NFC_DBG_MSG("%s : failed to copy to user space\n",
				    __func__);
			return -EFAULT;
		} else {
			//Nothing
		}

	}

	return count;
}

static respData_t *p61_dev_receiveData_internal(struct file *filp)
{
	short rPcb = 0;
	short rLen = 0;
	respData_t *header = NULL;
	respData_t *respData = NULL;
	unsigned char *wtx = NULL;
	unsigned char *data = NULL;
	//unsigned char *data1=NULL;
	int len = 0;
	int len1 = 0;
	int stat_timer;
Start:
	NFC_DBG_MSG("receiveData -Enter\n");

	// receive the T=1 header
	header = (respData_t *) receiveHeader(filp);
	if (header == NULL) {
		NFC_ERR_MSG("ERROR:Failed to receive header data\n");
		return NULL;
	}
	rPcb = header->data[0];
	rLen = (short)(header->data[1] & 0xFF);
	NFC_DBG_MSG("receive header data rPcb = 0x%x , rLen = %d\n",
		    rPcb, rLen);

#if 1

	//check the header if wtx is requested
	if ((rPcb & PH_SCAL_T1_S_BLOCK) == PH_SCAL_T1_S_BLOCK) {
		NFC_DBG_MSG("receiveDatav - WTX requested\n");
		data = gRecvBuff;
		len = 1;
		NFC_DBG_MSG("receiveDatav - WTX1 requested\n");
		receive(filp, &data, len,
			C_TRANSMIT_NO_STOP_CONDITION |
			C_TRANSMIT_NO_START_CONDITION);
		NFC_DBG_MSG("receiveDatav - WTX2 requested\n");
		receiveAndCheckChecksum(filp, rPcb, rLen, data, len);
		NFC_DBG_MSG("receiveDatav - WTX3 requested\n");
		NFC_DBG_MSG("value is %x %x", data[0], data[1]);
		memset(gRecvBuff, 0, 300);
		wtx = gRecvBuff;
		wtx[0] = 0x00;
		wtx[1] = 0xE3;
		wtx[2] = 0x01;
		wtx[3] = 0x01;
		wtx[4] = 0xE3;
		len1 = 5;
		udelay(1000);
		send(filp, &wtx, C_TRANSMIT_NORMAL_SPI_OPERATION, len1);
		udelay(1000);

		//gStRecvData->len = 5;
		//memcpy(gStRecvData->data, wtx, 5);;
#ifdef TIMER_ENABLE
		stat_timer = start_timer();
#endif
		//return gStRecvData;
		goto Start;
	}
	//check the header if retransmit is requested
	if ((rPcb & PH_SCAL_T1_R_BLOCK) == PH_SCAL_T1_R_BLOCK) {
		memset(data1, 0, 1);
		len1 = 1;
		receiveAndCheckChecksum(filp, rPcb, rLen, data1, len1);
		udelay(1000);
		send(filp, &lastFrame, C_TRANSMIT_NORMAL_SPI_OPERATION,
		     lastFrameLen);
		udelay(1000);
		goto Start;
//              return (ssize_t)p61_dev_receiveData_internal(filp);
	}
	//check the PCB byte and receive the rest of the frame
	if ((rPcb & PH_SCAL_T1_CHAINING) == PH_SCAL_T1_CHAINING) {
		NFC_DBG_MSG("Chained Frame Requested\n");

		return receiveChainedFrame(filp, rPcb, rLen);

	} else {
		NFC_DBG_MSG("receiveFrame Requested\n");
		respData = receiveFrame(filp, rPcb, rLen);
		NFC_DBG_MSG("***************** 0x%x \n", respData->data[0]);
		return respData;
	}
#endif
	return NULL;
}

/**
    * This function is used to receive a single T=1 frame
    *
    * @param rPcb
    *            PCB field of the current frame
    * @param rLen
    *            LEN field of the current frame
    * @param filp
    * 			 File pointer
    */

static respData_t *receiveFrame(struct file * filp, short rPcb, short rLen)
{

	int status = 0;
	respData_t *respData = NULL;
	NFC_DBG_MSG("receiveFrame -Enter\n");
	respData = gStRecvData;
	respData->data = gRecvBuff;
	respData->len = rLen;
	// modify the card send sequence counter
	seqCounterCard = (seqCounterCard ^ 1);

	// receive the DATA field and check the checksum
	status =
	    receive(filp, &(respData->data), respData->len,
		    C_TRANSMIT_NO_STOP_CONDITION |
		    C_TRANSMIT_NO_START_CONDITION);

	receiveAndCheckChecksum(filp, rPcb, rLen, respData->data,
				respData->len);

	NFC_DBG_MSG("receiveFrame -Exit\n");

	return respData;
}

/**
     * This function is used to receive a chained frame.
     *
     * @param rPcb
     *            PCB field of the current frame
     * @param rLen
     *            LEN field of the current frame
     * @param filp
     *            File pointer
     */

static respData_t *receiveChainedFrame(struct file * filp, short rPcb, short rLen)
{
	respData_t *data_rec = NULL;
	respData_t *header = NULL;
	respData_t *respData = NULL;
	respData_t *apdbuff = NULL;
	NFC_DBG_MSG("receiveChainedFrame -Enter\n");
	// receive a chained frame as long as chaining is indicated in the PCB
	do {
		// receive the DATA field of the current frame
		NFC_DBG_MSG("p61_dev_read - test4 count [0x%x] \n",
			    rLen);
		data_rec = receiveFrame(filp, rPcb, rLen);
		// write it into an apduBuffer memory
		memcpy((apduBuffer + apduBufferidx), data_rec->data,
		       data_rec->len);

		//update the index to next free slot
		apduBufferidx += data_rec->len;

		// send the acknowledge for the current frame
		udelay(1000);
		sendAcknowledge(filp);
		udelay(1000);
		// receive the header of the next frame
		header = receiveHeader(filp);

		rPcb = header->data[0];
		rLen = (header->data[1] & 0xFF);

	} while ((rPcb & PH_SCAL_T1_CHAINING) == PH_SCAL_T1_CHAINING);

	// receive the DATA field of the last frame

	respData = receiveFrame(filp, rPcb, rLen);
	memcpy(apduBuffer + apduBufferidx, respData->data, respData->len);
	//update the index to next free slot
	apduBufferidx += respData->len;

	// return the entire received apdu
	apdbuff = (respData_t *) kmalloc(sizeof(respData_t), GFP_KERNEL);
	if (apdbuff == NULL) {
		NFC_ERR_MSG("receiveChainedFrame 2-KMALLOC FAILED!!!\n");
		return NULL;
	}

	apdbuff->data = (unsigned char *)kmalloc(apduBufferidx, GFP_KERNEL);
	if (apdbuff->data == NULL) {
		NFC_ERR_MSG("receiveChainedFrame 3-KMALLOC FAILED!!!\n");
		return NULL;
	}
	memcpy(apdbuff->data, apduBuffer, apduBufferidx);
	apdbuff->len = apduBufferidx;

	NFC_DBG_MSG("receiveChainedFrame -Exit\n");
	return apdbuff;
}

/**
    * This function is used to send an acknowledge for an received I frame
    * in chaining mode.
    *
    */
static void sendAcknowledge(struct file *filp)
{
	unsigned char *ack = NULL;
	NFC_DBG_MSG("sendAcknowledge - Enter\n");
	ack = gSendframe;
	// prepare the acknowledge and send it
	NFC_DBG_MSG("seqCounterCard value is [0x%x]\n", seqCounterCard);
	ack[0] = 0x00;
	ack[1] =
	    (unsigned char)(PH_SCAL_T1_R_BLOCK |
			    (unsigned char)(seqCounterCard << 4));
	ack[2] = 0x00;
	ack[3] = helperComputeLRC(ack, 0, (sizeof(ack) / sizeof(ack[0])) - 2);

	send(filp, &ack, C_TRANSMIT_NORMAL_SPI_OPERATION,
	     sizeof(ack) / sizeof(ack[0]));

	NFC_DBG_MSG("sendAcknowledge - Exit\n");

}

/**
    * This function sends either a chained frame or a single T=1 frame
    *
    * @param buf
    *            the data to be send
    *
    */
/*static ssize_t p61_dev_sendData(struct file *filp, unsigned char *buf,
		size_t count, loff_t *offset)
		*/
static ssize_t p61_dev_sendData(struct file *filp, const char __user * buf,
				size_t count, loff_t * offset)
{
	int ret = -1;
	init();
	NFC_DBG_MSG("p61_dev_sendData %d - Enter \n", count);
	if (count <= ifs) {
		ret = sendFrame(filp, buf, PH_SCAL_T1_SINGLE_FRAME, count);
		NFC_DBG_MSG("Vaue of count_status is %d \n", ret);
	} else {
		//return sendChainedFrame(data);
		ret = sendChainedFrame(filp, buf, count);
	}
	NFC_DBG_MSG("p61_dev_sendData: count_status is %d \n", ret);
	return ret;
}

static long p61_dev_ioctl(struct file *filp, unsigned int cmd,
			  unsigned long arg)
{
	int ret;
	struct p61_dev *p61_dev = NULL;
	uint8_t buf[100];
	//NFC_DBG_MSG(KERN_ALERT "p61_dev_ioctl-Enter %x arg = 0x%x\n",cmd, arg);
	p61_dev = filp->private_data;
	NFC_DBG_MSG("yuyi,p61_dev_ioctl enter\n");

	switch (cmd) {
	case P61_SET_PWR:
		if (arg == 2) {
			NFC_DBG_MSG("yuyi,p61_dev_ioctl   download firmware \n");
			/* power on with firmware download (requires hw reset) */
			gpio_set_value(p61_dev->ven_gpio, 1);
			NFC_DBG_MSG("p61_dev_ioctl-1\n");
			msleep(20);
			gpio_set_value(p61_dev->ven_gpio, 0);
			NFC_DBG_MSG("p61_dev_ioctl-0\n");
			msleep(50);
			ret = spi_read(p61_dev->spi, (void *)buf, sizeof(buf));
			msleep(50);
			gpio_set_value(p61_dev->ven_gpio, 1);
			NFC_DBG_MSG("p61_dev_ioctl-1 \n");
			msleep(20);

		} else if (arg == 1) {
			NFC_DBG_MSG("yuyi,p61_dev_ioctl   power on \n");
			/* power on */
			NFC_DBG_MSG("p61_dev_ioctl-1 (arg = 1)\n");
			gpio_set_value(p61_dev->ven_gpio, 1);

		} else if (arg == 0) {
			NFC_DBG_MSG("yuyi,p61_dev_ioctl   power off \n");
			/* power off */
			NFC_DBG_MSG("p61_dev_ioctl-0 (arg = 0)\n");
			gpio_set_value(p61_dev->ven_gpio, 0);
			udelay(100);
		} else {
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
     * This function is used to send a chained frame.
     *
     * @param data
     *            the data to be send
     */
static int sendChainedFrame(struct file *filp, const unsigned char data[], int len)
{

	int count_status = 0;
	int length = len;
	int offset = 0;
	int ret = 0;
	unsigned char *lastDataPackage = NULL;
	unsigned char *dataPackage = NULL;
	NFC_DBG_MSG("sendChainedFrame - Enter\n");
	dataPackage = gDataPackage;
	do {
		NFC_DBG_MSG("sendChainedFrame \n");
		// send a chained frame and receive the acknowledge
		memcpy(&dataPackage[0], &data[offset], ifs);

		count_status =
		    sendFrame(filp, dataPackage, PH_SCAL_T1_CHAINING, ifs);
		if (count_status == 0) {
			NFC_ERR_MSG("ERROR1: Failed to send Frame\n");
			return -1;
		}
		receiveAcknowledge(filp);
		udelay(1000);
		length = length - ifs;
		offset = offset + ifs;
		ret += count_status;
	} while (length > ifs);

#if 1
	// send the last frame
	lastDataPackage = gDataPackage;
	memcpy(&lastDataPackage[0], &data[offset], length);

	count_status =
	    sendFrame(filp, lastDataPackage, PH_SCAL_T1_SINGLE_FRAME, length);

	if (count_status == 0)
	{
		NFC_ERR_MSG("ERROR2:Failed to send Frame\n");
		return -1;

	}
#endif
	NFC_DBG_MSG("sendChainedFrame - Exit\n");
	ret += count_status;
	return ret;
}

/**
     * This function is used to receive an Acknowledge of an I frame
     *
     */
static void receiveAcknowledge(struct file *filp)
{
	respData_t *header = NULL;
	short rPcb = 0;
	short rLen = 0;
	int len = 1;
	unsigned char *cs = NULL;
	NFC_DBG_MSG("receiveAcknowledge - Enter\n");
	cs = gRecvBuff;
	header = (respData_t *) receiveHeader(filp);
	rPcb = (header->data[0] & 0xFF);
	rLen = (header->data[1] & 0xFF);
	receiveAndCheckChecksum(filp, rPcb, rLen, cs, len);
	NFC_DBG_MSG("receiveAcknowledge - Exit\n");
}

/**
 * This function is used to receive the header of the next T=1 frame.
 * If no data is available the function polls the data line as long as it receives the
 * start of the header and then receives the entire header.
 *
 */
static respData_t *receiveHeader(struct file *filp)
{
	int count_status = 0;
	//unsigned char *ready=NULL;
	respData_t *header = NULL;
	unsigned char *r_frame = NULL;
	int len = 1;
	NFC_DBG_MSG("receiveHeader - Enter\n");
	header = gStRecvData;
	header->data = gRecvBuff;
	header->len = PH_SCAL_T1_HEADER_SIZE_NO_NAD;
	count_status =
	    receive(filp, &gRecvBuff, len, C_TRANSMIT_NO_STOP_CONDITION);
	NFC_DBG_MSG("sof is :0x%x\n", gRecvBuff[0]);
	// check if we received ready
#ifdef TIMER_ENABLE
again:
#endif

	while (gRecvBuff[0] != sof) {
		NFC_DBG_MSG("SOF not found\n");
		// receive one byte and keep SS line low
		count_status =
		    receive(filp, &gRecvBuff, len,
			    C_TRANSMIT_NO_STOP_CONDITION |
			    C_TRANSMIT_NO_START_CONDITION);
		NFC_DBG_MSG("in While SOF is : 0x%x \n",
			    gRecvBuff[0]);
	}
#ifdef TIMER_ENABLE
	if (timer_started) {
		timer_started = 0;
		cleanup_timer();
	}
	if (timer_expired == 1) {
		memset(gSendframe, 0, 300);
		r_frame = gSendframe;
		r_frame[0] = 0x00;
		r_frame[1] = 0x00;
		r_frame[2] = 0x00;
		r_frame[3] = 0x00;
		timer_started = 0;
		timer_expired = 0;
		cleanup_timer();
		// printk(KERN_INFO "************* Sending R Frame \n");
		send(filp, &r_frame, C_TRANSMIT_NORMAL_SPI_OPERATION, 4);
		goto again;

	}
#endif
	NFC_DBG_MSG("SOF FOUND\n");
	// we received ready byte, so we can receive the rest of the header and keep SS line low
	count_status =
	    receive(filp, &(header->data), header->len,
		    C_TRANSMIT_NO_STOP_CONDITION |
		    C_TRANSMIT_NO_START_CONDITION);
	NFC_DBG_MSG("receiveHeader -Exit\n");

	return header;
}

/**
     * This function is used to receive and check the checksum of the T=1 frame.
     *
     * @param rPcb
     *            PCB field of the current frame
     * @param rLen
     *            LEN field of the current frame
     * @param data
     *            DATA field of the current frame
     * @param dataLength
     *
     * @param filp
     * 			  File pointer
     *
     */
static void receiveAndCheckChecksum(struct file *filp, short rPcb, short rLen,
			     unsigned char data[], int dataLength)
{
	int lrc = rPcb ^ rLen;
	int receivedCs = 0;
	int expectedCs = 0;
	NFC_DBG_MSG("receiveAndCheckChecksum -Enter\n");

	dataLength = dataLength - csSize;

	// compute the expected CS

	expectedCs = lrc ^ helperComputeLRC(data, 0, dataLength);

	// receive the T=1 CS
	receive(filp, &checksum, csSize, C_TRANSMIT_NO_START_CONDITION);

	receivedCs = checksum[0];

	// compare the chechsums
	if (expectedCs != receivedCs) {
		NFC_DBG_MSG("Checksum error \n");
	}

	NFC_DBG_MSG("receiveAndCheckChecksum -Exit\n");
}

/**
     * Basic send function which directly calls the spi bird wrapper function
     *
     * @param data
     *            the data to be send
     *
     */
static int send(struct file *filp, unsigned char **data, unsigned char mode, int len)
{
	int count = 0;
	NFC_DBG_MSG("send - Enter\n");

	NFC_DBG_MSG("send - len = %d\n", len);

	// call to the spi bird wrapper
	count = p61_dev_write(filp, *data, len, 0x00);

	if (count == 0) {
		NFC_ERR_MSG("ERROR:Failed to send data to device\n");
		return -1;
	}
	return count;
}

static int receive(struct file *filp, unsigned char **data, int len,
	    unsigned char mode)
{
	static int count_status;
	NFC_DBG_MSG("receive -Enter\n");
	count_status = p61_dev_read(filp, *data, len, 0x00);
	if (count_status == 0 && len != 0) {
		NFC_ERR_MSG("ERROR:Failed to receive data from device\n");
		return -1;
	}

	NFC_DBG_MSG("receive -Exit\n");

	return count_status;

}

static ssize_t p61_dev_write(struct file *filp, const char *buf,
			     size_t count, loff_t * offset)
{
	int ret = -1;
	struct p61_dev *p61_dev = NULL;
	//char tmp[MAX_BUFFER_SIZE];
	NFC_DBG_MSG("p61_dev_write -Enter\n");
	p61_dev = filp->private_data;
	/* Write data */
	ret = spi_write(p61_dev->spi, buf, count);
	NFC_DBG_MSG("spi_write status = %d\n", ret);
	if (ret < 0) {
		ret = -EIO;
	}

	NFC_DBG_MSG("p61_dev_write -Exit\n");
	udelay(1000);
	return count;
}

static ssize_t p61_dev_read(struct file *filp, char *buf,
			    size_t count, loff_t * offset)
{
	int ret = -1;
	struct p61_dev *p61_dev = filp->private_data;
	NFC_DBG_MSG("p61_dev_read - Enter \n");
	mutex_lock(&p61_dev->read_mutex);
	NFC_DBG_MSG("p61_dev_read - aquried mutex - calling spi_read \n");
	NFC_DBG_MSG("p61_dev_read - test1 count [0x%x] \n", count);
	/** Read data */
#ifdef IRQ_ENABLE
	NFC_DBG_MSG("************ Test11 *****************\n");
	if (!gpio_get_value(p61_dev->irq_gpio)) {
		while (1) {
			NFC_DBG_MSG("************ Test1 *****************\n");
			NFC_DBG_MSG(" %s inside while(1) \n", __func__);
			p61_dev->irq_enabled = true;
			enable_irq(p61_dev->spi->irq);
			ret =
			    wait_event_interruptible(p61_dev->read_wq,
						     !p61_dev->irq_enabled);
			p61_disable_irq(p61_dev);
			if (ret) {
				NFC_DBG_MSG("p61_disable_irq() : Failed\n");
				goto fails;
			}
			NFC_DBG_MSG("************ Test2 *****************\n");
			if (gpio_get_value(p61_dev->irq_gpio))
				break;

			NFC_DBG_MSG("%s: spurious interrupt detected\n",
				    __func__);
		}
	}

	NFC_DBG_MSG("************  gpio already high read data Test11 *****************\n");
#endif
	NFC_DBG_MSG("************ Test3 *****************\n");
	ret = spi_read(p61_dev->spi, (void *)buf, count);
	if (0 > ret) {
		NFC_ERR_MSG("spi_read returns -1 \n");
		goto fails;
	}

	NFC_DBG_MSG("Read ret %d \n", ret);

	mutex_unlock(&p61_dev->read_mutex);

	if (0 == ret) {
		ret = count;
	}

	return ret;

fails:
	mutex_unlock(&p61_dev->read_mutex);
	return ret;
}

/**
 * This function is used to send a single T=1 frame.
 *
 * @param data
 *            the data to be send
 * @param mode
 *            used to signal chaining
 *
 */
static int sendFrame(struct file *filp, const char data[], char mode, int count)
{
	int count_status = 0;
	int len = count + headerSize + csSize;
	unsigned char *frame = NULL;
	NFC_DBG_MSG("sendFrame - Enter\n");
	frame = gSendframe;
	// update the send sequence counter of the terminal
	seqCounterTerm = (unsigned char)(seqCounterTerm ^ 1);

	// prepare the frame and send it
	frame[0] = 0x00;
	frame[1] = (unsigned char)(mode | (unsigned char)(seqCounterTerm << 6));
	frame[2] = (unsigned char)(count);

	memcpy((frame + 3), data, count);

	frame[count + headerSize] =
	    (unsigned char)helperComputeLRC(frame, 0, count + headerSize - 1);
	lastFrame = frame;
	lastFrameLen = len;
	count_status = send(filp, &frame, C_TRANSMIT_NORMAL_SPI_OPERATION, len);

	if (count_status == 0) {
		NFC_ERR_MSG("ERROR:Failed to send device\n");
		return -1;
	}

	NFC_DBG_MSG("sendFrame ret = %d - Exit\n", count_status);
	return count_status;
}

/**
     * Helper function to compute the LRC.
     *
     * @param data
     *            the data array
     * @param offset
     *            offset into the data array
     * @param length
     *            length value
     *
     */
static unsigned char helperComputeLRC(unsigned char data[], int offset, int length)
{
	int LRC = 0;
	int i = 0;
	NFC_DBG_MSG("helperComputeLRC - Enter\n");
	for (i = offset; i <= length; i++) {
		LRC = LRC ^ data[i];
	}
	NFC_DBG_MSG("LRC Value is  %x \n", LRC);
	return (unsigned char)LRC;
}

/**
    * This function initializes the T=1 module
    *
    */
static void init()
{
	NFC_DBG_MSG("init - Enter\n");
	apduBufferidx = 0;
	setAddress(0);
	setBitrate(100);

	NFC_DBG_MSG("init - Exit\n");
}

static void setAddress(short address)
{
	int stat = 0;
	NFC_DBG_MSG("setAddress -Enter\n");
	stat = nativeSetAddress(address);

	if (stat != 0) {
		NFC_ERR_MSG("set address failed.\n");
	}

	NFC_DBG_MSG("setAddress -Exit\n");
}

static void setBitrate(short bitrate)
{
	int stat = 0;
	NFC_DBG_MSG("setBitrate -Enter\n");
	stat = nativeSetBitrate(bitrate);

	if (stat != 0) {
		NFC_ERR_MSG("set bitrate failed.\n");
	}

	NFC_DBG_MSG("setBitrate -Exit\n");
}

static int nativeSetAddress(short address)
{
	NFC_DBG_MSG("nativeSetAddress -Enter\n");
	return 0;
}

static int nativeSetBitrate(short bitrate)
{
	NFC_DBG_MSG("nativeSetBitrate -Enter\n");

	return 0;
}

#ifdef IRQ_ENABLE
static void p61_disable_irq(struct p61_dev *p61_dev)
{
	unsigned long flags;

	NFC_DBG_MSG("Entry : %s\n", __func__);
	spin_lock_irqsave(&p61_dev->irq_enabled_lock, flags);
	if (p61_dev->irq_enabled) {
		disable_irq_nosync(p61_dev->spi->irq);
		p61_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&p61_dev->irq_enabled_lock, flags);
	NFC_DBG_MSG("Exit : %s\n", __func__);
}

static irqreturn_t p61_dev_irq_handler(int irq, void *dev_id)
{
	struct p61_dev *p61_dev = dev_id;

	NFC_DBG_MSG("Entry : %s\n", __func__);
	p61_disable_irq(p61_dev);

	/* Wake up waiting readers */
	wake_up(&p61_dev->read_wq);

	NFC_DBG_MSG("Exit : %s\n", __func__);
	return IRQ_HANDLED;
}
#endif

static inline void p61_set_data(struct spi_device *spi, void *data)
{
	dev_set_drvdata(&spi->dev, data);
}

static const struct file_operations p61_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = p61_dev_receiveData,
	.write = p61_dev_sendData,
	.open = p61_dev_open,
	.release = p61_dev_close,
	.poll = p61_dev_poll,
	.unlocked_ioctl = p61_dev_ioctl,	// TBD : need to implement
};

static int p61_probe(struct spi_device *spi)
{
	int ret = 0;
	int gpio;
	struct device *dev = &spi->dev;
	struct p61_dev *p61_dev = NULL;
	unsigned int irq_flags;

	printk("P61 with irq without log Entry : %s\n", __func__);

	NFC_DBG_MSG("chip select : %d , bus number = %d \n", spi->chip_select,
		    spi->master->bus_num);

	p61_dev = kzalloc(sizeof(*p61_dev), GFP_KERNEL);
	if (p61_dev == NULL) {
		NFC_ERR_MSG("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	gpio = of_get_named_gpio(dev->of_node, "nxp,spi-reset-gpio", 0);
	if (!gpio_is_valid(gpio))
		goto gpio_exit;

	ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_LOW,
				    "nfc-spi-reset-gpio");
	if (ret)
		dev_err(dev, "can't request spi_reset gpio %d", gpio);
	
	ret = gpio_direction_output(gpio, 0);

	p61_dev->ven_gpio = gpio;
	NFC_ERR_MSG("gpio_direction_output returned = 0x%x\n", ret);

#ifdef IRQ_ENABLE
	gpio = of_get_named_gpio(dev->of_node, "nxp,spi-irq-gpio", 0);
	if (!gpio_is_valid(gpio))
		goto gpio_exit;

	ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, "nfc-spi-irq-gpio");
	if (ret)
		dev_err(dev, "can't request spi_irq gpio %d", gpio);
	
	//ret = gpio_direction_input(gpio);
	p61_dev->irq_gpio = gpio;
#endif

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	//spi->max_speed_hz = 4000000;	//1000000;
	//spi->chip_select = SPI_NO_CS;
	ret = spi_setup(spi);
	if (ret < 0) {
		NFC_ERR_MSG("failed to do spi_setup()\n");
		goto err_exit0;
	}

	p61_dev->spi = spi;
	p61_dev->p61_device.minor = MISC_DYNAMIC_MINOR;
	p61_dev->p61_device.name = "p61";
	p61_dev->p61_device.fops = &p61_dev_fops;
	p61_dev->p61_device.parent = &spi->dev;


	gpio_set_value(p61_dev->ven_gpio, 1);
	msleep(20);
	NFC_DBG_MSG("p61_dev->rst_gpio = %d\n ", p61_dev->ven_gpio);

	p61_set_data(spi, p61_dev);
	/* init mutex and queues */
	init_waitqueue_head(&p61_dev->read_wq);
	mutex_init(&p61_dev->read_mutex);
	//spin_lock_init(&p61_dev->irq_enabled_lock);
#ifdef IRQ_ENABLE
	spin_lock_init(&p61_dev->irq_enabled_lock);
#endif

	ret = misc_register(&p61_dev->p61_device);
	if (ret < 0) {
		NFC_ERR_MSG("misc_register failed! %d\n", ret);
		goto err_exit0;
	}
#ifdef IRQ_ENABLE
	p61_dev->spi->irq = gpio_to_irq(p61_dev->irq_gpio);

	if (p61_dev->spi->irq < 0) {
		NFC_ERR_MSG("gpio_to_irq request failed gpio = 0x%x\n",
			    p61_dev->irq_gpio);
		goto err_exit0;
	}
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	p61_dev->irq_enabled = true;
	irq_flags = IRQF_TRIGGER_RISING | IRQF_ONESHOT;

	ret = request_irq(p61_dev->spi->irq, p61_dev_irq_handler,
			  irq_flags, p61_dev->p61_device.name, p61_dev);
	if (ret) {
		NFC_ERR_MSG("request_irq failed\n");
		goto err_exit0;
	}
	p61_disable_irq(p61_dev);

#endif

	NFC_DBG_MSG("Exit : %s\n", __func__);

	return ret;

//      err_exit1:
//      misc_deregister(&p61_dev->p61_device);
err_exit0:
	mutex_destroy(&p61_dev->read_mutex);
	gpio_free(p61_dev->irq_gpio);
	gpio_free(p61_dev->ven_gpio);
gpio_exit:
	kfree(p61_dev);
err_exit:
	return ret;
}

static inline void *p61_get_data(const struct spi_device *spi)
{
	return dev_get_drvdata(&spi->dev);
}

static int p61_remove(struct spi_device *spi)
{
	struct p61_dev *p61_dev = p61_get_data(spi);
	NFC_DBG_MSG("Entry : %s\n", __func__);
	NFC_DBG_MSG(" %s ::  name : %s ", __func__,
		    p61_dev->p61_device.name);

	free_irq(p61_dev->spi->irq, p61_dev);
	mutex_destroy(&p61_dev->read_mutex);
	misc_deregister(&p61_dev->p61_device);
	gpio_free(p61_dev->irq_gpio);
	gpio_free(p61_dev->ven_gpio);
	if (p61_dev != NULL)
		kfree(p61_dev);
	NFC_DBG_MSG("Exit : %s\n", __func__);
	return 0;
}

static struct of_device_id p61_of_match_table[] = {
	{.compatible = "nfc_ese_spi",},
	{},
};

static struct spi_driver pn544_spi_driver = {
	.driver = {
		   .name = "pn544_spi",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   .of_match_table = p61_of_match_table,
		   },
	.probe = p61_probe,
	.remove = p61_remove,
};

module_spi_driver(pn544_spi_driver);

MODULE_AUTHOR("MANJUNATHA VENKATESH");
MODULE_DESCRIPTION("NFC P61 SPI driver");
MODULE_LICENSE("GPL");
