/*
 * Simple synchronous userspace interface to SPI devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/completion.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/amba/bus.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/spi-s3c64xx.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/wakelock.h>

#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <mach/cpufreq.h>

#include <plat/gpio-cfg.h>
#include <plat/map-s5p.h>
#include "gf66xx-spi-tee.h"
#include "../../gud/gud-exynos5430/MobiCoreKernelApi/public/mobicore_driver_api.h"
#include "secdrv_Api.h"

/*spi device name*/
#define SPI_DEV_NAME   "spidev"
/*device name after register in charater*/
#define DEV_NAME "gf66xx-spi"
#define GF66XX_PID	"GF66XX"

/*control macro*/
#define  FW_UPDATE
#define  GF66XX_POWER_SAVING
#define  GF66XX_WATCHDOG

#define  	IRQ_AWAKE
#define  	GF66XX_HOME
#define  	EXYNOS_CHIPID
//#define  	TEST_DAEMON

#define GF66XX_SPI_VERSION "gf66xx_spi_tee_v0.96"

#ifdef FW_UPDATE
static unsigned char GF66XX_FW[]=
{
	#include "gf66xx_fw.i"
};
#endif

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define CHRD_DRIVER_NAME		"gf66xx"
#define CLASS_NAME				"gf66xx-spi"
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
#define GF66XX_CFG_LEN			249	/*config data length*/
#define GF66XX_CFG_ADDR			0x8047
#define FW_LENGTH 				(42*1024)

#ifdef EXYNOS_CHIPID
/*exynos chipid address*/
#define CHIPID0_OFFSET 		0x14
#define CHIPID1_OFFSET 		0x18

static unsigned int s5p_chip_id[2];
#endif

#define HMP_BOOST_TIMEOUT 	(2500 * MSEC_PER_SEC)
#define BIG_CORE_NUM        (2)
#define HMP_BOOST_FREQ 		2000000
static struct pm_qos_request fp_cpu_num_min_qos;
static struct pm_qos_request fp_cpu_freq_qos;
static struct class *gf66xx_spi_class;

/*  add for trustonic DCI interface and secure SPI driver  */
static u32 mc_deviceId = MC_DEVICE_ID_DEFAULT;
static const struct mc_uuid_t secdr_uuid = SECDRV_SPI_UUID;

static struct mc_session_handle secdr_SessionHandle;
static dciMessage_t *pDci  = NULL;
/* end for trustonic */

EVENT_TYPE g_lastEvent = GF66XX_NOEVENT;
u32 g_isIdle = 1;     //0: fp data is ready but not handled
extern int receive_keyhome_event;

static DECLARE_BITMAP(minors, N_SPI_MINORS);
/***********************************************************
gf66xx config
************************************************************/

struct config_buf {
    unsigned int date;
    unsigned char buffer[249];
};

struct gf66xx_config {
    unsigned char type; //hardware type
    unsigned char pid; //productor ID
    unsigned char config_num; //how many configs this productor has.
    struct config_buf *config;
};

#ifdef FW_UPDATE
static struct config_buf config_buf_list[] = {
        {
            .date = 0x7de090a,
            .buffer = {
				0x42,0xF0,0xF0,0xE4,0x0C,0x90,0x4D,0x03,0x00,0x19,0x0F,0xC8,0xC8,0xE4,0x0C,0x90,
				0x4D,0x02,0x80,0x03,0x11,0xA0,0x0D,0x00,0x14,0x0A,0x0F,0x0F,0x0F,0xB2,0x3F,0xB3,
				0x33,0x00,0x90,0x01,0x20,0xC0,0x03,0x28,0xB4,0x0F,0x14,0x14,0x40,0x47,0x4E,0x56,
				0x60,0x6B,0x4B,0x32,0x32,0xD0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x10,0x00,0xA0,0x0F,0xD0,0x07,0x20,
				0x03,0x62,0x0E,0x3A,0x15,0x22,0xD0,0x00,0x00,0x60,0x03,0x00,0x00,0x02,0x04,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,
				0x00,0xFF,0x0F,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x01
            },
        },
        {
            .date = 0x7de0a1c,
            .buffer = {
				0x43,0xE6,0xE6,0xE4,0x0C,0x90,0x4C,0x04,0x00,0x19,0x0F,0xC8,0xC8,0xE4,0x0C,0x90,
				0x4D,0x02,0x80,0x05,0x11,0xA0,0x0D,0x00,0x14,0x0A,0x0F,0x0F,0x0F,0xB3,0x3F,0xB3,
				0x33,0x00,0x90,0x01,0x20,0xC0,0x03,0x28,0xB4,0x0F,0x14,0x14,0x40,0x47,0x4E,0x56,
				0x60,0x6B,0x4B,0x32,0x32,0xD0,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x10,0x00,0xA0,0x0F,0xD0,0x07,0x20,
				0x03,0x62,0x0E,0x3A,0x15,0x22,0xD0,0x00,0x00,0x60,0x03,0x00,0x00,0x02,0x04,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x01,0x01,0x00,0x00,0x00,
				0x00,0xFF,0x0F,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
				0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x48,0x01
            }
        },
};
static struct gf66xx_config config_list[] = {
    {
        .type = 0,
        .pid = 1,
        .config_num = 2,
        .config = &config_buf_list[0],
    }
};
#endif

static u8 probe_finish = 0;
//#undef GF66XX_FASYNC

/**************************debug******************************/
#ifdef GF66XX_DEBUG
#undef GF66XX_DEBUG
#endif

#ifdef GF66XX_TEST
#undef GF66XX_TEST
#endif

#define TEST_BUF_LEN 2048
#define TEST_CNT 10000
#define SPI_SPEED_MIN   1*1000*1000
#define SPI_SPEED_MAX   8*1000*1000
static u32 spi_speed_low = SPI_SPEED_MIN;
static u32 spi_speed_high = SPI_SPEED_MAX;

#ifdef GF66XX_DEBUG
#define   gf66xx_dbg(fmt, args...) do{ \
					pr_warn("gf66xx:" fmt, ##args);\
				}while(0)
#define FUNC_ENTRY()  pr_warn("gf66xx:%s, entry\n", __func__)
#define FUNC_EXIT()  pr_warn("gf66xx:%s, exit\n", __func__)
#else
#define gf66xx_dbg(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
#endif

/******************function declaration************************/
#ifdef GF66XX_POWER_SAVING
static void gx_irq_output_config(struct gf66xx_dev *);
static void gx_irq_output_low_config(struct gf66xx_dev *);
#endif

static void gx_irq_gpio_config(struct gf66xx_dev *);
static void lock_big_core(void);
static int sec_spi_set_speed(struct gf66xx_dev *, u32);
static irqreturn_t gf66xx_irq(int irq, void* handle);

#ifdef GF66XX_WATCHDOG
static void gf66xx_timer_work(struct work_struct *work);
static void gf66xx_timer_func(unsigned long arg);
#endif

#ifdef FW_UPDATE
static int isUpdate(struct gf66xx_dev *gf66xx_dev);
static int gf66xx_fw_update_init(struct gf66xx_dev *gf66xx_dev);
extern	int gf66xx_fw_update(struct gf66xx_dev* gf66xx_dev, unsigned char *buf, unsigned short len);
#endif
/**********************************************************/
static DEFINE_MUTEX(gf66xx_mutex);

#ifdef GF66XX_POWER_SAVING
static int sleep_wake_cnt = 0;
#endif

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
/*************************data stream***********************
*	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
*     1B      |   1B    |  1B     |    2048B     |  2B      |
************************************************************/
static unsigned bufsiz = 2048+5;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");
/*
static void show_mem(u8 *buf, u32 len)
{
	u32 i = 0;

	pr_warn("\n");
	for(i=0; i<len; i=i+8) {
		pr_warn("0x%p: %02x %02x %02x %02x %02x %02x %02x %02x\n" ,
			buf+i,*(buf+i), *(buf+i+1), *(buf+i+2), *(buf+i+3),
			*(buf+i+4), *(buf+i+5), *(buf+i+6), *(buf+i+7));
	}
}
*/
#ifdef SPI_ASYNC
static void gf66xx_spi_complete(void *arg)
{
	complete(arg);
}
#endif

static int gf66xx_secdrv_dci_cmd(u8 cmd)
{
	int ret = 0;
	enum mc_result mcRet = MC_DRV_OK;

	pDci->command.header.commandId = cmd;
	//printk("mobicore, command for DCI transfer: %d\n", cmd);

	mcRet = mc_notify(&secdr_SessionHandle);
	if(MC_DRV_OK != mcRet){
		pr_err("mobicore: mc_notify failed, mcRet: %d\n", mcRet);
		ret = -EBUSY;
		return ret;
	}

	if(MC_DRV_OK != (mcRet = mc_wait_notification(&secdr_SessionHandle, MC_INFINITE_TIMEOUT))){
		pr_err("mobicore: mc_wait_notification failed, mcRet: %d\n", mcRet);
		ret = -EBUSY;
		return ret;
	}

	if(RSP_ID(cmd) != pDci->response.header.responseId){
		pr_err("mobicore: secdrv not send a response %d\n", pDci->response.header.responseId);
		ret = -EBUSY;
		return ret;
	}
	if(RET_SECDRV_OK != pDci->response.header.returnCode){
		pr_err("mobicore: secdrv not send a valid return code %d\n", pDci->response.header.returnCode);
		ret = -EBUSY;
		}

	return ret;
}

int gf66xx_spi_write_byte(struct gf66xx_dev *gf66xx_dev, u16 addr, u8 value)
{
#if 1
	int ret = 0;

	if(gf66xx_dev->secspi_init_ok){
		if(gf66xx_dev->spi->max_speed_hz != spi_speed_low){
			gf66xx_dev->spi->max_speed_hz = spi_speed_low;
			sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
		}

		mutex_lock(&gf66xx_dev->dci_lock);
		pDci->rw_data.len = 1;
		pDci->rw_data.direction = 1;
		pDci->rw_data.addr = addr;
		pDci->rw_data.data[0] = value;

		ret = gf66xx_secdrv_dci_cmd(SECDEV_CMD_RW_ADDR);
		mutex_unlock(&gf66xx_dev->dci_lock);
		//printk("%s: write address %x: %d, ret:%d", __func__, addr, value, ret);
	}else{
		ret = EACCES;
	}
	return ret;
#else
	int status = 0;
	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->buffer[GF66XX_WDATA_OFFSET] = value;
	status = gf66xx_spi_write_bytes(gf66xx_dev, addr, 1, gf66xx_dev->buffer);
	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
#endif
}

/**************************************************************
*gf66xx_addr: SPI device start address
*data_len: data length will write
*tx_buf: write buffer used for SPI transfer, no offset for true data!!!!
***************************************************************/
int gf66xx_spi_write_bytes(struct gf66xx_dev *gf66xx_dev,
				u16 addr, u32 data_len, u8 *tx_buf)
{
#if 1
	int ret = 0;

	if(gf66xx_dev->secspi_init_ok){
		if(gf66xx_dev->spi->max_speed_hz != spi_speed_low){
			gf66xx_dev->spi->max_speed_hz = spi_speed_low;
			sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
		}

		mutex_lock(&gf66xx_dev->dci_lock);
		pDci->rw_data.len = data_len;
		pDci->rw_data.direction = 1;
		pDci->rw_data.addr = addr;
		memcpy(pDci->rw_data.data, tx_buf, data_len);

		ret = gf66xx_secdrv_dci_cmd(SECDEV_CMD_RW_ADDR);
		mutex_unlock(&gf66xx_dev->dci_lock);
		//printk("%s: write address %x: %d, ret:%d", __func__, addr, value, ret);
	}else{
		ret = EACCES;
	}
	return ret;
#else
#ifdef SPI_ASYNC
	DECLARE_COMPLETION_ONSTACK(read_done);
#endif
	struct spi_message msg;
	struct spi_transfer *xfer;
	int ret = 0;

	xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	if( xfer == NULL){
		gf66xx_dbg("No memory for command.\n");
		return -ENOMEM;
	}

    /*send gf66xx command to device.*/
	spi_message_init(&msg);
	tx_buf[0] = GF66XX_W;
	tx_buf[1] = (u8)((addr >> 8)&0xFF);
	tx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tx_buf;
	xfer[0].len = data_len + 3;
	//xfer[0].delay_usecs = 5;
	xfer[0].delay_usecs = 0;
	spi_message_add_tail(xfer, &msg);
#ifdef SPI_ASYNC
	msg.complete = gf66xx_spi_complete;
	msg.context = &read_done;

	spin_lock_irq(&gf66xx_dev->spi_lock);
	ret = spi_async(gf66xx_dev->spi, &msg);
	spin_unlock_irq(&gf66xx_dev->spi_lock);
	if(ret == 0) {
		wait_for_completion(&read_done);
		if(msg.status == 0)
			ret = msg.actual_length - GF66XX_WDATA_OFFSET;
	}
#else
	spi_sync(gf66xx_dev->spi, &msg);
	ret = msg.actual_length - GF66XX_WDATA_OFFSET;
#endif
	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;

	return ret;
#endif
}


int gf66xx_spi_read_byte(struct gf66xx_dev *gf66xx_dev, u16 addr, u8 *value)
{
#if 1
	int ret = 0;

	if(gf66xx_dev->secspi_init_ok){
		if(gf66xx_dev->spi->max_speed_hz != spi_speed_low){
			gf66xx_dev->spi->max_speed_hz = spi_speed_low;
			sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
		}

		mutex_lock(&gf66xx_dev->dci_lock);
		pDci->rw_data.len = 1;
		pDci->rw_data.direction = 0;
		pDci->rw_data.addr = addr;
		//read data stored in pDci->rw_data.data

		ret = gf66xx_secdrv_dci_cmd(SECDEV_CMD_RW_ADDR);
		*value = pDci->rw_data.data[0];
		mutex_unlock(&gf66xx_dev->dci_lock);
		//printk("%s: read address %x: %d", __func__, addr, *value);
	}else{
		ret = EACCES;
	}
	return ret;
#else
	int status = 0;
	mutex_lock(&gf66xx_dev->buf_lock);
	status = gf66xx_spi_read_bytes(gf66xx_dev, addr, 1, gf66xx_dev->buffer);
	*value = gf66xx_dev->buffer[GF66XX_RDATA_OFFSET];
	mutex_unlock(&gf66xx_dev->buf_lock);
	return status;
#endif
}

/***************************************************************
*addr: SPI device start address
*data_len: data length will read
*rx_buf: read buffer used for SPI transfer, no offset for true data
****************************************************************/
int gf66xx_spi_read_bytes(struct gf66xx_dev *gf66xx_dev,
				u16 addr, u32 data_len, u8 *rx_buf)
{
#if 1
	int ret = 0;

	if(gf66xx_dev->secspi_init_ok){
		if(gf66xx_dev->spi->max_speed_hz != spi_speed_low){
			gf66xx_dev->spi->max_speed_hz = spi_speed_low;
			sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
		}

		mutex_lock(&gf66xx_dev->dci_lock);
		pDci->rw_data.len = data_len;
		pDci->rw_data.direction = 0;
		pDci->rw_data.addr = addr;
		//read data stored in pDci->rw_data.data

		ret = gf66xx_secdrv_dci_cmd(SECDEV_CMD_RW_ADDR);
		memcpy(rx_buf, pDci->rw_data.data, data_len);
		mutex_unlock(&gf66xx_dev->dci_lock);
	}else{
		ret = EACCES;
	}
	return ret;
#else
#ifdef SPI_ASYNC
	DECLARE_COMPLETION_ONSTACK(write_done);
#endif
	struct spi_message msg;
	struct spi_transfer *xfer;
	int ret = 0;

	xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	if( xfer == NULL){
		gf66xx_dbg("No memory for command.\n");
		return -ENOMEM;
	}

    /*send gf66xx command to device.*/
	spi_message_init(&msg);
	rx_buf[0] = GF66XX_W;
	rx_buf[1] = (u8)((addr >> 8)&0xFF);
	rx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = rx_buf;
	xfer[0].len = 3;
	//xfer[0].delay_usecs = 5;
	xfer[0].delay_usecs = 0;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf66xx_dev->spi, &msg);
	spi_message_init(&msg);

	//memset(rx_buf, 0xff, data_len);
	rx_buf[3] = GF66XX_R;
	xfer[1].tx_buf = &rx_buf[3];
	xfer[1].rx_buf = &rx_buf[3];
	xfer[1].len = data_len + 1;
	//xfer[1].delay_usecs = 5;
	xfer[1].delay_usecs = 0;
	spi_message_add_tail(&xfer[1], &msg);

#ifdef SPI_ASYNC
	msg.complete = gf66xx_spi_complete;
	msg.context = &write_done;

	spin_lock_irq(&gf66xx_dev->spi_lock);
	ret = spi_async(gf66xx_dev->spi, &msg);
	spin_unlock_irq(&gf66xx_dev->spi_lock);
	if(ret == 0) {
		wait_for_completion(&write_done);
		if(msg.status == 0)
			ret = msg.actual_length - 1;
	}
#else
	spi_sync(gf66xx_dev->spi, &msg);
	ret = msg.actual_length - 1;
#endif
	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;
	return ret;
#endif
}

/* functions add for DCI command use */
static int gf66xx_read_chip_ID(struct gf66xx_dev *gf66xx_dev)
{
	/*only called at init phase, so no dci_lock used*/
	u8 version = 0;
	pDci->readchipid.len = 10;
	pDci->readchipid.addr = 0x8000;
	memset(pDci->readchipid.chipid, 0x00, 10);

	/*make sure gf66xx_dev->secspi_init_ok is 1 before call*/
	gf66xx_secdrv_dci_cmd(SECDEV_CMD_READ_CHIP_ID);
	pr_info("%s: through DCI 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x, %02d.%02d.%02d\n", __func__,
			pDci->readchipid.chipid[0], pDci->readchipid.chipid[1], pDci->readchipid.chipid[2],
			pDci->readchipid.chipid[3], pDci->readchipid.chipid[4], pDci->readchipid.chipid[5],
			pDci->readchipid.chipid[7], pDci->readchipid.chipid[8], pDci->readchipid.chipid[9]);

	gf66xx_spi_read_byte(gf66xx_dev, 0x8000, &version);
	if(version != 0x47)
	{
		usleep_range(1000,1500);
		gf66xx_spi_read_byte(gf66xx_dev, 0x8000, &version);
		if(version != 0x47)
		{
			pr_err("[ERR] %s device detect error!! version = 0x%x\n", __func__, version);
			return -1;
		}
	}

	return 0;
}



/********************************************************************
*CPU output low level in RST pin to reset GF66XX. This is the MUST action for GF66XX.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
static void gf66xx_hw_reset(struct gf66xx_dev *dev)
{
	gpio_direction_output(dev->reset_gpio, 1);
	gpio_set_value(dev->reset_gpio, 0);
	usleep_range(1000, 1500);
	gpio_set_value(dev->reset_gpio, 1);
	msleep(60);
}

int  gf66xx_hw_reset1(struct gf66xx_dev *gf66xx_dev)
{
	int i = 0;
	u8  value = 0;

	if(!(gf66xx_dev->secspi_init_ok)){
		pr_err("%s: secure SPI not finish init, please call gf66xx_hw_reset instead\n", __func__);
		return -1;
	}

	for(i = 0; i < 100; i++) {
		gf66xx_hw_reset(gf66xx_dev);
		/*hold SS51 and DSP(write 0x0c to 0x4180)*/
		gf66xx_spi_write_byte(gf66xx_dev, 0x4180,0x0c);
		gf66xx_spi_read_byte(gf66xx_dev, 0x4180,&value);
		if(value == 0x0c){
			/*enable power of DSP and MCU*/
			gf66xx_dbg("value of 4180 is 0x%x\n", value);
			//gf66xx_spi_write_byte(gf66xx_dev, 0x4010,0x00);
			gf66xx_spi_write_byte(gf66xx_dev, 0x4180,0x00);
			break;
		} else {
			gf66xx_dbg("value of 0x4180 is 0x%x, expect 0x0c\n", value);
		}
	}
	if(i == 100) {
		pr_err("Failed to reset gf66xx.\n");
		return -1;
	} else {
		gf66xx_dbg("reset i = %d\n", i);
	}
	msleep(50);
	return 0;
}


static void gf66xx_wakeup(struct gf66xx_dev *gf66xx_dev)
{
#ifdef GF66XX_POWER_SAVING
	mutex_lock(&gf66xx_mutex);

	if((gf66xx_dev->software_available == 0) /*|| (gf66xx_dev->secspi_init_ok == 0)*/){
		//pr_info("software is diable,shouldn't awake the device\n");
		mutex_unlock(&gf66xx_mutex);
		return;
	}else if(sleep_wake_cnt != 1){
		pr_info("%s: sleep_wake_cnt not right\n", __func__);
		mutex_unlock(&gf66xx_mutex);
		return;
	}

#ifdef IRQ_AWAKE
	gx_irq_output_config(gf66xx_dev);
	usleep_range(3000, 4000);
	gx_irq_gpio_config(gf66xx_dev);
#else
	gf66xx_hw_reset(gf66xx_dev);
#endif

	sleep_wake_cnt--;
	if(sleep_wake_cnt != 0)
		pr_warn("Unbalance sleep_wake_cnt = %d\n", sleep_wake_cnt);

	/*lock big core when verify*/
	lock_big_core();
	mutex_unlock(&gf66xx_mutex);
#endif
}

static void gf66xx_wakeup_bottom(struct gf66xx_dev *gf66xx_dev)
{
	mutex_lock(&gf66xx_mutex);

	if(gf66xx_dev->secspi_init_ok == 0){
		pr_info("%s: secure spi not init\n", __func__);
		mutex_unlock(&gf66xx_mutex);
		return;
	}

	if(gf66xx_dev->mode == GF66XX_KEY_MODE) {
		//msleep(60);
		gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, gf66xx_dev->mode);
	}

	enable_irq(gf66xx_dev->spi->irq);
#ifdef GF66XX_WATCHDOG
	gf66xx_dev->gf66xx_timer.expires = jiffies + 2 * HZ;
	add_timer(&gf66xx_dev->gf66xx_timer);
#endif

	mutex_unlock(&gf66xx_mutex);
}

static void gf66xx_sleep(struct gf66xx_dev *gf66xx_dev)
{
#ifdef GF66XX_POWER_SAVING
	mutex_lock(&gf66xx_mutex);

	if((gf66xx_dev->mode == GF66XX_SLEEP_MODE) || (gf66xx_dev->secspi_init_ok == 0)){
		//pr_info("gf66xx has already been in SLEEP\n");
		mutex_unlock(&gf66xx_mutex);
		return;
	}else if(sleep_wake_cnt != 0){
		pr_info("%s: sleep_wake_cnt not right\n", __func__);
		mutex_unlock(&gf66xx_mutex);
		return;
	}

	wake_lock_timeout(&gf66xx_dev->fp_wakelock, 2 * HZ);

	disable_irq(gf66xx_dev->spi->irq);
	gx_irq_output_low_config(gf66xx_dev);
	mutex_lock(&gf66xx_dev->buf_lock);

	if(gf66xx_dev->spi->max_speed_hz != spi_speed_low){
		gf66xx_dev->spi->max_speed_hz = spi_speed_low;
		sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
	}
	gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS,GF66XX_SLEEP_MODE);
	mutex_unlock(&gf66xx_dev->buf_lock);
#ifdef GF66XX_WATCHDOG
	del_timer_sync(&gf66xx_dev->gf66xx_timer);
#endif
	//gf66xx_dev->mode = GF66XX_SLEEP_MODE;

	sleep_wake_cnt++;
	if(sleep_wake_cnt != 1)
		pr_warn("Unbalance sleep_wake_cnt = %d\n", sleep_wake_cnt);

	mutex_unlock(&gf66xx_mutex);
#endif
}

#ifdef FW_UPDATE
static bool hw_config(struct gf66xx_dev *gf66xx_dev)
{
	int timeout = 0;
	int   retry_cnt = 0;
	unsigned int  date = 0;
	u8  reg_value = 0;
	u8  pid = 0;
	u8  i = 0;
	u8  j = 0;
	u8 k;
	u8 clear_config_buffer[249] = {0};

	do{
	        /*Send read command to hardware.*/
		gf66xx_spi_write_byte(gf66xx_dev, 0x8041,0x02);
		gf66xx_spi_write_byte(gf66xx_dev, 0x8042,0xfe);

	        /*Waiting the chip to be respond this command.*/
	        msleep(10);
		gf66xx_spi_read_byte(gf66xx_dev, 0x8042,&reg_value);
		if(reg_value != 0xAA) {
			//printk("Chip isn't respond this command.\n");
			gf66xx_spi_write_byte(gf66xx_dev, 0x8041,0x00);
			continue;
		}

	        timeout = 0;
	        /*write the index of data to be read.*/
		gf66xx_spi_write_byte(gf66xx_dev, 0x5094,1);
	        /*Waiting the data to be ready.*/
		gf66xx_spi_read_byte(gf66xx_dev, 0x5094,&reg_value);
		while(reg_value != 0) {
			if(timeout++ > 200) {
				printk("Write data timeout.\n");
				gf66xx_spi_write_byte(gf66xx_dev, 0x8041,0);
				break;
			}
			udelay(1000);
			gf66xx_spi_read_byte(gf66xx_dev, 0x5094,&reg_value);
		}
	        if(timeout > 200)
	            continue;

	        /*read user data(4) from 0xA34A.*/
	        gf66xx_spi_read_bytes(gf66xx_dev, 0xA34A, 24, gf66xx_dev->buffer);

	        pid = (gf66xx_dev->buffer[0]);
	        date = (gf66xx_dev->buffer[0x10] << 24 ) |
	                (gf66xx_dev->buffer[0x11] << 16 ) |
	                (gf66xx_dev->buffer[0x12] << 8 ) |
	                (gf66xx_dev->buffer[0x13]);

	    /*exit command mode*/
		gf66xx_spi_write_byte(gf66xx_dev, 0x8041,0);

		//printk("date = 0x%x\n", date);
		for(i = 0; i < sizeof(config_list)/sizeof(struct gf66xx_config); i++) {
			if(pid == config_list[i].pid) {
				for(j = config_list[i].config_num ; j > 0; j--) {
					if(date >= config_list[i].config[j-1].date) {
                        printk("Using config. config_list[%d].config_buf_list[%d]\n", i, j-1);
						if(gf66xx_dev->config_need_clear) {
							/*clear config version*/
							for(k = 0; k < (sizeof(clear_config_buffer) / sizeof(clear_config_buffer[0])); k++)
								clear_config_buffer[k] = config_list[i + 1].config[j-1].buffer[k];
							clear_config_buffer[247] = clear_config_buffer[0] + clear_config_buffer[247];
							clear_config_buffer[0] = 0x00;
							memcpy(gf66xx_dev->buffer, clear_config_buffer, GF66XX_CFG_LEN);
							gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
						} else {
							/*write config*/
							memcpy(gf66xx_dev->buffer, config_list[i].config[j-1].buffer, GF66XX_CFG_LEN);
							gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
						}
                        return true;
					}
					if(j == 1) {
                        printk("Using config. config_list[%d].config_buf_list[0]\n",i);
						if(gf66xx_dev->config_need_clear) {
							/*clear config version*/
							for(k = 0; k < (sizeof(clear_config_buffer) / sizeof(clear_config_buffer[0])); k++)
								clear_config_buffer[k] = config_list[i + 1].config[j-1].buffer[k];
							clear_config_buffer[247] = clear_config_buffer[0] + clear_config_buffer[247];
							clear_config_buffer[0] = 0x00;
							memcpy(gf66xx_dev->buffer, clear_config_buffer, GF66XX_CFG_LEN);
							gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
						} else {
							/*write config*/
							memcpy(gf66xx_dev->buffer, config_list[i].config[j-1].buffer, GF66XX_CFG_LEN);
							gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_CFG_ADDR, GF66XX_CFG_LEN, gf66xx_dev->buffer);
						}
                        return true;
					}
				}
			}
		}
	}while(retry_cnt++ < 10);
	return false;
}
#endif

#ifdef GF66XX_TEST
static void write_test(struct gf66xx_dev *gf66xx_dev)
{
	FUNC_ENTRY();
	gf66xx_spi_write_bytes(gf66xx_dev, 0x8000, TEST_BUF_LEN, gf66xx_dev->buffer);
	FUNC_EXIT();
}

static void read_test(struct gf66xx_dev *gf66xx_dev)
{
	FUNC_ENTRY();
	gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, TEST_BUF_LEN, gf66xx_dev->buffer);
	FUNC_EXIT();
}
#endif //GF66XX_TEST

/*************************************************************************************
* Read-only message with current device setup
* NOT spport read operation in TEE version
**************************************************************************************/
static ssize_t gf66xx_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
#if 1
	pr_err("Not support read opertion in TEE version\n");
	return -EFAULT;
#else
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	ssize_t			status = 0;
	FUNC_ENTRY();
	if ((count > bufsiz)||(count == 0)) {
		pr_warn("Max size for write buffer is %d. wanted length is %d\n", bufsiz, count);
		FUNC_EXIT();
		return -EMSGSIZE;
	}

	mutex_lock(&gf66xx_dev->buf_lock);
	gf66xx_dev->spi->max_speed_hz = spi_speed_high;
	//spi_setup(gf66xx_dev->spi);
	sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);

	status = gf66xx_spi_read_bytes(gf66xx_dev, GF66XX_BUFFER_DATA, count, gf66xx_dev->buffer);
	if(status > 0) {
		unsigned long missing = 0;
		missing = copy_to_user(buf, gf66xx_dev->buffer, status);
		if(missing == status)
			status = -EFAULT;
	} else {
		pr_err("Failed to read data from SPI device.\n");
		status = -EFAULT;
	}

	mutex_unlock(&gf66xx_dev->buf_lock);
#ifdef GF66XX_TEST
{
	u8 status = 0;
	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &status);
	pr_warn("gf66xx_read, status = 0x%x\n", status);
}
#endif
	return status;
#endif
}

/***********************************************************************
* Write-only message with current device setup
* NOT spport write operation in TEE version
************************************************************************/
static ssize_t gf66xx_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
#if 1
	pr_err("Not support write opertion in TEE version\n");
	return -EFAULT;
#else
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	ssize_t			status = 0;
	FUNC_ENTRY();
	if(count > bufsiz) {
		pr_warn("Max size for write buffer is %d\n", bufsiz);
		return -EMSGSIZE;
	}

	mutex_lock(&gf66xx_dev->buf_lock);
	status = copy_from_user(gf66xx_dev->buffer, buf, count);
	if(status == 0) {
		gf66xx_dev->spi->max_speed_hz = spi_speed_high;
		//spi_setup(gf66xx_dev->spi);
		sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);

		status = gf66xx_spi_write_bytes(gf66xx_dev, GF66XX_BUFFER_DATA, count, gf66xx_dev->buffer);
	} else {
		pr_err("Failed to xfer data through SPI bus.\n");
		status = -EFAULT;
	}
	mutex_unlock(&gf66xx_dev->buf_lock);
	FUNC_EXIT();
	return status;
#endif
}

#ifdef EXYNOS_CHIPID
static void display_chip_id(void)
{
	s5p_chip_id[0] = __raw_readl(S5P_VA_CHIPID + CHIPID0_OFFSET);
	s5p_chip_id[1] = __raw_readl(S5P_VA_CHIPID + CHIPID1_OFFSET) & 0xFFFF;
	//pr_info("[info] %s Chip ID : %04x%08x\n", __func__, s5p_chip_id[1], s5p_chip_id[0]);
}
#endif

/*for TEE version*/
static int sec_spi_prepare(struct sec_spi_info *spi_info, struct spi_device *spi, struct gf66xx_dev* gf66xx_dev)
{
	struct s3c64xx_spi_csinfo *cs;
	struct s3c64xx_spi_driver_data *sdd = NULL;
	//struct gf66xx_dev *gf66xx_dev = NULL;;

	pr_info("sec_spi_prepare\n");

	//gf66xx_dev = container_of(spi, struct gf66xx_dev, spi);

	sdd = spi_master_get_devdata(spi->master);
	if (!sdd){
		pr_err("%s, failed to get devdata\n", __func__);
		return -EFAULT;
	}

	pm_runtime_get_sync(&sdd->pdev->dev);

	/*set spi clock rate ,SPEED *2 internally*/
	clk_set_rate(sdd->src_clk, spi_info->speed * 2);

	// MUST set cs low here, or application will hung!!
	// after call sec_spi_init, cs gpio will change to function pin, kernel NOT control it anymore
	cs = spi->controller_data;
	if(cs->line != (unsigned)NULL) {
		gpio_set_value(cs->line, 0);
		//pr_info("sec_spi_prepare, cs->line is 0\n");
	} else {
		//pr_info("sec_spi_prepare, cs->line is null\n");
	}
	return 0;
}

static int sec_spi_unprepare(struct sec_spi_info *spi_info, struct spi_device *spi)
{
	struct s3c64xx_spi_csinfo *cs;
	struct s3c64xx_spi_driver_data *sdd = NULL;

	pr_info("sec_spi_unprepare\n");

	sdd = spi_master_get_devdata(spi->master);
	if (!sdd)
		return -EFAULT;

	// disable chip select
	cs = spi->controller_data;
	if(cs->line != (unsigned)NULL){
		gpio_set_value(cs->line, 1);
		//pr_info("sec_spi_unprepare cs->line is 1\n");
	}

	pm_runtime_put(&sdd->pdev->dev);

	return 0;
}


static struct amba_device *adev_dma;
static int sec_spi_dma_prepare(struct sec_spi_info *spi_info)
{
	struct device_node *np;

	pr_info("sec_spi_dma_prepare enter\n");

	if (spi_info->port != 2)
		return -1;

	for_each_compatible_node(np, NULL, "arm,pl330")
	{
		if (!of_device_is_available(np))
			continue;

		if (!of_dma_secure_mode(np))
			continue;

		adev_dma = of_find_amba_device_by_node(np);

		pr_info("[%s]device_name:%s\n", __func__, dev_name(&adev_dma->dev));
		break;
	}

	if (adev_dma == NULL)
		return -1;

	pm_runtime_get_sync(&adev_dma->dev);

	return 0;
}

static int sec_spi_dma_unprepare(void)
{
	pr_info("sec_spi_dma_unprepare enter\n");

	if (adev_dma == NULL)
		return -1;

	pm_runtime_put(&adev_dma->dev);
	return 0;
}

static int sec_spi_set_speed(struct gf66xx_dev *gf66xx_dev, u32 new_clk)
{
	int ret;

	struct s3c64xx_spi_driver_data *sdd = NULL;
	sdd = spi_master_get_devdata(gf66xx_dev->spi->master);

	if (!sdd){
		pr_err(" %s, failed to get devdata\n", __func__);
		ret = -EFAULT;
		return ret;
	}

	// set spi clock rate
	if((new_clk <= SPI_SPEED_MAX) && (new_clk >= SPI_SPEED_MIN)){
		clk_set_rate(sdd->src_clk, new_clk * 2);
		ret = 0;
	}else{
		ret = -EFAULT;
	}

	return ret;
}

/*for TEE version*/
static long gf66xx_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf66xx_dev *gf66xx_dev = NULL;
	struct gf66xx_ioc_transfer *ioc = NULL;
	/*for TEE*/
	struct sec_spi_info spi_info;
	struct spi_speed_setting spi_speed;
	struct gf66xx_secdrv_cmd secdrv_cmd;
	enum mc_result mcRet = MC_DRV_OK;
	/*for TEE*/
	int err = 0;
	u32 tmp = 0;
	int retval = 0;
	u32 mode = 0xFFFFFFFF;
	u8 value = 0;

	FUNC_ENTRY();
	if (_IOC_TYPE(cmd) != GF66XX_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	  * IOC_DIR is from the user perspective, while access_ok is
	  * from the kernel perspective; so they look reversed.
	  */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
						(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
						(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	gf66xx_dev = (struct gf66xx_dev *)filp->private_data;

	switch(cmd) {
	case GF66XX_IOC_CMD:
		/* the use of ioc->buf is different with old implementation, NO OFFSET NEEDED now!!!!! */
		ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
		/*copy command data from user to kernel.*/
		if(copy_from_user(ioc, (struct gf66xx_ioc_transfer*)arg, sizeof(*ioc))){
			pr_err("Failed to copy command from user to kernel.\n");
			retval = -EFAULT;
			break;
		}

		if((ioc->len > bufsiz)||(ioc->len == 0)) {
			pr_warn("The request length[%d] is longer than supported maximum buffer length[%d].\n",
					ioc->len, bufsiz);
			retval = -EMSGSIZE;
			break;
		}

		mutex_lock(&gf66xx_dev->buf_lock);
		if(gf66xx_dev->spi->max_speed_hz != spi_speed_low){
			gf66xx_dev->spi->max_speed_hz = spi_speed_low;
			sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
		}
		if(ioc->cmd == GF66XX_R) {
			/*if want to read data from hardware.*/
			gf66xx_dbg("Read data from 0x%x, len = 0x%x buf = 0x%p\n", ioc->addr, ioc->len, ioc->buf);
			gf66xx_spi_read_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
			if(copy_to_user(ioc->buf, gf66xx_dev->buffer, ioc->len)) {
				pr_err("Failed to copy data from kernel to user.\n");
				retval = -EFAULT;
				mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}
		} else if (ioc->cmd == GF66XX_W) {
			/*if want to read data from hardware.*/
			gf66xx_dbg("Write data from 0x%x, len = 0x%x\n", ioc->addr, ioc->len);
			if(copy_from_user(gf66xx_dev->buffer, ioc->buf, ioc->len)){
				pr_err("Failed to copy data from user to kernel.\n");
				retval = -EFAULT;
				mutex_unlock(&gf66xx_dev->buf_lock);
				break;
			}
			gf66xx_spi_write_bytes(gf66xx_dev, ioc->addr, ioc->len, gf66xx_dev->buffer);
		} else {
			pr_warn("Error command for gf66xx.\n");
		}
		if(ioc != NULL) {
			kfree(ioc);
			ioc = NULL;
		}
		mutex_unlock(&gf66xx_dev->buf_lock);
		break;
	case GF66XX_IOC_REINIT:
		/* make sure secure SPI init finished first */
		disable_irq(gf66xx_dev->spi->irq);
		gf66xx_hw_reset1(gf66xx_dev);
		enable_irq(gf66xx_dev->spi->irq);
		//msleep(200); //waiting the hardware to wake-up.
		//gf66xx_hw_init(gf66xx_dev);
		pr_info("wake-up gf66xx\n");
		break;
	case GF66XX_IOC_SETSPEED:
		//not working for secure SPI!!!!
		retval = __get_user(tmp, (u32 __user*)arg);
		if(tmp > SPI_SPEED_MAX) {
			pr_warn("The maximum SPI speed is 8MHz.\n");
			retval = -EMSGSIZE;
			break;
		}
		if(retval == 0) {
			gf66xx_dev->spi->max_speed_hz=tmp;
			spi_setup(gf66xx_dev->spi);
			gf66xx_dbg("spi speed changed to %d\n", tmp);
		}
		break;
	case SEC_IOC_SPI_SETSPEED:
		/* setspeed ioctl for test only, please do NOT call outside normal driver!!! */
		/* SPI speed is controlled by normal driver only */
		retval = __get_user(tmp, (u32 __user*)arg);
		if(tmp > SPI_SPEED_MAX) {
			pr_warn("The maximum SPI speed is 8MHz.\n");
			retval = -EMSGSIZE;
			break;
		}else{
			gf66xx_dev->spi->max_speed_hz=tmp;
			sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
			pr_info("SEC_IOC_SPI_SETSPEED, speed = %d\n",tmp);
		}
		break;
	case GF66XX_IOC_INIT_SPI_SPEED_VARIABLE:
		//set speed global variable
		retval = copy_from_user(&spi_speed, (void __user *)arg, sizeof(spi_speed));
		if (retval) {
			retval = -EFAULT;
			break;
		}

		spi_speed_high = spi_speed.high_speed;
		spi_speed_low = spi_speed.low_speed;
		pr_info("init SPI speed global variable, [%d, %d]\n", spi_speed.high_speed, spi_speed.low_speed);
		break;
	case GF66XX_IOC_STOPTIMER:
		/*if device was unavailable,del_timer_sync was set in other places already*/
		#ifdef GF66XX_WATCHDOG
		if(gf66xx_dev->device_available == 1){
			del_timer_sync(&gf66xx_dev->gf66xx_timer);
		}
		#endif
		break;
	case GF66XX_IOC_STARTTIMER:
		/*if device was unavailable,add_timer was set in other places already*/
		#ifdef GF66XX_WATCHDOG
		if(gf66xx_dev->device_available == 1){
			gf66xx_dev->gf66xx_timer.expires = jiffies + 2 * HZ;
			add_timer(&gf66xx_dev->gf66xx_timer);
		}
		#endif
		break;
	case GF66XX_IOC_SETPULSE:
		/*Migrate the cpu to big and lock the big cpu at a higher freq in  retval ms*/
		lock_big_core();
		break;
	case GF66XX_IOC_SETMODE:
		retval = __get_user(mode, (u32 __user*)arg);
		if(gf66xx_dev->device_available == 0) {
			pr_info("[info] %s mode111 = %d\n", __func__, mode);
			/*screen off. record the mode*/
			if(mode == GF66XX_IMAGE_MODE || mode == GF66XX_KEY_MODE) {
				gf66xx_dev->software_available = 1;
				gf66xx_dev->mode = mode;
			} else if(mode == GF66XX_SLEEP_MODE){
				gf66xx_dev->software_available = 0;
				gf66xx_dev->mode = mode;
			} else {
				pr_warn("Unsupported mode[%d]\n", mode);
			}
		} else if(gf66xx_dev->device_available == 1) {
			pr_info("[info] %s mode222 = %d\n", __func__, mode);
			/*screen on.  Do the right thing.*/
			if(mode == gf66xx_dev->mode) {
				pr_info("gf66xx has already in mode[%d]\n", mode);
			} else {
				if(mode == GF66XX_IMAGE_MODE || mode == GF66XX_KEY_MODE) {
					gf66xx_dev->software_available = 1;
					/*judge the previous mode*/
					if(gf66xx_dev->mode == GF66XX_SLEEP_MODE){
						gf66xx_wakeup(gf66xx_dev);
						msleep(60);
						gf66xx_wakeup_bottom(gf66xx_dev);
					}
					gf66xx_dev->mode = mode;
					/*switch  mode*/
					gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, (u8)mode);
				} else if(mode == GF66XX_SLEEP_MODE){
					gf66xx_dev->software_available = 0;
					gf66xx_sleep(gf66xx_dev);
					gf66xx_dev->mode = mode;
				} else {
					pr_warn("Unsupported mode[%d]\n", mode);
				}
			}
		} else {
			pr_warn("Abnornal value: device_available value = %d\n", gf66xx_dev->device_available);
		}
		break;
	case GF66XX_IOC_GETCHIPID:
#ifdef EXYNOS_CHIPID
		display_chip_id();
		if(copy_to_user((void *)arg, s5p_chip_id, sizeof(s5p_chip_id))) {
			pr_err("Failed to copy chipId data from kernel to user.\n");
		}
#endif
		break;
	/*for TEE version*/
	case GF66XX_SEC_IOC_SPI_PREPARE:
		retval = copy_from_user(&spi_info, (void __user *)arg, sizeof(spi_info));
		if (retval) {
			retval = -EFAULT;
			break;
		}

		pr_info("%s ENABLE_SPI_CLOCK\n", __func__);
		gf66xx_dev->spi->max_speed_hz = spi_info.speed;
		retval = sec_spi_prepare(&spi_info, gf66xx_dev->spi, gf66xx_dev);
		if (retval < 0){
			pr_err("%s: Unable to enable spi clk\n",__func__);
		}

		retval = sec_spi_dma_prepare(&spi_info);
		if (retval) {
			pr_err("%s: Failed to prepare sec_dma\n", __func__);
			retval = -EFAULT;
		}
		break;
	case GF66XX_SEC_IOC_SPI_UNPREPARE:
		retval = copy_from_user(&spi_info, (void __user *)arg, sizeof(spi_info));
		if (retval) {
			retval = -EFAULT;
			break;
		}

		retval = sec_spi_unprepare(&spi_info, gf66xx_dev->spi);

		if (retval < 0){
			pr_err("%s: couldn't disable spi clks\n", __func__);
		}

		retval = sec_spi_dma_unprepare();
		if (retval) {
			pr_err("%s: Failed to unprepare sec_dma\n", __func__);
			retval = -EFAULT;
		}
		break;
	/*for TEE version*/
	case GF66XX_IOC_CONNECT_SECURE_DRIVER:
		#if 1
		//already open mc device in TAC?
		/* Open mobicore device */
		mcRet = mc_open_device(mc_deviceId);
		if(MC_DRV_OK != mcRet){
			pr_err("TEE: mc_open_device failed, mcRet: %d\n", mcRet);
			retval = -EFAULT;
			break;
		}
		pr_info("TEE: mc_open_device success, mcRet: %d\n", mcRet);
		#endif

		/* Allocating WSM for DCI */
		mcRet = mc_malloc_wsm(mc_deviceId, 0, sizeof(dciMessage_t), (u8 **) &pDci, 0);
		if(MC_DRV_OK != mcRet){
			pr_err("TEE: mc_malloc_wsm allocate DCI failed, mcRet: %d\n", mcRet);
			retval = -EFAULT;
			break;
		}
		pr_info("TEE: mc_malloc_wsm success, mcRet: %d\n", mcRet);

		/* Open session to the secure driver*/
		memset(&secdr_SessionHandle, 0, sizeof(secdr_SessionHandle));
		secdr_SessionHandle.device_id = mc_deviceId;
		mcRet = mc_open_session(&secdr_SessionHandle, &secdr_uuid, (u8 *)pDci, sizeof(dciMessage_t));
		if(MC_DRV_OK != mcRet){
			pr_err("TEE: mc_open_session failed, mcRet: %d\n", mcRet);
			retval = -EFAULT;
			break;
		}
		pr_info("TEE: mc_open_session success, mcRet: %d\n", mcRet);
		break;
	case GF66XX_IOC_CLOSE_SECURE_DRIVER:
		gf66xx_dev->secspi_init_ok = 0;

		#ifdef GF66XX_WATCHDOG
		del_timer_sync(&gf66xx_dev->gf66xx_timer);
		#endif

		/* TRUSTONIC suggest not close secure driver session, it may lead to t-base hung */
		mcRet = mc_close_session(&secdr_SessionHandle);
		if(MC_DRV_OK != mcRet){
			pr_err("TEE: mc_close_session failed, mcRet: %d\n", mcRet);
			retval = -EFAULT;
			break;
		}
		pr_info("TEE: mc_close_session success, mcRet: %d\n", mcRet);
		memset(&secdr_SessionHandle, 0, sizeof(secdr_SessionHandle));

		mcRet = mc_free_wsm(mc_deviceId, (u8 *)pDci);
		if(MC_DRV_OK != mcRet){
			pr_err("TEE: mc_free_wsm failed, mcRet: %d\n", mcRet);
			retval = -EFAULT;
			break;
		}
		pr_info("TEE: mc_free_wsm success, mcRet: %d\n", mcRet);

		#if 0   //will close by TLC
		mcRet = mc_close_device(mc_deviceId);
		if(MC_DRV_OK != mcRet){
			pr_err("TEE: mc_close_device failed, mcRet: %d\n", mcRet);
			retval = -EFAULT;
			break
		}
		printk("TEE: mc_close_device success, mcRet: %d\n", mcRet);
		#endif
		break;
	case GF66XX_IOC_DCI_TESTCASE:
		retval = copy_from_user(&secdrv_cmd, (void __user *)arg, sizeof(secdrv_cmd));
		if (retval) {
			retval = -EFAULT;
		}
		if(gf66xx_dev->secspi_init_ok){
			mutex_lock(&gf66xx_dev->dci_lock);
			pDci->testcase.len = secdrv_cmd.len;
			memcpy(pDci->testcase.data, secdrv_cmd.buf, secdrv_cmd.len);
			pr_info("TEE: copy from user string: %s\n", pDci->testcase.data);
			retval = gf66xx_secdrv_dci_cmd(secdrv_cmd.cmd);
			mutex_unlock(&gf66xx_dev->dci_lock);
			if (retval < 0){
				pr_err("%s: gf66xx_Secdrv_Request return failed(%d)\n", __func__, retval);
				break;
			}
			pr_info("%s: gf66xx_Secdrv_Request return success(%d)\n", __func__, retval);
			pr_info("TEE: receive after call secure driver: %s\n", pDci->testcase.data);
		}
		break;
	case GF66XX_IOC_SECURE_INIT:
		//add GF66XX special init for secure driver
		//SPI has transfered from normal to secure, no more access in normal driver
		pr_info("%s: secure SPI init finished======\n", __func__);
		gf66xx_dev->secspi_init_ok = 1;

#ifdef FW_UPDATE
		if(isUpdate(gf66xx_dev)) {
			unsigned char* fw = GF66XX_FW;
			/*Do upgrade action.*/
			gf66xx_fw_update_init(gf66xx_dev);
			gf66xx_fw_update(gf66xx_dev, fw, FW_LENGTH);
			gf66xx_hw_reset(gf66xx_dev);
			/*clear config version*/
			gf66xx_dev->config_need_clear = 1;
			if(!hw_config(gf66xx_dev))
				pr_info("[info] %s clear config version fail\n", __func__);
			/*finish clear config version*/
			gf66xx_dev->config_need_clear = 0;
			msleep(60);
		}

		//disable hw_config temporary
		/*write config*/
		if(!hw_config(gf66xx_dev)){
			pr_info("[info] %s write config fail\n", __func__);
		}
#endif

		retval = gf66xx_read_chip_ID(gf66xx_dev);
		if (retval < 0){
			pr_err("%s: gf66xx_read_chip_ID return failed(%d)!!!!!!!\n", __func__, retval);
			//TODO: chip ID not right, how to destory?
			gf66xx_dev->device_available = 0;
			gf66xx_dev->secspi_init_ok = 0;
			/*switch to non_secure world*/
			mutex_lock(&gf66xx_dev->dci_lock);
			pDci->switch_spi.to_secure = 0x00;
			retval = gf66xx_secdrv_dci_cmd(SECDEV_CMD_SWITCH_SPI);
			mutex_unlock(&gf66xx_dev->dci_lock);
			if (retval < 0){
				pr_err("%s: failed to switch spi to normal world(%d)\n", __func__, retval);
			}
			break;
		}
		/*change to key mode at startup*/
		gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &value);
		pr_info("%s: init fingerprint mode is %d, set to KEY MODE\n", __func__, value);
		gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, GF66XX_KEY_MODE);
		gf66xx_dev->mode = GF66XX_KEY_MODE;

		retval = request_threaded_irq(gf66xx_dev->spi->irq, NULL, gf66xx_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				dev_name(&(gf66xx_dev->spi->dev)), gf66xx_dev);
		if(!retval) {
			pr_info("[info] %s irq thread request success!\n", __func__);
			disable_irq(gf66xx_dev->spi->irq);
		}

#ifdef GF66XX_WATCHDOG
		//moved after secure SPI finished init
		INIT_WORK(&gf66xx_dev->spi_work, gf66xx_timer_work);
		init_timer(&gf66xx_dev->gf66xx_timer);
		gf66xx_dev->gf66xx_timer.function = gf66xx_timer_func;
		gf66xx_dev->gf66xx_timer.expires = jiffies + 20*HZ;
		gf66xx_dev->gf66xx_timer.data = (unsigned long)gf66xx_dev;
		add_timer(&gf66xx_dev->gf66xx_timer);
#endif

		fb_register_client(&gf66xx_dev->fb_notifier);
		register_pm_notifier(&gf66xx_dev->suspend_notifier);
		gf66xx_dev->device_available = 1;
		gf66xx_dev->software_available = 1;	/*image  mode (default) */

		break;
	case GF66XX_IOC_SET_IRQ:
		retval = __get_user(tmp, (u32 __user*)arg);
		if(tmp) {
			if(gf66xx_dev->secspi_init_ok){
				//enable irq by ioctl later
				pr_info("enable SPI gf66xx_irq\n");
				enable_irq(gf66xx_dev->spi->irq);
			}
		}else{
			if(gf66xx_dev->secspi_init_ok){
				//disable irq by ioctl later
				pr_info("disable SPI gf66xx_irq\n");
				disable_irq(gf66xx_dev->spi->irq);
			}
		}
		break;
	case GF66XX_IOC_HAS_IDLE:
		retval = __get_user(tmp, (u32 __user*)arg);
		if(tmp) {
			pr_info("fingerpint chip can enter idle\n");
			//gf66xx_dev->has_idle = 1;
		}else{
			pr_info("fingerpint chip will NOT enter idle\n");
			//gf66xx_dev->has_idle = 0;
		}
		break;
	case GF66XX_IOC_RESET_SAMPLE_STATUS:
		retval = __get_user(tmp, (u32 __user*)arg);
		if(tmp) {
			pr_info("GF66XX_IOC_RESET_SAMPLE_STATUS: reset sample status\n");
			mutex_lock(&gf66xx_dev->dci_lock);
			pDci->reset_sample.do_reset_sample = 1;
			gf66xx_secdrv_dci_cmd(SECDEV_CMD_RESET_SAMPLE);
			mutex_unlock(&gf66xx_dev->dci_lock);
			g_isIdle = 1;
		}else{
			//pr_info("GF66XX_IOC_RESET_SAMPLE_STATUS: TODO\n");
		}
		break;
	case GF66XX_IOC_GET_EVENT_TYPE:
		retval = __put_user(gf66xx_dev->event_type, (u32 __user*)arg);
		break;
	case GF66XX_IOC_SECDRV_WR_TEST:
		pr_info("GF66XX_IOC_SECDRV_WR_TEST: start\n");
		gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &value);
		tmp = value;
		pr_info("GF66XX_IOC_SECDRV_WR_TEST1: original mode 0x%x, change to debug mode\n", value);
		gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, GF66XX_DEBUG_MODE);
		gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &value);
		pr_info("GF66XX_IOC_SECDRV_WR_TEST2: now mode 0x%x, change to key mode\n", value);
		gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, GF66XX_KEY_MODE);
		gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &value);
		pr_info("GF66XX_IOC_SECDRV_WR_TEST3: now mode 0x%x, change to origin mode\n", value);
		gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, (u8)tmp);
		gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &value);
		pr_info("GF66XX_IOC_SECDRV_WR_TEST4: back to original mode 0x%x\n", value);

		gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 10, gf66xx_dev->buffer);
		pr_info("GF66XX_IOC_SECDRV_WR_TEST5: read chip ID 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x, %02d.%02d.%02d\n", 
			gf66xx_dev->buffer[0], gf66xx_dev->buffer[1], gf66xx_dev->buffer[2], 
			gf66xx_dev->buffer[3], gf66xx_dev->buffer[4], gf66xx_dev->buffer[5],
			gf66xx_dev->buffer[7], gf66xx_dev->buffer[8], gf66xx_dev->buffer[9]);
		break;
	default:
		pr_warn("gf66xx doesn't support this command(%d)\n", cmd);
		break;
	}
	FUNC_EXIT();
	return retval;
}

static unsigned int gf66xx_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &gf66xx_dev->buf_status);
	if((gf66xx_dev->buf_status & GF66XX_BUF_STA_MASK) == GF66XX_BUF_STA_READY) {
		return (POLLIN|POLLRDNORM);
	} else {
		gf66xx_dbg("Poll no data.\n");
	}
	return 0;
}

#ifdef FW_UPDATE
static int gf66xx_fw_update_init(struct gf66xx_dev *gf66xx_dev)
{
	u8 retry_cnt = 10;
	u8 value[2];
	/*1.reset output low level and delay 2ms*/
	gpio_set_value(gf66xx_dev->reset_gpio, 0);
	usleep_range(5000, 6000);

	/*2.reset output high level to reset chip*/
	gpio_set_value(gf66xx_dev->reset_gpio, 1);

	/*3.delay 100ms*/
	msleep(100);

	while(retry_cnt--)
	{
		/*4.Hold SS51 and DSP(0x4180 == 0x0C)*/
		gf66xx_spi_write_byte(gf66xx_dev, 0x4180,0x0c);
		gf66xx_spi_read_byte(gf66xx_dev, 0x4180,&value[0]);
		gf66xx_spi_read_byte(gf66xx_dev, 0x4030,&value[1]);
		pr_info("[gf66xx_fw_update]  %s hold SS51 and DSP,0x4180=0x%x,0x4030=0x%x,retry_cnt=%d !\n",__func__,value[0] ,value[1],retry_cnt);
		if (value[0] == 0x0C)/* && value[1] == 0*/
		{
			pr_info("[gf66xx_fw_update]  %s hold SS51 and DSP successfully!\n",__func__);
			break;
		}
	}
	pr_info("[gf66xx_fw_update] Hold retry_cnt=%d\n",retry_cnt);
	/*5.enable DSP and MCU power(0x4010 == 0x00)*/
	gf66xx_spi_write_byte(gf66xx_dev, 0x4010, 0);

	return 1;
}

#endif

#ifdef GF66XX_WATCHDOG
static void gf66xx_timer_work(struct work_struct *work)
{
	unsigned char value[4];
	int ret = 0;
	struct gf66xx_dev *gf66xx_dev;
#ifdef FW_UPDATE
	unsigned char* p_fw = GF66XX_FW;
#endif
	if(work == NULL)
	{
		pr_info("[info] %s wrong work\n",__func__);
		return;
	}
	gf66xx_dev = container_of(work, struct gf66xx_dev, spi_work);

	mutex_lock(&gf66xx_dev->buf_lock);
	if(gf66xx_dev->spi->max_speed_hz != spi_speed_low){
		gf66xx_dev->spi->max_speed_hz = spi_speed_low;
		sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
	}

	gf66xx_spi_read_byte(gf66xx_dev, 0x8040,&value[0]);
	gf66xx_spi_read_byte(gf66xx_dev, 0x8000,&value[1]);
	gf66xx_spi_read_byte(gf66xx_dev, 0x8046,&value[2]);
	if(value[0] == 0xC6 && value[1] == 0x47){
		//printk("######Jason no need to kick dog 111!\n");
		gf66xx_spi_write_byte(gf66xx_dev, 0x8040, 0xAA);
	}else{
		usleep_range(1000,1500);
		gf66xx_spi_read_byte(gf66xx_dev, 0x8040,&value[0]);
		gf66xx_spi_read_byte(gf66xx_dev, 0x8000,&value[1]);
		gf66xx_spi_read_byte(gf66xx_dev, 0x8046,&value[2]);
		if(value[0] == 0xC6 && value[1] == 0x47){
			//printk("######Jason no need to kick dog 222!\n");
			gf66xx_spi_write_byte(gf66xx_dev, 0x8040, 0xAA);
		}else {
			usleep_range(1000,1500);
			gf66xx_spi_read_byte(gf66xx_dev, 0x8040,&value[0]);
			gf66xx_spi_read_byte(gf66xx_dev, 0x8000,&value[1]);
			gf66xx_spi_read_byte(gf66xx_dev, 0x8046,&value[2]);
			if(value[0] == 0xC6 && value[1] == 0x47){
				//printk("######Jason no need to kick dog 222!\n");
				gf66xx_spi_write_byte(gf66xx_dev, 0x8040, 0xAA);
			}else{
				unsigned char version[16]={0};
				pr_warn("######Jason hardware works abnormal, do reset! 0x8040 = 0x%x  0x8000 = 0x%x 0x8046 = 0x%x \n",value[0],value[1],value[2]);
				mutex_unlock(&gf66xx_dev->buf_lock);
				disable_irq(gf66xx_dev->spi->irq);
				mutex_lock(&gf66xx_dev->buf_lock);
				gf66xx_hw_reset(gf66xx_dev);
				/*ensure fingerprint module finish reset*/
				msleep(300);

				gf66xx_spi_read_byte(gf66xx_dev, 0x41e4,&value[0]);
				//pr_info("[info] %s read 0x41e4 finish value = %d\n", __func__,value[0]);
				gf66xx_spi_read_bytes(gf66xx_dev, 0x8000, 10, gf66xx_dev->buffer);
				memcpy(version, gf66xx_dev->buffer, 10);
				if((value[0] != 0xbe) || memcmp(version, GF66XX_PID, 6))
				{
					regulator_disable(gf66xx_dev->gx_power);
					msleep(10);
					ret = regulator_enable(gf66xx_dev->gx_power);
					if(ret){
						pr_info("[info] %s power on fail\n", __func__);
					}
					msleep(300);
				}

#ifdef FW_UPDATE
				if((value[0] != 0xbe) || memcmp(version, GF66XX_PID, 6))
				{
					gf66xx_spi_read_byte(gf66xx_dev, 0x41e4,&value[0]);
					if((value[0] != 0xbe) || memcmp(version, GF66XX_PID, 6))
					{
						/********************firmware update******************/
						pr_info("[info] %s firmware update start\n", __func__);
						#ifdef GF66XX_WATCHDOG
						del_timer_sync(&gf66xx_dev->gf66xx_timer);
						#endif
						gf66xx_fw_update_init(gf66xx_dev);
						ret = gf66xx_fw_update(gf66xx_dev, p_fw, FW_LENGTH);
						if(ret){
							pr_info("[info] %s gf66xx_fw_update fail\n", __func__);
						}
						gf66xx_hw_reset(gf66xx_dev);
						#ifdef GF66XX_WATCHDOG
						gf66xx_dev->gf66xx_timer.expires = jiffies + 2 * HZ;
						add_timer(&gf66xx_dev->gf66xx_timer);
						#endif
					}
				}

				/*********************update config*******************/
				//pr_info("[info] %s write config \n", __func__);
				ret = gf66xx_spi_write_byte(gf66xx_dev, 0x8040, 0xAA);
				if(ret){
					pr_info("[info] %s write 0x8040 fail\n", __func__);
				}
				if(!hw_config(gf66xx_dev)){
					pr_info("[info] %s write config fail\n", __func__);
				}
				enable_irq(gf66xx_dev->spi->irq);
#endif
			}
		}
	}
	/*if mode was changed by reset, we should set the mode back to the primary mode*/
	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS,&value[3]);
	if(value[3] != gf66xx_dev->mode)
	{
		gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, gf66xx_dev->mode);
	}
	mutex_unlock(&gf66xx_dev->buf_lock);
}
#endif


/**********************************************************************
*                                             BUFFER STATUS Register
* bit:         7            |          6        |       5        |       4        |        3-2            |    1   |    0
*        BUFFER_STA  |  IMAGE_EN  |  KEY_EN  |  KEY_STA  |  NOSIE_LEVEL  |  RES  | RES
*   BUFFER_STATUS:  1 - ready, 0 - busy
*   IMAGE_EN:   1 - image mode
*   KEY_EN:   1 - key mode
*   KEY_STATUS(only valid when KEY_EN==1): 1 - key down; 0 - key on
*
*  0xB0 - key down event
*  0xA0 - key on event
*  0xC0 - fingerprint data ready event
***********************************************************************/
u8 sig_count = 0;

static irqreturn_t gf66xx_irq(int irq, void* handle)
{
	u8 mode = 0x80;
	u8	status = 0;
	struct gf66xx_dev *gf66xx_dev = (struct gf66xx_dev *)handle;

	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, &status);
	if(!(status & GF66XX_BUF_STA_MASK)) {
		pr_err("[gf66xx_irq] Invalid IRQ = 0x%x, buffer not ready\n", status);
		return IRQ_HANDLED;
	}

	if(!(status & (GF66XX_IMAGE_MASK |GF66XX_KEY_MASK))) {
		pr_err("[gf66xx_irq] Invalid IRQ = 0x%x, image and key neither set\n", status);
		gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0 /*(status & 0x7F)*/);
		return IRQ_HANDLED;
	}else if((status & (GF66XX_IMAGE_MASK |GF66XX_KEY_MASK)) == 0x60) {
		pr_err("[gf66xx_irq] Invalid IRQ = 0x%x, image and key both set\n", status);
		gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0 /*(status & 0x7F)*/);
		return IRQ_HANDLED;
	}

	gf66xx_spi_read_byte(gf66xx_dev, GF66XX_MODE_STATUS, &mode);
	switch(mode)
	{
		/*key home events*/
		case GF66XX_KEY_MODE:
			#ifdef GF66XX_HOME
				pr_info("[gf66xx_irq]: %s home key events [0x%x]\n",__func__, status);
				if((status & GF66XX_KEY_MASK) && (status & GF66XX_BUF_STA_MASK)) {
					input_report_key(gf66xx_dev->input, KEY_FINGERPRINT, (status & GF66XX_KEY_STA)>>4);
					input_sync(gf66xx_dev->input);
				}
				gf66xx_dev->event_type = ((GF66XX_HOMEKEY_EVENT<<16) | (u32)(status & 0x00FF));
				gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0 /*(status & 0x7F)*/);
				g_lastEvent = GF66XX_HOMEKEY_EVENT;
			#endif
			break;
		/*image mode events*/
		case GF66XX_IMAGE_MODE:
			if (!g_isIdle) {
				pr_info("[gf66xx_irq]: has fp data hasn't been handled\n");
				//gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0);
				if(0xA0!=status){
					//Fix the bug that dismiss  0xA0 interrput when the fp data has not been handed by upper level,modify by ZCK 20150907.
			        gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0);
				}
				break;
			}
			//pr_info("[gf66xx_irq]: %s fp data ready [0x%x]\n", __func__, status);
			if(status & GF66XX_IMAGE_MASK){
				//pr_info("[gf66xx_irq]: fp data is ready\n");
				if (GF66XX_KEYDOWN_EVENT == g_lastEvent) {
					if(gf66xx_dev->spi->max_speed_hz != spi_speed_high){
						gf66xx_dev->spi->max_speed_hz = spi_speed_high;
						sec_spi_set_speed(gf66xx_dev, gf66xx_dev->spi->max_speed_hz);
					}

					if(gf66xx_dev->secspi_init_ok){
						mutex_lock(&gf66xx_dev->dci_lock);
						pDci->forward_int.event_type = 3;
						pDci->forward_int.status = status;
						pDci->forward_int.mode = mode;
						pDci->forward_int.notify_application = 0;
						gf66xx_secdrv_dci_cmd(SECDEV_CMD_FORWARD_INT);
						mutex_unlock(&gf66xx_dev->dci_lock);

						if(pDci->forward_int.notify_application){
							pr_info("[gf66xx_irq]: last fp data interrupt, notify upper\n");
							//TODO: notify uppler level, not needed?
							#if 1 /* GF66XX_FASYNC */
							if(gf66xx_dev->async) {
								g_lastEvent = GF66XX_FP_DATA_EVENT;
								g_isIdle = 0;
								gf66xx_dev->event_type = ((sig_count<<24) | (GF66XX_FP_DATA_EVENT<<16) | (u32)(status & 0x00FF));
								sig_count++;
								kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
							}
							#endif
						}
					}
				} else {
					pr_info("[gf66xx_irq]: without key down event, so discard the image event\n");
					gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0);
				}
				/* clear fp data interrupt in secure driver! */
			}
			else if(status & GF66XX_KEY_MASK){
				if(status & GF66XX_KEY_STA){
					pr_info("[gf66xx_irq]: key down event in image mode\n");
					if (GF66XX_KEYDOWN_EVENT != g_lastEvent) {
#if 0
						mutex_lock(&gf66xx_dev->dci_lock);
						pDci->forward_int.event_type = 1;
						pDci->forward_int.status = status;
						pDci->forward_int.mode = mode;
						pDci->forward_int.notify_application = 0;
						gf66xx_secdrv_dci_cmd(SECDEV_CMD_FORWARD_INT);
						mutex_unlock(&gf66xx_dev->dci_lock);
#endif
						//TODO: send to upper level
						#if 1 /* GF66XX_FASYNC */
						if(gf66xx_dev->async) {
							g_lastEvent = GF66XX_KEYDOWN_EVENT;
							gf66xx_dev->event_type = ((sig_count<<24) | (GF66XX_KEYDOWN_EVENT<<16) | (u32)(status & 0x00FF));
							sig_count++;
							kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
						}
						#endif
					} else {
						pr_info("[gf66xx_irq]: duplicate keydown event\n");
						gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0);
					}
				}else{
					pr_info("[gf66xx_irq]: key up event in image mode\n");
					if (GF66XX_KEYUP_EVENT != g_lastEvent) {
						if (GF66XX_FP_DATA_EVENT != g_lastEvent) {
							pr_info("[gf66xx_irq]: reset sample status when receive up event\n");
							mutex_lock(&gf66xx_dev->dci_lock);
							pDci->reset_sample.do_reset_sample = 1;
							gf66xx_secdrv_dci_cmd(SECDEV_CMD_RESET_SAMPLE);
							mutex_unlock(&gf66xx_dev->dci_lock);
							g_isIdle = 1;
						}
#if 0
						mutex_lock(&gf66xx_dev->dci_lock);
						pDci->forward_int.event_type = 2;
						pDci->forward_int.status = status;
						pDci->forward_int.mode = mode;
						pDci->forward_int.notify_application = 0;
						gf66xx_secdrv_dci_cmd(SECDEV_CMD_FORWARD_INT);
						mutex_unlock(&gf66xx_dev->dci_lock);
#endif
						//TODO: send to upper level
						#if 1 /* GF66XX_FASYNC */
						if(gf66xx_dev->async) {
							g_lastEvent = GF66XX_KEYUP_EVENT;
							gf66xx_dev->event_type = ((sig_count<<24) | (GF66XX_KEYUP_EVENT<<16) | (u32)(status & 0x00FF));
							sig_count++;
							kill_fasync(&gf66xx_dev->async, SIGIO, POLL_IN);
						}
						#endif
					} else {
						pr_info("[gf66xx_irq]: duplicate key up event\n");
						gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0);
					}
				}
				/* clear key down/on interrupt in Hal! */
				//gf66xx_spi_write_byte(gf66xx_dev, GF66XX_BUFFER_STATUS, 0x0 /*(status & 0x7F)*/);
			}else{
				pr_err("[gf66xx_irq] Invalid IRQ = 0x%x, no invalid irq\n", status);
				return IRQ_HANDLED;
			}

			break;
	}
	return IRQ_HANDLED;
}

static int gf66xx_open(struct inode *inode, struct file *filp)
{
	struct gf66xx_dev *gf66xx_dev;
	int			status = -ENXIO;
	FUNC_ENTRY();
	mutex_lock(&device_list_lock);

	list_for_each_entry(gf66xx_dev, &device_list, device_entry) {
		if(gf66xx_dev->devt == inode->i_rdev) {
			gf66xx_dbg("Found\n");
			status = 0;
			break;
		}
	}

	if(status == 0){
		mutex_lock(&gf66xx_dev->buf_lock);
		if( gf66xx_dev->buffer == NULL) {
			gf66xx_dev->buffer = kzalloc(bufsiz + GF66XX_RDATA_OFFSET, GFP_KERNEL);
			if(gf66xx_dev->buffer == NULL) {
				dev_dbg(&gf66xx_dev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		mutex_unlock(&gf66xx_dev->buf_lock);
		if(status == 0) {
			gf66xx_dev->users++;
			filp->private_data = gf66xx_dev;
			nonseekable_open(inode, filp);
			pr_info("[info] %s Succeed to open device. irq = %d\n", __func__, gf66xx_dev->spi->irq);
			/*enale irq  at the first open*/
			if(gf66xx_dev->users == 1){
				//enable irq by ioctl later
				//enable_irq(gf66xx_dev->spi->irq);
			}
		}
	} else {
		gf66xx_dbg("No device for minor %d\n", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

#ifdef GF66XX_FASYNC
static int gf66xx_fasync(int fd, struct file *filp, int mode)
{
	struct gf66xx_dev *gf66xx_dev = filp->private_data;
	int ret;

	FUNC_ENTRY();
	ret = fasync_helper(fd, filp, mode, &gf66xx_dev->async);
	FUNC_EXIT();
	gf66xx_dbg("ret = %d\n", ret);
	return ret;
}
#endif

static int gf66xx_release(struct inode *inode, struct file *filp)
{
	struct gf66xx_dev *gf66xx_dev;
	int			status = 0;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	gf66xx_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close??*/
	gf66xx_dev->users --;
	if(!gf66xx_dev->users) {
		gf66xx_dbg("disble_irq. irq = %d\n", gf66xx_dev->spi->irq);
		//disable irq by ioctl later
		//disable_irq(gf66xx_dev->spi->irq);
		#ifdef GF66XX_WATCHDOG
		/*delete the timer when device was gone*/
		//del_timer_sync(&gf66xx_dev->gf66xx_timer);
		#endif
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static const struct file_operations gf66xx_fops = {
	.owner =	THIS_MODULE,
	.write =	gf66xx_write,
	.read =		gf66xx_read,
	.unlocked_ioctl = gf66xx_ioctl,
	.open =		gf66xx_open,
	.release =	gf66xx_release,
	.poll   = gf66xx_poll,
#ifdef GF66XX_FASYNC
	.fasync = gf66xx_fasync,
#endif
};

#ifdef FW_UPDATE
static int isUpdate(struct gf66xx_dev *gf66xx_dev)
{
	unsigned char version[10];
	unsigned short ver_fw = 0;
	unsigned char* fw = GF66XX_FW;
	unsigned char fw_running = 0;
	unsigned short ver_file = 0;

	gf66xx_spi_read_byte(gf66xx_dev, 0x41e4, &fw_running);
	if(fw_running == 0xbe) {
		/*firmware running*/
		ver_file = (*(fw+12))<<8 | (*(fw+13)); //get the fw version in the i file;
		/*In case we want to upgrade to a special firmware. Such as debug firmware.*/
		if(ver_file != 0x5a5a) {
			gf66xx_spi_read_bytes(gf66xx_dev,0x8000,10,gf66xx_dev->buffer);
			memcpy(version, gf66xx_dev->buffer, 10);
			if(memcmp(version, GF66XX_PID, 6)) {
				pr_info("ChipID: 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", version[0], version[1],version[2], version[3], version[4], version[5]);
				return 1;
			}
			if((version[7]>9) || (((version[8]&0xF0)>>4)>9) || ((version[8]&0x0F)>9)) {
				pr_info("version: 7-0x%x; 8-0x%x\n", version[7], version[8]);
				return 1;
			}

			ver_fw = ((version[7]<<4)<<8) | (version[8]<<4); //get the current fw version
			if(ver_fw != ver_file) {
				/*If the running firmware is different with  the file's firmware.  do upgrade.*/
				pr_info("[gf66xx_fw_update] Current Ver: 0x%x, Upgrade to Ver: 0x%x\n", ver_fw, ver_file);
				return 1;
			} else {
				/*if the running firmware is the same with the file's firmware, do not upgrade*/
				return 0;
			}
		}
	}else {
		/*no firmware.*/
		pr_info("[gf66xx_fw_update] No running firmware. Value = 0x%x\n", fw_running);
	}
	return 1;
}
#endif

static void gf66xx_cleanup(struct gf66xx_dev	*gf66xx_dev)
{
	pr_info("[info] %s\n",__func__);
	if (gpio_is_valid(gf66xx_dev->cs_gpio))
	{
		gpio_free(gf66xx_dev->cs_gpio);
		pr_info("remove cs_gpio success\n");
	}
	if (gpio_is_valid(gf66xx_dev->irq_gpio))
	{
		gpio_free(gf66xx_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf66xx_dev->reset_gpio))
	{
		gpio_free(gf66xx_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

#ifdef GF66XX_WATCHDOG
static void gf66xx_timer_func(unsigned long arg)
{
	struct gf66xx_dev *gf66xx_dev = (struct gf66xx_dev*)arg;
	if(gf66xx_dev == NULL)
	{
		pr_info("[info] %s can't get the gf66xx_dev\n",__func__);
		return;
	}
	schedule_work(&gf66xx_dev->spi_work);
	mod_timer(&gf66xx_dev->gf66xx_timer, jiffies + 2*HZ);
}
#endif

#ifdef GF66XX_POWER_SAVING
static void gx_irq_output_config(struct gf66xx_dev	*gf66xx_dev)
{
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get_select(&gf66xx_dev->spi->dev, "gx_output_hight");
	if (IS_ERR(pinctrl))
		dev_err(&gf66xx_dev->spi->dev, "failed to set gx output");
}

static void gx_irq_output_low_config(struct gf66xx_dev	*gf66xx_dev)
{
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get_select(&gf66xx_dev->spi->dev, "gx_output_low");
	if (IS_ERR(pinctrl))
		dev_err(&gf66xx_dev->spi->dev, "failed to set gx output low");
}
#endif

static void gx_irq_gpio_config(struct gf66xx_dev	*gf66xx_dev)
{
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get_select(&gf66xx_dev->spi->dev, "gx_irq_gpio");
	if (IS_ERR(pinctrl))
		dev_err(&gf66xx_dev->spi->dev, "failed to set external interrupt");
}


static void lock_big_core(void)
{
	pm_qos_update_request_timeout(&fp_cpu_num_min_qos, BIG_CORE_NUM, HMP_BOOST_TIMEOUT);
	set_hmp_boostpulse(HMP_BOOST_TIMEOUT);
	pm_qos_update_request_timeout(&fp_cpu_freq_qos, HMP_BOOST_FREQ, HMP_BOOST_TIMEOUT);
}

static int goodix_fb_state_chg_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct gf66xx_dev *gf66xx_dev;
	struct fb_event *evdata = data;
	unsigned int blank;
	int retval = 0;


	if (val != FB_EARLY_EVENT_BLANK)
		return 0;
	pr_info("[info] %s go to the goodix_fb_state_chg_callback value = %d\n", __func__,(int)val);
	gf66xx_dev = container_of(nb,struct gf66xx_dev,fb_notifier);
	if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf66xx_dev) {
		blank = *(int *)(evdata->data);
		switch(blank) {
		#if 1
		case FB_BLANK_POWERDOWN:
			if(gf66xx_dev->device_available == 1)
			{
				gf66xx_sleep(gf66xx_dev);
				/*device unavailable*/
				gf66xx_dev->device_available = 0;
				#if 0
				/* switch spi to non-secure mode before enter sleep */
				mutex_lock(&gf66xx_dev->dci_lock);
				pDci->switch_spi.to_secure = 0x00;
				retval = gf66xx_secdrv_dci_cmd(SECDEV_CMD_SWITCH_SPI);
				mutex_unlock(&gf66xx_dev->dci_lock);
				if (retval < 0){
					pr_err("%s: failed to switch spi to normal world(%d)\n", __func__, retval);
				}
				gf66xx_dev->secspi_init_ok = 0;
				#endif
			}
			break;
		#endif
		case FB_BLANK_UNBLANK:
			if(gf66xx_dev->device_available == 0){
				//need wakeup fp chip first
				gf66xx_wakeup(gf66xx_dev);
				/*device available*/
				gf66xx_dev->device_available = 1;

				if(gf66xx_dev->mode == GF66XX_KEY_MODE) {
					msleep(60);
					//restore mode in bottom half
					//gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, gf66xx_dev->mode);
				}

				mutex_lock(&gf66xx_dev->dci_lock);
				pDci->switch_spi.to_secure = 0x01;
				retval = gf66xx_secdrv_dci_cmd(SECDEV_CMD_SWITCH_SPI);
				mutex_unlock(&gf66xx_dev->dci_lock);
				if (retval < 0){
					pr_err("%s: failed to switch spi to secure world(%d)\n", __func__, retval);
				}
				gf66xx_dev->secspi_init_ok = 1;

				gf66xx_wakeup_bottom(gf66xx_dev);
			}
			break;
		default:
			pr_info("%s defalut\n", __func__);
			break;
		}
	}
	return NOTIFY_OK;
}

static int goodix_suspend_callback(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	struct gf66xx_dev *gf66xx_dev;
	int retval = 0;

	gf66xx_dev = container_of(nb, struct gf66xx_dev, suspend_notifier);
	if(!gf66xx_dev)
		return 0;
	pr_info("[info] %s go to the goodix_suspend_callback event = %d\n", __func__,(int)event);
	switch (event) {
		case PM_SUSPEND_PREPARE:
			pr_info("[info] %s receive PM_SUSPEND_PREPARE event\n", __func__);
			/* switch spi to non-secure mode before enter sleep */
			mutex_lock(&gf66xx_dev->dci_lock);
			pDci->switch_spi.to_secure = 0x00;
			retval = gf66xx_secdrv_dci_cmd(SECDEV_CMD_SWITCH_SPI);
			mutex_unlock(&gf66xx_dev->dci_lock);
			if (retval < 0){
				pr_err("%s: failed to switch spi to normal world(%d)\n", __func__, retval);
			}
			gf66xx_dev->secspi_init_ok = 0;
			break;
		case PM_POST_SUSPEND:
			pr_info("[info] %s receive PM_POST_SUSPEND event\n", __func__);
#ifndef TEST_DAEMON
			if((probe_finish == 0) || (gf66xx_dev->device_available == 1))
			{
				pr_info("[info] %s get resume fail, [%d, %d]\n", __func__,
						probe_finish, gf66xx_dev->device_available);
				return 0;
			}
			pr_info("[info] %s receive_keyhome_event = %d\n", __func__, receive_keyhome_event);
			if(receive_keyhome_event != 1) {
				pr_info("[info] %s wakeup no by keyhome,resume function do nothing\n", __func__);
				return 0;
			} else {
				receive_keyhome_event = 0;
			}

			gf66xx_wakeup(gf66xx_dev);
			/*device available*/
			gf66xx_dev->device_available = 1;
			if(gf66xx_dev->secspi_init_ok == 0){
				if(gf66xx_dev->mode == GF66XX_KEY_MODE) {
					msleep(50);
					//restore mode in bottom half
					//gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, gf66xx_dev->mode);
				}

				mutex_lock(&gf66xx_dev->dci_lock);
				pDci->switch_spi.to_secure = 0x01;
				retval = gf66xx_secdrv_dci_cmd(SECDEV_CMD_SWITCH_SPI);
				mutex_unlock(&gf66xx_dev->dci_lock);
				if (retval < 0){
					pr_err("%s: failed to switch spi to secure world(%d)\n", __func__, retval);
				} else {
					pr_info("%s sucess to swith spi to secure world(%d)\n", __func__, retval);
				}

				gf66xx_dev->secspi_init_ok = 1;
				gf66xx_wakeup_bottom(gf66xx_dev);
#endif
			}
			break;
		default:
			break;
	}
	return 0;
}


static struct notifier_block goodix_noti_block = {
	.notifier_call = goodix_fb_state_chg_callback,
};

static struct notifier_block goodix_sus_noti_callback = {
	.notifier_call = goodix_suspend_callback,
};
static int gf66xx_probe(struct spi_device *spi)
{
	struct gf66xx_dev	*gf66xx_dev;
	unsigned long		minor;
	struct s3c64xx_spi_csinfo *cs = spi->controller_data;
	int error = -EINVAL;
	int ret;
	int	status = -EINVAL;
	//u8 version = 0;

	/* Allocate driver data */
	gf66xx_dev = kzalloc(sizeof(*gf66xx_dev), GFP_KERNEL);
	if (!gf66xx_dev){
		gf66xx_dbg("Failed to alloc memory for gf66xx device.\n");
		FUNC_EXIT();
		return -ENOMEM;
	}

	/* Initialize the driver data */
	gf66xx_dev->spi = spi;
	spin_lock_init(&gf66xx_dev->spi_lock);
	mutex_init(&gf66xx_dev->buf_lock);
	INIT_LIST_HEAD(&gf66xx_dev->device_entry);
	wake_lock_init(&gf66xx_dev->fp_wakelock, WAKE_LOCK_SUSPEND, "fp_wake_lock");
	gf66xx_dev->cs_gpio    = -EINVAL;
	gf66xx_dev->irq_gpio   = -EINVAL;
	gf66xx_dev->reset_gpio = -EINVAL;
	gf66xx_dev->device_available = 0;
	mutex_init(&gf66xx_dev->dci_lock);
	gf66xx_dev->secspi_init_ok = 0;
	gf66xx_dev->event_type = GF66XX_NOEVENT;

	/*get cs gpio*/
	gf66xx_dev->cs_gpio = of_get_named_gpio(spi->dev.of_node, "gx,spi_cs_gpio", 0);
	if (!gpio_is_valid(gf66xx_dev->cs_gpio))
	{
		dev_err(&gf66xx_dev->spi->dev,	"[ERROR] gpio_request (cs) failed.\n");
		goto err;
	}
	ret = gpio_request(gf66xx_dev->cs_gpio, "gf_cs");
	if (ret) {
		dev_err(&gf66xx_dev->spi->dev, "could not request cs gpio, %d\n",ret);
		goto err;
	}
	error = gpio_direction_output(gf66xx_dev->cs_gpio, 1);
	cs->line = (unsigned)gf66xx_dev->cs_gpio;

	/*get irq resource*/
	gf66xx_dev->irq_gpio = of_get_named_gpio(spi->dev.of_node,"gx,spi_irq_gpio",0);
	if (!gpio_is_valid(gf66xx_dev->irq_gpio))
	{
		dev_err(&spi->dev,	"[ERROR] gpio_request (irq_gpio) failed.\n");
		goto err;
	}
	ret = gpio_request(gf66xx_dev->irq_gpio, "gf66xx_irq");
	if(ret){
		dev_err(&gf66xx_dev->spi->dev, "could not request irq gpio, %d\n",ret);
		goto err;
	}
	gpio_direction_input(gf66xx_dev->irq_gpio);
	gx_irq_gpio_config(gf66xx_dev);

	/*get reset resource*/
	gf66xx_dev->reset_gpio = of_get_named_gpio(spi->dev.of_node,"gx,spi_reset_gpio",0);
	if (!gpio_is_valid(gf66xx_dev->reset_gpio))
	{
		dev_err(&gf66xx_dev->spi->dev,	"[ERROR] gpio_request (reset) failed.\n");
		goto err;
	}
	ret = gpio_request(gf66xx_dev->reset_gpio, "gf66xx_rst");
	if(ret){
		dev_err(&gf66xx_dev->spi->dev, "could not request reset gpio, %d\n",ret);
		goto err;
	}

	/*get the power*/
	gf66xx_dev->gx_power = devm_regulator_get(&spi->dev,"vdd28_fp");
	if (IS_ERR(gf66xx_dev->gx_power)) {
		dev_err(&spi->dev, "gx fingerprint chip regulator is not available.\n");
		return EINVAL;
	}
	error = regulator_enable(gf66xx_dev->gx_power);
	if(error)
	{
		pr_info("[info] %s regulator enable fail\n",__func__);
		return error;
	}

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf66xx_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf66xx_spi_class, &spi->dev, gf66xx_dev->devt,
				    gf66xx_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf66xx_dev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	if (status == 0){
	    gf66xx_dev->buffer = kzalloc(bufsiz + GF66XX_RDATA_OFFSET, GFP_KERNEL);
		if(gf66xx_dev->buffer == NULL) {
			kfree(gf66xx_dev);
			status = -ENOMEM;
			goto err;
		}
		/*register device within input system.*/
		gf66xx_dev->input = input_allocate_device();
		if(gf66xx_dev->input == NULL) {
			gf66xx_dbg("Failed to allocate input device.\n");
			status = -ENOMEM;
			kfree(gf66xx_dev->buffer);
			kfree(gf66xx_dev);
			goto err;
		}

		__set_bit(EV_KEY, gf66xx_dev->input->evbit);
		__set_bit(KEY_FINGERPRINT, gf66xx_dev->input->keybit);

		gf66xx_dev->input->name = "gpio-keys";
		if(input_register_device(gf66xx_dev->input)) {
			gf66xx_dbg("Failed to register input device.\n");
		}

		/*SPI parameters.*/
		gf66xx_dev->spi->mode = SPI_MODE_0;
		//gf66xx_dev->software_available = 1;	/*image  mode (default) */
		gf66xx_dev->spi->max_speed_hz = spi_speed_low;
		gf66xx_dev->spi->irq = gpio_to_irq(gf66xx_dev->irq_gpio);
		gf66xx_dev->spi->bits_per_word = 8;
		spi_setup(gf66xx_dev->spi);
		gf66xx_dev->fb_notifier = goodix_noti_block;
		gf66xx_dev->suspend_notifier = goodix_sus_noti_callback;
		spi_set_drvdata(spi, gf66xx_dev);
		gf66xx_hw_reset(gf66xx_dev);

		pm_qos_add_request(&fp_cpu_num_min_qos, PM_QOS_CPU_NUM_MIN, 0);	/*request new qos*/
		pm_qos_add_request(&fp_cpu_freq_qos, PM_QOS_CPU_FREQ_MIN, 0);	/*request new qos*/
	}
	else
		goto err;

	probe_finish = 1;

	fp_get(FP_GOODIX);
	pr_info("[info]%s probe finished, normal driver version: %s\n", __func__, GF66XX_SPI_VERSION);
	return 0;
err:
	device_destroy(gf66xx_spi_class,gf66xx_dev->devt);
	gf66xx_cleanup(gf66xx_dev);
	kfree(gf66xx_dev);
	return status;
}

static int gf66xx_remove(struct spi_device *spi)
{
	struct gf66xx_dev	*gf66xx_dev = spi_get_drvdata(spi);
	FUNC_ENTRY();

	/* make sure ops on existing fds can abort cleanly */
	if(gf66xx_dev->spi->irq) {
		free_irq(gf66xx_dev->spi->irq, gf66xx_dev);
	}
	spin_lock_irq(&gf66xx_dev->spi_lock);
	gf66xx_dev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&gf66xx_dev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf66xx_dev->device_entry);
	device_destroy(gf66xx_spi_class, gf66xx_dev->devt);
	clear_bit(MINOR(gf66xx_dev->devt), minors);
	if (gf66xx_dev->users == 0) {
		if(gf66xx_dev->input != NULL)
			input_unregister_device(gf66xx_dev->input);

		if(gf66xx_dev->buffer != NULL)
			kfree(gf66xx_dev->buffer);
		kfree(gf66xx_dev);
	}
	mutex_unlock(&device_list_lock);

	FUNC_EXIT();
	return 0;
}

static int gf66xx_suspend(struct spi_device *spi, pm_message_t msg)
{
#if 0
	//int retval = 0;

	struct gf66xx_dev *gf66xx_dev = spi_get_drvdata(spi);
	if((probe_finish == 0) || (gf66xx_dev->device_available == 0))
	{
		pr_info("[info] %s get suspend fail\n", __func__);
		return 0;
	}
	//gf66xx_sleep(gf66xx_dev);
	/*device unavailable*/
	gf66xx_dev->device_available = 0;
#if 0
	mutex_lock(&gf66xx_dev->dci_lock);
	pDci->switch_spi.to_secure = 0x00;
	retval = gf66xx_secdrv_dci_cmd(SECDEV_CMD_SWITCH_SPI);
	mutex_unlock(&gf66xx_dev->dci_lock);
	if (retval < 0){
		pr_err("%s: failed to switch spi to normal world(%d)\n", __func__, retval);
	}
#endif
	gf66xx_dev->secspi_init_ok = 0;
#endif
	return 0;
}

#ifdef TEST_DAEMON
static int test_daemon(void *data)
{
	int retval = 0;
	struct gf66xx_dev *gf66xx_dev = (struct gf66xx_dev *)data;

	pr_info("[DAEMON TEST] %s daemon test\n", __func__);

	if(gf66xx_dev->secspi_init_ok == 0){
		if(gf66xx_dev->mode == GF66XX_KEY_MODE) {
			msleep(50);
			//restore mode in bottom half
			//gf66xx_spi_write_byte(gf66xx_dev, GF66XX_MODE_STATUS, gf66xx_dev->mode);
		}

		mutex_lock(&gf66xx_dev->dci_lock);
		pDci->switch_spi.to_secure = 0x01;
		retval = gf66xx_secdrv_dci_cmd(SECDEV_CMD_SWITCH_SPI);
		mutex_unlock(&gf66xx_dev->dci_lock);
		if (retval < 0){
			pr_err("%s: failed to switch spi to secure world(%d)\n", __func__, retval);
		} else {
			pr_info("%s sucess to swith spi to secure world(%d)\n", __func__, retval);
		}

		gf66xx_dev->secspi_init_ok = 1;
		gf66xx_wakeup_bottom(gf66xx_dev);
	}

	do_exit(0);
	return 0;
}
#endif

static int gf66xx_resume(struct spi_device *spi)
{
#ifdef TEST_DAEMON
	struct gf66xx_dev *gf66xx_dev = NULL;

	pr_info("[info] %s happend\n", __func__);
	gf66xx_dev = spi_get_drvdata(spi);
	if((probe_finish == 0) || (gf66xx_dev->device_available == 1))
	{
		pr_info("[info] %s get resume fail, [%d, %d]\n", __func__,
				probe_finish, gf66xx_dev->device_available);
		return 0;
	}
	pr_info("[info] %s receive_keyhome_event = %d\n", __func__, receive_keyhome_event);
	if(receive_keyhome_event != 1) {
		pr_info("[info] %s wakeup no by keyhome,resume function do nothing\n", __func__);
		return 0;
	} else {
		receive_keyhome_event = 0;
	}

	gf66xx_wakeup(gf66xx_dev);
	/*device available*/
	gf66xx_dev->device_available = 1;

	/*test for daemon*/
	gf66xx_dev->daemon_task = kthread_run(test_daemon, gf66xx_dev, "test_daemon");
	#endif
	return 0;
}

static struct of_device_id gx_of_match_table[] = {
	{
		.compatible = "gx_fingerprint",
	},
	{},
};


static struct spi_driver gf66xx_spi_driver = {
	.driver = {
		.name =		SPI_DEV_NAME,
		.owner =	THIS_MODULE,
		.of_match_table = gx_of_match_table,
	},
	.probe =	gf66xx_probe,
	.remove = 	gf66xx_remove,
	.suspend = gf66xx_suspend,
	.resume = gf66xx_resume,
};

static int __init gf66xx_init(void)
{
	int status;
	FUNC_ENTRY();

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf66xx_fops);
	if (status < 0){
		gf66xx_dbg("Failed to register char device!\n");
		FUNC_EXIT();
		return status;
	}
	gf66xx_spi_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf66xx_spi_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
		gf66xx_dbg("Failed to create class.\n");
		FUNC_EXIT();
		return PTR_ERR(gf66xx_spi_class);
	}
	status = spi_register_driver(&gf66xx_spi_driver);
	if (status < 0) {
		class_destroy(gf66xx_spi_class);
		unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
		gf66xx_dbg("Failed to register SPI driver.\n");
	}
	FUNC_EXIT();
	return status;
}
module_init(gf66xx_init);

static void __exit gf66xx_exit(void)
{
    FUNC_ENTRY();
	spi_unregister_driver(&gf66xx_spi_driver);
	class_destroy(gf66xx_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, gf66xx_spi_driver.driver.name);
	FUNC_EXIT();
}
module_exit(gf66xx_exit);

MODULE_AUTHOR("Jiangtao Yi <yijiangtao@goodix.com>; Ke Yang<yangke@goodix.com>");
MODULE_DESCRIPTION("Goodix Fingerprint chip SPI TEE driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf66xx-spi");
