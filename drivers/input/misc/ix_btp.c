/* IX BTP Sensor Driver
 *
 * Copyright (c) 2014 Crucialsoft Fingerprint <mskim@crucialtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
//#include <linux/earlysuspend.h>
#include <linux/spi/ix_btp.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/platform_data/spi-s3c64xx.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <mach/hardware.h>
//#define CS_CHANGE_VALUE	1
#define CS_CHANGE_VALUE	0	/*wdn*/
//#define NODIM
//#define __FULL_LINE__

/****************************************************************************
ix driver constants
*****************************************************************************/
#define IX_MAJOR                   	235

#ifdef __FULL_LINE__
#define IX_SPI_CLOCK_SPEED         	(10 * 1000 * 1000)
#else
#define IX_SPI_CLOCK_SPEED         	(10 * 1000 * 1000)
#endif
#define IX_DEFAULT_IRQ_TIMEOUT     	(100 * HZ / 1000)

//#define IX_IMAGE_BUFFER_SIZE       	(160 * 8 * 200)
#define IX_SENSOR_WIDTH				112U
#define IX_SENSOR_HEIGHT			88U
#define IX_DEFAULT_READ_LINE		8
#define IX_MAX_FRAME				25
#define IX_FRAME_SIZE				(IX_SENSOR_WIDTH * IX_SENSOR_HEIGHT)
#define IX_IMAGE_BUFFER_SIZE       	(IX_FRAME_SIZE * IX_MAX_FRAME)


#define IX_FINGER_DETECT_THRESH		5000
#define IX_FINGER_DETECT_COUNT		5
#define IX_ADC_COUNT				3

#define IX_DEV_NAME                	"btp"
#define IX_CLASS_NAME              	"ixsensor"
#define IX_WORKER_THREAD_NAME      	"ixworker"


/***************************************************************************
ix sensor hardware id
***************************************************************************/
#define IX_HW_ID             	0x50


/****************************************************************************
ix sensor commands
*****************************************************************************/
#define IX_CMD_SREST            0x01	/* Software reset */
#define IX_CMD_CMODE_EN	    	0x02	/* Go to capture mode */
#define IX_CMD_DMODE_EN	    	0x04	/* Go to finger detection mode */
#define IX_CMD_STBY_EN         	0x05	/* Go to standby mode */
#define IX_CMD_START_C         	0x10	/* Capture image start */
#define IX_CMD_READ_C_PLUS     	0x16	/* Read and capture image */
#define IX_CMD_READ_M		    0x17	/* Only read image */
#define IX_CMD_INT_C			0x0C	/* Clear interrupt resister */
#define IX_CMD_INT_R			0x0E	/* Read interrupt resister */


/*******************************************************************************
ix sensor register
********************************************************************************/
#define IX_REG_CHIP_ID              	0xA0
#define IX_REG_REVISION_NUMBER			0xA1
#define IX_REG_PGA_GAIN              	0xB0
#define IX_REG_PGA_OFFSET				0xB2
#define IX_REG_FINGER_UP_THRESHOLD		0x38
#define IX_REG_FINGER_DOWN_THRESHOLD	0x39
#define IX_REG_CP1_SET3					0xAC


/*********************************************************************************
ix sensor register default value
**********************************************************************************/

#define IX_DEFAULT_PGA_GAIN            	0x2f
#define IX_DEFAULT_PGA_OFFSET			0x61
#define IX_DEFAULT_CP1_SET3				0x22




#define IX_DEFAULT_FINGER_UP_THRESH		0xd9
#define IX_DEFAULT_FINGER_DOWN_THRESH	0x80


/*************************************************************************************
ix sensor irq flag
**************************************************************************************/
#define IX_IRQ_REBOOT      	            0xFF
#define IX_IRQ_CMD_DONE                	(1 << 7u)
#define IX_IRQ_AUTO_PARTIAL_DONE		(1 << 6u)
#define IX_IRQ_FINGER_DOWN     	        (1 << 1u)
#define IX_IRQ_FINGER_UP        	    (1 << 0u)


/****************************************************************************************
ix sensor size
*****************************************************************************************/
#define IX_SENSOR_SIZE_W_160           160
#define IX_SENSOR_SIZE_W_112           112
#define IX_SENSOR_SIZE_H_140           140
#define IX_SENSOR_SIZE_H_88            88
#define IX_SENSOR_SIZE_H_56            56
#define IX_SENSOR_SIZE_H_24            24
#define IX_SENSOR_SIZE_H_16            16
#define IX_SENSOR_SIZE_H_8             8


/**************************************************************************************
 ix i/o control
 **************************************************************************************/
#define IX_IOCTL_MAGIC_NO         	 0xFC
#define IX_IOCTL_START_CAPTURE	    _IO(IX_IOCTL_MAGIC_NO, 0)
#define IX_IOCTL_ABORT_CAPTURE	    _IO(IX_IOCTL_MAGIC_NO, 1)
#define IX_IOCTL_CAPTURE_SINGLE    	_IOW(IX_IOCTL_MAGIC_NO, 2, int)
#define IX_IOCTL_MAX_PERFORMANCE    	_IO(IX_IOCTL_MAGIC_NO, 4)
#define IX_IOCTL_DEFAULT_PERFORMANCE	_IO(IX_IOCTL_MAGIC_NO, 5)


/*************************************************************************************
ix spi read/write command
**************************************************************************************/
#define IX_CMD_SIZE						3
#define IX_SPI_READ_COMMAND				0xA8
#define IX_SPI_WRITE_COMMAND			0xA9

enum {
	FNGR_ST_NONE = 0,
	FNGR_ST_DETECTED,
	FNGR_ST_LOST,
};

enum {
	IX_THREAD_IDLE_MODE = 0,
	IX_THREAD_CAPTURE_MODE,
	IX_THREAD_EXIT,
};

enum {
	IX_MODE_CAPTURE = 0,
	IX_MODE_FINGERUP,
	IX_MODE_FINGERDOWN,
	IX_MODE_STANDBY,
};

#define ceil(x, y) \
	({ unsigned long __x = (x), __y = (y); (__x + __y - 1) / __y; })

//#define DEBUG_IX
#ifdef DEBUG_IX
#define DEBUG_PRINT(fmt,...) printk(fmt,##__VA_ARGS__);
#else
#define DEBUG_PRINT(fmt,...) do{}while(0)
#endif


/****************************************************************************
global variables
*****************************************************************************/
static int ix_device_count;


/*****************************************************************************
ix data types
******************************************************************************/
struct ix_info_S{
	u8 chipID;
	u8 revNO;
	int imageWidth;
	int imageHeight;
	int imageSize;
	int frameMax;
	int refImageSum[IX_ADC_COUNT][IX_FINGER_DETECT_COUNT];
	u8 *refImage;
};

struct ix_thread_task {
	int mode;
	int should_stop;
	struct semaphore sem_idle;
	wait_queue_head_t wait_job;
	struct task_struct *thread;
};

struct ix_adc_setup {
    u8 gain;
    u8 offset;
	u8 level;
};

struct ix_reg_setup {
    u8 finger_down_threshold;
    u8 finger_up_threshold;
};

struct ix_diag {
	u32 selftest;
	u32 sensortest;
	u32 capture_time;
	u32 frames_captured;
	u32 frames_stored;
	u32 finger_threshold;
	u32 finger_status;
	u32 module_message;
};

struct ix_data {
	struct spi_device *spi;
	struct class *class;
	struct device *device;
	struct cdev cdev;
	struct semaphore mutex;
	struct ix_thread_task thread_task;
	struct ix_adc_setup adc_setup;
	struct ix_reg_setup reg_setup;
	struct ix_diag diag;
	struct ix_info_S info;
	dev_t devno;
	wait_queue_head_t waiting_data_avail;
	wait_queue_head_t waiting_interrupt_return;
	u32 reset_gpio;
	u32 irq_gpio;
	u32 cs_gpio;
	u32 irq;
	u32 data_offset;
	u32 avail_data;
	u32 current_frame;
	u32 current_adc_table;
	bool capture_done;
	int interrupt_done;
	int pxl_sum[IX_FINGER_DETECT_COUNT];
	u8 *huge_buffer;
	bool isSleep;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
	bool isSleep;
#endif
	struct notifier_block notifier;

};

struct ix_attribute {
	struct device_attribute attr;
	size_t offset;
};


/************************************************************************
ix init value
*************************************************************************/
const static u8 adc_level[IX_ADC_COUNT]	= {2, 10, 4};//{9, 12, 15, 6, 3};
const static u8 adc_offset[IX_ADC_COUNT] = {94, 99, 104};

static unsigned char init_seq_value[] = {
/* Address, Value,  Delay(*100us) */
	0x20,	0x86,	0,
	0x21,	0x00,	0,
	0x22,	0x3f,	0,
	0x23,	0xff,	0,
	0x9a,	0x14,	0,
	0x9b,	0xb7,	0,
	0x9d,	0x01,	0,
#ifdef __FULL_LINE__
	0x9e,	0x08,	0,
#else
	0x9e,	0x88,	0,
#endif
	0xa3,	0x15,	0,
	0xa4,	0xf2,	0,
	0xa6,	0x4f,	0,
	0xaf,	0x3f,	0,
	//0xb0,	0x3f,	0,
	0xb1,	0x8a,	0,
	//0xb2,	0x61,	0,
	0xb3,	0x00,	0,
	0xb7,	0x03,	0,
	0xbd,	0x22,	0,
	0xc9,	0x00,	0,
	0x1c,	0xd2,	0,
	0x1d,	0x77,	1,
	0x1f,	0x90,	1,
	0x31,	0x40,	0,
	0x3A,	0x00,	0,
	0x3D,	0x10,	0,
	0x0F,	0xA3,	0,
	0xa9,	0x10,	0,
	0xa9,	0x13,	0,
	//0xac,	0x22,	0,
	0xaa,	0x04,	100,
};

#if 0
static unsigned char init_seq_down_capture[] = {
	0x3F,	0x02,	0,  /* finger down detect */
	0x0C,	0x00,	0,
};

static unsigned char init_seq_up_capture[] = {
	0x3F,	0x01,	0,  /* finger up detect */
	0x0C,	0x00,	0,
};
#endif

static unsigned char init_seq_down[] = {
	0x3F,	0x02,	0,  /* finger down detect */
	0x9D,	0x00,	0,
	0x9E,	0x80,	0,
	0x0C,	0x00,	0,
};

static unsigned char init_seq_up[] = {
	0x3F,	0x01,	0,  /* finger up detect */
	0x9D,	0x00,	0,
	0x9E,	0x80,	0,
	0x0C,	0x00,	0,
};


/**************************************************************************
function prototypes
**************************************************************************/
//static int __init ix_init(void);
//static void __exit ix_exit(void);
//static int __devinit ix_probe(struct spi_device *spi);
static int ix_probe(struct spi_device *spi);
//static int __devexit ix_remove(struct spi_device *spi);
static int ix_remove(struct spi_device *spi);

static int ix_suspend(struct device *dev);
static int ix_resume(struct device *dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ix_early_suspend(struct early_suspend *h);
static void ix_late_resume(struct early_suspend *h);
#endif

static int ix_open(struct inode *inode, struct file *file);
static int ix_release(struct inode *inode, struct file *file);
static ssize_t ix_read(struct file *file, char *buff, size_t count, loff_t *ppos);
static ssize_t ix_write(struct file *file, const char *buff, size_t count, loff_t *ppos);
static long ix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static unsigned int ix_poll(struct file *file, poll_table *wait);

static ssize_t ix_show_attr_adc_setup(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ix_store_attr_adc_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ix_show_attr_diag(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ix_store_attr_diag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ix_show_attr_reg_setup(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ix_store_attr_reg_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static int ix_init_param(struct ix_data *ix);
static void ix_set_info(struct ix_data *ix, int mode);
static int ix_reset(struct ix_data *ix);
static void ix_refresh(struct ix_data *ix);

static int ix_spi_read_reg(struct ix_data *ix, u8 addr, u8 *rx);
static int ix_spi_write_reg(struct ix_data *ix, u8 addr, u8 value);
static int ix_spi_read_image(struct ix_data *ix);
static int ix_spi_read_wait_irq(struct ix_data *ix, u8 *irq);
static int ix_wait_for_irq(struct ix_data *ix, int timeout);

static int ix_get_reference(struct ix_data *ix, bool isInnerPath);
static u8  ix_finger_detect(struct ix_data *ix);
static void ix_image_calibration(struct ix_data *ix);
static int ix_mode_change(struct ix_data *ix, int mode);

static int threadfn(void *_ix);
static int ix_start_thread(struct ix_data *ix, int mode);
static int ix_thread_goto_idle(struct ix_data *ix);
static int ix_auto_adc_ctrl(struct ix_data *ix, int count);
static int ix_capture_task(struct ix_data *ix);
static void ix_start_capture(struct ix_data *ix);
static int ix_selftest_short(struct ix_data *ix);
static int ix_sensor_test(struct ix_data *ix);

static void ix_set_performance(struct ix_data *ix, bool isMax);

/*******************************************************************************
External interface
********************************************************************************/
static const struct dev_pm_ops ix_pm = {
	.suspend = ix_suspend,
	.resume = ix_resume
};



static const struct file_operations ix_fops = {
	.owner	= THIS_MODULE,
	.open	= ix_open,
	.write	= ix_write,
	.read	= ix_read,
	.release	= ix_release,
	.poll	= ix_poll,
	.unlocked_ioctl	= ix_ioctl
};

#define IX_ATTR(__grp, __field, __mode)				\
{													\
	.attr = __ATTR(__field, (__mode),				\
			ix_show_attr_##__grp,					\
			ix_store_attr_##__grp),					\
	.offset = offsetof(struct ix_##__grp, __field)	\
}

#define IX_DEV_ATTR(_grp, _field, _mode)			\
	struct ix_attribute ix_attr_##_field = IX_ATTR(_grp, _field, (_mode))

#define ADC_SETUP_MODE (S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH)
static IX_DEV_ATTR(adc_setup, gain,		ADC_SETUP_MODE);
static IX_DEV_ATTR(adc_setup, offset,	ADC_SETUP_MODE);
static IX_DEV_ATTR(adc_setup, level,	ADC_SETUP_MODE);

static struct attribute *ix_adc_attrs[] = {
	&ix_attr_gain.attr.attr,
	&ix_attr_offset.attr.attr,
	&ix_attr_level.attr.attr,
	NULL
};

static const struct attribute_group ix_adc_attr_group = {
	.attrs = ix_adc_attrs,
	.name = "adc_setup"
};

#define DIAG_MODE (S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH)
static IX_DEV_ATTR(diag, selftest,			DIAG_MODE);
static IX_DEV_ATTR(diag, sensortest,		DIAG_MODE);
static IX_DEV_ATTR(diag, capture_time,		DIAG_MODE);
static IX_DEV_ATTR(diag, frames_captured,	DIAG_MODE);
static IX_DEV_ATTR(diag, frames_stored,		DIAG_MODE);
static IX_DEV_ATTR(diag, finger_threshold,	DIAG_MODE);
static IX_DEV_ATTR(diag, finger_status,		DIAG_MODE);
static IX_DEV_ATTR(diag, module_message,		DIAG_MODE);

static struct attribute *ix_diag_attrs[] = {
	&ix_attr_selftest.attr.attr,
	&ix_attr_sensortest.attr.attr,
	&ix_attr_capture_time.attr.attr,
	&ix_attr_frames_captured.attr.attr,
	&ix_attr_frames_stored.attr.attr,
	&ix_attr_finger_threshold.attr.attr,
	&ix_attr_finger_status.attr.attr,
	&ix_attr_module_message.attr.attr,
	NULL
};

static const struct attribute_group ix_diag_attr_group = {
	.attrs = ix_diag_attrs,
	.name = "diag"
};

#define REG_SETUP_MODE (S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH)
static IX_DEV_ATTR(reg_setup, finger_down_threshold,	REG_SETUP_MODE);
static IX_DEV_ATTR(reg_setup, finger_up_threshold,		REG_SETUP_MODE);

static struct attribute *ix_reg_attrs[] = {
	&ix_attr_finger_down_threshold.attr.attr,
	&ix_attr_finger_up_threshold.attr.attr,
	NULL
};

static const struct attribute_group ix_reg_attr_group = {
	.attrs = ix_reg_attrs,
	.name = "reg_setup"
};

static ssize_t ix_show_attr_adc_setup(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 *target;
	struct ix_data *ix;
	struct ix_attribute *ix_attr;
	ix = dev_get_drvdata(dev);
	ix_attr = container_of(attr, struct ix_attribute, attr);

	target = ((u8 *)&ix->adc_setup) + ix_attr->offset;

	if(ix_attr->offset == offsetof(struct ix_adc_setup, level))
	{
		ix->adc_setup.level = (ix->adc_setup.level & 0x0f) | 0x20;
	}

	return scnprintf(buf, PAGE_SIZE, "%s: %i\n", attr->attr.name, *target);
}


static ssize_t ix_store_attr_adc_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 *target;
	u8 tmp;
	struct ix_data *ix;
	struct ix_attribute *ix_attr;
	ix = dev_get_drvdata(dev);
	ix_attr = container_of(attr, struct ix_attribute, attr);

	if ((sscanf(buf, "%hhu", &tmp)) <= 0)
		return -EINVAL;

	target = ((u8 *)&ix->adc_setup) + ix_attr->offset;
	*target = tmp;

	return strnlen(buf, count);
}


static ssize_t ix_show_attr_diag(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ix_data *ix;
	struct ix_attribute *ix_attr;

	ix = dev_get_drvdata(dev);
	ix_attr = container_of(attr, struct ix_attribute, attr);

	if(ix_attr->offset == offsetof(struct ix_diag, selftest))
	{
		ix_selftest_short(ix);
		return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.selftest);
	}

	if(ix_attr->offset == offsetof(struct ix_diag, sensortest))
	{
		ix_sensor_test(ix);
		return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.sensortest);
	}

	if(ix_attr->offset == offsetof(struct ix_diag, capture_time))
	{
		return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.capture_time);
	}

	if(ix_attr->offset == offsetof(struct ix_diag, frames_captured))
	{
		return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.frames_captured);
	}

	if(ix_attr->offset == offsetof(struct ix_diag, frames_stored))
	{
		return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.frames_stored);
	}

	if(ix_attr->offset == offsetof(struct ix_diag, finger_threshold))
	{
		return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.finger_threshold);
	}

	if(ix_attr->offset == offsetof(struct ix_diag, finger_status))
	{
		return scnprintf(buf, PAGE_SIZE, "%i\n", ix->diag.finger_status);
	}
	if(ix_attr->offset == offsetof(struct ix_diag, module_message))
	{
		/*fingerprint module message description*/
	}

	return -ENOENT;
}


static ssize_t ix_store_attr_diag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u32 *target;
	u32 tmp;
	struct ix_data *ix;
	struct ix_attribute *ix_attr;
	ix = dev_get_drvdata(dev);
	ix_attr = container_of(attr, struct ix_attribute, attr);

	if ((sscanf(buf, "%d", &tmp)) <= 0)
		return -EINVAL;

	target = ((void *)&ix->diag) + ix_attr->offset;
	*target = tmp;

	return strnlen(buf, count);
	//return -EPERM;
}


static ssize_t ix_show_attr_reg_setup(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 *target;
	struct ix_data *ix;
	struct ix_attribute *ix_attr;
	ix = dev_get_drvdata(dev);
	ix_attr = container_of(attr, struct ix_attribute, attr);

	target = ((u8 *)&ix->reg_setup) + ix_attr->offset;
	return scnprintf(buf, PAGE_SIZE, "%s: %i\n", attr->attr.name, *target);
}


static ssize_t ix_store_attr_reg_setup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 *target;
	u8 tmp;

	struct ix_data *ix;
	struct ix_attribute *ix_attr;
	ix = dev_get_drvdata(dev);
	ix_attr = container_of(attr, struct ix_attribute, attr);

	if ((sscanf(buf, "%hhu", &tmp)) <= 0)
		return -EINVAL;

	target = ((u8 *)&ix->reg_setup) + ix_attr->offset;
	*target = tmp;

	return strnlen(buf, count);
}


static void ix_set_info(struct ix_data *ix, int mode)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix->info.imageWidth = IX_SENSOR_SIZE_W_112;
	ix->info.imageHeight = IX_SENSOR_SIZE_H_88;
	ix->info.imageSize = ix->info.imageWidth * ix->info.imageHeight;
	ix->info.frameMax = IX_MAX_FRAME;
}

static int ix_init_param(struct ix_data *ix)
{
	int error;
	int i, j;
	u8 rx;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	for(i=0; i<ARRAY_SIZE(init_seq_value); i+=3)
	{
		error = ix_spi_write_reg(ix, init_seq_value[i], init_seq_value[i+1]);
		if(error)
			return error;

		error = ix_spi_read_reg(ix, init_seq_value[i], &rx);
		if(error)
			return error;

		//DEBUG_PRINT(KERN_INFO "[INFO] init register 0x%02x[%02xh]\n", init_seq_value[i], rx);

		if(init_seq_value[i+2] > 0)
		{
			for(j=0; j<init_seq_value[i+2]; j++)
				udelay(100);
		}
	}

	/* setting adc */
	error = ix_spi_write_reg(ix, IX_REG_PGA_GAIN, ix->adc_setup.gain);
	if(error)
		return error;

	error = ix_spi_read_reg(ix, IX_REG_PGA_GAIN, &rx);
	if(error)
		return error;
	DEBUG_PRINT(KERN_INFO "[INFO] %s 0x%02x[%02xh]\n", __func__, IX_REG_PGA_GAIN, rx);

	error = ix_spi_write_reg(ix, IX_REG_PGA_OFFSET, ix->adc_setup.offset);
	if(error)
		return error;

	error = ix_spi_read_reg(ix, IX_REG_PGA_OFFSET, &rx);
	if(error)
		return error;
	DEBUG_PRINT(KERN_INFO "[INFO] %s 0x%02x[%02xh]\n", __func__, IX_REG_PGA_OFFSET, rx);

	error = ix_spi_write_reg(ix, IX_REG_CP1_SET3, ix->adc_setup.level);
	if(error)
		return error;

	error = ix_spi_read_reg(ix, IX_REG_CP1_SET3, &rx);
	if(error)
		return error;
	DEBUG_PRINT(KERN_INFO "[INFO] %s 0x%02x[%02xh]\n", __func__, IX_REG_CP1_SET3, rx);

	/* setting register */
	error = ix_spi_write_reg(ix, IX_REG_FINGER_UP_THRESHOLD, ix->reg_setup.finger_up_threshold);
	if(error)
		return error;

	error = ix_spi_read_reg(ix, IX_REG_FINGER_UP_THRESHOLD, &rx);
	if(error)
		return error;
	DEBUG_PRINT(KERN_INFO "[INFO] %s 0x%02x[%02xh]\n", __func__, IX_REG_FINGER_UP_THRESHOLD, rx);

	error = ix_spi_write_reg(ix, IX_REG_FINGER_DOWN_THRESHOLD, ix->reg_setup.finger_down_threshold);
	if(error)
		return error;

	error = ix_spi_read_reg(ix, IX_REG_FINGER_DOWN_THRESHOLD, &rx);
	if(error)
		return error;
	DEBUG_PRINT(KERN_INFO "[INFO] %s 0x%02x[%02xh]\n", __func__, IX_REG_FINGER_DOWN_THRESHOLD, rx);

	return 0;
}

static int ix_reset(struct ix_data *ix)
{
	int error;
	u8 rx;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	error = ix_spi_write_reg(ix, IX_CMD_SREST, 0x00);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_SREST failed.\n");
		return error;
	}

	udelay(500);

	disable_irq(ix->irq);
	ix->interrupt_done = 0;
	enable_irq(ix->irq);

	error = ix_spi_read_reg(ix, IX_CMD_INT_R, &rx);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_R failed.\n");
		return error;
	}
	if(rx != IX_IRQ_REBOOT)
	{
		dev_err(&ix->spi->dev, "[ERROR] hardware id mismatch : %02x expected %02x\n", rx, IX_IRQ_REBOOT);
		return -EIO;
	}

	error = ix_spi_write_reg(ix, IX_CMD_INT_C, 0x00);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_C failed.\n");
		return error;
	}

	error = ix_init_param(ix);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] ix_init_param failed.\n");
		return error;
	}

	return 0;
}

static void ix_refresh(struct ix_data *ix)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix_thread_goto_idle(ix);
	ix->avail_data = 0;
	ix->current_frame = 0;
	ix->data_offset = 0;
	ix->capture_done = false;
}

static int ix_spi_read_reg(struct ix_data *ix, u8 addr, u8 *rx)
{
	int error;
	u8 tx[3];

	struct spi_message m;
	struct spi_transfer t = {
		.cs_change = CS_CHANGE_VALUE,
		.delay_usecs = 0,
		.speed_hz = IX_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = tx,
		.len = 3,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	tx[0] = IX_SPI_READ_COMMAND;
	tx[1] = addr;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	error = spi_sync(ix->spi, &m);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
		return error;
	}

	*rx = tx[2];
	//pr_info("[INFO] tx[0] = %d  tx[1] = %d  tx[2] = %d\n",tx[0],tx[1],tx[2]);
	return 0;
}

static int ix_spi_write_reg(struct ix_data *ix, u8 addr, u8 value)
{
	int error;
	u8 tx[3];

	struct spi_message m;
	struct spi_transfer t = {
		.cs_change = CS_CHANGE_VALUE,
		.delay_usecs = 0,
		.speed_hz = IX_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = NULL,
		.len = 3,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	tx[0] = IX_SPI_WRITE_COMMAND;
	tx[1] = addr;
	tx[2] = value;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	error = spi_sync(ix->spi, &m);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
		return error;
	}

	return 0;
}


#ifdef __FULL_LINE__
static int ix_spi_read_image(struct ix_data *ix)
{
	int error;
	u8 tx[IX_CMD_SIZE];
	int pos;
	u8 rx;

	struct spi_message m;
	struct spi_transfer t;

	memset(&t, 0, sizeof(struct spi_transfer));

	if(ix->current_frame >= ix->info.frameMax)
	{
		ix->current_frame = 0;
	}

	tx[0] = IX_SPI_READ_COMMAND;
	tx[1] = IX_CMD_READ_M;
	tx[2] = 0x00;	/* dummy byte */

	t.cs_change = CS_CHANGE_VALUE,
	t.delay_usecs = 0,
	t.speed_hz = IX_SPI_CLOCK_SPEED,
	t.tx_buf = tx,
	t.rx_buf = ix->huge_buffer + (ix->current_frame * (ix->info.imageSize)),
	t.len = IX_CMD_SIZE + ix->info.imageSize,
	t.tx_dma = 0,
	t.rx_dma = 0,
	t.bits_per_word = 0,

	error = ix_spi_write_reg(ix, IX_CMD_START_C, 0x00);
	if(error)
		return error;

#if 0
	udelay(1500);
#else
	error = ix_spi_read_wait_irq(ix, &rx);
	if(error)
		return error;

	if(!(rx & IX_IRQ_AUTO_PARTIAL_DONE))
	{
		dev_err(&ix->spi->dev, "[ERROR] irq : %x expected %x\n", rx, IX_IRQ_AUTO_PARTIAL_DONE);
		return -EINTR;
	}
#endif

	error = ix_spi_write_reg(ix, IX_CMD_READ_C_PLUS, 0x00);
	if(error)
		return error;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	error = spi_sync(ix->spi, &m);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
		return error;
	}

	pos = ix->current_frame * ix->info.imageSize;
	memcpy(&ix->huge_buffer[pos], &ix->huge_buffer[pos + IX_CMD_SIZE], ix->info.imageSize);

	//just workaround
	ix->huge_buffer[pos + ix->info.imageWidth * 8] = ix->huge_buffer[pos + ix->info.imageWidth * 16];
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 16], &ix->huge_buffer[pos + ix->info.imageWidth * 24], 2);
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 24], &ix->huge_buffer[pos + ix->info.imageWidth * 32], 3);
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 32], &ix->huge_buffer[pos + ix->info.imageWidth * 40], 4);
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 40], &ix->huge_buffer[pos + ix->info.imageWidth * 48], 5);
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 48], &ix->huge_buffer[pos + ix->info.imageWidth * 56], 6);
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 56], &ix->huge_buffer[pos + ix->info.imageWidth * 64], 7);
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 64], &ix->huge_buffer[pos + ix->info.imageWidth * 72], 8);
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 72], &ix->huge_buffer[pos + ix->info.imageWidth * 80], 8);
    memcpy(&ix->huge_buffer[pos + ix->info.imageWidth * 80], &ix->huge_buffer[pos + ix->info.imageWidth * 81], 9);

#if 1
	error = ix_spi_read_wait_irq(ix, &rx);
	if(error)
		return error;

	if(!(rx & IX_IRQ_AUTO_PARTIAL_DONE))
	{
		dev_err(&ix->spi->dev, "[ERROR] irq : %x expected %x\n", rx, IX_IRQ_AUTO_PARTIAL_DONE);
		return -EINTR;
	}
#endif

	return 0;
}
#else
static int ix_spi_read_image(struct ix_data *ix)
{
	int error;
	u8 tx[IX_CMD_SIZE];
	int i;
	u8 rx;

	struct spi_message m;
	struct spi_transfer t;

	memset(&t, 0, sizeof(struct spi_transfer));

	if(ix->current_frame >= ix->info.frameMax)
	{
		ix->current_frame = 0;
	}

	tx[0] = IX_SPI_READ_COMMAND;
	tx[1] = IX_CMD_READ_M;
	tx[2] = 0x00;	/* dummy byte */

	t.cs_change = CS_CHANGE_VALUE,
	t.delay_usecs = 5,
	t.speed_hz = IX_SPI_CLOCK_SPEED,
	t.tx_buf = tx,
	t.rx_buf = ix->huge_buffer + (ix->current_frame * (ix->info.imageSize)),
	t.len = IX_CMD_SIZE + ix->info.imageWidth * IX_DEFAULT_READ_LINE,
	//t.len = IX_CMD_SIZE + ix->info.imageSize,
	t.tx_dma = 0,
	t.rx_dma = 0,
	t.bits_per_word = 0,

	error = ix_spi_write_reg(ix, IX_CMD_START_C, 0x00);
	if(error)
		return error;

#if 0
	udelay(1500);
#else
	error = ix_spi_read_wait_irq(ix, &rx);
	if(error)
		return error;

	if(!(rx & IX_IRQ_AUTO_PARTIAL_DONE))
	{
		dev_err(&ix->spi->dev, "[ERROR] irq : %x expected %x\n", rx, IX_IRQ_AUTO_PARTIAL_DONE);
		return -EINTR;
	}
#endif

	for(i=0; i<ceil(ix->info.imageHeight, IX_DEFAULT_READ_LINE); i++)
	{
		error = ix_spi_write_reg(ix, IX_CMD_READ_C_PLUS, 0x00);
		if(error)
			return error;

		spi_message_init(&m);
		spi_message_add_tail(&t, &m);

		error = spi_sync(ix->spi, &m);
		if (error)
		{
			dev_err(&ix->spi->dev, "[ERROR] spi_sync failed.\n");
			return error;
		}

		memcpy(t.rx_buf, t.rx_buf + IX_CMD_SIZE, ix->info.imageWidth * IX_DEFAULT_READ_LINE);

		t.rx_buf += ix->info.imageWidth * IX_DEFAULT_READ_LINE;
	}

#if 1
	error = ix_spi_read_wait_irq(ix, &rx);
	if(error)
		return error;

	if(!(rx & IX_IRQ_AUTO_PARTIAL_DONE))
	{
		dev_err(&ix->spi->dev, "[ERROR] irq : %x expected %x\n", rx, IX_IRQ_AUTO_PARTIAL_DONE);
		return -EINTR;
	}
#endif

	return 0;
}

#endif


static int ix_spi_read_wait_irq(struct ix_data *ix, u8 *irq)
{
	int error;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	while(true)
	{
		error = ix_wait_for_irq(ix, IX_DEFAULT_IRQ_TIMEOUT);
		if(error == 0)
			break;
		if(ix->thread_task.should_stop)
		{
			dev_info(&ix->spi->dev, "[INFO] ix_spi_read_wait_irq cancel.\n");
			return -EINTR;
		}
		if(error != -ETIMEDOUT)
		{
			dev_err(&ix->spi->dev, "[ERROR] wait_irq timeout.\n");
			return error;
		}
	}

	error = ix_spi_read_reg(ix, IX_CMD_INT_R, irq);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_R failed.\n");
		return error;
	}
	//dev_info(&ix->spi->dev, "[INFO] read irq %02x\n", *irq);



	error = ix_spi_write_reg(ix, IX_CMD_INT_C, 0x00);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_C failed.\n");
		return error;
	}

	return error;
}


static int ix_wait_for_irq(struct ix_data *ix, int timeout)
{
	int result;

	//DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	if (!timeout)
	{
		result = wait_event_interruptible(ix->waiting_interrupt_return, ix->interrupt_done);
	}
	else
	{
		result = wait_event_interruptible_timeout(ix->waiting_interrupt_return, ix->interrupt_done, timeout);
	}

	if (result < 0)
	{
		dev_err(&ix->spi->dev, "[ERROR] wait_event_interruptible - interrupted by signal.\n");
		return result;
	}

	if (result || !timeout)
	{
		ix->interrupt_done = 0;
		return 0;
	}

	return -ETIMEDOUT;
}


static int ix_get_reference(struct ix_data *ix, bool isInnerPath)
{
	int error;
	int i;
	u8 rx;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix->current_frame = 0;
	if(isInnerPath)
	{
		error = ix_spi_read_reg(ix, 0x45, &rx);
		if(error)
			return error;

		error = ix_spi_write_reg(ix, 0x45, 0x00);
		if(error)
			return error;
	}

	for(i=0; i<IX_ADC_COUNT; i++)
	{
		ix_auto_adc_ctrl(ix, i);
		error = ix_spi_read_image(ix);
		if(error)
		{
			dev_err(&ix->spi->dev, "[ERROR] read image failed.\n");
			return error;
		}
		ix_finger_detect(ix);
		memcpy(ix->info.refImageSum[i], ix->pxl_sum, sizeof(int)*IX_FINGER_DETECT_COUNT);
		memcpy(&ix->info.refImage[i*ix->info.imageSize], &ix->huge_buffer[0], ix->info.imageSize);
	}

	if(isInnerPath)
	{
		error = ix_spi_write_reg(ix, 0x45, rx);
		if(error)
			return error;

		error = ix_spi_write_reg(ix, IX_REG_PGA_OFFSET, ix->adc_setup.offset);
		if(error)
			return error;
	}

	return 0;
}


static u8 ix_finger_detect(struct ix_data *ix)
{
	//             up  left down right center
	int xstart[]={ 56,  15,  56, 97,  56 };
	int ystart[]={ 15,  44,  73,  44,  44 };

	int i, x, y, pos;
	int detect_area_cnt = 0;

	for(i=0; i<IX_FINGER_DETECT_COUNT; i++)
	{
		ix->pxl_sum[i] = 0;

		for (x=-5; x<6; x++)
		{
			for (y=-5; y<6; y++)
			{
				pos = (ystart[i]+y) * ix->info.imageWidth + xstart[i] + x;
				ix->pxl_sum[i]+=ix->huge_buffer[pos + (ix->current_frame * ix->info.imageSize)];
			}
		}
		//dev_info(&ix->spi->dev, "[INFO] fd[%d] ref[%d] pxl[%d] diff[%d]\n", i, ix->info.refImageSum[ix->current_adc_table][i] ,ix->pxl_sum[i], ix->info.refImageSum[ix->current_adc_table][i] - ix->pxl_sum[i]);

		//dev_info(&ix->spi->dev, "[INFO] detect_area_cnt[%d]\n", detect_area_cnt);
		if(ix->info.refImageSum[ix->current_adc_table][i] - ix->pxl_sum[i] > (int)ix->diag.finger_threshold)
		{
			detect_area_cnt++;
			//dev_info(&ix->spi->dev, "[INFO] detect_area_cnt[%d] fd[%d]\n", detect_area_cnt, i);
		}
	}

	if(detect_area_cnt >= 3)
	{
		ix->diag.finger_status = FNGR_ST_DETECTED;
		return IX_IRQ_FINGER_DOWN;
	}
	else
	{
		ix->diag.finger_status = FNGR_ST_LOST;
		return IX_IRQ_FINGER_UP;
	}
}


static void ix_image_calibration(struct ix_data *ix)
{
	int i;
	int ref_data;
	int real_data;
	int calibed_data;

	for(i=0; i<ix->info.imageSize; i++)
	{
		ref_data = ix->info.refImage[i + ix->current_adc_table * ix->info.imageSize];
		real_data = ix->huge_buffer[i + (ix->current_frame * ix->info.imageSize)];

		calibed_data = ref_data - real_data;

		if ( calibed_data > 255 )
			calibed_data = 255;
		else if ( calibed_data < 0 )
			calibed_data = 0;

		ix->huge_buffer[i + (ix->current_frame * ix->info.imageSize)] = calibed_data;
	}
}



static int ix_mode_change(struct ix_data *ix, int mode)
{
	int i, j;
	int error;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	switch(mode)
	{
		case IX_MODE_CAPTURE:
			error = ix_reset(ix);
			if (error)
			{
				dev_err(&ix->spi->dev, "ix_mode_chage, reset fail.\n");
				return error;
			}

			error = ix_spi_write_reg(ix, IX_CMD_CMODE_EN, 0x01);
			if (error)
			{
				dev_err(&ix->spi->dev, "[ERROR] IX_CMD_CMODE_EN failed.\n");
				return error;
			}
			break;

		case IX_MODE_FINGERDOWN:
			error = ix_reset(ix);
			if (error)
			{
				dev_err(&ix->spi->dev, "ix_mode_chage, reset fail.\n");
				return error;
			}

			for(i=0; i<ARRAY_SIZE(init_seq_down); i+=3)
			{
				error = ix_spi_write_reg(ix, init_seq_down[i], init_seq_down[i+1]);
				if(error)
					return error;

				if(init_seq_down[i+2] > 0)
				{
					for(j=0; j<init_seq_down[i+2]; j++)
						udelay(100);
				}
			}

			error = ix_spi_write_reg(ix, IX_CMD_DMODE_EN, 0x00);
			if (error)
			{
				dev_err(&ix->spi->dev, "[ERROR] IX_CMD_DMODE_EN failed.\n");
				return error;
			}

			//dev_info(&ix->spi->dev, "[INFO] IX_CMD_DMODE_EN - finger down.\n");
			break;

		case IX_MODE_FINGERUP:
			error = ix_reset(ix);
			if (error)
			{
				dev_err(&ix->spi->dev, "ix_mode_chage, reset fail.\n");
				return error;
			}

			for(i=0; i<ARRAY_SIZE(init_seq_up); i+=3)
			{
				error = ix_spi_write_reg(ix, init_seq_up[i], init_seq_up[i+1]);
				if(error)
					return error;

				if(init_seq_up[i+2] > 0)
				{
					for(j=0; j<init_seq_up[i+2]; j++)
						udelay(100);
				}
			}

			error = ix_spi_write_reg(ix, IX_CMD_DMODE_EN, 0x00);
			if (error)
			{
				dev_err(&ix->spi->dev, "[ERROR] IX_CMD_DMODE_EN failed.\n");
				return error;
			}

			//dev_info(&ix->spi->dev, "[INFO] IX_CMD_DMODE_EN - finger up.\n");
			break;

		case IX_MODE_STANDBY:
			ix_refresh(ix);

			error = ix_spi_write_reg(ix, IX_CMD_STBY_EN, 0x00);
			if (error)
			{
				dev_err(&ix->spi->dev, "[ERROR] IX_CMD_STBY_EN failed.\n");
				return error;
			}

			error = ix_spi_write_reg(ix, 0x1C, 0x00);
			if (error)
				return error;

			error = ix_spi_write_reg(ix, 0x1D, 0x00);
			if (error)
				return error;

			break;

		default:
			break;
	}

	return 0;
}


static int threadfn(void *_ix)
{
	struct ix_data *ix = _ix;

	while (!kthread_should_stop())
	{
		up(&ix->thread_task.sem_idle);
		wait_event_interruptible(ix->thread_task.wait_job, ix->thread_task.mode != IX_THREAD_IDLE_MODE);

		down(&ix->thread_task.sem_idle);

		switch (ix->thread_task.mode)
		{
			case IX_THREAD_CAPTURE_MODE:
				ix_capture_task(ix);
				break;

			default:
				break;
		}

		if(ix->thread_task.mode != IX_THREAD_EXIT)
			ix->thread_task.mode = IX_THREAD_IDLE_MODE;
	}

	return 0;
}


static int ix_start_thread(struct ix_data *ix, int mode)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	//dev_info(&ix->spi->dev, "[INFO] ix start mode = %d\n", mode);

	ix->thread_task.should_stop = 0;
	ix->thread_task.mode = mode;
	wake_up_interruptible(&ix->thread_task.wait_job);

	return 0;
}


static int ix_thread_goto_idle(struct ix_data *ix)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix->thread_task.should_stop = 1;
	ix->thread_task.mode = IX_THREAD_IDLE_MODE;
	if (down_interruptible(&ix->thread_task.sem_idle))
		return -ERESTARTSYS;

	up(&ix->thread_task.sem_idle);

	return 0;
}


static int ix_auto_adc_ctrl(struct ix_data *ix, int count)
{
	int error;

	//dev_info(&ix->spi->dev, "[INFO] auto adc ctrl level[%02x] offset[%02x].\n", adc_level[count]|0x20, adc_offset[count]);
	error = ix_spi_write_reg(ix, IX_REG_CP1_SET3, adc_level[count]|0x20);
	if(error)
	return error;

#if 0
	error = ix_spi_write_reg(ix, IX_REG_PGA_OFFSET, adc_offset[count]);
	if(error)
	return error;
#endif

	return 0;
}


static int ix_capture_task(struct ix_data *ix)
{
	int error = 0;
	u8 rx;
    int same_image_cnt = 0;
	struct timespec ts_start, ts_end, ts_delta;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix->diag.frames_captured = 0;
	ix->diag.frames_stored = 0;
	ix->diag.capture_time = 0;
	ix->current_adc_table = 0;

	//wait finger up
	dev_info(&ix->spi->dev, "[INFO] wait finger up.\n");
	ix->diag.finger_status = FNGR_ST_NONE;
	error = ix_mode_change(ix, IX_MODE_FINGERUP); // sleep mode ... just detect
	if(error)
		goto out;

	error = ix_spi_read_wait_irq(ix, &rx);
	if(error)
		goto out;

	if(rx & IX_IRQ_FINGER_UP)
	{
		dev_info(&ix->spi->dev, "[INFO] finger up interrupt OK.\n");
		error = ix_mode_change(ix, IX_MODE_CAPTURE);//normal mode ... capture mode
    	if(error)
    		goto out;

        while(true)
        {
			if(ix->thread_task.should_stop)
			{
				dev_info(&ix->spi->dev, "[INFO] capture task cancel.\n");
				error = -EINTR;
				goto out;
			}

    		error = ix_spi_read_image(ix);
    		if(error)
    		{
    			dev_err(&ix->spi->dev, "[ERROR] read image failed.\n");
    			goto out;
    		}

            if(ix_finger_detect(ix) == IX_IRQ_FINGER_UP)
            {
				dev_info(&ix->spi->dev, "[INFO] detect finger up.\n");
				msleep(50);
				break;
            }
        }
	}
	else
	{
		dev_err(&ix->spi->dev, "[ERROR] irq : 0x%02x expected 0x%02x\n", rx, IX_IRQ_FINGER_UP);
		error = -EINTR;
		goto out;
	}

	//wait finger down
	dev_info(&ix->spi->dev, "[INFO] wait finger down.\n");

	error = ix_mode_change(ix, IX_MODE_FINGERDOWN);
	if(error)
		goto out;

	error = ix_spi_read_wait_irq(ix, &rx);
	if(error)
		goto out;

	if(rx & IX_IRQ_FINGER_DOWN)
	{
		dev_info(&ix->spi->dev, "[INFO] finger down interrupt OK.\n");
		ix->diag.finger_status = FNGR_ST_DETECTED;
		msleep(50);
	}
	else
	{
		dev_err(&ix->spi->dev, "[ERROR] irq : 0x%02x expected 0x%02x\n", rx, IX_IRQ_FINGER_DOWN);
		error = -EINTR;
		goto out;
	}

	error = ix_mode_change(ix, IX_MODE_CAPTURE);
	if(error)
		goto out;

	ix->current_frame = 0;
	ix->avail_data = 0;
	ix->data_offset = 0;

	getnstimeofday(&ts_start);

	while(true)
	{
		if(ix->thread_task.should_stop)
		{
			dev_info(&ix->spi->dev, "[INFO] capture task cancel.\n");
			error = -EINTR;
			//goto out;
			break;
		}

		if(ix->current_adc_table < IX_ADC_COUNT)
			ix_auto_adc_ctrl(ix, ix->current_adc_table);
		else
			break;

		error = ix_spi_read_image(ix);
		if(error)
		{
			dev_err(&ix->spi->dev, "[ERROR] read image failed.\n");
			goto out;
		}

		ix->diag.frames_captured++;
#if 0
		if(ix_finger_detect(ix) == IX_IRQ_FINGER_UP)
		{
			dev_info(&ix->spi->dev, "[INFO] finger up.\n");
			break;
		}
#endif
		ix->diag.frames_stored++;

		ix_image_calibration(ix);
		ix->avail_data += ix->info.imageSize;
		ix->current_frame++;

        same_image_cnt++;
        if(same_image_cnt%2 == 0)
		    ix->current_adc_table++;

		wake_up_interruptible(&ix->waiting_data_avail);
	}

	getnstimeofday(&ts_end);
	ts_delta = timespec_sub(ts_end, ts_start);

	ix->diag.capture_time = ts_delta.tv_nsec / NSEC_PER_MSEC;
	ix->diag.capture_time += (ts_delta.tv_sec * MSEC_PER_SEC);

	if (ix->diag.capture_time > 0)
	{
		dev_info(&ix->spi->dev,	"[INFO] captured %lu frames (%lu kept) in %lu  ms (%lu fps)\n",
				(long unsigned int)ix->diag.frames_captured,
				(long unsigned int)ix->diag.frames_stored,
				(long unsigned int)ix->diag.capture_time,
				(long unsigned int)(ix->diag.frames_stored * MSEC_PER_SEC / ix->diag.capture_time));
				//(long unsigned int)(total_captures * MSEC_PER_SEC / ix->diag.capture_time));
	}

out:
	ix->diag.finger_status = FNGR_ST_NONE;
	ix->capture_done = true;
	wake_up_interruptible(&ix->waiting_data_avail);

	return error;
}


static void ix_start_capture(struct ix_data *ix)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix_refresh(ix);
	ix_start_thread(ix, IX_THREAD_CAPTURE_MODE);
	ix->capture_done = false;
	ix->diag.finger_status = FNGR_ST_NONE;
}


static int ix_selftest_short(struct ix_data *ix)
{
	int error;
	u8 rx;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix_refresh(ix);

	error = ix_reset(ix);
	if (error)
	{
		dev_err(&ix->spi->dev, "ix selftest, reset fail on entry.\n");
		goto err;
	}

	error = ix_spi_read_reg(ix, IX_REG_CHIP_ID, &rx);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_REG_CHIP_ID failed.\n");
		goto err;
	}

	dev_info(&ix->spi->dev, "[INFO] chip ID : 0x%02x\n", rx);
	if(rx != IX_HW_ID)
	{
		dev_err(&ix->spi->dev, "[ERROR] hardware id mismatch : %02x expected %02x\n", rx, IX_HW_ID);
		goto err;
	}

	ix_refresh(ix);

	error = ix_reset(ix);
	if (error)
	{
		dev_err(&ix->spi->dev, "ix selftest, reset fail on entry.\n");
		goto err;
	}

err:
	ix->diag.selftest = (error == 0)? 1 : 0;

	return error;
}


static int ix_sensor_test(struct ix_data *ix)
{
	int error;
	u8 rx;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix_refresh(ix);

	error = ix_reset(ix);
	if (error)
	{
		dev_err(&ix->spi->dev, "ix selftest, reset fail on entry.\n");
		goto err;
	}

	ix->thread_task.should_stop = 0;

	error = ix_spi_read_reg(ix, IX_REG_CHIP_ID, &rx);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_REG_CHIP_ID failed.\n");
		goto err;
	}

	dev_info(&ix->spi->dev, "[INFO] chip ID : 0x%02x\n", rx);
	if(rx != IX_HW_ID)
	{
		dev_err(&ix->spi->dev, "[ERROR] hardware id mismatch : %02x expected %02x\n", rx, IX_HW_ID);
		goto err;
	}

	error = ix_spi_write_reg(ix, IX_CMD_CMODE_EN, 0x01);
	if(error)
		goto err;

	error = ix_spi_write_reg(ix, IX_CMD_START_C, 0x00);
	if(error)
	{
		goto err;
	}

	error = ix_spi_read_wait_irq(ix, &rx);
	if(error)
	{
		goto err;
	}

	ix_refresh(ix);

	error = ix_reset(ix);
	if (error)
	{
		dev_err(&ix->spi->dev, "ix selftest, reset fail on entry.\n");
		goto err;
	}

err:
	ix->diag.sensortest = (error == 0)? 1 : 0;
	ix->thread_task.should_stop = 1;

	return error;
}


irqreturn_t ix_interrupt(int irq, void *_ix)
{
	struct ix_data *ix = _ix;
	if( gpio_get_value(ix->irq_gpio) )
	{
		//dev_info(&ix->spi->dev, "[INFO] interrupt\n");
		ix->interrupt_done = 1;
		wake_up_interruptible(&ix->waiting_interrupt_return);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}


static int ix_open(struct inode *inode, struct file *file)
{
	struct ix_data *ix;
	int error = 0;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	ix = container_of(inode->i_cdev, struct ix_data, cdev);

	if (down_interruptible(&ix->mutex))
		return -ERESTARTSYS;

	file->private_data = ix;

	up(&ix->mutex);

	return error;
}


static int ix_release(struct inode *inode, struct file *file)
{
	int error = 0;
	struct ix_data *ix = file->private_data;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

//#ifdef CONFIG_HAS_EARLYSUSPEND
	if(ix->isSleep == true)
		return 0;
//#endif

	if (down_interruptible(&ix->mutex))
		return -ERESTARTSYS;

	if (!atomic_read(&file->f_count))
	{
		ix_mode_change(ix, IX_MODE_STANDBY);
	}
	up(&ix->mutex);

	return error;
}



static ssize_t ix_read(struct file *file, char *buff, size_t count, loff_t *ppos)
{
	int error = 0;
	u32	read_cnt;
	u32	remain_cnt = 0;

	struct ix_data *ix = file->private_data;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	if (down_interruptible(&ix->mutex))
		return -ERESTARTSYS;

	error = wait_event_interruptible( ix->waiting_data_avail, (ix->capture_done || ix->avail_data));

	read_cnt = (count > ix->avail_data) ? ix->avail_data : count;

	if(ix->data_offset + read_cnt >= ix->info.frameMax * ix->info.imageSize)
	{
		remain_cnt = ix->info.frameMax * ix->info.imageSize - ix->data_offset;

		error = copy_to_user(buff, &ix->huge_buffer[ix->data_offset], remain_cnt);
		if (error < 0)
		{
			dev_err(&ix->spi->dev, "[ERROR] copy_to_user failed.\n");
			error = -EFAULT;
			goto out;
		}

		ix->data_offset = 0;
		read_cnt -= remain_cnt;
		ix->avail_data -= remain_cnt;
		if(ix->avail_data == 0)
		{
			error = remain_cnt;
			goto out;
		}
	}

	if(read_cnt > 0)
	{
		error = copy_to_user(buff + remain_cnt, &ix->huge_buffer[ix->data_offset], read_cnt);
		if (error < 0)
		{
			dev_err(&ix->spi->dev, "[ERROR] copy_to_user failed.\n");
			error = -EFAULT;
			goto out;
		}

		ix->data_offset += read_cnt;
		ix->avail_data -= read_cnt;
		error = read_cnt + remain_cnt;
	}

	//dev_info(&ix->spi->dev, "[INFO] ix_read data_offset:%d\n", ix->data_offset);

out:
	up(&ix->mutex);

	//dev_info(&ix->spi->dev, "[INFO] ix_read return : %d\n", error);

	return error;
}


static ssize_t ix_write(struct file *file, const char *buff, size_t count, loff_t *ppos)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);
	return -ENOTTY;
}


static unsigned int ix_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;
	struct ix_data *ix = file->private_data;

	if (down_interruptible(&ix->mutex))
		return -ERESTARTSYS;

	if ( (ix->avail_data == 0) && (ix->capture_done == false) )
		poll_wait(file, &ix->waiting_data_avail, wait);

	if(ix->avail_data > 0)
	{
		//dev_info(&ix->spi->dev, "[INFO] POLLIN\n");
		ret |= (POLLIN | POLLRDNORM);
	}
	else if(ix->capture_done)
	{
		//dev_info(&ix->spi->dev, "[INFO] POLLHUP\n");
		ret |= POLLHUP;
	}

	up(&ix->mutex);

	return ret;
}


static long ix_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int error;
	struct ix_data *ix = filp->private_data;
	error = 0;

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

//#ifdef CONFIG_HAS_EARLYSUSPEND
	if(ix->isSleep == true)
		return -EAGAIN;
//#endif

	if (down_interruptible(&ix->mutex))
		return -ERESTARTSYS;

	switch (cmd)
	{
		case IX_IOCTL_START_CAPTURE:
			ix_start_capture(ix);
			break;
		case IX_IOCTL_ABORT_CAPTURE:
			ix_refresh(ix);
			break;
		case IX_IOCTL_MAX_PERFORMANCE:
			ix_set_performance(ix, true);
			break;
		case IX_IOCTL_DEFAULT_PERFORMANCE:
			ix_set_performance(ix, false);
			break;
		default:
			error = -ENOTTY;
			break;
	}
	up(&ix->mutex);

	return error;
}



static int ix_cleanup(struct ix_data *ix)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);

	if (ix->thread_task.thread)
	{
		ix->thread_task.should_stop = 1;
		ix->thread_task.mode = IX_THREAD_EXIT;
		wake_up_interruptible(&ix->thread_task.wait_job);
		kthread_stop(ix->thread_task.thread);
	}

	if (!IS_ERR_OR_NULL(ix->device))
		device_destroy(ix->class, ix->devno);

	class_destroy(ix->class);

	if (gpio_is_valid(ix->cs_gpio))
	{
		gpio_free(ix->cs_gpio);
		pr_info("remove cs_gpio success\n");
	}

	if (ix->irq >= 0)
		free_irq(ix->irq, ix);

	if (gpio_is_valid(ix->irq_gpio))
	{
		gpio_free(ix->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}

	if (gpio_is_valid(ix->reset_gpio))
	{
		gpio_free(ix->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}

	if (ix->huge_buffer)
	{
		free_pages((unsigned long)ix->huge_buffer, get_order(IX_IMAGE_BUFFER_SIZE + IX_CMD_SIZE));
	}

	kfree(ix->info.refImage);
	kfree(ix);

	return 0;
}

static void ix_irq_gpio_config(struct ix_data *ix)
{
	struct pinctrl *pinctrl;

	pinctrl = devm_pinctrl_get_select(&ix->spi->dev, "ix_irq_gpio");
	if (IS_ERR(pinctrl))
		dev_err(&ix->spi->dev, "failed to set external interrupt");
}

static int ix_fb_state_chg_callback(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct ix_data *ix ;
	struct fb_event *evdata = data;
	unsigned int blank;

	if (val != FB_EVENT_BLANK)
		return 0;
	pr_info("[info] go to the ix_fb_state_chg_callback\n");
	ix = container_of(nb,struct ix_data,notifier);
#if 0
	blank = *(int *)evdata->data;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
		pr_info("%s state is suspend\n", __func__);
		ix_mode_change(ix, IX_MODE_STANDBY);
		break;
	case FB_BLANK_UNBLANK:
		pr_info("%s state is resume\n", __func__);
		break;
	default:
		pr_info("%s defalut\n", __func__);
		break;
	}
#else
	if (evdata && evdata->data && val == FB_EVENT_BLANK && ix) {
		blank = *(int *)(evdata->data);

		switch(blank) {
		case FB_BLANK_POWERDOWN:
		case FB_BLANK_NORMAL:
			dev_info(&ix->spi->dev, "[INFO] ix_late_suspend()\n");
			ix->isSleep = true;
			ix_mode_change(ix, IX_MODE_STANDBY);//deep sleep mode
			break;

		case FB_BLANK_UNBLANK:
			dev_info(&ix->spi->dev, "[INFO] ix_late_resume()\n");
			ix->isSleep = false;
			if (down_interruptible(&ix->mutex))
				return 0;
			up(&ix->mutex);
			break;

		default:
			pr_info("%s defalut\n", __func__);
			break;
		}
	}
#endif
	return NOTIFY_OK;
}


static struct notifier_block ix_noti_block = {
	.notifier_call = ix_fb_state_chg_callback,
};

static void ix_set_performance(struct ix_data *ix, bool isMax)
{
	//TODO

	if(isMax)
	{
		//set max performance
		dev_info(&ix->spi->dev, "[INFO] set max performance\n");
	}
	else
	{
		//set default performance
		dev_info(&ix->spi->dev, "[INFO] set default performance\n");
	}
}

static int ix_probe(struct spi_device *spi)
{
	struct ix_platform_data *ix_pdata;
	struct device *dev = &spi->dev;
	struct ix_data *ix = NULL;
	struct s3c64xx_spi_csinfo *cs = spi->controller_data;

	int error = 0;
	int ret =0;
	u8 rx;

	pr_info("[INFO] %s success!\n",__func__);

	/*allocate memory*/
	ix = kzalloc(sizeof(*ix), GFP_KERNEL);
	if (!ix)
	{
		dev_err(&spi->dev, "[ERROR] failed to allocate memory for struct ix_data\n");
		return -ENOMEM;
	}
	ix->huge_buffer = (u8 *)__get_free_pages(GFP_KERNEL, get_order(IX_IMAGE_BUFFER_SIZE + IX_CMD_SIZE));
	if (!ix->huge_buffer)
	{
		dev_err(&ix->spi->dev, "[ERROR] failed to get free pages\n");
		return -ENOMEM;
	}
	ix_pdata = kzalloc(sizeof(struct ix_platform_data),GFP_KERNEL);
	if(!ix_pdata)
	{
		dev_err(&spi->dev, "[ERROR] failed to allocate memory for struct ix_platform_data\n");
		return -ENOMEM;
	}

	/*initialized ix struct*/
	spi_set_drvdata(spi, ix);
	ix->spi = spi;
	ix->spi->mode = SPI_MODE_0;
	ix->spi->bits_per_word = 8;
	init_waitqueue_head(&ix->waiting_interrupt_return);
	init_waitqueue_head(&ix->waiting_data_avail);
	init_waitqueue_head(&ix->thread_task.wait_job);
	sema_init(&ix->thread_task.sem_idle, 0);
	sema_init(&ix->mutex, 0);
	ix->notifier = ix_noti_block;
	ix->reset_gpio = -EINVAL;
	ix->irq_gpio = -EINVAL;
	ix->irq = -EINVAL;
	ix->cs_gpio = -EINVAL;


	/*get cs resource*/
	ix->cs_gpio = of_get_named_gpio(dev->of_node,"ix,spi_cs_gpio",0);
	if (!gpio_is_valid(ix->cs_gpio))
	{
		dev_err(&ix->spi->dev,	"[ERROR] gpio_request (cs) failed.\n");
		goto err;
	}
	ret = gpio_request(ix->cs_gpio, "ix_cs");
	if (ret) {
		dev_err(dev, "could not request cs gpio, %d\n",ret);
		goto err;
	}
	error = gpio_direction_output(ix->cs_gpio, 1);
	cs->line = ix->cs_gpio;

	/*get irq resource*/
	ix_pdata->irq_gpio = of_get_named_gpio(dev->of_node,"ix,spi_irq_gpio",0);
	if (!gpio_is_valid(ix_pdata->irq_gpio))
	{
		dev_err(&ix->spi->dev,	"[ERROR] gpio_request (irq) failed.\n");
		goto err;
	}
	ix->irq_gpio = ix_pdata->irq_gpio;
	ret = gpio_request(ix->irq_gpio, "ix_irq");
	if (ret) {
		dev_err(dev, "could not request irq gpio, %d\n",
			ret);
	}
	error = gpio_direction_input(ix->irq_gpio);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] gpio_direction_input (irq) failed.\n");
		goto err;
	}
	ix_irq_gpio_config(ix);
	ix->irq = gpio_to_irq(ix->irq_gpio);
	if (ix->irq < 0)
	{
		dev_err(&ix->spi->dev, "[ERROR] gpio_to_irq failed.\n");
		error = ix->irq;
		goto err;
	}
	error = request_irq(ix->irq, ix_interrupt, IRQF_TRIGGER_RISING, "ix", ix);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] request_irq %i failed.\n", ix->irq);
		ix->irq = -EINVAL;
		goto err;
	}

	/*get reset resource*/
	ix_pdata->reset_gpio = of_get_named_gpio(dev->of_node,"ix,spi_reset_gpio",0);
	if (!gpio_is_valid(ix_pdata->reset_gpio))
	{
		dev_err(&ix->spi->dev,	"[ERROR] gpio_request (reset) failed.\n");
		goto err;
	}
	ix->reset_gpio = ix_pdata->reset_gpio;
	ret = gpio_request(ix->reset_gpio, "ix_reset");
	if (ret) {
		dev_err(dev, "could not request sleep gpio, %d\n",
			ret);
	}
	error = gpio_direction_output(ix->reset_gpio, 0);
	if (error)
	{
		dev_err(&ix->spi->dev,	"[ERROR] gpio_direction_output(reset) failed.\n");
		goto err;
	}

	error = spi_setup(ix->spi);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] spi_setup failed.\n");
		goto err;
	}

	gpio_set_value(ix->reset_gpio,0);
	udelay(100);
	gpio_set_value(ix->reset_gpio, 1);
	udelay(100);


	error = gpio_get_value(ix->irq_gpio) ? 0 : -EIO;
	if (error) {
		dev_err(&ix->spi->dev, "[ERROR] irq_gpio failed.\n");
		goto err;
	}
	disable_irq(ix->irq);
	ix->interrupt_done = 0;
	enable_irq(ix->irq);
	error = ix_spi_write_reg(ix, 0x1d, 0x6e);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] interrupt low failed.\n");
		goto err;
	}
	error = ix_spi_read_reg(ix, IX_CMD_INT_R, &rx);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_R failed.\n");
		goto err;
	}
	if(rx != IX_IRQ_REBOOT)
	{
		dev_err(&ix->spi->dev, "[ERROR] reset error: %x expected %x\n",	rx,	IX_IRQ_REBOOT);
		error = -EIO;
		goto err;
	}

	error = ix_spi_write_reg(ix, IX_CMD_INT_C, 0x00);	/*IRQ goes to low when all of the INT[0:7](0X0C) bit is low*/
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_INT_C failed.\n");
		goto err;
	}

	memset(&(ix->diag), 0, sizeof(ix->diag));
	/* register defalut value */
	ix->diag.finger_threshold = IX_FINGER_DETECT_THRESH;
	ix->reg_setup.finger_down_threshold = IX_DEFAULT_FINGER_DOWN_THRESH;
	ix->reg_setup.finger_up_threshold = IX_DEFAULT_FINGER_UP_THRESH;
	ix->adc_setup.gain = IX_DEFAULT_PGA_GAIN;
	ix->adc_setup.offset = IX_DEFAULT_PGA_OFFSET;
	ix->adc_setup.level = IX_DEFAULT_CP1_SET3;

	error = ix_spi_read_reg(ix, IX_REG_CHIP_ID, &ix->info.chipID);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_REG_CHIP_ID failed.\n");
		goto err;
	}

	error = ix_spi_read_reg(ix, IX_REG_REVISION_NUMBER, &ix->info.revNO);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_REG_REVSION_NUMBER failed.\n");
		goto err;
	}

	dev_info(&ix->spi->dev, "[INFO] *****************************************\n");
	dev_info(&ix->spi->dev, "[INFO] * Chip ID[0x%02x] : Revision Number[0x%02x] *\n", ix->info.chipID, ix->info.revNO);
	dev_info(&ix->spi->dev, "[INFO] *****************************************\n");

	if(ix->info.chipID != IX_HW_ID)
	{
		dev_err(&ix->spi->dev, "[ERROR] hardware id mismatch : %02x expected %02x\n", ix->info.chipID, IX_HW_ID);
		goto err;
	}
	else
	{
		ix_set_info(ix, IX_THREAD_CAPTURE_MODE);
		ix->info.refImage = kzalloc(ix->info.imageSize * IX_ADC_COUNT, GFP_KERNEL);
		if (!ix->info.refImage)
		{
			dev_err(&ix->spi->dev, "[ERROR] failed to allocate memory for refImage\n");
			error = -ENOMEM;
			goto err;
		}
	}

	error = ix_init_param(ix);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] ix_init_param failed.\n");
		goto err;
	}

	error = ix_spi_write_reg(ix, IX_CMD_CMODE_EN, 0x01);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_CMODE_EN failed.\n");
		return error;
	}

	error = ix_get_reference(ix, true);
	if(error)
	{
		dev_err(&ix->spi->dev, "[ERROR] ix_get reference failed.\n");
		goto err;
	}

	ix->devno = MKDEV(IX_MAJOR, ix_device_count++);

	ix->class = class_create(THIS_MODULE, IX_CLASS_NAME);
	if (IS_ERR(ix->class))
	{
		dev_err(&ix->spi->dev, "failed to create class.\n");
		error = PTR_ERR(ix->class);
		goto err;
	}

	ix->device = device_create(ix->class, NULL, ix->devno, NULL, "%s", IX_DEV_NAME);
	if (IS_ERR(ix->device))
	{
		dev_err(&ix->spi->dev, "[ERROR] device_create failed.\n");
		error = PTR_ERR(ix->device);
		goto err;
	}

	error = sysfs_create_group(&spi->dev.kobj, &ix_adc_attr_group);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] sysfs_create_group failed.\n");
		goto err;
	}

	error = sysfs_create_group(&spi->dev.kobj, &ix_diag_attr_group);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] sysfs_create_group failed.\n");
		goto err_sysf_1;
	}

	error = sysfs_create_group(&spi->dev.kobj, &ix_reg_attr_group);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] sysfs_create_group failed.\n");
		goto err_sysf_2;
	}

	error = register_chrdev_region(ix->devno, 1, IX_DEV_NAME);
	if (error)
	{
		dev_err(&ix->spi->dev,	"[ERROR] register_chrdev_region failed.\n");
		goto err_sysf_3;

	}

	cdev_init(&ix->cdev, &ix_fops);
	ix->cdev.owner = THIS_MODULE;

	error = cdev_add(&ix->cdev, ix->devno, 1);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] cdev_add failed.\n");
		goto err_chrdev;
	}

	ix->thread_task.mode = IX_THREAD_IDLE_MODE;
	ix->thread_task.thread = kthread_run(threadfn, ix, "%s", IX_WORKER_THREAD_NAME);
	if (IS_ERR(ix->thread_task.thread))
	{
		dev_err(&ix->spi->dev, "[ERROR] kthread_run failed.\n");
		goto err_cdev;
	}

	fb_register_client(&ix->notifier);

	error = ix_mode_change(ix, IX_MODE_STANDBY);
	if (error)
	{
		dev_err(&ix->spi->dev, "[ERROR] IX_CMD_CMD_STBY_EN failed.\n");
		goto err_cdev;
	}

	up(&ix->mutex);
	fp_get(FP_IDEX);
	return 0;

err_cdev:
	cdev_del(&ix->cdev);
err_chrdev:
	unregister_chrdev_region(ix->devno, 1);
err_sysf_3:
	sysfs_remove_group(&spi->dev.kobj, &ix_reg_attr_group);
err_sysf_2:
	sysfs_remove_group(&spi->dev.kobj, &ix_diag_attr_group);
err_sysf_1:
	sysfs_remove_group(&spi->dev.kobj, &ix_adc_attr_group);
err:
	ix_cleanup(ix);
	spi_set_drvdata(spi, NULL);
	return error;
}


static int ix_remove(struct spi_device *spi)
{
	struct ix_data *ix = spi_get_drvdata(spi);

	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);
	fb_unregister_client(&ix->notifier);
	sysfs_remove_group(&ix->spi->dev.kobj, &ix_reg_attr_group);
	sysfs_remove_group(&ix->spi->dev.kobj, &ix_adc_attr_group);
	sysfs_remove_group(&ix->spi->dev.kobj, &ix_diag_attr_group);

	ix_mode_change(ix, IX_MODE_STANDBY);

	cdev_del(&ix->cdev);
	unregister_chrdev_region(ix->devno, 1);
	ix_cleanup(ix);
	spi_set_drvdata(spi, NULL);

	return 0;
}


static int ix_suspend(struct device *dev)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);
	return 0;
}


static int ix_resume(struct device *dev)
{
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ix_early_suspend(struct early_suspend *h)
{
	struct ix_data * ix;
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);
	ix = container_of(h, struct ix_data, early_suspend);

	ix->isSleep = true;

	ix_mode_change(ix, IX_MODE_STANDBY);
	return;
}


static void ix_late_resume(struct early_suspend *h)
{
	struct ix_data * ix;
	DEBUG_PRINT(KERN_INFO "[INFO] %s:%i %s\n", __FILE__, __LINE__, __func__);
	ix = container_of(h, struct ix_data, early_suspend);

	dev_info(&ix->spi->dev, "[INFO] ix_late_resume()\n");

	ix->isSleep = false;

	if (down_interruptible(&ix->mutex))
		return;

	up(&ix->mutex);

	return;
}
#endif

static struct of_device_id ix_of_match_table[] = {
	{
		.compatible = "ix_fingerprint",
	},
	{},
};

static struct spi_driver ix_driver = {
	.driver = {
		.name	= IX_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm 	= &ix_pm,
		.of_match_table = ix_of_match_table,
	},
	.probe	= ix_probe,
	.remove	= ix_remove
};


module_spi_driver(ix_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Crucialsoft Fingerprint <mskim@crucialtec.com>");
MODULE_DESCRIPTION("IX BTP sensor driver.");

