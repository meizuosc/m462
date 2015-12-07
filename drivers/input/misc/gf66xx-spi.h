#ifndef __GF66XX_SPI_H
#define __GF66XX_SPI_H

#include <linux/types.h>


#define GF66XX_FASYNC 		1//If support fasync mechanism.


/********************GF66XX Mapping**********************/
#define GF66XX_BASE        (0x8000)
#define GF66XX_OFFSET(x)   (GF66XX_BASE + x)

#define GF66XX_VERSION			GF66XX_OFFSET(0)
#define GF66XX_CONFIG_DATA  		GF66XX_OFFSET(0x40)
#define GF66XX_MIXER_DATA			GF66XX_OFFSET(0x140)
#define GF66XX_BUFFER_STATUS		GF66XX_OFFSET(0x340)
#define GF66XX_BUFFER_DATA		GF66XX_OFFSET(0x341)
#define	GF66XX_MODE_STATUS		GF66XX_OFFSET(0x043)

#define GF66XX_BUF_STA_MASK		(0x1<<7)
#define	GF66XX_BUF_STA_READY		(0x1<<7)
#define	GF66XX_BUF_STA_BUSY		(0x0<<7)

#define	GF66XX_IMAGE_MASK			(0x1<<6)
#define	GF66XX_IMAGE_ENABLE		(0x1)
#define	GF66XX_IMAGE_DISABLE		(0x0)

#define	GF66XX_KEY_MASK			(0x1<<5)
#define	GF66XX_KEY_ENABLE			(0x1)
#define	GF66XX_KEY_DISABLE		(0x0)

#define	GF66XX_KEY_STA			(0x1<<4)

typedef enum {
	GF66XX_IMAGE_MODE = 0,
	GF66XX_KEY_MODE,
	GF66XX_SLEEP_MODE,
	GF66XX_FF_MODE,
	GF66XX_DEBUG_MODE = 0x56
}MODE;

/**********************GF66XX ops****************************/
#define GF66XX_W          0xF0
#define GF66XX_R          0xF1
#define GF66XX_WDATA_OFFSET	(0x3)
#define GF66XX_RDATA_OFFSET	(0x4)
/**********************************************************/

/**********************IO Magic**********************/
#define  GF66XX_IOC_MAGIC    'g'  //define magic number
struct gf66xx_ioc_transfer {
	u8	cmd;
	u8 reserve;
	u16	addr;
	u32 len;
	u8 *buf;
};

struct gf66xx_software_version {
	u32 magic;		/*magic number*/
	u32	is_update;	/*check software version;  (is_update=1 ==> update already)*/
};
//define commands
/*read/write GF66XX registers*/
#define  GF66XX_IOC_CMD			_IOWR(GF66XX_IOC_MAGIC, 1, struct gf66xx_ioc_transfer)
#define  GF66XX_IOC_REINIT		_IO(GF66XX_IOC_MAGIC, 0)
#define  GF66XX_IOC_SETSPEED	_IOW(GF66XX_IOC_MAGIC, 2, u32)
#define  GF66XX_IOC_STOPTIMER	_IO(GF66XX_IOC_MAGIC, 3)
#define  GF66XX_IOC_STARTTIMER	_IO(GF66XX_IOC_MAGIC, 4)
#define  GF66XX_IOC_SETPULSE	_IOW(GF66XX_IOC_MAGIC, 5,u32)	/*Migrate the cpu to big and lock the big cpu at a higher freq*/
#define  GF66XX_IOC_SETMODE		_IOW(GF66XX_IOC_MAGIC, 6,u32)
#define  GF66XX_IOC_GETCHIPID	_IOR(GF66XX_IOC_MAGIC, 7,unsigned int *)
#define  GF66XX_IOC_SETVERSION	_IOW(GF66XX_IOC_MAGIC, 8,u32)
#define  GF66XX_IOC_SHOWVERSION	_IOR(GF66XX_IOC_MAGIC, 9,u32)

#define  GF66XX_IOC_MAXNR    10

/*******************Refering to hardware platform*****************************/
#define 	GF66XX_RST_PIN   	EXYNOS4_GPX1(5)
#define 	GF66XX_IRQ_PIN   	EXYNOS4_GPX1(4)
#define 	GF66XX_IRQ_NUM   	gpio_to_irq(GF66XX_IRQ_PIN)
#define		GF66XX_MISO_PIN	EXYNOS4_GPB(2)

struct gf66xx_dev {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	struct input_dev        *input;

	struct workqueue_struct *spi_wq;
	struct work_struct     spi_work;
	struct timer_list gf66xx_timer;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned		users;
	u8			*buffer;
	u8			buf_status;
	u8 			device_available;
	u8			software_available;
	int			reset_gpio;
	int			irq_gpio;
	u32 		cs_gpio;
	u32 		update_sw_gpio;
	MODE 		mode;
	u8			config_need_clear;
#ifdef GF66XX_FASYNC
	struct  fasync_struct *async;
	struct regulator *gx_power;
	struct notifier_block notifier;
#endif
};

#endif
