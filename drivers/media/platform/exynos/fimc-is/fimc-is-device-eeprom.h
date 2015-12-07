#ifndef FIMC_IS_DEVICE_EEPROM_H
#define FIMC_IS_DEVICE_EEPROM_H

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/videonode.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/videodev2_exynos_media.h>
#include <linux/v4l2-mediabus.h>
#include <linux/bug.h>
#include <linux/syscalls.h>
#include <linux/vmalloc.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/regulator/consumer.h>

#include "fimc-is-core.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"
#include "fimc-is-regs.h"
#include "fimc-is-err.h"
#include "fimc-is-video.h"

#include "fimc-is-device-sensor.h"
#include "fimc-is-device-ischain.h"

#define EEPROM_SECTION1_SIZE 36
#define EEPROM_CHECKSUM_SIZE 4

//#define EEPROM_CHECKSUM_DEBUG

enum s3c24xx_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};

struct s3c24xx_i2c {
	struct list_head	node;
	wait_queue_head_t	wait;
	unsigned int            quirks;
	unsigned int		need_hw_init;
	unsigned int		suspended:1;

	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;

	unsigned int		tx_setup;
	unsigned int		irq;

	enum s3c24xx_i2c_state	state;
	unsigned long		clkrate;

	void __iomem		*regs;
	struct clk		*rate_clk;
	struct clk		*clk;
	struct device		*dev;
	struct i2c_adapter	adap;

	struct s3c2410_platform_i2c	*pdata;
	int			gpios[2];
	struct pinctrl          *pctrl;
};

enum fimc_is_eeprom_state {
	FIMC_IS_EEPROM_STATE_INIT,
	FIMC_IS_EEPROM_STATE_READONE,
	FIMC_IS_EEPROM_STATE_I2CFAIL,
	FIMC_IS_EEPROM_STATE_CHECKSUM_FAIL,
};

int fimc_is_eeprom_read(struct fimc_is_device_sensor *device);
int fimc_is_eeprom_get_cal_buf(char **buf);
u32 fimc_is_eeprom_check_state(void);

#endif /* FIMC_IS_DEVICE_EEPROM_H */

