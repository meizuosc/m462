/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is video functions
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/videonode.h>
#include <media/exynos_mc.h>
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
#include <linux/i2c.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/pinctrl-samsung.h>

#include "fimc-is-video.h"
#include "fimc-is-dt.h"
#include "fimc-is-device-eeprom.h"
#include "fimc-is-core.h"

#define FIMC_IS_MAX_CAL_SIZE	(64 * 1024)
static u32 eeprom_status = FIMC_IS_EEPROM_STATE_INIT;

//static char u8IndexReadArrayBuf[36] = {0};
//static char u8InformationArrayBuf[28] = {0};
//static char u8awbArrayBuf[20] = {0};
//static char u8afArrayBuf[40] = {0};
//static char u8lscArrayBuf[6624] = {0};

static char calibration_buf[FIMC_IS_MAX_CAL_SIZE];

static int fimc_is_eeprom_power_on(struct fimc_is_device_sensor *device)
{
	int ret = 0;
	struct exynos_platform_fimc_is_sensor *pdata;

	BUG_ON(!device);
	BUG_ON(!device->pdev);
	BUG_ON(!device->pdata);
	BUG_ON(!device->private_data);

	pdata = device->pdata;

	if (test_bit(FIMC_IS_SENSOR_GPIO_ON, &device->state)) {
		merr("%s : already gpio on", device, __func__);
		goto p_err;
	}

	if (!pdata->gpio_cfg) {
		merr("gpio_cfg is NULL", device);
		ret = -EINVAL;
		goto p_err;
	}

	ret = pdata->gpio_cfg(device->pdev, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON);
	if (ret) {
		merr("gpio_cfg is fail(%d)", device, ret);
		goto p_err;
	}

	set_bit(FIMC_IS_SENSOR_GPIO_ON, &device->state);
	
p_err:
	return ret;

}

static int fimc_is_eeprom_power_off(struct fimc_is_device_sensor *device)
{
	int ret = 0;
	struct exynos_platform_fimc_is_sensor *pdata;

	BUG_ON(!device);
	BUG_ON(!device->pdev);
	BUG_ON(!device->pdata);

	pdata = device->pdata;

	if (!test_bit(FIMC_IS_SENSOR_GPIO_ON, &device->state)) {
		merr("%s : already gpio off", device, __func__);
		goto p_err;
	}

	if (!pdata->gpio_cfg) {
		merr("gpio_cfg is NULL", device);
		ret = -EINVAL;
		goto p_err;
	}

	ret = pdata->gpio_cfg(device->pdev, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_OFF);
	if (ret) {
		merr("gpio_cfg is fail(%d)", device, ret);
		goto p_err;
	}

	clear_bit(FIMC_IS_SENSOR_GPIO_ON, &device->state);

p_err:
	return ret;
}

static int fimc_is_i2c_read(struct i2c_client *client, void *buf, u32 addr, size_t size)
{
	const u32 addr_size = 2, max_retry = 5;
	u8 addr_buf[addr_size];
	int retries = max_retry;
	int ret = 0;

	if (!client) {
		pr_info("%s: client is null\n", __func__);
		return -ENODEV;
	}

	/* Send addr */
	addr_buf[0] = ((u16)addr) >> 8;
	addr_buf[1] = (u8)addr;

	for (retries = max_retry; retries > 0; retries--) {
		ret = i2c_master_send(client, addr_buf, addr_size);
		if (likely(addr_size == ret))
			break;
		usleep_range(1000, 1000);
	}
	if (unlikely(retries < max_retry))
		/* logging*/;

	if (unlikely(ret < 0)) {
		pr_err("%s: error %d, fail to write 0x%04X\n", __func__, ret, addr);
		return ret;
	}

	/* Receive data */
	for (retries = max_retry; retries > 0; retries--) {
		ret = i2c_master_recv(client, buf, size);
		if (likely(ret == size))
			break;
		usleep_range(1000, 1000);
	}

	if (unlikely(retries < max_retry))
		/* logging*/;

	if (unlikely(ret < 0)) {
		pr_err("%s: error %d, fail to read 0x%04X\n", __func__, ret, addr);
		return ret;
	}

	return 0;
}

int fimc_is_eeprom_read(struct fimc_is_device_sensor *device)
{
	int ret = 0;
	int i = 0;
	u32 checksumofidx = 0;
	u32 info_start = 0;
	u32 info_size = 0;
	u32 awb_start = 0;
	u32 awb_size = 0;
	u32 af_start = 0;
	u32 af_size = 0;
	u32 lsc_start = 0;
	u32 lsc_size = 0;
	u32 checksumidx = 0;
	u32 checksuminfo = 0;
	u32 checksumawb = 0;
	u32 checksumaf = 0;
	u32 checksumlsc = 0;

#ifdef EEPROM_CHECKSUM_DEBUG
	//infro section
	u32 manufacture_id = 0;
	u32 module_version = 0;
	u32 year = 0;
	u32 month = 0;
	u32 day = 0;
	u32 reserverd = 0;

	//awb section
	u32 rn_coeff = 0;
	u32 rn_const = 0;
	u32 bn_coeff = 0;
	u32 bn_const = 0;
	
	//af section
	u32 af_inf_pos_typ = 0;
	u32 af_inf_pos_worst = 0;
	u32 af_macro_pos_typ = 0;
	u32 af_macro_pos_worst = 0;
	u32 af_default = 0;
	u32 vcm_bottom = 0;
	u32 vcm_top = 0;
	u32 vcm_bias = 0;
	u32 vcm_offset = 0;

	//lsc section
	u32 lsc_ver = 0;
	u32 lsc_length = 0;
	u32 lsc_ori = 0;
	u32 lsc_i0 = 0;
	u32 lsc_j0 = 0;
	u32 lsc_scale = 0;
	u32 lsc_acoff = 0;
	u32 lsc_bcoff = 0;
#endif

	struct exynos_platform_fimc_is_sensor *pdata;
	struct fimc_is_core *core;
	struct i2c_client			*client = NULL;
	struct mz_module_info *module_info;

	BUG_ON(!device);
	BUG_ON(!device->pdev);
	BUG_ON(!device->pdata);
	BUG_ON(!device->private_data);

	core = device->private_data;
	pdata = device->pdata;
	module_info  = &(core->sensor[REAR_SENSOR_ID].mz_modu_info);

	BUG_ON(!core->client0);
	BUG_ON(!core->client1);

	//Power On
	fimc_is_eeprom_power_on(device);

	pr_info("%s: start read \n", __func__);

	ret = fimc_is_i2c_read(core->client0, &calibration_buf[0], 0x00, EEPROM_SECTION1_SIZE);

	if (unlikely(ret)) {
		ret = fimc_is_i2c_read(core->client1, &calibration_buf[0], 0x00, EEPROM_SECTION1_SIZE);

		if (unlikely(ret)) {
			err("failed to fimc_is_i2c_read (%d)\n", ret);
			eeprom_status = FIMC_IS_EEPROM_STATE_I2CFAIL;
			/*
			* Assign default value.
			*/
			client = core->client0;
			ret = -EINVAL;
			goto exit;
		}

		client = core->client1;
		module_info ->vendor_id = SHARP_ID;
	} else {
		client = core->client0;
		module_info ->vendor_id = LITEON_ID;
	}

	checksumofidx 	= ( calibration_buf[0] ) | ( calibration_buf[1] << 8 ) | ( calibration_buf[2] << 16) | ( calibration_buf[3] << 24) ;
	info_start 		= ( calibration_buf[4] ) | ( calibration_buf[5] << 8 ) | ( calibration_buf[6] << 16) | ( calibration_buf[7] << 24) ;
	info_size 		= ( calibration_buf[8] ) | ( calibration_buf[9] << 8 ) | ( calibration_buf[10] << 16) | ( calibration_buf[11] << 24) ;
	awb_start 		= ( calibration_buf[12] ) | ( calibration_buf[13] << 8 ) | ( calibration_buf[14] << 16) | ( calibration_buf[15] << 24) ;
	awb_size 		= ( calibration_buf[16] ) | ( calibration_buf[17] << 8 ) | ( calibration_buf[18] << 16) | ( calibration_buf[19] << 24) ;
	af_start 		= ( calibration_buf[20] ) | ( calibration_buf[21] << 8 ) | ( calibration_buf[22] << 16) | ( calibration_buf[23] << 24) ;
	af_size 		= ( calibration_buf[24] ) | ( calibration_buf[25] << 8 ) | ( calibration_buf[26] << 16) | ( calibration_buf[27] << 24) ;
	lsc_start 		= ( calibration_buf[28] ) | ( calibration_buf[29] << 8 ) | ( calibration_buf[30] << 16) | ( calibration_buf[31] << 24) ;
	lsc_size 		= ( calibration_buf[32] ) | ( calibration_buf[33] << 8 ) | ( calibration_buf[34] << 16) | ( calibration_buf[35] << 24) ;

	module_info->project_id = calibration_buf[0x0];

#ifdef EEPROM_CHECKSUM_DEBUG
	pr_info("EEPROM Read checksumofidx 0x%x \n",checksumofidx);
	pr_info("EEPROM Read info_start 0x%x \n",info_start);
	pr_info("EEPROM Read info_size 0x%x \n",info_size);
	pr_info("EEPROM Read awb_start 0x%x \n",awb_start);
	pr_info("EEPROM Read awb_size 0x%x \n",awb_size);
	pr_info("EEPROM Read af_start 0x%x \n",af_start);
	pr_info("EEPROM Read af_size 0x%x \n",af_size);
	pr_info("EEPROM Read lsc_start 0x%x \n",lsc_start);
	pr_info("EEPROM Read lsc_size 0x%x \n",lsc_size);
#endif

	for(i=EEPROM_CHECKSUM_SIZE; i<EEPROM_SECTION1_SIZE; i++) {
		checksumidx += calibration_buf[i];
	}

	if(checksumofidx != checksumidx) {
		err("EEPROM section index check sum fail");
		eeprom_status = FIMC_IS_EEPROM_STATE_CHECKSUM_FAIL;
		ret = -EINVAL;
		goto exit;
	}

	checksumofidx = 0;

	ret = fimc_is_i2c_read(client, &calibration_buf[info_start], info_start, info_size);

	if (unlikely(ret)) {
		err("failed to fimc_is_i2c_read (%d)\n", ret);
		eeprom_status = FIMC_IS_EEPROM_STATE_I2CFAIL;
		ret = -EINVAL;
		goto exit;
	} 

	memcpy(module_info->project_name, &calibration_buf[info_start + 0x4],
		sizeof(module_info->project_name) - 1);
	memcpy(module_info->map_version, &calibration_buf[info_start + 0x10],
		sizeof(module_info->map_version) - 1);
	module_info->manufacture_id = calibration_buf[info_start + 0x14];
	module_info->module_version = calibration_buf[info_start + 0x15];

	pr_info("%s(), project's name/id:%s/0x%x, map_version:%s, manufacture_id:0x%x,"
		" module_version:0x%x\n",
		__func__, module_info->project_name, module_info->project_id,
		module_info->map_version,
		module_info->manufacture_id, module_info->module_version);

	checksumofidx 		= ( calibration_buf[info_start+0] ) | ( calibration_buf[info_start+1] << 8 ) | ( calibration_buf[info_start+2] << 16) | ( calibration_buf[info_start+3] << 24) ;

#ifdef EEPROM_CHECKSUM_DEBUG
	manufacture_id 		= ( calibration_buf[info_start+20] ) ;
	module_version		= ( calibration_buf[info_start+21] ) ;
	year 				= ( calibration_buf[info_start+22] ) ;
	month 				= ( calibration_buf[info_start+23] ) ;
	day 				= ( calibration_buf[info_start+24] ) ;
	reserverd 			= ( calibration_buf[info_start+25] ) | ( calibration_buf[info_start+26] << 8 ) | ( calibration_buf[info_start+27] << 16) ;

	pr_info("EEPROM Read checksumofidx 0x%x \n",checksumofidx);
	pr_info("EEPROM Read manufacture_id 0x%x \n",manufacture_id);
	pr_info("EEPROM Read module_version 0x%x \n",module_version);
	pr_info("EEPROM Read year 0x%x \n",year);
	pr_info("EEPROM Read month 0x%x \n",month);
	pr_info("EEPROM Read day 0x%x \n",day);
	pr_info("EEPROM Read reserverd 0x%x \n",reserverd);
#endif

	for(i=EEPROM_CHECKSUM_SIZE; i<info_size; i++) {
		checksuminfo += calibration_buf[i+info_start];
	}

	if(checksumofidx != checksuminfo) {
		err("EEPROM section information check sum fail");
		eeprom_status = FIMC_IS_EEPROM_STATE_CHECKSUM_FAIL;
		ret = -EINVAL;
		goto exit;
	}

	checksumofidx = 0;

	ret = fimc_is_i2c_read(client, &calibration_buf[0+awb_start], awb_start, awb_size);

	if (unlikely(ret)) {
		err("failed to fimc_is_i2c_read (%d)\n", ret);
		eeprom_status = FIMC_IS_EEPROM_STATE_I2CFAIL;
		ret = -EINVAL;
		goto exit;
	}

	checksumofidx	= ( calibration_buf[0+awb_start] ) | ( calibration_buf[1+awb_start] << 8 ) | ( calibration_buf[2+awb_start] << 16) | ( calibration_buf[3+awb_start] << 24) ;

#ifdef EEPROM_CHECKSUM_DEBUG
	rn_coeff		= ( calibration_buf[4+awb_start] ) | ( calibration_buf[5+awb_start] << 8 ) | ( calibration_buf[6+awb_start] << 16) | ( calibration_buf[7+awb_start] << 24) ;
	rn_const		= ( calibration_buf[8+awb_start] ) | ( calibration_buf[9+awb_start] << 8 ) | ( calibration_buf[10+awb_start] << 16) | ( calibration_buf[11+awb_start] << 24) ;
	bn_coeff		= ( calibration_buf[12+awb_start] ) | ( calibration_buf[13+awb_start] << 8 ) | ( calibration_buf[14+awb_start] << 16) | ( calibration_buf[15+awb_start] << 24) ;
	bn_const		= ( calibration_buf[16+awb_start] ) | ( calibration_buf[17+awb_start] << 8 ) | ( calibration_buf[18+awb_start] << 16) | ( calibration_buf[19+awb_start] << 24) ;

	pr_info("EEPROM Read checksumofidx 0x%x \n",checksumofidx);
	pr_info("EEPROM Read rn_coeff 0x%x \n",rn_coeff);
	pr_info("EEPROM Read rn_const 0x%x \n",rn_const);
	pr_info("EEPROM Read bn_coeff 0x%x \n",bn_coeff);
	pr_info("EEPROM Read bn_const 0x%x \n",bn_const);
#endif


	for(i=EEPROM_CHECKSUM_SIZE; i<awb_size; i++) {
		checksumawb += calibration_buf[i+awb_start];
	}

	if(checksumofidx != checksumawb) {
		err("EEPROM section awb check sum fail");
		eeprom_status = FIMC_IS_EEPROM_STATE_CHECKSUM_FAIL;
		ret = -EINVAL;
		goto exit;
	}

	checksumofidx = 0;
	ret = fimc_is_i2c_read(client, &calibration_buf[0 + af_start], af_start, af_size);
	if (unlikely(ret)) {
		err("failed to fimc_is_i2c_read (%d)\n", ret);
		eeprom_status = FIMC_IS_EEPROM_STATE_I2CFAIL;
		ret = -EINVAL;
		goto exit;
	} 
	
	checksumofidx			= ( calibration_buf[0+ af_start] ) | ( calibration_buf[1+ af_start] << 8 ) | ( calibration_buf[2+ af_start] << 16) | ( calibration_buf[3+ af_start] << 24) ;

#ifdef EEPROM_CHECKSUM_DEBUG
	af_inf_pos_typ			= ( calibration_buf[4+ af_start] ) | ( calibration_buf[5+ af_start] << 8 ) | ( calibration_buf[6+ af_start] << 16) | ( calibration_buf[7+ af_start] << 24) ;
	af_inf_pos_worst		= ( calibration_buf[8+ af_start] ) | ( calibration_buf[9+ af_start] << 8 ) | ( calibration_buf[10+ af_start] << 16) | ( calibration_buf[11+ af_start] << 24) ;
	af_macro_pos_typ		= ( calibration_buf[12+ af_start] ) | ( calibration_buf[13+ af_start] << 8 ) | ( calibration_buf[14+ af_start] << 16) | ( calibration_buf[15+ af_start] << 24) ;
	af_macro_pos_worst 		= ( calibration_buf[16+ af_start] ) | ( calibration_buf[17+ af_start] << 8 ) | ( calibration_buf[18+ af_start] << 16) | ( calibration_buf[19+ af_start] << 24) ;
	af_default				= ( calibration_buf[20+ af_start] ) | ( calibration_buf[21+ af_start] << 8 ) | ( calibration_buf[22+ af_start] << 16) | ( calibration_buf[23+ af_start] << 24) ;
	vcm_bottom				= ( calibration_buf[24+ af_start] ) | ( calibration_buf[25+ af_start] << 8 ) | ( calibration_buf[26+ af_start] << 16) | ( calibration_buf[27+ af_start] << 24) ;
	vcm_top					= ( calibration_buf[28+ af_start] ) | ( calibration_buf[29+ af_start] << 8 ) | ( calibration_buf[30+ af_start] << 16) | ( calibration_buf[31+ af_start] << 24) ;
	vcm_bias 				= ( calibration_buf[32+ af_start] ) | ( calibration_buf[33+ af_start] << 8 ) | ( calibration_buf[34+ af_start] << 16) | ( calibration_buf[35+ af_start] << 24) ;
	vcm_offset 				= ( calibration_buf[36+ af_start] ) | ( calibration_buf[37+ af_start] << 8 ) | ( calibration_buf[38+ af_start] << 16) | ( calibration_buf[39+ af_start] << 24) ;

	pr_info("EEPROM Read af_inf_pos_typ 0x%x \n",af_inf_pos_typ);
	pr_info("EEPROM Read af_inf_pos_worst 0x%x \n",af_inf_pos_worst);
	pr_info("EEPROM Read af_macro_pos_typ 0x%x \n",af_macro_pos_typ);
	pr_info("EEPROM Read af_macro_pos_worst 0x%x \n",af_macro_pos_worst);
	pr_info("EEPROM Read af_default 0x%x \n",af_default);
	pr_info("EEPROM Read vcm_bottom 0x%x \n",vcm_bottom);
	pr_info("EEPROM Read vcm_top 0x%x \n",vcm_top);
	pr_info("EEPROM Read vcm_bias 0x%x \n",vcm_bias);
	pr_info("EEPROM Read vcm_offset 0x%x \n",vcm_offset);
#endif

	for(i=EEPROM_CHECKSUM_SIZE; i<af_size; i++) {
		checksumaf += calibration_buf[i+ af_start];
	}

	if(checksumofidx != checksumaf) {
		err("EEPROM section af check sum fail");
		eeprom_status = FIMC_IS_EEPROM_STATE_CHECKSUM_FAIL;
		ret = -EINVAL;
		goto exit;
	}

	checksumofidx = 0;
	ret = fimc_is_i2c_read(client, &calibration_buf[0 + lsc_start], lsc_start, lsc_size);
	if (unlikely(ret)) {
		err("failed to fimc_is_i2c_read (%d)\n", ret);
		eeprom_status = FIMC_IS_EEPROM_STATE_I2CFAIL;
		ret = -EINVAL;
		goto exit;
	} 

	checksumofidx		= ( calibration_buf[0 + lsc_start] ) | ( calibration_buf[1+ lsc_start] << 8 ) | ( calibration_buf[2+ lsc_start] << 16) | ( calibration_buf[3+ lsc_start] << 24) ;

#ifdef EEPROM_CHECKSUM_DEBUG
	lsc_ver				= ( calibration_buf[4+ lsc_start] ) | ( calibration_buf[5+ lsc_start] << 8 ) | ( calibration_buf[6+ lsc_start] << 16) | ( calibration_buf[7+ lsc_start] << 24) ;
	lsc_length			= ( calibration_buf[8+ lsc_start] ) | ( calibration_buf[9+ lsc_start] << 8 ) | ( calibration_buf[10+ lsc_start] << 16) | ( calibration_buf[11+ lsc_start] << 24) ;
	lsc_ori				= ( calibration_buf[12+ lsc_start] ) | ( calibration_buf[13+ lsc_start] << 8 ) | ( calibration_buf[14+ lsc_start] << 16) | ( calibration_buf[15+ lsc_start] << 24) ;
	lsc_i0				= ( calibration_buf[16+ lsc_start] ) | ( calibration_buf[17+ lsc_start] << 8 ) | ( calibration_buf[18+ lsc_start] << 16) | ( calibration_buf[19+ lsc_start] << 24) ;
	lsc_j0				= ( calibration_buf[20+ lsc_start] ) | ( calibration_buf[21+ lsc_start] << 8 ) | ( calibration_buf[22+ lsc_start] << 16) | ( calibration_buf[23+ lsc_start] << 24) ;
	lsc_scale			= ( calibration_buf[24+ lsc_start] ) | ( calibration_buf[25+ lsc_start] << 8 ) | ( calibration_buf[26+ lsc_start] << 16) | ( calibration_buf[27+ lsc_start] << 24) ;
	lsc_acoff 			= ( calibration_buf[28+ lsc_start] ) | ( calibration_buf[29+ lsc_start] << 8 ) | ( calibration_buf[30+ lsc_start] << 16) | ( calibration_buf[31+ lsc_start] << 24) ;
	lsc_bcoff			= ( calibration_buf[32+ lsc_start] ) | ( calibration_buf[33+ lsc_start] << 8 ) | ( calibration_buf[34+ lsc_start] << 16) | ( calibration_buf[35+ lsc_start] << 24) ;

	pr_info("EEPROM Read lsc_ver 0x%x \n",lsc_ver);
	pr_info("EEPROM Read lsc_length 0x%x \n",lsc_length);
	pr_info("EEPROM Read lsc_ori 0x%x \n",lsc_ori);
	pr_info("EEPROM Read lsc_i0 0x%x \n",lsc_i0);
	pr_info("EEPROM Read lsc_j0 0x%x \n",lsc_j0);
	pr_info("EEPROM Read lsc_scale 0x%x \n",lsc_scale);
	pr_info("EEPROM Read lsc_acoff 0x%x \n",lsc_acoff);
	pr_info("EEPROM Read lsc_bcoff 0x%x \n",lsc_bcoff);
#endif

	for(i=EEPROM_CHECKSUM_SIZE; i<lsc_size; i++) {
		checksumlsc+= calibration_buf[i+ lsc_start];
	}

	if(checksumofidx != checksumlsc) {
		err("EEPROM section lsc check sum fail");
		eeprom_status = FIMC_IS_EEPROM_STATE_CHECKSUM_FAIL;
		ret = -EINVAL;
		goto exit;
	}


	eeprom_status = FIMC_IS_EEPROM_STATE_READONE;
	module_info->valid = true;
	pr_info("%s: end read \n", __func__);

exit:
	//Power Off
	{
		struct platform_device *i2c_pdev = to_platform_device(client->adapter->dev.parent);
		struct s3c24xx_i2c *i2c_pdata = (struct s3c24xx_i2c *)platform_get_drvdata(i2c_pdev);
		devm_free_irq((client->adapter->dev.parent), i2c_pdata->irq, i2c_pdata);
	}
	fimc_is_eeprom_power_off(device);

	return ret;
}


u32 fimc_is_eeprom_check_state(void)
{
	return eeprom_status;
}

int fimc_is_eeprom_get_cal_buf(char **buf)
{
	*buf = &calibration_buf[0];
	return 0;
}

