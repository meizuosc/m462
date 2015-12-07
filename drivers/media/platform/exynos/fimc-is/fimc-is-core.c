/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is core functions
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
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
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <mach/regs-clock.h>
#include <linux/pm_qos.h>
#include <linux/bug.h>
#include <linux/v4l2-mediabus.h>
#include <mach/devfreq.h>
#include <mach/bts.h>
#include <linux/gpio.h>
#include <plat/gpio-cfg.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include "fimc-is-core.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"
#include "fimc-is-regs.h"
#include "fimc-is-err.h"
#include "fimc-is-framemgr.h"
#include "fimc-is-dt.h"
#include "fimc-is-resourcemgr.h"
#include "fimc-is-clk-gate.h"
#include "fimc-is-dvfs.h"

#include "sensor/fimc-is-device-2p2.h"
#include "sensor/fimc-is-device-2p2_12m.h"
#include "sensor/fimc-is-device-3h5.h"
#include "sensor/fimc-is-device-3h7.h"
#include "sensor/fimc-is-device-3h7_sunny.h"
#include "sensor/fimc-is-device-3l2.h"
#include "sensor/fimc-is-device-4e5.h"
#include "sensor/fimc-is-device-6a3.h"
#include "sensor/fimc-is-device-6b2.h"
#include "sensor/fimc-is-device-8b1.h"
#include "sensor/fimc-is-device-6d1.h"
#include "sensor/fimc-is-device-imx135.h"
#include "sensor/fimc-is-device-imx175.h"
#include "sensor/fimc-is-device-imx240.h"
#include "sensor/fimc-is-device-4h5.h"
#include "sensor/fimc-is-device-3l2.h"
#include "sensor/fimc-is-device-2p2.h"
#include "sensor/fimc-is-device-imx220.h"
#include "sensor/fimc-is-device-ov5693.h"

#ifdef USE_OWN_FAULT_HANDLER
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0))
#include <linux/exynos_iovmm.h>
#else
#include <plat/sysmmu.h>
#endif
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
#define PM_QOS_CAM_THROUGHPUT	PM_QOS_RESERVED
#endif

struct fimc_is_from_info *sysfs_finfo = NULL;
struct fimc_is_from_info *sysfs_pinfo = NULL;

struct class *camera_class = NULL;
struct device *camera_front_dev = NULL;
struct device *camera_rear_dev = NULL;
struct device *fimc_is_dev = NULL;
struct fimc_is_core *sysfs_core;

extern bool crc32_fw_check;
extern bool crc32_c1_fw_check;
extern bool crc32_check;
extern bool crc32_c1_check;
extern bool crc32_check_factory;
extern bool crc32_c1_check_factory;
extern bool fw_version_crc_check;

extern struct pm_qos_request exynos_isp_qos_int;
extern struct pm_qos_request exynos_isp_qos_mem;
extern struct pm_qos_request exynos_isp_qos_cam;
extern struct pm_qos_request exynos_isp_qos_disp;

extern int fimc_is_3a0_video_probe(void *data);
extern int fimc_is_3a1_video_probe(void *data);
extern int fimc_is_isp_video_probe(void *data);
extern int fimc_is_scc_video_probe(void *data);
extern int fimc_is_scp_video_probe(void *data);
extern int fimc_is_vdc_video_probe(void *data);
extern int fimc_is_vdo_video_probe(void *data);
extern int fimc_is_3a0c_video_probe(void *data);
extern int fimc_is_3a1c_video_probe(void *data);

/* sysfs global variable for debug */
struct fimc_is_sysfs_debug sysfs_debug;

static int fimc_is_ischain_allocmem(struct fimc_is_core *this)
{
	int ret = 0;
	void *fw_cookie;
	size_t fw_size =
#ifdef ENABLE_ODC
				SIZE_ODC_INTERNAL_BUF * NUM_ODC_INTERNAL_BUF +
#endif
#ifdef ENABLE_VDIS
				SIZE_DIS_INTERNAL_BUF * NUM_DIS_INTERNAL_BUF +
#endif
#ifdef ENABLE_TDNR
				SIZE_DNR_INTERNAL_BUF * NUM_DNR_INTERNAL_BUF +
#endif
			FIMC_IS_A5_MEM_SIZE;

	fw_size = PAGE_ALIGN(fw_size);
	dbg_core("Allocating memory for FIMC-IS firmware.\n");

	fw_cookie = vb2_ion_private_alloc(this->mem.alloc_ctx, fw_size, 1, 0);

	if (IS_ERR(fw_cookie)) {
		err("Allocating bitprocessor buffer failed");
		fw_cookie = NULL;
		ret = -ENOMEM;
		goto exit;
	}

	ret = vb2_ion_dma_address(fw_cookie, &this->minfo.dvaddr);
	if ((ret < 0) || (this->minfo.dvaddr  & FIMC_IS_FW_BASE_MASK)) {
		err("The base memory is not aligned to 64MB.");
		vb2_ion_private_free(fw_cookie);
		this->minfo.dvaddr = 0;
		fw_cookie = NULL;
		ret = -EIO;
		goto exit;
	}
	dbg_core("Device vaddr = %08x , size = %08x\n",
		this->minfo.dvaddr, FIMC_IS_A5_MEM_SIZE);

	this->minfo.kvaddr = (u32)vb2_ion_private_vaddr(fw_cookie);
	if (IS_ERR((void *)this->minfo.kvaddr)) {
		err("Bitprocessor memory remap failed");
		vb2_ion_private_free(fw_cookie);
		this->minfo.kvaddr = 0;
		fw_cookie = NULL;
		ret = -EIO;
		goto exit;
	}

	vb2_ion_sync_for_device(fw_cookie, 0, fw_size, DMA_BIDIRECTIONAL);

exit:
	info("[COR] Device virtual for internal: %08x\n", this->minfo.kvaddr);
	this->minfo.fw_cookie = fw_cookie;

	return ret;
}

static int fimc_is_ishcain_initmem(struct fimc_is_core *this)
{
	int ret = 0;
	u32 offset;

	dbg_core("fimc_is_init_mem - ION\n");

	ret = fimc_is_ischain_allocmem(this);
	if (ret) {
		err("Couldn't alloc for FIMC-IS firmware\n");
		ret = -ENOMEM;
		goto exit;
	}

	offset = FW_SHARED_OFFSET;
	this->minfo.dvaddr_fshared = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_fshared = this->minfo.kvaddr + offset;

	offset = FIMC_IS_A5_MEM_SIZE - FIMC_IS_REGION_SIZE;
	this->minfo.dvaddr_region = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_region = this->minfo.kvaddr + offset;

	offset = FIMC_IS_A5_MEM_SIZE;
#ifdef ENABLE_ODC
	this->minfo.dvaddr_odc = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_odc = this->minfo.kvaddr + offset;
	offset += (SIZE_ODC_INTERNAL_BUF * NUM_ODC_INTERNAL_BUF);
#else
	this->minfo.dvaddr_odc = 0;
	this->minfo.kvaddr_odc = 0;
#endif

#ifdef ENABLE_VDIS
	this->minfo.dvaddr_dis = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_dis = this->minfo.kvaddr + offset;
	offset += (SIZE_DIS_INTERNAL_BUF * NUM_DIS_INTERNAL_BUF);
#else
	this->minfo.dvaddr_dis = 0;
	this->minfo.kvaddr_dis = 0;
#endif

#ifdef ENABLE_TDNR
	this->minfo.dvaddr_3dnr = this->minfo.dvaddr + offset;
	this->minfo.kvaddr_3dnr = this->minfo.kvaddr + offset;
	offset += (SIZE_DNR_INTERNAL_BUF * NUM_DNR_INTERNAL_BUF);
#else
	this->minfo.dvaddr_3dnr = 0;
	this->minfo.kvaddr_3dnr = 0;
#endif

	dbg_core("fimc_is_init_mem done\n");

exit:
	return ret;
}

#if defined(CONFIG_ARM_EXYNOS5433_BUS_DEVFREQ) && defined(CONFIG_CPU_THERMAL_IPA)
static int fimc_is_mif_throttling_notifier(struct notifier_block *nb,
		unsigned long val, void *v)
{
	struct fimc_is_core *core = NULL;
	struct fimc_is_device_sensor *device = NULL;
	int i;

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core is null");
		goto exit;
	}

	for (i = 0; i < FIMC_IS_MAX_NODES; i++) {
		if (test_bit(FIMC_IS_SENSOR_OPEN, &core->sensor[i].state)) {
			device = &core->sensor[i];
			break;
		}
	}

	if (device && !test_bit(FIMC_IS_SENSOR_FRONT_DTP_STOP, &device->state))
		/* Set DTP */
		set_bit(FIMC_IS_MIF_THROTTLING_STOP, &device->force_stop);
	else
		err("any sensor is not opened");

exit:
	err("MIF: cause of mif_throttling, mif_qos is [%lu]!!!\n", val);

	return NOTIFY_OK;
}

static struct notifier_block exynos_fimc_is_mif_throttling_nb = {
	.notifier_call = fimc_is_mif_throttling_notifier,
};
#endif

static int fimc_is_suspend(struct device *dev)
{
	pr_debug("FIMC_IS Suspend\n");
	return 0;
}

static int fimc_is_resume(struct device *dev)
{
	pr_debug("FIMC_IS Resume\n");
	return 0;
}

#ifdef USE_OWN_FAULT_HANDLER
static void fimc_is_print_minfo(struct fimc_is_minfo *minfo)
{
	if (minfo) {
		err("dvaddr : 0x%08x, size : %d", minfo->dvaddr, minfo->size);
		err("dvaddr_debug  : 0x%08x, dvaddr_region : 0x%08x", minfo->dvaddr_debug, minfo->dvaddr_region);
		err("dvaddr_shared : 0x%08x, dvaddr_odc    : 0x%08x", minfo->dvaddr_shared, minfo->dvaddr_odc);
		err("dvaddr_dis    : 0x%08x, dvaddr_3dnr   : 0x%08x", minfo->dvaddr_dis, minfo->dvaddr_3dnr);
	} else {
		err("core->minfo is NULL");
	}
}

static void __fimc_is_fault_handler(struct device *dev)
{
	u32 i, j, k;
	struct fimc_is_core *core;
	struct fimc_is_device_sensor *sensor;
	struct fimc_is_device_ischain *ischain;
	struct fimc_is_framemgr *framemgr;

	core = dev_get_drvdata(dev);
	if (core) {

		fimc_is_print_minfo(&core->minfo);
		fimc_is_hw_logdump(&core->interface);
		/* dump FW page table 1nd(~16KB), 2nd(16KB~32KB) */
		fimc_is_hw_memdump(&core->interface,
			core->minfo.kvaddr + 0x010F8000 /* TTB_BASE ~ 32KB */,
			core->minfo.kvaddr + 0x010F8000 + 0x8000);

		/* REAR SENSOR */
		sensor = &core->sensor[0];
		if (test_bit(FIMC_IS_SENSOR_OPEN, &sensor->state)) {
			framemgr = &sensor->vctx->q_dst.framemgr;
			for (i = 0; i < FRAMEMGR_MAX_REQUEST; ++i) {
				pr_err("LITE0 BUF[%d][0] = %X, 0x%08X, 0x%08X\n", i,
					(u32)framemgr->frame[i].memory,
					framemgr->frame[i].dvaddr_buffer[0],
					framemgr->frame[i].kvaddr_buffer[0]);
			}
		}

		/* FRONT SENSOR */
		sensor = &core->sensor[1];
		if (test_bit(FIMC_IS_SENSOR_OPEN, &sensor->state)) {
			framemgr = &sensor->vctx->q_dst.framemgr;
			for (i = 0; i < FRAMEMGR_MAX_REQUEST; ++i) {
				pr_err("LITE1 BUF[%d][0] = %X, 0x%08X. 0x%08X\n", i,
					(u32)framemgr->frame[i].memory,
					framemgr->frame[i].dvaddr_buffer[0],
					framemgr->frame[i].kvaddr_buffer[0]);
			}
		}

		/* ISCHAIN */
		for (i = 0; i < FIMC_IS_MAX_NODES; i++) {
			if (test_bit(FIMC_IS_ISCHAIN_OPEN, &(core->ischain[i].state))) {
				ischain = &core->ischain[i];
				/* 3AA */
				if (test_bit(FIMC_IS_SUBDEV_START, &ischain->group_3aa.leader.state)) {
					framemgr = &ischain->group_3aa.leader.vctx->q_src.framemgr;
					for (j = 0; j < framemgr->frame_cnt; ++j) {
						for (k = 0; k < framemgr->frame[j].planes; k++) {
							pr_err("[3AA:%d] BUF[%d][%d] = %X, 0x%08X, 0x%08X\n",
								i, j, k,
								(u32)framemgr->frame[j].memory,
								framemgr->frame[j].dvaddr_buffer[k],
								framemgr->frame[j].kvaddr_buffer[k]);
						}
					}
				}
				/* 3AAC */
				if (test_bit(FIMC_IS_SUBDEV_START, &ischain->taac.state)) {
					framemgr = &ischain->taac.leader->vctx->q_dst.framemgr;
					for (j = 0; j < framemgr->frame_cnt; ++j) {
						for (k = 0; k < framemgr->frame[j].planes; k++) {
							pr_err("[3AAC:%d] BUF[%d][%d] = %X, 0x%08X, 0x%08X\n",
								i, j, k,
								(u32)framemgr->frame[j].memory,
								framemgr->frame[j].dvaddr_buffer[k],
								framemgr->frame[j].kvaddr_buffer[k]);
						}
					}
				}
				/* 3AAP */
				if (test_bit(FIMC_IS_SUBDEV_START, &ischain->taap.state)) {
					framemgr = &ischain->taap.leader->vctx->q_dst.framemgr;
					for (j = 0; j < framemgr->frame_cnt; ++j) {
						for (k = 0; k < framemgr->frame[j].planes; k++) {
							pr_err("[3AAP:%d] BUF[%d][%d] = %X, 0x%08X, 0x%08X\n",
								i, j, k,
								(u32)framemgr->frame[j].memory,
								framemgr->frame[j].dvaddr_buffer[k],
								framemgr->frame[j].kvaddr_buffer[k]);
						}
					}
				}
				/* ISP */
				if (test_bit(FIMC_IS_SUBDEV_START, &ischain->group_isp.leader.state)) {
					framemgr = &ischain->group_isp.leader.vctx->q_src.framemgr;
					for (j = 0; j < framemgr->frame_cnt; ++j) {
						for (k = 0; k < framemgr->frame[j].planes; k++) {
							pr_err("[ISP:%d] BUF[%d][%d] = %X, 0x%08X, 0x%08X\n",
								i, j, k,
								(u32)framemgr->frame[j].memory,
								framemgr->frame[j].dvaddr_buffer[k],
								framemgr->frame[j].kvaddr_buffer[k]);
						}
					}
				}
				/* SCC */
				if (test_bit(FIMC_IS_SUBDEV_START, &ischain->scc.state)) {
					framemgr = &ischain->scc.vctx->q_dst.framemgr;
					for (j = 0; j < framemgr->frame_cnt; ++j) {
						for (k = 0; k < framemgr->frame[j].planes; k++) {
							pr_err("[SCC:%d] BUF[%d][%d] = %X, 0x%08X, 0x%08X\n",
								i, j, k,
								(u32)framemgr->frame[j].memory,
								framemgr->frame[j].dvaddr_buffer[k],
								framemgr->frame[j].kvaddr_buffer[k]);
						}
					}
				}
				/* VDC */
				if (test_bit(FIMC_IS_SUBDEV_START, &ischain->dis.state)) {
					framemgr = &ischain->dis.vctx->q_dst.framemgr;
					for (j = 0; j < framemgr->frame_cnt; ++j) {
						for (k = 0; k < framemgr->frame[j].planes; k++) {
							pr_err("[VDC:%d] BUF[%d][%d] = %X, 0x%08X, 0x%08X\n",
								i, j, k,
								(u32)framemgr->frame[j].memory,
								framemgr->frame[j].dvaddr_buffer[k],
								framemgr->frame[j].kvaddr_buffer[k]);
						}
					}
				}
				/* VDO */
				if (test_bit(FIMC_IS_SUBDEV_START, &ischain->group_dis.leader.state)) {
					framemgr = &ischain->group_dis.leader.vctx->q_src.framemgr;
					for (j = 0; j < framemgr->frame_cnt; ++j) {
						for (k = 0; k < framemgr->frame[j].planes; k++) {
							pr_err("[VDO:%d] BUF[%d][%d] = %X, 0x%08X, 0x%08X\n",
								i, j, k,
								(u32)framemgr->frame[j].memory,
								framemgr->frame[j].dvaddr_buffer[k],
								framemgr->frame[j].kvaddr_buffer[k]);
						}
					}
				}
				/* SCP */
				if (test_bit(FIMC_IS_SUBDEV_START, &ischain->scp.state)) {
					framemgr = &ischain->scp.vctx->q_dst.framemgr;
					for (j = 0; j < framemgr->frame_cnt; ++j) {
						for (k = 0; k < framemgr->frame[j].planes; k++) {
							pr_err("[SCP:%d] BUF[%d][%d] = %X, 0x%08X, 0x%08X\n",
								i, j, k,
								(u32)framemgr->frame[j].memory,
								framemgr->frame[j].dvaddr_buffer[k],
								framemgr->frame[j].kvaddr_buffer[k]);
						}
					}
				}
			}
		}
	} else {
		pr_err("failed to get core\n");
	}
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0))
#define SECT_ORDER 20
#define LPAGE_ORDER 16
#define SPAGE_ORDER 12

#define lv1ent_page(sent) ((*(sent) & 3) == 1)

#define lv1ent_offset(iova) ((iova) >> SECT_ORDER)
#define lv2ent_offset(iova) (((iova) & 0xFF000) >> SPAGE_ORDER)
#define lv2table_base(sent) (*(sent) & 0xFFFFFC00)

static unsigned long *section_entry(unsigned long *pgtable, unsigned long iova)
{
	return pgtable + lv1ent_offset(iova);
}

static unsigned long *page_entry(unsigned long *sent, unsigned long iova)
{
	return (unsigned long *)__va(lv2table_base(sent)) + lv2ent_offset(iova);
}

static char *sysmmu_fault_name[SYSMMU_FAULTS_NUM] = {
	"PAGE FAULT",
	"AR MULTI-HIT FAULT",
	"AW MULTI-HIT FAULT",
	"BUS ERROR",
	"AR SECURITY PROTECTION FAULT",
	"AR ACCESS PROTECTION FAULT",
	"AW SECURITY PROTECTION FAULT",
	"AW ACCESS PROTECTION FAULT",
	"UNKNOWN FAULT"
};

static int fimc_is_fault_handler(struct device *dev, const char *mmuname,
					enum exynos_sysmmu_inttype itype,
					unsigned long pgtable_base,
					unsigned long fault_addr)
{
	unsigned long *ent;

	if ((itype >= SYSMMU_FAULTS_NUM) || (itype < SYSMMU_PAGEFAULT))
		itype = SYSMMU_FAULT_UNKNOWN;

	pr_err("%s occured at 0x%lx by '%s'(Page table base: 0x%lx)\n",
		sysmmu_fault_name[itype], fault_addr, mmuname, pgtable_base);

	ent = section_entry(__va(pgtable_base), fault_addr);
	pr_err("\tLv1 entry: 0x%lx\n", *ent);

	if (lv1ent_page(ent)) {
		ent = page_entry(ent, fault_addr);
		pr_err("\t Lv2 entry: 0x%lx\n", *ent);
	}

	__fimc_is_fault_handler(dev);

	pr_err("Generating Kernel OOPS... because it is unrecoverable.\n");

	BUG();

	return 0;
}
#else
static int fimc_is_fault_handler(struct iommu_domain *domain,
	struct device *dev,
	unsigned long fault_addr,
	int fault_flag,
	void *token)
{
	pr_err("<FIMC-IS FAULT HANDLER>\n");
	pr_err("Device virtual(0x%X) is invalid access\n", (u32)fault_addr);

	__fimc_is_fault_handler(dev);

	return -EINVAL;
}
#endif
#endif /* USE_OWN_FAULT_HANDLER */

static ssize_t show_clk_gate_mode(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", sysfs_debug.clk_gate_mode);
}

static ssize_t store_clk_gate_mode(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
#ifdef HAS_FW_CLOCK_GATE
	switch (buf[0]) {
	case '0':
		sysfs_debug.clk_gate_mode = CLOCK_GATE_MODE_HOST;
		break;
	case '1':
		sysfs_debug.clk_gate_mode = CLOCK_GATE_MODE_FW;
		break;
	default:
		pr_debug("%s: %c\n", __func__, buf[0]);
		break;
	}
#endif
	return count;
}

static ssize_t show_en_clk_gate(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", sysfs_debug.en_clk_gate);
}

static ssize_t store_en_clk_gate(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
#ifdef ENABLE_CLOCK_GATE
	switch (buf[0]) {
	case '0':
		sysfs_debug.en_clk_gate = false;
		sysfs_debug.clk_gate_mode = CLOCK_GATE_MODE_HOST;
		break;
	case '1':
		sysfs_debug.en_clk_gate = true;
		sysfs_debug.clk_gate_mode = CLOCK_GATE_MODE_HOST;
		break;
	default:
		pr_debug("%s: %c\n", __func__, buf[0]);
		break;
	}
#endif
	return count;
}

static ssize_t show_en_dvfs(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", sysfs_debug.en_dvfs);
}

static ssize_t store_en_dvfs(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
#ifdef ENABLE_DVFS
	struct fimc_is_core *core =
		(struct fimc_is_core *)platform_get_drvdata(to_platform_device(dev));
	struct fimc_is_resourcemgr *resourcemgr;
	int i;

	BUG_ON(!core);

	resourcemgr = &core->resourcemgr;

	switch (buf[0]) {
	case '0':
		sysfs_debug.en_dvfs = false;
		/* update dvfs lever to max */
		mutex_lock(&resourcemgr->dvfs_ctrl.lock);
		for (i = 0; i < FIMC_IS_MAX_NODES; i++) {
			if (test_bit(FIMC_IS_ISCHAIN_OPEN, &((core->ischain[i]).state)))
				fimc_is_set_dvfs(&(core->ischain[i]), FIMC_IS_SN_MAX);
		}
		fimc_is_dvfs_init(resourcemgr);
		resourcemgr->dvfs_ctrl.static_ctrl->cur_scenario_id = FIMC_IS_SN_MAX;
		mutex_unlock(&resourcemgr->dvfs_ctrl.lock);
		break;
	case '1':
		/* It can not re-define static scenario */
		sysfs_debug.en_dvfs = true;
		break;
	default:
		pr_debug("%s: %c\n", __func__, buf[0]);
		break;
	}
#endif
	return count;
}

static DEVICE_ATTR(en_clk_gate, 0644, show_en_clk_gate, store_en_clk_gate);
static DEVICE_ATTR(clk_gate_mode, 0644, show_clk_gate_mode, store_clk_gate_mode);
static DEVICE_ATTR(en_dvfs, 0644, show_en_dvfs, store_en_dvfs);

static struct attribute *fimc_is_debug_entries[] = {
	&dev_attr_en_clk_gate.attr,
	&dev_attr_clk_gate_mode.attr,
	&dev_attr_en_dvfs.attr,
	NULL,
};
static struct attribute_group fimc_is_debug_attr_group = {
	.name	= "debug",
	.attrs	= fimc_is_debug_entries,
};

static int fimc_is_probe(struct platform_device *pdev)
{
	struct exynos_platform_fimc_is *pdata;
	struct resource *mem_res;
	struct resource *regs_res;
	struct fimc_is_core *core;
	int ret = -ENODEV;

	info("%s:start\n", __func__);

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
#ifdef CONFIG_OF
		pdata = fimc_is_parse_dt(&pdev->dev);
		if (IS_ERR(pdata))
#endif
			return PTR_ERR(pdata);
	}

	core = kzalloc(sizeof(struct fimc_is_core), GFP_KERNEL);
	if (!core) {
		err("core is NULL");
		return -ENOMEM;
	}

	fimc_is_dev = &pdev->dev;
	ret = dev_set_drvdata(fimc_is_dev, core);
	if (ret) {
		err("dev_set_drvdata is fail(%d)", ret);
		kfree(core);
		return ret;
	}

	core->pdev = pdev;
	core->pdata = pdata;
	core->id = pdev->id;
	core->debug_cnt = 0;
	device_init_wakeup(&pdev->dev, true);

	/* for mideaserver force down */
	atomic_set(&core->rsccount, 0);
	clear_bit(FIMC_IS_ISCHAIN_POWER_ON, &core->state);

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		dev_err(&pdev->dev, "Failed to get io memory region\n");
		goto p_err1;
	}

	regs_res = request_mem_region(mem_res->start, resource_size(mem_res),
					pdev->name);
	if (!regs_res) {
		dev_err(&pdev->dev, "Failed to request io memory region\n");
		goto p_err1;
	}

	core->regs_res = regs_res;
	core->regs =  ioremap_nocache(mem_res->start, resource_size(mem_res));
	if (!core->regs) {
		dev_err(&pdev->dev, "Failed to remap io region\n");
		goto p_err2;
	}

	core->irq = platform_get_irq(pdev, 0);
	if (core->irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq\n");
		goto p_err3;
	}

	fimc_is_mem_probe(&core->mem,
		core->pdev);

	fimc_is_interface_probe(&core->interface,
		(u32)core->regs,
		core->irq,
		core);

	fimc_is_resource_probe(&core->resourcemgr, core);

	/* group initialization */
	fimc_is_groupmgr_probe(&core->groupmgr);

	ret = sensor_6b2_probe(NULL, NULL);
	if (ret) {
		err("sensor_6b2_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_8b1_probe(NULL, NULL);
	if (ret) {
		err("sensor_8b1_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_6d1_probe(NULL, NULL);
	if (ret) {
		err("sensor_6d1_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_6a3_probe(NULL, NULL);
	if (ret) {
		err("sensor_6a3_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_imx135_probe(NULL, NULL);
	if (ret) {
		err("sensor_imx135_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_3l2_probe(NULL, NULL);
	if (ret) {
		err("sensor_3l2_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_2p2_probe(NULL, NULL);
	if (ret) {
		err("sensor_2p2_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_2p2_12m_probe(NULL, NULL);
	if (ret) {
		err("sensor_2p2_12m_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_3h5_probe(NULL, NULL);
	if (ret) {
		err("sensor_3h5_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_3h7_probe(NULL, NULL);
	if (ret) {
		err("sensor_3h7_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_3h7_sunny_probe(NULL, NULL);
	if (ret) {
		err("sensor_3h7_sunny_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_4e5_probe(NULL, NULL);
	if (ret) {
		err("sensor_4e5_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_imx175_probe(NULL, NULL);
	if (ret) {
		err("sensor_imx175_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_4h5_probe(NULL, NULL);
	if (ret) {
		err("sensor_4h5_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_imx240_probe(NULL, NULL);
	if (ret) {
		err("sensor_imx240_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_imx220_probe(NULL, NULL);
	if (ret) {
		err("sensor_imx220_probe is fail(%d)", ret);
		goto p_err3;
	}

	ret = sensor_ov5693_probe(NULL, NULL);
	if (ret) {
		err("sensor_ov5653_probe is fail(%d)", ret);
		goto p_err3;
	}

	/* device entity - ischain0 */
	fimc_is_ischain_probe(&core->ischain[0],
		&core->interface,
		&core->resourcemgr,
		&core->groupmgr,
		&core->mem,
		core->pdev,
		0,
		(u32)core->regs);

	/* device entity - ischain1 */
	fimc_is_ischain_probe(&core->ischain[1],
		&core->interface,
		&core->resourcemgr,
		&core->groupmgr,
		&core->mem,
		core->pdev,
		1,
		(u32)core->regs);

	/* device entity - ischain2 */
	fimc_is_ischain_probe(&core->ischain[2],
		&core->interface,
		&core->resourcemgr,
		&core->groupmgr,
		&core->mem,
		core->pdev,
		2,
		(u32)core->regs);

	ret = v4l2_device_register(&pdev->dev, &core->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register fimc-is v4l2 device\n");
		goto p_err3;
	}

	/* video entity - 3a0 */
	if (GET_FIMC_IS_NUM_OF_SUBIP(core, 3a0))
		fimc_is_3a0_video_probe(core);

	/* video entity - 3a0 capture */
	if (GET_FIMC_IS_NUM_OF_SUBIP(core, 3a0))
		fimc_is_3a0c_video_probe(core);

	/* video entity - 3a1 */
	if (GET_FIMC_IS_NUM_OF_SUBIP(core, 3a1))
		fimc_is_3a1_video_probe(core);

	/* video entity - 3a1 capture */
	if (GET_FIMC_IS_NUM_OF_SUBIP(core, 3a1))
		fimc_is_3a1c_video_probe(core);

	/* video entity - isp */
	if (GET_FIMC_IS_NUM_OF_SUBIP(core, isp))
		fimc_is_isp_video_probe(core);

	/*front video entity - scalerC */
	if (GET_FIMC_IS_NUM_OF_SUBIP(core, scc))
		fimc_is_scc_video_probe(core);

	/* back video entity - scalerP*/
	if (GET_FIMC_IS_NUM_OF_SUBIP(core, scp))
		fimc_is_scp_video_probe(core);

	if (GET_FIMC_IS_NUM_OF_SUBIP(core, dis)) {
		/* vdis video entity - vdis capture*/
		fimc_is_vdc_video_probe(core);
		/* vdis video entity - vdis output*/
		fimc_is_vdo_video_probe(core);
	}

	platform_set_drvdata(pdev, core);

	fimc_is_ishcain_initmem(core);

#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)
#if defined(CONFIG_VIDEOBUF2_ION)
	if (core->mem.alloc_ctx)
		vb2_ion_attach_iommu(core->mem.alloc_ctx);
#endif
#endif
	EXYNOS_MIF_ADD_NOTIFIER(&exynos_fimc_is_mif_throttling_nb);

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_enable(&pdev->dev);
#endif

#ifdef USE_OWN_FAULT_HANDLER
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0))
	exynos_sysmmu_set_fault_handler(fimc_is_dev, fimc_is_fault_handler);
#else
	iovmm_set_fault_handler(fimc_is_dev, fimc_is_fault_handler, NULL);
#endif
#endif

	dbg("%s : fimc_is_front_%d probe success\n", __func__, pdev->id);

	/* set sysfs for debuging */
	sysfs_debug.en_clk_gate = 0;
	sysfs_debug.en_dvfs = 1;
#ifdef ENABLE_CLOCK_GATE
	sysfs_debug.en_clk_gate = 1;
#ifdef HAS_FW_CLOCK_GATE
	sysfs_debug.clk_gate_mode = CLOCK_GATE_MODE_FW;
#else
	sysfs_debug.clk_gate_mode = CLOCK_GATE_MODE_HOST;
#endif
#endif
	ret = sysfs_create_group(&core->pdev->dev.kobj, &fimc_is_debug_attr_group);

#ifdef ENABLE_DVFS
	{
		struct fimc_is_resourcemgr *resourcemgr;
		resourcemgr = &core->resourcemgr;
		/* dvfs controller init */
		ret = fimc_is_dvfs_init(resourcemgr);
		if (ret)
			err("%s: fimc_is_dvfs_init failed!\n", __func__);
	}
#endif

	info("%s:end\n", __func__);
	return 0;

p_err3:
	iounmap(core->regs);
p_err2:
	release_mem_region(regs_res->start, resource_size(regs_res));
p_err1:
	kfree(core);
	return ret;
}

static int fimc_is_remove(struct platform_device *pdev)
{
	dbg("%s\n", __func__);
	return 0;
}

static const struct dev_pm_ops fimc_is_pm_ops = {
	.suspend		= fimc_is_suspend,
	.resume			= fimc_is_resume,
	.runtime_suspend	= fimc_is_runtime_suspend,
	.runtime_resume		= fimc_is_runtime_resume,
};

#if defined(CONFIG_OF) && defined(CONFIG_CAMERA_EEPROM_SUPPORT)
static int fimc_is_i2c0_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct fimc_is_core *core;
	static bool probe_retried = false;

	if (!fimc_is_dev)
		goto probe_defer;
	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core)
		goto probe_defer;

	core->client0 = client;

	pr_info("%s() \n", __func__);

	return 0;

probe_defer:
	if (probe_retried) {
		err("probe has already been retried!!");
		BUG();
	}

	probe_retried = true;
	err("core device is not yet probed");
	return -EPROBE_DEFER;
}

static int fimc_is_i2c0_remove(struct i2c_client *client)
{
	return 0;
}

static struct of_device_id fimc_is_i2c0_dt_ids[] = {
	{ .compatible = "onsemiconductor,le2464c_liteon",},
	{},
};
MODULE_DEVICE_TABLE(of, fimc_is_i2c0_dt_ids);

static const struct i2c_device_id fimc_is_i2c0_id[] = {
	{"le2464c_liteon", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fimc_is_i2c0_id);

static struct i2c_driver fimc_is_i2c0_driver = {
	.driver = {
		.name = "le2464c_liteon",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = fimc_is_i2c0_dt_ids,
#endif
	},
	.probe = fimc_is_i2c0_probe,
	.remove = fimc_is_i2c0_remove,
	.id_table = fimc_is_i2c0_id,
};
module_i2c_driver(fimc_is_i2c0_driver);

static int fimc_is_i2c1_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct fimc_is_core *core;
	static bool probe_retried = false;

	if (!fimc_is_dev)
		goto probe_defer;
	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core)
		goto probe_defer;

	core->client1 = client;

	pr_info("%s() \n", __func__);

	return 0;

probe_defer:
	if (probe_retried) {
		err("probe has already been retried!!");
		BUG();
	}

	probe_retried = true;
	err("core device is not yet probed");
	return -EPROBE_DEFER;
}

static int fimc_is_i2c1_remove(struct i2c_client *client)
{
	return 0;
}

static struct of_device_id fimc_is_i2c1_dt_ids[] = {
	{ .compatible = "onsemiconductor,le2464c_sharp",},
	{},
};
MODULE_DEVICE_TABLE(of, fimc_is_i2c1_dt_ids);

static const struct i2c_device_id fimc_is_i2c1_id[] = {
	{"le2464c_sharp", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fimc_is_i2c1_id);

static struct i2c_driver fimc_is_i2c1_driver = {
	.driver = {
		.name = "le2464c_sharp",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = fimc_is_i2c1_dt_ids,
#endif
	},
	.probe = fimc_is_i2c1_probe,
	.remove = fimc_is_i2c1_remove,
	.id_table = fimc_is_i2c1_id,
};
module_i2c_driver(fimc_is_i2c1_driver);

#endif


#ifdef CONFIG_OF
static int fimc_is_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct fimc_is_core *core;

	BUG_ON(!fimc_is_dev);

	dev_info(&spi->dev, "%s() +++++++\n", __func__);

	core = (struct fimc_is_core *)dev_get_drvdata(fimc_is_dev);
	if (!core) {
		err("core device is not yet probed");
		return -EPROBE_DEFER;
	}

#ifdef CONFIG_SOC_EXYNOS5422
	if (!strncmp(spi->modalias, "fimc_is_spi0", 12))
		spi->mode = SPI_MODE_1;
	else if (!strncmp(spi->modalias, "fimc_is_spi1", 12))
		spi->mode = SPI_MODE_0;
#endif

	/* spi->bits_per_word = 16; */
	if (spi_setup(spi)) {
		pr_err("failed to setup spi for fimc_is_spi\n");
		ret = -EINVAL;
		goto exit;
	}

	if (!strncmp(spi->modalias, "fimc_is_spi0", 12))
		core->spi0 = spi;

	if (!strncmp(spi->modalias, "fimc_is_spi1", 12)) {
		core->spi1 = spi;
	}

exit:
	return ret;
}

static int fimc_is_spi_remove(struct spi_device *spi)
{
	return 0;
}

static const struct of_device_id exynos_fimc_is_match[] = {
	{
		.compatible = "samsung,exynos5-fimc-is",
	},
	{
		.compatible = "samsung,fimc_is_spi0",
	},
	{
		.compatible = "samsung,fimc_is_spi1",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_fimc_is_match);

static struct spi_driver fimc_is_spi0_driver = {
	.driver = {
		.name = "fimc_is_spi0",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = exynos_fimc_is_match,
	},
	.probe	= fimc_is_spi_probe,
	.remove	= fimc_is_spi_remove,
};

module_spi_driver(fimc_is_spi0_driver);

static struct spi_driver fimc_is_spi1_driver = {
	.driver = {
		.name = "fimc_is_spi1",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = exynos_fimc_is_match,
	},
	.probe	= fimc_is_spi_probe,
	.remove = fimc_is_spi_remove,
};

module_spi_driver(fimc_is_spi1_driver);

static struct platform_driver fimc_is_driver = {
	.probe		= fimc_is_probe,
	.remove		= fimc_is_remove,
	.driver = {
		.name	= FIMC_IS_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &fimc_is_pm_ops,
		.of_match_table = exynos_fimc_is_match,
	}
};

module_platform_driver(fimc_is_driver);
#else
static struct platform_driver fimc_is_driver = {
	.probe		= fimc_is_probe,
	.remove	= __devexit_p(fimc_is_remove),
	.driver = {
		.name	= FIMC_IS_DRV_NAME,
		.owner	= THIS_MODULE,
		.pm	= &fimc_is_pm_ops,
	}
};

static int __init fimc_is_init(void)
{
	int ret = platform_driver_register(&fimc_is_driver);
	if (ret)
		err("platform_driver_register failed: %d\n", ret);
	return ret;
}

static void __exit fimc_is_exit(void)
{
	platform_driver_unregister(&fimc_is_driver);
}
module_init(fimc_is_init);
module_exit(fimc_is_exit);
#endif

MODULE_AUTHOR("Jiyoung Shin<idon.shin@samsung.com>");
MODULE_DESCRIPTION("Exynos FIMC_IS2 driver");
MODULE_LICENSE("GPL");
