/* arch/arm/mach-exynos/hardware.c
 *
 * Meizu Mobilephone Hardware information support
 *
 * Copyright (c) 2014 MEIZU Technology Co., Ltd.
 * Author: xuhanlong <xuhanlong@meizu.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>

#include <mach/hardware.h>

#define PRIVATE_SLOT_SIZE			(1024)
#define PRIVATE_SLOT_IDX_PSN		(65)
#define PSN_MAGIC	(0x50534E00)	//"PSN"

struct psn_info {
	int magic;
	char psn_code[32];
};

/*
 * Define:
 *        (0 << 16) :  JDI
 *        (1 << 16) :  SHARP
 */
extern int meizu_lcd_type;

static char machine_type[64] = "Unknown";
static u32 board_version = 0;
static struct psn_info psn;
int g_fp = 0;

extern int emmc_partition_rw(const char *part_name, int write, loff_t offset,
						void *buffer, size_t len);

void fp_get(int fp)
{
	g_fp = fp;
}

static char* find_fp(void)
{
	if(g_fp == FP_IDEX)
		return "idex";
	else if(g_fp == FP_GOODIX)
		return "goodix";
	else
		return "null";
}

u32 meizu_board_version(void)
{
	return board_version;
}

static u64 meizu_hardware_version(void)
{
	u64 hardware_version = 0;

	/* bit0: board version 1 */
	if (board_version == 1) {
		hardware_version |= 1ULL << 0;
	}
	/* bit1: board version 2/3 */
	if (board_version == 2 || board_version == 3) {
		hardware_version |= 1ULL << 1;
	}
	/* bit2: Sharp LCD module */
	if (meizu_lcd_type & (0x1 << 16)) {
		hardware_version |= 1ULL << 2;
	}

	return hardware_version;
}

static const char* meizu_get_psn(void)
{
	int ret;
	int offset = PRIVATE_SLOT_IDX_PSN * PRIVATE_SLOT_SIZE;

	if (psn.magic == PSN_MAGIC)
		return psn.psn_code;

	ret = emmc_partition_rw("private", 0, offset, &psn, sizeof(psn));
	if (ret < 0) {
		pr_err("%s: read PSN failed!(%d)\n", __func__, ret);
	}

	if (psn.magic == PSN_MAGIC) {
		return psn.psn_code;
	}

	pr_warn("%s: PSN not found, use default.\n", __func__);
	psn.magic = PSN_MAGIC;
	snprintf(psn.psn_code, sizeof(psn.psn_code), "123456789012");

	return psn.psn_code;
}

static int show_hwinfo(struct seq_file *seq, void *v)
{
	seq_printf(seq, "SOC             : %s\n", "EXYNOS5430");
	seq_printf(seq, "SOC Vendor      : %s\n", "SAMSUNG");
	seq_printf(seq, "Machine Type    : %s\n", machine_type);
	seq_printf(seq, "Board Version   : %d\n", board_version);
	seq_printf(seq, "Hardware Version: %#llx\n", meizu_hardware_version());
	seq_printf(seq, "FpVendor        : %s\n", find_fp());
	seq_printf(seq, "PSN             : %s\n", meizu_get_psn());
	return 0;
}

static int hwinfo_open(struct inode *inode, struct file *fp)
{
	return single_open(fp, show_hwinfo, NULL);
}

static const struct file_operations proc_hwinfo_fops = {
	.open = hwinfo_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init proc_hwinfo_init(void)
{
	struct device_node *np;
	const char *type;

	proc_create("hwinfo", 0, NULL, &proc_hwinfo_fops);

	np = of_find_node_by_path("/bootinfo");
	if (np == NULL) {
		pr_err("%s: Can't find /bootinfo node from FDT.\n", __func__);
		return -1;
	}

	if (of_property_read_u32(np, "board_version", &board_version) < 0) {
		pr_err("%s: read property board_version error.\n", __func__);
		return -1;
	}

	if (of_property_read_string(np, "machine_type", &type) < 0) {
		pr_err("%s: read property machine_type error.\n", __func__);
		return -1;
	}
	snprintf(machine_type, sizeof(machine_type), "%s", type);

	return 0;
}

late_initcall(proc_hwinfo_init);

MODULE_DESCRIPTION("Meizu Mobilephone Hardware Information");
MODULE_AUTHOR("xuhanlong <xuhanlong@meizu.com>");
MODULE_LICENSE("GPL");
