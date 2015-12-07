/*
 * arch/arm/mach-exynos/reset-reason-exynos5430.c
 *
 * Copyright (C) 2012 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/kmsg_dump.h>
#include <linux/of.h>
#include <linux/bootmem.h>
#include <linux/memblock.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <asm/page.h>

#include <plat/pm.h>

#include <mach/regs-pmu.h>
#include <mach/resetreason.h>


#define EXYNOS5430_RST_STAT_SWRESET		(1<<29)
#define EXYNOS5430_RST_STAT_WRSET		(1<<28)
#define EXYNOS5430_RST_STAT_A5IS_WDTRESET	(1<<27)
#define EXYNOS5430_RST_STAT_SSS_WDTRESET	(1<<26)
#define EXYNOS5430_RST_STAT_KFC_WDTRESET	(1<<24)
#define EXYNOS5430_RST_STAT_EAGLE_WDTRESET	(1<<23)
#define EXYNOS5430_RST_STAT_PIN_RESET		(1<<16)
#define EXYNOS5430_RST_STAT_EAGLE_WRESET3	(1<<3)
#define EXYNOS5430_RST_STAT_EAGLE_WRESET2	(1<<2)
#define EXYNOS5430_RST_STAT_EAGLE_WRESET1	(1<<1)
#define EXYNOS5430_RST_STAT_EAGLE_WRESET0	(1<<0)
#define BOOT_INFO_LABEL "Boot info: "
#define BOOT_FROM_LABEL "Last boot from: "
#define BOOT_CMD_LABEL  "Restart cmd : "
#define BOOT_STAT_LABEL "Reset stats:\n"

#define RESET_REASON_MEM_MAGIC (0x53535655)

static char resetreason[1024];
static unsigned long g_reset_reason_phy_addr;
static unsigned long g_reset_reason_mem_size;

enum reset_reason {
	SW_RESTART,		/* Software reboot */
	OOPS_RESTART,
	PANIC_RESTART,
	HALT_RESTART,
	POWEROFF_RESTART,
	EMERG_RESTART,
	CRIT_TEMP_RESTART,
	WARM_RESET,		/* hardware reset */
	A5IS_WDTRESET,
	SSS_WDTRESET,
	KFC_WDTRESET,
	EAGLE_WDTRESET,
	PIN_RESTART,
	EAGLE_WRESET,
	UNKNOWN_REASON,
	END_REASONS,		/* For end */
};

static char reset_reason_str[END_REASONS][20] = {
	"software reboot",
	"oops reboot",
	"panic reboot",
	"halt reboot",
	"poweroff reboot",
	"emerg reboot",
	"crit_temp reboot",
	"system wreset",
	"a5is_wdt reset",
	"sss_wdt reset",
	"kfc_wdt reset",
	"eagle_wdt reset",
	"hw_pin reset",
	"eagle wreset",
	"unknow reason",
};

static struct {
	int flag;
	const char *str;
	u32 mask;
} hw_resetreason_flags[] = {
	{ SW_RESTART, 		"software", 	EXYNOS5430_RST_STAT_SWRESET },
	{ WARM_RESET, 		"warm", 	EXYNOS5430_RST_STAT_WRSET },
	{ A5IS_WDTRESET, 	"isp_wdt", 	EXYNOS5430_RST_STAT_A5IS_WDTRESET },
	{ KFC_WDTRESET, 	"kfc_wdt", 	EXYNOS5430_RST_STAT_KFC_WDTRESET },
	{ EAGLE_WDTRESET, 	"eagle_wdt", 	EXYNOS5430_RST_STAT_EAGLE_WDTRESET },
	{ PIN_RESTART, 		"pin", 		EXYNOS5430_RST_STAT_PIN_RESET },
	{ EAGLE_WRESET, 	"eagle_warm", 	EXYNOS5430_RST_STAT_EAGLE_WRESET3 |
						EXYNOS5430_RST_STAT_EAGLE_WRESET2 |
						EXYNOS5430_RST_STAT_EAGLE_WRESET1 |
						EXYNOS5430_RST_STAT_EAGLE_WRESET0 },
};

static struct {
	int flag;
	int mask;
} sw_resetreason_flags[] = {
	{ OOPS_RESTART,		KMSG_DUMP_OOPS },
	{ PANIC_RESTART,	KMSG_DUMP_PANIC },
	{ SW_RESTART,		KMSG_DUMP_RESTART },
	{ HALT_RESTART,		KMSG_DUMP_HALT },
	{ POWEROFF_RESTART,	KMSG_DUMP_POWEROFF },
	{ EMERG_RESTART,	KMSG_DUMP_EMERG },
	{ CRIT_TEMP_RESTART,	KMSG_DUMP_CRIT_TEMP },
};

struct normal_reboot {
	int flag;
	char cmd[50];
};

/*The struct for record reset reason */
struct reset_info {
	uint32_t magic;
	int resetreason;	/*Record the last reset reason*/
	unsigned long long count[END_REASONS];
	struct normal_reboot normal_reboot_info;
};

struct reset_info *resetinfoPtr;

static void reset_normal_reboot_reason(void)
{
	memset(&resetinfoPtr->normal_reboot_info, 0, sizeof(struct normal_reboot));
}

void record_normal_reboot_reason(const char *cmd)
{
	if (!resetinfoPtr) {
		pr_info("%s: Get the cma ram failed \n", __func__);
		return;
	}

	if (cmd) {
		resetinfoPtr->normal_reboot_info.flag = 1;
		strcpy(resetinfoPtr->normal_reboot_info.cmd, cmd);
	} else {
		reset_normal_reboot_reason();
	}
}

static void inc_reset_reason(enum kmsg_dump_reason reason)
{
	if (!resetinfoPtr) {
		pr_info("%s: Get the cam ram failed \n", __func__);
		return;
	}

	resetinfoPtr->resetreason = reason;
	resetinfoPtr->count[resetinfoPtr->resetreason]++;
	pr_info("%s resetreason = %s, count = %llu \n", __func__,
		reset_reason_str[resetinfoPtr->resetreason],
		resetinfoPtr->count[resetinfoPtr->resetreason]);
}

static DEFINE_SPINLOCK(recorded_spin);
void record_reset_reason(enum kmsg_dump_reason reason)
{
	int i;
	static atomic_t recorded;

	spin_lock(&recorded_spin);
	if (atomic_read(&recorded)) {
		pr_debug("Already recorded the reset reason\n");
		spin_unlock(&recorded_spin);
		return;
	}

	/* Record once: record only the first reason */
	atomic_set(&recorded, 1);
	spin_unlock(&recorded_spin);
	for (i = 0; i < ARRAY_SIZE(sw_resetreason_flags); i++) {
		if (reason == sw_resetreason_flags[i].mask)
			inc_reset_reason(sw_resetreason_flags[i].flag);
	}
}

#define get_reset_reason_str(r)	(reset_reason_str[r])
#define get_reset_count(r) (resetinfoPtr->count[r])

/* /proc/resetreason */
static int resetreason_show(struct seq_file *p, void *v)
{
	int i;

	pr_debug("%s the resetreason is %s\n", __func__, resetreason);
	if (!resetinfoPtr) {
		pr_info("%s Get ram failed ....\n ", __func__);
		seq_printf(p, "%s", resetreason);
		return -EFAULT;
	}
	seq_printf(p, BOOT_FROM_LABEL "%02d, %s, %llu\n",
		resetinfoPtr->resetreason,
		reset_reason_str[resetinfoPtr->resetreason],
		resetinfoPtr->count[resetinfoPtr->resetreason]);
	if (resetinfoPtr->normal_reboot_info.flag == 1) {
		seq_printf(p, BOOT_CMD_LABEL "reboot %s\n",
			resetinfoPtr->normal_reboot_info.cmd);
	}
	seq_printf(p, BOOT_STAT_LABEL);

	for (i = SW_RESTART; i < END_REASONS; i++) {
		seq_printf(p, "[%02d, %s]\t (%llu)\n", i,
			get_reset_reason_str(i), get_reset_count(i));
	}

	return 0;
}

static void *resetreason_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *resetreason_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void resetreason_stop(struct seq_file *m, void *v)
{
	/* Nothing to do */
}

const struct seq_operations resetreason_op = {
	.start = resetreason_start,
	.next = resetreason_next,
	.stop = resetreason_stop,
	.show = resetreason_show
};

static int proc_resetreason_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &resetreason_op);
}

static const struct file_operations proc_resetreason_operations = {
	.open = proc_resetreason_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static int __init reset_reason_setup(char *str)
{
	unsigned long base = 0, size =0;
	struct device_node *np = NULL;

	np = of_find_node_by_name(NULL, "reset_reason");
	if (!np) {
		pr_err("Can't find reset_reason node!\n");
	}

	of_property_read_u32(np, "reset_reason,mem_address", (int*)&base);
	of_property_read_u32(np, "reset_reason,mem_size", (int*)&size);

	if (!base || !size)
		goto out;

	if (!(reserve_bootmem(base, size, BOOTMEM_EXCLUSIVE))) {
		g_reset_reason_phy_addr = base;
		g_reset_reason_mem_size = size;
		pr_info("reset_reason: memory reserved - base: 0x%08X,"
			"size: 0x%08X\n", (unsigned int)base, (unsigned int)size);
		return 0;
	}
out:
	pr_err("reset_reason: buffer reserved failed base:%08X, size:%08X\n",
			(unsigned int)base, (unsigned int)size);
	return -EINVAL;
}
__setup("reset_reason_setup", reset_reason_setup);

static void *reset_reason_ram_vmap(phys_addr_t start, size_t size)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	prot = pgprot_noncached(PAGE_KERNEL);

	pages = kmalloc(sizeof(struct page *) * page_count, GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n", __func__,
			page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);

	return vaddr;
}

static int reset_reason_mem_probe(struct platform_device *pdev)
{
	int i;
	u32 reasons;
	char buf[128];
	int ret = -EINVAL;
	
	pr_debug("Entering :%s\n", __func__);

	/*check reset reason*/
	reasons = __raw_readl(EXYNOS_RST_STAT);
	strlcpy(resetreason, "Last reset was: ", sizeof(resetreason));

	for (i = 0; i < ARRAY_SIZE(hw_resetreason_flags); i++) {
		if (reasons & hw_resetreason_flags[i].mask)
			strlcat(resetreason, hw_resetreason_flags[i].str,
				sizeof(resetreason));
	}

	snprintf(buf, sizeof(buf), " reset (RST_STAT=0x%x)\n", reasons);
	strlcat(resetreason, buf, sizeof(resetreason));
	pr_info("%s", resetreason);

	/*Register record mem driver*/
	if (!g_reset_reason_phy_addr || !g_reset_reason_mem_size) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "The memory address and size must be non-zero\n");
		goto err_out;
	}

	resetinfoPtr = reset_reason_ram_vmap(g_reset_reason_phy_addr,
						g_reset_reason_mem_size);
	if (!resetinfoPtr) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "Failed to map memory\n");
		goto err_out;
	}

	if (resetinfoPtr->magic != RESET_REASON_MEM_MAGIC) {
		pr_info("%s The ram which record reasons was damaged\n", __func__);
		memset(resetinfoPtr, 0, sizeof(struct reset_info));
		resetinfoPtr->magic = RESET_REASON_MEM_MAGIC;
	}

	/*inc reset reason, ignore software reset*/
	for (i = 1; i < ARRAY_SIZE(hw_resetreason_flags); i++) {
		if (reasons & hw_resetreason_flags[i].mask)
			inc_reset_reason(hw_resetreason_flags[i].flag);
	}

	proc_create("reset_reason", S_IFREG | S_IRUGO, NULL,
					&proc_resetreason_operations);

	return 0;

err_out:
	return ret;
}

static const struct of_device_id reset_reason_match[] = {
	{ .compatible = "samsung,exynos5430_reset_reason", },
	{},
};
MODULE_DEVICE_TABLE(of, reset_reason_match);

static struct platform_driver reset_reason_mem_driver = {
	.probe = reset_reason_mem_probe,
	.driver = {
		.name = "reset_reason",
		.owner = THIS_MODULE,
		.of_match_table = reset_reason_match,
	},
};

static int __init reset_reason_init(void)
{
	return platform_driver_register(&reset_reason_mem_driver);;
}
arch_initcall(reset_reason_init);

static void __exit reset_reason_exit(void)
{
	platform_driver_unregister(&reset_reason_mem_driver);
}
module_exit(reset_reason_exit);
