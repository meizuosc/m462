/*
 * Copyright (C) 2012 Meizu Technology Co.Ltd, Zhuhai, China
 *		http://www.meizu.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <plat/devs.h>
#include <plat/irqs.h>
#include <linux/mmc/host.h>
#include <plat/sdhci.h>
#include <plat/gpio-cfg.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/pinctrl/consumer.h>

#define CFG_MMC_PIN_DYNAMIC
static int wl_power;
#ifdef CFG_MMC_PIN_DYNAMIC
static struct pinctrl *wl_pinctrl;
#endif
extern struct platform_device *pdev_wl_dwmci;

static struct wake_lock wifi_wake_lock;
static DEFINE_MUTEX(wifi_mutex);

#define PREALLOC_WLAN_SEC_NUM		4
#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*16)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17
 
static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_static_scan_buf0;
void *wlan_static_scan_buf1;

static void *brcm_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;
	if (wlan_mem_array[section].size < size)
		return NULL;
	return wlan_mem_array[section].mem_ptr;
}

static int __init brcm_init_wifi_mem(void)
{
	int i;
	int j;

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf1)
		goto err_mem_alloc;
	printk(KERN_INFO"%s: WIFI MEM Allocated\n", __func__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

static int wlan_power_en(int onoff)
{
	int ret;
	pr_info("## %s(): %s power\n", __func__, onoff?"enable":"disable");
	wake_lock_timeout(&wifi_wake_lock, msecs_to_jiffies(5 * 1000));
	if (onoff) {
		ret = gpio_request_one(wl_power, GPIOF_OUT_INIT_HIGH,
				    "wl_power");
		if (ret) {
			pr_err("%s(), can NOT request gpio\n", __func__);
		}
		gpio_set_value(wl_power, 1);
		gpio_free(wl_power);
		msleep(200);
	} else {
		gpio_set_value(wl_power, 0);
		msleep(10);
	}
	return 0;
}

static int wlan_reset_en(int onoff)
{
	pr_info("### %s %d\n", __func__, onoff);
	gpio_set_value(wl_power, onoff ? 1 : 0);
	if (onoff)
		msleep(200);
	return 0;
}

extern void mmc_force_presence_change(struct platform_device *pdev, int val);

static int wlan_carddetect_en(int onoff)
{
	#ifdef CFG_MMC_PIN_DYNAMIC
	struct pinctrl_state *pins_state;
	char *gpio_wl_sfn = NULL;
	int ret = 0;
	#endif

	pr_info("### %s() %d\n", __func__, onoff);

	if(onoff) {
		#ifdef CFG_MMC_PIN_DYNAMIC
		gpio_wl_sfn = "gpio_wl_sfn_on";
		#else
		pr_info("%s(), not config mmc1 pin dynamically\n", __func__);
		#endif
	} else {
		#ifdef CFG_MMC_PIN_DYNAMIC
		gpio_wl_sfn = "gpio_wl_sfn_off";
		#else
		pr_info("%s(), not config mmc1 pin dynamically\n", __func__);
		#endif
	}

	#ifdef CFG_MMC_PIN_DYNAMIC
	pins_state = pinctrl_lookup_state(wl_pinctrl, gpio_wl_sfn);
	if (!IS_ERR(pins_state)) {
		ret = pinctrl_select_state(wl_pinctrl, pins_state);
		if (ret) {
			pr_err("%s(), could not set %s pins, ret:%d\n",
				__func__, gpio_wl_sfn, ret);
			return ret;
		}
	} else {
		ret = PTR_ERR(pins_state);
		pr_err("%s(), could not get %s pinstate, ret:%d\n",
			__func__, gpio_wl_sfn, ret);
		return ret;
	}
	#endif

	msleep(10);

	mmc_force_presence_change(pdev_wl_dwmci, onoff);
	msleep(400);
	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ        4
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
};

static struct cntry_locales_custom brcm_wlan_translate_custom_table[] = {
	/* Table should be filled out based on custom platform regulatory requirement */
	{"",	 "XY", 0}, /* Universal if Country code is unknown or empty */
	{"IR", "XY", 0}, /* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XY", 0}, /* Universal if Country code is SUDAN */
	{"SY", "XY", 0}, /* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XY", 0}, /* Universal if Country code is GREENLAND */
	{"PS", "XY", 0}, /* Universal if Country code is PALESTINE */
	{"TL", "XY", 0}, /* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XY", 0}, /* Universal if Country code is MARSHALL ISLANDS */
	{"PK", "XY", 0}, /* Universal if Country code is PAKISTAN */
	{"CK", "XY", 0}, /* Universal if Country code is Cook Island (13.4.27)*/
	{"CU", "XY", 0}, /* Universal if Country code is Cuba (13.4.27)*/
	{"FK", "XY", 0}, /* Universal if Country code is Falkland Island (13.4.27)*/
	{"FO", "XY", 0}, /* Universal if Country code is Faroe Island (13.4.27)*/
	{"GI", "XY", 0}, /* Universal if Country code is Gibraltar (13.4.27)*/
	{"IM", "XY", 0}, /* Universal if Country code is Isle of Man (13.4.27)*/
	{"CI", "XY", 0}, /* Universal if Country code is Ivory Coast (13.4.27)*/
	{"JE", "XY", 0}, /* Universal if Country code is Jersey (13.4.27)*/
	{"KP", "XY", 0}, /* Universal if Country code is North Korea (13.4.27)*/
	{"FM", "XY", 0}, /* Universal if Country code is Micronesia (13.4.27)*/
	{"MM", "XY", 0}, /* Universal if Country code is Myanmar (13.4.27)*/
	{"NU", "XY", 0}, /* Universal if Country code is Niue (13.4.27)*/
	{"NF", "XY", 0}, /* Universal if Country code is Norfolk Island (13.4.27)*/
	{"PN", "XY", 0}, /* Universal if Country code is Pitcairn Islands (13.4.27)*/
	{"PM", "XY", 0}, /* Universal if Country code is Saint Pierre and Miquelon (13.4.27)*/
	{"SS", "XY", 0}, /* Universal if Country code is South_Sudan (13.4.27)*/
	{"AL", "AL", 2},
	{"DZ", "DZ", 1},
	{"AS", "AS", 12}, /* changed 2 -> 12*/
	{"AI", "AI", 1},
	{"AG", "AG", 2},
	{"AR", "AR", 21},
	{"AW", "AW", 2},
	{"AU", "AU", 6},
	{"AT", "AT", 4},
	{"AZ", "AZ", 2},
	{"BS", "BS", 2},
	{"BH", "BH", 4},	/* changed 24 -> 4*/
	{"BD", "BD", 2},
	{"BY", "BY", 3},
	{"BE", "BE", 4},
	{"BM", "BM", 12},
	{"BA", "BA", 2},
	{"BR", "BR", 4},
	{"VG", "VG", 2},
	{"BN", "BN", 4},
	{"BG", "BG", 4},
	{"KH", "KH", 2},
	{"CA", "CA", 31},
	{"KY", "KY", 3},
	{"CN", "CN", 38},
	{"CO", "CO", 17},
	{"CR", "CR", 17},
	{"HR", "HR", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ET", "ET", 2},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GF", "GF", 2},
	{"DE", "DE", 7},
	{"GR", "GR", 4},
	{"GD", "GD", 2},
	{"GP", "GP", 2},
	{"GU", "GU", 12},
	{"HK", "HK", 2},
	{"HU", "HU", 4},
	{"IS", "IS", 4},
	{"IN", "IN", 3},
	{"ID", "KR", 25}, /* ID/1 -> KR/24 */
	{"IE", "IE", 5},
	{"IL", "BO", 0},	/* IL/7 -> BO/0 */
	{"IT", "IT", 4},
	{"JP", "JP", 58},
	{"JO", "JO", 3},
	{"KW", "KW", 5},
	{"LA", "LA", 2},
	{"LV", "LV", 4},
	{"LB", "LB", 5},
	{"LS", "LS", 2},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"MO", "MO", 2},
	{"MK", "MK", 2},
	{"MW", "MW", 1},
	{"MY", "MY", 3},
	{"MV", "MV", 3},
	{"MT", "MT", 4},
	{"MQ", "MQ", 2},
	{"MR", "MR", 2},
	{"MU", "MU", 2},
	{"YT", "YT", 2},
	{"MX", "MX", 20},
	{"MD", "MD", 2},
	{"MC", "MC", 1},
	{"ME", "ME", 2},
	{"MA", "MA", 2},
	{"NP", "NP", 3},
	{"NL", "NL", 4},
	{"AN", "AN", 2},
	{"NZ", "NZ", 4},
	{"NO", "NO", 4},
	{"OM", "OM", 4},
	{"PA", "PA", 17},
	{"PG", "PG", 2},
	{"PY", "PY", 2},
	{"PE", "PE", 20},
	{"PH", "PH", 5},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PR", "PR", 20},
	{"RE", "RE", 2},
	{"RO", "RO", 4},
	{"SN", "SN", 2},
	{"RS", "RS", 2},
	{"SG", "SG", 4},
	{"SK", "SK", 4},
	{"SI", "SI", 4},
	{"ES", "ES", 4},
	{"LK", "LK", 1},
	{"SE", "SE", 4},
	{"CH", "CH", 4},
	{"TW", "TW", 1},
	{"TH", "TH", 5},
	{"TT", "TT", 3},
	{"TR", "TR", 7},
	{"AE", "AE", 6},
	{"UG", "UG", 2},
	{"GB", "GB", 6},
	{"UY", "UY", 1},
	{"VI", "VI", 13},
	{"VA", "VA", 12}, /* changed 2 -> 12 */
	{"VE", "VE", 3},
	{"VN", "VN", 4},
	{"MA", "MA", 1},
	{"ZM", "ZM", 2},
	{"EC", "EC", 21},
	{"SV", "SV", 19},
	{"KR", "KR", 57},
	{"RU", "RU", 13},
	{"UA", "UA", 8},
	{"GT", "GT", 1},
	{"MN", "MN", 1},
	{"NI", "NI", 2},
	{"US", "Q2", 57},
};

static void __maybe_unused *brcm_wlan_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(brcm_wlan_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;

	for (i = 0; i < size; i++)
		if (strcmp(ccode,
		brcm_wlan_translate_custom_table[i].iso_abbrev) == 0)
			return &brcm_wlan_translate_custom_table[i];
	return &brcm_wlan_translate_custom_table[0];
}

extern int get_mac_from_device(unsigned char *buf);

static int brcm_wlan_get_mac_addr(unsigned char *buf)
{
	struct file *fp      = NULL;
	char macbuffer[18]   = {0};
	unsigned int fs_buffer[6]  = {0};
	int i = 0;
	mm_segment_t oldfs    = {0};
	char *mac_file       = "/data/calibration/mac_addr";
	int ret = 0;
	int no_mac = 0;

	pr_info("%s(), no_mac is %d\n", __func__, no_mac);
	fp = filp_open(mac_file, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_info("%s(), open %s err: %ld\n", __func__, mac_file, PTR_ERR(fp));
		if(no_mac) {
			get_random_bytes(buf, 6);
			buf[0] = 0x38;
			buf[1] = 0xBC;
			buf[2] = 0x1A;
		}

		pr_info("%s: write file %s\n", __func__, mac_file);

		snprintf(macbuffer, sizeof(macbuffer),"%02X:%02X:%02X:%02X:%02X:%02X\n",
				buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

		fp = filp_open(mac_file, O_RDWR | O_CREAT, 0644);
		if (IS_ERR(fp)) {
			pr_err("%s:create file %s error(%ld)\n", __func__, mac_file, PTR_ERR(fp));
		} else {
			oldfs = get_fs();
			set_fs(get_ds());

			if (fp->f_mode & FMODE_WRITE) {
				ret = fp->f_op->write(fp, (const char *)macbuffer,
						sizeof(macbuffer), &fp->f_pos);
				if (ret < 0)
					pr_err("%s:write file %s error(%d)\n", __func__, mac_file, ret);
			}
			set_fs(oldfs);
			filp_close(fp, NULL);
		}

	} else {
		pr_debug("%s(), open %s success\n",
			__func__, mac_file);
		if(no_mac) {
			ret = kernel_read(fp, 0, macbuffer, 18);
			if(ret <= 17) {
				pr_info("%s: read mac_info error, get random mac address\n", __func__);
				get_random_bytes(buf, 6);
			} else {
				macbuffer[17] = '\0';
				pr_debug("%s: read mac_info from file ok\n", __func__);
				sscanf(macbuffer, "%02X:%02X:%02X:%02X:%02X:%02X",
						(unsigned int *)&(fs_buffer[0]), (unsigned int *)&(fs_buffer[1]),
						(unsigned int *)&(fs_buffer[2]), (unsigned int *)&(fs_buffer[3]),
						(unsigned int *)&(fs_buffer[4]), (unsigned int *)&(fs_buffer[5]));
				for (i = 3; i < 6; i ++)
					buf[i] = (unsigned char)fs_buffer[i];
			}
			if (fp)
				filp_close(fp, NULL);

			buf[0] = 0x38;
			buf[1] = 0xBC;
			buf[2] = 0x1A;
		}
	}

	pr_info("mac address mac=%.2x:%.2x:%.2x:%.2x:%.2x:%.2x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	return 0;
}

#ifdef CONFIG_OF
struct wifi_platform_data *wifi_parse_dt(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct wifi_platform_data *pdata;

	pr_info("%s() ++++++\n", __func__);
	
	if (!np)
		return NULL;

	/* 0 is for irq */
	wl_power = of_get_gpio(np, 1);
	#ifdef CFG_MMC_PIN_DYNAMIC
	wl_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(wl_pinctrl)) {
		dev_err(&pdev->dev, "%s(), could not get pinctrl, ret:%ld\n",
			__func__, PTR_ERR(wl_pinctrl));
		return NULL;
	}
	#else
	pr_info("gpio wl_power:%d\n", wl_power);
	#endif

	pdata = kzalloc(sizeof(struct wifi_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "failed to allocate platform data\n");
		return NULL;
	}

	dev->platform_data = pdata;

	pdata->set_power = wlan_power_en;
	pdata->set_reset = wlan_reset_en;
	pdata->set_carddetect = wlan_carddetect_en;
	pdata->mem_prealloc = brcm_wlan_mem_prealloc;
	pdata->get_country_code = brcm_wlan_get_country_code;
	pdata->get_mac_addr = brcm_wlan_get_mac_addr;

	return pdata;
}
#else
struct wifi_platform_data *wifi_parse_dt(struct platform_device *pdev)
{
	return NULL;
}
#endif


static int __init m76_wifi_init(void)
{
	int ret;

	wake_lock_init(&wifi_wake_lock, WAKE_LOCK_SUSPEND, "wifi_ctrl_wake_lock");

	ret = brcm_init_wifi_mem();
	if (ret) {
		pr_err("%s(), init wifi mem err! ret:%d\n", __func__, ret);
	}

	return 0;
}

device_initcall(m76_wifi_init);
