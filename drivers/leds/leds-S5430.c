/*
 * ledtrig-S5430.c - LED Trigger Based on GPIO events
 *
 * Copyright 2009 Felipe Balbi <me@felipebalbi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#endif
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <plat/gpio-core.h>
#include <plat/pm.h>
#include <plat/gpio-cfg.h>

#include "leds.h"

#undef SUPPORT_CONTROL_LED_IN_SLEEP_MODE

enum {
	LED_BLUE = 0,
	LED_RED,
	LED_GREEN,
	LED_MAX,
};

#define LED_NUMBER LED_MAX

struct led_desc {
	const char name[10];
	int gpio;
};

struct leds_S5430_platform_data{
	int blue;
	int red;
	int green;
};

struct led_desc led_info[LED_NUMBER] =
{
	{"BLUE", 0},
	{"RED", 0},
	{"GREEN", 0},
};

static ssize_t data_read_normalmode(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
	ssize_t size = 0;
	int value = 0;
	int* gpio = file->private_data;

	value = __gpio_get_value(*gpio);
	if(value == 0)
		size = simple_read_from_buffer(data, count, ppos, "OFF\n", sizeof("OFF\n"));
	else if(value == 1)
		size = simple_read_from_buffer(data, count, ppos, "ON\n", sizeof("ON\n"));
	else
		pr_err("%s : GPIO_LED wrong data\n", __func__);

	return size;
}

static ssize_t data_write_normalmode(struct file *file, const char __user *data, size_t count, loff_t *ppos)
{
	ssize_t size = 0;
	char buffer[8];
	int* gpio = file->private_data;
	size = simple_write_to_buffer(buffer, ARRAY_SIZE(buffer), ppos, data, count);

	if(size > 0) {
		if(strncasecmp("ON",buffer,sizeof("ON")-1) == 0) {
			if (gpio_request(*gpio, "LED_CONTROL")) {
				pr_err("%s : GPIO_LED request port error\n", __func__);
			} else {
				gpio_direction_output(*gpio, 1);
				gpio_free(*gpio);
			}
		} else if (strncasecmp("OFF",buffer,sizeof("OFF")-1) == 0) {
			if (gpio_request(*gpio, "LED_CONTROL")) {
				pr_err("%s : GPIO_LED request port error\n", __func__);
			} else {
				gpio_direction_output(*gpio, 0);
				gpio_free(*gpio);
			}
		}
	}

	return size;
}

#ifdef SUPPORT_CONTROL_LED_IN_SLEEP_MODE
static ssize_t data_read_sleepmode(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
	ssize_t size = 0;
	int value = 0;
	int* gpio = file->private_data;
//	value = s5p_gpio_get_pd_cfg(*gpio);
	printk("[LED] gpio=%d\n",*gpio);

	if(value == 0)
		size = simple_read_from_buffer(data, count, ppos, "OFF\n", sizeof("OFF\n"));
	else if(value == 1)
		size = simple_read_from_buffer(data, count, ppos, "ON\n", sizeof("ON\n"));
	else
		pr_err("%s : GPIO_LED wrong data\n", __func__);

	return size;
}

static ssize_t data_write_sleepmode(struct file *file, const char __user *data, size_t count, loff_t *ppos)
{
	ssize_t size = 0;
	char buffer[8];
	int* gpio = file->private_data;
	size = simple_write_to_buffer(buffer, ARRAY_SIZE(buffer), ppos, data, count);

	printk("Led : because of sleep current, S5430 turn off LED\n");
	if(size > 0) {
		if(strncasecmp("ON",buffer,sizeof("ON")-1) == 0) {
			if (gpio_request(*gpio, "LED_CONTROL")) {
				pr_err("%s : GPIO_LED request port error\n", __func__);
			} else {
				gpio_free(*gpio);
			}
		} else if (strncasecmp("OFF",buffer,sizeof("OFF")-1) == 0) {
			if (gpio_request(*gpio, "LED_CONTROL")) {
				pr_err("%s : GPIO_LED request port error\n", __func__);
			} else {
				gpio_free(*gpio);
			}
		}
	}

	return size;
}
#endif

static const struct file_operations data_normal_ops = {
	.open       = simple_open,
	.read       = data_read_normalmode,
	.write      = data_write_normalmode,
	.llseek     = no_llseek,
};

#ifdef SUPPORT_CONTROL_LED_IN_SLEEP_MODE
static const struct file_operations data_sleep_ops = {
	.open       = simple_open,
	.read       = data_read_sleepmode,
	.write      = data_write_sleepmode,
	.llseek     = no_llseek,
};
#endif

static void initialize_debug_fs(void)
{
	struct dentry *root, *reg_dir;
	int i;

	pr_debug("%s is called.\n", __func__);
	root = debugfs_create_dir("led_control", NULL);
	if (IS_ERR(root)) {
		pr_err("Debug fs for led driver failed. (%ld)\n", PTR_ERR(root));
	}

	for (i = 0; i < LED_NUMBER; i++) {
		reg_dir = debugfs_create_dir(led_info[i].name, root);
		debugfs_create_file("normal", S_IRUGO|S_IWUSR, reg_dir, &led_info[i].gpio, &data_normal_ops);
#ifdef SUPPORT_CONTROL_LED_IN_SLEEP_MODE
		debugfs_create_file("sleep", S_IRUGO|S_IWUSR, reg_dir, &led_info[i].gpio, &data_sleep_ops);
#endif
	}
};

#ifdef CONFIG_OF
static struct leds_S5430_platform_data *leds_S5430_parse_dt(struct platform_device *pdev)
{
	struct leds_S5430_platform_data *pdata;
	struct device_node *np = pdev->dev.of_node;
	int gpio;

	if (!np)
		return ERR_PTR(-ENOENT);

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "failed to allocate platform data\n");
		return ERR_PTR(-ENOMEM);
	}

	pdev->dev.platform_data = pdata;

	//BLUE
	gpio = of_get_gpio(np, 0);
	if (!gpio_is_valid(gpio)) {
		return ERR_PTR(-EINVAL);
	}
	led_info[LED_BLUE].gpio = gpio;

	//RED
	gpio = of_get_gpio(np, 1);
	if (!gpio_is_valid(gpio)) {
		return ERR_PTR(-EINVAL);
	}
	led_info[LED_RED].gpio = gpio;

	//GREEN
	gpio = of_get_gpio(np, 2);
	if (!gpio_is_valid(gpio)) {
		return ERR_PTR(-EINVAL);
	}
	led_info[LED_GREEN].gpio = gpio;

	return pdata;
}
#endif

static int simple_led_driver_probe(struct platform_device *pdev)
{
	int ret = 0;
	leds_S5430_parse_dt(pdev);
	initialize_debug_fs();

	return ret;
}

static int simple_led_driver_remove(struct platform_device *pdev)
{

	return 0;
}

static int simple_led_driver_suspend(struct device *dev)
{
	return 0;
}

static int simple_led_driver_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops simple_led_driver_pm = {
	.suspend	= simple_led_driver_suspend,
	.resume		= simple_led_driver_resume,
};

#ifdef CONFIG_OF
static struct of_device_id s5430_led_dt_ids[] = {
	{ .compatible = "s5430,led" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5430_led_dt_ids);
#endif

static struct platform_driver simple_led_driver_driver = {
	.probe	= simple_led_driver_probe,
	.remove	= simple_led_driver_remove,
	.driver	= {
		.name	= "simple_led_control",
		.owner	= THIS_MODULE,
		.pm	= &simple_led_driver_pm,
#ifdef CONFIG_OF
		   .of_match_table = s5430_led_dt_ids,
#endif
	},
};

module_platform_driver(simple_led_driver_driver);


static int __init simple_led_driver_init(void)
{
	return 0;
}
late_initcall(simple_led_driver_init);

static void __exit simple_led_driver_exit(void)
{
}
module_exit(simple_led_driver_exit);

MODULE_AUTHOR("SOC CSE S/W Team");
MODULE_DESCRIPTION("GPIO LED Driver");
MODULE_LICENSE("GPL");
