/*
 * Led driver for meizu m6x
 *
 * Copyright (C) 2012 -2013 Meizu Technology Co.Ltd, Zhuhai, China
 * Author:		
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
	  
#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
//#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/mfd/mx-hub.h>

#define	MODE_CURRENT			0x01
#define	MODE_SLOPE				0x02
#define	MODE_TBLINK				0x04
#define MODE_TEST				0x08

#define	GET_MODE(x)	((x>>8)&0x0F)

 
 /*led private data*/
struct mx_hub_led {
	 struct CWMCU_data *data;
	 struct led_classdev led_cdev;
	 int	 brightness;
	 struct mutex mutex;
	 int id;
	 struct delayed_work pwm_dwork;
	 int value_bak;
	 int value_pwm;
#ifdef CONFIG_HAS_EARLYSUSPEND
	 struct early_suspend early_suspend;
#endif
};
 void mx_hub_led_brightness_set(struct led_classdev *led_cdev,
		  enum led_brightness value);

static int mx_hub_set_led_current(struct led_classdev *led_cdev, 
			 enum led_brightness cur)
{
	struct mx_hub_led *led =container_of(led_cdev, struct mx_hub_led, led_cdev);
	struct CWMCU_data *hub = led->data;

	int ret = 0;

	ret = mx_hub_write_addr(hub, MX_HUB_REG_LED_BRN, 1, &cur);
	if(ret < 0)
		dev_err(led_cdev->dev, "brightness set failed ret = %d \n",ret);

	return ret;

}
 
static int mx_hub_led_blink_set(struct led_classdev *led_cdev,
		int delay_off)
{
	struct mx_hub_led *led =container_of(led_cdev, struct mx_hub_led, led_cdev);
	struct CWMCU_data *hub = led->data;
	int ret = 0;

	printk("%s : delay off time is %d \n", __func__, delay_off); 
	if (delay_off == 0)
		return -1;

	ret = mx_hub_write_addr(hub, MX_HUB_REG_LED_SLP, 2, (u8 *)&delay_off);
	if(ret < 0)
		dev_err(led_cdev->dev, "blink set failed ret = %d \n",ret);

	return ret;
}

static int mx_hub_two_blink_set(struct led_classdev *led_cdev,
		int delay_off)
{
	struct mx_hub_led *led =container_of(led_cdev, struct mx_hub_led, led_cdev);
	struct CWMCU_data *hub = led->data;
	int ret = 0;

	printk("%s : delay off time is %d \n", __func__, delay_off); 
	if (delay_off == 0)
		return -1;

	ret = mx_hub_write_addr(hub, MX_HUB_REG_LED_FADE, 2, (u8 *)&delay_off);
	if(ret < 0)
		dev_err(led_cdev->dev, "blink set failed ret = %d \n",ret);

	return ret;
}

static int mx_hub_set_led_test(struct led_classdev *led_cdev, int flag)
{
	struct mx_hub_led *led =container_of(led_cdev, struct mx_hub_led, led_cdev);
	struct CWMCU_data *hub = led->data;
	int ret = 0;

	printk("%s : flag is %d \n", __func__, flag); 
	
	ret = mx_hub_write_addr(hub, MX_HUB_REG_LED_TEST, 2, (u8 *)&flag);
	if(ret < 0)
		dev_err(led_cdev->dev, "blink set failed ret = %d \n",ret);

	return ret;
}
void mx_hub_led_brightness_set(struct led_classdev *led_cdev,
		 enum led_brightness value)
{
	
	int mode;
	struct mx_hub_led *led =container_of(led_cdev, struct mx_hub_led, led_cdev);
	int ret = 0;
	int data = (value & 0x0FF);
	if (0 == data) {
		mode = MODE_CURRENT;
	}else {
		mode = GET_MODE(value);
	}
	printk("%s :value = 0x%.4X, mode = %d, data = %d \n",__func__, value,mode, data);
	
	switch( mode )
	{
	case MODE_CURRENT:
		if (data >= 250) data =249;
		else if (data == 1) data +=1;
		led->brightness = data;
		ret = mx_hub_set_led_current(led_cdev,data);
		break;

	case MODE_TEST:
		ret = mx_hub_set_led_test(led_cdev,data);
		break;

	case MODE_SLOPE:
		ret = mx_hub_led_blink_set(led_cdev,data);
		break;
	case MODE_TBLINK:
		ret = mx_hub_two_blink_set(led_cdev,data);
		break;
		
	default:		
		dev_err(led_cdev->dev, "mode  %d is valite \n",mode);
		ret = -EINVAL;			
		break;
	}

	if(ret < 0){
		dev_err(led_cdev->dev, "brightness set failed ret = %d \n",ret);
		led->value_bak = value;
		schedule_delayed_work(&led->pwm_dwork, HZ/2);
	}else{
		led->value_pwm = value;
	}
}

static void pwm_failed_handler(struct work_struct *work)
{
    struct mx_hub_led *led = container_of(work, struct mx_hub_led, pwm_dwork.work);

	if(led->value_bak == led->value_pwm || led->data->is_update)
		return;

	printk("%s: value_bak = %d, value_pwm = %d\n", __func__, led->value_bak, led->value_pwm);

	mx_hub_led_brightness_set(&led->led_cdev, led->value_bak);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mx_hub_led_early_suspend(struct early_suspend *h)
{
	 struct mx_hub_led *led =
			 container_of(h, struct mx_hub_led, early_suspend);
	
	dev_dbg(led->led_cdev.dev, "early suspend \n");
 	mx_hub_set_led_current(&led->led_cdev,0x00);
}
 
static void mx_hub_led_late_resume(struct early_suspend *h)
{
	 struct mx_hub_led *led =
			 container_of(h, struct mx_hub_led, early_suspend);
	 
	dev_dbg(led->led_cdev.dev, "early resume \n");
 	mx_hub_set_led_current(&led->led_cdev,led->brightness);
}
#endif
 
static int mx_hub_led_probe(struct platform_device *pdev)
{
	 struct CWMCU_data *hub = dev_get_drvdata(pdev->dev.parent);
	 struct CWMCU_platform_data *pdata = hub->board_data;
	 struct mx_hub_led *led;
	 char name[20];
	 int ret;
 
	 if (!pdata) {
		 dev_err(&pdev->dev, "no platform data\n");
		 ret = -ENODEV;
	 }
	printk(" **** %s begin !!!!\n", __func__);
 
	 led = kzalloc(sizeof(*led), GFP_KERNEL);
	 if (led == NULL) {
		 ret = -ENOMEM;
		 goto err_mem;
	 }
 
	 led->id = pdev->id; 
	 led->data = hub;
 
	 snprintf(name, sizeof(name), "mx7-led");
	 led->led_cdev.name = name;
	 led->led_cdev.brightness = 0;
	 led->led_cdev.max_brightness= 0xFFF;
	 led->led_cdev.brightness_set = mx_hub_led_brightness_set;
 
	 mutex_init(&led->mutex);
	 platform_set_drvdata(pdev, led);
 
	 ret = led_classdev_register(&pdev->dev, &led->led_cdev);
	 if (ret < 0)
		 goto err_register_led;

 	INIT_DELAYED_WORK(&led->pwm_dwork, pwm_failed_handler);
#ifdef CONFIG_HAS_EARLYSUSPEND
	 led->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	 led->early_suspend.suspend = mx_hub_led_early_suspend;
	 led->early_suspend.resume = mx_hub_led_late_resume;
	 register_early_suspend(&led->early_suspend);
#endif

	mx_hub_set_led_current(&led->led_cdev,0);
 	
	 return 0;
 
 err_register_led:
	 kfree(led);
 err_mem:
	 return ret;
}
 
static int mx_hub_led_remove(struct platform_device *pdev)
{
	 struct mx_hub_led *led = platform_get_drvdata(pdev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	 unregister_early_suspend(&led->early_suspend);
#endif
	 led_classdev_unregister(&led->led_cdev);
	 kfree(led);
 
	 return 0;
}
static void mx_hub_led_shutdown(struct platform_device *pdev)
{
	struct mx_hub_led *led = platform_get_drvdata(pdev);
 	mx_hub_set_led_current(&led->led_cdev,0);
}

const struct platform_device_id mx_hub_led_id[] = {
	 { "mx7-hub-led",0 },
	 { },
};
 
static struct platform_driver mx_hub_led_driver = {
	 .driver = {
		 .name	= "mx-led",
		 .owner = THIS_MODULE,
	 },
	 .probe  = mx_hub_led_probe,
	 .remove = mx_hub_led_remove,
	 .shutdown = mx_hub_led_shutdown,
	 .id_table = mx_hub_led_id,
};
 
static int __init mx_hub_led_init(void)
{
	printk(" **** %s begin !!!!\n", __func__);
	return platform_driver_register(&mx_hub_led_driver);
}
module_init(mx_hub_led_init);
 
static void __exit mx_hub_led_exit(void)
{
	platform_driver_unregister(&mx_hub_led_driver);
}
module_exit(mx_hub_led_exit); 


MODULE_AUTHOR("Chwei <Chwei@meizu.com>");
MODULE_DESCRIPTION("MX Sensor Hub Leds");
MODULE_LICENSE("GPL");
