/*
* Simple driver for Texas Instruments LM3697 Backlight chip
* Copyright (C) 2014 MEIZU
* Author: wangbo@meizu.com
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/

#ifndef __LINUX_LM3697_H__
#define __LINUX_LM3697_H__

#define LM3697_NAME "lm3697_bl"

enum lm3697_bank_type {
	LM3697_BANK_A,
	LM3697_BANK_B,
};

enum lm3697_bank_active {
	LM3697_BANK_DISABLE,
	LM3697_BANK_ENABLE,
};

enum lm3697_led_active {
	LM3697_LED_DISABLE,
	LM3697_LED_ENABLE,
};

enum lm3697_boost_freq {
	LM3697_BOOST_FREQ_500KHZ,
	LM3697_BOOST_REQ_1MHZ,
};

enum lm3697_ovp {
	LM3697_OVP_16V,
	LM3697_OVP_24V,
	LM3697_OVP_32V,
	LM3697_OVP_40V,
};

enum lm3697_pwm_ctrl {
	LM3697_PWM_CTRL_DISABLE,
	LM3697_PWM_CTRL_BANK_A,
	LM3697_PWM_CTRL_BANK_B,
	LM3697_PWM_CTRL_BANK_ALL,
};

enum lm3697_pwm_active {
	LM3697_PWM_ACTIVE_LOW,
	LM3697_PWM_ACTIVE_HIGH,
};

enum lm3697_pwm_zero_detection {
	LM3697_PWM_ZERO_DET_DISABLE,
	LM3697_PWM_ZERO_DET_ENABLE,
};


struct lm3697_platform_data {
	/* BANK & HVLED Config */
	unsigned int bank_a_enable;
	unsigned int bank_b_enable;
	unsigned int led1_type;
	unsigned int led2_type;
	unsigned int led3_type;
	unsigned int led1_enable;
	unsigned int led2_enable;
	unsigned int led3_enable;

	/* Boost Config */
	unsigned int boost_freq;
	unsigned int ovp;

	/* PWM Config */
	unsigned int pwm_ctrl;
	unsigned int pwm_polarity;
	unsigned int pwm_zero_detection;

	int default_brightness;
	int max_brightness;
};
#endif /* __LINUX_LM3697_H__ */
