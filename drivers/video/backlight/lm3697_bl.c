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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/fb.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_data/lm3697_bl.h>

#define REG_CHIP_REVSION                 0x00

#define REG_SOFT_RESET                   0x01
#define CHIP_NORMAL   (0 << 0)
#define CHIP_RESET    (1 << 0)

#define REG_HVLED_CONF                   0x10
#define HVLED1_IS_BANK_A_TYPE   (0 << 0)
#define HVLED1_IS_BANK_B_TYPE   (1 << 0)
#define HVLED2_IS_BANK_A_TYPE   (0 << 1)
#define HVLED2_IS_BANK_B_TYPE   (1 << 1)
#define HVLED3_IS_BANK_A_TYPE   (0 << 2)
#define HVLED3_IS_BANK_B_TYPE   (1 << 2)
#define HVLED_CONF_VALUE(value, led1_type, led2_type, led3_type) \
	do { \
		value = 0; \
		value |= ((led1_type == LM3697_BANK_A) ? HVLED1_IS_BANK_A_TYPE : HVLED1_IS_BANK_B_TYPE); \
		value |= ((led2_type == LM3697_BANK_A) ? HVLED2_IS_BANK_A_TYPE : HVLED2_IS_BANK_B_TYPE); \
		value |= ((led3_type == LM3697_BANK_A) ? HVLED3_IS_BANK_A_TYPE : HVLED3_IS_BANK_B_TYPE); \
	} while (0)


#define REG_BANK_A_S_RAMP_TIME           0x11
#define REG_BANK_B_S_RAMP_TIME           0x12
#define REG_BANK_AB_RUNTIME_RAMP_TIME    0x13
#define REG_BANK_AB_RUNTIME_RAMP_CONF    0x14

/* 0x15 RESERVE */
#define REG_BRIGHTNESS_CONF              0x16
#define REG_BANK_A_FULLSCALE_CUR_SETTING 0x17
#define REG_BANK_B_FULLSCALE_CUR_SETTING 0x18

#define REG_HVLED_FEEDBACK_CONF          0x19
#define HVLED1_FEEDBACK_CONNECT_COUT       (1 << 0)
#define HVLED1_FEEDBACK_NOT_CONNECT_COUT   (0 << 0)
#define HVLED2_FEEDBACK_CONNECT_COUT       (1 << 1)
#define HVLED2_FEEDBACK_NOT_CONNECT_COUT   (0 << 1)
#define HVLED3_FEEDBACK_CONNECT_COUT       (1 << 2)
#define HVLED3_FEEDBACK_NOT_CONNECT_COUT   (0 << 2)
#define HVLED_FEEDBACK_CONF_VALUE(value, led1_enable, led2_enable, led3_enable) \
	do { \
		value = 0; \
		value |= ((led1_enable == LM3697_LED_ENABLE) ? HVLED1_FEEDBACK_CONNECT_COUT : HVLED1_FEEDBACK_NOT_CONNECT_COUT); \
		value |= ((led2_enable == LM3697_LED_ENABLE) ? HVLED2_FEEDBACK_CONNECT_COUT : HVLED2_FEEDBACK_NOT_CONNECT_COUT); \
		value |= ((led3_enable == LM3697_LED_ENABLE) ? HVLED3_FEEDBACK_CONNECT_COUT : HVLED3_FEEDBACK_NOT_CONNECT_COUT); \
	} while (0)


#define REG_BOOST_CTRL                   0x1A
#define BOOST_FREQ_500KHZ (0 << 0)
#define BOOST_FREQ_1MHZ   (1 << 0)
#define OVP_16V  (0 << 1)
#define OVP_24V  (1 << 1)
#define OVP_32V  (2 << 1)
#define OVP_40V  (3 << 1)
#define AUTO_FREQ_ENABLE   (1 << 3)
#define AUTO_FREQ_DISABLE  (0 << 3)
#define UPDATE_BOOST_CTRL_VALUE(value, boost_freq, ovp) \
	do { \
		value = 0; \
		switch (boost_freq) { \
		case LM3697_BOOST_FREQ_500KHZ: \
			value |= BOOST_FREQ_500KHZ; \
			break; \
		case LM3697_BOOST_REQ_1MHZ: \
			value |= BOOST_FREQ_1MHZ; \
			break; \
		}; \
		switch (ovp) { \
		case LM3697_OVP_16V: \
			value |= OVP_16V; \
			break; \
		case LM3697_OVP_24V: \
			value |= OVP_24V; \
			break; \
		case LM3697_OVP_32V: \
			value |= OVP_32V; \
			break; \
		case LM3697_OVP_40V: \
			value |= OVP_40V; \
			break; \
		}; \
	} while (0)

#define REG_AUTO_FRE_THRESHOLD           0x1B

#define REG_PWM_CONF                     0x1C
#define PWM_BANK_A_ENABLE    (1 << 0)
#define PWM_BANK_A_DISABLE   (0 << 0)
#define PWM_BANK_B_ENABLE    (1 << 1)
#define PWM_BANK_B_DISABLE   (0 << 1)
#define PWM_ACTIVE_LOW       (0 << 2)
#define PWM_ACTIVE_HIGH      (1 << 2)
#define PWM_ZERO_DET_ENABLE  (1 << 3)
#define PWM_ZERO_DET_DISABLE (0 << 3)
#define PWM_CONF_VALUE(value, pwm_ctrl, pwm_polarity, pwm_zero_detection) \
	do { \
		int __pwm_inactive = 0; \
		value = 0; \
		switch (pwm_ctrl) { \
		case LM3697_PWM_CTRL_DISABLE: \
			__pwm_inactive = 1; \
			break; \
		case LM3697_PWM_CTRL_BANK_A: \
			value |= PWM_BANK_A_ENABLE; \
			break; \
		case LM3697_PWM_CTRL_BANK_B: \
			value |= PWM_BANK_B_ENABLE; \
			break; \
		case LM3697_PWM_CTRL_BANK_ALL: \
			value |= PWM_BANK_A_ENABLE; \
			value |= PWM_BANK_B_ENABLE; \
			break; \
		}; \
		if (!__pwm_inactive) { \
			value |= ((pwm_polarity == LM3697_PWM_ACTIVE_LOW) ? PWM_ACTIVE_LOW : PWM_ACTIVE_HIGH); \
			value |= ((pwm_zero_detection == LM3697_PWM_ZERO_DET_DISABLE) ? PWM_ZERO_DET_DISABLE : PWM_ZERO_DET_ENABLE); \
		} \
	} while (0)

#define REG_BANK_A_BRIGHTNESS_LSB        0x20
#define REG_BANK_A_BRIGHTNESS_MSB        0x21
#define REG_BANK_B_BRIGHTNESS_LSB        0x22
#define REG_BANK_B_BRIGHTNESS_MSB        0x23

#define REG_BANK_CONF             0x24
#define BANK_A_ENABLE   (1 << 0)
#define BANK_A_DISABLE  (0 << 0)
#define BANK_B_ENABLE   (1 << 1)
#define BANK_B_DISABLE  (0 << 1)
#define BANK_CONF_VALUE(value, bank_a_enable, bank_b_enable) \
	do { \
		value = 0; \
		value |= ((bank_a_enable == LM3697_BANK_ENABLE) ? BANK_A_ENABLE : BANK_A_DISABLE); \
		value |= ((bank_b_enable == LM3697_BANK_ENABLE) ? BANK_B_ENABLE : BANK_B_DISABLE); \
	} while (0)


#define REG_HVLED_OPEN_FAULT_STATUS      0xB0
#define REG_HVLED_SHORT_FAULT_STATUS     0xB2
#define REG_HVLED_FAULT_CONF             0xB4
#define HVLED_OPEN_FAULT_ENABLE   (1 << 0)
#define HVLED_OPEN_FAULT_DISABLE  (0 << 0)
#define HVLED_SHORT_FAULT_ENABLE  (1 << 1)
#define HVLED_SHORT_FAULT_DISABLE (0 << 1)

#define REG_MAX REG_HVLED_FAULT_CONF

struct lm3697_chip_data {
	struct device *dev;
	struct lm3697_platform_data *pdata;

	struct backlight_device *bled;
	struct regmap *regmap;
	struct regulator *bl_en;

	unsigned int version; /* Chip Version. */
	unsigned int enabled;
};

#ifdef CONFIG_BACKLIGHT_LM3697_DEBUG
static void lm3697_dump_register(struct lm3697_chip_data *pchip)
{
	struct lm3697_reg_data {
		unsigned int reg;
		unsigned int val;
		char tag[6];
	} lm3697_reg_data[] = {
		{REG_HVLED_CONF, 0, "0x10"},
		{REG_BANK_A_S_RAMP_TIME, 0, "0x11"},
		{REG_BANK_B_S_RAMP_TIME, 0, "0x12"},
		{REG_BANK_AB_RUNTIME_RAMP_TIME, 0, "0x13"},
		{REG_BANK_AB_RUNTIME_RAMP_CONF, 0, "0x14"},
		{REG_BRIGHTNESS_CONF, 0, "0x16"},
		{REG_BANK_A_FULLSCALE_CUR_SETTING, 0, "0x17"},
		{REG_BANK_B_FULLSCALE_CUR_SETTING, 0, "0x18"},
		{REG_HVLED_FEEDBACK_CONF, 0, "0x19"},
		{REG_BOOST_CTRL, 0, "0x1A"},
		{REG_AUTO_FRE_THRESHOLD, 0, "0x1B"},
		{REG_PWM_CONF, 0, "0x1C"},
		{REG_BANK_A_BRIGHTNESS_LSB, 0, "0x20"},
		{REG_BANK_A_BRIGHTNESS_MSB, 0, "0x21"},
		{REG_BANK_B_BRIGHTNESS_LSB, 0, "0x22"},
		{REG_BANK_B_BRIGHTNESS_MSB, 0, "0x23"},
		{REG_BANK_CONF, 0, "0x24"},
		{REG_HVLED_OPEN_FAULT_STATUS, 0, "0xB0"},
		{REG_HVLED_SHORT_FAULT_STATUS, 0, "0xB2"},
		{REG_HVLED_FAULT_CONF, 0, "0xB4"},
	};
	int i, ret = 0;

	printk("lm3697 register value - [reg:value]:\r\n");
	for (i = 0; i < ARRAY_SIZE(lm3697_reg_data); i++) {
		ret = regmap_read(pchip->regmap, lm3697_reg_data[i].reg, &lm3697_reg_data[i].val);
		if (ret < 0)
			printk("i2c error when lm3697 dump register!\r\n");
		printk("[%s : 0x%02x]\r\n", lm3697_reg_data[i].tag, lm3697_reg_data[i].val);
	}
}
#endif

static int lm3697_power_on(struct lm3697_chip_data *pchip, int onoff)
{
	int ret = 0;

        if (pchip->enabled == onoff)
                return 0;

        if (IS_ERR(pchip->bl_en)) {
                dev_err(pchip->dev, "failed to get the backlight chip regulator!\n");
                return PTR_ERR(pchip->bl_en);
        }

        if (onoff) {
		ret = regulator_enable(pchip->bl_en);
	} else {
                ret = regulator_disable(pchip->bl_en);
	}

        pchip->enabled = onoff;

        return ret;
}
	
static int lm3697_chip_init(struct lm3697_chip_data *pchip)
{
	int ret;
	unsigned int reg_val;
	struct lm3697_platform_data *pdata = pchip->pdata;

	/* power on */
	ret = lm3697_power_on(pchip, 1);
	if (ret) {
		dev_err(pchip->dev, "failed to power on backlight chip.\n");
		return ret;
	}

	ret = regmap_read(pchip->regmap, REG_CHIP_REVSION, &reg_val);
	if (ret < 0)
		goto reg_err;
	pchip->version = reg_val;

#ifdef CONFIG_BACKLIGHT_LM3697_VERFIY	
	ret = regmap_write(pchip->regmap, REG_HVLED_FAULT_CONF, HVLED_OPEN_FAULT_ENABLE);
	if (ret < 0)
		goto reg_err;
#endif

	HVLED_CONF_VALUE(reg_val, pdata->led1_type, pdata->led2_type, pdata->led3_type);
	ret = regmap_write(pchip->regmap, REG_HVLED_CONF, reg_val);
	if (ret < 0)
		goto reg_err;

	HVLED_FEEDBACK_CONF_VALUE(reg_val, pdata->led1_enable, pdata->led2_enable, pdata->led3_enable);
	ret = regmap_write(pchip->regmap, REG_HVLED_FEEDBACK_CONF, reg_val);
	if (ret < 0)
		goto reg_err;

	UPDATE_BOOST_CTRL_VALUE(reg_val, pdata->boost_freq, pdata->ovp);
	ret = regmap_update_bits(pchip->regmap, REG_BOOST_CTRL, 0x07, reg_val);
	if (ret < 0)
		goto reg_err;

	PWM_CONF_VALUE(reg_val, pdata->pwm_ctrl, pdata->pwm_polarity, pdata->pwm_zero_detection);
	ret = regmap_write(pchip->regmap, REG_PWM_CONF, reg_val);
	if (ret < 0)
		goto reg_err;

#ifdef CONFIG_BACKLIGHT_LM3697_VERFIY
	if (pdata->bank_a_enable == LM3697_BANK_ENABLE) {
		ret = regmap_write(pchip->regmap, REG_BANK_A_BRIGHTNESS_MSB, 0xff);
		if (ret < 0)
			goto reg_err;
	}
	if (pdata->bank_b_enable == LM3697_BANK_ENABLE) {
		ret = regmap_write(pchip->regmap, REG_BANK_B_BRIGHTNESS_MSB, 0xff);
		if (ret < 0)
			goto reg_err;
	}
#endif

	BANK_CONF_VALUE(reg_val, pdata->bank_a_enable, pdata->bank_b_enable);
	ret = regmap_write(pchip->regmap, REG_BANK_CONF, reg_val);
	if (ret < 0)
		goto reg_err;

#ifdef CONFIG_BACKLIGHT_LM3697_VERFIY
	ret = regmap_read(pchip->regmap, REG_HVLED_OPEN_FAULT_STATUS, &reg_val);
	if (ret < 0)
		goto reg_err;

	if ((pdata->led1_enable == LM3697_LED_ENABLE) && (reg_val & 0x01)) {
		printk("[Info] Lm3697 Led1 succeed to open! [0x%02x]\r\n", reg_val);
	}
	if ((pdata->led2_enable == LM3697_LED_ENABLE) && (reg_val & 0x02)) {
		printk("[Info] Lm3697 Led2 succeed to open! [0x%02x]\r\n", reg_val);
	}
	if ((pdata->led3_enable == LM3697_LED_ENABLE) && (reg_val & 0x04)) {
		printk("[Info] Lm3697 Led3 succeed to open! [0x%02x]\r\n", reg_val);
	}
#endif

#ifdef CONFIG_BACKLIGHT_LM3697_DEBUG
	lm3697_dump_register(pchip);
#endif

	return ret;

reg_err:
	/* Failed to init chip, power off chip. */
	ret = lm3697_power_on(pchip, 0);
	if (ret) {
		dev_err(pchip->dev, "failed to power off backlight chip.\n");
		return ret;
	}
	dev_err(pchip->dev, "i2c failed to access register when init chip.\n");
	return ret;
}

/* update and get brightness */
static int lm3697_bled_update_status(struct backlight_device *bl)
{
	int ret;
	unsigned int brightness_lsb = 0, brightness_msb = 0;
	struct lm3697_chip_data *pchip = bl_get_data(bl);
	struct lm3697_platform_data *pdata = pchip->pdata;

	dev_dbg(pchip->dev, "backlight event is %s [%s]\r\n",
		((bl->props.isNotify == 0) ? "not care" :
		(bl->props.fb_blank == FB_BLANK_UNBLANK ? "unblank" :
		(bl->props.fb_blank == FB_BLANK_POWERDOWN) ? "power down" : "other")),
		(bl->props.isNotify == 1 ? "notify" : "not notify"));

	if ((bl->props.isNotify == 1) &&
		(bl->props.fb_blank == FB_BLANK_UNBLANK)) {
		lm3697_chip_init(pchip);

		goto out;
	} else if ((bl->props.isNotify == 1) &&
		(bl->props.fb_blank == FB_BLANK_POWERDOWN)) {
		lm3697_power_on(pchip, 0);

		goto out;
	}

	if (!pchip->enabled) {
		dev_err(pchip->dev, "[Warn] Chip in power off when updating backlight status!\n");
		goto out;
	}

	dev_dbg(pchip->dev, "update brightness value:%d.\r\n",
		bl->props.brightness);

	brightness_lsb = bl->props.brightness & 0x7;
	brightness_msb = (bl->props.brightness >> 3) & 0xFF;

	if (likely(pdata->bank_a_enable == LM3697_BANK_ENABLE)) {
		goto BANK_A;
	} else if (pdata->bank_b_enable == LM3697_BANK_ENABLE) {
		goto BANK_B;
	}

	dev_err(pchip->dev, "[ERROR] please enable Bank A or Bank B!\r\n");
	goto out;

BANK_A:
	ret = regmap_write(pchip->regmap, REG_BANK_A_BRIGHTNESS_LSB, brightness_lsb);
	if (ret < 0)
		goto err_out;
	ret = regmap_write(pchip->regmap, REG_BANK_A_BRIGHTNESS_MSB, brightness_msb);
	if (ret < 0)
		goto err_out;

	return bl->props.brightness;

BANK_B:
	ret = regmap_write(pchip->regmap, REG_BANK_B_BRIGHTNESS_LSB, brightness_lsb);
	if (ret < 0)
		goto err_out;
	ret = regmap_write(pchip->regmap, REG_BANK_B_BRIGHTNESS_MSB, brightness_msb);
	if (ret < 0)
		goto err_out;

	return bl->props.brightness;

err_out:
	dev_err(pchip->dev, "failed to write i2c when updating backlight status.\n");
out:
	return bl->props.brightness;
}

static int lm3697_bled_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops lm3697_bled_ops = {
	.update_status = lm3697_bled_update_status,
	.get_brightness = lm3697_bled_get_brightness,
};

static const struct regmap_config lm3697_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static struct lm3697_platform_data *lm3697_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct lm3697_platform_data *pdata = NULL;

	if (!np) {
		dev_err(dev, "%s: no devicenode given.\r\n", of_node_full_name(np));
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(struct lm3697_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "no memory for lm3697 platform data.\r\n");
		return ERR_PTR(-ENOMEM);
	}

	if (of_find_property(np, "enable-bank-a", NULL))
		pdata->bank_a_enable = LM3697_BANK_ENABLE;

	if (of_find_property(np, "enable-bank-b", NULL))
		pdata->bank_b_enable = LM3697_BANK_ENABLE;

	if (of_property_read_u32(np, "led1-type", &pdata->led1_type))
		pdata->led1_type = LM3697_BANK_A;

	if (of_property_read_u32(np, "led2-type", &pdata->led2_type))
		pdata->led2_type = LM3697_BANK_A;

	if (of_property_read_u32(np, "led3-type", &pdata->led3_type))
		pdata->led3_type = LM3697_BANK_B;

	if (of_find_property(np, "enable-led1", NULL))
		pdata->led1_enable = LM3697_LED_ENABLE;

	if (of_find_property(np, "enable-led2", NULL))
		pdata->led2_enable = LM3697_LED_ENABLE;

	if (of_find_property(np, "enable-led3", NULL))
		pdata->led3_enable = LM3697_LED_ENABLE;

	if (of_property_read_u32(np, "boost-freq", &pdata->boost_freq))
		pdata->boost_freq = LM3697_BOOST_FREQ_500KHZ;

	if (of_property_read_u32(np, "ovp", &pdata->ovp))
		pdata->ovp = LM3697_OVP_32V;

	if (of_property_read_u32(np, "pwm-ctrl", &pdata->pwm_ctrl))
		pdata->pwm_ctrl = LM3697_PWM_CTRL_DISABLE;

	if (of_property_read_u32(np, "max-brightness", &pdata->max_brightness))
		pdata->max_brightness = 2047;

	if (of_property_read_u32(np, "def-brightness", &pdata->default_brightness))
		pdata->default_brightness = 1200;

	dev_dbg(dev, "lm3697 dump platform data:\r\n");
	dev_dbg(dev, "[bank_a_en:%d, bank_b_en:%d\r\n",
		pdata->bank_a_enable, pdata->bank_b_enable);
	dev_dbg(dev, "Led1_type:%d, Led2_type:%d, Led3_type:%d\r\n",
		pdata->led1_type, pdata->led2_type, pdata->led3_type);
	dev_dbg(dev, "Led1_en:%d, Led2_en:%d, Led3_en:%d\r\n",
		pdata->led1_enable, pdata->led2_enable, pdata->led3_enable);
	dev_dbg(dev, "boost:%d, ovp:%d, pwm:%d, def:%d, max:%d].\r\n",
		pdata->boost_freq, pdata->ovp, pdata->pwm_ctrl, pdata->default_brightness, pdata->max_brightness);

	return pdata;
}

static int lm3697_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	int ret;
	struct lm3697_chip_data *pchip = NULL;
	struct lm3697_platform_data *pdata = NULL;
	struct backlight_properties props;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "backlight chip i2c functionality check fail.\n");
		return -EOPNOTSUPP;
	}

	if (client->dev.of_node) {
		pdata = lm3697_parse_dt(&client->dev);
		if (IS_ERR(pdata)) {
			dev_err(&client->dev, "failed to parse lm3697 dt!\r\n");
			return -EINVAL;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "no lm3697 platform data!\r\n");
			return -EINVAL;
		}
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct lm3697_chip_data), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;

	pchip->pdata = pdata;
	pchip->dev = &client->dev;
	pchip->bl_en = devm_regulator_get(pchip->dev, "bl-5v");
	if (IS_ERR(pchip->bl_en)) {
		dev_err(&client->dev, "backlight chip regulator is not available.\n");
		return EINVAL;
	}

	pchip->regmap = devm_regmap_init_i2c(client, &lm3697_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "failed to allocate register map[%d].\n", ret);
		goto i2c_err_out;
	}
	i2c_set_clientdata(client, pchip);

	ret = lm3697_chip_init(pchip);
	if (ret < 0) {
		dev_err(&client->dev, "failed to init backlight chip.\n");
		goto err_out;
	}

	props.type = BACKLIGHT_RAW;
	props.brightness = pdata->default_brightness;
	props.max_brightness = pdata->max_brightness;
	pchip->bled = backlight_device_register(LM3697_NAME, pchip->dev, pchip,
				      &lm3697_bled_ops, &props);
	if (IS_ERR(pchip->bled)) {
		dev_err(&client->dev, "failed to register backlight chip.\n");
		ret = PTR_ERR(pchip->bled);
		goto err_out;
	}

	backlight_update_status(pchip->bled);

	dev_info(&client->dev, "[Ver%x] backlight chip succeed to load\r\n", pchip->version);

	return 0;

err_out:
	lm3697_power_on(pchip, 0);
i2c_err_out:
	regulator_put(pchip->bl_en);
	return ret;
}

static int lm3697_remove(struct i2c_client *client)
{
	struct lm3697_chip_data *pchip = i2c_get_clientdata(client);

	pchip->bled->props.brightness = 0;
	backlight_update_status(pchip->bled);

	regmap_write(pchip->regmap, REG_BANK_CONF, 0x0);

	if (pchip->bled) {
		backlight_device_unregister(pchip->bled);
	}
	
	lm3697_power_on(pchip, 0);

	regulator_put(pchip->bl_en);

	return 0;
}

static const struct i2c_device_id lm3697_id[] = {
	{LM3697_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id lm3697_bl_dt_ids[] = {
	{ .compatible = "ti,lm3697" },
	{ }
};
#endif

MODULE_DEVICE_TABLE(i2c, lm3697_id);
static struct i2c_driver lm3697_i2c_driver = {
	.driver = {
		   .name = LM3697_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(lm3697_bl_dt_ids),
#endif
	},
	.probe = lm3697_probe,
	.remove = lm3697_remove,
	.id_table = lm3697_id,
};

static int __init lm3697_init(void)
{
	int rc;

	rc = i2c_add_driver(&lm3697_i2c_driver);
	if (rc) {
		pr_err("%s failed: i2c_add_driver rc=%d\n", __func__, rc);
		goto init_exit;
	}
	return 0;

init_exit:
	return rc;
}

static void __exit lm3697_exit(void)
{
	i2c_del_driver(&lm3697_i2c_driver);
}

module_init(lm3697_init);
module_exit(lm3697_exit);

MODULE_DESCRIPTION("MEIZU Backlight driver for LM3697");
MODULE_AUTHOR("WangBo <wangbo@meizu.com>");
MODULE_LICENSE("GPL v2");
