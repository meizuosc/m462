/*
* Simple driver for TPS65132 LCD Voltage Supply chip
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
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>

#define TPS65132_NAME "tps65132-vol"

#define REG_TPS65132_VPOS  		0x00
#define REG_TPS65132_VNEG  		0x01
#define REG_TPS65132_APP_DIS 	        0x02
#define REG_TPS65132_CTRL		0xff
#define REG_MAX REG_TPS65132_CTRL

struct tps65132_platform_data {
	unsigned int vpos;
	unsigned int vneg;
	unsigned int app;
	unsigned int vpos_en;
	unsigned int vneg_en;
	unsigned int stored_en;
};

struct tps65132_chip_data {
	struct device *dev;
	struct tps65132_platform_data *pdata;
	struct regmap *regmap;
};

/* initialize chip */
static int tps65132_chip_init(struct tps65132_chip_data *pchip)
{
	int ret = 0;
	unsigned int reg_val;
	struct tps65132_platform_data *pdata = pchip->pdata;

	/* VPOS control */
	ret = regmap_write(pchip->regmap, REG_TPS65132_VPOS, pdata->vpos);
	if (ret < 0)
		goto reg_err;

	/* VNEG control */
	ret = regmap_write(pchip->regmap, REG_TPS65132_VNEG, pdata->vneg);
	if (ret < 0)
		goto reg_err;

	/* app dis control */
	reg_val = 0;
	reg_val |= ((pdata->app << 6) | (pdata->vpos_en << 1) | pdata->vneg_en);
	ret = regmap_write(pchip->regmap, REG_TPS65132_APP_DIS, reg_val);
	if (ret < 0)
		goto reg_err;

	/* set to the non-volatile eprom and wait*/
	reg_val = (1 << 7);
	ret = regmap_write(pchip->regmap, REG_TPS65132_CTRL, reg_val);
	if (ret < 0)
		goto reg_err;

	return ret;

reg_err:
	dev_err(pchip->dev, "i2c failed to access register when init chip\n");
	return ret;
}

static const struct regmap_config tps65132_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static struct tps65132_platform_data *tps65132_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct tps65132_platform_data *pdata = NULL;

	if (!np) {
		dev_err(dev, "%s: no devicenode given.\r\n", of_node_full_name(np));
		return ERR_PTR(-EINVAL);
	}

	pdata = devm_kzalloc(dev, sizeof(struct tps65132_platform_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "no memory for tps65132 platform data.\r\n");
		return ERR_PTR(-ENOMEM);
	}

	if (of_property_read_u32(np, "vpos", &pdata->vpos))
		pdata->vpos = 0xB;

	if (of_property_read_u32(np, "vneg", &pdata->vneg))
		pdata->vneg = 0xB;

	if (of_find_property(np, "enable-app", NULL))
		pdata->app = 1;

	if (of_find_property(np, "enable-vneg", NULL))
		pdata->vneg_en = 1;

	if (of_find_property(np, "enable-vpos", NULL))
		pdata->vpos_en = 1;

	if (of_find_property(np, "enable-store", NULL))
		pdata->stored_en = 1;

#if defined(CONFIG_DECON_LCD_M6X)
	pdata->vpos = 0xB;
	pdata->vneg = 0xB;
#elif defined(CONFIG_DECON_LCD_M7X)
	pdata->vpos = 0xF;
	pdata->vneg = 0xF;
#endif

	dev_dbg(dev, "tps65132 dump platform data:\r\n");
	dev_dbg(dev, "[vpos:0x%x, vneg:0x%x, app:%d, vneg_en:%d, vpos_en:%d, store:%d].\r\n",
		pdata->vpos, pdata->vneg, pdata->app, pdata->vneg_en, pdata->vpos_en, pdata->stored_en);

	return pdata;
}

static int tps65132_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct tps65132_platform_data *pdata = NULL;
	struct tps65132_chip_data *pchip = NULL;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Lcd voltage supply chip i2c functionality check fail.\n");
		return -EOPNOTSUPP;
	}

	if (client->dev.of_node) {
		pdata = tps65132_parse_dt(&client->dev);
		if (IS_ERR(pdata)) {
			dev_err(&client->dev, "failed to parse tps65132 dt!\r\n");
			return -EINVAL;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "no tps65132 platform data!\r\n");
			return -EINVAL;
		}
	}

	pchip = devm_kzalloc(&client->dev, sizeof(struct tps65132_chip_data), GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;

	pchip->pdata = pdata;
	pchip->dev = &client->dev;

	pchip->regmap = devm_regmap_init_i2c(client, &tps65132_regmap);
	if (IS_ERR(pchip->regmap)) {
		ret = PTR_ERR(pchip->regmap);
		dev_err(&client->dev, "failed to allocate register map[%d].\n", ret);
		return ret;
	}
	i2c_set_clientdata(client, pchip);

	tps65132_chip_init(pchip);

	dev_info(&client->dev, "lcd voltage supply chip succeed to load\r\n");

	return 0;
}

static int tps65132_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id tps65132_id[] = {
	{TPS65132_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id tps65132_dt_ids[] = {
	{ .compatible = "ti,tps65132" },
	{ }
};
#endif

MODULE_DEVICE_TABLE(i2c, tps65132_id);
static struct i2c_driver tps65132_i2c_driver = {
	.driver = {
		   .name = TPS65132_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(tps65132_dt_ids),
#endif
	},
	.probe = tps65132_probe,
	.remove = tps65132_remove,
	.id_table = tps65132_id,
};

static int __init tps65132_init(void)
{
	int rc;

	rc = i2c_add_driver(&tps65132_i2c_driver);
	if (rc) {
		pr_err("%s failed: i2c_add_driver rc=%d\n", __func__, rc);
		goto init_exit;
	}
	return 0;

init_exit:
	return rc;
}

static void __exit tps65132_exit(void)
{
	i2c_del_driver(&tps65132_i2c_driver);
}

module_init(tps65132_init);
module_exit(tps65132_exit);

MODULE_DESCRIPTION("MEIZU lcd voltage supply driver for TPS65132");
MODULE_AUTHOR("WangBo <wangbo@meizu.com>");
MODULE_LICENSE("GPL v2");
