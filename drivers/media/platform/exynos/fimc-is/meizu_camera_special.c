/*
 * Meizu special feature of camera
 *
 * Copyright (C) 2015 Meizu Technology Co.Ltd, Zhuhai, China
 * Author: 	QuDao	<qudao@meizu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include "meizu_camera_special.h"
#define LE2464_NAME "le2464"

static struct fimc_is_device_sensor *g_fimc_is_device_sensor;

int camera_power_on(struct fimc_is_device_sensor *device)
{
	int ret = 0;
	struct exynos_platform_fimc_is_sensor *pdata;

	BUG_ON(!device);
	BUG_ON(!device->pdev);
	BUG_ON(!device->pdata);

	pdata = device->pdata;

	if (test_bit(FIMC_IS_SENSOR_GPIO_ON, &device->state)) {
		merr("%s : already gpio on", device, __func__);
		ret = -EBUSY;
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

int camera_power_off(struct fimc_is_device_sensor *device)
{
	int ret = 0;
	struct exynos_platform_fimc_is_sensor *pdata;

	BUG_ON(!device);
	BUG_ON(!device->pdev);
	BUG_ON(!device->pdata);

	pdata = device->pdata;

	if (!test_bit(FIMC_IS_SENSOR_GPIO_ON, &device->state)) {
		merr("%s : already gpio off", device, __func__);
		ret = -EBUSY;
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


int get_module_type(struct fimc_is_device_sensor *device)
{
	struct mz_module_info *module_info;
	struct platform_device *pdev;

	module_info = &device->mz_modu_info;
	pdev = device->pdev;

	if (!module_info->valid) {
		pr_info("%s(), camera %d 's module info is not ready\n",
			__func__, pdev->id);
		return -ENODATA;
	}

	return module_info->vendor_id;
}

int camera_module_active(struct fimc_is_device_sensor *device, bool enable)
{
	int ret;
	//struct fimc_is_core *core = device->private_data;

	if (enable) {
		ret = camera_power_on(device);
	} else {
		ret = camera_power_off(device);
	}

	if (ret) {
		pr_err("%s(), power %d failed:%d\n",
			__func__, enable, ret);
		return ret;
	}

	//fimc_is_i2c_enable_irq(core, enable);
	return 0;
}

static int le2464_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	dev_info(&client->dev, "%s(), addr:0x%x, name:%s\n",
		__func__, client->addr, client->name);

	if (IS_ERR_OR_NULL(g_fimc_is_device_sensor)) {
		dev_err(&client->dev, "%s(), err!fimc_is_device_sensor is illegal:0x%p",
			__func__, g_fimc_is_device_sensor);
		return -EINVAL;
	}
	LC898212XD_probe(g_fimc_is_device_sensor, client);
	return 0;
}

static int le2464_remove(struct i2c_client *client)
{
	g_fimc_is_device_sensor = NULL;
	return 0;
}

static const struct i2c_device_id le2464_id[] = {
	{LE2464_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, le2464_id);


#ifdef CONFIG_OF
static struct of_device_id le2464_of_match_table[] = {
	{
		.compatible = "otp,le2464",
	},
	{},
};
MODULE_DEVICE_TABLE(of, le2464_of_match_table);
#else
#define le2464_of_match_table NULL
#endif


static struct i2c_driver le2464_i2c_driver = {
	.driver = {
		   .name = LE2464_NAME,
		   .owner = THIS_MODULE,
		   .pm = NULL,
			.of_match_table = le2464_of_match_table,
		   },
	.probe = le2464_probe,
	.remove = le2464_remove,
	.id_table = le2464_id,
};

int meizu_special_feature_probe(struct fimc_is_device_sensor * device)
{
	BUG_ON(!device);
	BUG_ON(!device->private_data);

	g_fimc_is_device_sensor = device;

	dev_info(&device->pdev->dev, "%s(), fimc_is_device_sensor:0x%p, add otp i2c driver\n",
		__func__, g_fimc_is_device_sensor);
	i2c_add_driver(&le2464_i2c_driver);
	return 0;
}

