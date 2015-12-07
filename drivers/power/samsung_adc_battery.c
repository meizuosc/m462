/*
 *	Simple battery guage driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/iio/consumer.h>
#include <linux/power/samsung_adc_battery.h>

#undef BACKGROUND_UPDATING
#undef ACTIVE_NOTIFICATION

struct samsung_adc_battery_data {
	struct power_supply			ps;
	struct iio_channel			*adc_channel;
	struct platform_device			*pdev;
	int					voltage;
	int					capacity;
	int					capacity_before;
	struct samsung_adc_battery_pdata	pdata;
#ifdef BACKGROUND_UPDATING
	struct delayed_work			measuring_work;
#endif
};

static void calculate_battery_level(struct samsung_adc_battery_data *battery_data)
{
	struct samsung_adc_battery_pdata *pdata = &battery_data->pdata;
	int i, adc_val = 0, temp;

	for(i = pdata->sample_count; i > 0; i--) {
		if (iio_read_channel_raw(battery_data->adc_channel, &temp) < 0) {
			dev_err(&battery_data->pdev->dev, "Reading ADC is failed.\n");
		}
		adc_val += temp;
	}
	adc_val /= pdata->sample_count;

	battery_data->voltage = adc_val * pdata->adc_per_uv / 1000;

	if (battery_data->voltage <= pdata->no_battery_mv) {
		battery_data->capacity = 100;
	} else if (battery_data->voltage < pdata->min_mv) {
		battery_data->capacity = 0;
	} else if (battery_data->voltage <= pdata->max_mv) {
		battery_data->capacity = (battery_data->voltage - pdata->min_mv)
				* 100 / (pdata->max_mv - pdata->min_mv);
	} else {
		battery_data->capacity = 100;
	}

	dev_dbg(&battery_data->pdev->dev, "adc_val:%03X, voltage:%dmV, capacity:%d%%\n", adc_val,
			battery_data->voltage, battery_data->capacity);
}

#ifdef BACKGROUND_UPDATING
static void measure_battery(struct work_struct *work)
{
	struct samsung_adc_battery_data *battery_data = container_of(
			to_delayed_work(work), struct samsung_adc_battery_data,
			measuring_work);

	battery_data->capacity_before = battery_data->capacity;

	calculate_battery_level(battery_data);

#ifdef ACTIVE_NOTIFICATION
	if (battery_data->capacity_before != battery_data->capacity) {
		power_supply_changed(&battery_data->ps);
	}
#endif

	schedule_delayed_work(&battery_data->measuring_work, msecs_to_jiffies(battery_data->pdata.interval_ms));
}
#endif

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int get_property(struct power_supply *ps,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct samsung_adc_battery_data *battery_data =
			container_of(ps, struct samsung_adc_battery_data, ps);
	dev_dbg(&battery_data->pdev->dev, "%s\n", __func__);

#ifndef BACKGROUND_UPDATING
	calculate_battery_level(battery_data);
#endif

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = battery_data->capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery_data->voltage;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int samsung_adc_of_property_read_u32(struct device *dev,
		const char * propname, u32 * out_value)
{
	if (of_property_read_u32(dev->of_node, propname, out_value)) {
		dev_err(dev, "failed to read %s\n", propname);
		return -EINVAL;
	}
	return 0;
}

static int samsung_adc_parse_dt(struct device *dev, struct samsung_adc_battery_pdata *pdata)
{
	int rc = 0;

	rc |= samsung_adc_of_property_read_u32(dev, "interval_ms", &pdata->interval_ms);
	rc |= samsung_adc_of_property_read_u32(dev, "sample_count", &pdata->sample_count);
	rc |= samsung_adc_of_property_read_u32(dev, "no_battery_mv", &pdata->no_battery_mv);
	rc |= samsung_adc_of_property_read_u32(dev, "min_mv", &pdata->min_mv);
	rc |= samsung_adc_of_property_read_u32(dev, "max_mv", &pdata->max_mv);
	rc |= samsung_adc_of_property_read_u32(dev, "adc_per_uv", &pdata->adc_per_uv);

	return (rc ? -EINVAL : 0);
}

static int samsung_adc_battery_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct samsung_adc_battery_data *battery_data;
	struct device_node *np = pdev->dev.of_node;

	battery_data = devm_kzalloc(&pdev->dev, sizeof(*battery_data), GFP_KERNEL);
	if (battery_data == NULL) {
		rc = -ENOMEM;
		goto err_alloc_battery_data;
	}

	battery_data->pdev = pdev;
	if (np == NULL) {
		memcpy(&battery_data->pdata, dev_get_platdata(&pdev->dev), sizeof(battery_data->pdata));
	} else {
		if ((rc = samsung_adc_parse_dt(&pdev->dev, &battery_data->pdata)) < 0) {
			goto err_s3c_adc_register;
		}
	}

	battery_data->adc_channel = iio_channel_get(&pdev->dev, "battery_adc");
	if (IS_ERR_OR_NULL(battery_data->adc_channel)) {
		dev_err(&pdev->dev, "cannot register adc: %ld\n", PTR_ERR(battery_data->adc_channel));
		rc = PTR_ERR(battery_data->adc_channel);
		goto err_s3c_adc_register;
	}

	battery_data->ps.name = "battery";
	battery_data->ps.type = POWER_SUPPLY_TYPE_BATTERY;
	battery_data->ps.properties = battery_props;
	battery_data->ps.num_properties = ARRAY_SIZE(battery_props);
	battery_data->ps.get_property = get_property;
	battery_data->ps.use_for_apm = 1;

	rc = power_supply_register(&pdev->dev, &battery_data->ps);
	if (rc) {
		dev_err(&pdev->dev,
			"Failed registering to power_supply class\n");
		goto err_power_supply_register;
	}

	platform_set_drvdata(pdev, battery_data);

	dev_info(&pdev->dev, "successfully loaded\n");

#ifdef BACKGROUND_UPDATING
	INIT_DEFERRABLE_WORK(&battery_data->measuring_work, measure_battery);
	/* Schedule timer to check current status */
	schedule_delayed_work(&battery_data->measuring_work, msecs_to_jiffies(battery_data->pdata.interval_ms));
#endif

	return rc;

err_power_supply_register:
	iio_channel_release(battery_data->adc_channel);
err_s3c_adc_register:
	devm_kfree(&pdev->dev, battery_data);
err_alloc_battery_data:
	return rc;
}

static int samsung_adc_battery_remove(struct platform_device *pdev)
{
	struct samsung_adc_battery_data *battery_data = platform_get_drvdata(pdev);
#ifdef BACKGROUND_UPDATING
	cancel_delayed_work_sync(&battery_data->measuring_work);
#endif
	iio_channel_release(battery_data->adc_channel);
	devm_kfree(&pdev->dev, battery_data);
	return 0;
}

#ifdef BACKGROUND_UPDATING
static int samsung_adc_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct samsung_adc_battery_data *battery_data = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "%s\n", __func__);
	cancel_delayed_work_sync(&battery_data->measuring_work);
	return 0;
}

static int samsung_adc_battery_resume(struct platform_device *pdev)
{
	struct samsung_adc_battery_data *battery_data = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "%s\n", __func__);
	schedule_delayed_work(&battery_data->measuring_work,
			msecs_to_jiffies(battery_data->pdata.interval_ms));
	return 0;
}
#else
#define samsung_adc_battery_suspend NULL
#define samsung_adc_battery_resume NULL
#endif

const struct of_device_id samsung_battery_of_match_table[] = {
	{
		.compatible = "samsung,simple_adc_battery",
	},
};

static struct platform_driver samsung_adc_battery_driver = {
	.probe	= samsung_adc_battery_probe,
	.remove	= samsung_adc_battery_remove,
	.suspend = samsung_adc_battery_suspend,
	.resume = samsung_adc_battery_resume,
	.driver	= {
		.name	= "samsung_adc_battery",
		.owner	= THIS_MODULE,
		.of_match_table = samsung_battery_of_match_table,
	},
};

module_platform_driver(samsung_adc_battery_driver);

MODULE_AUTHOR("Gyeongtaek Lee <gt82.lee@samsung.com>");
MODULE_DESCRIPTION("simple battery guage driver using Samsung ADC");
MODULE_LICENSE("GPL");
