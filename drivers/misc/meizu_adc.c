/*
 *	Simple  adc provider driver
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
#include <linux/power/meizu_adc.h>

#define TAG "[meizu_adc]"

struct meizu_adc_data {
	struct iio_channel			*batid_channel;
	struct iio_channel			*acid_channel;
	struct iio_channel			*thermal_channel;
	struct platform_device			*pdev;
};

static struct meizu_adc_data *global_adc_data;
static DEFINE_MUTEX(adc_lock);

int read_batid_adc(void)
{
	struct meizu_adc_data *adc_data;
	int i, adc_val = 0, temp=0;
	static const int sample_count = 5;

	mutex_lock(&adc_lock);
	if (global_adc_data) {
		adc_data = global_adc_data;
		for(i = sample_count; i > 0; i--) {
			if (iio_read_channel_raw(adc_data->batid_channel, &temp) < 0) {
				dev_err(&adc_data->pdev->dev, "Reading ADC batid failed.\n");
			}
			adc_val += temp;
		}
	}else{
		pr_info("%s adc channel not ready\n",__func__);
		adc_val = 0;
	}
	mutex_unlock(&adc_lock);

	return adc_val/sample_count;

}

int read_acid_adc(void)
{
	struct meizu_adc_data *adc_data;
	int i, adc_val = 0, temp=0;
	static const int sample_count = 5;

	mutex_lock(&adc_lock);
	if (global_adc_data) {
		adc_data = global_adc_data;
		for(i = sample_count; i > 0; i--) {
			if (iio_read_channel_raw(adc_data->acid_channel, &temp) < 0) {
				dev_err(&adc_data->pdev->dev, "Reading ADC acid failed.\n");
			}
			adc_val += temp;
		}
	}else{
		pr_info("%s adc channel is not ready\n",__func__);
		adc_val = 0;
	}
	mutex_unlock(&adc_lock);

	return adc_val/sample_count;
}

int read_thermal_adc(void)
{
	struct meizu_adc_data *adc_data;
	int i, adc_val = 0, temp;
	static int sample_count = 1;
	int ret = INVALID_ADC_VAL;

	mutex_lock(&adc_lock);
	if (global_adc_data) {
		adc_data = global_adc_data;

		for (i = sample_count; i > 0; i--) {
			if (iio_read_channel_raw(adc_data->thermal_channel, &temp) < 0) {
				dev_err(&adc_data->pdev->dev, "Reading ADC thermal_adc is failed.\n");
				--sample_count;
				temp = 0;
			}
			adc_val += temp;
		}

		if (sample_count)
			ret = adc_val/sample_count;

	} else {
		pr_warn("ADC is not ready, just waitiing .\n");
	}
	mutex_unlock(&adc_lock);

	return ret;
}

#if 0
static int meizu_adc_parse_dt(struct device *dev, struct meizu_adc__pdata *pdata)
{
	int rc = 0;

	rc |= meizu_adc_of_property_read_u32(dev, "interval_ms", &pdata->interval_ms);
	rc |= meizu_adc_of_property_read_u32(dev, "sample_count", &pdata->sample_count);
	rc |= meizu_adc_of_property_read_u32(dev, "no__mv", &pdata->no__mv);
	rc |= meizu_adc_of_property_read_u32(dev, "min_mv", &pdata->min_mv);
	rc |= meizu_adc_of_property_read_u32(dev, "max_mv", &pdata->max_mv);
	rc |= meizu_adc_of_property_read_u32(dev, "adc_per_uv", &pdata->adc_per_uv);

	return (rc ? -EINVAL : 0);
}
#endif
static int meizu_adc_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct meizu_adc_data *adc_data;
	//struct device_node *np = pdev->dev.of_node;

	pr_info(TAG"%s adc probe\n",__func__);
	adc_data = devm_kzalloc(&pdev->dev, sizeof(*adc_data), GFP_KERNEL);
	if (adc_data == NULL) {
		rc = -ENOMEM;
		goto err_alloc__data;
	}
	adc_data->pdev = pdev;
#if 0
	if (np == NULL) {
		memcpy(&_data->pdata, dev_get_platdata(&pdev->dev), sizeof(_data->pdata));
	} else {
		if ((rc = meizu_adc_parse_dt(&pdev->dev, &_data->pdata)) < 0) {
			goto err_s3c_adc_register;
		}
	}
#endif
	adc_data->batid_channel = iio_channel_get(&pdev->dev, "batid_adc");
	if (IS_ERR_OR_NULL(adc_data->batid_channel)) {
		dev_err(&pdev->dev, "cannot register adc batid: %ld\n", PTR_ERR(adc_data->batid_channel));
		rc = PTR_ERR(adc_data->batid_channel);
		goto err_get_batid;
	}
#if 1
	adc_data->acid_channel = iio_channel_get(&pdev->dev, "acid_adc");
	if (IS_ERR_OR_NULL(adc_data->acid_channel)) {
		dev_err(&pdev->dev, "cannot register adc acid: %ld\n", PTR_ERR(adc_data->acid_channel));
		rc = PTR_ERR(adc_data->acid_channel);
		goto err_get_acid;
	}
#endif
#if 1
	adc_data->thermal_channel = iio_channel_get(&pdev->dev, "thermal_adc");
	if (IS_ERR_OR_NULL(adc_data->acid_channel)) {
		dev_err(&pdev->dev, "cannot register adc acid: %ld\n", PTR_ERR(adc_data->acid_channel));
		rc = PTR_ERR(adc_data->acid_channel);
		goto err_get_acid;
	}
#endif
	platform_set_drvdata(pdev, adc_data);

	global_adc_data = adc_data;

	dev_info(&pdev->dev, "successfully loaded\n");

	return rc;
#if 1
err_get_acid:
	iio_channel_release(adc_data->batid_channel);
#endif
err_get_batid:
	devm_kfree(&pdev->dev, adc_data);
err_alloc__data:
	return rc;
}

static int meizu_adc_remove(struct platform_device *pdev)
{
	struct meizu_adc_data *adc_data = platform_get_drvdata(pdev);

	mutex_lock(&adc_lock);
	global_adc_data = NULL;

	iio_channel_release(adc_data->batid_channel);
	iio_channel_release(adc_data->acid_channel);
	iio_channel_release(adc_data->thermal_channel);
	devm_kfree(&pdev->dev, adc_data);
	mutex_unlock(&adc_lock);

	return 0;
}

#if 0
static void meizu_adc_shutdown(struct platform_device *pdev)
{
	struct meizu_adc_data *adc_data = platform_get_drvdata(pdev);

	mutex_lock(&adc_lock);
	global_adc_data = NULL;

	iio_channel_release(adc_data->batid_channel);
	iio_channel_release(adc_data->acid_channel);
	iio_channel_release(adc_data->thermal_channel);
	devm_kfree(&pdev->dev, adc_data);

	mutex_unlock(&adc_lock);
}

static int meizu_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct meizu_adc_data *_data = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "%s\n", __func__);
	cancel_delayed_work_sync(&_data->measuring_work);
	return 0;
}

static int meizu_adc_resume(struct platform_device *pdev)
{
	struct meizu_adc_data *_data = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "%s\n", __func__);
	schedule_delayed_work(&_data->measuring_work,
			msecs_to_jiffies(_data->pdata.interval_ms));
	return 0;
}
#else
#define meizu_adc__suspend NULL
#define meizu_adc__resume NULL
#endif

const struct of_device_id meizu_of_match_table[] = {
	{.compatible = "meizu,simple_adc",	},
	{},
};

static struct platform_driver meizu_adc_driver = {
	.probe	= meizu_adc_probe,
	.remove	= meizu_adc_remove,
#if 0
	.shutdown = meizu_adc_shutdown,
	.suspend = meizu_adc_suspend,
	.resume = meizu_adc_resume,
#endif
	.driver	= {
		.name = "meizu_adc_simple",
		.owner	= THIS_MODULE,
		.of_match_table = meizu_of_match_table,
	},
};

module_platform_driver(meizu_adc_driver);

MODULE_AUTHOR("Chucheng Luo <luochucheng@meizu.com>");
MODULE_DESCRIPTION("simple adc provider for meizu bat_id & ac_id");
MODULE_LICENSE("GPL");
