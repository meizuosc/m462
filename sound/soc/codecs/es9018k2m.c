/*
 * es9018k2m.c  --  ES9018K2M ALSA SoC Audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on wm8753.c by Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#endif
#include "es9018k2m.h"
#include <mach/hardware.h>

#define FIR_ENA

/*
 * es9018k2m register cache
 */
static const struct reg_default es9018k2m_reg_defaults[] = {
	{ 0, 0x00 },
	{ 1, 0x8c },
	{ 4, 0x00 },
	{ 5, 0x68 },
	{ 6, 0x42 },
	{ 7, 0x80 },
	{ 8, 0x10 },
	{ 9, 0x00 },
	{ 10,0x05 },
	{ 11,0x02 },
	{ 12,0x5a },
	{ 13,0x40 },
	{ 14,0x8a },
	{ 15,0x00 },
	{ 16,0x00 },
	{ 17,0xff },
	{ 18,0xff },
	{ 19,0xff },
	{ 20,0x7f },
	{ 21,0x00 },
	{ 26,0x00 },
	{ 27,0x00 },
	{ 28,0x00 },
	{ 29,0x00 },
	{ 30,0x00 },
	
};

#ifdef FIR_ENA
/* FIR COEFF DATA  */
static const int es9018k2m_fir_coeff_stage1[128] = {
	-49,-534,648,390,-615,-1167,1438,1609,-2064,-2691,3303,3899,-4754,-5727,6851,8010,
	-9472,-11076,12919,14922,-17208,-19829,22603,25895,-29203,-33406,37277,42543,-46992,-53633,58648,66951,
	-72490,-82908,88883,101926,-108191,-124585,130914,151571,-157631,-183836,189136,222635,-226464,-269796,
	271093,328001,-325148,-401473,391883,497181,-476456,-627565,587554,816958,-740618,-1119576,964400,1681304,
	-1306635,-3031620,1654634,8388607,8388607,1654634,-3031620,-1306635,1681304,964400,-1119576,-740618,816958,
	587554,-627565,-476456,497181,391883,-401473,-325148,328001,271093,-269796,-226464,222635,189136,-183836,
	-157631,151571,130914,-124585,-108191,101926,88883,-82908,-72490,66951,58648,-53633,-46992,42543,37277,
	-33406,-29203,25895,22603,-19829,-17208,14922,12919,-11076,-9472,8010,6851,-5727,-4754,3899,
	3303,-2691,-2064,1609,1438,-1167,-615,390,648,-534,-49

};

static const int es9018k2m_fir_coeff_stage2[16] = {
	0,0,0,7666,40221,132007,336854,704620,1267735,2007975,2833353,3605291,4156988,4355332,0,0,
};

static int es9018k2m_set_FIR_coeff(struct snd_soc_codec *codec, const int * coeff, int size)
{
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	int i;
	int address;

	if(es9018k2m->version == 0)
		return 0;
	if(size == 128 || size == 64)
		address = 0x0;
	else if(size == 16)
		address = 0x80;
	else {
		printk("%s:error FIR coeff,size=%d\n",__func__,size);
		return -EIO;
	}
	
	for(i = 0;i < size;i++) {
		snd_soc_write(codec, 26,address+i);
		snd_soc_write(codec, 27,coeff[i] & 0xff);
		snd_soc_write(codec, 28,(coeff[i] >> 8) & 0xff);
		snd_soc_write(codec, 29,(coeff[i] >> 16) & 0xff);
		snd_soc_write(codec, 30, 0x02);
	}

	//snd_soc_write(codec, 30, 0x05);//cos
	//snd_soc_write(codec, 30, 0x01);//sin
	snd_soc_write(codec, 30, 0x00);//sin

	return 0;
}
#endif

static unsigned int es9018k2m_read(struct snd_soc_codec *codec, unsigned int reg)
{
	int ret = 0;
	struct regmap *map = es9018k2m_priv->regmap;
	unsigned int  val;

	mutex_lock(&es9018k2m_priv->access_mutex);
	if(es9018k2m_priv->active) {
		ret = regmap_read(map, reg, &val);
		if(ret == 0) {
			ret = val;
		}
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&es9018k2m_priv->access_mutex);

	return ret;
}

static int es9018k2m_write(struct snd_soc_codec *codec, unsigned int reg,
		    unsigned int value)
{
	int ret;
	struct regmap *map = es9018k2m_priv->regmap;

	mutex_lock(&es9018k2m_priv->access_mutex);
	if(es9018k2m_priv->active)
		ret = regmap_write(map, reg, value);
	else
		ret = -EINVAL;
	mutex_unlock(&es9018k2m_priv->access_mutex);

	return ret;
}

static int es9018k2m_set_gain(struct snd_soc_codec *codec, int value)
{
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	struct es9018k2m_platform_data *pdata = codec->dev->platform_data;
	int ret = 0;

	mutex_lock(&es9018k2m->gain_mutex);
	if(value ==  es9018k2m->gain) {
		goto err;
	}

	if(meizu_board_version()) {
		switch (value) {
		case ES9018K2M_GAIN_CODEC:
			gpio_set_value(pdata->gpio_amplifier, 1);//mute
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x07);//GPIO1 LOW
			msleep(5);
			gpio_set_value(pdata->gpio_amplifier, 0);//unmute
			break;
		case ES9018K2M_GAIN_LOW:

			gpio_set_value(pdata->gpio_amplifier, 1);//mute

			snd_soc_update_bits(codec, ES9018K2M_GENERAL_SET, 0x03, 0x03);//ES9018 mute
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x07);//GPIO1 LOW

			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0xf0, 0xf0);//GPIO2 HIGH
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x0f);//GPIO1 HIGH

			if(es9018k2m->gain == 2) {
				msleep(50);
			}
			msleep(200);

			gpio_set_value(pdata->gpio_amplifier, 0);//unmute
			snd_soc_update_bits(codec, ES9018K2M_GENERAL_SET, 0x03, 0x00);//ES9018 unmute

			break;
		case ES9018K2M_GAIN_HIGH:

			gpio_set_value(pdata->gpio_amplifier, 1);//mute
			snd_soc_update_bits(codec, ES9018K2M_GENERAL_SET, 0x03, 0x03);//ES9018 mute
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x07);//GPIO1 LOW

			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0xf0, 0x70);//GPIO2 LOW
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x0f);//GPIO1 HIGH
			if(es9018k2m->gain == 1) {
				msleep(180);
			}

			msleep(5);
			snd_soc_update_bits(codec, ES9018K2M_GENERAL_SET, 0x03, 0x00);//ES9018 unmute
			printk("%s: pre gain=%d,curr gain=%d\n",__func__,es9018k2m->gain,value);

			gpio_set_value(pdata->gpio_amplifier, 0);//unmute

			break;
		default:
			ret = -EINVAL;
			goto err;
		}
	} else {

		switch (value) {
		case ES9018K2M_GAIN_CODEC:
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x07);//GPIO1 LOW
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0xf0, 0x70);//GPIO2 LOW

			break;
		case ES9018K2M_GAIN_LOW:
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x0f);//GPIO1 HIGH
			msleep(5);
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x07);//GPIO1 LOW
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0xf0, 0xf0);//GPIO2 HIGH
			break;
		case ES9018K2M_GAIN_HIGH:
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x0f);//GPIO1 HIGH
			msleep(5);
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0x0f, 0x0f);//GPIO1 HIGH
			snd_soc_update_bits(codec, ES9018K2M_GPIO_CONFIG, 0xf0, 0x70);//GPIO2 LOW
			break;
		default:
			ret = -EINVAL;
			goto err;
		}
	}

	es9018k2m->gain = value;
	printk("%s: gain=%d\n",__func__,es9018k2m->gain);
err:
	mutex_unlock(&es9018k2m->gain_mutex);
	return ret;
}


static int es9018k2m_poweron(struct snd_soc_codec *codec, int enable)
{
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	struct es9018k2m_platform_data *pdata = codec->dev->platform_data;
	int ret = 0;

	printk("es9018k2m %s:%d %s\n", __func__, __LINE__, enable > 0 ? "on" :"off");

	if(enable) {

		gpio_set_value(pdata->gpio_amplifier, 1);//mute
		/*enable the VCCA AVCC_L and AVCC_R for es9018*/
		gpio_set_value(pdata->gpio_vol_ena, 1);

		ret = regulator_enable(es9018k2m->vdd18_dac);
		if (ret != 0) {
			pr_err("%s : fail to enable vdd18_dav\n",__func__);
			return ret;
		}

		printk("es9018k2m %s :%d on \n", __func__, __LINE__);
		/*Enable the voltage for opa1662*/
		gpio_set_value(pdata->gpio_op_en, 1);

		usleep_range(1000, 1500);
	} else {

		printk("es9018k2m %s:%d off \n",__func__,__LINE__);

		gpio_set_value(pdata->gpio_amplifier, 1);//mute
		es9018k2m_set_gain(codec,ES9018K2M_GAIN_CODEC);

		/*disable amplifier */
		gpio_set_value(pdata->gpio_amplifier, 0);//unmute
		/* set soft_start low while in standby mode */
		if(pdata->gpio_osc_44khz != 0 || pdata->gpio_osc_48khz != 0)
			snd_soc_update_bits(codec, ES9018K2M_SOFT_START, 0x80, 0x00);
		/* Disable RESETB */
		gpio_set_value(pdata->gpio_resetb, 0);
		/*gate the osc output */
		gpio_set_value(pdata->gpio_osc_44khz, 0);
		gpio_set_value(pdata->gpio_osc_48khz, 0);

		mutex_lock(&es9018k2m->access_mutex);
		es9018k2m->active = 0;
		mutex_unlock(&es9018k2m->access_mutex);

		/*disable the voltage for opa1662*/
		gpio_set_value(pdata->gpio_op_en, 0);
		/*disable the voltage for AVCC*/
		gpio_set_value(pdata->gpio_vol_ena, 0);

		if (regulator_is_enabled(es9018k2m->vdd18_dac))
			regulator_disable(es9018k2m->vdd18_dac);

		gpio_set_value(pdata->gpio_amplifier, 0);//unmute

		regcache_mark_dirty(es9018k2m->regmap);
	}

	return 0;
}

struct es9018k2m_priv *es9018k2m_priv = NULL;

static BLOCKING_NOTIFIER_HEAD(es9018k2m_hp_notifier_list);

static int es9018k2m_headphone_detect(struct notifier_block *self,
							unsigned long action, void *data)
{
	if(action == 1) {
		//selection wm8998 hp;
		printk("es9018 plugin out headphone %s:\n",__func__);
		es9018k2m_set_gain(es9018k2m_priv->codec,ES9018K2M_GAIN_CODEC);
		return NOTIFY_OK;
	} else {
		return NOTIFY_DONE;
	}
}


static struct notifier_block es9018k2m_headphone_cb = {
	.notifier_call = es9018k2m_headphone_detect,
};

void es9018k2m_register_notify(struct blocking_notifier_head *list,
		struct notifier_block *nb)
{
	blocking_notifier_chain_register(list, nb);
}

void es9018k2m_unregister_notify(struct blocking_notifier_head *list,
		struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(list,nb);
}

void es9018k2m_blocking_notifier_call_chain(void )
{
	blocking_notifier_call_chain(es9018k2m_priv->es9018k2m_notifier_list,
				1, es9018k2m_priv);
}

EXPORT_SYMBOL_GPL(es9018k2m_blocking_notifier_call_chain);

static int es9018k2m_deemph[] = { 32000, 44100, 48000 };//CONFIG_LINF

static int es9018k2m_set_deemph(struct snd_soc_codec *codec)
{
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	int val, i, best;

	/* If we're using deemphasis select the nearest available sample
	 * rate.
	 */
	if (es9018k2m->deemph) {
		best = 1;
		for (i = 2; i < ARRAY_SIZE(es9018k2m_deemph); i++) {
			if (abs(es9018k2m_deemph[i] - es9018k2m->playback_fs) <
			    abs(es9018k2m_deemph[best] - es9018k2m->playback_fs))
				best = i;
		}

		val = (best << 4) & 0xbf;
	} else {
		best = 0;
		val = 0;
		val |= 0x40;
		
	}

	dev_dbg(codec->dev, "Set deemphasis %d (%dHz),val=0x%x\n",
		best, es9018k2m_deemph[best],val);

	return snd_soc_update_bits(codec, ES9018K2M_DEEMPHASIS, 0x70, val);
}

static int es9018k2m_get_deemph(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.enumerated.item[0] = es9018k2m->deemph;

	return 0;
}

static int es9018k2m_put_deemph(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	int deemph = ucontrol->value.enumerated.item[0];
	int ret = 0;

	if (deemph > 1)
		return -EINVAL;

	mutex_lock(&codec->mutex);
	if (es9018k2m->deemph != deemph) {
		es9018k2m->deemph = deemph;

		es9018k2m_set_deemph(codec);

		ret = 1;
	}
	mutex_unlock(&codec->mutex);

	return ret;
}

static const DECLARE_TLV_DB_SCALE(out_tlv, -12750, 50, 1);

static const char *DPLL_BW_txt[] = {
	"No Bandwidth0", "Lowest Bandwidth0","Low Bandwidth0","Med-low Bandwidth0",
	"Medium Bandwidth0","Med-High Bandwidth0","High Bandwidth0","Hihgest Bandwidth0",
	"No Bandwidth1", "Lowest Bandwidth1","Low Bandwidth1","Med-low Bandwidth1",
	"Medium Bandwidth1","Med-High Bandwidth1","High Bandwidth1","Hihgest Bandwidth1",
	};
static const struct soc_enum DPLL_BW =
	SOC_ENUM_SINGLE(ES9018K2M_DPLL, 4, 15, DPLL_BW_txt);

static const char *filter_shape_txt[] = {
	"fast rolloff", "slow rolloff","minimum phase","reserved",
	};
static const struct soc_enum  filter_shape =
	SOC_ENUM_SINGLE(ES9018K2M_GENERAL_SET, 5, 3, filter_shape_txt);

static const char * const es9018k2m_gain_texts[] = {
	"codec hp", "Hifi low gain", "Hifi high gain",
};
static const struct soc_enum es9018k2m_gain_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018k2m_gain_texts),
			es9018k2m_gain_texts);

static int es9018k2m_get_gain_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	ucontrol->value.enumerated.item[0] = es9018k2m->gain;

	return 0;
}

static int es9018k2m_put_gain_enum(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int value;
	int ret = 0;

	value = ucontrol->value.enumerated.item[0];
	es9018k2m_set_gain(codec, value);
	return ret;
}

#ifdef FIR_ENA

static const char * const es9018k2m_custom_fir_texts[] = {
	"Disable", "Enable",
};
static const struct soc_enum es9018k2m_custom_fir_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018k2m_custom_fir_texts),
			es9018k2m_custom_fir_texts);

static int es9018k2m_get_custom_fir_enum(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	ucontrol->value.enumerated.item[0] = es9018k2m->custom_fir_enable;

	return 0;
}

static int es9018k2m_put_custom_fir_enum(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	unsigned int value;
	int ret = 0;

	value = ucontrol->value.enumerated.item[0];
	/*if(value == es9018k2m->custom_fir_enable)
		return 0;*/
	if (value) {
		es9018k2m_set_FIR_coeff(codec,es9018k2m_fir_coeff_stage1,128);
		es9018k2m_set_FIR_coeff(codec,es9018k2m_fir_coeff_stage2,16);
		snd_soc_write(codec,30,0x1);
	} else {
		snd_soc_write(codec,30,0x0);
	}

	es9018k2m->custom_fir_enable = value;
	return ret;
}
#endif

static const struct snd_kcontrol_new es9018k2m_snd_controls[] = {

SOC_DOUBLE_R_TLV("Master Playback Volume", ES9018K2M_VOLUME1, ES9018K2M_VOLUME2,
		 0, 255, 0, out_tlv),
SOC_ENUM("Filter Shape", filter_shape),
SOC_SINGLE("Playback Left mute", ES9018K2M_GENERAL_SET, 0, 1, 0),
SOC_SINGLE("Playback Right mute", ES9018K2M_GENERAL_SET, 1, 1, 0),
SOC_SINGLE("bypass IIR", ES9018K2M_INPUT_SELECT, 2, 1, 0),
SOC_SINGLE("Bypass FIR", ES9018K2M_INPUT_SELECT, 0, 1, 0),
SOC_ENUM("DPLL Bandwidth", DPLL_BW),

SOC_SINGLE("THD Compensation", ES9018K2M_THD_COMPENSATION, 6, 1, 1),
SOC_SINGLE("2nd Harmonic Compensation", ES9018K2M_2_HARMONIC_COMPENSATION_1, 0, 255, 0),
SOC_SINGLE("3nd Harmonic Compensation", ES9018K2M_3_HARMONIC_COMPENSATION_1, 0, 255, 0),

SOC_SINGLE_BOOL_EXT("Playback Deemphasis Switch", 0,
		    es9018k2m_get_deemph, es9018k2m_put_deemph),
SOC_ENUM_EXT("Gain selection",
				 es9018k2m_gain_enum,
				 es9018k2m_get_gain_enum, es9018k2m_put_gain_enum),
#ifdef FIR_ENA
SOC_ENUM_EXT("custom FIR enable",
				 es9018k2m_custom_fir_enum,
				 es9018k2m_get_custom_fir_enum, es9018k2m_put_custom_fir_enum),

#endif
};

static int es9018k2m_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	//struct es9018k2m_platform_data *pdata = codec->dev->platform_data;

	int bick_div = es9018k2m->sysclk / params_rate(params)/64;
	u8 bick = 0;
	u8 iface = snd_soc_read(codec, ES9018K2M_INPUT_CONFIG) & 0x3f;

	pr_debug("ess9018: %s ########start\n",__func__);

	es9018k2m->playback_fs = params_rate(params);

	if(es9018k2m->master) {
		switch(bick_div) {
		case 16:
			bick = 0x40;
			break;
		case 8:
			bick = 0x20;
			break;
		case 4:
			bick = 0x00;
			break;
		default:
			return -EINVAL;
		}

		iface |= 0x80;

		if (es9018k2m->version)
			snd_soc_update_bits(codec, ES9018K2M_V_MODE_CONTROL, 0x60, bick);
		else
			snd_soc_update_bits(codec, ES9018K2M_W_MODE_CONTROL, 0x60, bick);

		pr_debug("ess9018: %s reg=0x%x value=0x%x,0x%x #####end\n",__func__,ES9018K2M_V_MODE_CONTROL,bick,snd_soc_read(codec, ES9018K2M_V_MODE_CONTROL));
		pr_debug("ess9018: %s reg=0x%x value=0x%x,0x%x #####end\n",__func__,ES9018K2M_W_MODE_CONTROL,bick,snd_soc_read(codec, ES9018K2M_W_MODE_CONTROL));

	} else {
		/* bit size */
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			iface |= 0x0;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			iface |= 0x40;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			iface |= 0x80;
			break;
		default:
			return -EINVAL;
		}
	}
	//es9018k2m_set_deemph(codec);

	snd_soc_write(codec, ES9018K2M_INPUT_CONFIG, iface);
	pr_debug("ess9018: %s reg=0x%x value=0x%x,0x%x #####end\n",__func__,ES9018K2M_INPUT_CONFIG,iface,snd_soc_read(codec, ES9018K2M_INPUT_CONFIG));

	/*set soft_start high */
	snd_soc_update_bits(codec, ES9018K2M_SOFT_START, 0x80, 0x80);
	return 0;
}

static int es9018k2m_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 mute_reg = snd_soc_read(codec, ES9018K2M_GENERAL_SET) & 0xfc;

	if (mute)
		snd_soc_write(codec, ES9018K2M_GENERAL_SET, mute_reg | 0x3);
	else
		snd_soc_write(codec, ES9018K2M_GENERAL_SET, mute_reg);
	
	return 0;
}

static int es9018k2m_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es9018k2m_platform_data *pdata = codec->dev->platform_data;
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	int version = 0;

	pr_debug("ess9018: %s ########start\n",__func__);

	switch (clk_id) {
	case ES9018K2M_SYSCLK_MCLK:
		break;
	default:
		return -EINVAL;
	}

	switch (freq) {
	case 49152000:
		/* Enable the corresponding clock */
		gpio_set_value(pdata->gpio_osc_48khz,1);
		gpio_set_value(pdata->gpio_osc_44khz,0);
		es9018k2m->sysclk = freq;
		break;	
	case 45158400:
		gpio_set_value(pdata->gpio_osc_44khz,1);
		gpio_set_value(pdata->gpio_osc_48khz,0);
		es9018k2m->sysclk = freq;
		break;
	default:
		printk("%s: errot system clock %d\n",__func__,freq);
		return -EINVAL;
	}
	usleep_range(1000, 1500);
	
	gpio_set_value(pdata->gpio_resetb, 1);

	mutex_lock(&es9018k2m->access_mutex);
	es9018k2m->active = 1;
	mutex_unlock(&es9018k2m->access_mutex);
	
	usleep_range(1000, 1500);
	
	pr_debug("es9018k2m reg=0x%d,value=0x%x\n",64,snd_soc_read(codec,64));
	
	version = (snd_soc_read(codec,64)>>2) & 0x0f;
	
	if(version == 0x4)
		es9018k2m->version = 0;//w version
	else if (version == 0xc)	
		es9018k2m->version = 1;//V version
	else {
		printk("es9018k2m can't support this version error\n");
		return -ENODEV;
	}
	regcache_sync(es9018k2m->regmap);
	
	pr_debug("es9018k2m %s :version:%d\n",__func__,es9018k2m->version);
	return 0;
}


static int es9018k2m_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	u8 iface = 0;
	u8 format = 0;

	pr_debug("ess9018: %s ########start\n",__func__);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x80;
		es9018k2m->master = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface |= 0x00;
		es9018k2m->master = 0;
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		format &= ~0x30;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		format |= 0x10;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
	case SND_SOC_DAIFMT_IB_NF:
	case SND_SOC_DAIFMT_NB_IF:
	default:
		return -EINVAL;
	}

	/* set iface */
	if (es9018k2m->version)
		snd_soc_write(codec, ES9018K2M_V_MODE_CONTROL, iface);
	else
		snd_soc_write(codec, ES9018K2M_W_MODE_CONTROL, iface);
	
	snd_soc_write(codec, ES9018K2M_INPUT_CONFIG, format);

	pr_debug("ess9018: %s ########end\n",__func__);

	return 0;
}

static int es9018k2m_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	es9018k2m_poweron(codec, 1);

	return 0;
}
static void es9018k2m_shutdown(struct snd_pcm_substream *substream, struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	es9018k2m_poweron(codec, 0);
}


#define ES9018K2M_RATES (SNDRV_PCM_RATE_8000_44100 | SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |SNDRV_PCM_RATE_96000|\
					 SNDRV_PCM_RATE_176400| SNDRV_PCM_RATE_192000)

#define ES9018K2M_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE|\
	SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_dai_ops es9018k2m_dai_ops = {
	.hw_params	= es9018k2m_hw_params,
	.digital_mute	= es9018k2m_mute,
	.set_sysclk	= es9018k2m_set_dai_sysclk,
	.set_fmt	= es9018k2m_set_dai_fmt,
	.startup = es9018k2m_startup,
	.shutdown = es9018k2m_shutdown,
};

static struct snd_soc_dai_driver es9018k2m_dai = {
	.name = "ess9018k2m-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = ES9018K2M_RATES,
		.formats = ES9018K2M_FORMATS,},
	.ops = &es9018k2m_dai_ops,
	.symmetric_rates = 1,
};

#ifdef CONFIG_PM
static int es9018k2m_suspend(struct snd_soc_codec *codec)
{

	return 0;
}

static int es9018k2m_resume(struct snd_soc_codec *codec)
{
	struct es9018k2m_platform_data *pdata = codec->dev->platform_data;

	gpio_set_value(pdata->gpio_amplifier, 0);//for wm8998 headphone output louder

	return 0;
}
#else
#define es9018k2m_suspend NULL
#define es9018k2m_resume NULL
#endif

#ifdef CONFIG_OF
static struct es9018k2m_platform_data *ess9018k2m_parse_dt(struct device *dev)
{
	struct es9018k2m_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int gpio;

	if (!np)
		return ERR_PTR(-ENOENT);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "ess9018k2m failed to allocate platform data\n");
		return ERR_PTR(-ENOMEM);
	}
	dev->platform_data = pdata;

	gpio = of_get_gpio(np, 0);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "ess9018k2m failed to get gpio_osc_48khz t gpio\n");
		return ERR_PTR(-EINVAL);
	}
	pdata->gpio_osc_48khz = gpio;

	gpio = of_get_gpio(np, 1);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "ess9018k2m failed to get gpio_osc_44khz gpio\n");
		return ERR_PTR(-EINVAL);
	}
	pdata->gpio_osc_44khz = gpio;

	gpio = of_get_gpio(np, 2);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "ess9018k2m failed to get gpio_resetb gpio\n");
		return ERR_PTR(-EINVAL);
	}
	pdata->gpio_resetb = gpio;

	gpio = of_get_gpio(np, 3);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "ess9018k2m failed to get gpio_amplifier gpio\n");
		return ERR_PTR(-EINVAL);
	}
	pdata->gpio_amplifier = gpio;

	gpio = of_get_gpio(np, 4);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "ess9018k2m failed to get gpio_vol_ena gpio\n");
		return ERR_PTR(-EINVAL);
	}
	pdata->gpio_vol_ena = gpio;

	gpio = of_get_gpio(np, 5);
	if (!gpio_is_valid(gpio)) {
		dev_err(dev, "ess9018k2m failed to get gpio_voltage gpio\n");
		return ERR_PTR(-EINVAL);
	}
	pdata->gpio_op_en= gpio;

	dev->platform_data = pdata;

	return pdata;
}
#else
static struct es9018k2m_platform_data *ess9018k2m_parse_dt(struct device *dev)
{
	struct es9018k2m_platform_data *pdata = dev->platform_data;

	return pdata;
}
#endif

static int es9018k2m_probe(struct snd_soc_codec *codec)
{
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	struct es9018k2m_platform_data *pdata = ess9018k2m_parse_dt(codec->dev);
	int ret = 0;

	if (!pdata)
		return -EINVAL;

	if (pdata) {
		if (gpio_is_valid(pdata->gpio_osc_48khz)) {
			ret = gpio_request_one(pdata->gpio_osc_48khz,
					GPIOF_OUT_INIT_LOW, "es9018k2m osc 48khz");
			if (ret)
				goto err_out;
		}

		if (gpio_is_valid(pdata->gpio_osc_44khz)) {
			ret = gpio_request_one(pdata->gpio_osc_44khz,
					GPIOF_OUT_INIT_LOW, "es9018k2m osc 44.1khz");
			if (ret)
				goto err_gpio_osc_44khz;
		}

		if (gpio_is_valid(pdata->gpio_resetb)) {
			ret = gpio_request_one(pdata->gpio_resetb,
					GPIOF_OUT_INIT_LOW, "es9018k2m resetb");
			if (ret)
				goto err_gpio_osc_resetb;
		}

		if (gpio_is_valid(pdata->gpio_amplifier)) {
			ret = gpio_request_one(pdata->gpio_amplifier,
					GPIOF_OUT_INIT_LOW, "es9018k2m mute");//GPIOF_OUT_INIT_HIGH for wm8998 hP louder
			if (ret)
				goto err_gpio_amplifier;
		}

		if (gpio_is_valid(pdata->gpio_vol_ena)) {
			ret = gpio_request_one(pdata->gpio_vol_ena,
					GPIOF_OUT_INIT_LOW, "es9018k2m voltage enable");
			if (ret)
				goto err_gpio_vol;
		}

		if (gpio_is_valid(pdata->gpio_op_en)) {
			ret = gpio_request_one(pdata->gpio_op_en,
					GPIOF_OUT_INIT_LOW, "es9018k2m pa voltage");
			if (ret)
				goto err_gpio_op_en;
		}
		
	}

	codec->control_data = es9018k2m->regmap;
	codec->read = es9018k2m_read;
	codec->write = es9018k2m_write;

	es9018k2m->vdd18_dac = regulator_get(codec->dev, "vdd18_dac");
	if (IS_ERR(es9018k2m->vdd18_dac)) {
		dev_err(codec->dev, "Failed to request vdd18_dac: %d\n", ret);
		goto err_gpio;
	}

	es9018k2m->codec = codec;

	printk("es9018k2m %s############# success\n",__func__);

	return 0;
	
err_gpio:
	if (gpio_is_valid(pdata->gpio_op_en))
			gpio_free(pdata->gpio_op_en);
err_gpio_op_en:
	if (gpio_is_valid(pdata->gpio_vol_ena))
		gpio_free(pdata->gpio_vol_ena);
err_gpio_vol:	
	if (gpio_is_valid(pdata->gpio_amplifier))
		gpio_free(pdata->gpio_amplifier);
err_gpio_amplifier:
	if (gpio_is_valid(pdata->gpio_resetb))
		gpio_free(pdata->gpio_resetb);
err_gpio_osc_resetb:
	if (gpio_is_valid(pdata->gpio_osc_44khz))
		gpio_free(pdata->gpio_osc_44khz);
err_gpio_osc_44khz:
	if (gpio_is_valid(pdata->gpio_osc_48khz))
		gpio_free(pdata->gpio_osc_48khz);
err_out:	

	return ret;
}

static int es9018k2m_remove(struct snd_soc_codec *codec)
{
	struct es9018k2m_priv *es9018k2m = snd_soc_codec_get_drvdata(codec);
	struct es9018k2m_platform_data *pdata = codec->dev->platform_data;

	es9018k2m_poweron(codec, 0);

	if (regulator_is_enabled(es9018k2m->vdd18_dac))
		regulator_disable(es9018k2m->vdd18_dac);
	regulator_put(es9018k2m->vdd18_dac);

	gpio_free(pdata->gpio_amplifier);
	gpio_free(pdata->gpio_resetb);
	gpio_free(pdata->gpio_osc_44khz);
	gpio_free(pdata->gpio_osc_48khz);
	gpio_free(pdata->gpio_vol_ena);
	gpio_free(pdata->gpio_op_en);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es9018k2m = {
	.probe =	es9018k2m_probe,
	.remove =	es9018k2m_remove,
	.suspend =	es9018k2m_suspend,
	.resume =	es9018k2m_resume,
	.controls =	es9018k2m_snd_controls,
	.num_controls = ARRAY_SIZE(es9018k2m_snd_controls),
};

static bool es9018k2m_readable(struct device *dev, unsigned int reg)
{
	if(reg <= ES9018K2M_CACHEREGNUM && reg != 2 && reg !=3)
		return 1;
	else if(65 <= reg && reg <= 69)
		return 1;
	else if(70 <= reg && reg <= 93)
		return 1;
	else
		return 0;
}

static bool es9018k2m_writeable(struct device *dev, unsigned int reg)
{
	if(reg > ES9018K2M_CACHEREGNUM)
		return  0;
	else if(reg == 0x2 || reg == 0x3)
		return 0;
	else
		return 1;
}

static const struct regmap_config es9018k2m_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 93,
	.readable_reg = es9018k2m_readable,
	.writeable_reg = es9018k2m_writeable,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = es9018k2m_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es9018k2m_reg_defaults),
};

static int es9018k2m_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct es9018k2m_priv *es9018k2m;
	int ret;

	pr_info("es9018k2m %s############# start\n",__func__);
	es9018k2m_priv = NULL;

	es9018k2m = kzalloc(sizeof(struct es9018k2m_priv), GFP_KERNEL);
	if (es9018k2m == NULL)
		return -ENOMEM;

	es9018k2m->regmap = regmap_init_i2c(i2c, &es9018k2m_regmap);
	if (IS_ERR(es9018k2m->regmap)) {
		ret = PTR_ERR(es9018k2m->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		goto err;
	}

	i2c_set_clientdata(i2c, es9018k2m);

	ret = snd_soc_register_codec(&i2c->dev,
			&soc_codec_dev_es9018k2m, &es9018k2m_dai, 1);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to register CODEC: %d\n", ret);
		goto err_regmap;
	}
	mutex_init(&es9018k2m->access_mutex);
	mutex_init(&es9018k2m->gain_mutex);
	es9018k2m->active = 0;
	es9018k2m->es9018k2m_notifier_list = &es9018k2m_hp_notifier_list;
	es9018k2m_priv = es9018k2m;
	es9018k2m_register_notify(es9018k2m_priv->es9018k2m_notifier_list,&es9018k2m_headphone_cb);

	pr_info("es9018k2m %s############# success\n",__func__);
	
	return 0;

err_regmap:
	regmap_exit(es9018k2m->regmap);
err:
	kfree(es9018k2m);
	return ret;
}

static int es9018k2m_i2c_remove(struct i2c_client *client)
{
	struct es9018k2m_priv *es9018k2m = i2c_get_clientdata(client);
	snd_soc_unregister_codec(&client->dev);
	regmap_exit(es9018k2m->regmap);
	es9018k2m_unregister_notify(es9018k2m_priv->es9018k2m_notifier_list,&es9018k2m_headphone_cb);
	es9018k2m_priv = NULL;
	kfree(es9018k2m);
	return 0;
}

static const struct i2c_device_id es9018k2m_i2c_id[] = {
	{ "ess9018k2m", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es9018k2m_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id es9018k2m_of_match[] = {
	{ .compatible = "ess,ess9018k2m", },
	{ }
};

MODULE_DEVICE_TABLE(of, es9018k2m_of_match);
#endif

static struct i2c_driver es9018k2m_i2c_driver = {
	.driver = {
		.name = "ess9018k2m",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF		
		.of_match_table = es9018k2m_of_match,
#endif		
	},
	.probe =    es9018k2m_i2c_probe,
	.remove =   es9018k2m_i2c_remove,
	.id_table = es9018k2m_i2c_id,
};

static int __init es9018k2m_modinit(void)
{
	int ret = 0;
	ret = i2c_add_driver(&es9018k2m_i2c_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register ES9018K2M I2C driver: %d\n", ret);
	}
	return ret;
}
module_init(es9018k2m_modinit);

static void __exit es9018k2m_exit(void)
{
	i2c_del_driver(&es9018k2m_i2c_driver);
}
module_exit(es9018k2m_exit);

MODULE_DESCRIPTION("ASoC ES9018K2M driver");
MODULE_AUTHOR("linfeng");
MODULE_LICENSE("GPL");

