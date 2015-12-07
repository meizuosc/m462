/*
 * exynos_thermal.c - Samsung EXYNOS TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2011 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *  Amit Daniel Kachhap <amit.kachhap@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ipa.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/platform_data/exynos_thermal.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/cpu_cooling.h>
#include <linux/qos_cooling.h>
#include <linux/kmsg_dump.h>
#include <linux/reboot.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <plat/cpu.h>
#include <mach/tmu.h>
#include <mach/cpufreq.h>
#include <mach/asv-exynos.h>
#include <mach/exynos-pm.h>
#include "cal_tmu.h"
#include "thermal_core.h"

static struct thermal_sensor_conf exynos_sensor_conf;
extern int meizu_ntc_read(void);
extern int __meizu_tmu_read(int ntc_temp, int cpu_temp, int cpu_trip_val);

#if defined(CONFIG_SOC_EXYNOS5433)
#include <mach/asv-exynos5_cal.h>
#endif

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
static struct cpumask mp_cluster_cpus[CA_END];
#endif

#define CA7_POLICY_CORE		((exynos_boot_cluster == CA7) ? 0 : 4)
#define CA15_POLICY_CORE 	((exynos_boot_cluster == CA15) ? 0 : 4)
#define CS_POLICY_CORE		0

#if defined(CONFIG_SOC_EXYNOS5430)
#define CPU_HOTPLUG_IN_TEMP	95
#define CPU_HOTPLUG_OUT_TEMP	105
#elif defined(CONFIG_SOC_EXYNOS5422)
#define CPU_HOTPLUG_IN_TEMP	95
#define CPU_HOTPLUG_OUT_TEMP	100
#endif

#ifdef CONFIG_EXYNOS_SWTRIP
#define SWTRIP_TEMP				110
#define SWTRIP_NOISE_COUNT		1

static unsigned int swtrip_counter = 0;
#endif

int g_power_mode = 2;

static bool is_tmu_probed;

extern int gpu_is_power_on(void);
static enum tmu_noti_state_t tmu_old_state = TMU_NORMAL;
static enum gpu_noti_state_t gpu_old_state = GPU_NORMAL;
static enum mif_noti_state_t mif_old_state = MIF_TH_LV1;
static bool is_suspending;
static bool is_cpu_hotplugged_out;

static BLOCKING_NOTIFIER_HEAD(exynos_tmu_notifier);
static BLOCKING_NOTIFIER_HEAD(exynos_gpu_notifier);

struct exynos_tmu_data {
	struct exynos_tmu_platform_data *pdata;
	struct resource *mem[EXYNOS_TMU_COUNT];
	void __iomem *base[EXYNOS_TMU_COUNT];
	int irq[EXYNOS_TMU_COUNT];
	enum soc_type soc;
	struct work_struct irq_work;
	struct mutex lock;
	struct cal_tmu_data *cal_data;
};

struct	thermal_trip_point_conf {
	int trip_val[MAX_TRIP_COUNT];
	int trip_count;
	u8 trigger_falling;
};

struct	thermal_cooling_conf {
	struct freq_clip_table freq_data[MAX_TRIP_COUNT];
	int size[THERMAL_TRIP_CRITICAL + 1];
	int freq_clip_count;
};

struct thermal_sensor_conf {
	char name[SENSOR_NAME_LEN];
	int (*read_temperature)(void *data);
	int (*write_emul_temp)(void *drv_data, unsigned long temp);
	struct thermal_trip_point_conf trip_data;
	struct thermal_cooling_conf cooling_data;
	void *private_data;
};

struct exynos_thermal_zone {
	enum thermal_device_mode mode;
	struct thermal_zone_device *therm_dev;
	struct thermal_cooling_device *cool_dev[MAX_COOLING_DEVICE];
	unsigned int cool_dev_size;
	struct platform_device *exynos4_dev;
	struct thermal_sensor_conf *sensor_conf;
	bool bind;
};

static struct exynos_thermal_zone *th_zone;
static struct platform_device *exynos_tmu_pdev;
static struct exynos_tmu_data *tmudata;
static void exynos_unregister_thermal(void);
static int exynos_register_thermal(struct thermal_sensor_conf *sensor_conf);
static int exynos5_tmu_cpufreq_notifier(struct notifier_block *notifier, unsigned long event, void *v);

static struct notifier_block exynos_cpufreq_nb = {
	.notifier_call = exynos5_tmu_cpufreq_notifier,
};

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
static void __init init_mp_cpumask_set(void)
{
	 unsigned int i;

	 for_each_cpu(i, cpu_possible_mask) {
		 if (exynos_boot_cluster == CA7) {
			 if (i >= NR_CA7)
				 cpumask_set_cpu(i, &mp_cluster_cpus[CA15]);
			 else
				 cpumask_set_cpu(i, &mp_cluster_cpus[CA7]);
		 } else {
			 if (i >= NR_CA15)
				 cpumask_set_cpu(i, &mp_cluster_cpus[CA7]);
			 else
				 cpumask_set_cpu(i, &mp_cluster_cpus[CA15]);
		 }
	 }
}
#endif

/* Get mode callback functions for thermal zone */
static int exynos_get_mode(struct thermal_zone_device *thermal,
			enum thermal_device_mode *mode)
{
	if (th_zone)
		*mode = th_zone->mode;
	return 0;
}

/* Set mode callback functions for thermal zone */
static int exynos_set_mode(struct thermal_zone_device *thermal,
			enum thermal_device_mode mode)
{
	if (!th_zone->therm_dev) {
		pr_notice("thermal zone not registered\n");
		return 0;
	}

	th_zone->mode = mode;
	thermal_zone_device_update(th_zone->therm_dev);
	return 0;
}


/* Get trip type callback functions for thermal zone */
static int exynos_get_trip_type(struct thermal_zone_device *thermal, int trip,
				 enum thermal_trip_type *type)
{
	int active_size, passive_size;

	active_size = th_zone->sensor_conf->cooling_data.size[THERMAL_TRIP_ACTIVE];
	passive_size = th_zone->sensor_conf->cooling_data.size[THERMAL_TRIP_PASSIVE];

	if (trip < active_size)
		*type = THERMAL_TRIP_ACTIVE;
	else if (trip >= active_size && trip < active_size + passive_size)
		*type = THERMAL_TRIP_PASSIVE;
	else if (trip >= active_size + passive_size)
		*type = THERMAL_TRIP_CRITICAL;
	else
		return -EINVAL;

	return 0;
}

/* Get trip temperature callback functions for thermal zone */
static int exynos_get_trip_temp(struct thermal_zone_device *thermal, int trip,
				unsigned long *temp)
{
	int active_size, passive_size;

	active_size = th_zone->sensor_conf->cooling_data.size[THERMAL_TRIP_ACTIVE];
	passive_size = th_zone->sensor_conf->cooling_data.size[THERMAL_TRIP_PASSIVE];

	if (trip < 0 || trip > active_size + passive_size)
		return -EINVAL;
	*temp = th_zone->sensor_conf->trip_data.trip_val[trip];
	/* convert the temperature into millicelsius */
	*temp = *temp * MCELSIUS;

	return 0;
}

/* Get critical temperature callback functions for thermal zone */
static int exynos_get_crit_temp(struct thermal_zone_device *thermal,
				unsigned long *temp)
{
	int ret;
	int active_size, passive_size;

	active_size = th_zone->sensor_conf->cooling_data.size[THERMAL_TRIP_ACTIVE];
	passive_size = th_zone->sensor_conf->cooling_data.size[THERMAL_TRIP_PASSIVE];

	/* Panic zone */
	ret = exynos_get_trip_temp(thermal, active_size + passive_size, temp);
	return ret;
}

/* Bind callback functions for thermal zone */
static int exynos_bind(struct thermal_zone_device *thermal,
			struct thermal_cooling_device *cdev)
{
        unsigned long level = 0;
	int ret = 0, i;
	enum thermal_trip_type type;

	/* find the cooling device registered*/
	for (i = 0; i < th_zone->cool_dev_size; i++)
		if (cdev == th_zone->cool_dev[i]) {
			break;
		}

	/* No matching cooling device */
	if (i == th_zone->cool_dev_size)
		return 0;

        cdev->ops->get_max_state(cdev, &level);

        exynos_get_trip_type(th_zone->therm_dev, 0, &type);
        switch (type) {
        case THERMAL_TRIP_ACTIVE:
        case THERMAL_TRIP_PASSIVE:
        case THERMAL_TRIP_CRITICAL:
                if (thermal_zone_bind_cooling_device(thermal, i, cdev,
                                                        level, 0)) {
                        pr_err("error binding cdev inst %d\n", i);
                        thermal->cooling_dev_en = false;
                        ret = -EINVAL;
                }
                th_zone->bind = true;
                break;
        default:
                ret = -EINVAL;
        }

	return ret;
}

/* Unbind callback functions for thermal zone */
static int exynos_unbind(struct thermal_zone_device *thermal,
			struct thermal_cooling_device *cdev)
{
	int ret = 0, i, tab_size;
	struct thermal_sensor_conf *data = th_zone->sensor_conf;
	enum thermal_trip_type type = 0;

	if (th_zone->bind == false)
		return 0;

	tab_size = data->cooling_data.freq_clip_count;

	if (tab_size == 0)
		return -EINVAL;

	/* find the cooling device registered*/
	for (i = 0; i < th_zone->cool_dev_size; i++)
		if (cdev == th_zone->cool_dev[i])
			break;

	/* No matching cooling device */
	if (i == th_zone->cool_dev_size)
		return 0;

	/* Bind the thermal zone to the cpufreq cooling device */
	for (i = 0; i < tab_size; i++) {
		exynos_get_trip_type(th_zone->therm_dev, i, &type);
		switch (type) {
		case THERMAL_TRIP_ACTIVE:
		case THERMAL_TRIP_PASSIVE:
		case THERMAL_TRIP_CRITICAL:
			if (thermal_zone_unbind_cooling_device(thermal, i,
								cdev)) {
				pr_err("error unbinding cdev inst=%d\n", i);
				ret = -EINVAL;
			}
			th_zone->bind = false;
			break;
		default:
			ret = -EINVAL;
		}
	}
	return ret;
}


int exynos_tmu_add_notifier(struct notifier_block *n)
{
	return blocking_notifier_chain_register(&exynos_tmu_notifier, n);
}

void exynos_tmu_call_notifier(enum tmu_noti_state_t cur_state, int temp)
{
	if (is_suspending)
		cur_state = TMU_COLD;

	if (cur_state != tmu_old_state) {
		if ((cur_state == TMU_COLD) ||
			((cur_state == TMU_NORMAL) && (tmu_old_state == TMU_COLD)))
			blocking_notifier_call_chain(&exynos_tmu_notifier, TMU_COLD, &cur_state);
		else
			blocking_notifier_call_chain(&exynos_tmu_notifier, cur_state, &tmu_old_state);
		if (cur_state == TMU_COLD)
			pr_info("tmu temperature state %d to %d\n", tmu_old_state, cur_state);
		else
			pr_info("tmu temperature state %d to %d, cur_temp : %d\n", tmu_old_state, cur_state, temp);
		tmu_old_state = cur_state;
	}
}

int exynos_gpu_add_notifier(struct notifier_block *n)
{
	return blocking_notifier_chain_register(&exynos_gpu_notifier, n);
}

void exynos_gpu_call_notifier(enum gpu_noti_state_t cur_state)
{
	if (is_suspending)
		cur_state = GPU_COLD;

	if (cur_state != gpu_old_state) {
		pr_info("gpu temperature state %d to %d\n", gpu_old_state, cur_state);
		blocking_notifier_call_chain(&exynos_gpu_notifier, cur_state, &cur_state);
		gpu_old_state = cur_state;
	}
}

static void exynos_check_tmu_noti_state(int temp)
{
	enum tmu_noti_state_t cur_state;

	/* check current temperature state */
	if (temp > HOT_CRITICAL_TEMP)
		cur_state = TMU_CRITICAL;
	else if (temp > HOT_NORMAL_TEMP && temp <= HOT_CRITICAL_TEMP)
		cur_state = TMU_HOT;
	else if (temp > COLD_TEMP && temp <= HOT_NORMAL_TEMP)
		cur_state = TMU_NORMAL;
	else
		cur_state = TMU_COLD;

	exynos_tmu_call_notifier(cur_state, temp);
}

static void exynos_check_mif_noti_state(int temp)
{
	enum mif_noti_state_t cur_state;

	/* check current temperature state */
	if (temp < MIF_TH_TEMP1)
		cur_state = MIF_TH_LV1;
	else if (temp >= MIF_TH_TEMP1 && temp < MIF_TH_TEMP2)
		cur_state = MIF_TH_LV2;
	else
		cur_state = MIF_TH_LV3;

	if (cur_state != mif_old_state) {
#ifdef CONFIG_SOC_EXYNOS5422
		pr_info("mif temperature state %d to %d\n", mif_old_state, cur_state);
#endif
		blocking_notifier_call_chain(&exynos_tmu_notifier, cur_state, &mif_old_state);
		mif_old_state = cur_state;
	}
}

static void exynos_check_gpu_noti_state(int temp)
{
	enum gpu_noti_state_t cur_state;

	/* check current temperature state */
	if (temp >= GPU_TH_TEMP5)
		cur_state = GPU_TRIPPING;
	else if (temp >= GPU_TH_TEMP4 && temp < GPU_TH_TEMP5)
		cur_state = GPU_THROTTLING4;
	else if (temp >= GPU_TH_TEMP3 && temp < GPU_TH_TEMP4)
		cur_state = GPU_THROTTLING3;
	else if (temp >= GPU_TH_TEMP2 && temp < GPU_TH_TEMP3)
		cur_state = GPU_THROTTLING2;
	else if (temp >= GPU_TH_TEMP1 && temp < GPU_TH_TEMP2)
		cur_state = GPU_THROTTLING1;
	else if (temp > COLD_TEMP && temp < GPU_TH_TEMP1)
		cur_state = GPU_NORMAL;
	else
		cur_state = GPU_COLD;

	exynos_gpu_call_notifier(cur_state);
}

/* Get temperature callback functions for thermal zone */
static int exynos_get_temp(struct thermal_zone_device *thermal,
			unsigned long *temp)
{
	void *data;

	if (!th_zone->sensor_conf) {
		pr_info("Temperature sensor not initialised\n");
		return -EINVAL;
	}
	data = th_zone->sensor_conf->private_data;
	*temp = th_zone->sensor_conf->read_temperature(data);
	/* convert the temperature into millicelsius */
	*temp = *temp * MCELSIUS;
	return 0;
}

/* Get temperature callback functions for thermal zone */
static int exynos_set_emul_temp(struct thermal_zone_device *thermal,
						unsigned long temp)
{
	void *data;
	int ret = -EINVAL;

	if (!th_zone->sensor_conf) {
		pr_info("Temperature sensor not initialised\n");
		return -EINVAL;
	}
	data = th_zone->sensor_conf->private_data;
	if (th_zone->sensor_conf->write_emul_temp)
		ret = th_zone->sensor_conf->write_emul_temp(data, temp);
	return ret;
}

extern int meizu_get_trend(void);
#define DROPPING_CNT_THRESHOLD 1

/* Get the temperature trend */
static int exynos_get_trend(struct thermal_zone_device *thermal,
			int trip, enum thermal_trend *trend)
{
	int ret;
	unsigned long trip_temp;
	static int dropping_cnt;

	ret = exynos_get_trip_temp(thermal, trip, &trip_temp);
	if (ret < 0)
		return ret;

	*trend = THERMAL_TREND_STABLE;
	/* If cpu temp stable and board temperature is not raising, don't tuning... */
	if ((thermal->temperature == trip_temp) && (meizu_get_trend() != THERMAL_TREND_RAISING)) {
		*trend = THERMAL_TREND_STABLE;
		dropping_cnt ++;
	/* If cpu temp is raising or board temperature is raising, try cooling */
	} else if ((thermal->temperature > trip_temp) || (meizu_get_trend() == THERMAL_TREND_RAISING)) {
		*trend = THERMAL_TREND_RAISING;
		dropping_cnt = 0;
	} else if ((thermal->temperature < trip_temp)) {
		dropping_cnt ++;
		if (dropping_cnt >= DROPPING_CNT_THRESHOLD) {
			dropping_cnt = 0;
			*trend = THERMAL_TREND_DROPPING;
		}
	}

	return 0;
}

#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5422) || defined(CONFIG_SOC_EXYNOS5433)

static struct pm_qos_request big_cpu_num_qos;

static int __ref exynos_throttle_cpu_hotplug(struct thermal_zone_device *thermal)
{
	int ret = 0;
	int cur_temp = 0;
	struct exynos_tmu_data *data = th_zone->sensor_conf->private_data;
	struct exynos_tmu_platform_data *pdata = data->pdata;

	if (!thermal->temperature)
		return -EINVAL;

	cur_temp = thermal->temperature / MCELSIUS;

	if (is_cpu_hotplugged_out) {
		if (cur_temp < pdata->hotplug_in_threshold) {
			/*
			 * If current temperature is lower than low threshold,
			 * call big_cores_hotplug(false) for hotplugged out cpus.
			 */
			pm_qos_update_request(&big_cpu_num_qos, NR_CA15);
			is_cpu_hotplugged_out = false;
		}
	} else {
		if (cur_temp >= pdata->hotplug_out_threshold) {
			/*
			 * If current temperature is higher than high threshold,
			 * call big_cores_hotplug(true) to hold temperature down.
			 */
			pm_qos_update_request(&big_cpu_num_qos, 0);
			is_cpu_hotplugged_out = true;
		}
	}

	return ret;
}
#endif

/* Operation callback functions for thermal zone */
static struct thermal_zone_device_ops const exynos_dev_ops = {
	.bind = exynos_bind,
	.unbind = exynos_unbind,
	.get_temp = exynos_get_temp,
	.set_emul_temp = exynos_set_emul_temp,
        .get_trend = exynos_get_trend,
	.get_mode = exynos_get_mode,
	.set_mode = exynos_set_mode,
	.get_trip_type = exynos_get_trip_type,
	.get_trip_temp = exynos_get_trip_temp,
	.get_crit_temp = exynos_get_crit_temp,
#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5422) || defined(CONFIG_SOC_EXYNOS5433)
	.throttle_cpu_hotplug = exynos_throttle_cpu_hotplug,
#endif
};

/*
 * This function may be called from interrupt based temperature sensor
 * when threshold is changed.
 */
static void exynos_report_trigger(void)
{
	unsigned int i;
	char data[10];
	char *envp[] = { data, NULL };
	enum thermal_trip_type type = 0;

	if (!th_zone || !th_zone->therm_dev)
		return;

        if (th_zone->mode == THERMAL_DEVICE_DISABLED)
		return;

	if (th_zone->bind == false) {
		for (i = 0; i < th_zone->cool_dev_size; i++) {
			if (!th_zone->cool_dev[i])
				continue;
			exynos_bind(th_zone->therm_dev,
					th_zone->cool_dev[i]);
		}
	}

	thermal_zone_device_update(th_zone->therm_dev);

	mutex_lock(&th_zone->therm_dev->lock);

        /*if the temp is too high then power off*/
        if (th_zone->therm_dev->last_temperature >= 115 * MCELSIUS) {
		kmsg_dump(KMSG_DUMP_CRIT_TEMP);
		orderly_poweroff(true);
        }

	/* Find the level for which trip happened */
	for (i = 0; i < th_zone->sensor_conf->trip_data.trip_count; i++) {
		if (th_zone->therm_dev->last_temperature >
                        th_zone->sensor_conf->trip_data.trip_val[i] * MCELSIUS) {
                        type = 1;
                }
			break;
	}

	if (th_zone->mode == THERMAL_DEVICE_ENABLED) {
		if (type)
			th_zone->therm_dev->passive_delay = ACTIVE_INTERVAL;
		else
			th_zone->therm_dev->passive_delay = PASSIVE_INTERVAL;
	}

	snprintf(data, sizeof(data), "%u", i);
	kobject_uevent_env(&th_zone->therm_dev->device.kobj, KOBJ_CHANGE, envp);
	mutex_unlock(&th_zone->therm_dev->lock);
}

/* Register with the in-kernel thermal management */
static int exynos_register_thermal(struct thermal_sensor_conf *sensor_conf)
{
	int ret, count = 0;

	if (!sensor_conf || !sensor_conf->read_temperature) {
		pr_err("Temperature sensor not initialised\n");
		return -EINVAL;
	}

	th_zone = kzalloc(sizeof(struct exynos_thermal_zone), GFP_KERNEL);
	if (!th_zone)
		return -ENOMEM;

	th_zone->sensor_conf = sensor_conf;
        th_zone->cool_dev[count] = qos_cooling_register();
        th_zone->cool_dev_size = 1;

	th_zone->therm_dev = thermal_zone_device_register(sensor_conf->name,
			th_zone->sensor_conf->trip_data.trip_count, 0, NULL, &exynos_dev_ops, NULL, PASSIVE_INTERVAL,
			IDLE_INTERVAL);

	if (IS_ERR(th_zone->therm_dev)) {
		pr_err("Failed to register thermal zone device\n");
		ret = PTR_ERR(th_zone->therm_dev);
		goto err_unregister;
	}
	th_zone->mode = THERMAL_DEVICE_ENABLED;

	pr_info("Exynos: Kernel Thermal management registered\n");

	return 0;

err_unregister:
	exynos_unregister_thermal();
	return ret;
}

/* Un-Register with the in-kernel thermal management */
static void exynos_unregister_thermal(void)
{
	int i;

	if (!th_zone)
		return;

	if (th_zone->therm_dev)
		thermal_zone_device_unregister(th_zone->therm_dev);

	for (i = 0; i < th_zone->cool_dev_size; i++) {
		if (th_zone->cool_dev[i])
			cpufreq_cooling_unregister(th_zone->cool_dev[i]);
	}

	kfree(th_zone);
	pr_info("Exynos: Kernel Thermal management unregistered\n");
}

static int exynos_tmu_initialize(struct platform_device *pdev, int id)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct exynos_tmu_platform_data *pdata = data->pdata;
	unsigned int status;
	unsigned int rising_threshold = 0, falling_threshold = 0;
#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)
	unsigned int rising_threshold7_4 = 0, falling_threshold7_4 = 0;
#endif
	int ret = 0, threshold_code, i, trigger_levs = 0;
	int timeout = 20000;

	mutex_lock(&data->lock);

	while(1) {
		status = readb(data->base[id] + EXYNOS_TMU_REG_STATUS);
		if (status)
			break;

		timeout--;
		if (!timeout) {
			pr_err("%s: timeout TMU busy\n", __func__);
			ret = -EBUSY;
			goto out;
		}

		cpu_relax();
		usleep_range(1, 2);
	};

	/* Count trigger levels to be enabled */
	for (i = 0; i < MAX_THRESHOLD_LEVS; i++)
		if (pdata->trigger_levels[g_power_mode][i])
			trigger_levs++;

	if (data->soc == SOC_ARCH_EXYNOS4210) {
		/* Write temperature code for threshold */
		threshold_code = cal_tmu_temp_to_code(data->cal_data, pdata->threshold, 0);
		if (threshold_code < 0) {
			ret = threshold_code;
			goto out;
		}
		writeb(threshold_code,
			data->base[0] + EXYNOS4210_TMU_REG_THRESHOLD_TEMP);
		for (i = 0; i < trigger_levs; i++)
			writeb(pdata->trigger_levels[g_power_mode][i],
			data->base[0] + EXYNOS4210_TMU_REG_TRIG_LEVEL0 + i * 4);

		writel(EXYNOS4210_TMU_INTCLEAR_VAL,
			data->base[i] + EXYNOS_TMU_REG_INTCLEAR);
	} else if (data->soc == SOC_ARCH_EXYNOS) {
		/* Write temperature code for rising and falling threshold */
		for (i = 0; i < trigger_levs; i++) {
			threshold_code = cal_tmu_temp_to_code(data->cal_data,
					pdata->trigger_levels[g_power_mode][i], id);
			if (threshold_code < 0) {
				ret = threshold_code;
				goto out;
			}
			rising_threshold |= threshold_code << 8 * i;
			if (pdata->threshold_falling) {
				threshold_code = cal_tmu_temp_to_code(data->cal_data,
						pdata->trigger_levels[g_power_mode][i] -
						pdata->threshold_falling, id);
				if (threshold_code > 0)
					falling_threshold |=
						threshold_code << 8 * i;
			}
		}

		writel(rising_threshold, data->base[id] + EXYNOS_THD_TEMP_RISE);
		writel(falling_threshold, data->base[id] + EXYNOS_THD_TEMP_FALL);
		writel(EXYNOS_TMU_CLEAR_RISE_INT | EXYNOS_TMU_CLEAR_FALL_INT, data->base[id] + EXYNOS_TMU_REG_INTCLEAR);
	} else if (data->soc == SOC_ARCH_EXYNOS543X) {
#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)
		for (i = 0; i < trigger_levs; i++) {
			threshold_code = cal_tmu_temp_to_code(data->cal_data,
					pdata->trigger_levels[g_power_mode][i], id);
			if (threshold_code < 0) {
				ret = threshold_code;
				goto out;
			}
			if (i < 4)
				rising_threshold |= threshold_code << (8 * i);
			else
				rising_threshold7_4 |= threshold_code << (8 * (i - 4));
			if (pdata->threshold_falling) {
				threshold_code = cal_tmu_temp_to_code(data->cal_data,
						pdata->trigger_levels[g_power_mode][i] -
						pdata->threshold_falling, id);
				if (threshold_code > 0) {
					if (i < 4)
						falling_threshold |= threshold_code << (8 * i);
					else
						falling_threshold7_4 |= threshold_code << (8 * (i - 4));
				}
			}
		}

		writel(rising_threshold,
				data->base[id] + EXYNOS_THD_TEMP_RISE3_0);
		writel(rising_threshold7_4,
				data->base[id] + EXYNOS_THD_TEMP_RISE7_4);
		writel(falling_threshold,
				data->base[id] + EXYNOS_THD_TEMP_FALL3_0);
		writel(falling_threshold7_4,
				data->base[id] + EXYNOS_THD_TEMP_FALL7_4);
		writel(EXYNOS_TMU_CLEAR_RISE_INT | EXYNOS_TMU_CLEAR_FALL_INT,
				data->base[id] + EXYNOS_TMU_REG_INTCLEAR);

		/* Adjuest sampling interval default -> 1ms */
		/* W/A for WTSR */
		writel(0xE10, data->base[id] + EXYNOS_TMU_REG_SAMPLING_INTERVAL);
#endif
	}
out:
	mutex_unlock(&data->lock);

	return ret;
}

static void exynos_tmu_get_efuse(struct platform_device *pdev, int id)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	struct exynos_tmu_platform_data *pdata = data->pdata;
	unsigned int trim_info;
	int timeout = 5;

	mutex_lock(&data->lock);

	if (data->soc == SOC_ARCH_EXYNOS) {
		__raw_writel(EXYNOS_TRIMINFO_RELOAD1,
				data->base[id] + EXYNOS_TRIMINFO_CONFIG);
		__raw_writel(EXYNOS_TRIMINFO_RELOAD2,
				data->base[id] + EXYNOS_TRIMINFO_CONTROL);
		while (readl(data->base[id] + EXYNOS_TRIMINFO_CONTROL) & EXYNOS_TRIMINFO_RELOAD1) {
			if (!timeout) {
				pr_err("Thermal TRIMINFO register reload failed\n");
				break;
			}
			timeout--;
			cpu_relax();
			usleep_range(5, 10);
		}
	}

	/* Save trimming info in order to perform calibration */
	trim_info = readl(data->base[id] + EXYNOS_TMU_REG_TRIMINFO);
#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)
	if (trim_info & CALIB_SEL_MASK)
		pdata->cal_type = TYPE_TWO_POINT_TRIMMING;
	else
		pdata->cal_type = TYPE_ONE_POINT_TRIMMING;

	data->cal_data->cal_type = pdata->cal_type;
	data->cal_data->vptat[id] = (trim_info & VPTAT_CTRL_MASK) >> VPTAT_CTRL_SHIFT;
#endif
	data->cal_data->temp_error1[id] = trim_info & EXYNOS_TMU_TRIM_TEMP_MASK;
	data->cal_data->temp_error2[id] = ((trim_info >> 8) & EXYNOS_TMU_TRIM_TEMP_MASK);

#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5422) || defined(CONFIG_SOC_EXYNOS5433)
	if (data->cal_data->temp_error1[id] == 0)
		data->cal_data->temp_error1[id] = pdata->efuse_value;
#else
	if ((EFUSE_MIN_VALUE > data->cal_data->temp_error1[id]) || (data->cal_data->temp_error1[id] > EFUSE_MAX_VALUE) ||
			(data->cal_data->temp_error1[id] == 0))
		data->cal_data->temp_error1[id] = pdata->efuse_value;
#endif

	mutex_unlock(&data->lock);
}

static void exynos_tmu_control(struct platform_device *pdev, int id, bool on)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);

	mutex_lock(&data->lock);

	cal_tmu_control(data->cal_data, id, on);

	mutex_unlock(&data->lock);
}

static int exynos_tmu_read(struct exynos_tmu_data *data)
{
	int temp, i, max = INT_MIN, min = INT_MAX, gpu_temp = 0;
	int alltemp[EXYNOS_TMU_COUNT] = {0, };
#ifdef CONFIG_EXYNOS_SWTRIP
	char tmustate_string[20];
	char *envp[2];
#endif

	mutex_lock(&data->lock);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		temp = cal_tmu_read(data->cal_data, i);
		alltemp[i] = temp;

		if (i == EXYNOS_GPU_NUMBER) {
#if !defined(CONFIG_RECOVERY_KERNEL) && !defined(CONFIG_CHARGE_KERNEL)
			if ((soc_is_exynos5430() || soc_is_exynos5433()) && !gpu_is_power_on())
				temp = COLD_TEMP + 1;
#endif
			gpu_temp = temp;
		} else {
			if (temp > max)
				max = temp;
			if (temp < min)
				min = temp;
		}

	}

#ifdef CONFIG_EXYNOS_SWTRIP
	if (max >= SWTRIP_TEMP)
		swtrip_counter++;
	else
		swtrip_counter = 0;

	if (swtrip_counter >= SWTRIP_NOISE_COUNT) {
		snprintf(tmustate_string, sizeof(tmustate_string), "TMUSTATE=%d", 3);
		envp[0] = tmustate_string;
		envp[1] = NULL;
		pr_err("[TMU] SW trip by reaching trip temp(%d)!\n", SWTRIP_TEMP);
		kobject_uevent_env(&th_zone->therm_dev->device.kobj, KOBJ_CHANGE, envp);
	}
#endif

	exynos_check_tmu_noti_state(max);
	exynos_check_mif_noti_state(max);
	exynos_check_gpu_noti_state(gpu_temp);

	mutex_unlock(&data->lock);
#if defined(CONFIG_CPU_THERMAL_IPA)
	check_switch_ipa_on(max);
#endif
	pr_debug("[TMU] TMU0 = %d, TMU1 = %d, TMU2 = %d, TMU3 = %d, TMU4 = %d    MAX = %d, GPU = %d\n",
			alltemp[0], alltemp[1], alltemp[2], alltemp[3], alltemp[4], max, gpu_temp);

	return max;
}

static int meizu_tmu_read(struct exynos_tmu_data *data)
{
	int ntc_temp, cpu_temp, cpu_trip_val;

	ntc_temp = meizu_ntc_read();
	cpu_temp = exynos_tmu_read(data);
	cpu_trip_val = exynos_sensor_conf.trip_data.trip_val[0];

	return __meizu_tmu_read(ntc_temp, cpu_temp, cpu_trip_val);
}

#if defined(CONFIG_CPU_THERMAL_IPA)
int ipa_hotplug(bool removecores)
{
	return big_cores_hotplug(removecores);
}
#endif


#ifdef CONFIG_THERMAL_EMULATION
static int exynos_tmu_set_emulation(void *drv_data, unsigned long temp)
{
	struct exynos_tmu_data *data = drv_data;
	unsigned int reg;
	int ret = -EINVAL;
	int i;

	if (data->soc == SOC_ARCH_EXYNOS4210)
		goto out;

	if (temp && temp < MCELSIUS)
		goto out;

	mutex_lock(&data->lock);

	if (temp)
		temp /= MCELSIUS;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		reg = readl(data->base[i] + EXYNOS_EMUL_CON);

		if (temp) {
			reg = (EXYNOS_EMUL_TIME << EXYNOS_EMUL_TIME_SHIFT) |
				(cal_tmu_temp_to_code(data->cal_data, temp, i)
				 << EXYNOS_EMUL_DATA_SHIFT) | EXYNOS_EMUL_ENABLE;
		} else {
			reg &= ~EXYNOS_EMUL_ENABLE;
		}

		writel(reg, data->base[i] + EXYNOS_EMUL_CON);
	}

	mutex_unlock(&data->lock);
	return 0;
out:
	return ret;
}
#else
static int exynos_tmu_set_emulation(void *drv_data,	unsigned long temp)
	{ return -EINVAL; }
#endif/*CONFIG_THERMAL_EMULATION*/

static void exynos_tmu_work(struct work_struct *work)
{
	struct exynos_tmu_data *data = container_of(work,
			struct exynos_tmu_data, irq_work);
	int i;

	mutex_lock(&data->lock);
	if (data->soc != SOC_ARCH_EXYNOS4210)
		for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		writel(EXYNOS_TMU_CLEAR_RISE_INT | EXYNOS_TMU_CLEAR_FALL_INT,
				data->base[i] + EXYNOS_TMU_REG_INTCLEAR);
		}
	else
		writel(EXYNOS4210_TMU_INTCLEAR_VAL,
				data->base[0] + EXYNOS_TMU_REG_INTCLEAR);
	mutex_unlock(&data->lock);

	exynos_report_trigger();
	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		enable_irq(data->irq[i]);
}

static irqreturn_t exynos_tmu_irq(int irq, void *id)
{
	struct exynos_tmu_data *data = id;
	int i;

	pr_debug("[TMUIRQ] irq = %d\n", irq);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		disable_irq_nosync(data->irq[i]);
	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}
static struct thermal_sensor_conf exynos_sensor_conf = {
	.name			= "exynos-therm",
	.read_temperature	= (int (*)(void *))meizu_tmu_read,
	.write_emul_temp	= exynos_tmu_set_emulation,
};
#if defined(CONFIG_CPU_THERMAL_IPA)
static struct ipa_sensor_conf ipa_sensor_conf = {
	.read_soc_temperature	= (int (*)(void *))exynos_tmu_read,
};
#endif
static int exynos_pm_notifier(struct notifier_block *notifier,
		unsigned long pm_event, void *v)
{
	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		is_suspending = true;
		exynos_tmu_call_notifier(TMU_COLD, 0);
		exynos_gpu_call_notifier(TMU_COLD);
		break;
	case PM_POST_SUSPEND:
		is_suspending = false;
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_pm_nb = {
	.notifier_call = exynos_pm_notifier,
};

#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)
void exynos_tmu_core_control(bool on, int id)
{
	int i;
	unsigned int con;
	struct exynos_tmu_data *data;

	if (exynos_tmu_pdev == NULL)
		return;

	data = platform_get_drvdata(exynos_tmu_pdev);

	if (!is_tmu_probed || data == NULL)
		return;

	con = readl(data->base[id] + EXYNOS_TMU_REG_CONTROL);
	con &= TMU_CONTROL_ONOFF_MASK;
	con |= (on ? EXYNOS_TMU_CORE_ON : EXYNOS_TMU_CORE_OFF);
	writel(con, data->base[id] + EXYNOS_TMU_REG_CONTROL);

	if (!on) {
		for (i = 0; i < IDLE_MAX_TIME; i++) {
			if (readl(data->base[id] + EXYNOS_TMU_REG_STATUS) & 0x1)
				break;
		}
		if (i == (IDLE_MAX_TIME - 1))
			pr_err("@@@@@ TMU CHECK BUSY @@@@@@\n");
	}
}
#endif

#if (defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)) && defined(CONFIG_CPU_IDLE)
static void exynos_tmu_all_cores_control(bool on)
{
	int i, j;
	unsigned int con;
	unsigned int status;
	struct exynos_tmu_data *data;

	if (exynos_tmu_pdev == NULL)
		return;

	data = platform_get_drvdata(exynos_tmu_pdev);

	if (data == NULL)
		return;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		con = readl(data->base[i] + EXYNOS_TMU_REG_CONTROL);
		con &= TMU_CONTROL_ONOFF_MASK;
		con |= (on ? EXYNOS_TMU_CORE_ON : EXYNOS_TMU_CORE_OFF);
		writel(con, data->base[i] + EXYNOS_TMU_REG_CONTROL);
	}

	if (!on) {
		for (j=0; j < IDLE_MAX_TIME; j++) {
			status = 0;

			for (i = 0; i < EXYNOS_TMU_COUNT; i++)
				status |= (((readl(data->base[i] + EXYNOS_TMU_REG_STATUS) & 0x1)) << i);

			if (status == 0x1F)
				break;

			}
		if (j == (IDLE_MAX_TIME - 1))
			pr_err("@@@@@ TMU CHECK BUSY @@@@@@\n");
	}
}

static int exynos_pm_dstop_notifier(struct notifier_block *notifier,
		unsigned long pm_event, void *v)
{
	switch (pm_event) {
	case LPA_ENTER:
		exynos_tmu_all_cores_control(false);
		break;
	case LPA_ENTER_FAIL:
	case LPA_EXIT:
		exynos_tmu_all_cores_control(true);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_pm_dstop_nb = {
	.notifier_call = exynos_pm_dstop_notifier,
};
#endif

#if defined(CONFIG_CPU_EXYNOS4210)
static struct exynos_tmu_platform_data const exynos4210_default_tmu_data = {
	.threshold = 80,
	.trigger_levels[0] = 5,
	.trigger_levels[1] = 20,
	.trigger_levels[2] = 30,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 0,
	.trigger_level4_en = 0,
	.trigger_level5_en = 0,
	.trigger_level6_en = 0,
	.trigger_level7_en = 0,
	.gain = 15,
	.reference_voltage = 7,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.freq_tab[0] = {
		.freq_clip_max = 800 * 1000,
		.temp_level = 85,
	},
	.freq_tab[1] = {
		.freq_clip_max = 200 * 1000,
		.temp_level = 100,
	},
	.freq_tab_count = 2,
	.type = SOC_ARCH_EXYNOS4210,
};
#define EXYNOS4210_TMU_DRV_DATA (&exynos4210_default_tmu_data)
#else
#define EXYNOS4210_TMU_DRV_DATA (NULL)
#endif

#if defined(CONFIG_SOC_EXYNOS5250) || defined(CONFIG_SOC_EXYNOS4412)
static struct exynos_tmu_platform_data const exynos_default_tmu_data = {
	.threshold_falling = 10,
	.trigger_levels[0] = 85,
	.trigger_levels[1] = 103,
	.trigger_levels[2] = 110,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 0,
	.trigger_level4_en = 0,
	.trigger_level5_en = 0,
	.trigger_level6_en = 0,
	.trigger_level7_en = 0,
	.gain = 8,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.freq_tab[0] = {
		.freq_clip_max = 800 * 1000,
		.temp_level = 85,
	},
	.freq_tab[1] = {
		.freq_clip_max = 200 * 1000,
		.temp_level = 103,
	},
	.freq_tab_count = 2,
	.type = SOC_ARCH_EXYNOS,
};
#define EXYNOS_TMU_DRV_DATA (&exynos_default_tmu_data)
#else
#define EXYNOS_TMU_DRV_DATA (NULL)
#endif

static struct exynos_tmu_platform_data exynos5430_tmu_data = {
	.threshold_falling = 2,
	.trigger_levels[0] = {50, 60, 90, 100, 105, 105, 110, 115},
	.trigger_levels[1] = {53, 80, 90, 100, 105, 105, 110, 115},
	.trigger_levels[2] = {60, 85, 90, 100, 105, 105, 110, 115},
	.trigger_levels[3] = {90, 93, 95, 100, 105, 108, 110, 115},
	.trigger_level0_en = 0,
	.trigger_level1_en = 0,
	.trigger_level2_en = 0,
	.trigger_level3_en = 1,
	.trigger_level4_en = 1,
	.trigger_level5_en = 1,
	.trigger_level6_en = 1,
	.trigger_level7_en = 1,
	.gain = 8,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 75,
	.freq_tab[0] = {
#ifdef CONFIG_SOC_EXYNOS5430_L
		.freq_clip_max =        1800 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {1800 * 1000, 1800 * 1000, 1800 * 1000, 1800 * 1000},
#endif //CONFIG_POWER_MODE
#else
		.freq_clip_max =        1900 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {1900 * 1000, 1900 * 1000, 1900 * 1000, 1900 * 1000},
#endif //CONFIG_POWER_MODE
#endif
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
#ifdef CONFIG_SOC_EXYNOS5430_L
		.freq_clip_max_kfc =        1300 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {1300 * 1000, 1300 * 1000, 1300 * 1000, 1300 * 1000},
#endif //CONFIG_POWER_MODE
#else
		.freq_clip_max_kfc =        1500 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {1500 * 1000, 1500 * 1000, 1500 * 1000, 1500 * 1000,},
#endif //CONFIG_POWER_MODE
#endif
#endif
		.temp_level =           70,
#ifdef CONFIG_POWER_MODE
		.temp_level_powerm = {50, 53, 60, 90,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[1] = {
		.freq_clip_max =        1800 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {1800 * 1000, 1800 * 1000, 1800 * 1000, 1800 * 1000,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
#ifdef CONFIG_SOC_EXYNOS5430_L
		.freq_clip_max_kfc =        1300 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {1300 * 1000, 1300 * 1000, 1300 * 1000, 1300 * 1000,},
#endif //CONFIG_POWER_MODE
#else
		.freq_clip_max_kfc =        1500 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {1500 * 1000, 1500 * 1000, 1500 * 1000, 1500 * 1000,},
#endif //CONFIG_POWER_MODE
#endif
#endif
		.temp_level =           85,
#ifdef CONFIG_POWER_MODE
		.temp_level_powerm = {55, 85, 85, 85,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[2] = {
		.freq_clip_max =        1500 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {1500 * 1000, 1500 * 1000, 1500 * 1000, 1500 * 1000,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
#ifdef CONFIG_SOC_EXYNOS5430_L
		.freq_clip_max_kfc =        1300 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {1300 * 1000, 1300 * 1000, 1300 * 1000, 1300 * 1000,},
#endif //CONFIG_POWER_MODE
#else
		.freq_clip_max_kfc =        1500 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {1500 * 1000, 1500 * 1000, 1500 * 1000, 1500 * 1000,},
#endif //CONFIG_POWER_MODE
#endif
#endif
		.temp_level =           90,
#ifdef CONFIG_POWER_MODE
		.temp_level_powerm = {60, 90, 90, 90,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[3] = {
		.freq_clip_max =        1300 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {1300 * 1000, 1300 * 1000, 1300 * 1000, 1300 * 1000,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
#ifdef CONFIG_SOC_EXYNOS5430_L
		.freq_clip_max_kfc =        1300 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {1300 * 1000, 1300 * 1000, 1300 * 1000, 1300 * 1000,},
#endif //CONFIG_POWER_MODE
#else
		.freq_clip_max_kfc =        1500 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {1500 * 1000, 1500 * 1000, 1500 * 1000, 1500 * 1000,},
#endif //CONFIG_POWER_MODE
#endif
#endif
		.temp_level =           95,
#ifdef CONFIG_POWER_MODE
		.temp_level_powerm = {65, 95, 95, 95,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[4] = {
		.freq_clip_max =        900 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {900 * 1000, 900 * 1000, 900 * 1000, 900 * 1000,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc =        1200 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {1200 * 1000, 1200 * 1000, 1200 * 1000, 1200 * 1000,},
#endif //CONFIG_POWER_MODE
#endif
		.temp_level =           100,
#ifdef CONFIG_POWER_MODE
		.temp_level_powerm = {70, 100, 100, 100,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[5] = {
		.freq_clip_max =        900 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {900 * 1000, 900 * 1000, 900 * 1000, 900 * 1000,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc =        500 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {500 * 1000, 500 * 1000, 500 * 1000, 500 * 1000,},
#endif //CONFIG_POWER_MODE
#endif
		.temp_level =           105,
#ifdef CONFIG_POWER_MODE
		.temp_level_powerm = {75, 105, 105, 105,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[6] = {
		.freq_clip_max =        900 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_powerm = {900 * 1000, 900 * 1000, 900 * 1000, 900 * 1000,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc =        500 * 1000,
#ifdef CONFIG_POWER_MODE
		.freq_clip_max_kfc_powerm = {500 * 1000, 500 * 1000, 500 * 1000, 500 * 1000,},
#endif //CONFIG_POWER_MODE
#endif
		.temp_level =           100,
#ifdef CONFIG_POWER_MODE
		.temp_level_powerm = {110,110,110, 110,},
#endif //CONFIG_POWER_MODE
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.size[THERMAL_TRIP_ACTIVE] = 1,
	.size[THERMAL_TRIP_PASSIVE] = 5,
       .freq_tab_count = 1,
#ifdef CONFIG_POWER_MODE
       .freq_tab_count_powerm = {1,1,1,1},
#endif //CONFIG_POWER_MODE
	.type = SOC_ARCH_EXYNOS543X,
};

#define EXYNOS5430_TMU_DRV_DATA (&exynos5430_tmu_data)

#ifdef CONFIG_POWER_MODE
DEFINE_MUTEX(thermal_mutex);
extern void switch_thermal_table(int power_mode);

static void update_trip_value(enum power_mode_idx power_mode)
{
        int i;
        int count = 0;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		exynos_tmu_control(exynos_tmu_pdev, i, false);

        g_power_mode = power_mode;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		exynos_tmu_initialize(exynos_tmu_pdev, i);
		exynos_tmu_control(exynos_tmu_pdev, i, true);
	}

        count = tmudata->pdata->freq_tab_count_powerm[power_mode];

        for (i = 0; i < count; i++) {
                exynos_sensor_conf.trip_data.trip_val[i] =
                    tmudata->pdata->threshold + tmudata->pdata->freq_tab[i].temp_level_powerm[power_mode];
        }

        for (i = 0; i < count; i++) {
                 exynos_sensor_conf.cooling_data.freq_data[i].temp_level =
                             tmudata->pdata->freq_tab[i].temp_level_powerm[power_mode];
        }
}

static int exynos_thermal_power_mode_notifier(struct notifier_block *notifier,
		unsigned long power_mode_event, void *v)
{
        if (power_mode_event > POWER_MODE_BENCHMARK) {
                pr_info("%s: invalid value \n", __func__);
                return NOTIFY_BAD;
        }

        mutex_lock(&thermal_mutex);

        update_trip_value(power_mode_event);

        mutex_unlock(&thermal_mutex);

        return NOTIFY_OK;
}

static struct notifier_block exynos_thermal_power_mode_nb = {
	.notifier_call = exynos_thermal_power_mode_notifier,
};
#endif

#if defined(CONFIG_SOC_EXYNOS5422)
static struct exynos_tmu_platform_data exynos5_tmu_data = {
	.threshold_falling = 2,
	.trigger_levels[0] = 80,
	.trigger_levels[1] = 90,
	.trigger_levels[2] = 100,
	.trigger_levels[3] = 115,
	.trigger_level0_en = 1,
	.trigger_level1_en = 1,
	.trigger_level2_en = 1,
	.trigger_level3_en = 1,
	.trigger_level4_en = 0,
	.trigger_level5_en = 0,
	.trigger_level6_en = 0,
	.trigger_level7_en = 0,
	.gain = 8,
	.reference_voltage = 16,
	.noise_cancel_mode = 4,
	.cal_type = TYPE_ONE_POINT_TRIMMING,
	.efuse_value = 55,
	.freq_tab[0] = {
		.freq_clip_max = 1700 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1300 * 1000,
#endif
		.temp_level = 80,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[1] = {
		.freq_clip_max = 1500 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1300 * 1000,
#endif
		.temp_level = 90,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[2] = {
		.freq_clip_max = 900 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1300 * 1000,
#endif
		.temp_level = 95,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[3] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 1200 * 1000,
#endif
		.temp_level = 100,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.freq_tab[4] = {
		.freq_clip_max = 800 * 1000,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.freq_clip_max_kfc = 800 * 1000,
#endif
		.temp_level = 110,
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		.mask_val = &mp_cluster_cpus[CA15],
		.mask_val_kfc = &mp_cluster_cpus[CA7],
#endif
	},
	.size[THERMAL_TRIP_ACTIVE] = 1,
	.size[THERMAL_TRIP_PASSIVE] = 4,
	.freq_tab_count = 5,
	.type = SOC_ARCH_EXYNOS,
};
#define EXYNOS5422_TMU_DRV_DATA (&exynos5_tmu_data)
#else
#define EXYNOS5422_TMU_DRV_DATA (NULL)
#endif

#if defined(CONFIG_SOC_EXYNOS5433)
static struct exynos_tmu_platform_data exynos5433_tmu_data = {
	.type = SOC_ARCH_EXYNOS543X,
};
#define EXYNOS5433_TMU_DRV_DATA (&exynos5433_tmu_data)
#else
#define EXYNOS5433_TMU_DRV_DATA (NULL)
#endif

#ifdef CONFIG_OF
static const struct of_device_id exynos_tmu_match[] = {
	{
		.compatible = "samsung,exynos4210-tmu",
		.data = (void *)EXYNOS4210_TMU_DRV_DATA,
	},
	{
		.compatible = "samsung,exynos4412-tmu",
		.data = (void *)EXYNOS_TMU_DRV_DATA,
	},
	{
		.compatible = "samsung,exynos5250-tmu",
		.data = (void *)EXYNOS_TMU_DRV_DATA,
	},
	{
		.compatible = "samsung,exynos5430-tmu",
		.data = (void *)EXYNOS5430_TMU_DRV_DATA,
	},
	{
		.compatible = "samsung,exynos5422-tmu",
		.data = (void *)EXYNOS5422_TMU_DRV_DATA,
	},
	{
		.compatible = "samsung,exynos5433-tmu",
		.data = (void *)EXYNOS5433_TMU_DRV_DATA,
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_tmu_match);
#endif

static struct platform_device_id exynos_tmu_driver_ids[] = {
	{
		.name		= "exynos4210-tmu",
		.driver_data    = (kernel_ulong_t)EXYNOS4210_TMU_DRV_DATA,
	},
	{
		.name		= "exynos5250-tmu",
		.driver_data    = (kernel_ulong_t)EXYNOS_TMU_DRV_DATA,
	},
	{
		.name		= "exynos5430-tmu",
		.driver_data	= (kernel_ulong_t)EXYNOS5430_TMU_DRV_DATA,
	},
	{
		.name		= "exynos5422-tmu",
		.driver_data	= (kernel_ulong_t)EXYNOS5422_TMU_DRV_DATA,
	},
	{
		.name		= "exynos5433-tmu",
		.driver_data	= (kernel_ulong_t)EXYNOS5433_TMU_DRV_DATA,
	},
	{ },
};
MODULE_DEVICE_TABLE(platform, exynos_tmu_driver_ids);

static inline struct  exynos_tmu_platform_data *exynos_get_driver_data(
			struct platform_device *pdev)
{
#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		const struct of_device_id *match;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		init_mp_cpumask_set();
#endif
		match = of_match_node(exynos_tmu_match, pdev->dev.of_node);
		if (!match)
			return NULL;
		return (struct exynos_tmu_platform_data *) match->data;
	}
#endif
	return (struct exynos_tmu_platform_data *)
			platform_get_device_id(pdev)->driver_data;
}

/* sysfs interface : /sys/devices/10060000.tmu/temp */
static ssize_t
exynos_thermal_sensor_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct exynos_tmu_data *data = th_zone->sensor_conf->private_data;
	unsigned long temp[EXYNOS_TMU_COUNT] = {0,};
	int i, len = 0;

	mutex_lock(&data->lock);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		temp[i] = cal_tmu_read(data->cal_data, i) * MCELSIUS;

	mutex_unlock(&data->lock);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		len += snprintf(&buf[len], PAGE_SIZE, "sensor%d : %ld\n", i, temp[i]);

	return len;
}

static DEVICE_ATTR(temp, S_IRUSR | S_IRGRP, exynos_thermal_sensor_temp, NULL);

/* sysfs interface : /sys/devices/10060000.tmu/curr_temp */
static ssize_t
exynos_thermal_curr_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct exynos_tmu_data *data = th_zone->sensor_conf->private_data;
	unsigned long temp[EXYNOS_TMU_COUNT];
	int i, len = 0;

	if (!(soc_is_exynos5422()))
		return -EPERM;

	if (EXYNOS_TMU_COUNT < 4)
		return -EPERM;

	mutex_lock(&data->lock);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		temp[i] = cal_tmu_read(data->cal_data, i) * 10;
	}

	mutex_unlock(&data->lock);

	/* rearrange temperature with core order
	   sensor0 -> 3 -> 2 -> 1 */
	len += snprintf(&buf[len], PAGE_SIZE, "%ld,", temp[0]);
	len += snprintf(&buf[len], PAGE_SIZE, "%ld,", temp[3]);
	len += snprintf(&buf[len], PAGE_SIZE, "%ld,", temp[2]);
	len += snprintf(&buf[len], PAGE_SIZE, "%ld\n", temp[1]);

	return len;
}

static DEVICE_ATTR(curr_temp, S_IRUGO, exynos_thermal_curr_temp, NULL);

/* sysfs interface : /sys/devices/10060000.tmu/cpu_temp */
static ssize_t
meizu_thermal_cpu_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct exynos_tmu_data *data = th_zone->sensor_conf->private_data;
	int temp;
	int len = 0;

	temp = exynos_tmu_read(data);

	len += snprintf(&buf[len], PAGE_SIZE, "%d\n", temp);

	return len;
}

static DEVICE_ATTR(cpu_temp, S_IRUGO, meizu_thermal_cpu_temp, NULL);

/* sysfs interface : /sys/devices/10060000.tmu/ntc_temp */
static ssize_t
meizu_thermal_ntc_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int temp;
	int len = 0;

	temp = meizu_ntc_read();

	len += snprintf(&buf[len], PAGE_SIZE, "%d\n", temp);

	return len;
}

static DEVICE_ATTR(ntc_temp, S_IRUGO, meizu_thermal_ntc_temp, NULL);

static struct attribute *exynos_thermal_sensor_attributes[] = {
	&dev_attr_temp.attr,
	&dev_attr_curr_temp.attr,
	&dev_attr_ntc_temp.attr,
	&dev_attr_cpu_temp.attr,
	NULL
};

static const struct attribute_group exynos_thermal_sensor_attr_group = {
	.attrs = exynos_thermal_sensor_attributes,
};

static void exynos_set_cal_data(struct exynos_tmu_data *data)
{
	int i;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		data->cal_data->base[i] = data->base[i];

	data->cal_data->gain = data->pdata->gain;
	data->cal_data->reference_voltage = data->pdata->reference_voltage;
	data->cal_data->noise_cancel_mode = data->pdata->noise_cancel_mode;
	data->cal_data->cal_type = data->pdata->cal_type;

	data->cal_data->trigger_level_en[0] = data->pdata->trigger_level0_en;
	data->cal_data->trigger_level_en[1] = data->pdata->trigger_level1_en;
	data->cal_data->trigger_level_en[2] = data->pdata->trigger_level2_en;
	data->cal_data->trigger_level_en[3] = data->pdata->trigger_level3_en;
	data->cal_data->trigger_level_en[4] = data->pdata->trigger_level4_en;
	data->cal_data->trigger_level_en[5] = data->pdata->trigger_level5_en;
	data->cal_data->trigger_level_en[6] = data->pdata->trigger_level6_en;
	data->cal_data->trigger_level_en[7] = data->pdata->trigger_level7_en;
}

static void exynos_tmu_regdump(struct platform_device *pdev, int id)
{
	struct exynos_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int reg_data;

	mutex_lock(&data->lock);

	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_TRIMINFO);
	pr_info("TRIMINFO[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_CONTROL);
	pr_info("TMU_CONTROL[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_CURRENT_TEMP);
	pr_info("CURRENT_TEMP[%d] = 0x%x\n", id, reg_data);
#if defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_RISE3_0);
	pr_info("THRESHOLD_TEMP_RISE3_0[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_RISE7_4);
	pr_info("THRESHOLD_TEMP_RISE7_4[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_FALL3_0);
	pr_info("THRESHOLD_TEMP_FALL3_0[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_FALL7_4);
	pr_info("THRESHOLD_TEMP_FALL7_4[%d] = 0x%x\n", id, reg_data);
#else
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_RISE);
	pr_info("THRESHOLD_TEMP_RISE[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_THD_TEMP_FALL);
	pr_info("THRESHOLD_TEMP_FALL[%d] = 0x%x\n", id, reg_data);
#endif
	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_INTEN);
	pr_info("INTEN[%d] = 0x%x\n", id, reg_data);
	reg_data = readl(data->base[id] + EXYNOS_TMU_REG_INTCLEAR);
	pr_info("INTCLEAR[%d] = 0x%x\n", id, reg_data);

	mutex_unlock(&data->lock);
}

#if defined(CONFIG_SOC_EXYNOS5433)
static int parse_trigger_data(struct device_node *np, struct exynos_tmu_platform_data *pdata, int i)
{
	int ret = 0;
	u32 enable, temp;
	struct device_node *np_trigger;
	char node_name[16];

	snprintf(node_name, sizeof(node_name), "trigger_level_%d", i);

	np_trigger = of_find_node_by_name(np, node_name);
	if (!np_trigger)
		return -EINVAL;

	of_property_read_u32(np_trigger, "temp", &temp);
	of_property_read_u32(np_trigger, "enable", &enable);

	pdata->trigger_levels[g_power_mode][i] = temp;
	switch (i) {
	case 0:
		pdata->trigger_level0_en = (enable == 0) ? 0 : 1;
		break;
	case 1:
		pdata->trigger_level1_en = (enable == 0) ? 0 : 1;
		break;
	case 2:
		pdata->trigger_level2_en = (enable == 0) ? 0 : 1;
		break;
	case 3:
		pdata->trigger_level3_en = (enable == 0) ? 0 : 1;
		break;
	case 4:
		pdata->trigger_level4_en = (enable == 0) ? 0 : 1;
		break;
	case 5:
		pdata->trigger_level5_en = (enable == 0) ? 0 : 1;
		break;
	case 6:
		pdata->trigger_level6_en = (enable == 0) ? 0 : 1;
		break;
	case 7:
		pdata->trigger_level7_en = (enable == 0) ? 0 : 1;
		break;
	}

	return ret;
}

static int parse_throttle_data(struct device_node *np, struct exynos_tmu_platform_data *pdata, int i)
{
	int ret = 0;
	struct device_node *np_throttle;
	char node_name[15];

	snprintf(node_name, sizeof(node_name), "throttle_tab_%d", i);

	np_throttle = of_find_node_by_name(np, node_name);
	if (!np_throttle)
		return -EINVAL;

	of_property_read_u32(np_throttle, "temp", &pdata->freq_tab[i].temp_level);
	of_property_read_u32(np_throttle, "freq_clip_max", &pdata->freq_tab[i].freq_clip_max);

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	of_property_read_u32(np_throttle, "freq_clip_max_kfc", &pdata->freq_tab[i].freq_clip_max_kfc);
	pdata->freq_tab[i].mask_val = &mp_cluster_cpus[CA15];
	pdata->freq_tab[i].mask_val_kfc = &mp_cluster_cpus[CA7];
#endif

	return ret;
}


static int exynos_tmu_parse_dt(struct device_node *np, struct exynos_tmu_platform_data *pdata)
{
	u32 value, cal_type, trigger_level_count;
	int ret = 0, i;

	if (!np)
		return -EINVAL;

	of_property_read_u32(np, "threshold_falling", &value);
	pdata->threshold_falling = value;
	of_property_read_u32(np, "gain", &value);
	pdata->gain = value;
	of_property_read_u32(np, "reference_voltage", &value);
	pdata->reference_voltage = value;
	of_property_read_u32(np, "noise_cancel_mode", &value);
	pdata->noise_cancel_mode = value;
	of_property_read_u32(np, "cal_type", &cal_type);
	of_property_read_u32(np, "efuse_value", &pdata->efuse_value);
	of_property_read_u32(np, "trigger_level_count", &trigger_level_count);
	of_property_read_u32(np, "throttle_count", &pdata->freq_tab_count);
	of_property_read_u32(np, "throttle_active_count", &pdata->size[THERMAL_TRIP_ACTIVE]);
	of_property_read_u32(np, "throttle_passive_count", &pdata->size[THERMAL_TRIP_PASSIVE]);
	of_property_read_u32(np, "hotplug_out_threshold", &pdata->hotplug_out_threshold);
	of_property_read_u32(np, "hotplug_in_threshold", &pdata->hotplug_in_threshold);

	for (i = 0; i < trigger_level_count; i++) {
		ret = parse_trigger_data(np, pdata, i);
		if (ret) {
			pr_err("Failed to load trigger data(%d)\n", i);
			return -EINVAL;
		}
	}

	for (i = 0; i < pdata->freq_tab_count; i++) {
		ret = parse_throttle_data(np, pdata, i);
		if (ret) {
			pr_err("Failed to load throttle data(%d)\n", i);
			return -EINVAL;
		}
	}

	if (cal_type == 1)
		pdata->cal_type = TYPE_ONE_POINT_TRIMMING;
	else if (cal_type == 2)
		pdata->cal_type = TYPE_TWO_POINT_TRIMMING;
	else
		pdata->cal_type = TYPE_NONE;

	return ret;
}
#endif

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
static int exynos5_tmu_cpufreq_notifier(struct notifier_block *notifier, unsigned long event, void *v)
{
	int ret = 0, i;

	switch (event) {
	case CPUFREQ_INIT_COMPLETE:
		ret = exynos_register_thermal(&exynos_sensor_conf);
		is_tmu_probed = true;

		if (ret) {
			dev_err(&exynos_tmu_pdev->dev, "Failed to register thermal interface\n");
			sysfs_remove_group(&exynos_tmu_pdev->dev.kobj, &exynos_thermal_sensor_attr_group);
			unregister_pm_notifier(&exynos_pm_nb);
#if (defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)) && defined(CONFIG_CPU_IDLE)
			exynos_pm_unregister_notifier(&exynos_pm_dstop_nb);
#endif
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
			exynos_cpufreq_init_unregister_notifier(&exynos_cpufreq_nb);
#endif
			platform_set_drvdata(exynos_tmu_pdev, NULL);
			for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
				if (tmudata->irq[i])
					free_irq(tmudata->irq[i], tmudata);
			}
			kfree(tmudata);

			return ret;
		}
#if defined(CONFIG_CPU_THERMAL_IPA)
		ipa_sensor_conf.private_data = exynos_sensor_conf.private_data;
		ipa_register_thermal_sensor(&ipa_sensor_conf);
#endif
		break;
	}
	return 0;
}
#endif

static int exynos_tmu_probe(struct platform_device *pdev)
{
	struct exynos_tmu_data *data;
	struct exynos_tmu_platform_data *pdata = pdev->dev.platform_data;
	int ret, i;
#if defined(CONFIG_SOC_EXYNOS5430)
	unsigned int spd_option_flag, spd_sel;
#endif

	exynos_tmu_pdev = pdev;
	is_suspending = false;

	if (!pdata)
		pdata = exynos_get_driver_data(pdev);

	if (!pdata) {
		dev_err(&pdev->dev, "No platform init data supplied.\n");
		return -ENODEV;
	}

#if defined(CONFIG_SOC_EXYNOS5433)
	ret = exynos_tmu_parse_dt(pdev->dev.of_node, pdata);
	if (ret) {
		dev_err(&pdev->dev, "Failed to load platform data from device tree.\n");
		return -ENODEV;
	}
#else
	pdata->hotplug_in_threshold = CPU_HOTPLUG_IN_TEMP;
	pdata->hotplug_out_threshold = CPU_HOTPLUG_OUT_TEMP;
#endif

#if defined(CONFIG_SOC_EXYNOS5430)
	exynos5430_get_egl_speed_option(&spd_option_flag, &spd_sel);
	if (spd_option_flag == EGL_DISABLE_SPD_OPTION)
		pdata->freq_tab[0].freq_clip_max = 1200 * 1000;
#endif
#if defined(CONFIG_SOC_EXYNOS5433)
	if (cal_get_table_ver() == 0) {
		pdata->freq_tab[0].freq_clip_max = 1100 * 1000;
		pdata->freq_tab[1].freq_clip_max = 1000 * 1000;
		pdata->freq_tab[2].freq_clip_max = 900 * 1000;
	}
#endif

	data = devm_kzalloc(&pdev->dev, sizeof(struct exynos_tmu_data),
					GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "Failed to allocate driver structure\n");
		return -ENOMEM;
	}

	data->cal_data = devm_kzalloc(&pdev->dev, sizeof(struct cal_tmu_data),
					GFP_KERNEL);
	if (!data->cal_data) {
		dev_err(&pdev->dev, "Failed to allocate cal data structure\n");
		return -ENOMEM;
	}

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	exynos_cpufreq_init_register_notifier(&exynos_cpufreq_nb);
#endif

	INIT_WORK(&data->irq_work, exynos_tmu_work);

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		data->irq[i] = platform_get_irq(pdev, i);
		if (data->irq[i] < 0) {
			ret = data->irq[i];
			dev_err(&pdev->dev, "Failed to get platform irq\n");
			goto err_get_irq;
		}

		ret = request_irq(data->irq[i], exynos_tmu_irq,
				IRQF_TRIGGER_RISING, "exynos_tmu", data);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request irq: %d\n", data->irq[i]);
			goto err_request_irq;
		}

		data->mem[i] = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!data->mem[i]) {
			ret = -ENOENT;
			dev_err(&pdev->dev, "Failed to get platform resource\n");
			goto err_get_resource;
		}

		data->base[i] = devm_request_and_ioremap(&pdev->dev, data->mem[i]);
		if (IS_ERR(data->base[i])) {
			ret = PTR_ERR(data->base[i]);
			dev_err(&pdev->dev, "Failed to ioremap memory\n");
			goto err_io_remap;
		}
	}

	if (pdata->type == SOC_ARCH_EXYNOS || pdata->type == SOC_ARCH_EXYNOS4210 ||
			pdata->type == SOC_ARCH_EXYNOS543X)
		data->soc = pdata->type;
	else {
		ret = -EINVAL;
		dev_err(&pdev->dev, "Platform not supported\n");
		goto err_soc_type;
	}

	data->pdata = pdata;
	tmudata = data;
	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	exynos_set_cal_data(data);

#if defined(CONFIG_SOC_EXYNOS5433)
	ret = cal_tmu_otp_read(data->cal_data);

	/* Save the eFuse value before initializing TMU */
	if (ret) {
		for (i = 0; i < EXYNOS_TMU_COUNT; i++)
			exynos_tmu_get_efuse(pdev, i);
	}
#else
	/* Save the eFuse value before initializing TMU */
	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		exynos_tmu_get_efuse(pdev, i);
#endif

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		ret = exynos_tmu_initialize(pdev, i);
		if (ret) {
			dev_err(&pdev->dev, "Failed to initialize TMU\n");
			goto err_tmu;
		}

		exynos_tmu_control(pdev, i, true);
		exynos_tmu_regdump(pdev, i);
	}

	mutex_lock(&data->lock);
	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		int temp = cal_tmu_read(data->cal_data, i);
		pr_debug("[TMU]temp[%d] : %d\n", i, temp);
	}
	mutex_unlock(&data->lock);


	/* Register the sensor with thermal management interface */
	(&exynos_sensor_conf)->private_data = data;
	exynos_sensor_conf.trip_data.trip_count = pdata->freq_tab_count;

	for (i = 0; i < pdata->freq_tab_count; i++) {
		exynos_sensor_conf.trip_data.trip_val[i] =
			pdata->threshold + pdata->freq_tab[i].temp_level;
	}

	exynos_sensor_conf.trip_data.trigger_falling = pdata->threshold_falling;

	exynos_sensor_conf.cooling_data.freq_clip_count =
						pdata->freq_tab_count;
	for (i = 0; i < pdata->freq_tab_count; i++) {
		exynos_sensor_conf.cooling_data.freq_data[i].freq_clip_max =
					pdata->freq_tab[i].freq_clip_max;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
		exynos_sensor_conf.cooling_data.freq_data[i].freq_clip_max_kfc =
					pdata->freq_tab[i].freq_clip_max_kfc;
#endif
		exynos_sensor_conf.cooling_data.freq_data[i].temp_level =
					pdata->freq_tab[i].temp_level;
		if (pdata->freq_tab[i].mask_val) {
			exynos_sensor_conf.cooling_data.freq_data[i].mask_val =
				pdata->freq_tab[i].mask_val;
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
			exynos_sensor_conf.cooling_data.freq_data[i].mask_val_kfc =
				pdata->freq_tab[i].mask_val_kfc;
#endif
		} else
			exynos_sensor_conf.cooling_data.freq_data[i].mask_val =
				cpu_all_mask;
	}

	exynos_sensor_conf.cooling_data.size[THERMAL_TRIP_ACTIVE] = pdata->size[THERMAL_TRIP_ACTIVE];
	exynos_sensor_conf.cooling_data.size[THERMAL_TRIP_PASSIVE] = pdata->size[THERMAL_TRIP_PASSIVE];

	register_pm_notifier(&exynos_pm_nb);
#if (defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)) && defined(CONFIG_CPU_IDLE)
	exynos_pm_register_notifier(&exynos_pm_dstop_nb);
#endif
#ifdef CONFIG_POWER_MODE
        power_mode_register_notifier(&exynos_thermal_power_mode_nb);
#endif
	ret = sysfs_create_group(&pdev->dev.kobj, &exynos_thermal_sensor_attr_group);
	if (ret)
		dev_err(&pdev->dev, "cannot create thermal sensor attributes\n");

	pm_qos_add_request(&big_cpu_num_qos, PM_QOS_CPU_NUM_MAX, NR_CA15);

	is_cpu_hotplugged_out = false;

	return 0;

err_tmu:
	platform_set_drvdata(pdev, NULL);
err_soc_type:
err_io_remap:
err_get_resource:
	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		if (data->irq[i])
			free_irq(data->irq[i], data);
	}
err_request_irq:
err_get_irq:
	kfree(data);

	return ret;
}

static int exynos_tmu_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		exynos_tmu_control(pdev, i, false);

	unregister_pm_notifier(&exynos_pm_nb);
#if (defined(CONFIG_SOC_EXYNOS5430) || defined(CONFIG_SOC_EXYNOS5433)) && defined(CONFIG_CPU_IDLE)
	exynos_pm_unregister_notifier(&exynos_pm_dstop_nb);
#endif
	exynos_unregister_thermal();

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int exynos_tmu_suspend(struct device *dev)
{
	int i;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++)
		exynos_tmu_control(to_platform_device(dev), i, false);

	return 0;
}

static int exynos_tmu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int i;

	for (i = 0; i < EXYNOS_TMU_COUNT; i++) {
		exynos_tmu_initialize(pdev, i);
		exynos_tmu_control(pdev, i, true);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(exynos_tmu_pm,
			 exynos_tmu_suspend, exynos_tmu_resume);
#define EXYNOS_TMU_PM	(&exynos_tmu_pm)
#else
#define EXYNOS_TMU_PM	NULL
#endif

static struct platform_driver exynos_tmu_driver = {
	.driver = {
		.name   = "exynos-tmu",
		.owner  = THIS_MODULE,
		.pm     = EXYNOS_TMU_PM,
		.of_match_table = of_match_ptr(exynos_tmu_match),
	},
	.probe = exynos_tmu_probe,
	.remove	= exynos_tmu_remove,
	.id_table = exynos_tmu_driver_ids,
};

module_platform_driver(exynos_tmu_driver);

MODULE_DESCRIPTION("EXYNOS TMU Driver");
MODULE_AUTHOR("Donggeun Kim <dg77.kim@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-tmu");
