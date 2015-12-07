/*
 * kernel/power/power_mode.c
 *
 * add /sys/power/power_mode for set save mode mode .
 *
 * Copyright (C) 2014 zhengwei. zhengwei <zhengwei@meizu.com>
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include "power.h"

#include <linux/power_mode.h>
#include <mach/cpufreq.h>
#include <linux/perf_mode.h>

static struct power_mode_info *cur_power_mode = NULL;
static int power_mode_debug;
static int power_mode_init_flag = 0;

static DEFINE_MUTEX(power_lock);

#define MAX_KFC_FREQ	(1500000)
#define MAX_CPU_FREQ	(2000000)
#define MAX_GPU_FREQ	(600)

struct power_pm_qos {
	struct pm_qos_request little_qos;	/* freqs */
	struct pm_qos_request big_qos;
	struct pm_qos_request gpu_qos;
	struct pm_qos_request mif_qos;
	struct pm_qos_request int_qos;
	struct pm_qos_request disp_qos;
	struct pm_qos_request isp_qos;
	struct pm_qos_request cpu_qos;		/* number */
	struct pm_qos_request kfc_qos;
};

enum {
	PM_QOS_POWER_MODE = 0,
	PM_QOS_THERMAL,
	PM_QOS_END,
};

static struct power_pm_qos power_pm_qos[PM_QOS_END];

/* For the available freqs, please see: kernel/power/perf_mode.c */

static struct power_mode_info power_mode_info[POWER_MODE_END] = {
	/* name,       big_freq, little_freq, gpu_freq, mif_freq, int_freq, disp_freq, isp_freq, little_num, big_num*/
	{ "low",       1500000,  1500000,     500,      825000,   543000,   317000,    666000,   4,          0 },
	{ "normal",    1800000,  1500000,     600,      825000,   543000,   317000,    777000,   4,          2 },
	{ "high",      2000000,  1500000,     600,      825000,   543000,   317000,    777000,   4,          4 },
	{ "benchmark", 2000000,  1500000,     600,      825000,   543000,   317000,    777000,   4,          4 },
	{ "custom",    2000000,  1500000,     600,      825000,   543000,   317000,    777000,   4,          4 },
	{ "thermal",   2000000,  1500000,     600,      825000,   543000,   317000,    777000,   4,          4 },
};

/* add power mode notify service*/

static BLOCKING_NOTIFIER_HEAD(power_mode_notifier_list);

int power_mode_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&power_mode_notifier_list, nb);
}
EXPORT_SYMBOL(power_mode_register_notifier);

int power_mode_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&power_mode_notifier_list, nb);
}
EXPORT_SYMBOL(power_mode_unregister_notifier);

int power_mode_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&power_mode_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(power_mode_notifier_call_chain);

/* request_power_mode(): Request a mode of power
 *
 * @mode: POWER_MODE_LOW...POWER_MODE_CUSTOM
 * @qos: PM_QOS_POWER_MODE...PM_QOS_THERMAL
 * @timeout: 0, never timeout; > 0, timeout
 */

static void request_power_mode_unlocked(unsigned int mode, int qos, unsigned long timeout)
{
	struct power_mode_info *p;
	struct power_pm_qos *q;

	if (mode >= POWER_MODE_END || mode < POWER_MODE_LOW)
		return;

	if (qos >= PM_QOS_END || qos < PM_QOS_POWER_MODE)
		return;

	p = &power_mode_info[mode];
	q = &power_pm_qos[qos];

	if (timeout) {
		/* Lock little cpus: 0~3 */
		pm_qos_update_request_timeout(&q->little_qos, p->little_freq, timeout);

		/* Lock big cpus: 4~7 */
		pm_qos_update_request_timeout(&q->big_qos, p->big_freq, timeout);

		/* Lock GPU */
		pm_qos_update_request_timeout(&q->gpu_qos, p->gpu_freq, timeout);

		/* Lock MIF */
		pm_qos_update_request_timeout(&q->mif_qos, p->mif_freq, timeout);

		/* Lock INT */
		pm_qos_update_request_timeout(&q->int_qos, p->int_freq, timeout);

		/* Lock DISP */
		pm_qos_update_request_timeout(&q->disp_qos, p->disp_freq, timeout);

		/* Lock ISP */
		pm_qos_update_request_timeout(&q->isp_qos, p->isp_freq, timeout);

		/* Lock CPU */
		pm_qos_update_request_timeout(&q->cpu_qos, p->big_num, timeout);

		/* Lock KFC */
		pm_qos_update_request_timeout(&q->kfc_qos, p->little_num, timeout);
	} else {
		/* Lock little cpus: 0~3 */
		pm_qos_update_request(&q->little_qos, p->little_freq);

		/* Lock big cpus: 4~7 */
		pm_qos_update_request(&q->big_qos, p->big_freq);

		/* Lock GPU */
		pm_qos_update_request(&q->gpu_qos, p->gpu_freq);

		/* Lock MIF */
		pm_qos_update_request(&q->mif_qos, p->mif_freq);

		/* Lock INT */
		pm_qos_update_request(&q->int_qos, p->int_freq);

		/* Lock DISP */
		pm_qos_update_request(&q->disp_qos, p->disp_freq);

		/* Lock ISP */
		pm_qos_update_request(&q->isp_qos, p->isp_freq);

		/* Lock CPU */
		pm_qos_update_request(&q->cpu_qos, p->big_num);

		/* Lock KFC */
		pm_qos_update_request(&q->kfc_qos, p->little_num);
	}

	if (power_mode_debug)
		pr_info("%s: little: %u big: %u gpu: %u mif: %u int: %u disp: %u isp: %u cpu: %d kfc: %d timeout: %lu (ms)\n", __func__,
			p->little_freq, p->big_freq, p->gpu_freq, p->mif_freq, p->int_freq, p->disp_freq, p->isp_freq,
			p->big_num, p->little_num, timeout ? timeout / 1000 : 0);
}

void __request_power_mode(unsigned int mode, int qos, unsigned long timeout)
{
	mutex_lock(&power_lock);
	request_power_mode_unlocked(mode, qos, timeout);
	mutex_unlock(&power_lock);
}

static void request_power_mode(unsigned int big_freq, unsigned int little_freq,
		unsigned int gpu_freq, unsigned int mif_freq, unsigned int int_freq,
		unsigned int disp_freq, unsigned int isp_freq,
		unsigned int little_num, unsigned int big_num, int mode, int qos)
{
	struct power_mode_info *p;

	mutex_lock(&power_lock);

	p = &power_mode_info[mode];
	p->big_freq = big_freq;
	p->little_freq = little_freq;
	p->gpu_freq = gpu_freq;
	p->mif_freq = mif_freq;
	p->int_freq = int_freq;
	p->disp_freq = disp_freq;
	p->isp_freq = isp_freq;
	p->little_num = little_num;
	p->big_num = big_num;

	request_power_mode_unlocked(mode, qos, 0);

	mutex_unlock(&power_lock);
}

void request_thermal_power_mode(unsigned int big_freq, unsigned int little_freq,
		unsigned int gpu_freq, unsigned int mif_freq, unsigned int int_freq,
		unsigned int disp_freq, unsigned int isp_freq,
		unsigned int little_num, unsigned int big_num)
{
	if (!power_mode_init_flag)
		return;

	request_power_mode(big_freq, little_freq, gpu_freq, mif_freq, int_freq, disp_freq, isp_freq,
		little_num, big_num, POWER_MODE_THERMAL, PM_QOS_THERMAL);
}

void request_custom_power_mode(unsigned int big_freq, unsigned int little_freq,
		unsigned int gpu_freq, unsigned int mif_freq, unsigned int int_freq,
		unsigned int disp_freq, unsigned int isp_freq,
		unsigned int little_num, unsigned int big_num)
{
	request_power_mode(big_freq, little_freq, gpu_freq, mif_freq, int_freq, disp_freq, isp_freq,
		little_num, big_num, POWER_MODE_CUSTOM, PM_QOS_POWER_MODE);
}

static void show_power_mode_list(void)
{
	int i;

	pr_debug("================== power mode ====================\n");

	pr_debug("%10s %10s %10s %10s %10s %10s %10s %10s %10s %10s\n", "name", "big_freq", "little_freq", "gpu_freq", "mif_freq", "int_freq", "disp_freq", "isp_freq", "little_num", "big_num");
	for (i = 0; i < POWER_MODE_END; i++) {
		if (i == POWER_MODE_BENCHMARK)
			continue;
		pr_debug("%10s %10u %10u %10u %10u %10u %10u %10u %10u %10u\n",
			power_mode_info[i].name, power_mode_info[i].big_freq, power_mode_info[i].little_freq,
			power_mode_info[i].gpu_freq, power_mode_info[i].mif_freq, power_mode_info[i].int_freq,
			power_mode_info[i].disp_freq, power_mode_info[i].isp_freq,
			power_mode_info[i].little_num, power_mode_info[i].big_num);
	}
}

static ssize_t show_power_mode(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	mutex_lock(&power_lock);
	if (strcmp("benchmark", cur_power_mode->name) == 0)
		ret = sprintf(buf, "power_mode: high.\n");
	else
		ret = sprintf(buf, "power_mode: %s\n", cur_power_mode->name);
	mutex_unlock(&power_lock);
	show_power_mode_list();

	return ret;
}

static ssize_t store_power_mode(struct kobject *kobj, struct attribute *attr,const char *buf, size_t count)
{
	char str_power_mode[POWER_MODE_LEN];
	int ret , i;

	ret = sscanf(buf, "%11s", str_power_mode);
	if (ret != 1)
		return -EINVAL;

	mutex_lock(&power_lock);
	for (i = 0; i < POWER_MODE_END; i++) {
		if (!strnicmp(power_mode_info[i].name, str_power_mode, POWER_MODE_LEN)) {
			break;
		}
	}

	if (i < POWER_MODE_END) {
		cur_power_mode = &power_mode_info[i];
		if (i <= POWER_MODE_CUSTOM)
			request_power_mode_unlocked(i, PM_QOS_POWER_MODE, 0);
		else
			request_power_mode_unlocked(i, PM_QOS_THERMAL, 0);

		power_mode_notifier_call_chain(i, cur_power_mode);

		if (strcmp("benchmark", cur_power_mode->name) == 0)
			pr_info("store power_mode to high... \n");
		else
			pr_info("store power_mode to %s \n", cur_power_mode->name);
	}
	mutex_unlock(&power_lock);

	return count;
}

static ssize_t show_power_debug(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;

	mutex_lock(&power_lock);
	ret = sprintf(buf, "debug: %u\n", power_mode_debug);
	mutex_unlock(&power_lock);

	return ret;
}

static ssize_t store_power_debug(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int debug;

	mutex_lock(&power_lock);

	ret = sscanf(buf, "%u", &debug);
	if (ret != 1)
		goto fail;

	power_mode_debug = debug;
	pr_info("%s: debug = %u\n", __func__, power_mode_debug);
	mutex_unlock(&power_lock);

	return count;

fail:
	pr_err("usage: echo debug > /sys/power/power_debug\n\n");
	mutex_unlock(&power_lock);

	return -EINVAL;
}

static ssize_t show_power_custom(struct kobject *kobj, struct attribute *attr, char *buf)
{
	int ret;
	struct power_mode_info *p;

	mutex_lock(&power_lock);
	p = &power_mode_info[POWER_MODE_CUSTOM];
	ret = sprintf(buf, "big: %u little: %u gpu: %u mif: %u int: %u disp: %u isp: %u kfc: %u cpu: %u\n",
		p->big_freq, p->little_freq, p->gpu_freq, p->mif_freq, p->int_freq, p->disp_freq, p->isp_freq,
		p->little_num, p->big_num);
	mutex_unlock(&power_lock);

	return ret;
}

static ssize_t store_power_custom(struct kobject *kobj, struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int big_freq, little_freq, gpu_freq, mif_freq, int_freq, disp_freq, isp_freq, little_num, big_num;
	struct power_mode_info *p;

	mutex_lock(&power_lock);

	ret = sscanf(buf, "%u %u %u %u %u %u %u %u %u", &big_freq, &little_freq, &gpu_freq, &mif_freq, &int_freq, &disp_freq,
			&isp_freq, &little_num, &big_num);
	if (ret != 9)
		goto fail;

	p = &power_mode_info[POWER_MODE_CUSTOM];
	p->big_freq = big_freq;
	p->little_freq = little_freq;
	p->gpu_freq = gpu_freq;
	p->mif_freq = mif_freq;
	p->int_freq = int_freq;
	p->disp_freq = disp_freq;
	p->isp_freq = isp_freq;
	p->little_num = little_num;
	p->big_num = big_num;

	if (power_mode_debug)
		pr_info("custom:\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\n",
			p->big_freq, p->little_freq, p->gpu_freq, p->mif_freq, p->int_freq, p->disp_freq, p->isp_freq,
			p->little_num, p->big_num);

	mutex_unlock(&power_lock);

	return count;

fail:
	pr_err("usage: echo big_freq little_freq gpu_freq mif_freq int_freq disp_freq isp_freq little_num big_num > /sys/power/power_custom\n\n");
	mutex_unlock(&power_lock);

	return -EINVAL;
}

define_one_global_rw(power_mode);
define_one_global_rw(power_debug);
define_one_global_rw(power_custom);

static struct attribute * g[] = {
	&power_mode.attr,
	&power_debug.attr,
	&power_custom.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

static int power_mode_init(void)
{
	int error, i;

	pr_err("M76 Power mode init, default POWER_MODE_NORMAL\n");

	error = sysfs_create_group(power_kobj, &attr_group);
	if (error)
		return error;

	mutex_init(&power_lock);
	cur_power_mode = &power_mode_info[POWER_MODE_NORMAL];
	/* show_power_mode_list(); */

	for (i = PM_QOS_POWER_MODE; i < PM_QOS_END; i++) {
		pm_qos_add_request(&power_pm_qos[i].little_qos, PM_QOS_KFC_FREQ_MAX, MAX_KFC_FREQ);
		pm_qos_add_request(&power_pm_qos[i].big_qos, PM_QOS_CPU_FREQ_MAX, MAX_CPU_FREQ);
		pm_qos_add_request(&power_pm_qos[i].gpu_qos, PM_QOS_GPU_FREQ_MAX, MAX_GPU_FREQ);
		pm_qos_add_request(&power_pm_qos[i].mif_qos, PM_QOS_BUS_FREQ_MAX, PM_QOS_BUS_FREQ_MAX_DEFAULT_VALUE);
		pm_qos_add_request(&power_pm_qos[i].int_qos, PM_QOS_DEVICE_FREQ_MAX, PM_QOS_DEVICE_FREQ_MAX_DEFAULT_VALUE);
		pm_qos_add_request(&power_pm_qos[i].disp_qos, PM_QOS_DISPLAY_FREQ_MAX, PM_QOS_DISPLAY_FREQ_MAX_DEFAULT_VALUE);
		pm_qos_add_request(&power_pm_qos[i].isp_qos, PM_QOS_CAM_FREQ_MAX, PM_QOS_CAM_FREQ_MAX_DEFAULT_VALUE);
		pm_qos_add_request(&power_pm_qos[i].cpu_qos, PM_QOS_CPU_NUM_MAX, NR_CA15);
		pm_qos_add_request(&power_pm_qos[i].kfc_qos, PM_QOS_KFC_NUM_MAX, NR_CA7);
	}

	power_mode_init_flag = 1;

	return error;
}

late_initcall_sync(power_mode_init);
