/*
 *  linux/drivers/thermal/qos_cooling.c
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/power_mode.h>
#include <linux/hrtimer.h>

static unsigned long cooling_current_state = 0;
static unsigned int int_freq =  543000;
static unsigned int disp_freq = 317000;
static unsigned int isp_freq =  777000;
struct thermal_cooling_device *cool_dev;
extern struct kobject *power_kobj;
DEFINE_MUTEX(qos_table_lock);

struct thermal_qos {
        unsigned int little_num;
        unsigned int big_num;
        unsigned int big_freq;
        unsigned int little_freq;
        unsigned int gpu_freq;
        unsigned int mif_freq;
        unsigned int power;
        unsigned int perf;
};

 /*little_num, big_num, cpu_freq, kfc_freq, power, perf*/
static struct thermal_qos qos_table[] = {
        {4, 4, 2000000, 1500000, 600, 825000, 1215, 37110},
        {4, 3, 2000000, 1500000, 600, 825000, 1143, 31110},
        {4, 4, 1800000, 1500000, 600, 825000, 1126, 36360},
        {4, 3, 1800000, 1500000, 600, 825000, 999, 30060},
        {4, 2, 2000000, 1500000, 600, 825000, 969, 24670},
        {4, 4, 1600000, 1500000, 600, 825000, 924, 33560},
        {4, 2, 1800000, 1500000, 600, 825000, 845, 23760},
        {4, 3, 1600000, 1500000, 600, 825000, 825, 27960},
        {4, 1, 2000000, 1500000, 600, 825000, 811, 18160},
        {4, 4, 1400000, 1500000, 600, 825000, 804, 30760},
        {4, 2, 1600000, 1500000, 600, 825000, 750, 22360},
        {4, 3, 1400000, 1500000, 600, 825000, 730, 25860},
        {4, 4, 1200000, 1500000, 600, 825000, 701, 27960},
        {4, 1, 1800000, 1500000, 600, 825000, 700, 17460},
        {4, 2, 1400000, 1500000, 600, 825000, 658, 20960},
        {4, 3, 1200000, 1500000, 600, 825000, 637, 23760},
        {4, 1, 1600000, 1500000, 600, 825000, 631, 16760},
        {4, 4, 1000000, 1500000, 550, 825000, 619, 25160},
        {4, 2, 1200000, 1500000, 550, 825000, 605, 19560},
        {4, 3, 1000000, 1500000, 550, 825000, 583, 21660},
        {4, 1, 1400000, 1500000, 550, 825000, 530, 16760},
        {4, 2, 1000000, 1500000, 550, 825000, 524, 18160},
        {4, 1, 1200000, 1500000, 500, 825000, 501, 15360},
        {4, 1, 1000000, 1500000, 500, 825000, 469, 14660},
        {4, 1, 1000000, 1400000, 500, 825000, 468, 14650},
        {4, 1, 1000000, 1200000, 500, 825000, 466, 14645},
        {3, 1, 1000000, 1400000, 500, 825000, 465, 14644},
        {4, 1, 1000000, 1000000, 500, 825000, 464, 14642},
        {3, 1, 1000000, 1200000, 500, 825000, 463, 14641},
        {3, 1, 1000000, 1000000, 500, 825000, 462, 14640},
        {4, 0, 2000000, 1500000, 500, 633000, 302, 11160},
        {4, 0, 2000000, 1500000, 420, 543000, 261, 10460},
        {4, 0, 2000000, 1400000, 420, 543000, 260,  9672},
        {4, 0, 2000000, 1400000, 420, 543000, 252,  7812},
        {4, 0, 2000000, 1400000, 420, 543000, 251,  7811},
};

/* NOTE: Should have the same size as qos_table */
static struct thermal_qos benchmark_qos_table[] = {
        {4, 4, 2000000, 1500000, 600, 825000, 1215, 37110},
        {4, 4, 1900000, 1500000, 600, 825000, 1126, 36360},
        {4, 4, 1800000, 1500000, 600, 825000, 924, 33560},
        {4, 4, 1700000, 1500000, 600, 825000, 924, 33560},
        {4, 4, 1600000, 1500000, 600, 825000, 924, 33560},
        {4, 4, 1500000, 1500000, 600, 825000, 924, 33560},
        {4, 4, 1400000, 1500000, 600, 825000, 924, 33560},
        {4, 4, 1300000, 1500000, 600, 825000, 924, 33560},
        {4, 3, 2000000, 1500000, 600, 825000, 825, 27960},
        {4, 3, 1900000, 1500000, 600, 825000, 825, 27960},
        {4, 3, 1800000, 1500000, 600, 825000, 825, 27960},
        {4, 3, 1700000, 1500000, 600, 825000, 825, 27960},
        {4, 3, 1600000, 1500000, 600, 825000, 825, 27960},
        {4, 3, 1500000, 1500000, 600, 825000, 825, 27960},
        {4, 3, 1400000, 1500000, 600, 825000, 825, 27960},
        {4, 3, 1300000, 1500000, 600, 825000, 825, 27960},
        {4, 2, 2000000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1900000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1800000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1700000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1600000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1500000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1400000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1300000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1200000, 1500000, 600, 825000, 701, 27960},
        {4, 2, 1100000, 1500000, 600, 825000, 701, 27960},
        {4, 1, 1800000, 1500000, 600, 825000, 701, 27960},
        {4, 1, 1700000, 1500000, 600, 825000, 701, 27960},
        {4, 1, 1600000, 1500000, 600, 825000, 701, 27960},
        {4, 1, 1500000, 1500000, 600, 825000, 701, 27960},
        {4, 1, 1400000, 1500000, 600, 825000, 701, 27960},
        {4, 1, 1200000, 1500000, 600, 825000, 701, 27960},
        {4, 1, 1000000, 1500000, 600, 825000, 701, 27960},
        {4, 1,  800000, 1500000, 600, 825000, 701, 27960},
        {4, 0, 2000000, 1500000, 600, 825000, 701, 27960},
};

extern int g_power_mode;

static struct thermal_qos *qos_table_list[POWER_MODE_BENCHMARK + 1] = {
	qos_table,
	qos_table,
	qos_table,
	benchmark_qos_table
};

#define QOS_MAX_STATE (sizeof(qos_table) / sizeof(struct thermal_qos) - 1)
static unsigned long whitelist_table[] = {QOS_MAX_STATE, 30, 25, 21, 15, 9, 0};
static unsigned long g_power_level = QOS_MAX_STATE;
static unsigned long g_whitelist_level = 0;
#define MAX_LIST (sizeof(whitelist_table) / sizeof(unsigned long) - 1)

static int power_level_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        return  sprintf(buf, "%lu\n", g_whitelist_level);
}

static ssize_t power_level_store(struct kobject *kobj, struct kobj_attribute *attr,
                                const char *buffer, size_t count)
{
	if (1 <= sscanf(buffer, "%lu", &g_whitelist_level))
	{
                if (g_whitelist_level <= MAX_LIST && g_whitelist_level >= 0) {
                        pr_debug("%s power_level input %lu, limited index is %ld\n", __func__,
                                        g_whitelist_level, whitelist_table[g_whitelist_level]);
                        g_power_level = whitelist_table[g_whitelist_level];

                        if (g_power_level > QOS_MAX_STATE)
                                g_power_level = QOS_MAX_STATE;
                        return count;
                } else {
                        pr_err("%s bad argument, the range is 0 ~ %d, restore level to 0\n", __func__, MAX_LIST);
                        g_power_level = QOS_MAX_STATE;
                        g_whitelist_level = 0;
                        return -EINVAL;
                }
	} else {
                pr_err("%s bad argument\n", __func__);
                return -EINVAL;
	}
}

static struct kobj_attribute power_level_attr =
        __ATTR(power_level, 0644, power_level_show, power_level_store);

static int qos_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
        *state = QOS_MAX_STATE;

        return 0;
}

static int qos_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = cooling_current_state;

	return 0;
}

#define NOW ktime_to_ms(ktime_get())
static DEFINE_RAW_SPINLOCK(pl_limit_lock);
static u64 pl_end_time = 0;
static int duration_time_show = 0;

static int set_pl_time_out_duration(int duration)
{
        unsigned long flags;
        u64 pl_end_time_new;

        raw_spin_lock_irqsave(&pl_limit_lock, flags);

        pl_end_time_new = NOW + duration * 1000; // change to second
        duration_time_show = duration;

        if (pl_end_time_new > pl_end_time)
                pl_end_time = pl_end_time_new;

        raw_spin_unlock_irqrestore(&pl_limit_lock, flags);

        return 0;
}

/* return 1 when time out */
static int is_time_out(void)
{
        int is_time_out;
        unsigned long flags;

        raw_spin_lock_irqsave(&pl_limit_lock, flags);
        if (NOW < pl_end_time)
                is_time_out = 0;
        else
                is_time_out = 1;
        raw_spin_unlock_irqrestore(&pl_limit_lock, flags);

        return is_time_out;
}

static int power_level_timeout_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        return  sprintf(buf, "%d\n", duration_time_show);
}

static ssize_t power_level_timeout_store(struct kobject *kobj, struct kobj_attribute *attr,
                                const char *buffer, size_t count)
{
        unsigned int duration;

        if (1 <= sscanf(buffer, "%u", &duration))
        {
                set_pl_time_out_duration(duration);

                return count;
        } else {
                pr_err("%s bad argument\n", __func__);

                return -EINVAL;
        }
}

static struct kobj_attribute power_level_timeout_attr =
        __ATTR(power_level_timeout, 0644, power_level_timeout_show, power_level_timeout_store);

static struct attribute *grup[] = {
        &power_level_attr.attr,
        &power_level_timeout_attr.attr,
        NULL,
};

static struct attribute_group attr_group = {
        .attrs = grup,
};

 /*little_qos big_qos, gpu_qos, mif_qos, int_qos, disp_qos, isp_qos,cpu_qos, kfc_qos*/
static int qos_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state)
{
        unsigned long i = 0, j;
        static unsigned long old_state = ULONG_MAX;
	struct thermal_qos *g_qos_table = qos_table_list[g_power_mode];

        cooling_current_state = state;

        if (state >= g_power_level) {
                if (is_time_out()){
                        g_whitelist_level = 0;
                        g_power_level = QOS_MAX_STATE;
                } else {
                        state = g_power_level;
                }
        }

        if (state != old_state) {
                old_state = state;

		i = state;
		/* the perf in benchmark qos table is already sorted */
		if (g_qos_table != benchmark_qos_table) {
			for (j = i; j <= QOS_MAX_STATE; j++) {
				if (g_qos_table[j].perf > g_qos_table[i].perf)
					    i = j;
			}
		}

                request_thermal_power_mode(g_qos_table[i].big_freq,
                                           g_qos_table[i].little_freq,
                                           g_qos_table[i].gpu_freq,
                                           g_qos_table[i].mif_freq,
                                           int_freq, disp_freq, isp_freq,
                                           g_qos_table[i].little_num,
                                           g_qos_table[i].big_num);
        }

        return 0;
}

/* Bind cpufreq callbacks to thermal cooling device ops */
static struct thermal_cooling_device_ops const qos_cooling_ops = {
	.get_max_state = qos_get_max_state,
	.get_cur_state = qos_get_cur_state,
	.set_cur_state = qos_set_cur_state,
};

struct thermal_cooling_device* qos_cooling_register(void)
{
        int ret = 0;
	char dev_name[THERMAL_NAME_LENGTH];

	snprintf(dev_name, sizeof(dev_name), "thermal-qos-cooling");

	cool_dev = thermal_cooling_device_register(dev_name, NULL,
						   &qos_cooling_ops);

        ret = sysfs_create_group(power_kobj, &attr_group);
        if (ret)
                pr_err("%s: faild to creat /sys/power/power_level\n",__func__);

	return cool_dev;
}
EXPORT_SYMBOL_GPL(cpufreq_cooling_register);

void qos_cooling_unregister(struct thermal_cooling_device *cdev)
{
	if (cool_dev != NULL )
                thermal_cooling_device_unregister(cool_dev);
}
EXPORT_SYMBOL_GPL(cpufreq_cooling_unregister);
