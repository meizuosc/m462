#ifndef POWER_MODE
#define POWER_MODE

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm_qos.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/cpufreq.h>
#include <linux/notifier.h>

#define POWER_MODE_LEN	(11)
#define BENCHMARK_HMP_BOOST_TIMEOUT_US (3 * 60 * 1000 * 1000UL)

struct power_mode_info {
	char		*name;
	unsigned int	big_freq;
	unsigned int	little_freq;
	unsigned int	gpu_freq;
	unsigned int	mif_freq;
	unsigned int	int_freq;
	unsigned int	disp_freq;
	unsigned int	isp_freq;
	unsigned int	little_num;
	unsigned int	big_num;
};

enum power_mode_idx {
	POWER_MODE_LOW,
	POWER_MODE_NORMAL,
	POWER_MODE_HIGH,
	POWER_MODE_BENCHMARK,
	POWER_MODE_CUSTOM,
	POWER_MODE_THERMAL,
	POWER_MODE_END,
};

#ifdef CONFIG_POWER_MODE
extern int power_mode_register_notifier(struct notifier_block *nb);
extern int power_mode_unregister_notifier(struct notifier_block *nb);
extern int power_mode_notifier_call_chain(unsigned long val, void *v);
extern void request_thermal_power_mode(unsigned int big_freq, unsigned int little_freq,
		unsigned int gpu_freq, unsigned int mif_freq, unsigned int int_freq,
		unsigned int disp_freq, unsigned int isp_freq,
		unsigned int little_num, unsigned int big_num);
#else
#define power_mode_register_notifier(nb)	do { } while (0)
#define power_mode_unregister_notifier(nb)	do { } while (0)
#define power_mode_notifier_call_chain(val, v)	do { } while (0)
#define request_thermal_power_mode(a,b,c,d,e)	do { } while (0)
#endif

#endif /* POWER_MODE */

