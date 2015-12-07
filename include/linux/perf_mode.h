#ifndef PERF_MODE
#define PERF_MODE

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

#include <mach/cpufreq.h>

#define PERF_MODE_LEN   (11)

struct perf_mode_info {
	char		*name;
	unsigned int	big_freq;
	unsigned int	little_freq;
	unsigned int	gpu_freq;
	unsigned int	mif_freq;
	unsigned int	int_freq;
	unsigned int	little_num;
	unsigned int	big_num;
};

enum {
	PERF_MODE_MIN = 0,
	PERF_MODE_LOW,
	PERF_MODE_NORMAL,
	PERF_MODE_HIGH,
	PERF_MODE_ULTRA,
	PERF_MODE_MAX,
	PERF_MODE_CUSTOM,
	PERF_MODE_BENCHMARK,
	PERF_MODE_END,
};

enum {
	PM_QOS_PERF_MODE = 0,
	PM_QOS_EVENT_FLICK,
	PM_QOS_EVENT_DOWN,
	PM_QOS_EVENT_UP,
	PM_QOS_CUSTOM,
	PM_QOS_BENCHMARK,
	PM_QOS_TOTAL,
};

enum {
	HMP_BOOST_NONE,
	HMP_BOOST_SEMI,
	HMP_BOOST_FORCE,
};

#define DEFAULT_PERF_MODE        PERF_MODE_NORMAL
#define DEFAULT_PERF_DEBUG       0
#define DEFAULT_BOOST            1
#define DEFAULT_BOOST_HMP        HMP_BOOST_SEMI
#define DEFAULT_BOOST_TIMEOUT_MS (1 * MSEC_PER_SEC)
#define DEFAULT_BOOST_HMP_TIMEOUT_MS (DEFAULT_BOOST_TIMEOUT_MS * 5)

#ifdef CONFIG_PERF_MODE
extern void request_perf_mode(unsigned int mode, int qos, int hmp_boost, unsigned long timeout, unsigned long hmp_timeout);
extern void cancel_perf_mode(unsigned int qos_mode);
#else
#define cancel_perf_mode(qos_mode)	do { } while (0)
#define request_perf_mode(a, b, c, d, e)	do { } while (0)
#endif

extern char *get_perf_mode_name(unsigned int mode);

#endif /* PERF_MODE */
