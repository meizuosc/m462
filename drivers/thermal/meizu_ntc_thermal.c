#include <linux/kernel.h>
#include <linux/compiler.h>

#include <linux/power/meizu_adc.h>
#include <linux/bsearch.h>
#include <linux/power_mode.h>
#include <linux/thermal.h>

#define ADC_BASE            4096
#define RESISTANCE_BASE     100000

#define NTC_TEMP_MAX        65
#define NTC_TEMP_MIN        -30
#define NTC_TEMP_INVALID    -1000

struct temp_table {
	int temp;
	int resistance;
	int k;
};

static struct temp_table ttable[] = {
	{60,  22220, -102},
	{55,  27090, -81},
	{50,  33190, -64},
	{45,  40900, -51},
	{40,  50680, -40},
	{35,  63180, -31},
	{30,  79230, -24},
	{25,  100000, -18},
	{20,  127000, -14},
	{15,  162500, -10},
	{10,  209400, -8},
	{5,   271800, -5},
	{0,   355600, -4},
	{-5,  469100, -3},
	{-10, 624100, -2},
	{-15, 837800, -1},
	{-20, 1135000, -26},
	{-25, 1154000, 0}, // no last k so 0 instead.
};

#define TEMP_TABLE_SIZE    ARRAY_SIZE(ttable)

static int resistance_cmp(const void *key, const void *ttp)
{
        const int res = (int)key;
        const struct temp_table *tt = ttp;
        const struct temp_table *tt_next = tt + 1;

        if (res >= tt->resistance) {
                  if (res <= tt_next->resistance)
                          return 0;
                  else
                          return 1;
        }

        return -1;
}

static struct temp_table *temp_table_find(int resistance)
{
        return bsearch((void *)resistance, ttable, TEMP_TABLE_SIZE, sizeof(struct temp_table), resistance_cmp);
}

#define NTC_READ_CNT_THRESHOLD 6
static int cur_ntc_temp = NTC_TEMP_INVALID;
static int ntc_read_cnt;

int meizu_ntc_read(void)
{
	int ntc_resistance = 0;
	long temp = NTC_TEMP_INVALID, adc_num = INVALID_ADC_VAL;
	struct temp_table *found = NULL;
	ntc_read_cnt ++;

	if (cur_ntc_temp != NTC_TEMP_INVALID && ntc_read_cnt <= NTC_READ_CNT_THRESHOLD)
		return cur_ntc_temp;

	if (ntc_read_cnt > NTC_READ_CNT_THRESHOLD)
		ntc_read_cnt = 0;

	/* get adc num */
	adc_num = read_thermal_adc();

	/* get temperature */
	if (adc_num == INVALID_ADC_VAL)
		/* adc is not ready */
		return NTC_TEMP_INVALID;

	ntc_resistance = (adc_num * RESISTANCE_BASE) / (ADC_BASE - adc_num);
	found = temp_table_find(ntc_resistance);
	if (found)
		temp = found->k * (ntc_resistance - found->resistance) / RESISTANCE_BASE + found->temp;

	/* out of table */
	if (temp == NTC_TEMP_INVALID) {
		if (ntc_resistance < ttable[0].resistance)
			temp = NTC_TEMP_MAX;
		else if (ntc_resistance > ttable[TEMP_TABLE_SIZE - 1].resistance)
			temp = NTC_TEMP_MIN;
	}

	return temp;
}

/* Implement the policy for the balance of temp and performance */

static int ntc_trip0_temp[] = { 38, 38, 38, 30 };
static int ntc_trip1_temp[] = { 40, 43, 46, 50 };
static int cpu_temp_offset[] = { 5, 5, 5, 0 };
static int cur_power_mode;

int meizu_get_trend(void)
{
	int cur_trip_temp = ntc_trip1_temp[cur_power_mode];

	if (cur_ntc_temp > cur_trip_temp)
		return THERMAL_TREND_RAISING;
	else if (cur_ntc_temp < cur_trip_temp)
		return THERMAL_TREND_DROPPING;
	else
		return THERMAL_TREND_STABLE;
}

extern void set_cpu_hot(bool hot);

int __meizu_tmu_read(int ntc_temp, int cpu_temp, int cpu_trip_val)
{
	int cur_trip_temp = ntc_trip0_temp[cur_power_mode];
	int cur_cpu_temp_offset = cpu_temp_offset[cur_power_mode];

	cur_ntc_temp = ntc_temp;

	pr_debug("%s: ntc = %d ntc_trip_temp = %d cpu = %d trip_val = %d\n", __func__, ntc_temp, cur_trip_temp, cpu_temp, cpu_trip_val);

#ifdef CONFIG_EXYNOS5_DYNAMIC_CPU_HOTPLUG
	if (cur_power_mode == POWER_MODE_NORMAL) {
		if ((cpu_temp >= cpu_trip_val - 1) && (ntc_temp >= cur_trip_temp))
			set_cpu_hot(true);

		if ((cpu_temp <= cpu_trip_val - 2) || (ntc_temp < cur_trip_temp - 1))
			set_cpu_hot(false);
	} else {
		set_cpu_hot(false);
	}
#endif

	if ((ntc_temp != NTC_TEMP_INVALID) && (ntc_temp < cur_trip_temp) && (cpu_temp < cpu_trip_val + cur_cpu_temp_offset))
		return ntc_temp;

	return cpu_temp;
}

#ifdef CONFIG_POWER_MODE
static int meizu_ntc_power_mode_notifier(struct notifier_block *notifier,
		unsigned long power_mode_event, void *v)
{
	if (power_mode_event > POWER_MODE_BENCHMARK) {
		pr_info("%s: invalid value \n", __func__);
		return NOTIFY_BAD;
	}

	cur_power_mode = power_mode_event;

	return NOTIFY_OK;
}

static struct notifier_block meizu_ntc_power_mode_nb = {
	.notifier_call = meizu_ntc_power_mode_notifier,
};
#endif

static int __init meizu_ntc_init(void)
{
	cur_power_mode = POWER_MODE_BENCHMARK;

#ifdef CONFIG_POWER_MODE
	power_mode_register_notifier(&meizu_ntc_power_mode_nb);
#endif

	return 0;
}

late_initcall_sync(meizu_ntc_init);
