#ifndef _SAMSUNG_ADC_BATTERY_H
#define _SAMSUNG_ADC_BATTERY_H

struct samsung_adc_battery_pdata {
	unsigned int interval_ms;
	unsigned int sample_count;
	unsigned int no_battery_mv;
	unsigned int min_mv;
	unsigned int max_mv;
	unsigned int adc_per_uv;
};
#endif
