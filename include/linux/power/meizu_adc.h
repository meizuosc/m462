#ifndef _MEIZU_ADC_H
#define _MEIZU_ADC_H

#define MEIZU_M76_AC_ADC_VAL 1820
#define INVALID_ADC_VAL (-1000)

extern int read_batid_adc(void);
extern int read_acid_adc(void);
extern int read_thermal_adc(void);

#endif
