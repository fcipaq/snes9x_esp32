#pragma once

#include <stdbool.h>
#include "hal/adc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool adc_init();
bool adc_setup_gpio(int gpio_num, adc_atten_t atten);
bool adc_digital_read(int gpio, int threshold);
int  adc_analog_read(int gpio);   // calibrated mV (real voltage for VBAT/VEXT)

int  adc_get_vbat_raw();          // raw ADC counts, uncalibrated, undivided
int  adc_get_vext_raw();          // raw ADC counts, uncalibrated, undivided

#ifdef __cplusplus
}
#endif
