#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include "hwcfg.h"
#include "power.h"
#include "adc.h"

#include "driver/gpio.h"
#include "soc/gpio_num.h"

int pwr_initialized = 0;
int pwr_sd_on = 0;

// ADC (millivolts of the real voltage, seeded with the first reading)
int bat_vol = 0;
int src_vol = 0;

#define ADC_AVG_WIN (50)

// Read battery voltage and average.
// adc_analog_read() returns calibrated millivolts, divider already compensated.
int pwr_get_bat_vol() {
  if (PIN_ANA_VBAT == GPIO_NUM_NC)
    return 0;

  int vol = adc_analog_read(PIN_ANA_VBAT);

  if (bat_vol == 0)
    bat_vol = vol;
  else
    bat_vol = (bat_vol * ADC_AVG_WIN + vol) / (ADC_AVG_WIN + 1);

  return bat_vol;
}

// Read external supply voltage and average
int pwr_get_src_vol() {
  if (PIN_ANA_VEXT == GPIO_NUM_NC)
    return 0;

  int vol = adc_analog_read(PIN_ANA_VEXT);

  if (src_vol == 0)
    src_vol = vol;
  else
    src_vol = (src_vol * ADC_AVG_WIN + vol) / (ADC_AVG_WIN + 1);

  return src_vol;
}

// Return type of currently used power source:
// either on battery or on grid
int pwr_get_source() {
  if (PIN_CHG_STAT == GPIO_NUM_NC)
    return PWR_ON_GRD;

  if (!gpio_get_level(PIN_CHG_STAT)) // || pwr_get_bat_vol() > (PWR_CHG_MIN_VOL)
  //if (pwr_get_src_vol() > 1300)  // TODO fix me
    return PWR_ON_GRD;
  else
    return PWR_ON_BAT;
}

void pwr_sd(int on) {
#ifdef PIN_SD_DET
  if (on) {
    if (!pwr_sd_on) {
      pwr_sd_on = 1;
      gpio_set_direction(PIN_SD_DET, GPIO_MODE_INPUT);
      gpio_set_pull_mode(PIN_SD_DET, GPIO_FLOATING);
    }
  }
  else {
    if (pwr_sd_on) {
      pwr_sd_on = 0;
      gpio_set_direction(PIN_SD_DET, GPIO_MODE_OUTPUT);
      gpio_set_level(PIN_SD_DET, 1);
    }
  }
#endif
}

bool pwr_sd_det() {
#ifdef PIN_SD_DET
  if (!pwr_sd_on)
    return false;
  else
    return gpio_get_level(PIN_SD_DET) == 0;
#else
  return true;
#endif
}

void pwr_shutdown() {
#ifdef PIN_PWR_EN
  if (!pwr_initialized)
    return;
  gpio_set_level(PIN_PWR_EN, 0);
#endif
}

int pwr_get_bat_level(int granularity) {
  //if (pwr_get_source() == PWR_ON_GRD)
    // some other logic

  int level = (pwr_get_bat_vol() - PWR_BAT_MIN_VOL) * 100 / (PWR_BAT_MAX_VOL - PWR_BAT_MIN_VOL);

  if (level > 100)
    level = 100;

  if (level < 0)
    level = 0;

  // return level in steps of "granularity", because the voltage divider
  // will otherwise deliver very inaccurate data
  level /= 100 / granularity;

  if (level > (granularity - 1))
    level = granularity - 1;

  if (level < 0)
    level = 0;

  return level;

}

void pwr_amp(int on) {
#ifdef PIN_SND_EN
  if (!pwr_initialized)
    return;
  gpio_set_level(PIN_SND_EN, on);
#endif
}

void snd_mute() {
  pwr_amp(0);
}

void snd_unmute() {
  pwr_amp(1);
}

void pwr_init() {
  if (pwr_initialized)
    return;

  // adc.cpp forces 12 dB for VBAT/VEXT (divider-scaled inputs)
  adc_init();
  if (PIN_ANA_VEXT != GPIO_NUM_NC)
    adc_setup_gpio(PIN_ANA_VEXT, ADC_ATTEN_DB_12);

  if (PIN_ANA_VBAT != GPIO_NUM_NC)
    adc_setup_gpio(PIN_ANA_VBAT, ADC_ATTEN_DB_12);

  if (PIN_CHG_STAT != GPIO_NUM_NC) {
    gpio_reset_pin(PIN_CHG_STAT);
    gpio_set_direction(PIN_CHG_STAT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_CHG_STAT, GPIO_PULLUP_ONLY);
  }

  if (PIN_PWR_EN != GPIO_NUM_NC) {
    gpio_reset_pin(PIN_PWR_EN);
    gpio_set_direction(PIN_PWR_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PWR_EN, 1);
  }

  if (PIN_SND_EN != GPIO_NUM_NC) {
    gpio_set_direction(PIN_SND_EN, GPIO_MODE_OUTPUT);
    pwr_amp(0);
  }

  pwr_sd(1);

  pwr_initialized = 1;
}
