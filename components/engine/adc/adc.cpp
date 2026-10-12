#include "adc.h"
#include "hwcfg.h"

#include <stdio.h>
#include <stdbool.h>

#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

#define ADC_UNITS      (2)
#define ADC_MAX_CHAN   (10)
#define ADC_OVERSAMPLE (16)   // raw samples averaged per read
#define ADC_SETTLE_US  (50)   // pause between samples so the sample cap can recharge

// VBAT and VEXT are each behind a 2 x R divider: pin voltage = real / 2.
// adc_analog_read() returns the REAL voltage for these two pins.
#define ADC_PWR_DIV    (2)

// The pin sees up to ~2.1 V (VBAT) / ~2.5 V (VEXT): needs 12 dB attenuation.
#define ADC_PWR_ATTEN  ADC_ATTEN_DB_12

static adc_oneshot_unit_handle_t adc_handle[ADC_UNITS] = { NULL, NULL };
static adc_cali_handle_t adc_cali_handle[ADC_UNITS][ADC_MAX_CHAN];
static adc_atten_t adc_chan_atten[ADC_UNITS][ADC_MAX_CHAN];

static bool adc_is_pwr_pin(int gpio) {
  return gpio >= 0 && (gpio == (int)PIN_ANA_VBAT || gpio == (int)PIN_ANA_VEXT);
}

// Nominal full-scale voltage (mV) per attenuation, used only when the chip
// has no eFuse calibration scheme. ESP32-P4: Vref = 1100 mV, attenuation
// factors k = 100 % / 75 % / 50 % / 25 %.
static int adc_nominal_fullscale_mv(adc_atten_t atten) {

  switch (atten) {
    case ADC_ATTEN_DB_0:   return 1100;
    case ADC_ATTEN_DB_2_5: return 1467;
    case ADC_ATTEN_DB_6:   return 2200;
    default:               return 4400;   // ADC_ATTEN_DB_12
  }

}

static bool adc_lookup(int gpio, adc_unit_t *unit, adc_channel_t *channel) {
  if (gpio < 0)
    return false;
  if (adc_oneshot_io_to_channel(gpio, unit, channel) != ESP_OK)
    return false;
  if ((int)*unit >= ADC_UNITS || (int)*channel >= ADC_MAX_CHAN)
    return false;
  return true;
}

bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle) {
  adc_cali_handle_t handle = NULL;
  esp_err_t ret = ESP_ERR_NOT_SUPPORTED;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t cfg = {};
  cfg.unit_id  = unit;
  cfg.chan     = channel;
  cfg.atten    = atten;
  cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
  ret = adc_cali_create_scheme_curve_fitting(&cfg, &handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t cfg = {};
  cfg.unit_id  = unit;
  cfg.atten    = atten;
  cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
  ret = adc_cali_create_scheme_line_fitting(&cfg, &handle);
#endif

  if (ret == ESP_OK) {
    *out_handle = handle;
    printf("ADC%d ch%d: calibration OK\n", (int)unit + 1, (int)channel);
    return true;
  }

  *out_handle = NULL;
  printf("ADC%d ch%d: NO calibration (err 0x%x), using nominal full scale %d mV\n",
         (int)unit + 1, (int)channel, (unsigned)ret,
         adc_nominal_fullscale_mv(atten));
  return false;
}

bool adc_setup_gpio(int gpio_num, adc_atten_t atten) {
  adc_unit_t unit;
  adc_channel_t channel;

  if (!adc_lookup(gpio_num, &unit, &channel)) {
    printf("ADC: GPIO %d is not a valid ADC pin\n", gpio_num);
    return false;
  }

  if (adc_is_pwr_pin(gpio_num)) {
    atten = ADC_PWR_ATTEN;
    // make sure no internal pull resistor loads the divider
    gpio_pullup_dis((gpio_num_t)gpio_num);
    gpio_pulldown_dis((gpio_num_t)gpio_num);
  }

  adc_oneshot_chan_cfg_t config = {};
  config.atten    = atten;
  config.bitwidth = ADC_BITWIDTH_DEFAULT;

  if (adc_oneshot_config_channel(adc_handle[unit], channel, &config) != ESP_OK) {
    printf("ADC: channel config failed for GPIO %d\n", gpio_num);
    return false;
  }

  adc_chan_atten[unit][channel] = atten;
  adc_calibration_init(unit, channel, atten, &adc_cali_handle[unit][channel]);

  return true;
}

// Oversampled raw (uncalibrated) read. The first conversion is discarded and
// samples are spaced out, because a high source impedance (5k..50k divider)
// cannot recharge the ADC sample capacitor between back-to-back conversions.
static bool adc_read_raw(int gpio, adc_unit_t *unit, adc_channel_t *channel, int *raw) {
  if (!adc_lookup(gpio, unit, channel))
    return false;
  if (adc_handle[*unit] == NULL)
    return false;

  int r;
  adc_oneshot_read(adc_handle[*unit], *channel, &r);   // dummy read
  esp_rom_delay_us(ADC_SETTLE_US);

  long sum = 0;
  int n = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; i++) {
    if (adc_oneshot_read(adc_handle[*unit], *channel, &r) == ESP_OK) {
      sum += r;
      n++;
    }
    esp_rom_delay_us(ADC_SETTLE_US);
  }
  if (n == 0)
    return false;

  *raw = (int)(sum / n);
  return true;
}

bool adc_digital_read(int gpio, int threshold) {
  if (gpio == GPIO_NUM_NC)
    return false;

  adc_unit_t unit;
  adc_channel_t channel;
  int raw;

  if (!adc_read_raw(gpio, &unit, &channel, &raw))
    return false;

  return raw > threshold;
}

// Millivolts at the pin (no divider compensation).
static int adc_read_pin_mv(int gpio) {
  adc_unit_t unit;
  adc_channel_t channel;
  int raw;

  if (!adc_read_raw(gpio, &unit, &channel, &raw))
    return 0;

  int mv = 0;
  adc_cali_handle_t cali = adc_cali_handle[unit][channel];

  if (!(cali != NULL && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK))
    mv = raw * adc_nominal_fullscale_mv(adc_chan_atten[unit][channel]) / 4095;

  return mv < 0 ? 0 : mv;
}

// Millivolts. For VBAT/VEXT the divider is compensated.
int adc_analog_read(int gpio) {
  if (gpio == GPIO_NUM_NC)
    return 0;

  int mv = adc_read_pin_mv(gpio);

  if (adc_is_pwr_pin(gpio))
    mv *= ADC_PWR_DIV;

  return mv;
}

// Raw (undivided, oversampled) ADC counts for debugging.
int adc_get_vbat_raw() {
  if (PIN_ANA_VBAT == GPIO_NUM_NC)
    return 0;
  adc_unit_t unit;
  adc_channel_t channel;
  int raw = 0;
  adc_read_raw(PIN_ANA_VBAT, &unit, &channel, &raw);
  return raw;
}

int adc_get_vext_raw() {
  if (PIN_ANA_VEXT == GPIO_NUM_NC)
    return 0;
  adc_unit_t unit;
  adc_channel_t channel;
  int raw = 0;
  adc_read_raw(PIN_ANA_VEXT, &unit, &channel, &raw);
  return raw;
}

bool adc_init() {
  static bool adc_init_complete = false;

  if (adc_init_complete)
    return true;

  for (int u = 0; u < ADC_UNITS; u++) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id  = (adc_unit_t)u;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle[u]));
  }

  adc_init_complete = true;
  return true;
}
