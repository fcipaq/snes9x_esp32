/*
 * Snes9x running on the ESP32-P4-Function-EV-Board
 *
 * Copyright (C) 2023 Daniel Kammer (daniel.kammer@web.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "driver/ledc.h"

#include <cassert>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_ldo_regulator.h"
#include "esp_dma_utils.h"
#include "spi_flash_mmap.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_ek79007.h"

#define TEST_LCD_H_RES (1024)
#define TEST_LCD_V_RES (600)
#define TEST_LCD_BIT_PER_PIXEL (16)
#define TEST_PIN_NUM_LCD_RST (GPIO_NUM_27)
#define TEST_PIN_NUM_BK_LIGHT (GPIO_NUM_26)  // set to -1 if not used
#define TEST_LCD_BK_LIGHT_ON_LEVEL (1)
#define TEST_LCD_BK_LIGHT_OFF_LEVEL !TEST_LCD_BK_LIGHT_ON_LEVEL
#define TEST_PIN_NUM_VER_FLIP (-1)
#define TEST_PIN_NUM_HOR_FLIP (-1)
#define TEST_LCD_ROTATE_LEVEL (1)

#if TEST_LCD_BIT_PER_PIXEL == 24
#define TEST_MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB888)
#elif TEST_LCD_BIT_PER_PIXEL == 18
#define TEST_MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB666)
#elif TEST_LCD_BIT_PER_PIXEL == 16
#define TEST_MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB565)
#endif

#define TEST_MIPI_DSI_PHY_PWR_LDO_CHAN (3)
#define TEST_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

extern "C" {
#include "snes9x/snes9x.h"
#include "snes9x/soundux.h"
#include "snes9x/memmap.h"
#include "snes9x/apu.h"
#include "snes9x/display.h"
#include "snes9x/cpuexec.h"
#include "snes9x/srtc.h"
#include "snes9x/save.h"
#include "snes9x/gfx.h"
}

static char *TAG = "ek79007_test";
static esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
static SemaphoreHandle_t refresh_finish = xSemaphoreCreateBinary();
;
volatile uint16_t *fb[2] = { NULL, NULL };
volatile uint16_t *fb_hw[2] = { NULL, NULL };

struct timeval _startTime;

volatile uint16_t *fb_back = NULL;
volatile int fb_num = 1;
volatile int line_block_cnt = 0;

volatile SemaphoreHandle_t lcdxfer_go = xSemaphoreCreateBinary();
volatile SemaphoreHandle_t vsync_event = xSemaphoreCreateBinary();

#define LINE_BUF_SIZE (2)

IRAM_ATTR static bool test_notify_refresh_ready(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) {
  BaseType_t need_yield = pdFALSE;

  fb_num++;
  if (fb_num > 1)
    fb_num = 0;

  uint16_t *fb = (uint16_t *)fb_hw[fb_num];

  // It might occur that fb_hw has not yet been passed, but the mipi
  // scanout has already begun (this is a race condition). In that
  // case we still need to increment the line counter or the
  // screen content will be shifted.
  if (fb != NULL) {

    int ofs = TEST_LCD_H_RES / 2 - SNES_WIDTH * 2 / 2;

    int ofs_y = (TEST_LCD_V_RES / 2 - SNES_HEIGHT * 2 / 2) / 2;

    if ((line_block_cnt * LINE_BUF_SIZE + LINE_BUF_SIZE > ofs_y * 2) && (line_block_cnt * LINE_BUF_SIZE + LINE_BUF_SIZE < (SNES_HEIGHT + ofs_y) * 2)) {
      int a = ((line_block_cnt - ofs_y) * LINE_BUF_SIZE) / 2 * SNES_WIDTH;

      for (int x = 0; x < SNES_WIDTH * 2; x++)
        fb[x + ofs] = fb[x + ofs + TEST_LCD_H_RES] = fb_back[x / 2 + a];
    } else {
      for (int x = 0; x < SNES_WIDTH * 2; x++)
        fb[x + ofs] = fb[x + ofs + TEST_LCD_H_RES] = 0x0000;  // black
    }

    esp_cache_msync((void *)fb_hw[fb_num], LINE_BUF_SIZE * TEST_LCD_H_RES * 2, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
  }

  line_block_cnt++;
  if (line_block_cnt == TEST_LCD_V_RES / LINE_BUF_SIZE - 0) {
    line_block_cnt = 0;
    xSemaphoreGiveFromISR(vsync_event, &need_yield);
    //      xSemaphoreGiveFromISR(vsync_event, &need_yield);
    //      portYIELD_FROM_ISR( need_yield );
  }

  return (need_yield == pdTRUE);
}

static void test_init_lcd(void) {
#if TEST_PIN_NUM_BK_LIGHT >= 0
  ESP_LOGI(TAG, "Turn on LCD backlight");
  gpio_config_t bk_gpio_config = {
    .pin_bit_mask = 1ULL << TEST_PIN_NUM_BK_LIGHT,
    .mode = GPIO_MODE_OUTPUT
  };
  assert(gpio_config(&bk_gpio_config) == ESP_OK);
  assert(gpio_set_level(TEST_PIN_NUM_BK_LIGHT, TEST_LCD_BK_LIGHT_ON_LEVEL) == ESP_OK);
#endif

  // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
#ifdef TEST_MIPI_DSI_PHY_PWR_LDO_CHAN
  ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
  esp_ldo_channel_config_t ldo_mipi_phy_config = {
    .chan_id = TEST_MIPI_DSI_PHY_PWR_LDO_CHAN,
    .voltage_mv = TEST_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
  };
  assert(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy) == ESP_OK);
#endif

  ESP_LOGI(TAG, "Initialize MIPI DSI bus");
  esp_lcd_dsi_bus_config_t bus_config = EK79007_PANEL_BUS_DSI_2CH_CONFIG();
  assert(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus) == ESP_OK);

  ESP_LOGI(TAG, "Install panel IO");
  esp_lcd_dbi_io_config_t dbi_config = EK79007_PANEL_IO_DBI_CONFIG();
  assert(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io) == ESP_OK);

  ESP_LOGI(TAG, "Install LCD driver of ek79007");
  esp_lcd_dpi_panel_config_t dpi_config = EK79007_1024_600_PANEL_60HZ_CONFIG(TEST_MIPI_DPI_PX_FORMAT);
  dpi_config.num_fbs = 2;
  ek79007_vendor_config_t vendor_config = {
    .mipi_config = {
      .dsi_bus = mipi_dsi_bus,
      .dpi_config = &dpi_config,
    },
    .flags = {
      .use_mipi_interface = 1,
    },
  };
  const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = TEST_PIN_NUM_LCD_RST,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = TEST_LCD_BIT_PER_PIXEL,
    .vendor_config = &vendor_config,
  };
  assert(esp_lcd_new_panel_ek79007(mipi_dbi_io, &panel_config, &panel_handle) == ESP_OK);
  assert(esp_lcd_dpi_panel_get_frame_buffer(panel_handle, 2, (void **)&fb_hw[0], (void **)&fb_hw[1]) == ESP_OK);
  assert(esp_lcd_panel_reset(panel_handle) == ESP_OK);
  assert(esp_lcd_panel_init(panel_handle) == ESP_OK);

  assert(refresh_finish);
  xSemaphoreGive(refresh_finish);
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
    .on_refresh_done = test_notify_refresh_ready,
  };
  assert(esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, refresh_finish) == ESP_OK);
}

static void test_deinit_lcd(void) {
  assert(esp_lcd_panel_del(panel_handle) == ESP_OK);
  assert(esp_lcd_panel_io_del(mipi_dbi_io) == ESP_OK);
  assert(esp_lcd_del_dsi_bus(mipi_dsi_bus) == ESP_OK);
  panel_handle = NULL;
  mipi_dbi_io = NULL;
  mipi_dsi_bus = NULL;

  if (ldo_mipi_phy) {
    assert(esp_ldo_release_channel(ldo_mipi_phy) == ESP_OK);
    ldo_mipi_phy = NULL;
  }

  vSemaphoreDelete(refresh_finish);
  refresh_finish = NULL;

#if TEST_PIN_NUM_BK_LIGHT >= 0
  assert(gpio_reset_pin(TEST_PIN_NUM_BK_LIGHT) == ESP_OK);
#endif
}

uint32_t millis() {
  struct timeval currentTime;
  gettimeofday(&currentTime, NULL);
  return (uint32_t)(((currentTime.tv_sec - _startTime.tv_sec) * 1000) + ((currentTime.tv_usec - _startTime.tv_usec) / 1000));  // - _timeSuspended;
}

int brightness = 4;
/* ---------- LCD END ------------*/

// controls
uint16_t dpad;
uint16_t buts;

//static bool apu_enabled = true;
//static bool lowpass_filter = false;
//static int frameskip = 4;

// snes stuff
bool overclock_cycles = false;
int one_c = 4, slow_one_c = 5, two_c = 6;
extern SGFX GFX;       // snes9x API
char *savestate_name;  // = "save.sav";  // short file name dos

// save state
uint32_t just_saved_timer;
int just_saved = 0;
int sd_init = 0;

// sound
#define AUDIO_SAMPLE_RATE (32040)
#define AUDIO_BUFFER_LENGTH (AUDIO_SAMPLE_RATE / 60)
#define AUDIO_LOW_PASS_RANGE ((60 * 65536) / 100)
int16_t audio_buf[2][AUDIO_BUFFER_LENGTH * 2];  // stereo


bool set_brightness(int level) {
  if ((level < 0) || (level > 255))
    return false;

  // ledcWrite(1 /* PWM channel */, level /* duty cycle */);
  ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, level));
  ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

  return true;
}

static void update_keymap(int id) {
}

static bool screenshot_handler(const char *filename, int width, int height) {
  //return rg_display_save_frame(filename, currentUpdate, width, height);
  return true;
}

static bool save_state_handler(char *filename) {
  bool ret = false;
#ifdef SD_ENABLED
  char filename_local[ROM_NAME_LEN + 2];

  filename_local[0] = '/';
  memcpy(&filename_local[1], filename, ROM_NAME_LEN);
  filename_local[ROM_NAME_LEN + 1] = 0;

  File savestate_file;

  //Serial.println("Savestate handler called...");

  if (!sd_init) {
    if (!SD.begin(PIN_SD_CS, SPI, 10000000)) {
      //Serial.println("Unable to init SD card");
      goto bailout;
    }

    sd_init = 1;
  }

  if (SD.exists(filename)) {
    if (!SD.remove(filename)) {
      //Serial.println("Unable to delete game state file");
      goto bailout;
    }
  }

  savestate_file = SD.open(filename_local, FILE_WRITE);

  if (!savestate_file) {
    //Serial.println("Unable to open save state file for writing.");
    goto bailout;
  }

  savestate_file.write((uint8_t *)&CPU, sizeof(CPU));
  savestate_file.write((uint8_t *)&ICPU, sizeof(ICPU));
  savestate_file.write((uint8_t *)&PPU, sizeof(PPU));
  savestate_file.write((uint8_t *)&DMA, sizeof(DMA));
  savestate_file.write((uint8_t *)Memory.VRAM, VRAM_SIZE);
  savestate_file.write((uint8_t *)Memory.RAM, RAM_SIZE);
  savestate_file.write((uint8_t *)Memory.SRAM, SRAM_SIZE);
  savestate_file.write((uint8_t *)Memory.FillRAM, FILLRAM_SIZE);
  savestate_file.write((uint8_t *)&APU, sizeof(APU));
  savestate_file.write((uint8_t *)&IAPU, sizeof(IAPU));
  savestate_file.write((uint8_t *)IAPU.RAM, 0x10000);
  savestate_file.write((uint8_t *)&SoundData, sizeof(SoundData));

  savestate_file.close();

  //Serial.println("Game state saved.");

  ret = true;

bailout:

  if (ret)
    just_saved = 1;
  else
    just_saved = 2;

  just_saved_timer = millis();
#endif
  return ret;
}

static bool load_state_handler(char *filename) {
#ifdef SD_ENABLED
  char filename_local[ROM_NAME_LEN + 2];

  filename_local[0] = '/';
  memcpy(&filename_local[1], filename, ROM_NAME_LEN);
  filename_local[ROM_NAME_LEN + 1] = 0;

  //Serial.println("Loadstate handler called...");

  if (!sd_init) {
    if (!SD.begin(PIN_SD_CS, SPI, 10000000)) {
      //Serial.println("Unable to init SD card");
      just_saved_timer = millis();
      just_saved = 2;
      return false;
    }

    sd_init = 1;
  }

  File readstate_file = SD.open(filename_local, FILE_READ);

  if (!readstate_file) {
    //Serial.println("Failed to load game state:");
    //Serial.print("*");
    //Serial.print(filename_local);
    //Serial.println("*");
    just_saved_timer = millis();
    just_saved = 2;
    return false;
  }

  S9xReset();

  // At this point we can't go back and a failure will corrupt the state anyway
  //Serial.println("Now reading to memory.");

  readstate_file.read((uint8_t *)&CPU, sizeof(CPU));
  readstate_file.read((uint8_t *)&ICPU, sizeof(ICPU));
  readstate_file.read((uint8_t *)&PPU, sizeof(PPU));
  readstate_file.read((uint8_t *)&DMA, sizeof(DMA));
  readstate_file.read((uint8_t *)Memory.VRAM, VRAM_SIZE);
  readstate_file.read((uint8_t *)Memory.RAM, RAM_SIZE);
  readstate_file.read((uint8_t *)Memory.SRAM, SRAM_SIZE);
  readstate_file.read((uint8_t *)Memory.FillRAM, FILLRAM_SIZE);
  readstate_file.read((uint8_t *)&APU, sizeof(APU));
  readstate_file.read((uint8_t *)&IAPU, sizeof(IAPU));
  readstate_file.read((uint8_t *)IAPU.RAM, 0x10000);
  readstate_file.read((uint8_t *)&SoundData, sizeof(SoundData));

  readstate_file.close();

  //Serial.println("Game state loaded.");

  return S9xLoadState();
#else
  return false;
#endif
}

uint32_t timer_str_ld = 0;
int str_ld_debounce = 0;

void check_load_save(void) {
}

static bool reset_handler(bool hard) {
  S9xReset();
  return true;
}

uint Pitch = SNES_WIDTH * 2;  // 16 BPP
uint ZPitch = SNES_WIDTH;

bool S9xInitDisplay(void) {
  GFX.Pitch = Pitch;
  GFX.ZPitch = ZPitch;
  GFX.Screen = (uint8_t *)fb[0];

  // seems crazy but works for Super Mario World et. al. and saves 112 KB of RAM
  GFX.SubScreen = GFX.Screen;                                                                          //(uint8_t*) heap_caps_malloc(GFX.Pitch * SNES_HEIGHT_EXTENDED, MALLOC_CAP_INTERNAL); // 112 KB
  GFX.ZBuffer = (uint8_t *)heap_caps_malloc(GFX.ZPitch * SNES_HEIGHT_EXTENDED, MALLOC_CAP_INTERNAL);   // 56 KB
  GFX.SubZBuffer = (uint8_t *)heap_caps_malloc(GFX.ZPitch * SNES_HEIGHT_EXTENDED, MALLOC_CAP_SPIRAM);  // 56 KB
  return GFX.Screen && GFX.SubScreen && GFX.ZBuffer && GFX.SubZBuffer;
}

void S9xDeinitDisplay(void) {
}

uint32_t S9xReadJoypad(int32_t port) {
  if (port != 0)
    return 0;

  uint32_t joypad = 0;

#if 0
  if (dpad & DPAD_LEFT) joypad |= SNES_LEFT_MASK;
  if (dpad & DPAD_RIGHT) joypad |= SNES_RIGHT_MASK;
  if (dpad & DPAD_UP) joypad |= SNES_UP_MASK;
  if (dpad & DPAD_DOWN) joypad |= SNES_DOWN_MASK;

  if ((buts & BUTTON_1) && (buts & BUTTON_2) && (buts & BUTTON_3)) {
    joypad |= SNES_START_MASK;
  } else {
    if (buts & BUTTON_1) joypad |= SNES_A_MASK;
    if (buts & BUTTON_2) joypad |= SNES_B_MASK;
    if (buts & BUTTON_3) joypad |= SNES_X_MASK;
  }
#endif

  return joypad;
}

bool S9xReadMousePosition(int32_t which1, int32_t *x, int32_t *y, uint32_t *buttons) {
  return false;
}

bool S9xReadSuperScopePosition(int32_t *x, int32_t *y, uint32_t *buttons) {
  return false;
}

bool JustifierOffscreen(void) {
  return true;
}

void JustifierButtons(uint32_t *justifiers) {
  (void)justifiers;
}

void emu_panic(char *str) {
  //  Serial.println(str);
  assert(NULL);
}

void lcdxfertask(void *parameter) {
#ifdef MIXSAMP_CORE2
  int mixsamp = 0;
#endif

  test_init_lcd();  // run on core 1

  while (1) {
    while (xSemaphoreTake(lcdxfer_go, portMAX_DELAY) != pdTRUE)
      ;
      /*
    while ( xSemaphoreTake(lcdxfer_go, 0) != pdTRUE ) {
#ifdef MIXSAMP_CORE2
      if (mixsamp) {
        S9xMixSamples(audio_buf[0], AUDIO_BUFFER_LENGTH * 2);
        mixsamp = 0;
      }
#endif      
    }
*/

#ifdef MIXSAMP_CORE2
    mixsamp = 1;
#endif
  }
}

void setup(void) {
  gettimeofday(&_startTime, NULL);

  assert(lcdxfer_go);
  xSemaphoreGive(lcdxfer_go);
  xSemaphoreTake(lcdxfer_go, 0);

  assert(vsync_event);
  xSemaphoreGive(vsync_event);

  for (int i = 0; i < 2; i++) {
    fb[i] = (uint16_t *)heap_caps_calloc(1, SNES_WIDTH * SNES_HEIGHT_EXTENDED * 2, MALLOC_CAP_DMA);
    assert(fb[i]);
  }

  fb_back = fb[1];

  xTaskCreatePinnedToCore(
    lcdxfertask,   /* Function to implement the task */
    "lcdXferTask", /* Name of the task */
    4096,          /* Stack size in words */
    NULL,          /* Task input parameter */
    0,             /* Priority of the task */
    NULL,          /* Task handle. */
    1);            /* Core where the task should run */

#ifdef SD_ENABLED
  //Serial.println("Initializing SPI...");
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, -1);
#endif
}

// freeRTOS calls app_main from C
extern "C" {
  extern void app_main();
}

void app_main(void) {
  setup();
  /****************************/
  /*    Unused pin to Hi-Z    */
  /****************************/

#if 0
  // SD
  gpio_set_direction(PIN_SD_MOSI, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SD_MISO, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SD_SCK, GPIO_MODE_INPUT);
#endif

#if 0
  ctrl_init();
#endif
  /****************************/
  /*     Graphics setup       */
  /****************************/
#if 0
  gpio_reset_pin((gpio_num_t) PIN_LCD_BL);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = (100000),
        .clk_cfg          = LEDC_USE_XTAL_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = PIN_LCD_BL,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 50, // max: 255
        .hpoint         = 0
    };

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
#endif

  Settings.CyclesPercentage = 100;
  Settings.H_Max = SNES_CYCLES_PER_SCANLINE;
  Settings.FrameTimePAL = 20000;
  Settings.FrameTimeNTSC = 16667;
  Settings.ControllerOption = SNES_JOYPAD;
  Settings.HBlankStart = (256 * Settings.H_Max) / SNES_HCOUNTER_MAX;
  Settings.SoundPlaybackRate = AUDIO_SAMPLE_RATE;
  Settings.DisableSoundEcho = false;
  Settings.InterpolatedSound = true;
#ifdef USE_BLARGG_APU
  Settings.SoundInputRate = AUDIO_SAMPLE_RATE;
#endif

  if (!S9xInitMemory())
    emu_panic("Memory init failed!");

  uint8_t *tmp_rom;

  // load a ROM from flash
  spi_flash_mmap_handle_t out_handle;
  assert(spi_flash_mmap(0x400000 /* size_t src_addr */,
                        0x400000 /* size_t size */,
                        SPI_FLASH_MMAP_DATA /* spi_flash_mmap_memory_t memory */,
                        (const void **)&(tmp_rom) /* const void **out_ptr */,
                        &out_handle /* spi_flash_mmap_handle_t *out_handle */)
         == ESP_OK);
  //Memory.ROM_Size = 2621440;  // All Stars + Super Mario World
  Memory.ROM_Size = 512 * 1024;  // Super Mario World

  // copy ROM to PSRAM
  for (uint32_t i = 0; i < Memory.ROM_Size; i++)
    Memory.ROM[i] = tmp_rom[i + 512];

  if (!S9xInitDisplay())
    emu_panic("Display init failed!");

  if (!S9xInitAPU())
    emu_panic("APU init failed!");

  if (!S9xInitSound(0, 0))
    emu_panic("Sound init failed!");

  if (!S9xInitGFX())
    emu_panic("Graphics init failed!");


  // load rom
  if (!LoadROM(NULL))
    emu_panic("ROM loading failed!");

  printf("ROM loaded.\n");

  savestate_name = (char *)Memory.ROMName;
  load_state_handler(savestate_name);  // try and load a saved state


#ifdef USE_BLARGG_APU
    //S9xSetSamplesAvailableCallback(S9xAudioCallback);
#else
//    rg_audio_sample_t mixbuffer[AUDIO_BUFFER_LENGTH];
//S9xSetPlaybackRate(Settings.SoundPlaybackRate);
#endif

  bool menuCancelled = false;
  bool menuPressed = false;

  int frame_no = 0;


#ifdef ENABLE_FRAMEDROPPING
  int frame = 0;
  int framedrop_no = 0;
#endif

  unsigned long fps_timer = millis();

  while (1) {
#if 0
      dpad = ctrl_dpad_state();
      buts = ctrl_button_state();
#endif
    check_load_save();

    S9xMainLoop();

#ifndef MIXSAMP_CORE2
    S9xMixSamples(audio_buf[0], AUDIO_BUFFER_LENGTH * 2);
#endif

    /* ---------- LCD ------------*/
    if (IPPU.RenderThisFrame) {
      if (just_saved) {
        uint16_t *scr = (uint16_t *)GFX.Screen;
        uint16_t col = just_saved == 1 ? 0b0000011111100000 : 0b1111100000000000;
        for (int x = 230; x < 240; x++)
          for (int y = 12; y < 22; y++) {
            scr[x + y * SNES_WIDTH] = col;
          }
        if (millis() - just_saved_timer > 2000)
          just_saved = 0;
      }

      while (xSemaphoreTake(vsync_event, 0) != pdTRUE)
        ;  // sync to LCD
      xSemaphoreGive(lcdxfer_go);

      if ((void *)GFX.Screen == (void *)fb[0]) {
        GFX.Screen = (uint8_t *)fb[1];
        fb_back = (uint16_t *)fb[0];
      } else {
        GFX.Screen = (uint8_t *)fb[0];
        fb_back = (uint16_t *)fb[1];
      }

      GFX.SubScreen = GFX.Screen;
    }

#ifdef ENABLE_FRAMEDROPPING
    frame++;
    if ((frame == framedrop_no) && framedrop_no) {
      frame = 0;
      IPPU.RenderThisFrame = false;
    } else {
      IPPU.RenderThisFrame = true;
    }
#endif

    /* ---------- LCD END------------*/

    frame_no++;
    if (millis() - fps_timer > 1000) {
      ESP_LOGI(TAG, "fps: %d", (int)(frame_no * 1000 / (millis() - fps_timer)));
      frame_no = 0;
      fps_timer = millis();
    }
  }
}
