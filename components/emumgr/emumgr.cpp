/*
 * emumgr - Emulation manager for Snes9x on the Pico Held 2
 *
 * Copyright (C) 2025 Daniel Kammer (daniel.kammer@web.de)
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

#include <string>
#include <initializer_list>

/* Pico Held hw library */
#include <picoengine.h>
#include "hwcfg.h"

#include "emumgr_hal.h"
#include "emumgr.h"
//#include "emumgr_lang_de.h"
#include "emumgr_lang_en.h"
#include "heapstrings.h"

// assets
#include "assets/rect.h"
#include "assets/scroll_bar.h"
#include "assets/battery.h"
#include "assets/thumbnail.h"
#include <fonts/fontfiles/picoheld_pixel_5x7_rosa.h>

extern "C" {
/* Snes9x */
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

#include <utility>  // debug printf

#ifdef CAP_FLASH_ROM
//#define HEADER_ROM
#ifdef HEADER_ROM
#include "../../rom_super_mario_world.h"
#endif
#endif

#define DEBUG_OUTPUT

#define ROM_NOT_LOADED (0)
#define ROM_IN_MEM (1)      // in memory but not loaded by the emulator
#define ROM_LOADED (2)      // loaded by the emulator

template<typename... Args>
void em_printf(const char* format, Args&&... args) {
#ifdef DEBUG_OUTPUT
  std::printf(format, std::forward<Args>(args)...);
#endif
}

uint32_t millis();
uint32_t micros();

/* ====================================================== variables ====================================================== */
// misc
int menu_first_run = 1;

emulator_state_t emulator_state;
//void* psram_rom;
int translucency = 50;
uint32_t brownout_det_timer = 0;
bool brownout_det = false;
bool in_menu = false;
int rom_loaded = ROM_NOT_LOADED;  // it means, there is no ROM data @ Memory.ROM yet

// starfield
#define NUM_STARS (50)
uint8_t* stars_x;
uint8_t* stars_y;
uint8_t* stars_color;
uint8_t* stars_speed;
uint32_t stars_timer;

// battery indicator
gbuffer_t draw_battery_gbuf;
int battery_last_level = -1;
int supply_type_last = -1;

/* ====================================================== end variables ====================================================== */
#define MENU_FONT picoheld_pixel_5x7_rosa

#define MENU_FONT_SHADOW_COLOR rgb_col_888_565(30, 30, 30)
#define MENU_FONT_SELECTED_COLOR rgb_col_888_565(230, 0, 200)
#define MENU_FONT_UNSELECTED_COLOR rgb_col_888_565(200, 200, 200)
#define MENU_FONT_CAPTION_COLOR rgb_col_888_565(200, 200, 200)

#define MENU_REPEAT_START (500)
#define MENU_REPEAT_DELAY (50)

// min clearance from screen border
#define MENU_MIN_X_CLEAR (30)  
#define MENU_MIN_Y_CLEAR (20)

#define MESSAGE_DELAY_LONG (2000)
#define MESSAGE_DELAY_SHORT (1000)
#define MESSAGE_DELAY_BLINKOFANEYE (750)

#define FILE_READ_CHUNK_SIZE (2048)

#define DRAW_BATTERY_X (219)
#define DRAW_BATTERY_Y (10)
#define BAT_CRIT_BLINK_DURATION (500)
#define BROWNOUT_DELAY (120000)  // time to shutdown system

#define SAVESTATE_DIRECTORY "roms/snes/save/"
#define ROM_DIRECTORY "roms/snes/"
#define MAX_NUM_SAVESTATES (6)

#define SUCCESS 0
#define ERR_UNSPEC -1
#define ERR_SD_OPEN_DIR -2
#define ERR_SD_OPEN_FIL -3
#define ERR_SD_READING -4
#define ERR_SD_WRITING -5
#define ERR_LOAD_ROM -6
#define ERR_PSRAM -7
#define ERR_ABORTED - 8
#define ERR_NO_ROMS_FOUND -9

#define MENU_LST "\xff"

const char str_gamestate_saved[] = { STR_GAMESTATE_SAVED"\0"\
                                     MENU_LST };

const char str_err_sd_mount[] = { STR_ERROR"\0"\
                                  STR_ERR_SD_MOUNT"\0"\
                                  MENU_LST };

const char str_err_open_file[] = { STR_ERROR"\0"\
                                   STR_ERR_OPEN_FILE"\0"\
                                   MENU_LST };

const char str_err_open_dir[] = { STR_ERROR"\0"\
                                  STR_ERR_OPEN_DIR"\0"\
                                   " \0"\
                                  ROM_DIRECTORY"\0"\
                                  MENU_LST };

const char str_err_read_file[] = { STR_ERROR"\0"\
                                   STR_ERR_OPEN_FILE"\0"\
                                   MENU_LST };

const char str_err_no_roms_found[] = { STR_ERROR"\0"\
                                       STR_ERR_NO_ROMS_FOUND"\0"\
                                       " \0"\
                                       ROM_DIRECTORY"\0"\
                                       MENU_LST };
                                   
const char str_err_write_file[] = { STR_ERROR"\0"\
                                    STR_ERR_OPEN_FILE"\0"\
                                    MENU_LST };

const char* main_menu_items = { STR_MAIN_TITLE"\0"\
					 									     STR_LAUNCH"\0"\
                                 STR_LOAD_STATE"\0"\
                                 STR_SAVE_STATE"\0"\
                                 STR_SOUND"\0"\
                                 STR_BRIGHTNESS"\0"\
                                 STR_SEL_ROM"\0"\
                                 STR_OPTIONS"\0"\
                                 STR_ABOUT"\0"\
                                 STR_SHUTDOWN"\0"\
                                 MENU_LST };

#ifdef CAP_VSYNC
const char settings_menu_items[] = { STR_OPTIONS"\0"\
					 									         STR_FPS"\0"\
                                     STR_REGION"\0"\
                                     STR_BAT"\0"\
                                     STR_SUBSCREEN_MEM "\0"\
                                     STR_VSYNC"\0"\
                                     STR_GO_BACK"\0"\
                                     MENU_LST };
#endif
#ifdef CAP_MANUAL_FRAMEDROP
const char settings_menu_items[] = { STR_OPTIONS"\0"\
					 									         STR_FPS"\0"\
                                     STR_REGION"\0"\
                                     STR_BAT"\0"\
                                     STR_SUBSCREEN_MEM "\0"\
                                     STR_FRAMEDROP_MODE "\0"\
                                     STR_GO_BACK"\0"\
                                     MENU_LST };
#endif

const char save_menu[] = { STR_SAVE_STATE"\0"\
                           MENU_LST };

const char load_menu[] = { STR_LOAD_STATE"\0"\
                           MENU_LST };

const char about[] = { STR_ABOUT_TEXT"\0"\
                       STR_VERSION "_" __DATE__ "_" __TIME__"\0"\
                       MENU_LST };

const char str_err_rom[] = {  STR_ERROR"\0"\
                              STR_ERR_ROM"\0"\
                              MENU_LST };

const char* bri_menu = STR_BRIGHTNESS;

const char* snd_menu = STR_SOUND;

const char* rom_directory = ROM_DIRECTORY;

const char* str_sel_rom = STR_SEL_ROM;

const char* str_subscreen_mem =  STR_SUBSCREEN_MEM;

const char* str_subscreen_psram =  STR_SUBSCREEN_PSRAM;

const char* str_subscreen_shared =  STR_SUBSCREEN_SHARED;

const char* str_vsync = STR_VSYNC;

const char str_loading[] = {  STR_LOADING"\0"\
                              MENU_LST };

const char str_go_back[] = {  STR_GO_BACK"\0"\
                              MENU_LST };

const char str_back[] = {  STR_BACK"\0"\
                              MENU_LST };

const char str_load_game_first[] = { STR_ERR_GAME_FIRST"\0"\
                                     MENU_LST };

int stars_init() {
  if (stars_x == NULL) {
    stars_x = (uint8_t*) em_psram_malloc(NUM_STARS);
    stars_y = (uint8_t*) em_psram_malloc(NUM_STARS);
    stars_speed = (uint8_t*) em_psram_malloc(NUM_STARS);
    stars_color = (uint8_t*) em_psram_malloc(NUM_STARS);

    if (!stars_x || !stars_y || !stars_color || !stars_speed) {
      em_printf("Error: Cannot allocate PSRAM\n");
      return ERR_UNSPEC;
    }

    for (int i = 0; i < NUM_STARS; i++) {
      stars_x[i] = rand() % 240;
      stars_y[i] = rand() % em_lcd_get_height();
      stars_speed[i] = (rand() % 3) + 1;
    }
  }

  return 0;
}

void stars_draw(gbuffer_t fb) {
  if (millis() - stars_timer < 30)
    return;

  stars_timer = millis();

  for (int i = 0; i < NUM_STARS; i++) {
    stars_x[i] += stars_speed[i];
    if (stars_x[i] > 247)
      stars_x[i] = 0;
  }

  color_t stars_color;
  color_t col[3];
  col[0] = rgb_col_888_565(100, 100, 100);
  col[1] = rgb_col_888_565(175, 175, 175);
  col[2] = rgb_col_888_565(255, 255, 255);

  uint16_t* gbuf = (uint16_t*) gbuf_get_dat_ptr(fb);
  int width = gbuf_get_width(fb);

  for (int i = 0; i < NUM_STARS; i++) {
      switch (stars_speed[i]) {
        case 1:
          stars_color = col[0];
          break;
        case 2:
          stars_color = col[1];
          break;
        case 3:
          stars_color = col[2];
          break;
      }
    //draw_pixel(stars_x[i], stars_y[i], stars_color, fb);
    gbuf[stars_x[i] + stars_y[i] * width] = stars_color;
  }
}

void draw_battery(int x, int y, int level, gbuffer_t fb_back) {
#if 0
  font_t* menu_font = (font_t*) MENU_FONT;
    char temp[10];
    pwr_get_source();
    itoa(pwr_get_bat_vol(), temp, 10);
    font_write_string(50, 0, 0xffff, (char*) temp, menu_font, fb_back);
    itoa(pwr_get_src_vol(), temp, 10);
    font_write_string(50, 15, 0xffff, (char*) temp, menu_font, fb_back);
    
    itoa(pwr_get_vbat_raw(), temp, 10);
    font_write_string(100, 0, 0xffff, (char*) temp, menu_font, fb_back);
    itoa(pwr_get_vext_raw(), temp, 10);
    font_write_string(100, 15, 0xffff, (char*) temp, menu_font, fb_back);

    itoa(pwr_get_bat_level(100), temp, 10);

    font_write_string(100, 30, 0xffff, (char*) temp, menu_font, fb_back);
    itoa(pwr_get_bat_level(4), temp, 10);
    font_write_string(50, 30, 0xffff, (char*) temp, menu_font, fb_back);

  return;
#endif

#if defined(CAP_BATTERY) && !defined(DEVBOARD)
  if (pwr_get_source() != PWR_ON_GRD) {
      if ((pwr_get_bat_vol() <= (PWR_BAT_MIN_VOL + 50)) && !brownout_det) {
      brownout_det_timer = millis();
      brownout_det = true;
    }

    if (brownout_det && (millis() - brownout_det_timer > BROWNOUT_DELAY))
      pwr_shutdown();

    if (brownout_det && (((millis() - brownout_det_timer ) % BAT_CRIT_BLINK_DURATION) > (BAT_CRIT_BLINK_DURATION / 2)))
      return;
  } else {
    brownout_det = false;
  }

  if (!emulator_state.show_bat && !brownout_det && !in_menu)
    return;

  if (gbuf_get_dat_ptr(draw_battery_gbuf) == NULL) {
    if (gbuf_create(&draw_battery_gbuf, gbuf_get_width(battery_normal_image), gbuf_get_height(battery_normal_image), (void*) em_psram_malloc(gbuf_get_width(battery_normal_image) * gbuf_get_height(battery_normal_image) * 2)) != BUF_SUCCESS) {
      em_printf("Error: Cannot allocate PSRAM\n");
      return;
    }       
  }

  if (level > 3)
    level = 3;

  if ((battery_last_level != level) || pwr_get_source() != supply_type_last) {
    draw_rect_fill(0, 0, gbuf_get_width(draw_battery_gbuf), gbuf_get_height(draw_battery_gbuf), 0xf81f, draw_battery_gbuf);
    battery_last_level = level;
    supply_type_last = pwr_get_source();
    if (pwr_get_source() == PWR_ON_GRD) {
      blit_buf(0, 0, 0xf81f, battery_normal_image, draw_battery_gbuf);
      blit_buf(5, 3, 0xf81f, flash_image, draw_battery_gbuf);
    } else {
      if (level) {
        blit_buf(0, 0, 0xf81f, battery_normal_image, draw_battery_gbuf);
        for (int i = 0; i < level; i++) {
          draw_rect_fill(17 - i * 5, 3, 17 - i * 5 - 3, 8, rgb_col_888_565(200, 200, 200), draw_battery_gbuf);
        }
      } else {
        blit_buf(0, 0, 0xf81f, battery_empty_image, draw_battery_gbuf);
      }
    }

  } else {
#ifdef ENABLE_PSRAM_FB
    return;
#endif      
  }

  if (brownout_det)
    blit_buf_blend(x, y, 0xf81f, 80, draw_battery_gbuf, fb_back);
  else
    blit_buf_blend(x, y, 0xf81f, 40, draw_battery_gbuf, fb_back);
#endif
}

int read_image_to_gbuf(char* filename, gbuffer_t fb) {
  int ret = SUCCESS;

  bool fr = em_f_open(filename, EM_FA_R);

  if (!fr) {
      em_printf("read_image_to_gbuf: open file (%s) failed: %d\n", filename, (int) fr);
      return ERR_SD_OPEN_FIL;
  }

  uint32_t pos = 0;
  uint8_t* buf = (uint8_t*) gbuf_get_dat_ptr(fb);

  uint32_t br;
  uint32_t size = SNES_WIDTH / 2 * SNES_HEIGHT / 2 * 2;

  fr = em_f_read(buf, size, &br);
  if (!fr || br != size) {
    em_printf("Read error occured. FR is: %d\n", fr);
    ret = ERR_SD_READING;
  }    

  em_f_close();

  return ret;
  
}

static bool screenshot_handler(const char *filename, int width, int height)
{
    //return rg_display_save_frame(filename, currentUpdate, width, height);
    return true;
}

int save_state_handler(char *filename)
{
  int ret = SUCCESS;
  bool fr;

  em_printf("Savestate handler called: %s\n", filename);

  fr = em_f_open(filename, EM_FA_RWC);

  if (!fr) {
    em_printf("savestate handler: open file failed: %d\n", (int) fr);
    return ERR_SD_OPEN_FIL;  // error opening
  }

  uint32_t bw;

  fr = em_f_write((uint8_t*) &CPU, sizeof(CPU), &bw);
  if (!fr || (bw != sizeof(CPU))) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) &ICPU, sizeof(ICPU), &bw);
  if (!fr || (bw != sizeof(ICPU))) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) &PPU, sizeof(PPU), &bw);
  if (!fr || (bw != sizeof(PPU))) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) &DMA, sizeof(DMA), &bw);
  if (!fr || (bw != sizeof(DMA))) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) Memory.VRAM, VRAM_SIZE, &bw);
  if (!fr || (bw != VRAM_SIZE)) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) Memory.RAM, RAM_SIZE, &bw);
  if (!fr || (bw != RAM_SIZE)) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) Memory.SRAM, SRAM_SIZE, &bw);
  if (!fr || (bw != SRAM_SIZE)) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) Memory.FillRAM, FILLRAM_SIZE, &bw);
  if (!fr || (bw != FILLRAM_SIZE)) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) &APU, sizeof(APU), &bw);
  if (!fr || (bw != sizeof(APU))) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) &IAPU, sizeof(IAPU), &bw);
  if (!fr || (bw != sizeof(IAPU))) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) IAPU.RAM, 0x10000, &bw);
  if (!fr || (bw != 0x10000)) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }
  fr = em_f_write((uint8_t*) &SoundData, sizeof(SoundData), &bw);
  if (!fr || (bw != sizeof(SoundData))) {
    ret = ERR_SD_WRITING;
    goto bailout;
  }

  em_printf("Game state saved.\n\n");

  bailout:
  
  em_f_close();

  return ret;
}

int screenshot_saver(char *filename, gbuffer_t fb)
{
  int ret = SUCCESS;

  em_printf("Screenshot handler called: %s\n", filename);

  bool fr;
  uint32_t bw;

  uint32_t filesize = SNES_WIDTH / 2 * SNES_HEIGHT / 2 * 2;
  uint32_t pos = 0;
  uint16_t* buf_src = (uint16_t*) gbuf_get_dat_ptr(fb);
  uint16_t* buf_dst;
  uint32_t size;
  
  fr = em_f_open(filename, EM_FA_RWC);

  if (!fr) {
    em_printf("screenshot_save: open file failed: %d\n", (int) fr);
    return ERR_SD_OPEN_FIL;  // error opening
  }

  buf_dst = (uint16_t*) em_psram_malloc(SNES_WIDTH / 2 * SNES_HEIGHT / 2 * 2);
  if (!buf_dst) {
    ret = ERR_PSRAM;
    goto bailout;
  }

  #if 0
  // bilinear interpolation
  for (int y = SNES_HEIGHT / 2 - 1; y < SNES_HEIGHT / 2; y++)
    for (int x = SNES_WIDTH / 2 - 1; x < SNES_WIDTH / 2; x++)
      buf_dst[x + y * SNES_WIDTH / 2] = 0;

  for (int y = 0; y < SNES_HEIGHT / 2 - 1; y++)
    for (int x = 0; x < SNES_WIDTH / 2 - 1; x++) {
          uint8_t r1 = (buf_src[x * 2 + y * 2 * SNES_WIDTH] >> 11) & 0x1f;
          uint8_t g1 = (buf_src[x * 2 + y * 2 * SNES_WIDTH] >> 5) & 0x3f;
          uint8_t b1 = (buf_src[x * 2 + y * 2 * SNES_WIDTH]) & 0x1f;
          uint8_t r2 = (buf_src[(x + 1) * 2 + y * 2 * SNES_WIDTH] >> 11) & 0x1f;
          uint8_t g2 = (buf_src[(x + 1) * 2 + y * 2 * SNES_WIDTH] >> 5) & 0x3f;
          uint8_t b2 = (buf_src[(x + 1) * 2 + y * 2 * SNES_WIDTH]) & 0x1f;
          uint8_t r3 = (buf_src[x * 2 + (y + 1) * 2 * SNES_WIDTH] >> 11) & 0x1f;
          uint8_t g3 = (buf_src[x * 2 + (y + 1) * 2 * SNES_WIDTH] >> 5) & 0x3f;
          uint8_t b3 = (buf_src[x * 2 + (y + 1) * 2 * SNES_WIDTH]) & 0x1f;
          uint8_t r4 = (buf_src[(x + 1) * 2 + (y + 1) * 2 * SNES_WIDTH] >> 11) & 0x1f;
          uint8_t g4 = (buf_src[(x + 1) * 2 + (y + 1) * 2 * SNES_WIDTH] >> 5) & 0x3f;
          uint8_t b4 = (buf_src[(x + 1) * 2 + (y + 1) * 2 * SNES_WIDTH]) & 0x1f;
          uint16_t a = ((r1 + r2 + r3 + r4) / 4) << 11 |
                       ((g1 + g2 + g3 + g4) / 4) << 5 |
                        (b1 + b2 + b3 + b4) / 4;
          // suppress chroma key
          if (a == 0xf81f)
            0xf81e;
          buf_dst[x + y * SNES_WIDTH / 2] = a;
  }
  #else
  for (int y = 0; y < SNES_HEIGHT / 2; y++)
    for (int x = 0; x < SNES_WIDTH / 2; x++) {
      uint16_t a = buf_src[x * 2 + y * 2 * SNES_WIDTH];
      // suppress chroma key
      if (a == 0xf81f)
        a = 0xf81e;
      buf_dst[x + y * SNES_WIDTH / 2] = a;
    }
  #endif

  size = SNES_WIDTH / 2 * SNES_HEIGHT / 2 * 2;
  fr = em_f_write((uint8_t*) buf_dst, size, &bw);
  if (!fr || (bw != size)) {
    em_printf("Error writing screenshot data.\n");
    ret = ERR_SD_WRITING;
    goto bailout;
  }

  em_printf("Screenshot saved.\n\n");

  bailout:

  em_f_close();

  if (buf_dst)
    em_psram_free(buf_dst);
  
  return ret;
}

int load_state_handler(char *filename)
{
  int ret = SUCCESS;
  uint32_t br;
  bool fr;

  em_printf("Loadstate handler called. File: %s..\n", filename);

  fr = em_f_open(filename, EM_FA_R);

  if (!fr) {
    em_printf("loadstate handler: open failed: %d\n", (int) fr);
    return ERR_SD_OPEN_FIL;
  }

  S9xReset();

  // At this point we can't go back and a failure will corrupt the state anyway
  em_printf("Now reading to memory.\n");

  fr = em_f_read((uint8_t*) &CPU, sizeof(CPU), &br);
  if (!fr || (br != sizeof(CPU))) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) &ICPU, sizeof(ICPU), &br);
  if (!fr || (br != sizeof(ICPU))) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) &PPU, sizeof(PPU), &br);
  if (!fr || (br != sizeof(PPU))) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) &DMA, sizeof(DMA), &br);
  if (!fr || (br != sizeof(DMA))) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) Memory.VRAM, VRAM_SIZE, &br);
  if (!fr || (br != VRAM_SIZE)) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) Memory.RAM, RAM_SIZE, &br);
  if (!fr || (br != RAM_SIZE)) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) Memory.SRAM, SRAM_SIZE, &br);
  if (!fr || (br != SRAM_SIZE)) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) Memory.FillRAM, FILLRAM_SIZE, &br);
  if (!fr || (br != FILLRAM_SIZE)) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) &APU, sizeof(APU), &br);
  if (!fr || (br != sizeof(APU))) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) &IAPU, sizeof(IAPU), &br);
  if (!fr || (br != sizeof(IAPU))) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) IAPU.RAM, 0x10000, &br);
  if (!fr || (br != 0x10000)) {
    ret = ERR_SD_READING;
    goto bailout;
  }
  fr = em_f_read((uint8_t*) &SoundData, sizeof(SoundData), &br);
  if (!fr || br != sizeof(SoundData)) {
    ret = ERR_SD_READING;
    goto bailout;
  }

  S9xLoadState();

  em_printf("Game state loaded.\n");

bailout:

  em_f_close();

  //return S9xLoadState();
  return ret;
}

//================================================================================================================================================================
#if 0

int calc_rom_checksum(uint8_t* rom_addr) {
  //todo
  return true;
}

int check_rom_valid(uint8_t* rom_addr) {
/*
  const char system_id_val1[16] = "SEGA MEGA DRIVE";
  const char system_id_val2[13] = "SEGA GENESIS";
  char system_id[16];

  memcpy(system_id, (void*)((uint32_t)rom_addr), 15);
  system_id[15] = 0;

  if (strcmp(system_id_val1, system_id))
    if (calc_rom_checksum(rom_addr))
      return true;

  system_id[12] = 0;

  if (strcmp(system_id_val2, system_id))
    if (calc_rom_checksum(rom_addr))
      return true;

  return false;
*/
  return true;
}

  // reset machine
  // let's just hope noone messed with the WD settings...
  watchdog_reboot(0, 0, 1);
  watchdog_enable(1, false);

  #endif


/* ----------------------- main function -----------------------*/
int draw_rectangle(coord_t x, coord_t y, coord_t width, coord_t height, int translucency, gbuffer_t fb) {
  gbuffer_t fb_tmp;

  if (gbuf_create(&fb_tmp, width, height, (void*) em_psram_malloc(width * height * 2)) != BUF_SUCCESS) {
    em_printf("Error: Cannot allocate PSRAM\n");
    return ERR_PSRAM;
  }

  draw_rect_fill(0,
                 0,
                 width - 1,
                 height - 1,
                 0x2124,
                 fb_tmp);
  
  draw_line(gbuf_get_width(corner_top_left), 0, width - gbuf_get_width(corner_top_left) - 1, 0, 0xa514, fb_tmp);
  draw_line(0, gbuf_get_height(corner_top_left), 0, height - gbuf_get_height(corner_bottom_left), 0xa514, fb_tmp);
  draw_line(width - 1, gbuf_get_height(corner_top_left), width - 1, height - gbuf_get_height(corner_bottom_left), 0x0000, fb_tmp);
  draw_line(gbuf_get_width(corner_top_left), height - 1, width - gbuf_get_width(corner_top_left) - 1, height - 1, 0x0000, fb_tmp);

  blit_buf(0, 0, BLIT_NO_ALPHA, corner_top_left, fb_tmp);
  blit_buf(0, height - gbuf_get_height(corner_bottom_left), BLIT_NO_ALPHA, corner_bottom_left, fb_tmp);
  blit_buf(width - gbuf_get_width(corner_top_right), 0, BLIT_NO_ALPHA, corner_top_right, fb_tmp);
  blit_buf(width - gbuf_get_width(corner_bottom_right), height - gbuf_get_height(corner_bottom_right), BLIT_NO_ALPHA, corner_bottom_right, fb_tmp);
  
  blit_buf_blend(x, y, 0xf81f, translucency, fb_tmp, fb);
  
  em_psram_free(gbuf_get_dat_ptr(fb_tmp));
  return 0;

}

int progress_bar(const char* caption, int value, gbuffer_t fb) {
  if ((value < 0) || (value > 100))
    return ERR_UNSPEC;
  
  gbuffer_t fb_back;
  em_lcd_refresh();
  em_get_backbuffer(&fb_back);

  color_t menu_font_shadow_color = MENU_FONT_SHADOW_COLOR;
  color_t menu_font_caption_color = MENU_FONT_CAPTION_COLOR;

  font_t* menu_font = (font_t*) MENU_FONT;

  int menu_border_width = gbuf_get_width(corner_top_left);
  int menu_border_height = gbuf_get_height(corner_top_left);

  int menu_offset_x = 50;

  int menu_width = gbuf_get_width(fb) - 2 * menu_offset_x;
  int bar_height = gbuf_get_height(bar_mid_empty_image);
  int menu_height = font_get_height(menu_font) + bar_height + 5;
  int bar_offset_y = 2;
  int font_offset_y = 4;

  int menu_offset_y = (gbuf_get_height(fb) - menu_height) / 2;

  blit_buf(0, 0, BLIT_NO_ALPHA, fb, fb_back);
  if (menu_first_run)
    stars_draw(fb_back);

  draw_rectangle(menu_offset_x - menu_border_width,
                  menu_offset_y - menu_border_height,
                  menu_width + menu_border_width * 2,
                  menu_height + menu_border_height * 2,
                  translucency,
                  fb_back);

  font_write_string_centered(1 + gbuf_get_width(fb_back) / 2 , 1 + gbuf_get_height(fb_back) / 2 - font_get_height(menu_font) - font_offset_y, menu_font_shadow_color, (char*) caption, menu_font, fb_back);
  font_write_string_centered(gbuf_get_width(fb_back) / 2 , gbuf_get_height(fb_back) / 2 - font_get_height(menu_font) - font_offset_y, menu_font_caption_color, (char*) caption, menu_font, fb_back);

  int bar_start = menu_offset_x + 10 + gbuf_get_width(bar_left_full_image);
  int bar_end = menu_offset_x + 10 + menu_width - 20 - gbuf_get_width(bar_right_full_image);
    
  for (int i = 0; i < bar_end - bar_start; i++) {
    if (i * 100 / (bar_end - bar_start) > value)
      blit_buf(bar_start + i, gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_mid_empty_image, fb_back);  
    else
      blit_buf(bar_start + i, gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_mid_full_image, fb_back);  
  }

  blit_buf(menu_offset_x + 10 + (menu_width - 20 - gbuf_get_width(bar_right_full_image)), gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_right_empty_image, fb_back);
  blit_buf(menu_offset_x + 10, gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_left_full_image, fb_back);
  blit_buf(menu_offset_x + 10 + gbuf_get_width(bar_right_full_image) + (menu_width - 20 - 2 * gbuf_get_width(bar_right_full_image)) * value / 100, gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_right_full_image, fb_back);

  draw_battery(DRAW_BATTERY_X, DRAW_BATTERY_Y, pwr_get_bat_level(4), fb_back);

  em_request_pageflip();

  return 0;

}

int load_rom_from_sd(char* filename, gbuffer_t fb) {
  int ret = 0;

  int step = 0;
  int percent = 0; 
  uint32_t readlen = FILE_READ_CHUNK_SIZE;

  bool fr = em_f_open(filename, EM_FA_R);

  if (!fr) {
      em_printf("open file failed\n");
      return ERR_SD_OPEN_FIL;
  }
  
  em_printf("open complete\n");

  Memory.ROM = Memory.PSRAM;

  uint32_t filesize = em_f_size();
  uint8_t* buf = (uint8_t*) Memory.ROM;
  uint32_t unread_bytes = filesize;

  Memory.ROM_Size = filesize;

  while (unread_bytes) {
    uint32_t bytes_read;
    if (unread_bytes < readlen)
      readlen = unread_bytes;
    fr = em_f_read(buf, readlen, &bytes_read);
    buf += bytes_read;
    unread_bytes -= bytes_read;
    if (!fr && unread_bytes) {
      em_printf("Read error occured, still %d unread bytes. FR is: %d\n", (int) unread_bytes, fr);
      ret = ERR_SD_READING;
      goto bailout;
    }
    
    percent = 100 - unread_bytes * 100 / filesize;
    if (percent >= step) {
      step += 5;  // smoothness
      //em_printf("Loading from SD: %d %\n", percent);
      progress_bar(str_loading, percent, fb);
    }
  }

  em_printf("successfully read %d bytes from file %s\n", (int) filesize, filename);
  
bailout:

  em_f_close();
  
  return ret;
}

int slider_menu(char* caption, int start_val, int step, int brightness, gbuffer_t fb) {
  gbuffer_t fb_back;

  color_t menu_font_shadow_color = MENU_FONT_SHADOW_COLOR;
  color_t menu_font_caption_color = MENU_FONT_CAPTION_COLOR;

  font_t* menu_font = (font_t*) MENU_FONT;

  int menu_border_width = gbuf_get_width(corner_top_left);
  int menu_border_height = gbuf_get_height(corner_top_left);

  int menu_offset_x = 50;

  int menu_width = gbuf_get_width(fb) - 2 * menu_offset_x;
  int bar_height = gbuf_get_height(bar_mid_empty_image);
  int menu_height = font_get_height(menu_font) + bar_height + 5;

  int menu_offset_y = (gbuf_get_height(fb) - menu_height) / 2;

  int b_debounce = 0;
  int b_debounce_timer = 0;
  int b_repeater = 0;

  int value = start_val;
  int first_run = 1;
  int buttons_locked = 1;

  int bar_offset_y = 2;
  int font_offset_y = 4;

  if ((step < 1) || (step > 50))
    step = 10;

  while (1) {
    em_lcd_refresh();
    em_get_backbuffer(&fb_back);
    blit_buf(0, 0, BLIT_NO_ALPHA, fb, fb_back);
    if (menu_first_run)
      stars_draw(fb_back);

    draw_rectangle(menu_offset_x - menu_border_width,
                   menu_offset_y - menu_border_height,
                   menu_width + menu_border_width * 2,
                   menu_height + menu_border_height * 2,
                   translucency,
                   fb_back);

    font_write_string_centered(1 + gbuf_get_width(fb_back) / 2 , 1 + gbuf_get_height(fb_back) / 2 - font_get_height(menu_font) - font_offset_y, menu_font_shadow_color, (char*) caption, menu_font, fb_back);
    font_write_string_centered(gbuf_get_width(fb_back) / 2 , gbuf_get_height(fb_back) / 2 - font_get_height(menu_font) - font_offset_y, menu_font_caption_color, (char*) caption, menu_font, fb_back);

    int bar_start = menu_offset_x + 10 + gbuf_get_width(bar_left_full_image);
    int bar_end = menu_offset_x + 10 + menu_width - 20 - gbuf_get_width(bar_right_full_image);
    
    for (int i = 0; i < bar_end - bar_start; i++) {
      if (i * 100 / (bar_end - bar_start) > value)
        blit_buf(bar_start + i, gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_mid_empty_image, fb_back);  
      else
        blit_buf(bar_start + i, gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_mid_full_image, fb_back);  
    }

    blit_buf(menu_offset_x + 10 + (menu_width - 20 - gbuf_get_width(bar_right_full_image)), gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_right_empty_image, fb_back);
    blit_buf(menu_offset_x + 10, gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_left_full_image, fb_back);
    blit_buf(menu_offset_x + 10 + gbuf_get_width(bar_right_full_image) + (menu_width - 20 - 2 * gbuf_get_width(bar_right_full_image)) * value / 100, gbuf_get_height(fb_back) / 2 + bar_offset_y, 0xf81f, bar_right_full_image, fb_back);

    draw_battery(DRAW_BATTERY_X, DRAW_BATTERY_Y, pwr_get_bat_level(4), fb_back);
    
    em_request_pageflip();

    em_ctrl_button_state_update();

    if (first_run && !em_ctrl_is_pressed(BUTTON_ANY)) {
          buttons_locked = 0;
          first_run = 0;
    }

    if (!buttons_locked) {
      if (em_ctrl_is_pressed(BUTTON_LEFT) && !b_debounce)
        value -= step;
      
      if (em_ctrl_is_pressed(BUTTON_RIGHT) && !b_debounce)
        value += step;
  
      if (value < 0)
        value = 0;

      if (value > 100)
        value = 100;

      if (brightness && value > 0)
        em_lcd_backlight_set(value);

      if ( em_ctrl_is_pressed(BUTTON_A) ||
           em_ctrl_is_pressed(BUTTON_B) ||
           em_ctrl_is_pressed(BUTTON_X) ||
           em_ctrl_is_pressed(BUTTON_Y) ||
           em_ctrl_is_pressed(BUTTON_L) ||
           em_ctrl_is_pressed(BUTTON_R) ||
           em_ctrl_is_pressed(BUTTON_ST) ||
           em_ctrl_is_pressed(BUTTON_SEL) ||
           em_ctrl_is_pressed(BUTTON_SW))
        return value;

      if (!em_ctrl_is_pressed(BUTTON_ANY)) {
        b_debounce = 0;
        b_repeater = 0;
      } else {
        if (b_debounce == 0) {
          b_debounce = 1;
          b_debounce_timer = millis();
        } else if ((millis() - b_debounce_timer > MENU_REPEAT_START) && !b_repeater) {
          b_repeater = 1;
          b_debounce = 0;
          b_debounce_timer = millis();         
        } else if ((millis() - b_debounce_timer > MENU_REPEAT_DELAY) && b_repeater) {
          b_debounce = 0;
          b_debounce_timer = millis();         
        }
      }
    }

  }

}

int selection_menu(char* item_list, bool centered, bool no_highlight, bool no_caption, int delay, gbuffer_t fb) {
  gbuffer_t fb_back;

  color_t menu_font_shadow_color = MENU_FONT_SHADOW_COLOR;
  color_t menu_font_selected_color = no_highlight ? MENU_FONT_UNSELECTED_COLOR : MENU_FONT_SELECTED_COLOR;
  color_t menu_font_unselected_color = MENU_FONT_UNSELECTED_COLOR;
  color_t menu_font_caption_color = MENU_FONT_CAPTION_COLOR;

  float menu_caption_offset = no_caption ? 0.4 : 1.2;
  
  font_t* menu_font = (font_t*) MENU_FONT;
  int menu_items_clearance_y = 5;
  int num_itms = get_item_cnt(item_list) - 1;  // minus the caption

  int menu_border_width = gbuf_get_width(corner_top_left);
  int menu_border_height =  gbuf_get_height(corner_top_left);
  int menu_scrollbar_width = 3 + gbuf_get_width(bar_up_full_image);

  // calculate menu height (depending on items to be shown)
  int menu_height = menu_caption_offset * menu_items_clearance_y - menu_items_clearance_y;  // difference for clearance of caption
  for (int i = 0; i < get_item_cnt(item_list); i++) {
    menu_height += font_get_height(menu_font) + menu_items_clearance_y;
    if (menu_height > (gbuf_get_height(fb) - MENU_MIN_Y_CLEAR * 2 - menu_border_height * 2)) {
      menu_height -= font_get_height(menu_font) + menu_items_clearance_y;
      break;
    }
  }

  int max_vis_items = (menu_height - font_get_height(menu_font) - menu_caption_offset) / (font_get_height(menu_font) + menu_items_clearance_y);

  menu_height -= menu_items_clearance_y;

  // calculate menu width (depending on item width)
  int menu_width = 0;
  if (max_vis_items < num_itms)
    menu_width = menu_scrollbar_width * 2;
  else
    menu_scrollbar_width = 0;

  for (int i = 0; i < get_item_cnt(item_list); i++) {
    int len = font_get_string_width(get_item(i, item_list), menu_font);
    if (len > menu_width)
      menu_width = len;
    if (menu_width > (gbuf_get_width(fb) - MENU_MIN_X_CLEAR * 2 - menu_border_width * 2)) {
      menu_width = gbuf_get_width(fb) - MENU_MIN_X_CLEAR * 2 - menu_border_width * 2;
      break;
    }
  }
  
  int menu_offset_x = (gbuf_get_width(fb) - menu_width) / 2;
  int menu_offset_y = (gbuf_get_height(fb) - menu_height) / 2;

  int b_debounce = 0;
  int b_debounce_timer = 0;
  int b_repeater = 0;

  int cur_itm = 0;
  int ofs_itm = 0;
  if (num_itms < max_vis_items)
    max_vis_items = num_itms;
  int first_run = 1;
  int buttons_locked = 1;

  uint32_t h_font_scrl_timer = millis() + 1000;
  int h_font_scrl_pos = 0;
  int h_font_scrl_dir = 1;
  uint32_t h_font_scrl_delay = 1000;

  bool timed = delay ? true : false;
  uint32_t timer = millis();

  while (!timed || (millis() - timer < delay) ) {
    em_lcd_refresh();
    em_get_backbuffer(&fb_back);
    blit_buf(0, 0, BLIT_NO_ALPHA, fb, fb_back);
    if (menu_first_run)
      stars_draw(fb_back);

    draw_rectangle(menu_offset_x - menu_border_width,
                   menu_offset_y - menu_border_height,
                   menu_width + menu_border_width * 2,
                   menu_height + menu_border_height * 2,
                   translucency,
                   fb_back);
    
    // caption
    if (!no_caption) {
      font_write_string_centered(1 + menu_offset_x + menu_width / 2, 1 + menu_offset_y, menu_font_shadow_color, get_item(0, item_list), menu_font, fb_back);
      font_write_string_centered(menu_offset_x + menu_width / 2, menu_offset_y, menu_font_caption_color, get_item(0, item_list), menu_font, fb_back);
    }
    /*
    draw_line(menu_offset_x + menu_width / 2 - font_get_string_width(get_item(0, item_list), menu_font) / 4,
              menu_offset_y + font_get_height(menu_font) + menu_caption_offset * menu_items_clearance_y / 2,
              menu_offset_x + menu_width / 2 + font_get_string_width(get_item(0, item_list), menu_font) / 4,
              menu_offset_y + font_get_height(menu_font) + menu_caption_offset * menu_items_clearance_y / 2,
              rgb_col_888_565(100, 100, 100),
              fb_back);
    */

    for (int i = 0; i < max_vis_items; i++) {
      
      // scrolling offset
      if (cur_itm >= ofs_itm + max_vis_items)
        ofs_itm++;

      if (cur_itm < ofs_itm)
        ofs_itm--;

      char* text = get_item(i + 1 + ofs_itm, item_list);
      
      if ((i + ofs_itm) == cur_itm) {
        // horizontal scrolling of long entries (e.g. file names)
        int len = font_get_string_width(text, menu_font);
        if (millis() - h_font_scrl_timer > h_font_scrl_delay) {
          h_font_scrl_delay = 30;
          h_font_scrl_timer = millis();
          if (h_font_scrl_dir == 1) {
            if (len + h_font_scrl_pos > (menu_width - menu_scrollbar_width))
              h_font_scrl_pos--;
            else
              h_font_scrl_dir = -1;
          } else {
            if (h_font_scrl_pos < 0)
              h_font_scrl_pos++;
            else
              h_font_scrl_dir = 1;
          }
        }
        if (centered) {
          font_write_string_centered_crop(menu_offset_x + menu_width / 2 + h_font_scrl_pos + 1,
                                          menu_offset_y + 1 + (i + menu_caption_offset) * (font_get_height(menu_font) + menu_items_clearance_y),
                                          menu_offset_x + 1,
                                          menu_offset_x + 1 + menu_width - menu_scrollbar_width,
                                          menu_font_shadow_color,
                                          text,
                                          menu_font,
                                          fb_back);
          font_write_string_centered_crop(menu_offset_x + menu_width / 2 + h_font_scrl_pos + 0,
                                          menu_offset_y + (i + menu_caption_offset) * (font_get_height(menu_font) + menu_items_clearance_y),
                                          menu_offset_x,
                                          menu_offset_x + menu_width - menu_scrollbar_width,
                                          menu_font_selected_color,
                                          text,
                                          menu_font,
                                          fb_back);
        } else {
          font_write_string_crop(menu_offset_x + h_font_scrl_pos + 1,
                                 menu_offset_y + 1 + (i + menu_caption_offset) * (font_get_height(menu_font) + menu_items_clearance_y),
                                 menu_offset_x + 1,
                                 menu_offset_x + 1 + menu_width - menu_scrollbar_width,
                                 menu_font_shadow_color,
                                 text,
                                 menu_font,
                                 fb_back);
          font_write_string_crop(menu_offset_x + h_font_scrl_pos + 0,
                                 menu_offset_y + (i + menu_caption_offset) * (font_get_height(menu_font) + menu_items_clearance_y),
                                 menu_offset_x,
                                 menu_offset_x + menu_width - menu_scrollbar_width,
                                 menu_font_selected_color,
                                 text,
                                 menu_font,
                                 fb_back);
        }
      } else {
        if (centered) {
          font_write_string_centered_crop(menu_offset_x + menu_width / 2 + 1,
                                          menu_offset_y + 1 + (i + menu_caption_offset) * (font_get_height(menu_font) + menu_items_clearance_y),
                                          menu_offset_x + 1,
                                          menu_offset_x + 1 + menu_width - menu_scrollbar_width,
                                          menu_font_shadow_color,
                                          text,
                                          menu_font,
                                          fb_back);
          font_write_string_centered_crop(menu_offset_x + menu_width / 2 + 0,
                                          menu_offset_y + (i + menu_caption_offset) * (font_get_height(menu_font) + menu_items_clearance_y),
                                          menu_offset_x,
                                          menu_offset_x + menu_width - menu_scrollbar_width,
                                          menu_font_unselected_color,
                                          text,
                                          menu_font,
                                          fb_back);
        } else {
          font_write_string_crop(menu_offset_x + 1,
                                 menu_offset_y + 1 + (i + menu_caption_offset) * (font_get_height(menu_font) + menu_items_clearance_y),
                                 menu_offset_x + 1,
                                 menu_offset_x + 1 + menu_width - menu_scrollbar_width,
                                 menu_font_shadow_color,
                                 text,
                                 menu_font,
                                 fb_back);
          font_write_string_crop(menu_offset_x + 0,
                                 menu_offset_y + (i + menu_caption_offset) * (font_get_height(menu_font) + menu_items_clearance_y),
                                 menu_offset_x,
                                 menu_offset_x + menu_width - menu_scrollbar_width,
                                 menu_font_unselected_color,
                                 text,
                                 menu_font,
                                 fb_back);
        }
      }

    }

    // scroll arrows to indicate additional items
    int scrl_up_arrow_y = menu_offset_y + menu_caption_offset * (font_get_height(menu_font) + menu_items_clearance_y);
    int scrl_down_arrow_y = menu_offset_y + menu_height - gbuf_get_height(bar_down_empty_image);;

    int total_range_y = scrl_down_arrow_y - scrl_up_arrow_y - gbuf_get_height(bar_up_full_image);
    float scrl_pcnt =  (float) ofs_itm / (float) (num_itms - max_vis_items);

    if (num_itms > max_vis_items) {
      // When there is a great number of items (like in a directory)
      // use a position indicator

      int scroll_bar_offset = gbuf_get_width(bar_up_full_image) - 2;

#if 1
      for (int i = 0; i < total_range_y; i++) {
        if ((i * 100) / total_range_y < scrl_pcnt * 100)
          blit_buf(menu_offset_x + menu_width - scroll_bar_offset, scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + i, 0xf81f, bar_mid_h_full_image, fb_back);
        else
          blit_buf(menu_offset_x + menu_width - scroll_bar_offset, scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + i, 0xf81f, bar_mid_h_empty_image, fb_back);
      }
#else
      draw_rect_fill(menu_offset_x + menu_width - scroll_bar_offset + 1,
                     scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + 0,
                     menu_offset_x + menu_width - scroll_bar_offset + gbuf_get_width(bar_up_full_image) - 2,
                     (int) (scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + (total_range_y * (scrl_pcnt * 100)) / 100 - 1),
                     0xc638,
                     fb_back);

      // full line left
      draw_line(menu_offset_x + menu_width - scroll_bar_offset,
                scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + 0,
                menu_offset_x + menu_width - scroll_bar_offset,
                (int) (scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + (total_range_y * (scrl_pcnt * 100)) / 100 - 1),
                0xa514,
                fb_back);

      // full line right
      draw_line(menu_offset_x + menu_width - scroll_bar_offset + gbuf_get_width(bar_up_full_image),
                scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + 0,
                menu_offset_x + menu_width - scroll_bar_offset + gbuf_get_width(bar_up_full_image),
                scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + total_range_y * scrl_pcnt - 1,
                0x0000,
                fb_back);

      // empty line left
      draw_line(menu_offset_x + menu_width - scroll_bar_offset,
                scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + total_range_y * scrl_pcnt,
                menu_offset_x + menu_width - scroll_bar_offset,
                scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + total_range_y - 2,
                0x0000,
                fb_back);

      // emptry line right
      draw_line(menu_offset_x + menu_width - scroll_bar_offset + gbuf_get_width(bar_up_full_image) - 1,
                scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + total_range_y * scrl_pcnt,
                menu_offset_x + menu_width - scroll_bar_offset + gbuf_get_width(bar_up_full_image) - 1,
                scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + total_range_y - 2,
                0xa514,
                fb_back);
#endif

      // bar upper end
      blit_buf(menu_offset_x + menu_width - scroll_bar_offset, scrl_up_arrow_y, 0xf81f, bar_up_full_image, fb_back);
      
      // bar lower end
      blit_buf(menu_offset_x + menu_width - scroll_bar_offset, scrl_down_arrow_y, 0xf81f, bar_down_empty_image, fb_back);

      // position indicator
      blit_buf(menu_offset_x + menu_width - scroll_bar_offset, scrl_up_arrow_y + gbuf_get_height(bar_up_full_image) + total_range_y * scrl_pcnt, 0xf81f, bar_down_full_image, fb_back);

    } /* else if (num_itms > max_vis_items) {
      // When there is only a small number of items
      // just use arrows to indicate there are more items you can scroll to
      
      // arrow up
      if (ofs_itm > 0)
        font_put_char(menu_offset_x + menu_width - 10, scrl_up_arrow_y, 0x0, 'A', menu_font, fb_back);

      // arrow down
      if (ofs_itm + max_vis_items < num_itms)
        font_put_char(menu_offset_x + menu_width - 10, scrl_down_arrow_y, 0x0, 'V', menu_font, fb_back);

    } */

    draw_battery(DRAW_BATTERY_X, DRAW_BATTERY_Y, pwr_get_bat_level(4), fb_back);
    
    em_request_pageflip();

    em_ctrl_button_state_update();

    if (first_run && buttons_locked && !em_ctrl_is_pressed(BUTTON_ANY)) {
      first_run = 0;
      buttons_locked = 0;
    }

    if (!buttons_locked) {
      // return selected item
      if (em_ctrl_is_pressed(BUTTON_A) ||
          em_ctrl_is_pressed(BUTTON_B) ||
          em_ctrl_is_pressed(BUTTON_X) ||
          em_ctrl_is_pressed(BUTTON_Y) ||
          em_ctrl_is_pressed(BUTTON_L) ||
          em_ctrl_is_pressed(BUTTON_R) ||
          em_ctrl_is_pressed(BUTTON_ST) ||
          em_ctrl_is_pressed(BUTTON_SEL) ||
          em_ctrl_is_pressed(BUTTON_SW))
        goto leave;

      if (em_ctrl_is_pressed(BUTTON_DOWN) && !b_debounce) {
        cur_itm++;
        
        if (cur_itm > (num_itms - 1))
          cur_itm = 0;

        h_font_scrl_timer = millis();
        h_font_scrl_pos = 0;
        h_font_scrl_dir = 1;
        h_font_scrl_delay = 1000;
      }

      if (em_ctrl_is_pressed(BUTTON_UP) && !b_debounce) {
        cur_itm--;

        if (cur_itm < 0)
          cur_itm = num_itms - 1;  
        
        h_font_scrl_timer = millis();
        h_font_scrl_pos = 0;
        h_font_scrl_dir = 1;
        h_font_scrl_delay = 1000;
      }

      if (!em_ctrl_is_pressed(BUTTON_ANY)) {
        b_debounce = 0;
        b_repeater = 0;
      } else {
        if (b_debounce == 0) {
          b_debounce = 1;
          b_debounce_timer = millis();
        } else if ((millis() - b_debounce_timer > MENU_REPEAT_START) && !b_repeater) {
          b_repeater = 1;
          b_debounce = 0;
          b_debounce_timer = millis();         
        } else if ((millis() - b_debounce_timer > MENU_REPEAT_DELAY) && b_repeater) {
          b_debounce = 0;
          b_debounce_timer = millis();         
        }
      }
    }

  }

leave:

  return cur_itm;

}  // selection_menu

void message(const char* item_list, bool centered, uint32_t delay, gbuffer_t fb) {
  selection_menu((char*) item_list, centered, true, false, delay, fb);
};

int savestate_selector(const char* caption, char* filename, gbuffer_t fb) {
  gbuffer_t fb_back;

  color_t menu_font_shadow_color = MENU_FONT_SHADOW_COLOR;
  color_t menu_font_selected_color = MENU_FONT_SELECTED_COLOR;
  color_t menu_font_unselected_color = MENU_FONT_UNSELECTED_COLOR;
  color_t menu_font_caption_color = MENU_FONT_CAPTION_COLOR;

  font_t* menu_font = (font_t*) MENU_FONT;

  int menu_border_width = gbuf_get_width(corner_top_left);
  int menu_border_height = gbuf_get_height(corner_top_left);

  int clearance_y = 5;
  int menu_height = em_lcd_get_height() / 2 + font_get_height(menu_font) + 2.5 * clearance_y - menu_border_height;  // difference for clearance of caption
  int menu_arrow_gap = 6;
  int menu_width = SNES_WIDTH / 2 + font_get_char_width('>', menu_font) + font_get_char_width('<', menu_font) + menu_arrow_gap;
  
  int menu_offset_x = (gbuf_get_width(fb) - menu_width) / 2;
  int menu_offset_y = (gbuf_get_height(fb) - menu_height) / 2;

  int b_debounce = 0;
  int b_debounce_timer = 0;
  int b_repeater = 0;

  int cur_itm = 0;
  int num_itms = MAX_NUM_SAVESTATES + 1;
  int itm_lst = -1;
  int first_run = 1;
  int buttons_locked = 1;

  gbuffer_t thumbnail;
  bool img_valid = false;
  
  if (gbuf_create(&thumbnail, SNES_WIDTH / 2, SNES_HEIGHT / 2, (void*) em_psram_malloc(SNES_WIDTH / 2 * SNES_HEIGHT / 2 * 2)) != BUF_SUCCESS) {
    em_printf("Error: Cannot allocate PSRAM\n");
    goto leave;
  }

  while (1) {
    em_lcd_refresh();
    em_get_backbuffer(&fb_back);
    blit_buf(0, 0, BLIT_NO_ALPHA, fb, fb_back);
    if (menu_first_run)
      stars_draw(fb_back);

    draw_rectangle(menu_offset_x - menu_border_width,
                   menu_offset_y - menu_border_height,
                   menu_width + menu_border_width * 2,
                   menu_height + menu_border_height * 2,
                   translucency,
                   fb_back);
    
    // caption
    font_write_string_centered(1 + menu_offset_x + menu_width / 2, 1 + menu_offset_y, menu_font_shadow_color, (char*) caption, menu_font, fb_back);
    font_write_string_centered(menu_offset_x + menu_width / 2, menu_offset_y, menu_font_caption_color, (char*) caption, menu_font, fb_back);

    if (cur_itm >= 0 && cur_itm < (MAX_NUM_SAVESTATES)) {
      if (itm_lst != cur_itm) {
        strcpy(filename,  SAVESTATE_DIRECTORY);
        strcat(filename, Memory.ROMName);
        char slot_num[3];
        itoa(cur_itm + 1, slot_num, 10);
        strcat(filename, slot_num);
        strcat(filename, ".img");

        img_valid = read_image_to_gbuf(filename, thumbnail) == SUCCESS;
        itm_lst = cur_itm;
      }

      if (img_valid) {
          blit_buf(0, 0, 0xf81e, thumb_corner_top_left, thumbnail);
          blit_buf(0, gbuf_get_height(thumbnail) - gbuf_get_height( thumb_corner_bottom_left), 0xf81e, thumb_corner_bottom_left, thumbnail);
          blit_buf(gbuf_get_width(thumbnail) - gbuf_get_width(thumb_corner_top_right), 0, 0xf81e, thumb_corner_top_right, thumbnail);
          blit_buf(gbuf_get_width(thumbnail) - gbuf_get_width(thumb_corner_bottom_right), gbuf_get_height(thumbnail) - gbuf_get_height(thumb_corner_bottom_right), 0xf81e, thumb_corner_bottom_right, thumbnail);

          blit_buf(menu_offset_x + menu_width / 2 - SNES_WIDTH / 4,
                   menu_offset_y + font_get_height(menu_font) + clearance_y,
                  0xf81f,
                  thumbnail, fb_back);  
      } else {
        /*
        draw_rect_fill(menu_offset_x + menu_width / 2 - SNES_WIDTH / 4,
                       menu_offset_y + font_get_height(menu_font) + clearance_y,
                       menu_offset_x + menu_width / 2 - SNES_WIDTH / 4  + SNES_WIDTH / 2 - 1,
                       menu_offset_y + font_get_height(menu_font) + clearance_y  + em_lcd_get_height() / 2 - 1,
                       0xf81f,
                       fb_back);
        */
      }

      blit_buf_blend(menu_offset_x + menu_width / 2 - gbuf_get_width(dot_image) / 2,
                     menu_offset_y + font_get_height(menu_font) + font_get_height(menu_font) / 2 + clearance_y + em_lcd_get_height() / 4 - gbuf_get_height(dot_image) / 2 ,
                     0xf81f,
                     80,
                     dot_image,
                     fb_back);       

      char debug_print[3];
      itoa(cur_itm + 1, debug_print, 10);    
      font_write_string(menu_offset_x + menu_width / 2 - font_get_string_width(debug_print, menu_font) / 2 + 1,
                                 menu_offset_y + font_get_height(menu_font) + clearance_y + em_lcd_get_height() / 4 + 1,
                                 menu_font_shadow_color,
                                 debug_print,
                                 menu_font,
                                 fb_back);
      font_write_string(menu_offset_x + menu_width / 2 - font_get_string_width(debug_print, menu_font) / 2,
                                 menu_offset_y + font_get_height(menu_font) + clearance_y + em_lcd_get_height() / 4,
                                 menu_font_caption_color,
                                 debug_print,
                                 menu_font,
                                 fb_back);

    } else {
      font_write_string_centered(menu_offset_x + menu_width / 2 + 1,
                                 menu_offset_y + font_get_height(menu_font) + clearance_y + em_lcd_get_height() / 4 + 1,
                                 menu_font_shadow_color,
                                 (char*) str_back,
                                 menu_font,
                                 fb_back);
      font_write_string_centered(menu_offset_x + menu_width / 2,
                                 menu_offset_y + font_get_height(menu_font) + clearance_y + em_lcd_get_height() / 4,
                                 menu_font_caption_color,
                                 (char*) str_back,
                                 menu_font,
                                 fb_back);
    }

    // left/right arrows
    if (cur_itm != 0) {
      font_put_char(menu_offset_x - menu_border_width + menu_arrow_gap + 1,
                    menu_offset_y + font_get_height(menu_font) + clearance_y + em_lcd_get_height() / 4 + 1,
                    menu_font_shadow_color,
                    '<',
                    menu_font,
                    fb_back);
      font_put_char(menu_offset_x - menu_border_width + menu_arrow_gap,
                    menu_offset_y + font_get_height(menu_font) + clearance_y + em_lcd_get_height() / 4,
                    menu_font_caption_color,
                    '<',
                    menu_font, fb_back);
    }

    if (cur_itm != (num_itms - 1)) {
      font_put_char(menu_offset_x + menu_border_width - menu_arrow_gap + menu_width - font_get_char_width('>', menu_font) + 1,
                    menu_offset_y + font_get_height(menu_font) + clearance_y + em_lcd_get_height() / 4 + 1,
                    menu_font_shadow_color,
                    '>',
                    menu_font,
                    fb_back);
      font_put_char(menu_offset_x + menu_border_width - menu_arrow_gap + menu_width - font_get_char_width('>', menu_font),
                    menu_offset_y + font_get_height(menu_font) + clearance_y + em_lcd_get_height() / 4,
                    menu_font_caption_color,
                    '>',
                    menu_font, fb_back);
    }

    draw_battery(DRAW_BATTERY_X, DRAW_BATTERY_Y, pwr_get_bat_level(4), fb_back);
    em_request_pageflip();

    em_ctrl_button_state_update();

    if (first_run && buttons_locked && !em_ctrl_is_pressed(BUTTON_ANY)) {
      first_run = 0;
      buttons_locked = 0;
    }

    if (!buttons_locked) {
      // return selected item
      if (em_ctrl_is_pressed(BUTTON_A) ||
          em_ctrl_is_pressed(BUTTON_B) ||
          em_ctrl_is_pressed(BUTTON_X) ||
          em_ctrl_is_pressed(BUTTON_Y) ||
          em_ctrl_is_pressed(BUTTON_L) ||
          em_ctrl_is_pressed(BUTTON_R) ||
          em_ctrl_is_pressed(BUTTON_ST) ||
          em_ctrl_is_pressed(BUTTON_SEL) ||
          em_ctrl_is_pressed(BUTTON_SW))
        goto leave;

      if (em_ctrl_is_pressed(BUTTON_RIGHT) && !b_debounce && (cur_itm < (num_itms - 1)))
        cur_itm++;

      if (em_ctrl_is_pressed(BUTTON_LEFT) && !b_debounce && (cur_itm > 0))
        cur_itm--;

      if (!em_ctrl_is_pressed(BUTTON_ANY)) {
        b_debounce = 0;
        b_repeater = 0;
      } else {
        if (b_debounce == 0) {
          b_debounce = 1;
          b_debounce_timer = millis();
        } else if ((millis() - b_debounce_timer > MENU_REPEAT_START) && !b_repeater) {
          b_repeater = 1;
          b_debounce = 0;
          b_debounce_timer = millis();         
        } else if ((millis() - b_debounce_timer > MENU_REPEAT_DELAY) && b_repeater) {
          b_debounce = 0;
          b_debounce_timer = millis();         
        }
      }
    }

  }

leave:

  if (gbuf_get_dat_ptr(thumbnail))
    em_psram_free((void*) gbuf_get_dat_ptr(thumbnail));

  return cur_itm;

}

#if 0
int copy_rom_to_flash(gbuffer_t fb) {

  //if (!psram_rom)
  //  return ERR_PSRAM;

  long erase_size = Memory.ROM_Size;
  uint32_t rom_flash_address = FLASH_ROM_ADDRESS;
  uint32_t bytes_remaining = Memory.ROM_Size;
  uint32_t bytes_written = 0;
  int percent;
  int percent_old = -1;

  if (erase_size % FLASH_SECTOR_SIZE)
    erase_size = (erase_size / FLASH_SECTOR_SIZE + 1) * FLASH_SECTOR_SIZE;

  em_printf("trying to erase %d bytes @ %d\n", (int) erase_size, (int) rom_flash_address);

  for (int i = 0; i < erase_size / FLASH_SECTOR_SIZE; i++) {
    //flash_range_erase(rom_flash_address + i * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
    int rc = flash_safe_execute(call_flash_range_erase, (void*) (rom_flash_address + i * FLASH_SECTOR_SIZE), UINT32_MAX);
    //hard_assert(rc == PICO_OK);
    if (rc == PICO_OK)
      em_printf("erased 2k @ %d\n", (int)rom_flash_address + i * FLASH_SECTOR_SIZE);
    else
      em_printf("failed to erase 2k @ %d reason: %d\n", (int)rom_flash_address + i * FLASH_SECTOR_SIZE, rc);
    //	percent = 100 * (i * FLASH_SECTOR_SIZE) / erase_size;
    percent = 66 * (i * FLASH_SECTOR_SIZE) / erase_size;
    if (percent - percent_old > 5) {
      percent_old = percent;
      progress_bar(str_loading, percent, fb);
    }
  }

  //percent_old = 0;

  while (bytes_remaining > 0) {
    //flash_range_program(rom_flash_address + bytes_written, &Memory.ROM[bytes_written], FLASH_SECTOR_SIZE);

    //percent = 100 - 100 * bytes_remaining / rom_file_size;
    percent = 100 - 33 * bytes_remaining / Memory.ROM_Size;
    if (percent - percent_old > 5) {
      percent_old = percent;
      progress_bar(str_loading, percent, fb);
    }

    bytes_remaining -= FLASH_SECTOR_SIZE;
    bytes_written += FLASH_SECTOR_SIZE;
  }
  
  Memory.ROM = (uint8_t*) rom_flash_address + XIP_BASE;
  Memory.ROM_is_writable = false;

  rom_loaded = ROM_IN_MEM;

  //em_psram_free(psram_rom);
  //psram_rom = (uint8_t*) NULL;
  em_flush_cache();

  return 0;

}
#endif

int menu_loadfile(gbuffer_t fb) {
  int res = 0;
  int ret = 0;
  char* buffer = (char*) em_psram_malloc(DIRECTORY_BUFFER_SIZE);
  char* filename = (char*) em_psram_malloc(520);

  if (!buffer || !filename) {
    ret = ERR_PSRAM;
    goto bailout;
  }

  clear_item_buffer(buffer);

  res = em_get_directory((char*) rom_directory, buffer);

  if (!res) {
    ret = ERR_SD_OPEN_DIR;
    goto bailout;
  }

  printf("%d files found\n", get_item_cnt_aligned(buffer));
  if (get_item_cnt(buffer) == 0) {
    ret = ERR_NO_ROMS_FOUND;
    goto bailout;
  }

  // add caption and "go back" at top of list
  insert_item_aligned(0, (char*) str_sel_rom, buffer);
  insert_item_aligned(1, (char*) str_go_back, buffer);

  res = selection_menu((char*) buffer, false, false, false, 0, fb);
  em_printf("Selection is %d of %d\n", res, get_item_cnt(buffer));
  if (res == 0) {
    ret = ERR_ABORTED;
    goto bailout;
  }

  strcpy(filename, rom_directory);
  strcat(filename, get_item(res + 1, buffer));

  em_printf("filename to load is: %s\n", filename);

  ret = load_rom_from_sd(filename, fb);
  if (ret != 0) {
    em_printf("load_rom_from_sd return %d\n", ret);
    goto bailout;
  }

#if 0
  em_flush_cache();
#endif

  Memory.ROM_is_writable = true;

#if 0
  copy_rom_to_flash(fb);
#endif

  S9xReset();

  if (!LoadROM(NULL)) {
    ret = ERR_LOAD_ROM;
    goto bailout;
  }

  rom_loaded = ROM_LOADED;
 
  for (int i = 0; i < SRAM_SIZE; i++)
    Memory.SRAM[i] = 0;

 bailout:

  // mind sequence
  if (filename)
    em_psram_free((void*) filename);

  if (buffer)
    em_psram_free((void*) buffer);

  return ret;
}

void bl_fade_in() {
#define BL_FADE_STEPS (50)
  float delta = (float) emulator_state.brightness / (float) BL_FADE_STEPS;
  float value = 0;

  for (int i = 0; i < BL_FADE_STEPS; i++) {
    em_lcd_backlight_set((int) value);
    value += delta;
    uint32_t timer = micros();
    while (micros() - timer < 5000);
  }

  em_lcd_backlight_set((int) emulator_state.brightness);

}

void load_rom_from_flash(gbuffer_t fb_saved) {
#ifdef HEADER_ROM
    Memory.ROM_is_writable = false;
    Memory.ROM = (uint8_t*) ROM_DATA;
    Memory.ROM_Size = ROM_SIZE;
    rom_loaded = ROM_IN_MEM;
#endif
#ifdef CAP_FLASH_ROM
    Memory.ROM_is_writable = false;
    Memory.ROM = (uint8_t*) (uint32_t (FLASH_ROM_ADDRESS + 4));
    Memory.ROM_Size = *((uint32_t*) (uint32_t FLASH_ROM_ADDRESS));
    if (Memory.ROM_Size)
      rom_loaded = ROM_IN_MEM;
#endif

    if (rom_loaded == ROM_IN_MEM) {
      if (LoadROM(NULL))
        rom_loaded = ROM_LOADED;
      else
        rom_loaded = ROM_NOT_LOADED;

      // Some ROMs get patched and checksum then fails
      //if (rom_loaded && (Memory.CalculatedChecksum != Memory.ROMChecksum))
      //  rom_loaded = ROM_NOT_LOADED;

      if (rom_loaded == ROM_NOT_LOADED)
        message(str_err_rom, true, 0, fb_saved);

    }  
}

void emumgr_menu(gbuffer_t fb) {
  if (menu_first_run) {
    em_hal_init();

    // TODO: fixme
    emulator_state.brightness = 40;
    emulator_state.volume = 2;
    emulator_state.show_bat = false;
    emulator_state.show_fps = false;

    stars_init();
    bl_fade_in();
  }

  gbuffer_t fb_saved;
  int res;
  int main_choice, sub_choice;
  char* char_tmp = (char*) em_psram_malloc(520);

  if (!char_tmp) {
    em_printf("Error: Cannot allocate PSRAM\n");
    return;
  }

  if (gbuf_create(&fb_saved, SNES_WIDTH, em_lcd_get_height(), (void*) em_psram_malloc(SNES_WIDTH * em_lcd_get_height() * 2)) != BUF_SUCCESS) {
    em_printf("Error: Cannot allocate PSRAM\n");
    return;
  }

  // save original screen contents
  blit_buf(0, 0, BLIT_NO_ALPHA, fb, fb_saved);

  snd_mute();
  em_sd_init();
  //if (!em_sd_init())
  //  message(str_err_sd_mount, true, 1000, fb_saved);
  in_menu = true;

#if defined CAP_FLASH_ROM || defined HEADER_ROM
  if (menu_first_run)
    load_rom_from_flash(fb_saved);
#endif

  while (1) {
    main_choice = selection_menu((char*) main_menu_items, true, false, true, 0, fb_saved);
    switch (main_choice) {
      case 0:
        switch (rom_loaded) {
          case 0:
            message(str_load_game_first, true, MESSAGE_DELAY_LONG, fb_saved);
            break;
          case 1:
            S9xReset();
        
            if (!LoadROM(NULL)) {
              message(str_err_rom, true, 0, fb_saved);
              break;
            }
            rom_loaded = ROM_LOADED;

            goto leave;
            break;
          case 2:
            goto leave;
            break;
          default:
            break;
        }
        
        break;
      case 1:
        if (rom_loaded != ROM_LOADED) {
          message(str_load_game_first, true, MESSAGE_DELAY_LONG, fb_saved);
          break;
        }
        if (!em_sd_init()) {
          message(str_err_sd_mount, true, 0, fb_saved);
          break;
        }
        sub_choice = savestate_selector(load_menu, char_tmp, fb_saved);
        if ((sub_choice >= 0) && (sub_choice < MAX_NUM_SAVESTATES)) {
          strcpy(char_tmp,  SAVESTATE_DIRECTORY);
          strcat(char_tmp, Memory.ROMName);
          char slot_num[3];
          itoa(sub_choice + 1, slot_num, 10);
          strcat(char_tmp, slot_num);
          switch (load_state_handler(char_tmp)) {
            case SUCCESS:
              goto leave;
            case ERR_SD_OPEN_FIL:
              em_sd_deinit();
              message(str_err_open_file, true, 0, fb_saved);
              break;
            case ERR_SD_READING:
              em_sd_deinit();
              message(str_err_read_file, true, 0, fb_saved);
              break;
            default:
              break;
          }
        }          
        break;
      case 2:
        if (rom_loaded != ROM_LOADED) {
          message(str_load_game_first, true, MESSAGE_DELAY_LONG, fb_saved);
          break;
        }
        if (!em_sd_init()) {
          message(str_err_sd_mount, true, 0, fb_saved);
          break;
        }
        strcpy(char_tmp,  SAVESTATE_DIRECTORY);
        em_f_mkdir(char_tmp);
        sub_choice = savestate_selector(save_menu, char_tmp, fb_saved);
        if ((sub_choice >= 0) && (sub_choice < MAX_NUM_SAVESTATES)) {
          strcpy(char_tmp,  SAVESTATE_DIRECTORY);
          strcat(char_tmp, Memory.ROMName);
          char slot_num[3];
          itoa(sub_choice + 1, slot_num, 10);
          strcat(char_tmp, slot_num);

          switch (save_state_handler(char_tmp)) {
            case SUCCESS:
              strcat(char_tmp, ".img");
              screenshot_saver(char_tmp, fb_saved);
              //em_sd_deinit();
              message(str_gamestate_saved, true, MESSAGE_DELAY_BLINKOFANEYE, fb_saved);
              goto leave;
            case ERR_SD_OPEN_FIL:
              em_sd_deinit();
              message(str_err_open_file, true, 0, fb_saved);
              break;
            case ERR_SD_WRITING:
              em_sd_deinit();
              message(str_err_write_file, true, 0, fb_saved);
              break;
            default:
              break;
          }

        }
          
        break;
      case 3:
        sub_choice = slider_menu((char*) snd_menu, emulator_state.volume * 10, 10, 0, fb_saved);
        if ((sub_choice >= 0) && (sub_choice <= 100))
          emulator_state.volume = sub_choice / 10;
        break;
      case 4:
        sub_choice = slider_menu((char*) bri_menu, emulator_state.brightness, 10, 1, fb_saved);
        if ((sub_choice >= 0) && (sub_choice <= 100))
          if (sub_choice == 0) sub_choice = 1;
        emulator_state.brightness = sub_choice;
        break;
      case 5:
        if (!em_sd_init()) {
          message(str_err_sd_mount, true, 0, fb_saved);
          break;
        }

        res = menu_loadfile(fb_saved);
        switch (res) {
          case SUCCESS:
            em_printf("Starting emulation\n");
            goto leave;
          case ERR_ABORTED:
            em_printf("aborted by user\n");
            break;
          case ERR_PSRAM:
            em_printf("no psram\n");
            break;
          case ERR_SD_OPEN_DIR:
            em_printf("cannot read dir\n");
            em_sd_deinit();
            message(str_err_open_dir, true, 0, fb_saved);
            break;
          case ERR_SD_OPEN_FIL:
            em_sd_deinit();
            message(str_err_open_file, true, 0, fb_saved);
            em_printf("cannot open file\n");
            break;
          case ERR_LOAD_ROM:
            message(str_err_rom, true, 0, fb_saved);
            em_printf("error reading file\n");
            break;
          case ERR_SD_READING:
            em_sd_deinit();
            message(str_err_read_file, true, 0, fb_saved);
            em_printf("error loading rom\n");
            break;
          case ERR_NO_ROMS_FOUND:
            message(str_err_no_roms_found, true, 0, fb_saved);
            break;
          default:
            break;
        }
        break;
      case 6:
        sub_choice = selection_menu((char*) settings_menu_items, true, false, false, 0, fb_saved);

        switch (sub_choice) {
          case 0:
            emulator_state.show_fps = !emulator_state.show_fps;

            strcpy(char_tmp, STR_FPS);
            strcat(char_tmp, ": ");
            if (emulator_state.show_fps)
              strcat(char_tmp, STR_ON);
            else
              strcat(char_tmp, STR_OFF);
            char_tmp[strlen(char_tmp) + 1] = terminatorstring;

            message(char_tmp, true, MESSAGE_DELAY_SHORT, fb_saved);
            break;
          case 1:
            emulator_state.region++;
            emulator_state.region %= 3;            

            strcpy(char_tmp, STR_REGION);
            strcat(char_tmp, ": ");
            if (emulator_state.region == 0) {
              strcat(char_tmp, STR_AUTO);
              Settings.ForcePAL = false;
              Settings.ForceNTSC = false;
            } else if (emulator_state.region == 1) {
              strcat(char_tmp, STR_PAL);
              Settings.ForcePAL = true;
              Settings.ForceNTSC = false;
            } else if (emulator_state.region == 2) {
              strcat(char_tmp, STR_NTSC);
              Settings.ForcePAL = false;
              Settings.ForceNTSC = true;
            }
            char_tmp[strlen(char_tmp) + 1] = terminatorstring;

            message(char_tmp, true, MESSAGE_DELAY_SHORT, fb_saved);
            break;
          case 2: 
            emulator_state.show_bat = !emulator_state.show_bat;

            strcpy(char_tmp, STR_BAT);
            strcat(char_tmp, ": ");
            if (emulator_state.show_bat)
              strcat(char_tmp, STR_ON);
            else
              strcat(char_tmp, STR_OFF);
            char_tmp[strlen(char_tmp) + 1] = terminatorstring;

            message(char_tmp, true, MESSAGE_DELAY_SHORT, fb_saved);

            break;
          case 3:
            emulator_state.psram_subscreen = !emulator_state.psram_subscreen;

            strcpy(char_tmp, str_subscreen_mem);
            strcat(char_tmp, ": ");
            if (emulator_state.psram_subscreen)
              strcat(char_tmp, str_subscreen_psram);
            else
              strcat(char_tmp, str_subscreen_shared);
            char_tmp[strlen(char_tmp) + 1] = terminatorstring;
            message(char_tmp, true, MESSAGE_DELAY_SHORT, fb_saved);

            break;
#ifdef CAP_MANUAL_FRAMEDROP
          case 4:
            emulator_state.manual_framedrop = !emulator_state.manual_framedrop;

            strcpy(char_tmp, STR_FRAMEDROP_MODE);
            strcat(char_tmp, ": ");
            if (emulator_state.manual_framedrop)
              strcat(char_tmp, STR_MANUAL);
            else
              strcat(char_tmp, STR_AUTO);           
            char_tmp[strlen(char_tmp) + 1] = terminatorstring;
            message(char_tmp, true, MESSAGE_DELAY_SHORT, fb_saved);

            break;
#endif
#ifdef CAP_VSYNC
          case 4:
            emulator_state.vsync = !emulator_state.vsync;

            strcpy(char_tmp, str_vsync);
            strcat(char_tmp, ": ");
            if (emulator_state.vsync)
              strcat(char_tmp, STR_ON);
            else
              strcat(char_tmp, STR_OFF);
            char_tmp[strlen(char_tmp) + 1] = terminatorstring;
            message(char_tmp, true, MESSAGE_DELAY_SHORT, fb_saved);
            em_lcd_set_vsync(emulator_state.vsync);

            break;
#endif
          default:
            break;
        }
        break; 
      case 7:
        message(about, true, 0, fb_saved);
        break;  
      case 8:
        pwr_shutdown();
        break;  
      default:
        break;
    }
  }

leave:

  em_lcd_refresh();  // for FRR this does nothing

  em_sd_deinit();
  if (!emulator_state.muted)
    snd_unmute();

  // mind sequence
  if (gbuf_get_dat_ptr(fb_saved))
    em_psram_free((void*) gbuf_get_dat_ptr(fb_saved));

  if (char_tmp)
    em_psram_free((void*) char_tmp);

  if (menu_first_run) {
    menu_first_run = 0;
    translucency = 90;
  }

  in_menu = false;

}
