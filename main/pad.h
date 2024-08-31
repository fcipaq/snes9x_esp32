#pragma once
#include <stdio.h>

#define USE_PAD 1

#define DPAD_LEFT 0x01
#define DPAD_RIGHT 0x02
#define DPAD_UP 0x04
#define DPAD_DOWN 0x08
#define BUTTON_1 0x10
#define BUTTON_2 0x20
#define BUTTON_3 0x40


uint32_t ctrl_dpad_state();
uint32_t ctrl_button_state();

void init_pad();
