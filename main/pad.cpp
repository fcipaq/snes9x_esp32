#include "driver/gpio.h"

#include "pad.h"

#define DPAD_LEFT_PIN (gpio_num_t)22
#define DPAD_RIGHT_PIN (gpio_num_t)20
#define DPAD_UP_PIN (gpio_num_t)21
#define DPAD_DOWN_PIN (gpio_num_t)23

#define BUTTON_1_PIN (gpio_num_t)6
#define BUTTON_2_PIN (gpio_num_t)5
#define BUTTON_3_PIN (gpio_num_t)4

uint32_t ctrl_dpad_state()
{
    uint32_t state = 0;
    if (!gpio_get_level(DPAD_LEFT_PIN))
        state |= DPAD_LEFT;
    if (!gpio_get_level(DPAD_RIGHT_PIN))
        state |= DPAD_RIGHT;
    if (!gpio_get_level(DPAD_UP_PIN))
        state |= DPAD_UP;

    if (!gpio_get_level(DPAD_DOWN_PIN))
        state |= DPAD_DOWN;

    return state;
}

uint32_t ctrl_button_state()
{
    uint32_t state = 0;
    if (!gpio_get_level(BUTTON_1_PIN))
        state |= BUTTON_1;
    if (!gpio_get_level(BUTTON_2_PIN))
        state |= BUTTON_2;
    if (!gpio_get_level(BUTTON_3_PIN))
        state |= BUTTON_3;

    return state;
}

void init_pad()
{
    gpio_reset_pin(DPAD_LEFT_PIN);
    gpio_reset_pin(DPAD_RIGHT_PIN);
    gpio_reset_pin(DPAD_UP_PIN);
    gpio_reset_pin(DPAD_DOWN_PIN);
    gpio_reset_pin(BUTTON_1_PIN);
    gpio_reset_pin(BUTTON_2_PIN);
    gpio_reset_pin(BUTTON_3_PIN);

    gpio_set_direction(DPAD_LEFT_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(DPAD_RIGHT_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(DPAD_UP_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(DPAD_DOWN_PIN, GPIO_MODE_INPUT);

    gpio_set_direction(BUTTON_1_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_2_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_3_PIN, GPIO_MODE_INPUT);
}
