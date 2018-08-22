#pragma once

#include <stdint.h>

uint8_t led_init(void);

void led_on(uint16_t _led);

void led_off(uint16_t _led);

void led_toggle(uint16_t _led);