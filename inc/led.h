#pragma once

#include <stdint.h>

#define LED1 LED1_PIN
//#define LED2 LED2_PIN
//#define LED3 LED3_PIN

int led_init(void);

void led_on(uint16_t _led);

void led_off(uint16_t _led);

void led_toggle(uint16_t _led);

void led_set_period(uint16_t _ms);

void test_led(void);

void led_task(void* _params);

