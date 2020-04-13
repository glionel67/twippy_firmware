/**
 * \file led.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief Handle the LED behavior
 */

#pragma once

#include <stdint.h>

enum { LED1=0, LED2, LED3, N_LEDS };

/*
 * \struct Led_t
 * \brief Structure containing the LED information
 */
typedef struct 
{
	uint16_t id;
	uint16_t gpio_pin;
	uint32_t period_ms;
	uint32_t last_change_ms;
} Led_t;

/**
 * \fn led_init
 * \brief Initialize the LEDs
 */
int led_init(void);

/**
 * \fn led_on
 * \brief Turn on the LED
 * \param _led: index of the LED to turn on
 */
void led_on(uint16_t _led);

/**
 * \fn led_off
 * \brief Turn off the LED
 * \param _led: index of the LED to turn off
 */
void led_off(uint16_t _led);

/**
 * \fn led_toggle
 * \brief Toggle the LED
 * \param _led: index of the LED to toggle
 */
void led_toggle(uint16_t _led);

/**
 * \fn led_set_period
 * \brief Set the period in ms of the LED
 * \param _led: index of the LED
 * \param _ms: period to set
 */
void led_set_period(uint16_t _led, uint32_t _ms);

/**
 * \fn led_test
 * \brief Test the LEDs
 */
void led_test(void);

/**
 * \fn led_task
 * \brief FreeRTOS task of the LEDs
 */
void led_task(void* _params);