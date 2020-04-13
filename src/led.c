/**
 * \file led.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief Handle the LED behavior
 */

#include "led.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

static Led_t leds[N_LEDS];

static const uint32_t led_task_period_ms = 100;

int led_init(void)
{
    // GPIO already init in init_gpios from gpio.h

    // Initialize LED structure
    for (int i = 0; i < N_LEDS; i++)
    {
        leds[i].id = i;
        leds[i].period_ms = 500;
        leds[i].last_change_ms = HAL_GetTick();

    }
    leds[LED1].gpio_pin = LED1_PIN;
    leds[LED2].gpio_pin = LED2_PIN;
    leds[LED3].gpio_pin = LED3_PIN;

    return OK;
}

void led_on(uint16_t _led)
{
    HAL_GPIO_WritePin(LEDS_GPIO_PORT, leds[_led].gpio_pin, GPIO_PIN_RESET);
}

void led_off(uint16_t _led)
{
    HAL_GPIO_WritePin(LEDS_GPIO_PORT, leds[_led].gpio_pin, GPIO_PIN_SET);
}

void led_toggle(uint16_t _led)
{
    HAL_GPIO_TogglePin(LEDS_GPIO_PORT, leds[_led].gpio_pin);
}

void led_set_period(uint16_t _led, uint32_t _ms)
{
    if (_ms < led_task_period_ms)
        _ms = led_task_period_ms;
    leds[_led].period_ms = _ms;
}

void led_test(void)
{
    for (int i = 0; i < 10; i++)
    {
        led_toggle(LED1);
        led_toggle(LED2);
        led_toggle(LED3);
        HAL_Delay(500);
    }
} // led_test

void led_task(void* _params)
{
    if (_params != 0) { }

    uint32_t tick_ms = 0;
    uint8_t i = 0;

    led_set_period(LED1, 1000);
    led_set_period(LED2, 500);
    led_set_period(LED3, 200);

    while (1)
    {
        tick_ms = HAL_GetTick();
        for (i = 0; i < N_LEDS; i++)
        {
            if (tick_ms - leds[i].last_change_ms >= leds[i].period_ms)
            {
                led_toggle(i);
                leds[i].last_change_ms = tick_ms;
            }
        }
        
        vTaskDelay(led_task_period_ms / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
} // led_task
