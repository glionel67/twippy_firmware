#include "led.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

static uint16_t led_period_ms = 500;

int led_init(void)
{
    // GPIO already init in init_gpios from gpio.h
    return OK;
}

void led_on(uint16_t _led)
{
    HAL_GPIO_WritePin(LEDS_GPIO_PORT, _led, GPIO_PIN_RESET);
}

void led_off(uint16_t _led)
{
    HAL_GPIO_WritePin(LEDS_GPIO_PORT, _led, GPIO_PIN_SET);
}

void led_toggle(uint16_t _led)
{
    HAL_GPIO_TogglePin(LEDS_GPIO_PORT, _led);
}

void led_set_period(uint16_t _ms)
{
    led_period_ms = _ms;
}

void test_led(void)
{
    for (int i=0;i<10;i++) {
        led_toggle(LED1);
        //led_toggle(LED2);
        //led_toggle(LED3);
        HAL_Delay(500);
    }
}

void led_task(void* _params)
{
    if (_params != 0) { }

    while (1) {
        led_toggle(LED1);
        vTaskDelay(led_period_ms/portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}
