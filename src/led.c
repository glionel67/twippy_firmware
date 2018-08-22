#include "led.h"
#include "main.h"

uint8_t led_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    LED_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = LED1_PIN | LED2_PIN | LED3_PIN;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    return 1;
}

void led_on(uint16_t _led) {
    HAL_GPIO_WritePin(LED_GPIO_PORT, _led, GPIO_PIN_RESET);
}

void led_off(uint16_t _led) {
    HAL_GPIO_WritePin(LED_GPIO_PORT, _led, GPIO_PIN_SET);
}

void led_toggle(uint16_t _led) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, _led);
}
