#include "buzzer.h"
#include "config.h"

TIM_HandleTypeDef TimHandleBuzzer;
static TIM_OC_InitTypeDef sConfigBuzzer;

uint32_t buzzerPeriod = ((64000000/10000)-1);

int init_buzzer(void) {
    int ret = 0;

    // Init. GPIOs done in init_gpio
    // Timer 1
    TIM_BUZZER_CLK_ENABLE();
    TimHandleBuzzer.Instance = TIM_BUZZER;
    TimHandleBuzzer.Init.Period = buzzerPeriod;
    TimHandleBuzzer.Init.Prescaler = 0;
    TimHandleBuzzer.Init.ClockDivision = 0;
    TimHandleBuzzer.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandleBuzzer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&TimHandleBuzzer) != HAL_OK) {
        return -1;
    }

    sConfigBuzzer.OCMode        = TIM_OCMODE_PWM1;
    sConfigBuzzer.OCPolarity    = TIM_OCPOLARITY_HIGH;
    sConfigBuzzer.OCIdleState   = TIM_OCIDLESTATE_RESET;
    sConfigBuzzer.OCFastMode    = TIM_OCFAST_DISABLE;
    sConfigBuzzer.OCNPolarity   = TIM_OCNPOLARITY_HIGH;
    sConfigBuzzer.OCNIdleState  = TIM_OCNIDLESTATE_RESET;
    sConfigBuzzer.Pulse = 0;
    ret = HAL_TIM_PWM_ConfigChannel(&TimHandleBuzzer, &sConfigBuzzer, TIM_BUZZER_CHANNEL);
    if (ret != HAL_OK) {
        return -1;
    }

    ret = HAL_TIM_PWM_Start(&TimHandleBuzzer, TIM_BUZZER_CHANNEL);
    if (ret != HAL_OK) {
        return -1;
    }

    return 0;
}

int setBuzzerFreq(uint32_t _freq) {
    buzzerPeriod = ((64000000/_freq)-1);

    TimHandleBuzzer.Instance = TIM_BUZZER;
    TimHandleBuzzer.Init.Period = buzzerPeriod;
    TimHandleBuzzer.Init.Prescaler = 0;
    TimHandleBuzzer.Init.ClockDivision = 0;
    TimHandleBuzzer.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandleBuzzer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&TimHandleBuzzer) != HAL_OK) {
        return -1;
    }

    return setBuzzerDutyCycle(50);
}

/*
 * _dc \in [0,100] %
 */
int setBuzzerDutyCycle(uint16_t _dc) {
    int ret = 0;
    _dc = (_dc*buzzerPeriod)/100;

    sConfigBuzzer.Pulse = _dc;
    ret = HAL_TIM_PWM_ConfigChannel(&TimHandleBuzzer, &sConfigBuzzer, TIM_BUZZER_CHANNEL);
    if (ret != HAL_OK) {
        return -1;
    }

    ret = HAL_TIM_PWM_Start(&TimHandleBuzzer, TIM_BUZZER_CHANNEL);
    if (ret != HAL_OK) {
        return -1;
    }
    return 0;
}

int buzzerTurnOn(void) {
    return setBuzzerDutyCycle(50);
}

int buzzerTurnOff(void) {
    return setBuzzerDutyCycle(0);
}

void testBuzzer(void) {
    setBuzzerFreq(50);
    HAL_Delay(200);
    setBuzzerFreq(100);
    HAL_Delay(200);
    setBuzzerFreq(200);
    HAL_Delay(200);
    setBuzzerFreq(300);
    HAL_Delay(200);
    setBuzzerFreq(400);
    HAL_Delay(200);
    setBuzzerFreq(500);
    HAL_Delay(200);
    setBuzzerFreq(600);
    HAL_Delay(200);
    setBuzzerFreq(700);
    HAL_Delay(200);
    setBuzzerFreq(800);
    HAL_Delay(200);
    setBuzzerFreq(900);
    HAL_Delay(200);
    setBuzzerFreq(1000);
}