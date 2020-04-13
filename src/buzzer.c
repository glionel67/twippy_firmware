#include "buzzer.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

TIM_HandleTypeDef TimHandleBuzzer;
static TIM_OC_InitTypeDef sConfigBuzzer;

static uint32_t timerClockFreq = 0;
static uint32_t buzzerPeriod = 0;

int init_buzzer(void)
{
    int ret = 0;

    timerClockFreq = 2 * HAL_RCC_GetPCLK1Freq();
    buzzerPeriod = timerClockFreq / 10000 - 1;

    // Init. GPIO done in init_gpios

    // Init. timer
    TIM_BUZZER_CLK_ENABLE();
    TimHandleBuzzer.Instance = TIM_BUZZER;
    TimHandleBuzzer.Init.Period = buzzerPeriod;
    TimHandleBuzzer.Init.Prescaler = 0;
    TimHandleBuzzer.Init.ClockDivision = 0;
    TimHandleBuzzer.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandleBuzzer.Init.RepetitionCounter = 0;
    if (HAL_OK != HAL_TIM_Base_Init(&TimHandleBuzzer))
        return NOK;

    sConfigBuzzer.OCMode        = TIM_OCMODE_PWM1;
    sConfigBuzzer.OCPolarity    = TIM_OCPOLARITY_HIGH;
    sConfigBuzzer.OCIdleState   = TIM_OCIDLESTATE_RESET;
    sConfigBuzzer.OCFastMode    = TIM_OCFAST_DISABLE;
    sConfigBuzzer.OCNPolarity   = TIM_OCNPOLARITY_HIGH;
    sConfigBuzzer.OCNIdleState  = TIM_OCNIDLESTATE_RESET;
    sConfigBuzzer.Pulse = 0;
    ret = HAL_TIM_PWM_ConfigChannel(&TimHandleBuzzer, 
            &sConfigBuzzer, TIM_BUZZER_CHANNEL);
    if (HAL_OK != ret)
        return NOK;

    if (HAL_OK != HAL_TIM_PWM_Start(&TimHandleBuzzer, TIM_BUZZER_CHANNEL))
        return NOK;

    turn_off_buzzer();

    return OK;
}

int set_buzzer_freq(uint32_t _freq)
{
    buzzerPeriod = ((timerClockFreq/_freq)-1);

    TimHandleBuzzer.Instance = TIM_BUZZER;
    TimHandleBuzzer.Init.Period = buzzerPeriod;
    TimHandleBuzzer.Init.Prescaler = 0;
    TimHandleBuzzer.Init.ClockDivision = 0;
    TimHandleBuzzer.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandleBuzzer.Init.RepetitionCounter = 0;
    if (HAL_OK != HAL_TIM_Base_Init(&TimHandleBuzzer))
        return NOK;

    return set_buzzer_dutyCycle(50);
}

/*
 * _dc \in [0,100] %
 */
int set_buzzer_dutyCycle(uint16_t _dc)
{
    int ret = 0;
    _dc = (_dc * buzzerPeriod) / 100;

    sConfigBuzzer.Pulse = _dc;
    ret = HAL_TIM_PWM_ConfigChannel(&TimHandleBuzzer, 
            &sConfigBuzzer, TIM_BUZZER_CHANNEL);
    if (HAL_OK != ret)
        return NOK;

    if (HAL_OK != HAL_TIM_PWM_Start(&TimHandleBuzzer, TIM_BUZZER_CHANNEL))
        return NOK;

    return OK;
}

int turn_on_buzzer(void)
{
    return set_buzzer_dutyCycle(50);
}

int turn_off_buzzer(void)
{
    return set_buzzer_dutyCycle(0);
    //return set_buzzer_dutyCycle(100);
}

void test_buzzer(void)
{
    uint32_t freqs[] = { 50, 100, 200, 300, 400, 500, 600, 700, 800, 900, 1000,
            2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000 }; // n=19
    for (int i = 0; i < 15; i++)
    {
        set_buzzer_freq(freqs[i]);
        HAL_Delay(200);
    }
    turn_off_buzzer();
}

void buzzer_task(void* _params)
{

    if (_params != 0) { }

    while (1)
    {
        vTaskDelay(500/portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}