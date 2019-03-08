/**
 * \file usTimer.c
 * \brief Microsecond [us] timer
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 */

#include <string.h>

#include "usTimer.h"
#include "main.h"

// Timer handler declaration
static TIM_HandleTypeDef TimHandle6;

static uint64_t usTime = 0;

int init_us_timer(void)
{
    int ret = 0;

    RCC_ClkInitTypeDef clkconfig;
    uint32_t uwTimclock = 0U, uwAPB1Prescaler = 0U;
    uint32_t pFLatency = 0U, uwPrescalerValue = 0U;
    
    // Get clock configuration
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
  
    // Get APB1 prescaler
    uwAPB1Prescaler = clkconfig.APB1CLKDivider;
  
    // Compute TIM6 clock
    if (uwAPB1Prescaler == RCC_HCLK_DIV1)
      uwTimclock = HAL_RCC_GetPCLK1Freq();
    else
      uwTimclock = 2 * HAL_RCC_GetPCLK1Freq();
  
    // Compute the prescaler value to have TIM6 counter clock equal to 1MHz
    uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);
    //uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2000000U) - 1U);

    __HAL_RCC_TIM6_CLK_ENABLE();
    TimHandle6.Instance                  = TIM6;
    TimHandle6.Init.Prescaler            = uwPrescalerValue;
    TimHandle6.Init.Period               = 0xFFFF;
    TimHandle6.Init.ClockDivision        = TIM_CLOCKDIVISION_DIV1;
    TimHandle6.Init.CounterMode          = TIM_COUNTERMODE_UP;
    TimHandle6.Init.RepetitionCounter    = 0;

    ret = HAL_TIM_Base_Init(&TimHandle6);
    if (ret != HAL_OK) {
        return NOK;
    }

    // Set timer IRQ priority
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1 ,0U);

    // Enable timer IRQ
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    // Start the timer in interrupt mode
    ret = HAL_TIM_Base_Start_IT(&TimHandle6);
    if (ret != HAL_OK) {
        return NOK;
    }

    usTime = 0;
    
    return OK;
}

void delay_us(uint32_t us)
{
  uint64_t start = get_us_time();
  while ((get_us_time() - start) <= us);
}

uint64_t get_us_time(void)
{
  return (usTime + TIM6->CNT);
}

void reset_us_timer(void)
{
  TIM6->CNT = 0;
  usTime = 0;
}

void suspend_us_timer(void)
{
  __HAL_TIM_DISABLE_IT(&TimHandle6, TIM_IT_UPDATE);
}

void resume_us_timer(void)
{
  __HAL_TIM_ENABLE_IT(&TimHandle6, TIM_IT_UPDATE);
}

void test_us_timer(void)
{
  uint64_t start = 0, stop = 0;
  uint32_t dt = 0;

  start = get_us_time();
  HAL_Delay(40);
  stop = get_us_time();
  dt = (stop - start) / 1000;
  printf("40ms <-> cnt=%ld, start=%ld, stop=%ld\r\n", dt, (uint32_t)start, (uint32_t)stop);
}

void __attribute__((used)) TIM6_DAC_IRQHandler(void)
{
  __HAL_TIM_CLEAR_IT(&TimHandle6, TIM_IT_UPDATE);
  usTime += 65535;
  //__sync_fetch_and_add(&usTime, 65535);
}