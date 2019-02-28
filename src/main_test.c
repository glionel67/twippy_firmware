// Includes
#include "main.h"
#include "config.h"
#include "gpio.h"
#include "uart1.h"
#include "uart2.h"
#include "led.h"

// libc
#include <stdio.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// Function prototypes
void SystemClock_Config(void);

// ---------------------------------------------------------------------------//
// --- Main
// ---------------------------------------------------------------------------//
int main(void)
{
  int ret = 0;

  HAL_Init(); // Init systick

  // Configure system clock to 180 MHz
  SystemClock_Config();

  // ------------------------------------------------------------------------ //
  // --- Init GPIO
  // ------------------------------------------------------------------------ //
  ret = init_gpios();
  if (ret != OK) {
    Error_Handler();
  }

  // ------------------------------------------------------------------------ //
  // --- Init LED
  // ------------------------------------------------------------------------ //
  ret = led_init();
  if (ret != OK) {
    Error_Handler();
  }

  // ------------------------------------------------------------------------ //
  // --- Init UART
  // ------------------------------------------------------------------------ //
  ret = uart2_init();
  if (ret == NOK) {
    Error_Handler();
  }

  ret = uart1_init();
  if (ret == NOK) {
    printf("uart1_init NOK\r\n");
    Error_Handler();
  }
  else {
    printf("uart1_init OK\r\n");
  }

  // ------------------------------------------------------------------------ //
  // --- Create FreeRTOS tasks
  // ------------------------------------------------------------------------ //
  if (!(pdPASS == xTaskCreate(led_task, (const char*)"led_task",
    LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL))) {
    printf("Failed to create led_task\r\n");
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(uart1_task, (const char*)"uart1_task",
    UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL))) {
    printf("Failed to create uart1_task\r\n");
    goto hell;
  }

  // if (!(pdPASS == xTaskCreate(uart2_task, (const char*)"uart2_task",
  //   UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL))) {
  //   printf("Failed to create uart2_task\r\n");
  //   goto hell;
  // }

  // Start FreeRTOS scheduler
  vTaskStartScheduler();

  while (1) {
    printf("Error: should not reach here!\r\n");
    goto hell;
  }

  hell: // Should never reach here
    Error_Handler();

    return 0;
}

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState        = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue   = 0x10;
  RCC_OscInitStruct.PLL.PLLState      = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM        = 16;
  RCC_OscInitStruct.PLL.PLLN        = 360;
  RCC_OscInitStruct.PLL.PLLP        = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ        = 7;
  RCC_OscInitStruct.PLL.PLLR        = 6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  ret = HAL_PWREx_EnableOverDrive();
  if (ret != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | 
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // To have a 45 MHz clock
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // To have a 90 MHz clock
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  led_set_period(200);

  while (1) {
    printf("ERROR!\r\n");
    HAL_Delay(1000);
    led_toggle(LED1);
  }
}
