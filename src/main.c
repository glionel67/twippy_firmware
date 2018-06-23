// Includes
#include "main.h"
#include "gpio.h"
#include "uart.h"
#include "encoders.h"
#include "usTimer.h"
#include "imu.h"
//#include "motor.h"
//#include "adc.h"

#include <string.h>

#include "stm32f4xx.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
//#include "timers.h"
//#include "semphr.h"

// Defines
#define DATASIZE 100

// Global variables
static xQueueHandle myQueue = 0;


// Function prototypes
void SystemClock_Config(void);
void myTask(void* _params);
void myTask2(void* _params);
void encTestTask(void* _params);
void motorTestTask(void* _params);
void adcTestTask(void* _params);
void imuTestTask(void* _params);

// ---------------------------------------------------------------------------//
// --- Main
// ---------------------------------------------------------------------------//
int main(void) {
  int ret = 0;
  uint8_t welcomeMsg[] = "Hello World!\r\n";
  //uint8_t data[DATASIZE] = { 0, };
  //int32_t enc1 = 0, enc2 = 0;
  //uint32_t ticks = 0;

  //SystemInit();
  //NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_Init();
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  //HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority, 0U);

  // Configure system clock to 180 MHz
  SystemClock_Config();

  ret = init_gpios();
  if (ret != 0) {
    Error_Handler();
  }

  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  HAL_Delay(2000);

  ret = uart1_init();
  if (ret != 0) {
    Error_Handler();
  }
  ret = uart2_init();
  if (ret != 0) {
    Error_Handler();
  }
  
  uart1_write(welcomeMsg, sizeof(welcomeMsg));
  uart2_write(welcomeMsg, sizeof(welcomeMsg));

  ret = init_us_timer();
  if (ret != 0) {
    Error_Handler();
  }

  test_us_timer();

  myQueue = xQueueCreate(1, sizeof(uint32_t));

  // Create FreeRTOS tasks
  if (!(pdPASS == xTaskCreate(uart1_task, (const char*)"uart1_task",
    2*configMINIMAL_STACK_SIZE, NULL, 1, NULL)))
    goto hell;
  if (!(pdPASS == xTaskCreate(uart2_task, (const char*)"uart2_task",
    2*configMINIMAL_STACK_SIZE, NULL, 1, NULL)))
    goto hell;

  if (!(pdPASS == xTaskCreate(myTask, (const char*)"task1",
    2*configMINIMAL_STACK_SIZE, NULL, 1, NULL)))
    goto hell;
  if (!(pdPASS == xTaskCreate(myTask2, (const char*)"task2",
    2*configMINIMAL_STACK_SIZE, NULL, 2, NULL)))
    goto hell;

  //if (!(pdPASS == xTaskCreate(enc_test_task, (const char*)"enc_test_task",
  //  2*configMINIMAL_STACK_SIZE, NULL, 2, NULL)))
  //  goto hell;

  if (!(pdPASS == xTaskCreate(imu_test_task, (const char*)"imu_test_task",
    2*configMINIMAL_STACK_SIZE, NULL, 2, NULL)))
    goto hell;

  vTaskStartScheduler();

  while (1) {
    char msg[] = "Error: should not reach here!\n";
    uart1_write((uint8_t*)msg, strlen(msg));
    uart2_write((uint8_t*)msg, strlen(msg));
    HAL_Delay(1000);
  }

  hell: // Should never reach here
    Error_Handler();

    return 0;
}

void SystemClock_Config(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

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

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // To have a 45 MHz clock
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // To have a 90 MHz clock
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void) {
  uint8_t errorMsg[] = "ERROR!\r\n";
  while (1) {
    uart1_write(errorMsg, sizeof(errorMsg));
    uart2_write(errorMsg, sizeof(errorMsg));
    HAL_Delay(1000);
  }
}

void print_msg(uint8_t* _msg, uint8_t _len) {

  uint8_t end = 0;
  while (_msg[end] != '\n')
    end++;
  _len = end+1;

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    //uart1_enque_data(_msg, (uint32_t)_len);
    //uart2_enque_data(_msg, (uint32_t)_len);
    uart1_send_data(_msg, _len);
    uart2_send_data(_msg, _len);
  }
  else {
    uart1_send_data(_msg, _len);
    uart2_send_data(_msg, _len);
  }
}

// void HAL_Delay(volatile uint32_t Delay) {
//   if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
//     uint32_t tickstart = HAL_GetTick();
//     uint32_t wait = Delay;
    
//     // Add a period to guarantee minimum wait
//     if (wait < HAL_MAX_DELAY)
//        wait++;
    
//     while ((HAL_GetTick() - tickstart) < wait);
//   }
//   else
//     vTaskDelay(Delay);
// }


void myTask(void* _params) {
  char data[DATASIZE] = { 0, };
  uint32_t ticks = 0;

  if (_params != 0) { }

  sprintf(data, "myTask\r\n");
  print_msg((uint8_t*)data, 20);
  memset(data, '\0', strlen(data));

  while (1) {
    ticks = HAL_GetTick();
    sprintf(data, "myTask: %lu\r\n", ticks);
    //print_msg((uint8_t*)data, 20);
    memset(data, '\0', strlen(data));
    xQueueOverwrite(myQueue, &ticks);
    vTaskDelay(1000/portTICK_RATE_MS);
    //HAL_Delay(1000);
  }

  vTaskDelete(NULL);
}

void myTask2(void* _params) {
  char data[DATASIZE] = { 0, };
  uint32_t ticks = 0;

  if (_params != 0) { }

  sprintf(data, "myTask2\r\n");
  print_msg((uint8_t*)data, 20);
  memset(data, '\0', strlen(data));
  while (1) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    //HAL_Delay(200);
    vTaskDelay(200/portTICK_RATE_MS);

    if (pdTRUE == xQueueReceive(myQueue, &ticks, 0)) {
      sprintf(data, "myTask2: %lu\r\n", ticks);
      //print_msg((uint8_t*)data, 20);
      memset(data, '\0', strlen(data));
    }
  }

  vTaskDelete(NULL);
}
