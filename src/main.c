// Includes
#include "main.h"
#include "config.h"
#include "gpio.h"
#include "uart1.h"
#include "uart2.h"
#include "spi.h"
#include "encoder.h"
#include "usTimer.h"
#include "imu.h"
#include "ahrs.h"
#include "motor.h"
#include "motor_control.h"
//#include "adc.h"
#include "servo.h"
#include "led.h"

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

// Function prototypes
void SystemClock_Config(void);

// ---------------------------------------------------------------------------//
// --- Main
// ---------------------------------------------------------------------------//
int main(void) {
  int ret = 0;
  uint8_t welcomeMsg[] = "\r\n--- Hello Twippy! ---\r\n";
  //uint8_t data[DATASIZE] = { 0, };
  //int32_t enc1 = 0, enc2 = 0;
  //uint32_t ticks = 0;

  HAL_Init();

  // Configure system clock to 180 MHz
  SystemClock_Config();

  // ------------------------------------------------------------------------ //
  // --- Init GPIO
  // ------------------------------------------------------------------------ //
  ret = init_gpios();
  if (!ret) {
    Error_Handler();
  }

  // ------------------------------------------------------------------------ //
  // --- Init LED
  // ------------------------------------------------------------------------ //
  ret = led_init();
  if (!ret) {
    Error_Handler();
  }

  // ------------------------------------------------------------------------ //
  // --- Init UART
  // ------------------------------------------------------------------------ //
  ret = uart1_init();
  if (ret != 0) {
    Error_Handler();
  }
  ret = uart2_init();
  if (ret != 0) {
    Error_Handler();
  }
  
  //UART1_WRITE(welcomeMsg, sizeof(welcomeMsg));
  UART2_WRITE(welcomeMsg, sizeof(welcomeMsg));

  // ------------------------------------------------------------------------ //
  // --- Init SPI
  // ------------------------------------------------------------------------ //
  ret = spi1_init();
  if (!ret) {
    char str[] = "spi1_init NOK\r\n";
    print_msg((uint8_t*)str, strlen(str));
    Error_Handler();
  }
  else {
    char str[] = "spi1_init OK\r\n";
    print_msg((uint8_t*)str, strlen(str));
  }

  // ------------------------------------------------------------------------ //
  // --- Init microsecond timer
  // ------------------------------------------------------------------------ //
  ret = init_us_timer();
  if (ret != 0) {
    char str[] = "init_us_timer NOK\r\n";
    print_msg((uint8_t*)str, strlen(str));
    Error_Handler();
  }
  else {
    char str[] = "init_us_timer OK\r\n";
    print_msg((uint8_t*)str, strlen(str));
  }

  test_us_timer();

  // ------------------------------------------------------------------------ //
  // --- Init encoder
  // ------------------------------------------------------------------------ //
  ret = init_encoders();
  if (ret != 0) {
    char str[] = "init_encoders NOK\n";
    print_msg((uint8_t*)str, strlen(str));
    Error_Handler();
  }
  else {
    char str[] = "init_encoders OK\r\n";
    print_msg((uint8_t*)str, strlen(str));
  }

  // ------------------------------------------------------------------------ //
  // --- Init motor
  // ------------------------------------------------------------------------ //
  ret = init_motors();
  if (ret != 0) {
    char str[] = "init_motors NOK\n";
    print_msg((uint8_t*)str, strlen(str));
    Error_Handler();
  }
  else {
    char str[] = "init_motors OK\r\n";
    print_msg((uint8_t*)str, strlen(str));
  }

  // ------------------------------------------------------------------------ //
  // --- Init motor control
  // ------------------------------------------------------------------------ //
  ret = motor_control_init();
  if (!ret) {
    char str[] = "motor_control_init NOK\n";
    print_msg((uint8_t*)str, strlen(str));
    Error_Handler();
  }
  else {
    char str[] = "motor_control_init OK\r\n";
    print_msg((uint8_t*)str, strlen(str));
  }

  // ------------------------------------------------------------------------ //
  // --- Init IMU
  // ------------------------------------------------------------------------ //
  ret = init_imu();
  if (!ret) {
    char str[] = "init_imu NOK\r\n";
    print_msg((uint8_t*)str, strlen(str));
    Error_Handler();
  }
  else {
    char str[] = "init_imu OK\r\n";
    print_msg((uint8_t*)str, strlen(str));
  }

  // ------------------------------------------------------------------------ //
  // --- Create FreeRTOS tasks
  // ------------------------------------------------------------------------ //
  if (!(pdPASS == xTaskCreate(uart1_task, (const char*)"uart1_task",
    UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create uart1_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(uart2_task, (const char*)"uart2_task",
    UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create uart2_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(encoder_task, (const char*)"encoder_task",
    ENCODER_TASK_STACK_SIZE, NULL, ENCODER_TASK_PRIORITY, NULL)))
    goto hell;

  if (!(pdPASS == xTaskCreate(imu_task, (const char*)"imu_task",
    IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create imu_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(imu_calibrate_gyro_bias_task, 
      (const char*)"imu_calibrate_gyro_bias_task",
      IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create imu_calibrate_gyro_bias_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
  }

  if (!(pdPASS == xTaskCreate(ahrs_task, (const char*)"ahrs_task",
    AHRS_TASK_STACK_SIZE, NULL, AHRS_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create ahrs_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }
/*
  if (!(pdPASS == xTaskCreate(motor_ident_task, (const char*)"motor_ident_task",
    MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create motor_ident_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }
*/
  if (!(pdPASS == xTaskCreate(motor_task, (const char*)"motor_task",
    MOTOR_TASK_STACK_SIZE, NULL, MOTOR_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create motor_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }
/*
  if (!(pdPASS == xTaskCreate(motor_control_task, (const char*)"motor_control_task",
    MOTOR_CONTROL_TASK_STACK_SIZE, NULL, MOTOR_CONTROL_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create motor_control_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(motor_control_test_task, (const char*)"motor_control_test_task",
    MOTOR_CONTROL_TASK_STACK_SIZE, NULL, MOTOR_CONTROL_TASK_PRIORITY, NULL))) {
    char msg[] = "Failed to create motor_control_test_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }
*/
  /*
  if (!(pdPASS == xTaskCreate(servo_task, (const char*)"servo_task",
    SERVO_TASK_STACK_SIZE, NULL, SERVO_TASK_STACK_SIZE, NULL))) {
    char msg[] = "Failed to create servo_task\r\n";
    print_msg((uint8_t*)msg, strlen(msg));
    goto hell;
  }
  */

  // Start FreeRTOS scheduler
  vTaskStartScheduler();

  while (1) {
    char msg[] = "Error: should not reach here!\n";
    print_msg((uint8_t*)msg, strlen(msg));
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
  GPIO_InitTypeDef GPIO_InitStruct;

  // Init LED D13 on PA5
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

  while (1) {
    //UART1_WRITE(errorMsg, sizeof(errorMsg));
    UART2_WRITE(errorMsg, sizeof(errorMsg));
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
}

void print_msg(uint8_t* _msg, uint8_t _len) {
  uint8_t end = 0;
  while (_msg[end] != '\n')
    end++;
  _len = end+1;

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    //uart1_enque_data(_msg, _len);
    //uart2_enque_data(_msg, _len);
    //UART1_WRITE(_msg, _len);
    UART2_WRITE(_msg, _len);
  }
  else {
    //UART1_WRITE(_msg, _len);
    UART2_WRITE(_msg, _len);
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