/**
 * \file main.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief main file
 */

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //

// Project
#include "main.h"
#include "config.h"
#include "gpio.h"
#include "uart1.h"
#include "uart2.h"
#include "uart3.h"
#include "i2c1.h"
#include "i2c2.h"
#include "spi.h"
#include "usTimer.h"
#include "imu.h"
#include "ahrs.h"
#include "motor_control.h"
#include "adc.h"
#include "servo.h"
#include "led.h"
#include "buzzer.h"
#include "mavlink_uart.h"
#include "gps.h"
#include "battery.h"
#include "state_machine.h"

// libC
#include <stdio.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //

/**
 * \fn SystemClock_Config
 * \brief Configure system clock to 180 MHz
 */
void SystemClock_Config(void);

// ---------------------------------------------------------------------------//
// --- Main
// ---------------------------------------------------------------------------//
int main(void)
{
  int ret = 0;

  // ------------------------------------------------------------------------ //
  // --- Init HAL
  // ------------------------------------------------------------------------ //
  ret = HAL_Init(); // Init systick
  if (HAL_OK != ret)
    Error_Handler();

  // Configure system clock to 180 MHz
  SystemClock_Config();

  // ------------------------------------------------------------------------ //
  // --- Init GPIO
  // ------------------------------------------------------------------------ //
  ret = init_gpios();
  if (OK != ret)
    Error_Handler();

  // ------------------------------------------------------------------------ //
  // --- Init LED
  // ------------------------------------------------------------------------ //
  ret = led_init();
  if (OK != ret)
    Error_Handler();

  // ------------------------------------------------------------------------ //
  // --- Init UART
  // ------------------------------------------------------------------------ //
  ret = uart2_init(); // USB COM with PC/RPi
  if (OK != ret)
    Error_Handler();

  HAL_Delay(5000);

  ret = uart1_init(); // GPS
  if (OK != ret)
  {
    printf("uart1_init NOK\r\n");
    Error_Handler();
  }
  else
    printf("uart1_init OK\r\n");

  ret = uart3_init(); // RF COM Bluetooth/WiFi
  if (OK != ret)
  {
    printf("uart3_init NOK\r\n");
    Error_Handler();
  }
  else
    printf("uart3_init OK\r\n");

  printf("\r\n--- Hello Twippy! ---\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init I2C
  // ------------------------------------------------------------------------ //
  ret = i2c1_init(); // RPi
  if (OK != ret)
  {
    printf("i2c1_init NOK\r\n");
    Error_Handler();
  }
  else
    printf("i2c1_init OK\r\n");

  ret = i2c2_init(); // Distance sensor
  if (OK != ret)
  {
    printf("i2c2_init NOK\r\n");
    Error_Handler();
  }
  else
    printf("i2c2_init OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init SPI
  // ------------------------------------------------------------------------ //
  ret = spi1_init(); // IMU MPU9250
  if (OK != ret)
  {
    printf("spi1_init NOK\r\n");
    Error_Handler();
  }
  else
    printf("spi1_init OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init microsecond timer
  // ------------------------------------------------------------------------ //
  ret = init_us_timer();
  if (OK != ret)
  {
    printf("init_us_timer NOK\r\n");
    Error_Handler();
  }
  else
    printf("init_us_timer OK\r\n");

  test_us_timer();

  // ------------------------------------------------------------------------ //
  // --- Init buzzer
  // ------------------------------------------------------------------------ //
  ret = init_buzzer();
  if (OK != ret)
  {
    printf("init_buzzer NOK\r\n");
    Error_Handler();
  }
  else
    printf("init_buzzer OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init ADC
  // ------------------------------------------------------------------------ //
  ret = init_adc();
  if (OK != ret)
  {
    printf("init_adc NOK\r\n");
    Error_Handler();
  }
  else
    printf("init_adc OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init encoder
  // ------------------------------------------------------------------------ //
  ret = init_encoders();
  if (OK != ret)
  {
    printf("init_encoders NOK\r\n");
    Error_Handler();
  }
  else
    printf("init_encoders OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init motor
  // ------------------------------------------------------------------------ //
  ret = init_motors();
  if (OK != ret)
  {
    printf("init_motors NOK\r\n");
    Error_Handler();
  }
  else
    printf("init_motors OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init motor control
  // ------------------------------------------------------------------------ //
  ret = motor_control_init();
  if (OK != ret)
  {
    printf("motor_control_init NOK\r\n");
    Error_Handler();
  }
  else
    printf("motor_control_init OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init IMU
  // ------------------------------------------------------------------------ //
  ret = init_imu();
  if (OK != ret)
  {
    printf("init_imu NOK\r\n");
    Error_Handler();
  }
  else
    printf("init_imu OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init Mavlink
  // ------------------------------------------------------------------------ //
  ret = mavlinkInit();
  if (OK != ret)
  {
    printf("mavlinkInit NOK\r\n");
    Error_Handler();
  }
  else
    printf("mavlinkInit OK\r\n");

  ret = mavlinkStart();
  if (OK != ret)
  {
    printf("mavlinkStart NOK\r\n");
    Error_Handler();
  }
  else
    printf("mavlinkStart OK\r\n");

  // ------------------------------------------------------------------------ //
  // --- Init GPS
  // ------------------------------------------------------------------------ //
  ret = gps_init();
  if (OK != ret)
  {
    printf("gps_init NOK\r\n");
    Error_Handler();
  }
  else
    printf("gps_init OK\r\n");

  ret = gps_start();
  if (OK != ret)
  {
    printf("gps_start NOK\r\n");
    Error_Handler();
  }
  else
    printf("gps_start OK\r\n");

  uint32_t hclkFreq = HAL_RCC_GetHCLKFreq();
  uint32_t pclk1Freq = HAL_RCC_GetPCLK1Freq();
  uint32_t pclk2Freq = HAL_RCC_GetPCLK2Freq();
  printf("hclkFreq=%lu, pclk1Freq=%lu, pclk2Freq=%lu\r\n", hclkFreq, pclk1Freq, pclk2Freq);

  // ------------------------------------------------------------------------ //
  // --- Create FreeRTOS tasks
  // ------------------------------------------------------------------------ //
  if (!(pdPASS == xTaskCreate(led_task, (const char*)"led_task",
    LED_TASK_STACK_SIZE, NULL, LED_TASK_PRIORITY, NULL)))
  {
    printf("Failed to create led_task\r\n");
    goto hell;
  }

  // if (!(pdPASS == xTaskCreate(uart1_task, (const char*)"uart1_task",
  //   UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL))) {
  //   printf("Failed to create uart1_task\r\n");
  //   goto hell;
  // }

  // if (!(pdPASS == xTaskCreate(uart2_task, (const char*)"uart2_task",
  //   UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL))) {
  //   printf("Failed to create uart2_task\r\n");
  //   goto hell;
  // }

  // if (!(pdPASS == xTaskCreate(uart3_task, (const char*)"uart3_task",
  //   UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL))) {
  //   printf("Failed to create uart3_task\r\n");
  //   goto hell;
  // }

  if (!(pdPASS == xTaskCreate(battery_task, (const char*)"battery_task",
    BATTERY_TASK_STACK_SIZE, NULL, BATTERY_TASK_PRIORITY, NULL)))
  {
    printf("Failed to create battery_task\r\n");
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(imu_task, (const char*)"imu_task",
    IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, NULL)))
  {
    printf("Failed to create imu_task\r\n");
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(imu_calibrate_gyro_bias_task, 
      (const char*)"imu_calibrate_gyro_bias_task",
      IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, NULL)))
  {
    printf("Failed to create imu_calibrate_gyro_bias_task\r\n");
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(ahrs_task, (const char*)"ahrs_task",
    AHRS_TASK_STACK_SIZE, NULL, AHRS_TASK_PRIORITY, NULL)))
  {
    printf("Failed to create ahrs_task\r\n");
    goto hell;
  }

  if (!(pdPASS == xTaskCreate(motor_control_task, (const char*)"motor_control_task",
    MOTOR_CONTROL_TASK_STACK_SIZE, NULL, MOTOR_CONTROL_TASK_PRIORITY, NULL)))
  {
    printf("Failed to create motor_control_task\r\n");
    goto hell;
  }

  /*
  if (!(pdPASS == xTaskCreate(servo_task, (const char*)"servo_task",
    SERVO_TASK_STACK_SIZE, NULL, SERVO_TASK_STACK_SIZE, NULL))) {
    printf("Failed to create servo_task\r\n");
    goto hell;
  }
  */

  // Start FreeRTOS scheduler
  vTaskStartScheduler();

  while (1)
  {
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

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE(); // Enable Power Control clock

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Enable HSI Oscillator and activate PLL with HSI as source
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; //0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if (HAL_OK != HAL_RCC_OscConfig(&RCC_OscInitStruct))
    Error_Handler();

  // Activate the OverDrive to reach the 180 MHz Frequency
  if (HAL_OK != HAL_PWREx_EnableOverDrive())
    Error_Handler();

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | 
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4; // To have a 45 MHz clock
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // To have a 90 MHz clock
  if (HAL_OK != HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5))
    Error_Handler();
} // SystemClock_Config

void Error_Handler(void)
{
  led_set_period(LED1, 100000000);
  led_set_period(LED2, 100000000);
  led_set_period(LED3, 100000000);

  while (1)
  {
    printf("ERROR!\r\n");
    HAL_Delay(1000);
    led_toggle(LED1);
    led_toggle(LED2);
    led_toggle(LED3);
  }
} // Error_Handler