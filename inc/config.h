#pragma once

/******************************************************************************/
/* FreeRTOS task priorities */
/******************************************************************************/
#define IMU_TASK_PRIORITY 5
#define UART_TASK_PRIORITY 4
#define ENCODER_TASK_PRIORITY 3
#define AHRS_TASK_PRIORITY 5
#define STABILIZER_TASK_PRIORITY 4
#define MOTOR_TASK_PRIORITY 4
#define MOTOR_CONTROL_TASK_PRIORITY 4

/******************************************************************************/
/* FreeRTOS task stack size */
/******************************************************************************/
#define IMU_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)
#define UART_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)
#define ENCODER_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)
#define AHRS_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)
#define STABILIZER_TASK_STACK_SIZE (3*configMINIMAL_STACK_SIZE)
#define MOTOR_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)
#define MOTOR_CONTROL_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)