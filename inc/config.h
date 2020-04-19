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
#define SERVO_TASK_PRIORITY 1
#define LED_TASK_PRIORITY 1
#define BATTERY_TASK_PRIORITY 1

#define MAVLINK_READ_TASK_PRIORITY 2
#define MAVLINK_WRITE_TASK_PRIORITY 2

#define GPS_TASK_PRIORITY 2

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
#define SERVO_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)
#define LED_TASK_STACK_SIZE (1*configMINIMAL_STACK_SIZE)
#define BATTERY_TASK_STACK_SIZE (1*configMINIMAL_STACK_SIZE)

#define MAVLINK_READ_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)
#define MAVLINK_WRITE_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)

#define GPS_TASK_STACK_SIZE (2*configMINIMAL_STACK_SIZE)