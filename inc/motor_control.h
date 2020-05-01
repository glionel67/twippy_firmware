/**
 * \file motor_control.h
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief Motor control functions
 */

#pragma once

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
#include <stdint.h>

#include "encoder.h"
#include "motor.h"

// -------------------------------------------------------------------------- //
// --- Defines
// -------------------------------------------------------------------------- //
#define MOTOR_CONTROL_PERIOD_MS (20) // in [ms]

#define MOTOR_CONTROL_QUEUE_SIZE (1)

#define ENCODER_QUEUE_SIZE (1)

#define MOTOR_QUEUE_SIZE (1)

// -------------------------------------------------------------------------- //
// --- Structs/enums
// -------------------------------------------------------------------------- //

/**
 * \enum MotorContolMode_e
 * \brief Choices of motor control
 */
enum
{ 
	MOTOR_CTRL_MODE_MANU = 0,
	MOTOR_CTRL_MODE_AUTO,
	MOTOR_CTRL_MODE_BRAKE
} MotorContolMode_e;


// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //

/**
 * \fn motor_control_init
 * \brief Initialize the motor control
 * \return OK if success, NOK otherwise
 */
uint8_t motor_control_init(void);

/**
 * \fn motor_control_reset
 * \brief Reset the PID controller
 */
void motor_control_reset(void);

/**
 * \fn motor_control_task
 * \brief Task for motor control
 */
void motor_control_task(void* _params);

/**
 * \fn motor_control_set_references
 * \brief Set the motor control input references
 * \param ref1: reference for left motor either [rpm] for AUTO or [mV] for MANU
 * \param ref2: reference for right motor either [rpm] for AUTO or [mV] for MANU
 * \param mode: either MANU = 0, AUTO = 1, BRAKE = 2
 */
void motor_control_set_references(int16_t ref1, int16_t ref2, uint8_t mode);

/**
 * \fn get_encoder_data
 * \brief Get the encoder data from the get_encoder_data queue
 */
uint8_t get_encoder_data(Encoders_t* enc, TickType_t xTicksToWait);

/**
 * \fn get_motor_data
 * \brief Get the motor data from the get_motor_data queue
 */
uint8_t get_motor_data(Motor_t* mot, TickType_t xTicksToWait);