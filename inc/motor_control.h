#pragma once

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
#include <stdint.h>

#include "motor.h"

// -------------------------------------------------------------------------- //
// --- Defines
// -------------------------------------------------------------------------- //
#define MOTOR_CONTROL_PERIOD_MS (20) // in [ms]

#define MOTOR_CONTROL_QUEUE_SIZE (1)

// -------------------------------------------------------------------------- //
// --- Structs/enums
// -------------------------------------------------------------------------- //
typedef struct ControlState_s {
    float des; // in [rpm] or [rad/s] ?
    float mes; // in [rpm] or [rad/s] ?
    float out; // Output of the PID
} ControlState_t;

typedef struct MotorControl_s {
    float timestamp;
    ControlState_t controlState[N_MOTORS];
} MotorControl_t;

typedef struct MotorDesiredVoltage_s {
    float timestamp;
    float voltage[N_MOTORS];
} MotorDesiredVoltage_t;

// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //
uint8_t motor_control_init(void);

uint8_t motor_control_test(void);

void motor_control_reset(void);

void motor_control_task(void* _params);

void motor_control_set_desired_wheel_speeds(float _rpm1, float _rpm2);

uint8_t motor_control_read_data(MotorControl_t* data, TickType_t xTicksToWait);

uint8_t motor_control_read_desired_voltage(MotorDesiredVoltage_t* data, TickType_t xTicksToWait);