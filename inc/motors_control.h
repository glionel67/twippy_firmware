#pragma once

#include <stdint.h>

#include "motor.h"

#define MOTOR_CONTROL_PERIOD_MS (20) // in [ms]

#define MOTOR_CONTROL_QUEUE_SIZE (1)

typedef struct MotorControl_s {
    float ref; // in [rpm] or [rad/s] ?
    float mes; // in [rpm] or [rad/s] ?
    float out; // Output of the PID
} MotorControl_t;

typedef struct MotorControls_s {
    float timestamp;
    MotorControl_t motorControls[N_MOTORS];
} MotorControls_t;

uint8_t motors_control_init(const float dt);
uint8_t motors_control_test(void);

void motors_control_reset(void);

void motors_control_task(void* _params);

void motors_control_set_ref_speeds(float _rpm1, float _rpm2);

uint8_t motors_control_read_data(MotorControls_t* data, TickType_t xTicksToWait);