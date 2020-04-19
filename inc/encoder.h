/**
 * \file encoder.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief Encoder functions used to measure the motor/wheel speed
 */

#pragma once

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
#include <stdint.h>

#include "FreeRTOS.h"

#include "motor.h"

// -------------------------------------------------------------------------- //
// --- Defines
// -------------------------------------------------------------------------- //
#define ENCODER_QUEUE_SIZE 1

#define ENCODER_MEASUREMENT_PERIOD_MS (20) // 20 ms <=> 50 Hz

// -------------------------------------------------------------------------- //
// --- Structs/enums
// -------------------------------------------------------------------------- //
typedef struct Encoder_s
{
    int32_t tick;
    int32_t rpm; // [RPM]
} Encoder_t;

typedef struct Encoders_s
{
    //uint32_t timestamp; // in [ms]
    float timestamp; // [s]
    Encoder_t encoders[N_MOTORS];
} Encoders_t;

typedef struct MotorMeasuredSpeed_s
{
    float timestamp; // in [s]
    float speed[N_MOTORS]; // in [rpm] (or [rad/s])
} MotorMeasuredSpeed_t;

// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //
int init_encoders(void);

uint32_t enc1_get_direction(void);
uint32_t enc2_get_direction(void);

int32_t enc1_get_counts(void);
int32_t enc2_get_counts(void);

int32_t enc1_get_and_reset_counts(void);
int32_t enc2_get_and_reset_counts(void);

int32_t enc1_get_rpm(void);
int32_t enc2_get_rpm(void);

void enc1_get_cts_and_rpm(int32_t* cts, int32_t* rpm);
void enc2_get_cts_and_rpm(int32_t* cts, int32_t* rpm);

void enc_get_cts_and_rpm(int32_t* cts1, int32_t* rpm1, int32_t* cts2, int32_t* rpm2);

void enc_test_task(void* _params);

void encoder_task(void* _params);

uint8_t encoder_read_data(Encoders_t* enc, TickType_t xTicksToWait);

uint8_t encoder_read_motor_measured_speed(MotorMeasuredSpeed_t* data, TickType_t xTicksToWait);