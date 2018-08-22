#pragma once

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
#include <stdint.h>

// -------------------------------------------------------------------------- //
// --- Defines
// -------------------------------------------------------------------------- //
#define SERVO_PWM_PRESCALER ((180000000/1000000)-1) // (180MHz/1MHz) - 1: to get 1 tick each 1 us
#define SERVO_PWM_PERIOD ((1000000/50)-1) // (180MHz/50Hz) - 1: to get F=50 Hz <-> T=20 ms

#define SERVO_MICROS_MIN    (500) // 500
#define SERVO_MICROS_MAX    (2500) // 2500
#define SERVO_MICROS_MID    (1500) // 1500

#define SERVO_PERIOD_MS (50) // in [ms]

#define SERVO_QUEUE_SIZE (1)

// -------------------------------------------------------------------------- //
// --- Structs/enums
// -------------------------------------------------------------------------- //

// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //
uint8_t servo_init(void);

void servo_task(void* _params);

float degreesToPulseLength(float degrees);

float pulseLengthToDegrees(float pulseLendth);

uint8_t servo_set_us(uint16_t us, uint32_t channel);

uint8_t servo_set_angle(float angle, uint32_t channel);