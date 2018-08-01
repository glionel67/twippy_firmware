#pragma once

#include <stdint.h>

#include "FreeRTOS.h"

#include "motor.h"


#define ENCODER_QUEUE_SIZE 1

#define ENCODER_MEASUREMENT_PERIOD_MS (20) // 20 ms <=> 50 Hz


typedef struct Encoder_s {
    int32_t tick;
    int32_t rpm;
} Encoder_t;

typedef struct Encoders_s {
    //uint32_t timestamp;
    float timestamp; // [s]
    Encoder_t encoders[N_MOTORS];
} Encoders_t;


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
