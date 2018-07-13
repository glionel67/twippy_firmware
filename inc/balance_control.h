#pragma once

#include <stdint.h>

#define BALANCE_CONTROL_DT  (.02) // 50 Hz

uint8_t balance_control_init(const float dt);
uint8_t balance_control_test(void);

void balance_control_correct_pitch(float pitchActual, 
        float pitchDesired, float* pitchRateDesired, float _dt);

float balance_control_correct_pitch_rate(float pitchRateActual,
        float pitchRateDesired, float _dt);

void balance_control_reset(void);

void balance_control_task(void* _params);