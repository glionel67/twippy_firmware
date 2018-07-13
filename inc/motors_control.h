#pragma once

#include <stdint.h>

uint8_t motors_control_init(const float dt);
uint8_t motors_control_test(void);

void motors_control_reset(void);

void motors_control_task(void* _params);
