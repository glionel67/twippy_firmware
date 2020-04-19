/**
 * \file buzzer.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief Buzzer functions
 */

#pragma once

#include <stdint.h>

int init_buzzer(void);
int set_buzzer_freq(uint32_t _freq);
int set_buzzer_dutyCycle(uint16_t _freq);
int turn_on_buzzer(void);
int turn_off_buzzer(void);
void test_buzzer(void);
void buzzer_task(void* _params);