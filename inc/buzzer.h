#pragma once

#include <stdint.h>


int init_buzzer(void);
int setBuzzerFreq(uint32_t _freq);
int setBuzzerDutyCycle(uint16_t _freq);
int buzzerTurnOn(void);
int buzzerTurnOff(void);

void testBuzzer(void);