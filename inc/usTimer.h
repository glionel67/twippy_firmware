#pragma once

#include <stdint.h>

int init_us_timer(void);

void delay_us(uint32_t us);

uint64_t get_us_time(void);

void reset_us_timer(void);

void suspend_us_timer(void);

void resume_us_timer(void);

void test_us_timer(void);