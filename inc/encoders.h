#pragma once

#include <stdint.h>

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
