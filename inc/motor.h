#pragma once

#include <stdint.h>

#define MOTORS_PWM_PRESCALER	0
#define MOTORS_PWM_PERIOD		((64000000/20000)-1) // =1599=(64MHz/40kHz) - 1

int init_motors(void);

int set_pwm1(uint16_t _pwm);
int set_pwm2(uint16_t _pwm);
int set_pwm12(uint16_t _pwm1, uint16_t _pwm2);

int set_dc_pwm1(float _dc);
int set_dc_pwm2(float _dc);
int set_dc_pwm12(float _dc1, float _dc2);

int set_m1_speed(int32_t _speed);
int set_m2_speed(int32_t _speed);

int set_m1_brake(int32_t _brake);
int set_m2_brake(int32_t _brake);

uint8_t get_m1_fault(void);
uint8_t get_m2_fault(void);

void brake_motor1(void);
void brake_motor2(void);
void brake_motor12(void);

void set_dir_motor1(uint8_t _sens);
void set_dir_motor2(uint8_t _sens);
void set_dir_motor12(uint8_t _sens1, uint8_t _sens2);

void test_motor1(void);
void test_motor2(void);
void test_motor12(void);

