#pragma once

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
#include <stdint.h>

// -------------------------------------------------------------------------- //
// --- Defines
// -------------------------------------------------------------------------- //
#define LEFT_MOTOR (0)
#define RIGHT_MOTOR (1)

#define FORWARD_DIR (1)
#define REVERSE_DIR (0)

#define BRAKE_ON (1)
#define BRAKE_OFF (0)

#define MOTORS_PWM_PRESCALER 0
#define MOTORS_PWM_PERIOD ((180000000/20000)-1) // (180MHz/20kHz) - 1 = 8999 vs (180MHz/40kHz) - 1 = 4499

#define MOTOR_QUEUE_SIZE 1

// -------------------------------------------------------------------------- //
// --- Structs/enums
// -------------------------------------------------------------------------- //
enum {
    MOTOR1=0, MOTOR2, N_MOTORS
} Motors_e;

typedef struct MotorState_s {
    float desiredVoltage;
    uint8_t direction; // = FORWARD_DIR or REVERSE_DIR
    uint16_t pwm;
    //uint16_t dutyCyclei; // Between 0-MOTORS_PWM_PERIOD <-> 0-100 % 
    float dutyCycle; // Between 0-1 %
    //uint8_t brake; // = BRAKE_ON or BRAKE_OFF
    //uint16_t currenti; // [mA]
    float current; // [A]
    uint8_t fault; // 0 = no, 1 = yes
} MotorState_t;

typedef struct Motor_s {
    float timestamp; // [s]
    MotorState_t motors[N_MOTORS];
} Motor_t;

// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //
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

void motor_test_task(void* _params);
void motor_ident_task(void* _params);

void motor_task(void* _params);

int motor_read_motor_data(Motor_t* mot);

int32_t voltageToPwm(float _volt, float _vbat, uint8_t _motor);
float getInputVoltage(void);
float sinusoidSignal(float a, float t);
float squareSignal(float f, float a, float t);
float triangularSignal(float f, float a, float t);