/**
 * \file motor.h
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief Motor low-level functions
 */

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

#define BRAKE_OFF (0)
#define BRAKE_ON (1)

#define MOTORS_PWM_PRESCALER 0
#define MOTORS_PWM_PERIOD ((180000000/20000)-1) // (180MHz/20kHz) - 1 = 8999 vs (180MHz/40kHz) - 1 = 4499

// -------------------------------------------------------------------------- //
// --- Structs/enums
// -------------------------------------------------------------------------- //

/**
 * \enum Dirs_e
 * \brief Possible motor directions
 */
enum
{
    REVERSE_DIR=0,
    FORWARD_DIR,
    BRAKE_DIR
} Dirs_e;

/**
 * \enum Motors_e
 * \brief List of motors
 */
enum
{
    MOTOR1=0,
    MOTOR2,
    N_MOTORS
} Motors_e;

/**
 * \struct MotorConfig_t
 * \brief Motor configuration
 */
typedef struct MotorConfig_s
{
    uint32_t channel; // PWM channel output
    uint16_t gpioInA; // GPIO number for INA
    uint16_t gpioInB; // GPIO number for INB
    uint16_t gpioEn; // GPIO number for enable
} MotorConfig_t;

/**
 * \struct MotorState_t
 * \brief Motor state information
 */
typedef struct MotorState_s
{
    int16_t encRpm; // Measured speed from encoder [RPM]
    int16_t refRpm; // Reference speed [RPM]
    int16_t refVoltage; // Applied voltage [mV]
    uint8_t direction; // = FORWARD_DIR, REVERSE_DIR or BRAKE_DIR
    uint16_t pwm;
    uint16_t dutyCycle; // Between 0-MOTORS_PWM_PERIOD <-> 0-100 %
    //uint8_t brake; // = BRAKE_ON or BRAKE_OFF
    int16_t current; // [mA]
    uint8_t fault; // 0 = no, 1 = yes
} MotorState_t;

/**
 * \struct Motor_t
 * \brief Structure containing the states of the left and right motors
 */
typedef struct Motor_s
{
    float timestamp; // [s]
    //uint32_t timestamp; // [ms]
    MotorState_t motors[N_MOTORS];
} Motor_t;

// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //

/**
 * \fn init_motors
 * \brief Initialize the left and right motor
 * \return OK if success, NOK otherwise
 */
uint8_t init_motors(void);

/**
 * \fn set_duty_cycle
 * \brief Set the duty cycle for a given motor
 * \param motor: index of the motor
 * \param dc: duty cycle to set
 * \return OK if success, NOK otherwise
 */
uint8_t set_duty_cycle(uint8_t motor, float dc);

/**
 * \fn set_speed
 * \brief Set the desired speed for a given motor
 * \param motor: index of the motor
 * \param speed: speed to set
 * \param brake: activate brake or not
 * \param motState: motor state struct to update the fields
 * \return OK if success, NOK otherwise
 */
uint8_t set_speed(uint8_t motor, int32_t speed, uint8_t brake, MotorState_t* motState);

/**
 * \fn set_dir
 * \brief Set the desired rotation direction for a given motor
 * \param motor: index of the motor
 * \param dir: direction to set: REVERSE_DIR=0, FORWARD_DIR, BRAKE_DIR
 */
void set_dir(uint8_t motor, uint8_t dir);

/**
 * \fn get_fault
 * \brief Get the fault state of a motor
 * \param motor: index of the motor
 * \return the fault state 0=OK, 1=NOK
 */
uint8_t get_fault(uint8_t motor);

/**
 * \fn voltageToPwm
 * \brief Convert a reference voltage to a PWM
 * \param volt: motor voltage [V]
 * \param vbat: battery voltage [V]
 * \return the PWM
 */
int32_t voltageToPwm(float volt, float vbat);

float sinusoidSignal(float a, float t);
float squareSignal(float f, float a, float t);
float triangularSignal(float f, float a, float t);