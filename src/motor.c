/**
 * \file motor.c
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief Motor low-level functions
 */

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
// libC
#include <stdio.h>
#include <string.h>
#include <math.h>

// Project
#include "motor.h"
#include "main.h"

// Timer handler declaration
static TIM_HandleTypeDef TimHandleMotors;

// Timer Output Compare Configuration Structure declaration
static TIM_OC_InitTypeDef sConfigMotors;
static MotorConfig_t motorConfig[N_MOTORS];

uint8_t init_motors(void)
{
    // Init. motor configurations (GPIO)
    motorConfig[MOTOR1].channel = TIM_PWM_MOTOR1_CHANNEL;
    motorConfig[MOTOR1].gpioInA = M1_INA_PIN;
    motorConfig[MOTOR1].gpioInB = M1_INB_PIN;
    motorConfig[MOTOR1].gpioEn = M1_EN_PIN;

    motorConfig[MOTOR2].channel = TIM_PWM_MOTOR2_CHANNEL;
    motorConfig[MOTOR2].gpioInA = M2_INA_PIN;
    motorConfig[MOTOR2].gpioInB = M2_INB_PIN;
    motorConfig[MOTOR2].gpioEn = M2_EN_PIN;

    // Init PWM motor timer handler
    TIM_PWM_MOTORS_CLK_ENABLE();
    TimHandleMotors.Instance = TIM_PWM_MOTORS;
    TimHandleMotors.Init.Prescaler = MOTORS_PWM_PRESCALER;
    TimHandleMotors.Init.Period = MOTORS_PWM_PERIOD;
    TimHandleMotors.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandleMotors.Init.CounterMode = TIM_COUNTERMODE_UP; // TIM_COUNTERMODE_CENTERALIGNED1,2,3
    TimHandleMotors.Init.RepetitionCounter = 0;
    if (HAL_OK != HAL_TIM_PWM_Init(&TimHandleMotors))
    {
        printf("init_motors: HAL_TIM_PWM_Init NOK\r\n");
        return NOK;
    }

    // Init PWM motor timer configuration
    sConfigMotors.OCMode = TIM_OCMODE_PWM1;
    sConfigMotors.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigMotors.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigMotors.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigMotors.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigMotors.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfigMotors.Pulse = 0;
    if (HAL_OK != HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
            motorConfig[MOTOR1].channel | motorConfig[MOTOR2].channel))
    {
        printf("init_motors: HAL_TIM_PWM_ConfigChannel NOK\r\n");
        return NOK;
    }

    if (HAL_OK != HAL_TIM_PWM_Start(&TimHandleMotors,
            motorConfig[MOTOR1].channel | motorConfig[MOTOR2].channel))
    {
        printf("init_motors: HAL_TIM_PWM_Start NOK\r\n");
        return NOK;
    }

    return OK;
} // init_motors

static uint8_t set_pwm(uint8_t motor, uint16_t pwm)
{
    sConfigMotors.Pulse = pwm;

    if (HAL_OK != HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
            motorConfig[motor].channel))
    {
        printf("set_pwm: HAL_TIM_PWM_ConfigChannel NOK\r\n");
        return NOK;
    }

    if (HAL_OK != HAL_TIM_PWM_Start(&TimHandleMotors, motorConfig[motor].channel))
    {
        printf("set_pwm: HAL_TIM_PWM_Start NOK\r\n");
        return NOK;
    }

    return OK;
} // set_pwm

// pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
// where DutyCycle is in percent, between 0 and 100% 
// 25% duty cycle: pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
uint8_t set_duty_cycle(uint8_t motor, float dc)
{
    uint16_t pwm = (uint16_t)round(dc * (float)MOTORS_PWM_PERIOD);

    if (OK == set_pwm(motor, pwm))
        return OK;
    else
        return NOK;
} // set_duty_cycle

uint8_t set_speed(uint8_t motor, int32_t speed, uint8_t brake, MotorState_t* motState)
{
    // 1. Set motor direction
    uint8_t dir = BRAKE_DIR;

    if (BRAKE_ON == brake)
        dir = BRAKE_DIR;
    else if (speed < 0)
        dir = REVERSE_DIR;
    else // speed >= 0
        dir = FORWARD_DIR;

    set_dir(motor, dir);

    // 2. Set motor PWM
    if (speed < 0) // Make the speed positive if necessary
        speed = -speed;
    if (speed > MOTORS_PWM_PERIOD) // Max PWM period
        speed = MOTORS_PWM_PERIOD;

    // 3. Update motor state struct fields
    motState->direction = dir;
    motState->pwm = (uint16_t)speed;
    motState->dutyCycle = (uint16_t)(speed / MOTORS_PWM_PERIOD);

    return set_pwm(motor, speed);
} // set_speed

uint8_t get_fault(uint8_t motor)
{
    return !HAL_GPIO_ReadPin(MOTORS_GPIO, motorConfig[motor].gpioEn); // 1 = fault
} // get_fault

void set_dir(uint8_t motor, uint8_t dir)
{
    if (FORWARD_DIR == dir)
    {
        HAL_GPIO_WritePin(MOTORS_GPIO, motorConfig[motor].gpioInA, 1);
        HAL_GPIO_WritePin(MOTORS_GPIO, motorConfig[motor].gpioInB, 0);
    }
    else if (REVERSE_DIR == dir)
    {
        HAL_GPIO_WritePin(MOTORS_GPIO, motorConfig[motor].gpioInA, 0);
        HAL_GPIO_WritePin(MOTORS_GPIO, motorConfig[motor].gpioInB, 1);
    }
    else // (BRAKE_DIR == dir)
    {
        HAL_GPIO_WritePin(MOTORS_GPIO, motorConfig[motor].gpioInA, 0); // Make the motor coast no
        HAL_GPIO_WritePin(MOTORS_GPIO, motorConfig[motor].gpioInB, 0); // matter which direction it is spinning.
    }
} // set_dir

/*void motor_ident_task(void* _params)
{
    MotorMeasuredSpeed_t motorMeasuredSpeeds;
    uint8_t ret = 0;
    float t = 0.f;
    float vBat = NOMINAL_BATTERY_VOLTAGE; // [V]
    int32_t speeds[N_MOTORS] = { 0, };

    if (_params != 0) { }

    memset((void*)&motorMeasuredSpeeds, 0, sizeof(MotorMeasuredSpeed_t));

    while (1)
    {
        ret = encoder_read_encoder_speed(&motorMeasuredSpeeds, 
                pdMS_TO_TICKS(ENCODER_MEASUREMENT_PERIOD_MS));
        if (ret)
        {
            printf("%3.3f,%3.3f,%3.3f\r\n",
                inputVoltage,
                motorMeasuredSpeeds.speed[MOTOR1],
                motorMeasuredSpeeds.speed[MOTOR2]);

            t = (float)get_us_time() * (float)1e-6;
            //inputVoltage = triangularSignal(.1, vBat, t);
            //inputVoltage = squareSignal(.1, vBat, t);
            inputVoltage = sinusoidSignal(3.1f, t);
            speeds[MOTOR1] = voltageToPwm(inputVoltage, vBat, MOTOR1);
            speeds[MOTOR2] = voltageToPwm(inputVoltage, vBat, MOTOR2);
            set_speed(MOTOR1, speeds[MOTOR1], BRAKE_OFF);
            set_speed(MOTOR2, speeds[MOTOR2], BRAKE_OFF);

            motors.timestamp = t;
            xQueueOverwrite(motorQueue, &motors);
        }
    }

    vTaskDelete(NULL);
} // motor_ident_task*/

int32_t voltageToPwm(float volt, float vbat)
{
    float percentage = volt / vbat;
    percentage = (percentage > 1.f) ? 1.f : percentage;
    percentage = (percentage < -1.f) ? -1.f : percentage;
    float pwmf = percentage * (float)MOTORS_PWM_PERIOD;
    return (int32_t)round(pwmf);
} // voltageToPwm

float sinusoidSignal(float a, float t)
{
    return a * (
        sin(2.f*M_PI*t) + sin(2.f*M_PI*.1f*t) + 
        sin(2.f*M_PI*.2f*t) + sin(2.f*M_PI*.3f*t) + 
        sin(2.f*M_PI*.4f*t) + sin(2.f*M_PI*.5f*t));
}

float squareSignal(float f, float a, float t)
{
    float voltage = sinf(2.f*M_PI*f*t);
    if (voltage >= 0.f)
        voltage = a;
    else
        voltage = 0.;
    return voltage;
}

float triangularSignal(float f, float a, float t)
{
    return (2.f * a / M_PI) * asinf(sinf(2.f*M_PI*f*t));
}