/**
 * \file motor_control.c
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief Motor control functions
 */

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "main.h"
#include "pid_controller.h"
#include "motor.h"
#include "motor_control.h"
#include "encoder.h"
#include "usTimer.h"

#define DEBUG_MODULE "motor_ctrl"

// PID controller parameters
#define PID_MOTOR1_KP (.5f)
#define PID_MOTOR1_KI (5.f)
#define PID_MOTOR1_KD (.01f)
#define PID_MOTOR1_KFFWD (1.f)
#define PID_MOTOR1_SAT (12.5f) // in [volt]
#define PID_MOTOR1_ISAT (PID_MOTOR1_SAT)

#define PID_MOTOR2_KP (PID_MOTOR1_KP)
#define PID_MOTOR2_KI (PID_MOTOR1_KI)
#define PID_MOTOR2_KD (PID_MOTOR1_KD)
#define PID_MOTOR2_KFFWD (1.f)
#define PID_MOTOR2_SAT (PID_MOTOR1_SAT) // in [volt]
#define PID_MOTOR2_ISAT (PID_MOTOR1_ISAT)

#define LOW_SPEED_SAT_RPM (5.f) // in RPM
#define RPM_PER_VOLT (17.f)

static Pid_t pidMotors[N_MOTORS]; // PID structure for each motor
//PidParams_t pidParamsMotors[N_MOTORS];
static uint8_t isInit = 0;
static float desiredWheelSpeed[N_MOTORS] = { 0, };
//static xQueueHandle motorControlQueue = 0;
static xQueueHandle motorDesiredVoltageQueue = 0;

uint8_t motor_control_init(void)
{
    if (isInit)
        return OK;

    /*
    motorControlQueue = xQueueCreate(MOTOR_CONTROL_QUEUE_SIZE, sizeof(MotorControls_t));
    if (motorControlQueue == 0) {
        char str[] = "motors_control_init: motorControlQueue creation NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return -1;
    }
    else {
        char str[] = "motors_control_init: motorControlQueue creation OK\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }
    */
    motorDesiredVoltageQueue = xQueueCreate(MOTOR_CONTROL_QUEUE_SIZE, sizeof(MotorDesiredVoltage_t));
    if (0 == motorDesiredVoltageQueue)
    {
        printf("motors_control_init: motorDesiredVoltageQueue creation NOK\r\n");
        return NOK;
    }
    else
        printf("motors_control_init: motorDesiredVoltageQueue creation OK\r\n");

    // Init PID controller for each motor
    float dt = (float)MOTOR_CONTROL_PERIOD_MS / 1000.f;
    PidParams_t pidParams;
    pidParams.dt = dt;

    // Motor 1
    pidParams.kffwd = PID_MOTOR1_KFFWD;
    pidParams.kp = PID_MOTOR1_KP;
    pidParams.ki = PID_MOTOR1_KI;
    pidParams.kd = PID_MOTOR1_KD;
    pidParams.sat = PID_MOTOR1_SAT;
    pidParams.isat = PID_MOTOR1_ISAT;
    pid_init(&pidMotors[MOTOR1], &pidParams);

    // Motor 2
    pidParams.kffwd = PID_MOTOR2_KFFWD;
    pidParams.kp = PID_MOTOR2_KP;
    pidParams.ki = PID_MOTOR2_KI;
    pidParams.kd = PID_MOTOR2_KD;
    pidParams.sat = PID_MOTOR2_SAT;
    pidParams.isat = PID_MOTOR2_ISAT;
    pid_init(&pidMotors[MOTOR2], &pidParams);

    isInit = 1;
    return OK;
}

uint8_t motor_control_test(void)
{
    return isInit;
}

void motor_control_reset(void)
{
    pid_reset(&pidMotors[MOTOR1]);
    pid_reset(&pidMotors[MOTOR2]);
}

void motor_control_task(void* _params)
{
    uint8_t ret = 0;
    MotorMeasuredSpeed_t measuredSpeeds;
    MotorDesiredVoltage_t desiredVoltages;
    MotorControl_t motorControls;

    float dt = 0, told = (float)get_us_time() * (float)1e-6;

    if (_params != 0) { }

    memset((void*)&measuredSpeeds, 0, sizeof(MotorMeasuredSpeed_t));
    memset((void*)&desiredVoltages, 0, sizeof(MotorDesiredVoltage_t));
    memset((void*)&motorControls, 0, sizeof(MotorControl_t));

    while (1)
    {
        // 1. Get encoder data
        ret = encoder_read_motor_measured_speed(&measuredSpeeds, 
                pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
        if (ret)
        {
            //motorControls.timestamp = measuredSpeeds.timestamp;
            motorControls.controlState[MOTOR1].mes = measuredSpeeds.speed[MOTOR1];
            motorControls.controlState[MOTOR2].mes = measuredSpeeds.speed[MOTOR2];
        }
        else
        {
            //motorControls.timestamp = (float)get_us_time() * (float)1e-6;
        }

        // 2. Get reference speeds
        // ... read from a queue without blocking
        //ret = read_desired_wheel_speed(&desiredWheelSpeeds, 0);
        motorControls.controlState[MOTOR1].des = desiredWheelSpeed[MOTOR1];
        motorControls.controlState[MOTOR2].des = desiredWheelSpeed[MOTOR2];

        // 3. PID control
        motorControls.timestamp = (float)get_us_time() * (float)1e-6;
        dt = motorControls.timestamp - told;
        told = motorControls.timestamp;

        if (desiredWheelSpeed[MOTOR1] < LOW_SPEED_SAT_RPM && 
            desiredWheelSpeed[MOTOR1] > -LOW_SPEED_SAT_RPM)
        {
            motorControls.controlState[MOTOR1].out = desiredWheelSpeed[MOTOR1] / RPM_PER_VOLT;
            //pid_reset(&pidMotors[MOTOR1]);
        }
        else
        {
            motorControls.controlState[MOTOR1].out = 
                pid_update(&pidMotors[MOTOR1], motorControls.controlState[MOTOR1].des,
                           motorControls.controlState[MOTOR1].mes, dt);
        }

        if (desiredWheelSpeed[MOTOR2] < LOW_SPEED_SAT_RPM && 
            desiredWheelSpeed[MOTOR2] > -LOW_SPEED_SAT_RPM)
        {
            motorControls.controlState[MOTOR2].out = desiredWheelSpeed[MOTOR2] / RPM_PER_VOLT;
            //pid_reset(&pidMotors[MOTOR2]);
        }
        else
        {
            motorControls.controlState[MOTOR2].out = 
                pid_update(&pidMotors[MOTOR2], motorControls.controlState[MOTOR2].des,
                           motorControls.controlState[MOTOR2].mes, dt);
        }

        if (desiredWheelSpeed[MOTOR1] * motorControls.controlState[MOTOR1].out < 0.f)
            motorControls.controlState[MOTOR1].out = 0.f;
        
        if (desiredWheelSpeed[MOTOR2] * motorControls.controlState[MOTOR2].out < 0.f)
            motorControls.controlState[MOTOR2].out = 0.f;

        //xQueueOverwrite(motorControlQueue, &motorControls); // Send debug data

        // 4. Send voltage commands to motors
        desiredVoltages.timestamp = motorControls.timestamp;
        desiredVoltages.voltage[MOTOR1] = motorControls.controlState[MOTOR1].out;
        desiredVoltages.voltage[MOTOR2] = motorControls.controlState[MOTOR2].out;
        xQueueOverwrite(motorDesiredVoltageQueue, &desiredVoltages);
    }

    vTaskDelete(NULL);
} // motor_control_task

void motor_control_test_task(void* _params)
{
    uint8_t ret = 0;
    MotorMeasuredSpeed_t measuredSpeeds;
    MotorDesiredVoltage_t desiredVoltages;
    MotorControl_t motorControls;
    float dt = 0, told = (float)get_us_time() * (float)1e-6;

    if (_params != 0) { }

    memset((void*)&measuredSpeeds, 0, sizeof(MotorMeasuredSpeed_t));
    memset((void*)&desiredVoltages, 0, sizeof(MotorDesiredVoltage_t));
    memset((void*)&motorControls, 0, sizeof(MotorControl_t));

    vTaskDelay(3000);
    float t0 = (float)get_us_time() * (float)1e-6;

    while (1)
    {
        // 1. Get encoder data
        ret = encoder_read_motor_measured_speed(&measuredSpeeds, 
                pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
        if (ret)
        {
            //motorControls.timestamp = measuredSpeeds.timestamp;
            motorControls.controlState[MOTOR1].mes = measuredSpeeds.speed[MOTOR1];
            motorControls.controlState[MOTOR2].mes = measuredSpeeds.speed[MOTOR2];
        }
        else
        {
            //motorControls.timestamp = (float)get_us_time() * (float)1e-6;
        }

        motorControls.timestamp = (float)get_us_time() * (float)1e-6;

        // 2. Get reference speeds
        desiredWheelSpeed[MOTOR1] = triangularSignal(.1, 200.f, motorControls.timestamp-t0);
        //desiredWheelSpeed[MOTOR1] = squareSignal(.1, 150.f, motorControls.timestamp-t0);
        //desiredWheelSpeed[MOTOR1] = sinusoidSignal(40.f, motorControls.timestamp-t0);
        desiredWheelSpeed[MOTOR2] = desiredWheelSpeed[MOTOR1];
        motorControls.controlState[MOTOR1].des = desiredWheelSpeed[MOTOR1];
        motorControls.controlState[MOTOR2].des = desiredWheelSpeed[MOTOR2];

        // 3. PID control
        dt = motorControls.timestamp - told;
        told = motorControls.timestamp;
        //motorControls.controlState[MOTOR1].out = 
        //    pid_update(&pidMotors[MOTOR1], motorControls.controlState[MOTOR1].des, motorControls.controlState[MOTOR1].mes, dt);
        //motorControls.controlState[MOTOR2].out = 
        //    pid_update(&pidMotors[MOTOR2], motorControls.controlState[MOTOR2].des, motorControls.controlState[MOTOR2].mes, dt);

        if (desiredWheelSpeed[MOTOR1] < LOW_SPEED_SAT_RPM && 
            desiredWheelSpeed[MOTOR1] > -LOW_SPEED_SAT_RPM)
        {
            motorControls.controlState[MOTOR1].out = desiredWheelSpeed[MOTOR1] / RPM_PER_VOLT;
            //pid_reset(&pidMotors[MOTOR1]);
        }
        else 
        {
            motorControls.controlState[MOTOR1].out = 
                pid_update(&pidMotors[MOTOR1], motorControls.controlState[MOTOR1].des,
                           motorControls.controlState[MOTOR1].mes, dt);
        }

        if (desiredWheelSpeed[MOTOR2] < LOW_SPEED_SAT_RPM && 
            desiredWheelSpeed[MOTOR2] > -LOW_SPEED_SAT_RPM)
        {
            motorControls.controlState[MOTOR2].out = desiredWheelSpeed[MOTOR2] / RPM_PER_VOLT;
            //pid_reset(&pidMotors[MOTOR2]);
        }
        else
        {
            motorControls.controlState[MOTOR2].out = 
                pid_update(&pidMotors[MOTOR2], motorControls.controlState[MOTOR2].des,
                           motorControls.controlState[MOTOR2].mes, dt);
        }

        if (desiredWheelSpeed[MOTOR1] * motorControls.controlState[MOTOR1].out < 0)
            motorControls.controlState[MOTOR1].out = 0.f;
        
        if (desiredWheelSpeed[MOTOR2] * motorControls.controlState[MOTOR2].out < 0)
            motorControls.controlState[MOTOR2].out = 0.f;

        //xQueueOverwrite(motorControlQueue, &motorControls); // Send debug data

        // 4. Send voltage commands to motors
        desiredVoltages.timestamp = motorControls.timestamp;
        desiredVoltages.voltage[MOTOR1] = motorControls.controlState[MOTOR1].out;
        desiredVoltages.voltage[MOTOR2] = motorControls.controlState[MOTOR2].out;
        xQueueOverwrite(motorDesiredVoltageQueue, &desiredVoltages);

        printf("%3.3f,%3.3f,%3.3f\r\n",
            desiredWheelSpeed[MOTOR1], desiredVoltages.voltage[MOTOR1],
            measuredSpeeds.speed[MOTOR1]);
    }

    vTaskDelete(NULL);
}

void motor_control_set_desired_wheel_speeds(float _speed1, float _speed2)
{
    desiredWheelSpeed[MOTOR1] = _speed1;
    desiredWheelSpeed[MOTOR2] = _speed2;
}

//uint8_t motor_control_read_data(MotorControl_t* data, TickType_t xTicksToWait) {
//    return (pdTRUE == xQueueReceive(motorControlQueue, data, xTicksToWait));
//}

uint8_t motor_control_read_desired_voltage(MotorDesiredVoltage_t* data, 
        TickType_t xTicksToWait)
{
    return (pdTRUE == xQueueReceive(motorDesiredVoltageQueue, data, xTicksToWait));
}
