/**
 * \file motor_control.c
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief Motor control functions
 */

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
// libC
#include <stdio.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Project
#include "main.h"
#include "adc.h"
#include "motor.h"
#include "motor_control.h"
#include "pid_controller.h"
#include "encoder.h"
#include "usTimer.h"

// -------------------------------------------------------------------------- //
// --- Defines
// -------------------------------------------------------------------------- //
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

// Motor/encoder parameters
static const int32_t gear_ratio = 50; // Motor reductor gear ratio
static const int32_t ppt = 64; // Encoder resolution (Number of point per turn)

// Motor control parameters
static Pid_t pidMotors[N_MOTORS]; // PID structure for each motor
//PidParams_t pidParamsMotors[N_MOTORS];

static uint8_t motorControlMode = MOTOR_CTRL_MODE_MANU;
static int16_t refCtrl[N_MOTORS] = { 0, };

// FreeRTOS queues
static xQueueHandle encoderDataQueue = 0;
static xQueueHandle motorDataQueue = 0;

uint8_t motor_control_init(void)
{
    // Create encoder queue
    encoderDataQueue = xQueueCreate(ENCODER_QUEUE_SIZE, sizeof(Encoders_t));
    if (0 == encoderDataQueue)
    {
        printf("motor_control_init: encoderDataQueue creation NOK\r\n");
        return NOK;
    }
    else
        printf("motor_control_init: encoderDataQueue creation OK\r\n");

    // Create motor queue
    motorDataQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(Motor_t));
    if (0 == motorDataQueue)
    {
        printf("motor_control_init: motorDataQueue creation NOK\r\n");
        return NOK;
    }
    else
        printf("motor_control_init: motorDataQueue creation OK\r\n");

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

    return OK;
} // motor_control_init

void motor_control_reset(void)
{
    pid_reset(&pidMotors[MOTOR1]);
    pid_reset(&pidMotors[MOTOR2]);
} // motor_control_reset

void motor_control_task(void* _params)
{
    if (_params != 0) { }

    int32_t den = 0, num = 0;
    uint64_t usTime = get_us_time();
    float told = (float)usTime * 0.000001f;
    float dt = 0.f;
    uint8_t brakeState = BRAKE_OFF;
    float refVolt[N_MOTORS] = {0, };
    int32_t refPwm[N_MOTORS] = {0, };
    float vbat = 0.f;
    
    Encoders_t encoders;
    Motor_t motors;

    // Reset queues
    memset((void*)&encoders, 0, sizeof(Encoders_t));
    memset((void*)&motors, 0, sizeof(Motor_t));

    // Periodic task
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Get timestamp
        usTime = get_us_time();
        encoders.timestamp = (float)usTime * 0.000001f;
        motors.timestamp = encoders.timestamp;

        // ---------------------------
        // --- 1. Get encoder data ---
        // ---------------------------

        // Read ticks and compute RPM
        taskENTER_CRITICAL();
        encoder_get_counts(&encoders.encoders[MOTOR1].counts, 
                            &encoders.encoders[MOTOR2].counts);
        taskEXIT_CRITICAL();

        // Get time difference
        dt = encoders.timestamp - told;
        told = encoders.timestamp;

        // Convert ticks to RPM
        den = (int32_t)((int32_t)dt * gear_ratio);

        num = (encoders.encoders[MOTOR1].counts * 60000) / ppt;
        encoders.encoders[MOTOR1].rpm = (int16_t)(num / den);
        motors.motors[MOTOR1].encRpm = encoders.encoders[MOTOR1].rpm;

        num = (encoders.encoders[MOTOR2].counts * 60000) / ppt;
        encoders.encoders[MOTOR2].rpm = (int16_t)(num / den);
        motors.motors[MOTOR2].encRpm = encoders.encoders[MOTOR2].rpm;

        // Send encoder data
        xQueueOverwrite(encoderDataQueue, &encoders);

        // -----------------------------
        // --- 2. Get motor currents ---
        // -----------------------------
        get_adc_imot12_ma(&motors.motors[MOTOR1].current, 
                            &motors.motors[MOTOR2].current);

        // ---------------------------
        // --- 3. Get motor faults ---
        // ---------------------------
        motors.motors[MOTOR1].fault = get_fault(MOTOR1);
        motors.motors[MOTOR2].fault = get_fault(MOTOR2);

        // ---------------------------------
        // --- 4. Get control references ---
        // ---------------------------------
        // ... refCtrl[MOTOR1], refCtrl[MOTOR2], motorControlMode

        // -----------------
        // --- 5 control ---
        // -----------------
        if (MOTOR_CTRL_MODE_MANU == motorControlMode)
        {
            // -----------------------
            // --- 5a. PID control ---
            // -----------------------
            brakeState = BRAKE_OFF;

            motors.motors[MOTOR1].refRpm = refCtrl[MOTOR1];
            motors.motors[MOTOR2].refRpm = refCtrl[MOTOR2];

            refVolt[MOTOR1] = pid_update(&pidMotors[MOTOR1], 
                                (float)motors.motors[MOTOR1].refRpm,
                                (float)motors.motors[MOTOR1].encRpm, dt);

            refVolt[MOTOR2] = pid_update(&pidMotors[MOTOR2], 
                                (float)motors.motors[MOTOR2].refRpm,
                                (float)motors.motors[MOTOR2].encRpm, dt);
        }
        else if (MOTOR_CTRL_MODE_AUTO == motorControlMode)
        {
            // --------------------------
            // --- 5b. Direct control ---
            // --------------------------
            brakeState = BRAKE_OFF;
            motors.motors[MOTOR1].refRpm = 0;
            motors.motors[MOTOR2].refRpm = 0;
            // Convert [mV] to [V]
            refVolt[MOTOR1] = (float)(refCtrl[MOTOR1] / 1000.f);
            refVolt[MOTOR2] = (float)(refCtrl[MOTOR2] / 1000.f);
        }
        else // MOTOR_CTRL_MODE_BRAKE == motorControlMode and other
        {
            brakeState = BRAKE_ON;
            motors.motors[MOTOR1].refRpm = 0;
            motors.motors[MOTOR2].refRpm = 0;
            // Convert [mV] to [V]
            refVolt[MOTOR1] = (float)(refCtrl[MOTOR1] / 1000.f);
            refVolt[MOTOR2] = (float)(refCtrl[MOTOR2] / 1000.f);
        } 

        // ----------------------------------
        // --- 6. Send commands to motors ---
        // ----------------------------------

        motors.motors[MOTOR1].refVoltage = (int16_t)(refVolt[MOTOR1] * 1000.f);
        motors.motors[MOTOR2].refVoltage = (int16_t)(refVolt[MOTOR2] * 1000.f);

        // Get current battery voltage [V]
        vbat = get_vbat_volt();

        // Convert voltage to PWM 
        refPwm[MOTOR1] = voltageToPwm(refVolt[MOTOR1], vbat);
        refPwm[MOTOR2] = voltageToPwm(refVolt[MOTOR2], vbat);

        // Set motor PWM and direction
        set_speed(MOTOR1, refPwm[MOTOR1], brakeState, &motors.motors[MOTOR1]);
        set_speed(MOTOR2, refPwm[MOTOR2], brakeState, &motors.motors[MOTOR2]);

        // Send motor data
        xQueueOverwrite(motorDataQueue, &motors);
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
} // motor_control_task

void motor_control_set_references(int16_t ref1, int16_t ref2, uint8_t mode)
{
    refCtrl[MOTOR1] = ref1; // [rpm] or [mV]
    refCtrl[MOTOR2] = ref2; // [rpm] or [mV]

    if (MOTOR_CTRL_MODE_MANU == mode || MOTOR_CTRL_MODE_AUTO == mode)
        motorControlMode = mode;
    else
        motorControlMode = MOTOR_CTRL_MODE_BRAKE; // Default mode is brake
} // motor_control_set_references

uint8_t get_encoder_data(Encoders_t* data, TickType_t xTicksToWait)
{
  return (pdTRUE == xQueueReceive(encoderDataQueue, data, xTicksToWait));
} // get_encoder_data

uint8_t get_motor_data(Motor_t* data, TickType_t xTicksToWait)
{
    return (pdTRUE == xQueueReceive(motorDataQueue, data, xTicksToWait));
} // get_motor_data