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

#define PID_MOTOR1_KP (.5)
#define PID_MOTOR1_KI (5.)
#define PID_MOTOR1_KD (.01)
#define PID_MOTOR1_SAT (210.) // in [rpm]
#define PID_MOTOR1_ISAT (400.)

#define PID_MOTOR2_KP (.5)
#define PID_MOTOR2_KI (5.)
#define PID_MOTOR2_KD (.01)
#define PID_MOTOR2_SAT (210.) // in [rpm]
#define PID_MOTOR2_ISAT (400.)


static Pid_t pidMotors[N_MOTORS];
//PidParams_t pidParamsMotors[N_MOTORS];
static uint8_t isInit = 0;
static float desiredWheelSpeed[N_MOTORS] = { 0, };
//static xQueueHandle motorControlQueue = 0;
static xQueueHandle motorDesiredVoltageQueue = 0;


uint8_t motor_control_init(void) {
    if (isInit)
        return 0;

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
    if (motorDesiredVoltageQueue == 0) {
        char str[] = "motors_control_init: motorDesiredVoltageQueue creation NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return -1;
    }
    else {
        char str[] = "motors_control_init: motorDesiredVoltageQueue creation OK\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }

    // Init. motor 1 PID
    float dt = (float)MOTOR_CONTROL_PERIOD_MS / 1000.f;
    pid_init(&pidMotors[MOTOR1], PID_MOTOR1_KP, PID_MOTOR1_KI, PID_MOTOR1_KD,
            PID_MOTOR1_SAT, PID_MOTOR1_ISAT, dt);
    // Init. motor 2 PID
    pid_init(&pidMotors[MOTOR2], PID_MOTOR2_KP, PID_MOTOR2_KI, PID_MOTOR2_KD,
            PID_MOTOR2_SAT, PID_MOTOR2_ISAT, dt);

    isInit = 1;
    return 1;
}

uint8_t motor_control_test(void) {
    return isInit;
}

void motor_control_reset(void) {
    pid_reset(&pidMotors[MOTOR1]);
    pid_reset(&pidMotors[MOTOR2]);
}

void motor_control_task(void* _params) {
    uint8_t ret = 0;
    MotorMeasuredSpeed_t measuredSpeeds;
    MotorDesiredVoltage_t desiredVoltages;
    MotorControl_t motorControls;
    float errors[N_MOTORS] = { 0, };
    float dt = 0, told = (float)get_us_time() * (float)1e-6;

    if (_params != 0) { }

    memset((void*)&measuredSpeeds, 0, sizeof(MotorMeasuredSpeed_t));
    memset((void*)&desiredVoltages, 0, sizeof(MotorDesiredVoltage_t));
    memset((void*)&motorControls, 0, sizeof(MotorControl_t));

    while (1) {
        // 1. Get encoder data
        ret = encoder_read_motor_measured_speed(&measuredSpeeds, 
                pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
        if (ret) {
            motorControls.controlState[MOTOR1].mes = measuredSpeeds.speed[MOTOR1];
            motorControls.controlState[MOTOR2].mes = measuredSpeeds.speed[MOTOR2];
        }
        else {

        }

        // 2. Get reference speeds
        // ... read from a queue without blocking
        //ret = read_desired_wheel_speed(&desiredWheelSpeeds, 0);
        motorControls.controlState[MOTOR1].des = desiredWheelSpeed[MOTOR1];
        motorControls.controlState[MOTOR2].des = desiredWheelSpeed[MOTOR2];

        // 3. Compute errors
        errors[MOTOR1] = motorControls.controlState[MOTOR1].des - 
                        motorControls.controlState[MOTOR1].mes;
        errors[MOTOR2] = motorControls.controlState[MOTOR2].des - 
                        motorControls.controlState[MOTOR2].mes;

        // 4. PID control
        motorControls.timestamp = (float)get_us_time() * (float)1e-6;
        dt = motorControls.timestamp - told;
        told = motorControls.timestamp;
        motorControls.controlState[MOTOR1].out = 
            pid_update(&pidMotors[MOTOR1], errors[MOTOR1], dt);
        motorControls.controlState[MOTOR2].out = 
            pid_update(&pidMotors[MOTOR2], errors[MOTOR2], dt);
        //xQueueOverwrite(motorControlQueue, &motorControls); // Send debug data

        // 5. Send voltage commands to motors
        desiredVoltages.timestamp = motorControls.timestamp;
        desiredVoltages.voltage[MOTOR1] = motorControls.controlState[MOTOR1].out;
        desiredVoltages.voltage[MOTOR2] = motorControls.controlState[MOTOR2].out;
        xQueueOverwrite(motorDesiredVoltageQueue, &desiredVoltages);
    }

    vTaskDelete(NULL);
}

void motor_control_set_desired_wheel_speeds(float _speed1, float _speed2) {
    desiredWheelSpeed[MOTOR1] = _speed1;
    desiredWheelSpeed[MOTOR2] = _speed2;
}

//uint8_t motor_control_read_data(MotorControl_t* data, TickType_t xTicksToWait) {
//    return (pdTRUE == xQueueReceive(motorControlQueue, data, xTicksToWait));
//}

uint8_t motor_control_read_desired_voltage(MotorDesiredVoltage_t* data, TickType_t xTicksToWait) {
    return (pdTRUE == xQueueReceive(motorDesiredVoltageQueue, data, xTicksToWait));
}
