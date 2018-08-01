#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "main.h"
#include "pid_controller.h"
#include "motor.h"
#include "encoders.h"

#define DEBUG_MODULE "motors_ctrl"

#define PID_MOTOR1_KP (1.)
#define PID_MOTOR1_KI (0.)
#define PID_MOTOR1_KD (0.)
#define PID_MOTOR1_SAT (220.) // in [rpm]
#define PID_MOTOR1_ISAT (300.)

#define PID_MOTOR2_KP (1.)
#define PID_MOTOR2_KI (0.)
#define PID_MOTOR2_KD (0.)
#define PID_MOTOR2_SAT (220.) // in [rpm]
#define PID_MOTOR2_ISAT (300.)


static MotorControls_t motorControls;
static Pid_t pidMotors[N_MOTORS];
//PidParams_t pidParamsMotors[N_MOTORS];
static uint8_t isInit = 0;

static xQueueHandle motorControlQueue = 0;


uint8_t motors_control_init(const float dt) {
    if (isInit)
        return 0;

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

    // Init. motor 1 PID
    pid_init(&pidMotors[MOTOR1], PID_MOTOR1_KP, PID_MOTOR1_KI, PID_MOTOR1_KD,
            PID_MOTOR1_SAT, PID_MOTOR1_ISAT, dt);
    // Init. motor 2 PID
    pid_init(&pidMotors[MOTOR2], PID_MOTOR2_KP, PID_MOTOR2_KI, PID_MOTOR2_KD,
            PID_MOTOR2_SAT, PID_MOTOR2_ISAT, dt);

    memset((void*)&motorControls, 0, sizeof(MotorControls_t));

    isInit = 1;
    return 1;
}

uint8_t motors_control_test(void) {
    return isInit;
}

void motors_control_reset(void) {
    pid_reset(&pidMotors[MOTOR1]);
    pid_reset(&pidMotors[MOTOR2]);
}

void motors_control_task(void* _params) {
    uint8_t ret = 0;
    Encoders_t encoders;
    float errors[N_MOTORS] = { 0, };
    float dt = 0, told = (float)get_us_time() * (float)1e-6;

    if (_params != 0) { }

    memset((void*)&encoders, 0, sizeof(Encoders_t));

    while (1) {
        // 1. Get encoder data
        ret = encoder_read_data(&encoders, pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
        if (ret) {
            motorControls.motorControls[MOTOR1].mes = encoders.encoders[MOTOR1].rpm;
            motorControls.motorControls[MOTOR2].mes = encoders.encoders[MOTOR2].rpm;
        }
        else {

        }

        // 2. Get reference speeds
        // ...

        // 3. Compute errors
        errors[MOTOR1] = motorControls.motorControls[MOTOR1].ref - 
                        motorControls.motorControls[MOTOR1].mes;
        errors[MOTOR2] = motorControls.motorControls[MOTOR2].ref - 
                        motorControls.motorControls[MOTOR2].mes;

        // 4. PID control
        motorControls.timestamp = (float)get_us_time() * (float)1e-6;
        dt = motorControls.timestamp - told;
        told = motorControls.timestamp;
        motorControls.motorControls[MOTOR1].out = 
            pid_update(&pidMotors[MOTOR1], errors[MOTOR1], dt);
        motorControls.motorControls[MOTOR2].out = 
            pid_update(&pidMotors[MOTOR2], errors[MOTOR2], dt);

        // 5. Send commands to motors
        xQueueOverwrite(motorControlQueue, &motorControls);
    }

    vTaskDelete(NULL);
}

void motors_control_set_ref(float _ref1, float _ref2) {
    motorControls.motorControls[MOTOR1].ref = _ref1;
    motorControls.motorControls[MOTOR2].ref = _ref2;
}

uint8_t motors_control_read_data(MotorControls_t* data, TickType_t xTicksToWait) {
  return (pdTRUE == xQueueReceive(motorControlQueue, data, xTicksToWait));
}
