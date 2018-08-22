#include <string.h>
//#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "servo.h"
#include "main.h"

static TIM_HandleTypeDef TimHandleServo;
static TIM_OC_InitTypeDef ConfigServo;

//static xQueueHandle servoQueue = 0;

uint8_t servo_init(void) {
    TIM_PWM_SERVO_CLK_ENABLE();
    TimHandleServo.Instance = TIM_PWM_SERVO;
    TimHandleServo.Init.Prescaler = SERVO_PWM_PRESCALER;
    TimHandleServo.Init.Period = SERVO_PWM_PERIOD;
    TimHandleServo.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TimHandleServo.Init.CounterMode = TIM_COUNTERMODE_UP; // TIM_COUNTERMODE_CENTERALIGNED1,2,3
    TimHandleServo.Init.RepetitionCounter = 0;
    if (HAL_TIM_PWM_Init(&TimHandleServo) != HAL_OK) {
        char str[] = "servo_init: HAL_TIM_PWM_Init NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }

    ConfigServo.OCMode = TIM_OCMODE_PWM1;
    ConfigServo.OCPolarity = TIM_OCPOLARITY_HIGH;
    ConfigServo.OCIdleState = TIM_OCIDLESTATE_RESET;
    ConfigServo.OCFastMode = TIM_OCFAST_DISABLE;
    ConfigServo.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    ConfigServo.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    ConfigServo.Pulse = SERVO_MICROS_MID;
    if (HAL_TIM_PWM_ConfigChannel(&TimHandleServo, &ConfigServo,
            TIM_PWM_SERVO1_CHANNEL | TIM_PWM_SERVO2_CHANNEL |
            TIM_PWM_SERVO3_CHANNEL | TIM_PWM_SERVO4_CHANNEL) != HAL_OK) {
        char str[] = "servo_init: HAL_TIM_PWM_ConfigChannel NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }
/*
    if (HAL_TIM_PWM_Start(&TimHandleServo, TIM_PWM_SERVO1_CHANNEL | 
            TIM_PWM_SERVO2_CHANNEL | TIM_PWM_SERVO3_CHANNEL | 
            TIM_PWM_SERVO4_CHANNEL) != HAL_OK) {
        char str[] = "servo_init: HAL_TIM_PWM_Start NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }
*/
    return 1;
}

void servo_task(void* _params) {
    float angleInc = 1.f; // [degree]
    float angleMin = 0.f; // [degree]
    float angleMax = 180.f; // [degree]
    float angle = .5f * (angleMin + angleMax); // [degree]
    uint8_t dir = 1;
    uint8_t ret = 0;

    ret = servo_init();
    if (!ret) {
        char str[] = "servo_init NOK\n";
        print_msg((uint8_t*)str, strlen(str));
        vTaskDelete(NULL);
        return;
    }
    else {
        char str[] = "servo_init OK\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }

    if (_params != 0) { }

    while (1) {
        // if (new servo angle) { servo_set_angle(angle); }
        // else wait...

        if (dir == 1)
            angle += angleInc;
        else
            angle -= angleInc;

        if (angle >= angleMax) 
            dir = 0;
        else if (angle <= angleMin)
            dir = 1;
        
        ret = servo_set_angle(angle, TIM_PWM_SERVO1_CHANNEL);
        ret = servo_set_angle(angle, TIM_PWM_SERVO2_CHANNEL);
        ret = servo_set_angle(angle, TIM_PWM_SERVO3_CHANNEL);
        ret = servo_set_angle(angle, TIM_PWM_SERVO4_CHANNEL);
        if (!ret) {
            char str[] = "servo_task: servo_set_angle NOK\r\n";
            print_msg((uint8_t*)str, strlen(str));
        }
        else {
            char str[] = "servo_task: servo_set_angle OK\r\n";
            print_msg((uint8_t*)str, strlen(str));
        }

        vTaskDelay(100/portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

float degreesToPulseLength(float degrees) {
    float pulseLength  = (SERVO_MICROS_MAX - SERVO_MICROS_MIN) * degrees / 180.f + SERVO_MICROS_MIN;
    return pulseLength;
}


float pulseLengthToDegrees(float pulseLendth) {
    float degrees = (pulseLendth - SERVO_MICROS_MIN) * 180.f / (SERVO_MICROS_MAX - SERVO_MICROS_MIN);
    return degrees;
}

uint8_t servo_set_us(uint16_t us, uint32_t channel) {
    ConfigServo.Pulse = us;

    if (HAL_TIM_PWM_ConfigChannel(&TimHandleServo, &ConfigServo, channel) != HAL_OK) {
        char str[] = "servo_set_angle: HAL_TIM_PWM_ConfigChannel NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }

    if (HAL_TIM_PWM_Start(&TimHandleServo, channel) != HAL_OK) {
        char str[] = "servo_set_angle: HAL_TIM_PWM_Start NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }

    return 1;
}

uint8_t servo_set_angle(float angle, uint32_t channel) {
    ConfigServo.Pulse = (uint16_t)degreesToPulseLength(angle);

    if (HAL_TIM_PWM_ConfigChannel(&TimHandleServo, &ConfigServo, channel) != HAL_OK) {
        char str[] = "servo_set_angle: HAL_TIM_PWM_ConfigChannel NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }

    if (HAL_TIM_PWM_Start(&TimHandleServo, channel) != HAL_OK) {
        char str[] = "servo_set_angle: HAL_TIM_PWM_Start NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }

    return 1;
}