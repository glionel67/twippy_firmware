/**
 * \file com.h
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief Communication functions to exchange data with other device
 */

#pragma once

#include <stdint.h>

#include "config.h"

#define DEBUGSIZE           63

#define PACKET_SIZE         85

#define N_HEADERS           2
#define MAX_DATA            255

#define MSG_HEADER1         (0xAC)
#define MSG_HEADER2         (0xDC)

#define RX_MSG_HEADER1      (0xAC)
#define RX_MSG_HEADER2      (0xDC)

#define TX_MSG_HEADER1      (0xAB)
#define TX_MSG_HEADER2      (0xBA)

/*
typedef enum {
    EMERGENCY_STOP=0,
} Messages_t;
*/

#define PWM_MODE            (0)
#define RPM_MODE            (1)

typedef enum
{
    EMERGENCY_STOP=1,

    SET_MOTORS_REF_RPM, // RPM
    SET_MOTOR_REF_RPM, // RPM
    SET_MOTORS_PWM,
    SET_MOTOR_PWM,
    SET_V_ROBOT, // mm/s
    SET_W_ROBOT, // mrad/s
    SET_REF_V_ROBOT, // mm/s
    SET_REF_W_ROBOT, // mrad/s
    SET_ROBOT_SPEEDS, // mm/s, mrad/s
    SET_REF_ROBOT_SPEEDS, // mm/s, mrad/s
    SET_ROBOT_POSE, // cm, cm, mrad
    SET_REF_ROBOT_POSE, // cm, cm, mrad
    SET_SERVOS_ANGLE, // degree
    SET_SERVO_ANGLE, // degree
    SET_MOTORS_PID_P,
    SET_MOTORS_PID_I,
    SET_MOTORS_PID_D,
    SET_WHEEL_RADIUS, // mm
    SET_WHEEL_BASE, // mm
    SET_PWM_RPM_MODE,

    GET_MOTORS_REF_RPM,
    GET_MOTOR_REF_RPM,
    GET_MOTORS_PWM,
    GET_MOTOR_PWM,
    GET_V_ROBOT,
    GET_W_ROBOT,
    GET_REF_V_ROBOT,
    GET_REF_W_ROBOT,
    GET_ROBOT_SPEEDS,
    GET_REF_ROBOT_SPEEDS,
    GET_ROBOT_POSE,
    GET_REF_ROBOT_POSE,
    GET_SERVO_ANGLES,
    GET_SERVO_ANGLE,
    GET_MOTORS_PID_P,
    GET_MOTORS_PID_I,
    GET_MOTORS_PID_D,
    GET_WHEEL_RADIUS,
    GET_WHEEL_BASE,

    GET_IMOTORS, // mA
    GET_IMOTOR, // mA
    GET_VBAT, // mV
    GET_IBAT, // mA
    GET_BAT_STATUS,
    GET_IMU_ACC,
    GET_IMU_GYRO,
    GET_IMU_MAG,
    GET_IMU_QUAT,
    GET_ENCS_TICKS,
    GET_ENC_TICKS,
    GET_IR_RANGES, // cm
    GET_IR_RANGE, // cm
    GET_US_RANGES, // cm
    GET_US_RANGE // cm
} Commands_t;


typedef struct
{
    uint8_t headers[N_HEADERS];
    uint8_t cmd;
    uint8_t size;
    uint8_t data[MAX_DATA];
    uint8_t cheksum;
} Packet_t;

typedef struct
{
    uint8_t startChar;
    uint16_t imot[N_MOTORS]; // in [mA]
    int16_t ticks[N_MOTORS]; // Encoder counts
    int16_t wref[N_MOTORS]; // in [RPM]
    int16_t wmes[N_MOTORS]; // in [RPM]
    uint16_t pwm[N_MOTORS];
    uint16_t vbat; // in [mV]
    uint16_t ibat; // in [mA]
    uint16_t ir[N_IR]; // in [cm]
    uint16_t us[N_US]; // in [cm]
    int16_t servo[N_SERVOS]; // Angular position in [degree]
    int16_t acc[N_AXIS]; // mg
    int16_t gyro[N_AXIS]; // mrad/s
    int16_t mag[N_AXIS]; // muT ?
    int16_t attitude[4]; // 1/1000
    int16_t pose[3]; // cm,cm,mrad
    int16_t v; // mm/s
    int16_t w; // mrad/s
    uint8_t endChar[2];
} DataPacket_t;

typedef union
{
    DataPacket_t packet;
    uint8_t bytes[PACKET_SIZE];
} DataPacket_u;

typedef struct
{
    int16_t rpm[N_MOTORS]; // [RPM]
    uint8_t newRpm[N_MOTORS];
    int16_t pwm[N_MOTORS];
    uint8_t newPwm[N_MOTORS];
    uint8_t pwmRpmMode; // 0 = PWM mode, 1 = RPM mode
    float pose[3];
    uint8_t newPose;
    int16_t servo[N_SERVOS]; // [degree]
    uint8_t newServo[N_SERVOS];
    uint8_t emergencyStop;
} Inputs_t;

int decode_msg(uint8_t* buf, uint8_t len);
int send_info(DataPacket_t* dataPacket, uint8_t output);
int send_packet(DataPacket_u* packet, uint8_t output);