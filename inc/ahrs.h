#pragma once

//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimised for reduced CPU load
//

#include <stdint.h>

#include "imu.h"

#define QUATERNION_SIZE 4
#define AHRS_QUEUE_SIZE 1

enum QuaternionIndices {
    QW=0, QX, QY, QZ
};

typedef struct Quaternion_s {
    float timestamp;
    float q[QUATERNION_SIZE]; // Quaternion attitude
} Quaternion_t;


uint8_t ahrs_init(void);

//void ahrs_test_task(void* _params);

void ahrs_task(void* _params);

//uint8_t ahrs_test(void);

uint8_t ahrs_get_quaternion(Quaternion_t* quat);
uint8_t ahrs_read_quaternion(Quaternion_t* quat);

void MadgwickAHRSupdate(float gx, float gy, float gz, 
    float ax, float ay, float az, 
    float mx, float my, float mz, float dt);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, 
    float ax, float ay, float az, float dt);
