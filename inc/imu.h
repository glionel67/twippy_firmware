/**
 * \file imu.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief IMU MPU9250 functions
 */

#pragma once

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
#include <stdint.h>

#include "FreeRTOS.h"

// -------------------------------------------------------------------------- //
// --- Defines
// -------------------------------------------------------------------------- //
#define IMU_MEASUREMENT_PERIOD_MS (2) // 2 ms <=> 500 Hz

#define IMU_QUEUE_SIZE 1

#ifndef N_AXES
#define N_AXES 3
#endif

// -------------------------------------------------------------------------- //
// --- Structs/enums
// -------------------------------------------------------------------------- //
typedef struct Imu6_s
{
    float timestamp; // Timestamp of the measurement [ms] or [us] ?
    float a[N_AXES]; // accelerometer [g] or [m/s2]
    float g[N_AXES]; // gyroscope [rad/s] or [deg/s]
} Imu6_t;

typedef struct Imu9_s
{
    float timestamp; // Timestamp of the measurement [ms] or [us] ?
    float a[N_AXES]; // accelerometer [g] or [m/s2]
    float g[N_AXES]; // gyroscope [rad/s] or [deg/s]
    float m[N_AXES]; // magnetometer [mT] ?
} Imu9_t;

// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //

/**
 * \fn init_imu
 * \brief initialize and configure the IMU
 * \return OK if success, NOK otherwise
 */
uint8_t init_imu(void);

void imu_test_task(void* _params);

void imu_task(void* _params);

uint8_t test_imu(void);

uint8_t is_imu_calibrated(void);

//uint8_t get_acc_data(float* ax, float* ay, float* az);

//uint8_t get_gyr_data(float* gx, float* gy, float* gz);

//uint8_t get_mag_data(float* mx, float* my, float* mz);

void get_imu6_data(Imu6_t* imu);

void get_imu9_data(Imu9_t* imu);

uint8_t imu_read_imu6_data(Imu6_t* imu, TickType_t xTicksToWait);

uint8_t imu_read_imu9_data(Imu9_t* imu, TickType_t xTicksToWait);

uint8_t imu_calibrate_gyro_bias(void);

void imu_calibrate_gyro_bias_task(void* _params);
