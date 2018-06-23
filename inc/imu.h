#pragma once

#include <stdint.h>

#include "mpu9250.h"

#define IMU_QUEUE_SIZE 1

typedef struct Imu6_s {
    float timestamp;
    float a[N_AXES]; // accelerometer
    float g[N_AXES]; // gyroscope
} Imu6_t;

typedef struct Imu9_s {
    float timestamp;
    float a[N_AXES]; // accelerometer
    float g[N_AXES]; // gyroscope
    float m[N_AXES]; // magnetometer
} Imu9_t;

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

uint8_t imu_read_data(Imu9_t* imu);