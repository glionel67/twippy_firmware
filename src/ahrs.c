/**
 * \file ahrs.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief AHRS (Attitude Heading Reference System) functions
 */
 
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ahrs.h"
#include "imu.h"
#include "main.h"

#define DEBUG_MODULE "ahrs"

static xQueueHandle ahrsQueue = 0;
static xQueueHandle pitchAndRateQueue = 0;

static float beta = .1f;
static float q0 = 1.f, q1 = 0.f, q2 = 0.f, q3 = 0.f;
static Attitude_t attitude;

uint8_t ahrs_init(void)
{
    // Init. attitude
    q0 = 1.f, q1 = 0.f, q2 = 0.f, q3 = 0.f;

    attitude.timestamp = 0.;
    attitude.qw = q0;
    attitude.qx = q1;
    attitude.qy = q2;
    attitude.qz = q3;
    attitude.wx = attitude.wy = attitude.wz = 0.;

    return 1;
}

void ahrs_reset(void)
{
    q0 = 1.f, q1 = 0.f, q2 = 0.f, q3 = 0.f;
}

void ahrs_task(void* _params)
{
    uint8_t ret = 0;
    float prevTs = 0., dt = 0.;
    float pitch = 0.f;
    Imu6_t imu;
    Quaternion_t quaternion;
    PitchAndRate_t pitchAndRate;
    //char data[100] = { 0, };

    memset((void*)&imu, 0, sizeof(Imu6_t));
    memset((void*)&quaternion, 0, sizeof(Quaternion_t));
    memset((void*)&pitchAndRate, 0, sizeof(PitchAndRate_t));

    if (_params != 0) { }

    ret = ahrs_init();
    if (!ret) {
        printf("ahrs_init error\r\n");
        Error_Handler();
    }

    ahrsQueue = xQueueCreate(AHRS_QUEUE_SIZE, sizeof(Quaternion_t));
    if (ahrsQueue == 0) {
        printf("ahrsQueue creation error\r\n");
        goto byeBye;
    }

    pitchAndRateQueue = xQueueCreate(AHRS_QUEUE_SIZE, sizeof(PitchAndRate_t));
    if (pitchAndRateQueue == 0) {
        printf("pitchAndRateQueue creation error\r\n");
        goto byeBye;
    }

    // Wait for first IMU measurement
    while (!imu_read_imu6_data(&imu, pdMS_TO_TICKS(IMU_MEASUREMENT_PERIOD_MS)));
    prevTs = imu.timestamp;

    while (1) {
        ret = imu_read_imu6_data(&imu, pdMS_TO_TICKS(AHRS_PERIOD_MS));
        if (ret) {
            // Compute delta time
            dt = imu.timestamp - prevTs;

            //printf("%s: t=%3.3f,ax=%3.3f,gx=%3.3f\r\n", DEBUG_MODULE,
            //        (float)imu.timestamp, (float)imu.a[0], (float)imu.g[0]);
            
            // Compute attitude
            //MadgwickAHRSupdate(imu.g[0], imu.g[1], imu.g[2], 
            //                    imu.a[0], imu.a[1], imu.a[2], 
            //                    imu.m[0], imu.m[1], imu.m[2], dt);
            
            MadgwickAHRSupdateIMU(imu.g[0], imu.g[1], imu.g[2], 
                                imu.a[0], imu.a[1], imu.a[2], dt);

            // Copy result quaternion
            quaternion.timestamp = imu.timestamp; // Get timestamp
            quaternion.q[QW] = q0;
            quaternion.q[QX] = q1;
            quaternion.q[QY] = q2;
            quaternion.q[QZ] = q3;

            //printf("%s: t=%3.3f,qw=%3.3f,qx=%3.3f,qy=%3.3f,qz=%3.3f\r\n",
            //        DEBUG_MODULE, (float)quaternion.timestamp, 
            //        (float)quaternion.q[QW], (float)quaternion.q[QX],
            //        quaternion.q[QY], quaternion.q[QZ]);

            // Convert quaternion to pitch angle for balance control
            quaternion_to_pitch(q0, q1, q2, q3, &pitch);

            pitchAndRate.timestamp = imu.timestamp;
            pitchAndRate.pitch = pitch;
            pitchAndRate.pitchRate = imu.g[1];

            // Send it over the queue
            //vTaskSuspendAll();
            xQueueOverwrite(ahrsQueue, &quaternion);
            xQueueOverwrite(pitchAndRateQueue, &pitchAndRate);
            //xTaskResumeAll();

            attitude.timestamp = imu.timestamp;
            attitude.qw = q0;
            attitude.qx = q1;
            attitude.qy = q2;
            attitude.qz = q3;
            attitude.wx = imu.g[0];
            attitude.wy = imu.g[1];
            attitude.wz = imu.g[2];

            prevTs = quaternion.timestamp;
        }
        //vTaskDelay(200/portTICK_RATE_MS);
    }

    byeBye:
    vTaskDelete(NULL);
}

void ahrs_get_attitude(Attitude_t* _att)
{
    _att->timestamp = attitude.timestamp;
    _att->qw = attitude.qw;
    _att->qx = attitude.qx;
    _att->qy = attitude.qy;
    _att->qz = attitude.qz;
    _att->wx = attitude.wx;
    _att->wy = attitude.wy;
    _att->wz = attitude.wz;
}

uint8_t ahrs_get_quaternion(Quaternion_t* quat)
{
    return (pdTRUE == xQueueReceive(ahrsQueue, quat, 0));
}

uint8_t ahrs_read_quaternion(Quaternion_t* quat)
{
    return (pdTRUE == xQueueReceive(ahrsQueue, quat, 0));
}

uint8_t ahrs_get_pitchAndRate(PitchAndRate_t* _pitchAndRate)
{
    return (pdTRUE == xQueueReceive(pitchAndRateQueue, _pitchAndRate, 0));
}

uint8_t ahrs_read_pitchAndRate(PitchAndRate_t* _pitchAndRate)
{
    return (pdTRUE == xQueueReceive(pitchAndRateQueue, _pitchAndRate, 0));
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MadgwickAHRSupdate(float gx, float gy, float gz, 
    float ax, float ay, float az, 
    float mx, float my, float mz, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz; 
    float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2;
    float q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.f) && (my == 0.f) && (mz == 0.f)) {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    /*
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);
    */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, 
    float ax, float ay, float az, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    /*
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);
    */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void quaternion_to_rollPitchYaw(float _qw, float _qx, float _qy, float _qz,
    float* _roll, float* _pitch, float* _yaw)
{
    float num = 2.f * (_qy * _qz + _qx * _qw);
    float den = 1.f - 2.f * (_qx * _qx + _qy * _qy); //w*w + z*z - y*y - x*x;
    *_roll = atan2f(num, den);
    *_pitch = asinf(2.f * (_qw * _qy - _qx * _qz));
    num = 2.f * (_qx * _qy + _qw * _qz);
    den = 1.f - 2.f * (_qy * _qy + _qz * _qz); //w*w - z*z - y*y + x*x;
    *_yaw = atan2(num, den);
}


inline void quaternion_to_pitch(float _qw, float _qx, float _qy, float _qz, 
    float* _pitch)
{
    *_pitch = asinf(2.f * (_qw * _qy - _qx * _qz));
}
