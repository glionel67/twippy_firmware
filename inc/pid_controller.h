/**
 * \file pid_controller.h
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief PID controller functions
 */

#pragma once

#include <stdint.h>

/**
 * \struct Pid_t
 * \brief PID structure
 */
typedef struct Pid_s
{
	float kp, ki, kd; // Proportional, integral and derivative gains
	float kffwd; // Feedforward gain
	float error, prevError; // Current and previous errors
	float integ; // Integral term
	float deriv; // Derivative tern
    float dt; // Controller sampling period [s]
	float outP, outI, outD; // Output from the proportional, integral and derivative terms
	float outF; // Output from the feedforward term
    float freezeIntegralThresh; // Freeze integration when the error is below this threshold
	float iSat; // Integral saturation term
	float sat; // Output saturation term
	uint8_t enableFilter; // Enable/disable Low-pass filter
	float cutoffFreq; // Low-pass filter cut-off frequency
	float rc; // Low-pass filter rc coefficient
    //float lastCall; // Last time the PID controller was called
} Pid_t;

/**
 * \struct PidParams_t
 * \brief PID parameters structure
 */
typedef struct PidParams_s
{
	float kffwd; // Feedforward gain
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float sat; // Command saturation
    float isat; // Integral saturation
    float dt; // Sampling period [s]
} PidParams_t;

/**
 * \fn pid_init
 * \brief Initialize the pid controller structure
 * \param _pid PID structure
 * \param _pidParams PID parameters
 */
void pid_init(Pid_t* _pid, PidParams_t* _pidParams);

/**
 * \fn pid_update
 * \brief PID command computation
 * \param _pid PID structure
 * \param _ref reference/setpoint to reach
 * \param _fdbk feedback/measurement
 * \param _dt control sampling interval in seconds [s]
 * */
float pid_update(Pid_t* _pid, float _ref, float _fdbk, float _dt);
void pid_reset(Pid_t* _pid);
void pid_set_filter(Pid_t* _pid, float _cutoffFreq);
void pid_set_dt(Pid_t* _pid, float _dt);
void pid_set_kp(Pid_t* _pid, float _kp);
void pid_set_ki(Pid_t* _pid, float _ki);
void pid_set_kd(Pid_t* _pid, float _kd);
void pid_set_kffwd(Pid_t* _pid, float _kffwd);
void pid_set_kpid(Pid_t* _pid, float _kp, float _ki, float _kd);
void pid_set_sat(Pid_t* _pid, float _sat);
void pid_set_isat(Pid_t* _pid, float _isat);
void pid_set_params(Pid_t* _pid, PidParams_t* _pidParams);
void pid_get_params(Pid_t* _pid, PidParams_t* _pidParams);