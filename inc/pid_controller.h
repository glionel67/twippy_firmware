#pragma once

#include <stdint.h>

typedef struct Pid_s {
	float kp, ki, kd; // Proportional, integral and derivative gain
	float kffwd; // Feedforward gain
	float error, prevError; // Current and previous error
	float integ; // Integral term
	float deriv; // Derivative tern
	float dt; // Sampling time [s]
	float outP, outI, outD; // Output from the proportional, integral and derivative terms
	float outF; // Output from the feedforward term
	float iSat; // Integral saturation term
	float sat; // Output saturation term
	uint8_t enableFilter; // Enable/disable Low-pass filter
	float cutoffFreq; // Low-pass filter cut-off frequency
	float rc; // Low-pass filter rc coefficient
} Pid_t;

typedef struct PidParams_s {
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float kffwd; // Feedforward gain
    float sat; // Command saturation
    float isat; // Integral saturation
    float dt; // Sampling period [s]
} PidParams_t;

void pid_init(Pid_t* _pid, float _kp, float _ki, float _kd, float _kffwd,
		float _sat, float _isat, float _dt);
float pid_update(Pid_t* _pid, float _e, float _dt);
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