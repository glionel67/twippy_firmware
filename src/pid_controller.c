#include "pid_controller.h"

#include <math.h>

/**
 * \fn pid_init
 * \brief Initialize the pid controller structure
 */
void pid_init(Pid_t* _pid, float _kp, float _ki, float _kd, float _kffwd,
		float _sat, float _isat, float _dt)
{
	_pid->kp = _kp;
	_pid->ki = _ki;
	_pid->kd = _kd;
	_pid->kffwd = _kffwd;
	_pid->iSat = _isat;
	_pid->sat = _sat;
	_pid->dt = _dt;
    _pid->freezeIntegralThresh = 0.1;
	_pid->integ = _pid->deriv = 0;
	_pid->error = _pid->prevError = 0;
	_pid->outP = 0.;
	_pid->outI = 0.;
	_pid->outD = 0.;
	_pid->outF = 0.;
}

/**
 * \fn pid_update
 * \brief PID command computation
 * \param _pid PID structure
 * \param _ref reference/setpoint to reach
 * \param _fdbk feedback/measurement
 * \param _dt control sampling interval in seconds [s]
 * */
float pid_update(Pid_t* _pid, float _ref, float _fdbk, float _dt)
{
	float cmd = 0;
	float alpha = 0, deriv = 0;

	// Check validity of dt
	if (_dt > 2.f * _pid->dt) {
		_dt = _pid->dt;
	}
    
    // Compute PID error
    float error = _ref - _fdbk;

	// Check validity of error
    if (isnan(error))
    {
		return cmd;
	}
    
	// Save error
    _pid->error = error;

    // Feedforward term
    _pid->outF = _pid->kffwd * _ref;

	// Proportional term
	_pid->outP = _pid->kp * _pid->error;

	// Integral term (sum of errors over time)
	// Integral freeze control
    if (fabsf(_pid->error) > _pid->freezeIntegralThresh)
    {
        _pid->integ += _pid->error * _pid->dt;
        
        // Integral saturation
        if (_pid->integ > _pid->iSat)
            _pid->integ = _pid->iSat;
        else if (_pid->integ < -_pid->iSat)
            _pid->integ = -_pid->iSat;
        
        _pid->outI = _pid->ki * _pid->integ;
    }
    else
    {
        _pid->outI = 0.;
    }

	// Derivative term + low pass filter (LPF)
	deriv = (_pid->error - _pid->prevError) / _pid->dt;
	if (_pid->enableFilter) {
		alpha = _pid->dt / (_pid->dt + _pid->rc);
		_pid->deriv += alpha * (deriv - _pid->deriv);
	}
	else {
		_pid->deriv = deriv;
	}
	_pid->outD = _pid->kd * _pid->deriv;

	// Final command output
	cmd = _pid->outF +_pid->outP + _pid->outI + _pid->outD;

	// Command saturation with anti wind-up
	if (cmd > _pid->sat) {
		//_pid->integ += _pid->sat - cmd;
        _pid->integ -= _pid->error * _pid->dt;
		cmd = _pid->sat;
	}
	else if (cmd < -_pid->sat) {
		//_pid->integ += -_pid->sat - cmd;
        _pid->integ -= _pid->error * _pid->dt;
		cmd = -_pid->sat;
	}

	// Save previous error
	_pid->prevError = _pid->error;

	// Check output command validity
	if (isnan(cmd))
		return 0;

	return cmd;
}

void pid_reset(Pid_t* _pid) {
	_pid->integ = _pid->deriv = 0.;
	_pid->error = _pid->prevError = 0.;
    _pid->outF = 0.;
    _pid->outP = 0.;
    _pid->outI = 0.;
    _pid->outD = 0.;
}

void pid_set_filter(Pid_t* _pid, float _cutoffFreq) {
	_pid->enableFilter = 1;
	_pid->cutoffFreq = _cutoffFreq;
	_pid->rc = 1.f / (2.f * M_PI * _cutoffFreq);
}

void pid_set_dt(Pid_t* _pid, float _dt) {
	_pid->dt = _dt;
}

void pid_set_kp(Pid_t* _pid, float _kp) {
	_pid->kp = _kp;
}

void pid_set_ki(Pid_t* _pid, float _ki) {
	_pid->ki = _ki;
}

void pid_set_kd(Pid_t* _pid, float _kd) {
	_pid->kd = _kd;
}

void pid_set_kffwd(Pid_t* _pid, float _kffwd) {
    _pid->kffwd = _kffwd;
}

void pid_set_kpid(Pid_t* _pid, float _kp, float _ki, float _kd) {
	_pid->kp = _kp;
	_pid->ki = _ki;
	_pid->kd = _kd;
}

void pid_set_sat(Pid_t* _pid, float _sat) {
	_pid->sat = _sat;
}

void pid_set_isat(Pid_t* _pid, float _isat) {
	_pid->iSat = _isat;
}
