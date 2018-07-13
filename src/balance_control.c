#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "balance_control.h"
#include "pid_controller.h"
#include "ahrs.h"
#include "imu.h"

#define DEBUG_MODULE "balance_ctrl"

#define PID_PITCH_RATE_KP 1.
#define PID_PITCH_RATE_KI .1
#define PID_PITCH_RATE_KD .1

#define PID_PITCH_RATE_ISAT 1000
#define PID_PITCH_RATE_SAT 1000

#define PID_PITCH_KP 1.
#define PID_PITCH_KI .1
#define PID_PITCH_KD .1

#define PID_PITCH_ISAT 1000
#define PID_PITCH_SAT 1000

#define ATTITUDE_LPF_CUTOFF_FREQ      15.0f
#define ATTITUDE_LPF_ENABLE false
#define ATTITUDE_RATE_LPF_CUTOFF_FREQ 30.0f
#define ATTITUDE_RATE_LPF_ENABLE false


Pid_t pidPitchRate;
Pid_t pidPitch;

static float pitchOutput;

static uint8_t isInit;

uint8_t balance_control_init(const float dt) {
  if (isInit)
    return 0;

  // Pitch rate controller
  pid_init(&pidPitchRate, PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
    PID_PITCH_RATE_SAT, PID_PITCH_RATE_ISAT, dt);

  // Pitch controller
  pid_init(&pidPitch, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_PITCH_SAT, 
    PID_PITCH_ISAT, dt);

  isInit = 1;

  return 1;
}

uint8_t balance_control_test(void) {
  return isInit;
}

float balance_control_correct_pitch_rate(float pitchRateActual,
        float pitchRateDesired, float _dt) {
  float error = pitchRateDesired - pitchRateActual;

  pitchOutput = pid_update(&pidPitchRate, error, _dt);

  return pitchOutput;
}

void balance_control_correct_pitch(float pitchActual, 
        float pitchDesired, float* pitchRateDesired, float _dt) {
  float error = pitchDesired - pitchActual;

  *pitchRateDesired = pid_update(&pidPitch, error, _dt);
}

void balance_control_reset(void) {
  pid_reset(&pidPitch);
  pid_reset(&pidPitchRate);
  isInit = 0;
}

void balance_control_task(void* _params) {
  uint8_t ret = 0;
  PitchAndRate_t pitchAndRate;
  float pitchRateDesired = 0.f;
  float prevTs = 0., dt = 0.;
  char data[100] = { 0, };

  memset((void*)&pitchAndRate, 0, sizeof(PitchAndRate_t));

  if (_params != 0) { }

  ret = balance_control_init(BALANCE_CONTROL_DT);
  if (!ret) {
    char str[] = "balance_control_init error\r\n";
    print_msg((uint8_t*)str, strlen(str));
    goto byeBye;
  }

  // Wait for first AHRS measurement
  while (!ahrs_get_pitchAndRate(&pitchAndRate)) {
    vTaskDelay(2/portTICK_RATE_MS);
  }
  prevTs = pitchAndRate.timestamp;

  while (1) {
    ret = ahrs_get_pitchAndRate(&pitchAndRate);
    if (ret) {
      dt = pitchAndRate.timestamp - prevTs;

      balance_control_correct_pitch(pitchAndRate.pitch, 0., 
        &pitchRateDesired, dt);

      pitchOutput = balance_control_correct_pitch_rate(pitchAndRate.pitchRate,
        pitchRateDesired, dt);

      sprintf(data, "%s: t=%3.3f,out=%3.3f\r\n", DEBUG_MODULE,
        (float)pitchAndRate.timestamp, pitchOutput);
      print_msg((uint8_t*)data, strlen(data));

      prevTs = pitchAndRate.timestamp;
    }
  }

  byeBye:
  vTaskDelete(NULL);
}

/*
LOG_GROUP_START(pid_attitude)
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitch.outP)
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitch.outI)
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitch.outD)
LOG_GROUP_STOP(pid_attitude)

LOG_GROUP_START(pid_rate)
LOG_ADD(LOG_FLOAT, pitch_outP, &pidPitchRate.outP)
LOG_ADD(LOG_FLOAT, pitch_outI, &pidPitchRate.outI)
LOG_ADD(LOG_FLOAT, pitch_outD, &pidPitchRate.outD)
LOG_GROUP_STOP(pid_rate)

PARAM_GROUP_START(pid_attitude)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitch.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitch.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitch.kd)
PARAM_GROUP_STOP(pid_attitude)

PARAM_GROUP_START(pid_rate)
PARAM_ADD(PARAM_FLOAT, pitch_kp, &pidPitchRate.kp)
PARAM_ADD(PARAM_FLOAT, pitch_ki, &pidPitchRate.ki)
PARAM_ADD(PARAM_FLOAT, pitch_kd, &pidPitchRate.kd)
PARAM_GROUP_STOP(pid_rate)
*/