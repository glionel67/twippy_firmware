#include <stdio.h>
#include <string.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "motor.h"
#include "motor_control.h"
#include "main.h"
#include "usTimer.h"
#include "encoder.h"


// Timer handler declaration
TIM_HandleTypeDef TimHandleMotors;

// Timer Output Compare Configuration Structure declaration
static TIM_OC_InitTypeDef sConfigMotors;
static Motor_t motors;
static xQueueHandle motorQueue = 0;
static float inputVoltage = 0.f;

int init_motors(void)
{
	int ret = 0;

	memset((void*)&motors, 0, sizeof(Motor_t));

	motorQueue = xQueueCreate(MOTOR_QUEUE_SIZE, sizeof(Motor_t));
	if (motorQueue == 0) {
		printf("init_motors: motorQueue creation NOK\r\n");
		return -1;
	}
    else {
        printf("init_motors: motorQueue creation OK\r\n");
    }

	// PWM motors
	TIM_PWM_MOTORS_CLK_ENABLE();
	TimHandleMotors.Instance = TIM_PWM_MOTORS;
	TimHandleMotors.Init.Prescaler = MOTORS_PWM_PRESCALER;
	TimHandleMotors.Init.Period = MOTORS_PWM_PERIOD;
	TimHandleMotors.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TimHandleMotors.Init.CounterMode = TIM_COUNTERMODE_UP; // TIM_COUNTERMODE_CENTERALIGNED1,2,3
	TimHandleMotors.Init.RepetitionCounter = 0;
	ret = HAL_TIM_PWM_Init(&TimHandleMotors);
	if (ret != HAL_OK) {
		printf("init_motors: HAL_TIM_PWM_Init NOK\r\n");
		return -1;
	}

	sConfigMotors.OCMode = TIM_OCMODE_PWM1;
	sConfigMotors.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigMotors.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigMotors.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigMotors.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigMotors.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfigMotors.Pulse = 0;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR1_CHANNEL | TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		printf("init_motors: HAL_TIM_PWM_ConfigChannel NOK\r\n");
		return -1;
	}

	ret = HAL_TIM_PWM_Start(&TimHandleMotors,
			TIM_PWM_MOTOR1_CHANNEL | TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		printf("init_motors: HAL_TIM_PWM_Start NOK\r\n");
		return -1;
	}

	return 0;
}

int set_pwm1(uint16_t _pwm)
{
	int ret = 0;
	motors.motors[MOTOR1].pwm = _pwm;
	sConfigMotors.Pulse = _pwm;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR1_CHANNEL);
	if (ret != HAL_OK) {
		printf("set_pwm1: HAL_TIM_PWM_ConfigChannel NOK\r\n");
		return -1;
	}
	ret = HAL_TIM_PWM_Start(&TimHandleMotors, TIM_PWM_MOTOR1_CHANNEL);
	if (ret != HAL_OK) {
		printf("set_pwm1: HAL_TIM_PWM_Start NOK\r\n");
		return -1;
	}
	return 0;
}

int set_pwm2(uint16_t _pwm)
{
	int ret = 0;
	motors.motors[MOTOR2].pwm = _pwm;
	sConfigMotors.Pulse = _pwm;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		printf("set_pwm2: HAL_TIM_PWM_ConfigChannel NOK\r\n");
		return -1;
	}
	ret = HAL_TIM_PWM_Start(&TimHandleMotors, TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		printf("set_pwm2: HAL_TIM_PWM_Start NOK\r\n");
		return -1;
	}
	return 0;
}

int set_pwm12(uint16_t _pwm1, uint16_t _pwm2)
{
	int ret = 0;
	motors.motors[MOTOR1].pwm = _pwm1;
	sConfigMotors.Pulse = _pwm1;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR1_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	ret = HAL_TIM_PWM_Start(&TimHandleMotors, TIM_PWM_MOTOR1_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	motors.motors[MOTOR2].pwm = _pwm2;
	sConfigMotors.Pulse = _pwm2;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	ret = HAL_TIM_PWM_Start(&TimHandleMotors, TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	return 0;
}

// pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
// where DutyCycle is in percent, between 0 and 100% 
// 25% duty cycle:     pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
int set_dc_pwm1(float _dc)
{
	motors.motors[MOTOR1].dutyCycle = _dc;
	motors.motors[MOTOR1].pwm = (uint16_t)(_dc * (float) MOTORS_PWM_PERIOD);
	return set_pwm1(motors.motors[MOTOR1].pwm);
}

int set_dc_pwm2(float _dc)
{
	motors.motors[MOTOR2].dutyCycle = _dc;
	motors.motors[MOTOR2].pwm = (uint16_t)(_dc * (float) MOTORS_PWM_PERIOD);
	return set_pwm2(motors.motors[MOTOR2].pwm);
}

int set_dc_pwm12(float _dc1, float _dc2)
{
	if (set_dc_pwm1(_dc1) != 0)
		return -1;
	if (set_dc_pwm2(_dc2) != 0)
		return -1;
	return 0;
}

int set_m1_speed(int32_t _speed)
{
	int ret = 0;

	// 1. Set motor direction
	if (_speed == 0) {
		motors.motors[MOTOR1].direction = FORWARD_DIR; // Assume forward
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0); // Make the motor coast no
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0); // matter which direction it is spinning.
	}
	else if (_speed < 0) {
		_speed = -_speed; // Make speed a positive quantity
		motors.motors[MOTOR1].direction = REVERSE_DIR; // reverse
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 1);
	}
	else {
		motors.motors[MOTOR1].direction = FORWARD_DIR; // forward
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0);
	}

	// 2. Set motor PWM
	if (_speed > MOTORS_PWM_PERIOD) // Max PWM period
		_speed = MOTORS_PWM_PERIOD;

	motors.motors[MOTOR1].pwm = (uint16_t)_speed;
	sConfigMotors.Pulse = (uint16_t)_speed;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR1_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	ret = HAL_TIM_PWM_Start(&TimHandleMotors, TIM_PWM_MOTOR1_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	return 0;
}

int set_m2_speed(int32_t _speed)
{
	int ret = 0;

	// 1. Set motor direction
	if (_speed == 0) {
		motors.motors[MOTOR2].direction = FORWARD_DIR; // Assume forward
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0); // Make the motor coast no
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0); // matter which direction it is spinning.
	}
	else if (_speed < 0) {
		_speed = -_speed; // Make speed a positive quantity
		motors.motors[MOTOR2].direction = REVERSE_DIR; // reverse
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 1);
	}
	else {
		motors.motors[MOTOR2].direction = FORWARD_DIR; // forward
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0);
	}

	// 2. Set motor PWM
	if (_speed > MOTORS_PWM_PERIOD) // Max PWM dutycycle
		_speed = MOTORS_PWM_PERIOD;

	motors.motors[MOTOR2].pwm = (uint16_t)_speed;
	sConfigMotors.Pulse = (uint16_t)_speed;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	ret = HAL_TIM_PWM_Start(&TimHandleMotors, TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	return 0;
}

int set_m1_brake(int32_t _brake)
{
	int ret = 0;

	// Normalize brake
	if (_brake) {
		_brake = -_brake;
	}
	if (_brake > MOTORS_PWM_PERIOD) // Max brake
		_brake = MOTORS_PWM_PERIOD;

	HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0); // Make the motor coast no
	HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0); // matter which direction it is spinning.

	sConfigMotors.Pulse = (uint16_t) _brake;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR1_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	ret = HAL_TIM_PWM_Start(&TimHandleMotors, TIM_PWM_MOTOR1_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	return 0;
}

int set_m2_brake(int32_t _brake)
{
	int ret = 0;

	// Normalize brake
	if (_brake) {
		_brake = -_brake;
	}
	if (_brake > MOTORS_PWM_PERIOD) // Max brake
		_brake = MOTORS_PWM_PERIOD;

	HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0); // Make the motor coast no
	HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0); // matter which direction it is spinning.

	sConfigMotors.Pulse = (uint16_t) _brake;
	ret = HAL_TIM_PWM_ConfigChannel(&TimHandleMotors, &sConfigMotors,
			TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	ret = HAL_TIM_PWM_Start(&TimHandleMotors, TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}
	return 0;
}

uint8_t get_m1_fault(void)
{
	motors.motors[MOTOR1].fault = !HAL_GPIO_ReadPin(MOTORS_GPIO, M1_EN_PIN); // 1 = fault
	return motors.motors[MOTOR1].fault;
}

uint8_t get_m2_fault(void)
{
	motors.motors[MOTOR2].fault = !HAL_GPIO_ReadPin(MOTORS_GPIO, M2_EN_PIN); // 1 = fault
	return motors.motors[MOTOR2].fault;
}

void brake_motor1(void)
{
	HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0); // Make the motor coast no
	HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0); // matter which direction it is spinning.
}

void brake_motor2(void)
{
	HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0); // Make the motor coast no
	HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0); // matter which direction it is spinning.
}

void brake_motor12(void) {
	HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0); // Make the motor coast no
	HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0); // matter which direction it is spinning.
	HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0); // Make the motor coast no
	HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0); // matter which direction it is spinning.
}

void set_dir_motor1(uint8_t _sens) {
	if (_sens == FORWARD_DIR) {
		motors.motors[MOTOR1].direction = FORWARD_DIR;
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0);
	} else {
		motors.motors[MOTOR1].direction = REVERSE_DIR;
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 1);
	}
}

void set_dir_motor2(uint8_t _sens) {
	if (_sens == FORWARD_DIR) {
		motors.motors[MOTOR2].direction = FORWARD_DIR;
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0);
	} else {
		motors.motors[MOTOR2].direction = REVERSE_DIR;
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 1);
	}
}

void set_dir_motor12(uint8_t _sens1, uint8_t _sens2) {
	if (_sens1 == FORWARD_DIR) {
		motors.motors[MOTOR1].direction = FORWARD_DIR;
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0);
	} else {
		motors.motors[MOTOR1].direction = REVERSE_DIR;
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 1);
	}

	if (_sens2 == FORWARD_DIR) {
		motors.motors[MOTOR2].direction = FORWARD_DIR;
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0);
	} else {
		motors.motors[MOTOR2].direction = REVERSE_DIR;
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 1);
	}
}

void test_motor1(void)
{
	int32_t i = 0;

	for (i = 0; i <= MOTORS_PWM_PERIOD; i++) {
		set_m1_speed(i);
		HAL_Delay(5);
	}

	for (i = MOTORS_PWM_PERIOD; i >= -MOTORS_PWM_PERIOD; i--) {
		set_m1_speed(i);
		HAL_Delay(5);
	}

	for (i = -MOTORS_PWM_PERIOD; i <= 0; i++) {
		set_m1_speed(i);
		HAL_Delay(5);
	}
}

void test_motor2(void)
{
	int32_t i = 0;

	for (i = 0; i <= MOTORS_PWM_PERIOD; i++) {
		set_m2_speed(i);
		HAL_Delay(5);
	}

	for (i = MOTORS_PWM_PERIOD; i >= -MOTORS_PWM_PERIOD; i--) {
		set_m2_speed(i);
		HAL_Delay(5);
	}

	for (i = -MOTORS_PWM_PERIOD; i <= 0; i++) {
		set_m2_speed(i);
		HAL_Delay(5);
	}
}

void test_motor12(void)
{
	int32_t i = 0;

	for (i = 0; i <= MOTORS_PWM_PERIOD; i++) {
		set_m1_speed(i);
		set_m2_speed(i);
		HAL_Delay(5);
	}

	for (i = MOTORS_PWM_PERIOD; i >= -MOTORS_PWM_PERIOD; i--) {
		set_m1_speed(i);
		set_m2_speed(i);
		HAL_Delay(5);
	}

	for (i = -MOTORS_PWM_PERIOD; i <= 0; i++) {
		set_m1_speed(i);
		set_m2_speed(i);
		HAL_Delay(5);
	}
}

void motor_test_task(void* _params)
{
	//int ret = 0;
	int32_t i = 0;
	uint8_t dir = FORWARD_DIR;

	if (_params != 0) { }

	while (1) {
		if (dir == FORWARD_DIR)
			i++;
		else
			i--;

		if (i >= MOTORS_PWM_PERIOD-1) 
			dir = REVERSE_DIR;
		else if (i <= -MOTORS_PWM_PERIOD+1)
			dir = FORWARD_DIR;
		
		set_m1_speed(i);
		set_m2_speed(i);

		motors.timestamp = (float)get_us_time() * (float)1e-6;
		xQueueOverwrite(motorQueue, &motors);

		vTaskDelay(5/portTICK_RATE_MS);
	}

	vTaskDelete(NULL);
}

void motor_ident_task(void* _params)
{
	MotorMeasuredSpeed_t motorMeasuredSpeeds;
	uint8_t ret = 0;
	float t = 0.f;
	float vBat = NOMINAL_BATTERY_VOLTAGE; // [V]
	int32_t speeds[N_MOTORS] = { 0, };

	if (_params != 0) { }

	memset((void*)&motorMeasuredSpeeds, 0, sizeof(MotorMeasuredSpeed_t));

	while (1) {
		ret = encoder_read_motor_measured_speed(&motorMeasuredSpeeds, 
				pdMS_TO_TICKS(ENCODER_MEASUREMENT_PERIOD_MS));
		if (ret) {
			printf("%3.3f,%3.3f,%3.3f\r\n",
				inputVoltage,
				motorMeasuredSpeeds.speed[MOTOR1],
				motorMeasuredSpeeds.speed[MOTOR2]);

			t = (float)get_us_time() * (float)1e-6;
			//inputVoltage = triangularSignal(.1, vBat, t);
			//inputVoltage = squareSignal(.1, vBat, t);
			inputVoltage = sinusoidSignal(3.1f, t);
			speeds[MOTOR1] = voltageToPwm(inputVoltage, vBat, MOTOR1);
			speeds[MOTOR2] = voltageToPwm(inputVoltage, vBat, MOTOR2);
			set_m1_speed(speeds[MOTOR1]);
			set_m2_speed(speeds[MOTOR2]);

			motors.timestamp = t;
			xQueueOverwrite(motorQueue, &motors);
		}
	}

	vTaskDelete(NULL);
}

void motor_task(void* _params)
{
	MotorDesiredVoltage_t desiredVoltages;
	uint8_t ret = 0;
	int32_t speeds[N_MOTORS] = { 0, };
	float vBat = NOMINAL_BATTERY_VOLTAGE; // [V]
	uint16_t iMot[N_MOTORS] = {0, }; // [mA]

	if (_params != 0) { }

	memset((void*)&desiredVoltages, 0, sizeof(MotorDesiredVoltage_t));

	/*
	ret = init_adc_motors();
	if (ret != 0) {
		char str[] = "init_adc_motors error\r\n";
		print_msg((uint8_t*)str, strlen(str));
		Error_Handler();
	}
	*/
	while (1) {
        // 1. Get desired motor voltage
        ret = motor_control_read_desired_voltage(&desiredVoltages, 
        		pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
        if (ret) {
        	motors.timestamp = desiredVoltages.timestamp;

            motors.motors[MOTOR1].desiredVoltage = desiredVoltages.voltage[MOTOR1];
            motors.motors[MOTOR2].desiredVoltage = desiredVoltages.voltage[MOTOR2];

            // Get current battery voltage
            //vBat = power_get_battery_voltage();

            // Convert voltage to PWM
			speeds[MOTOR1] = voltageToPwm(motors.motors[MOTOR1].desiredVoltage, vBat, MOTOR1);
			speeds[MOTOR2] = voltageToPwm(motors.motors[MOTOR2].desiredVoltage, vBat, MOTOR2);
			set_m1_speed(speeds[MOTOR1]);
			set_m2_speed(speeds[MOTOR2]);

			
			xQueueOverwrite(motorQueue, &motors);
        }
        else {
        	motors.timestamp = (float)get_us_time() * (float)1e-6;
        }

        // 2. Read ADC motor currents
        //get_adc_imot12_ma(&iMot[MOTOR1], &iMot[MOTOR2]);
        motors.motors[MOTOR1].current = (float)iMot[MOTOR1] / 1000.f;
        motors.motors[MOTOR2].current = (float)iMot[MOTOR2] / 1000.f;

        // 3. Get motor fault
        motors.motors[MOTOR1].fault = get_m1_fault();
        motors.motors[MOTOR2].fault = get_m2_fault();
		
		xQueueOverwrite(motorQueue, &motors);
	}

	vTaskDelete(NULL);
}

int motor_read_motor_data(Motor_t* mot)
{
	return (pdTRUE == xQueueReceive(motorQueue, mot, 0));
}

int32_t voltageToPwm(float _volt, float _vbat, uint8_t _motor)
{
	float percentage = _volt / _vbat;
	percentage = (percentage > 1.f) ? 1.f : percentage;
	percentage = (percentage < -1.f) ? -1.f : percentage;
	motors.motors[_motor].dutyCycle = percentage;
	float pwmf = percentage * (float)MOTORS_PWM_PERIOD;
	return (int32_t)round(pwmf);
}

float getInputVoltage(void)
{
	return inputVoltage;
}

float sinusoidSignal(float a, float t)
{
	return a * (
		sin(2.f*M_PI*t) + sin(2.f*M_PI*.1f*t) + 
		sin(2.f*M_PI*.2f*t) + sin(2.f*M_PI*.3f*t) + 
		sin(2.f*M_PI*.4f*t) + sin(2.f*M_PI*.5f*t));
}

float squareSignal(float f, float a, float t)
{
	float voltage = sinf(2.f*M_PI*f*t);
	if (voltage >= 0.f)
		voltage = a;
	else
		voltage = 0.;
	return voltage;
}

float triangularSignal(float f, float a, float t)
{
	return (2.f * a / M_PI) * asinf(sinf(2.f*M_PI*f*t));
}