#include "motor.h"
#include "main.h"

// Timer handler declaration
TIM_HandleTypeDef TimHandleMotors;

// Timer Output Compare Configuration Structure declaration
static TIM_OC_InitTypeDef sConfigMotors;

uint8_t dirMotors[N_MOTORS] = { 0, };
uint16_t pwmMotors[N_MOTORS] = { 0, };
float dcMotors[N_MOTORS] = { 0, };

int init_motors(void) {
	int ret = 0;
	uint8_t i = 0;

	for (i = 0; i < N_MOTORS; i++) {
		pwmMotors[i] = 0;
		dcMotors[i] = 0;
		dirMotors[i] = 0;
	}

	// PWM motors
	TIM_PWM_MOTORS_CLK_ENABLE();
	TimHandleMotors.Instance = TIM_PWM_MOTORS;
	TimHandleMotors.Init.Prescaler = MOTORS_PWM_PRESCALER;
	TimHandleMotors.Init.Period = MOTORS_PWM_PERIOD;
	TimHandleMotors.Init.ClockDivision = 0;
	TimHandleMotors.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandleMotors.Init.RepetitionCounter = 0;
	ret = HAL_TIM_PWM_Init(&TimHandleMotors);
	if (ret != HAL_OK) {
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
		return -1;
	}

	ret = HAL_TIM_PWM_Start(&TimHandleMotors,
			TIM_PWM_MOTOR1_CHANNEL | TIM_PWM_MOTOR2_CHANNEL);
	if (ret != HAL_OK) {
		return -1;
	}

	return 0;
}

int set_pwm1(uint16_t _pwm) {
	int ret = 0;
	pwmMotors[0] = _pwm;
	sConfigMotors.Pulse = _pwm;
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

int set_pwm2(uint16_t _pwm) {
	int ret = 0;
	pwmMotors[1] = _pwm;
	sConfigMotors.Pulse = _pwm;
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

int set_pwm12(uint16_t _pwm1, uint16_t _pwm2) {
	int ret = 0;
	pwmMotors[0] = _pwm1;
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
	pwmMotors[1] = _pwm2;
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

int set_dc_pwm1(float _dc) {
	dcMotors[0] = _dc;
	pwmMotors[0] = (uint16_t)(_dc * (float) MOTORS_PWM_PERIOD);
	return set_pwm1(pwmMotors[0]);
}

int set_dc_pwm2(float _dc) {
	dcMotors[1] = _dc;
	pwmMotors[1] = (uint16_t)(_dc * (float) MOTORS_PWM_PERIOD);
	return set_pwm2(pwmMotors[1]);
}

int set_dc_pwm12(float _dc1, float _dc2) {
	if (set_dc_pwm1(_dc1) != 0)
		return -1;
	if (set_dc_pwm2(_dc2) != 0)
		return -1;
	return 0;
}

int set_m1_speed(int32_t _speed) {
	int ret = 0;

	if (_speed < 0) {
		_speed = -_speed; // Make speed a positive quantity
		dirMotors[0] = 0; // reverse
	}
	else {
		dirMotors[0] = 1; // forward
	}

	if (_speed > MOTORS_PWM_PERIOD) // Max PWM dutycycle
		_speed = MOTORS_PWM_PERIOD;

	if (_speed == 0) {
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0); // Make the motor coast no
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0); // matter which direction it is spinning.
	}
	else if (dirMotors[0] == 0) { // reverse
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 1);
	}
	else {
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0);
	}

	pwmMotors[0] = (uint16_t)_speed;
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

int set_m2_speed(int32_t _speed) {
	int ret = 0;

	if (_speed < 0) {
		_speed = -_speed; // Make speed a positive quantity
		dirMotors[1] = 0; // reverse
	}
	else {
		dirMotors[1] = 1; // forward
	}

	if (_speed > MOTORS_PWM_PERIOD) // Max PWM dutycycle
		_speed = MOTORS_PWM_PERIOD;

	if (_speed == 0) {
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0); // Make the motor coast no
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0); // matter which direction it is spinning.
	}
	else if (dirMotors[1] == 0) { // reverse
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 1);
	}
	else {
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0);
	}

	pwmMotors[1] = (uint16_t)_speed;
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

int set_m1_brake(int32_t _brake) {
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

int set_m2_brake(int32_t _brake) {
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

uint8_t get_m1_fault(void) {
	return !HAL_GPIO_ReadPin(MOTORS_GPIO, M1_EN_PIN); // 1 = fault
}

uint8_t get_m2_fault(void) {
	return !HAL_GPIO_ReadPin(MOTORS_GPIO, M2_EN_PIN); // 1 = fault
}

void brake_motor1(void) {
	HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0); // Make the motor coast no
	HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0); // matter which direction it is spinning.
}

void brake_motor2(void) {
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
	if (_sens == 1) {
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0);
	} else {
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 1);
	}
}

void set_dir_motor2(uint8_t _sens) {
	if (_sens == 1) {
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0);
	} else {
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 1);
	}
}

void set_dir_motor12(uint8_t _sens1, uint8_t _sens2) {
	if (_sens1 == 1) {
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 0);
	} else {
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M1_INB_PIN, 1);
	}

	if (_sens2 == 1) {
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 1);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 0);
	} else {
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INA_PIN, 0);
		HAL_GPIO_WritePin(MOTORS_GPIO, M2_INB_PIN, 1);
	}
}

void test_motor1(void) {
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

void test_motor2(void) {
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

void test_motor12(void) {
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
