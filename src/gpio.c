#include "gpio.h"

#include "main.h"

int init_gpios(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	// Configure all unused GPIO port pins in Analog Input mode (floating input
	// trigger OFF), this will reduce the power consumption
	// and increase the device immunity against EMI/EMC

	// Enable GPIOs clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	// Set all GPIO pins as analog inputs
	GPIO_InitStruct.Pin = GPIO_PIN_All;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	// Init LED D13 on PA5
	//__HAL_RCC_GPIOA_CLK_ENABLE();
	//GPIO_InitStruct.Pin = GPIO_PIN_5;
	//GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	//GPIO_InitStruct.Pull = GPIO_PULLUP;
	//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

	// USART1
	USART1_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin = USART1_TX_PIN | USART1_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = USART1_GPIO_AF;
	HAL_GPIO_Init(USART1_GPIO_PORT, &GPIO_InitStruct);

	// USART2
	USART2_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin = USART2_TX_PIN | USART2_RX_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = USART2_GPIO_AF;
	HAL_GPIO_Init(USART2_GPIO_PORT, &GPIO_InitStruct);

	// Encoder1 (TIM3)
	ENC1A_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin = ENC1A_PIN | ENC1B_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(ENC1A_GPIO, &GPIO_InitStruct);

	// Encoder2 (TIM4)
	ENC2A_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Pin = ENC2A_PIN | ENC2B_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(ENC2A_GPIO, &GPIO_InitStruct);

	// Enable motor Isense
	ADC_IMOT_GPIO_CLK_ENABLE(); // Enable GPIO clock
	GPIO_InitStruct.Pin = ADC_IMOT1_PIN | ADC_IMOT2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_IMOT_GPIO, &GPIO_InitStruct);

	// Enable left/right motor PWM
	TIM_PWM_MOTORS_GPIO_CLK();
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	GPIO_InitStruct.Pin = TIM_PWM_MOTOR1_PIN | TIM_PWM_MOTOR2_PIN;
	HAL_GPIO_Init(TIM_PWM_MOTORS_GPIO_PORT, &GPIO_InitStruct);

	// Motors DIR+BRAKE
	MOTORS_GPIO_CLK_ENABLE();
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = M1_INA_PIN | M1_INB_PIN | M2_INA_PIN | M2_INB_PIN;
	HAL_GPIO_Init(MOTORS_GPIO, &GPIO_InitStruct);

	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pin = M1_EN_PIN | M2_EN_PIN;
	HAL_GPIO_Init(MOTORS_GPIO, &GPIO_InitStruct);

	// IMU MPU9250 SPI1
	SPI1_GPIO_CLK_ENALBE();
	// SPI SCK/MISO/MOSI GPIO pin configuration
	GPIO_InitStruct.Pin       = SPI1_CLK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Alternate = SPI1_GPIO_AF;
	HAL_GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStruct);

	// SPI1 SS pin setup
	GPIO_InitStruct.Pin       = SPI1_SS_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	HAL_GPIO_Init(SPI1_GPIO_PORT, &GPIO_InitStruct);
	// Set CS high
	HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

	// IMU interrupt pin configuration
	IMU_INT_CLK_ENABLE();
	GPIO_InitStruct.Pin   = IMU_INT_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN; // GPIO_NOPULL
	HAL_GPIO_Init(IMU_INT_GPIO, &GPIO_InitStruct);
	
    // Activate IMU INT PIN
    //HAL_NVIC_SetPriority(IMU_INT_IRQ, 5, 0);
    //HAL_NVIC_EnableIRQ(IMU_INT_IRQ);

	return 0;
}
