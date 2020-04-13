/**
 * \file gpio.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief GPIO initialization
 */

#include "gpio.h"
#include "main.h"

int init_gpios(void)
{
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

    // USART1
    USART1_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = USART1_TX_PIN | USART1_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USART1_GPIO_AF;
    HAL_GPIO_Init(USART1_GPIO_PORT, &GPIO_InitStruct);

    // USART2
    USART2_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = USART2_TX_PIN | USART2_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USART2_GPIO_AF;
    HAL_GPIO_Init(USART2_GPIO_PORT, &GPIO_InitStruct);

    // USART3
    USART3_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = USART3_TX_PIN | USART3_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_NOPULL
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = USART3_GPIO_AF;
    HAL_GPIO_Init(USART3_GPIO_PORT, &GPIO_InitStruct);

    // I2C1
    I2C1_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin       = I2C1_SCL_PIN | I2C1_SDA_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = I2C1_GPIO_AF;
    HAL_GPIO_Init(I2C1_GPIO_PORT, &GPIO_InitStruct);

    // I2C2
    I2C2_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin       = I2C2_SCL_PIN | I2C2_SDA_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = I2C2_GPIO_AF;
    HAL_GPIO_Init(I2C2_GPIO_PORT, &GPIO_InitStruct);

    // Encoder1 (TIM3)
    ENC1A_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = ENC1A_PIN | ENC1B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //GPIO_PULLUP; pullup on the PCB
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(ENC1A_GPIO, &GPIO_InitStruct);

    // Encoder2 (TIM4)
    ENC2A_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = ENC2A_PIN | ENC2B_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_NOPULL; //GPIO_PULLUP; pullup on the PCB
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

    // Enable servo PWM
    TIM_PWM_SERVO_GPIO_CLK();
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    GPIO_InitStruct.Pin = TIM_PWM_SERVO1_PIN | TIM_PWM_SERVO2_PIN | 
        TIM_PWM_SERVO3_PIN | TIM_PWM_SERVO4_PIN;
    HAL_GPIO_Init(TIM_PWM_SERVO_GPIO_PORT, &GPIO_InitStruct);

    // Enable buzzer PWM
    TIM_BUZZER_GPIO_CLK();
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; // GPIO_PULLUP
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    GPIO_InitStruct.Pin = TIM_BUZZER_GPIO_PIN;
    HAL_GPIO_Init(TIM_BUZZER_GPIO_PORT, &GPIO_InitStruct);

    // Enable LEDs
    LEDS_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; //GPIO_PULLDOWN
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = LED1_PIN | LED2_PIN | LED3_PIN;
    HAL_GPIO_Init(LEDS_GPIO_PORT, &GPIO_InitStruct);

    // Enable battery voltage and current monitor
    ADC_BAT_GPIO_CLK_ENABLE();
    GPIO_InitStruct.Pin = ADC_VBAT_PIN | ADC_IBAT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_BAT_GPIO_PORT, &GPIO_InitStruct);

    return OK;
} // init_gpios
