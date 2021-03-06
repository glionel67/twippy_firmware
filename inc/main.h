/**
 * \file main.h
 * \author Lionel GENEVE
 * \date 12/04/2020
 * \version 1.0
 * \brief Mapping and configuration of the GPIOs
 */

#pragma once

#include "stm32f4xx_hal.h"

/******************************************************************************/
/* Definition for USART1  */
/******************************************************************************/
#define USART1_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART1_GPIO_PORT            GPIOA
#define USART1_TX_PIN               GPIO_PIN_9
#define USART1_RX_PIN               GPIO_PIN_10
#define USART1_GPIO_AF              GPIO_AF7_USART1
#define USART1_BAUDRATE             115200 //9600
#define USART1_TIMEOUT              100 // [ms]
#define USART1_IRQ                  USART1_IRQn

/******************************************************************************/
/* Definition for USART2  */
/******************************************************************************/
#define USART2_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART2_GPIO_PORT            GPIOA
#define USART2_TX_PIN               GPIO_PIN_2
#define USART2_RX_PIN               GPIO_PIN_3
#define USART2_GPIO_AF              GPIO_AF7_USART2
#define USART2_BAUDRATE             115200 //9600
#define USART2_TIMEOUT              100 // [ms]
#define USART2_IRQ                  USART2_IRQn

/******************************************************************************/
/* Definition for USART3  */
/******************************************************************************/
#define USART3_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define USART3_GPIO_PORT            GPIOC
#define USART3_TX_PIN               GPIO_PIN_10
#define USART3_RX_PIN               GPIO_PIN_11
#define USART3_GPIO_AF              GPIO_AF7_USART3
#define USART3_BAUDRATE             115200 //9600
#define USART3_TIMEOUT              100 // [ms]
#define USART3_IRQ                  USART3_IRQn

/******************************************************************************/
/* Definition for I2C1  */
/******************************************************************************/
#define I2C1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C1_GPIO_PORT              GPIOB
#define I2C1_SCL_PIN                GPIO_PIN_8
#define I2C1_SDA_PIN                GPIO_PIN_9
#define I2C1_GPIO_AF                GPIO_AF4_I2C1
#define I2C1_CLOCK_SPEED            400000
#define I2C1_OWN_ADDRESS            0x30F
#define I2C1_TIMEOUT                100 // [ms]
// #define I2C1_EV_IRQn                    I2C1_EV_IRQn
// #define I2C1_EV_IRQHandler              I2C1_EV_IRQHandler
// #define I2C1_ER_IRQn                    I2C1_ER_IRQn
// #define I2C1_ER_IRQHandler              I2C1_ER_IRQHandler

/******************************************************************************/
/* Definition for I2C2  */
/******************************************************************************/
#define I2C2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2C2_GPIO_PORT              GPIOB
#define I2C2_SCL_PIN                GPIO_PIN_10
#define I2C2_SDA_PIN                GPIO_PIN_3
#define I2C2_GPIO_AF                GPIO_AF4_I2C2
#define I2C2_CLOCK_SPEED            400000
#define I2C2_OWN_ADDRESS            0x40F
#define I2C2_TIMEOUT                100 // [ms]
// #define I2C2_EV_IRQn                    I2C2_EV_IRQn
// #define I2C2_EV_IRQHandler              I2C2_EV_IRQHandler
// #define I2C2_ER_IRQn                    I2C2_ER_IRQn
// #define I2C2_ER_IRQHandler              I2C2_ER_IRQHandler

/******************************************************************************/
/* Definition for encoders */
/******************************************************************************/
#define TIM_ENC1            TIM3
#define TIM_ENC1_CLK_ENABLE()           __HAL_RCC_TIM3_CLK_ENABLE()
#define TIM_ENC1_CHANNEL_GPIO_PORT()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define TIM_ENC1_GPIO_PORT_CHANNEL      GPIOB
#define TIM_ENC1A_GPIO_PIN_CHANNEL      GPIO_PIN_4
#define TIM_ENC1A_GPIO_AF_CHANNEL       GPIO_AF2_TIM3
#define TIM_ENC1A_CHANNEL       TIM_CHANNEL_1
#define TIM_ENC1B_GPIO_PIN_CHANNEL      GPIO_PIN_5
#define TIM_ENC1B_GPIO_AF_CHANNEL       GPIO_AF2_TIM3
#define TIM_ENC1B_CHANNEL       TIM_CHANNEL_2

#define TIM_ENC2            TIM4
#define TIM_ENC2_CLK_ENABLE()           __HAL_RCC_TIM4_CLK_ENABLE()
#define TIM_ENC2_CHANNEL_GPIO_PORT()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define TIM_ENC2_GPIO_PORT_CHANNEL      GPIOB
#define TIM_ENC2A_GPIO_PIN_CHANNEL      GPIO_PIN_6
#define TIM_ENC2A_GPIO_AF_CHANNEL     GPIO_AF2_TIM4
#define TIM_ENC2A_CHANNEL       TIM_CHANNEL_1
#define TIM_ENC2B_GPIO_PIN_CHANNEL      GPIO_PIN_7
#define TIM_ENC2B_GPIO_AF_CHANNEL     GPIO_AF2_TIM4
#define TIM_ENC2B_CHANNEL       TIM_CHANNEL_2

#define ENC1A_GPIO          GPIOB
#define ENC1A_PIN         GPIO_PIN_4
#define ENC1A_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define ENC1A_IRQn                  TIM3_IRQn

#define ENC1B_GPIO          GPIOB
#define ENC1B_PIN         GPIO_PIN_5
#define ENC1B_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define ENC1B_IRQn                  TIM3_IRQn

#define ENC2A_GPIO          GPIOB
#define ENC2A_PIN         GPIO_PIN_6
#define ENC2A_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define ENC2A_IRQn                  TIM4_IRQn

#define ENC2B_GPIO          GPIOB
#define ENC2B_PIN         GPIO_PIN_7
#define ENC2B_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define ENC2B_IRQn                  TIM4_IRQn

#define LEFT_COUNT()        TIM_ENC1->CNT
#define RIGHT_COUNT()       TIM_ENC2->CNT

/******************************************************************************/
/* Definition for PWM motor (TIM)  */
/******************************************************************************/
#define TIM_PWM_MOTORS              TIM1
#define TIM_PWM_MOTORS_CLK_ENABLE() __HAL_RCC_TIM1_CLK_ENABLE()
#define TIM_PWM_MOTORS_GPIO_CLK()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define TIM_PWM_MOTORS_GPIO_PORT    GPIOA
#define TIM_PWM_MOTOR1_PIN          GPIO_PIN_8
#define TIM_PWM_MOTOR2_PIN          GPIO_PIN_11
#define TIM_PWM_MOTOR1_CHANNEL      TIM_CHANNEL_1
#define TIM_PWM_MOTOR2_CHANNEL      TIM_CHANNEL_4

/******************************************************************************/
/* Definition for motor DIR+BRAKE */
/******************************************************************************/
#define MOTORS_GPIO                 GPIOC
#define MOTORS_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define M1_INA_PIN                  GPIO_PIN_0
#define M1_INB_PIN                  GPIO_PIN_1
#define M1_EN_PIN                   GPIO_PIN_2
#define M2_INA_PIN                  GPIO_PIN_3
#define M2_INB_PIN                  GPIO_PIN_4
#define M2_EN_PIN                   GPIO_PIN_5

/******************************************************************************/
/* Definition for ADC1 (IMOT) */
/******************************************************************************/
// Definition for ADC1 clock resources
#define ADC_IMOT                        ADC1
#define ADC_IMOT_CLK_ENABLE()           __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC_IMOT_FORCE_RESET()          __HAL_RCC_ADC_FORCE_RESET()
#define ADC_IMOT_RELEASE_RESET()        __HAL_RCC_ADC_RELEASE_RESET()
#define ADC_IMOT_SAMPLING_TIME         	ADC_SAMPLETIME_15CYCLES
#define ADC_IMOT_RESOLUTION         	4096 // 12 bits, ((uint32_t)4095)
#define ADC_IMOT_VREF             		3300 // [mV]

// Definition for motor ISENSE
#define ADC_IMOT_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOA_CLK_ENABLE()
#define ADC_IMOT_GPIO             	GPIOA
#define ADC_IMOT1_PIN             	GPIO_PIN_0
#define ADC_IMOT2_PIN             	GPIO_PIN_1
#define ADC_IMOT1_CHANNEL         	ADC_CHANNEL_1
#define ADC_IMOT2_CHANNEL         	ADC_CHANNEL_2

#define IMOT_RATIO_NUM            	100 //1.65V/A
#define IMOT_RATIO_DEN            	165

#define IMOT_MAX                	1600 // [mA]

// Definition for Vbat and Ibat
#define ADC_BAT_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define ADC_BAT_GPIO_PORT           GPIOB
#define ADC_VBAT_PIN                GPIO_PIN_0
#define ADC_IBAT_PIN                GPIO_PIN_1
#define ADC_VBAT_CHANNEL            ADC_CHANNEL_8
#define ADC_IBAT_CHANNEL            ADC_CHANNEL_9

#define ADC_BAT_RESOLUTION          (4096.f)
#define ADC_BAT_VREF                (3.3f) // [Volt]
#define VBAT_RATIO                  (6.f)
#define IBAT_RATIO                  (15.1515f) // (66 mV/A) centered at 2.5 V
#define IBAT_ZERO                   (2.5f) // [Volt]

/******************************************************************************/
/* Definition for SPI1 (IMU MPU9250) */
/******************************************************************************/
#define SPI1_IMU SPI1
#define SPI1_CLK_ENABLE() __HAL_RCC_SPI1_CLK_ENABLE()

#define SPI1_GPIO_PORT GPIOA
#define SPI1_GPIO_AF GPIO_AF5_SPI1
#define SPI1_GPIO_CLK_ENALBE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPI1_SS_PIN GPIO_PIN_4
#define SPI1_CLK_PIN GPIO_PIN_5
#define SPI1_MISO_PIN GPIO_PIN_6
#define SPI1_MOSI_PIN GPIO_PIN_7

#define SPI1_TIMEOUT 10

#define IMU_INT_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define IMU_INT_GPIO GPIOB
#define IMU_INT_PIN GPIO_PIN_2
#define IMU_INT_IRQ EXTI2_IRQn

/******************************************************************************/
/* Definition for PWM Servo (TIM)  */
/******************************************************************************/
#define TIM_PWM_SERVO              TIM8
#define TIM_PWM_SERVO_CLK_ENABLE() __HAL_RCC_TIM8_CLK_ENABLE()
#define TIM_PWM_SERVO_GPIO_CLK()   __HAL_RCC_GPIOC_CLK_ENABLE()
#define TIM_PWM_SERVO_GPIO_PORT    GPIOC
#define TIM_PWM_SERVO1_PIN          GPIO_PIN_6
#define TIM_PWM_SERVO2_PIN          GPIO_PIN_7
#define TIM_PWM_SERVO3_PIN          GPIO_PIN_8
#define TIM_PWM_SERVO4_PIN          GPIO_PIN_9
#define TIM_PWM_SERVO1_CHANNEL      TIM_CHANNEL_1
#define TIM_PWM_SERVO2_CHANNEL      TIM_CHANNEL_2
#define TIM_PWM_SERVO3_CHANNEL      TIM_CHANNEL_3
#define TIM_PWM_SERVO4_CHANNEL      TIM_CHANNEL_4

/******************************************************************************/
/* Definition for PWM Buzzer (TIM)  */
/******************************************************************************/
#define TIM_BUZZER              TIM2
#define TIM_BUZZER_CLK_ENABLE() __HAL_RCC_TIM2_CLK_ENABLE()
#define TIM_BUZZER_GPIO_CLK()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define TIM_BUZZER_GPIO_PORT    GPIOA
#define TIM_BUZZER_GPIO_PIN     GPIO_PIN_15
#define TIM_BUZZER_CHANNEL      TIM_CHANNEL_1

/******************************************************************************/
/* Definition for LEDs  */
/******************************************************************************/
#define LEDS_GPIO_PORT                      GPIOC
#define LEDS_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()
#define LEDS_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOC_CLK_DISABLE()

#define LED1_PIN                            GPIO_PIN_13
#define LED2_PIN                            GPIO_PIN_14
#define LED3_PIN                            GPIO_PIN_15

/******************************************************************************/
/* Definition for dummies  */
/******************************************************************************/
#define NOK 0
#define OK 1

#define TRUE 1
#define FALSE 0

#define NOMINAL_BATTERY_VOLTAGE (12.f) // [Volt]


/******************************************************************************/
/* Function prototypes */
/******************************************************************************/
void Error_Handler(void);
