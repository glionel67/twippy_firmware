/**
 * \file spi.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief SPI1 functions
 */
 
#include "spi.h"
#include "main.h"

#define DEBUG_MODULE "spi"

SPI_HandleTypeDef Spi1Handle;

uint8_t spi1_init(void)
{
  // Enable SPI clock
  SPI1_CLK_ENABLE();

  // Set the SPI parameters
  Spi1Handle.Instance               = SPI1_IMU;
  Spi1Handle.Init.BaudRatePrescaler = SPI_BAUDRATE_0_7MHZ;
  Spi1Handle.Init.Direction         = SPI_DIRECTION_2LINES;
  Spi1Handle.Init.Mode              = SPI_MODE_MASTER;
  Spi1Handle.Init.CLKPhase          = SPI_PHASE_2EDGE; // SPI_PHASE_1EDGE
  Spi1Handle.Init.CLKPolarity       = SPI_POLARITY_HIGH; // SPI_POLARITY_LOW
  Spi1Handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  Spi1Handle.Init.CRCPolynomial     = 7;
  Spi1Handle.Init.DataSize          = SPI_DATASIZE_8BIT;
  Spi1Handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  Spi1Handle.Init.NSS               = SPI_NSS_SOFT; // SPI_NSS_HARD_OUTPUT
  Spi1Handle.Init.TIMode            = SPI_TIMODE_DISABLE;

  if (HAL_OK != HAL_SPI_Init(&Spi1Handle))
      return NOK;

  return OK;
}

void spi1_deInit(void)
{
  // ##-1- Reset peripherals
  __HAL_RCC_SPI1_FORCE_RESET();
  __HAL_RCC_SPI1_RELEASE_RESET();

  // ##-2- Disable peripherals and GPIO Clocks 
  HAL_GPIO_DeInit(SPI1_GPIO_PORT, SPI1_CLK_PIN);
  HAL_GPIO_DeInit(SPI1_GPIO_PORT, SPI1_MISO_PIN);
  HAL_GPIO_DeInit(SPI1_GPIO_PORT, SPI1_MOSI_PIN);
  HAL_GPIO_DeInit(SPI1_GPIO_PORT, SPI1_SS_PIN);

  HAL_SPI_DeInit(&Spi1Handle);
}

uint8_t spi1_set_speed(uint32_t baudratePrescaler)
{
  SPI1_IMU->CR1 = (SPI1_IMU->CR1 & ~SPI_BAUDRATEPRESCALER_256) | baudratePrescaler;
  /*
  HAL_SPI_DeInit(&Spi1Handle);

  // Enable SPI clock
  SPI1_CLK_ENABLE();
  __HAL_SPI_ENABLE(&Spi1Handle);

  // Set the SPI parameters
  Spi1Handle.Instance               = SPI1_IMU;
  Spi1Handle.Init.BaudRatePrescaler = baudratePrescaler;
  Spi1Handle.Init.Direction         = SPI_DIRECTION_2LINES;
  Spi1Handle.Init.Mode              = SPI_MODE_MASTER;
  Spi1Handle.Init.CLKPhase          = SPI_PHASE_2EDGE; // SPI_PHASE_1EDGE
  Spi1Handle.Init.CLKPolarity       = SPI_POLARITY_HIGH; // SPI_POLARITY_LOW
  Spi1Handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  Spi1Handle.Init.CRCPolynomial     = 7;
  Spi1Handle.Init.DataSize          = SPI_DATASIZE_8BIT;
  Spi1Handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  Spi1Handle.Init.NSS               = SPI_NSS_SOFT; // SPI_NSS_HARD_OUTPUT
  Spi1Handle.Init.TIMode            = SPI_TIMODE_DISABLE;

  if (HAL_SPI_Init(&Spi1Handle) != HAL_OK) {
      return 0;
  }
*/
  return OK;
}

uint8_t write_byte_spi1(uint8_t reg, uint8_t data)
{
  uint8_t ret = 0;
  uint8_t tx[2] = {reg, data};

  HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_RESET);

  ret = HAL_SPI_Transmit(&Spi1Handle, tx, 2, SPI1_TIMEOUT);

  HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

  if (HAL_OK != ret)
    return NOK;
  else
    return OK;
}


uint8_t read_byte_spi1(uint8_t reg, uint8_t* data)
{
  uint8_t ret = 0;
  uint8_t tx[2] = {reg | SPI_READ_WRITE_BIT, 0};
  uint8_t rx[2] = {0, 0};

  HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_RESET);

  ret = HAL_SPI_TransmitReceive(&Spi1Handle, tx, rx, 2, SPI1_TIMEOUT);

  HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

  *data = rx[1];

  if (HAL_OK != ret)
    return NOK;
  else
    return OK;

  //if (HAL_SPI_Transmit(&Spi1Handle, &reg, 1, SPI1_TIMEOUT) != HAL_OK)
  //    return 0;
  //if (HAL_SPI_Receive(&Spi1Handle, data, 1, SPI1_TIMEOUT) != HAL_OK)
  //    return 0;
}


uint8_t write_bytes_spi1(uint8_t reg, uint8_t* data, uint8_t length)
{
  uint8_t ret = 0;

  HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_RESET);

  ret = HAL_SPI_Transmit(&Spi1Handle, &reg, 1, SPI1_TIMEOUT);
  ret = HAL_SPI_Transmit(&Spi1Handle, data, length, SPI1_TIMEOUT);

  HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

  if (HAL_OK != ret)
    return NOK;
  else
    return OK;
}


uint8_t read_bytes_spi1(uint8_t reg, uint8_t* data, uint8_t length)
{
  uint8_t ret = 0;

  reg |= SPI_READ_WRITE_BIT;

  HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_RESET);

  ret = HAL_SPI_Transmit(&Spi1Handle, &reg, 1, SPI1_TIMEOUT);
  ret = HAL_SPI_Receive(&Spi1Handle, data, length, SPI1_TIMEOUT);

  HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

  if (HAL_OK != ret)
    return NOK;
  else
    return OK;
}
