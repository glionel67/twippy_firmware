#include "spi.h"
#include "main.h"

SPI_HandleTypeDef Spi1Handle;

int spi1_init(void) {
  
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

  if (HAL_SPI_Init(&Spi1Handle) != HAL_OK) {
      return -1;
  }

  return 0;
}

void spi1_deInit(void) {
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

int spi1_set_speed(uint32_t baudratePrescaler) {
  HAL_SPI_DeInit(&Spi1Handle);

  // Enable SPI clock
  SPI1_CLK_ENABLE();

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
      return -1;
  }

  return 0;
}

uint8_t write_byte_spi1(uint8_t reg, uint8_t data) {
    reg &= ~SPI_READ_WRITE_BIT;

    HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&Spi1Handle, &reg, 1, SPI1_TIMEOUT) != HAL_OK)
        return 0;
    if (HAL_SPI_Transmit(&Spi1Handle, &data, 1, SPI1_TIMEOUT) != HAL_OK)
        return 0;
    HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

    return 1;
}


uint8_t read_byte_spi1(uint8_t reg, uint8_t* data) {
    reg |= SPI_READ_WRITE_BIT;

    HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&Spi1Handle, &reg, 1, SPI1_TIMEOUT) != HAL_OK)
        return 0;
    if (HAL_SPI_Receive(&Spi1Handle, data, 1, SPI1_TIMEOUT) != HAL_OK)
        return 0;
    HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

    return 1;
}


uint8_t write_bytes_spi1(uint8_t reg, uint8_t* data, uint8_t length) {
    reg &= ~SPI_READ_WRITE_BIT;

    HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&Spi1Handle, &reg, 1, SPI1_TIMEOUT) != HAL_OK)
        return 0;
    if (HAL_SPI_Transmit(&Spi1Handle, data, length, SPI1_TIMEOUT) != HAL_OK)
        return 0;
    HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

    return 1;
}


uint8_t read_bytes_spi1(uint8_t reg, uint8_t* data, uint8_t length) {
    reg |= SPI_READ_WRITE_BIT;

    HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&Spi1Handle, &reg, 1, SPI1_TIMEOUT) != HAL_OK)
        return 0;
    if (HAL_SPI_Receive(&Spi1Handle, data, length, SPI1_TIMEOUT) != HAL_OK)
        return 0;
    HAL_GPIO_WritePin(SPI1_GPIO_PORT, SPI1_SS_PIN, GPIO_PIN_SET);

    return 1;
}