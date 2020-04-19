#include "i2c1.h"
#include "main.h"

I2C_HandleTypeDef I2cHandle1;

int i2c1_init(void)
{
    // Enable I2C1 clock
    __HAL_RCC_I2C1_CLK_ENABLE();

    // Configure the I2C peripheral
    I2cHandle1.Instance             = I2C1;
    I2cHandle1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle1.Init.ClockSpeed      = I2C1_CLOCK_SPEED;
    I2cHandle1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle1.Init.DutyCycle       = I2C_DUTYCYCLE_16_9; // or I2C_DUTYCYCLE_2 ?
    I2cHandle1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    I2cHandle1.Init.OwnAddress1     = I2C1_OWN_ADDRESS;
    I2cHandle1.Init.OwnAddress2     = 0xFE;

    if (HAL_OK != HAL_I2C_Init(&I2cHandle1))
        return NOK; // Initialization Error


    // NVIC for I2C1
    //HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1, 0);
    //HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    //HAL_NVIC_SetPriority(I2C1_EV_IRQn, 2, 0);
    //HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

    return OK;
}

void i2c1_deInit(void)
{
    // Reset peripherals
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    // Disable peripherals and GPIO Clocks
    HAL_GPIO_DeInit(I2C1_GPIO_PORT, I2C1_SCL_PIN);
    HAL_GPIO_DeInit(I2C1_GPIO_PORT, I2C1_SDA_PIN);
  
    // Disable the NVIC for I2C
    //HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    //HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
}

int i2c1_write_byte(uint8_t _addr, uint8_t _data)
{
    // Start the transmission process
    uint8_t res = HAL_I2C_Master_Transmit(&I2cHandle1, (uint16_t)_addr, &_data, 
                1, I2C1_TIMEOUT);
    //uint8_t res = HAL_I2C_Master_Transmit_IT(&I2cHandle1, (uint16_t)_addr, 
    //        &_data, 1, I2C1_TIMEOUT);
    if (HAL_OK != res)
        return NOK;
    
    // Wait for the end of the transfer
    //while (HAL_I2C_GetState(&I2cHandle1) != HAL_I2C_STATE_READY);

    return OK;
}

int i2c1_read_byte(uint8_t _addr, uint8_t* _data)
{
    // Put I2C peripheral in reception process
    uint8_t res = HAL_I2C_Master_Receive(&I2cHandle1, (uint16_t)_addr, _data,
                1, I2C1_TIMEOUT);
    //uint8_t res = HAL_I2C_Master_Receive_IT(&I2cHandle1, (uint16_t)_addr, 
    //            _data, 1, I2C1_TIMEOUT);
    if (HAL_OK != res)
        return NOK;

    //while (HAL_I2C_GetError(&I2cHandle1) == HAL_I2C_ERROR_AF);

    return OK;
}

int i2c1_write_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len)
{
    // Start the transmission process
    uint8_t res = HAL_I2C_Master_Transmit(&I2cHandle1, (uint16_t)_addr, _data, 
                _len, I2C1_TIMEOUT);
    //uint8_t res = HAL_I2C_Master_Transmit_IT(&I2cHandle1, (uint16_t)_addr, 
    //        _data, _len, I2C1_TIMEOUT);
    if (HAL_OK != res)
        return NOK;
    
    // Wait for the end of the transfer
    //while (HAL_I2C_GetState(&I2cHandle1) != HAL_I2C_STATE_READY);

    return OK;
}

int i2c1_read_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len)
{
    // Put I2C peripheral in reception process
    uint8_t res = HAL_I2C_Master_Receive(&I2cHandle1, (uint16_t)_addr, _data,
                _len, I2C1_TIMEOUT);
    //uint8_t res = HAL_I2C_Master_Receive_IT(&I2cHandle1, (uint16_t)_addr, 
    //            _data, _len, I2C1_TIMEOUT);
    if (HAL_OK != res)
        return NOK;

    //while (HAL_I2C_GetError(&I2cHandle1) == HAL_I2C_ERROR_AF);

    return OK;
}