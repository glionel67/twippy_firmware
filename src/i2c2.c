#include "i2c2.h"
#include "main.h"

I2C_HandleTypeDef I2cHandle2;

int i2c2_init(void)
{
    // Enable I2C2 clock
    __HAL_RCC_I2C2_CLK_ENABLE();

    // Configure the I2C peripheral
    I2cHandle2.Instance             = I2C2;
    I2cHandle2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle2.Init.ClockSpeed      = I2C2_CLOCK_SPEED;
    I2cHandle2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle2.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
    I2cHandle2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    I2cHandle2.Init.OwnAddress1     = I2C2_OWN_ADDRESS;
    I2cHandle2.Init.OwnAddress2     = 0xFE;

    if (HAL_I2C_Init(&I2cHandle2) != HAL_OK) { // Initialization Error
        return NOK; 
    }

    // NVIC for I2C2
    //HAL_NVIC_SetPriority(I2C2_ER_IRQn, 1, 0);
    //HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    //HAL_NVIC_SetPriority(I2C2_EV_IRQn, 2, 0);
    //HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);

    return OK;
}

void i2c2_deInit(void)
{
    // Reset peripherals
    __HAL_RCC_I2C2_FORCE_RESET();
    __HAL_RCC_I2C2_RELEASE_RESET();

    // Disable peripherals and GPIO Clocks
    HAL_GPIO_DeInit(I2C2_GPIO_PORT, I2C2_SCL_PIN);
    HAL_GPIO_DeInit(I2C2_GPIO_PORT, I2C2_SDA_PIN);
  
    // Disable the NVIC for I2C
    //HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
    //HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
}

int i2c2_write_byte(uint8_t _addr, uint8_t _data)
{
    // Start the transmission process
    uint8_t res = HAL_I2C_Master_Transmit(&I2cHandle2, (uint16_t)_addr, &_data, 
                1, I2C2_TIMEOUT);
    //uint8_t res = HAL_I2C_Master_Transmit_IT(&I2cHandle2, (uint16_t)_addr, 
    //        &_data, 1, I2C2_TIMEOUT);
    if (res != HAL_OK)
        return NOK;
    
    // Wait for the end of the transfer
    //while (HAL_I2C_GetState(&I2cHandle2) != HAL_I2C_STATE_READY);

    return OK;
}

int i2c2_read_byte(uint8_t _addr, uint8_t* _data)
{
    // Put I2C peripheral in reception process
    uint8_t res = HAL_I2C_Master_Receive(&I2cHandle2, (uint16_t)_addr, _data,
                1, I2C2_TIMEOUT);
    //uint8_t res = HAL_I2C_Master_Receive_IT(&I2cHandle2, (uint16_t)_addr, 
    //            _data, 1, I2C2_TIMEOUT);
    if (res != HAL_OK)
        return NOK;

    //while (HAL_I2C_GetError(&I2cHandle2) == HAL_I2C_ERROR_AF);

    return OK;
}

int i2c2_write_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len)
{
    // Start the transmission process
    uint8_t res = HAL_I2C_Master_Transmit(&I2cHandle2, (uint16_t)_addr, _data, 
                _len, I2C2_TIMEOUT);
    //uint8_t res = HAL_I2C_Master_Transmit_IT(&I2cHandle2, (uint16_t)_addr, 
    //        _data, _len, I2C2_TIMEOUT);
    if (res != HAL_OK)
        return NOK;
    
    // Wait for the end of the transfer
    //while (HAL_I2C_GetState(&I2cHandle2) != HAL_I2C_STATE_READY);

    return OK;
}

int i2c2_read_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len)
{
    // Put I2C peripheral in reception process
    uint8_t res = HAL_I2C_Master_Receive(&I2cHandle2, (uint16_t)_addr, _data,
                _len, I2C2_TIMEOUT);
    //uint8_t res = HAL_I2C_Master_Receive_IT(&I2cHandle2, (uint16_t)_addr, 
    //            _data, _len, I2C2_TIMEOUT);
    if (res != HAL_OK)
        return NOK;

    //while (HAL_I2C_GetError(&I2cHandle2) == HAL_I2C_ERROR_AF);

    return OK;
}