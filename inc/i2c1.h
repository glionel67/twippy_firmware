/**
 * \file i2c1.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief I2C1 functions
 */

#pragma once

#include <stdint.h>

int i2c1_init(void);
void i2c1_deInit(void);

int i2c1_write_byte(uint8_t _addr, uint8_t _data);
int i2c1_read_byte(uint8_t _addr, uint8_t* _data);
int i2c1_write_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len);
int i2c1_read_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len);