/**
 * \file i2c2.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief I2C2 functions
 */

#pragma once

#include <stdint.h>

int i2c2_init(void);
void i2c2_deInit(void);

int i2c2_write_byte(uint8_t _addr, uint8_t _data);
int i2c2_read_byte(uint8_t _addr, uint8_t* _data);
int i2c2_write_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len);
int i2c2_read_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len);