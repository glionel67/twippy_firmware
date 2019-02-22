#pragma once

#include <stdint.h>

int i2c2_init(void);
void i2c2_deInit(void);

int i2c2_write_byte(uint8_t _addr, uint8_t _data);
int i2c2_read_byte(uint8_t _addr, uint8_t* _data);
int i2c2_write_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len);
int i2c2_read_bytes(uint8_t _addr, uint8_t* _data, uint8_t _len);