/**
 * \file spi.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief SPI1 functions
 */

#pragma once

#include <stdint.h>

#define SPI_BAUDRATE_45MHZ SPI_BAUDRATEPRESCALER_2 // 45 MHz
#define SPI_BAUDRATE_22MHZ SPI_BAUDRATEPRESCALER_4 // 22.5 MHz
#define SPI_BAUDRATE_11MHZ SPI_BAUDRATEPRESCALER_8 // 11.2 MHz
#define SPI_BAUDRATE_5MHZ SPI_BAUDRATEPRESCALER_16 // 5.6 MHz
#define SPI_BAUDRATE_2MHZ SPI_BAUDRATEPRESCALER_32 // 2.8 MHz
#define SPI_BAUDRATE_1MHZ SPI_BAUDRATEPRESCALER_64 // 1.4 MHz
#define SPI_BAUDRATE_0_7MHZ SPI_BAUDRATEPRESCALER_128 // 0.7 MHz

#define SPI_READ_WRITE_BIT 0x80

uint8_t spi1_init(void);
void spi1_deInit(void);
uint8_t spi1_set_speed(uint32_t baudratePrescaler);

uint8_t write_byte_spi1(uint8_t reg, uint8_t data);
uint8_t read_byte_spi1(uint8_t reg, uint8_t* data);
uint8_t write_bytes_spi1(uint8_t reg, uint8_t* data, uint8_t length);
uint8_t read_bytes_spi1(uint8_t reg, uint8_t* data, uint8_t length);