#pragma once

#include <stdint.h>
#include "main.h"

extern UART_HandleTypeDef UartHandle1;
extern UART_HandleTypeDef UartHandle2;

#define UART1_BUSY 		(HAL_UART_GetState(&UartHandle1)==(HAL_UART_STATE_BUSY))
#define UART2_BUSY 		(HAL_UART_GetState(&UartHandle2)==(HAL_UART_STATE_BUSY))

int uart1_init(void);
void uart1_deInit(void);
int uart1_write(uint8_t* buf, uint8_t len);
int uart1_write_it(uint8_t* buf, uint8_t len);
int uart1_read(uint8_t* buf, uint8_t len);
void uart1_send_data(uint32_t size, uint8_t* data);

int uart2_init(void);
void uart2_deInit(void);
int uart2_write(uint8_t* buf, uint8_t len);
int uart2_write_it(uint8_t* buf, uint8_t len);
int uart2_read(uint8_t* buf, uint8_t len);
int uart2_read_it(uint8_t* buf, uint8_t len);
void uart2_send_data(uint32_t size, uint8_t* data);
