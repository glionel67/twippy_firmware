#pragma once

#include <stdint.h>
#include "main.h"

#define UART2_QUEUE_SIZE    128

extern UART_HandleTypeDef UartHandle2;

#define UART2_BUSY (HAL_UART_GetState(&UartHandle2)==(HAL_UART_STATE_BUSY))
#define UART2_WRITE(__data__, __len__) HAL_UART_Transmit(&UartHandle2, __data__, __len__, USART2_TIMEOUT)
#define UART2_READ(__data__, __len__) HAL_UART_Receive(&UartHandle2, __data__, __len__, USART2_TIMEOUT)


int uart2_init(void);
void uart2_deInit(void);
int uart2_write(uint8_t* buf, uint32_t len);
int uart2_read(uint8_t* buf, uint32_t len);
void uart2_send_data(uint8_t* data, uint32_t size);
uint32_t uart2_enque_data(uint8_t* data, uint32_t size);
void uart2_task(void* _params);