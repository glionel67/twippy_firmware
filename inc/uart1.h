#pragma once

#include <stdint.h>
#include "main.h"

extern UART_HandleTypeDef UartHandle1;

#define UART1_BUSY (HAL_UART_GetState(&UartHandle1)==(HAL_UART_STATE_BUSY))
#define UART1_WRITE(__data__, __len__) HAL_UART_Transmit(&UartHandle1, __data__, __len__, USART1_TIMEOUT)
#define UART1_READ(__data__, __len__) HAL_UART_Receive(&UartHandle1, __data__, __len__, USART1_TIMEOUT)


#define UART1_QUEUE_SIZE    128

int uart1_init(void);
void uart1_deInit(void);
int uart1_write(uint8_t* buf, uint8_t len);
int uart1_read(uint8_t* buf, uint8_t len);
void uart1_send_data(uint8_t* data, uint8_t size);
uint8_t uart1_enque_data(uint8_t* data, uint8_t size);
void uart1_task(void* _params);