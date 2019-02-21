#pragma once

#include <stdint.h>
#include "main.h"

extern UART_HandleTypeDef UartHandle3;

#define UART3_BUSY (HAL_UART_GetState(&UartHandle3)==(HAL_UART_STATE_BUSY))
#define UART3_WRITE(__data__, __len__) HAL_UART_Transmit(&UartHandle3, __data__, __len__, USART3_TIMEOUT)
#define UART3_READ(__data__, __len__) HAL_UART_Receive(&UartHandle3, __data__, __len__, USART3_TIMEOUT)


#define UART3_QUEUE_SIZE    128

int uart3_init(void);
void uart3_deInit(void);
int uart3_write(uint8_t* buf, uint8_t len);
int uart3_read(uint8_t* buf, uint8_t len);
void uart3_send_data(uint8_t* data, uint8_t size);
uint8_t uart3_enque_data(uint8_t* data, uint8_t size);
void uart3_task(void* _params);