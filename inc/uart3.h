#pragma once

// C lib
#include <stdint.h>
#include <stdbool.h>

#include "main.h"

extern UART_HandleTypeDef UartHandle3;

#define UART3_BUSY (HAL_UART_GetState(&UartHandle3)==(HAL_UART_STATE_BUSY))
#define UART3_WRITE(__data__, __len__) HAL_UART_Transmit(&UartHandle3, __data__, __len__, USART3_TIMEOUT)
#define UART3_READ(__data__, __len__) HAL_UART_Receive(&UartHandle3, __data__, __len__, USART3_TIMEOUT)


#define UART3_QUEUE_SIZE    128

int uart3_init(void);
void uart3_deInit(void);
bool uart3_is_init(void);
int uart3_write(uint8_t* buf, uint32_t len);
int uart3_read(uint8_t* buf, uint32_t len);
void uart3_send_data(uint8_t* data, uint32_t size);
uint32_t uart3_enque_data(uint8_t* data, uint32_t size);
int uart3_deque_byte(uint8_t* _data, uint32_t _timeToWait);
uint32_t uart3_deque_data(uint8_t* _data, uint32_t _len, uint32_t _timeToWait);
void uart3_task(void* _params);
bool uart3_has_overrun(void);