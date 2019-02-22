#pragma once

// C lib
#include <stdint.h>
#include <stdbool.h>


#include "main.h"

extern UART_HandleTypeDef UartHandle1;

#define UART1_BUSY (HAL_UART_GetState(&UartHandle1)==(HAL_UART_STATE_BUSY))
#define UART1_WRITE(__data__, __len__) HAL_UART_Transmit(&UartHandle1, __data__, __len__, USART1_TIMEOUT)
#define UART1_READ(__data__, __len__) HAL_UART_Receive(&UartHandle1, __data__, __len__, USART1_TIMEOUT)


#define UART1_QUEUE_SIZE    512

int uart1_init(void);
void uart1_deInit(void);
bool uart1_is_init(void);
int uart1_write(uint8_t* buf, uint32_t len);
int uart1_read(uint8_t* buf, uint32_t len);
void uart1_send_data(uint8_t* data, uint32_t size);
uint32_t uart1_enque_data(uint8_t* _data, uint32_t _len);
int uart1_deque_byte(uint8_t* _data, uint32_t _timeToWait);
uint32_t uart1_deque_data(uint8_t* _data, uint32_t _len, uint32_t _timeToWait);
void uart1_task(void* _params);
bool uart1_has_overrun(void);