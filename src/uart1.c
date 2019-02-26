/**
 * \file uart1.c
 * \brief GPS parser
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 */

#include "uart1.h"

#include <stdio.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

UART_HandleTypeDef UartHandle1;

static xQueueHandle uart1RxQueue = 0;
static xQueueHandle uart1TxQueue = 0;

static bool isInit = false;
static bool hasOverrun = false;

int uart1_init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();

  UartHandle1.Instance        = USART1;
  UartHandle1.Init.BaudRate   = USART1_BAUDRATE;
  UartHandle1.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle1.Init.StopBits   = UART_STOPBITS_1;
  UartHandle1.Init.Parity     = UART_PARITY_NONE;
  UartHandle1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle1.Init.Mode       = UART_MODE_TX_RX;
  UartHandle1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle1) != HAL_OK) {
    return NOK;
  }

  // UART TX FreeRTOS queue
  // uart1TxQueue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  // if (uart1TxQueue == 0) {
  //   return NOK;
  // }

  // UART RX FreeRTOS queue
  uart1RxQueue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  if (uart1RxQueue == 0) {
    return NOK;
  }

  // Enable UART RX interrupt
  // HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  // HAL_NVIC_SetPriority(USART1_IRQ, 10, 0);
  // HAL_NVIC_EnableIRQ(USART1_IRQ);
  // __HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_RXNE);
  // __HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_ERR);

  isInit = true;
  hasOverrun = false;

  return OK;
}

void uart1_deInit(void)
{
	__HAL_RCC_USART1_FORCE_RESET();
	__HAL_RCC_USART1_RELEASE_RESET();
  HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_TX_PIN);
  HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_RX_PIN);
  HAL_NVIC_DisableIRQ(USART1_IRQ);
  __HAL_UART_DISABLE_IT(&UartHandle1, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(&UartHandle1, UART_IT_ERR);

  isInit = false;
}

bool uart1_is_init(void)
{
  return isInit;
}

int uart1_write(uint8_t* buf, uint32_t len)
{
  uint8_t res = HAL_UART_Transmit(&UartHandle1, buf, len, USART1_TIMEOUT);
  if (res == HAL_OK)
   return OK;
  else
    return NOK;
}

int uart1_read(uint8_t* buf, uint32_t len)
{
  uint8_t res = HAL_UART_Receive(&UartHandle1, buf, len, USART1_TIMEOUT);
  if (res == HAL_OK)
	 return OK;
  else
    return NOK;
}

void uart1_send_data(uint8_t* data, uint32_t size)
{
	uint32_t i = 0;
	for (i=0;i<size;i++) {
	    while (!(USART1->SR & USART_FLAG_TXE));
      USART1->DR = (data[i] & (uint8_t)0xFF);
	 }
}

uint32_t uart1_enque_data(uint8_t* _data, uint32_t _len)
{
  uint32_t i = 0;

  // Check that the queue is created
  //if (uart1TxQueue == 0) return 0;
  
  for (i=0;i<_len;i++) {
    if (xQueueSend(uart1TxQueue, &_data[i], (TickType_t)1) == errQUEUE_FULL) {
      break;;
    }
  }
  return i; // Return the number of data written to the queue
}

int uart1_deque_byte(uint8_t* _data, uint32_t _timeToWait)
{
  if (xQueueReceive(uart1RxQueue, (void*)_data, (TickType_t)_timeToWait) == pdTRUE)
    return OK;
  else {
    *_data = 0;
    return NOK;
  }
}

uint32_t uart1_deque_data(uint8_t* _data, uint32_t _len, uint32_t _timeToWait)
{
  uint32_t i = 0;
  for (i=0;i<_len;i++) {
    if (xQueueReceive(uart1RxQueue, (void*)&_data[i], (TickType_t)_timeToWait) == pdFALSE)
      break;
  }
  return i;
}

void uart1_task(void* _params)
{
  uint8_t data = 0;
  uint8_t res = 0;
  bool keepOn = false;

  if (_params != 0) { }

  while (1) {
    do {
      res = HAL_UART_Receive(&UartHandle1, &data, 1, 0);
      if (res == HAL_OK) {
        printf("uart1_task: c=0x%x\r\n", data);
        if (xQueueSend(uart1RxQueue, (const void*)&data, (TickType_t)0) == pdTRUE)
          keepOn = true;
        else {
          printf("uart1_task: failed to enque rx data\r\n");
          keepOn = false;
        }
      }
      else
        keepOn = false;
    } while (keepOn);
    
    vTaskDelay(10/portTICK_RATE_MS); // 100 Hz
  }

  vTaskDelete(NULL);
}

bool uart1_has_overrun(void)
{
  bool result = hasOverrun;
  hasOverrun = false;
  return result;
}

void __attribute__((used)) USART1_IRQHandler(void)
{
  uint8_t rxByte = 0;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  if (USART1->SR & USART_SR_RXNE) {
    rxByte = (uint8_t)(USART1->DR & (uint8_t)0x00FF);
    //printf("uart1_int: 0x%x\r\n", rxByte);
    xQueueSendFromISR(uart1RxQueue, (const void*)&rxByte, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else {
    printf("uart1_int: ERROR\r\n");
    rxByte = (uint8_t)(USART1->DR & (uint8_t)0x00FF); // Flush data register
    /** if we get here, the error is most likely caused by an overrun!
     * - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun error)
     * - and IDLE (Idle line detected) pending bits are cleared by software sequence:
     * - reading USART_SR register followed reading the USART_DR register.
     */
    //asm volatile ("" : "=m" (UART1_TYPE->SR) : "r" (UART1_TYPE->SR)); // force non-optimizable reads
    //asm volatile ("" : "=m" (UART1_TYPE->DR) : "r" (UART1_TYPE->DR)); // of these two registers
    hasOverrun = true;
  }
}
