/**
 * \file uart1.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief UART1 functions
 */

#include "uart1.h"

// C lib
#include <stdio.h>
#include <string.h>

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

  if (HAL_OK != HAL_UART_Init(&UartHandle1))
    return NOK;

  // UART TX FreeRTOS queue
  // uart1TxQueue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  // if (uart1TxQueue == 0) {
  //   return NOK;
  // }

  // UART RX FreeRTOS queue
  uart1RxQueue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  if (0 == uart1RxQueue)
    return NOK;

  // Enable UART RX interrupt
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(USART1_IRQ, 10, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQ);
  __HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_ERR);

  isInit = true;
  hasOverrun = false;

  return OK;
} // uart1_init

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
} // uart1_deInit

bool uart1_is_init(void)
{
  return isInit;
} // uart1_is_init

int uart1_write(uint8_t* buf, uint32_t len)
{
  uint8_t res = HAL_OK;

  res = HAL_UART_Transmit(&UartHandle1, buf, (uint16_t)len, USART1_TIMEOUT);

  if (HAL_OK == res)
   return OK;
  else if (HAL_ERROR == res)
    printf("uart1_write: HAL_ERROR\r\n");
  else if (HAL_BUSY == res)
    printf("uart1_write: HAL_BUSY\r\n");
  else if (HAL_TIMEOUT == res)
    printf("uart1_write: HAL_TIMEOUT\r\n");
  else
    printf("uart1_write: unknown problem\r\n");

  return NOK;
} // uart1_write

int uart1_read(uint8_t* buf, uint32_t len)
{
  uint8_t res = HAL_OK;
  res = HAL_UART_Receive(&UartHandle1, buf, (uint16_t)len, USART1_TIMEOUT);

  if (HAL_OK == res)
   return OK;
  else if (HAL_ERROR == res)
    printf("uart1_read: HAL_ERROR\r\n");
  else if (HAL_BUSY == res)
    printf("uart1_read: HAL_BUSY\r\n");
  else if (HAL_TIMEOUT == res)
    printf("uart1_read: HAL_TIMEOUT\r\n");
  else
    printf("uart1_read: unknown problem\r\n");

  return NOK;
} // uart1_read

void uart1_send_data(uint8_t* data, uint32_t size)
{
	uint32_t i = 0;
	for (i = 0; i < size; i++)
  {
	    while (!(USART1->SR & USART_FLAG_TXE));
      USART1->DR = (data[i] & (uint8_t)0xFF);
	 }
} // uart1_send_data

uint32_t uart1_enque_data(uint8_t* _data, uint32_t _len)
{
  uint32_t i = 0;

  // Check that the queue is created
  //if (uart1TxQueue == 0) return 0;
  
  for (i = 0; i < _len; i++)
  {
    if (errQUEUE_FULL == xQueueSend(uart1TxQueue, &_data[i], (TickType_t)1))
    {
      break;
    }
  }
  return i; // Return the number of data written to the queue
} // uart1_enque_data

int uart1_deque_byte(uint8_t* _data, uint32_t _timeToWait)
{
  if (pdTRUE == xQueueReceive(uart1RxQueue, (void*)_data, (TickType_t)_timeToWait))
    return OK;
  else
  {
    *_data = 0;
    return NOK;
  }
} // uart1_enque_data

uint32_t uart1_deque_data(uint8_t* _data, uint32_t _len, uint32_t _timeToWait)
{
  uint32_t i = 0;

  for (i = 0; i < _len; i++)
  {
    if (pdFALSE == xQueueReceive(uart1RxQueue, (void*)&_data[i], (TickType_t)_timeToWait))
      break;
  }
  return i;
} // uart1_deque_data

void uart1_task(void* _params)
{
  uint8_t data = 0;
  uint8_t res = 0;
  bool keepOn = false;

  char msg[] = "Hello World!\r\n";

  if (_params != 0) { }

  while (1)
  {
    do {
      res = HAL_UART_Receive(&UartHandle1, &data, 1, 0);
      if (HAL_OK == res)
      {
        printf("uart1_task: c=0x%x\r\n", data);
        if (pdTRUE == xQueueSend(uart1RxQueue, (const void*)&data, (TickType_t)0))
          keepOn = true;
        else
        {
          printf("uart1_task: failed to enque rx data\r\n");
          keepOn = false;
        }
      }
      else
        keepOn = false;
    } while (keepOn);

    res = HAL_UART_Transmit(&UartHandle1, (uint8_t*)msg, (uint16_t)strlen(msg), USART1_TIMEOUT);
    if (HAL_OK == res)
    {
      //printf("uart1_write: HAL_OK\r\n");
    }
    else if (HAL_ERROR == res)
      printf("uart1_write: HAL_ERROR\r\n");
    else if (HAL_BUSY == res)
      printf("uart1_write: HAL_BUSY\r\n");
    else if (HAL_TIMEOUT == res)
      printf("uart1_write: HAL_TIMEOUT\r\n");
    
    vTaskDelay(10/portTICK_RATE_MS); // 100 Hz
  }

  vTaskDelete(NULL);
} // uart1_task

bool uart1_has_overrun(void)
{
  bool result = hasOverrun;
  hasOverrun = false;
  return result;
} // uart1_has_overrun

void __attribute__((used)) USART1_IRQHandler(void)
{
  uint8_t rxByte = 0;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  if (USART1->SR & USART_SR_RXNE)
  {
    rxByte = (uint8_t)(USART1->DR & (uint8_t)0x00FF);
    xQueueSendFromISR(uart1RxQueue, (const void*)&rxByte, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
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
} // USART1_IRQHandler