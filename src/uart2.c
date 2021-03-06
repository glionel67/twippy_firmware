/**
 * \file uart2.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief UART2 functions
 */

#include "uart2.h"

// C lib
#include <stdio.h>
//#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

UART_HandleTypeDef UartHandle2;

static xQueueHandle uart2RxQueue = 0;
static xQueueHandle uart2TxQueue = 0;

int uart2_init(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

  UartHandle2.Instance        = USART2;
  UartHandle2.Init.BaudRate   = USART2_BAUDRATE;
  UartHandle2.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle2.Init.StopBits   = UART_STOPBITS_1;
  UartHandle2.Init.Parity     = UART_PARITY_NONE;
  UartHandle2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle2.Init.Mode       = UART_MODE_TX_RX;
  UartHandle2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_OK != HAL_UART_Init(&UartHandle2))
    return NOK;

  uart2TxQueue = xQueueCreate(UART2_QUEUE_SIZE, sizeof(uint8_t));
  if (0 == uart2TxQueue)
    return NOK;

  uart2RxQueue = xQueueCreate(UART2_QUEUE_SIZE, sizeof(uint8_t));
  if (0 == uart2RxQueue)
    return NOK;

  // HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  // HAL_NVIC_SetPriority(USART2_IRQ, 11, 0);
  // HAL_NVIC_EnableIRQ(USART2_IRQn);
  // __HAL_UART_ENABLE_IT(&UartHandle2, UART_IT_RXNE);

  return OK;
} // uart2_init

void uart2_deInit(void)
{
  __HAL_RCC_USART2_FORCE_RESET();
  __HAL_RCC_USART2_RELEASE_RESET();
  HAL_GPIO_DeInit(USART2_GPIO_PORT, USART2_TX_PIN);
  HAL_GPIO_DeInit(USART2_GPIO_PORT, USART2_RX_PIN);
  __HAL_UART_DISABLE_IT(&UartHandle2, UART_IT_RXNE);
}

int uart2_write(uint8_t* buf, uint32_t len)
{
  uint8_t res = HAL_UART_Transmit(&UartHandle2, buf, (uint16_t)len, USART2_TIMEOUT);

  if (HAL_OK == res)
   return OK;
  else if (HAL_ERROR == res)
    printf("uart2_write: HAL_ERROR\r\n");
  else if (HAL_BUSY == res)
    printf("uart2_write: HAL_BUSY\r\n");
  else if (HAL_TIMEOUT == res)
    printf("uart2_write: HAL_TIMEOUT\r\n");
  else
    printf("uart2_write: unknown problem\r\n");

  return NOK;
} // uart2_write

int uart2_read(uint8_t* buf, uint32_t len)
{
  uint8_t res = HAL_UART_Receive(&UartHandle2, buf, (uint16_t)len, USART2_TIMEOUT);

  if (HAL_OK == res)
   return OK;
  else if (HAL_ERROR == res)
    printf("uart2_read: HAL_ERROR\r\n");
  else if (HAL_BUSY == res)
    printf("uart2_read: HAL_BUSY\r\n");
  else if (HAL_TIMEOUT == res)
    printf("uart2_read: HAL_TIMEOUT\r\n");
  else
    printf("uart2_read: unknown problem\r\n");

  return NOK;
} // uart2_read

void uart2_send_data(uint8_t* data, uint32_t size)
{
  uint32_t i = 0;

  for (i = 0; i < size; i++)
  {
      while (!(USART2->SR & USART_FLAG_TXE));
      USART2->DR = (data[i] & 0x00FF);
   }
}

uint32_t uart2_enque_data(uint8_t* data, uint32_t size)
{
  uint32_t i = 0;
  for (i = 0; i < size; i++)
  {
    if (errQUEUE_FULL == xQueueSend(uart2TxQueue, data+i, 10))
      return i;
  }
  return 0;
}

void uart2_task(void* _params)
{
  uint8_t data = 0;
  uint8_t rxIdx = 0;
  uint8_t rxBuffer[UART2_QUEUE_SIZE] = {0, };

  if (_params != 0) { }

  while (1)
  {
    if (pdTRUE == xQueueReceive(uart2TxQueue, &data, 0))
    {
      HAL_UART_Transmit(&UartHandle2, &data, 1, USART2_TIMEOUT);
    }
    if (pdTRUE == xQueueReceive(uart2RxQueue, rxBuffer+rxIdx, 0))
    {
      // Check end of transmission character: \n
      if (rxBuffer[rxIdx] == '\n')
      {
        // TODO

        // Reset idx
        rxIdx = 0;
      }
      else
      {
        rxIdx++;
      }
    }
    else
    {
      vTaskDelay(10/portTICK_RATE_MS);
    }
  }

  vTaskDelete(NULL);
}

void __attribute__((used)) USART2_IRQHandler(void)
{
  uint8_t rxByte = 0;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  if (__HAL_UART_GET_FLAG(&UartHandle2, UART_FLAG_RXNE))
  {
    rxByte = (uint8_t)(USART2->DR & (uint8_t)0x00FF);
    xQueueSendFromISR(uart2RxQueue, &rxByte, &xHigherPriorityTaskWoken);
  }
}
