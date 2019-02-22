#include "uart3.h"

#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

UART_HandleTypeDef UartHandle3;

static xQueueHandle uart3RxQueue = 0;
static xQueueHandle uart3TxQueue = 0;

int uart3_init(void) {
  __HAL_RCC_USART3_CLK_ENABLE();

  UartHandle3.Instance        = USART3;
  UartHandle3.Init.BaudRate   = USART3_BAUDRATE;
  UartHandle3.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle3.Init.StopBits   = UART_STOPBITS_1;
  UartHandle3.Init.Parity     = UART_PARITY_NONE;
  UartHandle3.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle3.Init.Mode       = UART_MODE_TX_RX;
  UartHandle3.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle3) != HAL_OK) {
    return NOK;
  }

  uart3TxQueue = xQueueCreate(UART3_QUEUE_SIZE, sizeof(uint8_t));
  if (uart3TxQueue == 0) {
    return NOK;
  }

  uart3RxQueue = xQueueCreate(UART3_QUEUE_SIZE, sizeof(uint8_t));
  if (uart3RxQueue == 0) {
    return NOK;
  }

  HAL_NVIC_SetPriority(USART3_IRQ, 10, 0);
  //HAL_NVIC_EnableIRQ(USART3_IRQ);
  __HAL_UART_ENABLE_IT(&UartHandle3, UART_IT_RXNE);

  return OK;
}

void uart3_deInit(void) {
    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();
  HAL_GPIO_DeInit(USART3_GPIO_PORT, USART3_TX_PIN);
  HAL_GPIO_DeInit(USART3_GPIO_PORT, USART3_RX_PIN);
  HAL_NVIC_DisableIRQ(USART3_IRQ);
  __HAL_UART_DISABLE_IT(&UartHandle3, UART_IT_RXNE);
}

int uart3_write(uint8_t* buf, uint32_t len) {
  uint8_t res = HAL_UART_Transmit(&UartHandle3, buf, len, USART3_TIMEOUT);
  if (res == HAL_OK)
   return OK;
  else
    return NOK;
}

int uart3_read(uint8_t* buf, uint32_t len) {
  uint8_t res = HAL_UART_Receive(&UartHandle3, buf, len, USART3_TIMEOUT);
  if (res == HAL_OK)
     return OK;
  else
    return NOK;
}

void uart3_send_data(uint8_t* data, uint32_t size) {
    uint32_t i = 0;
    for (i=0;i<size;i++) {
        while (!(USART3->SR & USART_FLAG_TXE));
        USART3->DR = (data[i] & 0x00FF);
     }
}

uint32_t uart3_enque_data(uint8_t* data, uint32_t size) {
  uint8_t i = 0;
  for (i=0;i<size;i++) {
    if (xQueueSend(uart3TxQueue, data+i, 10) == errQUEUE_FULL) {
      return i;
    }
  }
  return 0;
}

void uart3_task(void* _params) {
  uint8_t data = 0;
  uint8_t rxIdx = 0;
  uint8_t rxBuffer[UART3_QUEUE_SIZE] = {0, };
  uint8_t res = 0;

  if (_params != 0) { }

  while (1) {
    if (pdTRUE == xQueueReceive(uart3TxQueue, &data, 0)) {
      res = HAL_UART_Transmit(&UartHandle3, &data, 1, USART3_TIMEOUT);
      if (res != HAL_OK) {

      }
    }
    char msg[] = "uart3_task TX test\r\n";
      res = HAL_UART_Transmit(&UartHandle3, (uint8_t*)msg, strlen(msg), USART3_TIMEOUT);
      if (res != HAL_OK) {

      }
    if (pdTRUE == xQueueReceive(uart3RxQueue, rxBuffer+rxIdx, 0)) {
      char msg[] = "uart3_task received char\r\n";
      print_msg((uint8_t*)msg, strlen(msg));
      print_msg(&rxBuffer[rxIdx], 1);

      // Check end of transmission character: \n
      if (rxBuffer[rxIdx] == '\n') {
        // TODO

        // Reset idx
        rxIdx = 0;
      }
      else {
        rxIdx++;
      }
    }
    else {
      vTaskDelay(10/portTICK_RATE_MS);
    }
  }

  vTaskDelete(NULL);
}

void __attribute__((used)) USART3_IRQHandler(void) {
  uint8_t rxByte = 0;
  //HAL_UART_IRQHandler(&UartHandle3);
  if (__HAL_UART_GET_FLAG(&UartHandle3, UART_FLAG_RXNE)) {
    rxByte = (uint8_t)(UartHandle3.Instance->DR & (uint8_t)0x00FF);
    __HAL_UART_CLEAR_FLAG(&UartHandle3, UART_FLAG_RXNE);
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(uart3RxQueue, &rxByte, &xHigherPriorityTaskWoken);
  }
}
