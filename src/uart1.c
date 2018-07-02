#include "uart1.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

UART_HandleTypeDef UartHandle1;

static uint8_t rxByte = 0;
static xQueueHandle uart1RxQueue = 0;
static xQueueHandle uart1TxQueue = 0;

int uart1_init(void) {
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
    return -1;
  }

  uart1TxQueue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  if (uart1TxQueue == 0) {
    return -1;
  }

  uart1RxQueue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  if (uart1RxQueue == 0) {
    return -1;
  }

  HAL_NVIC_SetPriority(USART1_IRQ, 10, 0);
  //HAL_NVIC_EnableIRQ(USART1_IRQ);
  __HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_RXNE);

  return 0;
}

void uart1_deInit(void) {
	__HAL_RCC_USART1_FORCE_RESET();
	__HAL_RCC_USART1_RELEASE_RESET();
  HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_TX_PIN);
  HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_RX_PIN);
  HAL_NVIC_DisableIRQ(USART1_IRQ);
  __HAL_UART_DISABLE_IT(&UartHandle1, UART_IT_RXNE);
}

int uart1_write(uint8_t* buf, uint8_t len) {
	return HAL_UART_Transmit(&UartHandle1, buf, len, USART1_TIMEOUT);
}

int uart1_read(uint8_t* buf, uint8_t len) {
	return HAL_UART_Receive(&UartHandle1, buf, len, USART1_TIMEOUT);
}

void uart1_send_data(uint8_t* data, uint8_t size) {
	uint8_t i = 0;
	for (i=0;i<size;i++) {
	    while (!(USART1->SR & USART_FLAG_TXE));
	    USART1->DR = (data[i] & 0x00FF);
	 }
}

uint8_t uart1_enque_data(uint8_t* data, uint8_t size) {
  uint8_t i = 0;
  for (i=0;i<size;i++) {
    if (xQueueSend(uart1TxQueue, data+i, 10) == errQUEUE_FULL) {
      return i;
    }
  }
  return 0;
}

void uart1_task(void* _params) {
  uint8_t data = 0;
  uint8_t rxIdx = 0;
  uint8_t rxBuffer[UART1_QUEUE_SIZE] = {0, };

  if (_params != 0) { }

  while (1) {
    if (pdTRUE == xQueueReceive(uart1TxQueue, &data, 0)) {
      HAL_UART_Transmit(&UartHandle1, &data, 1, USART1_TIMEOUT);
    }
    if (pdTRUE == xQueueReceive(uart1RxQueue, rxBuffer+rxIdx, 0)) {
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

void __attribute__((used)) USART1_IRQHandler(void) {
  //HAL_UART_IRQHandler(&UartHandle1);
  if (__HAL_UART_GET_FLAG(&UartHandle1, UART_FLAG_RXNE)) {
    rxByte = (uint8_t)(UartHandle1.Instance->DR & (uint8_t)0x00FF);
    __HAL_UART_CLEAR_FLAG(&UartHandle1, UART_FLAG_RXNE);
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(uart1RxQueue, &rxByte, &xHigherPriorityTaskWoken);
  }
}
