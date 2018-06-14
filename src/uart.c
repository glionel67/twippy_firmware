#include "uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

UART_HandleTypeDef UartHandle1;
UART_HandleTypeDef UartHandle2;

static xQueueHandle uart1Queue = 0;
static xQueueHandle uart2Queue = 0;

uint8_t uart1_tx_ok = 1;
uint8_t uart2_tx_ok = 1;

int uart1_init(void) {
  // Enable USARTx clock
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

  __HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_RXNE);

  NVIC_EnableIRQ(USART1_IRQn);

  return 0;
}

void uart1_deInit(void) {
	__HAL_RCC_USART1_FORCE_RESET();
	__HAL_RCC_USART1_RELEASE_RESET();
  HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_TX_PIN);
  HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_RX_PIN);
}

int uart1_write(uint8_t* buf, uint8_t len) {
	return HAL_UART_Transmit(&UartHandle1, buf, len, USART1_TIMEOUT);
}

int uart1_write_it(uint8_t* buf, uint8_t len) {
	if (!uart1_tx_ok)
		return -1;
	uart1_tx_ok = 0;
	return HAL_UART_Transmit_IT(&UartHandle1, buf, len);
}

int uart1_read(uint8_t* buf, uint8_t len) {
	return HAL_UART_Receive(&UartHandle1, buf, len, USART1_TIMEOUT);
}

void uart1_send_data(uint8_t* data, uint32_t size) {
	uint32_t i = 0;

	for (i=0;i<size;i++) {
	    while (!(USART1->SR & USART_FLAG_TXE));
	    USART1->DR = (data[i] & 0x00FF);
	 }
}

uint32_t uart1_enque_data(uint8_t* data, uint32_t size) {
  uint32_t i = 0;
  for (i=0;i<size;i++) {
    if (xQueueSend(uart1Queue, data+i, 10) == errQUEUE_FULL) {
      return i;
    }
  }
  return 0;
}

void uart1_task(void* _params) {
  uint8_t data = 0;

  if (_params != 0) { }

  uart1Queue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  if (uart1Queue == 0)
    return;

  while (1) {
    if (pdTRUE == xQueueReceive(uart1Queue, &data, 10)) {
      HAL_UART_Transmit(&UartHandle1, &data, 1, USART1_TIMEOUT);
    }
    else {
      vTaskDelay(10/portTICK_RATE_MS);
    }
  }

  vTaskDelete(NULL);
}

int uart2_init(void) {
  __HAL_RCC_USART2_CLK_ENABLE();

  UartHandle2.Instance        = USART2;
  UartHandle2.Init.BaudRate   = USART2_BAUDRATE;
  UartHandle2.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle2.Init.StopBits   = UART_STOPBITS_1;
  UartHandle2.Init.Parity     = UART_PARITY_NONE;
  UartHandle2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle2.Init.Mode       = UART_MODE_TX_RX;
  UartHandle2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle2) != HAL_OK) {
    return -1;
  }

  __HAL_UART_ENABLE_IT(&UartHandle2, UART_IT_RXNE);

  NVIC_EnableIRQ(USART2_IRQn);

  return 0;
}

void uart2_deInit(void) {
	__HAL_RCC_USART2_FORCE_RESET();
	__HAL_RCC_USART2_RELEASE_RESET();
  HAL_GPIO_DeInit(USART2_GPIO_PORT, USART2_TX_PIN);
  HAL_GPIO_DeInit(USART2_GPIO_PORT, USART2_RX_PIN);
}

int uart2_write(uint8_t* buf, uint8_t len) {
	return HAL_UART_Transmit(&UartHandle2, buf, len, USART2_TIMEOUT);
}

int uart2_write_it(uint8_t* buf, uint8_t len) {
	if (!uart2_tx_ok)
		return -1;
	uart2_tx_ok = 0;
	return HAL_UART_Transmit_IT(&UartHandle2, buf, len);
}

int uart2_read(uint8_t* buf, uint8_t len) {
	return HAL_UART_Receive(&UartHandle2, buf, len, USART2_TIMEOUT);
}

int uart2_read_it(uint8_t* buf, uint8_t len) {
	return HAL_UART_Receive_IT(&UartHandle2, buf, len);
}

void uart2_send_data(uint8_t* data, uint32_t size) {
	uint32_t i = 0;

	for (i=0;i<size;i++) {
	    while (!(USART2->SR & USART_FLAG_TXE));
	    USART2->DR = (data[i] & 0x00FF);
	 }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
	if(UartHandle->Instance == USART1) {
		uart1_tx_ok = 1;
	}
	else if (UartHandle->Instance == USART2) {
		uart2_tx_ok = 1;
	}
}

uint32_t uart2_enque_data(uint8_t* data, uint32_t size) {
  uint32_t i = 0;
  for (i=0;i<size;i++) {
    if (xQueueSend(uart2Queue, data+i, 10) == errQUEUE_FULL) {
      return i;
    }
  }
  return 0;
}

void uart2_task(void* _params) {
  uint8_t data = 0;

  if (_params != 0) { }

  uart2Queue = xQueueCreate(UART2_QUEUE_SIZE, sizeof(uint8_t));
  if (uart2Queue == 0)
    return;

  while (1) {
    if (pdTRUE == xQueueReceive(uart2Queue, &data, 10)) {
      HAL_UART_Transmit(&UartHandle2, &data, 1, USART2_TIMEOUT);
    }
    else {
      vTaskDelay(10/portTICK_RATE_MS);
    }
  }

  vTaskDelete(NULL);
}
