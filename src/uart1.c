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
    return NOK;
  }

  uart1TxQueue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  if (uart1TxQueue == 0) {
    return NOK;
  }

  uart1RxQueue = xQueueCreate(UART1_QUEUE_SIZE, sizeof(uint8_t));
  if (uart1RxQueue == 0) {
    return NOK;
  }

  HAL_NVIC_SetPriority(USART1_IRQ, 10, 0);
  //HAL_NVIC_EnableIRQ(USART1_IRQ);
  __HAL_UART_ENABLE_IT(&UartHandle1, UART_IT_RXNE);

  isInit = true;

  return OK;
}

void uart1_deInit(void) {
	__HAL_RCC_USART1_FORCE_RESET();
	__HAL_RCC_USART1_RELEASE_RESET();
  HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_TX_PIN);
  HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_RX_PIN);
  HAL_NVIC_DisableIRQ(USART1_IRQ);
  __HAL_UART_DISABLE_IT(&UartHandle1, UART_IT_RXNE);

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
	    USART1->DR = (data[i] & 0x00FF);
	 }
}

uint32_t uart1_enque_data(uint8_t* _data, uint32_t _len)
{
  uint32_t i = 0;
  for (i=0;i<_len;i++) {
    if (xQueueSend(uart1TxQueue, &_data[i], (TickType_t)1) == errQUEUE_FULL) {
      break;;
    }
  }
  return i; // Return the number of data written to the queue
}

int uart1_deque_byte(uint8_t* _data, uint32_t _timeToWait)
{
  if (pdTRUE == xQueueReceive(uart1RxQueue, _data, (TickType_t)_timeToWait))
    return OK;
  else
    return NOK;
}

uint32_t uart1_deque_data(uint8_t* _data, uint32_t _len, uint32_t _timeToWait)
{
  uint32_t i = 0;
  for (i=0;i<_len;i++) {
    if (xQueueReceive(uart1RxQueue, &_data[i], (TickType_t)_timeToWait) == pdFALSE)
      break;
  }
  return i;
}

void uart1_task(void* _params)
{
  uint8_t data = 0;
  uint8_t res = 0;

  if (_params != 0) { }

  while (1) {
    // TX data in the TX queue
    while (pdTRUE == xQueueReceive(uart1TxQueue, &data, (TickType_t)0)) {
      //printf("uart1_task: rcv from TX queue and sending to uart1\r\n");
      res = HAL_UART_Transmit(&UartHandle1, &data, 1, USART1_TIMEOUT);
      if (res != HAL_OK) {
        printf("uart1_task: failed to send data via uart1\r\n");
      }
    }

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
  //HAL_UART_IRQHandler(&UartHandle1);
  if (__HAL_UART_GET_IT_SOURCE(&UartHandle1, UART_IT_RXNE)) {
  //if (__HAL_UART_GET_FLAG(&UartHandle1, UART_FLAG_RXNE)) {
    rxByte = (uint8_t)(UartHandle1.Instance->DR & (uint8_t)0x00FF);
    __HAL_UART_CLEAR_FLAG(&UartHandle1, UART_FLAG_RXNE);
    xQueueSendFromISR(uart1RxQueue, &rxByte, &xHigherPriorityTaskWoken);
  }
  else {
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
