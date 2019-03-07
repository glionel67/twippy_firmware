#include "uart3.h"

#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

UART_HandleTypeDef UartHandle3;

static xQueueHandle uart3RxQueue = 0;
static xQueueHandle uart3TxQueue = 0;

static bool isInit = false;
static bool hasOverrun = false;

int uart3_init(void)
{
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

  // UART TX FreeRTOS queue
  // uart3TxQueue = xQueueCreate(UART3_QUEUE_SIZE, sizeof(uint8_t));
  // if (uart3TxQueue == 0) {
  //   return NOK;
  // }

  // UART RX FreeRTOS queue
  uart3RxQueue = xQueueCreate(UART3_QUEUE_SIZE, sizeof(uint8_t));
  if (uart3RxQueue == 0) {
    return NOK;
  }

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(USART3_IRQ, 12, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQ);
  __HAL_UART_ENABLE_IT(&UartHandle3, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&UartHandle3, UART_IT_ERR);

  isInit = true;
  hasOverrun = false;

  return OK;
}

void uart3_deInit(void)
{
    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();
  HAL_GPIO_DeInit(USART3_GPIO_PORT, USART3_TX_PIN);
  HAL_GPIO_DeInit(USART3_GPIO_PORT, USART3_RX_PIN);
  HAL_NVIC_DisableIRQ(USART3_IRQ);
  __HAL_UART_DISABLE_IT(&UartHandle3, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(&UartHandle3, UART_IT_ERR);
}

bool uart3_is_init(void)
{
  return isInit;
}

int uart3_write(uint8_t* buf, uint32_t len)
{
  uint8_t res = HAL_UART_Transmit(&UartHandle3, buf, len, USART3_TIMEOUT);
  if (res == HAL_OK)
   return OK;
  else
    return NOK;
}

int uart3_read(uint8_t* buf, uint32_t len)
{
  uint8_t res = HAL_UART_Receive(&UartHandle3, buf, len, USART3_TIMEOUT);
  if (res == HAL_OK)
     return OK;
  else
    return NOK;
}

void uart3_send_data(uint8_t* data, uint32_t size)
{
    uint32_t i = 0;
    for (i=0;i<size;i++) {
        while (!(USART3->SR & USART_FLAG_TXE));
        USART3->DR = (data[i] & 0x00FF);
     }
}

uint32_t uart3_enque_data(uint8_t* data, uint32_t size)
{
  uint8_t i = 0;

  // Check that the queue is created
  //if (uart3TxQueue == 0) return 0;

  for (i=0;i<size;i++) {
    if (xQueueSend(uart3TxQueue, data+i, 10) == errQUEUE_FULL) {
      return i;
    }
  }
  return 0;
}

int uart3_deque_byte(uint8_t* _data, uint32_t _timeToWait)
{
  if (xQueueReceive(uart3RxQueue, (void*)_data, (TickType_t)_timeToWait) == pdTRUE)
    return OK;
  else {
    *_data = 0;
    return NOK;
  }
}

uint32_t uart3_deque_data(uint8_t* _data, uint32_t _len, uint32_t _timeToWait)
{
  uint32_t i = 0;
  for (i=0;i<_len;i++) {
    if (xQueueReceive(uart3RxQueue, (void*)&_data[i], (TickType_t)_timeToWait) == pdFALSE)
      break;
  }
  return i;
}

void uart3_task(void* _params)
{
  uint8_t data = 0;
  uint8_t res = 0;
  bool keepOn = false;

  if (_params != 0) { }

  while (1) {
    do {
      res = HAL_UART_Receive(&UartHandle3, &data, 1, 0);
      if (res == HAL_OK) {
        printf("uart3_task: c=0x%x\r\n", data);
        if (xQueueSend(uart3RxQueue, (const void*)&data, (TickType_t)0) == pdTRUE)
          keepOn = true;
        else {
          printf("uart3_task: failed to enque rx data\r\n");
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

bool uart3_has_overrun(void)
{
  bool result = hasOverrun;
  hasOverrun = false;
  return result;
}

void __attribute__((used)) USART3_IRQHandler(void)
{
  uint8_t rxByte = 0;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  if (USART3->SR & USART_SR_RXNE) {
    rxByte = (uint8_t)(USART3->DR & (uint8_t)0x00FF);
    //printf("uart3_int: 0x%x\r\n", rxByte);
    xQueueSendFromISR(uart3RxQueue, (const void*)&rxByte, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else {
    printf("uart3_int: ERROR\r\n");
    rxByte = (uint8_t)(USART3->DR & (uint8_t)0x00FF); // Flush data register
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
