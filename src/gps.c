/**
 * \file gps.h
 * \brief GPS parser
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 */

#include "gps.h"
#include "config.h"
#include "uart1.h"
#include "uart2.h"

#include <stdio.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static TaskHandle_t gpsTaskHandle = 0;
//static xQueueHandle gpsQueue = 0;
static bool isInit = false;
static bool isRunning = false;

int gps_init(void)
{    
    isRunning = false;
    isInit = true;
    return OK;
}

bool gps_is_init(void)
{
    return isInit;
}

bool gps_is_running(void)
{
    return isRunning;
}

int gps_start(void)
{
    isRunning = false;

    if (!(pdPASS == xTaskCreate(gps_task, (const char*)"gps_task",
            GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, &gpsTaskHandle))) {
        printf("gps_start: failed to create gps_task\n");
        return NOK;
    }

    isRunning = true;

    return OK;
}

int gps_stop(void)
{
    isRunning = false;
    gpsTaskHandle = 0;
    return OK;
}

void gps_task(void* _params)
{
    //TickType_t ticks = 0;
    int res = 0;
    uint8_t byte = 0;
    uint32_t lengthSend = 0;

    char msg[] = "Hello!\r\n";

    if (_params != 0) { }

    while (isRunning) {
        // TODO
        //ticks = xTaskGetTickCount();
        //printf("gps_task: ticks=%ld\r\n", ticks);
        //do { res = uart1_deque_byte(&byte, 10); } while (res == OK);
        res = uart1_deque_byte(&byte, 10);
        if (res == OK) {
            printf("gps_task: received c=0x%X from uart1\r\n", byte);
        }
        lengthSend = uart1_enque_data((uint8_t*)msg, strlen(msg));
        if (lengthSend == strlen(msg)) {
            //printf("gps_task: enque %ld data on uart1\r\n", lengthSend);
        }
        vTaskDelay(10/portTICK_RATE_MS); // 10 Hz
    }

    vTaskDelete(NULL);
}