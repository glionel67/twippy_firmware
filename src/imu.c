#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "imu.h"
#include "spi.h"
#include "main.h"
#include "usTimer.h"

#define DEBUG_MODULE "imu"

static xQueueHandle imuQueue = 0;
static xSemaphoreHandle imuDataReady;
static Mpu9250_t mpu9250;

uint8_t init_imu(void) {
    uint8_t ret = 0; 

    ret = mpu9250_init(&mpu9250);
    if (!ret) {
        char str[] = "mpu9250_init NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }
    else {
        char str[] = "mpu9250_init OK\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }

    // Augment SPI baudrate to access IMU data register
    ret = spi1_set_speed(SPI_BAUDRATE_11MHZ); //);
    if (!ret) {
        char str[] = "spi1_set_speed NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }
    else {
        char str[] = "spi1_set_speed OK\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }

    // Create imu queue
    imuQueue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(Imu9_t));
    if (imuQueue == 0) {
        char str[] = "imuQueue creation NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }
    else {
        char str[] = "imuQueue creation OK\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }

    // Create imu semaphore for interrupt synchronisation
    imuDataReady = xSemaphoreCreateBinary();
    if (imuDataReady == NULL) {
        char str[] = "imuDataReady semaphore creation NOK\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }
    else {
        char str[] = "imuDataReady semaphore creation OK\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }

    // Activate IMU INT PIN
    HAL_NVIC_SetPriority(IMU_INT_IRQ, 5, 0);
    HAL_NVIC_EnableIRQ(IMU_INT_IRQ);

    return 1;
}

uint8_t test_imu(void) {
    uint8_t ret = 0;
    uint8_t pass = 0;

    ret = mpu9250_check_devId(&mpu9250);
    if (!ret) {
        char str[] = "mpu9250_check_devId error\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }
    else
        pass++;

    ret = mpu9250_check_mag_devId(&mpu9250);
    if (!ret) {
        char str[] = "mpu9250_check_mag_devId error\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }
    else
        pass++;

    ret = mpu9250_get_acc_gyro_mag_temp(&mpu9250);
    if (!ret) {
        char str[] = "mpu9250_get_acc_gyro_mag_temp error\r\n";
        print_msg((uint8_t*)str, strlen(str));
    }
    else
        pass++;

    return pass;
}

void imu_test_task(void* _params) {
    uint8_t ret = 0;
    uint8_t i = 0;
    Imu9_t imu9;
    char data[100] = { 0, };

    memset((void*)&imu9, 0, sizeof(Imu9_t));

    if (_params != 0) { }
  
    while (1) {
        if (pdTRUE == xSemaphoreTake(imuDataReady, portMAX_DELAY)) {
            // Read data
            vTaskSuspendAll();
            //ret = mpu9250_get_acc_gyro_mag_temp(&mpu9250);
            ret = mpu9250_read_data_register(&mpu9250);
            xTaskResumeAll();

            if (ret) {
                // Get timestamp
                imu9.timestamp = (float)get_us_time() * (float)1e-6;

                // Extract data
                mpu9250_extract_data_register(&mpu9250);

                // Copy data
                for (i=0;i<N_AXES;i++) {
                    imu9.a[i] = mpu9250.a_raw[i];
                    imu9.g[i] = mpu9250.g_raw[i];
                    imu9.m[i] = mpu9250.m_raw[i];
                }

                xQueueOverwrite(imuQueue, &imu9);

                sprintf(data, "t=%3.3f,ax=%3.3f,ay=%3.3f,az=%3.3f,gx=%3.3f,gy=%3.3f,gz=%3.3f\r\n",
                    (float)imu9.timestamp, 
                    (float)imu9.a[0], (float)imu9.a[1], (float)imu9.a[2],
                    (float)imu9.g[0], (float)imu9.g[1], (float)imu9.g[2]);
            }
            else {
                if (!mpu9250.isImuInit) {
                    // TODO: emergency stop ?!
                }
            }
            //print_msg((uint8_t*)data, strlen(data));
        }
        vTaskDelay(200);
    }

    HAL_NVIC_DisableIRQ(IMU_INT_IRQ);
    vTaskDelete(NULL);
}

void imu_task(void* _params) {
    uint8_t ret = 0;
    uint8_t i = 0;
    Imu9_t imu9;

    if (_params != 0) { }

    while (1) {
        if (pdTRUE == xSemaphoreTake(imuDataReady, portMAX_DELAY)) {
            // Read data
            ret = mpu9250_get_acc_gyro_mag_temp(&mpu9250);
            if (ret) {
                // Get timestamp
                imu9.timestamp = (float)get_us_time() * (float)1e-6;
                // Copy data
                for (i=0;i<N_AXES;i++) {
                    imu9.a[i] = mpu9250.a_raw[i];
                    imu9.g[i] = mpu9250.g_raw[i];
                    imu9.m[i] = mpu9250.m_raw[i];
                }
                // Send it over the queue
                //vTaskSuspendAll();
                xQueueOverwrite(imuQueue, &imu9);
                //xTaskResumeAll();
            }
        }
    }

    HAL_NVIC_DisableIRQ(IMU_INT_IRQ);
    vTaskDelete(NULL);
}

uint8_t imu_read_data(Imu9_t* imu) {
    return (pdTRUE == xQueueReceive(imuQueue, imu, 0));
}

uint8_t is_imu_calibrated(void) {
    return mpu9250.isCalibrated;
}

void get_imu9_data(Imu9_t* imu) {
    uint8_t i = 0;
    for (i=0;i<N_AXES;i++) {
        imu->a[i] = mpu9250.a[i];
        imu->g[i] = mpu9250.g[i];
        imu->m[i] = mpu9250.m[i];
    }
}

 void get_imu6_data(Imu6_t* imu) {
    uint8_t i = 0;
    for (i=0;i<N_AXES;i++) {
        imu->a[i] = mpu9250.a[i];
        imu->g[i] = mpu9250.g[i];
    }
}

void __attribute__((used)) EXTI2_IRQHandler(void) {
    if (__HAL_GPIO_EXTI_GET_IT(IMU_INT_PIN) != RESET) {
        __HAL_GPIO_EXTI_CLEAR_IT(IMU_INT_PIN);
        //char data[] = "INT\r\n";
        //print_msg((uint8_t*)data, strlen(data));
        portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(imuDataReady, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}