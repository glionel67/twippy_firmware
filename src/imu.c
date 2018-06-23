#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "imu.h"
#include "main.h"
#include "usTimer.h"

static xQueueHandle imuQueue = 0;

static uint8_t imu_int_flag = 0;

static Mpu9250_t mpu9250;

uint8_t init_imu(void) {
    uint8_t ret = 0; 

    ret = mpu9250_init(&mpu9250);
    if (!ret) {
        char str[] = "mpu9250_init error\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return 0;
    }

    // Activate IMU INT PIN
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_NVIC_SetPriority(IMU_INT_IRQ, 1, 0);
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

    if (_params != 0) { }

    ret = init_imu();
    if (!ret) {
        char str[] = "init_imu error\r\n";
        print_msg((uint8_t*)str, strlen(str));
        return;
    }

    while (1) {
        if (imu_int_flag) {
            // Get timestamp
            imu9.timestamp = (float)get_us_time() * (float)1e-6;
            // Read data
            mpu9250_get_acc_gyro_mag_temp(&mpu9250);
            // Copy data
            for (i=0;i<N_AXES;i++) {
                imu9.a[i] = mpu9250.a[i];
                imu9.g[i] = mpu9250.g[i];
                imu9.m[i] = mpu9250.m[i];
            }
            // Print data
            sprintf(data, "t=%3.3f,ax=%3.3f,ay=%3.3f,az=%3.3f,gx=%3.3f,gy=%3.3f,gz=%3.3f\r\n",
                (float)imu9.timestamp, (float)imu9.a[0], (float)imu9.a[1], (float)imu9.a[2],
                (float)imu9.g[0], (float)imu9.g[1], (float)imu9.g[2]);
            print_msg((uint8_t*)data, strlen(data));
            // Clear flag
            imu_int_flag = 0;
            // Delay
            vTaskDelay(500/portTICK_RATE_MS);
        }
    }

    HAL_NVIC_DisableIRQ(IMU_INT_IRQ);
    vTaskDelete(NULL);
}

void imu_task(void* _params) {
    uint8_t ret = 0;
    uint8_t i = 0;
    Imu9_t imu9;

    if (_params != 0) { }

    ret = init_imu();
    if (!ret) {
        char str[] = "init_imu error\r\n";
        print_msg((uint8_t*)str, strlen(str));
        Error_Handler();
    }

    imuQueue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(Imu9_t));
    if (imuQueue == 0) {
        char str[] = "imuQueue creation error\r\n";
        print_msg((uint8_t*)str, strlen(str));
        Error_Handler();
    }

    while (1) {
        if (imu_int_flag) {
            // Get timestamp
            imu9.timestamp = (float)get_us_time() * (float)1e-6;
            // Read data
            mpu9250_get_acc_gyro_mag_temp(&mpu9250);
            // Copy data
            for (i=0;i<N_AXES;i++) {
                imu9.a[i] = mpu9250.a[i];
                imu9.g[i] = mpu9250.g[i];
                imu9.m[i] = mpu9250.m[i];
            }
            // Send it over the queue
            xQueueOverwrite(imuQueue, &imu9);
            // Clear flag
            imu_int_flag = 0;
        }
        vTaskDelay(1); // 1 ms
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
        //mpu9250_read_reg(&mpu9250);
        imu_int_flag = 1;
    }
}