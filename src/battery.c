/**
 * \file battery.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief Battery related functions (voltage and current)
 */

#include "battery.h"
#include "main.h"
#include "adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define BATTERY_TASK_PERIOD_MS (100) // 100 ms period <=> 10 Hz
#define BATTERY_QUEUE_SIZE (1)

#define VBAT_LPF_COEFF (0.015915494f) // LPF RC = 1 / (2 * \pi * fc), fc = 10 Hz
#define IBAT_LPF_COEFF (0.015915494f) // LPF RC = 1 / (2 * \pi * fc), fc = 10 Hz

#define BAT_CAPA_MAH (3300)  // [mAh]
#define VBAT_LOW (10000)  // [mV]
#define VBAT_CRITICAL_LOW (9500)  // [mV]
#define IBAT_MAX (10000) // [mA]
#define VBAT_LOW_TO (5000) // [ms]
#define VBAT_CRITICAL_LOW_TO (5000) // [ms]
#define IBAT_MAX_TO (500) // [ms]

static Battery_t battery;

static xQueueHandle batteryInfoQueue = 0;

int init_battery(void)
{
    // Low-level ADC initialization already done in init_adc
    if (!is_adc_init())
        return NOK;

    // Create battery info queue
    batteryInfoQueue = xQueueCreate(BATTERY_QUEUE_SIZE, sizeof(BatteryInfos_t));
    if (0 == batteryInfoQueue)
    {
        printf("init_battery: batteryInfoQueue creation NOK\r\n");
        return NOK;
    }
    else
        printf("init_battery: batteryInfoQueue creation OK\r\n");

    // Init battery parameters
    battery.infos.vbat = (uint16_t)(1000.0 * get_vbat_volt()); // [mV]
    battery.infos.ibat = (uint16_t)(1000.0 * get_ibat_amp()); // [mA]
    battery.infos.status = BAT_OK;

    battery.params.capacity_mah = BAT_CAPA_MAH; // [mAh]

    battery.params.low_vbat_thresh = VBAT_LOW; // [mV]
    battery.params.critical_vbat_thresh = VBAT_CRITICAL_LOW; // [mV]
    battery.params.max_ibat_thresh = IBAT_MAX; // [mA]

    battery.params.low_vbat_to = VBAT_LOW_TO; // [ms]
    battery.params.critical_vbat_to = VBAT_CRITICAL_LOW_TO; // [ms]
    battery.params.max_ibat_to = IBAT_MAX_TO; // [ms]

    battery.params.vbat_lpf_rc = VBAT_LPF_COEFF;
    battery.params.ibat_lpf_rc = IBAT_LPF_COEFF;

    return OK;
} // init_battery

int check_battery(void)
{
    static uint32_t low_vbat_start = 0; // [ms]
    static uint32_t critical_vbat_start = 0; // [ms]
    static uint32_t max_ibat_start = 0; // [ms]
    static uint32_t last_timestamp = 0; // [ms]
    uint32_t timestamp = HAL_GetTick(); // [ms]
    uint32_t dt = timestamp - last_timestamp; // [ms]
    float vbat = get_vbat_volt();
    float ibat = get_ibat_amp();
    float prevVbat = (float)battery.infos.vbat / 1000.f;
    float prevIbat = (float)battery.infos.ibat / 1000.f;
    float dtf = (float)dt / 1000.f;
    float alpha = 0.f;
    
    battery.infos.status = BAT_OK;

    // LPF vbat
    alpha = dtf / (dtf + battery.params.vbat_lpf_rc);
    vbat = alpha * vbat + (1.f - alpha) * prevVbat;
    battery.infos.vbat += (uint16_t)(vbat);
    // LPF ibat
    alpha = dtf / (dtf + battery.params.ibat_lpf_rc);
    ibat = alpha * ibat + (1.f - alpha) * prevIbat;
    battery.infos.ibat += (uint16_t)(ibat);

    // Low battery
    if (battery.infos.vbat < battery.params.low_vbat_thresh)
    {
        if (0 == low_vbat_start)
        {
            low_vbat_start = timestamp;
        }
        else if (timestamp - low_vbat_start > battery.params.low_vbat_to)
        {
            battery.infos.status = BAT_VLOW;
        }
    }
    else
    {
        low_vbat_start = 0;
    }

    // Critical battery
    if (battery.infos.vbat < battery.params.critical_vbat_thresh)
    {
        if (0 == critical_vbat_start)
        {
            critical_vbat_start = timestamp;
        }
        else if (timestamp - critical_vbat_start > battery.params.critical_vbat_to)
        {
            battery.infos.status = BAT_VCRITIC;
        }
    }
    else
    {
        critical_vbat_start = 0;
    }

    // High current
    if (battery.infos.ibat > battery.params.max_ibat_thresh)
    {
        if (0 == max_ibat_start)
        {
            max_ibat_start = timestamp;
        }
        else if (timestamp - max_ibat_start > battery.params.max_ibat_to)
        {
            battery.infos.status = BAT_IMAX;
        }
    }
    else
    {
        max_ibat_start = 0;
    }

    last_timestamp = timestamp;
    
    return battery.infos.status;
} // check_battery

void battery_task(void* _params)
{
    if (_params != 0) { }

    // Initialize battery
    int ret = init_battery();
    if (NOK == ret)
    {
        printf("battery_task: init_battery failed\r\n");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        // Check battery status
        ret = check_battery();

        // Send battery information over queue
        xQueueOverwrite(batteryInfoQueue, &battery.infos);

        vTaskDelay(BATTERY_TASK_PERIOD_MS / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
} // battery_task

void get_battery_params(BatteryParams_t* _batParams)
{
    _batParams->capacity_mah = battery.params.capacity_mah;

    _batParams->low_vbat_thresh = battery.params.low_vbat_thresh;
    _batParams->critical_vbat_thresh = battery.params.critical_vbat_thresh;
    _batParams->max_ibat_thresh = battery.params.max_ibat_thresh;

    _batParams->low_vbat_to = battery.params.low_vbat_to;
    _batParams->critical_vbat_to = battery.params.critical_vbat_to;
    _batParams->max_ibat_to = battery.params.max_ibat_to;

    _batParams->vbat_lpf_rc = battery.params.vbat_lpf_rc;
    _batParams->ibat_lpf_rc = battery.params.ibat_lpf_rc;
} // get_battery_params

void set_battery_params(BatteryParams_t* _batParams)
{
    battery.params.capacity_mah = _batParams->capacity_mah;

    battery.params.low_vbat_thresh = _batParams->low_vbat_thresh;
    battery.params.critical_vbat_thresh = _batParams->critical_vbat_thresh;
    battery.params.max_ibat_thresh = _batParams->max_ibat_thresh;

    battery.params.low_vbat_to = _batParams->low_vbat_to;
    battery.params.critical_vbat_to = _batParams->critical_vbat_to;
    battery.params.max_ibat_to = _batParams->max_ibat_to;

    battery.params.vbat_lpf_rc = _batParams->vbat_lpf_rc;
    battery.params.ibat_lpf_rc = _batParams->ibat_lpf_rc;
} // set_battery_params

uint8_t get_battery_infos(BatteryInfos_t* data, TickType_t xTicksToWait)
{
    return (pdTRUE == xQueueReceive(batteryInfoQueue, data, xTicksToWait));
} // get_battery_infos