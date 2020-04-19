/**
 * \file battery.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief Battery related functions (voltage and current)
 */

#pragma once

#include <stdint.h>

#include "FreeRTOS.h"

/**
 * \enum BatteryState_e
 * \brief Enumeration of the battery states
 */
enum
{
    BAT_OK=0, BAT_VLOW, BAT_VCRITIC, BAT_IMAX
} BatteryState_e;

/**
 * \struct BatteryInfos_t
 * \brief Structure containing the battery information
 */
typedef struct BatteryInfos_s
{
    uint16_t vbat; // [mV]
    uint16_t ibat; // [mA]
    uint8_t status; // BatteryState_e
} BatteryInfos_t;

/**
 * \struct BatteryParams_t
 * \brief Structure containing the battery parameters
 */
typedef struct BatteryParams_s
{
    uint16_t capacity_mah; // [mAh]

    uint16_t low_vbat_thresh; // [mV]
    uint16_t critical_vbat_thresh; // [mV]
    uint16_t max_ibat_thresh; // [mA]

    uint16_t low_vbat_to; // [ms]
    uint16_t critical_vbat_to; // [ms]
    uint16_t max_ibat_to; // [ms]

    float vbat_lpf_rc;
    float ibat_lpf_rc;
} BatteryParams_t;

/**
 * \struct Battery_t
 * \brief Structure containing the battery information and parameters
 */
typedef struct Battery_s
{
    BatteryInfos_t infos;
    BatteryParams_t params;
} Battery_t;

/**
 * \fn init_battery
 * \brief Initialize the battery functions
 * \return OK if success, NOK otherwise
 */
int init_battery(void);

/**
 * \fn check_battery
 * \brief Check the battery status
 * \return BatteryState_e the state of the battery
 */
int check_battery(void);

/**
 * \fn battery_task
 * \brief Task used to monitor the battery status (voltage + current)
 */
void battery_task(void* _params);

/**
 * \fn get_battery_params
 * \brief Return the battery parameters
 * \param _batParams battery parameters structure
 */
void get_battery_params(BatteryParams_t* _batParams);

/**
 * \fn set_battery_params
 * \brief Set the battery parameters
 * \param _batParams battery parameters structure
 */
void set_battery_params(BatteryParams_t* _batParams);

/**
 * \fn get_battery_infos
 * \brief Return the battery information
 * \param _bat battery structure used to store the information
 */
uint8_t get_battery_infos(BatteryInfos_t* data, TickType_t xTicksToWait);