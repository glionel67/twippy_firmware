#pragma once

#include <stdint.h>
#include "config.h"
#include "adc.h"

enum {
    BAT_OK=0, BAT_VLOW, BAT_VCRITIC, BAT_IMAX
};

typedef struct {
    uint16_t vbat; // [mV]
    uint16_t ibat; // [mA]
    uint8_t status;
    uint16_t low_vbat_thresh; // [mV]
    uint16_t max_ibat_thresh; // [mA]
    uint32_t low_vbat_start; // [ms]
    uint32_t max_ibat_start; // [ms]
    uint16_t low_vbat_to; // [ms]
    uint16_t max_ibat_to; // [ms]
    uint16_t critical_vbat_thresh; // [mV]
    uint32_t critical_vbat_start; // [ms]
    uint16_t critical_vbat_to; // [ms]
    uint32_t last_timestamp; // [ms]
    uint16_t capacity_mah;
} Battery_t;

int init_battery(Battery_t* _bat);

int process_battery(Battery_t* _bat);