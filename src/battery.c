#include "battery.h"

static Battery_t battery;

int init_battery(void)
{
    battery.vbat = get_vbat_mv(); // [mV]
    battery.ibat = get_ibat_ma(); // [mA]
    battery.status = BAT_OK;
    battery.low_vbat_thresh = VBAT_LOW; // [mV]
    battery.critical_vbat_thresh = VBAT_CRITICAL_LOW; // [mV]
    battery.max_ibat_thresh = IBAT_MAX; // [mA]
    battery.low_vbat_start = 0; // [ms]
    battery.critical_vbat_start = 0; // [ms]
    battery.max_ibat_start = 0; // [ms]
    battery.low_vbat_to = VBAT_LOW_TO; // [ms]
    battery.critical_vbat_to = VBAT_CRITICAL_LOW_TO; // [ms]
    battery.max_ibat_to = IBAT_MAX_TO; // [ms]
    battery.last_timestamp = HAL_GetTick(); // [ms]
    battery.capacity_mah = BAT_CAPA_MAH;

    return process_battery(_bat);
}

int check_battery(Battery_t* _bat)
{
    int ret = battery.status;
    uint32_t timestamp = HAL_GetTick();
    uint32_t dt = timestamp - battery.last_timestamp;
    uint32_t vbat = get_vbat_mv();
    uint32_t ibat = get_ibat_ma();

    // LPF vbat
    battery.vbat += (uint16_t)((dt*(vbat-(uint32_t)battery.vbat))/(dt+VBAT_LPF_COEFF));
    // LPF ibat
    battery.ibat += (uint16_t)((dt*(ibat-(uint32_t)battery.ibat))/(dt+IBAT_LPF_COEFF));

    // Low battery
    if (battery.vbat < battery.low_vbat_thresh) {
        if (battery.low_vbat_start == 0) {
            battery.low_vbat_start = timestamp;
        }
        else if (timestamp - battery.low_vbat_start > battery.low_vbat_to) {
            battery.status = BAT_VLOW;
            ret = BAT_VLOW;
        }
    }
    else {
        battery.low_vbat_start = 0;
    }
    // Critical battery
    if (battery.vbat < battery.critical_vbat_thresh) {
        if (battery.critical_vbat_start == 0) {
            battery.critical_vbat_start = timestamp;
        }
        else if (timestamp - battery.critical_vbat_start > battery.critical_vbat_to) {
            battery.status = BAT_VCRITIC;
            ret = BAT_VCRITIC;
        }
    }
    else {
        battery.critical_vbat_start = 0;
    }
    // High current
    if (battery.ibat > battery.max_ibat_thresh) {
        if (battery.max_ibat_start == 0) {
            battery.max_ibat_start = timestamp;
        }
        else if (timestamp - battery.max_ibat_start > battery.max_ibat_to) {
            battery.status = BAT_IMAX;
            ret = BAT_IMAX;
        }
    }
    else {
        battery.max_ibat_start = 0;
    }
    battery.last_timestamp = timestamp;
    return ret;
}

void get_battery(Battery_t* _bat)
{
    return &battery;
}