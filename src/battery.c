#include "battery.h"

int init_battery(Battery_t* _bat) {
    _bat->vbat = get_vbat_mv(); // [mV]
    _bat->ibat = get_ibat_ma(); // [mA]
    _bat->status = BAT_OK;
    _bat->low_vbat_thresh = VBAT_LOW; // [mV]
    _bat->critical_vbat_thresh = VBAT_CRITICAL_LOW; // [mV]
    _bat->max_ibat_thresh = IBAT_MAX; // [mA]
    _bat->low_vbat_start = 0; // [ms]
    _bat->critical_vbat_start = 0; // [ms]
    _bat->max_ibat_start = 0; // [ms]
    _bat->low_vbat_to = VBAT_LOW_TO; // [ms]
    _bat->critical_vbat_to = VBAT_CRITICAL_LOW_TO; // [ms]
    _bat->max_ibat_to = IBAT_MAX_TO; // [ms]
    _bat->last_timestamp = HAL_GetTick(); // [ms]
    _bat->capacity_mah = BAT_CAPA_MAH;

    return process_battery(_bat);
}

int process_battery(Battery_t* _bat) {
    int ret = _bat->status;
    uint32_t timestamp = HAL_GetTick();
    uint32_t dt = timestamp - _bat->last_timestamp;
    uint32_t vbat = get_vbat_mv();
    uint32_t ibat = get_ibat_ma();

    // LPF vbat
    _bat->vbat += (uint16_t)((dt*(vbat-(uint32_t)_bat->vbat))/(dt+VBAT_LPF_COEFF));
    // LPF ibat
    _bat->ibat += (uint16_t)((dt*(ibat-(uint32_t)_bat->ibat))/(dt+IBAT_LPF_COEFF));

    // Low battery
    if (_bat->vbat < _bat->low_vbat_thresh) {
        if (_bat->low_vbat_start == 0) {
            _bat->low_vbat_start = timestamp;
        }
        else if (timestamp - _bat->low_vbat_start > _bat->low_vbat_to) {
            _bat->status = BAT_VLOW;
            ret = BAT_VLOW;
        }
    }
    else {
        _bat->low_vbat_start = 0;
    }
    // Critical battery
    if (_bat->vbat < _bat->critical_vbat_thresh) {
        if (_bat->critical_vbat_start == 0) {
            _bat->critical_vbat_start = timestamp;
        }
        else if (timestamp - _bat->critical_vbat_start > _bat->critical_vbat_to) {
            _bat->status = BAT_VCRITIC;
            ret = BAT_VCRITIC;
        }
    }
    else {
        _bat->critical_vbat_start = 0;
    }
    // High current
    if (_bat->ibat > _bat->max_ibat_thresh) {
        if (_bat->max_ibat_start == 0) {
            _bat->max_ibat_start = timestamp;
        }
        else if (timestamp - _bat->max_ibat_start > _bat->max_ibat_to) {
            _bat->status = BAT_IMAX;
            ret = BAT_IMAX;
        }
    }
    else {
        _bat->max_ibat_start = 0;
    }
    _bat->last_timestamp = timestamp;
    return ret;
}