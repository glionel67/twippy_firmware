#pragma once

#include <stdint.h>


int init_adc_motors(void);
void deinit_adc_motors(void);

uint16_t get_adc_imot_value(uint32_t _channel);
uint16_t get_adc_imot1(void);
uint16_t get_adc_imot2(void);
void get_adc_imot12(uint16_t* _i1, uint16_t* _i2);
void get_adc_imot12_ma(uint16_t* _i1, uint16_t* _i2);

/*
uint16_t get_adc_vbat(void);
uint16_t get_vbat_mv(void);
uint16_t get_adc_ibat(void);
uint16_t get_ibat_ma(void);

uint16_t imot_adc_to_ma(uint16_t adc);
*/
