#pragma once

#include <stdint.h>


int init_adc(void);
void deinit_adc(void);

uint16_t get_adc_value(uint32_t _channel);
uint16_t get_adc_imot1(void);
uint16_t get_adc_imot2(void);
void get_adc_imot12(uint16_t* _i1, uint16_t* _i2);
void get_adc_imot12_ma(uint16_t* _i1, uint16_t* _i2);


uint16_t get_adc_vbat(void);
float get_vbat_volt(void);
uint16_t get_adc_ibat(void);
float get_ibat_amp(void);

void test_bat(void);

//uint16_t imot_adc_to_ma(uint16_t adc);
