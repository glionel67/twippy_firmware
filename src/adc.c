#include <stdio.h>
#include <string.h>

#include "adc.h"
#include "main.h"

ADC_HandleTypeDef AdcHandle1;
//ADC_HandleTypeDef AdcHandle2;

ADC_ChannelConfTypeDef sConfig1;
//ADC_ChannelConfTypeDef sConfig2;

int init_adc(void)
{
	ADC_IMOT_CLK_ENABLE();

	AdcHandle1.Instance = ADC_IMOT;
	if (HAL_ADC_DeInit(&AdcHandle1) != HAL_OK) // ADC de-initialization Error
		return -1;

	AdcHandle1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; /* Asynchronous clock mode, input ADC clock not divided */
	AdcHandle1.Init.Resolution = ADC_RESOLUTION_12B; /* 12-bit resolution for converted data */
	AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT; /* Right-alignment for converted data */
	AdcHandle1.Init.ScanConvMode = DISABLE; /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
	AdcHandle1.Init.EOCSelection = ADC_EOC_SINGLE_CONV; //DISABLE; /* EOC flag picked-up to indicate conversion end */
	AdcHandle1.Init.ContinuousConvMode = DISABLE; /* Continuous mode disabled to have only 1 conversion at each conversion trig */
	AdcHandle1.Init.NbrOfConversion = 1; /* Parameter discarded because sequencer is disabled */
	AdcHandle1.Init.DiscontinuousConvMode = DISABLE; /* Parameter discarded because sequencer is disabled */
	AdcHandle1.Init.NbrOfDiscConversion = 0; /* Parameter discarded because sequencer is disabled */
	AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1; /* Software start to trig the 1st conversion manually, without external event */
	AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
	AdcHandle1.Init.DMAContinuousRequests = DISABLE; /* DMA one-shot mode selected (not applied to this example) */

	if (HAL_ADC_Init(&AdcHandle1) != HAL_OK) // ADC initialization Error
		return -1;

	// Configure ADC regular channel
	sConfig1.Channel = ADC_IMOT1_CHANNEL; // Sampled channel number
	sConfig1.Rank = 1; // Rank of sampled channel number ADCx_CHANNEL
	sConfig1.SamplingTime = ADC_IMOT_SAMPLING_TIME; // Sampling time (number of clock cycles unit)
	sConfig1.Offset = 0; // Parameter discarded because offset correction is disabled

	if (HAL_ADC_ConfigChannel(&AdcHandle1, &sConfig1) != HAL_OK) // Channel Configuration Error
		return -1;

	//HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
	//HAL_NVIC_EnableIRQ(ADC_IRQn);

	return 0;
}

void deinit_adc(void)
{
	ADC_IMOT_FORCE_RESET();
	ADC_IMOT_RELEASE_RESET();
	HAL_ADC_DeInit(&AdcHandle1);
}

uint16_t get_adc_value(uint32_t _channel)
{
	uint16_t val = 0;

	// Configure ADC regular channel
	sConfig1.Channel = _channel; // Sampled channel number

	if (HAL_ADC_ConfigChannel(&AdcHandle1, &sConfig1) != HAL_OK)
		return 65535; // Channel Configuration Error

	// Start the conversion process
	if (HAL_ADC_Start(&AdcHandle1) != HAL_OK) // Start Conversation Error
		return 65535;

	// End Of Conversion flag not set on time
	if (HAL_ADC_PollForConversion(&AdcHandle1, 10) != HAL_OK)
		return 65535;

	// Check if the continuous conversion of regular channel is finished
	if ((HAL_ADC_GetState(&AdcHandle1) & HAL_ADC_STATE_EOC_REG)
			== HAL_ADC_STATE_EOC_REG) {
		// Get the converted value of regular channel
		val = HAL_ADC_GetValue(&AdcHandle1);
	} else {
		return 65535;
	}

	return val;
}

uint16_t get_adc_imot1(void)
{
	return get_adc_value(ADC_IMOT1_CHANNEL);
}

uint16_t get_adc_imot2(void)
{
	return get_adc_value(ADC_IMOT2_CHANNEL);
}

void get_adc_imot12(uint16_t* _i1, uint16_t* _i2)
{
	(*_i1) = (*_i2) = 65535;
	(*_i1) = get_adc_value(ADC_IMOT1_CHANNEL);
	(*_i2) = get_adc_value(ADC_IMOT2_CHANNEL);
}

void get_adc_imot12_ma(uint16_t* _i1, uint16_t* _i2)
{
	uint16_t adc = 0;
	uint32_t volt = 0;
	uint32_t ma = 0;

	adc = get_adc_value(ADC_IMOT1_CHANNEL);
	if (adc != 0xFFFF) {
		volt = ((uint32_t)adc*(uint32_t)ADC_IMOT_VREF)/((uint32_t)ADC_IMOT_RESOLUTION);
		ma = (volt*IMOT_RATIO_NUM)/IMOT_RATIO_DEN;
		(*_i1) = (uint16_t)ma;
	}
	else {
		(*_i1) = 0;
	}

	adc = get_adc_value(ADC_IMOT2_CHANNEL);
	if (adc != 0xFFFF) {
		volt = ((uint32_t)adc*(uint32_t)ADC_IMOT_VREF)/((uint32_t)ADC_IMOT_RESOLUTION);
		ma = (volt*IMOT_RATIO_NUM)/IMOT_RATIO_DEN;
		(*_i2) = (uint16_t)ma;
	}
	else {
		(*_i2) = 0;
	}
}

uint16_t get_adc_vbat(void)
{
	return get_adc_value(ADC_VBAT_CHANNEL);
}

float get_vbat_volt(void)
{
	uint16_t adc = get_adc_value(ADC_VBAT_CHANNEL);
	float volt = ((float)adc * ADC_BAT_VREF) / ADC_BAT_RESOLUTION;
	return (volt * VBAT_RATIO);
}

uint16_t get_adc_ibat(void)
{
	return get_adc_value(ADC_IBAT_CHANNEL);
}

float get_ibat_amp(void)
{
	uint16_t adc = get_adc_value(ADC_IBAT_CHANNEL);
	float volt = ((float)adc * ADC_BAT_VREF) / ADC_BAT_RESOLUTION;
	return ((volt - IBAT_ZERO) * IBAT_RATIO);
}

void test_bat(void)
{
	float vbat = 0.f, ibat = 0.f;
	for (int i=0;i<5;i++) {
		vbat = get_vbat_volt();
		ibat = get_ibat_amp();
		printf("vbat=%3.3f,ibat=%3.3f\r\n", (float)vbat, (float)ibat);
		HAL_Delay(1000);
	}
}