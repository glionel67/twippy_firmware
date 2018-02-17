#include "encoders.h"
#include "main.h"

// Timer handler declaration
TIM_HandleTypeDef TimHandleEnc1;
TIM_HandleTypeDef TimHandleEnc2;

TIM_Encoder_InitTypeDef enc1;
TIM_Encoder_InitTypeDef enc2;

static const int32_t gear_ratio = 50;
static const int32_t ppt = 64; // Encoder resolution (Number of point per turn)
static const int32_t ct_to_rpm = (60000/64); // = 60000/64

static int32_t enc1_sum = 0, enc2_sum = 0;
static int32_t enc1_diff = 0, enc2_diff = 0;
static uint16_t enc1_now = 32000, enc2_now = 32000;
static uint16_t enc1_old = 32000, enc2_old = 32000;
static int32_t num = 0, den = 0;
static int32_t enc1_rpm = 0, enc2_rpm = 0;
static uint32_t t1_old = 0, t2_old = 0;
static uint32_t t1_now = 0, t2_now = 0;
static uint32_t dt = 0;


int init_encoders(void) {
	int ret = 0;

	// Init. encoders timers
	// TIM3
	TIM_ENC1_CLK_ENABLE();
	TimHandleEnc1.Instance 					= TIM_ENC1;
	TimHandleEnc1.Init.Prescaler         	= 0;
	TimHandleEnc1.Init.Period            	= 0xFFFF;
	TimHandleEnc1.Init.ClockDivision     	= TIM_CLOCKDIVISION_DIV1;
	TimHandleEnc1.Init.CounterMode       	= TIM_COUNTERMODE_UP;
	TimHandleEnc1.Init.RepetitionCounter 	= 0;

	enc1.EncoderMode 	= TIM_ENCODERMODE_TI12;
	enc1.IC1Polarity 	= TIM_ICPOLARITY_RISING; //TIM_ICPolarity_BothEdge
	enc1.IC1Selection 	= TIM_ICSELECTION_DIRECTTI;
	enc1.IC1Prescaler 	= TIM_ICPSC_DIV1;
	enc1.IC1Filter 		= 0x00;
	enc1.IC2Polarity 	= TIM_ICPOLARITY_RISING;
	enc1.IC2Selection 	= TIM_ICSELECTION_DIRECTTI;
	enc1.IC2Prescaler 	= TIM_ICPSC_DIV1;
	enc1.IC2Filter 		= 0x00;

	ret = HAL_TIM_Encoder_Init(&TimHandleEnc1, &enc1);
	if (ret != HAL_OK)
		return -1;

	TIM_ENC1->CNT = enc1_now = enc1_old;
	t1_old = HAL_GetTick();
	ret = HAL_TIM_Encoder_Start(&TimHandleEnc1, TIM_CHANNEL_1 | TIM_CHANNEL_2); // TIM_CHANNEL_ALL
	//ret = HAL_TIM_Encoder_Start_IT(TimHandleEnc1, TIM_CHANNEL_1 | TIM_CHANNEL_2); // TIM_CHANNEL_ALL
	if (ret != HAL_OK)
		return -1;

	// TIM4
	TIM_ENC2_CLK_ENABLE();
	TimHandleEnc2.Instance 					= TIM_ENC2;
	TimHandleEnc2.Init.Prescaler         	= 0;
	TimHandleEnc2.Init.Period            	= 0xFFFF;
	TimHandleEnc2.Init.ClockDivision     	= TIM_CLOCKDIVISION_DIV1;
	TimHandleEnc2.Init.CounterMode       	= TIM_COUNTERMODE_UP;
	TimHandleEnc2.Init.RepetitionCounter 	= 0;

	enc2.EncoderMode 	= TIM_ENCODERMODE_TI12;
	enc2.IC1Polarity 	= TIM_ICPOLARITY_RISING; //TIM_ICPOLARITY_BOTHEDGE;
	enc2.IC1Selection 	= TIM_ICSELECTION_DIRECTTI;
	enc2.IC1Prescaler 	= TIM_ICPSC_DIV1;
	enc2.IC1Filter 		= 0x00;
	enc2.IC2Polarity 	= TIM_ICPOLARITY_RISING; //TIM_ICPOLARITY_BOTHEDGE;
	enc2.IC2Selection 	= TIM_ICSELECTION_DIRECTTI;
	enc2.IC2Prescaler 	= TIM_ICPSC_DIV1;
	enc2.IC2Filter 		= 0x00;

	ret = HAL_TIM_Encoder_Init(&TimHandleEnc2, &enc2);
	if (ret != HAL_OK)
		return -1;

	TIM_ENC2->CNT = enc2_now = enc2_old;
	t2_old = HAL_GetTick();
	ret = HAL_TIM_Encoder_Start(&TimHandleEnc2, TIM_CHANNEL_1 | TIM_CHANNEL_2); // TIM_CHANNEL_ALL
	//ret = HAL_TIM_Encoder_Start_IT(TimHandleEnc2, TIM_CHANNEL_1 | TIM_CHANNEL_2); // TIM_CHANNEL_ALL
	if (ret != HAL_OK)
		return -1;

	enc1_sum = 0, enc2_sum = 0;
	return 0;
}

uint32_t enc1_get_direction(void) {
	return __HAL_TIM_IS_TIM_COUNTING_DOWN(&TimHandleEnc1);
}

uint32_t enc2_get_direction(void) {
	return __HAL_TIM_IS_TIM_COUNTING_DOWN(&TimHandleEnc2);
}

int32_t enc1_get_counts(void) {
	enc1_now = TIM_ENC1->CNT;
	TIM_ENC1->CNT = enc1_old;
	enc1_diff = (int32_t)(enc1_now - enc1_old);
	enc1_sum += enc1_diff;
	return enc1_sum;
}

int32_t enc2_get_counts(void) {
	enc2_now = TIM_ENC2->CNT;
	TIM_ENC2->CNT = enc2_old;
	enc2_diff = (int32_t)(enc2_now - enc2_old);
	enc2_sum += enc2_diff;
	return enc2_sum;
}

int32_t enc1_get_rpm(void) {
	enc1_now = TIM_ENC1->CNT;
	TIM_ENC1->CNT = enc1_old;
	t1_now = HAL_GetTick();
	enc1_diff = enc1_now - enc1_old;

	dt = t1_now - t1_old;
	t1_old = t1_now;

    den = (int32_t)((int32_t)dt * gear_ratio);
    num = (enc1_diff * 60000) / ppt;
    enc1_rpm = num/den;
    return enc1_rpm;
}

int32_t enc2_get_rpm(void) {
	enc2_now = TIM_ENC2->CNT;
	TIM_ENC2->CNT = enc2_old;
	t2_now = HAL_GetTick();
	enc2_diff = enc2_now - enc2_old;

	dt = t2_now - t2_old;
	t2_old = t2_now;

    den = (int32_t)((int32_t)dt * gear_ratio);
    num = (enc2_diff * 60000) / ppt;
    enc2_rpm = num/den;
    return enc2_rpm;
}

void enc1_get_cts_and_rpm(int32_t* cts, int32_t* rpm) {
	enc1_now = TIM_ENC1->CNT;
	TIM_ENC1->CNT = enc1_old;
	t1_now = HAL_GetTick();
	(*cts) = enc1_now - enc1_old;

	dt = t1_now - t1_old;
	t1_old = t1_now;

    den = (int32_t)((int32_t)dt * gear_ratio);
    num = ((*cts) * 60000) / ppt;
    (*rpm) = num/den;
}

void enc2_get_cts_and_rpm(int32_t* cts, int32_t* rpm) {
	enc2_now = TIM_ENC2->CNT;
	TIM_ENC2->CNT = enc2_old;
	t2_now = HAL_GetTick();
	(*cts) = enc2_now - enc2_old;

	dt = t2_now - t2_old;
	t2_old = t2_now;

    den = (int32_t)((int32_t)dt * gear_ratio);
    num = ((*cts) * 60000) / ppt;
    (*rpm) = num/den;
}

void enc_get_cts_and_rpm(int32_t* cts1, int32_t* rpm1, int32_t* cts2, int32_t* rpm2) {
	enc1_now = TIM_ENC1->CNT;
	enc2_now = TIM_ENC2->CNT;
	TIM_ENC1->CNT = enc1_old;
	TIM_ENC2->CNT = enc2_old;

	t1_now = HAL_GetTick();

	(*cts1) = (int32_t)(enc1_now - enc1_old);
	(*cts2) = (int32_t)(enc2_now - enc2_old);

	dt = t1_now - t1_old;
	t1_old = t1_now;

    den = (int32_t)((int32_t)dt * gear_ratio);
    num = ((*cts1) * 60000) / ppt;
    enc1_rpm = num/den;
    num = ((*cts2) * 60000) / ppt;
    enc2_rpm = num/den;

    (*rpm1) = (int16_t)enc1_rpm;
    (*rpm2) = (int16_t)enc2_rpm;
}
