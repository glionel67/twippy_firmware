/**
 * \file encoder.c
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief Encoder functions used to measure the motor/wheel speed
 */

#include "encoder.h"
#include "main.h"

// Timer handler declaration
static TIM_HandleTypeDef TimHandleEnc1;
static TIM_HandleTypeDef TimHandleEnc2;

static uint16_t enc1_now = 32000, enc2_now = 32000;
static uint16_t enc1_old = 32000, enc2_old = 32000;

int init_encoders(void) 
{
    // Init. encoders timers

    // TIM3
    TIM_ENC1_CLK_ENABLE();
    TimHandleEnc1.Instance                  = TIM_ENC1;
    TimHandleEnc1.Init.Prescaler            = 0;
    TimHandleEnc1.Init.Period               = 0xFFFF;
    TimHandleEnc1.Init.ClockDivision        = TIM_CLOCKDIVISION_DIV1;
    TimHandleEnc1.Init.CounterMode          = TIM_COUNTERMODE_UP;
    TimHandleEnc1.Init.RepetitionCounter    = 0;

    TIM_Encoder_InitTypeDef enc1;
    enc1.EncoderMode    = TIM_ENCODERMODE_TI12;
    enc1.IC1Polarity    = TIM_ICPOLARITY_RISING; //TIM_ICPolarity_BothEdge
    enc1.IC1Selection   = TIM_ICSELECTION_DIRECTTI;
    enc1.IC1Prescaler   = TIM_ICPSC_DIV1;
    enc1.IC1Filter      = 0x00;
    enc1.IC2Polarity    = TIM_ICPOLARITY_RISING;
    enc1.IC2Selection   = TIM_ICSELECTION_DIRECTTI;
    enc1.IC2Prescaler   = TIM_ICPSC_DIV1;
    enc1.IC2Filter      = 0x00;

    if (HAL_OK != HAL_TIM_Encoder_Init(&TimHandleEnc1, &enc1))
        return NOK;

    TIM_ENC1->CNT = enc1_now = enc1_old;
    if (HAL_OK != HAL_TIM_Encoder_Start(&TimHandleEnc1, 
                    TIM_CHANNEL_1 | TIM_CHANNEL_2))
        return NOK;

    // TIM4
    TIM_ENC2_CLK_ENABLE();
    TimHandleEnc2.Instance                  = TIM_ENC2;
    TimHandleEnc2.Init.Prescaler            = 0;
    TimHandleEnc2.Init.Period               = 0xFFFF;
    TimHandleEnc2.Init.ClockDivision        = TIM_CLOCKDIVISION_DIV1;
    TimHandleEnc2.Init.CounterMode          = TIM_COUNTERMODE_UP;
    TimHandleEnc2.Init.RepetitionCounter    = 0;

    TIM_Encoder_InitTypeDef enc2;
    enc2.EncoderMode    = TIM_ENCODERMODE_TI12;
    enc2.IC1Polarity    = TIM_ICPOLARITY_RISING; //TIM_ICPOLARITY_BOTHEDGE;
    enc2.IC1Selection   = TIM_ICSELECTION_DIRECTTI;
    enc2.IC1Prescaler   = TIM_ICPSC_DIV1;
    enc2.IC1Filter      = 0x00;
    enc2.IC2Polarity    = TIM_ICPOLARITY_RISING; //TIM_ICPOLARITY_BOTHEDGE;
    enc2.IC2Selection   = TIM_ICSELECTION_DIRECTTI;
    enc2.IC2Prescaler   = TIM_ICPSC_DIV1;
    enc2.IC2Filter      = 0x00;

    if (HAL_OK != HAL_TIM_Encoder_Init(&TimHandleEnc2, &enc2))
        return NOK;

    TIM_ENC2->CNT = enc2_now = enc2_old;
    if (HAL_OK != HAL_TIM_Encoder_Start(&TimHandleEnc2, 
                    TIM_CHANNEL_1 | TIM_CHANNEL_2))
        return NOK;
    
    return OK;
} // init_encoders

uint32_t enc1_get_direction(void)
{
    return !__HAL_TIM_IS_TIM_COUNTING_DOWN(&TimHandleEnc1);
} // enc1_get_direction

uint32_t enc2_get_direction(void)
{
    return !__HAL_TIM_IS_TIM_COUNTING_DOWN(&TimHandleEnc2);
} // enc2_get_direction

//void encoder_get_counts(int32_t* cnt1, int32_t* cnt2)
void encoder_get_counts(int16_t* cnt1, int16_t* cnt2)
{
    // Read ticks and reset old values
    enc1_now = TIM_ENC1->CNT;
    enc2_now = TIM_ENC2->CNT;
    TIM_ENC1->CNT = enc1_old;
    TIM_ENC2->CNT = enc2_old;
    // Compute difference
    //*cnt1 = (int32_t)(enc1_now - enc1_old);
    //*cnt2 = (int32_t)(enc2_now - enc2_old);
    *cnt1 = (int16_t)(enc1_now - enc1_old);
    *cnt2 = (int16_t)(enc2_now - enc2_old);
} // encoder_get_counts