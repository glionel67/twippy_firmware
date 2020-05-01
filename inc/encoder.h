/**
 * \file encoder.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief Encoder functions used to measure the motor/wheel speed
 */

#pragma once

// -------------------------------------------------------------------------- //
// --- Includes
// -------------------------------------------------------------------------- //
#include <stdint.h>

// -------------------------------------------------------------------------- //
// --- Defines
// -------------------------------------------------------------------------- //

#define N_ENCODERS (2) // = N_MOTORS

// -------------------------------------------------------------------------- //
// --- Structs/enums
// -------------------------------------------------------------------------- //

/**
 * \struct Encoder_t
 * \brief Structure containing an encoder data (counts and rpm)
 */
typedef struct Encoder_s
{
    int16_t counts; // Number of encoder impulsions
    int16_t rpm; // Measured speed [rpm]
} Encoder_t;

/**
 * \struct Encoders_t
 * \brief Structure containing the stamped encoders data (counts and rpm)
 */
typedef struct Encoders_s
{
	float timestamp; // in [s]
    //uint32_t timestamp; // in [ms]
    Encoder_t encoders[N_ENCODERS];
} Encoders_t;

// -------------------------------------------------------------------------- //
// --- Prototypes
// -------------------------------------------------------------------------- //

/**
 * \fn init_encoders
 * \brief Initialize the left and right encoders
 */
int init_encoders(void);

/**
 * \fn enc1_get_direction
 * \brief Get the encoder direction (forward/reverse)
 */
uint32_t enc1_get_direction(void);
uint32_t enc2_get_direction(void);

/**
 * \fn encoder_get_counts
 * \brief Get the number of counts from the left and right encoders
 */
//void encoder_get_counts(int32_t* cnt1, int32_t* cnt2);
void encoder_get_counts(int16_t* cnt1, int16_t* cnt2);