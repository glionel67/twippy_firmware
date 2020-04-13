/**
 * \file gpio.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief GPIO initialization
 */

#pragma once

#include <stdint.h>

/**
 * \fn init_gpios
 * \brief Initialize the GPIO
 * \return OK if success, NOK if error
 */
int init_gpios(void);
