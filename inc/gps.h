/**
 * \file gps.h
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 * \brief GPS parser
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define GPS_TRAME_MAX_SIZE 100

typedef struct GpsData_s
{
    uint8_t heure, minute;
    float seconde;
    float latitude; /** [degrees] */
    char ns; /** 'N' or 'S' */
    float longitude; /** [degrees] */
    char ew; /** 'E' or 'W' */
    float altitude; /** [m] */
    float speedOverGround; /** [knot s] */
    float courseOverGroung; /** [degrees] */
} GpsData_t;

/**
 * \fn gps_init
 * \brief Initialize the GPS
 * \return OK if success, NOK otherwise
 */
int gps_init(void);
bool gps_is_init(void);
bool gps_is_running(void);

int gps_start(void);
int gps_stop(void);

/**
 * \fn gps_task
 * \brief GPS parser task to receive frames from uart and parse them
 */
void gps_task(void* _params);