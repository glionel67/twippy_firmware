/**
 * \file nmea.h
 * \brief NMEA GPS message definition
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 */

#pragma once

#include <stdint.h>

typedef struct NmeaGgaMsg_s {
    char[6] msgId; /**  Message ID */
    uint8_t hour; /** UTC time hour */
    uint8_t minute; /** UTC time minute */
    float second; /** UTC time second */
    float latitude; /** Latitude [degrees] */
    char ns; /** North/South indicator */
    float longitude; /** Longitude [degrees] */
    char ew; /** East/West indicator */
    uint8_t quality; /** Quality indicator for position fix */
    uint8_t nSatellites; /** Number of satellites used (range: 0-12) */
    float hdop; /** Horizontal Dilution of Precision */
    float altitude; /** Altitude above mean sea level */
    float geoidSeparation; /** Geoid separation: difference between ellipsoid and mean sea level */
    float diffAge; /** Age of differential corrections (blank when DGPS is not used) [s] */
    uint8_t diffStation; /** D of station providing differential corrections (blank when DGPS is not used) */
    uint8_t checksum; /** Checksum */
} NmeaGgaMsg_t;

typedef struct NmeaGllMsg_s {
    char[6] msgId; /**  Message ID */
    float latitude; /** Latitude [degrees] */
    char ns; /** North/South indicator */
    float longitude; /** Longitude [degrees] */
    char ew; /** East/West indicator */
    uint8_t hour; /** UTC time hour */
    uint8_t minute; /** UTC time minute */
    float second; /** UTC time second */
    char status; /** V = Data invalid or receiver warning, A = Data valid */
    char posMode; /** Positioning mode */
    uint8_t checksum; /** Checksum */
} NmeaGgaMsg_t;

typedef struct NmeaGnsMsg_s {
    char[6] msgId; /**  Message ID */
    uint8_t hour; /** UTC time hour */
    uint8_t minute; /** UTC time minute */
    float second; /** UTC time second */
    float latitude; /** Latitude [degrees] */
    char ns; /** North/South indicator */
    float longitude; /** Longitude [degrees] */
    char ew; /** East/West indicator */
    char posMode; /** Positioning mode */
    uint8_t nSatellites; /** Number of satellites used (range: 0-12) */
    float hdop; /** Horizontal Dilution of Precision */
    float altitude; /** Altitude above mean sea level */
    float geoidSeparation; /** Geoid separation: difference between ellipsoid and mean sea level */
    float diffAge; /** Age of differential corrections (blank when DGPS is not used) [s] */
    uint8_t diffStation; /** D of station providing differential corrections (blank when DGPS is not used) */
    char navStatus; /** Navigational status indicator (V = Equipment is not providing navigational status information) */
    uint8_t checksum; /** Checksum */
} NmeaGnsMsg_t;

typedef struct NmeaGsaMsg_s {
    char[6] msgId; /**  Message ID */
    char opMode; /** Operation mode */
    uint8_t navMode; /** Navigation mode 1=Fix not available, 2=2D fix, 3=3D fix */
    uint8_t satNum[12]; /** Satellite number */
    float pdop; /** Position dilution of precision */
    float hdop; /** Horizontal dilution of precision */
    float vdop; /** Vertical dilution of precision */
    uint8_t systemId; /** NMEA defined GNSS System ID */
    uint8_t checksum; /** Checksum */
} NmeaGsaMsg_t;

typedef struct NmeaGstMsg_s {
    char[6] msgId; /**  Message ID */
    uint8_t hour; /** UTC time hour */
    uint8_t minute; /** UTC time minute */
    float second; /** UTC time second */
    float rangeRms; /** RMS value of the standard deviation of the ranges */
    float stdMajor; /** Standard deviation of semi-major axis */
    float stdMinor; /** Standard deviation of semi-minor axis */
    float orient; /** Orientation of semi-major axis [deg] */
    float stdLat; /** Standard deviation of latitude error */
    float stdLong; /** Standard deviation of longitude error */
    float stdAlt; /** Standard deviation of altitude error */
    uint8_t checksum; /** Checksum */
} NmeaGstMsg_t;

typedef struct NmeaRmcMsg_s {
    char[6] msgId; /**  Message ID */
    uint8_t hour; /** UTC time hour */
    uint8_t minute; /** UTC time minute */
    char status; /** Status, V = Navigation receiver warning, A = Data valid */
    float latitude; /** Latitude */
    char ns; /** North/South indicator */
    float longitude; /** Longitude */
    char ew; /** East/West indicator */
    float speedOverGround; /** Speed over ground [knot s] */
    float courseOverGroung; /** Course over ground [degrees] */
    char date[6]; /** Date in day, month, year format */
    float magVar; /** Magnetic variation value [degrees] */
    char magVarEW; /** Magnetic variation E/W indicator */
    char posMode; /** Mode Indicator */
    char navStatus; /** Navigational status indicator (V = Equipment is not providing navigational status information) */
    uint8_t checksum; /** Checksum */
} NmeaRmcMsg_t;