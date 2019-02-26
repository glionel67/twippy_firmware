/**
 * \file nmea.h
 * \brief NMEA GPS message definition
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct NmeaGgaMsg_s {
    char msgId[6]; /**  Message ID */
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
    char msgId[6]; /**  Message ID */
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
} NmeaGllMsg_t;

typedef struct NmeaGnsMsg_s {
    char msgId[6]; /**  Message ID */
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
    char msgId[6]; /**  Message ID */
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
    char msgId[6]; /**  Message ID */
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
    char msgId[6]; /**  Message ID */
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

/**
 * \enum NmeaDecodeState_e
 * \brief Decoder state
 */
typedef enum {
    NMEA_DECODE_START = 0,
    NMEA_DECODE_ADDR1,
    NMEA_DECODE_ADDR2,
    NMEA_DECODE_ADDR3,
    NMEA_DECODE_ADDR4,
    NMEA_DECODE_ADDR5,
    NMEA_DECODE_ADDR6,
    NMEA_DECODE_PAYLOAD,
    NMEA_DECODE_CHKSUM1,
    NMEA_DECODE_CHKSUM2,
    NMEA_DECODE_END1,
    NMEA_DECODE_END2
} NmeaDecodeState_e;

#define NMEA_START_CHAR '$'
#define NMEA_ADDR1 'G'
#define NMEA_ADDR6 ','
#define NMEA_CHKSUM '*'
#define NMEA_END1 '\r'
#define NMEA_END2 '\n'

#define NMEA_MSG_ID_LEN 6

void decodeInit(void);

bool decodeMsg(char* _msg);

bool decodeGGA(char* _msg);
bool decodeGLL(char* _msg);
bool decodeGNS(char* _msg);
bool decodeGSA(char* _msg);
bool decodeGST(char* _msg);
bool decodeRMC(char* _msg);

bool decodeUtcTime(char* msg, uint8_t* h, uint8_t* m, float* s);
bool decodeLatitude(char* msg, float* latDeg);
bool decodeLongitude(char* msg, float* lonDeg);
bool decodeLatDir(char* msg, char* latDir);
bool decodeLonDir(char* msg, char* lonDir);
bool decodeAltitude(char* msg, float* alt);
bool decodeQuality(char* msg, uint8_t* qua);
bool decodeNumSat(char* msg, uint8_t* nSat);
bool decodeHdop(char* msg, float* hdop);
bool decodeGeoidSep(char* msg, float* geoSep);
bool decodeDiffAge(char* msg, float* diffAge);
bool decodeDiffSta(char* msg, uint8_t* diffSta);

bool nmeaParseChar(char _c);

//int computeChecksum(void);
