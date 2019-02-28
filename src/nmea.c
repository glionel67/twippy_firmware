/**
 * \file nmea.c
 * \brief NMEA GPS message definition
 * \author Lionel GENEVE
 * \date 22/02/2019
 * \version 1.0
 */

#include "nmea.h"

// C lib
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// #ifdef DEBUG
// #define PRINT_DEBUG(__msg__) printf("%s\r\n", __msg__);
// #elif
// #define PRINT_DEBUG(__msg__)
// #endif

#define DEBUG

static uint8_t decodeState = 0;
static char checksumField[2] = { 0, };
static uint8_t checksum = 0;
static char buffer[128] = { 0, };
static uint8_t payloadSize = 0;
static const uint8_t payloadThresh = 127;

void decodeInit(void)
{
    decodeState = NMEA_DECODE_START;
    checksumField[0] = checksumField[1] = 0;
    checksum = 0;
    payloadSize = 0;
}

bool decodeMsg(char* _msg)
{
    //printf("decodeMsg: %s\r\n", _msg);

    bool ret = false;
    if (strncmp((const char*)(_msg+3), "GGA", 3) == 0) {
        ret = decodeGGA(_msg);
    }
    else if (strncmp((const char*)(_msg+3), "GLL", 3) == 0) {
        ret = decodeGLL(_msg);
    }
    else if (strncmp((const char*)(_msg+3), "GNS", 3) == 0) {
        ret = decodeGNS(_msg);
    }
    else if (strncmp((const char*)(_msg+3), "GSA", 3) == 0) {
        ret = decodeGSA(_msg);
    }
    else if (strncmp((const char*)(_msg+3), "GST", 3) == 0) {
        ret = decodeGST(_msg);
    }
    else if (strncmp((const char*)(_msg+3), "RMC", 3) == 0) {
        ret = decodeRMC(_msg);
    }
    else {
        printf("Unknown NMEA GPS message\r\n");
        ret = false;
    }

    return ret;
}

bool decodeGGA(char* _msg)
{
    printf("decodeGGA: %s\r\n", _msg);

    uint8_t i = 0;
    bool ret = true;
    NmeaGgaMsg_t ggaMsg;
    char* tmp = strtok((char*)_msg, (const char*)',');

    while (tmp != NULL) {
        switch (i) {
            case 0: // Message ID
            //memcpy((void*)ggaMsg.msgId, (const void*)tmp, NMEA_MSG_ID_LEN);
            strncpy((char*)ggaMsg.msgId,(const char*)tmp, NMEA_MSG_ID_LEN);
            break;
            case 1: // UTC time
            ret &= decodeUtcTime(tmp, &ggaMsg.hour, &ggaMsg.minute, &ggaMsg.second);
            break;
            case 2: // Latitude
            ret &= decodeLatitude(tmp, &ggaMsg.latitude);
            break;
            case 3: // North/South indicator
            ret &= decodeLatDir(tmp, &ggaMsg.ns);
            break;
            case 4: // Longitude
            ret &= decodeLongitude(tmp, &ggaMsg.longitude);
            break;
            case 5: // East/West indicator
            ret &= decodeLonDir(tmp, &ggaMsg.ew);
            break;
            case 6: // Quality
            ret &= decodeQuality(tmp, &ggaMsg.quality);
            break;
            case 7: // Number of satellites
            ret &= decodeNumSat(tmp, &ggaMsg.nSatellites);
            break;
            case 8: // HDOP
            ret &= decodeHdop(tmp, &ggaMsg.hdop);
            break;
            case 9: // Altitude
            ret &= decodeAltitude(tmp, &ggaMsg.altitude);
            break;
            case 11: // Geoid separation
            ret &= decodeGeoidSep(tmp, &ggaMsg.geoidSeparation);
            break;
            case 13: // Age of differential corrections
            decodeDiffAge(tmp, &ggaMsg.diffAge);
            break;
            case 14: // ID of station providing differential corrections
            decodeDiffSta(tmp, &ggaMsg.diffStation);
            break;
            default:
            break;
        }

        tmp = strtok((char*)NULL, (const char*)',');
        i++;
    }

    return ret;
} // decodeGGA

bool decodeGLL(char* _msg)
{
    printf("decodeGLL: %s\r\n", _msg);

    uint8_t i = 0;
    bool ret = true;
    NmeaGllMsg_t gllMsg;
    char* tmp = strtok((char*)_msg, (const char*)',');

    while (tmp != NULL) {
        switch (i) {
            case 0: // Message ID
            //memcpy((void*)gllMsg.msgId, (const void*)tmp, NMEA_MSG_ID_LEN);
            strncpy((char*)gllMsg.msgId,(const char*)tmp, NMEA_MSG_ID_LEN);
            break;
            case 1: // Latitude
            ret &= decodeLatitude(tmp, &gllMsg.latitude);
            break;
            case 2: // North/South indicator
            ret &= decodeLatDir(tmp, &gllMsg.ns);
            break;
            case 3: // Longitude
            ret &= decodeLongitude(tmp, &gllMsg.longitude);
            break;
            case 4: // East/West indicator
            ret &= decodeLonDir(tmp, &gllMsg.ew);
            break;
            case 5: // UTC time
            ret &= decodeUtcTime(tmp, &gllMsg.hour, &gllMsg.minute, &gllMsg.second);
            break;
            case 6: // Status
            ret &= decodeChar(tmp, &gllMsg.status);
            break;
            case 7: // Positioning mode
            ret &= decodeChar(tmp, &gllMsg.posMode);
            break;
            default:
            break;
        }

        tmp = strtok((char*)NULL, (const char*)',');
        i++;
    }

    return ret;
} // decodeGLL

bool decodeGNS(char* _msg)
{
    printf("decodeGNS: %s\r\n", _msg);
    uint8_t i = 0;
    bool ret = true;
    NmeaGnsMsg_t gnsMsg;
    char* tmp = strtok((char*)_msg, (const char*)',');

    while (tmp != NULL) {
        switch (i) {
            case 0: // Message ID
            strncpy((char*)gnsMsg.msgId,(const char*)tmp, NMEA_MSG_ID_LEN);
            break;
            case 1: // UTC time
            ret &= decodeUtcTime(tmp, &gnsMsg.hour, &gnsMsg.minute, &gnsMsg.second);
            break;
            case 2: // Latitude
            ret &= decodeLatitude(tmp, &gnsMsg.latitude);
            break;
            case 3: // North/South indicator
            ret &= decodeLatDir(tmp, &gnsMsg.ns);
            break;
            case 4: // Longitude
            ret &= decodeLongitude(tmp, &gnsMsg.longitude);
            break;
            case 5: // East/West indicator
            ret &= decodeLonDir(tmp, &gnsMsg.ew);
            break;
            case 6: // Positioning mode
            ret &= decodeChar(tmp, &gnsMsg.posMode);
            break;
            case 7: // Number of satellites
            ret &= decodeNumSat(tmp, &gnsMsg.nSatellites);
            break;
            case 8: // HDOP
            ret &= decodeHdop(tmp, &gnsMsg.hdop);
            break;
            case 9: // Altitude
            ret &= decodeAltitude(tmp, &gnsMsg.altitude);
            break;
            case 10: // Geoid separation
            ret &= decodeGeoidSep(tmp, &gnsMsg.geoidSeparation);
            break;
            case 11: // Age of differential corrections
            decodeDiffAge(tmp, &gnsMsg.diffAge);
            break;
            case 12: // ID of station providing differential corrections
            decodeDiffSta(tmp, &gnsMsg.diffStation);
            break;
            case 13: // Navigational status indicator
            ret &= decodeChar(tmp, &gnsMsg.navStatus);
            break;
            default:
            break;
        }

        tmp = strtok((char*)NULL, (const char*)',');
        i++;
    }

    return ret;
} // decodeGNS

bool decodeGSA(char* _msg)
{
    printf("decodeGSA: %s\r\n", _msg);
    uint8_t i = 0;
    uint8_t n = 0;
    bool ret = true;
    NmeaGsaMsg_t gsaMsg;
    char* tmp = strtok((char*)_msg, (const char*)',');

    while (tmp != NULL) {
        switch (i) {
            case 0: // Message ID
            strncpy((char*)gsaMsg.msgId,(const char*)tmp, NMEA_MSG_ID_LEN);
            break;
            case 1: // Operation mode
            ret &= decodeChar(tmp, &gsaMsg.opMode);
            break;
            case 2: // Navigation mode
            ret &= decodeUint8(tmp, &gsaMsg.navMode);
            break;
            case 3: // Satellite number [x12]
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
            case 13:
            case 14:
            decodeUint8(tmp, &gsaMsg.satNum[n]);
            n++;
            break;
            case 15: // Position dilution of precision
            ret &= decodeFloat(tmp, &gsaMsg.pdop);
            break;
            case 16: // Horizontal dilution of precision
            ret &= decodeFloat(tmp, &gsaMsg.hdop);
            break;
            case 17: // Vertical dilution of precision
            ret &= decodeFloat(tmp, &gsaMsg.vdop);
            break;
            case 18: // Vertical dilution of precision
            ret &= decodeUint8(tmp, &gsaMsg.systemId);
            break;
            default:
            break;
        }

        tmp = strtok((char*)NULL, (const char*)',');
        i++;
    }

    return ret;
} // decodeGSA

bool decodeGST(char* _msg)
{
    printf("decodeGST: %s\r\n", _msg);

    uint8_t i = 0;
    bool ret = true;
    NmeaGstMsg_t gstMsg;
    char* tmp = strtok((char*)_msg, (const char*)',');

    while (tmp != NULL) {
        switch (i) {
            case 0: // Message ID
            strncpy((char*)gstMsg.msgId,(const char*)tmp, NMEA_MSG_ID_LEN);
            break;
            case 1: // UTC time
            ret &= decodeUtcTime(tmp, &gstMsg.hour, &gstMsg.minute, &gstMsg.second);
            break;
            case 2: // RMS value of the standard deviation of the ranges
            ret &= decodeFloat(tmp, &gstMsg.rangeRms);
            break;
            case 3: // Standard deviation of semi-major axis
            ret &= decodeFloat(tmp, &gstMsg.stdMajor);
            break;
            case 4: // Standard deviation of semi-minor axis
            ret &= decodeFloat(tmp, &gstMsg.stdMinor);
            break;
            case 5: // Orientation of semi-major axis
            ret &= decodeFloat(tmp, &gstMsg.orient);
            break;
            case 6: // Standard deviation of latitude error
            ret &= decodeFloat(tmp, &gstMsg.stdLat);
            break;
            case 7: // Standard deviation of longitude error
            ret &= decodeFloat(tmp, &gstMsg.stdLon);
            break;
            case 8: // Standard deviation of altitde error
            ret &= decodeFloat(tmp, &gstMsg.stdAlt);
            break;
            default:
            break;
        }

        tmp = strtok((char*)NULL, (const char*)',');
        i++;
    }

    return ret;
} // decodeGST

bool decodeRMC(char* _msg)
{
    printf("decodeRMC: %s\r\n", _msg);
    uint8_t i = 0;
    bool ret = true;
    NmeaRmcMsg_t rmcMsg;
    char* tmp = strtok((char*)_msg, (const char*)',');

    while (tmp != NULL) {
        switch (i) {
            case 0: // Message ID
            //memcpy((void*)rmcMsg.msgId, (const void*)tmp, NMEA_MSG_ID_LEN);
            strncpy((char*)rmcMsg.msgId,(const char*)tmp, NMEA_MSG_ID_LEN);
            break;
            case 1: // UTC time
            ret &= decodeUtcTime(tmp, &rmcMsg.hour, &rmcMsg.minute, &rmcMsg.second);
            break;
            case 2: // Status
            ret &= decodeStatus(tmp, &rmcMsg.status);
            break;
            case 3: // Latitude
            ret &= decodeLatitude(tmp, &rmcMsg.latitude);
            break;
            case 4: // North/South indicator
            ret &= decodeLatDir(tmp, &rmcMsg.ns);
            break;
            case 5: // Longitude
            ret &= decodeLongitude(tmp, &rmcMsg.longitude);
            break;
            case 6: // East/West indicator
            ret &= decodeLonDir(tmp, &rmcMsg.ew);
            break;
            case 7: // Speed over ground
            ret &= decodeSpeedOverGround(tmp, &rmcMsg.speedOverGround);
            break;
            case 8: // Course over ground
            ret &= decodeCourseOverGround(tmp, &rmcMsg.courseOverGroung);
            break;
            case 9: // Date (day, month, year)
            strncpy((char*)rmcMsg.date,(const char*)tmp, NMEA_MSG_DATE_LEN);
            break;
            case 10: // Magnetic variation value
            ret &= decodeFloat(tmp, &rmcMsg.magVar);
            break;
            case 11: // Magnetic variation E/W indicator
            ret &= decodeChar(tmp, &rmcMsg.magVarEW);
            break;
            case 12: // Mode Indicator
            ret &= decodeChar(tmp, &rmcMsg.posMode);
            break;
            case 13: // Navigational status indicator
            ret &= decodeChar(tmp, &rmcMsg.navStatus);
            break;
            default:
            break;
        }

        tmp = strtok((char*)NULL, (const char*)',');
        i++;
    }

    return ret;
}

bool decodeUtcTime(char* msg, uint8_t* h, uint8_t* m, float* s)
{
    char hour[2] = {0, };
    char minute[2] = {0, };
    char second[5] = {0, };
    (*h) = (*m) = 255;
    (*s) = -1.f;

    if (msg == NULL)
        return false;

    // Hour
    if (strncpy(hour, (const char*)msg, 2) != 0)
        *h = (uint8_t)atoi(hour);
    else
        return false;

#ifdef DEBUG
    printf("decodeUtcTime: h=%d,", *h);
#endif

    // Minute
    if (strncpy(minute, (const char*)(msg+2), 2) != 0)
        *m = (uint8_t)atoi(minute);
    else
        return false;
#ifdef DEBUG
    printf("decodeUtcTime: m=%d,", *m);
#endif

    // Second
    if (strncpy(second, (const char*)(msg+4), 5) != 0)
        *s = atof((const char*)second);
    else
        return false;
#ifdef DEBUG
    printf("decodeUtcTime: s=%f\r\n", *s);
#endif

    return true;
}

bool decodeLatitude(char* msg, float* latDeg)
{
    char deg[2] = {0, };
    char min[7] = {0, };
    float degrees = 0.f, minutes = 0.f;

    if (msg == NULL)
        return false;

    if (strncpy(deg, (const char*)msg, 2) != 0)
        degrees = atof((const char*)deg);
    else
        return false;

    if (strncpy(min, (const char*)(msg+2), 7) != 0)
        minutes = atof((const char*)min);
    else
        return false;

    *latDeg = degrees + (minutes / 60.f);
#ifdef DEBUG
    printf("decodeLatitude=%f [deg]\r\n", *latDeg);
#endif

    return true;
}

bool decodeLongitude(char* msg, float* lonDeg)
{
    char deg[3] = {0, };
    char min[7] = {0, };
    float degrees = 0.f, minutes = 0.f;

    if (msg == NULL)
        return false;

    if (strncpy(deg, (const char*)msg, 3) != 0)
        degrees = atof((const char*)deg);
    else
        return false;

    if (strncpy(min, (const char*)(msg+3), 7) != 0)
        minutes = atof((const char*)min);
    else
        return false;

    *lonDeg = degrees + (minutes / 60.f);
#ifdef DEBUG
    printf("decodeLongitude=%f [deg]\r\n", *lonDeg);
#endif

    return true;
}

bool decodeLatDir(char* msg, char* latDir)
{
    if (msg == 0)
        return false;

    latDir[0] = (char)msg[0];
#ifdef DEBUG
    printf("decodeLatDir=%c\r\n", latDir[0]);
#endif

    return true;
}

bool decodeLonDir(char* msg, char* lonDir)
{
    if (msg == 0)
        return false;

    lonDir[0] = (char)msg[0];
#ifdef DEBUG
    printf("decodeLonDir=%c\r\n", lonDir[0]);
#endif

    return true;
}

bool decodeAltitude(char* msg, float* alt)
{
    if (msg == NULL)
        return false;

    *alt = atof((const char*)msg);
#ifdef DEBUG
    printf("decodeAltitude=%f [m]\r\n", *alt);
#endif

    return true;
}

bool decodeQuality(char* msg, uint8_t* qua)
{
    if (msg == 0)
        return false;

    qua[0] = (uint8_t)msg[0];
#ifdef DEBUG
    printf("decodeQuality=%d\r\n", qua[0]);
#endif

    return true;
}

bool decodeNumSat(char* msg, uint8_t* nSat)
{
    if (msg == 0)
        return false;

    nSat[0] = (uint8_t)msg[0];
#ifdef DEBUG
    printf("decodeNumSat=%d\r\n", nSat[0]);
#endif

    return true;
}

bool decodeHdop(char* msg, float* hdop)
{
    if (msg == NULL)
        return false;

    *hdop = atof((const char*)msg);
#ifdef DEBUG
    printf("decodeHdop=%f\r\n", *hdop);
#endif

    return true;
}

bool decodeGeoidSep(char* msg, float* geoSep)
{
    if (msg == NULL)
        return false;

    *geoSep = atof((const char*)msg);
#ifdef DEBUG
    printf("decodeGeoidSep=%f\r\n", *geoSep);
#endif

    return true;
}

bool decodeDiffAge(char* msg, float* diffAge)
{
    if (msg == NULL)
        return false;

    *diffAge = atof((const char*)msg);
#ifdef DEBUG
    printf("decodeDiffAge=%f\r\n", *diffAge);
#endif

    return true;
}

bool decodeDiffSta(char* msg, uint8_t* diffSta)
{
    if (msg == 0)
        return false;

    diffSta[0] = (uint8_t)msg[0];
#ifdef DEBUG
    printf("decodeDiffSta=%d\r\n", diffSta[0]);
#endif

    return true;
}

bool decodeStatus(char* msg, char* stat)
{
    if (msg == 0)
        return false;

    stat[0] = (char)msg[0];
#ifdef DEBUG
    printf("decodeStatus=%c\r\n", stat[0]);
#endif

    return true;
}

bool decodeSpeedOverGround(char* msg, float* speed)
{
    if (msg == NULL)
        return false;

    *speed = atof((const char*)msg);
#ifdef DEBUG
    printf("decodeSpeedOverGround=%f\r\n", *speed);
#endif

    return true;
}

bool decodeCourseOverGround(char* msg, float* cog)
{
    if (msg == NULL)
        return false;

    *cog = atof((const char*)msg);
#ifdef DEBUG
    printf("decodeCourseOverGround=%f\r\n", *cog);
#endif

    return true;
}

bool decodeFloat(char* msg, float* f)
{
    if (msg == NULL) {
        *f = 0.f;
        return false;
    }

    *f = atof((const char*)msg);
#ifdef DEBUG
    printf("decodeFloat=%f\r\n", *f);
#endif

    return true;
}

bool decodeChar(char* msg, char* c)
{
    if (msg == 0) {
        *c = 0;
        return false;
    }

    c[0] = (char)msg[0];
#ifdef DEBUG
    printf("decodeChar=%c\r\n", c[0]);
#endif

    return true;
}

bool decodeUint8(char* msg, uint8_t* n)
{
    if (msg == 0) {
        *n = 0;
        return false;
    }

    n[0] = (uint8_t)msg[0];
#ifdef DEBUG
    printf("decodeUint8=%d\r\n", n[0]);
#endif

    return true;
}

bool nmeaParseChar(char _c)
{
    bool ret = false;

    //printf("%c", _c);

    switch (decodeState) {
        case NMEA_DECODE_START:
        {
            if (_c == NMEA_START_CHAR) {
                decodeState = NMEA_DECODE_ADDR1;
                buffer[payloadSize++] = _c;
            }
            break;
        }
        case NMEA_DECODE_ADDR1:
        {
            if (_c == NMEA_ADDR1) {
                decodeState = NMEA_DECODE_ADDR2;
                checksum = _c; // Bytewise or
                buffer[payloadSize] = _c;
                payloadSize++;
            }
            else {
                decodeInit();
            }
            break;
        }
        case NMEA_DECODE_ADDR2:
        {
            decodeState = NMEA_DECODE_ADDR3;
            checksum ^= _c; // Bytewise or
            buffer[payloadSize] = _c;
            payloadSize++;
            break;
        }
        case NMEA_DECODE_ADDR3:
        {
            decodeState = NMEA_DECODE_ADDR4;
            checksum ^= _c; // Bytewise or
            buffer[payloadSize] = _c;
            payloadSize++;
            break;
        }
        case NMEA_DECODE_ADDR4:
        {
            decodeState = NMEA_DECODE_ADDR5;
            checksum ^= _c; // Bytewise or
            buffer[payloadSize] = _c;
            payloadSize++;
            break;
        }
        case NMEA_DECODE_ADDR5:
        {
            decodeState = NMEA_DECODE_ADDR6;
            checksum ^= _c; // Bytewise or
            buffer[payloadSize] = _c;
            payloadSize++;
            break;
        }
        case NMEA_DECODE_ADDR6:
        {
            if (_c == NMEA_ADDR6) {
                decodeState = NMEA_DECODE_PAYLOAD;
                checksum ^= _c; // Bytewise or
                buffer[payloadSize] = _c;
                payloadSize++;
            }
            else {
                decodeInit();
            }
            break;
        }
        case NMEA_DECODE_PAYLOAD:
        {
            if (_c == NMEA_CHKSUM) {
                decodeState = NMEA_DECODE_CHKSUM1;
                printf("%c", _c);
            }
            else if (payloadSize > payloadThresh) {
                decodeInit();
            }
            else {
                checksum ^= _c; // Bytewise or
                buffer[payloadSize] = _c;
                payloadSize++;
            }
            break;
        }
        case NMEA_DECODE_CHKSUM1:
        {
            checksumField[0] = _c;
            decodeState = NMEA_DECODE_CHKSUM2;
            break;
        }
        case NMEA_DECODE_CHKSUM2:
        {
            checksumField[1] = _c;
            // Check cheksum
            uint8_t hn = checksumField[0] > '9' ? checksumField[0] - 'A' + 10 : checksumField[0] - '0';
            uint8_t ln = checksumField[1] > '9' ? checksumField[1] - 'A' + 10 : checksumField[1] - '0';
            uint8_t checksumMsg = (hn << 4 ) | ln;
            //printf("\r\nchecksumMsg=0x%X vs cheksum=0x%X\r\n", checksumMsg, checksum);
            if (checksum == checksumMsg) {
                decodeState = NMEA_DECODE_END1;
            }
            else {
                decodeInit();
            }
            break;
        }
        case NMEA_DECODE_END1:
        {
            if (_c == NMEA_END1) {
                decodeState = NMEA_DECODE_END2;
            }
            else {
                decodeInit();
            }
            break;
        }
        case NMEA_DECODE_END2:
        {
            if (_c == NMEA_END2) {
                // TODO: decode msg
                ret = decodeMsg(buffer);
            }
            decodeInit();
            break;
        }
        default:
        break;
    }

    return ret;
}