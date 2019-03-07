#include "mavlink_uart.h"

// C lib
#include <math.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "config.h"
#include "usTimer.h"
//#include "uart1.h"
//#include "uart2.h"
#include "uart3.h"
#include "imu.h"
#include "ahrs.h"

#define UART_READ(__data__, __len__) uart3_read(__data__, __len__)
#define UART_WRITE(__data__, __len__) uart3_write(__data__, __len__)

static bool sendImuData = true;
static bool sendBatteryData = true;

static TaskHandle_t readTaskHandle = 0;
static TaskHandle_t writeTaskHandle = 0;
static xQueueHandle mavlinkMsgQueue = 0;
static mavlink_status_t lastStatus;
static MavlinkMessages mavlinkMessages;
static uint8_t timeToExit = 0;
static uint8_t readingStatus = 0;
static uint8_t writingStatus = 0;

static int systemId = 0;
static int autopilotId = 0;
static int companionId = 0;

void resetTimestamps(Timestamps* _ts)
{
    _ts->heartbeat = 0;
    _ts->sys_status = 0;
    _ts->battery_status = 0;
    _ts->radio_status = 0;
    _ts->local_position_ned = 0;
    _ts->global_position_int = 0;
    _ts->position_target_local_ned = 0;
    _ts->position_target_global_int = 0;
    _ts->highres_imu = 0;
    _ts->attitude = 0;
}

int mavlinkInit(void)
{
    timeToExit = 0;
    readingStatus = 0;
    writingStatus = 0;

    systemId = 1;
    autopilotId = 1;
    companionId = 0;

    mavlinkMessages.systemId    = systemId;
    mavlinkMessages.autopilotId = autopilotId;
    mavlinkMessages.companionId = companionId;
    resetTimestamps(&mavlinkMessages.timestamps);

    lastStatus.packet_rx_drop_count = 0;
    return 1;
}

int mavlinkStart(void)
{
    // TODO: check that uart1 is init

    timeToExit = 0;

    // Read task
    if (!(pdPASS == xTaskCreate(mavlinkReadTask, (const char*)"mavlinkReadTask",
            MAVLINK_READ_TASK_STACK_SIZE, NULL, MAVLINK_READ_TASK_PRIORITY, 
            &readTaskHandle))) {
        // TODO: ERROR
        printf("mavlinkStart: failed to create mavlinkReadTask\r\n");
        return 0;
    }

    // Write task
    if (!(pdPASS == xTaskCreate(mavlinkWriteTask, (const char*)"mavlinkWriteTask",
            MAVLINK_WRITE_TASK_STACK_SIZE, NULL, MAVLINK_WRITE_TASK_PRIORITY, 
            &writeTaskHandle))) {
        // TODO: ERROR
        printf("mavlinkStart: failed to create mavlinkWriteTask\r\n");
        return 0;
    }

    return 1;
}

void mavlinkStop(void)
{
    timeToExit = 1;
}

int mavlinkReadMessage(mavlink_message_t* _msg)
{
    uint8_t cp = 0;
    mavlink_status_t status;
    uint8_t msgReceived = 0;

    int res = UART_READ(&cp, 1);

    if (res == OK) { // PARSE MESSAGE
        printf("mavlinkReadMessage: c=%c\r\n", cp);
        // the parsing
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, _msg, &status);

        // check for dropped packets
        if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) {
            printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
        }
        lastStatus = status;
    }
    else { // Couldn't read from port
        //printf("mavlinkReadMessage: could not read\r\n");
    }

    if (msgReceived) {
        // Report info
        printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", 
                _msg->msgid, _msg->sysid, _msg->compid);

        //fprintf(stderr,"Received serial data: ");
        
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = { 0, };

        // check message is write length
        unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, _msg);

        // message length error
        if (messageLength > MAVLINK_MAX_PACKET_LEN) {
            printf("mavlinkReadMessage: FATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\r\n");
        }
        else { // print out the buffer
            // unsigned int i = 0;
            // for (i=0;i<messageLength;i++) {
            //     unsigned char v=buffer[i];
            //     fprintf(stderr,"%02x ", v);
            // }
            // fprintf(stderr,"\n");
        }
    }

    // Done!
    return msgReceived;
}

void mavlinkReadMessages(void)
{
    uint8_t success = 0; // receive success flag
    uint8_t receivedAll = 0; // receive only one message
    Timestamps timestamps;
    resetTimestamps(&timestamps);

    // Blocking wait for new data
    while (!receivedAll && !timeToExit) {
        mavlink_message_t message;
        success = mavlinkReadMessage(&message);

        if (success) {
            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            mavlinkMessages.systemId  = message.sysid;
            mavlinkMessages.companionId = message.compid;

            // Handle Message ID
            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(mavlinkMessages.heartbeat));
                    mavlinkMessages.timestamps.heartbeat = get_us_time();
                    timestamps.heartbeat = mavlinkMessages.timestamps.heartbeat;
                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                    mavlink_msg_sys_status_decode(&message, &(mavlinkMessages.sys_status));
                    mavlinkMessages.timestamps.sys_status = get_us_time();
                    timestamps.sys_status = mavlinkMessages.timestamps.sys_status;
                    break;
                }

                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
                    mavlink_msg_battery_status_decode(&message, &(mavlinkMessages.battery_status));
                    mavlinkMessages.timestamps.battery_status = get_us_time();
                    timestamps.battery_status = mavlinkMessages.timestamps.battery_status;
                    break;
                }

                case MAVLINK_MSG_ID_RADIO_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                    mavlink_msg_radio_status_decode(&message, &(mavlinkMessages.radio_status));
                    mavlinkMessages.timestamps.radio_status = get_us_time();
                    timestamps.radio_status = mavlinkMessages.timestamps.radio_status;
                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                    mavlink_msg_local_position_ned_decode(&message, &(mavlinkMessages.local_position_ned));
                    mavlinkMessages.timestamps.local_position_ned = get_us_time();
                    timestamps.local_position_ned = mavlinkMessages.timestamps.local_position_ned;
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                    mavlink_msg_global_position_int_decode(&message, &(mavlinkMessages.global_position_int));
                    mavlinkMessages.timestamps.global_position_int = get_us_time();
                    timestamps.global_position_int = mavlinkMessages.timestamps.global_position_int;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                    mavlink_msg_position_target_local_ned_decode(&message, &(mavlinkMessages.position_target_local_ned));
                    mavlinkMessages.timestamps.position_target_local_ned = get_us_time();
                    timestamps.position_target_local_ned = mavlinkMessages.timestamps.position_target_local_ned;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                    mavlink_msg_position_target_global_int_decode(&message, &(mavlinkMessages.position_target_global_int));
                    mavlinkMessages.timestamps.position_target_global_int = get_us_time();
                    timestamps.position_target_global_int = mavlinkMessages.timestamps.position_target_global_int;
                    break;
                }

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_highres_imu_decode(&message, &(mavlinkMessages.highres_imu));
                    mavlinkMessages.timestamps.highres_imu = get_us_time();
                    timestamps.highres_imu = mavlinkMessages.timestamps.highres_imu;
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_msg_attitude_decode(&message, &(mavlinkMessages.attitude));
                    mavlinkMessages.timestamps.attitude = get_us_time();
                    timestamps.attitude = mavlinkMessages.timestamps.attitude;
                    break;
                }

                default:
                {
                    printf("mavlinkReadMessage: handle message id not handled\n");
                    // printf("Warning, did not handle message id %i\n",message.msgid);
                    break;
                }
            } // end: switch msgid

        } // end: if read message

        // Check for receipt of all items
        receivedAll =
                timestamps.heartbeat                  &&
             // timestamps.battery_status             &&
             // timestamps.radio_status               &&
             // timestamps.local_position_ned         &&
             // timestamps.global_position_int        &&
             // timestamps.position_target_local_ned  &&
             // timestamps.position_target_global_int &&
             // timestamps.highres_imu                &&
             // timestamps.attitude                   &&
                timestamps.sys_status
                ;

        // give the write thread time to use the port
        if (writingStatus > 0) {
            vTaskDelay(1/portTICK_RATE_MS);
        }
    } // end: while not received all

    return;
}

int mavlinkWriteMessage(const mavlink_message_t* _msg)
{
    uint8_t buf[300] = { 0, };

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer(buf, _msg);
    //printf("mavlinkWriteMessage: len=%u\r\n", len);

    // Write buffer to serial port, locks port while writing
    int res = UART_WRITE(buf, len);

    return res;
}

void mavlinkReadTask(void* _params)
{
    if (_params != 0) { }

    readingStatus = 1;

    while (!timeToExit) {
        mavlinkReadMessages();
        vTaskDelay(10/portTICK_RATE_MS);
    }

    readingStatus = 0;

    vTaskDelete(NULL);
}

void mavlinkWriteTask(void* _params)
{
    if (_params != 0) { }

    mavlinkMsgQueue = xQueueCreate(MAVLINK_QUEUE_SIZE, sizeof(mavlink_message_t));
    if (mavlinkMsgQueue == 0) {
        // TODO: handle error
        printf("mavlinkWriteTask: failed to create mavlinkMsgQueue\r\n");
        vTaskDelete(NULL);
    }

    writingStatus = 1;

    mavlink_message_t msg;
    int res = 0;
    TickType_t prevTick = xTaskGetTickCount();
    TickType_t currTick = prevTick;
    TickType_t deltaTick = 1000; // 1 Hz for heartbeat and system status

    while (!timeToExit) {
        // Send heartbeat and system status message
        currTick = xTaskGetTickCount();

        if (pdTRUE == xQueueReceive(mavlinkMsgQueue, &msg, (TickType_t)10)) {
            res = mavlinkWriteMessage(&msg);
            if (res == NOK) {
                // TODO: handle error
                printf("mavlinkWriteTask: failed to write message\r\n");
            }
        }

        if (sendImuData) {
            res = mavlinkSendImuMessage();
            if (res == NOK) {
                printf("mavlinkWriteTask: mavlinkSendImuMessage failed!\r\n");
            }
        }

        if (sendBatteryData) {
            res = mavlinkSendBatteryMessage();
            if (res == NOK) {
                printf("mavlinkWriteTask: sendBatteryData failed!\r\n");
            }
        }
        
        if (currTick - prevTick >= deltaTick) {
            mavlink_msg_heartbeat_pack(systemId, MAV_COMP_ID_AUTOPILOT1, &msg, 
                MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
                MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
            res = mavlinkWriteMessage(&msg);
            if (res == NOK) {
                // TODO: handle error
                printf("mavlinkWriteTask: failed to write heartbeat message\r\n");
            }

            mavlink_msg_sys_status_pack(systemId, MAV_COMP_ID_AUTOPILOT1, &msg,
                    0, 0, 0, 0, 12000, -1, -1, 0, 0, 0, 0, 0, 0);
            res = mavlinkWriteMessage(&msg);
            if (res == NOK) {
                // TODO: handle error
                printf("mavlinkWriteTask: failed to write sys_status message\r\n");
            }

            prevTick = currTick;
        }

        vTaskDelay(10/portTICK_RATE_MS);
    }

    // signal end
    writingStatus = 0;

    vQueueDelete(mavlinkMsgQueue);
    mavlinkMsgQueue = 0;

    vTaskDelete(NULL);
}

int mavlinkSendImuMessage(void)
{
    // Get IMU data
    Imu9_t imu9;
    get_imu9_data(&imu9);

    // Get current time [ms]
    TickType_t ticks = xTaskGetTickCount();
    
    // Scale data
    uint16_t ax = (uint16_t)round(imu9.a[0] / GRAVITY * 1000.);
    uint16_t ay = (uint16_t)round(imu9.a[1] / GRAVITY * 1000.);
    uint16_t az = (uint16_t)round(imu9.a[2] / GRAVITY * 1000.);

    uint16_t gx = (uint16_t)round(imu9.g[0] * 1000.);
    uint16_t gy = (uint16_t)round(imu9.g[1] * 1000.);
    uint16_t gz = (uint16_t)round(imu9.g[2] * 1000.);
    
    uint16_t mx = (uint16_t)round(imu9.m[0] * 1000.);
    uint16_t my = (uint16_t)round(imu9.m[1] * 1000.);
    uint16_t mz = (uint16_t)round(imu9.m[2] * 1000.);

    // Create mavlink message   
    mavlink_message_t msg;
    uint16_t msgLen = mavlink_msg_scaled_imu_pack(systemId, MAV_COMP_ID_IMU, 
        &msg, (uint32_t)ticks, ax, ay, az, gx, gy, gz, mx, my, mz);
    printf("mavlinkSendImuMessage: msgLen=%u\r\n", msgLen);
    if (msgLen <= 0)
        return NOK;
    
    // Send message
    int res = mavlinkWriteMessage(&msg);
    if (res == NOK) {
        printf("mavlinkSendImuMessage: failed to write message\r\n");
        return NOK;
    }

    return OK;
}

int mavlinkSendAttitudeMessage(void)
{
    // Get attitude data
    Attitude_t att;
    ahrs_get_attitude(&att);

    // Get current time [ms]
    TickType_t ticks = xTaskGetTickCount();
    
    // Create mavlink message   
    mavlink_message_t msg;
    uint16_t msgLen = mavlink_msg_attitude_quaternion_pack(systemId, 
        MAV_COMP_ID_AUTOPILOT1, &msg, (uint32_t)ticks, att.qw, att.qx, att.qy,
        att.qz, att.wx, att.wy, att.wz);
    printf("mavlinkSendAttitudeMessage: msgLen=%u\r\n", msgLen);
    if (msgLen <= 0)
        return NOK;

    //mavlink_msg_attitude_pack
    
    // Send message
    int res = mavlinkWriteMessage(&msg);
    if (res == NOK) {
        printf("mavlinkSendAttitudeMessage: failed to write message\r\n");
        return NOK;
    }

    return OK;
}

int mavlinkSendGpsMessage(void)
{
    // Get GPS data
    float latDeg = 7.0; // [deg]
    float lonDeg = 40.; // [deg]
    float altMet = 1.; // [m]
    float hdop = 1.;
    float vdop = 1.;
    float speedOfGround = 1.; // [knot/s] or [m/s] ?
    float courseOfGround = 0.; // [degrees] ?
    float ellipsoidAlt = 1.;
    float xyUncertainty = 1.;
    float zUncertainty = 1.;
    float speedUncertainty = 1.;
    float headingUncertainty = 1.;

    // Get current time [us]
    uint64_t usTime = get_us_time();
    // GPS fix type: GPS_FIX_TYPE_NO_GPS, GPS_FIX_TYPE_NO_FIX,
    // GPS_FIX_TYPE_2D_FIX, GPS_FIX_TYPE_3D_FIX, GPS_FIX_TYPE_DGPS,
    // GPS_FIX_TYPE_RTK_FLOAT, GPS_FIX_TYPE_RTK_FIXED, GPS_FIX_TYPE_STATIC, GPS_FIX_TYPE_PPP
    uint8_t fixType = GPS_FIX_TYPE_2D_FIX; 
    int32_t lat = (int32_t)round(latDeg * 1e7);
    int32_t lon = (int32_t)round(lonDeg * 1e7);
    int32_t alt = (int32_t)round(altMet * 1e3);
    uint16_t eph = (uint16_t)round(hdop); //  If unknown, set to: UINT16_MAX
    uint16_t epv = (uint16_t)round(vdop); //  If unknown, set to: UINT16_MAX
    uint16_t vel = (uint16_t)round(speedOfGround * 1e2); // [cm/s]  If unknown, set to: UINT16_MAX
    uint16_t cog = (uint16_t)round(courseOfGround * 1e2); // [cdeg]  If unknown, set to: UINT16_MAX
    uint8_t nSats = 6; // If unknown, set to 255
    int32_t ellAlt = (int32_t)round(ellipsoidAlt * 1e3);
    uint32_t hAcc = (int32_t)round(xyUncertainty * 1e3); // [mm] Position uncertainty. Positive for up.
    uint32_t vAcc = (int32_t)round(zUncertainty * 1e3); // [mm] Altitude uncertainty. Positive for up.
    uint32_t velAcc = (int32_t)round(speedUncertainty * 1e3); // [mm] Speed uncertainty. Positive for up.
    uint32_t hdgAcc = (int32_t)round(headingUncertainty * 1e5); // [degE5] Heading / track uncertainty

    // Create mavlink message   
    mavlink_message_t msg;
    uint16_t msgLen = mavlink_msg_gps_raw_int_pack(systemId, MAV_COMP_ID_GPS,
        &msg, usTime, fixType, lat, lon, alt, eph, epv, vel, cog, nSats, ellAlt,
        hAcc, vAcc,  velAcc, hdgAcc);
    printf("mavlinkSendGpsMessage: msgLen=%u\r\n", msgLen);
    if (msgLen <= 0)
        return NOK;

    // Send message
    int res = mavlinkWriteMessage(&msg);
    if (res == NOK) {
        printf("mavlinkSendAttitudeMessage: failed to write message\r\n");
        return NOK;
    }

    return OK;
}

int mavlinkSendBatteryMessage(void)
{
    // Get battery data

    uint8_t battId = 0;
    uint8_t battFct = MAV_BATTERY_FUNCTION_PROPULSION;
    uint8_t battType = MAV_BATTERY_TYPE_LIPO;
    int16_t temp = INT16_MAX;
    uint16_t volts[10] = { 0, };
    volts[0] = 12000; // @TODO get_vbat_mv();
    int16_t curr = -1; // @TODO get_ibat_ma();
    int32_t currCons = -1;
    int32_t nrjCons = -1;
    int8_t battRem = -1;
    int32_t timeRem = 0;
    uint8_t battState = MAV_BATTERY_CHARGE_STATE_OK;
    // Create mavlink message   
    mavlink_message_t msg;
    uint16_t msgLen = mavlink_msg_battery_status_pack(systemId,
        MAV_COMP_ID_AUTOPILOT1, &msg, battId, battFct, battType, temp, volts,
        curr, currCons, nrjCons, battRem, timeRem, battState);
    printf("mavlinkSendBatteryMessage: msgLen=%u\r\n", msgLen);
    if (msgLen <= 0)
        return NOK;
    
    // Send message
    int res = mavlinkWriteMessage(&msg);
    if (res == NOK) {
        printf("mavlinkSendBatteryMessage: failed to write message\r\n");
        return NOK;
    }

    return OK;
}

uint8_t enqueueMavlinkMessage(mavlink_message_t _msg)
{
    if (xQueueSend(mavlinkMsgQueue, &_msg, (TickType_t)10) != pdPASS)
        return 1;
    else
        return 0;
}