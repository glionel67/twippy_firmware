#include "mavlink_uart.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "config.h"
#include "usTimer.h"
#include "uart1.h"
#include "uart2.h"

#define UART_READ(__data__, __len__) uart1_read(__data__, __len__)
#define UART_WRITE(__data__, __len__) uart1_write(__data__, __len__)

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

void resetTimestamps(Timestamps* _ts) {
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
        printf("mavlinkStart: failed to create mavlinkReadTask\n");
        return 0;
    }

    // Write task
    if (!(pdPASS == xTaskCreate(mavlinkWriteTask, (const char*)"mavlinkWriteTask",
            MAVLINK_WRITE_TASK_STACK_SIZE, NULL, MAVLINK_WRITE_TASK_PRIORITY, 
            &writeTaskHandle))) {
        // TODO: ERROR
        printf("mavlinkStart: failed to create mavlinkWriteTask\n");
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

    int res = uart1_read(&cp, 1); // this function locks the port during read
    //int res = UART_READ(&cp, 1);

    if (res == OK) { // PARSE MESSAGE
        // the parsing
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, _msg, &status);

        // check for dropped packets
        if (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) {
            //printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
        }
        lastStatus = status;
    }
    else { // Couldn't read from port
        //fprintf(stderr, "ERROR: Could not read from fd %d\n", fd);
        //char msg[] = "mavlinkReadMessage: could not read\n";
        //print_msg((uint8_t*)msg, strlen(msg));
    }

    if (msgReceived) {
        // Report info
        //printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", 
        //        message.msgid, message.sysid, message.compid);

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

    // Write buffer to serial port, locks port while writing
    int res = uart1_write(buf, len);
    //int res = UART_WRITE(buf, len);

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
    TickType_t deltaTick = 100;

    while (!timeToExit) {
        if (pdTRUE == xQueueReceive(mavlinkMsgQueue, &msg, (TickType_t)10)) {
            res = mavlinkWriteMessage(&msg);
            if (res == NOK) {
                // TODO: handle error
                printf("mavlinkWriteTask: failed to write message\r\n");
            }
        }
        
        // Send heartbeat and system status message
        currTick = xTaskGetTickCount();
        if (currTick - prevTick >= deltaTick) {
            mavlink_msg_heartbeat_pack(systemId, systemId, &msg, 
                MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
                MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);
            res = mavlinkWriteMessage(&msg);
            if (res == NOK) {
                // TODO: handle error
                //char msg[] = "mavlinkWriteTask: failed to write heartbeat message\r\n";
                //print_msg((uint8_t*)msg, strlen(msg));
            }

            mavlink_msg_sys_status_pack(systemId, systemId, &msg, 0, 0, 0, 0,
                    12000, -1, -1, 0, 0, 0, 0, 0, 0);
            res = mavlinkWriteMessage(&msg);
            if (res == NOK) {
                // TODO: handle error
                //char msg[] = "mavlinkWriteTask: failed to write sys_status message\r\n";
                //print_msg((uint8_t*)msg, strlen(msg));
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

uint8_t enqueueMavlinkMessage(mavlink_message_t _msg)
{
    if (xQueueSend(mavlinkMsgQueue, &_msg, (TickType_t)10) != pdPASS)
        return 1;
    else
        return 0;
}