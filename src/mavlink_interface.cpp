#include "mavlink_interface.h"

extern "C" {
// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "config.h"
#include "usTimer.h"
}

static xQueueHandle mavlinkMsgQueue = 0;

uint8_t enqueueMavlinkMessage(mavlink_message_t _msg)
{
    if (xQueueSend(mavlinkMsgQueue, &_msg, (TickType_t)10) != pdPASS)
        return 1;
    else
        return 0;
}

MavlinkInterface::MavlinkInterface(/*Port* _port*/)
{
    // initialize attributes
    writeCount_ = 0;

    readingStatus_ = 0;      // whether the read thread is running
    writingStatus_ = 0;      // whether the write thread is running
    timeToExit_    = 0;  // flag to signal thread exit

    systemId_    = 0; // system id
    autopilotId_ = 0; // autopilot component id
    companionId_ = 0; // companion computer component id

    mavlinkMessages_.systemId    = systemId_;
    mavlinkMessages_.autopilotId = autopilotId_;
    mavlinkMessages_.companionId = companionId_;

    //port_ = _port; // serial port management object
}

MavlinkInterface::~MavlinkInterface()
{
}

void MavlinkInterface::readMessages(void)
{
    uint8_t success = 0; // receive success flag
    uint8_t receivedAll = 0; // receive only one message
    Timestamps timestamps;

    // Blocking wait for new data
    while (!receivedAll and !timeToExit_) {
        mavlink_message_t message;
        //success = port_->read_message(message);

        if (success) {
            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            mavlinkMessages_.systemId  = message.sysid;
            mavlinkMessages_.companionId = message.compid;

            // Handle Message ID
            switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(mavlinkMessages_.heartbeat));
                    mavlinkMessages_.timestamps.heartbeat = get_us_time();
                    timestamps.heartbeat = mavlinkMessages_.timestamps.heartbeat;
                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                    mavlink_msg_sys_status_decode(&message, &(mavlinkMessages_.sys_status));
                    mavlinkMessages_.timestamps.sys_status = get_us_time();
                    timestamps.sys_status = mavlinkMessages_.timestamps.sys_status;
                    break;
                }

                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
                    mavlink_msg_battery_status_decode(&message, &(mavlinkMessages_.battery_status));
                    mavlinkMessages_.timestamps.battery_status = get_us_time();
                    timestamps.battery_status = mavlinkMessages_.timestamps.battery_status;
                    break;
                }

                case MAVLINK_MSG_ID_RADIO_STATUS:
                {
                    //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                    mavlink_msg_radio_status_decode(&message, &(mavlinkMessages_.radio_status));
                    mavlinkMessages_.timestamps.radio_status = get_us_time();
                    timestamps.radio_status = mavlinkMessages_.timestamps.radio_status;
                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    //printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                    mavlink_msg_local_position_ned_decode(&message, &(mavlinkMessages_.local_position_ned));
                    mavlinkMessages_.timestamps.local_position_ned = get_us_time();
                    timestamps.local_position_ned = mavlinkMessages_.timestamps.local_position_ned;
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    //printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                    mavlink_msg_global_position_int_decode(&message, &(mavlinkMessages_.global_position_int));
                    mavlinkMessages_.timestamps.global_position_int = get_us_time();
                    timestamps.global_position_int = mavlinkMessages_.timestamps.global_position_int;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                    mavlink_msg_position_target_local_ned_decode(&message, &(mavlinkMessages_.position_target_local_ned));
                    mavlinkMessages_.timestamps.position_target_local_ned = get_us_time();
                    timestamps.position_target_local_ned = mavlinkMessages_.timestamps.position_target_local_ned;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                {
                    //printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                    mavlink_msg_position_target_global_int_decode(&message, &(mavlinkMessages_.position_target_global_int));
                    mavlinkMessages_.timestamps.position_target_global_int = get_us_time();
                    timestamps.position_target_global_int = mavlinkMessages_.timestamps.position_target_global_int;
                    break;
                }

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_highres_imu_decode(&message, &(mavlinkMessages_.highres_imu));
                    mavlinkMessages_.timestamps.highres_imu = get_us_time();
                    timestamps.highres_imu = mavlinkMessages_.timestamps.highres_imu;
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    //printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_msg_attitude_decode(&message, &(mavlinkMessages_.attitude));
                    mavlinkMessages_.timestamps.attitude = get_us_time();
                    timestamps.attitude = mavlinkMessages_.timestamps.attitude;
                    break;
                }

                default:
                {
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
        if (writingStatus_ > 0) {
            vTaskDelay(1/portTICK_RATE_MS);
            //usleep(100); // look for components of batches at 10kHz
        }
    } // end: while not received all

    return;
}

int MavlinkInterface::writeMessage(mavlink_message_t _msg)
{
    if (_msg.sysid == 1) {

    }

    // do the write
    //int len = port_->write_message(_msg);

    // book keep
    writeCount_++;

    // Done!
    //return len;
    return 0;
}

void MavlinkInterface::start(void)
{
    // Check that the port is opened

    // if (port_->status != 1) { // SERIAL_PORT_OPEN
    //     fprintf(stderr,"ERROR: serial port not open\n");
    //     throw 1;
    // }

    timeToExit_ = 0;

    // Read thread
    if (!(pdPASS == xTaskCreate(&startReadTask, (const char*)"startReadTask",
            MAVLINK_READ_TASK_STACK_SIZE, NULL, MAVLINK_READ_TASK_PRIORITY, 
            &readTaskHandle_))) {
        // TODO: ERROR
        //vTaskResume(readTaskHandle_);
    }

    // Write thread
    if (!(pdPASS == xTaskCreate(&startWriteTask, (const char*)"startWriteTask",
            MAVLINK_WRITE_TASK_STACK_SIZE, NULL, MAVLINK_WRITE_TASK_PRIORITY, 
            &writeTaskHandle_))) {
        // TODO: ERROR
        //vTaskResume(writeTaskHandle_);
    }
}

void MavlinkInterface::stop(void)
{
    // signal exit
    timeToExit_ = 1;

    // wait for exit
    //vTaskSuspend(readTaskHandle_);
    //vTaskSuspend(writeTaskHandle_);
}

void MavlinkInterface::startReadTask(void* _params)
{
    //if (_params != 0) { }

    static_cast<MavlinkInterface*>(_params)->readTask();

    // if (readingStatus_ != 0) {
    //     //fprintf(stderr,"read thread already running\n");
    // }
    // else {
    //     readTask();
    // }

    vTaskDelete(NULL);
}

void MavlinkInterface::startWriteTask(void* _params)
{
    //if (_params != 0) { }

    static_cast<MavlinkInterface*>(_params)->writeTask();

    // if (writingStatus_ != 0) {
    //     //fprintf(stderr,"write thread already running\n");
    // }
    // else {
    //     writeTask();
    // }

    vTaskDelete(NULL);
}

void MavlinkInterface::readTask(void)
{
    readingStatus_ = 1;

    while (!timeToExit_) {
        readMessages();
        vTaskDelay(10/portTICK_RATE_MS);
    }

    readingStatus_ = 0;
}

void MavlinkInterface::writeTask(void)
{
    mavlinkMsgQueue = xQueueCreate(MAVLINK_QUEUE_SIZE, sizeof(mavlink_message_t));
    if (mavlinkMsgQueue == 0) {
        // TODO: handle error
    }

    writingStatus_ = 1;

    mavlink_message_t msg;
    int len = 0;

    while (!timeToExit_) {
        // Read queue
        if (pdTRUE == xQueueReceive(mavlinkMsgQueue, &msg, 1)) {
            len = writeMessage(msg);
            if (len == 0) {
                // TODO: handle error
            }
        }
        vTaskDelay(10/portTICK_RATE_MS);
    }

    // signal end
    writingStatus_ = 0;

    vQueueDelete(mavlinkMsgQueue);

    mavlinkMsgQueue = 0;
}