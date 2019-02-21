#pragma once

#include <stdint.h>

extern "C" {
// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
}

#include "common/mavlink.h"

#define MAVLINK_QUEUE_SIZE 20

struct Timestamps {
    Timestamps() {
        resetTimestamps();
    }

    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;

    void resetTimestamps() {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        local_position_ned = 0;
        global_position_int = 0;
        position_target_local_ned = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
    }
}; // struct Timestamps

struct MavlinkMessages {
    int systemId;
    int companionId;
    int autopilotId;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;

    // System Status
    mavlink_sys_status_t sys_status;

    // Battery Status
    mavlink_battery_status_t battery_status;

    // Radio Status
    mavlink_radio_status_t radio_status;

    // Local Position
    mavlink_local_position_ned_t local_position_ned;

    // Global Position
    mavlink_global_position_int_t global_position_int;

    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;

    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;

    // HiRes IMU
    mavlink_highres_imu_t highres_imu;

    // Attitude
    mavlink_attitude_t attitude;

    // System Parameters?

    // Time Stamps
    Timestamps timestamps;

    void resetTimestamps() {
        timestamps.resetTimestamps();
    }
}; // struct MavlinkMessages

uint8_t enqueueMavlinkMessage(mavlink_message_t _msg);


class MavlinkInterface {
public:
    MavlinkInterface(/*Port* _port*/);
    ~MavlinkInterface();

    void readMessages(void);
    int writeMessage(mavlink_message_t _msg);

    void start(void);
    void stop(void);

    static void startReadTask(void* _params);
    static void startWriteTask(void* _params);

    void readTask(void);
    void writeTask(void);

    TaskHandle_t readTaskHandle_;
    TaskHandle_t writeTaskHandle_;

    int systemId_;
    int autopilotId_;
    int companionId_;

    uint8_t readingStatus_;
    uint8_t writingStatus_;
    uint64_t writeCount_;
    uint8_t timeToExit_;

    MavlinkMessages mavlinkMessages_;

    //Port* port_;   
}; // class MavlinkInterface