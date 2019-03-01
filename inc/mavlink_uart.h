#pragma once

#include <stdint.h>

#include "common/mavlink.h"

#define MAVLINK_QUEUE_SIZE 20

typedef struct Timestamps_s {
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
} Timestamps; // struct Timestamps

typedef struct MavlinkMessages_s {
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
} MavlinkMessages; // struct MavlinkMessages

void resetTimestamps(Timestamps* _ts);

int mavlinkInit(void);

int mavlinkStart(void);
void mavlinkStop(void);

int mavlinkReadMessage(mavlink_message_t* _msg);
void mavlinkReadMessages(void);

int mavlinkWriteMessage(const mavlink_message_t* _msg);

void mavlinkReadTask(void* _params);
void mavlinkWriteTask(void* _params);

int mavlinkSendImuMessage(void);
int mavlinkSendAttitudeMessage(void);
int mavlinkSendGpsMessage(void); // @TODO
int mavlinkSendWheelDistMessage(void); // @TODO
int mavlinkSendBatteryMessage(void);

uint8_t enqueueMavlinkMessage(mavlink_message_t _msg);