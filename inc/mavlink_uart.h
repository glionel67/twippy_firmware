#pragma once

#include "common/mavlink.h"

int mavlink_init(void);
int read_message(mavlink_message_t* _msg);
int write_message(const mavlink_message_t* _msg);

class MavlinkInterface {
public:
    uint8_t timeToExit_;
    //SerialPort* serialPort_;

    MavlinkInterface();
    ~MavlinkInterface();

    void readThread(void);
    void writeThread(void);
}; // class MavlinkInterface
