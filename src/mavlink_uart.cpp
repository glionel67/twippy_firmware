#include "mavlink_uart.h"

extern "C" {
#include "uart1.h"
#include "uart2.h"
} 


static mavlink_status_t lastStatus;

MavlinkInterface::MavlinkInterface()
{
    timeToExit_ = 0;
}

MavlinkInterface::~MavlinkInterface()
{
    
}

void MavlinkInterface::readThread(void)
{

}

void MavlinkInterface::writeThread(void)
{

}

int mavlink_init(void)
{
    lastStatus.packet_rx_drop_count = 0;
    return 0;
}

int read_message(mavlink_message_t* _msg)
{
    uint8_t cp = 0;
    mavlink_status_t status;
    uint8_t msgReceived = 0;

    int result = uart1_read(&cp, 1); // this function locks the port during read

    if (result > 0) { // PARSE MESSAGE
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
            //fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
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

int write_message(const mavlink_message_t* _msg)
{
    uint8_t buf[300] = { 0, };

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer(buf, _msg);

    // Write buffer to serial port, locks port while writing
    int bytesWritten = uart1_write(buf, len);

    return bytesWritten;
}