
/**************************************************
Simple demo that transmits mavlink messages
to the serial port.

**************************************************/
#include "../../libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h"
#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"

int cport_nr;

void sendAck()
{
    uint8_t bufferMsg[sizeof(mavlink_message_t) + 1];  // TODO Check this number
    mavlink_message_t ackMsg;

    // Create ack message passing the parameters
    mavlink_msg_ack_tm_pack(1, 1, &ackMsg,
                            1, 1);
    // Convert it into a byte stream
    int msgLen = mavlink_msg_to_send_buffer(bufferMsg, &ackMsg);

    // Send the message back to the sender
    RS232_SendBuf(cport_nr, bufferMsg, msgLen);

    printf("sent: %s\n", bufferMsg);
}

void sendDebugInfoReq()
{
    uint8_t bufferMsg[sizeof(mavlink_message_t) + 1];  // TODO Check this number
    mavlink_message_t msg;

    // Create ack message passing the parameters
    mavlink_msg_debug_info_tm_pack(1, 1, &msg, 1);
    // Convert it into a byte stream
    int msgLen = mavlink_msg_to_send_buffer(bufferMsg, &msg);

    // Send the message back to the sender
    RS232_SendBuf(cport_nr, bufferMsg, msgLen);

    printf("sent: %s\n", bufferMsg);
}


int main(int argc, char *argv[])
{

    if(argc < 2)
    {
      printf("Usage: %s <PORT>\n", argv[0]);
      return -1;
    }

    int i=0, bdrate = atoi(argv[2]);
    cport_nr = atoi(argv[1]);

    char mode[]={'8','N','1',0};

    if(RS232_OpenComport(cport_nr, bdrate, "8N1"))
    {
        printf("Can not open comport %d\n", cport_nr);
        return -1;
    }

    char c;

    while(1)
    {
        scanf("%c", &c);
        RS232_SendBuf(cport_nr, &c, 1);

        // sendDebugInfoReq();
        // sendAck();
    }

    return(0);
}
