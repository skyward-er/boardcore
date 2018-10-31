
/**************************************************

file: demo_tx.c
purpose: simple demo that transmits characters to
the serial port and print them on the screen,
exit the program by pressing Ctrl-C

compile with the command: gcc demo_tx.c rs232.c -Wall -Wextra -o2 -o test_tx

**************************************************/
#include <libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"

int cport_nr = 6;

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


int main()
{
  int i=0, bdrate=57600;  

  char mode[]={'8','N','1',0};

  if(RS232_OpenComport(cport_nr, bdrate, "8N1"))
  {
    printf("Can not open comport\n");
    return -1;
  }

  while(1)
  {
    char c;
    scanf("%c", &c);
    sendDebugInfoReq();

    //sendAck();
  }

  return(0);
}

