
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

int cport_nr = 5;

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


int main()
{
  int i=0,      
      bdrate=19200;       /* 9600 baud */

  char mode[]={'8','N','1',0},
       str[2][512];


  strcpy(str[0], "The quick brown fox jumped over the lazy grey dog.\n");

  strcpy(str[1], "Happy serial programming!\n");

  if(RS232_OpenComport(cport_nr, 19200, "8N1"))
  {
    printf("Can not open comport\n");

    return(0);
  }

  while(1)
  {
    sendAck();

#ifdef _WIN32
    Sleep(1000);
#else
    usleep(1000000);  /* sleep for 1 Second */
#endif

    i++;

    i %= 2;
  }

  return(0);
}

