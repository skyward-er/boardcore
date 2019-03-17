
/**************************************************
Ssimple demo that receives mavlink
messages on the serial port and prints them on the screen.

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


void handleMavlinkMessage(const mavlink_message_t* msg);

int main(int argc, char *argv[])
{

    if(argc < 2)
    {
      printf("Usage: %s <PORT>\n", argv[0]);
      return -1;
    }

    int i, n,
      cport_nr = atoi(argv[1]),     
      bdrate = atoi(argv[2]);

    unsigned char buf[1000];
    mavlink_message_t msg;
    mavlink_status_t mavstatus;

    char mode[]={'8','N','1',0};


    if(RS232_OpenComport(cport_nr, bdrate, mode))
    {
        printf("Can not open comport\n");

        return -1;
    }

    printf("Entering infinite loop\n");

    while(1)
    {
        if(RS232_PollComport(cport_nr, buf, 1000) > 0)
        {
            printf("[RCV] byte arrived\n");
            /* Parse one char at a time until you find a complete message */
            for(int i = 0; i < 1000; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &(mavstatus)))
                {
                    printf(
                        "[RCV] Received message with ID %d, sequence: %d from "
                        "component %d of system %d\n",
                        msg.msgid, msg.seq, msg.compid, msg.sysid);

                    /* Handle the command */
                    handleMavlinkMessage(&msg);
                }
            }
        }
        //sleep(10);
    }

// #ifdef _WIN32
//     Sleep(100);
// #else
//     usleep(100000);  /* sleep for 100 milliSeconds */
// #endif
//   }

  return(0);
}


void handleMavlinkMessage(const mavlink_message_t* msg) {
    switch(msg->msgid) {
        case MAVLINK_MSG_ID_DEBUG_INFO_TM: 
            printf("[RCV] Received debug info %d\n", mavlink_msg_debug_info_tm_get_placeholder(msg));
            break;
        case MAVLINK_MSG_ID_ACK_TM: 
            printf("[RCV] ACK received\n");
            break;
        default:
            printf("[Response] Unknown message received\n");
    }
}