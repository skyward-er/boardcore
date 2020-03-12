/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <Windows.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"

#define START_FRAME_DELIMITER 0xff
#define END_FRAME_DELIMITER 0xff
#define MAX_REP 254

// #define WAIT_FOR_ACK 
// #define DEBUG

/*
Sends a certain number of messages of a fixed length over serial
port, with increasing ids. 

Used for benchmarking LoRa modules (RF solutions Gamma868).
*/
int main(int argc, char *argv[])
{
    /* Read arguments */
    if(argc < 5)
    {
      printf("Usage: %s <PORT> <BAUD> <MSGSIZE> <SLEEP>\n", argv[0]);
      return -1;
    }

    int port = atoi(argv[1]);
    int bdrate = atoi(argv[2]);
    int msgsize = atoi(argv[3]);
    int sleepTime = atoi(argv[4]);

    /* Open serial */
    if(RS232_OpenComport(port, bdrate, "8N1"))
    {
        printf("Can not open comport %d\n", port);
        printf("Note: On Windows, COM1 = port 0, COM2 = port 1 ecc..\n");
        return -1;
    }

    /* Create message */
    unsigned char* c = malloc(msgsize);
    c[0] = 0xff;
    c[msgsize - 1 ] = 0xff;

    int errorCount = 0;
    uint8_t currentPkt = 0;

    /* Send & wait */
    while(currentPkt < MAX_REP)
    {
        /* Populate message */
        memset((unsigned char*)c + 1, currentPkt, msgsize - 2);

        /* Send */
        DWORD start = GetTickCount();
        RS232_SendBuf(port, c, msgsize);

#ifdef DEBUG
        /* Print sent chars */
        for(int i = 0; i < msgsize; i++) {
            printf("%d\n", c[i]);
        }
        printf("\n");
#endif

#ifdef WAIT_FOR_ACK
        /* wait ack */
        unsigned char rcvBuf[5];
        while(1) {

            if(RS232_PollComport(port, rcvBuf, 1) > 0) {
                // printf("[RCV] byte arrived\n");
                // printf("%c\n", rcvBuf[0] );

                if(rcvBuf[0] != '#'){
                    errorCount++;
                }

                RS232_flushRXTX(port);

                break;
            }
        }
#endif

        /* Print time & errors */
        uint32_t deltaTime = GetTickCount() - start;
        printf("Delta: %u errors: %d\n", deltaTime, errorCount);
        currentPkt++;

        if(sleepTime > 0)
            Sleep(sleepTime);
    }

    return(0);
}

