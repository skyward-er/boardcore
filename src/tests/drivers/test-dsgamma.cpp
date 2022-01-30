/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Alvise de'Faveri Tron, Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/gamma868/Gamma868.h>
#include <miosix.h>
#include <stdio.h>
#include <string.h>
#include <utils/Debug.h>

using namespace Boardcore;
using namespace std;
using miosix::FastInterruptDisableLock;
using miosix::Gpio;
using miosix::Thread;

// Protocol config
//#define DATA_LEN 16384

/* DISCOVERY F429I*/
typedef Gpio<GPIOA_BASE, 0> button;
// RTT calculation
// long long sendTime = 0;
enum State
{
    ST_STARTING = 0,
    ST_WAIT_START_FRAME,
    ST_WAIT_END_FRAME,
    ST_SEND_DATA
};

uint8_t max(uint8_t a, uint8_t b) { return a > b ? a : b; }

int main()
{
    // Discovery gpio setup
    {
        FastInterruptDisableLock dLock;
        button::mode(miosix::Mode::INPUT);
    }

    Gamma868 gamma("/dev/radio");  // create gamma object

    // printf("Press the button to start receiving \n");
    // Wait for button
    Thread::sleep(1000);
    // while (1)
    // {
    //     if (button::value() == 1)
    //         break;
    // }
    printf("Receiving...\n");

    //  uint8_t inputBuf[DATA_LEN];

    int index        = 0;
    int lostBytes    = 0;
    uint8_t pktCount = 0;

    int state = ST_STARTING;
    bool end  = false;

    uint32_t startT, endT;

    while (1)
    {
        while (!end)
        {
            switch (state)
            {
                case ST_STARTING:
                {
                    unsigned char c;
                    gamma.receive(&c, 1);
                    TRACE("Received %c\n", c);

                    if (c == 255)
                    {
                        TRACE("Starting!\n", 0);
                        startT = miosix::getTick();
                        // printf("%c", c);
                        //  inputBuf[index] = c;
                        ++index;
                        state = ST_WAIT_END_FRAME;
                        endT  = miosix::getTick();
                    }
                    break;
                }
                case ST_WAIT_START_FRAME:
                {
                    unsigned char c;
                    gamma.receive(&c, 1);
                    // TRACE("Received %c\n", c);

                    if (c == 255)
                    {
                        TRACE("Starting!\n", 0);

                        // printf("%c", c);
                        // inputBuf[index] = c;
                        ++index;
                        state = ST_WAIT_END_FRAME;
                    }
                    break;
                }
                case ST_WAIT_END_FRAME:
                {
                    unsigned char c;
                    // TRACE("Received %c\n", c);
                    gamma.receive(&c, 1);

                    // inputBuf[index++] = c;
                    ++index;
                    if (c != 255 && c != pktCount)
                    {
                        TRACE("Error: Expected: %d: %d\n", pktCount, (int)c);
                        lostBytes++;
                    }

                    if (c == 255 && pktCount < 251)
                    {
                        printf("Packet end %d. lost: %d\n", pktCount,
                               lostBytes);
                        ++pktCount;
                        // endT = miosix::getTick();
                        state = ST_WAIT_START_FRAME;
                    }

                    if (c == 255 && pktCount >= 252)
                    {
                        TRACE("Ending\n", 0);

                        state = ST_SEND_DATA;
                    }
                    /* if (c != 255)
                         pktCount = max(pktCount, c);*/
                    break;
                }
                case ST_SEND_DATA:
                {
                    endT = miosix::getTick();
                    end  = true;
                    break;
                    /*  uint8_t buf[] = {0x23, 0x23, 0x23, 0x23, 0x23};
                      gamma.send(buf, 1);
                      pktCount++;
                      TRACE("Sent ack.\n");
                      end = true;
                      break;*/
                }
            }
        }

        printf("Bytes received: %d\nDropped: %d,Time:%d ms\n", index, lostBytes,
               (int)(endT - startT));
        printf("Speed: %.3f KB/s\n",
               index / ((endT - startT) / 1024.0f) / 1024.0f);
        /*  for (int i = 0; i < index; i++)
          {
              printf("%c", inputBuf[i]);
          }
          printf("\n");*/

        lostBytes = 0;
        index     = 0;
        state     = 0;
        end       = false;
    }
    return 0;
}
