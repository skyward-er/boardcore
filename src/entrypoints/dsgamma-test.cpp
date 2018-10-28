

#include <stdio.h>
#include <string.h>
#include "Common.h"
#include "miosix.h"

using namespace std;
using miosix::Thread;
using miosix::FastInterruptDisableLock;
using miosix::Gpio;

#include <drivers/gamma868/Gamma868.h>
#include "drivers/HardwareTimer.h"

// Protocol config
//#define DATA_LEN 16384

/* DISCOVERY F429I*/
typedef Gpio<GPIOA_BASE, 0> button;
typedef HardwareTimer<uint32_t, 4> Clock;
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

    printf("Press the button to start receiving \n");
    // Wait for button
    Thread::sleep(1000);
    while (1)
    {
        if (button::value() == 1)
            break;
    }
    printf("Receiving...\n");

    //  uint8_t inputBuf[DATA_LEN];

    int index         = 0;
    int lost_bytes    = 0;
    uint8_t pkt_count = 0;

    int state = ST_STARTING;
    bool end  = false;

    uint32_t start_t, end_t;

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
                        TRACE("Starting!\n");
                        start_t = miosix::getTick();
                        // printf("%c", c);
                        //  inputBuf[index] = c;
                        ++index;
                        state   = ST_WAIT_END_FRAME;
                        start_t = miosix::getTick();
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
                        TRACE("Starting!\n");

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
                    if (c != 255 && c != pkt_count)
                    {
                        TRACE("Error: Expected: %d: %d\n", pkt_count, (int)c);
                        lost_bytes++;
                    }

                    if (c == 255 && pkt_count < 251)
                    {
                        printf("Packet end %d. lost: %d\n", pkt_count,
                               lost_bytes);
                        ++pkt_count;
                        // end_t = miosix::getTick();
                        state = ST_WAIT_START_FRAME;
                    }

                    if (c == 255 && pkt_count >= 252)
                    {
                        TRACE("Ending\n");

                        state = ST_SEND_DATA;
                    }
                    /* if (c != 255)
                         pkt_count = max(pkt_count, c);*/
                    break;
                }
                case ST_SEND_DATA:
                {
                    end_t = miosix::getTick();
                    end   = true;
                    break;
                    /*  uint8_t buf[] = {0x23, 0x23, 0x23, 0x23, 0x23};
                      gamma.send(buf, 1);
                      pkt_count++;
                      TRACE("Sent ack.\n");
                      end = true;
                      break;*/
                }
            }
        }

        printf("Bytes received: %d\nDropped: %d,Time:%d ms\n", index,
               lost_bytes, (int)(end_t - start_t));
        printf("Speed: %.3f KB/s\n",
               index / ((end_t - start_t) / 1024.0f) / 1024.0f);
        /*  for (int i = 0; i < index; i++)
          {
              printf("%c", inputBuf[i]);
          }
          printf("\n");*/

        lost_bytes = 0;
        index      = 0;
        state      = 0;
        end        = false;
    }
    return 0;
}
