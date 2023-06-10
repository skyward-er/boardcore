/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

/**
 * OCCURRENT:
 *  - 2 x Max485 (ttl-RS485 adapter); with small changes this test could be
 * compatible with other adapters
 *  - 1 x ttl-USB adapter
 *
 * SETUP:
 *  - connect the ttl-USB adapter to the default serial port and insert in the
 * computer
 *  - wire up the first Max485 adapter to the first serial port and connect the
 * /RE-DE pins to the control pins PC8 and PC9
 *  - wire up the second Max485 adapter to the second serial port and connect
 * the /RE-DE pins to the control pins PC1 and PC2
 *  - connect the two Max485 adapters wiring A to A and B to B
 *
 *  WIRINGS:
 *  Max485 n1   |   stm32f407vg_discovery ser 1
 *      DI      |       PA9 / PB6
 *      DE      |       PC9
 *      RE      |       PC8
 *      RO      |       PA10 / PB7
 *      VCC     |       3.3/5 V
 *      GND     |       GND
 *
 *  Max485 n2   |   stm32f407vg_discovery ser 2
 *      DI      |       PA2
 *      DE      |       PC1
 *      RE      |       PC2
 *      RO      |       PA3
 *      VCC     |       3.3/5 V
 *      GND     |       GND
 *
 *  Max485 n1   |   Max485 n2
 *      A       |       A
 *      B       |       B
 *
 */

#include "drivers/usart/USART.h"
#include "string.h"
#include "thread"

using namespace miosix;
using namespace Boardcore;

// control pins for the max485 attached to serial 1
using ctrlPin1_s1 = miosix::Gpio<GPIOC_BASE, 8>;
using ctrlPin2_s1 = miosix::Gpio<GPIOC_BASE, 9>;

// control pins for the max485 attached to serial 2
using ctrlPin1_s2 = miosix::Gpio<GPIOC_BASE, 1>;
using ctrlPin2_s2 = miosix::Gpio<GPIOC_BASE, 2>;

char msg[64] = "Testing communication :D";
char rcv[64];
int baudrates[] = {2400,   9600,   19200,  38400,  57600,
                   115200, 230400, 256000, 460800, 921600};

// function for the thread that has to read from serial
void readSer(USARTInterface &s) {}

// Communicatio: src -> dst
template <typename GPIO1_src, typename GPIO2_src, typename GPIO1_dst,
          typename GPIO2_dst>
void testCommunication(char *data, USARTInterface &src, USARTInterface &dst)
{
    // resetting the buffer so precedent tests won't affect this one
    memset(rcv, 0, strlen(rcv) + 1);

    // src: setting to transmitting
    GPIO1_src::high();
    GPIO2_src::high();

    // dst: setting to receiving
    GPIO1_dst::low();
    GPIO2_dst::low();

    // thread that reads from serial
    std::thread t(
        [&]()
        {
            dst.readBlocking(rcv, 64);
            printf("\t<--%d received: \t'%s'\n", dst.getId(), rcv);
        });

    printf("\t-->%d sending: \t'%s'\n", src.getId(), data);
    src.writeString(data);
    t.join();

    if (strcmp(data, rcv) == 0)
    {
        printf("*** %d -> %d WORKING!\n", src.getId(), dst.getId());
    }
    else
    {
        printf("### ERROR: %d -> %d!\n", src.getId(), dst.getId());
    }
}

int main()
{
    {
        InterruptDisableLock dsb;
        ctrlPin1_s1::mode(Mode::OUTPUT);
        ctrlPin2_s1::mode(Mode::OUTPUT);
        ctrlPin1_s2::mode(Mode::OUTPUT);
        ctrlPin2_s2::mode(Mode::OUTPUT);
    }

    printf("*** SERIAL 3 WORKING!\n");
    for (unsigned int iBaud = 0;
         iBaud < sizeof(baudrates) / sizeof(baudrates[0]); iBaud++)
    {
        Thread::sleep(1000);

        // instantiating the two USART drivers
        USART serial1(USART1, baudrates[iBaud]);
        USART serial2(USART2, baudrates[iBaud]);

        printf("\n########################### %d\n", (int)baudrates[iBaud]);
        // testing transmission "serial 1 <- serial 2"
        testCommunication<ctrlPin1_s2, ctrlPin2_s2, ctrlPin1_s1, ctrlPin2_s1>(
            msg, serial2, serial1);

        // testing transmission "serial 1 -> serial 2"
        testCommunication<ctrlPin1_s1, ctrlPin2_s1, ctrlPin1_s2, ctrlPin2_s2>(
            msg, serial1, serial2);
    }

    return 0;
}
