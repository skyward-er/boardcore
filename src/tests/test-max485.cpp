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
 * IMPLIED, INCLUDING BUT NOT LI
 * MITED TO THE WARRANTIES OF MERCHANTABILITY,
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
 *  Max485 n1   |   stm32f407vg_discovery
 *      DI      |       PA9
 *      DE      |       PC9
 *      RE      |       PC8
 *      RO      |       PA10
 *      VCC     |       3.3/5 V
 *      GND     |       GND
 *
 *  Max485 n2   |   stm32f407vg_discovery
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
 * WARNINGS:
 * - the baudrate has to be extremely low. "guaranteed" at 2400
 */

#include <Common.h>

#include "sensors/MBLoadCell/SerialInterface.h"
#include "string.h"
#include "thread"

#define BAUDRATE 2400

using namespace miosix;

// control pins for the max485 attached to serial 1
using ctrlPin1_s1 = miosix::Gpio<GPIOC_BASE, 8>;
using ctrlPin2_s1 = miosix::Gpio<GPIOC_BASE, 9>;

// control pins for the max485 attached to serial 2
using ctrlPin1_s2 = miosix::Gpio<GPIOC_BASE, 1>;
using ctrlPin2_s2 = miosix::Gpio<GPIOC_BASE, 2>;

char msg[64] = "Testing communication :D";
char rcv[64];

// function for the thread that has to read from serial
void readSer(SerialInterface s)
{
    s.recvString(rcv, 64);
    printf("\t<--%s received: \t'%s'\n", s.getPortName().c_str(), rcv);
}

// Communicatio: src -> dst
template <typename GPIO1_src, typename GPIO2_src, typename GPIO1_dst,
          typename GPIO2_dst>
void testCommunication(char *data, SerialInterface src, SerialInterface dst)
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
    thread t(readSer, dst);

    printf("\t-->%s sending: \t'%s'\n", src.getPortName().c_str(), data);
    src.sendString(data);
    t.join();

    if (strcmp(data, rcv) == 0)
    {
        printf("*** %s -> %s WORKING!\n", src.getPortName().c_str(),
               dst.getPortName().c_str());
    }
    else
    {
        printf("### ERROR: %s -> %s!\n", src.getPortName().c_str(),
               dst.getPortName().c_str());
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

    // Setting the baudrate to 2400, maximum functioning baudrate for the Max485
    // adapters
    SerialInterface serial1(BAUDRATE, 1, "ser1");
    SerialInterface serial2(BAUDRATE, 2, "ser2");

    if (!serial1.init())
    {
        printf("[Serial1] Wrong initialization\n");
        return 1;
    }

    if (!serial2.init())
    {
        printf("[Serial2] Wrong initialization\n");
        return 1;
    }

    while (true)
    {
        printf("\n###########################\n");
        printf("*** SERIAL 3 WORKING!\n");

        // testing transmission "serial 1 <- serial 2"
        testCommunication<ctrlPin1_s2, ctrlPin2_s2, ctrlPin1_s1, ctrlPin2_s1>(
            msg, serial2, serial1);

        // testing transmission "serial 1 -> serial 2"
        testCommunication<ctrlPin1_s1, ctrlPin2_s1, ctrlPin1_s2, ctrlPin2_s2>(
            msg, serial1, serial2);

        Thread::sleep(1000);
    }

    return 0;
}
