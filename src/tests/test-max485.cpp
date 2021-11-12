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

#include <Common.h>

#include "sensors/MBLoadCell/SerialInterface.h"
#include "string.h"
#include "thread"

using namespace miosix;

// control pins for the max485 attached to serial 1
using ctrlPin1_s1 = miosix::Gpio<GPIOC_BASE, 8>;
using ctrlPin2_s1 = miosix::Gpio<GPIOC_BASE, 9>;

// control pins for the max485 attached to serial 2
using ctrlPin1_s2 = miosix::Gpio<GPIOC_BASE, 1>;
using ctrlPin2_s2 = miosix::Gpio<GPIOC_BASE, 2>;


const char msg[5] = "ciao";
char rcv[64];

template <typename GPIO1, typename GPIO2>
void readSer(SerialInterface s)
{
    // setting to receiving
    GPIO1::low();
    GPIO2::low();
    s.recvData(rcv);
    TRACE("*received: '%s'*\n", rcv);
}

// serial1 -> serial2
void write1_read2(const char *data, SerialInterface s1, SerialInterface s2)
{
    function<void(void)> f = bind(readSer<ctrlPin1_s2, ctrlPin2_s2>, s2);
    thread t(f);

    // serial 1: setting to transmitting
    ctrlPin1_s1::high();
    ctrlPin2_s1::high();

    Thread::sleep(1000);

    TRACE("Sending: '%s'\n", data);
    s1.sendData(data);
    TRACE("Waiting for serial 2 to read\n");
    t.join();
    if (strcmp(data, rcv) == 0)
    {
        TRACE("Serial 1 -> serial 2 WORKING!\n");
    }
    else
    {
        TRACE("ERROR: Serial 1 -> serial 2!\n");
    }
}

// serial1 <- serial2
void read1_write2(const char *data, SerialInterface s1, SerialInterface s2)
{
    function<void(void)> f = bind(readSer<ctrlPin1_s1, ctrlPin2_s1>, s1);
    thread t(f);
    // serial 1: setting to transmitting
    ctrlPin1_s2::high();
    ctrlPin2_s2::high();

    Thread::sleep(1000);

    TRACE("Sending: '%s'\n", data);
    s2.sendData(data);
    TRACE("Waiting for serial 1 to read\n");
    t.join();
    if (strcmp(data, rcv) == 0)
    {
        TRACE("Serial 1 <- serial 2 WORKING!\n");
    }
    else
    {
        TRACE("ERROR: Serial 1 <- serial 2!\n");
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

    SerialInterface serial1(2400, 1, "ser1");
    SerialInterface serial2(2400, 2, "ser2");

    if (!serial1.init())
    {
        TRACE("[Serial1] Wrong initialization\n");
        return 1;
    }

    if (!serial2.init())
    {
        TRACE("[Serial2] Wrong initialization\n");
        return 1;
    }

    while (true)
    {
        TRACE("SERIAL 3 WORKING!\n");

        // testing transmission "serial 1 -> serial 2"
        write1_read2(msg, serial1, serial2);

        // testing transmission "serial 1 <- serial 2"
        {
            read1_write2(msg, serial1, serial2);
            TRACE("ser1<-ser2: '%s'\n", rcv);
        }

        Thread::sleep(1000);
    }

    return 0;
}
