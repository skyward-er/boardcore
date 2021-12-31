/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <sensors/VN100/VN100Serial.h>

#include <string>

using namespace miosix;
using namespace std;
using namespace Boardcore;

int main()
{
    char recvString[200];
    // Object string with the message
    string message("Communication\n\r");

    GpioPin tx(GPIOB_BASE, 6);
    GpioPin rx(GPIOB_BASE, 7);

    tx.mode(miosix::Mode::ALTERNATE);
    rx.mode(miosix::Mode::ALTERNATE);

    tx.alternateFunction(7);
    rx.alternateFunction(7);

    // Serial inferface
    VN100Serial serial{1, 115200};

    if (!serial.init())
    {
        printf("Init failed\n");
        return 0;
    }
    printf("Success!\n");

    // Loop to send the data via serial
    while (1)
    {
        serial.send(message.c_str(), message.length());
        // Sleep
        miosix::Thread::sleep(100);
        // Receive the string
        serial.recv(recvString, 200);
        // Print it out
        printf("%s\n", recvString);
    }

    serial.closeSerial();

    return 0;
}
