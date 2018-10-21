/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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

#include <stdio.h>
#include <string.h>
#include "miosix.h"

using namespace std;
using namespace miosix;

#include <drivers/gamma868/Gamma868.h>


// Protocol config
#define DATA_LEN 5

/* DISCOVERY F429I*/
typedef Gpio<GPIOA_BASE, 0> button;


// RTT calculation
// long long sendTime = 0;

int main()
{
    // Discovery gpio setup
    {
        FastInterruptDisableLock dLock;
        button::mode(Mode::INPUT);
    }

    Gamma868 gamma("/dev/auxtty");  // create gamma object

    printf("Press the button to start receiving \n");
    // Wait for button
    while (1)
    {
        if (button::value() == 1)
            break;
    }

    uint8_t inputBuf[DATA_LEN];

    while (1)
    {
        // long long arrivalTime = miosix::getTick();
        printf("Reading: \n");
        gamma.receive(inputBuf,DATA_LEN);

        printf("Received: ");

        for (int i = 0; i < DATA_LEN; i++)
        {
            printf("%c\n", inputBuf[i]);
        }

        // int rtt = arrivalTime - sendTime;
        // printf("\nRTT: %d\n\n", rtt);

        gamma.send(inputBuf,DATA_LEN);
        // sendTime = miosix::getTick();
    }

    return 0;
}