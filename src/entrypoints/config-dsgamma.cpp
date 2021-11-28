/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Nuno Barcellos
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

#include <stdio.h>
#include <string.h>

#include "miosix.h"

#include <drivers/gamma868/Gamma868.h>

using namespace Boardcore;
using namespace std;
using namespace miosix;

/* DISCOVERY F429I*/
typedef Gpio<GPIOA_BASE, 0> button;

// Gamma868 gamma("/dev/auxtty");  // create gamma object

void waitForButton();

int main()
{
    Gamma868 gamma("/dev/auxtty");  // create gamma object
    // Discovery gpio setup
    {
        FastInterruptDisableLock dLock;
        // greenLed::mode(Mode::OUTPUT);
        // redLed::mode(Mode::OUTPUT);
        button::mode(Mode::INPUT);
    }

    while (1)
    {
        printf("Press button to start gamma configuration.\n");
        waitForButton();

        // Write new configuration
        GammaConf newConf;

        newConf.local_addr[0] = 126;
        newConf.local_addr[1] = 126;
        newConf.local_addr[2] = 126;
        newConf.dest_addr[0]  = 126;
        newConf.dest_addr[1]  = 126;
        newConf.dest_addr[2]  = 126;
        newConf.lora_sf       = SF6;     // SF6
        newConf.lora_power    = dbm20;   //+20dbm
        newConf.handshake     = false;   // No handshake
        newConf.baudrate      = B_9600;  // 9600 baud

        // printf("LOCAL ADDRESS (3 bytes, 0-127 each):\n");
        // scanf("%d %d %d", &newConf.local_addr[0], &newConf.local_addr[1],
        //       &newConf.local_addr[2]);
        // printf("DESTINATION ADDRESS (3 bytes, 0-127 each):\n");
        // scanf("%d %d %d", &newConf.dest_addr[0], &newConf.dest_addr[1],
        //       &newConf.dest_addr[2]);
        // printf("LORA MODE (1-6):\n");
        // scanf("%d", &newConf.lora_mode);
        // printf("LORA POWER (0-15):\n");
        // scanf("%d", &newConf.lora_pow);
        // printf("HANDSHAKE (0-1):\n");
        // scanf("%d", &newConf.handshake);
        // printf("BAUDRATE (0-4):\n");
        // scanf("%d", &newConf.baudrate);

        bool configStatus = gamma.configure(newConf);
        if (configStatus == true)
        {
            printf("New configuration set!\n");
        }
        else
        {
            printf("Failed to set new configuration!\n");
        }
        Thread::sleep(500);
    }

    return 0;
}

/*
 * Waits for the discovery user button to be pressed (blocking).
 */
void waitForButton()
{
    while (1)
    {
        if (button::value() == 1)
            break;  // Wait for button
    }
}