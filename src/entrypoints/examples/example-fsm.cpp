/* 
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "example-fsm.h"
#include <miosix.h>
#include <cstdio>

// Sleep helper
void slp(unsigned int t = 100) { miosix::Thread::sleep(t); }

int main()
{
    FSMExample fsm;

    sEventBroker->start();  // Start broker thread
    fsm.start();            // Start FSM thread

    // State machine starts in state S1. Post EV_A to move to S2
    sEventBroker->post(Event{EV_A}, TOPIC_T1);

    // FSM now in State S2
    sEventBroker->post(Event{EV_E},
                       TOPIC_T1);  // This makes the FSM print hello world

    sEventBroker->post(Event{EV_C}, TOPIC_T1);  // Transition to S3

    miosix::Thread::sleep(
        100);  // Wait for the FSM thread to process the previous instructions

    printf("Waiting for the FSM to transition to S1\n");
    miosix::Thread::sleep(1000);

    // Since we've been in state S3, now v == 1 and EV_A will make the FSM
    // transition to S4 instead of S1
    sEventBroker->post(Event{EV_A}, TOPIC_T1);

    miosix::Thread::sleep(1000);
    printf("End\n");

    for (;;)
    {

        miosix::Thread::sleep(10000);
    }
}