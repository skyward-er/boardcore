/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
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

#include "boards/Homeone/DeploymentController/Deployment.h"
#include "boards/Homeone/Events.h"

using namespace HomeoneBoard;

int main()
{
    DeploymentController fsm;

    sEventBroker->start();
    fsm.start();

    Thread::sleep(2000);
    printf("Posting cut drogue\n");
    sEventBroker->post({EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
    Thread::sleep(1500);

    printf("Posting cut drogue TC\n");
    sEventBroker->post({EV_TC_CUT_FIRST_DROGUE}, TOPIC_TC);
    Thread::sleep(1500);

    printf("Posting cut main\n");
    sEventBroker->post({EV_TC_CUT_MAIN}, TOPIC_TC);
    Thread::sleep(1500);

    printf("Posting cut ALL\n");
    sEventBroker->post({EV_TC_CUT_ALL}, TOPIC_TC);
    Thread::sleep(5000);

    printf("Posting cut drogue then cut main\n");
    sEventBroker->post({EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
    Thread::sleep(300);
    sEventBroker->post({EV_TC_CUT_MAIN}, TOPIC_TC);
    Thread::sleep(5000);

    sEventBroker->post({EV_TC_NC_OPEN}, TOPIC_DEPLOYMENT);
    Thread::sleep(1000);

    printf("End\n");
    for (;;)
    {
        Thread::sleep(10000);
    }
}