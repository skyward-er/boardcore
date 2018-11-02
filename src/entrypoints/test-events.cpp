/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#include <Common.h>
#include <miosix.h>

#include "boards/Homeone/Events.h"
#include "boards/Homeone/Topics.h"
#include "events/EventBroker.h"
#include "events/FSM.h"
#include "events/Scheduler.h"

using namespace miosix;

using profiling1 = Gpio<GPIOD_BASE, 4>;

enum Topics : uint8_t
{
    TOPIC1,
    TOPIC2
};

enum TestEventSignal : uint8_t
{
    EV_EV1 = EV_FIRST_SIGNAL,
    EV_EV2,
    EV_DELAYED
};

class TestFSM : public FSM<TestFSM>
{
public:
    TestFSM() : FSM(&TestFSM::state0) {}

    ~TestFSM() {}

protected:
    void handleEvent(const Event& ev)
    {
        profiling1::low();
        float t = static_cast<float>(miosix::getTick()) / miosix::TICK_FREQ;
        printf("\nT: %.3f\n", t);
        FSM::handleEvent(ev);
    }

private:
    void state0(const Event& e)
    {
        switch (e.sig)
        {
            case EV_ENTRY:
                printf("Entering State 0\n");
                break;
            case EV_EXIT:
                printf("Exiting State 0\n");
                break;
            case EV_EV1:
                printf("Received EV_EV1\n");
                transition(&TestFSM::state1);
                break;
            case EV_EV2:
                printf("ERROR: Received EV_EV2\n");
                break;
            case EV_DELAYED:
                printf("ERROR: Received EV_DELAYED\n");
                break;
        }
    }

    void state1(const Event& e)
    {
        switch (e.sig)
        {
            case EV_ENTRY:
            {
                printf("Entering State 1\n");
                Event ev = {EV_DELAYED};
                sEventBroker->postDelayed(ev, TOPIC1, 2000);
                break;
            }
            case EV_EXIT:
                printf("Exiting State 1\n");
                break;

            case EV_EV1:
                printf("ERROR: Received EV_EV1\n");
                break;
            case EV_EV2:
                printf("ERROR: Received EV_EV2\n");
                break;
            case EV_DELAYED:
                printf("Received EV_DELAYED\n");
                transition(&TestFSM::state2);
                break;
        }
    }

    void state2(const Event& e)
    {
        switch (e.sig)
        {
            case EV_ENTRY:
            {
                printf("Entering State 2\n");
                Event ev      = {EV_DELAYED};
                delayed_ev_id = sEventBroker->postDelayed(ev, TOPIC1, 6000);
                break;
            }
            case EV_EXIT:
                printf("Exiting State 2\n");
                sEventBroker->removeDelayed(delayed_ev_id);
                break;

            case EV_EV1:
                printf("ERROR: Received EV_EV1\n");
                break;
            case EV_EV2:
                printf("Received EV_EV2\n");
                transition(&TestFSM::state3);
                break;
            case EV_DELAYED:
                printf("ERROR: Received EV_DELAYED\n");
                break;
        }
    }

    void state3(const Event& e)
    {
        switch (e.sig)
        {
            case EV_ENTRY:
            {
                printf("Entering State 3\n");
                break;
            }
            case EV_EXIT:
                printf("Exiting State 3\n");
                break;

            case EV_EV1:
                printf("ERROR: Received EV_EV1\n");
                break;
            case EV_EV2:
                printf("ERROR: Received EV_EV2\n");
                break;
            case EV_DELAYED:
                printf("ERROR: Received EV_DELAYED\n");
                break;
        }
    }

    uint8_t delayed_ev_id = 0;
};

// Test FSM

int main()
{
    TestFSM fsm;

    // Start active objects
    sEventBroker->start();
    fsm.start();

    profiling1::mode(Mode::OUTPUT);
    profiling1::low();
    printf("Test start.\n\n");

    Event ev1{EV_EV1};
    Event ev2{EV_EV2};

    // No one is subscribed yet
    // sEventBroker->post(ev1, TOPIC1);

    sEventBroker->subscribe(&fsm, TOPIC1);

    Thread::sleep(1000);
    sEventBroker->post(ev1, TOPIC2);
    Thread::sleep(1000);
    profiling1::high();
    sEventBroker->post(ev1, TOPIC1);
    Thread::sleep(5000);
    sEventBroker->post(ev2, TOPIC1);

    Thread::sleep(6000);

    printf("\n\n\nEnd\n");
    for (;;)
    {
        Thread::sleep(10000);
    }
}
