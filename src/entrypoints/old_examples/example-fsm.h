/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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
 * Basic State Machine example. Explaination of this example and a graphical
 * representation of the state machine can be found on the Skyward Boardcore
 * Wiki on Gitlab:
 * https://git.skywarder.eu/r2a/skyward-boardcore/blob/master/src/entrypoints/examples/state-machines-examples.md
 *
 * https://git.skywarder.eu/r2a/skyward-boardcore/wikis/State-Machines-Examples-and-Best-Practices
 */

#pragma once

#include <events/EventBroker.h>
#include <events/FSM.h>

/*
 * Enum containing custom events definitions
 */
enum ExampleEvents : uint8_t
{
    EV_A = EV_FIRST_SIGNAL,  // The first event must always have value
                             // EV_FIRST_SIGNAL
    EV_B,  // Values for the following event can be manually specified or
           // assigned automatically
    EV_C,
    EV_D,
    EV_E
};

enum ExampleTopics : uint8_t
{
    TOPIC_T1
};

/**
 * @brief Logs every event received by the state machine
 * This function can be expanded to log every event / transition of the state
 * machine for later analysis, and as an aid for testing the state machine.
 *
 * @param state Current state of the state machine
 * @param ev The event being handled
 */
void traceState(uint8_t state, const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            printf("(S%d) Entering state\n", state);
            break;
        }
        case EV_EXIT:
        {
            printf("(S%d) Exiting state\n", state);
            break;
        }
        default:
        {
            printf("(S%d) Received event %d\n", state, ev.sig);
            break;
        }
    }
}

/*
 * Simple State machine definition
 *
 * The class inherits publicly from the FSM class, and must pass itself as a
 * template argument to the parent class.
 */
class FSMExample : public FSM<FSMExample>
{
public:
    enum States : uint8_t
    {
        STATE_S1 = 1,
        STATE_S2,
        STATE_S3,
        STATE_S4
    };

    /*
     * FSMExample constructor. The initial state of the state machine is passed
     * to the FSM superclass via the member initializer list.
     *
     * Initial conditions are also applied here. In this example, v is set to 0
     */
    FSMExample() : FSM(&FSMExample::state_S1), v(0)
    {
        // Subscribe for events posted on TOPIC_T1
        sEventBroker->subscribe(this, TOPIC_T1);
    }

    ~FSMExample() { sEventBroker->unsubscribe(this); }

private:
    /*
     * State function definitions.
     * EV_ENTRY and EV_EXIT are automatically dispatched by the state machine
     * when a transition occurs, and must be always handled even in the case of
     * no-op
     */

    void state_S1(const Event& ev)
    {
        // Print every event we receive in order to see the behaviour of the
        // state machine on the terminal.
        traceState(STATE_S1, ev);

        switch (ev.sig)
        {
            // It's always good to add braces to every single case statement, to
            // avoid problems
            case EV_ENTRY:
            {
                // no-op
                break;
            }
            case EV_EXIT:
            {
                // no-op
                break;
            }
            case EV_A:
            {
                if (v == 0)
                {
                    // perform a state transition to S2 when receiving EV_A and
                    // v == 0
                    transition(&FSMExample::state_S2);
                }
                else
                {
                    // else, transition to S4
                    transition(&FSMExample::state_S4);
                }
                break;
            }
            default:
            {
                break;
            }
        }
    }

    void state_S2(const Event& ev)
    {
        traceState(STATE_S2, ev);

        switch (ev.sig)
        {
            case EV_ENTRY:
            {
                break;
            }
            case EV_EXIT:
            {
                break;
            }
            case EV_C:  // perform a state transition to S3 when receiving EV_C
            {
                transition(&FSMExample::state_S3);
                break;
            }
            case EV_E:  // print hello world when receiving EV_E
            {
                printf("Hello world!\n");
                break;
            }
            default:
            {
                break;
            }
        }
    }

    void state_S3(const Event& ev)
    {
        traceState(STATE_S3, ev);

        switch (ev.sig)
        {
            case EV_ENTRY:
            {
                // Set v = 1
                v = 1;

                // Post EV_D to itself in 1 seconds
                delayed_ev_id =
                    sEventBroker->postDelayed(Event{EV_D}, TOPIC_T1, 1000);

                break;
            }
            case EV_EXIT:
            {
                // Remove the delayed event in case it has not fired yet
                sEventBroker->removeDelayed(delayed_ev_id);
                break;
            }
            case EV_B:
            {
                transition(&FSMExample::state_S4);
                break;
            }
            case EV_D:  // We will receive this event 1 second after entering
                        // this state.
            {
                transition(&FSMExample::state_S1);
                break;
            }
            default:
            {
                break;
            }
        }
    }

    // Final state: No outbound transitions, so the state machine cannot exit
    // this state
    void state_S4(const Event& ev)
    {
        traceState(STATE_S4, ev);

        switch (ev.sig)
        {
            case EV_ENTRY:
            {
                break;
            }
            case EV_EXIT:
            {
                break;
            }
            case EV_D:
            {
                transition(&FSMExample::state_S3);
                break;
            }
            default:
            {
                break;
            }
        }
    }

    uint16_t delayed_ev_id;
    int v;
};