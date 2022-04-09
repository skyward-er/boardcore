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

#pragma once

#include <events/EventBroker.h>
#include <events/FSM.h>
#include <events/HSM.h>
#include <events/utils/EventCounter.h>
#include <miosix.h>

#include <cmath>
#include <map>

using miosix::FastMutex;
using miosix::getTick;
using miosix::Lock;
using std::map;

namespace Boardcore
{

/*
 * How long should we wait for the state machine to handle the event?
 * Value in milliseconds
 */
static const int EVENT_TIMING_UNCERTAINTY = 1;

/**
 * @brief Helper function used convert system ticks to milliseconds
 */
long long tickToMilliseconds(long long tick);

/**
 * Tests if a specific transition occurs in a Finite State Machine
 * in response to an event.
 * Requires protected-level access to the FSM object, usually achieved by
 * "#define protected public"
 *
 * @tparam FSM_type Finite State Machine class to be tested
 * @param fsm FSM object reference
 * @param ev The event that should trigger the transition
 * @param expectedState Expected state after transition (state function
 * pointer)
 * @return true If the state machine is in the expected state after posting the
 * event
 * @return false If the machine is not in the expected state after posting the
 * event
 */
template <class FSM_type>
bool testFSMTransition(FSM_type& fsm, const Event& ev,
                       void (FSM_type::*expectedState)(const Event&))
{
    fsm.handleEvent(ev);
    return fsm.testState(expectedState);
}

/**
 * @brief Test if a specific transition occurs in a Finite State Machine
 * in response to an event, posted on a specific topic.
 * Once the event is posted, the state machine will process it asynchronously,
 * so we have to wait a little bit for it to happen. The wait time is defined in
 * EVENT_TIMING_UNCERTAINTY. If the state machine takes longer than this
 * time to process the event, the test will likely fail, as a transition will
 * not have occured.
 *
 * @tparam FSM_type Finite State Machine class to be tested
 * @param fsm FSM object reference
 * @param ev The event that should trigger the transition
 * @param topic Topic to post the event on
 * @param expectedState Expected state after transition (state function
 * pointer)
 * @param broker Eventbroker instance (Defaults to the singleton instance)
 * @return true If the state machine is in the expected state after posting the
 * event
 * @return false If the machine is not in the expected state after posting the
 * event
 */
template <class FSM_type>
bool testFSMAsyncTransition(FSM_type& fsm, const Event& ev, uint8_t topic,
                            void (FSM_type::*expectedState)(const Event&),
                            EventBroker& broker = sEventBroker)
{
    broker.post(ev, topic);
    // Wait for the event to be handled
    miosix::Thread::sleep(EVENT_TIMING_UNCERTAINTY);
    return fsm.testState(expectedState);
}

/**
 * @brief Test if a specific transition occurs in a Hierarchical State Machine
 * in response to an event.
 * Requires protected-level access to the HSM object, usually achieved by
 * "#define protected public"
 *
 * @tparam HSM_type Hierarchical State Machine class to be tested
 * @param fsm FSM object reference
 * @param ev The event that should trigger the transition
 * @param expectedState Expected state after transition (state function
 * pointer)
 * @return true If the state machine is in the expected state after posting the
 * event
 * @return false If the machine is not in the expected state after posting the
 * event
 */
template <class HSM_type>
bool testHSMTransition(HSM_type& hsm, const Event& ev,
                       State (HSM_type::*expectedState)(const Event&))
{
    hsm.handleEvent(ev);
    return hsm.testState(expectedState);
}

/**
 * @brief Test if a specific transition occurs in a Hierarchical State Machine
 * in response to an event, posted on a specific topic.
 * Once the event is posted, the state machine will process it asynchronously,
 * so we have to wait a little bit for it to happen. The wait time is defined in
 * EVENT_TIMING_UNCERTAINTY. If the state machine takes longer than this
 * time to process the event, the test will likely fail, as a transition will
 * not have occured.
 *
 * @tparam HSM_type Hierarchical State Machine class to be tested
 * @param fsm FSM object reference
 * @param ev The event that should trigger the transition
 * @param topic Topic to post the event on
 * @param expectedState Expected state after transition (state function
 * pointer)
 * @param broker Eventbroker instance (Defaults to the singleton instance)
 * @return true If the state machine is in the expected state after posting the
 * event
 * @return false If the machine is not in the expected state after posting the
 * event
 */
template <class HSM_type>
bool testHSMAsyncTransition(HSM_type& hsm, const Event& ev, uint8_t topic,
                            State (HSM_type::*expectedState)(const Event&),
                            EventBroker& broker = sEventBroker)
{
    broker.post(ev, topic);
    // Wait for the event to be handled
    miosix::Thread::sleep(EVENT_TIMING_UNCERTAINTY);
    return hsm.testState(expectedState);
}

/**
 * @brief Checks if an event is posted in a specific time window
 * The time window is defined by the parameters as follows:
 *
 * window = [when - uncertainty, when + uncertainty].
 *
 * If the event is posted inside the time
 * window, the function returns true. False otherwise.
 *
 * Use this function with DEBUG *undefined*, as printfs are very slow and will
 * mess with the timings, ultimately failing the tests.
 *
 * @param eventId The event to be checked
 * @param topic The topic the event will be posted on
 * @param when Expected time at which the event will be posted, in system ticks
 * @param uncertainty Size of the time window
 * @param broker
 * @return True if the event is posted inside the time window
 */
bool expectEvent(uint8_t eventId, uint8_t topic, long long when,
                 long long uncertainty = EVENT_TIMING_UNCERTAINTY,
                 EventBroker& broker   = sEventBroker);

/**
 * @brief Waits until the specified event is received or a timeout expires
 *
 * @param event_    id The event to be checked
 * @param topic     The topic the event will be posted on
 * @param timeout   How long to wait for the event before returning. 0 for
 *                  infinite timeout.
 * @param broker
 * @return true     if the event is received before expiration
 *         false    if the timeout has expired
 */
bool waitForEvent(uint8_t event, uint8_t topic, long long timeout = 0,
                  EventBroker& broker = sEventBroker);

}  // namespace Boardcore
