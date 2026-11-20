/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci, Luca Erbetta
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

#include <events/Event.h>
#include <events/EventHandler.h>
#include <utils/collections/SyncQueue.h>

#include "ActiveObject.h"

namespace Boardcore
{

template <class T, size_t QueueSize = DEFAULT_SIZE>
class FSM : public EventHandler<QueueSize>
{
public:
    FSM(void (T::*initialState)(const Event&),
        unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
        miosix::Priority priority = miosix::MAIN_PRIORITY);

    virtual ~FSM();

    void transition(void (T::*nextState)(const Event&));

    /**
     * @brief Test if the FSM is in the given state.
     *
     * @param testState State to test.
     */
    bool testState(void (T::*testState)(const Event&));

protected:
    void handleEvent(const Event& e) override;

private:
    void (T::*state)(const Event&);
    Event specialEvent;
};

template <class T, size_t QueueSize>
FSM<T, QueueSize>::FSM(void (T::*initialState)(const Event&),
                       unsigned int stacksize, miosix::Priority priority)
    : EventHandler<QueueSize>(stacksize, priority)
{
    state        = initialState;
    specialEvent = EV_ENTRY;
    this->postEvent(specialEvent);
}

template <class T, size_t QueueSize>
FSM<T, QueueSize>::~FSM(){};

template <class T, size_t QueueSize>
void FSM<T, QueueSize>::transition(void (T::*nextState)(const Event&))
{
    specialEvent = EV_EXIT;
    (static_cast<T*>(this)->*state)(specialEvent);
    state        = nextState;
    specialEvent = EV_ENTRY;
    (static_cast<T*>(this)->*state)(specialEvent);
}

template <class T, size_t QueueSize>
bool FSM<T, QueueSize>::testState(void (T::*testState)(const Event&))
{
    return (this->state == testState);
}

template <class T, size_t QueueSize>
void FSM<T, QueueSize>::handleEvent(const Event& e)
{
    (static_cast<T*>(this)->*state)(e);
}

}  // namespace Boardcore
