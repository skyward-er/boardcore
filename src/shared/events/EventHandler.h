/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#include <events/Event.h>
#include <utils/Debug.h>
#include <utils/collections/SyncCircularBuffer.h>

#include "ActiveObject.h"

namespace Boardcore
{

class EventHandlerBase
{
public:
    EventHandlerBase() {}

    virtual ~EventHandlerBase() {};

    virtual void postEvent(const Event& ev) = 0;
};

template <size_t QueueSize = DEFAULT_SIZE>
class EventHandler : public EventHandlerBase, public ActiveObject
{
public:
    EventHandler(unsigned int stacksize    = miosix::STACK_DEFAULT_FOR_PTHREAD,
                 miosix::Priority priority = miosix::MAIN_PRIORITY);

    virtual ~EventHandler();

    void postEvent(const Event& ev) override;

    void stop() override;

protected:
    virtual void handleEvent(const Event& ev) = 0;

    void run() override;

    SyncCircularBuffer<Event, QueueSize> eventList;
};

template <size_t QueueSize>
inline EventHandler<QueueSize>::EventHandler(unsigned int stacksize,
                                             miosix::Priority priority)
    : ActiveObject(stacksize, priority)
{
}

template <size_t QueueSize>
inline EventHandler<QueueSize>::~EventHandler(){};

template <size_t QueueSize>
inline void EventHandler<QueueSize>::postEvent(const Event& ev)
{
    eventList.tryPut(ev);
}

template <size_t QueueSize>
inline void EventHandler<QueueSize>::stop()
{
    if (isRunning())
    {
        stopFlag = true;

        // Put empty event in the list to wake the runner thread
        eventList.tryPut(Event{EV_EMPTY});

        thread->join();
    }
}

template <size_t QueueSize>
inline void EventHandler<QueueSize>::run()
{
    while (!shouldStop())
    {
        Event ev = eventList.tryPop();
        handleEvent(ev);
    }
}

}  // namespace Boardcore
