/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Matteo Piazzolla
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

#include "EventBroker.h"

#include <diagnostic/StackLogger.h>
#include <utils/Debug.h>

namespace Boardcore
{

EventBroker::EventBroker() {}

void EventBroker::post(const Event& ev, uint8_t topic)
{
#ifdef TRACE_EVENTS
    LOG_DEBUG(logger, "Event: {}, Topic: {}", ev, topic);
#endif

    Lock<FastMutex> lock(mtxSubscribers);

    if (subscribers.count(topic) > 0)
    {
        vector<EventHandlerBase*>& subs = subscribers.at(topic);
        auto begin                      = subs.begin();
        auto end                        = subs.end();

        for (auto it = begin; it != end; it++)
        {
            // TODO: This may cause a deadlock if subscribe(...) in called in
            // postEvent(...), but it should never happen anyway. What to do?
            (*it)->postEvent(ev);
        }
    }
}

void EventBroker::removeDelayed(uint16_t id)
{
    Lock<FastMutex> lock(mtxDelayedEvents);
    for (auto it = delayedEvents.begin(); it != delayedEvents.end(); it++)
    {
        if ((*it).schedId == id)
        {
            delayedEvents.erase(it);
            break;
        }
    }
}

void EventBroker::subscribe(EventHandlerBase* subscriber, uint8_t topic)
{
    Lock<FastMutex> lock(mtxSubscribers);
    subscribers[topic].push_back(subscriber);
}

void EventBroker::unsubscribe(EventHandlerBase* subscriber, uint8_t topic)
{
    Lock<FastMutex> lock(mtxSubscribers);

    deleteSubscriber(subscribers.at(topic), subscriber);
}

void EventBroker::unsubscribe(EventHandlerBase* subscriber)
{
    Lock<FastMutex> lock(mtxSubscribers);
    for (auto it = subscribers.begin(); it != subscribers.end(); it++)
    {
        deleteSubscriber(it->second, subscriber);
    }
}

void EventBroker::clearDelayedEvents()
{
    Lock<FastMutex> lock(mtxDelayedEvents);
    delayedEvents.clear();
}

// Posts delayed events with expired deadline
void EventBroker::run()
{
    Lock<FastMutex> lock(mtxDelayedEvents);
    while (!shouldStop())
    {
        while (delayedEvents.size() > 0 &&
               delayedEvents.front().deadline <= getTick())
        {
            // Pop the first element
            DelayedEvent dev = delayedEvents.front();
            delayedEvents.erase(delayedEvents.begin());

            {
                // Unlock the mutex to avoid a deadlock if someone calls
                // postDelayed while receiving the event.
                Unlock<FastMutex> unlock(lock);
                post(dev.event, dev.topic);
            }
        }

        // When to wakeup for the next cycle
        long long sleepUntil =
            getTick() + EVENT_BROKER_MIN_DELAY * miosix::TICK_FREQ / 1000;

        if (delayedEvents.size() > 0)
        {
            // If a deadline expires earlier, sleep until the deadline instead
            if (delayedEvents.front().deadline < sleepUntil)
            {
                sleepUntil = delayedEvents.front().deadline;
            }
        }

        {
            // Unlock the mutex while sleeping
            Unlock<FastMutex> unlock(lock);
            StackLogger::getInstance().updateStack(THID_EVT_BROKER);

            Thread::sleepUntil(sleepUntil);
        }
    }
}

void EventBroker::deleteSubscriber(vector<EventHandlerBase*>& subs,
                                   EventHandlerBase* subscriber)
{
    auto it = subs.begin();

    while (it != subs.end())
    {
        if (*it == subscriber)
        {
            it = subs.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

}  // namespace Boardcore
