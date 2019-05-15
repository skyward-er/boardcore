/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Matteo Michele Piazzolla
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

#include "EventBroker.h"
#include "Debug.h"

EventBroker::EventBroker()
{
}

void EventBroker::post(const Event& ev, uint8_t topic)
{
    #ifdef TRACE_EVENTS
        TRACE("[EventBroker] Event: %d, Topic: %d\n", ev.sig, topic);
    #endif

    Lock<FastMutex> lock(mtx_subscribers);

    if (subscribers.count(topic) > 0)
    {
        vector<EventHandlerBase*>& subs = subscribers.at(topic);
        auto begin                  = subs.begin();
        auto end                    = subs.end();

        for (auto it = begin; it != end; it++)
        {
            // TODO: This may cause a deadlock if subscribe(...) in called in
            // postEvent(...), but it should never happen anyway. What to do?
            (*it)->postEvent(ev);
        }
    }
}

uint16_t EventBroker::postDelayed(const Event& ev, uint8_t topic,
                                  unsigned int delay_ms)
{
    Lock<FastMutex> lock(mtx_delayed_events);

    // Delay in system ticks
    long long delay_ticks =
        static_cast<long long>(delay_ms * miosix::TICK_FREQ / 1000);

    DelayedEvent dev{eventCounter++, ev, topic, getTick() + delay_ticks};
    bool added = false;

    // Add the new event in the list, ordered by deadline (first = nearest
    // deadline)
    for (auto it = delayed_events.begin(); it != delayed_events.end(); it++)
    {
        if (dev.deadline < (*it).deadline)
        {
            delayed_events.insert(it, dev);
            added = true;
            break;
        }
    }

    if (!added)  // In case this is the last/only event in the list
    {
        delayed_events.push_back(dev);
    }

    return dev.sched_id;
}

void EventBroker::removeDelayed(uint16_t id)
{
    Lock<FastMutex> lock(mtx_delayed_events);
    for (auto it = delayed_events.begin(); it != delayed_events.end(); it++)
    {
        if ((*it).sched_id == id)
        {
            delayed_events.erase(it);
            break;
        }
    }
}

// Posts delayed events with expired deadline
void EventBroker::run()
{
    Lock<FastMutex> lock(mtx_delayed_events);
    while (!shouldStop())
    {
        while (delayed_events.size() > 0 &&
               delayed_events.front().deadline <= getTick())
        {
            // Pop the first element
            DelayedEvent dev = delayed_events.front();
            delayed_events.erase(delayed_events.begin());

            {
                // Unlock the mutex to avoid a deadlock if someone calls
                // postDelayed while receiving the event.
                Unlock<FastMutex> unlock(lock);
                post(dev.event, dev.topic);
            }
        }

        // When to wakeup for the next cycle
        long long sleep_until =
            getTick() + EVENT_BROKER_MIN_DELAY * miosix::TICK_FREQ / 1000;

        if (delayed_events.size() > 0)
        {
            // If a deadline expires earlier, sleep until the deadline instead
            if (delayed_events.front().deadline < sleep_until)
            {
                sleep_until = delayed_events.front().deadline;
            }
        }

        {
            // Unlock the mutex while sleeping
            Unlock<FastMutex> unlock(lock);
            Thread::sleepUntil(sleep_until);
        }
    }
}

void EventBroker::subscribe(EventHandlerBase* subscriber, uint8_t topic)
{
    Lock<FastMutex> lock(mtx_subscribers);
    subscribers[topic].push_back(subscriber);
}

void EventBroker::unsubscribe(EventHandlerBase* subscriber, uint8_t topic)
{
    Lock<FastMutex> lock(mtx_subscribers);

    deleteSubscriber(subscribers.at(topic), subscriber);
}

void EventBroker::unsubscribe(EventHandlerBase* subscriber)
{
    Lock<FastMutex> lock(mtx_subscribers);
    for (auto it = subscribers.begin(); it != subscribers.end(); it++)
    {
        deleteSubscriber(it->second, subscriber);
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

void EventBroker::clearDelayedEvents()
{
    Lock<FastMutex> lock(mtx_delayed_events);
    delayed_events.clear();
}
