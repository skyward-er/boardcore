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

// Maximum lenght of the sleep in the event broker run method, in ms.
#define EVENT_BROKER_MAX_SLEEP 250;

EventBroker::EventBroker() : ActiveObject()  // TODO: Specify stack size
{
}

void EventBroker::post(const Event& ev, uint8_t topic)
{
    Lock<Mutex> lock(mtx_subscribers);

    auto begin = subscribers[topic].begin();
    auto end   = subscribers[topic].end();

    for (auto it = begin; it != end; it++)
    {
        (*it)->postEvent(ev);  // TODO: Should not pass a pointer to a local
                               // object
    }
}

uint16_t EventBroker::postDelayed(const Event& ev, uint8_t topic,
                                  unsigned int delay_ms)
{
    Lock<Mutex> lock(mtx_delayed_events);

    // Delay in system ticks
    long long delay_ticks =
        static_cast<long long>(delay_ms * miosix::TICK_FREQ / 1000);

    DelayedEvent dev = {eventCounter++, ev, topic, getTick() + delay_ticks};
    bool added       = false;

    // Add the new event in the list, ordered by deadline
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
    Lock<Mutex> lock(mtx_delayed_events);
    for (auto it = delayed_events.begin(); it != delayed_events.end(); it++)
    {
        if ((*it).sched_id == id)
        {
            delayed_events.erase(it);
            break;
        }
    }
}

void EventBroker::run()
{
    Lock<Mutex> lock(mtx_delayed_events);
    while (true)
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
                Unlock<Mutex> unlock(lock);
                post(dev.event, dev.topic);
            }
        }

        // How long to sleep in this cycle
        unsigned int sleep_ms = EVENT_BROKER_MAX_SLEEP;

        if (delayed_events.size() > 0)
        {
            long long interval_ticks =
                delayed_events.front().deadline - getTick();

            // Can only happen if the deadline has expired between the last time
            // we checked its deadline and here
            if (interval_ticks < 0)
            {
                interval_ticks = 0;
            }

            // Time until the next deadline in ms
            unsigned int interval_ms = static_cast<unsigned int>(
                interval_ticks / miosix::TICK_FREQ * 1000);

            if (sleep_ms > interval_ms)
            {
                sleep_ms = interval_ms;
            }
        }

        {
            // Unlock the mutex while sleeping
            Unlock<Mutex> unlock(lock);
            Thread::sleep(sleep_ms);
        }
    }
}

void EventBroker::subscribe(FSMBase* subscriber, uint8_t topic)
{
    Lock<Mutex> lock(mtx_subscribers);
    subscribers[topic].push_back(subscriber);
}
