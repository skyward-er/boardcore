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

#ifndef SRC_SHARED_EVENTS_EVENT_BROKER_H
#define SRC_SHARED_EVENTS_EVENT_BROKER_H

#include "ActiveObject.h"
#include "Singleton.h"

#include "events/Event.h"
#include "events/FSM.h"

#include <miosix.h>
#include <stdint.h>

#include <map>
#include <vector>

using std::map;
using std::vector;

using miosix::FastMutex;
using miosix::getTick;
using miosix::Lock;
using miosix::Thread;
using miosix::Unlock;

// Minimum guaranteed delay for an event posted with postDelayed(...) in ms
static constexpr unsigned int EVENT_BROKER_MIN_DELAY = 50;

/**
 * The EventBroker class implements the pub-sub paradigm to dispatch events to
 * multiple objects. An object of type FSM can subscribe to a topic in the
 * public topics enum and publish an event into it. The event will be posted in
 * to each FSM object subscribed to that specific topic.
 */
class EventBroker : public Singleton<EventBroker>, public ActiveObject
{
    friend class Singleton<EventBroker>;

public:
    /**
     * Posts an event to the specified topic.
     * @param ev
     * @param topic
     */
    void post(const Event& ev, uint8_t topic);

    /**
     * Posts an event after the specified delay.
     * @warning Events cannot be posted with a delay shorter than
     * EVENT_BROKER_MIN_DELAY
     *
     * @param event
     * @param topic
     * @param delay_ms Delay in milliseconds.
     * @return Unique id of the delayed event.
     */
    template <unsigned int delay_ms>
    uint16_t postDelayed(const Event& ev, uint8_t topic)
    {
        static_assert(
            delay_ms >= EVENT_BROKER_MIN_DELAY,
            "delay_ms must be longer or equal to EVENT_BROKER_MIN_DELAY");

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

    /**
     * Removes a delayed event before it is posted.
     * @param id The id returned by postDelayed(...).
     */
    void removeDelayed(uint16_t id);

    /**
     * Subscribe to a specific topic.
     * DO NOT call it in response to an event, or it will cause a deadlock.
     * @param subscriber
     * @param topic
     */
    void subscribe(EventHandlerBase* subscriber, uint8_t topic);

    /**
     * @brief Unsubscribe an EventHandler from a specific topic
     * This function should be used only for testing purposes
     * @param subscriber
     * @param topic
     */
    void unsubscribe(EventHandlerBase* subscriber, uint8_t topic);

    /**
     * @brief Unsubribe an EventHandler from all the topics it is subscribed to.
     * This function should be used only for testing purposes
     * @param subscriber
     */
    void unsubscribe(EventHandlerBase* subscriber);

    /**
     * @brief Unschedules all pending events.
     * This function should be used only for testing purposes
     */
    void clearDelayedEvents();

    /**
     * @brief Construct a new Event Broker object.
     * Public access required for testing purposes. Use the singleton interface
     * to access this class in production code.
     *
     */
    EventBroker();

    virtual ~EventBroker(){};

private:
    /**
     * Private structure for holding a delayed event data in the list.
     */
    struct DelayedEvent
    {
        uint16_t sched_id;
        Event event;
        uint8_t topic;
        long long deadline;
    };

    /**
     * Active Object run
     */
    void run() override;

    void deleteSubscriber(vector<EventHandlerBase*>& sub_vector,
                          EventHandlerBase* subscriber);

    vector<DelayedEvent> delayed_events;
    FastMutex mtx_delayed_events;

    map<uint8_t, vector<EventHandlerBase*>> subscribers;
    FastMutex mtx_subscribers;

    uint16_t eventCounter = 0;
};

#define sEventBroker Singleton<EventBroker>::getInstance()

#endif  // SRC_SHARED_EVENTS_EVENT_BROKER_H
