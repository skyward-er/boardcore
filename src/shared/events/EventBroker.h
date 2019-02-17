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

// Minimum guaranteed delay for an event posted with postDelayed(...)
static const unsigned int EVENT_BROKER_MIN_DELAY = 250;

/**
 * The EventBroker class implements the pub-sub paradigm to dispatch events to
 * multiple objects. An object of type FSM can subscribe to a topic in the
 * public topics enum and publish an event into it. The event will be posted in
 * to each FSM object subscribed to that specific topic.
 */
class EventBroker : Singleton<EventBroker>, public ActiveObject
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
     * Posts an event after the specified delay
     * @param event
     * @param topic
     * @param delay_ms Delay in milliseconds. Events with delay shorter than
     * EVENT_BROKER_MIN_DELAY are not guaranteed to be posted in time.
     * @return Unique id representing the event in the delayed events list.
     */
    uint16_t postDelayed(const Event& event, uint8_t topic,
                         unsigned int delay_ms);

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
    void subscribe(EventHandler* subscriber, uint8_t topic);

    /**
     * @brief Unsubscribe an EventHandler from a specific topic
     * This function should be used only for testing purposes
     * @param subscriber
     * @param topic
     */
    void unsubscribe(EventHandler* subscriber, uint8_t topic);

    /**
     * @brief Unsubribe an EventHandler from all the topics it is subscribed to.
     * This function should be used only for testing purposes
     * @param subscriber
     */
    void unsubscribe(EventHandler* subscriber);
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

    void deleteSubscriber(vector<EventHandler*>& sub_vector,
                          EventHandler* subscriber);

    vector<DelayedEvent> delayed_events;
    FastMutex mtx_delayed_events;

    map<uint8_t, vector<EventHandler*>> subscribers;
    FastMutex mtx_subscribers;

    uint16_t eventCounter = 0;
};

#define sEventBroker Singleton<EventBroker>::getInstance()

#endif  // SRC_SHARED_EVENTS_EVENT_BROKER_H
