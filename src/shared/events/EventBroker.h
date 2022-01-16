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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <events/Event.h>
#include <events/FSM.h>
#include <miosix.h>

#include <cstdint>
#include <map>
#include <vector>

#include "ActiveObject.h"
#include "Singleton.h"

using std::map;
using std::vector;

using miosix::FastMutex;
using miosix::getTick;
using miosix::Lock;
using miosix::Thread;
using miosix::Unlock;

namespace Boardcore
{

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
     * @param delayMs Delay in milliseconds.
     * @return Unique id of the delayed event.
     */
    template <unsigned int delayMs>
    uint16_t postDelayed(const Event& ev, uint8_t topic)
    {
        static_assert(
            delayMs >= EVENT_BROKER_MIN_DELAY,
            "delayMs must be longer or equal to EVENT_BROKER_MIN_DELAY");

        Lock<FastMutex> lock(mtxDelayedEvents);

        // Delay in system ticks
        long long delayTicks =
            static_cast<long long>(delayMs * miosix::TICK_FREQ / 1000);

        DelayedEvent dev{eventCounter++, ev, topic, getTick() + delayTicks};
        bool added = false;

        // Add the new event in the list, ordered by deadline (first = nearest
        // deadline)
        for (auto it = delayedEvents.begin(); it != delayedEvents.end(); it++)
        {
            if (dev.deadline < (*it).deadline)
            {
                delayedEvents.insert(it, dev);
                added = true;
                break;
            }
        }

        if (!added)  // In case this is the last/only event in the list
        {
            delayedEvents.push_back(dev);
        }

        return dev.schedId;
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
        uint16_t schedId;
        Event event;
        uint8_t topic;
        long long deadline;
    };

    /**
     * Active Object run
     */
    void run() override;

    void deleteSubscriber(vector<EventHandlerBase*>& subVector,
                          EventHandlerBase* subscriber);

    vector<DelayedEvent> delayedEvents;
    FastMutex mtxDelayedEvents;

    map<uint8_t, vector<EventHandlerBase*>> subscribers;
    FastMutex mtxSubscribers;

    uint16_t eventCounter = 0;

    PrintLogger logger = Logging::getLogger("eventbroker");
};

}  // namespace Boardcore

#define sEventBroker Boardcore::Singleton<Boardcore::EventBroker>::getInstance()
