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
#include <events/EventHandler.h>

namespace Boardcore
{

/**
 * @brief Helper class to count how many events are sent to the topic(s) it is
 * registered to.
 *
 * Useful if you want to check wether or not events are being
 * effectively posted
 */
class EventCounter : public EventHandler
{
public:
    /**
     * @brief Construct a new Event Counter object
     *
     * @param broker EventBroker to listen events to
     */
    EventCounter(EventBroker& broker) : broker(broker) {}

    ~EventCounter() { broker.unsubscribe(this); }

    /**
     * @brief Subscribes to a topic in the EventBroker
     *
     * @param topic
     */
    void subscribe(uint8_t topic) { broker.subscribe(this, topic); }

    // Override postEvent not to put events in a queue, just count the events it
    // receives.
    void postEvent(const Event& ev) override
    {
        Lock<FastMutex> l(mutex);

        ++mapCounter[ev];
        ++totalCount;

        lastEvent = ev;
    }

    /**
     * @brief Returns the number of times a specific event has been received.
     */
    unsigned int getCount(const Event& ev)
    {
        Lock<FastMutex> l(mutex);

        if (mapCounter.count(ev) == 1)
        {
            return mapCounter.at(ev);
        }

        return 0;
    }

    /**
     * @brief Returns how many events have been received in total
     */
    unsigned int getTotalCount() { return totalCount; }

    /**
     * @brief Returns the signature of the last event received (ev)
     */
    uint8_t getLastEvent() { return lastEvent; }

protected:
    // Do nothing
    void handleEvent(const Event& ev __attribute__((unused))) override{};

private:
    EventBroker& broker;

    FastMutex mutex;
    // Count how many times we have received each event
    map<uint8_t, unsigned int> mapCounter;

    // How many events we have received in total
    unsigned int totalCount = 0;
    uint8_t lastEvent;
};

}  // namespace Boardcore
