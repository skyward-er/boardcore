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

#include <functional>
#include <iostream>
#include <string>
#include <vector>

using std::cout;
using std::function;
using std::string;
using std::vector;

namespace Boardcore
{

/**
 * Class that subscribe to many topics and calls a callback when an event is
 * received.
 */
class EventSniffer
{
    using OnEventReceived = function<void(uint8_t, uint8_t)>;

public:
    /**
     * EventSniffer that sniffs only the specified topics
     * @param broker Event broker to subscribe to
     * @param topics Which topics to sniff
     * @param onEventReceived Callback to call upon receiving an event
     */
    EventSniffer(EventBroker& broker, vector<uint8_t> topics,
                 OnEventReceived onEventReceived)
        : broker(broker), onEventReceived(onEventReceived)
    {
        for (uint8_t t : topics)
        {
            sniffers.push_back(new Sniffer(*this, t));
        }
    }

    /**
     * EventSniffer that sniffs all the possible topics (0-255)
     * @param broker Event broker to subscribe to
     * @param onEventReceived Callback to call upon receiving an event
     */
    EventSniffer(EventBroker& broker, OnEventReceived onEventReceived)
        : broker(broker), onEventReceived(onEventReceived)
    {
        for (int t = 0; t <= 255; t++)
        {
            sniffers.push_back(new Sniffer(*this, (uint8_t)t));
        }
    }

    ~EventSniffer()
    {
        for (Sniffer* s : sniffers)
        {
            delete s;
        }
    }

private:
    class Sniffer : public EventHandlerBase
    {
    public:
        Sniffer(EventSniffer& parent, uint8_t topic)
            : parent(parent), topic(topic)
        {
            parent.broker.subscribe(this, topic);
        }

        void postEvent(const Event& ev) { parent.onEventReceived(ev, topic); }

        ~Sniffer() { parent.broker.unsubscribe(this); }

    private:
        EventSniffer& parent;
        uint8_t topic;
    };

    vector<Sniffer*> sniffers;

    EventBroker& broker;
    OnEventReceived onEventReceived;
};

}  // namespace Boardcore
