/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <vector>
#include "events/EventBroker.h"

using std::cout;
using std::function;
using std::string;
using std::vector;

/**
 * Class that subscribe to every possible topic and pretty prints every event it
 * receives
 */
class EventSniffer
{
    using NameFunction = function<string(uint8_t)>;

public:
    /**
     * @param broker Event broker to subscribe to
     * @param event_name_fun Function that takes the event sig and returns the
     * event name
     * @param topic_name_fun Function that takes a topic id and returns the
     * topic name
     */
    EventSniffer(EventBroker& broker, NameFunction event_name_fun,
                 NameFunction topic_name_fun)
        : broker(broker), event_name_fun(event_name_fun),
          topic_name_fun(topic_name_fun)
    {
        for (uint8_t t = 0; t < 256; t++)
        {
            sniffers.push_back(new Sniffer(*this, t));
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
        Sniffer(EventSniffer& parent, uint8_t topic) : parent(parent)
        {
            parent.broker.subscribe(this, topic);
            topic_name = parent.topic_name_fun(topic);
        }

        void postEvent(const Event& ev)
        {
            string ev_name = parent.event_name_fun(ev.sig);
            cout << "[EVENT] " << ev_name << " on " << topic_name << ".\n";
        }

        ~Sniffer() { parent.broker.unsubscribe(this); }

    private:
        EventSniffer& parent;
        string topic_name;
    };

    vector<Sniffer*> sniffers;

    EventBroker& broker;
    NameFunction event_name_fun;
    NameFunction topic_name_fun;
};