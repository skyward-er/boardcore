/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
 *
 * Permission is herbrokery granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
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

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <miosix.h>
#include <utils/catch.hpp>
#include <cstdio>

#include "events/EventBroker.h"
#include "utils/TestHelper.h"

using miosix::Thread;

//Uncertainty on the time of delivery of a delayed event, in ms
static const unsigned int EVENT_DELAY_UNCERTAINTY = 1;

enum TestTopics : uint8_t
{
    TOPIC_1,
    TOPIC_2,
    TOPIC_3
};

enum TestEvents : uint8_t
{
    EV_A,
    EV_B,
    EV_C,
    EV_D
};

TEST_CASE("EventBroker posts to different topics")
{
    Event ev;
    EventBroker broker;

    EventCounter sub1(broker);
    EventCounter sub2(broker);

    broker.subscribe(&sub1, TOPIC_1);
    broker.subscribe(&sub2, TOPIC_2);
    broker.subscribe(&sub1, TOPIC_2);

    // No events posted, so no events received
    REQUIRE(sub1.getTotalCount() == 0);
    REQUIRE(sub2.getTotalCount() == 0);

    SECTION("Post event on TOPIC_1, only sub1 should receive it")
    {
        ev.sig = EV_A;
        broker.post(ev, TOPIC_1);
        REQUIRE(sub1.getTotalCount() == 1);
        REQUIRE(sub1.getLastEvent() == ev.sig);
        REQUIRE(sub2.getTotalCount() == 0);
    }

    SECTION("Post event on TOPIC_2, both sub1 and sub2 should receive it")
    {
        ev.sig = EV_B;
        broker.post(ev, TOPIC_2);
        REQUIRE(sub1.getTotalCount() == 1);
        REQUIRE(sub2.getTotalCount() == 1);
        REQUIRE(sub1.getLastEvent() == ev.sig);
        REQUIRE(sub2.getLastEvent() == ev.sig);
    }

    SECTION("Post event on TOPIC_3, no one should receive it")
    {
        ev.sig = EV_C;
        broker.post(ev, TOPIC_3);

        REQUIRE(sub1.getTotalCount() == 0);
        REQUIRE(sub2.getTotalCount() == 0);
    }
}

TEST_CASE("EventHandlers can unsubscribe")
{
    Event ev{EV_A};
    EventBroker broker;

    EventCounter sub1(broker);
    EventCounter sub2(broker);
    EventCounter sub3(broker);

    broker.subscribe(&sub1, TOPIC_1);
    broker.subscribe(&sub1, TOPIC_2);

    broker.subscribe(&sub3, TOPIC_1);
    broker.subscribe(&sub3, TOPIC_2);

    // No events posted, so no events received
    REQUIRE(sub1.getTotalCount() == 0);
    REQUIRE(sub2.getTotalCount() == 0);
    REQUIRE(sub3.getTotalCount() == 0);

    // Check sub1 & sub are effectively subscribed
    broker.post(ev, TOPIC_1);
    broker.post(ev, TOPIC_2);

    REQUIRE(sub1.getTotalCount() == 2);
    REQUIRE(sub3.getTotalCount() == 2);

    SECTION("Unsubscribe sub1 from TOPIC_1")
    {
        broker.unsubscribe(&sub1, TOPIC_1);

        // No event should be received by sub1 via TOPIC_1
        broker.post(ev, TOPIC_1);
        REQUIRE(sub1.getTotalCount() == 2);
        REQUIRE(sub3.getTotalCount() == 3);

        // But it can still receive on TOPIC_2
        broker.post(ev, TOPIC_2);
        REQUIRE(sub1.getTotalCount() == 3);
        REQUIRE(sub3.getTotalCount() == 4);
    }

    SECTION("Unsubscribe sub1 from all topics")
    {
        broker.unsubscribe(&sub1);

        // No event should be received by sub2 via TOPIC_2
        broker.post(ev, TOPIC_1);
        REQUIRE(sub1.getTotalCount() == 2);
        REQUIRE(sub3.getTotalCount() == 3);

        // Neither via TOPIC_3
        broker.post(ev, TOPIC_2);
        REQUIRE(sub1.getTotalCount() == 2);
        REQUIRE(sub3.getTotalCount() == 4);
    }

    SECTION("Unsubscribe sub2 even if it not subscribed. Nothing should happen")
    {
        SECTION("Unsubscribe from TOPIC_1")
        {
            broker.unsubscribe(&sub2, TOPIC_1);

            broker.post(ev, TOPIC_1);
            broker.post(ev, TOPIC_2);

            REQUIRE(sub2.getTotalCount() == 0);

            // sub1 & sub3 should still receive events
            REQUIRE(sub1.getTotalCount() == 4);
            REQUIRE(sub3.getTotalCount() == 4);
        }

        SECTION("Unsubscribe from all topics")
        {
            broker.unsubscribe(&sub2);
            broker.post(ev, TOPIC_1);
            broker.post(ev, TOPIC_2);

            REQUIRE(sub2.getTotalCount() == 0);

            // sub1 & sub3 should still receive events
            REQUIRE(sub1.getTotalCount() == 4);
            REQUIRE(sub3.getTotalCount() == 4);
        }
    }
}

TEST_CASE("Events can be dalayed")
{
    Event ev{EV_A};
    EventBroker broker;

    EventCounter sub1(broker);

    broker.subscribe(&sub1, TOPIC_1);

    // Start the thread, required for delayed events.
    broker.start();

    // Post delayed event by 1000 ms
    broker.postDelayed(ev, TOPIC_1, 1000);

    Thread::sleep(1000 - EVENT_DELAY_UNCERTAINTY);
    // We shouldn't have received it yet
    REQUIRE(sub1.getTotalCount() == 0);
    Thread::sleep(EVENT_DELAY_UNCERTAINTY * 2);
    // Now it should have arrived
    REQUIRE(sub1.getTotalCount() == 1);

    // Stop the broker.
    broker.stop();
}