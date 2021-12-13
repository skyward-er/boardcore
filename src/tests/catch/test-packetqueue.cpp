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

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <catch2/catch.hpp>

#define private public

#include <utils/collections/SyncPacketQueue.h>

using namespace Boardcore;

static constexpr int BUF_LEN   = 20;
static constexpr int PKT_LEN   = 10;
static constexpr int QUEUE_LEN = 3;

uint8_t message_base[BUF_LEN] = {'0', '1', '2', '3', '4', '5', '6',
                                 '7', '8', '9', 'a', 'b', 'c', 'd',
                                 'e', 'f', 'g', 'h', 'i', 'j'};

uint8_t buf[BUF_LEN];

inline bool COMPARE(const uint8_t* buf, size_t len, const char* expected)
{
    for (size_t i = 0; i < len && expected[i] != '\0'; ++i)
    {
        CAPTURE(i);
        REQUIRE(buf[i] == expected[i]);
    }

    return true;
}

inline bool COMPARE(Packet<PKT_LEN> pkt, const char* expected)
{
    pkt.dump(buf);
    return COMPARE(buf, pkt.maxSize(), expected);
}

TEST_CASE("Packet tests")
{
    std::fill(buf, buf + BUF_LEN, 0);
    Packet<PKT_LEN> p;

    SECTION("Empty packet tests")
    {
        REQUIRE(p.isEmpty());
        REQUIRE(p.isFull() == false);
        REQUIRE(p.isReady() == false);
        REQUIRE(p.size() == 0);
        REQUIRE(p.maxSize() == PKT_LEN);

        REQUIRE(p.msgCount() == 0);
        // REQUIRE(p.dump(buf) == 0);
        for (int i = 0; i < BUF_LEN; i++)
        {
            REQUIRE(buf[i] == 0);
        }
    }

    SECTION("Adding stuff to packet")
    {

        REQUIRE(p.tryAppend(message_base, 5));
        uint64_t ts = p.timestamp();
        REQUIRE(miosix::getTick() - ts < 5);
        REQUIRE(p.dump(buf) == 5);
        COMPARE(buf, BUF_LEN, "01234");

        REQUIRE(p.isEmpty() == false);
        REQUIRE(p.size() == 5);
        REQUIRE(p.msgCount() == 1);

        REQUIRE(p.tryAppend(message_base + 5, 3));
        REQUIRE(p.dump(buf) == 8);
        COMPARE(buf, BUF_LEN, "01234567");
        REQUIRE(p.isEmpty() == false);
        REQUIRE(p.size() == 8);

        REQUIRE_FALSE(p.tryAppend(message_base + 8, 3));
        REQUIRE(p.dump(buf) == 8);
        COMPARE(buf, BUF_LEN, "01234567");
        REQUIRE(p.isEmpty() == false);
        REQUIRE(p.size() == 8);
        REQUIRE(p.msgCount() == 2);

        REQUIRE(p.tryAppend(message_base + 8, 2));
        REQUIRE(p.dump(buf) == 10);
        COMPARE(buf, BUF_LEN, "0123456789");
        REQUIRE(p.isEmpty() == false);
        REQUIRE(p.isFull());
        REQUIRE(p.size() == 10);
        REQUIRE(p.msgCount() == 3);

        REQUIRE(p.timestamp() == ts);

        p.clear();
        REQUIRE(p.isEmpty());
        REQUIRE(p.isFull() == false);
        REQUIRE(p.isReady() == false);
        REQUIRE(p.size() == 0);
        REQUIRE(p.maxSize() == PKT_LEN);

        REQUIRE(p.msgCount() == 0);
        REQUIRE(p.dump(buf) == 0);
    }

    SECTION("Edge cases")
    {
        INFO("Adding empty msg");
        REQUIRE_FALSE(p.tryAppend(message_base, 0));
        REQUIRE(p.isEmpty());
        REQUIRE(p.isFull() == false);
        REQUIRE(p.isReady() == false);
        REQUIRE(p.size() == 0);
        REQUIRE(p.maxSize() == PKT_LEN);

        REQUIRE(p.msgCount() == 0);
        REQUIRE(p.dump(buf) == 0);

        INFO("Adding too big msg");
        REQUIRE_FALSE(p.tryAppend(message_base, PKT_LEN + 1));

        REQUIRE(p.isEmpty());
        REQUIRE(p.isFull() == false);
        REQUIRE(p.isReady() == false);
        REQUIRE(p.size() == 0);
        REQUIRE(p.maxSize() == PKT_LEN);

        REQUIRE(p.msgCount() == 0);
        REQUIRE(p.dump(buf) == 0);

        INFO("Adding something to full packet")
        REQUIRE(p.tryAppend(message_base, PKT_LEN));
        REQUIRE_FALSE(p.tryAppend(message_base, 1));

        REQUIRE(p.isEmpty() == false);
        REQUIRE(p.isFull());
        REQUIRE(p.isReady() == false);
        REQUIRE(p.size() == PKT_LEN);
        REQUIRE(p.maxSize() == PKT_LEN);

        REQUIRE(p.msgCount() == 1);
        REQUIRE(p.dump(buf) == 10);
        COMPARE(buf, 10, "0123456789");
    }
}

TEST_CASE("PacketQueue tests")
{
    SyncPacketQueue<PKT_LEN, QUEUE_LEN> pq;

    REQUIRE_FALSE(pq.isFull());
    REQUIRE(pq.isEmpty());
    REQUIRE(pq.countNotEmpty() == 0);
    REQUIRE(pq.countReady() == 0);
    REQUIRE_THROWS(pq.get());
    REQUIRE_THROWS(pq.pop());

    SECTION("Normal operation")
    {
        INFO("Adding two elements to first packet");
        REQUIRE(pq.put(message_base, 4) == 0);
        REQUIRE(pq.put(message_base, 4) == 0);

        REQUIRE(pq.countReady() == 0);
        REQUIRE(pq.countNotEmpty() == 1);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        INFO("Adding third element and filling first packet");
        REQUIRE(pq.put(message_base, 2) == 0);

        REQUIRE(pq.countReady() == 1);
        REQUIRE(pq.countNotEmpty() == 1);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        Packet<PKT_LEN> p = pq.get();
        REQUIRE(p.msgCount() == 3);
        REQUIRE(p.isFull());
        REQUIRE(p.isReady());
        COMPARE(p, "0123012301");

        INFO("Adding element to second packet");
        REQUIRE(pq.put(message_base + 10, 4) == 0);

        REQUIRE(pq.countReady() == 1);
        REQUIRE(pq.countNotEmpty() == 2);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());
        REQUIRE_FALSE(pq.buffer.get(1).isReady());

        COMPARE(pq.buffer.get(1), "abcd");

        p = pq.get();  // Should still return first packet
        REQUIRE(p.msgCount() == 3);

        INFO(
            "Adding element not fitting the second packet, added to the third");
        REQUIRE(pq.put(message_base + 10, 7) == 0);
        p = pq.get();  // Should still return first packet
        REQUIRE(p.msgCount() == 3);

        REQUIRE(pq.countReady() == 2);
        REQUIRE(pq.countNotEmpty() == 3);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        REQUIRE(pq.buffer.get(0).isReady());
        REQUIRE(pq.buffer.get(1).isReady());
        REQUIRE_FALSE(pq.buffer.get(2).isReady());

        COMPARE(pq.buffer.get(0), "0123012301");
        COMPARE(pq.buffer.get(1), "abcd");
        COMPARE(pq.buffer.get(2), "abcdefg");

        INFO("Popping first element");
        p = pq.pop();  // Should still return first packet
        REQUIRE(p.msgCount() == 3);
        COMPARE(p, "0123012301");

        // Should now return what was the second element
        COMPARE(pq.get(), "abcd");

        REQUIRE(pq.buffer.get(0).isReady());
        REQUIRE_FALSE(pq.buffer.get(1).isReady());

        REQUIRE(pq.countReady() == 1);
        REQUIRE(pq.countNotEmpty() == 2);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        INFO("Adding a msg back to the first packet and filling it");
        REQUIRE(pq.put(message_base, 10) == 0);
        REQUIRE(pq.countReady() == 3);
        REQUIRE(pq.countNotEmpty() == 3);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE(pq.isFull());

        COMPARE(pq.buffer.get(0), "abcd");
        COMPARE(pq.buffer.get(1), "abcdefg");
        COMPARE(pq.buffer.get(2), "0123456789");

        INFO("Popping everything");

        p = pq.pop();
        COMPARE(p, "abcd");
        REQUIRE(p.isReady());

        p = pq.pop();
        COMPARE(p, "abcdefg");
        REQUIRE(p.isReady());

        p = pq.pop();
        COMPARE(p, "0123456789");
        REQUIRE(p.isReady());

        REQUIRE_FALSE(pq.isFull());
        REQUIRE(pq.isEmpty());
        REQUIRE(pq.countNotEmpty() == 0);
        REQUIRE(pq.countReady() == 0);
        REQUIRE_THROWS(pq.get());
        REQUIRE_THROWS(pq.pop());
    }

    SECTION("Edge cases")
    {
        INFO("Adding too big msg")
        REQUIRE(pq.put(message_base, PKT_LEN + 1) == -1);
        REQUIRE_FALSE(pq.isFull());
        REQUIRE(pq.isEmpty());
        REQUIRE(pq.countNotEmpty() == 0);
        REQUIRE(pq.countReady() == 0);

        INFO("Adding empty message")
        REQUIRE(pq.put(message_base, 0) == -1);

        INFO("Adding something to full queue")
        REQUIRE(pq.put(message_base, PKT_LEN) == 0);
        REQUIRE(pq.put(message_base + 5, PKT_LEN) == 0);
        REQUIRE(pq.put(message_base + 10, PKT_LEN) == 0);

        REQUIRE(pq.buffer.count() == 3);
        for (int i = 0; i < 3; i++)
        {
            CAPTURE(i);
            REQUIRE(pq.buffer.get(i).isReady());
        }
        REQUIRE(pq.isFull());

        REQUIRE(pq.put(message_base, PKT_LEN) == 1);
        REQUIRE(pq.buffer.last().isReady());
        REQUIRE(pq.isFull());

        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE(pq.countNotEmpty() == 3);
        REQUIRE(pq.countReady() == 3);

        INFO("Get/Pop on empty queue")
        REQUIRE_NOTHROW(pq.pop());
        REQUIRE_NOTHROW(pq.pop());
        REQUIRE_NOTHROW(pq.pop());

        REQUIRE_THROWS(pq.get());
        REQUIRE_THROWS(pq.pop());

        REQUIRE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());
    }
}
