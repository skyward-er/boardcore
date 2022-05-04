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

uint8_t messageBase[BUF_LEN] = {'0', '1', '2', '3', '4', '5', '6',
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

        REQUIRE(p.getMsgCount() == 0);
        // REQUIRE(p.dump(buf) == 0);
        for (int i = 0; i < BUF_LEN; i++)
        {
            REQUIRE(buf[i] == 0);
        }
    }

    SECTION("Adding stuff to packet")
    {
        // Add 5 bytes
        REQUIRE(p.append(messageBase, 5));
        uint64_t ts = p.getTimestamp();

        REQUIRE(Boardcore::TimestampTimer::getInstance().getTimestamp() - ts <
                5);
        REQUIRE(p.dump(buf) == 5);
        COMPARE(buf, BUF_LEN, "01234");

        REQUIRE(p.isEmpty() == false);
        REQUIRE(p.size() == 5);
        REQUIRE(p.getMsgCount() == 1);

        // Add 3 bytes
        REQUIRE(p.append(messageBase + 5, 3));
        REQUIRE(p.dump(buf) == 8);
        COMPARE(buf, BUF_LEN, "01234567");
        REQUIRE(p.isEmpty() == false);
        REQUIRE(p.size() == 8);

        // Trying to add 3 more bytes, only 2 should be written
        REQUIRE(p.append(messageBase + 8, 3) == 2);
        REQUIRE(p.dump(buf) == PKT_LEN);
        COMPARE(buf, BUF_LEN, "0123456789");
        REQUIRE(p.isEmpty() == false);
        REQUIRE(p.size() == PKT_LEN);
        REQUIRE(p.getMsgCount() == 3);

        p.clear();
        REQUIRE(p.isEmpty());
        REQUIRE(p.isFull() == false);
        REQUIRE(p.isReady() == false);
        REQUIRE(p.size() == 0);
        REQUIRE(p.maxSize() == PKT_LEN);

        REQUIRE(p.getMsgCount() == 0);
        REQUIRE(p.dump(buf) == 0);
    }

    SECTION("Edge cases")
    {
        INFO("Adding empty msg");
        REQUIRE_FALSE(p.append(messageBase, 0));
        REQUIRE(p.isEmpty());
        REQUIRE(p.isFull() == false);
        REQUIRE(p.isReady() == false);
        REQUIRE(p.size() == 0);
        REQUIRE(p.maxSize() == PKT_LEN);

        REQUIRE(p.getMsgCount() == 0);
        REQUIRE(p.dump(buf) == 0);

        INFO("Adding too big msg");
        REQUIRE(p.append(messageBase, PKT_LEN + 1) == PKT_LEN);

        REQUIRE_FALSE(p.isEmpty());
        REQUIRE(p.isFull());
        REQUIRE_FALSE(p.isReady());
        REQUIRE(p.size() == PKT_LEN);
        REQUIRE(p.maxSize() == PKT_LEN);

        REQUIRE(p.getMsgCount() == 1);
        REQUIRE(p.dump(buf) == PKT_LEN);
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
        REQUIRE(pq.put(messageBase, 4));
        REQUIRE(pq.put(messageBase, 4));

        // No packet should be ready
        REQUIRE(pq.countReady() == 0);
        REQUIRE(pq.countNotEmpty() == 1);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        INFO("Adding third element and filling first packet");
        REQUIRE(pq.put(messageBase, 2));

        // Now one single packet should be filled and ready
        REQUIRE(pq.countReady() == 1);
        REQUIRE(pq.countNotEmpty() == 1);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        // Check the packet content
        Packet<PKT_LEN> p = pq.get();
        REQUIRE(p.getMsgCount() == 3);
        REQUIRE(p.isFull());
        REQUIRE(p.isReady());
        COMPARE(p, "0123012301");

        INFO("Adding more data to create a second packet");
        REQUIRE(pq.put(messageBase + 10, 4));

        // The second packet should not be ready
        REQUIRE(pq.countReady() == 1);
        REQUIRE(pq.countNotEmpty() == 2);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());
        REQUIRE_FALSE(pq.buffer.get(1).isReady());
        COMPARE(pq.buffer.get(1), "abcd");

        p = pq.get();  // Should still return first packet
        REQUIRE(p.getMsgCount() == 3);

        INFO("Adding more data to create a third packet");
        REQUIRE(pq.put(messageBase + 10, 7));

        p = pq.get();  // Should still return first packet
        REQUIRE(p.getMsgCount() == 3);

        // Check all the packages
        REQUIRE(pq.buffer.get(0).isReady());
        REQUIRE(pq.buffer.get(0).size() == PKT_LEN);
        COMPARE(pq.buffer.get(0), "0123012301");
        REQUIRE(pq.buffer.get(1).isReady());
        REQUIRE(pq.buffer.get(1).size() == PKT_LEN);
        COMPARE(pq.buffer.get(1), "abcdabcdef");
        REQUIRE_FALSE(pq.buffer.get(2).isReady());
        REQUIRE(pq.buffer.get(2).size() == 1);
        COMPARE(pq.buffer.get(2), "g");

        // Check the queue stats
        REQUIRE(pq.countReady() == 2);
        REQUIRE(pq.countNotEmpty() == 3);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        INFO("Popping first element");
        p = pq.pop();  // Should still return first packet
        REQUIRE(p.getMsgCount() == 3);
        COMPARE(p, "0123012301");

        // The packets should now be shifted
        REQUIRE(pq.buffer.get(0).isReady());
        REQUIRE(pq.buffer.get(0).size() == PKT_LEN);
        COMPARE(pq.buffer.get(0), "abcdabcdef");
        REQUIRE_FALSE(pq.buffer.get(1).isReady());
        REQUIRE(pq.buffer.get(1).size() == 1);
        COMPARE(pq.buffer.get(1), "g");

        REQUIRE(pq.countReady() == 1);
        REQUIRE(pq.countNotEmpty() == 2);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        INFO("Adding more data to fill the last packet");
        REQUIRE(pq.put(messageBase, 10));
        REQUIRE(pq.countReady() == 2);
        REQUIRE(pq.countNotEmpty() == 3);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());

        // We should now have three packets
        REQUIRE(pq.buffer.get(0).isReady());
        REQUIRE(pq.buffer.get(0).size() == PKT_LEN);
        COMPARE(pq.buffer.get(0), "abcdabcdef");
        REQUIRE(pq.buffer.get(1).isReady());
        REQUIRE(pq.buffer.get(1).size() == PKT_LEN);
        COMPARE(pq.buffer.get(1), "g012345678");
        REQUIRE_FALSE(pq.buffer.get(2).isReady());
        REQUIRE(pq.buffer.get(2).size() == 1);
        COMPARE(pq.buffer.get(2), "9");

        // If we now add another 10 bytes the last packet, the last byte which
        // does not fit should be put in a new packet at the start of the queue
        REQUIRE(pq.put(messageBase, 10));
        REQUIRE(pq.countReady() == 2);
        REQUIRE(pq.countNotEmpty() == 3);
        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());
        REQUIRE(pq.buffer.get(0).isReady());
        REQUIRE(pq.buffer.get(0).size() == PKT_LEN);
        COMPARE(pq.buffer.get(0), "g012345678");
        REQUIRE(pq.buffer.get(1).isReady());
        REQUIRE(pq.buffer.get(1).size() == PKT_LEN);
        COMPARE(pq.buffer.get(1), "9012345678");
        REQUIRE_FALSE(pq.buffer.get(2).isReady());
        REQUIRE(pq.buffer.get(2).size() == 1);
        COMPARE(pq.buffer.get(2), "9");

        // And now by adding the last 9 bytes the queue should be marked ready
        REQUIRE(pq.put(messageBase + 10, 9));
        REQUIRE(pq.countReady() == 3);
        REQUIRE(pq.countNotEmpty() == 3);
        REQUIRE(pq.buffer.get(2).isReady());
        REQUIRE(pq.buffer.get(2).size() == PKT_LEN);
        COMPARE(pq.buffer.get(2), "9abcdefghi");

        INFO("Popping everything");

        p = pq.pop();
        REQUIRE(p.isReady());
        REQUIRE(p.size() == PKT_LEN);
        COMPARE(p, "g012345678");

        p = pq.pop();
        COMPARE(p, "9012345678");
        REQUIRE(p.isReady());

        p = pq.pop();
        COMPARE(p, "9abcdefghi");
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
        INFO("Adding too big msg");
        REQUIRE_FALSE(pq.put(messageBase, PKT_LEN * QUEUE_LEN + 1));
        REQUIRE_FALSE(pq.isFull());
        REQUIRE(pq.isEmpty());
        REQUIRE(pq.countNotEmpty() == 0);
        REQUIRE(pq.countReady() == 0);

        INFO("Adding empty message");
        REQUIRE_FALSE(pq.put(messageBase, 0));

        INFO("Adding something to full queue");
        REQUIRE(pq.put(messageBase, PKT_LEN));
        REQUIRE(pq.put(messageBase + 5, PKT_LEN));
        REQUIRE(pq.put(messageBase + 10, PKT_LEN));

        REQUIRE(pq.buffer.count() == 3);
        for (int i = 0; i < 3; i++)
        {
            CAPTURE(i);
            REQUIRE(pq.buffer.get(i).isReady());
        }
        REQUIRE(pq.isFull());

        REQUIRE(pq.put(messageBase, PKT_LEN) == 1);
        REQUIRE(pq.buffer.last().isReady());
        REQUIRE(pq.isFull());

        REQUIRE_FALSE(pq.isEmpty());
        REQUIRE(pq.countNotEmpty() == 3);
        REQUIRE(pq.countReady() == 3);

        INFO("Get/Pop on empty queue");
        REQUIRE_NOTHROW(pq.pop());
        REQUIRE_NOTHROW(pq.pop());
        REQUIRE_NOTHROW(pq.pop());

        REQUIRE_THROWS(pq.get());
        REQUIRE_THROWS(pq.pop());

        REQUIRE(pq.isEmpty());
        REQUIRE_FALSE(pq.isFull());
    }
}
