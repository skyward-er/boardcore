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

#include <utils/collections/CircularBuffer.h>
#include <utils/collections/SyncCircularBuffer.h>

#include <catch2/catch.hpp>

TEST_CASE("CircularBuffer - Initialization tests")
{
    CircularBuffer<int, 5> b1;
    CircularBuffer<int, 1> b2;
    CircularBuffer<int, 5656> b3;

    REQUIRE(b1.getSize() == 5);
    REQUIRE(b2.getSize() == 1);
    REQUIRE(b3.getSize() == 5656);

    REQUIRE(b1.count() == 0);

    REQUIRE(b1.isEmpty());
    REQUIRE_FALSE(b1.isFull());

    // There are no elements to return

    REQUIRE_THROWS(b1.pop());

    REQUIRE_THROWS(b1.get());
}

TEST_CASE("CircularBuffer - Fill tests")
{
    CircularBuffer<int, 5> b1;

    b1.put(1);

    REQUIRE_FALSE(b1.isEmpty());
    REQUIRE_FALSE(b1.isFull());
    REQUIRE(b1.count() == 1);

    // Filling the buffer...
    b1.put(2);
    b1.put(3);
    b1.put(4);
    b1.put(5);

    REQUIRE_FALSE(b1.isEmpty());
    REQUIRE(b1.isFull());

    REQUIRE(b1.count() == 5);

    // Don't throw exceptions if we are adding elements to a full buffer
    REQUIRE_NOTHROW(b1.put(6));

    REQUIRE_FALSE(b1.isEmpty());
    REQUIRE(b1.isFull());
    REQUIRE(b1.count() == 5);  // Count still 5

    // Go around another time
    b1.put(7);
    b1.put(8);
    b1.put(9);
    b1.put(10);
    b1.put(11);
    REQUIRE_FALSE(b1.isEmpty());
    REQUIRE(b1.isFull());
    REQUIRE(b1.count() == 5);  // Count still 5
}

TEST_CASE("CircularBuffer - Get and pop")
{
    CircularBuffer<int, 5> b1;

    b1.put(1);
    b1.put(2);
    b1.put(3);
    b1.put(4);
    b1.put(5);

    SECTION("Get")
    {
        REQUIRE(b1.get() == 1);  // return the first element

        // Nothing has been removed
        REQUIRE(b1.isFull());
        REQUIRE(b1.count() == 5);

        b1.put(6);

        REQUIRE(b1.get() == 2);  // return the first element

        // Two consecutive calls return the same element
        REQUIRE(b1.get() == 2); 


        // Nothing has been removed
        REQUIRE(b1.isFull());
        REQUIRE(b1.count() == 5);
    }

    SECTION("Indexed get")
    {
        REQUIRE(b1.get(0) == 1);
        REQUIRE(b1.get(1) == 2);
        REQUIRE(b1.get(2) == 3);
        REQUIRE(b1.get(3) == 4);
        REQUIRE(b1.get(4) == 5);

        REQUIRE_THROWS(b1.get(5));
        REQUIRE_THROWS(b1.get(-1));

        b1.pop();
        b1.pop();
        REQUIRE(b1.get(0) == 3);
        REQUIRE(b1.get(1) == 4);
        REQUIRE(b1.get(2) == 5);
        REQUIRE_THROWS(b1.get(3) == 4);
        REQUIRE_THROWS(b1.get(4) == 5);

        b1.pop();
        b1.pop();
        b1.pop();

        REQUIRE_THROWS(b1.get(0));
    }

    SECTION("Pop")
    {
        SECTION("Pop 1")
        {
            REQUIRE(b1.pop() == 1);  // return the first element

            REQUIRE_FALSE(b1.isFull());
            REQUIRE_FALSE(b1.isEmpty());
            REQUIRE(b1.count() == 4);
        }

        SECTION("Pop 2")
        {
            b1.put(6);

            REQUIRE(b1.pop() == 2);  // return the first element
            REQUIRE(b1.pop() == 3);  // return the second element

            REQUIRE_FALSE(b1.isFull());
            REQUIRE_FALSE(b1.isEmpty());
            REQUIRE(b1.count() == 3);
        }

        SECTION("Pop until empty")
        {
            // Empty the buffer
            b1.pop();
            b1.pop();
            b1.pop();
            b1.pop();
            b1.pop();

            REQUIRE_FALSE(b1.isFull());
            REQUIRE(b1.isEmpty());
            REQUIRE(b1.count() == 0);

            REQUIRE_THROWS(b1.get());
            REQUIRE_THROWS(b1.pop());
        }
    }

    SECTION("Tail go-around")
    {
        b1.put(6);
        b1.put(7);
        b1.put(8);
        b1.put(9);
        b1.put(10);
        b1.put(11);

        // Nothing has been removed
        REQUIRE(b1.isFull());
        REQUIRE_FALSE(b1.isEmpty());
        REQUIRE(b1.count() == 5);
    }
}

/**
 * Same identical tests, but for SynchedCircularBuffer
 * 
 */

TEST_CASE("SyncCircularBuffer - Initialization tests")
{
    SyncCircularBuffer<int, 5> b1;

    REQUIRE(b1.getSize() == 5);

    REQUIRE(b1.count() == 0);

    REQUIRE(b1.isEmpty());
    REQUIRE_FALSE(b1.isFull());

    // There are no elements to return

    REQUIRE_THROWS(b1.pop());

    REQUIRE_THROWS(b1.get());
}

TEST_CASE("SyncCircularBuffer - Fill tests")
{
    SyncCircularBuffer<int, 5> b1;

    b1.put(1);

    REQUIRE_FALSE(b1.isEmpty());
    REQUIRE_FALSE(b1.isFull());
    REQUIRE(b1.count() == 1);

    // Filling the buffer...
    b1.put(2);
    b1.put(3);
    b1.put(4);
    b1.put(5);

    REQUIRE_FALSE(b1.isEmpty());
    REQUIRE(b1.isFull());

    REQUIRE(b1.count() == 5);

    // Don't throw exceptions if we are adding elements to a full buffer
    REQUIRE_NOTHROW(b1.put(6));

    REQUIRE_FALSE(b1.isEmpty());
    REQUIRE(b1.isFull());
    REQUIRE(b1.count() == 5);  // Count still 5

    // Go around another time
    b1.put(7);
    b1.put(8);
    b1.put(9);
    b1.put(10);
    b1.put(11);
    REQUIRE_FALSE(b1.isEmpty());
    REQUIRE(b1.isFull());
    REQUIRE(b1.count() == 5);  // Count still 5
}

TEST_CASE("SyncCircularBuffer - Get and pop")
{
    SyncCircularBuffer<int, 5> b1;

    b1.put(1);
    b1.put(2);
    b1.put(3);
    b1.put(4);
    b1.put(5);

    SECTION("Get")
    {
        REQUIRE(b1.get() == 1);  // return the first element

        // Nothing has been removed
        REQUIRE(b1.isFull());
        REQUIRE(b1.count() == 5);

        b1.put(6);

        REQUIRE(b1.get() == 2);  // return the first element

        // Two consecutive calls return the same element
        REQUIRE(b1.get() == 2); 


        // Nothing has been removed
        REQUIRE(b1.isFull());
        REQUIRE(b1.count() == 5);
    }

    SECTION("Indexed get")
    {
        REQUIRE(b1.get(0) == 1);
        REQUIRE(b1.get(1) == 2);
        REQUIRE(b1.get(2) == 3);
        REQUIRE(b1.get(3) == 4);
        REQUIRE(b1.get(4) == 5);

        REQUIRE_THROWS(b1.get(5));
        REQUIRE_THROWS(b1.get(-1));

        b1.pop();
        b1.pop();
        REQUIRE(b1.get(0) == 3);
        REQUIRE(b1.get(1) == 4);
        REQUIRE(b1.get(2) == 5);
        REQUIRE_THROWS(b1.get(3) == 4);
        REQUIRE_THROWS(b1.get(4) == 5);

        b1.pop();
        b1.pop();
        b1.pop();

        REQUIRE_THROWS(b1.get(0));
    }

    SECTION("Pop")
    {
        SECTION("Pop 1")
        {
            REQUIRE(b1.pop() == 1);  // return the first element

            REQUIRE_FALSE(b1.isFull());
            REQUIRE_FALSE(b1.isEmpty());
            REQUIRE(b1.count() == 4);
        }

        SECTION("Pop 2")
        {
            b1.put(6);

            REQUIRE(b1.pop() == 2);  // return the first element
            REQUIRE(b1.pop() == 3);  // return the second element

            REQUIRE_FALSE(b1.isFull());
            REQUIRE_FALSE(b1.isEmpty());
            REQUIRE(b1.count() == 3);
        }

        SECTION("Pop until empty")
        {
            // Empty the buffer
            b1.pop();
            b1.pop();
            b1.pop();
            b1.pop();
            b1.pop();

            REQUIRE_FALSE(b1.isFull());
            REQUIRE(b1.isEmpty());
            REQUIRE(b1.count() == 0);

            REQUIRE_THROWS(b1.get());
            REQUIRE_THROWS(b1.pop());
        }
    }

    SECTION("Tail go-around")
    {
        b1.put(6);
        b1.put(7);
        b1.put(8);
        b1.put(9);
        b1.put(10);
        b1.put(11);

        // Nothing has been removed
        REQUIRE(b1.isFull());
        REQUIRE_FALSE(b1.isEmpty());
        REQUIRE(b1.count() == 5);
    }
}