/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <utils/MovingAverage.h>

#include <thread>

#include "sx1278-init.h"

using namespace Boardcore;
using namespace miosix;

// Simple xorshift RNG
uint32_t xorshift32()
{
    // https://xkcd.com/221/
    static uint32_t STATE = 0x08104444;

    uint32_t x = STATE;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;

    return STATE = x;
}

struct TestMsg
{
    static constexpr int WORD_COUNT = 60;

    uint32_t payload[WORD_COUNT];

    void generate()
    {
        // Setup the buffer so that the xor of the full thing is 0
        uint32_t acc = 0;
        for (int i = 0; i < WORD_COUNT - 1; i++)
        {
            // Fill only the lower 8bit to simulate a lot of zeroes
            payload[i] = xorshift32() & 0xff;
            acc ^= payload[i];
        }
        payload[WORD_COUNT - 1] = acc;
    }

    bool validate()
    {
        uint32_t acc = 0;
        for (int i = 0; i < WORD_COUNT; i++)
            acc ^= payload[i];

        return acc == 0;
    }
};

struct LinkStats
{
    MovingAverage<uint64_t, 10> rx_interval;
    MovingAverage<uint64_t, 10> tx_interval;

    int recv_count      = 0;
    int sent_count      = 0;
    int corrupted_count = 0;

    float rssi = 0.0f;
    float fei  = 0.0f;
    float snr  = 0.0f;

    uint64_t txBitrate()
    {
        if (sent_count != 0)
            return (1000000 / tx_interval.getAverage()) * sizeof(TestMsg);
        else
            return 0;
    }

    uint64_t rxBitrate()
    {
        if (recv_count != 0)
            return (1000000 / rx_interval.getAverage()) * sizeof(TestMsg);
        else
            return 0;
    }
};

LinkStats stats;

void recvLoop()
{
    uint64_t last = 0;
    while (1)
    {
        TestMsg msg = {};

        sx1278->receive(reinterpret_cast<uint8_t *>(&msg), sizeof(msg));
        if (msg.validate())
        {
            stats.recv_count++;

            stats.rssi = sx1278->getLastRxRssi();
            stats.fei  = sx1278->getLastRxFei();
            stats.snr  = sx1278->getLastRxSnr();
        }
        else
        {
            stats.recv_count++;
            stats.corrupted_count++;
        }

        uint32_t cur = TimestampTimer::getTimestamp();
        stats.rx_interval.push(cur - last);
        last = cur;
    }
}

void sendLoop()
{
    uint64_t last = 0;
    while (1)
    {
        TestMsg msg = {};
        msg.generate();

        sx1278->send(reinterpret_cast<uint8_t *>(&msg), sizeof(msg));
        stats.sent_count++;

        uint32_t cur = TimestampTimer::getTimestamp();
        stats.tx_interval.push(cur - last);
        last = cur;

#if !(defined DISABLE_RX || defined DISABLE_TX)
        // Rate limit if we are both sending!
        Thread::sleep(500);
#endif
    }
}

void spawnThreads()
{
#ifndef DISABLE_RX
    std::thread recv([]() { recvLoop(); });
    recv.detach();
#endif
#ifndef DISABLE_TX
    std::thread send([]() { sendLoop(); });
    send.detach();
#endif

    // For now, I'll keep it here, just in case ...
    /* std::thread watchdog([]() {
        while(1) {
            {
                FastInterruptDisableLock dlock;
                sx1278->handleDioIRQ();
            }
            Thread::sleep(2);
        }
    });
    watchdog.detach();
    //*/
}
