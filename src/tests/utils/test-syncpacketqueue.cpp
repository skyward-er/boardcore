/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/collections/SyncPacketQueue.h>

using namespace miosix;
using namespace Boardcore;

SyncPacketQueue<20, 10> queue;

void payloadGenerator();

int main()
{
    TaskScheduler scheduler;
    scheduler.addTask(payloadGenerator, 1000);
    scheduler.start();

    while (true)
    {
        queue.waitUntilNotEmpty();
        auto packet = queue.get();

        if (packet.isReady())
        {
            queue.pop();

            uint8_t packetContent[20];
            size_t size = packet.dump(packetContent);

            printf("[%.2f] ", TimestampTimer::getTimestamp() / 1e6);

            for (size_t i = 0; i < size; i++)
                printf("%c", packetContent[i]);

            printf(" size: %zu\n", size);
        }
        else
        {
            Thread::sleep(50);
        }
    }
}

void payloadGenerator()
{
    uint8_t msg[] = "012345678";
    queue.put(msg, 9);
    printf("Filled queue with 9 bytes: %s\n", msg);
}
