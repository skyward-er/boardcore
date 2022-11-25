/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

/*
 * The aim of this entrypoint is to measure the time overhead caused by
 * FastMutex usage in code, in a human understandable manner.
 *
 * The way the measure is done is the following: we have two concurrent threads
 * that have to repeat a simple operation, i.e. writing a small block of data on
 * SPI1, a certain number of times (defined by OPERATIONS_PER_THREAD macro).
 * Doing so, they will alternate as frequently as defined by a varying parameter
 * (from now on, let's call it Fragmentation Level: ranging from 0.f to 1.f it
 * changes how often a thread will call a mutex lock/unlock).
 *
 * In order to calculate time overhead we run and time the two threads doing the
 * same number of operations sequentially, with minimal usage of mutexes (2
 * locks and 2 unlocks).
 *
 */

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include <cmath>

#define DUMMY_BLOCK_SIZE (4)
#define OPERATIONS_PER_THREAD (20000)
#define MIN_FRAG_LEVEL (0.6f)  /* 50%  */
#define MAX_FRAG_LEVEL (1.f)   /* 100% */
#define STEP_FRAG_LEVEL (0.1f) /* 10%  */

using namespace Boardcore;
using namespace miosix;

/* block of zeros to write on SPI */
uint8_t dummy[DUMMY_BLOCK_SIZE] = {};

FastMutex mutex;
SPIBus bus(SPI1);

struct Params
{
    int numberOfOperations;
    int numberOfConsecutiveOperations;
};

// `params` must point to a Params struct.
void* runTest(void* params);

int main()
{
    Params p;
    Thread *a, *b;
    uint64_t timeStart, timeEnd, reference, interval;

    /* Using default SPI configuration (we don't care) */
    bus.configure(SPIBusConfig{});

    /*
     * First of all, we time running time of the threads running sequentially.
     */
    p         = {OPERATIONS_PER_THREAD, OPERATIONS_PER_THREAD};
    timeStart = TimestampTimer::getTimestamp() / 1000ull;

    a = Thread::create(runTest, 8096, Priority(), &p, Thread::JOINABLE);
    b = Thread::create(runTest, 8096, Priority(), &p, Thread::JOINABLE);
    a->join();
    b->join();

    timeEnd   = TimestampTimer::getTimestamp() / 1000ull;
    reference = timeEnd - timeStart;

    printf("Execution time without locks/unlocks: %lld ms\n", reference);

    for (float fragLevel = MIN_FRAG_LEVEL; fragLevel < MAX_FRAG_LEVEL + 0.001f;
         fragLevel += STEP_FRAG_LEVEL)
    {
        timeStart = TimestampTimer::getTimestamp() / 1000ull;

        p = {OPERATIONS_PER_THREAD,
             std::min(OPERATIONS_PER_THREAD,
                      static_cast<int>(std::round(1.f / fragLevel)))};
        a = Thread::create(runTest, 8096, Priority(), &p, Thread::JOINABLE);
        b = Thread::create(runTest, 8096, Priority(), &p, Thread::JOINABLE);

        a->join();
        b->join();

        timeEnd  = TimestampTimer::getTimestamp() / 1000ull;
        interval = timeEnd - timeStart;

        printf("Fragmentation Level: %2.1f%%\n", fragLevel * 100.f);
        printf("  Total execution time: %lld ms\n", interval);
        printf(
            "  # of FastMutex::lock(..) calls: %d\n",
            2 * static_cast<int>(std::ceil(OPERATIONS_PER_THREAD * fragLevel)));
        printf("  FastMutex Overhead: %lld ms (%+2.2lf%%)\n",
               (interval - reference),
               static_cast<double>(interval - reference) / reference * 100.f);
    }
}

void* runTest(void* params)
{
    Params* p = static_cast<Params*>(params);

    for (int i = p->numberOfOperations; i > 0;
         i -= p->numberOfConsecutiveOperations)
    {
        mutex.lock();
        for (int j = 0; j < std::min(i, p->numberOfConsecutiveOperations); j++)
        {
            bus.write(dummy, DUMMY_BLOCK_SIZE);
        }
        mutex.unlock();

        // We kindly ask the kernel to swap process
        Thread::yield();
    }

    return nullptr;
}