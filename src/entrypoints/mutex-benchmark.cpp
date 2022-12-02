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
 * FastMutex usage in code, and std::atomic variables, in a human understandable
 * manner.
 *
 * The way the measure of mutex overhead is done is the following: we have two
 * or more concurrent threads that have to repeat a simple operation, i.e.
 * writing a small block of data on SPI1, a certain number of times (defined by
 * OPERATIONS_PER_THREAD macro). Doing so, they will alternate as frequently as
 * defined by a varying parameter (`operationsBetweenLockGuards`, which
 * defines how many ops are run by each thread between lock() and unlock()).
 *
 * In order to test std::atomic, we measure the time needed to execute a certain
 * number of increments of a std::atomic<int>, an ordinary int (`volatile` to
 * force compiler not to skip ALU ops) and an int with mutex protection.
 *
 */
#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

#include <algorithm>
#include <atomic>
#include <cmath>

// Mutex benchmark options
#define RUN_MUTEX_BENCHMARK_TEST 1
#define DUMMY_BLOCK_SIZE 4
#define NUM_OPERATIONS 24000
#define MIN_OPS_BETWEEN_LOCK_GUARDS 1
#define MAX_OPS_BETWEEN_LOCK_GUARDS 3
#define STEP_OPS_BETWEEN_LOCK_GUARDS 1

// std::atomic/mutex comparison options
#define RUN_INCREMENTS_TEST 1
#define NUM_INCREMENTS 100000

// Multithreading options
#define MIN_THREADS 2
#define MAX_THREADS 10
#define STEP_THREADS 2

using namespace Boardcore;
using namespace miosix;

// Block of zeros to write on SPI
uint8_t dummy[DUMMY_BLOCK_SIZE] = {};

FastMutex mutex;
volatile int volatile_int;
std::atomic_int atomic_int;
SPIBus bus(SPI1);

struct MutexTestThreadData
{
    int numberOfOperations, numberOfConsecutiveOperations;
    uint64_t timestampStart, timestampEnd;
    bool useMutex;
};

struct IncrementTestThreadData
{
    uint64_t timestampStart, timestampEnd;
    bool useStdAtomic;
};

// `threadData` must point to the respective xxxThreadData struct.
void* mutexTestThreadRoutine(void* threadData);
void* incrementTestThreadRoutine(void* threadData);

int main()
{
    // Using default SPI configuration (we don't care)
    bus.configure(SPIBusConfig{});

#if RUN_MUTEX_BENCHMARK_TEST
    printf("*** FastMutex lock() and unlock() benchmark ***\n");

    // First, run the baseline test (no mutexes, no cry).
    {
        MutexTestThreadData threadData = {NUM_OPERATIONS, NUM_OPERATIONS, 0, 0,
                                          false};
        mutexTestThreadRoutine(&threadData);
        uint64_t executionTimeMs =
            (threadData.timestampEnd - threadData.timestampStart) / 1000ull;

        printf("Execution time without locks/unlocks: %lld ms\n",
               executionTimeMs);
    }

    // Now it's time for the real tests.
    for (int numberOfThreads = MIN_THREADS; numberOfThreads <= MAX_THREADS;
         numberOfThreads += STEP_THREADS)
    {
        const int operationsPerThread = NUM_OPERATIONS / numberOfThreads;

        for (int operationsBetweenLockGuards = MAX_OPS_BETWEEN_LOCK_GUARDS;
             operationsBetweenLockGuards >= MIN_OPS_BETWEEN_LOCK_GUARDS;
             operationsBetweenLockGuards -= STEP_OPS_BETWEEN_LOCK_GUARDS)
        {
            MutexTestThreadData threadDataArray[numberOfThreads];
            Thread* threads[numberOfThreads];
            for (int i = 0; i < numberOfThreads; i++)
            {
                // Start the concurrent threads
                threadDataArray[i] = {operationsPerThread,
                                      operationsBetweenLockGuards, 0, 0, true};
                threads[i] =
                    Thread::create(mutexTestThreadRoutine, 8096, Priority(),
                                   &threadDataArray[i], Thread::JOINABLE);
            }

            // Wait for all threads to end
            for (Thread* thread : threads)
            {
                thread->join();
            }

            // Take the minimum start time and the maximum end time
            uint64_t startTime =
                std::min_element(
                    threadDataArray, threadDataArray + numberOfThreads,
                    [](auto a, auto b)
                    { return a.timestampStart < b.timestampStart; })
                    ->timestampStart;
            uint64_t endTime =
                std::max_element(threadDataArray,
                                 threadDataArray + numberOfThreads,
                                 [](auto a, auto b)
                                 { return a.timestampEnd < b.timestampEnd; })
                    ->timestampEnd;

            uint64_t executionTimeMs = (endTime - startTime) / 1000ull;
            printf(
                "Execution time for %d locks/unlocks (%d threads): %lld ms\n",
                numberOfThreads *
                    (int)(std::ceil((double)(NUM_OPERATIONS / numberOfThreads) /
                                    operationsBetweenLockGuards)),
                numberOfThreads, executionTimeMs);
        }
    }
#endif

#if RUN_INCREMENTS_TEST
    printf("*** std::atomic<int> vs int vs mutex+int comparison ***\n");

    // Baseline test
    {
        uint64_t timestampStart, timestampEnd, executionTimeMs;

        timestampStart = TimestampTimer::getTimestamp();
        volatile_int   = 0;
        while (volatile_int < NUM_INCREMENTS)
        {
            ++volatile_int;
        }
        timestampEnd    = TimestampTimer::getTimestamp();
        executionTimeMs = (timestampEnd - timestampStart) / 1000ull;
        printf("Execution time for %d increments of volatile int: %lld ms\n",
               NUM_INCREMENTS, executionTimeMs);
    }

    for (int numberOfThreads = MIN_THREADS; numberOfThreads <= MAX_THREADS;
         numberOfThreads += STEP_THREADS)
    {
        for (bool useStdAtomic : {false, true})
        {
            IncrementTestThreadData threadDataArray[numberOfThreads];
            Thread* threads[numberOfThreads];
            volatile_int = 0;
            atomic_int   = 0;

            for (int i = 0; i < numberOfThreads; i++)
            {
                // Start the concurrent threads
                threadDataArray[i] = {0, 0, useStdAtomic};
                threads[i] =
                    Thread::create(incrementTestThreadRoutine, 8096, Priority(),
                                   &threadDataArray[i], Thread::JOINABLE);
            }

            // Wait for all threads to end
            for (Thread* thread : threads)
            {
                thread->join();
            }

            // Take the minimum start time and the maximum end time
            uint64_t startTime =
                std::min_element(
                    threadDataArray, threadDataArray + numberOfThreads,
                    [](auto a, auto b)
                    { return a.timestampStart < b.timestampStart; })
                    ->timestampStart;
            uint64_t endTime =
                std::max_element(threadDataArray,
                                 threadDataArray + numberOfThreads,
                                 [](auto a, auto b)
                                 { return a.timestampEnd < b.timestampEnd; })
                    ->timestampEnd;

            uint64_t executionTimeMs = (endTime - startTime) / 1000ull;
            printf(
                "Execution time for %d increments of %s (%d threads): %lld "
                "ms\n",
                NUM_INCREMENTS,
                useStdAtomic ? "atomic int" : "volatile int (with mutex)",
                numberOfThreads, executionTimeMs);
        }
    }
#endif

    return 0;
}

void* mutexTestThreadRoutine(void* param)
{
    MutexTestThreadData& threadData = *static_cast<MutexTestThreadData*>(param);
    threadData.timestampStart       = TimestampTimer::getTimestamp();

    for (int i = threadData.numberOfOperations; i > 0;
         i -= threadData.numberOfConsecutiveOperations)
    {
        if (threadData.useMutex)
        {
            mutex.lock();
        }

        for (int j = 0;
             j < std::min(i, threadData.numberOfConsecutiveOperations); j++)
        {
            bus.write(dummy, DUMMY_BLOCK_SIZE);
        }

        if (threadData.useMutex)
        {
            mutex.unlock();
        }
    }

    threadData.timestampEnd = TimestampTimer::getTimestamp();
    return nullptr;
}

void* incrementTestThreadRoutine(void* param)
{
    IncrementTestThreadData& threadData =
        *static_cast<IncrementTestThreadData*>(param);
    threadData.timestampStart = TimestampTimer::getTimestamp();

    // Note: volatile_int and atomic_int are set to 0 from the main thread.
    if (threadData.useStdAtomic)
    {
        do
        {
            ++atomic_int;
        } while (atomic_int < NUM_INCREMENTS);
    }
    else
    {
        mutex.lock();
        do
        {
            ++volatile_int;
            mutex.unlock();
            mutex.lock();
        } while (volatile_int < NUM_INCREMENTS);
        mutex.unlock();
    }

    threadData.timestampEnd = TimestampTimer::getTimestamp();
    return nullptr;
}