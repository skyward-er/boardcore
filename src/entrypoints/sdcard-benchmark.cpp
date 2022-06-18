/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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
#include <stdint.h>
#include <utils/Stats/Stats.h>

#include <array>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <vector>

using namespace Boardcore;
using namespace miosix;
using namespace std;

///< Number of writes to perform for each buffer size
const unsigned int NUM_WRITES = 1000;

vector<size_t> BUFFER_SIZES = {128,  256,  512,   1024,  2048,
                               4096, 8192, 16348, 32768, 65536};

array<float, NUM_WRITES> data;

/**
 * @brief Fills a buffer with random bytes.
 *
 * @param buf Pointer to the buffer to fill.
 * @param size Size of the buffer
 */
void rndFill(uint8_t* buf, size_t size);

/**
 * @brief Writes random bufferSize bytes on the SD card and measure the duration
 * of each transaction.
 *
 * @param bufferSize Number of bytes to write.
 * @param results Array where to store the duration measured for each write.
 * @return True if no error encountered.
 */
bool benchmark(size_t bufferSize, array<float, NUM_WRITES>& results);

/**
 * @brief Prints the test results for the specified buffer size.
 *
 * @param bufferSize Buffer size of the benchmark.
 * @param results Results form the benchmark.
 */
void printResults(size_t bufferSize, array<float, NUM_WRITES>& results);

int main()
{
    srand(time(NULL));

    for (size_t bufferSize : BUFFER_SIZES)
    {
        if (benchmark(bufferSize, data))
            printResults(bufferSize, data);
        else
            printf("Error while performing the benchmark (buffer size: %lu)\n",
                   (unsigned long)bufferSize);
    }

    printf("Test completed\n");

    while (true)
        Thread::sleep(1000);
}

void rndFill(uint8_t* buf, size_t size)
{
    for (size_t i = 0; i < size; i++)
        buf[i] = rand() % 256;
}

bool benchmark(size_t bufferSize, array<float, NUM_WRITES>& results)
{
    uint8_t* buffer = new uint8_t[bufferSize];

    FILE* file = fopen("/sd/buf.dat", "w");

    if (file == nullptr)
    {
        printf("Error opening file!\n");
        // cppcheck-suppress nullPointerRedundantCheck
        // cppcheck-suppress nullPointer
        fclose(file);
        // cppcheck-suppress uninitdata
        delete[] buffer;
        return false;
    }

    for (unsigned int i = 0; i < NUM_WRITES; i++)
    {
        rndFill(buffer, bufferSize);

        // Time in microseconds (us)
        uint64_t duration = TimestampTimer::getTimestamp();

        size_t writeResult = fwrite(buffer, 1, bufferSize, file);

        duration = TimestampTimer::getTimestamp() - duration;

        if (writeResult != bufferSize)
        {
            printf("fwrite error: %d\n", ferror(file));
            fclose(file);
            delete[] buffer;
            return false;
        }

        results[i] = duration;
    }

    fclose(file);
    delete[] buffer;
    return true;
}

void printResults(size_t bufferSize, array<float, NUM_WRITES>& results)
{
    // Compute statistics on the benchmark results
    Stats stats;
    for (float result : results)
        stats.add(result);
    StatsResult statsResults = stats.getStats();

    printf("\tBuffer size: %lu\n", (unsigned long)bufferSize);
    printf("Times:\n");
    printf("- mean:    % 6.1f us\n", statsResults.mean);
    printf("- std dev: % 6.1f us\n", statsResults.stdDev);
    printf("- min:     % 6.1f us\n", statsResults.minValue);
    printf("- max:     % 6.1f us\n", statsResults.maxValue);
    printf("Speeds:\n");
    printf("- mean: % 6.2fKiB/s\n",
           bufferSize / (statsResults.mean / 1e6) / 1024);
    printf("- min:  % 6.2fKiB/s\n",
           bufferSize / (statsResults.maxValue / 1e6) / 1024);
    printf("- max:  % 6.2fKiB/s\n",
           bufferSize / (statsResults.minValue / 1e6) / 1024);

    printf("\n");
}
