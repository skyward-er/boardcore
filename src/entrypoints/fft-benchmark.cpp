/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#pragma once

#include <algorithms/FFT.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/Stats/Stats.h>

#include <Eigen/Dense>
#include <array>

using namespace Boardcore;
using namespace std;

const unsigned int TAKES  = 5000;
const unsigned int BUFFER = 32;

array<float, TAKES> data;

/**
 * @brief Prints the test results for the specified buffer size.
 *
 * @param bufferSize Buffer size of the benchmark.
 * @param results Results form the benchmark.
 */
void printResults(size_t bufferSize, array<float, TAKES>& results);

int main()
{
    Eigen::Vector<float, BUFFER> input_signal =
        Eigen::Vector<float, BUFFER>::Zero();

    for (int i = 0; i < TAKES; i++)
    {
        for (size_t i = 0; i < input_signal.size(); i++)
        {
            input_signal(i) = (float)rand() / RAND_MAX;
        }

        int64_t duration = TimestampTimer::getTimestamp();
        FFT<BUFFER>::fft(input_signal);
        duration = TimestampTimer::getTimestamp() - duration;
        data[i]  = duration;
    }

    printResults(BUFFER, data);

    return 0;
}

void printResults(size_t bufferSize, array<float, TAKES>& results)
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
    printf("- mean: % 6.2f Hz\n", 1e6 / statsResults.mean);
    printf("- min:  % 6.2f Hz\n", 1e6 / statsResults.maxValue);
    printf("- max:  % 6.2f Hz\n", 1e6 / statsResults.minValue);

    printf("\n");
}
