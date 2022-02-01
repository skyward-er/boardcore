/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Federico Terraneo, Matteo Piazzolla
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

#include <math/Stats.h>

#include <cstdint>
#include <ostream>

namespace Boardcore
{

/**
 * @brief Statistics over a single task.
 *
 * 3 statistics are provided:
 * - Activation statistics: errors between the task intended execution tick and
 * the actual execution tick when it started;
 * - Period statistics: actual period between task executions;
 * - Workload statistics: time (in ticks) the task took to execute;
 * - Mixed events: cumulative number of missed executions;
 * - Failed events: Number of events ended with exceptions.
 */
struct TaskStatsResult
{
    uint8_t id;
    StatsResult activationStats;
    StatsResult periodStats;
    StatsResult workloadStats;
    uint32_t missedEvents;
    uint32_t failedEvents;

    static std::string header()
    {
        return "id,actMin,actMax,actMean,actStddev,actNsamples,"
               "periodMin,periodMax,periodMean,period_stddev,"
               "periodNSamples,workloadMin,workloadMax,workloadMean,"
               "workload_stddev,workloadNSample,missedEvents,failedEvents\n";
    }

    void print(std::ostream& os) const
    {
        os << (int)id << "," << activationStats.minValue << ","
           << activationStats.maxValue << "," << activationStats.mean << ","
           << activationStats.stdDev << "," << activationStats.nSamples << ","
           << periodStats.minValue << "," << periodStats.maxValue << ","
           << periodStats.mean << "," << periodStats.stdDev << ","
           << periodStats.nSamples << "," << workloadStats.minValue << ","
           << workloadStats.maxValue << "," << workloadStats.mean << ","
           << workloadStats.stdDev << "," << workloadStats.nSamples << ","
           << missedEvents << "," << failedEvents << "\n";
    }
};

}  // namespace Boardcore
