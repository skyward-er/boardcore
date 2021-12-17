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

#include <math/Stats.h>

#include <cstdint>
#include <ostream>

#pragma once

namespace Boardcore
{

/**
 * Statistics for a task
 */
struct TaskStatResult
{
    uint8_t id;                   ///< Task id
    StatsResult activationStats;  ///< Task activation stats
    StatsResult periodStats;      ///< Task period stats
    StatsResult workloadStats;    ///< Task workload stats

    static std::string header()
    {
        return "id,act_min,act_max,act_mean,act_stddev,act_nsamples,"
               "period_min,period_max,period_mean,period_stddev,period_"
               "nsamples,"
               "workload_min,workload_max,workload_mean,workload_stddev,"
               "workload_"
               "nsamples\n";
    }

    void print(std::ostream& os) const
    {
        os << (int)id << "," << activationStats.minValue << ","
           << activationStats.maxValue << "," << activationStats.mean << ","
           << activationStats.stdev << "," << activationStats.nSamples << ","
           << periodStats.minValue << "," << periodStats.maxValue << ","
           << periodStats.mean << "," << periodStats.stdev << ","
           << periodStats.nSamples << "," << workloadStats.minValue << ","
           << workloadStats.maxValue << "," << workloadStats.mean << ","
           << workloadStats.stdev << "," << workloadStats.nSamples << "\n";
    }
};

}  // namespace Boardcore
