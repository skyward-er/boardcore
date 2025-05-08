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

#include <logger/Logger.h>
#include <utils/Stats/Stats.h>

#include <chrono>
#include <cstdint>
#include <ostream>
#include <reflect.hpp>

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
    size_t id;
    int64_t period;  //< this was a std::chrono::nanoseconds
    StatsResult activationStats;
    StatsResult periodStats;
    StatsResult workloadStats;
    uint32_t missedEvents;
    uint32_t failedEvents;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            TaskStatsResult,
            FIELD_DEF(id) FIELD_DEF(period) FIELD_DEF2(
                activationStats, minValue) FIELD_DEF2(activationStats, maxValue)
                FIELD_DEF2(activationStats, mean) FIELD_DEF2(activationStats,
                                                             stdDev)
                    FIELD_DEF2(activationStats, nSamples) FIELD_DEF2(
                        periodStats, minValue) FIELD_DEF2(periodStats, maxValue)
                        FIELD_DEF2(periodStats, mean)
                            FIELD_DEF2(periodStats, stdDev) FIELD_DEF2(
                                periodStats, nSamples)
                                FIELD_DEF2(workloadStats, minValue) FIELD_DEF2(
                                    workloadStats, maxValue)
                                    FIELD_DEF2(workloadStats, mean)
                                        FIELD_DEF2(workloadStats, stdDev)
                                            FIELD_DEF2(workloadStats, nSamples)
                                                FIELD_DEF(missedEvents)
                                                    FIELD_DEF(failedEvents));
    }
};

}  // namespace Boardcore
