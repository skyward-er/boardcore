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
    std::chrono::nanoseconds period;
    StatsResult activationStats;
    StatsResult periodStats;
    StatsResult workloadStats;
    uint32_t missedEvents;
    uint32_t failedEvents;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            TaskStatsResult,
            FIELD_DEF(id) FIELD_DEF(period) FIELD_DEF(activationStats)
                FIELD_DEF(periodStats) FIELD_DEF(workloadStats)
                    FIELD_DEF(missedEvents) FIELD_DEF(failedEvents));
    }
};

template <>
struct Mapping<TaskStatsResult>
{
    static std::string getMappingString(const TaskStatsResult& value)
    {
        std::string mappingString;

        ADD_MAPPING_STRING("TaskStatsResult"), ADD_MAPPING_STRING("7");
        ADD_MAPPING_STRING("id"), ADD_MAPPING_STRING("t");
        ADD_MAPPING_STRING("period"), ADD_MAPPING_STRING("l");

        ADD_MAPPING_STRING("activationStats.minValue"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("activationStats.maxValue"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("activationStats.mean"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("activationStats.stdDev"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("activationStats.nSamples"), ADD_MAPPING_STRING("j");

        ADD_MAPPING_STRING("periodStats.minValue"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("periodStats.maxValue"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("periodStats.mean"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("periodStats.stdDev"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("periodStats.nSamples"), ADD_MAPPING_STRING("j");

        ADD_MAPPING_STRING("workloadStats.minValue"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("workloadStats.maxValue"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("workloadStats.mean"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("workloadStats.stdDev"), ADD_MAPPING_STRING("f");
        ADD_MAPPING_STRING("workloadStats.nSamples"), ADD_MAPPING_STRING("j");

        ADD_MAPPING_STRING("missedEvents"), ADD_MAPPING_STRING("j");
        ADD_MAPPING_STRING("failedEvents"), ADD_MAPPING_STRING("j");

        return mappingString;
    }
};

}  // namespace Boardcore
