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

#pragma once

#include <utils/Stats/Stats.h>

#include <reflect.hpp>

namespace Boardcore
{

struct CpuMeterData
{
    uint64_t timestamp = 0;
    float minValue     = 0;  ///< Min value found so far.
    float maxValue     = 0;  ///< Max value found so far.
    float mean         = 0;  ///< Mean of dataset.
    float stdDev       = 0;  ///< Standard deviation of dataset.
    uint32_t nSamples  = 0;  ///< Number of samples.

    uint32_t minFreeHeap  = 0;
    uint32_t freeHeap     = 0;
    uint32_t minFreeStack = 0;
    uint32_t freeStack    = 0;

    CpuMeterData() {}

    explicit CpuMeterData(uint64_t timestamp, StatsResult stats,
                          uint32_t freeHeap, uint32_t minFreeHeap,
                          uint32_t minFreeStack, uint32_t freeStack)
        : timestamp(timestamp), minValue(stats.minValue),
          maxValue(stats.maxValue), mean(stats.mean), stdDev(stats.stdDev),
          nSamples(stats.nSamples), minFreeHeap(minFreeHeap),
          freeHeap(freeHeap), minFreeStack(minFreeStack), freeStack(freeStack)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            CpuMeterData,
            FIELD_DEF(timestamp) FIELD_DEF(minValue) FIELD_DEF(maxValue)
                FIELD_DEF(mean) FIELD_DEF(stdDev) FIELD_DEF(nSamples)
                    FIELD_DEF(minFreeHeap) FIELD_DEF(freeHeap)
                        FIELD_DEF(minFreeStack) FIELD_DEF(freeStack));
    }
};

}  // namespace Boardcore
