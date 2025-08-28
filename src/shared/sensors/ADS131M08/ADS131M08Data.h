/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <sensors/SensorData.h>
#include <stdint.h>

#include <ostream>
#include <reflect.hpp>

#include "ADS131M08Defs.h"

namespace Boardcore
{

struct ADS131M08Data
{
    uint64_t timestamp                         = 0;
    float voltage[ADS131M08Defs::CHANNELS_NUM] = {};

    ADS131M08Data() = default;

    ADS131M08Data(uint64_t timestamp, float voltageCh1, float voltageCh2,
                  float voltageCh3, float voltageCh4, float voltageCh5,
                  float voltageCh6, float voltageCh7, float voltageCh8)
        : timestamp(timestamp),
          voltage{voltageCh1, voltageCh2, voltageCh3, voltageCh4,
                  voltageCh5, voltageCh6, voltageCh7, voltageCh8}
    {
    }

    const ADCData getVoltage(ADS131M08Defs::Channel channel)
    {
        return {timestamp, static_cast<uint8_t>(channel),
                voltage[static_cast<uint8_t>(channel)]};
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADS131M08Data,
                          FIELD_DEF(timestamp) FIELD_DEF(voltage));
    }
};

}  // namespace Boardcore
