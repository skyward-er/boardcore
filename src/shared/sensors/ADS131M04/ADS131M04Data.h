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

#include "ADS131M04Defs.h"

namespace Boardcore
{

struct ADS131M04Data
{
    uint64_t timestamp;
    float voltage[4];

    ADS131M04Data() : ADS131M04Data{0, 0, 0, 0, 0} {}

    ADS131M04Data(uint64_t timestamp, float voltageCh1, float voltageCh2,
                  float voltageCh3, float voltageCh4)
        : timestamp(timestamp)
    {
        voltage[0] = voltageCh1;
        voltage[1] = voltageCh2;
        voltage[2] = voltageCh3;
        voltage[3] = voltageCh4;
    }

    static std::string header()
    {
        return "timestamp,voltage_channel_1,voltage_channel_2,voltage_channel_"
               "3,voltage_channel_4\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << voltage[0] << "," << voltage[1] << ","
           << voltage[2] << "," << voltage[3] << "\n";
    }

    const ADCData getVoltage(ADS131M04Defs::Channel channel)
    {
        return {timestamp, static_cast<uint8_t>(channel),
                voltage[static_cast<uint8_t>(channel)]};
    }
};

}  // namespace Boardcore
