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

#include "ADS131M08Defs.h"

namespace Boardcore
{

struct ADS131M08Data
{
    uint64_t timestamp;
    float voltage[8];

    ADS131M08Data() : ADS131M08Data{0, 0, 0, 0, 0, 0, 0, 0, 0} {}

    ADS131M08Data(uint64_t timestamp, float voltageCh1, float voltageCh2,
                  float voltageCh3, float voltageCh4, float voltageCh5,
                  float voltageCh6, float voltageCh7, float voltageCh8)
        : timestamp(timestamp)
    {
        voltage[0] = voltageCh1;
        voltage[1] = voltageCh2;
        voltage[2] = voltageCh3;
        voltage[3] = voltageCh4;
        voltage[4] = voltageCh5;
        voltage[5] = voltageCh6;
        voltage[6] = voltageCh7;
        voltage[7] = voltageCh8;
    }

    static std::string header()
    {
        return "timestamp,voltage_channel_1,voltage_channel_2,voltage_channel_"
               "3,voltage_channel_4,voltage_channel_5,voltage_channel_6,"
               "voltage_channel_7,voltage_channel_8\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << voltage[0] << "," << voltage[1] << ","
           << voltage[2] << "," << voltage[3] << "," << voltage[4] << ","
           << voltage[5] << "," << voltage[6] << "," << voltage[7] << "\n";
    }

    const ADCData getVoltage(ADS131M08Defs::Channel channel)
    {
        return {timestamp, static_cast<uint8_t>(channel),
                voltage[static_cast<uint8_t>(channel)]};
    }
};

}  // namespace Boardcore
