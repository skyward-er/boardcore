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

namespace Boardcore
{

struct InternalADCData
{
    uint64_t timestamp = 0;
    float voltage[16];
    float temperature;
    float vBat;

    InternalADCData() {}

    static std::string header()
    {
        return "timestamp,voltage_0,voltage_1,voltage_2,voltage_3,voltage_4,"
               "voltage_5,voltage_6,voltage_7,voltage_8,voltage_9,voltage_10,"
               "voltage_11,voltage_12,voltage_13,voltage_14,voltage_15,"
               "temperature,vBat,voltage\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << ",";

        for (int i = 0; i < 16; i++)
        {
            os << voltage[i] << ",";
        }

        os << temperature << "," << vBat << "\n";
    }
};

}  // namespace Boardcore
