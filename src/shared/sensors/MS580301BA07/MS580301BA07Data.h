/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "sensors/SensorData.h"

struct MS5803Data : public PressureData, TemperatureData
{
    uint32_t raw_press;
    uint32_t raw_temp;

    MS5803Data() : PressureData{0, 0.0}, TemperatureData{0, 0.0} {}

    MS5803Data(uint64_t t, uint32_t raw_press, float press, uint32_t raw_temp,
               float temp)
        : PressureData{t, press}, TemperatureData{t, temp},
          raw_press(raw_press), raw_temp(raw_temp)
    {
    }

    static std::string header()
    {
        return "press_timestamp,raw_press,press,temp_timestamp,raw_temp,"
               "temp\n";
    }

    void print(std::ostream& os) const
    {
        os << press_timestamp << "," << raw_press << "," << press << ","
           << temp_timestamp << "," << raw_temp << "," << temp << "\n";
    }
};
