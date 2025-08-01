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

#include <reflect.hpp>

namespace Boardcore
{

struct BMP280Data : public TemperatureData, public PressureData
{
    BMP280Data() : TemperatureData{0, 0.0}, PressureData{0, 0.0} {}

    BMP280Data(uint64_t timestamp, float temperature, float pressure,
               float humidity)
        : TemperatureData{timestamp, temperature},
          PressureData{timestamp, pressure}

    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(BMP280Data,
                          EXTEND_DEF(TemperatureData) EXTEND_DEF(PressureData));
    }
};

}  // namespace Boardcore
