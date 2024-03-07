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

struct BME280Data : public TemperatureData,
                    public PressureData,
                    public HumidityData
{
    BME280Data()
        : TemperatureData{0, 0.0}, PressureData{0, Pascal(0)},
          HumidityData{0, 0.0}
    {
    }

    BME280Data(uint64_t timestamp, float temperature, Pascal pressure,
               float humidity)
        : TemperatureData{timestamp, temperature},
          PressureData{timestamp, pressure}, HumidityData{timestamp, humidity}

    {
    }

    static std::string header()
    {
        return "temperatureTimestamp,temperature,pressureTimestamp,pressure,"
               "humid_timestamp,humidity\n";
    }

    void print(std::ostream& os) const
    {
        os << temperatureTimestamp << "," << temperature << ","
           << pressureTimestamp << "," << pressure << "," << humidityTimestamp
           << "," << humidity << "\n";
    }
};

}  // namespace Boardcore
