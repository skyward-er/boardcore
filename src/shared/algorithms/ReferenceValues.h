/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli, Luca Conterio, Alberto Nidasio
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

#include <utils/Constants.h>

#include <ostream>

namespace Boardcore
{

/**
 * @brief Reference values for the Apogee Detection Algorithm.
 */
struct ReferenceValues
{
    // Launch site parameters
    float altitude;
    float pressure;
    float temperature;

    // Pressure and temperature at mean sea level for altitude calculation
    float mslPressure    = Constants::MSL_PRESSURE;
    float mslTemperature = Constants::MSL_TEMPERATURE;

    ReferenceValues(){};

    ReferenceValues(float altitude, float pressure, float temperature,
                    float mslPressure    = Constants::MSL_PRESSURE,
                    float mslTemperature = Constants::MSL_TEMPERATURE)
        : altitude(altitude), pressure(pressure), temperature(temperature),
          mslPressure(mslPressure), mslTemperature(mslTemperature)
    {
    }

    static std::string header()
    {
        return "altitude,pressure,temperature,mslPressure,mslTemperature\n";
    }

    void print(std::ostream& os) const
    {
        os << altitude << "," << pressure << "," << temperature << ","
           << mslPressure << "," << mslTemperature << "\n";
    }

    bool operator==(const ReferenceValues& other) const
    {
        return altitude == other.altitude && pressure == other.pressure &&
               temperature == other.temperature &&
               mslPressure == other.mslPressure &&
               mslTemperature == other.mslTemperature;
    }

    bool operator!=(const ReferenceValues& other) const
    {
        return !(*this == other);
    }
};

}  // namespace Boardcore
