/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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

struct LSM6DSRXData : public AccelerometerData, public GyroscopeData
{
    LSM6DSRXData()
        : AccelerometerData{0, MeterPerSecondSquared(0),
                            MeterPerSecondSquared(0), MeterPerSecondSquared(0)},
          GyroscopeData{0, 0.0, 0.0, 0.0}
    {
    }

    static std::string header()
    {
        return "accelerationTimestamp,accelerationX,accelerationY,"
               "accelerationZ,angularSpeedTimestamp,angularSpeedX,"
               "angularSpeedY,angularSpeedZ\n";
    }

    void print(std::ostream& os) const
    {
        os << accelerationTimestamp << "," << accelerationX.value() << ","
           << accelerationY.value() << "," << accelerationZ.value() << ","
           << angularSpeedTimestamp << "," << angularSpeedX << ","
           << angularSpeedY << "," << angularSpeedZ << "\n";
    }
};

struct LSM6DSRXTemperature : public TemperatureData
{
    static std::string header() { return "temperatureTimestamp,temperature\n"; }

    void print(std::ostream& os) const
    {
        os << temperatureTimestamp << "," << temperature << "\n";
    }
};

}  // namespace Boardcore
