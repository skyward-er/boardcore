/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#include "../AnalogPressureSensor.h"

namespace Boardcore
{

/**
 * @brief Driver for Honeywell's pressure sensors (absolute and differential)
 *
 * All this sensors shares the same transfer function which varies only by few
 * parameters: voltage supply and the sensor pressure range
 */
template <typename HoneywellPressureData>
class HoneywellPressureSensor
    : public AnalogPressureSensor<HoneywellPressureData>
{
public:
    using AnalogPressureSensor<HoneywellPressureData>::AnalogPressureSensor;

protected:
    ///< Common transfer function from volts to pascals (from datasheet pag 11)
    inline float voltageToPressure(float voltage)
    {
        float tmp;

        tmp = voltage -
              0.1 * AnalogPressureSensor<HoneywellPressureData>::supplyVoltage;
        tmp *= AnalogPressureSensor<HoneywellPressureData>::maxPressure -
               AnalogPressureSensor<HoneywellPressureData>::minPressure;
        tmp /= 0.8 * AnalogPressureSensor<HoneywellPressureData>::supplyVoltage;
        tmp += AnalogPressureSensor<HoneywellPressureData>::minPressure;

        return tmp;
    }
};

}  // namespace Boardcore
