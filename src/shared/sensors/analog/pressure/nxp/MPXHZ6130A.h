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

#include <sensors/analog/pressure/AnalogPressureSensor.h>
#include <utils/Stats/Stats.h>

#include "MPXHZ6130AData.h"

namespace Boardcore
{

/**
 * @brief Driver for NXP's MPXHZ6130A pressure sensor
 */
class MPXHZ6130A final : public AnalogPressureSensor<MPXHZ6130AData>
{
public:
    MPXHZ6130A(std::function<ADCData()> getVoltage,
               const float supplyVoltage = 5.0)
        : AnalogPressureSensor(getVoltage, supplyVoltage, 130000, 15000)
    {
    }

private:
    float voltageToPressure(float voltage) override
    {
        return (((voltage / supplyVoltage) + CONST_B) / CONST_A) * 1000.0;
    }

    // Constants from datasheet
    static constexpr float CONST_A = 0.007826;
    static constexpr float CONST_B = 0.07739;
};

}  // namespace Boardcore
