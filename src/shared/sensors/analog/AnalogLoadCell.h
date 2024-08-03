/* Copyright (c) 2022 Skyward Experimental Rocketry
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

#include <sensors/Sensor.h>

#include <functional>

#include "AnalogLoadCellData.h"

namespace Boardcore
{

class AnalogLoadCell : public Sensor<AnalogLoadCellData>
{
public:
    AnalogLoadCell(std::function<ADCData()> getVoltage, const float mVtoV,
                   const unsigned int fullScale, const float supplyVoltage = 5)
        : getVoltage(getVoltage),
          conversionCoeff(mVtoV * supplyVoltage / fullScale / 1e3)
    {
    }

    bool init() override { return true; };

    bool selfTest() override { return true; };

    /**
     * @brief Sets the offset that will be removed from the measured load.
     */
    void setOffset(const float offset) { this->offset = offset; }

    void updateOffset(float offset) { this->offset += offset; }

protected:
    AnalogLoadCellData sampleImpl() override
    {
        ADCData adcData = getVoltage();

        AnalogLoadCellData data;
        data.loadTimestamp = adcData.voltageTimestamp;
        data.voltage       = adcData.voltage;

        if (data.voltage != 0)
            data.load = data.voltage / conversionCoeff - offset;
        else
            data.load = -offset;

        return data;
    }

private:
    ///< Function that returns the sensor voltage.
    std::function<ADCData()> getVoltage;

    const float conversionCoeff;
    float offset = 0;
};

}  // namespace Boardcore
