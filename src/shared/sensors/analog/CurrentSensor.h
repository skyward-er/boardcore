/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

namespace Boardcore
{

/**
 * @brief Common class for current sensors.
 *
 * It needs a transfer function to convert the read voltage into current.
 */
class CurrentSensor : public Sensor<CurrentData>
{
public:
    static constexpr int MOVING_AVERAGE_N = 20;

    CurrentSensor(std::function<ADCData()> getVoltage,
                  std::function<float(float)> voltageToCurrent)
        : getVoltage(getVoltage), voltageToCurrent(voltageToCurrent)
    {
        lastSample.current = 0;
    }

    bool init() override { return true; };

    bool selfTest() override { return true; };

    ///< Converts the voltage value to pressure
    CurrentData sampleImpl() override
    {
        ADCData adc_data = getVoltage();

        if (lastSample.current == 0)
        {
            lastSample.current = voltageToCurrent(adc_data.voltage);
        }

        CurrentData current_data;
        current_data.currentTimestamp = adc_data.voltageTimestamp;

        // Moving average
        current_data.current = lastSample.current * MOVING_AVERAGE_COMP_COEFF;
        current_data.current +=
            voltageToCurrent(adc_data.voltage) * MOVING_AVERAGE_COEFF;

        return current_data;
    };

private:
    ///< Function that returns the adc voltage
    std::function<ADCData()> getVoltage;

    ///< Function that converts adc voltage to current
    std::function<float(float)> voltageToCurrent;

    static constexpr float MOVING_AVERAGE_COEFF = 1 / (float)MOVING_AVERAGE_N;
    static constexpr float MOVING_AVERAGE_COMP_COEFF = 1 - MOVING_AVERAGE_COEFF;
};

}  // namespace Boardcore
