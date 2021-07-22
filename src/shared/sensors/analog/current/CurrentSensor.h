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

#include "CurrentSensorData.h"

/**
 * @brief Common class for current sense sensors
 *
 * It needs a transfer function to convert the read voltage into current.
 */
class CurrentSensor : public Sensor<CurrentSensorData>
{
public:
    static constexpr int MOVING_AVAERAGE_N = 20;

    CurrentSensor(std::function<ADCData()> getADCVoltage_,
                  std::function<float(float)> adcToCurrent_)
        : getADCVoltage(getADCVoltage_), adcToCurrent(adcToCurrent_)
    {
        last_sample.current = 0;
    }

    bool init() override { return true; };

    bool selfTest() override { return true; };

    ///< Converts the voltage value to pressure
    CurrentSensorData sampleImpl() override
    {
        ADCData adc_data = getADCVoltage();

        if (last_sample.current == 0)
        {
            last_sample.current = adcToCurrent(adc_data.voltage);
        }

        CurrentSensorData current_data;
        current_data.adc_timestamp = adc_data.adc_timestamp;
        current_data.channel_id    = adc_data.channel_id;
        current_data.voltage       = adc_data.voltage;

        // Moving average
        current_data.current = last_sample.current * MOVING_AVAERAGE_COMP_COEFF;
        current_data.current +=
            adcToCurrent(adc_data.voltage) * MOVING_AVAERAGE_COEFF;

        return current_data;
    };

private:
    ///< Function that returns the adc voltage
    std::function<ADCData()> getADCVoltage;

    ///< Function that converts adc voltage to current
    std::function<float(float)> adcToCurrent;

    static constexpr float MOVING_AVAERAGE_COEFF = 1 / (float)MOVING_AVAERAGE_N;
    static constexpr float MOVING_AVAERAGE_COMP_COEFF =
        1 - MOVING_AVAERAGE_COEFF;
};