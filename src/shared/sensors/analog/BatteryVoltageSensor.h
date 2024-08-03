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

#include "BatteryVoltageSensorData.h"

namespace Boardcore
{

/**
 * @brief Common class for battery voltage sensors.
 *
 * It needs the conversion ratio, in order to convert the raw voltage to
 * battery voltage.
 */
class BatteryVoltageSensor : public Sensor<BatteryVoltageSensorData>
{
public:
    static constexpr int MOVING_AVERAGE_N = 20;

    BatteryVoltageSensor(std::function<ADCData()> getVoltage,
                         float conversionCoeff)
        : getVoltage(getVoltage), conversionCoeff(conversionCoeff)
    {
        lastSample.batVoltage = 0;
    }

    bool init() override { return true; };

    bool selfTest() override { return true; };

protected:
    ///< Converts the adc voltage value to battery voltage
    BatteryVoltageSensorData sampleImpl() override
    {
        ADCData adcData = getVoltage();

        if (lastSample.batVoltage == 0)
            lastSample.batVoltage = adcToBatteryVoltage(adcData.voltage);

        BatteryVoltageSensorData batData;
        batData.voltageTimestamp = adcData.voltageTimestamp;
        batData.channelId        = adcData.channelId;
        batData.voltage          = adcData.voltage;

        // Moving average
        batData.batVoltage = lastSample.batVoltage * MOVING_AVERAGE_COMP_COEFF;
        batData.batVoltage +=
            adcToBatteryVoltage(adcData.voltage) * MOVING_AVERAGE_COEFF;

        return batData;
    }

private:
    ///< Conversion function from adc volts to battery volts
    float adcToBatteryVoltage(float voltage)
    {
        return voltage * conversionCoeff;
    }

    ///< Function that returns the adc voltage
    std::function<ADCData()> getVoltage;

    float conversionCoeff;

    static constexpr float MOVING_AVERAGE_COEFF = 1 / (float)MOVING_AVERAGE_N;
    static constexpr float MOVING_AVERAGE_COMP_COEFF = 1 - MOVING_AVERAGE_COEFF;
};

}  // namespace Boardcore
