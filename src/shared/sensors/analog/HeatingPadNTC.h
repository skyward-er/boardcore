/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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
#include <sensors/SensorData.h>

#include <functional>

namespace Boardcore
{

/**
 * @brief Sensor class for the Heating Pad NTC.
 */
class HeatingPadNTC : public Sensor<TemperatureData>
{
public:
    /**
     * @brief Construct a HeatingPadNTC.
     *
     * @param getVoltage lambda to retrieve current voltage.
     * @param refVoltage voltage at reference temperature.
     * @param refResistance NTC reference resistance.
     * @param refTemperature reference temperature.
     * @param beta beta parameter for the NTC thermistor.
     * 
     * 
     */
    HeatingPadNTC(std::function<VoltageData()> getVoltage,
                   float refVoltage, float refResistance, float refTemperature, float beta)
        : getVoltage{std::move(getVoltage)}, refVoltage{refVoltage}, refResistance{refResistance},
          refTemperature{refTemperature}, beta{beta}
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }


protected:
    TemperatureData sampleImpl() override
    {
        auto voltage = getVoltage();
        return {voltage.voltageTimestamp, voltageToTemperature(voltage.voltage)};
    }

private:
    float voltageToTemperature(float voltage)
    {
        float resistance = (refResistance * voltage) / (refVoltage - voltage);

    float temperature = 1.0f / ((1.0f / refTemperature) + (1.0f / beta) * std::log(resistance / refResistance));

        return temperature;
    }
                                      
    std::function<VoltageData()> getVoltage;

    float refVoltage;
    const float refResistance;
    const float refTemperature;
    const float beta;
};

}  // namespace Boardcore
