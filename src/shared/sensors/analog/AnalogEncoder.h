/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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
 * @brief Sensor class for analog encoder
 * .
 */
class AnalogEncoder : public Sensor<ServoPositionData>
{
public:
    /**
     * @brief Construct a AnalogEncoder.
     *
     * @param getVoltage lambda to retrieve current voltage.
     * @param shuntResistance shunt resistance value.
     * @param minCurrent current at the 0 pressure point.
     * @param maxCurrent current at the maximum pressure point.
     */
    AnalogEncoder(std::function<VoltageData()> getVoltage,
                  float shuntResistance, float fullscaleVoltage,
                  int sensorResistance, int currentGain, float maxAngle)
        : getVoltage{std::move(getVoltage)}, shuntResistance{shuntResistance},
          fullscaleVoltage{fullscaleVoltage},
          sensorResistance{sensorResistance}, currentGain{currentGain},
          maxAngle{maxAngle}
    {
    }

    bool init() override { return true; }

    bool calibrate()
    {
        offset            = 0.0f;
        float rawPosition = sampleImpl().position;
        offset            = rawPosition;

        return true;
    }

    bool selfTest() override { return true; }

    float getShuntResistance() const { return shuntResistance; }

protected:
    ServoPositionData sampleImpl() override
    {
        auto voltage = getVoltage();
        return {voltage.voltageTimestamp, voltageToPosition(voltage.voltage)};
    }

private:
    float voltageToPosition(float voltage)
    {
        float rawPosition = (voltage * sensorResistance * maxAngle) /
                            (shuntResistance * currentGain * fullscaleVoltage);

        // Apply the offset to the raw value
        float offsetPosition = rawPosition - offset;

        /* if (offsetPosition <= 0.0f)
            return std::abs(360.0f - offsetPosition); */

        return std::abs(offsetPosition);
    }

    std::function<VoltageData()> getVoltage;

    const float shuntResistance;
    const float fullscaleVoltage;
    const int sensorResistance;
    const int currentGain;
    const float maxAngle;
    float offset = 0.0f;
};

}  // namespace Boardcore
