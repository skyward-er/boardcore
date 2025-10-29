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
                  float shuntResistance, float minCurrent = 4,
                  float maxCurrent = 20)
        : getVoltage{std::move(getVoltage)}, shuntResistance{shuntResistance},
          minCurrent{minCurrent}, maxCurrent{maxCurrent}
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }

    void setShuntResistance(float resistance) { shuntResistance = resistance; }

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
        // TODO: implement actual conversion
        return voltage;
    }

    std::function<VoltageData()> getVoltage;

    float shuntResistance;
    const float minCurrent;
    const float maxCurrent;
};

}  // namespace Boardcore
