/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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
 * @brief Sensor class for a two point calibrated load cell.
 */
class TwoPointAnalogLoadCell : public Sensor<LoadCellData>
{
public:
    /**
     * @brief Construct a TwoPointAnalogLoadCell.
     *
     * @param getVoltage lambda to retrieve current voltage.
     * @param p0Voltage voltage of point 0.
     * @param p0Mass mass of point 0.
     * @param p1Voltage voltage of point 1.
     * @param p1Mass mass of point 1.
     */
    TwoPointAnalogLoadCell(std::function<ADCData()> getVoltage, float p0Voltage,
                           float p0Mass, float p1Voltage, float p1Mass)
        : getVoltage{getVoltage},
          staticScale{(p1Mass - p0Mass) / (p1Voltage - p0Voltage)},
          staticOffset{p0Mass - staticScale * p0Voltage}
    {
    }

    TwoPointAnalogLoadCell(TwoPointAnalogLoadCell&& other)
        : getVoltage{std::move(other.getVoltage)}, offsetMutex{},
          dynamicOffset{other.dynamicOffset}, staticScale{other.staticScale},
          staticOffset{other.staticOffset}
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }

    /**
     * @brief Set the current load cell offset. Ignores any previous offsets.
     */
    void setOffset(float value)
    {
        miosix::Lock<miosix::FastMutex> lock{offsetMutex};
        dynamicOffset = value;
    }

    /**
     * @brief Update the current load cell offset. Adds the new value to the old
     * offset.
     */
    void updateOffset(float value)
    {
        miosix::Lock<miosix::FastMutex> lock{offsetMutex};
        dynamicOffset += value;
    }

    /**
     * @brief Retrieve the current offset.
     */
    float getOffset()
    {
        miosix::Lock<miosix::FastMutex> lock{offsetMutex};
        return dynamicOffset;
    }

protected:
    LoadCellData sampleImpl() override
    {
        auto voltage = getVoltage();
        auto mass    = -(voltage.voltage * staticScale + staticOffset);

        miosix::Lock<miosix::FastMutex> lock{offsetMutex};
        return {voltage.voltageTimestamp, mass - dynamicOffset};
    }

private:
    std::function<ADCData()> getVoltage;

    // std::atomic<float> does not support +=
    miosix::FastMutex offsetMutex;
    float dynamicOffset = 0.0f;

    const float staticScale;
    const float staticOffset;
};

}  // namespace Boardcore
