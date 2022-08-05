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

#include <sensors/Sensor.h>

#include <functional>

namespace Boardcore
{

/**
 * @brief Common class for all analog pressure sensors.
 *
 * All analog pressure sensors have a transfer function to convert the read
 * voltage into pressure and a range within which they operate.
 */
template <typename AnalogPressureData>
class AnalogPressureSensor : public Sensor<AnalogPressureData>
{
public:
    AnalogPressureSensor(std::function<ADCData()> getVoltage,
                         const float supplyVoltage = 5.0,
                         const float maxPressure   = 0,
                         const float minPressure   = 0)
        : getVoltage(getVoltage), supplyVoltage(supplyVoltage),
          maxPressure(maxPressure), minPressure(minPressure)
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }

    /**
     * @brief Sets the offset that will be removed from the measured pressure.
     */
    void setOffset(const float offset) { this->offset = offset; }

    AnalogPressureData sampleImpl() override
    {
        AnalogPressureData pressure;

        // Retrieve the voltage
        ADCData voltage = getVoltage();

        // Save the timestamp and convert the voltage
        pressure.pressureTimestamp = voltage.voltageTimestamp;
        pressure.pressure          = voltageToPressure(voltage.voltage);

        // Check if the pressure is in range
        if (pressure.pressure < minPressure)
            pressure.pressure = minPressure;
        else if (pressure.pressure > maxPressure)
            pressure.pressure = maxPressure;

        pressure.pressure -= offset;

        return pressure;
    }

protected:
    ///< Conversion function from volts to pascals.
    virtual float voltageToPressure(float voltage) = 0;

    ///< Function that returns the sensor voltage.
    std::function<ADCData()> getVoltage;

    const float supplyVoltage;
    const float maxPressure;
    const float minPressure;
    float offset = 0;
};

}  // namespace Boardcore
