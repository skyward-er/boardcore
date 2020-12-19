/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <sensors/Sensor.h>

#include <functional>

/**
 * @brief Common class for all analog pressure sensors
 *
 * All analog pressure sensors have a transfer function to convert the read
 * voltage into pressure and a range within which they operate.
 */
class AnalogPressureSensor : public Sensor<PressureData>
{
public:
    AnalogPressureSensor(std::function<ADCData()> getSensorVoltage_);

    AnalogPressureSensor(std::function<ADCData()> getSensorVoltage_,
                         const float V_SUPPLY_);

    ///< Converts the voltage value to pressure
    PressureData sampleImpl() override;

    bool selfTest() override { return 0; };

protected:
    ///< Transfer function from volts to pascals (from datasheet pag 11)
    virtual float voltageToPressure(float voltage) = 0;

    const float maxPressure = 0;

    const float minPressure = 0;

    ///< Function that returns the sensor voltage
    std::function<ADCData()> getSensorVoltage;

    const float V_SUPPLY = 5.0;  ///< Suppply voltage
};